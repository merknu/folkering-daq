//! xHCI (USB 3.0) Host Controller Driver for RP1
//!
//! RP1 contains two independent xHCI controllers at:
//!   xHCI 0: RP1 BAR1 + 0x20_0000 (0x1F_0020_0000)
//!   xHCI 1: RP1 BAR1 + 0x30_0000 (0x1F_0030_0000)
//!
//! xHCI uses ring-based DMA architecture:
//! - Command Ring: host → controller (admin commands)
//! - Event Ring: controller → host (completion notifications)
//! - Transfer Rings: per-endpoint data transfers (TRBs)
//!
//! Rings use Cycle Bit for producer/consumer synchronization.
//! Link TRB at end of each ring wraps back to start.

use alloc::vec::Vec;
use core::sync::atomic::{AtomicU64, AtomicBool, Ordering};

static XHCI_BASE: AtomicU64 = AtomicU64::new(0);
static XHCI_READY: AtomicBool = AtomicBool::new(false);

// === Capability Register Offsets (from xHCI base) ===
const CAPLENGTH: u64 = 0x00;
const HCIVERSION: u64 = 0x02;
const HCSPARAMS1: u64 = 0x04;
const HCSPARAMS2: u64 = 0x08;
const HCCPARAMS1: u64 = 0x10;
const DBOFF: u64 = 0x14;
const RTSOFF: u64 = 0x18;

// === Operational Register Offsets (from base + CAPLENGTH) ===
const USBCMD: u64 = 0x00;
const USBSTS: u64 = 0x04;
const PAGESIZE: u64 = 0x08;
const DNCTRL: u64 = 0x14;
const CRCR: u64 = 0x18;
const DCBAAP: u64 = 0x30;
const CONFIG: u64 = 0x38;

// Port Register Set starts at op_base + 0x400
const PORTSC_BASE: u64 = 0x400;

// USBCMD bits
const CMD_RUN: u32 = 1 << 0;
const CMD_HCRST: u32 = 1 << 1;
const CMD_INTE: u32 = 1 << 2;

// USBSTS bits
const STS_HCH: u32 = 1 << 0;

// === Transfer Request Block (TRB) — 16 bytes, DMA-aligned ===
/// xHCI TRB structure. Must be exactly 16 bytes and 16-byte aligned.
/// Hardware reads/writes these directly via DMA.
#[repr(C, align(16))]
#[derive(Clone, Copy)]
pub struct Trb {
    /// Buffer pointer (data/link address) or inline command parameters
    pub parameter: u64,
    /// Transfer length / completion code
    pub status: u32,
    /// TRB Type (bits 15:10), Cycle Bit (bit 0), IOC (bit 5), Chain (bit 4), etc.
    pub control: u32,
}

impl Trb {
    pub const fn new() -> Self {
        Self { parameter: 0, status: 0, control: 0 }
    }

    #[inline(always)]
    pub fn cycle_bit(&self) -> bool {
        (self.control & 1) != 0
    }

    #[inline(always)]
    pub fn toggle_cycle_bit(&mut self) {
        self.control ^= 1;
    }

    #[inline(always)]
    pub fn trb_type(&self) -> u8 {
        ((self.control >> 10) & 0x3F) as u8
    }

    #[inline(always)]
    pub fn set_trb_type(&mut self, t: u8) {
        self.control &= !(0x3F << 10);
        self.control |= ((t as u32) & 0x3F) << 10;
    }

    #[inline(always)]
    pub fn set_ioc(&mut self) {
        self.control |= 1 << 5;
    }

    #[inline(always)]
    pub fn completion_code(&self) -> u8 {
        (self.status >> 24) as u8
    }
}

// TRB Type codes
pub const TRB_NORMAL: u8 = 1;
pub const TRB_SETUP_STAGE: u8 = 2;
pub const TRB_DATA_STAGE: u8 = 3;
pub const TRB_STATUS_STAGE: u8 = 4;
pub const TRB_LINK: u8 = 6;
pub const TRB_ENABLE_SLOT: u8 = 9;
pub const TRB_ADDRESS_DEVICE: u8 = 11;
pub const TRB_CONFIG_ENDPOINT: u8 = 12;
pub const TRB_TRANSFER_EVENT: u8 = 32;
pub const TRB_CMD_COMPLETION: u8 = 33;
pub const TRB_PORT_STATUS_CHANGE: u8 = 34;

// === Ring (circular TRB buffer) ===
const RING_SIZE: usize = 256;

#[repr(C, align(4096))]
struct TrbRing {
    trbs: [Trb; RING_SIZE],
    enqueue_idx: usize,
    dequeue_idx: usize,
    cycle_bit: bool,
}

impl TrbRing {
    const fn new() -> Self {
        Self {
            trbs: [Trb::new(); RING_SIZE],
            enqueue_idx: 0,
            dequeue_idx: 0,
            cycle_bit: true,
        }
    }

    /// Enqueue a TRB. Sets cycle bit, advances index, handles Link TRB wrap.
    fn enqueue(&mut self, mut trb: Trb) -> usize {
        if self.cycle_bit {
            trb.control |= 1;
        } else {
            trb.control &= !1;
        }

        let idx = self.enqueue_idx;
        self.trbs[idx] = trb;
        self.enqueue_idx += 1;

        // Wrap: insert Link TRB pointing back to start
        if self.enqueue_idx >= RING_SIZE - 1 {
            let mut link = Trb::new();
            link.parameter = self.phys_addr();
            link.set_trb_type(TRB_LINK);
            // Toggle Cycle on wrap
            if self.cycle_bit { link.control |= 1; }
            link.control |= 1 << 1; // Toggle Cycle bit
            self.trbs[self.enqueue_idx] = link;
            self.enqueue_idx = 0;
            self.cycle_bit = !self.cycle_bit;
        }

        idx
    }

    fn phys_addr(&self) -> u64 {
        crate::virt_to_phys(&self.trbs[0] as *const Trb as u64)
    }
}

// === Event Ring Segment Table Entry ===
#[repr(C, align(64))]
struct ErstEntry {
    ring_segment_base: u64,
    ring_segment_size: u16,
    _reserved: [u8; 6],
}

// === Device Context Base Address Array ===
const MAX_SLOTS: usize = 32;

#[repr(C, align(4096))]
struct Dcbaa {
    entries: [u64; MAX_SLOTS + 1],
}

// === Controller State ===
static mut CMD_RING: TrbRing = TrbRing::new();
static mut EVT_RING: TrbRing = TrbRing::new();
static mut DCBAA: Dcbaa = Dcbaa { entries: [0; MAX_SLOTS + 1] };

// Per-endpoint transfer rings (for SIRIUS: EP1 OUT, EP1 IN, EP2 IN, EP4 IN, EP6 IN)
static mut TRANSFER_RINGS: [TrbRing; 8] = [
    TrbRing::new(), TrbRing::new(), TrbRing::new(), TrbRing::new(),
    TrbRing::new(), TrbRing::new(), TrbRing::new(), TrbRing::new(),
];

fn op_base() -> u64 {
    let base = XHCI_BASE.load(Ordering::Relaxed);
    let caplength = unsafe { core::ptr::read_volatile(base as *const u8) };
    base + caplength as u64
}

fn rt_base() -> u64 {
    let base = XHCI_BASE.load(Ordering::Relaxed);
    let rtsoff = unsafe { core::ptr::read_volatile((base + RTSOFF) as *const u32) };
    base + (rtsoff & !0x1F) as u64
}

fn db_base() -> u64 {
    let base = XHCI_BASE.load(Ordering::Relaxed);
    let dboff = unsafe { core::ptr::read_volatile((base + DBOFF) as *const u32) };
    base + (dboff & !0x3) as u64
}

/// Ring the doorbell for a slot
fn ring_doorbell(slot: u8, target: u8) {
    let addr = db_base() + (slot as u64) * 4;
    unsafe { core::ptr::write_volatile(addr as *mut u32, target as u32); }
}

/// Read an operational register
fn read_op(offset: u64) -> u32 {
    unsafe { core::ptr::read_volatile((op_base() + offset) as *const u32) }
}

/// Write an operational register
fn write_op(offset: u64, val: u32) {
    unsafe { core::ptr::write_volatile((op_base() + offset) as *mut u32, val); }
}

pub fn init() {
    // Get xHCI base from RP1 discovery
    let rp1 = match super::pci::rp1() {
        Some(rp1) => rp1,
        None => {
            crate::kprintln!("  xHCI: RP1 not found, cannot initialize");
            return;
        }
    };

    // Use xHCI Controller 0 (RP1 BAR1 + 0x20_0000)
    let xhci_base = rp1.xhci0_base;
    XHCI_BASE.store(xhci_base, Ordering::SeqCst);

    crate::kprintln!("  xHCI at RP1 BAR1+{:#x}", super::pci::RP1_XHCI0_OFFSET);

    unsafe {
        let base = xhci_base;

        // Read capabilities
        let hciversion = core::ptr::read_volatile((base + HCIVERSION) as *const u16);
        let hcsparams1 = core::ptr::read_volatile((base + HCSPARAMS1) as *const u32);
        let max_slots = hcsparams1 & 0xFF;
        let max_ports = (hcsparams1 >> 24) & 0xFF;

        crate::kprintln!("  xHCI v{}.{}, {} slots, {} ports",
            hciversion >> 8, hciversion & 0xFF, max_slots, max_ports);

        // Step 1: Halt controller
        let cmd = read_op(USBCMD);
        write_op(USBCMD, cmd & !CMD_RUN);
        while read_op(USBSTS) & STS_HCH == 0 { core::hint::spin_loop(); }

        // Step 2: Reset
        write_op(USBCMD, CMD_HCRST);
        while read_op(USBCMD) & CMD_HCRST != 0 { core::hint::spin_loop(); }
        while read_op(USBSTS) & STS_HCH == 0 { core::hint::spin_loop(); }

        // Step 3: Configure max slots
        let slots = max_slots.min(MAX_SLOTS as u32);
        write_op(CONFIG, slots);

        // Step 4: DCBAAP (Device Context Base Address Array Pointer)
        let dcbaa_phys = crate::virt_to_phys(&DCBAA as *const Dcbaa as u64);
        write_op(DCBAAP, dcbaa_phys as u32);
        write_op(DCBAAP + 4, (dcbaa_phys >> 32) as u32);

        // Step 5: Command Ring
        let crcr_phys = CMD_RING.phys_addr();
        write_op(CRCR, (crcr_phys as u32) | 1); // RCS=1
        write_op(CRCR + 4, (crcr_phys >> 32) as u32);

        // Step 6: Event Ring (Interrupter 0)
        let rt = rt_base();
        let ir0 = rt + 0x20;

        static mut ERST: ErstEntry = ErstEntry {
            ring_segment_base: 0,
            ring_segment_size: RING_SIZE as u16,
            _reserved: [0; 6],
        };
        ERST.ring_segment_base = EVT_RING.phys_addr();

        let erst_phys = crate::virt_to_phys(&ERST as *const _ as u64);

        // ERSTSZ = 1 segment
        core::ptr::write_volatile((ir0 + 0x08) as *mut u32, 1);

        // ERDP (Event Ring Dequeue Pointer)
        let erdp = EVT_RING.phys_addr();
        core::ptr::write_volatile((ir0 + 0x18) as *mut u32, erdp as u32 | (1 << 3));
        core::ptr::write_volatile((ir0 + 0x1C) as *mut u32, (erdp >> 32) as u32);

        // ERSTBA (Event Ring Segment Table Base Address)
        core::ptr::write_volatile((ir0 + 0x10) as *mut u32, erst_phys as u32);
        core::ptr::write_volatile((ir0 + 0x14) as *mut u32, (erst_phys >> 32) as u32);

        // Step 7: Enable interrupts and start
        core::ptr::write_volatile((ir0 + 0x00) as *mut u32, 0x2); // IMAN: IE=1
        write_op(USBCMD, CMD_RUN | CMD_INTE);

        while read_op(USBSTS) & STS_HCH != 0 { core::hint::spin_loop(); }

        crate::kprintln!("  xHCI controller running");
        XHCI_READY.store(true, Ordering::SeqCst);
    }
}

// === Public transfer API ===

pub fn bulk_transfer_in(slot: u8, endpoint: u8, buf: &mut [u8]) -> Result<usize, &'static str> {
    if !XHCI_READY.load(Ordering::Relaxed) { return Err("xHCI not ready"); }

    let mut trb = Trb::new();
    trb.parameter = buf.as_mut_ptr() as u64;
    trb.status = buf.len() as u32;
    trb.set_trb_type(TRB_NORMAL);
    trb.set_ioc();

    let ring_idx = endpoint as usize;
    unsafe {
        if ring_idx >= TRANSFER_RINGS.len() { return Err("endpoint out of range"); }
        TRANSFER_RINGS[ring_idx].enqueue(trb);
        // IN endpoint DCI = 2*N + 1
        ring_doorbell(slot, endpoint * 2 + 1);
    }

    // TODO: poll Event Ring for Transfer Event completion, return actual length
    Ok(buf.len())
}

pub fn bulk_transfer_out(slot: u8, endpoint: u8, buf: &[u8]) -> Result<usize, &'static str> {
    if !XHCI_READY.load(Ordering::Relaxed) { return Err("xHCI not ready"); }

    let mut trb = Trb::new();
    trb.parameter = buf.as_ptr() as u64;
    trb.status = buf.len() as u32;
    trb.set_trb_type(TRB_NORMAL);
    trb.set_ioc();

    let ring_idx = endpoint as usize;
    unsafe {
        if ring_idx >= TRANSFER_RINGS.len() { return Err("endpoint out of range"); }
        TRANSFER_RINGS[ring_idx].enqueue(trb);
        // OUT endpoint DCI = 2*N
        ring_doorbell(slot, endpoint * 2);
    }

    Ok(buf.len())
}

/// Poll event ring for completion events
pub fn poll_events() {
    if !XHCI_READY.load(Ordering::Relaxed) { return; }

    unsafe {
        let ring = &mut EVT_RING;
        loop {
            let trb = &ring.trbs[ring.dequeue_idx];
            if trb.cycle_bit() != ring.cycle_bit { break; }

            match trb.trb_type() {
                TRB_TRANSFER_EVENT => {
                    let _cc = trb.completion_code();
                    // TODO: wake waiting transfer, return actual byte count
                }
                TRB_CMD_COMPLETION => {
                    // TODO: process command result
                }
                TRB_PORT_STATUS_CHANGE => {
                    let port_id = ((trb.parameter >> 24) & 0xFF) as u8;
                    crate::kprintln!("  USB port {} status change", port_id);
                }
                _ => {}
            }

            ring.dequeue_idx += 1;
            if ring.dequeue_idx >= RING_SIZE {
                ring.dequeue_idx = 0;
                ring.cycle_bit = !ring.cycle_bit;
            }
        }
    }
}
