//! xHCI (USB 3.0) Host Controller Driver
//!
//! The xHCI specification defines a ring-based architecture:
//! - Command Ring: host → controller commands (device slot, endpoint config)
//! - Event Ring: controller → host notifications (transfer complete, port change)
//! - Transfer Rings: per-endpoint data transfer descriptors (TRBs)
//!
//! Pi 5 RP1 has an xHCI controller on the PCIe bus.
//! We need: device enumeration, bulk IN/OUT for SIRIUS endpoints.

use alloc::vec::Vec;
use core::sync::atomic::{AtomicU64, AtomicBool, Ordering};

/// xHCI MMIO base (from PCI BAR0)
static XHCI_BASE: AtomicU64 = AtomicU64::new(0);
static XHCI_READY: AtomicBool = AtomicBool::new(false);

// === xHCI Register Offsets (Capability Registers) ===
const CAPLENGTH: u64 = 0x00;        // Capability Register Length (1 byte)
const HCIVERSION: u64 = 0x02;       // Interface Version (2 bytes)
const HCSPARAMS1: u64 = 0x04;       // Structural Parameters 1
const HCSPARAMS2: u64 = 0x08;       // Structural Parameters 2
const HCCPARAMS1: u64 = 0x10;       // Capability Parameters 1
const DBOFF: u64 = 0x14;            // Doorbell Offset
const RTSOFF: u64 = 0x18;           // Runtime Register Space Offset

// === Operational Registers (base + CAPLENGTH) ===
const USBCMD: u64 = 0x00;           // USB Command
const USBSTS: u64 = 0x04;           // USB Status
const PAGESIZE: u64 = 0x08;         // Page Size
const DNCTRL: u64 = 0x14;           // Device Notification Control
const CRCR: u64 = 0x18;             // Command Ring Control
const DCBAAP: u64 = 0x30;           // Device Context Base Address Array Pointer
const CONFIG: u64 = 0x38;           // Configure

// USBCMD bits
const CMD_RUN: u32 = 1 << 0;
const CMD_HCRST: u32 = 1 << 1;
const CMD_INTE: u32 = 1 << 2;

// USBSTS bits
const STS_HCH: u32 = 1 << 0;       // HC Halted
const STS_EINT: u32 = 1 << 3;      // Event Interrupt

// === TRB (Transfer Request Block) — 16 bytes each ===
#[repr(C, align(16))]
#[derive(Clone, Copy, Default)]
pub struct Trb {
    pub param: u64,      // Parameter (data pointer or inline data)
    pub status: u32,     // Status / Transfer Length / Completion Code
    pub control: u32,    // Type, flags, cycle bit
}

impl Trb {
    pub fn trb_type(&self) -> u8 {
        ((self.control >> 10) & 0x3F) as u8
    }

    pub fn cycle_bit(&self) -> bool {
        self.control & 1 != 0
    }

    pub fn completion_code(&self) -> u8 {
        (self.status >> 24) as u8
    }
}

// TRB Types
const TRB_NORMAL: u32 = 1;
const TRB_SETUP_STAGE: u32 = 2;
const TRB_DATA_STAGE: u32 = 3;
const TRB_STATUS_STAGE: u32 = 4;
const TRB_LINK: u32 = 6;
const TRB_NO_OP: u32 = 8;
const TRB_ENABLE_SLOT: u32 = 9;
const TRB_DISABLE_SLOT: u32 = 10;
const TRB_ADDRESS_DEVICE: u32 = 11;
const TRB_CONFIG_ENDPOINT: u32 = 12;
const TRB_EVALUATE_CONTEXT: u32 = 13;
const TRB_RESET_ENDPOINT: u32 = 14;
const TRB_TRANSFER_EVENT: u32 = 32;
const TRB_CMD_COMPLETION: u32 = 33;
const TRB_PORT_STATUS_CHANGE: u32 = 34;

// === Ring (circular TRB buffer) ===
const RING_SIZE: usize = 256;

#[repr(C, align(4096))]
struct TrbRing {
    trbs: [Trb; RING_SIZE],
    enqueue_idx: usize,
    cycle_bit: bool,
}

impl TrbRing {
    const fn new() -> Self {
        Self {
            trbs: [Trb { param: 0, status: 0, control: 0 }; RING_SIZE],
            enqueue_idx: 0,
            cycle_bit: true,
        }
    }

    fn enqueue(&mut self, mut trb: Trb) -> usize {
        // Set cycle bit
        if self.cycle_bit {
            trb.control |= 1;
        } else {
            trb.control &= !1;
        }

        let idx = self.enqueue_idx;
        self.trbs[idx] = trb;
        self.enqueue_idx += 1;

        // Wrap around with Link TRB
        if self.enqueue_idx >= RING_SIZE - 1 {
            let link = Trb {
                param: &self.trbs[0] as *const Trb as u64,
                status: 0,
                control: (TRB_LINK << 10) | if self.cycle_bit { 1 } else { 0 } | (1 << 1), // Toggle Cycle
            };
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
struct EventRingSegmentTableEntry {
    ring_segment_base: u64,
    ring_segment_size: u16,
    _reserved: [u8; 6],
}

// === Device Context Base Address Array ===
const MAX_SLOTS: usize = 32;

#[repr(C, align(4096))]
struct Dcbaa {
    entries: [u64; MAX_SLOTS + 1], // slot 0 = scratchpad
}

// === xHCI Controller State ===
static mut COMMAND_RING: TrbRing = TrbRing::new();
static mut EVENT_RING: TrbRing = TrbRing::new();
static mut DCBAA: Dcbaa = Dcbaa { entries: [0; MAX_SLOTS + 1] };

// Per-endpoint transfer rings (slot, endpoint) → ring
// For SIRIUS we need: EP1 OUT, EP1 IN, EP2 IN, EP4 IN, EP6 IN
static mut TRANSFER_RINGS: [TrbRing; 8] = [
    TrbRing::new(), TrbRing::new(), TrbRing::new(), TrbRing::new(),
    TrbRing::new(), TrbRing::new(), TrbRing::new(), TrbRing::new(),
];

fn op_base() -> u64 {
    let base = XHCI_BASE.load(Ordering::Relaxed);
    let caplength = unsafe { crate::arch::aarch64::mmio_read32(base + CAPLENGTH) } & 0xFF;
    base + caplength as u64
}

fn rt_base() -> u64 {
    let base = XHCI_BASE.load(Ordering::Relaxed);
    let rtsoff = unsafe { crate::arch::aarch64::mmio_read32(base + RTSOFF) };
    base + (rtsoff & !0x1F) as u64
}

fn db_base() -> u64 {
    let base = XHCI_BASE.load(Ordering::Relaxed);
    let dboff = unsafe { crate::arch::aarch64::mmio_read32(base + DBOFF) };
    base + (dboff & !0x3) as u64
}

/// Ring the doorbell for a slot (0 = host controller, 1+ = device slots)
fn ring_doorbell(slot: u8, target: u8) {
    let addr = db_base() + (slot as u64) * 4;
    unsafe { crate::arch::aarch64::mmio_write32(addr, target as u32); }
}

pub fn init() {
    // Find xHCI on PCI bus (class 0C, subclass 03, prog_if 30)
    let xhci_dev = match super::pci::find_by_class(0x0C, 0x03) {
        Some(dev) if dev.prog_if == 0x30 => dev,
        _ => {
            crate::kprintln!("  xHCI: not found on PCI bus");
            return;
        }
    };

    // Get BAR0 (MMIO base)
    let bar0 = xhci_dev.bars[0] & !0xF;
    let xhci_base = crate::phys_to_virt(bar0);
    XHCI_BASE.store(xhci_base, Ordering::SeqCst);

    crate::kprintln!("  xHCI at PCI {:02x}:{:02x}.{}, BAR0={:#x}",
        xhci_dev.bus, xhci_dev.device, xhci_dev.function, bar0);

    unsafe {
        let base = xhci_base;

        // Read capabilities
        let hciversion = crate::arch::aarch64::mmio_read32(base + HCIVERSION) >> 16;
        let hcsparams1 = crate::arch::aarch64::mmio_read32(base + HCSPARAMS1);
        let max_slots = hcsparams1 & 0xFF;
        let max_ports = (hcsparams1 >> 24) & 0xFF;

        crate::kprintln!("  xHCI version {}.{}, {} slots, {} ports",
            hciversion >> 8, hciversion & 0xFF, max_slots, max_ports);

        let op = op_base();

        // Step 1: Stop the controller
        let cmd = crate::arch::aarch64::mmio_read32(op + USBCMD);
        crate::arch::aarch64::mmio_write32(op + USBCMD, cmd & !CMD_RUN);

        // Wait for halt
        while crate::arch::aarch64::mmio_read32(op + USBSTS) & STS_HCH == 0 {
            core::hint::spin_loop();
        }

        // Step 2: Reset controller
        crate::arch::aarch64::mmio_write32(op + USBCMD, CMD_HCRST);
        while crate::arch::aarch64::mmio_read32(op + USBCMD) & CMD_HCRST != 0 {
            core::hint::spin_loop();
        }
        while crate::arch::aarch64::mmio_read32(op + USBSTS) & STS_HCH == 0 {
            core::hint::spin_loop();
        }

        // Step 3: Configure max device slots
        let slots = max_slots.min(MAX_SLOTS as u32);
        crate::arch::aarch64::mmio_write32(op + CONFIG, slots);

        // Step 4: Set DCBAAP
        let dcbaa_phys = crate::virt_to_phys(&DCBAA as *const Dcbaa as u64);
        crate::arch::aarch64::mmio_write32(op + DCBAAP, dcbaa_phys as u32);
        crate::arch::aarch64::mmio_write32(op + DCBAAP + 4, (dcbaa_phys >> 32) as u32);

        // Step 5: Set Command Ring
        let crcr_phys = COMMAND_RING.phys_addr();
        // CRCR: ring pointer | RCS (Ring Cycle State) = 1
        crate::arch::aarch64::mmio_write32(op + CRCR, (crcr_phys as u32) | 1);
        crate::arch::aarch64::mmio_write32(op + CRCR + 4, (crcr_phys >> 32) as u32);

        // Step 6: Set up Event Ring (interrupter 0)
        let rt = rt_base();
        let ir0 = rt + 0x20; // Interrupter Register Set 0

        // Event Ring Segment Table
        static mut ERST: EventRingSegmentTableEntry = EventRingSegmentTableEntry {
            ring_segment_base: 0,
            ring_segment_size: RING_SIZE as u16,
            _reserved: [0; 6],
        };
        ERST.ring_segment_base = EVENT_RING.phys_addr();

        let erst_phys = crate::virt_to_phys(&ERST as *const _ as u64);

        // ERSTSZ = 1 (one segment)
        crate::arch::aarch64::mmio_write32(ir0 + 0x08, 1);

        // ERDP (Event Ring Dequeue Pointer)
        let erdp = EVENT_RING.phys_addr();
        crate::arch::aarch64::mmio_write32(ir0 + 0x18, erdp as u32 | (1 << 3)); // EHB
        crate::arch::aarch64::mmio_write32(ir0 + 0x1C, (erdp >> 32) as u32);

        // ERSTBA (Event Ring Segment Table Base Address)
        crate::arch::aarch64::mmio_write32(ir0 + 0x10, erst_phys as u32);
        crate::arch::aarch64::mmio_write32(ir0 + 0x14, (erst_phys >> 32) as u32);

        // Step 7: Enable interrupts and start controller
        crate::arch::aarch64::mmio_write32(ir0 + 0x00, 0x2); // IMAN: IE=1
        crate::arch::aarch64::mmio_write32(op + USBCMD, CMD_RUN | CMD_INTE);

        // Wait for running
        while crate::arch::aarch64::mmio_read32(op + USBSTS) & STS_HCH != 0 {
            core::hint::spin_loop();
        }

        crate::kprintln!("  xHCI controller running");
        XHCI_READY.store(true, Ordering::SeqCst);
    }
}

// === Public API for USB transfers ===

/// Send a control transfer (SETUP + DATA + STATUS stages)
pub fn control_transfer(
    slot: u8,
    request_type: u8,
    request: u8,
    value: u16,
    index: u16,
    data: Option<&mut [u8]>,
) -> Result<usize, &'static str> {
    if !XHCI_READY.load(Ordering::Relaxed) {
        return Err("xHCI not initialized");
    }

    let length = data.as_ref().map_or(0, |d| d.len()) as u16;

    // SETUP stage TRB
    let setup_param = (request_type as u64)
        | ((request as u64) << 8)
        | ((value as u64) << 16)
        | ((index as u64) << 32)
        | ((length as u64) << 48);

    let setup_trb = Trb {
        param: setup_param,
        status: 8, // TRB Transfer Length = 8 (setup packet is always 8 bytes)
        control: (TRB_SETUP_STAGE << 10) | (1 << 6), // IDT=1 (Immediate Data)
    };

    // DATA stage TRB (if data present)
    let data_trb = data.as_ref().map(|d| Trb {
        param: d.as_ptr() as u64,
        status: d.len() as u32,
        control: (TRB_DATA_STAGE << 10) | (1 << 16), // DIR=1 (IN)
    });

    // STATUS stage TRB
    let status_trb = Trb {
        param: 0,
        status: 0,
        control: (TRB_STATUS_STAGE << 10) | (1 << 5), // IOC=1
    };

    unsafe {
        let ring = &mut TRANSFER_RINGS[0]; // EP0 for default control pipe
        ring.enqueue(setup_trb);
        if let Some(dt) = data_trb {
            ring.enqueue(dt);
        }
        ring.enqueue(status_trb);

        // Ring doorbell (target = 1 for EP0)
        ring_doorbell(slot, 1);
    }

    // TODO: wait for transfer completion event on event ring
    Ok(length as usize)
}

/// Bulk IN transfer — reads data from a device endpoint
pub fn bulk_transfer_in(
    slot: u8,
    endpoint: u8,
    buf: &mut [u8],
) -> Result<usize, &'static str> {
    if !XHCI_READY.load(Ordering::Relaxed) {
        return Err("xHCI not initialized");
    }

    let trb = Trb {
        param: buf.as_mut_ptr() as u64,
        status: buf.len() as u32,
        control: (TRB_NORMAL << 10) | (1 << 5), // IOC=1
    };

    let ring_idx = endpoint as usize; // simplified mapping

    unsafe {
        if ring_idx >= TRANSFER_RINGS.len() {
            return Err("endpoint out of range");
        }
        TRANSFER_RINGS[ring_idx].enqueue(trb);

        // Doorbell: target = endpoint DCI (Device Context Index)
        // For IN endpoint N: DCI = 2*N + 1
        let dci = (endpoint as u8) * 2 + 1;
        ring_doorbell(slot, dci);
    }

    // TODO: poll event ring for completion, return actual transfer length
    Ok(buf.len())
}

/// Bulk OUT transfer — sends data to a device endpoint
pub fn bulk_transfer_out(
    slot: u8,
    endpoint: u8,
    buf: &[u8],
) -> Result<usize, &'static str> {
    if !XHCI_READY.load(Ordering::Relaxed) {
        return Err("xHCI not initialized");
    }

    let trb = Trb {
        param: buf.as_ptr() as u64,
        status: buf.len() as u32,
        control: (TRB_NORMAL << 10) | (1 << 5), // IOC=1
    };

    let ring_idx = endpoint as usize;

    unsafe {
        if ring_idx >= TRANSFER_RINGS.len() {
            return Err("endpoint out of range");
        }
        TRANSFER_RINGS[ring_idx].enqueue(trb);

        // For OUT endpoint N: DCI = 2*N
        let dci = (endpoint as u8) * 2;
        ring_doorbell(slot, dci);
    }

    Ok(buf.len())
}

/// Poll event ring for completion events
pub fn poll_events() {
    if !XHCI_READY.load(Ordering::Relaxed) { return; }

    unsafe {
        let ring = &mut EVENT_RING;
        loop {
            let trb = &ring.trbs[ring.enqueue_idx];
            if trb.cycle_bit() != ring.cycle_bit {
                break; // No new events
            }

            match trb.trb_type() {
                32 => { // Transfer Event
                    let _slot = ((trb.control >> 24) & 0xFF) as u8;
                    let _endpoint = ((trb.control >> 16) & 0x1F) as u8;
                    let _cc = trb.completion_code();
                    // TODO: wake up waiting transfer
                }
                33 => { // Command Completion Event
                    let _cc = trb.completion_code();
                    // TODO: process command result
                }
                34 => { // Port Status Change Event
                    let port_id = (trb.param >> 24) as u8;
                    crate::kprintln!("  USB port {} status change", port_id);
                    // TODO: enumerate newly connected device
                }
                _ => {}
            }

            ring.enqueue_idx += 1;
            if ring.enqueue_idx >= RING_SIZE {
                ring.enqueue_idx = 0;
                ring.cycle_bit = !ring.cycle_bit;
            }
        }
    }
}
