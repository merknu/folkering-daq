//! xHCI (USB 3.0) Host Controller Driver for RP1
//!
//! RP1 xHCI 0: BAR1 + 0x20_0000 (0x1F_0020_0000)
//! RP1 xHCI 1: BAR1 + 0x30_0000 (0x1F_0030_0000)
//!
//! Ring-based DMA architecture with synchronous completion polling.
//! Handles: controller init, port detection, device enumeration,
//! and bulk IN/OUT transfers for SIRIUS DAQ instrument.

use core::sync::atomic::{AtomicU64, AtomicBool, AtomicU32, Ordering};

static XHCI_BASE: AtomicU64 = AtomicU64::new(0);
static XHCI_READY: AtomicBool = AtomicBool::new(false);
static OP_BASE_CACHED: AtomicU64 = AtomicU64::new(0);
static DB_BASE_CACHED: AtomicU64 = AtomicU64::new(0);
static RT_BASE_CACHED: AtomicU64 = AtomicU64::new(0);
static MAX_PORTS: AtomicU32 = AtomicU32::new(0);

// === Capability Registers ===
const CAPLENGTH: u64 = 0x00;
const HCSPARAMS1: u64 = 0x04;
const HCCPARAMS1: u64 = 0x10;
const DBOFF: u64 = 0x14;
const RTSOFF: u64 = 0x18;

// === Operational Registers (base + CAPLENGTH) ===
const USBCMD: u64 = 0x00;
const USBSTS: u64 = 0x04;
const CRCR: u64 = 0x18;
const DCBAAP: u64 = 0x30;
const CONFIG: u64 = 0x38;
const PORTSC_BASE: u64 = 0x400; // Port 1 at +0x400, Port N at +0x400 + (N-1)*0x10

const CMD_RUN: u32 = 1 << 0;
const CMD_HCRST: u32 = 1 << 1;
const CMD_INTE: u32 = 1 << 2;
const STS_HCH: u32 = 1 << 0;

// PORTSC bits
const PORTSC_CCS: u32 = 1 << 0;   // Current Connect Status
const PORTSC_PED: u32 = 1 << 1;   // Port Enabled
const PORTSC_PR: u32 = 1 << 4;    // Port Reset
const PORTSC_PLS_MASK: u32 = 0xF << 5; // Port Link State
const PORTSC_PP: u32 = 1 << 9;    // Port Power
const PORTSC_SPEED_MASK: u32 = 0xF << 10; // Port Speed
const PORTSC_CSC: u32 = 1 << 17;  // Connect Status Change (W1C)
const PORTSC_PRC: u32 = 1 << 21;  // Port Reset Change (W1C)

// === TRB ===
#[repr(C, align(16))]
#[derive(Clone, Copy)]
pub struct Trb {
    pub parameter: u64,
    pub status: u32,
    pub control: u32,
}

impl Trb {
    pub const fn new() -> Self { Self { parameter: 0, status: 0, control: 0 } }

    #[inline] pub fn cycle_bit(&self) -> bool { (self.control & 1) != 0 }
    #[inline] pub fn trb_type(&self) -> u8 { ((self.control >> 10) & 0x3F) as u8 }
    #[inline] pub fn completion_code(&self) -> u8 { (self.status >> 24) as u8 }
    #[inline] pub fn transfer_length(&self) -> u32 { self.status & 0x00FF_FFFF }

    #[inline]
    pub fn set_type_cycle(&mut self, t: u8, cycle: bool) {
        self.control &= !(0x3F << 10 | 1);
        self.control |= ((t as u32) & 0x3F) << 10;
        if cycle { self.control |= 1; }
    }

    #[inline] pub fn set_ioc(&mut self) { self.control |= 1 << 5; }
}

pub const TRB_NORMAL: u8 = 1;
pub const TRB_SETUP_STAGE: u8 = 2;
pub const TRB_DATA_STAGE: u8 = 3;
pub const TRB_STATUS_STAGE: u8 = 4;
pub const TRB_LINK: u8 = 6;
pub const TRB_NO_OP_CMD: u8 = 23;
pub const TRB_ENABLE_SLOT: u8 = 9;
pub const TRB_DISABLE_SLOT: u8 = 10;
pub const TRB_ADDRESS_DEVICE: u8 = 11;
pub const TRB_CONFIG_ENDPOINT: u8 = 12;
pub const TRB_RESET_ENDPOINT: u8 = 14;
pub const TRB_TRANSFER_EVENT: u8 = 32;
pub const TRB_CMD_COMPLETION: u8 = 33;
pub const TRB_PORT_STATUS_CHANGE: u8 = 34;

// Completion codes
pub const CC_SUCCESS: u8 = 1;
pub const CC_SHORT_PACKET: u8 = 13;

// === Ring ===
const RING_SIZE: usize = 256;

#[repr(C, align(4096))]
struct TrbRing {
    trbs: [Trb; RING_SIZE],
    enqueue: usize,
    dequeue: usize,
    pcs: bool, // Producer Cycle State
}

impl TrbRing {
    const fn new() -> Self {
        Self { trbs: [Trb::new(); RING_SIZE], enqueue: 0, dequeue: 0, pcs: true }
    }

    fn enqueue(&mut self, mut trb: Trb) -> usize {
        if self.pcs { trb.control |= 1; } else { trb.control &= !1; }
        let idx = self.enqueue;
        // Volatile write — DMA target
        unsafe { core::ptr::write_volatile(&mut self.trbs[idx] as *mut Trb, trb); }
        self.enqueue += 1;
        if self.enqueue >= RING_SIZE - 1 {
            // Link TRB
            let mut link = Trb::new();
            link.parameter = self.phys_addr();
            link.set_type_cycle(TRB_LINK, self.pcs);
            link.control |= 1 << 1; // Toggle Cycle
            unsafe { core::ptr::write_volatile(&mut self.trbs[self.enqueue] as *mut Trb, link); }
            self.enqueue = 0;
            self.pcs = !self.pcs;
        }
        idx
    }

    fn phys_addr(&self) -> u64 {
        crate::virt_to_phys(&self.trbs[0] as *const Trb as u64)
    }
}

// === Event Ring Segment Table ===
#[repr(C, align(64))]
struct ErstEntry { base: u64, size: u16, _pad: [u8; 6] }

// === Device Context ===
const MAX_SLOTS: usize = 16;

#[repr(C, align(64))]
#[derive(Clone, Copy)]
struct SlotContext { data: [u32; 8] }
impl SlotContext { const fn new() -> Self { Self { data: [0; 8] } } }

#[repr(C, align(64))]
#[derive(Clone, Copy)]
struct EndpointContext { data: [u32; 8] }
impl EndpointContext { const fn new() -> Self { Self { data: [0; 8] } } }

// Input Context: slot + 31 endpoints
#[repr(C, align(4096))]
struct InputContext {
    input_control: [u32; 8],
    slot: SlotContext,
    endpoints: [EndpointContext; 31],
}

// Device Context: slot + 31 endpoints
#[repr(C, align(4096))]
struct DeviceContext {
    slot: SlotContext,
    endpoints: [EndpointContext; 31],
}

// === DCBAA ===
#[repr(C, align(4096))]
struct Dcbaa { entries: [u64; MAX_SLOTS + 1] }

// === Static State ===
static mut CMD_RING: TrbRing = TrbRing::new();
static mut EVT_RING: TrbRing = TrbRing::new();
static mut DCBAA: Dcbaa = Dcbaa { entries: [0; MAX_SLOTS + 1] };
static mut ERST: ErstEntry = ErstEntry { base: 0, size: RING_SIZE as u16, _pad: [0; 6] };

// Device contexts (one per slot)
static mut DEV_CONTEXTS: [DeviceContext; MAX_SLOTS] = {
    const DC: DeviceContext = DeviceContext {
        slot: SlotContext::new(),
        endpoints: [EndpointContext::new(); 31],
    };
    [DC; MAX_SLOTS]
};

static mut INPUT_CTX: InputContext = InputContext {
    input_control: [0; 8],
    slot: SlotContext::new(),
    endpoints: [EndpointContext::new(); 31],
};

// Transfer rings for each endpoint (0=EP0, 1=EP1OUT, 2=EP1IN, 3=EP2IN, etc.)
const NUM_EP_RINGS: usize = 16;
static mut EP_RINGS: [TrbRing; NUM_EP_RINGS] = {
    const R: TrbRing = TrbRing::new();
    [R; NUM_EP_RINGS]
};

// Last completed event (for synchronous polling)
static mut LAST_EVT: Trb = Trb::new();

// === MMIO Helpers ===
#[inline]
fn read32(addr: u64) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}
#[inline]
fn write32(addr: u64, v: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, v); }
}
#[inline]
fn read_op(off: u64) -> u32 { read32(OP_BASE_CACHED.load(Ordering::Relaxed) + off) }
#[inline]
fn write_op(off: u64, v: u32) { write32(OP_BASE_CACHED.load(Ordering::Relaxed) + off, v); }

fn ring_doorbell(slot: u8, target: u8) {
    let addr = DB_BASE_CACHED.load(Ordering::Relaxed) + (slot as u64) * 4;
    write32(addr, target as u32);
}

/// Read port status/control register
fn read_portsc(port: u32) -> u32 {
    let op = OP_BASE_CACHED.load(Ordering::Relaxed);
    read32(op + PORTSC_BASE + (port - 1) as u64 * 0x10)
}

/// Write port status/control register (preserving RsvdP, clearing RW1C with 0)
fn write_portsc(port: u32, val: u32) {
    let op = OP_BASE_CACHED.load(Ordering::Relaxed);
    write32(op + PORTSC_BASE + (port - 1) as u64 * 0x10, val);
}

// ============================================================
// Event Ring Polling (synchronous completion)
// ============================================================

/// Poll event ring until we get a completion event, or timeout.
/// Returns the event TRB. This is the core synchronization mechanism.
fn wait_for_event(timeout_us: u64) -> Result<Trb, &'static str> {
    let start = crate::arch::aarch64::timer::micros();
    let evt = unsafe { &mut EVT_RING };

    loop {
        // Read TRB at current dequeue position (volatile — DMA target)
        let trb = unsafe { core::ptr::read_volatile(&evt.trbs[evt.dequeue] as *const Trb) };

        // Check if this TRB belongs to us (cycle bit matches our expectation)
        if trb.cycle_bit() == evt.pcs {
            // Advance dequeue
            evt.dequeue += 1;
            if evt.dequeue >= RING_SIZE {
                evt.dequeue = 0;
                evt.pcs = !evt.pcs;
            }

            // Update ERDP (Event Ring Dequeue Pointer) to tell hardware we consumed it
            let rt = RT_BASE_CACHED.load(Ordering::Relaxed);
            let ir0 = rt + 0x20;
            let new_erdp = crate::virt_to_phys(&evt.trbs[evt.dequeue] as *const Trb as u64);
            write32(ir0 + 0x18, new_erdp as u32 | (1 << 3)); // EHB=1 (clear busy)
            write32(ir0 + 0x1C, (new_erdp >> 32) as u32);

            unsafe { LAST_EVT = trb; }
            return Ok(trb);
        }

        // Timeout check
        if crate::arch::aarch64::timer::micros() - start > timeout_us {
            return Err("xHCI event timeout");
        }

        core::hint::spin_loop();
    }
}

/// Wait specifically for a command completion event
fn wait_command_completion(timeout_us: u64) -> Result<(u8, u8), &'static str> {
    loop {
        let evt = wait_for_event(timeout_us)?;
        if evt.trb_type() == TRB_CMD_COMPLETION {
            let cc = evt.completion_code();
            let slot_id = ((evt.control >> 24) & 0xFF) as u8;
            return Ok((cc, slot_id));
        }
        // Not a command completion — might be port status change, handle it
        if evt.trb_type() == TRB_PORT_STATUS_CHANGE {
            let port = ((evt.parameter >> 24) & 0xFF) as u32;
            crate::kprintln!("  USB: port {} status change (during cmd wait)", port);
        }
    }
}

/// Wait for a transfer completion event on a specific endpoint
fn wait_transfer_completion(timeout_us: u64) -> Result<(u8, u32), &'static str> {
    loop {
        let evt = wait_for_event(timeout_us)?;
        if evt.trb_type() == TRB_TRANSFER_EVENT {
            let cc = evt.completion_code();
            // Residual = bytes NOT transferred
            let residual = evt.transfer_length();
            return Ok((cc, residual));
        }
        if evt.trb_type() == TRB_PORT_STATUS_CHANGE {
            let port = ((evt.parameter >> 24) & 0xFF) as u32;
            crate::kprintln!("  USB: port {} change (during transfer)", port);
        }
    }
}

// ============================================================
// USB Device Enumeration
// ============================================================

/// Send a command on the Command Ring and wait for completion
fn send_command(trb: Trb, timeout_us: u64) -> Result<(u8, u8), &'static str> {
    unsafe { CMD_RING.enqueue(trb); }
    // Ring Host Controller doorbell (slot 0, target 0)
    ring_doorbell(0, 0);
    wait_command_completion(timeout_us)
}

/// Enable Slot — ask xHCI to allocate a device slot
pub fn enable_slot() -> Result<u8, &'static str> {
    let mut trb = Trb::new();
    trb.set_type_cycle(TRB_ENABLE_SLOT, false); // cycle set by ring.enqueue
    let (cc, slot_id) = send_command(trb, 500_000)?; // 500ms timeout
    if cc != CC_SUCCESS {
        return Err("Enable Slot failed");
    }
    crate::kprintln!("  USB: slot {} enabled", slot_id);
    Ok(slot_id)
}

/// Reset a USB port
pub fn reset_port(port: u32) {
    let ps = read_portsc(port);
    // Set Port Reset, preserve PP and other RW bits, clear W1C bits
    let val = (ps & !(PORTSC_CSC | PORTSC_PRC)) | PORTSC_PR;
    write_portsc(port, val);

    // Wait for reset to complete (PRC set by hardware)
    let start = crate::arch::aarch64::timer::micros();
    loop {
        let ps = read_portsc(port);
        if ps & PORTSC_PRC != 0 {
            // Clear PRC (write-1-to-clear)
            write_portsc(port, ps | PORTSC_PRC);
            break;
        }
        if crate::arch::aarch64::timer::micros() - start > 500_000 {
            crate::kprintln!("  USB: port {} reset timeout", port);
            break;
        }
        core::hint::spin_loop();
    }
}

/// Get port speed (1=Full, 2=Low, 3=High, 4=SuperSpeed)
pub fn port_speed(port: u32) -> u8 {
    ((read_portsc(port) & PORTSC_SPEED_MASK) >> 10) as u8
}

/// Address Device — assign USB address using Input Context
pub fn address_device(slot_id: u8, port: u32, speed: u8) -> Result<(), &'static str> {
    unsafe {
        // Build Input Context
        INPUT_CTX.input_control = [0; 8];
        INPUT_CTX.input_control[1] = 0x3; // Add flags: Slot (bit 0) + EP0 (bit 1)

        // Slot Context (dword 0): Route String=0, Speed, Context Entries=1
        let speed_bits = (speed as u32) << 20;
        INPUT_CTX.slot.data[0] = speed_bits | (1 << 27); // Context Entries = 1
        INPUT_CTX.slot.data[1] = (port << 16) & 0x00FF_0000; // Root Hub Port Number

        // Endpoint 0 Context (Default Control Pipe)
        let max_packet = match speed {
            1 => 8,    // Full Speed: 8 bytes
            2 => 8,    // Low Speed: 8 bytes
            3 => 64,   // High Speed: 64 bytes
            4 => 512,  // SuperSpeed: 512 bytes
            _ => 64,
        };

        // EP0 context: EP Type = Control (4), MaxPacketSize, CErr=3
        let ep0 = &mut INPUT_CTX.endpoints[0];
        ep0.data[0] = 0; // EP State = Disabled (hardware sets to Running)
        ep0.data[1] = (4 << 3) | (3 << 1) | ((max_packet as u32) << 16); // Type=Control, CErr=3
        // TR Dequeue Pointer (physical address of EP0 transfer ring)
        let ep0_ring_phys = EP_RINGS[0].phys_addr();
        ep0.data[2] = (ep0_ring_phys as u32) | 1; // DCS=1 (Dequeue Cycle State)
        ep0.data[3] = (ep0_ring_phys >> 32) as u32;
        ep0.data[4] = (8 << 16); // Average TRB Length = 8 (for control)

        // Set up DCBAA entry for this slot
        let dev_ctx_phys = crate::virt_to_phys(&DEV_CONTEXTS[slot_id as usize - 1] as *const _ as u64);
        DCBAA.entries[slot_id as usize] = dev_ctx_phys;

        // Build Address Device command TRB
        let input_ctx_phys = crate::virt_to_phys(&INPUT_CTX as *const _ as u64);
        let mut trb = Trb::new();
        trb.parameter = input_ctx_phys;
        trb.set_type_cycle(TRB_ADDRESS_DEVICE, false);
        trb.control |= (slot_id as u32) << 24; // Slot ID in bits 31:24

        let (cc, _) = send_command(trb, 1_000_000)?; // 1s timeout
        if cc != CC_SUCCESS {
            crate::kprintln!("  USB: Address Device failed, CC={}", cc);
            return Err("Address Device failed");
        }

        crate::kprintln!("  USB: slot {} addressed (speed={}, port={})", slot_id, speed, port);
    }
    Ok(())
}

/// Scan all ports for connected devices
pub fn scan_ports() -> Option<(u32, u8)> {
    let num_ports = MAX_PORTS.load(Ordering::Relaxed);
    for port in 1..=num_ports {
        let ps = read_portsc(port);
        if ps & PORTSC_CCS != 0 {
            let speed = port_speed(port);
            let speed_str = match speed {
                1 => "Full (12 Mbps)",
                2 => "Low (1.5 Mbps)",
                3 => "High (480 Mbps)",
                4 => "SuperSpeed (5 Gbps)",
                _ => "Unknown",
            };
            crate::kprintln!("  USB: device on port {}, speed: {}", port, speed_str);
            return Some((port, speed));
        }
    }
    None
}

// ============================================================
// Controller Init
// ============================================================

pub fn init() {
    let rp1 = match super::pci::rp1() {
        Some(r) => r,
        None => { crate::kprintln!("  xHCI: RP1 not found"); return; }
    };

    let base = rp1.xhci0_base;
    XHCI_BASE.store(base, Ordering::SeqCst);

    // Cache derived bases
    let caplength = read32(base) & 0xFF;
    let op = base + caplength as u64;
    OP_BASE_CACHED.store(op, Ordering::SeqCst);

    let dboff = read32(base + DBOFF) & !0x3;
    DB_BASE_CACHED.store(base + dboff as u64, Ordering::SeqCst);

    let rtsoff = read32(base + RTSOFF) & !0x1F;
    RT_BASE_CACHED.store(base + rtsoff as u64, Ordering::SeqCst);

    let hcsparams1 = read32(base + HCSPARAMS1);
    let max_slots = hcsparams1 & 0xFF;
    let max_ports_val = (hcsparams1 >> 24) & 0xFF;
    MAX_PORTS.store(max_ports_val, Ordering::SeqCst);

    let hciversion = (read32(base) >> 16) & 0xFFFF;
    crate::kprintln!("  xHCI v{}.{}, {} slots, {} ports",
        hciversion >> 8, hciversion & 0xFF, max_slots, max_ports_val);

    // Halt
    write_op(USBCMD, read_op(USBCMD) & !CMD_RUN);
    while read_op(USBSTS) & STS_HCH == 0 { core::hint::spin_loop(); }

    // Reset
    write_op(USBCMD, CMD_HCRST);
    while read_op(USBCMD) & CMD_HCRST != 0 { core::hint::spin_loop(); }

    // Configure
    let slots = max_slots.min(MAX_SLOTS as u32);
    write_op(CONFIG, slots);

    unsafe {
        // DCBAAP
        let dcbaa_phys = crate::virt_to_phys(&DCBAA as *const _ as u64);
        write_op(DCBAAP, dcbaa_phys as u32);
        write_op(DCBAAP + 4, (dcbaa_phys >> 32) as u32);

        // Command Ring
        let crcr = CMD_RING.phys_addr();
        write_op(CRCR, (crcr as u32) | 1);
        write_op(CRCR + 4, (crcr >> 32) as u32);

        // Event Ring
        let rt = RT_BASE_CACHED.load(Ordering::Relaxed);
        let ir0 = rt + 0x20;
        ERST.base = EVT_RING.phys_addr();
        let erst_phys = crate::virt_to_phys(&ERST as *const _ as u64);

        write32(ir0 + 0x08, 1); // ERSTSZ
        let erdp = EVT_RING.phys_addr();
        write32(ir0 + 0x18, erdp as u32 | (1 << 3));
        write32(ir0 + 0x1C, (erdp >> 32) as u32);
        write32(ir0 + 0x10, erst_phys as u32);
        write32(ir0 + 0x14, (erst_phys >> 32) as u32);

        // Enable & run
        write32(ir0 + 0x00, 0x2); // IMAN: IE
        write_op(USBCMD, CMD_RUN | CMD_INTE);
        while read_op(USBSTS) & STS_HCH != 0 { core::hint::spin_loop(); }
    }

    crate::kprintln!("  xHCI running");
    XHCI_READY.store(true, Ordering::SeqCst);

    // Power on all ports
    for p in 1..=max_ports_val {
        let ps = read_portsc(p);
        if ps & PORTSC_PP == 0 {
            write_portsc(p, ps | PORTSC_PP);
        }
    }

    // Wait for port power stabilization
    crate::arch::aarch64::timer::delay_ms(100);

    // Scan for connected devices
    if let Some((port, speed)) = scan_ports() {
        // Reset the port
        reset_port(port);
        crate::kprintln!("  USB: port {} reset complete", port);

        // Enable slot and address device
        match enable_slot() {
            Ok(slot_id) => {
                if let Err(e) = address_device(slot_id, port, speed) {
                    crate::kprintln!("  USB: address failed: {}", e);
                }
            }
            Err(e) => crate::kprintln!("  USB: enable slot failed: {}", e),
        }
    } else {
        crate::kprintln!("  USB: no devices connected");
    }
}

// ============================================================
// Bulk Transfer API (synchronous)
// ============================================================

/// Bulk IN: read data from device endpoint. Returns actual bytes transferred.
pub fn bulk_transfer_in(slot: u8, endpoint: u8, buf: &mut [u8]) -> Result<usize, &'static str> {
    if !XHCI_READY.load(Ordering::Relaxed) { return Err("xHCI not ready"); }

    let ring_idx = (endpoint as usize) * 2 + 1; // IN endpoints are odd DCI
    if ring_idx >= NUM_EP_RINGS { return Err("endpoint out of range"); }

    let mut trb = Trb::new();
    trb.parameter = buf.as_mut_ptr() as u64;
    trb.status = buf.len() as u32;
    trb.set_type_cycle(TRB_NORMAL, false);
    trb.set_ioc();

    unsafe { EP_RINGS[ring_idx].enqueue(trb); }

    // Ring doorbell: DCI for IN EP N = 2*N + 1
    ring_doorbell(slot, endpoint * 2 + 1);

    // Wait for transfer completion
    let (cc, residual) = wait_transfer_completion(2_000_000)?; // 2s timeout
    if cc != CC_SUCCESS && cc != CC_SHORT_PACKET {
        return Err("bulk IN transfer error");
    }

    let transferred = buf.len() - residual as usize;
    Ok(transferred)
}

/// Bulk OUT: write data to device endpoint. Returns actual bytes transferred.
pub fn bulk_transfer_out(slot: u8, endpoint: u8, buf: &[u8]) -> Result<usize, &'static str> {
    if !XHCI_READY.load(Ordering::Relaxed) { return Err("xHCI not ready"); }

    let ring_idx = (endpoint as usize) * 2; // OUT endpoints are even DCI
    if ring_idx >= NUM_EP_RINGS { return Err("endpoint out of range"); }

    let mut trb = Trb::new();
    trb.parameter = buf.as_ptr() as u64;
    trb.status = buf.len() as u32;
    trb.set_type_cycle(TRB_NORMAL, false);
    trb.set_ioc();

    unsafe { EP_RINGS[ring_idx].enqueue(trb); }

    // Ring doorbell: DCI for OUT EP N = 2*N
    ring_doorbell(slot, endpoint * 2);

    let (cc, residual) = wait_transfer_completion(2_000_000)?;
    if cc != CC_SUCCESS {
        return Err("bulk OUT transfer error");
    }

    Ok(buf.len() - residual as usize)
}

/// Poll for port status changes (non-blocking)
pub fn poll_events() {
    if !XHCI_READY.load(Ordering::Relaxed) { return; }

    unsafe {
        let evt = &mut EVT_RING;
        loop {
            let trb = core::ptr::read_volatile(&evt.trbs[evt.dequeue] as *const Trb);
            if trb.cycle_bit() != evt.pcs { break; }

            if trb.trb_type() == TRB_PORT_STATUS_CHANGE {
                let port = ((trb.parameter >> 24) & 0xFF) as u32;
                crate::kprintln!("  USB: port {} status change", port);
            }

            evt.dequeue += 1;
            if evt.dequeue >= RING_SIZE { evt.dequeue = 0; evt.pcs = !evt.pcs; }

            // Update ERDP
            let rt = RT_BASE_CACHED.load(Ordering::Relaxed);
            let ir0 = rt + 0x20;
            let erdp = crate::virt_to_phys(&evt.trbs[evt.dequeue] as *const Trb as u64);
            write32(ir0 + 0x18, erdp as u32 | (1 << 3));
            write32(ir0 + 0x1C, (erdp >> 32) as u32);
        }
    }
}
