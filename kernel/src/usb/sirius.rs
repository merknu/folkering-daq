//! Dewesoft SIRIUSi-HS USB Protocol Driver
//!
//! Reverse-engineered from Wireshark USB captures (PQTech-openDAQ).
//! Ported from Python to bare-metal Rust.
//!
//! Hardware: SIRIUSi-HS, 8 analog channels, 20 kHz, 16-bit
//! USB: VID=0x1CED (Dewesoft), PID=0x1002
//!
//! Endpoints:
//!   EP1 OUT (0x01): Commands (512 B max)
//!   EP1 IN  (0x81): Responses (512 B max)
//!   EP2 IN  (0x82): ADC data (15872 B) — 992 frames × 8ch × int16 LE
//!   EP4 IN  (0x84): Status (20 B)
//!   EP6 IN  (0x86): Sync (15872 B) — must read in lockstep with EP2
//!
//! Protocol: 15-byte AD commands + 1-byte B1 poll for register R/W.
//! Heartbeat: B1 at ~150/sec during streaming (device disconnects otherwise).

use core::sync::atomic::{AtomicBool, AtomicU64, AtomicU8, Ordering};

pub const SIRIUS_VID: u16 = 0x1CED;
pub const SIRIUS_PID: u16 = 0x1002;

pub const ADC_PACKET_SIZE: usize = 15872;
pub const FRAMES_PER_PACKET: usize = 992;
pub const NUM_CHANNELS: usize = 8;
pub const SAMPLE_RATE: u32 = 20000;

static INITIALIZED: AtomicBool = AtomicBool::new(false);
static STREAMING: AtomicBool = AtomicBool::new(false);
static PACKETS_RX: AtomicU64 = AtomicU64::new(0);
static USB_SLOT: AtomicU8 = AtomicU8::new(0);

/// Double-buffered ADC data
static mut BUF_A: [[i16; FRAMES_PER_PACKET]; NUM_CHANNELS] = [[0; FRAMES_PER_PACKET]; NUM_CHANNELS];
static mut BUF_B: [[i16; FRAMES_PER_PACKET]; NUM_CHANNELS] = [[0; FRAMES_PER_PACKET]; NUM_CHANNELS];
static ACTIVE_BUF: AtomicBool = AtomicBool::new(false);

// ============================================================
// AD Command Protocol
// ============================================================

/// Build 15-byte AD command for register operations
fn build_ad_cmd(op: u8, slot: u8, register: u8, data: [u8; 3]) -> [u8; 15] {
    let (pad, s) = if op == 0x08 || op == 0x1C {
        ([0xFF, 0xFF, 0xFF], 0xFF)
    } else {
        ([0x00, 0x00, 0x00], slot)
    };
    [0xAD, 0x3F, 0x0C, 0x00, 0x00, 0x00, op, pad[0], pad[1], pad[2], s, register, data[0], data[1], data[2]]
}

/// Execute AD command with B1 poll cycle. Returns response bytes.
fn ad_command(op: u8, slot: u8, reg: u8, data: [u8; 3], max_polls: u32) -> Result<[u8; 64], &'static str> {
    let usb = USB_SLOT.load(Ordering::Relaxed);
    let cmd = build_ad_cmd(op, slot, reg, data);

    // Send AD command on EP1 OUT
    crate::drivers::xhci::bulk_transfer_out(usb, 1, &cmd)?;

    // Read ACK from EP1 IN
    let mut ack = [0u8; 64];
    crate::drivers::xhci::bulk_transfer_in(usb, 1, &mut ack)?;

    // B1 poll loop — wait for device to process the command
    let b1 = [0xB1u8];
    let mut resp = [0u8; 64];

    for _ in 0..max_polls {
        crate::drivers::xhci::bulk_transfer_out(usb, 1, &b1)?;
        let n = crate::drivers::xhci::bulk_transfer_in(usb, 1, &mut resp)?;

        if n > 0 {
            if op == 0x13 {
                // Write operation: 0x00 = confirmed
                return Ok(resp);
            } else {
                // Read operation: 0x01 = data ready
                if resp[0] == 0x01 {
                    return Ok(resp);
                }
            }
        }
    }

    Err("B1 poll timeout")
}

/// Write register via AD command
fn write_reg(slot: u8, reg: u8, data: [u8; 3]) -> Result<(), &'static str> {
    ad_command(0x13, slot, reg, data, 50)?;
    Ok(())
}

/// Write register (global, slot=0xFF)
fn write_global(reg: u8, data: [u8; 3], max_polls: u32) -> Result<(), &'static str> {
    ad_command(0x13, 0xFF, reg, data, max_polls)?;
    Ok(())
}

/// Read register via AD command
fn read_reg(slot: u8, reg: u8) -> Result<[u8; 64], &'static str> {
    ad_command(0x14, slot, reg, [0xFF, 0xFF, 0xFF], 50)
}

// ============================================================
// Simple Commands (direct EP1 send/receive)
// ============================================================

fn simple_cmd(cmd: &[u8]) -> Result<[u8; 64], &'static str> {
    let usb = USB_SLOT.load(Ordering::Relaxed);
    crate::drivers::xhci::bulk_transfer_out(usb, 1, cmd)?;
    let mut resp = [0u8; 64];
    crate::drivers::xhci::bulk_transfer_in(usb, 1, &mut resp)?;
    Ok(resp)
}

fn get_firmware_version() -> Result<[u8; 4], &'static str> {
    let r = simple_cmd(&[0x00])?;
    Ok([r[0], r[1], r[2], r[3]])
}

fn set_active_mode() -> Result<(), &'static str> { simple_cmd(&[0xA0, 0x01])?; Ok(()) }
fn get_slot_presence() -> Result<u8, &'static str> { Ok(simple_cmd(&[0xA1])?[0]) }
fn pre_start() -> Result<(), &'static str> { simple_cmd(&[0xA4, 0x00])?; Ok(()) }
fn init_reset() -> Result<(), &'static str> { simple_cmd(&[0xB0, 0x3F, 0x0C])?; Ok(()) }

fn heartbeat() -> Result<(), &'static str> {
    let usb = USB_SLOT.load(Ordering::Relaxed);
    crate::drivers::xhci::bulk_transfer_out(usb, 1, &[0xB1])?;
    let mut r = [0u8; 64];
    let _ = crate::drivers::xhci::bulk_transfer_in(usb, 1, &mut r);
    Ok(())
}

// ============================================================
// Start Acquisition (31 register writes + trigger)
// ============================================================

/// Global register writes to start ADC streaming.
/// Values from pcapng capture analysis.
static START_SEQ: &[(u8, [u8; 3], u32)] = &[
    (0x67, [0x00, 0x4E, 0x20], 50),   // Sample rate = 20000 Hz
    (0x7B, [0x00, 0x0C, 0x80], 50),   // Buffer config
    (0x82, [0x00, 0x00, 0x31], 50),   // Ch0 active
    (0x82, [0x01, 0x00, 0x31], 50),   // Ch1 active
    (0x82, [0x02, 0x00, 0x31], 50),   // Ch2 active
    (0x82, [0x03, 0x00, 0x31], 50),   // Ch3 active
    (0x82, [0x04, 0x00, 0x31], 50),   // Ch4 active
    (0x82, [0x05, 0x00, 0x31], 50),   // Ch5 active
    (0x82, [0x06, 0x00, 0x31], 50),   // Ch6 active
    (0x82, [0x07, 0x00, 0x31], 50),   // Ch7 active
    (0xE5, [0x00, 0x18, 0x00], 50),   // DMA/timing
    (0x6F, [0x3F, 0xFF, 0x23], 50),   // Channel enable mask
    (0x72, [0x00, 0x00, 0x02], 50),   // Trigger config
    (0x10, [0x00, 0x00, 0x00], 50),   // Control
    (0x11, [0x00, 0x00, 0x00], 50),   // Control
    (0x07, [0x03, 0x00, 0x00], 50),   // Mode flags
    (0x9C, [0x00, 0x64, 0x00], 50),   // Filter
    (0x98, [0x02, 0x14, 0x32], 50),   // Decimation
    (0x99, [0x60, 0x60, 0x00], 50),   // Sample timing
    (0x9D, [0x00, 0x00, 0x00], 50),   // Reset
    (0x96, [0xFF, 0xFF, 0xFF], 400),  // Apply config (SLOW — ~284 polls)
    (0xD0, [0x00, 0x00, 0x01], 50),   // DMA enable
    (0x68, [0x00, 0x00, 0xFF], 50),   // Channel mask (all 8)
    (0xCC, [0x00, 0x00, 0xC0], 50),   // DMA config
    (0xCD, [0x00, 0x00, 0x01], 50),   // DMA mode
    (0xCA, [0x10, 0x10, 0x10], 50),   // Slot mask 0-3
    (0xCB, [0x10, 0x10, 0x10], 50),   // Slot mask 4-7
    (0xCE, [0x10, 0x10, 0x00], 50),   // Slot enable
    (0xCF, [0x00, 0x00, 0x00], 50),   // Clear flags
    (0x84, [0x00, 0x00, 0x00], 50),   // Transfer start
    (0xC8, [0xFF, 0xFF, 0xFF], 50),   // DMA arm
    (0x64, [0xFF, 0xFF, 0xFF], 50),   // Streaming start
];

fn start_acquisition() -> Result<(), &'static str> {
    crate::kprintln!("  SIRIUS: Phase 1 — discovery");
    let _presence = get_slot_presence()?;
    set_active_mode()?;

    let fw = get_firmware_version()?;
    crate::kprintln!("  SIRIUS: FW {}.{}.{}.{}", fw[0], fw[1], fw[2], fw[3]);

    crate::kprintln!("  SIRIUS: Phase 2 — init/reset");
    init_reset()?;

    crate::kprintln!("  SIRIUS: Phase 3 — pre-start");
    pre_start()?;

    crate::kprintln!("  SIRIUS: Phase 4 — {} register writes", START_SEQ.len());
    for &(reg, data, polls) in START_SEQ {
        write_global(reg, data, polls)?;
    }

    // Phase 5: TRIGGER — register 0x02, read mode, up to 1000 polls
    // This is what starts EP2 data flow!
    crate::kprintln!("  SIRIUS: TRIGGER (reg 0x02)");
    ad_command(0x14, 0xFF, 0x02, [0xFF, 0xFF, 0xFF], 1000)?;

    // Confirm streaming — register 0x03, ~220ms after trigger
    crate::arch::aarch64::timer::delay_ms(220);
    ad_command(0x14, 0xFF, 0x03, [0xFF, 0xFF, 0xFF], 50)?;

    STREAMING.store(true, Ordering::SeqCst);
    crate::kprintln!("  SIRIUS: STREAMING — {} Hz, {} channels", SAMPLE_RATE, NUM_CHANNELS);
    Ok(())
}

// ============================================================
// ADC Data Decoding
// ============================================================

/// Deinterleave EP2 bulk data (992 frames × 8ch × int16 LE interleaved)
/// into per-channel arrays for the openDAQ server.
fn deinterleave(raw: &[u8], out: &mut [[i16; FRAMES_PER_PACKET]; NUM_CHANNELS]) {
    for frame in 0..FRAMES_PER_PACKET {
        let base = frame * NUM_CHANNELS * 2;
        for ch in 0..NUM_CHANNELS {
            let off = base + ch * 2;
            if off + 1 < raw.len() {
                out[ch][frame] = i16::from_le_bytes([raw[off], raw[off + 1]]);
            }
        }
    }
}

// ============================================================
// Public API
// ============================================================

pub fn init() {
    // SIRIUS init happens after xHCI has enumerated and addressed the device.
    // The slot ID is determined during xHCI init (first USB device found).
    // For now, assume slot 1.
    USB_SLOT.store(1, Ordering::Relaxed);

    match start_acquisition() {
        Ok(()) => {
            INITIALIZED.store(true, Ordering::SeqCst);
        }
        Err(e) => {
            crate::kprintln!("  SIRIUS: init failed — {}", e);
        }
    }
}

/// Main polling function — read ADC data + drain status/sync + heartbeat.
/// Call from kernel main loop.
pub fn poll() {
    if !STREAMING.load(Ordering::Relaxed) { return; }
    let usb = USB_SLOT.load(Ordering::Relaxed);

    // EP2 IN: ADC data (15872 bytes)
    let mut raw_adc = [0u8; ADC_PACKET_SIZE];
    match crate::drivers::xhci::bulk_transfer_in(usb, 2, &mut raw_adc) {
        Ok(n) if n > 0 => {
            let use_b = ACTIVE_BUF.load(Ordering::Relaxed);
            unsafe {
                if use_b {
                    deinterleave(&raw_adc[..n], &mut BUF_A);
                } else {
                    deinterleave(&raw_adc[..n], &mut BUF_B);
                }
            }
            ACTIVE_BUF.store(!use_b, Ordering::Release);
            PACKETS_RX.fetch_add(1, Ordering::Relaxed);
        }
        _ => {}
    }

    // EP4 IN: Status (20 bytes) — MUST drain to prevent FIFO stall
    let mut status = [0u8; 20];
    let _ = crate::drivers::xhci::bulk_transfer_in(usb, 4, &mut status);

    // EP6 IN: Sync (15872 bytes) — MUST drain in lockstep with EP2
    let mut sync = [0u8; ADC_PACKET_SIZE];
    let _ = crate::drivers::xhci::bulk_transfer_in(usb, 6, &mut sync);

    // B1 heartbeat — keep alive (~150/sec)
    let _ = heartbeat();
}

/// Get latest decoded ADC data
pub fn get_latest_data() -> &'static [[i16; FRAMES_PER_PACKET]; NUM_CHANNELS] {
    let use_b = ACTIVE_BUF.load(Ordering::Acquire);
    unsafe { if use_b { &BUF_B } else { &BUF_A } }
}

pub fn is_streaming() -> bool { STREAMING.load(Ordering::Relaxed) }
pub fn packets_received() -> u64 { PACKETS_RX.load(Ordering::Relaxed) }
