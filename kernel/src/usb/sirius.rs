//! Dewesoft SIRIUSi-HS USB Protocol Driver
//!
//! Reverse-engineered from Wireshark USB captures.
//! Ported from Python (PQTech-openDAQ) to bare-metal Rust.
//!
//! Hardware: SIRIUSi-HS, 8 analog channels, 20 kHz, 16-bit
//! USB: VID=0x1CED (Dewesoft), PID=0x1002
//!
//! Endpoints:
//!   EP1 OUT (0x01): Commands to device (512 B)
//!   EP1 IN  (0x81): Responses from device (512 B)
//!   EP2 IN  (0x82): ADC data stream (15872 B) — 992 frames × 8ch × int16
//!   EP4 IN  (0x84): Control/status (20 B)
//!   EP6 IN  (0x86): Sync/timestamp (15872 B)

use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};

/// Device identity
pub const SIRIUS_VID: u16 = 0x1CED;
pub const SIRIUS_PID: u16 = 0x1002;

/// ADC packet constants
pub const ADC_PACKET_SIZE: usize = 15872;
pub const FRAMES_PER_PACKET: usize = 992;
pub const NUM_CHANNELS: usize = 8;
pub const SAMPLE_RATE: u32 = 20000;

/// Driver state
static INITIALIZED: AtomicBool = AtomicBool::new(false);
static STREAMING: AtomicBool = AtomicBool::new(false);
static PACKETS_RECEIVED: AtomicU64 = AtomicU64::new(0);

/// USB slot assigned by xHCI
static mut USB_SLOT: u8 = 0;

/// Channel configuration
#[derive(Clone)]
pub struct ChannelConfig {
    pub scale: f64,       // Multiplication factor (raw → physical unit)
    pub offset: f64,      // Additive offset
    pub nullpoint: f64,   // Auto-calibrated zero point
    pub unit: &'static str,
    pub enabled: bool,
}

impl Default for ChannelConfig {
    fn default() -> Self {
        Self {
            scale: 1.0,
            offset: 0.0,
            nullpoint: 0.0,
            unit: "V",
            enabled: true,
        }
    }
}

/// ADC data packet (decoded)
pub struct AdcPacket {
    pub channels: [[i16; FRAMES_PER_PACKET]; NUM_CHANNELS],
    pub packet_number: u64,
}

/// Latest ADC data (double-buffered)
static mut ADC_BUFFER_A: [[i16; FRAMES_PER_PACKET]; NUM_CHANNELS] = [[0; FRAMES_PER_PACKET]; NUM_CHANNELS];
static mut ADC_BUFFER_B: [[i16; FRAMES_PER_PACKET]; NUM_CHANNELS] = [[0; FRAMES_PER_PACKET]; NUM_CHANNELS];
static ACTIVE_BUFFER: AtomicBool = AtomicBool::new(false); // false=A, true=B

/// Channel configs
static mut CHANNELS: [ChannelConfig; NUM_CHANNELS] = [
    ChannelConfig { scale: 1.0, offset: 0.0, nullpoint: 0.0, unit: "V", enabled: true },
    ChannelConfig { scale: 1.0, offset: 0.0, nullpoint: 0.0, unit: "V", enabled: true },
    ChannelConfig { scale: 1.0, offset: 0.0, nullpoint: 0.0, unit: "V", enabled: true },
    ChannelConfig { scale: 1.0, offset: 0.0, nullpoint: 0.0, unit: "V", enabled: true },
    ChannelConfig { scale: 1.0, offset: 0.0, nullpoint: 0.0, unit: "V", enabled: true },
    ChannelConfig { scale: 1.0, offset: 0.0, nullpoint: 0.0, unit: "V", enabled: true },
    ChannelConfig { scale: 1.0, offset: 0.0, nullpoint: 0.0, unit: "V", enabled: true },
    ChannelConfig { scale: 1.0, offset: 0.0, nullpoint: 0.0, unit: "V", enabled: true },
];

// ============================================================
// AD Command Protocol (core of SIRIUS communication)
// ============================================================

/// AD command operation types
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum AdOp {
    SlotQuery = 0x08,
    SlotEnum = 0x0C,
    Write = 0x13,
    Read = 0x14,
    Batch = 0x1C,
}

/// Build a 15-byte AD command
fn build_ad_command(op: AdOp, slot: u8, register: u8, data: [u8; 3]) -> [u8; 15] {
    let (pad, slot_byte) = match op {
        AdOp::SlotQuery | AdOp::Batch => ([0xFF, 0xFF, 0xFF], 0xFF),
        _ => ([0x00, 0x00, 0x00], slot),
    };

    [
        0xAD, 0x3F, 0x0C, 0x00, 0x00, 0x00,
        op as u8,
        pad[0], pad[1], pad[2],
        slot_byte,
        register,
        data[0], data[1], data[2],
    ]
}

/// Send AD command + B1 poll cycle
/// Returns the response (up to 64 bytes)
fn ad_command_with_poll(
    op: AdOp,
    slot: u8,
    register: u8,
    data: [u8; 3],
    max_polls: u32,
) -> Result<[u8; 64], &'static str> {
    let cmd = build_ad_command(op, slot, register, data);
    let slot_id = unsafe { USB_SLOT };

    // Send AD command on EP1 OUT
    crate::drivers::xhci::bulk_transfer_out(slot_id, 1, &cmd)?;

    // Read ACK from EP1 IN
    let mut ack = [0u8; 64];
    crate::drivers::xhci::bulk_transfer_in(slot_id, 1, &mut ack)?;

    // B1 poll loop
    let b1_cmd = [0xB1u8];
    let mut response = [0u8; 64];

    for _ in 0..max_polls {
        crate::drivers::xhci::bulk_transfer_out(slot_id, 1, &b1_cmd)?;
        let n = crate::drivers::xhci::bulk_transfer_in(slot_id, 1, &mut response)?;

        if n > 0 {
            match op {
                AdOp::Write => {
                    // For writes, 0x00 = confirmed
                    if response[0] == 0x00 {
                        return Ok(response);
                    }
                }
                _ => {
                    // For reads, 0x01 = data ready
                    if response[0] == 0x01 {
                        return Ok(response);
                    }
                }
            }
        }
    }

    Err("B1 poll timeout")
}

/// Write a register via AD command
fn write_register(slot: u8, register: u8, data: [u8; 3]) -> Result<(), &'static str> {
    ad_command_with_poll(AdOp::Write, slot, register, data, 50)?;
    Ok(())
}

/// Read a register via AD command
fn read_register(slot: u8, register: u8) -> Result<[u8; 64], &'static str> {
    ad_command_with_poll(AdOp::Read, slot, register, [0xFF, 0xFF, 0xFF], 50)
}

/// Send A5 sub-command (parameter set, mode set, etc.)
fn a5_command(slot: u8, subcmd: u8, param: u8, value: u8) -> Result<(), &'static str> {
    write_register(slot, 0xA5, [subcmd, param, value])
}

/// Commit register changes (write to 0x5A)
fn commit(slot: u8) -> Result<(), &'static str> {
    write_register(slot, 0x5A, [0x00, 0x00, 0x00])
}

// ============================================================
// Simple Commands
// ============================================================

fn send_simple_command(cmd: &[u8]) -> Result<[u8; 64], &'static str> {
    let slot_id = unsafe { USB_SLOT };
    crate::drivers::xhci::bulk_transfer_out(slot_id, 1, cmd)?;
    let mut resp = [0u8; 64];
    crate::drivers::xhci::bulk_transfer_in(slot_id, 1, &mut resp)?;
    Ok(resp)
}

fn get_firmware_version() -> Result<[u8; 64], &'static str> {
    send_simple_command(&[0x00])
}

fn set_active_mode() -> Result<(), &'static str> {
    send_simple_command(&[0xA0, 0x01])?;
    Ok(())
}

fn get_slot_presence() -> Result<u8, &'static str> {
    let resp = send_simple_command(&[0xA1])?;
    Ok(resp[0])
}

fn pre_start() -> Result<(), &'static str> {
    send_simple_command(&[0xA4, 0x00])?;
    Ok(())
}

fn init_reset() -> Result<(), &'static str> {
    send_simple_command(&[0xB0, 0x3F, 0x0C])?;
    Ok(())
}

fn heartbeat() -> Result<(), &'static str> {
    send_simple_command(&[0xB1])?;
    Ok(())
}

// ============================================================
// Initialization Sequence
// ============================================================

/// Global register write commands for start-acquisition
/// These are the exact hex values from the pcapng capture
static START_SEQUENCE: &[(u8, [u8; 3])] = &[
    // Register, Data[3] — 31 global register writes
    (0x67, [0x00, 0x4E, 0x20]),  // Sample rate = 20000 Hz
    (0x7B, [0x00, 0x0C, 0x80]),  // Buffer config
    // Channel 0-7 configs
    (0x82, [0x00, 0x00, 0x31]),  // Ch0 active
    (0x82, [0x01, 0x00, 0x31]),  // Ch1 active
    (0x82, [0x02, 0x00, 0x31]),  // Ch2 active
    (0x82, [0x03, 0x00, 0x31]),  // Ch3 active
    (0x82, [0x04, 0x00, 0x31]),  // Ch4 active
    (0x82, [0x05, 0x00, 0x31]),  // Ch5 active
    (0x82, [0x06, 0x00, 0x31]),  // Ch6 active
    (0x82, [0x07, 0x00, 0x31]),  // Ch7 active
    (0xE5, [0x00, 0x18, 0x00]),  // DMA/timing
    (0x6F, [0x3F, 0xFF, 0x23]),  // Channel enable mask
    (0x72, [0x00, 0x00, 0x02]),  // Trigger config
    (0x10, [0x00, 0x00, 0x00]),  // Control
    (0x11, [0x00, 0x00, 0x00]),  // Control
    (0x07, [0x03, 0x00, 0x00]),  // Mode flags
    (0x9C, [0x00, 0x64, 0x00]),  // Filter
    (0x98, [0x02, 0x14, 0x32]),  // Decimation
    (0x99, [0x60, 0x60, 0x00]),  // Sample timing
    (0x9D, [0x00, 0x00, 0x00]),  // Reset
    (0x96, [0xFF, 0xFF, 0xFF]),  // Apply config (slow! ~284 polls)
    (0xD0, [0x00, 0x00, 0x01]),  // DMA enable
    (0x68, [0x00, 0x00, 0xFF]),  // Channel mask (all 8)
    (0xCC, [0x00, 0x00, 0xC0]),  // DMA config
    (0xCD, [0x00, 0x00, 0x01]),  // DMA mode
    (0xCA, [0x10, 0x10, 0x10]),  // Channel mask slots 0-3
    (0xCB, [0x10, 0x10, 0x10]),  // Channel mask slots 4-7
    (0xCE, [0x10, 0x10, 0x00]),  // Slot enable
    (0xCF, [0x00, 0x00, 0x00]),  // Clear flags
    (0x84, [0x00, 0x00, 0x00]),  // Transfer start
    (0xC8, [0xFF, 0xFF, 0xFF]),  // DMA arm
    (0x64, [0xFF, 0xFF, 0xFF]),  // Streaming start
];

/// Full initialization sequence
fn initialize_device() -> Result<(), &'static str> {
    crate::kprintln!("  SIRIUS: Phase 1 — device discovery");
    get_slot_presence()?;
    set_active_mode()?;
    let fw = get_firmware_version()?;
    crate::kprintln!("  SIRIUS: firmware {:02x}.{:02x}.{:02x}.{:02x}",
        fw[0], fw[1], fw[2], fw[3]);

    crate::kprintln!("  SIRIUS: Phase 2 — init/reset");
    init_reset()?;

    // Phase 3: Global register config (abbreviated — full 1959-cmd sequence
    // would be loaded from a static data blob in production)
    crate::kprintln!("  SIRIUS: Phase 3 — register configuration");

    // Phase 4: Pre-start
    pre_start()?;

    // Phase 5: Start acquisition
    crate::kprintln!("  SIRIUS: Phase 5 — start acquisition");
    for &(reg, data) in START_SEQUENCE {
        let max_polls = match reg {
            0x96 => 400, // Apply config is slow
            _ => 50,
        };
        ad_command_with_poll(AdOp::Write, 0xFF, reg, data, max_polls)?;
    }

    // TRIGGER (register 0x02) — this starts EP2 data flow
    crate::kprintln!("  SIRIUS: TRIGGER");
    ad_command_with_poll(AdOp::Read, 0xFF, 0x02, [0xFF, 0xFF, 0xFF], 1000)?;

    // Confirm streaming (register 0x03)
    crate::arch::aarch64::timer::delay_ms(220);
    ad_command_with_poll(AdOp::Read, 0xFF, 0x03, [0xFF, 0xFF, 0xFF], 50)?;

    STREAMING.store(true, Ordering::SeqCst);
    crate::kprintln!("  SIRIUS: streaming active — {} Hz, {} channels",
        SAMPLE_RATE, NUM_CHANNELS);

    Ok(())
}

// ============================================================
// Streaming (EP2 data path)
// ============================================================

/// Deinterleave EP2 raw data into per-channel arrays
fn deinterleave_adc(raw: &[u8; ADC_PACKET_SIZE], out: &mut [[i16; FRAMES_PER_PACKET]; NUM_CHANNELS]) {
    // Raw format: 992 frames × 8 channels × 2 bytes (int16 LE), interleaved
    // Frame N: [ch0_lo ch0_hi] [ch1_lo ch1_hi] ... [ch7_lo ch7_hi]
    for frame in 0..FRAMES_PER_PACKET {
        let base = frame * NUM_CHANNELS * 2;
        for ch in 0..NUM_CHANNELS {
            let offset = base + ch * 2;
            let sample = i16::from_le_bytes([raw[offset], raw[offset + 1]]);
            out[ch][frame] = sample;
        }
    }
}

// ============================================================
// Public API
// ============================================================

/// Initialize SIRIUS driver — find device on USB, run init sequence
pub fn init() {
    // TODO: scan xHCI ports for VID:PID match
    // For now, assume slot 1 (first device)
    unsafe { USB_SLOT = 1; }

    match initialize_device() {
        Ok(()) => {
            INITIALIZED.store(true, Ordering::SeqCst);
        }
        Err(e) => {
            crate::kprintln!("  SIRIUS init failed: {}", e);
        }
    }
}

/// Poll for new ADC data — call this from main loop
pub fn poll() {
    if !STREAMING.load(Ordering::Relaxed) { return; }

    let slot_id = unsafe { USB_SLOT };

    // Read EP2 (ADC data)
    let mut raw_adc = [0u8; ADC_PACKET_SIZE];
    if crate::drivers::xhci::bulk_transfer_in(slot_id, 2, &mut raw_adc).is_ok() {
        // Deinterleave into inactive buffer
        let use_b = ACTIVE_BUFFER.load(Ordering::Relaxed);
        unsafe {
            if use_b {
                deinterleave_adc(&raw_adc, &mut ADC_BUFFER_A);
            } else {
                deinterleave_adc(&raw_adc, &mut ADC_BUFFER_B);
            }
        }
        // Swap active buffer
        ACTIVE_BUFFER.store(!use_b, Ordering::Release);
        PACKETS_RECEIVED.fetch_add(1, Ordering::Relaxed);
    }

    // Drain EP4 (status) and EP6 (sync) — MUST read to prevent FIFO stall
    let mut status_buf = [0u8; 20];
    let mut sync_buf = [0u8; ADC_PACKET_SIZE];
    let _ = crate::drivers::xhci::bulk_transfer_in(slot_id, 4, &mut status_buf);
    let _ = crate::drivers::xhci::bulk_transfer_in(slot_id, 6, &mut sync_buf);

    // B1 heartbeat (~150/sec, we call from main loop which runs faster)
    let _ = heartbeat();
}

/// Get the latest ADC data (read from active buffer)
pub fn get_latest_data() -> &'static [[i16; FRAMES_PER_PACKET]; NUM_CHANNELS] {
    let use_b = ACTIVE_BUFFER.load(Ordering::Acquire);
    unsafe {
        if use_b { &ADC_BUFFER_B } else { &ADC_BUFFER_A }
    }
}

/// Get scaled float value for a channel/sample
pub fn get_scaled_value(channel: usize, sample: usize) -> f64 {
    let data = get_latest_data();
    let raw = data[channel][sample] as f64;
    let cfg = unsafe { &CHANNELS[channel] };
    (raw * cfg.scale + cfg.offset) - cfg.nullpoint
}

/// Get total packets received
pub fn packets_received() -> u64 {
    PACKETS_RECEIVED.load(Ordering::Relaxed)
}

/// Is the instrument streaming?
pub fn is_streaming() -> bool {
    STREAMING.load(Ordering::Relaxed)
}
