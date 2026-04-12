//! DAQ core — openDAQ Native Streaming (daq.nd://) server
//!
//! Protocol architecture (bifurcated):
//!   Port 4840: OPC UA (device tree, configuration) — future
//!   Port 7420: Native Streaming (raw binary data) — implemented
//!
//! Data pipeline:
//!   USB ISR → SPSC Ring Buffer → { TCP broadcast (DewesoftX), WASM consumer (wasmi) }
//!
//! The ring buffer is the single source of truth for sensor data.
//! Both the network protocol and local WASM apps read from it.
//!
//! DewesoftX compatibility: locked to protocol v3.20.6

pub mod opendaq;
pub mod signal;
pub mod ringbuffer;

use core::sync::atomic::{AtomicBool, Ordering};
use ringbuffer::{DaqRingManager, DEFAULT_CAPACITY};
use crate::usb::sirius::{FRAMES_PER_PACKET, NUM_CHANNELS};

static DAQ_RUNNING: AtomicBool = AtomicBool::new(false);

/// Global ring buffer managers — initialized once at boot
/// INPUT: USB ISR writes raw ADC data here, WASM reads from here
/// OUTPUT: WASM writes filtered/processed data here, network reads from here
static mut RING_MANAGER: Option<DaqRingManager> = None;
static mut OUTPUT_RING_MANAGER: Option<DaqRingManager> = None;

/// Memory region for ring buffers (statically allocated to avoid heap fragmentation)
/// 9 channels × (128 header + 8192 × 4 data) = 9 × 32896 = ~296 KiB
const RING_BUF_SIZE: usize = DaqRingManager::total_bytes(NUM_CHANNELS, DEFAULT_CAPACITY);

#[repr(C, align(64))] // Cache-line aligned
struct RingMemory([u8; RING_BUF_SIZE]);

static mut RING_MEMORY: RingMemory = RingMemory([0u8; RING_BUF_SIZE]);
static mut OUTPUT_RING_MEMORY: RingMemory = RingMemory([0u8; RING_BUF_SIZE]);

pub fn init() {
    signal::init_signals();
    opendaq::init();

    // Initialize ring buffers from static memory
    unsafe {
        RING_MANAGER = Some(DaqRingManager::init(
            RING_MEMORY.0.as_mut_ptr(),
            NUM_CHANNELS,
            DEFAULT_CAPACITY,
        ));
        OUTPUT_RING_MANAGER = Some(DaqRingManager::init(
            OUTPUT_RING_MEMORY.0.as_mut_ptr(),
            NUM_CHANNELS,
            DEFAULT_CAPACITY,
        ));
    }

    crate::kprintln!("  DAQ: Ring buffers initialized ({}+{} KiB, {} ch × input/output)",
        RING_BUF_SIZE / 1024, RING_BUF_SIZE / 1024, NUM_CHANNELS + 1);

    DAQ_RUNNING.store(true, Ordering::SeqCst);
}

pub fn poll() {
    if !DAQ_RUNNING.load(Ordering::Relaxed) { return; }
    opendaq::poll();

    // Stream data if connected and SIRIUS is active
    #[cfg(feature = "pi5")]
    if crate::usb::sirius::is_streaming() {
        let data = crate::usb::sirius::get_latest_data();

        // Step 1: Push into ring buffer (always, even if no TCP client)
        push_to_ringbuffer(data);

        // Step 2: Forward to TCP if DewesoftX is connected
        if opendaq::is_connected() {
            opendaq::broadcast_data(data);
        }
    }
}

/// Push raw ADC data into the ring buffers.
/// Called from the main loop after the USB ISR delivers a frame.
fn push_to_ringbuffer(data: &[[i16; FRAMES_PER_PACKET]; NUM_CHANNELS]) {
    let mgr = unsafe {
        match &RING_MANAGER {
            Some(m) => m,
            None => return,
        }
    };

    // Convert i16 ADC samples to f32 and push into per-channel ring buffers
    // We use a small stack buffer to avoid heap allocation in the hot path
    let mut f32_buf = [0.0f32; FRAMES_PER_PACKET];

    for ch in 0..NUM_CHANNELS {
        // Convert i16 → f32 (normalized to [-1.0, 1.0])
        for i in 0..FRAMES_PER_PACKET {
            f32_buf[i] = data[ch][i] as f32 / 32768.0;
        }
        // Channel index +1 because index 0 is the time domain
        mgr.produce_channel(ch + 1, &f32_buf[..FRAMES_PER_PACKET]);
    }

    // Time domain: monotonic sample counter as f32
    // (WASM apps can use this to calculate elapsed time from sample_rate)
    static mut SAMPLE_COUNTER: u64 = 0;
    unsafe {
        for i in 0..FRAMES_PER_PACKET {
            f32_buf[i] = (SAMPLE_COUNTER + i as u64) as f32;
        }
        SAMPLE_COUNTER += FRAMES_PER_PACKET as u64;
    }
    mgr.produce_channel(0, &f32_buf[..FRAMES_PER_PACKET]);
}

// === Public API for WASM host functions ===

/// Read samples from a channel's ring buffer.
/// Returns number of samples actually read.
pub fn read_channel_samples(channel: usize, out: &mut [f32]) -> usize {
    let mgr = unsafe {
        match &RING_MANAGER {
            Some(m) => m,
            None => return 0,
        }
    };
    mgr.consume_channel(channel, out)
}

/// Get available sample count (minimum across all channels)
pub fn available_samples() -> u32 {
    let mgr = unsafe {
        match &RING_MANAGER {
            Some(m) => m,
            None => return 0,
        }
    };
    mgr.min_available()
}

// === Output Ring Buffer API (WASM writes, network reads) ===

/// Write processed/filtered samples to the output ring buffer.
/// Called by WASM host function folk_daq_write_output.
pub fn write_channel_output(channel: usize, samples: &[f32]) -> usize {
    let mgr = unsafe {
        match &OUTPUT_RING_MANAGER {
            Some(m) => m,
            None => return 0,
        }
    };
    mgr.produce_channel(channel, samples)
}

/// Read processed samples from the output ring buffer.
/// Called by the network layer for retransmission of filtered data.
pub fn read_output_samples(channel: usize, out: &mut [f32]) -> usize {
    let mgr = unsafe {
        match &OUTPUT_RING_MANAGER {
            Some(m) => m,
            None => return 0,
        }
    };
    mgr.consume_channel(channel, out)
}

/// Get available processed samples in the output ring buffer.
pub fn output_available() -> u32 {
    let mgr = unsafe {
        match &OUTPUT_RING_MANAGER {
            Some(m) => m,
            None => return 0,
        }
    };
    mgr.min_available()
}

/// Get the base pointer of the input ring buffer memory region
pub fn ring_memory_base() -> *const u8 {
    unsafe { RING_MEMORY.0.as_ptr() }
}

/// Get the total size of the ring buffer memory region
pub fn ring_memory_size() -> usize {
    RING_BUF_SIZE
}
