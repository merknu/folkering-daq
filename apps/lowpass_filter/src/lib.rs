//! Low-Pass Filter — Example DSP WASM App for Folkering DAQ
//!
//! Reads raw ADC samples from the ring buffer via folk_daq_read_samples,
//! applies a first-order IIR (Infinite Impulse Response) low-pass filter,
//! and writes filtered samples back via folk_daq_write_output.
//!
//! Filter: y[n] = α * x[n] + (1 - α) * y[n-1]
//!   where α = dt / (RC + dt)
//!   dt = 1 / sample_rate
//!   RC = 1 / (2π * cutoff_frequency)
//!
//! Default cutoff: 1000 Hz (configurable via WASM linear memory)
//!
//! Exports:
//!   init() — called once at load, calculates filter coefficients
//!   tick() — called every main loop iteration, processes available samples
//!
//! Build:
//!   cargo build --target wasm32-unknown-unknown --release
//!   # Output: target/wasm32-unknown-unknown/release/lowpass_filter.wasm
//!
//! Deploy:
//!   cat lowpass_filter.wasm | python3 -c "
//!     import sys, socket, struct
//!     w = open(sys.argv[1],'rb').read() if len(sys.argv)>1 else sys.stdin.buffer.read()
//!     s = socket.create_connection(('192.168.1.100', 7421))
//!     s.sendall(struct.pack('<I', len(w)) + w)
//!     print(s.recv(256).decode())
//!   " lowpass_filter.wasm

#![no_std]

// === Host Function Imports ===
// These are provided by the Folkering DAQ kernel via Silverfir-nano

extern "C" {
    /// Read f32 samples from a DAQ channel's ring buffer
    /// channel: 0=time, 1-8=ADC
    /// Returns: number of samples read
    fn folk_daq_read_samples(channel: i32, buf_ptr: i32, max_samples: i32) -> i32;

    /// Get minimum available samples across all channels
    fn folk_daq_available() -> i32;

    /// Get sample rate in Hz
    fn folk_daq_sample_rate() -> i32;

    /// Get number of ADC channels
    fn folk_daq_channel_count() -> i32;

    /// Write processed samples to output
    fn folk_daq_write_output(channel: i32, buf_ptr: i32, num_samples: i32) -> i32;

    /// Log message to kernel serial console
    fn folk_log(ptr: i32, len: i32) -> i32;
}

// === Constants ===

/// Maximum samples we process per tick (batch size)
const BATCH_SIZE: usize = 512;

/// Default cutoff frequency in Hz
const DEFAULT_CUTOFF_HZ: f32 = 1000.0;

/// Pi constant (no std::f32::consts available)
const PI: f32 = 3.14159265358979323846;

// === Filter State ===
// Stored in WASM linear memory (persists between tick() calls)

/// Filter coefficient α (calculated in init())
static mut ALPHA: f32 = 0.0;

/// Previous output per channel (y[n-1])
static mut PREV_OUTPUT: [f32; 8] = [0.0; 8];

/// Number of active channels
static mut NUM_CHANNELS: i32 = 0;

/// Input buffer (shared across channels, processed sequentially)
static mut INPUT_BUF: [f32; BATCH_SIZE] = [0.0; BATCH_SIZE];

/// Output buffer
static mut OUTPUT_BUF: [f32; BATCH_SIZE] = [0.0; BATCH_SIZE];

/// Total samples processed (for telemetry)
static mut TOTAL_PROCESSED: u64 = 0;

// === Exported Functions ===

/// Initialize the filter. Called once when the module is loaded.
#[no_mangle]
pub extern "C" fn init() -> i32 {
    unsafe {
        // Get system parameters from host
        let sample_rate = folk_daq_sample_rate() as f32;
        NUM_CHANNELS = folk_daq_channel_count();

        // Calculate filter coefficient
        // α = dt / (RC + dt)
        // where dt = 1/fs, RC = 1/(2π·fc)
        let dt = 1.0 / sample_rate;
        let rc = 1.0 / (2.0 * PI * DEFAULT_CUTOFF_HZ);
        ALPHA = dt / (rc + dt);

        // Reset filter state
        for i in 0..8 {
            PREV_OUTPUT[i] = 0.0;
        }
        TOTAL_PROCESSED = 0;

        // Log initialization
        let msg = b"LPF: initialized";
        folk_log(msg.as_ptr() as i32, msg.len() as i32);
    }
    0 // success
}

/// Process one tick's worth of samples.
/// Called every iteration of the kernel main loop.
#[no_mangle]
pub extern "C" fn tick() -> i32 {
    unsafe {
        // Check how many samples are available
        let available = folk_daq_available();
        if available <= 0 {
            return 0; // Nothing to do
        }

        // Process up to BATCH_SIZE samples at a time
        let to_process = if available > BATCH_SIZE as i32 {
            BATCH_SIZE as i32
        } else {
            available
        };

        // Process each channel
        let alpha = ALPHA;
        let one_minus_alpha = 1.0 - alpha;

        for ch in 0..NUM_CHANNELS {
            let ch_idx = (ch + 1) as usize; // +1 because channel 0 is time domain

            // Read raw samples from ring buffer
            let read = folk_daq_read_samples(
                ch + 1, // channel index (1-based for ADC)
                INPUT_BUF.as_ptr() as i32,
                to_process,
            );

            if read <= 0 { continue; }

            // Apply IIR low-pass filter
            let mut y_prev = PREV_OUTPUT[ch as usize];

            for i in 0..read as usize {
                // y[n] = α * x[n] + (1 - α) * y[n-1]
                let x = INPUT_BUF[i];
                let y = alpha * x + one_minus_alpha * y_prev;
                OUTPUT_BUF[i] = y;
                y_prev = y;
            }

            // Save state for next tick
            PREV_OUTPUT[ch as usize] = y_prev;

            // Write filtered samples to output
            folk_daq_write_output(
                ch + 1,
                OUTPUT_BUF.as_ptr() as i32,
                read,
            );
        }

        TOTAL_PROCESSED += to_process as u64;
    }
    0 // success
}

// === Panic handler (required for no_std) ===

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
