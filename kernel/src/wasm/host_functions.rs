//! DAQ Host Functions for WASM apps
//!
//! These are the functions that Silverfir-nano exposes to WASM modules.
//! Each function follows the pattern:
//!   (buf_ptr: i32, buf_len: i32, ...) -> i32
//! where the return value is bytes written, or negative on error.
//!
//! === Function Table ===
//!
//! | Name                    | Signature                              | Description                          |
//! |-------------------------|----------------------------------------|--------------------------------------|
//! | folk_daq_read_samples   | (ch: i32, ptr: i32, max: i32) -> i32   | Read f32 samples from ring buffer    |
//! | folk_daq_available      | () -> i32                              | Min available samples across channels|
//! | folk_daq_sample_rate    | () -> i32                              | Current sample rate in Hz            |
//! | folk_daq_channel_count  | () -> i32                              | Number of ADC channels               |
//! | folk_daq_write_output   | (ch: i32, ptr: i32, len: i32) -> i32   | Write processed data to output ring  |
//! | folk_daq_timestamp_ns   | () -> i64                              | Monotonic nanosecond timestamp       |
//! | folk_draw_rect          | (x,y,w,h,rgba: i32) -> i32            | Draw rectangle on compositor         |
//! | folk_draw_text          | (x,y,ptr,len,rgba: i32) -> i32        | Draw text on compositor              |
//! | folk_log                | (ptr: i32, len: i32) -> i32            | Print to kernel serial log           |

use crate::usb::sirius::{SAMPLE_RATE, NUM_CHANNELS};

/// Read f32 samples from a channel's ring buffer into WASM linear memory.
///
/// # Arguments
/// - `channel`: 0 = time domain, 1-8 = ADC channels
/// - `wasm_ptr`: destination pointer in WASM linear memory
/// - `max_samples`: maximum number of f32 values to read
///
/// # Returns
/// - Number of samples actually read (may be 0 if buffer empty)
/// - -1 if channel is out of range
/// - -2 if wasm_ptr is invalid
pub fn folk_daq_read_samples(
    wasm_memory: &mut [u8],
    channel: i32,
    wasm_ptr: i32,
    max_samples: i32,
) -> i32 {
    let channel = channel as usize;
    if channel > NUM_CHANNELS { return -1; }

    let ptr = wasm_ptr as usize;
    let max = max_samples as usize;
    let byte_len = max * 4; // f32 = 4 bytes

    if ptr + byte_len > wasm_memory.len() { return -2; }

    // Read directly into the WASM memory (zero-copy from ring to WASM)
    let out_slice = unsafe {
        core::slice::from_raw_parts_mut(
            wasm_memory.as_mut_ptr().add(ptr) as *mut f32,
            max,
        )
    };

    crate::daq::read_channel_samples(channel, out_slice) as i32
}

/// Get minimum available samples across all channels.
/// WASM apps should call this before folk_daq_read_samples to know
/// how many samples they can safely read from every channel.
pub fn folk_daq_available() -> i32 {
    crate::daq::available_samples() as i32
}

/// Get the current sample rate in Hz.
pub fn folk_daq_sample_rate() -> i32 {
    SAMPLE_RATE as i32
}

/// Get the number of ADC channels (excluding time domain).
pub fn folk_daq_channel_count() -> i32 {
    NUM_CHANNELS as i32
}

/// Write processed/filtered data to an output ring buffer.
/// Used for DSP pipelines where WASM apps filter data and send it
/// back to the kernel for network transmission or further processing.
///
/// # Returns
/// - Number of samples written
/// - -1 if channel out of range
/// - -2 if wasm_ptr invalid
pub fn folk_daq_write_output(
    wasm_memory: &[u8],
    channel: i32,
    wasm_ptr: i32,
    num_samples: i32,
) -> i32 {
    let channel = channel as usize;
    if channel > NUM_CHANNELS { return -1; }

    let ptr = wasm_ptr as usize;
    let count = num_samples as usize;
    let byte_len = count * 4;

    if ptr + byte_len > wasm_memory.len() { return -2; }

    let samples = unsafe {
        core::slice::from_raw_parts(
            wasm_memory.as_ptr().add(ptr) as *const f32,
            count,
        )
    };

    crate::daq::write_channel_output(channel, samples) as i32
}

/// Get a monotonic nanosecond timestamp.
/// Uses the architecture's high-resolution counter.
pub fn folk_daq_timestamp_ns() -> i64 {
    #[cfg(target_arch = "aarch64")]
    {
        let freq = crate::arch::aarch64::counter_freq();
        let val = crate::arch::aarch64::counter_value();
        if freq == 0 { return 0; }
        // ns = val * 1_000_000_000 / freq
        // Use 128-bit arithmetic to avoid overflow
        ((val as u128 * 1_000_000_000) / freq as u128) as i64
    }

    #[cfg(target_arch = "x86_64")]
    {
        // Read TSC (Time Stamp Counter)
        let tsc: u64;
        unsafe {
            core::arch::asm!("rdtsc", out("eax") _, out("edx") _, options(nomem, nostack));
            // Simplified: return raw TSC ticks (calibration needed for real ns)
            core::arch::asm!(
                "rdtsc",
                "shl rdx, 32",
                "or rax, rdx",
                out("rax") tsc,
                out("rdx") _,
                options(nomem, nostack)
            );
        }
        tsc as i64
    }

    #[cfg(not(any(target_arch = "aarch64", target_arch = "x86_64")))]
    { 0i64 }
}

/// Log a message to the kernel serial console.
/// Useful for debugging WASM apps.
pub fn folk_log(wasm_memory: &[u8], wasm_ptr: i32, len: i32) -> i32 {
    let ptr = wasm_ptr as usize;
    let length = len as usize;

    if ptr + length > wasm_memory.len() { return -1; }

    let bytes = &wasm_memory[ptr..ptr + length];
    if let Ok(msg) = core::str::from_utf8(bytes) {
        crate::kprintln!("[WASM] {}", msg);
        length as i32
    } else {
        -1
    }
}

/// Summary of all host functions for registration in the WASM runtime.
///
/// When Silverfir-nano loads a WASM module, it resolves imports by name.
/// This struct provides the mapping from import name → function pointer.
pub struct HostFunctionTable {
    pub entries: &'static [HostFunction],
}

pub struct HostFunction {
    pub module: &'static str,  // "env"
    pub name: &'static str,    // "folk_daq_read_samples"
    pub param_count: u8,       // number of i32/i64 params
    pub has_return: bool,      // whether it returns a value
}

/// All registered host functions
pub static HOST_FUNCTIONS: [HostFunction; 9] = [
    HostFunction { module: "env", name: "folk_daq_read_samples",  param_count: 3, has_return: true },
    HostFunction { module: "env", name: "folk_daq_available",     param_count: 0, has_return: true },
    HostFunction { module: "env", name: "folk_daq_sample_rate",   param_count: 0, has_return: true },
    HostFunction { module: "env", name: "folk_daq_channel_count", param_count: 0, has_return: true },
    HostFunction { module: "env", name: "folk_daq_write_output",  param_count: 3, has_return: true },
    HostFunction { module: "env", name: "folk_daq_timestamp_ns",  param_count: 0, has_return: true },
    HostFunction { module: "env", name: "folk_draw_rect",         param_count: 5, has_return: true },
    HostFunction { module: "env", name: "folk_draw_text",         param_count: 5, has_return: true },
    HostFunction { module: "env", name: "folk_log",               param_count: 2, has_return: true },
];
