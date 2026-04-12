//! DAQ Host Functions — wasmi linker registrations
//!
//! Registers all folk_daq_* functions with the wasmi Linker so that
//! WASM modules can import and call them.
//!
//! Each function follows the pattern:
//!   linker.func_wrap("env", "folk_daq_*", |caller: Caller<DaqHostState>, ...| { ... })
//!
//! Linear memory is accessed via caller.get_export("memory").

extern crate alloc;

use alloc::string::String;
use wasmi::*;
use super::runtime::DaqHostState;
use crate::usb::sirius::{SAMPLE_RATE, NUM_CHANNELS};

/// Register all DAQ host functions with the wasmi linker.
pub fn register_daq_functions(linker: &mut Linker<DaqHostState>) {
    // ── folk_daq_read_samples(channel: i32, buf_ptr: i32, max_samples: i32) -> i32 ──
    linker.func_wrap("env", "folk_daq_read_samples",
        |mut caller: Caller<DaqHostState>, channel: i32, buf_ptr: i32, max_samples: i32| -> i32 {
            let ch = channel as usize;
            if ch > NUM_CHANNELS { return -1; }

            let max = max_samples as usize;
            let ptr = buf_ptr as usize;

            // Read from ring buffer into a temp buffer
            let mut temp = [0.0f32; 512];
            let to_read = max.min(512);
            let read = crate::daq::read_channel_samples(ch, &mut temp[..to_read]);

            if read > 0 {
                // Write f32 data to WASM linear memory
                let mem = match caller.get_export("memory") {
                    Some(Extern::Memory(m)) => m,
                    _ => return -2,
                };
                for i in 0..read {
                    let bytes = temp[i].to_le_bytes();
                    let _ = mem.write(&mut caller, ptr + i * 4, &bytes);
                }
            }

            read as i32
        }
    ).unwrap();

    // ── folk_daq_available() -> i32 ──
    linker.func_wrap("env", "folk_daq_available",
        |_caller: Caller<DaqHostState>| -> i32 {
            crate::daq::available_samples() as i32
        }
    ).unwrap();

    // ── folk_daq_sample_rate() -> i32 ──
    linker.func_wrap("env", "folk_daq_sample_rate",
        |_caller: Caller<DaqHostState>| -> i32 {
            SAMPLE_RATE as i32
        }
    ).unwrap();

    // ── folk_daq_channel_count() -> i32 ──
    linker.func_wrap("env", "folk_daq_channel_count",
        |_caller: Caller<DaqHostState>| -> i32 {
            NUM_CHANNELS as i32
        }
    ).unwrap();

    // ── folk_daq_write_output(channel: i32, buf_ptr: i32, num_samples: i32) -> i32 ──
    linker.func_wrap("env", "folk_daq_write_output",
        |caller: Caller<DaqHostState>, channel: i32, buf_ptr: i32, num_samples: i32| -> i32 {
            let ch = channel as usize;
            if ch > NUM_CHANNELS { return -1; }

            let count = num_samples as usize;
            let ptr = buf_ptr as usize;
            let to_write = count.min(512);

            let mem = match caller.get_export("memory") {
                Some(Extern::Memory(m)) => m,
                _ => return -2,
            };

            // Read f32 data from WASM memory
            let mut temp = [0.0f32; 512];
            for i in 0..to_write {
                let mut b = [0u8; 4];
                let _ = mem.read(&caller, ptr + i * 4, &mut b);
                temp[i] = f32::from_le_bytes(b);
            }

            crate::daq::write_channel_output(ch, &temp[..to_write]) as i32
        }
    ).unwrap();

    // ── folk_daq_timestamp_ns() -> i64 ──
    linker.func_wrap("env", "folk_daq_timestamp_ns",
        |_caller: Caller<DaqHostState>| -> i64 {
            #[cfg(target_arch = "aarch64")]
            {
                let freq = crate::arch::aarch64::counter_freq();
                let val = crate::arch::aarch64::counter_value();
                if freq == 0 { return 0; }
                ((val as u128 * 1_000_000_000) / freq as u128) as i64
            }
            #[cfg(not(target_arch = "aarch64"))]
            { 0i64 }
        }
    ).unwrap();

    // ── folk_log(ptr: i32, len: i32) -> i32 ──
    linker.func_wrap("env", "folk_log",
        |mut caller: Caller<DaqHostState>, ptr: i32, len: i32| -> i32 {
            let p = ptr as usize;
            let l = len as usize;

            let mem = match caller.get_export("memory") {
                Some(Extern::Memory(m)) => m,
                _ => return -1,
            };

            if p + l > mem.data_size(&caller) { return -1; }

            let mut buf = alloc::vec![0u8; l];
            if mem.read(&caller, p, &mut buf).is_ok() {
                // Convert bytes to string (UTF-8)
                match alloc::string::String::from_utf8(buf) {
                    Ok(msg) => {
                        caller.data_mut().log_messages.push(msg);
                        return l as i32;
                    }
                    Err(_) => return -1,
                }
            }
            -1
        }
    ).unwrap();
}
