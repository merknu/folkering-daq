//! Signal descriptors for openDAQ protocol
//!
//! Signals are STATIC — initialized once at boot, never modified.
//! DewesoftX caches the device tree and crashes on runtime changes.
//!
//! valueType mapping (DewesoftX v3.20.6):
//!   3 = Int32 (two's complement)
//!   4 = Float32 (IEEE 754)

use crate::usb::sirius::{SAMPLE_RATE, NUM_CHANNELS};

/// Signal metadata (compile-time static)
pub struct SignalInfo {
    pub name: &'static str,
    pub public_id: &'static str,
    pub value_type: u8,     // 3=Int32, 4=Float32
    pub sample_rate: u32,
    pub unit: &'static str,
}

/// All signals (8 ADC channels + 1 time domain)
pub static SIGNALS: [SignalInfo; NUM_CHANNELS + 1] = [
    // Time domain (index 0) — must be declared BEFORE value signals
    SignalInfo { name: "Time", public_id: "sig_time", value_type: 3, sample_rate: SAMPLE_RATE, unit: "ticks" },
    // ADC channels (indices 1-8)
    SignalInfo { name: "Channel_0", public_id: "sig_ch0", value_type: 3, sample_rate: SAMPLE_RATE, unit: "V" },
    SignalInfo { name: "Channel_1", public_id: "sig_ch1", value_type: 3, sample_rate: SAMPLE_RATE, unit: "V" },
    SignalInfo { name: "Channel_2", public_id: "sig_ch2", value_type: 3, sample_rate: SAMPLE_RATE, unit: "V" },
    SignalInfo { name: "Channel_3", public_id: "sig_ch3", value_type: 3, sample_rate: SAMPLE_RATE, unit: "V" },
    SignalInfo { name: "Channel_4", public_id: "sig_ch4", value_type: 3, sample_rate: SAMPLE_RATE, unit: "V" },
    SignalInfo { name: "Channel_5", public_id: "sig_ch5", value_type: 3, sample_rate: SAMPLE_RATE, unit: "V" },
    SignalInfo { name: "Channel_6", public_id: "sig_ch6", value_type: 3, sample_rate: SAMPLE_RATE, unit: "V" },
    SignalInfo { name: "Channel_7", public_id: "sig_ch7", value_type: 3, sample_rate: SAMPLE_RATE, unit: "V" },
];

pub fn init_signals() {
    // Signals are compile-time static — nothing to initialize
    // Kept for API compatibility with boot sequence
}
