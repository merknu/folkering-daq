//! Signal management — describes ADC channels as openDAQ signals
//!
//! Each SIRIUS channel becomes an openDAQ signal with:
//! - Data descriptor (Int16 or Float64, sample rate, unit)
//! - Domain descriptor (linear time, 1MHz tick resolution)

use alloc::string::String;
use alloc::vec::Vec;

/// Signal descriptor for openDAQ protocol
#[derive(Clone)]
pub struct SignalDescriptor {
    pub id: String,             // e.g. "/device/io/ch0/value"
    pub name: String,           // e.g. "Channel 0"
    pub numeric_id: u32,        // Server-assigned numeric ID
    pub sample_type: SampleType,
    pub sample_rate: u32,
    pub unit: String,
    pub has_domain: bool,
}

#[derive(Clone, Copy)]
pub enum SampleType {
    Int16,
    Float64,
}

/// Domain signal (time axis)
pub struct DomainDescriptor {
    pub tick_resolution_num: u64,   // 1
    pub tick_resolution_den: u64,   // 1_000_000 (1 MHz)
    pub rule: DomainRule,
}

pub enum DomainRule {
    Linear { delta: u64, start: u64 },
}

/// All signal descriptors
static mut SIGNALS: Vec<SignalDescriptor> = Vec::new();
static mut DOMAIN: Option<DomainDescriptor> = None;

pub fn init_signals() {
    // Create 8 ADC channel signals
    let mut signals = Vec::with_capacity(9);

    for ch in 0..8 {
        signals.push(SignalDescriptor {
            id: alloc::format!("/device/io/ch{}/value", ch),
            name: alloc::format!("Channel {}", ch),
            numeric_id: (ch + 1) as u32,
            sample_type: SampleType::Int16,
            sample_rate: crate::usb::sirius::SAMPLE_RATE,
            unit: String::from("V"),
            has_domain: true,
        });
    }

    // Domain signal (shared time axis)
    let domain = DomainDescriptor {
        tick_resolution_num: 1,
        tick_resolution_den: 1_000_000, // 1 MHz
        rule: DomainRule::Linear {
            delta: 50, // 1_000_000 / 20_000 = 50 ticks per sample
            start: 0,
        },
    };

    unsafe {
        SIGNALS = signals;
        DOMAIN = Some(domain);
    }
}

pub fn get_signals() -> &'static [SignalDescriptor] {
    unsafe { &SIGNALS }
}

pub fn get_domain() -> Option<&'static DomainDescriptor> {
    unsafe { DOMAIN.as_ref() }
}

/// Serialize signal descriptor to openDAQ JSON format
/// Used in SignalAvailable messages
pub fn serialize_signal_json(signal: &SignalDescriptor) -> String {
    // openDAQ serialization format uses "__type" markers
    alloc::format!(
        r#"{{"__type":"Signal","globalId":"{}","name":"{}","dataDescriptor":{{"__type":"DataDescriptor","sampleType":"{}","rule":{{"__type":"ExplicitRule"}},"dimensions":[]}},"domainSignalId":"/device/io/time"}}"#,
        signal.id,
        signal.name,
        match signal.sample_type {
            SampleType::Int16 => "Int16",
            SampleType::Float64 => "Float64",
        }
    )
}
