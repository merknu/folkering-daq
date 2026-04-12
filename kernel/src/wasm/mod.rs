//! WASM Runtime for Folkering DAQ
//!
//! Uses wasmi (industry-standard WASM interpreter) for correct,
//! spec-compliant execution of DSP apps.
//!
//! Host functions (folk_daq_*) bridge between WASM apps and the
//! kernel's ring buffer / sensor subsystem.

pub mod runtime;
pub mod host_functions;
