//! WASM Runtime — Silverfir-nano host function interface
//!
//! This module defines the host functions that WASM apps can call
//! to interact with the DAQ subsystem and kernel services.
//!
//! Unlike Folkering OS (main), this runtime is stripped of all ML/AI
//! functions and focused purely on:
//!   - Sensor data access (ring buffer reads)
//!   - DSP output (processed data writes)
//!   - System timing
//!   - Display output (minimal compositor commands)
//!
//! Silverfir-nano will JIT-compile WASM modules and link these
//! host functions at load time.

pub mod host_functions;
