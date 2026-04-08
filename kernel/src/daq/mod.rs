//! DAQ core — openDAQ Native Streaming (daq.nd://) server
//!
//! Protocol architecture (bifurcated):
//!   Port 4840: OPC UA (device tree, configuration) — future
//!   Port 7420: Native Streaming (raw binary data) — implemented
//!
//! DewesoftX compatibility: locked to protocol v3.20.6

pub mod opendaq;
pub mod signal;

use core::sync::atomic::{AtomicBool, Ordering};
static DAQ_RUNNING: AtomicBool = AtomicBool::new(false);

pub fn init() {
    signal::init_signals();
    opendaq::init();
    DAQ_RUNNING.store(true, Ordering::SeqCst);
}

pub fn poll() {
    if !DAQ_RUNNING.load(Ordering::Relaxed) { return; }
    opendaq::poll();

    // Stream data if connected and SIRIUS is active
    #[cfg(feature = "pi5")]
    if crate::usb::sirius::is_streaming() && opendaq::is_connected() {
        let data = crate::usb::sirius::get_latest_data();
        opendaq::broadcast_data(data);
    }
}
