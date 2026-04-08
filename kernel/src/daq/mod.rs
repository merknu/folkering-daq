//! DAQ core — openDAQ protocol server + signal management
//!
//! Implements the openDAQ Native Streaming protocol (port 7420)
//! so that DewesoftX can discover and stream from this device.

pub mod opendaq;
pub mod signal;

use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, Ordering};

static DAQ_RUNNING: AtomicBool = AtomicBool::new(false);

pub fn init() {
    // Initialize signal descriptors for 8 SIRIUS channels
    signal::init_signals();

    // Start openDAQ server (will listen when network is ready)
    opendaq::init();

    DAQ_RUNNING.store(true, Ordering::SeqCst);
}

pub fn poll() {
    if !DAQ_RUNNING.load(Ordering::Relaxed) { return; }

    // Check for new openDAQ client connections
    opendaq::poll();

    // If streaming, push latest ADC data to subscribers
    if crate::usb::sirius::is_streaming() {
        let data = crate::usb::sirius::get_latest_data();
        opendaq::broadcast_data(data);
    }
}
