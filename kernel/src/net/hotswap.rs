//! Network Hot-Swap for WASM Modules
//!
//! Listens on TCP port 7421 for incoming WASM binaries.
//! Protocol (dead simple):
//!   1. Client connects to port 7421
//!   2. Client sends: [4-byte LE length][WASM binary bytes]
//!   3. Server loads with wasmi interpreter
//!   4. Server responds: "OK\n" or "ERR: <reason>\n"
//!   5. If OK, the running DSP module is hot-swapped atomically
//!
//! The data flow (ring buffer → WASM → output) is never interrupted.
//! During compilation, the old module continues to run. The swap happens
//! between two tick() calls, so no samples are lost.
//!
//! Usage from host machine:
//!   cat lowpass.wasm | nc <daq-ip> 7421
//!   # or with Python:
//!   sock.sendall(len(wasm).to_bytes(4, 'little') + wasm)

use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, Ordering};

/// Maximum WASM binary size we'll accept (256 KiB)
const MAX_WASM_SIZE: usize = 256 * 1024;

/// Port for WASM hot-swap
pub const HOTSWAP_PORT: u16 = 7421;

/// Receive state machine
enum RxState {
    /// Waiting for the 4-byte length header
    WaitingHeader,
    /// Receiving WASM binary data
    ReceivingData {
        expected: usize,
        buffer: Vec<u8>,
    },
}

static mut RX_STATE: RxState = RxState::WaitingHeader;
static mut HEADER_BUF: [u8; 4] = [0; 4];
static mut HEADER_POS: usize = 0;

/// Pending WASM module ready for swap (set by network handler, consumed by main loop)
static SWAP_PENDING: AtomicBool = AtomicBool::new(false);
static mut PENDING_WASM: Option<Vec<u8>> = None;

/// Called from the network poll loop when data arrives on the hotswap socket.
pub fn on_data_received(data: &[u8]) {
    unsafe {
        let mut offset = 0;

        while offset < data.len() {
            match &mut RX_STATE {
                RxState::WaitingHeader => {
                    // Accumulate 4 header bytes
                    while HEADER_POS < 4 && offset < data.len() {
                        HEADER_BUF[HEADER_POS] = data[offset];
                        HEADER_POS += 1;
                        offset += 1;
                    }

                    if HEADER_POS >= 4 {
                        let expected = u32::from_le_bytes(HEADER_BUF) as usize;
                        HEADER_POS = 0;

                        if expected == 0 || expected > MAX_WASM_SIZE {
                            crate::kprintln!("  HOTSWAP: rejected (size={}, max={})", expected, MAX_WASM_SIZE);
                            RX_STATE = RxState::WaitingHeader;
                            return;
                        }

                        crate::kprintln!("  HOTSWAP: receiving {} bytes WASM", expected);
                        RX_STATE = RxState::ReceivingData {
                            expected,
                            buffer: Vec::with_capacity(expected),
                        };
                    }
                }

                RxState::ReceivingData { expected, buffer } => {
                    let remaining = *expected - buffer.len();
                    let chunk = remaining.min(data.len() - offset);
                    buffer.extend_from_slice(&data[offset..offset + chunk]);
                    offset += chunk;

                    if buffer.len() >= *expected {
                        crate::kprintln!("  HOTSWAP: received complete WASM ({} bytes)", buffer.len());

                        // Move the buffer to pending
                        let wasm_data = core::mem::replace(buffer, Vec::new());
                        PENDING_WASM = Some(wasm_data);
                        SWAP_PENDING.store(true, Ordering::Release);

                        // Reset state for next upload
                        RX_STATE = RxState::WaitingHeader;
                    }
                }
            }
        }
    }
}

/// Check if there's a pending WASM module to swap in.
/// Called from the main loop. Returns the WASM bytes if available.
pub fn take_pending_wasm() -> Option<Vec<u8>> {
    if !SWAP_PENDING.load(Ordering::Acquire) {
        return None;
    }

    SWAP_PENDING.store(false, Ordering::Release);
    unsafe { PENDING_WASM.take() }
}

/// Send a response back to the client.
/// Called after compilation succeeds or fails.
pub fn send_response(success: bool, message: &str) {
    let response = if success {
        alloc::format!("OK: {}\n", message)
    } else {
        alloc::format!("ERR: {}\n", message)
    };

    let _ = crate::net::tcp_send_on_port(HOTSWAP_PORT, response.as_bytes());
}

/// Reset the receive state (called on disconnect)
pub fn on_disconnect() {
    unsafe {
        RX_STATE = RxState::WaitingHeader;
        HEADER_POS = 0;
    }
}
