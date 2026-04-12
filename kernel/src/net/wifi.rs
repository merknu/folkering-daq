//! WiFi Network Integration — CYW43455 ↔ smoltcp bridge
//!
//! Implements smoltcp's phy::Device trait for the CYW43455 WiFi chip.
//! This allows the existing TCP/IP stack (openDAQ, hot-swap, TCP shell)
//! to work over WiFi transparently — same ports, same protocols.
//!
//! Architecture:
//!   cyw43::send_frame() ← smoltcp TX path (Ethernet frames)
//!   cyw43::recv_frame() → smoltcp RX path (Ethernet frames)
//!
//! The CYW43455 is a FullMAC chip — it handles 802.11 association,
//! WPA2 encryption, and roaming internally. We only see Ethernet frames.

use smoltcp::phy;
use smoltcp::time::Instant;

/// smoltcp Device adapter for CYW43455 WiFi
pub struct WiFiDevice;

pub struct WiFiRxToken;
pub struct WiFiTxToken;

impl phy::Device for WiFiDevice {
    type RxToken<'a> = WiFiRxToken;
    type TxToken<'a> = WiFiTxToken;

    fn receive(&mut self, _timestamp: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        // Check if a frame is available from the WiFi chip
        // Quick peek: try to read status without actually consuming the frame
        if crate::drivers::cyw43::state() != crate::drivers::cyw43::WiFiState::Connected {
            return None;
        }

        // Return tokens — actual read/write happens in consume()
        Some((WiFiRxToken, WiFiTxToken))
    }

    fn transmit(&mut self, _timestamp: Instant) -> Option<Self::TxToken<'_>> {
        if crate::drivers::cyw43::state() != crate::drivers::cyw43::WiFiState::Connected {
            return None;
        }
        Some(WiFiTxToken)
    }

    fn capabilities(&self) -> phy::DeviceCapabilities {
        let mut caps = phy::DeviceCapabilities::default();
        caps.medium = phy::Medium::Ethernet;
        caps.max_transmission_unit = 1500; // Standard Ethernet MTU
        caps.max_burst_size = Some(1);
        caps
    }
}

impl phy::RxToken for WiFiRxToken {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        let mut buf = [0u8; 2048];
        match crate::drivers::cyw43::recv_frame(&mut buf) {
            Some(len) => f(&buf[..len]),
            None => f(&[]),
        }
    }
}

impl phy::TxToken for WiFiTxToken {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let mut buf = [0u8; 2048];
        let actual_len = len.min(buf.len());
        let result = f(&mut buf[..actual_len]);
        let _ = crate::drivers::cyw43::send_frame(&buf[..actual_len]);
        result
    }
}

/// Initialize WiFi and connect to a network.
/// After this, the WiFi device can be used with smoltcp.
pub fn init_and_connect(
    firmware: &[u8],
    nvram: &[u8],
    ssid: &str,
    password: &str,
) -> Result<WiFiDevice, &'static str> {
    crate::kprintln!("  WiFi: initializing CYW43455...");

    crate::drivers::cyw43::init(firmware, nvram)?;

    crate::kprintln!("  WiFi: connecting to '{}'...", ssid);
    crate::drivers::cyw43::join(ssid, password)?;

    crate::kprintln!("[OK] WiFi connected to '{}'", ssid);

    Ok(WiFiDevice)
}
