//! Network stack — smoltcp TCP/IP over RP1 Ethernet
//!
//! Provides:
//! - DHCP client for IP address
//! - TCP server sockets for openDAQ (ports 4840, 7420, 7414)
//! - UDP for mDNS discovery (port 5353)

pub mod socket;
pub mod mdns;

use alloc::vec;
use smoltcp::iface::{Config, Interface, SocketSet};
use smoltcp::wire::{EthernetAddress, IpCidr, Ipv4Address};
use smoltcp::socket::{dhcpv4, tcp, udp};
use smoltcp::time::Instant;
use spin::Mutex;
use core::sync::atomic::{AtomicBool, Ordering};

static NET_READY: AtomicBool = AtomicBool::new(false);

/// Placeholder NIC driver trait
pub trait NetworkDevice {
    fn mac_address(&self) -> EthernetAddress;
    fn send(&mut self, data: &[u8]);
    fn recv(&mut self) -> Option<alloc::vec::Vec<u8>>;
}

/// Timestamp in milliseconds (from ARM timer)
fn now_ms() -> i64 {
    crate::arch::aarch64::timer::millis() as i64
}

/// Initialize network stack
pub fn init() {
    // TODO: Initialize RP1 Ethernet MAC driver
    // TODO: Create smoltcp interface with DHCP
    // TODO: Bind TCP sockets for openDAQ
    // TODO: Bind UDP socket for mDNS

    crate::kprintln!("  Network: waiting for RP1 Ethernet driver...");

    // Placeholder — network init will be completed when RP1 driver is ready
    // For now, mark as not ready
    // NET_READY.store(true, Ordering::SeqCst);
}

/// Poll network stack — process incoming packets, advance TCP state machines
pub fn poll() {
    if !NET_READY.load(Ordering::Relaxed) { return; }

    // TODO: poll smoltcp interface
    // TODO: check for new TCP connections on openDAQ ports
    // TODO: respond to mDNS queries
}
