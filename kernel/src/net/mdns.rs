//! mDNS responder for openDAQ service discovery
//!
//! Announces:
//!   _opendaq-streaming-native._tcp.local  port 7420
//!
//! TXT records:
//!   caps=OPENDAQ_NS
//!   name=Folkering DAQ
//!   manufacturer=Dewesoft
//!   model=SIRIUSi-HS
//!   serialNumber=D019274CF6

use alloc::format;
use alloc::vec::Vec;

const MDNS_PORT: u16 = 5353;
const MDNS_MULTICAST: [u8; 4] = [224, 0, 0, 251];

/// Service info for mDNS announcement
pub struct ServiceInfo {
    pub name: &'static str,
    pub service_type: &'static str,  // e.g. "_opendaq-streaming-native._tcp"
    pub port: u16,
    pub txt_records: &'static [(&'static str, &'static str)],
}

/// Default openDAQ service
pub static OPENDAQ_SERVICE: ServiceInfo = ServiceInfo {
    name: "Folkering DAQ",
    service_type: "_opendaq-streaming-native._tcp",
    port: 7420,
    txt_records: &[
        ("caps", "OPENDAQ_NS"),
        ("path", "/"),
        ("manufacturer", "Dewesoft"),
        ("model", "SIRIUSi-HS"),
        ("serialNumber", "D019274CF6"),
    ],
};

/// Build mDNS response packet for a PTR query
pub fn build_response(service: &ServiceInfo, our_ip: [u8; 4]) -> Vec<u8> {
    let mut packet = Vec::with_capacity(512);

    // DNS header (response, authoritative)
    packet.extend_from_slice(&[
        0x00, 0x00, // Transaction ID
        0x84, 0x00, // Flags: response, authoritative
        0x00, 0x00, // Questions: 0
        0x00, 0x01, // Answers: 1 (PTR)
        0x00, 0x02, // Authority: 2 (SRV + TXT)
        0x00, 0x01, // Additional: 1 (A record)
    ]);

    // PTR record: _opendaq-streaming-native._tcp.local → Folkering DAQ._opendaq-...
    // (simplified — full mDNS encoding would use DNS name compression)
    let svc_name = format!("{}._tcp.local", service.service_type.trim_start_matches('_'));

    // For a proper implementation, we'd encode DNS names with length-prefixed labels.
    // This is a placeholder — the full mDNS packet builder will be implemented
    // when the UDP socket layer is ready.

    // SRV record: points to host + port
    // TXT record: key=value pairs
    // A record: hostname → IP

    packet
}

/// Handle incoming mDNS query packet
pub fn handle_query(packet: &[u8], our_ip: [u8; 4]) -> Option<Vec<u8>> {
    // Check if this is a query for our service type
    if packet.len() < 12 { return None; }

    let flags = u16::from_be_bytes([packet[2], packet[3]]);
    if flags & 0x8000 != 0 { return None; } // Not a query

    let questions = u16::from_be_bytes([packet[4], packet[5]]);
    if questions == 0 { return None; }

    // Check if the query matches our service type
    // (simplified string search — proper implementation parses DNS labels)
    let query_str = core::str::from_utf8(packet).unwrap_or("");
    if query_str.contains("opendaq") || query_str.contains("_services._dns-sd") {
        return Some(build_response(&OPENDAQ_SERVICE, our_ip));
    }

    None
}
