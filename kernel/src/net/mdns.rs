//! mDNS / DNS-SD Responder for openDAQ Service Discovery
//!
//! Listens on 224.0.0.251:5353 (UDP multicast).
//! Responds to queries for _opendaq-streaming-native._tcp.local
//! with PTR → SRV → TXT → A records.
//!
//! DNS record hierarchy:
//!   PTR: _opendaq-streaming-native._tcp.local → FolkeringDAQ._opendaq-streaming-native._tcp.local
//!   SRV: FolkeringDAQ._opendaq-... → folkering-daq.local:7420
//!   TXT: caps=OPENDAQ_NS, model=SIRIUSi-HS, serial=D019274CF6
//!   A:   folkering-daq.local → <our IP>

/// mDNS constants
pub const MDNS_PORT: u16 = 5353;
pub const MDNS_MULTICAST: [u8; 4] = [224, 0, 0, 251];

/// Service configuration
const INSTANCE_NAME: &[u8] = b"FolkeringDAQ";
const HOST_NAME: &[u8] = b"folkering-daq";
const SERVICE_PORT: u16 = 7420;

// DNS record types
const TYPE_A: u16 = 1;
const TYPE_PTR: u16 = 12;
const TYPE_TXT: u16 = 16;
const TYPE_SRV: u16 = 33;
const CLASS_IN: u16 = 1;
const CLASS_FLUSH: u16 = 0x8001; // Cache Flush + IN
const TTL: u32 = 120; // 2 minutes

/// Write a DNS name label to buffer. Returns bytes written.
/// "foo" becomes [3, 'f', 'o', 'o']
fn write_label(buf: &mut [u8], offset: usize, label: &[u8]) -> usize {
    if offset + 1 + label.len() > buf.len() { return 0; }
    buf[offset] = label.len() as u8;
    buf[offset + 1..offset + 1 + label.len()].copy_from_slice(label);
    1 + label.len()
}

/// Write the service type domain name:
/// _opendaq-streaming-native._tcp.local\0
fn write_service_type(buf: &mut [u8], offset: usize) -> usize {
    let mut o = offset;
    o += write_label(buf, o, b"_opendaq-streaming-native");
    o += write_label(buf, o, b"_tcp");
    o += write_label(buf, o, b"local");
    buf[o] = 0; // Root label
    o += 1;
    o - offset
}

/// Write the instance name:
/// FolkeringDAQ._opendaq-streaming-native._tcp.local\0
fn write_instance_name(buf: &mut [u8], offset: usize) -> usize {
    let mut o = offset;
    o += write_label(buf, o, INSTANCE_NAME);
    o += write_label(buf, o, b"_opendaq-streaming-native");
    o += write_label(buf, o, b"_tcp");
    o += write_label(buf, o, b"local");
    buf[o] = 0;
    o += 1;
    o - offset
}

/// Write hostname: folkering-daq.local\0
fn write_hostname(buf: &mut [u8], offset: usize) -> usize {
    let mut o = offset;
    o += write_label(buf, o, HOST_NAME);
    o += write_label(buf, o, b"local");
    buf[o] = 0;
    o += 1;
    o - offset
}

/// Write u16 big-endian
fn write_u16(buf: &mut [u8], offset: usize, val: u16) -> usize {
    let bytes = val.to_be_bytes();
    buf[offset] = bytes[0];
    buf[offset + 1] = bytes[1];
    2
}

/// Write u32 big-endian
fn write_u32(buf: &mut [u8], offset: usize, val: u32) -> usize {
    let bytes = val.to_be_bytes();
    buf[offset..offset + 4].copy_from_slice(&bytes);
    4
}

/// Build TXT record RDATA: length-prefixed key=value pairs
fn write_txt_rdata(buf: &mut [u8], offset: usize) -> usize {
    let pairs: &[&[u8]] = &[
        b"caps=OPENDAQ_NS",
        b"model=SIRIUSi-HS",
        b"serial=D019274CF6",
        b"path=/",
    ];

    let mut o = offset;
    for pair in pairs {
        buf[o] = pair.len() as u8; // Length prefix
        o += 1;
        buf[o..o + pair.len()].copy_from_slice(pair);
        o += pair.len();
    }
    o - offset
}

/// Build a complete mDNS response packet with PTR + SRV + TXT + A records.
/// Returns total packet size.
pub fn build_response(our_ip: [u8; 4], buf: &mut [u8]) -> usize {
    let mut o = 0;

    // DNS Header (12 bytes)
    o += write_u16(buf, o, 0x0000); // Transaction ID (0 for mDNS)
    o += write_u16(buf, o, 0x8400); // Flags: Response, Authoritative
    o += write_u16(buf, o, 0x0000); // Questions: 0
    o += write_u16(buf, o, 0x0001); // Answers: 1 (PTR)
    o += write_u16(buf, o, 0x0000); // Authority: 0
    o += write_u16(buf, o, 0x0003); // Additional: 3 (SRV, TXT, A)

    // === Answer: PTR Record ===
    // Name: _opendaq-streaming-native._tcp.local
    let ptr_name_start = o;
    o += write_service_type(buf, o);
    o += write_u16(buf, o, TYPE_PTR);
    o += write_u16(buf, o, CLASS_IN);
    o += write_u32(buf, o, TTL);
    // RDATA: instance name
    let rdata_start = o;
    o += 2; // Placeholder for RDATA length
    let rdata_body_start = o;
    o += write_instance_name(buf, o);
    let rdata_len = o - rdata_body_start;
    write_u16(buf, rdata_start, rdata_len as u16);

    // === Additional: SRV Record ===
    let srv_name_start = o;
    o += write_instance_name(buf, o);
    o += write_u16(buf, o, TYPE_SRV);
    o += write_u16(buf, o, CLASS_FLUSH);
    o += write_u32(buf, o, TTL);
    // SRV RDATA: Priority(2) + Weight(2) + Port(2) + Target(hostname)
    let srv_rdata_start = o;
    o += 2; // Placeholder
    let srv_body = o;
    o += write_u16(buf, o, 0); // Priority
    o += write_u16(buf, o, 0); // Weight
    o += write_u16(buf, o, SERVICE_PORT); // Port 7420
    o += write_hostname(buf, o); // Target: folkering-daq.local
    let srv_rdata_len = o - srv_body;
    write_u16(buf, srv_rdata_start, srv_rdata_len as u16);

    // === Additional: TXT Record ===
    o += write_instance_name(buf, o);
    o += write_u16(buf, o, TYPE_TXT);
    o += write_u16(buf, o, CLASS_FLUSH);
    o += write_u32(buf, o, TTL);
    let txt_rdata_start = o;
    o += 2; // Placeholder
    let txt_body = o;
    o += write_txt_rdata(buf, o);
    let txt_rdata_len = o - txt_body;
    write_u16(buf, txt_rdata_start, txt_rdata_len as u16);

    // === Additional: A Record ===
    o += write_hostname(buf, o);
    o += write_u16(buf, o, TYPE_A);
    o += write_u16(buf, o, CLASS_FLUSH);
    o += write_u32(buf, o, TTL);
    o += write_u16(buf, o, 4); // RDATA length = 4 bytes (IPv4)
    buf[o..o + 4].copy_from_slice(&our_ip);
    o += 4;

    o
}

/// Check if an mDNS query packet is asking for our service type.
pub fn is_query_for_us(packet: &[u8]) -> bool {
    if packet.len() < 12 { return false; }

    // Check it's a query (QR bit = 0)
    let flags = u16::from_be_bytes([packet[2], packet[3]]);
    if flags & 0x8000 != 0 { return false; }

    // Check at least one question
    let qdcount = u16::from_be_bytes([packet[4], packet[5]]);
    if qdcount == 0 { return false; }

    // Simple string search for our service type in the query
    let needle = b"_opendaq-streaming-native";
    for i in 0..packet.len().saturating_sub(needle.len()) {
        if &packet[i..i + needle.len()] == needle {
            return true;
        }
    }

    // Also respond to _services._dns-sd queries (service enumeration)
    let sd_needle = b"_services._dns-sd";
    for i in 0..packet.len().saturating_sub(sd_needle.len()) {
        if &packet[i..i + sd_needle.len()] == sd_needle {
            return true;
        }
    }

    false
}
