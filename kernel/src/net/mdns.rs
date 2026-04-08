//! mDNS / DNS-SD Responder for openDAQ Discovery
//!
//! Listens on 224.0.0.251:5353 (UDP multicast).
//!
//! openDAQ uses TWO service types (bifurcated architecture):
//!   _opendaq-nd._tcp.local    → Native Streaming data (port 7420)
//!   _opendaq-opcua._tcp.local → OPC UA device tree (port 4840)
//!
//! DewesoftX queries BOTH and requires:
//!   caps=OPENDAQ (NOT "OPENDAQ_NS")
//!   version=3.20.6 (CRITICAL — newer versions get rejected!)
//!   model=PQTech (must match OPC UA root device name)
//!   serial=<unique> (used for connection persistence)

pub const MDNS_PORT: u16 = 5353;
pub const MDNS_MULTICAST: [u8; 4] = [224, 0, 0, 251];

const INSTANCE_NAME: &[u8] = b"FolkeringDAQ";
const HOST_NAME: &[u8] = b"folkering-daq";
const ND_PORT: u16 = 7420;

const TYPE_A: u16 = 1;
const TYPE_PTR: u16 = 12;
const TYPE_TXT: u16 = 16;
const TYPE_SRV: u16 = 33;
const CLASS_IN: u16 = 1;
const CLASS_FLUSH: u16 = 0x8001;
const TTL: u32 = 120;

fn write_label(buf: &mut [u8], o: usize, label: &[u8]) -> usize {
    if o + 1 + label.len() > buf.len() { return 0; }
    buf[o] = label.len() as u8;
    buf[o + 1..o + 1 + label.len()].copy_from_slice(label);
    1 + label.len()
}

/// Write: _opendaq-nd._tcp.local\0
fn write_nd_service(buf: &mut [u8], o: usize) -> usize {
    let mut p = o;
    p += write_label(buf, p, b"_opendaq-nd");
    p += write_label(buf, p, b"_tcp");
    p += write_label(buf, p, b"local");
    buf[p] = 0; p += 1;
    p - o
}

/// Write: <instance>._opendaq-nd._tcp.local\0
fn write_nd_instance(buf: &mut [u8], o: usize) -> usize {
    let mut p = o;
    p += write_label(buf, p, INSTANCE_NAME);
    p += write_label(buf, p, b"_opendaq-nd");
    p += write_label(buf, p, b"_tcp");
    p += write_label(buf, p, b"local");
    buf[p] = 0; p += 1;
    p - o
}

/// Write: folkering-daq.local\0
fn write_hostname(buf: &mut [u8], o: usize) -> usize {
    let mut p = o;
    p += write_label(buf, p, HOST_NAME);
    p += write_label(buf, p, b"local");
    buf[p] = 0; p += 1;
    p - o
}

fn w16(buf: &mut [u8], o: usize, v: u16) -> usize {
    buf[o..o + 2].copy_from_slice(&v.to_be_bytes()); 2
}

fn w32(buf: &mut [u8], o: usize, v: u32) -> usize {
    buf[o..o + 4].copy_from_slice(&v.to_be_bytes()); 4
}

/// Build TXT RDATA with DewesoftX-compatible key-value pairs.
///
/// CRITICAL: version MUST be 3.20.6 — DewesoftX rejects v3.30+
/// CRITICAL: caps MUST be "OPENDAQ" (not "OPENDAQ_NS")
fn write_txt_rdata(buf: &mut [u8], o: usize) -> usize {
    let pairs: &[&[u8]] = &[
        b"caps=OPENDAQ",
        b"version=3.20.6",
        b"model=PQTech",
        b"serial=D019274CF6",
        b"path=/",
    ];
    let mut p = o;
    for pair in pairs {
        buf[p] = pair.len() as u8; // Length-prefixed (Pascal-style, NOT null-terminated)
        p += 1;
        buf[p..p + pair.len()].copy_from_slice(pair);
        p += pair.len();
    }
    p - o
}

/// Build complete mDNS response: PTR + SRV + TXT + A
pub fn build_response(our_ip: [u8; 4], buf: &mut [u8]) -> usize {
    let mut o = 0;

    // DNS Header (12 bytes, all Big-Endian)
    o += w16(buf, o, 0x0000); // Transaction ID (0 for mDNS)
    o += w16(buf, o, 0x8400); // Flags: Response + Authoritative
    o += w16(buf, o, 0x0000); // Questions: 0
    o += w16(buf, o, 0x0001); // Answers: 1 (PTR)
    o += w16(buf, o, 0x0000); // Authority: 0
    o += w16(buf, o, 0x0003); // Additional: 3 (SRV + TXT + A)

    // === PTR Record ===
    // Name: _opendaq-nd._tcp.local
    o += write_nd_service(buf, o);
    o += w16(buf, o, TYPE_PTR);
    o += w16(buf, o, CLASS_IN);
    o += w32(buf, o, TTL);
    let rdlen_pos = o; o += 2;
    let rd_start = o;
    o += write_nd_instance(buf, o);
    w16(buf, rdlen_pos, (o - rd_start) as u16);

    // === SRV Record ===
    o += write_nd_instance(buf, o);
    o += w16(buf, o, TYPE_SRV);
    o += w16(buf, o, CLASS_FLUSH);
    o += w32(buf, o, TTL);
    let rdlen_pos = o; o += 2;
    let rd_start = o;
    o += w16(buf, o, 0);        // Priority
    o += w16(buf, o, 0);        // Weight
    o += w16(buf, o, ND_PORT);  // Port 7420
    o += write_hostname(buf, o);
    w16(buf, rdlen_pos, (o - rd_start) as u16);

    // === TXT Record ===
    o += write_nd_instance(buf, o);
    o += w16(buf, o, TYPE_TXT);
    o += w16(buf, o, CLASS_FLUSH);
    o += w32(buf, o, TTL);
    let rdlen_pos = o; o += 2;
    let rd_start = o;
    o += write_txt_rdata(buf, o);
    w16(buf, rdlen_pos, (o - rd_start) as u16);

    // === A Record ===
    o += write_hostname(buf, o);
    o += w16(buf, o, TYPE_A);
    o += w16(buf, o, CLASS_FLUSH);
    o += w32(buf, o, TTL);
    o += w16(buf, o, 4);
    buf[o..o + 4].copy_from_slice(&our_ip);
    o += 4;

    o
}

/// Check if query is for our service
pub fn is_query_for_us(packet: &[u8]) -> bool {
    if packet.len() < 12 { return false; }
    // Must be a query (QR=0)
    if packet[2] & 0x80 != 0 { return false; }
    // At least one question
    if u16::from_be_bytes([packet[4], packet[5]]) == 0 { return false; }

    // Search for our service types
    for needle in &[b"_opendaq-nd" as &[u8], b"_opendaq-opcua", b"_services._dns-sd"] {
        for i in 0..packet.len().saturating_sub(needle.len()) {
            if &packet[i..i + needle.len()] == *needle {
                return true;
            }
        }
    }
    false
}
