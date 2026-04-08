//! openDAQ Native Streaming Protocol Server (port 7420)
//!
//! Wire protocol: WebSocket (binary) over TCP
//! Transport header: u32 LE — bits[31:28] = PayloadType, bits[27:0] = PayloadSize
//!
//! Connection sequence:
//! 1. Client connects via WebSocket
//! 2. Client sends TransportLayerProperties (type=10)
//! 3. Client sends StreamingProtocolInitRequest (type=11)
//! 4. Server sends SignalAvailable (type=2) for each signal
//! 5. Server sends ProtocolInitDone (type=6)
//! 6. Client sends SignalSubscribeCommand (type=4)
//! 7. Server sends SignalSubscribeAck (type=7)
//! 8. Server streams DataPackets (type=1)

use alloc::vec::Vec;
use alloc::string::String;
use core::sync::atomic::{AtomicU64, Ordering};

use crate::usb::sirius::{FRAMES_PER_PACKET, NUM_CHANNELS};

// Transport header payload types
const PT_STREAMING_PACKET: u8 = 1;
const PT_SIGNAL_AVAILABLE: u8 = 2;
const PT_SIGNAL_UNAVAILABLE: u8 = 3;
const PT_SUBSCRIBE: u8 = 4;
const PT_UNSUBSCRIBE: u8 = 5;
const PT_INIT_DONE: u8 = 6;
const PT_SUBSCRIBE_ACK: u8 = 7;
const PT_UNSUBSCRIBE_ACK: u8 = 8;
const PT_CONFIG: u8 = 9;
const PT_TRANSPORT_PROPS: u8 = 10;
const PT_STREAMING_INIT_REQ: u8 = 11;

// Packet types within StreamingPacket
const PKT_EVENT: u8 = 0;
const PKT_DATA: u8 = 1;

/// Running packet ID counter
static PACKET_ID: AtomicU64 = AtomicU64::new(1);

/// Client session state
struct Session {
    // TCP connection handle (placeholder)
    subscribed_signals: Vec<u32>,  // numeric IDs
    init_done: bool,
}

static mut SESSIONS: Vec<Session> = Vec::new();

/// Encode transport header
fn encode_header(payload_type: u8, payload_size: u32) -> [u8; 4] {
    let packed = ((payload_type as u32) << 28) | (payload_size & 0x0FFF_FFFF);
    packed.to_le_bytes()
}

/// Decode transport header
fn decode_header(bytes: &[u8; 4]) -> (u8, u32) {
    let packed = u32::from_le_bytes(*bytes);
    let payload_type = ((packed >> 28) & 0xF) as u8;
    let payload_size = packed & 0x0FFF_FFFF;
    (payload_type, payload_size)
}

/// Build SignalAvailable payload
fn build_signal_available(signal: &super::signal::SignalDescriptor) -> Vec<u8> {
    let mut payload = Vec::new();

    // SignalNumericId (u32 LE)
    payload.extend_from_slice(&signal.numeric_id.to_le_bytes());

    // SignalStringId (u16 LE length + bytes)
    let id_bytes = signal.id.as_bytes();
    payload.extend_from_slice(&(id_bytes.len() as u16).to_le_bytes());
    payload.extend_from_slice(id_bytes);

    // Serialized signal JSON
    let json = super::signal::serialize_signal_json(signal);
    payload.extend_from_slice(json.as_bytes());

    payload
}

/// Build a data packet for streaming
fn build_data_packet(
    signal_numeric_id: u32,
    data: &[i16],
    packet_id: u64,
) -> Vec<u8> {
    let mut packet = Vec::new();

    // Generic packet header (12 bytes for header_size in data mode = 44)
    let sample_bytes = data.len() * 2;
    let header_size: u8 = 44;
    let packet_type: u8 = PKT_DATA;

    packet.push(header_size);
    packet.push(packet_type);
    packet.push(0); // protocol_version
    packet.push(0); // flags

    packet.extend_from_slice(&signal_numeric_id.to_le_bytes());
    packet.extend_from_slice(&(sample_bytes as u32).to_le_bytes());

    // Data packet extra header (32 bytes)
    packet.extend_from_slice(&packet_id.to_le_bytes());       // packet_id
    packet.extend_from_slice(&0u64.to_le_bytes());             // domain_packet_id
    packet.extend_from_slice(&(data.len() as i64).to_le_bytes()); // sample_count
    packet.extend_from_slice(&0u64.to_le_bytes());             // packet_offset

    // Raw sample data (int16 LE)
    for &sample in data {
        packet.extend_from_slice(&sample.to_le_bytes());
    }

    packet
}

pub fn init() {
    // TODO: bind TCP listener on port 7420
    // TODO: set up WebSocket upgrade handler
    crate::kprintln!("  openDAQ: server initialized (port 7420)");
}

pub fn poll() {
    // TODO: accept new TCP connections
    // TODO: process incoming messages from clients
    // TODO: handle WebSocket framing
}

/// Broadcast ADC data to all subscribed clients
pub fn broadcast_data(data: &[[i16; FRAMES_PER_PACKET]; NUM_CHANNELS]) {
    let pid = PACKET_ID.fetch_add(1, Ordering::Relaxed);

    unsafe {
        for session in SESSIONS.iter() {
            if !session.init_done { continue; }

            for &sig_id in &session.subscribed_signals {
                let ch = (sig_id - 1) as usize;
                if ch >= NUM_CHANNELS { continue; }

                let packet = build_data_packet(sig_id, &data[ch], pid);
                let header = encode_header(PT_STREAMING_PACKET, packet.len() as u32);

                // TODO: send header + packet over TCP/WebSocket
                let _ = header;
                let _ = packet;
            }
        }
    }
}

/// Handle a new client connection (called when TCP accept succeeds)
pub fn on_client_connected() {
    unsafe {
        SESSIONS.push(Session {
            subscribed_signals: Vec::new(),
            init_done: false,
        });
    }
}

/// Handle incoming message from a client
pub fn on_client_message(session_idx: usize, data: &[u8]) {
    if data.len() < 4 { return; }

    let header: [u8; 4] = [data[0], data[1], data[2], data[3]];
    let (ptype, psize) = decode_header(&header);
    let payload = &data[4..];

    match ptype {
        PT_TRANSPORT_PROPS => {
            // Client sending properties (ClientType, etc.)
            // Parse JSON — for now just acknowledge
            crate::kprintln!("  openDAQ: client sent TransportLayerProperties");
        }
        PT_STREAMING_INIT_REQ => {
            // Send SignalAvailable for each signal
            let signals = super::signal::get_signals();
            for signal in signals {
                let avail = build_signal_available(signal);
                let header = encode_header(PT_SIGNAL_AVAILABLE, avail.len() as u32);
                // TODO: send header + avail to client
            }

            // Send ProtocolInitDone
            let done_header = encode_header(PT_INIT_DONE, 0);
            // TODO: send done_header to client

            unsafe {
                if let Some(session) = SESSIONS.get_mut(session_idx) {
                    session.init_done = true;
                }
            }
            crate::kprintln!("  openDAQ: streaming init complete for client {}", session_idx);
        }
        PT_SUBSCRIBE => {
            if payload.len() >= 4 {
                let sig_id = u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
                unsafe {
                    if let Some(session) = SESSIONS.get_mut(session_idx) {
                        session.subscribed_signals.push(sig_id);
                    }
                }

                // Send SubscribeAck
                let ack_payload = sig_id.to_le_bytes();
                let ack_header = encode_header(PT_SUBSCRIBE_ACK, 4);
                // TODO: send ack_header + ack_payload to client

                crate::kprintln!("  openDAQ: client {} subscribed to signal {}", session_idx, sig_id);
            }
        }
        PT_UNSUBSCRIBE => {
            if payload.len() >= 4 {
                let sig_id = u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
                unsafe {
                    if let Some(session) = SESSIONS.get_mut(session_idx) {
                        session.subscribed_signals.retain(|&id| id != sig_id);
                    }
                }
            }
        }
        _ => {
            crate::kprintln!("  openDAQ: unknown message type {}", ptype);
        }
    }
}
