//! openDAQ Native Streaming Protocol Server (port 7420)
//!
//! Wire protocol: WebSocket Binary over TCP
//!
//! Transport Header: 4 bytes (u32 LE)
//!   bits[31:28] = Payload Type (4-bit code)
//!   bits[27:0]  = Payload Size in bytes (max 268 MB)
//!
//! Handshake sequence (DewesoftX compatibility):
//!   1. Client connects via WebSocket (HTTP Upgrade → 101 Switching Protocols)
//!   2. Client → Server: StreamingInitRequest (0x0B) — header-only
//!   3. Server → Client: SignalAvailable (0x02) × N — one per signal
//!   4. Server → Client: InitDone (0x06) — header-only
//!   5. Client → Server: Subscribe (0x04) — per signal
//!   6. Server → Client: SubscribeAck (0x07) — 4 bytes (numeric ID)
//!   7. Server → Client: SignalPacket (0x01) — continuous data stream

use alloc::vec::Vec;
use alloc::string::String;
use core::sync::atomic::{AtomicU64, Ordering};

use crate::usb::sirius::{FRAMES_PER_PACKET, NUM_CHANNELS};

// === Payload Type Codes (4-bit, in upper nibble of transport header) ===
const PT_SIGNAL_PACKET: u8 = 0x01;         // Continuous data stream
const PT_SIGNAL_AVAILABLE: u8 = 0x02;      // Signal metadata notification
const PT_SIGNAL_UNAVAILABLE: u8 = 0x03;    // Signal removed
const PT_SUBSCRIBE: u8 = 0x04;             // Client subscribes to signal
const PT_UNSUBSCRIBE: u8 = 0x05;           // Client unsubscribes
const PT_INIT_DONE: u8 = 0x06;             // Signal enumeration complete
const PT_SUBSCRIBE_ACK: u8 = 0x07;         // Subscription confirmed
const PT_UNSUBSCRIBE_ACK: u8 = 0x08;       // Unsubscription confirmed
const PT_CONFIG: u8 = 0x09;                // Configuration packet
const PT_TRANSPORT_PROPS: u8 = 0x0A;       // Client transport properties
const PT_STREAMING_INIT_REQ: u8 = 0x0B;    // Client requests signal listing

// Packet buffer types (inside SignalPacket)
const PKT_EVENT: u8 = 0;
const PKT_DATA: u8 = 1;

/// Running packet ID
static PACKET_ID: AtomicU64 = AtomicU64::new(1);

/// openDAQ Transport Header — 4 bytes LE
///
/// Encodes payload type (4 bits) and payload size (28 bits) into a single u32.
/// This is called millions of times per second during streaming.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct TransportHeader {
    pub raw: u32,
}

impl TransportHeader {
    /// Construct header from type and size
    #[inline(always)]
    pub fn new(payload_type: u8, payload_size: u32) -> Self {
        let size_masked = payload_size & 0x0FFF_FFFF;
        let type_shifted = ((payload_type as u32) & 0x0F) << 28;
        Self { raw: type_shifted | size_masked }
    }

    /// Decode payload type (upper 4 bits)
    #[inline(always)]
    pub fn payload_type(&self) -> u8 {
        ((self.raw >> 28) & 0x0F) as u8
    }

    /// Decode payload size (lower 28 bits)
    #[inline(always)]
    pub fn payload_size(&self) -> u32 {
        self.raw & 0x0FFF_FFFF
    }

    /// Serialize to LE bytes for wire transmission
    #[inline(always)]
    pub fn to_le_bytes(&self) -> [u8; 4] {
        self.raw.to_le_bytes()
    }

    /// Parse from LE bytes received from wire
    #[inline(always)]
    pub fn from_le_bytes(bytes: [u8; 4]) -> Self {
        Self { raw: u32::from_le_bytes(bytes) }
    }
}

/// Client session state
struct Session {
    subscribed_signals: Vec<u32>,
    init_done: bool,
}

static mut SESSIONS: Vec<Session> = Vec::new();

/// Build SignalAvailable payload for a signal
///
/// Format: [NumericID: u32 LE][StringID len: u16 LE][StringID: bytes][JSON metadata]
fn build_signal_available(signal: &super::signal::SignalDescriptor) -> Vec<u8> {
    let mut payload = Vec::new();

    // Signal Numeric ID (4 bytes, uint32 LE)
    payload.extend_from_slice(&signal.numeric_id.to_le_bytes());

    // Signal Symbolic ID (uint16 length + string bytes)
    let id_bytes = signal.id.as_bytes();
    payload.extend_from_slice(&(id_bytes.len() as u16).to_le_bytes());
    payload.extend_from_slice(id_bytes);

    // Serialized JSON metadata (openDAQ format with __type markers)
    let json = super::signal::serialize_signal_json(signal);
    payload.extend_from_slice(json.as_bytes());

    payload
}

/// Build a Generic Packet Buffer Header + data for streaming
///
/// Header: 12 bytes (event) or 44 bytes (data)
/// For data: header + raw sample bytes
fn build_data_packet(signal_numeric_id: u32, data: &[i16], packet_id: u64) -> Vec<u8> {
    let sample_bytes = data.len() * 2;
    let mut packet = Vec::with_capacity(44 + sample_bytes);

    // Generic Packet Buffer Header (12 bytes)
    packet.push(44);                    // header_size (44 for data packets)
    packet.push(PKT_DATA);             // packet_type
    packet.push(0);                     // protocol_version
    packet.push(0);                     // flags

    packet.extend_from_slice(&signal_numeric_id.to_le_bytes());   // signal ID
    packet.extend_from_slice(&(sample_bytes as u32).to_le_bytes()); // payload size

    // Data Packet Extra Header (+32 bytes = total 44)
    packet.extend_from_slice(&packet_id.to_le_bytes());           // packet_id
    packet.extend_from_slice(&0u64.to_le_bytes());                 // domain_packet_id
    packet.extend_from_slice(&(data.len() as i64).to_le_bytes()); // sample_count
    packet.extend_from_slice(&0u64.to_le_bytes());                 // packet_offset

    // Raw sample data (int16 LE) — direct from xHCI DMA buffers
    for &sample in data {
        packet.extend_from_slice(&sample.to_le_bytes());
    }

    packet
}

pub fn init() {
    crate::kprintln!("  openDAQ: Native Streaming on :7420");
    crate::kprintln!("  openDAQ: awaiting WebSocket clients...");
}

pub fn poll() {
    // TODO: accept TCP connections on port 7420
    // TODO: handle WebSocket HTTP Upgrade handshake (101 Switching Protocols)
    // TODO: process incoming binary frames
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

                // Build 0x01 Signal Packet with transport header
                let packet = build_data_packet(sig_id, &data[ch], pid);
                let header = TransportHeader::new(PT_SIGNAL_PACKET, packet.len() as u32);

                // TODO: send header.to_le_bytes() + packet over WebSocket binary frame
                let _ = header;
                let _ = packet;
            }
        }
    }
}

/// Handle incoming message from a client (after WebSocket frame decoding)
pub fn on_client_message(session_idx: usize, data: &[u8]) {
    if data.len() < 4 { return; }

    let header = TransportHeader::from_le_bytes([data[0], data[1], data[2], data[3]]);
    let payload = &data[4..];

    match header.payload_type() {
        PT_TRANSPORT_PROPS => {
            // Client sending properties (ClientType: "Control"/"Monitoring")
            crate::kprintln!("  openDAQ: client properties received");
        }

        PT_STREAMING_INIT_REQ => {
            // Client requests signal enumeration
            // Respond with SignalAvailable for each signal, then InitDone
            let signals = super::signal::get_signals();
            for signal in signals {
                let avail = build_signal_available(signal);
                let _hdr = TransportHeader::new(PT_SIGNAL_AVAILABLE, avail.len() as u32);
                // TODO: send _hdr.to_le_bytes() + avail
            }

            // Domain signals must be subscribed before value signals
            // Send InitDone (header-only, 0 bytes payload)
            let _done = TransportHeader::new(PT_INIT_DONE, 0);
            // TODO: send _done.to_le_bytes()

            unsafe {
                if let Some(session) = SESSIONS.get_mut(session_idx) {
                    session.init_done = true;
                }
            }
            crate::kprintln!("  openDAQ: init done for client {}", session_idx);
        }

        PT_SUBSCRIBE => {
            // Client subscribes to a signal
            if payload.len() >= 4 {
                let sig_id = u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);

                unsafe {
                    if let Some(session) = SESSIONS.get_mut(session_idx) {
                        session.subscribed_signals.push(sig_id);
                    }
                }

                // Activate xHCI transfers for this signal in background,
                // then send SubscribeAck (4 bytes = numeric ID)
                let _ack = TransportHeader::new(PT_SUBSCRIBE_ACK, 4);
                // TODO: send _ack.to_le_bytes() + sig_id.to_le_bytes()

                crate::kprintln!("  openDAQ: subscribed signal {}", sig_id);
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
            crate::kprintln!("  openDAQ: unknown type {:#x}", header.payload_type());
        }
    }
}

/// Called when a new TCP connection is accepted on port 7420
pub fn on_client_connected() {
    unsafe {
        SESSIONS.push(Session {
            subscribed_signals: Vec::new(),
            init_done: false,
        });
    }
    crate::kprintln!("  openDAQ: new client connected");
}
