//! openDAQ Native Streaming Protocol Server (port 7420)
//!
//! Wire-level binary protocol for high-throughput sensor data streaming.
//! Designed for DewesoftX compatibility (locked to protocol v3.20.6).
//!
//! Architecture:
//! - Raw TCP on port 7420 (daq.nd:// transport) — NO WebSocket for data
//! - JSON metadata handshake at connection start (SignalAvailable)
//! - Binary DataPackets with 0xDA51 sync word for streaming
//! - TCP_NODELAY mandatory (Nagle's algorithm breaks real-time sync)
//!
//! CRITICAL DewesoftX compatibility notes:
//! - version MUST be 3.20.6 (v3.30+ causes silent handshake rejection)
//! - Device tree MUST be static (runtime changes break auto-streaming)
//! - JSON __type fields MUST use exact camelCase
//! - Stream IDs are hashed from publicId strings

use alloc::vec::Vec;
use core::sync::atomic::{AtomicU64, AtomicBool, Ordering};
use crate::usb::sirius::{FRAMES_PER_PACKET, NUM_CHANNELS};

// === Protocol Constants ===

/// Sync word: marks start of every binary data packet (Little-Endian: 0x51DA on wire)
const SYNC_WORD: u16 = 0xDA51;

/// Packet types
const PKT_TYPE_DATA: u8 = 0x01;

/// Connection state
static CONNECTED: AtomicBool = AtomicBool::new(false);
static METADATA_SENT: AtomicBool = AtomicBool::new(false);
static PACKET_SEQ: AtomicU64 = AtomicU64::new(0);

// === Binary DataPacket Header (12 bytes, ALL Little-Endian) ===
//
// Offset  Field           Type    Endian       Description
// 0x00    Sync Word       u16     LE           Fixed 0xDA51
// 0x02    Packet Type     u8      —            0x01 = DataPacket
// 0x03    Flags           u8      —            0x00 = raw uncompressed
// 0x04    Stream ID       u32     LE           Hash of publicId from JSON
// 0x08    Payload Size    u32     LE           Byte count (excluding header)
// 0x0C    Payload         [u8]    LE           Raw sample data

/// 12-byte binary packet header
#[repr(C, packed)]
pub struct DataPacketHeader {
    pub sync_word: u16,     // 0xDA51
    pub packet_type: u8,    // 0x01
    pub flags: u8,          // 0x00
    pub stream_id: u32,     // Hash of publicId
    pub payload_size: u32,  // Bytes of payload following header
}

impl DataPacketHeader {
    /// Create header for a data packet. All fields Little-Endian on wire.
    pub fn new(stream_id: u32, payload_bytes: u32) -> Self {
        Self {
            sync_word: SYNC_WORD.to_le(),
            packet_type: PKT_TYPE_DATA,
            flags: 0x00,
            stream_id: stream_id.to_le(),
            payload_size: payload_bytes.to_le(),
        }
    }

    /// Serialize to 12-byte array for TCP transmission
    pub fn to_bytes(&self) -> [u8; 12] {
        let mut buf = [0u8; 12];
        buf[0..2].copy_from_slice(&self.sync_word.to_ne_bytes());
        buf[2] = self.packet_type;
        buf[3] = self.flags;
        buf[4..8].copy_from_slice(&self.stream_id.to_ne_bytes());
        buf[8..12].copy_from_slice(&self.payload_size.to_ne_bytes());
        buf
    }
}

// === Stream ID Hashing ===

/// Simple hash of publicId string → u32 stream ID.
/// Must be deterministic and match between JSON metadata and binary packets.
fn hash_public_id(id: &str) -> u32 {
    let mut h: u32 = 5381;
    for b in id.bytes() {
        h = h.wrapping_mul(33).wrapping_add(b as u32);
    }
    h
}

// === JSON Metadata (SignalAvailable) ===
//
// DewesoftX v3.20.6 requires exact schema with:
// - __type fields for polymorphic deserialization
// - valueType as integer (4=Float32, 3=Int32)
// - camelCase property names
// - Linear rule even for explicit data

/// Build SignalAvailable JSON for a channel.
/// This is transmitted ONCE at connection start, never modified.
///
/// Schema matches DewesoftX v3.20.6 parser expectations exactly.
fn build_signal_json(channel: usize) -> Vec<u8> {
    // Use alloc::format! for simplicity — this only runs once at connection init
    let json = alloc::format!(
        r#"{{"__type":"Signal","name":"Channel_{}","publicId":"sig_ch{}","dataDescriptor":{{"__type":"DataDescriptor","valueType":3,"rule":{{"__type":"DataRule","type":"Linear","delta":1.0,"start":0.0}},"postScaling":null}}}}"#,
        channel, channel
    );
    json.into_bytes()
}

/// Build domain signal (time axis) JSON
fn build_domain_signal_json() -> Vec<u8> {
    let json = alloc::format!(
        r#"{{"__type":"Signal","name":"Time","publicId":"sig_time","dataDescriptor":{{"__type":"DataDescriptor","valueType":3,"rule":{{"__type":"DataRule","type":"Linear","delta":50,"start":0}},"postScaling":null}}}}"#
    );
    json.into_bytes()
}

// === Stream ID Table (static, computed once) ===

struct StreamInfo {
    public_id: &'static str,
    stream_id: u32,
}

const STREAM_COUNT: usize = NUM_CHANNELS + 1; // 8 channels + 1 time domain

static mut STREAMS: [StreamInfo; STREAM_COUNT] = {
    const EMPTY: StreamInfo = StreamInfo { public_id: "", stream_id: 0 };
    [EMPTY; STREAM_COUNT]
};

fn init_streams() {
    let channel_ids: [&str; 8] = [
        "sig_ch0", "sig_ch1", "sig_ch2", "sig_ch3",
        "sig_ch4", "sig_ch5", "sig_ch6", "sig_ch7",
    ];

    unsafe {
        // Domain signal (time)
        STREAMS[0] = StreamInfo {
            public_id: "sig_time",
            stream_id: hash_public_id("sig_time"),
        };

        // Data channels
        for i in 0..8 {
            STREAMS[i + 1] = StreamInfo {
                public_id: channel_ids[i],
                stream_id: hash_public_id(channel_ids[i]),
            };
        }
    }

    crate::kprintln!("  openDAQ: {} streams initialized (v3.20.6 compat)", STREAM_COUNT);
}

// === Protocol State Machine ===

#[derive(Clone, Copy, PartialEq)]
enum ProtoState {
    WaitingConnection,
    MetadataBroadcast,
    ActiveStreaming,
}

static mut STATE: ProtoState = ProtoState::WaitingConnection;

pub fn init() {
    init_streams();
    crate::kprintln!("  openDAQ: Native Streaming on :7420 (daq.nd://)");
    crate::kprintln!("  openDAQ: protocol v3.20.6 (DewesoftX compat)");
}

pub fn poll() {
    // State transitions handled by net::handle_opendaq_tcp()
}

/// Called when TCP connection established on port 7420.
/// Immediately sends all SignalAvailable metadata (static, never changes).
pub fn on_client_connected() {
    CONNECTED.store(true, Ordering::SeqCst);
    METADATA_SENT.store(false, Ordering::SeqCst);
    PACKET_SEQ.store(0, Ordering::SeqCst);
    unsafe { STATE = ProtoState::MetadataBroadcast; }
    crate::kprintln!("  openDAQ: client connected (daq.nd://)");
}

/// Generate all metadata payloads to send at connection start.
/// Returns Vec of JSON byte arrays — caller sends them over TCP.
pub fn get_metadata_payloads() -> Vec<Vec<u8>> {
    let mut payloads = Vec::with_capacity(STREAM_COUNT);

    // Domain signal first (DewesoftX requires time before value signals)
    payloads.push(build_domain_signal_json());

    // 8 data channels
    for ch in 0..NUM_CHANNELS {
        payloads.push(build_signal_json(ch));
    }

    METADATA_SENT.store(true, Ordering::SeqCst);
    unsafe { STATE = ProtoState::ActiveStreaming; }
    crate::kprintln!("  openDAQ: metadata sent ({} signals)", STREAM_COUNT);

    payloads
}

/// Build a binary DataPacket for one channel's samples.
///
/// Format: [12-byte header][payload: N × i16 LE]
/// Total: 12 + (FRAMES_PER_PACKET × 2) = 12 + 1984 = 1996 bytes per packet
pub fn build_data_packet(channel: usize, samples: &[i16]) -> Vec<u8> {
    let stream_idx = channel + 1; // +1 because index 0 is time domain
    let stream_id = unsafe { STREAMS[stream_idx].stream_id };
    let payload_bytes = (samples.len() * 2) as u32;

    let header = DataPacketHeader::new(stream_id, payload_bytes);
    let header_bytes = header.to_bytes();

    let total = 12 + payload_bytes as usize;
    let mut packet = Vec::with_capacity(total);
    packet.extend_from_slice(&header_bytes);

    // Raw samples: int16 Little-Endian (native on ARM LE)
    for &s in samples {
        packet.extend_from_slice(&s.to_le_bytes());
    }

    packet
}

/// Broadcast latest ADC data to connected client.
/// Called from main loop when streaming is active.
pub fn broadcast_data(data: &[[i16; FRAMES_PER_PACKET]; NUM_CHANNELS]) {
    if !CONNECTED.load(Ordering::Relaxed) { return; }
    if !METADATA_SENT.load(Ordering::Relaxed) { return; }

    PACKET_SEQ.fetch_add(1, Ordering::Relaxed);

    for ch in 0..NUM_CHANNELS {
        let packet = build_data_packet(ch, &data[ch]);
        // Send raw binary over TCP (NO WebSocket framing for daq.nd://)
        if crate::net::tcp_send_raw(&packet).is_err() {
            // Network congestion — drop oldest data, never block ADC
            break;
        }
    }
}

/// Handle incoming data from client on the streaming connection.
/// In daq.nd:// mode, client mostly sends subscription commands.
pub fn on_client_message(_data: &[u8]) {
    // For now, auto-subscribe all channels on connect
    // DewesoftX sends subscription requests, but we can auto-stream
}

pub fn is_connected() -> bool {
    CONNECTED.load(Ordering::Relaxed)
}

pub fn on_client_disconnected() {
    CONNECTED.store(false, Ordering::SeqCst);
    METADATA_SENT.store(false, Ordering::SeqCst);
    unsafe { STATE = ProtoState::WaitingConnection; }
    crate::kprintln!("  openDAQ: client disconnected");
}
