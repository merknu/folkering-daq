//! WebSocket Protocol (RFC 6455) for no_std
//!
//! Handles HTTP Upgrade handshake (SHA-1 + Base64) and binary frame encoding.
//! Used by openDAQ Native Streaming on port 7420.
//!
//! Server-to-client frames are NEVER masked (per RFC 6455).
//! Client-to-server frames ARE masked and must be unmasked on receive.

// === SHA-1 (RFC 3174) — stack-only, no heap ===

/// Compute SHA-1 hash of input data. Returns 20-byte digest.
pub fn sha1(data: &[u8]) -> [u8; 20] {
    let mut h0: u32 = 0x67452301;
    let mut h1: u32 = 0xEFCDAB89;
    let mut h2: u32 = 0x98BADCFE;
    let mut h3: u32 = 0x10325476;
    let mut h4: u32 = 0xC3D2E1F0;

    let bit_len = (data.len() as u64) * 8;

    // Pad: data + 0x80 + zeros + 8-byte big-endian bit length
    // Total must be multiple of 64 bytes
    let padded_len = ((data.len() + 9 + 63) / 64) * 64;
    // Process in 64-byte blocks
    let mut block = [0u8; 128]; // Max 2 blocks for WebSocket key (60 bytes)

    // Copy data
    let copy_len = data.len().min(block.len());
    block[..copy_len].copy_from_slice(&data[..copy_len]);
    block[data.len()] = 0x80;

    // Write bit length at end of last block
    let len_offset = padded_len - 8;
    block[len_offset..len_offset + 8].copy_from_slice(&bit_len.to_be_bytes());

    // Process each 64-byte block
    let num_blocks = padded_len / 64;
    for blk in 0..num_blocks {
        let offset = blk * 64;
        let chunk = &block[offset..offset + 64];

        // Expand 16 words to 80
        let mut w = [0u32; 80];
        for i in 0..16 {
            w[i] = u32::from_be_bytes([
                chunk[i * 4], chunk[i * 4 + 1], chunk[i * 4 + 2], chunk[i * 4 + 3],
            ]);
        }
        for i in 16..80 {
            w[i] = (w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16]).rotate_left(1);
        }

        let (mut a, mut b, mut c, mut d, mut e) = (h0, h1, h2, h3, h4);

        for i in 0..80 {
            let (f, k) = match i {
                0..=19 => ((b & c) | ((!b) & d), 0x5A827999u32),
                20..=39 => (b ^ c ^ d, 0x6ED9EBA1u32),
                40..=59 => ((b & c) | (b & d) | (c & d), 0x8F1BBCDCu32),
                _ => (b ^ c ^ d, 0xCA62C1D6u32),
            };

            let temp = a.rotate_left(5)
                .wrapping_add(f)
                .wrapping_add(e)
                .wrapping_add(k)
                .wrapping_add(w[i]);

            e = d;
            d = c;
            c = b.rotate_left(30);
            b = a;
            a = temp;
        }

        h0 = h0.wrapping_add(a);
        h1 = h1.wrapping_add(b);
        h2 = h2.wrapping_add(c);
        h3 = h3.wrapping_add(d);
        h4 = h4.wrapping_add(e);
    }

    let mut digest = [0u8; 20];
    digest[0..4].copy_from_slice(&h0.to_be_bytes());
    digest[4..8].copy_from_slice(&h1.to_be_bytes());
    digest[8..12].copy_from_slice(&h2.to_be_bytes());
    digest[12..16].copy_from_slice(&h3.to_be_bytes());
    digest[16..20].copy_from_slice(&h4.to_be_bytes());
    digest
}

// === Base64 Encoding (no heap) ===

/// Encode 20-byte SHA-1 digest to 28-byte Base64 string.
pub fn base64_encode_sha1(digest: &[u8; 20], output: &mut [u8; 28]) {
    const ALPHA: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    let mut i = 0;
    let mut o = 0;

    // 18 bytes = 6 complete triplets → 24 Base64 chars
    while i < 18 {
        let bits = ((digest[i] as u32) << 16) | ((digest[i + 1] as u32) << 8) | (digest[i + 2] as u32);
        output[o] = ALPHA[((bits >> 18) & 0x3F) as usize];
        output[o + 1] = ALPHA[((bits >> 12) & 0x3F) as usize];
        output[o + 2] = ALPHA[((bits >> 6) & 0x3F) as usize];
        output[o + 3] = ALPHA[(bits & 0x3F) as usize];
        i += 3;
        o += 4;
    }

    // Remaining 2 bytes → 3 Base64 chars + 1 padding
    let bits = ((digest[18] as u32) << 16) | ((digest[19] as u32) << 8);
    output[o] = ALPHA[((bits >> 18) & 0x3F) as usize];
    output[o + 1] = ALPHA[((bits >> 12) & 0x3F) as usize];
    output[o + 2] = ALPHA[((bits >> 6) & 0x3F) as usize];
    output[o + 3] = b'=';
}

// === WebSocket Handshake ===

/// RFC 6455 WebSocket GUID
const WS_GUID: &[u8] = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

/// Compute Sec-WebSocket-Accept from Sec-WebSocket-Key.
/// Returns 28-byte Base64 string.
pub fn compute_accept_key(client_key: &[u8], output: &mut [u8; 28]) {
    // Concatenate client key + GUID (max 24 + 36 = 60 bytes)
    let mut concat = [0u8; 64];
    let key_len = client_key.len().min(24);
    concat[..key_len].copy_from_slice(&client_key[..key_len]);
    concat[key_len..key_len + WS_GUID.len()].copy_from_slice(WS_GUID);
    let total = key_len + WS_GUID.len();

    let digest = sha1(&concat[..total]);
    base64_encode_sha1(&digest, output);
}

/// Build HTTP 101 Switching Protocols response for WebSocket upgrade.
/// Returns number of bytes written to `buf`.
pub fn build_upgrade_response(client_key: &[u8], buf: &mut [u8]) -> usize {
    let mut accept = [0u8; 28];
    compute_accept_key(client_key, &mut accept);

    let header = b"HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: ";
    let trailer = b"\r\n\r\n";

    let total = header.len() + accept.len() + trailer.len();
    if buf.len() < total { return 0; }

    buf[..header.len()].copy_from_slice(header);
    buf[header.len()..header.len() + 28].copy_from_slice(&accept);
    buf[header.len() + 28..total].copy_from_slice(trailer);

    total
}

/// Extract Sec-WebSocket-Key from HTTP request headers.
/// Returns the key bytes and length, or None.
pub fn extract_websocket_key(request: &[u8]) -> Option<&[u8]> {
    // Search for "Sec-WebSocket-Key: " header
    let needle = b"Sec-WebSocket-Key: ";
    let req = request;

    for i in 0..req.len().saturating_sub(needle.len()) {
        if &req[i..i + needle.len()] == needle {
            // Find end of header value (until \r\n)
            let start = i + needle.len();
            for end in start..req.len().saturating_sub(1) {
                if req[end] == b'\r' && req[end + 1] == b'\n' {
                    return Some(&req[start..end]);
                }
            }
        }
    }
    None
}

// === WebSocket Binary Frame Encoding ===

/// Pack payload into a WebSocket Binary Frame (opcode 0x02, no masking).
/// Returns total frame size written to `frame_buf`.
pub fn pack_binary_frame(payload: &[u8], frame_buf: &mut [u8]) -> Result<usize, ()> {
    let len = payload.len();
    let header_size = if len < 126 { 2 } else if len <= 0xFFFF { 4 } else { 10 };

    if frame_buf.len() < header_size + len {
        return Err(());
    }

    // Byte 0: FIN=1, Opcode=0x02 (Binary)
    frame_buf[0] = 0x82;

    // Byte 1+: length encoding (server→client: Mask bit = 0)
    let mut offset = 1;
    if len < 126 {
        frame_buf[offset] = len as u8;
        offset += 1;
    } else if len <= 0xFFFF {
        frame_buf[offset] = 126;
        frame_buf[offset + 1..offset + 3].copy_from_slice(&(len as u16).to_be_bytes());
        offset += 3;
    } else {
        frame_buf[offset] = 127;
        frame_buf[offset + 1..offset + 9].copy_from_slice(&(len as u64).to_be_bytes());
        offset += 9;
    }

    // Payload
    frame_buf[offset..offset + len].copy_from_slice(payload);

    Ok(offset + len)
}

/// Unmask a client-to-server WebSocket frame payload (in-place).
/// Client frames have Mask bit set and 4-byte masking key after length.
pub fn unmask_payload(mask_key: [u8; 4], data: &mut [u8]) {
    for (i, byte) in data.iter_mut().enumerate() {
        *byte ^= mask_key[i % 4];
    }
}

/// Parse a WebSocket frame header. Returns (opcode, payload_offset, payload_len, mask_key).
pub fn parse_frame_header(data: &[u8]) -> Option<(u8, usize, usize, Option<[u8; 4]>)> {
    if data.len() < 2 { return None; }

    let opcode = data[0] & 0x0F;
    let masked = (data[1] & 0x80) != 0;
    let len_byte = (data[1] & 0x7F) as usize;

    let (payload_len, mut offset) = if len_byte < 126 {
        (len_byte, 2)
    } else if len_byte == 126 {
        if data.len() < 4 { return None; }
        let len = u16::from_be_bytes([data[2], data[3]]) as usize;
        (len, 4)
    } else {
        if data.len() < 10 { return None; }
        let len = u64::from_be_bytes([
            data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9],
        ]) as usize;
        (len, 10)
    };

    let mask_key = if masked {
        if data.len() < offset + 4 { return None; }
        let key = [data[offset], data[offset + 1], data[offset + 2], data[offset + 3]];
        offset += 4;
        Some(key)
    } else {
        None
    };

    Some((opcode, offset, payload_len, mask_key))
}
