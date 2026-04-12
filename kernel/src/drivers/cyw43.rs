//! CYW43455 WiFi Driver — SDIO FullMAC for Raspberry Pi 5
//!
//! Implements the Broadcom/Cypress FullMAC WiFi protocol over SDIO.
//! The CYW43455 handles all 802.11 and WPA2 internally — we only need to:
//!   1. Upload firmware binary via SDIO backplane
//!   2. Send ioctl commands (scan, join, disconnect)
//!   3. Send/receive Ethernet frames (802.3, not 802.11)
//!
//! SDIO architecture (3 functions):
//!   F0: CCCR/FBR (card common control registers)
//!   F1: Backplane (access chip's internal AXI bus for firmware upload + control)
//!   F2: WLAN data (Ethernet frame send/receive)
//!
//! Backplane protocol:
//!   The chip has an internal 32-bit AXI address space.
//!   To access it via SDIO: set the backplane window register (3 bytes)
//!   then read/write via F1 CMD53 in the lower 32KB of that window.
//!
//! Reference: Linux brcmfmac driver, georgirobotics/cyw43-driver

extern crate alloc;

use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, Ordering};
use super::sdhci;

// === SDIO Constants ===

// CCCR (Card Common Control Registers) — accessed via CMD52 on F0
const CCCR_IO_ENABLE: u32   = 0x02; // I/O function enable
const CCCR_IO_READY: u32    = 0x03; // I/O function ready
const CCCR_INT_ENABLE: u32  = 0x04; // Interrupt enable
const CCCR_BUS_CTRL: u32    = 0x07; // Bus interface control

// Backplane window register (F0, 3 bytes)
const BACKPLANE_ADDR_LOW: u32  = 0x1000A; // Window address [14:8]
const BACKPLANE_ADDR_MID: u32  = 0x1000B; // Window address [22:15]
const BACKPLANE_ADDR_HIGH: u32 = 0x1000C; // Window address [31:23]

// F1 backplane fixed address (within 32KB window)
const SB_32BIT_WIN: u32 = 0x8000; // SBSDIO 32-bit backplane window

// CYW43455 internal core addresses
const CHIPCOMMON_BASE: u32 = 0x1800_0000; // Chip Common core
const ARMCR4_BASE: u32     = 0x1800_2000; // ARM CR4 core (CPU)
const SDIOD_BASE: u32      = 0x1800_4000; // SDIO device core
const WLAN_BASE: u32       = 0x1800_A000; // D11 WLAN core (802.11 MAC)
const SOCSRAM_BASE: u32    = 0x1800_4000; // SoC SRAM wrapper

// ARM CR4 registers (for firmware upload)
const ARMCR4_CAP: u32     = 0x04;
const ARMCR4_BANKIDX: u32 = 0x40;
const ARMCR4_BANKINFO: u32 = 0x44;

// Chip ID register
const CHIPID_ADDR: u32 = 0x1800_0000; // chipcommon chipid

// Firmware RAM base (where firmware binary is uploaded to)
const ATCM_RAM_BASE: u32 = 0x0000_0000; // ARM ATCM (tightly coupled memory)

// Frame header for F2 data
const SDPCM_HEADER_LEN: usize = 12;

// ioctl commands (WLC = Wireless LAN Controller)
const WLC_UP: u32              = 2;
const WLC_SET_INFRA: u32       = 20;
const WLC_SET_AUTH: u32        = 22;
const WLC_SET_SSID: u32        = 26;
const WLC_SET_CHANNEL: u32     = 30;
const WLC_SET_PASSIVE_SCAN: u32 = 49;
const WLC_SCAN: u32            = 50;
const WLC_DISASSOC: u32        = 52;
const WLC_SET_WSEC: u32        = 134;
const WLC_SET_WPA_AUTH: u32    = 165;
const WLC_SET_WSEC_PMK: u32    = 268; // Correct ioctl for passphrase (NOT WLC_SET_KEY=45)

// WPA/Security constants
const WSEC_AES: u32  = 0x04;
const WPA2_AUTH_PSK: u32 = 0x80;

// SDPCM channel types
const SDPCM_CONTROL_CHANNEL: u8 = 0;
const SDPCM_DATA_CHANNEL: u8    = 2;
const SDPCM_EVENT_CHANNEL: u8   = 1;

// Broadcom event types (subset)
const WLC_E_LINK: u32       = 16;
const WLC_E_AUTH: u32        = 3;
const WLC_E_ASSOC: u32       = 0;
const WLC_E_SET_SSID: u32    = 0;
const WLC_E_PSK_SUP: u32     = 46;

// BDC (Broadcom Dongle Control) header
const BDC_HEADER_LEN: usize = 4; // flags(2) + priority(1) + data_offset(1)

/// WiFi connection state
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WiFiState {
    Off,
    Initializing,
    FirmwareLoaded,
    Ready,
    Scanning,
    Connecting,
    Connected,
    Error,
}

static WIFI_STATE: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0);
static F2_READY: AtomicBool = AtomicBool::new(false);

fn set_state(state: WiFiState) {
    WIFI_STATE.store(state as u8, Ordering::SeqCst);
}

pub fn state() -> WiFiState {
    match WIFI_STATE.load(Ordering::Relaxed) {
        0 => WiFiState::Off,
        1 => WiFiState::Initializing,
        2 => WiFiState::FirmwareLoaded,
        3 => WiFiState::Ready,
        4 => WiFiState::Scanning,
        5 => WiFiState::Connecting,
        6 => WiFiState::Connected,
        _ => WiFiState::Error,
    }
}

// === Backplane Window Management ===

/// Current backplane window base (cached to avoid redundant writes)
static mut CURRENT_WINDOW: u32 = 0;

/// Set the backplane window to access a specific 32KB region
fn set_backplane_window(addr: u32) -> Result<(), &'static str> {
    let window = addr & !0x7FFF; // Align to 32KB

    unsafe {
        if window == CURRENT_WINDOW { return Ok(()); }

        // Set 3-byte window register via CMD52 on F0
        sdhci::cmd52_write(0, BACKPLANE_ADDR_LOW, ((window >> 8) & 0xFF) as u8)?;
        sdhci::cmd52_write(0, BACKPLANE_ADDR_MID, ((window >> 16) & 0xFF) as u8)?;
        sdhci::cmd52_write(0, BACKPLANE_ADDR_HIGH, ((window >> 24) & 0xFF) as u8)?;

        CURRENT_WINDOW = window;
    }
    Ok(())
}

/// Read a 32-bit value from the backplane (chip's internal AXI bus)
fn bp_read32(addr: u32) -> Result<u32, &'static str> {
    set_backplane_window(addr)?;
    let offset = (addr & 0x7FFF) | SB_32BIT_WIN;

    let mut buf = [0u8; 4];
    sdhci::cmd53_read(1, offset, &mut buf, true)?;
    Ok(u32::from_le_bytes(buf))
}

/// Write a 32-bit value to the backplane
fn bp_write32(addr: u32, val: u32) -> Result<(), &'static str> {
    set_backplane_window(addr)?;
    let offset = (addr & 0x7FFF) | SB_32BIT_WIN;

    let buf = val.to_le_bytes();
    sdhci::cmd53_write(1, offset, &buf, true)?;
    Ok(())
}

/// Read a block from the backplane (for firmware upload verification)
fn bp_read_block(addr: u32, buf: &mut [u8]) -> Result<usize, &'static str> {
    let mut offset = 0;
    while offset < buf.len() {
        let chunk = (buf.len() - offset).min(64); // 64-byte SDIO blocks
        let bp_addr = addr + offset as u32;
        set_backplane_window(bp_addr)?;
        let sdio_offset = (bp_addr & 0x7FFF) | SB_32BIT_WIN;
        sdhci::cmd53_read(1, sdio_offset, &mut buf[offset..offset + chunk], true)?;
        offset += chunk;
    }
    Ok(buf.len())
}

/// Write a block to the backplane (for firmware upload)
fn bp_write_block(addr: u32, data: &[u8]) -> Result<usize, &'static str> {
    let mut offset = 0;
    while offset < data.len() {
        let chunk = (data.len() - offset).min(64);
        let bp_addr = addr + offset as u32;
        set_backplane_window(bp_addr)?;
        let sdio_offset = (bp_addr & 0x7FFF) | SB_32BIT_WIN;
        sdhci::cmd53_write(1, sdio_offset, &data[offset..offset + chunk], true)?;
        offset += chunk;
    }
    Ok(data.len())
}

// === Core Reset ===

/// Reset an AHB core via the SDIO wrapper
fn core_reset(core_base: u32) -> Result<(), &'static str> {
    // Disable core: set RESET bit
    bp_write32(core_base + 0x408, 0x01)?; // resetctrl = RESET
    crate::arch::aarch64::timer::delay_ms(1);

    // Enable core: clear RESET, set clock
    bp_write32(core_base + 0x408, 0x00)?; // resetctrl = 0
    bp_write32(core_base + 0x800, 0x01)?; // ioctrl = CLOCK_EN
    crate::arch::aarch64::timer::delay_ms(1);

    Ok(())
}

// === Firmware Upload ===

/// Upload firmware binary to the CYW43455's ARM CR4 SRAM
fn upload_firmware(fw_data: &[u8]) -> Result<(), &'static str> {
    crate::kprintln!("  CYW43: uploading firmware ({} bytes)...", fw_data.len());

    // Halt the ARM core before uploading
    bp_write32(ARMCR4_BASE + 0x408, 0x01)?; // Hold in reset

    // Upload firmware to ATCM RAM (starts at 0x00000000 on chip)
    bp_write_block(ATCM_RAM_BASE, fw_data)?;

    // Verify first 4 bytes
    let check = bp_read32(ATCM_RAM_BASE)?;
    let expected = u32::from_le_bytes([fw_data[0], fw_data[1], fw_data[2], fw_data[3]]);
    if check != expected {
        crate::kprintln!("  CYW43: firmware verify FAILED: got {:#x}, expected {:#x}", check, expected);
        return Err("firmware verification failed");
    }

    crate::kprintln!("  CYW43: firmware uploaded and verified");
    Ok(())
}

/// Upload NVRAM (configuration) to the chip
fn upload_nvram(nvram: &[u8]) -> Result<(), &'static str> {
    // NVRAM goes at the end of RAM, rounded up to 4 bytes
    let nvram_padded_len = (nvram.len() + 3) & !3;

    // Read RAM size from chip
    bp_write32(ARMCR4_BASE + ARMCR4_BANKIDX, 0)?;
    let bank_info = bp_read32(ARMCR4_BASE + ARMCR4_BANKINFO)?;
    let ram_size = (bank_info & 0x7F) * 8192; // Bank size in bytes

    let nvram_addr = ram_size - nvram_padded_len as u32;

    crate::kprintln!("  CYW43: RAM size = {} KB, NVRAM at {:#x}", ram_size / 1024, nvram_addr);

    // Write NVRAM data
    bp_write_block(nvram_addr, nvram)?;

    // Write NVRAM length token at end of NVRAM region
    let token = (!nvram_padded_len as u32) >> 16;
    bp_write32(nvram_addr + nvram_padded_len as u32 - 4, token)?;

    Ok(())
}

/// Release the ARM core to start executing firmware
fn start_firmware() -> Result<(), &'static str> {
    // Release ARM from reset
    bp_write32(ARMCR4_BASE + 0x408, 0x00)?; // resetctrl = 0

    // Wait for F2 ready (firmware signals this via SDIO)
    for i in 0..200 {
        let status = sdhci::cmd52_read(0, CCCR_IO_READY)?;
        if status & 0x02 != 0 { // F1 ready
            if status & 0x04 != 0 { // F2 ready (WLAN)
                F2_READY.store(true, Ordering::SeqCst);
                crate::kprintln!("  CYW43: firmware started, F2 ready ({}ms)", i * 10);
                return Ok(());
            }
        }
        crate::arch::aarch64::timer::delay_ms(10);
    }

    Err("firmware F2 ready timeout")
}

// === ioctl Interface ===

/// Separate sequence counters for control and data channels
static mut CTL_SEQ: u8 = 0;
static mut DATA_SEQ: u8 = 0;
/// ioctl request ID (incremented per ioctl, used to match responses)
static mut IOCTL_ID: u16 = 0;

fn next_ctl_seq() -> u8 {
    unsafe { let s = CTL_SEQ; CTL_SEQ = s.wrapping_add(1); s }
}

fn next_data_seq() -> u8 {
    unsafe { let s = DATA_SEQ; DATA_SEQ = s.wrapping_add(1); s }
}

fn next_ioctl_id() -> u16 {
    unsafe { let s = IOCTL_ID; IOCTL_ID = s.wrapping_add(1); s }
}

/// Build SDPCM header (12 bytes)
fn build_sdpcm_header(buf: &mut [u8], total_len: usize, channel: u8, seq: u8, data_offset: u8) {
    let len = total_len as u16;
    buf[0..2].copy_from_slice(&len.to_le_bytes());
    buf[2..4].copy_from_slice(&(len ^ 0xFFFF).to_le_bytes()); // Ones' complement checksum
    buf[4] = seq;
    buf[5] = channel;
    buf[6] = 0; // next length (unused)
    buf[7] = data_offset;
    buf[8] = 0; // flow control
    buf[9] = 0; // credit
    buf[10] = 0;
    buf[11] = 0;
}

/// Send an ioctl command and wait for the response
fn send_ioctl(cmd: u32, data: &[u8], set: bool) -> Result<Vec<u8>, &'static str> {
    if !F2_READY.load(Ordering::Relaxed) {
        return Err("F2 not ready");
    }

    let ioctl_id = next_ioctl_id();
    let seq = next_ctl_seq();

    // CDC header is 16 bytes, placed after SDPCM header
    let cdc_len = 16 + data.len();
    let total_len = SDPCM_HEADER_LEN + cdc_len;

    // Pad to 8-byte alignment (SDIO requirement)
    let padded_len = (total_len + 7) & !7;
    let mut frame = alloc::vec![0u8; padded_len];

    // SDPCM header
    build_sdpcm_header(&mut frame, padded_len, SDPCM_CONTROL_CHANNEL, seq, SDPCM_HEADER_LEN as u8);

    // CDC header (16 bytes)
    let c = SDPCM_HEADER_LEN;
    frame[c..c + 4].copy_from_slice(&cmd.to_le_bytes());                   // cmd
    frame[c + 4..c + 8].copy_from_slice(&(data.len() as u32).to_le_bytes()); // output buf len
    let flags: u32 = (if set { 0x02u32 } else { 0u32 }) | ((ioctl_id as u32) << 16);
    frame[c + 8..c + 12].copy_from_slice(&flags.to_le_bytes());            // flags (SET + id)
    // frame[c+12..c+16] = status (0, filled by firmware on response)

    // Payload
    if !data.is_empty() {
        frame[c + 16..c + 16 + data.len()].copy_from_slice(data);
    }

    // Send via F2 CMD53
    sdhci::cmd53_write(2, 0, &frame, true)?;

    // Wait for ioctl response (poll F2 for control channel frames)
    for _ in 0..100 {
        crate::arch::aarch64::timer::delay_ms(10);

        if let Some((channel, payload)) = read_sdpcm_frame()? {
            if channel == SDPCM_CONTROL_CHANNEL && payload.len() >= 16 {
                // Parse CDC response header
                let resp_id = u16::from_le_bytes([payload[10], payload[11]]);
                if resp_id == ioctl_id {
                    let status = u32::from_le_bytes([payload[12], payload[13], payload[14], payload[15]]);
                    if status != 0 {
                        crate::kprintln!("  CYW43: ioctl {} failed, status={}", cmd, status);
                        return Err("ioctl failed");
                    }
                    return Ok(payload[16..].to_vec());
                }
            }
            // Got a non-matching frame — could be event, process it
            if channel == SDPCM_EVENT_CHANNEL {
                process_event(&payload);
            }
        }
    }

    // Timeout is not fatal for SET ioctls — firmware may not send a response
    Ok(Vec::new())
}

/// Read one SDPCM frame from F2. Returns (channel, payload) or None.
fn read_sdpcm_frame() -> Result<Option<(u8, Vec<u8>)>, &'static str> {
    // Check if F2 has data pending
    let f2_status = sdhci::cmd52_read(0, 0x04).unwrap_or(0);
    if f2_status & 0x04 == 0 {
        return Ok(None); // No F2 data
    }

    let mut raw = [0u8; 2048];
    sdhci::cmd53_read(2, 0, &mut raw, true)?;

    // Parse SDPCM header
    let frame_len = u16::from_le_bytes([raw[0], raw[1]]) as usize;
    let check = u16::from_le_bytes([raw[2], raw[3]]);

    // Verify ones' complement checksum
    if (frame_len as u16) ^ check != 0xFFFF {
        return Ok(None); // Corrupt frame
    }

    if frame_len < SDPCM_HEADER_LEN || frame_len > raw.len() {
        return Ok(None);
    }

    let channel = raw[5];
    let data_offset = raw[7] as usize;

    if data_offset >= frame_len {
        return Ok(None);
    }

    let payload = raw[data_offset..frame_len].to_vec();
    Ok(Some((channel, payload)))
}

/// Process a firmware event (from event channel)
fn process_event(payload: &[u8]) {
    // BDC header (4 bytes) + bcm_event header
    if payload.len() < BDC_HEADER_LEN + 72 {
        return; // Too short for event
    }

    let bdc_data_offset = (payload[3] as usize) * 4; // BDC data offset in 32-bit words
    let event_start = BDC_HEADER_LEN + bdc_data_offset;

    if event_start + 48 > payload.len() { return; }

    // bcm_event: skip Ethernet header (14) + bcm_event header (10)
    // Event type is at offset 24+4=28 from event_start (after eth+bcmeth headers)
    // Simplified: look for event_type at a known offset
    let evt_off = event_start + 24; // Past ether_header + bcm_msg
    if evt_off + 4 > payload.len() { return; }

    let event_type = u32::from_be_bytes([
        payload[evt_off], payload[evt_off + 1],
        payload[evt_off + 2], payload[evt_off + 3],
    ]);

    let status = if evt_off + 12 <= payload.len() {
        u32::from_be_bytes([
            payload[evt_off + 8], payload[evt_off + 9],
            payload[evt_off + 10], payload[evt_off + 11],
        ])
    } else { 0 };

    match event_type {
        WLC_E_LINK => {
            if status == 0 {
                crate::kprintln!("  CYW43: EVENT link up");
                set_state(WiFiState::Connected);
            } else {
                crate::kprintln!("  CYW43: EVENT link down (status={})", status);
                set_state(WiFiState::Ready);
            }
        }
        WLC_E_AUTH => {
            crate::kprintln!("  CYW43: EVENT auth (status={})", status);
        }
        WLC_E_PSK_SUP => {
            crate::kprintln!("  CYW43: EVENT WPA handshake (status={})", status);
        }
        _ => {
            // Ignore unknown events silently
        }
    }
}

// === Public WiFi API ===

/// Initialize CYW43455 with firmware and NVRAM
pub fn init(firmware: &[u8], nvram: &[u8]) -> Result<(), &'static str> {
    set_state(WiFiState::Initializing);

    if !sdhci::is_ready() {
        return Err("SDIO not initialized");
    }

    // Enable F1 (backplane)
    sdhci::cmd52_write(0, CCCR_IO_ENABLE, 0x02)?; // Enable F1
    crate::arch::aarch64::timer::delay_ms(10);

    // Wait for F1 ready
    for _ in 0..100 {
        let ready = sdhci::cmd52_read(0, CCCR_IO_READY)?;
        if ready & 0x02 != 0 { break; }
        crate::arch::aarch64::timer::delay_ms(10);
    }

    // Set F1 block size to 64
    sdhci::cmd52_write(0, 0x110, 64)?; // F1 block size low
    sdhci::cmd52_write(0, 0x111, 0)?;  // F1 block size high

    // Read chip ID for verification
    let chip_id = bp_read32(CHIPID_ADDR)?;
    let chip_num = chip_id & 0xFFFF;
    let chip_rev = (chip_id >> 16) & 0x0F;
    crate::kprintln!("  CYW43: chip ID = BCM{} rev {}", chip_num, chip_rev);

    if chip_num != 0x4345 {
        crate::kprintln!("  CYW43: WARNING — unexpected chip {:#x}, expected 0x4345", chip_num);
    }

    // Upload firmware
    upload_firmware(firmware)?;

    // Upload NVRAM
    upload_nvram(nvram)?;

    set_state(WiFiState::FirmwareLoaded);

    // Start firmware
    start_firmware()?;

    // Enable F2 (WLAN data)
    sdhci::cmd52_write(0, CCCR_IO_ENABLE, 0x06)?; // Enable F1 + F2
    sdhci::cmd52_write(0, CCCR_INT_ENABLE, 0x07)?; // Enable interrupts for F1 + F2

    // Set F2 block size to 512
    sdhci::cmd52_write(0, 0x210, 0)?;   // F2 block size low (512 & 0xFF = 0)
    sdhci::cmd52_write(0, 0x211, 2)?;   // F2 block size high (512 >> 8 = 2)

    // Bring up the interface
    send_ioctl(WLC_UP, &[], true)?;

    set_state(WiFiState::Ready);
    crate::kprintln!("[OK] CYW43455 WiFi ready (BCM{} rev {})", chip_num, chip_rev);

    Ok(())
}

/// Connect to a WiFi network (WPA2-AES PSK)
pub fn join(ssid: &str, password: &str) -> Result<(), &'static str> {
    if state() != WiFiState::Ready && state() != WiFiState::Connected {
        return Err("WiFi not ready");
    }

    set_state(WiFiState::Connecting);
    crate::kprintln!("  CYW43: connecting to '{}'...", ssid);

    // Step 1: Set infrastructure mode (1 = STA, not AP)
    send_ioctl(WLC_SET_INFRA, &1u32.to_le_bytes(), true)?;

    // Step 2: Set auth type (0 = open system, WPA2 handled by wsec)
    send_ioctl(WLC_SET_AUTH, &0u32.to_le_bytes(), true)?;

    // Step 3: Set cipher suite (WSEC_AES = CCMP for WPA2)
    send_ioctl(WLC_SET_WSEC, &WSEC_AES.to_le_bytes(), true)?;

    // Step 4: Set WPA auth mode (WPA2-PSK = 0x80)
    send_ioctl(WLC_SET_WPA_AUTH, &WPA2_AUTH_PSK.to_le_bytes(), true)?;

    // Step 5: Set passphrase via WLC_SET_WSEC_PMK (ioctl 268)
    // wsec_pmk_t: key_len(u16) + flags(u16) + key[64]
    let mut pmk = [0u8; 68];
    let pw_bytes = password.as_bytes();
    let pw_len = pw_bytes.len().min(63);
    pmk[0..2].copy_from_slice(&(pw_len as u16).to_le_bytes());
    pmk[2..4].copy_from_slice(&1u16.to_le_bytes()); // WSEC_PASSPHRASE flag
    pmk[4..4 + pw_len].copy_from_slice(&pw_bytes[..pw_len]);
    send_ioctl(WLC_SET_WSEC_PMK, &pmk, true)?;

    // Step 6: Trigger join — WLC_SET_SSID (ioctl 26)
    // wlc_ssid_t: ssid_len(u32) + ssid[32]
    let mut ssid_buf = [0u8; 36];
    let ssid_bytes = ssid.as_bytes();
    let ssid_len = ssid_bytes.len().min(32);
    ssid_buf[0..4].copy_from_slice(&(ssid_len as u32).to_le_bytes());
    ssid_buf[4..4 + ssid_len].copy_from_slice(&ssid_bytes[..ssid_len]);
    send_ioctl(WLC_SET_SSID, &ssid_buf, true)?;

    // Step 7: Wait for LINK event (firmware handles WPA2 4-way handshake)
    crate::kprintln!("  CYW43: waiting for association + WPA2 handshake...");
    for i in 0..150 {
        crate::arch::aarch64::timer::delay_ms(100);

        // Poll for events
        if let Ok(Some((channel, payload))) = read_sdpcm_frame() {
            if channel == SDPCM_EVENT_CHANNEL {
                process_event(&payload);
            }
        }

        // Check if process_event() set us to Connected
        if state() == WiFiState::Connected {
            crate::kprintln!("  CYW43: connected to '{}' ({}ms)", ssid, (i + 1) * 100);
            return Ok(());
        }
    }

    set_state(WiFiState::Error);
    crate::kprintln!("  CYW43: association timeout after 15s");
    Err("WiFi association timeout")
}

/// Disconnect from WiFi
pub fn disconnect() -> Result<(), &'static str> {
    send_ioctl(WLC_DISASSOC, &[], true)?;
    set_state(WiFiState::Ready);
    Ok(())
}

/// Send an Ethernet frame over WiFi (called by smoltcp TX path)
pub fn send_frame(data: &[u8]) -> Result<(), &'static str> {
    if !F2_READY.load(Ordering::Relaxed) { return Err("F2 not ready"); }

    let seq = next_data_seq();

    // SDPCM + BDC + Ethernet payload
    let data_offset = SDPCM_HEADER_LEN + BDC_HEADER_LEN;
    let total_len = data_offset + data.len();
    let padded_len = (total_len + 7) & !7; // 8-byte align for SDIO
    let mut frame = alloc::vec![0u8; padded_len];

    // SDPCM header
    build_sdpcm_header(&mut frame, padded_len, SDPCM_DATA_CHANNEL, seq, data_offset as u8);

    // BDC header (4 bytes): version=2, no flags, priority=0
    frame[SDPCM_HEADER_LEN] = 0x20; // BDC version 2
    frame[SDPCM_HEADER_LEN + 1] = 0; // flags
    frame[SDPCM_HEADER_LEN + 2] = 0; // priority
    frame[SDPCM_HEADER_LEN + 3] = 0; // data offset (0 extra 32-bit words)

    // Ethernet frame
    frame[data_offset..data_offset + data.len()].copy_from_slice(data);

    sdhci::cmd53_write(2, 0, &frame, true)?;
    Ok(())
}

/// Receive an Ethernet frame from WiFi (called by smoltcp RX path)
/// Returns the frame data length or None if no frame available
pub fn recv_frame(buf: &mut [u8]) -> Option<usize> {
    if !F2_READY.load(Ordering::Relaxed) { return None; }

    let (channel, payload) = match read_sdpcm_frame() {
        Ok(Some(frame)) => frame,
        _ => return None,
    };

    match channel {
        SDPCM_DATA_CHANNEL => {
            // Data frame — strip BDC header, extract Ethernet payload
            if payload.len() < BDC_HEADER_LEN { return None; }
            let bdc_data_offset = (payload[3] as usize) * 4; // Extra words
            let eth_start = BDC_HEADER_LEN + bdc_data_offset;
            if eth_start >= payload.len() { return None; }

            let eth_data = &payload[eth_start..];
            let copy_len = eth_data.len().min(buf.len());
            buf[..copy_len].copy_from_slice(&eth_data[..copy_len]);
            Some(copy_len)
        }
        SDPCM_EVENT_CHANNEL => {
            // Event frame — process it, return no data to smoltcp
            process_event(&payload);
            None
        }
        _ => None,
    }
}

/// Poll for and process pending events (call from main loop)
pub fn poll_events() {
    if !F2_READY.load(Ordering::Relaxed) { return; }

    // Drain all pending frames, process events
    for _ in 0..10 {
        match read_sdpcm_frame() {
            Ok(Some((SDPCM_EVENT_CHANNEL, payload))) => process_event(&payload),
            Ok(Some(_)) => {} // Data frame — ignore (smoltcp will pick it up)
            _ => break,       // No more frames
        }
    }
}

/// Get WiFi MAC address from the chip
pub fn mac_address() -> Result<[u8; 6], &'static str> {
    // Read from NVRAM or via ioctl
    // For now, read from backplane at known offset
    let mac_lo = bp_read32(WLAN_BASE + 0x120)?;
    let mac_hi = bp_read32(WLAN_BASE + 0x124)?;

    Ok([
        (mac_lo & 0xFF) as u8,
        ((mac_lo >> 8) & 0xFF) as u8,
        ((mac_lo >> 16) & 0xFF) as u8,
        ((mac_lo >> 24) & 0xFF) as u8,
        (mac_hi & 0xFF) as u8,
        ((mac_hi >> 8) & 0xFF) as u8,
    ])
}
