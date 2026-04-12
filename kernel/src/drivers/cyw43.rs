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

// ioctl commands
const WLC_SET_SSID: u32     = 26;
const WLC_SET_CHANNEL: u32  = 30;
const WLC_DISASSOC: u32     = 52;
const WLC_SET_PASSIVE_SCAN: u32 = 49;
const WLC_SCAN: u32         = 50;
const WLC_SET_WSEC: u32     = 134;
const WLC_SET_WPA_AUTH: u32 = 165;
const WLC_SET_SSID_JOIN: u32 = 26;
const WLC_SET_KEY: u32      = 45;
const WLC_SET_INFRA: u32    = 20;
const WLC_SET_AUTH: u32     = 22;
const WLC_UP: u32           = 2;

// WPA/Security constants
const WSEC_TKIP: u32 = 0x02;
const WSEC_AES: u32  = 0x04;
const WPA2_AUTH_PSK: u32 = 0x80;

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

/// Sequence number for SDPCM frames
static mut IOCTL_SEQ: u16 = 0;

/// Send an ioctl command to the firmware
fn send_ioctl(cmd: u32, data: &[u8], set: bool) -> Result<Vec<u8>, &'static str> {
    if !F2_READY.load(Ordering::Relaxed) {
        return Err("F2 not ready");
    }

    // Build SDPCM + CDC header
    let total_len = SDPCM_HEADER_LEN + 16 + data.len(); // SDPCM + CDC + payload
    let mut frame = alloc::vec![0u8; total_len];

    let seq = unsafe {
        let s = IOCTL_SEQ;
        IOCTL_SEQ = s.wrapping_add(1);
        s
    };

    // SDPCM header (12 bytes)
    frame[0..2].copy_from_slice(&(total_len as u16).to_le_bytes()); // frame length
    frame[2..4].copy_from_slice(&(!total_len as u16).to_le_bytes()); // ~frame length
    frame[4] = seq as u8; // sequence number
    frame[5] = 0; // channel: control
    frame[6] = 0; // next length
    frame[7] = (SDPCM_HEADER_LEN as u8); // data offset
    frame[8] = 0; // flow control
    frame[9] = 0; // max seq
    frame[10] = 0;
    frame[11] = 0;

    // CDC header (16 bytes) at offset SDPCM_HEADER_LEN
    let cdc_off = SDPCM_HEADER_LEN;
    frame[cdc_off..cdc_off + 4].copy_from_slice(&cmd.to_le_bytes()); // command
    frame[cdc_off + 4..cdc_off + 8].copy_from_slice(&(data.len() as u32).to_le_bytes()); // output length
    let flags: u32 = if set { 0x02 } else { 0x00 } | ((seq as u32) << 16); // SET flag + id
    frame[cdc_off + 8..cdc_off + 12].copy_from_slice(&flags.to_le_bytes()); // flags
    frame[cdc_off + 12..cdc_off + 16].copy_from_slice(&0u32.to_le_bytes()); // status

    // Payload
    if !data.is_empty() {
        frame[cdc_off + 16..cdc_off + 16 + data.len()].copy_from_slice(data);
    }

    // Send via F2
    sdhci::cmd53_write(2, 0, &frame, true)?;

    // Read response (simplified — real driver needs async event loop)
    let mut resp = alloc::vec![0u8; 512];
    crate::arch::aarch64::timer::delay_ms(50);
    let _ = sdhci::cmd53_read(2, 0, &mut resp, true);

    Ok(resp)
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

/// Connect to a WiFi network
pub fn join(ssid: &str, password: &str) -> Result<(), &'static str> {
    if state() != WiFiState::Ready && state() != WiFiState::Connected {
        return Err("WiFi not ready");
    }

    set_state(WiFiState::Connecting);
    crate::kprintln!("  CYW43: connecting to '{}'...", ssid);

    // Set infrastructure mode (1 = STA)
    let infra = 1u32.to_le_bytes();
    send_ioctl(WLC_SET_INFRA, &infra, true)?;

    // Set auth mode (0 = open)
    let auth = 0u32.to_le_bytes();
    send_ioctl(WLC_SET_AUTH, &auth, true)?;

    // Set security: WPA2-AES
    let wsec = WSEC_AES.to_le_bytes();
    send_ioctl(WLC_SET_WSEC, &wsec, true)?;

    // Set WPA auth: WPA2-PSK
    let wpa_auth = WPA2_AUTH_PSK.to_le_bytes();
    send_ioctl(WLC_SET_WPA_AUTH, &wpa_auth, true)?;

    // Set passphrase via WLC_SET_KEY ioctl
    // wsec_pmk_t structure: key_len(u16) + flags(u16) + key(64 bytes)
    let mut pmk = [0u8; 68];
    let pw_bytes = password.as_bytes();
    let pw_len = pw_bytes.len().min(63);
    pmk[0..2].copy_from_slice(&(pw_len as u16).to_le_bytes());
    pmk[2..4].copy_from_slice(&1u16.to_le_bytes()); // WSEC_PASSPHRASE flag
    pmk[4..4 + pw_len].copy_from_slice(&pw_bytes[..pw_len]);
    send_ioctl(WLC_SET_KEY, &pmk, true)?;

    // Join the network: WLC_SET_SSID
    // wlc_ssid_t: ssid_len(u32) + ssid(32 bytes)
    let mut ssid_buf = [0u8; 36];
    let ssid_bytes = ssid.as_bytes();
    let ssid_len = ssid_bytes.len().min(32);
    ssid_buf[0..4].copy_from_slice(&(ssid_len as u32).to_le_bytes());
    ssid_buf[4..4 + ssid_len].copy_from_slice(&ssid_bytes[..ssid_len]);
    send_ioctl(WLC_SET_SSID_JOIN, &ssid_buf, true)?;

    // Wait for association (simplified — real driver monitors events)
    for i in 0..100 {
        crate::arch::aarch64::timer::delay_ms(100);
        // Check F2 for association event
        let mut event_buf = [0u8; 512];
        if sdhci::cmd53_read(2, 0, &mut event_buf, true).is_ok() {
            // Check if we got an association response
            // (simplified — full driver parses SDPCM + BDC event headers)
            if event_buf[0] != 0 && event_buf[1] != 0 {
                crate::kprintln!("  CYW43: associated after {}ms", (i + 1) * 100);
                set_state(WiFiState::Connected);
                return Ok(());
            }
        }
    }

    set_state(WiFiState::Error);
    Err("association timeout")
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

    // Build SDPCM data frame
    let total_len = SDPCM_HEADER_LEN + 2 + data.len(); // +2 for BDC header
    let mut frame = alloc::vec![0u8; total_len];

    let seq = unsafe {
        let s = IOCTL_SEQ;
        IOCTL_SEQ = s.wrapping_add(1);
        s
    };

    // SDPCM header
    frame[0..2].copy_from_slice(&(total_len as u16).to_le_bytes());
    frame[2..4].copy_from_slice(&(!total_len as u16).to_le_bytes());
    frame[4] = seq as u8;
    frame[5] = 2; // channel: data
    frame[6] = 0;
    frame[7] = (SDPCM_HEADER_LEN + 2) as u8; // data offset (skip BDC)

    // BDC header (2 bytes)
    frame[SDPCM_HEADER_LEN] = 0; // flags
    frame[SDPCM_HEADER_LEN + 1] = 0; // priority

    // Ethernet frame payload
    frame[SDPCM_HEADER_LEN + 2..].copy_from_slice(data);

    sdhci::cmd53_write(2, 0, &frame, true)?;
    Ok(())
}

/// Receive an Ethernet frame from WiFi (called by smoltcp RX path)
/// Returns the frame data or None if no frame available
pub fn recv_frame(buf: &mut [u8]) -> Option<usize> {
    if !F2_READY.load(Ordering::Relaxed) { return None; }

    // Check if data is available on F2
    let status = sdhci::cmd52_read(0, 0x04).ok()?; // Read interrupt status
    if status & 0x04 == 0 { return None; } // No F2 interrupt

    // Read frame from F2
    let mut raw = [0u8; 2048];
    let _ = sdhci::cmd53_read(2, 0, &mut raw, true).ok()?;

    // Parse SDPCM header
    let frame_len = u16::from_le_bytes([raw[0], raw[1]]) as usize;
    if frame_len < SDPCM_HEADER_LEN + 2 || frame_len > raw.len() {
        return None;
    }

    let channel = raw[5];
    let data_offset = raw[7] as usize;

    if channel != 2 { return None; } // Not a data frame
    if data_offset >= frame_len { return None; }

    // Extract Ethernet frame (skip SDPCM + BDC headers)
    let eth_data = &raw[data_offset..frame_len];
    let copy_len = eth_data.len().min(buf.len());
    buf[..copy_len].copy_from_slice(&eth_data[..copy_len]);

    Some(copy_len)
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
