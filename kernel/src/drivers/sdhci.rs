//! BCM2712 SDHCI Driver — SD Host Controller for WiFi SDIO
//!
//! Drives the BCM2712's internal SDHCI controller (MMC1) which connects
//! to the CYW43455 WiFi/BT chip via SDIO.
//!
//! Physical base: 0x1001100000 (BCM2712, NOT behind RP1)
//! SDHCI Spec: v3.0
//! Base clock: ~200 MHz
//! Supported modes: SDR25, SDR50, SDR104, DDR50
//!
//! This driver implements the minimal SDIO command set:
//!   CMD0  — GO_IDLE_STATE
//!   CMD5  — IO_SEND_OP_COND (SDIO-specific)
//!   CMD3  — SEND_RELATIVE_ADDR
//!   CMD7  — SELECT_CARD
//!   CMD52 — IO_RW_DIRECT (single byte read/write)
//!   CMD53 — IO_RW_EXTENDED (block/multi-byte transfer)

use core::sync::atomic::{AtomicU64, Ordering};

/// BCM2712 SDHCI physical base for MMC1 (WiFi)
pub const SDHCI_WIFI_PHYS: u64 = 0x10_0110_0000;

// SDHCI Register Offsets (SD Host Controller Simplified Spec v3.0)
const SDMA_ADDR: u64       = 0x00; // SDMA System Address
const BLOCK_SIZE: u64      = 0x04; // Block Size (bits 11:0) + Transfer Mode (bits 31:16)
const BLOCK_COUNT: u64     = 0x06; // Block count (16-bit, at offset 0x06)
const ARGUMENT: u64        = 0x08; // Command Argument
const TRANSFER_MODE: u64   = 0x0C; // Transfer Mode (16-bit, at offset 0x0C)
const COMMAND: u64         = 0x0E; // Command (16-bit, at offset 0x0E)
const RESPONSE0: u64       = 0x10; // Response bits 31:0
const RESPONSE1: u64       = 0x14; // Response bits 63:32
const RESPONSE2: u64       = 0x18; // Response bits 95:64
const RESPONSE3: u64       = 0x1C; // Response bits 127:96
const BUFFER_DATA: u64     = 0x20; // Buffer Data Port (32-bit)
const PRESENT_STATE: u64   = 0x24; // Present State
const HOST_CTRL1: u64      = 0x28; // Host Control 1
const POWER_CTRL: u64      = 0x29; // Power Control (8-bit)
const CLOCK_CTRL: u64      = 0x2C; // Clock Control (16-bit)
const TIMEOUT_CTRL: u64    = 0x2E; // Timeout Control (8-bit)
const SOFTWARE_RESET: u64  = 0x2F; // Software Reset (8-bit)
const INT_STATUS: u64      = 0x30; // Normal Interrupt Status
const INT_STATUS_EN: u64   = 0x34; // Normal Interrupt Status Enable
const INT_SIGNAL_EN: u64   = 0x38; // Normal Interrupt Signal Enable
const CAPABILITIES0: u64   = 0x40; // Capabilities 0
const CAPABILITIES1: u64   = 0x44; // Capabilities 1
const HOST_CTRL2: u64      = 0x3E; // Host Control 2 (16-bit)

// Present State bits
const PS_CMD_INHIBIT: u32    = 1 << 0;
const PS_DAT_INHIBIT: u32   = 1 << 1;
const PS_CARD_INSERTED: u32  = 1 << 16;
const PS_CARD_STABLE: u32   = 1 << 17;
const PS_BUFFER_READ_EN: u32 = 1 << 11;
const PS_BUFFER_WRITE_EN: u32 = 1 << 10;

// Interrupt Status bits
const INT_CMD_COMPLETE: u32  = 1 << 0;
const INT_XFER_COMPLETE: u32 = 1 << 1;
const INT_BUFFER_READ: u32   = 1 << 5;
const INT_BUFFER_WRITE: u32  = 1 << 4;
const INT_ERROR: u32         = 1 << 15;
const INT_CMD_TIMEOUT: u32   = 1 << 16;
const INT_CMD_CRC: u32       = 1 << 17;
const INT_DATA_TIMEOUT: u32  = 1 << 20;
const INT_DATA_CRC: u32      = 1 << 21;
const INT_ALL_ERROR: u32     = 0xFFFF_0000;

// Software Reset bits
const RESET_ALL: u8  = 1 << 0;
const RESET_CMD: u8  = 1 << 1;
const RESET_DATA: u8 = 1 << 2;

// Transfer Mode bits
const TM_DMA_EN: u16        = 1 << 0;
const TM_BLOCK_COUNT_EN: u16 = 1 << 1;
const TM_AUTO_CMD12: u16    = 1 << 2;
const TM_READ: u16          = 1 << 4;
const TM_MULTI_BLOCK: u16   = 1 << 5;

// Command register encoding
const CMD_RESP_NONE: u16  = 0b00 << 0;
const CMD_RESP_136: u16   = 0b01 << 0;
const CMD_RESP_48: u16    = 0b10 << 0;
const CMD_RESP_48B: u16   = 0b11 << 0;
const CMD_CRC_CHECK: u16  = 1 << 3;
const CMD_INDEX_CHECK: u16 = 1 << 4;
const CMD_DATA_PRESENT: u16 = 1 << 5;

/// Timeout for command completion (spin iterations)
const CMD_TIMEOUT: u32 = 1_000_000;

static SDHCI_BASE: AtomicU64 = AtomicU64::new(0);

/// SDIO card state after enumeration
pub struct SdioCard {
    pub rca: u16,           // Relative Card Address
    pub num_functions: u8,  // Number of SDIO functions (1=backplane, 2=WLAN)
    pub ready: bool,
}

static mut CARD: Option<SdioCard> = None;

// MMIO helpers
unsafe fn reg_read32(offset: u64) -> u32 {
    let base = SDHCI_BASE.load(Ordering::Relaxed);
    core::ptr::read_volatile((base + offset) as *const u32)
}

unsafe fn reg_write32(offset: u64, val: u32) {
    let base = SDHCI_BASE.load(Ordering::Relaxed);
    core::ptr::write_volatile((base + offset) as *mut u32, val);
}

unsafe fn reg_read16(offset: u64) -> u16 {
    let base = SDHCI_BASE.load(Ordering::Relaxed);
    core::ptr::read_volatile((base + offset) as *const u16)
}

unsafe fn reg_write16(offset: u64, val: u16) {
    let base = SDHCI_BASE.load(Ordering::Relaxed);
    core::ptr::write_volatile((base + offset) as *mut u16, val);
}

unsafe fn reg_read8(offset: u64) -> u8 {
    let base = SDHCI_BASE.load(Ordering::Relaxed);
    core::ptr::read_volatile((base + offset) as *const u8)
}

unsafe fn reg_write8(offset: u64, val: u8) {
    let base = SDHCI_BASE.load(Ordering::Relaxed);
    core::ptr::write_volatile((base + offset) as *mut u8, val);
}

/// Wait for command inhibit to clear
unsafe fn wait_cmd_ready() -> bool {
    for _ in 0..CMD_TIMEOUT {
        if reg_read32(PRESENT_STATE) & PS_CMD_INHIBIT == 0 {
            return true;
        }
        core::hint::spin_loop();
    }
    false
}

/// Wait for data inhibit to clear
unsafe fn wait_data_ready() -> bool {
    for _ in 0..CMD_TIMEOUT {
        if reg_read32(PRESENT_STATE) & PS_DAT_INHIBIT == 0 {
            return true;
        }
        core::hint::spin_loop();
    }
    false
}

/// Wait for interrupt status bit(s) and clear them
unsafe fn wait_interrupt(mask: u32) -> Result<u32, &'static str> {
    for _ in 0..CMD_TIMEOUT {
        let status = reg_read32(INT_STATUS);
        if status & INT_ALL_ERROR != 0 {
            reg_write32(INT_STATUS, status); // Clear all
            return Err("SDHCI error interrupt");
        }
        if status & mask != 0 {
            reg_write32(INT_STATUS, status & mask); // Clear matched bits
            return Ok(status);
        }
        core::hint::spin_loop();
    }
    Err("SDHCI timeout")
}

/// Send an SDIO command and return the response
unsafe fn send_command(cmd_index: u8, argument: u32, resp_type: u16, data: bool) -> Result<u32, &'static str> {
    if !wait_cmd_ready() { return Err("CMD inhibit timeout"); }

    // Clear pending interrupts
    reg_write32(INT_STATUS, 0xFFFF_FFFF);

    // Set argument
    reg_write32(ARGUMENT, argument);

    // Build command register value
    let mut cmd_val = ((cmd_index as u16) << 8) | resp_type;
    if data {
        cmd_val |= CMD_DATA_PRESENT;
    }

    // Send command
    reg_write16(COMMAND, cmd_val);

    // Wait for completion
    wait_interrupt(INT_CMD_COMPLETE)?;

    // Read response
    Ok(reg_read32(RESPONSE0))
}

/// Initialize the SDHCI controller for SDIO operation
pub fn init() -> Result<(), &'static str> {
    // Map SDHCI registers via HHDM
    let base = crate::phys_to_virt(SDHCI_WIFI_PHYS);
    SDHCI_BASE.store(base, Ordering::SeqCst);

    crate::kprintln!("  SDHCI: WiFi controller at phys {:#x}, virt {:#x}",
        SDHCI_WIFI_PHYS, base);

    unsafe {
        // Read capabilities
        let caps0 = reg_read32(CAPABILITIES0);
        let caps1 = reg_read32(CAPABILITIES1);
        crate::kprintln!("  SDHCI: caps0={:#010x} caps1={:#010x}", caps0, caps1);

        // Software reset
        reg_write8(SOFTWARE_RESET, RESET_ALL);
        for _ in 0..CMD_TIMEOUT {
            if reg_read8(SOFTWARE_RESET) & RESET_ALL == 0 { break; }
            core::hint::spin_loop();
        }

        // Enable all interrupt status
        reg_write32(INT_STATUS_EN, 0xFFFF_FFFF);
        reg_write32(INT_SIGNAL_EN, 0); // Polling mode, no signal IRQs

        // Set timeout to maximum
        reg_write8(TIMEOUT_CTRL, 0x0E);

        // Power on: 3.3V (bits 3:1 = 0b111, bit 0 = enable)
        reg_write8(POWER_CTRL, 0x0F);

        // Set clock: enable internal clock, wait for stable, then enable SD clock
        // Use a slow clock first (400 kHz) for enumeration
        // Divider = base_clock / (2 * target) → 200MHz / (2 * 400kHz) = 250
        let divider: u16 = 250;
        let clock_val = ((divider & 0xFF) << 8) | (((divider >> 8) & 0x03) << 6) | 0x01; // Internal clock enable
        reg_write16(CLOCK_CTRL, clock_val);

        // Wait for internal clock stable
        for _ in 0..CMD_TIMEOUT {
            if reg_read16(CLOCK_CTRL) & 0x02 != 0 { break; }
            core::hint::spin_loop();
        }

        // Enable SD clock output
        reg_write16(CLOCK_CTRL, reg_read16(CLOCK_CTRL) | 0x04);

        crate::kprintln!("  SDHCI: clock enabled (400 kHz enumeration)");

        // Set bus width to 1-bit initially
        let hc1 = reg_read8(HOST_CTRL1 as u64);
        reg_write8(HOST_CTRL1 as u64, hc1 & !0x02); // Clear 4-bit mode

        // === SDIO Enumeration ===

        // CMD0 — GO_IDLE_STATE (no response)
        crate::arch::aarch64::timer::delay_ms(10);
        let _ = send_command(0, 0, CMD_RESP_NONE, false);
        crate::arch::aarch64::timer::delay_ms(10);

        // CMD5 — IO_SEND_OP_COND (SDIO only)
        // Argument 0 first to query voltage
        let resp = send_command(5, 0, CMD_RESP_48, false)?;
        let num_funcs = ((resp >> 28) & 0x07) as u8;
        let io_ocr = resp & 0x00FF_FFFF;
        crate::kprintln!("  SDIO: {} functions, OCR={:#08x}", num_funcs, io_ocr);

        // CMD5 again with matching voltage (3.3V = bit 20+21)
        let resp = send_command(5, io_ocr, CMD_RESP_48, false)?;
        if resp & (1 << 31) == 0 {
            return Err("SDIO card not ready after CMD5");
        }
        crate::kprintln!("  SDIO: card ready, {} functions", num_funcs);

        // CMD3 — SEND_RELATIVE_ADDR
        let resp = send_command(3, 0, CMD_RESP_48 | CMD_CRC_CHECK | CMD_INDEX_CHECK, false)?;
        let rca = (resp >> 16) as u16;
        crate::kprintln!("  SDIO: RCA = {:#06x}", rca);

        // CMD7 — SELECT_CARD
        let _ = send_command(7, (rca as u32) << 16, CMD_RESP_48B | CMD_CRC_CHECK | CMD_INDEX_CHECK, false)?;
        crate::kprintln!("  SDIO: card selected");

        // Switch to 4-bit bus width via SDIO CMD52
        // Write to CCCR register 0x07 (Bus Interface Control): bit 1:0 = 0b10 (4-bit)
        let current = cmd52_read(0, 0x07)?;
        cmd52_write(0, 0x07, (current & !0x03) | 0x02)?;

        // Set host controller to 4-bit mode
        let hc1 = reg_read8(HOST_CTRL1 as u64);
        reg_write8(HOST_CTRL1 as u64, hc1 | 0x02);
        crate::kprintln!("  SDIO: switched to 4-bit bus");

        // Increase clock to 25 MHz for data transfers
        // Divider = 200MHz / (2 * 25MHz) = 4
        reg_write16(CLOCK_CTRL, reg_read16(CLOCK_CTRL) & !0x04); // Disable SD clock
        let divider: u16 = 4;
        let clock_val = ((divider & 0xFF) << 8) | (((divider >> 8) & 0x03) << 6) | 0x01;
        reg_write16(CLOCK_CTRL, clock_val);
        for _ in 0..CMD_TIMEOUT {
            if reg_read16(CLOCK_CTRL) & 0x02 != 0 { break; }
            core::hint::spin_loop();
        }
        reg_write16(CLOCK_CTRL, reg_read16(CLOCK_CTRL) | 0x04);
        crate::kprintln!("  SDIO: clock raised to 25 MHz");

        // Store card info
        CARD = Some(SdioCard {
            rca,
            num_functions: num_funcs,
            ready: true,
        });

        crate::kprintln!("[OK] SDIO WiFi enumerated: CYW43455 ({} functions)", num_funcs);
    }

    Ok(())
}

// === CMD52 — IO_RW_DIRECT (single byte) ===

/// Read a single byte from an SDIO function register
pub fn cmd52_read(function: u8, addr: u32) -> Result<u8, &'static str> {
    // CMD52 argument: [31] R/W=0 (read), [30:28] function, [25:9] register, [7:0] write_data
    let arg = ((function as u32 & 0x07) << 28) | ((addr & 0x1FFFF) << 9);
    let resp = unsafe { send_command(52, arg, CMD_RESP_48 | CMD_CRC_CHECK | CMD_INDEX_CHECK, false)? };
    Ok((resp & 0xFF) as u8)
}

/// Write a single byte to an SDIO function register
pub fn cmd52_write(function: u8, addr: u32, data: u8) -> Result<u8, &'static str> {
    // CMD52 argument: [31] R/W=1 (write), [30:28] function, [25:9] register, [7:0] write_data
    let arg = (1u32 << 31) | ((function as u32 & 0x07) << 28) | ((addr & 0x1FFFF) << 9) | (data as u32);
    let resp = unsafe { send_command(52, arg, CMD_RESP_48 | CMD_CRC_CHECK | CMD_INDEX_CHECK, false)? };
    Ok((resp & 0xFF) as u8)
}

// === CMD53 — IO_RW_EXTENDED (block/multi-byte) ===

/// Read multiple bytes from an SDIO function using CMD53
pub fn cmd53_read(function: u8, addr: u32, buf: &mut [u8], increment_addr: bool) -> Result<usize, &'static str> {
    if buf.is_empty() { return Ok(0); }

    let block_size = 64u16; // CYW43455 uses 64-byte blocks
    let blocks = ((buf.len() + block_size as usize - 1) / block_size as usize) as u16;

    unsafe {
        if !wait_data_ready() { return Err("Data inhibit timeout"); }

        // Set block size and count
        reg_write16(BLOCK_SIZE as u64, block_size);
        reg_write16(BLOCK_COUNT as u64, blocks);

        // Transfer mode: read, block count enable, multi-block if > 1
        let mut tm: u16 = TM_READ | TM_BLOCK_COUNT_EN;
        if blocks > 1 { tm |= TM_MULTI_BLOCK; }
        reg_write16(TRANSFER_MODE, tm);

        // CMD53 argument
        let inc = if increment_addr { 1u32 << 26 } else { 0 };
        let block_mode = 1u32 << 27; // Block mode
        let arg = ((function as u32 & 0x07) << 28) | inc | block_mode
            | ((addr & 0x1FFFF) << 9) | (blocks as u32 & 0x1FF);

        send_command(53, arg, CMD_RESP_48 | CMD_CRC_CHECK | CMD_INDEX_CHECK, true)?;

        // Read data from buffer
        let mut offset = 0;
        for _ in 0..blocks {
            wait_interrupt(INT_BUFFER_READ)?;

            for _ in 0..(block_size / 4) {
                let word = reg_read32(BUFFER_DATA);
                let bytes = word.to_le_bytes();
                for &b in &bytes {
                    if offset < buf.len() {
                        buf[offset] = b;
                        offset += 1;
                    }
                }
            }
        }

        wait_interrupt(INT_XFER_COMPLETE)?;
    }

    Ok(buf.len())
}

/// Write multiple bytes to an SDIO function using CMD53
pub fn cmd53_write(function: u8, addr: u32, data: &[u8], increment_addr: bool) -> Result<usize, &'static str> {
    if data.is_empty() { return Ok(0); }

    let block_size = 64u16;
    let blocks = ((data.len() + block_size as usize - 1) / block_size as usize) as u16;

    unsafe {
        if !wait_data_ready() { return Err("Data inhibit timeout"); }

        reg_write16(BLOCK_SIZE as u64, block_size);
        reg_write16(BLOCK_COUNT as u64, blocks);

        // Transfer mode: write, block count enable
        let mut tm: u16 = TM_BLOCK_COUNT_EN;
        if blocks > 1 { tm |= TM_MULTI_BLOCK; }
        reg_write16(TRANSFER_MODE, tm);

        // CMD53 argument (write)
        let inc = if increment_addr { 1u32 << 26 } else { 0 };
        let block_mode = 1u32 << 27;
        let arg = (1u32 << 31) | ((function as u32 & 0x07) << 28) | inc | block_mode
            | ((addr & 0x1FFFF) << 9) | (blocks as u32 & 0x1FF);

        send_command(53, arg, CMD_RESP_48 | CMD_CRC_CHECK | CMD_INDEX_CHECK, true)?;

        // Write data to buffer
        let mut offset = 0;
        for _ in 0..blocks {
            wait_interrupt(INT_BUFFER_WRITE)?;

            for _ in 0..(block_size / 4) {
                let mut word_bytes = [0u8; 4];
                for b in &mut word_bytes {
                    *b = if offset < data.len() { data[offset] } else { 0 };
                    offset += 1;
                }
                reg_write32(BUFFER_DATA, u32::from_le_bytes(word_bytes));
            }
        }

        wait_interrupt(INT_XFER_COMPLETE)?;
    }

    Ok(data.len())
}

/// Check if WiFi SDIO card is initialized
pub fn is_ready() -> bool {
    unsafe { CARD.as_ref().map_or(false, |c| c.ready) }
}
