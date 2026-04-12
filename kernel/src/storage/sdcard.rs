//! SD Card Driver via RP1 DWC MSHC (Synopsys DesignWare Mobile Storage Host Controller)
//!
//! RP1 has two SDHCI/MSHC controllers:
//!   MMC0: BAR1 + 0x180000 — connected to microSD card slot
//!   MMC1: BAR1 + 0x184000 — connected to SDIO WiFi (not used here)
//!
//! The DWC MSHC is SDHCI-compatible for basic operations.
//! We use it in SDHCI mode for simplicity (CMD17 read, CMD24 write).
//!
//! The SD card boot partition (FAT32, 512MB) is at sector 16384.
//! The Linux root partition (ext4, 59GB) starts at sector 1064960.
//! For DAQ data, we use a dedicated region after the Linux partition,
//! or we write to the FAT32 boot partition.

use core::sync::atomic::{AtomicU64, Ordering};
use super::BlockDevice;

/// RP1 MMC0 AXI offset from BAR1
const MMC0_OFFSET: u64 = 0x18_0000;

// SDHCI registers (same as sdhci.rs but for a different controller)
const SDMA_ADDR: u64    = 0x00;
const BLOCK_SIZE: u64   = 0x04;
const BLOCK_COUNT: u64  = 0x06;
const ARGUMENT: u64     = 0x08;
const XFER_MODE: u64    = 0x0C;
const COMMAND: u64      = 0x0E;
const RESPONSE0: u64    = 0x10;
const BUFFER_DATA: u64  = 0x20;
const PRESENT_STATE: u64 = 0x24;
const INT_STATUS: u64   = 0x30;
const INT_STATUS_EN: u64 = 0x34;

const PS_CMD_INHIBIT: u32  = 1 << 0;
const PS_DAT_INHIBIT: u32  = 1 << 1;
const PS_BUF_READ_EN: u32  = 1 << 11;
const PS_BUF_WRITE_EN: u32 = 1 << 10;
const PS_CARD_INSERTED: u32 = 1 << 16;

const INT_CMD_COMPLETE: u32  = 1 << 0;
const INT_XFER_COMPLETE: u32 = 1 << 1;
const INT_BUF_READ: u32     = 1 << 5;
const INT_BUF_WRITE: u32    = 1 << 4;
const INT_ERROR: u32         = 1 << 15;

const CMD_RESP_48: u16       = 0b10;
const CMD_CRC_CHECK: u16     = 1 << 3;
const CMD_INDEX_CHECK: u16   = 1 << 4;
const CMD_DATA_PRESENT: u16  = 1 << 5;

const TIMEOUT: u32 = 500_000;

static SD_BASE: AtomicU64 = AtomicU64::new(0);
static SD_SECTORS: AtomicU64 = AtomicU64::new(0);

unsafe fn reg_read32(offset: u64) -> u32 {
    let base = SD_BASE.load(Ordering::Relaxed);
    core::ptr::read_volatile((base + offset) as *const u32)
}

unsafe fn reg_write32(offset: u64, val: u32) {
    let base = SD_BASE.load(Ordering::Relaxed);
    core::ptr::write_volatile((base + offset) as *mut u32, val);
}

unsafe fn reg_write16(offset: u64, val: u16) {
    let base = SD_BASE.load(Ordering::Relaxed);
    core::ptr::write_volatile((base + offset) as *mut u16, val);
}

unsafe fn wait_cmd() -> bool {
    for _ in 0..TIMEOUT {
        if reg_read32(PRESENT_STATE) & PS_CMD_INHIBIT == 0 { return true; }
        core::hint::spin_loop();
    }
    false
}

unsafe fn wait_data() -> bool {
    for _ in 0..TIMEOUT {
        if reg_read32(PRESENT_STATE) & PS_DAT_INHIBIT == 0 { return true; }
        core::hint::spin_loop();
    }
    false
}

unsafe fn wait_int(mask: u32) -> Result<(), &'static str> {
    for _ in 0..TIMEOUT {
        let status = reg_read32(INT_STATUS);
        if status & INT_ERROR != 0 {
            reg_write32(INT_STATUS, status);
            return Err("SD error");
        }
        if status & mask != 0 {
            reg_write32(INT_STATUS, status & mask);
            return Ok(());
        }
        core::hint::spin_loop();
    }
    Err("SD timeout")
}

unsafe fn send_cmd(index: u8, arg: u32, resp: u16, data: bool) -> Result<u32, &'static str> {
    if !wait_cmd() { return Err("SD CMD inhibit"); }
    reg_write32(INT_STATUS, 0xFFFF_FFFF);
    reg_write32(ARGUMENT, arg);
    let mut cmd = ((index as u16) << 8) | resp;
    if data { cmd |= CMD_DATA_PRESENT; }
    reg_write16(COMMAND, cmd);
    wait_int(INT_CMD_COMPLETE)?;
    Ok(reg_read32(RESPONSE0))
}

/// SD Card block device
pub struct SdCard;

/// Initialize SD card on RP1 MMC0
pub fn init(bar1_virt: u64) -> Result<SdCard, &'static str> {
    let base = bar1_virt + MMC0_OFFSET;
    SD_BASE.store(base, Ordering::SeqCst);

    unsafe {
        // Check card is inserted
        let state = reg_read32(PRESENT_STATE);
        if state & PS_CARD_INSERTED == 0 {
            return Err("SD card not inserted");
        }

        // Enable all interrupt status bits
        reg_write32(INT_STATUS_EN, 0xFFFF_FFFF);

        // Note: Linux has already initialized the SD card (enumerated, set bus width, etc.)
        // In bare-metal mode, we'd need CMD0 → CMD8 → ACMD41 → CMD2 → CMD3 → CMD7
        // For now, assume card is already initialized by firmware/bootloader

        crate::kprintln!("  SD: RP1 MMC0 at {:#x}, card present", base);
    }

    Ok(SdCard)
}

impl BlockDevice for SdCard {
    /// Read a 512-byte sector (CMD17 — READ_SINGLE_BLOCK)
    fn read_block(&self, lba: u64, buf: &mut [u8; 512]) -> Result<(), &'static str> {
        unsafe {
            if !wait_data() { return Err("SD data inhibit"); }

            // Set block size = 512, count = 1
            reg_write16(BLOCK_SIZE as u64, 512);
            reg_write16(BLOCK_COUNT as u64, 1);

            // Transfer mode: read, single block
            reg_write16(XFER_MODE, 1 << 4); // Read

            // CMD17 (READ_SINGLE_BLOCK), arg = LBA (for SDHC cards, sector address)
            send_cmd(17, lba as u32, CMD_RESP_48 | CMD_CRC_CHECK | CMD_INDEX_CHECK, true)?;

            // Wait for buffer read ready
            wait_int(INT_BUF_READ)?;

            // Read 128 × 32-bit words = 512 bytes
            for i in 0..128 {
                let word = reg_read32(BUFFER_DATA);
                let bytes = word.to_le_bytes();
                buf[i * 4] = bytes[0];
                buf[i * 4 + 1] = bytes[1];
                buf[i * 4 + 2] = bytes[2];
                buf[i * 4 + 3] = bytes[3];
            }

            wait_int(INT_XFER_COMPLETE)?;
        }
        Ok(())
    }

    /// Write a 512-byte sector (CMD24 — WRITE_BLOCK)
    fn write_block(&self, lba: u64, buf: &[u8; 512]) -> Result<(), &'static str> {
        unsafe {
            if !wait_data() { return Err("SD data inhibit"); }

            reg_write16(BLOCK_SIZE as u64, 512);
            reg_write16(BLOCK_COUNT as u64, 1);

            // Transfer mode: write, single block
            reg_write16(XFER_MODE, 0); // Write (no read flag)

            // CMD24 (WRITE_BLOCK)
            send_cmd(24, lba as u32, CMD_RESP_48 | CMD_CRC_CHECK | CMD_INDEX_CHECK, true)?;

            // Wait for buffer write ready
            wait_int(INT_BUF_WRITE)?;

            // Write 128 × 32-bit words
            for i in 0..128 {
                let word = u32::from_le_bytes([buf[i*4], buf[i*4+1], buf[i*4+2], buf[i*4+3]]);
                reg_write32(BUFFER_DATA, word);
            }

            wait_int(INT_XFER_COMPLETE)?;

            // Wait for card to finish programming (CMD13 poll or just delay)
            crate::arch::aarch64::timer::delay_ms(2);
        }
        Ok(())
    }

    fn sector_count(&self) -> u64 {
        SD_SECTORS.load(Ordering::Relaxed)
    }
}
