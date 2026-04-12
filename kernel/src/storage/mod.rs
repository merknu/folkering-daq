//! Storage subsystem — Block device access for persistent data
//!
//! Provides a BlockDevice trait and implementations for:
//!   - SD card via RP1 DWC MSHC (BAR1 + 0x180000)
//!   - Future: NVMe via PCIe
//!
//! Used by Synapse (knowledge graph) for non-volatile persistence.

pub mod sdcard;

/// Block device trait — 512-byte sector I/O
pub trait BlockDevice {
    /// Read one 512-byte sector at the given LBA
    fn read_block(&self, lba: u64, buf: &mut [u8; 512]) -> Result<(), &'static str>;

    /// Write one 512-byte sector at the given LBA
    fn write_block(&self, lba: u64, buf: &[u8; 512]) -> Result<(), &'static str>;

    /// Total number of 512-byte sectors
    fn sector_count(&self) -> u64;
}
