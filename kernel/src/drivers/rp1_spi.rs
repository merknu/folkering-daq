//! RP1 SPI Driver (DesignWare APB SSI) for Raspberry Pi 5
//!
//! SPI master controllers inside RP1 south bridge.
//! Compatible: Synopsys DesignWare APB SSI (snps,dw-apb-ssi)
//!
//! RP1 SPI instances (AXI offsets from BAR1):
//!   SPI0: 0x50000 (header pins: SCLK=GPIO11, MOSI=GPIO10, MISO=GPIO9, CE0=GPIO8, CE1=GPIO7)
//!   SPI1: 0x54000
//!   SPI2: 0x58000
//!   SPI3: 0x5C000
//!   SPI4: 0x60000
//!   SPI5: 0x64000
//!
//! For DAQ: SPI0 is the primary interface, directly accessible on the 40-pin header.

use core::sync::atomic::{AtomicU64, Ordering};

// RP1 SPI0 AXI offset from BAR1
const SPI0_OFFSET: u64 = 0x5_0000;

// DesignWare SSI Register Map (relative to SPI base)
const CTRLR0: u64   = 0x00; // Control Register 0
const CTRLR1: u64   = 0x04; // Control Register 1 (number of data frames for RX)
const SSIENR: u64   = 0x08; // SSI Enable (0=disable, 1=enable)
const SER: u64      = 0x10; // Slave Enable Register (bit per CS line)
const BAUDR: u64    = 0x14; // Baud Rate Divisor (SCLK = clk_ssi / BAUDR)
const TXFTLR: u64   = 0x18; // Transmit FIFO Threshold Level
const RXFTLR: u64   = 0x1C; // Receive FIFO Threshold Level
const TXFLR: u64    = 0x20; // Transmit FIFO Level (number of entries)
const RXFLR: u64    = 0x24; // Receive FIFO Level (number of entries)
const SR: u64       = 0x28; // Status Register
const IMR: u64      = 0x2C; // Interrupt Mask Register
const DR: u64       = 0x60; // Data Register (TX/RX FIFO access)

// Status Register bits
const SR_BUSY: u32  = 1 << 0; // SSI is transferring
const SR_TFNF: u32  = 1 << 1; // TX FIFO not full
const SR_TFE: u32   = 1 << 2; // TX FIFO empty
const SR_RFNE: u32  = 1 << 3; // RX FIFO not empty

// CTRLR0 bits
const CTRLR0_DFS_MASK: u32 = 0x1F << 16; // Data Frame Size (bits 20:16 for DW SSI v4+)
const CTRLR0_TMOD_MASK: u32 = 0x3 << 8;  // Transfer Mode
const CTRLR0_TMOD_TXRX: u32 = 0x0 << 8;  // TX and RX
const CTRLR0_TMOD_TX: u32   = 0x1 << 8;  // TX only
const CTRLR0_TMOD_RX: u32   = 0x2 << 8;  // RX only
const CTRLR0_SCPOL: u32     = 1 << 7;    // Serial Clock Polarity
const CTRLR0_SCPH: u32      = 1 << 6;    // Serial Clock Phase
const CTRLR0_FRF_SPI: u32   = 0x0 << 4;  // Frame format: Motorola SPI

/// Maximum transfer size per call (FIFO depth is typically 64 on RP1)
const FIFO_DEPTH: usize = 64;

static SPI_BASE: AtomicU64 = AtomicU64::new(0);

/// SPI clock mode (CPOL/CPHA)
#[derive(Clone, Copy, Debug)]
pub enum SpiMode {
    Mode0, // CPOL=0, CPHA=0
    Mode1, // CPOL=0, CPHA=1
    Mode2, // CPOL=1, CPHA=0
    Mode3, // CPOL=1, CPHA=1
}

/// SPI configuration
pub struct SpiConfig {
    pub clock_div: u16,    // SCLK = clk_ssi / clock_div (must be even, min 2)
    pub mode: SpiMode,
    pub bits_per_word: u8, // 4-32 bits
}

impl Default for SpiConfig {
    fn default() -> Self {
        SpiConfig {
            clock_div: 100,     // ~2 MHz assuming 200 MHz clk_ssi
            mode: SpiMode::Mode0,
            bits_per_word: 8,
        }
    }
}

// MMIO helpers
unsafe fn mmio_read(addr: u64) -> u32 {
    core::ptr::read_volatile(addr as *const u32)
}

unsafe fn mmio_write(addr: u64, val: u32) {
    core::ptr::write_volatile(addr as *mut u32, val);
}

/// Initialize SPI0 with the given RP1 BAR1 virtual base
pub fn init(bar1_virt: u64, config: &SpiConfig) {
    let base = bar1_virt + SPI0_OFFSET;
    SPI_BASE.store(base, Ordering::SeqCst);

    unsafe {
        // Disable SSI before configuration
        mmio_write(base + SSIENR, 0);

        // Configure CTRLR0: frame size, mode, SPI format
        let dfs = ((config.bits_per_word as u32 - 1) & 0x1F) << 16;
        let (cpol, cpha) = match config.mode {
            SpiMode::Mode0 => (0, 0),
            SpiMode::Mode1 => (0, CTRLR0_SCPH),
            SpiMode::Mode2 => (CTRLR0_SCPOL, 0),
            SpiMode::Mode3 => (CTRLR0_SCPOL, CTRLR0_SCPH),
        };
        mmio_write(base + CTRLR0, dfs | cpol | cpha | CTRLR0_FRF_SPI | CTRLR0_TMOD_TXRX);

        // Set baud rate
        let div = if config.clock_div < 2 { 2 } else { config.clock_div & !1 }; // Must be even
        mmio_write(base + BAUDR, div as u32);

        // Disable all interrupts (polling mode)
        mmio_write(base + IMR, 0);

        // Set FIFO thresholds
        mmio_write(base + TXFTLR, 0);
        mmio_write(base + RXFTLR, 0);

        // Enable SSI
        mmio_write(base + SSIENR, 1);
    }

    crate::kprintln!("  SPI0: RP1 DW-SSI at {:#x}, div={}, {}bpp, mode {:?}",
        base, config.clock_div, config.bits_per_word, config.mode);

    // Configure GPIO pins for SPI0 (Alt0 function on RP1)
    super::rp1_gpio::set_mode(7, super::rp1_gpio::PinMode::Alt0);  // CE1
    super::rp1_gpio::set_mode(8, super::rp1_gpio::PinMode::Alt0);  // CE0
    super::rp1_gpio::set_mode(9, super::rp1_gpio::PinMode::Alt0);  // MISO
    super::rp1_gpio::set_mode(10, super::rp1_gpio::PinMode::Alt0); // MOSI
    super::rp1_gpio::set_mode(11, super::rp1_gpio::PinMode::Alt0); // SCLK
}

/// Full-duplex SPI transfer: sends `tx` bytes, receives into `rx`.
/// Both slices must be the same length.
/// Returns number of bytes transferred.
pub fn transfer(tx: &[u8], rx: &mut [u8]) -> usize {
    let base = SPI_BASE.load(Ordering::Relaxed);
    if base == 0 { return 0; }

    let len = tx.len().min(rx.len());
    if len == 0 { return 0; }

    unsafe {
        // Select slave 0 (CE0)
        mmio_write(base + SER, 1);

        let mut tx_idx = 0;
        let mut rx_idx = 0;

        while rx_idx < len {
            // Fill TX FIFO
            while tx_idx < len {
                let sr = mmio_read(base + SR);
                if sr & SR_TFNF == 0 { break; } // TX FIFO full
                mmio_write(base + DR, tx[tx_idx] as u32);
                tx_idx += 1;
            }

            // Drain RX FIFO
            while rx_idx < tx_idx {
                let sr = mmio_read(base + SR);
                if sr & SR_RFNE == 0 { break; } // RX FIFO empty
                rx[rx_idx] = mmio_read(base + DR) as u8;
                rx_idx += 1;
            }
        }

        // Wait for transfer to complete
        while mmio_read(base + SR) & SR_BUSY != 0 {
            core::hint::spin_loop();
        }

        // Drain any remaining RX data
        while mmio_read(base + SR) & SR_RFNE != 0 && rx_idx < len {
            rx[rx_idx] = mmio_read(base + DR) as u8;
            rx_idx += 1;
        }

        // Deselect slave
        mmio_write(base + SER, 0);

        rx_idx
    }
}

/// Write-only SPI transfer (discard RX data)
pub fn write(data: &[u8]) -> usize {
    let base = SPI_BASE.load(Ordering::Relaxed);
    if base == 0 { return 0; }

    unsafe {
        // Switch to TX-only mode
        mmio_write(base + SSIENR, 0);
        let ctrl = mmio_read(base + CTRLR0);
        mmio_write(base + CTRLR0, (ctrl & !CTRLR0_TMOD_MASK) | CTRLR0_TMOD_TX);
        mmio_write(base + SSIENR, 1);

        mmio_write(base + SER, 1);

        for &byte in data {
            while mmio_read(base + SR) & SR_TFNF == 0 {
                core::hint::spin_loop();
            }
            mmio_write(base + DR, byte as u32);
        }

        // Wait for completion
        while mmio_read(base + SR) & SR_BUSY != 0 {
            core::hint::spin_loop();
        }

        mmio_write(base + SER, 0);

        // Restore TX+RX mode
        mmio_write(base + SSIENR, 0);
        mmio_write(base + CTRLR0, (ctrl & !CTRLR0_TMOD_MASK) | CTRLR0_TMOD_TXRX);
        mmio_write(base + SSIENR, 1);
    }

    data.len()
}

/// Read SPI register (for debugging)
pub fn read_status() -> u32 {
    let base = SPI_BASE.load(Ordering::Relaxed);
    if base == 0 { return 0; }
    unsafe { mmio_read(base + SR) }
}
