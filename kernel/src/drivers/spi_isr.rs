//! SPI Interrupt Service Routine — Zero-Copy to SPSC Ringbuffer
//!
//! Connects the RP1 SPI0 receive interrupt to the DAQ ring buffer.
//! When an external ADC (e.g. MCP3008, ADS1256) delivers data via SPI,
//! the ISR reads the SPI RX FIFO and pushes samples directly into the
//! lock-free SPSC ring buffer without any CPU polling.
//!
//! Interrupt source:
//!   BCM2712 internal SPI: GIC IRQ 150 (spi@107d004000)
//!   RP1 SPI: via RP1 MSI-X interrupt chip
//!
//! The ISR is designed to be as fast as possible:
//!   1. Read SPI status register
//!   2. While RX FIFO not empty: read DR → convert to f32 → push to ringbuffer
//!   3. Clear interrupt
//!   4. Return (no locking, no allocation)

use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};

/// GIC IRQ number for BCM2712 internal SPI0
const SPI_IRQ: u32 = 150;

/// SPI RX FIFO threshold (trigger interrupt after N samples)
const RX_THRESHOLD: u32 = 8;

/// Whether ISR-driven mode is active
static ISR_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Counter for ISR invocations (telemetry)
static ISR_COUNT: AtomicU64 = AtomicU64::new(0);

/// Counter for samples delivered via ISR
static SAMPLE_COUNT: AtomicU64 = AtomicU64::new(0);

/// Target channel in the ring buffer (0 = time domain, 1-8 = ADC channels)
static mut TARGET_CHANNEL: usize = 1;

/// ADC scale factor (raw → f32 normalized)
/// Default: 12-bit ADC → /4096.0, 16-bit → /32768.0
static mut ADC_SCALE: f32 = 1.0 / 4096.0;

/// Enable interrupt-driven SPI → Ringbuffer pipeline
///
/// Call after SPI and GIC are initialized.
/// `channel`: which ringbuffer channel to write to (1-8 for ADC channels)
/// `adc_bits`: ADC resolution (12, 14, 16) for normalization
pub fn enable(channel: usize, adc_bits: u8) {
    unsafe {
        TARGET_CHANNEL = channel;
        ADC_SCALE = 1.0 / ((1u32 << (adc_bits - 1)) as f32); // Signed normalization to ±1.0
    }

    // Configure SPI RX interrupt threshold
    let spi_base = super::rp1_spi::spi_base();
    if spi_base != 0 {
        unsafe {
            // Set RX FIFO threshold level (trigger when FIFO has >= N entries)
            core::ptr::write_volatile((spi_base + 0x1C) as *mut u32, RX_THRESHOLD);

            // Enable RX Full interrupt in IMR (Interrupt Mask Register)
            // Bit 4 = RXFIM (Receive FIFO Full Interrupt Mask)
            let imr = core::ptr::read_volatile((spi_base + 0x2C) as *const u32);
            core::ptr::write_volatile((spi_base + 0x2C) as *mut u32, imr | (1 << 4));
        }
    }

    // Register ISR with GIC
    crate::arch::aarch64::gic::register_handler(SPI_IRQ, spi_rx_isr);
    crate::arch::aarch64::gic::enable_irq(SPI_IRQ);

    ISR_ACTIVE.store(true, Ordering::SeqCst);

    crate::kprintln!("  SPI_ISR: enabled on GIC IRQ {}, channel {}, {} bits",
        SPI_IRQ, channel, adc_bits);
}

/// Disable interrupt-driven SPI
pub fn disable() {
    crate::arch::aarch64::gic::disable_irq(SPI_IRQ);
    ISR_ACTIVE.store(false, Ordering::SeqCst);

    // Disable SPI RX interrupt
    let spi_base = super::rp1_spi::spi_base();
    if spi_base != 0 {
        unsafe {
            core::ptr::write_volatile((spi_base + 0x2C) as *mut u32, 0); // Disable all SPI IRQs
        }
    }
}

/// The actual interrupt service routine — called by GIC dispatch
fn spi_rx_isr(_irq: u32) {
    let spi_base = super::rp1_spi::spi_base();
    if spi_base == 0 { return; }

    ISR_COUNT.fetch_add(1, Ordering::Relaxed);

    let channel = unsafe { TARGET_CHANNEL };
    let scale = unsafe { ADC_SCALE };

    // Read all available samples from SPI RX FIFO
    let sr_addr = spi_base + 0x28; // Status Register
    let dr_addr = spi_base + 0x60; // Data Register

    let mut samples_read = 0u32;

    unsafe {
        // While RX FIFO not empty (SR bit 3 = RFNE)
        while core::ptr::read_volatile(sr_addr as *const u32) & (1 << 3) != 0 {
            // Read raw sample from DR
            let raw = core::ptr::read_volatile(dr_addr as *const u32);

            // Convert to f32 (signed ADC value → normalized ±1.0)
            let sample = (raw as i16 as f32) * scale;

            // Push directly into the ring buffer (zero-copy, no lock)
            crate::daq::push_sample(channel, sample);

            samples_read += 1;

            // Safety limit: don't spin forever in ISR
            if samples_read >= 64 { break; }
        }

        // Clear interrupt by reading ISR register (auto-clear on DW SPI)
        let _ = core::ptr::read_volatile((spi_base + 0x30) as *const u32); // ICR
    }

    SAMPLE_COUNT.fetch_add(samples_read as u64, Ordering::Relaxed);
}

/// Get ISR telemetry
pub fn stats() -> (u64, u64) {
    (ISR_COUNT.load(Ordering::Relaxed), SAMPLE_COUNT.load(Ordering::Relaxed))
}

/// Check if ISR mode is active
pub fn is_active() -> bool {
    ISR_ACTIVE.load(Ordering::Relaxed)
}
