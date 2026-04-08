//! PL011 UART driver for Raspberry Pi 5
//!
//! BCM2712 has multiple UARTs. UART0 (PL011) is exposed on GPIO 14/15.
//! On Pi 5, the primary debug UART is accessed via the 3-pin debug header
//! or through the RP1 south bridge.
//!
//! For Limine boot: Limine configures UART before handing off to kernel,
//! so we can write immediately. We just need the base address.

use core::fmt;
use spin::Mutex;

/// PL011 UART base address on BCM2712 (Raspberry Pi 5)
/// The RP1 south bridge maps UART at this address in the ARM's physical space.
/// Limine sets up HHDM, so we access via: HHDM_OFFSET + UART_PHYS_BASE
///
/// Pi 5 debug UART: 0x1_0007_8000 (RP1 bar2 + 0x78000)
/// Fallback for QEMU virt: 0x0900_0000
const UART_PHYS_BASE_PI5: u64 = 0x1_0007_8000;
const UART_PHYS_BASE_QEMU_VIRT: u64 = 0x0900_0000;

/// PL011 Register offsets
const UARTDR: u64 = 0x000;     // Data Register
const UARTFR: u64 = 0x018;     // Flag Register
const UARTIBRD: u64 = 0x024;   // Integer Baud Rate
const UARTFBRD: u64 = 0x028;   // Fractional Baud Rate
const UARTLCR_H: u64 = 0x02C;  // Line Control
const UARTCR: u64 = 0x030;     // Control Register
const UARTIMSC: u64 = 0x038;   // Interrupt Mask

/// Flag register bits
const FR_TXFF: u32 = 1 << 5;   // Transmit FIFO full
const FR_RXFE: u32 = 1 << 4;   // Receive FIFO empty
const FR_BUSY: u32 = 1 << 3;   // UART busy

/// UART state
static UART: Mutex<Uart> = Mutex::new(Uart { base: 0 });

struct Uart {
    base: u64,
}

impl Uart {
    fn init(&mut self, base: u64) {
        self.base = base;

        unsafe {
            // Disable UART
            super::mmio_write32(base + UARTCR, 0);

            // Wait for any ongoing transmission
            while super::mmio_read32(base + UARTFR) & FR_BUSY != 0 {}

            // Set baud rate 115200 @ 48MHz clock (Pi 5)
            // Divider = 48_000_000 / (16 * 115200) = 26.041...
            // Integer = 26, Fractional = (0.041 * 64 + 0.5) = 3
            super::mmio_write32(base + UARTIBRD, 26);
            super::mmio_write32(base + UARTFBRD, 3);

            // 8N1, FIFO enable
            super::mmio_write32(base + UARTLCR_H, (3 << 5) | (1 << 4)); // WLEN=8, FEN=1

            // Disable all interrupts (we poll)
            super::mmio_write32(base + UARTIMSC, 0);

            // Enable UART, TX, RX
            super::mmio_write32(base + UARTCR, (1 << 0) | (1 << 8) | (1 << 9));
        }
    }

    fn putc(&self, c: u8) {
        if self.base == 0 { return; }
        unsafe {
            // Wait until TX FIFO has space
            while super::mmio_read32(self.base + UARTFR) & FR_TXFF != 0 {}
            super::mmio_write32(self.base + UARTDR, c as u32);
        }
    }

    fn getc(&self) -> Option<u8> {
        if self.base == 0 { return None; }
        unsafe {
            if super::mmio_read32(self.base + UARTFR) & FR_RXFE != 0 {
                return None; // FIFO empty
            }
            Some(super::mmio_read32(self.base + UARTDR) as u8)
        }
    }
}

impl fmt::Write for Uart {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for byte in s.bytes() {
            if byte == b'\n' {
                self.putc(b'\r');
            }
            self.putc(byte);
        }
        Ok(())
    }
}

/// Initialize UART with auto-detect base address
pub fn init() {
    let mut uart = UART.lock();

    // Try Pi 5 address first. If running in QEMU virt, use QEMU address.
    // Detection: read UARTFR — if we get a sensible value, UART exists.
    // For now, default to QEMU virt for development, Pi 5 for production.
    #[cfg(feature = "pi5")]
    let base = crate::phys_to_virt(UART_PHYS_BASE_PI5);

    #[cfg(not(feature = "pi5"))]
    let base = crate::phys_to_virt(UART_PHYS_BASE_QEMU_VIRT);

    uart.init(base);
}

/// Print formatted string to UART
pub fn _print(args: fmt::Arguments) {
    use fmt::Write;
    UART.lock().write_fmt(args).unwrap();
}

/// Read a byte from UART (non-blocking)
pub fn read_byte() -> Option<u8> {
    UART.lock().getc()
}
