//! PL011 UART driver for BCM2712 (Raspberry Pi 5)
//!
//! UART0 (PL011) is on the dedicated 3-pin debug header between HDMI ports.
//! Physical MMIO base: 0x10_7D00_1000 (BCM2712 internal, NOT behind RP1).
//! This means serial output works immediately at boot — no PCIe enumeration needed.
//!
//! Reference clock: 48 MHz (BCM2712 default)
//! Default baud: 115200 (8N1)

use core::fmt;
use spin::Mutex;

/// UART0 physical base address on BCM2712
const UART0_PHYS: u64 = 0x10_7D00_1000;

/// QEMU virt machine UART (for development/testing)
const UART_QEMU_VIRT: u64 = 0x0900_0000;

/// PL011 Register offsets
const UART_DR: u64 = 0x000;      // Data Register
const UART_FR: u64 = 0x018;      // Flag Register
const UART_IBRD: u64 = 0x024;    // Integer Baud Rate Divisor
const UART_FBRD: u64 = 0x028;    // Fractional Baud Rate Divisor
const UART_LCRH: u64 = 0x02C;    // Line Control Register
const UART_CR: u64 = 0x030;      // Control Register
const UART_ICR: u64 = 0x044;     // Interrupt Clear Register

/// Flag register bits
const FR_TXFF: u32 = 1 << 5;     // Transmit FIFO Full
const FR_RXFE: u32 = 1 << 4;     // Receive FIFO Empty

/// UART state
static UART: Mutex<Uart> = Mutex::new(Uart { base: 0 });

struct Uart {
    base: u64,
}

impl Uart {
    fn init(&mut self, base: u64) {
        self.base = base;

        unsafe {
            // Step 1: Disable UART completely before changing configuration
            core::ptr::write_volatile((base + UART_CR) as *mut u32, 0);

            // Step 2: Clear all pending interrupts from previous boot stages (start.elf etc.)
            core::ptr::write_volatile((base + UART_ICR) as *mut u32, 0x7FF);

            // Step 3: Set baud rate for 115200 @ 48 MHz reference clock
            // Divisor = 48_000_000 / (16 * 115200) = 26.0416...
            // Integer part = 26
            // Fractional part = int(0.0416 * 64 + 0.5) = 3
            core::ptr::write_volatile((base + UART_IBRD) as *mut u32, 26);
            core::ptr::write_volatile((base + UART_FBRD) as *mut u32, 3);

            // Step 4: Line control — 8N1 + FIFO enable
            // LCRH write MUST come after IBRD/FBRD — it latches the baud rate config.
            // Value 0x70 = WLEN=8bit (bits 6:5 = 0b11) + FEN=FIFO enable (bit 4 = 1)
            core::ptr::write_volatile((base + UART_LCRH) as *mut u32, 0x70);

            // Step 5: Enable UART + TX + RX
            // CR value 0x301 = UARTEN (bit 0) + TXE (bit 8) + RXE (bit 9)
            core::ptr::write_volatile((base + UART_CR) as *mut u32, 0x301);
        }
    }

    fn putc(&self, c: u8) {
        if self.base == 0 { return; }
        unsafe {
            // Spin until TX FIFO has space (TXFF bit goes low)
            while core::ptr::read_volatile((self.base + UART_FR) as *const u32) & FR_TXFF != 0 {
                core::hint::spin_loop();
            }
            core::ptr::write_volatile((self.base + UART_DR) as *mut u32, c as u32);
        }
    }

    fn getc(&self) -> Option<u8> {
        if self.base == 0 { return None; }
        unsafe {
            if core::ptr::read_volatile((self.base + UART_FR) as *const u32) & FR_RXFE != 0 {
                return None; // RX FIFO empty
            }
            Some(core::ptr::read_volatile((self.base + UART_DR) as *const u32) as u8)
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

/// Initialize UART0
pub fn init() {
    let mut uart = UART.lock();
    let base = crate::phys_to_virt(crate::platform::UART_PHYS);
    uart.init(base);
}

/// Print formatted text to UART
pub fn _print(args: fmt::Arguments) {
    use fmt::Write;
    UART.lock().write_fmt(args).unwrap();
}

/// Read a byte (non-blocking)
pub fn read_byte() -> Option<u8> {
    UART.lock().getc()
}
