//! RP1 GPIO Driver for Raspberry Pi 5
//!
//! GPIO controller inside the RP1 south bridge, accessed via PCIe BAR1.
//! Register layout: each GPIO pin has 8 bytes (STATUS + CTRL).
//!
//! RP1 GPIO base: BAR1 + 0xD0000 (28 GPIO pins, directly connected to 40-pin header)
//! RP1 RIO base:  BAR1 + 0xE0000 (Register I/O — fast read/write/toggle for GPIO)
//! RP1 PADS base: BAR1 + 0xF0000 (Pad control — drive strength, pull-up/down, slew)
//!
//! Physical GPIO header mapping (Pi 5, 40-pin):
//!   GPIO0-27 → RP1 pins 0-27
//!   GPIO pins are directly accessible after configuring FSEL (function select)

use core::sync::atomic::{AtomicU64, Ordering};

// RP1 AXI offsets from BAR1
const GPIO_OFFSET: u64 = 0xD_0000;
const RIO_OFFSET: u64  = 0xE_0000;
const PADS_OFFSET: u64 = 0xF_0000;

// GPIO register offsets per pin (8 bytes each)
const GPIO_STATUS: u64 = 0x00; // Read-only: pin level, interrupt status
const GPIO_CTRL: u64   = 0x04; // R/W: function select, output override, interrupt config

// RIO register offsets (atomic set/clear/toggle)
const RIO_OUT: u64       = 0x00; // Output value (read/write)
const RIO_OE: u64        = 0x04; // Output enable (1=output, 0=input)
const RIO_IN: u64        = 0x08; // Input value (read-only)
// Atomic set/clear/toggle (write-only, offset from base)
const RIO_OUT_SET: u64   = 0x2000;
const RIO_OUT_CLR: u64   = 0x3000;
const RIO_OE_SET: u64    = 0x2004;
const RIO_OE_CLR: u64    = 0x3004;

// PADS register offsets (4 bytes per pin, starting at offset 0x04)
const PADS_GPIO_BASE: u64 = 0x04; // Pad 0 starts at offset 4

// Pad control bits
const PAD_SLEWFAST: u32   = 1 << 0;
const PAD_SCHMITT: u32    = 1 << 1;
const PAD_PDE: u32        = 1 << 2; // Pull-down enable
const PAD_PUE: u32        = 1 << 3; // Pull-up enable
const PAD_DRIVE_2MA: u32  = 0 << 4;
const PAD_DRIVE_4MA: u32  = 1 << 4;
const PAD_DRIVE_8MA: u32  = 2 << 4;
const PAD_DRIVE_12MA: u32 = 3 << 4;
const PAD_IE: u32         = 1 << 6; // Input enable
const PAD_OD: u32         = 1 << 7; // Output disable

// Function select values (CTRL register bits [4:0])
const FSEL_ALT0: u32 = 0;
const FSEL_ALT1: u32 = 1;
const FSEL_ALT2: u32 = 2;
const FSEL_ALT3: u32 = 3;
const FSEL_ALT4: u32 = 4;
const FSEL_GPIO: u32 = 5;  // Software-controlled GPIO
const FSEL_ALT6: u32 = 6;
const FSEL_ALT7: u32 = 7;
const FSEL_ALT8: u32 = 8;
const FSEL_NULL: u32 = 0x1F; // No function (reset default)

/// Number of GPIO pins on RP1 header
pub const NUM_PINS: usize = 28;

static GPIO_BASE: AtomicU64 = AtomicU64::new(0);
static RIO_BASE: AtomicU64 = AtomicU64::new(0);
static PADS_BASE: AtomicU64 = AtomicU64::new(0);

/// Pin mode
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PinMode {
    Input,
    Output,
    Alt0, Alt1, Alt2, Alt3, Alt4, Alt5,
}

/// Pull resistor configuration
#[derive(Clone, Copy, Debug)]
pub enum Pull {
    None,
    Up,
    Down,
}

// MMIO helpers
unsafe fn mmio_read(addr: u64) -> u32 {
    core::ptr::read_volatile(addr as *const u32)
}

unsafe fn mmio_write(addr: u64, val: u32) {
    core::ptr::write_volatile(addr as *mut u32, val);
}

/// Initialize GPIO driver with RP1 BAR1 virtual base address
pub fn init(bar1_virt: u64) {
    let gpio = bar1_virt + GPIO_OFFSET;
    let rio = bar1_virt + RIO_OFFSET;
    let pads = bar1_virt + PADS_OFFSET;

    GPIO_BASE.store(gpio, Ordering::SeqCst);
    RIO_BASE.store(rio, Ordering::SeqCst);
    PADS_BASE.store(pads, Ordering::SeqCst);

    crate::kprintln!("  GPIO: RP1 GPIO at {:#x}, RIO at {:#x}, PADS at {:#x}",
        gpio, rio, pads);
    crate::kprintln!("  GPIO: {} pins available (header GPIO0-27)", NUM_PINS);
}

/// Set pin function (mode)
pub fn set_mode(pin: u8, mode: PinMode) {
    if pin as usize >= NUM_PINS { return; }
    let gpio = GPIO_BASE.load(Ordering::Relaxed);
    let rio = RIO_BASE.load(Ordering::Relaxed);
    if gpio == 0 { return; }

    let fsel = match mode {
        PinMode::Input | PinMode::Output => FSEL_GPIO,
        PinMode::Alt0 => FSEL_ALT0,
        PinMode::Alt1 => FSEL_ALT1,
        PinMode::Alt2 => FSEL_ALT2,
        PinMode::Alt3 => FSEL_ALT3,
        PinMode::Alt4 => FSEL_ALT4,
        PinMode::Alt5 => FSEL_GPIO, // No ALT5 on RP1
    };

    unsafe {
        // Set function select in GPIO CTRL register
        let ctrl_addr = gpio + (pin as u64 * 8) + GPIO_CTRL;
        let mut ctrl = mmio_read(ctrl_addr);
        ctrl = (ctrl & !0x1F) | fsel; // Clear bits [4:0], set FSEL
        mmio_write(ctrl_addr, ctrl);

        // Set output enable via RIO
        let bit = 1u32 << pin;
        match mode {
            PinMode::Output => mmio_write(rio + RIO_OE_SET, bit),
            PinMode::Input => mmio_write(rio + RIO_OE_CLR, bit),
            _ => {} // Alt modes handle OE automatically
        }

        // Configure pad for the mode
        let pad_addr = PADS_BASE.load(Ordering::Relaxed) + PADS_GPIO_BASE + (pin as u64 * 4);
        match mode {
            PinMode::Input => {
                mmio_write(pad_addr, PAD_IE | PAD_SCHMITT);
            }
            PinMode::Output => {
                mmio_write(pad_addr, PAD_DRIVE_4MA | PAD_SLEWFAST);
            }
            _ => {
                // Alt function: enable both input and output paths
                mmio_write(pad_addr, PAD_IE | PAD_DRIVE_4MA);
            }
        }
    }
}

/// Set pull-up/pull-down resistor
pub fn set_pull(pin: u8, pull: Pull) {
    if pin as usize >= NUM_PINS { return; }
    let pads = PADS_BASE.load(Ordering::Relaxed);
    if pads == 0 { return; }

    let pad_addr = pads + PADS_GPIO_BASE + (pin as u64 * 4);
    unsafe {
        let mut val = mmio_read(pad_addr);
        val &= !(PAD_PUE | PAD_PDE); // Clear pull bits
        match pull {
            Pull::Up => val |= PAD_PUE,
            Pull::Down => val |= PAD_PDE,
            Pull::None => {}
        }
        mmio_write(pad_addr, val);
    }
}

/// Write a digital value to an output pin
pub fn write(pin: u8, high: bool) {
    if pin as usize >= NUM_PINS { return; }
    let rio = RIO_BASE.load(Ordering::Relaxed);
    if rio == 0 { return; }

    let bit = 1u32 << pin;
    unsafe {
        if high {
            mmio_write(rio + RIO_OUT_SET, bit);
        } else {
            mmio_write(rio + RIO_OUT_CLR, bit);
        }
    }
}

/// Read the current level of a pin (works for both input and output pins)
pub fn read(pin: u8) -> bool {
    if pin as usize >= NUM_PINS { return false; }
    let rio = RIO_BASE.load(Ordering::Relaxed);
    if rio == 0 { return false; }

    unsafe {
        let val = mmio_read(rio + RIO_IN);
        (val >> pin) & 1 != 0
    }
}

/// Read all 28 pins at once (bit per pin)
pub fn read_all() -> u32 {
    let rio = RIO_BASE.load(Ordering::Relaxed);
    if rio == 0 { return 0; }
    unsafe { mmio_read(rio + RIO_IN) & 0x0FFF_FFFF }
}

/// Toggle an output pin
pub fn toggle(pin: u8) {
    if pin as usize >= NUM_PINS { return; }
    let rio = RIO_BASE.load(Ordering::Relaxed);
    if rio == 0 { return; }

    // Read current output, XOR the bit, write back
    unsafe {
        let current = mmio_read(rio + RIO_OUT);
        let toggled = current ^ (1u32 << pin);
        mmio_write(rio + RIO_OUT, toggled);
    }
}
