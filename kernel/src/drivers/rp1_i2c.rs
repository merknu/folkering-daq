//! RP1 I2C Driver (DesignWare I2C) for Raspberry Pi 5
//!
//! I2C master controllers inside RP1 south bridge.
//! Compatible: Synopsys DesignWare I2C (snps,designware-i2c)
//!
//! RP1 I2C instances (AXI offsets from BAR1):
//!   I2C0: 0x70000 (header pins: SDA=GPIO0, SCL=GPIO1)  ← PRIMARY
//!   I2C1: 0x74000 (header pins: SDA=GPIO2, SCL=GPIO3)
//!   I2C2: 0x78000
//!   I2C3: 0x7C000
//!   I2C4: 0x80000
//!   I2C5: 0x84000
//!   I2C6: 0x88000
//!
//! For DAQ: I2C0 and I2C1 are on the 40-pin header.
//! Typical sensors: BME280 (temp/humidity/pressure), MPU6050 (IMU), ADS1115 (ADC).

use core::sync::atomic::{AtomicU64, Ordering};

// RP1 I2C instance offsets from BAR1
const I2C0_OFFSET: u64 = 0x7_0000;
const I2C1_OFFSET: u64 = 0x7_4000;

// DesignWare I2C Register Map
const IC_CON: u64          = 0x00; // Control
const IC_TAR: u64          = 0x04; // Target address
const IC_DATA_CMD: u64     = 0x10; // Data/Command
const IC_SS_SCL_HCNT: u64  = 0x14; // Standard Speed SCL High Count
const IC_SS_SCL_LCNT: u64  = 0x18; // Standard Speed SCL Low Count
const IC_FS_SCL_HCNT: u64  = 0x1C; // Fast Speed SCL High Count
const IC_FS_SCL_LCNT: u64  = 0x20; // Fast Speed SCL Low Count
const IC_INTR_STAT: u64    = 0x2C; // Interrupt Status
const IC_INTR_MASK: u64    = 0x30; // Interrupt Mask
const IC_CLR_INTR: u64     = 0x40; // Clear Combined Interrupt
const IC_CLR_TX_ABRT: u64  = 0x54; // Clear TX Abort
const IC_ENABLE: u64       = 0x6C; // Enable (0=disable, 1=enable)
const IC_STATUS: u64       = 0x70; // Status
const IC_TXFLR: u64        = 0x74; // TX FIFO Level
const IC_RXFLR: u64        = 0x78; // RX FIFO Level
const IC_TX_ABRT_SOURCE: u64 = 0x80; // TX Abort Source
const IC_ENABLE_STATUS: u64  = 0x9C; // Enable Status

// IC_CON bits
const IC_CON_MASTER: u32       = 1 << 0; // Master mode
const IC_CON_SPEED_STD: u32    = 1 << 1; // Standard mode (100 kHz)
const IC_CON_SPEED_FAST: u32   = 2 << 1; // Fast mode (400 kHz)
const IC_CON_10BIT_SLAVE: u32  = 1 << 3; // 10-bit addressing (slave)
const IC_CON_10BIT_MASTER: u32 = 1 << 4; // 10-bit addressing (master)
const IC_CON_RESTART_EN: u32   = 1 << 5; // Enable restart conditions
const IC_CON_SLAVE_DISABLE: u32 = 1 << 6; // Disable slave mode

// IC_DATA_CMD bits
const IC_DATA_CMD_READ: u32  = 1 << 8;  // Read command
const IC_DATA_CMD_STOP: u32  = 1 << 9;  // Generate STOP after this byte
const IC_DATA_CMD_RESTART: u32 = 1 << 10; // Generate RESTART before this byte

// IC_STATUS bits
const IC_STATUS_ACTIVITY: u32 = 1 << 0; // I2C activity
const IC_STATUS_TFNF: u32    = 1 << 1;  // TX FIFO not full
const IC_STATUS_TFE: u32     = 1 << 2;  // TX FIFO empty
const IC_STATUS_RFNE: u32    = 1 << 3;  // RX FIFO not empty

// Clock values for 200 MHz IC_CLK (typical RP1 clock)
// Standard mode (100 kHz): period = 10μs → HCNT=900, LCNT=1000
// Fast mode (400 kHz): period = 2.5μs → HCNT=200, LCNT=300
const SS_HCNT: u32 = 900;
const SS_LCNT: u32 = 1000;
const FS_HCNT: u32 = 200;
const FS_LCNT: u32 = 300;

/// Timeout for I2C operations (in spin iterations)
const TIMEOUT: u32 = 100_000;

static I2C_BASES: [AtomicU64; 2] = [AtomicU64::new(0), AtomicU64::new(0)];

/// I2C bus index
#[derive(Clone, Copy, Debug)]
pub enum I2cBus {
    I2C0 = 0,
    I2C1 = 1,
}

/// I2C speed
#[derive(Clone, Copy, Debug)]
pub enum I2cSpeed {
    Standard, // 100 kHz
    Fast,     // 400 kHz
}

// MMIO helpers
unsafe fn mmio_read(addr: u64) -> u32 {
    core::ptr::read_volatile(addr as *const u32)
}

unsafe fn mmio_write(addr: u64, val: u32) {
    core::ptr::write_volatile(addr as *mut u32, val);
}

fn base_for(bus: I2cBus) -> u64 {
    I2C_BASES[bus as usize].load(Ordering::Relaxed)
}

/// Initialize an I2C bus
pub fn init(bar1_virt: u64, bus: I2cBus, speed: I2cSpeed) {
    let offset = match bus {
        I2cBus::I2C0 => I2C0_OFFSET,
        I2cBus::I2C1 => I2C1_OFFSET,
    };
    let base = bar1_virt + offset;
    I2C_BASES[bus as usize].store(base, Ordering::SeqCst);

    unsafe {
        // Disable before configuration
        mmio_write(base + IC_ENABLE, 0);
        while mmio_read(base + IC_ENABLE_STATUS) & 1 != 0 {
            core::hint::spin_loop();
        }

        // Master mode, restart enabled, slave disabled
        let speed_bits = match speed {
            I2cSpeed::Standard => IC_CON_SPEED_STD,
            I2cSpeed::Fast => IC_CON_SPEED_FAST,
        };
        mmio_write(base + IC_CON,
            IC_CON_MASTER | speed_bits | IC_CON_RESTART_EN | IC_CON_SLAVE_DISABLE);

        // Set clock counts
        mmio_write(base + IC_SS_SCL_HCNT, SS_HCNT);
        mmio_write(base + IC_SS_SCL_LCNT, SS_LCNT);
        mmio_write(base + IC_FS_SCL_HCNT, FS_HCNT);
        mmio_write(base + IC_FS_SCL_LCNT, FS_LCNT);

        // Disable all interrupts (polling mode)
        mmio_write(base + IC_INTR_MASK, 0);

        // Enable
        mmio_write(base + IC_ENABLE, 1);
    }

    // Configure GPIO pins
    match bus {
        I2cBus::I2C0 => {
            super::rp1_gpio::set_mode(0, super::rp1_gpio::PinMode::Alt0); // SDA0
            super::rp1_gpio::set_mode(1, super::rp1_gpio::PinMode::Alt0); // SCL0
            super::rp1_gpio::set_pull(0, super::rp1_gpio::Pull::Up);
            super::rp1_gpio::set_pull(1, super::rp1_gpio::Pull::Up);
        }
        I2cBus::I2C1 => {
            super::rp1_gpio::set_mode(2, super::rp1_gpio::PinMode::Alt0); // SDA1
            super::rp1_gpio::set_mode(3, super::rp1_gpio::PinMode::Alt0); // SCL1
            super::rp1_gpio::set_pull(2, super::rp1_gpio::Pull::Up);
            super::rp1_gpio::set_pull(3, super::rp1_gpio::Pull::Up);
        }
    }

    crate::kprintln!("  I2C{}: RP1 DW-I2C at {:#x}, {:?} mode",
        bus as u8, base, speed);
}

/// Set target slave address (7-bit)
fn set_target(bus: I2cBus, addr: u8) {
    let base = base_for(bus);
    if base == 0 { return; }

    unsafe {
        // Must disable before changing target address
        mmio_write(base + IC_ENABLE, 0);
        while mmio_read(base + IC_ENABLE_STATUS) & 1 != 0 {
            core::hint::spin_loop();
        }
        mmio_write(base + IC_TAR, (addr & 0x7F) as u32);
        mmio_write(base + IC_ENABLE, 1);
    }
}

/// Wait for TX FIFO to have space, with timeout
unsafe fn wait_tx_ready(base: u64) -> bool {
    for _ in 0..TIMEOUT {
        if mmio_read(base + IC_STATUS) & IC_STATUS_TFNF != 0 {
            return true;
        }
        core::hint::spin_loop();
    }
    false
}

/// Wait for RX FIFO to have data, with timeout
unsafe fn wait_rx_ready(base: u64) -> bool {
    for _ in 0..TIMEOUT {
        if mmio_read(base + IC_STATUS) & IC_STATUS_RFNE != 0 {
            return true;
        }
        core::hint::spin_loop();
    }
    false
}

/// Write bytes to an I2C device.
/// Returns number of bytes written, or 0 on error.
pub fn write_bytes(bus: I2cBus, addr: u8, data: &[u8]) -> usize {
    let base = base_for(bus);
    if base == 0 || data.is_empty() { return 0; }

    set_target(bus, addr);

    unsafe {
        // Clear any pending abort
        let _ = mmio_read(base + IC_CLR_TX_ABRT);

        for (i, &byte) in data.iter().enumerate() {
            if !wait_tx_ready(base) { return i; }

            let mut cmd = byte as u32;
            if i == data.len() - 1 {
                cmd |= IC_DATA_CMD_STOP; // STOP after last byte
            }
            mmio_write(base + IC_DATA_CMD, cmd);
        }

        // Wait for transfer to complete
        for _ in 0..TIMEOUT {
            let status = mmio_read(base + IC_STATUS);
            if status & IC_STATUS_TFE != 0 && status & IC_STATUS_ACTIVITY == 0 {
                break;
            }
            core::hint::spin_loop();
        }

        // Check for abort
        let abrt = mmio_read(base + IC_TX_ABRT_SOURCE);
        if abrt != 0 {
            let _ = mmio_read(base + IC_CLR_TX_ABRT);
            return 0;
        }
    }

    data.len()
}

/// Read bytes from an I2C device.
/// Returns number of bytes read, or 0 on error.
pub fn read_bytes(bus: I2cBus, addr: u8, buf: &mut [u8]) -> usize {
    let base = base_for(bus);
    if base == 0 || buf.is_empty() { return 0; }

    set_target(bus, addr);

    unsafe {
        let _ = mmio_read(base + IC_CLR_TX_ABRT);

        for i in 0..buf.len() {
            if !wait_tx_ready(base) { return i; }

            let mut cmd = IC_DATA_CMD_READ;
            if i == buf.len() - 1 {
                cmd |= IC_DATA_CMD_STOP;
            }
            mmio_write(base + IC_DATA_CMD, cmd);
        }

        // Read data from RX FIFO
        for i in 0..buf.len() {
            if !wait_rx_ready(base) { return i; }
            buf[i] = mmio_read(base + IC_DATA_CMD) as u8;
        }

        // Check for abort
        let abrt = mmio_read(base + IC_TX_ABRT_SOURCE);
        if abrt != 0 {
            let _ = mmio_read(base + IC_CLR_TX_ABRT);
            return 0;
        }
    }

    buf.len()
}

/// Write then read (common I2C pattern: write register address, then read data).
/// Generates a RESTART between write and read phases.
pub fn write_read(bus: I2cBus, addr: u8, write_data: &[u8], read_buf: &mut [u8]) -> usize {
    let base = base_for(bus);
    if base == 0 { return 0; }

    set_target(bus, addr);

    unsafe {
        let _ = mmio_read(base + IC_CLR_TX_ABRT);

        // Write phase (no STOP — RESTART will follow)
        for &byte in write_data {
            if !wait_tx_ready(base) { return 0; }
            mmio_write(base + IC_DATA_CMD, byte as u32);
        }

        // Read phase (RESTART before first byte, STOP after last)
        for i in 0..read_buf.len() {
            if !wait_tx_ready(base) { return 0; }

            let mut cmd = IC_DATA_CMD_READ;
            if i == 0 {
                cmd |= IC_DATA_CMD_RESTART;
            }
            if i == read_buf.len() - 1 {
                cmd |= IC_DATA_CMD_STOP;
            }
            mmio_write(base + IC_DATA_CMD, cmd);
        }

        // Drain RX FIFO
        for i in 0..read_buf.len() {
            if !wait_rx_ready(base) { return i; }
            read_buf[i] = mmio_read(base + IC_DATA_CMD) as u8;
        }

        let abrt = mmio_read(base + IC_TX_ABRT_SOURCE);
        if abrt != 0 {
            let _ = mmio_read(base + IC_CLR_TX_ABRT);
            return 0;
        }
    }

    read_buf.len()
}

/// Scan the I2C bus for devices (0x03-0x77).
/// Returns a bitmask of responding addresses.
pub fn scan(bus: I2cBus) -> u128 {
    let mut found: u128 = 0;

    for addr in 0x03..=0x77u8 {
        let mut buf = [0u8; 1];
        if read_bytes(bus, addr, &mut buf) > 0 {
            found |= 1u128 << addr;
            crate::kprintln!("  I2C{}: device found at {:#04x}", bus as u8, addr);
        }
    }

    found
}
