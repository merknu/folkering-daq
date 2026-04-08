//! ARM Generic Timer driver
//!
//! Uses CNTP (Non-secure Physical Timer) for kernel scheduling.
//! The timer fires a PPI (IRQ 30) at the configured interval.
//!
//! Pi 5 timer frequency: 54 MHz (BCM2712)
//! QEMU virt: 62.5 MHz (typically)

use core::sync::atomic::{AtomicU64, Ordering};

/// Timer frequency in Hz (read from CNTFRQ_EL0 at init)
static TIMER_FREQ: AtomicU64 = AtomicU64::new(0);

/// Tick counter (incremented on each timer IRQ)
static TICK_COUNT: AtomicU64 = AtomicU64::new(0);

/// Scheduling interval in microseconds
const TICK_INTERVAL_US: u64 = 1000; // 1ms tick = 1kHz scheduler

pub fn init() {
    let freq = super::counter_freq();
    TIMER_FREQ.store(freq, Ordering::SeqCst);
    crate::kprintln!("  Timer frequency: {} Hz", freq);

    // Calculate ticks per interval
    let ticks = freq * TICK_INTERVAL_US / 1_000_000;

    unsafe {
        core::arch::asm!("msr CNTP_TVAL_EL0, {0}", in(reg) ticks, options(nomem, nostack));
        core::arch::asm!("msr CNTP_CTL_EL0, {0}", in(reg) 1u64, options(nomem, nostack));
    }

    // Register timer IRQ handler (PPI 30)
    super::gic::register_handler(30, timer_irq_handler);
}

fn timer_irq_handler(_irq: u32) {
    TICK_COUNT.fetch_add(1, Ordering::Relaxed);

    // Re-arm timer
    let freq = TIMER_FREQ.load(Ordering::Relaxed);
    let ticks = freq * TICK_INTERVAL_US / 1_000_000;

    unsafe {
        core::arch::asm!("msr CNTP_TVAL_EL0, {0}", in(reg) ticks, options(nomem, nostack));
    }
}

/// Get current tick count (milliseconds since boot)
pub fn ticks() -> u64 {
    TICK_COUNT.load(Ordering::Relaxed)
}

/// Get microsecond timestamp (high resolution, from hardware counter)
pub fn micros() -> u64 {
    let freq = TIMER_FREQ.load(Ordering::Relaxed);
    if freq == 0 { return 0; }
    super::counter_value() * 1_000_000 / freq
}

/// Get millisecond timestamp
pub fn millis() -> u64 {
    micros() / 1000
}

/// Busy-wait for specified microseconds
pub fn delay_us(us: u64) {
    let start = super::counter_value();
    let freq = TIMER_FREQ.load(Ordering::Relaxed);
    let target = start + (freq * us / 1_000_000);
    while super::counter_value() < target {
        core::hint::spin_loop();
    }
}

/// Busy-wait for specified milliseconds
pub fn delay_ms(ms: u64) {
    delay_us(ms * 1000);
}
