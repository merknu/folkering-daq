//! AArch64 architecture support for Raspberry Pi 5 (BCM2712, Cortex-A76)

pub mod uart;
pub mod exceptions;
pub mod gic;
pub mod timer;

use core::arch::asm;

/// Wait For Event — low-power idle until interrupt/event
#[inline(always)]
pub fn wfe() {
    unsafe { asm!("wfe", options(nomem, nostack)); }
}

/// Send Event — wake up cores in WFE
#[inline(always)]
pub fn sev() {
    unsafe { asm!("sev", options(nomem, nostack)); }
}

/// Disable IRQs (DAIF mask)
#[inline(always)]
pub fn disable_interrupts() {
    unsafe { asm!("msr daifset, #0xf", options(nomem, nostack)); }
}

/// Enable IRQs
#[inline(always)]
pub fn enable_interrupts() {
    unsafe { asm!("msr daifclr, #0xf", options(nomem, nostack)); }
}

/// Read current exception level
#[inline(always)]
pub fn current_el() -> u8 {
    let el: u64;
    unsafe { asm!("mrs {0}, CurrentEL", out(reg) el, options(nomem, nostack)); }
    ((el >> 2) & 0x3) as u8
}

/// Read the counter frequency (CNTFRQ_EL0)
#[inline(always)]
pub fn counter_freq() -> u64 {
    let freq: u64;
    unsafe { asm!("mrs {0}, CNTFRQ_EL0", out(reg) freq, options(nomem, nostack)); }
    freq
}

/// Read the physical counter (CNTPCT_EL0)
#[inline(always)]
pub fn counter_value() -> u64 {
    let val: u64;
    unsafe { asm!("mrs {0}, CNTPCT_EL0", out(reg) val, options(nomem, nostack)); }
    val
}

/// Data Synchronization Barrier
#[inline(always)]
pub fn dsb_sy() {
    unsafe { asm!("dsb sy", options(nomem, nostack)); }
}

/// Instruction Synchronization Barrier
#[inline(always)]
pub fn isb() {
    unsafe { asm!("isb", options(nomem, nostack)); }
}

/// Data Memory Barrier
#[inline(always)]
pub fn dmb_sy() {
    unsafe { asm!("dmb sy", options(nomem, nostack)); }
}

/// Memory-Mapped I/O read (volatile, 32-bit)
#[inline(always)]
pub unsafe fn mmio_read32(addr: u64) -> u32 {
    core::ptr::read_volatile(addr as *const u32)
}

/// Memory-Mapped I/O write (volatile, 32-bit)
#[inline(always)]
pub unsafe fn mmio_write32(addr: u64, val: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, val) }
}
