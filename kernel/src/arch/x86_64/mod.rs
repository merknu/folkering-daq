//! X86_64 architecture support for Folkering DAQ
//!
//! This module provides the low-level primitives needed for JIT execution
//! and memory protection on x86_64.

use core::arch::asm;

/// Data Synchronization Barrier (Implicit on x86 for most stores, but good for clarity)
#[inline(always)]
pub fn dsb_sy() {
    // x86 is strongly ordered, but mfence ensures store visibility
    unsafe { asm!("mfence", options(nomem, nostack)); }
}

/// Instruction Synchronization Barrier
/// On x86, we use cpuid or an iret/jump to serialize the pipeline
#[inline(always)]
pub fn isb() {
    unsafe {
        asm!("cpuid", out("eax") _, out("ebx") _, out("ecx") _, out("edx") _, options(nomem, nostack));
    }
}

/// Invalidate a single page from the TLB
#[inline(always)]
pub fn invlpg(addr: u64) {
    unsafe {
        asm!("invlpg [{}]", in(reg) addr, options(nomem, nostack));
    }
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
