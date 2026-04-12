//! JIT Memory Management for Silverfir-nano
//!
//! Handles the W^X (Write XOR Execute) lifecycle for JIT-compiled code.
//!
//! Multi-arch: uses #[cfg(target_arch)] to select the correct cache-flush
//! and memory barrier instructions for ARM64 vs x86_64.

use core::alloc::Layout;

/// Memory permission states for JIT pages
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MemoryPermission {
    /// Writable but not executable — used while Silverfir writes machine code
    ReadWrite,
    /// Executable but not writable — used when code is ready to run
    ReadExecute,
}

/// A page of memory managed for JIT compilation.
///
/// Lifecycle:
///   1. allocate() → page is RW
///   2. Silverfir writes machine code into the page
///   3. make_executable() → page flips to RX + instruction cache flushed
///   4. CPU jumps into the page and runs the compiled WASM
pub struct JitPage {
    pub ptr: *mut u8,
    pub size: usize,
}

impl JitPage {
    /// Allocate a 4KB-aligned page for JIT code.
    /// Returns None if allocation fails.
    pub fn allocate(size: usize) -> Option<Self> {
        let layout = Layout::from_size_align(size, 4096).ok()?;
        let ptr = unsafe { alloc::alloc::alloc_zeroed(layout) };
        if ptr.is_null() {
            return None;
        }
        Some(Self { ptr, size })
    }

    /// Transition this page from RW → RX.
    ///
    /// # Safety
    /// - All writes to the page must be complete before calling this
    /// - The page must contain valid machine code for the current architecture
    pub unsafe fn make_executable(&self) {
        // Step 1: Update page table entries (arch-specific PTE manipulation)
        Self::update_pte_permissions(self.ptr, self.size, MemoryPermission::ReadExecute);

        // Step 2: Architecture-specific cache flush + barrier
        Self::flush_icache(self.ptr, self.size);
    }

    /// Transition back from RX → RW (for hot-swap of DSP algorithms)
    ///
    /// # Safety
    /// - No thread may be executing code in this page during the transition
    pub unsafe fn make_writable(&self) {
        Self::update_pte_permissions(self.ptr, self.size, MemoryPermission::ReadWrite);
    }

    /// Write machine code into the page (must be in RW state)
    pub fn write_code(&mut self, offset: usize, code: &[u8]) -> bool {
        if offset + code.len() > self.size {
            return false;
        }
        unsafe {
            core::ptr::copy_nonoverlapping(code.as_ptr(), self.ptr.add(offset), code.len());
        }
        true
    }

    /// Get a function pointer to the start of the JIT code
    ///
    /// # Safety
    /// - Page must be in RX state (call make_executable first)
    /// - Code at offset must be a valid function for the target architecture
    pub unsafe fn as_fn_ptr<F>(&self, offset: usize) -> *const F {
        self.ptr.add(offset) as *const F
    }

    // === Architecture-specific implementations ===

    #[cfg(target_arch = "aarch64")]
    unsafe fn flush_icache(ptr: *mut u8, size: usize) {
        // ARM64: clean data cache + invalidate instruction cache for the range
        let mut addr = ptr as usize;
        let end = addr + size;
        while addr < end {
            // Clean to Point of Unification (data cache)
            core::arch::asm!("dc cvau, {0}", in(reg) addr, options(nomem, nostack));
            addr += 64; // cache line size
        }
        // Data Synchronization Barrier — ensure all cache ops complete
        core::arch::asm!("dsb ish", options(nomem, nostack));
        // Invalidate instruction cache for the range
        addr = ptr as usize;
        while addr < end {
            core::arch::asm!("ic ivau, {0}", in(reg) addr, options(nomem, nostack));
            addr += 64;
        }
        // Final barriers
        core::arch::asm!("dsb ish", options(nomem, nostack));
        core::arch::asm!("isb", options(nomem, nostack));
    }

    #[cfg(target_arch = "x86_64")]
    unsafe fn flush_icache(_ptr: *mut u8, _size: usize) {
        // x86_64: instruction cache is kept coherent with data cache by hardware.
        // We only need a serializing instruction to ensure the pipeline
        // sees the new code after the PTE change.
        core::arch::asm!("mfence", options(nomem, nostack));
        // CPUID is the canonical serializing instruction on x86
        core::arch::asm!(
            "xor eax, eax",
            "cpuid",
            out("eax") _, out("ebx") _, out("ecx") _, out("edx") _,
            options(nomem, nostack)
        );
    }

    #[cfg(not(any(target_arch = "aarch64", target_arch = "x86_64")))]
    unsafe fn flush_icache(_ptr: *mut u8, _size: usize) {
        // Fallback: compiler fence only — may not be sufficient on all architectures
        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
    }

    unsafe fn update_pte_permissions(ptr: *mut u8, size: usize, perm: MemoryPermission) {
        let modified = super::pagetable::set_range_permissions(ptr, size, perm);
        if modified == 0 {
            crate::kprintln!("  JIT: WARNING — PTE update failed for {:#x} (size={})", ptr as u64, size);
        }
    }
}

impl Drop for JitPage {
    fn drop(&mut self) {
        if !self.ptr.is_null() {
            let layout = Layout::from_size_align(self.size, 4096).unwrap();
            unsafe { alloc::alloc::dealloc(self.ptr, layout); }
        }
    }
}
