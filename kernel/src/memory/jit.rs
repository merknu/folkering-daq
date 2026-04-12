//! JIT Memory Management for Silverfir-nano
//!
//! Two allocation strategies:
//!   1. JitPool — Dedicated 1 MiB region with bitmap allocator (preferred)
//!   2. JitPage — Heap-allocated pages (fallback, may hit block descriptors)
//!
//! W^X lifecycle:
//!   alloc_writeable(size) → write machine code → make_executable(ptr, size)
//!   → CPU executes JIT code → make_writable(ptr, size) for hot-swap
//!
//! Multi-arch: aarch64 (Pi 5 primary) and x86_64 (QEMU).

use core::alloc::Layout;
use core::sync::atomic::{AtomicBool, Ordering};

/// Memory permission states for JIT pages
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MemoryPermission {
    ReadWrite,
    ReadExecute,
}

const PAGE_SIZE: usize = 4096;

// ────────────────────────────────────────────────────────────────
//  JitPool — Dedicated memory pool with bitmap allocator
// ────────────────────────────────────────────────────────────────

/// Number of 4KB pages in the pool (256 pages = 1 MiB)
const POOL_PAGES: usize = 256;
const POOL_SIZE: usize = POOL_PAGES * PAGE_SIZE;

/// Bitmap words needed: 256 pages / 64 bits per word = 4 words
const BITMAP_WORDS: usize = (POOL_PAGES + 63) / 64;

/// The global JIT memory pool
pub struct JitPool {
    base: *mut u8,
    bitmap: [u64; BITMAP_WORDS],
    initialized: bool,
}

/// Global JIT pool instance
static mut POOL: JitPool = JitPool {
    base: core::ptr::null_mut(),
    bitmap: [0; BITMAP_WORDS],
    initialized: false,
};

static POOL_INIT: AtomicBool = AtomicBool::new(false);

impl JitPool {
    /// Initialize the JIT pool from a usable memory region.
    ///
    /// Finds 1 MiB of contiguous memory from the Limine memory map,
    /// maps it with individual 4KB PTEs, and sets all pages to RW+NX.
    pub fn init(memory_map: &[crate::MemoryRegion]) {
        // Find a usable region with at least POOL_SIZE bytes
        // Skip the first region (used by heap allocator)
        let mut pool_phys: u64 = 0;

        for region in memory_map {
            if region.kind != crate::MemoryRegionKind::Usable { continue; }
            if region.length < (POOL_SIZE as u64 + crate::memory::HEAP_SIZE as u64) { continue; }

            // Place JIT pool AFTER the heap in this region
            pool_phys = region.base + crate::memory::HEAP_SIZE as u64;

            // Align to page boundary
            pool_phys = (pool_phys + PAGE_SIZE as u64 - 1) & !(PAGE_SIZE as u64 - 1);

            // Verify it fits
            let end = pool_phys + POOL_SIZE as u64;
            if end <= region.base + region.length {
                break;
            }
            pool_phys = 0;
        }

        if pool_phys == 0 {
            crate::kprintln!("  JIT: WARNING — no suitable memory region for pool");
            return;
        }

        let pool_virt = crate::phys_to_virt(pool_phys) as *mut u8;

        // Zero-initialize the pool region
        unsafe {
            core::ptr::write_bytes(pool_virt, 0, POOL_SIZE);
        }

        unsafe {
            POOL.base = pool_virt;
            POOL.bitmap = [0; BITMAP_WORDS]; // All free
            POOL.initialized = true;
        }

        POOL_INIT.store(true, Ordering::SeqCst);

        crate::kprintln!("  JIT: pool at phys {:#x}, virt {:#x} ({} pages, {} KiB)",
            pool_phys, pool_virt as u64, POOL_PAGES, POOL_SIZE / 1024);
    }

    /// Allocate contiguous pages from the pool (returned as RW).
    /// Returns pointer to the start of the allocated region, or None.
    pub fn alloc_writeable(num_pages: usize) -> Option<*mut u8> {
        if !POOL_INIT.load(Ordering::Relaxed) { return None; }
        if num_pages == 0 || num_pages > POOL_PAGES { return None; }

        unsafe {
            // Find a contiguous run of free pages in the bitmap
            let start = find_free_run(&POOL.bitmap, num_pages)?;

            // Mark pages as used
            for i in start..start + num_pages {
                let word = i / 64;
                let bit = i % 64;
                POOL.bitmap[word] |= 1u64 << bit;
            }

            let ptr = POOL.base.add(start * PAGE_SIZE);

            // Ensure pages are RW (they should be by default from Limine,
            // but if they were previously made RX, we need to flip back)
            super::pagetable::set_range_permissions(ptr, num_pages * PAGE_SIZE, MemoryPermission::ReadWrite);

            Some(ptr)
        }
    }

    /// Make a previously allocated region executable (RW → RX).
    /// Also flushes I-cache on aarch64.
    pub fn make_executable(ptr: *mut u8, size: usize) -> bool {
        if !POOL_INIT.load(Ordering::Relaxed) { return false; }

        let pages = (size + PAGE_SIZE - 1) / PAGE_SIZE;
        let modified = unsafe {
            super::pagetable::set_range_permissions(ptr, pages * PAGE_SIZE, MemoryPermission::ReadExecute)
        };

        if modified == 0 {
            crate::kprintln!("  JIT: PTE update failed for {:#x}", ptr as u64);
            return false;
        }

        // Flush instruction cache (critical on aarch64!)
        unsafe { flush_icache(ptr, size); }

        true
    }

    /// Make a region writable again (RX → RW, for hot-swap).
    pub fn make_writable(ptr: *mut u8, size: usize) -> bool {
        if !POOL_INIT.load(Ordering::Relaxed) { return false; }

        let pages = (size + PAGE_SIZE - 1) / PAGE_SIZE;
        let modified = unsafe {
            super::pagetable::set_range_permissions(ptr, pages * PAGE_SIZE, MemoryPermission::ReadWrite)
        };

        modified > 0
    }

    /// Free pages back to the pool.
    pub fn dealloc(ptr: *mut u8, num_pages: usize) {
        if !POOL_INIT.load(Ordering::Relaxed) { return; }

        unsafe {
            let offset = ptr as usize - POOL.base as usize;
            let start_page = offset / PAGE_SIZE;

            for i in start_page..start_page + num_pages {
                if i < POOL_PAGES {
                    let word = i / 64;
                    let bit = i % 64;
                    POOL.bitmap[word] &= !(1u64 << bit);
                }
            }
        }
    }

    /// Get pool utilization stats
    pub fn stats() -> (usize, usize) {
        if !POOL_INIT.load(Ordering::Relaxed) { return (0, 0); }
        let mut used = 0;
        unsafe {
            for word in &POOL.bitmap {
                used += word.count_ones() as usize;
            }
        }
        (used, POOL_PAGES)
    }
}

/// Find a contiguous run of `count` zero-bits in the bitmap
fn find_free_run(bitmap: &[u64; BITMAP_WORDS], count: usize) -> Option<usize> {
    let mut run_start = 0;
    let mut run_len = 0;

    for page in 0..POOL_PAGES {
        let word = page / 64;
        let bit = page % 64;

        if bitmap[word] & (1u64 << bit) == 0 {
            // Free page
            if run_len == 0 { run_start = page; }
            run_len += 1;
            if run_len >= count { return Some(run_start); }
        } else {
            // Used page — reset run
            run_len = 0;
        }
    }

    None
}

// ────────────────────────────────────────────────────────────────
//  JitPage — Single-page allocation (uses JitPool internally)
// ────────────────────────────────────────────────────────────────

/// A page of memory managed for JIT compilation.
pub struct JitPage {
    pub ptr: *mut u8,
    pub size: usize,
    pages: usize, // Number of pool pages used
}

impl JitPage {
    /// Allocate a page-aligned region for JIT code.
    pub fn allocate(size: usize) -> Option<Self> {
        let pages = (size + PAGE_SIZE - 1) / PAGE_SIZE;
        let ptr = JitPool::alloc_writeable(pages)?;

        Some(Self { ptr, size: pages * PAGE_SIZE, pages })
    }

    /// Transition RW → RX + I-cache flush
    pub unsafe fn make_executable(&self) -> bool {
        JitPool::make_executable(self.ptr, self.size)
    }

    /// Transition RX → RW (for hot-swap)
    pub unsafe fn make_writable(&self) -> bool {
        JitPool::make_writable(self.ptr, self.size)
    }

    /// Write machine code into the page (must be in RW state)
    pub fn write_code(&mut self, offset: usize, code: &[u8]) -> bool {
        if offset + code.len() > self.size { return false; }
        unsafe {
            core::ptr::copy_nonoverlapping(code.as_ptr(), self.ptr.add(offset), code.len());
        }
        true
    }

    /// Get a function pointer to JIT code at the given offset
    pub unsafe fn as_fn_ptr<F>(&self, offset: usize) -> *const F {
        self.ptr.add(offset) as *const F
    }
}

impl Drop for JitPage {
    fn drop(&mut self) {
        if !self.ptr.is_null() {
            JitPool::dealloc(self.ptr, self.pages);
        }
    }
}

// ────────────────────────────────────────────────────────────────
//  Architecture-specific I-Cache flush
// ────────────────────────────────────────────────────────────────

#[cfg(target_arch = "aarch64")]
unsafe fn flush_icache(ptr: *mut u8, size: usize) {
    let mut addr = ptr as usize;
    let end = addr + size;

    // Step 1: Clean data cache to Point of Unification
    while addr < end {
        core::arch::asm!("dc cvau, {0}", in(reg) addr, options(nomem, nostack));
        addr += 64; // Cache line size on Cortex-A76
    }

    // Step 2: Data Synchronization Barrier (ensure DC ops complete)
    core::arch::asm!("dsb ish", options(nomem, nostack));

    // Step 3: Invalidate instruction cache
    addr = ptr as usize;
    while addr < end {
        core::arch::asm!("ic ivau, {0}", in(reg) addr, options(nomem, nostack));
        addr += 64;
    }

    // Step 4: Final barriers
    core::arch::asm!("dsb ish", options(nomem, nostack));
    core::arch::asm!("isb", options(nomem, nostack));
}

#[cfg(target_arch = "x86_64")]
unsafe fn flush_icache(_ptr: *mut u8, _size: usize) {
    // x86_64: I-cache is coherent with D-cache.
    // Only need a serializing instruction after PTE change.
    core::arch::asm!("mfence", options(nomem, nostack));
    core::arch::asm!(
        "xor eax, eax",
        "cpuid",
        out("eax") _, out("ebx") _, out("ecx") _, out("edx") _,
        options(nomem, nostack)
    );
}

#[cfg(not(any(target_arch = "aarch64", target_arch = "x86_64")))]
unsafe fn flush_icache(_ptr: *mut u8, _size: usize) {
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
}
