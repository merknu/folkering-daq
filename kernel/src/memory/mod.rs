//! Memory management — heap allocator + page frame allocator + JIT support
//!
//! Uses Limine memory map to find usable regions.
//! Heap is allocated from the largest usable region.
//! JIT module provides W^X memory for Silverfir-nano code generation.

pub mod jit;
pub mod pagetable;

use linked_list_allocator::LockedHeap;
use crate::MemoryRegion;
use crate::MemoryRegionKind;

#[global_allocator]
static ALLOCATOR: LockedHeap = LockedHeap::empty();

/// Heap size: 16 MiB (plenty for DAQ buffers + protocol state)
const HEAP_SIZE: usize = 16 * 1024 * 1024;

/// Initialize the heap allocator from memory map
pub fn init(memory_map: &[MemoryRegion]) {
    // Find largest usable region
    let mut best_base: u64 = 0;
    let mut best_size: u64 = 0;

    for region in memory_map {
        if region.kind == MemoryRegionKind::Usable && region.length > best_size {
            best_base = region.base;
            best_size = region.length;
        }
    }

    assert!(best_size >= HEAP_SIZE as u64, "Not enough memory for heap");

    // Place heap at start of largest usable region (via HHDM)
    let heap_start = crate::phys_to_virt(best_base) as usize;

    crate::kprintln!("  Heap: {:#x} — {:#x} ({} MiB)",
        heap_start, heap_start + HEAP_SIZE, HEAP_SIZE / 1024 / 1024);

    unsafe {
        ALLOCATOR.lock().init(heap_start as *mut u8, HEAP_SIZE);
    }
}
