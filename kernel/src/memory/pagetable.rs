//! Page Table Walker — Find and modify PTEs for JIT W^X transitions
//!
//! Supports both aarch64 (TTBR1_EL1, 4-level, 4KB granule) and
//! x86_64 (CR3, 4-level, 4KB pages).
//!
//! Limine sets up initial page tables with HHDM (Higher Half Direct Map).
//! All physical memory is accessible at `phys + hhdm_offset`.
//!
//! Key feature: Block Descriptor Splitting
//! If Limine mapped memory as 2MB or 1GB blocks, we split them into
//! individual 4KB PTEs on demand to enable per-page W^X control.

use super::jit::MemoryPermission;
use core::alloc::Layout;

const PAGE_SIZE: usize = 4096;
const ENTRIES_PER_TABLE: usize = 512;

// ── AArch64 Page Table Definitions ──────────────────────────────

#[cfg(target_arch = "aarch64")]
mod arch {
    pub const PTE_VALID: u64        = 1 << 0;
    pub const PTE_TABLE: u64        = 1 << 1;  // Table descriptor (vs block)
    pub const PTE_AF: u64           = 1 << 10; // Access Flag
    pub const PTE_SH_INNER: u64     = 3 << 8;  // Inner Shareable
    pub const PTE_PXN: u64          = 1 << 53; // Privileged eXecute Never
    pub const PTE_UXN: u64          = 1 << 54; // Unprivileged eXecute Never
    pub const PTE_AP_RW_EL1: u64    = 0b00 << 6;
    pub const PTE_AP_RO_EL1: u64    = 0b10 << 6;
    pub const PTE_AP_MASK: u64      = 0b11 << 6;

    /// Attribute bits to preserve when splitting blocks (AttrIndx + SH + AF + AP + UXN + PXN)
    pub const ATTR_MASK: u64 = 0x0060_0000_0000_0FFC | PTE_PXN | PTE_UXN;

    pub const PHYS_ADDR_MASK: u64 = 0x0000_FFFF_FFFF_F000;

    /// Block address masks per level
    pub const L1_BLOCK_MASK: u64 = 0x0000_FFFF_C000_0000; // 1GB aligned
    pub const L2_BLOCK_MASK: u64 = 0x0000_FFFF_FFE0_0000; // 2MB aligned

    #[inline]
    pub fn read_ttbr1() -> u64 {
        let val: u64;
        unsafe { core::arch::asm!("mrs {0}, TTBR1_EL1", out(reg) val, options(nomem, nostack)); }
        val & PHYS_ADDR_MASK
    }

    #[inline]
    pub fn read_ttbr0() -> u64 {
        let val: u64;
        unsafe { core::arch::asm!("mrs {0}, TTBR0_EL1", out(reg) val, options(nomem, nostack)); }
        val & PHYS_ADDR_MASK
    }

    #[inline]
    pub unsafe fn tlbi_page(vaddr: u64) {
        let page = vaddr >> 12;
        core::arch::asm!(
            "tlbi vae1is, {0}",
            "dsb ish",
            "isb",
            in(reg) page,
            options(nomem, nostack)
        );
    }

    /// Invalidate ALL TLB entries (used after block splitting)
    #[inline]
    pub unsafe fn tlbi_all() {
        core::arch::asm!(
            "tlbi vmalle1is",
            "dsb ish",
            "isb",
            options(nomem, nostack)
        );
    }

    #[inline]
    pub fn table_index(vaddr: u64, level: usize) -> usize {
        let shift = 12 + (3 - level) * 9;
        ((vaddr >> shift) & 0x1FF) as usize
    }

    #[inline]
    pub fn page_table_base(vaddr: u64) -> u64 {
        // Bit 55 determines TTBR: set = TTBR1 (kernel), clear = TTBR0 (user)
        if vaddr & (1u64 << 55) != 0 {
            read_ttbr1()
        } else {
            read_ttbr0()
        }
    }
}

// ── x86_64 Page Table Definitions ───────────────────────────────

#[cfg(target_arch = "x86_64")]
mod arch {
    pub const PTE_PRESENT: u64  = 1 << 0;
    pub const PTE_WRITABLE: u64 = 1 << 1;
    pub const PTE_PS: u64       = 1 << 7;  // Page Size (1 = huge page at this level)
    pub const PTE_NX: u64       = 1 << 63;

    pub const ATTR_MASK: u64 = 0xFFF0_0000_0000_0FFF; // All attribute bits
    pub const PHYS_ADDR_MASK: u64 = 0x000F_FFFF_FFFF_F000;

    #[inline]
    pub fn read_cr3() -> u64 {
        let val: u64;
        unsafe { core::arch::asm!("mov {0}, cr3", out(reg) val, options(nomem, nostack)); }
        val & PHYS_ADDR_MASK
    }

    #[inline]
    pub unsafe fn tlbi_page(vaddr: u64) {
        core::arch::asm!("invlpg [{}]", in(reg) vaddr, options(nomem, nostack));
    }

    #[inline]
    pub unsafe fn tlbi_all() {
        // Flush entire TLB by reloading CR3
        let cr3: u64;
        core::arch::asm!("mov {0}, cr3", out(reg) cr3, options(nomem, nostack));
        core::arch::asm!("mov cr3, {0}", in(reg) cr3, options(nomem, nostack));
    }

    #[inline]
    pub fn table_index(vaddr: u64, level: usize) -> usize {
        let shift = 12 + (3 - level) * 9;
        ((vaddr >> shift) & 0x1FF) as usize
    }

    #[inline]
    pub fn page_table_base(_vaddr: u64) -> u64 {
        read_cr3()
    }
}

// ── Block Descriptor Splitting ──────────────────────────────────

/// Allocate a zeroed 4KB page for a new page table.
/// Returns the PHYSICAL address of the new table.
unsafe fn alloc_page_table() -> Option<u64> {
    let layout = Layout::from_size_align(PAGE_SIZE, PAGE_SIZE).ok()?;
    let ptr = alloc::alloc::alloc_zeroed(layout);
    if ptr.is_null() { return None; }

    // Convert virtual (HHDM) address to physical
    let phys = crate::virt_to_phys(ptr as u64);
    Some(phys)
}

/// Split a block descriptor into a table of sub-entries.
///
/// On aarch64:
///   L1 block (1GB) → L2 table with 512 × 2MB block entries
///   L2 block (2MB) → L3 table with 512 × 4KB page entries
///
/// On x86_64:
///   L1 (PDPT) huge (1GB) → L2 table with 512 × 2MB entries
///   L2 (PD) huge (2MB)   → L3 table with 512 × 4KB entries
#[cfg(target_arch = "aarch64")]
unsafe fn split_block(entry_ptr: *mut u64, level: usize) -> bool {
    let block_entry = core::ptr::read_volatile(entry_ptr);

    // Must be a valid block (not already a table)
    if block_entry & arch::PTE_VALID == 0 { return false; }
    if block_entry & arch::PTE_TABLE != 0 { return true; } // Already a table

    // Allocate new page table
    let new_table_phys = match alloc_page_table() {
        Some(p) => p,
        None => {
            crate::kprintln!("  PTE: failed to allocate page table for block split");
            return false;
        }
    };

    let hhdm = crate::hhdm_offset();
    let new_table_virt = (new_table_phys + hhdm) as *mut u64;

    // Extract physical base and attributes from block descriptor
    let attrs = block_entry & arch::ATTR_MASK;

    match level {
        1 => {
            // L1 block = 1GB → split into 512 × 2MB L2 block entries
            let block_base = block_entry & arch::L1_BLOCK_MASK;
            for i in 0..ENTRIES_PER_TABLE {
                let sub_phys = block_base + (i as u64) * (2 * 1024 * 1024); // 2MB each
                // L2 block entry: valid + block (NOT table) + attributes
                let sub_entry = sub_phys | attrs | arch::PTE_VALID | arch::PTE_AF;
                // Note: PTE_TABLE bit is NOT set → these are block entries
                core::ptr::write_volatile(new_table_virt.add(i), sub_entry);
            }
        }
        2 => {
            // L2 block = 2MB → split into 512 × 4KB L3 page entries
            let block_base = block_entry & arch::L2_BLOCK_MASK;
            for i in 0..ENTRIES_PER_TABLE {
                let sub_phys = block_base + (i as u64) * PAGE_SIZE as u64;
                // L3 page entry: valid + table bit (at L3, bit 1 means "page" not "block")
                let sub_entry = sub_phys | attrs | arch::PTE_VALID | arch::PTE_TABLE | arch::PTE_AF;
                core::ptr::write_volatile(new_table_virt.add(i), sub_entry);
            }
        }
        _ => return false,
    }

    // Replace the block entry with a table descriptor pointing to new table
    let table_desc = new_table_phys | arch::PTE_VALID | arch::PTE_TABLE;
    core::ptr::write_volatile(entry_ptr, table_desc);

    // Invalidate entire TLB (block split affects many addresses)
    arch::tlbi_all();

    crate::kprintln!("  PTE: split L{} block at {:#x} → table at {:#x}",
        level, block_entry & arch::PHYS_ADDR_MASK, new_table_phys);

    true
}

#[cfg(target_arch = "x86_64")]
unsafe fn split_block(entry_ptr: *mut u64, level: usize) -> bool {
    let block_entry = core::ptr::read_volatile(entry_ptr);

    if block_entry & arch::PTE_PRESENT == 0 { return false; }
    if block_entry & arch::PTE_PS == 0 { return true; } // Already 4KB, not huge

    let new_table_phys = match alloc_page_table() {
        Some(p) => p,
        None => return false,
    };

    let hhdm = crate::hhdm_offset();
    let new_table_virt = (new_table_phys + hhdm) as *mut u64;

    let attrs = block_entry & arch::ATTR_MASK & !arch::PTE_PS; // Remove PS bit

    match level {
        1 => {
            // 1GB huge → 512 × 2MB huge entries
            let block_base = block_entry & 0x000F_FFFF_C000_0000;
            for i in 0..ENTRIES_PER_TABLE {
                let sub_phys = block_base + (i as u64) * (2 * 1024 * 1024);
                let sub_entry = sub_phys | attrs | arch::PTE_PRESENT | arch::PTE_PS;
                core::ptr::write_volatile(new_table_virt.add(i), sub_entry);
            }
        }
        2 => {
            // 2MB huge → 512 × 4KB regular entries
            let block_base = block_entry & 0x000F_FFFF_FFE0_0000;
            for i in 0..ENTRIES_PER_TABLE {
                let sub_phys = block_base + (i as u64) * PAGE_SIZE as u64;
                let sub_entry = sub_phys | attrs | arch::PTE_PRESENT;
                core::ptr::write_volatile(new_table_virt.add(i), sub_entry);
            }
        }
        _ => return false,
    }

    // Replace with table entry (remove PS bit, point to new table)
    let table_desc = new_table_phys | (block_entry & 0xFFF & !arch::PTE_PS) | arch::PTE_PRESENT;
    core::ptr::write_volatile(entry_ptr, table_desc);

    arch::tlbi_all();
    true
}

// ── Page Table Walker ───────────────────────────────────────────

/// Walk the 4-level page table to find the L3 PTE for a virtual address.
/// Splits block descriptors on demand if encountered.
unsafe fn walk_page_table(vaddr: u64) -> Option<*mut u64> {
    let hhdm = crate::hhdm_offset();
    let mut table_phys = arch::page_table_base(vaddr);

    for level in 0..3 {
        let index = arch::table_index(vaddr, level);
        let entry_ptr = ((table_phys + hhdm) as *mut u64).add(index);
        let entry = core::ptr::read_volatile(entry_ptr);

        // Check valid/present
        #[cfg(target_arch = "aarch64")]
        if entry & arch::PTE_VALID == 0 { return None; }
        #[cfg(target_arch = "x86_64")]
        if entry & arch::PTE_PRESENT == 0 { return None; }

        // Check for block/huge page descriptor
        #[cfg(target_arch = "aarch64")]
        {
            if entry & arch::PTE_TABLE == 0 {
                // Block descriptor — split it into a table
                if !split_block(entry_ptr, level) {
                    return None;
                }
                // Re-read the entry (now a table descriptor)
                let entry = core::ptr::read_volatile(entry_ptr);
                table_phys = entry & arch::PHYS_ADDR_MASK;
                continue;
            }
            table_phys = entry & arch::PHYS_ADDR_MASK;
        }

        #[cfg(target_arch = "x86_64")]
        {
            if entry & arch::PTE_PS != 0 {
                // Huge page — split it
                if !split_block(entry_ptr, level) {
                    return None;
                }
                let entry = core::ptr::read_volatile(entry_ptr);
                table_phys = entry & arch::PHYS_ADDR_MASK;
                continue;
            }
            table_phys = entry & arch::PHYS_ADDR_MASK;
        }
    }

    // Level 3: final PTE
    let index = arch::table_index(vaddr, 3);
    let entry_ptr = ((table_phys + hhdm) as *mut u64).add(index);
    let entry = core::ptr::read_volatile(entry_ptr);

    #[cfg(target_arch = "aarch64")]
    if entry & arch::PTE_VALID == 0 { return None; }
    #[cfg(target_arch = "x86_64")]
    if entry & arch::PTE_PRESENT == 0 { return None; }

    Some(entry_ptr)
}

// ── Permission Modification ─────────────────────────────────────

/// Change the memory permissions of a single 4KB page.
pub unsafe fn set_page_permissions(vaddr: u64, perm: MemoryPermission) -> bool {
    let pte_ptr = match walk_page_table(vaddr) {
        Some(p) => p,
        None => return false,
    };

    let mut entry = core::ptr::read_volatile(pte_ptr);

    #[cfg(target_arch = "aarch64")]
    {
        match perm {
            MemoryPermission::ReadWrite => {
                entry &= !arch::PTE_AP_MASK;
                entry |= arch::PTE_AP_RW_EL1;
                entry |= arch::PTE_PXN | arch::PTE_UXN; // No execute
            }
            MemoryPermission::ReadExecute => {
                entry &= !arch::PTE_AP_MASK;
                entry |= arch::PTE_AP_RO_EL1;
                entry &= !arch::PTE_PXN; // Allow privileged execute
                entry |= arch::PTE_UXN;  // Deny user execute
            }
        }
        entry |= arch::PTE_AF;
    }

    #[cfg(target_arch = "x86_64")]
    {
        match perm {
            MemoryPermission::ReadWrite => {
                entry |= arch::PTE_WRITABLE;
                entry |= arch::PTE_NX;
            }
            MemoryPermission::ReadExecute => {
                entry &= !arch::PTE_WRITABLE;
                entry &= !arch::PTE_NX;
            }
        }
    }

    core::ptr::write_volatile(pte_ptr, entry);
    arch::tlbi_page(vaddr);

    true
}

/// Change permissions for a range of pages.
/// Returns number of pages successfully modified.
pub unsafe fn set_range_permissions(base: *mut u8, size: usize, perm: MemoryPermission) -> usize {
    let mut addr = base as u64;
    let end = addr + size as u64;
    let mut count = 0;

    while addr < end {
        if set_page_permissions(addr, perm) {
            count += 1;
        }
        addr += PAGE_SIZE as u64;
    }

    count
}
