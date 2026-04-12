//! Page Table Walker — Find and modify PTEs for JIT W^X transitions
//!
//! Supports both aarch64 (TTBR1_EL1, 4-level, 4KB granule) and
//! x86_64 (CR3, 4-level, 4KB pages).
//!
//! Limine sets up the initial page tables with HHDM (Higher Half Direct Map),
//! so all physical memory is identity-mapped at `phys + hhdm_offset`.
//! We walk these existing tables to find the PTE for a given virtual address
//! and flip permission bits.

use super::jit::MemoryPermission;

const PAGE_SIZE: usize = 4096;

// === AArch64 Page Table Definitions ===

#[cfg(target_arch = "aarch64")]
mod arch {
    /// AArch64 4KB granule, 4-level translation (L0-L3)
    /// VA[47:39] = L0 index, VA[38:30] = L1, VA[29:21] = L2, VA[20:12] = L3
    const VA_BITS: usize = 48;
    const ENTRIES_PER_TABLE: usize = 512;

    /// PTE bit definitions for AArch64 (stage 1, EL1)
    pub const PTE_VALID: u64        = 1 << 0;
    pub const PTE_TABLE: u64        = 1 << 1;  // Table descriptor (vs block)
    pub const PTE_AF: u64           = 1 << 10; // Access Flag
    pub const PTE_PXN: u64          = 1 << 53; // Privileged eXecute Never
    pub const PTE_UXN: u64          = 1 << 54; // Unprivileged eXecute Never
    pub const PTE_AP_RW_EL1: u64    = 0b00 << 6; // AP[2:1] = RW at EL1
    pub const PTE_AP_RO_EL1: u64    = 0b10 << 6; // AP[2:1] = RO at EL1
    pub const PTE_AP_MASK: u64      = 0b11 << 6;

    /// Physical address mask (bits [47:12] for 4KB granule)
    pub const PHYS_ADDR_MASK: u64 = 0x0000_FFFF_FFFF_F000;

    /// Read TTBR1_EL1 (kernel page table base for higher-half addresses)
    #[inline]
    pub fn read_ttbr1() -> u64 {
        let val: u64;
        unsafe { core::arch::asm!("mrs {0}, TTBR1_EL1", out(reg) val, options(nomem, nostack)); }
        val & PHYS_ADDR_MASK
    }

    /// Read TTBR0_EL1 (page table base for lower-half addresses)
    #[inline]
    pub fn read_ttbr0() -> u64 {
        let val: u64;
        unsafe { core::arch::asm!("mrs {0}, TTBR0_EL1", out(reg) val, options(nomem, nostack)); }
        val & PHYS_ADDR_MASK
    }

    /// TLB invalidation for a single page (TLBI VAE1IS)
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

    /// Extract page table index at a given level (0-3) from virtual address
    #[inline]
    pub fn table_index(vaddr: u64, level: usize) -> usize {
        let shift = 12 + (3 - level) * 9; // L0: 39, L1: 30, L2: 21, L3: 12
        ((vaddr >> shift) & 0x1FF) as usize
    }

    /// Determine which TTBR to use based on virtual address
    #[inline]
    pub fn page_table_base(vaddr: u64) -> u64 {
        if vaddr >= (1u64 << 48) - (1u64 << 47) {
            // Higher half → TTBR1
            read_ttbr1()
        } else {
            // Lower half → TTBR0
            read_ttbr0()
        }
    }
}

// === x86_64 Page Table Definitions ===

#[cfg(target_arch = "x86_64")]
mod arch {
    /// x86_64 4-level paging: PML4 → PDPT → PD → PT
    /// VA[47:39] = PML4 index, VA[38:30] = PDPT, VA[29:21] = PD, VA[20:12] = PT

    /// PTE bit definitions for x86_64
    pub const PTE_PRESENT: u64  = 1 << 0;
    pub const PTE_WRITABLE: u64 = 1 << 1;
    pub const PTE_NX: u64       = 1 << 63; // No-Execute

    /// Physical address mask (bits [51:12])
    pub const PHYS_ADDR_MASK: u64 = 0x000F_FFFF_FFFF_F000;

    /// Read CR3 (page table root)
    #[inline]
    pub fn read_cr3() -> u64 {
        let val: u64;
        unsafe { core::arch::asm!("mov {0}, cr3", out(reg) val, options(nomem, nostack)); }
        val & PHYS_ADDR_MASK
    }

    /// TLB invalidation for a single page (INVLPG)
    #[inline]
    pub unsafe fn tlbi_page(vaddr: u64) {
        core::arch::asm!("invlpg [{}]", in(reg) vaddr, options(nomem, nostack));
    }

    /// Extract page table index at a given level (0-3) from virtual address
    #[inline]
    pub fn table_index(vaddr: u64, level: usize) -> usize {
        let shift = 12 + (3 - level) * 9; // L0(PML4): 39, L1(PDPT): 30, L2(PD): 21, L3(PT): 12
        ((vaddr >> shift) & 0x1FF) as usize
    }

    /// Page table base is always CR3
    #[inline]
    pub fn page_table_base(_vaddr: u64) -> u64 {
        read_cr3()
    }
}

// === Architecture-Independent Page Table Walker ===

/// Walk the 4-level page table to find the PTE for a given virtual address.
/// Returns a mutable pointer to the PTE entry.
///
/// # Safety
/// - Requires HHDM to be set up (all physical memory accessible via phys + hhdm_offset)
/// - Must be called with interrupts disabled or from a context where TLB flush is safe
unsafe fn walk_page_table(vaddr: u64) -> Option<*mut u64> {
    let hhdm = crate::hhdm_offset();
    let mut table_phys = arch::page_table_base(vaddr);

    // Walk levels 0-2 (each level points to next table)
    for level in 0..3 {
        let index = arch::table_index(vaddr, level);
        let entry_ptr = ((table_phys + hhdm) as *mut u64).add(index);
        let entry = core::ptr::read_volatile(entry_ptr);

        // Check if entry is valid/present
        #[cfg(target_arch = "aarch64")]
        if entry & arch::PTE_VALID == 0 { return None; }
        #[cfg(target_arch = "x86_64")]
        if entry & arch::PTE_PRESENT == 0 { return None; }

        // Extract physical address of next table
        #[cfg(target_arch = "aarch64")]
        {
            // Check if it's a table descriptor (not a block mapping)
            if entry & arch::PTE_TABLE == 0 {
                // Block mapping at level 1 or 2 — can't modify individual page permissions
                // Would need to split the block into a table first
                return None;
            }
            table_phys = entry & arch::PHYS_ADDR_MASK;
        }
        #[cfg(target_arch = "x86_64")]
        {
            table_phys = entry & arch::PHYS_ADDR_MASK;
        }
    }

    // Level 3: this is the actual page table entry
    let index = arch::table_index(vaddr, 3);
    let entry_ptr = ((table_phys + hhdm) as *mut u64).add(index);
    let entry = core::ptr::read_volatile(entry_ptr);

    #[cfg(target_arch = "aarch64")]
    if entry & arch::PTE_VALID == 0 { return None; }
    #[cfg(target_arch = "x86_64")]
    if entry & arch::PTE_PRESENT == 0 { return None; }

    Some(entry_ptr)
}

/// Change the memory permissions of a single page.
///
/// # Safety
/// - The page at `vaddr` must be mapped in the current page table
/// - No thread should be executing code in this page during RX→RW transition
pub unsafe fn set_page_permissions(vaddr: u64, perm: MemoryPermission) -> bool {
    let pte_ptr = match walk_page_table(vaddr) {
        Some(p) => p,
        None => {
            crate::kprintln!("  PTE: walk failed for {:#x}", vaddr);
            return false;
        }
    };

    let mut entry = core::ptr::read_volatile(pte_ptr);

    #[cfg(target_arch = "aarch64")]
    {
        match perm {
            MemoryPermission::ReadWrite => {
                // RW, no execute: AP=RW_EL1, PXN=1, UXN=1
                entry &= !arch::PTE_AP_MASK;
                entry |= arch::PTE_AP_RW_EL1;
                entry |= arch::PTE_PXN | arch::PTE_UXN;
            }
            MemoryPermission::ReadExecute => {
                // RX, no write: AP=RO_EL1, PXN=0, UXN=1
                entry &= !arch::PTE_AP_MASK;
                entry |= arch::PTE_AP_RO_EL1;
                entry &= !arch::PTE_PXN; // Allow privileged execute
                entry |= arch::PTE_UXN;   // Still deny user execute
            }
        }
        // Ensure Access Flag is set
        entry |= arch::PTE_AF;
    }

    #[cfg(target_arch = "x86_64")]
    {
        match perm {
            MemoryPermission::ReadWrite => {
                // RW, no execute: W=1, NX=1
                entry |= arch::PTE_WRITABLE;
                entry |= arch::PTE_NX;
            }
            MemoryPermission::ReadExecute => {
                // RX, no write: W=0, NX=0
                entry &= !arch::PTE_WRITABLE;
                entry &= !arch::PTE_NX;
            }
        }
    }

    // Write back the modified PTE
    core::ptr::write_volatile(pte_ptr, entry);

    // Invalidate TLB for this page
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
