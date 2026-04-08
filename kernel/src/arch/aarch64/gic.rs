//! GIC-400 (GICv2) driver for BCM2712
//!
//! BCM2712 uses a GIC-400 implementing the GICv2 architecture.
//! This means interrupts are configured via MMIO registers,
//! NOT via system registers (ICC_SRE_EL1 etc. as in GICv3).
//!
//! Components:
//! - GICD (Distributor): 0x10_7FFF_9000 — global interrupt routing/priority/enable
//! - GICC (CPU Interface): 0x10_7FFF_A000 — per-CPU acknowledge/EOI
//!
//! RP1 devices (xHCI, Ethernet) send MSI-X over PCIe which the BCM2712
//! MSI translator converts into SPI interrupts routed through GIC-400.

use core::sync::atomic::{AtomicU64, Ordering};

// Physical base addresses (BCM2712 hardware)
const GICD_PHYS: u64 = 0x10_7FFF_9000;
const GICC_PHYS: u64 = 0x10_7FFF_A000;

static GICD_BASE: AtomicU64 = AtomicU64::new(0);
static GICC_BASE: AtomicU64 = AtomicU64::new(0);

// GICD register offsets
const GICD_CTLR: u64 = 0x000;        // Distributor Control Register
const GICD_TYPER: u64 = 0x004;       // Interrupt Controller Type
const GICD_ISENABLER: u64 = 0x100;   // Set-Enable (32 IRQs per register)
const GICD_ICENABLER: u64 = 0x180;   // Clear-Enable
const GICD_ISPENDR: u64 = 0x200;     // Set-Pending
const GICD_ICPENDR: u64 = 0x280;     // Clear-Pending
const GICD_IPRIORITYR: u64 = 0x400;  // Priority (1 byte per IRQ)
const GICD_ITARGETSR: u64 = 0x800;   // Target CPU (1 byte per IRQ)
const GICD_ICFGR: u64 = 0xC00;       // Config (2 bits per IRQ: level/edge)

// GICC register offsets
const GICC_CTLR: u64 = 0x000;        // CPU Interface Control
const GICC_PMR: u64 = 0x004;         // Priority Mask
const GICC_IAR: u64 = 0x00C;         // Interrupt Acknowledge (read → get IRQ ID)
const GICC_EOIR: u64 = 0x010;        // End of Interrupt (write → clear IRQ)

// Maximum IRQ handlers
const MAX_IRQS: usize = 256;
static mut IRQ_HANDLERS: [Option<fn(u32)>; MAX_IRQS] = [None; MAX_IRQS];

/// Initialize GIC-400 distributor and CPU interface
pub fn init() {
    // Map physical addresses through HHDM
    let gicd = crate::phys_to_virt(GICD_PHYS);
    let gicc = crate::phys_to_virt(GICC_PHYS);
    GICD_BASE.store(gicd, Ordering::SeqCst);
    GICC_BASE.store(gicc, Ordering::SeqCst);

    unsafe {
        // Step 1: Disable distributor before configuration
        write_gicd(GICD_CTLR, 0);

        // Read number of supported IRQs
        let typer = read_gicd(GICD_TYPER);
        let num_irqs = ((typer & 0x1F) + 1) * 32;
        crate::kprintln!("  GIC-400 (GICv2): {} IRQs, GICD={:#x} GICC={:#x}",
            num_irqs, GICD_PHYS, GICC_PHYS);

        // Step 2: Configure all SPIs (IRQ 32+)
        for i in (32..num_irqs).step_by(4) {
            let offset = i as u64;
            // Set priority to 0xA0 (medium) for all
            write_gicd(GICD_IPRIORITYR + offset, 0xA0A0_A0A0);
            // Route all SPIs to CPU 0 (bitmask 0x01)
            write_gicd(GICD_ITARGETSR + offset, 0x0101_0101);
        }

        // Disable all SPIs initially
        for i in (32..num_irqs).step_by(32) {
            write_gicd(GICD_ICENABLER + (i / 32 * 4) as u64, 0xFFFF_FFFF);
        }

        // Step 3: Enable distributor (EnableGrp0 + EnableGrp1)
        write_gicd(GICD_CTLR, 0x3);

        // Step 4: Configure CPU Interface for this core
        // Set priority mask to allow all priorities
        write_gicc(GICC_PMR, 0xFF);

        // Step 5: Enable CPU Interface
        write_gicc(GICC_CTLR, 0x1);
    }
}

/// Enable a specific IRQ line
pub fn enable_irq(irq: u32) {
    let reg_offset = (irq / 32) as u64 * 4;
    let bit = 1u32 << (irq % 32);
    unsafe { write_gicd(GICD_ISENABLER + reg_offset, bit); }
}

/// Disable a specific IRQ line
pub fn disable_irq(irq: u32) {
    let reg_offset = (irq / 32) as u64 * 4;
    let bit = 1u32 << (irq % 32);
    unsafe { write_gicd(GICD_ICENABLER + reg_offset, bit); }
}

/// Register an IRQ handler and enable the IRQ
pub fn register_handler(irq: u32, handler: fn(u32)) {
    if (irq as usize) < MAX_IRQS {
        unsafe { IRQ_HANDLERS[irq as usize] = Some(handler); }
        enable_irq(irq);
    }
}

/// Handle an IRQ — called from exception vector (IRQ entry)
///
/// CRITICAL: Must read GICC_IAR to acknowledge, and write GICC_EOIR when done.
/// Failing to write EOIR will block all future interrupts of same/lower priority.
pub fn handle_irq() {
    let gicc = GICC_BASE.load(Ordering::Relaxed);

    unsafe {
        // Acknowledge interrupt — read IAR to get IRQ number
        let iar = core::ptr::read_volatile((gicc + GICC_IAR) as *const u32);
        let irq = iar & 0x3FF;

        if irq == 1023 {
            return; // Spurious interrupt
        }

        // Dispatch to registered handler
        if (irq as usize) < MAX_IRQS {
            if let Some(handler) = IRQ_HANDLERS[irq as usize] {
                handler(irq);
            } else {
                crate::kprintln!("UNHANDLED IRQ #{}", irq);
            }
        }

        // Signal End of Interrupt — MUST write the exact same value read from IAR
        core::ptr::write_volatile((gicc + GICC_EOIR) as *mut u32, iar);
    }
}

// Helper functions for volatile MMIO access

#[inline(always)]
unsafe fn read_gicd(offset: u64) -> u32 {
    let addr = GICD_BASE.load(Ordering::Relaxed) + offset;
    core::ptr::read_volatile(addr as *const u32)
}

#[inline(always)]
unsafe fn write_gicd(offset: u64, val: u32) {
    let addr = GICD_BASE.load(Ordering::Relaxed) + offset;
    core::ptr::write_volatile(addr as *mut u32, val);
}

#[inline(always)]
unsafe fn read_gicc(offset: u64) -> u32 {
    let addr = GICC_BASE.load(Ordering::Relaxed) + offset;
    core::ptr::read_volatile(addr as *const u32)
}

#[inline(always)]
unsafe fn write_gicc(offset: u64, val: u32) {
    let addr = GICC_BASE.load(Ordering::Relaxed) + offset;
    core::ptr::write_volatile(addr as *mut u32, val);
}
