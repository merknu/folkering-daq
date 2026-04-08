//! GICv3 (Generic Interrupt Controller) driver
//!
//! Raspberry Pi 5 BCM2712 uses GIC-400 (GICv2 compatible) but the
//! Cortex-A76 CPU interface supports GICv3 system registers.
//!
//! Key components:
//! - GICD (Distributor): Global interrupt routing, priority, enable
//! - GICR (Redistributor): Per-CPU SGI/PPI config (GICv3)
//! - GICC (CPU Interface): Interrupt acknowledgment/completion
//!
//! For Pi 5: Base addresses come from Device Tree, but common defaults:
//! - GICD: 0xFF841000 (BCM2712)
//! - GICC: 0xFF842000

use core::sync::atomic::{AtomicU64, Ordering};

static GICD_BASE: AtomicU64 = AtomicU64::new(0);
static GICC_BASE: AtomicU64 = AtomicU64::new(0);

// GICD registers
const GICD_CTLR: u64 = 0x000;
const GICD_TYPER: u64 = 0x004;
const GICD_ISENABLER: u64 = 0x100;   // Set-Enable (32 IRQs per register)
const GICD_ICENABLER: u64 = 0x180;   // Clear-Enable
const GICD_ISPENDR: u64 = 0x200;     // Set-Pending
const GICD_ICPENDR: u64 = 0x280;     // Clear-Pending
const GICD_IPRIORITYR: u64 = 0x400;  // Priority (1 byte per IRQ)
const GICD_ITARGETSR: u64 = 0x800;   // Target CPU (1 byte per IRQ)
const GICD_ICFGR: u64 = 0xC00;       // Config (2 bits per IRQ)

// GICC registers
const GICC_CTLR: u64 = 0x000;
const GICC_PMR: u64 = 0x004;     // Priority Mask
const GICC_IAR: u64 = 0x00C;     // Interrupt Acknowledge
const GICC_EOIR: u64 = 0x010;    // End of Interrupt

// IRQ numbers for Pi 5
pub const IRQ_TIMER: u32 = 30;       // Non-secure Physical Timer (PPI)
pub const IRQ_UART: u32 = 153;       // UART0 via RP1 (SPI, approximate)
pub const IRQ_XHCI: u32 = 0;        // Filled in from PCIe MSI setup
pub const IRQ_ETH: u32 = 0;         // Filled in from PCIe MSI setup

// Callback table for IRQ handlers
const MAX_IRQS: usize = 256;
static mut IRQ_HANDLERS: [Option<fn(u32)>; MAX_IRQS] = [None; MAX_IRQS];

/// Initialize GIC distributor and CPU interface
pub fn init() {
    // Default addresses for QEMU virt machine
    // Pi 5 addresses will be read from DTB later
    #[cfg(not(feature = "pi5"))]
    {
        GICD_BASE.store(crate::phys_to_virt(0x0800_0000), Ordering::SeqCst);
        GICC_BASE.store(crate::phys_to_virt(0x0801_0000), Ordering::SeqCst);
    }

    #[cfg(feature = "pi5")]
    {
        GICD_BASE.store(crate::phys_to_virt(0xFF84_1000), Ordering::SeqCst);
        GICC_BASE.store(crate::phys_to_virt(0xFF84_2000), Ordering::SeqCst);
    }

    let gicd = GICD_BASE.load(Ordering::Relaxed);
    let gicc = GICC_BASE.load(Ordering::Relaxed);

    unsafe {
        // Disable distributor
        super::mmio_write32(gicd + GICD_CTLR, 0);

        // Read number of supported IRQs
        let typer = super::mmio_read32(gicd + GICD_TYPER);
        let num_irqs = ((typer & 0x1F) + 1) * 32;
        crate::kprintln!("  GIC: {} IRQs supported", num_irqs);

        // Set all IRQs to lowest priority (0xFF) and target CPU0
        for i in (32..num_irqs).step_by(4) {
            super::mmio_write32(gicd + GICD_IPRIORITYR + i as u64, 0xA0A0A0A0);
            super::mmio_write32(gicd + GICD_ITARGETSR + i as u64, 0x01010101);
        }

        // Disable all SPIs initially
        for i in (32..num_irqs).step_by(32) {
            super::mmio_write32(gicd + GICD_ICENABLER + (i / 32 * 4) as u64, 0xFFFFFFFF);
        }

        // Enable distributor (Group 0 + Group 1)
        super::mmio_write32(gicd + GICD_CTLR, 0x3);

        // CPU interface: set priority mask to allow all, enable
        super::mmio_write32(gicc + GICC_PMR, 0xFF);
        super::mmio_write32(gicc + GICC_CTLR, 0x1);
    }
}

/// Enable a specific IRQ
pub fn enable_irq(irq: u32) {
    let gicd = GICD_BASE.load(Ordering::Relaxed);
    let reg = (irq / 32) as u64 * 4;
    let bit = 1u32 << (irq % 32);
    unsafe {
        super::mmio_write32(gicd + GICD_ISENABLER + reg, bit);
    }
}

/// Disable a specific IRQ
pub fn disable_irq(irq: u32) {
    let gicd = GICD_BASE.load(Ordering::Relaxed);
    let reg = (irq / 32) as u64 * 4;
    let bit = 1u32 << (irq % 32);
    unsafe {
        super::mmio_write32(gicd + GICD_ICENABLER + reg, bit);
    }
}

/// Register an IRQ handler
pub fn register_handler(irq: u32, handler: fn(u32)) {
    if (irq as usize) < MAX_IRQS {
        unsafe { IRQ_HANDLERS[irq as usize] = Some(handler); }
        enable_irq(irq);
    }
}

/// Handle an IRQ — called from exception vector
pub fn handle_irq() {
    let gicc = GICC_BASE.load(Ordering::Relaxed);

    unsafe {
        // Acknowledge interrupt
        let iar = super::mmio_read32(gicc + GICC_IAR);
        let irq = iar & 0x3FF; // IRQ ID (bits 9:0)

        if irq == 1023 {
            return; // Spurious interrupt
        }

        // Dispatch to handler
        if (irq as usize) < MAX_IRQS {
            if let Some(handler) = IRQ_HANDLERS[irq as usize] {
                handler(irq);
            } else {
                crate::kprintln!("UNHANDLED IRQ #{}", irq);
            }
        }

        // Signal End of Interrupt
        super::mmio_write32(gicc + GICC_EOIR, iar);
    }
}
