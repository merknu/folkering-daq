//! Folkering DAQ Kernel
//!
//! Bare-metal data acquisition OS for Raspberry Pi 5.
//! Drives Dewesoft SIRIUSi-HS via USB, exposes data over openDAQ protocols.
//!
//! # Architecture
//!
//! - **aarch64** (Cortex-A76, BCM2712)
//! - **Limine boot protocol** for consistent bootloader interface
//! - **xHCI USB** for SIRIUSi-HS instrument
//! - **RP1 Ethernet** for openDAQ streaming to DewesoftX
//! - **smoltcp** no_std TCP/IP stack

#![no_std]
#![feature(allocator_api)]
#![feature(alloc_error_handler)]

extern crate alloc;

pub mod platform;
pub mod arch;
pub mod drivers;
pub mod usb;
pub mod net;
pub mod daq;
pub mod memory;
pub mod task;
pub mod panic;

use core::sync::atomic::{AtomicU64, Ordering};

/// HHDM (Higher Half Direct Map) offset from Limine
static HHDM_OFFSET: AtomicU64 = AtomicU64::new(0);

pub fn init_hhdm_offset(offset: u64) {
    HHDM_OFFSET.store(offset, Ordering::SeqCst);
}

pub fn hhdm_offset() -> u64 {
    HHDM_OFFSET.load(Ordering::Relaxed)
}

/// Convert physical address to virtual (via HHDM)
#[inline]
pub fn phys_to_virt(phys: u64) -> u64 {
    phys + hhdm_offset()
}

/// Convert virtual address (HHDM region) to physical
#[inline]
pub fn virt_to_phys(virt: u64) -> u64 {
    virt - hhdm_offset()
}

/// Boot information extracted from Limine responses
pub struct BootInfo {
    pub hhdm_offset: u64,
    pub framebuffer: Option<FramebufferInfo>,
    pub memory_map: &'static [MemoryRegion],
    pub dtb_addr: Option<u64>,
}

pub struct FramebufferInfo {
    pub addr: u64,
    pub width: u32,
    pub height: u32,
    pub pitch: u32,
    pub bpp: u16,
}

#[derive(Debug, Clone, Copy)]
pub struct MemoryRegion {
    pub base: u64,
    pub length: u64,
    pub kind: MemoryRegionKind,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MemoryRegionKind {
    Usable,
    Reserved,
    AcpiReclaimable,
    BootloaderReclaimable,
    KernelAndModules,
    Framebuffer,
}

/// Main kernel initialization — called from main.rs after Limine handoff
pub fn kernel_main(boot_info: &BootInfo) -> ! {
    init_hhdm_offset(boot_info.hhdm_offset);

    // Phase 1: Serial console (debug output)
    arch::aarch64::uart::init();
    kprintln!("=== Folkering DAQ Edition ===");
    kprintln!("Platform: {}", platform::PLATFORM_NAME);
    kprintln!("HHDM offset: {:#x}", boot_info.hhdm_offset);

    // Phase 2: Exception vectors + GIC
    arch::aarch64::exceptions::init();
    kprintln!("[OK] Exception vectors installed");

    arch::aarch64::gic::init();
    kprintln!("[OK] GIC-400 (GICv2) interrupt controller");

    // Phase 3: Timer
    arch::aarch64::timer::init();
    kprintln!("[OK] ARM Generic Timer");

    // Phase 4: Memory allocator
    memory::init(boot_info.memory_map);
    kprintln!("[OK] Heap allocator");

    // Phase 5+: Platform-specific hardware init
    #[cfg(feature = "pi5")]
    {
        // Pi 5: RP1 south bridge → xHCI → SIRIUS → GEM Ethernet
        drivers::pci::init();
        kprintln!("[OK] PCIe / RP1 south bridge");

        drivers::xhci::init();
        kprintln!("[OK] xHCI USB controller");

        usb::sirius::init();
        kprintln!("[OK] SIRIUSi-HS DAQ instrument");

        net::init();
        kprintln!("[OK] Cadence GEM Ethernet + smoltcp");
    }

    #[cfg(feature = "qemu-virt")]
    {
        // QEMU virt: no RP1, no real USB — demo/test mode
        kprintln!("[QEMU] No RP1/xHCI/SIRIUS — test mode");
        kprintln!("[QEMU] Networking: use -device virtio-net-pci (future)");
        kprintln!("[QEMU] Uptime test running...");
    }

    // openDAQ server (works on both platforms once networking is up)
    daq::init();
    kprintln!("[OK] openDAQ signal descriptors");

    kprintln!("\n*** Folkering DAQ ready ***");
    kprintln!("  Timer freq: {} Hz", arch::aarch64::counter_freq());
    kprintln!("");

    // Timer IRQs disabled for now — we use counter-based polling
    // arch::aarch64::enable_interrupts();

    // Main loop — use counter-based timing instead of IRQ ticks
    let mut last_status = arch::aarch64::timer::millis();
    loop {
        #[cfg(feature = "pi5")]
        {
            usb::sirius::poll();
            net::poll();
        }

        daq::poll();

        // Periodic status (every ~5 seconds)
        let now = arch::aarch64::timer::millis();
        if now - last_status >= 5000 {
            last_status = now;
            kprintln!("[{}ms] alive", now);

            #[cfg(feature = "pi5")]
            if usb::sirius::is_streaming() {
                kprintln!("  SIRIUS: {} packets", usb::sirius::packets_received());
            }
        }

        // Yield CPU briefly (spin a few iterations then WFE)
        for _ in 0..1000 { core::hint::spin_loop(); }
    }
}

// Serial print macros
#[macro_export]
macro_rules! kprint {
    ($($arg:tt)*) => {{
        $crate::arch::aarch64::uart::_print(format_args!($($arg)*))
    }};
}

#[macro_export]
macro_rules! kprintln {
    () => {{ $crate::kprint!("\n") }};
    ($($arg:tt)*) => {{ $crate::kprint!("{}\n", format_args!($($arg)*)) }};
}
