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
pub mod wasm;
#[cfg(feature = "silverfir-jit")]
pub mod silverfir;
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

        // RP1 peripheral drivers (GPIO must be first — SPI/I2C depend on it for pin mux)
        if let Some(rp1) = drivers::pci::rp1() {
            drivers::rp1_gpio::init(rp1.bar1_virt);
            kprintln!("[OK] RP1 GPIO (28 pins)");

            drivers::rp1_spi::init(rp1.bar1_virt, &drivers::rp1_spi::SpiConfig::default());
            kprintln!("[OK] RP1 SPI0 (DW APB SSI)");

            drivers::rp1_i2c::init(rp1.bar1_virt, drivers::rp1_i2c::I2cBus::I2C0, drivers::rp1_i2c::I2cSpeed::Fast);
            drivers::rp1_i2c::init(rp1.bar1_virt, drivers::rp1_i2c::I2cBus::I2C1, drivers::rp1_i2c::I2cSpeed::Fast);
            kprintln!("[OK] RP1 I2C0 + I2C1 (DW I2C, 400 kHz)");
        }

        // SDHCI → SDIO → CYW43455 WiFi
        match drivers::sdhci::init() {
            Ok(()) => kprintln!("[OK] SDHCI WiFi (CYW43455 via SDIO)"),
            Err(e) => kprintln!("[WARN] SDHCI WiFi init failed: {}", e),
        }

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
    kprintln!("[OK] openDAQ signal descriptors + ring buffers");

    // Phase 6: WASM runtime (wasmi interpreter)
    kprintln!("[OK] wasmi WASM runtime ready");
    kprintln!("     Host functions: 7 DAQ functions registered via Linker");

    // Load embedded default DSP app (lowpass filter compiled into the kernel)
    static mut WASM_APP: Option<wasm::runtime::DaqWasmApp> = None;

    #[cfg(feature = "embedded-dsp")]
    {
        static EMBEDDED_WASM: &[u8] = include_bytes!("../../wasm-apps/lowpass_filter.wasm");
        kprintln!("  WASM: loading embedded DSP app ({} bytes)...", EMBEDDED_WASM.len());

        match wasm::runtime::DaqWasmApp::new(EMBEDDED_WASM) {
            Ok(mut app) => {
                match app.call_init() {
                    wasm::runtime::DaqWasmResult::Ok => {
                        kprintln!("  WASM: embedded lowpass filter loaded OK");
                        unsafe { WASM_APP = Some(app); }
                    }
                    _ => kprintln!("  WASM: embedded init failed"),
                }
            }
            Err(e) => kprintln!("  WASM: embedded load failed: {}", e),
        }
    }

    #[cfg(not(feature = "embedded-dsp"))]
    kprintln!("  WASM: no embedded app (upload via TCP :7421)");

    kprintln!("\n*** Folkering DAQ ready ***");
    kprintln!("  Timer freq: {} Hz", arch::aarch64::counter_freq());
    kprintln!("  Ring buffer: {} KiB ({} channels)",
        daq::ring_memory_size() / 1024, usb::sirius::NUM_CHANNELS + 1);
    kprintln!("  Hot-swap: TCP :7421 (python tools/deploy_wasm.py <app>)");
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

        // Check for hot-swap: load new WASM module from network
        if let Some(wasm_bytes) = net::hotswap::take_pending_wasm() {
            kprintln!("  HOTSWAP: loading {} bytes...", wasm_bytes.len());
            match wasm::runtime::DaqWasmApp::new(&wasm_bytes) {
                Ok(mut app) => {
                    match app.call_init() {
                        wasm::runtime::DaqWasmResult::Ok => {
                            kprintln!("  HOTSWAP: module loaded and initialized");
                            unsafe { WASM_APP = Some(app); }
                            net::hotswap::send_response(true, "module loaded");
                        }
                        _ => {
                            kprintln!("  HOTSWAP: init failed");
                            net::hotswap::send_response(false, "init failed");
                        }
                    }
                }
                Err(e) => {
                    kprintln!("  HOTSWAP: load error: {}", e);
                    net::hotswap::send_response(false, &e);
                }
            }
        }

        // Tick WASM DSP app (if loaded)
        unsafe {
            if let Some(ref mut app) = WASM_APP {
                if app.is_healthy() {
                    match app.tick() {
                        wasm::runtime::DaqWasmResult::Ok => {}
                        wasm::runtime::DaqWasmResult::OutOfFuel => {
                            kprintln!("  WASM: out of fuel (tick too expensive)");
                        }
                        wasm::runtime::DaqWasmResult::Trap(msg) => {
                            kprintln!("  WASM TRAP: {}", msg);
                        }
                        _ => {}
                    }
                }
            }
        }

        // Periodic status (every ~5 seconds)
        let now = arch::aarch64::timer::millis();
        if now - last_status >= 5000 {
            last_status = now;
            kprintln!("[{}ms] alive", now);

            #[cfg(feature = "pi5")]
            if usb::sirius::is_streaming() {
                kprintln!("  SIRIUS: {} packets", usb::sirius::packets_received());
            }

            // Report ring buffer fill level
            let avail = daq::available_samples();
            if avail > 0 {
                kprintln!("  Ring: {} samples buffered", avail);
            }

            // Report WASM app status
            unsafe {
                if let Some(ref app) = WASM_APP {
                    kprintln!("  WASM: {} ticks, healthy={}",
                        app.tick_count(), app.is_healthy());
                }
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
