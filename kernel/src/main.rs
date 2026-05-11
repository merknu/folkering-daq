//! Folkering DAQ Kernel Entry Point (aarch64)
//!
//! Limine boot protocol — identical API on x86-64 and aarch64.
//! Extracts boot info, then hands off to kernel_main().

#![no_std]
#![no_main]

extern crate alloc;
extern crate folkering_daq_kernel;

use limine::BaseRevision;
use limine::request::{
    RequestsStartMarker, RequestsEndMarker,
    FramebufferRequest, MemoryMapRequest, HhdmRequest, ModuleRequest,
    SmpRequest, DeviceTreeBlobRequest,
};

use folkering_daq_kernel::{
    BootInfo, FramebufferInfo, MemoryRegion, MemoryRegionKind,
};

// --- Limine Requests ---

#[used]
#[link_section = ".requests"]
static BASE_REVISION: BaseRevision = BaseRevision::new();

#[used]
#[link_section = ".requests"]
static FRAMEBUFFER_REQUEST: FramebufferRequest = FramebufferRequest::new();

#[used]
#[link_section = ".requests"]
static MEMORY_MAP_REQUEST: MemoryMapRequest = MemoryMapRequest::new();

#[used]
#[link_section = ".requests"]
static HHDM_REQUEST: HhdmRequest = HhdmRequest::new();

#[used]
#[link_section = ".requests"]
static MODULE_REQUEST: ModuleRequest = ModuleRequest::new();

#[used]
#[link_section = ".requests"]
static SMP_REQUEST: SmpRequest = SmpRequest::new();

/// Device Tree Blob — critical for Pi 5 hardware discovery
/// Limine passes the DTB provided by the Pi firmware
#[used]
#[link_section = ".requests"]
static DTB_REQUEST: DeviceTreeBlobRequest = DeviceTreeBlobRequest::new();

#[used]
#[link_section = ".requests_start_marker"]
static _START_MARKER: RequestsStartMarker = RequestsStartMarker::new();

#[used]
#[link_section = ".requests_end_marker"]
static _END_MARKER: RequestsEndMarker = RequestsEndMarker::new();

// --- Stack ---
// Limine provides an initial stack, but we allocate a larger one for kernel use.
#[used]
#[link_section = ".bss"]
static mut KERNEL_STACK: [u8; 64 * 1024] = [0; 64 * 1024]; // 64 KiB

// --- Entry Point ---

/// Boot stub: set up stack pointer, then jump to kmain_rust.
/// QEMU starts with SP=0, so we MUST set it up before any Rust code runs.
#[cfg(feature = "qemu-virt")]
core::arch::global_asm!(r"
.section .text.boot
.global kmain
kmain:
    // Enable FP/SIMD (CPACR_EL1.FPEN = 0b11)
    mov x0, #(3 << 20)
    msr CPACR_EL1, x0
    isb

    // Set up stack: use a dedicated region in BSS
    adrp x0, __boot_stack_top
    add x0, x0, :lo12:__boot_stack_top
    mov sp, x0
    bl kmain_rust
1:  wfe
    b 1b

.section .bss
.balign 16
__boot_stack_bottom:
    .space 65536
__boot_stack_top:
");

/// Rust entry point (called from asm stub with valid stack).
#[cfg(feature = "qemu-virt")]
#[no_mangle]
extern "C" fn kmain_rust() -> ! {
    // QEMU identity maps memory — HHDM offset is 0
    let hhdm_offset = 0u64;

    let framebuffer = None;

    // Build minimal memory map from QEMU's DTB or hardcode for -m 512M
    static mut MEMORY_REGIONS: [MemoryRegion; 2] = [
        MemoryRegion { base: 0x4800_0000, length: 0x1800_0000, kind: MemoryRegionKind::Usable }, // ~384 MB
        MemoryRegion { base: 0, length: 0, kind: MemoryRegionKind::Reserved },
    ];
    let regions = unsafe { &MEMORY_REGIONS[..1] };

    let dtb_addr = None;

    let boot_info = BootInfo {
        hhdm_offset,
        framebuffer,
        memory_map: regions,
        dtb_addr,
    };

    folkering_daq_kernel::kernel_main(&boot_info)
}

/// Bare-metal Raspberry Pi 5 boot stub.
///
/// Pi 5 firmware (start4.elf / Limine-less direct boot via `kernel=kernel8.img`)
/// loads our raw binary to physical `0x80000` and jumps there. Compared to the
/// QEMU/Limine paths we have to handle the rough edges ourselves:
///
///   * **MPIDR park.** All four cores start; only core 0 (Aff0=0) runs the
///     kernel — secondaries spin in `wfe`.
///   * **EL2 → EL1 drop.** Firmware leaves us at EL2 on Pi 4/5. EL1 is where
///     our exception-vector / timer / GIC code expects to run.
///   * **Stack.** Firmware doesn't set up `sp`; we point it at a dedicated
///     64 KiB region in `.bss`.
///   * **BSS zero.** Firmware doesn't promise zeroed BSS. The linker exports
///     `__bss_start`/`__bss_end` and we clear it before any Rust runs.
///   * **DTB pointer.** Firmware passes the device-tree-blob physical address
///     in `x0`. We preserve it through the trampoline so `kmain_rust` can
///     consume it.
///
/// Without this block, jumping straight from firmware to a Limine-format
/// kernel image puts us at the (uninitialised) higher-half VA at boot and
/// the silicon hangs in the first microsecond — which is the bug that
/// kept the UART totally silent on hardware.
#[cfg(feature = "pi5")]
core::arch::global_asm!(r#"
.section .text.boot
.global _start
_start:
    // EARLY-BOOT UART PROOF-OF-LIFE.
    //
    // Pi firmware enables and configures PL011 UART0 itself when
    // `enable_uart=1` + `uart_2ndstage=1` are set in config.txt, so by the
    // time we land here the controller is already pumping at 115200 8N1.
    // Banging a literal byte directly into the data register before *any*
    // other init proves three things at once:
    //   1. Pi-firmware actually loaded and jumped to our kernel8.img (not
    //      stuck booting from NVMe / silently failing parse).
    //   2. The dedicated 3-pin debug header is wired to UART0 (the address
    //      we're using) — not some other UART on the SoC.
    //   3. The USB-serial cable + CH340 + PowerShell pipeline at the
    //      other end is end-to-end functional.
    //
    // If the host sees ":-)" come over UART here, every later init phase
    // that goes silent is a real kernel bug we can debug. If the host
    // still sees nothing, the kernel isn't running at all — boot order,
    // SD priority, or wiring needs to be checked.
    //
    // The byte sequence ":-)\r\n" is six characters, sent without polling
    // FR_TXFF — firmware-set FIFO depth is 16 bytes so a six-byte burst
    // is always safe.
    movz    x4, #0x1000
    movk    x4, #0x7D00, lsl #16
    movk    x4, #0x0010, lsl #32   // x4 = 0x0000_0010_7D00_1000 (UART0 DR)
    mov     w5, #58                 // ':'
    str     w5, [x4]
    mov     w5, #45                 // '-'
    str     w5, [x4]
    mov     w5, #41                 // ')'
    str     w5, [x4]
    mov     w5, #13                 // '\r'
    str     w5, [x4]
    mov     w5, #10                 // '\n'
    str     w5, [x4]

    // Park secondary cores. MPIDR_EL1[1:0] = Aff0 (core id within cluster);
    // anything non-zero is parked in wfe.
    mrs     x1, mpidr_el1
    and     x1, x1, #3
    cbnz    x1, .Lpark_secondary

    // Drop to EL1 if we landed at EL2 (Pi 5 firmware default).
    mrs     x1, CurrentEL
    lsr     x1, x1, #2
    cmp     x1, #2
    b.ne    .Lskip_el2_drop

    // HCR_EL2.RW = 1 → EL1 is AArch64.
    mov     x2, #(1 << 31)
    msr     HCR_EL2, x2

    // SCTLR_EL1 reset value (MMU off, caches off, exception endian = LE).
    msr     SCTLR_EL1, xzr

    // SPSR_EL2 = EL1h, DAIF masked (we'll unmask later when GIC is up).
    mov     x2, #0x3c5
    msr     SPSR_EL2, x2

    adr     x2, .Lel1_entry
    msr     ELR_EL2, x2
    eret

.Lel1_entry:
.Lskip_el2_drop:
    // Enable FP/SIMD at EL1 so Rust codegen doesn't trap on a fmov.
    mov     x2, #(3 << 20)
    msr     CPACR_EL1, x2
    isb

    // Stack pointer.
    adrp    x2, __boot_stack_top
    add     x2, x2, :lo12:__boot_stack_top
    mov     sp, x2

    // Zero the BSS (8 bytes at a time; alignment guaranteed by linker).
    adrp    x2, __bss_start
    add     x2, x2, :lo12:__bss_start
    adrp    x3, __bss_end
    add     x3, x3, :lo12:__bss_end
.Lbss_zero:
    cmp     x2, x3
    b.ge    .Lbss_done
    str     xzr, [x2], #8
    b       .Lbss_zero
.Lbss_done:

    // x0 still holds the DTB pointer from firmware — pass it through.
    bl      kmain_rust

.Lhang:
    wfe
    b       .Lhang

.Lpark_secondary:
    wfe
    b       .Lpark_secondary

.section .bss
.balign 16
__boot_stack_bottom:
    .space 65536
__boot_stack_top:
"#);

/// Rust entry on Pi 5. Called from the asm stub above with valid `sp`,
/// zeroed BSS, and the device-tree-blob physical pointer in x0.
///
/// We don't have a Limine memory map here — the DTB is the authoritative
/// source for RAM regions, but parsing it requires alloc + a working heap
/// (chicken-and-egg). For the first-boot bring-up we hard-code a single
/// usable region covering the standard 4 GiB Pi 5 RAM minus the first
/// MiB (firmware/GPU/kernel-image). DTB-driven memory discovery can come
/// after we've proven serial output works.
#[cfg(feature = "pi5")]
#[no_mangle]
extern "C" fn kmain_rust(dtb_ptr: u64) -> ! {
    // No higher-half mapping — physical == virtual at this point.
    let hhdm_offset = 0u64;
    let framebuffer = None;

    // Provisional Pi 5 memory map: skip the lowest 1 MiB (firmware / GPU /
    // kernel image) and offer the rest as Usable. Refined later by parsing
    // the DTB `/memory@0` node.
    static mut MEMORY_REGIONS: [MemoryRegion; 1] = [MemoryRegion {
        base: 0x100000,
        length: 0xFFF0_0000, // ~4 GiB - 1 MiB
        kind: MemoryRegionKind::Usable,
    }];
    let regions = unsafe { &MEMORY_REGIONS[..] };

    let dtb_addr = if dtb_ptr != 0 { Some(dtb_ptr) } else { None };

    let boot_info = BootInfo {
        hhdm_offset,
        framebuffer,
        memory_map: regions,
        dtb_addr,
    };

    folkering_daq_kernel::kernel_main(&boot_info)
}
