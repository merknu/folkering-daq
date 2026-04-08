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

#[no_mangle]
extern "C" fn kmain() -> ! {
    // Verify Limine base revision is supported
    assert!(BASE_REVISION.is_supported());

    // Extract HHDM offset
    let hhdm_offset = HHDM_REQUEST.get_response()
        .expect("HHDM response missing")
        .offset();

    // Extract framebuffer
    let framebuffer = FRAMEBUFFER_REQUEST.get_response().and_then(|resp| {
        resp.framebuffers().next().map(|fb| {
            FramebufferInfo {
                addr: fb.addr() as u64,
                width: fb.width() as u32,
                height: fb.height() as u32,
                pitch: fb.pitch() as u32,
                bpp: fb.bpp(),
            }
        })
    });

    // Extract memory map
    let memory_map = MEMORY_MAP_REQUEST.get_response()
        .expect("Memory map response missing");

    // Convert Limine memory map to our format
    // Safety: we store in a static buffer that lives forever
    static mut MEMORY_REGIONS: [MemoryRegion; 128] = [MemoryRegion {
        base: 0, length: 0, kind: MemoryRegionKind::Reserved,
    }; 128];

    let entries = memory_map.entries();
    let count = entries.len().min(128);

    let regions = unsafe {
        for (i, entry) in entries.iter().take(count).enumerate() {
            MEMORY_REGIONS[i] = MemoryRegion {
                base: entry.base,
                length: entry.length,
                kind: match entry.entry_type {
                    limine::memory_map::EntryType::USABLE => MemoryRegionKind::Usable,
                    limine::memory_map::EntryType::ACPI_RECLAIMABLE => MemoryRegionKind::AcpiReclaimable,
                    limine::memory_map::EntryType::BOOTLOADER_RECLAIMABLE => MemoryRegionKind::BootloaderReclaimable,
                    limine::memory_map::EntryType::KERNEL_AND_MODULES => MemoryRegionKind::KernelAndModules,
                    limine::memory_map::EntryType::FRAMEBUFFER => MemoryRegionKind::Framebuffer,
                    _ => MemoryRegionKind::Reserved,
                },
            };
        }
        &MEMORY_REGIONS[..count]
    };

    // Extract DTB address (Pi firmware provides device tree)
    let dtb_addr = DTB_REQUEST.get_response().map(|resp| resp.dtb_ptr() as u64);

    // Build boot info and hand off to kernel
    let boot_info = BootInfo {
        hhdm_offset,
        framebuffer,
        memory_map: regions,
        dtb_addr,
    };

    folkering_daq_kernel::kernel_main(&boot_info)
}
