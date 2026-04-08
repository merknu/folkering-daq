//! PCIe enumeration for BCM2712
//!
//! BCM2712 has a PCIe Gen 2.0 x4 root complex connecting to the RP1 south bridge.
//! RP1 is a SINGLE-FUNCTION PCIe endpoint — all internal devices (xHCI, Ethernet)
//! are behind BAR1 at fixed AXI offsets, NOT as separate PCIe functions.
//!
//! PCIe ECAM base: 0x10_0012_0000 (BCM2712)
//! RP1 identification: Vendor ID 0x1DE4 ("Raspberry Pi Ltd"), Device ID 0x0001

use alloc::vec::Vec;
use core::sync::atomic::{AtomicU64, Ordering};

/// PCIe ECAM physical base (BCM2712)
const ECAM_PHYS: u64 = 0x10_0012_0000;

/// RP1 South Bridge identity
pub const RP1_VENDOR_ID: u16 = 0x1DE4; // Raspberry Pi Ltd
pub const RP1_DEVICE_ID: u16 = 0x0001; // RP1 PCIe 2.0 South Bridge

/// RP1 BAR1 base (firmware-initialized, mapped in AArch64 physical space)
pub const RP1_BAR1_PHYS: u64 = 0x1F_0000_0000;

/// RP1 internal AXI offsets from BAR1
pub const RP1_ETH_OFFSET: u64 = 0x10_0000;   // Cadence GEM Ethernet MAC
pub const RP1_XHCI0_OFFSET: u64 = 0x20_0000; // xHCI USB Controller 0
pub const RP1_XHCI1_OFFSET: u64 = 0x30_0000; // xHCI USB Controller 1

static ECAM_BASE: AtomicU64 = AtomicU64::new(0);

/// RP1 discovery result
pub struct Rp1Info {
    pub bus: u8,
    pub device: u8,
    pub function: u8,
    pub bar1_phys: u64,
    /// Virtual addresses for RP1 sub-devices (via HHDM)
    pub eth_base: u64,
    pub xhci0_base: u64,
    pub xhci1_base: u64,
}

static mut RP1: Option<Rp1Info> = None;

/// ECAM config space address for Bus:Device:Function + register offset
fn ecam_addr(bus: u8, dev: u8, func: u8, offset: u16) -> u64 {
    let base = ECAM_BASE.load(Ordering::Relaxed);
    base + ((bus as u64) << 20)
         | ((dev as u64) << 15)
         | ((func as u64) << 12)
         | (offset as u64 & 0xFFF)
}

fn read_config32(bus: u8, dev: u8, func: u8, offset: u16) -> u32 {
    let addr = ecam_addr(bus, dev, func, offset);
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

fn write_config32(bus: u8, dev: u8, func: u8, offset: u16, val: u32) {
    let addr = ecam_addr(bus, dev, func, offset);
    unsafe { core::ptr::write_volatile(addr as *mut u32, val); }
}

pub fn init() {
    ECAM_BASE.store(crate::phys_to_virt(ECAM_PHYS), Ordering::SeqCst);
    crate::kprintln!("  PCIe ECAM at phys {:#x}", ECAM_PHYS);

    // Scan bus 0 for RP1
    for bus in 0..2u8 {
        for dev in 0..32u8 {
            let id = read_config32(bus, dev, 0, 0x00);
            let vendor_id = (id & 0xFFFF) as u16;
            if vendor_id == 0xFFFF || vendor_id == 0x0000 { continue; }

            let device_id = (id >> 16) as u16;
            let class_reg = read_config32(bus, dev, 0, 0x08);
            let class_code = (class_reg >> 24) as u8;
            let subclass = (class_reg >> 16) as u8;

            crate::kprintln!("  PCI {:02x}:{:02x}.0 [{:04x}:{:04x}] class {:02x}:{:02x}",
                bus, dev, vendor_id, device_id, class_code, subclass);

            // Check if this is RP1
            if vendor_id == RP1_VENDOR_ID && device_id == RP1_DEVICE_ID {
                crate::kprintln!("  >>> RP1 South Bridge found!");

                // Read BAR1 (64-bit)
                let bar1_lo = read_config32(bus, dev, 0, 0x14); // BAR1 at offset 0x14
                let bar1_hi = read_config32(bus, dev, 0, 0x18); // BAR1 upper 32 bits
                let bar1_phys = ((bar1_hi as u64) << 32) | ((bar1_lo & !0xF) as u64);

                // Use firmware-assigned BAR1 or default
                let bar1 = if bar1_phys != 0 { bar1_phys } else { RP1_BAR1_PHYS };

                crate::kprintln!("  RP1 BAR1 = {:#x}", bar1);

                // Map RP1 sub-devices via HHDM
                let rp1 = Rp1Info {
                    bus, device: dev, function: 0,
                    bar1_phys: bar1,
                    eth_base: crate::phys_to_virt(bar1 + RP1_ETH_OFFSET),
                    xhci0_base: crate::phys_to_virt(bar1 + RP1_XHCI0_OFFSET),
                    xhci1_base: crate::phys_to_virt(bar1 + RP1_XHCI1_OFFSET),
                };

                crate::kprintln!("  RP1 Ethernet MAC: phys {:#x}", bar1 + RP1_ETH_OFFSET);
                crate::kprintln!("  RP1 xHCI 0:       phys {:#x}", bar1 + RP1_XHCI0_OFFSET);
                crate::kprintln!("  RP1 xHCI 1:       phys {:#x}", bar1 + RP1_XHCI1_OFFSET);

                unsafe { RP1 = Some(rp1); }

                // Enable bus mastering and memory space for RP1
                let cmd = read_config32(bus, dev, 0, 0x04);
                write_config32(bus, dev, 0, 0x04, cmd | 0x06); // Memory Space + Bus Master

                return;
            }
        }
    }

    crate::kprintln!("  WARNING: RP1 not found on PCIe bus!");
}

/// Get RP1 info (None if not found)
pub fn rp1() -> Option<&'static Rp1Info> {
    unsafe { RP1.as_ref() }
}
