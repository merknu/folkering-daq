//! PCIe enumeration for Raspberry Pi 5
//!
//! BCM2712 has a PCIe 2.0 x4 root complex connecting to the RP1 south bridge.
//! RP1 contains: xHCI USB, Ethernet MAC, GPIO, SPI, I2C, UART, etc.
//!
//! PCIe ECAM (Enhanced Configuration Access Mechanism) base from DTB.
//! Default on Pi 5: 0x1_0000_0000 (4 GiB mark)

use alloc::vec::Vec;
use core::sync::atomic::{AtomicU64, Ordering};

static ECAM_BASE: AtomicU64 = AtomicU64::new(0);

/// PCI device info
#[derive(Debug, Clone)]
pub struct PciDevice {
    pub bus: u8,
    pub device: u8,
    pub function: u8,
    pub vendor_id: u16,
    pub device_id: u16,
    pub class_code: u8,
    pub subclass: u8,
    pub prog_if: u8,
    pub bars: [u64; 6],
    pub irq_line: u8,
}

/// All discovered PCI devices
static mut DEVICES: Vec<PciDevice> = Vec::new();

/// ECAM config space address for a BDF
fn ecam_addr(bus: u8, dev: u8, func: u8, offset: u16) -> u64 {
    let base = ECAM_BASE.load(Ordering::Relaxed);
    base + ((bus as u64) << 20)
        | ((dev as u64) << 15)
        | ((func as u64) << 12)
        | (offset as u64 & 0xFFF)
}

fn read_config32(bus: u8, dev: u8, func: u8, offset: u16) -> u32 {
    let addr = ecam_addr(bus, dev, func, offset);
    unsafe { crate::arch::aarch64::mmio_read32(addr) }
}

fn write_config32(bus: u8, dev: u8, func: u8, offset: u16, val: u32) {
    let addr = ecam_addr(bus, dev, func, offset);
    unsafe { crate::arch::aarch64::mmio_write32(addr, val) }
}

pub fn init() {
    // Set ECAM base — will come from DTB eventually
    #[cfg(not(feature = "pi5"))]
    let ecam_phys = 0x3f00_0000_u64; // QEMU virt PCIe ECAM

    #[cfg(feature = "pi5")]
    let ecam_phys = 0x1_0000_0000_u64; // BCM2712 PCIe ECAM

    ECAM_BASE.store(crate::phys_to_virt(ecam_phys), Ordering::SeqCst);

    crate::kprintln!("  PCIe ECAM at phys {:#x}", ecam_phys);

    // Scan bus 0
    for dev in 0..32u8 {
        let id = read_config32(0, dev, 0, 0x00);
        let vendor_id = (id & 0xFFFF) as u16;
        if vendor_id == 0xFFFF { continue; }

        let device_id = (id >> 16) as u16;
        let class_reg = read_config32(0, dev, 0, 0x08);
        let class_code = (class_reg >> 24) as u8;
        let subclass = (class_reg >> 16) as u8;
        let prog_if = (class_reg >> 8) as u8;
        let irq = (read_config32(0, dev, 0, 0x3C) & 0xFF) as u8;

        // Read BARs
        let mut bars = [0u64; 6];
        for i in 0..6 {
            let bar_offset = 0x10 + (i as u16) * 4;
            let bar = read_config32(0, dev, 0, bar_offset);
            bars[i] = bar as u64;

            // 64-bit BAR: combine with next
            if bar & 0x4 != 0 && i < 5 {
                let bar_hi = read_config32(0, dev, 0, bar_offset + 4);
                bars[i] |= (bar_hi as u64) << 32;
            }
        }

        let device = PciDevice {
            bus: 0, device: dev, function: 0,
            vendor_id, device_id, class_code, subclass, prog_if,
            bars, irq_line: irq,
        };

        crate::kprintln!("  PCI {:02x}:{:02x}.0 [{:04x}:{:04x}] class {:02x}:{:02x}",
            0, dev, vendor_id, device_id, class_code, subclass);

        unsafe { DEVICES.push(device); }
    }
}

/// Find a PCI device by class/subclass
pub fn find_by_class(class: u8, subclass: u8) -> Option<&'static PciDevice> {
    unsafe {
        DEVICES.iter().find(|d| d.class_code == class && d.subclass == subclass)
    }
}

/// Find a PCI device by vendor/device ID
pub fn find_by_id(vendor: u16, device: u16) -> Option<&'static PciDevice> {
    unsafe {
        DEVICES.iter().find(|d| d.vendor_id == vendor && d.device_id == device)
    }
}
