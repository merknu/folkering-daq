//! Platform-specific constants for different targets.
//!
//! QEMU virt: Development/testing on any host machine
//! Pi 5: Raspberry Pi 5 (BCM2712 + RP1 south bridge)

// === UART ===

#[cfg(feature = "qemu-virt")]
pub const UART_PHYS: u64 = 0x0900_0000;

#[cfg(feature = "pi5")]
pub const UART_PHYS: u64 = 0x10_7D00_1000;

// === GIC (Interrupt Controller) ===

// QEMU virt: GICv2 at standard addresses
#[cfg(feature = "qemu-virt")]
pub const GICD_PHYS: u64 = 0x0800_0000;
#[cfg(feature = "qemu-virt")]
pub const GICC_PHYS: u64 = 0x0801_0000;

// Pi 5: GIC-400 (GICv2) at BCM2712 addresses
#[cfg(feature = "pi5")]
pub const GICD_PHYS: u64 = 0x10_7FFF_9000;
#[cfg(feature = "pi5")]
pub const GICC_PHYS: u64 = 0x10_7FFF_A000;

// === PCIe ===

#[cfg(feature = "qemu-virt")]
pub const ECAM_PHYS: u64 = 0x4010_0000; // QEMU virt PCIe ECAM

#[cfg(feature = "pi5")]
pub const ECAM_PHYS: u64 = 0x10_0012_0000; // BCM2712

// === Network ===

/// QEMU: use virtio-net (detected via PCI class 02:00)
/// Pi 5: use Cadence GEM at RP1 BAR1 + 0x10_0000
#[cfg(feature = "qemu-virt")]
pub const USE_VIRTIO_NET: bool = true;

#[cfg(feature = "pi5")]
pub const USE_VIRTIO_NET: bool = false;

// === Platform Name ===

#[cfg(feature = "qemu-virt")]
pub const PLATFORM_NAME: &str = "QEMU virt (aarch64)";

#[cfg(feature = "pi5")]
pub const PLATFORM_NAME: &str = "Raspberry Pi 5 (BCM2712)";
