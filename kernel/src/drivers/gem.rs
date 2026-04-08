//! Cadence GEM (GXL 1p09) Gigabit Ethernet MAC Driver for RP1
//!
//! Located at RP1 BAR1 + 0x10_0000 (0x1F_0010_0000 physical).
//! Supports 10/100/1000 Mbps via RGMII to BCM54213PE PHY.
//!
//! Uses 64-bit extended DMA descriptors (16 bytes each).
//! PHY reset via RP1 GPIO 32 (active low).

use core::sync::atomic::{compiler_fence, Ordering};
use core::ptr::{read_volatile, write_volatile};

// === GEM Register Offsets (from BAR1 + 0x10_0000) ===
const NCR: u64 = 0x0000;             // Network Control Register
const NCFGR: u64 = 0x0004;           // Network Configuration Register
const NSR: u64 = 0x0008;             // Network Status Register
const DMACFG: u64 = 0x0010;          // DMA Configuration
const RXBQB: u64 = 0x0018;           // RX Buffer Queue Base (lower 32)
const TXBQB: u64 = 0x001C;           // TX Buffer Queue Base (lower 32)
const ISR: u64 = 0x0024;             // Interrupt Status Register
const IER: u64 = 0x0028;             // Interrupt Enable Register
const IDR: u64 = 0x002C;             // Interrupt Disable Register
const MAN: u64 = 0x0034;             // PHY Maintenance (MDIO)
const SA1B: u64 = 0x0088;            // Specific Address 1 Bottom (MAC[0:3])
const SA1T: u64 = 0x008C;            // Specific Address 1 Top (MAC[4:5])
const UPPER_TX_Q_BASE: u64 = 0x04C8; // Upper 32 bits for TX queues (global)
const UPPER_RX_Q_BASE: u64 = 0x04D4; // Upper 32 bits for RX queues (global)

// NCR bits
const NCR_TX_EN: u32 = 1 << 3;
const NCR_RX_EN: u32 = 1 << 2;
const NCR_MDIO_EN: u32 = 1 << 4;

// NCFGR bits
const NCFGR_FD: u32 = 1 << 1;        // Full Duplex
const NCFGR_GBE: u32 = 1 << 10;      // Gigabit Mode Enable
const NCFGR_RX_CSUM: u32 = 1 << 24;  // RX Checksum Offload

// DMACFG bits
const DMACFG_ADDR64: u32 = 1 << 30;  // 64-bit address bus
const DMACFG_RX_EXT: u32 = 1 << 28;  // RX Extended Buffer Descriptor mode
const DMACFG_TX_EXT: u32 = 1 << 29;  // TX Extended Buffer Descriptor mode
const DMACFG_RX_BUF_SIZE_SHIFT: u32 = 16; // RX buffer size (in 64-byte units)
const DMACFG_TX_PBUF_SIZE: u32 = 1 << 10; // TX partial store and forward

// MDIO bits
const MAN_SOF: u32 = 0x01 << 30;     // Start of Frame (01)
const MAN_READ: u32 = 0x02 << 28;    // Read operation (10)
const MAN_WRITE: u32 = 0x01 << 28;   // Write operation (01)
const MAN_MUST10: u32 = 0x02 << 16;  // Must be 10

// PHY address on RP1
const PHY_ADDR: u8 = 0x01;

// === 64-bit Extended Buffer Descriptor (16 bytes) ===
/// Cadence GEM DMA descriptor for 64-bit extended mode.
/// Layout: [addr_low(4)] [ctrl(4)] [addr_high(4)] [unused(4)]
///
/// RX ownership: bit 0 of addr_low (0=MAC owns, 1=CPU owns)
/// TX ownership: bit 31 of ctrl (1=USED/CPU owns, 0=MAC owns for sending)
#[repr(C, align(16))]
#[derive(Clone, Copy)]
pub struct GemDescriptor {
    pub addr_low: u32,   // Lower 32 bits of buffer address. RX: bit0=ownership, bit1=wrap
    pub ctrl: u32,       // RX: len(0:12),SOF(14),EOF(15). TX: len(0:13),LAST(15),WRAP(30),USED(31)
    pub addr_high: u32,  // Upper 32 bits of buffer address
    pub unused: u32,     // Reserved / PTP timestamp
}

impl GemDescriptor {
    pub const fn empty() -> Self {
        Self { addr_low: 0, ctrl: 0, addr_high: 0, unused: 0 }
    }

    // --- RX ---

    /// Check if CPU owns this RX descriptor (bit 0 of addr_low = 1)
    pub fn is_rx_owned_by_cpu(&self) -> bool {
        let addr = unsafe { read_volatile(&self.addr_low) };
        (addr & 1) != 0
    }

    /// Get received packet length from RX ctrl (bits 0:12)
    pub fn rx_length(&self) -> u16 {
        let ctrl = unsafe { read_volatile(&self.ctrl) };
        (ctrl & 0x1FFF) as u16
    }

    /// Get buffer physical address (mask off flag bits)
    pub fn rx_buffer_addr(&self) -> u64 {
        let lo = unsafe { read_volatile(&self.addr_low) } & !0x3; // Clear bit 0,1
        let hi = unsafe { read_volatile(&self.addr_high) };
        ((hi as u64) << 32) | (lo as u64)
    }

    /// Release RX descriptor back to MAC (clear bit 0)
    pub fn release_rx_to_mac(&mut self) {
        compiler_fence(Ordering::Release);
        let mut addr = unsafe { read_volatile(&self.addr_low) };
        addr &= !1; // Clear ownership bit
        unsafe { write_volatile(&mut self.addr_low, addr); }
    }

    /// Set up RX descriptor with buffer address, clear ownership to MAC
    pub fn set_rx_buffer(&mut self, phys_addr: u64, is_wrap: bool) {
        let lo = (phys_addr & 0xFFFF_FFFC) as u32; // Align, clear bit 0,1
        let hi = (phys_addr >> 32) as u32;
        let lo = if is_wrap { lo | 2 } else { lo }; // Bit 1 = wrap
        // Bit 0 = 0 means MAC owns it
        unsafe {
            write_volatile(&mut self.addr_high, hi);
            write_volatile(&mut self.ctrl, 0);
            write_volatile(&mut self.unused, 0);
            compiler_fence(Ordering::Release);
            write_volatile(&mut self.addr_low, lo);
        }
    }

    // --- TX ---

    /// Check if CPU owns this TX descriptor (bit 31 of ctrl = 1 = USED)
    pub fn is_tx_used(&self) -> bool {
        let ctrl = unsafe { read_volatile(&self.ctrl) };
        (ctrl & (1 << 31)) != 0
    }

    /// Set TX descriptor ready for sending. Transfers ownership to MAC.
    pub fn set_tx_ready(&mut self, phys_addr: u64, len: u16, is_last: bool, is_wrap: bool) {
        let lo = (phys_addr & 0xFFFF_FFFF) as u32;
        let hi = (phys_addr >> 32) as u32;

        let mut ctrl = (len as u32) & 0x3FFF;
        if is_last { ctrl |= 1 << 15; }
        if is_wrap { ctrl |= 1 << 30; }
        // bit 31 = 0 → ownership to MAC (ready to send)

        unsafe {
            write_volatile(&mut self.addr_low, lo);
            write_volatile(&mut self.addr_high, hi);
            write_volatile(&mut self.unused, 0);
            compiler_fence(Ordering::Release);
            // This write transfers ownership — must be LAST
            write_volatile(&mut self.ctrl, ctrl);
        }
    }

    /// Mark TX descriptor as USED (CPU owns, MAC ignores)
    pub fn set_tx_used(&mut self, is_wrap: bool) {
        let mut ctrl = 1u32 << 31; // USED
        if is_wrap { ctrl |= 1 << 30; }
        unsafe { write_volatile(&mut self.ctrl, ctrl); }
    }
}

// === Ring Buffers ===
const RX_RING_SIZE: usize = 64;
const TX_RING_SIZE: usize = 64;
const RX_BUF_SIZE: usize = 2048; // Per-buffer size (max Ethernet frame + overhead)

#[repr(C, align(4096))]
struct RxRing {
    descs: [GemDescriptor; RX_RING_SIZE],
}

#[repr(C, align(4096))]
struct TxRing {
    descs: [GemDescriptor; TX_RING_SIZE],
}

// Packet buffers (DMA-accessible memory)
#[repr(C, align(4096))]
struct RxBuffers {
    data: [[u8; RX_BUF_SIZE]; RX_RING_SIZE],
}

#[repr(C, align(4096))]
struct TxBuffers {
    data: [[u8; RX_BUF_SIZE]; TX_RING_SIZE],
}

static mut RX_RING: RxRing = RxRing { descs: [GemDescriptor::empty(); RX_RING_SIZE] };
static mut TX_RING: TxRing = TxRing { descs: [GemDescriptor::empty(); TX_RING_SIZE] };
static mut RX_BUFS: RxBuffers = RxBuffers { data: [[0; RX_BUF_SIZE]; RX_RING_SIZE] };
static mut TX_BUFS: TxBuffers = TxBuffers { data: [[0; RX_BUF_SIZE]; TX_RING_SIZE] };

static mut GEM_BASE: u64 = 0;
static mut RX_IDX: usize = 0;
static mut TX_IDX: usize = 0;

// MAC address (will be read from OTP or set manually)
static mut MAC_ADDR: [u8; 6] = [0x2C, 0xCF, 0x67, 0x00, 0x00, 0x01];

// === MMIO Helpers ===
#[inline]
fn gem_read(offset: u64) -> u32 {
    unsafe { read_volatile((GEM_BASE + offset) as *const u32) }
}

#[inline]
fn gem_write(offset: u64, val: u32) {
    unsafe { write_volatile((GEM_BASE + offset) as *mut u32, val); }
}

// === MDIO (PHY Management) ===

fn mdio_read(phy: u8, reg: u8) -> u16 {
    let cmd = MAN_SOF | MAN_READ | ((phy as u32) << 23) | ((reg as u32) << 18) | MAN_MUST10;
    gem_write(MAN, cmd);

    // Wait for MDIO idle (NSR bit 2)
    while gem_read(NSR) & (1 << 2) == 0 {
        core::hint::spin_loop();
    }

    (gem_read(MAN) & 0xFFFF) as u16
}

fn mdio_write(phy: u8, reg: u8, val: u16) {
    let cmd = MAN_SOF | MAN_WRITE | ((phy as u32) << 23) | ((reg as u32) << 18) | MAN_MUST10 | (val as u32);
    gem_write(MAN, cmd);

    while gem_read(NSR) & (1 << 2) == 0 {
        core::hint::spin_loop();
    }
}

// === PHY Reset via GPIO 32 ===

fn phy_reset() {
    // GPIO 32 is the PHY reset pin (active low)
    // RP1 GPIO registers are at BAR1 + GPIO_OFFSET
    // For now, use a software PHY reset via MDIO register 0 bit 15
    crate::kprintln!("  GEM: PHY software reset via MDIO");
    mdio_write(PHY_ADDR, 0, 1 << 15); // BMCR: Software Reset
    crate::arch::aarch64::timer::delay_ms(10);

    // Wait for reset to complete (bit 15 auto-clears)
    let start = crate::arch::aarch64::timer::micros();
    loop {
        let bmcr = mdio_read(PHY_ADDR, 0);
        if bmcr & (1 << 15) == 0 { break; }
        if crate::arch::aarch64::timer::micros() - start > 1_000_000 {
            crate::kprintln!("  GEM: PHY reset timeout");
            break;
        }
    }
}

fn phy_auto_negotiate() {
    // Enable auto-negotiation and restart
    let bmcr = mdio_read(PHY_ADDR, 0);
    mdio_write(PHY_ADDR, 0, bmcr | (1 << 12) | (1 << 9)); // AN Enable + Restart AN

    crate::kprintln!("  GEM: PHY auto-negotiation started...");

    // Wait for link up (BMSR register 1, bit 2 = Link Status)
    let start = crate::arch::aarch64::timer::micros();
    loop {
        let bmsr = mdio_read(PHY_ADDR, 1);
        if bmsr & (1 << 2) != 0 {
            // Link is up — check speed
            let bmsr2 = mdio_read(PHY_ADDR, 1); // Read twice per spec
            let anlpar = mdio_read(PHY_ADDR, 5); // Link Partner Ability
            let stat1000 = mdio_read(PHY_ADDR, 10); // 1000BASE-T Status

            let gig = (stat1000 & (1 << 11)) != 0; // Partner 1000FD
            let speed = if gig { "1000 Mbps" } else if anlpar & (1 << 8) != 0 { "100 Mbps" } else { "10 Mbps" };
            let duplex = if gig || (anlpar & (1 << 6) != 0) { "Full" } else { "Half" };

            crate::kprintln!("  GEM: link up — {} {}-Duplex", speed, duplex);
            return;
        }
        if crate::arch::aarch64::timer::micros() - start > 5_000_000 {
            crate::kprintln!("  GEM: link timeout (no cable?)");
            return;
        }
        crate::arch::aarch64::timer::delay_ms(100);
    }
}

// === DMA Ring Initialization ===

fn init_rx_ring() {
    unsafe {
        for i in 0..RX_RING_SIZE {
            let buf_phys = crate::virt_to_phys(&RX_BUFS.data[i][0] as *const u8 as u64);
            let is_wrap = i == RX_RING_SIZE - 1;
            RX_RING.descs[i].set_rx_buffer(buf_phys, is_wrap);
        }
        RX_IDX = 0;
    }
}

fn init_tx_ring() {
    unsafe {
        for i in 0..TX_RING_SIZE {
            let is_wrap = i == TX_RING_SIZE - 1;
            TX_RING.descs[i].set_tx_used(is_wrap); // CPU owns initially
        }
        TX_IDX = 0;
    }
}

// === Public API ===

/// Initialize Cadence GEM Ethernet MAC
pub fn init(gem_base: u64) {
    unsafe { GEM_BASE = gem_base; }

    crate::kprintln!("  GEM: Cadence Ethernet MAC at {:#x}", gem_base);

    // Disable TX/RX before configuration
    gem_write(NCR, 0);

    // Enable MDIO
    gem_write(NCR, NCR_MDIO_EN);

    // PHY init
    phy_reset();
    phy_auto_negotiate();

    // Configure NCFGR: Gigabit + Full Duplex
    gem_write(NCFGR, NCFGR_GBE | NCFGR_FD);

    // Configure DMA: 64-bit addressing + extended descriptors
    let dmacfg = DMACFG_ADDR64 | DMACFG_RX_EXT | DMACFG_TX_EXT
        | ((RX_BUF_SIZE as u32 / 64) << DMACFG_RX_BUF_SIZE_SHIFT)
        | DMACFG_TX_PBUF_SIZE;
    gem_write(DMACFG, dmacfg);

    // Initialize DMA rings
    init_rx_ring();
    init_tx_ring();

    unsafe {
        // Program ring base addresses (64-bit)
        let rx_ring_phys = crate::virt_to_phys(&RX_RING.descs[0] as *const _ as u64);
        let tx_ring_phys = crate::virt_to_phys(&TX_RING.descs[0] as *const _ as u64);

        // Upper 32 bits (global for all queues)
        gem_write(UPPER_RX_Q_BASE, (rx_ring_phys >> 32) as u32);
        gem_write(UPPER_TX_Q_BASE, (tx_ring_phys >> 32) as u32);

        // Lower 32 bits
        gem_write(RXBQB, rx_ring_phys as u32);
        gem_write(TXBQB, tx_ring_phys as u32);

        // Program MAC address
        let mac = MAC_ADDR;
        let sa1b = (mac[0] as u32) | ((mac[1] as u32) << 8) | ((mac[2] as u32) << 16) | ((mac[3] as u32) << 24);
        let sa1t = (mac[4] as u32) | ((mac[5] as u32) << 8);
        gem_write(SA1B, sa1b);
        gem_write(SA1T, sa1t);
    }

    // Disable all interrupts (we poll)
    gem_write(IDR, 0xFFFF_FFFF);

    // Enable TX + RX + MDIO
    gem_write(NCR, NCR_TX_EN | NCR_RX_EN | NCR_MDIO_EN);

    crate::kprintln!("  GEM: TX/RX enabled, DMA 64-bit mode");
}

/// Receive a packet. Returns slice of received data, or None.
pub fn receive<'a>() -> Option<(&'a [u8], usize)> {
    unsafe {
        let desc = &RX_RING.descs[RX_IDX];
        if !desc.is_rx_owned_by_cpu() {
            return None;
        }

        let len = desc.rx_length() as usize;
        if len == 0 || len > RX_BUF_SIZE {
            // Invalid — release and skip
            RX_RING.descs[RX_IDX].release_rx_to_mac();
            RX_IDX = (RX_IDX + 1) % RX_RING_SIZE;
            return None;
        }

        let data = &RX_BUFS.data[RX_IDX][..len];
        let idx = RX_IDX;

        Some((data, idx))
    }
}

/// Release RX buffer back to MAC after processing
pub fn release_rx(idx: usize) {
    unsafe {
        RX_RING.descs[idx].release_rx_to_mac();
        RX_IDX = (RX_IDX + 1) % RX_RING_SIZE;
    }
}

/// Transmit a packet. Copies data to TX buffer and triggers DMA.
pub fn transmit(data: &[u8]) -> Result<(), &'static str> {
    if data.len() > RX_BUF_SIZE {
        return Err("packet too large");
    }

    unsafe {
        let desc = &TX_RING.descs[TX_IDX];
        if !desc.is_tx_used() {
            return Err("TX ring full");
        }

        // Copy packet data to TX buffer
        TX_BUFS.data[TX_IDX][..data.len()].copy_from_slice(data);

        let buf_phys = crate::virt_to_phys(&TX_BUFS.data[TX_IDX][0] as *const u8 as u64);
        let is_wrap = TX_IDX == TX_RING_SIZE - 1;

        // Set descriptor ready — transfers ownership to MAC
        TX_RING.descs[TX_IDX].set_tx_ready(buf_phys, data.len() as u16, true, is_wrap);

        TX_IDX = (TX_IDX + 1) % TX_RING_SIZE;

        // Trigger TX start (write to NCR)
        let ncr = gem_read(NCR);
        gem_write(NCR, ncr | (1 << 9)); // TX Start
    }

    Ok(())
}

/// Get MAC address
pub fn mac_address() -> [u8; 6] {
    unsafe { MAC_ADDR }
}
