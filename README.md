# Folkering DAQ Edition

Bare-metal aarch64 operating system for data acquisition on Raspberry Pi 5.  
Drives a **Dewesoft SIRIUSi-HS** instrument via USB and streams data over **openDAQ Native Streaming** to DewesoftX.

> A specialized distro of [Folkering OS](https://github.com/merknu/folkering-os), purpose-built for industrial DAQ.

## Architecture

```
Raspberry Pi 5 (BCM2712)
├── Folkering DAQ kernel (bare-metal, 45 KB)
│   ├── xHCI USB ──► SIRIUSi-HS (8ch × 20kHz × int16)
│   ├── Cadence GEM Ethernet ──► smoltcp TCP/IP
│   ├── openDAQ server (daq.nd://:7420)
│   └── mDNS discovery (_opendaq-nd._tcp.local)
│
├── USB ──► Dewesoft SIRIUSi-HS
└── Ethernet ──► DewesoftX (auto-discovers via mDNS)
```

## Quick Start

### Run in QEMU (development)

```bash
# Prerequisites: Rust nightly + QEMU
rustup toolchain install nightly-2026-01-20 --component rust-src llvm-tools-preview --target aarch64-unknown-none

# Build and run
cargo build --release
qemu-system-aarch64 -M virt -cpu cortex-a76 -m 512M -nographic \
    -kernel target/aarch64-unknown-none/release/kernel
```

Expected output:
```
=== Folkering DAQ Edition ===
Platform: QEMU virt (aarch64)
[OK] Exception vectors installed
[OK] GIC-400 (GICv2) interrupt controller
[OK] ARM Generic Timer
[OK] Heap allocator
  openDAQ: 9 streams initialized (v3.20.6 compat)
  openDAQ: Native Streaming on :7420 (daq.nd://)
*** Folkering DAQ ready ***
[5009ms] alive
[10009ms] alive
```

Press `Ctrl-A X` to exit QEMU.

### Run in Docker

```bash
docker compose up
```

### Deploy to Proxmox

```bash
./tools/deploy-proxmox.sh <proxmox-host>
```

### Build for Raspberry Pi 5

```bash
cargo build --release --features pi5 --no-default-features
# Output: target/aarch64-unknown-none/release/kernel
```

## Build Targets

| Feature | Platform | Use |
|---|---|---|
| `qemu-virt` (default) | QEMU aarch64 virt | Development & testing |
| `pi5` | Raspberry Pi 5 | Production with real hardware |

## Project Structure

```
kernel/src/
├── main.rs                 # Limine (Pi5) / direct boot (QEMU) entry
├── lib.rs                  # Boot sequence + main loop
├── platform.rs             # Per-platform constants (UART, GIC, PCIe)
├── arch/aarch64/
│   ├── exceptions.rs       # Exception vector table (global_asm!)
│   ├── gic.rs              # GIC-400 (GICv2) interrupt controller
│   ├── timer.rs            # ARM Generic Timer
│   └── uart.rs             # PL011 UART serial console
├── drivers/
│   ├── pci.rs              # PCIe ECAM + RP1 south bridge discovery
│   ├── xhci.rs             # xHCI USB 3.0 (ring-based DMA, sync completion)
│   └── gem.rs              # Cadence GEM Gigabit Ethernet MAC (64-bit DMA)
├── usb/
│   └── sirius.rs           # SIRIUSi-HS reverse-engineered USB protocol
├── net/
│   ├── mod.rs              # smoltcp integration (phy::Device for GEM)
│   ├── mdns.rs             # mDNS/DNS-SD responder
│   └── websocket.rs        # RFC 6455 (SHA-1, Base64, framing)
└── daq/
    ├── opendaq.rs           # Native Streaming server (0xDA51 binary packets)
    └── signal.rs            # Signal descriptors (8ch + time domain)
```

## Hardware Addresses (BCM2712 / RP1)

| Component | Physical Address |
|---|---|
| UART0 (PL011) | `0x10_7D00_1000` |
| GIC-400 GICD | `0x10_7FFF_9000` |
| GIC-400 GICC | `0x10_7FFF_A000` |
| PCIe ECAM | `0x10_0012_0000` |
| RP1 BAR1 | `0x1F_0000_0000` |
| RP1 Ethernet (GEM) | BAR1 + `0x10_0000` |
| RP1 xHCI 0 | BAR1 + `0x20_0000` |
| RP1 xHCI 1 | BAR1 + `0x30_0000` |

## openDAQ Protocol (DewesoftX Compatible)

The server implements **openDAQ Native Streaming v3.20.6** over raw TCP on port 7420.

> **Warning:** DewesoftX rejects protocol versions above 3.20.6 silently.

**mDNS discovery:**
- Service: `_opendaq-nd._tcp.local`
- TXT: `caps=OPENDAQ`, `version=3.20.6`, `model=PQTech`

**Binary DataPacket (12-byte header, Little-Endian):**
```
Offset  Field          Type    Description
0x00    Sync Word      u16     0xDA51
0x02    Packet Type    u8      0x01 = DataPacket
0x03    Flags          u8      0x00 = raw uncompressed
0x04    Stream ID      u32     djb2 hash of publicId
0x08    Payload Size   u32     byte count of sample data
0x0C    Payload        [u8]    raw int16 LE samples
```

## SIRIUS USB Protocol

Reverse-engineered from Wireshark USB captures ([PQTech-openDAQ](https://github.com/sverrekm/PQTech-openDAQ)).

- **VID:** `0x1CED` (Dewesoft), **PID:** `0x1002`
- **EP1 OUT/IN:** AD/B1 command protocol (15-byte commands, 1-byte poll)
- **EP2 IN:** ADC data — 15872 bytes (992 frames × 8 channels × int16 LE)
- **EP4 IN:** Status (20 bytes) — must drain to prevent FIFO stall
- **EP6 IN:** Sync (15872 bytes) — must read in lockstep with EP2
- **Heartbeat:** B1 at ~150/sec (device disconnects without it)

## License

MIT OR Apache-2.0
