# Folkering DAQ Edition

Bare-metal aarch64 operating system for Raspberry Pi 5.  
Drives a **Dewesoft SIRIUSi-HS** instrument via USB and streams data over **openDAQ Native Streaming** to DewesoftX.

> A specialized distro of [Folkering OS](https://github.com/merknu/folkering-os), purpose-built for industrial DAQ.

## What It Does

A Raspberry Pi 5 running Folkering DAQ replaces the need for a full Windows PC with DewesoftX drivers. Plug the SIRIUSi-HS into the Pi's USB port, connect Ethernet, and DewesoftX on any PC on the network auto-discovers it.

```
┌──────────────┐   USB   ┌─────────────────────────────────┐  Ethernet  ┌────────────┐
│ SIRIUSi-HS   │────────►│ Raspberry Pi 5                  │───────────►│ DewesoftX  │
│ 8ch, 20kHz   │         │ Folkering DAQ (bare-metal)      │   mDNS     │ (any PC)   │
│ VID:1CED     │         │ Boot: ~2 sec, RAM: ~20 MB       │   :7420    │            │
└──────────────┘         └─────────────────────────────────┘            └────────────┘
```

## Deploy to Raspberry Pi 5

### 1. Build the kernel

```bash
# Install Rust nightly with aarch64 target
rustup toolchain install nightly-2026-01-20 \
    --component rust-src llvm-tools-preview \
    --target aarch64-unknown-none

# Build for Pi 5
cargo build --release --features pi5 --no-default-features
```

### 2. Prepare the SD card

Format a microSD card with a FAT32 boot partition, then copy:

```
SD Card (FAT32)
├── config.txt          # Pi firmware config
├── kernel8.img         # ← Our kernel (renamed from ELF)
├── bcm2712-rpi-5-b.dtb # Device tree (from Pi firmware)
├── fixup4.dat          # Pi firmware files
└── start4.elf          # Pi firmware files
```

Firmware files from [raspberrypi/firmware](https://github.com/raspberrypi/firmware/tree/master/boot).

**config.txt:**
```ini
arm_64bit=1
kernel=kernel8.img
enable_uart=1
uart_2ndstage=1
dtoverlay=disable-bt
```

### 3. Connect and boot

1. Insert SD card into Pi 5
2. Connect SIRIUSi-HS to USB 3.0 port
3. Connect Ethernet cable to your LAN
4. Power on — boot takes ~2 seconds
5. Open DewesoftX → device appears automatically via mDNS

### Serial debug console

Connect a USB-to-serial adapter to the Pi 5 debug header (between HDMI ports):
```
Pin 1: GND
Pin 2: TX (from Pi)  → connect to RX on adapter
Pin 3: RX (to Pi)    → connect to TX on adapter

Baud: 115200, 8N1
```

## Development (QEMU)

For testing without Pi hardware:

```bash
# Build (QEMU is the default target)
cargo build --release

# Run
qemu-system-aarch64 -M virt -cpu cortex-a76 -m 512M -nographic \
    -kernel target/aarch64-unknown-none/release/kernel
```

```
=== Folkering DAQ Edition ===
Platform: QEMU virt (aarch64)
[OK] GIC-400 (GICv2) interrupt controller
[OK] ARM Generic Timer
[OK] Heap allocator
  openDAQ: 9 streams initialized (v3.20.6 compat)
*** Folkering DAQ ready ***
[5009ms] alive
```

`Ctrl-A X` to exit.

## Project Structure

```
kernel/src/
├── platform.rs             # Pi 5 vs QEMU constants
├── arch/aarch64/
│   ├── exceptions.rs       # Exception vector table
│   ├── gic.rs              # GIC-400 interrupt controller
│   ├── timer.rs            # ARM Generic Timer
│   └── uart.rs             # PL011 UART (debug console)
├── drivers/
│   ├── pci.rs              # PCIe → RP1 south bridge discovery
│   ├── xhci.rs             # USB 3.0 host controller
│   └── gem.rs              # Cadence GEM Gigabit Ethernet
├── usb/
│   └── sirius.rs           # SIRIUSi-HS protocol (reverse-engineered)
├── net/
│   ├── mod.rs              # smoltcp TCP/IP stack
│   ├── mdns.rs             # mDNS service discovery
│   └── websocket.rs        # RFC 6455 (for browser clients)
└── daq/
    ├── opendaq.rs           # Native Streaming binary protocol
    └── signal.rs            # 8 ADC channels + time domain
```

## BCM2712 / RP1 Hardware Map

| Component | Address | Notes |
|---|---|---|
| UART0 (debug) | `0x10_7D00_1000` | On SoC, works without RP1 |
| GIC-400 | `0x10_7FFF_9000` | GICv2, MMIO-based |
| PCIe ECAM | `0x10_0012_0000` | RP1 is VID:0x1DE4 PID:0x0001 |
| RP1 Ethernet | BAR1 + `0x10_0000` | Cadence GEM, RGMII to BCM54213PE |
| RP1 xHCI 0 | BAR1 + `0x20_0000` | USB 3.0 ports |

## openDAQ Protocol

DewesoftX compatible — locked to **protocol v3.20.6**.

**mDNS:** `_opendaq-nd._tcp.local`, TXT: `caps=OPENDAQ`, `version=3.20.6`

**Binary packets (port 7420, raw TCP):**
```
[0xDA51 sync][type:u8][flags:u8][stream_id:u32 LE][size:u32 LE][samples...]
```

## Based On

- [PQTech-openDAQ](https://github.com/sverrekm/PQTech-openDAQ) — reverse-engineered SIRIUS USB protocol
- [openDAQ SDK](https://github.com/openDAQ/openDAQ) — protocol reference
- [Folkering OS](https://github.com/merknu/folkering-os) — kernel architecture

## License

Copyright (c) 2026 PQTech. All rights reserved. See [LICENSE](LICENSE).
