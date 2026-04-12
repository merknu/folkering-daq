# Folkering DAQ Edition

Bare-metal aarch64 operating system for Raspberry Pi 5.  
Drives a **Dewesoft SIRIUSi-HS** instrument via USB, streams data over **openDAQ Native Streaming**, and runs **WASM DSP apps** for real-time signal processing.

> A specialized distro of [Folkering OS](https://github.com/merknu/folkering-os), purpose-built for industrial DAQ.

## What It Does

A Raspberry Pi 5 running Folkering DAQ replaces the need for a full Windows PC with DewesoftX drivers. Plug the SIRIUSi-HS into the Pi's USB port, connect Ethernet, and DewesoftX on any PC on the network auto-discovers it.

WASM DSP apps run on the sensor data in real-time and can be hot-swapped over the network without rebooting.

```
                                     ┌──────────────────────┐
                                     │ WASM DSP Apps        │
┌──────────────┐   USB   ┌──────────┤ lowpass / bandpass /  │
│ SIRIUSi-HS   │────────►│ Ring     │ peak_detect / FFT     │
│ 8ch, 20kHz   │         │ Buffer   │ (hot-swap via :7421)  │
│ VID:1CED     │         ├──────────┼──────────────────────►├──► Processed output
└──────────────┘         │ Pi 5     │ wasmi interpreter     │
                         │ Folkering├──────────────────────┘
┌──────────────┐  I2C/SPI│ DAQ OS   │
│ Sensors      │────────►│          │──── :7420 ──► DewesoftX (openDAQ)
│ BME280, ADC  │         │          │──── :2222 ──► TCP shell (nc)
└──────────────┘         │          │──── :5353 ──► mDNS discovery
                         └──────────┘
```

## Architecture

### Data Pipeline

```
USB ISR → i16 ADC → SPSC Ring Buffer (f32, zero-copy) → wasmi WASM tick()
                         ↓                                      ↓
                    TCP :7420 (DewesoftX)              Output Ring Buffer
                                                            ↓
                                                     TCP retransmission
```

### Network Services

| Port | Protocol | Purpose |
|------|----------|---------|
| 7420 | TCP (raw binary) | openDAQ Native Streaming to DewesoftX |
| 7421 | TCP (length-prefixed) | WASM hot-swap (upload new DSP modules) |
| 2222 | TCP (plaintext) | Remote shell (`help`, `uptime`, `daq status`) |
| 5353 | UDP (mDNS) | `_opendaq-nd._tcp.local` service discovery |

### Driver Coverage

| Peripheral | Driver | Status |
|-----------|--------|--------|
| PL011 UART (serial console) | `arch/aarch64/uart.rs` | Working |
| GICv2 interrupt controller | `arch/aarch64/gic.rs` | Working |
| ARM Generic Timer | `arch/aarch64/timer.rs` | Working |
| Exception vectors | `arch/aarch64/exceptions.rs` | Working |
| PCIe / RP1 discovery | `drivers/pci.rs` | Working |
| Cadence GEM Gigabit Ethernet | `drivers/gem.rs` | Working |
| xHCI USB 3.0 | `drivers/xhci.rs` | Working |
| SIRIUSi-HS DAQ instrument | `usb/sirius.rs` | Working |
| **RP1 GPIO (28 pins)** | `drivers/rp1_gpio.rs` | Working |
| **RP1 SPI (DW APB SSI)** | `drivers/rp1_spi.rs` | Working |
| **RP1 I2C (DW I2C)** | `drivers/rp1_i2c.rs` | Working |
| smoltcp TCP/IP stack | `net/mod.rs` | Working |

### WASM DSP App Suite

Pre-built apps in `wasm-apps/` (all < 2 KiB):

| App | Size | Description |
|-----|------|-------------|
| `lowpass_filter.wasm` | 781 B | 1st-order IIR, 1000 Hz cutoff |
| `bandpass_filter.wasm` | 827 B | Cascaded HP+LP, 200-2000 Hz |
| `peak_detector.wasm` | 888 B | Envelope follower + threshold trigger |
| `fft_analyzer.wasm` | 1872 B | 256-point radix-2 FFT with Hanning window |

Deploy a new DSP app without rebooting:
```bash
python tools/deploy_wasm.py lowpass_filter --ip 192.168.1.100
# or manually:
cat wasm-apps/fft_analyzer.wasm | nc 192.168.1.100 7421
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

# Build with embedded DSP app (lowpass filter baked into kernel)
cargo build --release --features "pi5,embedded-dsp" --no-default-features
```

### 2. Prepare the SD card

Format a microSD card with a FAT32 boot partition, then copy:

```
SD Card (FAT32)
├── config.txt          # Pi firmware config
├── kernel8.img         # Our kernel (renamed from ELF)
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
4. Power on (boot takes ~2 seconds)
5. Open DewesoftX: device appears automatically via mDNS

### Serial debug console

Connect a USB-to-serial adapter to the Pi 5 debug header:
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

`Ctrl-A X` to exit.

## Building WASM DSP Apps

```bash
# Build a single app
cd apps/lowpass_filter
cargo build --target wasm32-unknown-unknown --release

# Deploy to running DAQ node
python tools/deploy_wasm.py lowpass_filter --ip <pi-ip>
```

### Writing a Custom DSP App

```rust
#![no_std]

extern "C" {
    fn folk_daq_read_samples(channel: i32, buf_ptr: i32, max: i32) -> i32;
    fn folk_daq_available() -> i32;
    fn folk_daq_sample_rate() -> i32;
    fn folk_daq_channel_count() -> i32;
    fn folk_daq_write_output(channel: i32, buf_ptr: i32, len: i32) -> i32;
    fn folk_log(ptr: i32, len: i32) -> i32;
}

#[no_mangle]
pub extern "C" fn init() -> i32 { /* one-time setup */ 0 }

#[no_mangle]
pub extern "C" fn tick() -> i32 { /* called every main loop iteration */ 0 }
```

## Project Structure

```
kernel/src/
├── platform.rs              # Pi 5 vs QEMU constants
├── arch/aarch64/
│   ├── exceptions.rs        # Exception vector table
│   ├── gic.rs               # GIC-400 interrupt controller
│   ├── timer.rs             # ARM Generic Timer (millis/micros)
│   └── uart.rs              # PL011 UART (debug console)
├── drivers/
│   ├── pci.rs               # PCIe → RP1 south bridge discovery
│   ├── xhci.rs              # USB 3.0 host controller
│   ├── gem.rs               # Cadence GEM Gigabit Ethernet
│   ├── rp1_gpio.rs          # RP1 GPIO (28 pins, header access)
│   ├── rp1_spi.rs           # RP1 SPI (DesignWare APB SSI)
│   └── rp1_i2c.rs           # RP1 I2C (DesignWare I2C, 2 buses)
├── usb/
│   └── sirius.rs            # SIRIUSi-HS protocol (reverse-engineered)
├── net/
│   ├── mod.rs               # smoltcp TCP/IP stack
│   ├── mdns.rs              # mDNS service discovery
│   ├── hotswap.rs           # WASM hot-swap receiver (port 7421)
│   └── tcp_shell.rs         # Remote shell (port 2222)
├── daq/
│   ├── opendaq.rs           # Native Streaming binary protocol
│   ├── signal.rs            # 8 ADC channels + time domain
│   └── ringbuffer.rs        # Lock-free SPSC ring buffer
├── wasm/
│   ├── runtime.rs           # wasmi-based WASM interpreter
│   └── host_functions.rs    # folk_daq_* host function bridge
├── memory/
│   ├── jit.rs               # W^X page management (future JIT)
│   └── pagetable.rs         # Page table walker (aarch64/x86_64)
└── silverfir/               # Experimental JIT compiler (future)
    ├── parser.rs
    ├── compiler.rs
    └── runtime.rs

apps/                        # WASM DSP modules (build separately)
├── lowpass_filter/
├── bandpass_filter/
├── peak_detector/
└── fft_analyzer/

tools/
└── deploy_wasm.py           # Build + upload WASM to DAQ node
```

## BCM2712 / RP1 Hardware Map

| Component | Address / Offset | Driver |
|---|---|---|
| UART0 (PL011) | `0x10_7D00_1000` | `uart.rs` |
| GIC-400 | `0x10_7FFF_9000` | `gic.rs` |
| PCIe ECAM | `0x10_0012_0000` | `pci.rs` |
| RP1 Ethernet (GEM) | BAR1 + `0x10_0000` | `gem.rs` |
| RP1 xHCI USB 0 | BAR1 + `0x20_0000` | `xhci.rs` |
| RP1 GPIO | BAR1 + `0x0D_0000` | `rp1_gpio.rs` |
| RP1 RIO (fast GPIO) | BAR1 + `0x0E_0000` | `rp1_gpio.rs` |
| RP1 PADS | BAR1 + `0x0F_0000` | `rp1_gpio.rs` |
| RP1 SPI0 | BAR1 + `0x05_0000` | `rp1_spi.rs` |
| RP1 I2C0 | BAR1 + `0x07_0000` | `rp1_i2c.rs` |
| RP1 I2C1 | BAR1 + `0x07_4000` | `rp1_i2c.rs` |

## openDAQ Protocol

DewesoftX compatible, locked to **protocol v3.20.6**.

**mDNS:** `_opendaq-nd._tcp.local`, TXT: `caps=OPENDAQ`, `version=3.20.6`

**Binary packets (port 7420, raw TCP):**
```
[0xDA51 sync][type:u8][flags:u8][stream_id:u32 LE][size:u32 LE][samples...]
```

## Based On

- [PQTech-openDAQ](https://github.com/sverrekm/PQTech-openDAQ) — reverse-engineered SIRIUS USB protocol
- [openDAQ SDK](https://github.com/openDAQ/openDAQ) — protocol reference
- [Folkering OS](https://github.com/merknu/folkering-os) — kernel architecture
- [wasmi](https://github.com/wasmi-labs/wasmi) — WebAssembly interpreter
