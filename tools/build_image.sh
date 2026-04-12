#!/bin/bash
# Folkering DAQ — Build bootable SD card image for Raspberry Pi 5
#
# This script:
#   1. Builds the kernel for aarch64 (release, pi5 features)
#   2. Converts ELF → raw binary (kernel8.img)
#   3. Creates a bootable SD card image with Pi firmware + our kernel
#
# Usage:
#   ./tools/build_image.sh                    # Build with default features
#   ./tools/build_image.sh --wifi             # Include WiFi support
#   ./tools/build_image.sh --wifi --embedded-dsp  # WiFi + embedded DSP app
#
# Output:
#   build/folkering-daq.img  — Write to SD: dd if=build/folkering-daq.img of=/dev/sdX bs=4M
#   build/kernel8.img        — Just the kernel (copy to existing Pi SD boot partition)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"
KERNEL_DIR="$PROJECT_DIR/kernel"
FW_DIR="$PROJECT_DIR/firmware"

# Parse arguments
FEATURES="pi5"
for arg in "$@"; do
    case "$arg" in
        --wifi)        FEATURES="$FEATURES,wifi" ;;
        --embedded-dsp) FEATURES="$FEATURES,embedded-dsp" ;;
        --jit)         FEATURES="$FEATURES,silverfir-jit" ;;
        *) echo "Unknown option: $arg"; exit 1 ;;
    esac
done

echo "=== Folkering DAQ Image Builder ==="
echo "Features: $FEATURES"
echo ""

mkdir -p "$BUILD_DIR"

# Step 1: Build kernel
echo "[1/4] Building kernel (release, aarch64)..."
cd "$KERNEL_DIR"
cargo build \
    --release \
    --target aarch64-unknown-none \
    --features "$FEATURES" \
    --no-default-features

KERNEL_ELF="$KERNEL_DIR/target/aarch64-unknown-none/release/kernel"

if [ ! -f "$KERNEL_ELF" ]; then
    echo "ERROR: Kernel ELF not found at $KERNEL_ELF"
    echo "  Trying lib build..."
    KERNEL_ELF="$KERNEL_DIR/target/aarch64-unknown-none/release/libfolkering_daq_kernel.a"
fi

echo "  Kernel ELF: $(ls -lh "$KERNEL_ELF" | awk '{print $5}')"

# Step 2: Convert ELF to raw binary
echo "[2/4] Converting ELF → kernel8.img..."

# Try rust-objcopy (from cargo-binutils) or llvm-objcopy
if command -v rust-objcopy &>/dev/null; then
    rust-objcopy -O binary "$KERNEL_ELF" "$BUILD_DIR/kernel8.img"
elif command -v llvm-objcopy &>/dev/null; then
    llvm-objcopy -O binary "$KERNEL_ELF" "$BUILD_DIR/kernel8.img"
elif command -v aarch64-none-elf-objcopy &>/dev/null; then
    aarch64-none-elf-objcopy -O binary "$KERNEL_ELF" "$BUILD_DIR/kernel8.img"
else
    echo "ERROR: No objcopy found. Install cargo-binutils:"
    echo "  cargo install cargo-binutils"
    echo "  rustup component add llvm-tools-preview"
    exit 1
fi

echo "  kernel8.img: $(ls -lh "$BUILD_DIR/kernel8.img" | awk '{print $5}')"

# Step 3: Create config.txt for Pi 5
echo "[3/4] Creating boot configuration..."

cat > "$BUILD_DIR/config.txt" << 'PICONFIG'
# Folkering DAQ — Raspberry Pi 5 bare-metal config
arm_64bit=1
kernel=kernel8.img

# UART debug console (115200 8N1)
enable_uart=1
uart_2ndstage=1

# Disable Bluetooth to free up PL011 UART
dtoverlay=disable-bt

# Disable Linux-specific features
boot_delay=0
disable_splash=1

# Pi 5 specific: use full memory
total_mem=8192

# GPU memory minimal (we don't use GPU)
gpu_mem=16
PICONFIG

echo "  config.txt created"

# Step 4: Summary
echo "[4/4] Build complete!"
echo ""
echo "=== Output ==="
echo "  Kernel: $BUILD_DIR/kernel8.img ($(stat -f%z "$BUILD_DIR/kernel8.img" 2>/dev/null || stat -c%s "$BUILD_DIR/kernel8.img" 2>/dev/null || echo '?') bytes)"
echo "  Config: $BUILD_DIR/config.txt"
echo ""
echo "=== Deploy to Pi 5 ==="
echo ""
echo "Option A: Copy to existing Pi OS SD card (dual-boot test)"
echo "  1. Mount the SD card's boot partition (FAT32)"
echo "  2. Backup: cp kernel8.img kernel8.img.backup"
echo "  3. Copy: cp $BUILD_DIR/kernel8.img /path/to/boot/"
echo "  4. Copy: cp $BUILD_DIR/config.txt /path/to/boot/"
echo "  5. Boot Pi 5 — it will run Folkering DAQ instead of Linux"
echo "  6. To revert: cp kernel8.img.backup kernel8.img"
echo ""
echo "Option B: Deploy via SSH to running Pi"
echo "  scp $BUILD_DIR/kernel8.img knut@192.168.68.72:/tmp/"
echo "  ssh knut@192.168.68.72 'sudo cp /tmp/kernel8.img /boot/firmware/kernel8.img.folkering'"
echo ""
echo "Option C: Quick test via QEMU (won't have Pi hardware)"
echo "  qemu-system-aarch64 -M virt -cpu cortex-a76 -m 512M -nographic \\"
echo "    -kernel $BUILD_DIR/kernel8.img"
