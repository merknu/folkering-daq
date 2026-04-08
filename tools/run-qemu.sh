#!/bin/bash
# Run Folkering DAQ kernel in QEMU aarch64 virt machine
# Requires: qemu-system-aarch64
#
# QEMU virt provides: PL011 UART, GICv2, virtio devices
# Kernel boots via -kernel flag (Limine not needed for direct kernel boot)

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
KERNEL="$PROJECT_DIR/target/aarch64-unknown-none/release/kernel"

# Build if needed
if [ ! -f "$KERNEL" ] || [ "$1" = "build" ]; then
    echo "=== Building kernel (qemu-virt) ==="
    cd "$PROJECT_DIR"
    cargo build --release --features qemu-virt
fi

echo "=== Launching QEMU aarch64 ==="
echo "  Press Ctrl-A X to exit"
echo ""

exec qemu-system-aarch64 \
    -M virt \
    -cpu cortex-a76 \
    -m 512M \
    -nographic \
    -serial mon:stdio \
    -kernel "$KERNEL" \
    "$@"
