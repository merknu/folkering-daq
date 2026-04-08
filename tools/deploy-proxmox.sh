#!/bin/bash
# Deploy Folkering DAQ to Proxmox as a QEMU VM
#
# Usage: ./deploy-proxmox.sh <proxmox-host> [vmid]
#
# Creates an aarch64 QEMU VM on Proxmox with:
# - Direct kernel boot (no UEFI/Limine needed)
# - virtio-net for networking
# - Serial console for monitoring

set -e

PROXMOX_HOST="${1:?Usage: $0 <proxmox-host> [vmid]}"
VMID="${2:-900}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
KERNEL="$PROJECT_DIR/target/aarch64-unknown-none/release/kernel"

# Build for QEMU
echo "=== Building kernel ==="
cd "$PROJECT_DIR"
cargo build --release --features qemu-virt

# Upload kernel to Proxmox
echo "=== Uploading to $PROXMOX_HOST ==="
scp "$KERNEL" "root@$PROXMOX_HOST:/var/lib/vz/folkering-daq-kernel.elf"

# Create/update VM
echo "=== Configuring VM $VMID ==="
ssh "root@$PROXMOX_HOST" bash -s <<'REMOTE_SCRIPT'
VMID=__VMID__

# Delete old VM if exists
qm destroy $VMID 2>/dev/null || true

# Create aarch64 VM
qm create $VMID \
    --name folkering-daq \
    --memory 512 \
    --cores 4 \
    --arch aarch64 \
    --machine virt \
    --cpu host \
    --net0 virtio,bridge=vmbr0 \
    --serial0 socket \
    --args "-kernel /var/lib/vz/folkering-daq-kernel.elf -nographic"

echo "VM $VMID created. Start with: qm start $VMID"
echo "Console: qm terminal $VMID"
REMOTE_SCRIPT

echo "=== Done! ==="
echo "  Start: ssh root@$PROXMOX_HOST 'qm start $VMID'"
echo "  Console: ssh root@$PROXMOX_HOST 'qm terminal $VMID'"
