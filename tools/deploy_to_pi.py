#!/usr/bin/env python3
"""
Folkering DAQ — Deploy kernel to Raspberry Pi 5 via SSH

Builds the kernel, converts to binary, and deploys to the Pi's boot partition.
The Pi must be running Pi OS with SSH access.

Usage:
    python tools/deploy_to_pi.py                    # Build + deploy
    python tools/deploy_to_pi.py --wifi             # With WiFi support
    python tools/deploy_to_pi.py --test-boot        # Deploy AND reboot into folkering-daq
    python tools/deploy_to_pi.py --revert           # Restore original Linux kernel

The kernel is deployed as /boot/firmware/kernel8.img.folkering
To actually boot it, use --test-boot which swaps it in and reboots.
"""

import argparse
import subprocess
import sys
import os
from pathlib import Path

PROJECT_DIR = Path(__file__).parent.parent
KERNEL_DIR = PROJECT_DIR / "kernel"
BUILD_DIR = PROJECT_DIR / "build"

PI_HOST = "knut@192.168.68.72"
PI_BOOT = "/boot/firmware"

def run(cmd, **kwargs):
    print(f"  $ {cmd}")
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True, **kwargs)
    if result.returncode != 0:
        print(f"  FAILED: {result.stderr.strip()}")
        return False
    return True

def build_kernel(features):
    print(f"\n[BUILD] Compiling kernel (features: {features})...")

    os.makedirs(BUILD_DIR, exist_ok=True)

    ok = run(
        f"cargo build --release --target aarch64-unknown-none "
        f"--features \"{features}\" --no-default-features",
        cwd=str(KERNEL_DIR)
    )
    if not ok:
        print("  Kernel build failed!")
        sys.exit(1)

    elf = KERNEL_DIR / "target" / "aarch64-unknown-none" / "release" / "kernel"
    if not elf.exists():
        print(f"  ERROR: ELF not found at {elf}")
        sys.exit(1)

    print(f"  ELF size: {elf.stat().st_size:,} bytes")

    # Convert ELF to raw binary
    img = BUILD_DIR / "kernel8.img"

    for objcopy in ["rust-objcopy", "llvm-objcopy", "aarch64-none-elf-objcopy"]:
        if run(f"{objcopy} -O binary {elf} {img}"):
            break
    else:
        print("  ERROR: No objcopy found. Run: cargo install cargo-binutils")
        sys.exit(1)

    print(f"  Binary size: {img.stat().st_size:,} bytes")
    return img

def deploy_to_pi(img_path):
    print(f"\n[DEPLOY] Uploading to {PI_HOST}...")

    # Upload kernel binary
    ok = run(f"scp {img_path} {PI_HOST}:/tmp/kernel8.img.folkering")
    if not ok:
        print("  SCP failed — is the Pi reachable?")
        sys.exit(1)

    # Copy to boot partition (requires sudo)
    ok = run(f'ssh {PI_HOST} "sudo cp /tmp/kernel8.img.folkering {PI_BOOT}/kernel8.img.folkering"')
    if not ok:
        sys.exit(1)

    # Create config for folkering-daq
    config = """# Folkering DAQ bare-metal config
arm_64bit=1
kernel=kernel8.img.folkering
enable_uart=1
uart_2ndstage=1
dtoverlay=disable-bt
boot_delay=0
disable_splash=1
gpu_mem=16"""

    run(f'ssh {PI_HOST} "echo \'{config}\' | sudo tee {PI_BOOT}/config-folkering.txt > /dev/null"')

    print(f"  Kernel deployed to {PI_BOOT}/kernel8.img.folkering")
    print(f"  Config at {PI_BOOT}/config-folkering.txt")

def test_boot():
    print(f"\n[TEST-BOOT] Swapping kernel and rebooting...")

    # Backup original kernel and config
    cmds = [
        f"sudo cp {PI_BOOT}/kernel8.img {PI_BOOT}/kernel8.img.linux-backup",
        f"sudo cp {PI_BOOT}/config.txt {PI_BOOT}/config.txt.linux-backup",
        f"sudo cp {PI_BOOT}/kernel8.img.folkering {PI_BOOT}/kernel8.img",
        f"sudo cp {PI_BOOT}/config-folkering.txt {PI_BOOT}/config.txt",
    ]

    for cmd in cmds:
        run(f'ssh {PI_HOST} "{cmd}"')

    print("  Kernel swapped. Rebooting Pi...")
    print("  Connect serial console (115200 8N1) to see boot output.")
    print("  To revert: python tools/deploy_to_pi.py --revert")

    run(f'ssh {PI_HOST} "sudo reboot"')

def revert():
    print(f"\n[REVERT] Restoring original Linux kernel...")

    cmds = [
        f"sudo cp {PI_BOOT}/kernel8.img.linux-backup {PI_BOOT}/kernel8.img",
        f"sudo cp {PI_BOOT}/config.txt.linux-backup {PI_BOOT}/config.txt",
    ]

    for cmd in cmds:
        run(f'ssh {PI_HOST} "{cmd}"')

    print("  Linux kernel restored. Rebooting...")
    run(f'ssh {PI_HOST} "sudo reboot"')

def main():
    parser = argparse.ArgumentParser(description="Deploy Folkering DAQ to Raspberry Pi 5")
    parser.add_argument("--wifi", action="store_true", help="Enable WiFi support")
    parser.add_argument("--embedded-dsp", action="store_true", help="Embed lowpass filter")
    parser.add_argument("--jit", action="store_true", help="Enable Silverfir JIT")
    parser.add_argument("--test-boot", action="store_true", help="Swap kernel and reboot")
    parser.add_argument("--revert", action="store_true", help="Restore Linux kernel")
    parser.add_argument("--deploy-only", action="store_true", help="Skip build, just deploy")
    parser.add_argument("--ip", default="192.168.68.72", help="Pi IP address")

    args = parser.parse_args()

    global PI_HOST
    PI_HOST = f"knut@{args.ip}"

    if args.revert:
        revert()
        return

    # Build features
    features = "pi5"
    if args.wifi: features += ",wifi"
    if args.embedded_dsp: features += ",embedded-dsp"
    if args.jit: features += ",silverfir-jit"

    if not args.deploy_only:
        img = build_kernel(features)
    else:
        img = BUILD_DIR / "kernel8.img"
        if not img.exists():
            print(f"ERROR: {img} not found. Build first without --deploy-only")
            sys.exit(1)

    deploy_to_pi(img)

    if args.test_boot:
        test_boot()
    else:
        print(f"\n[DONE] Kernel ready on Pi. To test-boot:")
        print(f"  python tools/deploy_to_pi.py --test-boot")

if __name__ == "__main__":
    main()
