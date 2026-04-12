#!/usr/bin/env python3
"""
Folkering DAQ — WASM Hot-Swap Deploy Tool

Builds a WASM DSP module, uploads it to the DAQ node, and verifies deployment.

Usage:
    python deploy_wasm.py <app_name> [--ip <daq_ip>] [--port <port>] [--no-build]

Examples:
    python deploy_wasm.py lowpass_filter
    python deploy_wasm.py lowpass_filter --ip 192.168.1.100
    python deploy_wasm.py lowpass_filter --no-build  # skip cargo build, use existing .wasm

Protocol:
    1. Connect TCP to <ip>:<port> (default 192.168.1.100:7421)
    2. Send: [4-byte LE length][WASM binary]
    3. Receive: "OK: <msg>\\n" or "ERR: <msg>\\n"
"""

import argparse
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

# Project root (relative to this script)
PROJECT_ROOT = Path(__file__).parent.parent
APPS_DIR = PROJECT_ROOT / "apps"
WASM_DIR = PROJECT_ROOT / "wasm-apps"

DEFAULT_IP = "192.168.1.100"
DEFAULT_PORT = 7421
TIMEOUT = 10  # seconds


def build_wasm(app_name: str) -> Path:
    """Build the WASM module using cargo."""
    app_dir = APPS_DIR / app_name
    if not app_dir.exists():
        print(f"ERROR: App directory not found: {app_dir}")
        sys.exit(1)

    print(f"[BUILD] Building {app_name} for wasm32-unknown-unknown...")
    result = subprocess.run(
        ["cargo", "build", "--target", "wasm32-unknown-unknown", "--release"],
        cwd=str(app_dir),
        capture_output=True,
        text=True,
    )

    if result.returncode != 0:
        print(f"[BUILD] FAILED:\n{result.stderr}")
        sys.exit(1)

    # Find the .wasm output
    # Cargo uses hyphens in dir names but underscores in filenames
    wasm_name = app_name.replace("-", "_") + ".wasm"
    wasm_path = app_dir / "target" / "wasm32-unknown-unknown" / "release" / wasm_name

    if not wasm_path.exists():
        print(f"ERROR: WASM binary not found at {wasm_path}")
        sys.exit(1)

    size = wasm_path.stat().st_size
    print(f"[BUILD] OK — {wasm_name} ({size} bytes)")

    # Copy to wasm-apps/ for reference
    WASM_DIR.mkdir(exist_ok=True)
    dest = WASM_DIR / wasm_name
    import shutil
    shutil.copy2(wasm_path, dest)

    return wasm_path


def deploy_wasm(wasm_path: Path, ip: str, port: int) -> bool:
    """Upload WASM binary to DAQ node via TCP hot-swap protocol."""
    wasm_bytes = wasm_path.read_bytes()
    size = len(wasm_bytes)

    print(f"[DEPLOY] Connecting to {ip}:{port}...")

    try:
        sock = socket.create_connection((ip, port), timeout=TIMEOUT)
    except (ConnectionRefusedError, socket.timeout, OSError) as e:
        print(f"[DEPLOY] Connection failed: {e}")
        return False

    try:
        # Send: [4-byte LE length][WASM binary]
        header = struct.pack("<I", size)
        sock.sendall(header + wasm_bytes)
        print(f"[DEPLOY] Sent {size} bytes, waiting for response...")

        # Receive response
        sock.settimeout(TIMEOUT)
        response = b""
        while b"\n" not in response:
            chunk = sock.recv(256)
            if not chunk:
                break
            response += chunk

        response_str = response.decode("utf-8", errors="replace").strip()
        print(f"[DEPLOY] Response: {response_str}")

        if response_str.startswith("OK"):
            print(f"[DEPLOY] SUCCESS — module deployed and running")
            return True
        else:
            print(f"[DEPLOY] FAILED — {response_str}")
            return False

    finally:
        sock.close()


def main():
    parser = argparse.ArgumentParser(
        description="Deploy WASM DSP modules to Folkering DAQ nodes"
    )
    parser.add_argument("app_name", help="Name of the app directory in apps/")
    parser.add_argument("--ip", default=DEFAULT_IP, help=f"DAQ node IP (default: {DEFAULT_IP})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"Hot-swap port (default: {DEFAULT_PORT})")
    parser.add_argument("--no-build", action="store_true", help="Skip cargo build, use existing .wasm")
    parser.add_argument("--wasm", type=str, help="Path to pre-built .wasm file (overrides app_name)")

    args = parser.parse_args()

    if args.wasm:
        wasm_path = Path(args.wasm)
        if not wasm_path.exists():
            print(f"ERROR: WASM file not found: {wasm_path}")
            sys.exit(1)
        print(f"[INFO] Using pre-built WASM: {wasm_path} ({wasm_path.stat().st_size} bytes)")
    elif args.no_build:
        wasm_name = args.app_name.replace("-", "_") + ".wasm"
        wasm_path = WASM_DIR / wasm_name
        if not wasm_path.exists():
            print(f"ERROR: Pre-built WASM not found: {wasm_path}")
            print(f"       Run without --no-build to build it first.")
            sys.exit(1)
        print(f"[INFO] Using existing WASM: {wasm_path} ({wasm_path.stat().st_size} bytes)")
    else:
        wasm_path = build_wasm(args.app_name)

    success = deploy_wasm(wasm_path, args.ip, args.port)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
