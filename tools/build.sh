#!/bin/bash
# Build Folkering DAQ kernel for aarch64
set -e

cd "$(dirname "$0")/.."

echo "=== Building Folkering DAQ Kernel (aarch64) ==="
cd kernel
cargo build --release 2>&1

echo ""
echo "=== Build complete ==="
ls -la target/aarch64-folkering-daq/release/kernel 2>/dev/null || echo "(binary in target dir)"
