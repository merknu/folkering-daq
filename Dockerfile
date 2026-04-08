# Folkering DAQ — Build + Run in QEMU aarch64
# Usage:
#   docker build -t folkering-daq .
#   docker run --rm -it folkering-daq          # Serial console
#   docker run --rm -it folkering-daq build    # Just build, copy kernel out

FROM rust:1.85-bookworm AS builder

# Install QEMU and build tools
RUN apt-get update && apt-get install -y \
    qemu-system-arm \
    xorriso \
    mtools \
    wget \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install Rust nightly with aarch64 target
RUN rustup toolchain install nightly-2026-01-20 \
    --component rust-src llvm-tools-preview \
    --target aarch64-unknown-none wasm32-unknown-unknown

# Download Limine bootloader (v8.x, supports aarch64)
RUN git clone --branch v8.x-binary --depth=1 https://github.com/limine-bootloader/limine.git /opt/limine

WORKDIR /build
COPY . .

# Build kernel
RUN cd kernel && \
    cargo +nightly-2026-01-20 build --release --features qemu-virt && \
    cp target/aarch64-unknown-none/release/kernel /build/boot/kernel.elf

# Create bootable ISO for QEMU
RUN mkdir -p /build/iso/boot && \
    cp /build/boot/kernel.elf /build/iso/boot/ && \
    cp /build/boot/limine.conf /build/iso/boot/ && \
    cp /opt/limine/BOOTAA64.EFI /build/iso/EFI/BOOT/BOOTAA64.EFI 2>/dev/null || true

# === Runtime stage ===
FROM debian:bookworm-slim AS runtime

RUN apt-get update && apt-get install -y \
    qemu-system-arm \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /build/boot/kernel.elf /folkering/kernel.elf
COPY --from=builder /build/boot/limine.conf /folkering/limine.conf
COPY --from=builder /opt/limine /opt/limine

# Create QEMU launch script
RUN echo '#!/bin/bash\n\
qemu-system-aarch64 \
  -M virt \
  -cpu cortex-a76 \
  -m 512M \
  -nographic \
  -serial mon:stdio \
  -kernel /folkering/kernel.elf \
  "$@"' > /folkering/run.sh && chmod +x /folkering/run.sh

WORKDIR /folkering

ENTRYPOINT ["/folkering/run.sh"]
