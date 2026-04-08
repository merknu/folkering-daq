# Run Folkering DAQ kernel in QEMU aarch64 (Windows)
# Requires: qemu-system-aarch64 in PATH
# Install: winget install QEMU.QEMU  OR  choco install qemu

$ErrorActionPreference = "Stop"
$ProjectDir = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
if (-not $ProjectDir) { $ProjectDir = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path) }
$Kernel = "$ProjectDir\target\aarch64-unknown-none\release\kernel"

# Build
Write-Host "=== Building kernel (qemu-virt) ===" -ForegroundColor Green
Push-Location $ProjectDir
cargo build --release --features qemu-virt
Pop-Location

if (-not (Test-Path $Kernel)) {
    Write-Host "ERROR: Kernel not found at $Kernel" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "=== Launching QEMU aarch64 ===" -ForegroundColor Green
Write-Host "  Press Ctrl-A X to exit"
Write-Host ""

& qemu-system-aarch64 `
    -M virt `
    -cpu cortex-a76 `
    -m 512M `
    -nographic `
    -serial mon:stdio `
    -kernel $Kernel `
    @args
