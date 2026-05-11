fn main() {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let dir = manifest_dir.replace('\\', "/");

    // Select linker script based on platform feature.
    //
    // qemu-virt  → linker-qemu.ld  (identity at 0x40080000, kernel-arg boot)
    // pi5        → linker-pi5.ld   (identity at 0x80000, Pi-firmware boot)
    // (neither)  → linker.ld       (higher-half VA, Limine boot — used by
    //                               legacy/QEMU+Limine setups)
    //
    // build.rs uses env vars rather than `cfg!` since `cfg!(feature=...)`
    // doesn't see Cargo features inside the build script.
    let script = if std::env::var("CARGO_FEATURE_QEMU_VIRT").is_ok() {
        format!("{}/linker-qemu.ld", dir)
    } else if std::env::var("CARGO_FEATURE_PI5").is_ok() {
        format!("{}/linker-pi5.ld", dir)
    } else {
        format!("{}/linker.ld", dir)
    };

    println!("cargo:rustc-link-arg=-T{}", script);
    println!("cargo:rustc-link-arg=-z");
    println!("cargo:rustc-link-arg=common-page-size=0x1000");
    println!("cargo:rustc-link-arg=-z");
    println!("cargo:rustc-link-arg=max-page-size=0x1000");
    println!("cargo:rustc-link-arg=-no-pie");
    println!("cargo:rerun-if-changed=linker.ld");
    println!("cargo:rerun-if-changed=linker-qemu.ld");
    println!("cargo:rerun-if-changed=linker-pi5.ld");
}
