fn main() {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let dir = manifest_dir.replace('\\', "/");

    // Select linker script based on platform feature
    let linker_script = if cfg!(feature = "qemu-virt") {
        // Check if CARGO_FEATURE_QEMU_VIRT is set (build.rs uses env vars)
        "linker-qemu.ld"
    } else {
        "linker.ld"
    };

    // Use env var check since cfg! doesn't work for features in build.rs
    let script = if std::env::var("CARGO_FEATURE_QEMU_VIRT").is_ok() {
        format!("{}/linker-qemu.ld", dir)
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
}
