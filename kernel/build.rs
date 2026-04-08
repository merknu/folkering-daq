fn main() {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let linker_ld = format!("{}/linker.ld", manifest_dir.replace('\\', "/"));

    println!("cargo:rustc-link-arg=-T{}", linker_ld);
    println!("cargo:rustc-link-arg=-z");
    println!("cargo:rustc-link-arg=common-page-size=0x1000");
    println!("cargo:rustc-link-arg=-z");
    println!("cargo:rustc-link-arg=max-page-size=0x1000");
    println!("cargo:rustc-link-arg=-no-pie");
    println!("cargo:rerun-if-changed=linker.ld");
}
