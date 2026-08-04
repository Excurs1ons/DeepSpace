//! build.rs — 用 cbindgen 从 ffi.rs 自动生成 include/deepspace.h
//!
//! 手写头文件容易与 Rust 侧漂移；改为每次构建时从 `#[no_mangle] extern "C"`
//! 导出自动生成，保证 ABI 契约永远与实现一致。

fn main() {
    let crate_dir = std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR");
    let config = cbindgen::Config::from_file("cbindgen.toml").expect("cbindgen.toml");

    cbindgen::Builder::new()
        .with_crate(&crate_dir)
        .with_config(config)
        .generate()
        .expect("cbindgen generate failed")
        // 输出到仓库根 include/（文档、smoke test 引用此路径）
        .write_to_file("../include/deepspace.h");

    // 仅当 ffi 或配置变化时重跑生成
    println!("cargo:rerun-if-changed=src/ffi.rs");
    println!("cargo:rerun-if-changed=cbindgen.toml");
}
