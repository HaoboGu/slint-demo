fn main() {
    slint_build::compile_with_config(
        "ui/printerdemo.slint",
        slint_build::CompilerConfiguration::new()
            .embed_resources(slint_build::EmbedResourcesKind::EmbedForSoftwareRenderer),
    )
    .unwrap();

    // Put `memory.x` in our output directory and ensure it's
    // on the linker search path.
    // let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    // File::create(out.join("memory.x"))
    //     .unwrap()
    //     .write_all(include_bytes!("memory.x"))
    //     .unwrap();
    // println!("cargo:rustc-link-search={}", out.display());

    // By default, Cargo will re-run a build script whenever
    // any file in the project changes. By specifying `memory.x`
    // here, we ensure the build script is only re-run when
    // `memory.x` is changed.
    // println!("cargo:rerun-if-changed=memory.x");

    // Specify linker arguments.

    // `--nmagic` is required if memory section addresses are not aligned to 0x10000,
    // for example the FLASH and RAM sections in your `memory.x`.
    // See https://github.com/rust-embedded/cortex-m-quickstart/pull/95
    println!("cargo:rustc-link-arg=--nmagic");

    // Set the linker script to the one provided by cortex-m-rt.
    println!("cargo:rustc-link-arg=-Tlink.x");

    // Set the extra linker script from defmt
    println!("cargo:rustc-link-arg=-Tdefmt.x");

    // Use flip-link overflow check: https://github.com/knurling-rs/flip-link
    println!("cargo:rustc-linker=flip-link");
}
