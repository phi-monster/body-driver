//! Link the **proven** fast face into the slow face.
//!
//! 🔴 The alternative — reimplementing the limit / force / watchdog rules in Rust "to avoid the
//! FFI" — would leave two copies of the rule and one proof, and the unproven copy is the one that
//! drifts. So the Ada static library is a hard dependency of this crate.
//!
//! If it is absent, the build **fails loudly** rather than falling back to a Rust reimplementation.
//! A silent fallback is exactly the failure shape this whole layer exists to eliminate: everything
//! keeps working, nothing reports a problem, and the guarantee is quietly gone.
//!
//! Build the library first:
//!   cd ../fast && gprbuild -P body_layer_fast_lib.gpr

use std::path::PathBuf;

fn main() {
    // Tests of the pure-Rust half must still run on a machine without a GNAT toolchain, so the
    // link is opt-in via a feature. What is NOT allowed is silently substituting Rust logic.
    if std::env::var("CARGO_FEATURE_FAST").is_err() {
        println!("cargo:rerun-if-changed=build.rs");
        return;
    }

    let here = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let libdir = here.join("../fast/lib");
    let lib = libdir.join("libbodylayerfast.a");

    if !lib.exists() {
        panic!(
            "the proven fast face is not built: {} is missing.\n\
             Build it first:  cd ../fast && gprbuild -P body_layer_fast_lib.gpr\n\
             This build does NOT fall back to a Rust reimplementation -- two copies of a safety \
             rule with one proof is worse than one copy with none, because the unproven copy is \
             the one that drifts.",
            lib.display()
        );
    }

    println!("cargo:rustc-link-search=native={}", libdir.display());
    println!("cargo:rustc-link-lib=static=bodylayerfast");

    // GNAT's runtime, linked STATICALLY.
    //
    // 🔴 Not a packaging preference. A dynamic `libgnat` means the safety kernel's presence depends
    // on a file resolving correctly on the target at run time -- and "the guarantee silently is not
    // there" is the exact failure this layer exists to remove. Statically linked, the proof and the
    // binary travel together.
    let rts = std::env::var("BODY_LAYER_GNAT_LIB").unwrap_or_else(|_| {
        panic!(
            "BODY_LAYER_GNAT_LIB is not set. Point it at GNAT's adalib, e.g.\n               export BODY_LAYER_GNAT_LIB=$(ls -d \
             ~/.local/share/alire/toolchains/gnat_native_*/lib/gcc/*/*/adalib | head -1)"
        )
    });
    // 🔴 By FULL PATH to the archive, not `-lgnat`.  GNAT ships `libgnat.a` and `libgnat.dylib`
    // side by side in the same directory, and Apple's linker has no `-Bstatic`, so `-lgnat` picks
    // the dylib -- silently producing a binary whose safety kernel is a file that has to resolve
    // at run time on the robot.  Naming the archive removes the choice.
    let archive = std::path::Path::new(&rts).join("libgnat.a");
    if !archive.exists() {
        panic!("GNAT static runtime not found at {}", archive.display());
    }
    println!("cargo:rustc-link-search=native={rts}");
    println!("cargo:rustc-link-arg={}", archive.display());

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=../fast/lib/libbodylayerfast.a");
}
