//! Builds `crates/badge-sim` for `wasm32-unknown-unknown` and places the
//! module at `$OUT_DIR/badge_sim.wasm`, which `src/sim/guest.rs` embeds with
//! `include_bytes!`.
//!
//! - `BADGE_SIM_WASM=<path>` skips the build and embeds that file instead
//!   (prebuilt module, or a machine without the wasm32 target).
//! - Otherwise this runs `cargo build -p badge-sim --release --locked
//!   --target wasm32-unknown-unknown` in the workspace, with its own target
//!   directory (`<target>/sim-wasm`) so it never waits on the lock of the
//!   build that runs this script. Compiler wrappers and RUSTFLAGS of the
//!   outer build (clippy, coverage) are not passed on.
//! - If that fails (typically: `rustup target add wasm32-unknown-unknown`
//!   not done), a warning is printed and an empty module is embedded;
//!   `badgectl sim` then reports the problem when it starts.
//!
//! The module is always built with badge-core's default features; the outer
//! build's `--features badge-core/dev` does not reach it.

use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

fn main() {
    if let Err(e) = run() {
        println!("cargo:warning=badge-sim.wasm not built: {e}");
        if let Some(out) = env::var_os("OUT_DIR") {
            let _ = fs::write(Path::new(&out).join("badge_sim.wasm"), b"");
        }
    }
}

/// The workspace target directory, found from `OUT_DIR`
/// (`<target>/<profile>/build/<pkg>-<hash>/out`), else `OUT_DIR` itself.
fn target_dir(out: &Path) -> PathBuf {
    out.ancestors()
        .find(|p| p.file_name().is_some_and(|n| n == "build"))
        .and_then(Path::parent)
        .and_then(Path::parent)
        .map_or_else(|| out.join("sim-target"), |t| t.join("sim-wasm"))
}

fn run() -> Result<(), String> {
    let out = PathBuf::from(env::var_os("OUT_DIR").ok_or("OUT_DIR unset")?);
    let dest = out.join("badge_sim.wasm");
    println!("cargo:rerun-if-env-changed=BADGE_SIM_WASM");
    if let Some(prebuilt) = env::var_os("BADGE_SIM_WASM") {
        let prebuilt = PathBuf::from(prebuilt);
        println!("cargo:rerun-if-changed={}", prebuilt.display());
        fs::copy(&prebuilt, &dest).map_err(|e| format!("copying {}: {e}", prebuilt.display()))?;
        return Ok(());
    }
    let manifest = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").ok_or("CARGO_MANIFEST_DIR unset")?);
    let root = manifest.join("../..");
    for dep in ["crates/badge-sim", "crates/badge-core", "crates/badge-proto", "proto", "Cargo.lock"] {
        println!("cargo:rerun-if-changed={}", root.join(dep).display());
    }
    let target = target_dir(&out);
    let cargo = env::var_os("CARGO").unwrap_or_else(|| "cargo".into());
    let status = Command::new(cargo)
        .current_dir(&root)
        .args(["build", "-p", "badge-sim", "--release", "--locked", "--target", "wasm32-unknown-unknown"])
        .arg("--target-dir")
        .arg(&target)
        .env_remove("RUSTFLAGS")
        .env_remove("CARGO_ENCODED_RUSTFLAGS")
        .env_remove("RUSTC_WRAPPER")
        .env_remove("RUSTC_WORKSPACE_WRAPPER")
        .env_remove("CARGO_BUILD_TARGET")
        .env_remove("CARGO_TARGET_DIR")
        .env_remove("CARGO_LLVM_COV")
        .status()
        .map_err(|e| format!("running cargo: {e}"))?;
    if !status.success() {
        return Err(format!("cargo build for wasm32 failed ({status}); is the wasm32-unknown-unknown target installed?"));
    }
    let built = target.join("wasm32-unknown-unknown/release/badge_sim.wasm");
    fs::copy(&built, &dest).map_err(|e| format!("copying {}: {e}", built.display()))?;
    Ok(())
}
