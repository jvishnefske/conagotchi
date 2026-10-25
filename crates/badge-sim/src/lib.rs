//! The badge's functional core (`badge-core`) as a wasm32 module, so
//! `badgectl sim` can run many badges on a PC with the exact logic of the
//! firmware.
//!
//! - [`badge`]: one simulated badge, `App` plus a pure shell that renders the
//!   screen and answers the hardware requests the simulator has no hardware
//!   for.
//! - [`fb`]: the 240 × 240 RGB565 framebuffer `ui::render` draws onto.
//! - [`inputs`]: export arguments to core `Input`s.
//! - [`wire`]: the tagged record format of the module's output (one record
//!   per effect for the runner), encoded by the guest and decoded by the host.
//! - `abi` (wasm32 only): the `sim_*` C-ABI exports and the memory protocol.
//!
//! The crate is `no_std` and allocation-free on every target. On the host it
//! links `std` only so that its `cdylib` has a panic handler and its tests
//! can use `std`; the code itself never uses it. `unsafe_code` is denied
//! crate-wide and forbidden in every module except `abi`, which needs the
//! (unsafe, in Rust 2024) `no_mangle` attribute to export symbols; see its
//! docs.
//!
//! Build the module with `cargo build -p badge-sim --release --target
//! wasm32-unknown-unknown`; `badgectl`'s build script does this
//! automatically.

#![no_std]
#![deny(unsafe_code)]

#[cfg(not(target_arch = "wasm32"))]
extern crate std;

#[cfg(target_arch = "wasm32")]
mod abi;
pub mod badge;
pub mod fb;
pub mod inputs;
pub mod wire;
