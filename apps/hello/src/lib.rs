//! Demo payload for the Conagotchi badge, on middleware's `mw-payload` SDK.
//!
//! - `init`: subscribes to [`HELLO`] and to the badge's button edges
//!   ([`BUTTONS`]), shows [`GREETING`] on the HOST screen (`badge/text`),
//!   logs "hello from wasm" and asks for a tick every [`STEP_MS`].
//! - `tick`: moves one lit LED (dim cyan) along the chain (`badge/leds`,
//!   24 bytes R G B) and, every [`PUBLISH_MS`], publishes a little-endian
//!   `u32` counter on [`HELLO`].
//! - `on_message`: logs "rx N" for a counter sample from another badge
//!   (or "rx <len> bytes" for anything else) and "button B down" /
//!   "button B up" for a `[button, pressed]` edge.
//!
//! Failures (a full subscription table, a refused publish) are logged, never
//! fatal. Built by `scripts/build-apps.sh` into `apps/hello.wasm`.
//!
//! The logic is in `app` under `#![forbid(unsafe_code)]`. The crate root is
//! only `deny(unsafe_code)`: `mw_payload::payload!` expands to the five ABI
//! exports with `#[allow(unsafe_code)] #[unsafe(no_mangle)]`, which a
//! crate-wide `forbid` would reject (E0453).

#![no_std]
#![deny(unsafe_code)]

mod app;

pub use app::{BUTTONS, GREETING, HELLO, Hello, LED_COUNT, LEDS, PUBLISH_MS, STEP_MS, TEXT, dot_frame};

mw_payload::payload!(Hello);
