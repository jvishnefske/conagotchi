//! Functional core of the Conagotchi discovery firmware.
//!
//! Everything here is pure, `no_std`, heap-free and tested on the host:
//!
//! - [`ble_adv`]: BLE advertising set contents and the OZ advert forwarder.
//! - [`ble_metrics`]: BLE air metrics shown on the BLE screen.
//! - [`board`]: the badge pin map and GPIO probe allowlist.
//! - [`color`]: the shared RGB colour type.
//! - [`contacts`]: badges we completed a Chi trade with.
//! - [`gatt`]: the GATT service "Conagotchi" (UUIDs, write mapping, info text).
//! - [`frame`]: COBS framing used on the RTT channels (firmware and host).
//! - [`io`]: the [`io::Input`] / [`io::Effect`] boundary to the firmware shell.
//! - [`buttons`]: debouncing of raw button levels into press/release edges.
//! - [`leds`]: LED patterns and brightness scaling.
//! - [`lfs`]: read-only littlefs root lookup and stock radio key import.
//! - [`oz`]: stock-badge interop (OZ adverts, signing, lobby, trade, link UART).
//! - [`pet`]: the animated pixel-art cat of the PET screen (state and sprites).
//! - [`app`]: the application state machine, `App::handle(Input) -> (App, Effects)`.
//! - [`probe_layout`]: word layout of the firmware `PROBE` block and its mailbox.
//! - [`qr`]: the project URL as a build-time generated QR code (HOME screen).
//! - [`radio_rank`]: Wi-Fi/BLE result ranking and advertising-data parsing.
//! - [`store`]: record layout of the firmware `conag` flash partition.
//! - [`sync`]: LED timing sync between badges (network time).
//! - [`ui`]: renders the app model onto any `embedded-graphics` draw target.

#![no_std]

pub mod app;
pub mod ble_adv;
pub mod ble_metrics;
pub mod board;
pub mod buttons;
pub mod color;
pub mod contacts;
pub mod frame;
pub mod gatt;
pub mod io;
pub mod leds;
pub mod lfs;
pub mod oz;
pub mod pet;
pub mod probe_layout;
pub mod qr;
pub mod radio_rank;
pub mod store;
pub mod sync;
pub mod ui;
