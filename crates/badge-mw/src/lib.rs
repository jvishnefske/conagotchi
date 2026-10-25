//! Conagotchi's bindings of the [`middleware`] traits.
//!
//! This crate is where the badge validates the middleware concept: badge
//! features are written against `middleware::{Publisher, Subscriber,
//! Requester, Responder, Handler}` and carried by whichever transport fits.
//!
//! - [`keys`]: the keys badges publish on (shared by firmware, simulator and
//!   wasm apps).
//! - [`espnow`]: an ESP-NOW broadcast backend (`Publisher` + `Subscriber`)
//!   over a small radio trait, host-tested with an in-memory fake.
//! - [`ble`]: adapters that bind the middleware's BLE advertising and GATT
//!   backends to the badge's single advertising set and GATT server.
//!
//! `no_std`, no heap, `#![forbid(unsafe_code)]`; async fns in traits as in
//! the middleware crate.

#![no_std]
#![forbid(unsafe_code)]
#![allow(async_fn_in_trait)]

pub mod ble;
pub mod espnow;
pub mod keys;
