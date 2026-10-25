//! Adapters binding the middleware BLE backends (advertising pub/sub and
//! GATT RPC) to the badge radio (FR-38).
//!
//! The badge has one legacy advertising set and one GATT server, both owned
//! by the firmware's BLE task. These modules are the pure, host-tested part
//! of that binding; the firmware only moves bytes between them and
//! trouble-host.
//!
//! - [`mux`]: [`mux::AdvMux`], the immutable schedule that time-shares the
//!   single advertising set between the stock OZ trade beacon and queued
//!   middleware publishes (beacon at least 70 % of the time, never absent
//!   for more than one 300 ms middleware slot).
//! - [`advert`]: [`advert::MuxAdvertiser`], a middleware `Advertiser` that
//!   hands advertising data to the multiplexer (so the middleware's
//!   `AdvertisingPublisher` works unchanged); [`advert::ScanQueue`], a
//!   bounded queue of raw scan reports implementing the middleware `Scanner`
//!   (`try_next_report` pops without waiting); and [`advert::app_subscriber`],
//!   the middleware `AdvertisingSubscriber` over that queue preset for wasm
//!   app keys (the `app` prefix filter and the [`advert::KNOWN_APP_KEYS`]
//!   learned in its `ShortIdTable`).
//! - [`rpc`]: [`rpc::BadgeHandler`], the pure middleware `Handler` behind the
//!   badge's GATT `Responder` (keys `conagotchi/info`, `conagotchi/pet`,
//!   `conagotchi/pattern`), and [`rpc::RpcLink`], a middleware `GattServer`
//!   over the abstract [`rpc::RpcIo`] (the RPC characteristic's writes,
//!   notifications and the connection lifecycle).
//!
//! The middleware types the firmware drives (`AdvertisingPublisher`,
//! `AttMtu`, `ShortIdError`, `AdvertisingError`) are re-exported here.
//!
//! Middleware adverts use the manufacturer-data company id [`MW_COMPANY_ID`],
//! the Bluetooth SIG id reserved for testing (`0xFFFF`), which is also the
//! middleware default. The stock OZ beacon uses its own manufacturer data, so
//! the two never collide in the scanner.

pub mod advert;
pub mod mux;
pub mod rpc;

#[cfg(test)]
mod testutil;

/// The middleware items the firmware's BLE task needs, re-exported so it
/// does not depend on the middleware crate directly.
pub use middleware::ble::advert::{
    AdvertisingError, AdvertisingPublisher, MAX_ADVERT_PAYLOAD, ShortIdError,
};
pub use middleware::ble::gatt::AttMtu;

/// Company id carried in the manufacturer data of middleware adverts
/// (`middleware::ble::advert::TEST_COMPANY_ID`, `0xFFFF`).
pub const MW_COMPANY_ID: u16 = middleware::ble::advert::TEST_COMPANY_ID;

/// Longest app key text the BLE adapters store (equal to `wasm_abi::MAX_KEY`
/// and to the ESP-NOW transport's `MAX_KEY`; the firmware asserts this at
/// compile time).
pub const APP_KEY_LEN: usize = 64;
