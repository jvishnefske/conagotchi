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
//!   `AdvertisingPublisher` works unchanged), and [`advert::ScanQueue`], a
//!   bounded queue of raw scan reports implementing the middleware
//!   `Scanner` (so `AdvertisingSubscriber` works unchanged).
//! - [`appkeys`]: [`appkeys::AppKeyTable`], the dynamic subscription table the
//!   firmware uses instead of `AdvertisingSubscriber`: it resolves the 16-bit
//!   short id in a middleware advert back to an `app/...` key text and drops
//!   repeats per sender.
//! - [`rpc`]: [`rpc::BadgeHandler`], the pure middleware `Handler` behind the
//!   badge's GATT `Responder` (keys `conagotchi/info`, `conagotchi/pet`,
//!   `conagotchi/pattern`), and [`rpc::RpcLink`], a middleware `GattLink`
//!   over the abstract [`rpc::RpcIo`] (the RPC characteristic's writes and
//!   notifications).
//!
//! The middleware traits and types the firmware drives (`Key`, `Publisher`,
//! `Responder`, `AdvertisingPublisher`, `AttMtu`) are re-exported here.
//!
//! Middleware adverts use the manufacturer-data company id [`MW_COMPANY_ID`],
//! the Bluetooth SIG id reserved for testing (`0xFFFF`), which is also the
//! middleware default. The stock OZ beacon uses its own manufacturer data, so
//! the two never collide in the scanner.

pub mod advert;
pub mod appkeys;
pub mod mux;
pub mod rpc;

#[cfg(test)]
mod testutil;

/// The middleware items the firmware's BLE task needs, re-exported so it
/// does not depend on the middleware crate directly.
pub use middleware::ble::advert::AdvertisingPublisher;
pub use middleware::ble::gatt::AttMtu;
pub use middleware::{Key, Publisher, Responder};

/// Company id carried in the manufacturer data of middleware adverts
/// (`middleware::ble::advert::TEST_COMPANY_ID`, `0xFFFF`).
pub const MW_COMPANY_ID: u16 = middleware::ble::advert::TEST_COMPANY_ID;

/// Longest app key text the BLE adapters store (equal to `wasm_abi::MAX_KEY`;
/// the firmware asserts this at compile time).
pub const APP_KEY_LEN: usize = 64;

/// Largest payload one middleware advert carries: 31 bytes minus Flags (3),
/// the manufacturer AD header and company id (4) and the publish header
/// `[short id][sequence]` (3).
pub const MAX_ADVERT_PAYLOAD: usize =
    middleware::ble::advert::MAX_ADV_DATA - 3 - 4 - middleware::ble::advert::PUBLISH_HEADER_LEN;
