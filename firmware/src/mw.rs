//! The badge's middleware bus between the wasm app and the radio transports.
//!
//! - [`publish_app`]: an app publish, offered to every transport that
//!   carries app keys (ESP-NOW and BLE advertising); each has its own bounded
//!   queue, and a full queue drops the message for that transport only.
//! - [`APP_SAMPLES`]: samples on app keys received by any transport, for the
//!   wasm task (which drops duplicates arriving over both transports).
//!
//! Keys are validated by the caller (`badge_mw::keys::app_may_use`).

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use heapless::{String, Vec};

/// Longest key on the bus.
pub const KEY_LEN: usize = wasm_abi::MAX_KEY;
/// Largest payload on the bus.
pub const PAYLOAD_LEN: usize = wasm_abi::MAX_PAYLOAD;

/// One message on an app key.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MwMessage {
    /// The key (`app/...`).
    pub key: String<KEY_LEN>,
    /// The payload.
    pub payload: Vec<u8, PAYLOAD_LEN>,
}

/// App publishes waiting for the ESP-NOW transport.
pub static ESPNOW_OUT: Channel<CriticalSectionRawMutex, MwMessage, 4> = Channel::new();
/// App publishes waiting for the BLE advertising transport.
pub static BLE_OUT: Channel<CriticalSectionRawMutex, MwMessage, 4> = Channel::new();
/// Samples on app keys received by any transport, for the wasm task.
pub static APP_SAMPLES: Channel<CriticalSectionRawMutex, MwMessage, 8> = Channel::new();

/// Offers an app publish to every transport. Returns how many accepted it.
pub fn publish_app(msg: &MwMessage) -> usize {
    usize::from(ESPNOW_OUT.try_send(msg.clone()).is_ok()) + usize::from(BLE_OUT.try_send(msg.clone()).is_ok())
}
