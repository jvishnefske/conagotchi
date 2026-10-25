//! The boundary between the functional core and the imperative shell.
//!
//! The firmware shell turns hardware happenings into [`Input`]s, feeds them to
//! `App::handle`, and carries out the returned [`Effect`]s. The core never
//! touches hardware; every side effect it wants is described here.

use badge_proto::{AccessPoint, BadgeMsg, BleDevice, ErrorCode, Indicator, WifiState};
use heapless::{String, Vec};

use crate::ble_metrics::BleSnapshot;
use crate::color::Color;
use crate::contacts::Contact;
use crate::oz::{self, sign::RadioKey};

/// Milliseconds since boot, as reported by the shell's monotonic clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Millis(pub u64);

/// Bit set of raw (undebounced) button levels; bit n set = `Button(n)` held.
///
/// Bit numbering follows the protocol `Button` enum: 1 BOOT, 2 LEFT,
/// 3 SELECT, 4 START, 5 RIGHT.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ButtonMask(pub u8);

/// Maximum Wi-Fi scan entries carried through the core.
pub const MAX_APS: usize = 12;
/// Maximum BLE scan entries carried through the core.
pub const MAX_BLE: usize = 12;
/// Maximum effects produced by a single `App::handle` call (budget derived
/// in the `app` module docs).
pub const MAX_EFFECTS: usize = 12;
/// Largest legacy advertising data (`Effect::Beacon`).
pub const MAX_ADV_DATA: usize = 31;

/// Where a stock-protocol packet came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Via {
    /// A BLE advertisement heard by the continuous scan.
    Ble,
    /// The wired link UART.
    Link,
}

/// What the firmware's flash storage reports.
#[derive(Debug, Clone, PartialEq)]
pub enum StorageEvent {
    /// One stored contact. At boot the firmware sends every stored contact
    /// this way, oldest-saved first, before [`StorageEvent::Loaded`]; one at
    /// a time so that `Input` stays small (it is queued in fixed channels).
    Contact(Contact),
    /// Boot-time load of the `conag` partition finished (sent once, early).
    Loaded {
        /// Radio key, if one was provisioned or imported from the stock
        /// filesystem. `None` = the badge sends unsigned packets.
        key: Option<RadioKey>,
    },
}

/// Something that happened, reported by the shell.
#[derive(Debug, Clone, PartialEq)]
pub enum Input {
    /// Periodic tick (the shell sends one every ~20 ms) with fresh button levels.
    Tick {
        /// Current time.
        now: Millis,
        /// Raw button levels sampled at `now`.
        buttons: ButtonMask,
    },
    /// A complete, successfully decoded host message.
    Host(badge_proto::HostMsg),
    /// A host frame arrived but could not be decoded (bad COBS or protobuf).
    HostGarbage,
    /// Result of an [`Effect::GpioRead`] / [`Effect::GpioWrite`].
    Gpio {
        /// Sequence number of the originating request.
        seq: u32,
        /// Pin that was accessed.
        pin: u32,
        /// Level read back, or the failure.
        result: Result<bool, ErrorCode>,
    },
    /// Result of an [`Effect::AdcRead`].
    Adc {
        /// Sequence number of the originating request.
        seq: u32,
        /// The reading, or the failure.
        result: Result<badge_proto::AdcLevel, ErrorCode>,
    },
    /// Result of an [`Effect::WifiScan`].
    WifiScanned {
        /// Sequence number of the originating request; `None` if started from the badge UI.
        seq: Option<u32>,
        /// Access points found, strongest first.
        result: Result<Vec<AccessPoint, MAX_APS>, ErrorCode>,
    },
    /// Wi-Fi link state, either answering [`Effect::WifiConnect`] /
    /// [`Effect::WifiStatus`] (`seq` set) or an unsolicited change (`seq` None).
    Wifi {
        /// Sequence number of the originating request, if any.
        seq: Option<u32>,
        /// Current state, or the failure.
        result: Result<WifiState, ErrorCode>,
    },
    /// Result of an [`Effect::BleScan`].
    BleScanned {
        /// Sequence number of the originating request; `None` if started from the badge UI.
        seq: Option<u32>,
        /// Devices heard, strongest first.
        result: Result<Vec<BleDevice, MAX_BLE>, ErrorCode>,
    },
    /// Result of an [`Effect::BleAdvertise`].
    BleAdvertised {
        /// Sequence number of the originating request.
        seq: u32,
        /// Success, or the failure.
        result: Result<(), ErrorCode>,
    },
    /// A stock-protocol packet: for BLE the bytes after the "OZ" company id
    /// (see [`oz::oz_payload`]), for the link UART a decoded frame payload.
    /// Signature checking is the core's job.
    RadioPacket {
        /// Transport it arrived on.
        via: Via,
        /// Signal strength in dBm (0 for the link UART).
        rssi: i8,
        /// Kind byte onward.
        payload: Vec<u8, { oz::MAX_PAYLOAD }>,
    },
    /// Once-a-second BLE air metrics from the continuous scan.
    BleStats(BleSnapshot),
    /// An LED sync beacon from another badge (ESP-NOW).
    SyncBeacon {
        /// Sender's sync id.
        from: u32,
        /// Sender's network time when it sent the beacon.
        network_ms: u32,
        /// Local time the beacon was received.
        rx_at: Millis,
    },
    /// Flash storage report.
    Storage(StorageEvent),
    /// Answer to an [`Effect::Wasm`] command from the wasm runtime.
    Wasm {
        /// Sequence number of the originating request.
        seq: u32,
        /// Runtime status after the command, or the failure.
        result: Result<badge_proto::WasmStatus, ErrorCode>,
    },
    /// A line the wasm app logged (`badge::log`); sent to the host as a
    /// `WasmLog` event.
    WasmLog(String<{ WASM_LOG_LEN }>),
}

/// Samples taken when `AdcRead.samples` is 0.
pub const DEFAULT_ADC_SAMPLES: u32 = 16;
/// Most samples one `AdcRead` may take.
pub const MAX_ADC_SAMPLES: u32 = 64;

/// Longest wasm log line carried through the core.
pub const WASM_LOG_LEN: usize = 64;
/// Largest wasm upload chunk (matches `WasmLoad.data`).
pub const WASM_CHUNK: usize = 192;

/// A wasm runtime command, executed by the firmware's wasm task (or the
/// simulator) and answered with [`Input::Wasm`].
// `Load` carries the chunk inline (no heap to box into); commands travel in
// fixed-capacity effect vectors, so the size is bounded and known.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, PartialEq)]
pub enum WasmCmd {
    /// Store one upload chunk; `offset` 0 starts a new upload.
    Load {
        /// Byte offset of `data` in the module.
        offset: u32,
        /// Full module size.
        total: u32,
        /// The chunk.
        data: Vec<u8, WASM_CHUNK>,
    },
    /// Instantiate the loaded module and run `init`.
    Start,
    /// Stop the running app (module stays loaded).
    Stop,
    /// Report status only.
    Status,
}

/// A primitive the shell draws directly when the host paints the screen.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DrawOp {
    /// Fill the whole panel.
    Fill(Color),
    /// Fill a rectangle (clipped to the panel by the shell).
    Rect {
        /// Left edge.
        x: u16,
        /// Top edge.
        y: u16,
        /// Width.
        w: u16,
        /// Height.
        h: u16,
        /// Fill colour.
        color: Color,
    },
}

/// Something the core wants the shell to do.
// `Send` dwarfs the other variants, but boxing needs a heap; effects live in a
// fixed-capacity `Effects` vector, so the size is bounded and known.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, PartialEq)]
pub enum Effect {
    /// Encode, frame and write this message to the host.
    Send(BadgeMsg),
    /// Write these colours to the WS2812 chain (already brightness-scaled).
    Leds([Color; crate::board::led::RGB_COUNT]),
    /// Switch an indicator LED.
    Indicator(Indicator, bool),
    /// The UI model changed; re-render the whole screen via `ui::render`.
    Redraw,
    /// Only the animated region of the current screen changed; re-render it
    /// via `ui::render_animation` (no full-screen clear, so no flicker). The
    /// shell may skip it when a `Redraw` is pending in the same tick.
    Animate,
    /// Draw a primitive on the panel (host canvas mode).
    Draw(DrawOp),
    /// Read a probe pin; answer with [`Input::Gpio`].
    GpioRead {
        /// Request sequence number.
        seq: u32,
        /// Pin, already checked against `board::PROBE_PINS`.
        pin: u32,
    },
    /// Sample a pin with ADC1; answer with [`Input::Adc`].
    AdcRead {
        /// Request sequence number.
        seq: u32,
        /// Pin, already checked against `board::ADC_PINS`.
        pin: u32,
        /// Number of samples, 1..=[`MAX_ADC_SAMPLES`].
        samples: u32,
    },
    /// Drive a probe pin; answer with [`Input::Gpio`].
    GpioWrite {
        /// Request sequence number.
        seq: u32,
        /// Pin, already checked against `board::PROBE_PINS`.
        pin: u32,
        /// Level to drive.
        level: bool,
    },
    /// Start a Wi-Fi scan; answer with [`Input::WifiScanned`].
    WifiScan {
        /// Request sequence number, `None` if started from the badge UI.
        seq: Option<u32>,
    },
    /// Join a network; answer with [`Input::Wifi`].
    WifiConnect {
        /// Request sequence number.
        seq: u32,
        /// Network name.
        ssid: String<32>,
        /// WPA2 passphrase, empty for open networks.
        password: String<64>,
    },
    /// Report link state; answer with [`Input::Wifi`].
    WifiStatus {
        /// Request sequence number.
        seq: u32,
    },
    /// Start a BLE scan; answer with [`Input::BleScanned`].
    BleScan {
        /// Request sequence number, `None` if started from the badge UI.
        seq: Option<u32>,
        /// Scan duration in milliseconds.
        duration_ms: u32,
    },
    /// Start (non-empty name) or stop (empty) advertising; answer with
    /// [`Input::BleAdvertised`].
    BleAdvertise {
        /// Request sequence number.
        seq: u32,
        /// Advertised local name.
        name: String<29>,
    },
    /// Replace the advertising data of our (single) BLE advertising set,
    /// without restarting it. Complete AD structures, ≤ 31 bytes.
    Beacon(Vec<u8, MAX_ADV_DATA>),
    /// Write this complete, already framed packet to the link UART.
    LinkSend(Vec<u8, { oz::link::MAX_FRAME }>),
    /// Broadcast an LED sync beacon over ESP-NOW.
    SyncBroadcast {
        /// Our sync id.
        id: u32,
        /// Our network time now.
        network_ms: u32,
    },
    /// Persist this contact (insert or replace by id).
    SaveContact(Contact),
    /// Erase all stored contacts.
    ClearContacts,
    /// Run a wasm runtime command; answer with [`Input::Wasm`].
    Wasm {
        /// Request sequence number.
        seq: u32,
        /// What to do.
        cmd: WasmCmd,
    },
}

/// Effects produced by one `App::handle` call, in execution order.
pub type Effects = Vec<Effect, MAX_EFFECTS>;
