//! Radio shell: Wi-Fi station (with `DHCPv4`) and Bluetooth LE, running side
//! by side under esp-radio coexistence.
//!
//! The rest of the firmware talks to the radios only through two static
//! channels:
//!
//! - [`RADIO_CMDS`] carries the radio [`Effect`]s the functional core emits
//!   (`WifiScan`, `WifiConnect`, `WifiStatus`, `BleScan`, `BleAdvertise`).
//!   `SyncBroadcast` goes to the Wi-Fi task's ESP-NOW sender and `Beacon`
//!   to the BLE task's advertising set; neither gets an answer. Any other
//!   effect is logged and dropped.
//! - [`RADIO_EVENTS`] carries the matching [`Input`]s back (`WifiScanned`,
//!   `Wifi`, `BleScanned`, `BleAdvertised`), plus unsolicited
//!   `Input::Wifi { seq: None, .. }` whenever the link or DHCP lease changes
//!   and unsolicited `Input::SyncBeacon` for received ESP-NOW sync beacons.
//!   The BLE task adds unsolicited `Input::RadioPacket` (OZ adverts),
//!   `Input::BleStats` (once a second) and `Input::Host` (GATT writes).
//!
//! Every command receives exactly one answer. A command that arrives while
//! its radio is busy with a long operation (scan, connect) is answered
//! immediately with `ErrorCode::Busy` (`WifiStatus` is answered with the
//! last known state instead).
//!
//! [`start`] spawns four tasks: a dispatcher that splits [`RADIO_CMDS`] by
//! radio, the Wi-Fi task ([`wifi`]), the embassy-net runner, and the BLE task
//! ([`ble`]). No credentials are compiled in; networks are joined only with
//! what `WifiConnect` supplies at runtime.

pub mod ble;
pub mod rank;
pub mod wifi;

use badge_core::io::{Effect, Input};
use bt_hci::controller::ExternalController;
use defmt::warn;
use embassy_executor::Spawner;
use embassy_net::StackResources;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use esp_radio::ble::controller::BleConnector;
use esp_radio::wifi::{ControllerConfig, Interface, WifiController};
use static_cell::StaticCell;

/// Radio effects to execute, sent by the main firmware.
///
/// Only `Effect::WifiScan`, `WifiConnect`, `WifiStatus`, `BleScan` and
/// `BleAdvertise` (plus `SyncBroadcast` and `Beacon`, unanswered) are acted upon;
/// anything else is logged and dropped.
pub static RADIO_CMDS: Channel<CriticalSectionRawMutex, Effect, 4> = Channel::new();

/// Radio results and unsolicited Wi-Fi link changes, for the main firmware.
///
/// The main firmware must keep draining this channel: radio tasks block on a
/// full channel rather than drop an answer. The BLE task's unsolicited
/// `RadioPacket` and `BleStats` inputs use `try_send` instead and are dropped
/// (and counted) when it is full, so the BLE host runner never blocks.
pub static RADIO_EVENTS: Channel<CriticalSectionRawMutex, Input, 4> = Channel::new();

/// Number of embassy-net sockets reserved (DHCP uses one; spare for DNS/UDP).
const NET_SOCKETS: usize = 3;

/// Static storage for the embassy-net stack (sockets, DHCP state).
static NET_RESOURCES: StaticCell<StackResources<NET_SOCKETS>> = StaticCell::new();

/// Why [`start`] could not bring up the radios.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum RadioError {
    /// [`start`] was called more than once.
    AlreadyStarted,
    /// The Wi-Fi driver refused to initialise.
    WifiInit(esp_radio::wifi::WifiError),
    /// The station network interface was already taken.
    WifiInterface,
    /// The BLE controller refused to initialise.
    BleInit(esp_radio::ble::controller::BleInitError),
    /// A radio task could not be spawned (its pool is already in use).
    Spawn,
}

/// Brings up Wi-Fi (station mode, not yet associated) and BLE, and spawns the
/// radio tasks.
///
/// Must be called once, after `esp_rtos::start` and after the `esp_alloc`
/// heap is set up (the radio drivers allocate from it).
///
/// # Errors
///
/// Returns [`RadioError`] if called twice, if a driver fails to initialise,
/// or if a task cannot be spawned.
pub fn start(
    spawner: &Spawner,
    wifi: esp_hal::peripherals::WIFI<'static>,
    bt: esp_hal::peripherals::BT<'static>,
) -> Result<(), RadioError> {
    let resources = NET_RESOURCES
        .try_init(StackResources::new())
        .ok_or(RadioError::AlreadyStarted)?;

    let controller =
        WifiController::new(wifi, ControllerConfig::default()).map_err(RadioError::WifiInit)?;
    let interface = Interface::try_station().ok_or(RadioError::WifiInterface)?;

    let rng = esp_hal::rng::Rng::new();
    let seed = (u64::from(rng.random()) << 32) | u64::from(rng.random());
    let (stack, runner) = embassy_net::new(
        interface,
        embassy_net::Config::dhcpv4(embassy_net::DhcpConfig::default()),
        resources,
        seed,
    );

    let connector =
        BleConnector::new(bt, esp_radio::ble::Config::default()).map_err(RadioError::BleInit)?;
    let ble_controller: ble::BleController = ExternalController::new(connector);

    spawner.spawn(net_task(runner).map_err(|_| RadioError::Spawn)?);
    spawner.spawn(wifi::wifi_task(controller, stack).map_err(|_| RadioError::Spawn)?);
    spawner.spawn(ble::ble_task(ble_controller).map_err(|_| RadioError::Spawn)?);
    spawner.spawn(dispatch_task().map_err(|_| RadioError::Spawn)?);
    Ok(())
}

/// Drives the embassy-net stack (DHCP, ARP, sockets) forever.
#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, Interface>) {
    runner.run().await;
}

/// Where a radio effect is executed.
enum Route {
    /// Handled by the Wi-Fi task.
    Wifi(wifi::WifiCmd),
    /// Handled by the BLE task.
    Ble(ble::BleCmd),
    /// An LED sync beacon `(id, network_ms)`, broadcast over ESP-NOW by the
    /// Wi-Fi task.
    Sync(u32, u32),
    /// Not a radio effect.
    Ignored,
}

/// Classifies an effect by the radio that executes it.
fn route(effect: Effect) -> Route {
    match effect {
        Effect::WifiScan { seq } => Route::Wifi(wifi::WifiCmd::Scan { seq }),
        Effect::WifiConnect {
            seq,
            ssid,
            password,
        } => Route::Wifi(wifi::WifiCmd::Connect {
            seq,
            ssid,
            password,
        }),
        Effect::WifiStatus { seq } => Route::Wifi(wifi::WifiCmd::Status { seq }),
        Effect::BleScan { seq, duration_ms } => Route::Ble(ble::BleCmd::Scan { seq, duration_ms }),
        Effect::BleAdvertise { seq, name } => Route::Ble(ble::BleCmd::Advertise { seq, name }),
        Effect::Beacon(data) => Route::Ble(ble::BleCmd::Beacon(data)),
        Effect::SyncBroadcast { id, network_ms } => Route::Sync(id, network_ms),
        _ => Route::Ignored,
    }
}

/// Splits [`RADIO_CMDS`] into the per-radio queues.
///
/// The radio tasks keep receiving even while busy (answering `Busy`), so the
/// forwarding `send`s only wait for a moment.
#[embassy_executor::task]
async fn dispatch_task() {
    loop {
        match route(RADIO_CMDS.receive().await) {
            Route::Wifi(cmd) => wifi::WIFI_CMDS.send(cmd).await,
            Route::Ble(cmd) => ble::BLE_CMDS.send(cmd).await,
            Route::Sync(id, network_ms) => wifi::queue_beacon(id, network_ms),
            Route::Ignored => warn!("radio: ignoring non-radio effect"),
        }
    }
}
