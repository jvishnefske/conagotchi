//! Wi-Fi station task: scanning, joining a network with runtime-supplied
//! credentials, `DHCPv4` via embassy-net, and link-state reporting.
//!
//! The task owns the [`WifiController`] outright and handles one
//! [`WifiCmd`] at a time. While a scan or connect is in flight it keeps
//! draining [`WIFI_CMDS`], answering further scans/connects with
//! `ErrorCode::Busy` and status requests with the last known [`Link`]. When
//! idle it polls the link once per [`POLL_INTERVAL`] and emits an unsolicited
//! `Input::Wifi { seq: None, .. }` when association or the DHCP lease changes.
//!
//! Every radio await is wrapped in a timeout; no credentials are compiled in.
//!
//! # LED sync over ESP-NOW (FR-31)
//!
//! The task also owns the ESP-NOW instance, obtained from the controller
//! with `WifiController::esp_now` (it shares the started station interface;
//! esp-radio pre-registers the broadcast peer). [`queue_beacon`] hands it the
//! latest `Effect::SyncBroadcast` through [`SYNC_TX`] (latest wins, never
//! blocks the dispatcher). The beacon's network time is advanced by the time
//! it spent queued, then it is broadcast to `FF:FF:FF:FF:FF:FF` as a
//! [`SyncBeacon`] (`OZL1` + id + network ms). Beacons are only sent while no
//! scan or connect is running; one older than [`MAX_TX_AGE`] is dropped.
//!
//! Received frames are served concurrently with the command loop and also
//! while a scan or connect is in flight, so each is stamped with the embassy
//! time at reception (esp-radio's RX timestamp is the Wi-Fi MAC's own
//! microsecond timer, not the embassy time base, so it is not used). Frames
//! from our own MAC or that do not decode are ignored; good ones become
//! `Input::SyncBeacon` via `try_send` on [`RADIO_EVENTS`], counting drops
//! when it is full instead of blocking.
//!
//! Channel: when not associated the radio is parked on [`SYNC_CHANNEL`]
//! (1) at start, after every scan and connect attempt, and by the
//! once-a-second poll if anything left it elsewhere. When associated the
//! radio follows the AP's channel, so badges on the same AP stay in sync.
//! Gaps: during a scan (up to [`SCAN_TIMEOUT`]) or connect (up to
//! [`CONNECT_TIMEOUT`]) the radio hops channels, so no beacons are sent and
//! few are heard; an associated badge only hears badges on its AP's channel.

use core::pin::pin;

use badge_core::io::{Input, MAX_APS, Millis};
use badge_core::sync::SyncBeacon;
use badge_proto::{AccessPoint, ErrorCode, WifiState};
use defmt::{info, warn};
use embassy_futures::select::{Either3, Either4, select3, select4};
use embassy_net::Stack;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use esp_radio::esp_now::{BROADCAST_ADDRESS, EspNowReceiver, EspNowSender, ReceivedData};
use esp_radio::wifi::scan::ScanConfig;
use esp_radio::wifi::sta::StationConfig;
use esp_radio::wifi::{
    AuthenticationMethod, AuthenticationMethodConfig, Config, Password, SecondaryChannel, Ssid,
    WifiController,
};
use heapless::{String, Vec};

use super::RADIO_EVENTS;
use super::rank::{rank_aps, truncate_str};

/// Wi-Fi commands, split out of `Effect` by the dispatcher.
#[derive(Debug, Clone, PartialEq)]
pub enum WifiCmd {
    /// Scan for access points; answered with `Input::WifiScanned`.
    Scan {
        /// Request sequence number, `None` if started from the badge UI.
        seq: Option<u32>,
    },
    /// Join a network; answered with `Input::Wifi`.
    Connect {
        /// Request sequence number.
        seq: u32,
        /// Network name.
        ssid: String<32>,
        /// WPA2 passphrase (8..=64 bytes), empty for an open network.
        password: String<64>,
    },
    /// Report link state; answered with `Input::Wifi`.
    Status {
        /// Request sequence number.
        seq: u32,
    },
}

/// Queue from the dispatcher to the Wi-Fi task.
pub static WIFI_CMDS: Channel<CriticalSectionRawMutex, WifiCmd, 2> = Channel::new();

/// Upper bound on one scan, including coex airtime sharing with BLE.
const SCAN_TIMEOUT: Duration = Duration::from_secs(10);
/// Association plus DHCP lease must complete within this time.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(15);
/// Upper bound on tearing down an association.
const DISCONNECT_TIMEOUT: Duration = Duration::from_secs(3);
/// Upper bound on embassy-net noticing the old lease is gone.
const LEASE_DROP_TIMEOUT: Duration = Duration::from_secs(1);
/// How often the idle task samples link state for unsolicited reports.
pub const POLL_INTERVAL: Duration = Duration::from_secs(1);
/// Maximum raw scan records pulled from the driver before ranking.
const MAX_RAW_APS: usize = 32;
/// Minimum WPA2 passphrase length (IEEE 802.11i).
const MIN_PASSPHRASE: usize = 8;
/// Channel the radio sits on while not associated, for ESP-NOW sync.
pub const SYNC_CHANNEL: u8 = 1;
/// A queued sync beacon older than this is dropped instead of sent.
pub const MAX_TX_AGE: Duration = Duration::from_millis(100);
/// Upper bound on one ESP-NOW broadcast (send callback included).
const SYNC_SEND_TIMEOUT: Duration = Duration::from_millis(50);
/// Log sync counters every this many received beacons.
const SYNC_LOG_EVERY: u32 = 256;

/// A sync beacon waiting to be broadcast.
#[derive(Debug, Clone, Copy)]
pub struct Outgoing {
    /// The beacon as the core emitted it.
    beacon: SyncBeacon,
    /// When it was handed to the Wi-Fi task.
    queued_at: Instant,
}

/// Latest sync beacon to broadcast (a newer one replaces an unsent one).
pub static SYNC_TX: Signal<CriticalSectionRawMutex, Outgoing> = Signal::new();

/// Queues `Effect::SyncBroadcast { id, network_ms }` for broadcast. Never
/// blocks.
pub fn queue_beacon(id: u32, network_ms: u32) {
    SYNC_TX.signal(Outgoing {
        beacon: SyncBeacon { id, network_ms },
        queued_at: Instant::now(),
    });
}

/// ESP-NOW sync counters (all wrap).
#[derive(Debug, Clone, Copy, Default, defmt::Format)]
struct SyncStats {
    /// Beacons forwarded to the core.
    rx: u32,
    /// Beacons dropped because [`RADIO_EVENTS`] was full.
    rx_dropped: u32,
    /// Frames that were not sync beacons.
    rx_bad: u32,
    /// Beacons broadcast.
    tx: u32,
    /// Broadcasts that failed, timed out or were too old to send.
    tx_failed: u32,
}

/// The ESP-NOW half of the Wi-Fi task.
struct SyncLink {
    sender: EspNowSender,
    receiver: EspNowReceiver,
    /// Our station MAC, to ignore our own frames.
    own_mac: [u8; 6],
    stats: SyncStats,
}

impl SyncLink {
    /// Forwards one received frame to the core if it is a beacon from
    /// someone else; `rx_at` is the embassy time it was taken off the queue.
    fn received(&mut self, frame: &ReceivedData, rx_at: Millis) {
        if frame.info.src_address == self.own_mac {
            return;
        }
        let Ok(beacon) = SyncBeacon::decode(frame.data()) else {
            self.stats.rx_bad = self.stats.rx_bad.wrapping_add(1);
            return;
        };
        let input = Input::SyncBeacon {
            from: beacon.id,
            network_ms: beacon.network_ms,
            rx_at,
        };
        if RADIO_EVENTS.try_send(input).is_ok() {
            self.stats.rx = self.stats.rx.wrapping_add(1);
            if self.stats.rx.is_multiple_of(SYNC_LOG_EVERY) {
                info!("sync: {}", self.stats);
            }
        } else {
            self.stats.rx_dropped = self.stats.rx_dropped.wrapping_add(1);
            warn!(
                "sync: event queue full, beacon dropped ({})",
                self.stats.rx_dropped
            );
        }
    }

    /// Broadcasts `out`, its network time advanced by its queueing age.
    async fn send(&mut self, out: Outgoing) {
        let age = out.queued_at.elapsed();
        if age > MAX_TX_AGE {
            self.stats.tx_failed = self.stats.tx_failed.wrapping_add(1);
            return;
        }
        let age_ms = u32::try_from(age.as_millis()).unwrap_or(u32::MAX);
        let wire = SyncBeacon {
            network_ms: out.beacon.network_ms.wrapping_add(age_ms),
            ..out.beacon
        }
        .encode();
        // On timeout the send future is dropped before its callback; the
        // next send then resets the callback flag, so at worst one status is
        // misattributed.
        match with_timeout(
            SYNC_SEND_TIMEOUT,
            self.sender.send_async(&BROADCAST_ADDRESS, &wire),
        )
        .await
        {
            Ok(Ok(())) => self.stats.tx = self.stats.tx.wrapping_add(1),
            _ => self.stats.tx_failed = self.stats.tx_failed.wrapping_add(1),
        }
    }
}

/// Embassy time now, as the core's [`Millis`].
fn now_millis() -> Millis {
    Millis(Instant::now().as_millis())
}

/// Parks the radio on [`SYNC_CHANNEL`] unless associated (then the AP's
/// channel rules) or already there.
fn park_channel(controller: &mut WifiController<'static>) {
    if controller.is_connected() {
        return;
    }
    if matches!(controller.channel(), Ok((ch, _)) if ch == SYNC_CHANNEL) {
        return;
    }
    if let Err(e) = controller.set_channel(SYNC_CHANNEL, SecondaryChannel::None) {
        warn!("wifi: cannot park on channel {}: {}", SYNC_CHANNEL, e);
    }
}

/// The Wi-Fi task's view of the station link. Immutable: every observation
/// produces a fresh value.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Link {
    /// Associated with an access point.
    pub connected: bool,
    /// SSID most recently requested via `WifiConnect` (empty before the first).
    pub ssid: String<32>,
    /// DHCP-assigned IPv4 address, if a lease is held.
    pub ipv4: Option<[u8; 4]>,
    /// Last sampled RSSI in dBm (0 when not associated).
    pub rssi: i32,
}

impl Link {
    /// Converts to the protocol representation (`ipv4` empty without a lease).
    pub fn to_proto(&self) -> WifiState {
        WifiState {
            connected: self.connected,
            ssid: self.ssid.clone(),
            ipv4: self
                .ipv4
                .map(|ip| Vec::from_slice(&ip).unwrap_or_default())
                .unwrap_or_default(),
            rssi: self.rssi,
        }
    }

    /// True when association or lease differ (RSSI jitter is not a change).
    fn differs_from(&self, other: &Link) -> bool {
        self.connected != other.connected || self.ipv4 != other.ipv4
    }
}

/// Samples the current link state from the driver and the network stack.
fn observe(controller: &WifiController<'static>, stack: Stack<'static>, prev: &Link) -> Link {
    let connected = controller.is_connected();
    Link {
        connected,
        ssid: prev.ssid.clone(),
        ipv4: if connected {
            stack.config_v4().map(|c| c.address.address().octets())
        } else {
            None
        },
        rssi: if connected {
            controller.rssi().unwrap_or(prev.rssi)
        } else {
            0
        },
    }
}

/// Owns the Wi-Fi controller and the ESP-NOW instance, and serves
/// [`WIFI_CMDS`], [`SYNC_TX`] and received sync beacons forever.
///
/// Creating the ESP-NOW instance panics only if one already exists, which
/// cannot happen: this task is its only creator and its pool size is one.
#[embassy_executor::task]
pub async fn wifi_task(mut controller: WifiController<'static>, stack: Stack<'static>) {
    let (_manager, sender, receiver) = controller.esp_now().split();
    let mut sync = SyncLink {
        sender,
        receiver,
        own_mac: own_mac(),
        stats: SyncStats::default(),
    };
    park_channel(&mut controller);
    let mut link = Link::default();
    let mut next_poll = Instant::now() + POLL_INTERVAL;
    loop {
        let event = select4(
            WIFI_CMDS.receive(),
            Timer::at(next_poll),
            sync.receiver.receive_async(),
            SYNC_TX.wait(),
        )
        .await;
        match event {
            Either4::First(cmd) => {
                let (answer, next) = execute(cmd, &mut controller, stack, &link, &mut sync).await;
                link = next;
                park_channel(&mut controller);
                RADIO_EVENTS.send(answer).await;
                next_poll = Instant::now() + POLL_INTERVAL;
            }
            Either4::Third(frame) => sync.received(&frame, now_millis()),
            Either4::Fourth(out) => sync.send(out).await,
            Either4::Second(()) => {
                next_poll = Instant::now() + POLL_INTERVAL;
                park_channel(&mut controller);
                let now = observe(&controller, stack, &link);
                if now.differs_from(&link) {
                    info!(
                        "wifi: link change connected={} ip={}",
                        now.connected,
                        now.ipv4.is_some()
                    );
                    RADIO_EVENTS
                        .send(Input::Wifi {
                            seq: None,
                            result: Ok(now.to_proto()),
                        })
                        .await;
                }
                link = now;
            }
        }
    }
}

/// Runs one command to completion and returns its answer plus the new link.
async fn execute(
    cmd: WifiCmd,
    controller: &mut WifiController<'static>,
    stack: Stack<'static>,
    link: &Link,
    sync: &mut SyncLink,
) -> (Input, Link) {
    match cmd {
        WifiCmd::Scan { seq } => {
            let result = while_busy(scan(controller), link, sync).await;
            if let Err(e) = &result {
                warn!("wifi: scan failed ({})", e.0);
            }
            (Input::WifiScanned { seq, result }, link.clone())
        }
        WifiCmd::Connect {
            seq,
            ssid,
            password,
        } => {
            info!("wifi: connecting");
            let outcome =
                while_busy(connect(controller, stack, &ssid, &password), link, sync).await;
            let next = match &outcome {
                Ok(l) => l.clone(),
                Err(_) => observe(
                    controller,
                    stack,
                    &Link {
                        ssid,
                        ..Link::default()
                    },
                ),
            };
            let result = outcome.map(|l| l.to_proto());
            (
                Input::Wifi {
                    seq: Some(seq),
                    result,
                },
                next,
            )
        }
        WifiCmd::Status { seq } => {
            let now = observe(controller, stack, link);
            (
                Input::Wifi {
                    seq: Some(seq),
                    result: Ok(now.to_proto()),
                },
                now,
            )
        }
    }
}

/// Polls `op` to completion while answering any command that arrives in the
/// meantime: scans/connects get `Busy`, status gets the last known `link`.
/// Received sync beacons keep being stamped and forwarded; none are sent.
///
/// Terminates when `op` does; callers bound `op` with a timeout.
async fn while_busy<F: Future>(op: F, link: &Link, sync: &mut SyncLink) -> F::Output {
    let mut op = pin!(op);
    loop {
        match select3(
            op.as_mut(),
            WIFI_CMDS.receive(),
            sync.receiver.receive_async(),
        )
        .await
        {
            Either3::First(out) => return out,
            Either3::Second(cmd) => RADIO_EVENTS.send(busy_reply(&cmd, link)).await,
            Either3::Third(frame) => sync.received(&frame, now_millis()),
        }
    }
}

/// Our station MAC (the eFuse base MAC on the ESP32-S3).
fn own_mac() -> [u8; 6] {
    let mut mac = [0u8; 6];
    for (dst, src) in mac
        .iter_mut()
        .zip(esp_hal::efuse::base_mac_address().as_bytes())
    {
        *dst = *src;
    }
    mac
}

/// The answer to `cmd` while the Wi-Fi task is occupied.
fn busy_reply(cmd: &WifiCmd, link: &Link) -> Input {
    match *cmd {
        WifiCmd::Scan { seq } => Input::WifiScanned {
            seq,
            result: Err(ErrorCode::Busy),
        },
        WifiCmd::Connect { seq, .. } => Input::Wifi {
            seq: Some(seq),
            result: Err(ErrorCode::Busy),
        },
        WifiCmd::Status { seq } => Input::Wifi {
            seq: Some(seq),
            result: Ok(link.to_proto()),
        },
    }
}

/// Scans all channels and returns the strongest [`MAX_APS`] access points.
async fn scan(
    controller: &mut WifiController<'static>,
) -> Result<Vec<AccessPoint, MAX_APS>, ErrorCode> {
    let config = ScanConfig::default().with_max(MAX_RAW_APS);
    let records = with_timeout(SCAN_TIMEOUT, controller.scan_async(&config))
        .await
        .map_err(|_| ErrorCode::Radio)?
        .map_err(|_| ErrorCode::Radio)?;
    // `records` is allocated by esp-radio on its heap; it is only read here.
    Ok(rank_aps(
        records.iter().map(|r| AccessPoint {
            ssid: truncate_str(r.ssid.as_str()),
            bssid: Vec::from_slice(&r.bssid).unwrap_or_default(),
            rssi: i32::from(r.signal_strength),
            channel: u32::from(r.channel),
            auth: r.auth_method.map_or(0, auth_code),
        }),
        MAX_RAW_APS,
    ))
}

/// Protocol code for an authentication method: the esp-radio
/// `AuthenticationMethod` declaration index (0 = open). Unknown future
/// variants map to `u32::MAX`.
pub fn auth_code(method: AuthenticationMethod) -> u32 {
    match method {
        AuthenticationMethod::None => 0,
        AuthenticationMethod::Wep => 1,
        AuthenticationMethod::Wpa => 2,
        AuthenticationMethod::Wpa2Personal => 3,
        AuthenticationMethod::WpaWpa2Personal => 4,
        AuthenticationMethod::Wpa2Enterprise => 5,
        AuthenticationMethod::Wpa3Personal => 6,
        AuthenticationMethod::Wpa2Wpa3Personal => 7,
        AuthenticationMethod::WapiPersonal => 8,
        AuthenticationMethod::Owe => 9,
        AuthenticationMethod::Wpa3EntSuiteB192Bit => 10,
        AuthenticationMethod::Wpa3ExtPsk => 11,
        AuthenticationMethod::Wpa3ExtPskMixed => 12,
        AuthenticationMethod::Dpp => 13,
        AuthenticationMethod::Wpa3Enterprise => 14,
        AuthenticationMethod::Wpa2Wpa3Enterprise => 15,
        AuthenticationMethod::WpaEnterprise => 16,
        _ => u32::MAX,
    }
}

/// Builds the authentication config: open for an empty password, WPA2
/// Personal otherwise (the driver treats it as a minimum, so WPA2/WPA3
/// transition networks also work).
fn auth_config(password: &str) -> Result<AuthenticationMethodConfig, ErrorCode> {
    if password.is_empty() {
        return Ok(AuthenticationMethodConfig::Open);
    }
    if password.len() < MIN_PASSPHRASE {
        return Err(ErrorCode::BadArgument);
    }
    let password = Password::try_from(password).map_err(|_| ErrorCode::BadArgument)?;
    Ok(AuthenticationMethodConfig::Wpa2Personal(password))
}

/// Drops any current association and waits (bounded) for the old lease to go.
async fn disconnect(controller: &mut WifiController<'static>, stack: Stack<'static>) {
    // `NotConnected` (already idle) and timeouts are both fine here.
    let _ = with_timeout(DISCONNECT_TIMEOUT, controller.disconnect_async()).await;
    let _ = with_timeout(LEASE_DROP_TIMEOUT, stack.wait_config_down()).await;
}

/// Joins `ssid` and waits for a DHCP lease, all within [`CONNECT_TIMEOUT`].
///
/// On failure the half-open association is torn down and
/// `ErrorCode::Radio` (or `BadArgument` for unusable credentials) returned.
async fn connect(
    controller: &mut WifiController<'static>,
    stack: Stack<'static>,
    ssid: &String<32>,
    password: &String<64>,
) -> Result<Link, ErrorCode> {
    let auth = auth_config(password.as_str())?;
    let target = Ssid::try_from(ssid.as_str()).map_err(|_| ErrorCode::BadArgument)?;
    if ssid.is_empty() {
        return Err(ErrorCode::BadArgument);
    }

    disconnect(controller, stack).await;
    let config = Config::Station(
        StationConfig::default()
            .with_ssid(target)
            .with_authentication(auth),
    );
    controller
        .set_config(&config)
        .map_err(|_| ErrorCode::Radio)?;

    let attempt = with_timeout(CONNECT_TIMEOUT, async {
        controller.connect_async().await.map_err(|_| ())?;
        stack.wait_config_up().await;
        Ok::<(), ()>(())
    })
    .await;

    if let Ok(Ok(())) = attempt {
        let link = observe(
            controller,
            stack,
            &Link {
                ssid: ssid.clone(),
                ..Link::default()
            },
        );
        info!("wifi: connected, lease={}", link.ipv4.is_some());
        Ok(link)
    } else {
        warn!("wifi: connect failed or timed out");
        disconnect(controller, stack).await;
        Err(ErrorCode::Radio)
    }
}
