//! Bluetooth LE task: one advertising set, a continuous passive scan, the
//! air-metrics collector and the GATT service "Conagotchi", using the
//! trouble-host stack over esp-radio's HCI connector.
//!
//! The task owns the BLE host. The host runner and the command loop run
//! concurrently inside the task (`select`); advertising reports reach the
//! command loop through a [`Collector`] that the runner calls for every HCI
//! advertising-report event. All decisions are made by pure badge-core code
//! (`ble_adv`, `ble_metrics`, `gatt`, `radio_rank`); this module only moves
//! bytes.
//!
//! # Advertising
//!
//! One legacy connectable-scannable (`ADV_IND`) set, interval
//! [`ADV_INTERVAL`] (100 ms, like the stock badges). It is started with
//! `Peripheral::advertise` and Flags-only data (trouble requires Flags first
//! in connectable data), then immediately given the real contents with
//! `Peripheral::update_adv_data`, which issues `LE Set Advertising Data` and
//! `LE Set Scan Response Data` without restarting the set. The update passes
//! `Advertisement::NonconnectableScannableUndirected` only so trouble skips
//! its Flags check: the PDU type stays the one set at start. What the
//! ADV_IND carries is decided by the pure `badge_mw::ble::mux::AdvMux`
//! (FR-38): the latest `Effect::Beacon` data (Flags only before the first
//! beacon), except for one 300 ms slot per queued middleware advert, after
//! which the beacon is back for at least 700 ms (beacon >= 70 % of the time,
//! never absent for more than one slot). The multiplexer is stepped when a
//! beacon or publish arrives, at its own deadline (a timer in the command
//! loop) and every housekeeping period; whenever its `on_air` data differs
//! from what was last installed the set is updated in place. The scan
//! response carries the Complete Local Name, `"conagotchi"` from boot.
//! `BleAdvertise` changes the name in place (one `BleAdvertised` reply
//! each); an empty name stops the set, and a later non-empty name starts it
//! again.
//!
//! # Middleware advertising pub/sub
//!
//! App publishes drained from [`mw::BLE_OUT`] go through the middleware's
//! `AdvertisingPublisher` (kept for the life of the task, so its sequence
//! number advances) over `badge_mw::ble::advert::MuxAdvertiser`, which only
//! queues them in the multiplexer: manufacturer data under company id
//! `badge_mw::ble::MW_COMPANY_ID` (`0xFFFF`), `[short id][seq][payload]`,
//! payloads up to 21 bytes. A payload too large or a full queue drops the
//! message (counted). The key is also learned by the app subscriber below.
//!
//! Every scan report that is not an OZ beacon is pushed into the
//! `badge_mw::ble::advert::ScanQueue` of the middleware's
//! `AdvertisingSubscriber` (built by `badge_mw::ble::advert::app_subscriber`:
//! company id `0xFFFF`, filter `Prefix("app")`), which is then drained with
//! `try_recv` inside the runner's callback. Its `ShortIdTable` maps the
//! advert's short id back to an `app/...` key text it has learned (the
//! preloaded `KNOWN_APP_KEYS`, plus every key the app has published on; a key
//! whose short id collides with a learned one is refused and counted), and
//! it drops repeats per (sender, key). A fresh sample on a key that passes
//! `app_may_use` goes to [`mw::APP_SAMPLES`] with `try_send` (dropped and
//! counted when full).
//!
//! # Scanning
//!
//! A passive scan with interval = window = [`SCAN_WINDOW`] (30 ms, 100 %
//! duty, matching the stock badges' `gap_scan(0, 30000, 30000, False)`)
//! starts once and runs for the life of the task, concurrently with
//! advertising (trouble keeps separate command states for scanning and
//! advertising, and the controller interleaves them). Every report:
//!
//! - feeds `ble_metrics::BleMetrics`; a snapshot goes to the app as
//!   `Input::BleStats` once a second;
//! - while a `BleScan` request is open, is merged into its device table; the
//!   request is answered from that table after its (clamped) duration, with
//!   exactly one `BleScanned`;
//! - if it carries an OZ manufacturer AD, passes through
//!   `ble_adv::ForwardFilter` (changed payload, or 250 ms since the last
//!   forward for that address) and is sent as `Input::RadioPacket`.
//!
//! The runner never blocks on [`RADIO_EVENTS`]: packets and stats use
//! `try_send` and count drops. Command replies use `send`, which only waits
//! in the command loop while the runner keeps running.
//!
//! # GATT
//!
//! The service UUIDs and value formats are documented in `badge_core::gatt`.
//! Reads of info, pet mode and LED pattern are refreshed from the PROBE block
//! just before the attribute server answers. Writes are converted with
//! `gatt::write_to_host_msg` and pushed into [`RADIO_EVENTS`] as
//! `Input::Host` with `GATT_SEQ`; a bad length or value is rejected with the
//! matching ATT error. One connection at most: when a central connects the
//! controller stops advertising, the link is dropped after [`IDLE_TIMEOUT`]
//! without any ATT request or link event, or after [`MAX_CONNECTION`] in any
//! case (traffic cannot extend it, so a central cannot keep the badge off the
//! air), and advertising resumes within one [`HOUSEKEEPING`] period after the
//! disconnect.
//!
//! # Middleware GATT RPC
//!
//! The GATT server also hosts the middleware service
//! (`badge_mw::ble::rpc::SERVICE_UUID`, `4d579a3e-1c52-4b0f-0000-000000000000`)
//! with its RPC characteristic (`4d579a3e-1c52-4b0f-0000-000000000001`,
//! write, write without response, notify, values up to [`RPC_VALUE`] bytes).
//! A write is copied, with the connection's ATT MTU, into a task-local
//! channel, as are the central's connect and disconnect; an RPC loop running
//! beside the command loop reads them through `badge_mw::ble::rpc::RpcIo`
//! (lifecycle events reach the middleware as `Inbound::Connected` /
//! `Disconnected`, which discard a partially received request), runs the
//! middleware `BleServer`'s `serve_one` (fragment buffers of [`RPC_VALUE`]
//! bytes) with `badge_mw::ble::rpc::BadgeHandler` (answers from the PROBE
//! block at the moment of the request) and queues each response fragment
//! (at most `min(MTU - 3, RPC_VALUE)` bytes) on a second channel, which the command loop sends as notifications to the connected
//! central (only after it enabled them). Sets (`conagotchi/pet`,
//! `conagotchi/pattern` with a one-byte payload) are drained from the
//! handler's outbox into [`RADIO_EVENTS`] as `Input::Host` with `GATT_SEQ`,
//! like the Conagotchi service's writes. A write arriving while the inbound
//! channel is full is rejected (Insufficient Resources), and a lifecycle
//! event is dropped (the next request's first fragment still replaces a
//! stale partial one); fragments queued for notification while no central is
//! connected are discarded.
//!
//! If the host runner ever fails, the task keeps answering every BLE command
//! with `ErrorCode::Radio` so each command still gets exactly one reply.

use core::cell::{Cell, RefCell};

use badge_core::ble_adv::{self, DEFAULT_NAME, ForwardFilter};
use badge_core::ble_metrics::BleMetrics;
use badge_core::gatt::{self, GattError, Writable};
use badge_core::io::{Input, MAX_ADV_DATA, MAX_BLE, Millis, Via};
use badge_core::{oz, probe_layout};
use badge_mw::ble::advert::{AppSubscriber, MuxAdvertiser, ScanQueue, app_subscriber, mw_template};
use badge_mw::ble::rpc::{
    self as mw_rpc, BadgeHandler, BadgeSnapshot, RpcEvent, RpcIo, RpcServer, SnapshotSource,
};
use badge_mw::ble::{APP_KEY_LEN, AdvertisingPublisher, AttMtu, MAX_ADVERT_PAYLOAD};
use badge_mw::keys::app_may_use;
use badge_mw::{Key, Publisher, Responder, Subscriber};
use badge_proto::{BleDevice, ErrorCode};
use defmt::{Debug2Format, info, warn};
use embassy_futures::select::{Either, Either3, select, select3};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Ticker, Timer, with_timeout};
use esp_radio::ble::controller::BleConnector;
use heapless::{String, Vec};
use trouble_host::prelude::*;

use super::RADIO_EVENTS;
use super::rank::{AdvReport, merge_report, rank_devices};
use crate::{mw, probe};

/// The HCI controller type the BLE host runs on.
pub type BleController = ExternalController<BleConnector<'static>, 1>;

/// BLE commands, split out of `Effect` by the dispatcher.
#[derive(Debug, Clone, PartialEq)]
pub enum BleCmd {
    /// Report the devices heard for a while; answered with `Input::BleScanned`.
    Scan {
        /// Request sequence number, `None` if started from the badge UI.
        seq: Option<u32>,
        /// Requested duration, clamped to [`MIN_SCAN_MS`]..=[`MAX_SCAN_MS`].
        duration_ms: u32,
    },
    /// Set the advertised name (non-empty) or stop advertising (empty);
    /// answered with `Input::BleAdvertised`.
    Advertise {
        /// Request sequence number.
        seq: u32,
        /// Local name for the scan response.
        name: String<29>,
    },
    /// Replace the advertising data in place; never answered.
    Beacon(Vec<u8, MAX_ADV_DATA>),
}

/// Queue from the dispatcher to the BLE task.
pub static BLE_CMDS: Channel<CriticalSectionRawMutex, BleCmd, 4> = Channel::new();

/// Shortest scan honoured.
pub const MIN_SCAN_MS: u32 = 100;
/// Longest scan honoured.
pub const MAX_SCAN_MS: u32 = 30_000;
/// Advertising interval (min = max), as on the stock badges.
pub const ADV_INTERVAL: Duration = Duration::from_millis(100);
/// Passive scan interval and window (100 % duty cycle).
pub const SCAN_WINDOW: Duration = Duration::from_millis(30);
/// Period of the command loop's housekeeping (connection accept, scan
/// deadline, stats, idle check, advertising restart).
pub const HOUSEKEEPING: Duration = Duration::from_millis(100);
/// A GATT link without ATT traffic for this long is dropped.
pub const IDLE_TIMEOUT: Duration = Duration::from_secs(60);
/// A GATT link is dropped this long after it was accepted, whatever its
/// traffic: while connected the badge does not advertise (no trade beacon).
pub const MAX_CONNECTION: Duration = Duration::from_secs(120);
/// Interval between `Input::BleStats` snapshots.
const STATS_PERIOD: Duration = Duration::from_secs(1);
/// Delay before retrying a failed advertising start.
const ADV_RETRY: Duration = Duration::from_secs(5);
/// Upper bound on any single HCI command sequence.
const HCI_TIMEOUT: Duration = Duration::from_secs(2);
/// Attempts to start the continuous scan at boot.
const SCAN_START_ATTEMPTS: usize = 3;
/// Distinct advertisers tracked during one `BleScan` before the weakest is evicted.
const MAX_TRACKED: usize = 32;
/// Reports processed per HCI event (an event carries at most a handful).
const MAX_REPORTS_PER_EVENT: usize = 16;
/// Concurrent BLE connections the host reserves storage for.
const CONNECTIONS_MAX: usize = 1;
/// L2CAP channels the host reserves storage for (signalling + ATT).
const L2CAP_CHANNELS_MAX: usize = 2;
/// Longest GATT write we look at (the longest valid one is 3 bytes).
const MAX_WRITE: usize = 4;
/// Largest value of the middleware RPC characteristic (one fragment).
pub const RPC_VALUE: usize = 64;
/// RPC fragments buffered from the central to the RPC loop.
const RPC_RX_DEPTH: usize = 4;
/// RPC response fragments buffered from the RPC loop to the central.
const RPC_TX_DEPTH: usize = 2;
/// Scan reports the app subscriber queues (drained after every push).
const APP_SCAN_QUEUE: usize = 2;
/// Room for one app sample: its key text plus an advert payload.
const APP_SAMPLE_BUF: usize = APP_KEY_LEN + MAX_ADVERT_PAYLOAD;

const _: () = assert!(badge_mw::ble::APP_KEY_LEN == mw::KEY_LEN);
const _: () = assert!(badge_mw::ble::MAX_ADVERT_PAYLOAD <= mw::PAYLOAD_LEN);

type Host<'a> = Stack<'a, BleController, DefaultPacketPool>;
type Adv<'a> = Advertiser<'a, BleController, DefaultPacketPool>;
type Gatt<'a, 's> = GattConnection<'a, 's, DefaultPacketPool>;
/// The middleware advertising publisher over the multiplexer.
type MwPublisher = AdvertisingPublisher<'static, MuxAdvertiser>;

/// One RPC characteristic event, from the command loop to the RPC loop.
enum RpcIn {
    /// A write: the value and the connection's ATT MTU.
    Write { mtu: u16, data: Vec<u8, RPC_VALUE> },
    /// A central connected.
    Connected,
    /// The central disconnected.
    Disconnected,
}

/// Writes and lifecycle events from the central to the RPC loop.
type RpcRx = Channel<NoopRawMutex, RpcIn, RPC_RX_DEPTH>;
/// Response fragments from the RPC loop to the central.
type RpcTx = Channel<NoopRawMutex, Vec<u8, RPC_VALUE>, RPC_TX_DEPTH>;

/// The GATT server: GAP, the "Conagotchi" service and the middleware service.
#[gatt_server(connections_max = CONNECTIONS_MAX, mutex_type = NoopRawMutex)]
struct Server {
    badge: BadgeService,
    mw: MwService,
}

/// The middleware GATT service: one RPC characteristic carrying request
/// fragments (writes) and response fragments (notifications).
#[gatt_service(uuid = mw_rpc::SERVICE_UUID)]
struct MwService {
    /// Middleware RPC fragments.
    #[characteristic(uuid = mw_rpc::RPC_CHARACTERISTIC_UUID, write, write_without_response, notify)]
    rpc: Vec<u8, RPC_VALUE>,
}

/// Service "Conagotchi" (UUIDs and formats: `badge_core::gatt`).
#[gatt_service(uuid = gatt::SERVICE_UUID)]
struct BadgeService {
    /// Firmware, badge id and Chi as text.
    #[characteristic(uuid = gatt::INFO_UUID, read)]
    info: Vec<u8, { gatt::INFO_LEN }>,
    /// Selected `PetMode`.
    #[characteristic(uuid = gatt::PET_MODE_UUID, read, write)]
    pet_mode: u8,
    /// Current `LedPattern`.
    #[characteristic(uuid = gatt::LED_PATTERN_UUID, read, write)]
    led_pattern: u8,
    /// Colour for all LEDs (R, G, B).
    #[characteristic(uuid = gatt::LED_COLOR_UUID, write)]
    led_color: [u8; 3],
}

/// Sees every advertising report (called by the host runner).
///
/// Lives on the BLE task's stack and is shared by reference between the
/// runner and the command loop. Both run in the same task and no borrow is
/// held across an await.
struct Collector {
    metrics: RefCell<BleMetrics>,
    forward: RefCell<ForwardFilter>,
    /// Devices heard since the open `BleScan` began.
    seen: RefCell<Vec<BleDevice, MAX_TRACKED>>,
    /// True while a `BleScan` request is open.
    scanning: Cell<bool>,
    /// `RadioPacket`s forwarded / dropped because [`RADIO_EVENTS`] was full.
    forwarded: Cell<u32>,
    dropped: Cell<u32>,
    /// The middleware subscriber for app keys (over a scan report queue).
    app_sub: RefCell<AppSubscriber<APP_SCAN_QUEUE>>,
    /// App samples delivered to / dropped because [`mw::APP_SAMPLES`] was full
    /// or the subscriber failed.
    app_rx: Cell<u32>,
    app_dropped: Cell<u32>,
    /// App keys not learned (table full or short id collision).
    app_unlearned: Cell<u32>,
}

impl Collector {
    const fn new() -> Self {
        Self {
            metrics: RefCell::new(BleMetrics::new()),
            forward: RefCell::new(ForwardFilter::new()),
            seen: RefCell::new(Vec::new()),
            scanning: Cell::new(false),
            forwarded: Cell::new(0),
            dropped: Cell::new(0),
            app_sub: RefCell::new(AppSubscriber::new(ScanQueue::new())),
            app_rx: Cell::new(0),
            app_dropped: Cell::new(0),
            app_unlearned: Cell::new(0),
        }
    }

    /// Installs the app subscriber preset (app filter, known keys); not
    /// `const`, so not done in `new`.
    fn load_known_keys(&self) {
        match app_subscriber() {
            Ok(sub) => {
                if let Ok(mut slot) = self.app_sub.try_borrow_mut() {
                    *slot = sub;
                }
            }
            Err(e) => warn!("ble: app subscriber preset refused: {}", Debug2Format(&e)),
        }
    }

    /// Lets adverts on `key` reach the app. A full table or a short id
    /// collision keeps the learned keys (counted).
    fn learn_key(&self, key: Key<'_>) {
        let learned = self
            .app_sub
            .try_borrow_mut()
            .is_ok_and(|mut sub| sub.learn(key).is_ok());
        if !learned {
            self.app_unlearned
                .set(self.app_unlearned.get().wrapping_add(1));
        }
    }

    /// Feeds a non-OZ advert to the app subscriber and delivers the fresh
    /// samples on app keys to the app.
    fn app_advert(&self, report: &AdvReport<'_>) {
        let Ok(mut sub) = self.app_sub.try_borrow_mut() else {
            return;
        };
        let queue = core::mem::take(sub.scanner_mut()).push(report.addr, report.rssi, report.data);
        *sub.scanner_mut() = queue;
        let mut buf = [0u8; APP_SAMPLE_BUF];
        // Every `try_recv` consumes at least one queued report, so this
        // drains the queue.
        for _ in 0..=APP_SCAN_QUEUE {
            let sample = match sub.try_recv(&mut buf) {
                Ok(Some(sample)) => sample,
                Ok(None) => break,
                Err(_) => {
                    self.app_dropped.set(self.app_dropped.get().wrapping_add(1));
                    continue;
                }
            };
            let key = sample.key.as_str();
            if !app_may_use(key) {
                continue;
            }
            let (Ok(key), Ok(payload)) = (String::try_from(key), Vec::from_slice(sample.payload))
            else {
                self.app_dropped.set(self.app_dropped.get().wrapping_add(1));
                continue;
            };
            let counter = if mw::APP_SAMPLES
                .try_send(mw::MwMessage { key, payload })
                .is_ok()
            {
                &self.app_rx
            } else {
                &self.app_dropped
            };
            counter.set(counter.get().wrapping_add(1));
        }
    }

    /// Clears the device table and opens a `BleScan`.
    fn begin(&self) {
        if let Ok(mut seen) = self.seen.try_borrow_mut() {
            seen.clear();
        }
        self.scanning.set(true);
    }

    /// Closes the `BleScan` and returns the strongest devices, strongest first.
    fn finish(&self) -> Vec<BleDevice, MAX_BLE> {
        self.scanning.set(false);
        self.seen
            .try_borrow()
            .map(|seen| rank_devices(&seen))
            .unwrap_or_default()
    }

    /// Takes the metrics snapshot for `now`.
    fn snapshot(&self, now: Millis) -> Option<badge_core::ble_metrics::BleSnapshot> {
        self.metrics
            .try_borrow_mut()
            .ok()
            .map(|mut m| m.snapshot(now))
    }

    /// Handles one report heard at `now`.
    fn report(&self, report: &AdvReport<'_>, now: Millis) {
        let payload = oz::oz_payload(report.data);
        if let Ok(mut metrics) = self.metrics.try_borrow_mut() {
            metrics.observe(report.addr, report.rssi, payload.is_some(), now);
        }
        if self.scanning.get()
            && let Ok(mut seen) = self.seen.try_borrow_mut()
        {
            merge_report(&mut seen, report);
        }
        let Some(payload) = payload else {
            self.app_advert(report);
            return;
        };
        let admitted = self
            .forward
            .try_borrow_mut()
            .is_ok_and(|mut f| f.admit(report.addr, payload, now));
        if !admitted {
            return;
        }
        let Ok(payload) = Vec::from_slice(payload.get(..oz::MAX_PAYLOAD).unwrap_or(payload)) else {
            return;
        };
        let packet = Input::RadioPacket {
            via: Via::Ble,
            rssi: report.rssi,
            payload,
        };
        let counter = if RADIO_EVENTS.try_send(packet).is_ok() {
            &self.forwarded
        } else {
            &self.dropped
        };
        counter.set(counter.get().wrapping_add(1));
    }
}

impl EventHandler for Collector {
    fn on_adv_reports(&self, reports: LeAdvReportsIter<'_>) {
        let now = Millis(Instant::now().as_millis());
        for report in reports.take(MAX_REPORTS_PER_EVENT).flatten() {
            // Copy out of the packed HCI struct before use.
            let addr = report.addr;
            let addr_kind = report.addr_kind;
            let rssi = report.rssi;
            let data = report.data;
            self.report(
                &AdvReport {
                    addr: addr.into_inner(),
                    addr_kind: addr_kind.into_inner(),
                    rssi,
                    data,
                },
                now,
            );
        }
    }
}

/// Owns the BLE host and serves [`BLE_CMDS`] forever.
#[embassy_executor::task]
pub async fn ble_task(controller: BleController) {
    let mut resources: HostResources<
        BleController,
        DefaultPacketPool,
        CONNECTIONS_MAX,
        L2CAP_CHANNELS_MAX,
    > = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).build();
    let mut runner = stack.runner();
    let collector = Collector::new();
    collector.load_known_keys();
    let rpc_rx = RpcRx::new();
    let rpc_tx = RpcTx::new();
    let server = match Server::new_default(DEFAULT_NAME) {
        Ok(server) => Some(server),
        Err(e) => {
            warn!("ble: GATT server unavailable: {}", e);
            None
        }
    };

    if let Either3::First(result) = select3(
        runner.run_with_handler(&collector),
        serve(&stack, &collector, server.as_ref(), &rpc_rx, &rpc_tx),
        rpc_loop(&rpc_rx, &rpc_tx),
    )
    .await
    {
        match result {
            Ok(()) => warn!("ble: host runner exited"),
            Err(e) => warn!("ble: host runner failed: {}", e),
        }
    }
    loop {
        if let Some(reply) = error_reply(&BLE_CMDS.receive().await, ErrorCode::Radio) {
            RADIO_EVENTS.send(reply).await;
        }
    }
}

/// An error answer to `cmd`; `None` for the unanswered `Beacon`.
fn error_reply(cmd: &BleCmd, code: ErrorCode) -> Option<Input> {
    match *cmd {
        BleCmd::Scan { seq, .. } => Some(Input::BleScanned {
            seq,
            result: Err(code),
        }),
        BleCmd::Advertise { seq, .. } => Some(Input::BleAdvertised {
            seq,
            result: Err(code),
        }),
        BleCmd::Beacon(_) => None,
    }
}

/// What the command loop keeps between events.
struct Session<'d, 's> {
    /// Scan-response name; empty = advertising stopped by request.
    name: String<29>,
    /// Middleware publisher; its advertiser owns the advertising multiplexer
    /// (beacon plus queued middleware adverts).
    publisher: MwPublisher,
    /// The advertising data last installed in the running set.
    installed: Vec<u8, MAX_ADV_DATA>,
    /// App publishes queued for advertising / dropped.
    mw_sent: u32,
    mw_dropped: u32,
    /// Keeps the advertising set alive; dropping it stops advertising.
    adv: Option<Adv<'d>>,
    /// Do not try to (re)start advertising before this time.
    adv_retry_at: Instant,
    /// The connected central, if any.
    conn: Option<Gatt<'d, 's>>,
    /// Last ATT traffic (or connect) on `conn`.
    last_activity: Instant,
    /// When `conn` was accepted.
    connected_at: Instant,
    /// The open `BleScan`: its sequence number and deadline.
    scan: Option<(Option<u32>, Instant)>,
    /// When the next `BleStats` is due.
    next_stats: Instant,
    /// When the next counters log line is due.
    next_log: Instant,
}

/// What woke the command loop.
enum Wake<'a, 's> {
    Cmd(BleCmd),
    Tick,
    Gatt(GattConnectionEvent<'a, 's, DefaultPacketPool>),
    Publish(mw::MwMessage),
    MuxDeadline,
    RpcOut(Vec<u8, RPC_VALUE>),
}

/// Command loop: runs while the host runner is alive.
async fn serve(
    stack: &Host<'_>,
    collector: &Collector,
    server: Option<&Server<'static>>,
    rpc_rx: &RpcRx,
    rpc_tx: &RpcTx,
) {
    let mut scanner = Scanner::new(stack.central());
    let mut peripheral = stack.peripheral();
    let config = ScanConfig {
        active: false,
        interval: SCAN_WINDOW,
        window: SCAN_WINDOW,
        ..ScanConfig::default()
    };
    // Dropping the session would disable scanning, so it lives for the rest
    // of the loop.
    let mut attempts = 0;
    let session = loop {
        attempts += 1;
        if let Ok(Ok(session)) = with_timeout(HCI_TIMEOUT, scanner.scan(&config)).await {
            info!("ble: continuous passive scan running");
            break Some(session);
        }
        warn!("ble: continuous scan could not start");
        if attempts >= SCAN_START_ATTEMPTS {
            break None;
        }
    };
    let scanning = session.is_some();

    let now = Instant::now();
    let mut s = Session {
        name: ble_adv_name(DEFAULT_NAME),
        publisher: AdvertisingPublisher::new(MuxAdvertiser::new(), mw_template()),
        installed: Vec::new(),
        mw_sent: 0,
        mw_dropped: 0,
        adv: None,
        adv_retry_at: now,
        conn: None,
        last_activity: now,
        connected_at: now,
        scan: None,
        next_stats: now + STATS_PERIOD,
        next_log: now,
    };
    let mut ticker = Ticker::every(HOUSEKEEPING);

    loop {
        let event = {
            let gatt_next = async {
                match &s.conn {
                    Some(conn) => conn.next().await,
                    None => core::future::pending().await,
                }
            };
            let deadline = s.publisher.advertiser().mux().next_deadline();
            let mux_due = async {
                match deadline {
                    Some(at) => Timer::at(Instant::from_millis(at.0)).await,
                    None => core::future::pending().await,
                }
            };
            let first = select3(BLE_CMDS.receive(), ticker.next(), gatt_next);
            let second = select3(mw::BLE_OUT.receive(), mux_due, rpc_tx.receive());
            match select(first, second).await {
                Either::First(Either3::First(cmd)) => Wake::Cmd(cmd),
                Either::First(Either3::Second(())) => Wake::Tick,
                Either::First(Either3::Third(ev)) => Wake::Gatt(ev),
                Either::Second(Either3::First(msg)) => Wake::Publish(msg),
                Either::Second(Either3::Second(())) => Wake::MuxDeadline,
                Either::Second(Either3::Third(fragment)) => Wake::RpcOut(fragment),
            }
        };
        match event {
            Wake::Cmd(cmd) => command(&mut s, &mut peripheral, collector, scanning, cmd).await,
            Wake::Tick => housekeeping(&mut s, &mut peripheral, collector, server, rpc_rx).await,
            Wake::Gatt(ev) => {
                let mtu = s.conn.as_ref().map_or(23, |c| c.raw().att_mtu());
                if gatt_event(stack, server, ev, rpc_rx, mtu).await {
                    info!("ble: central disconnected");
                    s.conn = None;
                    // Dropped when full (see the module docs).
                    let _ = rpc_rx.try_send(RpcIn::Disconnected);
                } else {
                    s.last_activity = Instant::now();
                }
            }
            Wake::Publish(msg) => {
                publish_app(&mut s, collector, &msg).await;
                refresh_adv(&mut s, &mut peripheral).await;
            }
            Wake::MuxDeadline => refresh_adv(&mut s, &mut peripheral).await,
            Wake::RpcOut(fragment) => {
                if let (Some(conn), Some(server)) = (&s.conn, server) {
                    let sent =
                        with_timeout(HCI_TIMEOUT, server.mw.rpc.notify(conn, &fragment, false))
                            .await;
                    if !matches!(sent, Ok(Ok(()))) {
                        warn!("ble: rpc notification failed");
                    }
                }
            }
        }
    }
}

/// Queues one app publish in the advertising multiplexer through the
/// middleware publisher, and lets adverts on its key reach the app.
async fn publish_app(s: &mut Session<'_, '_>, collector: &Collector, msg: &mw::MwMessage) {
    let key = msg.key.as_str();
    let queued = match Key::new(key) {
        Ok(k) if app_may_use(key) => {
            let ok = s.publisher.publish(k, &msg.payload).await.is_ok();
            if ok {
                collector.learn_key(k);
            }
            ok
        }
        _ => false,
    };
    if queued {
        s.mw_sent = s.mw_sent.wrapping_add(1);
    } else {
        s.mw_dropped = s.mw_dropped.wrapping_add(1);
    }
}

/// Steps the advertising multiplexer to now and, if advertising, installs
/// its data when it differs from what is on air.
async fn refresh_adv(
    s: &mut Session<'_, '_>,
    peripheral: &mut Peripheral<'_, BleController, DefaultPacketPool>,
) {
    let adv = s.publisher.advertiser_mut();
    adv.step(Millis(Instant::now().as_millis()));
    let on_air = adv.mux().on_air();
    if s.adv.is_none() || on_air == s.installed.as_slice() {
        return;
    }
    let Ok(data) = Vec::from_slice(on_air) else {
        return;
    };
    if update(peripheral, &data, &s.name).await.is_ok() {
        s.installed = data;
    } else {
        warn!("ble: advertising data update failed");
    }
}

/// `name` as the stored scan-response name.
fn ble_adv_name(name: &str) -> String<29> {
    badge_core::radio_rank::truncate_str(name)
}

/// Executes one command, sending its single reply (if it has one).
async fn command<'d>(
    s: &mut Session<'d, '_>,
    peripheral: &mut Peripheral<'d, BleController, DefaultPacketPool>,
    collector: &Collector,
    scanning: bool,
    cmd: BleCmd,
) {
    match cmd {
        BleCmd::Scan { seq, duration_ms } => {
            let reply = if s.scan.is_some() {
                Some(Err(ErrorCode::Busy))
            } else if !scanning {
                Some(Err(ErrorCode::Radio))
            } else {
                let ms = u64::from(duration_ms.clamp(MIN_SCAN_MS, MAX_SCAN_MS));
                collector.begin();
                s.scan = Some((seq, Instant::now() + Duration::from_millis(ms)));
                None
            };
            if let Some(result) = reply {
                RADIO_EVENTS.send(Input::BleScanned { seq, result }).await;
            }
        }
        BleCmd::Advertise { seq, name } => {
            s.name = name;
            let result = if s.name.is_empty() {
                s.adv = None;
                info!("ble: advertising stopped");
                Ok(())
            } else if s.conn.is_some() {
                // Resumes with the new name when the central disconnects.
                Ok(())
            } else if s.adv.is_some() {
                let data = on_air(s);
                let result = update(peripheral, &data, &s.name).await;
                if result.is_ok() {
                    s.installed = data;
                }
                result
            } else {
                start(s, peripheral).await
            };
            RADIO_EVENTS
                .send(Input::BleAdvertised { seq, result })
                .await;
        }
        BleCmd::Beacon(data) => {
            if s.publisher.advertiser_mut().set_beacon(&data).is_err() {
                warn!("ble: beacon longer than advertising data");
            }
            refresh_adv(s, peripheral).await;
        }
    }
}

/// Periodic work: accept a central, finish a due scan, send stats, drop an
/// idle link, (re)start advertising.
async fn housekeeping<'d, 's>(
    s: &mut Session<'d, 's>,
    peripheral: &mut Peripheral<'d, BleController, DefaultPacketPool>,
    collector: &Collector,
    server: Option<&'s Server<'static>>,
    rpc_rx: &RpcRx,
) {
    let now = Instant::now();
    // Keys the payload subscribed to (bounded by the channel capacity).
    while let Ok(key) = mw::APP_SUBSCRIBED.try_receive() {
        if let Ok(key) = Key::new(&key) {
            collector.learn_key(key);
        }
    }
    if s.conn.is_none()
        && let Some(conn) = peripheral.try_accept()
    {
        // The controller stopped advertising; release our handle too.
        s.adv = None;
        match server.map(|srv| conn.with_attribute_server(srv)) {
            Some(Ok(gatt)) => {
                info!("ble: central connected");
                // Dropped when full (see the module docs).
                let _ = rpc_rx.try_send(RpcIn::Connected);
                s.conn = Some(gatt);
                s.last_activity = now;
                s.connected_at = now;
            }
            // `conn` was consumed; dropping the failed result disconnects.
            Some(Err(_)) | None => warn!("ble: central connected without GATT; dropped"),
        }
    }
    if let Some(conn) = &s.conn {
        if now >= s.connected_at + MAX_CONNECTION {
            info!("ble: central connected too long, disconnecting");
            conn.raw().disconnect();
            // Re-arm so the request is repeated only once per period.
            s.connected_at = now;
            s.last_activity = now;
        } else if now >= s.last_activity + IDLE_TIMEOUT {
            info!("ble: idle central, disconnecting");
            conn.raw().disconnect();
            s.last_activity = now;
        }
    }
    if let Some((seq, deadline)) = s.scan
        && now >= deadline
    {
        s.scan = None;
        let devices = collector.finish();
        info!("ble: scan heard {} devices", devices.len());
        RADIO_EVENTS
            .send(Input::BleScanned {
                seq,
                result: Ok(devices),
            })
            .await;
    }
    if now >= s.next_stats {
        s.next_stats = now + STATS_PERIOD;
        if let Some(snapshot) = collector.snapshot(Millis(now.as_millis())) {
            // Stats are periodic; a full queue just skips one.
            let _ = RADIO_EVENTS.try_send(Input::BleStats(snapshot));
        }
    }
    if now >= s.next_log {
        s.next_log = now + Duration::from_secs(10);
        info!(
            "ble: oz packets forwarded {} dropped {}",
            collector.forwarded.get(),
            collector.dropped.get()
        );
        info!(
            "ble: mw adverts queued {} dropped {}, app samples {} dropped {}, keys unlearned {}",
            s.mw_sent,
            s.mw_dropped,
            collector.app_rx.get(),
            collector.app_dropped.get(),
            collector.app_unlearned.get()
        );
    }
    if s.adv.is_none()
        && s.conn.is_none()
        && !s.name.is_empty()
        && now >= s.adv_retry_at
        && start(s, peripheral).await.is_err()
    {
        s.adv_retry_at = now + ADV_RETRY;
    }
    refresh_adv(s, peripheral).await;
}

/// The multiplexer's current advertising data, stepped to now.
fn on_air(s: &mut Session<'_, '_>) -> Vec<u8, MAX_ADV_DATA> {
    let adv = s.publisher.advertiser_mut();
    adv.step(Millis(Instant::now().as_millis()));
    Vec::from_slice(adv.mux().on_air()).unwrap_or_default()
}

/// Starts the advertising set with the multiplexer's current data.
async fn start<'d>(
    s: &mut Session<'d, '_>,
    peripheral: &mut Peripheral<'d, BleController, DefaultPacketPool>,
) -> Result<(), ErrorCode> {
    let data = on_air(s);
    let handle = start_advertising(peripheral, &data, &s.name).await?;
    s.adv = Some(handle);
    s.installed = data;
    Ok(())
}

/// Handles one event of the connected central. Returns true once it has
/// disconnected.
async fn gatt_event(
    stack: &Host<'_>,
    server: Option<&Server<'_>>,
    event: GattConnectionEvent<'_, '_, DefaultPacketPool>,
    rpc_rx: &RpcRx,
    mtu: u16,
) -> bool {
    match event {
        GattConnectionEvent::Disconnected { .. } => return true,
        GattConnectionEvent::RequestConnectionParams(req) => {
            if with_timeout(HCI_TIMEOUT, req.accept(None, stack))
                .await
                .is_err()
            {
                warn!("ble: connection parameter update timed out");
            }
        }
        GattConnectionEvent::Gatt { event } => match server {
            Some(server) => gatt_request(server, event, rpc_rx, mtu).await,
            None => {
                if let Ok(reply) = event.accept() {
                    reply.send().await;
                }
            }
        },
        _ => {}
    }
    false
}

/// Answers one ATT request: refresh read values, map writes to host messages,
/// hand RPC fragments to the RPC loop.
async fn gatt_request(
    server: &Server<'_>,
    event: GattEvent<'_, '_, DefaultPacketPool>,
    rpc_rx: &RpcRx,
    mtu: u16,
) {
    let svc = &server.badge;
    let mut host_msg = None;
    let reply = match event {
        GattEvent::Read(read) => {
            let handle = read.handle();
            // A failed set leaves the previous value; the read still succeeds.
            let _ = if handle == svc.info.handle {
                let text =
                    gatt::info_string(crate::FIRMWARE, &base_mac(), probe::word(probe_layout::CHI));
                svc.info.set(
                    server,
                    &Vec::from_slice(text.as_bytes()).unwrap_or_default(),
                )
            } else if handle == svc.pet_mode.handle {
                svc.pet_mode.set(
                    server,
                    &gatt::probe_byte(probe::word(probe_layout::PET_SELECTED)),
                )
            } else if handle == svc.led_pattern.handle {
                svc.led_pattern.set(
                    server,
                    &gatt::probe_byte(probe::word(probe_layout::PATTERN)),
                )
            } else {
                Ok(())
            };
            read.accept()
        }
        GattEvent::Write(write) if write.handle() == server.mw.rpc.handle => {
            let data = write.with_data(|offset, data| {
                if offset == 0 {
                    Vec::from_slice(data).ok()
                } else {
                    None
                }
            });
            match data {
                None => write.reject(AttErrorCode::INVALID_ATTRIBUTE_VALUE_LENGTH),
                Some(data) => match rpc_rx.try_send(RpcIn::Write { mtu, data }) {
                    Ok(()) => write.accept(),
                    Err(_) => write.reject(AttErrorCode::INSUFFICIENT_RESOURCES),
                },
            }
        }
        GattEvent::Write(write) => {
            let handle = write.handle();
            let target = if handle == svc.pet_mode.handle {
                Some(Writable::PetMode)
            } else if handle == svc.led_pattern.handle {
                Some(Writable::LedPattern)
            } else if handle == svc.led_color.handle {
                Some(Writable::LedColor)
            } else {
                None
            };
            match target {
                None => write.accept(),
                Some(target) => {
                    let mapped = write.with_data(|offset, data| {
                        let data: Vec<u8, MAX_WRITE> =
                            Vec::from_slice(data).map_err(|_| GattError::BadLength)?;
                        if offset != 0 {
                            return Err(GattError::BadLength);
                        }
                        gatt::write_to_host_msg(target, &data)
                    });
                    match mapped {
                        Ok(msg) => {
                            host_msg = Some(msg);
                            write.accept()
                        }
                        Err(GattError::BadLength) => {
                            write.reject(AttErrorCode::INVALID_ATTRIBUTE_VALUE_LENGTH)
                        }
                        Err(GattError::BadValue) => write.reject(AttErrorCode::VALUE_NOT_ALLOWED),
                    }
                }
            }
        }
        other => other.accept(),
    };
    if let Ok(reply) = reply {
        reply.send().await;
    }
    if let Some(msg) = host_msg {
        RADIO_EVENTS.send(Input::Host(msg)).await;
    }
}

/// Badge state for the RPC handler, read from the PROBE block.
struct ProbeSnapshot;

impl SnapshotSource for ProbeSnapshot {
    fn snapshot(&self) -> BadgeSnapshot {
        BadgeSnapshot {
            firmware: crate::FIRMWARE,
            mac: base_mac(),
            chi: probe::word(probe_layout::CHI),
            pet_selected: probe::word(probe_layout::PET_SELECTED),
            pattern: probe::word(probe_layout::PATTERN),
        }
    }
}

/// Why the RPC characteristic I/O failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RpcIoError {
    /// A value longer than the characteristic or the caller's buffer.
    TooLong,
}

/// The RPC characteristic as two task-local channels (see the module docs).
struct ChannelRpcIo<'a> {
    rx: &'a RpcRx,
    tx: &'a RpcTx,
    /// MTU of the connection the last write came from (the `BleServer` caps
    /// fragments at [`RPC_VALUE`] bytes).
    mtu: AttMtu,
}

impl RpcIo for ChannelRpcIo<'_> {
    type Error = RpcIoError;

    fn att_mtu(&self) -> AttMtu {
        self.mtu
    }

    async fn notify(&mut self, data: &[u8]) -> Result<(), RpcIoError> {
        let value = Vec::from_slice(data).map_err(|_| RpcIoError::TooLong)?;
        self.tx.send(value).await;
        Ok(())
    }

    async fn next_event<'b>(&mut self, buf: &'b mut [u8]) -> Result<RpcEvent<'b>, RpcIoError> {
        // Cancel-safe: an embassy channel `receive` pops only when ready.
        match self.rx.receive().await {
            RpcIn::Write { mtu, data } => {
                // trouble reports a legal MTU; the fallback is never used.
                self.mtu = AttMtu::new(mtu).unwrap_or(AttMtu::DEFAULT);
                let out = buf.get_mut(..data.len()).ok_or(RpcIoError::TooLong)?;
                out.copy_from_slice(&data);
                Ok(RpcEvent::Write(out))
            }
            RpcIn::Connected => {
                self.mtu = AttMtu::DEFAULT;
                Ok(RpcEvent::Connected)
            }
            RpcIn::Disconnected => Ok(RpcEvent::Disconnected),
        }
    }
}

/// Serves middleware RPC requests forever (see the module docs).
async fn rpc_loop(rx: &RpcRx, tx: &RpcTx) {
    let mut server: RpcServer<ChannelRpcIo<'_>, RPC_VALUE> = mw_rpc::rpc_server(ChannelRpcIo {
        rx,
        tx,
        mtu: AttMtu::DEFAULT,
    });
    let mut handler: BadgeHandler<ProbeSnapshot> = BadgeHandler::new(ProbeSnapshot);
    let mut scratch = [0u8; mw_rpc::RPC_SCRATCH];
    loop {
        if let Err(e) = server.serve_one(&mut handler, &mut scratch).await {
            warn!("ble: rpc request failed: {}", Debug2Format(&e));
        }
        for _ in 0..mw_rpc::OUTBOX {
            let Some(msg) = handler.take_host_msg() else {
                break;
            };
            RADIO_EVENTS.send(Input::Host(msg)).await;
        }
    }
}

/// The eFuse base MAC (the badge id is its last three bytes).
fn base_mac() -> [u8; 6] {
    let addr = esp_hal::efuse::base_mac_address();
    let mut mac = [0u8; 6];
    for (dst, src) in mac.iter_mut().zip(addr.as_bytes()) {
        *dst = *src;
    }
    mac
}

/// Puts advertising data `adv` and the `name` scan response on air
/// without restarting the advertising set.
async fn update(
    peripheral: &mut Peripheral<'_, BleController, DefaultPacketPool>,
    adv: &[u8],
    name: &str,
) -> Result<(), ErrorCode> {
    let (scan_data, len) = ble_adv::scan_response(name);
    // The variant only selects trouble's validation (no Flags check); the
    // PDU type set by `advertise` is unchanged.
    let data = Advertisement::NonconnectableScannableUndirected {
        adv_data: ble_adv::adv_data(adv),
        scan_data: scan_data.get(..len).unwrap_or(&[]),
    };
    match with_timeout(HCI_TIMEOUT, peripheral.update_adv_data(data)).await {
        Ok(Ok(())) => Ok(()),
        _ => Err(ErrorCode::Radio),
    }
}

/// Starts the connectable-scannable advertising set with Flags-only data,
/// then installs advertising data `adv` and the `name` scan response.
/// Returns the handle
/// that keeps it running.
async fn start_advertising<'d>(
    peripheral: &mut Peripheral<'d, BleController, DefaultPacketPool>,
    adv_data: &[u8],
    name: &str,
) -> Result<Adv<'d>, ErrorCode> {
    let (scan_data, len) = ble_adv::scan_response(name);
    let params = AdvertisementParameters {
        interval_min: ADV_INTERVAL,
        interval_max: ADV_INTERVAL,
        ..AdvertisementParameters::default()
    };
    let adv = Advertisement::ConnectableScannableUndirected {
        adv_data: &ble_adv::DEFAULT_ADV_DATA,
        scan_data: scan_data.get(..len).unwrap_or(&[]),
    };
    let Ok(Ok(handle)) = with_timeout(HCI_TIMEOUT, peripheral.advertise(&params, adv)).await else {
        warn!("ble: advertising could not start");
        return Err(ErrorCode::Radio);
    };
    if adv_data != ble_adv::DEFAULT_ADV_DATA {
        update(peripheral, adv_data, name).await?;
    }
    info!("ble: advertising");
    Ok(handle)
}
