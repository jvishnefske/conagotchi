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
//! its Flags check: the PDU type stays the one set at start. The ADV_IND
//! carries exactly the latest `Effect::Beacon` data (Flags only before the
//! first beacon); the scan response carries the Complete Local Name,
//! `"conagotchi"` from boot. `BleAdvertise` changes the name in place (one
//! `BleAdvertised` reply each); an empty name stops the set, and a later
//! non-empty name starts it again.
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
//! without any ATT request or link event, and advertising resumes within one
//! [`HOUSEKEEPING`] period after the disconnect.
//!
//! If the host runner ever fails, the task keeps answering every BLE command
//! with `ErrorCode::Radio` so each command still gets exactly one reply.

use core::cell::{Cell, RefCell};

use badge_core::ble_adv::{self, DEFAULT_NAME, ForwardFilter};
use badge_core::ble_metrics::BleMetrics;
use badge_core::gatt::{self, GattError, Writable};
use badge_core::io::{Input, MAX_ADV_DATA, MAX_BLE, Millis, Via};
use badge_core::{oz, probe_layout};
use badge_proto::{BleDevice, ErrorCode};
use defmt::{info, warn};
use embassy_futures::select::{Either, Either3, select, select3};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Ticker, with_timeout};
use esp_radio::ble::controller::BleConnector;
use heapless::{String, Vec};
use trouble_host::prelude::*;

use super::RADIO_EVENTS;
use super::rank::{AdvReport, merge_report, rank_devices};
use crate::probe;

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

type Host<'a> = Stack<'a, BleController, DefaultPacketPool>;
type Adv<'a> = Advertiser<'a, BleController, DefaultPacketPool>;
type Gatt<'a, 's> = GattConnection<'a, 's, DefaultPacketPool>;

/// The GATT server: GAP plus the "Conagotchi" service.
#[gatt_server(connections_max = CONNECTIONS_MAX, mutex_type = NoopRawMutex)]
struct Server {
    badge: BadgeService,
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
        let Some(payload) = payload else { return };
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
    let server = match Server::new_default(DEFAULT_NAME) {
        Ok(server) => Some(server),
        Err(e) => {
            warn!("ble: GATT server unavailable: {}", e);
            None
        }
    };

    if let Either::First(result) = select(
        runner.run_with_handler(&collector),
        serve(&stack, &collector, server.as_ref()),
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
    /// Latest `Effect::Beacon` data.
    beacon: Vec<u8, MAX_ADV_DATA>,
    /// Keeps the advertising set alive; dropping it stops advertising.
    adv: Option<Adv<'d>>,
    /// Do not try to (re)start advertising before this time.
    adv_retry_at: Instant,
    /// The connected central, if any.
    conn: Option<Gatt<'d, 's>>,
    /// Last ATT traffic (or connect) on `conn`.
    last_activity: Instant,
    /// The open `BleScan`: its sequence number and deadline.
    scan: Option<(Option<u32>, Instant)>,
    /// When the next `BleStats` is due.
    next_stats: Instant,
    /// When the next counters log line is due.
    next_log: Instant,
}

/// Command loop: runs while the host runner is alive.
async fn serve(stack: &Host<'_>, collector: &Collector, server: Option<&Server<'static>>) {
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
        beacon: Vec::new(),
        adv: None,
        adv_retry_at: now,
        conn: None,
        last_activity: now,
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
            select3(BLE_CMDS.receive(), ticker.next(), gatt_next).await
        };
        match event {
            Either3::First(cmd) => command(&mut s, &mut peripheral, collector, scanning, cmd).await,
            Either3::Second(()) => housekeeping(&mut s, &mut peripheral, collector, server).await,
            Either3::Third(ev) => {
                if gatt_event(stack, server, ev).await {
                    info!("ble: central disconnected");
                    s.conn = None;
                } else {
                    s.last_activity = Instant::now();
                }
            }
        }
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
                update(peripheral, &s.beacon, &s.name).await
            } else {
                let started = start_advertising(peripheral, &s.beacon, &s.name).await;
                started.map(|handle| s.adv = Some(handle))
            };
            RADIO_EVENTS
                .send(Input::BleAdvertised { seq, result })
                .await;
        }
        BleCmd::Beacon(data) => {
            s.beacon = data;
            if s.adv.is_some() && update(peripheral, &s.beacon, &s.name).await.is_err() {
                warn!("ble: advertising data update failed");
            }
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
) {
    let now = Instant::now();
    if s.conn.is_none()
        && let Some(conn) = peripheral.try_accept()
    {
        // The controller stopped advertising; release our handle too.
        s.adv = None;
        match server.map(|srv| conn.with_attribute_server(srv)) {
            Some(Ok(gatt)) => {
                info!("ble: central connected");
                s.conn = Some(gatt);
                s.last_activity = now;
            }
            // `conn` was consumed; dropping the failed result disconnects.
            Some(Err(_)) | None => warn!("ble: central connected without GATT; dropped"),
        }
    }
    if let Some(conn) = &s.conn
        && now >= s.last_activity + IDLE_TIMEOUT
    {
        info!("ble: idle central, disconnecting");
        conn.raw().disconnect();
        s.last_activity = now;
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
    }
    if s.adv.is_none() && s.conn.is_none() && !s.name.is_empty() && now >= s.adv_retry_at {
        match start_advertising(peripheral, &s.beacon, &s.name).await {
            Ok(handle) => s.adv = Some(handle),
            Err(_) => s.adv_retry_at = now + ADV_RETRY,
        }
    }
}

/// Handles one event of the connected central. Returns true once it has
/// disconnected.
async fn gatt_event(
    stack: &Host<'_>,
    server: Option<&Server<'_>>,
    event: GattConnectionEvent<'_, '_, DefaultPacketPool>,
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
            Some(server) => gatt_request(server, event).await,
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

/// Answers one ATT request: refresh read values, map writes to host messages.
async fn gatt_request(server: &Server<'_>, event: GattEvent<'_, '_, DefaultPacketPool>) {
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

/// The eFuse base MAC (the badge id is its last three bytes).
fn base_mac() -> [u8; 6] {
    let addr = esp_hal::efuse::base_mac_address();
    let mut mac = [0u8; 6];
    for (dst, src) in mac.iter_mut().zip(addr.as_bytes()) {
        *dst = *src;
    }
    mac
}

/// Puts `beacon` (or the Flags default) and the `name` scan response on air
/// without restarting the advertising set.
async fn update(
    peripheral: &mut Peripheral<'_, BleController, DefaultPacketPool>,
    beacon: &[u8],
    name: &str,
) -> Result<(), ErrorCode> {
    let (scan_data, len) = ble_adv::scan_response(name);
    // The variant only selects trouble's validation (no Flags check); the
    // PDU type set by `advertise` is unchanged.
    let data = Advertisement::NonconnectableScannableUndirected {
        adv_data: ble_adv::adv_data(beacon),
        scan_data: scan_data.get(..len).unwrap_or(&[]),
    };
    match with_timeout(HCI_TIMEOUT, peripheral.update_adv_data(data)).await {
        Ok(Ok(())) => Ok(()),
        _ => Err(ErrorCode::Radio),
    }
}

/// Starts the connectable-scannable advertising set with Flags-only data,
/// then installs `beacon` and the `name` scan response. Returns the handle
/// that keeps it running.
async fn start_advertising<'d>(
    peripheral: &mut Peripheral<'d, BleController, DefaultPacketPool>,
    beacon: &[u8],
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
    if !beacon.is_empty() {
        update(peripheral, beacon, name).await?;
    }
    info!("ble: advertising");
    Ok(handle)
}
