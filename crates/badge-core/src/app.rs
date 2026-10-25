//! The application state machine.
//!
//! [`App`] is the whole model of the badge: screen, LEDs, buttons, radio scan
//! results, host text and counters. [`App::handle`] consumes the current
//! state and one [`Input`] and returns the next state plus the [`Effects`] the
//! shell must carry out. It never touches hardware and never allocates.
//!
//! The public transition stays a pure function of (state, input) to (state,
//! effects): `handle` takes the state by value and returns the next one, and
//! nothing outside it can observe an intermediate state. Inside, the state is
//! moved once into a local and the private helpers mutate it through
//! `&mut self` instead of taking and returning `Self` by value. `App` is
//! several kilobytes and a chain of by-value helpers (tick, trade lobby,
//! beacon) is not guaranteed copy elision, so each level could copy the whole
//! state onto the stack and overflow the MCU's main task stack.
//!
//! Protocol rules enforced here:
//! - every `HostMsg` gets exactly one `Response` echoing its `seq` (FR-6),
//!   either immediately or when the shell reports the matching result;
//! - undecodable frames are answered with `Error{DECODE}` on seq 0 (FR-5);
//! - mailbox messages (seq [`crate::probe_layout::MAILBOX_SEQ`]) are handled
//!   exactly like RTT messages.
//!
//! The PET screen ([`crate::pet`]) animates on every tick: a change of the
//! selected or active pet mode emits `Redraw` (the mode label changes), a
//! frame-only change emits `Animate`, and never both in one call. The pet
//! advances on every tick even when another screen is showing, so AUTO keeps
//! cycling.
//!
//! One call never emits more than [`MAX_EFFECTS`] effects: a tick accepts at
//! most [`MAX_EDGES_PER_TICK`] button edges (each costs an event plus at most
//! one action effect), one of `Redraw` / `Animate` is emitted at most once,
//! and `Leds` at most once. The `PetChanged` event is sent last and only when
//! a slot is left; otherwise it is deferred to the next call with room (a
//! tick without edges always has room). It always reports the current
//! selected/active pair, so changes made before it goes out coalesce into one
//! event. Any overflow would be counted by [`App::dropped_effects`].
//!
//! A tick also runs the trade lobby (`app::trade`, at most
//! [`TRADE_TICK_EFFECTS`] effects: advert update, link frame, trade event)
//! and the LED sync clock (at most one `SyncBroadcast`), so the tick budget
//! is `2 * MAX_EDGES_PER_TICK + Redraw + Leds + PetChanged + SyncBroadcast +
//! TRADE_TICK_EFFECTS`, checked at compile time. Radio inputs
//! (`RadioPacket`) stay within the same bound.
//!
//! LEDs are rendered from the network time of [`crate::sync::SyncClock`], so
//! badges that hear each other animate in phase.
//!
//! Without the `dev` feature the GPIO screen is not in [`SCREEN_ORDER`], GPIO
//! requests are answered `UNSUPPORTED` and `Info.probe_pins` is empty.

use badge_proto::{
    AccessPoint, Ack, BadgeMsg, BadgeMsg_, BleDevice, BleMetrics, BleScanResult, Button,
    ButtonEvent, ButtonState, ErrorCode, Event, Event_, GpioLevel, HostMsg, HostMsg_::Cmd,
    Indicator, Info,
    LedPattern, PetChanged, PetMode, Pong, Response, Response_::Body, Screen, ScreenChanged,
    SyncState, WifiScanResult, WifiState,
};
use heapless::{String, Vec};

#[cfg(feature = "dev")]
use crate::board;
use crate::board::{display, led::RGB_COUNT};
use crate::ble_metrics::BleSnapshot;
use crate::buttons::Debouncer;
use crate::color::Color;
use crate::contacts::Contacts;
use crate::io::{
    ButtonMask, DrawOp, Effect, Effects, Input, MAX_APS, MAX_BLE, MAX_EFFECTS, Millis,
    ScanKind, StorageEvent,
};
use crate::leds::{self, DEFAULT_BRIGHTNESS};
use crate::oz::sign::RadioKey;
use crate::pet::Pet;
use crate::sync::SyncClock;

mod trade;
mod wasm;

pub use trade::{CONTACT_ROWS, LobbyRow, TRADE_TICK_EFFECTS, TradeModel};

/// Number of screen values; `Screen` values are `0..SCREEN_COUNT` (GPIO is
/// a valid value even when the `dev` feature leaves it out of the order).
pub const SCREEN_COUNT: i32 = 10;
/// Screens in LEFT / RIGHT order (and page-dot order); PET follows HOME.
#[cfg(feature = "dev")]
pub const SCREEN_ORDER: [Screen; 10] = [
    Screen::Home,
    Screen::Pet,
    Screen::Trade,
    Screen::Contacts,
    Screen::Leds,
    Screen::Ble,
    Screen::Wifi,
    Screen::Buttons,
    Screen::Host,
    Screen::Gpio,
];
/// Screens in LEFT / RIGHT order (and page-dot order); PET follows HOME.
#[cfg(not(feature = "dev"))]
pub const SCREEN_ORDER: [Screen; 9] = [
    Screen::Home,
    Screen::Pet,
    Screen::Trade,
    Screen::Contacts,
    Screen::Leds,
    Screen::Ble,
    Screen::Wifi,
    Screen::Buttons,
    Screen::Host,
];
/// Chip name reported by `GetInfo`.
pub const CHIP: &str = "esp32s3";
/// BLE scan duration used when the request says 0, and by the badge UI.
pub const DEFAULT_BLE_SCAN_MS: u32 = 3000;
/// Longest BLE scan a request may ask for.
pub const MAX_BLE_SCAN_MS: u32 = 10_000;
/// Button edges accepted per tick; further edges are deferred to later ticks.
pub const MAX_EDGES_PER_TICK: usize = 2;
/// Minimum interval between redraws of the animated LEDS screen.
pub const LEDS_REDRAW_MS: u64 = 250;
/// Number of entries in [`App::press_counts`] (indexed by `Button` value).
pub const PRESS_COUNT_LEN: usize = 6;

/// Fixed facts about the board, supplied by the shell at boot.
#[derive(Clone, Copy, Debug)]
pub struct BoardInfo {
    /// Wi-Fi station MAC address.
    pub mac: [u8; 6],
    /// Firmware name and version, reported by `GetInfo`.
    pub firmware: &'static str,
}

/// Protocol traffic counters (all wrap on overflow).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Counters {
    /// Host frames received: decoded messages plus undecodable frames.
    pub frames_rx: u32,
    /// `Effect::Send` effects emitted.
    pub frames_tx: u32,
    /// Host frames that could not be decoded.
    pub decode_errors: u32,
}

/// Progress of a Wi-Fi or BLE scan.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ScanState {
    /// No scan has been run yet.
    #[default]
    Idle,
    /// A scan is running; new scan requests are refused with `BUSY`.
    Scanning,
    /// The last scan finished; its results are stored.
    Done,
    /// The last scan failed with this code.
    Failed(ErrorCode),
}

impl ScanState {
    /// True while a scan is running.
    pub fn is_busy(self) -> bool {
        self == ScanState::Scanning
    }
}

/// State of the two discrete indicator LEDs.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Indicators {
    /// Red alert LED.
    pub alert: bool,
    /// Green raffle LED.
    pub raffle: bool,
}

/// Outcome of the last GPIO probe access.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GpioResult {
    /// Pin accessed.
    pub pin: u32,
    /// Level read (or driven), or the failure.
    pub result: Result<bool, ErrorCode>,
}

/// Text shown on the HOST screen.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HostText {
    text: String<256>,
    fg: Color,
    bg: Color,
}

impl Default for HostText {
    fn default() -> Self {
        Self {
            text: String::new(),
            fg: Color::WHITE,
            bg: Color::BLACK,
        }
    }
}

impl HostText {
    /// The text; `'\n'` separates lines. Empty until the host sends some.
    pub fn text(&self) -> &str {
        &self.text
    }

    /// Text colour (default white).
    pub fn fg(&self) -> Color {
        self.fg
    }

    /// Background colour (default black).
    pub fn bg(&self) -> Color {
        self.bg
    }
}

/// The complete application model. See the module docs.
#[derive(Clone, Debug)]
pub struct App {
    info: BoardInfo,
    now: Millis,
    debouncer: Debouncer,
    press_counts: [u32; PRESS_COUNT_LEN],
    screen: Screen,
    pattern: LedPattern,
    brightness: u8,
    manual: [Color; RGB_COUNT],
    leds: [Color; RGB_COUNT],
    counters: Counters,
    dropped_effects: u32,
    indicators: Indicators,
    wifi_scan: ScanState,
    aps: Vec<AccessPoint, MAX_APS>,
    wifi: Result<WifiState, ErrorCode>,
    ble_scan: ScanState,
    ble: Vec<BleDevice, MAX_BLE>,
    adv_request: String<29>,
    advertising: String<29>,
    host: HostText,
    gpio: Option<GpioResult>,
    /// Time and LED colours of the last emitted `Redraw`; `None` before the first.
    drawn: Option<(Millis, [Color; RGB_COUNT])>,
    pet: Pet,
    /// Pet (selected, active) pair last reported in a `PetChanged` event.
    pet_reported: (PetMode, PetMode),
    /// Trade lobby, current trade and TRADE / CONTACTS screen state.
    trade: TradeModel,
    /// Badges we completed a trade with.
    contacts: Contacts,
    /// Radio key; `None` until storage reports one (packets go unsigned).
    key: Option<RadioKey>,
    /// Latest BLE air metrics.
    ble_stats: BleSnapshot,
    /// LED sync clock.
    sync: SyncClock,
    /// Payload upload state (middleware loader).
    loader: middleware::loader::Loader,
}

/// Effects being collected for one `handle` call.
struct Out<'a> {
    fx: &'a mut Effects,
    dropped: u32,
}

impl Out<'_> {
    fn push(&mut self, effect: Effect) {
        if self.fx.push(effect).is_err() {
            self.dropped = self.dropped.saturating_add(1);
        }
    }

    /// Requests a redraw unless one is already queued.
    fn redraw(&mut self) {
        if !self.fx.contains(&Effect::Redraw) {
            self.push(Effect::Redraw);
        }
    }

    fn send(&mut self, kind: BadgeMsg_::Kind) {
        self.push(Effect::Send(BadgeMsg { kind: Some(kind) }));
    }

    fn respond(&mut self, seq: u32, body: Body) {
        self.send(BadgeMsg_::Kind::Response(Response {
            seq,
            body: Some(body),
        }));
    }

    fn ack(&mut self, seq: u32) {
        self.respond(seq, Body::Ack(Ack {}));
    }

    fn error(&mut self, seq: u32, code: ErrorCode, detail: &str) {
        let error = badge_proto::Error {
            code,
            detail: truncated(detail),
        };
        self.respond(seq, Body::Error(error));
    }

    fn event(&mut self, kind: Event_::Kind) {
        self.send(BadgeMsg_::Kind::Event(Event { kind: Some(kind) }));
    }
}

/// Copies as much of `s` as fits into a `String<N>`, never splitting a char.
fn truncated<const N: usize>(s: &str) -> String<N> {
    let mut out = String::new();
    for c in s.chars() {
        if out.push(c).is_err() {
            break;
        }
    }
    out
}

/// Press-count index for `button`, `None` if out of range.
fn button_index(button: Button) -> Option<usize> {
    usize::try_from(button.0)
        .ok()
        .filter(|&i| i < PRESS_COUNT_LEN)
}

/// Returns the screen `delta` positions from `screen` in [`SCREEN_ORDER`],
/// wrapping. An unknown screen counts as HOME.
fn cycle_screen(screen: Screen, delta: i32) -> Screen {
    let len = i32::try_from(SCREEN_ORDER.len()).unwrap_or(1);
    let at = SCREEN_ORDER.iter().position(|&s| s == screen).unwrap_or(0);
    let at = i32::try_from(at).unwrap_or(0);
    let next = usize::try_from((at + delta).rem_euclid(len)).unwrap_or(0);
    SCREEN_ORDER.get(next).copied().unwrap_or(Screen::Home)
}

impl App {
    /// Boot state: PET screen (the cat greets you) with the pet in AUTO,
    /// RAINBOW at [`DEFAULT_BRIGHTNESS`], LEDs not yet written (the first
    /// tick emits `Leds` and `Redraw`).
    pub fn new(info: BoardInfo) -> App {
        let pet = Pet::new();
        App {
            info,
            now: Millis(0),
            debouncer: Debouncer::default(),
            press_counts: [0; PRESS_COUNT_LEN],
            screen: Screen::Pet,
            pattern: LedPattern::Rainbow,
            brightness: DEFAULT_BRIGHTNESS,
            manual: [Color::BLACK; RGB_COUNT],
            leds: [Color::BLACK; RGB_COUNT],
            counters: Counters::default(),
            dropped_effects: 0,
            indicators: Indicators::default(),
            wifi_scan: ScanState::Idle,
            aps: Vec::new(),
            wifi: Ok(WifiState::default()),
            ble_scan: ScanState::Idle,
            ble: Vec::new(),
            adv_request: truncated(crate::ble_adv::DEFAULT_NAME),
            // The firmware advertises the default name from boot.
            advertising: truncated(crate::ble_adv::DEFAULT_NAME),
            host: HostText::default(),
            gpio: None,
            drawn: None,
            pet,
            pet_reported: (pet.selected(), pet.active()),
            trade: TradeModel::new(crate::oz::PeerId::from_mac(&info.mac)),
            contacts: Contacts::new(),
            key: None,
            ble_stats: BleSnapshot::default(),
            sync: SyncClock::new(SyncClock::id_from_mac(&info.mac)),
            loader: middleware::loader::Loader::Idle,
        }
    }

    /// Applies one input; returns the next state and the effects to perform,
    /// in order. Never returns more than [`MAX_EFFECTS`] effects.
    pub fn handle(self, input: Input) -> (App, Effects) {
        let mut app = self;
        let mut fx = Effects::new();
        app.step(input, &mut fx);
        (app, fx)
    }

    /// The same transition as [`App::handle`], applied in place: replaces
    /// the contents of `fx` with the effects of `input`. For the firmware,
    /// which keeps the app and one effect buffer in its task state so that
    /// neither (5 KB and 10 KB) is copied onto the MCU stack per input.
    pub fn step(&mut self, input: Input, fx: &mut Effects) {
        fx.clear();
        let mut out = Out { fx, dropped: 0 };
        let pet_before = self.pet;
        self.dispatch(input, &mut out);
        self.refresh_leds(&mut out);
        self.pet_output(pet_before, &mut out);
        self.finish(&out);
    }

    /// Routes one input to its handler, mutating the state in place.
    fn dispatch(&mut self, input: Input, out: &mut Out) {
        match input {
            Input::Tick { now, buttons } => self.tick(now, buttons, out),
            Input::Host(msg) => {
                self.count_rx(false);
                self.host(msg, out);
            }
            Input::HostGarbage => {
                out.error(0, ErrorCode::Decode, "undecodable frame");
                self.count_rx(true);
            }
            Input::Gpio { seq, pin, result } => self.gpio_done(seq, pin, result, out),
            Input::Adc { seq, result } => match result {
                Ok(level) => out.respond(seq, Body::Adc(level)),
                Err(code) => out.error(seq, code, "adc"),
            },
            Input::WifiScanned { seq, result } => self.wifi_scanned(seq, result, out),
            Input::Wifi { seq, result } => self.wifi_state(seq, result, out),
            Input::BleScanned { seq, result } => self.ble_scanned(seq, result, out),
            Input::BleAdvertised { seq, result } => self.ble_advertised(seq, result, out),
            Input::RadioPacket { via, rssi, payload } => {
                self.radio_packet(via, rssi, &payload, out);
            }
            Input::BleStats(stats) => self.ble_stats(stats, out),
            Input::SyncBeacon {
                from,
                network_ms,
                rx_at,
            } => {
                self.sync = self.sync.on_beacon(from, network_ms, rx_at);
            }
            Input::Storage(StorageEvent::Contact(contact)) => {
                self.contacts = core::mem::take(&mut self.contacts).with(contact);
            }
            Input::Storage(StorageEvent::Loaded { key }) => self.storage_loaded(key, out),
            Input::Wasm { seq, result } => Self::wasm_done(seq, result, out),
            Input::WasmLog(text) => Self::wasm_log(text, out),
        }
    }

    // ------------------------------------------------------------ accessors

    /// The response an [`Effect::SendScan`] stands for, built from the stored
    /// scan results (call it on the state `handle` returned).
    pub fn scan_response(&self, kind: ScanKind, seq: u32) -> BadgeMsg {
        let body = match kind {
            ScanKind::Wifi => Body::WifiScan(WifiScanResult { aps: self.aps.clone() }),
            ScanKind::Ble => Body::BleScan(BleScanResult { devices: self.ble.clone() }),
        };
        BadgeMsg {
            kind: Some(BadgeMsg_::Kind::Response(Response { seq, body: Some(body) })),
        }
    }

    /// Board facts supplied at boot.
    pub fn info(&self) -> BoardInfo {
        self.info
    }

    /// Current screen.
    pub fn screen(&self) -> Screen {
        self.screen
    }

    /// Current LED pattern.
    pub fn pattern(&self) -> LedPattern {
        self.pattern
    }

    /// Global LED brightness, 0..=255.
    pub fn brightness(&self) -> u8 {
        self.brightness
    }

    /// LED colours last rendered (brightness-scaled, as sent to the chain).
    pub fn leds(&self) -> [Color; RGB_COUNT] {
        self.leds
    }

    /// Colours set by `SetLed` / `SetLeds`, shown by the MANUAL pattern.
    pub fn manual_leds(&self) -> [Color; RGB_COUNT] {
        self.manual
    }

    /// Debounced pressed-button mask.
    pub fn pressed(&self) -> ButtonMask {
        self.debouncer.pressed()
    }

    /// Press counts indexed by `Button` value (index 0 is always 0).
    pub fn press_counts(&self) -> [u32; PRESS_COUNT_LEN] {
        self.press_counts
    }

    /// Time of the last tick.
    pub fn uptime(&self) -> Millis {
        self.now
    }

    /// Protocol counters.
    pub fn counters(&self) -> Counters {
        self.counters
    }

    /// Effects that did not fit in an `Effects` vector (should stay 0).
    pub fn dropped_effects(&self) -> u32 {
        self.dropped_effects
    }

    /// Alert / raffle LED state.
    pub fn indicators(&self) -> Indicators {
        self.indicators
    }

    /// Progress of the Wi-Fi scan.
    pub fn wifi_scan_state(&self) -> ScanState {
        self.wifi_scan
    }

    /// Access points from the last successful Wi-Fi scan, strongest first.
    pub fn access_points(&self) -> &[AccessPoint] {
        &self.aps
    }

    /// Last reported Wi-Fi link state, or the last link error.
    pub fn wifi_link(&self) -> Result<&WifiState, ErrorCode> {
        self.wifi.as_ref().map_err(|e| *e)
    }

    /// Progress of the BLE scan.
    pub fn ble_scan_state(&self) -> ScanState {
        self.ble_scan
    }

    /// Devices from the last successful BLE scan, strongest first.
    pub fn ble_devices(&self) -> &[BleDevice] {
        &self.ble
    }

    /// Name currently advertised over BLE; empty when not advertising.
    pub fn ble_advertising(&self) -> &str {
        &self.advertising
    }

    /// Text and colours for the HOST screen.
    pub fn host_text(&self) -> &HostText {
        &self.host
    }

    /// Outcome of the last GPIO probe access, if any.
    pub fn last_gpio(&self) -> Option<GpioResult> {
        self.gpio
    }

    /// Animation state of the pet (published in the PROBE block as
    /// `PET_SELECTED` / `PET_ACTIVE`).
    pub fn pet(&self) -> Pet {
        self.pet
    }

    /// Trade lobby and current trade.
    pub fn trade(&self) -> &TradeModel {
        &self.trade
    }

    /// Stored contacts.
    pub fn contacts(&self) -> &Contacts {
        &self.contacts
    }

    /// True if a radio key is loaded (outgoing packets are signed).
    pub fn signed(&self) -> bool {
        self.key.is_some()
    }

    /// Latest BLE air metrics.
    pub fn ble_metrics(&self) -> BleSnapshot {
        self.ble_stats
    }

    /// LED sync clock.
    pub fn sync(&self) -> SyncClock {
        self.sync
    }

    // ------------------------------------------------------------ internals

    fn count_rx(&mut self, garbage: bool) {
        self.counters.frames_rx = self.counters.frames_rx.wrapping_add(1);
        if garbage {
            self.counters.decode_errors = self.counters.decode_errors.wrapping_add(1);
        }
    }

    /// Books the effects of this call into the counters and redraw tracking.
    fn finish(&mut self, out: &Out) {
        let sent = out
            .fx
            .iter()
            .filter(|e| matches!(e, Effect::Send(_) | Effect::SendScan { .. }))
            .count();
        let sent = u32::try_from(sent).unwrap_or(u32::MAX);
        self.counters.frames_tx = self.counters.frames_tx.wrapping_add(sent);
        self.dropped_effects = self.dropped_effects.saturating_add(out.dropped);
        if out.fx.contains(&Effect::Redraw) {
            self.drawn = Some((self.now, self.leds));
        }
    }

    /// Re-renders the LED chain; emits `Leds` only if a colour changed.
    fn refresh_leds(&mut self, out: &mut Out) {
        let leds = leds::render(
            self.pattern,
            &self.manual,
            self.brightness,
            self.sync.network_time(self.now),
        );
        if leds != self.leds {
            self.leds = leds;
            out.push(Effect::Leds(leds));
        }
    }

    /// Switches screen; a real change emits `Redraw` then `ScreenChanged`.
    fn set_screen(&mut self, screen: Screen, out: &mut Out) {
        if screen != self.screen {
            self.screen = screen;
            out.redraw();
            out.event(Event_::Kind::Screen(ScreenChanged { screen }));
        }
    }

    /// Display effects and the `PetChanged` event for the pet having gone
    /// from `before` to its current state during this call.
    ///
    /// On the PET screen a mode change (selected or active) redraws, and a
    /// picture-only change animates unless a redraw is already queued.
    /// `PetChanged` is sent only if a slot is left, else on a later call.
    fn pet_output(&mut self, before: Pet, out: &mut Out) {
        let pair = (self.pet.selected(), self.pet.active());
        if self.screen == Screen::Pet {
            if pair != (before.selected(), before.active()) {
                out.redraw();
            } else if self.pet.change_from(&before).frame && !out.fx.contains(&Effect::Redraw) {
                out.push(Effect::Animate);
            }
        }
        if pair != self.pet_reported && out.fx.len() < MAX_EFFECTS {
            self.pet_reported = pair;
            out.event(Event_::Kind::Pet(PetChanged {
                selected: pair.0,
                active: pair.1,
            }));
        }
    }

    /// Requests a redraw if `screen` is the one showing.
    fn redraw_if_on(&self, screen: Screen, out: &mut Out) {
        if self.screen == screen {
            out.redraw();
        }
    }

    fn tick(&mut self, now: Millis, raw: ButtonMask, out: &mut Out) {
        self.now = now;
        // Advance first so a SELECT edge in this tick restarts at `now`.
        self.pet = self.pet.advance(now).0;
        let (debouncer, edges) =
            self.debouncer
                .clone()
                .update_limited(now, raw, MAX_EDGES_PER_TICK);
        self.debouncer = debouncer;
        for &(button, pressed) in &edges {
            out.event(Event_::Kind::Button(ButtonEvent {
                button,
                pressed,
                at_ms: now.0,
            }));
            if pressed {
                self.button_pressed(button, out);
            }
        }
        if !edges.is_empty() {
            self.redraw_if_on(Screen::Buttons, out);
        }
        self.trade_tick(out);
        let (sync, beacon) = self.sync.tick(now);
        self.sync = sync;
        if let Some(b) = beacon {
            out.push(Effect::SyncBroadcast {
                id: b.id,
                network_ms: b.network_ms,
            });
        }
        self.refresh_leds(out);
        let stale = match self.drawn {
            None => true,
            Some((at, shown)) => match self.screen {
                Screen::Leds => shown != self.leds && now.0.saturating_sub(at.0) >= LEDS_REDRAW_MS,
                _ => false,
            },
        };
        if stale {
            out.redraw();
        }
    }

    /// Navigation and screen actions for a debounced press.
    fn button_pressed(&mut self, button: Button, out: &mut Out) {
        if let Some(count) = button_index(button).and_then(|i| self.press_counts.get_mut(i)) {
            *count = count.wrapping_add(1);
        }
        let screen = self.screen;
        if matches!(screen, Screen::Trade | Screen::Contacts) && self.trade_button(button, out) {
            return;
        }
        match button {
            Button::Left => self.set_screen(cycle_screen(screen, -1), out),
            Button::Right => self.set_screen(cycle_screen(screen, 1), out),
            Button::Select => self.select(out),
            _ => {}
        }
    }

    /// SELECT: PET → next pet mode, LEDS → next pattern (OFF → RAINBOW →
    /// BREATHE), WIFI / BLE → start a scan unless busy. TRADE / CONTACTS
    /// buttons are handled by [`trade`] first.
    fn select(&mut self, out: &mut Out) {
        match self.screen {
            // Display effects and `PetChanged` follow from `pet_output`.
            Screen::Pet => self.pet = self.pet.next_mode(self.now),
            Screen::Leds => {
                self.pattern = leds::next_on_select(self.pattern);
                out.redraw();
            }
            Screen::Wifi if !self.wifi_scan.is_busy() => {
                self.wifi_scan = ScanState::Scanning;
                out.push(Effect::WifiScan { seq: None });
                out.redraw();
            }
            Screen::Ble if !self.ble_scan.is_busy() => {
                self.ble_scan = ScanState::Scanning;
                out.push(Effect::BleScan {
                    seq: None,
                    duration_ms: DEFAULT_BLE_SCAN_MS,
                });
                out.redraw();
            }
            _ => {}
        }
    }

    fn host(&mut self, msg: HostMsg, out: &mut Out) {
        let seq = msg.seq;
        let Some(cmd) = msg.cmd else {
            out.error(seq, ErrorCode::Unsupported, "no command");
            return;
        };
        match cmd {
            Cmd::Ping(p) => {
                out.respond(
                    seq,
                    Body::Pong(Pong {
                        nonce: p.nonce,
                        uptime_ms: self.now.0,
                    }),
                );
            }
            Cmd::GetInfo(_) => out.respond(seq, Body::Info(self.info_body())),
            Cmd::GetButtons(_) => {
                let mut press_counts = Vec::new();
                for &c in &self.press_counts {
                    press_counts.push(c).ok();
                }
                let pressed_mask = u32::from(self.pressed().0);
                out.respond(
                    seq,
                    Body::Buttons(ButtonState {
                        pressed_mask,
                        press_counts,
                    }),
                );
            }
            Cmd::SetLed(s) => {
                let slot = usize::try_from(s.index)
                    .ok()
                    .and_then(|i| self.manual.get_mut(i));
                match slot {
                    Some(slot) => {
                        *slot = Color::from_packed(s.color.map_or(0, |c| c.rgb));
                        self.pattern = LedPattern::Manual;
                        self.redraw_if_on(Screen::Leds, out);
                        out.ack(seq);
                    }
                    None => out.error(seq, ErrorCode::BadArgument, "led index > 7"),
                }
            }
            Cmd::SetLeds(s) => {
                for (slot, c) in self.manual.iter_mut().zip(s.colors.iter()) {
                    *slot = Color::from_packed(c.rgb);
                }
                self.pattern = LedPattern::Manual;
                self.redraw_if_on(Screen::Leds, out);
                out.ack(seq);
            }
            Cmd::SetPattern(s) => {
                let brightness = u8::try_from(s.brightness);
                match brightness {
                    Ok(b) if leds::is_supported(s.pattern) => {
                        self.pattern = s.pattern;
                        if b != 0 {
                            self.brightness = b;
                        }
                        self.redraw_if_on(Screen::Leds, out);
                        out.ack(seq);
                    }
                    _ => out.error(seq, ErrorCode::BadArgument, "pattern or brightness"),
                }
            }
            Cmd::SetIndicator(s) => {
                let valid = match s.which {
                    Indicator::Alert => {
                        self.indicators.alert = s.on;
                        true
                    }
                    Indicator::Raffle => {
                        self.indicators.raffle = s.on;
                        true
                    }
                    _ => false,
                };
                if valid {
                    out.push(Effect::Indicator(s.which, s.on));
                    out.ack(seq);
                } else {
                    out.error(seq, ErrorCode::BadArgument, "indicator");
                }
            }
            Cmd::ShowScreen(s) => {
                if SCREEN_ORDER.contains(&s.screen) {
                    self.set_screen(s.screen, out);
                    out.ack(seq);
                } else {
                    out.error(seq, ErrorCode::BadArgument, "screen");
                }
            }
            Cmd::DisplayText(t) => {
                self.host = HostText {
                    text: t.text,
                    fg: t.fg.map_or(Color::WHITE, |c| Color::from_packed(c.rgb)),
                    bg: t.bg.map_or(Color::BLACK, |c| Color::from_packed(c.rgb)),
                };
                self.set_screen(Screen::Host, out);
                out.redraw();
                out.ack(seq);
            }
            Cmd::DisplayFill(f) => {
                let color = Color::from_packed(f.color.map_or(0, |c| c.rgb));
                self.set_screen(Screen::Host, out);
                out.push(Effect::Draw(DrawOp::Fill(color)));
                out.ack(seq);
            }
            Cmd::DisplayRect(r) => match rect_op(&r) {
                Some(op) => {
                    self.set_screen(Screen::Host, out);
                    out.push(Effect::Draw(op));
                    out.ack(seq);
                }
                None => out.error(seq, ErrorCode::BadArgument, "rect outside panel"),
            },
            #[cfg(feature = "dev")]
            Cmd::GpioRead(g) => {
                if board::probe_allowed(g.pin) {
                    out.push(Effect::GpioRead { seq, pin: g.pin });
                } else {
                    out.error(seq, ErrorCode::PinNotAllowed, "not a probe pin");
                }
            }
            #[cfg(feature = "dev")]
            Cmd::GpioWrite(g) => {
                if board::probe_allowed(g.pin) {
                    out.push(Effect::GpioWrite {
                        seq,
                        pin: g.pin,
                        level: g.level,
                    });
                } else {
                    out.error(seq, ErrorCode::PinNotAllowed, "not a probe pin");
                }
            }
            #[cfg(feature = "dev")]
            Cmd::AdcRead(a) => {
                if !board::adc_allowed(a.pin) {
                    out.error(seq, ErrorCode::PinNotAllowed, "ADC1 pins: 1 3 5 6 7 8");
                } else if a.samples > crate::io::MAX_ADC_SAMPLES {
                    out.error(seq, ErrorCode::BadArgument, "samples > 64");
                } else {
                    let samples = match a.samples {
                        0 => crate::io::DEFAULT_ADC_SAMPLES,
                        n => n,
                    };
                    out.push(Effect::AdcRead { seq, pin: a.pin, samples });
                }
            }
            #[cfg(not(feature = "dev"))]
            Cmd::AdcRead(_) => {
                out.error(seq, ErrorCode::Unsupported, "adc needs dev build");
            }
            #[cfg(not(feature = "dev"))]
            Cmd::GpioRead(_) | Cmd::GpioWrite(_) => {
                out.error(seq, ErrorCode::Unsupported, "gpio needs dev build");
            }
            Cmd::WifiScan(_) => {
                if self.wifi_scan.is_busy() {
                    out.error(seq, ErrorCode::Busy, "wifi scan running");
                } else {
                    self.wifi_scan = ScanState::Scanning;
                    out.push(Effect::WifiScan { seq: Some(seq) });
                    self.redraw_if_on(Screen::Wifi, out);
                }
            }
            Cmd::WifiConnect(c) => {
                if c.ssid.is_empty() {
                    out.error(seq, ErrorCode::BadArgument, "empty ssid");
                } else {
                    out.push(Effect::WifiConnect {
                        seq,
                        ssid: c.ssid,
                        password: c.password,
                    });
                }
            }
            Cmd::WifiStatus(_) => out.push(Effect::WifiStatus { seq }),
            Cmd::BleScan(b) => {
                if self.ble_scan.is_busy() {
                    out.error(seq, ErrorCode::Busy, "ble scan running");
                } else {
                    let duration_ms = match b.duration_ms {
                        0 => DEFAULT_BLE_SCAN_MS,
                        d => d.min(MAX_BLE_SCAN_MS),
                    };
                    self.ble_scan = ScanState::Scanning;
                    out.push(Effect::BleScan {
                        seq: Some(seq),
                        duration_ms,
                    });
                    self.redraw_if_on(Screen::Ble, out);
                }
            }
            Cmd::BleAdvertise(a) => {
                self.adv_request = a.name.clone();
                out.push(Effect::BleAdvertise { seq, name: a.name });
            }
            Cmd::SetPetMode(m) => match self.pet.select(m.mode, self.now) {
                Ok(pet) => {
                    self.pet = pet;
                    self.set_screen(Screen::Pet, out);
                    out.ack(seq);
                }
                Err(_) => out.error(seq, ErrorCode::BadArgument, "pet mode"),
            },
            Cmd::GetBleMetrics(_) => {
                let m = self.ble_stats;
                let lobby_peers = u32::try_from(self.trade.lobby_len()).unwrap_or(u32::MAX);
                out.respond(
                    seq,
                    Body::BleMetrics(BleMetrics {
                        adv_per_s: u32::from(m.adv_per_s),
                        unique_10s: u32::from(m.unique_10s),
                        oz_badges: u32::from(m.oz_badges),
                        strongest_rssi: i32::from(m.strongest_rssi),
                        lobby_peers,
                    }),
                );
            }
            Cmd::GetSync(_) => out.respond(
                seq,
                Body::Sync(SyncState {
                    self_id: self.sync.self_id(),
                    leader_id: self.sync.leader(),
                    offset_ms: self.sync.offset_ms(),
                    peers: self.sync.peers(),
                }),
            ),
            cmd @ (Cmd::SetChi(_)
            | Cmd::TradeInvite(_)
            | Cmd::TradeCancel(_)
            | Cmd::ListContacts(_)
            | Cmd::ClearContacts(_)
            | Cmd::GetLobby(_)) => self.trade_host(seq, cmd, out),
            cmd @ (Cmd::WasmBegin(_)
            | Cmd::WasmChunk(_)
            | Cmd::WasmCommit(_)
            | Cmd::WasmStart(_)
            | Cmd::WasmStop(_)
            | Cmd::GetWasm(_)) => self.wasm_host(seq, cmd, out),
        }
    }

    /// Stores fresh BLE metrics; the BLE screen shows them.
    fn ble_stats(&mut self, stats: BleSnapshot, out: &mut Out) {
        if stats != self.ble_stats {
            self.ble_stats = stats;
            self.redraw_if_on(Screen::Ble, out);
        }
    }

    fn info_body(&self) -> Info {
        let mut mac = Vec::new();
        mac.extend_from_slice(&self.info.mac).ok();
        #[cfg(feature = "dev")]
        let probe_pins = {
            let mut pins = Vec::new();
            for pin in board::PROBE_PINS {
                pins.push(u32::from(pin.0)).ok();
            }
            pins
        };
        #[cfg(not(feature = "dev"))]
        let probe_pins = Vec::new();
        Info {
            firmware: truncated(self.info.firmware),
            chip: truncated(CHIP),
            mac,
            uptime_ms: self.now.0,
            free_heap: 0,
            probe_pins,
        }
    }

    fn gpio_done(
        &mut self,
        seq: u32,
        pin: u32,
        result: Result<bool, ErrorCode>,
        out: &mut Out,
    ) {
        self.gpio = Some(GpioResult { pin, result });
        self.redraw_if_on(Screen::Gpio, out);
        match result {
            Ok(level) => out.respond(seq, Body::Gpio(GpioLevel { pin, level })),
            Err(code) => out.error(seq, code, "gpio"),
        }
    }

    fn wifi_scanned(
        &mut self,
        seq: Option<u32>,
        result: Result<Vec<AccessPoint, MAX_APS>, ErrorCode>,
        out: &mut Out,
    ) {
        match result {
            Ok(aps) => {
                self.aps = aps;
                self.wifi_scan = ScanState::Done;
            }
            Err(code) => self.wifi_scan = ScanState::Failed(code),
        }
        self.redraw_if_on(Screen::Wifi, out);
        if let Some(seq) = seq {
            match self.wifi_scan {
                ScanState::Failed(code) => out.error(seq, code, "wifi scan"),
                _ => out.push(Effect::SendScan { seq, kind: ScanKind::Wifi }),
            }
        }
    }

    fn wifi_state(
        &mut self,
        seq: Option<u32>,
        result: Result<WifiState, ErrorCode>,
        out: &mut Out,
    ) {
        self.wifi = result;
        self.redraw_if_on(Screen::Wifi, out);
        if let Some(seq) = seq {
            match &self.wifi {
                Ok(state) => out.respond(seq, Body::WifiState(state.clone())),
                Err(code) => out.error(seq, *code, "wifi"),
            }
        }
    }

    fn ble_scanned(
        &mut self,
        seq: Option<u32>,
        result: Result<Vec<BleDevice, MAX_BLE>, ErrorCode>,
        out: &mut Out,
    ) {
        match result {
            Ok(devices) => {
                self.ble = devices;
                self.ble_scan = ScanState::Done;
            }
            Err(code) => self.ble_scan = ScanState::Failed(code),
        }
        self.redraw_if_on(Screen::Ble, out);
        if let Some(seq) = seq {
            match self.ble_scan {
                ScanState::Failed(code) => out.error(seq, code, "ble scan"),
                _ => out.push(Effect::SendScan { seq, kind: ScanKind::Ble }),
            }
        }
    }

    fn ble_advertised(&mut self, seq: u32, result: Result<(), ErrorCode>, out: &mut Out) {
        match result {
            Ok(()) => {
                self.advertising = self.adv_request.clone();
                self.redraw_if_on(Screen::Ble, out);
                out.ack(seq);
            }
            Err(code) => out.error(seq, code, "ble advertise"),
        }
    }
}

/// Validates a `DisplayRect` and clamps it to the panel.
///
/// Returns `None` if the origin is off the panel or the size is zero.
fn rect_op(r: &badge_proto::DisplayRect) -> Option<DrawOp> {
    let (width, height) = (u32::from(display::WIDTH), u32::from(display::HEIGHT));
    if r.x >= width || r.y >= height || r.w == 0 || r.h == 0 {
        return None;
    }
    let w = r.w.min(width - r.x);
    let h = r.h.min(height - r.y);
    Some(DrawOp::Rect {
        x: u16::try_from(r.x).ok()?,
        y: u16::try_from(r.y).ok()?,
        w: u16::try_from(w).ok()?,
        h: u16::try_from(h).ok()?,
        color: Color::from_packed(r.color.map_or(0, |c| c.rgb)),
    })
}

// Compile-time check that the tick budget documented above fits:
// edges (event + action each), Redraw/Animate, Leds, PetChanged,
// SyncBroadcast, and the trade lobby's share.
const _: () = assert!(MAX_EDGES_PER_TICK * 2 + 4 + TRADE_TICK_EFFECTS <= MAX_EFFECTS);

#[cfg(test)]
mod tests;
