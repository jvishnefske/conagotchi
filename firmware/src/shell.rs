//! The imperative shell: the 20 ms core loop around `badge_core::app::App`.
//!
//! Every tick [`run`]:
//!
//! 1. samples the buttons and feeds `Input::Tick`;
//! 2. drains up to [`MAX_HOST_PER_TICK`] host frames from RTT
//!    (`Input::Host` / `Input::HostGarbage`);
//! 3. drains up to [`MAX_RADIO_PER_TICK`] results from
//!    [`RADIO_EVENTS`];
//!    then up to [`MAX_WASM_PER_TICK`] inputs from the wasm task
//!    ([`wasm::try_next`]: command answers, payload logs, payload badge
//!    commands), then at most one request from the USB serial loader link
//!    ([`serial_link::try_next`], `Input::Host` with a seq in
//!    `probe_layout::SERIAL_SEQ`);
//! 4. takes a pending `PROBE` mailbox command as `Input::Host`;
//! 5. feeds follow-up inputs produced while executing effects (GPIO results,
//!    `BUSY` answers when the radio queue is full), bounded per tick;
//! 6. renders the UI once if any input asked for `Effect::Redraw`, otherwise
//!    redraws just the animated region if any input asked for `Effect::Animate`;
//! 7. publishes the `PROBE` block and, every [`HEARTBEAT_MS`], logs a
//!    heartbeat.
//!
//! Each input goes through `app.step(input, effects)`, the in-place form of
//! `App::handle`: the app and the one effect buffer live in static cells
//! built by [`prepare`] (owned by the shell state, never shared), so neither
//! is copied per input, and the effects are executed by reference straight
//! out of the buffer.
//!
//! Stack: all embassy tasks share the main stack. [`run`]'s future only
//! awaits the ticker; one tick is the out-of-line `Shell::tick`, so the
//! tick's temporaries are on the stack only while it runs, not in the frame
//! of every poll (inlined, they made that frame about 43 KB).
//!
//! Effects are executed in order: `Send` → RTT (a button event also goes to
//! the wasm task, [`wasm::button`], as a `badge/buttons` sample for the
//! payload; a `WasmLog` event also goes to [`serial_link::log`]; a
//! `Response` whose seq is in `SERIAL_SEQ` goes only to
//! [`serial_link::answer`], never to RTT), `Leds` → RMT,
//! `Indicator` → GPIO, `Draw` → panel, `GpioRead`/`GpioWrite` → probe pins
//! with the result fed back, radio effects → [`RADIO_CMDS`], `LinkSend` →
//! [`link_uart::send`], `SaveContact`/`ClearContacts` → [`storage::submit`]
//! (both queue without waiting and drop when full), `Wasm` →
//! [`wasm::submit`] (answered `BUSY` when its queue is full; the wasm task
//! answers every command it accepts exactly once, drained in step 3). `Redraw` is
//! coalesced to one render per tick (a full render also covers a pending
//! `Animate`); a pending render is flushed before any `Draw` so drawing order
//! is preserved.
//!
//! Hardware that failed to initialise is `None` in [`Hardware`]; the loop
//! keeps running and the corresponding effects are skipped.
//!
//! The GPIO probe exists only with the `dev` feature. It is not part of
//! [`Hardware`]: it sits in `gpio_probe`'s slot, shared with the wasm
//! task's payload GPIO calls, and each request borrows it for one
//! synchronous call (`gpio_probe::with`; no probe answers `UNSUPPORTED`, a
//! probe in use `BUSY`). Without `dev`, `GpioRead` / `GpioWrite` /
//! `AdcRead` are always answered with `UNSUPPORTED` (the core does not emit
//! them in that build).

use badge_core::app::{App, BoardInfo};
use badge_core::io::{ButtonMask, Effect, Effects, Input, Millis};
use badge_core::ui;
use badge_proto::ErrorCode;
use defmt::{info, warn};
use embassy_sync::channel::TrySendError;
use embassy_time::{Duration, Instant, Ticker};
use heapless::Deque;
use static_cell::{ConstStaticCell, StaticCell};

use crate::buttons::Buttons;
use crate::display::Display;
#[cfg(feature = "dev")]
use crate::gpio_probe;
use crate::leds::{Indicators, Ws2812};
use crate::probe::{self, Mailbox, Snapshot};
use crate::radio::{RADIO_CMDS, RADIO_EVENTS};
use crate::rtt::HostLink;
use crate::storage::{self, StorageCmd};
use crate::wasm;
use crate::link_uart;
use crate::serial_link;

/// Core loop period.
pub const TICK_MS: u64 = 20;
/// Host frames handled per tick.
pub const MAX_HOST_PER_TICK: usize = 4;
/// Radio results handled per tick.
pub const MAX_RADIO_PER_TICK: usize = 4;
/// Wasm answers, app logs and app host messages handled per tick.
pub const MAX_WASM_PER_TICK: usize = 4;
/// Capacity of the follow-up input queue.
const FOLLOWUP_CAP: usize = 8;
/// Follow-up inputs handled per tick.
const MAX_FOLLOWUPS_PER_TICK: usize = 16;
/// Interval between heartbeat log lines.
pub const HEARTBEAT_MS: u64 = 5_000;

/// The drivers the shell executes effects on.
pub struct Hardware {
    /// RTT proto link.
    pub link: &'static mut HostLink,
    /// GC9A01 panel, if it initialised.
    pub display: Option<Display>,
    /// WS2812B chain, if the RMT initialised.
    pub leds: Option<Ws2812>,
    /// Alert / raffle LEDs.
    pub indicators: Indicators,
    /// Button inputs.
    pub buttons: Buttons,
}

/// Counters for the heartbeat log.
#[derive(Debug, Clone, Copy, Default)]
struct Stats {
    ticks: u32,
    renders: u32,
    render_errors: u32,
    send_errors: u32,
    led_errors: u32,
    draw_errors: u32,
    followups_dropped: u32,
    radio_busy: u32,
    max_tick_us: u64,
}

/// Shell state that persists across ticks.
///
/// The large parts (the 5 KB app, the effect buffer of `MAX_EFFECTS`
/// effects and the follow-up queue) live in static cells and are only
/// borrowed here, so neither this value nor the core loop's future copies
/// them, and building them never happens inside a poll of the loop.
pub struct Shell {
    app: &'static mut App,
    effects: &'static mut Effects,
    io: Io,
    last_buttons: ButtonMask,
    next_heartbeat: u64,
}

/// The part of the shell that executes effects: drivers, the follow-up
/// queue, pending renders and counters. Separate from the app and effect
/// buffer so that effects are executed by reference straight out of the
/// buffer, without copying each one (an `Effect` is several hundred bytes)
/// onto the stack.
struct Io {
    hw: Hardware,
    followups: &'static mut Deque<Input, FOLLOWUP_CAP>,
    redraw_pending: bool,
    animate_pending: bool,
    stats: Stats,
}

static SHELL: StaticCell<Shell> = StaticCell::new();
static APP: StaticCell<App> = StaticCell::new();
static EFFECTS: ConstStaticCell<Effects> = ConstStaticCell::new(Effects::new());
static FOLLOWUPS: ConstStaticCell<Deque<Input, FOLLOWUP_CAP>> = ConstStaticCell::new(Deque::new());

/// Builds the shell state in its static cell (the app in its own), out of
/// line, so that none of it is constructed or copied inside a poll of the
/// core loop. Call once, before [`run`]; a second call is a startup bug.
#[inline(never)]
pub fn prepare(hw: Hardware, info: BoardInfo) -> &'static mut Shell {
    SHELL.init(Shell::new(hw, info))
}

/// Runs the core loop forever on the state from [`prepare`].
///
/// The future only holds the state reference and the ticker; it awaits the
/// ticker and calls [`Shell::tick`], which is kept out of line: inlined,
/// the decoding, input and effect temporaries of one tick were part of the
/// frame of every poll of this future (about 43 KB of the main stack).
pub async fn run(shell: &'static mut Shell) -> ! {
    let mut ticker = Ticker::every(Duration::from_millis(TICK_MS));
    info!("core loop running, tick {} ms", TICK_MS);
    loop {
        ticker.next().await;
        shell.tick();
    }
}

impl Shell {
    /// Builds the shell state; the app is constructed in its static cell
    /// here, out of line of the core loop's poll.
    #[inline(never)]
    fn new(hw: Hardware, info: BoardInfo) -> Self {
        Shell {
            // `run` is called once (from `main`), so each cell is
            // initialised once: a second call is a startup bug.
            app: APP.init_with(|| App::new(info)),
            effects: EFFECTS.take(),
            io: Io {
                hw,
                followups: FOLLOWUPS.take(),
                redraw_pending: false,
                animate_pending: false,
                stats: Stats::default(),
            },
            last_buttons: ButtonMask::default(),
            next_heartbeat: HEARTBEAT_MS,
        }
    }

    /// One core loop tick (steps 1 to 7 of the module docs).
    #[inline(never)]
    fn tick(&mut self) {
        let start = Instant::now();
        let now = Millis(start.as_millis());

        let buttons = self.io.hw.buttons.sample();
        if buttons != self.last_buttons {
            info!("buttons raw {=u8:#07b}", buttons.0);
            self.last_buttons = buttons;
        }
        self.feed(Input::Tick { now, buttons });

        for _ in 0..MAX_HOST_PER_TICK {
            match self.io.hw.link.next_input() {
                Some(input) => self.feed(input),
                None => break,
            }
        }

        for _ in 0..MAX_RADIO_PER_TICK {
            match RADIO_EVENTS.try_receive() {
                Ok(input) => self.feed(input),
                Err(_) => break,
            }
        }

        for _ in 0..MAX_WASM_PER_TICK {
            match wasm::try_next() {
                Some(input) => self.feed(input),
                None => break,
            }
        }

        if let Some(input) = serial_link::try_next() {
            self.feed(input);
        }

        match probe::take_mailbox() {
            Mailbox::Empty => {}
            Mailbox::Command(msg) => self.feed(Input::Host(msg)),
            Mailbox::Unknown(cmd) => warn!("PROBE mailbox: unknown command {}", cmd),
        }

        for _ in 0..MAX_FOLLOWUPS_PER_TICK {
            match self.io.followups.pop_front() {
                Some(input) => self.feed(input),
                None => break,
            }
        }

        self.io.flush_redraw(self.app);
        probe::publish(&snapshot(self.app));

        self.io.stats.ticks = self.io.stats.ticks.wrapping_add(1);
        let elapsed = start.elapsed().as_micros();
        self.io.stats.max_tick_us = self.io.stats.max_tick_us.max(elapsed);
        if now.0 >= self.next_heartbeat {
            self.next_heartbeat = now.0.saturating_add(HEARTBEAT_MS);
            self.io.heartbeat(self.app);
        }
    }

    /// Feeds one input to the app and executes the resulting effects.
    fn feed(&mut self, input: Input) {
        self.app.step(input, self.effects);
        // In order, by reference: no effect is copied out of the buffer.
        for effect in self.effects.iter() {
            self.io.execute(effect, self.app);
        }
    }
}

impl Io {
    /// Carries out one effect; `app` is the state after the input.
    fn execute(&mut self, effect: &Effect, app: &App) {
        match effect {
            Effect::Send(msg) => {
                if let Some((seq, answer)) = badge_mw::serial::serial_answer(msg) {
                    serial_link::answer(seq, answer);
                    return;
                }
                if let Some(text) = badge_mw::serial::wasm_log(msg) {
                    serial_link::log(text);
                }
                if let Some((button, pressed)) = badge_mw::payload::button_edge(msg) {
                    wasm::button(button, pressed);
                }
                if let Err(e) = self.hw.link.send(msg) {
                    self.stats.send_errors = self.stats.send_errors.wrapping_add(1);
                    warn!("proto-up send failed: {}", e);
                }
            }
            // Scan results are built from the next state here, so the large
            // lists never travel inside an `Effect`.
            &Effect::SendScan { seq, kind } => {
                if let Err(e) = self.hw.link.send(&app.scan_response(kind, seq)) {
                    self.stats.send_errors = self.stats.send_errors.wrapping_add(1);
                    warn!("proto-up send failed: {}", e);
                }
            }
            Effect::Leds(colors) => {
                if let Some(leds) = self.hw.leds.as_mut()
                    && let Err(e) = leds.write(colors)
                {
                    self.stats.led_errors = self.stats.led_errors.wrapping_add(1);
                    warn!("WS2812 write failed: {}", e);
                }
            }
            &Effect::Indicator(which, on) => self.hw.indicators.set(which, on),
            Effect::Redraw => self.redraw_pending = true,
            Effect::Animate => self.animate_pending = true,
            &Effect::Draw(op) => {
                self.flush_redraw(app);
                if let Some(display) = self.hw.display.as_mut()
                    && let Err(e) = display.draw(op)
                {
                    self.stats.draw_errors = self.stats.draw_errors.wrapping_add(1);
                    warn!("display draw failed: {}", e);
                }
            }
            #[cfg(feature = "dev")]
            &Effect::GpioRead { seq, pin } => {
                let result = gpio_probe::with(|gpio| gpio.read(pin)).unwrap_or_else(|e| Err(e.error_code()));
                self.follow_up(Input::Gpio { seq, pin, result });
            }
            #[cfg(feature = "dev")]
            &Effect::GpioWrite { seq, pin, level } => {
                let result = gpio_probe::with(|gpio| gpio.write(pin, level)).unwrap_or_else(|e| Err(e.error_code()));
                self.follow_up(Input::Gpio { seq, pin, result });
            }
            #[cfg(feature = "dev")]
            &Effect::AdcRead { seq, pin, samples } => {
                let result = gpio_probe::with(|gpio| gpio.adc_read(pin, samples)).unwrap_or_else(|e| Err(e.error_code()));
                self.follow_up(Input::Adc { seq, result });
            }
            // No probe pins without `dev` (the core never asks then anyway).
            #[cfg(not(feature = "dev"))]
            &(Effect::GpioRead { seq, pin } | Effect::GpioWrite { seq, pin, .. }) => {
                let result = Err(ErrorCode::Unsupported);
                self.follow_up(Input::Gpio { seq, pin, result });
            }
            #[cfg(not(feature = "dev"))]
            &Effect::AdcRead { seq, .. } => {
                self.follow_up(Input::Adc { seq, result: Err(ErrorCode::Unsupported) });
            }
            radio @ (Effect::WifiScan { .. }
            | Effect::WifiConnect { .. }
            | Effect::WifiStatus { .. }
            | Effect::BleScan { .. }
            | Effect::BleAdvertise { .. }
            | Effect::Beacon(_)
            | Effect::SyncBroadcast { .. }) => {
                if let Err(TrySendError::Full(rejected)) = RADIO_CMDS.try_send(radio.clone()) {
                    self.stats.radio_busy = self.stats.radio_busy.wrapping_add(1);
                    if let Some(reply) = radio_error(&rejected, ErrorCode::Busy) {
                        self.follow_up(reply);
                    }
                }
            }
            Effect::LinkSend(frame) => link_uart::send(frame.clone()),
            Effect::SaveContact(contact) => storage::submit(StorageCmd::Save(contact.clone())),
            Effect::ClearContacts => storage::submit(StorageCmd::Clear),
            // The wasm task answers on its own queue (drained below); a
            // full command queue is answered BUSY here.
            Effect::Wasm { seq, cmd } => {
                if let Some(busy) = wasm::submit(*seq, cmd.clone()) {
                    self.follow_up(busy);
                }
            }
        }
    }

    /// Queues an input produced by an effect for later in this tick.
    fn follow_up(&mut self, input: Input) {
        if self.followups.push_back(input).is_err() {
            self.stats.followups_dropped = self.stats.followups_dropped.wrapping_add(1);
            warn!("follow-up queue full; input dropped");
        }
    }

    /// Renders the whole UI if a redraw is pending, else only the animated
    /// region if an animation frame is pending.
    fn flush_redraw(&mut self, app: &App) {
        let animate = core::mem::take(&mut self.animate_pending);
        if !core::mem::take(&mut self.redraw_pending) {
            if animate
                && let Some(display) = self.hw.display.as_mut()
                && ui::render_animation(app, display.target()).is_err()
            {
                self.stats.render_errors = self.stats.render_errors.wrapping_add(1);
                warn!("display: animation render failed");
            }
            return;
        }
        let Some(display) = self.hw.display.as_mut() else {
            return;
        };
        let first = self.stats.renders == 0 && self.stats.render_errors == 0;
        match ui::render(app, display.target()) {
            Ok(()) => {
                self.stats.renders = self.stats.renders.wrapping_add(1);
                if first {
                    info!("display: first render Ok");
                }
            }
            Err(_) => {
                self.stats.render_errors = self.stats.render_errors.wrapping_add(1);
                warn!("display: render failed");
            }
        }
    }

    /// Logs the periodic heartbeat.
    fn heartbeat(&self, app: &App) {
        let c = app.counters();
        let s = &self.stats;
        info!(
            "heartbeat up={}ms ticks={} screen={} pattern={} pressed={=u8:#07b} rx={} tx={} decode_err={} renders={} render_err={} send_err={} led_err={} draw_err={} busy={} fu_drop={} max_tick={}us",
            app.uptime().0,
            s.ticks,
            app.screen().0,
            app.pattern().0,
            app.pressed().0,
            c.frames_rx,
            c.frames_tx,
            c.decode_errors,
            s.renders,
            s.render_errors,
            s.send_errors,
            s.led_errors,
            s.draw_errors,
            s.radio_busy,
            s.followups_dropped,
            s.max_tick_us,
        );
    }
}

/// The error answer for a radio effect that could not be carried out.
fn radio_error(effect: &Effect, code: ErrorCode) -> Option<Input> {
    match effect {
        Effect::WifiScan { seq } => Some(Input::WifiScanned { seq: *seq, result: Err(code) }),
        Effect::WifiConnect { seq, .. } | Effect::WifiStatus { seq } => {
            Some(Input::Wifi { seq: Some(*seq), result: Err(code) })
        }
        Effect::BleScan { seq, .. } => Some(Input::BleScanned { seq: *seq, result: Err(code) }),
        Effect::BleAdvertise { seq, .. } => Some(Input::BleAdvertised { seq: *seq, result: Err(code) }),
        _ => None,
    }
}

/// Collects the PROBE block values from the app accessors.
fn snapshot(app: &App) -> Snapshot {
    let c = app.counters();
    Snapshot {
        uptime_ms: app.uptime().0,
        buttons: app.pressed().0,
        press_counts: app.press_counts(),
        screen: app.screen().0,
        pattern: app.pattern().0,
        brightness: app.brightness(),
        leds: app.leds(),
        frames_rx: c.frames_rx,
        frames_tx: c.frames_tx,
        decode_errors: c.decode_errors,
        pet_selected: app.pet().selected().0,
        pet_active: app.pet().active().0,
        sync_offset_ms: app.sync().offset_ms(),
        sync_leader: app.sync().leader(),
        contacts: u32::try_from(app.contacts().len()).unwrap_or(u32::MAX),
        trade_state: app.trade().state().0,
        trade_peer: app.trade().peer().map_or(0, |p| p.to_u32()),
        chi: app.trade().chi().0,
    }
}

