//! The wasm payload task: runs one middleware payload ("mw" ABI) with
//! middleware's `wasm-host` (wasmi) next to the core loop, and keeps the
//! installed image in the `payload` flash partition so it survives resets.
//!
//! # Commands and answers
//!
//! The shell hands every `Effect::Wasm { seq, cmd }` to [`submit`] (a
//! two-slot queue; when it is full the shell answers `BUSY` itself). The task
//! executes one command at a time and answers it with exactly one
//! `Input::Wasm { seq, result }` on a separate reply queue, sent with an
//! awaiting `send` so an answer is never dropped. The core has already
//! validated the upload (order, length, CRC; `middleware::loader::Loader`).
//!
//! - `Erase { len }` drops a running payload, then erases the partition for
//!   `len` bytes: LOADING.
//! - `Write { offset, data }` writes a chunk (only while LOADING, else
//!   `BAD_ARGUMENT`); `size` counts the bytes received.
//! - `Activate { len, crc }` writes the header and verifies the image
//!   against its CRC by reading it back (only while LOADING): STOPPED. The
//!   image is now persistent and autostarts on boot.
//! - `Start` (`BAD_ARGUMENT` unless an image is installed) drops a running
//!   payload, checks the heap (`BUSY` if there is less free than
//!   `badge_mw::payload::load_heap_needed`), reads the image into a heap
//!   buffer of its exact length (at most `PAYLOAD_MAX_IMAGE`, CRC checked),
//!   and calls `Payload::load` with [`LIMITS`]; the buffer is freed right
//!   after. RUNNING on success; on a load or `init` error the payload stays
//!   installed and STOPPED with the error text.
//! - `Stop` drops the payload: STOPPED (no effect while EMPTY or LOADING).
//! - `Status` only reports.
//!
//! Every other outcome is `Ok(WasmStatus)` (see
//! `badge_mw::payload::Lifecycle`): a flash failure makes it EMPTY with the
//! error text (the partition can no longer be trusted), a trap or fuel
//! exhaustion TRAPPED with the trap text. The payload is dropped on a trap,
//! which frees its heap; `Start` loads it again. The flash work happens in
//! the storage task, which owns the `FLASH` peripheral
//! ([`storage::payload`], see `storage` for the ownership design).
//!
//! # Boot
//!
//! On start the task asks the storage task whether a valid image is
//! installed (header and CRC). If so it starts it, unless BOOT was held at
//! reset ([`start`]'s `boot_held`, sampled by `main`), which is safe mode:
//! the payload stays installed and STOPPED and a warning is logged
//! (`badge_mw::payload::boot_action`). The core loop does not wait for any
//! of this; commands queue meanwhile.
//!
//! # Ticks and delivery
//!
//! The task sleeps until the earliest of: a command, the payload's next
//! `tick` (`Payload::next_tick`, the period it chose with `set_tick`; at
//! most [`IDLE_MS`] while none is due), a sample on [`mw::APP_SAMPLES`]
//! (app keys heard over ESP-NOW or BLE), or a button edge from [`button`].
//! `tick` gets the embassy time in milliseconds (wrapping `u32`); the
//! badge's network time lives in the core and is not shared. Samples and
//! edges go to `Payload::deliver`, which runs `on_message` only for keys
//! the payload subscribed to; a button edge is the `badge/buttons` sample
//! `[button, pressed]` (`badge_mw::keys::button_sample`). Without a running
//! payload they are drained and dropped.
//!
//! # Device (host calls)
//!
//! [`BadgeDevice`] implements `wasm_host::Device` and never blocks:
//!
//! - `log` → defmt and an `Input::WasmLog` (cut to `WASM_LOG_LEN` bytes);
//! - `publish` → `badge_mw::keys::route_payload_publish`: a `badge/*` key
//!   becomes `Input::Host` with seq [`WASM_SEQ`] (`SetLeds`, `DisplayText`,
//!   `SetPetMode`), exactly as if the host had sent it, and an unchanged LED
//!   frame is not sent again; an `app/...` key goes to [`mw::publish_app`]
//!   (ESP-NOW and BLE queues, at most `mw::PAYLOAD_LEN` bytes). Unknown keys
//!   are `InvalidKey`, malformed payloads `InvalidArgument`, full queues
//!   `Transport`;
//! - `now_ms` → embassy time;
//! - `gpio_*` → `Unsupported` in every build. The probe pins belong to the
//!   shell's `GpioProbe` (dev builds); routing payload GPIO through the
//!   shell is a follow-up;
//! - the ABI 2 calls (IRQs, timers, SPI, UART) keep wasm-host's
//!   `Unsupported` defaults.
//!
//! Inputs for the core go through a bounded event queue that the shell
//! drains with [`try_next`]; a full queue drops the input and counts it,
//! and the count is logged every [`REPORT_MS`] while it grows, with the
//! longest single call into the payload.
//!
//! # Limits and heap
//!
//! [`LIMITS`]: `board::PAYLOAD_MAX_MEMORY` (16 KiB) of linear memory, the
//! size middleware's `payloads/scripts/build-byte-pages.sh` builds for, and
//! `board::PAYLOAD_FUEL_PER_CALL` (50 000) per call into the payload; both
//! are shared with `badgectl sim`. wasmi charges fuel for
//! lazily compiling a function on its first call as well as for executing
//! it; middleware's example payloads need 10 000 to 20 000 for `init`, and
//! a single function over about 10 KiB of code cannot complete its first
//! call. The task shares the executor with the core loop, so the fuel also
//! bounds how long one call stalls the badge.
//!
//! wasmi allocates from the shared esp-alloc heap; `main` adds a
//! [`HEAP_BYTES`] region for it (esp-alloc cannot reserve a region for one
//! user, so the payload and esp-radio share all regions). A payload uses
//! about 28 KiB plus twice its image at peak (measured, see
//! `badge_mw::payload::load_heap_needed`), so `Start` refuses (`BUSY`) when
//! less than that bound is free: an allocation failure inside wasmi would
//! halt the badge.

use core::sync::atomic::{AtomicU32, Ordering};

use badge_core::board::{PAYLOAD_FUEL_PER_CALL, PAYLOAD_MAX_IMAGE, PAYLOAD_MAX_MEMORY};
use badge_core::io::{Input, WASM_LOG_LEN, WasmCmd};
use badge_core::probe_layout::WASM_SEQ;
use badge_mw::keys::{self, BADGE_LEDS, PayloadRoute};
use badge_mw::payload::{self, BootAction, Lifecycle};
use badge_proto::{Button, ErrorCode, HostMsg, HostMsg_::Cmd, WasmStatus};
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{Either4, select4};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, TrySendError};
use embassy_time::{Instant, Timer};
use esp_base::dispatch::tick_delay;
use heapless::{String, Vec};
use middleware::Key;
use wasm_host::{AbiError, Device, Limits, Payload, Pull};

use crate::mw::{self, MwMessage};
use crate::storage::{self, PayloadDone, PayloadError, PayloadOp};

/// The payload's resource limits: `board::PAYLOAD_MAX_MEMORY` and
/// `board::PAYLOAD_FUEL_PER_CALL`, the values `badgectl sim` uses too.
pub const LIMITS: Limits = Limits::bytes(PAYLOAD_MAX_MEMORY).with_fuel(PAYLOAD_FUEL_PER_CALL);
/// Size of the heap region `main` adds for the payload runtime.
///
/// This trades heap for main stack, which gets all RAM left over. Measured on
/// hardware (2026-10-25, stack painted over the debug link, then payload
/// start/stop, Wi-Fi and BLE scans and a GATT RPC): peak stack use is about
/// 53.5 KB, because every embassy task, including this one compiling a
/// payload, runs on the one main stack. 16 KiB here leaves about 64.7 KB of
/// stack (about 11 KB margin). With it, a ~6 KB payload starts with about
/// 36 KB of heap still free; payloads near `PAYLOAD_MAX_IMAGE` may be refused
/// with BUSY by the free-heap guard.
pub const HEAP_BYTES: usize = 16 * 1024;
/// Longest sleep when no tick is due (the loop then re-checks).
pub const IDLE_MS: u32 = 1_000;
/// Interval between drop / timing reports.
pub const REPORT_MS: u64 = 5_000;
/// Capacity of the button edge queue.
pub const BUTTON_QUEUE: usize = 8;

/// One request from the shell.
struct Request {
    seq: u32,
    cmd: WasmCmd,
}

/// The answer to one request: its seq and `Input::Wasm::result`.
type Reply = (u32, Result<WasmStatus, ErrorCode>);

/// A payload output for the core, kept smaller than an [`Input`] so the
/// event queue stays small; [`try_next`] turns it into the input.
#[allow(clippy::large_enum_variant, reason = "fixed-size queue slots; boxing would allocate")]
enum AppEvent {
    /// `log` line (`Input::WasmLog`).
    Log(String<WASM_LOG_LEN>),
    /// A routed `badge/*` publish (`Input::Host`, seq [`WASM_SEQ`]).
    Host(Cmd),
}

/// Why [`start`] failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum WasmError {
    /// The wasm task is already running.
    Spawn,
}

static CMDS: Channel<CriticalSectionRawMutex, Request, 2> = Channel::new();
static REPLIES: Channel<CriticalSectionRawMutex, Reply, 2> = Channel::new();
static EVENTS: Channel<CriticalSectionRawMutex, AppEvent, 8> = Channel::new();
static BUTTONS: Channel<CriticalSectionRawMutex, (Button, bool), BUTTON_QUEUE> = Channel::new();
/// Inputs dropped because [`EVENTS`] was full.
static DROPPED: AtomicU32 = AtomicU32::new(0);
/// Longest single call into the payload, in microseconds.
static MAX_CALL_US: AtomicU32 = AtomicU32::new(0);

/// Spawns the wasm task. `boot_held`: BOOT was held at reset (safe mode, no
/// autostart).
pub fn start(spawner: &Spawner, boot_held: bool) -> Result<(), WasmError> {
    spawner.spawn(wasm_task(boot_held).map_err(|_| WasmError::Spawn)?);
    Ok(())
}

/// Queues a wasm command for the task. Returns the `BUSY` answer to feed
/// back if the queue is full.
pub fn submit(seq: u32, cmd: WasmCmd) -> Option<Input> {
    match CMDS.try_send(Request { seq, cmd }) {
        Ok(()) => None,
        Err(TrySendError::Full(rejected)) => Some(Input::Wasm { seq: rejected.seq, result: Err(ErrorCode::Busy) }),
    }
}

/// Offers a button edge to the payload (`badge/buttons`); dropped if the
/// queue is full.
pub fn button(button: Button, pressed: bool) {
    if BUTTONS.try_send((button, pressed)).is_err() {
        DROPPED.fetch_add(1, Ordering::Relaxed);
    }
}

/// The next input from the wasm task for the core: answers first, then
/// payload logs and badge commands.
pub fn try_next() -> Option<Input> {
    if let Ok((seq, result)) = REPLIES.try_receive() {
        return Some(Input::Wasm { seq, result });
    }
    EVENTS.try_receive().ok().map(|event| match event {
        AppEvent::Log(text) => Input::WasmLog(text),
        AppEvent::Host(cmd) => Input::Host(HostMsg { seq: WASM_SEQ, cmd: Some(cmd) }),
    })
}

/// Offers a payload output to the shell without waiting; false (and
/// counted) if dropped.
fn emit(event: AppEvent) -> bool {
    let sent = EVENTS.try_send(event).is_ok();
    if !sent {
        DROPPED.fetch_add(1, Ordering::Relaxed);
    }
    sent
}

/// Embassy time in milliseconds on the ABI's wrapping `u32` clock.
fn now_ms() -> u32 {
    // Truncation is the wrap.
    Instant::now().as_millis() as u32
}

/// The badge side of the payload's host calls (see the module docs).
#[derive(Default)]
pub struct BadgeDevice {
    /// Last LED frame sent to the core.
    last_leds: Option<[u8; 24]>,
}

impl BadgeDevice {
    /// Hands a routed `badge/*` command to the core, skipping an unchanged
    /// LED frame.
    fn local(&mut self, key: Key<'_>, data: &[u8], cmd: Cmd) -> Result<(), AbiError> {
        let leds = if key == BADGE_LEDS { <[u8; 24]>::try_from(data).ok() } else { None };
        if leds.is_some() && leds == self.last_leds {
            return Ok(());
        }
        if !emit(AppEvent::Host(cmd)) {
            return Err(AbiError::Transport);
        }
        if leds.is_some() {
            self.last_leds = leds;
        }
        Ok(())
    }
}

/// Offers an `app/...` publish to the radio transports.
fn radio(key: Key<'_>, data: &[u8]) -> Result<(), AbiError> {
    let mut msg = MwMessage { key: String::new(), payload: Vec::new() };
    msg.key.push_str(key.as_str()).map_err(|_| AbiError::InvalidKey)?;
    msg.payload.extend_from_slice(data).map_err(|_| AbiError::InvalidArgument)?;
    if mw::publish_app(&msg) == 0 { Err(AbiError::Transport) } else { Ok(()) }
}

impl Device for BadgeDevice {
    fn log(&mut self, line: &str) {
        info!("wasm: {=str}", line);
        emit(AppEvent::Log(payload::truncate(line)));
    }

    fn publish(&mut self, key: Key<'_>, data: &[u8]) -> Result<(), AbiError> {
        match keys::route_payload_publish(key, data).map_err(payload::abi_error)? {
            PayloadRoute::Local(cmd) => self.local(key, data, cmd),
            PayloadRoute::Radio => radio(key, data),
        }
    }

    fn now_ms(&mut self) -> u32 {
        now_ms()
    }

    fn gpio_output(&mut self, _pin: u8) -> Result<(), AbiError> {
        Err(AbiError::Unsupported)
    }

    fn gpio_input(&mut self, _pin: u8, _pull: Pull) -> Result<(), AbiError> {
        Err(AbiError::Unsupported)
    }

    fn gpio_write(&mut self, _pin: u8, _high: bool) -> Result<(), AbiError> {
        Err(AbiError::Unsupported)
    }

    fn gpio_read(&mut self, _pin: u8) -> Result<bool, AbiError> {
        Err(AbiError::Unsupported)
    }
}

/// Records how long a call into the payload took.
fn timed<R>(f: impl FnOnce() -> R) -> R {
    let start = Instant::now();
    let r = f();
    MAX_CALL_US.fetch_max(u32::try_from(start.elapsed().as_micros()).unwrap_or(u32::MAX), Ordering::Relaxed);
    r
}

/// The task's state: the loaded payload (if running) and its lifecycle.
struct Runner {
    payload: Option<Payload<BadgeDevice>>,
    life: Lifecycle,
}

impl Runner {
    /// The status answer.
    fn status(&self) -> WasmStatus {
        self.life.status(PAYLOAD_MAX_IMAGE)
    }

    /// Moves the lifecycle on with `f`.
    fn update(&mut self, f: impl FnOnce(Lifecycle) -> Lifecycle) {
        let life = core::mem::replace(&mut self.life, Lifecycle::boot(None));
        self.life = f(life);
    }

    /// A flash operation failed: no trusted image remains.
    fn storage_failed(&mut self, e: PayloadError) {
        warn!("wasm: payload storage failed: {}", defmt::Display2Format(&e));
        self.payload = None;
        self.update(|l| l.storage_failed(e));
    }

    /// Drops the payload after a trap.
    fn trapped(&mut self, trap: wasm_host::Trap) {
        warn!("wasm: payload stopped: {}", defmt::Display2Format(&trap));
        self.payload = None;
        self.update(|l| l.trapped(trap));
    }

    /// Loads and starts the installed image. `Err` is the command's error
    /// answer (`BAD_ARGUMENT` with nothing installed, `BUSY` when the heap is
    /// too low); load failures are `Ok` with the status STOPPED.
    async fn start(&mut self) -> Result<(), ErrorCode> {
        if !self.life.startable() {
            return Err(ErrorCode::BadArgument);
        }
        // Free the old instance before measuring the heap.
        self.payload = None;
        self.update(Lifecycle::stopped);
        let size = self.life.status(0).size;
        let need = usize::try_from(payload::load_heap_needed(size, PAYLOAD_MAX_MEMORY)).unwrap_or(usize::MAX);
        let free = esp_alloc::HEAP.free();
        if free < need {
            warn!("wasm: only {} B heap free, need {} to start", free, need);
            return Err(ErrorCode::Busy);
        }
        let image = match storage::payload(PayloadOp::Load).await {
            Ok(PayloadDone::Image(image)) => image,
            Ok(_) => return Err(ErrorCode::BadArgument),
            Err(PayloadError::NoMemory) => return Err(ErrorCode::Busy),
            Err(e) => {
                self.storage_failed(e);
                return Ok(());
            }
        };
        let loaded = timed(|| Payload::load(&image, BadgeDevice::default(), LIMITS));
        drop(image);
        match loaded {
            Ok(p) => {
                info!("wasm: payload started ({} B image)", size);
                // Payloads subscribe in `init`: announce the app keys so BLE
                // can resolve their short ids.
                for key in p.subscriptions().filter(|k| badge_mw::keys::app_may_use(k)) {
                    let mut text = heapless::String::new();
                    if text.push_str(key).is_ok() {
                        let _ = mw::APP_SUBSCRIBED.try_send(text);
                    }
                }
                self.payload = Some(p);
                self.update(Lifecycle::started);
            }
            Err(e) => {
                warn!("wasm: payload did not load: {}", defmt::Display2Format(&e));
                self.update(|l| l.load_failed(e));
            }
        }
        info!("wasm: heap used {} free {}", esp_alloc::HEAP.used(), esp_alloc::HEAP.free());
        Ok(())
    }

    /// Runs one storage operation of an upload; a failure empties the slot.
    async fn store(&mut self, op: PayloadOp) -> Option<PayloadDone> {
        match storage::payload(op).await {
            Ok(done) => Some(done),
            Err(e) => {
                self.storage_failed(e);
                None
            }
        }
    }

    /// Carries out one command and builds its answer.
    async fn execute(&mut self, req: Request) -> Reply {
        let Request { seq, cmd } = req;
        let outcome = match cmd {
            WasmCmd::Erase { len } => {
                self.payload = None;
                if self.store(PayloadOp::Erase { len }).await.is_some() {
                    self.update(Lifecycle::erased);
                }
                Ok(())
            }
            WasmCmd::Write { offset, data } => {
                if self.life.loading() {
                    let end = u32::try_from(data.len()).ok().and_then(|n| offset.checked_add(n));
                    if self.store(PayloadOp::Write { offset, data }).await.is_some() {
                        self.update(|l| l.written(end.unwrap_or(u32::MAX)));
                    }
                    Ok(())
                } else {
                    Err(ErrorCode::BadArgument)
                }
            }
            WasmCmd::Activate { len, crc } => {
                if self.life.loading() {
                    if let Some(PayloadDone::Installed(header)) = self.store(PayloadOp::Activate { len, crc }).await {
                        info!("wasm: payload installed ({} B)", header.len);
                        self.update(|l| l.installed(header.len));
                    }
                    Ok(())
                } else {
                    Err(ErrorCode::BadArgument)
                }
            }
            WasmCmd::Start => self.start().await,
            WasmCmd::Stop => {
                self.payload = None;
                self.update(Lifecycle::stopped);
                Ok(())
            }
            WasmCmd::Status => Ok(()),
        };
        (seq, outcome.map(|()| self.status()))
    }

    /// Calls `tick` if one is due now.
    fn tick(&mut self) {
        let Some(p) = self.payload.as_mut() else {
            return;
        };
        let now = now_ms();
        if p.next_tick().is_none_or(|due| tick_delay(now, due) > 0) {
            return;
        }
        match timed(|| p.tick(now)) {
            Ok(()) => self.update(Lifecycle::ticked),
            Err(trap) => self.trapped(trap),
        }
    }

    /// Delivers one sample to a subscribed running payload.
    fn deliver(&mut self, key: Key<'_>, data: &[u8]) {
        let Some(p) = self.payload.as_mut() else {
            return;
        };
        if let Err(trap) = timed(|| p.deliver(key, data)) {
            self.trapped(trap);
        }
    }

    /// Milliseconds until the next tick is due (at most [`IDLE_MS`]).
    fn sleep_ms(&self) -> u32 {
        self.payload
            .as_ref()
            .and_then(Payload::next_tick)
            .map_or(IDLE_MS, |due| tick_delay(now_ms(), due).min(IDLE_MS))
    }

    /// Boot: find the installed image and start it unless in safe mode.
    async fn boot(&mut self, boot_held: bool) {
        let installed = match storage::payload(PayloadOp::Installed).await {
            Ok(PayloadDone::Installed(header)) => Some(header.len),
            Ok(_) | Err(PayloadError::Store(esp_base::store::StoreError::NoImage)) => None,
            Err(e) => {
                self.storage_failed(e);
                None
            }
        };
        if let Some(len) = installed {
            self.life = Lifecycle::boot(Some(len));
        }
        match payload::boot_action(installed.is_some(), boot_held) {
            BootAction::Idle => info!("wasm: no payload installed"),
            BootAction::SafeMode => warn!("wasm: BOOT held at reset: safe mode, payload not started"),
            BootAction::Autostart => {
                info!("wasm: autostarting the installed payload");
                if let Err(code) = self.start().await {
                    warn!("wasm: autostart refused ({})", code.0);
                    self.update(|l| l.load_failed("autostart: not enough heap"));
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn wasm_task(boot_held: bool) {
    let mut runner = Runner { payload: None, life: Lifecycle::boot(None) };
    runner.boot(boot_held).await;
    let mut next_report = REPORT_MS;
    let mut reported = (0u32, 0u32);
    loop {
        let sleep = Timer::after_millis(u64::from(runner.sleep_ms()));
        match select4(CMDS.receive(), sleep, mw::APP_SAMPLES.receive(), BUTTONS.receive()).await {
            Either4::First(req) => {
                let reply = runner.execute(req).await;
                REPLIES.send(reply).await;
            }
            Either4::Second(()) => runner.tick(),
            Either4::Third(sample) => {
                if let Ok(key) = Key::new(sample.key.as_str()) {
                    runner.deliver(key, &sample.payload);
                }
            }
            Either4::Fourth((button, pressed)) => {
                let (key, data) = keys::button_sample(button, pressed);
                runner.deliver(key, &data);
            }
        }
        let now = Instant::now().as_millis();
        if now >= next_report {
            next_report = now.saturating_add(REPORT_MS);
            let current = (DROPPED.load(Ordering::Relaxed), MAX_CALL_US.load(Ordering::Relaxed));
            if current != reported {
                warn!("wasm: {} inputs dropped (queues full); longest payload call {} us", current.0, current.1);
                reported = current;
            }
        }
    }
}
