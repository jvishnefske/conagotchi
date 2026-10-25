//! The wasm app task: owns the [`Runtime`] (the same `wasm-runtime` crate
//! `badgectl sim` embeds) and runs the uploaded app next to the core loop.
//!
//! # Commands and answers
//!
//! The shell hands every `Effect::Wasm { seq, cmd }` to [`submit`] (a
//! two-slot queue; when it is full the shell answers `BUSY` itself). The task
//! executes one command at a time and answers it with exactly one
//! `Input::Wasm { seq, result }` on a separate reply queue, sent with an
//! awaiting `send` so an answer is never dropped. `result` is the runtime
//! status (`Ok`) for every command that reached the runtime, including a
//! module that failed validation (`EMPTY` plus `error`) or an app that
//! trapped (`TRAPPED` plus `error`); it is `BAD_ARGUMENT` for an
//! out-of-order or oversized chunk, or a start/stop with nothing loaded, and
//! `BUSY` when the heap is too low to compile or start safely.
//!
//! # Ticks and samples
//!
//! Every [`TICK_MS`] the task drains up to [`SAMPLES_PER_TICK`] samples
//! from [`mw::APP_SAMPLES`] into [`Runtime::deliver`] (which drops
//! duplicates heard over both ESP-NOW and BLE) and, if an app runs, calls
//! its `tick` with the embassy time in milliseconds. The badge's network
//! time lives in the core's sync state on the shell task and is not shared,
//! so apps see local monotonic time.
//!
//! # Host calls
//!
//! The [`Host`] implementation never blocks:
//!
//! - `log` → defmt and an `Input::WasmLog` (a `WasmLog` event on RTT);
//! - `publish` → [`mw::publish_app`] (ESP-NOW and BLE queues);
//! - `set_leds` / `show_text` / `set_pet` → `Input::Host` messages with seq
//!   [`WASM_SEQ`] (`SetLeds`, `DisplayText`, `SetPetMode`), exactly as if the
//!   host had sent them; an unchanged LED frame is not sent again, which
//!   keeps the RTT acks down.
//!
//! These inputs go through a bounded event queue that the shell drains with
//! [`try_next`]; a full queue drops the input and counts it, and the count
//! is logged every [`REPORT_MS`] while it grows.
//!
//! # Heap
//!
//! wasmi allocates from the shared esp-alloc heap; `main` adds a
//! [`HEAP_BYTES`] region for it (esp-alloc cannot reserve a region for one
//! user, so the wasm runtime and esp-radio share all regions). The ABI
//! limits bound what one app can take (about 48 KiB worst case on the host,
//! 38 KiB for the demo app; see `crates/wasm-runtime/tests/fr36_heap.rs`).
//! Because an allocation failure inside wasmi would halt the badge, the task
//! refuses (`BUSY`) to compile or instantiate while less than
//! [`HEAP_NEEDED`] is free.
//!
//! The badge boots with no app, and an uploaded module lives in RAM only (it
//! is gone after a reset).

use core::sync::atomic::{AtomicU32, Ordering};

use badge_core::io::{Input, WASM_LOG_LEN, WasmCmd};
use badge_core::probe_layout::WASM_SEQ;
use badge_proto::{
    DisplayText, ErrorCode, HostMsg, HostMsg_::Cmd, PetMode, Rgb, SetLeds, SetPetMode, WasmStatus,
};
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, TrySendError};
use embassy_time::{Duration, Instant, Ticker};
use heapless::{String, Vec};
use wasm_abi::LED_COUNT;
use wasm_runtime::{Host, Runtime, RuntimeError, Status};

use crate::mw::{self, MwMessage};

/// Period of the app's `tick`.
pub const TICK_MS: u64 = 20;
/// Received samples handed to the runtime per tick.
pub const SAMPLES_PER_TICK: usize = 8;
/// Size of the heap region `main` adds for the wasm runtime.
pub const HEAP_BYTES: usize = 48 * 1024;
/// Free heap required before compiling a module or starting an app.
pub const HEAP_NEEDED: usize = 32 * 1024;
/// Interval between drop reports.
pub const REPORT_MS: u64 = 5_000;

/// One request from the shell.
struct Request {
    seq: u32,
    cmd: WasmCmd,
}

/// The answer to one request: its seq and `Input::Wasm::result`.
type Reply = (u32, Result<WasmStatus, ErrorCode>);

/// An app output for the core, kept smaller than an [`Input`] so the event
/// queue stays small; [`try_next`] turns it into the input.
#[allow(clippy::large_enum_variant, reason = "fixed-size queue slots; boxing would allocate")]
enum AppEvent {
    /// `log` line (`Input::WasmLog`).
    Log(String<WASM_LOG_LEN>),
    /// `set_leds` / `show_text` / `set_pet` (`Input::Host`, seq [`WASM_SEQ`]).
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
/// Inputs dropped because [`EVENTS`] was full.
static DROPPED: AtomicU32 = AtomicU32::new(0);

/// Spawns the wasm task.
pub fn start(spawner: &Spawner) -> Result<(), WasmError> {
    spawner.spawn(wasm_task().map_err(|_| WasmError::Spawn)?);
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

/// The next input from the wasm task for the core: answers first, then app
/// logs and host messages.
pub fn try_next() -> Option<Input> {
    if let Ok((seq, result)) = REPLIES.try_receive() {
        return Some(Input::Wasm { seq, result });
    }
    EVENTS.try_receive().ok().map(|event| match event {
        AppEvent::Log(text) => Input::WasmLog(text),
        AppEvent::Host(cmd) => Input::Host(HostMsg { seq: WASM_SEQ, cmd: Some(cmd) }),
    })
}

/// Offers an app output to the shell without waiting; counts it if dropped.
fn emit(event: AppEvent) {
    if EVENTS.try_send(event).is_err() {
        DROPPED.fetch_add(1, Ordering::Relaxed);
    }
}

/// Embassy time in milliseconds.
fn now_ms() -> i64 {
    i64::try_from(Instant::now().as_millis()).unwrap_or(i64::MAX)
}

/// The badge side of the app's host calls.
#[derive(Default)]
struct BadgeHost {
    /// Last LED frame sent to the core.
    last_leds: Option<[u8; 3 * LED_COUNT]>,
}

impl BadgeHost {
    fn host_msg(cmd: Cmd) {
        emit(AppEvent::Host(cmd));
    }
}

impl Host for BadgeHost {
    fn now_ms(&mut self) -> i64 {
        now_ms()
    }

    fn log(&mut self, line: &str) {
        info!("wasm: {=str}", line);
        let mut text = String::<WASM_LOG_LEN>::new();
        for c in line.chars() {
            if text.push(c).is_err() {
                break;
            }
        }
        emit(AppEvent::Log(text));
    }

    fn publish(&mut self, key: &str, payload: &[u8]) -> Result<(), Status> {
        if !badge_mw::keys::app_may_use(key) {
            return Err(Status::BadKey);
        }
        let mut msg = MwMessage { key: String::new(), payload: Vec::new() };
        msg.key.push_str(key).map_err(|_| Status::BadKey)?;
        msg.payload.extend_from_slice(payload).map_err(|_| Status::TooLong)?;
        if mw::publish_app(&msg) == 0 { Err(Status::Full) } else { Ok(()) }
    }

    fn set_leds(&mut self, rgb: &[u8; 3 * LED_COUNT]) -> Result<(), Status> {
        if self.last_leds.as_ref() == Some(rgb) {
            return Ok(());
        }
        self.last_leds = Some(*rgb);
        let mut colors = Vec::new();
        for px in rgb.chunks_exact(3) {
            if let &[r, g, b] = px {
                let rgb = (u32::from(r) << 16) | (u32::from(g) << 8) | u32::from(b);
                colors.push(Rgb { rgb }).map_err(|_| Status::TooLong)?;
            }
        }
        Self::host_msg(Cmd::SetLeds(SetLeds { colors }));
        Ok(())
    }

    fn show_text(&mut self, text: &str) -> Result<(), Status> {
        let mut owned = String::new();
        owned.push_str(text).map_err(|_| Status::TooLong)?;
        Self::host_msg(Cmd::DisplayText(DisplayText { text: owned, fg: None, bg: None }));
        Ok(())
    }

    fn set_pet(&mut self, mode: i32) -> Result<(), Status> {
        Self::host_msg(Cmd::SetPetMode(SetPetMode { mode: PetMode(mode) }));
        Ok(())
    }
}

/// True if the heap has room to compile or instantiate; logs if not.
fn heap_ok() -> bool {
    let free = esp_alloc::HEAP.free();
    if free < HEAP_NEEDED {
        warn!("wasm: only {} B heap free, need {}", free, HEAP_NEEDED);
    }
    free >= HEAP_NEEDED
}

/// Carries out one command and builds its answer.
fn execute(rt: &mut Runtime, host: &mut BadgeHost, req: Request) -> Reply {
    let Request { seq, cmd } = req;
    // Heap use is worth logging only when it changes much.
    let mut log_heap = true;
    let outcome = match cmd {
        WasmCmd::Load { offset, total, data } => {
            let last = u32::try_from(data.len()).ok().and_then(|n| offset.checked_add(n)) == Some(total);
            log_heap = last;
            if last && !heap_ok() {
                return (seq, Err(ErrorCode::Busy));
            }
            rt.load_chunk(offset, total, &data)
        }
        WasmCmd::Start => {
            // Free the old instance before checking the heap.
            rt.stop();
            host.last_leds = None;
            if !heap_ok() {
                return (seq, Err(ErrorCode::Busy));
            }
            rt.start(host)
        }
        WasmCmd::Stop => {
            rt.stop();
            Ok(())
        }
        WasmCmd::Status => {
            log_heap = false;
            Ok(())
        }
    };
    let status = rt.status();
    if let Err(e) = outcome {
        warn!("wasm: {} ({=str})", defmt::Debug2Format(&e), status.error.as_str());
    }
    let result = match outcome {
        Ok(()) | Err(RuntimeError::Invalid | RuntimeError::Trapped) => Ok(status),
        Err(RuntimeError::BadChunk | RuntimeError::NotLoaded | RuntimeError::NotRunning) => {
            Err(ErrorCode::BadArgument)
        }
    };
    if log_heap {
        info!("wasm: heap used {} free {}", esp_alloc::HEAP.used(), esp_alloc::HEAP.free());
    }
    (seq, result)
}

/// One app tick: deliver received samples, then run `tick`.
fn tick(rt: &mut Runtime, host: &mut BadgeHost) {
    for _ in 0..SAMPLES_PER_TICK {
        let Ok(sample) = mw::APP_SAMPLES.try_receive() else {
            break;
        };
        rt.deliver(&sample.key, &sample.payload);
    }
    if !rt.is_running() {
        return;
    }
    if rt.tick(host, now_ms()) == Err(RuntimeError::Trapped) {
        warn!("wasm: app stopped: {=str}", rt.status().error.as_str());
    }
}

#[embassy_executor::task]
async fn wasm_task() {
    let mut rt = Runtime::new();
    let mut host = BadgeHost::default();
    let mut ticker = Ticker::every(Duration::from_millis(TICK_MS));
    let mut next_report = REPORT_MS;
    let mut reported = 0u32;
    info!("wasm task running (no app loaded)");
    loop {
        match select(CMDS.receive(), ticker.next()).await {
            Either::First(req) => {
                let reply = execute(&mut rt, &mut host, req);
                REPLIES.send(reply).await;
            }
            Either::Second(()) => tick(&mut rt, &mut host),
        }
        let now = Instant::now().as_millis();
        if now >= next_report {
            next_report = now.saturating_add(REPORT_MS);
            let dropped = DROPPED.load(Ordering::Relaxed);
            if dropped != reported {
                warn!("wasm: {} inputs dropped (event queue full)", dropped);
                reported = dropped;
            }
        }
    }
}
