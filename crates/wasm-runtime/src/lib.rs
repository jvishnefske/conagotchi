//! The wasm app runtime, shared by the badge firmware and `badgectl sim`, so
//! an app behaves identically on both. Implements the [`wasm_abi`] contract
//! on the [wasmi] interpreter.
//!
//! # Memory
//!
//! The runtime needs a heap (wasmi compiles the module and owns the app's
//! linear memory); it is `no_std` + `alloc`. Every allocation is bounded: the
//! upload buffer by [`wasm_abi::MAX_MODULE`] (freed once compiled), the
//! compiled code by [`wasm_abi::MAX_CODE_BYTES`] and
//! [`wasm_abi::MAX_DATA_BYTES`] (checked before compiling), linear memory by
//! [`wasm_abi::MAX_MEMORY_BYTES`] (wasmi `StoreLimits`), the value stack by
//! [`STACK_BYTES`], parse sizes by wasmi's strict limits. The compiled form
//! is several times the wasm code size; `tests/fr36_heap.rs` measures the
//! peak for the demo app and for a module at the limits. Each upload gets a
//! fresh wasmi `Engine`, so replacing a module frees everything the old one
//! compiled.
//!
//! # Lifecycle
//!
//! The state machine reported in [`WasmStatus::state`]:
//!
//! - `EMPTY`: nothing loaded (also after a module failed validation; the
//!   reason is in `error`).
//! - `LOADING`: [`Runtime::load_chunk`] with offset 0 started an upload
//!   (dropping any module or app); chunks must follow in order, with the
//!   same `total` (at most `MAX_MODULE`). The last chunk compiles and
//!   validates the module ([`validate`]) and test-instantiates it, which
//!   checks the memory limit.
//! - `LOADED`: a valid module; [`Runtime::start`] instantiates it, samples
//!   the host time and runs `init`.
//! - `RUNNING`: [`Runtime::tick`] runs `tick(now_ms)`. Starting again
//!   restarts from a fresh instance; [`Runtime::stop`] drops the instance.
//! - `TRAPPED`: `init` / `tick` trapped or ran out of fuel; the instance is
//!   dropped, `error` says why, and `start` may try again.
//!
//! Each `init` / `tick` call gets [`wasm_abi::FUEL_PER_CALL`] fuel; the fuel
//! consumed since start and the tick count are reported in the status.
//!
//! # Host calls
//!
//! The embedder supplies a [`Host`] for each call. Imports queue their
//! effects, which the runtime hands to the host when the call returns (the
//! private `imports` module); host failures at that point are not visible to
//! the app (they are counted in [`Runtime::host_failures`]).
//!
//! # Samples
//!
//! The embedder feeds received samples with [`Runtime::deliver`]. A sample is
//! queued only if the app subscribed to its key; the queue holds
//! [`wasm_abi::RECV_QUEUE`] samples and drops the oldest. A sample identical
//! (key and payload) to one delivered less than [`DEDUP_MS`] earlier is
//! dropped, because the badge hears app samples over both ESP-NOW and BLE.
//! "Earlier" is measured in host time as of the last `start` / `tick`, so
//! the window has the tick's granularity, and at most [`DEDUP_CAP`] recent
//! samples are remembered.

#![no_std]
#![forbid(unsafe_code)]

extern crate alloc;

mod imports;
pub mod validate;

use alloc::boxed::Box;
use alloc::vec::Vec;

use badge_proto::{WasmState, WasmStatus};
use heapless::Deque;
pub use wasm_abi::Status;
use wasm_abi::{
    EXPORT_INIT, EXPORT_MEMORY, EXPORT_TICK, FUEL_PER_CALL, MAX_MEMORY_BYTES, MAX_MODULE,
};
use wasmi::{Engine, Instance, Linker, Module, Store, StoreLimitsBuilder, TrapCode, TypedFunc};

use crate::imports::{Sample, State};
pub use crate::validate::{MAX_CALL_DEPTH, STACK_BYTES};

/// Capacity of [`WasmStatus::error`].
pub const ERROR_LEN: usize = 48;
/// Status error text.
pub type ErrorText = heapless::String<ERROR_LEN>;
/// Window in which an identical sample is treated as a duplicate.
pub const DEDUP_MS: i64 = 500;
/// Recent samples remembered for duplicate detection.
pub const DEDUP_CAP: usize = 8;

/// What the app can do to the world. Implemented by the firmware shell and
/// by the simulator.
pub trait Host {
    /// Monotonic milliseconds (sampled once per `start`; `tick` gets its
    /// time as an argument).
    fn now_ms(&mut self) -> i64;
    /// One log line from the app (already length-limited, valid UTF-8).
    fn log(&mut self, line: &str);
    /// Publish on an app key (already validated with
    /// [`wasm_abi::app_key_is_valid`], the `badge_mw::keys::app_may_use`
    /// rule).
    fn publish(&mut self, key: &str, payload: &[u8]) -> Result<(), Status>;
    /// Set the 8 LEDs (R G B each).
    fn set_leds(&mut self, rgb: &[u8; 3 * wasm_abi::LED_COUNT]) -> Result<(), Status>;
    /// Show text on the HOST screen.
    fn show_text(&mut self, text: &str) -> Result<(), Status>;
    /// Select a pet mode (`PetMode` value, already range-checked).
    fn set_pet(&mut self, mode: i32) -> Result<(), Status>;
}

/// Why a runtime command failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeError {
    /// Upload chunk out of order, outside the module, or module too large.
    BadChunk,
    /// No complete module is loaded.
    NotLoaded,
    /// The module failed validation, lacks a required export, or could not
    /// be instantiated (see [`WasmStatus::error`]).
    Invalid,
    /// The app is not running.
    NotRunning,
    /// The app trapped or ran out of fuel (it is now stopped).
    Trapped,
}

/// A validated module and the engine that compiled it.
struct Loaded {
    engine: Engine,
    module: Module,
    size: u32,
}

/// A running instance.
struct App {
    store: Store<State>,
    init: TypedFunc<(), ()>,
    tick: TypedFunc<i64, ()>,
}

/// The upload state machine (see the crate docs).
enum Phase {
    Empty,
    Loading {
        total: u32,
        bytes: Vec<u8>,
    },
    Loaded(Loaded),
    /// Boxed: the store data (queues, outbox) is ~1.7 KiB.
    Running(Loaded, Box<App>),
    Trapped(Loaded),
}

/// A sample remembered for duplicate detection.
struct Recent {
    sample: Sample,
    at_ms: i64,
}

/// The runtime: upload buffer, instance, subscriptions and receive queue.
pub struct Runtime {
    phase: Phase,
    error: ErrorText,
    fuel_used: u64,
    ticks: u32,
    /// Host time of the last `start` / `tick`.
    now_ms: i64,
    recent: Deque<Recent, DEDUP_CAP>,
    host_failures: u32,
}

impl Default for Runtime {
    fn default() -> Runtime {
        Runtime::new()
    }
}

impl Runtime {
    /// An empty runtime.
    pub fn new() -> Runtime {
        Runtime {
            phase: Phase::Empty,
            error: ErrorText::new(),
            fuel_used: 0,
            ticks: 0,
            now_ms: 0,
            recent: Deque::new(),
            host_failures: 0,
        }
    }

    /// Takes one upload chunk; `offset` 0 starts a new upload (stopping any
    /// running app). The chunk that completes the module compiles and
    /// validates it: [`RuntimeError::Invalid`] leaves the runtime `EMPTY`
    /// with the reason in the status.
    pub fn load_chunk(&mut self, offset: u32, total: u32, data: &[u8]) -> Result<(), RuntimeError> {
        if total == 0 || usize::try_from(total).map_or(true, |t| t > MAX_MODULE) {
            return Err(RuntimeError::BadChunk);
        }
        if offset == 0 {
            // Drop the old module (and its engine) before allocating anew.
            self.phase = Phase::Empty;
            self.error.clear();
            let mut bytes = Vec::new();
            bytes
                .try_reserve_exact(usize::try_from(total).map_err(|_| RuntimeError::BadChunk)?)
                .map_err(|_| RuntimeError::BadChunk)?;
            self.phase = Phase::Loading { total, bytes };
        }
        let Phase::Loading {
            total: expected,
            bytes,
        } = &mut self.phase
        else {
            return Err(RuntimeError::BadChunk);
        };
        let received = u32::try_from(bytes.len()).map_err(|_| RuntimeError::BadChunk)?;
        let len = u32::try_from(data.len()).map_err(|_| RuntimeError::BadChunk)?;
        let end = offset.checked_add(len).ok_or(RuntimeError::BadChunk)?;
        if total != *expected || offset != received || end > total {
            return Err(RuntimeError::BadChunk);
        }
        bytes.extend_from_slice(data);
        if end < total {
            return Ok(());
        }
        let Phase::Loading { bytes, .. } = core::mem::replace(&mut self.phase, Phase::Empty) else {
            return Err(RuntimeError::BadChunk);
        };
        match compile(bytes, total) {
            Ok(loaded) => {
                self.phase = Phase::Loaded(loaded);
                Ok(())
            }
            Err(text) => {
                self.error = text;
                Err(RuntimeError::Invalid)
            }
        }
    }

    /// Instantiates the loaded module and runs `init` (restarting if the app
    /// is already running).
    pub fn start<H: Host>(&mut self, host: &mut H) -> Result<(), RuntimeError> {
        let loaded = match core::mem::replace(&mut self.phase, Phase::Empty) {
            Phase::Loaded(l) | Phase::Trapped(l) | Phase::Running(l, _) => l,
            other => {
                self.phase = other;
                return Err(RuntimeError::NotLoaded);
            }
        };
        self.error.clear();
        self.fuel_used = 0;
        self.ticks = 0;
        self.recent.clear();
        self.now_ms = host.now_ms();
        let mut app = match instantiate(&loaded) {
            Ok(app) => Box::new(app),
            Err(text) => {
                self.error = text;
                self.phase = Phase::Loaded(loaded);
                return Err(RuntimeError::Invalid);
            }
        };
        let init = app.init;
        let result = self.call(&mut app, host, |store| init.call(store, ()));
        self.settle(loaded, app, result)
    }

    /// Stops the app; the module stays loaded (a trapped app becomes
    /// `LOADED` again).
    pub fn stop(&mut self) {
        self.phase = match core::mem::replace(&mut self.phase, Phase::Empty) {
            Phase::Running(l, _) | Phase::Trapped(l) => Phase::Loaded(l),
            other => other,
        };
    }

    /// Runs one `tick(now_ms)` if the app is running.
    pub fn tick<H: Host>(&mut self, host: &mut H, now_ms: i64) -> Result<(), RuntimeError> {
        let (loaded, mut app) = match core::mem::replace(&mut self.phase, Phase::Empty) {
            Phase::Running(l, a) => (l, a),
            other => {
                self.phase = other;
                return Err(RuntimeError::NotRunning);
            }
        };
        self.now_ms = now_ms;
        self.ticks = self.ticks.saturating_add(1);
        let tick = app.tick;
        let result = self.call(&mut app, host, |store| tick.call(store, now_ms));
        self.settle(loaded, app, result)
    }

    /// Queues a received sample if the app subscribed to `key`; returns true
    /// if it was queued (false if not running, not subscribed, malformed, or
    /// a duplicate within [`DEDUP_MS`]).
    pub fn deliver(&mut self, key: &str, payload: &[u8]) -> bool {
        let Phase::Running(_, app) = &mut self.phase else {
            return false;
        };
        let state = app.store.data_mut();
        if !state.is_subscribed(key) {
            return false;
        }
        let Some(sample) = Sample::new(key.as_bytes(), payload) else {
            return false;
        };
        let now = self.now_ms;
        while self
            .recent
            .front()
            .is_some_and(|r| now.saturating_sub(r.at_ms) >= DEDUP_MS)
        {
            self.recent.pop_front();
        }
        if self.recent.iter().any(|r| r.sample == sample) {
            return false;
        }
        if self.recent.is_full() {
            self.recent.pop_front();
        }
        // Cannot fail: a slot was just freed if needed.
        let _ = self.recent.push_back(Recent {
            sample: sample.clone(),
            at_ms: now,
        });
        state.enqueue(sample);
        true
    }

    /// True if an app is running (`tick` will call it).
    pub fn is_running(&self) -> bool {
        matches!(self.phase, Phase::Running(..))
    }

    /// True if the app subscribed to `key`.
    pub fn is_subscribed(&self, key: &str) -> bool {
        self.state().is_some_and(|s| s.is_subscribed(key))
    }

    /// The keys the running app subscribed to (empty when not running).
    pub fn subscriptions(&self) -> impl Iterator<Item = &str> + '_ {
        self.state()
            .into_iter()
            .flat_map(|s| s.subscriptions.iter().map(|k| k.as_str()))
    }

    /// Host calls (publish, LEDs, text, pet) the host rejected since this
    /// runtime was created.
    pub fn host_failures(&self) -> u32 {
        self.host_failures
    }

    /// Status for `GetWasm` / `WasmStatus`.
    pub fn status(&self) -> WasmStatus {
        let (state, size) = match &self.phase {
            Phase::Empty => (WasmState::Empty, 0),
            Phase::Loading { bytes, .. } => (
                WasmState::Loading,
                u32::try_from(bytes.len()).unwrap_or(u32::MAX),
            ),
            Phase::Loaded(l) => (WasmState::Loaded, l.size),
            Phase::Running(l, _) => (WasmState::Running, l.size),
            Phase::Trapped(l) => (WasmState::Trapped, l.size),
        };
        WasmStatus {
            state,
            size,
            fuel_used: self.fuel_used,
            ticks: self.ticks,
            error: self.error.clone(),
        }
    }

    fn state(&self) -> Option<&State> {
        match &self.phase {
            Phase::Running(_, app) => Some(app.store.data()),
            _ => None,
        }
    }

    /// Runs one guest call with fresh fuel, then flushes its effects to the
    /// host (also after a trap).
    fn call<H: Host>(
        &mut self,
        app: &mut App,
        host: &mut H,
        f: impl FnOnce(&mut Store<State>) -> Result<(), wasmi::Error>,
    ) -> Result<(), ErrorText> {
        app.store.data_mut().now_ms = self.now_ms;
        let result = match app.store.set_fuel(FUEL_PER_CALL) {
            Ok(()) => f(&mut app.store).map_err(|e| trap_text(&e)),
            Err(e) => Err(validate::text(format_args!("{e}"))),
        };
        let left = app.store.get_fuel().unwrap_or(0);
        self.fuel_used = self
            .fuel_used
            .saturating_add(FUEL_PER_CALL.saturating_sub(left));
        let failures = app.store.data_mut().outbox.flush(host);
        self.host_failures = self.host_failures.saturating_add(failures);
        result
    }

    /// Stores the outcome of a guest call as the next phase.
    fn settle(
        &mut self,
        loaded: Loaded,
        app: Box<App>,
        result: Result<(), ErrorText>,
    ) -> Result<(), RuntimeError> {
        match result {
            Ok(()) => {
                self.phase = Phase::Running(loaded, app);
                Ok(())
            }
            Err(text) => {
                self.error = text;
                self.phase = Phase::Trapped(loaded);
                Err(RuntimeError::Trapped)
            }
        }
    }
}

/// Compiles and validates a complete module in a fresh engine, then
/// instantiates it once (and drops the instance) so memory and table limits
/// are checked at upload rather than at start.
fn compile(bytes: Vec<u8>, size: u32) -> Result<Loaded, ErrorText> {
    validate::section_sizes(&bytes)?;
    let engine = Engine::new(&validate::config());
    let module = Module::new(&engine, &bytes).map_err(|e| validate::text(format_args!("{e}")))?;
    // The module keeps its own copy of what it needs: free the upload now.
    drop(bytes);
    validate::check(&module)?;
    let loaded = Loaded {
        engine,
        module,
        size,
    };
    instantiate(&loaded)?;
    Ok(loaded)
}

/// Instantiates a validated module with the `badge` imports and limits.
fn instantiate(loaded: &Loaded) -> Result<App, ErrorText> {
    let err = |e: wasmi::Error| validate::text(format_args!("{e}"));
    let memory_bytes =
        usize::try_from(MAX_MEMORY_BYTES).map_err(|_| validate::text(format_args!("limit")))?;
    let limits = StoreLimitsBuilder::new()
        .memory_size(memory_bytes)
        .memories(1)
        .tables(1)
        .table_elements(256)
        .instances(1)
        .build();
    let mut store = Store::new(&loaded.engine, State::new(limits));
    store.limiter(|state| &mut state.limits);
    let mut linker = Linker::new(&loaded.engine);
    imports::define(&mut linker).map_err(err)?;
    let instance: Instance = linker
        .instantiate_and_start(&mut store, &loaded.module)
        .map_err(err)?;
    let memory = instance
        .get_memory(&store, EXPORT_MEMORY)
        .ok_or_else(|| validate::text(format_args!("missing export {EXPORT_MEMORY}")))?;
    store.data_mut().memory = Some(memory);
    let init = instance
        .get_typed_func::<(), ()>(&store, EXPORT_INIT)
        .map_err(err)?;
    let tick = instance
        .get_typed_func::<i64, ()>(&store, EXPORT_TICK)
        .map_err(err)?;
    Ok(App { store, init, tick })
}

/// The status text for a failed call.
fn trap_text(error: &wasmi::Error) -> ErrorText {
    match error.as_trap_code() {
        Some(TrapCode::OutOfFuel) => validate::text(format_args!("out of fuel")),
        Some(code) => validate::text(format_args!("trap: {code}")),
        None => validate::text(format_args!("{error}")),
    }
}
