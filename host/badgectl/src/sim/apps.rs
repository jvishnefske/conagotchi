//! Wasm payloads inside simulated badges, run by middleware's `wasm-host`
//! ("mw" ABI), exactly as the firmware runs them.
//!
//! Each simulated badge owns one [`AppSlot`]:
//!
//! - a [`PayloadStore`], the simulated payload partition. It follows the
//!   core's `WasmCmd` upload effects (`Erase` / `Write` / `Activate`, the
//!   last one re-checking length and CRC-32) and survives a simulated reboot
//!   ([`AppSlot::power_cycle`]), like the flash partition;
//! - the running `wasm_host::Payload<SimBadgeDevice>`, loaded from the
//!   store by `Start` (and on boot, if an image is installed:
//!   [`AppSlot::autostart`]) with [`limits`] (the firmware's limits).
//!
//! [`SimBadgeDevice`] is the payload's view of the badge. It records what the
//! payload did during a call as [`AppAction`]s and the world carries them out
//! afterwards (log lines become `WasmLog` inputs, `app/...` publishes go to
//! the bus, `badge/leds` / `badge/text` / `badge/pet` become host messages
//! with seq `probe_layout::WASM_SEQ`), so the payload never touches the
//! world directly. Publishes are routed by
//! `badge_mw::keys::route_payload_publish`, the same function the firmware
//! uses. `now_ms` is the badge's virtual uptime; GPIO is `Unsupported`.
//!
//! [`AppSlot::command`] executes one `WasmCmd` and returns the answer for
//! `Input::Wasm`: a store step that does not fit the upload in progress, a
//! CRC mismatch and `Start` with nothing installed are `BAD_ARGUMENT`; a
//! module the runtime rejects, or a trap, is `Ok` with state `TRAPPED` and
//! the reason in `error`.

use badge_core::board::{PAYLOAD_FUEL_PER_CALL, PAYLOAD_MAX_IMAGE, PAYLOAD_MAX_MEMORY};
use badge_core::probe_layout::WASM_SEQ;
use badge_mw::keys::{PayloadRoute, RouteError, route_payload_publish};
use badge_proto::{ErrorCode, HostMsg, WasmState, WasmStatus};
use badge_sim::wire::WasmRecord;
use middleware::Key;
use middleware::crc::Crc32;
use wasm_host::{AbiError, Device, Limits, Payload, Pull};

/// The payload limits of the badge: [`PAYLOAD_MAX_MEMORY`] bytes of linear
/// memory and [`PAYLOAD_FUEL_PER_CALL`] fuel per call.
pub const fn limits() -> Limits {
    Limits::bytes(PAYLOAD_MAX_MEMORY).with_fuel(PAYLOAD_FUEL_PER_CALL)
}

/// Something a payload did during one call.
#[derive(Debug, Clone, PartialEq)]
pub enum AppAction {
    /// A log line.
    Log(String),
    /// A publish on an `app/...` key, for the bus.
    Publish(String, Vec<u8>),
    /// A host message on behalf of the payload (`WASM_SEQ`); boxed, it
    /// dwarfs the other variants.
    Host(Box<HostMsg>),
}

/// The payload's view of a simulated badge: records actions.
#[derive(Debug, Default)]
pub struct SimBadgeDevice {
    now_ms: u32,
    actions: Vec<AppAction>,
}

impl SimBadgeDevice {
    /// A device whose clock reads `now_ms`.
    pub fn new(now_ms: u32) -> Self {
        Self { now_ms, actions: Vec::new() }
    }

    /// Takes the recorded actions.
    pub fn take_actions(&mut self) -> Vec<AppAction> {
        std::mem::take(&mut self.actions)
    }
}

impl Device for SimBadgeDevice {
    fn log(&mut self, line: &str) {
        self.actions.push(AppAction::Log(line.to_string()));
    }

    fn publish(&mut self, key: Key<'_>, data: &[u8]) -> Result<(), AbiError> {
        match route_payload_publish(key, data) {
            Ok(PayloadRoute::Local(cmd)) => {
                self.actions.push(AppAction::Host(Box::new(HostMsg { seq: WASM_SEQ, cmd: Some(cmd) })));
                Ok(())
            }
            Ok(PayloadRoute::Radio) => {
                self.actions.push(AppAction::Publish(key.as_str().to_string(), data.to_vec()));
                Ok(())
            }
            Err(RouteError::UnknownKey) => Err(AbiError::InvalidKey),
            Err(RouteError::BadPayload) => Err(AbiError::InvalidArgument),
        }
    }

    fn now_ms(&mut self) -> u32 {
        self.now_ms
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

/// Why a store step was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum StoreError {
    /// `Erase` for an empty image or one over [`PAYLOAD_MAX_IMAGE`].
    #[error("image length {0} is not in 1..={PAYLOAD_MAX_IMAGE}")]
    Length(u32),
    /// `Write` / `Activate` without an erased store.
    #[error("no upload in progress")]
    NotLoading,
    /// `Write` beyond the erased length.
    #[error("write past the image end")]
    OutOfRange,
    /// `Activate` with a length other than the erased one.
    #[error("activate length {got} differs from the erased {expected}")]
    LengthMismatch {
        /// Erased length.
        expected: u32,
        /// Activated length.
        got: u32,
    },
    /// The stored image does not match the CRC-32.
    #[error("stored image CRC {actual:#010x} differs from {expected:#010x}")]
    Crc {
        /// CRC from `Activate`.
        expected: u32,
        /// CRC of the stored bytes.
        actual: u32,
    },
}

/// What the simulated payload partition holds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Phase {
    /// Nothing installed.
    #[default]
    Empty,
    /// Erased for an image of `len` bytes; `written` is the highest byte
    /// written so far.
    Loading { len: u32, written: u32 },
    /// A complete, CRC-verified image.
    Installed { len: u32 },
}

/// The simulated payload partition (persistent across a simulated reboot).
/// Each step returns the next store; a refused step leaves the old one.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct PayloadStore {
    image: Vec<u8>,
    phase: Phase,
}

impl PayloadStore {
    /// An empty store.
    pub fn new() -> Self {
        Self::default()
    }

    /// Erases the store for an image of `len` bytes.
    pub fn erase(&self, len: u32) -> Result<Self, StoreError> {
        if len == 0 || len > PAYLOAD_MAX_IMAGE {
            return Err(StoreError::Length(len));
        }
        let size = usize::try_from(len).map_err(|_| StoreError::Length(len))?;
        Ok(Self { image: vec![0xFF; size], phase: Phase::Loading { len, written: 0 } })
    }

    /// Writes `data` at `offset` of the image being loaded.
    pub fn write(&self, offset: u32, data: &[u8]) -> Result<Self, StoreError> {
        let Phase::Loading { len, written } = self.phase else { return Err(StoreError::NotLoading) };
        let start = usize::try_from(offset).map_err(|_| StoreError::OutOfRange)?;
        let end = start.checked_add(data.len()).ok_or(StoreError::OutOfRange)?;
        let mut image = self.image.clone();
        image.get_mut(start..end).ok_or(StoreError::OutOfRange)?.copy_from_slice(data);
        let end = u32::try_from(end).map_err(|_| StoreError::OutOfRange)?;
        Ok(Self { image, phase: Phase::Loading { len, written: written.max(end) } })
    }

    /// Records the image as installed after checking its length and CRC-32.
    pub fn activate(&self, len: u32, crc: u32) -> Result<Self, StoreError> {
        let Phase::Loading { len: expected, .. } = self.phase else { return Err(StoreError::NotLoading) };
        if len != expected {
            return Err(StoreError::LengthMismatch { expected, got: len });
        }
        let actual = Crc32::checksum(&self.image);
        if actual != crc {
            return Err(StoreError::Crc { expected: crc, actual });
        }
        Ok(Self { image: self.image.clone(), phase: Phase::Installed { len } })
    }

    /// The installed image, if any.
    pub fn installed(&self) -> Option<&[u8]> {
        matches!(self.phase, Phase::Installed { .. }).then_some(self.image.as_slice())
    }

    /// True while an upload is in progress.
    pub fn loading(&self) -> bool {
        matches!(self.phase, Phase::Loading { .. })
    }

    /// `WasmStatus.size`: the installed length, or the bytes written so far
    /// while loading.
    pub fn size(&self) -> u32 {
        match self.phase {
            Phase::Empty => 0,
            Phase::Loading { written, .. } => written,
            Phase::Installed { len } => len,
        }
    }
}

/// Whether the payload runs.
enum Run {
    /// Not running.
    Stopped,
    /// Loaded and running.
    Running(Box<Payload<SimBadgeDevice>>),
    /// Rejected at load or trapped; the reason.
    Trapped(String),
}

/// One badge's payload store and runtime.
pub struct AppSlot {
    store: PayloadStore,
    run: Run,
    ticks: u32,
}

impl Default for AppSlot {
    fn default() -> Self {
        Self::new()
    }
}

/// `true` once wrapping time `now` has reached `due`.
fn reached(now: u32, due: u32) -> bool {
    now.wrapping_sub(due) < u32::MAX / 2
}

/// `text` cut to fit `WasmStatus.error`, on a character boundary.
fn error_text(text: &str) -> heapless::String<48> {
    let mut out = heapless::String::new();
    for ch in text.chars() {
        if out.push(ch).is_err() {
            break;
        }
    }
    out
}

impl AppSlot {
    /// An empty store, nothing running.
    pub fn new() -> Self {
        Self { store: PayloadStore::new(), run: Run::Stopped, ticks: 0 }
    }

    /// The payload store.
    #[cfg(test)]
    pub fn store(&self) -> &PayloadStore {
        &self.store
    }

    /// Executes one payload command at uptime `now_ms`; returns the answer
    /// and the payload's actions (from `init` on `Start`).
    pub fn command(&mut self, cmd: WasmRecord<'_>, now_ms: u32) -> (Result<WasmStatus, ErrorCode>, Vec<AppAction>) {
        let step = match cmd {
            WasmRecord::Erase { len } => {
                self.stop();
                self.store.erase(len)
            }
            WasmRecord::Write { offset, data } => self.store.write(offset, data),
            WasmRecord::Activate { len, crc } => self.store.activate(len, crc),
            WasmRecord::Start => {
                return match self.start(now_ms) {
                    Some(actions) => (Ok(self.status()), actions),
                    None => (Err(ErrorCode::BadArgument), Vec::new()),
                };
            }
            WasmRecord::Stop => {
                self.stop();
                Ok(self.store.clone())
            }
            WasmRecord::Status => Ok(self.store.clone()),
        };
        match step {
            Ok(store) => {
                self.store = store;
                (Ok(self.status()), Vec::new())
            }
            Err(_) => (Err(ErrorCode::BadArgument), Vec::new()),
        }
    }

    /// Stops the payload (the store keeps it).
    fn stop(&mut self) {
        self.run = Run::Stopped;
    }

    /// (Re)loads the installed image and runs `init`; `None` if nothing is
    /// installed. A rejected module or a trapping `init` leaves the slot
    /// `TRAPPED`.
    fn start(&mut self, now_ms: u32) -> Option<Vec<AppAction>> {
        let image = self.store.installed()?;
        self.ticks = 0;
        let (run, actions) = match Payload::load(image, SimBadgeDevice::new(now_ms), limits()) {
            Ok(mut payload) => {
                let actions = payload.device().take_actions();
                (Run::Running(Box::new(payload)), actions)
            }
            Err(e) => (Run::Trapped(e.to_string()), Vec::new()),
        };
        self.run = run;
        Some(actions)
    }

    /// A simulated reboot: the payload stops, the store is kept.
    #[cfg(test)]
    pub fn power_cycle(&mut self) {
        self.stop();
    }

    /// Boot: starts the installed payload, as the firmware does; returns its
    /// actions (none if nothing is installed).
    pub fn autostart(&mut self, now_ms: u32) -> Vec<AppAction> {
        self.start(now_ms).unwrap_or_default()
    }

    /// Runs the running payload with its device clock at `now_ms`; a trap
    /// leaves the slot `TRAPPED`. Returns the payload's actions.
    fn with_payload<F>(&mut self, now_ms: u32, f: F) -> Vec<AppAction>
    where
        F: FnOnce(&mut Payload<SimBadgeDevice>, &mut u32) -> Result<(), wasm_host::Trap>,
    {
        let Run::Running(payload) = &mut self.run else { return Vec::new() };
        payload.device().now_ms = now_ms;
        let result = f(payload, &mut self.ticks);
        let actions = payload.device().take_actions();
        if let Err(trap) = result {
            self.run = Run::Trapped(trap.to_string());
        }
        actions
    }

    /// Calls the payload's `tick` if one is due at uptime `now_ms`; returns
    /// its actions.
    pub fn tick(&mut self, now_ms: u32) -> Vec<AppAction> {
        self.with_payload(now_ms, |p, ticks| {
            if !p.next_tick().is_some_and(|due| reached(now_ms, due)) {
                return Ok(());
            }
            *ticks = ticks.wrapping_add(1);
            p.tick(now_ms).map(|_| ())
        })
    }

    /// Hands a sample to the payload if it subscribed to exactly `key`;
    /// returns its actions.
    pub fn deliver(&mut self, key: &str, payload: &[u8], now_ms: u32) -> Vec<AppAction> {
        let Ok(key) = Key::new(key) else { return Vec::new() };
        self.with_payload(now_ms, |p, _| p.deliver(key, payload).map(|_| ()))
    }

    /// The payload status, as `GetWasm` reports it.
    pub fn status(&self) -> WasmStatus {
        let (state, error) = match (&self.run, self.store.installed().is_some()) {
            _ if self.store.loading() => (WasmState::Loading, heapless::String::new()),
            (_, false) => (WasmState::Empty, heapless::String::new()),
            (Run::Running(_), true) => (WasmState::Running, heapless::String::new()),
            (Run::Trapped(e), true) => (WasmState::Trapped, error_text(e)),
            (Run::Stopped, true) => (WasmState::Stopped, heapless::String::new()),
        };
        WasmStatus { state, size: self.store.size(), ticks: self.ticks, error, capacity: PAYLOAD_MAX_IMAGE }
    }
}

#[cfg(test)]
mod tests {
    use badge_proto::HostMsg_::Cmd;
    use badge_proto::{PetMode, SetPetMode};

    use super::*;

    /// A payload that subscribes to `app/in`, ticks every 10 ms, and on
    /// every tick or message publishes on `app/out`; 1-byte pages, 4 KiB.
    const ECHOISH: &str = r#"(module
      (import "mw" "log" (func $log (param i32 i32)))
      (import "mw" "publish" (func $publish (param i32 i32 i32 i32) (result i32)))
      (import "mw" "subscribe" (func $subscribe (param i32 i32) (result i32)))
      (import "mw" "set_tick" (func $set_tick (param i32) (result i32)))
      (memory (export "memory") 4096 4096 (pagesize 1))
      (data (i32.const 16) "app/in")
      (data (i32.const 32) "app/out")
      (data (i32.const 48) "up")
      (func (export "init")
        (drop (call $subscribe (i32.const 16) (i32.const 6)))
        (drop (call $set_tick (i32.const 10)))
        (call $log (i32.const 48) (i32.const 2)))
      (func (export "tick") (param i32)
        (drop (call $publish (i32.const 32) (i32.const 7) (i32.const 48) (i32.const 2))))
      (func (export "mw_alloc") (param i32) (result i32) (i32.const 1024))
      (func (export "on_message") (param i32 i32 i32)
        (drop (call $publish (i32.const 32) (i32.const 7) (local.get 1) (local.get 2)))))"#;

    fn install(slot: &mut AppSlot, image: &[u8]) {
        let len = u32::try_from(image.len()).unwrap();
        assert!(slot.command(WasmRecord::Erase { len }, 0).0.is_ok());
        for (i, c) in image.chunks(192).enumerate() {
            let offset = u32::try_from(i * 192).unwrap();
            assert!(slot.command(WasmRecord::Write { offset, data: c }, 0).0.is_ok());
        }
        let st = slot.command(WasmRecord::Activate { len, crc: Crc32::checksum(image) }, 0).0.unwrap();
        assert_eq!((st.state, st.size, st.capacity), (WasmState::Stopped, len, PAYLOAD_MAX_IMAGE));
    }

    #[test]
    fn fr39_device_routes_publishes_like_the_badge() {
        let mut d = SimBadgeDevice::new(5);
        d.log("hi");
        assert_eq!(d.publish(Key::from_static("conagotchi/sync"), b"x"), Err(AbiError::InvalidKey));
        assert_eq!(d.publish(Key::from_static("app/hello"), b"x"), Ok(()));
        assert_eq!(d.publish(Key::from_static("badge/pet"), &[9]), Err(AbiError::InvalidArgument));
        assert_eq!(d.publish(Key::from_static("badge/pet"), &[2]), Ok(()));
        assert_eq!(d.publish(Key::from_static("badge/leds"), &[0; 23]), Err(AbiError::InvalidArgument));
        assert_eq!(d.now_ms(), 5);
        assert_eq!(d.gpio_output(1), Err(AbiError::Unsupported));
        assert_eq!(d.gpio_read(1), Err(AbiError::Unsupported));
        let a = d.take_actions();
        assert_eq!(a[0], AppAction::Log("hi".into()));
        assert_eq!(a[1], AppAction::Publish("app/hello".into(), b"x".to_vec()));
        let AppAction::Host(m) = &a[2] else { panic!("{a:?}") };
        assert_eq!(m.seq, WASM_SEQ);
        assert!(matches!(m.cmd, Some(Cmd::SetPetMode(SetPetMode { mode: PetMode::Walk }))));
        assert_eq!(a.len(), 3);
        assert!(d.take_actions().is_empty());
    }

    #[test]
    fn fr39_payload_store_follows_erase_write_activate() {
        let image = [1u8, 2, 3, 4, 5];
        let crc = Crc32::checksum(&image);
        let s = PayloadStore::new();
        assert_eq!(s.write(0, &image), Err(StoreError::NotLoading));
        assert_eq!(s.erase(0), Err(StoreError::Length(0)));
        assert_eq!(s.erase(PAYLOAD_MAX_IMAGE + 1), Err(StoreError::Length(PAYLOAD_MAX_IMAGE + 1)));
        let s = s.erase(5).unwrap();
        assert!(s.loading());
        assert_eq!(s.write(3, &image[..3]), Err(StoreError::OutOfRange));
        let s = s.write(0, &image[..3]).unwrap();
        assert_eq!(s.size(), 3);
        assert_eq!(s.activate(5, crc), Err(StoreError::Crc { expected: crc, actual: Crc32::checksum(&[1, 2, 3, 0xFF, 0xFF]) }));
        let s = s.write(3, &image[3..]).unwrap();
        assert_eq!(s.activate(4, crc), Err(StoreError::LengthMismatch { expected: 5, got: 4 }));
        let s = s.activate(5, crc).unwrap();
        assert_eq!(s.installed(), Some(&image[..]));
        assert_eq!(s.size(), 5);
        assert_eq!(s.activate(5, crc), Err(StoreError::NotLoading));
    }

    #[test]
    fn fr39_slot_runs_the_installed_payload_and_survives_a_power_cycle() {
        let image = wat::parse_str(ECHOISH).unwrap();
        let mut slot = AppSlot::new();
        assert_eq!(slot.command(WasmRecord::Start, 0).0, Err(ErrorCode::BadArgument));
        assert_eq!(slot.status().state, WasmState::Empty);
        install(&mut slot, &image);
        let (st, actions) = slot.command(WasmRecord::Start, 0);
        assert_eq!(st.unwrap().state, WasmState::Running);
        assert_eq!(actions, [AppAction::Log("up".into())]);
        assert!(slot.tick(5).is_empty(), "first tick due at 10");
        assert_eq!(slot.tick(10), [AppAction::Publish("app/out".into(), b"up".to_vec())]);
        assert_eq!(slot.deliver("app/in", b"x", 12), [AppAction::Publish("app/out".into(), b"x".to_vec())]);
        assert!(slot.deliver("app/other", b"x", 12).is_empty());
        assert_eq!(slot.status().ticks, 1);
        slot.power_cycle();
        assert_eq!(slot.status().state, WasmState::Stopped);
        assert_eq!(slot.store().installed(), Some(image.as_slice()));
        assert_eq!(slot.autostart(0), [AppAction::Log("up".into())]);
        assert_eq!(slot.status().state, WasmState::Running);
        let (st, _) = slot.command(WasmRecord::Stop, 0);
        assert_eq!(st.unwrap().state, WasmState::Stopped);
        // Erase stops the payload and starts a new upload.
        let (st, _) = slot.command(WasmRecord::Erase { len: 10 }, 0);
        assert_eq!((st.as_ref().unwrap().state, st.unwrap().size), (WasmState::Loading, 0));
        assert_eq!(slot.command(WasmRecord::Activate { len: 10, crc: 0 }, 0).0, Err(ErrorCode::BadArgument));
    }

    #[test]
    fn fr39_rejected_modules_and_traps_report_trapped() {
        let mut slot = AppSlot::new();
        install(&mut slot, b"\0asm junk");
        let st = slot.command(WasmRecord::Start, 0).0.unwrap();
        assert_eq!(st.state, WasmState::Trapped);
        assert!(st.error.starts_with("invalid module"), "{:?}", st.error);
        // A tick that loops forever runs out of the badge's fuel.
        let spin = wat::parse_str(
            r#"(module (import "mw" "set_tick" (func $t (param i32) (result i32)))
                 (memory (export "memory") 1024 1024 (pagesize 1))
                 (func (export "init") (drop (call $t (i32.const 10))))
                 (func (export "tick") (param i32) (loop $l (br $l))))"#,
        )
        .unwrap();
        install(&mut slot, &spin);
        assert_eq!(slot.command(WasmRecord::Start, 0).0.unwrap().state, WasmState::Running);
        assert!(slot.tick(10).is_empty());
        let st = slot.status();
        assert_eq!(st.state, WasmState::Trapped);
        assert_eq!(st.error.as_str(), "out of fuel");
        // 64 KiB pages are over the 16 KiB memory cap.
        let big = wat::parse_str(r#"(module (memory (export "memory") 1))"#).unwrap();
        install(&mut slot, &big);
        let st = slot.command(WasmRecord::Start, 0).0.unwrap();
        assert_eq!(st.state, WasmState::Trapped);
        assert!(st.error.starts_with("memory needs 65536"), "{:?}", st.error);
    }
}
