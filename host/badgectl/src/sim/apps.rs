//! Wasm apps inside simulated badges, run by the shared `wasm-runtime`.
//!
//! Each simulated badge owns one [`AppSlot`]: a `wasm_runtime::Runtime` plus
//! an [`AppHost`] that records what the app asked for during a call. The
//! world executes the recorded [`AppAction`]s afterwards (log lines become
//! `WasmLog` inputs, publishes go to the bus, LEDs / text / pet become host
//! messages with seq `probe_layout::WASM_SEQ`, like on the badge), so the
//! runtime never touches the world directly.
//!
//! [`AppSlot::command`] executes an `Effect::Wasm` command and returns the
//! answer for `Input::Wasm`. Runtime errors map onto protocol errors as
//! follows: a bad upload chunk, no loaded module and "not running" are
//! `BAD_ARGUMENT`; an invalid module or a trap is reported as `Ok` with the
//! runtime status (its `error` text says what happened).

use badge_core::color::Color;
use badge_core::probe_layout::WASM_SEQ;
use badge_proto::{
    DisplayText, ErrorCode, HostMsg, HostMsg_::Cmd, PetMode, Rgb, SetLeds, SetPetMode, WasmStatus,
};
use badge_sim::wire::WasmRecord;
use wasm_runtime::{Host, Runtime, RuntimeError, Status};

/// Something an app did during one runtime call.
#[derive(Debug, Clone, PartialEq)]
pub enum AppAction {
    /// A log line.
    Log(String),
    /// A publish on an app key.
    Publish(String, Vec<u8>),
    /// A host message on behalf of the app (`WASM_SEQ`); boxed, it dwarfs
    /// the other variants.
    Host(Box<HostMsg>),
}

/// The runtime's view of a simulated badge: collects actions.
#[derive(Debug, Default)]
pub struct AppHost {
    now_ms: i64,
    actions: Vec<AppAction>,
}

impl AppHost {
    fn host_msg(&mut self, cmd: Cmd) {
        self.actions.push(AppAction::Host(Box::new(HostMsg { seq: WASM_SEQ, cmd: Some(cmd) })));
    }
}

/// Converts `s` to a heapless string, refusing (not truncating) overlong text.
fn hstring<const N: usize>(s: &str) -> Result<heapless::String<N>, Status> {
    let mut out = heapless::String::new();
    out.push_str(s).map_err(|_| Status::TooLong)?;
    Ok(out)
}

impl Host for AppHost {
    fn now_ms(&mut self) -> i64 {
        self.now_ms
    }

    fn log(&mut self, line: &str) {
        self.actions.push(AppAction::Log(line.to_string()));
    }

    fn publish(&mut self, key: &str, payload: &[u8]) -> Result<(), Status> {
        if !badge_mw::keys::app_may_use(key) {
            return Err(Status::BadKey);
        }
        self.actions.push(AppAction::Publish(key.to_string(), payload.to_vec()));
        Ok(())
    }

    fn set_leds(&mut self, rgb: &[u8; 3 * wasm_abi::LED_COUNT]) -> Result<(), Status> {
        let mut colors = heapless::Vec::new();
        for &[r, g, b] in rgb.as_chunks::<3>().0 {
            colors.push(Rgb { rgb: Color::new(r, g, b).packed() }).map_err(|_| Status::TooLong)?;
        }
        self.host_msg(Cmd::SetLeds(SetLeds { colors }));
        Ok(())
    }

    fn show_text(&mut self, text: &str) -> Result<(), Status> {
        self.host_msg(Cmd::DisplayText(DisplayText { text: hstring(text)?, fg: None, bg: None }));
        Ok(())
    }

    fn set_pet(&mut self, mode: i32) -> Result<(), Status> {
        if !(0..=5).contains(&mode) {
            return Err(Status::BadArgument);
        }
        self.host_msg(Cmd::SetPetMode(SetPetMode { mode: PetMode(mode) }));
        Ok(())
    }
}

/// The protocol answer for a runtime result.
pub fn answer(result: Result<(), RuntimeError>, status: WasmStatus) -> Result<WasmStatus, ErrorCode> {
    match result {
        Ok(()) | Err(RuntimeError::Invalid | RuntimeError::Trapped) => Ok(status),
        Err(RuntimeError::BadChunk | RuntimeError::NotLoaded | RuntimeError::NotRunning) => {
            Err(ErrorCode::BadArgument)
        }
    }
}

/// One badge's wasm runtime.
#[derive(Default)]
pub struct AppSlot {
    runtime: Runtime,
    host: AppHost,
}

impl AppSlot {
    /// An empty slot.
    pub fn new() -> Self {
        Self::default()
    }

    /// Executes a runtime command at network time `now_ms`; returns the
    /// answer and the app's actions.
    pub fn command(&mut self, cmd: WasmRecord<'_>, now_ms: i64) -> (Result<WasmStatus, ErrorCode>, Vec<AppAction>) {
        self.host.now_ms = now_ms;
        let result = match cmd {
            WasmRecord::Load { offset, total, data } => self.runtime.load_chunk(offset, total, data),
            WasmRecord::Start => self.runtime.start(&mut self.host),
            WasmRecord::Stop => {
                self.runtime.stop();
                Ok(())
            }
            WasmRecord::Status => Ok(()),
        };
        let reply = answer(result, self.runtime.status());
        (reply, std::mem::take(&mut self.host.actions))
    }

    /// Runs one app tick if an app is running; returns its actions.
    pub fn tick(&mut self, now_ms: i64) -> Vec<AppAction> {
        if self.runtime.status().state != badge_proto::WasmState::Running {
            return Vec::new();
        }
        self.host.now_ms = now_ms;
        // A trap stops the app; the status reports it, so the error itself
        // carries nothing more.
        let _ = self.runtime.tick(&mut self.host, now_ms);
        std::mem::take(&mut self.host.actions)
    }

    /// Hands a received sample to the app if it subscribed to `key`.
    pub fn deliver(&mut self, key: &str, payload: &[u8]) -> bool {
        self.runtime.deliver(key, payload)
    }

    /// Keys the app subscribed to.
    pub fn subscriptions(&self) -> Vec<String> {
        self.runtime.subscriptions().map(str::to_string).collect()
    }

    /// Runtime status.
    pub fn status(&self) -> WasmStatus {
        self.runtime.status()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fr39_app_host_records_actions_and_checks_arguments() {
        let mut h = AppHost::default();
        h.log("hi");
        assert_eq!(h.publish("conagotchi/sync", b"x"), Err(Status::BadKey));
        assert_eq!(h.publish("app/hello", b"x"), Ok(()));
        assert_eq!(h.set_pet(9), Err(Status::BadArgument));
        assert_eq!(h.set_pet(2), Ok(()));
        assert_eq!(h.show_text(&"x".repeat(300)), Err(Status::TooLong));
        assert_eq!(h.show_text("yo"), Ok(()));
        let mut rgb = [0u8; 24];
        rgb[0] = 255;
        assert_eq!(h.set_leds(&rgb), Ok(()));
        assert_eq!(h.actions[0], AppAction::Log("hi".into()));
        assert_eq!(h.actions[1], AppAction::Publish("app/hello".into(), b"x".to_vec()));
        let AppAction::Host(m) = &h.actions[2] else { panic!() };
        assert_eq!(m.seq, WASM_SEQ);
        assert!(matches!(m.cmd, Some(Cmd::SetPetMode(SetPetMode { mode: PetMode::Walk }))));
        let AppAction::Host(m) = &h.actions[4] else { panic!() };
        let Some(Cmd::SetLeds(l)) = &m.cmd else { panic!() };
        assert_eq!(l.colors.len(), 8);
        assert_eq!(l.colors[0].rgb, 0xFF_0000);
    }

    #[test]
    fn fr39_runtime_errors_map_to_protocol_answers() {
        let st = WasmStatus::default();
        assert_eq!(answer(Ok(()), st.clone()), Ok(st.clone()));
        assert_eq!(answer(Err(RuntimeError::Trapped), st.clone()), Ok(st.clone()));
        assert_eq!(answer(Err(RuntimeError::Invalid), st.clone()), Ok(st.clone()));
        assert_eq!(answer(Err(RuntimeError::BadChunk), st.clone()), Err(ErrorCode::BadArgument));
        assert_eq!(answer(Err(RuntimeError::NotLoaded), st), Err(ErrorCode::BadArgument));
    }
}
