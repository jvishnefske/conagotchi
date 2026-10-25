//! Pure decisions of the badge's wasm payload runner (the firmware's wasm
//! task and its simulator twin): the payload lifecycle reported in
//! `WasmStatus`, the boot-time autostart / safe-mode choice, the button
//! edges delivered to payloads, and the mapping of publish refusals to the
//! "mw" ABI's error codes.
//!
//! [`Lifecycle`] is a value type: every transition consumes it and returns
//! the next one, so the imperative shell only moves it along. Its phase is
//! middleware's `loader::PayloadStatus`, whose values are the
//! `badge_proto::WasmState` values (the proto says so), so
//! [`Lifecycle::status`] maps one to the other by number.
//!
//! The transitions follow the core's [`WasmCmd`](badge_core::io::WasmCmd)
//! contract:
//!
//! - `Erase` → [`Lifecycle::erased`] (LOADING, size 0);
//! - each `Write` → [`Lifecycle::written`] (size = bytes received so far);
//! - `Activate` → [`Lifecycle::installed`] (STOPPED, size = image length);
//! - a flash failure anywhere → [`Lifecycle::storage_failed`] (EMPTY: the
//!   partition can no longer be trusted);
//! - `Start` → [`Lifecycle::started`] (RUNNING, ticks reset) or
//!   [`Lifecycle::load_failed`] (STOPPED with the error text);
//! - a trap or fuel exhaustion → [`Lifecycle::trapped`] (TRAPPED);
//! - `Stop` → [`Lifecycle::stopped`] (STOPPED if a payload was running or
//!   had trapped).

use badge_core::io::ButtonMask;
use badge_proto::{BadgeMsg, BadgeMsg_, Button, Event_, WasmState, WasmStatus};
use core::fmt::{self, Write as _};
use heapless::String;
use middleware::loader::PayloadStatus;
use middleware::wasm_abi::AbiError;

use crate::keys::RouteError;

/// Capacity of `WasmStatus.error` (`badge-proto/build.rs`).
pub const ERROR_LEN: usize = 48;

/// The payload's lifecycle as reported in `WasmStatus`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Lifecycle {
    phase: PayloadStatus,
    size: u32,
    ticks: u32,
    error: String<ERROR_LEN>,
}

impl Lifecycle {
    /// After boot: STOPPED with the installed image's length, or EMPTY.
    pub fn boot(installed: Option<u32>) -> Self {
        match installed {
            Some(len) => Self::with(PayloadStatus::Stopped, len),
            None => Self::with(PayloadStatus::Empty, 0),
        }
    }

    fn with(phase: PayloadStatus, size: u32) -> Self {
        Lifecycle {
            phase,
            size,
            ticks: 0,
            error: String::new(),
        }
    }

    /// The current phase.
    pub fn phase(&self) -> PayloadStatus {
        self.phase
    }

    /// True while an upload is in progress (between `Erase` and `Activate`).
    pub fn loading(&self) -> bool {
        self.phase == PayloadStatus::Loading
    }

    /// True if an image is installed, so `Start` may load it.
    pub fn startable(&self) -> bool {
        matches!(
            self.phase,
            PayloadStatus::Stopped | PayloadStatus::Running | PayloadStatus::Trapped
        )
    }

    /// The partition was erased for a new image; any payload was stopped.
    pub fn erased(self) -> Self {
        Self::with(PayloadStatus::Loading, 0)
    }

    /// Image bytes up to `end` are written.
    pub fn written(self, end: u32) -> Self {
        Lifecycle { size: end, ..self }
    }

    /// The image of `len` bytes is installed (header written, CRC verified).
    pub fn installed(self, len: u32) -> Self {
        Self::with(PayloadStatus::Stopped, len)
    }

    /// A flash operation failed: no trusted image remains.
    pub fn storage_failed(self, why: impl fmt::Display) -> Self {
        Lifecycle {
            error: error_text(why),
            ..Self::with(PayloadStatus::Empty, 0)
        }
    }

    /// The installed payload was loaded and its `init` ran.
    pub fn started(self) -> Self {
        Self::with(PayloadStatus::Running, self.size)
    }

    /// The installed image could not be read or did not load; it stays
    /// installed but stopped.
    pub fn load_failed(self, why: impl fmt::Display) -> Self {
        Lifecycle {
            error: error_text(why),
            ..Self::with(PayloadStatus::Stopped, self.size)
        }
    }

    /// The running payload trapped or ran out of fuel.
    pub fn trapped(self, why: impl fmt::Display) -> Self {
        Lifecycle {
            phase: PayloadStatus::Trapped,
            error: error_text(why),
            ..self
        }
    }

    /// The payload was stopped; an upload in progress or an empty slot is
    /// unaffected.
    pub fn stopped(self) -> Self {
        match self.phase {
            PayloadStatus::Running | PayloadStatus::Trapped => Lifecycle {
                phase: PayloadStatus::Stopped,
                ..self
            },
            _ => self,
        }
    }

    /// One `tick` call ran.
    pub fn ticked(self) -> Self {
        Lifecycle {
            ticks: self.ticks.wrapping_add(1),
            ..self
        }
    }

    /// The `WasmStatus` answer, with the partition's `capacity`.
    pub fn status(&self, capacity: u32) -> WasmStatus {
        WasmStatus {
            state: WasmState(self.phase as i32),
            size: self.size,
            ticks: self.ticks,
            error: self.error.clone(),
            capacity,
        }
    }
}

/// Heap a payload needs beyond its linear memory and image: wasm-host's
/// engine, store and instance (about 11.6 KiB measured) plus 8 KiB of
/// allocator overhead and fragmentation margin.
pub const LOAD_HEAP_OVERHEAD: u32 = 20 * 1024;

/// Free heap required before loading an image of `image_len` bytes into a
/// payload with `memory_bytes` of linear memory: the overhead, the memory,
/// and twice the image (the RAM copy read from flash, plus wasmi's
/// compiled code, which lazy compilation fills in as functions first run).
///
/// Measured with wasm-host at middleware rev 6b1720e on a 32-bit target
/// (wasm32-wasip1 under a counting allocator) with 16 KiB of memory: peak
/// use is 28 KiB + 2 × image (37 KiB for a 4.7 KiB image, 59 KiB for
/// 15.6 KiB), which this bound covers with the margin.
pub const fn load_heap_needed(image_len: u32, memory_bytes: u32) -> u32 {
    LOAD_HEAP_OVERHEAD.saturating_add(memory_bytes).saturating_add(image_len.saturating_mul(2))
}

/// What the wasm task does with an installed image at boot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootAction {
    /// Nothing installed (or the image failed its CRC): stay EMPTY.
    Idle,
    /// Load and start the installed payload.
    Autostart,
    /// An image is installed but BOOT was held at reset: leave it STOPPED.
    SafeMode,
}

/// Decides the boot action from whether a valid image is installed and
/// whether the BOOT button was held at reset.
pub fn boot_action(installed: bool, boot_held: bool) -> BootAction {
    match (installed, boot_held) {
        (false, _) => BootAction::Idle,
        (true, true) => BootAction::SafeMode,
        (true, false) => BootAction::Autostart,
    }
}

/// True if `button` is held in a raw button sample (bit n = `Button(n)`),
/// e.g. BOOT at reset for safe mode.
pub fn held(raw: ButtonMask, button: Button) -> bool {
    u32::try_from(button.0)
        .ok()
        .and_then(|n| 1u8.checked_shl(n))
        .is_some_and(|bit| raw.0 & bit != 0)
}

/// The button edge carried by a `ButtonEvent` the core sends to the host,
/// if `msg` is one.
pub fn button_edge(msg: &BadgeMsg) -> Option<(Button, bool)> {
    match &msg.kind {
        Some(BadgeMsg_::Kind::Event(event)) => match &event.kind {
            Some(Event_::Kind::Button(b)) => Some((b.button, b.pressed)),
            _ => None,
        },
        _ => None,
    }
}

/// The ABI error a refused payload publish returns.
pub fn abi_error(e: RouteError) -> AbiError {
    match e {
        RouteError::UnknownKey => AbiError::InvalidKey,
        RouteError::BadPayload => AbiError::InvalidArgument,
    }
}

/// `text` cut to at most `N` bytes on a character boundary.
pub fn truncate<const N: usize>(text: &str) -> String<N> {
    let mut out = String::new();
    // Bounded by N: push fails once full.
    for c in text.chars() {
        if out.push(c).is_err() {
            break;
        }
    }
    out
}

/// A `Display` value rendered into [`ERROR_LEN`] bytes, cut on a character
/// boundary.
pub fn error_text(why: impl fmt::Display) -> String<ERROR_LEN> {
    let mut out = Truncating(String::new());
    // `Truncating` never fails; a formatter error only ends the text early.
    let _ = write!(out, "{why}");
    out.0
}

/// A `fmt::Write` sink that keeps what fits and drops the rest.
struct Truncating<const N: usize>(String<N>);

impl<const N: usize> fmt::Write for Truncating<N> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let room = N - self.0.len();
        let cut = (0..=room.min(s.len()))
            .rev()
            .find(|&i| s.is_char_boundary(i))
            .unwrap_or(0);
        let head = s.get(..cut).unwrap_or("");
        // `head` fits by construction.
        let _ = self.0.push_str(head);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use badge_proto::{ButtonEvent, Event};

    #[test]
    fn fr41_upload_install_start_stop_walks_the_lifecycle() {
        let l = Lifecycle::boot(None);
        assert_eq!(l.status(16384).state, WasmState::Empty);
        assert_eq!(l.status(16384).capacity, 16384);
        assert!(!l.startable());
        let l = l.erased();
        assert!(l.loading());
        let l = l.written(192).written(300);
        let s = l.status(16384);
        assert_eq!((s.state, s.size), (WasmState::Loading, 300));
        let l = l.installed(300);
        assert_eq!(
            (l.status(0).state, l.status(0).size),
            (WasmState::Stopped, 300)
        );
        assert!(l.startable());
        let l = l.started().ticked().ticked();
        let s = l.status(0);
        assert_eq!((s.state, s.ticks, s.size), (WasmState::Running, 2, 300));
        let l = l.stopped();
        assert_eq!(l.status(0).state, WasmState::Stopped);
        assert_eq!(l.status(0).ticks, 2, "ticks stay until the next start");
        assert_eq!(l.started().status(0).ticks, 0);
    }

    #[test]
    fn fr41_traps_load_errors_and_flash_failures_are_reported() {
        let l = Lifecycle::boot(Some(100)).started().trapped("out of fuel");
        let s = l.status(0);
        assert_eq!(
            (s.state, s.error.as_str()),
            (WasmState::Trapped, "out of fuel")
        );
        assert!(l.startable());
        assert_eq!(l.clone().stopped().status(0).state, WasmState::Stopped);
        let l = l.started();
        assert_eq!(l.status(0).error.as_str(), "", "a restart clears the error");
        let l = Lifecycle::boot(Some(100)).load_failed("unknown import mw.x");
        let s = l.status(0);
        assert_eq!(
            (s.state, s.size, s.error.as_str()),
            (WasmState::Stopped, 100, "unknown import mw.x")
        );
        let l = Lifecycle::boot(Some(100))
            .erased()
            .storage_failed("flash error");
        let s = l.status(0);
        assert_eq!(
            (s.state, s.size, s.error.as_str()),
            (WasmState::Empty, 0, "flash error")
        );
        assert!(!l.startable());
        // Stop does not disturb an upload or an empty slot.
        assert!(Lifecycle::boot(None).erased().stopped().loading());
        assert_eq!(
            Lifecycle::boot(None).stopped().phase(),
            PayloadStatus::Empty
        );
    }

    #[test]
    fn fr41_phase_numbers_match_the_proto_states() {
        for (phase, state) in [
            (PayloadStatus::Empty, WasmState::Empty),
            (PayloadStatus::Stopped, WasmState::Stopped),
            (PayloadStatus::Running, WasmState::Running),
            (PayloadStatus::Trapped, WasmState::Trapped),
            (PayloadStatus::Loading, WasmState::Loading),
        ] {
            assert_eq!(WasmState(phase as i32), state);
        }
    }

    #[test]
    fn heap_bound_covers_the_measured_peaks() {
        // (image bytes, measured peak) with 16 KiB of memory.
        for (image, peak) in [(4712u32, 37_053u32), (9308, 46_313), (15_665, 58_909)] {
            let need = load_heap_needed(image, 16 * 1024);
            assert!(need >= peak + 4096, "{image}: {need} < {peak} + margin");
        }
        assert_eq!(load_heap_needed(u32::MAX, 1), u32::MAX);
    }

    #[test]
    fn fr41_boot_autostarts_unless_boot_is_held() {
        assert_eq!(boot_action(false, false), BootAction::Idle);
        assert_eq!(boot_action(false, true), BootAction::Idle);
        assert_eq!(boot_action(true, false), BootAction::Autostart);
        assert_eq!(boot_action(true, true), BootAction::SafeMode);
        assert!(held(ButtonMask(0b00_0010), Button::Boot));
        assert!(!held(ButtonMask(0b11_1101), Button::Boot));
        assert!(held(ButtonMask(0b01_0000), Button::Start));
        assert!(!held(ButtonMask(0xFF), Button(9)));
        assert!(!held(ButtonMask(0xFF), Button(-1)));
    }

    #[test]
    fn fr42_button_events_become_edges_and_other_messages_do_not() {
        let msg = |kind| BadgeMsg {
            kind: Some(BadgeMsg_::Kind::Event(Event { kind: Some(kind) })),
        };
        let press = msg(Event_::Kind::Button(ButtonEvent {
            button: Button::Start,
            pressed: true,
            at_ms: 5,
        }));
        assert_eq!(button_edge(&press), Some((Button::Start, true)));
        let release = msg(Event_::Kind::Button(ButtonEvent {
            button: Button::Boot,
            pressed: false,
            at_ms: 6,
        }));
        assert_eq!(button_edge(&release), Some((Button::Boot, false)));
        assert_eq!(button_edge(&BadgeMsg { kind: None }), None);
        let log = msg(Event_::Kind::WasmLog(badge_proto::WasmLog {
            text: String::new(),
        }));
        assert_eq!(button_edge(&log), None);
    }

    #[test]
    fn fr42_publish_refusals_map_to_abi_errors() {
        assert_eq!(abi_error(RouteError::UnknownKey), AbiError::InvalidKey);
        assert_eq!(abi_error(RouteError::BadPayload), AbiError::InvalidArgument);
    }

    #[test]
    fn texts_are_cut_on_character_boundaries() {
        assert_eq!(truncate::<4>("abcdef").as_str(), "abcd");
        assert_eq!(truncate::<4>("aé€").as_str(), "aé");
        assert_eq!(truncate::<8>("hi").as_str(), "hi");
        let long = error_text(format_args!(
            "{}{}",
            std::string::String::from("x").repeat(40),
            "€€€€"
        ));
        assert_eq!(long.len(), 46);
        assert!(long.ends_with('€'));
        assert_eq!(error_text("short").as_str(), "short");
    }
}
