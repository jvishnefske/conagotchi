//! One simulated badge: the functional core plus a tiny pure shell.
//!
//! [`SimBadge::handle`] feeds one [`Input`] to `App::handle` and carries out
//! the effects that need no outside world:
//!
//! - `Redraw` / `Animate` / `Draw` render onto the framebuffer (via
//!   `ui::render`, `ui::render_animation`, [`fb::apply_draw_op`]) and leave
//!   one [`ScreenChange`] record. `Animate` is skipped when a `Redraw` is in
//!   the same batch, as the firmware does.
//! - Radio requests the simulator has no hardware for are answered at once
//!   ([`local_answer`]): GPIO is `UNSUPPORTED`, Wi-Fi and BLE scans find
//!   nothing, Wi-Fi is never connected, BLE advertising succeeds. The
//!   answers are fed back as inputs, at most [`MAX_ROUNDS`] rounds per call
//!   (an answer only ever produces a response, so one round suffices).
//!
//! Every other effect becomes a [`crate::wire`] record for the runner. Records that
//! do not fit are counted in [`SimBadge::dropped`], never truncated.

#![forbid(unsafe_code)]

use badge_core::app::{App, BoardInfo};
use badge_core::io::{Effect, Input, MAX_EFFECTS, StorageEvent};
use badge_proto::{ErrorCode, WifiState};

use crate::fb::{self, FB_BYTES, FrameBuffer};
use crate::wire::{OutBuf, ScreenChange, WireError};

/// Firmware string reported by `GetInfo` in the simulator.
pub const FIRMWARE: &str = concat!("badge-sim ", env!("CARGO_PKG_VERSION"));
/// Most answer rounds per [`SimBadge::handle`] call.
pub const MAX_ROUNDS: usize = 4;

/// The immediate simulated answer to a hardware request, if it is one.
pub fn local_answer(effect: &Effect) -> Option<Input> {
    Some(match effect {
        Effect::GpioRead { seq, pin } | Effect::GpioWrite { seq, pin, .. } => {
            Input::Gpio { seq: *seq, pin: *pin, result: Err(ErrorCode::Unsupported) }
        }
        Effect::AdcRead { seq, .. } => Input::Adc { seq: *seq, result: Err(ErrorCode::Unsupported) },
        Effect::WifiScan { seq } => Input::WifiScanned { seq: *seq, result: Ok(heapless::Vec::new()) },
        Effect::WifiConnect { seq, .. } => Input::Wifi { seq: Some(*seq), result: Err(ErrorCode::Radio) },
        Effect::WifiStatus { seq } => Input::Wifi { seq: Some(*seq), result: Ok(WifiState::default()) },
        Effect::BleScan { seq, .. } => Input::BleScanned { seq: *seq, result: Ok(heapless::Vec::new()) },
        Effect::BleAdvertise { seq, .. } => Input::BleAdvertised { seq: *seq, result: Ok(()) },
        _ => return None,
    })
}

/// A simulated badge.
#[derive(Debug, Clone)]
pub struct SimBadge {
    app: App,
    dropped: u32,
}

impl SimBadge {
    /// Boots a badge with `mac`: a fresh app, told that storage is loaded
    /// with no radio key and no contacts (the badge sends unsigned packets),
    /// and an initial full render.
    pub fn boot<const N: usize>(mac: [u8; 6], fb: &mut [u8; FB_BYTES], out: &mut OutBuf<N>) -> SimBadge {
        let badge = SimBadge { app: App::new(BoardInfo { mac, firmware: FIRMWARE }), dropped: 0 };
        let badge = badge.handle(Input::Storage(StorageEvent::Loaded { key: None }), fb, out);
        badge.redraw(fb, out)
    }

    /// The core state.
    pub fn app(&self) -> &App {
        &self.app
    }

    /// Records dropped because the output buffer was full.
    pub fn dropped(&self) -> u32 {
        self.dropped
    }

    fn redraw<const N: usize>(self, fb: &mut [u8; FB_BYTES], out: &mut OutBuf<N>) -> SimBadge {
        let Ok(()) = badge_core::ui::render(&self.app, &mut FrameBuffer::new(fb));
        self.note(out.screen(ScreenChange::Redraw))
    }

    fn note(self, r: Result<(), WireError>) -> SimBadge {
        match r {
            Ok(()) | Err(WireError::NotARecord) => self,
            Err(WireError::Full | WireError::Encode) => SimBadge { dropped: self.dropped.wrapping_add(1), ..self },
        }
    }

    /// Handles one input; writes records to `out` and renders into `fb`.
    pub fn handle<const N: usize>(self, input: Input, fb: &mut [u8; FB_BYTES], out: &mut OutBuf<N>) -> SimBadge {
        let mut badge = self;
        let mut pending: heapless::Vec<Input, MAX_EFFECTS> = heapless::Vec::new();
        let mut next = Some(input);
        let mut rounds = 0usize;
        while let Some(input) = next.take().or_else(|| pending.pop()) {
            if rounds >= MAX_ROUNDS * MAX_EFFECTS {
                break;
            }
            rounds += 1;
            let (app, effects) = badge.app.handle(input);
            badge.app = app;
            let redraw = effects.iter().any(|e| matches!(e, Effect::Redraw));
            for effect in &effects {
                badge = match effect {
                    Effect::Redraw => badge,
                    Effect::Animate if redraw => badge,
                    Effect::Animate => {
                        let Ok(()) = badge_core::ui::render_animation(&badge.app, &mut FrameBuffer::new(fb));
                        badge.note(out.screen(ScreenChange::Animate))
                    }
                    Effect::Draw(op) => {
                        let Ok(()) = fb::apply_draw_op(*op, &mut FrameBuffer::new(fb));
                        badge.note(out.screen(ScreenChange::Draw))
                    }
                    Effect::SendScan { seq, kind } => {
                        let msg = Effect::Send(badge.app.scan_response(*kind, *seq));
                        badge.note(out.effect(&msg))
                    }
                    other => match local_answer(other) {
                        Some(answer) => {
                            let full = pending.push(answer).is_err();
                            SimBadge { dropped: badge.dropped.wrapping_add(u32::from(full)), ..badge }
                        }
                        None => badge.note(out.effect(other)),
                    },
                };
            }
            if redraw {
                badge = badge.redraw(fb, out);
            }
        }
        badge
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use std::boxed::Box;
    use std::vec::Vec;

    use badge_proto::{
        BadgeMsg, BadgeMsg_, HostMsg, HostMsg_::Cmd, Response_::Body, Screen, ShowScreen, WifiScan,
    };

    use super::*;
    use crate::wire::{Record, Records};

    const MAC: [u8; 6] = [2, 0, 0xC0, 0xFF, 0xEE, 1];

    fn responses(out: &[u8]) -> Vec<badge_proto::Response> {
        Records::new(out)
            .filter_map(|r| match r {
                Ok(Record::Send(pb)) => match badge_proto::decode::<BadgeMsg>(pb).ok()?.kind? {
                    BadgeMsg_::Kind::Response(r) => Some(r),
                    BadgeMsg_::Kind::Event(_) => None,
                },
                _ => None,
            })
            .collect()
    }

    #[test]
    fn fr39_boot_renders_home() {
        let mut fb = Box::new([0u8; FB_BYTES]);
        let mut out = OutBuf::<4096>::new();
        let b = SimBadge::boot(MAC, &mut fb, &mut out);
        assert!(Records::new(out.as_bytes()).any(|r| r == Ok(Record::Screen(ScreenChange::Redraw))));
        assert!(fb.iter().any(|&x| x != 0));
        assert_eq!(b.app().info().firmware, FIRMWARE);
        assert_eq!(b.dropped(), 0);
    }

    #[test]
    fn fr39_hardware_requests_are_answered_locally() {
        let mut fb = Box::new([0u8; FB_BYTES]);
        let mut out = OutBuf::<4096>::new();
        let b = SimBadge::boot(MAC, &mut fb, &mut out);
        out.clear();
        let b = b.handle(Input::Host(HostMsg { seq: 11, cmd: Some(Cmd::WifiScan(WifiScan {})) }), &mut fb, &mut out);
        let r = responses(out.as_bytes());
        assert_eq!(r.len(), 1);
        assert_eq!(r[0].seq, 11);
        assert!(matches!(&r[0].body, Some(Body::WifiScan(s)) if s.aps.is_empty()));
        out.clear();
        let b = b.handle(
            Input::Host(HostMsg { seq: 12, cmd: Some(Cmd::GpioRead(badge_proto::GpioRead { pin: 1 })) }),
            &mut fb,
            &mut out,
        );
        let r = responses(out.as_bytes());
        assert_eq!(r.len(), 1);
        assert!(matches!(&r[0].body, Some(Body::Error(_))));
        assert_eq!(b.dropped(), 0);
    }

    #[test]
    fn fr39_show_screen_redraws_once() {
        let mut fb = Box::new([0u8; FB_BYTES]);
        let mut out = OutBuf::<4096>::new();
        let b = SimBadge::boot(MAC, &mut fb, &mut out);
        out.clear();
        fb.fill(0);
        let b = b.handle(
            Input::Host(HostMsg { seq: 1, cmd: Some(Cmd::ShowScreen(ShowScreen { screen: Screen::Contacts })) }),
            &mut fb,
            &mut out,
        );
        let screens = Records::new(out.as_bytes()).filter(|r| matches!(r, Ok(Record::Screen(_)))).count();
        assert_eq!(screens, 1);
        assert_eq!(b.app().screen(), Screen::Contacts);
        assert!(fb.iter().any(|&x| x != 0));
    }

    #[test]
    fn fr39_local_answers_cover_every_hardware_request() {
        assert!(local_answer(&Effect::BleAdvertise { seq: 1, name: heapless::String::new() }).is_some());
        assert!(local_answer(&Effect::BleScan { seq: None, duration_ms: 1 }).is_some());
        assert!(local_answer(&Effect::WifiStatus { seq: 1 }).is_some());
        assert!(
            local_answer(&Effect::WifiConnect { seq: 1, ssid: heapless::String::new(), password: heapless::String::new() })
                .is_some()
        );
        assert!(local_answer(&Effect::GpioWrite { seq: 1, pin: 1, level: true }).is_some());
        assert!(local_answer(&Effect::AdcRead { seq: 1, pin: 1, samples: 16 }).is_some());
        assert!(local_answer(&Effect::Redraw).is_none());
        assert!(local_answer(&Effect::ClearContacts).is_none());
    }
}
