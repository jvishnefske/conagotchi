//! Debouncing of raw button levels into press/release edges.
//!
//! The shell samples the five active-low buttons every tick and hands the
//! core a raw [`ButtonMask`]. [`Debouncer`] accepts a new level for a button
//! only after the raw level has been stable for [`DEBOUNCE_MS`], and reports
//! each accepted change as an edge. It is a plain value: `update` consumes the
//! old state and returns the new one.

use badge_proto::Button;
use heapless::Vec;

use crate::io::{ButtonMask, Millis};

/// A raw level must hold this long (milliseconds) before it is accepted.
pub const DEBOUNCE_MS: u64 = 30;

/// Number of real buttons (protocol values 1..=5).
pub const BUTTON_COUNT: usize = 5;

/// All real buttons in protocol order; edges are reported in this order.
pub const BUTTONS: [Button; BUTTON_COUNT] = [
    Button::Boot,
    Button::Left,
    Button::Select,
    Button::Start,
    Button::Right,
];

/// Edges reported by one update: `(button, pressed)`.
pub type Edges = Vec<(Button, bool), BUTTON_COUNT>;

/// Per-button debounce state for the five badge buttons.
///
/// Bits of the input mask that do not correspond to a real button (bit 0,
/// bits 6 and 7) are ignored.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Debouncer {
    /// Debounced (accepted) levels.
    stable: ButtonMask,
    /// Raw levels seen on the previous update.
    raw: ButtonMask,
    /// When each button's raw level last changed, indexed like [`BUTTONS`].
    changed_at: [Millis; BUTTON_COUNT],
}

/// Returns true if bit `button` is set in `mask`.
fn bit(mask: ButtonMask, button: Button) -> bool {
    u32::try_from(button.0)
        .ok()
        .and_then(|b| 1u8.checked_shl(b))
        .is_some_and(|m| mask.0 & m != 0)
}

/// Returns `mask` with bit `button` set to `level`.
fn with_bit(mask: ButtonMask, button: Button, level: bool) -> ButtonMask {
    let m = u32::try_from(button.0)
        .ok()
        .and_then(|b| 1u8.checked_shl(b))
        .unwrap_or(0);
    if level {
        ButtonMask(mask.0 | m)
    } else {
        ButtonMask(mask.0 & !m)
    }
}

impl Debouncer {
    /// Feeds the raw levels sampled at `now`; returns the new state and every
    /// accepted edge, in [`BUTTONS`] order.
    pub fn update(self, now: Millis, raw: ButtonMask) -> (Debouncer, Edges) {
        self.update_limited(now, raw, BUTTON_COUNT)
    }

    /// Like [`Debouncer::update`] but accepts at most `limit` edges. Buttons
    /// whose change is not accepted keep their old debounced level and are
    /// accepted on a later update (their raw level is already stable), so
    /// no edge is lost unless the raw level flips back first.
    pub fn update_limited(self, now: Millis, raw: ButtonMask, limit: usize) -> (Debouncer, Edges) {
        let mut next = self;
        let mut edges = Edges::new();
        for (button, changed_at) in BUTTONS.iter().zip(next.changed_at.iter_mut()) {
            let level = bit(raw, *button);
            if level != bit(next.raw, *button) {
                *changed_at = now;
            }
            let settled = now.0.saturating_sub(changed_at.0) >= DEBOUNCE_MS;
            if settled && level != bit(next.stable, *button) && edges.len() < limit {
                next.stable = with_bit(next.stable, *button, level);
                edges.push((*button, level)).ok();
            }
        }
        next.raw = ButtonMask(raw.0 & !0b1100_0001);
        (next, edges)
    }

    /// Debounced pressed mask (bit n = `Button(n)` held).
    pub fn pressed(&self) -> ButtonMask {
        self.stable
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    const LEFT: ButtonMask = ButtonMask(1 << 2);

    fn run(d: Debouncer, steps: &[(u64, u8)]) -> (Debouncer, std::vec::Vec<(u64, Button, bool)>) {
        let mut d = d;
        let mut out = std::vec::Vec::new();
        for &(t, raw) in steps {
            let (n, edges) = d.update(Millis(t), ButtonMask(raw));
            d = n;
            out.extend(edges.iter().map(|&(b, p)| (t, b, p)));
        }
        (d, out)
    }

    #[test]
    fn press_accepted_after_stable_period() {
        let (d, edges) = run(
            Debouncer::default(),
            &[(0, LEFT.0), (20, LEFT.0), (29, LEFT.0), (30, LEFT.0)],
        );
        assert_eq!(edges, std::vec![(30, Button::Left, true)]);
        assert_eq!(d.pressed(), LEFT);
    }

    #[test]
    fn bounce_is_rejected() {
        // Chatter shorter than DEBOUNCE_MS never produces an edge.
        let (d, edges) = run(
            Debouncer::default(),
            &[
                (0, LEFT.0),
                (10, 0),
                (20, LEFT.0),
                (35, 0),
                (50, LEFT.0),
                (70, 0),
            ],
        );
        assert!(edges.is_empty());
        assert_eq!(d.pressed(), ButtonMask(0));
    }

    #[test]
    fn press_then_release_each_emit_one_edge() {
        let (d, edges) = run(
            Debouncer::default(),
            &[
                (0, LEFT.0),
                (40, LEFT.0),
                (60, LEFT.0),
                (100, 0),
                (120, 0),
                (140, 0),
                (160, 0),
            ],
        );
        assert_eq!(
            edges,
            std::vec![(40, Button::Left, true), (140, Button::Left, false)]
        );
        assert_eq!(d.pressed(), ButtonMask(0));
    }

    #[test]
    fn invalid_bits_are_ignored() {
        let (d, edges) = run(
            Debouncer::default(),
            &[(0, 0b1100_0001), (100, 0b1100_0001)],
        );
        assert!(edges.is_empty());
        assert_eq!(d.pressed(), ButtonMask(0));
    }

    #[test]
    fn simultaneous_edges_reported_in_order() {
        let all = 0b0011_1110;
        let (d, edges) = run(Debouncer::default(), &[(0, all), (30, all)]);
        let buttons: std::vec::Vec<_> = edges.iter().map(|e| e.1).collect();
        assert_eq!(buttons, BUTTONS.to_vec());
        assert_eq!(d.pressed(), ButtonMask(all));
    }

    #[test]
    fn limited_update_defers_extra_edges() {
        let all = ButtonMask(0b0011_1110);
        let d = Debouncer::default().update(Millis(0), all).0;
        let (d, e1) = d.update_limited(Millis(30), all, 2);
        assert_eq!(e1.as_slice(), &[(Button::Boot, true), (Button::Left, true)]);
        let (d, e2) = d.update_limited(Millis(50), all, 2);
        assert_eq!(
            e2.as_slice(),
            &[(Button::Select, true), (Button::Start, true)]
        );
        let (d, e3) = d.update_limited(Millis(70), all, 2);
        assert_eq!(e3.as_slice(), &[(Button::Right, true)]);
        assert_eq!(d.pressed(), all);
    }
}
