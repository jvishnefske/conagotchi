//! The pet: an animated pixel-art cat shown on the PET screen.
//!
//! [`Pet`] is a small, copyable animation state machine. The user (SELECT on
//! the PET screen) or the host (`SetPetMode`) picks a [`PetMode`]; AUTO walks
//! through IDLE → WALK → EAT → PLAY → SLEEP → IDLE, spending
//! [`AUTO_DWELL_MS`] in each. [`Pet::advance`] is called on every tick and
//! derives the animation frame from the time spent in the active mode, so the
//! animation speed does not depend on the tick rate.
//!
//! Besides the frame, a mode may move the whole sprite: WALK drifts left and
//! right (mirroring the cat when it walks left) and PLAY bounces up while
//! leaping. [`Pet::offset`] reports that shift in screen pixels, bounded by
//! [`DRIFT_MAX`] and [`BOUNCE_MAX`] so the renderer can size its box.
//!
//! The art lives in [`sprites`]; [`pixel`] looks up one logical pixel. All
//! arithmetic is integer and every lookup is bounds-checked.

use badge_proto::PetMode;

use crate::color::Color;
use crate::io::Millis;

pub mod sprites;

use sprites::Frame;

/// Width and height of a frame, in logical pixels.
pub const SPRITE: u8 = 32;
/// Screen pixels per logical pixel (each side).
pub const SCALE: u8 = 4;
/// Time AUTO spends in each mode before moving to the next.
pub const AUTO_DWELL_MS: u64 = 8000;
/// Largest horizontal walk drift either side of centre, in screen pixels.
pub const DRIFT_MAX: i16 = 16;
/// Largest upward play bounce, in screen pixels.
pub const BOUNCE_MAX: i16 = 12;

/// Screen pixels the walking cat moves per frame.
const DRIFT_STEP: i16 = 2;
/// Frames per walk drift cycle (out right, back left, home again).
const DRIFT_CYCLE: u32 = 32;
/// Vertical offset of each PLAY frame (negative is up).
const PLAY_BOUNCE: [i16; 4] = [0, -8, -12, -6];

/// Modes in SELECT order; [`Pet::next_mode`] steps through this list.
pub const MODES: [PetMode; 6] = [
    PetMode::Auto,
    PetMode::Idle,
    PetMode::Walk,
    PetMode::Eat,
    PetMode::Play,
    PetMode::Sleep,
];

/// Modes AUTO cycles through, in order.
pub const AUTO_ORDER: [PetMode; 5] = [
    PetMode::Idle,
    PetMode::Walk,
    PetMode::Eat,
    PetMode::Play,
    PetMode::Sleep,
];

/// True for the six defined `PetMode` values.
pub fn is_valid(mode: PetMode) -> bool {
    MODES.contains(&mode)
}

/// Time each frame of `mode` is shown, in milliseconds (AUTO and unknown
/// values use the IDLE period).
pub fn frame_period(mode: PetMode) -> u64 {
    match mode {
        PetMode::Walk => 150,
        PetMode::Eat => 250,
        PetMode::Play => 120,
        PetMode::Sleep => 700,
        _ => 400,
    }
}

/// The frame sequence of `mode` (AUTO and unknown values show IDLE).
pub fn frames(mode: PetMode) -> &'static [&'static Frame] {
    match mode {
        PetMode::Walk => sprites::WALK,
        PetMode::Eat => sprites::EAT,
        PetMode::Play => sprites::PLAY,
        PetMode::Sleep => sprites::SLEEP,
        _ => sprites::IDLE,
    }
}

/// Number of frames in the sequence of `mode` (at least 1).
pub fn frame_count(mode: PetMode) -> u8 {
    u8::try_from(frames(mode).len()).unwrap_or(u8::MAX).max(1)
}

/// Colour of a palette character; `None` for transparent or unknown ones.
pub fn palette(c: u8) -> Option<Color> {
    match c {
        b'o' => Some(Color::new(40, 28, 48)),
        b'f' => Some(Color::new(245, 160, 70)),
        b's' => Some(Color::new(196, 98, 34)),
        b'p' => Some(Color::new(255, 150, 175)),
        b'w' => Some(Color::new(255, 248, 235)),
        b'a' => Some(Color::new(90, 190, 255)),
        _ => None,
    }
}

/// Colour of logical pixel (`x`, `y`) of frame `frame` of `mode`, or `None`
/// where the frame is transparent or the coordinates are outside it.
///
/// `frame` wraps around the mode's sequence length.
pub fn pixel(mode: PetMode, frame: u8, x: u8, y: u8) -> Option<Color> {
    let seq = frames(mode);
    let index = usize::from(frame) % seq.len().max(1);
    let row = seq.get(index)?.get(usize::from(y))?;
    palette(*row.as_bytes().get(usize::from(x))?)
}

/// A `PetMode` value outside the six defined modes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InvalidMode(pub PetMode);

/// What [`Pet::advance`] changed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PetChange {
    /// The picture changed: frame index, offset or facing.
    pub frame: bool,
    /// The active mode changed (AUTO moved on).
    pub mode: bool,
}

/// Animation state of the pet. See the module docs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pet {
    selected: PetMode,
    active: PetMode,
    /// When the active mode started (AUTO dwell and animation phase).
    since: Millis,
    /// Whole frame periods elapsed since `since`.
    step: u32,
}

impl Default for Pet {
    fn default() -> Self {
        Self::new()
    }
}

impl Pet {
    /// Boot state: AUTO selected, IDLE active, at time 0.
    pub fn new() -> Pet {
        Pet::start(PetMode::Auto, Millis(0))
    }

    /// Selected mode with its animation restarted at `now`; AUTO starts on
    /// IDLE. `mode` must be valid.
    fn start(mode: PetMode, now: Millis) -> Pet {
        let active = if mode == PetMode::Auto {
            PetMode::Idle
        } else {
            mode
        };
        Pet {
            selected: mode,
            active,
            since: now,
            step: 0,
        }
    }

    /// The mode chosen by the user or host (may be AUTO).
    pub fn selected(&self) -> PetMode {
        self.selected
    }

    /// The mode animating now (never AUTO).
    pub fn active(&self) -> PetMode {
        self.active
    }

    /// Index of the current frame in the active mode's sequence.
    pub fn frame(&self) -> u8 {
        let count = u32::from(frame_count(self.active));
        u8::try_from(self.step % count).unwrap_or(0)
    }

    /// Shift of the sprite from its home position, in screen pixels
    /// (`x` right, `y` down). Always within `±DRIFT_MAX` and
    /// `-BOUNCE_MAX..=0`.
    pub fn offset(&self) -> (i16, i16) {
        match self.active {
            PetMode::Walk => (self.drift(), 0),
            PetMode::Play => {
                let y = PLAY_BOUNCE
                    .get(usize::from(self.frame()))
                    .copied()
                    .unwrap_or(0);
                (0, y)
            }
            _ => (0, 0),
        }
    }

    /// True when the sprite must be drawn mirrored (walking left).
    pub fn flipped(&self) -> bool {
        self.active == PetMode::Walk && self.drift_phase() >= DRIFT_CYCLE / 2
    }

    /// Position in the walk drift cycle, starting from the centre moving right.
    fn drift_phase(&self) -> u32 {
        (self.step % DRIFT_CYCLE + DRIFT_CYCLE / 4) % DRIFT_CYCLE
    }

    /// Triangle wave over the drift cycle: 0 → right edge → left edge → 0.
    fn drift(&self) -> i16 {
        let t = i16::try_from(self.drift_phase()).unwrap_or(0);
        let half = i16::try_from(DRIFT_CYCLE / 2).unwrap_or(0);
        let quarter = half / 2;
        let p = if t < half {
            t - quarter
        } else {
            3 * quarter - t
        };
        p * DRIFT_STEP
    }

    /// Everything the renderer draws from this state.
    fn picture(&self) -> (PetMode, u8, (i16, i16), bool) {
        (self.active, self.frame(), self.offset(), self.flipped())
    }

    /// Selects `mode` at `now`, restarting its animation; AUTO restarts the
    /// cycle on IDLE.
    ///
    /// # Errors
    ///
    /// [`InvalidMode`] if `mode` is not one of the six defined values; the
    /// state is unchanged.
    pub fn select(self, mode: PetMode, now: Millis) -> Result<Pet, InvalidMode> {
        if is_valid(mode) {
            Ok(Pet::start(mode, now))
        } else {
            Err(InvalidMode(mode))
        }
    }

    /// Selects the mode after the selected one in [`MODES`], wrapping
    /// SLEEP → AUTO (the SELECT button on the PET screen).
    pub fn next_mode(self, now: Millis) -> Pet {
        let index = MODES.iter().position(|&m| m == self.selected).unwrap_or(0);
        let next = MODES
            .get((index + 1) % MODES.len())
            .copied()
            .unwrap_or(PetMode::Auto);
        Pet::start(next, now)
    }

    /// Advances the animation to `now`; in AUTO, moves to the next mode once
    /// the active one has run for [`AUTO_DWELL_MS`]. A clock that went
    /// backwards restarts the active mode's animation at `now`.
    pub fn advance(self, now: Millis) -> (Pet, PetChange) {
        let mut next = self;
        if now < next.since {
            next.since = now;
            next.step = 0;
        }
        let elapsed = now.0.saturating_sub(next.since.0);
        if next.selected == PetMode::Auto && elapsed >= AUTO_DWELL_MS {
            let index = AUTO_ORDER
                .iter()
                .position(|&m| m == next.active)
                .unwrap_or(0);
            next.active = AUTO_ORDER
                .get((index + 1) % AUTO_ORDER.len())
                .copied()
                .unwrap_or(PetMode::Idle);
            next.since = now;
            next.step = 0;
        } else {
            let steps = elapsed / frame_period(next.active).max(1);
            next.step = u32::try_from(steps).unwrap_or(u32::MAX);
        }
        let change = next.change_from(&self);
        (next, change)
    }

    /// What differs between `before` and this state, as the renderer sees it.
    pub fn change_from(&self, before: &Pet) -> PetChange {
        PetChange {
            frame: self.picture() != before.picture(),
            mode: self.active != before.active,
        }
    }
}

#[cfg(test)]
mod tests;
