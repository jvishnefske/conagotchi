//! LED patterns and brightness scaling for the 8-LED WS2812B chain.
//!
//! [`render`] is a pure function of the pattern, the manual colour buffer,
//! the global brightness and the time. Animated patterns are derived from the
//! time alone, so they advance whenever the app ticks; the app passes the
//! *network* time from [`crate::sync`], so badges that hear each other show
//! the same phase. All arithmetic is integer; LED 0 is assumed to be the
//! leftmost LED.
//!
//! The patterns are OFF, MANUAL (host-set colours), RAINBOW and BREATHE.
//! [`next_on_select`] is the badge's SELECT cycle, which skips MANUAL.

use badge_proto::LedPattern;

use crate::board::led::RGB_COUNT;
use crate::color::Color;
use crate::io::Millis;

/// Brightness used at boot; WS2812Bs at full power are blinding.
pub const DEFAULT_BRIGHTNESS: u8 = 32;

/// Period of the rainbow rotation and the breathe cycle, in milliseconds.
pub const CYCLE_MS: u64 = 2000;

/// Colour wheel: 0 red → 85 green → 170 blue → back to red at 255.
pub fn hue(h: u8) -> Color {
    let h = u16::from(h);
    // Each third of the wheel ramps one channel down and the next one up.
    let (seg, pos) = (h / 85, (h % 85) * 3);
    let up = clamp_u8(pos);
    let down = clamp_u8(255 - pos);
    match seg {
        // h == 255 lands in segment 3 at position 0: back to red.
        0 | 3 => Color::new(down, up, 0),
        1 => Color::new(0, down, up),
        _ => Color::new(up, 0, down),
    }
}

/// Saturating conversion to a channel value.
fn clamp_u8(v: u16) -> u8 {
    u8::try_from(v).unwrap_or(u8::MAX)
}

/// Scales `c` by `level / 255`.
fn scale(c: Color, level: u8) -> Color {
    let s = |v: u8| clamp_u8((u16::from(v) * (u16::from(level) + 1)) >> 8);
    Color::new(s(c.r), s(c.g), s(c.b))
}

/// Rainbow: the wheel spread over the chain, rotating once per [`CYCLE_MS`].
fn rainbow(now: Millis) -> [Color; RGB_COUNT] {
    let offset = (now.0 % CYCLE_MS) * 256 / CYCLE_MS;
    let mut out = [Color::BLACK; RGB_COUNT];
    for (i, c) in out.iter_mut().enumerate() {
        let h = (offset + (i as u64) * 256 / RGB_COUNT as u64) % 256;
        *c = hue(u8::try_from(h).unwrap_or(0));
    }
    out
}

/// Breathe: warm white rising and falling linearly over [`CYCLE_MS`].
fn breathe(now: Millis) -> [Color; RGB_COUNT] {
    let half = CYCLE_MS / 2;
    let phase = now.0 % CYCLE_MS;
    let ramp = if phase < half {
        phase
    } else {
        CYCLE_MS - phase
    };
    let level = u8::try_from(ramp * 255 / half).unwrap_or(u8::MAX);
    [scale(Color::new(255, 220, 180), level); RGB_COUNT]
}

/// True for the patterns this firmware implements (OFF, MANUAL, RAINBOW,
/// BREATHE); the removed values 4 and 5 and anything else are rejected.
pub fn is_supported(pattern: LedPattern) -> bool {
    matches!(
        pattern,
        LedPattern::Off | LedPattern::Manual | LedPattern::Rainbow | LedPattern::Breathe
    )
}

/// The badge's SELECT cycle on the LEDS screen: OFF → RAINBOW → BREATHE →
/// OFF. MANUAL (reachable only from the host, GATT or mailbox) and unknown
/// values continue with OFF.
pub fn next_on_select(pattern: LedPattern) -> LedPattern {
    match pattern {
        LedPattern::Off => LedPattern::Rainbow,
        LedPattern::Rainbow => LedPattern::Breathe,
        _ => LedPattern::Off,
    }
}

/// Computes the 8 LED colours to show, already brightness-scaled.
///
/// OFF (and any unsupported pattern value) is all black; MANUAL shows
/// `manual`; RAINBOW and BREATHE animate from `now`. Every channel is then
/// scaled by `(c * (brightness + 1)) >> 8`, so 255 passes colours through
/// unchanged.
pub fn render(
    pattern: LedPattern,
    manual: &[Color; RGB_COUNT],
    brightness: u8,
    now: Millis,
) -> [Color; RGB_COUNT] {
    let raw = match pattern {
        LedPattern::Manual => *manual,
        LedPattern::Rainbow => rainbow(now),
        LedPattern::Breathe => breathe(now),
        _ => [Color::BLACK; RGB_COUNT],
    };
    raw.map(|c| scale(c, brightness))
}

#[cfg(test)]
mod tests {
    use super::*;

    const MANUAL: [Color; 8] = [
        Color::new(255, 0, 0),
        Color::new(0, 255, 0),
        Color::new(0, 0, 255),
        Color::new(255, 255, 255),
        Color::new(1, 2, 3),
        Color::new(128, 128, 128),
        Color::new(0, 0, 0),
        Color::new(10, 20, 30),
    ];

    #[test]
    fn hue_wheel_primaries_and_range() {
        assert_eq!(hue(0), Color::new(255, 0, 0));
        assert_eq!(hue(85), Color::new(0, 255, 0));
        assert_eq!(hue(170), Color::new(0, 0, 255));
        // Every hue is fully saturated: channels sum to 255.
        for h in 0..=255u8 {
            let c = hue(h);
            assert_eq!(
                u16::from(c.r) + u16::from(c.g) + u16::from(c.b),
                255,
                "hue {h}"
            );
        }
    }

    #[test]
    fn off_and_unknown_are_black() {
        for p in [LedPattern::Off, LedPattern(4), LedPattern(5), LedPattern(42), LedPattern(-1)] {
            assert_eq!(
                render(p, &MANUAL, 255, Millis(1234)),
                [Color::BLACK; 8]
            );
        }
    }

    #[test]
    fn manual_passes_through_at_full_brightness() {
        assert_eq!(
            render(LedPattern::Manual, &MANUAL, 255, Millis(0)),
            MANUAL
        );
    }

    #[test]
    fn brightness_scales_with_integer_math() {
        let out = render(LedPattern::Manual, &MANUAL, 127, Millis(0));
        assert_eq!(out[0], Color::new(127, 0, 0));
        assert_eq!(out[3], Color::new(127, 127, 127));
        let out = render(LedPattern::Manual, &MANUAL, 0, Millis(0));
        assert_eq!(out[3], Color::new(0, 0, 0));
        assert_eq!(
            render(
                LedPattern::Manual,
                &MANUAL,
                DEFAULT_BRIGHTNESS,
                Millis(0)
            )[0]
            .r,
            32
        );
    }

    #[test]
    fn rainbow_spreads_and_rotates() {
        let a = render(LedPattern::Rainbow, &MANUAL, 255, Millis(0));
        assert_eq!(a[0], hue(0));
        assert_eq!(a[4], hue(128));
        let b = render(LedPattern::Rainbow, &MANUAL, 255, Millis(500));
        assert_eq!(b[0], hue(64));
        assert_ne!(a, b);
        // One full period later it is back where it started.
        assert_eq!(
            render(LedPattern::Rainbow, &MANUAL, 255, Millis(CYCLE_MS)),
            a
        );
    }

    #[test]
    fn breathe_is_a_triangle_wave() {
        let at = |t| render(LedPattern::Breathe, &MANUAL, 255, Millis(t))[0];
        assert_eq!(at(0), Color::BLACK);
        assert_eq!(at(1000).r, 255);
        assert!(at(500).r > 100 && at(500).r < 160);
        assert_eq!(at(500), at(1500));
        assert_eq!(at(2000), Color::BLACK);
        let all = render(LedPattern::Breathe, &MANUAL, 255, Millis(700));
        assert!(all.iter().all(|c| *c == all[0]));
    }

    #[test]
    fn select_cycles_off_rainbow_breathe_and_skips_manual() {
        assert_eq!(next_on_select(LedPattern::Off), LedPattern::Rainbow);
        assert_eq!(next_on_select(LedPattern::Rainbow), LedPattern::Breathe);
        assert_eq!(next_on_select(LedPattern::Breathe), LedPattern::Off);
        assert_eq!(next_on_select(LedPattern::Manual), LedPattern::Off);
        assert_eq!(next_on_select(LedPattern(4)), LedPattern::Off);
    }

    #[test]
    fn only_four_patterns_are_supported() {
        for v in 0..4 {
            assert!(is_supported(LedPattern(v)));
        }
        for v in [4, 5, 6, -1] {
            assert!(!is_supported(LedPattern(v)));
        }
    }
}
