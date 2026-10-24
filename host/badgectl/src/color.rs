//! Colour argument parsing.
//!
//! Accepts `#rrggbb`, `rrggbb`, `0xrrggbb` (case-insensitive hex) and a small
//! set of names. The result is the shared [`badge_core::color::Color`], which
//! packs into the protocol's `0x00RRGGBB` word.

use badge_core::color::Color;

/// A colour argument could not be understood.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error(
    "invalid colour `{0}`: expected #rrggbb, rrggbb, 0xrrggbb or one of \
     red, green, blue, white, black, off, purple, orange, yellow, cyan"
)]
pub struct ColorError(pub String);

/// Named colours accepted on the command line.
const NAMED: &[(&str, Color)] = &[
    ("red", Color::new(255, 0, 0)),
    ("green", Color::new(0, 255, 0)),
    ("blue", Color::new(0, 0, 255)),
    ("white", Color::WHITE),
    ("black", Color::BLACK),
    ("off", Color::BLACK),
    ("purple", Color::new(128, 0, 255)),
    ("orange", Color::new(255, 96, 0)),
    ("yellow", Color::new(255, 255, 0)),
    ("cyan", Color::new(0, 255, 255)),
];

/// Parses a colour argument.
pub fn parse_color(input: &str) -> Result<Color, ColorError> {
    let lower = input.trim().to_ascii_lowercase();
    if let Some((_, c)) = NAMED.iter().find(|(name, _)| *name == lower) {
        return Ok(*c);
    }
    let hex = lower
        .strip_prefix('#')
        .or_else(|| lower.strip_prefix("0x"))
        .unwrap_or(&lower);
    if hex.len() != 6 || !hex.bytes().all(|b| b.is_ascii_hexdigit()) {
        return Err(ColorError(input.to_string()));
    }
    u32::from_str_radix(hex, 16)
        .map(Color::from_packed)
        .map_err(|_| ColorError(input.to_string()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hex_forms_parse() {
        let want = Color::new(0x12, 0xab, 0xef);
        assert_eq!(parse_color("#12abef"), Ok(want));
        assert_eq!(parse_color("12ABEF"), Ok(want));
        assert_eq!(parse_color("0x12aBeF"), Ok(want));
        assert_eq!(parse_color(" 0X12abef "), Ok(want));
    }

    #[test]
    fn names_parse() {
        assert_eq!(parse_color("red"), Ok(Color::new(255, 0, 0)));
        assert_eq!(parse_color("Off"), Ok(Color::BLACK));
        assert_eq!(parse_color("black"), Ok(Color::BLACK));
        assert_eq!(parse_color("WHITE"), Ok(Color::WHITE));
        assert_eq!(parse_color("cyan").map(Color::packed), Ok(0x0000_ffff));
    }

    #[test]
    fn bad_input_rejected() {
        for bad in ["", "#12345", "1234567", "#gg0000", "0x", "pink", "+12345", "#-12345"] {
            assert!(parse_color(bad).is_err(), "{bad} should be rejected");
        }
    }
}
