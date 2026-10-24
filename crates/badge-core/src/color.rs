//! 24-bit colour value shared by the LEDs, display and protocol.

/// An 8-bit-per-channel RGB colour.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Color {
    /// Red channel.
    pub r: u8,
    /// Green channel.
    pub g: u8,
    /// Blue channel.
    pub b: u8,
}

impl Color {
    /// Black / LED off.
    pub const BLACK: Self = Self::new(0, 0, 0);
    /// Full white.
    pub const WHITE: Self = Self::new(255, 255, 255);

    /// Creates a colour from its channels.
    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }

    /// Unpacks the protocol's `0x00RRGGBB` representation (upper byte ignored).
    pub const fn from_packed(rgb: u32) -> Self {
        Self::new((rgb >> 16) as u8, (rgb >> 8) as u8, rgb as u8)
    }

    /// Packs into the protocol's `0x00RRGGBB` representation.
    pub const fn packed(self) -> u32 {
        ((self.r as u32) << 16) | ((self.g as u32) << 8) | self.b as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packing_round_trips_and_ignores_top_byte() {
        let c = Color::new(0x12, 0x34, 0x56);
        assert_eq!(c.packed(), 0x0012_3456);
        assert_eq!(Color::from_packed(0xff12_3456), c);
    }
}
