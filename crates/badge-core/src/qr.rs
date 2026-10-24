//! The project URL as a QR code, for the HOME screen.
//!
//! The bitmap is generated at build time by `build.rs` (host-side `qrcode`
//! crate, error-correction level M) and stored as one `u32` per module row in
//! [`QR`]; [`QR_SIZE`] is the side length in modules, without the quiet zone.
//! [`module`] is the only accessor the renderer needs.

include!(concat!(env!("OUT_DIR"), "/qr_data.rs"));

/// True if module (`x`, `y`) is dark. Coordinates outside the symbol
/// (including the quiet zone) are light.
pub fn module(x: usize, y: usize) -> bool {
    if x >= QR_SIZE {
        return false;
    }
    QR.get(y).is_some_and(|row| row >> x & 1 != 0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fr32_symbol_is_version_3_29_modules() {
        assert_eq!(QR_VERSION, 3);
        assert_eq!(QR_SIZE, 29);
        assert_eq!(QR_URL, "https://github.com/jvishnefske/conagotchi");
    }

    #[test]
    fn fr32_finder_patterns_and_bounds() {
        // Top-left finder: dark 7x7 ring, light ring inside, dark 3x3 core.
        for i in 0..7 {
            assert!(module(i, 0) && module(i, 6) && module(0, i) && module(6, i));
        }
        assert!(!module(1, 1) && !module(5, 5));
        assert!(module(3, 3));
        // Separator next to the top-right finder is light.
        assert!(!module(QR_SIZE - 8, 0));
        // Outside the symbol is light.
        assert!(!module(QR_SIZE, 0));
        assert!(!module(0, QR_SIZE));
        assert!(!module(usize::MAX, usize::MAX));
    }
}
