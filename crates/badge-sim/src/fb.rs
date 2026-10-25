//! The simulated panel: a 240 × 240 RGB565 framebuffer.
//!
//! Pixels are stored row-major, one `u16` per pixel, little-endian (the
//! raw RGB565 value `rrrrrggggggbbbbb`), [`FB_BYTES`] bytes in all.
//! [`FrameBuffer`] implements `embedded-graphics`' `DrawTarget`, so
//! `badge_core::ui::render` draws onto it exactly as onto the real panel;
//! [`apply_draw_op`] paints the host canvas primitives (`Effect::Draw`).
//! [`rgb888`] converts one stored pixel back to 8-bit RGB for PNG output.

#![forbid(unsafe_code)]

use badge_core::board::display::{HEIGHT, WIDTH};
use badge_core::io::DrawOp;
use badge_core::ui::to_rgb565;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::pixelcolor::raw::RawU16;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};

/// Panel width in pixels.
pub const FB_WIDTH: usize = WIDTH as usize;
/// Panel height in pixels.
pub const FB_HEIGHT: usize = HEIGHT as usize;
/// Framebuffer size in bytes.
pub const FB_BYTES: usize = FB_WIDTH * FB_HEIGHT * 2;

/// A draw target over framebuffer bytes.
#[derive(Debug)]
pub struct FrameBuffer<'a> {
    bytes: &'a mut [u8; FB_BYTES],
}

impl<'a> FrameBuffer<'a> {
    /// Wraps the framebuffer storage.
    pub fn new(bytes: &'a mut [u8; FB_BYTES]) -> Self {
        Self { bytes }
    }
}

impl OriginDimensions for FrameBuffer<'_> {
    fn size(&self) -> Size {
        Size::new(u32::from(WIDTH), u32::from(HEIGHT))
    }
}

impl DrawTarget for FrameBuffer<'_> {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(p, c) in pixels {
            let (Ok(x), Ok(y)) = (usize::try_from(p.x), usize::try_from(p.y)) else {
                continue;
            };
            if x >= FB_WIDTH || y >= FB_HEIGHT {
                continue;
            }
            let at = 2 * (y * FB_WIDTH + x);
            if let Some(px) = self.bytes.get_mut(at..at + 2) {
                px.copy_from_slice(&RawU16::from(c).into_inner().to_le_bytes());
            }
        }
        Ok(())
    }
}

/// Paints one host canvas primitive (clipped to the panel).
pub fn apply_draw_op<D: DrawTarget<Color = Rgb565>>(op: DrawOp, target: &mut D) -> Result<(), D::Error> {
    match op {
        DrawOp::Fill(c) => target.clear(to_rgb565(c)),
        DrawOp::Rect { x, y, w, h, color } => Rectangle::new(
            Point::new(i32::from(x), i32::from(y)),
            Size::new(u32::from(w), u32::from(h)),
        )
        .into_styled(PrimitiveStyle::with_fill(to_rgb565(color)))
        .draw(target),
    }
}

/// The 8-bit RGB value of pixel (`x`, `y`) in framebuffer bytes; black if
/// outside the buffer. Channels are expanded by bit replication.
pub fn rgb888(bytes: &[u8], x: usize, y: usize) -> [u8; 3] {
    let at = 2 * (y * FB_WIDTH + x);
    let Some(&[lo, hi]) = bytes.get(at..at + 2).and_then(|s| s.first_chunk::<2>()) else {
        return [0; 3];
    };
    let v = u16::from_le_bytes([lo, hi]);
    let (r5, g6, b5) = ((v >> 11) & 0x1F, (v >> 5) & 0x3F, v & 0x1F);
    let expand5 = |c: u16| u8::try_from((c << 3) | (c >> 2)).unwrap_or(u8::MAX);
    let expand6 = |c: u16| u8::try_from((c << 2) | (c >> 4)).unwrap_or(u8::MAX);
    [expand5(r5), expand6(g6), expand5(b5)]
}

#[cfg(test)]
mod tests {
    extern crate std;
    use std::boxed::Box;

    use badge_core::color::Color;

    use super::*;

    fn fb() -> Box<[u8; FB_BYTES]> {
        Box::new([0; FB_BYTES])
    }

    #[test]
    fn fr39_fill_and_rect_paint_clipped_pixels() {
        let mut bytes = fb();
        let mut target = FrameBuffer::new(&mut bytes);
        apply_draw_op(DrawOp::Fill(Color::new(255, 0, 0)), &mut target).unwrap();
        let op = DrawOp::Rect { x: 230, y: 230, w: 50, h: 50, color: Color::new(0, 0, 255) };
        apply_draw_op(op, &mut target).unwrap();
        assert_eq!(rgb888(&bytes[..], 0, 0), [255, 0, 0]);
        assert_eq!(rgb888(&bytes[..], 239, 239), [0, 0, 255]);
        assert_eq!(rgb888(&bytes[..], 229, 239), [255, 0, 0]);
        assert_eq!(rgb888(&bytes[..], 240, 240), [0, 0, 0], "outside");
    }

    #[test]
    fn fr39_ui_renders_onto_the_framebuffer() {
        use badge_core::app::{App, BoardInfo};
        let app = App::new(BoardInfo { mac: [2, 0, 0xC0, 0xFF, 0xEE, 1], firmware: "sim" });
        let mut bytes = fb();
        badge_core::ui::render(&app, &mut FrameBuffer::new(&mut bytes)).unwrap();
        assert!(bytes.iter().any(|&b| b != 0), "HOME screen drew something");
    }
}
