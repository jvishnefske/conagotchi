//! Regression tests for the HOME screen QR code (FR32): the rendered frame
//! decodes to the project URL, the white QR box lies inside the visible
//! circle, modules map to 4x4 pixel blocks, and HOME is static over time.

extern crate std;

use std::{vec, vec::Vec as StdVec};

use core::convert::Infallible;
use embedded_graphics::{Pixel, pixelcolor::raw::RawU16};

use super::*;
use crate::app::BoardInfo;
use crate::io::{ButtonMask, Input, Millis};

const SIZE: usize = 240;
/// Firmware string as the shell builds it (crate name and version).
const FIRMWARE: &str = "conagotchi-fw 0.1.0";
/// Background colour the test framebuffer starts with, so untouched pixels
/// are distinguishable from drawn ones.
const UNTOUCHED: u16 = 0xAAAA;

/// Test-only full-size RGB565 framebuffer.
struct Fb {
    px: StdVec<u16>,
}

impl Fb {
    fn new() -> Self {
        Self {
            px: vec![UNTOUCHED; SIZE * SIZE],
        }
    }

    fn get(&self, x: usize, y: usize) -> u16 {
        self.px[y * SIZE + x]
    }

    /// Approximate 8-bit luma of pixel (`x`, `y`).
    fn luma(&self, x: usize, y: usize) -> u8 {
        let c = u32::from(self.get(x, y));
        let r = (c >> 11 & 0x1F) * 255 / 31;
        let g = (c >> 5 & 0x3F) * 255 / 63;
        let b = (c & 0x1F) * 255 / 31;
        u8::try_from((299 * r + 587 * g + 114 * b) / 1000).unwrap_or(u8::MAX)
    }
}

impl OriginDimensions for Fb {
    fn size(&self) -> Size {
        Size::new(240, 240)
    }
}

impl DrawTarget for Fb {
    type Color = Rgb565;
    type Error = Infallible;

    fn draw_iter<I: IntoIterator<Item = Pixel<Rgb565>>>(
        &mut self,
        pixels: I,
    ) -> Result<(), Infallible> {
        for Pixel(p, c) in pixels {
            if let (Ok(x @ 0..240), Ok(y @ 0..240)) = (usize::try_from(p.x), usize::try_from(p.y)) {
                self.px[y * SIZE + x] = RawU16::from(c).into_inner();
            }
        }
        Ok(())
    }
}

fn raw(c: Color) -> u16 {
    RawU16::from(to_rgb565(c)).into_inner()
}

/// A booted app switched to HOME at `t` ms.
fn home_at(t: u64) -> App {
    let app = App::new(BoardInfo {
        mac: [0xde, 0xad, 0xbe, 0xef, 0x00, 0x01],
        firmware: FIRMWARE,
    });
    let app = app
        .handle(Input::Tick {
            now: Millis(t),
            buttons: ButtonMask(0),
        })
        .0;
    let app = app
        .handle(Input::Host(badge_proto::HostMsg {
            seq: 1,
            cmd: Some(badge_proto::HostMsg_::Cmd::ShowScreen(
                badge_proto::ShowScreen {
                    screen: Screen::Home,
                },
            )),
        }))
        .0;
    assert_eq!(app.screen(), Screen::Home);
    app
}

fn frame(app: &App) -> Fb {
    let mut fb = Fb::new();
    render(app, &mut fb).expect("infallible");
    fb
}

/// Top-left corner of the QR box in framebuffer coordinates.
fn box_origin() -> (usize, usize) {
    let tl = QR_BOX.top_left;
    (
        usize::try_from(tl.x).expect("x"),
        usize::try_from(tl.y).expect("y"),
    )
}

/// Decodes every QR grid found in the frame.
fn decode(fb: &Fb) -> StdVec<std::string::String> {
    let mut img = rqrr::PreparedImage::prepare_from_greyscale(SIZE, SIZE, |x, y| fb.luma(x, y));
    img.detect_grids()
        .into_iter()
        .map(|g| g.decode().expect("QR decodes").1)
        .collect()
}

#[test]
fn fr32_home_qr_decodes_to_project_url() {
    let fb = frame(&home_at(1_000));
    assert_eq!(
        decode(&fb),
        vec![std::string::String::from(
            "https://github.com/jvishnefske/conagotchi"
        )]
    );
}

#[test]
fn fr32_home_qr_box_is_inside_circle_and_clear_of_dots() {
    let (x0, y0) = box_origin();
    let side = QR_BOX_SIDE;
    assert_eq!(side, 132);
    for y in y0..y0 + side {
        for x in x0..x0 + side {
            // Pixel centre distance from panel centre, in half-pixels.
            let dx = 2 * x as i64 + 1 - 240;
            let dy = 2 * y as i64 + 1 - 240;
            assert!(
                dx * dx + dy * dy <= 240 * 240,
                "QR pixel ({x},{y}) outside circle"
            );
        }
    }
    assert!(y0 as i32 > DOTS_Y + 3, "QR box overlaps the page dots");
    assert_eq!(x0 + side / 2, 120, "QR box is not centred");
}

#[test]
fn fr32_home_qr_modules_are_4px_blocks_with_white_quiet_zone() {
    let fb = frame(&home_at(1_000));
    let (x0, y0) = box_origin();
    let (black, white) = (raw(Color::BLACK), raw(Color::WHITE));
    for y in 0..QR_BOX_SIDE {
        for x in 0..QR_BOX_SIDE {
            let (mx, my) = (x / QR_SCALE, y / QR_SCALE);
            let dark = mx >= QR_QUIET
                && my >= QR_QUIET
                && qr::module(mx - QR_QUIET, my - QR_QUIET);
            let want = if dark { black } else { white };
            assert_eq!(fb.get(x0 + x, y0 + y), want, "QR box pixel ({x},{y})");
        }
    }
}

#[test]
fn fr32_home_shows_firmware_version_under_qr() {
    let fb = frame(&home_at(1_000));
    let (_, y0) = box_origin();
    let top = y0 + QR_BOX_SIDE;
    let text = raw(TEXT);
    let lit = (top..top + 14)
        .flat_map(|y| (0..SIZE).map(move |x| (x, y)))
        .filter(|&(x, y)| fb.get(x, y) == text)
        .count();
    assert!(lit > 50, "only {lit} version text pixels under the QR box");
}

#[test]
fn fr32_home_frame_is_static_over_time() {
    // No uptime on HOME: frames seconds apart are identical.
    let a = frame(&home_at(1_000));
    let b = frame(&home_at(3_725_000));
    assert!(a.px == b.px, "HOME frame changed over time");
}
