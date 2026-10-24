//! Regression tests for the BLE screen's metrics rows (FR-29).

extern crate std;

use std::{vec, vec::Vec as StdVec};

use badge_proto::{BleAdvertise, HostMsg, HostMsg_::Cmd, ShowScreen};
use core::convert::Infallible;
use embedded_graphics::{Pixel, pixelcolor::raw::RawU16};

use super::*;
use crate::app::BoardInfo;
use crate::ble_metrics::BleSnapshot;
use crate::io::{Effect, Input};

/// Test-only 240x240 framebuffer.
struct Fb {
    px: StdVec<u16>,
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
                self.px[y * 240 + x] = RawU16::from(c).into_inner();
            }
        }
        Ok(())
    }
}

fn app() -> App {
    App::new(BoardInfo {
        mac: [0xe8, 0x3d, 0xc1, 0xf6, 0x0a, 0xc8],
        firmware: "fw",
    })
}

fn host(app: App, seq: u32, cmd: Cmd) -> (App, crate::io::Effects) {
    app.handle(Input::Host(HostMsg {
        seq,
        cmd: Some(cmd),
    }))
}

fn on_ble(app: App) -> App {
    host(
        app,
        1,
        Cmd::ShowScreen(ShowScreen {
            screen: Screen::Ble,
        }),
    )
    .0
}

fn stats(app: App, s: BleSnapshot) -> (App, crate::io::Effects) {
    app.handle(Input::BleStats(s))
}

fn lines(app: &App) -> [String<40>; 4] {
    ble_lines(app)
}

#[test]
fn fr29_ble_screen_defaults() {
    let l = lines(&app());
    assert_eq!(l[0].as_str(), "adv: conagotchi");
    assert_eq!(l[1].as_str(), "adv/s 0 uniq/10s 0");
    assert_eq!(l[2].as_str(), "OZ badges 0 lobby 0");
    assert_eq!(l[3].as_str(), "strongest --");
}

#[test]
fn fr29_ble_screen_shows_metrics() {
    let snap = BleSnapshot {
        adv_per_s: 42,
        unique_10s: 17,
        oz_badges: 3,
        strongest_rssi: -41,
    };
    let (app, fx) = stats(on_ble(app()), snap);
    assert!(
        fx.contains(&Effect::Redraw),
        "fresh metrics redraw the BLE screen"
    );
    let l = lines(&app);
    assert_eq!(l[1].as_str(), "adv/s 42 uniq/10s 17");
    assert_eq!(l[2].as_str(), "OZ badges 3 lobby 0");
    assert_eq!(l[3].as_str(), "strongest -41 dBm");
    // Unchanged metrics do not redraw.
    let (_, fx) = stats(app, snap);
    assert!(!fx.contains(&Effect::Redraw));
}

#[test]
fn fr29_ble_screen_shows_advertised_name() {
    let mut name = heapless::String::new();
    name.push_str("conagotchi").ok();
    let (app, _) = host(app(), 7, Cmd::BleAdvertise(BleAdvertise { name }));
    let (app, _) = app.handle(Input::BleAdvertised {
        seq: 7,
        result: Ok(()),
    });
    assert_eq!(lines(&app)[0].as_str(), "adv: conagotchi");
}

#[test]
fn fr29_ble_screen_worst_case_stays_inside_circle() {
    let snap = BleSnapshot {
        adv_per_s: u16::MAX,
        unique_10s: u16::MAX,
        oz_badges: u16::MAX,
        strongest_rssi: i8::MIN,
    };
    let (app, _) = stats(on_ble(app()), snap);
    let mut fb = Fb {
        px: vec![0xAAAA; 240 * 240],
    };
    render(&app, &mut fb).expect("infallible");
    let bg = RawU16::from(to_rgb565(BG)).into_inner();
    let mut drawn = 0usize;
    for y in 0..240i32 {
        for x in 0..240i32 {
            let px = fb.px[(y * 240 + x) as usize];
            let (dx, dy) = (x - CENTER, y - CENTER);
            if dx * dx + dy * dy >= RADIUS * RADIUS {
                assert_eq!(px, bg, "pixel ({x},{y}) outside the circle");
            } else if px != bg {
                drawn += 1;
            }
        }
    }
    assert!(drawn > 0, "the screen drew something");
    assert_eq!(lines(&app)[3].as_str(), "strongest -128 dBm");
}
