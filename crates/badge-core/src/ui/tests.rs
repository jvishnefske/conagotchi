//! Regression tests for [`render`] on an in-memory 240x240 framebuffer.

extern crate std;

use std::{vec, vec::Vec as StdVec};

use badge_proto::{
    AccessPoint, BleDevice, DisplayText, HostMsg, HostMsg_::Cmd, Rgb, SetLeds, SetPattern,
    ShowScreen, WifiState,
};
use core::convert::Infallible;
use embedded_graphics::{Pixel, pixelcolor::raw::RawU16};

use super::*;
use crate::app::BoardInfo;
use crate::io::{ButtonMask, Input, Millis};

const SIZE: usize = 240;

/// Test-only full-size framebuffer.
struct Fb {
    px: StdVec<u16>,
}

impl Fb {
    fn new() -> Self {
        Self {
            px: vec![0xAAAA; SIZE * SIZE],
        }
    }

    fn get(&self, x: usize, y: usize) -> u16 {
        self.px[y * SIZE + x]
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

fn host(app: App, cmd: Cmd) -> App {
    app.handle(Input::Host(HostMsg {
        seq: 1,
        cmd: Some(cmd),
    }))
    .0
}

fn show(app: App, screen: Screen) -> App {
    host(app, Cmd::ShowScreen(ShowScreen { screen }))
}

/// An app with worst-case content everywhere: long names, full lists.
fn busy_app() -> App {
    let info = BoardInfo {
        mac: [0xde, 0xad, 0xbe, 0xef, 0x00, 0x01],
        firmware: "conagotchi-discovery-fw 0.1.0-x",
    };
    let app = App::new(info)
        .handle(Input::Tick {
            now: Millis(3_725_000),
            buttons: ButtonMask(0),
        })
        .0;
    let mut aps = heapless::Vec::new();
    let mut devices = heapless::Vec::new();
    for i in 0..12 {
        let mut ssid = heapless::String::new();
        ssid.push_str("WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW").ok();
        aps.push(AccessPoint {
            ssid,
            rssi: -100 + i,
            ..Default::default()
        })
        .ok();
        let mut addr = heapless::Vec::new();
        addr.extend_from_slice(&[1, 2, 3, 4, 5, 6]).ok();
        let name = if i % 2 == 0 {
            "MMMMMMMMMMMMMMMMMMMMMMMMMMMMM"
        } else {
            ""
        };
        let mut n = heapless::String::new();
        n.push_str(name).ok();
        devices
            .push(BleDevice {
                addr,
                rssi: -99,
                name: n,
                ..Default::default()
            })
            .ok();
    }
    let app = app
        .handle(Input::WifiScanned {
            seq: None,
            result: Ok(aps),
        })
        .0;
    let app = app
        .handle(Input::BleScanned {
            seq: None,
            result: Ok(devices),
        })
        .0;
    let mut ipv4 = heapless::Vec::new();
    ipv4.extend_from_slice(&[192, 168, 100, 200]).ok();
    let mut ssid = heapless::String::new();
    ssid.push_str("a-very-long-network-name-012345").ok();
    let app = app
        .handle(Input::Wifi {
            seq: None,
            result: Ok(WifiState {
                connected: true,
                ssid,
                ipv4,
                rssi: -3,
            }),
        })
        .0;
    host(
        app,
        Cmd::SetPattern(SetPattern {
            pattern: LedPattern::Rainbow,
            brightness: 255,
        }),
    )
}

/// Renders and returns the frame.
fn frame(app: &App) -> Fb {
    let mut fb = Fb::new();
    render(app, &mut fb).expect("infallible");
    fb
}

/// Asserts nothing but `bg` lies outside the visible circle, and something
/// other than `bg` lies inside it.
fn assert_inside_circle(fb: &Fb, bg: u16, what: &str) {
    let mut lit = 0;
    for y in 0..SIZE {
        for x in 0..SIZE {
            if fb.get(x, y) == bg {
                continue;
            }
            lit += 1;
            // Pixel centre distance from panel centre, in half-pixels.
            let dx = 2 * x as i64 + 1 - 240;
            let dy = 2 * y as i64 + 1 - 240;
            assert!(
                dx * dx + dy * dy <= 240 * 240,
                "{what}: pixel ({x},{y}) outside circle"
            );
        }
    }
    assert!(lit > 100, "{what}: only {lit} pixels drawn");
}

/// Every screen, in page-dot order.
const ALL_SCREENS: [Screen; SCREEN_ORDER.len()] = SCREEN_ORDER;

#[test]
fn rgb565_conversion() {
    assert_eq!(to_rgb565(Color::WHITE), Rgb565::WHITE);
    assert_eq!(to_rgb565(Color::BLACK), Rgb565::BLACK);
    assert_eq!(to_rgb565(Color::new(255, 0, 0)), Rgb565::RED);
    assert_eq!(to_rgb565(Color::new(0, 255, 0)), Rgb565::GREEN);
    assert_eq!(to_rgb565(Color::new(0, 0, 255)), Rgb565::BLUE);
}

#[test]
fn chord_matches_circle() {
    assert_eq!(chord_half(120), 120);
    assert_eq!(chord_half(0), 0);
    assert_eq!(chord_half(240), 0);
    assert_eq!(chord_half(-5), 0);
    assert_eq!(chord_half(48), 96); // 72-96-120 triangle
    for y in 0..240 {
        let h = chord_half(y);
        let dy = y - 120;
        assert!(h * h + dy * dy <= 14_400 && (h + 1) * (h + 1) + dy * dy > 14_400);
    }
}

#[test]
fn every_screen_renders_inside_the_circle() {
    let app = busy_app();
    for screen in ALL_SCREENS {
        let fb = frame(&show(app.clone(), screen));
        assert_inside_circle(&fb, raw(BG), screen_name(screen));
    }
}

#[test]
fn fresh_app_renders_every_screen() {
    let app = App::new(BoardInfo {
        mac: [0; 6],
        firmware: "fw",
    });
    for screen in ALL_SCREENS {
        let fb = frame(&show(app.clone(), screen));
        assert_inside_circle(&fb, raw(BG), screen_name(screen));
    }
}

#[test]
fn every_screen_shows_page_dot_for_itself() {
    let app = busy_app();
    let frames: StdVec<Fb> = ALL_SCREENS
        .iter()
        .map(|s| frame(&show(app.clone(), *s)))
        .collect();
    for (i, fb) in frames.iter().enumerate() {
        // Dot i is filled with the accent colour; check its centre pixel.
        let half = ALL_SCREENS.len() as i32 / 2;
        let x = (120 + (i as i32 - half) * 12) as usize;
        assert_eq!(fb.get(x, 60), raw(ACCENT), "screen {i}");
    }
}

#[test]
fn host_screen_uses_host_colours_and_wraps() {
    let mut text = heapless::String::new();
    text.push_str("1\n2\n3\n4\n5\n").ok();
    for _ in 0..230 {
        text.push('X').ok();
    }
    let bg = Color::new(0, 0, 128);
    let fg = Color::new(255, 255, 0);
    let app = host(
        busy_app(),
        Cmd::DisplayText(DisplayText {
            text,
            fg: Some(Rgb { rgb: fg.packed() }),
            bg: Some(Rgb { rgb: bg.packed() }),
        }),
    );
    let fb = frame(&app);
    assert_eq!(fb.get(0, 0), raw(bg), "background filled");
    assert_inside_circle(&fb, raw(bg), "HOST");
    // The long line wraps all the way down to the last content row.
    let last_row = row_y(ROWS - 1) as usize;
    let lit = (0..SIZE).any(|x| (last_row..last_row + 10).any(|y| fb.get(x, y) == raw(fg)));
    assert!(lit, "wrapped text missing on last row");
}

#[test]
fn control_characters_cannot_escape_their_row() {
    let mut text = heapless::String::new();
    text.push_str("a\tb\rc").ok();
    let app = host(
        busy_app(),
        Cmd::DisplayText(DisplayText {
            text,
            fg: None,
            bg: None,
        }),
    );
    assert_eq!(row_text("a\tb\rc", 10).as_str(), "a?b?c");
    assert_inside_circle(&frame(&app), raw(BG), "HOST control chars");
}

#[test]
fn buttons_screen_highlights_held_buttons() {
    let app = show(busy_app(), Screen::Buttons);
    let idle = frame(&app);
    let t = app.uptime().0;
    let app = app
        .handle(Input::Tick {
            now: Millis(t + 10),
            buttons: ButtonMask(1 << 3),
        })
        .0;
    let app = app
        .handle(Input::Tick {
            now: Millis(t + 40),
            buttons: ButtonMask(1 << 3),
        })
        .0;
    assert_eq!(app.pressed(), ButtonMask(1 << 3));
    let held = frame(&app);
    assert_ne!(idle.px, held.px);
    let accent = held.px.iter().filter(|&&p| p == raw(ACCENT)).count();
    let accent_idle = idle.px.iter().filter(|&&p| p == raw(ACCENT)).count();
    assert!(accent > accent_idle + 1000, "no highlight bar");
}

#[test]
fn leds_screen_shows_unscaled_swatches() {
    let mut colors = heapless::Vec::new();
    colors.push(Rgb { rgb: 0x00ff_0000 }).ok();
    let app = host(
        show(busy_app(), Screen::Leds),
        Cmd::SetLeds(SetLeds { colors }),
    );
    let app = host(
        app,
        Cmd::SetPattern(SetPattern {
            pattern: LedPattern::Manual,
            brightness: 32,
        }),
    );
    assert_eq!(app.leds()[0], Color::new(32, 0, 0));
    let fb = frame(&app);
    // LED 0 is the top swatch of the ring centred at (120, 146).
    assert_eq!(fb.get(120, 110), raw(Color::new(255, 0, 0)));
}

#[test]
fn unscale_inverts_brightness() {
    for b in [0u8, 1, 31, 32, 127, 255] {
        for v in [0u8, 1, 100, 255] {
            let scaled = crate::leds::render(
                LedPattern::Manual,
                &[Color::new(v, v, v); 8],
                b,
                Millis(0),
            )[0];
            let back = unscale(scaled, b);
            assert!(back.r <= v, "b={b} v={v} back={}", back.r);
            assert!(
                u16::from(v) - u16::from(back.r) <= 256 / (u16::from(b) + 1),
                "b={b} v={v}"
            );
        }
    }
}

#[test]
fn names_cover_every_value() {
    for s in 0..crate::app::SCREEN_COUNT {
        assert_ne!(screen_name(Screen(s)), "?");
    }
    for m in 0..6 {
        assert_ne!(pet_mode_name(PetMode(m)), "?");
    }
    for p in 0..4 {
        assert_ne!(pattern_name(LedPattern(p)), "?");
    }
    for b in BUTTONS {
        assert_ne!(button_name(b), "?");
    }
    for e in 0..7 {
        assert_ne!(error_name(ErrorCode(e)), "ERROR");
    }
}

// ------------------------------------------------------------------- pet

/// Test-only framebuffer that counts writes per pixel, including ones off
/// the panel.
struct CountingFb {
    fb: Fb,
    writes: StdVec<u32>,
    off_panel: u32,
}

impl CountingFb {
    fn new() -> Self {
        Self {
            fb: Fb::new(),
            writes: vec![0; SIZE * SIZE],
            off_panel: 0,
        }
    }
}

impl OriginDimensions for CountingFb {
    fn size(&self) -> Size {
        Size::new(240, 240)
    }
}

impl DrawTarget for CountingFb {
    type Color = Rgb565;
    type Error = Infallible;

    fn draw_iter<I: IntoIterator<Item = Pixel<Rgb565>>>(
        &mut self,
        pixels: I,
    ) -> Result<(), Infallible> {
        for Pixel(p, c) in pixels {
            match (usize::try_from(p.x), usize::try_from(p.y)) {
                (Ok(x @ 0..240), Ok(y @ 0..240)) => {
                    self.writes[y * SIZE + x] += 1;
                    self.fb.px[y * SIZE + x] = RawU16::from(c).into_inner();
                }
                _ => self.off_panel += 1,
            }
        }
        Ok(())
    }
}

fn set_pet(app: App, mode: PetMode) -> App {
    host(app, Cmd::SetPetMode(badge_proto::SetPetMode { mode }))
}

fn at(app: App, t: u64) -> App {
    app.handle(Input::Tick {
        now: Millis(t),
        buttons: ButtonMask(0),
    })
    .0
}

fn in_box(x: usize, y: usize) -> bool {
    PET_BOX.contains(Point::new(x as i32, y as i32))
}

#[test]
fn fr23_pet_box_is_inside_circle_and_clear_of_title_dots_and_label() {
    assert_eq!(PET_BOX.top_left, Point::new(40, 64));
    assert_eq!(PET_BOX.size, Size::new(160, 140));
    let br = PET_BOX.bottom_right().expect("non-empty");
    for (x, y) in [(40, 64), (br.x, 64), (40, br.y), (br.x, br.y)] {
        let dx = 2 * x as i64 + 1 - 240;
        let dy = 2 * y as i64 + 1 - 240;
        assert!(dx * dx + dy * dy <= 240 * 240, "corner ({x},{y})");
    }
    assert!(PET_LABEL_Y > br.y, "overlaps label");
    // The sprite fits with every offset the pet can produce.
    assert_eq!(PET_BOX.size.width, 128 + 2 * DRIFT_MAX as u32);
    assert_eq!(PET_BOX.size.height, 128 + BOUNCE_MAX as u32);
}

#[test]
fn fr23_render_animation_writes_each_box_pixel_once() {
    for mode in crate::pet::MODES {
        let mut app = set_pet(busy_app(), mode);
        let start = app.uptime().0;
        // Several frames, covering walk drift both ways and play bounce.
        for k in 0..24u64 {
            app = at(app, start + k * 160);
            let mut fb = CountingFb::new();
            fb.fb.px.fill(raw(BG));
            render_animation(&app, &mut fb).expect("infallible");
            assert_eq!(fb.off_panel, 0);
            for y in 0..SIZE {
                for x in 0..SIZE {
                    let want = u32::from(in_box(x, y));
                    assert_eq!(fb.writes[y * SIZE + x], want, "{mode:?} k={k} ({x},{y})");
                }
            }
            assert_inside_circle(&fb.fb, raw(BG), "pet animation");
        }
    }
}

#[test]
fn fr23_render_animation_is_noop_off_pet() {
    let app = busy_app();
    for screen in ALL_SCREENS.iter().filter(|s| **s != Screen::Pet) {
        let mut fb = CountingFb::new();
        render_animation(&show(app.clone(), *screen), &mut fb).expect("infallible");
        assert!(fb.writes.iter().all(|&n| n == 0), "{screen:?}");
        assert_eq!(fb.off_panel, 0);
    }
}

#[test]
fn fr23_full_render_draws_sprite_through_animation_path() {
    for mode in crate::pet::MODES {
        let app = at(set_pet(busy_app(), mode), 3_725_000 + 500);
        let full = frame(&app);
        assert_inside_circle(&full, raw(BG), "PET");
        let mut anim = Fb::new();
        render_animation(&app, &mut anim).expect("infallible");
        for y in 0..SIZE {
            for x in 0..SIZE {
                if in_box(x, y) {
                    assert_eq!(full.get(x, y), anim.get(x, y), "{mode:?} ({x},{y})");
                }
            }
        }
        // Mode label under the box.
        let label = (0..SIZE).any(|x| {
            (PET_LABEL_Y as usize..PET_LABEL_Y as usize + 10).any(|y| full.get(x, y) == raw(DIM))
        });
        assert!(label, "{mode:?}: no label");
    }
}

#[test]
fn fr23_sprite_pixels_match_pet_art() {
    let app = set_pet(busy_app(), PetMode::Idle);
    let fb = frame(&app);
    let (left, top) = (
        PET_BOX.top_left.x + DRIFT_MAX as i32,
        PET_BOX.top_left.y + BOUNCE_MAX as i32,
    );
    for ly in 0..SPRITE {
        for lx in 0..SPRITE {
            let want = crate::pet::pixel(PetMode::Idle, 0, lx, ly).map_or(raw(BG), raw);
            for (dx, dy) in [(0, 0), (3, 3)] {
                let x = (left + i32::from(lx) * 4 + dx) as usize;
                let y = (top + i32::from(ly) * 4 + dy) as usize;
                assert_eq!(fb.get(x, y), want, "logical ({lx},{ly})");
            }
        }
    }
}

#[test]
fn fr23_walking_left_mirrors_and_shifts_the_sprite() {
    let app = set_pet(busy_app(), PetMode::Walk);
    let start = app.uptime().0;
    let mut seen_flip = false;
    let mut boxes = StdVec::new();
    let mut app = app;
    for k in 0..32u64 {
        app = at(app, start + k * 150);
        let pet = app.pet();
        let mut fb = Fb::new();
        render_animation(&app, &mut fb).expect("infallible");
        let (dx, _) = pet.offset();
        let left = PET_BOX.top_left.x + DRIFT_MAX as i32 + i32::from(dx);
        let top = PET_BOX.top_left.y + BOUNCE_MAX as i32;
        // Spot-check a row of logical pixels against the (mirrored) art.
        for lx in 0..SPRITE {
            let src = if pet.flipped() { SPRITE - 1 - lx } else { lx };
            let want = crate::pet::pixel(PetMode::Walk, pet.frame(), src, 20).map_or(raw(BG), raw);
            let x = (left + i32::from(lx) * 4) as usize;
            assert_eq!(fb.get(x, (top + 80) as usize), want, "k={k} lx={lx}");
        }
        seen_flip |= pet.flipped();
        boxes.push(fb.px);
    }
    assert!(seen_flip);
    boxes.dedup();
    assert!(boxes.len() > 16, "animation barely changes");
}
