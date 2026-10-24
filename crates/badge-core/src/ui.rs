//! Renders the app model onto any `embedded-graphics` draw target.
//!
//! The panel is a 240x240 GC9A01 behind a round window, so every screen keeps
//! its content inside the inscribed circle: each text row is truncated to the
//! chord width at its height (minus a margin), and all shapes are placed well
//! inside the circle. [`render`] always draws a full frame: background, title,
//! page dots, then the content of the current [`Screen`].
//!
//! The PET screen's cat lives in [`PET_BOX`]. [`render_animation`] repaints
//! only that box, streaming each of its pixels exactly once (transparent
//! sprite pixels get the background colour), so the panel never shows a
//! cleared box between frames. [`render`] draws the cat through the same path.
//!
//! HOME shows the project URL QR code ([`crate::qr`]) at 4 px per module with
//! a 2-module white quiet zone in [`QR_BOX`], and the firmware version under
//! it. The GPIO screen is compiled only with the `dev` feature.
//!
//! Host `DisplayFill` / `DisplayRect` primitives are not drawn here; the shell
//! paints those directly (see `io::DrawOp`).

use core::fmt::Write;

use badge_proto::{Button, ErrorCode, LedPattern, PetMode, Screen};
use embedded_graphics::{
    mono_font::{
        MonoFont, MonoTextStyle,
        ascii::{FONT_6X10, FONT_10X20},
    },
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, PrimitiveStyle, Rectangle},
    text::{Alignment, Baseline, Text, TextStyleBuilder},
};
use heapless::String;

use crate::app::{App, SCREEN_ORDER, ScanState};
use crate::board::led::RGB_COUNT;
use crate::buttons::BUTTONS;
use crate::color::Color;
use crate::pet::{self, BOUNCE_MAX, DRIFT_MAX, SCALE, SPRITE};
use crate::qr::{self, QR_SIZE};

/// Horizontal / vertical centre of the panel.
const CENTER: i32 = 120;
/// Radius of the visible circle.
const RADIUS: i32 = 120;
/// Pixels kept clear between text and the edge of the circle.
const MARGIN: i32 = 6;
/// Top of the title row.
const TITLE_Y: i32 = 28;
/// Centre row of the page dots.
const DOTS_Y: i32 = 60;
/// Top of the first content row.
const CONTENT_Y: i32 = 72;
/// Height of a content row (FONT_6X10 plus spacing).
const ROW_H: i32 = 13;
/// Number of content rows between the dots and the hint row.
const ROWS: usize = 10;
/// Top of the hint row at the bottom of the circle.
const HINT_Y: i32 = 202;
/// Most list entries (APs, BLE devices) shown on one screen.
const MAX_LIST: usize = 6;

/// Side of the unshifted sprite on screen, in pixels.
const SPRITE_PX: i32 = SPRITE as i32 * SCALE as i32;
/// Width of [`PET_BOX`]: the sprite plus the walk drift on both sides.
const PET_BOX_W: i32 = SPRITE_PX + 2 * DRIFT_MAX as i32;
/// Height of [`PET_BOX`]: the sprite plus the play bounce above it.
const PET_BOX_H: i32 = SPRITE_PX + BOUNCE_MAX as i32;
/// Top of [`PET_BOX`], just below the page dots.
const PET_BOX_Y: i32 = 64;
/// Region repainted by [`render_animation`]: 160x140 at (40, 64), centred
/// horizontally and entirely inside the visible circle. At offset (0, 0) the
/// sprite sits at its bottom centre.
pub const PET_BOX: Rectangle = Rectangle::new(
    Point::new(CENTER - PET_BOX_W / 2, PET_BOX_Y),
    Size::new(PET_BOX_W as u32, PET_BOX_H as u32),
);
/// Top of the pet mode label, under [`PET_BOX`].
const PET_LABEL_Y: i32 = PET_BOX_Y + PET_BOX_H + 2;
// The box starts below the page dots; the label ends inside the circle.
const _: () = assert!(PET_BOX_Y > DOTS_Y + 3);
const _: () = assert!(PET_LABEL_Y + 10 < 2 * RADIUS - MARGIN);

/// Pixels per QR module on screen.
const QR_SCALE: usize = 4;
/// Light modules of quiet zone around the QR symbol on each side.
const QR_QUIET: usize = 2;
/// Side of [`QR_BOX`]: (29 + 2 * 2) modules * 4 px = 132 px.
const QR_BOX_SIDE: usize = (QR_SIZE + 2 * QR_QUIET) * QR_SCALE;
/// Top of [`QR_BOX`], just below the page dots.
const QR_BOX_Y: i32 = 66;
/// The white HOME QR box (symbol plus quiet zone): 132x132 at (54, 66),
/// centred horizontally and entirely inside the visible circle.
pub const QR_BOX: Rectangle = Rectangle::new(
    Point::new(CENTER - QR_BOX_SIDE as i32 / 2, QR_BOX_Y),
    Size::new(QR_BOX_SIDE as u32, QR_BOX_SIDE as u32),
);
/// Top of the firmware version line under [`QR_BOX`].
const QR_TEXT_Y: i32 = QR_BOX_Y + QR_BOX_SIDE as i32 + 4;
// The box clears the page dots, its farthest (bottom) corners lie inside the
// circle, and the version line ends inside the circle.
const _: () = assert!(QR_BOX_Y > DOTS_Y + 3);
const _: () = {
    let half = QR_BOX_SIDE as i32 / 2;
    let dy = QR_BOX_Y + QR_BOX_SIDE as i32 - CENTER;
    assert!(half * half + dy * dy < RADIUS * RADIUS);
};
const _: () = assert!(QR_TEXT_Y + 10 < 2 * RADIUS - MARGIN);

const BG: Color = Color::BLACK;
const TEXT: Color = Color::WHITE;
const DIM: Color = Color::new(128, 128, 128);
const ACCENT: Color = Color::new(255, 160, 0);

/// Converts a 24-bit colour to the panel's RGB565.
pub fn to_rgb565(c: Color) -> Rgb565 {
    Rgb565::new(c.r >> 3, c.g >> 2, c.b >> 3)
}

/// Title shown for each screen.
pub fn screen_name(screen: Screen) -> &'static str {
    match screen {
        Screen::Home => "HOME",
        Screen::Buttons => "BUTTONS",
        Screen::Leds => "LEDS",
        Screen::Wifi => "WI-FI",
        Screen::Ble => "BLE",
        Screen::Gpio => "GPIO",
        Screen::Host => "HOST",
        Screen::Pet => "PET",
        Screen::Trade => "TRADE",
        Screen::Contacts => "CONTACTS",
        _ => "?",
    }
}

/// Display name of a pet mode (the protocol name).
pub fn pet_mode_name(mode: PetMode) -> &'static str {
    match mode {
        PetMode::Auto => "AUTO",
        PetMode::Idle => "IDLE",
        PetMode::Walk => "WALK",
        PetMode::Eat => "EAT",
        PetMode::Play => "PLAY",
        PetMode::Sleep => "SLEEP",
        _ => "?",
    }
}

/// What the cat is doing in `mode`, for the label under the sprite.
fn pet_activity(mode: PetMode) -> &'static str {
    match mode {
        PetMode::Walk => "walking",
        PetMode::Eat => "eating",
        PetMode::Play => "playing",
        PetMode::Sleep => "sleeping",
        _ => "sitting",
    }
}

/// Display name of an LED pattern.
pub fn pattern_name(pattern: LedPattern) -> &'static str {
    match pattern {
        LedPattern::Off => "OFF",
        LedPattern::Manual => "MANUAL",
        LedPattern::Rainbow => "RAINBOW",
        LedPattern::Breathe => "BREATHE",
        _ => "?",
    }
}

/// Display name of a button.
pub fn button_name(button: Button) -> &'static str {
    match button {
        Button::Boot => "BOOT",
        Button::Left => "LEFT",
        Button::Select => "SELECT",
        Button::Start => "START",
        Button::Right => "RIGHT",
        _ => "?",
    }
}

/// Short display name of an error code.
pub fn error_name(code: ErrorCode) -> &'static str {
    match code {
        ErrorCode::None => "NONE",
        ErrorCode::Decode => "DECODE",
        ErrorCode::BadArgument => "BAD ARG",
        ErrorCode::PinNotAllowed => "PIN NOT ALLOWED",
        ErrorCode::Busy => "BUSY",
        ErrorCode::Radio => "RADIO",
        ErrorCode::Unsupported => "UNSUPPORTED",
        _ => "ERROR",
    }
}

/// Half-width of the visible circle on pixel row `y`, 0 outside it.
fn chord_half(y: i32) -> i32 {
    let dy = (y - CENTER).abs();
    if dy >= RADIUS {
        return 0;
    }
    let target = RADIUS * RADIUS - dy * dy;
    // Integer square root by binary search over 0..=RADIUS (8 steps).
    let (mut lo, mut hi) = (0, RADIUS);
    while lo < hi {
        let mid = (lo + hi + 1) / 2;
        if mid * mid <= target {
            lo = mid
        } else {
            hi = mid - 1
        }
    }
    lo
}

/// Characters of `font` that fit centred on a row spanning `top..top+height`.
fn fit_chars(top: i32, font: &MonoFont<'_>) -> usize {
    let height = i32::try_from(font.character_size.height).unwrap_or(RADIUS);
    let width = i32::try_from(font.character_size.width)
        .unwrap_or(RADIUS)
        .max(1);
    let half = chord_half(top).min(chord_half(top + height)) - MARGIN;
    usize::try_from(2 * half / width).unwrap_or(0)
}

/// Splits `s` after at most `n` characters.
fn split_chars(s: &str, n: usize) -> (&str, &str) {
    let at = s.char_indices().nth(n).map_or(s.len(), |(i, _)| i);
    (s.get(..at).unwrap_or(""), s.get(at..).unwrap_or(""))
}

/// A row of text, truncated to fit and with control characters replaced.
fn row_text(s: &str, max: usize) -> String<64> {
    let mut out = String::new();
    for c in split_chars(s, max).0.chars() {
        let c = if c.is_control() { '?' } else { c };
        if out.push(c).is_err() {
            break;
        }
    }
    out
}

/// Formats into a fixed buffer; output that does not fit is cut off.
fn format<const N: usize>(args: core::fmt::Arguments<'_>) -> String<N> {
    let mut s = String::new();
    s.write_fmt(args).ok();
    s
}

/// Draws `s` centred horizontally with its top at `top`, cut to fit the circle.
fn text_at<D>(
    target: &mut D,
    s: &str,
    top: i32,
    font: &MonoFont<'_>,
    color: Color,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let line = row_text(s, fit_chars(top, font));
    let style = MonoTextStyle::new(font, to_rgb565(color));
    let layout = TextStyleBuilder::new()
        .alignment(Alignment::Center)
        .baseline(Baseline::Top)
        .build();
    Text::with_text_style(&line, Point::new(CENTER, top), style, layout).draw(target)?;
    Ok(())
}

/// Top pixel row of content row `row`.
fn row_y(row: usize) -> i32 {
    CONTENT_Y + ROW_H * i32::try_from(row).unwrap_or(0)
}

/// Draws `s` on content row `row` in the small font.
fn row<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    row: usize,
    s: &str,
    color: Color,
) -> Result<(), D::Error> {
    text_at(target, s, row_y(row), &FONT_6X10, color)
}

/// Draws the bottom hint line.
fn hint<D: DrawTarget<Color = Rgb565>>(target: &mut D, s: &str) -> Result<(), D::Error> {
    text_at(target, s, HINT_Y, &FONT_6X10, DIM)
}

/// Draws one dot per screen in [`SCREEN_ORDER`], the current one filled.
fn page_dots<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    current: Screen,
    fg: Color,
) -> Result<(), D::Error> {
    const SPACING: i32 = 12;
    const DIAMETER: u32 = 6;
    let count = SCREEN_ORDER.len() as i32;
    for (i, &screen) in (0i32..).zip(SCREEN_ORDER.iter()) {
        let x = CENTER + (i - count / 2) * SPACING - 3;
        let style = if screen == current {
            PrimitiveStyle::with_fill(to_rgb565(ACCENT))
        } else {
            PrimitiveStyle::with_stroke(to_rgb565(fg), 1)
        };
        Circle::new(Point::new(x, DOTS_Y - 3), DIAMETER)
            .into_styled(style)
            .draw(target)?;
    }
    Ok(())
}

/// Renders a full frame of the current screen.
pub fn render<D>(app: &App, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let screen = app.screen();
    let (bg, fg) = match screen {
        Screen::Host => (app.host_text().bg(), app.host_text().fg()),
        _ => (BG, TEXT),
    };
    target.clear(to_rgb565(bg))?;
    let title_color = if screen == Screen::Host { fg } else { ACCENT };
    text_at(
        target,
        screen_name(screen),
        TITLE_Y,
        &FONT_10X20,
        title_color,
    )?;
    page_dots(
        target,
        screen,
        if screen == Screen::Host { fg } else { DIM },
    )?;
    match screen {
        Screen::Buttons => buttons_screen(app, target),
        Screen::Leds => leds_screen(app, target),
        Screen::Wifi => wifi_screen(app, target),
        Screen::Ble => ble_screen(app, target),
        #[cfg(feature = "dev")]
        Screen::Gpio => gpio_screen(app, target),
        Screen::Host => host_screen(app, target),
        Screen::Pet => pet_screen(app, target),
        Screen::Trade => trade::trade_screen(app, target),
        Screen::Contacts => trade::contacts_screen(app, target),
        _ => home_screen(app, target),
    }
}

/// Repaints only [`PET_BOX`] with the current pet frame, every box pixel
/// exactly once via `fill_contiguous`. A no-op on every screen but PET.
pub fn render_animation<D>(app: &App, target: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    if app.screen() != Screen::Pet {
        return Ok(());
    }
    let pet = app.pet();
    let (mode, frame, flipped) = (pet.active(), pet.frame(), pet.flipped());
    let (dx, dy) = pet.offset();
    // Sprite top-left inside the box; the offsets are bounded by the margins.
    let left = i32::from(DRIFT_MAX) + i32::from(dx);
    let top = i32::from(BOUNCE_MAX) + i32::from(dy);
    let scale = i32::from(SCALE);
    let last = i32::from(SPRITE) - 1;
    let bg = to_rgb565(BG);
    let colors = (0..PET_BOX_H).flat_map(move |y| {
        (0..PET_BOX_W).map(move |x| {
            let (sx, sy) = (x - left, y - top);
            if !(0..SPRITE_PX).contains(&sx) || !(0..SPRITE_PX).contains(&sy) {
                return bg;
            }
            let lx = if flipped {
                last - sx / scale
            } else {
                sx / scale
            };
            match (u8::try_from(lx), u8::try_from(sy / scale)) {
                (Ok(lx), Ok(ly)) => pet::pixel(mode, frame, lx, ly).map_or(bg, to_rgb565),
                _ => bg,
            }
        })
    });
    target.fill_contiguous(&PET_BOX, colors)
}

fn pet_screen<D: DrawTarget<Color = Rgb565>>(app: &App, target: &mut D) -> Result<(), D::Error> {
    let pet = app.pet();
    let activity = pet_activity(pet.active());
    let label: String<32> = if pet.selected() == PetMode::Auto {
        format(format_args!("AUTO - {activity}"))
    } else {
        format(format_args!("{activity}"))
    };
    text_at(target, &label, PET_LABEL_Y, &FONT_6X10, DIM)?;
    render_animation(app, target)
}

/// HOME: the project URL QR code in [`QR_BOX`], the firmware version below.
///
/// Nothing here changes over time, so HOME redraws only when the app model
/// asks for it (no per-second uptime).
fn home_screen<D: DrawTarget<Color = Rgb565>>(app: &App, target: &mut D) -> Result<(), D::Error> {
    render_qr(target)?;
    text_at(target, app.info().firmware, QR_TEXT_Y, &FONT_6X10, TEXT)
}

/// Paints [`QR_BOX`]: dark modules black, quiet zone and light modules white,
/// every box pixel exactly once via `fill_contiguous`.
fn render_qr<D: DrawTarget<Color = Rgb565>>(target: &mut D) -> Result<(), D::Error> {
    let (dark, light) = (to_rgb565(Color::BLACK), to_rgb565(Color::WHITE));
    let side = QR_BOX_SIDE;
    let colors = (0..side).flat_map(move |y| {
        (0..side).map(move |x| {
            let mx = (x / QR_SCALE).checked_sub(QR_QUIET);
            let my = (y / QR_SCALE).checked_sub(QR_QUIET);
            match (mx, my) {
                (Some(mx), Some(my)) if qr::module(mx, my) => dark,
                _ => light,
            }
        })
    });
    target.fill_contiguous(&QR_BOX, colors)
}

fn buttons_screen<D: DrawTarget<Color = Rgb565>>(
    app: &App,
    target: &mut D,
) -> Result<(), D::Error> {
    const SPACING: i32 = 22;
    let pressed = app.pressed();
    let counts = app.press_counts();
    for (i, button) in (0i32..).zip(BUTTONS.iter()) {
        let top = CONTENT_Y + 6 + i * SPACING;
        let index = usize::try_from(button.0).unwrap_or(0);
        let held = u32::try_from(button.0)
            .ok()
            .and_then(|b| 1u8.checked_shl(b))
            .is_some_and(|bit| pressed.0 & bit != 0);
        let count = counts.get(index).copied().unwrap_or(0);
        let label: String<24> = format(format_args!("{:<7}{:>6}", button_name(*button), count));
        let color = if held {
            Rectangle::new(Point::new(CENTER - 60, top - 4), Size::new(120, 18))
                .into_styled(PrimitiveStyle::with_fill(to_rgb565(ACCENT)))
                .draw(target)?;
            BG
        } else {
            TEXT
        };
        text_at(target, &label, top, &FONT_6X10, color)?;
    }
    hint(target, "press any button")
}

/// Undoes brightness scaling so the swatches show the colour, not its dimness.
fn unscale(c: Color, brightness: u8) -> Color {
    let b = u16::from(brightness) + 1;
    let s = |v: u8| u8::try_from((u16::from(v) << 8) / b).unwrap_or(u8::MAX);
    Color::new(s(c.r), s(c.g), s(c.b))
}

fn leds_screen<D: DrawTarget<Color = Rgb565>>(app: &App, target: &mut D) -> Result<(), D::Error> {
    // Swatches sit on a ring around this point.
    const RING_CENTER: Point = Point::new(CENTER, 146);
    /// Swatch offsets for 8 LEDs clockwise from the top, radius ~36 px.
    const OFFSETS: [(i32, i32); RGB_COUNT] = [
        (0, -36),
        (25, -25),
        (36, 0),
        (25, 25),
        (0, 36),
        (-25, 25),
        (-36, 0),
        (-25, -25),
    ];
    const DIAMETER: u32 = 18;
    let pattern: String<32> = format(format_args!("pattern {}", pattern_name(app.pattern())));
    row(target, 0, &pattern, TEXT)?;
    let brightness: String<32> = format(format_args!("brightness {}", app.brightness()));
    row(target, 1, &brightness, DIM)?;
    for (&(dx, dy), &c) in OFFSETS.iter().zip(app.leds().iter()) {
        let top_left = RING_CENTER + Point::new(dx - 9, dy - 9);
        let shown = to_rgb565(unscale(c, app.brightness()));
        let circle = Circle::new(top_left, DIAMETER);
        circle
            .into_styled(PrimitiveStyle::with_fill(shown))
            .draw(target)?;
        circle
            .into_styled(PrimitiveStyle::with_stroke(to_rgb565(DIM), 1))
            .draw(target)?;
    }
    hint(target, "SELECT: next pattern")
}

/// One status line describing a scan.
fn scan_status(state: ScanState, found: usize, noun: &str) -> String<40> {
    match state {
        ScanState::Idle => format(format_args!("SELECT to scan")),
        ScanState::Scanning => format(format_args!("scanning...")),
        ScanState::Done => format(format_args!("{found} {noun}")),
        ScanState::Failed(code) => format(format_args!("scan failed: {}", error_name(code))),
    }
}

fn wifi_screen<D: DrawTarget<Color = Rgb565>>(app: &App, target: &mut D) -> Result<(), D::Error> {
    let aps = app.access_points();
    row(
        target,
        0,
        &scan_status(app.wifi_scan_state(), aps.len(), "networks"),
        TEXT,
    )?;
    let link: String<64> = match app.wifi_link() {
        Ok(state) if state.connected => match state.ipv4.as_slice() {
            [a, b, c, d] => format(format_args!("{} {a}.{b}.{c}.{d}", state.ssid)),
            _ => format(format_args!("{} (no IP)", state.ssid)),
        },
        Ok(_) => format(format_args!("link down")),
        Err(code) => format(format_args!("link error: {}", error_name(code))),
    };
    row(target, 1, &link, DIM)?;
    for (i, ap) in aps.iter().take(MAX_LIST).enumerate() {
        let name = if ap.ssid.is_empty() {
            "<hidden>"
        } else {
            ap.ssid.as_str()
        };
        let line: String<48> = format(format_args!("{:>4} {}", ap.rssi, name));
        row(target, i + 3, &line, TEXT)?;
    }
    hint(target, "SELECT: scan")
}

/// First content row of the BLE device list.
const BLE_LIST_ROW: usize = 6;
/// BLE devices shown under the metrics (rows 6..=9).
const BLE_LIST: usize = ROWS - BLE_LIST_ROW;

/// The BLE screen's status lines (rows 1..=4): the advertised name, then
/// the air metrics from the last `Input::BleStats` and the trade lobby size.
fn ble_lines(app: &App) -> [String<40>; 4] {
    let m = app.ble_metrics();
    let adv = match app.ble_advertising() {
        "" => format(format_args!("not advertising")),
        name => format(format_args!("adv: {name}")),
    };
    let strongest = match m.strongest_rssi {
        0 => format(format_args!("strongest --")),
        rssi => format(format_args!("strongest {rssi} dBm")),
    };
    [
        adv,
        format(format_args!("adv/s {} uniq/10s {}", m.adv_per_s, m.unique_10s)),
        format(format_args!("OZ badges {} lobby {}", m.oz_badges, app.trade().lobby_len())),
        strongest,
    ]
}

fn ble_screen<D: DrawTarget<Color = Rgb565>>(app: &App, target: &mut D) -> Result<(), D::Error> {
    let devices = app.ble_devices();
    row(
        target,
        0,
        &scan_status(app.ble_scan_state(), devices.len(), "devices"),
        TEXT,
    )?;
    for (i, line) in ble_lines(app).iter().enumerate() {
        row(target, i + 1, line, if i == 0 { DIM } else { TEXT })?;
    }
    for (i, dev) in devices.iter().take(BLE_LIST).enumerate() {
        let line: String<48> = if dev.name.is_empty() {
            // Address bytes arrive little-endian; show them most significant first.
            let mut addr: String<24> = String::new();
            for (n, b) in dev.addr.iter().rev().enumerate() {
                let sep = if n == 0 { "" } else { ":" };
                write!(addr, "{sep}{b:02X}").ok();
            }
            format(format_args!("{:>4} {}", dev.rssi, addr))
        } else {
            format(format_args!("{:>4} {}", dev.rssi, dev.name))
        };
        row(target, i + BLE_LIST_ROW, &line, TEXT)?;
    }
    hint(target, "SELECT: scan")
}

/// GPIO probe screen; only in `dev` builds (elsewhere GPIO is never shown).
#[cfg(feature = "dev")]
fn gpio_screen<D: DrawTarget<Color = Rgb565>>(app: &App, target: &mut D) -> Result<(), D::Error> {
    use crate::app::GpioResult;
    row(target, 0, "probe pins", DIM)?;
    // Pack the pin numbers into as few rows as fit, rows 1..=3.
    let mut pins = crate::board::PROBE_PINS.iter().peekable();
    for r in 1..=3 {
        let max = fit_chars(row_y(r), &FONT_6X10);
        let mut line: String<64> = String::new();
        while let Some(pin) = pins.peek() {
            let item: String<4> = format(format_args!("{}", pin.0));
            let sep = usize::from(!line.is_empty());
            if line.len() + sep + item.len() > max {
                break;
            }
            if sep == 1 {
                line.push(' ').ok();
            }
            line.push_str(&item).ok();
            pins.next();
        }
        row(target, r, &line, TEXT)?;
    }
    let last: String<40> = match app.last_gpio() {
        None => format(format_args!("no access yet")),
        Some(GpioResult {
            pin,
            result: Ok(level),
        }) => format(format_args!(
            "GPIO{pin} = {}",
            if level { "HIGH" } else { "LOW" }
        )),
        Some(GpioResult {
            pin,
            result: Err(code),
        }) => format(format_args!("GPIO{pin}: {}", error_name(code))),
    };
    row(target, 5, "last access", DIM)?;
    row(target, 6, &last, TEXT)?;
    hint(target, "badgectl gpio")
}

fn host_screen<D: DrawTarget<Color = Rgb565>>(app: &App, target: &mut D) -> Result<(), D::Error> {
    let host = app.host_text();
    if host.text().is_empty() {
        return row(target, 3, "waiting for host", host.fg());
    }
    // Word-agnostic wrap: '\n' starts a new row, long lines continue on the next.
    let mut lines = host.text().split('\n');
    let mut current = lines.next();
    for r in 0..ROWS {
        let Some(rest) = current else { break };
        let (head, tail) = split_chars(rest, fit_chars(row_y(r), &FONT_6X10));
        row(target, r, head, host.fg())?;
        current = if tail.is_empty() {
            lines.next()
        } else {
            Some(tail)
        };
    }
    Ok(())
}

mod trade;

#[cfg(test)]
mod tests;

#[cfg(test)]
mod ble_tests;

#[cfg(test)]
mod qr_tests;
