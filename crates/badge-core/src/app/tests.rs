//! Regression tests for [`App`]; each test names the FR it covers.

extern crate std;

use std::vec::Vec as StdVec;

use badge_proto::{
    AccessPoint, BleAdvertise, BleDevice, BleScan, DisplayFill, DisplayRect, DisplayText,
    GetButtons, GetInfo, GpioRead, GpioWrite, Ping, Rgb, SetIndicator, SetLed, SetLeds, SetPattern,
    SetPetMode, ShowScreen, WifiConnect, WifiScan, WifiStatus,
};

use super::*;
use crate::probe_layout::{self, MAILBOX_SEQ, mailbox};

const INFO: BoardInfo = BoardInfo {
    mac: [0x24, 0x6f, 0x28, 1, 2, 3],
    firmware: "badge-fw 0.1.0",
};
const LEFT: u8 = 1 << 2;
const SELECT: u8 = 1 << 3;
const RIGHT: u8 = 1 << 5;

/// A freshly booted app that has seen its first tick at t=0.
fn booted() -> App {
    App::new(INFO)
        .handle(Input::Tick {
            now: Millis(0),
            buttons: ButtonMask(0),
        })
        .0
}

/// A booted app switched to the HOME screen (no pet-mode SELECT action).
fn on_home() -> App {
    host(
        booted(),
        1,
        Cmd::ShowScreen(ShowScreen {
            screen: Screen::Home,
        }),
    )
    .0
}

fn tick(app: App, t: u64, mask: u8) -> (App, Effects) {
    app.handle(Input::Tick {
        now: Millis(t),
        buttons: ButtonMask(mask),
    })
}

fn host(app: App, seq: u32, cmd: Cmd) -> (App, Effects) {
    app.handle(Input::Host(HostMsg {
        seq,
        cmd: Some(cmd),
    }))
}

/// Holds `mask` from `t` for 40 ms then releases it for 40 ms; returns all effects.
fn click(app: App, t: u64, mask: u8) -> (App, StdVec<Effect>) {
    let mut all = StdVec::new();
    let mut app = app;
    for (dt, m) in [(0, mask), (30, mask), (40, 0), (70, 0)] {
        let (next, fx) = tick(app, t + dt, m);
        app = next;
        all.extend(fx);
    }
    (app, all)
}

fn responses(fx: &[Effect]) -> StdVec<Response> {
    fx.iter()
        .filter_map(|e| match e {
            Effect::Send(BadgeMsg {
                kind: Some(BadgeMsg_::Kind::Response(r)),
            }) => Some(r.clone()),
            _ => None,
        })
        .collect()
}

fn events(fx: &[Effect]) -> StdVec<Event_::Kind> {
    fx.iter()
        .filter_map(|e| match e {
            Effect::Send(BadgeMsg {
                kind: Some(BadgeMsg_::Kind::Event(Event { kind: Some(k) })),
            }) => Some(k.clone()),
            _ => None,
        })
        .collect()
}

/// Asserts `fx` holds exactly one response, for `seq`, and returns its body.
fn only_response(fx: &[Effect], seq: u32) -> Body {
    let rs = responses(fx);
    assert_eq!(rs.len(), 1, "expected one response in {fx:?}");
    assert_eq!(rs[0].seq, seq);
    rs[0].body.clone().expect("body")
}

fn error_code(body: &Body) -> ErrorCode {
    match body {
        Body::Error(e) => e.code,
        other => panic!("expected error, got {other:?}"),
    }
}

fn rgb(rgb: u32) -> Option<Rgb> {
    Some(Rgb { rgb })
}

fn ap(name: &str, rssi: i32) -> AccessPoint {
    AccessPoint {
        ssid: truncated(name),
        rssi,
        channel: 6,
        ..Default::default()
    }
}

// ------------------------------------------------------------------ boot

#[test]
fn boot_state_and_first_tick() {
    let app = App::new(INFO);
    assert_eq!(app.screen(), Screen::Pet);
    assert_eq!(app.pattern(), LedPattern::Rainbow);
    assert_eq!(app.brightness(), DEFAULT_BRIGHTNESS);
    let (app, fx) = tick(app, 0, 0);
    assert!(fx.iter().any(|e| matches!(e, Effect::Leds(_))));
    assert!(fx.contains(&Effect::Redraw));
    assert_eq!(
        app.leds(),
        leds::render(
            LedPattern::Rainbow,
            &[Color::BLACK; 8],
            32,
            Millis(0)
        )
    );
}

// ------------------------------------------------------------- transport

#[test]
fn fr5_garbage_gets_decode_error_and_counts() {
    let (app, fx) = booted().handle(Input::HostGarbage);
    assert_eq!(error_code(&only_response(&fx, 0)), ErrorCode::Decode);
    assert_eq!(
        app.counters(),
        Counters {
            frames_rx: 1,
            frames_tx: 1,
            decode_errors: 1
        }
    );
}

#[test]
fn fr6_missing_command_is_unsupported() {
    let (_, fx) = booted().handle(Input::Host(HostMsg { seq: 5, cmd: None }));
    assert_eq!(error_code(&only_response(&fx, 5)), ErrorCode::Unsupported);
}

#[test]
fn fr6_every_request_gets_exactly_one_response() {
    let cmds = immediate_cmds();
    for (i, cmd) in cmds.into_iter().enumerate() {
        let seq = 1000 + i as u32;
        let (_, fx) = host(booted(), seq, cmd.clone());
        let rs = responses(&fx);
        assert_eq!(rs.len(), 1, "{cmd:?} -> {fx:?}");
        assert_eq!(rs[0].seq, seq);
    }
    // Deferred commands answer once, when the shell reports back.
    let app = booted();
    #[cfg(feature = "dev")]
    let app = {
        let (app, fx) = host(app, 1, Cmd::GpioRead(GpioRead { pin: 5 }));
        assert!(responses(&fx).is_empty());
        let (app, fx) = app.handle(Input::Gpio {
            seq: 1,
            pin: 5,
            result: Ok(true),
        });
        assert!(matches!(
            only_response(&fx, 1),
            Body::Gpio(GpioLevel {
                pin: 5,
                level: true
            })
        ));
        app
    };
    let (app, fx) = host(app, 2, Cmd::WifiStatus(WifiStatus {}));
    assert_eq!(fx.as_slice(), &[Effect::WifiStatus { seq: 2 }]);
    let (_, fx) = app.handle(Input::Wifi {
        seq: Some(2),
        result: Ok(WifiState::default()),
    });
    assert!(matches!(only_response(&fx, 2), Body::WifiState(_)));
}

#[test]
fn counters_track_rx_and_tx() {
    let app = booted();
    let (app, _) = host(app, 1, Cmd::Ping(Ping { nonce: 1 }));
    let (app, _) = host(
        app,
        2,
        Cmd::ShowScreen(ShowScreen {
            screen: Screen::Buttons,
        }),
    );
    // Ack + ScreenChanged event = 2 sends; Pong = 1.
    assert_eq!(
        app.counters(),
        Counters {
            frames_rx: 2,
            frames_tx: 3,
            decode_errors: 0
        }
    );
}

// ------------------------------------------------------------------ info

#[test]
fn fr7_ping_returns_nonce_and_uptime() {
    let (app, _) = tick(booted(), 1234, 0);
    let (_, fx) = host(app, 9, Cmd::Ping(Ping { nonce: 0xdead_beef }));
    assert_eq!(
        only_response(&fx, 9),
        Body::Pong(Pong {
            nonce: 0xdead_beef,
            uptime_ms: 1234
        })
    );
}

#[test]
fn fr7_get_info_reports_board() {
    let (app, _) = tick(booted(), 500, 0);
    let (_, fx) = host(app, 3, Cmd::GetInfo(GetInfo {}));
    let Body::Info(info) = only_response(&fx, 3) else {
        panic!("not info")
    };
    assert_eq!(info.firmware.as_str(), "badge-fw 0.1.0");
    assert_eq!(info.chip.as_str(), "esp32s3");
    assert_eq!(info.mac.as_slice(), &INFO.mac);
    assert_eq!(info.uptime_ms, 500);
    #[cfg(feature = "dev")]
    let pins: StdVec<u32> = crate::board::PROBE_PINS.iter().map(|g| u32::from(g.0)).collect();
    #[cfg(not(feature = "dev"))]
    let pins: StdVec<u32> = StdVec::new();
    assert_eq!(info.probe_pins.as_slice(), pins.as_slice());
}

#[test]
fn info_firmware_name_is_truncated_not_dropped() {
    let long = BoardInfo {
        mac: [0; 6],
        firmware: "a-firmware-name-that-is-far-too-long-for-32",
    };
    let (_, fx) = host(App::new(long), 1, Cmd::GetInfo(GetInfo {}));
    let Body::Info(info) = only_response(&fx, 1) else {
        panic!("not info")
    };
    assert_eq!(info.firmware.len(), 32);
}

// --------------------------------------------------------------- buttons

#[test]
fn fr8_debounced_press_and_release_emit_events() {
    let app = on_home();
    let (app, fx) = tick(app, 100, SELECT);
    assert!(events(&fx).is_empty());
    let (app, fx) = tick(app, 120, SELECT);
    assert!(events(&fx).is_empty(), "accepted before 30 ms");
    let (app, fx) = tick(app, 130, SELECT);
    assert_eq!(
        events(&fx),
        [Event_::Kind::Button(ButtonEvent {
            button: Button::Select,
            pressed: true,
            at_ms: 130
        })]
    );
    assert_eq!(app.pressed(), ButtonMask(SELECT));
    let (app, _) = tick(app, 140, 0);
    let (app, fx) = tick(app, 170, 0);
    assert_eq!(
        events(&fx),
        [Event_::Kind::Button(ButtonEvent {
            button: Button::Select,
            pressed: false,
            at_ms: 170
        })]
    );
    assert_eq!(app.pressed(), ButtonMask(0));
}

#[test]
fn fr8_bounce_emits_nothing() {
    let mut app = booted();
    for (i, t) in (100..300).step_by(10).enumerate() {
        let (next, fx) = tick(app, t, if i % 2 == 0 { LEFT } else { 0 });
        assert!(events(&fx).is_empty());
        app = next;
    }
    assert_eq!(app.press_counts(), [0; 6]);
}

#[test]
fn fr9_get_buttons_reports_mask_and_counts() {
    let (app, _) = click(booted(), 100, SELECT);
    let (app, _) = click(app, 200, SELECT);
    let (app, _) = click(app, 300, 1 << 4); // START
    let (app, _) = tick(app, 400, SELECT);
    let (app, _) = tick(app, 430, SELECT);
    let (_, fx) = host(app, 4, Cmd::GetButtons(GetButtons {}));
    let Body::Buttons(state) = only_response(&fx, 4) else {
        panic!("not buttons")
    };
    assert_eq!(state.pressed_mask, u32::from(SELECT));
    assert_eq!(state.press_counts.as_slice(), &[0, 0, 0, 3, 1, 0]);
}

// ------------------------------------------------------------------ LEDs

#[test]
fn fr10_set_led_switches_to_manual() {
    let app = booted();
    let (app, _) = host(
        app,
        1,
        Cmd::SetPattern(SetPattern {
            pattern: LedPattern::Off,
            brightness: 255,
        }),
    );
    let (app, fx) = host(
        app,
        2,
        Cmd::SetLed(SetLed {
            index: 3,
            color: rgb(0x00ff_0000),
        }),
    );
    assert_eq!(only_response(&fx, 2), Body::Ack(Ack {}));
    assert_eq!(app.pattern(), LedPattern::Manual);
    let mut expect = [Color::BLACK; 8];
    expect[3] = Color::new(255, 0, 0);
    assert!(fx.contains(&Effect::Leds(expect)));
    assert_eq!(app.leds(), expect);
}

#[test]
fn fr10_set_led_out_of_range_is_bad_argument() {
    for index in [8, 100, u32::MAX] {
        let (app, fx) = host(
            booted(),
            7,
            Cmd::SetLed(SetLed {
                index,
                color: rgb(0xffffff),
            }),
        );
        assert_eq!(error_code(&only_response(&fx, 7)), ErrorCode::BadArgument);
        assert_eq!(
            app.pattern(),
            LedPattern::Rainbow,
            "pattern must not change"
        );
        assert_eq!(fx.len(), 1);
    }
}

#[test]
fn fr10_set_leds_sets_first_n() {
    let app = booted();
    let (app, _) = host(
        app,
        1,
        Cmd::SetPattern(SetPattern {
            pattern: LedPattern::Off,
            brightness: 255,
        }),
    );
    let mut colors = heapless::Vec::new();
    for c in [0x0000_00ff, 0x0000_ff00, 0x00ff_0000] {
        colors.push(Rgb { rgb: c }).ok();
    }
    let (app, fx) = host(app, 2, Cmd::SetLeds(SetLeds { colors }));
    assert_eq!(only_response(&fx, 2), Body::Ack(Ack {}));
    assert_eq!(app.pattern(), LedPattern::Manual);
    let leds = app.leds();
    assert_eq!(
        &leds[..3],
        &[
            Color::new(0, 0, 255),
            Color::new(0, 255, 0),
            Color::new(255, 0, 0)
        ]
    );
    assert!(leds[3..].iter().all(|c| *c == Color::BLACK));
}

#[test]
fn fr11_set_pattern_and_brightness() {
    let (app, fx) = host(
        booted(),
        1,
        Cmd::SetPattern(SetPattern {
            pattern: LedPattern::Rainbow,
            brightness: 200,
        }),
    );
    assert_eq!(only_response(&fx, 1), Body::Ack(Ack {}));
    assert_eq!((app.pattern(), app.brightness()), (LedPattern::Rainbow, 200));
    // 0 keeps the brightness.
    let (app, _) = host(
        app,
        2,
        Cmd::SetPattern(SetPattern {
            pattern: LedPattern::Breathe,
            brightness: 0,
        }),
    );
    assert_eq!(
        (app.pattern(), app.brightness()),
        (LedPattern::Breathe, 200)
    );
    for bad in [
        SetPattern {
            pattern: LedPattern(4),
            brightness: 10,
        },
        SetPattern {
            pattern: LedPattern(5),
            brightness: 10,
        },
        SetPattern {
            pattern: LedPattern(-1),
            brightness: 10,
        },
        SetPattern {
            pattern: LedPattern::Off,
            brightness: 256,
        },
    ] {
        let (next, fx) = host(app.clone(), 3, Cmd::SetPattern(bad));
        assert_eq!(error_code(&only_response(&fx, 3)), ErrorCode::BadArgument);
        assert_eq!(
            (next.pattern(), next.brightness()),
            (LedPattern::Breathe, 200)
        );
    }
}

#[test]
fn fr11_animation_advances_on_ticks_and_static_is_quiet() {
    let (mut app, _) = host(
        booted(),
        1,
        Cmd::SetPattern(SetPattern {
            pattern: LedPattern::Rainbow,
            brightness: 255,
        }),
    );
    let mut writes = 0;
    for t in (20..=1000).step_by(20) {
        let (next, fx) = tick(app, t, 0);
        writes += fx.iter().filter(|e| matches!(e, Effect::Leds(_))).count();
        app = next;
    }
    assert!(writes >= 40, "rainbow animates on most ticks, got {writes}");
    let (mut app, _) = host(
        app,
        2,
        Cmd::SetPattern(SetPattern {
            pattern: LedPattern::Off,
            brightness: 0,
        }),
    );
    for t in (1020..=2000).step_by(20) {
        let (next, fx) = tick(app, t, 0);
        assert!(
            fx.iter().all(|e| !matches!(e, Effect::Leds(_))),
            "static pattern rewrote LEDs"
        );
        app = next;
    }
}

#[test]
fn fr12_set_indicator() {
    let (app, fx) = host(
        booted(),
        1,
        Cmd::SetIndicator(SetIndicator {
            which: Indicator::Alert,
            on: true,
        }),
    );
    assert_eq!(fx[0], Effect::Indicator(Indicator::Alert, true));
    assert_eq!(only_response(&fx, 1), Body::Ack(Ack {}));
    assert!(app.indicators().alert);
    let (app, fx) = host(
        app,
        2,
        Cmd::SetIndicator(SetIndicator {
            which: Indicator::Raffle,
            on: true,
        }),
    );
    assert_eq!(fx[0], Effect::Indicator(Indicator::Raffle, true));
    assert_eq!(
        app.indicators(),
        Indicators {
            alert: true,
            raffle: true
        }
    );
    for which in [Indicator::Unspecified, Indicator(9)] {
        let (_, fx) = host(
            app.clone(),
            3,
            Cmd::SetIndicator(SetIndicator { which, on: true }),
        );
        assert_eq!(fx.len(), 1);
        assert_eq!(error_code(&only_response(&fx, 3)), ErrorCode::BadArgument);
    }
}

// --------------------------------------------------------------- display

#[test]
fn fr13_left_right_cycle_screens_with_wrap() {
    let last = SCREEN_ORDER[SCREEN_ORDER.len() - 1];
    let (app, fx) = click(on_home(), 100, LEFT);
    assert_eq!(app.screen(), last);
    assert!(fx.contains(&Effect::Redraw));
    assert!(events(&fx).contains(&Event_::Kind::Screen(ScreenChanged {
        screen: last
    })));
    let (app, _) = click(app, 200, RIGHT);
    assert_eq!(app.screen(), Screen::Home);
    let mut app = app;
    let order = SCREEN_ORDER[1..].iter().chain([Screen::Home].iter());
    for (i, want) in order.enumerate() {
        app = click(app, 300 + 100 * i as u64, RIGHT).0;
        assert_eq!(app.screen(), *want);
    }
}

#[test]
fn fr13_select_on_leds_cycles_patterns() {
    let (mut app, _) = host(
        booted(),
        1,
        Cmd::ShowScreen(ShowScreen {
            screen: Screen::Leds,
        }),
    );
    // Boots on RAINBOW; SELECT never lands on MANUAL.
    let order = [
        LedPattern::Breathe,
        LedPattern::Off,
        LedPattern::Rainbow,
        LedPattern::Breathe,
    ];
    for (i, want) in order.iter().enumerate() {
        let (next, fx) = click(app, 100 + 100 * i as u64, SELECT);
        assert_eq!(next.pattern(), *want);
        assert!(fx.contains(&Effect::Redraw));
        app = next;
    }
}

#[test]
fn fr13_select_on_wifi_and_ble_scans_once() {
    let (app, _) = host(
        booted(),
        1,
        Cmd::ShowScreen(ShowScreen {
            screen: Screen::Wifi,
        }),
    );
    let (app, fx) = click(app, 100, SELECT);
    assert!(fx.contains(&Effect::WifiScan { seq: None }));
    assert_eq!(app.wifi_scan_state(), ScanState::Scanning);
    let (app, fx) = click(app, 200, SELECT);
    assert!(
        !fx.iter().any(|e| matches!(e, Effect::WifiScan { .. })),
        "busy scan restarted"
    );
    let (app, _) = host(
        app,
        2,
        Cmd::ShowScreen(ShowScreen {
            screen: Screen::Ble,
        }),
    );
    let (app, fx) = click(app, 300, SELECT);
    assert!(fx.contains(&Effect::BleScan {
        seq: None,
        duration_ms: DEFAULT_BLE_SCAN_MS
    }));
    assert_eq!(app.ble_scan_state(), ScanState::Scanning);
    // UI-started scans finish without sending a response.
    let (app, fx) = app.handle(Input::BleScanned {
        seq: None,
        result: Ok(heapless::Vec::new()),
    });
    assert!(responses(&fx).is_empty());
    assert!(fx.contains(&Effect::Redraw));
    assert_eq!(app.ble_scan_state(), ScanState::Done);
}

#[test]
fn fr13_redraw_only_on_visible_change() {
    let (mut app, _) = host(
        booted(),
        1,
        Cmd::ShowScreen(ShowScreen {
            screen: Screen::Buttons,
        }),
    );
    for t in (20..2000).step_by(20) {
        let (next, fx) = tick(app, t, 0);
        assert!(!fx.contains(&Effect::Redraw), "BUTTONS screen redrawn at {t}");
        app = next;
    }
    // HOME is static (QR code and version): no periodic redraw.
    let (mut app, _) = host(
        app,
        2,
        Cmd::ShowScreen(ShowScreen {
            screen: Screen::Home,
        }),
    );
    let mut redraws = 0;
    for t in (2000..5000).step_by(20) {
        let (next, fx) = tick(app, t, 0);
        redraws += usize::from(fx.contains(&Effect::Redraw));
        app = next;
    }
    assert_eq!(redraws, 0);
}

#[test]
fn fr13_buttons_screen_redraws_on_edges() {
    let (app, _) = host(
        booted(),
        1,
        Cmd::ShowScreen(ShowScreen {
            screen: Screen::Buttons,
        }),
    );
    let (app, fx) = tick(app, 100, 1 << 4);
    assert!(!fx.contains(&Effect::Redraw));
    let (_, fx) = tick(app, 130, 1 << 4);
    assert!(fx.contains(&Effect::Redraw));
}

#[test]
fn fr13_leds_screen_redraw_is_throttled() {
    let (mut app, _) = host(
        booted(),
        1,
        Cmd::ShowScreen(ShowScreen {
            screen: Screen::Leds,
        }),
    );
    let mut redraws = 0;
    for t in (20..=1000).step_by(20) {
        let (next, fx) = tick(app, t, 0);
        redraws += usize::from(fx.contains(&Effect::Redraw));
        app = next;
    }
    assert!((3..=4).contains(&redraws), "{redraws} redraws in 1 s");
}

#[test]
fn fr14_show_screen() {
    let (app, fx) = host(
        booted(),
        1,
        Cmd::ShowScreen(ShowScreen {
            screen: Screen::Ble,
        }),
    );
    assert_eq!(app.screen(), Screen::Ble);
    assert_eq!(fx[0], Effect::Redraw);
    assert_eq!(
        events(&fx),
        [Event_::Kind::Screen(ScreenChanged {
            screen: Screen::Ble
        })]
    );
    assert_eq!(only_response(&fx, 1), Body::Ack(Ack {}));
    // Same screen: just the ack.
    let (app, fx) = host(
        app,
        2,
        Cmd::ShowScreen(ShowScreen {
            screen: Screen::Ble,
        }),
    );
    assert_eq!(fx.len(), 1);
    #[cfg(not(feature = "dev"))]
    let bad_screens = [Screen(10), Screen(-1), Screen::Gpio];
    #[cfg(feature = "dev")]
    let bad_screens = [Screen(10), Screen(-1)];
    for bad in bad_screens {
        let (next, fx) = host(app.clone(), 3, Cmd::ShowScreen(ShowScreen { screen: bad }));
        assert_eq!(error_code(&only_response(&fx, 3)), ErrorCode::BadArgument);
        assert_eq!(next.screen(), Screen::Ble);
    }
}

#[test]
fn fr14_display_text_switches_to_host() {
    let text = truncated("hello\nbadge");
    let (app, fx) = host(
        booted(),
        1,
        Cmd::DisplayText(DisplayText {
            text,
            fg: rgb(0x00ff_0000),
            bg: None,
        }),
    );
    assert_eq!(app.screen(), Screen::Host);
    assert_eq!(app.host_text().text(), "hello\nbadge");
    assert_eq!(app.host_text().fg(), Color::new(255, 0, 0));
    assert_eq!(app.host_text().bg(), Color::BLACK);
    assert_eq!(fx.iter().filter(|e| **e == Effect::Redraw).count(), 1);
    assert_eq!(only_response(&fx, 1), Body::Ack(Ack {}));
    // Already on HOST: still redraws, no screen event.
    let (app, fx) = host(app, 2, Cmd::DisplayText(DisplayText::default()));
    assert!(fx.contains(&Effect::Redraw));
    assert!(events(&fx).is_empty());
    assert_eq!(app.host_text().fg(), Color::WHITE);
}

#[test]
fn fr14_display_fill_redraws_before_drawing() {
    let (app, fx) = host(
        booted(),
        1,
        Cmd::DisplayFill(DisplayFill {
            color: rgb(0x0000_ff00),
        }),
    );
    let draw = Effect::Draw(DrawOp::Fill(Color::new(0, 255, 0)));
    let redraw_at = fx
        .iter()
        .position(|e| *e == Effect::Redraw)
        .expect("redraw");
    let draw_at = fx.iter().position(|e| *e == draw).expect("draw");
    assert!(redraw_at < draw_at);
    assert_eq!(only_response(&fx, 1), Body::Ack(Ack {}));
    assert_eq!(app.screen(), Screen::Host);
    // On HOST already: no redraw that would wipe the canvas.
    let (_, fx) = host(app, 2, Cmd::DisplayFill(DisplayFill { color: None }));
    assert!(!fx.contains(&Effect::Redraw));
    assert!(fx.contains(&Effect::Draw(DrawOp::Fill(Color::BLACK))));
}

#[test]
fn fr14_display_rect_validates_and_clamps() {
    let rect = |x, y, w, h| {
        Cmd::DisplayRect(DisplayRect {
            x,
            y,
            w,
            h,
            color: rgb(0xffffff),
        })
    };
    let (app, fx) = host(booted(), 1, rect(200, 230, 100, u32::MAX));
    assert!(fx.contains(&Effect::Draw(DrawOp::Rect {
        x: 200,
        y: 230,
        w: 40,
        h: 10,
        color: Color::WHITE
    })));
    assert_eq!(only_response(&fx, 1), Body::Ack(Ack {}));
    for bad in [
        rect(240, 0, 1, 1),
        rect(0, 240, 1, 1),
        rect(0, 0, 0, 1),
        rect(0, 0, 1, 0),
        rect(u32::MAX, 0, 1, 1),
    ] {
        let (_, fx) = host(app.clone(), 2, bad);
        assert_eq!(fx.len(), 1);
        assert_eq!(error_code(&only_response(&fx, 2)), ErrorCode::BadArgument);
    }
}

// ------------------------------------------------------------------ GPIO

#[cfg(not(feature = "dev"))]
#[test]
fn fr15_gpio_is_unsupported_without_dev_feature() {
    for cmd in [
        Cmd::GpioRead(GpioRead { pin: 1 }),
        Cmd::GpioWrite(GpioWrite {
            pin: 5,
            level: true,
        }),
    ] {
        let (_, fx) = host(booted(), 1, cmd);
        assert_eq!(fx.len(), 1);
        assert_eq!(error_code(&only_response(&fx, 1)), ErrorCode::Unsupported);
    }
}

#[cfg(feature = "dev")]
#[test]
fn fr15_gpio_only_on_probe_pins() {
    let (app, fx) = host(
        booted(),
        1,
        Cmd::GpioWrite(GpioWrite {
            pin: 5,
            level: true,
        }),
    );
    assert_eq!(
        fx.as_slice(),
        &[Effect::GpioWrite {
            seq: 1,
            pin: 5,
            level: true
        }]
    );
    for pin in [0, 2, 19, 45, 49, u32::MAX] {
        let (_, fx) = host(app.clone(), 2, Cmd::GpioRead(GpioRead { pin }));
        assert_eq!(
            error_code(&only_response(&fx, 2)),
            ErrorCode::PinNotAllowed,
            "pin {pin}"
        );
        let (_, fx) = host(
            app.clone(),
            3,
            Cmd::GpioWrite(GpioWrite { pin, level: true }),
        );
        assert_eq!(
            error_code(&only_response(&fx, 3)),
            ErrorCode::PinNotAllowed,
            "pin {pin}"
        );
        assert_eq!(fx.len(), 1);
    }
    let (app, _) = host(
        app,
        4,
        Cmd::ShowScreen(ShowScreen {
            screen: Screen::Gpio,
        }),
    );
    let (app, fx) = app.handle(Input::Gpio {
        seq: 1,
        pin: 5,
        result: Err(ErrorCode::Radio),
    });
    assert_eq!(error_code(&only_response(&fx, 1)), ErrorCode::Radio);
    assert!(fx.contains(&Effect::Redraw));
    assert_eq!(
        app.last_gpio(),
        Some(GpioResult {
            pin: 5,
            result: Err(ErrorCode::Radio)
        })
    );
}

// ----------------------------------------------------------------- Wi-Fi

#[test]
fn fr16_wifi_scan_busy_and_result() {
    let (app, fx) = host(booted(), 1, Cmd::WifiScan(WifiScan {}));
    assert_eq!(fx.as_slice(), &[Effect::WifiScan { seq: Some(1) }]);
    let (app, fx) = host(app, 2, Cmd::WifiScan(WifiScan {}));
    assert_eq!(error_code(&only_response(&fx, 2)), ErrorCode::Busy);
    let mut aps = heapless::Vec::new();
    aps.push(ap("strong", -40)).ok();
    aps.push(ap("weak", -80)).ok();
    let (app, fx) = app.handle(Input::WifiScanned {
        seq: Some(1),
        result: Ok(aps.clone()),
    });
    assert_eq!(
        only_response(&fx, 1),
        Body::WifiScan(WifiScanResult { aps: aps.clone() })
    );
    assert_eq!(app.access_points(), aps.as_slice());
    assert_eq!(app.wifi_scan_state(), ScanState::Done);
    // Not busy any more; a failed scan keeps the old list.
    let (app, _) = host(app, 3, Cmd::WifiScan(WifiScan {}));
    let (app, fx) = app.handle(Input::WifiScanned {
        seq: Some(3),
        result: Err(ErrorCode::Radio),
    });
    assert_eq!(error_code(&only_response(&fx, 3)), ErrorCode::Radio);
    assert_eq!(app.wifi_scan_state(), ScanState::Failed(ErrorCode::Radio));
    assert_eq!(app.access_points().len(), 2);
}

#[test]
fn fr17_wifi_connect_and_status() {
    let (_, fx) = host(booted(), 1, Cmd::WifiConnect(WifiConnect::default()));
    assert_eq!(error_code(&only_response(&fx, 1)), ErrorCode::BadArgument);
    let connect = WifiConnect {
        ssid: truncated("lab"),
        password: truncated("hunter22"),
    };
    let (app, fx) = host(booted(), 2, Cmd::WifiConnect(connect));
    assert_eq!(
        fx.as_slice(),
        &[Effect::WifiConnect {
            seq: 2,
            ssid: truncated("lab"),
            password: truncated("hunter22")
        }]
    );
    let mut ipv4 = heapless::Vec::new();
    ipv4.extend_from_slice(&[10, 0, 0, 7]).ok();
    let state = WifiState {
        connected: true,
        ssid: truncated("lab"),
        ipv4,
        rssi: -50,
    };
    let (app, fx) = app.handle(Input::Wifi {
        seq: Some(2),
        result: Ok(state.clone()),
    });
    assert_eq!(only_response(&fx, 2), Body::WifiState(state.clone()));
    assert_eq!(app.wifi_link(), Ok(&state));
    // Unsolicited link change: stored, no response.
    let (app, fx) = app.handle(Input::Wifi {
        seq: None,
        result: Err(ErrorCode::Radio),
    });
    assert!(responses(&fx).is_empty());
    assert_eq!(app.wifi_link(), Err(ErrorCode::Radio));
}

// ------------------------------------------------------------------- BLE

#[test]
fn fr18_ble_scan_duration_busy_and_result() {
    for (asked, used) in [
        (0, 3000),
        (1, 1),
        (5000, 5000),
        (10_000, 10_000),
        (10_001, 10_000),
        (u32::MAX, 10_000),
    ] {
        let (_, fx) = host(booted(), 1, Cmd::BleScan(BleScan { duration_ms: asked }));
        assert_eq!(
            fx.as_slice(),
            &[Effect::BleScan {
                seq: Some(1),
                duration_ms: used
            }]
        );
    }
    let (app, _) = host(booted(), 1, Cmd::BleScan(BleScan { duration_ms: 0 }));
    let (app, fx) = host(app, 2, Cmd::BleScan(BleScan { duration_ms: 0 }));
    assert_eq!(error_code(&only_response(&fx, 2)), ErrorCode::Busy);
    let mut devices = heapless::Vec::new();
    devices
        .push(BleDevice {
            rssi: -60,
            name: truncated("tag"),
            ..Default::default()
        })
        .ok();
    let (app, fx) = app.handle(Input::BleScanned {
        seq: Some(1),
        result: Ok(devices.clone()),
    });
    assert_eq!(
        only_response(&fx, 1),
        Body::BleScan(BleScanResult {
            devices: devices.clone()
        })
    );
    assert_eq!(app.ble_devices(), devices.as_slice());
    assert!(!app.ble_scan_state().is_busy());
}

#[test]
fn fr19_ble_advertise() {
    // Boots advertising the default name, as the firmware does.
    assert_eq!(booted().ble_advertising(), "conagotchi");
    let name: String<29> = truncated("badge-rs");
    let (app, fx) = host(
        booted(),
        1,
        Cmd::BleAdvertise(BleAdvertise { name: name.clone() }),
    );
    assert_eq!(fx.as_slice(), &[Effect::BleAdvertise { seq: 1, name }]);
    assert_eq!(app.ble_advertising(), "conagotchi", "unchanged until confirmed");
    let (app, fx) = app.handle(Input::BleAdvertised {
        seq: 1,
        result: Ok(()),
    });
    assert_eq!(only_response(&fx, 1), Body::Ack(Ack {}));
    assert_eq!(app.ble_advertising(), "badge-rs");
    let (app, _) = host(app, 2, Cmd::BleAdvertise(BleAdvertise::default()));
    let (app, fx) = app.handle(Input::BleAdvertised {
        seq: 2,
        result: Err(ErrorCode::Radio),
    });
    assert_eq!(error_code(&only_response(&fx, 2)), ErrorCode::Radio);
    assert_eq!(app.ble_advertising(), "badge-rs");
}

// --------------------------------------------------------------- mailbox

#[test]
fn fr21_mailbox_commands_drive_the_badge() {
    let mail =
        |cmd, a0, a1| Input::Host(probe_layout::mailbox_to_host_msg(cmd, a0, a1).expect("known"));
    let (app, fx) = booted().handle(mail(mailbox::SET_PATTERN, 1, 255));
    assert_eq!(only_response(&fx, MAILBOX_SEQ), Body::Ack(Ack {}));
    assert_eq!((app.pattern(), app.brightness()), (LedPattern::Manual, 255));
    let (app, fx) = app.handle(mail(mailbox::SET_LED, 7, 0x0012_3456));
    assert_eq!(only_response(&fx, MAILBOX_SEQ), Body::Ack(Ack {}));
    assert_eq!(app.leds()[7], Color::new(0x12, 0x34, 0x56));
    let (app, fx) = app.handle(mail(mailbox::SHOW_SCREEN, 3, 0));
    assert_eq!(only_response(&fx, MAILBOX_SEQ), Body::Ack(Ack {}));
    assert_eq!(app.screen(), Screen::Wifi);
    let (_, fx) = app.handle(mail(mailbox::SET_LED, 8, 0));
    assert_eq!(
        error_code(&only_response(&fx, MAILBOX_SEQ)),
        ErrorCode::BadArgument
    );
}

// ------------------------------------------------------------------- pet

fn pet_changed(fx: &[Effect]) -> StdVec<(PetMode, PetMode)> {
    events(fx)
        .into_iter()
        .filter_map(|e| match e {
            Event_::Kind::Pet(p) => Some((p.selected, p.active)),
            _ => None,
        })
        .collect()
}

fn set_pet(app: App, seq: u32, mode: PetMode) -> (App, Effects) {
    host(app, seq, Cmd::SetPetMode(SetPetMode { mode }))
}

/// Asserts the display rule: never `Redraw` and `Animate` together.
fn assert_one_display_effect(fx: &[Effect]) {
    let display = fx
        .iter()
        .filter(|e| matches!(e, Effect::Redraw | Effect::Animate))
        .count();
    assert!(display <= 1, "{fx:?}");
}

#[test]
fn fr23_boot_screen_is_pet_in_auto() {
    let app = App::new(INFO);
    assert_eq!(app.screen(), Screen::Pet);
    assert_eq!(
        (app.pet().selected(), app.pet().active()),
        (PetMode::Auto, PetMode::Idle)
    );
    let (_, fx) = tick(app, 0, 0);
    assert!(fx.contains(&Effect::Redraw));
    assert!(!fx.contains(&Effect::Animate));
    assert!(pet_changed(&fx).is_empty(), "nothing changed yet");
}

#[test]
fn fr23_screen_order_puts_pet_after_home() {
    // Every screen value is reachable, except GPIO without the dev feature.
    for v in 0..SCREEN_COUNT {
        let reachable = SCREEN_ORDER.contains(&Screen(v));
        assert_eq!(
            reachable,
            cfg!(feature = "dev") || Screen(v) != Screen::Gpio,
            "screen {v}"
        );
    }
    assert_eq!(cycle_screen(Screen::Home, 1), Screen::Pet);
    assert_eq!(cycle_screen(Screen::Pet, 1), Screen::Trade);
    assert_eq!(cycle_screen(Screen::Pet, -1), Screen::Home);
    let last = if cfg!(feature = "dev") { Screen::Gpio } else { Screen::Host };
    assert_eq!(cycle_screen(Screen::Home, -1), last);
    assert_eq!(
        cycle_screen(Screen(99), 1),
        Screen::Pet,
        "unknown counts as HOME"
    );
    let (app, fx) = host(
        booted(),
        1,
        Cmd::ShowScreen(ShowScreen {
            screen: Screen::Pet,
        }),
    );
    assert_eq!(only_response(&fx, 1), Body::Ack(Ack {}));
    assert_eq!(app.screen(), Screen::Pet);
}

#[test]
fn fr23_select_cycles_pet_modes_on_pet_only() {
    let mut app = booted();
    let order = [
        PetMode::Idle,
        PetMode::Walk,
        PetMode::Eat,
        PetMode::Play,
        PetMode::Sleep,
        PetMode::Auto,
    ];
    for (i, want) in order.iter().enumerate() {
        let (next, fx) = click(app, 100 + 100 * i as u64, SELECT);
        assert_eq!(next.pet().selected(), *want);
        let active = if *want == PetMode::Auto {
            PetMode::Idle
        } else {
            *want
        };
        assert_eq!(next.pet().active(), active);
        assert_eq!(pet_changed(&fx), [(*want, active)]);
        assert!(fx.contains(&Effect::Redraw), "label changes");
        app = next;
    }
    // SELECT elsewhere leaves the pet alone.
    let (app, _) = host(
        app,
        1,
        Cmd::ShowScreen(ShowScreen {
            screen: Screen::Home,
        }),
    );
    let (app, fx) = click(app, 1000, SELECT);
    assert_eq!(app.pet().selected(), PetMode::Auto);
    assert!(pet_changed(&fx).is_empty());
}

#[test]
fn fr23_pet_animates_without_redraw() {
    let (mut app, _) = set_pet(booted(), 1, PetMode::Sleep);
    let mut animates = StdVec::new();
    for t in (20..=3000).step_by(20) {
        let (next, fx) = tick(app, t, 0);
        assert!(
            !fx.contains(&Effect::Redraw),
            "frame-only change redrew at {t}"
        );
        assert_one_display_effect(&fx);
        if fx.contains(&Effect::Animate) {
            animates.push(t);
        }
        app = next;
    }
    assert_eq!(animates, [700, 1400, 2100, 2800]);
    // Off the PET screen the pet keeps time but asks for nothing.
    let (mut app, _) = host(
        app,
        2,
        Cmd::ShowScreen(ShowScreen {
            screen: Screen::Buttons,
        }),
    );
    for t in (3020..=6000).step_by(20) {
        let (next, fx) = tick(app, t, 0);
        assert!(!fx.contains(&Effect::Animate) && !fx.contains(&Effect::Redraw));
        app = next;
    }
    assert_eq!(app.pet().frame(), 0, "6000 ms into SLEEP: 8 frames of 4");
}

#[test]
fn fr23_auto_advance_emits_pet_changed() {
    // On the PET screen: the mode switch redraws (label) instead of animating.
    let mut app = booted();
    let mut seen = StdVec::new();
    for t in (20..=16_000).step_by(20) {
        let (next, fx) = tick(app, t, 0);
        assert_one_display_effect(&fx);
        for p in pet_changed(&fx) {
            assert!(
                fx.contains(&Effect::Redraw),
                "mode change at {t} without redraw"
            );
            seen.push((t, p));
        }
        app = next;
    }
    assert_eq!(
        seen,
        [
            (8000, (PetMode::Auto, PetMode::Walk)),
            (16_000, (PetMode::Auto, PetMode::Eat))
        ]
    );
    // On another screen AUTO keeps cycling and still reports.
    let (mut app, _) = host(
        app,
        1,
        Cmd::ShowScreen(ShowScreen {
            screen: Screen::Buttons,
        }),
    );
    let mut seen = StdVec::new();
    for t in (16_020..=24_000).step_by(20) {
        let (next, fx) = tick(app, t, 0);
        assert!(!fx.contains(&Effect::Redraw) && !fx.contains(&Effect::Animate));
        seen.extend(pet_changed(&fx));
        app = next;
    }
    assert_eq!(seen, [(PetMode::Auto, PetMode::Play)]);
}

#[test]
fn fr24_set_pet_mode_switches_to_pet_and_acks() {
    let (app, fx) = set_pet(on_home(), 7, PetMode::Sleep);
    assert_eq!(app.screen(), Screen::Pet);
    assert_eq!(
        (app.pet().selected(), app.pet().active()),
        (PetMode::Sleep, PetMode::Sleep)
    );
    assert_eq!(fx[0], Effect::Redraw);
    assert_eq!(
        events(&fx),
        [
            Event_::Kind::Screen(ScreenChanged {
                screen: Screen::Pet
            }),
            Event_::Kind::Pet(PetChanged {
                selected: PetMode::Sleep,
                active: PetMode::Sleep
            })
        ]
    );
    assert_eq!(only_response(&fx, 7), Body::Ack(Ack {}));
    assert_one_display_effect(&fx);
    // Already on PET in that mode: just the ack.
    let (app, fx) = set_pet(app, 8, PetMode::Sleep);
    assert_eq!(fx.as_slice().len(), 1, "{fx:?}");
    assert_eq!(only_response(&fx, 8), Body::Ack(Ack {}));
    // Another mode while on PET: redraw and event, no screen change.
    let (app, fx) = set_pet(app, 9, PetMode::Auto);
    assert!(fx.contains(&Effect::Redraw));
    assert_eq!(pet_changed(&fx), [(PetMode::Auto, PetMode::Idle)]);
    assert!(
        !events(&fx)
            .iter()
            .any(|e| matches!(e, Event_::Kind::Screen(_)))
    );
    // Invalid values are refused and change nothing.
    let (app, _) = host(
        app,
        10,
        Cmd::ShowScreen(ShowScreen {
            screen: Screen::Leds,
        }),
    );
    for bad in [PetMode(6), PetMode(-1), PetMode(i32::MIN)] {
        let (next, fx) = set_pet(app.clone(), 11, bad);
        assert_eq!(fx.len(), 1);
        assert_eq!(error_code(&only_response(&fx, 11)), ErrorCode::BadArgument);
        assert_eq!(next.screen(), Screen::Leds);
        assert_eq!(next.pet(), app.pet());
    }
}

#[test]
fn fr24_mailbox_set_pet_mode() {
    let mail =
        |cmd, a0, a1| Input::Host(probe_layout::mailbox_to_host_msg(cmd, a0, a1).expect("known"));
    let (app, fx) = on_home().handle(mail(mailbox::SET_PET_MODE, 3, 0));
    assert_eq!(only_response(&fx, MAILBOX_SEQ), Body::Ack(Ack {}));
    assert_eq!(app.screen(), Screen::Pet);
    assert_eq!(app.pet().selected(), PetMode::Eat);
    assert_eq!(pet_changed(&fx), [(PetMode::Eat, PetMode::Eat)]);
    let (next, fx) = app.clone().handle(mail(mailbox::SET_PET_MODE, 9, 0));
    assert_eq!(
        error_code(&only_response(&fx, MAILBOX_SEQ)),
        ErrorCode::BadArgument
    );
    assert_eq!(next.pet(), app.pet());
}

#[test]
fn fr24_pet_changed_is_deferred_when_budget_is_full_then_delivered() {
    // WIFI screen, SELECT + RIGHT accepted on the tick where AUTO moves on:
    // 2 button events + WifiScan + ScreenChanged + Redraw + Leds. The budget
    // reserves room for PetChanged, so it goes out in the same tick; the
    // deferral path (PetChanged only when a slot is left) never drops it.
    let (app, _) = host(
        booted(),
        1,
        Cmd::ShowScreen(ShowScreen {
            screen: Screen::Wifi,
        }),
    );
    let (app, _) = tick(app, 7970, SELECT | RIGHT);
    let (app, fx) = tick(app, 8000, SELECT | RIGHT);
    assert_eq!(app.pet().active(), PetMode::Walk);
    assert!(fx.len() <= MAX_EFFECTS, "{fx:?}");
    assert_eq!(pet_changed(&fx), [(PetMode::Auto, PetMode::Walk)]);
    assert_eq!(app.dropped_effects(), 0);
    let (app, fx) = tick(app, 8020, SELECT | RIGHT);
    assert!(pet_changed(&fx).is_empty(), "reported once");
    let (_, fx) = tick(app, 8040, SELECT | RIGHT);
    assert!(pet_changed(&fx).is_empty(), "reported once");
}

// ---------------------------------------------------------- effect bound

/// Every request type with ordinary and edge-case arguments.
fn immediate_cmds() -> StdVec<Cmd> {
    let mut colors = heapless::Vec::new();
    for i in 0..8 {
        colors
            .push(Rgb {
                rgb: i * 0x0011_1111,
            })
            .ok();
    }
    StdVec::from([
        Cmd::Ping(Ping { nonce: 1 }),
        Cmd::GetInfo(GetInfo {}),
        Cmd::SetLed(SetLed {
            index: 0,
            color: None,
        }),
        Cmd::SetLed(SetLed {
            index: 8,
            color: None,
        }),
        Cmd::SetLeds(SetLeds { colors }),
        Cmd::SetLeds(SetLeds::default()),
        Cmd::SetPattern(SetPattern {
            pattern: LedPattern::Breathe,
            brightness: 9,
        }),
        Cmd::SetPattern(SetPattern {
            pattern: LedPattern(99),
            brightness: 9,
        }),
        Cmd::SetIndicator(SetIndicator {
            which: Indicator::Alert,
            on: true,
        }),
        Cmd::SetIndicator(SetIndicator {
            which: Indicator::Unspecified,
            on: true,
        }),
        Cmd::GetButtons(GetButtons {}),
        Cmd::ShowScreen(ShowScreen {
            screen: Screen::Leds,
        }),
        Cmd::ShowScreen(ShowScreen { screen: Screen(70) }),
        Cmd::DisplayText(DisplayText {
            text: truncated("x\ny"),
            fg: None,
            bg: rgb(0x000080),
        }),
        Cmd::DisplayFill(DisplayFill { color: rgb(1) }),
        Cmd::DisplayRect(DisplayRect {
            x: 1,
            y: 1,
            w: 1,
            h: 1,
            color: None,
        }),
        Cmd::DisplayRect(DisplayRect {
            x: 999,
            y: 1,
            w: 1,
            h: 1,
            color: None,
        }),
        Cmd::GpioRead(GpioRead { pin: 2 }),
        Cmd::GpioWrite(GpioWrite {
            pin: 45,
            level: true,
        }),
        Cmd::WifiConnect(WifiConnect::default()),
        Cmd::SetPetMode(SetPetMode {
            mode: PetMode::Play,
        }),
        Cmd::SetPetMode(SetPetMode {
            mode: PetMode::Auto,
        }),
        Cmd::SetPetMode(SetPetMode { mode: PetMode(6) }),
    ])
}

fn all_inputs() -> StdVec<Input> {
    let mut inputs: StdVec<Input> = immediate_cmds()
        .into_iter()
        .map(|c| {
            Input::Host(HostMsg {
                seq: 1,
                cmd: Some(c),
            })
        })
        .collect();
    for cmd in [
        Cmd::GpioRead(GpioRead { pin: 1 }),
        Cmd::GpioWrite(GpioWrite {
            pin: 1,
            level: false,
        }),
        Cmd::WifiScan(WifiScan {}),
        Cmd::WifiConnect(WifiConnect {
            ssid: truncated("a"),
            password: truncated(""),
        }),
        Cmd::WifiStatus(WifiStatus {}),
        Cmd::BleScan(BleScan { duration_ms: 0 }),
        Cmd::BleAdvertise(BleAdvertise {
            name: truncated("n"),
        }),
    ] {
        inputs.push(Input::Host(HostMsg {
            seq: 2,
            cmd: Some(cmd),
        }));
    }
    let mut full_aps = heapless::Vec::new();
    let mut full_ble = heapless::Vec::new();
    for i in 0..12 {
        full_aps.push(ap("ap", -40 - i)).ok();
        full_ble
            .push(BleDevice {
                rssi: -40 - i,
                ..Default::default()
            })
            .ok();
    }
    inputs.extend([
        Input::Host(HostMsg { seq: 3, cmd: None }),
        Input::HostGarbage,
        Input::Gpio {
            seq: 1,
            pin: 1,
            result: Ok(false),
        },
        Input::Gpio {
            seq: 1,
            pin: 1,
            result: Err(ErrorCode::Radio),
        },
        Input::WifiScanned {
            seq: Some(2),
            result: Ok(full_aps),
        },
        Input::WifiScanned {
            seq: None,
            result: Err(ErrorCode::Radio),
        },
        Input::Wifi {
            seq: Some(2),
            result: Ok(WifiState::default()),
        },
        Input::Wifi {
            seq: None,
            result: Err(ErrorCode::Radio),
        },
        Input::BleScanned {
            seq: Some(2),
            result: Ok(full_ble),
        },
        Input::BleScanned {
            seq: None,
            result: Err(ErrorCode::Radio),
        },
        Input::BleAdvertised {
            seq: 2,
            result: Ok(()),
        },
        Input::BleAdvertised {
            seq: 2,
            result: Err(ErrorCode::Radio),
        },
    ]);
    inputs
}

#[test]
fn fr23_effects_never_exceed_bound() {
    let masks = [
        0u8,
        LEFT,
        RIGHT,
        SELECT,
        LEFT | SELECT,
        RIGHT | SELECT,
        LEFT | RIGHT,
        0b0011_1110,
        0xff,
    ];
    let mut checked = 0;
    for screen in SCREEN_ORDER {
        for pattern in 0..4 {
            for mode in crate::pet::MODES {
                let (start, _) = set_pet(booted(), 1, mode);
                let (start, _) = host(start, 1, Cmd::ShowScreen(ShowScreen { screen }));
                let (start, _) = host(
                    start,
                    1,
                    Cmd::SetPattern(SetPattern {
                        pattern: LedPattern(pattern),
                        brightness: 255,
                    }),
                );
                // Every host / shell input from this state.
                for input in all_inputs() {
                    let (next, fx) = start.clone().handle(input.clone());
                    assert!(fx.len() <= MAX_EFFECTS, "{input:?} -> {fx:?}");
                    assert_eq!(next.dropped_effects(), 0, "{input:?} overflowed");
                    assert_one_display_effect(&fx);
                    checked += 1;
                }
                // Button chords, including all five at once, held then
                // released; once early and once across the first AUTO switch.
                for base in [100, AUTO_SWITCH - 30] {
                    for &mask in &masks {
                        let mut app = start.clone();
                        for (dt, m) in [
                            (0, mask),
                            (30, mask),
                            (50, mask),
                            (70, mask),
                            (100, 0),
                            (130, 0),
                            (150, 0),
                            (170, 0),
                        ] {
                            let t = base + dt;
                            let (next, fx) = tick(app, t, m);
                            assert!(fx.len() <= MAX_EFFECTS, "mask {mask:#x} at {t}: {fx:?}");
                            assert_eq!(next.dropped_effects(), 0);
                            assert_one_display_effect(&fx);
                            app = next;
                            checked += 1;
                        }
                        // Deferred edges are delivered eventually: all released.
                        assert_eq!(app.pressed(), ButtonMask(0));
                        // A deferred PetChanged goes out on the next quiet tick.
                        let (app, _) = tick(app, base + 190, 0);
                        assert_eq!(
                            app.pet_reported,
                            (app.pet().selected(), app.pet().active()),
                            "PetChanged never delivered"
                        );
                    }
                }
            }
        }
    }
    assert!(checked > 10_000);
}

/// First AUTO mode switch after boot.
const AUTO_SWITCH: u64 = crate::pet::AUTO_DWELL_MS;

#[test]
fn chord_of_five_counts_every_press() {
    let all = 0b0011_1110;
    let mut app = booted();
    let mut presses = 0;
    for t in (100..400).step_by(20) {
        let (next, fx) = tick(app, t, all);
        presses += events(&fx)
            .iter()
            .filter(|e| matches!(e, Event_::Kind::Button(b) if b.pressed))
            .count();
        app = next;
    }
    assert_eq!(presses, 5);
    assert_eq!(app.press_counts(), [0, 1, 1, 1, 1, 1]);
}

// ------------------------------------------------------------------- adc

#[cfg(feature = "dev")]
#[test]
fn fr40_adc_read_on_adc1_probe_pins_only() {
    use badge_proto::AdcRead;
    let (_, fx) = host(booted(), 1, Cmd::AdcRead(AdcRead { pin: 3, samples: 0 }));
    assert_eq!(fx.as_slice(), &[Effect::AdcRead { seq: 1, pin: 3, samples: 16 }]);
    let (_, fx) = host(booted(), 2, Cmd::AdcRead(AdcRead { pin: 8, samples: 64 }));
    assert_eq!(fx.as_slice(), &[Effect::AdcRead { seq: 2, pin: 8, samples: 64 }]);
    // ADC2 probe pin, non-probe pin, too many samples.
    for (pin, samples, code) in [
        (15, 0, ErrorCode::PinNotAllowed),
        (2, 0, ErrorCode::PinNotAllowed),
        (1, 65, ErrorCode::BadArgument),
    ] {
        let (_, fx) = host(booted(), 3, Cmd::AdcRead(AdcRead { pin, samples }));
        assert_eq!(fx.len(), 1);
        assert_eq!(error_code(&only_response(&fx, 3)), code);
    }
}

#[cfg(feature = "dev")]
#[test]
fn fr40_adc_result_is_the_one_response() {
    use badge_proto::AdcLevel;
    let level = AdcLevel { pin: 1, millivolts: 1850, min_mv: 1840, max_mv: 1861, samples: 16 };
    let (_, fx) = booted().handle(Input::Adc { seq: 4, result: Ok(level) });
    assert_eq!(only_response(&fx, 4), Body::Adc(level));
    let (_, fx) = booted().handle(Input::Adc { seq: 5, result: Err(ErrorCode::Radio) });
    assert_eq!(error_code(&only_response(&fx, 5)), ErrorCode::Radio);
}

#[cfg(not(feature = "dev"))]
#[test]
fn fr40_adc_is_unsupported_without_dev_feature() {
    use badge_proto::AdcRead;
    let (_, fx) = host(booted(), 1, Cmd::AdcRead(AdcRead { pin: 1, samples: 0 }));
    assert_eq!(fx.len(), 1);
    assert_eq!(error_code(&only_response(&fx, 1)), ErrorCode::Unsupported);
}
