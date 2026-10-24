//! TRADE and CONTACTS screens.
//!
//! TRADE (content rows 0..=9):
//! - row 0: our id and the Chi we offer; row 1: "signed" / "unsigned";
//! - row 2: the trade state ("inviting 1A2B3C", "got HCK from 1A2B3C", ...);
//! - row 3: how many badges are in the lobby;
//! - rows 4..=9: up to [`LOBBY_ROWS`] lobby rows, `id tag rssi via mark`
//!   (`REQ` = it requests us, `INV` = we invited it), scrolled so the pick
//!   cursor (`>`, accent colour) stays visible;
//! - hint: what SELECT / START do now.
//!
//! CONTACTS: a header with the count and page, then one page of
//! [`CONTACT_ROWS`] contacts newest first (`id tag xN name`); SELECT pages.
//!
//! Every line goes through the shared row helpers, so it is cut to the
//! circle's chord at its height.

use badge_proto::TradeState;
use embedded_graphics::{pixelcolor::Rgb565, prelude::*};
use heapless::String;

use super::{ACCENT, DIM, TEXT, format, hint, row};
use crate::app::{App, CONTACT_ROWS, LobbyRow};
use crate::io::Via;
use crate::oz::chi_name;

/// Lobby rows shown at once.
pub const LOBBY_ROWS: usize = 6;
/// Content row of the first lobby row.
const LOBBY_ROW0: usize = 4;
/// Content row of the first contact.
const CONTACT_ROW0: usize = 2;

/// The trade-state line.
fn state_line(app: &App) -> String<40> {
    let t = app.trade();
    let peer = t.peer();
    let id = peer.as_ref().map_or("------", |p| p.as_str());
    match t.state() {
        TradeState::Inviting => format(format_args!("inviting {id}")),
        TradeState::Exchanging if t.unknown_tag() => format(format_args!("{id}: unknown chi")),
        TradeState::Exchanging => format(format_args!("trading with {id}")),
        TradeState::Complete => {
            format(format_args!("got {} from {id}", chi_name(t.received())))
        }
        TradeState::Lost => format(format_args!("{id}: no answer")),
        TradeState::Cancelled => format(format_args!("cancelled {id}")),
        _ => format(format_args!("lobby open")),
    }
}

/// One lobby row's text.
fn lobby_line(r: &LobbyRow, cursor: bool) -> String<40> {
    let mark = match (r.invited, r.wants) {
        (true, _) => "INV",
        (false, true) => "REQ",
        _ => "",
    };
    let via = match r.via {
        Via::Ble => "BT",
        Via::Link => "LINK",
    };
    let at = if cursor { ">" } else { " " };
    format(format_args!(
        "{at}{} {} {:>4} {via} {mark}",
        r.id.as_str(),
        chi_name(r.chi),
        r.rssi
    ))
}

/// Lobby list and current trade.
pub(super) fn trade_screen<D: DrawTarget<Color = Rgb565>>(
    app: &App,
    target: &mut D,
) -> Result<(), D::Error> {
    let t = app.trade();
    let head: String<40> = format(format_args!(
        "{} offers {}",
        t.me().as_str(),
        chi_name(t.chi())
    ));
    row(target, 0, &head, TEXT)?;
    row(
        target,
        1,
        if app.signed() { "signed" } else { "unsigned" },
        DIM,
    )?;
    row(target, 2, &state_line(app), ACCENT)?;
    let rows = t.rows();
    let count: String<24> = format(format_args!("{} nearby", rows.len()));
    row(target, 3, &count, DIM)?;
    let cursor = t.picking();
    if rows.is_empty() {
        row(target, LOBBY_ROW0, "no badges nearby", DIM)?;
    }
    let first = cursor.map_or(0, |c| c.saturating_sub(LOBBY_ROWS - 1));
    for (i, r) in rows.iter().enumerate().skip(first).take(LOBBY_ROWS) {
        let here = cursor == Some(i);
        let color = if here { ACCENT } else { TEXT };
        row(target, LOBBY_ROW0 + (i - first), &lobby_line(r, here), color)?;
    }
    let busy = t.session().active_peer().is_some();
    hint(
        target,
        match (cursor.is_some(), busy) {
            (true, _) => "L/R pick SEL go",
            (false, true) => "START: cancel",
            (false, false) => "SELECT: pick",
        },
    )
}

/// Scrollable contact list.
pub(super) fn contacts_screen<D: DrawTarget<Color = Rgb565>>(
    app: &App,
    target: &mut D,
) -> Result<(), D::Error> {
    let contacts = app.contacts();
    if contacts.is_empty() {
        row(target, 0, "no contacts yet", DIM)?;
        return hint(target, "trade to add");
    }
    let pages = contacts.len().div_ceil(CONTACT_ROWS).max(1);
    let page = app.trade().contacts_page().min(pages - 1);
    let head: String<32> = format(format_args!(
        "{} contacts  {}/{}",
        contacts.len(),
        page + 1,
        pages
    ));
    row(target, 0, &head, TEXT)?;
    for (i, c) in contacts
        .newest_first()
        .skip(page * CONTACT_ROWS)
        .take(CONTACT_ROWS)
        .enumerate()
    {
        let line: String<40> = format(format_args!(
            "{} {} x{} {}",
            c.id.as_str(),
            chi_name(c.chi),
            c.trades,
            c.name
        ));
        row(target, CONTACT_ROW0 + i, &line, TEXT)?;
    }
    hint(
        target,
        if pages > 1 {
            "SELECT: next page"
        } else {
            "trade to add"
        },
    )
}

#[cfg(test)]
mod tests {
    extern crate std;

    use core::convert::Infallible;
    use std::{vec, vec::Vec as StdVec};

    use badge_proto::{Chi, HostMsg, HostMsg_::Cmd, Screen, ShowScreen, TradeInvite};
    use embedded_graphics::{Pixel, pixelcolor::raw::RawU16};

    use super::super::{BG, render, to_rgb565};
    use super::*;
    use crate::app::BoardInfo;
    use crate::contacts::Contacts;
    use crate::io::{ButtonMask, Input, Millis, StorageEvent};
    use crate::oz::{PeerId, sign, sign::RadioKey};

    /// Feeds stored contacts one by one, then `Loaded`, as the firmware does at boot.
    fn restore(app: App, contacts: Contacts, key: Option<RadioKey>) -> (App, crate::io::Effects) {
        let app = contacts.as_slice().iter().fold(app, |app, c| {
            app.handle(Input::Storage(StorageEvent::Contact(c.clone()))).0
        });
        app.handle(Input::Storage(StorageEvent::Loaded { key }))
    }

    const SIZE: usize = 240;
    /// A fake key: never a real one.
    const FAKE: RadioKey = RadioKey([0x11; 32]);

    /// Test-only full-size framebuffer.
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
                if let (Ok(x @ 0..240), Ok(y @ 0..240)) = (usize::try_from(p.x), usize::try_from(p.y))
                {
                    self.px[y * SIZE + x] = RawU16::from(c).into_inner();
                }
            }
            Ok(())
        }
    }

    fn frame(app: &App) -> Fb {
        let mut fb = Fb {
            px: vec![0xAAAA; SIZE * SIZE],
        };
        render(app, &mut fb).expect("infallible");
        fb
    }

    /// Nothing but background outside the circle; something drawn inside.
    fn assert_inside_circle(fb: &Fb, what: &str) {
        let bg = RawU16::from(to_rgb565(BG)).into_inner();
        let mut lit = 0;
        for y in 0..SIZE {
            for x in 0..SIZE {
                if fb.px[y * SIZE + x] == bg {
                    continue;
                }
                lit += 1;
                let dx = 2 * x as i64 + 1 - 240;
                let dy = 2 * y as i64 + 1 - 240;
                assert!(dx * dx + dy * dy <= 240 * 240, "{what}: pixel ({x},{y}) outside");
            }
        }
        assert!(lit > 100, "{what}: only {lit} pixels");
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

    /// A raw-kind BLE body carrying `text`, signed with the fake key.
    fn lobby_packet(app: App, text: &str, rssi: i8) -> App {
        let signed: heapless::Vec<u8, 32> =
            sign::sign(Some(&FAKE), text.as_bytes()).expect("fits");
        let mut payload = heapless::Vec::new();
        payload.push(0).ok();
        payload.extend_from_slice(&signed).ok();
        app.handle(Input::RadioPacket {
            via: crate::io::Via::Ble,
            rssi,
            payload,
        })
        .0
    }

    /// Booted app with a full lobby (8 badges, one requesting us), keyed.
    fn busy_trade() -> App {
        let app = App::new(BoardInfo {
            mac: [0xde, 0xad, 0xbe, 0xef, 0x00, 0x01],
            firmware: "t",
        });
        let mut contacts = Contacts::new();
        for n in 0..64u32 {
            contacts = contacts
                .record_trade(PeerId::from_u32(0xF00000 + n), Chi::Ovr, "WWWWWWWW")
                .0;
        }
        for _ in 0..3 {
            contacts = contacts.record_trade(PeerId::from_u32(0xF00001), Chi::Cry, "").0;
        }
        let mut app = restore(app, contacts, Some(FAKE))
            .0
            .handle(Input::Tick {
                now: Millis(1000),
                buttons: ButtonMask(0),
            })
            .0;
        for n in 0..8u32 {
            let to = if n == 3 { "EF0001" } else { "------" };
            let text = std::format!("OZGT:{:06X}:{to}:~ADM", 0xA00000 + n);
            app = lobby_packet(app, &text, -55);
        }
        let mut peer = heapless::String::new();
        peer.push_str("A00005").ok();
        host(app, Cmd::TradeInvite(TradeInvite { peer }))
    }

    #[test]
    fn fr25_trade_screen_stays_inside_circle() {
        let app = show(busy_trade(), Screen::Trade);
        assert_inside_circle(&frame(&app), "TRADE");
        // Pick mode with the cursor scrolled to the last row.
        let mut app = app;
        let mut t = 1100;
        for mask in [1u8 << 3, 0, 1 << 2, 0] {
            for _ in 0..3 {
                app = app
                    .handle(Input::Tick {
                        now: Millis(t),
                        buttons: ButtonMask(mask),
                    })
                    .0;
                t += 20;
            }
        }
        assert_eq!(app.trade().picking(), Some(7));
        assert_inside_circle(&frame(&app), "TRADE pick");
        // Every trade state line.
        let app = lobby_packet(app, "OZGT:A00005:EF0001:ZZZ0", -40);
        assert_eq!(app.trade().state(), TradeState::Exchanging);
        assert_inside_circle(&frame(&app), "TRADE unknown");
        let app = lobby_packet(app, "OZGT:A00005:EF0001:HCK1", -40);
        assert_eq!(app.trade().state(), TradeState::Complete);
        assert!(state_line(&app).contains("got HCK"));
        assert_inside_circle(&frame(&app), "TRADE complete");
    }

    #[test]
    fn fr25_empty_trade_screen_renders() {
        let app = App::new(BoardInfo {
            mac: [0; 6],
            firmware: "t",
        });
        let app = show(app, Screen::Trade);
        assert_inside_circle(&frame(&app), "TRADE empty");
        assert_eq!(state_line(&app).as_str(), "lobby open");
    }

    #[test]
    fn fr26_contacts_screen_pages_stay_inside_circle() {
        let mut app = show(busy_trade(), Screen::Contacts);
        let mut t = 1100;
        for page in 0..12 {
            assert_eq!(app.trade().contacts_page(), page % 11);
            assert_inside_circle(&frame(&app), "CONTACTS");
            for mask in [1u8 << 3, 0] {
                for _ in 0..3 {
                    app = app
                        .handle(Input::Tick {
                            now: Millis(t),
                            buttons: ButtonMask(mask),
                        })
                        .0;
                    t += 20;
                }
            }
        }
        let empty = App::new(BoardInfo {
            mac: [0; 6],
            firmware: "t",
        });
        assert_inside_circle(&frame(&show(empty, Screen::Contacts)), "CONTACTS empty");
    }

    #[test]
    fn fr25_lobby_line_format() {
        let r = LobbyRow {
            id: PeerId::from_u32(0x1A2B3C),
            rssi: -48,
            via: Via::Ble,
            chi: Chi::Hck,
            wants: true,
            invited: false,
        };
        assert_eq!(lobby_line(&r, true).as_str(), ">1A2B3C HCK  -48 BT REQ");
        let r = LobbyRow {
            via: Via::Link,
            rssi: 0,
            chi: Chi::Unspecified,
            invited: true,
            ..r
        };
        assert_eq!(lobby_line(&r, false).as_str(), " 1A2B3C ?    0 LINK INV");
    }
}
