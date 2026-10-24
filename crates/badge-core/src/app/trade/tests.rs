//! Regression tests for the trade / contacts part of [`App`] (FR-25, FR-26,
//! FR-27), including a full trade between two simulated badges.

extern crate std;

use std::vec::Vec as StdVec;

use badge_proto::{
    BadgeMsg, BadgeMsg_, ClearContacts, Event as PEvent, GetLobby, HostMsg, ListContacts, Response,
    SetChi, ShowScreen, TradeCancel, TradeInvite,
};

use super::*;
use crate::app::{BoardInfo, MAX_EDGES_PER_TICK};
use crate::contacts::Contact;
use crate::io::{ButtonMask, Effects, Input, MAX_EFFECTS, Millis, StorageEvent};
use crate::oz::oz_payload;

const MAC_A: [u8; 6] = [0x24, 0x6f, 0x28, 0x0A, 0xC8, 0xF6];
const MAC_B: [u8; 6] = [0x24, 0x6f, 0x28, 0x1A, 0x2B, 0x3C];
/// A fake key: never a real one.
const FAKE: RadioKey = RadioKey([0x11; 32]);
const LEFT: u8 = 1 << 2;
const SELECT: u8 = 1 << 3;
const START: u8 = 1 << 4;
const RIGHT: u8 = 1 << 5;

/// Feeds stored contacts one by one, then `Loaded`, as the firmware does at boot.
fn restore(app: App, contacts: Contacts, key: Option<RadioKey>) -> (App, Effects) {
    let app = contacts.as_slice().iter().fold(app, |app, c| {
        app.handle(Input::Storage(StorageEvent::Contact(c.clone()))).0
    });
    app.handle(Input::Storage(StorageEvent::Loaded { key }))
}

fn app(mac: [u8; 6]) -> App {
    App::new(BoardInfo {
        mac,
        firmware: "test",
    })
}

fn id(s: &str) -> PeerId {
    PeerId::parse(s.as_bytes()).expect("id")
}

fn host(app: App, seq: u32, cmd: Cmd) -> (App, Effects) {
    app.handle(Input::Host(HostMsg {
        seq,
        cmd: Some(cmd),
    }))
}

fn tick(app: App, t: u64, mask: u8) -> (App, Effects) {
    app.handle(Input::Tick {
        now: Millis(t),
        buttons: ButtonMask(mask),
    })
}

fn keyed(app: App, key: Option<RadioKey>) -> App {
    restore(app, Contacts::new(), key)
    .0
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
                kind: Some(BadgeMsg_::Kind::Event(PEvent { kind: Some(k) })),
            }) => Some(k.clone()),
            _ => None,
        })
        .collect()
}

fn trade_events(fx: &[Effect]) -> StdVec<TradeEvent> {
    events(fx)
        .into_iter()
        .filter_map(|k| match k {
            Event_::Kind::Trade(t) => Some(t),
            _ => None,
        })
        .collect()
}

fn one_body(fx: &[Effect], seq: u32) -> Body {
    let r = responses(fx);
    assert_eq!(r.len(), 1, "exactly one response: {fx:?}");
    assert_eq!(r[0].seq, seq);
    r[0].body.clone().expect("body")
}

fn is_ack(fx: &[Effect], seq: u32) -> bool {
    matches!(one_body(fx, seq), Body::Ack(_))
}

fn error_code(fx: &[Effect], seq: u32) -> Option<ErrorCode> {
    match one_body(fx, seq) {
        Body::Error(e) => Some(e.code),
        _ => None,
    }
}

fn invite_cmd(peer: &str) -> Cmd {
    let mut p = String::new();
    p.push_str(peer).ok();
    Cmd::TradeInvite(TradeInvite { peer: p })
}

/// The OZ body (kind byte onward) of a Beacon's advertising data.
fn ble_input(ad: &[u8], rssi: i8) -> Input {
    let body = oz_payload(ad).expect("OZ advert");
    Input::RadioPacket {
        via: Via::Ble,
        rssi,
        payload: Vec::from_slice(body).expect("fits"),
    }
}

/// A lobby packet as a BLE radio input, signed with `key`.
fn lobby_input(text: &str, key: Option<&RadioKey>, rssi: i8) -> Input {
    let signed: Vec<u8, ADVERT_MAX> = sign::sign(key, text.as_bytes()).expect("fits");
    let ad = adv::encode(&signed).expect("fits");
    ble_input(&ad, rssi)
}

/// One simulated badge: the app plus what its radio is doing.
struct Badge {
    app: App,
    adv: Option<Vec<u8, MAX_ADV_DATA>>,
    link: link::Decoder,
    fx: StdVec<Effect>,
}

impl Badge {
    fn new(mac: [u8; 6], key: Option<RadioKey>) -> Badge {
        Badge {
            app: keyed(app(mac), key),
            adv: None,
            link: link::Decoder::default(),
            fx: StdVec::new(),
        }
    }

    fn apply(&mut self, input: Input) -> StdVec<Effect> {
        let (next, fx) = self.app.clone().handle(input);
        assert!(fx.len() <= MAX_EFFECTS);
        assert_eq!(next.dropped_effects(), 0);
        self.app = next;
        self.fx.extend(fx.iter().cloned());
        fx.to_vec()
    }
}

/// One 20 ms step of `me`: tick, then hand its radio output to `other`.
fn step(me: &mut Badge, other: &mut Badge, t: u64, ble: bool, wire: bool) {
    let fx = me.apply(Input::Tick {
        now: Millis(t),
        buttons: ButtonMask(0),
    });
    let trade_fx = fx
        .iter()
        .filter(|e| matches!(e, Effect::Beacon(_) | Effect::LinkSend(_)))
        .count();
    assert!(trade_fx <= TRADE_TICK_EFFECTS);
    for e in fx {
        match e {
            Effect::Beacon(ad) => me.adv = Some(ad),
            Effect::LinkSend(frame) if wire => {
                for byte in frame {
                    if let Some(payload) = other.link.push(byte) {
                        other.apply(Input::RadioPacket {
                            via: Via::Link,
                            rssi: 0,
                            payload,
                        });
                    }
                }
            }
            _ => {}
        }
    }
    // The advertiser repeats the current advert; the peer hears it.
    if ble && let Some(ad) = me.adv.clone() {
        other.apply(ble_input(&ad, -42));
    }
}

/// Ticks both badges every 20 ms over `from..=to`. Each badge hears the
/// other's current advert every tick when `ble`, and its link frames when
/// `wire`.
fn run(a: &mut Badge, b: &mut Badge, from: u64, to: u64, ble: bool, wire: bool) {
    let mut t = from;
    while t <= to {
        step(a, b, t, ble, wire);
        step(b, a, t, ble, wire);
        t += 20;
    }
}

fn completed_trades(fx: &[Effect]) -> StdVec<TradeEvent> {
    trade_events(fx)
        .into_iter()
        .filter(|e| e.state == TradeState::Complete)
        .collect()
}

fn full_trade(key_a: Option<RadioKey>, key_b: Option<RadioKey>, ble: bool, wire: bool) {
    let mut a = Badge::new(MAC_A, key_a);
    let mut b = Badge::new(MAC_B, key_b);
    let (next, _) = host(b.app.clone(), 1, Cmd::SetChi(SetChi { chi: Chi::Cry }));
    b.app = next;
    run(&mut a, &mut b, 0, 1000, ble, wire);
    assert_eq!(a.app.trade().lobby_len(), 1);
    assert_eq!(b.app.trade().rows().first().map(|r| r.chi), Some(Chi::Vib));
    // Each side invites the other.
    a.apply(Input::Host(HostMsg {
        seq: 2,
        cmd: Some(invite_cmd("1a2b3c")),
    }));
    b.apply(Input::Host(HostMsg {
        seq: 2,
        cmd: Some(invite_cmd("0AC8F6")),
    }));
    run(&mut a, &mut b, 1020, 8000, ble, wire);
    for (me, other, got) in [(&a, "1A2B3C", Chi::Cry), (&b, "0AC8F6", Chi::Vib)] {
        let done = completed_trades(&me.fx);
        assert_eq!(done.len(), 1, "{:?}", trade_events(&me.fx));
        assert_eq!(done[0].peer.as_str(), other);
        assert_eq!(done[0].chi, got);
        let contact = me.app.contacts().get(id(other)).expect("contact saved");
        assert_eq!((contact.chi, contact.trades), (got, 1));
        assert!(me.fx.iter().any(|e| matches!(e, Effect::SaveContact(c) if c.id == id(other))));
        assert!(events(&me.fx).iter().any(|k| matches!(k, Event_::Kind::ContactAdded(_))));
        assert_eq!(me.app.trade().state(), TradeState::Complete);
        assert!(!me.app.trade().session().linked(), "left after the linger");
    }
}

#[test]
fn fr25_two_badges_trade_over_ble_signed() {
    full_trade(Some(FAKE), Some(FAKE), true, false);
}

#[test]
fn fr25_two_badges_trade_over_ble_unsigned() {
    full_trade(None, None, true, false);
}

#[test]
fn fr25_two_badges_trade_over_link_uart() {
    full_trade(Some(FAKE), Some(FAKE), false, true);
}

#[test]
fn fr27_keyed_badge_ignores_unsigned_and_foreign_keys() {
    for other in [None, Some(RadioKey([0x22; 32]))] {
        let mut a = Badge::new(MAC_A, Some(FAKE));
        let mut b = Badge::new(MAC_B, other);
        run(&mut a, &mut b, 0, 1000, true, true);
        assert_eq!(a.app.trade().lobby_len(), 0);
    }
}

#[test]
fn fr25_first_tick_sets_31_byte_signed_advert_and_link_frame() {
    let a = keyed(app(MAC_A), Some(FAKE));
    let (a, fx) = tick(a, 0, 0);
    let ad = fx
        .iter()
        .find_map(|e| match e {
            Effect::Beacon(ad) => Some(ad.clone()),
            _ => None,
        })
        .expect("beacon");
    let mut want = std::vec![30u8, 0xFF, b'O', b'Z', 0];
    want.extend_from_slice(b"OZGT:0AC8F6:------:~VIB");
    want.extend_from_slice(&[0x65, 0xb9, 0x13]);
    assert_eq!(&ad[..], &want[..]);
    let frame = fx
        .iter()
        .find_map(|e| match e {
            Effect::LinkSend(f) => Some(f.clone()),
            _ => None,
        })
        .expect("link frame");
    assert_eq!(&frame[..3], &[b'O', b'Z', 26]);
    assert_eq!(&frame[3..29], &want[5..]);
    // Unchanged advert: later beacons send only the link frame.
    let mut t = 20;
    let mut app = a;
    let mut beacons = 0;
    let mut frames = 0;
    while t < 3000 {
        let (next, fx) = tick(app, t, 0);
        app = next;
        beacons += fx.iter().filter(|e| matches!(e, Effect::Beacon(_))).count();
        frames += fx.iter().filter(|e| matches!(e, Effect::LinkSend(_))).count();
        t += 20;
    }
    assert_eq!(beacons, 0);
    assert!((6..=10).contains(&frames), "{frames}");
}

#[test]
fn fr25_ble_rssi_gate_is_minus_55() {
    let a = app(MAC_A);
    let (a, _) = tick(a, 0, 0);
    let (a, _) = a.handle(lobby_input("OZGT:1A2B3C:------:~HCK", None, -56));
    assert_eq!(a.trade().lobby_len(), 0);
    let (a, _) = a.handle(lobby_input("OZGT:1A2B3C:------:~HCK", None, -55));
    assert_eq!(a.trade().lobby_len(), 1);
    // Link packets have no gate.
    let (a, _) = a.handle(Input::RadioPacket {
        via: Via::Link,
        rssi: -100,
        payload: Vec::from_slice(b"OZGT:2A2B3C:------:~ADM").expect("fits"),
    });
    assert_eq!(a.trade().lobby_len(), 2);
    let rows = a.trade().rows();
    assert_eq!(rows.iter().find(|r| r.id == id("2A2B3C")).map(|r| r.via), Some(Via::Link));
}

#[test]
fn fr26_overheard_profile_names_contact() {
    let a = keyed(app(MAC_A), Some(FAKE));
    let (a, _) = tick(a, 0, 0);
    let mut contacts = Contacts::new();
    contacts = contacts.record_trade(id("1A2B3C"), Chi::Hck, "").0;
    let a = restore(a, contacts, Some(FAKE))
        .0;
    // Profiles are heard across the room: no RSSI gate.
    let (a, fx) = a.handle(lobby_input("OZN1a2b3cADM07Ruffles", Some(&FAKE), -90));
    let saved: StdVec<&Contact> = fx
        .iter()
        .filter_map(|e| match e {
            Effect::SaveContact(c) => Some(c),
            _ => None,
        })
        .collect();
    assert_eq!(saved.len(), 1);
    assert_eq!(saved[0].name.as_str(), "Ruffles");
    assert_eq!(saved[0].trades, 1);
    // Same name again: nothing to save. Unsigned: ignored.
    let (a, fx) = a.handle(lobby_input("OZN1A2B3CADM07Ruffles", Some(&FAKE), -90));
    assert!(!fx.iter().any(|e| matches!(e, Effect::SaveContact(_))));
    let (a, fx) = a.handle(lobby_input("OZN1A2B3CADM07Mallory", None, -40));
    assert!(!fx.iter().any(|e| matches!(e, Effect::SaveContact(_))));
    assert_eq!(a.contacts().get(id("1A2B3C")).map(|c| c.name.as_str()), Some("Ruffles"));
}

#[test]
fn fr25_linking_packet_can_complete_in_one_call() {
    let a = app(MAC_A);
    let (a, _) = tick(a, 0, 0);
    let (a, _) = a.handle(lobby_input("OZN1A2B3CHCK03Bob", None, -40));
    let (a, _) = a.handle(lobby_input("OZGT:1A2B3C:------:~HCK", None, -40));
    let (a, _) = a.handle(lobby_input("OZN1A2B3CHCK03Bob", None, -40));
    let (a, _) = host(a, 1, invite_cmd("1A2B3C"));
    // We have not sent "got yours" yet, but the peer confirms ours: linked,
    // tag received and confirmed in one packet.
    let (a, fx) = a.handle(lobby_input("OZGT:1A2B3C:0AC8F6:HCK1", None, -40));
    let states: StdVec<TradeState> = trade_events(&fx).iter().map(|e| e.state).collect();
    assert_eq!(states, [TradeState::Exchanging, TradeState::Complete]);
    assert!(fx.len() <= MAX_EFFECTS);
    let c = a.contacts().get(id("1A2B3C")).expect("saved");
    assert_eq!(c.name.as_str(), "Bob");
    assert_eq!(c.chi, Chi::Hck);
}

#[test]
fn fr25_host_requests_answer_once() {
    let a = keyed(app(MAC_A), Some(FAKE));
    let (a, _) = tick(a, 0, 0);
    let (a, fx) = host(a, 1, Cmd::SetChi(SetChi { chi: Chi::Unspecified }));
    assert_eq!(error_code(&fx, 1), Some(ErrorCode::BadArgument));
    let (a, fx) = host(a, 2, Cmd::SetChi(SetChi { chi: Chi(77) }));
    assert_eq!(error_code(&fx, 2), Some(ErrorCode::BadArgument));
    let (a, fx) = host(a, 3, Cmd::SetChi(SetChi { chi: Chi::Pkt }));
    assert!(is_ack(&fx, 3));
    assert_eq!(a.trade().chi(), Chi::Pkt);
    for bad in ["", "12345", "1234567", "XYZXYZ"] {
        let (_, fx) = host(a.clone(), 4, invite_cmd(bad));
        assert_eq!(error_code(&fx, 4), Some(ErrorCode::BadArgument), "{bad}");
    }
    let (a, fx) = host(a, 5, invite_cmd("0ac8f6"));
    assert_eq!(error_code(&fx, 5), Some(ErrorCode::BadArgument));
    // Unknown peer: allowed, like stock.
    let (a, fx) = host(a, 6, invite_cmd("1a2b3c"));
    assert!(is_ack(&fx, 6));
    let ev = trade_events(&fx);
    assert_eq!(ev.len(), 1);
    assert_eq!((ev[0].peer.as_str(), ev[0].state), ("1A2B3C", TradeState::Inviting));
    let (a, fx) = host(a, 7, Cmd::GetLobby(GetLobby {}));
    match one_body(&fx, 7) {
        Body::Lobby(l) => {
            assert!(l.signed);
            assert_eq!(l.chi, Chi::Pkt);
            assert_eq!(l.state, TradeState::Inviting);
            assert_eq!(l.peer.as_str(), "1A2B3C");
            assert!(l.peers.is_empty());
        }
        other => panic!("{other:?}"),
    }
    let (a, fx) = host(a, 8, Cmd::TradeCancel(TradeCancel {}));
    assert!(is_ack(&fx, 8));
    assert_eq!(trade_events(&fx).first().map(|e| e.state), Some(TradeState::Cancelled));
    // Cancel with nothing going on: just an ack.
    let (a, fx) = host(a, 9, Cmd::TradeCancel(TradeCancel {}));
    assert!(is_ack(&fx, 9));
    assert!(trade_events(&fx).is_empty());
    // Linked: invites and Chi changes are BUSY.
    let (a, _) = host(a, 10, invite_cmd("1A2B3C"));
    let (a, _) = a.handle(lobby_input("OZGT:1A2B3C:0AC8F6:~HCK", Some(&FAKE), -40));
    let (a, fx) = host(a, 11, invite_cmd("2A2B3C"));
    assert_eq!(error_code(&fx, 11), Some(ErrorCode::Busy));
    let (_, fx) = host(a, 12, Cmd::SetChi(SetChi { chi: Chi::Adm }));
    assert_eq!(error_code(&fx, 12), Some(ErrorCode::Busy));
}

#[test]
fn fr26_list_and_clear_contacts() {
    let mut contacts = Contacts::new();
    for n in 0..11u32 {
        contacts = contacts.record_trade(PeerId::from_u32(n), Chi::Vib, "").0;
    }
    let a = restore(app(MAC_A), contacts, None)
        .0;
    let (a, fx) = host(a, 1, Cmd::ListContacts(ListContacts { offset: 0 }));
    match one_body(&fx, 1) {
        Body::Contacts(c) => {
            assert_eq!((c.contacts.len(), c.total, c.offset), (8, 11, 0));
            assert_eq!(c.contacts[0].id.as_str(), "00000A");
        }
        other => panic!("{other:?}"),
    }
    let (a, fx) = host(a, 2, Cmd::ListContacts(ListContacts { offset: 8 }));
    match one_body(&fx, 2) {
        Body::Contacts(c) => assert_eq!((c.contacts.len(), c.total, c.offset), (3, 11, 8)),
        other => panic!("{other:?}"),
    }
    let (a, fx) = host(a, 3, Cmd::ListContacts(ListContacts { offset: u32::MAX }));
    match one_body(&fx, 3) {
        Body::Contacts(c) => assert_eq!((c.contacts.len(), c.total), (0, 11)),
        other => panic!("{other:?}"),
    }
    let (a, fx) = host(a, 4, Cmd::ClearContacts(ClearContacts {}));
    assert!(is_ack(&fx, 4));
    assert!(fx.contains(&Effect::ClearContacts));
    assert!(a.contacts().is_empty());
}

fn on_screen(app: App, screen: Screen) -> App {
    host(app, 1, Cmd::ShowScreen(ShowScreen { screen })).0
}

/// Presses `mask` at `t` (held 40 ms) and releases; returns all effects.
fn press(app: App, t: u64, mask: u8) -> (App, StdVec<Effect>) {
    let mut all = StdVec::new();
    let mut app = app;
    for (dt, m) in [(0, mask), (30, mask), (40, 0), (70, 0)] {
        let (next, fx) = tick(app, t + dt, m);
        all.extend(fx.iter().cloned());
        app = next;
    }
    (app, all)
}

#[test]
fn fr25_trade_buttons_pick_invite_cancel() {
    let a = app(MAC_A);
    let (a, _) = tick(a, 0, 0);
    let a = on_screen(a, Screen::Trade);
    let (a, _) = a.handle(lobby_input("OZGT:1A2B3C:------:~HCK", None, -40));
    let (a, _) = a.handle(lobby_input("OZGT:2A2B3C:------:~PKT", None, -40));
    // LEFT / RIGHT outside pick mode navigate.
    let (a, _) = press(a, 100, RIGHT);
    assert_eq!(a.screen(), Screen::Contacts);
    let (a, _) = press(a, 200, LEFT);
    assert_eq!(a.screen(), Screen::Trade);
    let (a, fx) = press(a, 300, SELECT);
    assert_eq!(a.trade().picking(), Some(0));
    assert!(fx.contains(&Effect::Redraw));
    let (a, _) = press(a, 400, RIGHT);
    assert_eq!(a.screen(), Screen::Trade);
    assert_eq!(a.trade().picking(), Some(1));
    let (a, _) = press(a, 500, RIGHT);
    assert_eq!(a.trade().picking(), Some(0), "wraps");
    let (a, _) = press(a, 600, LEFT);
    assert_eq!(a.trade().picking(), Some(1));
    let (a, fx) = press(a, 700, SELECT);
    assert_eq!(a.trade().picking(), None);
    assert_eq!(a.trade().state(), TradeState::Inviting);
    assert_eq!(a.trade().peer(), Some(id("2A2B3C")));
    assert_eq!(trade_events(&fx).len(), 1);
    // START outside pick mode cancels.
    let (a, fx) = press(a, 800, START);
    assert_eq!(a.trade().state(), TradeState::Cancelled);
    assert_eq!(trade_events(&fx).len(), 1);
    // START in pick mode backs out without inviting.
    let (a, _) = press(a, 900, SELECT);
    let (a, _) = press(a, 1000, START);
    assert_eq!(a.trade().picking(), None);
    assert_eq!(a.trade().state(), TradeState::Cancelled);
    // SELECT on the invited row cancels it (stock toggles).
    let (a, _) = host(a, 1, invite_cmd("1A2B3C"));
    let (a, _) = press(a, 1100, SELECT);
    let (a, _) = press(a, 1200, SELECT);
    assert_eq!(a.trade().state(), TradeState::Cancelled);
}

#[test]
fn fr26_contacts_select_pages() {
    let mut contacts = Contacts::new();
    for n in 0..13u32 {
        contacts = contacts.record_trade(PeerId::from_u32(n), Chi::Vib, "").0;
    }
    let a = restore(app(MAC_A), contacts, None)
        .0;
    let (a, _) = tick(a, 0, 0);
    let a = on_screen(a, Screen::Contacts);
    let mut pages = StdVec::new();
    let mut a = a;
    for i in 0..4 {
        let (next, fx) = press(a, 100 + i * 100, SELECT);
        assert!(fx.contains(&Effect::Redraw));
        a = next;
        pages.push(a.trade().contacts_page());
    }
    assert_eq!(pages, [1, 2, 0, 1]);
    let (a, _) = press(a, 600, LEFT);
    assert_eq!(a.screen(), Screen::Trade);
}

#[test]
fn fr25_trade_screen_does_not_redraw_every_tick() {
    let a = app(MAC_A);
    let (a, _) = tick(a, 0, 0);
    let a = on_screen(a, Screen::Trade);
    let (mut a, _) = tick(a, 20, 0);
    let mut redraws = 0;
    let mut t = 40;
    while t < 5000 {
        let (next, fx) = tick(a, t, 0);
        redraws += fx.iter().filter(|e| **e == Effect::Redraw).count();
        a = next;
        t += 20;
    }
    assert_eq!(redraws, 0);
    // A new badge redraws once, its expiry 4 s later once more.
    let (a, fx) = a.handle(lobby_input("OZGT:1A2B3C:------:~HCK", None, -42));
    assert!(fx.contains(&Effect::Redraw));
    let (a, fx) = a.handle(lobby_input("OZGT:1A2B3C:------:~HCK", None, -44));
    assert!(!fx.contains(&Effect::Redraw), "same 8 dB step");
    let mut a = a;
    let mut redraws = 0;
    let mut t = 5000;
    while t < 10_000 {
        let (next, fx) = tick(a, t, 0);
        redraws += fx.iter().filter(|e| **e == Effect::Redraw).count();
        a = next;
        t += 20;
    }
    assert_eq!(redraws, 1);
    assert_eq!(a.trade().lobby_len(), 0);
}

/// Every new radio and host input, in states with a full lobby, an invite,
/// a link and a pending completion.
fn trade_inputs() -> StdVec<Input> {
    let mut inputs = StdVec::new();
    for text in [
        "OZGT:1A2B3C:0AC8F6:HCK1",
        "OZGT:1A2B3C:0AC8F6:~HCK",
        "OZGT:1A2B3C:------:~HCK",
        "OZGT:1A2B3C:0AC8F6:ZZZ1",
        "OZGT:FFFFFF:0AC8F6:~OVR",
        "OZN1A2B3CADM07Ruffles",
        "OZGT:bad",
    ] {
        for key in [None, Some(&FAKE)] {
            inputs.push(lobby_input(text, key, -40));
            let signed: Vec<u8, ADVERT_MAX> = sign::sign(key, text.as_bytes()).expect("fits");
            inputs.push(Input::RadioPacket {
                via: Via::Link,
                rssi: 0,
                payload: Vec::from_slice(&signed).expect("fits"),
            });
        }
    }
    inputs.push(Input::RadioPacket {
        via: Via::Ble,
        rssi: -40,
        payload: Vec::from_slice(&[9; 30]).expect("fits"),
    });
    inputs.push(Input::RadioPacket {
        via: Via::Ble,
        rssi: -40,
        payload: Vec::new(),
    });
    let mut contacts = Contacts::new();
    for n in 0..64u32 {
        contacts = contacts.record_trade(PeerId::from_u32(n), Chi::Vib, "NNNNNNNN").0;
    }
    for c in contacts.as_slice() {
        inputs.push(Input::Storage(StorageEvent::Contact(c.clone())));
    }
    inputs.push(Input::Storage(StorageEvent::Loaded { key: Some(FAKE) }));
    for (seq, cmd) in [
        Cmd::SetChi(SetChi { chi: Chi::Hck }),
        Cmd::SetChi(SetChi { chi: Chi::Unspecified }),
        invite_cmd("1A2B3C"),
        invite_cmd("nope"),
        Cmd::TradeCancel(TradeCancel {}),
        Cmd::ListContacts(ListContacts { offset: 0 }),
        Cmd::ClearContacts(ClearContacts {}),
        Cmd::GetLobby(GetLobby {}),
    ]
    .into_iter()
    .enumerate()
    {
        inputs.push(Input::Host(HostMsg {
            seq: u32::try_from(seq).expect("small"),
            cmd: Some(cmd),
        }));
    }
    inputs
}

/// Start states: full lobby, inviting, linked, about to complete, lingering.
fn trade_states() -> StdVec<App> {
    let base = keyed(app(MAC_A), None);
    let (mut full, _) = tick(base, 0, 0);
    for n in 0..9u32 {
        let text = std::format!("OZGT:{:06X}:0AC8F6:~VIB", 0x100000 + n);
        full = full.handle(lobby_input(&text, None, -40)).0;
    }
    let inviting = host(full.clone(), 1, invite_cmd("1A2B3C")).0;
    let linked = inviting
        .clone()
        .handle(lobby_input("OZGT:1A2B3C:0AC8F6:~HCK", None, -40))
        .0;
    let got = linked
        .clone()
        .handle(lobby_input("OZGT:1A2B3C:0AC8F6:HCK0", None, -40))
        .0;
    let lingering = got
        .clone()
        .handle(lobby_input("OZGT:1A2B3C:0AC8F6:HCK1", None, -40))
        .0;
    std::vec![full, inviting, linked, got, lingering]
}

#[test]
fn fr23_trade_effects_never_exceed_bound() {
    let masks = [0u8, LEFT, RIGHT, SELECT, START, LEFT | SELECT, 0b0011_1110];
    let mut checked = 0;
    for start in trade_states() {
        for screen in [Screen::Trade, Screen::Contacts, Screen::Pet] {
            let start = on_screen(start.clone(), screen);
            for input in trade_inputs() {
                let (next, fx) = start.clone().handle(input.clone());
                assert!(fx.len() <= MAX_EFFECTS, "{input:?} -> {fx:?}");
                assert_eq!(next.dropped_effects(), 0);
                let display = fx.iter().filter(|e| matches!(e, Effect::Redraw | Effect::Animate)).count();
                assert!(display <= 1);
                checked += 1;
            }
            for &mask in &masks {
                let mut app = start.clone();
                // Across the 6 s lost timer and the 20 s invite timer too.
                for base in [100u64, 6_000, 20_000] {
                    for (dt, m) in [(0, mask), (30, mask), (50, mask), (100, 0), (130, 0), (150, 0)] {
                        let (next, fx) = tick(app, base + dt, m);
                        assert!(fx.len() <= MAX_EFFECTS, "{mask:#x} {fx:?}");
                        assert_eq!(next.dropped_effects(), 0);
                        let radio = fx
                            .iter()
                            .filter(|e| {
                                matches!(
                                    e,
                                    Effect::Beacon(_) | Effect::LinkSend(_) | Effect::SaveContact(_)
                                )
                            })
                            .count();
                        assert!(radio <= TRADE_TICK_EFFECTS);
                        app = next;
                        checked += 1;
                    }
                }
            }
        }
    }
    assert!(checked > 500);
    // The tick budget leaves room for our share.
    const { assert!(MAX_EDGES_PER_TICK * 2 + 4 + TRADE_TICK_EFFECTS <= MAX_EFFECTS) };
}

#[test]
fn fr25_lost_in_tick_after_chi_arrived_defers_beacon_but_completes() {
    let a = keyed(app(MAC_A), None);
    let (a, _) = tick(a, 0, 0);
    let (a, _) = host(a, 1, invite_cmd("1A2B3C"));
    let (a, _) = a.handle(lobby_input("OZGT:1A2B3C:0AC8F6:HCK0", None, -40));
    assert_eq!(a.trade().received(), Chi::Hck);
    // Silence for 6 s: the tick both completes (stock counts it) and would
    // beacon; the completion's 3 effects come first, the beacon next tick.
    let mut app = a;
    let mut t = 20;
    let mut done = StdVec::new();
    while t <= 6_200 {
        let (next, fx) = tick(app, t, 0);
        if !completed_trades(&fx).is_empty() {
            assert!(!fx.iter().any(|e| matches!(e, Effect::LinkSend(_))));
            done.push(t);
        }
        app = next;
        t += 20;
    }
    assert_eq!(done.len(), 1);
    assert_eq!(app.contacts().get(id("1A2B3C")).map(|c| c.chi), Some(Chi::Hck));
}
