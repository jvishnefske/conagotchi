//! Regression tests for the stock trade state machine (FR-25).

extern crate std;

use std::vec::Vec as StdVec;

use super::*;
use crate::oz::lobby::parse;

fn id(s: &str) -> PeerId {
    PeerId::parse(s.as_bytes()).expect("id")
}

const A: &str = "0AC8F6";
const B: &str = "1A2B3C";

/// Feeds `text` (a lobby packet) to `s` at `t` over BLE at -40 dBm.
fn hear(s: Session, t: u64, text: &str) -> (Session, Events) {
    let p = parse(text.as_bytes()).expect("lobby packet");
    s.on_packet(Millis(t), Via::Ble, -40, &p)
}

fn beacon_text(s: Session, t: u64) -> (Session, std::string::String) {
    let (s, b) = s.beacon(Millis(t));
    (s, std::string::String::from_utf8(b.to_vec()).expect("ascii"))
}

/// A session for A, linked with B (B invited us, we invited B).
fn linked() -> Session {
    let (s, r) = Session::new(id(A), Chi::Vib).invite(Millis(0), id(B));
    assert!(r.is_ok());
    let (s, ev) = hear(s, 10, "OZGT:1A2B3C:0AC8F6:~HCK");
    assert_eq!(ev.as_slice(), [Event { peer: id(B), state: TradeState::Exchanging, chi: Chi::Unspecified }]);
    assert!(s.linked());
    s
}

/// Runs two sessions against each other, every 20 ms from `from` to `to`,
/// delivering each beacon to the other; returns them and all events.
fn run(
    mut a: Session,
    mut b: Session,
    from: u64,
    to: u64,
) -> (Session, Session, StdVec<Event>, StdVec<Event>) {
    let (mut ea, mut eb) = (StdVec::new(), StdVec::new());
    let mut t = from;
    while t <= to {
        let now = Millis(t);
        let (na, ev) = a.tick(now);
        a = na;
        ea.extend(ev);
        let (nb, ev) = b.tick(now);
        b = nb;
        eb.extend(ev);
        if a.beacon_due(now) {
            let (na, pkt) = a.beacon(now);
            a = na;
            let p = parse(&pkt).expect("ours parses");
            let (nb, ev) = b.on_packet(now, Via::Ble, -45, &p);
            b = nb;
            eb.extend(ev);
        }
        if b.beacon_due(now) {
            let (nb, pkt) = b.beacon(now);
            b = nb;
            let p = parse(&pkt).expect("ours parses");
            let (na, ev) = a.on_packet(now, Via::Link, 0, &p);
            a = na;
            ea.extend(ev);
        }
        t += 20;
    }
    (a, b, ea, eb)
}

#[test]
fn fr25_two_sessions_trade_and_both_complete() {
    let a = Session::new(id(A), Chi::Vib);
    let b = Session::new(id(B), Chi::Hck);
    // Both in the lobby for a second: they list each other, nobody trades.
    let (a, b, ea, eb) = run(a, b, 0, 1000);
    assert!(ea.is_empty() && eb.is_empty());
    assert_eq!(a.lobby().players(Millis(1000)).first().map(|p| p.chi()), Some(Chi::Hck));
    assert_eq!(b.lobby().players(Millis(1000)).first().map(|p| p.chi()), Some(Chi::Vib));
    // Each invites the other.
    let (a, r) = a.invite(Millis(1000), id(B));
    assert!(r.is_ok());
    let (b, r) = b.invite(Millis(1200), id(A));
    assert!(r.is_ok());
    let (a, b, ea, eb) = run(a, b, 1000, 6000);
    let states = |ev: &[Event]| ev.iter().map(|e| e.state).collect::<StdVec<_>>();
    assert_eq!(states(&ea), [TradeState::Exchanging, TradeState::Complete]);
    assert_eq!(states(&eb), [TradeState::Exchanging, TradeState::Complete]);
    assert_eq!(ea.last().map(|e| (e.peer, e.chi)), Some((id(B), Chi::Hck)));
    assert_eq!(eb.last().map(|e| (e.peer, e.chi)), Some((id(A), Chi::Vib)));
    // Both lingered, left, and are back in the lobby; the result stays.
    for s in [&a, &b] {
        assert_eq!(s.state(), TradeState::Complete);
        assert_eq!(s.active_peer(), None);
        assert!(!s.linked());
    }
    assert_eq!(a.received(), Chi::Hck);
    assert_eq!(b.received(), Chi::Vib);
    let (_, text) = beacon_text(a, 6000);
    assert_eq!(text, "OZGT:0AC8F6:------:~VIB");
}

#[test]
fn fr25_flag_digits_drive_receive_and_confirm() {
    let s = linked();
    let (s, text) = beacon_text(s, 20);
    assert_eq!(text, "OZGT:0AC8F6:1A2B3C:VIB0");
    // Shorter than 4 characters: ignored.
    let (s, ev) = hear(s, 30, "OZGT:1A2B3C:0AC8F6:HCK");
    assert!(ev.is_empty());
    assert_eq!(s.received(), Chi::Unspecified);
    // Not a digit counts as 0: received, not confirmed; confirm goes out now.
    let (s, ev) = hear(s, 40, "OZGT:1A2B3C:0AC8F6:HCKx");
    assert!(ev.is_empty());
    assert_eq!(s.received(), Chi::Hck);
    assert!(s.beacon_due(Millis(40)));
    let (s, text) = beacon_text(s, 40);
    assert_eq!(text, "OZGT:0AC8F6:1A2B3C:VIB1");
    // Bit 0 set (3 = got yours + organizer): complete.
    let (s, ev) = hear(s, 50, "OZGT:1A2B3C:0AC8F6:HCK3");
    assert_eq!(ev.as_slice(), [Event { peer: id(B), state: TradeState::Complete, chi: Chi::Hck }]);
    // A repeat changes nothing.
    let (s, ev) = hear(s, 60, "OZGT:1A2B3C:0AC8F6:HCK1");
    assert!(ev.is_empty());
    assert_eq!(s.state(), TradeState::Complete);
}

#[test]
fn fr25_confirm_before_receive_completes_on_receive() {
    let s = linked();
    // The peer claims it has ours but its tag is "~" note: nothing yet.
    let (s, ev) = hear(s, 20, "OZGT:1A2B3C:0AC8F6:~HCK");
    assert!(ev.is_empty());
    let (s, ev) = hear(s, 30, "OZGT:1A2B3C:0AC8F6:PKT1");
    assert_eq!(ev.as_slice(), [Event { peer: id(B), state: TradeState::Complete, chi: Chi::Pkt }]);
    assert!(s.linked());
}

#[test]
fn fr25_unknown_tag_never_completes() {
    let s = linked();
    let (s, ev) = hear(s, 20, "OZGT:1A2B3C:0AC8F6:ZZZ1");
    assert!(ev.is_empty());
    assert!(s.unknown_tag());
    assert_eq!(s.state(), TradeState::Exchanging);
    let (mut s, text) = beacon_text(s, 30);
    assert_eq!(text, "OZGT:0AC8F6:1A2B3C:VIB0", "never says got yours");
    for t in (40..5000).step_by(100) {
        let (n, ev) = hear(s, t, "OZGT:1A2B3C:0AC8F6:CHI1");
        assert!(ev.is_empty());
        s = n;
    }
    assert_eq!(s.state(), TradeState::Exchanging);
    // Cancel gets out.
    let (s, ev) = s.cancel(Millis(5000));
    assert_eq!(ev.as_slice(), [Event { peer: id(B), state: TradeState::Cancelled, chi: Chi::Unspecified }]);
    assert!(!s.unknown_tag());
}

#[test]
fn fr25_linked_peer_lost_after_6s_of_silence() {
    let s = linked(); // last heard at 10
    let (s, ev) = s.tick(Millis(10 + LOST_MS - 1));
    assert!(ev.is_empty());
    let (s, ev) = s.tick(Millis(10 + LOST_MS));
    assert_eq!(ev.as_slice(), [Event { peer: id(B), state: TradeState::Lost, chi: Chi::Unspecified }]);
    assert_eq!(s.active_peer(), None);
    assert_eq!(s.peer(), Some(id(B)));
}

#[test]
fn fr25_peer_leaving_after_its_chi_arrived_counts_as_complete() {
    let s = linked();
    let (s, ev) = hear(s, 20, "OZGT:1A2B3C:0AC8F6:ADM0");
    assert!(ev.is_empty());
    // It went back to the lobby (addresses nobody): stock counts the trade.
    let (s, ev) = hear(s, 30, "OZGT:1A2B3C:------:~ADM");
    assert_eq!(ev.as_slice(), [Event { peer: id(B), state: TradeState::Complete, chi: Chi::Adm }]);
    assert_eq!(s.active_peer(), None);
    // Leaving before anything arrived is a lost link.
    let s = linked();
    let (_, ev) = hear(s, 30, "OZGT:1A2B3C:------:~ADM");
    assert_eq!(ev.as_slice(), [Event { peer: id(B), state: TradeState::Lost, chi: Chi::Unspecified }]);
}

#[test]
fn fr25_linger_1500ms_then_leave() {
    let s = linked();
    let (s, _) = hear(s, 20, "OZGT:1A2B3C:0AC8F6:CRY1");
    assert_eq!(s.state(), TradeState::Complete);
    let (s, ev) = s.tick(Millis(20 + LINGER_MS - 1));
    assert!(ev.is_empty());
    assert!(s.linked(), "still confirming");
    let (s, text) = beacon_text(s, 20 + LINGER_MS - 1);
    assert_eq!(text, "OZGT:0AC8F6:1A2B3C:VIB1");
    let (s, ev) = s.tick(Millis(20 + LINGER_MS));
    assert!(ev.is_empty(), "leaving after completing is not a new state");
    assert!(!s.linked());
    assert_eq!(s.state(), TradeState::Complete);
    assert!(s.beacon_due(Millis(20 + LINGER_MS)));
    let (s, text) = beacon_text(s, 20 + LINGER_MS);
    assert_eq!(text, "OZGT:0AC8F6:------:~VIB");
    // A leftover packet from the old peer changes nothing.
    let (s, ev) = hear(s, 1600, "OZGT:1A2B3C:0AC8F6:CRY1");
    assert!(ev.is_empty());
    assert_eq!(s.received(), Chi::Cry);
}

#[test]
fn fr25_invite_lapses_after_20s_and_rules() {
    let s = Session::new(id(A), Chi::Vib);
    let (s, r) = s.invite(Millis(100), id(A));
    assert_eq!(r, Err(InviteError::Myself));
    // Unknown (unheard) peers may be invited, like stock `invite`.
    let (s, r) = s.invite(Millis(100), id(B));
    assert_eq!(r.map(|e| e.len()), Ok(1));
    assert_eq!(s.state(), TradeState::Inviting);
    let (s, text) = beacon_text(s, 100);
    assert_eq!(text, "OZGT:0AC8F6:1A2B3C:~VIB");
    // Another badge addressing us does not link.
    let (s, ev) = hear(s, 200, "OZGT:FFFFFF:0AC8F6:~OVR");
    assert!(ev.is_empty());
    assert!(!s.linked());
    assert!(s.lobby().get(id("FFFFFF")).is_some_and(|p| p.wants()));
    // B in the lobby, not addressing us: still inviting, and no lapse reset.
    let (s, _) = hear(s, 300, "OZGT:1A2B3C:------:~HCK");
    let (s, ev) = s.tick(Millis(100 + INVITE_MS - 1));
    assert!(ev.is_empty());
    let (s, ev) = s.tick(Millis(100 + INVITE_MS));
    assert_eq!(ev.as_slice(), [Event { peer: id(B), state: TradeState::Lost, chi: Chi::Unspecified }]);
    // Replace an invite: one event for the new peer, none for the old one.
    let (s, _) = s.invite(Millis(30_000), id(B));
    let (s, r) = s.invite(Millis(30_001), id("FFFFFF"));
    assert_eq!(
        r.map(|e| e.to_vec()),
        Ok(std::vec![Event { peer: id("FFFFFF"), state: TradeState::Inviting, chi: Chi::Unspecified }])
    );
    // Linked: further invites and Chi changes are refused.
    let (s, _) = hear(s, 30_002, "OZGT:FFFFFF:0AC8F6:~OVR");
    assert!(s.linked());
    let (s, r) = s.invite(Millis(30_003), id(B));
    assert_eq!(r, Err(InviteError::Linked));
    let (_, r) = s.set_chi(Chi::Adm);
    assert_eq!(r, Err(ChiError::Linked));
}

#[test]
fn fr25_set_chi_and_ignored_packets() {
    let s = Session::new(id(A), Chi::Vib);
    let (s, r) = s.set_chi(Chi::Unspecified);
    assert_eq!(r, Err(ChiError::Unknown));
    let (s, r) = s.set_chi(Chi(99));
    assert_eq!(r, Err(ChiError::Unknown));
    let (s, r) = s.set_chi(Chi::Ovr);
    assert_eq!(r, Ok(()));
    let (s, text) = beacon_text(s, 0);
    assert_eq!(text, "OZGT:0AC8F6:------:~OVR");
    // Our own id and other games are ignored entirely.
    let (s, _) = hear(s, 10, "OZGT:0AC8F6:------:~HCK");
    let (s, _) = hear(s, 10, "OZGC:1A2B3C:------:~HCK");
    assert!(s.lobby().is_empty(Millis(10)));
    assert_eq!(Session::new(id(A), Chi::Unspecified).chi(), Chi::Vib);
}

#[test]
fn fr25_beacon_interval_is_300_to_450ms() {
    let mut s = Session::new(id(A), Chi::Vib);
    let mut seen = std::collections::BTreeSet::new();
    let mut last = None;
    for t in 0..200_000u64 {
        if s.beacon_due(Millis(t)) {
            if let Some(prev) = last {
                let gap = t - prev;
                assert!((BEACON_MS..=BEACON_MS + BEACON_JITTER_MS).contains(&gap), "{gap}");
                seen.insert(gap);
            }
            last = Some(t);
            s = s.beacon(Millis(t)).0;
        }
    }
    assert!(seen.len() > 50, "jitter spreads: {}", seen.len());
}
