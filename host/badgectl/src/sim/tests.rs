//! End-to-end simulator tests (FR-39): several badge-sim instances on the
//! in-process middleware bus, driven through the same request path as RTT.
//! Everything is deterministic and offline.

use badge_proto::{
    BadgeMsg_, Chi, Event_, HostMsg, HostMsg_::Cmd, Response_::Body, SetChi, TradeEvent, TradeInvite, TradeState,
};

use super::bus::LocalBus;
use super::guest::SimModule;
use super::link::{SimLink, StepClock};
use super::world::{World, WorldConfig};
use crate::session::{Conn, Observer, UpError};

/// Collects nothing (events are checked through the world's history).
struct Quiet;

impl Observer for Quiet {
    fn log_bytes(&mut self, _: &[u8]) {}
    fn event(&mut self, _: &badge_proto::Event) {}
    fn stray_response(&mut self, _: &badge_proto::Response) {}
    fn frame_error(&mut self, e: &UpError) {
        panic!("frame error {e}");
    }
}

fn world(badges: usize) -> World<LocalBus> {
    let module = SimModule::new(None).expect("badge-sim.wasm is embedded");
    World::new(&module, WorldConfig { badges, ..WorldConfig::default() }, LocalBus::new()).unwrap()
}

fn hs<const N: usize>(s: &str) -> heapless::String<N> {
    let mut out = heapless::String::new();
    out.push_str(s).unwrap();
    out
}

fn events(w: &World<LocalBus>, i: usize) -> Vec<Event_::Kind> {
    w.node(i)
        .unwrap()
        .history()
        .iter()
        .filter_map(|(_, k)| match k {
            BadgeMsg_::Kind::Event(e) => e.kind.clone(),
            BadgeMsg_::Kind::Response(_) => None,
        })
        .collect()
}

fn trades(w: &World<LocalBus>, i: usize) -> Vec<TradeEvent> {
    events(w, i)
        .into_iter()
        .filter_map(|k| match k {
            Event_::Kind::Trade(t) => Some(t),
            _ => None,
        })
        .collect()
}

#[test]
fn fr39_two_sim_badges_complete_a_chi_trade() {
    let mut w = world(2);
    let (id0, id1) = (w.node(0).unwrap().peer_id(), w.node(1).unwrap().peer_id());
    w.run_for(3000).unwrap();
    w.send_host(1, &HostMsg { seq: 1, cmd: Some(Cmd::SetChi(SetChi { chi: Chi::Cry })) }).unwrap();
    w.run_for(1000).unwrap();
    // Badge 0 invites badge 1 through the RTT request path (Conn over SimLink).
    let invite = HostMsg { seq: 77, cmd: Some(Cmd::TradeInvite(TradeInvite { peer: hs(id1.as_str()) })) };
    let resp = {
        let mut conn = Conn::new(SimLink::new(&mut w, 0).unwrap(), StepClock::new(20));
        conn.request(&invite, 5000, &mut Quiet).unwrap()
    };
    assert_eq!(resp.seq, 77);
    assert!(matches!(resp.body, Some(Body::Ack(_))), "{resp:?}");
    // The person at badge 1 accepts by inviting back (stock rule: both invite).
    w.send_host(1, &HostMsg { seq: 2, cmd: Some(Cmd::TradeInvite(TradeInvite { peer: hs(id0.as_str()) })) })
        .unwrap();
    w.run_for(10_000).unwrap();
    for (me, other, got) in [(0, id1, Chi::Cry), (1, id0, Chi::Vib)] {
        let done: Vec<_> = trades(&w, me).into_iter().filter(|t| t.state == TradeState::Complete).collect();
        assert_eq!(done.len(), 1, "badge {me}: {:?}", trades(&w, me));
        assert_eq!((done[0].peer.as_str(), done[0].chi), (other.as_str(), got));
        assert!(
            events(&w, me).iter().any(
                |k| matches!(k, Event_::Kind::ContactAdded(c) if c.contact.as_ref().is_some_and(|c| c.id.as_str() == other.as_str()))
            ),
            "badge {me} ContactAdded"
        );
    }
}

#[test]
fn fr39_two_sim_badges_converge_led_sync() {
    let mut w = world(2);
    // Badge 1 boots 1234 ms later, so their local clocks differ.
    w.run_for(1500).unwrap();
    w.run_for(8000).unwrap();
    for _ in 0..50 {
        w.step().unwrap();
        let (a, b) = (w.node(0).unwrap().leds(), w.node(1).unwrap().leds());
        assert!(a.is_some());
        assert_eq!(a, b, "at {} ms", w.now_ms());
    }
    // Both agree on the leader through GetSync (request path on each).
    let mut leaders = Vec::new();
    for i in 0..2 {
        let msg = HostMsg { seq: 5, cmd: Some(Cmd::GetSync(badge_proto::GetSync {})) };
        let mut conn = Conn::new(SimLink::new(&mut w, i).unwrap(), StepClock::new(20));
        let Some(Body::Sync(s)) = conn.request(&msg, 5000, &mut Quiet).unwrap().body else { panic!("sync") };
        assert_eq!(s.peers, 1);
        leaders.push(s.leader_id);
    }
    assert_eq!(leaders[0], leaders[1]);
}

#[test]
fn fr39_sim_is_deterministic() {
    let run = || {
        let mut w = world(3);
        w.run_for(4000).unwrap();
        (0..3).map(|i| (w.node(i).unwrap().leds(), w.node(i).unwrap().history().len())).collect::<Vec<_>>()
    };
    assert_eq!(run(), run());
}

#[test]
fn fr39_inputs_before_boot_are_refused() {
    let mut w = world(2);
    let msg = HostMsg { seq: 1, cmd: Some(Cmd::GetInfo(badge_proto::GetInfo {})) };
    assert!(w.send_host(1, &msg).is_err(), "badge 1 boots after the stagger");
    w.step().unwrap();
    assert!(w.send_host(0, &msg).is_ok());
}

#[test]
fn fr39_sim_get_info_and_unsupported_hardware_over_the_link() {
    let mut w = world(1);
    w.run_for(100).unwrap();
    let mut conn = Conn::new(SimLink::new(&mut w, 0).unwrap(), StepClock::new(20));
    let info = HostMsg { seq: 9, cmd: Some(Cmd::GetInfo(badge_proto::GetInfo {})) };
    let Some(Body::Info(i)) = conn.request(&info, 1000, &mut Quiet).unwrap().body else { panic!("info") };
    assert_eq!(i.mac.as_slice(), &[2, 0, 0xC0, 0xFF, 0xEE, 1]);
    assert!(conn.read_block(0).is_err(), "no PROBE block in the simulator");
    assert!(SimLink::new(&mut w, 3).is_err());
}

/// The demo app: every badge runs it, each publishes a counter on
/// `app/hello` every 2 s, and each logs "rx ..." for the other's samples.
#[test]
fn fr39_wasm_app_publish_reaches_the_other_badge() {
    let module = std::fs::read(concat!(env!("CARGO_MANIFEST_DIR"), "/../../apps/hello.wasm")).expect("apps/hello.wasm");
    let mut w = world(2);
    w.run_for(1300).unwrap();
    let mut seq = 100;
    for i in 0..2 {
        for msg in super::app_msgs(&module, &mut seq).unwrap() {
            w.send_host(i, &msg).unwrap();
        }
    }
    // The bus follows the apps' own subscriptions (made in `init`), before
    // any app has published (the first publish is 2 s after start).
    assert!(w.subscribed_app_keys().any(|k| k == "app/hello"));
    w.run_for(7000).unwrap();
    for i in 0..2 {
        let st = w.node(i).unwrap().wasm_status();
        assert_eq!(st.state, badge_proto::WasmState::Running, "badge {i}: {st:?}");
        let logs: Vec<String> = events(&w, i)
            .into_iter()
            .filter_map(|k| match k {
                Event_::Kind::WasmLog(l) => Some(l.text.as_str().to_string()),
                _ => None,
            })
            .collect();
        assert!(logs.iter().any(|l| l == "hello from wasm"), "badge {i}: {logs:?}");
        assert!(logs.iter().any(|l| l.starts_with("rx")), "badge {i} heard the other app: {logs:?}");
    }
}

/// The shipped demo scenario parses, runs, and completes the trade.
#[test]
fn fr39_demo_scenario_completes_the_trade() {
    let text = include_str!("../../scenarios/trade.txt");
    let steps = super::script::parse(text).unwrap();
    let mut w = world(2);
    let mut scenario = super::Scenario::new(&steps, &w).unwrap();
    let mut seq = 1;
    while w.now_ms() < 10_000 {
        scenario.run_due(&mut w, &mut seq).unwrap();
        w.step().unwrap();
    }
    assert!(scenario.finished());
    for i in 0..2 {
        assert!(trades(&w, i).iter().any(|t| t.state == TradeState::Complete), "badge {i}");
        let syncs = w
            .node(i)
            .unwrap()
            .history()
            .iter()
            .filter(|(_, k)| matches!(k, BadgeMsg_::Kind::Response(r) if matches!(r.body, Some(Body::Sync(_)))))
            .count();
        assert_eq!(syncs, 1, "badge {i} answered its sync request");
    }
}
