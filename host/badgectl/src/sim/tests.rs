//! End-to-end simulator tests (FR-39): several badge-sim instances on the
//! in-process middleware bus, driven through the same request path as RTT,
//! some running wasm payloads on middleware's wasm-host. Everything is
//! deterministic and offline.

use badge_core::color::Color;
use badge_proto::{
    BadgeMsg_, Chi, Event_, HostMsg, HostMsg_::Cmd, Response_::Body, SetChi, TradeEvent, TradeInvite, TradeState,
    WasmState,
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

/// The demo payload's tick period (apps/hello `STEP_MS`).
const STEP_MS: u32 = 100;

fn hello() -> Vec<u8> {
    std::fs::read(concat!(env!("CARGO_MANIFEST_DIR"), "/../../apps/hello.wasm")).expect("apps/hello.wasm")
}

/// Uploads `module` to badge `i` and starts it, through the core (the
/// `WasmBegin` / `WasmChunk` / `WasmCommit` / `WasmStart` requests).
fn install(w: &mut World<LocalBus>, i: usize, module: &[u8], seq: &mut u32) {
    for msg in super::app_msgs(module, seq).unwrap() {
        w.send_host(i, &msg).unwrap();
    }
}

fn logs(w: &World<LocalBus>, i: usize) -> Vec<String> {
    events(w, i)
        .into_iter()
        .filter_map(|k| match k {
            Event_::Kind::WasmLog(l) => Some(l.text.as_str().to_string()),
            _ => None,
        })
        .collect()
}

/// Responses with a `WasmStatus` body, in order.
fn wasm_answers(w: &World<LocalBus>, i: usize) -> Vec<badge_proto::WasmStatus> {
    w.node(i)
        .unwrap()
        .history()
        .iter()
        .filter_map(|(_, k)| match k {
            BadgeMsg_::Kind::Response(r) => match &r.body {
                Some(Body::Wasm(s)) => Some(s.clone()),
                _ => None,
            },
            BadgeMsg_::Kind::Event(_) => None,
        })
        .collect()
}

/// The demo payload: every badge runs it, each publishes a counter on
/// `app/hello` every 2 s, and each logs "rx ..." for the other's samples.
#[test]
fn fr39_wasm_app_publish_reaches_the_other_badge() {
    let module = hello();
    let mut w = world(2);
    w.run_for(1300).unwrap();
    let mut seq = 100;
    for i in 0..2 {
        install(&mut w, i, &module, &mut seq);
    }
    w.run_for(7000).unwrap();
    for i in 0..2 {
        let st = w.node(i).unwrap().wasm_status();
        assert_eq!(st.state, WasmState::Running, "badge {i}: {st:?}");
        assert!(st.ticks > 0, "badge {i}: {st:?}");
        let logs = logs(&w, i);
        assert!(logs.iter().any(|l| l == "hello from wasm"), "badge {i}: {logs:?}");
        assert!(logs.iter().any(|l| l.starts_with("rx ")), "badge {i} heard the other app: {logs:?}");
        // Every upload step and the start were answered once each, all Ok.
        let answers = wasm_answers(&w, i);
        let steps = module.len().div_ceil(badge_core::io::WASM_CHUNK) + 3;
        assert_eq!(answers.len(), steps, "badge {i}");
        assert_eq!(answers.last().map(|s| s.state), Some(WasmState::Running));
    }
}

/// `badge/leds` from the payload becomes `SetLeds` on the badge: the demo's
/// moving dot shows on the LEDs.
#[test]
fn fr39_payload_badge_leds_drive_leds() {
    let mut w = world(1);
    w.run_for(100).unwrap();
    install(&mut w, 0, &hello(), &mut 1);
    // The first tick (and LED frame) is due one period after init.
    w.run_for(u64::from(STEP_MS)).unwrap();
    let mut lit = std::collections::BTreeSet::new();
    for _ in 0..60 {
        w.step().unwrap();
        let leds = w.node(0).unwrap().leds().expect("leds shown");
        let on: Vec<usize> = (0..leds.len()).filter(|&n| leds[n] != Color::BLACK).collect();
        assert_eq!(on.len(), 1, "one dot at {} ms: {leds:?}", w.now_ms());
        let first = on.first().copied().unwrap_or_default();
        // Dim cyan, scaled by the badge's LED brightness.
        let c = leds[first];
        assert!(c.r == 0 && c.g == c.b && c.g > 0, "{c:?}");
        lit.insert(first);
    }
    assert_eq!(lit.len(), 8, "the dot moved along the whole chain");
    // `badge/text` at init reached the HOST screen request path too.
    assert!(
        w.node(0).unwrap().history().iter().any(
            |(_, k)| matches!(k, BadgeMsg_::Kind::Response(r) if r.seq == badge_core::probe_layout::WASM_SEQ)
        ),
        "the core answered the payload's badge/* requests"
    );
}

/// Button edges reach a payload subscribed to `badge/buttons`.
#[test]
fn fr39_payload_receives_button_edges() {
    let mut w = world(1);
    w.run_for(100).unwrap();
    install(&mut w, 0, &hello(), &mut 1);
    w.run_for(100).unwrap();
    let select = u8::try_from(badge_proto::Button::Select.0).unwrap();
    w.set_buttons(0, 1 << select).unwrap();
    w.run_for(200).unwrap();
    w.set_buttons(0, 0).unwrap();
    w.run_for(200).unwrap();
    let logs = logs(&w, 0);
    let want_down = format!("button {select} down");
    let want_up = format!("button {select} up");
    let down = logs.iter().position(|l| *l == want_down);
    let up = logs.iter().position(|l| *l == want_up);
    assert!(down.is_some() && up > down, "{logs:?}");
}

/// The payload store survives a simulated reboot and the installed payload
/// autostarts, like the firmware's persisted payload partition.
#[test]
fn fr39_payload_persists_across_sim_restart() {
    let module = hello();
    let mut w = world(1);
    w.run_for(100).unwrap();
    install(&mut w, 0, &module, &mut 1);
    w.run_for(500).unwrap();
    assert_eq!(w.node(0).unwrap().wasm_status().state, WasmState::Running);
    w.reboot(0).unwrap();
    assert_eq!(w.node(0).unwrap().wasm_status().state, WasmState::Stopped);
    assert!(!w.node(0).unwrap().booted());
    w.run_for(500).unwrap();
    let st = w.node(0).unwrap().wasm_status();
    assert_eq!((st.state, st.size), (WasmState::Running, u32::try_from(module.len()).unwrap()), "{st:?}");
    assert_eq!(w.node(0).unwrap().app().store().installed(), Some(module.as_slice()));
    assert_eq!(logs(&w, 0).iter().filter(|l| *l == "hello from wasm").count(), 2, "init ran again after the reboot");
}

/// middleware's echo example (payloads/examples/echo; here its WAT form,
/// templates/echo.wat, with 1-byte pages and 16 KiB of memory to fit the
/// badge) runs unchanged: what arrives on `app/echo/in` goes back out on
/// `app/echo/out` to the other badges.
#[test]
fn fr39_middleware_echo_example_runs() {
    const ECHO: &str = r#"(module
      (import "mw" "log"       (func $log (param i32 i32)))
      (import "mw" "publish"   (func $publish (param i32 i32 i32 i32) (result i32)))
      (import "mw" "subscribe" (func $subscribe (param i32 i32) (result i32)))
      (memory (export "memory") 16384 16384 (pagesize 1))
      (data (i32.const 16) "app/echo/in")
      (data (i32.const 32) "app/echo/out")
      (data (i32.const 48) "echo: started")
      (data (i32.const 64) "echo: message")
      (data (i32.const 80) "echo: subscribe failed")
      (global $rx i32 (i32.const 1024))
      (global $rx_len i32 (i32.const 512))
      (global $slot (mut i32) (i32.const -1))
      (func (export "init")
        (global.set $slot (call $subscribe (i32.const 16) (i32.const 11)))
        (if (i32.lt_s (global.get $slot) (i32.const 0))
          (then (call $log (i32.const 80) (i32.const 22)))
          (else (call $log (i32.const 48) (i32.const 13)))))
      (func (export "mw_alloc") (param $len i32) (result i32)
        (if (result i32) (i32.gt_u (local.get $len) (global.get $rx_len))
          (then (i32.const 0))
          (else (global.get $rx))))
      (func (export "on_message") (param $slot i32) (param $ptr i32) (param $len i32)
        (if (i32.ne (local.get $slot) (global.get $slot)) (then (return)))
        (call $log (i32.const 64) (i32.const 13))
        (drop (call $publish (i32.const 32) (i32.const 12) (local.get $ptr) (local.get $len)))))"#;
    // Badge 1 listens on app/echo/out with a WAT that logs what it hears.
    const LISTEN: &str = r#"(module
      (import "mw" "log"       (func $log (param i32 i32)))
      (import "mw" "subscribe" (func $subscribe (param i32 i32) (result i32)))
      (memory (export "memory") 2048 2048 (pagesize 1))
      (data (i32.const 16) "app/echo/out")
      (func (export "init") (drop (call $subscribe (i32.const 16) (i32.const 12))))
      (func (export "mw_alloc") (param i32) (result i32) (i32.const 1024))
      (func (export "on_message") (param i32 i32 i32) (call $log (local.get 1) (local.get 2))))"#;
    let mut w = world(2);
    w.run_for(1300).unwrap();
    let mut seq = 1;
    install(&mut w, 0, &wat::parse_str(ECHO).unwrap(), &mut seq);
    install(&mut w, 1, &wat::parse_str(LISTEN).unwrap(), &mut seq);
    w.publish_external("app/echo/in", b"ping").unwrap();
    w.run_for(100).unwrap();
    assert_eq!(logs(&w, 0), ["echo: started", "echo: message"]);
    assert_eq!(logs(&w, 1), ["ping"]);
    for i in 0..2 {
        assert_eq!(w.node(i).unwrap().wasm_status().state, WasmState::Running);
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
