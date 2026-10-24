//! Regression tests for FR-31 LED timing sync: wire format, the pure
//! [`SyncClock`] rules, a deterministic multi-badge simulation (skewed
//! clocks, jitter and loss from a seeded LCG) and an app-level LED check.

extern crate std;

use std::vec::Vec as StdVec;

use super::*;
use crate::app::{App, BoardInfo};
use crate::io::{ButtonMask, Effect, Input};

/// Seeded linear congruential generator (Knuth MMIX constants).
struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> u32 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 33) as u32
    }

    /// Uniform in `0..n`.
    fn below(&mut self, n: u32) -> u32 {
        self.next() % n
    }
}

/// One simulated badge.
struct Badge {
    clock: SyncClock,
    /// True time at which it booted.
    boot: u64,
    /// Local time at boot (non-zero to exercise wraparound).
    base: u64,
    /// Clock error in parts per million.
    ppm: i64,
    /// Transmits and receives while true.
    alive: bool,
}

impl Badge {
    fn new(id: u32, boot: u64, base: u64, ppm: i64) -> Badge {
        Badge {
            clock: SyncClock::new(id),
            boot,
            base,
            ppm,
            alive: true,
        }
    }

    fn booted(&self, t: u64) -> bool {
        self.alive && t >= self.boot
    }

    /// Local time at true time `t` (only meaningful once booted).
    fn local(&self, t: u64) -> Millis {
        let elapsed = t.saturating_sub(self.boot);
        let scaled = elapsed as i64 * (1_000_000 + self.ppm) / 1_000_000;
        Millis(self.base + scaled as u64)
    }

    fn net(&self, t: u64) -> NetMs {
        self.clock.network_ms(self.local(t))
    }
}

/// Simulation step (the firmware core ticks every 20 ms).
const STEP_MS: u64 = 20;

/// A world of badges with lossy, jittery delivery.
struct World {
    badges: StdVec<Badge>,
    rng: Lcg,
    /// Percentage of beacons lost per receiver.
    loss_pct: u32,
    /// Maximum extra delivery delay beyond [`RX_LATENCY_MS`] (either sign).
    jitter_ms: u32,
    /// True time.
    t: u64,
}

impl World {
    fn new(badges: StdVec<Badge>, seed: u64, loss_pct: u32, jitter_ms: u32) -> World {
        World {
            badges,
            rng: Lcg(seed),
            loss_pct,
            jitter_ms,
            t: 0,
        }
    }

    /// Advances one step: every live badge ticks, beacons are delivered.
    fn step(&mut self) {
        let t = self.t;
        let mut sent = StdVec::new();
        for (i, b) in self.badges.iter_mut().enumerate() {
            if b.booted(t) {
                let (clock, beacon) = b.clock.tick(b.local(t));
                b.clock = clock;
                if let Some(beacon) = beacon {
                    sent.push((i, beacon));
                }
            }
        }
        for (from, beacon) in sent {
            let encoded = beacon.encode();
            for (i, b) in self.badges.iter_mut().enumerate() {
                if i == from || !b.booted(t) || self.rng.below(100) < self.loss_pct {
                    continue;
                }
                let spread = 2 * self.jitter_ms + 1;
                let delay =
                    u64::from(RX_LATENCY_MS + self.rng.below(spread)) - u64::from(self.jitter_ms);
                let Ok(rx) = SyncBeacon::decode(&encoded) else {
                    continue;
                };
                b.clock = b.clock.on_beacon(rx.id, rx.network_ms, b.local(t + delay));
            }
        }
        self.t += STEP_MS;
    }

    fn run_until(&mut self, t: u64) {
        while self.t < t {
            self.step();
        }
    }

    /// Largest pairwise network-time difference among live, booted badges.
    fn spread(&self) -> u32 {
        let live: StdVec<&Badge> = self.badges.iter().filter(|b| b.booted(self.t)).collect();
        let mut worst = 0;
        for a in &live {
            for b in &live {
                worst = worst.max(a.net(self.t).since(b.net(self.t)).unsigned_abs());
            }
        }
        worst
    }
}

/// Three badges, ±200 ppm, different boot times, 20% loss, ±2 ms jitter.
fn three_badges(seed: u64) -> World {
    World::new(
        std::vec![
            Badge::new(0x30, 0, 5_000, 200),
            Badge::new(0x10, 1_234, 90_000_000, -200),
            Badge::new(0x20, 7_777, 42, 50),
        ],
        seed,
        20,
        2,
    )
}

#[test]
fn fr31_beacon_roundtrip() {
    let b = SyncBeacon {
        id: 0x0102_0304,
        network_ms: 0xfffe_fdfc,
    };
    let wire = b.encode();
    assert_eq!(&wire[..4], b"OZL1");
    assert_eq!(wire.len(), BEACON_LEN);
    assert_eq!(&wire[4..8], &[4, 3, 2, 1]);
    assert_eq!(SyncBeacon::decode(&wire), Ok(b));
}

#[test]
fn fr31_beacon_rejects_bad_magic_and_length() {
    let mut wire = SyncBeacon {
        id: 1,
        network_ms: 2,
    }
    .encode();
    assert_eq!(SyncBeacon::decode(&wire[..11]), Err(BeaconError::BadLength));
    assert_eq!(SyncBeacon::decode(&[]), Err(BeaconError::BadLength));
    let mut long = [0u8; 13];
    long[..12].copy_from_slice(&wire);
    assert_eq!(SyncBeacon::decode(&long), Err(BeaconError::BadLength));
    wire[3] = b'2';
    assert_eq!(SyncBeacon::decode(&wire), Err(BeaconError::BadMagic));
}

#[test]
fn fr31_beacon_every_500_ms() {
    let mut clock = SyncClock::new(7);
    let mut at = StdVec::new();
    for now in (0..=2_000).step_by(20) {
        let (next, beacon) = clock.tick(Millis(now));
        clock = next;
        if let Some(b) = beacon {
            assert_eq!(b.id, 7);
            assert_eq!(b.network_ms, now as u32);
            at.push(now);
        }
    }
    assert_eq!(at, std::vec![0, 500, 1_000, 1_500, 2_000]);
    // A long gap sends one beacon, not a burst.
    let (clock, first) = clock.tick(Millis(10_000));
    let (_, second) = clock.tick(Millis(10_020));
    assert!(first.is_some() && second.is_none());
}

#[test]
fn fr31_netms_wraps() {
    let a = NetMs(u32::MAX - 4);
    let b = a.shifted(10);
    assert_eq!(b, NetMs(5));
    assert_eq!(b.since(a), 10);
    assert_eq!(a.since(b), -10);
    assert_eq!(NetMs::from_local(Millis(u64::from(u32::MAX) + 3)), NetMs(2));
}

/// A follower whose leader (id 1, past probation) is heard at local 2000.
fn following(error: i32) -> SyncClock {
    let rx = Millis(2_000);
    let clock = SyncClock::new(9)
        .on_beacon(1, 0, Millis(0))
        .on_beacon(1, 0, Millis(1_000));
    assert_eq!(clock.leader(), 1);
    let offset = clock.offset_ms();
    let ours = clock.network_ms(rx);
    let theirs = ours.shifted(error).shifted(-(RX_LATENCY_MS as i32));
    let next = clock.on_beacon(1, theirs.0, rx);
    SyncClock {
        offset_ms: next.offset_ms.wrapping_sub(offset),
        ..next
    }
}

#[test]
fn fr31_step_on_large_error() {
    assert_eq!(following(10_000).offset_ms(), 10_000);
    assert_eq!(following(-501).offset_ms(), -501);
    assert_eq!(following(1_000_000_000).offset_ms(), 1_000_000_000);
}

#[test]
fn fr31_slew_is_bounded() {
    assert_eq!(following(500).offset_ms(), 20);
    assert_eq!(following(-300).offset_ms(), -20);
    assert_eq!(following(30).offset_ms(), 15);
    assert_eq!(following(-7).offset_ms(), -3);
    assert_eq!(following(0).offset_ms(), 0);
}

#[test]
fn fr31_ignores_own_beacons() {
    let clock = SyncClock::new(5);
    let next = clock.on_beacon(5, 123_456, Millis(10));
    assert_eq!(next, clock);
    assert_eq!(next.peers(), 0);
}

#[test]
fn fr31_only_leader_steers() {
    // Leader 1 established; beacons from 7 (higher than us, 5) do nothing.
    let clock = SyncClock::new(5)
        .on_beacon(1, 0, Millis(0))
        .on_beacon(1, 1_000, Millis(1_000));
    let before = clock.offset_ms();
    let after = clock.on_beacon(7, 400_000, Millis(1_100));
    assert_eq!(after.offset_ms(), before);
    assert_eq!(after.leader(), 1);
    assert_eq!(after.peers(), 2);
}

#[test]
fn fr31_lone_badge_adopts_higher_id_time() {
    // We (id 1) are alone; badge 9 belongs to an existing group.
    let clock = SyncClock::new(1).on_beacon(9, 50_000, Millis(1_000));
    let expect = 50_000 + RX_LATENCY_MS;
    assert_eq!(clock.network_ms(Millis(1_000)), NetMs(expect));
    // A lone badge hearing a lower id does not adopt; it follows once the
    // lower id is past probation.
    let clock = SyncClock::new(9).on_beacon(1, 50_000, Millis(1_000));
    assert_eq!(clock.offset_ms(), 0);
    assert_eq!(clock.leader(), 9);
}

#[test]
fn fr31_alone_keeps_offset() {
    let clock = following(10_000);
    let offset = clock.offset_ms();
    let (alone, _) = clock.tick(Millis(10_000));
    assert_eq!(alone.peers(), 0);
    assert_eq!(alone.leader(), 9);
    assert_eq!(alone.offset_ms(), offset);
}

#[test]
fn fr31_peer_table_keeps_lowest_ids() {
    let mut clock = SyncClock::new(1_000);
    for id in (100..112).rev() {
        clock = clock.on_beacon(id, 0, Millis(10));
    }
    assert_eq!(clock.peers(), MAX_PEERS as u32);
    // 108..=111 were evicted by lower ids; a higher newcomer is refused.
    clock = clock.on_beacon(500, 0, Millis(20));
    let ids: StdVec<u32> = clock.peers.iter().flatten().map(|p| p.id).collect();
    let mut sorted = ids.clone();
    sorted.sort_unstable();
    assert_eq!(sorted, (100..108).collect::<StdVec<u32>>());
    // After probation, the lowest leads.
    clock = clock.on_beacon(100, 0, Millis(1_100));
    assert_eq!(clock.leader(), 100);
}

#[test]
fn fr31_three_badges_converge() {
    for seed in 1..=5 {
        let mut w = three_badges(seed);
        w.run_until(30_000);
        assert!(w.spread() <= 10, "seed {seed}: spread {}", w.spread());
        for b in &w.badges {
            assert_eq!(b.clock.leader(), 0x10);
            assert_eq!(b.clock.peers(), 2);
        }
    }
}

#[test]
fn fr31_no_oscillation_after_convergence() {
    let mut w = three_badges(42);
    w.run_until(30_000);
    let mut worst = 0;
    while w.t < 630_000 {
        w.step();
        worst = worst.max(w.spread());
    }
    assert!(worst <= 10, "worst spread {worst}");
}

#[test]
fn fr31_leader_handover_without_jump() {
    let mut w = three_badges(7);
    w.run_until(30_000);
    // The leader (0x10, index 1) goes silent.
    w.badges[1].alive = false;
    let mut prev: StdVec<NetMs> = w.badges.iter().map(|b| b.net(w.t)).collect();
    while w.t < 60_000 {
        w.step();
        for (i, b) in w.badges.iter().enumerate().filter(|(_, b)| b.alive) {
            let now = b.net(w.t);
            let advance = now.since(prev[i]);
            // 20 ms per step, ±200 ppm, plus at most one bounded slew.
            assert!((0..=45).contains(&advance), "badge {i} jumped {advance}");
            prev[i] = now;
        }
        assert!(w.spread() <= 10, "spread {} at {}", w.spread(), w.t);
    }
    for b in w.badges.iter().filter(|b| b.alive) {
        assert_eq!(b.clock.leader(), 0x20);
        assert_eq!(b.clock.peers(), 1);
    }
}

#[test]
fn fr31_newcomer_with_lowest_id_does_not_jump_group() {
    let mut w = World::new(
        std::vec![
            Badge::new(0x30, 0, 0, 150),
            Badge::new(0x20, 3_000, 1_000_000, -150),
            Badge::new(0x01, 40_000, 0, 0),
        ],
        11,
        10,
        2,
    );
    w.run_until(35_000);
    assert!(w.spread() <= 10);
    let mut prev: StdVec<NetMs> = w.badges.iter().map(|b| b.net(w.t)).collect();
    while w.t < 80_000 {
        w.step();
        for (i, (b, p)) in w.badges.iter().zip(prev.iter_mut()).take(2).enumerate() {
            let now = b.net(w.t);
            let advance = now.since(*p);
            assert!((0..=45).contains(&advance), "badge {i} jumped {advance}");
            *p = now;
        }
    }
    assert!(w.spread() <= 10);
    for b in &w.badges {
        assert_eq!(b.clock.leader(), 0x01);
    }
}

#[test]
fn fr31_wraparound_at_u32_max() {
    // Local clocks straddle the u32 wrap: one starts just below it, the
    // other far away; network time crosses u32::MAX during the run.
    let near = u64::from(u32::MAX) - 20_000;
    let mut w = World::new(
        std::vec![
            Badge::new(2, 0, near, 200),
            Badge::new(1, 500, 3 * u64::from(u32::MAX) + 77, -200),
        ],
        3,
        20,
        2,
    );
    w.run_until(15_000);
    assert!(w.spread() <= 10, "before wrap: {}", w.spread());
    let mut worst = 0;
    while w.t < 60_000 {
        w.step();
        worst = worst.max(w.spread());
    }
    assert!(worst <= 10, "worst spread across wrap {worst}");
    // The follower's network time really wrapped.
    assert!(w.badges[0].local(w.t).0 > u64::from(u32::MAX));
}

/// An app whose station MAC ends in `id`.
fn app(id: u8) -> App {
    App::new(BoardInfo {
        mac: [0x24, 0x6f, 0x28, 0, 0, id],
        firmware: "badge-fw test",
    })
}

/// Ticks `app` at local `now`; returns it and any beacon it broadcasts.
fn tick(app: App, now: u64) -> (App, Option<(u32, u32)>) {
    let (app, fx) = app.handle(Input::Tick {
        now: Millis(now),
        buttons: ButtonMask(0),
    });
    let beacon = fx.iter().find_map(|e| match *e {
        Effect::SyncBroadcast { id, network_ms } => Some((id, network_ms)),
        _ => None,
    });
    (app, beacon)
}

#[test]
fn fr31_two_apps_render_identical_leds() {
    // Badge B boots 3337 ms after A; both tick every 20 ms of true time.
    let (boot_a, boot_b) = (0u64, 3_337u64);
    let (mut a, mut b) = (app(1), app(2));
    let mut same = 0;
    for t in (boot_b..40_000).step_by(20) {
        let (la, lb) = (t - boot_a, t - boot_b);
        let (na, ba) = tick(a, la);
        let (nb, bb) = tick(b, lb);
        a = na;
        b = nb;
        let latency = u64::from(RX_LATENCY_MS);
        if let Some((id, network_ms)) = ba {
            b = b
                .handle(Input::SyncBeacon {
                    from: id,
                    network_ms,
                    rx_at: Millis(lb + latency),
                })
                .0;
        }
        if let Some((id, network_ms)) = bb {
            a = a
                .handle(Input::SyncBeacon {
                    from: id,
                    network_ms,
                    rx_at: Millis(la + latency),
                })
                .0;
        }
        if t >= 10_000 {
            assert_eq!(a.leds(), b.leds(), "LEDs differ at true time {t}");
            same += 1;
        }
    }
    assert!(same > 1_000);
    assert_eq!(a.sync().leader(), 0x2800_0001);
    assert_eq!(b.sync().leader(), 0x2800_0001);
    assert_eq!(b.sync().peers(), 1);
}
