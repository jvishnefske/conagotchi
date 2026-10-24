//! Regression tests for [`Pet`] and the sprite data (FR-23, FR-24).

extern crate std;

use std::vec::Vec as StdVec;

use super::*;

/// Advances `pet` tick by tick (20 ms) from `from` to `to` inclusive and
/// returns it with every change seen, tagged with its time.
fn run(pet: Pet, from: u64, to: u64) -> (Pet, StdVec<(u64, Pet, PetChange)>) {
    let mut pet = pet;
    let mut seen = StdVec::new();
    for t in (from..=to).step_by(20) {
        let (next, change) = pet.advance(Millis(t));
        if change != PetChange::default() {
            seen.push((t, next, change));
        }
        pet = next;
    }
    (pet, seen)
}

#[test]
fn fr23_boot_state_is_auto_idle() {
    let pet = Pet::new();
    assert_eq!(pet, Pet::default());
    assert_eq!(
        (pet.selected(), pet.active()),
        (PetMode::Auto, PetMode::Idle)
    );
    assert_eq!(
        (pet.frame(), pet.offset(), pet.flipped()),
        (0, (0, 0), false)
    );
}

#[test]
fn fr23_frame_periods_per_mode() {
    for (mode, period) in [
        (PetMode::Idle, 400),
        (PetMode::Walk, 150),
        (PetMode::Eat, 250),
        (PetMode::Play, 120),
        (PetMode::Sleep, 700),
    ] {
        assert_eq!(frame_period(mode), period);
        let pet = Pet::new().select(mode, Millis(1000)).expect("valid");
        let count = u64::from(frame_count(mode));
        for k in 0..3 * count {
            let at = 1000 + k * period;
            let (p, _) = pet.advance(Millis(at));
            assert_eq!(
                u64::from(p.frame()),
                k % count,
                "{mode:?} at +{}",
                k * period
            );
            if period > 1 {
                let (p, _) = pet.advance(Millis(at + period - 1));
                assert_eq!(u64::from(p.frame()), k % count, "{mode:?} just before step");
            }
        }
    }
}

#[test]
fn fr23_frame_change_reported_only_when_picture_changes() {
    let pet = Pet::new().select(PetMode::Sleep, Millis(0)).expect("valid");
    let (_, seen) = run(pet, 0, 2799);
    let times: StdVec<u64> = seen.iter().map(|s| s.0).collect();
    assert_eq!(times, [700, 1400, 2100]);
    assert!(seen.iter().all(|s| s.2
        == PetChange {
            frame: true,
            mode: false
        }));
}

#[test]
fn fr23_auto_cycles_modes_in_order_with_dwell() {
    let (pet, seen) = run(Pet::new(), 0, 6 * AUTO_DWELL_MS);
    let switches: StdVec<(u64, PetMode)> = seen
        .iter()
        .filter(|s| s.2.mode)
        .map(|s| (s.0, s.1.active()))
        .collect();
    assert_eq!(
        switches,
        [
            (8000, PetMode::Walk),
            (16000, PetMode::Eat),
            (24000, PetMode::Play),
            (32000, PetMode::Sleep),
            (40000, PetMode::Idle),
            (48000, PetMode::Walk),
        ]
    );
    assert_eq!(pet.selected(), PetMode::Auto);
    // A mode switch restarts the animation, and is also a picture change.
    assert!(
        seen.iter()
            .filter(|s| s.2.mode)
            .all(|s| s.1.frame() == 0 && s.2.frame)
    );
}

#[test]
fn fr23_fixed_mode_never_moves_on() {
    let pet = Pet::new().select(PetMode::Eat, Millis(0)).expect("valid");
    let (pet, seen) = run(pet, 0, 3 * AUTO_DWELL_MS);
    assert!(seen.iter().all(|s| !s.2.mode));
    assert_eq!((pet.selected(), pet.active()), (PetMode::Eat, PetMode::Eat));
}

#[test]
fn fr23_next_mode_cycles_and_wraps() {
    let mut pet = Pet::new();
    let order = [
        PetMode::Idle,
        PetMode::Walk,
        PetMode::Eat,
        PetMode::Play,
        PetMode::Sleep,
        PetMode::Auto,
        PetMode::Idle,
    ];
    for (i, want) in order.iter().enumerate() {
        pet = pet.next_mode(Millis(100 * i as u64));
        assert_eq!(pet.selected(), *want);
        let active = if *want == PetMode::Auto {
            PetMode::Idle
        } else {
            *want
        };
        assert_eq!(pet.active(), active);
        assert_eq!(pet.frame(), 0, "animation restarts");
    }
}

#[test]
fn fr24_select_validates_and_auto_restarts_cycle() {
    let (pet, _) = Pet::new().advance(Millis(AUTO_DWELL_MS));
    assert_eq!(pet.active(), PetMode::Walk);
    let again = pet.select(PetMode::Auto, Millis(9000)).expect("valid");
    assert_eq!(
        (again.selected(), again.active()),
        (PetMode::Auto, PetMode::Idle)
    );
    // The dwell restarts from the select.
    assert_eq!(
        again.advance(Millis(9000 + AUTO_DWELL_MS - 1)).0.active(),
        PetMode::Idle
    );
    assert_eq!(
        again.advance(Millis(9000 + AUTO_DWELL_MS)).0.active(),
        PetMode::Walk
    );
    for bad in [PetMode(6), PetMode(-1), PetMode(i32::MAX)] {
        assert_eq!(pet.select(bad, Millis(0)), Err(InvalidMode(bad)));
        assert!(!is_valid(bad));
    }
    for mode in MODES {
        assert!(is_valid(mode));
    }
}

#[test]
fn fr23_clock_going_backwards_restarts_phase() {
    let pet = Pet::new()
        .select(PetMode::Walk, Millis(10_000))
        .expect("valid");
    let (pet, _) = pet.advance(Millis(10_450));
    assert_eq!(pet.frame(), 3);
    let (pet, change) = pet.advance(Millis(5));
    assert_eq!(pet.frame(), 0);
    assert!(change.frame && !change.mode);
    let (pet, _) = pet.advance(Millis(5 + 150));
    assert_eq!(pet.frame(), 1);
    // Far future: no overflow, no panic.
    let (pet, _) = pet.advance(Millis(u64::MAX));
    assert!(pet.frame() < frame_count(PetMode::Walk));
    // AUTO across a huge gap moves one mode per call.
    let (auto, change) = Pet::new().advance(Millis(u64::MAX));
    assert!(change.mode);
    assert_eq!(auto.active(), PetMode::Walk);
}

#[test]
fn fr23_offsets_stay_in_bounds_and_walk_bounces() {
    for mode in AUTO_ORDER {
        let pet = Pet::new().select(mode, Millis(0)).expect("valid");
        let period = frame_period(mode);
        let mut xs = StdVec::new();
        for k in 0..200u64 {
            let (p, _) = pet.advance(Millis(k * period));
            let (x, y) = p.offset();
            assert!((-DRIFT_MAX..=DRIFT_MAX).contains(&x), "{mode:?} x={x}");
            assert!((-BOUNCE_MAX..=0).contains(&y), "{mode:?} y={y}");
            xs.push((x, p.flipped()));
        }
        match mode {
            PetMode::Walk => {
                // Starts centred, reaches both edges, and faces the way it moves.
                assert_eq!(xs[0], (0, false));
                assert!(xs.iter().any(|&(x, _)| x == DRIFT_MAX));
                assert!(xs.iter().any(|&(x, _)| x == -DRIFT_MAX));
                for w in xs.windows(2) {
                    let (a, b) = (w[0].0, w[1].0);
                    assert!((a - b).abs() <= 2, "drift jumps {a} -> {b}");
                    // The cat faces the way it is about to move.
                    if b > a {
                        assert!(!w[0].1, "moving right but flipped");
                    }
                    if b < a {
                        assert!(w[0].1, "moving left but not flipped");
                    }
                }
            }
            PetMode::Play => {
                assert!(xs.iter().all(|&(x, f)| x == 0 && !f));
                let ys: StdVec<i16> = (0..4u64)
                    .map(|k| pet.advance(Millis(k * period)).0.offset().1)
                    .collect();
                assert!(ys.contains(&0) && ys.contains(&-BOUNCE_MAX));
            }
            _ => assert!(xs.iter().all(|&p| p == (0, false))),
        }
    }
}

// --------------------------------------------------------------- sprites

/// Every distinct frame referenced by the sequences.
fn all_frames() -> StdVec<&'static Frame> {
    AUTO_ORDER
        .iter()
        .flat_map(|&m| frames(m).iter().copied())
        .collect()
}

#[test]
fn fr23_frames_are_32x32_palette_only() {
    for (i, frame) in all_frames().into_iter().enumerate() {
        assert_eq!(frame.len(), usize::from(SPRITE));
        for (y, row) in frame.iter().enumerate() {
            assert_eq!(row.len(), usize::from(SPRITE), "frame {i} row {y}");
            for c in row.bytes() {
                assert!(
                    sprites::PALETTE.contains(&c),
                    "frame {i} row {y}: {:?}",
                    c as char
                );
            }
        }
        // Not blank.
        assert!(frame.iter().any(|r| r.bytes().any(|c| c != b'.')));
    }
}

#[test]
fn fr23_every_mode_has_distinct_frames() {
    for mode in AUTO_ORDER {
        let seq = frames(mode);
        assert!((2..=4).contains(&seq.len()), "{mode:?}");
        assert_eq!(usize::from(frame_count(mode)), seq.len());
        let distinct = seq
            .iter()
            .enumerate()
            .filter(|(i, f)| seq.iter().take(*i).all(|g| g != *f))
            .count();
        assert!(distinct >= 2, "{mode:?} has {distinct} distinct frames");
        // Consecutive frames differ, so every step is visible.
        for i in 0..seq.len() {
            assert_ne!(seq[i], seq[(i + 1) % seq.len()], "{mode:?} frame {i}");
        }
    }
    // Different modes look different.
    for (i, a) in AUTO_ORDER.iter().enumerate() {
        for b in AUTO_ORDER.iter().skip(i + 1) {
            assert_ne!(frames(*a)[0], frames(*b)[0], "{a:?} vs {b:?}");
        }
    }
}

#[test]
fn fr23_palette_is_small_and_used() {
    assert_eq!(palette(b'.'), None);
    let mut colors = StdVec::new();
    for &c in sprites::PALETTE.iter().skip(1) {
        let color = palette(c).expect("palette colour");
        assert!(!colors.contains(&color));
        colors.push(color);
        assert!(
            all_frames()
                .iter()
                .any(|f| f.iter().any(|r| r.as_bytes().contains(&c))),
            "{:?} unused",
            c as char
        );
    }
    assert_eq!(colors.len(), 6);
}

#[test]
fn fr23_pixel_lookup_matches_data_and_is_total() {
    let frame = sprites::IDLE[0];
    for y in 0..SPRITE {
        for x in 0..SPRITE {
            let c = frame[usize::from(y)].as_bytes()[usize::from(x)];
            assert_eq!(pixel(PetMode::Idle, 0, x, y), palette(c));
        }
    }
    // Out-of-range coordinates are transparent; frame indices wrap.
    assert_eq!(pixel(PetMode::Idle, 0, SPRITE, 0), None);
    assert_eq!(pixel(PetMode::Idle, 0, 0, SPRITE), None);
    assert_eq!(pixel(PetMode::Idle, 0, u8::MAX, u8::MAX), None);
    let n = frame_count(PetMode::Sleep);
    for (x, y) in [(10, 25), (20, 28)] {
        assert_eq!(
            pixel(PetMode::Sleep, n + 1, x, y),
            pixel(PetMode::Sleep, 1, x, y)
        );
    }
    // AUTO and unknown modes fall back to IDLE art.
    assert_eq!(
        pixel(PetMode::Auto, 0, 15, 15),
        pixel(PetMode::Idle, 0, 15, 15)
    );
    assert_eq!(
        pixel(PetMode(42), 0, 15, 15),
        pixel(PetMode::Idle, 0, 15, 15)
    );
}
