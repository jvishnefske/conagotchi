//! BLE air metrics: what the continuous passive scan hears.
//!
//! The firmware's BLE task owns one [`BleMetrics`] collector and feeds it
//! every advertising report with [`BleMetrics::observe`]; once a second it
//! takes a [`BleMetrics::snapshot`] and sends it to the app as
//! `Input::BleStats`. Everything here is pure and bounded:
//!
//! - Report counts and the strongest RSSI live in [`WINDOW_S`] rolling
//!   one-second buckets, indexed by `second % WINDOW_S` and tagged with the
//!   second they hold, so a stale bucket is recognised without a sweep.
//! - Distinct addresses live in a fixed table of [`MAX_ADDRS`] entries with
//!   their last-seen time (and last time an OZ advert came from them). When
//!   the table is full, entries older than the window are dropped first; if
//!   every entry is still fresh, the least recently seen non-OZ entry is
//!   evicted (the least recently seen OZ entry if all are OZ), so OZ badges
//!   are kept preferentially. The unique count therefore saturates at
//!   [`MAX_ADDRS`].
//! - Time is the shell's monotonic clock. A report or snapshot stamped
//!   slightly earlier than the newest time seen (a reordering of up to one
//!   window) is treated as happening at the newest time. A jump backwards by
//!   more than the window (a clock reset or wrap) clears the collector and
//!   starts over from the new time. Nothing ever panics or overflows.

use crate::io::Millis;

/// Length of the rolling window, in seconds (and number of buckets).
pub const WINDOW_S: usize = 10;
/// Length of the rolling window, in milliseconds.
pub const WINDOW_MS: u64 = 1_000 * WINDOW_S as u64;
/// Distinct addresses tracked for the unique and OZ counts.
pub const MAX_ADDRS: usize = 64;

/// Metrics over the recent past, as shown on the BLE screen.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BleSnapshot {
    /// Advertisements heard in the last complete second.
    pub adv_per_s: u16,
    /// Distinct addresses heard in the last 10 s (at most [`MAX_ADDRS`]).
    pub unique_10s: u16,
    /// Distinct addresses with an OZ manufacturer AD in the last 10 s.
    pub oz_badges: u16,
    /// Strongest RSSI in the last complete second, dBm; 0 if nothing was heard.
    pub strongest_rssi: i8,
}

/// One second of reports.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Bucket {
    /// The second (`ms / 1000`) this bucket holds.
    second: u64,
    /// Reports heard in that second (saturating).
    count: u16,
    /// Strongest RSSI heard in that second; meaningless when `count == 0`.
    strongest: i8,
}

impl Bucket {
    const EMPTY: Bucket = Bucket {
        second: u64::MAX,
        count: 0,
        strongest: i8::MIN,
    };
}

/// One distinct address.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Seen {
    addr: [u8; 6],
    /// Last time any report came from `addr`.
    seen_at: u64,
    /// Last time an OZ advert came from `addr`, if ever.
    oz_at: Option<u64>,
}

/// The rolling collector. See the module docs for the eviction rules.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BleMetrics {
    buckets: [Bucket; WINDOW_S],
    addrs: heapless::Vec<Seen, MAX_ADDRS>,
    /// Newest time observed so far.
    latest: Option<u64>,
}

impl Default for BleMetrics {
    fn default() -> Self {
        Self::new()
    }
}

/// True if `at` lies within the window ending at `now` (both in ms).
fn fresh(at: u64, now: u64) -> bool {
    now.saturating_sub(at) < WINDOW_MS
}

impl BleMetrics {
    /// An empty collector.
    pub const fn new() -> Self {
        Self {
            buckets: [Bucket::EMPTY; WINDOW_S],
            addrs: heapless::Vec::new(),
            latest: None,
        }
    }

    /// Moves the clock to `now` and returns the effective time: `now`
    /// itself, the newest time seen if `now` is slightly earlier, or `now`
    /// after a reset if it jumped back by more than a window.
    fn advance(&mut self, now: Millis) -> u64 {
        let now = now.0;
        match self.latest {
            Some(latest) if now < latest => {
                if latest - now <= WINDOW_MS {
                    return latest;
                }
                *self = Self::new();
            }
            _ => {}
        }
        self.latest = Some(now);
        now
    }

    /// Records one advertising report from `addr` at `now`.
    pub fn observe(&mut self, addr: [u8; 6], rssi: i8, is_oz: bool, now: Millis) {
        let now = self.advance(now);
        let second = now / 1_000;
        let slot = (second % WINDOW_S as u64) as usize;
        if let Some(bucket) = self.buckets.get_mut(slot) {
            if bucket.second != second {
                *bucket = Bucket {
                    second,
                    count: 0,
                    strongest: i8::MIN,
                };
            }
            bucket.count = bucket.count.saturating_add(1);
            bucket.strongest = bucket.strongest.max(rssi);
        }
        let oz_at = is_oz.then_some(now);
        if let Some(entry) = self.addrs.iter_mut().find(|s| s.addr == addr) {
            entry.seen_at = now;
            if oz_at.is_some() {
                entry.oz_at = oz_at;
            }
            return;
        }
        if self.addrs.is_full() {
            self.addrs.retain(|s| fresh(s.seen_at, now));
        }
        if self.addrs.is_full()
            && let Some(victim) = self.victim()
        {
            self.addrs.swap_remove(victim);
        }
        // Cannot fail: an entry was removed above if the table was full.
        let _ = self.addrs.push(Seen {
            addr,
            seen_at: now,
            oz_at,
        });
    }

    /// Index of the entry to evict from a full table of fresh entries: the
    /// least recently seen non-OZ entry, else the least recently seen one.
    fn victim(&self) -> Option<usize> {
        let oldest = |want_oz: bool| {
            self.addrs
                .iter()
                .enumerate()
                .filter(|(_, s)| s.oz_at.is_some() == want_oz)
                .min_by_key(|(_, s)| s.seen_at)
                .map(|(i, _)| i)
        };
        oldest(false).or_else(|| oldest(true))
    }

    /// The metrics as of `now`.
    pub fn snapshot(&mut self, now: Millis) -> BleSnapshot {
        let now = self.advance(now);
        let previous = (now / 1_000).checked_sub(1);
        let last = previous.and_then(|second| {
            let slot = (second % WINDOW_S as u64) as usize;
            self.buckets
                .get(slot)
                .filter(|b| b.second == second && b.count > 0)
        });
        let unique = self.addrs.iter().filter(|s| fresh(s.seen_at, now)).count();
        let oz = self
            .addrs
            .iter()
            .filter(|s| s.oz_at.is_some_and(|t| fresh(t, now)))
            .count();
        BleSnapshot {
            adv_per_s: last.map_or(0, |b| b.count),
            unique_10s: u16::try_from(unique).unwrap_or(u16::MAX),
            oz_badges: u16::try_from(oz).unwrap_or(u16::MAX),
            strongest_rssi: last.map_or(0, |b| b.strongest),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn addr(n: u8) -> [u8; 6] {
        [n, 0, 0, 0, 0, 0xC0]
    }

    #[test]
    fn fr29_empty_collector_reports_zero() {
        let mut m = BleMetrics::new();
        assert_eq!(m.snapshot(Millis(0)), BleSnapshot::default());
        assert_eq!(m.snapshot(Millis(123_456)), BleSnapshot::default());
    }

    #[test]
    fn fr29_counts_last_complete_second() {
        let mut m = BleMetrics::new();
        m.observe(addr(1), -70, false, Millis(1_100));
        m.observe(addr(2), -40, true, Millis(1_500));
        m.observe(addr(1), -60, false, Millis(1_900));
        // Still inside second 1: the last complete second (0) is empty.
        let s = m.snapshot(Millis(1_950));
        assert_eq!(s.adv_per_s, 0);
        assert_eq!(s.unique_10s, 2);
        assert_eq!(s.oz_badges, 1);
        let s = m.snapshot(Millis(2_010));
        assert_eq!(s.adv_per_s, 3);
        assert_eq!(s.strongest_rssi, -40);
        assert_eq!(s.unique_10s, 2);
        // Second 2 was silent.
        let s = m.snapshot(Millis(3_000));
        assert_eq!(s.adv_per_s, 0);
        assert_eq!(s.strongest_rssi, 0);
        assert_eq!(s.unique_10s, 2);
    }

    #[test]
    fn fr29_window_expiry() {
        let mut m = BleMetrics::new();
        m.observe(addr(1), -50, true, Millis(0));
        m.observe(addr(2), -50, false, Millis(5_000));
        assert_eq!(m.snapshot(Millis(9_999)).unique_10s, 2);
        let s = m.snapshot(Millis(10_000));
        assert_eq!((s.unique_10s, s.oz_badges), (1, 0));
        // Address 1 heard again, but without OZ data: not an OZ badge.
        m.observe(addr(1), -50, false, Millis(12_000));
        let s = m.snapshot(Millis(12_000));
        assert_eq!((s.unique_10s, s.oz_badges), (2, 0));
        assert_eq!(m.snapshot(Millis(22_000)), BleSnapshot::default());
        // A bucket ten seconds old is not mistaken for the last second.
        m.observe(addr(3), -30, false, Millis(30_500));
        assert_eq!(m.snapshot(Millis(41_100)).adv_per_s, 0);
    }

    #[test]
    fn fr29_saturates_at_64_and_keeps_oz_badges() {
        let mut m = BleMetrics::new();
        m.observe(addr(200), -80, true, Millis(0));
        for i in 0..100u8 {
            m.observe(addr(i), -60, false, Millis(1 + u64::from(i)));
        }
        let s = m.snapshot(Millis(200));
        assert_eq!(s.unique_10s, MAX_ADDRS as u16);
        assert_eq!(s.oz_badges, 1, "the OZ badge survives eviction");
        // The newest addresses are kept, the oldest non-OZ ones evicted.
        m.observe(addr(99), -60, false, Millis(201));
        assert_eq!(m.snapshot(Millis(201)).unique_10s, MAX_ADDRS as u16);
        // After the window passes, stale entries make room again.
        m.observe(addr(250), -60, false, Millis(20_000));
        assert_eq!(m.snapshot(Millis(20_000)).unique_10s, 1);
    }

    #[test]
    fn fr29_all_oz_table_evicts_oldest() {
        let mut m = BleMetrics::new();
        for i in 0..=MAX_ADDRS as u8 {
            m.observe(addr(i), -60, true, Millis(u64::from(i)));
        }
        let s = m.snapshot(Millis(100));
        assert_eq!((s.unique_10s, s.oz_badges), (64, 64));
    }

    #[test]
    fn fr29_bursty_reports_saturate_counters() {
        let mut m = BleMetrics::new();
        // 70 000 reports from three advertisers inside one second.
        for i in 0..70_000u32 {
            let a = (i % 3) as u8;
            m.observe(
                addr(a),
                -90 + a as i8,
                a == 0,
                Millis(5_000 + u64::from(i % 1_000)),
            );
        }
        let s = m.snapshot(Millis(6_000));
        assert_eq!(s.adv_per_s, u16::MAX);
        assert_eq!((s.unique_10s, s.oz_badges), (3, 1));
        assert_eq!(s.strongest_rssi, -88);
    }

    #[test]
    fn fr29_time_going_backwards_is_clamped_or_resets() {
        let mut m = BleMetrics::new();
        m.observe(addr(1), -50, false, Millis(10_000));
        // Slightly earlier report: counted at the newest time.
        m.observe(addr(2), -40, false, Millis(9_500));
        let s = m.snapshot(Millis(11_000));
        assert_eq!((s.adv_per_s, s.unique_10s, s.strongest_rssi), (2, 2, -40));
        // A snapshot from the past sees the same state as the newest time.
        assert_eq!(m.snapshot(Millis(10_500)).unique_10s, 2);
        // A big jump back resets the collector.
        m.observe(addr(3), -70, false, Millis(100));
        let s = m.snapshot(Millis(1_000));
        assert_eq!((s.adv_per_s, s.unique_10s), (1, 1));
    }

    #[test]
    fn fr29_clock_wrap_is_safe() {
        let mut m = BleMetrics::new();
        let near_end = u64::MAX - 500;
        m.observe(addr(1), -50, true, Millis(near_end));
        m.observe(addr(1), -50, true, Millis(u64::MAX));
        let s = m.snapshot(Millis(u64::MAX));
        assert_eq!((s.unique_10s, s.oz_badges), (1, 1));
        // Wrapped to zero: treated as a reset.
        m.observe(addr(2), -60, false, Millis(0));
        let s = m.snapshot(Millis(1_000));
        assert_eq!((s.adv_per_s, s.unique_10s, s.oz_badges), (1, 1, 0));
    }
}
