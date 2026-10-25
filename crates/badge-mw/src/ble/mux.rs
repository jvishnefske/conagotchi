//! Time-sharing of the badge's single advertising set (pure).
//!
//! The ADV_IND of the badge's one legacy advertising set must carry the stock
//! OZ trade beacon (stock badges drop a peer 4 s after its last beacon and
//! scan at 100 % duty), but middleware publishes are also advertising data.
//! [`AdvMux`] decides which of the two is on air:
//!
//! - In the beacon phase the latest beacon is on air ([`ble_adv::adv_data`]:
//!   the beacon, or Flags only while there is none).
//! - A queued middleware advert is promoted once the beacon has been on air
//!   for [`BEACON_HOLD_MS`] (700 ms), and is then on air for [`MW_HOLD_MS`]
//!   (300 ms, [`MW_EVENTS`] advertising events of [`ADV_EVENT_MS`]), after
//!   which the beacon returns. So the beacon is on air at least 70 % of the
//!   time and is never absent for more than one 300 ms slot, well inside the
//!   [`MAX_BEACON_GAP_MS`] (400 ms) budget, which leaves room for the
//!   firmware's timer and HCI latency.
//! - While there is no beacon at all, queued adverts follow one another
//!   without the hold (there is nothing to protect).
//! - The queue holds [`MUX_QUEUE`] adverts. An advert with the same company
//!   id and short id as a queued one replaces it in place (the newest value
//!   on a key wins and keeps its place); otherwise a full queue rejects it
//!   with [`MuxError::QueueFull`].
//! - A clock that went backwards ends a middleware slot at once and restarts
//!   the beacon hold, so it can only lengthen the beacon's share.
//!
//! Every transition consumes the multiplexer and returns the next one; the
//! firmware calls [`AdvMux::step`] at [`AdvMux::next_deadline`], compares
//! [`AdvMux::on_air`] with what it last installed and updates the set in
//! place when they differ.

use badge_core::ble_adv;
use badge_core::io::Millis;
use heapless::Deque;
use middleware::ble::advert::{MAX_ADV_DATA, manufacturer_data};

/// One advertising event (the badge's advertising interval).
pub const ADV_EVENT_MS: u64 = 100;
/// Advertising events a middleware advert stays on air.
pub const MW_EVENTS: u64 = 3;
/// Time a middleware advert stays on air.
pub const MW_HOLD_MS: u64 = ADV_EVENT_MS * MW_EVENTS;
/// Time the beacon stays on air before the next middleware advert.
pub const BEACON_HOLD_MS: u64 = 700;
/// The longest the beacon may be absent (the design budget the schedule is
/// tested against).
pub const MAX_BEACON_GAP_MS: u64 = 400;
/// Middleware adverts the multiplexer queues.
pub const MUX_QUEUE: usize = 4;

const _: () = assert!(MW_HOLD_MS < MAX_BEACON_GAP_MS);
// Beacon share >= 70 %: hold / (hold + slot) >= 7 / 10.
const _: () = assert!(BEACON_HOLD_MS * 10 >= (BEACON_HOLD_MS + MW_HOLD_MS) * 7);

/// Why the multiplexer refused data.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MuxError {
    /// More than 31 bytes of advertising data.
    TooLong,
    /// The queue is full and the advert replaces none of the queued ones.
    QueueFull,
}

/// Legacy advertising data (at most 31 bytes) held by value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdvBytes {
    buf: [u8; MAX_ADV_DATA],
    len: usize,
}

impl AdvBytes {
    /// Copies `data`.
    ///
    /// # Errors
    ///
    /// [`MuxError::TooLong`] for more than 31 bytes.
    pub fn new(data: &[u8]) -> Result<Self, MuxError> {
        let mut buf = [0u8; MAX_ADV_DATA];
        buf.get_mut(..data.len())
            .ok_or(MuxError::TooLong)?
            .copy_from_slice(data);
        Ok(Self {
            buf,
            len: data.len(),
        })
    }

    /// The bytes.
    pub fn as_bytes(&self) -> &[u8] {
        self.buf.get(..self.len).unwrap_or(&[])
    }

    /// `(company id, short id)` of a middleware publish, read from the first
    /// manufacturer data field; `None` if there is none or it is too short.
    fn publish_id(&self) -> Option<(u16, u16)> {
        match manufacturer_data(self.as_bytes()) {
            Ok(Some((company, [lo, hi, _, ..]))) => Some((company, u16::from_le_bytes([*lo, *hi]))),
            _ => None,
        }
    }
}

/// What is on air and since when.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// The beacon (or the Flags default).
    Beacon { since: Millis },
    /// A middleware advert.
    Mw { data: AdvBytes, since: Millis },
}

/// The advertising schedule (see the module docs). `Q` is the queue size.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AdvMux<const Q: usize = MUX_QUEUE> {
    beacon: Option<AdvBytes>,
    queue: Deque<AdvBytes, Q>,
    phase: Phase,
}

impl<const Q: usize> Default for AdvMux<Q> {
    fn default() -> Self {
        Self::new()
    }
}

/// `now - since`, or `None` if the clock went backwards.
fn elapsed(now: Millis, since: Millis) -> Option<u64> {
    now.0.checked_sub(since.0)
}

impl<const Q: usize> AdvMux<Q> {
    /// No beacon, empty queue, the Flags default on air since time 0.
    pub const fn new() -> Self {
        Self {
            beacon: None,
            queue: Deque::new(),
            phase: Phase::Beacon { since: Millis(0) },
        }
    }

    /// Replaces the beacon (`Effect::Beacon` data); empty data means none.
    ///
    /// # Errors
    ///
    /// [`MuxError::TooLong`] for more than 31 bytes (the multiplexer is
    /// unchanged, as it was consumed only on success).
    pub fn with_beacon(self, beacon: &[u8]) -> Result<Self, MuxError> {
        let beacon = if beacon.is_empty() {
            None
        } else {
            Some(AdvBytes::new(beacon)?)
        };
        Ok(Self { beacon, ..self })
    }

    /// Queues a middleware advert (see the module docs for coalescing).
    ///
    /// # Errors
    ///
    /// [`MuxError::TooLong`] or [`MuxError::QueueFull`].
    pub fn enqueue(self, data: &[u8]) -> Result<Self, MuxError> {
        let advert = AdvBytes::new(data)?;
        let mut queue = self.queue;
        let id = advert.publish_id();
        let slot = queue
            .iter_mut()
            .find(|q| id.is_some() && q.publish_id() == id);
        match slot {
            Some(slot) => *slot = advert,
            None => queue.push_back(advert).map_err(|_| MuxError::QueueFull)?,
        }
        Ok(Self { queue, ..self })
    }

    /// Advances the schedule to `now`: ends a finished middleware slot, then
    /// promotes the next queued advert if the beacon has had its hold.
    pub fn step(self, now: Millis) -> Self {
        self.expire(now).promote(now)
    }

    fn expire(self, now: Millis) -> Self {
        match self.phase {
            Phase::Mw { since, .. } if elapsed(now, since).is_none_or(|e| e >= MW_HOLD_MS) => {
                Self {
                    phase: Phase::Beacon { since: now },
                    ..self
                }
            }
            _ => self,
        }
    }

    fn promote(self, now: Millis) -> Self {
        let Phase::Beacon { since } = self.phase else {
            return self;
        };
        let Some(elapsed) = elapsed(now, since) else {
            // Clock went backwards: restart the hold.
            return Self {
                phase: Phase::Beacon { since: now },
                ..self
            };
        };
        if self.beacon.is_some() && elapsed < BEACON_HOLD_MS {
            return self;
        }
        let mut queue = self.queue;
        match queue.pop_front() {
            Some(data) => Self {
                queue,
                phase: Phase::Mw { data, since: now },
                ..self
            },
            None => Self { queue, ..self },
        }
    }

    /// The advertising data that should be on air now.
    pub fn on_air(&self) -> &[u8] {
        match &self.phase {
            Phase::Mw { data, .. } => data.as_bytes(),
            Phase::Beacon { .. } => {
                ble_adv::adv_data(self.beacon.as_ref().map_or(&[][..], |b| b.as_bytes()))
            }
        }
    }

    /// True while the beacon (or the Flags default) is on air.
    pub fn showing_beacon(&self) -> bool {
        matches!(self.phase, Phase::Beacon { .. })
    }

    /// Middleware adverts waiting.
    pub fn pending(&self) -> usize {
        self.queue.len()
    }

    /// When [`AdvMux::step`] next changes something, if nothing else happens
    /// before: the end of the middleware slot, or the end of the beacon hold
    /// while adverts are queued. `None` when idle.
    pub fn next_deadline(&self) -> Option<Millis> {
        match self.phase {
            Phase::Mw { since, .. } => Some(Millis(since.0.saturating_add(MW_HOLD_MS))),
            Phase::Beacon { since } if !self.queue.is_empty() => {
                let hold = if self.beacon.is_some() {
                    BEACON_HOLD_MS
                } else {
                    0
                };
                Some(Millis(since.0.saturating_add(hold)))
            }
            Phase::Beacon { .. } => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ble::MW_COMPANY_ID;
    use middleware::ble::advert::AdvertisementBuilder;

    const BEACON: [u8; 8] = [7, 0xFF, b'O', b'Z', 1, 2, 3, 4];

    fn mw(short_id: u16, seq: u8) -> AdvBytes {
        let id = short_id.to_le_bytes();
        let body = [id[0], id[1], seq, 0xAA];
        let ad = AdvertisementBuilder::new()
            .company_id(MW_COMPANY_ID)
            .manufacturer_data(&body)
            .build()
            .unwrap();
        AdvBytes::new(ad.as_bytes()).unwrap()
    }

    fn with_beacon() -> AdvMux {
        AdvMux::new().with_beacon(&BEACON).unwrap()
    }

    #[test]
    fn fr38_mux_shows_beacon_until_something_is_queued() {
        let m = AdvMux::<4>::new();
        assert_eq!(m.on_air(), &ble_adv::DEFAULT_ADV_DATA);
        let m = m.with_beacon(&BEACON).unwrap().step(Millis(5_000));
        assert_eq!(m.on_air(), &BEACON);
        assert!(m.showing_beacon());
        assert_eq!(m.next_deadline(), None);
        let m = m.with_beacon(&[]).unwrap();
        assert_eq!(m.on_air(), &ble_adv::DEFAULT_ADV_DATA);
        assert_eq!(
            AdvMux::<4>::new().with_beacon(&[0; 32]),
            Err(MuxError::TooLong)
        );
    }

    #[test]
    fn fr38_mux_publish_shows_for_slot_then_beacon_returns() {
        let a = mw(1, 0);
        let m = with_beacon()
            .step(Millis(1_000))
            .enqueue(a.as_bytes())
            .unwrap();
        // Beacon has been on air since 0 for > 700 ms: promoted at once.
        assert_eq!(m.next_deadline(), Some(Millis(BEACON_HOLD_MS)));
        let m = m.step(Millis(1_000));
        assert_eq!(m.on_air(), a.as_bytes());
        assert_eq!(m.next_deadline(), Some(Millis(1_300)));
        let m = m.step(Millis(1_299));
        assert_eq!(m.on_air(), a.as_bytes());
        let m = m.step(Millis(1_300));
        assert_eq!(m.on_air(), &BEACON);
        assert_eq!(m.next_deadline(), None);
    }

    #[test]
    fn fr38_mux_beacon_hold_separates_publishes() {
        let (a, b) = (mw(1, 0), mw(2, 0));
        let m = with_beacon()
            .enqueue(a.as_bytes())
            .unwrap()
            .enqueue(b.as_bytes())
            .unwrap()
            .step(Millis(700));
        assert_eq!(m.on_air(), a.as_bytes());
        let m = m.step(Millis(1_000));
        assert_eq!(m.on_air(), &BEACON);
        assert_eq!(m.next_deadline(), Some(Millis(1_700)));
        let m = m.step(Millis(1_699));
        assert!(m.showing_beacon());
        let m = m.step(Millis(1_700));
        assert_eq!(m.on_air(), b.as_bytes());
        assert_eq!(m.pending(), 0);
    }

    #[test]
    fn fr38_mux_without_beacon_publishes_back_to_back() {
        let (a, b) = (mw(1, 0), mw(2, 0));
        let m = AdvMux::<4>::new()
            .enqueue(a.as_bytes())
            .unwrap()
            .enqueue(b.as_bytes())
            .unwrap();
        assert_eq!(m.next_deadline(), Some(Millis(0)));
        let m = m.step(Millis(10));
        assert_eq!(m.on_air(), a.as_bytes());
        let m = m.step(Millis(310));
        assert_eq!(m.on_air(), b.as_bytes());
    }

    #[test]
    fn fr38_mux_queue_coalesces_per_key_and_is_bounded() {
        let m = AdvMux::<2>::new()
            .with_beacon(&BEACON)
            .unwrap()
            .enqueue(mw(1, 0).as_bytes())
            .unwrap()
            .enqueue(mw(2, 0).as_bytes())
            .unwrap();
        // Same key: replaces in place, even when full.
        let m = m.enqueue(mw(1, 1).as_bytes()).unwrap();
        assert_eq!(m.pending(), 2);
        assert_eq!(
            m.clone().enqueue(mw(3, 0).as_bytes()),
            Err(MuxError::QueueFull)
        );
        // Non-middleware data is never coalesced.
        let m2 = AdvMux::<2>::new().enqueue(&[2, 1, 6]).unwrap();
        assert_eq!(
            m2.enqueue(&[2, 1, 6]).unwrap().enqueue(&[2, 1, 6]),
            Err(MuxError::QueueFull)
        );
        let m = m.step(Millis(700));
        assert_eq!(m.on_air(), mw(1, 1).as_bytes());
        assert_eq!(m.clone().enqueue(&[0; 40]), Err(MuxError::TooLong));
    }

    #[test]
    fn fr38_mux_clock_going_backwards_favours_the_beacon() {
        let m = with_beacon()
            .enqueue(mw(1, 0).as_bytes())
            .unwrap()
            .enqueue(mw(2, 0).as_bytes())
            .unwrap()
            .step(Millis(10_000));
        assert!(!m.showing_beacon());
        // Back in time: the slot ends and the hold restarts.
        let m = m.step(Millis(50));
        assert!(m.showing_beacon());
        let m = m.step(Millis(600));
        assert!(m.showing_beacon());
        let m = m.step(Millis(750));
        assert!(!m.showing_beacon());
        // Backwards during the beacon phase restarts its hold too.
        let m = m.step(Millis(1_050)).step(Millis(20));
        assert!(m.showing_beacon());
        assert_eq!(m.next_deadline(), None);
    }

    /// Simulates 60 s with a publish always pending, stepping at the
    /// deadline plus a timer lateness of up to 30 ms, and checks the duty cycle and
    /// the longest beacon gap.
    #[test]
    fn fr38_mux_keeps_beacon_duty_and_gap_under_load() {
        let mut m = with_beacon();
        let mut beacon_ms = 0u64;
        let mut gap = 0u64;
        let mut worst_gap = 0u64;
        let mut seq = 0u8;
        let end = 60_000u64;
        for t in 0..end {
            if m.pending() == 0 {
                seq = seq.wrapping_add(1);
                m = m.enqueue(mw(u16::from(seq % 3), seq).as_bytes()).unwrap();
            }
            // Late timer: steps only every 30 ms.
            if t % 30 == 0 {
                m = m.step(Millis(t));
            }
            if m.showing_beacon() {
                beacon_ms += 1;
                gap = 0;
            } else {
                gap += 1;
                worst_gap = worst_gap.max(gap);
            }
        }
        assert!(
            beacon_ms * 10 >= end * 7,
            "beacon share {beacon_ms} of {end}"
        );
        assert!(worst_gap <= MAX_BEACON_GAP_MS, "gap {worst_gap}");
        assert!(worst_gap >= MW_HOLD_MS);
    }
}
