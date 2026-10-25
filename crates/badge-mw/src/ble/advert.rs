//! Radio-trait adapters that let the middleware's advertising pub/sub run on
//! the badge unchanged.
//!
//! - [`MuxAdvertiser`] implements `middleware::ble::advert::Advertiser`.
//!   `set_advertising_data` does not touch the radio: it queues the data in
//!   the owned [`AdvMux`], which decides when it goes on air (see
//!   [`super::mux`]). Wrap it in `AdvertisingPublisher::new(adapter,
//!   `[`mw_template`]`())` and keep that publisher for the life of the task,
//!   so its sequence number keeps advancing; reach the multiplexer with
//!   `publisher.advertiser_mut()`. A queue that is full surfaces as
//!   `AdvertisingError::Radio(MuxError::QueueFull)` and does not advance the
//!   sequence.
//! - [`mw_template`] is the advertising template for middleware publishes:
//!   Flags plus manufacturer data under [`MW_COMPANY_ID`]. It leaves
//!   [`MAX_ADVERT_PAYLOAD`](super::MAX_ADVERT_PAYLOAD) (21) payload bytes; the
//!   128-bit service UUID does not fit beside a payload, so these adverts are
//!   not recognised by `middleware::ble::advert::discover`.
//! - [`ScanQueue`] implements `middleware::ble::advert::Scanner` over a
//!   bounded queue of raw reports the firmware's scan callback pushes
//!   ([`ScanQueue::push`]; when full the oldest report is dropped and
//!   counted). `next_report` never waits: an empty queue is
//!   [`ScanQueueError::Empty`], so an `AdvertisingSubscriber` over it is
//!   polled until that error, a bounded drain.

use heapless::Deque;
use middleware::ble::advert::{
    Address, AdvertisementBuilder, Advertiser, MAX_ADV_DATA, ScanReport, Scanner,
};

use super::MW_COMPANY_ID;
use super::mux::{AdvBytes, AdvMux, MUX_QUEUE, MuxError};

/// Advertising template for middleware publishes (see the module docs).
pub const fn mw_template() -> AdvertisementBuilder<'static> {
    AdvertisementBuilder::new().company_id(MW_COMPANY_ID)
}

/// `Advertiser` that queues into an [`AdvMux`] (see the module docs).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct MuxAdvertiser<const Q: usize = MUX_QUEUE> {
    mux: AdvMux<Q>,
}

impl<const Q: usize> MuxAdvertiser<Q> {
    /// An adapter with an empty multiplexer.
    pub const fn new() -> Self {
        Self { mux: AdvMux::new() }
    }

    /// The multiplexer.
    pub fn mux(&self) -> &AdvMux<Q> {
        &self.mux
    }

    /// Applies `f` to the multiplexer; on error the multiplexer is unchanged.
    ///
    /// # Errors
    ///
    /// Whatever `f` returns.
    pub fn update(
        &mut self,
        f: impl FnOnce(AdvMux<Q>) -> Result<AdvMux<Q>, MuxError>,
    ) -> Result<(), MuxError> {
        self.mux = f(self.mux.clone())?;
        Ok(())
    }

    /// Replaces the OZ beacon.
    ///
    /// # Errors
    ///
    /// [`MuxError::TooLong`].
    pub fn set_beacon(&mut self, beacon: &[u8]) -> Result<(), MuxError> {
        self.update(|m| m.with_beacon(beacon))
    }

    /// Advances the schedule to `now` (see [`AdvMux::step`]).
    pub fn step(&mut self, now: badge_core::io::Millis) {
        self.mux = self.mux.clone().step(now);
    }
}

impl<const Q: usize> Advertiser for MuxAdvertiser<Q> {
    type Error = MuxError;

    async fn set_advertising_data(&mut self, data: &[u8]) -> Result<(), MuxError> {
        self.update(|m| m.enqueue(data))
    }
}

/// Why [`ScanQueue::next_report`](Scanner::next_report) returned no report.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScanQueueError {
    /// No report is queued.
    Empty,
    /// The caller's buffer is shorter than the queued report (it is dropped).
    BufferTooSmall,
}

/// One queued scan report.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Raw {
    address: [u8; 6],
    rssi: i8,
    data: AdvBytes,
}

/// Bounded queue of raw scan reports implementing `Scanner` (see the module
/// docs). `R` is the capacity.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ScanQueue<const R: usize> {
    reports: Deque<Raw, R>,
    dropped: u32,
}

impl<const R: usize> ScanQueue<R> {
    /// An empty queue.
    pub const fn new() -> Self {
        Self {
            reports: Deque::new(),
            dropped: 0,
        }
    }

    /// Queues one report heard from `address`. Data longer than 31 bytes is
    /// counted as dropped; a full queue drops (and counts) its oldest report.
    pub fn push(self, address: [u8; 6], rssi: i8, data: &[u8]) -> Self {
        let Ok(data) = AdvBytes::new(data) else {
            return Self {
                dropped: self.dropped.wrapping_add(1),
                ..self
            };
        };
        let mut reports = self.reports;
        let mut dropped = self.dropped;
        if reports.is_full() {
            reports.pop_front();
            dropped = dropped.wrapping_add(1);
        }
        // Cannot fail: a slot was freed above if the queue was full.
        let _ = reports.push_back(Raw {
            address,
            rssi,
            data,
        });
        Self { reports, dropped }
    }

    /// Reports waiting.
    pub fn len(&self) -> usize {
        self.reports.len()
    }

    /// True if no report is waiting.
    pub fn is_empty(&self) -> bool {
        self.reports.is_empty()
    }

    /// Reports dropped so far (queue full or oversized).
    pub fn dropped(&self) -> u32 {
        self.dropped
    }
}

impl<const R: usize> Scanner for ScanQueue<R> {
    type Error = ScanQueueError;

    async fn next_report<'b>(
        &mut self,
        buf: &'b mut [u8],
    ) -> Result<ScanReport<'b>, ScanQueueError> {
        let raw = self.reports.pop_front().ok_or(ScanQueueError::Empty)?;
        let bytes = raw.data.as_bytes();
        let out = buf
            .get_mut(..bytes.len())
            .ok_or(ScanQueueError::BufferTooSmall)?;
        out.copy_from_slice(bytes);
        Ok(ScanReport {
            address: Address(raw.address),
            rssi: raw.rssi,
            data: out,
        })
    }
}

/// The report buffer size a `Scanner` is offered.
pub const REPORT_LEN: usize = MAX_ADV_DATA;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ble::testutil::block_on;
    use badge_core::io::Millis;
    use middleware::ble::advert::{
        AdvertisingError, AdvertisingPublisher, AdvertisingSubscriber, manufacturer_data,
    };
    use middleware::ble::uuid::short_id;
    use middleware::{Key, Publisher, Subscriber};

    const HELLO: Key<'static> = Key::from_static("app/hello");
    const OTHER: Key<'static> = Key::from_static("app/other");
    const BEACON: [u8; 6] = [5, 0xFF, b'O', b'Z', 9, 9];
    const PEER: [u8; 6] = [1, 2, 3, 4, 5, 6];

    #[test]
    fn fr38_publisher_queues_into_mux_with_company_id() {
        let mut p = AdvertisingPublisher::new(MuxAdvertiser::<4>::new(), mw_template());
        block_on(p.publish(HELLO, b"hi")).unwrap();
        let mux = p.advertiser_mut().mux().clone();
        assert_eq!(mux.pending(), 1);
        // Promoted at once (no beacon yet).
        let mux = mux.step(Millis(1));
        let (company, data) = manufacturer_data(mux.on_air()).unwrap().unwrap();
        assert_eq!(company, MW_COMPANY_ID);
        let id = short_id(HELLO).to_le_bytes();
        assert_eq!(data, &[id[0], id[1], 0, b'h', b'i']);
    }

    #[test]
    fn fr38_publisher_reports_full_queue_and_payload_limit() {
        let mut p = AdvertisingPublisher::new(MuxAdvertiser::<1>::new(), mw_template());
        assert!(block_on(p.publish(HELLO, &[0; crate::ble::MAX_ADVERT_PAYLOAD])).is_ok());
        assert!(matches!(
            block_on(p.publish(HELLO, &[0; crate::ble::MAX_ADVERT_PAYLOAD + 1])),
            Err(AdvertisingError::Advert(_))
        ));
        // Same key coalesces even when full; another key does not fit.
        assert!(block_on(p.publish(HELLO, b"x")).is_ok());
        assert_eq!(
            block_on(p.publish(OTHER, b"x")),
            Err(AdvertisingError::Radio(MuxError::QueueFull))
        );
        assert_eq!(p.sequence().get(), 2);
    }

    #[test]
    fn fr38_scan_queue_is_bounded_and_drops_oldest() {
        let q = ScanQueue::<2>::new()
            .push(PEER, -40, &[2, 1, 6])
            .push(PEER, -41, &[2, 1, 5])
            .push(PEER, -42, &[2, 1, 4])
            .push(PEER, -43, &[0; 32]);
        assert_eq!(q.len(), 2);
        assert_eq!(q.dropped(), 2);
        let mut q = q;
        let mut buf = [0u8; REPORT_LEN];
        let r = block_on(q.next_report(&mut buf)).unwrap();
        assert_eq!((r.rssi, r.data), (-41, &[2, 1, 5][..]));
        let mut small = [0u8; 2];
        assert_eq!(
            block_on(q.next_report(&mut small)),
            Err(ScanQueueError::BufferTooSmall)
        );
        assert!(q.is_empty());
        assert_eq!(
            block_on(q.next_report(&mut buf)),
            Err(ScanQueueError::Empty)
        );
    }

    /// AdvertisingPublisher -> MuxAdvertiser -> (on air) -> ScanQueue ->
    /// AdvertisingSubscriber, with the OZ beacon time-sharing the set.
    #[test]
    fn fr38_advert_round_trip_through_mux_and_scan_queue() {
        let mut p = AdvertisingPublisher::new(MuxAdvertiser::<4>::new(), mw_template());
        p.advertiser_mut().set_beacon(&BEACON).unwrap();
        block_on(p.publish(HELLO, b"one")).unwrap();
        block_on(p.publish(OTHER, b"two")).unwrap();

        // Sample what is on air every advertising event for 3 s.
        let mut q = ScanQueue::<64>::new();
        let mut t = 0u64;
        while t < 3_000 {
            let adv = p.advertiser_mut();
            adv.step(Millis(t));
            q = q.push(PEER, -50, adv.mux().on_air());
            t += 100;
        }
        let mut s = AdvertisingSubscriber::new(q, [HELLO, OTHER]).with_company_id(MW_COMPANY_ID);
        let mut buf = [0u8; 64];
        let first = block_on(s.recv(&mut buf)).unwrap();
        assert_eq!((first.key, first.payload), (HELLO, &b"one"[..]));
        let second = block_on(s.recv(&mut buf)).unwrap();
        assert_eq!((second.key, second.payload), (OTHER, &b"two"[..]));
        // Repeats (each advert is on air for 3 events) and beacons are dropped.
        assert_eq!(
            block_on(s.recv(&mut buf)),
            Err(AdvertisingError::Radio(ScanQueueError::Empty))
        );
    }
}
