//! Radio-trait adapters that let the middleware's advertising pub/sub run on
//! the badge unchanged.
//!
//! - [`MuxAdvertiser`] implements `middleware::ble::advert::Advertiser`.
//!   `set_advertising_data` does not touch the radio: it queues the data in
//!   the owned [`AdvMux`], which decides when it goes on air (see
//!   [`super::mux`]); the middleware documents this time-sharing as allowed.
//!   Wrap it in `AdvertisingPublisher::new(adapter, `[`mw_template`]`())` and
//!   keep that publisher for the life of the task, so its sequence number
//!   keeps advancing; read the multiplexer with `publisher.advertiser()` and
//!   drive it with `publisher.advertiser_mut()`. A queue that is full
//!   surfaces as `AdvertisingError::Radio(MuxError::QueueFull)` and does not
//!   advance the sequence.
//! - [`mw_template`] is the advertising template for middleware publishes:
//!   Flags plus manufacturer data under [`MW_COMPANY_ID`]. It leaves
//!   [`MAX_ADVERT_PAYLOAD`](super::MAX_ADVERT_PAYLOAD) (21) payload bytes; the
//!   128-bit service UUID does not fit beside a payload, so these adverts are
//!   not recognised by `middleware::ble::advert::discover`.
//! - [`ScanQueue`] is middleware's `middleware::ble::advert::ScanQueue`
//!   (re-exported): a bounded queue of scan reports implementing `Scanner`.
//!   The firmware's scan callback pushes each report in place through
//!   `AdvertisingSubscriber::scanner_mut` with the non-consuming
//!   `ScanQueue::push_report` (when full the oldest report is dropped and
//!   counted; data over 31 bytes is refused), then drains the subscriber
//!   with `Subscriber::try_recv` until `Ok(None)`. `Scanner::next_report`
//!   would wait for a push (the queue keeps a waker); the badge never calls
//!   it. A report longer than the caller's buffer is dropped with
//!   [`ScanQueueError::BufferTooSmall`]; subscribers offer [`REPORT_LEN`]
//!   bytes, so they never see it.
//! - [`app_subscriber`] builds the middleware `AdvertisingSubscriber` the
//!   badge uses for wasm app keys ([`AppSubscriber`]): company id
//!   [`MW_COMPANY_ID`], the filter `KeyFilter::Prefix("app")`, and the
//!   [`KNOWN_APP_KEYS`] learned in its `ShortIdTable`. An advert carries
//!   only the 16-bit short id of its key, and a prefix filter matches only
//!   learned keys, so the firmware also learns (`AdvertisingSubscriber::learn`)
//!   every app key it publishes on. The table holds [`APP_KEYS`] keys of at
//!   most [`APP_KEY_LEN`] bytes; a full table or a key whose short id is taken
//!   by another key is refused (`ShortIdError::Full` / `Collision`). Each
//!   sample carries the key text in `sample.key` and the sender address and
//!   sequence in `sample.meta`; repeats are dropped per (sender, key) over
//!   the [`SENDERS`] most recent pairs.

pub use middleware::ble::advert::{ScanQueue, ScanQueueError};
use middleware::ble::advert::{
    AdvertisementBuilder, Advertiser, AdvertisingSubscriber, MAX_ADV_DATA, ShortIdError,
};
use middleware::{Key, KeyFilter};

use super::mux::{AdvMux, MUX_QUEUE, MuxError};
use super::{APP_KEY_LEN, MW_COMPANY_ID};

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

/// The report buffer size a `Scanner` is offered.
pub const REPORT_LEN: usize = MAX_ADV_DATA;

/// App keys every badge understands without being told (demo apps).
pub const KNOWN_APP_KEYS: [Key<'static>; 4] = [
    Key::from_static("app/hello"),
    Key::from_static("app/ping"),
    Key::from_static("app/chat"),
    Key::from_static("app/color"),
];
/// Keys (and filters) the app subscriber holds.
pub const APP_KEYS: usize = 8;
/// `(sender, key)` pairs the app subscriber remembers for dedup.
pub const SENDERS: usize = 16;
/// The filter selecting every learned app key.
pub const APP_FILTER: KeyFilter<'static> = KeyFilter::Prefix(Key::from_static("app"));

/// The middleware subscriber for app keys over a [`ScanQueue`] of `R`
/// reports (see the module docs).
pub type AppSubscriber<const R: usize> =
    AdvertisingSubscriber<ScanQueue<R>, APP_KEYS, APP_KEY_LEN, SENDERS>;

/// An [`AppSubscriber`] over an empty queue, with [`APP_FILTER`] and the
/// [`KNOWN_APP_KEYS`] learned (see the module docs).
///
/// # Errors
///
/// `ShortIdError` if the known keys do not fit or collide (a build-time
/// property, covered by a test).
pub fn app_subscriber<const R: usize>() -> Result<AppSubscriber<R>, ShortIdError> {
    let mut s = AppSubscriber::new(ScanQueue::new())
        .with_company_id(MW_COMPANY_ID)
        .with_filter(APP_FILTER)?;
    for key in KNOWN_APP_KEYS {
        s.learn(key)?;
    }
    Ok(s)
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use crate::ble::testutil::block_on;
    use badge_core::io::Millis;
    use middleware::ble::advert::{
        Address, AdvertisingError, AdvertisingPublisher, Pushed, ScanReport, Scanner,
        manufacturer_data,
    };
    use middleware::ble::uuid::short_id;
    use middleware::{Publisher, SenderId, Subscriber};

    const HELLO: Key<'static> = Key::from_static("app/hello");
    const OTHER: Key<'static> = Key::from_static("app/other");
    const BEACON: [u8; 6] = [5, 0xFF, b'O', b'Z', 9, 9];
    const PEER: [u8; 6] = [1, 2, 3, 4, 5, 6];
    const PEER2: [u8; 6] = [6, 5, 4, 3, 2, 1];

    /// Advertising data the badge's publisher produces for `key`.
    fn advert(key: Key<'_>, seq: u8, payload: &[u8]) -> std::vec::Vec<u8> {
        let id = short_id(key).to_le_bytes();
        let mut body = std::vec![id[0], id[1], seq];
        body.extend_from_slice(payload);
        mw_template()
            .manufacturer_data(&body)
            .build()
            .unwrap()
            .as_bytes()
            .to_vec()
    }

    /// Pushes one report from `address` into `q` in place.
    fn push<const R: usize>(
        q: &mut ScanQueue<R>,
        address: [u8; 6],
        rssi: i8,
        data: &[u8],
    ) -> Result<Pushed, ScanQueueError> {
        q.push_report(&ScanReport {
            address: Address(address),
            rssi,
            data,
        })
    }

    /// `(key, payload, sender, seq)` of one received sample.
    type Got = (
        std::string::String,
        std::vec::Vec<u8>,
        Option<SenderId>,
        Option<u8>,
    );

    /// Drains `s` with `try_recv`, collecting every sample.
    fn drain<const R: usize>(s: &mut AppSubscriber<R>) -> std::vec::Vec<Got> {
        let mut out = std::vec::Vec::new();
        let mut buf = [0u8; 128];
        while let Some(sample) = s.try_recv(&mut buf).unwrap() {
            out.push((
                sample.key.as_str().into(),
                sample.payload.to_vec(),
                sample.meta.sender,
                sample.meta.seq,
            ));
        }
        out
    }

    #[test]
    fn fr38_publisher_queues_into_mux_with_company_id() {
        let mut p = AdvertisingPublisher::new(MuxAdvertiser::<4>::new(), mw_template());
        block_on(p.publish(HELLO, b"hi")).unwrap();
        let mux = p.advertiser().mux().clone();
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
        assert_eq!(p.max_payload(), Some(crate::ble::MAX_ADVERT_PAYLOAD));
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
        let mut q = ScanQueue::<2>::new();
        assert_eq!(push(&mut q, PEER, -40, &[2, 1, 6]), Ok(Pushed::Queued));
        assert_eq!(push(&mut q, PEER, -41, &[2, 1, 5]), Ok(Pushed::Queued));
        assert_eq!(push(&mut q, PEER, -42, &[2, 1, 4]), Ok(Pushed::DisplacedOldest));
        assert_eq!(push(&mut q, PEER, -43, &[0; 32]), Err(ScanQueueError::TooLong));
        assert_eq!(q.len(), 2);
        assert_eq!(q.dropped(), 2);
        let mut buf = [0u8; REPORT_LEN];
        let r = q.try_next_report(&mut buf).unwrap().unwrap();
        assert_eq!((r.rssi, r.data), (-41, &[2, 1, 5][..]));
        let mut small = [0u8; 2];
        assert_eq!(
            q.try_next_report(&mut small),
            Err(ScanQueueError::BufferTooSmall)
        );
        assert!(q.is_empty());
        assert_eq!(q.dropped(), 3);
        assert_eq!(q.try_next_report(&mut buf), Ok(None));
        push(&mut q, PEER, -44, &[2, 1, 3]).unwrap();
        // A report is queued, so `next_report` returns at once.
        let r = block_on(q.next_report(&mut buf)).unwrap();
        assert_eq!((r.address, r.data), (Address(PEER), &[2, 1, 3][..]));
    }

    #[test]
    fn fr38_app_subscriber_knows_demo_keys_and_learns_app_keys() {
        let mut s = app_subscriber::<4>().unwrap();
        assert_eq!(s.keys().len(), KNOWN_APP_KEYS.len());
        for key in KNOWN_APP_KEYS {
            assert!(s.keys().contains(key));
        }
        // Not learned yet: its adverts cannot be resolved.
        let ad = advert(OTHER, 0, b"x");
        push(s.scanner_mut(), PEER, -40, &ad).unwrap();
        assert!(drain(&mut s).is_empty());
        s.learn(OTHER).unwrap();
        s.learn(OTHER).unwrap();
        assert_eq!(s.keys().len(), KNOWN_APP_KEYS.len() + 1);
        push(s.scanner_mut(), PEER, -40, &ad).unwrap();
        assert_eq!(drain(&mut s).len(), 1);
        // Learned but outside the app prefix: never delivered.
        let sync = Key::from_static("conagotchi/sync");
        s.learn(sync).unwrap();
        push(s.scanner_mut(), PEER, -40, &advert(sync, 0, b"s")).unwrap();
        assert!(drain(&mut s).is_empty());
        // The table is bounded.
        for k in ["app/a", "app/b"] {
            s.learn(Key::new(k).unwrap()).unwrap();
        }
        assert_eq!(s.learn(Key::from_static("app/c")), Err(ShortIdError::Full));
        let long = std::format!("app/{}", "k".repeat(APP_KEY_LEN));
        assert_eq!(
            s.learn(Key::new(&long).unwrap()),
            Err(ShortIdError::TooLong)
        );
    }

    #[test]
    fn fr38_app_subscriber_dedups_per_sender_and_ignores_other_adverts() {
        let mut s = app_subscriber::<8>().unwrap();
        let hello = advert(HELLO, 0, b"hi");
        let foreign = AdvertisementBuilder::new()
            .company_id(0x0059)
            .manufacturer_data(&[1, 2, 3])
            .build()
            .unwrap();
        let ho = advert(HELLO, 1, b"ho");
        let reports: [([u8; 6], &[u8]); 7] = [
            (PEER, &hello),
            (PEER, &hello),
            // Another badge also starting at sequence 0 is not a repeat.
            (PEER2, &hello),
            (PEER, &ho),
            // OZ beacon, foreign company, malformed: ignored.
            (PEER, &BEACON),
            (PEER, foreign.as_bytes()),
            (PEER, &[9, 0xFF]),
        ];
        for (from, ad) in reports {
            push(s.scanner_mut(), from, -40, ad).unwrap();
        }
        let got = drain(&mut s);
        let from = |mac| Some(Address(mac).sender());
        assert_eq!(
            got,
            std::vec![
                ("app/hello".into(), b"hi".to_vec(), from(PEER), Some(0)),
                ("app/hello".into(), b"hi".to_vec(), from(PEER2), Some(0)),
                ("app/hello".into(), b"ho".to_vec(), from(PEER), Some(1)),
            ]
        );
        assert!(s.scanner_mut().is_empty());
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
        let mut s = app_subscriber::<64>().unwrap();
        s.learn(OTHER).unwrap();
        let mut t = 0u64;
        while t < 3_000 {
            p.advertiser_mut().step(Millis(t));
            push(s.scanner_mut(), PEER, -50, p.advertiser().mux().on_air()).unwrap();
            t += 100;
        }
        let got = drain(&mut s);
        // Repeats (each advert is on air for 3 events) and beacons are dropped.
        assert_eq!(
            got,
            std::vec![
                (
                    "app/hello".into(),
                    b"one".to_vec(),
                    Some(Address(PEER).sender()),
                    Some(0)
                ),
                (
                    "app/other".into(),
                    b"two".to_vec(),
                    Some(Address(PEER).sender()),
                    Some(1)
                ),
            ]
        );
    }
}
