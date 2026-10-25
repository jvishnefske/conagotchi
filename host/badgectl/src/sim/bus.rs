//! The simulator's message bus, built on the `middleware` traits.
//!
//! [`LocalBus`] is an in-process implementation of `middleware::Publisher`
//! and `middleware::Subscriber`: one bounded FIFO per subscription, filled by
//! every publish on exactly that key (middleware keys have no wildcards).
//! It is pure and deterministic: a publish is visible to every existing
//! subscription immediately, in publish order; a subscription made later
//! sees only later samples; a full queue drops its oldest sample (counted).
//!
//! `Subscriber::recv` waits for the next sample. The simulator runs in
//! discrete ticks and must only drain what is already there, so
//! [`poll_now`] polls a future exactly once with a no-op waker: an empty
//! subscription yields `None` instead of blocking. The Zenoh backend
//! (`super::zenoh_net`) is drained the same way.
//!
//! [`Network`] is the small surface the world needs (subscribe, publish,
//! drain one) so it runs unchanged over [`LocalBus`] or Zenoh; it exists
//! because the middleware has no trait for *creating* a subscription.
//!
//! Samples carry no sender identity in the middleware, but a radio does not
//! hear itself, so every payload starts with an 8-byte little-endian origin
//! (the badge MAC): [`envelope`] / [`open_envelope`].

use std::collections::VecDeque;
use std::future::Future;
use std::pin::pin;
use std::task::{Context, Poll, Waker};

use middleware::{Key, KeyError, Publisher, Sample, Subscriber, transport::sample_into};

/// Samples a subscription holds before the oldest is dropped.
pub const QUEUE_DEPTH: usize = 64;
/// Bytes of the origin header in every simulator payload.
pub const ORIGIN_LEN: usize = 8;
/// Receive buffer size: the longest key plus the largest payload we send.
pub const SAMPLE_BUF: usize = middleware::key::MAX_KEY_LEN + 256;

/// Polls `fut` once; `None` if it is not ready.
pub fn poll_now<F: Future>(fut: F) -> Option<F::Output> {
    let mut fut = pin!(fut);
    let mut cx = Context::from_waker(Waker::noop());
    match fut.as_mut().poll(&mut cx) {
        Poll::Ready(v) => Some(v),
        Poll::Pending => None,
    }
}

/// `origin` (LE) followed by `body`.
pub fn envelope(origin: u64, body: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(ORIGIN_LEN + body.len());
    out.extend_from_slice(&origin.to_le_bytes());
    out.extend_from_slice(body);
    out
}

/// Splits a payload into origin and body; `None` if shorter than the header.
pub fn open_envelope(payload: &[u8]) -> Option<(u64, &[u8])> {
    let (origin, body) = payload.split_first_chunk::<ORIGIN_LEN>()?;
    Some((u64::from_le_bytes(*origin), body))
}

/// Errors of the in-process bus.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum BusError {
    /// The key is not a valid middleware key.
    #[error("invalid key: {0:?}")]
    Key(KeyError),
    /// The receive buffer cannot hold key plus payload (the sample stays queued).
    #[error("receive buffer too small: need {needed}, have {capacity}")]
    BufferTooSmall {
        /// Bytes required.
        needed: usize,
        /// Bytes available.
        capacity: usize,
    },
    /// No such subscription.
    #[error("unknown subscription {0}")]
    UnknownSubscription(usize),
}

/// Handle of one [`LocalBus`] subscription.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SubId(usize);

#[derive(Debug, Clone)]
struct Queue {
    key: String,
    samples: VecDeque<Vec<u8>>,
}

/// The in-process bus.
#[derive(Debug, Clone, Default)]
pub struct LocalBus {
    queues: Vec<Queue>,
    dropped: u64,
}

impl LocalBus {
    /// An empty bus.
    pub fn new() -> Self {
        Self::default()
    }

    /// Subscribes to `key`.
    pub fn subscribe(&mut self, key: Key<'_>) -> SubId {
        self.queues.push(Queue { key: key.as_str().to_string(), samples: VecDeque::new() });
        SubId(self.queues.len() - 1)
    }

    /// A publisher handle (borrows the bus).
    pub fn publisher(&mut self) -> BusPublisher<'_> {
        BusPublisher { bus: self }
    }

    /// A subscriber handle for `id` (borrows the bus).
    pub fn subscriber(&mut self, id: SubId) -> BusSubscriber<'_> {
        BusSubscriber { bus: self, id }
    }

    /// Samples dropped because a queue was full.
    #[cfg(test)]
    pub fn dropped(&self) -> u64 {
        self.dropped
    }

    /// Samples waiting on `id`.
    #[cfg(test)]
    pub fn pending(&self, id: SubId) -> usize {
        self.queues.get(id.0).map_or(0, |q| q.samples.len())
    }
}

/// `middleware::Publisher` over a [`LocalBus`].
#[derive(Debug)]
pub struct BusPublisher<'a> {
    bus: &'a mut LocalBus,
}

impl Publisher for BusPublisher<'_> {
    type Error = BusError;

    async fn publish(&mut self, key: Key<'_>, payload: &[u8]) -> Result<(), BusError> {
        let mut dropped = 0;
        for q in self.bus.queues.iter_mut().filter(|q| q.key == key.as_str()) {
            if q.samples.len() >= QUEUE_DEPTH {
                q.samples.pop_front();
                dropped += 1;
            }
            q.samples.push_back(payload.to_vec());
        }
        self.bus.dropped += dropped;
        Ok(())
    }
}

/// `middleware::Subscriber` for one [`LocalBus`] subscription. `recv` stays
/// pending while the queue is empty (use [`poll_now`]).
#[derive(Debug)]
pub struct BusSubscriber<'a> {
    bus: &'a mut LocalBus,
    id: SubId,
}

impl Subscriber for BusSubscriber<'_> {
    type Error = BusError;

    async fn recv<'b>(&mut self, buf: &'b mut [u8]) -> Result<Sample<'b>, BusError> {
        let q = self.bus.queues.get_mut(self.id.0).ok_or(BusError::UnknownSubscription(self.id.0))?;
        let Some(payload) = q.samples.front() else {
            return std::future::pending().await;
        };
        let needed = q.key.len() + payload.len();
        if needed > buf.len() {
            return Err(BusError::BufferTooSmall { needed, capacity: buf.len() });
        }
        let payload = q.samples.pop_front().unwrap_or_default();
        let key = q.key.clone();
        let capacity = buf.len();
        sample_into(buf, &key, &payload).ok_or(BusError::BufferTooSmall { needed, capacity })
    }
}

/// What the simulator needs from a transport.
pub trait Network {
    /// A subscription handle.
    type Sub;
    /// Transport failure.
    type Error: std::error::Error + Send + Sync + 'static;
    /// Subscribes to `key`.
    fn subscribe(&mut self, key: &str) -> Result<Self::Sub, Self::Error>;
    /// Publishes `payload` on `key`.
    fn publish(&mut self, key: &str, payload: &[u8]) -> Result<(), Self::Error>;
    /// The next sample already received on `sub`, if any: (key, payload).
    fn try_recv(&mut self, sub: &mut Self::Sub) -> Result<Option<(String, Vec<u8>)>, Self::Error>;
}

/// Copies a sample out of the receive buffer.
pub fn owned(sample: Sample<'_>) -> (String, Vec<u8>) {
    (sample.key.as_str().to_string(), sample.payload.to_vec())
}

impl Network for LocalBus {
    type Sub = SubId;
    type Error = BusError;

    fn subscribe(&mut self, key: &str) -> Result<SubId, BusError> {
        Ok(LocalBus::subscribe(self, Key::new(key).map_err(BusError::Key)?))
    }

    fn publish(&mut self, key: &str, payload: &[u8]) -> Result<(), BusError> {
        let key = Key::new(key).map_err(BusError::Key)?;
        poll_now(self.publisher().publish(key, payload)).unwrap_or(Ok(()))
    }

    fn try_recv(&mut self, sub: &mut SubId) -> Result<Option<(String, Vec<u8>)>, BusError> {
        let mut buf = [0u8; SAMPLE_BUF];
        let mut s = self.subscriber(*sub);
        poll_now(s.recv(&mut buf)).transpose().map(|o| o.map(owned))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const A: Key<'static> = Key::from_static("app/a");
    const B: Key<'static> = Key::from_static("app/b");

    fn drain(bus: &mut LocalBus, mut id: SubId) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        while let Some((_, p)) = bus.try_recv(&mut id).unwrap() {
            out.push(p);
        }
        out
    }

    #[test]
    fn fr39_bus_delivers_to_every_subscriber_of_the_exact_key_in_order() {
        let mut bus = LocalBus::new();
        let a1 = bus.subscribe(A);
        let a2 = bus.subscribe(A);
        let b = bus.subscribe(B);
        Network::publish(&mut bus, "app/a", b"1").unwrap();
        Network::publish(&mut bus, "app/a", b"2").unwrap();
        Network::publish(&mut bus, "app/b", b"x").unwrap();
        Network::publish(&mut bus, "app", b"nobody").unwrap();
        assert_eq!(drain(&mut bus, a1), [b"1".to_vec(), b"2".to_vec()]);
        assert_eq!(drain(&mut bus, a2), [b"1".to_vec(), b"2".to_vec()]);
        assert_eq!(drain(&mut bus, b), [b"x".to_vec()]);
        assert_eq!(drain(&mut bus, a1), Vec::<Vec<u8>>::new(), "empty: no blocking");
    }

    #[test]
    fn fr39_bus_late_subscriber_sees_only_later_samples() {
        let mut bus = LocalBus::new();
        Network::publish(&mut bus, "app/a", b"early").unwrap();
        let a = bus.subscribe(A);
        Network::publish(&mut bus, "app/a", b"late").unwrap();
        assert_eq!(drain(&mut bus, a), [b"late".to_vec()]);
    }

    #[test]
    fn fr39_bus_recv_through_the_middleware_trait() {
        let mut bus = LocalBus::new();
        let a = bus.subscribe(A);
        let mut buf = [0u8; 16];
        assert!(poll_now(bus.subscriber(a).recv(&mut buf)).is_none(), "pending when empty");
        poll_now(bus.publisher().publish(A, b"hello")).unwrap().unwrap();
        let mut tiny = [0u8; 6];
        let err = poll_now(bus.subscriber(a).recv(&mut tiny)).unwrap();
        assert_eq!(err, Err(BusError::BufferTooSmall { needed: 10, capacity: 6 }));
        assert_eq!(bus.pending(a), 1, "too-small buffer keeps the sample");
        let s = poll_now(bus.subscriber(a).recv(&mut buf)).unwrap().unwrap();
        assert_eq!((s.key, s.payload), (A, &b"hello"[..]));
        assert!(matches!(
            poll_now(bus.subscriber(SubId(9)).recv(&mut buf)),
            Some(Err(BusError::UnknownSubscription(9)))
        ));
    }

    #[test]
    fn fr39_bus_full_queue_drops_oldest() {
        let mut bus = LocalBus::new();
        let a = bus.subscribe(A);
        for i in 0..QUEUE_DEPTH + 3 {
            Network::publish(&mut bus, "app/a", &[u8::try_from(i).unwrap()]).unwrap();
        }
        assert_eq!(bus.dropped(), 3);
        let got = drain(&mut bus, a);
        assert_eq!(got.len(), QUEUE_DEPTH);
        assert_eq!(got[0], [3]);
    }

    #[test]
    fn fr39_bus_rejects_invalid_keys_and_envelopes_round_trip() {
        let mut bus = LocalBus::new();
        assert!(Network::subscribe(&mut bus, "a b").is_err());
        assert!(Network::publish(&mut bus, "a//b", b"").is_err());
        let e = envelope(0x0200_C0FF_EE01, b"body");
        assert_eq!(open_envelope(&e), Some((0x0200_C0FF_EE01, &b"body"[..])));
        assert_eq!(open_envelope(b"short"), None);
    }
}
