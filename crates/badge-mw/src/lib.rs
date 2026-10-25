//! Conagotchi's bindings of the [`middleware`] crate (0.2).
//!
//! Badge features are written against `middleware::{Publisher, Subscriber,
//! Responder, Handler}` and carried by whichever transport fits. Since
//! middleware 0.2 the transports themselves live upstream; this crate keeps
//! only what is specific to the badge.
//!
//! Middleware features the badge uses:
//!
//! - `espnow`: `middleware::espnow::EspNowTransport` carries LED sync
//!   beacons ([`keys::SYNC`]) and wasm app keys (`KeyFilter::Prefix("app")`)
//!   between badges; the firmware implements its `EspNowRadio` over
//!   esp-radio. Re-exported as [`espnow`].
//! - `ble`: `AdvertisingPublisher` / `AdvertisingSubscriber` (with its
//!   `ShortIdTable`) for app keys in advertising data, and `BleServer` for
//!   the GATT RPC responder.
//! - core: `Key`, `KeyFilter`, `Sample` with `SampleMeta` (sender and
//!   sequence), `Subscriber::try_recv`.
//!
//! Badge-specific parts:
//!
//! - [`keys`]: the keys badges publish on and the `badge/*` routing of wasm
//!   payload publishes (shared by firmware, simulator and wasm apps).
//! - [`ble`]: the advertising multiplexer that time-shares the badge's one
//!   advertising set with the stock OZ trade beacon, the `Advertiser` and
//!   `Scanner` adapters over it, the app-key subscriber preset, and the GATT
//!   RPC handler plus its `GattServer` adapter.
//!
//! `no_std`, no heap, `#![forbid(unsafe_code)]`; async fns in traits as in
//! the middleware crate.

#![no_std]
#![forbid(unsafe_code)]
#![allow(async_fn_in_trait)]

pub mod ble;
pub mod keys;
pub mod payload;

/// The middleware ESP-NOW transport (`espnow` feature), re-exported so the
/// firmware needs no direct `middleware` dependency.
pub use middleware::espnow;
/// Core middleware items the firmware drives, re-exported for the same
/// reason.
pub use middleware::{
    Key, KeyFilter, Publisher, Responder, Sample, SampleMeta, SenderId, Subscriber,
};

/// The keys a badge receives over ESP-NOW: LED sync beacons and every wasm
/// app key.
pub const ESPNOW_FILTERS: [KeyFilter<'static>; 2] = [
    KeyFilter::Exact(keys::SYNC),
    KeyFilter::Prefix(Key::from_static("app")),
];

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use badge_core::sync::SyncBeacon;
    use core::pin::pin;
    use core::task::{Context, Poll, Waker};
    use espnow::{EspNowRadio, EspNowTransport, Frame, HEADER_LEN, MAX_FRAME, Mac};
    use std::borrow::ToOwned;
    use std::collections::VecDeque;
    use std::vec::Vec;

    const ME: Mac = [2, 0, 0, 0, 0, 1];
    const PEER: Mac = [2, 0, 0, 0, 0, 2];

    /// In-memory radio: `inbox` feeds the receive calls, `sent` collects
    /// broadcasts.
    #[derive(Default)]
    struct Air {
        inbox: VecDeque<(Vec<u8>, Mac)>,
        sent: Vec<Vec<u8>>,
    }

    impl EspNowRadio for Air {
        type Error = ();
        async fn broadcast(&mut self, frame: &[u8]) -> Result<(), ()> {
            self.sent.push(frame.to_vec());
            Ok(())
        }
        async fn receive<'b>(&mut self, buf: &'b mut [u8]) -> Result<(&'b [u8], Mac), ()> {
            self.try_receive(buf)?.ok_or(())
        }
        fn try_receive<'b>(&mut self, buf: &'b mut [u8]) -> Result<Option<(&'b [u8], Mac)>, ()> {
            let Some((frame, src)) = self.inbox.pop_front() else {
                return Ok(None);
            };
            let out = buf.get_mut(..frame.len()).ok_or(())?;
            out.copy_from_slice(&frame);
            Ok(Some((out, src)))
        }
    }

    /// Polls `f` once; these futures never wait.
    fn ready<F: core::future::Future>(f: F) -> F::Output {
        match pin!(f).poll(&mut Context::from_waker(Waker::noop())) {
            Poll::Ready(v) => v,
            Poll::Pending => panic!("future waited"),
        }
    }

    /// LED sync beacons on `keys::SYNC` survive the middleware ESP-NOW frame
    /// unchanged (the transport itself is tested upstream).
    #[test]
    fn fr37_sync_beacon_round_trips_through_espnow_frame() {
        let beacon = SyncBeacon {
            id: 0x00F6_0AC8,
            network_ms: 123_456,
        };
        let wire = beacon.encode();
        let mut out = [0u8; MAX_FRAME];
        let n = Frame {
            seq: 7,
            key: keys::SYNC,
            payload: &wire,
        }
        .encode(&mut out)
        .unwrap();
        assert_eq!(n, HEADER_LEN + keys::SYNC.as_str().len() + wire.len());
        assert_eq!(&out[..3], b"MW\x01");
        let frame = Frame::decode(&out[..n]).unwrap();
        assert_eq!((frame.seq, frame.key), (7, keys::SYNC));
        assert_eq!(SyncBeacon::decode(frame.payload), Ok(beacon));
        assert!(KeyFilter::Exact(keys::SYNC).matches(frame.key));
        assert!(!KeyFilter::Prefix(Key::from_static("app")).matches(frame.key));
        assert!(KeyFilter::Prefix(Key::from_static("app")).matches(Key::from_static("app/hello")));
    }

    /// The badge's filters deliver sync beacons and app keys with sender
    /// meta, and skip other keys and our own frames.
    #[test]
    fn fr37_badge_filters_select_sync_and_app_keys() {
        let mut me = EspNowTransport::<_, 4, 2>::new(Air::default())
            .with_filters(&ESPNOW_FILTERS)
            .unwrap()
            .with_own_address(ME);
        let mut peer = EspNowTransport::<_, 4, 2>::new(Air::default());
        let wire = SyncBeacon {
            id: 1,
            network_ms: 2,
        }
        .encode();
        ready(peer.publish(Key::from_static("conagotchi/oz"), b"no")).unwrap();
        ready(peer.publish(keys::SYNC, &wire)).unwrap();
        ready(peer.publish(Key::from_static("app/hello"), b"hi")).unwrap();
        ready(peer.publish(Key::from_static("badge/leds"), &[0; 24])).unwrap();
        let mut frames: VecDeque<_> = peer.radio_mut().sent.drain(..).map(|f| (f, PEER)).collect();
        // Our own broadcast echoed back is ignored.
        frames.push_back((frames[1].0.clone(), ME));
        me.radio_mut().inbox = frames;
        let mut buf = [0u8; MAX_FRAME - HEADER_LEN];
        let mut got = Vec::new();
        while let Some(s) = me.try_recv(&mut buf).unwrap() {
            got.push((s.key.as_str().to_owned(), s.payload.to_vec(), s.meta));
        }
        let from = |seq| SampleMeta {
            sender: Some(SenderId::from_mac(PEER)),
            seq: Some(seq),
        };
        assert_eq!(
            got,
            std::vec![
                ("conagotchi/sync".into(), wire.to_vec(), from(1)),
                ("app/hello".into(), b"hi".to_vec(), from(2)),
            ]
        );
    }
}
