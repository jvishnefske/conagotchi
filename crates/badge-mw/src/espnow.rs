//! ESP-NOW broadcast backend for the middleware [`Publisher`] / [`Subscriber`]
//! traits.
//!
//! # Frame format (version 1)
//!
//! Every publication is one broadcast ESP-NOW frame:
//!
//! | bytes | field                                                        |
//! |-------|--------------------------------------------------------------|
//! | 2     | magic [`MAGIC`] (`MW`)                                       |
//! | 1     | version [`VERSION`] (1)                                      |
//! | 2     | `short_id(key)`, u16 little-endian (the middleware BLE hash) |
//! | 1     | sequence number of the sender (wrapping)                     |
//! | 1     | key length `n` (1..=[`MAX_KEY`])                             |
//! | n     | key bytes (a valid middleware [`Key`])                       |
//! | rest  | payload (0..=[`MAX_PAYLOAD`] bytes)                          |
//!
//! The fixed header is [`HEADER_LEN`] (7) bytes and the key is carried in
//! full: 16-bit short ids can collide and a receiver needs the key text to
//! build a [`Sample`]. The overhead per frame is therefore `7 + key length`
//! bytes (22 for `conagotchi/sync`). The short id is still sent: a receiver
//! checks it against the key as a cheap integrity check, and it keeps the
//! header aligned with the BLE advertising backend. Keys on this transport
//! are limited to [`MAX_KEY`] bytes (the wasm bus limit) so that every
//! payload up to [`MAX_PAYLOAD`] fits one ESP-NOW v1 frame ([`MAX_FRAME`] =
//! 250 bytes) whatever the key; the limits are enforced strictly, not per
//! key length, so what fits never depends on the key.
//!
//! # Transport
//!
//! [`EspNowTransport`] owns an [`EspNowRadio`] and implements both traits:
//!
//! - **Publish** encodes a frame carrying the transport's sequence number
//!   and broadcasts it; the number advances (wrapping) only after a
//!   successful broadcast, so it is per sender, shared by all keys.
//! - **Subscribe** to a fixed key slice (`&'a [Key<'a>]`, mirroring the
//!   middleware's `AdvertisingSubscriber`) plus optional key prefixes
//!   ([`EspNowTransport::with_prefixes`]) for dynamic key sets such as wasm
//!   app keys.
//! - **Dedupe**: a bounded [`SeenTable`] of `K` entries keyed by
//!   `(source MAC, key hash)` holding the last sequence number; a frame
//!   repeating it is dropped. When the table is full the least recently
//!   updated entry is replaced. A sender that restarts may lose one frame
//!   if its first sequence number equals the last one remembered for it.
//! - **Ignored frames**: frames without the `MW` magic (other ESP-NOW
//!   users), from our own address ([`EspNowTransport::with_own_address`]),
//!   on keys not subscribed, or repeats, are skipped; after
//!   [`MAX_IGNORED_FRAMES`] of them in one `recv` it returns
//!   [`EspNowError::IgnoredLimit`] so a call is always bounded in frames.
//! - **Errors**: a frame with the `MW` magic that does not decode is
//!   returned as [`EspNowError::Frame`] (so the caller can count it); a
//!   sample that does not fit the caller's buffer is
//!   [`EspNowError::BufferTooSmall`]; radio failures are
//!   [`EspNowError::Radio`]; an oversized key or payload on publish is
//!   [`EspNowError::Frame`] with [`FrameError::KeyTooLong`] or
//!   [`FrameError::PayloadTooLarge`].
//!
//! The middleware items a user of this module needs (`Key`, `Publisher`,
//! `Subscriber`, `Sample`) are re-exported, so the firmware needs no direct
//! `middleware` dependency.
//!
//! [`EspNowTransport::recv_from`] also returns the sender's MAC and the
//! frame's sequence number, which the `Subscriber` trait cannot express.
//!
//! Everything here is pure except the radio calls; the transport is tested
//! on the host with an in-memory fake radio.

use middleware::ble::uuid::short_id;
use middleware::transport::sample_into;
pub use middleware::{Key, Publisher, Sample, Subscriber};

/// First two bytes of every middleware ESP-NOW frame.
pub const MAGIC: [u8; 2] = *b"MW";
/// Frame format version.
pub const VERSION: u8 = 1;
/// Fixed header: magic, version, short id, sequence, key length.
pub const HEADER_LEN: usize = 7;
/// Largest ESP-NOW v1 frame.
pub const MAX_FRAME: usize = 250;
/// Longest key this transport carries.
pub const MAX_KEY: usize = 64;
/// Largest payload this transport carries (fits with a [`MAX_KEY`] key).
pub const MAX_PAYLOAD: usize = MAX_FRAME - HEADER_LEN - MAX_KEY;
/// Frames skipped in one `recv` before it gives up.
pub const MAX_IGNORED_FRAMES: usize = 256;

/// A 48-bit ESP-NOW (Wi-Fi MAC) address.
pub type Mac = [u8; 6];

/// Why bytes are not a valid frame, or a publication cannot be framed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// Shorter than the header plus the declared key.
    TooShort,
    /// Longer than [`MAX_FRAME`].
    TooLong,
    /// The first two bytes are not [`MAGIC`].
    BadMagic,
    /// A version this code does not understand.
    BadVersion(u8),
    /// Key length 0 or above [`MAX_KEY`] in a received frame.
    BadKeyLength,
    /// The key bytes are not a valid middleware key.
    BadKey,
    /// The short id does not match the key.
    IdMismatch,
    /// Publishing a key longer than [`MAX_KEY`].
    KeyTooLong,
    /// Publishing a payload longer than [`MAX_PAYLOAD`].
    PayloadTooLarge,
}

/// One decoded frame; `key` and `payload` borrow the frame bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame<'a> {
    /// The sender's sequence number.
    pub seq: u8,
    /// The key the payload was published on.
    pub key: Key<'a>,
    /// The payload.
    pub payload: &'a [u8],
}

impl<'a> Frame<'a> {
    /// Writes this frame into `out` and returns its length.
    ///
    /// # Errors
    ///
    /// [`FrameError::KeyTooLong`] or [`FrameError::PayloadTooLarge`] when the
    /// key or payload exceed this transport's limits.
    pub fn encode(&self, out: &mut [u8; MAX_FRAME]) -> Result<usize, FrameError> {
        let key = self.key.as_bytes();
        if key.len() > MAX_KEY {
            return Err(FrameError::KeyTooLong);
        }
        if self.payload.len() > MAX_PAYLOAD {
            return Err(FrameError::PayloadTooLarge);
        }
        let key_len = u8::try_from(key.len()).map_err(|_| FrameError::KeyTooLong)?;
        let id = short_id(self.key).to_le_bytes();
        let header = [MAGIC[0], MAGIC[1], VERSION, id[0], id[1], self.seq, key_len];
        let n = HEADER_LEN + key.len() + self.payload.len();
        let slot = out.get_mut(..n).ok_or(FrameError::PayloadTooLarge)?;
        let (head, rest) = slot.split_at_mut(HEADER_LEN);
        let (key_slot, payload_slot) = rest.split_at_mut(key.len());
        head.copy_from_slice(&header);
        key_slot.copy_from_slice(key);
        payload_slot.copy_from_slice(self.payload);
        Ok(n)
    }

    /// Parses a received frame.
    ///
    /// # Errors
    ///
    /// Any [`FrameError`] other than the publish-only `KeyTooLong` and
    /// `PayloadTooLarge`.
    pub fn decode(bytes: &'a [u8]) -> Result<Frame<'a>, FrameError> {
        if bytes.len() > MAX_FRAME {
            return Err(FrameError::TooLong);
        }
        let [m0, m1, rest @ ..] = bytes else {
            return Err(FrameError::TooShort);
        };
        if [*m0, *m1] != MAGIC {
            return Err(FrameError::BadMagic);
        }
        let [version, id_lo, id_hi, seq, key_len, body @ ..] = rest else {
            return Err(FrameError::TooShort);
        };
        if *version != VERSION {
            return Err(FrameError::BadVersion(*version));
        }
        let key_len = usize::from(*key_len);
        if key_len == 0 || key_len > MAX_KEY {
            return Err(FrameError::BadKeyLength);
        }
        let (key, payload) = body.split_at_checked(key_len).ok_or(FrameError::TooShort)?;
        let key = core::str::from_utf8(key).map_err(|_| FrameError::BadKey)?;
        let key = Key::new(key).map_err(|_| FrameError::BadKey)?;
        if short_id(key) != u16::from_le_bytes([*id_lo, *id_hi]) {
            return Err(FrameError::IdMismatch);
        }
        Ok(Frame {
            seq: *seq,
            key,
            payload,
        })
    }
}

/// 32-bit FNV-1a of a key, identifying it in the [`SeenTable`].
fn key_hash(key: Key<'_>) -> u32 {
    key.as_bytes().iter().fold(0x811c_9dc5_u32, |h, b| {
        (h ^ u32::from(*b)).wrapping_mul(0x0100_0193)
    })
}

/// One remembered `(source, key)` and its last sequence number.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Seen {
    src: Mac,
    key: u32,
    seq: u8,
    /// Value of the table's clock when last updated (for replacement).
    stamp: u32,
}

/// Last sequence number seen per `(source MAC, key)`, `K` entries.
///
/// Immutable: [`SeenTable::observe`] returns a new table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SeenTable<const K: usize> {
    entries: [Option<Seen>; K],
    clock: u32,
}

impl<const K: usize> Default for SeenTable<K> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const K: usize> SeenTable<K> {
    /// A table that has seen nothing.
    pub const fn new() -> Self {
        SeenTable {
            entries: [None; K],
            clock: 0,
        }
    }

    /// Records `seq` for `(src, key)`. Returns the new table and whether the
    /// frame is fresh (not a repeat of the last sequence number for that
    /// pair). A new pair takes a free entry or replaces the least recently
    /// updated one; with `K == 0` every frame is fresh.
    pub fn observe(self, src: Mac, key: Key<'_>, seq: u8) -> (Self, bool) {
        let key = key_hash(key);
        let clock = self.clock.wrapping_add(1);
        let mut entries = self.entries;
        let existing = entries
            .iter()
            .position(|e| matches!(e, Some(s) if s.src == src && s.key == key));
        if let Some(i) = existing
            && matches!(entries.get(i), Some(Some(s)) if s.seq == seq)
        {
            return (self, false);
        }
        let slot = existing
            .or_else(|| entries.iter().position(Option::is_none))
            .or_else(|| {
                entries
                    .iter()
                    .enumerate()
                    .max_by_key(|(_, e)| e.map_or(u32::MAX, |s| clock.wrapping_sub(s.stamp)))
                    .map(|(i, _)| i)
            });
        if let Some(entry) = slot.and_then(|i| entries.get_mut(i)) {
            *entry = Some(Seen {
                src,
                key,
                seq,
                stamp: clock,
            });
        }
        (SeenTable { entries, clock }, true)
    }
}

/// Radio side of the transport: broadcast frames and receive them.
///
/// Implementations copy a received frame into `buf` (the transport offers
/// [`MAX_FRAME`] bytes) and report one that does not fit as their own
/// error, never truncating.
pub trait EspNowRadio {
    /// Binding-specific failure.
    type Error: core::fmt::Debug;

    /// Broadcasts one frame to every listening station.
    async fn broadcast(&mut self, frame: &[u8]) -> Result<(), Self::Error>;

    /// Waits for the next frame, copies it into `buf` and returns it with
    /// the sender's MAC.
    async fn receive<'b>(&mut self, buf: &'b mut [u8]) -> Result<(&'b [u8], Mac), Self::Error>;
}

/// Failure of the ESP-NOW transport.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EspNowError<E> {
    /// The radio failed.
    Radio(E),
    /// A received `MW` frame was malformed, or a publication cannot be
    /// framed (key or payload too large).
    Frame(FrameError),
    /// The caller's buffer cannot hold the sample's key and payload.
    BufferTooSmall,
    /// [`MAX_IGNORED_FRAMES`] frames passed without a matching one.
    IgnoredLimit,
}

/// A received sample with what the `Subscriber` trait cannot carry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Received<'b> {
    /// Key and payload, borrowing the caller's buffer.
    pub sample: Sample<'b>,
    /// The sender's MAC.
    pub src: Mac,
    /// The sender's sequence number.
    pub seq: u8,
}

/// Middleware publisher and subscriber over ESP-NOW broadcast, remembering
/// `K` `(sender, key)` pairs for duplicate suppression.
pub struct EspNowTransport<'a, R, const K: usize> {
    radio: R,
    keys: &'a [Key<'a>],
    prefixes: &'a [&'a str],
    own: Option<Mac>,
    seq: u8,
    seen: SeenTable<K>,
}

impl<'a, R: EspNowRadio, const K: usize> EspNowTransport<'a, R, K> {
    /// A transport over `radio` subscribed to exactly `keys`.
    pub const fn new(radio: R, keys: &'a [Key<'a>]) -> Self {
        EspNowTransport {
            radio,
            keys,
            prefixes: &[],
            own: None,
            seq: 0,
            seen: SeenTable::new(),
        }
    }

    /// Also subscribes to every key starting with one of `prefixes`.
    pub fn with_prefixes(self, prefixes: &'a [&'a str]) -> Self {
        EspNowTransport { prefixes, ..self }
    }

    /// Ignores frames whose source is `mac` (our own station address).
    pub fn with_own_address(self, mac: Mac) -> Self {
        EspNowTransport {
            own: Some(mac),
            ..self
        }
    }

    /// The sequence number the next publish will carry.
    pub const fn sequence(&self) -> u8 {
        self.seq
    }

    /// Borrows the radio.
    pub fn radio_mut(&mut self) -> &mut R {
        &mut self.radio
    }

    /// True if `key` is one of the subscribed keys or starts with a
    /// subscribed prefix.
    pub fn subscribed(&self, key: Key<'_>) -> bool {
        self.keys.iter().any(|k| k.as_str() == key.as_str())
            || self.prefixes.iter().any(|p| key.as_str().starts_with(p))
    }

    /// Waits for the next fresh sample on a subscribed key and writes it into
    /// `buf`, also returning the sender's MAC and sequence number.
    ///
    /// # Errors
    ///
    /// See [`EspNowError`]; after an error the transport is still usable.
    pub async fn recv_from<'b>(
        &mut self,
        buf: &'b mut [u8],
    ) -> Result<Received<'b>, EspNowError<R::Error>> {
        let mut raw = [0u8; MAX_FRAME];
        for _ in 0..MAX_IGNORED_FRAMES {
            let (bytes, src) = self
                .radio
                .receive(&mut raw)
                .await
                .map_err(EspNowError::Radio)?;
            if self.own == Some(src) {
                continue;
            }
            let frame = match Frame::decode(bytes) {
                Ok(frame) => frame,
                Err(_) if !bytes.starts_with(&MAGIC) => continue,
                Err(e) => return Err(EspNowError::Frame(e)),
            };
            if !self.subscribed(frame.key) {
                continue;
            }
            let (seen, fresh) = self.seen.observe(src, frame.key, frame.seq);
            self.seen = seen;
            if !fresh {
                continue;
            }
            let sample = sample_into(buf, frame.key.as_str(), frame.payload)
                .ok_or(EspNowError::BufferTooSmall)?;
            return Ok(Received {
                sample,
                src,
                seq: frame.seq,
            });
        }
        Err(EspNowError::IgnoredLimit)
    }
}

impl<R: EspNowRadio, const K: usize> Publisher for EspNowTransport<'_, R, K> {
    type Error = EspNowError<R::Error>;

    async fn publish(&mut self, key: Key<'_>, payload: &[u8]) -> Result<(), Self::Error> {
        let mut out = [0u8; MAX_FRAME];
        let n = Frame {
            seq: self.seq,
            key,
            payload,
        }
        .encode(&mut out)
        .map_err(EspNowError::Frame)?;
        let frame = out
            .get(..n)
            .ok_or(EspNowError::Frame(FrameError::TooLong))?;
        self.radio
            .broadcast(frame)
            .await
            .map_err(EspNowError::Radio)?;
        self.seq = self.seq.wrapping_add(1);
        Ok(())
    }
}

impl<R: EspNowRadio, const K: usize> Subscriber for EspNowTransport<'_, R, K> {
    type Error = EspNowError<R::Error>;

    async fn recv<'b>(&mut self, buf: &'b mut [u8]) -> Result<Sample<'b>, Self::Error> {
        self.recv_from(buf).await.map(|r| r.sample)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use core::pin::pin;
    use core::task::{Context, Poll, Waker};
    use heapless::{Deque, Vec};
    use std::vec::Vec as StdVec;

    const A: Mac = [2, 0, 0, 0, 0, 0xA];
    const B: Mac = [2, 0, 0, 0, 0, 0xB];
    const SYNC: Key<'static> = Key::from_static("conagotchi/sync");
    const HELLO: Key<'static> = Key::from_static("app/hello");
    const OTHER: Key<'static> = Key::from_static("other/thing");

    /// Polls a future that never really waits (the fake radio is always
    /// ready) to completion.
    fn block_on<F: Future>(f: F) -> F::Output {
        let mut f = pin!(f);
        let mut cx = Context::from_waker(Waker::noop());
        for _ in 0..1000 {
            if let Poll::Ready(out) = f.as_mut().poll(&mut cx) {
                return out;
            }
        }
        panic!("future did not complete");
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum FakeError {
        /// Nothing queued to receive.
        Empty,
        /// The frame does not fit the receive buffer.
        TooLarge,
    }

    /// In-memory radio: `inbox` feeds `receive`, `sent` collects broadcasts.
    #[derive(Default)]
    struct FakeRadio {
        inbox: Deque<(Vec<u8, 300>, Mac), 64>,
        sent: StdVec<StdVec<u8>>,
    }

    impl FakeRadio {
        fn push(&mut self, frame: &[u8], src: Mac) {
            self.inbox
                .push_back((Vec::from_slice(frame).unwrap(), src))
                .unwrap();
        }
    }

    impl EspNowRadio for FakeRadio {
        type Error = FakeError;

        async fn broadcast(&mut self, frame: &[u8]) -> Result<(), FakeError> {
            self.sent.push(frame.to_vec());
            Ok(())
        }

        async fn receive<'b>(&mut self, buf: &'b mut [u8]) -> Result<(&'b [u8], Mac), FakeError> {
            let (frame, src) = self.inbox.pop_front().ok_or(FakeError::Empty)?;
            let slot = buf.get_mut(..frame.len()).ok_or(FakeError::TooLarge)?;
            slot.copy_from_slice(&frame);
            Ok((slot, src))
        }
    }

    fn frame(seq: u8, key: Key<'_>, payload: &[u8]) -> StdVec<u8> {
        let mut out = [0u8; MAX_FRAME];
        let n = Frame { seq, key, payload }.encode(&mut out).unwrap();
        out[..n].to_vec()
    }

    #[test]
    fn fr37_frame_round_trip_and_layout() {
        let bytes = frame(7, SYNC, b"OZL1abcdefgh");
        assert_eq!(&bytes[..3], b"MW\x01");
        assert_eq!(u16::from_le_bytes([bytes[3], bytes[4]]), short_id(SYNC));
        assert_eq!(bytes[5], 7);
        assert_eq!(usize::from(bytes[6]), SYNC.as_str().len());
        assert_eq!(bytes.len(), HEADER_LEN + SYNC.as_str().len() + 12);
        let f = Frame::decode(&bytes).unwrap();
        assert_eq!(f.seq, 7);
        assert_eq!(f.key, SYNC);
        assert_eq!(f.payload, b"OZL1abcdefgh");
    }

    #[test]
    fn fr37_largest_frame_fits_espnow_v1() {
        let key_text = [b'k'; MAX_KEY];
        let key = Key::new(core::str::from_utf8(&key_text).unwrap()).unwrap();
        let bytes = frame(0, key, &[0xAB; MAX_PAYLOAD]);
        assert_eq!(bytes.len(), MAX_FRAME);
        assert_eq!(Frame::decode(&bytes).unwrap().payload.len(), MAX_PAYLOAD);
    }

    #[test]
    fn fr37_oversized_publications_are_refused() {
        let mut out = [0u8; MAX_FRAME];
        let long = Frame {
            seq: 0,
            key: HELLO,
            payload: &[0; MAX_PAYLOAD + 1],
        };
        assert_eq!(long.encode(&mut out), Err(FrameError::PayloadTooLarge));
        let key_text = [b'k'; MAX_KEY + 1];
        let key = Key::new(core::str::from_utf8(&key_text).unwrap()).unwrap();
        let long_key = Frame {
            seq: 0,
            key,
            payload: b"",
        };
        assert_eq!(long_key.encode(&mut out), Err(FrameError::KeyTooLong));

        let mut t: EspNowTransport<'_, _, 4> = EspNowTransport::new(FakeRadio::default(), &[]);
        let r = block_on(t.publish(HELLO, &[0; MAX_PAYLOAD + 1]));
        assert_eq!(r, Err(EspNowError::Frame(FrameError::PayloadTooLarge)));
        assert!(t.radio_mut().sent.is_empty());
        assert_eq!(t.sequence(), 0);
    }

    #[test]
    fn fr37_malformed_frames_are_rejected() {
        let good = frame(1, SYNC, b"x");
        assert_eq!(Frame::decode(b""), Err(FrameError::TooShort));
        assert_eq!(Frame::decode(b"OZL1abcdefgh"), Err(FrameError::BadMagic));
        assert_eq!(Frame::decode(&good[..5]), Err(FrameError::TooShort));
        assert_eq!(Frame::decode(&good[..10]), Err(FrameError::TooShort));
        let mut v = good.clone();
        v[2] = 2;
        assert_eq!(Frame::decode(&v), Err(FrameError::BadVersion(2)));
        let mut v = good.clone();
        v[6] = 0;
        assert_eq!(Frame::decode(&v), Err(FrameError::BadKeyLength));
        let mut v = good.clone();
        v[6] = (MAX_KEY + 1) as u8;
        assert_eq!(Frame::decode(&v), Err(FrameError::BadKeyLength));
        let mut v = good.clone();
        v[HEADER_LEN] = b' ';
        assert_eq!(Frame::decode(&v), Err(FrameError::BadKey));
        let mut v = good.clone();
        v[3] ^= 1;
        assert_eq!(Frame::decode(&v), Err(FrameError::IdMismatch));
        assert_eq!(
            Frame::decode(&[b'M'; MAX_FRAME + 1]),
            Err(FrameError::TooLong)
        );
    }

    #[test]
    fn fr37_malformed_mw_frame_is_an_error_foreign_frames_are_skipped() {
        let mut radio = FakeRadio::default();
        radio.push(b"OZL1abcdefgh", A); // foreign (old sync beacon): skipped
        let mut bad = frame(1, SYNC, b"x");
        bad[3] ^= 1;
        radio.push(&bad, A);
        radio.push(&frame(2, SYNC, b"y"), A);
        let keys = [SYNC];
        let mut t: EspNowTransport<'_, _, 4> = EspNowTransport::new(radio, &keys);
        let mut buf = [0u8; 64];
        assert_eq!(
            block_on(t.recv(&mut buf)),
            Err(EspNowError::Frame(FrameError::IdMismatch))
        );
        let s = block_on(t.recv(&mut buf)).unwrap();
        assert_eq!(s.payload, b"y");
    }

    #[test]
    fn fr37_round_trip_through_subscriber_reports_source_and_seq() {
        let mut radio = FakeRadio::default();
        radio.push(&frame(9, SYNC, b"beacon"), A);
        let keys = [SYNC];
        let mut t: EspNowTransport<'_, _, 4> = EspNowTransport::new(radio, &keys);
        let mut buf = [0u8; 64];
        let r = block_on(t.recv_from(&mut buf)).unwrap();
        assert_eq!(r.sample.key, SYNC);
        assert_eq!(r.sample.payload, b"beacon");
        assert_eq!(r.src, A);
        assert_eq!(r.seq, 9);
        assert_eq!(
            block_on(t.recv(&mut buf)),
            Err(EspNowError::Radio(FakeError::Empty))
        );
    }

    #[test]
    fn fr37_two_subscribers_filter_by_key_and_prefix() {
        let frames = [
            frame(0, SYNC, b"s"),
            frame(1, HELLO, b"h"),
            frame(2, OTHER, b"o"),
        ];
        let mut ra = FakeRadio::default();
        let mut rb = FakeRadio::default();
        for f in &frames {
            ra.push(f, A);
            rb.push(f, A);
        }
        let sync_only = [SYNC];
        let mut sa: EspNowTransport<'_, _, 4> = EspNowTransport::new(ra, &sync_only);
        let prefixes = ["app/"];
        let mut sb: EspNowTransport<'_, _, 4> =
            EspNowTransport::new(rb, &[]).with_prefixes(&prefixes);
        let mut buf = [0u8; 64];
        assert_eq!(block_on(sa.recv(&mut buf)).unwrap().payload, b"s");
        assert!(matches!(
            block_on(sa.recv(&mut buf)),
            Err(EspNowError::Radio(FakeError::Empty))
        ));
        let s = block_on(sb.recv(&mut buf)).unwrap();
        assert_eq!((s.key, s.payload), (HELLO, &b"h"[..]));
        assert!(matches!(
            block_on(sb.recv(&mut buf)),
            Err(EspNowError::Radio(FakeError::Empty))
        ));
        assert!(sb.subscribed(Key::from_static("app/x/y")));
        assert!(!sb.subscribed(SYNC));
    }

    #[test]
    fn fr37_duplicates_are_dropped_per_source_and_key() {
        let mut radio = FakeRadio::default();
        let f = frame(5, SYNC, b"1");
        radio.push(&f, A);
        radio.push(&f, A); // repeat: dropped
        radio.push(&f, B); // same seq, other sender: fresh
        radio.push(&frame(5, HELLO, b"2"), A); // same seq, other key: fresh
        radio.push(&frame(6, SYNC, b"3"), A); // next seq: fresh
        let keys = [SYNC, HELLO];
        let mut t: EspNowTransport<'_, _, 4> = EspNowTransport::new(radio, &keys);
        let mut buf = [0u8; 64];
        let mut got = StdVec::new();
        while let Ok(r) = block_on(t.recv_from(&mut buf)) {
            got.push((r.src, r.sample.payload.to_vec()));
        }
        assert_eq!(
            got,
            [
                (A, b"1".to_vec()),
                (B, b"1".to_vec()),
                (A, b"2".to_vec()),
                (A, b"3".to_vec())
            ]
        );
    }

    #[test]
    fn fr37_seen_table_replaces_least_recent_entry() {
        let t: SeenTable<2> = SeenTable::new();
        let (t, fresh) = t.observe(A, SYNC, 1);
        assert!(fresh);
        let (t, fresh) = t.observe(B, SYNC, 1);
        assert!(fresh);
        let (t, fresh) = t.observe(A, SYNC, 2); // A is now the most recent
        assert!(fresh);
        let (t, fresh) = t.observe(A, HELLO, 1); // evicts B
        assert!(fresh);
        let (t, fresh) = t.observe(A, SYNC, 2);
        assert!(!fresh, "A/SYNC kept");
        let (_, fresh) = t.observe(B, SYNC, 1);
        assert!(fresh, "B/SYNC was evicted");
        let none: SeenTable<0> = SeenTable::new();
        let (none, f1) = none.observe(A, SYNC, 1);
        let (_, f2) = none.observe(A, SYNC, 1);
        assert!(f1 && f2);
    }

    #[test]
    fn fr37_own_frames_and_ignored_limit() {
        let mut radio = FakeRadio::default();
        radio.push(&frame(0, SYNC, b"me"), A);
        radio.push(&frame(0, SYNC, b"peer"), B);
        let keys = [SYNC];
        let mut t: EspNowTransport<'_, _, 4> =
            EspNowTransport::new(radio, &keys).with_own_address(A);
        let mut buf = [0u8; 64];
        assert_eq!(block_on(t.recv(&mut buf)).unwrap().payload, b"peer");

        let mut radio = FakeRadio::default();
        for _ in 0..MAX_IGNORED_FRAMES.min(64) {
            radio.push(&frame(0, OTHER, b""), B);
        }
        let mut t: EspNowTransport<'_, _, 4> = EspNowTransport::new(radio, &keys);
        // 64 ignored frames, then the radio runs dry: the loop is bounded and
        // the radio error surfaces.
        assert_eq!(
            block_on(t.recv(&mut buf)),
            Err(EspNowError::Radio(FakeError::Empty))
        );
    }

    #[test]
    fn fr37_ignored_limit_is_reported() {
        /// A radio that always delivers an unsubscribed frame.
        struct Noisy(StdVec<u8>, usize);
        impl EspNowRadio for Noisy {
            type Error = FakeError;
            async fn broadcast(&mut self, _: &[u8]) -> Result<(), FakeError> {
                Ok(())
            }
            async fn receive<'b>(
                &mut self,
                buf: &'b mut [u8],
            ) -> Result<(&'b [u8], Mac), FakeError> {
                self.1 += 1;
                let slot = buf.get_mut(..self.0.len()).ok_or(FakeError::TooLarge)?;
                slot.copy_from_slice(&self.0);
                Ok((slot, B))
            }
        }
        let keys = [SYNC];
        let mut t: EspNowTransport<'_, _, 4> =
            EspNowTransport::new(Noisy(frame(0, OTHER, b""), 0), &keys);
        let mut buf = [0u8; 64];
        assert_eq!(block_on(t.recv(&mut buf)), Err(EspNowError::IgnoredLimit));
        assert_eq!(t.radio_mut().1, MAX_IGNORED_FRAMES);
    }

    #[test]
    fn fr37_buffer_too_small() {
        let mut radio = FakeRadio::default();
        radio.push(&frame(0, SYNC, b"0123456789"), A);
        let keys = [SYNC];
        let mut t: EspNowTransport<'_, _, 4> = EspNowTransport::new(radio, &keys);
        let mut buf = [0u8; 20]; // key (15) fits, payload (10) does not
        assert_eq!(block_on(t.recv(&mut buf)), Err(EspNowError::BufferTooSmall));
    }

    #[test]
    fn fr37_end_to_end_publish_to_subscribe() {
        let mut publisher: EspNowTransport<'_, _, 4> =
            EspNowTransport::new(FakeRadio::default(), &[]);
        block_on(publisher.publish(SYNC, b"OZL1\x01\0\0\0\x02\0\0\0")).unwrap();
        block_on(publisher.publish(HELLO, b"hi")).unwrap();
        block_on(publisher.publish(OTHER, b"no")).unwrap();
        assert_eq!(publisher.sequence(), 3);
        let sent = core::mem::take(&mut publisher.radio_mut().sent);
        assert_eq!(sent.len(), 3);

        let mut radio = FakeRadio::default();
        for f in &sent {
            radio.push(f, A);
        }
        let keys = [SYNC];
        let prefixes = ["app/"];
        let mut sub: EspNowTransport<'_, _, 8> =
            EspNowTransport::new(radio, &keys).with_prefixes(&prefixes);
        let mut buf = [0u8; 128];
        let r = block_on(sub.recv_from(&mut buf)).unwrap();
        assert_eq!((r.sample.key, r.seq), (SYNC, 0));
        assert_eq!(r.sample.payload, b"OZL1\x01\0\0\0\x02\0\0\0");
        let r = block_on(sub.recv_from(&mut buf)).unwrap();
        assert_eq!(
            (r.sample.key, r.sample.payload, r.seq),
            (HELLO, &b"hi"[..], 1)
        );
        assert!(block_on(sub.recv_from(&mut buf)).is_err());
    }
}
