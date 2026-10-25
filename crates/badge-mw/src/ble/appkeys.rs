//! Dynamic app-key subscriptions for middleware adverts (pure).
//!
//! The middleware's `AdvertisingSubscriber` is built with a fixed array of
//! keys, and an advert carries only the 16-bit `short_id` of its key, never
//! the key text. Wasm apps subscribe at run time, and the badge must hand
//! them the key text (`app/...`). [`AppKeyTable`] therefore decodes adverts
//! itself with the middleware's public parser (`manufacturer_data`) and
//! `short_id`:
//!
//! - It holds up to `K` key texts, each with its short id. Keys enter it
//!   with [`AppKeyTable::learn`], which accepts only keys an app may use
//!   ([`app_may_use`]): [`AppKeyTable::with_known`] preloads
//!   [`KNOWN_APP_KEYS`], and the firmware also learns every key an app
//!   publishes on (and any key the wasm task reports as subscribed). A full
//!   table refuses new keys ([`KeyTableError::Full`]).
//! - [`carries_company`] is a cheap pre-check for the scan callback, so the
//!   table is only touched for middleware adverts.
//! - [`AppKeyTable::decode`] accepts a report whose first manufacturer data
//!   field has the configured company id and at least `[short id][seq]`, and
//!   whose short id matches a learned key (the first one learned, should two
//!   keys share an id). The sample is fresh unless the last one seen from the
//!   same sender address on the same short id had the same sequence number;
//!   unlike the middleware's `DedupTable` (one entry per key), the sender is
//!   part of the identity, since every badge's sequence starts at 0. The
//!   sender table keeps the `D` most recently inserted pairs.

use heapless::{Deque, String, Vec};
use middleware::Key;
use middleware::ble::advert::manufacturer_data;
use middleware::ble::uuid::short_id;

use super::{APP_KEY_LEN, MW_COMPANY_ID};
use crate::keys::app_may_use;

/// App keys every badge understands without being told (demo apps).
pub const KNOWN_APP_KEYS: [&str; 4] = ["app/hello", "app/ping", "app/chat", "app/color"];
/// Default number of keys held.
pub const APP_KEYS: usize = 8;
/// Default number of `(sender, short id)` pairs remembered for dedup.
pub const SENDERS: usize = 16;

/// True if the first manufacturer data field of `data` is under
/// `company_id` (a cheap pre-check before [`AppKeyTable::decode`]).
pub fn carries_company(data: &[u8], company_id: u16) -> bool {
    matches!(manufacturer_data(data), Ok(Some((c, _))) if c == company_id)
}

/// Why a key was not learned.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KeyTableError {
    /// Not an `app/...` key (see [`app_may_use`]) or longer than
    /// [`APP_KEY_LEN`].
    NotAppKey,
    /// The table holds `K` keys already.
    Full,
}

/// A decoded app sample; `payload` borrows the report.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Decoded<'d> {
    /// Index of the key in the table ([`AppKeyTable::key`]).
    pub slot: usize,
    /// The publisher's sequence number.
    pub sequence: u8,
    /// The payload.
    pub payload: &'d [u8],
}

/// Last sequence seen per sender and short id.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Seen {
    address: [u8; 6],
    id: u16,
    sequence: u8,
}

/// The table (see the module docs).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AppKeyTable<const K: usize = APP_KEYS, const D: usize = SENDERS> {
    company_id: u16,
    keys: Vec<(u16, String<APP_KEY_LEN>), K>,
    seen: Deque<Seen, D>,
}

impl<const K: usize, const D: usize> Default for AppKeyTable<K, D> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const K: usize, const D: usize> AppKeyTable<K, D> {
    /// An empty table accepting company id [`MW_COMPANY_ID`].
    pub const fn new() -> Self {
        Self {
            company_id: MW_COMPANY_ID,
            keys: Vec::new(),
            seen: Deque::new(),
        }
    }

    /// A table preloaded with as many [`KNOWN_APP_KEYS`] as fit.
    pub fn with_known() -> Self {
        KNOWN_APP_KEYS
            .iter()
            .fold(Self::new(), |t, k| t.clone().learn(k).unwrap_or(t))
    }

    /// Accepts manufacturer data under `company_id` instead.
    pub fn with_company_id(self, company_id: u16) -> Self {
        Self { company_id, ..self }
    }

    /// Adds `key` (already present: unchanged).
    ///
    /// # Errors
    ///
    /// [`KeyTableError::NotAppKey`] or [`KeyTableError::Full`].
    pub fn learn(self, key: &str) -> Result<Self, KeyTableError> {
        if !app_may_use(key) {
            return Err(KeyTableError::NotAppKey);
        }
        if self.keys.iter().any(|(_, k)| k.as_str() == key) {
            return Ok(self);
        }
        let id = Key::new(key)
            .map(short_id)
            .map_err(|_| KeyTableError::NotAppKey)?;
        let text = String::try_from(key).map_err(|_| KeyTableError::NotAppKey)?;
        let mut keys = self.keys;
        keys.push((id, text)).map_err(|_| KeyTableError::Full)?;
        Ok(Self { keys, ..self })
    }

    /// The key text in `slot`.
    pub fn key(&self, slot: usize) -> Option<&str> {
        self.keys.get(slot).map(|(_, k)| k.as_str())
    }

    /// Keys held.
    pub fn len(&self) -> usize {
        self.keys.len()
    }

    /// True if no key is held.
    pub fn is_empty(&self) -> bool {
        self.keys.is_empty()
    }

    /// Decodes advertising `data` heard from `address`. Returns the next
    /// table and the sample if it is a fresh one on a learned key.
    pub fn decode<'d>(self, address: [u8; 6], data: &'d [u8]) -> (Self, Option<Decoded<'d>>) {
        let Ok(Some((company, [lo, hi, sequence, payload @ ..]))) = manufacturer_data(data) else {
            return (self, None);
        };
        if company != self.company_id {
            return (self, None);
        }
        let id = u16::from_le_bytes([*lo, *hi]);
        let Some(slot) = self.keys.iter().position(|(k, _)| *k == id) else {
            return (self, None);
        };
        let (next, fresh) = self.observe(address, id, *sequence);
        let sample = fresh.then_some(Decoded {
            slot,
            sequence: *sequence,
            payload,
        });
        (next, sample)
    }

    /// Records `sequence` for `(address, id)`; true if it differs from the
    /// last one recorded.
    fn observe(self, address: [u8; 6], id: u16, sequence: u8) -> (Self, bool) {
        let mut seen = self.seen;
        let entry = Seen {
            address,
            id,
            sequence,
        };
        let fresh = match seen.iter_mut().find(|s| s.address == address && s.id == id) {
            Some(s) if s.sequence == sequence => false,
            Some(s) => {
                *s = entry;
                true
            }
            None => {
                if seen.is_full() {
                    seen.pop_front();
                }
                // Cannot fail: a slot was freed above if the deque was full.
                let _ = seen.push_back(entry);
                true
            }
        };
        (Self { seen, ..self }, fresh)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use crate::ble::advert::{MuxAdvertiser, mw_template};
    use crate::ble::testutil::block_on;
    use middleware::Publisher;
    use middleware::ble::advert::{AdvertisementBuilder, AdvertisingPublisher};

    const A: [u8; 6] = [1; 6];
    const B: [u8; 6] = [2; 6];

    fn advert(company: u16, key: &str, seq: u8, payload: &[u8]) -> std::vec::Vec<u8> {
        let id = short_id(Key::new(key).unwrap()).to_le_bytes();
        let mut body = std::vec![id[0], id[1], seq];
        body.extend_from_slice(payload);
        AdvertisementBuilder::new()
            .company_id(company)
            .manufacturer_data(&body)
            .build()
            .unwrap()
            .as_bytes()
            .to_vec()
    }

    #[test]
    fn fr38_app_keys_learned_dynamically_and_confined_to_app_prefix() {
        let t = AppKeyTable::<2, 4>::new();
        assert_eq!(
            t.clone().learn("conagotchi/sync"),
            Err(KeyTableError::NotAppKey)
        );
        assert_eq!(t.clone().learn("app/a b"), Err(KeyTableError::NotAppKey));
        let t = t.learn("app/x").unwrap().learn("app/x").unwrap();
        assert_eq!(t.len(), 1);
        let t = t.learn("app/y").unwrap();
        assert_eq!(t.clone().learn("app/z"), Err(KeyTableError::Full));
        assert_eq!(t.key(1), Some("app/y"));
        let known = AppKeyTable::<8, 4>::with_known();
        assert_eq!(known.len(), KNOWN_APP_KEYS.len());
        assert_eq!(AppKeyTable::<2, 4>::with_known().len(), 2);
    }

    #[test]
    fn fr38_app_keys_decode_resolves_key_text_and_dedups_per_sender() {
        let t = AppKeyTable::<4, 4>::new().learn("app/hello").unwrap();
        let ad = advert(MW_COMPANY_ID, "app/hello", 0, b"hi");
        let (t, s) = t.decode(A, &ad);
        let s = s.unwrap();
        assert_eq!(
            (t.key(s.slot), s.sequence, s.payload),
            (Some("app/hello"), 0, &b"hi"[..])
        );
        // Same sender, same sequence: a repeat.
        let (t, s) = t.decode(A, &ad);
        assert_eq!(s, None);
        // Another badge also starting at sequence 0 is not a repeat.
        let (t, s) = t.decode(B, &ad);
        assert!(s.is_some());
        let ho = advert(MW_COMPANY_ID, "app/hello", 1, b"ho");
        let (t, s) = t.decode(A, &ho);
        assert_eq!(s.unwrap().payload, b"ho");
        // Unknown key, other company, OZ beacon, malformed: ignored.
        let other = advert(MW_COMPANY_ID, "app/other", 7, b"");
        let (t, s) = t.decode(A, &other);
        assert_eq!(s, None);
        let company = advert(0x0059, "app/hello", 9, b"");
        let (t, s) = t.decode(A, &company);
        assert_eq!(s, None);
        let (t, s) = t.decode(A, &[5, 0xFF, b'O', b'Z', 1, 2]);
        assert_eq!(s, None);
        assert!(carries_company(&ad, MW_COMPANY_ID));
        assert!(!carries_company(&company, MW_COMPANY_ID));
        assert!(!carries_company(&[9, 0xFF], MW_COMPANY_ID));
        let (_, s) = t.decode(A, &[9, 0xFF]);
        assert_eq!(s, None);
    }

    #[test]
    fn fr38_app_keys_sender_table_is_bounded() {
        let mut t = AppKeyTable::<1, 2>::new().learn("app/hello").unwrap();
        let ad = advert(MW_COMPANY_ID, "app/hello", 3, b"");
        for addr in [[1; 6], [2; 6], [3; 6]] {
            let (n, s) = t.decode(addr, &ad);
            assert!(s.is_some());
            t = n;
        }
        // [1;6] was evicted, so its repeat counts as fresh again.
        let (t, s) = t.decode([1; 6], &ad);
        assert!(s.is_some());
        let (_, s) = t.decode([3; 6], &ad);
        assert!(s.is_none());
    }

    /// The badge's own publisher output decodes back to the key text.
    #[test]
    fn fr38_app_keys_decode_middleware_publisher_output() {
        let mut p = AdvertisingPublisher::new(MuxAdvertiser::<4>::new(), mw_template());
        block_on(p.publish(Key::from_static("app/chat"), b"yo")).unwrap();
        let on_air = p
            .advertiser_mut()
            .mux()
            .clone()
            .step(badge_core::io::Millis(0));
        let t = AppKeyTable::<8, 4>::with_known();
        let (t, s) = t.decode(A, on_air.on_air());
        let s = s.unwrap();
        assert_eq!(t.key(s.slot), Some("app/chat"));
        assert_eq!(s.payload, b"yo");
    }
}
