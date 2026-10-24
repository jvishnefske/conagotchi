//! Pure helpers for the firmware's BLE advertising set and the OZ packet
//! forwarder.
//!
//! The badge runs one legacy connectable-scannable advertising set:
//!
//! - its ADV_IND carries exactly the advertising data of the latest
//!   `Effect::Beacon` ([`adv_data`]), or [`DEFAULT_ADV_DATA`] (Flags only)
//!   until the first beacon or after an empty one;
//! - its scan response carries the Complete Local Name ([`scan_response`]),
//!   [`DEFAULT_NAME`] until a `BleAdvertise` request changes it.
//!
//! [`ForwardFilter`] decides which OZ adverts heard by the continuous scan
//! are forwarded to the app as `Input::RadioPacket`: one per address when the
//! payload changed, otherwise at most one per [`FORWARD_INTERVAL_MS`]. Its
//! table is bounded ([`FORWARD_SLOTS`] addresses, least recently forwarded
//! evicted first) and payloads are compared by a 32-bit FNV-1a hash plus
//! length, so a hash collision only delays a forward until the interval
//! passes.

use crate::io::{MAX_ADV_DATA, Millis};
use crate::radio_rank::{AD_COMPLETE_LOCAL_NAME, AD_FLAGS, FLAGS_GENERAL_LE_ONLY, truncate_str};

/// Local name advertised (in the scan response) from boot.
pub const DEFAULT_NAME: &str = "conagotchi";
/// Advertising data used until the first non-empty `Effect::Beacon`: Flags
/// (LE General Discoverable, BR/EDR not supported).
pub const DEFAULT_ADV_DATA: [u8; 3] = [2, AD_FLAGS, FLAGS_GENERAL_LE_ONLY];
/// Longest local name that fits a scan response (31 - 2 header bytes).
pub const MAX_NAME: usize = MAX_ADV_DATA - 2;
/// Minimum time between two forwards of an unchanged payload per address.
pub const FORWARD_INTERVAL_MS: u64 = 250;
/// Addresses tracked by [`ForwardFilter`].
pub const FORWARD_SLOTS: usize = 16;

/// The advertising data to put on air for `beacon`: the beacon itself
/// (cut to 31 bytes), or [`DEFAULT_ADV_DATA`] if it is empty.
pub fn adv_data(beacon: &[u8]) -> &[u8] {
    if beacon.is_empty() {
        &DEFAULT_ADV_DATA
    } else {
        beacon.get(..MAX_ADV_DATA).unwrap_or(beacon)
    }
}

/// Encodes `name` as a scan response holding one Complete Local Name AD
/// structure. Returns the buffer and the number of bytes used; names longer
/// than [`MAX_NAME`] bytes are cut on a character boundary.
pub fn scan_response(name: &str) -> ([u8; MAX_ADV_DATA], usize) {
    let mut buf = [0u8; MAX_ADV_DATA];
    let name: heapless::String<MAX_NAME> = truncate_str(name);
    let bytes = name.as_bytes();
    let end = 2 + bytes.len();
    // bytes.len() <= 29, so the length byte fits and the copy is in bounds.
    buf[0] = u8::try_from(bytes.len() + 1).unwrap_or(u8::MAX);
    buf[1] = AD_COMPLETE_LOCAL_NAME;
    if let Some(dst) = buf.get_mut(2..end) {
        dst.copy_from_slice(bytes);
    }
    (buf, end)
}

/// 32-bit FNV-1a hash of `data`.
fn fnv1a(data: &[u8]) -> u32 {
    data.iter().fold(0x811C_9DC5u32, |h, &b| {
        (h ^ u32::from(b)).wrapping_mul(0x0100_0193)
    })
}

/// One address's last forward.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Forwarded {
    addr: [u8; 6],
    hash: u32,
    len: usize,
    at: u64,
}

/// Rate limiter and deduplicator for OZ adverts (see the module docs).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct ForwardFilter {
    slots: heapless::Vec<Forwarded, FORWARD_SLOTS>,
}

impl ForwardFilter {
    /// An empty filter.
    pub const fn new() -> Self {
        Self {
            slots: heapless::Vec::new(),
        }
    }

    /// Returns true if `payload` from `addr` heard at `now` should be
    /// forwarded, and records the forward. A clock that went backwards
    /// counts as "interval passed".
    pub fn admit(&mut self, addr: [u8; 6], payload: &[u8], now: Millis) -> bool {
        let entry = Forwarded {
            addr,
            hash: fnv1a(payload),
            len: payload.len(),
            at: now.0,
        };
        if let Some(slot) = self.slots.iter_mut().find(|s| s.addr == addr) {
            let same = slot.hash == entry.hash && slot.len == entry.len;
            let recent = now.0 >= slot.at && now.0 - slot.at < FORWARD_INTERVAL_MS;
            if same && recent {
                return false;
            }
            *slot = entry;
            return true;
        }
        if self.slots.is_full() {
            let oldest = self
                .slots
                .iter()
                .enumerate()
                .min_by_key(|(_, s)| s.at)
                .map(|(i, _)| i);
            if let Some(i) = oldest {
                self.slots.swap_remove(i);
            }
        }
        // Cannot fail: a slot was freed above if the table was full.
        let _ = self.slots.push(entry);
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::radio_rank::ad_local_name;

    #[test]
    fn fr29_adv_data_defaults_to_flags() {
        assert_eq!(adv_data(&[]), &DEFAULT_ADV_DATA);
        let beacon = [5, 0xFF, b'O', b'Z', 0, 1];
        assert_eq!(adv_data(&beacon), &beacon);
        let long = [7u8; 40];
        assert_eq!(adv_data(&long).len(), MAX_ADV_DATA);
    }

    #[test]
    fn fr29_scan_response_carries_name() {
        let (buf, len) = scan_response(DEFAULT_NAME);
        assert_eq!(ad_local_name(&buf[..len]), Some(DEFAULT_NAME.as_bytes()));
        let long = "abcdefghijklmnopqrstuvwxyz0123456789";
        let (buf, len) = scan_response(long);
        assert_eq!(len, MAX_ADV_DATA);
        assert_eq!(
            ad_local_name(&buf[..len]),
            Some(&long.as_bytes()[..MAX_NAME])
        );
        let (buf, len) = scan_response("");
        assert_eq!(&buf[..len], &[1, AD_COMPLETE_LOCAL_NAME]);
    }

    #[test]
    fn fr29_forward_dedups_and_rate_limits() {
        let mut f = ForwardFilter::new();
        let a = [1, 2, 3, 4, 5, 6];
        assert!(f.admit(a, b"\x03hello", Millis(1_000)));
        assert!(!f.admit(a, b"\x03hello", Millis(1_100)));
        assert!(!f.admit(a, b"\x03hello", Millis(1_249)));
        assert!(f.admit(a, b"\x03hello", Millis(1_250)));
        // A changed payload is forwarded at once.
        assert!(f.admit(a, b"\x03hellp", Millis(1_260)));
        assert!(!f.admit(a, b"\x03hellp", Millis(1_270)));
        // Another address is independent.
        assert!(f.admit([9; 6], b"\x03hellp", Millis(1_270)));
        // Clock going backwards forwards rather than stalls.
        assert!(f.admit(a, b"\x03hellp", Millis(10)));
    }

    #[test]
    fn fr29_forward_table_is_bounded() {
        let mut f = ForwardFilter::new();
        for i in 0..40u8 {
            assert!(f.admit([i; 6], b"x", Millis(u64::from(i))));
        }
        assert_eq!(f.slots.len(), FORWARD_SLOTS);
        // The newest addresses are still remembered, the oldest were evicted.
        assert!(!f.admit([39; 6], b"x", Millis(50)));
        assert!(f.admit([0; 6], b"x", Millis(50)));
    }
}
