//! Pure, hardware-free helpers for turning raw radio observations into the
//! protocol result lists.
//!
//! Everything here is deterministic and bounded: inputs are slices or
//! iterators that are explicitly `take`n to a fixed limit, outputs are
//! fixed-capacity `heapless` collections. Nothing touches the radio, so the
//! module is unit-tested on the host by `cargo test -p badge-core`. The
//! firmware's `radio::rank` module re-exports it.

use badge_proto::{AccessPoint, BleDevice};
use heapless::{String, Vec};

/// Advertising-data type: Shortened Local Name (Bluetooth Core Supplement, Part A, 1.2).
pub const AD_SHORTENED_LOCAL_NAME: u8 = 0x08;
/// Advertising-data type: Complete Local Name.
pub const AD_COMPLETE_LOCAL_NAME: u8 = 0x09;
/// Advertising-data type: Flags.
pub const AD_FLAGS: u8 = 0x01;
/// Flags value: LE General Discoverable Mode | BR/EDR Not Supported.
pub const FLAGS_GENERAL_LE_ONLY: u8 = 0x06;
/// Size of a legacy advertising PDU payload.
pub const LEGACY_ADV_LEN: usize = 31;

/// Truncates `s` to at most `N` bytes, cutting on a UTF-8 character boundary.
///
/// Never fails: an over-long string is shortened rather than rejected.
pub fn truncate_str<const N: usize>(s: &str) -> String<N> {
    let mut end = s.len().min(N);
    // A char is at most 4 bytes, so this walks back at most 3 times.
    while end > 0 && !s.is_char_boundary(end) {
        end -= 1;
    }
    let mut out = String::new();
    // Cannot fail: `end <= N` and `s[..end]` is valid UTF-8.
    let _ = out.push_str(s.get(..end).unwrap_or(""));
    out
}

/// Decodes `bytes` as UTF-8, keeping the longest valid prefix, then truncates
/// it to `N` bytes on a character boundary.
pub fn lossy_utf8<const N: usize>(bytes: &[u8]) -> String<N> {
    let valid = match core::str::from_utf8(bytes) {
        Ok(s) => s,
        Err(e) => bytes
            .get(..e.valid_up_to())
            .and_then(|b| core::str::from_utf8(b).ok())
            .unwrap_or(""),
    };
    truncate_str(valid)
}

/// Keeps the `N` strongest access points from `aps`, strongest first.
///
/// At most `limit` input items are examined, which bounds the work even for a
/// misbehaving iterator. Ties keep scan order (stable).
pub fn rank_aps<const N: usize>(
    aps: impl Iterator<Item = AccessPoint>,
    limit: usize,
) -> Vec<AccessPoint, N> {
    let mut out: Vec<AccessPoint, N> = Vec::new();
    for ap in aps.take(limit) {
        insert_ranked(&mut out, ap, |a| a.rssi);
    }
    out
}

/// Inserts `item` into `list` (sorted by `key` descending), evicting the
/// weakest entry when full. Items weaker than every entry of a full list are
/// dropped. Equal keys keep insertion order.
fn insert_ranked<T, const N: usize>(list: &mut Vec<T, N>, item: T, key: impl Fn(&T) -> i32) {
    let k = key(&item);
    let pos = list.iter().position(|e| key(e) < k).unwrap_or(list.len());
    if pos >= N {
        return;
    }
    if list.is_full() {
        let _ = list.pop();
    }
    // Cannot fail: there is room and `pos <= len`.
    let _ = list.insert(pos, item);
}

/// Finds the local name in legacy advertising data.
///
/// Prefers the Complete Local Name (0x09) over the Shortened Local Name
/// (0x08). Malformed or truncated AD structures end the walk; the loop is
/// bounded because every step advances by at least one byte.
pub fn ad_local_name(data: &[u8]) -> Option<&[u8]> {
    let mut rest = data;
    let mut shortened = None;
    while let Some((&len, tail)) = rest.split_first() {
        let len = usize::from(len);
        if len == 0 {
            // Zero-length structure marks early termination of significant data.
            break;
        }
        let Some(structure) = tail.get(..len) else {
            break;
        };
        rest = tail.get(len..).unwrap_or(&[]);
        if let Some((&kind, payload)) = structure.split_first() {
            match kind {
                AD_COMPLETE_LOCAL_NAME => return Some(payload),
                AD_SHORTENED_LOCAL_NAME => shortened = Some(payload),
                _ => {}
            }
        }
    }
    shortened
}

/// Encodes connectable legacy advertising data: Flags (LE General
/// Discoverable, BR/EDR not supported) followed by the Complete Local Name.
///
/// Returns the buffer and the number of bytes used. The 31-byte PDU minus the
/// 3-byte Flags structure and the 2-byte name header leaves 26 bytes, so
/// longer names are truncated on a character boundary.
pub fn encode_adv_data(name: &str) -> ([u8; LEGACY_ADV_LEN], usize) {
    let mut buf = [0u8; LEGACY_ADV_LEN];
    let name: String<{ LEGACY_ADV_LEN - 5 }> = truncate_str(name);
    let bytes = name.as_bytes();
    let header = [
        2,
        AD_FLAGS,
        FLAGS_GENERAL_LE_ONLY,
        0,
        AD_COMPLETE_LOCAL_NAME,
    ];
    buf[..5].copy_from_slice(&header);
    // bytes.len() <= 26, so the length byte fits and the copy is in bounds.
    buf[3] = u8::try_from(bytes.len() + 1).unwrap_or(u8::MAX);
    let end = 5 + bytes.len();
    if let Some(dst) = buf.get_mut(5..end) {
        dst.copy_from_slice(bytes);
    }
    (buf, end)
}

/// One advertising report, as received from the controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdvReport<'a> {
    /// Device address, little-endian as on air.
    pub addr: [u8; 6],
    /// HCI address type (0 public, 1 random, ...).
    pub addr_kind: u8,
    /// Received signal strength in dBm.
    pub rssi: i8,
    /// Raw advertising (or scan-response) data.
    pub data: &'a [u8],
}

/// Merges one advertising report into the table of devices heard so far.
///
/// Devices are keyed by address: a repeat sighting keeps the strongest RSSI
/// and fills in the name if it was not yet known (names often arrive only in
/// the scan response). When the table is full, a new device replaces the
/// weakest entry if it is stronger, otherwise it is dropped.
pub fn merge_report<const N: usize>(seen: &mut Vec<BleDevice, N>, report: &AdvReport<'_>) {
    let name = ad_local_name(report.data).map(lossy_utf8::<29>);
    let rssi = i32::from(report.rssi);
    if let Some(dev) = seen.iter_mut().find(|d| d.addr.as_slice() == report.addr) {
        dev.rssi = dev.rssi.max(rssi);
        if let Some(name) = name
            && !name.is_empty()
        {
            dev.name = name;
        }
        return;
    }
    let dev = BleDevice {
        addr: Vec::from_slice(&report.addr).unwrap_or_default(),
        addr_kind: u32::from(report.addr_kind),
        rssi,
        name: name.unwrap_or_default(),
    };
    if !seen.is_full() {
        let _ = seen.push(dev);
        return;
    }
    let weakest = seen
        .iter()
        .enumerate()
        .min_by_key(|(_, d)| d.rssi)
        .map(|(i, d)| (i, d.rssi));
    if let Some((i, weakest_rssi)) = weakest
        && rssi > weakest_rssi
        && let Some(slot) = seen.get_mut(i)
    {
        *slot = dev;
    }
}

/// Keeps the `N` strongest devices from `seen`, strongest first.
pub fn rank_devices<const N: usize, const M: usize>(seen: &Vec<BleDevice, M>) -> Vec<BleDevice, N> {
    let mut out: Vec<BleDevice, N> = Vec::new();
    for dev in seen {
        insert_ranked(&mut out, dev.clone(), |d| d.rssi);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ap(ssid: &str, rssi: i32) -> AccessPoint {
        AccessPoint {
            ssid: truncate_str(ssid),
            rssi,
            ..Default::default()
        }
    }

    #[test]
    fn truncate_respects_char_boundary() {
        let s: String<4> = truncate_str("ab\u{e9}\u{e9}");
        assert_eq!(s.as_str(), "ab\u{e9}");
        let s: String<3> = truncate_str("ab\u{e9}");
        assert_eq!(s.as_str(), "ab");
        let s: String<32> = truncate_str("short");
        assert_eq!(s.as_str(), "short");
    }

    #[test]
    fn lossy_keeps_valid_prefix() {
        let s: String<29> = lossy_utf8(b"abc\xffdef");
        assert_eq!(s.as_str(), "abc");
    }

    #[test]
    fn rank_aps_sorts_and_caps() {
        let input = (0..20).map(|i| ap("x", -90 + i));
        let out: Vec<AccessPoint, 12> = rank_aps(input, 64);
        assert_eq!(out.len(), 12);
        assert_eq!(out[0].rssi, -71);
        assert_eq!(out[11].rssi, -82);
        assert!(out.windows(2).all(|w| w[0].rssi >= w[1].rssi));
    }

    #[test]
    fn rank_aps_respects_limit() {
        let input = (0..20).map(|i| ap("x", i));
        let out: Vec<AccessPoint, 12> = rank_aps(input, 3);
        assert_eq!(out.len(), 3);
        assert_eq!(out[0].rssi, 2);
    }

    #[test]
    fn ties_are_stable() {
        let input = [ap("a", -50), ap("b", -50), ap("c", -40)].into_iter();
        let out: Vec<AccessPoint, 12> = rank_aps(input, 64);
        let names: [&str; 3] = [
            out[0].ssid.as_str(),
            out[1].ssid.as_str(),
            out[2].ssid.as_str(),
        ];
        assert_eq!(names, ["c", "a", "b"]);
    }

    #[test]
    fn ad_name_prefers_complete() {
        let data = [
            2, 0x01, 0x06, 3, 0x08, b'a', b'b', 4, 0x09, b'x', b'y', b'z',
        ];
        assert_eq!(ad_local_name(&data), Some(&b"xyz"[..]));
        let data = [2, 0x01, 0x06, 3, 0x08, b'a', b'b'];
        assert_eq!(ad_local_name(&data), Some(&b"ab"[..]));
    }

    #[test]
    fn ad_name_handles_malformed() {
        assert_eq!(ad_local_name(&[]), None);
        assert_eq!(ad_local_name(&[0, 0x09, b'a']), None);
        assert_eq!(ad_local_name(&[10, 0x09, b'a']), None);
        assert_eq!(ad_local_name(&[1, 0x09]), Some(&b""[..]));
    }

    #[test]
    fn adv_data_roundtrips_name() {
        let (buf, len) = encode_adv_data("badge");
        assert_eq!(&buf[..3], &[2, 0x01, 0x06]);
        assert_eq!(ad_local_name(&buf[..len]), Some(&b"badge"[..]));
        let long = "abcdefghijklmnopqrstuvwxyz0123";
        let (buf, len) = encode_adv_data(long);
        assert_eq!(len, LEGACY_ADV_LEN);
        assert_eq!(ad_local_name(&buf[..len]), Some(&long.as_bytes()[..26]));
    }

    fn report(addr: u8, rssi: i8, data: &[u8]) -> AdvReport<'_> {
        AdvReport {
            addr: [addr, 0, 0, 0, 0, 0],
            addr_kind: 1,
            rssi,
            data,
        }
    }

    #[test]
    fn merge_dedupes_keeps_strongest_and_name() {
        let mut seen: Vec<BleDevice, 4> = Vec::new();
        merge_report(&mut seen, &report(1, -80, &[]));
        merge_report(&mut seen, &report(1, -60, &[3, 0x09, b'h', b'i']));
        merge_report(&mut seen, &report(1, -70, &[]));
        assert_eq!(seen.len(), 1);
        assert_eq!(seen[0].rssi, -60);
        assert_eq!(seen[0].name.as_str(), "hi");
        assert_eq!(seen[0].addr_kind, 1);
        assert_eq!(seen[0].addr.as_slice(), &[1, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn merge_full_table_evicts_weakest() {
        let mut seen: Vec<BleDevice, 2> = Vec::new();
        merge_report(&mut seen, &report(1, -80, &[]));
        merge_report(&mut seen, &report(2, -70, &[]));
        merge_report(&mut seen, &report(3, -90, &[]));
        assert!(seen.iter().all(|d| d.addr[0] != 3));
        merge_report(&mut seen, &report(4, -50, &[]));
        assert!(seen.iter().any(|d| d.addr[0] == 4));
        assert!(seen.iter().all(|d| d.addr[0] != 1));
    }

    #[test]
    fn rank_devices_orders() {
        let mut seen: Vec<BleDevice, 8> = Vec::new();
        for (i, r) in [-70i8, -40, -90, -55].iter().enumerate() {
            merge_report(&mut seen, &report(i as u8, *r, &[]));
        }
        let out: Vec<BleDevice, 3> = rank_devices(&seen);
        let rssis: [i32; 3] = [out[0].rssi, out[1].rssi, out[2].rssi];
        assert_eq!(rssis, [-40, -55, -70]);
    }
}
