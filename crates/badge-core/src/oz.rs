//! Interoperability with stock Conagotchi badges (MicroPython firmware).
//!
//! The stock firmware sends short text payloads as one BLE manufacturer AD
//! (type `0xFF`, company id bytes `4F 5A` = "OZ", then a kind byte), over the
//! wired link UART, and over IR. Every payload carries a 3-byte truncated
//! HMAC-SHA256 tag keyed by the badge's `radio_key.txt`; keyed stock badges
//! drop unsigned packets. The interop reference is `../stock/vfs/*.py`.
//!
//! Everything here is pure: the firmware moves bytes, this module decides.
//!
//! - [`PeerId`]: the 6-hex-digit badge id used on air.
//! - [`oz_payload`]: finds the OZ manufacturer data in advertising data.
//! - [`chi_tag`] / [`chi_from_tag`]: the three-letter Chi tags on air.
//! - [`Profile`] / [`parse_profile`]: BLAT `OZN` profiles (display names).
//! - [`adv`]: the OZ manufacturer AD (`ble.py`), byte-exact.
//! - [`sign`]: the radio key and packet tags (`badge_radio.py`).
//! - [`link`]: link UART framing (`link.py`).
//! - [`lobby`]: lobby packets and the table of badges heard (`lobby_session.py`).
//! - [`trade`]: the Chi trade state machine (`trade_session.py`).
//!
//! A received packet travels: BLE advert → [`oz_payload`] (shell) →
//! [`adv::decode_body`] (kind byte → stock text payload), or link UART →
//! [`link::Decoder`]; then [`sign::verify`] strips the tag; then
//! [`lobby::parse`] or [`parse_profile`]. Sending reverses it:
//! [`lobby::pack`] → [`sign::sign`] → [`adv::encode`] and [`link::encode`].

pub mod adv;
pub mod link;
pub mod lobby;
pub mod sign;
pub mod trade;

use badge_proto::Chi;
use heapless::String;

/// Company id bytes of the OZ manufacturer AD, as they appear on air.
pub const OZ_CID: [u8; 2] = *b"OZ";
/// AD type of manufacturer-specific data.
pub const AD_MANUFACTURER: u8 = 0xFF;
/// Largest OZ payload (kind byte onward) carried through the core. A legacy
/// advert holds at most 31 - 2 (AD header) - 2 (CID) = 27; link frames may
/// be longer.
pub const MAX_PAYLOAD: usize = 64;

/// A badge id on air: the last 6 hex digits of the badge's `unique_id`,
/// uppercase ASCII (`"0AC8F6"`-style).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct PeerId([u8; 6]);

/// Why bytes are not a valid [`PeerId`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BadPeerId;

impl PeerId {
    /// Parses 6 hex digits (either case); stored uppercase.
    pub fn parse(s: &[u8]) -> Result<PeerId, BadPeerId> {
        let digits: [u8; 6] = s.try_into().map_err(|_| BadPeerId)?;
        let mut out = [0u8; 6];
        for (dst, &c) in out.iter_mut().zip(digits.iter()) {
            *dst = match c {
                b'0'..=b'9' | b'A'..=b'F' => c,
                b'a'..=b'f' => c.to_ascii_uppercase(),
                _ => return Err(BadPeerId),
            };
        }
        Ok(PeerId(out))
    }

    /// The id of a badge with this base MAC: the stock firmware's
    /// `unique_id` on ESP32-S3 is the base MAC, so the id is its last three
    /// bytes in hex.
    pub fn from_mac(mac: &[u8; 6]) -> PeerId {
        PeerId::from_u32(u32::from_be_bytes([0, mac[3], mac[4], mac[5]]))
    }

    /// The id whose hex digits spell the low 24 bits of `v`
    /// (`0x0AC8F6` → `"0AC8F6"`). Used by the PROBE mailbox.
    pub fn from_u32(v: u32) -> PeerId {
        const HEX: &[u8; 16] = b"0123456789ABCDEF";
        let mut out = [0u8; 6];
        for (i, dst) in out.iter_mut().enumerate() {
            let shift = 20 - 4 * i as u32;
            *dst = HEX[((v >> shift) & 0xF) as usize];
        }
        PeerId(out)
    }

    /// The id as a 24-bit number (inverse of [`PeerId::from_u32`]).
    pub fn to_u32(self) -> u32 {
        self.0.iter().fold(0u32, |acc, &c| {
            let nibble = match c {
                b'0'..=b'9' => c - b'0',
                b'A'..=b'F' => c - b'A' + 10,
                // Only `PeerId::default()` holds anything else.
                _ => 0,
            };
            (acc << 4) | u32::from(nibble)
        })
    }

    /// The 6 ASCII bytes.
    pub fn as_bytes(&self) -> &[u8; 6] {
        &self.0
    }

    /// The id as text.
    pub fn as_str(&self) -> &str {
        // Always ASCII hex by construction.
        core::str::from_utf8(&self.0).unwrap_or("??????")
    }
}

/// Returns the OZ payload (kind byte onward) if `ad` (raw advertising data,
/// a sequence of length-type-value AD structures) contains a manufacturer AD
/// with company id "OZ". Malformed AD structures end the walk.
pub fn oz_payload(ad: &[u8]) -> Option<&[u8]> {
    let mut rest = ad;
    // Each AD structure consumes at least one byte, so this is bounded by `ad.len()`.
    while let Some((&len, tail)) = rest.split_first() {
        let len = usize::from(len);
        if len == 0 {
            return None;
        }
        let item = tail.get(..len)?;
        rest = tail.get(len..)?;
        if let Some((&AD_MANUFACTURER, data)) = item.split_first()
            && let Some(payload) = data.strip_prefix(&OZ_CID)
        {
            return Some(payload);
        }
    }
    None
}

/// The six Chi tags the stock firmware knows (`characters/*.py` `tag`), in
/// `Chi` enum order.
const CHI_TAGS: [(Chi, &[u8; 3]); 6] = [
    (Chi::Vib, b"VIB"),
    (Chi::Hck, b"HCK"),
    (Chi::Pkt, b"PKT"),
    (Chi::Adm, b"ADM"),
    (Chi::Cry, b"CRY"),
    (Chi::Ovr, b"OVR"),
];

/// The three-letter tag of `chi` on air, `None` for UNSPECIFIED or an
/// unknown value.
pub fn chi_tag(chi: Chi) -> Option<&'static [u8; 3]> {
    CHI_TAGS.iter().find(|(c, _)| *c == chi).map(|(_, t)| *t)
}

/// The Chi with this tag (exact, case-sensitive like stock), `None` if the
/// tag is not one of the six.
pub fn chi_from_tag(tag: &[u8]) -> Option<Chi> {
    CHI_TAGS
        .iter()
        .find(|(_, t)| t.as_slice() == tag)
        .map(|(c, _)| *c)
}

/// The tag of `chi` as text, `"?"` if it has none (for display).
pub fn chi_name(chi: Chi) -> &'static str {
    chi_tag(chi)
        .and_then(|t| core::str::from_utf8(t).ok())
        .unwrap_or("?")
}

/// Longest BLAT display name (`blat_session.NAME_MAX`).
pub const NAME_MAX: usize = 8;

/// A BLAT profile packet: `OZN<sender6><tag3><level2><name≤8>`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Profile {
    /// Sending badge.
    pub sender: PeerId,
    /// Its active Chi's tag as sent (any three printable characters).
    pub tag: [u8; 3],
    /// That Chi's level, 0..=99.
    pub level: u8,
    /// Display name; empty means "use the badge id".
    pub name: String<NAME_MAX>,
}

/// Parses a (verified, tag-stripped) BLAT profile packet, mirroring
/// `blat_session.parse`: the whole packet must be printable ASCII, the
/// sender six hex digits (either case), the level two digits; the name is
/// the next up to 8 characters. Anything else (including `OZM` message
/// pieces) is `None`.
pub fn parse_profile(packet: &[u8]) -> Option<Profile> {
    if !packet.iter().all(|&b| (32..=126).contains(&b)) {
        return None;
    }
    let body = packet.strip_prefix(b"OZN")?;
    let sender = PeerId::parse(body.get(..6)?).ok()?;
    if body.len() < 11 {
        return None;
    }
    let tag: [u8; 3] = body.get(6..9)?.try_into().ok()?;
    let level = match body.get(9..11)? {
        [a @ b'0'..=b'9', b @ b'0'..=b'9'] => (a - b'0') * 10 + (b - b'0'),
        _ => return None,
    };
    let raw = body.get(11..).unwrap_or(&[]);
    let raw = raw.get(..NAME_MAX).unwrap_or(raw);
    let mut name = String::new();
    // Printable ASCII checked above, so every byte is one char.
    for &b in raw {
        name.push(char::from(b)).ok()?;
    }
    Some(Profile {
        sender,
        tag,
        level,
        name,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fr25_chi_tags_round_trip_and_reject_unknown() {
        for chi in [Chi::Vib, Chi::Hck, Chi::Pkt, Chi::Adm, Chi::Cry, Chi::Ovr] {
            let tag = chi_tag(chi).expect("tag");
            assert_eq!(chi_from_tag(tag), Some(chi));
        }
        assert_eq!(chi_tag(Chi::Unspecified), None);
        assert_eq!(chi_tag(Chi(42)), None);
        assert_eq!(chi_from_tag(b"CHI"), None);
        assert_eq!(chi_from_tag(b"vib"), None);
        assert_eq!(chi_from_tag(b"VI"), None);
        assert_eq!(chi_name(Chi::Cry), "CRY");
        assert_eq!(chi_name(Chi::Unspecified), "?");
    }

    #[test]
    fn fr26_parse_profile_mirrors_blat() {
        let p = parse_profile(b"OZN0ac8f6ADM07Ruffles!!x").expect("profile");
        assert_eq!(p.sender.as_str(), "0AC8F6");
        assert_eq!(&p.tag, b"ADM");
        assert_eq!(p.level, 7);
        assert_eq!(p.name.as_str(), "Ruffles!");
        // Empty name (= badge id).
        assert_eq!(parse_profile(b"OZN0AC8F6VIB00").expect("p").name.as_str(), "");
        // Level not digits, too short, bad sender, non-printable, message piece.
        assert_eq!(parse_profile(b"OZN0AC8F6VIBx0Bob"), None);
        assert_eq!(parse_profile(b"OZN0AC8F6VIB0"), None);
        assert_eq!(parse_profile(b"OZN0AC8FGVIB00Bob"), None);
        assert_eq!(parse_profile(b"OZN0AC8F6VIB00B\x01b"), None);
        assert_eq!(parse_profile(&[b'O', b'Z', b'N', 0x80]), None);
        assert_eq!(parse_profile(b"OZN0AC8F6VIB00Bo\nb"), None);
        assert_eq!(parse_profile(b"OZM0AC8F6012hello"), None);
    }

    #[test]
    fn peer_id_parses_uppercases_and_round_trips() {
        let id = PeerId::parse(b"0ac8f6").expect("hex");
        assert_eq!(id.as_str(), "0AC8F6");
        assert_eq!(id.to_u32(), 0x0A_C8F6);
        assert_eq!(PeerId::from_u32(0x0A_C8F6), id);
        assert!(PeerId::parse(b"0AC8F").is_err());
        assert!(PeerId::parse(b"0AC8FG").is_err());
        assert_eq!(PeerId::from_mac(&[0xe8, 0x3d, 0xc1, 0xf6, 0x0a, 0xc8]).as_str(), "F60AC8");
    }

    #[test]
    fn oz_payload_finds_manufacturer_ad() {
        // Flags AD, then OZ manufacturer AD with kind 0 and body "hi".
        let ad = [2, 0x01, 0x06, 6, 0xFF, b'O', b'Z', 0, b'h', b'i'];
        assert_eq!(oz_payload(&ad), Some(&[0, b'h', b'i'][..]));
        // Other company id.
        assert_eq!(oz_payload(&[4, 0xFF, 0x4C, 0x00, 1]), None);
        // Truncated structure.
        assert_eq!(oz_payload(&[9, 0xFF, b'O', b'Z']), None);
        assert_eq!(oz_payload(&[]), None);
    }
}
