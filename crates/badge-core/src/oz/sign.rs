//! Packet signing: 3-byte truncated HMAC-SHA256 tags keyed by the badge's
//! radio key, exactly as stock `badge_radio.py` does it.
//!
//! What is tagged is the stock *text payload*: for BLE the bytes
//! [`super::adv::decode_body`] reconstructs (after the kind byte, so neither
//! the AD header, the "OZ" company id nor the kind byte are covered), for
//! the link UART the frame payload. The tag is appended:
//! `payload ‖ HMAC-SHA256(key, payload)[..3]`.
//!
//! Stock rules mirrored here:
//! - with a key, every packet sent is tagged ([`sign`]) and every packet
//!   received must carry a correct tag, which is stripped ([`verify`]);
//!   a packet of [`TAG_LEN`] bytes or less is rejected;
//! - without a key, packets go out untagged and are accepted unchanged
//!   (a tagged packet from a keyed badge then keeps its 3 tag bytes, as on
//!   an unkeyed stock badge).
//!
//! The key is provisioned per badge (stock `radio_key.txt`, imported into
//! our `conag` partition on first boot). It is never committed, never logged
//! and never printed: [`RadioKey`]'s `Debug` shows only that a key exists.
//! Stock accepts any hex key of at least 16 bytes; HMAC zero-pads keys up to
//! its 64-byte block, so a shorter key zero-padded to [`KEY_LEN`] gives
//! identical tags ([`RadioKey::from_bytes`] does that padding).

use heapless::Vec;
use hmac::{Hmac, Mac};
use sha2::Sha256;

/// Length of the radio key in bytes (shorter stock keys are zero-padded).
pub const KEY_LEN: usize = 32;
/// Length of the truncated tag appended to every signed payload.
pub const TAG_LEN: usize = 3;
/// Shortest key stock `load_key` accepts.
pub const KEY_MIN: usize = 16;

/// The radio key. Absent key = packets are sent unsigned.
#[derive(Clone, PartialEq, Eq)]
pub struct RadioKey(pub [u8; KEY_LEN]);

impl core::fmt::Debug for RadioKey {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("RadioKey(<redacted>)")
    }
}

/// Why bytes are not a usable radio key.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BadKey;

impl RadioKey {
    /// A key of [`KEY_MIN`]..=[`KEY_LEN`] bytes, zero-padded (HMAC-equivalent
    /// to the unpadded key).
    pub fn from_bytes(key: &[u8]) -> Result<RadioKey, BadKey> {
        if !(KEY_MIN..=KEY_LEN).contains(&key.len()) {
            return Err(BadKey);
        }
        let mut out = [0u8; KEY_LEN];
        out.get_mut(..key.len()).ok_or(BadKey)?.copy_from_slice(key);
        Ok(RadioKey(out))
    }

    /// Parses the text of a stock `radio_key.txt`: hex digits (either case)
    /// with surrounding whitespace, like `load_key`. Fewer than 16 or more
    /// than 32 bytes, an odd digit count or a non-hex character is an error.
    pub fn from_hex(text: &[u8]) -> Result<RadioKey, BadKey> {
        let text = text.trim_ascii();
        let (pairs, odd) = text.as_chunks::<2>();
        if !odd.is_empty() {
            return Err(BadKey);
        }
        let mut bytes: Vec<u8, KEY_LEN> = Vec::new();
        for [h, l] in pairs {
            bytes
                .push((nibble(*h)? << 4) | nibble(*l)?)
                .map_err(|_| BadKey)?;
        }
        RadioKey::from_bytes(&bytes)
    }
}

fn nibble(c: u8) -> Result<u8, BadKey> {
    match c {
        b'0'..=b'9' => Ok(c - b'0'),
        b'a'..=b'f' => Ok(c - b'a' + 10),
        b'A'..=b'F' => Ok(c - b'A' + 10),
        _ => Err(BadKey),
    }
}

/// Why a packet was not signed or not accepted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SignError {
    /// Received packet of [`TAG_LEN`] bytes or less (no room for a body).
    TooShort,
    /// The tag does not match the body under our key.
    BadTag,
    /// The signed packet does not fit the output buffer.
    TooLong,
}

/// Full HMAC-SHA256 of `data` under `key` (any length, per RFC 2104).
pub fn hmac_sha256(key: &[u8], data: &[u8]) -> [u8; 32] {
    // HMAC accepts keys of any length; `new_from_slice` cannot fail for it.
    match <Hmac<Sha256> as Mac>::new_from_slice(key) {
        Ok(mut mac) => {
            mac.update(data);
            mac.finalize().into_bytes().into()
        }
        Err(_) => [0; 32],
    }
}

/// The first [`TAG_LEN`] bytes of HMAC-SHA256(key, data) (stock `tag`).
pub fn tag(key: &RadioKey, data: &[u8]) -> [u8; TAG_LEN] {
    let full = hmac_sha256(&key.0, data);
    [full[0], full[1], full[2]]
}

/// `msg` with its tag appended when there is a key, unchanged otherwise
/// (stock `BadgeRadio.send`).
pub fn sign<const N: usize>(key: Option<&RadioKey>, msg: &[u8]) -> Result<Vec<u8, N>, SignError> {
    let mut out = Vec::new();
    out.extend_from_slice(msg).map_err(|_| SignError::TooLong)?;
    if let Some(key) = key {
        out.extend_from_slice(&tag(key, msg))
            .map_err(|_| SignError::TooLong)?;
    }
    Ok(out)
}

/// The body of a received packet (stock `verified` when keyed): with a key,
/// `packet` minus its tag if the tag is right, else an error; without a
/// key, `packet` unchanged.
pub fn verify<'a>(key: Option<&RadioKey>, packet: &'a [u8]) -> Result<&'a [u8], SignError> {
    let Some(key) = key else {
        return Ok(packet);
    };
    if packet.len() <= TAG_LEN {
        return Err(SignError::TooShort);
    }
    let (body, got) = packet.split_at(packet.len() - TAG_LEN);
    if tag(key, body) == got {
        Ok(body)
    } else {
        Err(SignError::BadTag)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    extern crate std;
    use std::format;

    /// A fake key: never a real one.
    const FAKE: RadioKey = RadioKey([0x11; KEY_LEN]);

    fn hex(s: &str) -> std::vec::Vec<u8> {
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).expect("hex"))
            .collect()
    }

    #[test]
    fn debug_never_shows_key_bytes() {
        let s = format!("{:?}", RadioKey([0xAB; KEY_LEN]));
        assert!(!s.contains("AB") && !s.contains("171"), "{s}");
    }

    #[test]
    fn fr27_hmac_matches_rfc4231() {
        let cases: [(std::vec::Vec<u8>, std::vec::Vec<u8>, &str); 5] = [
            (
                std::vec![0x0b; 20],
                b"Hi There".to_vec(),
                "b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7",
            ),
            (
                b"Jefe".to_vec(),
                b"what do ya want for nothing?".to_vec(),
                "5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843",
            ),
            (
                std::vec![0xaa; 20],
                std::vec![0xdd; 50],
                "773ea91e36800e46854db8ebd09181a72959098b3ef8c122d9635514ced565fe",
            ),
            (
                (1..=25).collect(),
                std::vec![0xcd; 50],
                "82558a389a443c0ea4cc819899f2083a85f0faa3e578f8077a2e3ff46729665b",
            ),
            (
                std::vec![0xaa; 131],
                b"Test Using Larger Than Block-Size Key - Hash Key First".to_vec(),
                "60e431591ee0b67f0d8a26aacbf5b77f8e0bc6213728c5140546040f0ee37f54",
            ),
        ];
        for (key, data, want) in &cases {
            assert_eq!(hmac_sha256(key, data).to_vec(), hex(want));
        }
        // A short key zero-padded into a RadioKey gives the same tag.
        let k = RadioKey::from_bytes(&(1..=25).collect::<std::vec::Vec<u8>>()).expect("key");
        assert_eq!(tag(&k, &[0xcd; 50]).to_vec(), hex("82558a"));
    }

    #[test]
    fn fr27_sign_appends_truncated_tag_over_text_payload() {
        // Vectors from Python hmac with the fake key.
        let cases: [(&[u8], &str); 3] = [
            (b"OZGT:0AC8F6:------:~VIB", "65b913"),
            (b"OZGT:0AC8F6:1A2B3C:VIB1", "bbe00b"),
            (b"OZGT:1A2B3C:0AC8F6:HCK0", "152542"),
        ];
        for (msg, want) in cases {
            let signed: Vec<u8, 32> = sign(Some(&FAKE), msg).expect("fits");
            assert_eq!(&signed[..msg.len()], msg);
            assert_eq!(signed[msg.len()..].to_vec(), hex(want));
            assert_eq!(verify(Some(&FAKE), &signed), Ok(msg));
        }
    }

    #[test]
    fn fr27_verify_rejects_short_and_bad_tags() {
        assert_eq!(verify(Some(&FAKE), b"abc"), Err(SignError::TooShort));
        assert_eq!(verify(Some(&FAKE), b""), Err(SignError::TooShort));
        let mut signed: Vec<u8, 32> = sign(Some(&FAKE), b"OZGT:0AC8F6:------:~VIB").expect("fits");
        if let Some(last) = signed.last_mut() {
            *last ^= 1;
        }
        assert_eq!(verify(Some(&FAKE), &signed), Err(SignError::BadTag));
        // Unsigned packet to a keyed badge: dropped.
        assert_eq!(
            verify(Some(&FAKE), b"OZGT:0AC8F6:------:~VIB"),
            Err(SignError::BadTag)
        );
        // Another key's tag: dropped.
        let other = RadioKey([0x22; KEY_LEN]);
        let signed: Vec<u8, 32> = sign(Some(&other), b"OZGT:0AC8F6:------:~VIB").expect("fits");
        assert_eq!(verify(Some(&FAKE), &signed), Err(SignError::BadTag));
    }

    #[test]
    fn fr27_no_key_sends_and_accepts_unsigned() {
        let out: Vec<u8, 32> = sign(None, b"OZGT:0AC8F6:------:~VIB").expect("fits");
        assert_eq!(&out[..], b"OZGT:0AC8F6:------:~VIB");
        assert_eq!(verify(None, b"x"), Ok(&b"x"[..]));
        // A tagged packet is accepted as is, tag included.
        let signed: Vec<u8, 32> = sign(Some(&FAKE), b"OZGT:0AC8F6:------:~VIB").expect("fits");
        assert_eq!(verify(None, &signed), Ok(&signed[..]));
        let too_small: Result<Vec<u8, 4>, _> = sign(Some(&FAKE), b"abcd");
        assert_eq!(too_small, Err(SignError::TooLong));
    }

    #[test]
    fn fr27_key_parsing_mirrors_load_key() {
        let k = RadioKey::from_hex(b"  1111111111111111111111111111111111111111111111111111111111111111\n")
            .expect("hex");
        assert_eq!(k, FAKE);
        let short = RadioKey::from_hex(b"0B0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b").expect("16 bytes");
        assert_eq!(&short.0[..16], &[0x0b; 16]);
        assert_eq!(&short.0[16..], &[0; 16]);
        assert_eq!(RadioKey::from_hex(b"0b0b0b"), Err(BadKey)); // too short
        assert_eq!(RadioKey::from_hex(b"0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0"), Err(BadKey)); // odd
        assert_eq!(RadioKey::from_hex(b"zz0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b"), Err(BadKey));
        assert_eq!(RadioKey::from_bytes(&[0; 33]), Err(BadKey));
    }
}
