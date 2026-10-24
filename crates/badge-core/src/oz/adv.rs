//! The OZ manufacturer AD of stock `ble.py`, byte for byte.
//!
//! A stock badge's advertising data is exactly one AD structure (no Flags
//! AD, no name), using the whole 31-byte legacy budget at most:
//!
//! `[len] [0xFF] ['O' 'Z'] [kind] [body…]`, `len` = bytes after itself.
//!
//! The body compacts a stock text payload (signature bytes included):
//! - kind 1 (char): `OZC1:<chi>:<rest>` → `[1][len(chi)][chi][rest]`;
//! - kind 2 (stamp): `OZS1:<rest>` → `[2][rest]`;
//! - kind 0 (raw): anything else, verbatim (this carries lobby packets).
//!
//! [`encode`] builds the complete advertising data for `Effect::Beacon`;
//! [`decode_body`] reverses it from the kind byte on (what the shell hands
//! the core in `Input::RadioPacket`), and [`decode`] from raw advertising
//! data. The stock adverts are non-connectable, 100 ms interval.

use heapless::Vec;

use super::{MAX_PAYLOAD, OZ_CID, oz_payload};
use crate::io::MAX_ADV_DATA;

/// Kind byte of a raw body.
pub const KIND_RAW: u8 = 0;
/// Kind byte of a compacted Chi offer (`OZC1:`).
pub const KIND_CHAR: u8 = 1;
/// Kind byte of a vendor stamp (`OZS1:`).
pub const KIND_STAMP: u8 = 2;
/// Text prefix of a Chi offer.
pub const PREFIX_CHAR: &[u8] = b"OZC1:";
/// Text prefix of a vendor stamp.
pub const PREFIX_STAMP: &[u8] = b"OZS1:";
/// Most manufacturer bytes (company id + kind + body): 31 - 2 (AD header).
pub const MAX_MFG: usize = MAX_ADV_DATA - 2;

/// The payload does not fit one legacy advert (stock `encode_adv` → None).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TooLong;

/// Complete advertising data carrying `payload` (stock `encode_adv`).
pub fn encode(payload: &[u8]) -> Result<Vec<u8, MAX_ADV_DATA>, TooLong> {
    let mut mfg: Vec<u8, MAX_MFG> = Vec::new();
    mfg.extend_from_slice(&OZ_CID).map_err(|_| TooLong)?;
    if let Some(rest) = payload.strip_prefix(PREFIX_CHAR) {
        // `partition(b":")`: no colon = the whole rest is the Chi id.
        let (chi, tail) = match rest.iter().position(|&b| b == b':') {
            Some(i) => (rest.get(..i).unwrap_or(&[]), rest.get(i + 1..).unwrap_or(&[])),
            None => (rest, &[][..]),
        };
        let clen = u8::try_from(chi.len()).map_err(|_| TooLong)?;
        mfg.extend_from_slice(&[KIND_CHAR, clen]).map_err(|_| TooLong)?;
        mfg.extend_from_slice(chi).map_err(|_| TooLong)?;
        mfg.extend_from_slice(tail).map_err(|_| TooLong)?;
    } else if let Some(rest) = payload.strip_prefix(PREFIX_STAMP) {
        mfg.push(KIND_STAMP).map_err(|_| TooLong)?;
        mfg.extend_from_slice(rest).map_err(|_| TooLong)?;
    } else {
        mfg.push(KIND_RAW).map_err(|_| TooLong)?;
        mfg.extend_from_slice(payload).map_err(|_| TooLong)?;
    }
    let len = u8::try_from(mfg.len() + 1).map_err(|_| TooLong)?;
    let mut adv = Vec::new();
    adv.extend_from_slice(&[len, super::AD_MANUFACTURER])
        .map_err(|_| TooLong)?;
    adv.extend_from_slice(&mfg).map_err(|_| TooLong)?;
    Ok(adv)
}

/// The stock text payload of an OZ body, kind byte onward (stock
/// `_decode_body`). `None` for an empty body, an unknown kind, or a char
/// body whose length byte overruns it.
pub fn decode_body(data: &[u8]) -> Option<Vec<u8, MAX_PAYLOAD>> {
    let (&kind, p) = data.split_first()?;
    let mut out = Vec::new();
    match kind {
        KIND_CHAR => {
            let (&cl, rest) = p.split_first()?;
            let cl = usize::from(cl);
            let chi = rest.get(..cl)?;
            out.extend_from_slice(PREFIX_CHAR).ok()?;
            out.extend_from_slice(chi).ok()?;
            out.push(b':').ok()?;
            out.extend_from_slice(rest.get(cl..)?).ok()?;
        }
        KIND_STAMP => {
            out.extend_from_slice(PREFIX_STAMP).ok()?;
            out.extend_from_slice(p).ok()?;
        }
        KIND_RAW => out.extend_from_slice(p).ok()?,
        _ => return None,
    }
    Some(out)
}

/// The stock text payload in raw advertising data (stock `decode_adv`).
pub fn decode(ad: &[u8]) -> Option<Vec<u8, MAX_PAYLOAD>> {
    decode_body(oz_payload(ad)?)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::oz::sign::{RadioKey, sign};

    #[test]
    fn fr25_signed_lobby_advert_is_exactly_31_bytes() {
        // "OZGT:0AC8F6:1A2B3C:VIB1" signed with the fake key [0x11; 32]
        // (tag 0xbb 0xe0 0x0b from Python hmac).
        let key = RadioKey([0x11; 32]);
        let signed: Vec<u8, 32> = sign(Some(&key), b"OZGT:0AC8F6:1A2B3C:VIB1").expect("fits");
        let adv = encode(&signed).expect("fits");
        let mut want = std::vec![30u8, 0xFF, b'O', b'Z', 0];
        want.extend_from_slice(b"OZGT:0AC8F6:1A2B3C:VIB1");
        want.extend_from_slice(&[0xbb, 0xe0, 0x0b]);
        assert_eq!(want.len(), 31);
        assert_eq!(&adv[..], &want[..]);
        // Round trip through the shell's oz_payload and decode_body.
        assert_eq!(decode(&adv).as_deref(), Some(&signed[..]));
        let body = oz_payload(&adv).expect("ours");
        assert_eq!(body[0], KIND_RAW);
    }

    extern crate std;

    #[test]
    fn fr25_char_and_stamp_bodies_match_ble_py() {
        let adv = encode(b"OZC1:packachi:aca704").expect("fits");
        let mut want = std::vec![0u8, 0xFF, b'O', b'Z', 1, 8];
        want.extend_from_slice(b"packachi");
        want.extend_from_slice(b"aca704");
        want[0] = u8::try_from(want.len() - 1).expect("small");
        assert_eq!(&adv[..], &want[..]);
        assert_eq!(decode(&adv).as_deref(), Some(&b"OZC1:packachi:aca704"[..]));
        // No second colon: stock `partition` puts it all in the Chi id.
        let adv = encode(b"OZC1:vib").expect("fits");
        assert_eq!(&adv[4..], &[1, 3, b'v', b'i', b'b']);
        assert_eq!(decode(&adv).as_deref(), Some(&b"OZC1:vib:"[..]));
        let adv = encode(b"OZS1:VEND:0AC8F6").expect("fits");
        assert_eq!(&adv[4..5], &[2]);
        assert_eq!(&adv[5..], b"VEND:0AC8F6");
        assert_eq!(decode(&adv).as_deref(), Some(&b"OZS1:VEND:0AC8F6"[..]));
    }

    #[test]
    fn fr25_encode_rejects_oversize_and_decode_rejects_junk() {
        assert_eq!(encode(&[b'x'; 26]).map(|a| a.len()), Ok(31));
        assert_eq!(encode(&[b'x'; 27]), Err(TooLong));
        assert_eq!(decode_body(&[]), None);
        assert_eq!(decode_body(&[9, 1, 2]), None);
        assert_eq!(decode_body(&[KIND_CHAR]), None);
        assert_eq!(decode_body(&[KIND_CHAR, 5, b'a']), None);
        assert_eq!(decode_body(&[KIND_RAW]).as_deref(), Some(&[][..]));
    }
}
