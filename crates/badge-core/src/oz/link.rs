//! Link UART framing (stock `link.py`): `"OZ"` + length + payload + sum8, at
//! 115200 baud on GPIO39 (TX) / GPIO40 (RX).
//!
//! The checksum is the 8-bit sum of every byte before it (magic and length
//! included). Payloads are at most [`MAX_LINK_PAYLOAD`] (48) bytes, the
//! stock `_IR_MAX_PAYLOAD` shared by the IR and wired links.
//!
//! [`encode`] builds a frame; [`Decoder`] consumes received bytes one at a
//! time and resynchronises after garbage the way stock `_scan_frames` does:
//! a length over 48 or a bad checksum skips one byte past the frame start
//! and searches for the next `"OZ"` in what was already received.

use heapless::Vec;

use super::MAX_PAYLOAD;

/// Largest encoded frame buffer: 2 magic + 1 length + payload + 1 checksum.
pub const MAX_FRAME: usize = MAX_PAYLOAD + 4;
/// Largest payload stock accepts on the link (`_IR_MAX_PAYLOAD`).
pub const MAX_LINK_PAYLOAD: usize = 48;
/// Frame start marker.
pub const MAGIC: [u8; 2] = *b"OZ";

/// Why a payload cannot be framed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TooLong;

/// 8-bit sum of `data` (stock `_checksum`).
pub fn checksum(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |acc, &b| acc.wrapping_add(b))
}

/// Encodes `payload` as one link frame (stock `_make_frame`).
pub fn encode(payload: &[u8]) -> Result<Vec<u8, MAX_FRAME>, TooLong> {
    if payload.len() > MAX_LINK_PAYLOAD {
        return Err(TooLong);
    }
    let len = u8::try_from(payload.len()).map_err(|_| TooLong)?;
    let mut frame = Vec::new();
    frame.extend_from_slice(&MAGIC).map_err(|_| TooLong)?;
    frame.push(len).map_err(|_| TooLong)?;
    frame.extend_from_slice(payload).map_err(|_| TooLong)?;
    let sum = checksum(&frame);
    frame.push(sum).map_err(|_| TooLong)?;
    Ok(frame)
}

/// Streaming frame decoder. Feed bytes with [`Decoder::push`].
///
/// Invariant between calls: the buffer is a proper prefix of a possible
/// frame (starts with `'O'`, then `'Z'`, then a length ≤ 48, and is shorter
/// than the frame it announces), so it never holds more than 51 bytes.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Decoder {
    buf: Vec<u8, MAX_FRAME>,
}

/// What the head of the buffer is.
enum Head {
    /// Possible frame start, more bytes needed.
    Partial,
    /// Cannot start a frame here: drop one byte.
    Junk,
    /// A whole frame of this many bytes.
    Frame(usize),
}

impl Decoder {
    /// Consumes one received byte; returns a payload when a frame with a
    /// valid checksum completes.
    pub fn push(&mut self, byte: u8) -> Option<Vec<u8, MAX_PAYLOAD>> {
        if self.buf.push(byte).is_err() {
            // Unreachable by the invariant; recover by starting over.
            self.buf.clear();
            return None;
        }
        // Each pass either returns or drops one byte: bounded by MAX_FRAME.
        for _ in 0..=MAX_FRAME {
            match self.head() {
                Head::Partial => return None,
                Head::Junk => self.drop_front(1),
                Head::Frame(n) => {
                    let frame = self.buf.get(..n).unwrap_or(&[]);
                    let (body, sum) = frame.split_at(n.saturating_sub(1));
                    if sum.first() == Some(&checksum(body)) {
                        let mut out = Vec::new();
                        out.extend_from_slice(body.get(3..).unwrap_or(&[])).ok()?;
                        self.drop_front(n);
                        return Some(out);
                    }
                    self.drop_front(1);
                }
            }
        }
        None
    }

    fn head(&self) -> Head {
        match self.buf.as_slice() {
            [] => Head::Partial,
            [a, ..] if *a != MAGIC[0] => Head::Junk,
            [_] => Head::Partial,
            [_, b, ..] if *b != MAGIC[1] => Head::Junk,
            [_, _] => Head::Partial,
            [_, _, len, ..] if usize::from(*len) > MAX_LINK_PAYLOAD => Head::Junk,
            [_, _, len, ..] => {
                let n = usize::from(*len) + 4;
                if self.buf.len() >= n {
                    Head::Frame(n)
                } else {
                    Head::Partial
                }
            }
        }
    }

    fn drop_front(&mut self, n: usize) {
        let rest: Vec<u8, MAX_FRAME> =
            Vec::from_slice(self.buf.get(n..).unwrap_or(&[])).unwrap_or_default();
        self.buf = rest;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    extern crate std;
    use std::vec::Vec as StdVec;

    fn feed(d: &mut Decoder, bytes: &[u8]) -> StdVec<StdVec<u8>> {
        bytes
            .iter()
            .filter_map(|&b| d.push(b))
            .map(|p| p.to_vec())
            .collect()
    }

    #[test]
    fn fr25_frame_golden_bytes_and_checksum() {
        let f = encode(b"hi").expect("fits");
        // 'O'+'Z'+2+'h'+'i' = 0x4F+0x5A+0x02+0x68+0x69 = 0x17C -> 0x7C.
        assert_eq!(&f[..], &[0x4F, 0x5A, 0x02, b'h', b'i', 0x7C]);
        assert_eq!(&encode(b"").expect("fits")[..], &[0x4F, 0x5A, 0, 0xA9]);
        assert!(encode(&[0; 48]).is_ok());
        assert_eq!(encode(&[0; 49]), Err(TooLong));
        let mut d = Decoder::default();
        assert_eq!(feed(&mut d, &f), [b"hi".to_vec()]);
    }

    #[test]
    fn fr25_decoder_resyncs_after_garbage_and_bad_frames() {
        let good = encode(b"OZGT:0AC8F6:------:~VIB").expect("fits");
        let mut bad = encode(b"nope").expect("fits");
        if let Some(sum) = bad.last_mut() {
            *sum ^= 0xFF;
        }
        let mut stream = StdVec::new();
        stream.extend_from_slice(b"\x00\xffOOZ"); // junk, a lone O, then "OZ"+len 'O'(>48)
        stream.extend_from_slice(&bad);
        stream.extend_from_slice(b"O"); // split magic
        stream.extend_from_slice(&good);
        stream.extend_from_slice(&[b'O', b'Z', 200]); // bogus length
        stream.extend_from_slice(&good);
        let mut d = Decoder::default();
        let got = feed(&mut d, &stream);
        assert_eq!(got.len(), 2, "{got:?}");
        assert!(got.iter().all(|p| p == b"OZGT:0AC8F6:------:~VIB"));
    }

    #[test]
    fn fr25_decoder_finds_frame_inside_a_bad_one() {
        // A bad outer frame whose body holds a whole valid frame: stock
        // `_scan_frames` resyncs one byte past the outer start and finds it.
        let inner = encode(b"ok").expect("fits");
        let mut outer = StdVec::from([b'O', b'Z', u8::try_from(inner.len()).expect("small")]);
        outer.extend_from_slice(&inner);
        outer.push(0); // wrong checksum
        let mut d = Decoder::default();
        assert_eq!(feed(&mut d, &outer), [b"ok".to_vec()]);
        // And the decoder is clean afterwards.
        assert_eq!(feed(&mut d, &encode(b"x").expect("fits")), [b"x".to_vec()]);
    }

    #[test]
    fn fr25_decoder_survives_any_byte_stream() {
        let mut d = Decoder::default();
        let mut x: u32 = 1;
        for _ in 0..20_000 {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            let b = [b'O', b'Z', (x >> 8) as u8, (x >> 16) as u8 & 0x3F][(x & 3) as usize];
            if let Some(p) = d.push(b) {
                assert!(p.len() <= MAX_LINK_PAYLOAD);
            }
            assert!(d.buf.len() < MAX_LINK_PAYLOAD + 4);
        }
    }
}
