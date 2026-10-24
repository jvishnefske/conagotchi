//! COBS framing for the RTT byte streams.
//!
//! Each protobuf message travels as one frame: the message bytes are
//! COBS-encoded (so they contain no `0x00`) and followed by a single `0x00`
//! delimiter. A reader can join the stream at any point and resynchronise at
//! the next delimiter.
//!
//! [`encode`] writes one complete frame (including the delimiter).
//! [`Decoder`] is a bounded streaming decoder fed one byte at a time.

/// Worst-case encoded size of a `len`-byte payload, including the delimiter.
pub const fn max_encoded_len(len: usize) -> usize {
    len + len / 254 + 2
}

/// Framing failures.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// The output buffer cannot hold the encoded frame.
    BufferTooSmall,
    /// A frame exceeded the decoder capacity and was discarded.
    Overflow,
    /// The bytes between two delimiters are not valid COBS.
    Corrupt,
}

/// COBS-encodes `payload` into `out` and appends the `0x00` delimiter.
///
/// Returns the total number of bytes written.
pub fn encode(payload: &[u8], out: &mut [u8]) -> Result<usize, FrameError> {
    if out.len() < max_encoded_len(payload.len()) {
        return Err(FrameError::BufferTooSmall);
    }
    let mut code_idx = 0usize;
    let mut write = 1usize;
    let mut code: u8 = 1;
    for &byte in payload {
        if byte == 0 {
            out[code_idx] = code;
            code_idx = write;
            write += 1;
            code = 1;
        } else {
            out[write] = byte;
            write += 1;
            code += 1;
            if code == 0xff {
                out[code_idx] = code;
                code_idx = write;
                write += 1;
                code = 1;
            }
        }
    }
    out[code_idx] = code;
    out[write] = 0;
    Ok(write + 1)
}

/// Decodes a COBS block (without delimiter) in place, returning the payload
/// length. The payload occupies `buf[..len]` afterwards.
fn decode_in_place(buf: &mut [u8]) -> Result<usize, FrameError> {
    let mut read = 0usize;
    let mut write = 0usize;
    while read < buf.len() {
        let code = buf[read];
        if code == 0 {
            return Err(FrameError::Corrupt);
        }
        read += 1;
        let run = usize::from(code) - 1;
        let end = read.checked_add(run).ok_or(FrameError::Corrupt)?;
        if end > buf.len() {
            return Err(FrameError::Corrupt);
        }
        buf.copy_within(read..end, write);
        write += run;
        read = end;
        if code != 0xff && read < buf.len() {
            buf[write] = 0;
            write += 1;
        }
    }
    Ok(write)
}

/// Streaming frame decoder with a fixed `N`-byte buffer.
///
/// Feed bytes with [`Decoder::push`]. When a delimiter completes a frame, the
/// decoded payload is returned and the decoder resets. Frames larger than `N`
/// encoded bytes are dropped and reported once as [`FrameError::Overflow`].
#[derive(Debug, Clone)]
pub struct Decoder<const N: usize> {
    buf: [u8; N],
    len: usize,
    overflowed: bool,
}

impl<const N: usize> Default for Decoder<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Decoder<N> {
    /// Creates an empty decoder.
    pub const fn new() -> Self {
        Self { buf: [0; N], len: 0, overflowed: false }
    }

    /// Feeds one byte. Returns `Some` when a delimiter ends a frame.
    ///
    /// Empty frames (two delimiters in a row) are skipped silently.
    pub fn push(&mut self, byte: u8) -> Option<Result<&[u8], FrameError>> {
        if byte != 0 {
            if self.len < N {
                self.buf[self.len] = byte;
                self.len += 1;
            } else {
                self.overflowed = true;
            }
            return None;
        }
        let len = core::mem::take(&mut self.len);
        if core::mem::take(&mut self.overflowed) {
            return Some(Err(FrameError::Overflow));
        }
        if len == 0 {
            return None;
        }
        Some(decode_in_place(&mut self.buf[..len]).map(|n| &self.buf[..n]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn round_trip(payload: &[u8]) {
        let mut wire = [0u8; 1100];
        let n = encode(payload, &mut wire).expect("encode");
        assert!(n <= max_encoded_len(payload.len()));
        assert_eq!(wire[n - 1], 0);
        assert!(!wire[..n - 1].contains(&0), "COBS body must not contain 0x00");
        let mut dec = Decoder::<1100>::new();
        let mut out = None;
        for &b in &wire[..n] {
            if let Some(r) = dec.push(b) {
                let frame = r.expect("decode");
                let mut copy = [0u8; 1100];
                copy[..frame.len()].copy_from_slice(frame);
                out = Some((copy, frame.len()));
            }
        }
        let (copy, len) = out.expect("frame completed");
        assert_eq!(&copy[..len], payload);
    }

    #[test]
    fn round_trips_edge_cases() {
        round_trip(&[]);
        round_trip(&[0]);
        round_trip(&[0, 0, 0]);
        round_trip(&[1, 2, 3]);
        round_trip(&[1, 0, 2, 0]);
        let ramp: [u8; 600] = core::array::from_fn(|i| (i % 256) as u8);
        round_trip(&ramp);
        let nonzero: [u8; 254] = [7; 254];
        round_trip(&nonzero);
        let nonzero: [u8; 255] = [7; 255];
        round_trip(&nonzero);
        let nonzero: [u8; 1000] = [0x5a; 1000];
        round_trip(&nonzero);
    }

    #[test]
    fn encode_rejects_small_buffer() {
        let mut out = [0u8; 3];
        assert_eq!(encode(&[1, 2, 3], &mut out), Err(FrameError::BufferTooSmall));
    }

    #[test]
    fn decoder_resyncs_after_overflow() {
        let mut dec = Decoder::<4>::new();
        for b in [1u8, 2, 3, 4, 5, 6] {
            assert!(dec.push(b).is_none());
        }
        assert_eq!(dec.push(0), Some(Err(FrameError::Overflow)));
        // Next frame decodes normally.
        let mut wire = [0u8; 8];
        let n = encode(&[9, 9], &mut wire).unwrap();
        let mut got = None;
        for &b in &wire[..n] {
            if let Some(r) = dec.push(b) {
                got = Some(r.map(|f| f.len()));
            }
        }
        assert_eq!(got, Some(Ok(2)));
    }

    #[test]
    fn decoder_skips_empty_frames_and_flags_corruption() {
        let mut dec = Decoder::<8>::new();
        assert!(dec.push(0).is_none());
        // Code byte 5 claims 4 data bytes but only 1 follows.
        assert!(dec.push(5).is_none());
        assert!(dec.push(1).is_none());
        assert_eq!(dec.push(0), Some(Err(FrameError::Corrupt)));
    }
}
