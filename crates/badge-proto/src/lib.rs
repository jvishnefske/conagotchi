//! Conagotchi discovery protocol types.
//!
//! The Rust types are generated at build time from `proto/badge.proto` by
//! `micropb-gen` and re-exported here. Every variable-length field is a
//! fixed-capacity `heapless` container, so encoding and decoding never
//! allocate. The same crate is used by the firmware (no_std) and by the host
//! tool `badgectl` (std).
//!
//! [`encode`] and [`decode`] are the only entry points callers need; framing
//! (COBS) lives in `badge-core::frame`.

#![no_std]

#[allow(clippy::all, nonstandard_style, unused, irrefutable_let_patterns)]
mod generated {
    include!(concat!(env!("OUT_DIR"), "/badge.rs"));
}

pub use generated::badge_::*;
pub use micropb;

use micropb::{MessageDecode, MessageEncode, PbDecoder, PbEncoder};

/// Upper bound on the encoded size of any [`HostMsg`].
pub const HOST_MSG_MAX: usize = match HostMsg::MAX_SIZE {
    Ok(n) => n,
    Err(_) => 512,
};

/// Upper bound on the encoded size of any [`BadgeMsg`].
pub const BADGE_MSG_MAX: usize = match BadgeMsg::MAX_SIZE {
    Ok(n) => n,
    Err(_) => 1024,
};

/// Why a message could not be encoded or decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CodecError {
    /// The output buffer is smaller than the encoded message.
    BufferTooSmall,
    /// The input bytes are not a valid message (or exceed a field capacity).
    Malformed,
}

/// Encodes `msg` into `out`, returning the number of bytes written.
pub fn encode<M: MessageEncode>(msg: &M, out: &mut [u8]) -> Result<usize, CodecError> {
    let size = msg.compute_size();
    let dst = out.get_mut(..size).ok_or(CodecError::BufferTooSmall)?;
    let mut encoder = PbEncoder::new(SliceWriter { buf: dst, len: 0 });
    msg.encode(&mut encoder).map_err(|_| CodecError::BufferTooSmall)?;
    Ok(encoder.into_writer().len)
}

/// Decodes a complete message from `bytes`.
pub fn decode<M: MessageDecode + Default>(bytes: &[u8]) -> Result<M, CodecError> {
    let mut msg = M::default();
    let mut decoder = PbDecoder::new(bytes);
    msg.decode(&mut decoder, bytes.len())
        .map_err(|_| CodecError::Malformed)?;
    Ok(msg)
}

/// `micropb::PbWrite` over a borrowed byte slice.
struct SliceWriter<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl micropb::PbWrite for SliceWriter<'_> {
    type Error = CodecError;

    fn pb_write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        let end = self.len.checked_add(data.len()).ok_or(CodecError::BufferTooSmall)?;
        let dst = self
            .buf
            .get_mut(self.len..end)
            .ok_or(CodecError::BufferTooSmall)?;
        dst.copy_from_slice(data);
        self.len = end;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn host_msg_round_trips() {
        let mut text = heapless::String::new();
        text.push_str("hello\nbadge").ok();
        let msg = HostMsg {
            seq: 7,
            cmd: Some(HostMsg_::Cmd::DisplayText(DisplayText {
                text,
                fg: Some(Rgb { rgb: 0x00ff_8000 }),
                bg: None,
            })),
        };
        let mut buf = [0u8; HOST_MSG_MAX];
        let n = encode(&msg, &mut buf).expect("encode");
        let back: HostMsg = decode(&buf[..n]).expect("decode");
        assert_eq!(back, msg);
    }

    #[test]
    fn badge_msg_round_trips_with_full_scan() {
        let mut devices = heapless::Vec::new();
        for i in 0..12u8 {
            let mut addr = heapless::Vec::new();
            addr.extend_from_slice(&[i; 6]).ok();
            let mut name = heapless::String::new();
            name.push_str("conagotchi-device-name-xxxxxx").ok();
            devices
                .push(BleDevice { addr, addr_kind: 1, rssi: -40 - i32::from(i), name })
                .ok();
        }
        let msg = BadgeMsg {
            kind: Some(BadgeMsg_::Kind::Response(Response {
                seq: u32::MAX,
                body: Some(Response_::Body::BleScan(BleScanResult { devices })),
            })),
        };
        let mut buf = [0u8; BADGE_MSG_MAX];
        let n = encode(&msg, &mut buf).expect("encode");
        assert!(n <= BADGE_MSG_MAX);
        let back: BadgeMsg = decode(&buf[..n]).expect("decode");
        assert_eq!(back, msg);
    }

    #[test]
    fn encode_reports_small_buffer() {
        let msg = HostMsg { seq: 1, cmd: Some(HostMsg_::Cmd::Ping(Ping { nonce: 99 })) };
        let mut buf = [0u8; 2];
        assert_eq!(encode(&msg, &mut buf), Err(CodecError::BufferTooSmall));
    }

    #[test]
    fn decode_rejects_garbage() {
        let garbage = [0xff_u8; 8];
        assert_eq!(decode::<HostMsg>(&garbage), Err(CodecError::Malformed));
    }

    #[test]
    fn decode_rejects_oversized_string() {
        // DisplayText.text is capped at 256 bytes; 300 must be rejected.
        let mut wire = [0u8; 320];
        // HostMsg.display_text (field 10, LEN) { text (field 1, LEN) 300 bytes }
        let inner_len = 3 + 300; // tag + 2-byte varint + payload
        wire[0] = (10 << 3) | 2;
        wire[1] = (inner_len as u8 & 0x7f) | 0x80;
        wire[2] = (inner_len >> 7) as u8;
        wire[3] = (1 << 3) | 2;
        wire[4] = (300u16 as u8 & 0x7f) | 0x80;
        wire[5] = (300u16 >> 7) as u8;
        for b in &mut wire[6..306] {
            *b = b'a';
        }
        assert_eq!(decode::<HostMsg>(&wire[..306]), Err(CodecError::Malformed));
    }
}
