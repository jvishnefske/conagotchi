//! The tagged binary record format of `sim_take_output`.
//!
//! The simulated badge turns every [`Effect`] the shell must carry out into
//! one record; the runner (`badgectl sim`) decodes them with [`Records`].
//! A record is
//!
//! - `tag`: one byte, a [`Tag`] value;
//! - `len`: the body length, `u16` little-endian;
//! - `body`: `len` bytes, laid out per tag (all integers little-endian):
//!   - [`Tag::Send`] holds one protobuf-encoded `BadgeMsg`;
//!   - [`Tag::Beacon`] holds the advertising data (complete AD structures);
//!   - [`Tag::Sync`] holds sync id `u32`, network time `u32`;
//!   - [`Tag::Leds`] holds 8 × (R, G, B);
//!   - [`Tag::Indicator`] holds `Indicator` value `i32`, on `u8` (0 / 1);
//!   - [`Tag::Screen`] holds one [`ScreenChange`] byte (the framebuffer was
//!     already updated by the guest);
//!   - [`Tag::Wasm`] holds request seq `u32` and a command byte ([`WasmOp`]),
//!     then per command: erase holds image length `u32`; write holds offset
//!     `u32` and the chunk bytes; activate holds image length `u32` and
//!     CRC-32 `u32`; start, stop and status hold nothing more;
//!   - [`Tag::LinkSend`] holds one framed link-UART packet;
//!   - [`Tag::SaveContact`] holds one protobuf-encoded `Contact`;
//!   - [`Tag::ClearContacts`] is empty.
//!
//! Effects the guest answers itself (GPIO, Wi-Fi, BLE scan and advertise)
//! never become records. Encoding never truncates: a record that does not fit
//! the buffer is refused whole ([`WireError::Full`]), so the buffer always
//! holds complete records. Decoding stops at the first malformed record.

#![forbid(unsafe_code)]

use badge_core::board::led::RGB_COUNT;
use badge_core::color::Color;
use badge_core::io::{Effect, WasmCmd};
use badge_proto::Indicator;

/// Bytes before a record body: tag plus length.
pub const HEADER_LEN: usize = 3;

/// Record kinds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Tag {
    /// A message for the host (protobuf `BadgeMsg`).
    Send = 1,
    /// New BLE advertising data.
    Beacon = 2,
    /// An LED sync beacon to broadcast.
    Sync = 3,
    /// New LED colours.
    Leds = 4,
    /// An indicator LED switched.
    Indicator = 5,
    /// The framebuffer changed.
    Screen = 6,
    /// A wasm runtime command.
    Wasm = 7,
    /// A link-UART frame.
    LinkSend = 8,
    /// Persist a contact (protobuf `Contact`).
    SaveContact = 9,
    /// Erase all contacts.
    ClearContacts = 10,
}

impl Tag {
    /// The tag for a record byte, `None` if unknown.
    pub const fn from_u8(b: u8) -> Option<Tag> {
        Some(match b {
            1 => Tag::Send,
            2 => Tag::Beacon,
            3 => Tag::Sync,
            4 => Tag::Leds,
            5 => Tag::Indicator,
            6 => Tag::Screen,
            7 => Tag::Wasm,
            8 => Tag::LinkSend,
            9 => Tag::SaveContact,
            10 => Tag::ClearContacts,
            _ => return None,
        })
    }
}

/// How the framebuffer changed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ScreenChange {
    /// Full re-render (`Effect::Redraw`).
    Redraw = 0,
    /// Animated region only (`Effect::Animate`).
    Animate = 1,
    /// A host canvas primitive (`Effect::Draw`).
    Draw = 2,
}

/// Payload command byte of a [`Tag::Wasm`] record (one per `WasmCmd`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum WasmOp {
    /// Erase the payload store for a new image.
    Erase = 0,
    /// Write one image chunk.
    Write = 1,
    /// Record the verified image as installed.
    Activate = 2,
    /// Start the installed payload.
    Start = 3,
    /// Stop the payload.
    Stop = 4,
    /// Report status.
    Status = 5,
}

/// Why a record could not be written.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WireError {
    /// The output buffer has no room for the whole record.
    Full,
    /// The body exceeds `u16::MAX` bytes or a protobuf message failed to
    /// encode.
    Encode,
    /// This effect is not carried as a record (the guest handles it).
    NotARecord,
}

/// A fixed-capacity buffer of complete records.
#[derive(Debug, Clone)]
pub struct OutBuf<const N: usize> {
    buf: [u8; N],
    len: usize,
}

impl<const N: usize> Default for OutBuf<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> OutBuf<N> {
    /// An empty buffer.
    pub const fn new() -> Self {
        Self { buf: [0; N], len: 0 }
    }

    /// The records written so far.
    pub fn as_bytes(&self) -> &[u8] {
        self.buf.get(..self.len).unwrap_or(&[])
    }

    /// Start of the storage (the exported output region).
    pub fn storage(&self) -> &[u8; N] {
        &self.buf
    }

    /// Bytes written.
    pub fn len(&self) -> usize {
        self.len
    }

    /// True if no record was written.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Drops every record.
    pub fn clear(&mut self) {
        self.len = 0;
    }

    /// Appends one record whose body is produced by `body` into the given
    /// slice (returning the body length). Nothing is kept on failure.
    fn record_with<F>(&mut self, tag: Tag, body: F) -> Result<(), WireError>
    where
        F: FnOnce(&mut [u8]) -> Result<usize, WireError>,
    {
        let start = self.len;
        let room = self.buf.get_mut(start..).ok_or(WireError::Full)?;
        let (head, rest) = room.split_at_mut_checked(HEADER_LEN).ok_or(WireError::Full)?;
        let n = body(rest)?;
        let len = u16::try_from(n).map_err(|_| WireError::Encode)?;
        let [lo, hi] = len.to_le_bytes();
        head.copy_from_slice(&[tag as u8, lo, hi]);
        self.len = start + HEADER_LEN + n;
        Ok(())
    }

    /// Appends a record from concatenated body parts.
    pub fn record(&mut self, tag: Tag, parts: &[&[u8]]) -> Result<(), WireError> {
        self.record_with(tag, |out| {
            let mut at = 0usize;
            for part in parts {
                let end = at.checked_add(part.len()).ok_or(WireError::Full)?;
                out.get_mut(at..end).ok_or(WireError::Full)?.copy_from_slice(part);
                at = end;
            }
            Ok(at)
        })
    }

    /// Appends a record holding one protobuf message.
    pub fn message<M: badge_proto::micropb::MessageEncode>(&mut self, tag: Tag, msg: &M) -> Result<(), WireError> {
        self.record_with(tag, |out| {
            badge_proto::encode(msg, out).map_err(|e| match e {
                badge_proto::CodecError::BufferTooSmall => WireError::Full,
                badge_proto::CodecError::Malformed => WireError::Encode,
            })
        })
    }

    /// Appends the record for `effect`.
    ///
    /// # Errors
    ///
    /// [`WireError::NotARecord`] for effects the guest answers or applies
    /// itself (GPIO, Wi-Fi, BLE requests, `Redraw` / `Animate` / `Draw`,
    /// which become [`OutBuf::screen`] records after rendering), plus the
    /// buffer errors.
    pub fn effect(&mut self, effect: &Effect) -> Result<(), WireError> {
        match effect {
            Effect::Send(msg) => self.message(Tag::Send, msg),
            Effect::Beacon(ad) => self.record(Tag::Beacon, &[ad]),
            Effect::SyncBroadcast { id, network_ms } => {
                self.record(Tag::Sync, &[&id.to_le_bytes(), &network_ms.to_le_bytes()])
            }
            Effect::Leds(colors) => {
                let mut rgb = [0u8; 3 * RGB_COUNT];
                for (slot, c) in rgb.as_chunks_mut::<3>().0.iter_mut().zip(colors.iter()) {
                    *slot = [c.r, c.g, c.b];
                }
                self.record(Tag::Leds, &[&rgb])
            }
            Effect::Indicator(which, on) => {
                self.record(Tag::Indicator, &[&which.0.to_le_bytes(), &[u8::from(*on)]])
            }
            Effect::Wasm { seq, cmd } => {
                let seq = seq.to_le_bytes();
                match cmd {
                    WasmCmd::Erase { len } => {
                        self.record(Tag::Wasm, &[&seq, &[WasmOp::Erase as u8], &len.to_le_bytes()])
                    }
                    WasmCmd::Write { offset, data } => {
                        self.record(Tag::Wasm, &[&seq, &[WasmOp::Write as u8], &offset.to_le_bytes(), data])
                    }
                    WasmCmd::Activate { len, crc } => self.record(
                        Tag::Wasm,
                        &[&seq, &[WasmOp::Activate as u8], &len.to_le_bytes(), &crc.to_le_bytes()],
                    ),
                    WasmCmd::Start => self.record(Tag::Wasm, &[&seq, &[WasmOp::Start as u8]]),
                    WasmCmd::Stop => self.record(Tag::Wasm, &[&seq, &[WasmOp::Stop as u8]]),
                    WasmCmd::Status => self.record(Tag::Wasm, &[&seq, &[WasmOp::Status as u8]]),
                }
            }
            Effect::LinkSend(frame) => self.record(Tag::LinkSend, &[frame]),
            Effect::SaveContact(c) => self.message(Tag::SaveContact, &c.to_proto()),
            Effect::ClearContacts => self.record(Tag::ClearContacts, &[]),
            // Expanded by the guest (`SimBadge`) into a `Send` record first.
            Effect::SendScan { .. } => Err(WireError::NotARecord),
            Effect::Redraw
            | Effect::Animate
            | Effect::Draw(_)
            | Effect::GpioRead { .. }
            | Effect::GpioWrite { .. }
            | Effect::AdcRead { .. }
            | Effect::WifiScan { .. }
            | Effect::WifiConnect { .. }
            | Effect::WifiStatus { .. }
            | Effect::BleScan { .. }
            | Effect::BleAdvertise { .. } => Err(WireError::NotARecord),
        }
    }

    /// Appends a framebuffer-changed record.
    pub fn screen(&mut self, change: ScreenChange) -> Result<(), WireError> {
        self.record(Tag::Screen, &[&[change as u8]])
    }
}

/// A payload command decoded from a [`Tag::Wasm`] record (mirrors
/// `badge_core::io::WasmCmd`, borrowing the chunk).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WasmRecord<'a> {
    /// Stop any running payload and erase the store for `len` bytes.
    Erase {
        /// Image length.
        len: u32,
    },
    /// Write `data` at `offset` in the image.
    Write {
        /// Byte offset in the image.
        offset: u32,
        /// Chunk bytes.
        data: &'a [u8],
    },
    /// The image is complete; record it as installed.
    Activate {
        /// Image length.
        len: u32,
        /// Image CRC-32, verified by the core.
        crc: u32,
    },
    /// Start the installed payload.
    Start,
    /// Stop the payload.
    Stop,
    /// Report status.
    Status,
}

/// One decoded record; slices borrow the record buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Record<'a> {
    /// Protobuf `BadgeMsg` bytes.
    Send(&'a [u8]),
    /// Advertising data.
    Beacon(&'a [u8]),
    /// Sync beacon to broadcast.
    Sync {
        /// Sender's sync id.
        id: u32,
        /// Sender's network time.
        network_ms: u32,
    },
    /// LED colours.
    Leds([Color; RGB_COUNT]),
    /// Indicator switched.
    Indicator {
        /// Which indicator.
        which: Indicator,
        /// New state.
        on: bool,
    },
    /// Framebuffer changed.
    Screen(ScreenChange),
    /// Wasm runtime command.
    Wasm {
        /// Request seq to answer with `sim_input_wasm`.
        seq: u32,
        /// Command.
        cmd: WasmRecord<'a>,
    },
    /// Link-UART frame.
    LinkSend(&'a [u8]),
    /// Protobuf `Contact` bytes.
    SaveContact(&'a [u8]),
    /// Erase contacts.
    ClearContacts,
}

/// Why a record could not be decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// The buffer ends inside a record.
    Truncated,
    /// Unknown tag byte.
    UnknownTag(u8),
    /// The body does not match its tag's layout.
    BadBody(Tag),
}

fn le_u32(b: &[u8]) -> Option<u32> {
    b.try_into().ok().map(u32::from_le_bytes)
}

fn le_i32(b: &[u8]) -> Option<i32> {
    b.try_into().ok().map(i32::from_le_bytes)
}

/// Decodes one record body.
fn decode_body(tag: Tag, body: &[u8]) -> Option<Record<'_>> {
    Some(match tag {
        Tag::Send => Record::Send(body),
        Tag::Beacon => Record::Beacon(body),
        Tag::Sync => {
            let (id, net) = body.split_at_checked(4)?;
            Record::Sync { id: le_u32(id)?, network_ms: le_u32(net)? }
        }
        Tag::Leds => {
            let rgb: &[u8; 3 * RGB_COUNT] = body.try_into().ok()?;
            let mut colors = [Color::BLACK; RGB_COUNT];
            for (c, &[r, g, b]) in colors.iter_mut().zip(rgb.as_chunks::<3>().0) {
                *c = Color::new(r, g, b);
            }
            Record::Leds(colors)
        }
        Tag::Indicator => {
            let (which, on) = body.split_at_checked(4)?;
            let on = match on {
                [0] => false,
                [1] => true,
                _ => return None,
            };
            Record::Indicator { which: Indicator(le_i32(which)?), on }
        }
        Tag::Screen => Record::Screen(match body {
            [0] => ScreenChange::Redraw,
            [1] => ScreenChange::Animate,
            [2] => ScreenChange::Draw,
            _ => return None,
        }),
        Tag::Wasm => {
            let (seq, rest) = body.split_at_checked(4)?;
            let (&op, rest) = rest.split_first()?;
            let cmd = match (op, rest) {
                (0, len) => WasmRecord::Erase { len: le_u32(len)? },
                (1, rest) => {
                    let (offset, data) = rest.split_at_checked(4)?;
                    WasmRecord::Write { offset: le_u32(offset)?, data }
                }
                (2, rest) => {
                    let (len, crc) = rest.split_at_checked(4)?;
                    WasmRecord::Activate { len: le_u32(len)?, crc: le_u32(crc)? }
                }
                (3, []) => WasmRecord::Start,
                (4, []) => WasmRecord::Stop,
                (5, []) => WasmRecord::Status,
                _ => return None,
            };
            Record::Wasm { seq: le_u32(seq)?, cmd }
        }
        Tag::LinkSend => Record::LinkSend(body),
        Tag::SaveContact => Record::SaveContact(body),
        Tag::ClearContacts if body.is_empty() => Record::ClearContacts,
        Tag::ClearContacts => return None,
    })
}

/// Iterator over the records of an output buffer.
///
/// Yields `Err` once for the first malformed record and then stops; it is
/// bounded by the buffer length (every record consumes at least
/// [`HEADER_LEN`] bytes).
#[derive(Debug, Clone)]
pub struct Records<'a> {
    rest: &'a [u8],
}

impl<'a> Records<'a> {
    /// Records in `bytes`.
    pub fn new(bytes: &'a [u8]) -> Self {
        Self { rest: bytes }
    }
}

impl<'a> Iterator for Records<'a> {
    type Item = Result<Record<'a>, DecodeError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.rest.is_empty() {
            return None;
        }
        let bytes = self.rest;
        self.rest = &[];
        let Some((&[tag, lo, hi], rest)) = bytes.split_first_chunk::<HEADER_LEN>() else {
            return Some(Err(DecodeError::Truncated));
        };
        let Some(tag) = Tag::from_u8(tag) else {
            return Some(Err(DecodeError::UnknownTag(tag)));
        };
        let Some((body, tail)) = rest.split_at_checked(usize::from(u16::from_le_bytes([lo, hi]))) else {
            return Some(Err(DecodeError::Truncated));
        };
        match decode_body(tag, body) {
            Some(record) => {
                self.rest = tail;
                Some(Ok(record))
            }
            None => Some(Err(DecodeError::BadBody(tag))),
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use std::vec::Vec;

    use badge_core::contacts::Contact;
    use badge_core::oz::PeerId;
    use badge_proto::{BadgeMsg, BadgeMsg_, Chi, Response, Response_::Body};

    use super::*;

    fn all(bytes: &[u8]) -> Vec<Result<Record<'_>, DecodeError>> {
        Records::new(bytes).collect()
    }

    #[test]
    fn fr39_every_record_round_trips() {
        let mut out = OutBuf::<2048>::new();
        let msg = BadgeMsg {
            kind: Some(BadgeMsg_::Kind::Response(Response { seq: 9, body: Some(Body::Ack(badge_proto::Ack {})) })),
        };
        let leds = [Color::new(1, 2, 3); RGB_COUNT];
        let contact = Contact {
            id: PeerId::from_u32(0xABCDEF),
            chi: Chi::Cry,
            name: heapless::String::new(),
            trades: 2,
        };
        let effects = [
            Effect::Send(msg.clone()),
            Effect::Beacon(heapless::Vec::from_slice(&[2, 1, 6]).unwrap()),
            Effect::SyncBroadcast { id: 0xC0FF_EE01, network_ms: 12345 },
            Effect::Leds(leds),
            Effect::Indicator(Indicator::Raffle, true),
            Effect::Wasm { seq: 3, cmd: WasmCmd::Erase { len: 400 } },
            Effect::Wasm { seq: 4, cmd: WasmCmd::Write { offset: 192, data: heapless::Vec::from_slice(&[9; 5]).unwrap() } },
            Effect::Wasm { seq: 8, cmd: WasmCmd::Activate { len: 400, crc: 0xDEAD_BEEF } },
            Effect::Wasm { seq: 5, cmd: WasmCmd::Start },
            Effect::Wasm { seq: 6, cmd: WasmCmd::Stop },
            Effect::Wasm { seq: 7, cmd: WasmCmd::Status },
            Effect::LinkSend(heapless::Vec::from_slice(&[0xAA, 0x55]).unwrap()),
            Effect::SaveContact(contact.clone()),
            Effect::ClearContacts,
        ];
        for e in &effects {
            out.effect(e).unwrap();
        }
        out.screen(ScreenChange::Animate).unwrap();
        let recs: Vec<Record<'_>> = Records::new(out.as_bytes()).map(Result::unwrap).collect();
        assert_eq!(recs.len(), effects.len() + 1);
        let Record::Send(pb) = recs[0] else { panic!("send") };
        assert_eq!(badge_proto::decode::<BadgeMsg>(pb), Ok(msg));
        assert_eq!(recs[1], Record::Beacon(&[2, 1, 6]));
        assert_eq!(recs[2], Record::Sync { id: 0xC0FF_EE01, network_ms: 12345 });
        assert_eq!(recs[3], Record::Leds(leds));
        assert_eq!(recs[4], Record::Indicator { which: Indicator::Raffle, on: true });
        assert_eq!(recs[5], Record::Wasm { seq: 3, cmd: WasmRecord::Erase { len: 400 } });
        assert_eq!(recs[6], Record::Wasm { seq: 4, cmd: WasmRecord::Write { offset: 192, data: &[9; 5] } });
        assert_eq!(recs[7], Record::Wasm { seq: 8, cmd: WasmRecord::Activate { len: 400, crc: 0xDEAD_BEEF } });
        assert_eq!(recs[8], Record::Wasm { seq: 5, cmd: WasmRecord::Start });
        assert_eq!(recs[9], Record::Wasm { seq: 6, cmd: WasmRecord::Stop });
        assert_eq!(recs[10], Record::Wasm { seq: 7, cmd: WasmRecord::Status });
        assert_eq!(recs[11], Record::LinkSend(&[0xAA, 0x55]));
        let Record::SaveContact(pb) = recs[12] else { panic!("contact") };
        assert_eq!(badge_proto::decode::<badge_proto::Contact>(pb), Ok(contact.to_proto()));
        assert_eq!(recs[13], Record::ClearContacts);
        assert_eq!(recs[14], Record::Screen(ScreenChange::Animate));
    }

    #[test]
    fn fr39_header_layout_is_tag_then_le_length() {
        let mut out = OutBuf::<16>::new();
        out.effect(&Effect::SyncBroadcast { id: 1, network_ms: 2 }).unwrap();
        assert_eq!(out.as_bytes(), &[3, 8, 0, 1, 0, 0, 0, 2, 0, 0, 0]);
        out.clear();
        assert!(out.is_empty());
        assert_eq!(all(out.as_bytes()), []);
    }

    #[test]
    fn fr39_full_buffer_refuses_whole_records() {
        let mut out = OutBuf::<12>::new();
        out.record(Tag::Beacon, &[&[1, 2, 3, 4]]).unwrap();
        assert_eq!(out.len(), 7);
        assert_eq!(out.record(Tag::Beacon, &[&[1, 2, 3]]), Err(WireError::Full));
        assert_eq!(out.len(), 7, "nothing partial kept");
        out.record(Tag::Beacon, &[&[1, 2]]).unwrap();
        assert_eq!(out.len(), 12);
        assert_eq!(out.screen(ScreenChange::Redraw), Err(WireError::Full));
        assert_eq!(all(out.as_bytes()).len(), 2);
    }

    #[test]
    fn fr39_guest_handled_effects_are_not_records() {
        let mut out = OutBuf::<64>::new();
        for e in [Effect::Redraw, Effect::Animate, Effect::GpioRead { seq: 1, pin: 2 }, Effect::WifiScan { seq: None }] {
            assert_eq!(out.effect(&e), Err(WireError::NotARecord));
        }
        assert!(out.is_empty());
    }

    #[test]
    fn fr39_malformed_records_stop_decoding() {
        assert_eq!(all(&[1, 5]), [Err(DecodeError::Truncated)]);
        assert_eq!(all(&[2, 4, 0, 1]), [Err(DecodeError::Truncated)]);
        assert_eq!(all(&[99, 0, 0]), [Err(DecodeError::UnknownTag(99))]);
        assert_eq!(all(&[3, 1, 0, 0]), [Err(DecodeError::BadBody(Tag::Sync))]);
        assert_eq!(all(&[6, 1, 0, 7]), [Err(DecodeError::BadBody(Tag::Screen))]);
        assert_eq!(all(&[7, 5, 0, 1, 0, 0, 0, 9]), [Err(DecodeError::BadBody(Tag::Wasm))]);
        // Erase with a short length, activate without its CRC, start with a body.
        assert_eq!(all(&[7, 7, 0, 1, 0, 0, 0, 0, 1, 0]), [Err(DecodeError::BadBody(Tag::Wasm))]);
        assert_eq!(all(&[7, 9, 0, 1, 0, 0, 0, 2, 1, 0, 0, 0]), [Err(DecodeError::BadBody(Tag::Wasm))]);
        assert_eq!(all(&[7, 6, 0, 1, 0, 0, 0, 3, 0]), [Err(DecodeError::BadBody(Tag::Wasm))]);
        assert_eq!(all(&[5, 5, 0, 1, 0, 0, 0, 2]), [Err(DecodeError::BadBody(Tag::Indicator))]);
        assert_eq!(all(&[10, 1, 0, 0]), [Err(DecodeError::BadBody(Tag::ClearContacts))]);
        // A good record, then garbage: one Ok, one Err, then the end.
        assert_eq!(all(&[10, 0, 0, 1]), [Ok(Record::ClearContacts), Err(DecodeError::Truncated)]);
        for b in 1..=10 {
            assert_eq!(Tag::from_u8(b).map(|t| t as u8), Some(b));
        }
        assert_eq!(Tag::from_u8(0), None);
    }
}
