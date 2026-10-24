//! RTT transport: defmt logs plus the protobuf host link.
//!
//! [`init`] sets up the RTT control block with three channels:
//!
//! | Direction | # | Name         | Size  | Use                                   |
//! |-----------|---|--------------|-------|---------------------------------------|
//! | up        | 0 | `defmt`      | 2048  | defmt log frames (global logger)      |
//! | up        | 1 | `proto-up`   | 4096  | COBS-framed `BadgeMsg`, badge → host  |
//! | down      | 0 | `proto-down` | 1024  | COBS-framed `HostMsg`, host → badge   |
//!
//! Both proto channels are `NoBlockSkip`: if the host is not reading, a
//! complete frame is dropped rather than stalling the badge or emitting a
//! truncated frame.
//!
//! [`HostLink::next_input`] drains the down channel through a bounded
//! [`frame::Decoder`] and turns each complete frame into
//! [`Input::Host`] or [`Input::HostGarbage`]. [`HostLink::send`] encodes a
//! `BadgeMsg` with `badge_proto::encode`, COBS-frames it and writes it to the
//! up channel. All buffers live inside [`HostLink`], which `main` places in a
//! `StaticCell`; nothing here allocates.

use badge_core::frame::{self, Decoder};
use badge_core::io::Input;
use badge_proto::{BADGE_MSG_MAX, BadgeMsg, HOST_MSG_MAX, HostMsg};
use rtt_target::{DownChannel, UpChannel};

/// Encoded capacity of the host-frame decoder.
const RX_FRAME_MAX: usize = frame::max_encoded_len(HOST_MSG_MAX);
/// Encoded capacity of one outgoing frame.
const TX_FRAME_MAX: usize = frame::max_encoded_len(BADGE_MSG_MAX);
/// Bytes pulled from the down channel per read.
const RX_CHUNK: usize = 64;
/// Upper bound on bytes consumed by one [`HostLink::next_input`] call
/// (the size of the down channel buffer), so a call is always bounded.
const RX_BUDGET: usize = 1024;

/// Why a message could not be sent to the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum SendError {
    /// Protobuf encoding failed (message larger than `BADGE_MSG_MAX`).
    Encode,
    /// COBS framing failed (frame buffer too small).
    Frame,
    /// The up channel had no room; the frame was dropped whole.
    ChannelFull,
}

/// Owner of the two proto RTT channels and their buffers.
pub struct HostLink {
    up: UpChannel,
    down: DownChannel,
    decoder: Decoder<RX_FRAME_MAX>,
    rx_chunk: [u8; RX_CHUNK],
    rx_pos: usize,
    rx_len: usize,
    pb_buf: [u8; BADGE_MSG_MAX],
    frame_buf: [u8; TX_FRAME_MAX],
}

/// Initialises RTT, installs channel 0 as the defmt logger and returns the
/// proto link. Must be called exactly once, before any defmt logging.
pub fn init() -> HostLink {
    let channels = rtt_target::rtt_init! {
        up: {
            0: {
                size: 2048,
                mode: rtt_target::ChannelMode::NoBlockSkip,
                name: "defmt"
            }
            1: {
                size: 4096,
                mode: rtt_target::ChannelMode::NoBlockSkip,
                name: "proto-up"
            }
        }
        down: {
            0: {
                size: 1024,
                name: "proto-down"
            }
        }
    };
    rtt_target::set_defmt_channel(channels.up.0);
    HostLink::new(channels.up.1, channels.down.0)
}

impl HostLink {
    fn new(up: UpChannel, down: DownChannel) -> Self {
        Self {
            up,
            down,
            decoder: Decoder::new(),
            rx_chunk: [0; RX_CHUNK],
            rx_pos: 0,
            rx_len: 0,
            pb_buf: [0; BADGE_MSG_MAX],
            frame_buf: [0; TX_FRAME_MAX],
        }
    }

    /// Returns the next complete host frame as an [`Input`], or `None` when
    /// no complete frame is available yet.
    ///
    /// Consumes at most [`RX_BUDGET`] bytes per call; partial frames stay in
    /// the decoder until the next call.
    pub fn next_input(&mut self) -> Option<Input> {
        for _ in 0..RX_BUDGET {
            if self.rx_pos >= self.rx_len {
                self.rx_len = self.down.read(&mut self.rx_chunk);
                self.rx_pos = 0;
                if self.rx_len == 0 {
                    return None;
                }
            }
            let byte = self.rx_chunk.get(self.rx_pos).copied().unwrap_or(0);
            self.rx_pos += 1;
            if let Some(result) = self.decoder.push(byte) {
                return Some(match result {
                    Ok(payload) => match badge_proto::decode::<HostMsg>(payload) {
                        Ok(msg) => Input::Host(msg),
                        Err(_) => Input::HostGarbage,
                    },
                    Err(_) => Input::HostGarbage,
                });
            }
        }
        None
    }

    /// Encodes, frames and writes `msg` to the `proto-up` channel.
    pub fn send(&mut self, msg: &BadgeMsg) -> Result<(), SendError> {
        let n = badge_proto::encode(msg, &mut self.pb_buf).map_err(|_| SendError::Encode)?;
        let payload = self.pb_buf.get(..n).ok_or(SendError::Encode)?;
        let len = frame::encode(payload, &mut self.frame_buf).map_err(|_| SendError::Frame)?;
        let wire = self.frame_buf.get(..len).ok_or(SendError::Frame)?;
        if self.up.write(wire) == len {
            Ok(())
        } else {
            Err(SendError::ChannelFull)
        }
    }
}
