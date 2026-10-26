//! Middleware's serial loader protocol (`middleware::loader`) mapped onto
//! the badge core, so a browser (the middleware web app over Web Serial) or
//! `mwctl` can install and run payloads over the badge's USB-Serial-JTAG CDC
//! port exactly as on an esp-base board.
//!
//! The firmware's serial link task does the I/O; everything it decides is
//! here and pure:
//!
//! - [`route_request`]: a request on a `sys/` key becomes a core command
//!   (fed to the core as `Input::Host` with a seq from
//!   `probe_layout::SERIAL_SEQ`), plus what to do with the core's answer
//!   ([`Pending`]). `sys/info` is answered from `GetWasm` so it reports the
//!   live payload state.
//! - [`route_publish`]: a publish from the host on an `app/...` key is
//!   delivered to the running payload; anything else is refused.
//! - [`reply`]: the core's `Response` body for that seq becomes the
//!   middleware response body ([`DeviceInfo`] for `sys/info`, empty for the
//!   loader steps) or a [`HandlerError`].
//! - [`payload_status`]: `WasmState` as the `sys/payload/status` byte.
//! - [`serial_answer`] and [`wasm_log`]: what the shell taps out of the
//!   core's `BadgeMsg`s for the serial link. A response whose seq is in
//!   `SERIAL_SEQ` becomes a [`CoreAnswer`] (only the part [`reply`] reads,
//!   so the queue to the link task holds 80 bytes, not a whole `Body`); a
//!   `WasmLog` event's text goes out as a `sys/log` publish.
//!
//! | middleware key      | core command  | response                  |
//! |---------------------|---------------|---------------------------|
//! | `sys/info`          | `GetWasm`     | [`DeviceInfo`]            |
//! | `sys/loader/begin`  | `WasmBegin`   | empty                     |
//! | `sys/loader/chunk`  | `WasmChunk`   | empty                     |
//! | `sys/loader/commit` | `WasmCommit`  | empty                     |
//! | `sys/payload/start` | `WasmStart`   | empty                     |
//! | `sys/payload/stop`  | `WasmStop`    | empty                     |
//!
//! The badge's chunk bound (`badge_core::io::WASM_CHUNK`) equals
//! `loader::CHUNK_MAX`, so every valid chunk maps onto one `WasmChunk`.

use badge_core::probe_layout::is_serial_seq;
use badge_proto::{
    BadgeMsg, BadgeMsg_, ErrorCode, Event_, GetWasm, WasmStatus, HostMsg_::Cmd, Response_::Body, WasmBegin, WasmChunk, WasmCommit,
    WasmStart, WasmState, WasmStop,
};
use middleware::loader::{self, Command, DeviceInfo, PayloadStatus};
use middleware::transport::HandlerError;
use middleware::Key;

use crate::keys;

/// Chip name the badge reports in `sys/info`, so hosts can tell it from an
/// esp-base ESP32-S3 (different board features, same loader).
pub const CHIP: &str = "esp32s3-conagotchi";

/// How to turn the core's answer to a routed request into the middleware
/// response.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pending {
    /// `sys/info`: answer with [`DeviceInfo`] built from the `WasmStatus`.
    Info,
    /// A loader or payload step: answer with an empty body.
    Ack,
}

/// A request routed to the core.
// `Cmd` holds a whole `WasmChunk` (no heap to box into); the value is
// short-lived and its size is bounded.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, PartialEq)]
pub struct CoreRequest {
    /// The command to feed the core.
    pub cmd: Cmd,
    /// How to answer once the core responds.
    pub pending: Pending,
}

/// Maps a middleware request onto a core command.
///
/// Errors: a non-`sys/` key is [`HandlerError::NotFound`] (the badge serves
/// no application requests over serial); a malformed body is
/// [`HandlerError::InvalidRequest`] (from [`Command::parse`]).
pub fn route_request(key: Key<'_>, body: &[u8]) -> Result<CoreRequest, HandlerError> {
    let command = Command::parse(key, body)?.ok_or(HandlerError::NotFound)?;
    let (cmd, pending) = match command {
        Command::Info => (Cmd::GetWasm(GetWasm {}), Pending::Info),
        Command::Begin { len, crc } => (Cmd::WasmBegin(WasmBegin { len, crc32: crc }), Pending::Ack),
        Command::Chunk { offset, data } => {
            let data = heapless::Vec::from_slice(data).map_err(|_| HandlerError::InvalidRequest)?;
            (Cmd::WasmChunk(WasmChunk { offset, data }), Pending::Ack)
        }
        Command::Commit => (Cmd::WasmCommit(WasmCommit {}), Pending::Ack),
        Command::Start => (Cmd::WasmStart(WasmStart {}), Pending::Ack),
        Command::Stop => (Cmd::WasmStop(WasmStop {}), Pending::Ack),
    };
    Ok(CoreRequest { cmd, pending })
}

/// What a publish from the host does.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PublishRoute {
    /// Deliver to the running payload as an `app/...` sample.
    App,
}

/// Decides what a host publish on `key` does: `app/...` keys go to the
/// payload; `sys/`, `badge/` and every other key are refused
/// ([`HandlerError::NotFound`]) so the host cannot spoof badge-internal
/// keys.
pub fn route_publish(key: Key<'_>) -> Result<PublishRoute, HandlerError> {
    if keys::app_may_use(key.as_str()) {
        Ok(PublishRoute::App)
    } else {
        Err(HandlerError::NotFound)
    }
}

/// The `sys/payload/status` value for a core payload state.
pub const fn payload_status(state: WasmState) -> PayloadStatus {
    match state {
        WasmState::Stopped => PayloadStatus::Stopped,
        WasmState::Running => PayloadStatus::Running,
        WasmState::Trapped => PayloadStatus::Trapped,
        WasmState::Loading => PayloadStatus::Loading,
        _ => PayloadStatus::Empty,
    }
}

/// Maps a core error code onto the middleware status.
pub const fn handler_error(code: ErrorCode) -> HandlerError {
    match code {
        ErrorCode::BadArgument | ErrorCode::Decode | ErrorCode::PinNotAllowed => {
            HandlerError::InvalidRequest
        }
        ErrorCode::Unsupported => HandlerError::NotFound,
        _ => HandlerError::Internal,
    }
}

/// Firmware identity reported in `sys/info`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Identity<'a> {
    /// `wasm_abi::ABI_VERSION` of the badge's wasm-host.
    pub abi_version: u16,
    /// Firmware name and version (at most `loader::MAX_NAME` bytes).
    pub firmware: &'a str,
}

/// The part of a core response body that [`reply`] depends on, small enough
/// to queue between tasks (a `Body` holds every response kind, a WasmStatus
/// is 72 bytes).
#[derive(Debug, Clone, PartialEq)]
pub enum CoreAnswer {
    /// `Body::Wasm`.
    Status(WasmStatus),
    /// `Body::Error` (its code; the detail text is not reported).
    Error(ErrorCode),
    /// Any other body kind.
    Other,
}

impl CoreAnswer {
    /// Extracts the answer from a core response body.
    pub fn of(body: &Body) -> Self {
        match body {
            Body::Wasm(status) => CoreAnswer::Status(status.clone()),
            Body::Error(e) => CoreAnswer::Error(e.code),
            _ => CoreAnswer::Other,
        }
    }
}

/// Turns the core's response body for a routed request into the middleware
/// response body, written to `out`; returns its length.
///
/// `Error` bodies map through [`handler_error`]; a body of the wrong kind
/// (not `WasmStatus` or `Error`) is [`HandlerError::Internal`]; an `out`
/// too small for the [`DeviceInfo`] is [`HandlerError::BufferTooSmall`].
pub fn reply(
    pending: Pending,
    body: &Body,
    id: Identity<'_>,
    out: &mut [u8],
) -> Result<usize, HandlerError> {
    reply_answer(pending, &CoreAnswer::of(body), id, out)
}

/// [`reply`] for an answer already extracted with [`CoreAnswer::of`].
pub fn reply_answer(
    pending: Pending,
    answer: &CoreAnswer,
    id: Identity<'_>,
    out: &mut [u8],
) -> Result<usize, HandlerError> {
    match answer {
        CoreAnswer::Error(code) => Err(handler_error(*code)),
        CoreAnswer::Status(status) => match pending {
            Pending::Ack => Ok(0),
            Pending::Info => DeviceInfo {
                abi_version: id.abi_version,
                capacity: status.capacity,
                status: payload_status(status.state),
                chip: CHIP,
                firmware: id.firmware,
            }
            .encode(out)
            .ok_or(HandlerError::BufferTooSmall),
        },
        CoreAnswer::Other => Err(HandlerError::Internal),
    }
}

/// If `msg` is the core's response to a serial request (seq in
/// `probe_layout::SERIAL_SEQ`), its seq and answer; the shell sends it to the
/// serial link instead of RTT.
pub fn serial_answer(msg: &BadgeMsg) -> Option<(u32, CoreAnswer)> {
    match &msg.kind {
        Some(BadgeMsg_::Kind::Response(r)) if is_serial_seq(r.seq) => {
            Some((r.seq, r.body.as_ref().map_or(CoreAnswer::Other, CoreAnswer::of)))
        }
        _ => None,
    }
}

/// The text of a `WasmLog` event, if `msg` is one (published on [`LOG`]).
pub fn wasm_log(msg: &BadgeMsg) -> Option<&str> {
    match &msg.kind {
        Some(BadgeMsg_::Kind::Event(event)) => match &event.kind {
            Some(Event_::Kind::WasmLog(log)) => Some(log.text.as_str()),
            _ => None,
        },
        _ => None,
    }
}

/// The key device log lines are published on (`sys/log`).
pub const LOG: Key<'static> = loader::keys::LOG;
/// The key payload state changes are published on (`sys/payload/status`).
pub const STATUS: Key<'static> = loader::keys::STATUS;

#[cfg(test)]
mod tests {
    use super::*;
    use badge_core::probe_layout::SERIAL_SEQ;
    use badge_proto::{Ack, Error, Event, Response, WasmLog};

    const ID: Identity<'static> = Identity { abi_version: 3, firmware: "conagotchi 0.1.0" };

    fn status(state: WasmState) -> Body {
        Body::Wasm(WasmStatus {
            state,
            size: 100,
            ticks: 0,
            error: heapless::String::new(),
            capacity: 16 * 1024,
        })
    }

    #[test]
    fn fr44_loader_keys_map_onto_core_wasm_commands() {
        let begin = [&1000u32.to_le_bytes()[..], &0xDEAD_BEEFu32.to_le_bytes()].concat();
        assert_eq!(
            route_request(loader::keys::BEGIN, &begin),
            Ok(CoreRequest {
                cmd: Cmd::WasmBegin(WasmBegin { len: 1000, crc32: 0xDEAD_BEEF }),
                pending: Pending::Ack
            })
        );
        let mut chunk = 512u32.to_le_bytes().to_vec();
        chunk.extend_from_slice(&[7; loader::CHUNK_MAX]);
        let Ok(CoreRequest { cmd: Cmd::WasmChunk(c), pending: Pending::Ack }) =
            route_request(loader::keys::CHUNK, &chunk)
        else {
            panic!("chunk")
        };
        assert_eq!((c.offset, c.data.len()), (512, loader::CHUNK_MAX));
        for (key, cmd) in [
            (loader::keys::COMMIT, Cmd::WasmCommit(WasmCommit {})),
            (loader::keys::START, Cmd::WasmStart(WasmStart {})),
            (loader::keys::STOP, Cmd::WasmStop(WasmStop {})),
        ] {
            assert_eq!(route_request(key, &[]), Ok(CoreRequest { cmd, pending: Pending::Ack }));
        }
        assert_eq!(
            route_request(loader::keys::INFO, &[]),
            Ok(CoreRequest { cmd: Cmd::GetWasm(GetWasm {}), pending: Pending::Info })
        );
    }

    #[test]
    fn fr44_bad_requests_are_refused() {
        assert_eq!(route_request(loader::keys::BEGIN, &[1, 2]), Err(HandlerError::InvalidRequest));
        let mut big = 0u32.to_le_bytes().to_vec();
        big.extend_from_slice(&[0; loader::CHUNK_MAX + 1]);
        assert_eq!(route_request(loader::keys::CHUNK, &big), Err(HandlerError::InvalidRequest));
        assert_eq!(route_request(loader::keys::COMMIT, &[1]), Err(HandlerError::InvalidRequest));
        assert_eq!(route_request(Key::from_static("sys/other"), &[]), Err(HandlerError::NotFound));
        assert_eq!(route_request(Key::from_static("app/x"), &[]), Err(HandlerError::NotFound));
    }

    #[test]
    fn fr44_host_publishes_reach_only_app_keys() {
        assert_eq!(route_publish(Key::from_static("app/hello")), Ok(PublishRoute::App));
        for k in ["badge/leds", "sys/log", "conagotchi/sync", "app"] {
            assert_eq!(route_publish(Key::new(k).unwrap()), Err(HandlerError::NotFound), "{k}");
        }
    }

    #[test]
    fn fr44_info_reply_is_a_device_info_from_the_live_status() {
        let mut out = [0u8; 80];
        let n = reply(Pending::Info, &status(WasmState::Running), ID, &mut out).unwrap();
        let info = DeviceInfo::decode(&out[..n]).unwrap();
        assert_eq!(
            info,
            DeviceInfo {
                abi_version: 3,
                capacity: 16 * 1024,
                status: PayloadStatus::Running,
                chip: CHIP,
                firmware: "conagotchi 0.1.0",
            }
        );
        assert_eq!(
            reply(Pending::Info, &status(WasmState::Running), ID, &mut [0; 4]),
            Err(HandlerError::BufferTooSmall)
        );
    }

    #[test]
    fn fr44_step_replies_are_empty_and_errors_map_to_status() {
        assert_eq!(reply(Pending::Ack, &status(WasmState::Loading), ID, &mut []), Ok(0));
        let err = |code| Body::Error(Error { code, detail: heapless::String::new() });
        assert_eq!(
            reply(Pending::Ack, &err(ErrorCode::BadArgument), ID, &mut []),
            Err(HandlerError::InvalidRequest)
        );
        assert_eq!(
            reply(Pending::Ack, &err(ErrorCode::Busy), ID, &mut []),
            Err(HandlerError::Internal)
        );
        assert_eq!(
            reply(Pending::Ack, &err(ErrorCode::Unsupported), ID, &mut []),
            Err(HandlerError::NotFound)
        );
        assert_eq!(
            reply(Pending::Ack, &Body::Ack(Ack {}), ID, &mut []),
            Err(HandlerError::Internal)
        );
    }

    #[test]
    fn fr44_payload_states_map_to_status_bytes() {
        assert_eq!(payload_status(WasmState::Empty), PayloadStatus::Empty);
        assert_eq!(payload_status(WasmState::Stopped), PayloadStatus::Stopped);
        assert_eq!(payload_status(WasmState::Running), PayloadStatus::Running);
        assert_eq!(payload_status(WasmState::Trapped), PayloadStatus::Trapped);
        assert_eq!(payload_status(WasmState::Loading), PayloadStatus::Loading);
        assert_eq!(payload_status(WasmState(99)), PayloadStatus::Empty);
    }

    #[test]
    fn fr44_serial_responses_and_logs_are_tapped_from_core_messages() {
        let response = |seq, body| BadgeMsg {
            kind: Some(BadgeMsg_::Kind::Response(Response { seq, body: Some(body) })),
        };
        let seq = SERIAL_SEQ.start + 5;
        let running = status(WasmState::Running);
        let Body::Wasm(st) = &running else { panic!("status") };
        assert_eq!(
            serial_answer(&response(seq, running.clone())),
            Some((seq, CoreAnswer::Status(st.clone())))
        );
        let err = Body::Error(Error { code: ErrorCode::Busy, detail: heapless::String::new() });
        assert_eq!(serial_answer(&response(seq, err)), Some((seq, CoreAnswer::Error(ErrorCode::Busy))));
        assert_eq!(serial_answer(&response(seq, Body::Ack(Ack {}))), Some((seq, CoreAnswer::Other)));
        // RTT seqs stay on RTT.
        assert_eq!(serial_answer(&response(7, running)), None);
        let log = BadgeMsg {
            kind: Some(BadgeMsg_::Kind::Event(Event {
                kind: Some(Event_::Kind::WasmLog(WasmLog { text: heapless::String::try_from("hi").unwrap() })),
            })),
        };
        assert_eq!(wasm_log(&log), Some("hi"));
        assert_eq!(serial_answer(&log), None);
        assert_eq!(wasm_log(&response(seq, Body::Ack(Ack {}))), None);
    }

    #[test]
    fn fr44_reply_answer_agrees_with_reply() {
        let bodies = [
            status(WasmState::Stopped),
            Body::Error(Error { code: ErrorCode::BadArgument, detail: heapless::String::new() }),
            Body::Ack(Ack {}),
        ];
        for body in &bodies {
            for pending in [Pending::Info, Pending::Ack] {
                let (mut a, mut b) = ([0u8; 80], [0u8; 80]);
                let ra = reply(pending, body, ID, &mut a);
                let rb = reply_answer(pending, &CoreAnswer::of(body), ID, &mut b);
                assert_eq!(ra, rb);
                assert_eq!(a, b);
            }
        }
    }
}
