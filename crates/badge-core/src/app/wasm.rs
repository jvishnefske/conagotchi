//! Wasm app control: `WasmLoad` / `WasmStart` / `WasmStop` / `GetWasm`.
//!
//! The core keeps no wasm state; it validates the request, hands it to the
//! runtime as [`Effect::Wasm`], and answers the request when the runtime
//! reports back with [`Input::Wasm`] (exactly one response per request,
//! FR-6). Log lines from the app become `WasmLog` events.

use badge_proto::{BadgeMsg_, ErrorCode, Event_, HostMsg_::Cmd, WasmLog, WasmStatus};
use heapless::{String, Vec};

use super::{App, Out};
use crate::io::{Effect, WASM_LOG_LEN, WasmCmd};

/// Largest module the runtime accepts (mirrors `wasm_abi::MAX_MODULE`).
pub const MAX_MODULE: u32 = 16 * 1024;

impl App {
    /// Validates a wasm request and forwards it to the runtime.
    pub(super) fn wasm_host(seq: u32, cmd: Cmd, out: &mut Out) {
        let cmd = match cmd {
            Cmd::WasmLoad(l) => {
                let len = u32::try_from(l.data.len()).unwrap_or(u32::MAX);
                let end = l.offset.checked_add(len);
                if l.total == 0 || l.total > MAX_MODULE || end.is_none_or(|e| e > l.total) {
                    out.error(seq, ErrorCode::BadArgument, "wasm chunk outside module");
                    return;
                }
                let mut data = Vec::new();
                if data.extend_from_slice(&l.data).is_err() {
                    out.error(seq, ErrorCode::BadArgument, "wasm chunk too long");
                    return;
                }
                WasmCmd::Load { offset: l.offset, total: l.total, data }
            }
            Cmd::WasmStart(_) => WasmCmd::Start,
            Cmd::WasmStop(_) => WasmCmd::Stop,
            Cmd::GetWasm(_) => WasmCmd::Status,
            _ => {
                out.error(seq, ErrorCode::Unsupported, "not a wasm request");
                return;
            }
        };
        out.push(Effect::Wasm { seq, cmd });
    }

    /// The runtime answered a wasm request.
    pub(super) fn wasm_done(seq: u32, result: Result<WasmStatus, ErrorCode>, out: &mut Out) {
        match result {
            Ok(status) => out.respond(seq, badge_proto::Response_::Body::Wasm(status)),
            Err(code) => out.error(seq, code, "wasm"),
        }
    }

    /// Forwards an app log line to the host.
    pub(super) fn wasm_log(text: String<WASM_LOG_LEN>, out: &mut Out) {
        out.send(BadgeMsg_::Kind::Event(badge_proto::Event {
            kind: Some(Event_::Kind::WasmLog(WasmLog { text })),
        }));
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use badge_proto::{
        BadgeMsg, BadgeMsg_, Event_, GetWasm, HostMsg, Response_::Body, WasmLoad, WasmStart,
        WasmState,
    };

    use super::*;
    use crate::app::BoardInfo;
    use crate::io::Input;

    fn app() -> App {
        App::new(BoardInfo { mac: [0; 6], firmware: "t" })
    }

    fn host(cmd: Cmd) -> crate::io::Effects {
        app().handle(Input::Host(HostMsg { seq: 7, cmd: Some(cmd) })).1
    }

    fn error_of(fx: &[Effect]) -> Option<ErrorCode> {
        fx.iter().find_map(|e| match e {
            Effect::Send(BadgeMsg { kind: Some(BadgeMsg_::Kind::Response(r)) }) => match &r.body {
                Some(Body::Error(e)) => Some(e.code),
                _ => None,
            },
            _ => None,
        })
    }

    fn chunk(offset: u32, total: u32, n: usize) -> Cmd {
        let mut data = heapless::Vec::new();
        data.resize(n, 0xAB).ok();
        Cmd::WasmLoad(WasmLoad { offset, data, total })
    }

    #[test]
    fn fr36_valid_requests_become_runtime_commands() {
        let fx = host(chunk(0, 300, 192));
        assert!(fx.iter().any(|e| matches!(
            e,
            Effect::Wasm { seq: 7, cmd: WasmCmd::Load { offset: 0, total: 300, data } } if data.len() == 192
        )));
        assert_eq!(error_of(&fx), None);
        let started = host(Cmd::WasmStart(WasmStart {}));
        assert!(started.iter().any(|e| matches!(e, Effect::Wasm { cmd: WasmCmd::Start, .. })));
        let status = host(Cmd::GetWasm(GetWasm {}));
        assert!(status.iter().any(|e| matches!(e, Effect::Wasm { cmd: WasmCmd::Status, .. })));
    }

    #[test]
    fn fr36_bad_chunks_are_rejected_without_reaching_the_runtime() {
        for cmd in [chunk(0, 0, 1), chunk(0, MAX_MODULE + 1, 1), chunk(200, 300, 192), chunk(u32::MAX, 300, 1)] {
            let fx = host(cmd);
            assert_eq!(error_of(&fx), Some(ErrorCode::BadArgument));
            assert!(!fx.iter().any(|e| matches!(e, Effect::Wasm { .. })));
        }
    }

    #[test]
    fn fr36_runtime_answer_is_the_one_response() {
        let status = WasmStatus { state: WasmState::Running, size: 300, ..Default::default() };
        let (_, fx) = app().handle(Input::Wasm { seq: 9, result: Ok(status.clone()) });
        let responses: std::vec::Vec<_> = fx
            .iter()
            .filter_map(|e| match e {
                Effect::Send(BadgeMsg { kind: Some(BadgeMsg_::Kind::Response(r)) }) => Some(r),
                _ => None,
            })
            .collect();
        assert_eq!(responses.len(), 1);
        assert_eq!(responses[0].seq, 9);
        assert_eq!(responses[0].body, Some(Body::Wasm(status)));
        let (_, fx) = app().handle(Input::Wasm { seq: 9, result: Err(ErrorCode::Busy) });
        assert_eq!(error_of(&fx), Some(ErrorCode::Busy));
    }

    #[test]
    fn fr36_log_lines_become_events() {
        let mut text = String::new();
        text.push_str("hello").ok();
        let (_, fx) = app().handle(Input::WasmLog(text.clone()));
        assert!(fx.iter().any(|e| matches!(
            e,
            Effect::Send(BadgeMsg { kind: Some(BadgeMsg_::Kind::Event(ev)) })
                if ev.kind == Some(Event_::Kind::WasmLog(WasmLog { text: text.clone() }))
        )));
    }
}
