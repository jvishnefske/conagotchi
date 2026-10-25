//! Wasm payload control: `WasmBegin` / `WasmChunk` / `WasmCommit` /
//! `WasmStart` / `WasmStop` / `GetWasm`.
//!
//! The upload runs middleware's pure loader state machine
//! (`middleware::loader::Loader`): chunks must arrive in order, the length
//! must match and the CRC-32 is checked at commit. A rejected step answers
//! `BAD_ARGUMENT` at once and resets the transfer; an accepted step becomes
//! an [`Effect::Wasm`] (`Erase` / `Write` / `Activate`) that the runtime
//! carries out and answers with [`Input::Wasm`], so every request gets
//! exactly one response (FR-6). A resent last chunk is acknowledged with the
//! runtime status. Start / Stop / Status go straight to the runtime. Log
//! lines from the payload become `WasmLog` events.

use badge_proto::{BadgeMsg_, ErrorCode, Event_, HostMsg_::Cmd, WasmLog, WasmStatus};
use heapless::{String, Vec};
use middleware::loader::{Command, Effect as LoaderEffect, Loader};

use super::{App, Out};
use crate::board::PAYLOAD_MAX_IMAGE;
use crate::io::{Effect, WASM_LOG_LEN, WasmCmd};

impl App {
    /// Runs a payload request through the loader and forwards the outcome to
    /// the runtime.
    pub(super) fn wasm_host(&mut self, seq: u32, cmd: Cmd, out: &mut Out) {
        let command = match &cmd {
            Cmd::WasmBegin(b) => Command::Begin { len: b.len, crc: b.crc32 },
            Cmd::WasmChunk(c) => Command::Chunk { offset: c.offset, data: &c.data },
            Cmd::WasmCommit(_) => Command::Commit,
            Cmd::WasmStart(_) => {
                out.push(Effect::Wasm { seq, cmd: WasmCmd::Start });
                return;
            }
            Cmd::WasmStop(_) => {
                out.push(Effect::Wasm { seq, cmd: WasmCmd::Stop });
                return;
            }
            Cmd::GetWasm(_) => {
                out.push(Effect::Wasm { seq, cmd: WasmCmd::Status });
                return;
            }
            _ => {
                out.error(seq, ErrorCode::Unsupported, "not a wasm request");
                return;
            }
        };
        let (loader, step) = self.loader.step(&command, PAYLOAD_MAX_IMAGE);
        self.loader = loader;
        let cmd = match step {
            Ok(LoaderEffect::None) => WasmCmd::Status,
            Ok(LoaderEffect::Erase { len }) => WasmCmd::Erase { len },
            Ok(LoaderEffect::Write { offset, data }) => {
                let mut chunk = Vec::new();
                if chunk.extend_from_slice(data).is_err() {
                    self.loader = Loader::Idle;
                    out.error(seq, ErrorCode::BadArgument, "wasm chunk too long");
                    return;
                }
                WasmCmd::Write { offset, data: chunk }
            }
            Ok(LoaderEffect::Activate { len, crc }) => WasmCmd::Activate { len, crc },
            Err(_) => {
                out.error(seq, ErrorCode::BadArgument, "wasm upload rejected; restart with WasmBegin");
                return;
            }
        };
        out.push(Effect::Wasm { seq, cmd });
    }

    /// The runtime answered a payload request.
    pub(super) fn wasm_done(seq: u32, result: Result<WasmStatus, ErrorCode>, out: &mut Out) {
        match result {
            Ok(status) => out.respond(seq, badge_proto::Response_::Body::Wasm(status)),
            Err(code) => out.error(seq, code, "wasm"),
        }
    }

    /// Forwards a payload log line to the host.
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
        BadgeMsg, BadgeMsg_, Event_, GetWasm, HostMsg, Response_::Body, WasmBegin, WasmChunk,
        WasmCommit, WasmStart, WasmState,
    };
    use middleware::crc::Crc32;

    use super::*;
    use crate::app::BoardInfo;
    use crate::io::Input;

    fn app() -> App {
        App::new(BoardInfo { mac: [0; 6], firmware: "t" })
    }

    fn host(app: App, seq: u32, cmd: Cmd) -> (App, crate::io::Effects) {
        app.handle(Input::Host(HostMsg { seq, cmd: Some(cmd) }))
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

    fn wasm_cmd(fx: &[Effect]) -> Option<&WasmCmd> {
        fx.iter().find_map(|e| match e {
            Effect::Wasm { cmd, .. } => Some(cmd),
            _ => None,
        })
    }

    fn chunk(offset: u32, data: &[u8]) -> Cmd {
        let mut d = heapless::Vec::new();
        d.extend_from_slice(data).ok();
        Cmd::WasmChunk(WasmChunk { offset, data: d })
    }

    #[test]
    fn fr41_upload_walks_the_middleware_loader() {
        let image: std::vec::Vec<u8> = (0..300u32).map(|i| i as u8).collect();
        let crc = Crc32::checksum(&image);
        let len = image.len() as u32;
        let (a, fx) = host(app(), 1, Cmd::WasmBegin(WasmBegin { len, crc32: crc }));
        assert_eq!(wasm_cmd(&fx), Some(&WasmCmd::Erase { len }));
        let (a, fx) = host(a, 2, chunk(0, &image[..192]));
        assert!(matches!(wasm_cmd(&fx), Some(WasmCmd::Write { offset: 0, data }) if data.len() == 192));
        // Resending the last chunk is acknowledged with the status.
        let (a, fx) = host(a, 3, chunk(0, &image[..192]));
        assert_eq!(wasm_cmd(&fx), Some(&WasmCmd::Status));
        let (a, fx) = host(a, 4, chunk(192, &image[192..]));
        assert!(matches!(wasm_cmd(&fx), Some(WasmCmd::Write { offset: 192, .. })));
        let (_, fx) = host(a, 5, Cmd::WasmCommit(WasmCommit {}));
        assert_eq!(wasm_cmd(&fx), Some(&WasmCmd::Activate { len, crc }));
    }

    #[test]
    fn fr41_bad_uploads_are_rejected_without_reaching_the_runtime() {
        let image = [7u8; 64];
        let crc = Crc32::checksum(&image);
        // Too large for the RAM-bounded image limit.
        let (_, fx) = host(app(), 1, Cmd::WasmBegin(WasmBegin { len: PAYLOAD_MAX_IMAGE + 1, crc32: 0 }));
        assert_eq!(error_of(&fx), Some(ErrorCode::BadArgument));
        assert!(wasm_cmd(&fx).is_none());
        // Chunk without Begin; out-of-order chunk; wrong CRC at commit.
        let (_, fx) = host(app(), 2, chunk(0, &image));
        assert_eq!(error_of(&fx), Some(ErrorCode::BadArgument));
        let (a, _) = host(app(), 3, Cmd::WasmBegin(WasmBegin { len: 64, crc32: crc }));
        let (_, fx) = host(a.clone(), 4, chunk(32, &image[32..]));
        assert_eq!(error_of(&fx), Some(ErrorCode::BadArgument));
        let (a, _) = host(a, 5, chunk(0, &image));
        let (a, fx) = {
            let (a, _) = host(a, 6, Cmd::WasmBegin(WasmBegin { len: 64, crc32: crc ^ 1 }));
            let (a, _) = host(a, 7, chunk(0, &image));
            host(a, 8, Cmd::WasmCommit(WasmCommit {}))
        };
        assert_eq!(error_of(&fx), Some(ErrorCode::BadArgument));
        assert!(wasm_cmd(&fx).is_none());
        // After a rejection the transfer is reset: a chunk needs a new Begin.
        let (_, fx) = host(a, 9, chunk(0, &image));
        assert_eq!(error_of(&fx), Some(ErrorCode::BadArgument));
    }

    #[test]
    fn fr41_start_stop_status_go_to_the_runtime() {
        let (_, fx) = host(app(), 1, Cmd::WasmStart(WasmStart {}));
        assert_eq!(wasm_cmd(&fx), Some(&WasmCmd::Start));
        let (_, fx) = host(app(), 2, Cmd::GetWasm(GetWasm {}));
        assert_eq!(wasm_cmd(&fx), Some(&WasmCmd::Status));
    }

    #[test]
    fn fr41_runtime_answer_is_the_one_response_and_logs_become_events() {
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
        assert_eq!(responses[0].body, Some(Body::Wasm(status)));
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
