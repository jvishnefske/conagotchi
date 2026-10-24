//! Request/response state machine over an abstract [`Link`].
//!
//! The badge transport is three byte streams (defmt log up, protocol up,
//! protocol down) plus raw word access for the PROBE block. [`Link`] models
//! exactly that, so [`Conn`] — framing, sequence matching, event routing,
//! timeouts and the mailbox handshake — runs unchanged against the probe-rs
//! implementation (`crate::probe`) and against the in-memory fake used by the
//! tests below. Time comes from a [`Clock`], so timeouts are tested without
//! sleeping.

use std::collections::VecDeque;

use badge_core::frame::{self, FrameError};
use badge_core::probe_layout as layout;
use badge_proto::{BADGE_MSG_MAX, BadgeMsg, BadgeMsg_, Event, HOST_MSG_MAX, HostMsg, Response};

/// Largest COBS frame the badge can send.
pub const UP_FRAME_MAX: usize = frame::max_encoded_len(BADGE_MSG_MAX);
/// Largest COBS frame the host sends.
pub const DOWN_FRAME_MAX: usize = frame::max_encoded_len(HOST_MSG_MAX);
/// Idle delay between polls when no bytes arrived.
pub const POLL_IDLE_MS: u64 = 5;
/// Uptime advance that proves the firmware republished the PROBE block after
/// taking a mailbox command (two 20 ms ticks).
pub const MAILBOX_SETTLE_MS: u32 = 40;

/// Byte and word access to the badge.
pub trait Link {
    /// Transport error.
    type Error: std::error::Error + Send + Sync + 'static;
    /// Writes as many bytes as fit into the down channel; returns the count.
    fn write_down(&mut self, bytes: &[u8]) -> Result<usize, Self::Error>;
    /// Reads available protocol bytes from the up channel.
    fn read_up(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error>;
    /// Reads available defmt log bytes (0 if there is no log channel).
    fn read_log(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error>;
    /// Reads `out.len()` 32-bit words starting at `addr`.
    fn read_words(&mut self, addr: u64, out: &mut [u32]) -> Result<(), Self::Error>;
    /// Writes one 32-bit word at `addr`.
    fn write_word(&mut self, addr: u64, value: u32) -> Result<(), Self::Error>;
}

/// Monotonic time source.
pub trait Clock {
    /// Milliseconds since an arbitrary origin.
    fn now_ms(&self) -> u64;
    /// Waits about `ms` milliseconds.
    fn sleep_ms(&mut self, ms: u64);
}

/// Real-time [`Clock`].
#[derive(Debug, Clone, Copy)]
pub struct SystemClock {
    origin: std::time::Instant,
}

impl Default for SystemClock {
    fn default() -> Self {
        Self { origin: std::time::Instant::now() }
    }
}

impl Clock for SystemClock {
    fn now_ms(&self) -> u64 {
        u64::try_from(self.origin.elapsed().as_millis()).unwrap_or(u64::MAX)
    }

    fn sleep_ms(&mut self, ms: u64) {
        std::thread::sleep(std::time::Duration::from_millis(ms));
    }
}

/// A problem with one received frame; the stream continues.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum UpError {
    /// COBS framing failed (overflow or corruption).
    #[error("bad frame: {0:?}")]
    Frame(FrameError),
    /// The frame is not a valid `BadgeMsg`.
    #[error("undecodable BadgeMsg ({0} bytes)")]
    Decode(usize),
    /// A `BadgeMsg` without content.
    #[error("empty BadgeMsg")]
    Empty,
}

/// Receives everything that is not the awaited response.
pub trait Observer {
    /// Raw defmt bytes from the log channel.
    fn log_bytes(&mut self, bytes: &[u8]);
    /// An unsolicited event.
    fn event(&mut self, event: &Event);
    /// A response that does not answer the current request.
    fn stray_response(&mut self, response: &Response);
    /// A frame that could not be decoded.
    fn frame_error(&mut self, error: &UpError);
}

/// Session failures.
#[derive(Debug, thiserror::Error)]
pub enum SessionError<E: std::error::Error + Send + Sync + 'static> {
    /// The link failed.
    #[error("link: {0}")]
    Link(#[source] E),
    /// The request could not be encoded.
    #[error("cannot encode request: {0}")]
    Encode(String),
    /// The down channel did not accept the whole frame in time.
    #[error("timed out after {timeout_ms} ms writing request seq {seq} ({written}/{len} bytes sent)")]
    WriteTimeout {
        /// Request sequence number.
        seq: u32,
        /// Timeout used.
        timeout_ms: u64,
        /// Bytes accepted.
        written: usize,
        /// Frame length.
        len: usize,
    },
    /// No response with the request's sequence number arrived in time.
    #[error("timed out after {timeout_ms} ms waiting for the response to seq {seq}")]
    ResponseTimeout {
        /// Request sequence number.
        seq: u32,
        /// Timeout used.
        timeout_ms: u64,
    },
    /// The PROBE block magic/version does not match.
    #[error(
        "no valid PROBE block at {addr:#x} (magic {magic:#010x}, version {version}; expected version {}); \
         a version mismatch means old firmware: reflash it",
        layout::VERSION_VALUE
    )]
    BadMagic {
        /// Block address.
        addr: u64,
        /// Magic word read.
        magic: u32,
        /// Version word read.
        version: u32,
    },
    /// The firmware did not clear the mailbox command in time.
    #[error("firmware did not take mailbox command {cmd} within {timeout_ms} ms")]
    MailboxTimeout {
        /// Command written.
        cmd: u32,
        /// Timeout used.
        timeout_ms: u64,
    },
}

/// Encodes a host message as one COBS frame (with delimiter).
pub fn encode_frame(msg: &HostMsg) -> Result<Vec<u8>, String> {
    let mut pb = [0u8; HOST_MSG_MAX];
    let n = badge_proto::encode(msg, &mut pb).map_err(|e| format!("{e:?}"))?;
    let mut out = vec![0u8; DOWN_FRAME_MAX];
    let m = frame::encode(pb.get(..n).unwrap_or(&[]), &mut out).map_err(|e| format!("{e:?}"))?;
    out.truncate(m);
    Ok(out)
}

/// Streaming decoder for the protocol up channel.
#[derive(Debug, Clone, Default)]
pub struct UpStream {
    decoder: frame::Decoder<UP_FRAME_MAX>,
}

impl UpStream {
    /// Feeds received bytes; returns every message (or error) they complete.
    pub fn feed(&mut self, bytes: &[u8]) -> Vec<Result<BadgeMsg_::Kind, UpError>> {
        let mut out = Vec::new();
        for &b in bytes {
            match self.decoder.push(b) {
                None => {}
                Some(Err(e)) => out.push(Err(UpError::Frame(e))),
                Some(Ok(payload)) => out.push(
                    badge_proto::decode::<BadgeMsg>(payload)
                        .map_err(|_| UpError::Decode(payload.len()))
                        .and_then(|m| m.kind.ok_or(UpError::Empty)),
                ),
            }
        }
        out
    }
}

/// A connection to the badge over some [`Link`].
pub struct Conn<L, C> {
    link: L,
    clock: C,
    up: UpStream,
    pending: VecDeque<BadgeMsg_::Kind>,
}

impl<L: Link, C: Clock> Conn<L, C> {
    /// Wraps a link and a clock.
    pub fn new(link: L, clock: C) -> Self {
        Self { link, clock, up: UpStream::default(), pending: VecDeque::new() }
    }

    /// The underlying link (tests inspect the fake through it).
    #[cfg(test)]
    pub fn link_mut(&mut self) -> &mut L {
        &mut self.link
    }

    /// Reads both up channels once. Decoded messages are queued; returns
    /// whether any byte arrived.
    fn pump<O: Observer>(&mut self, obs: &mut O) -> Result<bool, SessionError<L::Error>> {
        let mut buf = [0u8; 1024];
        let n_log = self.link.read_log(&mut buf).map_err(SessionError::Link)?;
        if let Some(bytes) = buf.get(..n_log).filter(|b| !b.is_empty()) {
            obs.log_bytes(bytes);
        }
        let n_up = self.link.read_up(&mut buf).map_err(SessionError::Link)?;
        for item in self.up.feed(buf.get(..n_up).unwrap_or(&[])) {
            match item {
                Ok(kind) => self.pending.push_back(kind),
                Err(e) => obs.frame_error(&e),
            }
        }
        Ok(n_log + n_up > 0)
    }

    /// Routes queued messages; returns the response for `seq` if present.
    fn dispatch<O: Observer>(&mut self, seq: Option<u32>, obs: &mut O) -> Option<Response> {
        while let Some(kind) = self.pending.pop_front() {
            match kind {
                BadgeMsg_::Kind::Event(e) => obs.event(&e),
                BadgeMsg_::Kind::Response(r) if Some(r.seq) == seq => return Some(r),
                BadgeMsg_::Kind::Response(r) => obs.stray_response(&r),
            }
        }
        None
    }

    /// Sends `msg` and waits up to `timeout_ms` for its response.
    ///
    /// Logs, events and unrelated responses that arrive meanwhile go to `obs`.
    pub fn request<O: Observer>(
        &mut self,
        msg: &HostMsg,
        timeout_ms: u64,
        obs: &mut O,
    ) -> Result<Response, SessionError<L::Error>> {
        let frame = encode_frame(msg).map_err(SessionError::Encode)?;
        let deadline = self.clock.now_ms().saturating_add(timeout_ms);
        let mut written = 0usize;
        while written < frame.len() {
            let n = self
                .link
                .write_down(frame.get(written..).unwrap_or(&[]))
                .map_err(SessionError::Link)?;
            written += n;
            if n == 0 {
                if self.clock.now_ms() >= deadline {
                    return Err(SessionError::WriteTimeout {
                        seq: msg.seq,
                        timeout_ms,
                        written,
                        len: frame.len(),
                    });
                }
                self.pump(obs)?;
                self.dispatch(None, obs);
                self.clock.sleep_ms(POLL_IDLE_MS);
            }
        }
        loop {
            let active = self.pump(obs)?;
            if let Some(r) = self.dispatch(Some(msg.seq), obs) {
                return Ok(r);
            }
            if self.clock.now_ms() >= deadline {
                return Err(SessionError::ResponseTimeout { seq: msg.seq, timeout_ms });
            }
            if !active {
                self.clock.sleep_ms(POLL_IDLE_MS);
            }
        }
    }

    /// Streams logs, events and responses to `obs` until `stop()` is true.
    pub fn stream<O: Observer, F: FnMut() -> bool>(
        &mut self,
        obs: &mut O,
        mut stop: F,
    ) -> Result<(), SessionError<L::Error>> {
        while !stop() {
            let active = self.pump(obs)?;
            self.dispatch(None, obs);
            if !active {
                self.clock.sleep_ms(POLL_IDLE_MS);
            }
        }
        Ok(())
    }

    /// Reads the whole PROBE block at `base`.
    pub fn read_block(&mut self, base: u64) -> Result<[u32; layout::WORDS], SessionError<L::Error>> {
        let mut words = [0u32; layout::WORDS];
        self.link.read_words(base, &mut words).map_err(SessionError::Link)?;
        Ok(words)
    }

    /// Writes a mailbox command (arguments first, then the command word) and
    /// waits up to `timeout_ms` for the firmware to clear it.
    ///
    /// The block must carry the current layout version; a layout v2 block
    /// (older firmware) is accepted for the commands v2 already had
    /// (`SET_LED` through `SET_PET_MODE`).
    ///
    /// Returns the block as read after the command was taken.
    pub fn poke_mailbox(
        &mut self,
        base: u64,
        (cmd, arg0, arg1): (u32, u32, u32),
        timeout_ms: u64,
    ) -> Result<[u32; layout::WORDS], SessionError<L::Error>> {
        let before = self.read_block(base)?;
        let (magic, version) = (before[layout::MAGIC], before[layout::VERSION]);
        let version_ok = version == layout::VERSION_VALUE
            || (version == crate::peek::LEGACY_VERSION && cmd <= layout::mailbox::SET_PET_MODE);
        if magic != layout::MAGIC_VALUE || !version_ok {
            return Err(SessionError::BadMagic { addr: base, magic, version });
        }
        let word = |i: usize| base + 4 * i as u64;
        self.link.write_word(word(layout::MAILBOX_ARG0), arg0).map_err(SessionError::Link)?;
        self.link.write_word(word(layout::MAILBOX_ARG1), arg1).map_err(SessionError::Link)?;
        self.link.write_word(word(layout::MAILBOX_CMD), cmd).map_err(SessionError::Link)?;
        let deadline = self.clock.now_ms().saturating_add(timeout_ms);
        let taken_at = loop {
            let words = self.read_block(base)?;
            if words[layout::MAILBOX_CMD] == 0 {
                break words[layout::UPTIME_MS];
            }
            if self.clock.now_ms() >= deadline {
                return Err(SessionError::MailboxTimeout { cmd, timeout_ms });
            }
            self.clock.sleep_ms(POLL_IDLE_MS * 4);
        };
        // The firmware clears the command word when it takes the command but
        // only republishes the block on a later tick, so wait until the
        // uptime word shows a refresh made after the command was handled.
        loop {
            let words = self.read_block(base)?;
            let advanced = words[layout::UPTIME_MS].wrapping_sub(taken_at);
            if advanced >= MAILBOX_SETTLE_MS || self.clock.now_ms() >= deadline {
                return Ok(words);
            }
            self.clock.sleep_ms(POLL_IDLE_MS * 4);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use badge_proto::{
        Ack, Button, ButtonEvent, Event_, HostMsg_::Cmd, Ping, Pong, Response_::Body,
    };
    use std::cell::Cell;
    use std::rc::Rc;

    #[derive(Debug, thiserror::Error)]
    #[error("fake link failure")]
    struct FakeError;

    type Responder = Box<dyn FnMut(&HostMsg) -> Vec<BadgeMsg>>;

    /// In-memory badge: decodes down frames, answers via `responder`.
    struct FakeLink {
        down: frame::Decoder<DOWN_FRAME_MAX>,
        up: VecDeque<u8>,
        log: VecDeque<u8>,
        responder: Responder,
        received: Vec<HostMsg>,
        /// Max bytes accepted per write; every other write accepts 0.
        write_chunk: usize,
        writes: usize,
        stall_writes: bool,
        /// Max bytes delivered per up read.
        read_chunk: usize,
        mem_base: u64,
        mem: Vec<u32>,
        /// Reads of the block before the firmware clears the cmd word
        /// (`None` = never).
        clear_after: Option<usize>,
        block_reads: usize,
        /// Reads since the fake firmware took the mailbox command.
        since_clear: Option<usize>,
        fail: bool,
    }

    impl FakeLink {
        fn new(responder: Responder) -> Self {
            Self {
                down: frame::Decoder::new(),
                up: VecDeque::new(),
                log: VecDeque::new(),
                responder,
                received: Vec::new(),
                write_chunk: usize::MAX,
                writes: 0,
                stall_writes: false,
                read_chunk: usize::MAX,
                mem_base: 0x3fc8_0000,
                mem: Vec::new(),
                clear_after: Some(0),
                block_reads: 0,
                since_clear: None,
                fail: false,
            }
        }

        fn send(&mut self, msg: &BadgeMsg) {
            let mut pb = [0u8; BADGE_MSG_MAX];
            let n = badge_proto::encode(msg, &mut pb).unwrap();
            let mut wire = vec![0u8; UP_FRAME_MAX];
            let m = frame::encode(&pb[..n], &mut wire).unwrap();
            self.up.extend(&wire[..m]);
        }
    }

    impl Link for FakeLink {
        type Error = FakeError;

        fn write_down(&mut self, bytes: &[u8]) -> Result<usize, FakeError> {
            if self.fail {
                return Err(FakeError);
            }
            self.writes += 1;
            if self.stall_writes || (self.write_chunk != usize::MAX && self.writes.is_multiple_of(2)) {
                return Ok(0);
            }
            let n = bytes.len().min(self.write_chunk);
            for &b in &bytes[..n] {
                if let Some(Ok(payload)) = self.down.push(b) {
                    let msg: HostMsg = badge_proto::decode(payload).unwrap();
                    self.received.push(msg.clone());
                    for reply in (self.responder)(&msg) {
                        self.send(&reply);
                    }
                }
            }
            Ok(n)
        }

        fn read_up(&mut self, buf: &mut [u8]) -> Result<usize, FakeError> {
            let n = buf.len().min(self.up.len()).min(self.read_chunk);
            for slot in &mut buf[..n] {
                *slot = self.up.pop_front().unwrap();
            }
            Ok(n)
        }

        fn read_log(&mut self, buf: &mut [u8]) -> Result<usize, FakeError> {
            let n = buf.len().min(self.log.len());
            for slot in &mut buf[..n] {
                *slot = self.log.pop_front().unwrap();
            }
            Ok(n)
        }

        fn read_words(&mut self, addr: u64, out: &mut [u32]) -> Result<(), FakeError> {
            let start = usize::try_from((addr - self.mem_base) / 4).unwrap();
            // Model the firmware: it clears the command word when taking a
            // command, republishes the block two ticks later (here: two reads),
            // and advances uptime by one 20 ms tick per read.
            if self.mem.len() == layout::WORDS {
                self.mem[layout::UPTIME_MS] = self.mem[layout::UPTIME_MS].wrapping_add(20);
                if self.mem[layout::MAILBOX_CMD] == layout::mailbox::SHOW_SCREEN
                    && self.clear_after.is_some_and(|n| self.block_reads >= n)
                {
                    self.since_clear = Some(0);
                }
                if let Some(n) = self.since_clear {
                    if n >= 2 {
                        self.mem[layout::SCREEN] = self.mem[layout::MAILBOX_ARG0];
                    }
                    self.since_clear = Some(n + 1);
                }
            }
            if self.clear_after.is_some_and(|n| self.block_reads >= n) {
                self.mem[layout::MAILBOX_CMD] = 0;
            }
            self.block_reads += 1;
            out.copy_from_slice(&self.mem[start..start + out.len()]);
            Ok(())
        }

        fn write_word(&mut self, addr: u64, value: u32) -> Result<(), FakeError> {
            let idx = usize::try_from((addr - self.mem_base) / 4).unwrap();
            self.mem[idx] = value;
            self.block_reads = 0;
            Ok(())
        }
    }

    #[derive(Default)]
    struct FakeClock {
        now: Rc<Cell<u64>>,
    }

    impl Clock for FakeClock {
        fn now_ms(&self) -> u64 {
            self.now.get()
        }
        fn sleep_ms(&mut self, ms: u64) {
            self.now.set(self.now.get() + ms);
        }
    }

    #[derive(Default)]
    struct Recorder {
        logs: Vec<u8>,
        events: Vec<Event>,
        stray: Vec<Response>,
        errors: Vec<UpError>,
    }

    impl Observer for Recorder {
        fn log_bytes(&mut self, bytes: &[u8]) {
            self.logs.extend_from_slice(bytes);
        }
        fn event(&mut self, event: &Event) {
            self.events.push(event.clone());
        }
        fn stray_response(&mut self, response: &Response) {
            self.stray.push(response.clone());
        }
        fn frame_error(&mut self, error: &UpError) {
            self.errors.push(error.clone());
        }
    }

    fn response(seq: u32, body: Body) -> BadgeMsg {
        BadgeMsg { kind: Some(BadgeMsg_::Kind::Response(Response { seq, body: Some(body) })) }
    }

    fn button_event(at_ms: u64) -> BadgeMsg {
        BadgeMsg {
            kind: Some(BadgeMsg_::Kind::Event(Event {
                kind: Some(Event_::Kind::Button(ButtonEvent { button: Button::Start, pressed: true, at_ms })),
            })),
        }
    }

    fn pong_responder() -> Responder {
        Box::new(|msg: &HostMsg| match &msg.cmd {
            Some(Cmd::Ping(p)) => vec![
                button_event(1),
                response(msg.seq.wrapping_add(1000), Body::Ack(Ack {})),
                response(msg.seq, Body::Pong(Pong { nonce: p.nonce, uptime_ms: 42 })),
            ],
            _ => vec![response(msg.seq, Body::Ack(Ack {}))],
        })
    }

    fn ping(seq: u32) -> HostMsg {
        HostMsg { seq, cmd: Some(Cmd::Ping(Ping { nonce: 77 })) }
    }

    #[test]
    fn request_matches_seq_and_routes_the_rest() {
        let mut conn = Conn::new(FakeLink::new(pong_responder()), FakeClock::default());
        conn.link_mut().log.extend([1u8, 2, 3]);
        let mut rec = Recorder::default();
        let r = conn.request(&ping(5), 1000, &mut rec).unwrap();
        assert_eq!(r, Response { seq: 5, body: Some(Body::Pong(Pong { nonce: 77, uptime_ms: 42 })) });
        assert_eq!(rec.events.len(), 1);
        assert_eq!(rec.stray.len(), 1);
        assert_eq!(rec.stray[0].seq, 1005);
        assert_eq!(rec.logs, [1, 2, 3]);
        assert!(rec.errors.is_empty());
        assert_eq!(conn.link_mut().received, [ping(5)]);
    }

    #[test]
    fn partial_writes_and_reads_complete() {
        let mut link = FakeLink::new(pong_responder());
        link.write_chunk = 3;
        link.read_chunk = 2;
        let mut conn = Conn::new(link, FakeClock::default());
        let mut rec = Recorder::default();
        let r = conn.request(&ping(9), 5000, &mut rec).unwrap();
        assert_eq!(r.seq, 9);
        assert!(conn.link_mut().writes > 2);
    }

    #[test]
    fn response_after_late_event_and_garbage() {
        let responder: Responder = Box::new(|_| Vec::new());
        let mut conn = Conn::new(FakeLink::new(responder), FakeClock::default());
        // Stale bytes from a frame we joined halfway, then a bad frame.
        conn.link_mut().up.extend([0x41u8, 0x42, 0x00, 0x05, 0x01, 0x00]);
        conn.link_mut().send(&button_event(3));
        conn.link_mut().send(&response(11, Body::Ack(Ack {})));
        let mut rec = Recorder::default();
        let r = conn.request(&ping(11), 1000, &mut rec).unwrap();
        assert_eq!(r.body, Some(Body::Ack(Ack {})));
        assert_eq!(rec.events.len(), 1);
        assert_eq!(rec.errors.len(), 2, "{:?}", rec.errors);
    }

    #[test]
    fn response_timeout() {
        let responder: Responder = Box::new(|_| Vec::new());
        let clock = FakeClock::default();
        let now = clock.now.clone();
        let mut conn = Conn::new(FakeLink::new(responder), clock);
        let err = conn.request(&ping(3), 250, &mut Recorder::default()).unwrap_err();
        assert!(matches!(err, SessionError::ResponseTimeout { seq: 3, timeout_ms: 250 }));
        assert!(now.get() >= 250);
    }

    #[test]
    fn write_stall_times_out() {
        let mut link = FakeLink::new(pong_responder());
        link.stall_writes = true;
        let mut conn = Conn::new(link, FakeClock::default());
        let err = conn.request(&ping(3), 100, &mut Recorder::default()).unwrap_err();
        assert!(matches!(err, SessionError::WriteTimeout { written: 0, .. }), "{err}");
    }

    #[test]
    fn link_errors_propagate() {
        let mut link = FakeLink::new(pong_responder());
        link.fail = true;
        let mut conn = Conn::new(link, FakeClock::default());
        assert!(matches!(
            conn.request(&ping(1), 100, &mut Recorder::default()),
            Err(SessionError::Link(FakeError))
        ));
    }

    #[test]
    fn stream_routes_until_stopped() {
        let responder: Responder = Box::new(|_| Vec::new());
        let mut conn = Conn::new(FakeLink::new(responder), FakeClock::default());
        conn.link_mut().send(&button_event(1));
        conn.link_mut().send(&response(layout::MAILBOX_SEQ, Body::Ack(Ack {})));
        conn.link_mut().log.extend([9u8]);
        let mut rec = Recorder::default();
        let mut polls = 0;
        conn.stream(&mut rec, || {
            polls += 1;
            polls > 5
        })
        .unwrap();
        assert_eq!(rec.events.len(), 1);
        assert_eq!(rec.stray[0].seq, layout::MAILBOX_SEQ);
        assert_eq!(rec.logs, [9]);
    }

    fn block_link(clear_after: Option<usize>) -> FakeLink {
        let responder: Responder = Box::new(|_| Vec::new());
        let mut link = FakeLink::new(responder);
        link.mem = vec![0; layout::WORDS];
        link.mem[layout::MAGIC] = layout::MAGIC_VALUE;
        link.mem[layout::VERSION] = layout::VERSION_VALUE;
        link.clear_after = clear_after;
        link
    }

    #[test]
    fn mailbox_poke_writes_args_then_waits_for_clear() {
        let mut conn = Conn::new(block_link(Some(3)), FakeClock::default());
        let base = conn.link_mut().mem_base;
        let words = conn.poke_mailbox(base, (layout::mailbox::SET_LED, 4, 0x00ff00), 1000).unwrap();
        assert_eq!(words[layout::MAILBOX_CMD], 0);
        assert_eq!(words[layout::MAILBOX_ARG0], 4);
        assert_eq!(words[layout::MAILBOX_ARG1], 0x00ff00);
    }

    #[test]
    fn mailbox_poke_returns_block_published_after_the_command() {
        // Regression: poke used to print the block read at the moment the
        // command word cleared, one tick before the firmware republished it.
        let mut conn = Conn::new(block_link(Some(1)), FakeClock::default());
        let base = conn.link_mut().mem_base;
        let words = conn.poke_mailbox(base, (layout::mailbox::SHOW_SCREEN, 2, 0), 1000).unwrap();
        assert_eq!(words[layout::MAILBOX_CMD], 0);
        assert_eq!(words[layout::SCREEN], 2, "block must reflect the handled command");
    }

    #[test]
    fn mailbox_poke_times_out_and_checks_magic() {
        let mut conn = Conn::new(block_link(None), FakeClock::default());
        let base = conn.link_mut().mem_base;
        let err = conn.poke_mailbox(base, (2, 1, 0), 100).unwrap_err();
        assert!(matches!(err, SessionError::MailboxTimeout { cmd: 2, .. }));

        let mut link = block_link(Some(0));
        link.mem[layout::MAGIC] = 0xdead_beef;
        let mut conn = Conn::new(link, FakeClock::default());
        let err = conn.poke_mailbox(base, (2, 1, 0), 100).unwrap_err();
        assert!(matches!(err, SessionError::BadMagic { magic: 0xdead_beef, .. }));
        assert_eq!(conn.link_mut().mem[layout::MAILBOX_CMD], 0, "must not write on bad magic");
    }

    #[test]
    fn mailbox_poke_on_v2_block_allows_only_v2_commands() {
        let mut link = block_link(Some(0));
        link.mem[layout::VERSION] = crate::peek::LEGACY_VERSION;
        let mut conn = Conn::new(link, FakeClock::default());
        let base = conn.link_mut().mem_base;
        assert!(conn.poke_mailbox(base, (layout::mailbox::SET_PET_MODE, 1, 0), 1000).is_ok());
        let err = conn.poke_mailbox(base, (layout::mailbox::TRADE_INVITE, 0x0A_C8F6, 0), 1000).unwrap_err();
        assert!(matches!(err, SessionError::BadMagic { version: 2, .. }), "{err}");
        let err = conn.poke_mailbox(base, (layout::mailbox::SET_CHI, 1, 0), 1000).unwrap_err();
        assert!(matches!(err, SessionError::BadMagic { .. }), "{err}");
    }

    #[test]
    fn encode_frame_is_delimited_cobs() {
        let f = encode_frame(&ping(1)).unwrap();
        assert_eq!(f.last(), Some(&0));
        assert!(!f[..f.len() - 1].contains(&0));
        let mut up = UpStream::default();
        // A HostMsg is not a BadgeMsg, but the framing must still complete.
        assert_eq!(up.feed(&f).len(), 1);
    }
}
