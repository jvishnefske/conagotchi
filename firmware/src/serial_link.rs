//! The USB serial loader link (FR-44): middleware's serial protocol on the
//! USB-Serial-JTAG CDC port, so the middleware web app (Web Serial) and
//! `mwctl` install and run payloads exactly as on an esp-base board. RTT
//! keeps working next to it: probe-rs uses the JTAG side of the same USB
//! device, and nothing else in the firmware claims `USB_DEVICE` (defmt goes
//! to RTT, panics to `panic-rtt-target`).
//!
//! # Wire format
//!
//! Frames are middleware's [`frame`] layout (request, response, publish;
//! CRC-16, COBS, `0x00` delimiter), cut from the byte stream by
//! [`frame::Splitter`] and encoded with [`frame::encode`], so the bytes are
//! the same as esp-base's `SerialLink`. The link task does not use
//! `SerialLink` itself: its buffers are sized for any frame
//! (`MAX_RAW_FRAME`, about 3.2 KB in all), while the badge's frames are
//! bounded (a 512-byte loader chunk in, an 80-byte `DeviceInfo` or log line
//! out), and every byte of static RAM comes out of the shared main stack.
//! The buffers here are [`RX_WIRE`] bytes in and [`TX_WIRE`] out. A received
//! run longer than [`RX_WIRE`] is dropped (counted) like any other invalid
//! run; the host times out. Every frame sent is preceded by one `0x00`, so a
//! frame cut short by a send timeout is closed off as an invalid run instead
//! of being glued to the next frame.
//!
//! # Requests
//!
//! A request is mapped with [`badge_mw::serial::route_request`]. A refused
//! request is answered at once. Otherwise the command goes to the core as
//! `Input::Host(HostMsg { seq, cmd })` through [`REQUESTS`] (drained by the
//! shell with [`try_next`]), with `seq` taken from
//! `probe_layout::SERIAL_SEQ` in turn. The shell routes the core's
//! `Response` for a seq in that range back here through [`answer`] instead
//! of RTT, and the link answers with [`badge_mw::serial::reply_answer`]
//! (`sys/info` carries `middleware::wasm_abi::ABI_VERSION` and the firmware
//! name). One request is outstanding at a time: while it waits, no input is
//! read (the host's next frames wait in the USB FIFO), so the loader stays
//! sequential like the web app. If the core has not answered within
//! [`REQUEST_TIMEOUT_MS`], the request is answered `Internal`; a late answer
//! is then discarded by its seq.
//!
//! # Publishes
//!
//! From the host: a publish on an `app/...` key
//! ([`badge_mw::serial::route_publish`]) goes to [`mw::APP_SAMPLES`] for the
//! payload (at most `mw::KEY_LEN` / `mw::PAYLOAD_LEN` bytes); other keys,
//! oversized samples and a full queue drop it (counted).
//!
//! To the host: payload log lines ([`log`], tapped by the shell from the
//! core's `WasmLog` events) go out on `sys/log`, and payload state changes
//! ([`status`], from the wasm task) as the one-byte `sys/payload/status`.
//! Both queue in [`OUTBOUND`] without waiting; a full queue drops (counted),
//! so the core loop and the wasm task never block on the host.
//!
//! # Sends
//!
//! Each send is bounded by [`SEND_TIMEOUT_MS`]: USB-Serial-JTAG transmits
//! only while a host reads the port, and with nobody listening the frame is
//! dropped instead of stalling the link task.
//!
//! The link state ([`Link`], about 0.8 KB) lives in a `StaticCell`; the task
//! future holds only a reference to it and the outstanding request.

use core::sync::atomic::{AtomicU32, Ordering};

use badge_core::io::Input;
use badge_core::probe_layout::SERIAL_SEQ;
use badge_mw::serial::{self, CoreAnswer, Identity, Pending};
use badge_proto::HostMsg;
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_io_async::{Read, Write};
use esp_hal::Async;
use esp_hal::peripherals::USB_DEVICE;
use esp_hal::usb::usb_serial_jtag::{UsbSerialJtag, UsbSerialJtagRx, UsbSerialJtagTx};
use heapless::{String, Vec};
use middleware::Key;
use middleware::loader::PayloadStatus;
use middleware::serial::cobs;
use middleware::serial::frame::{self, Frame, Split, Splitter};
use middleware::transport::HandlerError;
use static_cell::StaticCell;

use crate::mw::{self, MwMessage};

/// Longest raw (pre-COBS) frame received: a `sys/loader/chunk` request
/// with a 512-byte chunk and a key of up to 64 bytes, plus headers and CRC.
pub const RX_RAW: usize = 3 + 64 + 4 + badge_core::io::WASM_CHUNK + 2;
/// Receive buffer: [`RX_RAW`] after COBS.
pub const RX_WIRE: usize = cobs::max_encoded_len(RX_RAW);
/// Longest raw frame sent: a `sys/log` publish of a 64-byte line, or a
/// `DeviceInfo` response (under 80 bytes).
pub const TX_RAW: usize = 128;
/// Transmit buffer: the leading `0x00`, [`TX_RAW`] after COBS, delimiter.
pub const TX_WIRE: usize = 1 + cobs::max_encoded_len(TX_RAW) + 1;
/// Largest response body (a `DeviceInfo`).
const BODY_MAX: usize = 96;
/// Bytes pulled from the USB FIFO per read.
const RX_CHUNK: usize = 64;
/// How long the link waits for the core to answer a request.
pub const REQUEST_TIMEOUT_MS: u64 = 3_000;
/// Longest a single frame write may take before it is abandoned.
pub const SEND_TIMEOUT_MS: u64 = 200;
/// Interval between drop reports.
const REPORT_MS: u64 = 5_000;

/// One message for the host.
enum Outbound {
    /// A `sys/log` line.
    Log(String<{ badge_core::io::WASM_LOG_LEN }>),
    /// A `sys/payload/status` change.
    Status(PayloadStatus),
}

/// Serial requests for the core (one outstanding at a time).
static REQUESTS: Channel<CriticalSectionRawMutex, HostMsg, 1> = Channel::new();
/// The core's answers to serial requests, from the shell.
static ANSWERS: Channel<CriticalSectionRawMutex, (u32, CoreAnswer), 2> = Channel::new();
/// Publishes waiting for the host.
static OUTBOUND: Channel<CriticalSectionRawMutex, Outbound, 4> = Channel::new();
/// Messages dropped: full queues, invalid or refused frames, failed sends.
static DROPPED: AtomicU32 = AtomicU32::new(0);

static LINK: StaticCell<Link> = StaticCell::new();

/// Why [`start`] failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum SerialError {
    /// The link task is already running.
    Spawn,
}

/// Takes the USB-Serial-JTAG peripheral and spawns the link task.
pub fn start(spawner: &Spawner, usb: USB_DEVICE<'static>) -> Result<(), SerialError> {
    let (rx, tx) = UsbSerialJtag::new(usb).into_async().split();
    let link = LINK.init(Link {
        rx: RxSide { usb: rx, splitter: Splitter::new(), chunk: [0; RX_CHUNK], pos: 0, len: 0 },
        tx: TxSide { usb: tx, scratch: [0; TX_RAW], wire: [0; TX_WIRE] },
    });
    spawner.spawn(link_task(link, crate::FIRMWARE).map_err(|_| SerialError::Spawn)?);
    Ok(())
}

/// The next serial request for the core, if any.
pub fn try_next() -> Option<Input> {
    REQUESTS.try_receive().ok().map(Input::Host)
}

/// The core answered serial request `seq` (called by the shell instead of
/// sending the response over RTT). Dropped if the queue is full; the
/// request then times out.
pub fn answer(seq: u32, answer: CoreAnswer) {
    if ANSWERS.try_send((seq, answer)).is_err() {
        dropped();
    }
}

/// Offers a payload log line to the host (`sys/log`).
pub fn log(text: &str) {
    offer(Outbound::Log(badge_mw::payload::truncate(text)));
}

/// Offers a payload state change to the host (`sys/payload/status`).
pub fn status(status: PayloadStatus) {
    offer(Outbound::Status(status));
}

fn offer(msg: Outbound) {
    if OUTBOUND.try_send(msg).is_err() {
        dropped();
    }
}

fn dropped() {
    DROPPED.fetch_add(1, Ordering::Relaxed);
}

/// The seq after `seq`, wrapping within `SERIAL_SEQ`.
const fn next_seq(seq: u32) -> u32 {
    if seq.wrapping_add(1) >= SERIAL_SEQ.end || seq < SERIAL_SEQ.start {
        SERIAL_SEQ.start
    } else {
        seq + 1
    }
}

/// The link's buffers and the two halves of the port.
struct Link {
    rx: RxSide,
    tx: TxSide,
}

/// Receive state: all of it lives here, so a read cancelled by `select`
/// loses nothing (esp-hal's reads drain the FIFO synchronously and only
/// await while it is empty).
struct RxSide {
    usb: UsbSerialJtagRx<'static, Async>,
    splitter: Splitter<RX_WIRE>,
    chunk: [u8; RX_CHUNK],
    pos: usize,
    len: usize,
}

impl RxSide {
    /// Feeds buffered bytes to the splitter until a run is complete
    /// (`true`, left in the splitter) or the chunk is used up. Bounded by
    /// [`RX_CHUNK`].
    fn pump(&mut self) -> bool {
        while self.pos < self.len {
            let Some(&byte) = self.chunk.get(self.pos) else { break };
            self.pos += 1;
            match self.splitter.push(byte) {
                Split::Pending => {}
                Split::Overflow => dropped(),
                Split::Frame(_) => return true,
            }
        }
        false
    }

    /// Waits for the next delimited run, left in the splitter.
    async fn next_run(&mut self) {
        loop {
            if self.pump() {
                return;
            }
            // The error type is `Infallible`.
            let Ok(n) = self.usb.read(&mut self.chunk).await;
            self.pos = 0;
            self.len = n.min(RX_CHUNK);
        }
    }
}

/// Transmit buffers.
struct TxSide {
    usb: UsbSerialJtagTx<'static, Async>,
    scratch: [u8; TX_RAW],
    wire: [u8; TX_WIRE],
}

impl TxSide {
    /// Encodes and writes one frame, bounded by [`SEND_TIMEOUT_MS`];
    /// failures drop it (the host retries requests).
    async fn send(&mut self, frame: &Frame<'_>) {
        let Some(out) = self.wire.get_mut(1..) else { return };
        let Ok(n) = frame::encode(frame, &mut self.scratch, out) else {
            dropped();
            return;
        };
        self.wire[0] = 0;
        let Some(bytes) = self.wire.get(..=n) else { return };
        let usb = &mut self.usb;
        let sent = with_timeout(Duration::from_millis(SEND_TIMEOUT_MS), async {
            let Ok(()) = usb.write_all(bytes).await;
            let Ok(()) = usb.flush().await;
        })
        .await;
        if sent.is_err() {
            dropped();
        }
    }

    async fn respond(&mut self, id: u8, result: Result<&[u8], HandlerError>) {
        self.send(&Frame::Response { id, result }).await;
    }
}

/// The request waiting for the core.
#[derive(Clone, Copy)]
struct Waiting {
    /// Host request id.
    id: u8,
    /// Core seq.
    seq: u32,
    /// How to answer.
    pending: Pending,
    /// When to give up.
    deadline: Instant,
}

/// Decoded input that needs a response or a delivery.
enum Received {
    /// Nothing to do (invalid run, host response, delivered publish).
    Nothing,
    /// A request refused before reaching the core.
    Refused { id: u8, error: HandlerError },
    /// A request now waiting for the core.
    Sent(Waiting),
}

/// Handles the run in the splitter. `seq` is the seq for a routed request.
fn receive(rx: &mut RxSide, seq: u32) -> Received {
    let Ok(frame) = frame::decode_wire(rx.splitter.frame()) else {
        dropped();
        return Received::Nothing;
    };
    match frame {
        Frame::Request { id, key, payload } => match serial::route_request(key, payload) {
            Err(error) => Received::Refused { id, error },
            Ok(req) => {
                if REQUESTS.try_send(HostMsg { seq, cmd: Some(req.cmd) }).is_err() {
                    return Received::Refused { id, error: HandlerError::Internal };
                }
                let deadline = Instant::now() + Duration::from_millis(REQUEST_TIMEOUT_MS);
                Received::Sent(Waiting { id, seq, pending: req.pending, deadline })
            }
        },
        Frame::Publish { key, payload } => {
            if serial::route_publish(key).is_err() || !deliver(key, payload) {
                dropped();
            }
            Received::Nothing
        }
        Frame::Response { .. } => Received::Nothing,
    }
}

/// Offers a host publish to the payload; false if it does not fit or the
/// queue is full.
fn deliver(key: Key<'_>, payload: &[u8]) -> bool {
    let mut msg = MwMessage { key: String::new(), payload: Vec::new() };
    msg.key.push_str(key.as_str()).is_ok()
        && msg.payload.extend_from_slice(payload).is_ok()
        && mw::APP_SAMPLES.try_send(msg).is_ok()
}

/// Sends one outbound publish.
async fn publish(tx: &mut TxSide, msg: Outbound) {
    match msg {
        Outbound::Log(text) => {
            tx.send(&Frame::Publish { key: serial::LOG, payload: text.as_bytes() }).await;
        }
        Outbound::Status(s) => tx.send(&Frame::Publish { key: serial::STATUS, payload: &[s as u8] }).await,
    }
}

#[embassy_executor::task]
async fn link_task(link: &'static mut Link, firmware: &'static str) {
    let id = Identity { abi_version: middleware::wasm_abi::ABI_VERSION, firmware };
    let Link { rx, tx } = link;
    let mut seq = SERIAL_SEQ.start;
    let mut waiting: Option<Waiting> = None;
    let mut next_report = Instant::now();
    let mut reported = 0u32;
    info!("serial loader link on USB-Serial-JTAG");
    loop {
        match waiting {
            Some(w) => match select3(ANSWERS.receive(), OUTBOUND.receive(), Timer::at(w.deadline)).await {
                Either3::First((got, answer)) if got == w.seq => {
                    let mut body = [0u8; BODY_MAX];
                    let result = serial::reply_answer(w.pending, &answer, id, &mut body)
                        .and_then(|n| body.get(..n).ok_or(HandlerError::BufferTooSmall));
                    tx.respond(w.id, result).await;
                    waiting = None;
                }
                // A late answer to a request that already timed out.
                Either3::First(_) => {}
                Either3::Second(msg) => publish(tx, msg).await,
                Either3::Third(()) => {
                    warn!("serial: core did not answer seq {=u32:#x}", w.seq);
                    tx.respond(w.id, Err(HandlerError::Internal)).await;
                    waiting = None;
                }
            },
            None => match select3(rx.next_run(), ANSWERS.receive(), OUTBOUND.receive()).await {
                Either3::First(()) => match receive(rx, seq) {
                    Received::Nothing => {}
                    Received::Refused { id, error } => tx.respond(id, Err(error)).await,
                    Received::Sent(w) => {
                        seq = next_seq(seq);
                        waiting = Some(w);
                    }
                },
                Either3::Second(_) => {}
                Either3::Third(msg) => publish(tx, msg).await,
            },
        }
        if Instant::now() >= next_report {
            next_report = Instant::now() + Duration::from_millis(REPORT_MS);
            let now = DROPPED.load(Ordering::Relaxed);
            if now != reported {
                warn!("serial: {} messages dropped", now);
                reported = now;
            }
        }
    }
}
