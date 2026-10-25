//! The `badge` imports and the per-instance state they act on.
//!
//! An instance's [`State`] lives in the wasmi `Store`. Imports never call
//! the embedder's [`Host`](crate::Host) directly: the host is borrowed only
//! for the duration of one `init` / `tick`, while the store lives as long as
//! the app. Instead every effect is queued in the bounded [`Outbox`] and
//! carried out by [`Outbox::flush`] when the call returns (also after a
//! trap, so the logs leading up to it are kept). `now_ms` returns the time
//! sampled when the call started.
//!
//! Every pointer/length pair is checked against linear memory
//! ([`Status::OutOfBounds`]); keys must satisfy
//! [`wasm_abi::app_key_is_valid`] ([`Status::BadKey`]).

use heapless::{Deque, String, Vec};
use wasm_abi::{
    IMPORT_MODULE, LED_COUNT, MAX_KEY, MAX_LOG, MAX_LOG_PER_CALL, MAX_PAYLOAD,
    MAX_PUBLISH_PER_CALL, MAX_SUBSCRIPTIONS, MAX_TEXT, PET_MODES, RECV_QUEUE, Status, imports,
};
use wasmi::{Caller, Error, Linker, Memory, StoreLimits};

use crate::Host;

/// One message on an app key.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Sample {
    /// The key (`app/...`).
    pub key: String<MAX_KEY>,
    /// The payload.
    pub payload: Vec<u8, MAX_PAYLOAD>,
}

impl Sample {
    /// A sample from raw parts; `None` if the key is not a valid app key or
    /// the payload is too long.
    pub(crate) fn new(key: &[u8], payload: &[u8]) -> Option<Sample> {
        if !wasm_abi::app_key_is_valid(key) {
            return None;
        }
        let key = core::str::from_utf8(key).ok()?;
        let mut out = Sample {
            key: String::new(),
            payload: Vec::new(),
        };
        out.key.push_str(key).ok()?;
        out.payload.extend_from_slice(payload).ok()?;
        Some(out)
    }
}

/// Effects queued by one call, carried out by [`Outbox::flush`].
#[derive(Debug, Default)]
pub(crate) struct Outbox {
    logs: Vec<String<MAX_LOG>, MAX_LOG_PER_CALL>,
    publishes: Vec<Sample, MAX_PUBLISH_PER_CALL>,
    /// Last `set_leds` of the call (later calls overwrite earlier ones).
    leds: Option<[u8; 3 * LED_COUNT]>,
    /// Last `show_text` of the call.
    text: Option<String<MAX_TEXT>>,
    /// Last `set_pet` of the call.
    pet: Option<i32>,
}

impl Outbox {
    /// Hands everything queued to the host, in the order logs, publishes,
    /// LEDs, text, pet, and empties the outbox. Host failures are counted in
    /// the return value (the app already returned).
    pub(crate) fn flush<H: Host>(&mut self, host: &mut H) -> u32 {
        let mut failures = 0u32;
        let mut count =
            |r: Result<(), Status>| failures = failures.saturating_add(u32::from(r.is_err()));
        for line in core::mem::take(&mut self.logs) {
            host.log(&line);
        }
        for sample in core::mem::take(&mut self.publishes) {
            count(host.publish(&sample.key, &sample.payload));
        }
        if let Some(rgb) = self.leds.take() {
            count(host.set_leds(&rgb));
        }
        if let Some(text) = self.text.take() {
            count(host.show_text(&text));
        }
        if let Some(mode) = self.pet.take() {
            count(host.set_pet(mode));
        }
        failures
    }
}

/// Per-instance state (the wasmi store data).
#[derive(Debug)]
pub(crate) struct State {
    /// Memory and instance limits enforced by wasmi.
    pub limits: StoreLimits,
    /// The guest's exported memory, set right after instantiation.
    pub memory: Option<Memory>,
    /// Host time sampled when the current call started.
    pub now_ms: i64,
    /// Keys the app subscribed to.
    pub subscriptions: Vec<String<MAX_KEY>, MAX_SUBSCRIPTIONS>,
    /// Samples waiting for `recv` (oldest first).
    pub queue: Deque<Sample, RECV_QUEUE>,
    /// Effects of the current call.
    pub outbox: Outbox,
}

impl State {
    /// Fresh state for a new instance.
    pub(crate) fn new(limits: StoreLimits) -> State {
        State {
            limits,
            memory: None,
            now_ms: 0,
            subscriptions: Vec::new(),
            queue: Deque::new(),
            outbox: Outbox::default(),
        }
    }

    /// True if the app subscribed to `key`.
    pub(crate) fn is_subscribed(&self, key: &str) -> bool {
        self.subscriptions.iter().any(|k| k == key)
    }

    /// Queues a sample for `recv`, dropping the oldest when full.
    pub(crate) fn enqueue(&mut self, sample: Sample) {
        if self.queue.is_full() {
            self.queue.pop_front();
        }
        // Cannot fail: a slot was just freed if needed.
        let _ = self.queue.push_back(sample);
    }
}

/// The byte range `ptr..ptr + len` of a memory of `size` bytes.
fn span(size: usize, ptr: i32, len: i32) -> Result<core::ops::Range<usize>, Status> {
    let start = usize::try_from(ptr).map_err(|_| Status::OutOfBounds)?;
    let len = usize::try_from(len).map_err(|_| Status::OutOfBounds)?;
    let end = start.checked_add(len).ok_or(Status::OutOfBounds)?;
    if end > size {
        return Err(Status::OutOfBounds);
    }
    Ok(start..end)
}

/// The guest bytes at `ptr..ptr + len`.
fn bytes<'a>(caller: &'a Caller<'_, State>, ptr: i32, len: i32) -> Result<&'a [u8], Status> {
    let memory = caller.data().memory.ok_or(Status::Unavailable)?;
    let data = memory.data(caller);
    data.get(span(data.len(), ptr, len)?)
        .ok_or(Status::OutOfBounds)
}

/// Converts a result into the import's `i32` return value.
fn code(result: Result<(), Status>) -> i32 {
    match result {
        Ok(()) => 0,
        Err(status) => status.code(),
    }
}

/// Reads and checks an app key argument.
fn key_arg(caller: &Caller<'_, State>, ptr: i32, len: i32) -> Result<String<MAX_KEY>, Status> {
    let raw = bytes(caller, ptr, len)?;
    if !wasm_abi::app_key_is_valid(raw) {
        return Err(Status::BadKey);
    }
    let text = core::str::from_utf8(raw).map_err(|_| Status::BadKey)?;
    let mut key = String::new();
    key.push_str(text).map_err(|_| Status::BadKey)?;
    Ok(key)
}

/// A log line from guest bytes: invalid UTF-8 becomes U+FFFD, the line is
/// cut to [`MAX_LOG`] bytes on a char boundary. Only the first `MAX_LOG`
/// bytes are examined, so the work is bounded whatever `len` is.
pub(crate) fn log_line(raw: &[u8]) -> String<MAX_LOG> {
    let truncated = raw.len() > MAX_LOG;
    let window = raw.get(..MAX_LOG).unwrap_or(raw);
    let mut line = String::new();
    let mut chunks = window.utf8_chunks().peekable();
    while let Some(chunk) = chunks.next() {
        for c in chunk.valid().chars() {
            if line.push(c).is_err() {
                return line;
            }
        }
        // A char cut by the window is not an encoding error.
        let cut = truncated && chunks.peek().is_none();
        if !chunk.invalid().is_empty() && !cut && line.push(char::REPLACEMENT_CHARACTER).is_err() {
            return line;
        }
    }
    line
}

fn log(caller: Caller<'_, State>, ptr: i32, len: i32) {
    let mut caller = caller;
    // A bad pointer in `log` is ignored: there is no return value to report it.
    let Ok(raw) = bytes(&caller, ptr, len) else {
        return;
    };
    let line = log_line(raw);
    // Lines beyond MAX_LOG_PER_CALL are dropped.
    let _ = caller.data_mut().outbox.logs.push(line);
}

fn publish(caller: Caller<'_, State>, key_ptr: i32, key_len: i32, ptr: i32, len: i32) -> i32 {
    let mut caller = caller;
    let result = (|| {
        let key = key_arg(&caller, key_ptr, key_len)?;
        let data = bytes(&caller, ptr, len)?;
        let mut payload = Vec::new();
        payload
            .extend_from_slice(data)
            .map_err(|_| Status::TooLong)?;
        let sample = Sample { key, payload };
        caller
            .data_mut()
            .outbox
            .publishes
            .push(sample)
            .map_err(|_| Status::Full)
    })();
    code(result)
}

fn subscribe(caller: Caller<'_, State>, ptr: i32, len: i32) -> i32 {
    let mut caller = caller;
    let result = (|| {
        let key = key_arg(&caller, ptr, len)?;
        let state = caller.data_mut();
        if state.is_subscribed(&key) {
            return Ok(());
        }
        state.subscriptions.push(key).map_err(|_| Status::Full)
    })();
    code(result)
}

fn recv(caller: Caller<'_, State>, ptr: i32, cap: i32) -> i32 {
    let mut caller = caller;
    let result = (|| {
        let memory = caller.data().memory.ok_or(Status::Unavailable)?;
        let (data, state) = memory.data_and_store_mut(&mut caller);
        let range = span(data.len(), ptr, cap)?;
        let buf = data.get_mut(range).ok_or(Status::OutOfBounds)?;
        let Some(sample) = state.queue.front() else {
            return Ok(0);
        };
        // BufferTooSmall leaves the sample queued.
        let n = wasm_abi::encode_sample(sample.key.as_bytes(), &sample.payload, buf)?;
        state.queue.pop_front();
        i32::try_from(n).map_err(|_| Status::TooLong)
    })();
    result.unwrap_or_else(Status::code)
}

fn set_leds(caller: Caller<'_, State>, ptr: i32) -> i32 {
    let mut caller = caller;
    let result = (|| {
        let raw = bytes(&caller, ptr, 3 * LED_COUNT as i32)?;
        let rgb: [u8; 3 * LED_COUNT] = raw.try_into().map_err(|_| Status::OutOfBounds)?;
        caller.data_mut().outbox.leds = Some(rgb);
        Ok(())
    })();
    code(result)
}

fn show_text(caller: Caller<'_, State>, ptr: i32, len: i32) -> i32 {
    let mut caller = caller;
    let result = (|| {
        let raw = bytes(&caller, ptr, len)?;
        if raw.len() > MAX_TEXT {
            return Err(Status::TooLong);
        }
        let text = core::str::from_utf8(raw).map_err(|_| Status::BadArgument)?;
        let mut owned = String::new();
        owned.push_str(text).map_err(|_| Status::TooLong)?;
        caller.data_mut().outbox.text = Some(owned);
        Ok(())
    })();
    code(result)
}

fn set_pet(caller: Caller<'_, State>, mode: i32) -> i32 {
    let mut caller = caller;
    if !(0..PET_MODES).contains(&mode) {
        return Status::BadArgument.code();
    }
    caller.data_mut().outbox.pet = Some(mode);
    0
}

fn now_ms(caller: Caller<'_, State>) -> i64 {
    caller.data().now_ms
}

/// Defines every `badge` import in `linker`.
pub(crate) fn define(linker: &mut Linker<State>) -> Result<(), Error> {
    let m = IMPORT_MODULE;
    linker.func_wrap(m, imports::NOW_MS, now_ms)?;
    linker.func_wrap(m, imports::LOG, log)?;
    linker.func_wrap(m, imports::PUBLISH, publish)?;
    linker.func_wrap(m, imports::SUBSCRIBE, subscribe)?;
    linker.func_wrap(m, imports::RECV, recv)?;
    linker.func_wrap(m, imports::SET_LEDS, set_leds)?;
    linker.func_wrap(m, imports::SHOW_TEXT, show_text)?;
    linker.func_wrap(m, imports::SET_PET, set_pet)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spans_reject_negative_and_overflowing_ranges() {
        assert_eq!(span(10, 2, 3), Ok(2..5));
        assert_eq!(span(10, 0, 10), Ok(0..10));
        assert_eq!(span(10, 8, 3), Err(Status::OutOfBounds));
        assert_eq!(span(10, -1, 1), Err(Status::OutOfBounds));
        assert_eq!(span(10, 1, -1), Err(Status::OutOfBounds));
        assert_eq!(
            span(usize::MAX, i32::MAX, i32::MAX),
            Ok(i32::MAX as usize..2 * i32::MAX as usize)
        );
    }

    #[test]
    fn log_lines_are_repaired_and_truncated() {
        assert_eq!(log_line(b"hi\xffthere").as_str(), "hi\u{fffd}there");
        let long = [b'x'; 100];
        assert_eq!(log_line(&long).len(), MAX_LOG);
        // A 2-byte char cut by the window is dropped, not replaced.
        let mut cut = [b'a'; MAX_LOG + 1];
        cut[MAX_LOG - 1] = 0xc3;
        cut[MAX_LOG] = 0xa9;
        let line = log_line(&cut);
        assert_eq!(line.len(), MAX_LOG - 1);
        assert!(!line.contains('\u{fffd}'));
    }

    #[test]
    fn samples_require_app_keys() {
        assert!(Sample::new(b"app/x", b"1").is_some());
        assert!(Sample::new(b"conagotchi/sync", b"1").is_none());
        assert!(Sample::new(b"app/x", &[0; MAX_PAYLOAD + 1]).is_none());
    }
}
