//! The ABI between a Conagotchi wasm app (the guest) and whatever runs it
//! (the host). The badge firmware and `badgectl sim` both run apps through
//! the same `wasm-runtime` crate (wasmi), which implements exactly these
//! imports and calls exactly these exports, so an app behaves the same on the
//! badge and in the simulator.
//!
//! # Module shape
//!
//! - Imports come from module [`IMPORT_MODULE`] (`"badge"`), listed in
//!   [`imports`] with their wasm signatures.
//! - The guest exports a linear memory named [`EXPORT_MEMORY`], a function
//!   [`EXPORT_INIT`] `() -> ()` run once on start, and [`EXPORT_TICK`]
//!   `(now_ms: i64) -> ()` run once per host tick (20 ms on the badge).
//! - At most [`MAX_MODULE`] bytes of module (of which at most
//!   [`MAX_CODE_BYTES`] of code and [`MAX_DATA_BYTES`] of data; custom
//!   sections are ignored) and [`MAX_MEMORY_BYTES`] of linear memory. The memory must use the custom-page-sizes proposal with
//!   [`PAGE_SIZE`]-byte pages (Rust apps link with `--page-size=1`), because a
//!   classic 64 KiB page alone exceeds the limit. Each `init`/`tick` call
//!   gets [`FUEL_PER_CALL`] fuel; running out is a trap, so every call is
//!   bounded. Floating-point instructions and a wasm `start` function are
//!   rejected.
//! - Per call an app may queue at most [`MAX_PUBLISH_PER_CALL`] publishes and
//!   [`MAX_LOG_PER_CALL`] log lines; the host carries them out (together
//!   with the last `set_leds` / `show_text` / `set_pet` of the call) when the
//!   call returns.
//!
//! # Messaging
//!
//! Apps talk to the world through the same pub/sub model as the `middleware`
//! crate: `subscribe(key)` once, then `publish(key, data)` and `recv(buf)`.
//! Keys follow middleware key rules (see [`key_is_valid`]) and apps may only
//! use keys under [`APP_KEY_PREFIX`] (see [`app_key_is_valid`]). On the badge the
//! host carries them over ESP-NOW (and BLE advertising); in the simulator
//! over the simulator's middleware bus.
//!
//! # Return codes
//!
//! Imports returning `i32` use `>= 0` for success (a length or 0) and a
//! negative [`Status`] code for failure.

#![no_std]
// Only `guest::raw` (the wasm import declarations) may opt out, with a
// documented `allow`; `forbid` would make that impossible.
#![deny(unsafe_code)]

/// Module name of every host import.
pub const IMPORT_MODULE: &str = "badge";
/// Exported linear memory.
pub const EXPORT_MEMORY: &str = "memory";
/// Exported `() -> ()`, called once after instantiation.
pub const EXPORT_INIT: &str = "init";
/// Exported `(now_ms: i64) -> ()`, called every host tick.
pub const EXPORT_TICK: &str = "tick";

/// Largest module accepted, in bytes.
pub const MAX_MODULE: usize = 16 * 1024;
/// Linear-memory page size the runtime expects, in bytes (custom-page-sizes
/// proposal, `pagesize 1`). Any page size is accepted as long as the memory
/// fits [`MAX_MEMORY_BYTES`], which a classic 64 KiB page never does.
pub const PAGE_SIZE: u32 = 1;
/// Largest linear memory, in bytes (initial size and any `memory.grow`).
pub const MAX_MEMORY_BYTES: u32 = 16 * 1024;
/// Largest code section (all function bodies), in bytes. The interpreter's
/// compiled form is several times larger than the wasm code, so this, not
/// [`MAX_MODULE`], bounds the runtime's heap.
pub const MAX_CODE_BYTES: usize = 4 * 1024;
/// Largest data section (initial memory contents), in bytes.
pub const MAX_DATA_BYTES: usize = 4 * 1024;
/// Largest linear memory, in [`PAGE_SIZE`]-byte pages.
pub const MAX_MEMORY_PAGES: u32 = MAX_MEMORY_BYTES / PAGE_SIZE;
/// Fuel granted to each `init` / `tick` call.
pub const FUEL_PER_CALL: u64 = 20_000;
/// Longest key accepted by `publish` / `subscribe`.
pub const MAX_KEY: usize = 64;
/// Largest payload accepted by `publish` and delivered by `recv`.
pub const MAX_PAYLOAD: usize = 64;
/// Publishes one `init` / `tick` call may queue; more return
/// [`Status::Full`].
pub const MAX_PUBLISH_PER_CALL: usize = 4;
/// Log lines one `init` / `tick` call may queue; more are dropped.
pub const MAX_LOG_PER_CALL: usize = 4;
/// Number of `PetMode` values `set_pet` accepts (`0..PET_MODES`).
pub const PET_MODES: i32 = 6;
/// Most keys one app may subscribe to.
pub const MAX_SUBSCRIPTIONS: usize = 4;
/// Samples queued for the app before the oldest is dropped.
pub const RECV_QUEUE: usize = 8;
/// Longest `log` line forwarded (longer lines are truncated on a char
/// boundary).
pub const MAX_LOG: usize = 64;
/// Longest `show_text` string.
pub const MAX_TEXT: usize = 256;
/// Number of RGB LEDs `set_leds` writes (3 bytes each, R G B).
pub const LED_COUNT: usize = 8;

/// Negative return codes of the imports.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum Status {
    /// A pointer/length pair is outside linear memory.
    OutOfBounds = -1,
    /// A key is malformed (see [`key_is_valid`]) or too long.
    BadKey = -2,
    /// A payload, text or log line is too long.
    TooLong = -3,
    /// The subscription table or an outgoing queue is full.
    Full = -4,
    /// `recv`: the caller's buffer cannot hold the next sample (it stays
    /// queued).
    BufferTooSmall = -5,
    /// An argument value is out of range (e.g. pet mode).
    BadArgument = -6,
    /// The host cannot do this right now (e.g. no radio).
    Unavailable = -7,
}

impl Status {
    /// The `i32` the import returns.
    pub const fn code(self) -> i32 {
        self as i32
    }

    /// Inverse of [`Status::code`]; `None` for non-negative or unknown values.
    pub const fn from_code(code: i32) -> Option<Status> {
        Some(match code {
            -1 => Status::OutOfBounds,
            -2 => Status::BadKey,
            -3 => Status::TooLong,
            -4 => Status::Full,
            -5 => Status::BufferTooSmall,
            -6 => Status::BadArgument,
            -7 => Status::Unavailable,
            _ => return None,
        })
    }
}

/// Host imports (module [`IMPORT_MODULE`]). Each constant is the import name;
/// the doc gives the wasm signature and semantics. Pointers are offsets into
/// the guest's exported memory.
pub mod imports {
    /// `() -> i64`: host monotonic milliseconds, sampled once when the call
    /// starts (in `tick` it equals the `now_ms` argument).
    pub const NOW_MS: &str = "now_ms";
    /// `(ptr: i32, len: i32) -> ()`: log a UTF-8 line (badge: `WasmLog`
    /// event + defmt; invalid UTF-8 is replaced, overlong lines truncated).
    pub const LOG: &str = "log";
    /// `(key_ptr, key_len, data_ptr, data_len: i32) -> i32`: publish `data`
    /// on `key` (an app key, see [`app_key_is_valid`](super::app_key_is_valid)).
    /// Returns 0 or a [`Status`](super::Status); `Full` after
    /// [`MAX_PUBLISH_PER_CALL`](super::MAX_PUBLISH_PER_CALL) in one call. The
    /// host sends it when the call returns.
    pub const PUBLISH: &str = "publish";
    /// `(key_ptr, key_len: i32) -> i32`: receive samples published on `key`.
    /// Subscribing twice is a no-op. Returns 0 or a status.
    pub const SUBSCRIBE: &str = "subscribe";
    /// `(buf_ptr, buf_cap: i32) -> i32`: pop the next queued sample into the
    /// buffer as `[key_len: u8][key bytes][payload bytes]`. Returns the
    /// number of bytes written, 0 if nothing is queued, or a status.
    pub const RECV: &str = "recv";
    /// `(ptr: i32) -> i32`: set the 8 RGB LEDs from 24 bytes (R G B per
    /// LED); switches the LED pattern to MANUAL. Returns 0 or a status.
    pub const SET_LEDS: &str = "set_leds";
    /// `(ptr, len: i32) -> i32`: show UTF-8 text on the HOST screen
    /// (`'\n'` separates lines). Returns 0 or a status.
    pub const SHOW_TEXT: &str = "show_text";
    /// `(mode: i32) -> i32`: select a pet mode (`PetMode` value 0..=5).
    /// Returns 0 or a status.
    pub const SET_PET: &str = "set_pet";
}

/// True if `key` follows the middleware key rules: 1..=[`MAX_KEY`] bytes,
/// `/`-separated non-empty segments of ASCII letters, digits, `_`, `-`, `.`.
/// (The middleware itself allows 128 bytes; apps are held to [`MAX_KEY`].)
pub fn key_is_valid(key: &[u8]) -> bool {
    if key.is_empty() || key.len() > MAX_KEY {
        return false;
    }
    let mut segment = 0usize;
    for &b in key {
        if b == b'/' {
            if segment == 0 {
                return false;
            }
            segment = 0;
        } else if b.is_ascii_alphanumeric() || b == b'_' || b == b'-' || b == b'.' {
            segment += 1;
        } else {
            return false;
        }
    }
    segment != 0
}

/// Prefix every app key must start with (mirrors
/// `badge_mw::keys::APP_PREFIX`), so apps cannot spoof the badge's own keys.
pub const APP_KEY_PREFIX: &str = "app/";

/// True if an app may publish or subscribe on `key`: [`key_is_valid`] and
/// under [`APP_KEY_PREFIX`] with at least one more segment. Duplicates
/// `badge_mw::keys::app_may_use` so this crate stays dependency-free (the
/// middleware allows longer keys; apps are held to [`MAX_KEY`]).
pub fn app_key_is_valid(key: &[u8]) -> bool {
    key.starts_with(APP_KEY_PREFIX.as_bytes()) && key_is_valid(key)
}

/// Writes one `recv` record (`[key_len][key][payload]`) into `out`.
///
/// Returns the record length, or [`Status::BufferTooSmall`] if it does not
/// fit, or [`Status::BadKey`] / [`Status::TooLong`] for invalid input.
/// Hosts use this so both runtimes produce identical records.
pub fn encode_sample(key: &[u8], payload: &[u8], out: &mut [u8]) -> Result<usize, Status> {
    if !key_is_valid(key) {
        return Err(Status::BadKey);
    }
    if payload.len() > MAX_PAYLOAD {
        return Err(Status::TooLong);
    }
    let len = 1 + key.len() + payload.len();
    let dst = out.get_mut(..len).ok_or(Status::BufferTooSmall)?;
    let (head, rest) = dst.split_at_mut(1);
    let (k, p) = rest.split_at_mut(key.len());
    // key_is_valid bounds the key to MAX_KEY (< 256).
    head[0] = u8::try_from(key.len()).map_err(|_| Status::BadKey)?;
    k.copy_from_slice(key);
    p.copy_from_slice(payload);
    Ok(len)
}

/// Splits a `recv` record into `(key, payload)`; `None` if malformed.
pub fn decode_sample(record: &[u8]) -> Option<(&[u8], &[u8])> {
    let (&len, rest) = record.split_first()?;
    let len = usize::from(len);
    let key = rest.get(..len)?;
    let payload = rest.get(len..)?;
    key_is_valid(key).then_some((key, payload))
}

#[cfg(feature = "guest")]
pub mod guest;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keys_follow_middleware_rules() {
        for good in [&b"a"[..], b"conagotchi/sync", b"app/hello-1.x_y"] {
            assert!(key_is_valid(good), "{good:?}");
        }
        for bad in [&b""[..], b"/a", b"a/", b"a//b", b"a b", b"a/*", &[b'a'; MAX_KEY + 1][..]] {
            assert!(!key_is_valid(bad), "{bad:?}");
        }
    }

    #[test]
    fn app_keys_are_confined_to_the_app_prefix() {
        assert!(app_key_is_valid(b"app/hello"));
        assert!(app_key_is_valid(b"app/x/y"));
        for bad in [&b"conagotchi/sync"[..], b"app/", b"app", b"app/a b", b"apps/x"] {
            assert!(!app_key_is_valid(bad), "{bad:?}");
        }
    }

    #[test]
    fn sample_records_round_trip_and_never_truncate() {
        let mut buf = [0u8; 16];
        let n = encode_sample(b"a/b", b"xyz", &mut buf).unwrap();
        assert_eq!(&buf[..n], b"\x03a/bxyz");
        assert_eq!(decode_sample(&buf[..n]), Some((&b"a/b"[..], &b"xyz"[..])));
        assert_eq!(encode_sample(b"a/b", b"xyz", &mut buf[..6]), Err(Status::BufferTooSmall));
        assert_eq!(encode_sample(b"a b", b"", &mut buf), Err(Status::BadKey));
        assert_eq!(encode_sample(b"a", &[0; MAX_PAYLOAD + 1], &mut [0; 128]), Err(Status::TooLong));
        assert_eq!(decode_sample(b"\x09ab"), None);
    }

    #[test]
    fn status_codes_round_trip() {
        for s in [
            Status::OutOfBounds,
            Status::BadKey,
            Status::TooLong,
            Status::Full,
            Status::BufferTooSmall,
            Status::BadArgument,
            Status::Unavailable,
        ] {
            assert!(s.code() < 0);
            assert_eq!(Status::from_code(s.code()), Some(s));
        }
        assert_eq!(Status::from_code(0), None);
    }
}
