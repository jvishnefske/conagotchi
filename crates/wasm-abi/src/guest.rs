//! Guest-side wrappers (feature `guest`): safe functions over the raw
//! imports, for apps compiled to `wasm32-unknown-unknown`. On other targets
//! the imports cannot be linked, so everything except
//! [`export_app!`](crate::export_app) is
//! compiled only for `target_arch = "wasm32"`.
//!
//! # The one `unsafe`
//!
//! The import declarations in the private `raw` module are a Rust 2024
//! `unsafe extern "C"` block whose items are all marked `safe fn`: they take
//! plain integers and raw pointers (which the host bounds-checks against
//! linear memory and never trusts), so calling them needs no `unsafe`. The
//! `unsafe_code` lint still fires on the block itself, so the crate is
//! `#![deny(unsafe_code)]` and only `raw` carries `#[allow(unsafe_code)]`.
//! App crates can stay `#![forbid(unsafe_code)]`.
//!
//! # Exports
//!
//! [`export_app!`](crate::export_app) defines the `init` / `tick` exports.
//! Its expansion uses `#[unsafe(export_name)]`; because the macro comes from
//! this crate, the `unsafe_code` lint does not fire in the app crate.
//!
//! # Results
//!
//! Every wrapper that can fail returns `Result<_, Status>`, decoding the
//! negative return codes of the ABI.

use crate::Status;

/// Defines the app's `init` and `tick` exports from two plain functions:
/// `export_app!(init: my_init, tick: my_tick);` with `fn my_init()` and
/// `fn my_tick(now_ms: i64)`.
#[macro_export]
macro_rules! export_app {
    (init: $init:path, tick: $tick:path $(,)?) => {
        // In an anonymous const so the shims' names cannot clash with the
        // app's own `init` / `tick` functions.
        const _: () = {
            /// `init` export: called once after instantiation.
            #[unsafe(export_name = "init")]
            extern "C" fn __badge_export_init() {
                $init()
            }

            /// `tick` export: called every host tick with the host time.
            #[unsafe(export_name = "tick")]
            extern "C" fn __badge_export_tick(now_ms: i64) {
                $tick(now_ms)
            }
        };
    };
}

#[cfg(target_arch = "wasm32")]
#[allow(unsafe_code)]
mod raw {
    //! Raw imports from module `badge` (see [`crate::imports`]). Every item
    //! is a `safe fn`: arguments are integers and pointers the host checks.

    #[link(wasm_import_module = "badge")]
    unsafe extern "C" {
        #[link_name = "now_ms"]
        pub safe fn now_ms() -> i64;
        #[link_name = "log"]
        pub safe fn log(ptr: *const u8, len: usize);
        #[link_name = "publish"]
        pub safe fn publish(
            key: *const u8,
            key_len: usize,
            data: *const u8,
            data_len: usize,
        ) -> i32;
        #[link_name = "subscribe"]
        pub safe fn subscribe(key: *const u8, key_len: usize) -> i32;
        #[link_name = "recv"]
        pub safe fn recv(buf: *mut u8, cap: usize) -> i32;
        #[link_name = "set_leds"]
        pub safe fn set_leds(rgb: *const u8) -> i32;
        #[link_name = "show_text"]
        pub safe fn show_text(ptr: *const u8, len: usize) -> i32;
        #[link_name = "set_pet"]
        pub safe fn set_pet(mode: i32) -> i32;
    }
}

/// Maps an import's `i32` result to `Ok(non-negative)` or the [`Status`].
/// Unknown negative codes become [`Status::BadArgument`].
#[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
fn check(code: i32) -> Result<usize, Status> {
    usize::try_from(code).map_err(|_| Status::from_code(code).unwrap_or(Status::BadArgument))
}

/// Host milliseconds, sampled when the current call started.
#[cfg(target_arch = "wasm32")]
pub fn now_ms() -> i64 {
    raw::now_ms()
}

/// Logs one line (truncated by the host to [`crate::MAX_LOG`] bytes; at most
/// [`crate::MAX_LOG_PER_CALL`] lines per call are kept).
#[cfg(target_arch = "wasm32")]
pub fn log(line: &str) {
    raw::log(line.as_ptr(), line.len())
}

/// Publishes `data` on the app key `key`.
#[cfg(target_arch = "wasm32")]
pub fn publish(key: &str, data: &[u8]) -> Result<(), Status> {
    check(raw::publish(
        key.as_ptr(),
        key.len(),
        data.as_ptr(),
        data.len(),
    ))
    .map(drop)
}

/// Subscribes to the app key `key` (a no-op if already subscribed).
#[cfg(target_arch = "wasm32")]
pub fn subscribe(key: &str) -> Result<(), Status> {
    check(raw::subscribe(key.as_ptr(), key.len())).map(drop)
}

/// Pops the next received sample into `buf` and returns `(key, payload)`
/// borrowed from it, or `None` if nothing is queued. A buffer of
/// `1 + MAX_KEY + MAX_PAYLOAD` bytes always fits.
#[cfg(target_arch = "wasm32")]
pub fn recv(buf: &mut [u8]) -> Result<Option<(&str, &[u8])>, Status> {
    let n = check(raw::recv(buf.as_mut_ptr(), buf.len()))?;
    if n == 0 {
        return Ok(None);
    }
    split_record(buf.get(..n).ok_or(Status::OutOfBounds)?).map(Some)
}

/// Splits a `recv` record into a `&str` key and the payload.
#[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
fn split_record(record: &[u8]) -> Result<(&str, &[u8]), Status> {
    let (key, payload) = crate::decode_sample(record).ok_or(Status::BadKey)?;
    let key = core::str::from_utf8(key).map_err(|_| Status::BadKey)?;
    Ok((key, payload))
}

/// Sets the 8 LEDs (R G B each) and switches the pattern to manual.
#[cfg(target_arch = "wasm32")]
pub fn set_leds(rgb: &[u8; 3 * crate::LED_COUNT]) -> Result<(), Status> {
    check(raw::set_leds(rgb.as_ptr())).map(drop)
}

/// Shows `text` on the HOST screen (`'\n'` separates lines).
#[cfg(target_arch = "wasm32")]
pub fn show_text(text: &str) -> Result<(), Status> {
    check(raw::show_text(text.as_ptr(), text.len())).map(drop)
}

/// Selects a pet mode (`PetMode` value, `0..PET_MODES`).
#[cfg(target_arch = "wasm32")]
pub fn set_pet(mode: i32) -> Result<(), Status> {
    check(raw::set_pet(mode)).map(drop)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn return_codes_map_to_status() {
        assert_eq!(check(0), Ok(0));
        assert_eq!(check(7), Ok(7));
        assert_eq!(check(Status::Full.code()), Err(Status::Full));
        assert_eq!(check(-99), Err(Status::BadArgument));
    }

    #[test]
    fn records_split_into_str_key_and_payload() {
        assert_eq!(split_record(b"\x05app/xhi"), Ok(("app/x", &b"hi"[..])));
        assert_eq!(split_record(b"\x09ab"), Err(Status::BadKey));
    }
}
