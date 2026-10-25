//! The C-ABI export surface of the wasm32 module (compiled only for
//! `target_arch = "wasm32"`).
//!
//! # Why this module allows `unsafe_code`
//!
//! A wasm export needs an unmangled symbol. In Rust 2024 both ways to get
//! one, `#[unsafe(no_mangle)]` and `#[unsafe(export_name = "...")]`, are
//! *unsafe attributes*, and the `unsafe_code` lint reports them, so a crate
//! under `#![forbid(unsafe_code)]` cannot export anything (`forbid` cannot
//! be relaxed further down). The crate therefore uses
//! `#![deny(unsafe_code)]`, every other module re-forbids it, and this one
//! module allows it. The only thing allowed here is the `no_mangle`
//! attribute itself: there are no `unsafe` blocks, functions or impls. The
//! attribute is sound because every exported name is prefixed `sim_` and is
//! unique in the module.
//!
//! # Memory protocol
//!
//! The exports take no pointers: dereferencing a host-supplied address would
//! need `unsafe`. Instead the module exposes three fixed regions inside its
//! own statics, whose addresses never change:
//!
//! - [`sim_buf_ptr`]: the input region ([`IN_CAP`] bytes). The host writes
//!   one input there, then calls the export with the length.
//! - [`sim_out_ptr`]: the output region ([`OUT_CAP`] bytes) of `wire`
//!   records. [`sim_take_output`] returns their length; the bytes stay valid
//!   until the next `sim_input_*` call, which starts a fresh batch.
//! - [`sim_framebuffer_ptr`]: the 240 × 240 RGB565 framebuffer
//!   (`fb::FB_BYTES` bytes, see [`crate::fb`]).
//!
//! The statics sit behind `spin` mutexes (the module is single-threaded, so
//! a lock is never contended) to get interior mutability without `unsafe`.
//! Every `sim_input_*` call before `sim_init` is ignored. Integer results
//! are `i32` (wasm has no unsigned types); negative means an error.

#![allow(unsafe_code)]

use spin::mutex::SpinMutex;

use crate::badge::SimBadge;
use crate::fb::FB_BYTES;
use crate::inputs;
use crate::wire::OutBuf;

/// Capacity of the input region.
pub const IN_CAP: usize = 1024;
/// Capacity of the output region.
pub const OUT_CAP: usize = 16 * 1024;

/// Everything the module keeps.
struct State {
    badge: Option<SimBadge>,
    input: [u8; IN_CAP],
    out: OutBuf<OUT_CAP>,
    /// The host took the output; the next input starts a fresh batch.
    taken: bool,
}

static STATE: SpinMutex<State> =
    SpinMutex::new(State { badge: None, input: [0; IN_CAP], out: OutBuf::new(), taken: false });
static FB: SpinMutex<[u8; FB_BYTES]> = SpinMutex::new([0; FB_BYTES]);

/// A pointer as a wasm32 `i32` address.
fn addr(p: *const u8) -> i32 {
    i32::try_from(p.addr()).unwrap_or(-1)
}

/// The first `len` bytes of the input region; empty if `len` is out of range.
fn input_len(len: i32) -> usize {
    usize::try_from(len).ok().filter(|&n| n <= IN_CAP).unwrap_or(0)
}

/// Feeds one input built from the input region.
fn feed(len: i32, make: impl FnOnce(&[u8]) -> Option<badge_core::io::Input>) -> i32 {
    let mut st = STATE.lock();
    let mut fb = FB.lock();
    let State { badge, input, out, taken } = &mut *st;
    if *taken {
        out.clear();
        *taken = false;
    }
    let n = input_len(len);
    let Some(input) = make(input.get(..n).unwrap_or(&[])) else {
        return -1;
    };
    match badge.take() {
        Some(b) => {
            *badge = Some(b.handle(input, &mut fb, out));
            0
        }
        None => -1,
    }
}

/// Boots the badge with the MAC in the low 48 bits of `mac_lo`.
#[unsafe(no_mangle)]
pub extern "C" fn sim_init(mac_lo: i64) {
    let mut st = STATE.lock();
    let mut fb = FB.lock();
    st.out.clear();
    st.taken = false;
    let badge = SimBadge::boot(inputs::mac_from_i64(mac_lo), &mut fb, &mut st.out);
    st.badge = Some(badge);
}

/// Address of the input region.
#[unsafe(no_mangle)]
pub extern "C" fn sim_buf_ptr() -> i32 {
    addr(STATE.lock().input.as_ptr())
}

/// Capacity of the input region.
#[unsafe(no_mangle)]
pub extern "C" fn sim_buf_cap() -> i32 {
    i32::try_from(IN_CAP).unwrap_or(0)
}

/// Address of the output region.
#[unsafe(no_mangle)]
pub extern "C" fn sim_out_ptr() -> i32 {
    addr(STATE.lock().out.storage().as_ptr())
}

/// Address of the framebuffer.
#[unsafe(no_mangle)]
pub extern "C" fn sim_framebuffer_ptr() -> i32 {
    addr(FB.lock().as_ptr())
}

/// Periodic tick with raw button levels.
#[unsafe(no_mangle)]
pub extern "C" fn sim_input_tick(now_ms: i64, buttons: i32) -> i32 {
    feed(0, |_| Some(inputs::tick(now_ms, buttons)))
}

/// One encoded `HostMsg` (`len` bytes in the input region).
#[unsafe(no_mangle)]
pub extern "C" fn sim_input_host(len: i32) -> i32 {
    feed(len, |b| Some(inputs::host(b)))
}

/// A stock-protocol packet (`len` payload bytes in the input region).
#[unsafe(no_mangle)]
pub extern "C" fn sim_input_radio(via: i32, rssi: i32, len: i32) -> i32 {
    feed(len, |b| inputs::radio(via, rssi, b))
}

/// An LED sync beacon.
#[unsafe(no_mangle)]
pub extern "C" fn sim_input_sync(from: i32, network_ms: i32, rx_at: i64) -> i32 {
    feed(0, |_| Some(inputs::sync(from, network_ms, rx_at)))
}

/// The runtime's answer to a wasm command: `code >= 0` is the length of an
/// encoded `WasmStatus` in the input region, `code < 0` is `-ErrorCode`.
#[unsafe(no_mangle)]
pub extern "C" fn sim_input_wasm(seq: i32, code: i32) -> i32 {
    feed(code.max(0), |b| Some(inputs::wasm(seq, code, b)))
}

/// A wasm app log line (`len` UTF-8 bytes in the input region).
#[unsafe(no_mangle)]
pub extern "C" fn sim_input_wasm_log(len: i32) -> i32 {
    feed(len, |b| Some(inputs::wasm_log(b)))
}

/// Length of the pending records at [`sim_out_ptr`]; they stay valid until
/// the next `sim_input_*` call.
#[unsafe(no_mangle)]
pub extern "C" fn sim_take_output() -> i32 {
    let mut st = STATE.lock();
    st.taken = true;
    i32::try_from(st.out.len()).unwrap_or(-1)
}

/// Records dropped so far because the output region was full.
#[unsafe(no_mangle)]
pub extern "C" fn sim_dropped() -> i32 {
    STATE.lock().badge.as_ref().map_or(-1, |b| i32::try_from(b.dropped()).unwrap_or(i32::MAX))
}

/// Traps on panic (`unreachable`); production paths do not panic.
#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    core::arch::wasm32::unreachable()
}
