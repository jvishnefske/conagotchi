//! Demo wasm app for the Conagotchi badge.
//!
//! - `init`: subscribes to `app/hello`, logs "hello from wasm" and shows a
//!   greeting on the HOST screen.
//! - `tick`: moves one lit LED along the chain every [`STEP_MS`], publishes
//!   a little-endian `u32` counter on `app/hello` every [`PUBLISH_MS`], and
//!   logs every sample received on its subscriptions.
//!
//! State lives in atomics (wasm32 without threads lowers them to plain
//! loads and stores), so the app needs no `unsafe` and no `static mut`.
//! Built by `scripts/build-apps.sh` into `apps/hello.wasm`.

#![no_std]
#![forbid(unsafe_code)]

use core::sync::atomic::{AtomicI64, AtomicU32, Ordering::Relaxed};

use wasm_abi::guest;

/// The app's pub/sub key.
const KEY: &str = "app/hello";
/// Time between LED steps.
const STEP_MS: i64 = 100;
/// Time between counter publishes.
const PUBLISH_MS: i64 = 2_000;
/// Samples handled per tick (bounds the work per call).
const RECV_PER_TICK: usize = 4;

static COUNTER: AtomicU32 = AtomicU32::new(0);
static DOT: AtomicU32 = AtomicU32::new(0);
static NEXT_STEP: AtomicI64 = AtomicI64::new(0);
static NEXT_PUBLISH: AtomicI64 = AtomicI64::new(0);

fn init() {
    let now = guest::now_ms();
    NEXT_STEP.store(now, Relaxed);
    NEXT_PUBLISH.store(now.saturating_add(PUBLISH_MS), Relaxed);
    if guest::subscribe(KEY).is_err() {
        guest::log("subscribe failed");
    }
    guest::log("hello from wasm");
    // A failure only means the text is not shown.
    let _ = guest::show_text("hello from wasm\npublishing on\napp/hello");
}

fn tick(now_ms: i64) {
    if now_ms >= NEXT_STEP.load(Relaxed) {
        NEXT_STEP.store(now_ms.saturating_add(STEP_MS), Relaxed);
        step_leds();
    }
    if now_ms >= NEXT_PUBLISH.load(Relaxed) {
        NEXT_PUBLISH.store(now_ms.saturating_add(PUBLISH_MS), Relaxed);
        let n = COUNTER.fetch_add(1, Relaxed).wrapping_add(1);
        if guest::publish(KEY, &n.to_le_bytes()).is_err() {
            guest::log("publish failed");
        }
    }
    let mut buf = [0u8; 1 + wasm_abi::MAX_KEY + wasm_abi::MAX_PAYLOAD];
    for _ in 0..RECV_PER_TICK {
        match guest::recv(&mut buf) {
            Ok(Some((_key, payload))) => log_sample(payload),
            Ok(None) | Err(_) => break,
        }
    }
}

/// Lights the next LED (dim cyan) and clears the others.
fn step_leds() {
    let dot = DOT.load(Relaxed) % wasm_abi::LED_COUNT as u32;
    DOT.store((dot + 1) % wasm_abi::LED_COUNT as u32, Relaxed);
    let mut rgb = [0u8; 3 * wasm_abi::LED_COUNT];
    let at = dot as usize * 3;
    if let Some(px) = rgb.get_mut(at..at + 3) {
        px.copy_from_slice(&[0, 40, 40]);
    }
    // A failure only means this frame is not shown.
    let _ = guest::set_leds(&rgb);
}

/// Logs "rx " plus the payload as a counter (4 bytes) or its length.
fn log_sample(payload: &[u8]) {
    let mut line = [0u8; 16];
    line[..3].copy_from_slice(b"rx ");
    let n = match <[u8; 4]>::try_from(payload) {
        Ok(bytes) => u32::from_le_bytes(bytes),
        Err(_) => payload.len() as u32,
    };
    let len = 3 + write_decimal(n, &mut line[3..]);
    if let Ok(text) = core::str::from_utf8(line.get(..len).unwrap_or(&[])) {
        guest::log(text);
    }
}

/// Writes `n` in decimal into `out` (at least 10 bytes); returns the length.
fn write_decimal(mut n: u32, out: &mut [u8]) -> usize {
    let mut digits = [0u8; 10];
    let mut len = 0;
    loop {
        digits[len] = b'0' + (n % 10) as u8;
        len += 1;
        n /= 10;
        if n == 0 || len == digits.len() {
            break;
        }
    }
    for (dst, src) in out.iter_mut().zip(digits[..len].iter().rev()) {
        *dst = *src;
    }
    len.min(out.len())
}

wasm_abi::export_app!(init: init, tick: tick);

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    core::arch::wasm32::unreachable()
}
