//! The demo payload's logic (see the crate docs). Every line of hand-written
//! code lives here, under `forbid(unsafe_code)`; the crate root only expands
//! `mw_payload::payload!`, whose generated exports need `#[unsafe(no_mangle)]`.

#![forbid(unsafe_code)]

use mw_payload::{LogLine, Payload, Slot};

/// The app's pub/sub key (carried between badges).
pub const HELLO: &str = "app/hello";
/// Button edges from the badge: `[button, pressed]`.
pub const BUTTONS: &str = "badge/buttons";
/// LED colours for the badge: 8 × (R, G, B).
pub const LEDS: &str = "badge/leds";
/// Text for the badge's HOST screen.
pub const TEXT: &str = "badge/text";
/// Shown at start.
pub const GREETING: &str = "hello from wasm\npublishing on\napp/hello";
/// Number of RGB LEDs.
pub const LED_COUNT: usize = 8;
/// Tick period: one LED step.
pub const STEP_MS: u32 = 100;
/// Time between counter publishes.
pub const PUBLISH_MS: u32 = 2_000;

/// Application state.
pub struct Hello {
    hello: Option<Slot>,
    buttons: Option<Slot>,
    dot: usize,
    counter: u32,
    next_publish: u32,
}

/// Logs `"<what> error <code>"`.
fn log_err(what: &str, err: mw_payload::AbiError) {
    let mut line = LogLine::new();
    line.push_str(what).push_str(" error ").push_u32(err.code().unsigned_abs());
    line.log();
}

/// Subscribes to `key`, logging a failure.
fn subscribe(key: &str) -> Option<Slot> {
    mw_payload::subscribe(key).map_err(|e| log_err("subscribe", e)).ok()
}

/// The LED frame with only LED `dot` lit (dim cyan).
pub fn dot_frame(dot: usize) -> [u8; 3 * LED_COUNT] {
    let mut rgb = [0u8; 3 * LED_COUNT];
    let at = (dot % LED_COUNT) * 3;
    if let Some(px) = rgb.get_mut(at..at + 3) {
        px.copy_from_slice(&[0, 40, 40]);
    }
    rgb
}

/// True once wrapping time `now` has reached `due`.
fn reached(now: u32, due: u32) -> bool {
    now.wrapping_sub(due) < u32::MAX / 2
}

impl Payload for Hello {
    fn init() -> Self {
        let hello = subscribe(HELLO);
        let buttons = subscribe(BUTTONS);
        if let Err(e) = mw_payload::publish(TEXT, GREETING.as_bytes()) {
            log_err("text", e);
        }
        if let Err(e) = mw_payload::set_tick(STEP_MS) {
            log_err("set_tick", e);
        }
        mw_payload::log("hello from wasm");
        Hello { hello, buttons, dot: 0, counter: 0, next_publish: mw_payload::now_ms().wrapping_add(PUBLISH_MS) }
    }

    fn tick(&mut self, now_ms: u32) {
        // A refused frame only means it is not shown.
        let _ = mw_payload::publish(LEDS, &dot_frame(self.dot));
        self.dot = (self.dot + 1) % LED_COUNT;
        if reached(now_ms, self.next_publish) {
            self.next_publish = now_ms.wrapping_add(PUBLISH_MS);
            self.counter = self.counter.wrapping_add(1);
            if let Err(e) = mw_payload::publish(HELLO, &self.counter.to_le_bytes()) {
                log_err("publish", e);
            }
        }
    }

    fn on_message(&mut self, slot: Slot, data: &[u8]) {
        let mut line = LogLine::new();
        if Some(slot) == self.hello {
            line.push_str("rx ");
            match <[u8; 4]>::try_from(data) {
                Ok(n) => line.push_u32(u32::from_le_bytes(n)),
                Err(_) => line.push_u32(u32::try_from(data.len()).unwrap_or(u32::MAX)).push_str(" bytes"),
            };
        } else if Some(slot) == self.buttons {
            let [button, pressed] = data else { return };
            line.push_str("button ").push_u32(u32::from(*button));
            line.push_str(if *pressed != 0 { " down" } else { " up" });
        } else {
            return;
        }
        line.log();
    }
}
