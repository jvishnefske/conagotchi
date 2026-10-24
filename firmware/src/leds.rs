//! WS2812B chain and the two indicator LEDs.
//!
//! [`Ws2812`] drives the 8-LED WS2812B chain on GPIO45 with RMT channel 0.
//! The RMT runs at 80 MHz (12.5 ns ticks) and each data bit is one pulse
//! code (T0H 400 ns / T0L 850 ns, T1H 800 ns / T1L 450 ns), sent in GRB
//! order, MSB first. Eight LEDs need exactly 192 codes, which fit in four
//! RMT memory blocks (4 × 48 words) given to channel 0, so a frame is
//! loaded in one go and never needs CPU refills that radio interrupts could
//! delay. The last bit's low phase has length 0, which doubles as the RMT
//! end marker; the idle level is low, so the bit still ends correctly. The
//! ≥ 50 µs latch gap comes for free because frames are at most one per
//! 20 ms tick.
//!
//! [`Indicators`] owns the active-high alert (GPIO2, red) and raffle (GPIO4,
//! green) LEDs.
//!
//! The pulse encoding is written here rather than taken from
//! `esp-hal-smartled` because that crate's `flush` unwraps an `Option` that
//! is left empty after a failed transmit; here the channel is returned on
//! every path and a failure is just an error.

use badge_core::board::led::RGB_COUNT;
use badge_core::color::Color;
use badge_proto::Indicator;
use esp_hal::Blocking;
use esp_hal::gpio::interconnect::PeripheralOutput;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::peripherals::{GPIO2, GPIO4, RMT};
use esp_hal::rmt::{Channel, PulseCode, Rmt, Tx, TxChannelConfig, TxChannelCreator};
use esp_hal::time::Rate;

/// RMT tick rate.
const RMT_MHZ: u32 = 80;
/// Pulse codes per frame: 24 bits per LED.
const CODES: usize = RGB_COUNT * 24;
/// RMT memory blocks claimed by channel 0 (48 codes each on the ESP32-S3).
const MEM_BLOCKS: u8 = 4;

const _: () = assert!(CODES <= MEM_BLOCKS as usize * 48);

/// Ticks per nanosecond-specified phase at [`RMT_MHZ`].
const fn ticks(ns: u32) -> u16 {
    (ns * RMT_MHZ / 1000) as u16
}

const ZERO: PulseCode = PulseCode::new(Level::High, ticks(400), Level::Low, ticks(850));
const ONE: PulseCode = PulseCode::new(Level::High, ticks(800), Level::Low, ticks(450));
const ZERO_LAST: PulseCode = PulseCode::new(Level::High, ticks(400), Level::Low, 0);
const ONE_LAST: PulseCode = PulseCode::new(Level::High, ticks(800), Level::Low, 0);

/// LED driver failures.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum LedError {
    /// RMT peripheral or channel configuration failed.
    Config,
    /// The RMT reported a transmit error.
    Transmit,
    /// The channel was lost by an earlier failure (cannot happen with the
    /// esp-hal 1.2 API, which returns it on every path).
    NoChannel,
}

/// Encodes a frame of colours into RMT pulse codes (GRB, MSB first).
fn encode(colors: &[Color; RGB_COUNT], out: &mut [PulseCode; CODES]) {
    let mut slots = out.iter_mut();
    for c in colors {
        for byte in [c.g, c.r, c.b] {
            for bit in (0..8).rev() {
                if let Some(slot) = slots.next() {
                    *slot = if byte & (1 << bit) != 0 { ONE } else { ZERO };
                }
            }
        }
    }
    if let Some(last) = out.last_mut() {
        *last = if *last == ONE { ONE_LAST } else { ZERO_LAST };
    }
}

/// WS2812B chain driver on one RMT TX channel.
pub struct Ws2812 {
    channel: Option<Channel<'static, Blocking, Tx>>,
    codes: [PulseCode; CODES],
}

impl Ws2812 {
    /// Configures RMT channel 0 on `pin` (GPIO45 on the badge).
    pub fn new(rmt: RMT<'static>, pin: impl PeripheralOutput<'static>) -> Result<Self, LedError> {
        let rmt = Rmt::new(rmt, Rate::from_mhz(RMT_MHZ)).map_err(|_| LedError::Config)?;
        let config = TxChannelConfig::default()
            .with_clk_divider(1)
            .with_idle_output(true)
            .with_idle_output_level(Level::Low)
            .with_carrier_modulation(false)
            .with_memsize(MEM_BLOCKS);
        let channel = rmt
            .channel0
            .configure_tx(&config)
            .map_err(|_| LedError::Config)?
            .with_pin(pin);
        Ok(Self { channel: Some(channel), codes: [ZERO; CODES] })
    }

    /// Sends one frame to the chain, blocking for ≈ 240 µs.
    pub fn write(&mut self, colors: &[Color; RGB_COUNT]) -> Result<(), LedError> {
        encode(colors, &mut self.codes);
        let channel = self.channel.take().ok_or(LedError::NoChannel)?;
        let (channel, result) = match channel.transmit(&self.codes) {
            Ok(tx) => match tx.wait() {
                Ok(ch) => (ch, Ok(())),
                Err((_, ch)) => (ch, Err(LedError::Transmit)),
            },
            Err((_, ch)) => (ch, Err(LedError::Transmit)),
        };
        self.channel = Some(channel);
        result
    }
}

/// The red alert and green raffle indicator LEDs (active-high).
pub struct Indicators {
    alert: Output<'static>,
    raffle: Output<'static>,
}

impl Indicators {
    /// Claims GPIO2 (alert) and GPIO4 (raffle), both initially off.
    pub fn new(alert: GPIO2<'static>, raffle: GPIO4<'static>) -> Self {
        Self {
            alert: Output::new(alert, Level::Low, OutputConfig::default()),
            raffle: Output::new(raffle, Level::Low, OutputConfig::default()),
        }
    }

    /// Switches one indicator. Unknown indicators are ignored (the core has
    /// already rejected them).
    pub fn set(&mut self, which: Indicator, on: bool) {
        let level = Level::from(on);
        if which == Indicator::Alert {
            self.alert.set_level(level);
        } else if which == Indicator::Raffle {
            self.raffle.set_level(level);
        }
    }
}
