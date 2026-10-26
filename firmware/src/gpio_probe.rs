//! GPIO probe: host-driven reads, writes and ADC samples of the free header
//! pins (dev builds only).
//!
//! [`GpioProbe`] owns every pin in [`badge_core::board::PROBE_PINS`],
//! obtained from the named GPIO peripherals in `main` (no stealing):
//!
//! - pins that are not ADC1-capable are held as [`Flex`] drivers
//!   ([`ProbePin`]), starting as floating inputs;
//! - the ADC1-capable pins ([`badge_core::board::ADC_PINS`]: GPIO 1, 3, 5,
//!   6, 7, 8) are held as raw peripherals in [`AdcPins`] together with the
//!   `ADC1` peripheral. Each request reborrows the pin and builds the driver
//!   it needs: a [`Flex`] for a digital read or write (`Flex::new` resets the
//!   pad, and `Flex` has no `Drop`, so a driven level persists after the
//!   request), or a calibrated ADC pin for [`GpioProbe::adc_read`].
//!
//! - [`GpioProbe::read`] releases any drive (back to a floating input) and
//!   samples the pad level.
//! - [`GpioProbe::write`] sets the level, enables the push-pull driver and
//!   reads the pad back, so the result shows whether the line actually
//!   followed (a shorted or loaded pin reads back differently).
//! - [`GpioProbe::adc_read`] takes `samples` one-shot conversions on ADC1 at
//!   11 dB attenuation with the eFuse curve calibration (about 0..3100 mV)
//!   and reports mean, min and max in millivolts. Each conversion is polled
//!   at most [`MAX_POLLS`] times, so the call is bounded. ADC2 pins
//!   (GPIO 15..18) are not sampled: ADC2 is shared with Wi-Fi on the S3.
//!
//! - [`GpioProbe::apply`] carries out a payload's pin operation
//!   (`badge_mw::payload::gpio::PinOp`): an input with a pull resistor,
//!   sampled, or a driven output, read back.
//!
//! Requests for any other pin return `ErrorCode::PinNotAllowed`; the core
//! already filters them, this is a second line of defence.
//!
//! # Ownership: one slot, handed out per call
//!
//! Two users share the probe: the shell (host `GpioRead` / `GpioWrite` /
//! `AdcRead`) and the wasm task (a payload's `gpio_*` calls,
//! `wasm::BadgeDevice`). Both run as tasks of the one embassy executor, and
//! a payload's host call is synchronous, so the shell can never run while a
//! payload call waits: a request/answer handshake between the two would
//! only ever time out. Instead `main` [`install`]s the probe in one static
//! slot, and each user borrows it for the duration of one synchronous call
//! through [`with`]: the probe is taken out of the slot (a short critical
//! section), used with interrupts enabled, and put back before `with`
//! returns. Nothing holds it across an `await`, so on one executor the slot
//! is never found empty; if it were (a reentrant call), `with` answers
//! `Unavailable::InUse` instead of waiting. A probe that failed setup or was
//! never installed is `Unavailable::Absent`.

use core::cell::Cell;
use core::sync::atomic::{AtomicBool, Ordering};

use badge_core::board::{ADC_PINS, PROBE_PINS};
use badge_mw::payload::gpio::{AbiPull, PinOp, Unavailable};
use badge_proto::{AdcLevel, ErrorCode};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use esp_hal::analog::adc::{Adc, AdcCalCurve, AdcChannel, AdcConfig, Attenuation};
use esp_hal::gpio::{AnalogPin, AnyPin, Flex, InputConfig, Level, OutputConfig, Pin, Pull};
use esp_hal::peripherals::{ADC1, GPIO1, GPIO3, GPIO5, GPIO6, GPIO7, GPIO8};

/// Number of probe pins held as [`Flex`] drivers (all but the ADC pins).
pub const FLEX_COUNT: usize = PROBE_PINS.len() - ADC_PINS.len();
/// Total number of probe pins.
pub const COUNT: usize = PROBE_PINS.len();
/// Polls of one conversion before giving up (a conversion takes a few µs).
pub const MAX_POLLS: u32 = 10_000;

/// The probe's slot (see the module docs); empty until [`install`].
static SLOT: Mutex<CriticalSectionRawMutex, Cell<Option<GpioProbe>>> = Mutex::new(Cell::new(None));

/// Puts the probe in its slot for [`with`]; called once by `main`.
pub fn install(probe: GpioProbe) {
    SLOT.lock(|slot| slot.set(Some(probe)));
    INSTALLED.store(true, Ordering::Relaxed);
}

/// Runs `f` on the probe, taken out of its slot for the call and put back
/// afterwards.
///
/// # Errors
///
/// `Unavailable::Absent` if no probe was installed, `Unavailable::InUse` if
/// another caller has it (never on one executor, see the module docs).
pub fn with<R>(f: impl FnOnce(&mut GpioProbe) -> R) -> Result<R, Unavailable> {
    let Some(mut probe) = SLOT.lock(Cell::take) else {
        return Err(Unavailable::empty_slot(INSTALLED.load(Ordering::Relaxed)));
    };
    let result = f(&mut probe);
    SLOT.lock(|slot| slot.set(Some(probe)));
    Ok(result)
}

/// Set by [`install`]: tells an empty slot in use from no probe at all.
static INSTALLED: AtomicBool = AtomicBool::new(false);

/// One non-ADC probe pin: its GPIO number and driver.
pub struct ProbePin {
    number: u8,
    flex: Flex<'static>,
}

impl ProbePin {
    /// Wraps a GPIO peripheral as a floating input.
    pub fn new(pin: impl Pin + 'static) -> Self {
        let number = pin.number();
        Self { number, flex: floating(pin) }
    }
}

/// The ADC1-capable probe pins and the ADC1 peripheral, held raw.
pub struct AdcPins {
    /// GPIO1 (ADC1 channel 0).
    pub gpio1: GPIO1<'static>,
    /// GPIO3 (ADC1 channel 2).
    pub gpio3: GPIO3<'static>,
    /// GPIO5 (ADC1 channel 4).
    pub gpio5: GPIO5<'static>,
    /// GPIO6 (ADC1 channel 5).
    pub gpio6: GPIO6<'static>,
    /// GPIO7 (ADC1 channel 6).
    pub gpio7: GPIO7<'static>,
    /// GPIO8 (ADC1 channel 7).
    pub gpio8: GPIO8<'static>,
    /// The ADC1 peripheral.
    pub adc1: ADC1<'static>,
}

/// A fresh floating-input driver on `pin` (input buffer on, output off, no
/// pull).
fn floating<'d>(pin: impl Pin + 'd) -> Flex<'d> {
    let mut flex = Flex::new(pin);
    flex.apply_input_config(&InputConfig::default().with_pull(Pull::None));
    flex.apply_output_config(&OutputConfig::default());
    flex.set_output_enable(false);
    flex.set_input_enable(true);
    flex
}

/// Setup failures.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum ProbeSetupError {
    /// The pin at this index is not the expected non-ADC probe pin.
    PinMismatch {
        /// Index into the pin array.
        index: usize,
        /// GPIO number supplied.
        got: u8,
    },
}

/// Owner of all probe pins.
pub struct GpioProbe {
    pins: [ProbePin; FLEX_COUNT],
    adc: AdcPins,
}

impl GpioProbe {
    /// Takes the non-ADC probe pins, in `PROBE_PINS` order with the ADC pins
    /// left out, and the ADC pins.
    pub fn new(pins: [ProbePin; FLEX_COUNT], adc: AdcPins) -> Result<Self, ProbeSetupError> {
        let expected = PROBE_PINS.iter().filter(|p| !ADC_PINS.contains(p));
        for (index, (pin, want)) in pins.iter().zip(expected).enumerate() {
            if pin.number != want.0 {
                return Err(ProbeSetupError::PinMismatch { index, got: pin.number });
            }
        }
        Ok(Self { pins, adc })
    }

    /// Runs `f` on a digital driver for probe pin `number`.
    fn with_flex<R>(&mut self, number: u32, f: impl FnOnce(&mut Flex<'_>) -> R) -> Result<R, ErrorCode> {
        if let Some(p) = self.pins.iter_mut().find(|p| u32::from(p.number) == number) {
            return Ok(f(&mut p.flex));
        }
        let a = &mut self.adc;
        let pin: AnyPin<'_> = match number {
            1 => a.gpio1.reborrow().into(),
            3 => a.gpio3.reborrow().into(),
            5 => a.gpio5.reborrow().into(),
            6 => a.gpio6.reborrow().into(),
            7 => a.gpio7.reborrow().into(),
            8 => a.gpio8.reborrow().into(),
            _ => return Err(ErrorCode::PinNotAllowed),
        };
        Ok(f(&mut floating(pin)))
    }

    /// Releases `pin` to a floating input and returns its level.
    pub fn read(&mut self, pin: u32) -> Result<bool, ErrorCode> {
        self.with_flex(pin, |flex| {
            flex.set_output_enable(false);
            flex.set_input_enable(true);
            flex.is_high()
        })
    }

    /// Drives `pin` to `level` and returns the level read back from the pad.
    pub fn write(&mut self, pin: u32, level: bool) -> Result<bool, ErrorCode> {
        self.with_flex(pin, |flex| {
            flex.set_level(Level::from(level));
            flex.set_input_enable(true);
            flex.set_output_enable(true);
            flex.is_high()
        })
    }

    /// Makes `pin` an input with `pull` and returns its pad level.
    pub fn sense(&mut self, pin: u32, pull: Pull) -> Result<bool, ErrorCode> {
        self.with_flex(pin, |flex| {
            flex.set_output_enable(false);
            flex.apply_input_config(&InputConfig::default().with_pull(pull));
            flex.set_input_enable(true);
            flex.is_high()
        })
    }

    /// Carries out a payload's pin operation; returns the level read from
    /// the pad (for a `Drive`, the read-back as in [`GpioProbe::write`]).
    pub fn apply(&mut self, op: PinOp) -> Result<bool, ErrorCode> {
        match op {
            PinOp::Sense { pin, pull } => self.sense(u32::from(pin), hal_pull(pull)),
            PinOp::Drive { pin, level } => self.write(u32::from(pin), level),
        }
    }

    /// Samples an ADC1 probe pin `samples` times (1..=64, checked by the
    /// core) and returns mean / min / max in millivolts.
    pub fn adc_read(&mut self, pin: u32, samples: u32) -> Result<AdcLevel, ErrorCode> {
        let a = &mut self.adc;
        let (sum, min, max) = match pin {
            1 => sample(&mut a.adc1, a.gpio1.reborrow(), samples),
            3 => sample(&mut a.adc1, a.gpio3.reborrow(), samples),
            5 => sample(&mut a.adc1, a.gpio5.reborrow(), samples),
            6 => sample(&mut a.adc1, a.gpio6.reborrow(), samples),
            7 => sample(&mut a.adc1, a.gpio7.reborrow(), samples),
            8 => sample(&mut a.adc1, a.gpio8.reborrow(), samples),
            _ => Err(ErrorCode::PinNotAllowed),
        }?;
        let n = samples.max(1);
        Ok(AdcLevel { pin, millivolts: sum / n, min_mv: min, max_mv: max, samples: n })
    }
}

/// The esp-hal pull for an ABI pull.
const fn hal_pull(pull: AbiPull) -> Pull {
    match pull {
        AbiPull::None => Pull::None,
        AbiPull::Up => Pull::Up,
        AbiPull::Down => Pull::Down,
    }
}

/// Takes `samples` calibrated conversions of `pin`; returns (sum, min, max)
/// in millivolts, or `ErrorCode::Radio` if a conversion never completes.
fn sample<P>(adc1: &mut ADC1<'static>, pin: P, samples: u32) -> Result<(u32, u32, u32), ErrorCode>
where
    P: AdcChannel + AnalogPin,
{
    let mut config = AdcConfig::new();
    let mut adc_pin = config.enable_pin_with_cal::<P, AdcCalCurve<ADC1<'_>>>(pin, Attenuation::_11dB);
    let mut adc = Adc::new(adc1.reborrow(), config);
    let (mut sum, mut min, mut max) = (0u32, u32::MAX, 0u32);
    for _ in 0..samples.max(1) {
        let mut value = None;
        for _ in 0..MAX_POLLS {
            if let Ok(v) = adc.read_oneshot(&mut adc_pin) {
                value = Some(u32::from(v));
                break;
            }
        }
        let v = value.ok_or(ErrorCode::Radio)?;
        sum = sum.saturating_add(v);
        min = min.min(v);
        max = max.max(v);
    }
    Ok((sum, min, max))
}
