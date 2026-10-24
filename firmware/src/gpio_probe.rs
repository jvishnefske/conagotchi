//! GPIO probe: host-driven reads and writes of the free header pins.
//!
//! [`GpioProbe`] owns one [`Flex`] driver for every pin in
//! [`badge_core::board::PROBE_PINS`], obtained from the named GPIO
//! peripherals in `main` (no stealing). Pins start as floating inputs: input
//! buffer on, output driver off, no pull.
//!
//! - [`GpioProbe::read`] releases any drive (back to a floating input) and
//!   samples the pad level.
//! - [`GpioProbe::write`] sets the level, enables the push-pull driver and
//!   reads the pad back, so the result shows whether the line actually
//!   followed (a shorted or loaded pin reads back differently).
//!
//! Requests for any other pin return `ErrorCode::PinNotAllowed`; the core
//! already filters them, this is a second line of defence.

use badge_core::board::PROBE_PINS;
use badge_proto::ErrorCode;
use esp_hal::gpio::{Flex, InputConfig, Level, OutputConfig, Pin, Pull};

/// Number of probe pins.
pub const COUNT: usize = PROBE_PINS.len();

/// One probe pin: its GPIO number and driver.
pub struct ProbePin {
    number: u8,
    flex: Flex<'static>,
}

impl ProbePin {
    /// Wraps a GPIO peripheral as a floating input.
    pub fn new(pin: impl Pin + 'static) -> Self {
        let number = pin.number();
        let mut flex = Flex::new(pin);
        flex.apply_input_config(&InputConfig::default().with_pull(Pull::None));
        flex.apply_output_config(&OutputConfig::default());
        flex.set_output_enable(false);
        flex.set_input_enable(true);
        Self { number, flex }
    }
}

/// Setup failures.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum ProbeSetupError {
    /// The pin at this index is not `PROBE_PINS[index]`.
    PinMismatch {
        /// Index into the pin array.
        index: usize,
        /// GPIO number supplied.
        got: u8,
    },
}

/// Owner of all probe pins.
pub struct GpioProbe {
    pins: [ProbePin; COUNT],
}

impl GpioProbe {
    /// Takes the probe pins, which must be given in `PROBE_PINS` order.
    pub fn new(pins: [ProbePin; COUNT]) -> Result<Self, ProbeSetupError> {
        for (index, (pin, expected)) in pins.iter().zip(PROBE_PINS).enumerate() {
            if pin.number != expected.0 {
                return Err(ProbeSetupError::PinMismatch { index, got: pin.number });
            }
        }
        Ok(Self { pins })
    }

    fn pin(&mut self, number: u32) -> Result<&mut Flex<'static>, ErrorCode> {
        self.pins
            .iter_mut()
            .find(|p| u32::from(p.number) == number)
            .map(|p| &mut p.flex)
            .ok_or(ErrorCode::PinNotAllowed)
    }

    /// Releases `pin` to a floating input and returns its level.
    pub fn read(&mut self, pin: u32) -> Result<bool, ErrorCode> {
        let flex = self.pin(pin)?;
        flex.set_output_enable(false);
        flex.set_input_enable(true);
        Ok(flex.is_high())
    }

    /// Drives `pin` to `level` and returns the level read back from the pad.
    pub fn write(&mut self, pin: u32, level: bool) -> Result<bool, ErrorCode> {
        let flex = self.pin(pin)?;
        flex.set_level(Level::from(level));
        flex.set_input_enable(true);
        flex.set_output_enable(true);
        Ok(flex.is_high())
    }
}
