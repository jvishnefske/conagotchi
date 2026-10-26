//! Pure decisions behind a payload's `gpio_*` host calls on the badge (dev
//! builds): which pins it may use, what each call does to them, and how the
//! firmware's probe errors map onto the "mw" ABI.
//!
//! A payload may only use [`PROBE_PINS`]; any other pin is
//! `AbiError::Unsupported` (the `wasm_host::Device` contract for pins the
//! board reserves).
//!
//! [`PayloadPins`] remembers the mode the payload gave each probe pin. It is
//! a value: every call consumes it and returns the next table together with
//! the [`PinOp`] the firmware applies to the probe. The firmware keeps the
//! new table only when the operation succeeded. The semantics follow
//! middleware's reference firmware:
//!
//! - [`PayloadPins::output`]: push-pull output, driven low;
//! - [`PayloadPins::input`]: input with the given pull;
//! - [`PayloadPins::write`]: only on an output (`InvalidArgument` otherwise);
//! - [`PayloadPins::read`]: an input's pad level ([`ReadStep::Apply`]) or an
//!   output's driven level ([`ReadStep::Level`], no hardware access); a pin
//!   the payload has not configured is `InvalidArgument`.
//!
//! Each [`PinOp`] carries the pin's whole configuration, so the firmware
//! re-applies it on every call: the shell's host `GpioRead` / `GpioWrite`
//! commands share the pins and may have changed them in between, and the
//! ADC-capable pins get a fresh driver per access anyway.
//!
//! [`probe_error`] maps the probe's `ErrorCode` and [`Unavailable`] maps a
//! probe that is missing or in use to the ABI error (and to the
//! `ErrorCode` the shell answers host commands with).

use badge_core::board::PROBE_PINS;
use badge_proto::ErrorCode;
use middleware::wasm_abi::{AbiError, Pull};

/// The ABI's pull setting, re-exported for the firmware's mapping.
pub use middleware::wasm_abi::Pull as AbiPull;

/// Number of pins a payload may use.
pub const PINS: usize = PROBE_PINS.len();

/// What a payload made of one pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// Not configured by the payload.
    Unset,
    /// Input with this pull.
    Input(Pull),
    /// Output driving this level (`true` = high).
    Output(bool),
}

/// The probe operation for one call: the pin's complete configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinOp {
    /// Make `pin` an input with `pull` and sample its pad level.
    Sense {
        /// GPIO number (a probe pin).
        pin: u8,
        /// Pull resistor.
        pull: Pull,
    },
    /// Drive `pin` to `level` as a push-pull output.
    Drive {
        /// GPIO number (a probe pin).
        pin: u8,
        /// Level, `true` = high.
        level: bool,
    },
}

/// How a `gpio_read` is answered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReadStep {
    /// Apply this [`PinOp::Sense`] and return the level it reads.
    Apply(PinOp),
    /// The pin is an output: its driven level, without touching it.
    Level(bool),
}

/// Why the probe could not be used for a call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Unavailable {
    /// No probe on this badge (setup failed).
    Absent,
    /// Another caller holds it right now.
    InUse,
}

impl Unavailable {
    /// Why an empty probe slot is empty: in use by another caller if a
    /// probe was `installed`, else absent.
    pub const fn empty_slot(installed: bool) -> Self {
        if installed { Unavailable::InUse } else { Unavailable::Absent }
    }

    /// The ABI error a payload gets.
    pub const fn abi_error(self) -> AbiError {
        match self {
            Unavailable::Absent => AbiError::Unsupported,
            Unavailable::InUse => AbiError::Busy,
        }
    }

    /// The error code a host command gets.
    pub const fn error_code(self) -> ErrorCode {
        match self {
            Unavailable::Absent => ErrorCode::Unsupported,
            Unavailable::InUse => ErrorCode::Busy,
        }
    }
}

/// The ABI error for a probe failure.
pub fn probe_error(code: ErrorCode) -> AbiError {
    if code == ErrorCode::PinNotAllowed || code == ErrorCode::Unsupported {
        AbiError::Unsupported
    } else if code == ErrorCode::Busy {
        AbiError::Busy
    } else {
        AbiError::Transport
    }
}

/// Index of `pin` in [`PROBE_PINS`], or `Unsupported`.
fn index(pin: u8) -> Result<usize, AbiError> {
    PROBE_PINS
        .iter()
        .position(|g| g.0 == pin)
        .ok_or(AbiError::Unsupported)
}

/// The modes a payload gave the probe pins (see the module docs).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PayloadPins {
    modes: [Mode; PINS],
}

impl Default for PayloadPins {
    fn default() -> Self {
        Self::new()
    }
}

impl PayloadPins {
    /// Every pin unset (a freshly loaded payload).
    pub const fn new() -> Self {
        Self { modes: [Mode::Unset; PINS] }
    }

    /// The mode of `pin`; `Unset` for a pin outside [`PROBE_PINS`].
    pub fn mode(&self, pin: u8) -> Mode {
        index(pin)
            .ok()
            .and_then(|i| self.modes.get(i).copied())
            .unwrap_or(Mode::Unset)
    }

    /// The table with `pin` set to `mode`.
    fn with(self, pin: u8, mode: Mode) -> Result<Self, AbiError> {
        let i = index(pin)?;
        let mut modes = self.modes;
        let slot = modes.get_mut(i).ok_or(AbiError::Unsupported)?;
        *slot = mode;
        Ok(Self { modes })
    }

    /// `gpio_output`: drive `pin` low.
    ///
    /// # Errors
    ///
    /// `Unsupported` for a pin outside [`PROBE_PINS`].
    pub fn output(self, pin: u8) -> Result<(Self, PinOp), AbiError> {
        Ok((self.with(pin, Mode::Output(false))?, PinOp::Drive { pin, level: false }))
    }

    /// `gpio_input`: make `pin` an input with `pull`.
    ///
    /// # Errors
    ///
    /// `Unsupported` for a pin outside [`PROBE_PINS`].
    pub fn input(self, pin: u8, pull: Pull) -> Result<(Self, PinOp), AbiError> {
        Ok((self.with(pin, Mode::Input(pull))?, PinOp::Sense { pin, pull }))
    }

    /// `gpio_write`: drive output `pin` to `high`.
    ///
    /// # Errors
    ///
    /// `Unsupported` for a pin outside [`PROBE_PINS`], `InvalidArgument` if
    /// the payload has not made it an output.
    pub fn write(self, pin: u8, high: bool) -> Result<(Self, PinOp), AbiError> {
        index(pin)?;
        match self.mode(pin) {
            Mode::Output(_) => Ok((self.with(pin, Mode::Output(high))?, PinOp::Drive { pin, level: high })),
            Mode::Unset | Mode::Input(_) => Err(AbiError::InvalidArgument),
        }
    }

    /// `gpio_read`: how to answer it (the table does not change).
    ///
    /// # Errors
    ///
    /// `Unsupported` for a pin outside [`PROBE_PINS`], `InvalidArgument` if
    /// the payload has not configured it.
    pub fn read(&self, pin: u8) -> Result<ReadStep, AbiError> {
        index(pin)?;
        match self.mode(pin) {
            Mode::Input(pull) => Ok(ReadStep::Apply(PinOp::Sense { pin, pull })),
            Mode::Output(level) => Ok(ReadStep::Level(level)),
            Mode::Unset => Err(AbiError::InvalidArgument),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A probe pin (GPIO15) and a pin the board uses (GPIO2, an indicator).
    const PIN: u8 = 15;
    const RESERVED: u8 = 2;

    #[test]
    fn payload_gpio_only_probe_pins() {
        let pins = PayloadPins::new();
        for p in PROBE_PINS {
            assert!(pins.output(p.0).is_ok(), "GPIO{} refused", p.0);
        }
        for pin in [RESERVED, 0, 19, 20, 45, 49, 255] {
            assert_eq!(pins.output(pin), Err(AbiError::Unsupported));
            assert_eq!(pins.input(pin, Pull::Up), Err(AbiError::Unsupported));
            assert_eq!(pins.write(pin, true), Err(AbiError::Unsupported));
            assert_eq!(pins.read(pin), Err(AbiError::Unsupported));
            assert_eq!(pins.mode(pin), Mode::Unset);
        }
    }

    #[test]
    fn payload_gpio_unconfigured_pin_is_invalid() {
        let pins = PayloadPins::new();
        assert_eq!(pins.read(PIN), Err(AbiError::InvalidArgument));
        assert_eq!(pins.write(PIN, true), Err(AbiError::InvalidArgument));
        let (pins, _) = pins.input(PIN, Pull::None).unwrap();
        // An input cannot be written.
        assert_eq!(pins.write(PIN, true), Err(AbiError::InvalidArgument));
    }

    #[test]
    fn payload_gpio_output_write_read() {
        let (pins, op) = PayloadPins::new().output(PIN).unwrap();
        assert_eq!(op, PinOp::Drive { pin: PIN, level: false });
        assert_eq!(pins.mode(PIN), Mode::Output(false));
        let (pins, op) = pins.write(PIN, true).unwrap();
        assert_eq!(op, PinOp::Drive { pin: PIN, level: true });
        // An output reads back its driven level without touching the pin.
        assert_eq!(pins.read(PIN), Ok(ReadStep::Level(true)));
        // Other pins are unaffected.
        assert_eq!(pins.mode(16), Mode::Unset);
    }

    #[test]
    fn payload_gpio_input_reads_pad_with_its_pull() {
        let (pins, op) = PayloadPins::new().input(PIN, Pull::Down).unwrap();
        assert_eq!(op, PinOp::Sense { pin: PIN, pull: Pull::Down });
        assert_eq!(
            pins.read(PIN),
            Ok(ReadStep::Apply(PinOp::Sense { pin: PIN, pull: Pull::Down }))
        );
        // Reconfiguring as output switches the answer.
        let (pins, _) = pins.output(PIN).unwrap();
        assert_eq!(pins.read(PIN), Ok(ReadStep::Level(false)));
    }

    #[test]
    fn payload_gpio_transitions_are_values() {
        let before = PayloadPins::new();
        let (after, _) = before.output(PIN).unwrap();
        // The old table is untouched: a failed probe call keeps it.
        assert_eq!(before.mode(PIN), Mode::Unset);
        assert_eq!(after.mode(PIN), Mode::Output(false));
    }

    #[test]
    fn payload_gpio_error_mapping() {
        assert_eq!(probe_error(ErrorCode::PinNotAllowed), AbiError::Unsupported);
        assert_eq!(probe_error(ErrorCode::Unsupported), AbiError::Unsupported);
        assert_eq!(probe_error(ErrorCode::Busy), AbiError::Busy);
        assert_eq!(probe_error(ErrorCode::Radio), AbiError::Transport);
        assert_eq!(Unavailable::Absent.abi_error(), AbiError::Unsupported);
        assert_eq!(Unavailable::InUse.abi_error(), AbiError::Busy);
        assert_eq!(Unavailable::Absent.error_code(), ErrorCode::Unsupported);
        assert_eq!(Unavailable::InUse.error_code(), ErrorCode::Busy);
        assert_eq!(Unavailable::empty_slot(true), Unavailable::InUse);
        assert_eq!(Unavailable::empty_slot(false), Unavailable::Absent);
    }
}
