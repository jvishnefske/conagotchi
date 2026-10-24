//! Raw sampling of the five badge buttons.
//!
//! Each button is an active-low input with the internal pull-up enabled:
//! BOOT GPIO0, LEFT GPIO47, SELECT GPIO21, START GPIO9, RIGHT GPIO46.
//! [`Buttons::sample`] returns the undebounced levels as a [`ButtonMask`]
//! whose bit n is set while `badge_proto::Button(n)` is held. Debouncing and
//! edge detection happen in the functional core.

use badge_core::io::ButtonMask;
use badge_proto::Button;
use esp_hal::gpio::{Input, InputConfig, Pull};
use esp_hal::peripherals::{GPIO0, GPIO9, GPIO21, GPIO46, GPIO47};

/// The five button inputs, each paired with its protocol `Button` value.
pub struct Buttons {
    inputs: [(Button, Input<'static>); 5],
}

impl Buttons {
    /// Claims the button pins and enables their pull-ups.
    pub fn new(
        boot: GPIO0<'static>,
        left: GPIO47<'static>,
        select: GPIO21<'static>,
        start: GPIO9<'static>,
        right: GPIO46<'static>,
    ) -> Self {
        let cfg = InputConfig::default().with_pull(Pull::Up);
        Self {
            inputs: [
                (Button::Boot, Input::new(boot, cfg)),
                (Button::Left, Input::new(left, cfg)),
                (Button::Select, Input::new(select, cfg)),
                (Button::Start, Input::new(start, cfg)),
                (Button::Right, Input::new(right, cfg)),
            ],
        }
    }

    /// Samples all buttons; bit n set = `Button(n)` held (pin low).
    pub fn sample(&self) -> ButtonMask {
        let bits = self
            .inputs
            .iter()
            .filter(|(_, pin)| pin.is_low())
            .fold(0u8, |acc, (button, _)| acc | bit(*button));
        ButtonMask(bits)
    }
}

/// Mask bit for a protocol button value (0 for out-of-range values).
fn bit(button: Button) -> u8 {
    u32::try_from(button.0)
        .ok()
        .and_then(|n| 1u8.checked_shl(n))
        .unwrap_or(0)
}
