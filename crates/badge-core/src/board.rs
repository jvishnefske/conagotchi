//! Conagotchi 2026 board pin map.
//!
//! Source: the stock MicroPython firmware's `config.py` (read back from the
//! badge's littlefs partition), cross-checked against the PCB silkscreen.
//! The module is an ESP32-S3-WROOM-1 with 8 MB flash and no PSRAM.
//!
//! The firmware owns the GPIO peripherals by name (`peripherals.GPIO45`), so
//! these constants are the single place that documents which number is used
//! for what. They also drive the GPIO probe allowlist.

/// GPIO number newtype, so pin numbers cannot be confused with other integers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Gpio(pub u8);

/// Buttons, active-low with internal pull-up.
pub mod button {
    use super::Gpio;
    /// BOOT button (strapping pin; safe to read after boot).
    pub const BOOT: Gpio = Gpio(0);
    /// START button.
    pub const START: Gpio = Gpio(9);
    /// SELECT button.
    pub const SELECT: Gpio = Gpio(21);
    /// Right-arrow button (strapping pin).
    pub const RIGHT: Gpio = Gpio(46);
    /// Left-arrow button.
    pub const LEFT: Gpio = Gpio(47);
}

/// LEDs.
pub mod led {
    use super::Gpio;
    /// Red "alert" indicator, active-high.
    pub const ALERT: Gpio = Gpio(2);
    /// Green "raffle" indicator, active-high.
    pub const RAFFLE: Gpio = Gpio(4);
    /// WS2812B chain data line (strapping pin).
    pub const RGB_DATA: Gpio = Gpio(45);
    /// Number of WS2812B LEDs on the chain.
    pub const RGB_COUNT: usize = 8;
}

/// GC9A01 240x240 round display on SPI.
pub mod display {
    use super::Gpio;
    /// Chip select.
    pub const CS: Gpio = Gpio(10);
    /// MOSI (silkscreen "SDA").
    pub const MOSI: Gpio = Gpio(11);
    /// SCK (silkscreen "SCL").
    pub const SCK: Gpio = Gpio(12);
    /// Data/command select.
    pub const DC: Gpio = Gpio(13);
    /// Reset, active-low.
    pub const RST: Gpio = Gpio(14);
    /// Panel width in pixels.
    pub const WIDTH: u16 = 240;
    /// Panel height in pixels.
    pub const HEIGHT: u16 = 240;
    /// SPI clock used by the stock firmware.
    pub const SPI_HZ: u32 = 40_000_000;
}

/// IR badge-to-badge link (38 kHz carrier, TSOP38238 receiver).
pub mod ir {
    use super::Gpio;
    /// IR LED drive.
    pub const TX: Gpio = Gpio(41);
    /// Demodulated receiver output, idle high.
    pub const RX: Gpio = Gpio(42);
}

/// Wired badge-to-badge UART on the LINK header.
pub mod link {
    use super::Gpio;
    /// UART TX.
    pub const TX: Gpio = Gpio(39);
    /// UART RX.
    pub const RX: Gpio = Gpio(40);
    /// Baud rate used by the stock firmware.
    pub const BAUD: u32 = 115_200;
}

/// Pins reserved by the module or the debug link. Never exposed to probing.
///
/// 19/20 are the USB-Serial-JTAG D-/D+ that probe-rs talks over; 26..=32 are
/// the in-package SPI flash.
pub const RESERVED: &[Gpio] = &[
    Gpio(19), Gpio(20), Gpio(26), Gpio(27), Gpio(28), Gpio(29), Gpio(30), Gpio(31), Gpio(32),
];

/// Pins the GPIO probe commands may read and drive.
///
/// These are pins with no on-board function that the WROOM-1 brings out,
/// plus the UART0 header pins (free because logging uses RTT). Pins that
/// drive on-board hardware are deliberately absent.
pub const PROBE_PINS: &[Gpio] = &[
    Gpio(1), Gpio(3), Gpio(5), Gpio(6), Gpio(7), Gpio(8), Gpio(15), Gpio(16), Gpio(17),
    Gpio(18), Gpio(35), Gpio(36), Gpio(37), Gpio(38), Gpio(43), Gpio(44), Gpio(48),
];

/// Returns true if `pin` may be used by the GPIO probe commands.
pub fn probe_allowed(pin: u32) -> bool {
    PROBE_PINS.iter().any(|g| u32::from(g.0) == pin)
}

#[cfg(test)]
mod tests {
    use super::*;

    const USED: &[Gpio] = &[
        button::BOOT, button::START, button::SELECT, button::RIGHT, button::LEFT,
        led::ALERT, led::RAFFLE, led::RGB_DATA,
        display::CS, display::MOSI, display::SCK, display::DC, display::RST,
        ir::TX, ir::RX, link::TX, link::RX,
    ];

    #[test]
    fn no_pin_is_assigned_twice() {
        for (i, a) in USED.iter().enumerate() {
            for b in &USED[i + 1..] {
                assert_ne!(a, b, "pin {} assigned twice", a.0);
            }
        }
    }

    #[test]
    fn probe_pins_avoid_board_and_reserved_pins() {
        for p in PROBE_PINS {
            assert!(!USED.contains(p), "probe pin {} drives board hardware", p.0);
            assert!(!RESERVED.contains(p), "probe pin {} is reserved", p.0);
            assert!(p.0 <= 48, "GPIO{} does not exist on the S3", p.0);
        }
    }

    #[test]
    fn probe_allowlist_matches() {
        assert!(probe_allowed(1));
        assert!(!probe_allowed(19)); // USB-JTAG
        assert!(!probe_allowed(45)); // WS2812 data
        assert!(!probe_allowed(99));
    }
}
