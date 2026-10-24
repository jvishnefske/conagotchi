//! GC9A01 240×240 round display on SPI2.
//!
//! Wiring (see `badge_core::board::display`): SCK GPIO12, MOSI GPIO11,
//! CS GPIO10, DC GPIO13, RST GPIO14. The bus is a blocking esp-hal [`Spi`]
//! at 40 MHz, SPI mode 0, wrapped in an `embedded-hal-bus`
//! [`ExclusiveDevice`] for chip-select handling, and driven by `mipidsi`'s
//! GC9A01 model through its buffered [`SpiInterface`] (a 4 KiB static
//! pixel buffer, so pixel batches go out in large SPI writes).
//!
//! Panel configuration: colour inversion **on** and **BGR** subpixel order,
//! the usual setting for GC9A01 modules; if colours look swapped or
//! inverted on the badge, change [`INVERSION`] / [`ORDER`].
//!
//! [`Panel`] implements `DrawTarget<Color = Rgb565>`, so `badge_core::ui`
//! renders onto it directly. [`Display::draw`] executes a
//! [`DrawOp`] from the host canvas commands, clipping rectangles to the
//! panel.

use badge_core::board::display as pins;
use badge_core::io::DrawOp;
use badge_core::ui::to_rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use esp_hal::Blocking;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::peripherals::{GPIO10, GPIO11, GPIO12, GPIO13, GPIO14, SPI2};
use esp_hal::spi::Mode;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::time::Rate;
use mipidsi::interface::SpiInterface;
use mipidsi::models::GC9A01;
use mipidsi::options::{ColorInversion, ColorOrder};

/// Colour inversion setting sent to the panel.
pub const INVERSION: ColorInversion = ColorInversion::Inverted;
/// Subpixel order sent to the panel.
pub const ORDER: ColorOrder = ColorOrder::Bgr;
/// Size of the `mipidsi` pixel batching buffer.
pub const BUFFER_LEN: usize = 4096;

type SpiDev = ExclusiveDevice<Spi<'static, Blocking>, Output<'static>, NoDelay>;
type Interface = SpiInterface<'static, SpiDev, Output<'static>>;

/// The initialised panel; an `embedded-graphics` `DrawTarget<Color = Rgb565>`.
pub type Panel = mipidsi::Display<Interface, GC9A01, Output<'static>>;

/// Display bring-up failures.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum DisplayError {
    /// The SPI peripheral rejected the configuration.
    SpiConfig,
    /// Chip-select setup failed.
    ChipSelect,
    /// The controller init sequence failed.
    Init,
    /// An SPI transfer failed while drawing.
    Bus,
}

/// Peripherals the display needs.
pub struct DisplayPins {
    /// SPI controller.
    pub spi: SPI2<'static>,
    /// Clock, GPIO12.
    pub sck: GPIO12<'static>,
    /// Data out, GPIO11.
    pub mosi: GPIO11<'static>,
    /// Chip select, GPIO10.
    pub cs: GPIO10<'static>,
    /// Data/command, GPIO13.
    pub dc: GPIO13<'static>,
    /// Reset, GPIO14.
    pub rst: GPIO14<'static>,
}

/// The GC9A01 panel.
pub struct Display {
    panel: Panel,
}

impl Display {
    /// Brings the panel up (hardware reset, init sequence, display on).
    ///
    /// Blocks for ≈ 330 ms in the controller's power-up delays.
    pub fn new(p: DisplayPins, buffer: &'static mut [u8]) -> Result<Self, DisplayError> {
        let config = SpiConfig::default()
            .with_frequency(Rate::from_hz(pins::SPI_HZ))
            .with_mode(Mode::_0);
        let spi = Spi::new(p.spi, config)
            .map_err(|_| DisplayError::SpiConfig)?
            .with_sck(p.sck)
            .with_mosi(p.mosi);
        let cs = Output::new(p.cs, Level::High, OutputConfig::default());
        let dc = Output::new(p.dc, Level::Low, OutputConfig::default());
        let rst = Output::new(p.rst, Level::High, OutputConfig::default());
        let device = ExclusiveDevice::new_no_delay(spi, cs).map_err(|_| DisplayError::ChipSelect)?;
        let di = SpiInterface::new(device, dc, buffer);
        let panel = mipidsi::Builder::new(GC9A01, di)
            .reset_pin(rst)
            .display_size(pins::WIDTH, pins::HEIGHT)
            .invert_colors(INVERSION)
            .color_order(ORDER)
            .init(&mut Delay::new())
            .map_err(|_| DisplayError::Init)?;
        Ok(Self { panel })
    }

    /// The panel as an `embedded-graphics` draw target.
    pub fn target(&mut self) -> &mut Panel {
        &mut self.panel
    }

    /// Executes a host canvas primitive.
    pub fn draw(&mut self, op: DrawOp) -> Result<(), DisplayError> {
        match op {
            DrawOp::Fill(color) => self.panel.clear(to_rgb565(color)),
            DrawOp::Rect { x, y, w, h, color } => {
                let area = Rectangle::new(
                    Point::new(i32::from(x), i32::from(y)),
                    Size::new(u32::from(w), u32::from(h)),
                )
                .intersection(&self.panel.bounding_box());
                self.panel.fill_solid(&area, to_rgb565(color))
            }
        }
        .map_err(|_| DisplayError::Bus)
    }
}
