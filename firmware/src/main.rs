//! Conagotchi discovery firmware: entry point of the imperative shell.
//!
//! `main` brings the badge up in this order and then runs the core loop
//! ([`shell::run`]) forever:
//!
//! 1. RTT channels and the defmt logger ([`rtt::init`]);
//! 2. clocks at maximum, the esp-alloc heap (reclaimed bootloader RAM plus
//!    64 KiB for esp-radio, plus [`wasm::HEAP_BYTES`] for the wasm payload
//!    runtime; one shared allocator) — apart from esp-radio only the
//!    payload runtime and its image buffer allocate;
//! 3. the esp-rtos scheduler on TIMG0 / `FROM_CPU_INTR0`;
//! 4. drivers: buttons, indicators, WS2812 chain, GC9A01 display and, in
//!    `dev` builds only, the GPIO probe pins, installed in `gpio_probe`'s
//!    slot shared by the shell and payload GPIO calls (each failure is
//!    logged and that driver is left out); the buttons are sampled once here, and SELECT held
//!    while the badge boots selects payload safe mode (not BOOT, a strapping
//!    pin that enters the ROM download mode when held through a reset);
//! 5. the radio task ([`radio::start`]), the flash storage task
//!    ([`storage::start`]: sole owner of `FLASH`; stored contacts and radio
//!    key, first-boot import of the stock key, the `payload` partition), the
//!    wasm payload task ([`wasm::start`]: autostarts the installed payload
//!    unless in safe mode, without delaying the boot) and the link UART
//!    tasks ([`link_uart::start`]), and the USB serial loader link
//!    ([`serial_link::start`]: middleware's serial protocol on the
//!    USB-Serial-JTAG CDC port, next to RTT on its JTAG side);
//! 6. the core loop, with [`BoardInfo`] built from the eFuse base MAC and the
//!    crate name and version.
//!
//! Steps 1 to 5 and building the core loop state ([`shell::prepare`]) run
//! in out-of-line synchronous functions (`setup`, `boot`), so `main`'s
//! future holds only a reference to the shell state and the frame of each
//! of its polls stays small (48 B): all embassy tasks share the one main
//! stack, which is the RAM left after `.data`, `.bss` and the heap regions
//! (`.stack` in `xtensa-esp32s3-elf-size -A`, about 46 KB with the 28 KiB
//! [`wasm::HEAP_BYTES`]). `scripts/stack-report.py` reports each task's
//! deepest static call chain from the ELF.
//!
//! Modules: [`rtt`] (transport), [`buttons`], [`leds`], [`display`],
//! `gpio_probe` (`dev` feature only), [`probe`] (debugger status block),
//! [`radio`], [`storage`], [`wasm`], [`link_uart`], [`serial_link`],
//! [`shell`].
//!
//! Without the `dev` feature the GPIO probe module is compiled out and its 17
//! pins are never claimed, so they stay unconfigured (reset state).

#![no_std]
#![no_main]
#![forbid(unsafe_code)]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

mod buttons;
mod display;
#[cfg(feature = "dev")]
mod gpio_probe;
mod leds;
mod link_uart;
mod mw;
mod probe;
mod radio;
mod rtt;
mod serial_link;
mod shell;
mod storage;
mod wasm;

use badge_core::app::BoardInfo;
use defmt::{info, warn};
use embassy_executor::Spawner;
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use panic_rtt_target as _;
use static_cell::{ConstStaticCell, StaticCell};

use crate::buttons::Buttons;
use crate::display::{Display, DisplayPins};
#[cfg(feature = "dev")]
use crate::gpio_probe::{AdcPins, GpioProbe, ProbePin};
use crate::leds::{Indicators, Ws2812};
use crate::rtt::HostLink;
use crate::shell::Hardware;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

/// Firmware identification reported by `GetInfo` and serial `sys/info`
/// (at most `middleware::loader::MAX_NAME`, 32 bytes).
const FIRMWARE: &str = concat!(env!("CARGO_PKG_NAME"), " ", env!("CARGO_PKG_VERSION"));
const _: () = assert!(FIRMWARE.len() <= middleware::loader::MAX_NAME);

static HOST_LINK: StaticCell<HostLink> = StaticCell::new();
static DISPLAY_BUFFER: ConstStaticCell<[u8; display::BUFFER_LEN]> =
    ConstStaticCell::new([0; display::BUFFER_LEN]);

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let shell = setup(&spawner);
    shell::run(shell).await
}

/// Boots the badge and builds the core loop state; out of line so `main`'s
/// future (and the frame of each of its polls) holds only a reference.
#[inline(never)]
fn setup(spawner: &Spawner) -> &'static mut shell::Shell {
    let (hw, info) = boot(spawner);
    shell::prepare(hw, info)
}

/// Brings the badge up (steps 1 to 5 of the module docs) and returns the
/// drivers and identity for the core loop.
///
/// Kept out of line: inlined into `main`'s future, its temporaries (driver
/// construction, task futures moved into their pools) would enlarge the
/// stack frame of every later poll of the core loop.
#[inline(never)]
fn boot(spawner: &Spawner) -> (Hardware, BoardInfo) {
    let link = HOST_LINK.init(rtt::init());

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    // Wi-Fi/BLE coexistence needs more RAM than the reclaimed region alone.
    esp_alloc::heap_allocator!(size: 64 * 1024);
    // Room for the wasm payload runtime (wasm-host / wasmi), see `wasm`
    // (shared allocator).
    esp_alloc::heap_allocator!(size: wasm::HEAP_BYTES);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    info!("{} booting", FIRMWARE);

    let mac = efuse_mac();
    info!("base MAC {=[u8]:02x}", mac);

    let buttons = Buttons::new(
        peripherals.GPIO0,
        peripherals.GPIO47,
        peripherals.GPIO21,
        peripherals.GPIO9,
        peripherals.GPIO46,
    );
    let indicators = Indicators::new(peripherals.GPIO2, peripherals.GPIO4);
    info!("buttons + indicators ready");
    // SELECT held while the badge boots: payload safe mode (the installed
    // payload is not autostarted). Not BOOT: BOOT is GPIO0, a strapping pin,
    // and holding it through a reset enters the ROM download mode instead.
    let boot_held = badge_mw::payload::held(buttons.sample(), badge_proto::Button::Select);

    let leds = match Ws2812::new(peripherals.RMT, peripherals.GPIO45) {
        Ok(leds) => {
            info!("WS2812 chain ready on GPIO45 / RMT ch0");
            Some(leds)
        }
        Err(e) => {
            warn!("WS2812 init failed: {}", e);
            None
        }
    };

    let display_pins = DisplayPins {
        spi: peripherals.SPI2,
        sck: peripherals.GPIO12,
        mosi: peripherals.GPIO11,
        cs: peripherals.GPIO10,
        dc: peripherals.GPIO13,
        rst: peripherals.GPIO14,
    };
    let display = match Display::new(display_pins, DISPLAY_BUFFER.take()) {
        Ok(d) => {
            info!("GC9A01 init Ok (invert on, BGR, 40 MHz)");
            Some(d)
        }
        Err(e) => {
            warn!("GC9A01 init failed: {}", e);
            None
        }
    };

    // GPIO probe pins are claimed only in dev builds.
    #[cfg(feature = "dev")]
    {
        // ADC1-capable pins are held raw (digital or ADC driver per request).
        let adc_pins = AdcPins {
            gpio1: peripherals.GPIO1,
            gpio3: peripherals.GPIO3,
            gpio5: peripherals.GPIO5,
            gpio6: peripherals.GPIO6,
            gpio7: peripherals.GPIO7,
            gpio8: peripherals.GPIO8,
            adc1: peripherals.ADC1,
        };
        let probe_pins = [
            ProbePin::new(peripherals.GPIO15),
            ProbePin::new(peripherals.GPIO16),
            ProbePin::new(peripherals.GPIO17),
            ProbePin::new(peripherals.GPIO18),
            ProbePin::new(peripherals.GPIO35),
            ProbePin::new(peripherals.GPIO36),
            ProbePin::new(peripherals.GPIO37),
            ProbePin::new(peripherals.GPIO38),
            ProbePin::new(peripherals.GPIO43),
            ProbePin::new(peripherals.GPIO44),
            ProbePin::new(peripherals.GPIO48),
        ];
        // Shared by the shell and payload GPIO calls (see `gpio_probe`).
        match GpioProbe::new(probe_pins, adc_pins) {
            Ok(g) => {
                gpio_probe::install(g);
                info!("GPIO probe ready ({} pins)", gpio_probe::COUNT);
            }
            Err(e) => warn!("GPIO probe setup failed: {}", e),
        }
    }

    match radio::start(spawner, peripherals.WIFI, peripherals.BT) {
        Ok(()) => info!("radio task started"),
        Err(e) => warn!("radio start failed: {}", e),
    }

    match storage::start(spawner, peripherals.FLASH) {
        Ok(()) => info!("storage task started"),
        Err(e) => warn!("storage start failed: {}", e),
    }

    match wasm::start(spawner, boot_held) {
        Ok(()) => info!("wasm task started"),
        Err(e) => warn!("wasm start failed: {}", e),
    }

    match link_uart::start(spawner, peripherals.UART1, peripherals.GPIO39, peripherals.GPIO40) {
        Ok(()) => info!("link UART ready on GPIO39/40"),
        Err(e) => warn!("link UART start failed: {}", e),
    }

    match serial_link::start(spawner, peripherals.USB_DEVICE) {
        Ok(()) => info!("serial loader link started"),
        Err(e) => warn!("serial loader link start failed: {}", e),
    }

    let hw = Hardware {
        link,
        display,
        leds,
        indicators,
        buttons,
    };
    (hw, BoardInfo { mac, firmware: FIRMWARE })
}

/// Reads the factory base MAC (the Wi-Fi station address) from eFuse.
fn efuse_mac() -> [u8; 6] {
    let addr = esp_hal::efuse::base_mac_address();
    let mut mac = [0u8; 6];
    for (dst, src) in mac.iter_mut().zip(addr.as_bytes()) {
        *dst = *src;
    }
    mac
}
