//! Real [`Hardware`] for `adopt` / `restore` (imperative shell).
//!
//! - Probe list and chip identification: probe-rs `Lister` (local USB
//!   enumeration only) and a normal, non-halting attach.
//! - Flash read and serial write: the `espflash` crate over the badge's
//!   USB-Serial-JTAG CDC port, found by matching the port's USB serial
//!   number to the badge MAC. espflash resets the chip into the ROM loader
//!   with the USB-Serial-JTAG sequence, loads its RAM stub, and checks every
//!   read and write with MD5. After writing, the badge is reset with the RTC
//!   watchdog when the strapping register permits it, else with the
//!   USB-Serial-JTAG hard reset.
//! - JTAG write (`--no-backup`): probe-rs `FlashLoader` with the raw
//!   segments, then a core reset.
//! - Verification: probe-rs RTT attach (scanning RAM, or at the ELF's
//!   `_SEGGER_RTT` when known) and one `GetInfo` request through
//!   [`crate::session::Conn`].
//!
//! [`elf_image`] turns a firmware ELF into the bootloader + partition table +
//! app segments with espflash's esp-idf image builder, the same code
//! `espflash save-image` and probe-rs' `IdfLoader` use.

use std::borrow::Cow;
use std::io::{BufRead, Read, Write};
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use anyhow::{Context, Result, anyhow, bail};
use espflash::connection::{Connection, ResetAfterOperation, ResetBeforeOperation};
use espflash::flasher::{FlashData, FlashFrequency, FlashMode, FlashSettings, FlashSize, Flasher};
use espflash::image_format::idf::IdfBootloaderFormat;
use espflash::target::{Chip, ProgressCallbacks};
use probe_rs::probe::DebugProbeSelector;
use probe_rs::probe::list::Lister;
use serialport::{SerialPortType, UsbPortInfo};

use crate::adopt::{ESP_JTAG_PID, ESP_VID, FlashImage, Hardware, Mac, ProbeEntry, Segment};
use crate::probe::ProbeLink;
use crate::session::{Conn, SystemClock};

/// Baud rate for the ROM loader handshake (USB CDC ignores it).
const BAUD: u32 = 115_200;
/// espflash read block size and window.
const READ_BLOCK: u32 = 0x1000;
const READ_IN_FLIGHT: u32 = 64;

/// The badge's SPI flash settings: DIO, 80 MHz, 8 MB, as in the stock
/// bootloader header.
fn flash_settings() -> FlashSettings {
    let mut settings = FlashSettings::default();
    settings.mode = Some(FlashMode::Dio);
    settings.freq = Some(FlashFrequency::_80Mhz);
    settings.size = Some(FlashSize::_8Mb);
    settings
}

/// Builds bootloader (espflash's bundled esp-idf bootloader for ESP32-S3),
/// partition table (from `partitions_csv`) and app image from a firmware ELF.
pub fn elf_image(elf: &[u8], partitions_csv: &Path) -> Result<FlashImage> {
    let chip = Chip::Esp32s3;
    let data = FlashData::new(flash_settings(), 0, None, chip, chip.default_xtal_frequency());
    let format = IdfBootloaderFormat::new(elf, &data, Some(partitions_csv), None, None, None)
        .with_context(|| format!("building the esp-idf image with {}", partitions_csv.display()))?;
    let segments = format
        .flash_segments()
        .map(|s| Segment { addr: s.addr, data: s.data.into_owned() })
        .collect();
    Ok(FlashImage::from_segments(segments)?)
}

/// probe-rs selector for one badge.
pub fn selector(mac: Mac) -> DebugProbeSelector {
    DebugProbeSelector {
        vendor_id: ESP_VID,
        product_id: ESP_JTAG_PID,
        interface: None,
        serial_number: Some(mac.usb_serial()),
    }
}

/// Attaches probe-rs to one badge without reset or halt.
pub fn attach(chip: &str, mac: Mac) -> Result<probe_rs::Session> {
    let probe = Lister::new().open(selector(mac)).with_context(|| format!("cannot open the JTAG probe of {mac}"))?;
    probe
        .attach(chip, probe_rs::Permissions::default())
        .with_context(|| format!("cannot attach to {chip} on {mac}"))
}

/// The CDC serial port of a badge.
fn find_port(mac: Mac) -> Result<(String, UsbPortInfo)> {
    let ports = serialport::available_ports().context("listing serial ports")?;
    ports
        .into_iter()
        .find_map(|p| match p.port_type {
            SerialPortType::UsbPort(info)
                if info.vid == ESP_VID
                    && info.pid == ESP_JTAG_PID
                    && info.serial_number.as_deref().and_then(|s| Mac::parse(s).ok()) == Some(mac) =>
            {
                Some((p.port_name, info))
            }
            _ => None,
        })
        .ok_or_else(|| anyhow!("no serial port with USB serial {mac} (is the badge plugged in?)"))
}

/// Connects espflash (ROM loader + stub) to a badge.
fn connect(mac: Mac) -> Result<Flasher> {
    let (name, info) = find_port(mac)?;
    let port = serialport::new(&name, BAUD)
        .flow_control(serialport::FlowControl::None)
        .open_native()
        .with_context(|| format!("opening {name}"))?;
    let conn = Connection::new(
        port,
        info,
        ResetAfterOperation::WatchdogReset,
        ResetBeforeOperation::UsbReset,
        BAUD,
    );
    let mut flasher = Flasher::connect(conn, true, true, false, Some(Chip::Esp32s3), None)
        .with_context(|| format!("connecting to the ROM loader on {name}"))?;
    flasher.set_flash_size(FlashSize::_8Mb);
    Ok(flasher)
}

/// Resets out of the loader: RTC watchdog if the strap allows it (no
/// download-mode re-latch), else the USB-Serial-JTAG hard reset.
fn reset_to_app(flasher: &mut Flasher) -> Result<()> {
    let chip = flasher.chip();
    let conn = flasher.connection();
    if chip.can_rtc_wdt_reset(conn).unwrap_or(false) {
        chip.rtc_wdt_reset(conn).context("watchdog reset")?;
    } else {
        conn.reset().context("hard reset")?;
    }
    Ok(())
}

/// Percentage progress on stderr.
#[derive(Default)]
struct Progress {
    addr: u32,
    total: usize,
    last: usize,
}

impl ProgressCallbacks for Progress {
    fn init(&mut self, addr: u32, total: usize) {
        self.addr = addr;
        self.total = total.max(1);
        self.last = usize::MAX;
    }
    fn update(&mut self, current: usize) {
        let pct = current.saturating_mul(100) / self.total;
        if pct / 10 != self.last / 10 || self.last == usize::MAX {
            eprint!("\r  {:#08x}: {pct:3}%", self.addr);
            let _ = std::io::stderr().flush();
            self.last = pct;
        }
    }
    fn verifying(&mut self) {
        eprint!(" verifying");
    }
    fn finish(&mut self, skipped: bool) {
        eprintln!("{}", if skipped { " skipped" } else { " done" });
    }
}

/// probe-rs + espflash implementation of [`Hardware`].
pub struct RealHardware {
    chip: String,
    rtt_addr: Option<u64>,
    stop: &'static AtomicBool,
}

impl RealHardware {
    /// `chip` is the probe-rs target, `rtt_addr` the `_SEGGER_RTT` address
    /// of the image being flashed (if it came from an ELF), `stop` the
    /// Ctrl-C flag.
    pub fn new(chip: &str, rtt_addr: Option<u64>, stop: &'static AtomicBool) -> Self {
        Self { chip: chip.to_string(), rtt_addr, stop }
    }

    fn rtt_link(&self, mac: Mac, wait: Duration) -> Result<ProbeLink> {
        let session = attach(&self.chip, mac)?;
        ProbeLink::with_rtt(session, self.rtt_addr, wait)
    }
}

impl Hardware for RealHardware {
    fn probes(&mut self) -> Result<Vec<ProbeEntry>> {
        Ok(Lister::new()
            .list_all()
            .into_iter()
            .map(|p| ProbeEntry { vendor_id: p.vendor_id, product_id: p.product_id, serial: p.serial_number })
            .collect())
    }

    fn stop_requested(&self) -> bool {
        self.stop.load(Ordering::SeqCst)
    }

    fn sleep_ms(&mut self, ms: u64) {
        std::thread::sleep(Duration::from_millis(ms));
    }

    fn say(&mut self, line: &str) {
        eprintln!("{line}");
    }

    fn ask(&mut self, question: &str) -> Option<String> {
        eprint!("{question}");
        let _ = std::io::stderr().flush();
        let mut line = String::new();
        match std::io::stdin().lock().read_line(&mut line) {
            Ok(0) | Err(_) => None,
            Ok(_) => Some(line),
        }
    }

    fn chip(&mut self, badge: Mac) -> Result<String> {
        let session = attach(&self.chip, badge)?;
        Ok(session.target().name.clone())
    }

    fn file_exists(&self, path: &Path) -> bool {
        path.exists()
    }

    fn write_file(&mut self, path: &Path, data: &[u8], overwrite: bool) -> Result<()> {
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
        }
        let mut options = std::fs::OpenOptions::new();
        options.write(true);
        if overwrite {
            options.create(true).truncate(true);
        } else {
            options.create_new(true);
        }
        let mut file = options.open(path).with_context(|| format!("creating {}", path.display()))?;
        file.write_all(data).with_context(|| format!("writing {}", path.display()))?;
        file.sync_all().with_context(|| format!("syncing {}", path.display()))?;
        Ok(())
    }

    fn read_flash(&mut self, badge: Mac, addr: u32, len: u32) -> Result<Vec<u8>> {
        let mut flasher = connect(badge)?;
        let tmp = std::env::temp_dir().join(format!(
            "badgectl-{}-{}-{addr:x}.bin",
            std::process::id(),
            badge.file_stem()
        ));
        eprintln!("  reading {len:#x} bytes at {addr:#x} ...");
        let started = Instant::now();
        let read = flasher.read_flash(addr, len, READ_BLOCK, READ_IN_FLIGHT, tmp.clone());
        let data = read.map_err(anyhow::Error::from).and_then(|()| {
            let mut buf = Vec::with_capacity(len as usize);
            std::fs::File::open(&tmp)?.read_to_end(&mut buf)?;
            Ok(buf)
        });
        let _ = std::fs::remove_file(&tmp);
        let data = data.with_context(|| format!("reading flash {addr:#x}+{len:#x} from {badge}"))?;
        eprintln!("  read {:#x} bytes in {:.1?}", data.len(), started.elapsed());
        // Leave the badge as it was: back to its app.
        reset_to_app(&mut flasher)?;
        Ok(data)
    }

    fn write_serial(&mut self, badge: Mac, image: &FlashImage) -> Result<()> {
        let mut flasher = connect(badge)?;
        let segments: Vec<espflash::image_format::Segment<'_>> = image
            .segments()
            .iter()
            .map(|s| espflash::image_format::Segment { addr: s.addr, data: Cow::Borrowed(s.data.as_slice()) })
            .collect();
        flasher
            .write_bins_to_flash(&segments, &mut Progress::default())
            .with_context(|| format!("writing flash of {badge}"))?;
        reset_to_app(&mut flasher)
    }

    fn write_jtag(&mut self, badge: Mac, image: &FlashImage) -> Result<()> {
        let mut session = attach(&self.chip, badge)?;
        let mut loader = session.target().flash_loader();
        for s in image.segments() {
            loader.add_data(u64::from(s.addr), &s.data).with_context(|| format!("segment at {:#x}", s.addr))?;
        }
        let mut options = probe_rs::flashing::DownloadOptions::default();
        options.verify = true;
        loader.commit(&mut session, options).with_context(|| format!("JTAG flashing {badge}"))?;
        session.core(0)?.reset().context("reset after flashing")?;
        Ok(())
    }

    fn boot_output(&mut self, badge: Mac, ms: u64) -> Vec<u8> {
        // The port re-enumerates after the reset; take whatever arrives.
        let deadline = Instant::now() + Duration::from_millis(ms);
        let mut out = Vec::new();
        while Instant::now() < deadline {
            let Ok((name, _)) = find_port(badge) else {
                std::thread::sleep(Duration::from_millis(50));
                continue;
            };
            let Ok(mut port) = serialport::new(&name, BAUD).timeout(Duration::from_millis(100)).open() else {
                std::thread::sleep(Duration::from_millis(50));
                continue;
            };
            let mut buf = [0u8; 512];
            while Instant::now() < deadline {
                match port.read(&mut buf) {
                    Ok(n) => out.extend_from_slice(buf.get(..n).unwrap_or(&[])),
                    Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                    Err(_) => break,
                }
            }
        }
        out
    }

    fn rtt_alive(&mut self, badge: Mac, timeout_ms: u64) -> bool {
        let deadline = Instant::now() + Duration::from_millis(timeout_ms);
        while Instant::now() < deadline && !self.stop_requested() {
            let left = deadline.saturating_duration_since(Instant::now());
            if self.rtt_link(badge, left.min(Duration::from_secs(3))).is_ok() {
                return true;
            }
            std::thread::sleep(Duration::from_millis(250));
        }
        false
    }

    fn info(&mut self, badge: Mac) -> Result<String> {
        let link = self.rtt_link(badge, Duration::from_secs(5))?;
        let mut conn = Conn::new(link, SystemClock::default());
        let seq = crate::request::initial_seq(u64::from(std::process::id()));
        let msg = crate::request::host_msg(&crate::cli::RequestCmd::Info, seq)?;
        let resp = conn.request(&msg, 5000, &mut Quiet)?;
        if crate::format::is_error(&resp) {
            bail!("badge answered {}", crate::format::response_text(&resp));
        }
        Ok(crate::format::response_text(&resp))
    }
}

/// Drops everything but the awaited response.
struct Quiet;

impl crate::session::Observer for Quiet {
    fn log_bytes(&mut self, _bytes: &[u8]) {}
    fn event(&mut self, _event: &badge_proto::Event) {}
    fn stray_response(&mut self, _response: &badge_proto::Response) {}
    fn frame_error(&mut self, _error: &crate::session::UpError) {}
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Cross-check against the espflash CLI. Needs a firmware ELF and the
    /// output of `espflash save-image --chip esp32s3 --merge --skip-padding
    /// --partition-table firmware/partitions.csv --flash-size 8mb
    /// --flash-mode dio --flash-freq 80mhz <elf> <merged>`; run with
    /// `BADGECTL_TEST_ELF=... BADGECTL_TEST_MERGED=... cargo test -- --ignored`.
    #[test]
    #[ignore = "needs a firmware build and an espflash-merged image"]
    fn elf_image_matches_espflash_save_image() {
        let elf = std::fs::read(std::env::var("BADGECTL_TEST_ELF").unwrap()).unwrap();
        let merged = std::fs::read(std::env::var("BADGECTL_TEST_MERGED").unwrap()).unwrap();
        let csv = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../firmware/partitions.csv");
        let image = elf_image(&elf, &csv).unwrap();
        assert!(image.end() <= u64::from(crate::adopt::FLASH_IMAGE_END));
        assert_eq!(image.merged(), merged);
    }

    #[test]
    fn selector_names_the_badge() {
        let mac = Mac::parse("e8:3d:c1:f6:0a:c8").unwrap();
        let sel = selector(mac);
        assert_eq!((sel.vendor_id, sel.product_id), (ESP_VID, ESP_JTAG_PID));
        assert_eq!(sel.serial_number.as_deref(), Some("E8:3D:C1:F6:0A:C8"));
        let parsed: DebugProbeSelector = "303a:1001:E8:3D:C1:F6:0A:C8".parse().unwrap();
        assert_eq!(parsed, sel);
    }

    #[test]
    fn flash_settings_match_stock_header() {
        let s = flash_settings();
        assert_eq!(s.mode, Some(FlashMode::Dio));
        assert_eq!(s.freq, Some(FlashFrequency::_80Mhz));
        assert_eq!(s.size, Some(FlashSize::_8Mb));
    }
}
