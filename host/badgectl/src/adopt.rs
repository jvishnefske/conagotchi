//! `badgectl adopt` and `badgectl restore`: install the Conagotchi firmware
//! on a stock badge from a PC, keeping a backup of what was there.
//!
//! # Flow
//!
//! 1. The image to install is resolved first ([`ImageSource`]): a firmware
//!    ELF turned into bootloader + partition table + app, a merged `.bin`
//!    written at 0x0, or a clone of another attached badge (bootloader,
//!    partition table and factory app read from it, see [`clone_image`]).
//! 2. The USB probe list is polled once a second ([`POLL_MS`]). Badges that
//!    are attached when `adopt` starts are recorded and ignored
//!    ([`ProbeWatch`]); the first ESP USB-Serial-JTAG probe (VID:PID
//!    [`ESP_VID`]:[`ESP_JTAG_PID`]) that appears afterwards is the target.
//!    Its USB serial string *is* its MAC: the ESP32-S3 USB-Serial-JTAG
//!    peripheral reports the factory MAC as `E8:3D:C1:F6:0A:C8`, which is
//!    what probe-rs puts in `DebugProbeInfo::serial_number` and what the
//!    CDC serial port reports as its USB serial number ([`Mac`]).
//! 3. The chip is identified with a probe-rs attach (no reset, no halt),
//!    then one `[y/N]` question is asked ([`parse_answer`]: anything but
//!    `y`/`yes`, including EOF, means no). `--yes` skips it.
//! 4. Unless `--no-backup`, flash 0x0..[`FLASH_IMAGE_END`] is read to
//!    `~/.local/share/conagotchi/backups/<mac>.bin` ([`backup_path`]); an
//!    existing backup is never overwritten without `--force`.
//! 5. The image is written. Only 0x0..0x200000 is ever touched: the `conag`
//!    data partition at 0x200000 (contacts, radio key) and the stock `vfs`
//!    and `doom` partitions after it survive.
//! 6. The badge must come up with RTT within [`RTT_WAIT_MS`]. If it does not,
//!    or the ROM banner says it is waiting for a download
//!    ([`classify_boot`]), the user is asked to replug it once
//!    ([`Replug`]); then an `info` request confirms the firmware runs.
//!
//! # Transport choice
//!
//! probe-rs 0.32 has no public API to read SPI flash: its flash-algorithm
//! read is crate-private, and `MemoryInterface::read` only sees the part of
//! flash the cache MMU currently maps (the running app), not the bootloader,
//! partition table or the rest of the 2 MB. The `espflash` crate (already in
//! the dependency tree through `probe-rs-espressif`, same 4.6 version) reads
//! any flash range through the ROM/stub serial protocol over the badge's own
//! CDC port, with an MD5 check of the data, and writes raw segments with MD5
//! verification. So the backup path uses espflash for both the read and the
//! write, and resets with the RTC watchdog where the strapping register
//! allows it (the reset esptool uses on USB-Serial-JTAG to avoid re-latching
//! download mode). `--no-backup` never enters serial download mode: it
//! writes the same segments with probe-rs over JTAG and the badge boots
//! directly.
//!
//! Everything in this module is pure or runs against the [`Hardware`]
//! trait, so the whole flow is unit-tested with a fake badge; the real
//! implementation lives in `crate::badge_io`.

use std::collections::BTreeSet;
use std::ffi::OsStr;
use std::fmt;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};

/// Espressif USB vendor id.
pub const ESP_VID: u16 = 0x303a;
/// Product id of the USB-Serial-JTAG peripheral (ESP32-S3, C3, C6...).
pub const ESP_JTAG_PID: u16 = 0x1001;

/// First flash byte the installer never touches (start of `conag`).
pub const FLASH_IMAGE_END: u32 = 0x20_0000;
/// Bootloader offset.
pub const BOOTLOADER_ADDR: u32 = 0x0;
/// Largest bootloader (it must end before the partition table).
pub const BOOTLOADER_MAX: u32 = 0x8000;
/// Partition table offset.
pub const PARTITION_TABLE_ADDR: u32 = 0x8000;
/// Partition table sector size.
pub const PARTITION_TABLE_MAX: u32 = 0x1000;
/// Factory app offset.
pub const APP_ADDR: u32 = 0x1_0000;
/// Factory partition size (0x10000..0x200000).
pub const APP_MAX: u32 = 0x1F_0000;
/// Size of an `adopt` backup: everything below [`FLASH_IMAGE_END`].
pub const BACKUP_LEN: u32 = FLASH_IMAGE_END;
/// First byte of an ESP application or bootloader image header.
pub const ESP_IMAGE_MAGIC: u8 = 0xE9;

/// Probe list poll interval.
pub const POLL_MS: u64 = 1000;
/// How long a freshly flashed badge has to show its RTT control block.
pub const RTT_WAIT_MS: u64 = 10_000;
/// How long serial output is captured after a serial flash (ROM banner).
pub const BOOT_CAPTURE_MS: u64 = 1500;
/// Upper bound on the replug wait (unplug + plug), in polls.
pub const REPLUG_MAX_POLLS: u32 = 180;

/// Flash regions copied by `--from-badge`: (name, offset, length).
pub const CLONE_REGIONS: [(&str, u32, u32); 3] = [
    ("bootloader", BOOTLOADER_ADDR, BOOTLOADER_MAX),
    ("partition table", PARTITION_TABLE_ADDR, PARTITION_TABLE_MAX),
    ("factory app", APP_ADDR, APP_MAX),
];

/// Failures of the pure adopt logic.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum AdoptError {
    /// A probe serial / MAC string is not 6 hex bytes.
    #[error("`{0}` is not a MAC address (expected 6 hex bytes like E8:3D:C1:F6:0A:C8)")]
    BadMac(String),
    /// Neither `XDG_DATA_HOME` nor `HOME` is usable.
    #[error("cannot find a data directory: set HOME or XDG_DATA_HOME")]
    NoDataDir,
    /// The backup file already exists.
    #[error("backup {} already exists; pass --force to overwrite it", .0.display())]
    BackupExists(PathBuf),
    /// A segment does not fit its slot.
    #[error("{name} at {addr:#x} is {len:#x} bytes; at most {max:#x} fit")]
    TooLarge {
        /// Segment name.
        name: &'static str,
        /// Offset.
        addr: u32,
        /// Actual length.
        len: usize,
        /// Capacity.
        max: u32,
    },
    /// A segment starts at an offset that is not part of the layout.
    #[error("unexpected image segment at {0:#x} (expected 0x0, 0x8000 or 0x10000)")]
    UnexpectedSegment(u32),
    /// Two segments start at the same offset.
    #[error("two image segments at {0:#x}")]
    DuplicateSegment(u32),
    /// The image has no app at 0x10000.
    #[error("image has no app at 0x10000")]
    NoApp,
    /// A segment that must hold an ESP image does not start with 0xE9.
    #[error("{0} does not start with an ESP image header (0xE9)")]
    NotAnImage(&'static str),
    /// An empty merged image.
    #[error("the image is empty")]
    Empty,
    /// A backup file of the wrong size.
    #[error("backup is {len:#x} bytes; an adopt backup is exactly {:#x}", BACKUP_LEN)]
    BackupSize {
        /// Actual length.
        len: usize,
    },
    /// No badge (or more than one) to pick.
    #[error("{0}")]
    Pick(String),
}

/// A badge MAC, as reported in its USB serial string.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Mac([u8; 6]);

impl Mac {
    /// Parses 12 hex digits, optionally separated by `:`, `-` or `.`
    /// (`E8:3D:C1:F6:0A:C8`, `e8-3d-c1-f6-0a-c8`, `e83dc1f60ac8`).
    pub fn parse(s: &str) -> Result<Mac, AdoptError> {
        let bad = || AdoptError::BadMac(s.to_string());
        let digits: Vec<u8> = s
            .trim()
            .bytes()
            .filter(|b| !matches!(b, b':' | b'-' | b'.'))
            .map(|b| match b {
                b'0'..=b'9' => Ok(b - b'0'),
                b'a'..=b'f' => Ok(b - b'a' + 10),
                b'A'..=b'F' => Ok(b - b'A' + 10),
                _ => Err(bad()),
            })
            .collect::<Result<_, _>>()?;
        if digits.len() != 12 {
            return Err(bad());
        }
        let mut out = [0u8; 6];
        for (dst, [hi, lo]) in out.iter_mut().zip(digits.as_chunks::<2>().0) {
            *dst = (hi << 4) | lo;
        }
        Ok(Mac(out))
    }

    /// Lowercase hex without separators: the backup file stem.
    pub fn file_stem(&self) -> String {
        self.0.iter().map(|b| format!("{b:02x}")).collect()
    }

    /// The probe-rs / USB serial string (`E8:3D:C1:F6:0A:C8`).
    pub fn usb_serial(&self) -> String {
        self.to_string()
    }
}

impl fmt::Display for Mac {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let parts: Vec<String> = self.0.iter().map(|b| format!("{b:02X}")).collect();
        f.write_str(&parts.join(":"))
    }
}

/// Accepts a bare probe serial (the MAC) or a probe-rs selector
/// `VID:PID:SERIAL` and returns the MAC.
pub fn mac_from_selector(s: &str) -> Result<Mac, AdoptError> {
    Mac::parse(s).or_else(|e| match s.splitn(3, ':').nth(2) {
        Some(serial) => Mac::parse(serial),
        None => Err(e),
    })
}

/// One entry of the USB debug-probe list.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProbeEntry {
    /// USB vendor id.
    pub vendor_id: u16,
    /// USB product id.
    pub product_id: u16,
    /// USB serial string, if any.
    pub serial: Option<String>,
}

impl ProbeEntry {
    /// The badge MAC if this is an ESP USB-Serial-JTAG probe with a MAC
    /// serial string.
    pub fn badge_mac(&self) -> Option<Mac> {
        if self.vendor_id != ESP_VID || self.product_id != ESP_JTAG_PID {
            return None;
        }
        Mac::parse(self.serial.as_deref()?).ok()
    }
}

/// MACs of the badges in a probe list.
pub fn badges(probes: &[ProbeEntry]) -> BTreeSet<Mac> {
    probes.iter().filter_map(ProbeEntry::badge_mac).collect()
}

/// Remembers which badges are attached, to spot new ones.
///
/// A badge present at the start is ignored until it is unplugged; plugging
/// it back in then counts as new.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct ProbeWatch {
    known: BTreeSet<Mac>,
}

impl ProbeWatch {
    /// Records the badges attached now.
    pub fn start(probes: &[ProbeEntry]) -> ProbeWatch {
        ProbeWatch { known: badges(probes) }
    }

    /// Badges known to be attached.
    pub fn known(&self) -> &BTreeSet<Mac> {
        &self.known
    }

    /// Takes a new probe list; returns the next watch state and the badges
    /// that appeared since the last list (sorted).
    pub fn poll(&self, probes: &[ProbeEntry]) -> (ProbeWatch, Vec<Mac>) {
        let now = badges(probes);
        let new = now.difference(&self.known).copied().collect();
        (ProbeWatch { known: now }, new)
    }
}

/// The one confirmation question.
pub fn confirm_question(mac: Mac, chip: &str) -> String {
    format!("Flash Conagotchi firmware onto badge {mac} ({chip})? [y/N] ")
}

/// Interprets the answer to the `[y/N]` question: only `y` or `yes`
/// (any case, surrounding whitespace ignored) is yes. Anything else, an
/// empty line or EOF (`None`) is no.
pub fn parse_answer(line: Option<&str>) -> bool {
    line.map(str::trim)
        .is_some_and(|a| a.eq_ignore_ascii_case("y") || a.eq_ignore_ascii_case("yes"))
}

/// Directory for backups: `$XDG_DATA_HOME/conagotchi/backups` when that is
/// an absolute path, else `$HOME/.local/share/conagotchi/backups`.
pub fn backup_dir(xdg_data_home: Option<&OsStr>, home: Option<&OsStr>) -> Result<PathBuf, AdoptError> {
    let usable = |v: Option<&OsStr>| v.map(Path::new).filter(|p| p.is_absolute()).map(Path::to_path_buf);
    let data = match usable(xdg_data_home) {
        Some(d) => d,
        None => usable(home).ok_or(AdoptError::NoDataDir)?.join(".local").join("share"),
    };
    Ok(data.join("conagotchi").join("backups"))
}

/// Backup file for `mac` in `dir`: `<dir>/e83dc1f60ac8.bin`.
pub fn backup_path(dir: &Path, mac: Mac) -> PathBuf {
    dir.join(format!("{}.bin", mac.file_stem()))
}

/// Refuses to overwrite an existing backup unless `force`.
pub fn check_backup_slot(path: &Path, exists: bool, force: bool) -> Result<(), AdoptError> {
    if exists && !force { Err(AdoptError::BackupExists(path.to_path_buf())) } else { Ok(()) }
}

/// One contiguous piece of flash content.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Segment {
    /// Flash offset.
    pub addr: u32,
    /// Bytes.
    pub data: Vec<u8>,
}

impl Segment {
    /// First offset after the segment.
    pub fn end(&self) -> u64 {
        u64::from(self.addr) + self.data.len() as u64
    }
}

/// A validated image for flash 0x0..[`FLASH_IMAGE_END`]: segments sorted by
/// offset, non-overlapping, each inside its slot.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FlashImage {
    segments: Vec<Segment>,
}

fn slot(addr: u32) -> Result<(&'static str, u32), AdoptError> {
    match addr {
        BOOTLOADER_ADDR => Ok(("bootloader", BOOTLOADER_MAX)),
        PARTITION_TABLE_ADDR => Ok(("partition table", PARTITION_TABLE_MAX)),
        APP_ADDR => Ok(("app", APP_MAX)),
        other => Err(AdoptError::UnexpectedSegment(other)),
    }
}

impl FlashImage {
    /// The standard layout: bootloader at 0x0, partition table at 0x8000,
    /// app at 0x10000 (at most [`APP_MAX`] bytes).
    pub fn from_parts(bootloader: Vec<u8>, partition_table: Vec<u8>, app: Vec<u8>) -> Result<Self, AdoptError> {
        Self::from_segments(vec![
            Segment { addr: BOOTLOADER_ADDR, data: bootloader },
            Segment { addr: PARTITION_TABLE_ADDR, data: partition_table },
            Segment { addr: APP_ADDR, data: app },
        ])
    }

    /// Validates segments (e.g. espflash's `flash_segments()`): only the
    /// three layout offsets, each at most once and within its slot, and an
    /// app must be present. Empty segments are dropped.
    pub fn from_segments(mut segments: Vec<Segment>) -> Result<Self, AdoptError> {
        segments.retain(|s| !s.data.is_empty());
        segments.sort_by_key(|s| s.addr);
        for pair in segments.windows(2) {
            if let [a, b] = pair
                && a.addr == b.addr
            {
                return Err(AdoptError::DuplicateSegment(a.addr));
            }
        }
        for s in &segments {
            let (name, max) = slot(s.addr)?;
            if s.data.len() > max as usize {
                return Err(AdoptError::TooLarge { name, addr: s.addr, len: s.data.len(), max });
            }
        }
        if !segments.iter().any(|s| s.addr == APP_ADDR) {
            return Err(AdoptError::NoApp);
        }
        Ok(Self { segments })
    }

    /// A merged binary to be written at 0x0 (at most [`FLASH_IMAGE_END`]
    /// bytes).
    pub fn from_merged(bin: Vec<u8>) -> Result<Self, AdoptError> {
        if bin.is_empty() {
            return Err(AdoptError::Empty);
        }
        if bin.len() > FLASH_IMAGE_END as usize {
            return Err(AdoptError::TooLarge { name: "merged image", addr: 0, len: bin.len(), max: FLASH_IMAGE_END });
        }
        Ok(Self { segments: vec![Segment { addr: 0, data: bin }] })
    }

    /// An `adopt` backup (exactly [`BACKUP_LEN`] bytes) to write back at 0x0.
    pub fn from_backup(bin: Vec<u8>) -> Result<Self, AdoptError> {
        if bin.len() != BACKUP_LEN as usize {
            return Err(AdoptError::BackupSize { len: bin.len() });
        }
        Self::from_merged(bin)
    }

    /// Segments in offset order.
    pub fn segments(&self) -> &[Segment] {
        &self.segments
    }

    /// First offset after the image (never above [`FLASH_IMAGE_END`]).
    pub fn end(&self) -> u64 {
        self.segments.iter().map(Segment::end).max().unwrap_or(0)
    }

    /// Total payload bytes.
    pub fn payload_len(&self) -> usize {
        self.segments.iter().map(|s| s.data.len()).sum()
    }

    /// One binary for offset 0x0: gaps filled with 0xFF, no padding after
    /// the last segment (so nothing at or above 0x200000 is erased). This
    /// is what `espflash save-image --merge --skip-padding` produces; the
    /// flashing paths write the segments instead, so only the tests use it.
    #[cfg(test)]
    pub fn merged(&self) -> Vec<u8> {
        let len = usize::try_from(self.end()).unwrap_or(0);
        let mut out = vec![0xFF; len];
        for s in &self.segments {
            let start = s.addr as usize;
            if let Some(dst) = out.get_mut(start..start + s.data.len()) {
                dst.copy_from_slice(&s.data);
            }
        }
        out
    }
}

/// `data` without its trailing erased (0xFF) bytes, rounded up to a
/// multiple of 4 bytes (flash writes are word sized).
pub fn trim_erased(data: &[u8]) -> Vec<u8> {
    let used = data.iter().rposition(|&b| b != 0xFF).map_or(0, |i| i + 1);
    let len = used.next_multiple_of(4).min(data.len());
    data.get(..len).unwrap_or(&[]).to_vec()
}

/// Builds the `--from-badge` image from the raw [`CLONE_REGIONS`] read
/// from the source badge. Trailing erased bytes are dropped; bootloader
/// and app must start with an ESP image header.
pub fn clone_image(bootloader: &[u8], partition_table: &[u8], app: &[u8]) -> Result<FlashImage, AdoptError> {
    let boot = trim_erased(bootloader);
    let table = trim_erased(partition_table);
    let app = trim_erased(app);
    if boot.first() != Some(&ESP_IMAGE_MAGIC) {
        return Err(AdoptError::NotAnImage("source bootloader"));
    }
    if app.first() != Some(&ESP_IMAGE_MAGIC) {
        return Err(AdoptError::NotAnImage("source factory app"));
    }
    if table.first() != Some(&0xAA) {
        return Err(AdoptError::NotAnImage("source partition table"));
    }
    FlashImage::from_parts(boot, table, app)
}

/// True if captured serial output shows the ROM waiting in download mode.
pub fn rom_download_banner(output: &[u8]) -> bool {
    const MARKERS: [&[u8]; 3] = [b"waiting for download", b"(DOWNLOAD(", b"DOWNLOAD_BOOT"];
    MARKERS.iter().any(|m| output.windows(m.len()).any(|w| w == *m))
}

/// What happened after flashing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootOutcome {
    /// The firmware's RTT block appeared.
    Running,
    /// The ROM banner says the chip is waiting for a download.
    StuckRomBanner,
    /// No RTT within [`RTT_WAIT_MS`].
    NoRtt,
}

/// Classifies the boot from captured serial output and whether RTT came up.
pub fn classify_boot(output: &[u8], rtt_alive: bool) -> BootOutcome {
    if rtt_alive {
        BootOutcome::Running
    } else if rom_download_banner(output) {
        BootOutcome::StuckRomBanner
    } else {
        BootOutcome::NoRtt
    }
}

/// Replug wait: the badge must disappear, then reappear.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Replug {
    /// Waiting for the badge to be unplugged.
    WaitUnplug,
    /// Unplugged; waiting for it to come back.
    WaitPlug,
    /// Back.
    Done,
}

impl Replug {
    /// Next state given whether the badge is in the probe list now.
    pub fn step(self, present: bool) -> Replug {
        match (self, present) {
            (Replug::WaitUnplug, false) => Replug::WaitPlug,
            (Replug::WaitPlug, true) | (Replug::Done, _) => Replug::Done,
            (state, _) => state,
        }
    }
}

/// Picks the badge to talk to: `wanted` if given (it must be attached),
/// else the only attached badge.
pub fn pick_badge(probes: &[ProbeEntry], wanted: Option<Mac>) -> Result<Mac, AdoptError> {
    let present = badges(probes);
    match wanted {
        Some(mac) if present.contains(&mac) => Ok(mac),
        Some(mac) => Err(AdoptError::Pick(format!("badge {mac} is not attached"))),
        None => {
            let mut it = present.iter();
            match (it.next(), it.next()) {
                (Some(mac), None) => Ok(*mac),
                (None, _) => Err(AdoptError::Pick("no badge attached".into())),
                (Some(_), Some(_)) => {
                    let list: Vec<String> = present.iter().map(Mac::to_string).collect();
                    Err(AdoptError::Pick(format!(
                        "several badges attached ({}); choose one with --probe",
                        list.join(", ")
                    )))
                }
            }
        }
    }
}

/// Where the image comes from.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ImageSource {
    /// Already built (ELF or merged bin).
    Image(FlashImage),
    /// Cloned from another attached badge.
    Badge(Mac),
}

/// `adopt` options.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AdoptOptions {
    /// Skip the question.
    pub yes: bool,
    /// Back up first (serial path); `false` = probe-rs JTAG only.
    pub backup: bool,
    /// Overwrite an existing backup.
    pub force: bool,
    /// Backup directory.
    pub backup_dir: PathBuf,
}

/// How `adopt` ended.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AdoptOutcome {
    /// Flashed and verified.
    Adopted {
        /// The badge.
        mac: Mac,
        /// Backup written, if any.
        backup: Option<PathBuf>,
        /// The user had to replug it.
        replugged: bool,
        /// Text of the `info` response.
        info: String,
    },
    /// The user answered no.
    Declined(Mac),
    /// Ctrl-C while waiting for a badge.
    Stopped,
}

/// Badge I/O used by the flow; implemented with probe-rs and espflash in
/// `crate::badge_io` and with an in-memory fake in the tests.
pub trait Hardware {
    /// Current USB debug probes.
    fn probes(&mut self) -> Result<Vec<ProbeEntry>>;
    /// True once the user pressed Ctrl-C.
    fn stop_requested(&self) -> bool;
    /// Waits about `ms` milliseconds.
    fn sleep_ms(&mut self, ms: u64);
    /// Prints one status line.
    fn say(&mut self, line: &str);
    /// Asks a question; `None` on EOF.
    fn ask(&mut self, question: &str) -> Option<String>;
    /// Identifies the chip behind the badge's JTAG probe.
    fn chip(&mut self, badge: Mac) -> Result<String>;
    /// True if `path` exists.
    fn file_exists(&self, path: &Path) -> bool;
    /// Writes `data` to `path` (creating directories); must not replace an
    /// existing file unless `overwrite`.
    fn write_file(&mut self, path: &Path, data: &[u8], overwrite: bool) -> Result<()>;
    /// Reads `len` bytes of flash at `addr` (serial).
    fn read_flash(&mut self, badge: Mac, addr: u32, len: u32) -> Result<Vec<u8>>;
    /// Writes the image over serial, then resets the badge.
    fn write_serial(&mut self, badge: Mac, image: &FlashImage) -> Result<()>;
    /// Writes the image over JTAG with probe-rs, then resets the badge.
    fn write_jtag(&mut self, badge: Mac, image: &FlashImage) -> Result<()>;
    /// Serial output captured for about `ms` milliseconds (best effort).
    fn boot_output(&mut self, badge: Mac, ms: u64) -> Vec<u8>;
    /// True if the firmware's RTT block shows up within `timeout_ms`.
    fn rtt_alive(&mut self, badge: Mac, timeout_ms: u64) -> bool;
    /// Sends an `info` request; returns the rendered response.
    fn info(&mut self, badge: Mac) -> Result<String>;
}

fn present<H: Hardware>(hw: &mut H, mac: Mac) -> Result<bool> {
    Ok(badges(&hw.probes()?).contains(&mac))
}

/// Waits (bounded by [`REPLUG_MAX_POLLS`]) for `mac` to be unplugged and
/// plugged back in.
pub fn wait_replug<H: Hardware>(hw: &mut H, mac: Mac) -> Result<()> {
    let mut state = Replug::WaitUnplug;
    for _ in 0..REPLUG_MAX_POLLS {
        if hw.stop_requested() {
            bail!("interrupted while waiting for the badge to be replugged");
        }
        state = state.step(present(hw, mac)?);
        if state == Replug::Done {
            return Ok(());
        }
        hw.sleep_ms(POLL_MS);
    }
    bail!("badge {mac} was not replugged within {} s", u64::from(REPLUG_MAX_POLLS) * POLL_MS / 1000)
}

/// Reads [`CLONE_REGIONS`] from `source` and builds the image.
pub fn read_clone<H: Hardware>(hw: &mut H, source: Mac) -> Result<FlashImage> {
    hw.say(&format!("reading bootloader, partition table and factory app from {source}"));
    let mut parts = Vec::with_capacity(CLONE_REGIONS.len());
    for (name, addr, len) in CLONE_REGIONS {
        parts.push(hw.read_flash(source, addr, len).with_context(|| format!("reading {name} from {source}"))?);
    }
    match parts.as_slice() {
        [boot, table, app] => Ok(clone_image(boot, table, app)?),
        _ => bail!("internal: expected three clone regions"),
    }
}

/// Waits for a badge that was not attached at the start (or was replugged).
fn wait_for_new_badge<H: Hardware>(hw: &mut H, exclude: Option<Mac>) -> Result<Option<Mac>> {
    let mut watch = ProbeWatch::start(&hw.probes()?);
    let ignored = watch.known().len();
    hw.say(&format!(
        "waiting for a new badge (plug it in now; {ignored} already attached ignored; Ctrl-C to stop)"
    ));
    loop {
        if hw.stop_requested() {
            return Ok(None);
        }
        hw.sleep_ms(POLL_MS);
        let (next, new) = watch.poll(&hw.probes()?);
        watch = next;
        if let Some(mac) = new.into_iter().find(|m| Some(*m) != exclude) {
            return Ok(Some(mac));
        }
    }
}

/// Runs `adopt`: resolve image, wait for a new badge, confirm, back up,
/// flash, verify.
pub fn run_adopt<H: Hardware>(hw: &mut H, opts: &AdoptOptions, source: ImageSource) -> Result<AdoptOutcome> {
    let (image, exclude) = match source {
        ImageSource::Image(image) => (image, None),
        ImageSource::Badge(src) => (read_clone(hw, src)?, Some(src)),
    };
    hw.say(&format!(
        "image: {} segment(s), {} bytes, ends at {:#x}",
        image.segments().len(),
        image.payload_len(),
        image.end()
    ));
    let Some(mac) = wait_for_new_badge(hw, exclude)? else {
        return Ok(AdoptOutcome::Stopped);
    };
    let chip = hw.chip(mac).with_context(|| format!("identifying badge {mac}"))?;
    hw.say(&format!("new badge: chip {chip}, MAC {mac}"));
    let yes = opts.yes || parse_answer(hw.ask(&confirm_question(mac, &chip)).as_deref());
    if !yes {
        hw.say("not flashed");
        return Ok(AdoptOutcome::Declined(mac));
    }
    let backup = if opts.backup {
        let path = backup_path(&opts.backup_dir, mac);
        check_backup_slot(&path, hw.file_exists(&path), opts.force)?;
        hw.say(&format!("backing up flash 0x0..{BACKUP_LEN:#x} to {}", path.display()));
        let data = hw.read_flash(mac, 0, BACKUP_LEN).context("reading the backup")?;
        if data.len() != BACKUP_LEN as usize {
            bail!("backup read returned {:#x} bytes, expected {BACKUP_LEN:#x}", data.len());
        }
        hw.write_file(&path, &data, opts.force)?;
        Some(path)
    } else {
        None
    };
    hw.say("flashing");
    let boot_output = if opts.backup {
        hw.write_serial(mac, &image)?;
        hw.boot_output(mac, BOOT_CAPTURE_MS)
    } else {
        hw.write_jtag(mac, &image)?;
        Vec::new()
    };
    let outcome = classify_boot(&boot_output, hw.rtt_alive(mac, RTT_WAIT_MS));
    let replugged = outcome != BootOutcome::Running;
    if replugged {
        let why = match outcome {
            BootOutcome::StuckRomBanner => "the ROM is waiting for a download",
            _ => "no firmware RTT block within 10 s",
        };
        hw.say(&format!("badge seems stuck in download mode ({why}): unplug it and plug it back in once"));
        wait_replug(hw, mac)?;
        if !hw.rtt_alive(mac, RTT_WAIT_MS) {
            bail!("badge {mac} still does not run the firmware after a replug; check it with `badgectl monitor`");
        }
    }
    let info = hw.info(mac).context("verifying with an info request")?;
    hw.say(&info);
    hw.say(&format!("badge {mac} adopted"));
    Ok(AdoptOutcome::Adopted { mac, backup, replugged, info })
}

/// Runs `restore`: writes `backup` (an adopt backup) back at 0x0 of `mac`.
pub fn run_restore<H: Hardware>(hw: &mut H, mac: Mac, backup: Vec<u8>) -> Result<()> {
    let image = FlashImage::from_backup(backup)?;
    hw.say(&format!("restoring {:#x} bytes at 0x0 onto badge {mac}", image.end()));
    hw.write_serial(mac, &image)?;
    hw.say("restored; if the badge does not boot, unplug it and plug it back in");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::{BTreeMap, VecDeque};

    const A: &str = "E8:3D:C1:F6:0A:C8";
    const B: &str = "E8:3D:C1:00:11:22";

    fn mac(s: &str) -> Mac {
        Mac::parse(s).unwrap()
    }

    fn esp(serial: &str) -> ProbeEntry {
        ProbeEntry { vendor_id: ESP_VID, product_id: ESP_JTAG_PID, serial: Some(serial.into()) }
    }

    #[test]
    fn mac_normalisation() {
        let m = mac(A);
        assert_eq!(m, Mac([0xE8, 0x3D, 0xC1, 0xF6, 0x0A, 0xC8]));
        assert_eq!(m.to_string(), A);
        assert_eq!(m.usb_serial(), A);
        assert_eq!(m.file_stem(), "e83dc1f60ac8");
        assert_eq!(Mac::parse("e8-3d-c1-f6-0a-c8"), Ok(m));
        assert_eq!(Mac::parse("e83dc1f60ac8"), Ok(m));
        assert_eq!(Mac::parse(" E83D.C1F6.0AC8 "), Ok(m));
        for bad in ["", "E8:3D:C1:F6:0A", "E8:3D:C1:F6:0A:C8:00", "G8:3D:C1:F6:0A:C8", "not a mac"] {
            assert!(matches!(Mac::parse(bad), Err(AdoptError::BadMac(_))), "{bad}");
        }
        assert_eq!(mac_from_selector(A), Ok(m));
        assert_eq!(mac_from_selector("303a:1001:E8:3D:C1:F6:0A:C8"), Ok(m));
        assert!(mac_from_selector("303a:1001").is_err());
    }

    #[test]
    fn probe_filtering_and_diffing() {
        let other = ProbeEntry { vendor_id: 0x0483, product_id: 0x3748, serial: Some(A.into()) };
        let no_serial = ProbeEntry { vendor_id: ESP_VID, product_id: ESP_JTAG_PID, serial: None };
        let junk = esp("123");
        assert_eq!(other.badge_mac(), None);
        assert_eq!(no_serial.badge_mac(), None);
        assert_eq!(junk.badge_mac(), None);
        assert_eq!(esp(A).badge_mac(), Some(mac(A)));

        let watch = ProbeWatch::start(&[esp(A), other.clone()]);
        assert_eq!(watch.known().len(), 1);
        // Nothing new while only the start badge is there.
        let (watch, new) = watch.poll(&[esp(A), other]);
        assert!(new.is_empty());
        // A second badge appears.
        let (watch, new) = watch.poll(&[esp(A), esp(B)]);
        assert_eq!(new, [mac(B)]);
        // It is not reported twice.
        let (watch, new) = watch.poll(&[esp(B), esp(A)]);
        assert!(new.is_empty());
        // The start badge is unplugged and replugged: now it counts.
        let (watch, new) = watch.poll(&[esp(B)]);
        assert!(new.is_empty());
        let (_, new) = watch.poll(&[esp(B), esp(A)]);
        assert_eq!(new, [mac(A)]);
    }

    #[test]
    fn prompt_answers() {
        assert_eq!(
            confirm_question(mac(A), "esp32s3"),
            "Flash Conagotchi firmware onto badge E8:3D:C1:F6:0A:C8 (esp32s3)? [y/N] "
        );
        for yes in ["y", "Y", "yes", "YES", " yes \n", "Yes"] {
            assert!(parse_answer(Some(yes)), "{yes:?}");
        }
        for no in ["", "\n", "n", "no", "N", "yep", "sure", "y y", "1"] {
            assert!(!parse_answer(Some(no)), "{no:?}");
        }
        assert!(!parse_answer(None), "EOF means no");
    }

    #[test]
    fn backup_paths() {
        let dir = backup_dir(Some(OsStr::new("/data")), Some(OsStr::new("/home/u"))).unwrap();
        assert_eq!(dir, PathBuf::from("/data/conagotchi/backups"));
        let dir = backup_dir(None, Some(OsStr::new("/home/u"))).unwrap();
        assert_eq!(dir, PathBuf::from("/home/u/.local/share/conagotchi/backups"));
        // A relative or empty XDG_DATA_HOME is ignored, as the spec says.
        let dir = backup_dir(Some(OsStr::new("rel")), Some(OsStr::new("/home/u"))).unwrap();
        assert_eq!(dir, PathBuf::from("/home/u/.local/share/conagotchi/backups"));
        let dir = backup_dir(Some(OsStr::new("")), Some(OsStr::new("/home/u"))).unwrap();
        assert_eq!(dir, PathBuf::from("/home/u/.local/share/conagotchi/backups"));
        assert_eq!(backup_dir(None, None), Err(AdoptError::NoDataDir));
        assert_eq!(backup_dir(None, Some(OsStr::new("relative"))), Err(AdoptError::NoDataDir));
        let path = backup_path(Path::new("/b"), mac("e8:3d:c1:f6:0a:c8"));
        assert_eq!(path, PathBuf::from("/b/e83dc1f60ac8.bin"));
        assert_eq!(check_backup_slot(&path, false, false), Ok(()));
        assert_eq!(check_backup_slot(&path, true, true), Ok(()));
        assert_eq!(check_backup_slot(&path, true, false), Err(AdoptError::BackupExists(path)));
    }

    fn app(len: usize) -> Vec<u8> {
        let mut v = vec![0x11; len];
        if let Some(first) = v.first_mut() {
            *first = ESP_IMAGE_MAGIC;
        }
        v
    }

    #[test]
    fn merged_image_layout() {
        let image = FlashImage::from_parts(vec![0xE9, 1, 2, 3], vec![0xAA, 0x50], app(8)).unwrap();
        let addrs: Vec<u32> = image.segments().iter().map(|s| s.addr).collect();
        assert_eq!(addrs, [0x0, 0x8000, 0x10000]);
        assert_eq!(image.end(), 0x10008);
        assert_eq!(image.payload_len(), 14);
        let merged = image.merged();
        assert_eq!(merged.len(), 0x10008);
        assert_eq!(&merged[..4], &[0xE9, 1, 2, 3]);
        assert!(merged[4..0x8000].iter().all(|&b| b == 0xFF));
        assert_eq!(&merged[0x8000..0x8002], &[0xAA, 0x50]);
        assert!(merged[0x8002..0x10000].iter().all(|&b| b == 0xFF));
        assert_eq!(&merged[0x10000..], app(8).as_slice());

        // The biggest app ends exactly at 0x200000; one more byte is rejected.
        let full = FlashImage::from_parts(vec![0xE9], vec![0xAA], app(APP_MAX as usize)).unwrap();
        assert_eq!(full.end(), u64::from(FLASH_IMAGE_END));
        assert_eq!(full.merged().len(), FLASH_IMAGE_END as usize);
        assert!(matches!(
            FlashImage::from_parts(vec![0xE9], vec![0xAA], app(APP_MAX as usize + 1)),
            Err(AdoptError::TooLarge { name: "app", .. })
        ));
        assert!(matches!(
            FlashImage::from_parts(vec![0; 0x8001], vec![0xAA], app(4)),
            Err(AdoptError::TooLarge { name: "bootloader", .. })
        ));
        assert!(matches!(
            FlashImage::from_parts(vec![0xE9], vec![0; 0x1001], app(4)),
            Err(AdoptError::TooLarge { name: "partition table", .. })
        ));
        assert_eq!(FlashImage::from_parts(vec![0xE9], vec![0xAA], vec![]), Err(AdoptError::NoApp));
    }

    #[test]
    fn segments_are_validated() {
        let seg = |addr, len| Segment { addr, data: vec![0xE9; len] };
        // espflash order is not guaranteed; the image is sorted.
        let image = FlashImage::from_segments(vec![seg(0x10000, 4), seg(0, 4), seg(0x8000, 4)]).unwrap();
        assert_eq!(image.segments()[0].addr, 0);
        assert_eq!(
            FlashImage::from_segments(vec![seg(0x10000, 4), seg(0x200000, 4)]),
            Err(AdoptError::UnexpectedSegment(0x200000))
        );
        assert_eq!(
            FlashImage::from_segments(vec![seg(0x10000, 4), seg(0x9000, 4)]),
            Err(AdoptError::UnexpectedSegment(0x9000))
        );
        assert_eq!(
            FlashImage::from_segments(vec![seg(0x10000, 4), seg(0x10000, 4)]),
            Err(AdoptError::DuplicateSegment(0x10000))
        );
        // App-only images are fine (keeps the badge's bootloader and table).
        assert!(FlashImage::from_segments(vec![seg(0x10000, 4), seg(0, 0)]).is_ok());
    }

    #[test]
    fn merged_bins_and_backups() {
        let bin = FlashImage::from_merged(vec![0xE9; 0x1000]).unwrap();
        assert_eq!(bin.segments().len(), 1);
        assert_eq!(bin.end(), 0x1000);
        assert_eq!(FlashImage::from_merged(vec![]), Err(AdoptError::Empty));
        assert!(matches!(
            FlashImage::from_merged(vec![0; FLASH_IMAGE_END as usize + 1]),
            Err(AdoptError::TooLarge { name: "merged image", .. })
        ));
        assert!(FlashImage::from_merged(vec![0; FLASH_IMAGE_END as usize]).is_ok());
        assert!(FlashImage::from_backup(vec![0; BACKUP_LEN as usize]).is_ok());
        assert_eq!(FlashImage::from_backup(vec![0; 0x1000]), Err(AdoptError::BackupSize { len: 0x1000 }));
        // A full 8 MB dump must never be restored over conag/vfs.
        assert!(FlashImage::from_backup(vec![0; 0x80_0000]).is_err());
    }

    #[test]
    fn erased_tails_are_trimmed() {
        assert_eq!(trim_erased(&[1, 2, 3, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]), [1, 2, 3, 0xFF]);
        assert_eq!(trim_erased(&[1, 2, 3, 4, 0xFF, 0xFF]), [1, 2, 3, 4]);
        assert_eq!(trim_erased(&[0xFF; 16]), Vec::<u8>::new());
        assert_eq!(trim_erased(&[1, 0xFF]), [1, 0xFF]);
        assert_eq!(trim_erased(&[]), Vec::<u8>::new());
    }

    fn region(prefix: &[u8], len: u32) -> Vec<u8> {
        let mut v = vec![0xFF; len as usize];
        v[..prefix.len()].copy_from_slice(prefix);
        v
    }

    #[test]
    fn clone_image_from_regions() {
        let image = clone_image(
            &region(&[0xE9, 3, 2], BOOTLOADER_MAX),
            &region(&[0xAA, 0x50, 1], PARTITION_TABLE_MAX),
            &region(&[0xE9, 7, 7, 7, 7, 7], APP_MAX),
        )
        .unwrap();
        let lens: Vec<usize> = image.segments().iter().map(|s| s.data.len()).collect();
        assert_eq!(lens, [4, 4, 8]);
        assert!(image.end() < u64::from(FLASH_IMAGE_END));
        assert_eq!(
            clone_image(&region(&[], BOOTLOADER_MAX), &region(&[0xAA], 0x1000), &region(&[0xE9], APP_MAX)),
            Err(AdoptError::NotAnImage("source bootloader"))
        );
        assert_eq!(
            clone_image(&region(&[0xE9], 0x8000), &region(&[0xAA], 0x1000), &region(&[], APP_MAX)),
            Err(AdoptError::NotAnImage("source factory app"))
        );
        assert_eq!(
            clone_image(&region(&[0xE9], 0x8000), &region(&[0x00], 0x1000), &region(&[0xE9], APP_MAX)),
            Err(AdoptError::NotAnImage("source partition table"))
        );
    }

    #[test]
    fn stuck_in_download_detection() {
        let banner = b"ESP-ROM:esp32s3-20210327\r\nBuild:Mar 27 2021\r\nrst:0x15 (USB_UART_CHIP_RESET),boot:0x0 (DOWNLOAD(USB/UART0))\r\nwaiting for download\r\n";
        assert!(rom_download_banner(banner));
        assert!(rom_download_banner(b"boot:0x23 (DOWNLOAD(USB/UART0))"));
        assert!(rom_download_banner(b"..waiting for download"));
        let normal = b"ESP-ROM:esp32s3-20210327\r\nrst:0x1 (POWERON),boot:0x8 (SPI_FAST_FLASH_BOOT)\r\nload:0x3fce3810";
        assert!(!rom_download_banner(normal));
        assert!(!rom_download_banner(b""));
        assert_eq!(classify_boot(banner, false), BootOutcome::StuckRomBanner);
        assert_eq!(classify_boot(normal, false), BootOutcome::NoRtt);
        assert_eq!(classify_boot(b"", false), BootOutcome::NoRtt);
        assert_eq!(classify_boot(banner, true), BootOutcome::Running, "RTT is proof the firmware runs");
        assert_eq!(classify_boot(b"", true), BootOutcome::Running);
    }

    #[test]
    fn replug_state_machine() {
        let s = Replug::WaitUnplug;
        assert_eq!(s.step(true), Replug::WaitUnplug);
        let s = s.step(false);
        assert_eq!(s, Replug::WaitPlug);
        assert_eq!(s.step(false), Replug::WaitPlug);
        assert_eq!(s.step(true), Replug::Done);
        assert_eq!(Replug::Done.step(false), Replug::Done);
    }

    #[test]
    fn picking_a_badge() {
        let probes = [esp(A), esp(B)];
        assert_eq!(pick_badge(&probes, Some(mac(B))), Ok(mac(B)));
        assert!(matches!(pick_badge(&probes, None), Err(AdoptError::Pick(m)) if m.contains("several")));
        assert_eq!(pick_badge(&[esp(A)], None), Ok(mac(A)));
        assert!(matches!(pick_badge(&[], None), Err(AdoptError::Pick(m)) if m.contains("no badge")));
        assert!(matches!(pick_badge(&[esp(A)], Some(mac(B))), Err(AdoptError::Pick(m)) if m.contains("not attached")));
    }

    /// Scripted badge rig.
    #[derive(Default)]
    struct Fake {
        /// Probe lists returned by successive polls; the last one repeats.
        lists: VecDeque<Vec<ProbeEntry>>,
        answers: VecDeque<Option<String>>,
        files: BTreeMap<PathBuf, Vec<u8>>,
        flash: BTreeMap<Mac, Vec<u8>>,
        /// RTT results of successive `rtt_alive` calls (default false).
        rtt: VecDeque<bool>,
        boot_output: Vec<u8>,
        log: Vec<String>,
        asked: Vec<String>,
        serial_writes: Vec<(Mac, FlashImage)>,
        jtag_writes: Vec<(Mac, FlashImage)>,
        reads: Vec<(Mac, u32, u32)>,
        stop_after_polls: Option<usize>,
        polls: usize,
    }

    impl Fake {
        fn with_lists(lists: Vec<Vec<ProbeEntry>>) -> Self {
            Self { lists: lists.into(), ..Self::default() }
        }
    }

    impl Hardware for Fake {
        fn probes(&mut self) -> Result<Vec<ProbeEntry>> {
            self.polls += 1;
            let list = if self.lists.len() > 1 { self.lists.pop_front() } else { self.lists.front().cloned() };
            Ok(list.unwrap_or_default())
        }
        fn stop_requested(&self) -> bool {
            self.stop_after_polls.is_some_and(|n| self.polls >= n)
        }
        fn sleep_ms(&mut self, _ms: u64) {}
        fn say(&mut self, line: &str) {
            self.log.push(line.to_string());
        }
        fn ask(&mut self, question: &str) -> Option<String> {
            self.asked.push(question.to_string());
            self.answers.pop_front().flatten()
        }
        fn chip(&mut self, _badge: Mac) -> Result<String> {
            Ok("esp32s3".into())
        }
        fn file_exists(&self, path: &Path) -> bool {
            self.files.contains_key(path)
        }
        fn write_file(&mut self, path: &Path, data: &[u8], overwrite: bool) -> Result<()> {
            if self.files.contains_key(path) && !overwrite {
                bail!("exists");
            }
            self.files.insert(path.to_path_buf(), data.to_vec());
            Ok(())
        }
        fn read_flash(&mut self, badge: Mac, addr: u32, len: u32) -> Result<Vec<u8>> {
            self.reads.push((badge, addr, len));
            let flash = self.flash.get(&badge).cloned().unwrap_or_else(|| vec![0xFF; 0x80_0000]);
            Ok(flash[addr as usize..(addr + len) as usize].to_vec())
        }
        fn write_serial(&mut self, badge: Mac, image: &FlashImage) -> Result<()> {
            self.serial_writes.push((badge, image.clone()));
            Ok(())
        }
        fn write_jtag(&mut self, badge: Mac, image: &FlashImage) -> Result<()> {
            self.jtag_writes.push((badge, image.clone()));
            Ok(())
        }
        fn boot_output(&mut self, _badge: Mac, _ms: u64) -> Vec<u8> {
            self.boot_output.clone()
        }
        fn rtt_alive(&mut self, _badge: Mac, _timeout_ms: u64) -> bool {
            self.rtt.pop_front().unwrap_or(false)
        }
        fn info(&mut self, badge: Mac) -> Result<String> {
            Ok(format!("mac: {badge}"))
        }
    }

    fn opts(backup: bool) -> AdoptOptions {
        AdoptOptions { yes: false, backup, force: false, backup_dir: PathBuf::from("/b") }
    }

    fn image() -> FlashImage {
        FlashImage::from_parts(vec![0xE9], vec![0xAA], app(16)).unwrap()
    }

    #[test]
    fn adopt_backs_up_flashes_and_verifies() {
        let mut hw = Fake::with_lists(vec![vec![esp(A)], vec![esp(A)], vec![esp(A), esp(B)]]);
        hw.answers.push_back(Some("y".into()));
        hw.rtt.push_back(true);
        let out = run_adopt(&mut hw, &opts(true), ImageSource::Image(image())).unwrap();
        let backup = PathBuf::from("/b/e83dc1001122.bin");
        assert_eq!(
            out,
            AdoptOutcome::Adopted { mac: mac(B), backup: Some(backup.clone()), replugged: false, info: format!("mac: {B}") }
        );
        assert_eq!(hw.asked, [confirm_question(mac(B), "esp32s3")]);
        assert_eq!(hw.reads, [(mac(B), 0, BACKUP_LEN)]);
        assert_eq!(hw.files.get(&backup).map(Vec::len), Some(BACKUP_LEN as usize));
        assert_eq!(hw.serial_writes, [(mac(B), image())]);
        assert!(hw.jtag_writes.is_empty());
        assert!(hw.log.iter().any(|l| l.contains("new badge: chip esp32s3, MAC E8:3D:C1:00:11:22")));
    }

    #[test]
    fn adopt_declines_on_eof_or_invalid_answer() {
        for answer in [None, Some("maybe".to_string()), Some(String::new())] {
            let mut hw = Fake::with_lists(vec![vec![], vec![esp(B)]]);
            hw.answers.push_back(answer);
            let out = run_adopt(&mut hw, &opts(true), ImageSource::Image(image())).unwrap();
            assert_eq!(out, AdoptOutcome::Declined(mac(B)));
            assert!(hw.reads.is_empty() && hw.serial_writes.is_empty() && hw.files.is_empty());
        }
    }

    #[test]
    fn adopt_yes_skips_question_and_no_backup_uses_jtag() {
        let mut hw = Fake::with_lists(vec![vec![], vec![esp(B)]]);
        hw.rtt.push_back(true);
        let o = AdoptOptions { yes: true, ..opts(false) };
        let out = run_adopt(&mut hw, &o, ImageSource::Image(image())).unwrap();
        assert!(matches!(out, AdoptOutcome::Adopted { backup: None, replugged: false, .. }));
        assert!(hw.asked.is_empty());
        assert!(hw.reads.is_empty() && hw.files.is_empty() && hw.serial_writes.is_empty());
        assert_eq!(hw.jtag_writes, [(mac(B), image())]);
    }

    #[test]
    fn adopt_refuses_to_overwrite_backup() {
        let mut hw = Fake::with_lists(vec![vec![], vec![esp(B)]]);
        hw.answers.push_back(Some("yes".into()));
        let path = backup_path(Path::new("/b"), mac(B));
        hw.files.insert(path.clone(), vec![1]);
        let err = run_adopt(&mut hw, &opts(true), ImageSource::Image(image())).unwrap_err();
        assert!(err.to_string().contains("--force"), "{err}");
        assert!(hw.serial_writes.is_empty(), "must not flash without a fresh backup");
        assert_eq!(hw.files[&path], [1]);

        let mut hw = Fake::with_lists(vec![vec![], vec![esp(B)]]);
        hw.files.insert(path.clone(), vec![1]);
        hw.rtt.push_back(true);
        let o = AdoptOptions { yes: true, force: true, ..opts(true) };
        assert!(run_adopt(&mut hw, &o, ImageSource::Image(image())).is_ok());
        assert_eq!(hw.files[&path].len(), BACKUP_LEN as usize);
    }

    #[test]
    fn adopt_asks_for_one_replug_when_stuck() {
        // Lists: start, new badge, then (replug wait) present, gone, back.
        let mut hw = Fake::with_lists(vec![vec![], vec![esp(B)], vec![esp(B)], vec![], vec![esp(B)]]);
        hw.boot_output = b"boot:0x0 (DOWNLOAD(USB/UART0))\r\nwaiting for download\r\n".to_vec();
        hw.rtt.extend([false, true]);
        let o = AdoptOptions { yes: true, ..opts(true) };
        let out = run_adopt(&mut hw, &o, ImageSource::Image(image())).unwrap();
        assert!(matches!(out, AdoptOutcome::Adopted { replugged: true, .. }), "{out:?}");
        let stuck = |l: &String| l.contains("waiting for a download") && l.contains("plug it back in");
        assert!(hw.log.iter().any(stuck), "{:?}", hw.log);
    }

    #[test]
    fn adopt_fails_if_still_not_running_after_replug() {
        let mut hw = Fake::with_lists(vec![vec![], vec![esp(B)], vec![], vec![esp(B)]]);
        let o = AdoptOptions { yes: true, ..opts(false) };
        let err = run_adopt(&mut hw, &o, ImageSource::Image(image())).unwrap_err();
        assert!(err.to_string().contains("still does not run"), "{err}");
        assert!(hw.log.iter().any(|l| l.contains("no firmware RTT block")), "{:?}", hw.log);
    }

    #[test]
    fn replug_wait_is_bounded() {
        let mut hw = Fake::with_lists(vec![vec![esp(B)]]);
        let err = wait_replug(&mut hw, mac(B)).unwrap_err();
        assert!(err.to_string().contains("not replugged"), "{err}");
        assert_eq!(hw.polls, REPLUG_MAX_POLLS as usize);
    }

    #[test]
    fn adopt_stops_on_ctrl_c() {
        let mut hw = Fake::with_lists(vec![vec![esp(A)]]);
        hw.stop_after_polls = Some(3);
        let out = run_adopt(&mut hw, &opts(true), ImageSource::Image(image())).unwrap();
        assert_eq!(out, AdoptOutcome::Stopped);
    }

    #[test]
    fn adopt_clones_from_another_badge() {
        let mut src = vec![0xFF; 0x80_0000];
        src[0] = 0xE9;
        src[0x8000] = 0xAA;
        src[0x9000] = 0x42; // nvs: must not be copied
        src[0x10000] = 0xE9;
        src[0x10001] = 0x05;
        src[0x200000] = 0x77; // conag: must not be copied
        let mut hw = Fake::with_lists(vec![vec![esp(A)], vec![esp(A), esp(B)]]);
        hw.flash.insert(mac(A), src);
        hw.rtt.push_back(true);
        let o = AdoptOptions { yes: true, ..opts(false) };
        let out = run_adopt(&mut hw, &o, ImageSource::Badge(mac(A))).unwrap();
        assert!(matches!(out, AdoptOutcome::Adopted { mac: m, .. } if m == mac(B)));
        assert_eq!(hw.reads, [(mac(A), 0, 0x8000), (mac(A), 0x8000, 0x1000), (mac(A), 0x10000, 0x1F_0000)]);
        let (target, written) = &hw.jtag_writes[0];
        assert_eq!(*target, mac(B));
        let addrs: Vec<u32> = written.segments().iter().map(|s| s.addr).collect();
        assert_eq!(addrs, [0, 0x8000, 0x10000]);
        assert_eq!(written.segments()[2].data, [0xE9, 0x05, 0xFF, 0xFF]);
        assert!(written.end() <= u64::from(FLASH_IMAGE_END));
    }

    #[test]
    fn source_badge_replug_is_not_a_target() {
        let mut hw = Fake::with_lists(vec![vec![esp(A)], vec![], vec![esp(A)], vec![esp(A), esp(B)]]);
        let mut src = vec![0xFF; 0x80_0000];
        src[0] = 0xE9;
        src[0x8000] = 0xAA;
        src[0x10000] = 0xE9;
        hw.flash.insert(mac(A), src);
        hw.rtt.push_back(true);
        let o = AdoptOptions { yes: true, ..opts(false) };
        let out = run_adopt(&mut hw, &o, ImageSource::Badge(mac(A))).unwrap();
        assert!(matches!(out, AdoptOutcome::Adopted { mac: m, .. } if m == mac(B)));
    }

    #[test]
    fn restore_writes_backup_at_zero() {
        let mut hw = Fake::default();
        run_restore(&mut hw, mac(A), vec![0x5A; BACKUP_LEN as usize]).unwrap();
        let (target, image) = &hw.serial_writes[0];
        assert_eq!(*target, mac(A));
        assert_eq!(image.segments()[0].addr, 0);
        assert_eq!(image.end(), u64::from(BACKUP_LEN));
        let err = run_restore(&mut hw, mac(A), vec![0; 0x80_0000]).unwrap_err();
        assert!(err.to_string().contains("exactly"), "{err}");
        assert_eq!(hw.serial_writes.len(), 1);
    }
}
