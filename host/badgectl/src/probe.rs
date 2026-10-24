//! probe-rs implementation of [`Link`] (imperative shell).
//!
//! Opens the selected debug probe, attaches to the running chip without
//! reset or halt (`Permissions::default()`, normal attach), and optionally
//! attaches to the RTT control block — at the ELF's `_SEGGER_RTT` address if
//! known, otherwise by scanning target RAM. Channels are found by name
//! (`proto-up`, `proto-down`) with numeric fallbacks (up 1, down 0); up
//! channel 0 carries defmt logs. Flashing uses the esp-idf image format from
//! `probe-rs-espressif`, as `probe-rs run` does for ESP chips, with the
//! firmware's partition table.

use std::path::Path;
use std::time::Duration;

use anyhow::{Context, Result, anyhow, bail};
use probe_rs::probe::{DebugProbeSelector, list::Lister};
use probe_rs::rtt::{Rtt, ScanRegion};
use probe_rs::{MemoryInterface, Permissions, Session};

use crate::session::Link;

/// Name of the protocol up channel.
pub const UP_NAME: &str = "proto-up";
/// Name of the protocol down channel.
pub const DOWN_NAME: &str = "proto-down";

/// Registers Espressif targets, probe driver and image format with probe-rs.
pub fn register_espressif() {
    probe_rs_espressif::register_plugin();
}

/// Opens the probe (`VID:PID[:SERIAL]`, default first) and attaches to
/// `chip` without resetting or halting it.
pub fn attach(chip: &str, probe: Option<&str>) -> Result<Session> {
    let lister = Lister::new();
    let probe = match probe {
        Some(sel) => {
            let selector: DebugProbeSelector =
                sel.parse().with_context(|| format!("invalid probe selector `{sel}`"))?;
            lister.open(selector).with_context(|| format!("cannot open probe {sel}"))?
        }
        None => {
            let probes = lister.list_all();
            let first = probes.first().ok_or_else(|| anyhow!("no debug probe found"))?;
            first.open().with_context(|| format!("cannot open probe {first:?}"))?
        }
    };
    probe
        .attach(chip, Permissions::default())
        .with_context(|| format!("cannot attach to {chip}"))
}

/// Flashes `elf` in esp-idf image format with the given partition table
/// CSV (espflash's default table when `None`), then resets the core so it
/// runs.
pub fn flash(session: &mut Session, elf: &Path, partition_table: Option<&Path>) -> Result<()> {
    let loader = probe_rs_espressif::image_format::IdfLoader {
        partition_table: partition_table.map(Path::to_path_buf),
        ..Default::default()
    };
    let mut options = probe_rs::flashing::DownloadOptions::default();
    options.verify = false;
    probe_rs::flashing::download_file_with_options(session, elf, loader, options)
        .with_context(|| format!("flashing {} failed", elf.display()))?;
    session.core(0)?.reset().context("reset after flashing failed")?;
    Ok(())
}

/// Resolved RTT channels.
struct RttPorts {
    rtt: Rtt,
    up: usize,
    down: usize,
    log: Option<usize>,
}

/// probe-rs backed [`Link`].
pub struct ProbeLink {
    session: Session,
    rtt: Option<RttPorts>,
}

impl ProbeLink {
    /// A link for raw word access only (peek / poke).
    pub fn words_only(session: Session) -> Self {
        Self { session, rtt: None }
    }

    /// Attaches to RTT, waiting up to `timeout` for the control block.
    ///
    /// `control_block` is the `_SEGGER_RTT` address from the ELF, if known.
    /// When `read_logs` is false the defmt channel is still drained (so a
    /// blocking-mode firmware cannot stall) but its bytes are dropped by the
    /// caller.
    pub fn with_rtt(mut session: Session, control_block: Option<u64>, timeout: Duration) -> Result<Self> {
        let region = control_block.map_or(ScanRegion::Ram, ScanRegion::Exact);
        let mut rtt = {
            let mut core = session.core(0)?;
            probe_rs::rtt::try_attach_to_rtt(&mut core, timeout, &region)
                .context("cannot attach to the RTT control block (is the firmware running?)")?
        };
        let up = rtt
            .up_channels()
            .iter()
            .position(|c| c.name() == Some(UP_NAME))
            .or_else(|| rtt.up_channels().iter().position(|c| c.number() == 1))
            .ok_or_else(|| anyhow!("RTT up channel `{UP_NAME}` not found"))?;
        let down = rtt
            .down_channels()
            .iter()
            .position(|c| c.name() == Some(DOWN_NAME))
            .or_else(|| rtt.down_channels().iter().position(|c| c.number() == 0))
            .ok_or_else(|| anyhow!("RTT down channel `{DOWN_NAME}` not found"))?;
        let log = rtt.up_channels().iter().position(|c| c.number() == 0).filter(|&i| i != up);
        Ok(Self { session, rtt: Some(RttPorts { rtt, up, down, log }) })
    }

    fn ports(&mut self) -> Result<(&mut Session, &mut RttPorts), probe_rs::Error> {
        match self.rtt.as_mut() {
            Some(p) => Ok((&mut self.session, p)),
            None => Err(probe_rs::Error::Other("RTT not attached".into())),
        }
    }
}

impl Link for ProbeLink {
    type Error = probe_rs::Error;

    fn write_down(&mut self, bytes: &[u8]) -> Result<usize, Self::Error> {
        let (session, p) = self.ports()?;
        let mut core = session.core(0)?;
        match p.rtt.down_channels().get_mut(p.down) {
            Some(ch) => ch.write(&mut core, bytes).map_err(rtt_err),
            None => Ok(0),
        }
    }

    fn read_up(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let (session, p) = self.ports()?;
        let mut core = session.core(0)?;
        match p.rtt.up_channels().get_mut(p.up) {
            Some(ch) => ch.read(&mut core, buf).map_err(rtt_err),
            None => Ok(0),
        }
    }

    fn read_log(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let (session, p) = self.ports()?;
        let Some(log) = p.log else { return Ok(0) };
        let mut core = session.core(0)?;
        match p.rtt.up_channels().get_mut(log) {
            Some(ch) => ch.read(&mut core, buf).map_err(rtt_err),
            None => Ok(0),
        }
    }

    fn read_words(&mut self, addr: u64, out: &mut [u32]) -> Result<(), Self::Error> {
        self.session.core(0)?.read_32(addr, out)
    }

    fn write_word(&mut self, addr: u64, value: u32) -> Result<(), Self::Error> {
        self.session.core(0)?.write_word_32(addr, value)
    }
}

fn rtt_err(e: probe_rs::rtt::Error) -> probe_rs::Error {
    probe_rs::Error::Other(format!("RTT: {e}"))
}

/// Fails early with a clear message if `path` is missing.
pub fn require_file(path: &Path) -> Result<()> {
    if !path.is_file() {
        bail!("{} does not exist", path.display());
    }
    Ok(())
}
