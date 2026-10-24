//! `badgectl`: host CLI for the Conagotchi discovery firmware.
//!
//! Functional core: [`color`], [`request`], [`format`](mod@format), [`peek`], [`elf`],
//! the [`session`] state machine and the [`adopt`] flow are pure and
//! unit-tested against in-memory fakes. Imperative shell: [`probe`]
//! (probe-rs attach, RTT, flashing), [`badge_io`] (probe-rs + espflash
//! hardware for `adopt` / `restore`), [`logs`] (defmt decoding and printing)
//! and this file (argument dispatch, Ctrl-C, exit codes).
//!
//! Exit codes: 0 success, 1 error response or failure, 2 timeout.

mod adopt;
mod badge_io;
mod cli;
mod color;
mod elf;
mod format;
mod logs;
mod peek;
mod probe;
mod request;
mod session;

use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use anyhow::{Context, Result, anyhow};
use clap::Parser;

use cli::{AdoptArgs, Cli, Command, RequestCmd};
use logs::{DefmtInfo, Printer};
use peek::ProbeBlock;
use probe::ProbeLink;
use session::{Conn, SessionError, SystemClock};

/// Set by the Ctrl-C handler; polled by streaming commands.
static STOP: AtomicBool = AtomicBool::new(false);

/// Default firmware ELF, relative to the repository root.
const DEFAULT_ELF: &str = "firmware/target/xtensa-esp32s3-none-elf/release/conagotchi-fw";

/// Default partition table, relative to the repository root.
const DEFAULT_PARTITIONS: &str = "firmware/partitions.csv";

/// Minimum RTT attach wait after flashing (the firmware must boot first).
const RTT_ATTACH_AFTER_FLASH_MS: u64 = 10_000;

/// The firmware image, if one was found.
struct Firmware {
    path: PathBuf,
    bytes: Vec<u8>,
    symbols: elf::Symbols,
}

fn main() -> ExitCode {
    match run(Cli::parse()) {
        Ok(code) => code,
        Err(e) => {
            eprintln!("error: {e:#}");
            ExitCode::from(1)
        }
    }
}

/// Candidate default ELF paths: repo root derived from this crate's
/// location, then the current directory.
fn default_elf_candidates() -> Vec<PathBuf> {
    let repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    vec![repo.join(DEFAULT_ELF), PathBuf::from(DEFAULT_ELF)]
}

/// Candidate default partition tables, like [`default_elf_candidates`].
fn default_partitions() -> Option<PathBuf> {
    let repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    [repo.join(DEFAULT_PARTITIONS), PathBuf::from(DEFAULT_PARTITIONS)].into_iter().find(|p| p.is_file())
}

fn load_firmware(cli: &Cli) -> Result<Option<Firmware>> {
    let path = match &cli.elf {
        Some(p) => {
            probe::require_file(p)?;
            p.clone()
        }
        None => match default_elf_candidates().into_iter().find(|p| p.is_file()) {
            Some(p) => p,
            None => return Ok(None),
        },
    };
    let path = path.canonicalize().unwrap_or(path);
    let bytes = std::fs::read(&path).with_context(|| format!("reading {}", path.display()))?;
    let symbols = elf::symbols_from_elf(&bytes).with_context(|| format!("symbols of {}", path.display()))?;
    Ok(Some(Firmware { path, bytes, symbols }))
}

fn run(cli: Cli) -> Result<ExitCode> {
    probe::register_espressif();
    let fw = load_firmware(&cli)?;
    match &cli.command {
        Command::Addr => {
            let fw = fw.ok_or_else(|| anyhow!("`addr` needs the firmware ELF (--elf)"))?;
            println!("{}", fw.path.display());
            println!("{}", format::addr_report(&cli.chip, fw.symbols.probe, fw.symbols.rtt));
            Ok(ExitCode::SUCCESS)
        }
        Command::Peek { addr } => {
            let base = probe_addr(*addr, fw.as_ref())?;
            let session = probe::attach(&cli.chip, cli.probe.as_deref())?;
            let mut conn = Conn::new(ProbeLink::words_only(session), SystemClock::default());
            let words = conn.read_block(base)?;
            print_block(&words, base, cli.json)?;
            Ok(ExitCode::SUCCESS)
        }
        Command::Poke { addr, cmd } => {
            let base = probe_addr(*addr, fw.as_ref())?;
            let session = probe::attach(&cli.chip, cli.probe.as_deref())?;
            let mut conn = Conn::new(ProbeLink::words_only(session), SystemClock::default());
            match conn.poke_mailbox(base, request::poke_words(cmd), cli.timeout) {
                Ok(words) => {
                    print_block(&words, base, cli.json)?;
                    Ok(ExitCode::SUCCESS)
                }
                Err(e @ SessionError::MailboxTimeout { .. }) => {
                    eprintln!("error: {e}");
                    Ok(ExitCode::from(2))
                }
                Err(e) => Err(e.into()),
            }
        }
        Command::Flash => {
            let fw = fw.ok_or_else(|| anyhow!("`flash` needs the firmware ELF (--elf)"))?;
            let mut session = probe::attach(&cli.chip, cli.probe.as_deref())?;
            let partitions = default_partitions();
            match &partitions {
                Some(p) => eprintln!("flashing {} with {} ...", fw.path.display(), p.display()),
                None => eprintln!(
                    "flashing {} ... (warning: {DEFAULT_PARTITIONS} not found, using espflash's default table)",
                    fw.path.display()
                ),
            }
            probe::flash(&mut session, &fw.path, partitions.as_deref())?;
            eprintln!("flashed; target reset, attaching to RTT");
            let wait = cli.timeout.max(RTT_ATTACH_AFTER_FLASH_MS);
            stream(&cli, Some(&fw), session, wait)
        }
        Command::Monitor | Command::Events => {
            let session = probe::attach(&cli.chip, cli.probe.as_deref())?;
            stream(&cli, fw.as_ref(), session, cli.timeout)
        }
        Command::Adopt(args) => run_adopt(&cli, fw.as_ref(), args),
        Command::Restore { backup } => run_restore(&cli, backup),
        Command::Request(req) => {
            let session = probe::attach(&cli.chip, cli.probe.as_deref())?;
            if request::is_contacts_all(req) {
                return all_contacts(&cli, fw.as_ref(), session);
            }
            send_request(&cli, fw.as_ref(), session, req)
        }
    }
}

/// Resolves the image `adopt` installs.
fn adopt_source(fw: Option<&Firmware>, args: &AdoptArgs) -> Result<(adopt::ImageSource, Option<u64>)> {
    if let Some(serial) = &args.from_badge {
        return Ok((adopt::ImageSource::Badge(adopt::mac_from_selector(serial)?), None));
    }
    let partitions = || -> Result<PathBuf> {
        match &args.partitions {
            Some(p) => {
                probe::require_file(p)?;
                Ok(p.clone())
            }
            None => default_partitions()
                .ok_or_else(|| anyhow!("partition table {DEFAULT_PARTITIONS} not found; pass --partitions")),
        }
    };
    match &args.source {
        Some(path) if path.extension().is_some_and(|e| e.eq_ignore_ascii_case("bin")) => {
            let bytes = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
            Ok((adopt::ImageSource::Image(adopt::FlashImage::from_merged(bytes)?), None))
        }
        Some(path) => {
            let bytes = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
            let rtt = elf::symbols_from_elf(&bytes).ok().and_then(|s| s.rtt);
            Ok((adopt::ImageSource::Image(badge_io::elf_image(&bytes, &partitions()?)?), rtt))
        }
        None => {
            let fw = fw.ok_or_else(|| anyhow!("no firmware ELF found: build it, or pass --elf / --source"))?;
            eprintln!("image from {}", fw.path.display());
            let image = badge_io::elf_image(&fw.bytes, &partitions()?)?;
            Ok((adopt::ImageSource::Image(image), fw.symbols.rtt))
        }
    }
}

fn run_adopt(cli: &Cli, fw: Option<&Firmware>, args: &AdoptArgs) -> Result<ExitCode> {
    ctrlc::set_handler(|| STOP.store(true, Ordering::SeqCst)).context("installing Ctrl-C handler")?;
    let (source, rtt) = adopt_source(fw, args)?;
    let backup_dir = adopt::backup_dir(
        std::env::var_os("XDG_DATA_HOME").as_deref(),
        std::env::var_os("HOME").as_deref(),
    )?;
    let opts = adopt::AdoptOptions { yes: args.yes, backup: !args.no_backup, force: args.force, backup_dir };
    let mut hw = badge_io::RealHardware::new(&cli.chip, rtt, &STOP);
    match adopt::run_adopt(&mut hw, &opts, source)? {
        adopt::AdoptOutcome::Adopted { backup, .. } => {
            if let Some(path) = backup {
                eprintln!("backup: {} (restore with `badgectl restore {}`)", path.display(), path.display());
            }
            Ok(ExitCode::SUCCESS)
        }
        adopt::AdoptOutcome::Declined(_) | adopt::AdoptOutcome::Stopped => Ok(ExitCode::from(1)),
    }
}

fn run_restore(cli: &Cli, backup: &Path) -> Result<ExitCode> {
    let bytes = std::fs::read(backup).with_context(|| format!("reading {}", backup.display()))?;
    let mut hw = badge_io::RealHardware::new(&cli.chip, None, &STOP);
    let wanted = cli.probe.as_deref().map(adopt::mac_from_selector).transpose()?;
    let probes = adopt::Hardware::probes(&mut hw)?;
    let mac = adopt::pick_badge(&probes, wanted)?;
    adopt::run_restore(&mut hw, mac, bytes)?;
    Ok(ExitCode::SUCCESS)
}

/// `contacts --all`: requests pages until the badge's total is reached.
fn all_contacts(cli: &Cli, fw: Option<&Firmware>, session: probe_rs::Session) -> Result<ExitCode> {
    use badge_proto::Response_::Body;
    let mut seq = request::initial_seq(entropy());
    let link = rtt_link(fw, session, cli.timeout)?;
    let defmt = defmt_info(fw);
    let mut printer = Printer::new(defmt.as_ref(), cli.json);
    let mut conn = Conn::new(link, SystemClock::default());
    let mut contacts = Vec::new();
    let mut total = 0;
    let mut offset = 0;
    for _ in 0..request::MAX_CONTACT_PAGES {
        let req = request::contacts_at(offset);
        let msg = request::host_msg(&req, seq)?;
        seq = request::next_seq(seq);
        let resp = match conn.request(&msg, request::timeout_ms(&req, cli.timeout), &mut printer) {
            Ok(r) => r,
            Err(e @ (SessionError::ResponseTimeout { .. } | SessionError::WriteTimeout { .. })) => {
                eprintln!("error: {e}");
                return Ok(ExitCode::from(2));
            }
            Err(e) => return Err(e.into()),
        };
        let Some(Body::Contacts(page)) = &resp.body else {
            print_response(cli, &resp);
            return Ok(ExitCode::from(1));
        };
        total = page.total;
        contacts.extend(page.contacts.iter().cloned());
        match request::next_contacts_offset(page) {
            Some(next) => offset = next,
            None => break,
        }
    }
    if cli.json {
        println!("{}", serde_json::json!({ "kind": "response", "body": format::contacts_json(&contacts, 0, total) }));
    } else {
        println!("{}", format::contacts_text(&contacts, 0, total));
    }
    Ok(ExitCode::SUCCESS)
}

fn print_response(cli: &Cli, resp: &badge_proto::Response) {
    if cli.json {
        println!("{}", format::response_json(resp));
    } else {
        println!("{}", format::response_text(resp));
    }
}

fn probe_addr(addr: Option<u64>, fw: Option<&Firmware>) -> Result<u64> {
    addr.or_else(|| fw.and_then(|f| f.symbols.probe))
        .ok_or_else(|| anyhow!("PROBE address unknown: pass --elf with the firmware or --addr 0x..."))
}

fn print_block(words: &[u32], base: u64, json: bool) -> Result<()> {
    let block = ProbeBlock::from_words(words)?;
    if json {
        println!("{}", serde_json::json!({ "addr": base, "probe": block.to_json() }));
    } else {
        println!("PROBE @ {base:#010x}");
        println!("{}", block.render());
    }
    if let Some(warning) = block.warning() {
        eprintln!("{warning}");
    }
    Ok(())
}

fn rtt_link(fw: Option<&Firmware>, session: probe_rs::Session, wait_ms: u64) -> Result<ProbeLink> {
    let rtt_addr = fw.and_then(|f| f.symbols.rtt);
    ProbeLink::with_rtt(session, rtt_addr, Duration::from_millis(wait_ms))
}

fn defmt_info(fw: Option<&Firmware>) -> Option<DefmtInfo> {
    let fw = fw?;
    match DefmtInfo::from_elf(&fw.bytes) {
        Ok(Some(info)) => Some(info),
        Ok(None) => {
            eprintln!("warning: {} has no defmt table; logs are not shown", fw.path.display());
            None
        }
        Err(e) => {
            eprintln!("warning: cannot read defmt table: {e:#}; logs are not shown");
            None
        }
    }
}

fn stream(cli: &Cli, fw: Option<&Firmware>, session: probe_rs::Session, wait_ms: u64) -> Result<ExitCode> {
    ctrlc::set_handler(|| STOP.store(true, Ordering::SeqCst)).context("installing Ctrl-C handler")?;
    let link = rtt_link(fw, session, wait_ms)?;
    let defmt = defmt_info(fw);
    let mut printer = Printer::new(defmt.as_ref(), cli.json);
    let mut conn = Conn::new(link, SystemClock::default());
    eprintln!("streaming; Ctrl-C to stop");
    conn.stream(&mut printer, || STOP.load(Ordering::SeqCst))?;
    Ok(ExitCode::SUCCESS)
}

fn send_request(cli: &Cli, fw: Option<&Firmware>, session: probe_rs::Session, req: &RequestCmd) -> Result<ExitCode> {
    let seq = request::initial_seq(entropy());
    let msg = request::host_msg(req, seq)?;
    let link = rtt_link(fw, session, cli.timeout)?;
    let defmt = defmt_info(fw);
    let mut printer = Printer::new(defmt.as_ref(), cli.json);
    let mut conn = Conn::new(link, SystemClock::default());
    match conn.request(&msg, request::timeout_ms(req, cli.timeout), &mut printer) {
        Ok(resp) => {
            print_response(cli, &resp);
            if let Some(hint) = request::hint(req, &resp) {
                eprintln!("{hint}");
            }
            Ok(if format::is_error(&resp) { ExitCode::from(1) } else { ExitCode::SUCCESS })
        }
        Err(e @ (SessionError::ResponseTimeout { .. } | SessionError::WriteTimeout { .. })) => {
            eprintln!("error: {e}");
            Ok(ExitCode::from(2))
        }
        Err(e) => Err(e.into()),
    }
}

/// Process-specific entropy for the first sequence number.
fn entropy() -> u64 {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| u64::try_from(d.as_nanos() & u128::from(u64::MAX)).unwrap_or(0));
    nanos ^ (u64::from(std::process::id()) << 20)
}
