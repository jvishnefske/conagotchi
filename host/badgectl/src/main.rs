//! `badgectl`: host CLI for the Conagotchi discovery firmware.
//!
//! Functional core: [`color`], [`request`], [`format`](mod@format), [`peek`], [`elf`],
//! [`stack`], [`rtt_check`], the [`session`] state machine and the [`adopt`]
//! flow are pure and unit-tested against in-memory fakes. Imperative shell: [`probe`]
//! (probe-rs attach, RTT, flashing), [`badge_io`] (probe-rs + espflash
//! hardware for `adopt` / `restore`), [`logs`] (defmt decoding and printing)
//! and this file (argument dispatch, Ctrl-C, exit codes).
//!
//! [`sim`] runs simulated badges (`badgectl sim`); with `--sim <n>` every
//! request command talks to badge `n` of a fresh simulation through the
//! same [`session::Conn`] request path as RTT.
//!
//! `badgectl stack paint` / `stack report` measure the main stack's
//! high-water mark by painting it over the debug link ([`stack`] has the
//! method and the symbols used), and `badgectl rtt-check` validates the
//! `_SEGGER_RTT` control block and the stack guard word ([`rtt_check`]).
//! `scripts/stack-watermark.sh` wraps them for soak runs.
//!
//! Exit codes: 0 success, 1 error response or failure (also: stack not
//! painted, stack guard changed, RTT block corrupt), 2 timeout.

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
mod rtt_check;
mod session;
mod sim;
mod stack;

use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use anyhow::{Context, Result, anyhow};
use clap::Parser;

use cli::{AdoptArgs, Cli, Command, PaintArgs, RequestCmd, StackCmd};
use logs::{DefmtInfo, Printer};
use peek::ProbeBlock;
use probe::ProbeLink;
use session::{Clock, Conn, Link, SessionError, SystemClock};

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
    if let Some(badge) = cli.sim {
        let Command::Request(req) = &cli.command else {
            return Err(anyhow!("--sim works with request commands; use `badgectl sim` to run a simulation"));
        };
        return sim_request(&cli, badge, req);
    }
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
        Command::Stack { op } => {
            let fw = fw.ok_or_else(|| anyhow!("`stack` needs the firmware ELF (--elf)"))?;
            let regions = stack_regions(&fw)?;
            let mut session = probe::attach(&cli.chip, cli.probe.as_deref())?;
            warn_if_other_build(&mut session, &fw);
            match op {
                StackCmd::Paint(args) => stack_paint(&cli, &mut session, &regions, args),
                StackCmd::Report { pattern, painted_top } => {
                    stack_report(&cli, &mut session, &regions, *pattern, *painted_top)
                }
            }
        }
        Command::RttCheck { addr } => {
            let addr = addr
                .or_else(|| fw.as_ref().and_then(|f| f.symbols.rtt))
                .ok_or_else(|| anyhow!("_SEGGER_RTT address unknown: pass --elf with the firmware or --addr 0x..."))?;
            let regions = match &fw {
                Some(f) => stack_regions(f).ok(),
                None => None,
            };
            let mut session = probe::attach(&cli.chip, cli.probe.as_deref())?;
            if let Some(f) = &fw {
                warn_if_other_build(&mut session, f);
            }
            rtt_check_cmd(&cli, &mut session, addr, regions.as_deref())
        }
        Command::Adopt(args) => run_adopt(&cli, fw.as_ref(), args),
        Command::Restore { backup } => run_restore(&cli, backup),
        Command::Sim(args) => {
            ctrlc::set_handler(|| STOP.store(true, Ordering::SeqCst)).context("installing Ctrl-C handler")?;
            sim::run(args, cli.sim_wasm.as_deref(), cli.json, &STOP)?;
            Ok(ExitCode::SUCCESS)
        }
        Command::Request(req) => {
            let session = probe::attach(&cli.chip, cli.probe.as_deref())?;
            let link = rtt_link(fw.as_ref(), session, cli.timeout)?;
            let defmt = defmt_info(fw.as_ref());
            let mut printer = Printer::new(defmt.as_ref(), cli.json);
            let mut conn = Conn::new(link, SystemClock::default());
            request_on(&cli, &mut conn, &mut printer, req)
        }
    }
}

/// `--sim <badge> <request>`: a fresh simulation, warmed up, then the
/// request over a [`sim::link::SimLink`].
fn sim_request(cli: &Cli, badge: usize, req: &RequestCmd) -> Result<ExitCode> {
    if badge >= cli.sim_badges {
        return Err(anyhow!("--sim {badge}: the simulation has {} badge(s) (--sim-badges)", cli.sim_badges));
    }
    let module = sim::load_module(cli.sim_wasm.as_deref())?;
    let cfg = sim::world::WorldConfig { badges: cli.sim_badges, ..sim::world::WorldConfig::default() };
    let tick = cfg.tick_ms;
    let mut world = sim::world::World::new(&module, cfg, sim::bus::LocalBus::new())?;
    world.run_for(cli.sim_warmup_ms)?;
    let id = world.node(badge)?.peer_id();
    eprintln!(
        "simulated badge {badge} ({}) of {}, after {} ms",
        id.as_str(),
        cli.sim_badges,
        world.now_ms()
    );
    let mut printer = Printer::new(None, cli.json);
    let mut conn = Conn::new(sim::link::SimLink::new(&mut world, badge)?, sim::link::StepClock::new(tick));
    request_on(cli, &mut conn, &mut printer, req)
}

/// Maps a request outcome to an exit code (timeouts are 2).
fn finish<E: std::error::Error + Send + Sync + 'static>(
    result: Result<badge_proto::Response, SessionError<E>>,
) -> Result<Result<badge_proto::Response, ExitCode>> {
    match result {
        Ok(r) => Ok(Ok(r)),
        Err(e @ (SessionError::ResponseTimeout { .. } | SessionError::WriteTimeout { .. })) => {
            eprintln!("error: {e}");
            Ok(Err(ExitCode::from(2)))
        }
        Err(e) => Err(e.into()),
    }
}

/// Runs one request command over any link: a single request, the paged
/// `contacts --all`, or the chunked `wasm load`.
fn request_on<L: Link, C: Clock>(
    cli: &Cli,
    conn: &mut Conn<L, C>,
    printer: &mut Printer<'_>,
    req: &RequestCmd,
) -> Result<ExitCode> {
    let seq = request::initial_seq(entropy());
    if request::is_contacts_all(req) {
        return all_contacts(cli, conn, printer, seq);
    }
    if let Some(path) = request::wasm_upload(req) {
        return wasm_upload(cli, conn, printer, path, seq);
    }
    if let Some(samples) = request::adc_scan(req) {
        return adc_scan(cli, conn, printer, samples, seq);
    }
    let msg = request::host_msg(req, seq)?;
    let resp = match finish(conn.request(&msg, request::timeout_ms(req, cli.timeout), printer))? {
        Ok(r) => r,
        Err(code) => return Ok(code),
    };
    print_response(cli, &resp);
    if let Some(hint) = request::hint(req, &resp) {
        eprintln!("{hint}");
    }
    Ok(if format::is_error(&resp) { ExitCode::from(1) } else { ExitCode::SUCCESS })
}

/// `wasm load <file>`: `WasmBegin`, every `WasmChunk` in order (a progress
/// line on stderr) and `WasmCommit`, stopping at the first error response;
/// then the payload status.
fn wasm_upload<L: Link, C: Clock>(
    cli: &Cli,
    conn: &mut Conn<L, C>,
    printer: &mut Printer<'_>,
    path: &Path,
    mut seq: u32,
) -> Result<ExitCode> {
    use badge_proto::HostMsg_::Cmd;
    let module = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
    let plan = request::wasm_plan(&module)?;
    let total = module.len();
    for cmd in plan {
        let progress = match &cmd {
            Cmd::WasmChunk(c) => Some(c.offset as usize + c.data.len()),
            _ => None,
        };
        let msg = badge_proto::HostMsg { seq, cmd: Some(cmd) };
        seq = request::next_seq(seq);
        let resp = match finish(conn.request(&msg, cli.timeout, printer))? {
            Ok(r) => r,
            Err(code) => return Ok(code),
        };
        if format::is_error(&resp) {
            eprintln!();
            print_response(cli, &resp);
            return Ok(ExitCode::from(1));
        }
        if let Some(end) = progress {
            eprint!("\rloading {}: {end}/{total} bytes", path.display());
        }
    }
    eprintln!();
    let status = badge_proto::HostMsg { seq, cmd: Some(Cmd::GetWasm(badge_proto::GetWasm {})) };
    let resp = match finish(conn.request(&status, cli.timeout, printer))? {
        Ok(r) => r,
        Err(code) => return Ok(code),
    };
    print_response(cli, &resp);
    Ok(if format::is_error(&resp) { ExitCode::from(1) } else { ExitCode::SUCCESS })
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
fn all_contacts<L: Link, C: Clock>(
    cli: &Cli,
    conn: &mut Conn<L, C>,
    printer: &mut Printer<'_>,
    mut seq: u32,
) -> Result<ExitCode> {
    use badge_proto::Response_::Body;
    let mut contacts = Vec::new();
    let mut total = 0;
    let mut offset = 0;
    for _ in 0..request::MAX_CONTACT_PAGES {
        let req = request::contacts_at(offset);
        let msg = request::host_msg(&req, seq)?;
        seq = request::next_seq(seq);
        let resp = match finish(conn.request(&msg, request::timeout_ms(&req, cli.timeout), printer))? {
            Ok(r) => r,
            Err(code) => return Ok(code),
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

/// `adc scan`: one `AdcRead` per ADC1 probe pin; stops at the first error.
fn adc_scan<L: Link, C: Clock>(
    cli: &Cli,
    conn: &mut Conn<L, C>,
    printer: &mut Printer<'_>,
    samples: u32,
    mut seq: u32,
) -> Result<ExitCode> {
    for pin in request::ADC_SCAN_PINS {
        let msg = badge_proto::HostMsg { seq, cmd: Some(request::adc_read(pin, samples)) };
        seq = request::next_seq(seq);
        let resp = match finish(conn.request(&msg, cli.timeout, printer))? {
            Ok(r) => r,
            Err(code) => return Ok(code),
        };
        print_response(cli, &resp);
        if format::is_error(&resp) {
            let req = RequestCmd::Adc { op: cli::AdcCmd::Scan { samples } };
            if let Some(hint) = request::hint(&req, &resp) {
                eprintln!("{hint}");
            }
            return Ok(ExitCode::from(1));
        }
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

/// Attempts at sampling an SP inside the main stack (esp-rtos may be
/// running a thread with a heap-allocated stack when the core halts).
const SP_ATTEMPTS: usize = 5;
/// Words written per probe-rs call while painting.
const PAINT_CHUNK_WORDS: usize = 1024;

/// Warns on stderr when the ELF's `PROBE` address does not hold the PROBE
/// magic: the badge then runs a different build, and every address taken
/// from the ELF (stack bounds, `_SEGGER_RTT`) is suspect.
fn warn_if_other_build(session: &mut probe_rs::Session, fw: &Firmware) {
    let Some(base) = fw.symbols.probe else { return };
    let addr = base + badge_core::probe_layout::MAGIC as u64 * 4;
    let mut w = [0u32; 1];
    match probe::read_words(session, addr, &mut w) {
        Ok(()) if w[0] == badge_core::probe_layout::MAGIC_VALUE => {}
        Ok(()) => eprintln!(
            "warning: no PROBE magic at {addr:#010x} ({:#010x}): the badge is probably running a different build \
             than {}; pass the flashed ELF with --elf",
            w[0],
            fw.path.display()
        ),
        Err(e) => eprintln!("warning: cannot read the PROBE magic: {e:#}"),
    }
}

/// Stack regions of the firmware image (CPU 0 first).
fn stack_regions(fw: &Firmware) -> Result<Vec<stack::StackRegion>> {
    let table = elf::symbol_table(&fw.bytes).with_context(|| format!("symbols of {}", fw.path.display()))?;
    Ok(stack::regions_from_symbols(table.iter().map(|(n, a)| (n.as_str(), *a)))?)
}

/// Writes `plan` with `pattern`, bottom (coldest) first.
fn write_paint(session: &mut probe_rs::Session, plan: &stack::PaintPlan, pattern: u32) -> Result<()> {
    let chunk = vec![pattern; PAINT_CHUNK_WORDS];
    let mut addr = plan.start;
    let mut left = plan.words();
    while left > 0 {
        let n = left.min(PAINT_CHUNK_WORDS);
        probe::write_words(session, addr, chunk.get(..n).unwrap_or(&chunk))?;
        addr += n as u64 * 4;
        left -= n;
    }
    Ok(())
}

/// `stack paint`: paints CPU 0's main stack below SP minus the margin (or
/// below `--top`).
fn stack_paint(
    cli: &Cli,
    session: &mut probe_rs::Session,
    regions: &[stack::StackRegion],
    args: &PaintArgs,
) -> Result<ExitCode> {
    let region = regions.first().ok_or(stack::StackError::NoStackSymbols)?;
    let paint = |session: &mut probe_rs::Session, limit: u64| -> Result<stack::PaintPlan> {
        let plan = stack::plan_paint(region, limit)?;
        if !args.dry_run {
            write_paint(session, &plan, args.pattern)?;
        }
        Ok(plan)
    };
    let (plan, sp) = match (args.top, args.halt) {
        (Some(top), _) => (paint(session, top)?, None),
        (None, true) => probe::with_core_halted(session, |s, sp| {
            let limit = stack::paint_limit(region, sp, args.margin)?;
            Ok((paint(s, limit)?, Some(sp)))
        })?,
        (None, false) => {
            let (sp, ok) = probe::sample_sp(session, SP_ATTEMPTS, |sp| region.contains_sp(sp))?;
            if !ok {
                return Err(stack::StackError::SpOutside { sp, bottom: region.bottom(), top: region.top() }.into());
            }
            let limit = stack::paint_limit(region, sp, args.margin)?;
            (paint(session, limit)?, Some(sp))
        }
    };
    if cli.json {
        println!(
            "{}",
            serde_json::json!({
                "region": region.name(), "start": plan.start, "end": plan.end, "bytes": plan.len(),
                "pattern": args.pattern, "sp": sp, "margin": args.margin, "halted": args.halt,
                "dry_run": args.dry_run,
            })
        );
    } else {
        let mut line = stack::paint_summary(region, &plan, args.pattern, sp, args.margin);
        if args.dry_run {
            line = line.replacen("painted", "would paint", 1);
        }
        println!("{line}");
        println!("next: badgectl stack report --painted-top {:#010x}", plan.end);
    }
    Ok(ExitCode::SUCCESS)
}

/// Reads the guard word of `region`, if it has one.
fn read_guard(session: &mut probe_rs::Session, region: &stack::StackRegion) -> Result<stack::Guard> {
    let Some(addr) = region.guard() else { return Ok(stack::Guard::Absent) };
    let mut w = [0u32; 1];
    probe::read_words(session, addr, &mut w)?;
    Ok(stack::guard_state(Some(addr), Some(w[0])))
}

/// `stack report`: scans every region from its paintable bottom upward.
fn stack_report(
    cli: &Cli,
    session: &mut probe_rs::Session,
    regions: &[stack::StackRegion],
    pattern: u32,
    painted_top: Option<u64>,
) -> Result<ExitCode> {
    let mut alarm = false;
    for (i, region) in regions.iter().enumerate() {
        let base = region.paintable_bottom();
        let count = usize::try_from((region.top() - base) / 4)?;
        let mut words = vec![0u32; count];
        probe::read_words(session, base, &mut words)?;
        let report = stack::StackReport {
            region: region.clone(),
            watermark: stack::find_watermark(base, &words, pattern),
            pattern,
            // `stack paint` only paints CPU 0.
            painted_top: painted_top.filter(|_| i == 0),
            guard: read_guard(session, region)?,
        };
        alarm |= report.is_alarm();
        if cli.json {
            println!("{}", report.to_json());
        } else {
            println!("{}", report.render());
        }
    }
    Ok(if alarm { ExitCode::from(1) } else { ExitCode::SUCCESS })
}

/// `rtt-check`: reads and validates the control block, its channel names
/// and the CPU 0 stack guard word.
fn rtt_check_cmd(
    cli: &Cli,
    session: &mut probe_rs::Session,
    addr: u64,
    regions: Option<&[stack::StackRegion]>,
) -> Result<ExitCode> {
    let map = &rtt_check::ESP32S3;
    let len = usize::try_from(map.ram.end.saturating_sub(addr))?.clamp(rtt_check::HEADER_LEN, rtt_check::READ_LEN);
    let mut bytes = vec![0u8; len];
    probe::read_bytes(session, addr, &mut bytes)?;
    let block = rtt_check::ControlBlock::parse(&bytes)?;
    let mut read_name = |ptr: u32| -> Option<String> {
        let p = u64::from(ptr);
        if !(map.ram.contains(&p) || map.rodata.contains(&p)) {
            return None;
        }
        let mut buf = [0u8; rtt_check::NAME_MAX];
        probe::read_bytes(session, p, &mut buf).ok()?;
        rtt_check::c_name(&buf)
    };
    let names = rtt_check::Names {
        up: block.up.iter().map(|c| read_name(c.name_ptr)).collect(),
        down: block.down.iter().map(|c| read_name(c.name_ptr)).collect(),
    };
    let findings = rtt_check::check(&block, addr, map, Some(&rtt_check::FIRMWARE_LAYOUT), &names);
    let guard = match regions.and_then(|r| r.first()) {
        Some(region) => read_guard(session, region)?,
        None => stack::Guard::Absent,
    };
    let guard_bad = matches!(guard, stack::Guard::Changed { .. });
    if cli.json {
        let mut j = rtt_check::to_json(&block, addr, &names, &findings);
        j["stack_guard"] = match guard {
            stack::Guard::Absent => serde_json::Value::Null,
            stack::Guard::Intact(a) => serde_json::json!({ "addr": a, "ok": true }),
            stack::Guard::Changed { addr, value } => serde_json::json!({ "addr": addr, "ok": false, "value": value }),
        };
        println!("{j}");
    } else {
        println!("{}", rtt_check::render(&block, addr, &names, &findings));
        match guard {
            stack::Guard::Absent => println!("stack guard    unknown (no ELF or no __stack_chk_guard)"),
            stack::Guard::Intact(a) => println!("stack guard    {a:#010x} = {:#010x} (ok)", stack::GUARD_VALUE),
            stack::Guard::Changed { addr, value } => println!(
                "stack guard    {addr:#010x} = {value:#010x} (CHANGED, expected {:#010x})",
                stack::GUARD_VALUE
            ),
        }
    }
    Ok(if rtt_check::is_corrupt(&findings) || guard_bad { ExitCode::from(1) } else { ExitCode::SUCCESS })
}

/// Process-specific entropy for the first sequence number.
fn entropy() -> u64 {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| u64::try_from(d.as_nanos() & u128::from(u64::MAX)).unwrap_or(0));
    nanos ^ (u64::from(std::process::id()) << 20)
}
