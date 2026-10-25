//! `badgectl sim`: simulated badges on the PC.
//!
//! The badge's functional core runs as the `badge-sim` wasm32 module (one
//! wasmi instance per badge, [`guest`]); the [`world`] steps them on a
//! shared virtual clock and connects them through a middleware bus
//! ([`bus::LocalBus`] in-process, or Zenoh with the `zenoh` feature,
//! `zenoh_net`), so Chi trades and LED sync work between simulated badges
//! exactly as between real ones. Each badge can run a wasm payload on
//! middleware's `wasm-host` ("mw" ABI), with a simulated payload partition
//! that follows the same upload commands as the firmware's ([`apps`]).
//! [`link::SimLink`] implements the RTT
//! `Link`, so `badgectl --sim <n> <request>` uses the normal request path.
//! [`script`] parses scenarios, [`png`] writes screens.
//!
//! This file is the imperative shell of `badgectl sim`: it reads the script
//! and app files, picks the bus, runs the loop (optionally paced to wall
//! time), prints messages and writes PNGs. `--app FILE.wasm` installs and
//! starts the payload on every badge through the same requests as `badgectl
//! wasm load` plus `wasm start` ([`app_msgs`]).

pub mod apps;
pub mod bus;
pub mod guest;
pub mod link;
pub mod png;
pub mod script;
pub mod world;
#[cfg(feature = "zenoh")]
pub mod zenoh_net;

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use anyhow::{Context, Result, anyhow, bail};
use badge_proto::{BadgeMsg_, GetWasm, HostMsg, HostMsg_::Cmd, WasmStart};

use crate::request::RequestError;

use crate::cli::{RequestCmd, SimArgs};
use crate::{format, request};
use bus::{LocalBus, Network};
use guest::SimModule;
use world::{World, WorldConfig};

/// The world parameters of `badgectl sim`.
pub fn world_config(args: &SimArgs) -> WorldConfig {
    WorldConfig {
        badges: args.badges,
        mac_base: args.mac_base,
        stagger_ms: args.stagger_ms,
        rssi: args.rssi,
        ..WorldConfig::default()
    }
}

/// Compiles the badge-sim module (`path`, else the embedded one).
pub fn load_module(path: Option<&Path>) -> Result<SimModule> {
    match path {
        Some(p) => {
            let bytes = std::fs::read(p).with_context(|| format!("reading {}", p.display()))?;
            Ok(SimModule::new(Some(&bytes))?)
        }
        None => Ok(SimModule::new(None)?),
    }
}

/// `cmds` as host messages with consecutive sequence numbers from `seq`
/// (advanced past them).
fn numbered(cmds: Vec<Cmd>, seq: &mut u32) -> Vec<HostMsg> {
    cmds.into_iter()
        .map(|cmd| {
            let s = *seq;
            *seq = request::next_seq(s);
            HostMsg { seq: s, cmd: Some(cmd) }
        })
        .collect()
}

/// The upload of `module` (`WasmBegin`, `WasmChunk`s, `WasmCommit`;
/// `request::wasm_plan`) followed by `then`.
fn upload_then(module: &[u8], then: Cmd) -> Result<Vec<Cmd>, RequestError> {
    let mut cmds = request::wasm_plan(module)?;
    cmds.push(then);
    Ok(cmds)
}

/// The host messages for one request: one, or an upload (`wasm load`:
/// begin, chunks, commit, then `GetWasm`). Advances `seq`.
pub fn host_msgs(req: &RequestCmd, seq: &mut u32) -> Result<Vec<HostMsg>> {
    if let Some(path) = request::wasm_upload(req) {
        let module = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
        return Ok(numbered(upload_then(&module, Cmd::GetWasm(GetWasm {}))?, seq));
    }
    let msg = request::host_msg(req, *seq)?;
    *seq = request::next_seq(*seq);
    Ok(vec![msg])
}

/// The upload of `module` followed by `WasmStart` (`--app`).
pub fn app_msgs(module: &[u8], seq: &mut u32) -> Result<Vec<HostMsg>> {
    Ok(numbered(upload_then(module, Cmd::WasmStart(WasmStart {}))?, seq))
}

/// One printed line for a message of badge `badge` at time `t`.
pub fn message_line(badge: usize, id: &str, t: u64, kind: &BadgeMsg_::Kind, json: bool) -> String {
    match (kind, json) {
        (BadgeMsg_::Kind::Event(e), false) => format!("[{t:>7} ms] badge {badge} {id} {}", format::event_text(e)),
        (BadgeMsg_::Kind::Response(r), false) => {
            format!("[{t:>7} ms] badge {badge} {id} seq {}: {}", r.seq, format::response_text(r).replace('\n', "\n    "))
        }
        (BadgeMsg_::Kind::Event(e), true) => with_badge(format::event_json(e), badge, id, t),
        (BadgeMsg_::Kind::Response(r), true) => with_badge(format::response_json(r), badge, id, t),
    }
}

fn with_badge(mut v: serde_json::Value, badge: usize, id: &str, t: u64) -> String {
    if let Some(o) = v.as_object_mut() {
        o.insert("badge".into(), badge.into());
        o.insert("id".into(), id.into());
        o.insert("t_ms".into(), t.into());
    }
    v.to_string()
}

/// A resolved script: requests waiting for their time.
pub struct Scenario {
    steps: Vec<(u64, usize, RequestCmd)>,
    done: Vec<bool>,
}

impl Scenario {
    /// Resolves `steps` against the world's badges (`@N`, badge indices).
    pub fn new<N: Network>(steps: &[script::Step], world: &World<N>) -> Result<Self> {
        let ids = (0..world.len()).map(|i| world.node(i).map(world::Node::peer_id)).collect::<Result<Vec<_>, _>>()?;
        let steps = steps
            .iter()
            .map(|s| {
                if s.badge >= world.len() {
                    return Err(anyhow!("script line {}: no badge {}", s.line, s.badge));
                }
                Ok((s.at_ms, s.badge, script::resolve(s, &ids)?))
            })
            .collect::<Result<Vec<_>>>()?;
        let done = vec![false; steps.len()];
        Ok(Self { steps, done })
    }

    /// Sends every request whose time has come and whose badge has booted
    /// (in script order); later ones wait.
    pub fn run_due<N: Network>(&mut self, world: &mut World<N>, seq: &mut u32) -> Result<()> {
        let now = world.now_ms();
        for ((at, badge, req), done) in self.steps.iter().zip(self.done.iter_mut()) {
            if *done || *at > now || !world.node(*badge)?.booted() {
                continue;
            }
            *done = true;
            for msg in host_msgs(req, seq)? {
                world.send_host(*badge, &msg)?;
            }
        }
        Ok(())
    }

    /// True once every step ran.
    pub fn finished(&self) -> bool {
        self.done.iter().all(|d| *d)
    }
}

/// Everything `run_world` needs besides the world.
struct RunPlan<'a> {
    args: &'a SimArgs,
    json: bool,
    steps: Vec<script::Step>,
    app: Option<Vec<u8>>,
    stop: &'a AtomicBool,
}

/// Runs `badgectl sim`.
pub fn run(args: &SimArgs, sim_wasm: Option<&Path>, json: bool, stop: &AtomicBool) -> Result<()> {
    if args.badges == 0 {
        bail!("--badges must be at least 1");
    }
    let module = load_module(sim_wasm)?;
    let steps = match &args.script {
        Some(p) => script::parse(&std::fs::read_to_string(p).with_context(|| format!("reading {}", p.display()))?)?,
        None => Vec::new(),
    };
    let app = match &args.app {
        Some(p) => Some(std::fs::read(p).with_context(|| format!("reading {}", p.display()))?),
        None => None,
    };
    if let Some(dir) = &args.png {
        std::fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
    }
    let plan = RunPlan { args, json, steps, app, stop };
    let cfg = world_config(args);
    if args.zenoh.is_empty() && args.zenoh_listen.is_empty() {
        return run_world(World::new(&module, cfg, LocalBus::new())?, plan);
    }
    run_zenoh(&module, cfg, plan)
}

#[cfg(feature = "zenoh")]
fn run_zenoh(module: &SimModule, cfg: WorldConfig, plan: RunPlan<'_>) -> Result<()> {
    let net = zenoh_net::ZenohNet::open(&plan.args.zenoh_listen, &plan.args.zenoh)?;
    run_world(World::new(module, cfg, net)?, plan)
}

#[cfg(not(feature = "zenoh"))]
fn run_zenoh(_module: &SimModule, _cfg: WorldConfig, _plan: RunPlan<'_>) -> Result<()> {
    Err(anyhow!("--zenoh needs badgectl built with `--features zenoh`"))
}

fn png_path(dir: &Path, badge: usize, t: u64) -> PathBuf {
    dir.join(format!("badge{badge}-{t:08}.png"))
}

fn run_world<N: Network>(mut world: World<N>, plan: RunPlan<'_>) -> Result<()> {
    let args = plan.args;
    let ids: Vec<_> = (0..world.len()).map(|i| world.node(i).map(world::Node::peer_id)).collect::<Result<_, _>>()?;
    let mut scenario = Scenario::new(&plan.steps, &world)?;
    let mut seq = 1u32;
    let mut app_started = vec![false; world.len()];
    let mut last_png = vec![None::<u64>; world.len()];
    let start = Instant::now();
    eprintln!(
        "simulating {} badge(s): {}{}",
        world.len(),
        ids.iter().map(|i| i.as_str()).collect::<Vec<_>>().join(" "),
        if args.duration_ms == 0 { ", Ctrl-C to stop".to_string() } else { format!(" for {} ms", args.duration_ms) }
    );
    while args.duration_ms == 0 || world.now_ms() < args.duration_ms {
        if plan.stop.load(Ordering::SeqCst) {
            break;
        }
        let now = world.now_ms();
        scenario.run_due(&mut world, &mut seq)?;
        world.step()?;
        for (i, started) in app_started.iter_mut().enumerate() {
            if let Some(module) = &plan.app
                && !*started
                && world.node(i)?.booted()
            {
                *started = true;
                for msg in app_msgs(module, &mut seq)? {
                    world.send_host(i, &msg)?;
                }
            }
        }
        for (i, id) in ids.iter().enumerate() {
            for (t, kind) in world.take_messages(i)? {
                let show = args.events || matches!(kind, BadgeMsg_::Kind::Response(_));
                if show {
                    println!("{}", message_line(i, id.as_str(), t, &kind, plan.json));
                }
            }
            if let Some(dir) = &args.png {
                let due = last_png[i].is_none_or(|t| now >= t + args.png_every_ms);
                if due && let Some(fb) = world.take_frame(i)? {
                    last_png[i] = Some(now);
                    let path = png_path(dir, i, now);
                    std::fs::write(&path, png::framebuffer_png(&fb))
                        .with_context(|| format!("writing {}", path.display()))?;
                }
            }
        }
        if args.realtime {
            let target = start + Duration::from_millis(world.now_ms());
            if let Some(wait) = target.checked_duration_since(Instant::now()) {
                std::thread::sleep(wait);
            }
        }
    }
    if !scenario.finished() {
        eprintln!("warning: the run ended before every script step ran");
    }
    for (i, id) in ids.iter().enumerate() {
        let dropped = world.dropped(i)?;
        if dropped > 0 {
            eprintln!("badge {i}: {dropped} output record(s) dropped (output region full)");
        }
        let node = world.node(i)?;
        let leds = node.leds().map_or_else(
            || "-".to_string(),
            |c| c.iter().map(|c| format!("{:06x}", c.packed())).collect::<Vec<_>>().join(" "),
        );
        eprintln!("badge {i} {}: leds {leds}; {}", id.as_str(), format::wasm_status_text(&node.wasm_status()));
        if let Some(dir) = &args.png {
            let path = png_path(dir, i, world.now_ms());
            std::fs::write(&path, png::framebuffer_png(&world.frame(i)?))
                .with_context(|| format!("writing {}", path.display()))?;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests;
