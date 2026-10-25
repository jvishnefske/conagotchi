//! The simulated world: N badges, one bus, virtual time.
//!
//! [`World::step`] advances virtual time by one tick (20 ms by default) and
//! runs every badge in index order, so a run is fully deterministic:
//!
//! 1. A badge whose boot time has come is booted (`sim_init`); badge `i`
//!    boots at `i * stagger_ms`, so the badges' local clocks differ and LED
//!    sync has something to do.
//! 2. Each booted badge gets a tick at its local time (time since its own
//!    boot) and its wasm app, if one runs, gets one `tick`.
//! 3. A badge's current advert (its last `Beacon`) is re-published every
//!    `adv_interval_ms`, as the firmware's advertiser repeats it.
//! 4. Every badge drains its subscriptions.
//!
//! Effect records are routed as follows (see `badge_sim::wire`):
//!
//! - `Send` goes to the badge's message history ([`World::take_messages`])
//!   and, once a host link is attached ([`World::write_down`]), COBS-framed
//!   to its up stream ([`World::read_up`]), exactly what RTT carries;
//! - `Beacon` replaces the current advert, published on `keys::OZ` as the AD
//!   bytes; receivers get `RadioPacket{via: BLE, rssi, payload:
//!   oz_payload(ad)}`;
//! - `Sync` is published at once on `keys::SYNC` as `SyncBeacon` wire bytes;
//!   receivers get `SyncBeacon` with `rx_at` = their local time plus
//!   `latency_ms` (the firmware's assumed ESP-NOW latency);
//! - `Wasm` runs on the badge's [`AppSlot`] and is answered with
//!   `sim_input_wasm`; app publishes go to the bus and arrive at every
//!   badge whose app subscribed to the key;
//! - `Leds` and `Screen` update [`Node::leds`] and the PNG dirty flag;
//!   link-UART frames, indicators and contact storage have no simulated
//!   hardware and are dropped.
//!
//! Every payload on the bus carries the sender's MAC first
//! ([`super::bus::envelope`]); a badge ignores its own samples.
//!
//! App keys: the middleware has no wildcard subscriptions, so the bus
//! subscriptions follow the apps: after every app command and tick, every
//! key in `Runtime::subscriptions()` of any local app gets a bus
//! subscription on every badge (keys local apps publish on and the keys
//! in [`WorldConfig::app_keys`] are subscribed too). A badge whose app did
//! not subscribe ignores the sample (`Runtime::deliver`).

use std::collections::BTreeSet;

use badge_core::color::Color;
use badge_core::frame;
use badge_core::oz::{self, PeerId};
use badge_core::sync::SyncBeacon;
use badge_mw::keys;
use badge_proto::{BadgeMsg, BadgeMsg_, HOST_MSG_MAX, HostMsg};
use badge_sim::inputs::VIA_BLE;
use badge_sim::wire::{DecodeError, Record, Records};

use super::apps::{AppAction, AppSlot};
use super::bus::{Network, envelope, open_envelope};
use super::guest::{Guest, GuestError, SimModule};
use crate::session::{DOWN_FRAME_MAX, UP_FRAME_MAX};

/// Default MAC base: badge `i` gets `base + i + 1` (`02:00:c0:ff:ee:01`,
/// a locally administered address, peer id `FFEE01`).
pub const DEFAULT_MAC_BASE: u64 = 0x0200_C0FF_EE00;
/// Messages kept per badge when nobody drains them.
pub const HISTORY_MAX: usize = 100_000;
/// Up-stream bytes kept per badge when the link is not read.
pub const UP_MAX: usize = 1 << 20;
/// Samples drained per subscription per step (more stay queued).
pub const DRAIN_MAX: usize = 256;
/// Follow-up guest calls per routed batch (wasm answers, app actions).
pub const ROUTE_MAX: usize = 256;

/// World parameters.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WorldConfig {
    /// Number of badges.
    pub badges: usize,
    /// MAC base (badge `i` is `mac_base + i + 1`).
    pub mac_base: u64,
    /// Virtual tick length.
    pub tick_ms: u64,
    /// Boot delay between consecutive badges.
    pub stagger_ms: u64,
    /// RSSI of every simulated BLE reception.
    pub rssi: i8,
    /// Advert repeat interval.
    pub adv_interval_ms: u64,
    /// Added to `rx_at` of received sync beacons.
    pub latency_ms: u64,
    /// App keys every badge subscribes to from the start.
    pub app_keys: Vec<String>,
}

impl Default for WorldConfig {
    fn default() -> Self {
        Self {
            badges: 2,
            mac_base: DEFAULT_MAC_BASE,
            tick_ms: 20,
            stagger_ms: 1234,
            rssi: -40,
            adv_interval_ms: 100,
            latency_ms: u64::from(badge_core::sync::RX_LATENCY_MS),
            app_keys: Vec::new(),
        }
    }
}

/// Simulation failures.
#[derive(Debug, thiserror::Error)]
pub enum SimError {
    /// The wasm guest failed.
    #[error("badge {badge}: {source}")]
    Guest {
        /// Badge index.
        badge: usize,
        /// Cause.
        #[source]
        source: GuestError,
    },
    /// The bus failed.
    #[error("bus: {0}")]
    Net(#[source] Box<dyn std::error::Error + Send + Sync>),
    /// The guest produced a malformed record.
    #[error("badge {badge}: malformed output record: {error:?}")]
    Wire {
        /// Badge index.
        badge: usize,
        /// Decode error.
        error: DecodeError,
    },
    /// No badge with this index.
    #[error("no badge {0} (the simulation has {1})")]
    NoBadge(usize, usize),
    /// A message could not be encoded.
    #[error("cannot encode message: {0}")]
    Encode(String),
    /// Too many follow-up calls in one batch (a guest feedback loop).
    #[error("badge {0}: routing did not settle")]
    Runaway(usize),
}

/// One simulated badge and its surroundings.
pub struct Node<S> {
    guest: Guest,
    mac: u64,
    boot_at: u64,
    booted: bool,
    advert: Option<Vec<u8>>,
    next_adv: u64,
    subs: Vec<(String, S)>,
    app: AppSlot,
    link: bool,
    up: Vec<u8>,
    down: frame::Decoder<DOWN_FRAME_MAX>,
    history: Vec<(u64, BadgeMsg_::Kind)>,
    leds: Option<[Color; badge_core::board::led::RGB_COUNT]>,
    net_ref: Option<(u64, u32)>,
    fb_dirty: bool,
}

impl<S> Node<S> {
    /// Its stock-protocol peer id.
    pub fn peer_id(&self) -> PeerId {
        PeerId::from_mac(&mac_bytes(self.mac))
    }

    /// Booted yet.
    pub fn booted(&self) -> bool {
        self.booted
    }

    /// Last LED colours the badge showed.
    pub fn leds(&self) -> Option<[Color; badge_core::board::led::RGB_COUNT]> {
        self.leds
    }

    /// Messages sent so far and not yet taken.
    #[cfg(test)]
    pub fn history(&self) -> &[(u64, BadgeMsg_::Kind)] {
        &self.history
    }

    /// Wasm runtime status.
    pub fn wasm_status(&self) -> badge_proto::WasmStatus {
        self.app.status()
    }

    /// Network time estimate at local time `local` (local time before the
    /// first own sync beacon).
    fn network_ms(&self, local: u64) -> i64 {
        match self.net_ref {
            Some((at, net)) => i64::from(net.wrapping_add(u32::try_from(local.saturating_sub(at)).unwrap_or(0))),
            None => i64::try_from(local).unwrap_or(i64::MAX),
        }
    }
}

/// The MAC bytes of a 48-bit value.
pub fn mac_bytes(mac: u64) -> [u8; 6] {
    let [_, _, a, b, c, d, e, f] = mac.to_be_bytes();
    [a, b, c, d, e, f]
}

/// The simulation.
pub struct World<N: Network> {
    cfg: WorldConfig,
    net: N,
    nodes: Vec<Node<N::Sub>>,
    now: u64,
    app_keys: BTreeSet<String>,
}

fn net_err<E: std::error::Error + Send + Sync + 'static>(e: E) -> SimError {
    SimError::Net(Box::new(e))
}

impl<N: Network> World<N> {
    /// Builds the world: one guest per badge, each subscribed to the sync
    /// and OZ keys and to `cfg.app_keys`. Nothing is booted before the first
    /// [`World::step`].
    pub fn new(module: &SimModule, cfg: WorldConfig, mut net: N) -> Result<Self, SimError> {
        let mut nodes = Vec::with_capacity(cfg.badges);
        for i in 0..cfg.badges {
            let guest = module.instantiate().map_err(|source| SimError::Guest { badge: i, source })?;
            let mut subs = Vec::new();
            for key in [keys::SYNC.as_str(), keys::OZ.as_str()] {
                subs.push((key.to_string(), net.subscribe(key).map_err(net_err)?));
            }
            let index = u64::try_from(i).unwrap_or(u64::MAX);
            nodes.push(Node {
                guest,
                mac: cfg.mac_base.wrapping_add(index + 1) & 0xFFFF_FFFF_FFFF,
                boot_at: index.saturating_mul(cfg.stagger_ms),
                booted: false,
                advert: None,
                next_adv: 0,
                subs,
                app: AppSlot::new(),
                link: false,
                up: Vec::new(),
                down: frame::Decoder::new(),
                history: Vec::new(),
                leds: None,
                net_ref: None,
                fb_dirty: false,
            });
        }
        let mut world = Self { cfg, net, nodes, now: 0, app_keys: BTreeSet::new() };
        for key in world.cfg.app_keys.clone() {
            world.ensure_app_key(&key)?;
        }
        Ok(world)
    }

    /// Virtual time of the next step.
    pub fn now_ms(&self) -> u64 {
        self.now
    }

    /// Number of badges.
    pub fn len(&self) -> usize {
        self.nodes.len()
    }

    /// Badge `i`.
    pub fn node(&self, i: usize) -> Result<&Node<N::Sub>, SimError> {
        self.nodes.get(i).ok_or(SimError::NoBadge(i, self.nodes.len()))
    }

    fn node_mut(&mut self, i: usize) -> Result<&mut Node<N::Sub>, SimError> {
        let n = self.nodes.len();
        self.nodes.get_mut(i).ok_or(SimError::NoBadge(i, n))
    }

    fn local(&self, i: usize) -> Result<u64, SimError> {
        Ok(self.now.saturating_sub(self.node(i)?.boot_at))
    }

    fn guest_call<F>(&mut self, i: usize, f: F) -> Result<Vec<u8>, SimError>
    where
        F: FnOnce(&mut Guest) -> Result<Vec<u8>, GuestError>,
    {
        let node = self.node_mut(i)?;
        f(&mut node.guest).map_err(|source| SimError::Guest { badge: i, source })
    }

    /// Subscribes every badge to app key `key` (once).
    fn ensure_app_key(&mut self, key: &str) -> Result<(), SimError> {
        if !self.app_keys.insert(key.to_string()) {
            return Ok(());
        }
        for node in &mut self.nodes {
            let sub = self.net.subscribe(key).map_err(net_err)?;
            node.subs.push((key.to_string(), sub));
        }
        Ok(())
    }

    /// Subscribes the bus to every key badge `i`'s app subscribed to.
    fn follow_app_subscriptions(&mut self, i: usize) -> Result<(), SimError> {
        for key in self.node(i)?.app.subscriptions() {
            self.ensure_app_key(&key)?;
        }
        Ok(())
    }

    /// App keys the bus is subscribed to.
    #[cfg(test)]
    pub fn subscribed_app_keys(&self) -> impl Iterator<Item = &str> {
        self.app_keys.iter().map(String::as_str)
    }

    /// Advances one tick.
    pub fn step(&mut self) -> Result<(), SimError> {
        for i in 0..self.nodes.len() {
            let now = self.now;
            let node = self.node_mut(i)?;
            if !node.booted && now >= node.boot_at {
                node.booted = true;
                node.next_adv = now;
                let mac = node.mac;
                let out = self.guest_call(i, |g| g.init(mac))?;
                self.route(i, out)?;
            }
            if !self.node(i)?.booted {
                continue;
            }
            let local = self.local(i)?;
            let out = self.guest_call(i, |g| g.tick(local, 0))?;
            self.route(i, out)?;
            let node = self.node_mut(i)?;
            let net_now = node.network_ms(local);
            let actions = node.app.tick(net_now);
            self.follow_app_subscriptions(i)?;
            self.actions(i, actions)?;
            let interval = self.cfg.adv_interval_ms.max(1);
            let node = self.node_mut(i)?;
            if now >= node.next_adv {
                node.next_adv = now + interval;
                if let Some(ad) = node.advert.clone() {
                    let payload = envelope(node.mac, &ad);
                    self.net.publish(keys::OZ.as_str(), &payload).map_err(net_err)?;
                }
            }
        }
        for i in 0..self.nodes.len() {
            self.drain(i)?;
        }
        self.now += self.cfg.tick_ms;
        Ok(())
    }

    /// Steps until virtual time has advanced by at least `ms`.
    pub fn run_for(&mut self, ms: u64) -> Result<(), SimError> {
        let end = self.now.saturating_add(ms);
        while self.now < end {
            self.step()?;
        }
        Ok(())
    }

    /// Hands everything already received on badge `i`'s subscriptions to it.
    fn drain(&mut self, i: usize) -> Result<(), SimError> {
        let subs = self.node(i)?.subs.len();
        for s in 0..subs {
            for _ in 0..DRAIN_MAX {
                let Some(node) = self.nodes.get_mut(i) else { break };
                let Some((_, sub)) = node.subs.get_mut(s) else { break };
                let Some((key, payload)) = self.net.try_recv(sub).map_err(net_err)? else { break };
                self.receive(i, &key, &payload)?;
            }
        }
        Ok(())
    }

    /// One received sample for badge `i`.
    fn receive(&mut self, i: usize, key: &str, payload: &[u8]) -> Result<(), SimError> {
        let node = self.node(i)?;
        let Some((origin, body)) = open_envelope(payload) else { return Ok(()) };
        if origin == node.mac || !node.booted {
            return Ok(());
        }
        if key == keys::OZ.as_str() {
            let Some(pkt) = oz::oz_payload(body) else { return Ok(()) };
            let rssi = self.cfg.rssi;
            let out = self.guest_call(i, |g| g.radio(VIA_BLE, rssi, pkt))?;
            self.route(i, out)
        } else if key == keys::SYNC.as_str() {
            let Ok(b) = SyncBeacon::decode(body) else { return Ok(()) };
            let rx_at = self.local(i)? + self.cfg.latency_ms;
            let out = self.guest_call(i, |g| g.sync(b.id, b.network_ms, rx_at))?;
            self.route(i, out)
        } else {
            self.node_mut(i)?.app.deliver(key, body);
            Ok(())
        }
    }

    /// Carries out the records of one guest call (and their follow-ups).
    fn route(&mut self, i: usize, first: Vec<u8>) -> Result<(), SimError> {
        let mut work = std::collections::VecDeque::from([first]);
        let mut calls = 0usize;
        while let Some(bytes) = work.pop_front() {
            calls += 1;
            if calls > ROUTE_MAX {
                return Err(SimError::Runaway(i));
            }
            for record in Records::new(&bytes) {
                let record = record.map_err(|error| SimError::Wire { badge: i, error })?;
                match record {
                    Record::Send(pb) => self.sent(i, pb)?,
                    Record::Beacon(ad) => self.node_mut(i)?.advert = Some(ad.to_vec()),
                    Record::Sync { id, network_ms } => {
                        let local = self.local(i)?;
                        let node = self.node_mut(i)?;
                        node.net_ref = Some((local, network_ms));
                        let payload = envelope(node.mac, &SyncBeacon { id, network_ms }.encode());
                        self.net.publish(keys::SYNC.as_str(), &payload).map_err(net_err)?;
                    }
                    Record::Leds(c) => self.node_mut(i)?.leds = Some(c),
                    Record::Screen(_) => self.node_mut(i)?.fb_dirty = true,
                    Record::Wasm { seq, cmd } => {
                        let local = self.local(i)?;
                        let node = self.node_mut(i)?;
                        let net_now = node.network_ms(local);
                        let (answer, actions) = node.app.command(cmd, net_now);
                        self.follow_app_subscriptions(i)?;
                        let mut pb = [0u8; 256];
                        let answer = match &answer {
                            Ok(st) => badge_proto::encode(st, &mut pb)
                                .map(|n| pb.get(..n).unwrap_or(&[]))
                                .map_err(|e| SimError::Encode(format!("{e:?}")))
                                .map(Ok)?,
                            Err(code) => Err(*code),
                        };
                        let out = self.guest_call(i, |g| g.wasm(seq, answer))?;
                        work.push_back(out);
                        work.extend(self.action_calls(i, actions)?);
                    }
                    Record::Indicator { .. }
                    | Record::LinkSend(_)
                    | Record::SaveContact(_)
                    | Record::ClearContacts => {}
                }
            }
        }
        Ok(())
    }

    /// Executes app actions and routes what they produce.
    fn actions(&mut self, i: usize, actions: Vec<AppAction>) -> Result<(), SimError> {
        for out in self.action_calls(i, actions)? {
            self.route(i, out)?;
        }
        Ok(())
    }

    /// Executes app actions; returns the guest outputs still to route.
    fn action_calls(&mut self, i: usize, actions: Vec<AppAction>) -> Result<Vec<Vec<u8>>, SimError> {
        let mut outs = Vec::new();
        for action in actions {
            match action {
                AppAction::Log(line) => outs.push(self.guest_call(i, |g| g.wasm_log(&line))?),
                AppAction::Publish(key, payload) => {
                    self.ensure_app_key(&key)?;
                    let payload = envelope(self.node(i)?.mac, &payload);
                    self.net.publish(&key, &payload).map_err(net_err)?;
                }
                AppAction::Host(msg) => {
                    let pb = encode_host(&msg)?;
                    outs.push(self.guest_call(i, |g| g.host(&pb))?);
                }
            }
        }
        Ok(outs)
    }

    /// A `Send` record of badge `i`.
    fn sent(&mut self, i: usize, pb: &[u8]) -> Result<(), SimError> {
        let now = self.now;
        let node = self.node_mut(i)?;
        if let Some(kind) = badge_proto::decode::<BadgeMsg>(pb).ok().and_then(|m| m.kind) {
            if node.history.len() >= HISTORY_MAX {
                node.history.remove(0);
            }
            node.history.push((now, kind));
        }
        if node.link {
            let mut framed = [0u8; UP_FRAME_MAX];
            let n = frame::encode(pb, &mut framed).map_err(|e| SimError::Encode(format!("{e:?}")))?;
            if node.up.len() + n <= UP_MAX {
                node.up.extend_from_slice(framed.get(..n).unwrap_or(&[]));
            }
        }
        Ok(())
    }

    /// Feeds one host message to badge `i` directly (no framing).
    pub fn send_host(&mut self, i: usize, msg: &HostMsg) -> Result<(), SimError> {
        let pb = encode_host(msg)?;
        let out = self.guest_call(i, |g| g.host(&pb))?;
        self.route(i, out)
    }

    /// Host-link bytes (COBS frames, like RTT down) for badge `i`; attaches
    /// the link, so responses are framed into its up stream from now on.
    pub fn write_down(&mut self, i: usize, bytes: &[u8]) -> Result<(), SimError> {
        self.node_mut(i)?.link = true;
        for &b in bytes {
            let node = self.node_mut(i)?;
            let frame = match node.down.push(b) {
                Some(Ok(payload)) => payload.to_vec(),
                // A broken frame is answered like the firmware does: as an
                // undecodable host message.
                Some(Err(_)) => vec![0xFF],
                None => continue,
            };
            let out = self.guest_call(i, |g| g.host(&frame))?;
            self.route(i, out)?;
        }
        Ok(())
    }

    /// Bytes waiting on badge `i`'s up stream.
    pub fn up_len(&self, i: usize) -> Result<usize, SimError> {
        Ok(self.node(i)?.up.len())
    }

    /// Moves up-stream bytes of badge `i` into `buf`.
    pub fn read_up(&mut self, i: usize, buf: &mut [u8]) -> Result<usize, SimError> {
        let node = self.node_mut(i)?;
        let n = buf.len().min(node.up.len());
        if let (Some(dst), Some(src)) = (buf.get_mut(..n), node.up.get(..n)) {
            dst.copy_from_slice(src);
        }
        node.up.drain(..n);
        Ok(n)
    }

    /// Takes badge `i`'s message history.
    pub fn take_messages(&mut self, i: usize) -> Result<Vec<(u64, BadgeMsg_::Kind)>, SimError> {
        Ok(std::mem::take(&mut self.node_mut(i)?.history))
    }

    /// Badge `i`'s framebuffer if it changed since the last call.
    pub fn take_frame(&mut self, i: usize) -> Result<Option<Vec<u8>>, SimError> {
        let node = self.node_mut(i)?;
        if !std::mem::take(&mut node.fb_dirty) {
            return Ok(None);
        }
        node.guest.framebuffer().map(Some).map_err(|source| SimError::Guest { badge: i, source })
    }

    /// Output records badge `i` dropped because its output region was full.
    pub fn dropped(&mut self, i: usize) -> Result<u32, SimError> {
        self.node_mut(i)?.guest.dropped().map_err(|source| SimError::Guest { badge: i, source })
    }

    /// Badge `i`'s current framebuffer.
    pub fn frame(&self, i: usize) -> Result<Vec<u8>, SimError> {
        self.node(i)?.guest.framebuffer().map_err(|source| SimError::Guest { badge: i, source })
    }
}

/// Protobuf bytes of a host message.
pub fn encode_host(msg: &HostMsg) -> Result<Vec<u8>, SimError> {
    let mut pb = [0u8; HOST_MSG_MAX];
    let n = badge_proto::encode(msg, &mut pb).map_err(|e| SimError::Encode(format!("{e:?}")))?;
    Ok(pb.get(..n).unwrap_or(&[]).to_vec())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fr39_mac_layout_and_peer_ids() {
        assert_eq!(mac_bytes(DEFAULT_MAC_BASE + 1), [2, 0, 0xC0, 0xFF, 0xEE, 1]);
        assert_eq!(PeerId::from_mac(&mac_bytes(DEFAULT_MAC_BASE + 2)).as_str(), "FFEE02");
    }
}
