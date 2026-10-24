//! LED timing sync between badges running this firmware (FR-31).
//!
//! Badges broadcast a [`SyncBeacon`] every [`BEACON_INTERVAL_MS`] over
//! ESP-NOW: the 4-byte magic [`BEACON_MAGIC`] (`OZL1`), the sender's sync id
//! (u32, from its MAC, see [`SyncClock::id_from_mac`]) and its network time
//! (u32 milliseconds, wrapping), all little-endian, [`BEACON_LEN`] bytes in
//! total. [`SyncBeacon::encode`] and [`SyncBeacon::decode`] are the only wire
//! code; decode rejects a wrong length or magic.
//!
//! [`SyncClock`] is an immutable value: every input returns a new clock. It
//! keeps an offset between local uptime and *network time*; LED animations
//! are rendered from [`SyncClock::network_time`], so badges that hear each
//! other show the same phase.
//!
//! # Algorithm
//!
//! - **Network time** is a [`NetMs`]: the low 32 bits of local uptime plus
//!   the offset, wrapping at 2^32 ms (about 49.7 days). All comparisons use
//!   wrapping differences, so wraparound is invisible to the algorithm; the
//!   LED phase jumps once per wrap, on all synced badges at the same moment.
//! - **Peers**: a fixed table of [`MAX_PEERS`] `(id, last heard)` entries. A
//!   peer is fresh if heard within [`PEER_TIMEOUT_MS`]. When the table is
//!   full of fresh peers, a new peer replaces the highest id only if its own
//!   id is lower, so the table always holds the lowest ids heard.
//! - **Leader**: the lowest id among ourselves and the fresh peers that have
//!   been heard over a span of at least [`LEADER_PROBATION_MS`]. The
//!   probation (a small refinement of "lowest id heard in the last 3 s")
//!   gives a newcomer time to hear the group and join its timeline (see
//!   *Joining*) before the group starts following it.
//! - **Steering**: only the current leader's beacons steer the offset. The
//!   error is the leader's network time plus [`RX_LATENCY_MS`] minus our
//!   network time at reception. If `|error| > STEP_THRESHOLD_MS` the offset
//!   is stepped by the whole error; otherwise it slews by half the error
//!   (a gain of 1/2 halves the reception jitter that reaches the LEDs),
//!   clamped to [`MAX_SLEW_MS`] per beacon.
//! - **Alone**: the offset is *kept*, not reset to 0. Network time then runs
//!   at the local rate with no jump, so the LEDs never glitch when the last
//!   peer leaves, and a returning group is still close in time (only drift
//!   separates them).
//! - **Joining** (the one exception to "only the leader steers"): a badge
//!   that has no fresh peer and hears a beacon from a *higher* id adopts that
//!   beacon's time in one step. A newcomer with the lowest id therefore takes
//!   over the existing group's timeline during its probation, and the group
//!   sees no jump when it becomes leader. Only the newcomer's own LEDs jump,
//!   once, as it joins. Two lone badges meeting: only the lower id adopts,
//!   so they do not swap times.
//! - **Handover**: when the leader goes silent, the next lowest id takes over
//!   after [`PEER_TIMEOUT_MS`]. It was following the old leader, so its time
//!   already matches the group's and nobody jumps. Two separately synced
//!   groups that meet merge onto the lower leader; the other group steps or
//!   slews once (unavoidable without a richer protocol).

use crate::io::Millis;

/// First four bytes of every sync beacon.
pub const BEACON_MAGIC: [u8; 4] = *b"OZL1";
/// Length of an encoded beacon: magic, id, network time.
pub const BEACON_LEN: usize = 12;
/// Interval between our beacons, in local milliseconds.
pub const BEACON_INTERVAL_MS: u64 = 500;
/// A peer not heard for this long is forgotten (and can no longer lead).
pub const PEER_TIMEOUT_MS: u64 = 3000;
/// Size of the peer table.
pub const MAX_PEERS: usize = 8;
/// A new peer must be heard over this span before it can lead.
pub const LEADER_PROBATION_MS: u64 = 1000;
/// Largest offset change per leader beacon while slewing.
pub const MAX_SLEW_MS: i32 = 20;
/// Errors larger than this (in magnitude) are corrected in one step.
pub const STEP_THRESHOLD_MS: u32 = 500;
/// Assumed time from the sender stamping `network_ms` to our `rx_at`.
///
/// The sender stamps the beacon at the start of its core tick; the firmware
/// corrects for time spent queued before transmission, leaving the rest of
/// the tick (well under 1 ms), the ESP-NOW transmission (about 0.3 ms at
/// 1 Mbit/s plus any coexistence wait) and the receiver's wake-up to stamp
/// `rx_at` (well under 1 ms). 2 ms is the estimate; a wrong value shifts
/// every follower by the same amount relative to the leader, it does not
/// cause drift or oscillation.
pub const RX_LATENCY_MS: u32 = 2;

/// A network time in milliseconds, wrapping at 2^32.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct NetMs(pub u32);

impl NetMs {
    /// The low 32 bits of a local time (truncation is the wrap).
    pub fn from_local(now: Millis) -> NetMs {
        NetMs(now.0 as u32)
    }

    /// This time shifted by `ms` (either sign), wrapping.
    pub fn shifted(self, ms: i32) -> NetMs {
        NetMs(self.0.wrapping_add_signed(ms))
    }

    /// `self - earlier` as the shortest signed distance (wrap-aware).
    pub fn since(self, earlier: NetMs) -> i32 {
        self.0.wrapping_sub(earlier.0) as i32
    }
}

/// Why a received buffer is not a sync beacon.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BeaconError {
    /// Not exactly [`BEACON_LEN`] bytes.
    BadLength,
    /// The first four bytes are not [`BEACON_MAGIC`].
    BadMagic,
}

/// A beacon to broadcast, or one received.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SyncBeacon {
    /// The sender's sync id.
    pub id: u32,
    /// The sender's network time when it sent the beacon, wrapping.
    pub network_ms: u32,
}

impl SyncBeacon {
    /// Wire form: magic, id (LE), network time (LE).
    pub fn encode(&self) -> [u8; BEACON_LEN] {
        let mut out = [0u8; BEACON_LEN];
        let (magic, rest) = out.split_at_mut(4);
        let (id, net) = rest.split_at_mut(4);
        magic.copy_from_slice(&BEACON_MAGIC);
        id.copy_from_slice(&self.id.to_le_bytes());
        net.copy_from_slice(&self.network_ms.to_le_bytes());
        out
    }

    /// Parses the wire form.
    ///
    /// # Errors
    ///
    /// [`BeaconError::BadLength`] unless `bytes` is exactly [`BEACON_LEN`]
    /// long, [`BeaconError::BadMagic`] if it does not start with
    /// [`BEACON_MAGIC`].
    pub fn decode(bytes: &[u8]) -> Result<SyncBeacon, BeaconError> {
        let raw: &[u8; BEACON_LEN] = bytes.try_into().map_err(|_| BeaconError::BadLength)?;
        let [m0, m1, m2, m3, i0, i1, i2, i3, n0, n1, n2, n3] = *raw;
        if [m0, m1, m2, m3] != BEACON_MAGIC {
            return Err(BeaconError::BadMagic);
        }
        Ok(SyncBeacon {
            id: u32::from_le_bytes([i0, i1, i2, i3]),
            network_ms: u32::from_le_bytes([n0, n1, n2, n3]),
        })
    }
}

/// One entry of the peer table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Peer {
    /// The peer's sync id.
    id: u32,
    /// Local time of the first beacon of its current presence.
    first_heard: Millis,
    /// Local time of its latest beacon.
    last_heard: Millis,
}

impl Peer {
    /// Heard within [`PEER_TIMEOUT_MS`] of `now`.
    fn fresh(&self, now: Millis) -> bool {
        now.0.saturating_sub(self.last_heard.0) < PEER_TIMEOUT_MS
    }

    /// Fresh and past its [`LEADER_PROBATION_MS`].
    fn eligible(&self, now: Millis) -> bool {
        self.fresh(now)
            && self.last_heard.0.saturating_sub(self.first_heard.0) >= LEADER_PROBATION_MS
    }
}

/// Sync state: offset, peer table and beacon schedule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SyncClock {
    /// Our sync id.
    self_id: u32,
    /// Network time minus local uptime (wrapping, in the u32 domain).
    offset_ms: i32,
    /// Latest local time seen by `tick` or `on_beacon`.
    now: Millis,
    /// Local time our next beacon is due.
    next_beacon: Millis,
    /// Peers heard; `None` slots are free.
    peers: [Option<Peer>; MAX_PEERS],
}

impl SyncClock {
    /// A clock that is alone: network time = uptime. The first beacon is due
    /// on the first tick.
    pub fn new(self_id: u32) -> SyncClock {
        SyncClock {
            self_id,
            offset_ms: 0,
            now: Millis(0),
            next_beacon: Millis(0),
            peers: [None; MAX_PEERS],
        }
    }

    /// The sync id of a badge with this MAC (its last four bytes).
    pub fn id_from_mac(mac: &[u8; 6]) -> u32 {
        u32::from_be_bytes([mac[2], mac[3], mac[4], mac[5]])
    }

    /// Takes a beacon from `from` carrying its `network_ms`, received at
    /// local time `rx_at`. Our own id is ignored.
    pub fn on_beacon(self, from: u32, network_ms: u32, rx_at: Millis) -> SyncClock {
        if from == self.self_id {
            return self;
        }
        let now = Millis(self.now.0.max(rx_at.0));
        let was_alone = SyncClock { now, ..self }.peers() == 0;
        let next = SyncClock {
            now,
            peers: remember(self.peers, from, rx_at, now),
            ..self
        };
        let error = NetMs(network_ms)
            .shifted(RX_LATENCY_MS as i32)
            .since(next.network_ms(rx_at));
        let leader = next.leader();
        let offset_ms = if from == leader {
            steer(next.offset_ms, error)
        } else if was_alone && from > self.self_id {
            next.offset_ms.wrapping_add(error)
        } else {
            next.offset_ms
        };
        SyncClock { offset_ms, ..next }
    }

    /// Advances to `now`, forgetting stale peers; returns a beacon when one
    /// is due (every [`BEACON_INTERVAL_MS`]).
    pub fn tick(self, now: Millis) -> (SyncClock, Option<SyncBeacon>) {
        let now = Millis(self.now.0.max(now.0));
        let peers = self.peers.map(|p| p.filter(|p| p.fresh(now)));
        let next = SyncClock { now, peers, ..self };
        if now.0 < self.next_beacon.0 {
            return (next, None);
        }
        let due = self.next_beacon.0.saturating_add(BEACON_INTERVAL_MS);
        let next_beacon = if due > now.0 {
            Millis(due)
        } else {
            Millis(now.0.saturating_add(BEACON_INTERVAL_MS))
        };
        let beacon = SyncBeacon {
            id: self.self_id,
            network_ms: next.network_ms(now).0,
        };
        (
            SyncClock {
                next_beacon,
                ..next
            },
            Some(beacon),
        )
    }

    /// Network time at local time `now`.
    pub fn network_ms(&self, now: Millis) -> NetMs {
        NetMs::from_local(now).shifted(self.offset_ms)
    }

    /// Network time at local time `now`, as the LED renderer's [`Millis`]
    /// (always below 2^32, see [`NetMs`]).
    pub fn network_time(&self, now: Millis) -> Millis {
        Millis(u64::from(self.network_ms(now).0))
    }

    /// Our id.
    pub fn self_id(&self) -> u32 {
        self.self_id
    }

    /// The current leader's id: the lowest of ours and the fresh peers'
    /// past their [`LEADER_PROBATION_MS`].
    pub fn leader(&self) -> u32 {
        let now = self.now;
        self.peers
            .iter()
            .flatten()
            .filter(|p| p.eligible(now))
            .fold(self.self_id, |low, p| low.min(p.id))
    }

    /// Network time minus local uptime (wrapping, in the u32 domain).
    pub fn offset_ms(&self) -> i32 {
        self.offset_ms
    }

    /// Other badges heard in the last [`PEER_TIMEOUT_MS`] (at most
    /// [`MAX_PEERS`]).
    pub fn peers(&self) -> u32 {
        self.fresh_peers().fold(0, |n, _| n + 1)
    }

    /// The table entries still fresh at the latest time seen.
    fn fresh_peers(&self) -> impl Iterator<Item = &Peer> {
        let now = self.now;
        self.peers.iter().flatten().filter(move |p| p.fresh(now))
    }
}

/// The offset after a leader beacon with this `error`: a step beyond
/// [`STEP_THRESHOLD_MS`], otherwise half the error clamped to
/// [`MAX_SLEW_MS`].
fn steer(offset_ms: i32, error: i32) -> i32 {
    if error.unsigned_abs() > STEP_THRESHOLD_MS {
        offset_ms.wrapping_add(error)
    } else {
        offset_ms.wrapping_add((error / 2).clamp(-MAX_SLEW_MS, MAX_SLEW_MS))
    }
}

/// The peer table after hearing `id` at `at`: its entry is refreshed, or it
/// takes a free or stale slot, or it evicts the highest id if lower.
fn remember(
    peers: [Option<Peer>; MAX_PEERS],
    id: u32,
    at: Millis,
    now: Millis,
) -> [Option<Peer>; MAX_PEERS] {
    let mut out = peers;
    let slot = if let Some(i) = peers.iter().position(|p| p.is_some_and(|p| p.id == id)) {
        Some(i)
    } else if let Some(i) = peers.iter().position(|p| p.is_none_or(|p| !p.fresh(now))) {
        Some(i)
    } else {
        peers
            .iter()
            .enumerate()
            .filter_map(|(i, p)| p.map(|p| (i, p.id)))
            .max_by_key(|&(_, pid)| pid)
            .filter(|&(_, pid)| id < pid)
            .map(|(i, _)| i)
    };
    if let Some(entry) = slot.and_then(|i| out.get_mut(i)) {
        *entry = Some(match entry {
            Some(p) if p.id == id && p.fresh(now) => Peer {
                id,
                first_heard: p.first_heard,
                last_heard: Millis(p.last_heard.0.max(at.0)),
            },
            _ => Peer {
                id,
                first_heard: at,
                last_heard: at,
            },
        });
    }
    out
}

#[cfg(test)]
mod tests;
