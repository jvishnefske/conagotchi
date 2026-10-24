//! The stock Chi trade (`trade_session.py` on top of `lobby_session.py`,
//! game `T`) as a pure state machine.
//!
//! Every badge with the lobby open beacons `OZGT:<me>:------:~<TAG>` every
//! 300 ms plus 0..=150 ms of jitter ([`Session::beacon`]). Inviting a badge
//! addresses it (`OZGT:<me>:<peer>:~<TAG>`); the two are *linked* once each
//! has received a packet from the other addressed to itself, so both people
//! agree. Linked, each side sends `<TAG><flags>` where flags is a digit,
//! bit 0 = "got yours". A side collects the peer's tag the first time it
//! arrives (and beacons at once), and the trade is complete once it has the
//! peer's tag and the peer confirmed ours. It then keeps sending for
//! [`LINGER_MS`] so the peer sees the last confirmation, and leaves.
//!
//! Mirrored stock rules:
//! - only the invited peer's packets move the trade; others just update
//!   the lobby; packets from our own id are ignored;
//! - an invite not returned within [`INVITE_MS`] lapses ([`TradeState::Lost`]);
//! - a linked peer silent for [`LOST_MS`], or addressing someone else (it
//!   left), ends the trade: [`TradeState::Lost`], unless its Chi had already
//!   arrived, which stock counts as done ([`TradeState::Complete`]);
//! - payloads shorter than 4 characters are ignored; an unknown tag is
//!   never collected, so that trade never completes ([`Session::unknown_tag`]);
//! - inviting is refused while linked and for our own id; a new invite
//!   replaces an unanswered one.
//!
//! Our additions: the state is reported as a [`TradeState`] with an
//! [`Event`] on every change (≤ [`MAX_EVENTS`] per call); after the linger
//! we leave by ourselves (stock waits for a button press); jitter comes
//! from a xorshift sequence seeded by our id, not an RNG.
//!
//! Time is whatever the caller passes (`App::now`); nothing here reads a
//! clock.

use badge_proto::{Chi, TradeState};
use heapless::Vec;

use super::lobby::{
    self, BEACON_JITTER_MS, BEACON_MS, GAME_TRADE, Lobby, NOTE_MARK, PACKET_MAX, Packet,
};
use super::{PeerId, chi_from_tag, chi_tag};
use crate::io::{Millis, Via};

pub use super::lobby::{INVITE_MS, LOST_MS};

/// Keep confirming this long after completing (`COMPLETE_LINGER_MS`).
pub const LINGER_MS: u64 = 1_500;
/// Most [`Event`]s one call returns (linked and completed by one packet).
pub const MAX_EVENTS: usize = 2;

/// A change of the trade state, for `TradeEvent`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Event {
    /// The trade's peer.
    pub peer: PeerId,
    /// New state.
    pub state: TradeState,
    /// Chi received; set only for [`TradeState::Complete`].
    pub chi: Chi,
}

/// Events from one call, in order.
pub type Events = Vec<Event, MAX_EVENTS>;

/// Why an invite was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InviteError {
    /// That is our own id.
    Myself,
    /// Already linked with a peer.
    Linked,
}

/// Why the offered Chi cannot change.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChiError {
    /// UNSPECIFIED or not one of the six tags.
    Unknown,
    /// Linked: the peer is already receiving our tag.
    Linked,
}

/// Why the current peer is being dropped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Drop {
    /// Linked peer silent for LOST_MS.
    Quiet,
    /// Linked peer addressed someone else.
    Left,
    /// Invite not returned within INVITE_MS.
    Lapsed,
    /// Cancelled here.
    Cancel,
    /// Replaced by a new invite.
    Replace,
    /// Linger over after completing.
    Finished,
}

/// Lobby plus the current trade. See the module docs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Session {
    me: PeerId,
    chi: Chi,
    lobby: Lobby,
    now: Millis,
    active: Option<PeerId>,
    linked: bool,
    peer_since: Millis,
    peer_seen: Millis,
    next_beacon: Millis,
    rng: u32,
    received: Option<Chi>,
    unknown_tag: bool,
    confirmed: bool,
    done_at: Option<Millis>,
    state: TradeState,
    last_peer: Option<PeerId>,
    last_chi: Chi,
}

impl Session {
    /// Idle session for badge `me` offering `chi` (a tagless Chi is
    /// replaced by VIB). The first beacon is due at once.
    pub fn new(me: PeerId, chi: Chi) -> Session {
        Session {
            me,
            chi: if chi_tag(chi).is_some() { chi } else { Chi::Vib },
            lobby: Lobby::new(),
            now: Millis(0),
            active: None,
            linked: false,
            peer_since: Millis(0),
            peer_seen: Millis(0),
            next_beacon: Millis(0),
            rng: (me.to_u32() ^ 0x9E37_79B9) | 1,
            received: None,
            unknown_tag: false,
            confirmed: false,
            done_at: None,
            state: TradeState::Idle,
            last_peer: None,
            last_chi: Chi::Unspecified,
        }
    }

    // -------------------------------------------------------------- reads

    /// Our id.
    pub fn me(&self) -> PeerId {
        self.me
    }

    /// The Chi we offer.
    pub fn chi(&self) -> Chi {
        self.chi
    }

    /// The badges heard.
    pub fn lobby(&self) -> &Lobby {
        &self.lobby
    }

    /// Time of the last call.
    pub fn now(&self) -> Millis {
        self.now
    }

    /// State of the current (or last) trade.
    pub fn state(&self) -> TradeState {
        self.state
    }

    /// Peer of the current (or last) trade.
    pub fn peer(&self) -> Option<PeerId> {
        self.last_peer
    }

    /// The badge invited or linked right now, if any.
    pub fn active_peer(&self) -> Option<PeerId> {
        self.active
    }

    /// True while both badges address each other.
    pub fn linked(&self) -> bool {
        self.linked
    }

    /// Chi received in the current trade, or in the last completed one.
    pub fn received(&self) -> Chi {
        self.received.unwrap_or(self.last_chi)
    }

    /// True if the linked peer sent a tag we do not know (that trade can
    /// never complete; cancel it).
    pub fn unknown_tag(&self) -> bool {
        self.unknown_tag
    }

    /// True if our next beacon is due at `now`.
    pub fn beacon_due(&self, now: Millis) -> bool {
        now >= self.next_beacon
    }

    // ------------------------------------------------------------ changes

    /// Timers (stock `poll` minus reading and beaconing): lobby expiry,
    /// invite lapse, lost link, end of the linger.
    pub fn tick(mut self, now: Millis) -> (Session, Events) {
        let mut events = Events::new();
        self.now = now;
        self.lobby = self.lobby.expire(now);
        if self.active.is_some() {
            if self.linked && now.0.saturating_sub(self.peer_seen.0) >= LOST_MS {
                self.drop_peer(Drop::Quiet, &mut events);
            } else if !self.linked && now.0.saturating_sub(self.peer_since.0) >= INVITE_MS {
                self.drop_peer(Drop::Lapsed, &mut events);
            }
        }
        if self.done_at.is_some_and(|t| now >= t) {
            self.drop_peer(Drop::Finished, &mut events);
        }
        (self, events)
    }

    /// Our beacon packet (untagged) and the time of the next one: 300 ms +
    /// 0..=150 ms jitter. Call when [`Session::beacon_due`].
    pub fn beacon(mut self, now: Millis) -> (Session, Vec<u8, PACKET_MAX>) {
        let tag = chi_tag(self.chi).unwrap_or(b"VIB");
        let mut payload: Vec<u8, 4> = Vec::new();
        if self.linked {
            let flags = if self.received.is_some() { b'1' } else { b'0' };
            payload.extend_from_slice(tag).ok();
            payload.push(flags).ok();
        } else {
            payload.push(NOTE_MARK as u8).ok();
            payload.extend_from_slice(tag).ok();
        }
        // Tags are three ASCII letters, so this always fits.
        let packet = lobby::pack(GAME_TRADE, self.me, self.active, &payload).unwrap_or_default();
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 17;
        self.rng ^= self.rng << 5;
        let jitter = u64::from(self.rng) % (BEACON_JITTER_MS + 1);
        self.next_beacon = Millis(now.0.saturating_add(BEACON_MS + jitter));
        (self, packet)
    }

    /// A verified lobby packet heard at `now` (stock `handle_packet` and
    /// `on_payload`). The caller applies the Bluetooth RSSI gate.
    pub fn on_packet(
        mut self,
        now: Millis,
        via: Via,
        rssi: i8,
        p: &Packet<'_>,
    ) -> (Session, Events) {
        let mut events = Events::new();
        self.now = now;
        if p.game != GAME_TRADE || p.sender == self.me {
            return (self, events);
        }
        let to_me = p.to == Some(self.me);
        self.lobby = self.lobby.heard(now, via, rssi, p, to_me);
        let Some(peer) = self.active.filter(|&q| q == p.sender) else {
            return (self, events);
        };
        self.peer_seen = now;
        if !to_me {
            if self.linked {
                self.drop_peer(Drop::Left, &mut events);
            }
            return (self, events);
        }
        if !self.linked {
            self.linked = true;
            self.report(TradeState::Exchanging, peer, Chi::Unspecified, &mut events);
        }
        if !p.payload.is_empty() && !p.is_note() {
            self.on_payload(p.payload, peer, &mut events);
        }
        (self, events)
    }

    /// Invites `peer` (stock `invite`): replaces an unanswered invite.
    pub fn invite(mut self, now: Millis, peer: PeerId) -> (Session, Result<Events, InviteError>) {
        let mut events = Events::new();
        self.now = now;
        if peer == self.me {
            return (self, Err(InviteError::Myself));
        }
        if self.linked {
            return (self, Err(InviteError::Linked));
        }
        self.drop_peer(Drop::Replace, &mut events);
        self.active = Some(peer);
        self.peer_since = now;
        self.peer_seen = now;
        self.next_beacon = now;
        self.report(TradeState::Inviting, peer, Chi::Unspecified, &mut events);
        (self, Ok(events))
    }

    /// Cancels an invite or leaves a trade (stock `leave`). A completed
    /// trade stays complete.
    pub fn cancel(mut self, now: Millis) -> (Session, Events) {
        let mut events = Events::new();
        self.now = now;
        self.drop_peer(Drop::Cancel, &mut events);
        (self, events)
    }

    /// Changes the Chi we offer.
    pub fn set_chi(mut self, chi: Chi) -> (Session, Result<(), ChiError>) {
        if chi_tag(chi).is_none() {
            return (self, Err(ChiError::Unknown));
        }
        if self.linked {
            return (self, Err(ChiError::Linked));
        }
        self.chi = chi;
        (self, Ok(()))
    }

    /// Records a display name for a lobby badge.
    pub fn named(mut self, id: PeerId, name: &str) -> Session {
        self.lobby = self.lobby.named(id, name);
        self
    }

    // ---------------------------------------------------------- internals

    /// Stock `on_payload`: `<tag><flags>` from the linked peer.
    fn on_payload(&mut self, text: &str, peer: PeerId, events: &mut Events) {
        let mut chars = text.char_indices();
        let Some((flag_at, flag)) = chars.nth(3) else {
            return; // fewer than 4 characters
        };
        let flags = flag.to_digit(10).unwrap_or(0);
        if self.received.is_none() {
            match chi_from_tag(text.as_bytes().get(..flag_at).unwrap_or(&[])) {
                Some(chi) => {
                    self.received = Some(chi);
                    self.unknown_tag = false;
                    self.next_beacon = self.now; // confirm at once
                }
                None => self.unknown_tag = true,
            }
        }
        if flags & 1 != 0 {
            self.confirmed = true;
        }
        if let (Some(chi), true, None) = (self.received, self.confirmed, self.done_at) {
            self.done_at = Some(Millis(self.now.0.saturating_add(LINGER_MS)));
            self.report(TradeState::Complete, peer, chi, events);
        }
    }

    /// Stock `_drop_peer`, plus the state report for why.
    fn drop_peer(&mut self, why: Drop, events: &mut Events) {
        let Some(peer) = self.active else {
            return;
        };
        if self.state != TradeState::Complete || self.last_peer != Some(peer) {
            match (why, self.received) {
                (Drop::Quiet | Drop::Left, Some(chi)) => {
                    self.report(TradeState::Complete, peer, chi, events)
                }
                (Drop::Quiet | Drop::Left | Drop::Lapsed, _) => {
                    self.report(TradeState::Lost, peer, Chi::Unspecified, events)
                }
                (Drop::Cancel, _) => {
                    self.report(TradeState::Cancelled, peer, Chi::Unspecified, events)
                }
                (Drop::Replace | Drop::Finished, _) => {}
            }
        }
        if matches!(why, Drop::Cancel | Drop::Finished) {
            self.next_beacon = self.now;
        }
        self.active = None;
        self.linked = false;
        self.received = None;
        self.unknown_tag = false;
        self.confirmed = false;
        self.done_at = None;
    }

    /// Moves to `state` with `peer`, reporting it if anything changed.
    fn report(&mut self, state: TradeState, peer: PeerId, chi: Chi, events: &mut Events) {
        if state == TradeState::Complete {
            self.last_chi = chi;
        }
        if state != self.state || self.last_peer != Some(peer) {
            self.state = state;
            self.last_peer = Some(peer);
            // At most two changes per call by construction (see MAX_EVENTS).
            events.push(Event { peer, state, chi }).ok();
        }
    }
}

#[cfg(test)]
mod tests;
