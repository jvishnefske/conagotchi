//! Stock lobby packets and the table of badges heard (`lobby_session.py`).
//!
//! Every lobby packet is one text beacon, repeated:
//!
//! `OZG<game>:<sender6>:<to6>:<payload>`
//!
//! - `game`: one letter, `T` for Chi trades ([`GAME_TRADE`]);
//! - `sender`: the sender's six uppercase hex digits ([`PeerId`]);
//! - `to`: the invited or linked badge, or `------` ([`ANYONE`]);
//! - `payload`: at most [`PAYLOAD_MAX`] (4) characters, never `:`. In the
//!   lobby it is [`NOTE_MARK`] + the note (the offered Chi's tag, `~VIB`);
//!   while linked it is the game's state (for trades `VIB1`, see
//!   [`super::trade`]).
//!
//! 26 bytes of a legacy advert are ours (31 - AD header - CID - kind), minus
//! the 3 tag bytes, which leaves 23 for the packet ([`PACKET_MAX`]).
//!
//! [`parse`] mirrors stock `parse` (exactly four `:`-separated parts, a
//! 4-character head, 6-character ids, ids uppercased) with one stricter
//! rule: the sender must be hex (every stock id is), and a `to` that is not
//! hex (e.g. `------`) is "nobody".
//!
//! [`Lobby`] keeps at most [`LOBBY_MAX`] badges (stock keeps 100); a new
//! badge in a full table replaces the one heard longest ago, like stock
//! `remember`. Rows silent for [`NEARBY_MS`] are dropped by
//! [`Lobby::expire`]. A row shows as [`Via::Link`] while the cable carried
//! it within [`LINK_TIMEOUT_MS`] (or it was never heard over the air), else
//! as [`Via::Ble`].

use badge_proto::Chi;
use heapless::{String, Vec};

use super::{NAME_MAX, PeerId, chi_from_tag, sign::TAG_LEN};
use crate::io::{Millis, Via};

/// Game letter of the Chi trade lobby.
pub const GAME_TRADE: char = 'T';
/// `to` field while just in the lobby.
pub const ANYONE: [u8; 6] = *b"------";
/// What one Bluetooth advert carries for a stock packet (tag included).
pub const ADVERT_MAX: usize = 26;
/// Length of `OZGx:123456:123456:`.
const HEADER_LEN: usize = 19;
/// Longest payload (`lobby_session.PAYLOAD_MAX`).
pub const PAYLOAD_MAX: usize = ADVERT_MAX - TAG_LEN - HEADER_LEN;
/// Marks a lobby note in the payload.
pub const NOTE_MARK: char = '~';
/// Longest note.
pub const NOTE_MAX: usize = PAYLOAD_MAX - 1;
/// Longest packet we send (before the tag).
pub const PACKET_MAX: usize = HEADER_LEN + PAYLOAD_MAX;
/// Beacon interval (plus jitter).
pub const BEACON_MS: u64 = 300;
/// Beacon jitter, 0..=this many ms.
pub const BEACON_JITTER_MS: u64 = 150;
/// An invite not returned within this lapses.
pub const INVITE_MS: u64 = 20_000;
/// A linked peer silent this long is gone.
pub const LOST_MS: u64 = 6_000;
/// A lobby badge silent this long drops off the list.
pub const NEARBY_MS: u64 = 4_000;
/// A lobby badge counts as on the cable this long after a cable packet.
pub const LINK_TIMEOUT_MS: u64 = 3_000;
/// Bluetooth lobby packets weaker than this are ignored (stock
/// `config.BLE_RSSI_MIN`, a proximity gate of a metre or two).
pub const RSSI_MIN: i8 = -55;
/// Most badges kept in the lobby table.
pub const LOBBY_MAX: usize = 8;

/// A parsed lobby packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Packet<'a> {
    /// Game letter.
    pub game: char,
    /// Sending badge.
    pub sender: PeerId,
    /// Addressed badge; `None` for `------` (or any non-hex id).
    pub to: Option<PeerId>,
    /// Payload text (may be empty; may carry trailing tag bytes when we have
    /// no key and the sender had one).
    pub payload: &'a str,
}

impl<'a> Packet<'a> {
    /// True if the payload is a lobby note (`~…`).
    pub fn is_note(&self) -> bool {
        self.payload.starts_with(NOTE_MARK)
    }

    /// The note: up to [`NOTE_MAX`] characters after the mark, if a note.
    pub fn note(&self) -> Option<&'a str> {
        let rest = self.payload.strip_prefix(NOTE_MARK)?;
        let end = rest.char_indices().nth(NOTE_MAX).map_or(rest.len(), |(i, _)| i);
        rest.get(..end)
    }
}

/// Parses a verified, tag-stripped stock packet; `None` if it is not a
/// well-formed lobby packet.
pub fn parse(packet: &[u8]) -> Option<Packet<'_>> {
    if !packet.starts_with(b"OZG") || packet.len() < 5 {
        return None;
    }
    let text = core::str::from_utf8(packet).ok()?;
    let mut parts = text.split(':');
    let (head, sender, to, payload) = (parts.next()?, parts.next()?, parts.next()?, parts.next()?);
    if parts.next().is_some() {
        return None;
    }
    if head.chars().count() != 4 || sender.chars().count() != 6 || to.chars().count() != 6 {
        return None;
    }
    let game = head.chars().nth(3)?;
    let sender = PeerId::parse(sender.as_bytes()).ok()?;
    let to = PeerId::parse(to.as_bytes()).ok();
    Some(Packet {
        game,
        sender,
        to,
        payload,
    })
}

/// Why a lobby packet cannot be built.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PackError {
    /// Payload longer than [`PAYLOAD_MAX`] or containing `:`.
    BadPayload,
    /// Game letter is not one ASCII character other than `:`.
    BadGame,
}

/// Builds `OZG<game>:<me>:<to|------>:<payload>` (stock `pack`).
pub fn pack(
    game: char,
    me: PeerId,
    to: Option<PeerId>,
    payload: &[u8],
) -> Result<Vec<u8, PACKET_MAX>, PackError> {
    if payload.len() > PAYLOAD_MAX || payload.contains(&b':') {
        return Err(PackError::BadPayload);
    }
    let game = u8::try_from(game)
        .ok()
        .filter(|g| g.is_ascii() && *g != b':')
        .ok_or(PackError::BadGame)?;
    let to = to.map_or(ANYONE, |t| *t.as_bytes());
    let mut out = Vec::new();
    for piece in [
        &b"OZG"[..],
        &[game],
        b":",
        me.as_bytes(),
        b":",
        &to,
        b":",
        payload,
    ] {
        out.extend_from_slice(piece)
            .map_err(|_| PackError::BadPayload)?;
    }
    Ok(out)
}

/// One badge in the lobby.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Peer {
    id: PeerId,
    rssi: i8,
    air: bool,
    cable: Option<Millis>,
    wants: bool,
    seen: Millis,
    chi: Chi,
    name: String<NAME_MAX>,
}

impl Peer {
    /// Its badge id.
    pub fn id(&self) -> PeerId {
        self.id
    }

    /// How it is heard now: the cable while it carried it recently.
    pub fn via(&self, now: Millis) -> Via {
        let on_cable = self
            .cable
            .is_some_and(|t| now.0.saturating_sub(t.0) < LINK_TIMEOUT_MS);
        if on_cable || !self.air {
            Via::Link
        } else {
            Via::Ble
        }
    }

    /// Signal strength of its last advert, 0 while shown on the cable.
    pub fn rssi(&self, now: Millis) -> i8 {
        match self.via(now) {
            Via::Link => 0,
            Via::Ble => self.rssi,
        }
    }

    /// True if its last packet addressed us (it requests a trade).
    pub fn wants(&self) -> bool {
        self.wants
    }

    /// The Chi its lobby note offers, UNSPECIFIED if unknown.
    pub fn chi(&self) -> Chi {
        self.chi
    }

    /// Display name from an overheard BLAT profile, empty if none.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Time it was last heard.
    pub fn seen(&self) -> Millis {
        self.seen
    }
}

/// The badges heard in one game's lobby. See the module docs.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Lobby {
    peers: Vec<Peer, LOBBY_MAX>,
}

impl Lobby {
    /// An empty lobby.
    pub fn new() -> Lobby {
        Lobby { peers: Vec::new() }
    }

    /// Records a packet from `p.sender` heard at `now` (stock
    /// `handle_packet`'s table update; the caller filters the game and our
    /// own id). `to_me` = it addressed us.
    pub fn heard(mut self, now: Millis, via: Via, rssi: i8, p: &Packet<'_>, to_me: bool) -> Lobby {
        let at = match self.peers.iter().position(|e| e.id == p.sender) {
            Some(i) => Some(i),
            None => {
                let fresh = Peer {
                    id: p.sender,
                    rssi: 0,
                    air: false,
                    cable: None,
                    wants: false,
                    seen: now,
                    chi: Chi::Unspecified,
                    name: String::new(),
                };
                if self.peers.push(fresh.clone()).is_ok() {
                    Some(self.peers.len() - 1)
                } else {
                    // Full: replace the badge heard longest ago.
                    let oldest = self
                        .peers
                        .iter()
                        .enumerate()
                        .min_by_key(|(_, e)| e.seen)
                        .map(|(i, _)| i);
                    if let Some(slot) = oldest.and_then(|i| self.peers.get_mut(i)) {
                        *slot = fresh;
                    }
                    oldest
                }
            }
        };
        if let Some(e) = at.and_then(|i| self.peers.get_mut(i)) {
            match via {
                Via::Link => e.cable = Some(now),
                Via::Ble => {
                    e.air = true;
                    e.rssi = rssi;
                }
            }
            if let Some(note) = p.note() {
                e.chi = chi_from_tag(note.as_bytes()).unwrap_or(Chi::Unspecified);
            }
            e.wants = to_me;
            e.seen = now;
        }
        self
    }

    /// Sets the display name of `id` if it is in the table.
    pub fn named(mut self, id: PeerId, name: &str) -> Lobby {
        if let Some(e) = self.peers.iter_mut().find(|e| e.id == id) {
            let mut n = String::new();
            for c in name.chars() {
                if n.push(c).is_err() {
                    break;
                }
            }
            e.name = n;
        }
        self
    }

    /// Drops badges silent for [`NEARBY_MS`] or more.
    pub fn expire(mut self, now: Millis) -> Lobby {
        self.peers
            .retain(|e| now.0.saturating_sub(e.seen.0) < NEARBY_MS);
        self
    }

    /// The entry for `id`, if heard (expired or not).
    pub fn get(&self, id: PeerId) -> Option<&Peer> {
        self.peers.iter().find(|e| e.id == id)
    }

    /// Badges heard within [`NEARBY_MS`] of `now`, in stock `players()`
    /// order: those requesting us first, then by id.
    pub fn players(&self, now: Millis) -> Vec<Peer, LOBBY_MAX> {
        let mut rows: Vec<Peer, LOBBY_MAX> = self
            .peers
            .iter()
            .filter(|e| now.0.saturating_sub(e.seen.0) < NEARBY_MS)
            .cloned()
            .collect();
        rows.sort_unstable_by_key(|e| (!e.wants, e.id));
        rows
    }

    /// Number of badges heard within [`NEARBY_MS`] of `now`.
    pub fn len(&self, now: Millis) -> usize {
        self.peers
            .iter()
            .filter(|e| now.0.saturating_sub(e.seen.0) < NEARBY_MS)
            .count()
    }

    /// True if nobody is heard within [`NEARBY_MS`] of `now`.
    pub fn is_empty(&self, now: Millis) -> bool {
        self.len(now) == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(s: &str) -> PeerId {
        PeerId::parse(s.as_bytes()).expect("id")
    }

    #[test]
    fn fr25_parse_needs_exactly_four_parts() {
        let p = parse(b"OZGT:0ac8f6:------:~VIB").expect("lobby");
        assert_eq!(p.game, 'T');
        assert_eq!(p.sender, id("0AC8F6"));
        assert_eq!(p.to, None);
        assert_eq!(p.payload, "~VIB");
        assert!(p.is_note());
        assert_eq!(p.note(), Some("VIB"));
        let p = parse(b"OZGT:0AC8F6:1a2b3c:VIB1").expect("linked");
        assert_eq!(p.to, Some(id("1A2B3C")));
        assert!(!p.is_note());
        // Empty payload is fine; three or five parts are not.
        assert_eq!(parse(b"OZGT:0AC8F6:------:").map(|p| p.payload), Some(""));
        assert_eq!(parse(b"OZGT:0AC8F6:------"), None);
        assert_eq!(parse(b"OZGT:0AC8F6:------:~VIB:x"), None);
        // Head, id lengths, prefix, UTF-8, hex sender.
        assert_eq!(parse(b"OZGTT:0AC8F6:------:~VIB"), None);
        assert_eq!(parse(b"OZGT:0AC8F:------:~VIB"), None);
        assert_eq!(parse(b"OZGT:0AC8F6:-----:~VIB"), None);
        assert_eq!(parse(b"OZNT:0AC8F6:------:~VIB"), None);
        assert_eq!(parse(b"OZG"), None);
        assert_eq!(parse(b"OZGT:0AC8F6:------:~V\xff"), None);
        assert_eq!(parse(b"OZGT:0AC8FZ:------:~VIB"), None);
        // Long notes are cut to NOTE_MAX like stock `payload[1:4]`.
        assert_eq!(parse(b"OZGT:0AC8F6:------:~VIBXX").and_then(|p| p.note()), Some("VIB"));
    }

    #[test]
    fn fr25_pack_golden_and_limits() {
        let me = id("0AC8F6");
        let p = pack(GAME_TRADE, me, None, b"~VIB").expect("fits");
        assert_eq!(&p[..], b"OZGT:0AC8F6:------:~VIB");
        assert_eq!(p.len(), PACKET_MAX);
        let p = pack(GAME_TRADE, me, Some(id("1A2B3C")), b"HCK1").expect("fits");
        assert_eq!(&p[..], b"OZGT:0AC8F6:1A2B3C:HCK1");
        assert_eq!(pack(GAME_TRADE, me, None, b"~VIBX"), Err(PackError::BadPayload));
        assert_eq!(pack(GAME_TRADE, me, None, b"a:b"), Err(PackError::BadPayload));
        assert_eq!(pack('é', me, None, b""), Err(PackError::BadGame));
        assert_eq!(PAYLOAD_MAX, 4);
        assert_eq!(NOTE_MAX, 3);
    }

    #[test]
    fn fr25_lobby_rows_expire_and_order_requests_first() {
        let me = id("0AC8F6");
        let mut lobby = Lobby::new();
        for (i, s) in ["CCCCCC", "AAAAAA", "BBBBBB"].iter().enumerate() {
            let pkt: std::vec::Vec<u8> = if *s == "BBBBBB" {
                std::format!("OZGT:{s}:0AC8F6:~PKT").into_bytes()
            } else {
                std::format!("OZGT:{s}:------:~HCK").into_bytes()
            };
            let p = parse(&pkt).expect("lobby");
            let to_me = p.to == Some(me);
            lobby = lobby.heard(Millis(i as u64 * 1000), Via::Ble, -40, &p, to_me);
        }
        let rows = lobby.players(Millis(2500));
        let ids: std::vec::Vec<PeerId> = rows.iter().map(|r| r.id()).collect();
        assert_eq!(ids, [id("BBBBBB"), id("AAAAAA"), id("CCCCCC")]);
        assert_eq!(rows[0].chi(), Chi::Pkt);
        assert!(rows[0].wants());
        assert_eq!(rows[1].rssi(Millis(2500)), -40);
        // CCCCCC was heard at 0: gone at 4000, AAAAAA at 1000: gone at 5000.
        assert_eq!(lobby.len(Millis(3999)), 3);
        assert_eq!(lobby.len(Millis(4000)), 2);
        let lobby = lobby.expire(Millis(5000));
        assert_eq!(lobby.len(Millis(5000)), 1);
        assert!(lobby.get(id("CCCCCC")).is_none());
    }

    #[test]
    fn fr25_lobby_via_link_then_ble_and_full_table_replaces_oldest() {
        let p = parse(b"OZGT:111111:------:~VIB").expect("lobby");
        let lobby = Lobby::new().heard(Millis(0), Via::Link, 0, &p, false);
        assert_eq!(lobby.get(p.sender).map(|e| e.via(Millis(0))), Some(Via::Link));
        let lobby = lobby.heard(Millis(100), Via::Ble, -50, &p, false);
        // Cable heard within 3 s: still LINK, rssi 0.
        assert_eq!(lobby.get(p.sender).map(|e| e.via(Millis(2999))), Some(Via::Link));
        assert_eq!(lobby.get(p.sender).map(|e| e.rssi(Millis(2999))), Some(0));
        assert_eq!(lobby.get(p.sender).map(|e| e.via(Millis(3000))), Some(Via::Ble));
        assert_eq!(lobby.get(p.sender).map(|e| e.rssi(Millis(3000))), Some(-50));
        // Fill the table, then one more replaces the oldest (111111 at 100).
        let mut lobby = lobby;
        for i in 2..=9u32 {
            let s = std::format!("OZGT:{:06X}:------:~ADM", i * 0x111111);
            let q = parse(s.as_bytes()).expect("lobby");
            lobby = lobby.heard(Millis(100 + u64::from(i)), Via::Ble, -30, &q, false);
        }
        assert_eq!(lobby.len(Millis(200)), LOBBY_MAX);
        assert!(lobby.get(p.sender).is_none());
        // A note-less payload keeps the old note; a name sticks.
        let q = parse(b"OZGT:222222:------:").expect("lobby");
        let lobby = lobby.heard(Millis(300), Via::Ble, -30, &q, false).named(q.sender, "Ruffles");
        let e = lobby.get(q.sender).expect("kept");
        assert_eq!(e.chi(), Chi::Adm);
        assert_eq!(e.name(), "Ruffles");
    }

    extern crate std;
}
