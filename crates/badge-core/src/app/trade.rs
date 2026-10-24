//! Chi trade and contacts inside the app: the lobby, the current trade, the
//! TRADE / CONTACTS screens, and their host requests.
//!
//! The protocol logic lives in [`crate::oz`]; this module feeds it inputs and
//! turns its outcomes into effects:
//!
//! - **Tick** ([`App::trade_tick`]): lobby timers, then our beacon when due.
//!   The beacon packet is signed (if a key is loaded), sent on the link UART
//!   every time (`LinkSend`, a frame is one-shot) and given to the BLE
//!   advertiser only when the advertising data changed (`Beacon`; the shell
//!   keeps re-advertising it at 100 ms). A trade event costs one effect, a
//!   completed trade three (`TradeEvent`, `SaveContact`, `ContactAdded`);
//!   the beacon goes out only if its worst case (2) still fits
//!   [`TRADE_TICK_EFFECTS`], else on the next tick.
//! - **Radio packets** ([`App::radio_packet`]): BLE bodies are rebuilt from
//!   the kind byte ([`crate::oz::adv::decode_body`]), link payloads are used
//!   as they are; the tag is checked ([`crate::oz::sign::verify`]); lobby
//!   packets below −55 dBm over BLE are dropped; `OZN` profiles (any RSSI)
//!   name lobby rows and contacts (`SaveContact` when a name changes).
//! - **Buttons** ([`App::trade_button`]): on TRADE, SELECT enters pick mode;
//!   in pick mode LEFT / RIGHT move the cursor, SELECT invites the picked
//!   badge (or cancels if it is the one already invited) and START backs
//!   out; outside pick mode START cancels the current trade and LEFT /
//!   RIGHT navigate. On CONTACTS, SELECT shows the next page of
//!   [`CONTACT_ROWS`] and LEFT / RIGHT navigate.
//! - **Host** ([`App::trade_host`]): `SetChi`, `TradeInvite`, `TradeCancel`,
//!   `ListContacts`, `ClearContacts`, `GetLobby`, each answered exactly once.
//!   An invite to a badge not (yet) in the lobby is allowed, like stock.
//!
//! The TRADE screen is redrawn when what it shows changes (rows, RSSI in
//! 8 dB steps, state, pick cursor, key), never just because time passed.

use badge_proto::{
    Button, Chi, ContactAdded, ErrorCode, Event_, HostMsg_::Cmd, Lobby, LobbyPeer, Response_::Body,
    Screen, TradeEvent, TradeState,
};
use heapless::{String, Vec};

use super::{App, Out};
use crate::contacts::{self, Contacts};
use crate::io::{Effect, MAX_ADV_DATA, Via};
use crate::oz::lobby::{self, ADVERT_MAX, LOBBY_MAX, RSSI_MIN};
use crate::oz::trade::{ChiError, Event, InviteError, Session};
use crate::oz::{PeerId, adv, link, parse_profile, sign, sign::RadioKey};

/// Most effects one tick of the trade lobby may emit (see the `app` budget).
pub const TRADE_TICK_EFFECTS: usize = 3;
/// Contacts shown per CONTACTS screen page.
pub const CONTACT_ROWS: usize = 6;

/// One lobby row as shown and reported.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LobbyRow {
    /// Badge id.
    pub id: PeerId,
    /// Signal strength (0 on the cable).
    pub rssi: i8,
    /// How it is heard.
    pub via: Via,
    /// The Chi it offers, UNSPECIFIED if unknown.
    pub chi: Chi,
    /// It requests a trade with us.
    pub wants: bool,
    /// We invited it (or are trading with it).
    pub invited: bool,
}

/// Lobby, current trade and TRADE / CONTACTS screen state.
#[derive(Debug, Clone, PartialEq)]
pub struct TradeModel {
    session: Session,
    pick: bool,
    cursor: u8,
    page: u8,
    /// Advertising data of the last `Beacon` effect (empty before the first).
    adv: Vec<u8, MAX_ADV_DATA>,
}

impl TradeModel {
    /// Idle model for the badge with id `me`, offering VIB.
    pub fn new(me: PeerId) -> TradeModel {
        TradeModel {
            session: Session::new(me, Chi::Vib),
            pick: false,
            cursor: 0,
            page: 0,
            adv: Vec::new(),
        }
    }

    /// Our badge id on air.
    pub fn me(&self) -> PeerId {
        self.session.me()
    }

    /// The Chi we offer.
    pub fn chi(&self) -> Chi {
        self.session.chi()
    }

    /// State of the current (or last) trade.
    pub fn state(&self) -> TradeState {
        self.session.state()
    }

    /// Peer of the current (or last) trade.
    pub fn peer(&self) -> Option<PeerId> {
        self.session.peer()
    }

    /// Chi received in the current or last completed trade.
    pub fn received(&self) -> Chi {
        self.session.received()
    }

    /// True if the linked peer offers a tag we do not know.
    pub fn unknown_tag(&self) -> bool {
        self.session.unknown_tag()
    }

    /// The protocol state machine.
    pub fn session(&self) -> &Session {
        &self.session
    }

    /// Badges currently in the lobby.
    pub fn lobby_len(&self) -> usize {
        self.session.lobby().len(self.session.now())
    }

    /// Lobby rows in display order (requests first, then by id).
    pub fn rows(&self) -> Vec<LobbyRow, LOBBY_MAX> {
        let now = self.session.now();
        let invited = self.session.active_peer();
        self.session
            .lobby()
            .players(now)
            .iter()
            .map(|p| LobbyRow {
                id: p.id(),
                rssi: p.rssi(now),
                via: p.via(now),
                chi: p.chi(),
                wants: p.wants(),
                invited: invited == Some(p.id()),
            })
            .collect()
    }

    /// Pick-mode cursor (row index, clamped to the rows), `None` outside
    /// pick mode.
    pub fn picking(&self) -> Option<usize> {
        self.pick.then(|| {
            usize::from(self.cursor).min(self.lobby_len().saturating_sub(1))
        })
    }

    /// CONTACTS page shown (0-based, [`CONTACT_ROWS`] per page).
    pub fn contacts_page(&self) -> usize {
        usize::from(self.page)
    }

    /// FNV-1a over everything the TRADE screen shows.
    fn view_key(&self, signed: bool) -> u32 {
        let mut h: u32 = 0x811C_9DC5;
        let mut eat = |b: u8| {
            h ^= u32::from(b);
            h = h.wrapping_mul(0x0100_0193);
        };
        for b in self.me().as_bytes() {
            eat(*b);
        }
        for v in [
            self.chi().0,
            self.state().0,
            self.received().0,
            i32::from(signed),
            i32::from(self.unknown_tag()),
            self.picking().map_or(-1, |c| i32::try_from(c).unwrap_or(0)),
            self.peer().map_or(-1, |p| i32::try_from(p.to_u32()).unwrap_or(0)),
            i32::from(self.session.active_peer().is_some()),
        ] {
            for b in v.to_le_bytes() {
                eat(b);
            }
        }
        for r in self.rows() {
            for b in r.id.as_bytes() {
                eat(*b);
            }
            eat(r.rssi.div_euclid(8) as u8);
            eat(u8::from(r.via == Via::Link));
            eat(r.chi.0 as u8);
            eat(u8::from(r.wants) | (u8::from(r.invited) << 1));
        }
        h
    }
}

/// Number of CONTACTS pages for `len` contacts (at least 1).
fn contact_pages(len: usize) -> usize {
    len.div_ceil(CONTACT_ROWS).max(1)
}

impl App {
    /// Lobby timers: beacons, expiry, trade timeouts. At most
    /// [`TRADE_TICK_EFFECTS`] effects.
    pub(super) fn trade_tick(mut self, out: &mut Out) -> Self {
        let before = self.trade.view_key(self.signed());
        let now = self.now;
        let (session, events) = self.trade.session.tick(now);
        self.trade.session = session;
        let mut used = 0;
        // A tick changes the state at most once (one timer fires).
        if let Some(&e) = events.last() {
            used += self.trade_event(e, out);
        }
        if self.trade.session.beacon_due(now) && used + 2 <= TRADE_TICK_EFFECTS {
            let (session, packet) = self.trade.session.beacon(now);
            self.trade.session = session;
            self = self.send_beacon(&packet, out);
        }
        self.trade_redraw(before, out);
        self
    }

    /// Signs `packet`, frames it for the link and updates the advert.
    fn send_beacon(mut self, packet: &[u8], out: &mut Out) -> Self {
        let Ok(signed) = sign::sign::<ADVERT_MAX>(self.key.as_ref(), packet) else {
            return self;
        };
        if let Ok(ad) = adv::encode(&signed)
            && ad != self.trade.adv
        {
            self.trade.adv = ad.clone();
            out.push(Effect::Beacon(ad));
        }
        if let Ok(frame) = link::encode(&signed) {
            out.push(Effect::LinkSend(frame));
        }
        self
    }

    /// Reports a trade state change; a completed trade also saves the
    /// contact. Returns the number of effects emitted.
    fn trade_event(&mut self, e: Event, out: &mut Out) -> usize {
        let mut peer = String::new();
        peer.push_str(e.peer.as_str()).ok();
        out.event(Event_::Kind::Trade(TradeEvent {
            peer,
            state: e.state,
            chi: e.chi,
        }));
        if e.state != TradeState::Complete {
            return 1;
        }
        let name: String<{ contacts::NAME_LEN }> = self
            .trade
            .session
            .lobby()
            .get(e.peer)
            .map(|p| {
                let mut n = String::new();
                n.push_str(p.name()).ok();
                n
            })
            .unwrap_or_default();
        let (list, contact) =
            core::mem::take(&mut self.contacts).record_trade(e.peer, e.chi, &name);
        self.contacts = list;
        out.event(Event_::Kind::ContactAdded(ContactAdded {
            contact: Some(contact.to_proto()),
        }));
        out.push(Effect::SaveContact(contact));
        self.redraw_if_on(Screen::Contacts, out);
        3
    }

    /// Redraws TRADE if what it shows differs from `before`.
    fn trade_redraw(&self, before: u32, out: &mut Out) {
        if self.trade.view_key(self.signed()) != before {
            self.redraw_if_on(Screen::Trade, out);
        }
    }

    /// A stock-protocol packet arrived.
    pub(super) fn radio_packet(mut self, via: Via, rssi: i8, payload: &[u8], out: &mut Out) -> Self {
        let before = self.trade.view_key(self.signed());
        let packet = match via {
            Via::Ble => adv::decode_body(payload),
            Via::Link => Vec::from_slice(payload).ok(),
        };
        let Some(packet) = packet else {
            return self;
        };
        let Ok(body) = sign::verify(self.key.as_ref(), &packet) else {
            return self;
        };
        if let Some(p) = lobby::parse(body) {
            if via == Via::Ble && rssi < RSSI_MIN {
                return self;
            }
            let (session, events) = self.trade.session.on_packet(self.now, via, rssi, &p);
            self.trade.session = session;
            for &e in &events {
                self.trade_event(e, out);
            }
        } else if let Some(profile) = parse_profile(body)
            && profile.sender != self.trade.me()
            && !profile.name.is_empty()
        {
            self.trade.session = self.trade.session.named(profile.sender, &profile.name);
            let (list, changed) =
                core::mem::take(&mut self.contacts).named(profile.sender, &profile.name);
            self.contacts = list;
            if let Some(contact) = changed {
                out.push(Effect::SaveContact(contact));
                self.redraw_if_on(Screen::Contacts, out);
            }
        }
        self.trade_redraw(before, out);
        self
    }

    /// Storage finished loading: the contacts arrived one by one before
    /// this, the radio key comes with it.
    pub(super) fn storage_loaded(mut self, key: Option<RadioKey>, out: &mut Out) -> Self {
        self.key = key;
        self.trade.page = 0;
        self.redraw_if_on(Screen::Trade, out);
        self.redraw_if_on(Screen::Contacts, out);
        self
    }

    /// A button pressed on TRADE / CONTACTS. `Ok` = handled here, `Err` =
    /// fall through to normal navigation.
    // `Result<Self, Self>` hands the app back either way (no copy, no heap).
    #[allow(clippy::result_large_err)]
    pub(super) fn trade_button(mut self, button: Button, out: &mut Out) -> Result<Self, Self> {
        let before = self.trade.view_key(self.signed());
        match self.screen {
            Screen::Trade => {
                let rows = self.trade.rows();
                let len = rows.len();
                match (self.trade.pick, button) {
                    (false, Button::Select) => {
                        self.trade.pick = true;
                        self.trade.cursor = 0;
                    }
                    (false, Button::Start) if self.trade.session.active_peer().is_some() => {
                        self = self.trade_cancel(out);
                    }
                    (false, _) => return Err(self),
                    (true, Button::Left | Button::Right) => {
                        if len > 0 {
                            let at = usize::from(self.trade.cursor).min(len - 1);
                            let next = if button == Button::Left {
                                (at + len - 1) % len
                            } else {
                                (at + 1) % len
                            };
                            self.trade.cursor = u8::try_from(next).unwrap_or(0);
                        }
                    }
                    (true, Button::Select) => {
                        self.trade.pick = false;
                        let at = usize::from(self.trade.cursor);
                        if let Some(row) = rows.get(at).or(rows.last()) {
                            if row.invited {
                                self = self.trade_cancel(out);
                            } else {
                                let (session, result) =
                                    self.trade.session.invite(self.now, row.id);
                                self.trade.session = session;
                                if let Ok(events) = result {
                                    for &e in &events {
                                        self.trade_event(e, out);
                                    }
                                }
                            }
                        }
                    }
                    (true, Button::Start) => self.trade.pick = false,
                    (true, _) => return Err(self),
                }
                self.trade_redraw(before, out);
                Ok(self)
            }
            Screen::Contacts if button == Button::Select => {
                let pages = contact_pages(self.contacts.len());
                let next = (usize::from(self.trade.page) + 1) % pages;
                self.trade.page = u8::try_from(next).unwrap_or(0);
                out.redraw();
                Ok(self)
            }
            _ => Err(self),
        }
    }

    /// Leaves the current trade, reporting it.
    fn trade_cancel(mut self, out: &mut Out) -> Self {
        let (session, events) = self.trade.session.cancel(self.now);
        self.trade.session = session;
        for &e in &events {
            self.trade_event(e, out);
        }
        self
    }

    /// SetChi, TradeInvite, TradeCancel, ListContacts, ClearContacts,
    /// GetLobby. Answers exactly one response.
    pub(super) fn trade_host(mut self, seq: u32, cmd: Cmd, out: &mut Out) -> Self {
        let before = self.trade.view_key(self.signed());
        match cmd {
            Cmd::SetChi(c) => {
                let (session, result) = self.trade.session.set_chi(c.chi);
                self.trade.session = session;
                match result {
                    Ok(()) => out.ack(seq),
                    Err(ChiError::Unknown) => out.error(seq, ErrorCode::BadArgument, "chi"),
                    Err(ChiError::Linked) => out.error(seq, ErrorCode::Busy, "trading"),
                }
            }
            Cmd::TradeInvite(t) => match PeerId::parse(t.peer.as_bytes()) {
                Err(_) => out.error(seq, ErrorCode::BadArgument, "peer id"),
                Ok(peer) => {
                    let (session, result) = self.trade.session.invite(self.now, peer);
                    self.trade.session = session;
                    match result {
                        Ok(events) => {
                            out.ack(seq);
                            for &e in &events {
                                self.trade_event(e, out);
                            }
                        }
                        Err(InviteError::Myself) => {
                            out.error(seq, ErrorCode::BadArgument, "own id")
                        }
                        Err(InviteError::Linked) => out.error(seq, ErrorCode::Busy, "trading"),
                    }
                }
            },
            Cmd::TradeCancel(_) => {
                out.ack(seq);
                self = self.trade_cancel(out);
            }
            Cmd::ListContacts(l) => {
                let offset = usize::try_from(l.offset).unwrap_or(usize::MAX);
                let page = self.contacts.page(offset).map(|c| c.to_proto()).collect();
                out.respond(
                    seq,
                    Body::Contacts(badge_proto::Contacts {
                        contacts: page,
                        total: u32::try_from(self.contacts.len()).unwrap_or(u32::MAX),
                        offset: l.offset,
                    }),
                );
            }
            Cmd::ClearContacts(_) => {
                self.contacts = Contacts::new();
                self.trade.page = 0;
                out.push(Effect::ClearContacts);
                out.ack(seq);
                self.redraw_if_on(Screen::Contacts, out);
            }
            Cmd::GetLobby(_) => out.respond(seq, Body::Lobby(self.lobby_body())),
            _ => out.error(seq, ErrorCode::Unsupported, "trade"),
        }
        self.trade_redraw(before, out);
        self
    }

    /// The `Lobby` response: peers strongest first.
    fn lobby_body(&self) -> Lobby {
        let mut rows = self.trade.rows();
        rows.sort_unstable_by_key(|r| (core::cmp::Reverse(r.rssi), r.id));
        let peers = rows
            .iter()
            .map(|r| {
                let mut id = String::new();
                id.push_str(r.id.as_str()).ok();
                LobbyPeer {
                    id,
                    rssi: i32::from(r.rssi),
                    via: match r.via {
                        Via::Ble => badge_proto::Via::Ble,
                        Via::Link => badge_proto::Via::Link,
                    },
                    chi: r.chi,
                }
            })
            .collect();
        let mut peer = String::new();
        if let Some(p) = self.trade.session.active_peer() {
            peer.push_str(p.as_str()).ok();
        }
        Lobby {
            peers,
            chi: self.trade.chi(),
            signed: self.signed(),
            state: self.trade.state(),
            peer,
        }
    }
}

#[cfg(test)]
mod tests;
