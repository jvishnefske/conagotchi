//! Contacts: badges we completed a Chi trade with.
//!
//! The stock firmware has no contacts feature; the nearest thing is
//! `peers.txt` (ids of completed trades). We keep, per peer id, the Chi last
//! received, the trade count and a display name if an `OZN` profile was
//! overheard. At most [`MAX_CONTACTS`], unique by id; the firmware persists
//! each change (`Effect::SaveContact`).
//!
//! The list is kept in trade-recency order, least recent first:
//! - [`Contacts::record_trade`] (a completed trade) moves the contact to
//!   the end, adding it if new; when the list is full, a new contact evicts
//!   the *least recently traded* one (the front). The store applies the
//!   same policy: a `SaveContact` of a new id into a full store replaces
//!   the contact saved longest ago.
//! - [`Contacts::named`] (an overheard name) updates in place.
//! - [`Contacts::with`] (boot-time load) inserts in the given order.
//!
//! Display and paging ([`Contacts::page`]) go newest first.

use badge_proto::Chi;
use heapless::{String, Vec};

use crate::oz::PeerId;

/// Most contacts stored.
pub const MAX_CONTACTS: usize = 64;
/// Longest display name kept.
pub const NAME_LEN: usize = 8;
/// Contacts per `ListContacts` page.
pub const PAGE: usize = 8;

/// One contact.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Contact {
    /// Peer badge id.
    pub id: PeerId,
    /// Chi received in the last completed trade.
    pub chi: Chi,
    /// Display name from an overheard `OZN` profile; empty if unknown.
    pub name: String<NAME_LEN>,
    /// Completed trades with this badge.
    pub trades: u16,
}

impl Contact {
    /// The protocol form of this contact.
    pub fn to_proto(&self) -> badge_proto::Contact {
        let mut id = String::new();
        id.push_str(self.id.as_str()).ok();
        badge_proto::Contact {
            id,
            chi: self.chi,
            name: self.name.clone(),
            trades: u32::from(self.trades),
        }
    }
}

/// Copies as much of `s` as fits a name, never splitting a char.
fn name_of(s: &str) -> String<NAME_LEN> {
    let mut out = String::new();
    for c in s.chars() {
        if out.push(c).is_err() {
            break;
        }
    }
    out
}

/// The contact list, in trade-recency order, unique by id.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Contacts {
    list: Vec<Contact, MAX_CONTACTS>,
}

impl Contacts {
    /// An empty list.
    pub fn new() -> Contacts {
        Contacts { list: Vec::new() }
    }

    /// All contacts, least recently traded first.
    pub fn as_slice(&self) -> &[Contact] {
        &self.list
    }

    /// Number of contacts.
    pub fn len(&self) -> usize {
        self.list.len()
    }

    /// True if there are none.
    pub fn is_empty(&self) -> bool {
        self.list.is_empty()
    }

    /// The contact with this id.
    pub fn get(&self, id: PeerId) -> Option<&Contact> {
        self.list.iter().find(|c| c.id == id)
    }

    /// Adds `contact` (or replaces the one with its id, in place) without
    /// counting a trade: for loading the stored list. A new id in a full
    /// list evicts the front.
    pub fn with(mut self, contact: Contact) -> Contacts {
        if let Some(slot) = self.list.iter_mut().find(|c| c.id == contact.id) {
            *slot = contact;
            return self;
        }
        self.push_back(contact);
        self
    }

    /// A completed trade with `id` that brought `chi`: trades + 1, Chi
    /// updated, name kept (or set from `name` if non-empty), moved to the
    /// most recent end. Returns the list and the contact to save.
    pub fn record_trade(mut self, id: PeerId, chi: Chi, name: &str) -> (Contacts, Contact) {
        let old = self
            .list
            .iter()
            .position(|c| c.id == id)
            .map(|i| self.list.remove(i));
        let mut contact = old.unwrap_or(Contact {
            id,
            chi,
            name: String::new(),
            trades: 0,
        });
        contact.chi = chi;
        contact.trades = contact.trades.saturating_add(1);
        if !name.is_empty() {
            contact.name = name_of(name);
        }
        self.push_back(contact.clone());
        (self, contact)
    }

    /// An overheard display name for `id`: returns the updated contact if
    /// `id` is a contact and the (non-empty) name differs.
    pub fn named(mut self, id: PeerId, name: &str) -> (Contacts, Option<Contact>) {
        let name = name_of(name);
        if name.is_empty() {
            return (self, None);
        }
        let changed = self
            .list
            .iter_mut()
            .find(|c| c.id == id && c.name != name)
            .map(|c| {
                c.name = name;
                c.clone()
            });
        (self, changed)
    }

    /// Contacts newest first.
    pub fn newest_first(&self) -> impl Iterator<Item = &Contact> {
        self.list.iter().rev()
    }

    /// Up to [`PAGE`] contacts starting at `offset` in newest-first order.
    pub fn page(&self, offset: usize) -> impl Iterator<Item = &Contact> {
        self.newest_first().skip(offset).take(PAGE)
    }

    fn push_back(&mut self, contact: Contact) {
        if self.list.is_full() && !self.list.is_empty() {
            self.list.remove(0);
        }
        self.list.push(contact).ok();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    extern crate std;
    use std::vec::Vec as StdVec;

    fn id(n: u32) -> PeerId {
        PeerId::from_u32(n)
    }

    #[test]
    fn fr26_record_trade_upserts_by_id() {
        let (c, saved) = Contacts::new().record_trade(id(1), Chi::Vib, "");
        assert_eq!(saved.trades, 1);
        let (c, _) = c.record_trade(id(2), Chi::Hck, "Bob");
        let (c, _) = c.named(id(1), "Ruffles");
        let (c, saved) = c.record_trade(id(1), Chi::Pkt, "");
        assert_eq!(c.len(), 2);
        assert_eq!(saved.trades, 2);
        assert_eq!(saved.chi, Chi::Pkt);
        assert_eq!(saved.name.as_str(), "Ruffles", "name kept");
        // Most recent first.
        let order: StdVec<PeerId> = c.newest_first().map(|c| c.id).collect();
        assert_eq!(order, [id(1), id(2)]);
        assert_eq!(c.get(id(2)).map(|c| c.name.as_str()), Some("Bob"));
    }

    #[test]
    fn fr26_full_list_evicts_least_recently_traded() {
        let mut c = Contacts::new();
        for n in 0..MAX_CONTACTS as u32 {
            c = c.record_trade(id(n), Chi::Vib, "").0;
        }
        // Trade again with 0: it becomes the most recent, so 1 is evicted next.
        let (c, _) = c.record_trade(id(0), Chi::Adm, "");
        let (c, _) = c.record_trade(id(1000), Chi::Cry, "");
        assert_eq!(c.len(), MAX_CONTACTS);
        assert!(c.get(id(1)).is_none());
        assert!(c.get(id(0)).is_some());
        assert_eq!(c.newest_first().next().map(|c| c.id), Some(id(1000)));
        // Loading into a full list evicts the front too.
        let c = c.with(Contact { id: id(2000), ..Default::default() });
        assert!(c.get(id(2)).is_none());
        assert_eq!(c.len(), MAX_CONTACTS);
    }

    #[test]
    fn fr26_named_updates_only_known_contacts_and_changes() {
        let (c, _) = Contacts::new().record_trade(id(1), Chi::Vib, "");
        let (c, changed) = c.named(id(1), "A-very-long-name");
        assert_eq!(changed.map(|c| c.name), Some(name_of("A-very-l")));
        let (c, changed) = c.named(id(1), "A-very-long-name");
        assert_eq!(changed, None, "same name: nothing to save");
        let (c, changed) = c.named(id(9), "Eve");
        assert_eq!(changed, None);
        let (_, changed) = c.named(id(1), "");
        assert_eq!(changed, None);
    }

    #[test]
    fn fr26_pages_newest_first() {
        let mut c = Contacts::new();
        for n in 0..20 {
            c = c.record_trade(id(n), Chi::Vib, "").0;
        }
        let p0: StdVec<PeerId> = c.page(0).map(|c| c.id).collect();
        assert_eq!(p0.len(), PAGE);
        assert_eq!(p0[0], id(19));
        let p2: StdVec<PeerId> = c.page(16).map(|c| c.id).collect();
        assert_eq!(p2, [id(3), id(2), id(1), id(0)]);
        assert_eq!(c.page(20).count(), 0);
        assert_eq!(c.page(usize::MAX).count(), 0);
        let proto = c.page(0).next().map(Contact::to_proto).expect("one");
        assert_eq!(proto.id.as_str(), "000013");
        assert_eq!(proto.trades, 1);
    }
}
