//! Record layout of the firmware's `conag` flash partition.
//!
//! The firmware keeps a `sequential-storage` key/value map in the `conag`
//! partition (0x200000, 8 KiB, the stock `otadata` slot). This module is the
//! pure half: map keys, value encodings, the boot-time fold of stored items
//! into contacts and a radio key ([`Loader`]), and the save order that picks
//! which contact a full store evicts ([`SaveOrder`]: the one saved longest
//! ago, matching `Contacts`). The firmware only moves bytes.
//!
//! Keys are `u32`:
//!
//! - [`KEY_RADIO`] (1): the radio key, [`KEY_RECORD_LEN`] bytes.
//! - [`CONTACT_BASE`] | 24-bit peer id: one contact, [`CONTACT_RECORD_LEN`]
//!   bytes.
//!
//! Every value starts with the format byte [`FORMAT`]; values with another
//! format, a wrong length or invalid fields are skipped (and counted), never
//! fatal. Little-endian throughout.
//!
//! Contact value: format, id (u32, low 24 bits), chi (i32), trades (u16),
//! name length (u8, at most [`NAME_LEN`]), name bytes (UTF-8, zero-padded to
//! [`NAME_LEN`]).

use badge_proto::Chi;
use heapless::{String, Vec};

use crate::contacts::{Contact, MAX_CONTACTS, NAME_LEN};
use crate::oz::PeerId;
use crate::oz::sign::{KEY_LEN, RadioKey};

/// Map key of the radio key.
pub const KEY_RADIO: u32 = 1;
/// Map key prefix of contacts; the low 24 bits are the peer id.
pub const CONTACT_BASE: u32 = 0x0100_0000;
/// Value format version.
pub const FORMAT: u8 = 1;
/// Length of an encoded contact value.
pub const CONTACT_RECORD_LEN: usize = 1 + 4 + 4 + 2 + 1 + NAME_LEN;
/// Length of an encoded radio key value.
pub const KEY_RECORD_LEN: usize = 1 + KEY_LEN;
/// Longest value of any record.
pub const MAX_RECORD: usize = if CONTACT_RECORD_LEN > KEY_RECORD_LEN { CONTACT_RECORD_LEN } else { KEY_RECORD_LEN };

const _: () = assert!(MAX_RECORD >= CONTACT_RECORD_LEN && MAX_RECORD >= KEY_RECORD_LEN);

/// Why a stored value was not accepted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecordError {
    /// Wrong length for its key.
    Length,
    /// Unknown format byte.
    Format,
    /// A field is out of range (name too long or not UTF-8, id mismatch).
    Field,
}

/// What a map key holds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Record {
    /// The radio key.
    RadioKey,
    /// The contact with this peer id.
    Contact(PeerId),
    /// Not ours (skipped).
    Unknown,
}

/// The map key of the contact `id`.
pub fn contact_key(id: PeerId) -> u32 {
    CONTACT_BASE | (id.to_u32() & 0x00FF_FFFF)
}

/// Classifies a map key.
pub fn record_of(key: u32) -> Record {
    if key == KEY_RADIO {
        Record::RadioKey
    } else if key & 0xFF00_0000 == CONTACT_BASE {
        Record::Contact(PeerId::from_u32(key))
    } else {
        Record::Unknown
    }
}

/// Encodes the radio key value.
pub fn encode_key(key: &RadioKey) -> [u8; KEY_RECORD_LEN] {
    let mut out = [0u8; KEY_RECORD_LEN];
    out[0] = FORMAT;
    out[1..].copy_from_slice(&key.0);
    out
}

/// Decodes a radio key value.
///
/// # Errors
///
/// [`RecordError`] on a wrong length or format.
pub fn decode_key(value: &[u8]) -> Result<RadioKey, RecordError> {
    let (&format, rest) = value.split_first().ok_or(RecordError::Length)?;
    if format != FORMAT {
        return Err(RecordError::Format);
    }
    let bytes: [u8; KEY_LEN] = rest.try_into().map_err(|_| RecordError::Length)?;
    Ok(RadioKey(bytes))
}

/// Encodes a contact value.
pub fn encode_contact(c: &Contact) -> [u8; CONTACT_RECORD_LEN] {
    let mut out = [0u8; CONTACT_RECORD_LEN];
    out[0] = FORMAT;
    out[1..5].copy_from_slice(&c.id.to_u32().to_le_bytes());
    out[5..9].copy_from_slice(&c.chi.0.to_le_bytes());
    out[9..11].copy_from_slice(&c.trades.to_le_bytes());
    let name = c.name.as_bytes();
    let n = name.len().min(NAME_LEN);
    out[11] = n as u8;
    out[12..12 + n].copy_from_slice(&name[..n]);
    out
}

/// Decodes a contact value.
///
/// # Errors
///
/// [`RecordError`] on a wrong length or format, an id above 24 bits, or a
/// name that is too long or not UTF-8.
pub fn decode_contact(value: &[u8]) -> Result<Contact, RecordError> {
    let v: &[u8; CONTACT_RECORD_LEN] = value.try_into().map_err(|_| RecordError::Length)?;
    if v[0] != FORMAT {
        return Err(RecordError::Format);
    }
    let id = u32::from_le_bytes([v[1], v[2], v[3], v[4]]);
    if id > 0x00FF_FFFF {
        return Err(RecordError::Field);
    }
    let chi = i32::from_le_bytes([v[5], v[6], v[7], v[8]]);
    let trades = u16::from_le_bytes([v[9], v[10]]);
    let n = usize::from(v[11]);
    let raw = v.get(12..12 + n).filter(|_| n <= NAME_LEN).ok_or(RecordError::Field)?;
    let text = core::str::from_utf8(raw).map_err(|_| RecordError::Field)?;
    let mut name = String::<NAME_LEN>::new();
    name.push_str(text).map_err(|_| RecordError::Field)?;
    Ok(Contact { id: PeerId::from_u32(id), chi: Chi(chi), name, trades })
}

/// Boot-time fold of the stored map items, oldest first (the order
/// `sequential-storage` iterates them): the newest value per key wins, and
/// contacts end up in save order, least recently saved first, the order
/// `Contacts` keeps (least recently traded first).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Loader {
    contacts: Vec<Contact, MAX_CONTACTS>,
    key: Option<RadioKey>,
    skipped: u16,
}

impl Loader {
    /// Nothing loaded yet.
    pub fn new() -> Loader {
        Loader::default()
    }

    /// Folds in one stored item. A contact already loaded moves to the end
    /// with its new value; a new contact into a full list evicts the front
    /// (the store never holds more than [`MAX_CONTACTS`], so that only
    /// happens after damage). Unknown keys, undecodable values and a
    /// contact whose value disagrees with its key are skipped and counted.
    #[must_use]
    pub fn push(self, key: u32, value: &[u8]) -> Loader {
        let Loader { contacts, key: radio, skipped } = self;
        let skip = |contacts, radio| Loader { contacts, key: radio, skipped: skipped.saturating_add(1) };
        match record_of(key) {
            Record::RadioKey => match decode_key(value) {
                Ok(k) => Loader { contacts, key: Some(k), skipped },
                Err(_) => skip(contacts, radio),
            },
            Record::Contact(id) => match decode_contact(value) {
                Ok(c) if c.id == id => Loader { contacts: to_back(contacts, c).0, key: radio, skipped },
                _ => skip(contacts, radio),
            },
            Record::Unknown => skip(contacts, radio),
        }
    }

    /// Contacts loaded so far, least recently saved first.
    pub fn contacts(&self) -> &[Contact] {
        &self.contacts
    }

    /// The radio key, if one was stored.
    pub fn key(&self) -> Option<&RadioKey> {
        self.key.as_ref()
    }

    /// Items skipped so far.
    pub fn skipped(&self) -> u16 {
        self.skipped
    }

    /// Takes the loaded contacts and key.
    pub fn finish(self) -> (Vec<Contact, MAX_CONTACTS>, Option<RadioKey>) {
        (self.contacts, self.key)
    }
}

/// Moves (or adds) `item` to the back of `list`, matching by id; a new id
/// into a full list evicts the front, which is returned.
fn to_back<T: HasId>(mut list: Vec<T, MAX_CONTACTS>, item: T) -> (Vec<T, MAX_CONTACTS>, Option<T>) {
    let id = item.peer();
    let mut evicted = None;
    if let Some(pos) = list.iter().position(|old| old.peer() == id) {
        list.remove(pos);
    } else if list.is_full() {
        evicted = Some(list.remove(0));
    }
    // There is room now: either an entry was removed or the list was not full.
    let _ = list.push(item);
    (list, evicted)
}

/// Things kept by peer id.
trait HasId {
    fn peer(&self) -> PeerId;
}

impl HasId for Contact {
    fn peer(&self) -> PeerId {
        self.id
    }
}

impl HasId for PeerId {
    fn peer(&self) -> PeerId {
        *self
    }
}

/// The contact ids in the store, least recently saved first: tells the
/// firmware which record a save into a full store must delete, and which
/// records `ClearContacts` removes.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct SaveOrder {
    ids: Vec<PeerId, MAX_CONTACTS>,
}

impl SaveOrder {
    /// The order of contacts loaded at boot (as [`Loader::contacts`]).
    pub fn from_contacts(contacts: &[Contact]) -> SaveOrder {
        let ids = contacts.iter().take(MAX_CONTACTS).fold(Vec::new(), |list, c| to_back(list, c.id).0);
        SaveOrder { ids }
    }

    /// Records a save of `id`: it becomes the most recent. Returns the new
    /// order and the id evicted to make room (delete its record), if any.
    #[must_use]
    pub fn save(self, id: PeerId) -> (SaveOrder, Option<PeerId>) {
        let (ids, evicted) = to_back(self.ids, id);
        (SaveOrder { ids }, evicted)
    }

    /// Stored ids, least recently saved first.
    pub fn ids(&self) -> &[PeerId] {
        &self.ids
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    fn contact(id: u32, chi: i32, name: &str, trades: u16) -> Contact {
        let mut n = String::new();
        n.push_str(name).unwrap();
        Contact { id: PeerId::from_u32(id), chi: Chi(chi), name: n, trades }
    }

    #[test]
    fn fr28_key_record_round_trips() {
        let k = RadioKey([7; KEY_LEN]);
        assert_eq!(decode_key(&encode_key(&k)), Ok(k));
        assert_eq!(decode_key(&[]), Err(RecordError::Length));
        assert_eq!(decode_key(&[FORMAT; 10]), Err(RecordError::Length));
        let mut bad = encode_key(&RadioKey([1; KEY_LEN]));
        bad[0] = 9;
        assert_eq!(decode_key(&bad), Err(RecordError::Format));
    }

    #[test]
    fn contact_record_round_trips() {
        for c in [contact(0x0AC8F6, 3, "ALICE", 2), contact(0xFFFFFF, -1, "", 65535), contact(0, 0, "12345678", 0)] {
            assert_eq!(decode_contact(&encode_contact(&c)), Ok(c));
        }
    }

    #[test]
    fn contact_record_rejects_bad_fields() {
        let good = encode_contact(&contact(0x123456, 1, "BOB", 1));
        assert_eq!(decode_contact(&good[..19]), Err(RecordError::Length));
        let mut v = good;
        v[0] = 2;
        assert_eq!(decode_contact(&v), Err(RecordError::Format));
        let mut v = good;
        v[11] = 9;
        assert_eq!(decode_contact(&v), Err(RecordError::Field));
        let mut v = good;
        v[12] = 0xFF;
        assert_eq!(decode_contact(&v), Err(RecordError::Field));
        let mut v = good;
        v[4] = 1;
        assert_eq!(decode_contact(&v), Err(RecordError::Field));
    }

    #[test]
    fn keys_classify() {
        let id = PeerId::from_u32(0x0AC8F6);
        assert_eq!(contact_key(id), 0x010A_C8F6);
        assert_eq!(record_of(contact_key(id)), Record::Contact(id));
        assert_eq!(record_of(KEY_RADIO), Record::RadioKey);
        assert_eq!(record_of(0x0200_0000), Record::Unknown);
    }

    #[test]
    fn fr28_loader_newest_value_wins() {
        let a1 = contact(1, 1, "A", 1);
        let b = contact(2, 2, "B", 1);
        let a2 = contact(1, 3, "A2", 2);
        let k1 = RadioKey([1; KEY_LEN]);
        let k2 = RadioKey([2; KEY_LEN]);
        let l = Loader::new()
            .push(KEY_RADIO, &encode_key(&k1))
            .push(contact_key(a1.id), &encode_contact(&a1))
            .push(contact_key(b.id), &encode_contact(&b))
            .push(contact_key(a2.id), &encode_contact(&a2))
            .push(KEY_RADIO, &encode_key(&k2));
        assert_eq!(l.contacts(), &[b, a2][..]);
        assert_eq!(l.key(), Some(&k2));
        assert_eq!(l.skipped(), 0);
    }

    #[test]
    fn loader_skips_garbage_and_mismatched_ids() {
        let a = contact(1, 1, "A", 1);
        let l = Loader::new()
            .push(0x7777, &[1, 2, 3])
            .push(KEY_RADIO, &[FORMAT])
            .push(contact_key(PeerId::from_u32(2)), &encode_contact(&a))
            .push(contact_key(a.id), &[0; 3]);
        assert!(l.contacts().is_empty());
        assert_eq!(l.key(), None);
        assert_eq!(l.skipped(), 4);
    }

    #[test]
    fn loader_bounds_contacts_evicting_oldest() {
        let mut l = Loader::new();
        for i in 0..(MAX_CONTACTS as u32 + 5) {
            let c = contact(i, 0, "", 1);
            l = l.push(contact_key(c.id), &encode_contact(&c));
        }
        assert_eq!(l.contacts().len(), MAX_CONTACTS);
        assert_eq!(l.contacts()[0].id, PeerId::from_u32(5));
        assert_eq!(l.skipped(), 0);
        let (list, key) = l.finish();
        assert_eq!(list.len(), MAX_CONTACTS);
        assert!(key.is_none());
    }

    #[test]
    fn save_order_moves_to_back_and_evicts_oldest() {
        let ids: std::vec::Vec<Contact> = (0..3).map(|i| contact(i, 0, "", 1)).collect();
        let order = SaveOrder::from_contacts(&ids);
        let (order, ev) = order.save(PeerId::from_u32(0));
        assert_eq!(ev, None);
        let want: [PeerId; 3] = [1, 2, 0].map(PeerId::from_u32);
        assert_eq!(order.ids(), &want[..]);
        let mut order = order;
        for i in 3..MAX_CONTACTS as u32 {
            let (next, ev) = order.save(PeerId::from_u32(i));
            assert_eq!(ev, None);
            order = next;
        }
        let (order, ev) = order.save(PeerId::from_u32(999));
        assert_eq!(ev, Some(PeerId::from_u32(1)));
        assert_eq!(order.ids().len(), MAX_CONTACTS);
        assert_eq!(order.ids().last(), Some(&PeerId::from_u32(999)));
    }
}
