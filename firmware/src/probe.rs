//! The `PROBE` debugger status block.
//!
//! [`PROBE`] is a `#[repr(C)]` array of `AtomicU32` words in RAM whose layout
//! is defined by [`badge_core::probe_layout`]. The host locates it through the
//! ELF symbol table (any symbol whose demangled name ends in `probe::PROBE`)
//! and reads or writes it with plain debugger memory accesses while the badge
//! runs; no protocol and no `#[no_mangle]` are needed.
//!
//! - [`publish`] copies a [`Snapshot`] of the app model into the block every
//!   tick (magic and version words included, so a reader can validate it).
//! - [`word`] reads one word back (the BLE task answers GATT reads with it).
//! - [`take_mailbox`] polls the mailbox: when the debugger has written a
//!   non-zero command word, the command and its two arguments are converted
//!   with [`probe_layout::mailbox_to_host_msg`] and the command word is
//!   cleared, so the debugger can see that it was consumed.
//!
//! Only atomics are used, so the block is shared between the firmware and the
//! debugger without `unsafe` or locks.

use core::sync::atomic::{AtomicU32, Ordering};

use badge_core::color::Color;
use badge_core::probe_layout;
use badge_proto::HostMsg;

/// The word block read and written by the debugger.
#[repr(C)]
pub struct ProbeBlock {
    words: [AtomicU32; probe_layout::WORDS],
}

const _: () = assert!(core::mem::size_of::<ProbeBlock>() == 4 * probe_layout::WORDS);
const _: () = assert!(core::mem::offset_of!(ProbeBlock, words) == 0);

impl ProbeBlock {
    const fn new() -> Self {
        Self { words: [const { AtomicU32::new(0) }; probe_layout::WORDS] }
    }

    fn store(&self, index: usize, value: u32) {
        if let Some(word) = self.words.get(index) {
            word.store(value, Ordering::Relaxed);
        }
    }

    fn load(&self, index: usize) -> u32 {
        self.words.get(index).map_or(0, |w| w.load(Ordering::Relaxed))
    }
}

/// The firmware's debugger-visible status block.
#[used]
pub static PROBE: ProbeBlock = ProbeBlock::new();

/// Values published into [`PROBE`] each tick, taken from the app accessors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Snapshot {
    /// Uptime in milliseconds (truncated to 32 bits in the block).
    pub uptime_ms: u64,
    /// Debounced pressed-button mask.
    pub buttons: u8,
    /// Press counters indexed by `Button` value.
    pub press_counts: [u32; 6],
    /// Current `Screen` value.
    pub screen: i32,
    /// Current `LedPattern` value.
    pub pattern: i32,
    /// LED brightness 0..=255.
    pub brightness: u8,
    /// LED colours.
    pub leds: [Color; 8],
    /// Host frames received.
    pub frames_rx: u32,
    /// Frames sent to the host.
    pub frames_tx: u32,
    /// Host frames that failed to decode.
    pub decode_errors: u32,
    /// Selected `PetMode` value (may be AUTO).
    pub pet_selected: i32,
    /// `PetMode` value currently animating.
    pub pet_active: i32,
    /// LED sync offset (network time − uptime), ms.
    pub sync_offset_ms: i32,
    /// LED sync leader id.
    pub sync_leader: u32,
    /// Stored contacts.
    pub contacts: u32,
    /// Current `TradeState` value.
    pub trade_state: i32,
    /// Current trade peer as a 24-bit id, 0 when none.
    pub trade_peer: u32,
    /// `Chi` value we offer.
    pub chi: i32,
}

/// Reads word `index` of [`PROBE`] (0 if out of range); the BLE task uses it
/// to answer GATT reads with the values published by the core loop.
pub fn word(index: usize) -> u32 {
    PROBE.load(index)
}

/// Writes `snap` (plus magic and version) into [`PROBE`].
pub fn publish(snap: &Snapshot) {
    let p = &PROBE;
    p.store(probe_layout::MAGIC, probe_layout::MAGIC_VALUE);
    p.store(probe_layout::VERSION, probe_layout::VERSION_VALUE);
    // Truncation is the documented contract: the word wraps after ~49 days.
    p.store(probe_layout::UPTIME_MS, snap.uptime_ms as u32);
    p.store(probe_layout::BUTTONS, u32::from(snap.buttons));
    for (i, count) in snap.press_counts.iter().enumerate() {
        p.store(probe_layout::PRESS_COUNT + i, *count);
    }
    p.store(probe_layout::SCREEN, snap.screen as u32);
    p.store(probe_layout::PATTERN, snap.pattern as u32);
    p.store(probe_layout::BRIGHTNESS, u32::from(snap.brightness));
    for (i, c) in snap.leds.iter().enumerate() {
        p.store(probe_layout::LEDS + i, c.packed());
    }
    p.store(probe_layout::FRAMES_RX, snap.frames_rx);
    p.store(probe_layout::FRAMES_TX, snap.frames_tx);
    p.store(probe_layout::DECODE_ERRORS, snap.decode_errors);
    p.store(probe_layout::PET_SELECTED, snap.pet_selected as u32);
    p.store(probe_layout::PET_ACTIVE, snap.pet_active as u32);
    p.store(probe_layout::SYNC_OFFSET, snap.sync_offset_ms as u32);
    p.store(probe_layout::SYNC_LEADER, snap.sync_leader);
    p.store(probe_layout::CONTACTS, snap.contacts);
    p.store(probe_layout::TRADE_STATE, snap.trade_state as u32);
    p.store(probe_layout::TRADE_PEER, snap.trade_peer);
    p.store(probe_layout::CHI, snap.chi as u32);
}

/// Outcome of polling the mailbox.
#[derive(Debug, Clone, PartialEq)]
#[allow(clippy::large_enum_variant, reason = "no heap to box into; returned by value once per tick")]
pub enum Mailbox {
    /// No command pending.
    Empty,
    /// A known command, converted to the equivalent host message.
    Command(HostMsg),
    /// An unknown command word (cleared and ignored).
    Unknown(u32),
}

/// Takes a pending mailbox command, clearing the command word.
pub fn take_mailbox() -> Mailbox {
    let p = &PROBE;
    let cmd = p.load(probe_layout::MAILBOX_CMD);
    if cmd == 0 {
        return Mailbox::Empty;
    }
    let arg0 = p.load(probe_layout::MAILBOX_ARG0);
    let arg1 = p.load(probe_layout::MAILBOX_ARG1);
    p.store(probe_layout::MAILBOX_CMD, 0);
    match probe_layout::mailbox_to_host_msg(cmd, arg0, arg1) {
        Some(msg) => Mailbox::Command(msg),
        None => Mailbox::Unknown(cmd),
    }
}
