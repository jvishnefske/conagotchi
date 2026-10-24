//! Decoding of the firmware `PROBE` block.
//!
//! [`ProbeBlock::from_words`] turns the little-endian words read from target
//! RAM into named fields, using the word indices from
//! `badge_core::probe_layout` (the single source of truth shared with the
//! firmware). [`ProbeBlock::render`] and [`ProbeBlock::to_json`] present it.
//!
//! Layout v3 added the sync, contacts, trade and Chi words
//! ([`V3Words`]). A v2 block (older firmware) is still decoded: its first
//! [`V2_WORDS`] words are unchanged, the v3 words are reported as absent and
//! [`ProbeBlock::warning`] suggests a reflash.

use std::fmt::Write as _;

use badge_core::probe_layout as layout;
use badge_core::oz::PeerId;
use badge_proto::{Button, Chi, LedPattern, PetMode, Screen, TradeState};
use serde_json::{Value, json};

use crate::format::{
    button_name, chi_name, pattern_name, pet_mode_name, pressed_names, screen_name, trade_state_name, uptime,
};

/// Words in a layout v2 block (everything before [`layout::SYNC_OFFSET`]).
pub const V2_WORDS: usize = layout::SYNC_OFFSET;
/// The older layout version that is still decoded.
pub const LEGACY_VERSION: u32 = 2;

/// Words added in layout v3.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct V3Words {
    /// LED sync offset in ms (network time − uptime).
    pub sync_offset_ms: i32,
    /// LED sync leader id.
    pub sync_leader: u32,
    /// Stored contacts.
    pub contacts: u32,
    /// `TradeState` value.
    pub trade_state: u32,
    /// Trade peer as a 24-bit id, 0 when idle.
    pub trade_peer: u32,
    /// `Chi` value offered in trades.
    pub chi: u32,
}

impl V3Words {
    /// The trade peer id as text, `None` when idle (0).
    pub fn trade_peer_id(&self) -> Option<PeerId> {
        (self.trade_peer != 0).then(|| PeerId::from_u32(self.trade_peer))
    }
}

/// The word slice does not hold a full block.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error("PROBE block needs {expected} words, got {got}")]
pub struct ShortBlock {
    /// Words required.
    pub expected: usize,
    /// Words supplied.
    pub got: usize,
}

/// Decoded PROBE block.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProbeBlock {
    /// Magic marker (should be `MAGIC_VALUE`).
    pub magic: u32,
    /// Layout version.
    pub version: u32,
    /// Uptime in milliseconds (wrapping).
    pub uptime_ms: u32,
    /// Pressed-button mask.
    pub buttons: u32,
    /// Press counts indexed by `Button` value.
    pub press_counts: [u32; 6],
    /// Current screen.
    pub screen: u32,
    /// Current LED pattern.
    pub pattern: u32,
    /// LED brightness.
    pub brightness: u32,
    /// LED colours `0x00RRGGBB`.
    pub leds: [u32; 8],
    /// Host frames received.
    pub frames_rx: u32,
    /// Frames sent to the host.
    pub frames_tx: u32,
    /// Undecodable host frames.
    pub decode_errors: u32,
    /// Pending mailbox command (0 = empty).
    pub mailbox_cmd: u32,
    /// Mailbox argument 0.
    pub mailbox_arg0: u32,
    /// Mailbox argument 1.
    pub mailbox_arg1: u32,
    /// Selected `PetMode` value (may be AUTO).
    pub pet_selected: u32,
    /// `PetMode` value currently animating (never AUTO).
    pub pet_active: u32,
    /// Layout v3 words; `None` for a v2 block.
    pub v3: Option<V3Words>,
}

fn array<const N: usize>(words: &[u32], start: usize) -> [u32; N] {
    core::array::from_fn(|i| words.get(start + i).copied().unwrap_or(0))
}

fn mailbox_name(cmd: u32) -> String {
    match cmd {
        0 => "empty".into(),
        layout::mailbox::SET_LED => "SET_LED".into(),
        layout::mailbox::SHOW_SCREEN => "SHOW_SCREEN".into(),
        layout::mailbox::SET_PATTERN => "SET_PATTERN".into(),
        layout::mailbox::SET_PET_MODE => "SET_PET_MODE".into(),
        layout::mailbox::TRADE_INVITE => "TRADE_INVITE".into(),
        layout::mailbox::SET_CHI => "SET_CHI".into(),
        n => format!("unknown({n})"),
    }
}

fn enum_i32(v: u32) -> i32 {
    i32::try_from(v).unwrap_or(-1)
}

impl ProbeBlock {
    /// Decodes the block from at least [`V2_WORDS`] words.
    ///
    /// The v3 words are decoded only when the version word is 3 or more and
    /// all `layout::WORDS` words are present; otherwise [`Self::v3`] is
    /// `None`.
    pub fn from_words(words: &[u32]) -> Result<Self, ShortBlock> {
        if words.len() < V2_WORDS {
            return Err(ShortBlock { expected: V2_WORDS, got: words.len() });
        }
        let w = |i: usize| words.get(i).copied().unwrap_or(0);
        let version = w(layout::VERSION);
        let v3 = (version >= layout::VERSION_VALUE && words.len() >= layout::WORDS).then(|| V3Words {
            sync_offset_ms: i32::from_ne_bytes(w(layout::SYNC_OFFSET).to_ne_bytes()),
            sync_leader: w(layout::SYNC_LEADER),
            contacts: w(layout::CONTACTS),
            trade_state: w(layout::TRADE_STATE),
            trade_peer: w(layout::TRADE_PEER),
            chi: w(layout::CHI),
        });
        Ok(Self {
            magic: w(layout::MAGIC),
            version: w(layout::VERSION),
            uptime_ms: w(layout::UPTIME_MS),
            buttons: w(layout::BUTTONS),
            press_counts: array(words, layout::PRESS_COUNT),
            screen: w(layout::SCREEN),
            pattern: w(layout::PATTERN),
            brightness: w(layout::BRIGHTNESS),
            leds: array(words, layout::LEDS),
            frames_rx: w(layout::FRAMES_RX),
            frames_tx: w(layout::FRAMES_TX),
            decode_errors: w(layout::DECODE_ERRORS),
            mailbox_cmd: w(layout::MAILBOX_CMD),
            mailbox_arg0: w(layout::MAILBOX_ARG0),
            mailbox_arg1: w(layout::MAILBOX_ARG1),
            pet_selected: w(layout::PET_SELECTED),
            pet_active: w(layout::PET_ACTIVE),
            v3,
        })
    }

    /// True if magic and version match this host's layout.
    pub fn magic_ok(&self) -> bool {
        self.magic == layout::MAGIC_VALUE && self.version == layout::VERSION_VALUE
    }

    /// True for a valid layout v2 block (older firmware, still decoded).
    pub fn is_legacy(&self) -> bool {
        self.magic == layout::MAGIC_VALUE && self.version == LEGACY_VERSION
    }

    /// Warning to print when [`Self::magic_ok`] is false, `None` otherwise.
    ///
    /// A v2 block gets a note that the v3 words are missing. A good magic
    /// with any other version means the firmware and this tool disagree on
    /// the block layout (usually old firmware: reflash).
    pub fn warning(&self) -> Option<String> {
        if self.magic_ok() {
            None
        } else if self.is_legacy() {
            Some(format!(
                "note: PROBE layout v{LEGACY_VERSION} (older firmware); sync, contacts, trade and chi \
                 words need v{}: reflash to see them",
                layout::VERSION_VALUE
            ))
        } else if self.magic != layout::MAGIC_VALUE {
            Some("warning: magic mismatch; wrong address or firmware not running?".into())
        } else {
            Some(format!(
                "warning: PROBE layout version mismatch (badge v{}, badgectl expects v{}); \
                 the firmware is probably old: reflash it",
                self.version,
                layout::VERSION_VALUE
            ))
        }
    }

    /// Multi-line human-readable rendering.
    pub fn render(&self) -> String {
        let mut out = String::new();
        let status = if self.magic_ok() {
            "ok"
        } else if self.is_legacy() {
            "ok, older layout"
        } else {
            "MISMATCH"
        };
        let _ = writeln!(
            out,
            "magic          {:#010x} version {} ({status}; expected {:#010x} v{})",
            self.magic,
            self.version,
            layout::MAGIC_VALUE,
            layout::VERSION_VALUE
        );
        let _ = writeln!(out, "uptime         {} ({} ms)", uptime(u64::from(self.uptime_ms)), self.uptime_ms);
        let _ = writeln!(out, "buttons        {:#06x} [{}]", self.buttons, pressed_names(self.buttons));
        let counts: Vec<String> = self
            .press_counts
            .iter()
            .enumerate()
            .skip(1)
            .map(|(i, c)| format!("{}={c}", button_name(Button(enum_i32(i as u32)))))
            .collect();
        let _ = writeln!(out, "press counts   {}", counts.join(" "));
        let _ = writeln!(out, "screen         {} ({})", screen_name(Screen(enum_i32(self.screen))), self.screen);
        let _ = writeln!(
            out,
            "pattern        {} ({})",
            pattern_name(LedPattern(enum_i32(self.pattern))),
            self.pattern
        );
        let _ = writeln!(out, "brightness     {}", self.brightness);
        let _ = writeln!(
            out,
            "pet            selected {}, active {}",
            pet_mode_name(PetMode(enum_i32(self.pet_selected))),
            pet_mode_name(PetMode(enum_i32(self.pet_active)))
        );
        let leds: Vec<String> = self.leds.iter().map(|c| format!("{c:06x}")).collect();
        let _ = writeln!(out, "leds           {}", leds.join(" "));
        let _ = writeln!(
            out,
            "frames         rx {} tx {} decode errors {}",
            self.frames_rx, self.frames_tx, self.decode_errors
        );
        let _ = write!(
            out,
            "mailbox        cmd {} arg0 {:#x} arg1 {:#x}",
            mailbox_name(self.mailbox_cmd),
            self.mailbox_arg0,
            self.mailbox_arg1
        );
        match &self.v3 {
            Some(v) => {
                let _ = write!(
                    out,
                    "\nsync           leader {:08x} offset {:+} ms\ncontacts       {}\ntrade          {} {}\nchi            {} ({})",
                    v.sync_leader,
                    v.sync_offset_ms,
                    v.contacts,
                    trade_state_name(TradeState(enum_i32(v.trade_state))),
                    v.trade_peer_id().as_ref().map_or("-", PeerId::as_str),
                    chi_name(Chi(enum_i32(v.chi))),
                    v.chi
                );
            }
            None => out.push_str("\nsync/contacts/trade/chi  not in this layout"),
        }
        out
    }

    fn v3_json(&self) -> Value {
        match &self.v3 {
            None => Value::Null,
            Some(v) => json!({
                "sync_offset_ms": v.sync_offset_ms, "sync_leader": v.sync_leader,
                "contacts": v.contacts,
                "trade_state": v.trade_state,
                "trade_state_name": trade_state_name(TradeState(enum_i32(v.trade_state))),
                "trade_peer": v.trade_peer_id().as_ref().map_or(Value::Null, |p| Value::from(p.as_str())),
                "chi": v.chi, "chi_name": chi_name(Chi(enum_i32(v.chi))),
            }),
        }
    }

    /// JSON rendering.
    pub fn to_json(self) -> Value {
        json!({
            "magic": self.magic, "version": self.version, "magic_ok": self.magic_ok(),
            "legacy": self.is_legacy(), "v3": self.v3_json(),
            "uptime_ms": self.uptime_ms, "buttons": self.buttons, "press_counts": self.press_counts,
            "screen": self.screen, "screen_name": screen_name(Screen(enum_i32(self.screen))),
            "pattern": self.pattern, "pattern_name": pattern_name(LedPattern(enum_i32(self.pattern))),
            "brightness": self.brightness,
            "pet": {
                "selected": self.pet_selected,
                "selected_name": pet_mode_name(PetMode(enum_i32(self.pet_selected))),
                "active": self.pet_active,
                "active_name": pet_mode_name(PetMode(enum_i32(self.pet_active))),
            },
            "leds": self.leds.iter().map(|c| format!("{c:06x}")).collect::<Vec<_>>(),
            "frames_rx": self.frames_rx, "frames_tx": self.frames_tx, "decode_errors": self.decode_errors,
            "mailbox": { "cmd": self.mailbox_cmd, "arg0": self.mailbox_arg0, "arg1": self.mailbox_arg1 },
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> [u32; layout::WORDS] {
        let mut w = [0u32; layout::WORDS];
        w[layout::MAGIC] = layout::MAGIC_VALUE;
        w[layout::VERSION] = layout::VERSION_VALUE;
        w[layout::UPTIME_MS] = 61_005;
        w[layout::BUTTONS] = 1 << 3;
        for i in 0..6 {
            w[layout::PRESS_COUNT + i] = 10 + i as u32;
        }
        w[layout::SCREEN] = 3;
        w[layout::PATTERN] = 2;
        w[layout::BRIGHTNESS] = 64;
        for i in 0..8 {
            w[layout::LEDS + i] = 0x0011_2200 + i as u32;
        }
        w[layout::FRAMES_RX] = 4;
        w[layout::FRAMES_TX] = 5;
        w[layout::DECODE_ERRORS] = 1;
        w[layout::MAILBOX_CMD] = layout::mailbox::SET_LED;
        w[layout::MAILBOX_ARG0] = 2;
        w[layout::MAILBOX_ARG1] = 0xff00ff;
        w[layout::PET_SELECTED] = 0;
        w[layout::PET_ACTIVE] = 2;
        w[layout::SYNC_OFFSET] = (-250i32) as u32;
        w[layout::SYNC_LEADER] = 0xc100_1122;
        w[layout::CONTACTS] = 3;
        w[layout::TRADE_STATE] = 2;
        w[layout::TRADE_PEER] = 0x0A_C8F6;
        w[layout::CHI] = 5;
        w
    }

    fn v2_sample() -> Vec<u32> {
        let mut w = sample()[..V2_WORDS].to_vec();
        w[layout::VERSION] = LEGACY_VERSION;
        w
    }

    #[test]
    fn decodes_every_field() {
        let b = ProbeBlock::from_words(&sample()).unwrap();
        assert!(b.magic_ok());
        assert_eq!(b.uptime_ms, 61_005);
        assert_eq!(b.press_counts, [10, 11, 12, 13, 14, 15]);
        assert_eq!(b.leds[7], 0x0011_2207);
        assert_eq!((b.frames_rx, b.frames_tx, b.decode_errors), (4, 5, 1));
        assert_eq!((b.mailbox_cmd, b.mailbox_arg0, b.mailbox_arg1), (1, 2, 0xff00ff));
        assert_eq!((b.pet_selected, b.pet_active), (0, 2));
    }

    #[test]
    fn decodes_pet_words() {
        let mut w = sample();
        w[layout::PET_SELECTED] = 5;
        w[layout::PET_ACTIVE] = 5;
        w[layout::MAILBOX_CMD] = layout::mailbox::SET_PET_MODE;
        let b = ProbeBlock::from_words(&w).unwrap();
        assert_eq!((b.pet_selected, b.pet_active), (5, 5));
        let text = b.render();
        assert!(text.contains("pet            selected SLEEP, active SLEEP"));
        assert!(text.contains("cmd SET_PET_MODE"));
        w[layout::PET_ACTIVE] = 42;
        let text = ProbeBlock::from_words(&w).unwrap().render();
        assert!(text.contains("active PET_MODE(42)"));
    }

    #[test]
    fn decodes_v3_words() {
        let b = ProbeBlock::from_words(&sample()).unwrap();
        let v = b.v3.expect("v3 words");
        assert_eq!(v.sync_offset_ms, -250);
        assert_eq!(v.sync_leader, 0xc100_1122);
        assert_eq!(v.contacts, 3);
        assert_eq!(v.trade_state, 2);
        assert_eq!(v.trade_peer_id().map(|p| p.to_u32()), Some(0x0A_C8F6));
        assert_eq!(v.chi, 5);
        let text = b.render();
        assert!(text.contains("sync           leader c1001122 offset -250 ms"), "{text}");
        assert!(text.contains("contacts       3"));
        assert!(text.contains("trade          EXCHANGING 0AC8F6"));
        assert!(text.contains("chi            CRY (5)"));
        let j = b.to_json();
        assert_eq!(j["v3"]["trade_peer"], "0AC8F6");
        assert_eq!(j["v3"]["chi_name"], "CRY");
        assert_eq!(j["v3"]["sync_offset_ms"], -250);
        assert_eq!(j["legacy"], false);
        let mut idle = sample();
        idle[layout::TRADE_STATE] = 0;
        idle[layout::TRADE_PEER] = 0;
        let b = ProbeBlock::from_words(&idle).unwrap();
        assert!(b.render().contains("trade          IDLE -"));
        assert_eq!(b.to_json()["v3"]["trade_peer"], Value::Null);
    }

    #[test]
    fn mailbox_names_include_v3_commands() {
        let mut w = sample();
        w[layout::MAILBOX_CMD] = layout::mailbox::TRADE_INVITE;
        assert!(ProbeBlock::from_words(&w).unwrap().render().contains("cmd TRADE_INVITE"));
        w[layout::MAILBOX_CMD] = layout::mailbox::SET_CHI;
        assert!(ProbeBlock::from_words(&w).unwrap().render().contains("cmd SET_CHI"));
    }

    #[test]
    fn accepts_v2_block() {
        let b = ProbeBlock::from_words(&v2_sample()).unwrap();
        assert!(!b.magic_ok());
        assert!(b.is_legacy());
        assert_eq!(b.v3, None);
        assert_eq!(b.pet_active, 2);
        let text = b.render();
        assert!(text.contains("ok, older layout"), "{text}");
        assert!(!text.contains("MISMATCH"));
        assert!(text.contains("not in this layout"));
        let warn = b.warning().expect("note");
        assert!(warn.starts_with("note:") && warn.contains("v2") && warn.contains("v3"), "{warn}");
        assert_eq!(b.to_json()["v3"], Value::Null);
        assert_eq!(b.to_json()["legacy"], true);
        // A full-length read of v2 firmware: the trailing words are unrelated RAM.
        let mut long = sample();
        long[layout::VERSION] = LEGACY_VERSION;
        assert_eq!(ProbeBlock::from_words(&long).unwrap().v3, None);
    }

    #[test]
    fn version_mismatch_warns_about_old_firmware() {
        assert_eq!(layout::VERSION_VALUE, 3);
        assert_eq!(ProbeBlock::from_words(&sample()).unwrap().warning(), None);
        let mut old = sample();
        old[layout::VERSION] = 1;
        let b = ProbeBlock::from_words(&old).unwrap();
        assert!(!b.magic_ok());
        let warn = b.warning().expect("warning");
        assert!(warn.contains("version mismatch") && warn.contains("old"), "{warn}");
        assert!(warn.contains("v1") && warn.contains("v3"), "{warn}");
        assert!(b.render().contains("MISMATCH"));
        let mut bad = sample();
        bad[layout::MAGIC] = 0;
        let warn = ProbeBlock::from_words(&bad).unwrap().warning().expect("warning");
        assert!(warn.contains("magic mismatch"), "{warn}");
    }

    #[test]
    fn rejects_short_input() {
        assert_eq!(
            ProbeBlock::from_words(&[0; 5]),
            Err(ShortBlock { expected: V2_WORDS, got: 5 })
        );
    }

    #[test]
    fn renders_names() {
        let text = ProbeBlock::from_words(&sample()).unwrap().render();
        assert!(text.contains("(ok;"));
        assert!(text.contains("1m01.005s"));
        assert!(text.contains("[SELECT]"));
        assert!(text.contains("BOOT=11"));
        assert!(text.contains("screen         WIFI (3)"));
        assert!(text.contains("pattern        RAINBOW (2)"));
        assert!(text.contains("pet            selected AUTO, active WALK"));
        assert!(text.contains("112200 112201"));
        assert!(text.contains("cmd SET_LED arg0 0x2 arg1 0xff00ff"));
        let mut bad = sample();
        bad[layout::MAGIC] = 0;
        assert!(ProbeBlock::from_words(&bad).unwrap().render().contains("MISMATCH"));
    }

    #[test]
    fn json_rendering() {
        let j = ProbeBlock::from_words(&sample()).unwrap().to_json();
        assert_eq!(j["screen_name"], "WIFI");
        assert_eq!(j["magic_ok"], true);
        assert_eq!(j["mailbox"]["arg1"], 0xff00ff);
        assert_eq!(j["pet"]["selected"], 0);
        assert_eq!(j["pet"]["selected_name"], "AUTO");
        assert_eq!(j["pet"]["active"], 2);
        assert_eq!(j["pet"]["active_name"], "WALK");
    }
}
