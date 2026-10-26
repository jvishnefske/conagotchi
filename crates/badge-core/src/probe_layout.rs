//! Layout of the firmware's `PROBE` status block.
//!
//! The firmware keeps a `#[repr(C)]` block of `AtomicU32` words in RAM and
//! refreshes it every tick. A debugger can read it without any protocol
//! (`probe-rs read b32 <addr> 32`, or `badgectl peek`), and can poke the
//! mailbox words to drive the badge with nothing but memory writes.
//!
//! Word offsets below are the contract between the firmware struct (checked
//! there with `core::mem::offset_of!`) and the host tool.

use badge_proto::{
    Chi, HostMsg, HostMsg_::Cmd, LedPattern, PetMode, Rgb, Screen, SetChi, SetLed, SetPattern,
    SetPetMode, ShowScreen, TradeInvite,
};

use crate::oz::PeerId;

/// Value of word [`MAGIC`]: "BADGE001" squeezed into a word.
pub const MAGIC_VALUE: u32 = 0xBAD6_E001;
/// Value of word [`VERSION`] for this layout.
pub const VERSION_VALUE: u32 = 3;

/// Word index: magic marker.
pub const MAGIC: usize = 0;
/// Word index: layout version.
pub const VERSION: usize = 1;
/// Word index: uptime in milliseconds (wraps after ~49 days).
pub const UPTIME_MS: usize = 2;
/// Word index: debounced pressed-button mask (bit n = `Button(n)`).
pub const BUTTONS: usize = 3;
/// Word index: first of 6 press counters, indexed by `Button` value.
pub const PRESS_COUNT: usize = 4;
/// Word index: current `Screen` value.
pub const SCREEN: usize = 10;
/// Word index: current `LedPattern` value.
pub const PATTERN: usize = 11;
/// Word index: LED brightness 0..=255.
pub const BRIGHTNESS: usize = 12;
/// Word index: first of 8 LED colours as `0x00RRGGBB`.
pub const LEDS: usize = 13;
/// Word index: host frames received.
pub const FRAMES_RX: usize = 21;
/// Word index: frames sent to the host.
pub const FRAMES_TX: usize = 22;
/// Word index: host frames that failed to decode.
pub const DECODE_ERRORS: usize = 23;
/// Word index: mailbox command written by the debugger (0 = empty).
pub const MAILBOX_CMD: usize = 24;
/// Word index: mailbox argument 0.
pub const MAILBOX_ARG0: usize = 25;
/// Word index: mailbox argument 1.
pub const MAILBOX_ARG1: usize = 26;
/// Word index: selected `PetMode` (may be AUTO).
pub const PET_SELECTED: usize = 27;
/// Word index: `PetMode` currently animating (never AUTO).
pub const PET_ACTIVE: usize = 28;
/// Word index: LED sync offset in ms (network time − uptime), as `i32` bits.
pub const SYNC_OFFSET: usize = 29;
/// Word index: LED sync leader id (our own id when alone).
pub const SYNC_LEADER: usize = 30;
/// Word index: number of stored contacts.
pub const CONTACTS: usize = 31;
/// Word index: current `TradeState` value.
pub const TRADE_STATE: usize = 32;
/// Word index: current trade peer as a 24-bit id (`0x0AC8F6` = "0AC8F6"),
/// 0 when idle.
pub const TRADE_PEER: usize = 33;
/// Word index: `Chi` value we offer in trades.
pub const CHI: usize = 34;
/// Number of words in the block.
pub const WORDS: usize = 35;

/// Mailbox commands. Write the arguments first, then the command word; the
/// firmware clears the command word once it has taken it.
pub mod mailbox {
    /// Set LED `arg0` (0..7) to colour `arg1` (`0x00RRGGBB`).
    pub const SET_LED: u32 = 1;
    /// Show screen `arg0` (`Screen` value).
    pub const SHOW_SCREEN: u32 = 2;
    /// Set LED pattern `arg0` (`LedPattern` value), brightness `arg1` (0 = keep).
    pub const SET_PATTERN: u32 = 3;
    /// Set pet mode `arg0` (`PetMode` value).
    pub const SET_PET_MODE: u32 = 4;
    /// Invite peer `arg0` (24-bit id, `0x0AC8F6` = "0AC8F6") to a Chi trade.
    pub const TRADE_INVITE: u32 = 5;
    /// Offer Chi `arg0` (`Chi` value) in trades.
    pub const SET_CHI: u32 = 6;
}

/// Sequence number used for host messages synthesised from the mailbox.
/// Responses to them are still sent on RTT, tagged with this value.
pub const MAILBOX_SEQ: u32 = 0xFFFF_FFFF;

/// Sequence number used for host messages synthesised from GATT writes.
/// Their responses go to RTT like any other.
pub const GATT_SEQ: u32 = 0xFFFF_FFFE;

/// Sequence number used for host messages synthesised from wasm app calls
/// (`set_leds`, `show_text`, `set_pet`). Their responses go to RTT too.
pub const WASM_SEQ: u32 = 0xFFFF_FFFD;

/// Sequence numbers the USB serial loader (middleware's serial protocol on
/// the USB-Serial-JTAG CDC port) uses for the host messages it feeds the
/// core. The shell routes responses whose seq falls in this range back to
/// the serial link instead of RTT, so RTT hosts (whose seqs count up from
/// small values) and the serial link never see each other's answers. The
/// serial link hands out seqs from the range in turn, wrapping.
pub const SERIAL_SEQ: core::ops::Range<u32> = 0xFF00_0000..0xFFFF_0000;

/// True if `seq` belongs to the USB serial loader ([`SERIAL_SEQ`]).
pub const fn is_serial_seq(seq: u32) -> bool {
    seq >= SERIAL_SEQ.start && seq < SERIAL_SEQ.end
}

/// Converts a mailbox command into the equivalent host message.
///
/// Returns `None` for an empty (0) or unknown command, which the firmware
/// simply clears.
pub fn mailbox_to_host_msg(cmd: u32, arg0: u32, arg1: u32) -> Option<HostMsg> {
    let cmd = match cmd {
        mailbox::SET_LED => Cmd::SetLed(SetLed { index: arg0, color: Some(Rgb { rgb: arg1 }) }),
        mailbox::SHOW_SCREEN => Cmd::ShowScreen(ShowScreen { screen: Screen(arg0 as i32) }),
        mailbox::SET_PATTERN => Cmd::SetPattern(SetPattern {
            pattern: LedPattern(arg0 as i32),
            brightness: arg1,
        }),
        mailbox::SET_PET_MODE => Cmd::SetPetMode(SetPetMode { mode: PetMode(arg0 as i32) }),
        mailbox::TRADE_INVITE => {
            let mut peer = heapless::String::new();
            peer.push_str(PeerId::from_u32(arg0).as_str()).ok()?;
            Cmd::TradeInvite(TradeInvite { peer })
        }
        mailbox::SET_CHI => Cmd::SetChi(SetChi { chi: Chi(arg0 as i32) }),
        _ => return None,
    };
    Some(HostMsg { seq: MAILBOX_SEQ, cmd: Some(cmd) })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fr44_serial_seqs_are_disjoint_from_the_reserved_seqs() {
        assert!(is_serial_seq(SERIAL_SEQ.start));
        assert!(is_serial_seq(SERIAL_SEQ.end - 1));
        assert!(!is_serial_seq(SERIAL_SEQ.end));
        for seq in [0, 1, 0x00FF_FFFF, MAILBOX_SEQ, GATT_SEQ, WASM_SEQ] {
            assert!(!is_serial_seq(seq), "{seq:#x}");
        }
    }

    #[test]
    fn offsets_are_contiguous_and_in_bounds() {
        assert_eq!(PRESS_COUNT + 6, SCREEN);
        assert_eq!(LEDS + 8, FRAMES_RX);
        assert_eq!(PET_ACTIVE + 1, SYNC_OFFSET);
        assert_eq!(TRADE_PEER + 1, CHI);
        assert_eq!(CHI + 1, WORDS);
    }

    #[test]
    fn mailbox_maps_known_commands() {
        let msg = mailbox_to_host_msg(mailbox::SET_LED, 3, 0x00ff_0000).expect("set led");
        assert_eq!(msg.seq, MAILBOX_SEQ);
        assert_eq!(
            msg.cmd,
            Some(Cmd::SetLed(SetLed { index: 3, color: Some(Rgb { rgb: 0x00ff_0000 }) }))
        );
        assert!(matches!(
            mailbox_to_host_msg(mailbox::SHOW_SCREEN, 4, 0).and_then(|m| m.cmd),
            Some(Cmd::ShowScreen(ShowScreen { screen: Screen::Ble }))
        ));
        assert!(matches!(
            mailbox_to_host_msg(mailbox::SET_PET_MODE, 5, 0).and_then(|m| m.cmd),
            Some(Cmd::SetPetMode(SetPetMode { mode: PetMode::Sleep }))
        ));
        let invite = mailbox_to_host_msg(mailbox::TRADE_INVITE, 0x0A_C8F6, 0).and_then(|m| m.cmd);
        let Some(Cmd::TradeInvite(t)) = invite else { panic!("trade invite") };
        assert_eq!(t.peer.as_str(), "0AC8F6");
        assert!(matches!(
            mailbox_to_host_msg(mailbox::SET_CHI, 5, 0).and_then(|m| m.cmd),
            Some(Cmd::SetChi(SetChi { chi: Chi::Cry }))
        ));
        assert!(mailbox_to_host_msg(0, 0, 0).is_none());
        assert!(mailbox_to_host_msg(99, 0, 0).is_none());
    }
}
