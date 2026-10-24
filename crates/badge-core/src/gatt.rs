//! The pure half of the badge's GATT service "Conagotchi".
//!
//! The firmware declares the service with trouble-host's `#[gatt_server]`
//! and only moves bytes: it answers reads with the values formatted here and
//! turns writes into host messages with [`write_to_host_msg`], tagged with
//! [`probe_layout::GATT_SEQ`], so a GATT write acts exactly like the matching
//! host request (FR-30). Invalid writes are rejected before they reach the
//! app, with the ATT error [`GattError::att_code`] names.
//!
//! The service and characteristic UUIDs are fixed random (version 4) UUIDs
//! sharing one base, `936363ab-7686-4c11-8c9c-9fa6dd48dfc6`, with the first
//! 32 bits counting up:
//!
//! - service "Conagotchi": `936363ab-7686-4c11-8c9c-9fa6dd48dfc6`
//! - info (read, UTF-8 text, see [`info_string`]):
//!   `936363ac-7686-4c11-8c9c-9fa6dd48dfc6`
//! - pet mode (read/write, one byte, a `PetMode` value):
//!   `936363ad-7686-4c11-8c9c-9fa6dd48dfc6`
//! - LED pattern (read/write, one byte, a `LedPattern` value):
//!   `936363ae-7686-4c11-8c9c-9fa6dd48dfc6`
//! - LED colour (write, three bytes R, G, B; sets all 8 LEDs, pattern MANUAL):
//!   `936363af-7686-4c11-8c9c-9fa6dd48dfc6`

use badge_proto::{
    Chi, HostMsg, HostMsg_::Cmd, LedPattern, PetMode, Rgb, SetLeds, SetPattern, SetPetMode,
};
use heapless::String;

use crate::board::led::RGB_COUNT;
use crate::leds;
use crate::oz::PeerId;
use crate::probe_layout;

/// Service "Conagotchi".
pub const SERVICE_UUID: u128 = 0x9363_63ab_7686_4c11_8c9c_9fa6_dd48_dfc6;
/// Characteristic: info text (read).
pub const INFO_UUID: u128 = 0x9363_63ac_7686_4c11_8c9c_9fa6_dd48_dfc6;
/// Characteristic: pet mode (read/write u8).
pub const PET_MODE_UUID: u128 = 0x9363_63ad_7686_4c11_8c9c_9fa6_dd48_dfc6;
/// Characteristic: LED pattern (read/write u8).
pub const LED_PATTERN_UUID: u128 = 0x9363_63ae_7686_4c11_8c9c_9fa6_dd48_dfc6;
/// Characteristic: LED colour (write, 3 bytes RGB).
pub const LED_COLOR_UUID: u128 = 0x9363_63af_7686_4c11_8c9c_9fa6_dd48_dfc6;

/// Capacity of the info text.
pub const INFO_LEN: usize = 64;

/// A writable characteristic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Writable {
    /// Pet mode, one byte.
    PetMode,
    /// LED pattern, one byte.
    LedPattern,
    /// LED colour, three bytes.
    LedColor,
}

impl Writable {
    /// Exact length of a valid write.
    pub const fn value_len(self) -> usize {
        match self {
            Writable::PetMode | Writable::LedPattern => 1,
            Writable::LedColor => 3,
        }
    }
}

/// Why a write was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GattError {
    /// The value has the wrong length.
    BadLength,
    /// The value is not a supported pet mode or LED pattern.
    BadValue,
}

impl GattError {
    /// The ATT error code to answer with: Invalid Attribute Value Length
    /// (0x0D) or Value Not Allowed (0x13).
    pub const fn att_code(self) -> u8 {
        match self {
            GattError::BadLength => 0x0D,
            GattError::BadValue => 0x13,
        }
    }
}

/// True if `mode` is a defined pet mode.
fn pet_mode_known(mode: PetMode) -> bool {
    matches!(
        mode,
        PetMode::Auto
            | PetMode::Idle
            | PetMode::Walk
            | PetMode::Eat
            | PetMode::Play
            | PetMode::Sleep
    )
}

/// Converts a write of `data` to `target` into the equivalent host message,
/// tagged with [`probe_layout::GATT_SEQ`].
///
/// # Errors
///
/// [`GattError::BadLength`] unless `data` has exactly [`Writable::value_len`]
/// bytes; [`GattError::BadValue`] for an unknown pet mode or an unsupported
/// LED pattern.
pub fn write_to_host_msg(target: Writable, data: &[u8]) -> Result<HostMsg, GattError> {
    if data.len() != target.value_len() {
        return Err(GattError::BadLength);
    }
    let cmd = match (target, data) {
        (Writable::PetMode, &[v]) => {
            let mode = PetMode(i32::from(v));
            if !pet_mode_known(mode) {
                return Err(GattError::BadValue);
            }
            Cmd::SetPetMode(SetPetMode { mode })
        }
        (Writable::LedPattern, &[v]) => {
            let pattern = LedPattern(i32::from(v));
            if !leds::is_supported(pattern) {
                return Err(GattError::BadValue);
            }
            Cmd::SetPattern(SetPattern {
                pattern,
                brightness: 0,
            })
        }
        (Writable::LedColor, &[r, g, b]) => {
            let rgb = Rgb {
                rgb: u32::from_be_bytes([0, r, g, b]),
            };
            let mut colors = heapless::Vec::new();
            for _ in 0..RGB_COUNT {
                colors.push(rgb).map_err(|_| GattError::BadLength)?;
            }
            Cmd::SetLeds(SetLeds { colors })
        }
        _ => return Err(GattError::BadLength),
    };
    Ok(HostMsg {
        seq: probe_layout::GATT_SEQ,
        cmd: Some(cmd),
    })
}

/// Name of a Chi, as the stock badges spell it in packets.
pub fn chi_name(chi: Chi) -> &'static str {
    match chi {
        Chi::Vib => "VIB",
        Chi::Hck => "HCK",
        Chi::Pkt => "PKT",
        Chi::Adm => "ADM",
        Chi::Cry => "CRY",
        Chi::Ovr => "OVR",
        _ => "-",
    }
}

/// The info characteristic's text: `"<firmware>;id=<id6>;chi=<chi>"`, where
/// id6 is the badge id (last 3 MAC bytes, uppercase hex) and chi the
/// [`chi_name`] of the PROBE `CHI` word. Cut to [`INFO_LEN`] bytes.
pub fn info_string(firmware: &str, mac: &[u8; 6], chi_word: u32) -> String<INFO_LEN> {
    let chi = Chi(i32::try_from(chi_word).unwrap_or(0));
    let id = PeerId::from_mac(mac);
    let mut out = String::new();
    for part in [firmware, ";id=", id.as_str(), ";chi=", chi_name(chi)] {
        for c in part.chars() {
            if out.push(c).is_err() {
                return out;
            }
        }
    }
    out
}

/// A PROBE word (pet mode or LED pattern value) as the one-byte read value,
/// saturated to 255.
pub fn probe_byte(word: u32) -> u8 {
    u8::try_from(word).unwrap_or(u8::MAX)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cmd(target: Writable, data: &[u8]) -> Cmd {
        let msg = write_to_host_msg(target, data).expect("valid write");
        assert_eq!(msg.seq, probe_layout::GATT_SEQ);
        msg.cmd.expect("command")
    }

    #[test]
    fn fr30_pet_mode_write_maps_to_set_pet_mode() {
        assert_eq!(
            cmd(Writable::PetMode, &[5]),
            Cmd::SetPetMode(SetPetMode {
                mode: PetMode::Sleep
            })
        );
        assert_eq!(
            write_to_host_msg(Writable::PetMode, &[6]),
            Err(GattError::BadValue)
        );
    }

    #[test]
    fn fr30_pattern_write_maps_to_set_pattern_keeping_brightness() {
        assert_eq!(
            cmd(Writable::LedPattern, &[2]),
            Cmd::SetPattern(SetPattern {
                pattern: LedPattern::Rainbow,
                brightness: 0
            })
        );
        assert_eq!(
            write_to_host_msg(Writable::LedPattern, &[4]),
            Err(GattError::BadValue)
        );
    }

    #[test]
    fn fr30_colour_write_sets_all_leds() {
        let Cmd::SetLeds(s) = cmd(Writable::LedColor, &[0x12, 0x34, 0x56]) else {
            panic!("set leds")
        };
        assert_eq!(s.colors.len(), RGB_COUNT);
        assert!(s.colors.iter().all(|c| c.rgb == 0x0012_3456));
    }

    #[test]
    fn fr30_bad_length_is_rejected() {
        for (target, data) in [
            (Writable::PetMode, &[][..]),
            (Writable::PetMode, &[1, 2][..]),
            (Writable::LedPattern, &[1, 0][..]),
            (Writable::LedColor, &[1, 2][..]),
            (Writable::LedColor, &[1, 2, 3, 4][..]),
        ] {
            assert_eq!(write_to_host_msg(target, data), Err(GattError::BadLength));
        }
        assert_eq!(GattError::BadLength.att_code(), 0x0D);
        assert_eq!(GattError::BadValue.att_code(), 0x13);
    }

    #[test]
    fn fr30_info_string_has_firmware_id_and_chi() {
        let mac = [0xe8, 0x3d, 0xc1, 0xf6, 0x0a, 0xc8];
        let info = info_string("conagotchi-fw 0.1.0", &mac, 2);
        assert_eq!(info.as_str(), "conagotchi-fw 0.1.0;id=F60AC8;chi=HCK");
        assert_eq!(info_string("fw", &mac, 0).as_str(), "fw;id=F60AC8;chi=-");
        assert_eq!(
            info_string("fw", &mac, u32::MAX).as_str(),
            "fw;id=F60AC8;chi=-"
        );
        let long = "0123456789012345678901234567890123456789012345678901234567890123456789";
        let info = info_string(long, &mac, 1);
        assert_eq!(info.len(), INFO_LEN);
    }

    #[test]
    fn fr30_probe_byte_saturates() {
        assert_eq!(probe_byte(3), 3);
        assert_eq!(probe_byte(255), 255);
        assert_eq!(probe_byte(256), 255);
    }
}
