//! Keys badges publish on, and the local `badge/*` keys through which a wasm
//! payload ("mw" ABI) reaches badge features. All are valid middleware keys
//! (checked at compile time by `Key::from_static`).
//!
//! Payload publishes are routed by [`route_payload_publish`]:
//!
//! - `badge/leds` (24 bytes, R G B per LED) → `SetLeds` (switches to MANUAL);
//! - `badge/text` (UTF-8, at most 256 bytes) → `DisplayText` on HOST;
//! - `badge/pet` (1 byte, a `PetMode` value) → `SetPetMode`;
//! - `app/...` → carried between badges (ESP-NOW and BLE advertising);
//! - anything else → refused.
//!
//! The badge delivers button edges to payloads subscribed to
//! [`BADGE_BUTTONS`] as `[button, pressed]` ([`button_sample`]).

use badge_proto::{DisplayText, HostMsg_::Cmd, PetMode, Rgb, SetLeds, SetPetMode};
use middleware::Key;

/// LED timing sync beacons (`sync::SyncBeacon` wire bytes).
pub const SYNC: Key<'static> = Key::from_static("conagotchi/sync");
/// Stock OZ adverts relayed between simulated badges (the advert's AD bytes).
pub const OZ: Key<'static> = Key::from_static("conagotchi/oz");
/// Prefix every wasm app key must start with, so apps cannot spoof the
/// badge's own keys.
pub const APP_PREFIX: &str = "app/";

/// True if a wasm app may publish or subscribe on `key`.
pub fn app_may_use(key: &str) -> bool {
    key.starts_with(APP_PREFIX) && Key::new(key).is_ok()
}

/// Payload → badge: 24 bytes of LED colours.
pub const BADGE_LEDS: Key<'static> = Key::from_static("badge/leds");
/// Payload → badge: text for the HOST screen.
pub const BADGE_TEXT: Key<'static> = Key::from_static("badge/text");
/// Payload → badge: one `PetMode` byte.
pub const BADGE_PET: Key<'static> = Key::from_static("badge/pet");
/// Badge → payload: button edges, `[button, pressed]`.
pub const BADGE_BUTTONS: Key<'static> = Key::from_static("badge/buttons");

/// Where a payload publish goes.
// `Local` carries a whole protobuf command (no heap to box into); routes are
// short-lived values, so the size is bounded and known.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, PartialEq)]
pub enum PayloadRoute {
    /// A badge feature: feed this command to the core (seq `WASM_SEQ`).
    Local(Cmd),
    /// An app key: offer it to the radio transports.
    Radio,
}

/// Why a payload publish was refused (maps to the ABI's `InvalidKey` /
/// `InvalidArgument`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RouteError {
    /// Not a `badge/*` key the badge understands, and not an `app/` key.
    UnknownKey,
    /// Right key, malformed payload (wrong length, bad UTF-8, bad mode).
    BadPayload,
}

/// Decides what a payload publish on `key` with `payload` does.
pub fn route_payload_publish(key: Key<'_>, payload: &[u8]) -> Result<PayloadRoute, RouteError> {
    let key = key.as_str();
    if key == BADGE_LEDS.as_str() {
        let rgb: &[u8; 24] = payload.try_into().map_err(|_| RouteError::BadPayload)?;
        let mut colors = heapless::Vec::new();
        for [r, g, b] in rgb.as_chunks::<3>().0 {
            let packed = (u32::from(*r) << 16) | (u32::from(*g) << 8) | u32::from(*b);
            colors
                .push(Rgb { rgb: packed })
                .map_err(|_| RouteError::BadPayload)?;
        }
        Ok(PayloadRoute::Local(Cmd::SetLeds(SetLeds { colors })))
    } else if key == BADGE_TEXT.as_str() {
        let text = core::str::from_utf8(payload).map_err(|_| RouteError::BadPayload)?;
        let mut out = heapless::String::new();
        out.push_str(text).map_err(|_| RouteError::BadPayload)?;
        Ok(PayloadRoute::Local(Cmd::DisplayText(DisplayText {
            text: out,
            fg: None,
            bg: None,
        })))
    } else if key == BADGE_PET.as_str() {
        match payload {
            [mode] if *mode < 6 => Ok(PayloadRoute::Local(Cmd::SetPetMode(SetPetMode {
                mode: PetMode(i32::from(*mode)),
            }))),
            _ => Err(RouteError::BadPayload),
        }
    } else if app_may_use(key) {
        Ok(PayloadRoute::Radio)
    } else {
        Err(RouteError::UnknownKey)
    }
}

/// The `badge/buttons` sample for a button edge: `[button, pressed]`.
pub fn button_sample(button: badge_proto::Button, pressed: bool) -> (Key<'static>, [u8; 2]) {
    let b = u8::try_from(button.0).unwrap_or(0);
    (BADGE_BUTTONS, [b, u8::from(pressed)])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn app_keys_are_confined_to_the_app_prefix() {
        assert!(app_may_use("app/hello"));
        assert!(app_may_use("app/x/y"));
        assert!(!app_may_use("conagotchi/sync"));
        assert!(!app_may_use("app/"));
        assert!(!app_may_use("app/a b"));
        assert_ne!(SYNC, OZ);
    }

    #[test]
    fn fr42_payload_publishes_route_to_badge_features_or_radio() {
        let mut rgb = [0u8; 24];
        rgb[0] = 0xFF;
        rgb[23] = 0x10;
        let Ok(PayloadRoute::Local(Cmd::SetLeds(s))) = route_payload_publish(BADGE_LEDS, &rgb)
        else {
            panic!("leds")
        };
        assert_eq!(s.colors.len(), 8);
        assert_eq!(s.colors[0].rgb, 0xFF_0000);
        assert_eq!(s.colors[7].rgb, 0x10);
        assert_eq!(
            route_payload_publish(BADGE_LEDS, &rgb[..23]),
            Err(RouteError::BadPayload)
        );
        assert!(matches!(
            route_payload_publish(BADGE_TEXT, b"hi"),
            Ok(PayloadRoute::Local(Cmd::DisplayText(t))) if t.text.as_str() == "hi"
        ));
        assert_eq!(
            route_payload_publish(BADGE_TEXT, &[0xFF]),
            Err(RouteError::BadPayload)
        );
        assert!(matches!(
            route_payload_publish(BADGE_PET, &[5]),
            Ok(PayloadRoute::Local(Cmd::SetPetMode(SetPetMode {
                mode: PetMode::Sleep
            })))
        ));
        assert_eq!(
            route_payload_publish(BADGE_PET, &[6]),
            Err(RouteError::BadPayload)
        );
        assert_eq!(
            route_payload_publish(Key::from_static("app/hello"), b"x"),
            Ok(PayloadRoute::Radio)
        );
        for k in ["badge/other", "conagotchi/sync", "x"] {
            assert_eq!(
                route_payload_publish(Key::new(k).unwrap(), b""),
                Err(RouteError::UnknownKey),
                "{k}"
            );
        }
    }

    #[test]
    fn fr42_button_edges_become_badge_buttons_samples() {
        assert_eq!(
            button_sample(badge_proto::Button::Select, true),
            (BADGE_BUTTONS, [3, 1])
        );
    }
}
