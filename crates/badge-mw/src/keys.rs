//! Keys badges publish on. Valid middleware keys (checked at compile time by
//! `Key::from_static`).

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
}
