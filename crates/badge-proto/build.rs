//! Generates the protocol types from `proto/badge.proto`.
//!
//! `protox` (a pure-Rust protobuf compiler) produces the file descriptor set,
//! so no system `protoc` is needed. `micropb-gen` then turns it into no_std
//! Rust with fixed-capacity `heapless` containers.

use std::path::PathBuf;

use micropb_gen::{Config, Generator, config::OptionalRepr};
use prost::Message;

/// Fixed capacity for every container field: (field path, capacity).
///
/// Repeated fields use `max_len` (elements); string/bytes fields use
/// `max_bytes`. Keep these in sync with the limits documented in the proto.
const REPEATED: &[(&str, u32)] = &[
    (".badge.SetLeds.colors", 8),
    (".badge.Info.probe_pins", 24),
    (".badge.ButtonState.press_counts", 6),
    (".badge.WifiScanResult.aps", 12),
    (".badge.BleScanResult.devices", 12),
    (".badge.Contacts.contacts", 8),
    (".badge.Lobby.peers", 8),
];

const BYTES: &[(&str, u32)] = &[
    (".badge.DisplayText.text", 256),
    (".badge.WifiConnect.ssid", 32),
    (".badge.WifiConnect.password", 64),
    (".badge.BleAdvertise.name", 29),
    (".badge.Error.detail", 48),
    (".badge.Info.firmware", 32),
    (".badge.Info.chip", 16),
    (".badge.Info.mac", 6),
    (".badge.AccessPoint.ssid", 32),
    (".badge.AccessPoint.bssid", 6),
    (".badge.WifiState.ssid", 32),
    (".badge.WifiState.ipv4", 4),
    (".badge.BleDevice.addr", 6),
    (".badge.BleDevice.name", 29),
    (".badge.TradeInvite.peer", 6),
    (".badge.Contact.id", 6),
    (".badge.Contact.name", 8),
    (".badge.LobbyPeer.id", 6),
    (".badge.Lobby.peer", 6),
    (".badge.TradeEvent.peer", 6),
    (".badge.WasmChunk.data", 192),
    (".badge.WasmStatus.error", 48),
    (".badge.WasmLog.text", 64),
];

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let root = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR")?).join("../../proto");
    let proto = root.join("badge.proto");
    println!("cargo:rerun-if-changed={}", proto.display());
    println!("cargo:rerun-if-changed=build.rs");

    let fdset = protox::compile([&proto], [&root])?;
    let out_dir = PathBuf::from(std::env::var("OUT_DIR")?);
    let fdset_path = out_dir.join("badge.fdset");
    std::fs::write(&fdset_path, fdset.encode_to_vec())?;

    let mut generator = Generator::new();
    generator
        .use_container_heapless()
        .calculate_max_size(true)
        .configure(".", Config::new().optional_repr(OptionalRepr::Option));
    for (path, len) in REPEATED {
        generator.configure(path, Config::new().max_len(*len));
    }
    for (path, len) in BYTES {
        generator.configure(path, Config::new().max_bytes(*len));
    }
    generator.compile_fdset_file(&fdset_path, out_dir.join("badge.rs"))?;
    Ok(())
}
