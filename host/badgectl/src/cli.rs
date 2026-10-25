//! Command-line definition (clap derive).
//!
//! Only argument shapes live here; turning them into protocol messages is
//! [`crate::request`]'s job so that it can be tested without a probe.

use std::path::PathBuf;

use badge_core::color::Color;
use badge_core::oz::PeerId;
use clap::{Args, Parser, Subcommand, ValueEnum};

use crate::color::parse_color;

/// Talk to the Conagotchi discovery firmware through a probe-rs debug probe.
#[derive(Debug, Parser)]
#[command(name = "badgectl", version, about, propagate_version = true)]
pub struct Cli {
    /// Firmware ELF (symbols, defmt table, RTT block). Defaults to the
    /// release build under `firmware/target` if it exists.
    #[arg(long, global = true)]
    pub elf: Option<PathBuf>,
    /// Debug probe selector `VID:PID[:SERIAL]` (hex). Default: first probe.
    #[arg(long, global = true)]
    pub probe: Option<String>,
    /// probe-rs target name.
    #[arg(long, global = true, default_value = "esp32s3")]
    pub chip: String,
    /// Response timeout in milliseconds (scans use at least 15000).
    #[arg(long, global = true, default_value_t = 5000)]
    pub timeout: u64,
    /// Print responses and events as JSON lines.
    #[arg(long, global = true)]
    pub json: bool,
    /// Send the request to badge N of a fresh simulation instead of a probe
    /// (see `badgectl sim`); the simulation runs `--sim-warmup-ms` first.
    #[arg(long, global = true, value_name = "N")]
    pub sim: Option<usize>,
    /// Badges in the `--sim` simulation.
    #[arg(long, global = true, default_value_t = 2)]
    pub sim_badges: usize,
    /// Simulated time to run before a `--sim` request (lobby and sync settle).
    #[arg(long, global = true, default_value_t = 3000)]
    pub sim_warmup_ms: u64,
    /// badge-sim wasm module to run instead of the one built into badgectl.
    #[arg(long, global = true, value_name = "FILE")]
    pub sim_wasm: Option<PathBuf>,
    /// What to do.
    #[command(subcommand)]
    pub command: Command,
}

/// Top-level subcommands.
#[derive(Debug, Subcommand)]
pub enum Command {
    /// Flash the ELF (esp-idf image format), reset, then monitor.
    Flash,
    /// Stream decoded defmt logs, events and responses until Ctrl-C.
    Monitor,
    /// Stream button / screen events (and logs) until Ctrl-C.
    Events,
    /// Read and decode the firmware's PROBE block.
    Peek {
        /// PROBE block address (default: from the ELF symbol table).
        #[arg(long, value_parser = parse_u64)]
        addr: Option<u64>,
    },
    /// Drive the badge through the PROBE mailbox (no RTT needed).
    Poke {
        /// PROBE block address (default: from the ELF symbol table).
        #[arg(long, value_parser = parse_u64)]
        addr: Option<u64>,
        /// Mailbox command.
        #[command(subcommand)]
        cmd: PokeCmd,
    },
    /// Print PROBE / _SEGGER_RTT addresses and stock probe-rs command lines.
    Addr,
    /// Wait for a newly plugged badge, back it up and flash Conagotchi onto it.
    Adopt(AdoptArgs),
    /// Write a backup made by `adopt` back to flash offset 0x0.
    Restore {
        /// Backup file (`~/.local/share/conagotchi/backups/<mac>.bin`).
        backup: PathBuf,
    },
    /// Run simulated badges (the badge core as wasm) talking over a
    /// middleware bus.
    Sim(SimArgs),
    /// Protocol requests sent over RTT.
    #[command(flatten)]
    Request(RequestCmd),
}

/// Options of `sim`.
#[derive(Debug, Clone, PartialEq, Eq, Args)]
pub struct SimArgs {
    /// Number of simulated badges.
    #[arg(long, default_value_t = 2)]
    pub badges: usize,
    /// Simulated run time in ms (0 = until Ctrl-C).
    #[arg(long, default_value_t = 10_000)]
    pub duration_ms: u64,
    /// Pace the simulation to wall-clock time.
    #[arg(long)]
    pub realtime: bool,
    /// Print every message the badges send (events, responses).
    #[arg(long)]
    pub events: bool,
    /// Write each badge's screen as PNG into this directory.
    #[arg(long, value_name = "DIR")]
    pub png: Option<PathBuf>,
    /// Minimum simulated time between two PNGs of one badge.
    #[arg(long, default_value_t = 1000)]
    pub png_every_ms: u64,
    /// Scenario: lines of `<at_ms> <badge> <request...>` (`@N` = badge N's id).
    #[arg(long, value_name = "FILE")]
    pub script: Option<PathBuf>,
    /// Load and start this wasm app on every badge right after boot.
    #[arg(long, value_name = "FILE")]
    pub app: Option<PathBuf>,
    /// Extra app keys every badge subscribes to (keys only remote processes
    /// publish on).
    #[arg(long = "app-key", value_name = "KEY")]
    pub app_keys: Vec<String>,
    /// MAC of badge 0 minus one, hex (use distinct bases per process with --zenoh).
    #[arg(long, value_parser = parse_u64, default_value = "0x0200c0ffee00")]
    pub mac_base: u64,
    /// Boot delay between consecutive badges (their clocks differ by this).
    #[arg(long, default_value_t = 1234)]
    pub stagger_ms: u64,
    /// RSSI of simulated BLE receptions (the lobby ignores below -55).
    #[arg(long, default_value_t = -40, allow_negative_numbers = true)]
    pub rssi: i8,
    /// Connect the bus to this Zenoh endpoint (e.g. tcp/127.0.0.1:7447).
    #[arg(long, value_name = "ENDPOINT")]
    pub zenoh: Vec<String>,
    /// Listen for Zenoh peers on this endpoint.
    #[arg(long, value_name = "ENDPOINT")]
    pub zenoh_listen: Vec<String>,
}

/// Requests that become one `HostMsg` and wait for one `Response`.
#[derive(Debug, Clone, PartialEq, Subcommand)]
pub enum RequestCmd {
    /// Round-trip check; prints the badge uptime.
    Ping {
        /// Nonce echoed back (default: the sequence number).
        nonce: Option<u32>,
    },
    /// Firmware, chip, MAC, uptime, probe pins.
    Info,
    /// Pressed buttons and press counters.
    Buttons,
    /// Set one LED (switches the pattern to manual).
    Led {
        /// LED index 0..7.
        index: u32,
        /// Colour.
        #[arg(value_parser = parse_color)]
        color: Color,
    },
    /// Set LEDs 0.. in order (1 to 8 colours).
    Leds {
        /// Colours in index order.
        #[arg(required = true, num_args = 1..=8, value_parser = parse_color)]
        colors: Vec<Color>,
    },
    /// Select the LED pattern.
    Pattern {
        /// Pattern.
        pattern: PatternArg,
        /// Global brightness 1..255 (omit or 0 to keep).
        #[arg(long)]
        brightness: Option<u8>,
    },
    /// Switch the red alert / green raffle LED.
    Indicator {
        /// Which indicator.
        which: IndicatorArg,
        /// On or off.
        state: OnOff,
    },
    /// Show a screen.
    Screen {
        /// Screen.
        screen: ScreenArg,
    },
    /// Choose what the pet does (also switches to the PET screen).
    Pet {
        /// Pet mode; `auto` lets the pet pick its own activity.
        mode: PetArg,
    },
    /// Show text on the HOST screen (`\n` starts a new line).
    Text {
        /// Text, at most 256 bytes.
        text: String,
        /// Foreground colour.
        #[arg(long, value_parser = parse_color)]
        fg: Option<Color>,
        /// Background colour.
        #[arg(long, value_parser = parse_color)]
        bg: Option<Color>,
    },
    /// Fill the HOST canvas.
    Fill {
        /// Colour.
        #[arg(value_parser = parse_color)]
        color: Color,
    },
    /// Paint a rectangle on the HOST canvas.
    Rect {
        /// Left.
        x: u32,
        /// Top.
        y: u32,
        /// Width.
        w: u32,
        /// Height.
        h: u32,
        /// Colour.
        #[arg(value_parser = parse_color)]
        color: Color,
    },
    /// GPIO probe pins.
    Gpio {
        /// Operation.
        #[command(subcommand)]
        op: GpioCmd,
    },
    /// Wi-Fi station.
    Wifi {
        /// Operation.
        #[command(subcommand)]
        op: WifiCmd,
    },
    /// Bluetooth LE.
    Ble {
        /// Operation.
        #[command(subcommand)]
        op: BleCmd,
    },
    /// Choose the Chi offered in trades.
    Chi {
        /// Chi tag.
        chi: ChiArg,
    },
    /// Chi trades with lobby peers.
    Trade {
        /// Operation.
        #[command(subcommand)]
        op: TradeCmd,
    },
    /// Badges heard in the trade lobby, plus the current trade.
    Lobby,
    /// Stored contacts (one page of 8, or every page with `--all`).
    #[command(args_conflicts_with_subcommands = true)]
    Contacts {
        /// Operation (omit to list).
        #[command(subcommand)]
        op: Option<ContactsCmd>,
        /// First contact to list.
        #[arg(long, default_value_t = 0)]
        offset: u32,
        /// Page through until every stored contact is listed.
        #[arg(long)]
        all: bool,
    },
    /// LED sync state (leader, offset, peers).
    Sync,
    /// The on-badge wasm app.
    Wasm {
        /// Operation.
        #[command(subcommand)]
        op: WasmOp,
    },
    /// Sample ADC1-capable probe pins (dev firmware), e.g. to look for a
    /// battery voltage divider.
    Adc {
        /// Operation.
        #[command(subcommand)]
        op: AdcCmd,
    },
}

/// `adc` operations.
#[derive(Debug, Clone, PartialEq, Subcommand)]
pub enum AdcCmd {
    /// Sample one pin (GPIO 1, 3, 5, 6, 7 or 8).
    Read {
        /// GPIO number.
        pin: u32,
        /// Samples to average (1..=64; default 16).
        #[arg(long, default_value_t = 16, value_parser = clap::value_parser!(u32).range(1..=64))]
        samples: u32,
    },
    /// Sample every ADC1 probe pin, one line each.
    Scan {
        /// Samples to average per pin (1..=64; default 16).
        #[arg(long, default_value_t = 16, value_parser = clap::value_parser!(u32).range(1..=64))]
        samples: u32,
    },
}

/// `wasm` operations.
#[derive(Debug, Clone, PartialEq, Subcommand)]
pub enum WasmOp {
    /// Upload a module (chunked `WasmLoad`), then print the runtime status.
    Load {
        /// The `.wasm` file (at most 16 KiB).
        file: PathBuf,
    },
    /// Instantiate the loaded module and start ticking it.
    Start,
    /// Stop the app (the module stays loaded).
    Stop,
    /// Runtime state, module size, ticks and fuel.
    Status,
}

/// `trade` operations.
#[derive(Debug, Clone, PartialEq, Subcommand)]
pub enum TradeCmd {
    /// Invite a lobby peer to trade.
    Invite {
        /// Peer id: 6 hex digits (last 6 of its unique id).
        #[arg(value_parser = parse_peer_id)]
        peer: PeerId,
    },
    /// Cancel the current trade.
    Cancel,
}

/// `contacts` operations.
#[derive(Debug, Clone, PartialEq, Subcommand)]
pub enum ContactsCmd {
    /// Delete every stored contact.
    Clear,
}

/// Options of `adopt`.
#[derive(Debug, Clone, PartialEq, Eq, Args)]
pub struct AdoptArgs {
    /// Do not ask for confirmation.
    #[arg(long)]
    pub yes: bool,
    /// Skip the backup and flash over JTAG with probe-rs (boots directly).
    #[arg(long, conflicts_with = "force")]
    pub no_backup: bool,
    /// Overwrite an existing backup of the same badge.
    #[arg(long)]
    pub force: bool,
    /// Image to flash: a firmware ELF (default: the release build) or a
    /// merged `.bin` written at 0x0 (at most 0x200000 bytes).
    #[arg(long)]
    pub source: Option<PathBuf>,
    /// Copy bootloader, partition table and factory app from the attached
    /// badge with this probe serial (its MAC) instead of a file.
    #[arg(long, conflicts_with = "source")]
    pub from_badge: Option<String>,
    /// Partition table CSV used with an ELF source.
    #[arg(long)]
    pub partitions: Option<PathBuf>,
}

/// `gpio` operations.
#[derive(Debug, Clone, PartialEq, Subcommand)]
pub enum GpioCmd {
    /// Read a probe pin.
    Read {
        /// GPIO number.
        pin: u32,
    },
    /// Drive a probe pin.
    Write {
        /// GPIO number.
        pin: u32,
        /// Level.
        #[arg(value_parser = clap::value_parser!(u8).range(0..=1))]
        level: u8,
    },
}

/// `wifi` operations.
#[derive(Debug, Clone, PartialEq, Subcommand)]
pub enum WifiCmd {
    /// Scan for access points.
    Scan,
    /// Join a network.
    Connect {
        /// SSID (at most 32 bytes).
        ssid: String,
        /// WPA2 password; prefer BADGE_WIFI_PASSWORD to keep it out of shell history.
        #[arg(long, env = "BADGE_WIFI_PASSWORD", hide_env_values = true)]
        password: Option<String>,
    },
    /// Link state.
    Status,
}

/// `ble` operations.
#[derive(Debug, Clone, PartialEq, Subcommand)]
pub enum BleCmd {
    /// Scan for advertisers.
    Scan {
        /// Scan duration (0 = firmware default of 3000 ms).
        #[arg(long, default_value_t = 0)]
        duration_ms: u32,
    },
    /// Advertise a name (at most 29 bytes).
    Advertise {
        /// Local name.
        name: String,
    },
    /// Stop advertising.
    Stop,
    /// Advertisement rates, unique addresses, OZ badges, lobby size.
    Metrics,
}

/// Mailbox commands for `poke`.
#[derive(Debug, Clone, PartialEq, Subcommand)]
pub enum PokeCmd {
    /// Set LED `index` to `color`.
    SetLed {
        /// LED index 0..7.
        index: u32,
        /// Colour.
        #[arg(value_parser = parse_color)]
        color: Color,
    },
    /// Show a screen.
    Screen {
        /// Screen.
        screen: ScreenArg,
    },
    /// Select the LED pattern.
    Pattern {
        /// Pattern.
        pattern: PatternArg,
        /// Brightness (omit or 0 to keep).
        #[arg(long)]
        brightness: Option<u8>,
    },
    /// Choose what the pet does.
    Pet {
        /// Pet mode.
        mode: PetArg,
    },
    /// Invite a lobby peer to a Chi trade.
    TradeInvite {
        /// Peer id: 6 hex digits.
        #[arg(value_parser = parse_peer_id)]
        peer: PeerId,
    },
    /// Choose the Chi offered in trades.
    Chi {
        /// Chi tag.
        chi: ChiArg,
    },
}

/// LED pattern names.
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum PatternArg {
    /// All off.
    Off,
    /// Colours set by `led` / `leds`.
    Manual,
    /// Rainbow.
    Rainbow,
    /// Breathing.
    Breathe,
}

/// Chi tags (the stock firmware's `CHI_TAGS`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum ChiArg {
    /// VIB.
    Vib,
    /// HCK.
    Hck,
    /// PKT.
    Pkt,
    /// ADM.
    Adm,
    /// CRY.
    Cry,
    /// OVR.
    Ovr,
}

/// Indicator LED names.
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum IndicatorArg {
    /// Red alert LED.
    Alert,
    /// Green raffle LED.
    Raffle,
}

/// On / off switch.
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum OnOff {
    /// On.
    On,
    /// Off.
    Off,
}

/// Screen names.
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum ScreenArg {
    /// Home.
    Home,
    /// Buttons.
    Buttons,
    /// LEDs.
    Leds,
    /// Wi-Fi.
    Wifi,
    /// Bluetooth LE.
    Ble,
    /// GPIO.
    Gpio,
    /// Host text / canvas.
    Host,
    /// Animated pet.
    Pet,
    /// Chi trade lobby.
    Trade,
    /// Contacts from completed trades.
    Contacts,
}

/// Pet mode names.
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum PetArg {
    /// The pet picks its own activity.
    Auto,
    /// Sit still.
    Idle,
    /// Walk around.
    Walk,
    /// Eat.
    Eat,
    /// Play.
    Play,
    /// Sleep.
    Sleep,
}

/// Parses a 6-hex-digit peer id (either case; stored uppercase).
pub fn parse_peer_id(input: &str) -> Result<PeerId, String> {
    PeerId::parse(input.trim().as_bytes())
        .map_err(|_| format!("invalid peer id `{input}`: expected 6 hex digits like 0AC8F6"))
}

/// Parses a decimal or `0x`-prefixed hexadecimal address.
pub fn parse_u64(input: &str) -> Result<u64, String> {
    let t = input.trim();
    let parsed = match t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
        Some(hex) => u64::from_str_radix(hex, 16),
        None => t.parse(),
    };
    parsed.map_err(|e| format!("invalid address `{input}`: {e}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cli_definition_is_consistent() {
        use clap::CommandFactory;
        Cli::command().debug_assert();
    }

    #[test]
    fn globals_parse_after_subcommand() {
        let cli = Cli::try_parse_from(["badgectl", "ping", "--timeout", "7", "--json"]).unwrap();
        assert_eq!(cli.timeout, 7);
        assert!(cli.json);
        assert_eq!(cli.chip, "esp32s3");
        assert!(matches!(cli.command, Command::Request(RequestCmd::Ping { nonce: None })));
    }

    #[test]
    fn leds_limits_count() {
        let nine = ["badgectl", "leds", "red", "red", "red", "red", "red", "red", "red", "red", "red"];
        assert!(Cli::try_parse_from(nine).is_err());
        assert!(Cli::try_parse_from(["badgectl", "leds"]).is_err());
        assert!(Cli::try_parse_from(["badgectl", "leds", "red", "#00ff00"]).is_ok());
    }

    #[test]
    fn gpio_level_is_bit() {
        assert!(Cli::try_parse_from(["badgectl", "gpio", "write", "5", "2"]).is_err());
        assert!(Cli::try_parse_from(["badgectl", "gpio", "write", "5", "1"]).is_ok());
    }

    #[test]
    fn pet_modes_parse() {
        for (name, want) in [
            ("auto", PetArg::Auto),
            ("idle", PetArg::Idle),
            ("walk", PetArg::Walk),
            ("eat", PetArg::Eat),
            ("play", PetArg::Play),
            ("sleep", PetArg::Sleep),
        ] {
            let cli = Cli::try_parse_from(["badgectl", "pet", name]).unwrap();
            assert!(matches!(cli.command, Command::Request(RequestCmd::Pet { mode }) if mode == want));
            let cli = Cli::try_parse_from(["badgectl", "poke", "pet", name]).unwrap();
            assert!(matches!(cli.command, Command::Poke { cmd: PokeCmd::Pet { mode }, .. } if mode == want));
        }
        assert!(Cli::try_parse_from(["badgectl", "pet"]).is_err());
        assert!(Cli::try_parse_from(["badgectl", "pet", "dance"]).is_err());
        let cli = Cli::try_parse_from(["badgectl", "screen", "pet"]).unwrap();
        assert!(matches!(cli.command, Command::Request(RequestCmd::Screen { screen: ScreenArg::Pet })));
    }

    fn parse(args: &[&str]) -> Result<Command, clap::Error> {
        let argv: Vec<&str> = std::iter::once("badgectl").chain(args.iter().copied()).collect();
        Cli::try_parse_from(argv).map(|c| c.command)
    }

    #[test]
    fn peer_ids_parse() {
        assert_eq!(parse_peer_id("0ac8f6").map(|p| p.to_u32()), Ok(0x0A_C8F6));
        assert_eq!(parse_peer_id(" 0AC8F6 ").map(|p| p.to_u32()), Ok(0x0A_C8F6));
        assert!(parse_peer_id("0AC8F").is_err());
        assert!(parse_peer_id("0AC8F6A").is_err());
        assert!(parse_peer_id("0AC8G6").is_err());
        assert!(parse(&["trade", "invite", "zzzzzz"]).is_err());
        assert!(parse(&["trade", "invite"]).is_err());
    }

    #[test]
    fn trade_chi_lobby_sync_parse() {
        let Ok(Command::Request(RequestCmd::Trade { op: TradeCmd::Invite { peer } })) =
            parse(&["trade", "invite", "0ac8f6"])
        else {
            panic!("trade invite")
        };
        assert_eq!(peer.as_str(), "0AC8F6");
        assert!(matches!(parse(&["trade", "cancel"]), Ok(Command::Request(RequestCmd::Trade { op: TradeCmd::Cancel }))));
        for (name, want) in [
            ("vib", ChiArg::Vib),
            ("hck", ChiArg::Hck),
            ("pkt", ChiArg::Pkt),
            ("adm", ChiArg::Adm),
            ("cry", ChiArg::Cry),
            ("ovr", ChiArg::Ovr),
        ] {
            assert!(matches!(parse(&["chi", name]), Ok(Command::Request(RequestCmd::Chi { chi })) if chi == want));
            assert!(matches!(parse(&["poke", "chi", name]), Ok(Command::Poke { cmd: PokeCmd::Chi { chi }, .. }) if chi == want));
        }
        assert!(parse(&["chi", "zzz"]).is_err());
        assert!(matches!(parse(&["lobby"]), Ok(Command::Request(RequestCmd::Lobby))));
        assert!(matches!(parse(&["sync"]), Ok(Command::Request(RequestCmd::Sync))));
        assert!(matches!(parse(&["ble", "metrics"]), Ok(Command::Request(RequestCmd::Ble { op: BleCmd::Metrics }))));
        let Ok(Command::Poke { cmd: PokeCmd::TradeInvite { peer }, .. }) = parse(&["poke", "trade-invite", "00ABCD"]) else {
            panic!("poke trade-invite")
        };
        assert_eq!(peer.to_u32(), 0xABCD);
    }

    #[test]
    fn contacts_parse() {
        assert!(matches!(
            parse(&["contacts"]),
            Ok(Command::Request(RequestCmd::Contacts { op: None, offset: 0, all: false }))
        ));
        assert!(matches!(
            parse(&["contacts", "--offset", "8"]),
            Ok(Command::Request(RequestCmd::Contacts { op: None, offset: 8, all: false }))
        ));
        assert!(matches!(
            parse(&["contacts", "--all"]),
            Ok(Command::Request(RequestCmd::Contacts { op: None, offset: 0, all: true }))
        ));
        assert!(matches!(
            parse(&["contacts", "clear"]),
            Ok(Command::Request(RequestCmd::Contacts { op: Some(ContactsCmd::Clear), .. }))
        ));
        assert!(parse(&["contacts", "--all", "clear"]).is_err());
    }

    #[test]
    fn removed_patterns_and_new_screens() {
        assert!(parse(&["pattern", "chase"]).is_err());
        assert!(parse(&["pattern", "buttons"]).is_err());
        assert!(parse(&["pattern", "breathe"]).is_ok());
        assert!(matches!(parse(&["screen", "trade"]), Ok(Command::Request(RequestCmd::Screen { screen: ScreenArg::Trade }))));
        assert!(matches!(parse(&["screen", "contacts"]), Ok(Command::Request(RequestCmd::Screen { screen: ScreenArg::Contacts }))));
    }

    #[test]
    fn adopt_and_restore_parse() {
        let Ok(Command::Adopt(a)) = parse(&["adopt"]) else { panic!("adopt") };
        assert!(!a.yes && !a.no_backup && !a.force && a.source.is_none() && a.from_badge.is_none());
        let Ok(Command::Adopt(a)) = parse(&["adopt", "--yes", "--force", "--source", "x.bin"]) else { panic!() };
        assert!(a.yes && a.force);
        assert_eq!(a.source, Some(PathBuf::from("x.bin")));
        let Ok(Command::Adopt(a)) = parse(&["adopt", "--from-badge", "E8:3D:C1:00:11:22"]) else { panic!() };
        assert_eq!(a.from_badge.as_deref(), Some("E8:3D:C1:00:11:22"));
        assert!(parse(&["adopt", "--no-backup", "--force"]).is_err());
        assert!(parse(&["adopt", "--source", "a.elf", "--from-badge", "x"]).is_err());
        let Ok(Command::Restore { backup }) = parse(&["restore", "b.bin", "--probe", "303a:1001:E8"]) else {
            panic!("restore")
        };
        assert_eq!(backup, PathBuf::from("b.bin"));
        assert!(parse(&["restore"]).is_err());
    }

    #[test]
    fn adc_parse() {
        assert!(matches!(
            parse(&["adc", "read", "3"]),
            Ok(Command::Request(RequestCmd::Adc { op: AdcCmd::Read { pin: 3, samples: 16 } }))
        ));
        assert!(matches!(
            parse(&["adc", "scan", "--samples", "64"]),
            Ok(Command::Request(RequestCmd::Adc { op: AdcCmd::Scan { samples: 64 } }))
        ));
        assert!(parse(&["adc", "scan", "--samples", "65"]).is_err());
        assert!(parse(&["adc", "read"]).is_err());
    }

    #[test]
    fn wasm_and_sim_parse() {
        let Ok(Command::Request(RequestCmd::Wasm { op: WasmOp::Load { file } })) = parse(&["wasm", "load", "a.wasm"])
        else {
            panic!("wasm load")
        };
        assert_eq!(file, PathBuf::from("a.wasm"));
        for (name, want) in [("start", WasmOp::Start), ("stop", WasmOp::Stop), ("status", WasmOp::Status)] {
            assert!(matches!(parse(&["wasm", name]), Ok(Command::Request(RequestCmd::Wasm { op })) if op == want));
        }
        assert!(parse(&["wasm", "load"]).is_err());
        let Ok(Command::Sim(a)) = parse(&["sim"]) else { panic!("sim") };
        assert_eq!((a.badges, a.duration_ms, a.rssi, a.mac_base), (2, 10_000, -40, 0x0200_c0ff_ee00));
        assert!(!a.realtime && !a.events && a.zenoh.is_empty());
        let Ok(Command::Sim(a)) =
            parse(&["sim", "--badges", "3", "--rssi", "-50", "--zenoh", "tcp/127.0.0.1:7447", "--app-key", "app/x"])
        else {
            panic!("sim opts")
        };
        assert_eq!((a.badges, a.rssi), (3, -50));
        assert_eq!(a.zenoh, ["tcp/127.0.0.1:7447"]);
        assert_eq!(a.app_keys, ["app/x"]);
        let cli = Cli::try_parse_from(["badgectl", "--sim", "1", "lobby"]).unwrap();
        assert_eq!((cli.sim, cli.sim_badges, cli.sim_warmup_ms), (Some(1), 2, 3000));
    }

    #[test]
    fn addresses_parse() {
        assert_eq!(parse_u64("0x3fc8_0000".replace('_', "").as_str()), Ok(0x3fc8_0000));
        assert_eq!(parse_u64("4096"), Ok(4096));
        assert!(parse_u64("zz").is_err());
    }
}
