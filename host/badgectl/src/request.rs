//! CLI arguments to protocol messages.
//!
//! Pure functions: [`host_msg`] builds the `HostMsg` for a request
//! subcommand, [`timeout_ms`] picks how long to wait for its response, and
//! [`poke_words`] maps a `poke` subcommand onto the PROBE mailbox words.
//! [`next_contacts_offset`] drives `contacts --all` paging and [`hint`]
//! explains error responses that have a known cause.
//! Capacity limits of the generated types are checked here so the user gets
//! a clear message instead of a silently truncated field.

use badge_core::color::Color;
use badge_core::probe_layout::mailbox;
use badge_core::oz::PeerId;
use badge_proto::{
    BleAdvertise, BleScan, Chi, ClearContacts, Contacts, DisplayFill, DisplayRect, DisplayText, ErrorCode,
    GetBleMetrics, GetButtons, GetInfo, GetLobby, GetSync, GpioRead, GpioWrite, HostMsg, HostMsg_::Cmd,
    Indicator, LedPattern, ListContacts, PetMode, Ping, Response, Response_::Body, Rgb, Screen, SetChi,
    SetIndicator, SetLed, SetLeds, SetPattern, SetPetMode, ShowScreen, TradeCancel, TradeInvite, WifiConnect,
    WifiScan, WifiStatus,
};

use crate::cli::{
    BleCmd, ChiArg, ContactsCmd, GpioCmd, IndicatorArg, OnOff, PatternArg, PetArg, PokeCmd, RequestCmd,
    ScreenArg, TradeCmd, WifiCmd,
};

/// Minimum wait for requests that run a radio scan or join a network.
pub const SLOW_TIMEOUT_MS: u64 = 15_000;

/// A request cannot be represented in the protocol.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum RequestError {
    /// A string is longer than its fixed protocol capacity.
    #[error("{field} is {len} bytes; the protocol allows at most {max}")]
    TooLong {
        /// Field name.
        field: &'static str,
        /// Actual length in bytes.
        len: usize,
        /// Capacity in bytes.
        max: usize,
    },
    /// `leds` needs between 1 and 8 colours.
    #[error("expected 1 to 8 colours, got {0}")]
    LedCount(usize),
}

fn rgb(c: Color) -> Rgb {
    Rgb { rgb: c.packed() }
}

fn hstring<const N: usize>(field: &'static str, s: &str) -> Result<heapless::String<N>, RequestError> {
    let mut out = heapless::String::new();
    out.push_str(s)
        .map_err(|_| RequestError::TooLong { field, len: s.len(), max: N })?;
    Ok(out)
}

/// Protocol value of a pattern argument.
pub fn pattern(p: PatternArg) -> LedPattern {
    match p {
        PatternArg::Off => LedPattern::Off,
        PatternArg::Manual => LedPattern::Manual,
        PatternArg::Rainbow => LedPattern::Rainbow,
        PatternArg::Breathe => LedPattern::Breathe,
    }
}

/// Protocol value of a Chi argument.
pub fn chi(c: ChiArg) -> Chi {
    match c {
        ChiArg::Vib => Chi::Vib,
        ChiArg::Hck => Chi::Hck,
        ChiArg::Pkt => Chi::Pkt,
        ChiArg::Adm => Chi::Adm,
        ChiArg::Cry => Chi::Cry,
        ChiArg::Ovr => Chi::Ovr,
    }
}

fn peer_string(peer: PeerId) -> Result<heapless::String<6>, RequestError> {
    hstring("peer", peer.as_str())
}

/// Protocol value of a screen argument.
pub fn screen(s: ScreenArg) -> Screen {
    match s {
        ScreenArg::Home => Screen::Home,
        ScreenArg::Buttons => Screen::Buttons,
        ScreenArg::Leds => Screen::Leds,
        ScreenArg::Wifi => Screen::Wifi,
        ScreenArg::Ble => Screen::Ble,
        ScreenArg::Gpio => Screen::Gpio,
        ScreenArg::Host => Screen::Host,
        ScreenArg::Pet => Screen::Pet,
        ScreenArg::Trade => Screen::Trade,
        ScreenArg::Contacts => Screen::Contacts,
    }
}

/// Protocol value of a pet mode argument.
pub fn pet_mode(m: PetArg) -> PetMode {
    match m {
        PetArg::Auto => PetMode::Auto,
        PetArg::Idle => PetMode::Idle,
        PetArg::Walk => PetMode::Walk,
        PetArg::Eat => PetMode::Eat,
        PetArg::Play => PetMode::Play,
        PetArg::Sleep => PetMode::Sleep,
    }
}

/// Turns the two-character escape `\n` typed on a shell into a newline.
pub fn unescape_newlines(text: &str) -> String {
    text.replace("\\n", "\n")
}

/// Builds the command payload for a request subcommand.
///
/// `seq` doubles as the default ping nonce.
pub fn command(req: &RequestCmd, seq: u32) -> Result<Cmd, RequestError> {
    Ok(match req {
        RequestCmd::Ping { nonce } => Cmd::Ping(Ping { nonce: nonce.unwrap_or(seq) }),
        RequestCmd::Info => Cmd::GetInfo(GetInfo {}),
        RequestCmd::Buttons => Cmd::GetButtons(GetButtons {}),
        RequestCmd::Led { index, color } => Cmd::SetLed(SetLed { index: *index, color: Some(rgb(*color)) }),
        RequestCmd::Leds { colors } => {
            let mut out = heapless::Vec::<Rgb, 8>::new();
            if colors.is_empty() {
                return Err(RequestError::LedCount(0));
            }
            for c in colors {
                out.push(rgb(*c)).map_err(|_| RequestError::LedCount(colors.len()))?;
            }
            Cmd::SetLeds(SetLeds { colors: out })
        }
        RequestCmd::Pattern { pattern: p, brightness } => Cmd::SetPattern(SetPattern {
            pattern: pattern(*p),
            brightness: u32::from(brightness.unwrap_or(0)),
        }),
        RequestCmd::Indicator { which, state } => Cmd::SetIndicator(SetIndicator {
            which: match which {
                IndicatorArg::Alert => Indicator::Alert,
                IndicatorArg::Raffle => Indicator::Raffle,
            },
            on: *state == OnOff::On,
        }),
        RequestCmd::Screen { screen: s } => Cmd::ShowScreen(ShowScreen { screen: screen(*s) }),
        RequestCmd::Pet { mode } => Cmd::SetPetMode(SetPetMode { mode: pet_mode(*mode) }),
        RequestCmd::Text { text, fg, bg } => Cmd::DisplayText(DisplayText {
            text: hstring("text", &unescape_newlines(text))?,
            fg: fg.map(rgb),
            bg: bg.map(rgb),
        }),
        RequestCmd::Fill { color } => Cmd::DisplayFill(DisplayFill { color: Some(rgb(*color)) }),
        RequestCmd::Rect { x, y, w, h, color } => Cmd::DisplayRect(DisplayRect {
            x: *x,
            y: *y,
            w: *w,
            h: *h,
            color: Some(rgb(*color)),
        }),
        RequestCmd::Gpio { op: GpioCmd::Read { pin } } => Cmd::GpioRead(GpioRead { pin: *pin }),
        RequestCmd::Gpio { op: GpioCmd::Write { pin, level } } => {
            Cmd::GpioWrite(GpioWrite { pin: *pin, level: *level != 0 })
        }
        RequestCmd::Wifi { op: WifiCmd::Scan } => Cmd::WifiScan(WifiScan {}),
        RequestCmd::Wifi { op: WifiCmd::Connect { ssid, password } } => Cmd::WifiConnect(WifiConnect {
            ssid: hstring("ssid", ssid)?,
            password: hstring("password", password.as_deref().unwrap_or(""))?,
        }),
        RequestCmd::Wifi { op: WifiCmd::Status } => Cmd::WifiStatus(WifiStatus {}),
        RequestCmd::Ble { op: BleCmd::Scan { duration_ms } } => {
            Cmd::BleScan(BleScan { duration_ms: *duration_ms })
        }
        RequestCmd::Ble { op: BleCmd::Advertise { name } } => {
            Cmd::BleAdvertise(BleAdvertise { name: hstring("name", name)? })
        }
        RequestCmd::Ble { op: BleCmd::Stop } => Cmd::BleAdvertise(BleAdvertise::default()),
        RequestCmd::Ble { op: BleCmd::Metrics } => Cmd::GetBleMetrics(GetBleMetrics {}),
        RequestCmd::Chi { chi: c } => Cmd::SetChi(SetChi { chi: chi(*c) }),
        RequestCmd::Trade { op: TradeCmd::Invite { peer } } => {
            Cmd::TradeInvite(TradeInvite { peer: peer_string(*peer)? })
        }
        RequestCmd::Trade { op: TradeCmd::Cancel } => Cmd::TradeCancel(TradeCancel {}),
        RequestCmd::Lobby => Cmd::GetLobby(GetLobby {}),
        RequestCmd::Contacts { op: Some(ContactsCmd::Clear), .. } => Cmd::ClearContacts(ClearContacts {}),
        RequestCmd::Contacts { op: None, offset, .. } => Cmd::ListContacts(ListContacts { offset: *offset }),
        RequestCmd::Sync => Cmd::GetSync(GetSync {}),
    })
}

/// Builds the complete `HostMsg` for a request subcommand.
pub fn host_msg(req: &RequestCmd, seq: u32) -> Result<HostMsg, RequestError> {
    Ok(HostMsg { seq, cmd: Some(command(req, seq)?) })
}

/// How long to wait for the response to `req`, given the `--timeout` value.
pub fn timeout_ms(req: &RequestCmd, base_ms: u64) -> u64 {
    match req {
        RequestCmd::Wifi { op: WifiCmd::Scan | WifiCmd::Connect { .. } } => base_ms.max(SLOW_TIMEOUT_MS),
        RequestCmd::Ble { op: BleCmd::Scan { duration_ms } } => {
            base_ms.max(SLOW_TIMEOUT_MS).max(u64::from(*duration_ms).saturating_add(5_000))
        }
        _ => base_ms,
    }
}

/// Mailbox `(cmd, arg0, arg1)` for a `poke` subcommand.
pub fn poke_words(cmd: &PokeCmd) -> (u32, u32, u32) {
    match cmd {
        PokeCmd::SetLed { index, color } => (mailbox::SET_LED, *index, color.packed()),
        PokeCmd::Screen { screen: s } => (mailbox::SHOW_SCREEN, screen(*s).0 as u32, 0),
        PokeCmd::Pattern { pattern: p, brightness } => {
            (mailbox::SET_PATTERN, pattern(*p).0 as u32, u32::from(brightness.unwrap_or(0)))
        }
        PokeCmd::Pet { mode } => (mailbox::SET_PET_MODE, pet_mode(*mode).0 as u32, 0),
        PokeCmd::TradeInvite { peer } => (mailbox::TRADE_INVITE, peer.to_u32(), 0),
        PokeCmd::Chi { chi: c } => (mailbox::SET_CHI, chi(*c).0 as u32, 0),
    }
}

/// Upper bound on `contacts --all` requests (the badge stores far fewer
/// than `MAX_CONTACT_PAGES * 8` contacts), so a misbehaving badge cannot
/// keep the host looping.
pub const MAX_CONTACT_PAGES: usize = 64;

/// Offset of the next `ListContacts` page after `page`, or `None` when
/// every stored contact has been listed.
///
/// An empty page also ends the walk, even if `total` claims more, so a
/// badge whose store shrank mid-walk cannot cause an endless loop.
pub fn next_contacts_offset(page: &Contacts) -> Option<u32> {
    let len = u32::try_from(page.contacts.len()).ok()?;
    if len == 0 {
        return None;
    }
    let next = page.offset.checked_add(len)?;
    (next < page.total).then_some(next)
}

/// True if `req` is `contacts --all` (paged listing).
pub fn is_contacts_all(req: &RequestCmd) -> bool {
    matches!(req, RequestCmd::Contacts { op: None, all: true, .. })
}

/// The same contacts request starting at `offset`.
pub fn contacts_at(offset: u32) -> RequestCmd {
    RequestCmd::Contacts { op: None, offset, all: false }
}

/// A hint to print under an error response whose cause is known.
pub fn hint(req: &RequestCmd, resp: &Response) -> Option<&'static str> {
    match (req, &resp.body) {
        (RequestCmd::Gpio { .. }, Some(Body::Error(e))) if e.code == ErrorCode::Unsupported => Some(
            "hint: the GPIO probe is only built into development firmware; \
             rebuild it with `cargo build --release --features dev` and reflash",
        ),
        _ => None,
    }
}

/// First sequence number for this process, derived from some entropy so two
/// back-to-back invocations are unlikely to reuse a `seq`.
///
/// The top bit is always clear, so the result never equals
/// [`badge_core::probe_layout::MAILBOX_SEQ`], and it is never 0.
pub fn initial_seq(entropy: u64) -> u32 {
    let folded = ((entropy ^ (entropy >> 32)) as u32) & 0x7fff_ffff;
    folded.max(1)
}

/// The sequence number after `seq`, staying in `1..=0x7fff_ffff` like
/// [`initial_seq`].
pub fn next_seq(seq: u32) -> u32 {
    (seq & 0x7fff_ffff) % 0x7fff_ffff + 1
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::{Cli, Command};
    use badge_core::probe_layout::{MAILBOX_SEQ, mailbox_to_host_msg};
    use clap::Parser;

    fn parse(args: &[&str]) -> RequestCmd {
        let argv: Vec<&str> = std::iter::once("badgectl").chain(args.iter().copied()).collect();
        match Cli::try_parse_from(argv).expect("parse").command {
            Command::Request(r) => r,
            other => panic!("not a request: {other:?}"),
        }
    }

    fn cmd(args: &[&str]) -> Cmd {
        command(&parse(args), 42).expect("command")
    }

    #[test]
    fn ping_defaults_nonce_to_seq() {
        assert_eq!(cmd(&["ping"]), Cmd::Ping(Ping { nonce: 42 }));
        assert_eq!(cmd(&["ping", "7"]), Cmd::Ping(Ping { nonce: 7 }));
        let msg = host_msg(&parse(&["info"]), 9).unwrap();
        assert_eq!(msg.seq, 9);
        assert_eq!(msg.cmd, Some(Cmd::GetInfo(GetInfo {})));
    }

    #[test]
    fn led_commands() {
        assert_eq!(
            cmd(&["led", "3", "#ff8000"]),
            Cmd::SetLed(SetLed { index: 3, color: Some(Rgb { rgb: 0xff8000 }) })
        );
        let Cmd::SetLeds(set) = cmd(&["leds", "red", "0x00ff00", "0000ff"]) else { panic!() };
        let words: Vec<u32> = set.colors.iter().map(|c| c.rgb).collect();
        assert_eq!(words, [0xff0000, 0x00ff00, 0x0000ff]);
        assert_eq!(
            command(&RequestCmd::Leds { colors: vec![] }, 1),
            Err(RequestError::LedCount(0))
        );
        assert_eq!(
            command(&RequestCmd::Leds { colors: vec![Color::BLACK; 9] }, 1),
            Err(RequestError::LedCount(9))
        );
    }

    #[test]
    fn pattern_indicator_screen() {
        assert_eq!(
            cmd(&["pattern", "rainbow", "--brightness", "40"]),
            Cmd::SetPattern(SetPattern { pattern: LedPattern::Rainbow, brightness: 40 })
        );
        assert_eq!(
            cmd(&["pattern", "off"]),
            Cmd::SetPattern(SetPattern { pattern: LedPattern::Off, brightness: 0 })
        );
        assert_eq!(
            cmd(&["indicator", "raffle", "on"]),
            Cmd::SetIndicator(SetIndicator { which: Indicator::Raffle, on: true })
        );
        assert_eq!(
            cmd(&["indicator", "alert", "off"]),
            Cmd::SetIndicator(SetIndicator { which: Indicator::Alert, on: false })
        );
        assert_eq!(cmd(&["screen", "wifi"]), Cmd::ShowScreen(ShowScreen { screen: Screen::Wifi }));
        assert_eq!(cmd(&["screen", "host"]), Cmd::ShowScreen(ShowScreen { screen: Screen::Host }));
    }

    #[test]
    fn display_commands() {
        let Cmd::DisplayText(t) = cmd(&["text", "hi\\nthere", "--fg", "white", "--bg", "blue"]) else {
            panic!()
        };
        assert_eq!(t.text.as_str(), "hi\nthere");
        assert_eq!(t.fg, Some(Rgb { rgb: 0xffffff }));
        assert_eq!(t.bg, Some(Rgb { rgb: 0x0000ff }));
        let long = "x".repeat(257);
        assert_eq!(
            command(&RequestCmd::Text { text: long, fg: None, bg: None }, 1),
            Err(RequestError::TooLong { field: "text", len: 257, max: 256 })
        );
        assert_eq!(cmd(&["fill", "off"]), Cmd::DisplayFill(DisplayFill { color: Some(Rgb { rgb: 0 }) }));
        assert_eq!(
            cmd(&["rect", "1", "2", "30", "40", "red"]),
            Cmd::DisplayRect(DisplayRect { x: 1, y: 2, w: 30, h: 40, color: Some(Rgb { rgb: 0xff0000 }) })
        );
    }

    #[test]
    fn gpio_commands() {
        assert_eq!(cmd(&["gpio", "read", "5"]), Cmd::GpioRead(GpioRead { pin: 5 }));
        assert_eq!(cmd(&["gpio", "write", "6", "1"]), Cmd::GpioWrite(GpioWrite { pin: 6, level: true }));
        assert_eq!(cmd(&["gpio", "write", "6", "0"]), Cmd::GpioWrite(GpioWrite { pin: 6, level: false }));
    }

    #[test]
    fn wifi_commands() {
        assert_eq!(cmd(&["wifi", "scan"]), Cmd::WifiScan(WifiScan {}));
        assert_eq!(cmd(&["wifi", "status"]), Cmd::WifiStatus(WifiStatus {}));
        let Cmd::WifiConnect(c) = cmd(&["wifi", "connect", "lab", "--password", "hunter22"]) else { panic!() };
        assert_eq!((c.ssid.as_str(), c.password.as_str()), ("lab", "hunter22"));
        let ssid = "s".repeat(33);
        assert!(matches!(
            command(&RequestCmd::Wifi { op: WifiCmd::Connect { ssid, password: None } }, 1),
            Err(RequestError::TooLong { field: "ssid", .. })
        ));
        let password = Some("p".repeat(65));
        assert!(matches!(
            command(&RequestCmd::Wifi { op: WifiCmd::Connect { ssid: "a".into(), password } }, 1),
            Err(RequestError::TooLong { field: "password", .. })
        ));
    }

    #[test]
    fn ble_commands() {
        assert_eq!(cmd(&["ble", "scan"]), Cmd::BleScan(BleScan { duration_ms: 0 }));
        assert_eq!(cmd(&["ble", "scan", "--duration-ms", "5000"]), Cmd::BleScan(BleScan { duration_ms: 5000 }));
        let Cmd::BleAdvertise(a) = cmd(&["ble", "advertise", "conagotchi"]) else { panic!() };
        assert_eq!(a.name.as_str(), "conagotchi");
        assert_eq!(cmd(&["ble", "stop"]), Cmd::BleAdvertise(BleAdvertise::default()));
        assert!(command(&RequestCmd::Ble { op: BleCmd::Advertise { name: "n".repeat(30) } }, 1).is_err());
    }

    #[test]
    fn every_request_encodes_within_host_msg_max() {
        let cases: &[&[&str]] = &[
            &["ping"], &["info"], &["buttons"], &["led", "7", "white"],
            &["leds", "red", "red", "red", "red", "red", "red", "red", "red"],
            &["pattern", "breathe", "--brightness", "255"], &["indicator", "alert", "on"],
            &["chi", "ovr"], &["trade", "invite", "FFFFFF"], &["trade", "cancel"], &["lobby"],
            &["contacts", "--offset", "4294967295"], &["contacts", "clear"], &["ble", "metrics"], &["sync"],
            &["screen", "contacts"],
            &["screen", "gpio"], &["fill", "cyan"], &["rect", "0", "0", "240", "240", "red"],
            &["gpio", "write", "48", "1"], &["screen", "pet"], &["pet", "sleep"], &["wifi", "scan"], &["ble", "scan", "--duration-ms", "9999"],
        ];
        for args in cases {
            let msg = host_msg(&parse(args), u32::MAX - 1).unwrap();
            let mut buf = [0u8; badge_proto::HOST_MSG_MAX];
            let n = badge_proto::encode(&msg, &mut buf).unwrap();
            let back: HostMsg = badge_proto::decode(&buf[..n]).unwrap();
            assert_eq!(back, msg, "{args:?}");
        }
        let text = RequestCmd::Text { text: "t".repeat(256), fg: Some(Color::WHITE), bg: Some(Color::BLACK) };
        let msg = host_msg(&text, u32::MAX).unwrap();
        let mut buf = [0u8; badge_proto::HOST_MSG_MAX];
        assert!(badge_proto::encode(&msg, &mut buf).is_ok());
    }

    #[test]
    fn slow_requests_get_longer_timeouts() {
        assert_eq!(timeout_ms(&parse(&["ping"]), 5000), 5000);
        assert_eq!(timeout_ms(&parse(&["wifi", "scan"]), 5000), 15_000);
        assert_eq!(timeout_ms(&parse(&["wifi", "scan"]), 30_000), 30_000);
        assert_eq!(timeout_ms(&parse(&["wifi", "connect", "x"]), 1000), 15_000);
        assert_eq!(timeout_ms(&parse(&["ble", "scan", "--duration-ms", "20000"]), 5000), 25_000);
        assert_eq!(timeout_ms(&parse(&["ble", "scan"]), 5000), 15_000);
    }

    #[test]
    fn poke_words_match_firmware_mailbox_mapping() {
        let cases = [
            PokeCmd::SetLed { index: 2, color: Color::new(1, 2, 3) },
            PokeCmd::Screen { screen: ScreenArg::Ble },
            PokeCmd::Pattern { pattern: PatternArg::Breathe, brightness: Some(9) },
            PokeCmd::Pet { mode: PetArg::Walk },
            PokeCmd::Pet { mode: PetArg::Auto },
            PokeCmd::Chi { chi: ChiArg::Pkt },
        ];
        let expected = [
            Cmd::SetLed(SetLed { index: 2, color: Some(Rgb { rgb: 0x010203 }) }),
            Cmd::ShowScreen(ShowScreen { screen: Screen::Ble }),
            Cmd::SetPattern(SetPattern { pattern: LedPattern::Breathe, brightness: 9 }),
            Cmd::SetPetMode(SetPetMode { mode: PetMode::Walk }),
            Cmd::SetPetMode(SetPetMode { mode: PetMode::Auto }),
            Cmd::SetChi(SetChi { chi: Chi::Pkt }),
        ];
        for (poke, want) in cases.iter().zip(expected) {
            let (c, a0, a1) = poke_words(poke);
            let msg = mailbox_to_host_msg(c, a0, a1).expect("known mailbox command");
            assert_eq!(msg.cmd, Some(want));
        }
    }

    #[test]
    fn pet_commands() {
        let cases = [
            ("auto", PetMode::Auto, 0),
            ("idle", PetMode::Idle, 1),
            ("walk", PetMode::Walk, 2),
            ("eat", PetMode::Eat, 3),
            ("play", PetMode::Play, 4),
            ("sleep", PetMode::Sleep, 5),
        ];
        for (name, mode, value) in cases {
            assert_eq!(cmd(&["pet", name]), Cmd::SetPetMode(SetPetMode { mode }));
            assert_eq!(mode.0, value);
        }
        assert_eq!(cmd(&["screen", "pet"]), Cmd::ShowScreen(ShowScreen { screen: Screen::Pet }));
        assert_eq!(timeout_ms(&parse(&["pet", "eat"]), 5000), 5000);
    }

    #[test]
    fn poke_pet_words() {
        assert_eq!(poke_words(&PokeCmd::Pet { mode: PetArg::Sleep }), (mailbox::SET_PET_MODE, 5, 0));
        assert_eq!(poke_words(&PokeCmd::Pet { mode: PetArg::Auto }), (4, 0, 0));
        assert_eq!(poke_words(&PokeCmd::Screen { screen: ScreenArg::Pet }), (mailbox::SHOW_SCREEN, 7, 0));
    }

    #[test]
    fn trade_chi_lobby_commands() {
        let Cmd::TradeInvite(t) = cmd(&["trade", "invite", "0ac8f6"]) else { panic!("trade invite") };
        assert_eq!(t.peer.as_str(), "0AC8F6");
        assert_eq!(cmd(&["trade", "cancel"]), Cmd::TradeCancel(TradeCancel {}));
        let chis = [
            ("vib", Chi::Vib, 1),
            ("hck", Chi::Hck, 2),
            ("pkt", Chi::Pkt, 3),
            ("adm", Chi::Adm, 4),
            ("cry", Chi::Cry, 5),
            ("ovr", Chi::Ovr, 6),
        ];
        for (name, want, value) in chis {
            assert_eq!(cmd(&["chi", name]), Cmd::SetChi(SetChi { chi: want }));
            assert_eq!(want.0, value);
        }
        assert_eq!(cmd(&["lobby"]), Cmd::GetLobby(GetLobby {}));
        assert_eq!(cmd(&["sync"]), Cmd::GetSync(GetSync {}));
        assert_eq!(cmd(&["ble", "metrics"]), Cmd::GetBleMetrics(GetBleMetrics {}));
        assert_eq!(cmd(&["screen", "trade"]), Cmd::ShowScreen(ShowScreen { screen: Screen::Trade }));
        assert_eq!(cmd(&["screen", "contacts"]), Cmd::ShowScreen(ShowScreen { screen: Screen::Contacts }));
        assert_eq!(Screen::Trade.0, 8);
        assert_eq!(Screen::Contacts.0, 9);
    }

    #[test]
    fn contacts_commands() {
        assert_eq!(cmd(&["contacts"]), Cmd::ListContacts(ListContacts { offset: 0 }));
        assert_eq!(cmd(&["contacts", "--offset", "16"]), Cmd::ListContacts(ListContacts { offset: 16 }));
        assert_eq!(cmd(&["contacts", "--all"]), Cmd::ListContacts(ListContacts { offset: 0 }));
        assert_eq!(cmd(&["contacts", "clear"]), Cmd::ClearContacts(ClearContacts {}));
        assert!(is_contacts_all(&parse(&["contacts", "--all"])));
        assert!(!is_contacts_all(&parse(&["contacts"])));
        assert!(!is_contacts_all(&parse(&["contacts", "clear"])));
        assert_eq!(contacts_at(8), RequestCmd::Contacts { op: None, offset: 8, all: false });
    }

    fn page(offset: u32, len: usize, total: u32) -> Contacts {
        let mut contacts = heapless::Vec::new();
        for _ in 0..len {
            contacts.push(badge_proto::Contact::default()).unwrap();
        }
        Contacts { contacts, total, offset }
    }

    #[test]
    fn contacts_paging() {
        assert_eq!(next_contacts_offset(&page(0, 8, 20)), Some(8));
        assert_eq!(next_contacts_offset(&page(8, 8, 20)), Some(16));
        assert_eq!(next_contacts_offset(&page(16, 4, 20)), None);
        assert_eq!(next_contacts_offset(&page(0, 8, 8)), None);
        assert_eq!(next_contacts_offset(&page(0, 0, 0)), None);
        // An empty page ends the walk even if total claims more.
        assert_eq!(next_contacts_offset(&page(8, 0, 20)), None);
        assert_eq!(next_contacts_offset(&page(u32::MAX, 1, u32::MAX)), None);
        // Walking a 20-contact store takes three requests.
        let mut offsets = vec![0];
        while let Some(next) = next_contacts_offset(&page(*offsets.last().unwrap(), 8.min(20 - *offsets.last().unwrap() as usize), 20)) {
            offsets.push(next);
            assert!(offsets.len() <= MAX_CONTACT_PAGES);
        }
        assert_eq!(offsets, [0, 8, 16]);
    }

    #[test]
    fn gpio_unsupported_hints_at_dev_feature() {
        let unsupported = Response {
            seq: 1,
            body: Some(Body::Error(badge_proto::Error { code: ErrorCode::Unsupported, detail: Default::default() })),
        };
        let hint_text = hint(&parse(&["gpio", "read", "5"]), &unsupported).expect("hint");
        assert!(hint_text.contains("--features dev"), "{hint_text}");
        assert!(hint(&parse(&["gpio", "write", "5", "1"]), &unsupported).is_some());
        assert_eq!(hint(&parse(&["info"]), &unsupported), None);
        let other = Response {
            seq: 1,
            body: Some(Body::Error(badge_proto::Error { code: ErrorCode::PinNotAllowed, detail: Default::default() })),
        };
        assert_eq!(hint(&parse(&["gpio", "read", "5"]), &other), None);
        let ok = Response { seq: 1, body: Some(Body::Ack(badge_proto::Ack {})) };
        assert_eq!(hint(&parse(&["gpio", "read", "5"]), &ok), None);
    }

    #[test]
    fn poke_trade_and_chi_words() {
        let peer = crate::cli::parse_peer_id("0AC8F6").unwrap();
        assert_eq!(poke_words(&PokeCmd::TradeInvite { peer }), (mailbox::TRADE_INVITE, 0x0A_C8F6, 0));
        assert_eq!(poke_words(&PokeCmd::Chi { chi: ChiArg::Ovr }), (mailbox::SET_CHI, 6, 0));
        let (c, a0, a1) = poke_words(&PokeCmd::TradeInvite { peer });
        let Some(Cmd::TradeInvite(t)) = mailbox_to_host_msg(c, a0, a1).and_then(|m| m.cmd) else {
            panic!("firmware maps trade invite")
        };
        assert_eq!(t.peer.as_str(), "0AC8F6");
    }

    #[test]
    fn next_seq_wraps_within_range() {
        assert_eq!(next_seq(1), 2);
        assert_eq!(next_seq(0x7fff_fffe), 0x7fff_ffff);
        assert_eq!(next_seq(0x7fff_ffff), 1);
        assert_eq!(next_seq(0), 1);
        assert_eq!(next_seq(u32::MAX), 1);
    }

    #[test]
    fn seq_is_nonzero_and_never_mailbox() {
        for e in [0u64, 1, u64::MAX, 0xffff_ffff, 0x8000_0000_0000_0000, 0x1234_5678_9abc_def0] {
            let s = initial_seq(e);
            assert_ne!(s, 0);
            assert_ne!(s, MAILBOX_SEQ);
            assert!(s < 0x8000_0000);
        }
    }
}
