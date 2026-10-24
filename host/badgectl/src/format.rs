//! Human-readable and JSON rendering of badge messages.
//!
//! Everything returns a `String` or a `serde_json::Value`; the shell decides
//! where to print it. The generated protocol types do not implement
//! `Serialize`, so the JSON shape is built by hand here and is the stable
//! machine interface of `badgectl --json`.

use std::fmt::Write as _;

use badge_proto::{
    AccessPoint, BleDevice, BleMetrics, Button, ButtonState, Chi, Contact, ErrorCode, Event, Event_, Info,
    LedPattern, Lobby, PetMode, Response, Response_::Body, Screen, SyncState, TradeState, Via, WifiState,
};
use serde_json::{Value, json};

/// Name of a `Button` value.
pub fn button_name(b: Button) -> String {
    match b {
        Button::Boot => "BOOT".into(),
        Button::Left => "LEFT".into(),
        Button::Select => "SELECT".into(),
        Button::Start => "START".into(),
        Button::Right => "RIGHT".into(),
        Button::Unspecified => "UNSPECIFIED".into(),
        Button(n) => format!("BUTTON({n})"),
    }
}

/// Name of a `Screen` value.
pub fn screen_name(s: Screen) -> String {
    match s {
        Screen::Home => "HOME".into(),
        Screen::Buttons => "BUTTONS".into(),
        Screen::Leds => "LEDS".into(),
        Screen::Wifi => "WIFI".into(),
        Screen::Ble => "BLE".into(),
        Screen::Gpio => "GPIO".into(),
        Screen::Host => "HOST".into(),
        Screen::Pet => "PET".into(),
        Screen::Trade => "TRADE".into(),
        Screen::Contacts => "CONTACTS".into(),
        Screen(n) => format!("SCREEN({n})"),
    }
}

/// Name of a `LedPattern` value.
pub fn pattern_name(p: LedPattern) -> String {
    match p {
        LedPattern::Off => "OFF".into(),
        LedPattern::Manual => "MANUAL".into(),
        LedPattern::Rainbow => "RAINBOW".into(),
        LedPattern::Breathe => "BREATHE".into(),
        LedPattern(n) => format!("PATTERN({n})"),
    }
}

/// Name of a `Chi` value: the three-letter on-air tag.
pub fn chi_name(c: Chi) -> String {
    match c {
        Chi::Unspecified => "-".into(),
        Chi::Vib => "VIB".into(),
        Chi::Hck => "HCK".into(),
        Chi::Pkt => "PKT".into(),
        Chi::Adm => "ADM".into(),
        Chi::Cry => "CRY".into(),
        Chi::Ovr => "OVR".into(),
        Chi(n) => format!("CHI({n})"),
    }
}

/// Name of a `TradeState` value.
pub fn trade_state_name(t: TradeState) -> String {
    match t {
        TradeState::Idle => "IDLE".into(),
        TradeState::Inviting => "INVITING".into(),
        TradeState::Exchanging => "EXCHANGING".into(),
        TradeState::Complete => "COMPLETE".into(),
        TradeState::Lost => "LOST".into(),
        TradeState::Cancelled => "CANCELLED".into(),
        TradeState(n) => format!("TRADE_STATE({n})"),
    }
}

/// Name of a `Via` value.
pub fn via_name(v: Via) -> String {
    match v {
        Via::Unspecified => "-".into(),
        Via::Ble => "BLE".into(),
        Via::Link => "LINK".into(),
        Via(n) => format!("VIA({n})"),
    }
}

/// A peer id string, or `-` when empty.
fn id_or_dash(id: &str) -> &str {
    if id.is_empty() { "-" } else { id }
}

/// Name of a `PetMode` value.
pub fn pet_mode_name(m: PetMode) -> String {
    match m {
        PetMode::Auto => "AUTO".into(),
        PetMode::Idle => "IDLE".into(),
        PetMode::Walk => "WALK".into(),
        PetMode::Eat => "EAT".into(),
        PetMode::Play => "PLAY".into(),
        PetMode::Sleep => "SLEEP".into(),
        PetMode(n) => format!("PET_MODE({n})"),
    }
}

/// Name of an `ErrorCode` value.
pub fn error_code_name(c: ErrorCode) -> String {
    match c {
        ErrorCode::None => "NONE".into(),
        ErrorCode::Decode => "DECODE".into(),
        ErrorCode::BadArgument => "BAD_ARGUMENT".into(),
        ErrorCode::PinNotAllowed => "PIN_NOT_ALLOWED".into(),
        ErrorCode::Busy => "BUSY".into(),
        ErrorCode::Radio => "RADIO".into(),
        ErrorCode::Unsupported => "UNSUPPORTED".into(),
        ErrorCode(n) => format!("ERROR({n})"),
    }
}

/// Name of an esp-radio `AuthMethod` discriminant.
pub fn auth_name(auth: u32) -> String {
    const NAMES: [&str; 9] = [
        "open", "WEP", "WPA", "WPA2", "WPA/WPA2", "WPA2-ENT", "WPA3", "WPA2/WPA3", "WAPI",
    ];
    usize::try_from(auth)
        .ok()
        .and_then(|i| NAMES.get(i))
        .map_or_else(|| format!("auth({auth})"), |s| (*s).to_string())
}

/// `aa:bb:cc:dd:ee:ff` in the given byte order.
pub fn mac(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(":")
}

/// BLE address bytes arrive little-endian (as on air); print them the
/// conventional most-significant-first way.
pub fn ble_addr(bytes: &[u8]) -> String {
    let mut rev = bytes.to_vec();
    rev.reverse();
    mac(&rev)
}

/// Dotted IPv4, or `-` when there is no lease.
pub fn ipv4(bytes: &[u8]) -> String {
    match bytes {
        [a, b, c, d] => format!("{a}.{b}.{c}.{d}"),
        _ => "-".into(),
    }
}

/// `1h02m03.456s`-style uptime.
pub fn uptime(ms: u64) -> String {
    let (h, rem) = (ms / 3_600_000, ms % 3_600_000);
    let (m, rem) = (rem / 60_000, rem % 60_000);
    let (s, milli) = (rem / 1000, rem % 1000);
    if h > 0 {
        format!("{h}h{m:02}m{s:02}.{milli:03}s")
    } else if m > 0 {
        format!("{m}m{s:02}.{milli:03}s")
    } else {
        format!("{s}.{milli:03}s")
    }
}

/// Names of the buttons set in a pressed mask, or `none`.
pub fn pressed_names(mask: u32) -> String {
    let names: Vec<String> = (1..32)
        .filter(|bit| mask & (1 << bit) != 0)
        .map(|bit| button_name(Button(bit)))
        .collect();
    if names.is_empty() { "none".into() } else { names.join(", ") }
}

/// True if the response carries an `Error` body.
pub fn is_error(r: &Response) -> bool {
    matches!(r.body, Some(Body::Error(_)))
}

fn info_text(i: &Info) -> String {
    let pins: Vec<String> = i.probe_pins.iter().map(u32::to_string).collect();
    format!(
        "firmware:   {}\nchip:       {}\nmac:        {}\nuptime:     {} ({} ms)\nfree heap:  {} bytes\nprobe pins: {}",
        i.firmware,
        i.chip,
        mac(&i.mac),
        uptime(i.uptime_ms),
        i.uptime_ms,
        i.free_heap,
        pins.join(" ")
    )
}

fn buttons_text(b: &ButtonState) -> String {
    let mut out = format!("pressed: {}\n", pressed_names(b.pressed_mask));
    out.push_str("button   presses\n");
    for (i, count) in b.press_counts.iter().enumerate().skip(1) {
        let name = button_name(Button(i32::try_from(i).unwrap_or(i32::MAX)));
        let _ = writeln!(out, "{name:<8} {count}");
    }
    out.trim_end().to_string()
}

fn wifi_scan_text(aps: &[AccessPoint]) -> String {
    if aps.is_empty() {
        return "no access points found".into();
    }
    let width = aps.iter().map(|a| a.ssid.len()).max().unwrap_or(0).max(4);
    let mut out = format!("{:<width$}  {:<17}  {:>4}  {:>3}  AUTH\n", "SSID", "BSSID", "RSSI", "CH");
    for a in aps {
        let ssid = if a.ssid.is_empty() { "<hidden>" } else { a.ssid.as_str() };
        let _ = writeln!(
            out,
            "{ssid:<width$}  {:<17}  {:>4}  {:>3}  {}",
            mac(&a.bssid),
            a.rssi,
            a.channel,
            auth_name(a.auth)
        );
    }
    let _ = write!(out, "{} access point(s)", aps.len());
    out
}

fn wifi_state_text(w: &WifiState) -> String {
    if w.connected {
        format!("connected to \"{}\" ip {} rssi {} dBm", w.ssid, ipv4(&w.ipv4), w.rssi)
    } else if w.ssid.is_empty() {
        "not connected".into()
    } else {
        format!("not connected (last ssid \"{}\")", w.ssid)
    }
}

fn ble_scan_text(devices: &[BleDevice]) -> String {
    if devices.is_empty() {
        return "no advertisers heard".into();
    }
    let mut out = format!("{:<17}  {:>4}  {:>4}  NAME\n", "ADDR", "KIND", "RSSI");
    for d in devices {
        let _ = writeln!(out, "{:<17}  {:>4}  {:>4}  {}", ble_addr(&d.addr), d.addr_kind, d.rssi, d.name);
    }
    let _ = write!(out, "{} device(s)", devices.len());
    out
}

/// Table of contacts; `offset` is the index of the first one and `total`
/// the number stored on the badge.
pub fn contacts_text(contacts: &[Contact], offset: u32, total: u32) -> String {
    if contacts.is_empty() {
        return if total == 0 {
            "no contacts".into()
        } else {
            format!("no contacts at offset {offset} ({total} stored)")
        };
    }
    let mut out = format!("{:>3}  {:<6}  {:<3}  {:>6}  NAME\n", "#", "ID", "CHI", "TRADES");
    for (i, c) in contacts.iter().enumerate() {
        let index = u64::from(offset) + i as u64;
        let _ = writeln!(
            out,
            "{index:>3}  {:<6}  {:<3}  {:>6}  {}",
            c.id.as_str(),
            chi_name(c.chi),
            c.trades,
            c.name.as_str()
        );
    }
    let first = u64::from(offset) + 1;
    let last = u64::from(offset) + contacts.len() as u64;
    let _ = write!(out, "contacts {first}..{last} of {total}");
    out
}

/// JSON object for one contact.
pub fn contact_json(c: &Contact) -> Value {
    json!({
        "id": c.id.as_str(), "chi": chi_name(c.chi), "chi_value": c.chi.0,
        "name": c.name.as_str(), "trades": c.trades,
    })
}

/// JSON body for a list of contacts (one page or a whole `--all` walk).
pub fn contacts_json(contacts: &[Contact], offset: u32, total: u32) -> Value {
    json!({
        "type": "contacts", "offset": offset, "total": total,
        "contacts": contacts.iter().map(contact_json).collect::<Vec<_>>(),
    })
}

fn ble_metrics_text(m: &BleMetrics) -> String {
    let strongest = if m.strongest_rssi == 0 { "-".to_string() } else { format!("{} dBm", m.strongest_rssi) };
    format!(
        "adv/s:          {}\nunique (10 s):  {}\nOZ badges:      {}\nstrongest:      {strongest}\nlobby peers:    {}",
        m.adv_per_s, m.unique_10s, m.oz_badges, m.lobby_peers
    )
}

fn sync_text(s: &SyncState) -> String {
    let role = if s.leader_id == s.self_id { " (us)" } else { "" };
    format!(
        "self id:  {:08x}\nleader:   {:08x}{role}\noffset:   {:+} ms\npeers:    {}",
        s.self_id, s.leader_id, s.offset_ms, s.peers
    )
}

fn lobby_text(l: &Lobby) -> String {
    let mut out = format!(
        "offering {} ({})\ntrade    {} {}\n",
        chi_name(l.chi),
        if l.signed { "signed" } else { "unsigned: no radio key" },
        trade_state_name(l.state),
        id_or_dash(&l.peer)
    );
    if l.peers.is_empty() {
        out.push_str("no peers in the lobby");
        return out;
    }
    let _ = writeln!(out, "{:<6}  {:>4}  {:<4}  CHI", "ID", "RSSI", "VIA");
    for p in &l.peers {
        let _ = writeln!(out, "{:<6}  {:>4}  {:<4}  {}", p.id.as_str(), p.rssi, via_name(p.via), chi_name(p.chi));
    }
    let _ = write!(out, "{} peer(s)", l.peers.len());
    out
}

/// Human-readable rendering of a response body.
pub fn response_text(r: &Response) -> String {
    match &r.body {
        None => format!("empty response (seq {})", r.seq),
        Some(Body::Ack(_)) => "ok".into(),
        Some(Body::Error(e)) if e.detail.is_empty() => format!("error: {}", error_code_name(e.code)),
        Some(Body::Error(e)) => format!("error: {}: {}", error_code_name(e.code), e.detail),
        Some(Body::Pong(p)) => format!("pong nonce={} uptime={} ({} ms)", p.nonce, uptime(p.uptime_ms), p.uptime_ms),
        Some(Body::Info(i)) => info_text(i),
        Some(Body::Buttons(b)) => buttons_text(b),
        Some(Body::Gpio(g)) => {
            format!("GPIO{} = {} ({})", g.pin, u8::from(g.level), if g.level { "high" } else { "low" })
        }
        Some(Body::WifiScan(s)) => wifi_scan_text(&s.aps),
        Some(Body::WifiState(w)) => wifi_state_text(w),
        Some(Body::BleScan(s)) => ble_scan_text(&s.devices),
        Some(Body::Contacts(c)) => contacts_text(&c.contacts, c.offset, c.total),
        Some(Body::BleMetrics(m)) => ble_metrics_text(m),
        Some(Body::Sync(s)) => sync_text(s),
        Some(Body::Lobby(l)) => lobby_text(l),
    }
}

/// Human-readable rendering of an event.
pub fn event_text(e: &Event) -> String {
    match &e.kind {
        Some(Event_::Kind::Button(b)) => format!(
            "event: button {} {} at {} ms",
            button_name(b.button),
            if b.pressed { "pressed" } else { "released" },
            b.at_ms
        ),
        Some(Event_::Kind::Screen(s)) => format!("event: screen -> {}", screen_name(s.screen)),
        Some(Event_::Kind::Pet(p)) => format!(
            "event: pet selected={} active={}",
            pet_mode_name(p.selected),
            pet_mode_name(p.active)
        ),
        Some(Event_::Kind::Trade(t)) if t.state == TradeState::Complete => format!(
            "event: trade {} {} received {}",
            id_or_dash(&t.peer),
            trade_state_name(t.state),
            chi_name(t.chi)
        ),
        Some(Event_::Kind::Trade(t)) => {
            format!("event: trade {} {}", id_or_dash(&t.peer), trade_state_name(t.state))
        }
        Some(Event_::Kind::ContactAdded(c)) => match &c.contact {
            Some(c) if c.name.is_empty() => {
                format!("event: contact added {} {} ({} trades)", c.id.as_str(), chi_name(c.chi), c.trades)
            }
            Some(c) => format!(
                "event: contact added {} \"{}\" {} ({} trades)",
                c.id.as_str(),
                c.name.as_str(),
                chi_name(c.chi),
                c.trades
            ),
            None => "event: contact added (empty)".into(),
        },
        None => "event: (empty)".into(),
    }
}

/// JSON rendering of a response.
pub fn response_json(r: &Response) -> Value {
    let body = match &r.body {
        None => json!({ "type": "empty" }),
        Some(Body::Ack(_)) => json!({ "type": "ack" }),
        Some(Body::Error(e)) => json!({
            "type": "error", "code": error_code_name(e.code), "code_value": e.code.0, "detail": e.detail.as_str(),
        }),
        Some(Body::Pong(p)) => json!({ "type": "pong", "nonce": p.nonce, "uptime_ms": p.uptime_ms }),
        Some(Body::Info(i)) => json!({
            "type": "info", "firmware": i.firmware.as_str(), "chip": i.chip.as_str(), "mac": mac(&i.mac),
            "uptime_ms": i.uptime_ms, "free_heap": i.free_heap, "probe_pins": i.probe_pins.as_slice(),
        }),
        Some(Body::Buttons(b)) => json!({
            "type": "buttons", "pressed_mask": b.pressed_mask,
            "pressed": (1..32).filter(|bit| b.pressed_mask & (1 << bit) != 0)
                .map(|bit| button_name(Button(bit))).collect::<Vec<_>>(),
            "press_counts": b.press_counts.as_slice(),
        }),
        Some(Body::Gpio(g)) => json!({ "type": "gpio", "pin": g.pin, "level": g.level }),
        Some(Body::WifiScan(s)) => json!({
            "type": "wifi_scan",
            "aps": s.aps.iter().map(|a| json!({
                "ssid": a.ssid.as_str(), "bssid": mac(&a.bssid), "rssi": a.rssi,
                "channel": a.channel, "auth": a.auth, "auth_name": auth_name(a.auth),
            })).collect::<Vec<_>>(),
        }),
        Some(Body::WifiState(w)) => json!({
            "type": "wifi_state", "connected": w.connected, "ssid": w.ssid.as_str(),
            "ipv4": if w.ipv4.len() == 4 { Value::from(ipv4(&w.ipv4)) } else { Value::Null },
            "rssi": w.rssi,
        }),
        Some(Body::BleScan(s)) => json!({
            "type": "ble_scan",
            "devices": s.devices.iter().map(|d| json!({
                "addr": ble_addr(&d.addr), "addr_kind": d.addr_kind, "rssi": d.rssi, "name": d.name.as_str(),
            })).collect::<Vec<_>>(),
        }),
        Some(Body::Contacts(c)) => contacts_json(&c.contacts, c.offset, c.total),
        Some(Body::BleMetrics(m)) => json!({
            "type": "ble_metrics", "adv_per_s": m.adv_per_s, "unique_10s": m.unique_10s,
            "oz_badges": m.oz_badges, "strongest_rssi": m.strongest_rssi, "lobby_peers": m.lobby_peers,
        }),
        Some(Body::Sync(s)) => json!({
            "type": "sync", "self_id": s.self_id, "leader_id": s.leader_id,
            "leader_is_self": s.leader_id == s.self_id, "offset_ms": s.offset_ms, "peers": s.peers,
        }),
        Some(Body::Lobby(l)) => json!({
            "type": "lobby", "chi": chi_name(l.chi), "chi_value": l.chi.0, "signed": l.signed,
            "state": trade_state_name(l.state), "state_value": l.state.0,
            "peer": if l.peer.is_empty() { Value::Null } else { Value::from(l.peer.as_str()) },
            "peers": l.peers.iter().map(|p| json!({
                "id": p.id.as_str(), "rssi": p.rssi, "via": via_name(p.via), "via_value": p.via.0,
                "chi": chi_name(p.chi), "chi_value": p.chi.0,
            })).collect::<Vec<_>>(),
        }),
    };
    json!({ "kind": "response", "seq": r.seq, "body": body })
}

/// JSON rendering of an event.
pub fn event_json(e: &Event) -> Value {
    let body = match &e.kind {
        Some(Event_::Kind::Button(b)) => json!({
            "type": "button", "button": button_name(b.button), "button_value": b.button.0,
            "pressed": b.pressed, "at_ms": b.at_ms,
        }),
        Some(Event_::Kind::Screen(s)) => json!({
            "type": "screen", "screen": screen_name(s.screen), "screen_value": s.screen.0,
        }),
        Some(Event_::Kind::Pet(p)) => json!({
            "type": "pet", "selected": pet_mode_name(p.selected), "selected_value": p.selected.0,
            "active": pet_mode_name(p.active), "active_value": p.active.0,
        }),
        Some(Event_::Kind::Trade(t)) => json!({
            "type": "trade", "peer": t.peer.as_str(), "state": trade_state_name(t.state),
            "state_value": t.state.0, "chi": chi_name(t.chi), "chi_value": t.chi.0,
        }),
        Some(Event_::Kind::ContactAdded(c)) => json!({
            "type": "contact_added",
            "contact": c.contact.as_ref().map_or(Value::Null, contact_json),
        }),
        None => json!({ "type": "empty" }),
    };
    json!({ "kind": "event", "body": body })
}

/// Report for `badgectl addr`: symbol addresses plus stock probe-rs command
/// lines that read the PROBE block and poke its mailbox.
pub fn addr_report(chip: &str, probe: Option<u64>, rtt: Option<u64>) -> String {
    use badge_core::probe_layout::{MAILBOX_ARG0, MAILBOX_ARG1, MAILBOX_CMD, WORDS, mailbox};
    let mut out = String::new();
    let show = |a: Option<u64>| a.map_or_else(|| "not found".to_string(), |a| format!("{a:#010x}"));
    let _ = writeln!(out, "PROBE        {}", show(probe));
    let _ = writeln!(out, "_SEGGER_RTT  {}", show(rtt));
    if let Some(base) = probe {
        let word = |i: usize| base + 4 * i as u64;
        let _ = writeln!(out, "\nread the block:");
        let _ = writeln!(out, "  probe-rs read --chip {chip} b32 {base:#010x} {WORDS}");
        let _ = writeln!(out, "set LED <i> to 0x00RRGGBB (args first, then the command word):");
        let _ = writeln!(
            out,
            "  probe-rs write --chip {chip} b32 {:#010x} <i> <0xRRGGBB>",
            word(MAILBOX_ARG0)
        );
        let _ = writeln!(out, "  probe-rs write --chip {chip} b32 {:#010x} {}", word(MAILBOX_CMD), mailbox::SET_LED);
        let _ = writeln!(out, "show screen <n>:");
        let _ = writeln!(out, "  probe-rs write --chip {chip} b32 {:#010x} <n>", word(MAILBOX_ARG0));
        let _ = writeln!(out, "  probe-rs write --chip {chip} b32 {:#010x} {}", word(MAILBOX_CMD), mailbox::SHOW_SCREEN);
        let _ = writeln!(out, "set pattern <p> brightness <b>:");
        let _ = writeln!(out, "  probe-rs write --chip {chip} b32 {:#010x} <p>", word(MAILBOX_ARG0));
        let _ = writeln!(out, "  probe-rs write --chip {chip} b32 {:#010x} <b>", word(MAILBOX_ARG1));
        let _ = writeln!(out, "  probe-rs write --chip {chip} b32 {:#010x} {}", word(MAILBOX_CMD), mailbox::SET_PATTERN);
        let _ = writeln!(out, "set pet mode <m> (0 auto, 1 idle, 2 walk, 3 eat, 4 play, 5 sleep):");
        let _ = writeln!(out, "  probe-rs write --chip {chip} b32 {:#010x} <m>", word(MAILBOX_ARG0));
        let _ = writeln!(out, "  probe-rs write --chip {chip} b32 {:#010x} {}", word(MAILBOX_CMD), mailbox::SET_PET_MODE);
        let _ = writeln!(out, "invite peer <id> to a trade (id as hex, e.g. 0x0AC8F6):");
        let _ = writeln!(out, "  probe-rs write --chip {chip} b32 {:#010x} <0xID>", word(MAILBOX_ARG0));
        let _ = writeln!(out, "  probe-rs write --chip {chip} b32 {:#010x} {}", word(MAILBOX_CMD), mailbox::TRADE_INVITE);
        let _ = writeln!(out, "offer chi <c> (1 VIB, 2 HCK, 3 PKT, 4 ADM, 5 CRY, 6 OVR):");
        let _ = writeln!(out, "  probe-rs write --chip {chip} b32 {:#010x} <c>", word(MAILBOX_ARG0));
        let _ = writeln!(out, "  probe-rs write --chip {chip} b32 {:#010x} {}", word(MAILBOX_CMD), mailbox::SET_CHI);
    }
    out.trim_end().to_string()
}

#[cfg(test)]
mod tests {
    use super::*;
    use badge_proto::{
        Ack, BleScanResult, ButtonEvent, Error, GpioLevel, PetChanged, Pong, ScreenChanged, WifiScanResult,
    };

    fn hs<const N: usize>(s: &str) -> heapless::String<N> {
        let mut out = heapless::String::new();
        out.push_str(s).unwrap();
        out
    }

    fn hv<T: Clone, const N: usize>(s: &[T]) -> heapless::Vec<T, N> {
        heapless::Vec::from_slice(s).unwrap()
    }

    fn resp(body: Body) -> Response {
        Response { seq: 5, body: Some(body) }
    }

    #[test]
    fn names_cover_known_and_unknown() {
        assert_eq!(button_name(Button::Select), "SELECT");
        assert_eq!(button_name(Button(9)), "BUTTON(9)");
        assert_eq!(screen_name(Screen::Host), "HOST");
        assert_eq!(screen_name(Screen::Pet), "PET");
        assert_eq!(screen_name(Screen::Trade), "TRADE");
        assert_eq!(screen_name(Screen::Contacts), "CONTACTS");
        assert_eq!(screen_name(Screen(10)), "SCREEN(10)");
        assert_eq!(pet_mode_name(PetMode::Auto), "AUTO");
        assert_eq!(pet_mode_name(PetMode::Idle), "IDLE");
        assert_eq!(pet_mode_name(PetMode::Walk), "WALK");
        assert_eq!(pet_mode_name(PetMode::Eat), "EAT");
        assert_eq!(pet_mode_name(PetMode::Play), "PLAY");
        assert_eq!(pet_mode_name(PetMode::Sleep), "SLEEP");
        assert_eq!(pet_mode_name(PetMode(9)), "PET_MODE(9)");
        assert_eq!(pattern_name(LedPattern::Breathe), "BREATHE");
        assert_eq!(pattern_name(LedPattern(4)), "PATTERN(4)");
        assert_eq!(pattern_name(LedPattern(5)), "PATTERN(5)");
        assert_eq!(chi_name(Chi::Vib), "VIB");
        assert_eq!(chi_name(Chi::Ovr), "OVR");
        assert_eq!(chi_name(Chi::Unspecified), "-");
        assert_eq!(chi_name(Chi(7)), "CHI(7)");
        assert_eq!(trade_state_name(TradeState::Exchanging), "EXCHANGING");
        assert_eq!(trade_state_name(TradeState::Cancelled), "CANCELLED");
        assert_eq!(trade_state_name(TradeState(9)), "TRADE_STATE(9)");
        assert_eq!(via_name(Via::Link), "LINK");
        assert_eq!(via_name(Via::Ble), "BLE");
        assert_eq!(via_name(Via(3)), "VIA(3)");
        assert_eq!(error_code_name(ErrorCode::PinNotAllowed), "PIN_NOT_ALLOWED");
        assert_eq!(auth_name(0), "open");
        assert_eq!(auth_name(3), "WPA2");
        assert_eq!(auth_name(99), "auth(99)");
    }

    #[test]
    fn small_formatters() {
        assert_eq!(mac(&[0xde, 0xad, 0xbe, 0xef, 0, 1]), "de:ad:be:ef:00:01");
        assert_eq!(ble_addr(&[1, 2, 3, 4, 5, 6]), "06:05:04:03:02:01");
        assert_eq!(ipv4(&[192, 168, 1, 20]), "192.168.1.20");
        assert_eq!(ipv4(&[]), "-");
        assert_eq!(uptime(1_234), "1.234s");
        assert_eq!(uptime(61_005), "1m01.005s");
        assert_eq!(uptime(3_723_456), "1h02m03.456s");
        assert_eq!(pressed_names(0), "none");
        assert_eq!(pressed_names((1 << 2) | (1 << 4)), "LEFT, START");
    }

    #[test]
    fn simple_responses() {
        assert_eq!(response_text(&resp(Body::Ack(Ack {}))), "ok");
        assert_eq!(
            response_text(&resp(Body::Error(Error { code: ErrorCode::Busy, detail: hs("scan running") }))),
            "error: BUSY: scan running"
        );
        assert!(is_error(&resp(Body::Error(Error::default()))));
        assert!(!is_error(&resp(Body::Ack(Ack {}))));
        assert_eq!(
            response_text(&resp(Body::Pong(Pong { nonce: 7, uptime_ms: 1500 }))),
            "pong nonce=7 uptime=1.500s (1500 ms)"
        );
        assert_eq!(response_text(&resp(Body::Gpio(GpioLevel { pin: 5, level: true }))), "GPIO5 = 1 (high)");
        assert_eq!(response_text(&Response { seq: 3, body: None }), "empty response (seq 3)");
    }

    #[test]
    fn info_and_buttons() {
        let info = Info {
            firmware: hs("conagotchi-fw 0.1.0"),
            chip: hs("esp32s3"),
            mac: hv(&[1, 2, 3, 4, 5, 6]),
            uptime_ms: 2000,
            free_heap: 1234,
            probe_pins: hv(&[1, 3, 5]),
        };
        let text = response_text(&resp(Body::Info(info)));
        assert!(text.contains("conagotchi-fw 0.1.0"));
        assert!(text.contains("01:02:03:04:05:06"));
        assert!(text.contains("probe pins: 1 3 5"));
        let b = ButtonState { pressed_mask: 1 << 3, press_counts: hv(&[0, 1, 2, 3, 4, 5]) };
        let text = response_text(&resp(Body::Buttons(b)));
        assert!(text.starts_with("pressed: SELECT\n"));
        assert!(text.contains("RIGHT    5"));
        assert!(!text.contains("UNSPECIFIED"));
    }

    #[test]
    fn wifi_tables() {
        let aps = [
            AccessPoint { ssid: hs("lab"), bssid: hv(&[0xaa; 6]), rssi: -40, channel: 6, auth: 3 },
            AccessPoint { ssid: hs(""), bssid: hv(&[1; 6]), rssi: -88, channel: 11, auth: 0 },
        ];
        let text = response_text(&resp(Body::WifiScan(WifiScanResult { aps: hv(&aps) })));
        let lines: Vec<&str> = text.lines().collect();
        assert!(lines[0].starts_with("SSID"));
        assert!(lines[0].contains("BSSID") && lines[0].contains("RSSI") && lines[0].ends_with("AUTH"));
        assert!(lines[1].contains("aa:aa:aa:aa:aa:aa") && lines[1].contains("-40") && lines[1].ends_with("WPA2"));
        assert!(lines[2].starts_with("<hidden>") && lines[2].ends_with("open"));
        assert_eq!(lines[3], "2 access point(s)");
        let empty = response_text(&resp(Body::WifiScan(WifiScanResult::default())));
        assert_eq!(empty, "no access points found");
        let up = WifiState { connected: true, ssid: hs("lab"), ipv4: hv(&[10, 0, 0, 2]), rssi: -50 };
        assert_eq!(response_text(&resp(Body::WifiState(up))), "connected to \"lab\" ip 10.0.0.2 rssi -50 dBm");
        assert_eq!(response_text(&resp(Body::WifiState(WifiState::default()))), "not connected");
    }

    #[test]
    fn ble_table() {
        let d = BleDevice { addr: hv(&[6, 5, 4, 3, 2, 1]), addr_kind: 1, rssi: -70, name: hs("tag") };
        let text = response_text(&resp(Body::BleScan(BleScanResult { devices: hv(&[d]) })));
        let lines: Vec<&str> = text.lines().collect();
        assert!(lines[0].starts_with("ADDR") && lines[0].ends_with("NAME"));
        assert!(lines[1].starts_with("01:02:03:04:05:06") && lines[1].ends_with("tag"));
        assert_eq!(lines[2], "1 device(s)");
    }

    #[test]
    fn events() {
        let press = Event {
            kind: Some(Event_::Kind::Button(ButtonEvent { button: Button::Left, pressed: true, at_ms: 99 })),
        };
        assert_eq!(event_text(&press), "event: button LEFT pressed at 99 ms");
        let screen = Event { kind: Some(Event_::Kind::Screen(ScreenChanged { screen: Screen::Ble })) };
        assert_eq!(event_text(&screen), "event: screen -> BLE");
        let j = event_json(&press);
        assert_eq!(j["kind"], "event");
        assert_eq!(j["body"]["button"], "LEFT");
        assert_eq!(j["body"]["pressed"], true);
        assert_eq!(event_json(&screen)["body"]["screen_value"], 4);
    }

    #[test]
    fn pet_events() {
        let pet = Event {
            kind: Some(Event_::Kind::Pet(PetChanged { selected: PetMode::Auto, active: PetMode::Walk })),
        };
        assert_eq!(event_text(&pet), "event: pet selected=AUTO active=WALK");
        let j = event_json(&pet);
        assert_eq!(j["kind"], "event");
        assert_eq!(j["body"]["type"], "pet");
        assert_eq!(j["body"]["selected"], "AUTO");
        assert_eq!(j["body"]["selected_value"], 0);
        assert_eq!(j["body"]["active"], "WALK");
        assert_eq!(j["body"]["active_value"], 2);
        let screen = Event { kind: Some(Event_::Kind::Screen(ScreenChanged { screen: Screen::Pet })) };
        assert_eq!(event_text(&screen), "event: screen -> PET");
        assert_eq!(event_json(&screen)["body"]["screen"], "PET");
        assert_eq!(event_json(&screen)["body"]["screen_value"], 7);
    }

    #[test]
    fn json_responses() {
        let j = response_json(&resp(Body::Error(Error { code: ErrorCode::Radio, detail: hs("x") })));
        assert_eq!(j["seq"], 5);
        assert_eq!(j["body"]["type"], "error");
        assert_eq!(j["body"]["code"], "RADIO");
        let aps = [AccessPoint { ssid: hs("lab"), bssid: hv(&[0xaa; 6]), rssi: -40, channel: 6, auth: 3 }];
        let j = response_json(&resp(Body::WifiScan(WifiScanResult { aps: hv(&aps) })));
        assert_eq!(j["body"]["aps"][0]["ssid"], "lab");
        assert_eq!(j["body"]["aps"][0]["auth_name"], "WPA2");
        let j = response_json(&resp(Body::WifiState(WifiState::default())));
        assert_eq!(j["body"]["ipv4"], Value::Null);
        let b = ButtonState { pressed_mask: 1 << 1, press_counts: hv(&[0, 4]) };
        let j = response_json(&resp(Body::Buttons(b)));
        assert_eq!(j["body"]["pressed"][0], "BOOT");
        assert_eq!(j["body"]["press_counts"][1], 4);
    }

    fn contact(id: &str, chi: Chi, name: &str, trades: u32) -> Contact {
        Contact { id: hs(id), chi, name: hs(name), trades }
    }

    #[test]
    fn contacts_table_and_json() {
        let page = badge_proto::Contacts {
            contacts: hv(&[contact("0AC8F6", Chi::Hck, "ada", 2), contact("123ABC", Chi::Ovr, "", 1)]),
            total: 10,
            offset: 8,
        };
        let text = response_text(&resp(Body::Contacts(page.clone())));
        let lines: Vec<&str> = text.lines().collect();
        assert!(lines[0].contains("ID") && lines[0].contains("CHI") && lines[0].ends_with("NAME"), "{text}");
        assert!(lines[1].starts_with("  8  0AC8F6  HCK") && lines[1].ends_with("ada"), "{text}");
        assert!(lines[2].starts_with("  9  123ABC  OVR"), "{text}");
        assert_eq!(lines[3], "contacts 9..10 of 10");
        let j = response_json(&resp(Body::Contacts(page)));
        assert_eq!(j["body"]["type"], "contacts");
        assert_eq!(j["body"]["total"], 10);
        assert_eq!(j["body"]["offset"], 8);
        assert_eq!(j["body"]["contacts"][0]["id"], "0AC8F6");
        assert_eq!(j["body"]["contacts"][0]["chi"], "HCK");
        assert_eq!(j["body"]["contacts"][0]["chi_value"], 2);
        assert_eq!(j["body"]["contacts"][1]["name"], "");
        assert_eq!(j["body"]["contacts"][1]["trades"], 1);
        assert_eq!(contacts_text(&[], 0, 0), "no contacts");
        assert_eq!(contacts_text(&[], 16, 3), "no contacts at offset 16 (3 stored)");
        let all = contacts_json(&vec![contact("AAAAAA", Chi::Vib, "", 1); 9], 0, 9);
        assert_eq!(all["contacts"].as_array().map(Vec::len), Some(9));
    }

    #[test]
    fn ble_metrics_and_sync() {
        let m = BleMetrics { adv_per_s: 40, unique_10s: 12, oz_badges: 3, strongest_rssi: -51, lobby_peers: 2 };
        let text = response_text(&resp(Body::BleMetrics(m)));
        assert!(text.contains("adv/s:          40"), "{text}");
        assert!(text.contains("unique (10 s):  12"));
        assert!(text.contains("OZ badges:      3"));
        assert!(text.contains("strongest:      -51 dBm"));
        assert!(text.contains("lobby peers:    2"));
        let quiet = response_text(&resp(Body::BleMetrics(BleMetrics::default())));
        assert!(quiet.contains("strongest:      -"), "{quiet}");
        let j = response_json(&resp(Body::BleMetrics(m)));
        assert_eq!(j["body"]["type"], "ble_metrics");
        assert_eq!(j["body"]["strongest_rssi"], -51);
        assert_eq!(j["body"]["oz_badges"], 3);

        let s = SyncState { self_id: 0xc100_1122, leader_id: 0x0000_00aa, offset_ms: -1500, peers: 4 };
        let text = response_text(&resp(Body::Sync(s)));
        assert!(text.contains("self id:  c1001122"), "{text}");
        assert!(text.contains("leader:   000000aa\n"), "{text}");
        assert!(text.contains("offset:   -1500 ms"));
        assert!(text.contains("peers:    4"));
        let alone = SyncState { self_id: 7, leader_id: 7, offset_ms: 0, peers: 0 };
        let text = response_text(&resp(Body::Sync(alone)));
        assert!(text.contains("leader:   00000007 (us)") && text.contains("+0 ms"), "{text}");
        let j = response_json(&resp(Body::Sync(s)));
        assert_eq!(j["body"]["type"], "sync");
        assert_eq!(j["body"]["leader_is_self"], false);
        assert_eq!(j["body"]["offset_ms"], -1500);
    }

    #[test]
    fn lobby_table_and_json() {
        let peers = [
            badge_proto::LobbyPeer { id: hs("0AC8F6"), rssi: -48, via: Via::Ble, chi: Chi::Cry },
            badge_proto::LobbyPeer { id: hs("00BEEF"), rssi: 0, via: Via::Link, chi: Chi::Unspecified },
        ];
        let l = Lobby { peers: hv(&peers), chi: Chi::Vib, signed: true, state: TradeState::Inviting, peer: hs("0AC8F6") };
        let text = response_text(&resp(Body::Lobby(l.clone())));
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines[0], "offering VIB (signed)");
        assert_eq!(lines[1], "trade    INVITING 0AC8F6");
        assert!(lines[2].starts_with("ID") && lines[2].ends_with("CHI"));
        assert!(lines[3].starts_with("0AC8F6") && lines[3].contains("-48") && lines[3].contains("BLE") && lines[3].ends_with("CRY"));
        assert!(lines[4].starts_with("00BEEF") && lines[4].contains("LINK") && lines[4].ends_with('-'));
        assert_eq!(lines[5], "2 peer(s)");
        let j = response_json(&resp(Body::Lobby(l)));
        assert_eq!(j["body"]["type"], "lobby");
        assert_eq!(j["body"]["signed"], true);
        assert_eq!(j["body"]["state"], "INVITING");
        assert_eq!(j["body"]["peer"], "0AC8F6");
        assert_eq!(j["body"]["peers"][1]["via"], "LINK");
        assert_eq!(j["body"]["peers"][0]["chi_value"], 5);
        let empty = Lobby::default();
        let text = response_text(&resp(Body::Lobby(empty.clone())));
        assert!(text.contains("unsigned: no radio key") && text.contains("trade    IDLE -"), "{text}");
        assert!(text.ends_with("no peers in the lobby"));
        assert_eq!(response_json(&resp(Body::Lobby(empty)))["body"]["peer"], Value::Null);
    }

    #[test]
    fn trade_and_contact_events() {
        let done = Event {
            kind: Some(Event_::Kind::Trade(badge_proto::TradeEvent {
                peer: hs("0AC8F6"),
                state: TradeState::Complete,
                chi: Chi::Pkt,
            })),
        };
        assert_eq!(event_text(&done), "event: trade 0AC8F6 COMPLETE received PKT");
        let lost = Event {
            kind: Some(Event_::Kind::Trade(badge_proto::TradeEvent {
                peer: hs("0AC8F6"),
                state: TradeState::Lost,
                chi: Chi::Unspecified,
            })),
        };
        assert_eq!(event_text(&lost), "event: trade 0AC8F6 LOST");
        let j = event_json(&done);
        assert_eq!(j["body"]["type"], "trade");
        assert_eq!(j["body"]["state"], "COMPLETE");
        assert_eq!(j["body"]["state_value"], 3);
        assert_eq!(j["body"]["chi"], "PKT");
        let added = Event {
            kind: Some(Event_::Kind::ContactAdded(badge_proto::ContactAdded {
                contact: Some(contact("0AC8F6", Chi::Pkt, "ada", 1)),
            })),
        };
        assert_eq!(event_text(&added), "event: contact added 0AC8F6 \"ada\" PKT (1 trades)");
        let anon = Event {
            kind: Some(Event_::Kind::ContactAdded(badge_proto::ContactAdded {
                contact: Some(contact("0AC8F6", Chi::Pkt, "", 3)),
            })),
        };
        assert_eq!(event_text(&anon), "event: contact added 0AC8F6 PKT (3 trades)");
        let j = event_json(&added);
        assert_eq!(j["body"]["type"], "contact_added");
        assert_eq!(j["body"]["contact"]["name"], "ada");
        let none = Event { kind: Some(Event_::Kind::ContactAdded(badge_proto::ContactAdded { contact: None })) };
        assert_eq!(event_text(&none), "event: contact added (empty)");
        assert_eq!(event_json(&none)["body"]["contact"], Value::Null);
    }

    #[test]
    fn addr_report_has_stock_commands() {
        let text = addr_report("esp32s3", Some(0x3fc8_1000), Some(0x3fc8_2000));
        assert!(text.contains("PROBE        0x3fc81000"));
        assert!(text.contains("_SEGGER_RTT  0x3fc82000"));
        let words = badge_core::probe_layout::WORDS;
        assert!(text.contains(&format!("probe-rs read --chip esp32s3 b32 0x3fc81000 {words}\n")));
        // MAILBOX_CMD is word 24 -> +0x60; ARG0 word 25 -> +0x64.
        assert!(text.contains("b32 0x3fc81060 1"));
        assert!(text.contains("b32 0x3fc81064 <i> <0xRRGGBB>"));
        assert!(text.contains("set pet mode <m>"));
        assert!(text.contains("b32 0x3fc81064 <m>"));
        assert!(text.contains("b32 0x3fc81060 4"));
        assert!(text.contains("b32 0x3fc81060 5") && text.contains("<0xID>"));
        assert!(text.contains("b32 0x3fc81060 6") && text.contains("6 OVR"));
        let missing = addr_report("esp32s3", None, None);
        assert!(missing.contains("not found") && !missing.contains("probe-rs read"));
    }
}
