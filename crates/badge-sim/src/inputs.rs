//! Raw export arguments to core [`Input`]s.
//!
//! The C-ABI exports take only integers and a length into the exported input
//! region; these pure functions validate them and build the `Input` the
//! core expects. Out-of-range integers are clamped or rejected here, never
//! passed through as garbage.

#![forbid(unsafe_code)]

use badge_core::io::{ButtonMask, Input, Millis, Via, WASM_LOG_LEN};
use badge_core::oz;
use badge_proto::{ErrorCode, HostMsg, WasmStatus};

/// `via` argument value for a BLE advertisement.
pub const VIA_BLE: i32 = 1;
/// `via` argument value for the link UART.
pub const VIA_LINK: i32 = 2;

/// The badge MAC from the low 48 bits of `mac_lo` (most significant byte
/// first, so `0x0200_C0FF_EE01` is `02:00:c0:ff:ee:01`).
pub fn mac_from_i64(mac_lo: i64) -> [u8; 6] {
    let [_, _, a, b, c, d, e, f] = mac_lo.to_be_bytes();
    [a, b, c, d, e, f]
}

/// Milliseconds as the core's clock; negative values clamp to 0.
pub fn millis(ms: i64) -> Millis {
    Millis(u64::try_from(ms).unwrap_or(0))
}

/// A tick at `now_ms` with raw button levels (bit n = `Button(n)`; bits
/// above 7 are ignored).
pub fn tick(now_ms: i64, buttons: i32) -> Input {
    Input::Tick { now: millis(now_ms), buttons: ButtonMask((buttons & 0xFF) as u8) }
}

/// One encoded `HostMsg`, or `HostGarbage` if it does not decode.
pub fn host(bytes: &[u8]) -> Input {
    match badge_proto::decode::<HostMsg>(bytes) {
        Ok(msg) => Input::Host(msg),
        Err(_) => Input::HostGarbage,
    }
}

/// A stock-protocol packet; `None` for an unknown `via` or a payload larger
/// than [`oz::MAX_PAYLOAD`]. RSSI saturates to the `i8` range.
pub fn radio(via: i32, rssi: i32, payload: &[u8]) -> Option<Input> {
    let via = match via {
        VIA_BLE => Via::Ble,
        VIA_LINK => Via::Link,
        _ => return None,
    };
    let rssi = i8::try_from(rssi.clamp(i32::from(i8::MIN), i32::from(i8::MAX))).unwrap_or(i8::MIN);
    let payload = heapless::Vec::<u8, { oz::MAX_PAYLOAD }>::from_slice(payload).ok()?;
    Some(Input::RadioPacket { via, rssi, payload })
}

/// An LED sync beacon from `from`, received at local time `rx_at`.
pub fn sync(from: i32, network_ms: i32, rx_at: i64) -> Input {
    Input::SyncBeacon {
        from: u32::from_ne_bytes(from.to_ne_bytes()),
        network_ms: u32::from_ne_bytes(network_ms.to_ne_bytes()),
        rx_at: millis(rx_at),
    }
}

/// The runtime's answer to a wasm command: `code >= 0` means `bytes` is an
/// encoded `WasmStatus` (code is its length, already sliced by the caller);
/// `code < 0` is `-ErrorCode`. An undecodable status is a `DECODE` error.
pub fn wasm(seq: i32, code: i32, bytes: &[u8]) -> Input {
    let seq = u32::from_ne_bytes(seq.to_ne_bytes());
    let result = if code < 0 {
        Err(ErrorCode(code.saturating_neg()))
    } else {
        badge_proto::decode::<WasmStatus>(bytes).map_err(|_| ErrorCode::Decode)
    };
    Input::Wasm { seq, result }
}

/// A wasm app log line; invalid UTF-8 is replaced and long lines are
/// truncated on a character boundary to [`WASM_LOG_LEN`] bytes.
pub fn wasm_log(bytes: &[u8]) -> Input {
    let mut line = heapless::String::<WASM_LOG_LEN>::new();
    for chunk in bytes.utf8_chunks() {
        for ch in chunk.valid().chars().chain(chunk.invalid().first().map(|_| char::REPLACEMENT_CHARACTER)) {
            if line.push(ch).is_err() {
                return Input::WasmLog(line);
            }
        }
    }
    Input::WasmLog(line)
}

#[cfg(test)]
mod tests {
    extern crate std;

    use badge_proto::{HostMsg_::Cmd, Ping, WasmState};

    use super::*;

    #[test]
    fn fr39_mac_and_tick_arguments() {
        assert_eq!(mac_from_i64(0x0200_C0FF_EE01), [2, 0, 0xC0, 0xFF, 0xEE, 1]);
        assert_eq!(mac_from_i64(-1), [0xFF; 6]);
        assert_eq!(tick(-5, 0x1_0F), Input::Tick { now: Millis(0), buttons: ButtonMask(0x0F) });
    }

    #[test]
    fn fr39_host_bytes_decode_or_are_garbage() {
        let msg = HostMsg { seq: 3, cmd: Some(Cmd::Ping(Ping { nonce: 1 })) };
        let mut buf = [0u8; 64];
        let n = badge_proto::encode(&msg, &mut buf).unwrap();
        assert_eq!(host(&buf[..n]), Input::Host(msg));
        assert_eq!(host(&[0xFF, 0xFF, 0xFF]), Input::HostGarbage);
    }

    #[test]
    fn fr39_radio_and_sync_arguments() {
        let Some(Input::RadioPacket { via, rssi, payload }) = radio(VIA_BLE, -300, b"abc") else {
            panic!("radio")
        };
        assert_eq!((via, rssi, payload.as_slice()), (Via::Ble, -128, &b"abc"[..]));
        assert!(radio(9, 0, b"x").is_none());
        assert!(radio(VIA_LINK, 0, &[0; oz::MAX_PAYLOAD + 1]).is_none());
        assert_eq!(
            sync(-1, 7, 100),
            Input::SyncBeacon { from: u32::MAX, network_ms: 7, rx_at: Millis(100) }
        );
    }

    #[test]
    fn fr39_wasm_answers_and_logs() {
        let st = WasmStatus { state: WasmState::Running, size: 10, ..WasmStatus::default() };
        let mut buf = [0u8; 64];
        let n = badge_proto::encode(&st, &mut buf).unwrap();
        let n_i32 = i32::try_from(n).unwrap();
        assert_eq!(wasm(2, n_i32, &buf[..n]), Input::Wasm { seq: 2, result: Ok(st) });
        assert_eq!(wasm(2, -6, &[]), Input::Wasm { seq: 2, result: Err(ErrorCode::Unsupported) });
        assert_eq!(wasm(2, 3, &[0xFF, 0xFF, 0xFF]), Input::Wasm { seq: 2, result: Err(ErrorCode::Decode) });
        let Input::WasmLog(l) = wasm_log(b"hi \xFF there") else { panic!() };
        assert_eq!(l.as_str(), "hi \u{FFFD} there");
        let Input::WasmLog(l) = wasm_log("é".repeat(40).as_bytes()) else { panic!() };
        assert_eq!(l.len(), 64);
    }
}
