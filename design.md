# Conagotchi Discovery Firmware — Design

An alternative Rust firmware for the Conagotchi 2026 conference badge
(OzSec 2026, ESP32-S3-WROOM-1, 8 MB flash, no PSRAM). It turns the badge into
a playground: every peripheral can be exercised from the badge's own buttons
and screen, from protobuf messages sent by the host tool `badgectl` over
probe-rs RTT, or by reading and writing a status block in RAM with plain
`probe-rs read` / `probe-rs write`.

## Hardware (from stock firmware `config.py` + silkscreen)

| Function | GPIO |
|---|---|
| Buttons BOOT / START / SELECT / RIGHT / LEFT (active-low, pull-up) | 0 / 9 / 21 / 46 / 47 |
| WS2812B chain, 8 LEDs | 45 |
| Alert LED (red) / Raffle LED (green) | 2 / 4 |
| GC9A01 240×240 SPI: CS / MOSI / SCK / DC / RST | 10 / 11 / 12 / 13 / 14 |
| IR TX / RX (38 kHz) | 41 / 42 |
| Link UART TX / RX | 39 / 40 |
| USB-Serial-JTAG (probe-rs) | 19 / 20 — never touched |

The authoritative table is `crates/badge-core/src/board.rs`.

The stock image (MicroPython 1.27 + littlefs + DOOM partition) is backed up
outside this repo in `../stock/stock_full_8MB.bin` (sha256 alongside).

## Architecture

- `proto/badge.proto` — the protocol. `HostMsg` host→badge, `BadgeMsg`
  badge→host. Fixed capacities for every container.
- `crates/badge-proto` — micropb-generated no_std types + `encode`/`decode`.
- `crates/badge-core` — functional core. `App::handle(Input) -> (App, Effects)`
  is pure; `ui::render` draws the model on any embedded-graphics target.
  COBS framing, debouncer, LED patterns, pin map, PROBE block layout.
- `firmware/` — imperative shell (esp-hal 1.2, esp-rtos/embassy, esp-radio).
  Turns hardware into `Input`s and executes `Effect`s.
- `host/badgectl` — probe-rs based CLI: flash/attach, defmt log decoding,
  protobuf request/response, PROBE block peek/poke.

Transport: RTT up channel 0 = defmt logs, up channel 1 `proto-up`,
down channel 0 `proto-down`. Each message is one COBS frame ending in 0x00.

Heap: a bounded `esp-alloc` heap exists only because esp-radio requires it.
Application code is heap-free (heapless containers, static cells).

## MVP functional requirements

Status: `[x]` = validated on hardware or by the named regression test.

### Platform
- [x] FR-1 Firmware builds for `xtensa-esp32s3-none-elf` and runs via `probe-rs run`, logging with defmt over RTT. — hw 2026-10-23: `probe-rs run` boot log shows all drivers + radio up
- [x] FR-2 No `unsafe` in project code; no panics on production paths; app code allocates nothing. — `#![forbid(unsafe_code)]` in firmware, no `unsafe` in crates/host; heap only via esp-radio

### Transport
- [x] FR-3 Messages are COBS-framed; decoder resynchronises after overflow/corruption. — `badge-core::frame::tests`
- [x] FR-4 All protocol messages round-trip within fixed capacities; oversize fields are rejected. — `badge-proto::tests`
- [x] FR-5 Undecodable host frames produce an `Error{DECODE}` response and increment the decode-error counter. — `badge-core::app::tests::fr5_*`
- [x] FR-6 Every `HostMsg` gets exactly one `Response` echoing its `seq`. — `badge-core::app::tests::fr6_every_request_gets_exactly_one_response`

### Info
- [x] FR-7 `Ping` returns `Pong` with nonce and uptime; `GetInfo` returns firmware, chip, MAC, uptime, probe pins. — hw: `badgectl ping 42`, `badgectl info` (MAC e8:3d:c1:f6:0a:c8)

### Buttons
- [x] FR-8 The five buttons are debounced (stable ≥ 30 ms); each press/release emits a `ButtonEvent`. — hw: press/release events streamed; `buttons::tests`, `fr8_*`
- [x] FR-9 `GetButtons` returns the pressed mask and per-button press counts. — hw: `badgectl buttons` counts; `fr9_*`

### LEDs
- [x] FR-10 `SetLed`/`SetLeds` set individual WS2812 colours (switching the pattern to MANUAL); out-of-range index → `BAD_ARGUMENT`. — hw: `led`, `leds`, `led 9` → BAD_ARGUMENT; `fr10_*`
- [x] FR-11 `SetPattern` selects OFF/MANUAL/RAINBOW/BREATHE and global brightness (CHASE/BUTTONS were removed; values 4/5 are rejected with `BAD_ARGUMENT`); animated patterns advance on ticks. — hw: `pattern rainbow`, PROBE leds animate; `leds::tests`, `fr11_*`
- [x] FR-12 `SetIndicator` switches the red alert and green raffle LEDs. — hw: commands acked; `fr12_*` (visual check by a human pending)

### Display
- [x] FR-13 The GC9A01 shows a UI with screens HOME, PET, TRADE, CONTACTS, LEDS, BLE, WIFI, BUTTONS, HOST (plus GPIO in dev builds); LEFT/RIGHT cycle screens, SELECT runs the screen action (LEDS: OFF → RAINBOW → BREATHE, WIFI/BLE: scan). — hw: button presses emit ScreenChanged; `fr13_*`, `ui::tests` (visual check pending)
- [x] FR-14 `ShowScreen` switches screens and emits `ScreenChanged`; `DisplayText` shows text on HOST; `DisplayFill`/`DisplayRect` paint the HOST canvas. — hw: `screen`, `text`, `rect` acked; `fr14_*` (visual check pending)

### GPIO probe
- [x] FR-15 In a `dev` build, `GpioRead`/`GpioWrite` work on `board::PROBE_PINS` only; any other pin → `PIN_NOT_ALLOWED`. — hw: `gpio read 1`, `gpio write 5 1`, `gpio read 19` → PIN_NOT_ALLOWED; `fr15_*`

### Wi-Fi
- [x] FR-16 `WifiScan` returns up to 12 APs (SSID, BSSID, RSSI, channel, auth), strongest first; concurrent scan → `BUSY`. — hw: `badgectl wifi scan` listed 5 APs sorted by RSSI; `radio_rank::tests`
- [ ] FR-17 `WifiConnect` joins a WPA2/open network with credentials supplied at runtime (none baked in) and reports DHCP IPv4; `WifiStatus` reports link state.

### Bluetooth LE
- [x] FR-18 `BleScan` returns up to 12 advertisers (address, RSSI, name) heard within the duration. — hw: `badgectl ble scan` listed 10 devices with names; `radio_rank::tests`
- [x] FR-19 `BleAdvertise` advertises a connectable/scannable name; empty name stops advertising. — hw: host `bluetoothctl scan le` saw `conagotchi-rs` advertising

### Debugger access
- [x] FR-20 A `PROBE` block (layout `badge-core::probe_layout`) is refreshed each tick and readable with probe-rs while the badge runs. — hw: stock `probe-rs read b32 <PROBE> 27` shows magic bad6e001 and live state
- [x] FR-21 Writing the PROBE mailbox (SET_LED, SHOW_SCREEN, SET_PATTERN) drives the badge without RTT. — hw: stock `probe-rs write` of SHOW_SCREEN switched screen; `badgectl poke`; `fr21_*`, `session::tests::mailbox_*`

### Host tool
- [x] FR-22 `badgectl` attaches over probe-rs, streams decoded defmt logs, and exposes a subcommand per request type plus `events`, `peek`, `poke`. — hw: every request subcommand plus `events`, `peek`, `poke`, `addr` exercised

### Pet
- [x] FR-23 PET screen shows an animated cat; SELECT cycles modes AUTO/IDLE/WALK/EAT/PLAY/SLEEP; AUTO rotates modes every ~8 s. — `fr23_*` (badge-core `pet`, `app`, `ui` tests) — hw 2026-10-23: boots on PET, AUTO advanced EAT→PLAY (PetChanged events), 250 ticks/5 s while animating; `fr23_*`, `pet::tests` (visual check by a human pending)
- [x] FR-24 `SetPetMode` (and mailbox SET_PET_MODE) selects the mode from the host; mode changes emit PetChanged; PROBE exposes selected/active mode. — `fr24_*` (badge-core); PROBE words 27/28 published by firmware — hw: `badgectl pet <mode>` for all six modes, `poke pet play`, `peek` shows selected/active; `fr24_*`

### Stock-badge interop (OZ protocol)
- [ ] FR-25 The badge trades Chi with stock MicroPython badges in both directions over BLE (single OZ manufacturer AD, lobby `OZGT` packets, address/flag exchange, 1.5 s linger), ignores peers weaker than −55 dBm, and never completes a trade for an unknown tag. — `fr25_*` (golden stock vectors, two-badge simulations over BLE/link); hw 2026-10-24: advert decoded by BlueZ as manufacturer 0x5a4f `00 OZGT:F60AC8:------:~VIB` + 3-byte tag, `trade invite` → INVITING → LOST after 20 s. Pending: trade with a stock badge (needs a second badge).
- [ ] FR-26 Every completed trade adds or updates a contact (id, Chi received, trade count, `OZN` name if overheard), at most 64, persisted across reboots; `ListContacts`/`ClearContacts` and the CONTACTS screen expose them. — `fr26_*`, `store::tests`; hw: `contacts --all` on an empty store. Pending: a real trade, then reboot.
- [x] FR-27 With a radio key, outgoing packets carry the 3-byte truncated HMAC-SHA256 tag and incoming packets with a bad or missing tag are rejected; without a key the badge sends unsigned and the TRADE screen says so. — `fr27_*` (RFC 4231 vectors, fake keys only); hw: `lobby` reports "signed" after import. Pending: a stock badge accepting our tag.
- [x] FR-28 On first boot the firmware imports `radio_key.txt` from the stock littlefs `vfs` partition into its own `conag` partition (the stock layout is otherwise unchanged); the key is logged only as present/absent and appears in no source file or release binary. — `fr28_*` (littlefs fixtures with fake keys); hw 2026-10-24: first boot `storage: radio key imported`, after reset `storage: radio key present`; a byte search for the real key over the repo, the ELF and the merged image found nothing.

### BLE
- [x] FR-29 One connectable-scannable advertising set carries the OZ advert (name in the scan response) while a continuous passive scan runs; the BLE screen and `GetBleMetrics` show adverts/s, unique addresses (10 s), OZ badges, strongest RSSI and lobby peers. — `fr29_*`; hw: `badgectl ble metrics` (19 adv/s, 16 unique, −67 dBm), `bluetoothctl` sees `conagotchi` (scan response) with the OZ manufacturer data, `ble scan` and `wifi scan` (7 APs) still work.
- [x] FR-30 A GATT service "Conagotchi" offers info (read), pet mode and LED pattern (read/write) and LED colour (write); writes act exactly like the matching host requests. — `fr30_*`; hw (bluetoothctl): info read `conagotchi-fw 0.1.0;id=F60AC8;chi=VIB`, pet write 5 → PROBE SLEEP, pattern write 3 → BREATHE, pattern 4 rejected, colour write → MANUAL on all 8 LEDs.

### LED sync
- [ ] FR-31 Badges running this firmware broadcast ESP-NOW sync beacons; LED animations render from network time, so nearby badges converge to within ±20 ms of the leader (lowest id heard in 3 s) and hand over when it goes silent; `GetSync` and the PROBE block report leader and offset. — `fr31_*` (simulated 3 badges, ±200 ppm, loss; two Apps render identical LEDs); hw: single badge reports itself as leader, offset 0. Pending: two badges.

### Product
- [x] FR-32 HOME shows a scannable QR code for `https://github.com/jvishnefske/conagotchi` with the firmware version underneath. — `fr32_*` (rqrr decodes the rendered frame to the URL). Pending: phone scan of the panel.
- [x] FR-33 Without the `dev` cargo feature the GPIO screen, the probe-pin claims and the GPIO requests are compiled out (`UNSUPPORTED`, empty `probe_pins`); CI builds and tests both variants. — `scripts/check.sh`, `fr15_*`; hw: default build answers `gpio read 1` with UNSUPPORTED, `--features dev` build reads GPIO1 and lists 17 probe pins.

### Flashing and adoption

Quick start (no Rust toolchain needed). With Nix installed and the badge plugged in over USB, run `nix shell nixpkgs#espflash`, then `curl -LO https://github.com/jvishnefske/conagotchi/releases/latest/download/conagotchi-merged.bin`, then `espflash write-bin 0x0 conagotchi-merged.bin`. The image holds only the bootloader, the partition table and the app, and it ends below 0x200000 (it is saved with `--skip-padding`). The stock vfs partition, including `radio_key.txt`, which the firmware imports on first boot, and the doom partition are left untouched. The stock nvs and phy_init areas between 0x9000 and 0x10000 are erased. If the badge stays dark afterwards, unplug it and plug it back in once to clear the download-mode latch. To keep a backup of the stock app, use `badgectl adopt` instead.

Flash layout: `firmware/partitions.csv` is the stock table with the unused otadata slot turned into `conag` (0x200000, 8 KiB, contacts and the imported key). Without otadata the bootloader always boots `factory`. Restoring stock MicroPython means writing the stock image back, partition table included (`badgectl restore`, or `espflash write-bin 0 stock_full_8MB.bin`).
- [ ] FR-34 `badgectl adopt` notices a newly plugged badge, asks one confirmation, backs up 0x0–0x200000, flashes bootloader + partition table + app, and verifies the badge answers `info`; `badgectl restore` writes a backup back. — `adopt::tests` (fake hardware). Pending: hardware run with a freshly plugged stock badge.
- [ ] FR-35 A tagged release produces `conagotchi-merged.bin` (no key, not past 0x200000) that the three-command Nix quick start flashes over USB. — hw 2026-10-24: a locally built merged image (0xE9480 bytes) written with Nix-provided espflash at 0x0 booted without a replug and kept the key. Pending: a real tagged release.


## Spike: cloning from one badge to another without a PC (findings, 2026-10-23)

Read-only research. No shipping code came out of it.

The badge already has a wired link to other badges. The stock `config.py` puts the link UART on GPIO39 (TX) and GPIO40 (RX) at 115200 baud. The front of the PCB has a 3-pin LINK RX / GND / LINK TX header. The back has a 4-pin "UART G T R V" header and BOOT and RESET buttons.

USB host mode exists but is immature. esp-hal 1.2.2 ships an embassy USB host driver for the S3 OTG peripheral (`usb/otg/embassy_usb_host.rs`, on embassy-usb-synopsys-otg with the `host` feature). embassy-usb-host 0.1.0 provides a CDC-ACM host class with DTR/RTS control. The `unsafe` stays inside esp-hal.

The big cost of host mode is the debug link. Enabling OTG moves the single internal USB PHY on GPIO19/20 away from USB-Serial-JTAG until reset, so probe-rs, RTT and the CDC console are gone while cloning. The PHY-select eFuse must never be burned.

The badge cannot power a peer. It runs from a 3.7 V 800 mAh Li-ion cell through a charger and an LDO. The USB-C port is sink-only, with CC resistors and no boost converter. A peer runs from its own battery, and a C-to-C cable between two sinks still carries D+/D-.

No no_std Rust implementation of the ESP ROM loader exists. espflash keeps its protocol behind the std `serialport` feature. The only embedded implementation is Espressif's C esp-serial-flasher. A pure functional-core port would cover:

- SLIP framing;
- SYNC, SPI_ATTACH, FLASH_BEGIN/DATA/END, SPI_FLASH_MD5 and CHANGE_BAUDRATE;
- uncompressed writes.

That is roughly 600–900 lines, host-testable against recorded esptool traces.

The S3 ROM has no READ_FLASH, and a badge has nowhere to hold a 2 MB backup. So a clone from one badge to another cannot back up the peer the way `badgectl adopt` does.

Over-the-air updates between badges already running this firmware are possible. esp-bootloader-esp-idf 0.6 has an `OtaUpdater` that needs a real otadata partition and two OTA slots. Two constraints follow:

- **Partition layout.** The doom region (640 KB) is smaller than the app, so it cannot hold a second slot. The workable layout is: shrink nvs to 0x4000, move `conag` to 0xD000, ota_0 at 0x10000 (960 KB), ota_1 at 0x100000 (1 MB), and a real otadata at 0x200000, with vfs and doom untouched. This conflicts with today's use of 0x200000 for `conag`, so it would have to be decided before a mass rollout.
- **Flash stalls.** esp-storage erases stall interrupts for 45–400 ms, and ESP-NOW queues only 10 frames. The target slot should therefore be erased up front, and the transfer should be windowed and acknowledged.

Expected transfer time is 15–40 s for 800 KB with 250-byte frames, or under 10 s with ESP-NOW v2 frames. Images must be SHA-256 checked and Ed25519 signed, with a confirmation on both badges. ESP-NOW cannot install onto a stock badge.

Recommendation:

1. **Wired UART clone (best for installing onto a stock badge).** Connect the host badge's LINK header to the new badge's back UART header, which is presumably UART0 on GPIO43/44; this needs checking with a meter. Put the new badge in download mode with BOOT plus RESET. A ROM-loader core in badge-core then writes the image at up to 921600 baud. Estimate: 4–6 days. A one-day proof of concept would do SYNC and a chip-id READ_REG, then write and MD5-check a 64 KB pattern outside vfs.
2. **USB-host clone (defer).** Same core, plus USB host and CDC-ACM. High risk, and the debug link is lost during cloning.
3. **ESP-NOW OTA (later).** An update spreader between badges already running this firmware, after the A/B partition change. Verify first that espflash's bootloader honours otadata.
