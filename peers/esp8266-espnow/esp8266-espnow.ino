// ESP8266 middleware peer over ESP-NOW (Arduino core 3.1.x).
//
// Speaks middleware's MW v1 ESP-NOW frame (mw_frame.h, checked byte for byte
// against the Rust implementation by test_mw_frame.cpp), so it interoperates
// with the conagotchi badge (ESP32-S3, middleware `espnow` backend):
//
// - parks on Wi-Fi channel 1 (where an unassociated badge listens) and
//   broadcasts to FF:FF:FF:FF:FF:FF;
// - every 2 s publishes a little-endian u32 counter starting at 8266000 on
//   `app/hello`, the key the badge's demo payload subscribes to; the payload
//   logs "rx 8266000", "rx 8266001", ...;
// - prints every MW frame it hears as
//   "rx <mac> <key> seq=<n> len=<n> payload=<hex>" (other ESP-NOW traffic is
//   counted and ignored), with per-sender duplicate suppression;
// - echoes each new badge `app/hello` payload back on `app/esp8266/echo`.
//
// Serial: 115200 baud. Prints "ready <mac>" at boot and a status line every
// 10 s.

#include <ESP8266WiFi.h>

extern "C" {
#include <espnow.h>
#include <user_interface.h>
}

#include "mw_frame.h"

namespace {

constexpr uint8_t kChannel = 1;
constexpr uint32_t kPublishMs = 2000;
constexpr uint32_t kStatusMs = 10000;
constexpr uint32_t kCounterBase = 8266000;
constexpr size_t kQueue = 8;

uint8_t kBroadcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

// Frames received in the Wi-Fi callback, consumed in loop(). Single producer
// (callback), single consumer (loop); a full queue drops the frame.
struct Rx {
  uint8_t mac[6];
  uint8_t len;
  uint8_t data[mw::kMaxFrame];
};
Rx queue[kQueue];
volatile uint8_t head = 0;  // next slot the callback writes
volatile uint8_t tail = 0;  // next slot loop() reads
volatile uint32_t dropped = 0;

uint8_t own_mac[6];
uint8_t tx_seq = 0;
uint32_t counter = kCounterBase;
uint32_t rx_frames = 0, rx_foreign = 0, rx_bad = 0, rx_dup = 0, tx_ok = 0, tx_fail = 0;
uint32_t next_publish = 0, next_status = 0;
mw::SeenTable<16> seen;
bool echo_pending = false;
uint8_t echo_payload[4];

void on_recv(uint8_t* mac, uint8_t* data, uint8_t len) {
  const uint8_t next = static_cast<uint8_t>((head + 1) % kQueue);
  if (next == tail || len > mw::kMaxFrame) {
    dropped = dropped + 1;
    return;
  }
  Rx& slot = queue[head];
  memcpy(slot.mac, mac, 6);
  memcpy(slot.data, data, len);
  slot.len = len;
  head = next;
}

void print_mac(const uint8_t* mac) {
  for (int i = 0; i < 6; ++i) {
    if (i) Serial.print(':');
    if (mac[i] < 16) Serial.print('0');
    Serial.print(mac[i], HEX);
  }
}

bool publish(const char* key, const uint8_t* payload, size_t len) {
  uint8_t frame[mw::kMaxFrame];
  // The sequence advances before sending, as middleware requires, so a failed
  // send never makes the next message look like a repeat.
  const uint8_t seq = tx_seq++;
  const int n = mw::encode(seq, key, strlen(key), payload, len, frame, sizeof frame);
  if (n < 0) return false;
  const bool ok = esp_now_send(kBroadcast, frame, n) == 0;
  if (ok) ++tx_ok; else ++tx_fail;
  return ok;
}

void handle(const Rx& rx) {
  if (memcmp(rx.mac, own_mac, 6) == 0) return;
  mw::Frame f{};
  const int err = mw::decode(rx.data, rx.len, &f);
  if (err == mw::kBadMagic) { ++rx_foreign; return; }
  if (err != mw::kOk) { ++rx_bad; return; }
  if (!seen.observe(rx.mac, f.key, f.key_len, f.seq)) { ++rx_dup; return; }
  ++rx_frames;
  Serial.print("rx ");
  print_mac(rx.mac);
  Serial.print(' ');
  Serial.write(reinterpret_cast<const uint8_t*>(f.key), f.key_len);
  Serial.printf(" seq=%u len=%u payload=", f.seq, static_cast<unsigned>(f.payload_len));
  for (size_t i = 0; i < f.payload_len && i < 32; ++i) Serial.printf("%02x", f.payload[i]);
  if (f.payload_len == 4) {
    const uint32_t v = f.payload[0] | (f.payload[1] << 8) | (f.payload[2] << 16) |
                       (static_cast<uint32_t>(f.payload[3]) << 24);
    Serial.printf(" (u32 %u)", v);
  }
  Serial.println();
  if (f.key_len == 9 && memcmp(f.key, "app/hello", 9) == 0 && f.payload_len == 4) {
    memcpy(echo_payload, f.payload, 4);
    echo_pending = true;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  wifi_set_channel(kChannel);
  WiFi.macAddress(own_mac);
  if (esp_now_init() != 0) {
    Serial.println("esp_now_init failed");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_recv_cb(on_recv);
  esp_now_add_peer(kBroadcast, ESP_NOW_ROLE_COMBO, kChannel, nullptr, 0);
  Serial.print("\nready ");
  print_mac(own_mac);
  Serial.printf(" channel=%u\n", wifi_get_channel());
  next_publish = millis() + kPublishMs;
  next_status = millis() + kStatusMs;
}

void loop() {
  // Bounded: at most kQueue frames per pass.
  for (size_t i = 0; i < kQueue && tail != head; ++i) {
    handle(queue[tail]);
    tail = static_cast<uint8_t>((tail + 1) % kQueue);
  }
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - next_publish) >= 0) {
    next_publish = now + kPublishMs;
    const uint8_t payload[4] = {static_cast<uint8_t>(counter), static_cast<uint8_t>(counter >> 8),
                                static_cast<uint8_t>(counter >> 16), static_cast<uint8_t>(counter >> 24)};
    if (publish("app/hello", payload, sizeof payload)) {
      Serial.printf("tx app/hello u32 %u\n", counter);
    }
    ++counter;
  }
  if (echo_pending) {
    echo_pending = false;
    publish("app/esp8266/echo", echo_payload, sizeof echo_payload);
  }
  if (static_cast<int32_t>(now - next_status) >= 0) {
    next_status = now + kStatusMs;
    Serial.printf("status rx=%u foreign=%u bad=%u dup=%u dropped=%u tx_ok=%u tx_fail=%u ch=%u\n", rx_frames,
                  rx_foreign, rx_bad, rx_dup, dropped, tx_ok, tx_fail, wifi_get_channel());
  }
  delay(5);
}
