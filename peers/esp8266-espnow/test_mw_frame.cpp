// Host test for mw_frame.h: the C++ MW v1 frame must be byte-identical to
// middleware's Rust `espnow::Frame`. The golden frames below were produced
// by middleware 6b1720e (`Frame::encode`), so a mismatch means the ESP8266
// peer and the badge would not understand each other.
//
// Build and run: g++ -std=c++17 -Wall -Wextra -Werror test_mw_frame.cpp && ./a.out
// (scripts/check.sh does this).
#include <cstdio>
#include <cstring>
#include <initializer_list>

#include "mw_frame.h"

static int failures = 0;
#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
      ++failures;                                                          \
    }                                                                      \
  } while (0)

struct Golden {
  uint8_t seq;
  const char* key;
  uint16_t short_id;
  const uint8_t* payload;
  size_t payload_len;
  const uint8_t* frame;
  size_t frame_len;
};

static const uint8_t kSyncPayload[] = {'O', 'Z', 'L', '1', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'};
static const uint8_t kSyncFrame[] = {
    0x4d, 0x57, 0x01, 0xa2, 0xe5, 0x07, 0x0f, 0x63, 0x6f, 0x6e, 0x61, 0x67,
    0x6f, 0x74, 0x63, 0x68, 0x69, 0x2f, 0x73, 0x79, 0x6e, 0x63, 0x4f, 0x5a,
    0x4c, 0x31, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68};
static const uint8_t kHelloPayload[] = {0x3d, 0x00, 0x00, 0x00};
static const uint8_t kHelloFrame[] = {
    0x4d, 0x57, 0x01, 0xec, 0x7f, 0x00, 0x09, 0x61, 0x70, 0x70,
    0x2f, 0x68, 0x65, 0x6c, 0x6c, 0x6f, 0x3d, 0x00, 0x00, 0x00};
static const uint8_t kEchoFrame[] = {
    0x4d, 0x57, 0x01, 0x1d, 0x51, 0xff, 0x10, 0x61, 0x70, 0x70, 0x2f, 0x65,
    0x73, 0x70, 0x38, 0x32, 0x36, 0x36, 0x2f, 0x65, 0x63, 0x68, 0x6f};

static const Golden kGolden[] = {
    {7, "conagotchi/sync", 0xe5a2, kSyncPayload, sizeof kSyncPayload, kSyncFrame, sizeof kSyncFrame},
    {0, "app/hello", 0x7fec, kHelloPayload, sizeof kHelloPayload, kHelloFrame, sizeof kHelloFrame},
    {255, "app/esp8266/echo", 0x511d, nullptr, 0, kEchoFrame, sizeof kEchoFrame},
};

static void golden_frames_match_rust() {
  for (const Golden& g : kGolden) {
    const size_t key_len = std::strlen(g.key);
    CHECK(mw::short_id(g.key, key_len) == g.short_id);
    uint8_t out[mw::kMaxFrame];
    const int n = mw::encode(g.seq, g.key, key_len, g.payload, g.payload_len, out, sizeof out);
    CHECK(n == static_cast<int>(g.frame_len));
    CHECK(n > 0 && std::memcmp(out, g.frame, g.frame_len) == 0);
    mw::Frame f{};
    CHECK(mw::decode(g.frame, g.frame_len, &f) == mw::kOk);
    CHECK(f.seq == g.seq);
    CHECK(f.key_len == key_len && std::memcmp(f.key, g.key, key_len) == 0);
    CHECK(f.payload_len == g.payload_len);
    CHECK(g.payload_len == 0 || std::memcmp(f.payload, g.payload, g.payload_len) == 0);
  }
}

static void malformed_frames_are_rejected() {
  mw::Frame f{};
  uint8_t buf[sizeof kHelloFrame];
  std::memcpy(buf, kHelloFrame, sizeof buf);
  CHECK(mw::decode(buf, 3, &f) == mw::kTooShort);
  buf[0] = 'X';
  CHECK(mw::decode(buf, sizeof buf, &f) == mw::kBadMagic);
  std::memcpy(buf, kHelloFrame, sizeof buf);
  buf[2] = 2;
  CHECK(mw::decode(buf, sizeof buf, &f) == mw::kBadVersion);
  std::memcpy(buf, kHelloFrame, sizeof buf);
  buf[3] ^= 1;
  CHECK(mw::decode(buf, sizeof buf, &f) == mw::kIdMismatch);
  std::memcpy(buf, kHelloFrame, sizeof buf);
  buf[6] = 40;  // key runs past the end
  CHECK(mw::decode(buf, sizeof buf, &f) == mw::kTooShort);
  buf[6] = 0;
  CHECK(mw::decode(buf, sizeof buf, &f) == mw::kBadKeyLength);
  std::memcpy(buf, kHelloFrame, sizeof buf);
  buf[10] = ' ';  // "app hello"
  CHECK(mw::decode(buf, sizeof buf, &f) == mw::kBadKey);
}

static void keys_and_limits_follow_middleware() {
  for (const char* good : {"a", "app/hello", "app/x_y-1.z"}) CHECK(mw::key_is_valid(good, std::strlen(good)));
  for (const char* bad : {"", "/a", "a/", "a//b", "a b", "a/*"}) CHECK(!mw::key_is_valid(bad, std::strlen(bad)));
  char long_key[mw::kMaxKey + 1];
  std::memset(long_key, 'k', sizeof long_key);
  CHECK(mw::key_is_valid(long_key, mw::kMaxKey));
  CHECK(!mw::key_is_valid(long_key, mw::kMaxKey + 1));
  uint8_t payload[mw::kMaxFrame] = {};
  uint8_t out[mw::kMaxFrame];
  // Key and payload share kMaxFrame - kHeaderLen = 243 bytes.
  CHECK(mw::encode(0, long_key, mw::kMaxKey, payload, 179, out, sizeof out) == static_cast<int>(mw::kMaxFrame));
  CHECK(mw::encode(0, long_key, mw::kMaxKey, payload, 180, out, sizeof out) == mw::kTooLong);
  CHECK(mw::encode(0, "a", 1, payload, 242, out, sizeof out) == static_cast<int>(mw::kMaxFrame));
  CHECK(mw::encode(0, "app/hello", 9, payload, 4, out, 10) == mw::kBufferTooSmall);
}

static void dedup_is_per_sender_and_key() {
  mw::SeenTable<2> t;
  const uint8_t a[6] = {1, 1, 1, 1, 1, 1};
  const uint8_t b[6] = {2, 2, 2, 2, 2, 2};
  CHECK(t.observe(a, "app/x", 5, 0));
  CHECK(!t.observe(a, "app/x", 5, 0));  // repeat
  CHECK(t.observe(b, "app/x", 5, 0));   // other sender, same seq
  CHECK(t.observe(a, "app/x", 5, 1));   // next seq
  CHECK(t.observe(a, "app/y", 5, 1));   // evicts the least recent (b)
  CHECK(t.observe(b, "app/x", 5, 0));   // b forgotten: looks fresh again
}

int main() {
  golden_frames_match_rust();
  malformed_frames_are_rejected();
  keys_and_limits_follow_middleware();
  dedup_is_per_sender_and_key();
  if (failures != 0) {
    std::printf("%d check(s) failed\n", failures);
    return 1;
  }
  std::printf("mw_frame: all checks passed\n");
  return 0;
}
