// MW v1 ESP-NOW frames: a dependency-free C++ port of middleware's
// `espnow::Frame` (jvishnefske/middleware, src/espnow.rs) for peers that do
// not run Rust, here an ESP8266 on the Arduino core.
//
// Wire format (little-endian), at most kMaxFrame (250) bytes:
//   'M' 'W' | version 1 | short_id u16 | seq u8 | key_len u8 | key | payload
// short_id is the 64-bit FNV-1a of the key folded to 16 bits (the same id
// the BLE advertising backend uses). Keys follow middleware key rules:
// 1..=64 bytes, '/'-separated non-empty segments of [A-Za-z0-9_.-].
// Key and payload share 243 bytes (kMaxFrame - kHeaderLen).
//
// Pure functions only (no Arduino headers), so the host test in
// test_mw_frame.cpp checks them against golden frames produced by the Rust
// implementation.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace mw {

constexpr size_t kHeaderLen = 7;
constexpr size_t kMaxFrame = 250;
constexpr size_t kMaxKey = 64;
constexpr uint8_t kVersion = 1;

enum Error : int {
  kOk = 0,
  kTooShort = -1,
  kBadMagic = -2,
  kBadVersion = -3,
  kBadKeyLength = -4,
  kBadKey = -5,
  kIdMismatch = -6,
  kTooLong = -7,
  kBufferTooSmall = -8,
};

inline uint64_t fnv1a64(const uint8_t* bytes, size_t len) {
  uint64_t hash = 0xcbf29ce484222325ULL;
  for (size_t i = 0; i < len; ++i) {
    hash ^= bytes[i];
    hash *= 0x00000100000001b3ULL;
  }
  return hash;
}

inline uint32_t fnv1a32(const uint8_t* bytes, size_t len) {
  uint32_t hash = 0x811c9dc5u;
  for (size_t i = 0; i < len; ++i) {
    hash ^= bytes[i];
    hash *= 0x01000193u;
  }
  return hash;
}

inline uint16_t short_id(const char* key, size_t len) {
  const uint64_t h = fnv1a64(reinterpret_cast<const uint8_t*>(key), len);
  return static_cast<uint16_t>(h ^ (h >> 16) ^ (h >> 32) ^ (h >> 48));
}

// True if key[0..len) is a valid middleware key no longer than kMaxKey.
inline bool key_is_valid(const char* key, size_t len) {
  if (len == 0 || len > kMaxKey) return false;
  size_t segment = 0;
  for (size_t i = 0; i < len; ++i) {
    const char c = key[i];
    if (c == '/') {
      if (segment == 0) return false;
      segment = 0;
    } else if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
               (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.') {
      ++segment;
    } else {
      return false;
    }
  }
  return segment != 0;
}

// Encodes a frame into out[0..cap). Returns its length, or a negative Error.
inline int encode(uint8_t seq, const char* key, size_t key_len,
                  const uint8_t* payload, size_t payload_len, uint8_t* out,
                  size_t cap) {
  if (!key_is_valid(key, key_len)) return kBadKey;
  const size_t len = kHeaderLen + key_len + payload_len;
  if (len > kMaxFrame) return kTooLong;
  if (len > cap) return kBufferTooSmall;
  const uint16_t id = short_id(key, key_len);
  out[0] = 'M';
  out[1] = 'W';
  out[2] = kVersion;
  out[3] = static_cast<uint8_t>(id & 0xff);
  out[4] = static_cast<uint8_t>(id >> 8);
  out[5] = seq;
  out[6] = static_cast<uint8_t>(key_len);
  for (size_t i = 0; i < key_len; ++i) out[kHeaderLen + i] = static_cast<uint8_t>(key[i]);
  for (size_t i = 0; i < payload_len; ++i) out[kHeaderLen + key_len + i] = payload[i];
  return static_cast<int>(len);
}

// A decoded frame; key and payload point into the decoded buffer.
struct Frame {
  uint8_t seq;
  const char* key;
  size_t key_len;
  const uint8_t* payload;
  size_t payload_len;
};

// Decodes buf[0..len) into *out. Returns kOk or a negative Error. A frame
// without the "MW" magic is kBadMagic (not ours: skip it silently).
inline int decode(const uint8_t* buf, size_t len, Frame* out) {
  if (len < kHeaderLen) return len >= 2 && (buf[0] != 'M' || buf[1] != 'W') ? kBadMagic : kTooShort;
  if (buf[0] != 'M' || buf[1] != 'W') return kBadMagic;
  if (len > kMaxFrame) return kTooLong;
  if (buf[2] != kVersion) return kBadVersion;
  const size_t key_len = buf[6];
  if (key_len == 0 || key_len > kMaxKey) return kBadKeyLength;
  if (kHeaderLen + key_len > len) return kTooShort;
  const char* key = reinterpret_cast<const char*>(buf + kHeaderLen);
  if (!key_is_valid(key, key_len)) return kBadKey;
  const uint16_t id = static_cast<uint16_t>(buf[3] | (buf[4] << 8));
  if (id != short_id(key, key_len)) return kIdMismatch;
  out->seq = buf[5];
  out->key = key;
  out->key_len = key_len;
  out->payload = buf + kHeaderLen + key_len;
  out->payload_len = len - kHeaderLen - key_len;
  return kOk;
}

// Last sequence seen per (sender MAC, key), like middleware's
// dedup::SeenTable: two senders on one key never suppress each other; a full
// table replaces its least recently updated entry.
template <size_t N>
class SeenTable {
 public:
  // Records seq from mac on key; returns true if the message is fresh.
  bool observe(const uint8_t mac[6], const char* key, size_t key_len, uint8_t seq) {
    const uint32_t hash = fnv1a32(reinterpret_cast<const uint8_t*>(key), key_len);
    ++clock_;
    size_t slot = N;
    for (size_t i = 0; i < N; ++i) {
      if (used_[i] && hash_[i] == hash && same_mac(i, mac)) { slot = i; break; }
    }
    if (slot == N) {
      for (size_t i = 0; i < N; ++i) if (!used_[i]) { slot = i; break; }
    }
    if (slot == N) {
      uint32_t oldest = 0;
      for (size_t i = 0; i < N; ++i) {
        const uint32_t age = clock_ - stamp_[i];
        if (slot == N || age > oldest) { slot = i; oldest = age; }
      }
    }
    if (slot == N) return true;  // N == 0
    const bool fresh = !(used_[slot] && hash_[slot] == hash && same_mac(slot, mac) && seq_[slot] == seq);
    used_[slot] = true;
    hash_[slot] = hash;
    seq_[slot] = seq;
    stamp_[slot] = clock_;
    for (size_t b = 0; b < 6; ++b) mac_[slot][b] = mac[b];
    return fresh;
  }

 private:
  bool same_mac(size_t i, const uint8_t mac[6]) const {
    for (size_t b = 0; b < 6; ++b) if (mac_[i][b] != mac[b]) return false;
    return true;
  }
  bool used_[N == 0 ? 1 : N] = {};
  uint8_t mac_[N == 0 ? 1 : N][6] = {};
  uint32_t hash_[N == 0 ? 1 : N] = {};
  uint8_t seq_[N == 0 ? 1 : N] = {};
  uint32_t stamp_[N == 0 ? 1 : N] = {};
  uint32_t clock_ = 0;
};

}  // namespace mw
