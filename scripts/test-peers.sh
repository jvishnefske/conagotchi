#!/usr/bin/env bash
# Host tests for non-Rust middleware peers: the ESP8266 sketch's MW v1
# ESP-NOW frame (peers/esp8266-espnow/mw_frame.h) must stay byte-identical to
# middleware's Rust implementation (golden frames in test_mw_frame.cpp).
set -euo pipefail
cd "$(dirname "$0")/.."
CXX="${CXX:-g++}"
out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT
"$CXX" -std=c++17 -Wall -Wextra -Werror peers/esp8266-espnow/test_mw_frame.cpp -o "$out/test_mw_frame"
"$out/test_mw_frame"
