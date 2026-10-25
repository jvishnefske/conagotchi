#!/usr/bin/env bash
# CI checks for both feature sets (default and `dev`).
#
# Host workspace: tests and clippy (-D warnings), without and with
# `--features badge-core/dev`, plus the no_std build of badge-core for
# thumbv7em-none-eabihf; badge-sim for wasm32 and apps/hello.wasm freshness. Firmware: release build and clippy, default and
# `--features dev`, using the esp toolchain from ~/export-esp-new.sh, and the
# app image size (espflash save-image) against the 0x1D0000 factory partition.
#
# Environment:
#   SKIP_FIRMWARE=1   skip the firmware part (e.g. no esp toolchain installed);
#                     it is also skipped with a notice if the export script is
#                     missing.
#   ESP_EXPORT=path   esp toolchain export script (default ~/export-esp-new.sh).
#   HOST_PACKAGES="-p badge-core"
#                     restrict the host part to some packages (default: the
#                     whole workspace).
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"

host_packages="${HOST_PACKAGES:---workspace}"
# shellcheck disable=SC2206 # intentional word splitting of the package list
host_args=($host_packages)

step() { printf '\n==> %s\n' "$*"; }

for features in "" "--features badge-core/dev"; do
    label="${features:-default features}"
    # shellcheck disable=SC2086 # empty or a single flag pair
    {
        step "host tests ($label)"
        cargo +stable test "${host_args[@]}" $features
        step "host clippy ($label)"
        cargo +stable clippy "${host_args[@]}" --all-targets $features -- -D warnings
    }
done

step "peers: ESP8266 MW v1 frame matches the Rust golden frames"
if command -v "${CXX:-g++}" >/dev/null; then
    scripts/test-peers.sh
else
    echo "skipped: no C++ compiler"
fi

step "badge-core no_std build (thumbv7em-none-eabihf)"
cargo +stable build -p badge-core --target thumbv7em-none-eabihf
cargo +stable build -p badge-core --target thumbv7em-none-eabihf --features dev

step "wasm: badge-sim for wasm32 and the committed demo app is current"
cargo +stable build -p badge-sim --target wasm32-unknown-unknown --release
scripts/build-apps.sh --check

esp_export="${ESP_EXPORT:-$HOME/export-esp-new.sh}"
if [[ "${SKIP_FIRMWARE:-0}" == "1" ]]; then
    step "firmware skipped (SKIP_FIRMWARE=1)"
elif [[ ! -f "$esp_export" ]]; then
    step "firmware skipped ($esp_export not found)"
else
    # shellcheck disable=SC1090 # path chosen at runtime
    . "$esp_export"
    cd "$root/firmware"
    for features in "" "--features dev"; do
        label="${features:-default features}"
        # shellcheck disable=SC2086 # empty or a single flag pair
        {
            step "firmware build ($label)"
            cargo build --release $features
            step "firmware clippy ($label)"
            cargo clippy --release $features -- -D warnings
            step "firmware app image fits factory 0x1D0000 ($label)"
            image="$(mktemp)"
            espflash save-image --chip esp32s3 --flash-size 8mb \
                target/xtensa-esp32s3-none-elf/release/conagotchi-fw "$image" >/dev/null
            size="$(stat -c %s "$image")"
            rm -f "$image"
            echo "app image $size of $((0x1D0000)) bytes"
            if (( size > 0x1D0000 )); then
                echo "error: app image exceeds the factory partition (partitions.csv)" >&2
                exit 1
            fi
        }
    done
fi

step "all checks passed"
