#!/usr/bin/env bash
# CI checks for both feature sets (default and `dev`).
#
# Host workspace: tests and clippy (-D warnings), without and with
# `--features badge-core/dev`, plus the no_std build of badge-core for
# thumbv7em-none-eabihf. Firmware: release build and clippy, default and
# `--features dev`, using the esp toolchain from ~/export-esp-new.sh.
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

step "badge-core no_std build (thumbv7em-none-eabihf)"
cargo +stable build -p badge-core --target thumbv7em-none-eabihf
cargo +stable build -p badge-core --target thumbv7em-none-eabihf --features dev

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
        }
    done
fi

step "all checks passed"
