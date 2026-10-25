#!/usr/bin/env bash
# Builds the wasm demo apps and copies them next to their sources.
#
#   scripts/build-apps.sh          build apps/hello and write apps/hello.wasm
#   scripts/build-apps.sh --check  build, and fail if apps/hello.wasm differs
#                                  from the fresh build (artifact out of date)
#
# Each app is its own cargo workspace (not a member of the host workspace)
# built on middleware's `mw-payload` SDK (a git dependency, pinned to the
# workspace's middleware rev in the app's Cargo.toml and Cargo.lock). Its
# `.cargo/config.toml` selects wasm32-unknown-unknown and puts the build
# under target/apps; this script sets the linker flags, as middleware's
# payloads/scripts/build-byte-pages.sh does: 1-byte pages, a fixed
# MEMORY_BYTES (16384, the badge's wasm-host memory cap) linear memory with
# a STACK_BYTES (4096) shadow stack. It also remaps the cargo home and the
# repository paths that panic locations embed, so the module does not
# depend on where it was built and `--check` works on any machine. The committed apps/*.wasm are what the simulator tests load
# and what `badgectl wasm load` uploads; each must fit
# badge_core::board::PAYLOAD_MAX_IMAGE (16 KiB). Requires the stable
# toolchain with the wasm32-unknown-unknown target (rustup +stable target
# add wasm32-unknown-unknown) and network access for the git dependency on
# the first build.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
check=0
if [[ "${1:-}" == "--check" ]]; then
    check=1
fi

MEMORY_BYTES=16384
STACK_BYTES=4096
cargo_home="${CARGO_HOME:-$HOME/.cargo}"
# RUSTFLAGS replaces the rustflags of any config file, for the wasm32 target
# only (the build targets wasm32 through .cargo/config.toml).
export RUSTFLAGS="-C link-arg=--page-size=1 \
-C link-arg=-zstack-size=$STACK_BYTES \
-C link-arg=--initial-memory=$MEMORY_BYTES \
-C link-arg=--max-memory=$MEMORY_BYTES \
--remap-path-prefix=$cargo_home=/cargo \
--remap-path-prefix=$root=/conagotchi"

status=0
for app in hello; do
    (cd "$root/apps/$app" && cargo +stable build --release --locked)
    built="$root/target/apps/wasm32-unknown-unknown/release/$app.wasm"
    dest="$root/apps/$app.wasm"
    size=$(stat -c %s "$built")
    if (( size > 16384 )); then
        echo "$app.wasm is $size bytes, over badge_core::board::PAYLOAD_MAX_IMAGE (16384)" >&2
        exit 1
    fi
    if (( check )); then
        if cmp -s "$built" "$dest"; then
            echo "$app.wasm up to date ($size bytes)"
        else
            echo "$app.wasm is out of date: run scripts/build-apps.sh" >&2
            status=1
        fi
    else
        cp "$built" "$dest"
        echo "wrote apps/$app.wasm ($size bytes)"
    fi
done
exit "$status"
