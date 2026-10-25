#!/usr/bin/env bash
# Builds the wasm demo apps and copies them next to their sources.
#
#   scripts/build-apps.sh          build apps/hello and write apps/hello.wasm
#   scripts/build-apps.sh --check  build, and fail if apps/hello.wasm differs
#                                  from the fresh build (artifact out of date)
#
# Each app is its own cargo workspace (not a member of the host workspace);
# its `.cargo/config.toml` selects wasm32-unknown-unknown, 1-byte pages and
# a fixed 8 KiB memory, and puts the build under target/apps. The committed
# apps/*.wasm are what the wasm-runtime tests load and what
# `badgectl wasm load` uploads. Requires the stable toolchain with the
# wasm32-unknown-unknown target (rustup +stable target add
# wasm32-unknown-unknown).
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
check=0
if [[ "${1:-}" == "--check" ]]; then
    check=1
fi

status=0
for app in hello; do
    (cd "$root/apps/$app" && cargo +stable build --release)
    built="$root/target/apps/wasm32-unknown-unknown/release/$app.wasm"
    dest="$root/apps/$app.wasm"
    size=$(stat -c %s "$built")
    if (( size > 16384 )); then
        echo "$app.wasm is $size bytes, over wasm_abi::MAX_MODULE (16384)" >&2
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
