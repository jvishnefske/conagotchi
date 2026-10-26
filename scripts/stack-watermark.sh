#!/usr/bin/env bash
# Main stack high-water mark of the running badge: paint, soak, report.
#
#   1. `badgectl stack paint` fills the unused part of the main stack
#      (above esp-hal's guard word, below the sampled SP minus the margin)
#      with the paint pattern. Sampling SP halts core 0 for a moment;
#      `-t ADDR` paints below ADDR instead and does not halt.
#   2. Waits `-s SECONDS`, or runs the soak command `-c CMD` until it exits.
#      With `-r`, runs `badgectl rtt-check` every `-i` seconds (default 30)
#      meanwhile and remembers failures.
#   3. `badgectl stack report --painted-top <top of the paint>` prints the
#      high-water mark and margin.
#
# Usage:
#   scripts/stack-watermark.sh [-s SECONDS | -c CMD] [-r] [-i SECONDS]
#                              [-m MARGIN] [-t ADDR] [-H] [-- BADGECTL_ARGS...]
#
#   -s SECONDS  soak by waiting (default 60)
#   -c CMD      soak by running CMD with bash -c (overrides -s)
#   -r          run rtt-check every -i seconds during the soak, and once at the end
#   -i SECONDS  rtt-check interval (default 30)
#   -m MARGIN   bytes kept unpainted below SP (badgectl default 16384)
#   -t ADDR     paint below ADDR instead of SP minus margin (no halt)
#   -H          keep core 0 halted while painting (`stack paint --halt`)
#   -- ARGS     global badgectl options, e.g. `-- --probe 303a:1001:<MAC> --elf fw.elf`
#
# The ELF must be the build that is flashed (badgectl warns when the PROBE
# magic is missing at the ELF's address). With more than one debug probe
# attached, pass `--probe` after `--`: the default is the first probe listed.
# A soak command that uses the same probe can collide with -r's rtt-check.
#
# Environment:
#   BADGECTL=path  badgectl binary (default: target/release/badgectl, built
#                  with cargo if missing).
#
# Exit status: 0 when every report and rtt-check passed, 1 otherwise
# (stack not painted, guard word changed, RTT block corrupt, soak command
# failed), 2 on usage errors.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

seconds=60
soak_cmd=""
rtt=0
interval=30
paint_args=()

usage() { sed -n '2,/^set -euo/p' "${BASH_SOURCE[0]}" | sed -e '/^set -euo/d' -e 's/^# \{0,1\}//'; }

while getopts ":s:c:ri:m:t:Hh" opt; do
    case "$opt" in
        s) seconds="$OPTARG" ;;
        c) soak_cmd="$OPTARG" ;;
        r) rtt=1 ;;
        i) interval="$OPTARG" ;;
        m) paint_args+=(--margin "$OPTARG") ;;
        t) paint_args+=(--top "$OPTARG") ;;
        H) paint_args+=(--halt) ;;
        h) usage; exit 0 ;;
        :) echo "error: -$OPTARG needs a value" >&2; exit 2 ;;
        *) echo "error: unknown option -$OPTARG" >&2; usage >&2; exit 2 ;;
    esac
done
shift $((OPTIND - 1))
[[ "${1:-}" == "--" ]] && shift
global=("$@")

for n in "$seconds" "$interval"; do
    [[ "$n" =~ ^[0-9]+$ ]] || { echo "error: '$n' is not a number of seconds" >&2; exit 2; }
done
(( interval > 0 )) || { echo "error: -i must be positive" >&2; exit 2; }

badgectl="${BADGECTL:-$root/target/release/badgectl}"
if [[ -z "${BADGECTL:-}" && ! -x "$badgectl" ]]; then
    echo "building badgectl ..." >&2
    cargo build --release -q -p badgectl --manifest-path "$root/Cargo.toml"
fi

bctl() { "$badgectl" "${global[@]}" "$@"; }

failed=0
rtt_runs=0
rtt_failures=0

check_rtt() {
    rtt_runs=$((rtt_runs + 1))
    printf '\n==> rtt-check #%d at +%ds\n' "$rtt_runs" "$SECONDS"
    if ! bctl rtt-check; then
        rtt_failures=$((rtt_failures + 1))
        failed=1
    fi
}

printf '==> stack paint\n'
paint_out="$(bctl stack paint "${paint_args[@]}")"
printf '%s\n' "$paint_out"
painted_top="$(printf '%s\n' "$paint_out" | sed -n 's/^painted 0x[0-9a-f]*\.\.\(0x[0-9a-f]*\) .*/\1/p' | head -n 1)"
if [[ -z "$painted_top" ]]; then
    echo "error: could not read the paint range from badgectl's output" >&2
    exit 1
fi

SECONDS=0
next_rtt=$interval
if [[ -n "$soak_cmd" ]]; then
    printf '\n==> soak: %s\n' "$soak_cmd"
    bash -c "$soak_cmd" &
    soak_pid=$!
    trap 'kill "$soak_pid" 2>/dev/null || true' INT TERM
    while kill -0 "$soak_pid" 2>/dev/null; do
        if (( rtt && SECONDS >= next_rtt )); then
            check_rtt
            next_rtt=$((SECONDS + interval))
        fi
        sleep 1
    done
    if ! wait "$soak_pid"; then
        echo "soak command failed" >&2
        failed=1
    fi
    trap - INT TERM
else
    printf '\n==> soak: waiting %ss\n' "$seconds"
    while (( SECONDS < seconds )); do
        if (( rtt && SECONDS >= next_rtt )); then
            check_rtt
            next_rtt=$((SECONDS + interval))
        fi
        sleep 1
    done
fi

(( rtt )) && check_rtt

printf '\n==> stack report after %ss\n' "$SECONDS"
if ! bctl stack report --painted-top "$painted_top"; then
    failed=1
fi

if (( rtt )); then
    printf '\nrtt-check: %d run(s), %d failed\n' "$rtt_runs" "$rtt_failures"
fi
exit "$failed"
