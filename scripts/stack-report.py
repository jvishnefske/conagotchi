#!/usr/bin/env python3
"""Static stack report for the Xtensa firmware ELF.

Reads `xtensa-esp32s3-elf-objdump -d` output and prints, for every embassy
task's poll function (and any extra roots given with --root), its own
frame and the deepest call chain below it, using the frame sizes from the
windowed-ABI `entry a1, N` prologues (plus the `l32r` / `movsp` sequence
LLVM emits for frames above 32 KiB).

Callees are the direct `call8` targets and every function address loaded by
`l32r` (so calls through a literal are counted; a function pointer that is
only stored is counted too, which can only overestimate). Calls through
pointers computed at run time (trait objects, the executor's task table) are
not seen; each task is therefore reported from its own poll function.
Recursion is cut at the first repeated function and reported.

Usage: stack-report.py ELF [--root SUBSTRING ...] [--chain N] [--frames N] [--disasm FILE]
"""

import argparse
import re
import subprocess
import sys

FUNC = re.compile(r"^([0-9a-f]+) <(.*)>:$")
ENTRY = re.compile(r"\bentry\ta1, (0x[0-9a-f]+|\d+)")
L32R = re.compile(r"\bl32r\ta\d+, [0-9a-f]+ <.*> \(([0-9a-f]+)(?: <.*>)?\)$")
CALL = re.compile(r"\bcall(?:0|4|8|12)\t([0-9a-f]+)")


def is_poll(name):
    """True for an embassy task's poll function."""
    return name.startswith("<embassy_executor::raw::TaskStorage<") and name.endswith(">::poll")


def parse(elf, objdump, disasm=None):
    if disasm:
        with open(disasm, encoding="utf-8") as f:
            out = f.read()
    else:
        out = subprocess.run([objdump, "-d", "--no-show-raw-insn", "-C", elf],
                             check=True, capture_output=True, text=True).stdout
    names, frames, refs = {}, {}, {}
    cur = None
    state = 0
    lit = None
    for line in out.splitlines():
        m = FUNC.match(line)
        if m:
            cur = int(m.group(1), 16)
            names[cur] = m.group(2)
            frames[cur] = 0
            refs[cur] = set()
            state, lit, seen = 0, None, 0
            continue
        if cur is None:
            continue
        if state == 0:
            m = ENTRY.search(line)
            if m:
                frames[cur] = int(m.group(1), 0)
                state, seen = 1, 0
        elif state == 1:
            seen += 1
            m = L32R.search(line)
            if m:
                lit = int(m.group(1), 16)
            if "movsp\ta1" in line and lit is not None:
                frames[cur] += lit
                state = 2
            elif seen >= 4:
                state = 2
        for rx in (L32R, CALL):
            for m in rx.finditer(line):
                refs[cur].add(int(m.group(1), 16))
    # Task poll functions are entered only from the executor (spawning
    # stores their address), so they are roots, never callees.
    polls = {a for a, n in names.items() if is_poll(n)}
    for f in refs:
        refs[f] = {r for r in refs[f] if r in names and r != f and r not in polls}
    return names, frames, refs


def deepest(root, frames, refs, memo, stack, cycles):
    """Deepest chain from `root`: (bytes, [functions])."""
    if root in memo:
        return memo[root]
    stack.add(root)
    best = (0, [])
    for callee in refs[root]:
        if callee in stack:
            cycles.add((root, callee))
            continue
        depth = deepest(callee, frames, refs, memo, stack, cycles)
        if depth[0] > best[0]:
            best = depth
    stack.discard(root)
    result = (frames[root] + best[0], [root] + best[1])
    memo[root] = result
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("elf")
    ap.add_argument("--objdump", default="xtensa-esp32s3-elf-objdump")
    ap.add_argument("--disasm", help="read this saved `objdump -d --no-show-raw-insn -C` "
                    "output instead of running objdump on ELF")
    ap.add_argument("--root", action="append", default=[],
                    help="extra root: substring of a function name")
    ap.add_argument("--chain", type=int, default=8, help="chain entries shown per root")
    ap.add_argument("--frames", type=int, default=0, help="also list the N largest frames")
    args = ap.parse_args()

    names, frames, refs = parse(args.elf, args.objdump, args.disasm)
    roots = [a for a, n in names.items() if is_poll(n)]
    for sub in args.root:
        roots += [a for a, n in names.items() if sub in n and a not in roots]
    memo, cycles = {}, set()
    rows = []
    for r in roots:
        total, chain = deepest(r, frames, refs, memo, set(), cycles)
        rows.append((total, r, chain))
    for total, r, chain in sorted(rows, reverse=True):
        name = re.sub(r"^<embassy_executor::raw::TaskStorage<(.*)>>::poll$", r"\1", names[r])
        name = re.sub(r"::__(\w+)_task::__\w+_inner_function::\{closure#0\}", r"::\1", name)
        print(f"{total:6d}  {name[:110]}")
        for f in chain[: args.chain]:
            print(f"        {frames[f]:6d}  {names[f][:100]}")
    if cycles:
        print(f"\n{len(cycles)} recursive edge(s) cut, e.g. "
              f"{names[next(iter(cycles))[0]][:60]} -> {names[next(iter(cycles))[1]][:60]}")
    if args.frames:
        print("\nlargest frames:")
        for a in sorted(frames, key=frames.get, reverse=True)[: args.frames]:
            print(f"{frames[a]:6d}  {names[a][:120]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
