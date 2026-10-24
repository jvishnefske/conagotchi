#!/usr/bin/env python3
"""Regenerates the littlefs v2 fixture images used by `lfs/tests.rs`.

Requires littlefs-python (tested with 0.19.0, littlefs 2.11). Run it from
any directory; the images are written next to this script:

    python3 -m venv venv && venv/bin/pip install littlefs-python
    venv/bin/python gen_fixtures.py

The geometry matches the stock badge `vfs` partition (MicroPython lfs2):
4096-byte blocks, name_max 63, attr_max 1022, 32-byte read/prog size,
128-byte cache, on-disk version 2.1. Only the block count is reduced, to keep
the images small.

Every key in here is FAKE. The real radio key never goes into this repo.
"""

import os
import struct

from littlefs import LittleFS

HERE = os.path.dirname(os.path.abspath(__file__))
BLOCK = 4096

# Fake keys: 32 bytes of obvious test patterns, as hex text.
FAKE_KEY_A = "00112233445566778899aabbccddeeff" "0123456789abcdeffedcba9876543210"
FAKE_KEY_B = "deadbeef" * 8


def new_fs(blocks):
    return LittleFS(
        block_size=BLOCK,
        block_count=blocks,
        read_size=32,
        prog_size=32,
        cache_size=128,
        lookahead_size=32,
        name_max=63,
        attr_max=1022,
        disk_version=0x00020001,
    )


def write(fs, name, data):
    mode = "wb" if isinstance(data, bytes) else "w"
    with fs.open(name, mode) as f:
        f.write(data)


def pattern(n, mod):
    return bytes(i % mod for i in range(n))


def save(fs, name):
    path = os.path.join(HERE, name)
    with open(path, "wb") as f:
        f.write(bytes(fs.context.buffer))
    print("wrote", name, len(fs.context.buffer), "bytes")
    return bytes(fs.context.buffer)


def revs(img):
    return tuple(struct.unpack_from("<I", img, b * BLOCK)[0] for b in (0, 1))


def key_inline():
    """Inline fake key with surrounding whitespace, plus non-inline files."""
    fs = new_fs(8)
    write(fs, "boot.py", "# fake boot\n")
    write(fs, "radio_key.txt", " \t" + FAKE_KEY_A + "\r\n")
    write(fs, "big.txt", pattern(1000, 251))    # CTZ, single block
    write(fs, "huge.bin", pattern(6000, 253))   # CTZ, two blocks
    fs.mkdir("data")
    write(fs, "short_key.txt", "0011223344556677" "8899aabbccddeeff\n")  # 16 bytes
    write(fs, "bad_key.txt", "not hex at all\n")
    write(fs, "empty.txt", "")
    return save(fs, "key_inline.img")


def key_split():
    """Root directory split over several metadata pairs (hardtails)."""
    fs = new_fs(16)
    for i in range(120):
        name = "f%03d_" % i + "x" * 58   # 63-byte names
        write(fs, name, "")
    write(fs, "radio_key.txt", FAKE_KEY_B + "\n")
    return save(fs, "key_split.img")


def key_rewrite():
    """Both blocks of the root pair hold valid commits: the older block has
    key A, the newer one key B."""
    fs = new_fs(8)
    write(fs, "radio_key.txt", FAKE_KEY_A + "\n")
    start = revs(bytes(fs.context.buffer))
    for i in range(400):
        write(fs, "filler.txt", "%04d" % i + "y" * 60)
        now = revs(bytes(fs.context.buffer))
        if now != start and 0xFFFFFFFF not in now:
            break
    else:
        raise SystemExit("root pair never compacted")
    before = revs(bytes(fs.context.buffer))
    write(fs, "radio_key.txt", FAKE_KEY_B + "\n")
    after = revs(bytes(fs.context.buffer))
    if before != after:
        raise SystemExit("key B write compacted the pair; adjust the loop")
    return save(fs, "key_rewrite.img")


def key_missing():
    """A valid filesystem without radio_key.txt."""
    fs = new_fs(8)
    write(fs, "boot.py", "# fake boot\n")
    write(fs, "radio_key.bak", FAKE_KEY_A + "\n")
    return save(fs, "key_missing.img")


def corrupt(img):
    """key_inline.img with the first commit of both root blocks damaged."""
    buf = bytearray(img)
    for b in (0, 1):
        buf[b * BLOCK + 8] ^= 0x55
    with open(os.path.join(HERE, "corrupt.img"), "wb") as f:
        f.write(buf)
    print("wrote corrupt.img", len(buf), "bytes")


if __name__ == "__main__":
    corrupt(key_inline())
    key_split()
    key_rewrite()
    key_missing()
