//! Read-only lookup of one file in the root directory of a littlefs v2
//! image, and parsing of the stock `radio_key.txt`.
//!
//! The stock MicroPython firmware keeps its files on a littlefs v2 (on-disk
//! version 2.1) filesystem in the `vfs` partition: 4096-byte blocks,
//! `name_max` 63. On first boot our firmware reads the stock radio key from
//! there ([`read_root_file`] + [`parse_radio_key`]) and copies it into its own
//! `conag` partition. Nothing here writes; the stock filesystem stays intact.
//!
//! The format follows littlefs `SPEC.md` and the reference `lfs_dir_fetchmatch`
//! / `lfs_dir_getslice`:
//!
//! - A metadata pair is two blocks. Each starts with a little-endian 32-bit
//!   revision count; the block with the larger count (sequence comparison) is
//!   tried first, the other one if the first holds no valid commit.
//! - Tags are big-endian 32-bit words, each XORed with the previous tag
//!   (the first with `0xFFFF_FFFF`): valid bit, 11-bit type, 10-bit id,
//!   10-bit length (`0x3FF` = deleted, no data).
//! - Commits end in a CRC tag (type `0x500`..`0x5FE`) holding the CRC-32
//!   (reflected `0x04C11DB7`, initial `0xFFFF_FFFF`, no final XOR) of every
//!   byte since the start of the commit. CRCs are verified: tags after the
//!   last commit with a matching CRC (a torn write) are ignored, exactly like
//!   littlefs does. The low type bit of a CRC tag flips the expected valid
//!   bit of the next commit. `0x5FF` (FCRC, written by littlefs 2.6+) is an
//!   ordinary tag.
//! - The file name is looked up by a forward pass over the valid commits
//!   (tracking ids through create/delete splices); its struct tag is then
//!   found by walking the tags backwards from the last commit, so the newest
//!   value wins.
//! - A directory too large for one pair continues in the pair named by its
//!   hardtail (tail tag with the split bit). Softtails link to the *next*
//!   directory and are not followed. At most [`MAX_DIR_PAIRS`] pairs are
//!   visited.
//!
//! Supported file layouts: inline files (struct type `0x201`) and CTZ files
//! (`0x202`) that fit in their first block. Larger CTZ skip-list files give
//! [`LfsError::NotSupported`]; a radio key is ~65 bytes and always inline on
//! the stock geometry (inline limit 128 bytes).
//!
//! Not implemented, by design: pending moves recorded in the global state
//! (a rename interrupted by power loss may make a file visible under both
//! names; either copy holds the same data), custom attributes, and paths with
//! directories.
//!
//! Every loop is bounded: tags per block by `block_size / 4`, tag data by the
//! 10-bit length, pairs by [`MAX_DIR_PAIRS`]. No allocation, no recursion.

use crate::oz::sign::{KEY_LEN, RadioKey};

#[cfg(test)]
mod tests;

/// Block reads from the device holding the filesystem.
pub trait ReadBlock {
    /// Device error.
    type Error;
    /// Fills `buf` with the bytes at `off` within `block`.
    fn read(&mut self, block: u32, off: u32, buf: &mut [u8]) -> Result<(), Self::Error>;
}

/// A whole filesystem image in memory (tests, host tools).
#[derive(Debug, Clone, Copy)]
pub struct SliceDevice<'a> {
    image: &'a [u8],
    block_size: u32,
}

/// A read outside a [`SliceDevice`] image.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OutOfRange;

impl<'a> SliceDevice<'a> {
    /// Wraps `image`, a sequence of `block_size`-byte blocks.
    pub fn new(image: &'a [u8], block_size: u32) -> SliceDevice<'a> {
        SliceDevice { image, block_size }
    }
}

impl ReadBlock for SliceDevice<'_> {
    type Error = OutOfRange;

    fn read(&mut self, block: u32, off: u32, buf: &mut [u8]) -> Result<(), OutOfRange> {
        let start = u64::from(block) * u64::from(self.block_size) + u64::from(off);
        let start = usize::try_from(start).map_err(|_| OutOfRange)?;
        let end = start.checked_add(buf.len()).ok_or(OutOfRange)?;
        let src = self.image.get(start..end).ok_or(OutOfRange)?;
        buf.copy_from_slice(src);
        Ok(())
    }
}

/// Filesystem geometry known to the caller (from the partition table).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Geometry {
    /// Erase block size in bytes; must match the superblock.
    pub block_size: u32,
    /// Blocks available on the device; the superblock may not claim more.
    pub block_count: u32,
}

impl Geometry {
    /// The stock badge `vfs` partition: 0x202000..0x760000, 4096-byte blocks.
    pub const STOCK_VFS: Geometry = Geometry { block_size: 4096, block_count: 0x55E };
}

/// Why a lookup failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LfsError<E> {
    /// The device read failed.
    Io(E),
    /// The geometry is unusable (block size not a multiple of 4 or below
    /// 128 bytes, fewer than 2 blocks) or disagrees with the superblock.
    BadGeometry,
    /// The root pair holds no littlefs superblock.
    NotLittlefs,
    /// The on-disk major version is not 2 (the value read is attached).
    UnsupportedVersion(u32),
    /// Neither block of the metadata pair starting at this block holds a
    /// valid commit, or a pointer leads outside the device.
    Corrupt(u32),
    /// No such file in the root directory.
    NotFound,
    /// The name exists but is a directory (or the superblock entry).
    NotAFile,
    /// The name is empty, longer than 255 bytes or contains `/`.
    BadName,
    /// The file is larger than the caller's buffer (its size is attached).
    TooBig(u32),
    /// A CTZ file spanning more than one block.
    NotSupported,
    /// The root directory spans more than [`MAX_DIR_PAIRS`] metadata pairs.
    TooManyPairs,
}

/// Most metadata pairs of one directory visited (hardtail hops + 1).
pub const MAX_DIR_PAIRS: usize = 64;
/// Longest name littlefs allows (`LFS_NAME_MAX`).
pub const NAME_MAX: usize = 255;

const INVALID: u32 = 0x8000_0000;
const NO_TAG: u32 = 0xFFFF_FFFF;
const TYPE_NAME: u32 = 0x000;
const TYPE_REG: u32 = 0x001;
const TYPE_SUPERBLOCK: u32 = 0x0FF;
const TYPE_STRUCT: u32 = 0x200;
const TYPE_INLINE: u32 = 0x201;
const TYPE_CTZ: u32 = 0x202;
const TYPE_SPLICE: u32 = 0x400;
const TYPE_CREATE: u32 = 0x401;
const TYPE_DELETE: u32 = 0x4FF;
const TYPE_TAIL: u32 = 0x600;
const TYPE_CCRC: u32 = 0x500;
const SUPERBLOCK_LEN: u32 = 24;
const CHUNK: usize = 32;

const fn mktag(kind: u32, id: u32, size: u32) -> u32 {
    (kind << 20) | (id << 10) | size
}
const fn type1(t: u32) -> u32 {
    (t >> 20) & 0x700
}
const fn type2(t: u32) -> u32 {
    (t >> 20) & 0x780
}
const fn type3(t: u32) -> u32 {
    (t >> 20) & 0x7FF
}
const fn chunk(t: u32) -> u32 {
    (t >> 20) & 0xFF
}
const fn id(t: u32) -> u32 {
    (t >> 10) & 0x3FF
}
const fn size(t: u32) -> u32 {
    t & 0x3FF
}
const fn is_delete(t: u32) -> bool {
    t & 0x3FF == 0x3FF
}
/// Bytes the tag occupies on disk, header included.
const fn dsize(t: u32) -> u32 {
    4 + size(t.wrapping_add(is_delete(t) as u32))
}
/// The id adjustment of a create (+1) / delete (-1) splice, as a tag delta.
const fn splice_delta(t: u32) -> u32 {
    ((chunk(t) as u8 as i8) as i32 as u32) << 10
}

/// littlefs CRC-32: reflected `0x04C11DB7`, nibble table, no final XOR.
fn crc32(mut crc: u32, data: &[u8]) -> u32 {
    const TABLE: [u32; 16] = [
        0x0000_0000, 0x1DB7_1064, 0x3B6E_20C8, 0x26D9_30AC, 0x76DC_4190, 0x6B6B_51F4,
        0x4DB2_6158, 0x5005_713C, 0xEDB8_8320, 0xF00F_9344, 0xD6D6_A3E8, 0xCB61_B38C,
        0x9B64_C2B0, 0x86D3_D2D4, 0xA00A_E278, 0xBDBD_F21C,
    ];
    for &b in data {
        crc = (crc >> 4) ^ TABLE[((crc ^ u32::from(b)) & 0xF) as usize];
        crc = (crc >> 4) ^ TABLE[((crc ^ u32::from(b >> 4)) & 0xF) as usize];
    }
    crc
}

fn read_u32<R: ReadBlock>(dev: &mut R, block: u32, off: u32) -> Result<[u8; 4], LfsError<R::Error>> {
    let mut w = [0u8; 4];
    dev.read(block, off, &mut w).map_err(LfsError::Io)?;
    Ok(w)
}

/// State of a metadata block after its last valid commit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct MDir {
    /// Block holding the commits.
    block: u32,
    /// End of the last valid commit.
    off: u32,
    /// Last tag of that commit, with the valid-bit phase applied.
    etag: u32,
    /// Tail pair, if any.
    tail: Option<[u32; 2]>,
    /// The tail is a hardtail: this directory continues there.
    split: bool,
    /// Name tag matching the searched name (id adjusted), or `NO_TAG`.
    found: u32,
}

/// Forward pass over `block`: verifies the commit CRCs and, if `name` is
/// given, tracks the tag of the matching name entry. `None` if the block has
/// no valid commit.
fn fetch_block<R: ReadBlock>(
    dev: &mut R,
    geo: Geometry,
    block: u32,
    name: &[u8],
) -> Result<Option<MDir>, LfsError<R::Error>> {
    let rev = read_u32(dev, block, 0)?;
    let mut crc = crc32(NO_TAG, &rev);
    let mut off = 0u32;
    let mut ptag = NO_TAG;
    let mut best = NO_TAG;
    let mut tail = None;
    let mut split = false;
    let mut done: Option<MDir> = None;

    for _ in 0..geo.block_size / 4 {
        off += dsize(ptag);
        if off.saturating_add(4) > geo.block_size {
            break;
        }
        let raw = read_u32(dev, block, off)?;
        crc = crc32(crc, &raw);
        let tag = u32::from_be_bytes(raw) ^ ptag;
        if tag & INVALID != 0 || off.saturating_add(dsize(tag)) > geo.block_size {
            break;
        }
        ptag = tag;

        if type2(tag) == TYPE_CCRC {
            let stored = u32::from_le_bytes(read_u32(dev, block, off + 4)?);
            if stored != crc {
                break;
            }
            ptag ^= (chunk(tag) & 1) << 31;
            done = Some(MDir { block, off: off + dsize(tag), etag: ptag, tail, split, found: best });
            crc = NO_TAG;
            continue;
        }

        // CRC the data, comparing it with `name` and keeping the first 8
        // bytes (a tail pair) on the way.
        let len = dsize(tag) - 4;
        let is_name = type1(tag) == TYPE_NAME;
        let mut same = is_name && len as usize == name.len();
        let mut head = [0u8; 8];
        let mut buf = [0u8; CHUNK];
        let mut pos = 0u32;
        for _ in 0..len.div_ceil(CHUNK as u32) {
            let n = (len - pos).min(CHUNK as u32) as usize;
            let part = &mut buf[..n];
            dev.read(block, off + 4 + pos, part).map_err(LfsError::Io)?;
            crc = crc32(crc, part);
            let p = pos as usize;
            if same {
                same = name.get(p..p + n) == Some(&part[..]);
            }
            for (i, &b) in part.iter().enumerate() {
                if let Some(h) = head.get_mut(p + i) {
                    *h = b;
                }
            }
            pos += n as u32;
        }

        if type1(tag) == TYPE_SPLICE {
            if tag == mktag(TYPE_DELETE, 0, 0) | (mktag(0, 0x3FF, 0) & best) {
                best |= INVALID;
            } else if best != NO_TAG && id(tag) <= id(best) {
                best = best.wrapping_add(splice_delta(tag));
            }
        } else if type1(tag) == TYPE_TAIL {
            split = chunk(tag) & 1 != 0;
            let [a0, a1, a2, a3, b0, b1, b2, b3] = head;
            tail = Some([u32::from_le_bytes([a0, a1, a2, a3]), u32::from_le_bytes([b0, b1, b2, b3])]);
        }

        if is_name && !name.is_empty() {
            let key = mktag(0x7FF, 0x3FF, 0);
            if same {
                best = tag;
            } else if key & tag == key & best {
                // Our entry's id was reused for another name.
                best = NO_TAG;
            }
        }
    }
    Ok(done)
}

/// Fetches the metadata pair `pair`, newest valid block first.
fn fetch_pair<R: ReadBlock>(
    dev: &mut R,
    geo: Geometry,
    pair: [u32; 2],
    name: &[u8],
) -> Result<MDir, LfsError<R::Error>> {
    if pair[0] >= geo.block_count || pair[1] >= geo.block_count {
        return Err(LfsError::Corrupt(pair[0]));
    }
    let r0 = u32::from_le_bytes(read_u32(dev, pair[0], 0)?);
    let r1 = u32::from_le_bytes(read_u32(dev, pair[1], 0)?);
    let order = if (r1.wrapping_sub(r0) as i32) > 0 { [pair[1], pair[0]] } else { pair };
    for block in order {
        if let Some(dir) = fetch_block(dev, geo, block, name)? {
            return Ok(dir);
        }
    }
    Err(LfsError::Corrupt(pair[0]))
}

/// Newest tag matching `gtag` under `gmask`, walking backwards from the last
/// commit: `(tag, data offset)`, or `None` if absent or deleted.
fn get<R: ReadBlock>(
    dev: &mut R,
    geo: Geometry,
    dir: &MDir,
    gmask: u32,
    gtag: u32,
) -> Result<Option<(u32, u32)>, LfsError<R::Error>> {
    let mut off = dir.off;
    let mut ntag = dir.etag;
    let mut gdiff = 0u32;
    for _ in 0..geo.block_size / 4 {
        if off < 4 + dsize(ntag) {
            break;
        }
        off -= dsize(ntag);
        let tag = ntag;
        ntag = (u32::from_be_bytes(read_u32(dev, dir.block, off)?) ^ tag) & !INVALID;

        if id(gmask) != 0 && type1(tag) == TYPE_SPLICE && id(tag) <= id(gtag.wrapping_sub(gdiff)) {
            if tag == mktag(TYPE_CREATE, 0, 0) | (mktag(0, 0x3FF, 0) & gtag.wrapping_sub(gdiff)) {
                return Ok(None);
            }
            gdiff = gdiff.wrapping_add(splice_delta(tag));
        }
        if gmask & tag == gmask & gtag.wrapping_sub(gdiff) {
            if is_delete(tag) {
                return Ok(None);
            }
            return Ok(Some((tag.wrapping_add(gdiff), off + 4)));
        }
    }
    Ok(None)
}

/// Checks the superblock entry of the root pair against `geo`.
fn check_superblock<R: ReadBlock>(dev: &mut R, geo: Geometry, root: &MDir) -> Result<(), LfsError<R::Error>> {
    let mask = mktag(0x7FF, 0x3FF, 0);
    let Some((_, off)) = get(dev, geo, root, mask, mktag(TYPE_SUPERBLOCK, 0, 8))? else {
        return Err(LfsError::NotLittlefs);
    };
    let mut magic = [0u8; 8];
    dev.read(root.block, off, &mut magic).map_err(LfsError::Io)?;
    if &magic != b"littlefs" {
        return Err(LfsError::NotLittlefs);
    }
    let Some((tag, off)) = get(dev, geo, root, mask, mktag(TYPE_INLINE, 0, SUPERBLOCK_LEN))? else {
        return Err(LfsError::NotLittlefs);
    };
    if size(tag) < 12 {
        return Err(LfsError::NotLittlefs);
    }
    let mut sb = [0u8; 12];
    dev.read(root.block, off, &mut sb).map_err(LfsError::Io)?;
    let [v0, v1, v2, v3, s0, s1, s2, s3, c0, c1, c2, c3] = sb;
    let version = u32::from_le_bytes([v0, v1, v2, v3]);
    if version >> 16 != 2 {
        return Err(LfsError::UnsupportedVersion(version));
    }
    let block_size = u32::from_le_bytes([s0, s1, s2, s3]);
    let block_count = u32::from_le_bytes([c0, c1, c2, c3]);
    if block_size != geo.block_size || block_count > geo.block_count || block_count < 2 {
        return Err(LfsError::BadGeometry);
    }
    Ok(())
}

/// Reads the file `name` from the root directory into `out`; returns its
/// length.
///
/// # Errors
///
/// See [`LfsError`]: device errors, a damaged or foreign filesystem, a
/// missing file or a directory, a file larger than `out`, or a multi-block
/// CTZ file.
pub fn read_root_file<R: ReadBlock>(
    dev: &mut R,
    geo: Geometry,
    name: &[u8],
    out: &mut [u8],
) -> Result<usize, LfsError<R::Error>> {
    if geo.block_size < 128 || !geo.block_size.is_multiple_of(4) || geo.block_count < 2 {
        return Err(LfsError::BadGeometry);
    }
    if name.is_empty() || name.len() > NAME_MAX || name.contains(&b'/') {
        return Err(LfsError::BadName);
    }
    let mut pair = [0u32, 1];
    for hop in 0..MAX_DIR_PAIRS {
        let dir = fetch_pair(dev, geo, pair, name)?;
        if hop == 0 {
            check_superblock(dev, geo, &dir)?;
        }
        if dir.found & INVALID == 0 {
            if type3(dir.found) != TYPE_REG {
                return Err(LfsError::NotAFile);
            }
            return read_struct(dev, geo, &dir, id(dir.found), out);
        }
        match (dir.split, dir.tail) {
            (true, Some(next)) => pair = next,
            _ => return Err(LfsError::NotFound),
        }
    }
    Err(LfsError::TooManyPairs)
}

/// Reads the contents of entry `fid` of `dir` through its struct tag.
fn read_struct<R: ReadBlock>(
    dev: &mut R,
    geo: Geometry,
    dir: &MDir,
    fid: u32,
    out: &mut [u8],
) -> Result<usize, LfsError<R::Error>> {
    let Some((tag, off)) = get(dev, geo, dir, mktag(0x700, 0x3FF, 0), mktag(TYPE_STRUCT, fid, 8))? else {
        return Err(LfsError::Corrupt(dir.block));
    };
    let (block, off, len) = match type3(tag) {
        TYPE_INLINE => (dir.block, off, size(tag)),
        TYPE_CTZ => {
            let mut ctz = [0u8; 8];
            dev.read(dir.block, off, &mut ctz).map_err(LfsError::Io)?;
            let [h0, h1, h2, h3, s0, s1, s2, s3] = ctz;
            let head = u32::from_le_bytes([h0, h1, h2, h3]);
            let len = u32::from_le_bytes([s0, s1, s2, s3]);
            if len > geo.block_size {
                return Err(LfsError::NotSupported);
            }
            if head >= geo.block_count {
                return Err(LfsError::Corrupt(dir.block));
            }
            (head, 0, len)
        }
        _ => return Err(LfsError::NotSupported),
    };
    let n = len as usize;
    let dst = out.get_mut(..n).ok_or(LfsError::TooBig(len))?;
    if n > 0 {
        dev.read(block, off, dst).map_err(LfsError::Io)?;
    }
    Ok(n)
}

/// Why `radio_key.txt` contents are not a usable key.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KeyError {
    /// Not an even number of hex digits (whitespace aside).
    NotHex,
    /// Fewer than 16 bytes (the stock firmware ignores such keys too).
    TooShort(usize),
    /// More than [`KEY_LEN`] bytes: the stock firmware would use it, but
    /// [`RadioKey`] holds 32 bytes (see [`parse_radio_key`]).
    TooLong(usize),
}

/// Parses the text of the stock `radio_key.txt` like stock `load_key`:
/// surrounding whitespace is stripped, the rest is hex (either case;
/// whitespace between byte pairs is skipped, as CPython `bytes.fromhex`
/// does), and at least 16 bytes are required.
///
/// Keys shorter than [`KEY_LEN`] are zero-padded by [`RadioKey::from_bytes`].
/// That is exact for HMAC-SHA256, which pads every key of up to 64 bytes
/// with zeros to the 64-byte block, so `K` and `K || 0…` give the same tags.
/// Keys longer than 32 bytes are rejected ([`KeyError::TooLong`]).
///
/// Unlike [`RadioKey::from_hex`] this also accepts whitespace between byte
/// pairs and says why a key was refused.
///
/// # Errors
///
/// [`KeyError`] when the text is not hex or has an unusable length.
pub fn parse_radio_key(text: &[u8]) -> Result<RadioKey, KeyError> {
    let text = text.trim_ascii();
    let mut key = [0u8; KEY_LEN];
    let mut len = 0usize;
    let mut high: Option<u8> = None;
    for &c in text {
        if c.is_ascii_whitespace() {
            if high.is_some() {
                return Err(KeyError::NotHex);
            }
            continue;
        }
        let nibble = match c {
            b'0'..=b'9' => c - b'0',
            b'a'..=b'f' => c - b'a' + 10,
            b'A'..=b'F' => c - b'A' + 10,
            _ => return Err(KeyError::NotHex),
        };
        match high.take() {
            None => high = Some(nibble),
            Some(h) => {
                if let Some(slot) = key.get_mut(len) {
                    *slot = (h << 4) | nibble;
                }
                len += 1;
            }
        }
    }
    if high.is_some() {
        return Err(KeyError::NotHex);
    }
    if len < 16 {
        return Err(KeyError::TooShort(len));
    }
    let bytes = key.get(..len).ok_or(KeyError::TooLong(len))?;
    RadioKey::from_bytes(bytes).map_err(|_| KeyError::TooLong(len))
}

/// Name of the stock key file in the `vfs` root.
pub const RADIO_KEY_FILE: &[u8] = b"radio_key.txt";
/// Largest key file read (a 64-byte key in hex with whitespace).
pub const MAX_KEY_FILE: usize = 160;

/// Why the stock key could not be imported.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImportError<E> {
    /// Reading the filesystem failed.
    Fs(LfsError<E>),
    /// The file exists but holds no usable key.
    Key(KeyError),
}

/// Reads and parses [`RADIO_KEY_FILE`] from the filesystem on `dev`.
///
/// # Errors
///
/// [`ImportError::Fs`] if the file cannot be read (including
/// [`LfsError::NotFound`]), [`ImportError::Key`] if its text is unusable.
pub fn import_radio_key<R: ReadBlock>(dev: &mut R, geo: Geometry) -> Result<RadioKey, ImportError<R::Error>> {
    let mut buf = [0u8; MAX_KEY_FILE];
    let n = read_root_file(dev, geo, RADIO_KEY_FILE, &mut buf).map_err(ImportError::Fs)?;
    let text = buf.get(..n).ok_or(ImportError::Key(KeyError::NotHex))?;
    parse_radio_key(text).map_err(ImportError::Key)
}
