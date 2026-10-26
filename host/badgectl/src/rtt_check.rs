//! Validation of the `_SEGGER_RTT` control block (functional core).
//!
//! The block, as rtt-target 0.6 lays it out on a 32-bit target, is a
//! 24-byte header (16-byte ID `"SEGGER RTT"` NUL padded, then the up and
//! down channel counts as `u32`) followed by one 24-byte descriptor per
//! up channel and then per down channel: name pointer, buffer pointer,
//! size, write offset, read offset and flags.
//!
//! [`ControlBlock::parse`] decodes the bytes read from the target and
//! [`check`] lists every field that is not plausible as a [`Finding`]:
//!
//! - [`Severity::Corrupt`]: the ID string, channel counts outside
//!   1..=[`MAX_CHANNELS`], a buffer pointer or `pointer + size` outside RAM,
//!   a zero size, write or read offsets not below the size, flag bits other
//!   than the two mode bits, a name pointer outside RAM and flash-mapped
//!   rodata, and buffers overlapping each other or the control block.
//! - [`Severity::Differs`]: plausible but unlike the firmware's layout
//!   ([`FIRMWARE_LAYOUT`], from `firmware/src/rtt.rs`): channel counts,
//!   names or sizes. A firmware change to the channel table shows up here
//!   rather than as corruption.
//!
//! The firmware has no guard of its own around the control block; the
//! nearest one is esp-hal's stack guard word, which `badgectl rtt-check`
//! reports alongside (see [`crate::stack`]).

use std::fmt::Write as _;
use std::ops::Range;

use serde_json::{Value, json};

/// The ID the block starts with.
pub const ID: [u8; 16] = *b"SEGGER RTT\0\0\0\0\0\0";
/// Header bytes (ID and the two counts).
pub const HEADER_LEN: usize = 24;
/// Bytes per channel descriptor.
pub const CHANNEL_LEN: usize = 24;
/// Most channels per direction considered sane.
pub const MAX_CHANNELS: u32 = 16;
/// Bytes to read to cover the header and [`MAX_CHANNELS`] each way.
pub const READ_LEN: usize = HEADER_LEN + 2 * MAX_CHANNELS as usize * CHANNEL_LEN;
/// Longest channel name read from the target.
pub const NAME_MAX: usize = 32;
/// Mode bits of the flags word (`NoBlockSkip`, `NoBlockTrim`, `BlockIfFull`).
const MODE_MASK: u32 = 0b11;

/// Where buffers and names may live.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MemoryMap {
    /// Data RAM: buffers and the block itself.
    pub ram: Range<u64>,
    /// Extra ranges a name may point into (flash-mapped rodata).
    pub rodata: Range<u64>,
}

/// ESP32-S3 internal SRAM on the data bus, and the DROM rodata window.
pub const ESP32S3: MemoryMap = MemoryMap { ram: 0x3FC8_8000..0x3FD0_0000, rodata: 0x3C00_0000..0x3E00_0000 };

/// Expected name and size of one channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChannelSpec {
    /// Channel name.
    pub name: &'static str,
    /// Buffer size in bytes.
    pub size: u32,
}

/// The firmware's channel table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Layout {
    /// Up channels in order.
    pub up: &'static [ChannelSpec],
    /// Down channels in order.
    pub down: &'static [ChannelSpec],
}

/// The channels `firmware/src/rtt.rs` sets up.
pub const FIRMWARE_LAYOUT: Layout = Layout {
    up: &[ChannelSpec { name: "defmt", size: 2048 }, ChannelSpec { name: "proto-up", size: 4096 }],
    down: &[ChannelSpec { name: "proto-down", size: 1024 }],
};

/// One channel descriptor as read.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Channel {
    /// Pointer to the NUL-terminated name.
    pub name_ptr: u32,
    /// Pointer to the ring buffer.
    pub buffer: u32,
    /// Ring buffer size.
    pub size: u32,
    /// Write offset.
    pub write: u32,
    /// Read offset.
    pub read: u32,
    /// Flags (mode in bits 0..2).
    pub flags: u32,
}

/// Up or down.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// Target to host.
    Up,
    /// Host to target.
    Down,
}

impl Direction {
    fn as_str(self) -> &'static str {
        match self {
            Self::Up => "up",
            Self::Down => "down",
        }
    }
}

/// The decoded control block.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ControlBlock {
    /// ID bytes.
    pub id: [u8; 16],
    /// Up channel count as stored.
    pub max_up: u32,
    /// Down channel count as stored.
    pub max_down: u32,
    /// Up descriptors (at most [`MAX_CHANNELS`], fewer if the count is bad).
    pub up: Vec<Channel>,
    /// Down descriptors.
    pub down: Vec<Channel>,
}

/// The byte slice is shorter than the header.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error("RTT control block needs at least {HEADER_LEN} bytes, got {0}")]
pub struct ShortRead(pub usize);

fn word(bytes: &[u8], at: usize) -> u32 {
    let mut w = [0u8; 4];
    if let Some(src) = bytes.get(at..at + 4) {
        w.copy_from_slice(src);
    }
    u32::from_le_bytes(w)
}

fn channel_at(bytes: &[u8], at: usize) -> Option<Channel> {
    bytes.get(at..at + CHANNEL_LEN).map(|c| Channel {
        name_ptr: word(c, 0),
        buffer: word(c, 4),
        size: word(c, 8),
        write: word(c, 12),
        read: word(c, 16),
        flags: word(c, 20),
    })
}

impl ControlBlock {
    /// Decodes the header and as many descriptors as the counts (each
    /// clipped to [`MAX_CHANNELS`]) and `bytes` allow.
    pub fn parse(bytes: &[u8]) -> Result<Self, ShortRead> {
        if bytes.len() < HEADER_LEN {
            return Err(ShortRead(bytes.len()));
        }
        let mut id = [0u8; 16];
        id.copy_from_slice(bytes.get(..16).unwrap_or(&[0; 16]));
        let max_up = word(bytes, 16);
        let max_down = word(bytes, 20);
        let n_up = max_up.min(MAX_CHANNELS) as usize;
        let n_down = max_down.min(MAX_CHANNELS) as usize;
        let up: Vec<Channel> =
            (0..n_up).map_while(|i| channel_at(bytes, HEADER_LEN + i * CHANNEL_LEN)).collect();
        // Down descriptors follow all `max_up` up descriptors; with a bad up
        // count their position is unknown, so they are not decoded.
        let down = if max_up <= MAX_CHANNELS {
            (0..n_down).map_while(|i| channel_at(bytes, HEADER_LEN + (n_up + i) * CHANNEL_LEN)).collect()
        } else {
            Vec::new()
        };
        Ok(Self { id, max_up, max_down, up, down })
    }

    /// Bytes the block occupies according to its counts (clipped).
    pub fn len(&self) -> u64 {
        (HEADER_LEN + (self.max_up.min(MAX_CHANNELS) + self.max_down.min(MAX_CHANNELS)) as usize * CHANNEL_LEN)
            as u64
    }

    /// Descriptors with their direction and index.
    pub fn channels(&self) -> impl Iterator<Item = (Direction, usize, &Channel)> {
        let up = self.up.iter().enumerate().map(|(i, c)| (Direction::Up, i, c));
        let down = self.down.iter().enumerate().map(|(i, c)| (Direction::Down, i, c));
        up.chain(down)
    }
}

/// How bad a finding is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Severity {
    /// Not a valid control block value.
    Corrupt,
    /// Valid, but not what the firmware sets up.
    Differs,
}

/// One questionable field.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Finding {
    /// Severity.
    pub severity: Severity,
    /// Field path, e.g. `up[1].size`.
    pub field: String,
    /// Value read, formatted.
    pub value: String,
    /// What was expected.
    pub expected: String,
}

fn finding(severity: Severity, field: impl Into<String>, value: impl Into<String>, expected: impl Into<String>) -> Finding {
    Finding { severity, field: field.into(), value: value.into(), expected: expected.into() }
}

/// Printable ID bytes (`\0` and non-ASCII escaped).
pub fn id_text(id: &[u8]) -> String {
    id.iter().flat_map(|b| std::ascii::escape_default(*b)).map(char::from).collect::<String>().replace("\\x00", "\\0")
}

/// Names read from the target for each channel, indexed like
/// [`ControlBlock::up`] / [`ControlBlock::down`]; `None` when unreadable.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Names {
    /// Up channel names.
    pub up: Vec<Option<String>>,
    /// Down channel names.
    pub down: Vec<Option<String>>,
}

impl Names {
    fn get(&self, dir: Direction, i: usize) -> Option<&str> {
        let list = match dir {
            Direction::Up => &self.up,
            Direction::Down => &self.down,
        };
        list.get(i).and_then(|n| n.as_deref())
    }
}

/// The name at the start of `bytes` (up to the first NUL), if it is
/// printable ASCII and NUL terminated within `bytes`.
pub fn c_name(bytes: &[u8]) -> Option<String> {
    let end = bytes.iter().position(|b| *b == 0)?;
    let name = bytes.get(..end)?;
    name.iter().all(|b| b.is_ascii_graphic() || *b == b' ').then(|| String::from_utf8_lossy(name).into_owned())
}

fn in_range(range: &Range<u64>, start: u64, len: u64) -> bool {
    start >= range.start && start.checked_add(len).is_some_and(|end| end <= range.end)
}

fn overlaps(a: &Range<u64>, b: &Range<u64>) -> bool {
    a.start < b.end && b.start < a.end
}

/// Every questionable field of `block`, read from `addr`.
pub fn check(block: &ControlBlock, addr: u64, map: &MemoryMap, layout: Option<&Layout>, names: &Names) -> Vec<Finding> {
    use Severity::{Corrupt, Differs};
    let mut out = Vec::new();
    if block.id != ID {
        out.push(finding(Corrupt, "id", format!("\"{}\"", id_text(&block.id)), format!("\"{}\"", id_text(&ID))));
    }
    for (field, count) in [("max_up_channels", block.max_up), ("max_down_channels", block.max_down)] {
        if !(1..=MAX_CHANNELS).contains(&count) {
            out.push(finding(Corrupt, field, count.to_string(), format!("1..={MAX_CHANNELS}")));
        }
    }
    if let Some(l) = layout {
        for (field, count, want) in
            [("max_up_channels", block.max_up, l.up.len()), ("max_down_channels", block.max_down, l.down.len())]
        {
            if (1..=MAX_CHANNELS).contains(&count) && count as usize != want {
                out.push(finding(Differs, field, count.to_string(), format!("{want} (firmware/src/rtt.rs)")));
            }
        }
    }
    let block_range = addr..addr + block.len();
    let mut buffers: Vec<(String, Range<u64>)> = Vec::new();
    for (dir, i, c) in block.channels() {
        let path = format!("{}[{i}]", dir.as_str());
        let buf = u64::from(c.buffer);
        let size = u64::from(c.size);
        let mut size_ok = false;
        if !in_range(&map.ram, buf, 1) {
            out.push(finding(Corrupt, format!("{path}.buffer"), format!("{buf:#010x}"), ram_text(map)));
        } else if size == 0 || !in_range(&map.ram, buf, size) {
            out.push(finding(
                Corrupt,
                format!("{path}.size"),
                size.to_string(),
                format!("1..={} (buffer {buf:#010x} must end in RAM)", map.ram.end - buf),
            ));
        } else {
            size_ok = true;
            let range = buf..buf + size;
            if overlaps(&range, &block_range) {
                out.push(finding(
                    Corrupt,
                    format!("{path}.buffer"),
                    format!("{buf:#010x}..{:#010x}", range.end),
                    format!("outside the control block {:#010x}..{:#010x}", block_range.start, block_range.end),
                ));
            }
            if let Some((other, _)) = buffers.iter().find(|(_, r)| overlaps(r, &range)) {
                out.push(finding(
                    Corrupt,
                    format!("{path}.buffer"),
                    format!("{buf:#010x}..{:#010x}", range.end),
                    format!("no overlap with {other}.buffer"),
                ));
            }
            buffers.push((path.clone(), range));
        }
        if size != 0 {
            for (field, v) in [("write", c.write), ("read", c.read)] {
                if v >= c.size {
                    out.push(finding(Corrupt, format!("{path}.{field}"), v.to_string(), format!("< size {}", c.size)));
                }
            }
        }
        if c.flags & !MODE_MASK != 0 || c.flags & MODE_MASK == MODE_MASK {
            out.push(finding(Corrupt, format!("{path}.flags"), format!("{:#x}", c.flags), "mode 0, 1 or 2, no other bits"));
        }
        let name_ptr = u64::from(c.name_ptr);
        let name_ok = in_range(&map.ram, name_ptr, 1) || in_range(&map.rodata, name_ptr, 1);
        if !name_ok {
            out.push(finding(Corrupt, format!("{path}.name"), format!("{name_ptr:#010x}"), "pointer into RAM or rodata"));
        }
        let spec = layout.and_then(|l| match dir {
            Direction::Up => l.up.get(i),
            Direction::Down => l.down.get(i),
        });
        if let Some(spec) = spec {
            if name_ok
                && let Some(name) = names.get(dir, i)
                && name != spec.name
            {
                out.push(finding(Differs, format!("{path}.name"), format!("\"{name}\""), format!("\"{}\"", spec.name)));
            }
            if size_ok && c.size != spec.size {
                out.push(finding(Differs, format!("{path}.size"), c.size.to_string(), spec.size.to_string()));
            }
        }
    }
    out
}

fn ram_text(map: &MemoryMap) -> String {
    format!("in RAM {:#010x}..{:#010x}", map.ram.start, map.ram.end)
}

/// True if any finding is [`Severity::Corrupt`].
pub fn is_corrupt(findings: &[Finding]) -> bool {
    findings.iter().any(|f| f.severity == Severity::Corrupt)
}

/// Human-readable rendering: the block, then `OK` or one line per finding.
pub fn render(block: &ControlBlock, addr: u64, names: &Names, findings: &[Finding]) -> String {
    let mut out = String::new();
    let _ = writeln!(
        out,
        "_SEGGER_RTT    {addr:#010x}: id \"{}\", {} up, {} down",
        id_text(&block.id),
        block.max_up,
        block.max_down
    );
    for (dir, i, c) in block.channels() {
        let name = names.get(dir, i).map_or_else(|| format!("{:#010x}", c.name_ptr), |n| format!("\"{n}\""));
        let _ = writeln!(
            out,
            "  {:<4} {i}  {name:<12} buffer {:#010x} size {:>5} write {:>5} read {:>5} flags {:#x}",
            dir.as_str(),
            c.buffer,
            c.size,
            c.write,
            c.read,
            c.flags
        );
    }
    if findings.is_empty() {
        out.push_str("rtt            OK");
    } else {
        let status = if is_corrupt(findings) { "CORRUPT" } else { "OK, differs from the firmware layout" };
        let _ = write!(out, "rtt            {status}");
        for f in findings {
            let tag = match f.severity {
                Severity::Corrupt => "corrupt",
                Severity::Differs => "differs",
            };
            let _ = write!(out, "\n  {tag:<8} {} = {} (expected {})", f.field, f.value, f.expected);
        }
    }
    out
}

/// JSON rendering.
pub fn to_json(block: &ControlBlock, addr: u64, names: &Names, findings: &[Finding]) -> Value {
    let channels: Vec<Value> = block
        .channels()
        .map(|(dir, i, c)| {
            json!({
                "dir": dir.as_str(), "index": i, "name": names.get(dir, i), "name_ptr": c.name_ptr,
                "buffer": c.buffer, "size": c.size, "write": c.write, "read": c.read, "flags": c.flags,
            })
        })
        .collect();
    let findings: Vec<Value> = findings
        .iter()
        .map(|f| {
            json!({
                "severity": match f.severity { Severity::Corrupt => "corrupt", Severity::Differs => "differs" },
                "field": f.field, "value": f.value, "expected": f.expected,
            })
        })
        .collect();
    json!({
        "addr": addr, "id": id_text(&block.id), "max_up": block.max_up, "max_down": block.max_down,
        "channels": channels, "ok": !findings.iter().any(|f| f["severity"] == "corrupt"), "findings": findings,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const ADDR: u64 = 0x3fcc_8cf4;

    fn put(bytes: &mut Vec<u8>, w: u32) {
        bytes.extend_from_slice(&w.to_le_bytes());
    }

    /// A block like the firmware's: defmt, proto-up / proto-down.
    fn good() -> Vec<u8> {
        let mut b = ID.to_vec();
        put(&mut b, 2);
        put(&mut b, 1);
        for (name, buf, size, flags) in
            [(0x3c00_1000, 0x3fcc_9000, 2048, 0), (0x3c00_1010, 0x3fcc_a000, 4096, 0), (0x3c00_1020, 0x3fcc_8000, 1024, 0)]
        {
            for w in [name, buf, size, 10, 5, flags] {
                put(&mut b, w);
            }
        }
        b
    }

    fn names() -> Names {
        Names { up: vec![Some("defmt".into()), Some("proto-up".into())], down: vec![Some("proto-down".into())] }
    }

    fn run(bytes: &[u8]) -> Vec<Finding> {
        let block = ControlBlock::parse(bytes).unwrap();
        check(&block, ADDR, &ESP32S3, Some(&FIRMWARE_LAYOUT), &names())
    }

    fn fields(f: &[Finding]) -> Vec<&str> {
        f.iter().map(|f| f.field.as_str()).collect()
    }

    fn set(bytes: &mut [u8], at: usize, w: u32) {
        bytes[at..at + 4].copy_from_slice(&w.to_le_bytes());
    }

    /// Offset of field `f` (0 name .. 5 flags) of descriptor `n` (up then down).
    fn at(n: usize, f: usize) -> usize {
        HEADER_LEN + n * CHANNEL_LEN + f * 4
    }

    #[test]
    fn parses_the_firmware_block() {
        let block = ControlBlock::parse(&good()).unwrap();
        assert_eq!((block.max_up, block.max_down, block.up.len(), block.down.len()), (2, 1, 2, 1));
        assert_eq!(block.up[1], Channel { name_ptr: 0x3c00_1010, buffer: 0x3fcc_a000, size: 4096, write: 10, read: 5, flags: 0 });
        assert_eq!(block.len(), 96);
        assert!(run(&good()).is_empty());
        let block = ControlBlock::parse(&good()).unwrap();
        let text = render(&block, ADDR, &names(), &[]);
        assert!(text.contains("id \"SEGGER RTT\\0\\0\\0\\0\\0\\0\", 2 up, 1 down"), "{text}");
        assert!(text.ends_with("rtt            OK"), "{text}");
        assert_eq!(to_json(&block, ADDR, &names(), &[])["ok"], true);
    }

    #[test]
    fn short_read_is_an_error() {
        assert_eq!(ControlBlock::parse(&[0; 10]), Err(ShortRead(10)));
    }

    #[test]
    fn overwritten_id_and_counts_are_corrupt() {
        let mut b = good();
        b[0..4].copy_from_slice(b"\xa5\xa5\xa5\xa5");
        set(&mut b, 16, 0xa5a5_a5a5);
        let f = run(&b);
        assert_eq!(fields(&f), ["id", "max_up_channels"]);
        assert!(is_corrupt(&f));
        assert_eq!(f[0].value, "\"\\xa5\\xa5\\xa5\\xa5ER RTT\\0\\0\\0\\0\\0\\0\"");
        // With a bad up count the down descriptors are not decoded.
        assert!(ControlBlock::parse(&b).unwrap().down.is_empty());
        let mut b = good();
        set(&mut b, 20, 0);
        assert_eq!(fields(&run(&b)), ["max_down_channels"]);
    }

    #[test]
    fn bad_descriptor_fields_are_named() {
        let mut b = good();
        set(&mut b, at(0, 1), 0x0000_0010); // up[0].buffer outside RAM
        set(&mut b, at(1, 2), 0x0100_0000); // up[1].size runs past RAM
        set(&mut b, at(2, 3), 1024); // down[0].write == size
        set(&mut b, at(2, 4), 4000); // down[0].read > size
        set(&mut b, at(2, 5), 0x8000_0003); // down[0].flags
        set(&mut b, at(2, 0), 0x1234); // down[0].name pointer
        let f = run(&b);
        assert_eq!(
            fields(&f),
            ["up[0].buffer", "up[1].size", "down[0].write", "down[0].read", "down[0].flags", "down[0].name"]
        );
        assert!(f.iter().all(|f| f.severity == Severity::Corrupt));
        let block = ControlBlock::parse(&b).unwrap();
        let text = render(&block, ADDR, &names(), &f);
        assert!(text.contains("rtt            CORRUPT"), "{text}");
        assert!(text.contains("corrupt  down[0].write = 1024 (expected < size 1024)"), "{text}");
        let j = to_json(&block, ADDR, &names(), &f);
        assert_eq!(j["ok"], false);
        assert_eq!(j["findings"][0]["field"], "up[0].buffer");
    }

    #[test]
    fn overlapping_buffers_are_corrupt() {
        let mut b = good();
        set(&mut b, at(1, 1), 0x3fcc_9400); // up[1] inside up[0]
        assert_eq!(fields(&run(&b)), ["up[1].buffer"]);
        let mut b = good();
        set(&mut b, at(2, 1), (ADDR - 16) as u32); // down[0] over the block
        set(&mut b, at(2, 2), 64);
        let f = run(&b);
        assert_eq!(fields(&f), ["down[0].buffer", "down[0].size"]);
        assert_eq!(f[1].severity, Severity::Differs);
    }

    #[test]
    fn layout_differences_are_not_corruption() {
        let mut b = good();
        set(&mut b, at(1, 2), 8192);
        let mut n = names();
        n.up[0] = Some("logs".into());
        let block = ControlBlock::parse(&b).unwrap();
        let f = check(&block, ADDR, &ESP32S3, Some(&FIRMWARE_LAYOUT), &n);
        assert_eq!(fields(&f), ["up[0].name", "up[1].size"]);
        assert!(!is_corrupt(&f));
        assert!(render(&block, ADDR, &n, &f).contains("OK, differs from the firmware layout"));
        assert!(check(&block, ADDR, &ESP32S3, None, &n).is_empty());
        // Unreadable names are not findings.
        assert!(check(&ControlBlock::parse(&good()).unwrap(), ADDR, &ESP32S3, Some(&FIRMWARE_LAYOUT), &Names::default()).is_empty());
    }

    #[test]
    fn c_names() {
        assert_eq!(c_name(b"defmt\0junk").as_deref(), Some("defmt"));
        assert_eq!(c_name(b"no terminator"), None);
        assert_eq!(c_name(b"\xa5\xa5\0"), None);
        assert_eq!(id_text(b"A\0\x01"), "A\\0\\x01");
    }
}
