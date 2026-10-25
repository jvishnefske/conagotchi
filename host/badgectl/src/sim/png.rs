//! A minimal PNG encoder for simulator framebuffers.
//!
//! [`encode_rgb`] writes an 8-bit RGB image: signature, `IHDR`, one `IDAT`
//! holding a zlib stream of *stored* (uncompressed) deflate blocks with
//! filter type 0 on every row, and `IEND`, with the CRC-32 and Adler-32
//! checksums the format requires. Files are larger than compressed PNGs
//! (about 170 KiB for 240 × 240) but need no dependency.
//! [`framebuffer_png`] converts a `badge_sim::fb` framebuffer first.

use badge_sim::fb::{FB_HEIGHT, FB_WIDTH, rgb888};

/// CRC-32 (IEEE, reflected) of `data`, continuing from `crc`.
fn crc32(crc: u32, data: &[u8]) -> u32 {
    let mut c = !crc;
    for &b in data {
        c ^= u32::from(b);
        for _ in 0..8 {
            c = if c & 1 != 0 { (c >> 1) ^ 0xEDB8_8320 } else { c >> 1 };
        }
    }
    !c
}

/// Adler-32 of `data`.
fn adler32(data: &[u8]) -> u32 {
    let (mut a, mut b) = (1u32, 0u32);
    for &x in data {
        a = (a + u32::from(x)) % 65_521;
        b = (b + a) % 65_521;
    }
    (b << 16) | a
}

fn chunk(out: &mut Vec<u8>, kind: &[u8; 4], data: &[u8]) {
    out.extend_from_slice(&u32::try_from(data.len()).unwrap_or(u32::MAX).to_be_bytes());
    out.extend_from_slice(kind);
    out.extend_from_slice(data);
    out.extend_from_slice(&crc32(crc32(0, kind), data).to_be_bytes());
}

/// PNG bytes of a `width` × `height` RGB image (`rgb.len()` must be
/// `3 * width * height`; missing pixels are black).
pub fn encode_rgb(width: u32, height: u32, rgb: &[u8]) -> Vec<u8> {
    let row = 3 * width as usize;
    let mut raw = Vec::with_capacity((row + 1) * height as usize);
    for y in 0..height as usize {
        raw.push(0);
        let start = y * row;
        let line = rgb.get(start..start + row).unwrap_or(&[]);
        raw.extend_from_slice(line);
        raw.resize(raw.len() + (row - line.len()), 0);
    }
    let mut z = vec![0x78, 0x01];
    let mut blocks = raw.chunks(65_535).peekable();
    if blocks.peek().is_none() {
        z.extend_from_slice(&[1, 0, 0, 0xFF, 0xFF]);
    }
    while let Some(block) = blocks.next() {
        let len = u16::try_from(block.len()).unwrap_or(u16::MAX);
        z.push(u8::from(blocks.peek().is_none()));
        z.extend_from_slice(&len.to_le_bytes());
        z.extend_from_slice(&(!len).to_le_bytes());
        z.extend_from_slice(block);
    }
    z.extend_from_slice(&adler32(&raw).to_be_bytes());
    let mut ihdr = Vec::with_capacity(13);
    ihdr.extend_from_slice(&width.to_be_bytes());
    ihdr.extend_from_slice(&height.to_be_bytes());
    ihdr.extend_from_slice(&[8, 2, 0, 0, 0]);
    let mut out = b"\x89PNG\r\n\x1a\n".to_vec();
    chunk(&mut out, b"IHDR", &ihdr);
    chunk(&mut out, b"IDAT", &z);
    chunk(&mut out, b"IEND", &[]);
    out
}

/// PNG bytes of a simulator framebuffer.
pub fn framebuffer_png(fb: &[u8]) -> Vec<u8> {
    let mut rgb = Vec::with_capacity(3 * FB_WIDTH * FB_HEIGHT);
    for y in 0..FB_HEIGHT {
        for x in 0..FB_WIDTH {
            rgb.extend_from_slice(&rgb888(fb, x, y));
        }
    }
    encode_rgb(FB_WIDTH as u32, FB_HEIGHT as u32, &rgb)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fr39_checksums_match_reference_values() {
        assert_eq!(crc32(0, b"123456789"), 0xCBF4_3926);
        assert_eq!(adler32(b"Wikipedia"), 0x11E6_0398);
    }

    #[test]
    fn fr39_png_layout_of_a_tiny_image() {
        let png = encode_rgb(2, 1, &[255, 0, 0, 0, 255, 0]);
        assert_eq!(&png[..8], b"\x89PNG\r\n\x1a\n");
        assert_eq!(&png[12..16], b"IHDR");
        assert_eq!(&png[16..24], &[0, 0, 0, 2, 0, 0, 0, 1]);
        assert_eq!(&png[png.len() - 8..png.len() - 4], b"IEND");
        // IDAT: zlib header, one final stored block of 7 bytes (filter + 2 px).
        let idat = 8 + 25;
        assert_eq!(&png[idat + 4..idat + 8], b"IDAT");
        assert_eq!(&png[idat + 8..idat + 15], &[0x78, 0x01, 1, 7, 0, 0xF8, 0xFF]);
        assert_eq!(&png[idat + 15..idat + 22], &[0, 255, 0, 0, 0, 255, 0]);
    }

    #[test]
    fn fr39_framebuffer_png_splits_into_stored_blocks() {
        let fb = vec![0xFFu8; badge_sim::fb::FB_BYTES];
        let png = framebuffer_png(&fb);
        let raw: usize = (3 * 240 + 1) * 240;
        let blocks = raw.div_ceil(65_535);
        assert_eq!(png.len(), 8 + 25 + 12 + 2 + 5 * blocks + raw + 4 + 12);
    }
}
