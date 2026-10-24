//! FR-28 (radio key import) tests over littlefs images made by
//! `fixtures/gen_fixtures.py` (littlefs-python, stock geometry, fake keys).

extern crate std;

use std::format;
use std::vec::Vec;

use super::*;

static INLINE: &[u8] = include_bytes!("fixtures/key_inline.img");
static SPLIT: &[u8] = include_bytes!("fixtures/key_split.img");
static REWRITE: &[u8] = include_bytes!("fixtures/key_rewrite.img");
static MISSING: &[u8] = include_bytes!("fixtures/key_missing.img");
static CORRUPT: &[u8] = include_bytes!("fixtures/corrupt.img");

const BS: u32 = 4096;

/// `FAKE_KEY_A` of the generator.
const KEY_A: [u8; 32] = [
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10,
];
/// `FAKE_KEY_B` of the generator.
const KEY_B: [u8; 32] = [
    0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef,
    0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef,
];

fn geo(img: &[u8]) -> Geometry {
    Geometry { block_size: BS, block_count: (img.len() / BS as usize) as u32 }
}

fn read(img: &[u8], name: &str) -> Result<Vec<u8>, LfsError<OutOfRange>> {
    let mut buf = [0u8; 8192];
    let n = read_root_file(&mut SliceDevice::new(img, BS), geo(img), name.as_bytes(), &mut buf)?;
    Ok(buf[..n].to_vec())
}

fn import(img: &[u8]) -> Result<RadioKey, ImportError<OutOfRange>> {
    import_radio_key(&mut SliceDevice::new(img, BS), geo(img))
}

fn rev(img: &[u8], block: usize) -> u32 {
    let o = block * BS as usize;
    u32::from_le_bytes([img[o], img[o + 1], img[o + 2], img[o + 3]])
}

/// The root block with the newer revision count.
fn newer_root_block(img: &[u8]) -> usize {
    if (rev(img, 1).wrapping_sub(rev(img, 0)) as i32) > 0 { 1 } else { 0 }
}

#[test]
fn fr28_crc32_matches_littlefs() {
    // CRC-32/JAMCRC check value: CRC-32 without the final inversion.
    assert_eq!(crc32(0xFFFF_FFFF, b"123456789"), !0xCBF4_3926);
}

#[test]
fn fr28_inline_fake_key_imported() {
    assert_eq!(import(INLINE), Ok(RadioKey(KEY_A)));
}

#[test]
fn fr28_inline_file_raw_text_keeps_whitespace() {
    let text = read(INLINE, "radio_key.txt").unwrap();
    assert_eq!(text.len(), 2 + 64 + 2);
    assert!(text.starts_with(b" \t") && text.ends_with(b"\r\n"));
}

#[test]
fn fr28_single_block_ctz_file_read() {
    let data = read(INLINE, "big.txt").unwrap();
    let want: Vec<u8> = (0..1000u32).map(|i| (i % 251) as u8).collect();
    assert_eq!(data, want);
}

#[test]
fn fr28_multi_block_ctz_file_not_supported() {
    assert_eq!(read(INLINE, "huge.bin"), Err(LfsError::NotSupported));
}

#[test]
fn fr28_other_inline_files_read() {
    assert_eq!(read(INLINE, "boot.py").unwrap(), b"# fake boot\n");
    assert_eq!(read(INLINE, "empty.txt").unwrap(), b"");
}

#[test]
fn fr28_directory_and_superblock_are_not_files() {
    assert_eq!(read(INLINE, "data"), Err(LfsError::NotAFile));
    assert_eq!(read(INLINE, "littlefs"), Err(LfsError::NotAFile));
}

#[test]
fn fr28_similar_names_do_not_match() {
    assert_eq!(read(INLINE, "radio_key.tx"), Err(LfsError::NotFound));
    assert_eq!(read(INLINE, "radio_key.txT"), Err(LfsError::NotFound));
    assert_eq!(read(MISSING, "radio_key.txt"), Err(LfsError::NotFound));
}

#[test]
fn fr28_missing_key_file() {
    assert_eq!(import(MISSING), Err(ImportError::Fs(LfsError::NotFound)));
    // The decoy next to it is readable, so the image itself is fine.
    assert!(read(MISSING, "radio_key.bak").is_ok());
}

#[test]
fn fr28_short_and_bad_key_files() {
    let short = read(INLINE, "short_key.txt").unwrap();
    let mut want = [0u8; 32];
    want[..16].copy_from_slice(&KEY_A[..16]);
    assert_eq!(parse_radio_key(&short), Ok(RadioKey(want)));
    let bad = read(INLINE, "bad_key.txt").unwrap();
    assert_eq!(parse_radio_key(&bad), Err(KeyError::NotHex));
}

#[test]
fn fr28_root_split_across_hardtails() {
    let mut dev = SliceDevice::new(SPLIT, BS);
    let root = fetch_pair(&mut dev, geo(SPLIT), [0, 1], b"radio_key.txt").unwrap();
    assert!(root.split && root.tail.is_some(), "fixture must split the root");
    assert_eq!(root.found & INVALID, INVALID, "key must not be in the first pair");
    assert_eq!(import(SPLIT), Ok(RadioKey(KEY_B)));
}

#[test]
fn fr28_every_file_of_a_split_root_found() {
    for i in 0..120 {
        let mut name = format!("f{i:03}_");
        name.push_str(&"x".repeat(58));
        assert_eq!(read(SPLIT, &name), Ok(Vec::new()), "{name}");
    }
}

#[test]
fn fr28_both_blocks_valid_newer_revision_wins() {
    let g = geo(REWRITE);
    let mut dev = SliceDevice::new(REWRITE, BS);
    assert!(fetch_block(&mut dev, g, 0, b"").unwrap().is_some());
    assert!(fetch_block(&mut dev, g, 1, b"").unwrap().is_some());
    assert_ne!(rev(REWRITE, 0), rev(REWRITE, 1));
    assert_eq!(import(REWRITE), Ok(RadioKey(KEY_B)));
}

#[test]
fn fr28_damaged_newer_block_falls_back_to_older() {
    let mut img = REWRITE.to_vec();
    let newer = newer_root_block(&img);
    img[newer * BS as usize + 8] ^= 0x55;
    assert_eq!(import(&img), Ok(RadioKey(KEY_A)));
}

#[test]
fn fr28_torn_last_commit_is_ignored() {
    let g = geo(REWRITE);
    let newer = newer_root_block(REWRITE) as u32;
    let dir = fetch_block(&mut SliceDevice::new(REWRITE, BS), g, newer, b"").unwrap().unwrap();
    let mut img = REWRITE.to_vec();
    let last_data = (newer * BS + dir.off - dsize(dir.etag) - 1) as usize;
    img[last_data] ^= 0x01;
    // The key B commit fails its CRC; the state before it had key A.
    assert_eq!(import(&img), Ok(RadioKey(KEY_A)));
}

#[test]
fn fr28_corrupt_image_reports_corrupt() {
    assert_eq!(import(CORRUPT), Err(ImportError::Fs(LfsError::Corrupt(0))));
}

#[test]
fn fr28_erased_and_zeroed_flash_are_corrupt() {
    let erased = std::vec![0xFFu8; 4 * BS as usize];
    assert_eq!(read(&erased, "radio_key.txt"), Err(LfsError::Corrupt(0)));
    let zeros = std::vec![0u8; 4 * BS as usize];
    assert_eq!(read(&zeros, "radio_key.txt"), Err(LfsError::Corrupt(0)));
}

#[test]
fn fr28_geometry_must_match_superblock() {
    let mut buf = [0u8; 128];
    let mut dev = SliceDevice::new(INLINE, BS);
    let wrong_size = Geometry { block_size: 8192, block_count: 4 };
    assert_eq!(read_root_file(&mut dev, wrong_size, RADIO_KEY_FILE, &mut buf), Err(LfsError::BadGeometry));
    let too_few = Geometry { block_size: BS, block_count: 4 };
    assert_eq!(read_root_file(&mut dev, too_few, RADIO_KEY_FILE, &mut buf), Err(LfsError::BadGeometry));
    let odd = Geometry { block_size: 100, block_count: 8 };
    assert_eq!(read_root_file(&mut dev, odd, RADIO_KEY_FILE, &mut buf), Err(LfsError::BadGeometry));
}

#[test]
fn fr28_truncated_device_is_an_io_error() {
    let half = &INLINE[..BS as usize + 16];
    let mut buf = [0u8; 128];
    let r = read_root_file(&mut SliceDevice::new(half, BS), geo(INLINE), RADIO_KEY_FILE, &mut buf);
    assert!(matches!(r, Err(LfsError::Io(OutOfRange)) | Err(LfsError::Corrupt(_))), "{r:?}");
}

#[test]
fn fr28_bad_names_rejected() {
    assert_eq!(read(INLINE, ""), Err(LfsError::BadName));
    assert_eq!(read(INLINE, "data/x"), Err(LfsError::BadName));
    assert_eq!(read(INLINE, &"a".repeat(256)), Err(LfsError::BadName));
}

#[test]
fn fr28_small_buffer_reports_size() {
    let mut buf = [0u8; 10];
    let r = read_root_file(&mut SliceDevice::new(INLINE, BS), geo(INLINE), RADIO_KEY_FILE, &mut buf);
    assert_eq!(r, Err(LfsError::TooBig(68)));
}

#[test]
fn fr28_parse_matches_stock_load_key() {
    let hex = b"00112233445566778899AABBCCDDEEFF0123456789abcdeffedcba9876543210";
    assert_eq!(parse_radio_key(hex), Ok(RadioKey(KEY_A)));
    let mut padded = Vec::from(&b"\n\t "[..]);
    padded.extend_from_slice(hex);
    padded.extend_from_slice(b"\r\n\x0c");
    assert_eq!(parse_radio_key(&padded), Ok(RadioKey(KEY_A)));
    let spaced = b"00 11 22 33 44 55 66 77 88 99 aa bb cc dd ee ff 01 23 45 67 89 ab cd ef fe dc ba 98 76 54 32 10";
    assert_eq!(parse_radio_key(spaced), Ok(RadioKey(KEY_A)));
}

#[test]
fn fr28_parse_rejects_like_stock() {
    assert_eq!(parse_radio_key(b""), Err(KeyError::TooShort(0)));
    assert_eq!(parse_radio_key(b"  \n"), Err(KeyError::TooShort(0)));
    assert_eq!(parse_radio_key(b"00112233445566778899aabbccddee"), Err(KeyError::TooShort(15)));
    assert_eq!(parse_radio_key(b"0011223344556677889"), Err(KeyError::NotHex));
    assert_eq!(parse_radio_key(b"0 0112233445566778899aabbccddeeff"), Err(KeyError::NotHex));
    assert_eq!(parse_radio_key(b"zz112233445566778899aabbccddeeff"), Err(KeyError::NotHex));
    let long = [b'a'; 66];
    assert_eq!(parse_radio_key(&long), Err(KeyError::TooLong(33)));
}

#[test]
fn fr28_sixteen_byte_key_is_zero_padded() {
    let mut want = [0u8; 32];
    want[..16].copy_from_slice(&KEY_A[..16]);
    assert_eq!(parse_radio_key(b"00112233445566778899aabbccddeeff"), Ok(RadioKey(want)));
}

#[test]
fn fr28_errors_never_show_key_bytes() {
    let s = format!("{:?} {:?}", import(INLINE), parse_radio_key(b"deadbeefdeadbeefdeadbeefdeadbeef"));
    assert!(!s.contains("dead") && !s.contains("222") && !s.contains("173"), "{s}");
}
