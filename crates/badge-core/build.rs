//! Build script: encodes the project URL as a QR code for the HOME screen.
//!
//! Runs on the host (std is fine here). It encodes [`URL`] with the `qrcode`
//! crate at error-correction level M, letting the crate pick the smallest
//! version (version 3, 29x29 modules, for the current URL), and writes
//! `qr_data.rs` into `OUT_DIR` with:
//!
//! - `QR_URL`: the encoded text;
//! - `QR_VERSION`: the QR version the crate picked;
//! - `QR_SIZE`: modules per side;
//! - `QR`: one `u32` per module row, bit `x` set when module `x` is dark.
//!
//! `src/qr.rs` includes that file. The build fails if the symbol is wider than
//! 32 modules, since rows would no longer fit in a `u32`.

use std::{env, fs, path::PathBuf};

use qrcode::{Color, EcLevel, QrCode, Version};

/// Text encoded in the HOME screen QR code.
const URL: &str = "https://github.com/jvishnefske/conagotchi";

fn main() {
    println!("cargo::rerun-if-changed=build.rs");
    let code = QrCode::with_error_correction_level(URL.as_bytes(), EcLevel::M)
        .expect("URL does not fit any QR version at ECC M");
    let version = match code.version() {
        Version::Normal(v) => v,
        Version::Micro(v) => panic!("unexpected micro QR version {v}"),
    };
    let size = code.width();
    assert!(size <= 32, "QR is {size} modules wide; rows must fit a u32");
    let colors = code.to_colors();
    let mut rows = String::new();
    for y in 0..size {
        let mut bits = 0u32;
        for x in 0..size {
            if colors[y * size + x] == Color::Dark {
                bits |= 1 << x;
            }
        }
        rows.push_str(&format!("    0x{bits:08X},\n"));
    }
    let out = format!(
        "/// Text encoded in the QR code.\n\
         pub const QR_URL: &str = {URL:?};\n\
         /// QR version picked by the encoder (error correction level M).\n\
         pub const QR_VERSION: u8 = {version};\n\
         /// Modules per side of the QR symbol (no quiet zone).\n\
         pub const QR_SIZE: usize = {size};\n\
         /// Module rows, top first; bit `x` of a row is set when module `x` is dark.\n\
         pub const QR: [u32; QR_SIZE] = [\n{rows}];\n"
    );
    let dir = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR is set by cargo"));
    fs::write(dir.join("qr_data.rs"), out).expect("write qr_data.rs");
}
