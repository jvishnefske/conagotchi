//! defmt log decoding and the terminal [`Observer`] (imperative shell).
//!
//! The defmt table and source locations come from the firmware ELF. Without
//! an ELF the log bytes are still drained from the target but discarded.
//! Logs go to stderr so stdout stays clean for responses (and `--json`).

use std::io::IsTerminal;

use badge_proto::{Event, Response};
use defmt_decoder::{DecodeError, Locations, StreamDecoder, Table};

use crate::format;
use crate::session::{Observer, UpError};

/// Upper bound on frames decoded from one chunk of log bytes.
const MAX_FRAMES_PER_CHUNK: usize = 4096;

/// Parsed defmt metadata from the ELF.
pub struct DefmtInfo {
    table: Table,
    locations: Option<Locations>,
}

impl DefmtInfo {
    /// Parses the defmt table; `Ok(None)` if the ELF has no defmt data.
    pub fn from_elf(elf: &[u8]) -> anyhow::Result<Option<Self>> {
        let Some(table) = Table::parse(elf)? else { return Ok(None) };
        let locations = table.get_locations(elf).ok().filter(|l| !l.is_empty());
        Ok(Some(Self { table, locations }))
    }
}

/// Prints logs, events, stray responses and frame errors.
pub struct Printer<'t> {
    defmt: Option<(&'t DefmtInfo, Box<dyn StreamDecoder + Send + Sync + 't>)>,
    json: bool,
    colored: bool,
}

impl<'t> Printer<'t> {
    /// A printer using `defmt` (if any) for the log channel.
    pub fn new(defmt: Option<&'t DefmtInfo>, json: bool) -> Self {
        Self {
            defmt: defmt.map(|d| (d, d.table.new_stream_decoder())),
            json,
            colored: std::io::stderr().is_terminal(),
        }
    }
}

impl Observer for Printer<'_> {
    fn log_bytes(&mut self, bytes: &[u8]) {
        let Some((info, decoder)) = self.defmt.as_mut() else { return };
        decoder.received(bytes);
        for _ in 0..MAX_FRAMES_PER_CHUNK {
            match decoder.decode() {
                Ok(frame) => {
                    eprintln!("{}", frame.display(self.colored));
                    let loc = info.locations.as_ref().and_then(|l| l.get(&frame.index()));
                    if let Some(loc) = loc {
                        eprintln!("└─ {} @ {}:{}", loc.module, loc.file.display(), loc.line);
                    }
                }
                Err(DecodeError::UnexpectedEof) => break,
                Err(DecodeError::Malformed) => {
                    eprintln!("(malformed defmt frame)");
                    if !info.table.encoding().can_recover() {
                        break;
                    }
                }
            }
        }
    }

    fn event(&mut self, event: &Event) {
        if self.json {
            println!("{}", format::event_json(event));
        } else {
            println!("{}", format::event_text(event));
        }
    }

    fn stray_response(&mut self, response: &Response) {
        if self.json {
            println!("{}", format::response_json(response));
        } else {
            println!("response seq {}: {}", response.seq, format::response_text(response));
        }
    }

    fn frame_error(&mut self, error: &UpError) {
        eprintln!("warning: proto-up: {error}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn elf_without_defmt_is_an_error_not_a_panic() {
        assert!(DefmtInfo::from_elf(b"not an elf").is_err());
    }

    /// Checks the real firmware's defmt table when a build is present
    /// (skipped on a fresh checkout).
    #[test]
    fn firmware_defmt_table_parses_if_built() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../firmware/target/xtensa-esp32s3-none-elf/release/conagotchi-fw");
        let Ok(bytes) = std::fs::read(&path) else { return };
        let info = DefmtInfo::from_elf(&bytes).expect("parse").expect("defmt table present");
        assert!(!info.table.is_empty());
        let syms = crate::elf::symbols_from_elf(&bytes).expect("symbols");
        assert!(syms.probe.is_some() && syms.rtt.is_some(), "{syms:?}");
    }
}
