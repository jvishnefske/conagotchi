//! Firmware ELF symbol lookup.
//!
//! The firmware's `PROBE` static is an ordinary (mangled) Rust symbol, so it
//! is found by demangling every data symbol and matching the path suffix
//! `::PROBE` ([`matches_probe_symbol`]). `_SEGGER_RTT` is the unmangled RTT
//! control block. Selection over `(demangled name, address)` pairs is the
//! pure [`select_symbols`]; [`symbols_from_elf`] feeds it from an ELF image.

use object::{Object, ObjectSymbol, SymbolKind};

/// RTT control block symbol.
pub const RTT_SYMBOL: &str = "_SEGGER_RTT";

/// Addresses of interest in the firmware image.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Symbols {
    /// Address of the `PROBE` block.
    pub probe: Option<u64>,
    /// Address of the RTT control block.
    pub rtt: Option<u64>,
}

/// Symbol lookup failures.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ElfError {
    /// The bytes are not a readable object file.
    #[error("cannot parse ELF: {0}")]
    Parse(String),
    /// More than one distinct symbol matched `::PROBE`.
    #[error("ambiguous PROBE symbol: {0}")]
    Ambiguous(String),
}

/// True for the demangled path of the firmware `PROBE` static
/// (`PROBE` itself or any `path::to::PROBE`).
pub fn matches_probe_symbol(demangled: &str) -> bool {
    demangled == "PROBE" || demangled.ends_with("::PROBE")
}

/// Demangles a symbol name without the trailing hash.
pub fn demangle(name: &str) -> String {
    format!("{:#}", rustc_demangle::demangle(name))
}

/// Picks the PROBE and RTT addresses from `(demangled name, address)` pairs.
///
/// Duplicate entries for the same address (e.g. local + global aliases) are
/// fine; two different addresses matching `::PROBE` are an error.
pub fn select_symbols<'a, I>(symbols: I) -> Result<Symbols, ElfError>
where
    I: IntoIterator<Item = (&'a str, u64)>,
{
    let mut found = Symbols::default();
    let mut probe_names: Vec<String> = Vec::new();
    for (name, addr) in symbols {
        if name == RTT_SYMBOL {
            found.rtt = Some(addr);
        } else if matches_probe_symbol(name) {
            match found.probe {
                Some(prev) if prev != addr => {
                    probe_names.push(format!("{name} @ {addr:#x}"));
                    return Err(ElfError::Ambiguous(probe_names.join(", ")));
                }
                _ => {
                    found.probe = Some(addr);
                    probe_names.push(format!("{name} @ {addr:#x}"));
                }
            }
        }
    }
    Ok(found)
}

/// Reads the symbol table of an ELF image and selects the PROBE / RTT symbols.
pub fn symbols_from_elf(bytes: &[u8]) -> Result<Symbols, ElfError> {
    let file = object::File::parse(bytes).map_err(|e| ElfError::Parse(e.to_string()))?;
    let named: Vec<(String, u64)> = file
        .symbols()
        .filter(|s| matches!(s.kind(), SymbolKind::Data | SymbolKind::Unknown))
        .filter_map(|s| s.name().ok().map(|n| (demangle(n), s.address())))
        .collect();
    select_symbols(named.iter().map(|(n, a)| (n.as_str(), *a)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn probe_symbol_matching() {
        assert!(matches_probe_symbol("conagotchi_fw::probe::PROBE"));
        assert!(matches_probe_symbol("PROBE"));
        assert!(!matches_probe_symbol("conagotchi_fw::probe::PROBE_LEN"));
        assert!(!matches_probe_symbol("conagotchi_fw::NOPROBE"));
        assert!(!matches_probe_symbol("conagotchi_fw::PROBE::inner"));
    }

    #[test]
    fn demangles_legacy_rust_symbols() {
        let mangled = "_ZN13conagotchi_fw5probe5PROBE17h0123456789abcdefE";
        assert_eq!(demangle(mangled), "conagotchi_fw::probe::PROBE");
        assert!(matches_probe_symbol(&demangle(mangled)));
        assert_eq!(demangle(RTT_SYMBOL), RTT_SYMBOL);
    }

    #[test]
    fn demangles_v0_rust_symbols() {
        let mangled = "_RNvNtCs1234_13conagotchi_fw5probe5PROBE";
        assert!(matches_probe_symbol(&demangle(mangled)), "{}", demangle(mangled));
    }

    #[test]
    fn selects_addresses() {
        let syms = [
            ("defmt::export::FOO", 1),
            ("conagotchi_fw::probe::PROBE", 0x3fc8_1000),
            ("conagotchi_fw::probe::PROBE", 0x3fc8_1000),
            ("_SEGGER_RTT", 0x3fc8_2000),
        ];
        assert_eq!(
            select_symbols(syms),
            Ok(Symbols { probe: Some(0x3fc8_1000), rtt: Some(0x3fc8_2000) })
        );
        assert_eq!(select_symbols([("x", 1)]), Ok(Symbols::default()));
    }

    #[test]
    fn ambiguous_probe_is_an_error() {
        let syms = [("a::PROBE", 1), ("b::PROBE", 2)];
        assert!(matches!(select_symbols(syms), Err(ElfError::Ambiguous(_))));
    }

    #[test]
    fn garbage_is_not_an_elf() {
        assert!(matches!(symbols_from_elf(b"not an elf at all"), Err(ElfError::Parse(_))));
    }
}
