//! Engine configuration and the module checks run when an upload completes.
//!
//! `config` fixes every limit wasmi can enforce: fuel metering on, a small
//! value stack and call depth, no floats, no `start` function, the
//! custom-page-sizes proposal on, wasmi's strict parse limits and eager
//! compilation (so a module's memory cost is paid, and checked, at upload).
//!
//! Before compiling, [`section_sizes`] walks the section headers and
//! rejects a module whose code or data section exceeds
//! [`MAX_CODE_BYTES`] / [`MAX_DATA_BYTES`] (these bound the compiled size).
//!
//! `check` then accepts a compiled module only if
//!
//! - every import is a function from [`IMPORT_MODULE`] listed in
//!   [`wasm_abi::imports`] with exactly its ABI signature;
//! - it exports a memory [`EXPORT_MEMORY`] and the functions
//!   [`EXPORT_INIT`] `() -> ()` and [`EXPORT_TICK`] `(i64) -> ()`.
//!
//! wasmi does not expose a memory's page size, so the
//! [`MAX_MEMORY_BYTES`](wasm_abi::MAX_MEMORY_BYTES) limit is checked by the
//! runtime with a trial instantiation under the store limits right after
//! these checks.

use core::fmt::Write;

use wasm_abi::{
    EXPORT_INIT, EXPORT_MEMORY, EXPORT_TICK, IMPORT_MODULE, MAX_CODE_BYTES, MAX_DATA_BYTES, imports,
};
use wasmi::{CompilationMode, Config, EnforcedLimits, ExternType, FuncType, Module, ValType};

use crate::ErrorText;

/// Bytes of wasmi value stack per instance (wasmi traps beyond it).
pub const STACK_BYTES: usize = 4 * 1024;
/// Deepest wasm call nesting allowed.
pub const MAX_CALL_DEPTH: usize = 64;

/// The engine configuration every module runs under.
pub(crate) fn config() -> Config {
    let mut config = Config::default();
    // Max before min: wasmi requires min <= max at every step.
    config
        .set_max_stack_height(STACK_BYTES)
        .set_min_stack_height(1024)
        .set_max_recursion_depth(MAX_CALL_DEPTH)
        .set_max_cached_stacks(1)
        .consume_fuel(true)
        .floats(false)
        .allow_start_fn(false)
        .wasm_custom_page_sizes(true)
        .wasm_multi_memory(false)
        .wasm_tail_call(false)
        .ignore_custom_sections(true)
        .compilation_mode(CompilationMode::Eager)
        .enforced_limits(EnforcedLimits::strict());
    config
}

/// Section id of the code section.
const CODE_SECTION: u8 = 10;
/// Section id of the data section.
const DATA_SECTION: u8 = 11;

/// Reads an unsigned LEB128 `u32` at `*at`, advancing it.
fn leb_u32(bytes: &[u8], at: &mut usize) -> Option<u32> {
    let mut value = 0u32;
    for shift in [0u32, 7, 14, 21, 28] {
        let byte = *bytes.get(*at)?;
        *at += 1;
        value |= u32::from(byte & 0x7f).checked_shl(shift)?;
        if byte & 0x80 == 0 {
            return Some(value);
        }
    }
    None
}

/// Sizes of the code and data sections of a wasm binary, rejecting ones
/// over the ABI limits. Walks only section headers (at most one step per
/// section, so bounded by the module length); full validation is wasmi's.
pub fn section_sizes(bytes: &[u8]) -> Result<(usize, usize), ErrorText> {
    let malformed = || text(format_args!("malformed module"));
    let mut at = 8; // magic + version, checked by wasmi
    let (mut code, mut data) = (0usize, 0usize);
    while at < bytes.len() {
        let id = *bytes.get(at).ok_or_else(malformed)?;
        at += 1;
        let len = usize::try_from(leb_u32(bytes, &mut at).ok_or_else(malformed)?)
            .map_err(|_| malformed())?;
        match id {
            CODE_SECTION => code = code.saturating_add(len),
            DATA_SECTION => data = data.saturating_add(len),
            _ => {}
        }
        at = at
            .checked_add(len)
            .filter(|end| *end <= bytes.len())
            .ok_or_else(malformed)?;
    }
    if code > MAX_CODE_BYTES {
        return Err(text(format_args!("code {code} B over {MAX_CODE_BYTES} B")));
    }
    if data > MAX_DATA_BYTES {
        return Err(text(format_args!("data {data} B over {MAX_DATA_BYTES} B")));
    }
    Ok((code, data))
}

/// ABI signature of each import: (name, params, results).
const IMPORTS: [(&str, &[ValType], &[ValType]); 8] = [
    (imports::NOW_MS, &[], &[ValType::I64]),
    (imports::LOG, &[ValType::I32, ValType::I32], &[]),
    (
        imports::PUBLISH,
        &[ValType::I32, ValType::I32, ValType::I32, ValType::I32],
        &[ValType::I32],
    ),
    (
        imports::SUBSCRIBE,
        &[ValType::I32, ValType::I32],
        &[ValType::I32],
    ),
    (
        imports::RECV,
        &[ValType::I32, ValType::I32],
        &[ValType::I32],
    ),
    (imports::SET_LEDS, &[ValType::I32], &[ValType::I32]),
    (
        imports::SHOW_TEXT,
        &[ValType::I32, ValType::I32],
        &[ValType::I32],
    ),
    (imports::SET_PET, &[ValType::I32], &[ValType::I32]),
];

fn signature_is(ty: &FuncType, params: &[ValType], results: &[ValType]) -> bool {
    ty.params() == params && ty.results() == results
}

/// Checks a compiled module against the ABI; the error says what is wrong.
pub(crate) fn check(module: &Module) -> Result<(), ErrorText> {
    for import in module.imports() {
        let expected = IMPORTS.iter().find(|(name, _, _)| *name == import.name());
        let ok = match (import.ty(), expected) {
            (ExternType::Func(ty), Some((_, params, results))) => {
                import.module() == IMPORT_MODULE && signature_is(ty, params, results)
            }
            _ => false,
        };
        if !ok {
            return Err(text(format_args!(
                "bad import {}.{}",
                import.module(),
                import.name()
            )));
        }
    }
    let mut memory = false;
    let mut init = false;
    let mut tick = false;
    for export in module.exports() {
        match (export.name(), export.ty()) {
            (EXPORT_MEMORY, ExternType::Memory(_)) => memory = true,
            (EXPORT_INIT, ExternType::Func(ty)) => init = signature_is(ty, &[], &[]),
            (EXPORT_TICK, ExternType::Func(ty)) => tick = signature_is(ty, &[ValType::I64], &[]),
            _ => {}
        }
    }
    for (present, name) in [
        (memory, EXPORT_MEMORY),
        (init, EXPORT_INIT),
        (tick, EXPORT_TICK),
    ] {
        if !present {
            return Err(text(format_args!("missing export {name}")));
        }
    }
    Ok(())
}

/// Formats into an [`ErrorText`], truncating on a char boundary.
pub(crate) fn text(args: core::fmt::Arguments<'_>) -> ErrorText {
    let mut out = Truncate(ErrorText::new());
    // Truncate never fails.
    let _ = out.write_fmt(args);
    out.0
}

/// A `fmt::Write` that keeps what fits and drops the rest.
struct Truncate(ErrorText);

impl Write for Truncate {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for c in s.chars() {
            if self.0.push(c).is_err() {
                break;
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn leb128_decodes_and_rejects_overlong() {
        let mut at = 0;
        assert_eq!(leb_u32(&[0xe5, 0x8e, 0x26], &mut at), Some(624_485));
        assert_eq!(at, 3);
        assert_eq!(leb_u32(&[0x80; 6], &mut 0), None);
        assert_eq!(leb_u32(&[0x80], &mut 0), None);
    }

    #[test]
    fn section_walk_bounds_code_and_data() {
        let header = b"\0asm\x01\0\0\0";
        let mut module = header.to_vec();
        module.extend_from_slice(&[10, 3, 0, 0, 0, 11, 2, 0, 0]);
        assert_eq!(section_sizes(&module), Ok((3, 2)));
        let mut big = header.to_vec();
        big.extend_from_slice(&[10, 0x81, 0x20]); // 4097 bytes
        big.resize(big.len() + 4097, 0);
        assert!(section_sizes(&big).is_err());
        let mut short = header.to_vec();
        short.extend_from_slice(&[10, 5, 0]);
        assert_eq!(
            section_sizes(&short),
            Err(text(format_args!("malformed module")))
        );
    }

    #[test]
    fn error_text_truncates() {
        let long = [b'x'; 100];
        let s = core::str::from_utf8(&long).unwrap_or("");
        assert_eq!(text(format_args!("{s}")).len(), crate::ERROR_LEN);
        assert_eq!(text(format_args!("a{}", 1)).as_str(), "a1");
    }
}
