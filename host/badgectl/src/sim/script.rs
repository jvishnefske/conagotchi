//! Scripted scenarios for `badgectl sim --script <file>`.
//!
//! One step per line: `<at_ms> <badge> <request...>`, where `<request>` is
//! any `badgectl` request command line (`trade invite @1`, `chi cry`,
//! `wasm load apps/hello.wasm`, `text "hi there"`, ...). `@N` is replaced
//! by the peer id of badge `N`, so scripts do not depend on MACs. Blank
//! lines and lines starting with `#` are ignored; double quotes group words.
//! Steps run in time order (stable for equal times) when the world reaches
//! `at_ms`.
//!
//! [`parse`] checks the syntax; [`resolve`] substitutes peer ids and parses
//! the request with the same clap definition as the command line.

use badge_core::oz::PeerId;
use clap::Parser;

use crate::cli::RequestCmd;

/// One scripted request.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Step {
    /// 1-based line number.
    pub line: usize,
    /// Virtual time to run at.
    pub at_ms: u64,
    /// Target badge.
    pub badge: usize,
    /// Request words (before `@N` substitution).
    pub words: Vec<String>,
}

/// Script errors.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ScriptError {
    /// A line does not start with `<at_ms> <badge>` followed by a request.
    #[error("line {0}: expected `<at_ms> <badge> <request...>`")]
    Syntax(usize),
    /// Unbalanced double quote.
    #[error("line {0}: unterminated quote")]
    Quote(usize),
    /// `@N` names a badge that does not exist.
    #[error("line {line}: no badge {badge} for @{badge}")]
    NoBadge {
        /// Line number.
        line: usize,
        /// Badge index.
        badge: usize,
    },
    /// The request words do not parse.
    #[error("line {line}: {message}")]
    Request {
        /// Line number.
        line: usize,
        /// clap's message.
        message: String,
    },
}

/// Splits a line into words; double quotes group (and are removed).
pub fn words(line: &str, number: usize) -> Result<Vec<String>, ScriptError> {
    let mut out = Vec::new();
    let mut cur = String::new();
    let mut quoted = false;
    let mut any = false;
    for ch in line.chars() {
        match ch {
            '"' => {
                quoted = !quoted;
                any = true;
            }
            c if c.is_whitespace() && !quoted => {
                if any {
                    out.push(std::mem::take(&mut cur));
                    any = false;
                }
            }
            c => {
                cur.push(c);
                any = true;
            }
        }
    }
    if quoted {
        return Err(ScriptError::Quote(number));
    }
    if any {
        out.push(cur);
    }
    Ok(out)
}

/// Parses a script; steps come back sorted by time.
pub fn parse(text: &str) -> Result<Vec<Step>, ScriptError> {
    let mut steps = Vec::new();
    for (idx, raw) in text.lines().enumerate() {
        let line = idx + 1;
        let trimmed = raw.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }
        let w = words(trimmed, line)?;
        let (Some(at), Some(badge)) = (w.first(), w.get(1)) else {
            return Err(ScriptError::Syntax(line));
        };
        let (Ok(at_ms), Ok(badge)) = (at.parse::<u64>(), badge.parse::<usize>()) else {
            return Err(ScriptError::Syntax(line));
        };
        let words: Vec<String> = w.into_iter().skip(2).collect();
        if words.is_empty() {
            return Err(ScriptError::Syntax(line));
        }
        steps.push(Step { line, at_ms, badge, words });
    }
    steps.sort_by_key(|s| s.at_ms);
    Ok(steps)
}

/// A request line parsed on its own.
#[derive(Debug, Parser)]
#[command(no_binary_name = true, name = "script")]
struct Line {
    #[command(subcommand)]
    req: RequestCmd,
}

/// Substitutes `@N` and parses the step's request.
pub fn resolve(step: &Step, ids: &[PeerId]) -> Result<RequestCmd, ScriptError> {
    let mut args = Vec::with_capacity(step.words.len());
    for w in &step.words {
        match w.strip_prefix('@').map(str::parse::<usize>) {
            Some(Ok(n)) => {
                let id = ids.get(n).ok_or(ScriptError::NoBadge { line: step.line, badge: n })?;
                args.push(id.as_str().to_string());
            }
            _ => args.push(w.clone()),
        }
    }
    Line::try_parse_from(args)
        .map(|l| l.req)
        .map_err(|e| ScriptError::Request { line: step.line, message: e.to_string().trim().to_string() })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::{ChiArg, TradeCmd, WasmOp};

    fn ids() -> Vec<PeerId> {
        vec![PeerId::from_u32(0xFFEE01), PeerId::from_u32(0xFFEE02)]
    }

    #[test]
    fn fr39_script_parses_sorts_and_substitutes_peer_ids() {
        let text = "# demo\n\n2000 1 trade invite @0\n1000 0 chi cry\n2000 0 text \"hi  there\"\n";
        let steps = parse(text).unwrap();
        assert_eq!(steps.iter().map(|s| s.at_ms).collect::<Vec<_>>(), [1000, 2000, 2000]);
        assert_eq!(steps[1].line, 3, "stable for equal times");
        assert_eq!(resolve(&steps[0], &ids()), Ok(RequestCmd::Chi { chi: ChiArg::Cry }));
        let Ok(RequestCmd::Trade { op: TradeCmd::Invite { peer } }) = resolve(&steps[1], &ids()) else {
            panic!("invite")
        };
        assert_eq!(peer.as_str(), "FFEE01");
        assert!(matches!(resolve(&steps[2], &ids()), Ok(RequestCmd::Text { text, .. }) if text == "hi  there"));
    }

    #[test]
    fn fr39_script_errors_name_the_line() {
        assert_eq!(parse("x 0 info"), Err(ScriptError::Syntax(1)));
        assert_eq!(parse("\n10 0"), Err(ScriptError::Syntax(2)));
        assert_eq!(parse("10 0 text \"open"), Err(ScriptError::Quote(1)));
        let s = parse("5 0 trade invite @7").unwrap();
        assert_eq!(resolve(&s[0], &ids()), Err(ScriptError::NoBadge { line: 1, badge: 7 }));
        let s = parse("5 0 frobnicate").unwrap();
        assert!(matches!(resolve(&s[0], &ids()), Err(ScriptError::Request { line: 1, .. })));
        let s = parse("5 0 wasm load apps/hello.wasm").unwrap();
        assert!(matches!(
            resolve(&s[0], &ids()),
            Ok(RequestCmd::Wasm { op: WasmOp::Load { .. } })
        ));
    }
}
