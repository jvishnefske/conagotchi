//! Stack high-water measurement by painting (functional core).
//!
//! The firmware's main stack is the `.stack` section of esp-hal 1.x's
//! linker script (`ld/sections/stack.x`): it runs from `_stack_end_cpu0`
//! (lowest address, the limit the stack grows towards) up to
//! `_stack_start_cpu0` (highest address, where SP starts). The generic
//! `_stack_end` / `_stack_start` aliases are used when the `_cpu0` names are
//! missing, and `_stack_end_cpu1` / `_stack_start_cpu1` add a second region
//! if an image defines them (the badge image does not). esp-hal's stack
//! guard word `__stack_chk_guard` sits `ESP_HAL_CONFIG_STACK_GUARD_OFFSET`
//! (60) bytes above the bottom and holds [`GUARD_VALUE`]; a CPU store to it
//! trips esp-hal's watchpoint, so painting never touches it.
//!
//! Embassy tasks all run on this one stack (the firmware's executor runs in
//! the main thread); esp-rtos threads such as the Wi-Fi task get stacks from
//! the heap at run time, which no ELF symbol describes, so they are not
//! measured here.
//!
//! The method, as pure steps the probe shell in `main.rs` drives:
//!
//! 1. [`regions_from_symbols`] finds the regions in the ELF symbol table.
//! 2. [`paint_limit`] turns a sampled SP and a safety margin into the upper
//!    bound of the paint, and [`plan_paint`] clips it to the paintable part
//!    of the region (above the guard word, word aligned). Painting goes
//!    from the bottom up to that bound only, so the live part above SP is
//!    never written.
//! 3. After the badge has run, [`find_watermark`] skips any unpainted words
//!    at the bottom (a paint that started higher, e.g. a manual one) until
//!    [`MIN_PAINT_RUN`] pattern words in a row, then counts the pattern words
//!    still intact upward; the first changed word is the deepest point the
//!    stack reached. [`StackReport`] turns that into
//!    used bytes and margin and renders it as text or JSON.

use std::fmt::Write as _;

use serde_json::{Value, json};

/// Highest address (initial SP) of CPU 0's main stack.
pub const STACK_START_CPU0: &str = "_stack_start_cpu0";
/// Lowest address (overflow limit) of CPU 0's main stack.
pub const STACK_END_CPU0: &str = "_stack_end_cpu0";
/// Generic alias of [`STACK_START_CPU0`].
pub const STACK_START: &str = "_stack_start";
/// Generic alias of [`STACK_END_CPU0`].
pub const STACK_END: &str = "_stack_end";
/// Highest address of CPU 1's stack, when an image defines one.
pub const STACK_START_CPU1: &str = "_stack_start_cpu1";
/// Lowest address of CPU 1's stack, when an image defines one.
pub const STACK_END_CPU1: &str = "_stack_end_cpu1";
/// esp-hal's stack guard word (stack-protector canary and watchpoint).
pub const STACK_GUARD: &str = "__stack_chk_guard";
/// esp-hal's default `ESP_HAL_CONFIG_STACK_GUARD_VALUE` (3740121773).
pub const GUARD_VALUE: u32 = 0xDEED_BAAD;
/// Default paint word, as the CLI default (`0xa5a5a5a5`).
pub const DEFAULT_PATTERN_ARG: &str = "0xa5a5a5a5";
/// Default distance kept between the sampled SP and the top of the paint.
pub const DEFAULT_MARGIN: u64 = 16 * 1024;
/// Consecutive pattern words that mark the start of a paint.
pub const MIN_PAINT_RUN: usize = 16;
/// Bytes per painted word.
const WORD: u64 = 4;

/// Problems turning symbols, SP or margins into a paint.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum StackError {
    /// Neither `_stack_*_cpu0` nor `_stack_*` is in the image.
    #[error("no main stack symbols ({STACK_END_CPU0}/{STACK_START_CPU0} or {STACK_END}/{STACK_START}) in the ELF")]
    NoStackSymbols,
    /// The bounds are reversed, empty or not word aligned.
    #[error("bad stack bounds {bottom:#x}..{top:#x}")]
    BadBounds {
        /// Lowest address.
        bottom: u64,
        /// Highest address.
        top: u64,
    },
    /// The sampled SP is outside the region (e.g. another thread was running).
    #[error("SP {sp:#010x} is outside the stack {bottom:#010x}..{top:#010x}")]
    SpOutside {
        /// Sampled SP.
        sp: u64,
        /// Region bottom.
        bottom: u64,
        /// Region top.
        top: u64,
    },
    /// The limit leaves nothing to paint above the guard.
    #[error("nothing to paint: limit {limit:#010x} is at or below the paintable bottom {bottom:#010x}")]
    NoRoom {
        /// Upper bound requested.
        limit: u64,
        /// First paintable address.
        bottom: u64,
    },
}

/// One stack region, `bottom..top` (top exclusive), growing downward.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StackRegion {
    name: &'static str,
    bottom: u64,
    top: u64,
    guard: Option<u64>,
}

impl StackRegion {
    /// A region with word-aligned `bottom < top` and an optional guard word
    /// (kept only if it lies inside the region).
    pub fn new(name: &'static str, bottom: u64, top: u64, guard: Option<u64>) -> Result<Self, StackError> {
        if bottom >= top || !bottom.is_multiple_of(WORD) || !top.is_multiple_of(WORD) {
            return Err(StackError::BadBounds { bottom, top });
        }
        let guard = guard.filter(|g| *g >= bottom && g.saturating_add(WORD) <= top && g.is_multiple_of(WORD));
        Ok(Self { name, bottom, top, guard })
    }

    /// Region label (`cpu0`, `cpu1`).
    pub fn name(&self) -> &'static str {
        self.name
    }

    /// Lowest address (the overflow limit).
    pub fn bottom(&self) -> u64 {
        self.bottom
    }

    /// One past the highest address (initial SP).
    pub fn top(&self) -> u64 {
        self.top
    }

    /// Address of the guard word, if the region has one.
    pub fn guard(&self) -> Option<u64> {
        self.guard
    }

    /// Size in bytes.
    pub fn size(&self) -> u64 {
        self.top - self.bottom
    }

    /// True if `addr` is inside `bottom..=top` (SP may equal top when empty).
    pub fn contains_sp(&self, addr: u64) -> bool {
        addr >= self.bottom && addr <= self.top
    }

    /// First address painting may write: just above the guard word, or the
    /// bottom when there is no guard. The bytes below the guard are left
    /// alone; a stack that reaches them has already hit the guard.
    pub fn paintable_bottom(&self) -> u64 {
        self.guard.map_or(self.bottom, |g| g + WORD)
    }
}

/// Finds the stack regions in `(demangled name, address)` pairs.
///
/// CPU 0 comes first and is required; CPU 1 is added when both of its
/// symbols are present. The guard word belongs to CPU 0.
pub fn regions_from_symbols<'a, I>(symbols: I) -> Result<Vec<StackRegion>, StackError>
where
    I: IntoIterator<Item = (&'a str, u64)>,
{
    let mut get = std::collections::BTreeMap::new();
    for (name, addr) in symbols {
        get.entry(name).or_insert(addr);
    }
    let pick = |a: &str, b: &str| get.get(a).or_else(|| get.get(b)).copied();
    let (Some(bottom), Some(top)) = (pick(STACK_END_CPU0, STACK_END), pick(STACK_START_CPU0, STACK_START)) else {
        return Err(StackError::NoStackSymbols);
    };
    let mut regions = vec![StackRegion::new("cpu0", bottom, top, get.get(STACK_GUARD).copied())?];
    if let (Some(&bottom), Some(&top)) = (get.get(STACK_END_CPU1), get.get(STACK_START_CPU1)) {
        regions.push(StackRegion::new("cpu1", bottom, top, None)?);
    }
    Ok(regions)
}

/// Upper bound (exclusive) of a paint below a sampled `sp`, keeping
/// `margin` bytes free under it.
pub fn paint_limit(region: &StackRegion, sp: u64, margin: u64) -> Result<u64, StackError> {
    if !region.contains_sp(sp) {
        return Err(StackError::SpOutside { sp, bottom: region.bottom, top: region.top });
    }
    Ok(sp.saturating_sub(margin))
}

/// The words to paint: `start..end`, both word aligned.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PaintPlan {
    /// First painted address.
    pub start: u64,
    /// One past the last painted address.
    pub end: u64,
}

impl PaintPlan {
    /// Painted bytes.
    pub fn len(&self) -> u64 {
        self.end - self.start
    }

    /// Painted words.
    pub fn words(&self) -> usize {
        usize::try_from(self.len() / WORD).unwrap_or(usize::MAX)
    }
}

/// Clips a paint with upper bound `limit` (exclusive) to the paintable part
/// of `region`, rounding the bound down to a word.
pub fn plan_paint(region: &StackRegion, limit: u64) -> Result<PaintPlan, StackError> {
    let start = region.paintable_bottom();
    let end = (limit.min(region.top) / WORD) * WORD;
    if end <= start {
        return Err(StackError::NoRoom { limit, bottom: start });
    }
    Ok(PaintPlan { start, end })
}

/// Result of scanning painted words from the bottom upward.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Watermark {
    /// Address of the first scanned word.
    pub base: u64,
    /// Unpainted words below the start of the paint (all of them when no
    /// paint was found).
    pub skipped_words: usize,
    /// Pattern words from the start of the paint up to the first change.
    pub intact_words: usize,
    /// Words scanned.
    pub scanned_words: usize,
}

impl Watermark {
    /// Lowest address that is no longer pattern: the deepest point the
    /// stack reached, or the end of the scan if every word is intact.
    pub fn first_used(&self) -> u64 {
        self.base + (self.skipped_words + self.intact_words) as u64 * WORD
    }

    /// Address where the paint starts.
    pub fn paint_start(&self) -> u64 {
        self.base + self.skipped_words as u64 * WORD
    }

    /// One past the last scanned address.
    pub fn scan_end(&self) -> u64 {
        self.base + self.scanned_words as u64 * WORD
    }

    /// True if every scanned word still holds the pattern.
    pub fn all_intact(&self) -> bool {
        self.intact_words > 0 && self.skipped_words + self.intact_words == self.scanned_words
    }
}

/// Finds the paint in `words` (read from `base` upward): the first run of
/// [`MIN_PAINT_RUN`] pattern words (or a shorter run reaching the end), and
/// how many pattern words follow from there before the first change.
pub fn find_watermark(base: u64, words: &[u32], pattern: u32) -> Watermark {
    let is_run = |i: usize| words.get(i..).is_some_and(|rest| {
        let n = rest.len().min(MIN_PAINT_RUN);
        n > 0 && rest.iter().take(n).all(|w| *w == pattern)
    });
    let skipped_words = (0..words.len()).find(|&i| is_run(i)).unwrap_or(words.len());
    let intact_words = words.get(skipped_words..).map_or(0, |rest| rest.iter().take_while(|w| **w == pattern).count());
    Watermark { base, skipped_words, intact_words, scanned_words: words.len() }
}

/// What a scan says about the stack.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    /// No run of pattern words anywhere: the stack was never painted (or
    /// with another pattern), or it grew down through the whole paint.
    NotPainted,
    /// The stack reached [`Watermark::first_used`].
    Reached,
    /// Nothing below the paint top was touched: the peak is shallower than
    /// the paint (peak use is at most `top - painted_top`).
    BelowPaint,
}

/// Guard word state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Guard {
    /// The region has no guard symbol.
    Absent,
    /// The word holds [`GUARD_VALUE`].
    Intact(u64),
    /// The word holds something else.
    Changed {
        /// Address.
        addr: u64,
        /// Value read.
        value: u32,
    },
}

/// Classifies a guard word read from `addr`.
pub fn guard_state(addr: Option<u64>, value: Option<u32>) -> Guard {
    match (addr, value) {
        (Some(addr), Some(GUARD_VALUE)) => Guard::Intact(addr),
        (Some(addr), Some(value)) => Guard::Changed { addr, value },
        _ => Guard::Absent,
    }
}

/// A stack report: region, scan and guard.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StackReport {
    /// The region.
    pub region: StackRegion,
    /// The scan of its paintable part.
    pub watermark: Watermark,
    /// Pattern looked for.
    pub pattern: u32,
    /// Top of the last paint, if the caller knows it (`stack paint` prints it).
    pub painted_top: Option<u64>,
    /// Guard word.
    pub guard: Guard,
}

impl StackReport {
    /// Classification of the scan.
    pub fn verdict(&self) -> Verdict {
        let first = self.watermark.first_used();
        if self.watermark.intact_words == 0 {
            Verdict::NotPainted
        } else if self.watermark.all_intact() || self.painted_top.is_some_and(|t| first >= t) {
            Verdict::BelowPaint
        } else {
            Verdict::Reached
        }
    }

    /// Peak bytes used: from the deepest changed word to the top. For
    /// [`Verdict::BelowPaint`] this is an upper bound.
    pub fn used(&self) -> u64 {
        let first = match (self.verdict(), self.painted_top) {
            (Verdict::BelowPaint, Some(t)) => t.min(self.watermark.first_used()),
            _ => self.watermark.first_used(),
        };
        self.region.top.saturating_sub(first)
    }

    /// Untouched bytes between the guard (or bottom) and the deepest use.
    pub fn margin(&self) -> u64 {
        self.region.top.saturating_sub(self.used()).saturating_sub(self.region.paintable_bottom())
    }

    /// Peak use as a whole percentage of the region.
    pub fn percent(&self) -> u64 {
        self.used() * 100 / self.region.size()
    }

    /// True when the report shows a problem (guard changed or not painted).
    pub fn is_alarm(&self) -> bool {
        matches!(self.guard, Guard::Changed { .. }) || self.verdict() == Verdict::NotPainted
    }

    /// Human-readable rendering.
    pub fn render(&self) -> String {
        let r = &self.region;
        let mut out = String::new();
        let _ = writeln!(out, "stack {}     {:#010x}..{:#010x} ({} bytes)", r.name, r.bottom, r.top, r.size());
        let _ = writeln!(out, "guard          {}", guard_text(self.guard));
        let _ = write!(
            out,
            "scanned        {:#010x}..{:#010x}, pattern {:#010x}, {} words intact",
            self.watermark.base,
            self.watermark.scan_end(),
            self.pattern,
            self.watermark.intact_words
        );
        if self.watermark.skipped_words > 0 && self.watermark.intact_words > 0 {
            let _ = write!(
                out,
                " from {:#010x} ({} unpainted bytes below)",
                self.watermark.paint_start(),
                self.watermark.skipped_words as u64 * WORD
            );
        }
        out.push('\n');
        let line = match self.verdict() {
            Verdict::NotPainted => format!(
                "high-water     unknown: no run of {MIN_PAINT_RUN} pattern words (not painted, or the stack grew through the whole paint)"
            ),
            Verdict::Reached => {
                let mut line = format!(
                    "high-water     {:#010x}: {} bytes used ({}%), margin {} bytes",
                    self.watermark.first_used(),
                    self.used(),
                    self.percent(),
                    self.margin()
                );
                if self.painted_top.is_none() {
                    let _ = write!(
                        line,
                        "\nnote           if {:#010x} is the top of the last paint, the stack never reached \
                         the paint (pass --painted-top)",
                        self.watermark.first_used()
                    );
                }
                line
            }
            Verdict::BelowPaint => format!(
                "high-water     above the paint: at most {} bytes used ({}%), margin at least {} bytes",
                self.used(),
                self.percent(),
                self.margin()
            ),
        };
        out.push_str(&line);
        out
    }

    /// JSON rendering.
    pub fn to_json(&self) -> Value {
        let r = &self.region;
        let (guard_addr, guard_ok, guard_value) = match self.guard {
            Guard::Absent => (Value::Null, Value::Null, Value::Null),
            Guard::Intact(a) => (a.into(), true.into(), GUARD_VALUE.into()),
            Guard::Changed { addr, value } => (addr.into(), false.into(), value.into()),
        };
        let verdict = match self.verdict() {
            Verdict::NotPainted => "not_painted",
            Verdict::Reached => "reached",
            Verdict::BelowPaint => "below_paint",
        };
        let known = self.verdict() != Verdict::NotPainted;
        json!({
            "region": r.name, "bottom": r.bottom, "top": r.top, "size": r.size(),
            "guard": { "addr": guard_addr, "ok": guard_ok, "value": guard_value },
            "pattern": self.pattern, "scan_start": self.watermark.base, "scan_end": self.watermark.scan_end(),
            "intact_words": self.watermark.intact_words,
            "paint_start": (self.watermark.intact_words > 0).then(|| self.watermark.paint_start()), "painted_top": self.painted_top,
            "verdict": verdict,
            "high_water": known.then(|| self.watermark.first_used()),
            "used": known.then(|| self.used()),
            "margin": known.then(|| self.margin()),
            "percent": known.then(|| self.percent()),
        })
    }
}

fn guard_text(guard: Guard) -> String {
    match guard {
        Guard::Absent => "none".into(),
        Guard::Intact(a) => format!("{a:#010x} = {GUARD_VALUE:#010x} (ok)"),
        Guard::Changed { addr, value } => {
            format!("{addr:#010x} = {value:#010x} (CHANGED, expected {GUARD_VALUE:#010x})")
        }
    }
}

/// One-line summary of a paint (the first line `stack paint` prints;
/// `scripts/stack-watermark.sh` parses the `start..end` pair).
pub fn paint_summary(region: &StackRegion, plan: &PaintPlan, pattern: u32, sp: Option<u64>, margin: u64) -> String {
    let source = match sp {
        Some(sp) => format!("SP {sp:#010x} - margin {margin}"),
        None => "--top".into(),
    };
    format!(
        "painted {:#010x}..{:#010x} ({} bytes of {} in stack {}) with {pattern:#010x}; limit from {source}",
        plan.start,
        plan.end,
        plan.len(),
        region.size(),
        region.name
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    const DEFAULT_PATTERN: u32 = 0xA5A5_A5A5;

    // Values from the badge's release ELF (esp-hal 1.2.2).
    const BOTTOM: u64 = 0x3fcc_bbc0;
    const TOP: u64 = 0x3fcd_b700;
    const GUARD: u64 = 0x3fcc_bbfc;

    fn region() -> StackRegion {
        StackRegion::new("cpu0", BOTTOM, TOP, Some(GUARD)).unwrap()
    }

    #[test]
    fn finds_cpu0_region_and_guard_from_symbols() {
        let syms = [
            ("_stack_end", BOTTOM),
            (STACK_END_CPU0, BOTTOM),
            (STACK_START_CPU0, TOP),
            ("_stack_start", TOP),
            (STACK_GUARD, GUARD),
            ("_bss_end", 0x3fcc_bbbe),
        ];
        let regions = regions_from_symbols(syms).unwrap();
        assert_eq!(regions, vec![region()]);
        assert_eq!(regions[0].size(), 64_320);
        assert_eq!(regions[0].paintable_bottom(), GUARD + 4);
    }

    #[test]
    fn falls_back_to_generic_names_and_adds_cpu1() {
        let syms = [
            (STACK_END, 0x1000),
            (STACK_START, 0x2000),
            (STACK_END_CPU1, 0x3000),
            (STACK_START_CPU1, 0x3800),
        ];
        let regions = regions_from_symbols(syms).unwrap();
        assert_eq!(regions.len(), 2);
        assert_eq!((regions[0].bottom(), regions[0].top(), regions[0].guard()), (0x1000, 0x2000, None));
        assert_eq!((regions[1].name(), regions[1].size()), ("cpu1", 0x800));
    }

    #[test]
    fn missing_or_bad_symbols_are_errors() {
        assert_eq!(regions_from_symbols([("_stack_end_cpu0", 0x1000)]), Err(StackError::NoStackSymbols));
        assert!(matches!(
            regions_from_symbols([(STACK_END_CPU0, 0x2000), (STACK_START_CPU0, 0x1000)]),
            Err(StackError::BadBounds { .. })
        ));
        assert!(StackRegion::new("x", 0x1001, 0x2000, None).is_err());
        // A guard outside the region is ignored.
        assert_eq!(StackRegion::new("x", 0x1000, 0x2000, Some(0x2000)).unwrap().guard(), None);
    }

    #[test]
    fn paint_stays_below_sp_minus_margin_and_above_guard() {
        let r = region();
        let sp = 0x3fcd_8001;
        let limit = paint_limit(&r, sp, DEFAULT_MARGIN).unwrap();
        assert_eq!(limit, sp - DEFAULT_MARGIN);
        let plan = plan_paint(&r, limit).unwrap();
        assert_eq!(plan.start, GUARD + 4);
        assert_eq!(plan.end, 0x3fcd_4000);
        assert!(plan.end <= sp - DEFAULT_MARGIN);
        assert_eq!(plan.end % 4, 0);
        assert_eq!(plan.words() as u64 * 4, plan.len());
        // The guard word itself is never in the plan.
        assert!(plan.start > GUARD);
    }

    #[test]
    fn paint_rejects_bad_sp_and_no_room() {
        let r = region();
        assert!(matches!(paint_limit(&r, 0x3fc9_0000, 0), Err(StackError::SpOutside { .. })));
        assert!(matches!(paint_limit(&r, TOP + 4, 0), Err(StackError::SpOutside { .. })));
        let limit = paint_limit(&r, BOTTOM + 0x100, DEFAULT_MARGIN).unwrap();
        assert!(matches!(plan_paint(&r, limit), Err(StackError::NoRoom { .. })));
        assert!(matches!(plan_paint(&r, GUARD + 4), Err(StackError::NoRoom { .. })));
        // A limit above the top is clipped to the top.
        assert_eq!(plan_paint(&r, TOP + 0x1000).unwrap().end, TOP);
    }

    #[test]
    fn watermark_counts_leading_pattern_words() {
        let p = DEFAULT_PATTERN;
        let mut words = vec![p; 20];
        words.push(7);
        words.extend([p; 30]);
        let wm = find_watermark(0x1000, &words, p);
        assert_eq!((wm.skipped_words, wm.intact_words, wm.first_used()), (0, 20, 0x1050));
        assert_eq!(wm.scan_end(), 0x1000 + 51 * 4);
        assert!(!wm.all_intact());
        let wm = find_watermark(0x1000, &[p, p], p);
        assert!(wm.all_intact());
        assert_eq!(find_watermark(0x1000, &[0, 0], p).intact_words, 0);
        assert!(!find_watermark(0x1000, &[], p).all_intact());
        assert_eq!(find_watermark(0x1000, &[], p).first_used(), 0x1000);
    }

    #[test]
    fn watermark_skips_unpainted_bottom_and_stray_pattern_words() {
        let p = DEFAULT_PATTERN;
        // Junk, a stray pattern word, junk, then a 40-word paint and live data.
        let mut words = vec![0, 1, p, 2, 3];
        words.extend([p; 40]);
        words.extend([9, p, 9]);
        let wm = find_watermark(0x1000, &words, p);
        assert_eq!((wm.skipped_words, wm.intact_words), (5, 40));
        assert_eq!(wm.paint_start(), 0x1014);
        assert_eq!(wm.first_used(), 0x1014 + 160);
        // A run shorter than MIN_PAINT_RUN in the middle is not a paint.
        let mut words = vec![0; 4];
        words.extend([p; MIN_PAINT_RUN - 1]);
        words.push(0);
        assert_eq!(find_watermark(0x1000, &words, p).intact_words, 0);
    }

    fn report(intact: usize, scanned: usize, painted_top: Option<u64>, guard: Option<u32>) -> StackReport {
        let r = region();
        StackReport {
            watermark: Watermark {
                base: r.paintable_bottom(),
                skipped_words: 0,
                intact_words: intact,
                scanned_words: scanned,
            },
            guard: guard_state(r.guard(), guard),
            region: r,
            pattern: DEFAULT_PATTERN,
            painted_top,
        }
    }

    #[test]
    fn report_reached_gives_used_and_margin() {
        let scanned = ((TOP - (GUARD + 4)) / 4) as usize;
        let rep = report(2_000, scanned, Some(0x3fcd_4000), Some(GUARD_VALUE));
        assert_eq!(rep.verdict(), Verdict::Reached);
        let first = GUARD + 4 + 8_000;
        assert_eq!(rep.used(), TOP - first);
        assert_eq!(rep.margin(), 8_000);
        assert_eq!(rep.used() + rep.margin() + 64, rep.region.size());
        assert!(!rep.is_alarm());
        let text = rep.render();
        assert!(text.contains(&format!("high-water     {first:#010x}: {} bytes used", rep.used())), "{text}");
        assert!(text.contains("margin 8000 bytes"), "{text}");
        assert!(text.contains("= 0xdeedbaad (ok)"), "{text}");
        let j = rep.to_json();
        assert_eq!(j["verdict"], "reached");
        assert_eq!(j["margin"], 8_000);
        assert_eq!(j["guard"]["ok"], true);
    }

    #[test]
    fn report_with_late_paint_start_and_unknown_top_adds_a_note() {
        let scanned = ((TOP - (GUARD + 4)) / 4) as usize;
        let mut rep = report(1_000, scanned, None, Some(GUARD_VALUE));
        rep.watermark.skipped_words = 500;
        assert_eq!(rep.verdict(), Verdict::Reached);
        let first = GUARD + 4 + 6_000;
        assert_eq!(rep.watermark.first_used(), first);
        // Margin counts from the guard: the stack never got below the paint.
        assert_eq!(rep.margin(), 6_000);
        let text = rep.render();
        assert!(text.contains(&format!("1000 words intact from {:#010x} (2000 unpainted bytes below)", GUARD + 4 + 2_000)), "{text}");
        assert!(text.contains("note           if"), "{text}");
        assert_eq!(rep.to_json()["paint_start"], GUARD + 4 + 2_000);
    }

    #[test]
    fn report_below_paint_is_an_upper_bound() {
        let painted_top = 0x3fcd_4000;
        let words = ((painted_top - (GUARD + 4)) / 4) as usize;
        // Scan covers the whole region; the first change is the paint top.
        let scanned = ((TOP - (GUARD + 4)) / 4) as usize;
        let rep = report(words, scanned, Some(painted_top), Some(GUARD_VALUE));
        assert_eq!(rep.verdict(), Verdict::BelowPaint);
        assert_eq!(rep.used(), TOP - painted_top);
        assert!(rep.render().contains("above the paint: at most"));
        // Without a known paint top, an all-intact scan is also below the paint.
        let rep = report(words, words, None, Some(GUARD_VALUE));
        assert_eq!(rep.verdict(), Verdict::BelowPaint);
        assert_eq!(rep.to_json()["verdict"], "below_paint");
    }

    #[test]
    fn report_not_painted_and_guard_changed_are_alarms() {
        let rep = report(0, 100, None, Some(GUARD_VALUE));
        assert_eq!(rep.verdict(), Verdict::NotPainted);
        assert!(rep.is_alarm());
        assert!(rep.render().contains("unknown"));
        assert_eq!(rep.to_json()["used"], Value::Null);
        let rep = report(10, 100, None, Some(0));
        assert!(rep.is_alarm());
        assert!(rep.render().contains("CHANGED, expected 0xdeedbaad"), "{}", rep.render());
        assert_eq!(rep.to_json()["guard"]["ok"], false);
        assert_eq!(guard_state(None, Some(1)), Guard::Absent);
    }

    #[test]
    fn paint_summary_names_the_range() {
        let r = region();
        let plan = PaintPlan { start: GUARD + 4, end: 0x3fcd_4000 };
        let s = paint_summary(&r, &plan, DEFAULT_PATTERN, Some(0x3fcd_8000), DEFAULT_MARGIN);
        assert!(s.starts_with("painted 0x3fccbc00..0x3fcd4000 ("), "{s}");
        assert!(s.contains("SP 0x3fcd8000 - margin 16384"), "{s}");
        assert!(paint_summary(&r, &plan, 1, None, 0).ends_with("limit from --top"));
    }
}
