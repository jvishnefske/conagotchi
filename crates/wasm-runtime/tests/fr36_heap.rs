//! FR-36 heap budget: peak heap of uploading, starting and ticking the demo
//! app, measured with the `peak_alloc` global allocator (its own test binary,
//! so the allocator sees nothing else). The host has 64-bit pointers, so this
//! is an upper bound for the badge's 32-bit build; the firmware's wasm heap
//! region ([`HEAP_BUDGET`] here) is sized from it. Measured both for the
//! demo app and for a synthetic module at the code / data / memory limits.

use peak_alloc::PeakAlloc;
use wasm_abi::{MAX_CODE_BYTES, MAX_DATA_BYTES, MAX_MEMORY_BYTES, Status};
use wasm_runtime::{Host, Runtime};

#[global_allocator]
static PEAK: PeakAlloc = PeakAlloc;

/// Heap any module within the ABI limits may use (the badge's wasm region).
const HEAP_BUDGET: usize = 64 * 1024;
/// Heap the demo app may use.
const DEMO_BUDGET: usize = 44 * 1024;

const HELLO: &[u8] = include_bytes!("../../../apps/hello.wasm");

struct Quiet;

impl Host for Quiet {
    fn now_ms(&mut self) -> i64 {
        0
    }
    fn log(&mut self, _: &str) {}
    fn publish(&mut self, _: &str, _: &[u8]) -> Result<(), Status> {
        Ok(())
    }
    fn set_leds(&mut self, _: &[u8; 24]) -> Result<(), Status> {
        Ok(())
    }
    fn show_text(&mut self, _: &str) -> Result<(), Status> {
        Ok(())
    }
    fn set_pet(&mut self, _: i32) -> Result<(), Status> {
        Ok(())
    }
}

/// Uploads, starts and ticks `module`; returns (loaded, running, peak) heap
/// bytes and checks the runtime frees everything when dropped.
fn measure(module: &[u8]) -> (usize, usize, usize) {
    let base = PEAK.current_usage();
    PEAK.reset_peak_usage();
    let mut rt = Runtime::new();
    let total = u32::try_from(module.len()).unwrap();
    for (i, chunk) in module.chunks(192).enumerate() {
        let result = rt.load_chunk(u32::try_from(i * 192).unwrap(), total, chunk);
        assert_eq!(result, Ok(()), "{}", rt.status().error);
    }
    let loaded = PEAK.current_usage() - base;
    rt.start(&mut Quiet).unwrap();
    for t in 0..200 {
        rt.tick(&mut Quiet, t * 20).unwrap();
        rt.deliver("app/hello", &[1, 2, 3, 4]);
    }
    let running = PEAK.current_usage() - base;
    let peak = PEAK.peak_usage() - base;
    drop(rt);
    assert_eq!(PEAK.current_usage(), base, "runtime leaked");
    (loaded, running, peak)
}

/// A module at the limits: ~`MAX_CODE_BYTES` of code in 30 functions
/// calling each other, `MAX_DATA_BYTES` of data, `MAX_MEMORY_BYTES` memory.
fn worst_case() -> Vec<u8> {
    let mut funcs = String::new();
    for f in 0..30 {
        let mut body = String::new();
        for i in 0..4 {
            body += &format!(
                "local.get 0 i32.const {i} i32.add local.tee 1 local.get 1 i32.load offset=4 local.get 1 i32.mul i32.store \
                 local.get 1 i32.const 3 i32.and (if (then local.get 0 call $f{next} drop)) ",
                next = (f + 1) % 30
            );
        }
        funcs += &format!("(func $f{f} (param i32) (result i32) (local i32) {body} local.get 1)\n");
    }
    let data = "x".repeat(MAX_DATA_BYTES - 32);
    let module = wat::parse_str(format!(
        r#"(module
            (memory (export "memory") {MAX_MEMORY_BYTES} (pagesize 1))
            (data (i32.const 0) "{data}")
            {funcs}
            (func (export "init"))
            (func (export "tick") (param i64)))"#
    ))
    .unwrap();
    let (code, _) = wasm_runtime::validate::section_sizes(&module).unwrap();
    assert!(
        code > MAX_CODE_BYTES * 3 / 4,
        "worst case has only {code} B of code"
    );
    module
}

#[test]
fn fr36_heap_stays_within_the_badge_budget() {
    let (loaded, running, peak) = measure(HELLO);
    let (w_loaded, w_running, w_peak) = measure(&worst_case());
    println!("demo app heap: loaded {loaded} B, running {running} B, peak {peak} B");
    println!("worst case heap: loaded {w_loaded} B, running {w_running} B, peak {w_peak} B");
    assert!(
        peak <= DEMO_BUDGET,
        "demo peak {peak} B over {DEMO_BUDGET} B"
    );
    assert!(
        w_peak <= HEAP_BUDGET,
        "worst-case peak {w_peak} B over {HEAP_BUDGET} B"
    );
}
