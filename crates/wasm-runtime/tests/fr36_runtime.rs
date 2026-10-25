//! FR-36 wasm runtime tests: real modules (the committed demo app
//! `apps/hello.wasm` and small WAT modules) run through the public
//! [`Runtime`] API against a recording [`Host`].
//!
//! `apps/hello.wasm` is regenerated with `scripts/build-apps.sh`;
//! `scripts/build-apps.sh --check` verifies the committed artifact matches
//! the sources.

use badge_proto::WasmState;
use wasm_abi::{
    FUEL_PER_CALL, MAX_CODE_BYTES, MAX_DATA_BYTES, MAX_MEMORY_BYTES, MAX_MODULE, MAX_SUBSCRIPTIONS,
    RECV_QUEUE, Status,
};
use wasm_runtime::{DEDUP_MS, Host, Runtime, RuntimeError};

const HELLO: &[u8] = include_bytes!("../../../apps/hello.wasm");

/// Records every host call.
#[derive(Default)]
struct Fake {
    now: i64,
    logs: Vec<String>,
    published: Vec<(String, Vec<u8>)>,
    leds: Vec<[u8; 24]>,
    texts: Vec<String>,
    pets: Vec<i32>,
    fail_publish: bool,
}

impl Host for Fake {
    fn now_ms(&mut self) -> i64 {
        self.now
    }
    fn log(&mut self, line: &str) {
        self.logs.push(line.to_string());
    }
    fn publish(&mut self, key: &str, payload: &[u8]) -> Result<(), Status> {
        self.published.push((key.to_string(), payload.to_vec()));
        if self.fail_publish {
            Err(Status::Unavailable)
        } else {
            Ok(())
        }
    }
    fn set_leds(&mut self, rgb: &[u8; 24]) -> Result<(), Status> {
        self.leds.push(*rgb);
        Ok(())
    }
    fn show_text(&mut self, text: &str) -> Result<(), Status> {
        self.texts.push(text.to_string());
        Ok(())
    }
    fn set_pet(&mut self, mode: i32) -> Result<(), Status> {
        self.pets.push(mode);
        Ok(())
    }
}

/// Uploads `module` in 192-byte chunks (the `WasmLoad` size).
fn load(rt: &mut Runtime, module: &[u8]) -> Result<(), RuntimeError> {
    let total = u32::try_from(module.len()).unwrap();
    let mut result = Ok(());
    for (i, chunk) in module.chunks(192).enumerate() {
        result = rt.load_chunk(u32::try_from(i * 192).unwrap(), total, chunk);
        if result.is_err() {
            break;
        }
    }
    result
}

fn wat(src: &str) -> Vec<u8> {
    wat::parse_str(src).unwrap()
}

/// A module with the required exports, the given imports / body snippets,
/// a 1 KiB memory with 1-byte pages and `data` at offset 0.
fn app(imports: &str, data: &str, init: &str, tick: &str) -> Vec<u8> {
    wat(&format!(
        r#"(module
            {imports}
            (memory (export "memory") 1024 1024 (pagesize 1))
            (data (i32.const 0) "{data}")
            (func (export "init") {init})
            (func (export "tick") (param $now i64) {tick}))"#
    ))
}

const LOG: &str = r#"(import "badge" "log" (func $log (param i32 i32)))"#;
const SUB: &str = r#"(import "badge" "subscribe" (func $sub (param i32 i32) (result i32)))"#;
const PUB: &str = r#"(import "badge" "publish" (func $pub (param i32 i32 i32 i32) (result i32)))"#;
const RECV: &str = r#"(import "badge" "recv" (func $recv (param i32 i32) (result i32)))"#;
const TEXT: &str = r#"(import "badge" "show_text" (func $text (param i32 i32) (result i32)))"#;
const LEDS: &str = r#"(import "badge" "set_leds" (func $leds (param i32) (result i32)))"#;
const PET: &str = r#"(import "badge" "set_pet" (func $pet (param i32) (result i32)))"#;

/// Traps unless the i32 on the stack equals `expected`.
fn expect(expected: i32) -> String {
    format!("i32.const {expected} i32.ne (if (then unreachable))")
}

/// Runs a module whose `init` asserts with [`expect`]; Ok if init passed.
fn run_init(module: &[u8]) -> (Runtime, Fake, Result<(), RuntimeError>) {
    let mut rt = Runtime::new();
    let mut host = Fake::default();
    load(&mut rt, module).unwrap();
    let result = rt.start(&mut host);
    (rt, host, result)
}

#[test]
fn fr36_demo_app_runs_end_to_end() {
    assert!(
        HELLO.len() <= MAX_MODULE,
        "demo app too big: {}",
        HELLO.len()
    );
    let mut rt = Runtime::new();
    let mut host = Fake {
        now: 1_000,
        ..Fake::default()
    };
    load(&mut rt, HELLO).unwrap();
    assert_eq!(rt.status().state, WasmState::Loaded);
    assert_eq!(rt.status().size, u32::try_from(HELLO.len()).unwrap());

    rt.start(&mut host).unwrap();
    assert_eq!(host.logs, ["hello from wasm"]);
    assert_eq!(host.texts.len(), 1);
    assert!(host.texts[0].starts_with("hello from wasm"));
    assert!(rt.is_subscribed("app/hello"));
    assert_eq!(rt.subscriptions().collect::<Vec<_>>(), ["app/hello"]);

    // 2.1 s of 20 ms ticks: the dot moves every 100 ms, one publish at 2 s.
    for t in (1_000..=3_100).step_by(20) {
        rt.tick(&mut host, t).unwrap();
    }
    assert!(host.leds.len() >= 20, "LED frames: {}", host.leds.len());
    let lit = |f: &[u8; 24]| f.chunks(3).position(|px| px != [0, 0, 0]);
    assert_eq!(lit(&host.leds[0]), Some(0));
    assert_eq!(lit(&host.leds[1]), Some(1));
    assert_eq!(
        host.published,
        [("app/hello".to_string(), 1u32.to_le_bytes().to_vec())]
    );

    // A received sample is logged on the next tick.
    assert!(rt.deliver("app/hello", &7u32.to_le_bytes()));
    assert!(!rt.deliver("app/other", b"x"), "not subscribed");
    rt.tick(&mut host, 3_120).unwrap();
    assert_eq!(host.logs.last().map(String::as_str), Some("rx 7"));

    let status = rt.status();
    assert_eq!(status.state, WasmState::Running);
    assert_eq!(status.ticks, 107);
    assert!(status.fuel_used > 0 && status.fuel_used < 108 * FUEL_PER_CALL);
    assert!(status.error.is_empty());

    assert!(rt.is_running());
    rt.stop();
    assert!(!rt.is_running());
    assert_eq!(rt.status().state, WasmState::Loaded);
    assert_eq!(rt.tick(&mut host, 4_000), Err(RuntimeError::NotRunning));
    assert!(!rt.deliver("app/hello", b"x"));
    assert_eq!(rt.subscriptions().count(), 0);
}

#[test]
fn fr36_upload_state_machine() {
    let module = app("", "", "", "");
    let total = u32::try_from(module.len()).unwrap();
    let mut rt = Runtime::new();
    assert_eq!(rt.status().state, WasmState::Empty);
    assert_eq!(rt.start(&mut Fake::default()), Err(RuntimeError::NotLoaded));
    // No upload in progress.
    assert_eq!(
        rt.load_chunk(4, total, &module[4..8]),
        Err(RuntimeError::BadChunk)
    );
    // Size limits.
    assert_eq!(rt.load_chunk(0, 0, &[]), Err(RuntimeError::BadChunk));
    let over = u32::try_from(MAX_MODULE + 1).unwrap();
    assert_eq!(
        rt.load_chunk(0, over, &module[..4]),
        Err(RuntimeError::BadChunk)
    );

    rt.load_chunk(0, total, &module[..4]).unwrap();
    assert_eq!(rt.status().state, WasmState::Loading);
    assert_eq!(rt.status().size, 4);
    // Out of order, gap, other total, past the end: rejected, upload kept.
    assert_eq!(
        rt.load_chunk(8, total, &module[8..12]),
        Err(RuntimeError::BadChunk)
    );
    assert_eq!(
        rt.load_chunk(4, total + 1, &module[4..8]),
        Err(RuntimeError::BadChunk)
    );
    assert_eq!(
        rt.load_chunk(4, total, &[0; 1000]),
        Err(RuntimeError::BadChunk)
    );
    assert_eq!(rt.status().size, 4);
    rt.load_chunk(4, total, &module[4..]).unwrap();
    assert_eq!(rt.status().state, WasmState::Loaded);

    // Offset 0 restarts, also while running.
    let mut host = Fake::default();
    rt.start(&mut host).unwrap();
    assert_eq!(rt.status().state, WasmState::Running);
    rt.load_chunk(0, total, &module[..10]).unwrap();
    assert_eq!(rt.status().state, WasmState::Loading);
    assert_eq!(rt.tick(&mut host, 0), Err(RuntimeError::NotRunning));
    rt.load_chunk(10, total, &module[10..]).unwrap();
    assert_eq!(rt.status().state, WasmState::Loaded);

    // Garbage fails validation: EMPTY with a reason.
    assert_eq!(
        load(&mut rt, b"\0asm\x01\0\0\0garbage"),
        Err(RuntimeError::Invalid)
    );
    assert_eq!(rt.status().state, WasmState::Empty);
    assert!(!rt.status().error.is_empty());
}

#[test]
fn fr36_trap_stops_the_app_and_restart_recovers() {
    let module = app(
        "",
        "",
        "",
        "local.get $now i64.const 100 i64.ge_s (if (then unreachable))",
    );
    let mut rt = Runtime::new();
    let mut host = Fake::default();
    load(&mut rt, &module).unwrap();
    rt.start(&mut host).unwrap();
    rt.tick(&mut host, 50).unwrap();
    assert_eq!(rt.tick(&mut host, 100), Err(RuntimeError::Trapped));
    let status = rt.status();
    assert_eq!(status.state, WasmState::Trapped);
    assert!(status.error.contains("trap"), "{}", status.error);
    assert_eq!(rt.tick(&mut host, 120), Err(RuntimeError::NotRunning));
    // Start again: fresh instance, counters reset.
    rt.start(&mut host).unwrap();
    assert_eq!(rt.status().state, WasmState::Running);
    assert_eq!(rt.status().ticks, 0);
    assert!(rt.status().error.is_empty());
}

#[test]
fn fr36_init_trap_reports_trapped() {
    let (rt, _, result) = run_init(&app("", "", "unreachable", ""));
    assert_eq!(result, Err(RuntimeError::Trapped));
    assert_eq!(rt.status().state, WasmState::Trapped);
}

#[test]
fn fr36_fuel_exhaustion_is_a_trap() {
    let module = app("", "", "", "(loop $l (br $l))");
    let mut rt = Runtime::new();
    let mut host = Fake::default();
    load(&mut rt, &module).unwrap();
    rt.start(&mut host).unwrap();
    assert_eq!(rt.tick(&mut host, 0), Err(RuntimeError::Trapped));
    let status = rt.status();
    assert_eq!(status.state, WasmState::Trapped);
    assert_eq!(status.error.as_str(), "out of fuel");
    assert!(
        status.fuel_used >= FUEL_PER_CALL - 10,
        "{}",
        status.fuel_used
    );
}

#[test]
fn fr36_out_of_bounds_pointers_are_rejected() {
    let oob = Status::OutOfBounds.code();
    let imports = [LOG, PUB, SUB, RECV, TEXT, LEDS].concat();
    let init = [
        // Past the end, straddling the end, negative length, negative pointer.
        format!("i32.const 2000 i32.const 4 call $text {}", expect(oob)),
        format!("i32.const 1020 i32.const 8 call $text {}", expect(oob)),
        format!("i32.const 0 i32.const -1 call $sub {}", expect(oob)),
        format!(
            "i32.const -4 i32.const 4 i32.const 0 i32.const 0 call $pub {}",
            expect(oob)
        ),
        format!(
            "i32.const 0 i32.const 5 i32.const 1022 i32.const 4 call $pub {}",
            expect(oob)
        ),
        format!("i32.const 1010 call $leds {}", expect(oob)),
        format!("i32.const 1023 i32.const 10 call $recv {}", expect(oob)),
        // log has no result: a bad pointer is ignored.
        "i32.const 5000 i32.const 1 call $log".to_string(),
    ]
    .concat();
    let (rt, host, result) = run_init(&app(&imports, "app/x", &init, ""));
    assert_eq!(result, Ok(()), "{}", rt.status().error);
    assert!(host.logs.is_empty() && host.texts.is_empty() && host.leds.is_empty());
    assert!(host.published.is_empty());
}

#[test]
fn fr36_bad_keys_are_rejected() {
    let bad = Status::BadKey.code();
    // Data: "conagotchi/sync" @0 (15), "app/a b" @15 (7), "app/" @22 (4), "app/ok" @26 (6).
    let data = "conagotchi/syncapp/a bapp/app/ok";
    let init = [
        format!("i32.const 0 i32.const 15 call $sub {}", expect(bad)),
        format!("i32.const 15 i32.const 7 call $sub {}", expect(bad)),
        format!("i32.const 22 i32.const 4 call $sub {}", expect(bad)),
        format!(
            "i32.const 0 i32.const 15 i32.const 0 i32.const 1 call $pub {}",
            expect(bad)
        ),
        format!("i32.const 26 i32.const 6 call $sub {}", expect(0)),
        format!("i32.const 26 i32.const 6 call $sub {}", expect(0)),
        format!(
            "i32.const 26 i32.const 6 i32.const 0 i32.const 2 call $pub {}",
            expect(0)
        ),
        // Payload over MAX_PAYLOAD.
        format!(
            "i32.const 26 i32.const 6 i32.const 0 i32.const 65 call $pub {}",
            expect(Status::TooLong.code())
        ),
    ]
    .concat();
    let (rt, host, result) = run_init(&app(&[SUB, PUB].concat(), data, &init, ""));
    assert_eq!(result, Ok(()), "{}", rt.status().error);
    assert_eq!(rt.subscriptions().collect::<Vec<_>>(), ["app/ok"]);
    assert_eq!(host.published, [("app/ok".to_string(), b"co".to_vec())]);
}

#[test]
fn fr36_missing_export_is_invalid() {
    let no_tick =
        wat(r#"(module (memory (export "memory") 16 (pagesize 1)) (func (export "init")))"#);
    let mut rt = Runtime::new();
    assert_eq!(load(&mut rt, &no_tick), Err(RuntimeError::Invalid));
    assert_eq!(rt.status().state, WasmState::Empty);
    assert_eq!(rt.status().error.as_str(), "missing export tick");

    let bad_tick = wat(
        r#"(module (memory (export "memory") 16 (pagesize 1)) (func (export "init")) (func (export "tick") (param i32)))"#,
    );
    assert_eq!(load(&mut rt, &bad_tick), Err(RuntimeError::Invalid));
    let no_memory = wat(r#"(module (func (export "init")) (func (export "tick") (param i64)))"#);
    assert_eq!(load(&mut rt, &no_memory), Err(RuntimeError::Invalid));
    assert_eq!(rt.status().error.as_str(), "missing export memory");
}

#[test]
fn fr36_bad_imports_are_invalid() {
    let mut rt = Runtime::new();
    for import in [
        r#"(import "env" "log" (func (param i32 i32)))"#,
        r#"(import "badge" "nope" (func))"#,
        r#"(import "badge" "log" (func (param i32) (result i32)))"#,
        r#"(import "badge" "now_ms" (global i64))"#,
    ] {
        assert_eq!(
            load(&mut rt, &app(import, "", "", "")),
            Err(RuntimeError::Invalid),
            "{import}"
        );
        assert!(
            rt.status().error.starts_with("bad import"),
            "{}",
            rt.status().error
        );
    }
}

#[test]
fn fr36_memory_and_feature_limits() {
    let mut rt = Runtime::new();
    let module = |mem: &str| {
        wat(&format!(
            r#"(module (memory (export "memory") {mem}) (func (export "init")) (func (export "tick") (param i64)))"#
        ))
    };
    // A classic 64 KiB page exceeds the limit.
    let max = MAX_MEMORY_BYTES;
    assert_eq!(load(&mut rt, &module("1")), Err(RuntimeError::Invalid));
    assert_eq!(
        load(&mut rt, &module(&format!("{} (pagesize 1)", max + 1))),
        Err(RuntimeError::Invalid)
    );
    load(&mut rt, &module(&format!("{max} (pagesize 1)"))).unwrap();
    // Growing past the limit fails (returns -1) instead of allocating.
    let grow = app(
        "",
        "",
        &format!("i32.const {max} memory.grow {}", expect(-1)),
        "",
    );
    let (rt2, _, result) = run_init(&grow);
    assert_eq!(result, Ok(()), "{}", rt2.status().error);
    // Floats and start functions are rejected.
    let float = app("", "", "f32.const 1 drop", "");
    assert_eq!(load(&mut rt, &float), Err(RuntimeError::Invalid));
    let start = wat(
        r#"(module (memory (export "memory") 16 (pagesize 1)) (func $s) (start $s) (func (export "init")) (func (export "tick") (param i64)))"#,
    );
    assert_eq!(load(&mut rt, &start), Err(RuntimeError::Invalid));
}

#[test]
fn fr36_code_and_data_sections_are_bounded() {
    let mut rt = Runtime::new();
    // Each `i32.const 1 drop` is 3 bytes of code.
    let body = |n: usize| "i32.const 1 drop ".repeat(n / 3);
    load(&mut rt, &app("", "", &body(MAX_CODE_BYTES - 64), "")).unwrap();
    assert_eq!(
        load(&mut rt, &app("", "", &body(MAX_CODE_BYTES + 3), "")),
        Err(RuntimeError::Invalid)
    );
    assert!(
        rt.status().error.starts_with("code "),
        "{}",
        rt.status().error
    );
    let data = |n: usize| {
        wat(&format!(
            r#"(module (memory (export "memory") {MAX_MEMORY_BYTES} (pagesize 1)) (data (i32.const 0) "{}")
                (func (export "init")) (func (export "tick") (param i64)))"#,
            "x".repeat(n)
        ))
    };
    load(&mut rt, &data(MAX_DATA_BYTES - 16)).unwrap();
    assert_eq!(
        load(&mut rt, &data(MAX_DATA_BYTES + 1)),
        Err(RuntimeError::Invalid)
    );
    assert!(
        rt.status().error.starts_with("data "),
        "{}",
        rt.status().error
    );
}

#[test]
fn fr36_host_calls_are_checked_and_flushed() {
    // Data: "app/q" @0, 24 LED bytes @5, "hi" @29, invalid UTF-8 @31.
    let data =
        r"app/q\01\02\03\04\05\06\07\08\09\0a\0b\0c\0d\0e\0f\10\11\12\13\14\15\16\17\18hi\ff";
    let init = [
        format!("i32.const 5 call $leds {}", expect(0)),
        format!("i32.const 29 i32.const 2 call $text {}", expect(0)),
        format!(
            "i32.const 31 i32.const 1 call $text {}",
            expect(Status::BadArgument.code())
        ),
        format!(
            "i32.const 0 i32.const 300 call $text {}",
            expect(Status::TooLong.code())
        ),
        format!("i32.const 3 call $pet {}", expect(0)),
        format!(
            "i32.const 6 call $pet {}",
            expect(Status::BadArgument.code())
        ),
        format!(
            "i32.const -1 call $pet {}",
            expect(Status::BadArgument.code())
        ),
        // Five publishes: the fifth is over MAX_PUBLISH_PER_CALL.
        format!(
            "i32.const 0 i32.const 5 i32.const 29 i32.const 2 call $pub {}",
            expect(0)
        )
        .repeat(4),
        format!(
            "i32.const 0 i32.const 5 i32.const 29 i32.const 2 call $pub {}",
            expect(Status::Full.code())
        ),
        // Six log lines: four are kept.
        "i32.const 29 i32.const 3 call $log ".repeat(6),
    ]
    .concat();
    let imports = [LOG, PUB, TEXT, LEDS, PET].concat();
    let mut rt = Runtime::new();
    let mut host = Fake {
        fail_publish: true,
        ..Fake::default()
    };
    load(&mut rt, &app(&imports, data, &init, "")).unwrap();
    assert_eq!(rt.start(&mut host), Ok(()), "{}", rt.status().error);
    let rgb: Vec<u8> = (1..=24).collect();
    assert_eq!(host.leds, [<[u8; 24]>::try_from(rgb).unwrap()]);
    assert_eq!(host.texts, ["hi"]);
    assert_eq!(host.pets, [3]);
    assert_eq!(host.published.len(), 4);
    assert_eq!(host.logs, ["hi\u{fffd}"; 4]);
    assert_eq!(rt.host_failures(), 4);
}

/// Subscribes to `app/t` in init; each tick pops one sample and logs the raw
/// record (or logs "full" if the buffer is too small for it).
fn receiver() -> Vec<u8> {
    let tick = r#"
        (local $n i32)
        i32.const 64 local.get $now i32.wrap_i64 call $recv local.set $n
        local.get $n i32.const -5 i32.eq (if (then i32.const 6 i32.const 4 call $log))
        local.get $n i32.const 0 i32.gt_s (if (then i32.const 64 local.get $n call $log))"#;
    let init = format!("i32.const 0 i32.const 5 call $sub {}", expect(0));
    app(&[SUB, RECV, LOG].concat(), "app/t full", &init, tick)
}

#[test]
fn fr36_receive_queue_drops_the_oldest() {
    let mut rt = Runtime::new();
    let mut host = Fake::default();
    load(&mut rt, &receiver()).unwrap();
    rt.start(&mut host).unwrap();
    for i in 0..u8::try_from(RECV_QUEUE + 2).unwrap() {
        assert!(rt.deliver("app/t", &[b'a' + i]));
    }
    // The tick's `now` is the recv buffer capacity; 200 fits every record.
    for _ in 0..RECV_QUEUE + 1 {
        rt.tick(&mut host, 200).unwrap();
    }
    let payloads: Vec<char> = host
        .logs
        .iter()
        .map(|l| l.chars().last().unwrap())
        .collect();
    assert_eq!(payloads, ['c', 'd', 'e', 'f', 'g', 'h', 'i', 'j']);
    assert_eq!(host.logs[0], "\u{5}app/tc");
}

#[test]
fn fr36_recv_into_a_small_buffer_keeps_the_sample() {
    let mut rt = Runtime::new();
    let mut host = Fake::default();
    load(&mut rt, &receiver()).unwrap();
    rt.start(&mut host).unwrap();
    assert!(rt.deliver("app/t", b"xyz"));
    // Record is 1 + 5 + 3 = 9 bytes: capacity 8 is too small.
    rt.tick(&mut host, 8).unwrap();
    assert_eq!(host.logs, ["full"]);
    rt.tick(&mut host, 9).unwrap();
    assert_eq!(host.logs, ["full", "\u{5}app/txyz"]);
}

#[test]
fn fr36_duplicate_samples_within_the_window_are_dropped() {
    let mut rt = Runtime::new();
    let mut host = Fake::default();
    load(&mut rt, &receiver()).unwrap();
    rt.start(&mut host).unwrap();
    rt.tick(&mut host, 1).unwrap();
    assert!(rt.deliver("app/t", b"same"));
    assert!(!rt.deliver("app/t", b"same"), "second transport copy");
    assert!(rt.deliver("app/t", b"other"));
    rt.tick(&mut host, 1 + DEDUP_MS - 1).unwrap();
    assert!(!rt.deliver("app/t", b"same"));
    rt.tick(&mut host, 1 + DEDUP_MS).unwrap();
    assert!(rt.deliver("app/t", b"same"), "window passed");
    // Malformed samples are never queued.
    assert!(!rt.deliver("app/t", &[0; 65]));
}

#[test]
fn fr36_subscription_table_is_bounded() {
    // Keys "app/0".."app/4" at offsets 0, 5, 10, 15, 20.
    let data = "app/0app/1app/2app/3app/4";
    let mut init = String::new();
    for i in 0..MAX_SUBSCRIPTIONS {
        init += &format!("i32.const {} i32.const 5 call $sub {}", i * 5, expect(0));
    }
    init += &format!(
        "i32.const 20 i32.const 5 call $sub {}",
        expect(Status::Full.code())
    );
    let (rt, _, result) = run_init(&app(SUB, data, &init, ""));
    assert_eq!(result, Ok(()), "{}", rt.status().error);
    assert_eq!(rt.subscriptions().count(), MAX_SUBSCRIPTIONS);
    assert!(rt.is_subscribed("app/3") && !rt.is_subscribed("app/4"));
}

#[test]
fn fr36_now_ms_is_the_call_time() {
    let imports = r#"(import "badge" "now_ms" (func $now (result i64)))"#;
    let init = "call $now i64.const 1234 i64.ne (if (then unreachable))";
    let tick = "call $now local.get $now i64.ne (if (then unreachable))";
    let mut rt = Runtime::new();
    let mut host = Fake {
        now: 1234,
        ..Fake::default()
    };
    load(&mut rt, &app(imports, "", init, tick)).unwrap();
    rt.start(&mut host).unwrap();
    rt.tick(&mut host, 99_999).unwrap();
}
