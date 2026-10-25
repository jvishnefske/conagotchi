//! One instance of the `badge-sim` wasm32 module, run with wasmi.
//!
//! [`SimModule`] compiles the module once (embedded by `build.rs`, or read
//! from `--sim-wasm`); [`SimModule::instantiate`] gives each simulated badge
//! its own [`Guest`] (own store, memory and statics). Every input method
//! writes its bytes into the module's input region, calls the matching
//! `sim_*` export, and returns the output records the call produced (the
//! `badge_sim::wire` format), copied out of the module's output region.
//! The module has no imports, so it can only compute.

use badge_sim::fb::FB_BYTES;
use wasmi::{Engine, Instance, Linker, Memory, Module, Store, TypedFunc};

/// The module built by `build.rs` (empty if that build failed).
pub const EMBEDDED: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/badge_sim.wasm"));

/// Guest failures.
#[derive(Debug, thiserror::Error)]
pub enum GuestError {
    /// The embedded module is empty: build.rs could not build it.
    #[error(
        "badge-sim.wasm was not built into badgectl (run `rustup target add wasm32-unknown-unknown` and rebuild, \
         or pass --sim-wasm)"
    )]
    NotBuilt,
    /// wasmi rejected the module or a call trapped.
    #[error("wasm: {0}")]
    Wasm(#[from] wasmi::Error),
    /// A required export is missing or has the wrong type.
    #[error("badge-sim module lacks export `{0}`")]
    MissingExport(&'static str),
    /// The input does not fit the module's input region.
    #[error("input of {len} bytes exceeds the {cap}-byte input region")]
    TooLong {
        /// Input length.
        len: usize,
        /// Region capacity.
        cap: usize,
    },
    /// A region lies outside linear memory.
    #[error("module region {0} lies outside its memory")]
    Memory(&'static str),
    /// The export refused the input (not booted, or invalid arguments).
    #[error("`{0}` refused the input")]
    Rejected(&'static str),
}

/// A compiled `badge-sim` module.
pub struct SimModule {
    engine: Engine,
    module: Module,
}

impl SimModule {
    /// Compiles `bytes` (the embedded module when `None`).
    pub fn new(bytes: Option<&[u8]>) -> Result<Self, GuestError> {
        let bytes = bytes.unwrap_or(EMBEDDED);
        if bytes.is_empty() {
            return Err(GuestError::NotBuilt);
        }
        let engine = Engine::default();
        let module = Module::new(&engine, bytes)?;
        Ok(Self { engine, module })
    }

    /// A fresh instance.
    pub fn instantiate(&self) -> Result<Guest, GuestError> {
        let mut store = Store::new(&self.engine, ());
        let linker = Linker::<()>::new(&self.engine);
        let instance = linker.instantiate_and_start(&mut store, &self.module)?;
        Guest::new(store, instance)
    }
}

/// Typed exports.
struct Exports {
    init: TypedFunc<i64, ()>,
    tick: TypedFunc<(i64, i32), i32>,
    host: TypedFunc<i32, i32>,
    radio: TypedFunc<(i32, i32, i32), i32>,
    sync: TypedFunc<(i32, i32, i64), i32>,
    wasm: TypedFunc<(i32, i32), i32>,
    wasm_log: TypedFunc<i32, i32>,
    take: TypedFunc<(), i32>,
    dropped: TypedFunc<(), i32>,
}

/// Regions of the module's memory.
#[derive(Debug, Clone, Copy)]
struct Regions {
    input: usize,
    input_cap: usize,
    output: usize,
    fb: usize,
}

/// One running `badge-sim` instance.
pub struct Guest {
    store: Store<()>,
    memory: Memory,
    f: Exports,
    regions: Regions,
}

fn func<P: wasmi::WasmParams, R: wasmi::WasmResults>(
    store: &Store<()>,
    instance: &Instance,
    name: &'static str,
) -> Result<TypedFunc<P, R>, GuestError> {
    instance.get_typed_func::<P, R>(store, name).map_err(|_| GuestError::MissingExport(name))
}

fn addr(v: i32, name: &'static str) -> Result<usize, GuestError> {
    usize::try_from(v).map_err(|_| GuestError::Memory(name))
}

impl Guest {
    fn new(mut store: Store<()>, instance: Instance) -> Result<Self, GuestError> {
        let memory = instance.get_memory(&store, "memory").ok_or(GuestError::MissingExport("memory"))?;
        let f = Exports {
            init: func(&store, &instance, "sim_init")?,
            tick: func(&store, &instance, "sim_input_tick")?,
            host: func(&store, &instance, "sim_input_host")?,
            radio: func(&store, &instance, "sim_input_radio")?,
            sync: func(&store, &instance, "sim_input_sync")?,
            wasm: func(&store, &instance, "sim_input_wasm")?,
            wasm_log: func(&store, &instance, "sim_input_wasm_log")?,
            take: func(&store, &instance, "sim_take_output")?,
            dropped: func(&store, &instance, "sim_dropped")?,
        };
        let ptr = |name: &'static str, store: &mut Store<()>| -> Result<usize, GuestError> {
            let g: TypedFunc<(), i32> = func(store, &instance, name)?;
            addr(g.call(store, ())?, name)
        };
        let regions = Regions {
            input: ptr("sim_buf_ptr", &mut store)?,
            input_cap: ptr("sim_buf_cap", &mut store)?,
            output: ptr("sim_out_ptr", &mut store)?,
            fb: ptr("sim_framebuffer_ptr", &mut store)?,
        };
        let size = memory.data_size(&store);
        if regions.input + regions.input_cap > size || regions.fb + FB_BYTES > size {
            return Err(GuestError::Memory("input/framebuffer"));
        }
        Ok(Self { store, memory, f, regions })
    }

    fn write_input(&mut self, bytes: &[u8]) -> Result<i32, GuestError> {
        if bytes.len() > self.regions.input_cap {
            return Err(GuestError::TooLong { len: bytes.len(), cap: self.regions.input_cap });
        }
        self.memory
            .write(&mut self.store, self.regions.input, bytes)
            .map_err(|_| GuestError::Memory("input"))?;
        i32::try_from(bytes.len()).map_err(|_| GuestError::TooLong { len: bytes.len(), cap: self.regions.input_cap })
    }

    fn take(&mut self) -> Result<Vec<u8>, GuestError> {
        let len = addr(self.f.take.call(&mut self.store, ())?, "output")?;
        let mut out = vec![0u8; len];
        self.memory
            .read(&self.store, self.regions.output, &mut out)
            .map_err(|_| GuestError::Memory("output"))?;
        Ok(out)
    }

    fn done(&mut self, status: i32, name: &'static str) -> Result<Vec<u8>, GuestError> {
        // The output is drained even when the call was refused.
        let out = self.take()?;
        if status < 0 { Err(GuestError::Rejected(name)) } else { Ok(out) }
    }

    /// Boots the badge with `mac` (low 48 bits).
    pub fn init(&mut self, mac: u64) -> Result<Vec<u8>, GuestError> {
        let mac = i64::from_ne_bytes(mac.to_ne_bytes());
        self.f.init.call(&mut self.store, mac)?;
        self.take()
    }

    /// Periodic tick at local time `now_ms`.
    pub fn tick(&mut self, now_ms: u64, buttons: u8) -> Result<Vec<u8>, GuestError> {
        let now = i64::try_from(now_ms).unwrap_or(i64::MAX);
        let st = self.f.tick.call(&mut self.store, (now, i32::from(buttons)))?;
        self.done(st, "sim_input_tick")
    }

    /// One encoded `HostMsg`.
    pub fn host(&mut self, msg: &[u8]) -> Result<Vec<u8>, GuestError> {
        let len = self.write_input(msg)?;
        let st = self.f.host.call(&mut self.store, len)?;
        self.done(st, "sim_input_host")
    }

    /// A stock-protocol packet (`via` 1 BLE, 2 link).
    pub fn radio(&mut self, via: i32, rssi: i8, payload: &[u8]) -> Result<Vec<u8>, GuestError> {
        let len = self.write_input(payload)?;
        let st = self.f.radio.call(&mut self.store, (via, i32::from(rssi), len))?;
        self.done(st, "sim_input_radio")
    }

    /// An LED sync beacon received at local time `rx_at`.
    pub fn sync(&mut self, from: u32, network_ms: u32, rx_at: u64) -> Result<Vec<u8>, GuestError> {
        let from = i32::from_ne_bytes(from.to_ne_bytes());
        let net = i32::from_ne_bytes(network_ms.to_ne_bytes());
        let rx = i64::try_from(rx_at).unwrap_or(i64::MAX);
        let st = self.f.sync.call(&mut self.store, (from, net, rx))?;
        self.done(st, "sim_input_sync")
    }

    /// The runtime's answer: an encoded `WasmStatus`, or an error code.
    pub fn wasm(&mut self, seq: u32, answer: Result<&[u8], badge_proto::ErrorCode>) -> Result<Vec<u8>, GuestError> {
        let code = match answer {
            Ok(status) => self.write_input(status)?,
            Err(code) => code.0.saturating_neg().min(-1),
        };
        let seq = i32::from_ne_bytes(seq.to_ne_bytes());
        let st = self.f.wasm.call(&mut self.store, (seq, code))?;
        self.done(st, "sim_input_wasm")
    }

    /// A wasm app log line.
    pub fn wasm_log(&mut self, line: &str) -> Result<Vec<u8>, GuestError> {
        let len = self.write_input(line.as_bytes())?;
        let st = self.f.wasm_log.call(&mut self.store, len)?;
        self.done(st, "sim_input_wasm_log")
    }

    /// Records dropped inside the module so far.
    pub fn dropped(&mut self) -> Result<u32, GuestError> {
        Ok(u32::try_from(self.f.dropped.call(&mut self.store, ())?).unwrap_or(0))
    }

    /// A copy of the framebuffer (`badge_sim::fb` layout).
    pub fn framebuffer(&self) -> Result<Vec<u8>, GuestError> {
        let mut fb = vec![0u8; FB_BYTES];
        self.memory.read(&self.store, self.regions.fb, &mut fb).map_err(|_| GuestError::Memory("framebuffer"))?;
        Ok(fb)
    }
}

#[cfg(test)]
mod tests {
    use badge_proto::{BadgeMsg, BadgeMsg_, GetInfo, HostMsg, HostMsg_::Cmd, Response_::Body};
    use badge_sim::wire::{Record, Records};

    use super::*;

    #[test]
    fn fr39_embedded_module_boots_and_answers_get_info() {
        let module = SimModule::new(None).expect("badge-sim.wasm embedded");
        let mut g = module.instantiate().unwrap();
        let boot = g.init(0x0200_C0FF_EE01).unwrap();
        assert!(Records::new(&boot).all(|r| r.is_ok()));
        let mut pb = [0u8; 64];
        let n = badge_proto::encode(&HostMsg { seq: 42, cmd: Some(Cmd::GetInfo(GetInfo {})) }, &mut pb).unwrap();
        let out = g.host(&pb[..n]).unwrap();
        let info = Records::new(&out)
            .find_map(|r| match r {
                Ok(Record::Send(b)) => match badge_proto::decode::<BadgeMsg>(b).ok()?.kind? {
                    BadgeMsg_::Kind::Response(r) if r.seq == 42 => match r.body? {
                        Body::Info(i) => Some(i),
                        _ => None,
                    },
                    _ => None,
                },
                _ => None,
            })
            .expect("info response");
        assert_eq!(info.mac.as_slice(), &[2, 0, 0xC0, 0xFF, 0xEE, 1]);
        assert!(info.firmware.starts_with("badge-sim"));
        assert!(g.framebuffer().unwrap().iter().any(|&b| b != 0));
        assert_eq!(g.dropped().unwrap(), 0);
        assert!(g.tick(20, 0).is_ok());
        assert!(matches!(g.host(&vec![0; 5000]), Err(GuestError::TooLong { .. })));
    }

    #[test]
    fn fr39_inputs_before_init_are_refused() {
        let module = SimModule::new(None).unwrap();
        let mut g = module.instantiate().unwrap();
        assert!(matches!(g.tick(0, 0), Err(GuestError::Rejected(_))));
        assert!(matches!(SimModule::new(Some(&[])), Err(GuestError::NotBuilt)));
        assert!(SimModule::new(Some(b"not wasm")).is_err());
    }
}
