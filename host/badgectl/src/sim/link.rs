//! The RTT [`Link`] over a simulated badge, so `crate::session::Conn` (and
//! every request command) runs unchanged against the simulator.
//!
//! [`SimLink::write_down`] feeds COBS frames to the badge like the RTT down
//! channel; [`SimLink::read_up`] advances the whole world by one tick when
//! nothing is waiting, then returns the badge's framed messages. There is no
//! defmt log channel and no PROBE block, so `peek` / `poke` are refused.
//!
//! [`StepClock`] is the matching virtual clock: every idle wait advances it
//! by one tick, which is exactly one world step (each idle poll reads the
//! link once), so `--timeout` counts simulated milliseconds.

use super::bus::Network;
use super::world::{SimError, World};
use crate::session::{Clock, Link};

/// Errors of the simulated link.
#[derive(Debug, thiserror::Error)]
pub enum SimLinkError {
    /// The simulation failed.
    #[error("simulator: {0}")]
    Sim(#[from] SimError),
    /// Word access (PROBE block) does not exist in the simulator.
    #[error("the simulator has no PROBE block (peek/poke need hardware)")]
    NoProbeBlock,
}

/// A host link to badge `badge` of a world.
pub struct SimLink<'w, N: Network> {
    world: &'w mut World<N>,
    badge: usize,
}

impl<'w, N: Network> SimLink<'w, N> {
    /// Attaches to badge `badge`.
    pub fn new(world: &'w mut World<N>, badge: usize) -> Result<Self, SimError> {
        world.node(badge)?;
        Ok(Self { world, badge })
    }
}

impl<N: Network> Link for SimLink<'_, N> {
    type Error = SimLinkError;

    fn write_down(&mut self, bytes: &[u8]) -> Result<usize, SimLinkError> {
        self.world.write_down(self.badge, bytes)?;
        Ok(bytes.len())
    }

    fn read_up(&mut self, buf: &mut [u8]) -> Result<usize, SimLinkError> {
        if self.world.up_len(self.badge)? == 0 {
            self.world.step()?;
        }
        Ok(self.world.read_up(self.badge, buf)?)
    }

    fn read_log(&mut self, _buf: &mut [u8]) -> Result<usize, SimLinkError> {
        Ok(0)
    }

    fn read_words(&mut self, _addr: u64, _out: &mut [u32]) -> Result<(), SimLinkError> {
        Err(SimLinkError::NoProbeBlock)
    }

    fn write_word(&mut self, _addr: u64, _value: u32) -> Result<(), SimLinkError> {
        Err(SimLinkError::NoProbeBlock)
    }
}

/// Virtual clock: each wait is one tick of simulated time.
#[derive(Debug, Clone, Copy)]
pub struct StepClock {
    now: u64,
    tick: u64,
}

impl StepClock {
    /// A clock advancing `tick_ms` per wait.
    pub fn new(tick_ms: u64) -> Self {
        Self { now: 0, tick: tick_ms.max(1) }
    }
}

impl Clock for StepClock {
    fn now_ms(&self) -> u64 {
        self.now
    }

    fn sleep_ms(&mut self, _ms: u64) {
        self.now += self.tick;
    }
}
