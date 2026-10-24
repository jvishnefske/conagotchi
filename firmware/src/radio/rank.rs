//! Pure ranking and advertising-data helpers for the radio tasks.
//!
//! The implementation lives in `badge_core::radio_rank` so its tests run on
//! the host with the rest of the functional core; this module re-exports it
//! under the name the radio tasks use.

pub use badge_core::radio_rank::*;
