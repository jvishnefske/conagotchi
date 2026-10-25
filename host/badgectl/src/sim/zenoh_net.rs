//! The simulator bus over Zenoh (cargo feature `zenoh`), so simulated badges
//! in different processes or on different machines meet.
//!
//! Built on the middleware's `ZenohTransport`, whose session config has
//! multicast scouting and gossip disabled: the session only listens on and
//! connects to the endpoints given (`--zenoh-listen`, `--zenoh`), so it
//! never probes the network. Each badge declares its own subscriptions
//! (Zenoh also delivers a session's puts to its own subscribers, which is
//! why every payload carries its origin, see `super::bus`).
//!
//! The middleware API is async; the world is a synchronous tick loop, so
//! publishes and declarations run on a private Tokio runtime with
//! `block_on`, and receiving polls `recv` once ([`super::bus::poll_now`]).
//! Delivery across the network is asynchronous: a sample may arrive a tick
//! or more after it was published, so runs over Zenoh are not
//! deterministic.

use middleware::zenoh::{SessionConfig, ZenohError, ZenohSubscriber, ZenohTransport};
use middleware::{Key, KeyError, Publisher, Subscriber};

use super::bus::{Network, SAMPLE_BUF, owned, poll_now};

/// Zenoh bus failures.
#[derive(Debug, thiserror::Error)]
pub enum ZenohNetError {
    /// The middleware's Zenoh backend failed.
    #[error("{0}")]
    Zenoh(#[from] ZenohError),
    /// Invalid key.
    #[error("invalid key: {0:?}")]
    Key(KeyError),
    /// The Tokio runtime could not start.
    #[error("tokio runtime: {0}")]
    Runtime(#[from] std::io::Error),
}

/// The bus over one Zenoh session.
pub struct ZenohNet {
    rt: tokio::runtime::Runtime,
    transport: ZenohTransport,
}

impl ZenohNet {
    /// Opens a session listening on `listen` and connecting to `connect`
    /// (explicit endpoints only, e.g. `tcp/127.0.0.1:7447`).
    pub fn open(listen: &[String], connect: &[String]) -> Result<Self, ZenohNetError> {
        let rt = tokio::runtime::Builder::new_multi_thread().worker_threads(2).enable_all().build()?;
        let config = listen.iter().fold(SessionConfig::new(), |c, e| c.listen(e.clone()));
        let config = connect.iter().fold(config, |c, e| c.connect(e.clone())).build()?;
        let transport = rt.block_on(ZenohTransport::open(config))?;
        Ok(Self { rt, transport })
    }
}

impl Network for ZenohNet {
    type Sub = ZenohSubscriber;
    type Error = ZenohNetError;

    fn subscribe(&mut self, key: &str) -> Result<ZenohSubscriber, ZenohNetError> {
        let key = Key::new(key).map_err(ZenohNetError::Key)?;
        Ok(self.rt.block_on(self.transport.subscriber(key))?)
    }

    fn publish(&mut self, key: &str, payload: &[u8]) -> Result<(), ZenohNetError> {
        let key = Key::new(key).map_err(ZenohNetError::Key)?;
        let Self { rt, transport } = self;
        Ok(rt.block_on(transport.publish(key, payload))?)
    }

    fn try_recv(&mut self, sub: &mut ZenohSubscriber) -> Result<Option<(String, Vec<u8>)>, ZenohNetError> {
        let mut buf = vec![0u8; SAMPLE_BUF];
        Ok(poll_now(sub.recv(&mut buf)).transpose()?.map(owned))
    }
}

#[cfg(test)]
mod tests {
    use badge_proto::{GetSync, HostMsg, HostMsg_::Cmd, Response_::Body};

    use super::*;
    use crate::session::{Conn, Observer, UpError};
    use crate::sim::guest::SimModule;
    use crate::sim::link::{SimLink, StepClock};
    use crate::sim::world::{World, WorldConfig};

    struct Quiet;

    impl Observer for Quiet {
        fn log_bytes(&mut self, _: &[u8]) {}
        fn event(&mut self, _: &badge_proto::Event) {}
        fn stray_response(&mut self, _: &badge_proto::Response) {}
        fn frame_error(&mut self, _: &UpError) {}
    }

    fn sync_peers<N: Network>(w: &mut World<N>, badge: usize) -> u32 {
        let msg = HostMsg { seq: 3, cmd: Some(Cmd::GetSync(GetSync {})) };
        let mut conn = Conn::new(SimLink::new(w, badge).unwrap(), StepClock::new(20));
        match conn.request(&msg, 2000, &mut Quiet).unwrap().body {
            Some(Body::Sync(s)) => s.peers,
            other => panic!("{other:?}"),
        }
    }

    /// One isolated session (no endpoints, no scouting): two badges in one
    /// world hear each other through Zenoh's local delivery.
    #[test]
    fn fr39_zenoh_isolated_session_carries_sync_between_badges() {
        let module = SimModule::new(None).unwrap();
        let net = ZenohNet::open(&[], &[]).unwrap();
        let mut w = World::new(&module, WorldConfig::default(), net).unwrap();
        w.run_for(4000).unwrap();
        assert_eq!(sync_peers(&mut w, 0), 1);
        assert_eq!(sync_peers(&mut w, 1), 1);
    }

    /// Two processes' worth of badges on two sessions joined over an
    /// explicit loopback endpoint (no scouting, no other traffic).
    #[test]
    #[ignore = "opens a TCP listener on 127.0.0.1:47447; run with --ignored"]
    fn fr39_zenoh_loopback_joins_two_worlds() {
        const EP: &str = "tcp/127.0.0.1:47447";
        let module = SimModule::new(None).unwrap();
        let a_net = ZenohNet::open(&[EP.to_string()], &[]).unwrap();
        let b_net = ZenohNet::open(&[], &[EP.to_string()]).unwrap();
        let cfg = |base| WorldConfig { badges: 1, mac_base: base, ..WorldConfig::default() };
        let mut a = World::new(&module, cfg(0x0200_C0FF_EE00), a_net).unwrap();
        let mut b = World::new(&module, cfg(0x0200_C0FF_EF00), b_net).unwrap();
        // Let the session connect, then step both worlds side by side, at
        // roughly wall-clock pace so remote samples arrive in time.
        std::thread::sleep(std::time::Duration::from_millis(500));
        for _ in 0..200 {
            a.step().unwrap();
            b.step().unwrap();
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        assert_eq!(sync_peers(&mut a, 0), 1);
        assert_eq!(sync_peers(&mut b, 0), 1);
    }
}
