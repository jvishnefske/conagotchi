//! The badge's middleware GATT RPC: a pure `Handler` and a `GattServer` over
//! the RPC characteristic.
//!
//! The firmware adds the middleware service [`SERVICE_UUID`]
//! (`4d579a3e-1c52-4b0f-0000-000000000000`) with the one characteristic
//! [`RPC_CHARACTERISTIC_UUID`] (`4d579a3e-1c52-4b0f-0000-000000000001`, write,
//! write without response, notify) to its GATT server, and runs
//! `Responder::serve_one` of the middleware `BleServer` built by
//! [`rpc_server`] in a loop. Request and response framing is the
//! middleware's (`frame`): fragments `[id][index][total u16 LE][data]`,
//! request body `[key length][key][payload]`, response body
//! `[status][payload]`.
//!
//! # Link
//!
//! [`RpcIo`] is the characteristic as the firmware sees it: the next
//! [`RpcEvent`] (a value a central wrote, or a central connecting or
//! disconnecting), a notification to send, and the ATT MTU. [`RpcLink`]
//! turns it into a middleware `GattServer`: a write becomes
//! `Inbound::Value` on the RPC characteristic, the lifecycle events become
//! `Inbound::Connected` / `Inbound::Disconnected` (so the server discards a
//! partial request cut by a disconnect), `notify` on the RPC characteristic
//! notifies, and a notify on any other characteristic is
//! [`RpcLinkError::UnknownCharacteristic`]. Messages are reassembled into
//! [`RPC_N`] (128) bytes; the fragment buffer size `A` is chosen by the
//! caller of [`rpc_server`] (the firmware uses its characteristic size).
//!
//! # Handler
//!
//! [`BadgeHandler`] answers [`SERVICES`] from a [`BadgeSnapshot`] that its
//! [`SnapshotSource`] provides at the moment of each request (the firmware
//! reads the PROBE block):
//!
//! - [`INFO`] `conagotchi/info`: empty request; the response is the GATT info
//!   text `"<firmware>;id=<id6>;chi=<chi>"` (`badge_core::gatt::info_string`).
//! - [`PET`] `conagotchi/pet`: an empty request reads the selected pet mode
//!   (one byte); a one-byte request sets it and the response echoes it.
//! - [`PATTERN`] `conagotchi/pattern`: the same for the LED pattern.
//! - A longer payload, or an invalid pet mode or pattern, is
//!   `HandlerError::InvalidRequest`; any other key is
//!   `HandlerError::NotFound`.
//!
//! A set does not change badge state directly: it is validated exactly like
//! the equivalent Conagotchi GATT write (`badge_core::gatt::write_to_host_msg`)
//! and queued in an outbox of `O` entries, which the firmware drains with
//! [`BadgeHandler::take_host_msg`] into `badge_proto::HostMsg`s tagged with
//! `GATT_SEQ`, so the app stays the source of truth; later reads show the
//! new value once the app has applied it. A full outbox answers
//! `HandlerError::Internal`.

use badge_core::gatt::{self, Writable};
use badge_proto::HostMsg;
use heapless::Deque;
use middleware::ble::gatt::{AttMtu, BleServer, GattServer, Inbound};
use middleware::ble::uuid::{self, Uuid128};
use middleware::{Handler, HandlerError, Key};

/// `conagotchi/info`: badge info text.
pub const INFO: Key<'static> = Key::from_static("conagotchi/info");
/// `conagotchi/pet`: read or set the pet mode.
pub const PET: Key<'static> = Key::from_static("conagotchi/pet");
/// `conagotchi/pattern`: read or set the LED pattern.
pub const PATTERN: Key<'static> = Key::from_static("conagotchi/pattern");
/// The keys the badge's responder serves (the handler answers `NotFound`
/// for any other).
pub const SERVICES: [Key<'static>; 3] = [INFO, PET, PATTERN];

/// Reassembly size of RPC messages (request or response body).
pub const RPC_N: usize = 128;
/// Handler scratch: the longest response (the info text).
pub const RPC_SCRATCH: usize = gatt::INFO_LEN;
/// Set requests the handler queues before answering `Internal`.
pub const OUTBOX: usize = 2;

/// The middleware service, as a `u128` for trouble-host's `#[gatt_service]`.
pub const SERVICE_UUID: u128 = u128::from_be_bytes(uuid::SERVICE_UUID.0);
/// The RPC characteristic, as a `u128` for trouble-host.
pub const RPC_CHARACTERISTIC_UUID: u128 = u128::from_be_bytes(uuid::RPC_CHARACTERISTIC_UUID.0);

/// Badge state the handler answers from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BadgeSnapshot {
    /// Firmware name and version.
    pub firmware: &'static str,
    /// The eFuse base MAC (the badge id is its last three bytes).
    pub mac: [u8; 6],
    /// PROBE `CHI` word.
    pub chi: u32,
    /// PROBE `PET_SELECTED` word.
    pub pet_selected: u32,
    /// PROBE `PATTERN` word.
    pub pattern: u32,
}

/// Provides the snapshot at the moment a request is handled.
pub trait SnapshotSource {
    /// The current badge state.
    fn snapshot(&self) -> BadgeSnapshot;
}

impl SnapshotSource for BadgeSnapshot {
    fn snapshot(&self) -> BadgeSnapshot {
        *self
    }
}

/// A validated set request waiting in the outbox.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Set {
    target: Writable,
    value: u8,
}

/// The responder's pure handler (see the module docs).
#[derive(Debug, Clone)]
pub struct BadgeHandler<S, const O: usize = OUTBOX> {
    source: S,
    outbox: Deque<Set, O>,
}

impl<S: SnapshotSource, const O: usize> BadgeHandler<S, O> {
    /// A handler reading badge state from `source`.
    pub const fn new(source: S) -> Self {
        Self {
            source,
            outbox: Deque::new(),
        }
    }

    /// Takes the oldest queued set as the equivalent host message.
    pub fn take_host_msg(&mut self) -> Option<HostMsg> {
        // Values were validated when queued, so conversion succeeds.
        self.outbox
            .pop_front()
            .and_then(|s| gatt::write_to_host_msg(s.target, &[s.value]).ok())
    }

    /// Sets queued.
    pub fn pending(&self) -> usize {
        self.outbox.len()
    }

    /// Read (empty request) or set (one byte) of `target`, whose current
    /// value is `word`.
    fn read_or_set(
        &mut self,
        target: Writable,
        word: u32,
        request: &[u8],
        response: &mut [u8],
    ) -> Result<usize, HandlerError> {
        let value = match request {
            [] => gatt::probe_byte(word),
            [v] => {
                gatt::write_to_host_msg(target, &[*v]).map_err(|_| HandlerError::InvalidRequest)?;
                self.outbox
                    .push_back(Set { target, value: *v })
                    .map_err(|_| HandlerError::Internal)?;
                *v
            }
            _ => return Err(HandlerError::InvalidRequest),
        };
        write(response, &[value])
    }
}

/// Copies `bytes` into `response`.
fn write(response: &mut [u8], bytes: &[u8]) -> Result<usize, HandlerError> {
    response
        .get_mut(..bytes.len())
        .ok_or(HandlerError::BufferTooSmall)?
        .copy_from_slice(bytes);
    Ok(bytes.len())
}

impl<S: SnapshotSource, const O: usize> Handler for BadgeHandler<S, O> {
    fn handle(
        &mut self,
        key: Key<'_>,
        request: &[u8],
        response: &mut [u8],
    ) -> Result<usize, HandlerError> {
        let snap = self.source.snapshot();
        if key == INFO {
            if !request.is_empty() {
                return Err(HandlerError::InvalidRequest);
            }
            let text = gatt::info_string(snap.firmware, &snap.mac, snap.chi);
            write(response, text.as_bytes())
        } else if key == PET {
            self.read_or_set(Writable::PetMode, snap.pet_selected, request, response)
        } else if key == PATTERN {
            self.read_or_set(Writable::LedPattern, snap.pattern, request, response)
        } else {
            Err(HandlerError::NotFound)
        }
    }
}

/// One event on the RPC characteristic; `Write` borrows the caller's buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RpcEvent<'b> {
    /// A central wrote this value to the RPC characteristic.
    Write(&'b [u8]),
    /// A central connected.
    Connected,
    /// The central disconnected.
    Disconnected,
}

/// The RPC characteristic as the firmware sees it.
pub trait RpcIo {
    /// Binding-specific failure.
    type Error: core::fmt::Debug;

    /// The connection's ATT MTU.
    fn att_mtu(&self) -> AttMtu;

    /// Notifies the connected central with `data` on the RPC characteristic.
    async fn notify(&mut self, data: &[u8]) -> Result<(), Self::Error>;

    /// Waits for the next event, copying a written value into `buf`. Must be
    /// cancel-safe: dropping the future before it is ready loses no event.
    async fn next_event<'b>(&mut self, buf: &'b mut [u8]) -> Result<RpcEvent<'b>, Self::Error>;
}

/// Failure of [`RpcLink`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RpcLinkError<E> {
    /// The I/O failed.
    Io(E),
    /// A notification for a characteristic other than the RPC one.
    UnknownCharacteristic,
}

/// `GattServer` over an [`RpcIo`] (see the module docs).
#[derive(Debug)]
pub struct RpcLink<I> {
    io: I,
}

impl<I> RpcLink<I> {
    /// Wraps `io`.
    pub const fn new(io: I) -> Self {
        Self { io }
    }

    /// Borrows the I/O.
    pub fn io_mut(&mut self) -> &mut I {
        &mut self.io
    }
}

impl<I: RpcIo> GattServer for RpcLink<I> {
    type Error = RpcLinkError<I::Error>;

    fn att_mtu(&self) -> AttMtu {
        self.io.att_mtu()
    }

    async fn notify(&mut self, uuid: Uuid128, data: &[u8]) -> Result<(), Self::Error> {
        if uuid != uuid::RPC_CHARACTERISTIC_UUID {
            return Err(RpcLinkError::UnknownCharacteristic);
        }
        self.io.notify(data).await.map_err(RpcLinkError::Io)
    }

    async fn receive<'b>(&mut self, buf: &'b mut [u8]) -> Result<Inbound<'b>, Self::Error> {
        Ok(
            match self.io.next_event(buf).await.map_err(RpcLinkError::Io)? {
                RpcEvent::Write(data) => Inbound::Value {
                    uuid: uuid::RPC_CHARACTERISTIC_UUID,
                    data,
                },
                RpcEvent::Connected => Inbound::Connected,
                RpcEvent::Disconnected => Inbound::Disconnected,
            },
        )
    }
}

/// The badge's RPC responder over `io` with `A`-byte fragment buffers.
pub type RpcServer<I, const A: usize> = BleServer<RpcLink<I>, RPC_N, A>;

/// The badge's RPC responder over `io`; the [`BadgeHandler`] passed to
/// `serve_one` decides which keys exist ([`SERVICES`]).
pub const fn rpc_server<I: RpcIo, const A: usize>(io: I) -> RpcServer<I, A> {
    BleServer::new(RpcLink::new(io))
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use crate::ble::testutil::{block_on, join};
    use badge_proto::HostMsg_::Cmd;
    use badge_proto::{LedPattern, PetMode, SetPattern, SetPetMode};
    use core::cell::RefCell;
    use core::future::poll_fn;
    use core::task::Poll;
    use middleware::ble::frame::{Fragmenter, Progress, Reassembler, parse_response};
    use middleware::ble::gatt::{BleClient, BleError, GattClient};
    use middleware::{Requester, Responder};
    use std::collections::VecDeque;
    use std::rc::Rc;
    use std::vec::Vec;

    /// Fragment buffer size of the badge's server in these tests.
    const A: usize = 244;

    const SNAP: BadgeSnapshot = BadgeSnapshot {
        firmware: "fw 1",
        mac: [0xe8, 0x3d, 0xc1, 0xf6, 0x0a, 0xc8],
        chi: 2,
        pet_selected: 3,
        pattern: 2,
    };

    /// What the central's link delivers to the badge.
    #[derive(Debug, Clone, PartialEq, Eq)]
    enum ToBadge {
        Write(Vec<u8>),
        Connected,
        Disconnected,
    }

    /// Values in flight between the central and the badge.
    #[derive(Default)]
    struct Air {
        to_badge: VecDeque<ToBadge>,
        to_central: VecDeque<Vec<u8>>,
    }

    type Shared = Rc<RefCell<Air>>;

    /// Copies `v` into `buf`.
    fn copy<'b>(buf: &'b mut [u8], v: &[u8]) -> Result<&'b [u8], ()> {
        let out = buf.get_mut(..v.len()).ok_or(())?;
        out.copy_from_slice(v);
        Ok(out)
    }

    /// The badge side: the RPC characteristic.
    struct BadgeIo {
        air: Shared,
        mtu: AttMtu,
    }

    impl RpcIo for BadgeIo {
        type Error = ();
        fn att_mtu(&self) -> AttMtu {
            self.mtu
        }
        async fn notify(&mut self, data: &[u8]) -> Result<(), ()> {
            assert!(data.len() <= self.mtu.payload());
            self.air.borrow_mut().to_central.push_back(data.to_vec());
            Ok(())
        }
        async fn next_event<'b>(&mut self, buf: &'b mut [u8]) -> Result<RpcEvent<'b>, ()> {
            let ev = poll_fn(|_| {
                self.air
                    .borrow_mut()
                    .to_badge
                    .pop_front()
                    .map_or(Poll::Pending, Poll::Ready)
            })
            .await;
            Ok(match ev {
                ToBadge::Write(v) => RpcEvent::Write(copy(buf, &v)?),
                ToBadge::Connected => RpcEvent::Connected,
                ToBadge::Disconnected => RpcEvent::Disconnected,
            })
        }
    }

    /// The central side: a middleware `GattClient`.
    struct CentralLink {
        air: Shared,
        mtu: AttMtu,
    }

    impl GattClient for CentralLink {
        type Error = ();
        fn att_mtu(&self) -> AttMtu {
            self.mtu
        }
        async fn write(&mut self, uuid: Uuid128, data: &[u8]) -> Result<(), ()> {
            assert_eq!(uuid, uuid::RPC_CHARACTERISTIC_UUID);
            assert!(data.len() <= self.mtu.payload());
            self.air
                .borrow_mut()
                .to_badge
                .push_back(ToBadge::Write(data.to_vec()));
            Ok(())
        }
        async fn receive<'b>(&mut self, buf: &'b mut [u8]) -> Result<Inbound<'b>, ()> {
            let v = poll_fn(|_| {
                self.air
                    .borrow_mut()
                    .to_central
                    .pop_front()
                    .map_or(Poll::Pending, Poll::Ready)
            })
            .await;
            Ok(Inbound::Value {
                uuid: uuid::RPC_CHARACTERISTIC_UUID,
                data: copy(buf, &v)?,
            })
        }
        fn try_receive<'b>(&mut self, buf: &'b mut [u8]) -> Result<Option<Inbound<'b>>, ()> {
            let Some(v) = self.air.borrow_mut().to_central.pop_front() else {
                return Ok(None);
            };
            Ok(Some(Inbound::Value {
                uuid: uuid::RPC_CHARACTERISTIC_UUID,
                data: copy(buf, &v)?,
            }))
        }
    }

    type Central = BleClient<'static, CentralLink, RPC_N, A>;
    type Badge = RpcServer<BadgeIo, A>;

    fn pair(mtu: u16) -> (Central, Badge, Shared) {
        let air = Shared::default();
        let mtu = AttMtu::new(mtu).unwrap();
        let central = BleClient::new(CentralLink {
            air: air.clone(),
            mtu,
        });
        let badge = rpc_server(BadgeIo {
            air: air.clone(),
            mtu,
        });
        (central, badge, air)
    }

    /// One request from the middleware's own requester to the badge.
    fn call(
        handler: &mut BadgeHandler<BadgeSnapshot>,
        mtu: u16,
        key: Key<'_>,
        request: &[u8],
    ) -> Result<Vec<u8>, BleError<()>> {
        let (mut central, mut badge, _) = pair(mtu);
        let mut resp = [0u8; RPC_N];
        let mut scratch = [0u8; RPC_SCRATCH];
        let (got, served) = join(
            central.request(key, request, &mut resp),
            badge.serve_one(handler, &mut scratch),
        );
        served.unwrap();
        got.map(<[u8]>::to_vec)
    }

    #[test]
    fn fr38_rpc_uuids_are_the_middleware_ones() {
        assert_eq!(SERVICE_UUID, 0x4d57_9a3e_1c52_4b0f_0000_0000_0000_0000);
        assert_eq!(
            RPC_CHARACTERISTIC_UUID,
            0x4d57_9a3e_1c52_4b0f_0000_0000_0000_0001
        );
    }

    #[test]
    fn fr38_rpc_info_end_to_end_with_middleware_requester() {
        let mut h = BadgeHandler::new(SNAP);
        // Default MTU: the 31-byte answer is fragmented.
        let got = call(&mut h, 23, INFO, b"").unwrap();
        assert_eq!(got, b"fw 1;id=F60AC8;chi=HCK");
        let got = call(&mut h, 247, INFO, b"").unwrap();
        assert_eq!(got, b"fw 1;id=F60AC8;chi=HCK");
        assert_eq!(
            call(&mut h, 23, INFO, b"x"),
            Err(BleError::Handler(HandlerError::InvalidRequest))
        );
        assert_eq!(h.pending(), 0);
    }

    #[test]
    fn fr38_rpc_pet_read_and_set_reports_host_msg() {
        let mut h = BadgeHandler::new(SNAP);
        assert_eq!(call(&mut h, 23, PET, b"").unwrap(), [3]);
        assert_eq!(h.take_host_msg(), None);
        assert_eq!(call(&mut h, 23, PET, &[5]).unwrap(), [5]);
        let msg = h.take_host_msg().unwrap();
        assert_eq!(msg.seq, badge_core::probe_layout::GATT_SEQ);
        assert_eq!(
            msg.cmd,
            Some(Cmd::SetPetMode(SetPetMode {
                mode: PetMode::Sleep
            }))
        );
        // The app is the source of truth: reads still show the snapshot.
        assert_eq!(call(&mut h, 23, PET, b"").unwrap(), [3]);
        assert_eq!(
            call(&mut h, 23, PET, &[6]),
            Err(BleError::Handler(HandlerError::InvalidRequest))
        );
        assert_eq!(
            call(&mut h, 23, PET, &[1, 2]),
            Err(BleError::Handler(HandlerError::InvalidRequest))
        );
        assert_eq!(h.take_host_msg(), None);
    }

    #[test]
    fn fr38_rpc_pattern_read_and_set_reports_host_msg() {
        let mut h = BadgeHandler::new(SNAP);
        assert_eq!(call(&mut h, 23, PATTERN, b"").unwrap(), [2]);
        assert_eq!(call(&mut h, 23, PATTERN, &[2]).unwrap(), [2]);
        assert_eq!(
            h.take_host_msg().unwrap().cmd,
            Some(Cmd::SetPattern(SetPattern {
                pattern: LedPattern::Rainbow,
                brightness: 0
            }))
        );
        assert_eq!(
            call(&mut h, 23, PATTERN, &[4]),
            Err(BleError::Handler(HandlerError::InvalidRequest))
        );
    }

    #[test]
    fn fr38_rpc_unknown_key_is_not_found_and_full_outbox_is_internal() {
        let mut h = BadgeHandler::new(SNAP);
        assert_eq!(
            call(&mut h, 23, Key::from_static("conagotchi/other"), b""),
            Err(BleError::Handler(HandlerError::NotFound))
        );
        // The pure handler says the same without the transport's key list.
        let mut buf = [0u8; 8];
        assert_eq!(
            h.handle(Key::from_static("x"), b"", &mut buf),
            Err(HandlerError::NotFound)
        );
        assert!(call(&mut h, 23, PET, &[1]).is_ok());
        assert!(call(&mut h, 23, PET, &[2]).is_ok());
        assert_eq!(
            call(&mut h, 23, PET, &[3]),
            Err(BleError::Handler(HandlerError::Internal))
        );
        assert_eq!(h.pending(), OUTBOX);
        assert!(h.take_host_msg().is_some());
        assert!(h.take_host_msg().is_some());
        assert_eq!(h.take_host_msg(), None);
        let mut tiny = [0u8; 4];
        assert_eq!(
            h.handle(INFO, b"", &mut tiny),
            Err(HandlerError::BufferTooSmall)
        );
    }

    #[test]
    fn fr38_rpc_link_maps_writes_and_lifecycle_to_inbound() {
        let air = Shared::default();
        let mut link = RpcLink::new(BadgeIo {
            air: air.clone(),
            mtu: AttMtu::DEFAULT,
        });
        assert_eq!(link.att_mtu(), AttMtu::DEFAULT);
        assert_eq!(
            block_on(link.notify(uuid::SERVICE_UUID, b"x")),
            Err(RpcLinkError::UnknownCharacteristic)
        );
        block_on(link.notify(uuid::RPC_CHARACTERISTIC_UUID, b"ok")).unwrap();
        assert_eq!(air.borrow_mut().to_central.pop_front().unwrap(), b"ok");
        air.borrow_mut().to_badge.extend([
            ToBadge::Connected,
            ToBadge::Write(std::vec![1, 2]),
            ToBadge::Disconnected,
        ]);
        let mut buf = [0u8; 8];
        assert_eq!(block_on(link.receive(&mut buf)), Ok(Inbound::Connected));
        assert_eq!(
            block_on(link.receive(&mut buf)),
            Ok(Inbound::Value {
                uuid: uuid::RPC_CHARACTERISTIC_UUID,
                data: &[1, 2]
            })
        );
        assert_eq!(block_on(link.receive(&mut buf)), Ok(Inbound::Disconnected));
        assert!(link.io_mut().air.borrow().to_badge.is_empty());
    }

    /// A request cut by a disconnect is discarded; the next central's
    /// request is answered.
    #[test]
    fn fr38_rpc_disconnect_discards_partial_request() {
        let (mut central, mut badge, air) = pair(23);
        let mut h: BadgeHandler<_> = BadgeHandler::new(SNAP);
        let mut scratch = [0u8; RPC_SCRATCH];
        // First fragment of a two-fragment request (id 9), then the link drops.
        let f = Fragmenter::request(9, PET, &[0; 5], 20).unwrap();
        assert_eq!(f.count(), 2);
        let mut out = [0u8; 20];
        let first = f.fragment(0, &mut out).unwrap().to_vec();
        air.borrow_mut().to_badge.extend([
            ToBadge::Connected,
            ToBadge::Write(first),
            ToBadge::Disconnected,
            ToBadge::Connected,
        ]);
        let mut resp = [0u8; RPC_N];
        let (got, served) = join(
            central.request(INFO, b"", &mut resp),
            badge.serve_one(&mut h, &mut scratch),
        );
        served.unwrap();
        assert_eq!(got.unwrap(), b"fw 1;id=F60AC8;chi=HCK");
        assert!(!badge.request_in_progress());
        assert_eq!(h.pending(), 0);
    }

    /// The exact bytes a PC central writes for a `conagotchi/info` request
    /// (documented for bluetoothctl), and the badge's answer to them.
    #[test]
    fn fr38_rpc_info_request_bytes_for_bluetoothctl() {
        let f = Fragmenter::request(0, INFO, b"", 20).unwrap();
        let mut out = [0u8; 20];
        assert_eq!(f.count(), 1);
        let frag = f.fragment(0, &mut out).unwrap().to_vec();
        let mut expect = std::vec![0x00, 0x00, 0x10, 0x00, 0x0f];
        expect.extend_from_slice(b"conagotchi/info");
        assert_eq!(frag, expect);

        let air = Shared::default();
        air.borrow_mut().to_badge.push_back(ToBadge::Write(frag));
        let mut badge: Badge = rpc_server(BadgeIo {
            air: air.clone(),
            mtu: AttMtu::new(185).unwrap(),
        });
        let mut h: BadgeHandler<_> = BadgeHandler::new(SNAP);
        let mut scratch = [0u8; RPC_SCRATCH];
        block_on(badge.serve_one(&mut h, &mut scratch)).unwrap();
        let reply = air.borrow_mut().to_central.pop_front().unwrap();
        let Progress::Complete(m) = Reassembler::<RPC_N>::new().accept(&reply).unwrap() else {
            panic!("one fragment at MTU 185")
        };
        assert_eq!(m.id(), 0);
        assert_eq!(
            parse_response(m.as_bytes()).unwrap(),
            Ok(&b"fw 1;id=F60AC8;chi=HCK"[..])
        );
        // [id 0][index 0][len 23 LE][status 0][text]
        assert_eq!(&reply[..5], &[0, 0, 23, 0, 0]);
    }
}
