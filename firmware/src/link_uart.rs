//! Link UART shell: the wired badge-to-badge link on the LINK header
//! (UART1, 115200 8N1, TX GPIO39 / RX GPIO40, `badge_core::board::link`).
//!
//! [`start`] spawns two tasks:
//!
//! - the receive task feeds every byte to `badge_core::oz::link::Decoder`
//!   and sends each complete payload to the core loop as
//!   `Input::RadioPacket { via: Via::Link, rssi: 0, payload }` through
//!   `RADIO_EVENTS`; if that channel is full the packet is dropped and
//!   counted (the link must not back up into the UART FIFO);
//! - the transmit task writes each frame queued by [`send`] (the shell's
//!   `Effect::LinkSend`, already framed by the core).
//!
//! [`send`] never waits: [`TX_QUEUE`] frames are buffered and a frame that
//! does not fit is dropped and counted. UART errors (overflow, framing,
//! glitches; common with nothing plugged in) are counted and logged
//! sparingly; the decoder resynchronises by itself.

use core::sync::atomic::{AtomicU32, Ordering};

use badge_core::board::link;
use badge_core::io::{Input, Via};
use badge_core::oz::link::{Decoder, MAX_FRAME};
use defmt::warn;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use esp_hal::Async;
use esp_hal::uart::{Config, ConfigError, Uart, UartRx, UartTx};
use heapless::Vec;

use crate::radio::RADIO_EVENTS;

/// Frames buffered for transmission.
pub const TX_QUEUE: usize = 4;
/// Log one line per this many errors or drops of a kind.
const LOG_EVERY: u32 = 64;

/// Why [`start`] failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum LinkError {
    /// The UART rejected its configuration.
    Config(ConfigError),
    /// A link task is already running.
    Spawn,
}

static TX_FRAMES: Channel<CriticalSectionRawMutex, Vec<u8, MAX_FRAME>, TX_QUEUE> = Channel::new();
static TX_DROPPED: AtomicU32 = AtomicU32::new(0);
static RX_DROPPED: AtomicU32 = AtomicU32::new(0);
static RX_ERRORS: AtomicU32 = AtomicU32::new(0);

/// Increments `counter`; true when this occurrence should be logged.
fn count(counter: &AtomicU32) -> (u32, bool) {
    let n = counter.fetch_add(1, Ordering::Relaxed).wrapping_add(1);
    (n, n % LOG_EVERY == 1)
}

/// Queues a framed packet for the link without waiting; a full queue drops
/// it (counted, logged).
pub fn send(frame: Vec<u8, MAX_FRAME>) {
    if TX_FRAMES.try_send(frame).is_err() {
        let (n, log) = count(&TX_DROPPED);
        if log {
            warn!("link: tx queue full, frame dropped ({} so far)", n);
        }
    }
}

/// Configures UART1 on the LINK pins and spawns the link tasks.
///
/// # Errors
///
/// [`LinkError`] if the UART configuration is rejected or a task is
/// already running.
pub fn start(
    spawner: &Spawner,
    uart: esp_hal::peripherals::UART1<'static>,
    tx: esp_hal::peripherals::GPIO39<'static>,
    rx: esp_hal::peripherals::GPIO40<'static>,
) -> Result<(), LinkError> {
    let config = Config::default().with_baudrate(link::BAUD);
    let uart = Uart::new(uart, config).map_err(LinkError::Config)?.with_tx(tx).with_rx(rx).into_async();
    let (rx, tx) = uart.split();
    spawner.spawn(rx_task(rx).map_err(|_| LinkError::Spawn)?);
    spawner.spawn(tx_task(tx).map_err(|_| LinkError::Spawn)?);
    Ok(())
}

/// Decodes received bytes into `Input::RadioPacket`s.
#[embassy_executor::task]
async fn rx_task(mut rx: UartRx<'static, Async>) {
    let mut decoder = Decoder::default();
    let mut buf = [0u8; 64];
    loop {
        let n = match rx.read_async(&mut buf).await {
            Ok(n) => n,
            Err(e) => {
                let (n, log) = count(&RX_ERRORS);
                if log {
                    warn!("link: rx error {} ({} so far)", e, n);
                }
                continue;
            }
        };
        for &byte in buf.get(..n).unwrap_or(&[]) {
            if let Some(payload) = decoder.push(byte) {
                let input = Input::RadioPacket { via: Via::Link, rssi: 0, payload };
                if RADIO_EVENTS.try_send(input).is_err() {
                    let (n, log) = count(&RX_DROPPED);
                    if log {
                        warn!("link: event queue full, packet dropped ({} so far)", n);
                    }
                }
            }
        }
    }
}

/// Writes queued frames out.
#[embassy_executor::task]
async fn tx_task(mut tx: UartTx<'static, Async>) {
    loop {
        let frame = TX_FRAMES.receive().await;
        let mut rest: &[u8] = &frame;
        // Each write moves at least one byte, so MAX_FRAME rounds suffice.
        for _ in 0..MAX_FRAME {
            if rest.is_empty() {
                break;
            }
            match tx.write_async(rest).await {
                Ok(0) => break,
                Ok(n) => rest = rest.get(n..).unwrap_or(&[]),
                Err(e) => {
                    warn!("link: tx error {}", e);
                    break;
                }
            }
        }
    }
}
