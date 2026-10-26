//! Flash storage shell: contacts and the radio key in the `conag` partition,
//! the one-time import of the stock radio key, and the installed wasm
//! payload in the `payload` partition.
//!
//! # Flash ownership
//!
//! esp-storage's `FlashStorage` takes the `FLASH` peripheral by value, so
//! there is exactly one: this task owns it. It lives in a `RefCell` local to
//! the task, and [`FlashRef`] (a copyable borrow of that cell implementing
//! the `embedded-storage` NOR traits) is handed to each user: the
//! `sequential-storage` contacts map, the read-only `vfs` reader, and
//! esp-base's `PayloadStore`. Every flash call borrows the cell only for its
//! own synchronous duration, so the borrows never overlap (a failed borrow
//! would surface as `FlashStorageError::IoError`, never as a panic). The
//! wasm task reaches the payload partition through [`payload`], a request /
//! reply pair over this task's one-slot payload queue.
//!
//! Layout (`partitions.csv`): `conag` is the stock `otadata` slot,
//! 0x200000..0x202000 (two 4 KiB sectors), holding a `sequential-storage`
//! map with `u32` keys. The keys and value encodings are pure and live in
//! `badge_core::store`; this module only moves bytes. The stock `vfs`
//! littlefs partition (0x202000, 0x55E blocks of 4 KiB) is only ever read.
//!
//! [`start`] spawns the storage task, which:
//!
//! 1. reads every stored item and folds them with `store::Loader`;
//! 2. if no radio key is stored, looks up `radio_key.txt` in the stock `vfs`
//!    filesystem (`badge_core::lfs::import_radio_key`, read-only) and, if it
//!    parses, stores it in `conag`;
//! 3. sends `Input::Storage(StorageEvent::Loaded { .. })` to the core loop
//!    through `RADIO_EVENTS` (which the shell drains every tick), even if
//!    the flash could not be read (then with no contacts and no key);
//! 4. serves commands forever: `Save` stores one contact (a new id into a
//!    full store first deletes the contact saved longest ago,
//!    `store::SaveOrder`), `Clear` deletes every stored contact and keeps
//!    the key, and a [`PayloadOp`] from the wasm task (its own one-slot
//!    queue, so the eight contact command slots stay small) is run and
//!    answered on a one-slot reply queue.
//!
//! # Payload partition
//!
//! `payload` is 0x1E0000..0x200000 (128 KiB), laid out as in middleware's
//! esp-base firmware (`esp_base::image`): a header sector (magic, length,
//! CRC-32), then the image. `esp_base::store::PayloadStore` does the work:
//! `Erase` erases the header and the image sectors, `Write` writes in-order
//! chunks as aligned words, `Activate` flushes the last word, writes the
//! header and is then verified by re-reading the image against its CRC.
//! An interrupted upload leaves an erased header, so nothing is installed.
//! `Installed` verifies the image without loading it (boot autostart);
//! `Load` reads it into a heap buffer of exactly the image length (at most
//! `board::PAYLOAD_MAX_IMAGE`; the allocation is fallible) and checks the
//! CRC again. Images are capped at `PAYLOAD_MAX_IMAGE` although the
//! partition holds 124 KiB, because the whole image is loaded into RAM.
//!
//! The key is never logged: only "present", "imported" or "absent" (with
//! the reason, which never contains key bytes).
//!
//! The shell must not stall, so [`submit`] never waits: the command queue
//! holds [`QUEUE`] commands and a command that does not fit is dropped and
//! counted. Flash access is blocking (esp-storage runs the ROM routines with
//! interrupts disabled during writes and erases); the task yields to the
//! executor between commands and between the deletions of a `Clear`, but a
//! sector erase (tens of ms) still stalls everything while it runs; a
//! payload `Erase` of a 16 KiB image erases five sectors in one call. The
//! wasm task's [`payload`] waits for room in its queue instead (it has
//! nothing else to do until the answer arrives).
//!
//! Every loop is bounded ([`MAX_ITEMS`] stored items, `MAX_CONTACTS`
//! deletions); the map's data buffer lives in the task's static future.

use alloc::vec::Vec;
use core::cell::RefCell;
use core::fmt;
use core::ops::Range;
use core::sync::atomic::{AtomicU32, Ordering};

use badge_core::board::PAYLOAD_MAX_IMAGE;
use badge_core::contacts::Contact;
use badge_core::io::{Input, StorageEvent, WASM_CHUNK};
use badge_core::lfs::{self, Geometry, ReadBlock};
use badge_core::oz::sign::RadioKey;
use badge_core::store::{self, Loader, SaveOrder};
use defmt::{info, warn};
use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embedded_storage::nor_flash::{ErrorType, MultiwriteNorFlash, NorFlash, ReadNorFlash};
use esp_base::image::Header;
use esp_base::store::{PayloadStore, StoreError};
use esp_storage::{FlashStorage, FlashStorageError};
use sequential_storage::cache::Cache;
use sequential_storage::map::{MapConfig, MapStorage};

use crate::radio::RADIO_EVENTS;

/// Flash range of the `conag` partition.
pub const CONAG: Range<u32> = 0x20_0000..0x20_2000;
/// Flash offset of the stock `vfs` partition (littlefs).
pub const VFS_BASE: u32 = 0x20_2000;
/// Flash range of the `payload` partition (the installed wasm payload).
pub const PAYLOAD: Range<u32> = 0x1E_0000..0x20_0000;
/// Capacity of the command queue.
pub const QUEUE: usize = 8;
/// Most stored items read at boot: 8 KiB / 16 bytes (smallest item).
pub const MAX_ITEMS: usize = 512;

/// What the shell asks the storage task to do.
pub enum StorageCmd {
    /// Persist this contact (insert or replace by id).
    Save(Contact),
    /// Delete every stored contact (the radio key stays).
    Clear,
}

/// An operation on the `payload` partition (see the module docs).
#[allow(clippy::large_enum_variant, reason = "fixed-size queue slots; boxing would allocate")]
pub enum PayloadOp {
    /// Erase the header and the sectors for an image of `len` bytes.
    Erase {
        /// Image length.
        len: u32,
    },
    /// Write image bytes at `offset` (in order).
    Write {
        /// Byte offset in the image.
        offset: u32,
        /// The chunk.
        data: heapless::Vec<u8, WASM_CHUNK>,
    },
    /// Complete the image: write the header, then verify the CRC.
    Activate {
        /// Image length.
        len: u32,
        /// Image CRC-32.
        crc: u32,
    },
    /// Verify the installed image without loading it.
    Installed,
    /// Read the installed image into RAM (CRC checked).
    Load,
}

/// A successful [`PayloadOp`].
pub enum PayloadDone {
    /// `Erase` or `Write` done.
    Written,
    /// `Activate` or `Installed`: a valid image with this header.
    Installed(Header),
    /// `Load`: the image bytes.
    Image(Vec<u8>),
}

/// Why a [`PayloadOp`] failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PayloadError {
    /// The store refused the operation or the flash failed.
    Store(StoreError),
    /// The heap had no room for the image buffer.
    NoMemory,
    /// The partition range is not usable (never for the built-in table).
    Unavailable,
}

impl fmt::Display for PayloadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PayloadError::Store(e) => write!(f, "{e}"),
            PayloadError::NoMemory => f.write_str("no heap for the image"),
            PayloadError::Unavailable => f.write_str("payload partition unavailable"),
        }
    }
}

/// The answer to one [`PayloadOp`].
pub type PayloadReply = Result<PayloadDone, PayloadError>;

/// Why [`start`] failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StorageError {
    /// The storage task is already running.
    Spawn,
}

static CMDS: Channel<CriticalSectionRawMutex, StorageCmd, QUEUE> = Channel::new();
/// Payload partition operations; one is outstanding at a time (only the
/// wasm task sends them, and it waits for each answer). Separate from
/// [`CMDS`] so that queue's slots are not sized for a 512-byte chunk.
static PAYLOAD_OPS: Channel<CriticalSectionRawMutex, PayloadOp, 1> = Channel::new();
/// Answers to [`PAYLOAD_OPS`].
static PAYLOAD_REPLIES: Channel<CriticalSectionRawMutex, PayloadReply, 1> = Channel::new();
static DROPPED: AtomicU32 = AtomicU32::new(0);

/// Runs one payload partition operation and waits for its answer. Waits
/// for room in the payload queue, and until the boot load (contacts, key
/// import) has finished. Only the wasm task may call this.
pub async fn payload(op: PayloadOp) -> PayloadReply {
    PAYLOAD_OPS.send(op).await;
    PAYLOAD_REPLIES.receive().await
}

/// Queues a command without waiting; a full queue drops it (counted, logged).
pub fn submit(cmd: StorageCmd) {
    if CMDS.try_send(cmd).is_err() {
        let n = DROPPED.fetch_add(1, Ordering::Relaxed).wrapping_add(1);
        warn!("storage: queue full, command dropped ({} so far)", n);
    }
}

/// Spawns the storage task on `flash`.
///
/// The firmware runs on core 0 only, but after a debugger reset the APP core
/// can be left running its ROM idle loop, which esp-storage reports as
/// `OtherCoreRunning`. Auto-park stalls it for the duration of each write and
/// releases it afterwards.
///
/// # Errors
///
/// [`StorageError::Spawn`] if the task is already running.
pub fn start(spawner: &Spawner, flash: esp_hal::peripherals::FLASH<'static>) -> Result<(), StorageError> {
    spawner.spawn(storage_task(FlashStorage::new(flash).multicore_auto_park()).map_err(|_| StorageError::Spawn)?);
    Ok(())
}

/// Word-aligned data buffer for the map (key + largest value, rounded up).
#[repr(align(4))]
struct DataBuf([u8; 64]);

/// A shared borrow of the task's flash: every NOR call borrows the cell for
/// its own duration only (see the module docs).
#[derive(Clone, Copy)]
struct FlashRef<'a>(&'a RefCell<FlashStorage<'static>>);

impl FlashRef<'_> {
    /// Runs `f` on the flash; a busy cell (never expected) is an I/O error.
    fn with<R>(self, f: impl FnOnce(&mut FlashStorage<'static>) -> Result<R, FlashStorageError>) -> Result<R, FlashStorageError> {
        let mut flash = self.0.try_borrow_mut().map_err(|_| FlashStorageError::IoError)?;
        f(&mut flash)
    }
}

impl ErrorType for FlashRef<'_> {
    type Error = FlashStorageError;
}

impl ReadNorFlash for FlashRef<'_> {
    const READ_SIZE: usize = FlashStorage::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), FlashStorageError> {
        self.with(|f| ReadNorFlash::read(f, offset, bytes))
    }

    fn capacity(&self) -> usize {
        self.0.try_borrow().map_or(0, |f| ReadNorFlash::capacity(&*f))
    }
}

impl NorFlash for FlashRef<'_> {
    const WRITE_SIZE: usize = FlashStorage::WRITE_SIZE;
    const ERASE_SIZE: usize = FlashStorage::ERASE_SIZE;

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), FlashStorageError> {
        self.with(|f| NorFlash::write(f, offset, bytes))
    }

    fn erase(&mut self, from: u32, to: u32) -> Result<(), FlashStorageError> {
        self.with(|f| NorFlash::erase(f, from, to))
    }
}

impl MultiwriteNorFlash for FlashRef<'_> {}

type Map<'a> = MapStorage<u32, BlockingAsync<FlashRef<'a>>, Cache<
    sequential_storage::cache::Uncached,
    sequential_storage::cache::Uncached,
    sequential_storage::cache::Uncached,
    u32,
>>;
type MapError = sequential_storage::Error<FlashStorageError>;

/// Opens the map over `conag`.
fn open(flash: FlashRef<'_>) -> Option<Map<'_>> {
    match MapConfig::try_new(CONAG) {
        Ok(config) => Some(MapStorage::new(BlockingAsync::new(flash), config, Cache::new_uncached())),
        Err(e) => {
            warn!("storage: bad map range: {}", e);
            None
        }
    }
}

/// Reads every stored item (oldest first) into a [`Loader`].
async fn load(map: &mut Map<'_>, buf: &mut [u8]) -> Result<Loader, MapError> {
    let mut iter = map.fetch_all_items(buf).await?;
    let mut loader = Loader::new();
    for _ in 0..MAX_ITEMS {
        match iter.next::<&[u8]>(buf).await? {
            Some((key, value)) => loader = loader.push(key, value),
            None => break,
        }
    }
    Ok(loader)
}

/// Read-only block access to the stock `vfs` partition.
struct Vfs<'a> {
    flash: FlashRef<'a>,
}

impl ReadBlock for Vfs<'_> {
    type Error = FlashStorageError;

    fn read(&mut self, block: u32, off: u32, buf: &mut [u8]) -> Result<(), FlashStorageError> {
        let geo = Geometry::STOCK_VFS;
        let len = u32::try_from(buf.len()).map_err(|_| FlashStorageError::OutOfBounds)?;
        let end = off.checked_add(len).ok_or(FlashStorageError::OutOfBounds)?;
        if block >= geo.block_count || end > geo.block_size {
            return Err(FlashStorageError::OutOfBounds);
        }
        let at = VFS_BASE + block * geo.block_size + off;
        // The inherent read accepts any length and alignment.
        self.flash.with(|f| f.read(at, buf))
    }
}

/// Boot load, key import, then the command loop.
#[embassy_executor::task]
async fn storage_task(flash: FlashStorage<'static>) {
    let cell = RefCell::new(flash);
    let flash = FlashRef(&cell);
    let mut buf = DataBuf([0; 64]);
    let buf = &mut buf.0;

    let loader = match open(flash) {
        Some(mut map) => match load(&mut map, buf).await {
            Ok(loader) => loader,
            Err(e) => {
                warn!("storage: load failed: {}", e);
                Loader::new()
            }
        },
        None => Loader::new(),
    };
    if loader.skipped() > 0 {
        warn!("storage: skipped {} unreadable items", loader.skipped());
    }
    let (list, stored_key) = loader.finish();

    let key = match stored_key {
        Some(key) => {
            info!("storage: radio key present");
            Some(key)
        }
        None => import_key(flash, buf).await,
    };

    let order = SaveOrder::from_contacts(&list);
    info!("storage: {} contacts loaded", list.len());
    // One contact per input keeps `Input` small; at most MAX_CONTACTS sends.
    for contact in list {
        RADIO_EVENTS.send(Input::Storage(StorageEvent::Contact(contact))).await;
    }
    RADIO_EVENTS.send(Input::Storage(StorageEvent::Loaded { key })).await;

    let mut map = open(flash);
    if map.is_none() {
        warn!("storage: map unavailable; contacts will not be saved");
    }
    let mut store = match PayloadStore::new(flash, PAYLOAD.start, PAYLOAD.end - PAYLOAD.start, PAYLOAD_MAX_IMAGE) {
        Ok(store) => Some(store),
        Err(e) => {
            warn!("storage: payload partition unusable: {}", defmt::Display2Format(&e));
            None
        }
    };
    let mut order = order;
    loop {
        match select(CMDS.receive(), PAYLOAD_OPS.receive()).await {
            Either::First(StorageCmd::Save(contact)) => {
                if let Some(map) = map.as_mut() {
                    order = save(map, buf, order, &contact).await;
                }
            }
            Either::First(StorageCmd::Clear) => {
                if let Some(map) = map.as_mut() {
                    order = clear(map, buf, order).await;
                }
            }
            Either::Second(op) => {
                let reply = match store.as_mut() {
                    Some(store) => payload_op(store, op),
                    None => Err(PayloadError::Unavailable),
                };
                PAYLOAD_REPLIES.send(reply).await;
            }
        }
        embassy_futures::yield_now().await;
    }
}

/// Carries out one payload partition operation.
fn payload_op<F: NorFlash>(store: &mut PayloadStore<F>, op: PayloadOp) -> PayloadReply {
    let done = match op {
        PayloadOp::Erase { len } => store.erase(len).map(|()| PayloadDone::Written),
        PayloadOp::Write { offset, data } => store.write(offset, &data).map(|()| PayloadDone::Written),
        PayloadOp::Activate { len, crc } => {
            store.activate(len, crc).and_then(|()| store.verify()).map(PayloadDone::Installed)
        }
        PayloadOp::Installed => store.verify().map(PayloadDone::Installed),
        PayloadOp::Load => return load_image(store),
    };
    done.map_err(PayloadError::Store)
}

/// Reads the installed image into an exactly sized heap buffer.
fn load_image<F: NorFlash>(store: &mut PayloadStore<F>) -> PayloadReply {
    let header = store.header().ok_or(PayloadError::Store(StoreError::NoImage))?;
    let len = usize::try_from(header.len).map_err(|_| PayloadError::Store(StoreError::TooLarge))?;
    let mut image = Vec::new();
    image.try_reserve_exact(len).map_err(|_| PayloadError::NoMemory)?;
    // Within the reserved capacity: no further allocation.
    image.resize(len, 0);
    store.load(header, &mut image).map_err(PayloadError::Store)?;
    Ok(PayloadDone::Image(image))
}

/// Imports the stock key from `vfs` into `conag`; logs only the outcome.
async fn import_key(flash: FlashRef<'_>, buf: &mut [u8]) -> Option<RadioKey> {
    let key = match lfs::import_radio_key(&mut Vfs { flash }, Geometry::STOCK_VFS) {
        Ok(key) => key,
        Err(e) => {
            info!("storage: radio key absent ({})", defmt::Debug2Format(&e));
            return None;
        }
    };
    let Some(mut map) = open(flash) else {
        info!("storage: radio key imported (not persisted)");
        return Some(key);
    };
    let value = store::encode_key(&key);
    match map.store_item(buf, &store::KEY_RADIO, &&value[..]).await {
        Ok(()) => info!("storage: radio key imported"),
        Err(e) => warn!("storage: radio key imported, persisting failed: {}", e),
    }
    Some(key)
}

/// Stores `contact`, evicting the contact saved longest ago when full.
async fn save(map: &mut Map<'_>, buf: &mut [u8], order: SaveOrder, contact: &Contact) -> SaveOrder {
    let (order, evicted) = order.save(contact.id);
    if let Some(old) = evicted
        && let Err(e) = map.remove_item(buf, &store::contact_key(old)).await
    {
        warn!("storage: evicting a contact failed: {}", e);
    }
    let value = store::encode_contact(contact);
    if let Err(e) = map.store_item(buf, &store::contact_key(contact.id), &&value[..]).await {
        warn!("storage: saving a contact failed: {}", e);
    }
    order
}

/// Deletes every stored contact, yielding between deletions.
async fn clear(map: &mut Map<'_>, buf: &mut [u8], order: SaveOrder) -> SaveOrder {
    let mut failed = 0u32;
    for &id in order.ids() {
        if map.remove_item(buf, &store::contact_key(id)).await.is_err() {
            failed = failed.wrapping_add(1);
        }
        embassy_futures::yield_now().await;
    }
    if failed > 0 {
        warn!("storage: {} contacts could not be deleted", failed);
    } else {
        info!("storage: contacts cleared");
    }
    SaveOrder::default()
}
