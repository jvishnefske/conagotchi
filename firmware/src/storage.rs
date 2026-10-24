//! Flash storage shell: contacts and the radio key in the `conag` partition,
//! plus the one-time import of the stock radio key.
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
//! 4. serves [`submit`]ted commands forever: `Save` stores one contact (a new
//!    id into a full store first deletes the contact saved longest ago,
//!    `store::SaveOrder`), `Clear` deletes every stored contact and keeps
//!    the key.
//!
//! The key is never logged: only "present", "imported" or "absent" (with
//! the reason, which never contains key bytes).
//!
//! The shell must not stall, so [`submit`] never waits: the command queue
//! holds [`QUEUE`] commands and a command that does not fit is dropped and
//! counted. Flash access is blocking (esp-storage runs the ROM routines with
//! interrupts disabled during writes and erases); the task yields to the
//! executor between commands and between the deletions of a `Clear`, but a
//! sector erase (tens of ms) still stalls everything while it runs.
//!
//! Every loop is bounded ([`MAX_ITEMS`] stored items, `MAX_CONTACTS`
//! deletions); the map's data buffer lives in the task's static future.

use core::ops::Range;
use core::sync::atomic::{AtomicU32, Ordering};

use badge_core::contacts::Contact;
use badge_core::io::{Input, StorageEvent};
use badge_core::lfs::{self, Geometry, ReadBlock};
use badge_core::oz::sign::RadioKey;
use badge_core::store::{self, Loader, SaveOrder};
use defmt::{info, warn};
use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use esp_storage::{FlashStorage, FlashStorageError};
use sequential_storage::cache::Cache;
use sequential_storage::map::{MapConfig, MapStorage};

use crate::radio::RADIO_EVENTS;

/// Flash range of the `conag` partition.
pub const CONAG: Range<u32> = 0x20_0000..0x20_2000;
/// Flash offset of the stock `vfs` partition (littlefs).
pub const VFS_BASE: u32 = 0x20_2000;
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

/// Why [`start`] failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StorageError {
    /// The storage task is already running.
    Spawn,
}

static CMDS: Channel<CriticalSectionRawMutex, StorageCmd, QUEUE> = Channel::new();
static DROPPED: AtomicU32 = AtomicU32::new(0);

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

type Map<'a> = MapStorage<u32, BlockingAsync<&'a mut FlashStorage<'static>>, Cache<
    sequential_storage::cache::Uncached,
    sequential_storage::cache::Uncached,
    sequential_storage::cache::Uncached,
    u32,
>>;
type MapError = sequential_storage::Error<FlashStorageError>;

/// Opens the map over `conag`.
fn open<'a>(flash: &'a mut FlashStorage<'static>) -> Option<Map<'a>> {
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
    flash: &'a mut FlashStorage<'static>,
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
        self.flash.read(VFS_BASE + block * geo.block_size + off, buf)
    }
}

/// Boot load, key import, then the command loop.
#[embassy_executor::task]
async fn storage_task(flash: FlashStorage<'static>) {
    let mut flash = flash;
    let mut buf = DataBuf([0; 64]);
    let buf = &mut buf.0;

    let loader = match open(&mut flash) {
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
        None => import_key(&mut flash, buf).await,
    };

    let order = SaveOrder::from_contacts(&list);
    info!("storage: {} contacts loaded", list.len());
    // One contact per input keeps `Input` small; at most MAX_CONTACTS sends.
    for contact in list {
        RADIO_EVENTS.send(Input::Storage(StorageEvent::Contact(contact))).await;
    }
    RADIO_EVENTS.send(Input::Storage(StorageEvent::Loaded { key })).await;

    let Some(mut map) = open(&mut flash) else {
        warn!("storage: map unavailable; contacts will not be saved");
        return;
    };
    let mut order = order;
    loop {
        order = match CMDS.receive().await {
            StorageCmd::Save(contact) => save(&mut map, buf, order, &contact).await,
            StorageCmd::Clear => clear(&mut map, buf, order).await,
        };
        embassy_futures::yield_now().await;
    }
}

/// Imports the stock key from `vfs` into `conag`; logs only the outcome.
async fn import_key(flash: &mut FlashStorage<'static>, buf: &mut [u8]) -> Option<RadioKey> {
    let key = match lfs::import_radio_key(&mut Vfs { flash: &mut *flash }, Geometry::STOCK_VFS) {
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
