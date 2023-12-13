use defmt::{debug, error, info};
use rtic::Mutex;
use rtic_monotonics::{systick::Systick, Monotonic};

use crate::w25q::{self, FlashMem, MemError};

use ollyfc_common::log::LogInfoPage;
use ollyfc_common::{FlightLogData, LOG_SIZE};

// size of the log item, in bytes
pub const LOG_PAGE_SIZE: usize = crate::w25q::PAGE_SIZE as usize;
pub const LOGS_IN_PAGE: usize = LOG_PAGE_SIZE / LOG_SIZE;

pub struct FlightLogger<T: FlashMem> {
    pub mem: T,
    addr_ptr: u32,
    block_start_ptr: u32,
    block_end_ptr: u32,
    block_size: u32,
    n_blocks: u32,
}

impl<T: FlashMem> FlightLogger<T> {
    pub fn new(mem: T, start: u32, nblocks: u32) -> Self {
        let block_count = w25q::N_BLOCKS_32K as u32;
        let block_size = w25q::BLOCK_32K_SIZE as u32;

        if start > block_count {
            panic!("Start block address out of bounds");
        } else if start < 1 {
            panic!("Start block no. must be 1 or greater");
        }
        if start + nblocks > block_count {
            panic!("End block address out of bounds");
        }

        let block_start_ptr = start * block_size;
        let addr_ptr = block_start_ptr;
        let block_end_ptr = (start + nblocks) * block_size;

        // create the object
        let mut this = Self {
            mem: mem,
            addr_ptr: addr_ptr,
            block_start_ptr: block_start_ptr,
            block_end_ptr: block_end_ptr,
            block_size: block_size,
            n_blocks: nblocks,
        };
        info!(
            "Created logger. start=0x{:x} end=0x{:x} blocksize={} nblocks={}",
            this.block_start_ptr, this.block_end_ptr, this.block_size, this.n_blocks
        );
        this.write_info_page();

        return this;
    }

    /// info page
    pub fn write_info_page(&mut self) {
        // sector erase
        match self.mem.sector_erase(0x00) {
            Ok(_) => (),
            Err(e) => match e {
                MemError::SpiError(_) => error!("SPI error"),
                MemError::NotAlignedError => error!("Page not aligned"),
                MemError::OutOfBoundsError => error!("Data out of bounds"),
            },
        }

        let info = LogInfoPage::new(
            self.block_start_ptr,
            self.block_end_ptr,
            self.block_size,
            self.n_blocks,
        );

        let mut buf = [0u8; LOG_PAGE_SIZE];

        info.to_bytes().iter().enumerate().for_each(|(i, b)| {
            buf[i] = *b;
        });

        match self.mem.page_program(0x00, &buf) {
            Ok(_) => (),
            Err(e) => match e {
                MemError::SpiError(_) => error!("SPI error"),
                MemError::NotAlignedError => error!("Page not aligned"),
                MemError::OutOfBoundsError => error!("Data out of bounds"),
            },
        };
    }

    pub fn read_info_page(&mut self) -> LogInfoPage {
        let mut buf = [0u8; LOG_PAGE_SIZE];
        match self.mem.read(0x00, &mut buf) {
            Ok(_) => (),
            Err(e) => match e {
                MemError::SpiError(_) => error!("SPI error"),
                MemError::NotAlignedError => error!("Page not aligned"),
                MemError::OutOfBoundsError => error!("Data out of bounds"),
            },
        }

        return LogInfoPage::from_bytes(&buf);
    }

    pub fn log(&mut self, data: &[FlightLogData; LOGS_IN_PAGE]) {
        // erase the sector before writing
        if self.addr_ptr % self.mem.sector_size() as u32 == 0 {
            self.mem.sector_erase(self.addr_ptr);
        }

        // serialize onto a page
        let mut buf = [0u8; LOG_PAGE_SIZE];
        for (i, log) in data.iter().enumerate() {
            buf[i * LOG_SIZE..(i + 1) * LOG_SIZE].copy_from_slice(&log.to_bytes());
        }

        // write the page
        info!("Writing page at 0x{:x}", self.addr_ptr);
        match self.mem.page_program(self.addr_ptr, &buf) {
            Ok(_) => (),
            Err(e) => match e {
                MemError::SpiError(_) => error!("SPI error"),
                MemError::NotAlignedError => error!("Page not aligned"),
                MemError::OutOfBoundsError => error!("Data out of bounds"),
            },
        };

        // increment the address pointer
        self.incr_addr_ptr();

        // rollover
        if self.addr_ptr >= self.block_end_ptr {
            self.addr_ptr = self.block_start_ptr;
        }
    }

    fn incr_addr_ptr(&mut self) {
        self.addr_ptr += self.mem.page_size() as u32;
    }

    pub fn addr_ptr(&self) -> u32 {
        self.addr_ptr
    }
    pub fn block_start_ptr(&self) -> u32 {
        self.block_start_ptr
    }
    pub fn block_end_ptr(&self) -> u32 {
        self.block_end_ptr
    }
    pub fn n_blocks(&self) -> u32 {
        self.n_blocks
    }
    pub fn erase_block(&mut self, block: u32) {
        let address = self.block_start_ptr + block * self.block_size;
        info!(
            "Erasing block {} of {} at 0x{:x} to 0x{:x}",
            block,
            self.n_blocks,
            address,
            address + self.block_size
        );
        self.mem.block64_erase(address);
    }
}

pub async fn log_write_task_fn(
    mut cx: crate::app::log_write_task::Context<'_>,
    mut log_ch_r: rtic_sync::channel::Receiver<
        'static,
        FlightLogData,
        { crate::LOGDATA_CHAN_SIZE },
    >,
) {
    loop {
        let data = match log_ch_r.recv().await {
            Ok(data) => data,
            Err(e) => match e {
                rtic_sync::channel::ReceiveError::NoSender => {
                    debug!("No sender.");
                    continue;
                }
                rtic_sync::channel::ReceiveError::Empty => {
                    debug!("Empty queue.");
                    continue;
                }
            },
        };
        // Store the received data in the log buffer, and when the group index is high enough, reset it and write to flash.

        let now = Systick::now().ticks();

        cx.local.log_buffer[*cx.local.log_grp_idx as usize] = data;
        *cx.local.log_grp_idx += 1;
        info!(
            "{} Write: recvd log from {}:\t{} of {} for {} of {} bytes",
            now,
            data.timestamp,
            *cx.local.log_grp_idx,
            LOGS_IN_PAGE,
            *cx.local.log_grp_idx as usize * LOG_SIZE,
            LOG_PAGE_SIZE,
        );
        if *cx.local.log_grp_idx as usize >= LOGS_IN_PAGE {
            *cx.local.log_grp_idx = 0;

            cx.shared
                .logger
                .lock(|logger| logger.log(&cx.local.log_buffer));
        }
    }
}
