use defmt::{debug, error};
use rtic_monotonics::Monotonic;

use crate::w25q::{FlashMem, MemError};

use ollyfc_common::{FlightLogData, LOG_SIZE};

// size of the log item, in bytes
pub const LOG_PAGE_SIZE: usize = crate::w25q::PAGE_SIZE as usize;
pub const LOGS_IN_PAGE: usize = LOG_PAGE_SIZE / LOG_SIZE;

pub struct FlightLogger<T: FlashMem> {
    pub mem: T,
    addr_ptr: u32,
}

impl<T: FlashMem> FlightLogger<T> {
    pub fn new(mem: T) -> Self {
        Self {
            mem: mem,
            addr_ptr: 0u32,
        }
    }

    pub fn init(&mut self, start: u32) {
        debug!("Initializing logger at {:?}", start);
        self.addr_ptr = start;
    }

    pub fn log(&mut self, data: &[FlightLogData; 8]) {
        let mut buf = [0u8; 256];
        for (i, log) in data.iter().enumerate() {
            buf[i * LOG_SIZE..(i + 1) * LOG_SIZE].copy_from_slice(&log.to_bytes());
        }
        match self.mem.page_program(self.addr_ptr, &buf) {
            Ok(_) => (),
            Err(e) => match e {
                MemError::SpiError(_) => error!("SPI error"),
                MemError::NotAlignedError => error!("Page not aligned"),
                MemError::OutOfBoundsError => error!("Data out of bounds"),
            },
        };
        self.incr_addr_ptr();
    }

    fn incr_addr_ptr(&mut self) {
        self.addr_ptr += self.mem.page_size() as u32;
    }
}

pub async fn log_write(
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
        let now = rtic_monotonics::systick::Systick::now().duration_since_epoch();
    }
}
