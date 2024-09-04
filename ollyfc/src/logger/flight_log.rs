use embedded_hal::{delay, spi};
use w25q::W25Q;

pub trait FlashInterface {
    type Error;
    fn erase_block(&mut self, address: u32) -> Result<(), Self::Error>;
    fn write_page(&mut self, address: u32, data: &[u8]) -> Result<(), Self::Error>;

    const PAGE_SIZE: usize;
    const SECTOR_SIZE: usize;
    const N_BLOCKS: usize;
    const PAGES_PER_BLOCK: usize = Self::SECTOR_SIZE / Self::PAGE_SIZE;
}

impl<S, D> FlashInterface for W25Q<S, D>
where
    S: spi::SpiDevice,
    D: delay::DelayNs,
{
    type Error = S::Error;
    const PAGE_SIZE: usize = w25q::PAGE_SIZE;
    const SECTOR_SIZE: usize = w25q::SECTOR_SIZE;
    const N_BLOCKS: usize = w25q::SECTOR_COUNT;

    fn erase_block(&mut self, address: u32) -> Result<(), Self::Error> {
        self.block_erase_32kb(address)?;
        Ok(())
    }

    fn write_page(&mut self, address: u32, data: &[u8]) -> Result<(), Self::Error> {
        self.page_program(address, data)?;
        Ok(())
    }
}
pub struct FlightLogger<DEV: FlashInterface>
where
    [(); DEV::PAGE_SIZE]: Sized,
{
    w25q_dev: DEV,
    cache: [u8; DEV::PAGE_SIZE],
    buffer_position: usize,
}

impl<DEV> FlightLogger<DEV>
where
    DEV: FlashInterface,
    [(); DEV::PAGE_SIZE]: Sized,
{
    pub fn new(w25q_dev: DEV) -> Self {
        Self {
            w25q_dev,
            cache: [0; DEV::PAGE_SIZE],
            buffer_position: 0,
        }
    }

    pub fn log_data(&mut self, log: super::Log) -> Result<(), DEV::Error> {
        let log_bytes = log.as_bytes();
        let mut remaining_bytes = log_bytes;

        while !remaining_bytes.is_empty() {
            let available_space = DEV::PAGE_SIZE - self.buffer_position;

            // Ensure we have space for at least the marker
            if available_space < 1 {
                self.write_out()?;
                self.clear_buffer();
                continue;
            }

            let max_chunk_size = available_space - 1; // Reserve 1 byte for marker
            let chunk_size = core::cmp::min(max_chunk_size, remaining_bytes.len());
            let (chunk, new_remaining) = remaining_bytes.split_at(chunk_size);

            let marker = if remaining_bytes.len() <= max_chunk_size {
                ContinuationMarker::Complete
            } else if self.buffer_position == 0 {
                ContinuationMarker::Start
            } else if new_remaining.is_empty() {
                ContinuationMarker::End
            } else {
                ContinuationMarker::Middle
            };

            // Write marker
            self.cache[self.buffer_position] = marker as u8;
            self.buffer_position += 1;

            // Write chunk
            let end = self.buffer_position + chunk.len();
            self.cache[self.buffer_position..end].copy_from_slice(chunk);
            self.buffer_position += chunk.len();

            remaining_bytes = new_remaining;

            // If buffer is full, write it out
            if self.buffer_position >= DEV::PAGE_SIZE {
                self.write_out()?;
                self.clear_buffer();
            }
        }

        Ok(())
    }

    fn write_out(&mut self) -> Result<(), DEV::Error> {
        // Implement page tracking logic here
        let page_address = 0; // This should be calculated based on the current page
        self.w25q_dev.write_page(page_address, &self.cache)
    }

    fn clear_buffer(&mut self) {
        self.cache.fill(0);
        self.buffer_position = 0;
    }
}

#[repr(u8)]
enum ContinuationMarker {
    Complete = 0x00,
    Start = 0x01,
    Middle = 0x02,
    End = 0x03,
}
