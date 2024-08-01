#[cfg(feature = "std")]
use serde::{Deserialize, Serialize};

pub const LOG_INFO_SIZE: usize = 24;
pub const LOG_SIZE: usize = 64;

#[cfg_attr(feature = "std", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "no_std", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LogInfoPage {
    pub block_start_ptr: u32, // address of first block
    pub block_end_ptr: u32,   // ending address of last block
    pub block_size: u32,      // size of each block in bytes
    pub n_blocks: u32,        // total number of blocks in log region
    pub current_page: u32,    // address of "current" page -- last page written
    pub page_size: u32,       // size of each page in bytes
}

impl LogInfoPage {
    pub fn new(
        block_start_ptr: u32,
        block_end_ptr: u32,
        block_size: u32,
        n_blocks: u32,
        start_page_addr: u32,
        page_size: u32,
    ) -> Self {
        Self {
            block_start_ptr,
            block_end_ptr,
            block_size,
            n_blocks,
            current_page: start_page_addr,
            page_size,
        }
    }

    pub fn default() -> Self {
        Self {
            block_start_ptr: 0,
            block_end_ptr: 0,
            block_size: 0,
            n_blocks: 0,
            current_page: 0,
            page_size: 0,
        }
    }

    pub fn n_logs_in_region(&self) -> u32 {
        self.n_blocks * self.block_size / crate::log::LOG_SIZE as u32
    }

    pub fn from_bytes(bytes: &[u8]) -> Self {
        let mut block_start_ptr = [0u8; 4];
        let mut block_end_ptr = [0u8; 4];
        let mut block_size = [0u8; 4];
        let mut n_blocks = [0u8; 4];
        let mut current_page = [0u8; 4];
        let mut page_size = [0u8; 4];

        block_start_ptr.copy_from_slice(&bytes[0..4]);
        block_end_ptr.copy_from_slice(&bytes[4..8]);
        block_size.copy_from_slice(&bytes[8..12]);
        n_blocks.copy_from_slice(&bytes[12..16]);
        current_page.copy_from_slice(&bytes[16..20]);
        page_size.copy_from_slice(&bytes[20..24]);

        Self {
            block_start_ptr: u32::from_le_bytes(block_start_ptr),
            block_end_ptr: u32::from_le_bytes(block_end_ptr),
            block_size: u32::from_le_bytes(block_size),
            n_blocks: u32::from_le_bytes(n_blocks),
            current_page: u32::from_le_bytes(current_page),
            page_size: u32::from_le_bytes(page_size),
        }
    }
    pub fn to_bytes(&self) -> [u8; LOG_INFO_SIZE] {
        let mut bytes = [0u8; LOG_INFO_SIZE];
        bytes[0..4].copy_from_slice(&self.block_start_ptr.to_le_bytes());
        bytes[4..8].copy_from_slice(&self.block_end_ptr.to_le_bytes());
        bytes[8..12].copy_from_slice(&self.block_size.to_le_bytes());
        bytes[12..16].copy_from_slice(&self.n_blocks.to_le_bytes());
        bytes[16..20].copy_from_slice(&self.current_page.to_le_bytes());
        bytes[20..24].copy_from_slice(&self.page_size.to_le_bytes());
        bytes
    }
}
