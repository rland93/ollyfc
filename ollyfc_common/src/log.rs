#[cfg_attr(feature = "use_defmt", derive(defmt::Format))]
#[cfg_attr(not(feature = "use_defmt"), derive(Debug, Clone, Copy, PartialEq))]
pub struct LogInfoPage {
    pub block_start_ptr: u32,
    pub block_end_ptr: u32,
    pub block_size: u32,
    pub n_blocks: u32,
}

impl LogInfoPage {
    pub fn new(block_start_ptr: u32, block_end_ptr: u32, block_size: u32, n_blocks: u32) -> Self {
        Self {
            block_start_ptr,
            block_end_ptr,
            block_size,
            n_blocks,
        }
    }

    pub fn from_bytes(bytes: &[u8]) -> Self {
        let mut block_start_ptr = [0u8; 4];
        let mut block_end_ptr = [0u8; 4];
        let mut block_size = [0u8; 4];
        let mut n_blocks = [0u8; 4];
        block_start_ptr.copy_from_slice(&bytes[0..4]);
        block_end_ptr.copy_from_slice(&bytes[4..8]);
        block_size.copy_from_slice(&bytes[8..12]);
        n_blocks.copy_from_slice(&bytes[12..16]);
        Self {
            block_start_ptr: u32::from_le_bytes(block_start_ptr),
            block_end_ptr: u32::from_le_bytes(block_end_ptr),
            block_size: u32::from_le_bytes(block_size),
            n_blocks: u32::from_le_bytes(n_blocks),
        }
    }
    pub fn to_bytes(&self) -> [u8; 16] {
        let mut bytes = [0u8; 16];
        bytes[0..4].copy_from_slice(&self.block_start_ptr.to_le_bytes());
        bytes[4..8].copy_from_slice(&self.block_end_ptr.to_le_bytes());
        bytes[8..12].copy_from_slice(&self.block_size.to_le_bytes());
        bytes[12..16].copy_from_slice(&self.n_blocks.to_le_bytes());
        bytes
    }
}
