use core::mem::size_of;

use static_assertions::const_assert;

pub trait Crc32: Sized {
    fn init(&mut self);
    fn update(&mut self, data: &[u8]) -> u32;
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct Message {
    command: crate::cmd::Command,
    length: u8,
    crc: u32,
    data: [u8; 58],
}

const_assert!(size_of::<Message>() <= 64);

impl Message {
    pub fn new<C: Crc32>(command: crate::cmd::Command, data: &[u8], hasher: &mut C) -> Self {
        let mut msg = Message {
            command,
            length: data.len() as u8,
            data: [0; 58],
            crc: 0,
        };
        msg.data[..data.len()].copy_from_slice(data);
        msg.update_crc(hasher);
        msg
    }

    pub fn update_crc<C: Crc32>(&mut self, hasher: &mut C) {
        hasher.init();
        let bytes = unsafe {
            core::slice::from_raw_parts(self as *const _ as *const u8, size_of::<Self>() - 4)
        };
        self.crc = hasher.update(bytes);
    }

    pub fn verify_crc<C: Crc32>(&self, hasher: &mut C) -> bool {
        hasher.init();
        let bytes = unsafe {
            core::slice::from_raw_parts(self as *const _ as *const u8, size_of::<Self>() - 4)
        };
        let calculated_crc = hasher.update(bytes);
        calculated_crc == self.crc
    }

    pub fn as_bytes(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(self as *const _ as *const u8, size_of::<Self>()) }
    }

    pub fn from_bytes(bytes: &[u8]) -> Self {
        assert!(bytes.len() >= size_of::<Self>());
        let mut msg = Self {
            command: crate::cmd::Command::default(),
            length: 0,
            data: [0; 58],
            crc: 0,
        };
        unsafe {
            core::ptr::copy_nonoverlapping(
                bytes.as_ptr(),
                &mut msg as *mut _ as *mut u8,
                size_of::<Self>(),
            );
        }
        msg
    }
}
