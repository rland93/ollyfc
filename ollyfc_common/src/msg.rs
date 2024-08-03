use core::mem::size_of;

pub const MAX_PAYLOAD_SIZE: usize = 1024;

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct MessageHeader {
    command: crate::cmd::Command,
    length: u16,
}

impl MessageHeader {
    pub fn from_bytes(bytes: &[u8]) -> Self {
        assert!(bytes.len() == size_of::<MessageHeader>());
        unsafe { *(bytes.as_ptr() as *const MessageHeader) }
    }

    pub fn as_bytes(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(self as *const _ as *const u8, size_of::<MessageHeader>())
        }
    }

    pub fn command(&self) -> crate::cmd::Command {
        self.command
    }

    pub fn length(&self) -> u16 {
        self.length
    }
}

pub struct Message<'a> {
    header: MessageHeader,
    payload: &'a [u8],
}

impl<'a> Message<'a> {
    pub fn new(command: crate::cmd::Command, data: &'a [u8]) -> Self {
        assert!(data.len() <= MAX_PAYLOAD_SIZE);
        Self {
            header: MessageHeader {
                command,
                length: data.len() as u16,
            },
            payload: data,
        }
    }

    pub fn as_bytes(&self) -> (&[u8], &[u8]) {
        (self.header.as_bytes(), self.payload)
    }

    pub fn from_bytes(header_bytes: &[u8], data: &'a [u8]) -> Self {
        assert!(header_bytes.len() == size_of::<MessageHeader>());
        let header = unsafe { *(header_bytes.as_ptr() as *const MessageHeader) };
        Self {
            header,
            payload: data,
        }
    }

    pub fn command(&self) -> crate::cmd::Command {
        self.header.command
    }

    pub fn length(&self) -> u16 {
        self.header.length
    }

    pub fn data(&self) -> &[u8] {
        self.payload
    }
}
