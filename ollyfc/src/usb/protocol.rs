use ollyfc_common::cmd::Command;
use static_assertions::const_assert;

use stm32f4xx_hal::otg_fs;

use usb_device::bus;
use usb_device::bus::UsbBus;
use usb_device::class_prelude;
use usb_device::device;
use usb_device::endpoint;
use usb_device::Result;

const VENDOR_ID: u16 = 0x16c0;
const PRODUCT_ID: u16 = 0x27dd;

use stm32f4xx_hal::crc32;

#[repr(C, packed)]
#[derive(Clone, Copy)]
struct Message {
    // 64 bytes       // n   | total
    command: Command, // 1   | 1
    length: u8,       // 1   | 2
    crc: u32,         // 4   | 6
    data: [u8; 58],   // 58  | 64
}
const_assert!(core::mem::size_of::<Message>() <= 64);

impl Message {
    fn new(command: Command, data: &[u8], hasher: &mut crc32::Crc32) -> Self {
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

    fn update_crc(&mut self, hasher: &mut crc32::Crc32) {
        hasher.init(); // Reset the CRC state
        let bytes = unsafe {
            core::slice::from_raw_parts(
                self as *const _ as *const u8,
                core::mem::size_of::<Self>() - 4,
            )
        };
        self.crc = hasher.update_bytes(bytes);
    }

    fn verify_crc(&self, hasher: &mut crc32::Crc32) -> bool {
        hasher.init(); // Reset the CRC state
        let bytes = unsafe {
            core::slice::from_raw_parts(
                self as *const _ as *const u8,
                core::mem::size_of::<Self>() - 4,
            )
        };
        let calculated_crc = hasher.update_bytes(bytes);
        calculated_crc == self.crc
    }

    fn as_bytes(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(self as *const _ as *const u8, core::mem::size_of::<Self>())
        }
    }

    fn from_bytes(bytes: &[u8]) -> Self {
        // TODO FIXME: assert here, error handling
        assert!(bytes.len() >= core::mem::size_of::<Self>());
        let mut msg = Self {
            command: Command::default(),
            length: 0,
            data: [0; 58],
            crc: 0,
        };
        unsafe {
            core::ptr::copy_nonoverlapping(
                bytes.as_ptr(),
                &mut msg as *mut _ as *mut u8,
                core::mem::size_of::<Self>(),
            );
        }
        msg
    }
}

pub struct BulkTransferClass<'a, B: UsbBus> {
    crc_hasher: crc32fast::Hasher,
    interface: class_prelude::InterfaceNumber,
    read_ep: endpoint::EndpointOut<'a, B>,
    write_ep: endpoint::EndpointIn<'a, B>,
    read_buffer: [u8; 64],
    write_buffer: [u8; 64],
    read_pos: usize,
    write_pos: usize,
}
impl<B: UsbBus> BulkTransferClass<'_, B> {
    pub fn new(alloc: &class_prelude::UsbBusAllocator<B>) -> BulkTransferClass<'_, B> {
        let hasher = crc32fast::Hasher::new();
        BulkTransferClass {
            crc_hasher: hasher,
            interface: alloc.interface(),
            read_ep: alloc.bulk(64),
            write_ep: alloc.bulk(64),
            read_buffer: [0; 64],
            write_buffer: [0; 64],
            read_pos: 0,
            write_pos: 0,
        }
    }

    pub fn write(&mut self, message: &Message) -> Result<usize> {
        let bytes = message.as_bytes();
        self.write_ep.write(bytes)
    }

    pub fn read(&mut self, hasher: &mut crc32::Crc32) -> Option<Message> {
        // haven't gotten all data
        if self.read_pos < core::mem::size_of::<Message>() {
            match self.read_ep.read(&mut self.read_buffer[self.read_pos..]) {
                Ok(count) => self.read_pos += count,
                _ => return None,
            }
        }

        // full message received
        if self.read_pos >= core::mem::size_of::<Message>() {
            // parse message
            let message = Message::from_bytes(&self.read_buffer[..core::mem::size_of::<Message>()]);

            if message.verify_crc(hasher) {
                self.read_pos = 0;
                Some(message)
            } else {
                // CRC fail
                None
            }
        } else {
            // not enough data
            None
        }
    }
}

pub struct FcDevice<'a, BUS>
where
    BUS: UsbBus,
{
    usb_dev: device::UsbDevice<'a, BUS>,
    bulk_transfer: BulkTransferClass<'a, BUS>,
}

impl FcDevice<'_, otg_fs::UsbBus<otg_fs::USB>> {
    #[allow(non_snake_case)]
    pub fn new(
        usb: otg_fs::USB,
        USB_BUS: &'static mut Option<bus::UsbBusAllocator<otg_fs::UsbBus<otg_fs::USB>>>,
        EP_MEMORY: &'static mut [u32; 1024],
    ) -> Self {
        USB_BUS.replace(otg_fs::UsbBus::new(usb, EP_MEMORY));
        let usb_bus = USB_BUS.as_ref().unwrap();
        let usb_dev =
            device::UsbDeviceBuilder::new(usb_bus, device::UsbVidPid(VENDOR_ID, PRODUCT_ID))
                .device_class(0xFF)
                .strings(&[device::StringDescriptors::default()
                    .manufacturer("rland93")
                    .product("ollyfc")
                    .serial_number("01")])
                .unwrap()
                .build();
        let bulk_transfer = BulkTransferClass::new(usb_bus);
        Self {
            usb_dev,
            bulk_transfer,
        }
    }
}
