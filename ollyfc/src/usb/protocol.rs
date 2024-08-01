use ollyfc_common::msg::Message;

use stm32f4xx_hal::otg_fs;

use stm32f4xx_hal::pac::CRC;
use usb_device::bus;
use usb_device::bus::UsbBus;
use usb_device::class_prelude;
use usb_device::device;
use usb_device::endpoint;
use usb_device::Result;

const VENDOR_ID: u16 = 0x16c0;
const PRODUCT_ID: u16 = 0x27dd;

use ollyfc_common::msg::Crc32;
use stm32f4xx_hal::crc32;

struct FirmwareCrc32 {
    periph: crc32::Crc32,
}

// wrapper type to get around orphan rule
impl FirmwareCrc32 {
    pub fn new(crc_periph: CRC) -> Self {
        Self {
            periph: crc32::Crc32::new(crc_periph),
        }
    }
}

impl Crc32 for FirmwareCrc32 {
    fn init(&mut self) {
        self.periph.init();
    }

    fn update(&mut self, data: &[u8]) -> u32 {
        self.periph.update_bytes(data)
    }
}

pub struct BulkTransferClass<'a, B: UsbBus> {
    hasher: FirmwareCrc32,
    interface: class_prelude::InterfaceNumber,
    read_ep: endpoint::EndpointOut<'a, B>,
    write_ep: endpoint::EndpointIn<'a, B>,
    read_buffer: [u8; 64],
    write_buffer: [u8; 64],
    read_pos: usize,
    write_pos: usize,
}
impl<B: UsbBus> BulkTransferClass<'_, B> {
    pub fn new(
        alloc: &class_prelude::UsbBusAllocator<B>,
        crc_periph: CRC,
    ) -> BulkTransferClass<'_, B> {
        // there can be only one.
        let hasher = FirmwareCrc32::new(crc_periph);
        BulkTransferClass {
            hasher: hasher,
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

    pub fn read(&mut self) -> Option<Message> {
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

            if message.verify_crc(&mut self.hasher) {
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

impl<B: UsbBus> class_prelude::UsbClass<B> for BulkTransferClass<'_, B> {
    fn get_configuration_descriptors(
        &self,
        writer: &mut usb_device::descriptor::DescriptorWriter,
    ) -> Result<()> {
        writer.interface(self.interface, 0xff, 0x00, 0x00)?;
        writer.endpoint(&self.read_ep)?;
        writer.endpoint(&self.write_ep)?;
        Ok(())
    }

    fn reset(&mut self) {
        self.read_pos = 0;
        self.write_pos = 0;
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
        crc_periph: CRC,
    ) -> Self {
        USB_BUS.replace(otg_fs::UsbBus::new(usb, EP_MEMORY));
        let usb_bus = USB_BUS.as_ref().unwrap();

        // Create BulkTransferClass first
        let bulk_transfer = BulkTransferClass::new(usb_bus, crc_periph);

        // Then create UsbDevice
        let usb_dev =
            device::UsbDeviceBuilder::new(usb_bus, device::UsbVidPid(VENDOR_ID, PRODUCT_ID))
                .device_class(0xFF)
                .strings(&[device::StringDescriptors::default()
                    .manufacturer("rland93")
                    .product("ollyfc")
                    .serial_number("01")])
                .unwrap()
                .build();

        Self {
            usb_dev,
            bulk_transfer,
        }
    }

    pub fn poll(&mut self) -> bool {
        self.usb_dev.poll(&mut [&mut self.bulk_transfer])
    }

    pub fn state(&self) -> device::UsbDeviceState {
        self.usb_dev.state()
    }

    pub fn send_msg(&mut self, message: &Message) -> Result<usize> {
        self.bulk_transfer.write(message)
    }

    pub fn recv_msg(&mut self) -> Option<Message> {
        self.bulk_transfer.read()
    }
}
