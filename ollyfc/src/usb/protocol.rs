use ollyfc_common::msg::Message;
use ollyfc_common::msg::MessageHeader;
use ollyfc_common::msg::MAX_PAYLOAD_SIZE;
use ollyfc_common::{USB_PID, USB_VID};
use stm32f4xx_hal::otg_fs;
use usb_device::{bus, bus::UsbBus, class_prelude, device, endpoint, Result};

const MAX_MSG_SIZE: usize = size_of::<MessageHeader>() + MAX_PAYLOAD_SIZE;
static mut MSG_DATA: [u8; MAX_MSG_SIZE] = [0u8; MAX_MSG_SIZE];

pub struct BulkTransferClass<'a, B: UsbBus> {
    interface: class_prelude::InterfaceNumber,
    read_ep: endpoint::EndpointOut<'a, B>,
    write_ep: endpoint::EndpointIn<'a, B>,
    read_buffer: [u8; MAX_MSG_SIZE],
    write_buffer: [u8; MAX_MSG_SIZE],
    read_pos: usize,
    write_pos: usize,
}

impl<B: UsbBus> BulkTransferClass<'_, B> {
    pub fn new(alloc: &class_prelude::UsbBusAllocator<B>) -> BulkTransferClass<'_, B> {
        BulkTransferClass {
            interface: alloc.interface(),
            read_ep: alloc.bulk(64),
            write_ep: alloc.bulk(64),
            read_buffer: [0; MAX_MSG_SIZE],
            write_buffer: [0; MAX_MSG_SIZE],
            read_pos: 0,
            write_pos: 0,
        }
    }

    pub fn write(&mut self, message: &Message) -> Result<usize> {
        let (header, body) = message.as_bytes();
        let mut written = 0;
        written += self.write_ep.write(header)?;
        written += self.write_ep.write(body)?;
        Ok(written)
    }

    pub fn read(&mut self) -> Option<Message> {
        // Try to read more data if buffer is not full
        if self.read_pos < self.read_buffer.len() {
            match self.read_ep.read(&mut self.read_buffer[self.read_pos..]) {
                Ok(count) => {
                    self.read_pos += count;
                    defmt::debug!("Read {} bytes, total: {}", count, self.read_pos);
                }
                Err(usb_device::UsbError::WouldBlock) => {
                    defmt::debug!("No data available (WouldBlock)");
                }
                Err(e) => {
                    defmt::error!("USB read error: {:?}", defmt::Debug2Format(&e));
                    return None;
                }
            }
        }
        self.process_buffer()
    }

    fn process_buffer(&mut self) -> Option<Message> {
        // Ensure we have at least a complete header
        if self.read_pos < size_of::<MessageHeader>() {
            return None;
        }

        // Parse the header
        let header = MessageHeader::from_bytes(&self.read_buffer[..size_of::<MessageHeader>()]);
        let total_message_size = size_of::<MessageHeader>() + header.length() as usize;

        // Check if the message size is valid
        if total_message_size > size_of::<MessageHeader>() + MAX_PAYLOAD_SIZE {
            defmt::error!("Message too large: {} bytes", total_message_size);
            self.read_pos = 0; // Reset buffer as we've lost synchronization
            return None;
        }

        // Check if we have the complete message
        if self.read_pos < total_message_size {
            defmt::debug!(
                "Incomplete message. Have: {}, Need: {}",
                self.read_pos,
                total_message_size
            );
            return None;
        }

        // Copy message data to a temporary buffer
        unsafe {
            MSG_DATA[..total_message_size].copy_from_slice(&self.read_buffer[..total_message_size])
        };

        // Create the message
        let message = unsafe {
            Message::new(
                header.command(),
                &MSG_DATA[size_of::<MessageHeader>()..total_message_size],
            )
        };

        // Remove the processed message from the buffer
        self.read_buffer.copy_within(total_message_size.., 0);
        self.read_pos -= total_message_size;

        defmt::info!(
            "Valid message received, remaining buffer: {}",
            self.read_pos
        );
        Some(message)
    }

    pub fn reset(&mut self) {
        self.read_pos = 0;
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
        let bulk_transfer = BulkTransferClass::new(usb_bus);
        let usb_dev = device::UsbDeviceBuilder::new(usb_bus, device::UsbVidPid(USB_VID, USB_PID))
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
        let result = self.usb_dev.poll(&mut [&mut self.bulk_transfer]);
        if result {
            defmt::debug!("USB device polled successfully");
            match self.recv_msg() {
                Some(msg) => {
                    defmt::info!("Received message: {:?}", msg.as_bytes());
                    // Process the message...
                    true
                }
                None => {
                    defmt::debug!("No message received during poll");
                    false
                }
            }
        } else {
            false
        }
    }

    pub fn state(&self) -> device::UsbDeviceState {
        self.usb_dev.state()
    }

    pub fn send_msg(&mut self, message: &Message) -> Result<usize> {
        let result = self.bulk_transfer.write(message);
        match &result {
            Ok(bytes) => defmt::info!("Sent message, {} bytes written", bytes),
            Err(e) => defmt::warn!("Failed to send message: {:?}", defmt::Debug2Format(e)),
        }
        result
    }

    pub fn recv_msg(&mut self) -> Option<Message> {
        self.bulk_transfer.read()
    }

    pub fn reset(&mut self) {
        defmt::info!("Resetting USB device");
        self.bulk_transfer.reset();
    }
}
