use crc32fast::hash;
use defmt::{info, warn};
use ollyfc_common::{cmd::Command, LOG_SIZE};
use rtic::Mutex;
use rtic_monotonics::{
    systick::{ExtU32, Systick},
    Monotonic,
};
use stm32f4xx_hal::{
    nb,
    otg_fs::{UsbBus, UsbBusType, USB},
};
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid},
    UsbError,
};

use crate::xfer_protoc::Xfer;
use crate::{w25q::MemError, xfer_protoc::HEADER_LEN};

#[allow(non_snake_case)]
pub fn usb_setup(
    usb: USB,
    USB_BUS: &'static mut Option<UsbBusAllocator<UsbBusType>>,
    EP_MEMORY: &'static mut [u32; 1024],
) -> (
    UsbDevice<'static, UsbBusType>,
    usbd_serial::SerialPort<'static, UsbBusType>,
) {
    USB_BUS.replace(UsbBus::new(usb, EP_MEMORY));
    let usb_ser = usbd_serial::SerialPort::new(&USB_BUS.as_ref().unwrap());
    let usb_dev = UsbDeviceBuilder::new(&USB_BUS.as_ref().unwrap(), UsbVidPid(0x1209, 0x6EF1))
        .device_class(usbd_serial::USB_CLASS_CDC)
        .strings(&[StringDescriptors::default()
            .manufacturer("OllyFC")
            .product("OllyFC Flight Computer")
            .serial_number("0001")])
        .unwrap()
        .build();

    (usb_dev, usb_ser)
}

pub async fn usb_task_fn(cx: &mut crate::app::usb_task::Context<'_>) {
    let mut cmd_buf = [0u8; 256];

    // primary loop for receiving and handling USB commands
    loop {
        // receive a command
        let len = match cx.local.xfer.receive(&mut cmd_buf).await {
            Ok(l) => l,
            Err(e) => {
                warn!("An error occurred in recv.");
                Systick::delay(1u32.millis()).await;
                continue;
            }
        };
        // slice out data
        let data = &cmd_buf[HEADER_LEN..HEADER_LEN + len];

        // handle invalid command
        if data.len() != 1 {
            warn!("Invalid command received");
            continue;
        }

        // pass to command handlers
        match Command::from_byte(data[0]) {
            Command::Invalid => {
                info!("Invalid command received");
            }
            Command::Acknowledge => {
                info!("Received acknowledge");
            }
            Command::GetFlashDataInfo => {
                info!("Received get flash data info");
                get_flash_info_handler(cx).await;
            }
            Command::GetLogData => {
                info!("Received get log data");
            }
            _ => {
                info!("Command 0x{:x} not implemented", data[0]);
            }
        };
    }
}

pub async fn get_flash_info_handler(cx: &mut crate::app::usb_task::Context<'_>) {
    let info = cx.shared.logger.lock(|logger| logger.read_info_page());
    cx.local.xfer.send(&info.to_bytes()).await;
}
