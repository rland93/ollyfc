use defmt::{error, info};
use ollyfc_common::{cmd::Command, FlightLogData};
use rtic::{mutex_prelude::TupleExt02, Mutex};
use rtic_sync::channel::Sender;
use stm32f4xx_hal::otg_fs::{UsbBus, UsbBusType, USB};
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid},
};

use crate::{w25q::W25Q, FlashPage};

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
    let mut buf: [u8; 128] = [0u8; 128];
    let dev = &mut *cx.local.usb_dev;
    let ser = &mut *cx.local.usb_ser;

    loop {
        let mut count: usize = 0;

        if dev.poll(&mut [ser]) {
            // events
            if let Ok(ct) = ser.read(&mut buf) {
                count = ct;
            } else {
                count = 0;
            }
        }

        // send to command handler
        if count > 0 {
            match Command::from_byte(buf[0]) {
                Command::Invalid => {
                    error!("Invalid command received");
                }
                Command::Acknowledge => {
                    info!("Received acknowledge");
                    ser.write(&[Command::Acknowledge.to_byte()]).unwrap();
                }
                _ => {
                    error!("Command not implemented");
                }
            }
        }
    }
}
