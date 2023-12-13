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

pub struct UsbIoHandler {}

impl UsbIoHandler {
    fn new() -> Self {
        Self {}
    }

    fn read_flash(&mut self, addr: u32, count: u32, mem: &mut W25Q) -> FlashPage {
        let mut buf = [0u8; 256];

        mem.read(addr, &mut buf);

        return buf;
    }

    fn send_log(
        &mut self,
        usb_ch_s: &mut Sender<'static, FlashPage, { crate::USB_CH_SZ }>,
        data: FlashPage,
    ) {
        usb_ch_s
            .try_send(data)
            .expect("Failed to send data to USB channel");
    }
}

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

pub async fn usb_task_fn(
    cx: &mut crate::app::usb_task::Context<'_>,
    usb_ch_s: &mut rtic_sync::channel::Sender<'static, FlashPage, { crate::USB_CH_SZ }>,
) {
    let mut buf: [u8; 128] = [0u8; 128];
    let mut tx_buf: FlashPage = [0u8; 256];
    let mut io_handler = UsbIoHandler::new();

    loop {
        let usb_dev = &mut cx.shared.usb_dev;
        let usb_ser = &mut cx.shared.usb_ser;

        let mut count: usize = 0;

        // lock & read serial
        (usb_dev, usb_ser).lock(|dev, ser| {
            if (*dev).poll(&mut [ser]) {
                // events
                if let Ok(ct) = ser.read(&mut buf) {
                    count = ct;
                }
            }
        });

        // send to command handler
        if count > 0 {
            match Command::from_byte(buf[0]) {
                Command::Invalid => {
                    error!("Invalid command received");
                }
                Command::Acknowledge => {
                    info!("Received acknowledge");
                    tx_buf[0] = Command::Acknowledge.to_byte();
                    info!("send ack to tx buf...");
                    usb_ch_s
                        .send(tx_buf)
                        .await
                        .unwrap_or_else(|e| error!("USB SendQueue Receiver does not exist."));
                    info!("sent ack to tx buf");
                    continue;
                }
                _ => {
                    error!("Command not implemented");
                }
            }
        }
    }
}

pub async fn usb_send_task_fn(
    cx: &mut crate::app::usb_send_task::Context<'_>,
    mut usb_ch_r: rtic_sync::channel::Receiver<'static, FlashPage, { crate::USB_CH_SZ }>,
) {
    loop {
        let usb_dev = &mut cx.shared.usb_dev;
        let usb_ser = &mut cx.shared.usb_ser;

        // receive from channel
        let rx_buf = usb_ch_r
            .recv()
            .await
            .expect("Failed to receive data from USB channel. No sender");
        info!("got data from channel. sending to serial device");

        (usb_dev, usb_ser).lock(|_dev, ser| {
            // write both chunks
            let mut tx_buf = [0u8; 128];
            for i in 0..2 {
                for j in 0..128 {
                    tx_buf[j] = rx_buf[i * 128 + j];
                }
                info!("sending chunk {} to serial device...", i);
                match ser.write(&tx_buf) {
                    Ok(_) => info!("sent chunk {} to serial device", i),
                    Err(e) => error!("failed to send {}", i),
                }
            }
        });
    }
}
