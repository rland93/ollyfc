use defmt::{debug, error, info, warn};
use ollyfc_common::{cmd::Command, LOG_SIZE};
use rtic::Mutex;
use rtic_monotonics::{
    systick::{ExtU32, Systick},
    Monotonic,
};
use stm32f4xx_hal::otg_fs::{UsbBus, UsbBusType, USB};
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid},
};

use crate::w25q::MemError;

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
                    error!("Invalid command received {}", Command::Invalid.to_str());
                }
                Command::Acknowledge => {
                    info!("Received acknowledge");
                    ser.write(&[Command::Acknowledge.to_byte()]).unwrap();
                }
                Command::GetFlashDataInfo => {
                    info!("Received get flash data info");
                    let info = cx.shared.logger.lock(|logger| logger.read_info_page());
                    let mut info_buf: [u8; 128] = [0u8; 128];
                    info.to_bytes().iter().enumerate().for_each(|(i, b)| {
                        info_buf[i] = *b;
                    });
                    ser.write(&info_buf).unwrap();
                }
                Command::GetLogData => {
                    info!("Received get log data");
                    let mut ct: u32 = 0;
                    let mut data: [u8; LOG_SIZE] = [0u8; LOG_SIZE];
                    loop {
                        debug!("{} Readout {}", Systick::now().ticks(), ct);
                        match cx.shared.logger.lock(|logger| {
                            let addr = logger.block_start_ptr() + ct * LOG_SIZE as u32;
                            logger.mem.read(addr, &mut data)?;
                            Ok(())
                        }) {
                            Ok(_) => (),
                            Err(e) => match e {
                                MemError::SpiError(_) => error!("SPI error"),
                                MemError::NotAlignedError => error!("Page not aligned"),
                                MemError::OutOfBoundsError => error!("Data out of bounds"),
                            },
                        }

                        debug!("{} Wait for ser {}", Systick::now().ticks(), ct);
                        // delay until rts
                        while !ser.rts() {
                            Systick::delay(1u32.millis()).await;
                        }

                        debug!("{} Write/flush {}", Systick::now().ticks(), ct);
                        let nbytes = match ser.write(&data) {
                            Ok(n) => n,
                            Err(e) => {
                                error!("Write buffer is full");
                                panic!("Write buffer is full");
                            }
                        };
                        match ser.flush() {
                            Ok(_) => {}
                            Err(e) => {
                                warn!("Write buffer is full");
                            }
                        };
                        ct += 1;
                    }
                }

                _ => {
                    error!("Command 0x{:x} not implemented", buf[0]);
                }
            }
        }
    }
}
