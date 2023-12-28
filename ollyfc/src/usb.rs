use defmt::{error, info, warn};
use ollyfc_common::cmd::Command;
use rtic::Mutex;
use rtic_monotonics::systick::{ExtU32, Systick};
use stm32f4xx_hal::otg_fs::{UsbBus, UsbBusType, USB};
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid},
    UsbError,
};

use crate::w25q::PAGE_SIZE;
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
            Err(_e) => {
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
                match get_flash_info_handler(cx).await {
                    Ok(_) => {}
                    Err(_e) => {
                        error!("an error occurred in flash info handler.");
                    }
                };
            }

            Command::GetLogData => {
                info!("Received get log data");
                match get_log_data_handler(cx).await {
                    Ok(_) => {}
                    Err(_e) => {
                        error!("an error occurred in log data handler.");
                    }
                };
            }

            _ => {
                info!("Command 0x{:x} not implemented", data[0]);
            }
        };
    }
}

pub async fn get_flash_info_handler(
    cx: &mut crate::app::usb_task::Context<'_>,
) -> Result<(), UsbError> {
    // get flash info page
    let mut info = cx.shared.logger.lock(|logger| logger.read_info_page());
    /* TODO: hardcoded for testing */
    info.page_size = PAGE_SIZE;
    info.block_end_ptr = info.block_start_ptr + PAGE_SIZE * 4;
    cx.local.xfer.send(&info.to_bytes()).await?;
    Ok(())
}

pub async fn get_log_data_handler(
    cx: &mut crate::app::usb_task::Context<'_>,
) -> Result<(), UsbError> {
    // enter a loop to acquire log data. That way we can avoid the overhead
    // of needing to call commands for each log entry read.
    let mut rx_buf = [0u8; 10];
    let mut page_buf = [0u8; PAGE_SIZE as usize];
    loop {
        // receive address
        cx.local.xfer.receive(&mut rx_buf).await?;
        let addr = u32::from_le_bytes(rx_buf[HEADER_LEN..HEADER_LEN + 4].try_into().unwrap());

        info!("Received address: 0x{:x}", addr);

        // TODO: properly account for this termination of the loop
        #[allow(non_snake_case)]
        let TERMINATION_ADDR = 0xFFFFFFFFu32;
        if addr == TERMINATION_ADDR {
            info!("Done with direct address mode.");
            break;
        }

        // get flash page at that address
        match cx
            .shared
            .logger
            .lock(|logger| logger.mem.read(addr, &mut page_buf))
        {
            Ok(_) => {}
            Err(e) => {
                match e {
                    MemError::OutOfBoundsError => {
                        warn!("Invalid address received");
                    }
                    MemError::NotAlignedError => {
                        warn!("Not aligned");
                    }
                    MemError::SpiError(_e) => {
                        panic!("SPI error");
                    }
                }
                break;
            }
        }

        // send page
        cx.local.xfer.send(&page_buf).await?;
    }
    Ok(())
}
