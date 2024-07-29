#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use stm32f4xx_hal::{otg_fs, prelude::*};

use usb_device::prelude::*;
use usbd_serial::SerialPort;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [USART1])]
mod app {

    use core::ptr::addr_of_mut;

    use rtic_monotonics::systick::Systick;

    use super::*;

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, otg_fs::UsbBusType>,
        usb_serial: SerialPort<'static, otg_fs::UsbBusType>,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<otg_fs::UsbBusType>> = None;

        let dp = cx.device;
        let rcc = dp.RCC.constrain();
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 48_000_000, systick_mono_token);
        let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

        let gpioa = dp.GPIOA.split();

        // *** Begin USB setup ***
        let usb = otg_fs::USB {
            usb_global: dp.OTG_FS_GLOBAL,
            usb_device: dp.OTG_FS_DEVICE,
            usb_pwrclk: dp.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into(),
            pin_dp: gpioa.pa12.into(),
            hclk: clocks.hclk(),
        };

        unsafe {
            USB_BUS.replace(otg_fs::UsbBus::new(
                usb,
                addr_of_mut!(EP_MEMORY).as_mut().unwrap_unchecked(),
            ));
        }

        let usb_serial = usbd_serial::SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });
        let usb_dev = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap() },
            UsbVidPid(0x1209, 0x0117),
        )
        .device_class(usbd_serial::USB_CLASS_CDC)
        .strings(&[StringDescriptors::default()
            .manufacturer("rland93")
            .product("ollyfc")
            .serial_number("0")])
        .unwrap()
        .build();

        (
            Shared {
                usb_dev,
                usb_serial,
            },
            Local {},
        )
    }

    #[task(binds=OTG_FS, shared=[usb_dev, usb_serial])]
    fn usb_fs(cx: usb_fs::Context) {
        (cx.shared.usb_dev, cx.shared.usb_serial).lock(|usb_dev, usb_serial| {
            if usb_dev.poll(&mut [usb_serial]) {
                let mut buf = [0u8; 64];

                match usb_serial.read(&mut buf) {
                    Ok(count) if count > 0 => {
                        let mut write_offset = 0;
                        while write_offset < count {
                            match usb_serial.write(&mut buf[write_offset..count]) {
                                Ok(len) if len > 0 => {
                                    write_offset += len;
                                }
                                _ => {}
                            }
                        }
                        // print char
                        defmt::info!("{}", core::str::from_utf8(&buf[..count]).unwrap());
                    }
                    _ => {}
                }
            }
        });
    }
}
