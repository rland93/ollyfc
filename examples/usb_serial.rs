#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers=[TIM4, TIM5])]
mod app {
    use defmt::info;
    use stm32f4xx_hal::{
        otg_fs::{UsbBus, UsbBusType, USB},
        prelude::*,
    };
    use usb_device::{class_prelude::UsbBusAllocator, prelude::*};
    use usbd_serial::SerialPort;

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, UsbBus<USB>>,
        usb_ser: SerialPort<'static, UsbBus<USB>>,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;

        let dp = ctx.device;

        let rcc = dp.RCC.constrain();
        // Setup system clocks
        let hse = 25.MHz();
        let sysclk = 84.MHz();
        let clocks = rcc
            .cfgr
            .use_hse(hse)
            .sysclk(sysclk)
            .require_pll48clk()
            .freeze();

        let gpioa = dp.GPIOA.split();
        let gpioc = dp.GPIOC.split();

        // *** Begin USB setup ***

        let usb = USB {
            usb_global: dp.OTG_FS_GLOBAL,
            usb_device: dp.OTG_FS_DEVICE,
            usb_pwrclk: dp.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into(),
            pin_dp: gpioa.pa12.into(),
            hclk: clocks.hclk(),
        };
        unsafe {
            USB_BUS.replace(UsbBus::new(usb, &mut EP_MEMORY));
        }

        let usb_ser = usbd_serial::SerialPort::new(unsafe { &USB_BUS.as_ref().unwrap() });

        let usb_dev = UsbDeviceBuilder::new(
            unsafe { &USB_BUS.as_ref().unwrap() },
            UsbVidPid(0x16c0, 0x27dd),
        )
        .device_class(usbd_serial::USB_CLASS_CDC)
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake Company")
            .product("Product")
            .serial_number("TEST")])
        .unwrap()
        .build();

        (Shared { usb_dev, usb_ser }, Local {})
    }

    #[task(binds=OTG_FS, shared=[usb_dev, usb_ser])]
    fn usb_fs(cx: usb_fs::Context) {
        let usb_fs::SharedResources {
            mut usb_dev,
            mut usb_ser,
            ..
        } = cx.shared;

        (&mut usb_dev, &mut usb_ser).lock(|usb_dev, usb_serial| {
            if usb_dev.poll(&mut [usb_serial]) {
                let mut buf = [0u8; 64];

                match usb_serial.read(&mut buf) {
                    Ok(count) if count > 0 => {
                        info!("{}", buf);
                        let mut write_offset = 0;
                        while write_offset < count {
                            match usb_serial.write(&mut buf[write_offset..count]) {
                                Ok(len) if len > 0 => {
                                    write_offset += len;
                                }
                                _ => {}
                            }
                        }
                    }
                    _ => {}
                }
            }
        });
    }
}
