#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use defmt::info;
use ollyfc::usb::protocol::FcDevice;
use stm32f4xx_hal::{gpio, otg_fs, prelude::*};

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [USART1])]
mod app {

    use rtic_monotonics::systick::Systick;
    use usb_device::device;

    use super::*;

    #[shared]
    struct Shared {
        fc_dev: FcDevice<'static, otg_fs::UsbBus<otg_fs::USB>>,
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

        let usb_pin_dm = gpioa.pa11.into_alternate();
        let usb_pin_dp = gpioa.pa12.into_alternate();
        let usb_dev = otg_fs::USB {
            usb_global: dp.OTG_FS_GLOBAL,
            usb_device: dp.OTG_FS_DEVICE,
            usb_pwrclk: dp.OTG_FS_PWRCLK,
            pin_dm: gpio::alt::otg_fs::Dm::PA11(usb_pin_dm),
            pin_dp: gpio::alt::otg_fs::Dp::PA12(usb_pin_dp),
            hclk: clocks.hclk(),
        };

        let fc_dev = FcDevice::new(
            usb_dev,
            unsafe { &mut USB_BUS },
            unsafe { &mut EP_MEMORY },
            dp.CRC,
        );

        (Shared { fc_dev: fc_dev }, Local {})
    }

    #[task(binds=OTG_FS, shared=[fc_dev])]
    fn usb_fs(mut cx: usb_fs::Context) {
        cx.shared.fc_dev.lock(|dev| {
            let poll = dev.poll();
            if !poll {
                return;
            }

            match dev.state() {
                device::UsbDeviceState::Default => {
                    info!("USB Default");
                    // Device has just been connected or reset
                    // Initialize device here if needed
                }
                device::UsbDeviceState::Addressed => {
                    info!("USB Addressed");
                    // Device has been assigned an address
                    // Set configuration here if needed
                }
                device::UsbDeviceState::Configured => {
                    info!("USB Configured");
                    // Device is connected and configured
                    // Handle incoming data here
                }
                device::UsbDeviceState::Suspend => {
                    info!("USB Suspend");
                    // Device has been disconnected
                    // Handle disconnection here
                }
            }
        });

        // Clear interrupt
    }

    #[task(shared=[fc_dev])]
    async fn handle_incoming_msg(mut cx: handle_incoming_msg::Context) {
        cx.shared.fc_dev.lock(|dev| {});
    }
}
