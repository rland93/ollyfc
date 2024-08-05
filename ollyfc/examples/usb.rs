#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use defmt::info;
use ollyfc::usb::protocol::FcDevice;
use stm32f4xx_hal::{gpio, otg_fs, prelude::*};

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [USART1])]
mod app {

    use rtic_monotonics::{systick::Systick, Monotonic};
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
        let hse = 12.MHz();
        let sysclk = 64.MHz();
        let clocks = rcc
            .cfgr
            .use_hse(hse)
            .sysclk(sysclk)
            .require_pll48clk()
            .freeze();

        let _syscfg = dp.SYSCFG.constrain();
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, sysclk.to_Hz(), systick_mono_token);

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

        #[allow(static_mut_refs)]
        let fc_dev = FcDevice::new(usb_dev, unsafe { &mut USB_BUS }, unsafe { &mut EP_MEMORY });

        (Shared { fc_dev }, Local {})
    }

    #[task(binds=OTG_FS, shared=[fc_dev], local=[poll_count: u32 = 0])]
    fn usb_fs(mut cx: usb_fs::Context) {
        cx.shared.fc_dev.lock(|dev| {
            *cx.local.poll_count += 1;
            if *cx.local.poll_count % 1000 == 0 {
                defmt::debug!("USB interrupt fired {} times", *cx.local.poll_count);
            }

            let poll_result = dev.poll();
            let current_state = dev.state();

            defmt::debug!(
                "USB State: {:?}, Poll result: {}",
                match current_state {
                    device::UsbDeviceState::Default => "Default",
                    device::UsbDeviceState::Addressed => "Addressed",
                    device::UsbDeviceState::Configured => "Configured",
                    device::UsbDeviceState::Suspend => "Suspend",
                },
                poll_result
            );

            if poll_result {
                defmt::debug!("USB device polled successfully in interrupt");
                // Additional processing if needed
            }
        });
    }

    #[task(shared=[fc_dev])]
    async fn handle_incoming_msg(mut cx: handle_incoming_msg::Context) {
        info!("Handling incoming messages task spawned.");

        cx.shared.fc_dev.lock(|dev| {
            if dev.poll() {
                while let Some(msg) = dev.recv_msg() {
                    defmt::info!("Received message: {:?}", msg.as_bytes());
                }
            }
        });
    }
}
