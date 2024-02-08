#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers=[TIM4])]
mod app {
    use defmt::info;
    use stm32f4xx_hal::prelude::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let dp = ctx.device;

        let rcc = dp.RCC.constrain();
        // Setup system clocks
        // let hse = 16.MHz();
        let sysclk = 32.MHz();
        let clocks = rcc.cfgr.sysclk(sysclk).freeze();

        let gpioa = dp.GPIOA.split();
        let gpioc = dp.GPIOC.split();

        info!("Setup completed.");

        (Shared {}, Local {})
    }
}
