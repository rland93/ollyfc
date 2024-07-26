#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(generic_arg_infer)]
#![feature(stmt_expr_attributes)]

use defmt_rtt as _;
use panic_probe as _;

use stm32f4xx_hal::prelude::*;

#[rtic::app(device = stm32f4xx_hal::pac)]
mod app {

    use rtic_monotonics::systick::Systick;

    use super::*;
    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut dp = cx.device;
        let rcc = dp.RCC.constrain();
        let hse = 25.MHz();
        let sysclk = 64.MHz();
        let clocks = rcc
            .cfgr
            .use_hse(hse)
            .sysclk(sysclk)
            .require_pll48clk()
            .freeze();

        let mut syscfg = dp.SYSCFG.constrain();
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 64_000_000, systick_mono_token);

        (Shared {}, Local {})
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            rtic::export::wfi();
        }
    }
}
