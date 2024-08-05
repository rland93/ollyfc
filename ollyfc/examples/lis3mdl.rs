#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(generic_arg_infer)]
#![feature(stmt_expr_attributes)]

use defmt_rtt as _;
use panic_probe as _;

use rtic_monotonics::systick::Systick;
use stm32f4xx_hal::prelude::*;

use lis3mdl::Lis3mdl;
use rtic_monotonics::Monotonic;
use stm32f4xx_hal::i2c;

#[rtic::app(device = stm32f4xx_hal::pac)]
mod app {

    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        dev_mag: Lis3mdl<i2c::I2c3>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
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
        let gpioc = dp.GPIOC.split();

        let scl = gpioa.pa8;
        let sda = gpioc.pc9;
        let i2c3 = i2c::I2c::new(dp.I2C3, (scl, sda), i2c::Mode::standard(400.kHz()), &clocks);

        let dev_mag = lis3mdl::Lis3mdl::new(i2c3, lis3mdl::Address::Addr1C).unwrap();

        read_sensor::spawn().unwrap();

        (Shared {}, Local { dev_mag: dev_mag })
    }

    #[task(local=[dev_mag])]
    async fn read_sensor(cx: read_sensor::Context) {
        let mag = cx.local.dev_mag;
        loop {
            let now = rtic_monotonics::systick::Systick::now();

            let raw = mag.get_raw_mag_axes().unwrap();
            defmt::info!("x: {}, y: {}, z: {}", raw.x, raw.y, raw.z);

            rtic_monotonics::systick::Systick::delay_until(now + 20u32.millis()).await;
        }
    }
}
