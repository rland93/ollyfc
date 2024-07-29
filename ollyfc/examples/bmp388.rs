#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(generic_arg_infer)]
#![feature(stmt_expr_attributes)]

use defmt_rtt as _;
use panic_probe as _;

use bmp388::{Blocking, BMP388};
use ollyfc::sensor::pressure::BMP388_CFG;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::{i2c, pac};

#[rtic::app(device = stm32f4xx_hal::pac)]
mod app {

    use super::*;
    use rtic_monotonics::{systick::Systick, Monotonic};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        sensor: BMP388<i2c::I2c<pac::I2C3>, Blocking>,
    }

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

        let gpioa = dp.GPIOA.split();
        let gpioc = dp.GPIOC.split();

        let i2c3_sda = gpioc.pc9.into_alternate_open_drain();
        let i2c3_scl = gpioa.pa8.into_alternate_open_drain();
        let i2c3 = dp.I2C3.i2c((i2c3_scl, i2c3_sda), 400.kHz(), &clocks);
        let mut delay = dp.TIM2.delay_ms(&clocks);
        let cfg = BMP388_CFG;
        let sensor = cfg.setup_blocking(i2c3, &mut delay).unwrap();

        read_sensor::spawn().unwrap();

        (Shared {}, Local { sensor: sensor })
    }

    #[task(local=[sensor])]
    async fn read_sensor(cx: read_sensor::Context) {
        let sensor = cx.local.sensor;
        loop {
            let now = rtic_monotonics::systick::Systick::now();

            let data = sensor.sensor_values().unwrap();
            defmt::info!("Pressure: {} Pa", data.pressure);
            defmt::info!("Temperature: {} C", data.temperature);

            rtic_monotonics::systick::Systick::delay_until(now + 20u32.millis()).await;
        }
    }
}
