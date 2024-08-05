#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use bmp388::{Blocking, BMP388};
use num_traits::Pow;
use ollyfc::sensor::pressure::BMP388_CFG;
use rtic_monotonics::{systick::Systick, Monotonic};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::{i2c, pac};

#[rtic::app(device = stm32f4xx_hal::pac)]
mod app {

    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        sensor: BMP388<i2c::I2c<pac::I2C3>, Blocking>,
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

        let mut delay = dp.TIM2.delay_ms(&clocks);
        let cfg = BMP388_CFG;
        defmt::info!("cfg address={:x}", cfg.address);
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
            defmt::info!(
                "Altitude: {} m {} Pa {} C",
                altitude(data.pressure),
                data.pressure,
                data.temperature
            );

            rtic_monotonics::systick::Systick::delay_until(now + 20u32.millis()).await;
        }
    }
}

pub fn altitude(pressure: f64) -> f64 {
    let altitude = 44307.69396 * (1.0 - (pressure / 101325.0).pow(0.190284));
    altitude
}
