#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use embedded_hdc1080_rs::Hdc1080;
use rtic_monotonics::systick::Systick;
use rtic_monotonics::Monotonic;

use stm32f4xx_hal::{i2c, pac, prelude::*, timer};

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {

    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        temp_sensor: Hdc1080<i2c::I2c3, timer::Delay<pac::TIM2, 1000>>,
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

        let delay = dp.TIM2.delay_ms(&clocks);

        let scl = gpioa.pa8;
        let sda = gpioc.pc9;
        let i2c3 = i2c::I2c::new(dp.I2C3, (scl, sda), i2c::Mode::standard(400.kHz()), &clocks);
        let temp_sensor = Hdc1080::new(i2c3, delay).unwrap();

        temp_task::spawn().unwrap();

        (Shared {}, Local { temp_sensor })
    }

    #[task (local = [temp_sensor])]
    async fn temp_task(cx: temp_task::Context) {
        let temp_sensor = cx.local.temp_sensor;
        loop {
            let now = rtic_monotonics::systick::Systick::now();

            let temp = temp_sensor.temperature().unwrap();
            let hum = temp_sensor.humidity().unwrap();
            let scaled_temp = (temp * 100.0) as i32;
            let int_part = (scaled_temp / 100) as i16;
            let frac_part = (scaled_temp % 100) as u8;
            let scaled_hum = (hum * 100.0) as i32;
            let int_hum = (scaled_hum / 100) as i16;
            let frac_hum = (scaled_hum % 100) as u8;
            defmt::info!("Temperature: {}.{} C", int_part, frac_part);
            defmt::info!("Humidity: {}.{} %", int_hum, frac_hum);

            rtic_monotonics::systick::Systick::delay_until(now + 20u32.millis()).await;
        }
    }
}
