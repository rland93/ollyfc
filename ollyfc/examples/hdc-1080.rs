#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use panic_probe as _;
use defmt::info;
use defmt_rtt as _;
use rtic_monotonics::systick::Systick;
use stm32f4xx_hal::{
    prelude::*, 
    i2c::I2c3,
    pac::TIM9,
    pac::TIM10
};
use embedded_hdc1080_rs as hdc1080;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {


    use super::*;
    

    #[shared]
    struct Shared {
        tim10: stm32f4xx_hal::timer::Delay<TIM10, 1000>
    }

    #[local]
    struct Local {
        temp_sensor: hdc1080::Hdc1080<I2c3, stm32f4xx_hal::timer::Delay<TIM9, 1000>>
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        // Setup clocks
        let dp = cx.device;
        let rcc = dp.RCC.constrain();
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 48_000_000, systick_mono_token);
        let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

        // delay
        let tim9 = dp.TIM9.delay_ms(&clocks);
        let tim10 = dp.TIM10.delay_ms(&clocks);


        info!("hdc 1080 test");
        let gpioa = dp.GPIOA.split();
        let gpioc = dp.GPIOC.split();

        let i2c3_scl = gpioa.pa8.into_alternate_open_drain();
        let i2c3_sda = gpioc.pc9.into_alternate_open_drain();

        info!("i2c3...");
        let i2c3 = I2c3::new(dp.I2C3, (i2c3_scl, i2c3_sda), 400.kHz(), &clocks);

        let temp_sensor = hdc1080::Hdc1080::new(i2c3, tim9).unwrap();

        temp_task::spawn().unwrap();

        (Shared {tim10}, Local {temp_sensor})
    }

    #[task (local = [temp_sensor], shared=[tim10])]
    async fn temp_task(mut cx: temp_task::Context) {
        let temp_sensor = cx.local.temp_sensor;
        loop {
            cx.shared.tim10.lock( |tim10|  tim10.delay_ms(100u32));
            let temp = temp_sensor.temperature().unwrap();
            let hum = temp_sensor.humidity().unwrap();
            let scaled_temp = (temp * 100.0) as i32;
            let int_part = (scaled_temp / 100) as i16;
            let frac_part = (scaled_temp % 100) as u8;
            let scaled_hum = (hum * 100.0) as i32;
            let int_hum = (scaled_hum / 100) as i16;
            let frac_hum = (scaled_hum % 100) as u8;
            info!("Temperature: {}.{} C", int_part, frac_part);
            info!("Humidity: {}.{} %", int_hum, frac_hum);
        }
    }
}