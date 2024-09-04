#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use bmp388::{Blocking, BMP388};
use num_traits::Pow;
use rtic_monotonics::systick_monotonic;
use rtic_monotonics::Monotonic;
systick_monotonic!(Mono, 1000);

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
        Mono::start(cx.core.SYST, sysclk.to_Hz());

        let gpioa = dp.GPIOA.split();
        let gpioc = dp.GPIOC.split();

        let scl = gpioa.pa8;
        let sda = gpioc.pc9;
        let i2c3 = i2c::I2c::new(dp.I2C3, (scl, sda), i2c::Mode::standard(400.kHz()), &clocks);

        let mut delay = dp.TIM2.delay_ms(&clocks);
        let sensor = bmp388::Config::builder()
            .address(bmp388::Addr::Secondary as u8)
            .oversampling(bmp388::config::OversamplingConfig {
                osr_pressure: bmp388::Oversampling::x4,
                osr_temperature: bmp388::Oversampling::x4,
            })
            .sampling_rate(bmp388::SamplingRate::ms5)
            .build()
            .setup_blocking(i2c3, &mut delay)
            .unwrap();
        defmt::info!("cfg address={:x}", bmp388::Addr::Secondary as u8);

        read_sensor::spawn().unwrap();

        (Shared {}, Local { sensor: sensor })
    }

    #[task(local=[sensor])]
    async fn read_sensor(cx: read_sensor::Context) {
        let sensor = cx.local.sensor;
        loop {
            let now = Mono::now();
            let data = sensor.sensor_values().unwrap();
            defmt::info!(
                "Altitude: {} m {} Pa {} C",
                altitude(data.pressure),
                data.pressure,
                data.temperature
            );

            Mono::delay_until(now + 20u32.millis()).await;
        }
    }
}

pub fn altitude(pressure: f64) -> f64 {
    let altitude = 44307.69396 * (1.0 - (pressure / 101325.0).pow(0.190284));
    altitude
}
