// BOARD BRING-UP (BBU) UTILITY for 6 axis IMU
//
// This file is intended to quickly verify that the BMI088 is working properly.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt_rtt as _;
use panic_probe as _;

use defmt::{debug, info};
use rtic_monotonics::Monotonic;

use bmi088::interface::I2cInterface;
use stm32f4xx_hal::{i2c::I2c1, prelude::*};

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {
    use ollyfc::err::log;
    use stm32f4xx_hal::{
        gpio::{gpioc, Edge},
        pac::TIM1,
    };

    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        sensor: bmi088::Bmi088<I2cInterface<I2c1>>,
        delay: stm32f4xx_hal::timer::Delay<TIM1, 1000>,
        //gyro_int: stm32f4xx_hal::gpio::Pin<'C', 3>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let mut dp = ctx.device;

        let rcc = dp.RCC.constrain();
        let hse = 16.MHz();
        let sysclk = 32.MHz();
        let clocks = rcc
            .cfgr
            .use_hse(hse)
            .sysclk(sysclk)
            .require_pll48clk()
            .freeze();

        let mut syscfg = dp.SYSCFG.constrain();
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        rtic_monotonics::systick::Systick::start(ctx.core.SYST, 48_000_000, systick_mono_token);

        // GPIO
        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();

        // I2C
        info!("i2c...");
        let i2c1_scl = gpiob.pb6.into_alternate_open_drain();
        let i2c1_sda = gpiob.pb7.into_alternate_open_drain();
        let i2c1 = I2c1::new(dp.I2C1, (i2c1_scl, i2c1_sda), 400.kHz(), &clocks);
        let mut delay = dp.TIM1.delay_ms(&clocks);
        info!("bmp088...");
        let mut sensor = bmi088::Bmi088::new_with_i2c(i2c1, false, false);

        // reset
        info!("reset sensor...");
        sensor.acc_soft_reset().unwrap();
        delay.delay_ms(50);
        sensor.gyro_soft_reset().unwrap();
        delay.delay_ms(250);
        info!("done!");
        // enable both
        sensor.acc_enable_write(bmi088::AccPowerEnable::On).unwrap();
        sensor
            .acc_wake_suspend_write(bmi088::AccPowerConf::Active)
            .unwrap();

        sensor
            .gyro_power_mode_write(bmi088::GyroPowerMode::Normal)
            .unwrap();

        // configure accelerometer
        let configuration = bmi088::AccelerometerConfig {
            conf: bmi088::AccConf {
                acc_bwp: bmi088::AccBandwidth::X4,
                acc_odr: bmi088::AccDataRate::Hz50,
            },
            acc_range: bmi088::AccRange::G3,
        };
        sensor.acc_configuration_write(configuration).unwrap();

        // configure gyroscope
        let bw = bmi088::GyroBandwidth::Hz32;
        let range = bmi088::GyroRange::Dps2000;
        sensor.gyro_bandwidth_write(bw).unwrap();
        sensor.gyro_range_write(range).unwrap();

        // interrupt
        // let pin_active = bmi088::PinActive::ActiveHigh;
        // let pin_behavior = bmi088::PinBehavior::PushPull;

        // sensor.acc_map_drdy(bmi088::AccDrdyMap::Int1).unwrap();

        // // set up interrupt
        // let mut gyro_int_pin = gpioc.pc3.into_pull_down_input();
        // gyro_int_pin.make_interrupt_source(&mut syscfg);
        // gyro_int_pin.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
        // gyro_int_pin.enable_interrupt(&mut dp.EXTI);

        info!("Setup completed.");

        imu_task::spawn().unwrap();

        (
            Shared {},
            Local {
                sensor,
                delay,
                //gyro_int: gyro_int_pin,
            },
        )
    }

    #[task(shared=[], local=[sensor, delay])]
    async fn imu_task(cx: imu_task::Context) {
        let s = cx.local.sensor;

        loop {
            let now = rtic_monotonics::systick::Systick::now();
            info!("imu task");
            let cid = s.acc_chip_id().unwrap();

            let time = s.sensor_time_24bit().unwrap();
            let temp = s.temperature().unwrap();
            let acc = s.acc_data().unwrap();
            let gyro = s.gyro_read_rate().unwrap();

            debug!(
                "t={:05}\ttemp={:05}\tacc=[{:05},{:05},{:05}]\tgyro=[{:05},{:05},{:05}]",
                time, temp, acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z
            );

            rtic_monotonics::systick::Systick::delay_until(now + 200u32.millis()).await;
        }
    }
}
