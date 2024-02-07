#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt_rtt as _;
use panic_probe as _;

use bmi160::Bmi160;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers=[TIM4])]
mod app {
    use bmi160::{
        AccelerometerPowerMode, AccelerometerRange, GyroRange, GyroscopePowerMode,
        MagnetometerData, Sensor3DData, SlaveAddr,
    };
    use defmt::info;
    use rtic_monotonics::{systick::Systick, Monotonic};
    use stm32f4xx_hal::{
        i2c::{I2c, I2c1},
        pac::I2C1,
        prelude::*,
    };

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        imu: bmi160::Bmi160<bmi160::interface::I2cInterface<I2c1>>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let dp = ctx.device;
        let rcc = dp.RCC.constrain();
        // Setup system clocks
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
        Systick::start(ctx.core.SYST, 64_000_000, systick_mono_token);

        // Set up BMI160
        info!("Set up i2c");
        let gpiob = dp.GPIOB.split();
        let scl = gpiob.pb6.into_alternate_open_drain();
        let sda = gpiob.pb7.into_alternate_open_drain();
        let i2c_dev = dp.I2C1.i2c((scl, sda), 400u32.kHz(), &clocks);

        info!("Set up BMI160");
        let mut imu = bmi160::Bmi160::new_with_i2c(i2c_dev, SlaveAddr::Alternative(false));
        let cid = imu.chip_id().unwrap();
        info!("Chip ID: {:04x}", cid);

        imu.set_accel_power_mode(AccelerometerPowerMode::Normal)
            .unwrap();
        imu.set_gyro_power_mode(GyroscopePowerMode::Normal).unwrap();
        imu.set_magnet_power_mode(bmi160::MagnetometerPowerMode::Normal)
            .unwrap();
        sensor_task::spawn().unwrap();
        (Shared {}, Local { imu: imu })
    }

    #[task(local=[imu])]
    async fn sensor_task(cx: sensor_task::Context) {
        let imu = cx.local.imu;

        let mut tictoc = 0u32;
        loop {
            let now = Systick::now();
            info!("Sensor task: {}", now.ticks());

            let selector = bmi160::SensorSelector::new().accel().gyro();
            let data = match imu.data(selector) {
                Ok(d) => d,
                Err(e) => {
                    info!("Error reading data");
                    Systick::delay_until(now + 250u32.millis()).await;

                    continue;
                }
            };

            let default_3ddata = Sensor3DData {
                x: -1,
                y: -1,
                z: -1,
            };

            let default_magnetometer = MagnetometerData {
                axes: default_3ddata,
                hall_resistence: u16::MAX,
            };

            let accel = match data.accel {
                Some(a) => a,
                None => {
                    defmt::debug!("No accel data");
                    default_3ddata
                }
            };
            let gyro = match data.gyro {
                Some(g) => g,
                None => {
                    defmt::debug!("No gyro data");
                    default_3ddata
                }
            };

            info!(
                "Accel: [{} {} {}], Gyro: [{} {} {}]",
                accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z,
            );

            let mut accrange = AccelerometerRange::G2;
            let mut gyrorange = GyroRange::Scale125;
            if tictoc % 2 == 0 {
                defmt::debug!("Setting accel range to G2");
                imu.set_accel_range(AccelerometerRange::G2).unwrap();
                imu.set_gyro_range(GyroRange::Scale125);
            } else {
                defmt::debug!("Setting accel range to G8");
                imu.set_accel_range(AccelerometerRange::G8).unwrap();
                imu.set_gyro_range(GyroRange::Scale2000);
            }

            tictoc += 1;

            Systick::delay_until(now + 500u32.millis()).await;
        }
    }
}
