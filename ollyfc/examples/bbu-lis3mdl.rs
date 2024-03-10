#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt_rtt as _;
use panic_probe as _;

use defmt::{debug, info};
use ollyfc::err::log;
use stm32f4xx_hal::gpio::Edge;
use stm32f4xx_hal::i2c::I2c1;
use stm32f4xx_hal::prelude::*;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {

    use super::*;

    #[shared]
    struct Shared {
        magnetometer: lis3mdl::Lis3mdl<I2c1>,
    }

    #[local]
    struct Local {
        mag_int: stm32f4xx_hal::gpio::Pin<'C', 0>,
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
        rtic_monotonics::systick::Systick::start(ctx.core.SYST, 32_000_000, systick_mono_token);

        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();

        // i2c
        info!("i2c...");

        let i2c1_sda = gpiob.pb7.into_alternate_open_drain();
        let i2c1_scl = gpiob.pb6.into_alternate_open_drain();
        let i2c1 = I2c1::new(dp.I2C1, (i2c1_scl, i2c1_sda), 400.kHz(), &clocks);

        // lis3mdl
        info!("lis3mdl...");
        let mut magnetometer = lis3mdl::Lis3mdl::new(i2c1, lis3mdl::Address::Addr1C)
            .unwrap_or_else(|e| {
                log::log_err(log::err_mag_h(e));
                panic!()
            });

        // set rate
        magnetometer
            .set_data_rate(lis3mdl::DataRate::ODR_80Hz)
            .unwrap_or_else(|e| {
                log::log_err(log::err_mag_h(e));
                panic!()
            });

        // operating mode
        magnetometer
            .set_operating_mode(lis3mdl::OperatingMode::UltraHighPerformance)
            .unwrap_or_else(|e| {
                log::log_err(log::err_mag_h(e));
                panic!()
            });

        // sensitivity
        magnetometer
            .set_full_scale(lis3mdl::FullScale::Fs4g)
            .unwrap_or_else(|e| {
                log::log_err(log::err_mag_h(e));
                panic!()
            });

        // block data read update
        magnetometer
            .set_block_data_update(true)
            .unwrap_or_else(|e| {
                log::log_err(log::err_mag_h(e));
                panic!()
            });

        // continuous
        magnetometer
            .set_measurement_mode(lis3mdl::MeasurementMode::Continuous)
            .unwrap_or_else(|e| {
                log::log_err(log::err_mag_h(e));
                panic!()
            });

        // interrupt configuration
        let mut mag_int_pin = gpioc.pc0.into_pull_down_input();
        mag_int_pin.make_interrupt_source(&mut syscfg);
        mag_int_pin.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
        mag_int_pin.enable_interrupt(&mut dp.EXTI);

        rtic::pend(stm32f4xx_hal::pac::Interrupt::EXTI0);
        (
            Shared {
                magnetometer: magnetometer,
            },
            Local {
                mag_int: mag_int_pin,
            },
        )
    }

    #[task(binds = EXTI0, shared=[magnetometer], local=[mag_int])]
    fn mag_int(cx: mag_int::Context) {
        // spawn the magnetometer s/w task
        magnetometer_task::spawn().unwrap_or_else(|_| {
            log::log_err(log::err_os_h(log::RticErr::SpawnFail));
            panic!()
        });

        // clear pending
        cx.local.mag_int.clear_interrupt_pending_bit();
    }

    #[task(shared=[magnetometer], local=[])]
    async fn magnetometer_task(cx: magnetometer_task::Context) {
        let mut magnetometer = cx.shared.magnetometer;
        let raw = magnetometer.lock(|m| {
            m.get_raw_mag_axes().unwrap_or_else(|e| {
                log::log_err(log::err_mag_h(e));
                panic!()
            })
        });

        let x_mgauss = (raw.x as i32 * 1000) / 6842;
        let y_mgauss = (raw.y as i32 * 1000) / 6842;
        let z_mgauss = (raw.z as i32 * 1000) / 6842;

        debug!("Field [{:?},{:?},{:?}]", x_mgauss, y_mgauss, z_mgauss);
    }
}
