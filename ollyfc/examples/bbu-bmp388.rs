// BOARD BRING-UP (BBU) UTILITY for atmospheric pressure sensor.
//
// This file is intended to quickly verify that the BMP388 is working properly.
// See BST-BMP388-DS001-07
//

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers=[TIM4])]
mod app {
    use bmp388::Blocking;
    use defmt::info;
    use ollyfc::{err::log, sensor::pressure::BMP388_CFG};
    use rtic_monotonics::Monotonic;
    use stm32f4xx_hal::{gpio::Edge, i2c::I2c1, pac::EXTI, prelude::*};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        psens: bmp388::BMP388<I2c1, Blocking>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let mut dp = ctx.device;

        let rcc = dp.RCC.constrain();
        let hse = 16.MHz();
        let sysclk = 16.MHz();
        let clocks = rcc
            .cfgr
            .use_hse(hse)
            .sysclk(sysclk)
            .require_pll48clk()
            .freeze();

        let mut syscfg = dp.SYSCFG.constrain();
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        rtic_monotonics::systick::Systick::start(ctx.core.SYST, 64_000_000, systick_mono_token);

        // GPIO
        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();

        // I2C
        info!("i2c...");
        let i2c1_scl = gpiob.pb6.into_alternate_open_drain();
        let i2c1_sda = gpiob.pb7.into_alternate_open_drain();
        let i2c1 = I2c1::new(dp.I2C1, (i2c1_scl, i2c1_sda), 400.kHz(), &clocks);
        let mut delay = dp.TIM1.delay_us(&clocks);
        info!("bmp388...");
        let sensor =
            bmp388::Config::setup_blocking(&BMP388_CFG, i2c1, &mut delay).unwrap_or_else(|e| {
                log::log_err(log::err_i2c_h(e));
                panic!()
            });
        info!("interrupt...");
        let mut pres_int_pin = gpioc.pc1.into_pull_up_input();
        pres_int_pin.make_interrupt_source(&mut syscfg);
        pres_int_pin.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
        pres_int_pin.enable_interrupt(&mut dp.EXTI);
        unsafe { cortex_m::peripheral::NVIC::unmask(pres_int_pin.interrupt()) }
        info!("Setup completed.");

        (Shared {}, Local { psens: sensor })
    }

    #[task(binds = EXTI1)]
    fn pres_int(cx: pres_int::Context) {
        defmt::debug!("new pressure data.");
    }

    #[task(local=[psens], shared=[])]
    async fn pressure_task(mut cx: pressure_task::Context) {
        info!("Exercising pressure sensor...");
        let sensor = cx.local.psens;

        // 500 ms loop to read sensor values
        loop {
            let now = rtic_monotonics::systick::Systick::now();

            let status = sensor.status().unwrap_or_else(|e| {
                log::log_err(log::err_i2c_h(e));
                panic!()
            });

            info!(
                "status: p={:?} t={:?} c={:?}",
                status.pressure_data_ready, status.temperature_data_ready, status.command_ready
            );

            let alt = sensor.altitude().unwrap_or_else(|e| {
                log::log_err(log::err_i2c_h(e));
                panic!()
            });

            info!("altitude: {:?}", alt);

            rtic_monotonics::systick::Systick::delay_until(now + 50u32.millis()).await;
        }
    }
}
