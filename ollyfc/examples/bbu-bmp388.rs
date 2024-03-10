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
    use stm32f4xx_hal::{gpio::Edge, i2c::I2c1, prelude::*};

    #[shared]
    struct Shared {
        psens: bmp388::BMP388<I2c1, Blocking>,
    }

    #[local]
    struct Local {
        pres_int: stm32f4xx_hal::gpio::Pin<'C', 1>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let mut dp = ctx.device;

        let rcc = dp.RCC.constrain();
        let hse = 16.MHz();
        let sysclk = 48.MHz();
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
        let mut pres_int_pin = gpioc.pc1.into_pull_down_input();
        pres_int_pin.make_interrupt_source(&mut syscfg);
        pres_int_pin.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
        pres_int_pin.enable_interrupt(&mut dp.EXTI);
        info!("Setup completed.");

        (
            Shared { psens: sensor },
            Local {
                pres_int: pres_int_pin,
            },
        )
    }

    #[task(binds = EXTI1, shared=[psens], local=[pres_int])]
    fn pres_int(mut cx: pres_int::Context) {
        // Spawn the pressure s/w task.
        pressure_task::spawn().unwrap_or_else(|_| {
            log::log_err(log::err_os_h(log::RticErr::SpawnFail));
            panic!()
        });

        // clear interrupt status
        cx.shared.psens.lock(|psens| {
            // Reset the interrupt
            let int_status = psens.int_status().unwrap_or_else(|e| {
                log::log_err(log::err_i2c_h(e));
                panic!()
            });

            // Should never hit!
            if !int_status.data_ready {
                let err = bmp388::Error {
                    fatal: true,
                    cmd: false,
                    config: false,
                };
                log::log_err(log::err_pres_h(err))
            }
        });

        cx.local.pres_int.clear_interrupt_pending_bit();
    }

    #[task(shared=[psens], local=[])]
    async fn pressure_task(mut cx: pressure_task::Context) {
        let (alt, temp): (f64, f64) = cx.shared.psens.lock(|psens| {
            let a = psens.altitude().unwrap_or_else(|e| {
                log::log_err(log::err_i2c_h(e));
                panic!()
            });
            let t = psens
                .sensor_values()
                .unwrap_or_else(|e| {
                    log::log_err(log::err_i2c_h(e));
                    panic!()
                })
                .temperature;
            (a, t)
        });

        info!("Altitude: {}m, Temp {}", alt, temp);
    }
}
