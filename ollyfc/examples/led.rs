#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use rtic_monotonics::systick::Systick;
use stm32f4xx_hal::{pac, prelude::*};

#[rtic::app(device = stm32f4xx_hal::pac)]
mod app {
    use stm32f4xx_hal::timer::{self, Channel3};

    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led_pwm: timer::Pwm<pac::TIM2, Channel3<pac::TIM2>, 1000_000>,
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

        // Configure GPIO pin B10 for PWM output
        let gpiob = dp.GPIOB.split();
        let channels = Channel3::new(gpiob.pb10);
        let mut pwm = dp.TIM2.pwm_us(channels, 100.micros(), &clocks);
        pwm.enable(timer::Channel::C3);
        pwm.set_polarity(timer::Channel::C3, timer::Polarity::ActiveHigh);

        blink_led::spawn().unwrap();

        (Shared {}, Local { led_pwm: pwm })
    }

    #[task(local = [led_pwm])]
    async fn blink_led(cx: blink_led::Context) {
        let led_pwm = cx.local.led_pwm;
        let max_duty = led_pwm.get_max_duty();
        let mut increasing = true;
        let mut duty: u16 = 0;

        loop {
            if increasing {
                duty += max_duty / 100; // Increment by 1%
                if duty >= max_duty {
                    increasing = false;
                }
            } else {
                duty = duty.saturating_sub(max_duty / 100);
                if duty == 0 {
                    increasing = true;
                }
            }

            led_pwm.set_duty(timer::Channel::C3, duty);
            Systick::delay(10.millis()).await;
        }
    }
}
