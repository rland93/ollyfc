#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;
use rtic_monotonics::systick::Systick;
use stm32f4xx_hal::{
    pac::TIM1,
    pac::TIM10,
    prelude::*,
    timer::{Channel, Channel2, ChannelBuilder, Polarity, PwmHz, C2},
};

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        tim10: stm32f4xx_hal::timer::Delay<TIM10, 1000>,
        pwm: PwmHz<TIM1, ChannelBuilder<TIM1, C2, true>>,
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

        // delay
        let tim10 = dp.TIM10.delay_ms(&clocks);

        info!("buzzer test");
        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();

        let channels = Channel2::new(gpioa.pa9).with_complementary(gpiob.pb0);
        let mut pwm: PwmHz<TIM1, ChannelBuilder<TIM1, C2, true>> =
            dp.TIM1.pwm_hz(channels, 1500.Hz(), &clocks);
        let max_duty: u16 = pwm.get_max_duty();
        pwm.set_polarity(Channel::C2, Polarity::ActiveHigh);
        pwm.set_complementary_polarity(Channel::C2, Polarity::ActiveHigh);
        pwm.set_duty(Channel::C2, max_duty / 2);

        buzz_task::spawn().unwrap();

        (Shared {}, Local { tim10, pwm })
    }

    #[task (local = [tim10, pwm], shared=[])]
    async fn buzz_task(cx: buzz_task::Context) {
        cx.local.pwm.enable_complementary(Channel::C2);

        loop {
            info!("Playing arm sequence");
            play_sequence(cx.local.tim10, cx.local.pwm, &ARM_SEQUENCE);
            cx.local.tim10.delay_ms(500);

            info!("Playing error sequence");
            play_sequence(cx.local.tim10, cx.local.pwm, &ERROR_SEQUENCE);
            cx.local.tim10.delay_ms(500);

            info!("Playing nominal sequence");
            play_sequence(cx.local.tim10, cx.local.pwm, &NOMINAL_SEQUENCE);
            cx.local.tim10.delay_ms(500);

            info!("Playing USB connect sequence");
            play_sequence(cx.local.tim10, cx.local.pwm, &USB_CONNECT_SEQUENCE);
            cx.local.tim10.delay_ms(500);

            info!("Playing boot sequence");
            play_sequence(cx.local.tim10, cx.local.pwm, &BOOT_SEQUENCE);
            cx.local.tim10.delay_ms(500);

            info!("Playing power swap sequence");
            play_sequence(cx.local.tim10, cx.local.pwm, &POWER_SWAP_SEQUENCE);
            cx.local.tim10.delay_ms(500);
        }
    }

    fn play_sequence(
        tim10: &mut stm32f4xx_hal::timer::Delay<TIM10, 1000>,
        pwm: &mut PwmHz<TIM1, ChannelBuilder<TIM1, C2, true>>,
        sequence: &Sequence,
    ) {
        for (i, (freq, on_time, dead_time)) in sequence.iter().enumerate() {
            info!(
                "Tone {}: Frequency = {} Hz, On Time = {} ms, Dead Time = {} ms",
                i + 1,
                freq,
                on_time,
                dead_time
            );

            pwm.set_period(freq.Hz());
            pwm.set_duty(Channel::C2, pwm.get_max_duty() / 2);
            tim10.delay_ms(*on_time);
            pwm.set_duty(Channel::C2, 0);
            tim10.delay_ms(*dead_time);
        }
    }

    type Sequence = [(u32, u32, u32); 3];

    const ARM_SEQUENCE: Sequence = [(1000, 300, 50), (1200, 100, 25), (1600, 50, 500)];
    const ERROR_SEQUENCE: Sequence = [(800, 100, 50), (800, 100, 50), (800, 100, 500)];
    const NOMINAL_SEQUENCE: Sequence = [(1500, 150, 75), (1700, 150, 75), (1900, 150, 500)];
    const USB_CONNECT_SEQUENCE: Sequence = [(2000, 100, 50), (2200, 100, 50), (2400, 100, 500)];
    const BOOT_SEQUENCE: Sequence = [(1000, 50, 25), (1500, 50, 25), (2000, 50, 500)];
    const POWER_SWAP_SEQUENCE: Sequence = [(1800, 50, 25), (1600, 50, 25), (1200, 50, 500)];
}
