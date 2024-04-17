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
    pac::TIM1,
    pac::TIM10,
    timer::{Channel2, ChannelBuilder, PwmHz, Channel, Polarity, C2},
};

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {

    use super::*;
    
    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        tim10: stm32f4xx_hal::timer::Delay<TIM10, 1000>,
        pwm: PwmHz<TIM1, ChannelBuilder<TIM1, C2, true>> 
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
        let tim10 = dp.TIM10.delay_ms(&clocks);

        info!("buzzer test");
        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();

        let channels = Channel2::new(gpioa.pa9).with_complementary(gpiob.pb0);
        let mut pwm: PwmHz<TIM1, ChannelBuilder<TIM1, C2, true>> = dp.TIM1.pwm_hz(channels, 1500.Hz(), &clocks);
        let max_duty: u16 = pwm.get_max_duty();
        pwm.set_polarity(Channel::C2, Polarity::ActiveHigh);
        pwm.set_complementary_polarity(Channel::C2, Polarity::ActiveHigh);
        pwm.set_duty(Channel::C2, max_duty / 2);

        buzz_task::spawn().unwrap();

        (Shared {}, Local {tim10, pwm })
    }

    #[task (local = [tim10, pwm], shared=[])]
    async fn buzz_task(cx: buzz_task::Context) {
        cx.local.pwm.enable_complementary(Channel::C2);
        
        let notes = [
            1318,
            1975,
            1975,
            1480,
            1318,
            1318,
            987,
            1318,
            1318,
            1480,
            1318,
        ];
        
        let timings = [
            500,
            250,
            200,
            500,
            250,
            200,
            500,
            250,
            200,
            500,
            300,
        ];
        
        let dead_times= [
            50,
            200,
            300,
            50,
            200,
            300,
            50,
            200,
            300,
            50,
            200,
        ];
  
        loop {
            // Use `iter()` and `zip()` to combine iterators
            for (i, ((note, duration), dead_time)) in notes.iter().zip(timings.iter()).zip(dead_times.iter()).enumerate() {
                info!("Note {}: Frequency = {}, Duration = {} ms, Dead Time = {} ms", i + 1, note, duration, dead_time);

                cx.local.pwm.set_period(note.Hz());
                cx.local.pwm.set_duty(Channel::C2, cx.local.pwm.get_max_duty() / 2);
                cx.local.tim10.delay_ms(*duration);
                cx.local.pwm.set_duty(Channel::C2, 0);
                cx.local.tim10.delay_ms(*dead_time);
            }
            cx.local.tim10.delay_ms(1000u32);

        }
    }
}