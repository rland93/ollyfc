#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use defmt::info;
use defmt_rtt as _;
use libm;
use ollyfc::pwm::servo::Servo;
use panic_probe as _;
use pwm_pca9685::Channel;
use rtic_monotonics::systick_monotonic;
use rtic_monotonics::Monotonic;
systick_monotonic!(Mono, 1000);
use stm32f4xx_hal::{i2c::I2c1, pac::TIM10, prelude::*};

use pwm_pca9685::Pca9685;

const PCA_9685_ADDR: u8 = 0x41;
const PI: f32 = 3.14159265359;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {

    use super::*;

    #[shared]
    struct Shared {
        tim10: stm32f4xx_hal::timer::Delay<TIM10, 1000>,
        pwm: Pca9685<I2c1>,
    }

    #[local]
    struct Local {}

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

        // delay
        let tim10 = dp.TIM10.delay_ms(&clocks);

        info!("pca 9865 test");
        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();

        let i2c1_scl = gpiob.pb8.into_alternate_open_drain();
        let i2c1_sda = gpiob.pb9.into_alternate_open_drain();

        info!("i2c1...");
        let i2c1 = I2c1::new(dp.I2C1, (i2c1_scl, i2c1_sda), 400.kHz(), &clocks);

        let mut pwm = Pca9685::new(i2c1, PCA_9685_ADDR).unwrap();
        pwm.enable().unwrap();
        pwm.set_prescale(121).unwrap();

        info!("pwm enable pin");
        let mut servo_en = gpioc.pc3.into_push_pull_output();
        servo_en.set_low();

        servo_task::spawn().unwrap();
        led_task::spawn().unwrap();

        (Shared { tim10, pwm }, Local {})
    }

    #[task(shared=[pwm])]
    async fn led_task(mut cx: led_task::Context) {
        let pwm_range = 4095;
        let frequency = 1.0; // heartbeat frequency (s)
        let step_size = 0.1; // step size (s)

        loop {
            for i in (0..360).step_by(5) {
                let now = Mono::now();
                let rad = (i as f32) * PI / 180.0;
                let sine_wave = match libm::sinf(rad) {
                    x if x > 0.0 => x,
                    _ => 0.0,
                };
                let duty_cycle = (sine_wave * pwm_range as f32) as u16;

                cx.shared.pwm.lock(|pwm| {
                    pwm.set_channel_on_off(Channel::C11, 0, duty_cycle).unwrap();
                    pwm.set_channel_on_off(Channel::C12, duty_cycle, pwm_range)
                        .unwrap();
                });

                let delay = (step_size / frequency * 1000.0) as u32;

                Mono::delay_until(now + delay.millis()).await;
            }
        }
    }

    #[task(shared=[tim10, pwm])]
    async fn servo_task(mut cx: servo_task::Context) {
        defmt::debug!("Servo task");
        let servo = Servo::new(209, 408);

        loop {
            for (i, _) in (0..180).step_by(1).zip(0..180).step_by(1) {
                let now = Mono::now();

                let angle1 = i as f32;
                let counts1 = servo.angle_to_counts(angle1);

                let angle2 = i as f32;
                let counts2 = servo.angle_to_counts(angle2);

                if i == 0 {
                    defmt::debug!("set");
                }

                cx.shared.pwm.lock(|pwm| {
                    pwm.set_channel_on_off(Channel::C1, counts1.on, counts1.off)
                        .unwrap();
                    pwm.set_channel_on_off(Channel::C2, counts2.on, counts2.off)
                        .unwrap();
                });

                Mono::delay_until(now + 25u32.millis()).await;
            }
        }
    }
}
