#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_variables)]

use panic_probe as _;
mod w25q;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {
    use crate::w25q::W25Q;
    use defmt::info;
    use defmt_rtt as _;
    use rtic_monotonics::systick::Systick;
    use stm32f4xx_hal::{
        gpio::{Output, Pin},
        pac::SPI2,
        prelude::*,
        spi::{Mode, Phase, Polarity, Spi},
        timer::DelayUs,
    };

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        mem: W25Q,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        // Setup clocks
        let rcc = cx.device.RCC.constrain();
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 48_000_000, systick_mono_token);

        let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

        info!("Booting...");

        // Peripherals
        let gpiob = cx.device.GPIOB.split();
        let gpioa = cx.device.GPIOA.split();

        // SPI2
        let spi2_sck = gpiob.pb13.into_alternate();
        let spi2_miso = gpiob.pb14.into_alternate();
        let spi2_mosi = gpiob.pb15.into_alternate();
        let spi2_cs: Pin<'A', 12, Output> = gpioa.pa12.into_push_pull_output();
        let spi2: Spi<SPI2> = Spi::new(
            cx.device.SPI2,
            (spi2_sck, spi2_miso, spi2_mosi),
            Mode {
                polarity: Polarity::IdleLow,
                phase: Phase::CaptureOnFirstTransition,
            },
            100.kHz(),
            &clocks,
        );
        let timer = cx.device.TIM3.delay_us(&clocks);

        let mem = W25Q::new(spi2, spi2_cs, timer);

        info!("Finished init.");

        mem_task::spawn().unwrap();

        (Shared {}, Local { mem })
    }

    #[task(local=[mem])]
    async fn mem_task(cx: mem_task::Context) {
        info!("Sector Erase...");
        cx.local.mem.sector_erase(0x000000).unwrap();

        cx.local.mem.timer.delay_ms(100u32);

        info!("Page Program...");
        let mut buf = [0u8; 256];
        for i in 0..4 {
            buf[i] = i as u8;
        }
        cx.local.mem.page_program(0x000000, &buf).unwrap();

        info!("Read Page...");
        let mut buf = [0u8; 256];
        cx.local.mem.read(0x000000, &mut buf).unwrap();
        info!("Page: {:?}", buf);
    }
}
