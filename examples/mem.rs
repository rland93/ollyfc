#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use panic_probe as _;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {
    use defmt::info;
    use defmt_rtt as _;

    use ollyfc::w25q;
    use rtic_monotonics::systick::Systick;

    use stm32f4xx_hal::{
        gpio::{Output, Pin},
        pac::SPI1,
        prelude::*,
        spi::{Mode, Phase, Polarity, Spi},
    };

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        mem: w25q::W25Q,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        // Setup clocks
        let rcc = cx.device.RCC.constrain();
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 48_000_000, systick_mono_token);
        let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();
        info!("Booted. Initializing...");

        info!("Peripherals...");
        info!("gpio...");
        // Peripherals
        let gpioa = cx.device.GPIOA.split();

        info!("spi1, mem...");
        let spi1_sck = gpioa.pa5.into_alternate();
        let spi1_miso = gpioa.pa6.into_alternate();
        let spi1_mosi = gpioa.pa7.into_alternate();
        let spi1_cs: Pin<'A', 4, Output> = gpioa.pa4.into_push_pull_output();
        let spi1: Spi<SPI1> = Spi::new(
            cx.device.SPI1,
            (spi1_sck, spi1_miso, spi1_mosi),
            Mode {
                polarity: Polarity::IdleLow,
                phase: Phase::CaptureOnFirstTransition,
            },
            100.kHz(),
            &clocks,
        );
        let timer = cx.device.TIM3.delay_us(&clocks);
        let mem = w25q::W25Q::new(spi1, spi1_cs, timer);
        info!("Initialized peripherals");
        info!("Spawning test task...");
        mem_test_task::spawn().unwrap();

        (Shared {}, Local { mem })
    }

    #[task (local = [mem])]
    async fn mem_test_task(cx: mem_test_task::Context) {
        let mem = cx.local.mem;

        let (a, b) = mem.unique_id().unwrap();
        info!("Unique ID: {:?}, {:?}", a, b);

        let mut to_read = [0u8; 256];
        mem.read(0x00000000, &mut to_read).unwrap();
        info!("Read: {:?}", to_read);

        info!("Erase sector");
        mem.sector_erase(0x000000).unwrap();

        mem.read(0x00000000, &mut to_read).unwrap();
        info!("Read: {:?}", to_read);

        info!("Write page");
        let mut to_write = [0u8; 256];
        for i in 0..64 {
            to_write[i] = i as u8;
        }
        mem.page_program(0x00000000, &to_write).unwrap();

        info!("Reset the chip and verify a successful read on boot to complete the test.");
    }
}
