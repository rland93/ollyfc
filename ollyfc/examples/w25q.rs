#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(generic_arg_infer)]
#![feature(stmt_expr_attributes)]

use defmt::{debug, info};
use embedded_hal_bus::spi::ExclusiveDevice;
use stm32f4xx_hal::{pac, prelude::*, spi};

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = stm32f4xx_hal::pac)]
mod app {

    use embedded_io::{Read, Seek, SeekFrom, Write};
    use rtic_monotonics::{systick::Systick, Monotonic};
    use stm32f4xx_hal::timer::Delay;

    use super::*;
    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        w25q_dev: w25q::W25Q<
            ExclusiveDevice<
                spi::Spi<pac::SPI1>,
                stm32f4xx_hal::gpio::Pin<'A', 4, stm32f4xx_hal::gpio::Output>,
                embedded_hal_bus::spi::NoDelay,
            >,
            Delay<pac::TIM2, 1000>,
        >,
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

        let gpioa = dp.GPIOA.split();
        let delay = dp.TIM2.delay_ms(&clocks);

        let spi1 = spi::Spi1::new(
            dp.SPI1,
            (
                gpioa.pa5.into_alternate(),
                gpioa.pa6.into_alternate(),
                gpioa.pa7.into_alternate(),
            ),
            spi::Mode {
                polarity: spi::Polarity::IdleLow,
                phase: spi::Phase::CaptureOnFirstTransition,
            },
            1.MHz(),
            &clocks,
        );

        let cs = gpioa.pa4.into_push_pull_output();
        let spidev = ExclusiveDevice::new_no_delay(spi1, cs).unwrap();
        let w25q_dev = w25q::W25Q::new_with_spi(spidev, delay);

        info!("device initialized {}", w25q_dev);

        mem_test::spawn().unwrap();

        (Shared {}, Local { w25q_dev })
    }

    #[task(local=[w25q_dev])]
    async fn mem_test(cx: mem_test::Context) {
        let w25q_dev = cx.local.w25q_dev;
        loop {
            let now = rtic_monotonics::systick::Systick::now();

            info!("test starts...");

            // read 256 bytes from the beginning of the flash
            info!("reading 256 bytes from the beginning of the flash");
            let mut buf = [0u8; 256];
            w25q_dev.seek(SeekFrom::Start(0)).unwrap();
            let bytes_read = w25q_dev.read(&mut buf).unwrap();
            debug!("read {} bytes", bytes_read);
            debug!("read: {:?}", buf[0..4]);

            // erase
            info!("erasing 32KB block at 0");
            w25q_dev.block_erase_32kb(0).unwrap();
            let mut buf = [0u8; 256];
            w25q_dev.seek(SeekFrom::Start(0)).unwrap();
            let bytes_read = w25q_dev.read(&mut buf).unwrap();
            debug!("read {} bytes", bytes_read);
            debug!("read: {:?}", buf[0..4]);

            // write
            info!("writing 256 bytes of 0xAA to the beginning of the flash");
            w25q_dev.seek(SeekFrom::Start(0)).unwrap();
            let bytes_to_write = [0xAAu8; 256];
            let bytes_written = w25q_dev.write(&bytes_to_write).unwrap();
            debug!("wrote {} bytes = {:x}", bytes_written, 0xAAu8);

            // read
            info!("reading 256 bytes from the beginning of the flash");
            let mut buf = [0u8; 256];
            w25q_dev.seek(SeekFrom::Start(0)).unwrap();
            let bytes_read = w25q_dev.read(&mut buf).unwrap();
            debug!("read {} bytes", bytes_read);
            debug!("read: {:?}", buf[0..4]);

            rtic_monotonics::systick::Systick::delay_until(now + 5000u32.millis()).await;
        }
    }
}
