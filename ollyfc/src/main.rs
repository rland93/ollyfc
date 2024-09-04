#![no_std]
#![no_main]

/// modules
use ollyfc::io;

/// configuration
use defmt_rtt as _;
use panic_probe as _;

/// system
use rtic_monotonics::systick_monotonic;
systick_monotonic!(Mono, 1000);

/// hal
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::{dma, gpio, pac, serial, spi};
use w25q::W25Q;

/// type
type SpiFlash = W25Q<
    ExclusiveDevice<spi::Spi<pac::SPI1>, gpio::Pin<'A', 4, gpio::Output>, NoDelay>,
    stm32f4xx_hal::timer::Delay<pac::TIM2, 1000>,
>;

type RxTransfer = dma::Transfer<
    dma::Stream2<pac::DMA2>,
    4,
    serial::Rx<pac::USART1>,
    dma::PeripheralToMemory,
    &'static mut [u8; SBUS_BUFFER_SIZE],
>;

const SBUS_BUFFER_SIZE: usize = 25;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [USART1, I2C2_ER, I2C2_EV, SPI4])]
mod app {

    use stm32f4xx_hal::serial;

    use super::*;

    #[shared]
    struct Shared {
        sbus_transfer: RxTransfer,
    }

    #[local]
    struct Local {
        _mem: SpiFlash,
        sbus_buffer: Option<&'static mut [u8; SBUS_BUFFER_SIZE]>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let dp = cx.device;
        let rcc = dp.RCC.constrain();
        let hse = 16.MHz();
        let sysclk = 64.MHz();
        let clocks = rcc
            .cfgr
            .use_hse(hse)
            .sysclk(sysclk)
            .require_pll48clk()
            .freeze();

        let _syscfg = dp.SYSCFG.constrain();
        Mono::start(cx.core.SYST, sysclk.to_Hz());

        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();

        // SPI Flash
        let spi1 = dp.SPI1.spi(
            (
                gpioa.pa5.into_alternate(),
                gpioa.pa6.into_alternate(),
                gpioa.pa7.into_alternate(),
            ),
            embedded_hal::spi::MODE_0,
            8.MHz(),
            &clocks,
        );
        let cs = gpioa.pa4.into_push_pull_output();
        let spidev = ExclusiveDevice::new_no_delay(spi1, cs).unwrap();
        let mem = W25Q::new_with_spi(spidev, dp.TIM2.delay_ms(&clocks));
        // SPI Flash

        // SBUS UART
        let rx_pin = gpiob.pb7.into_alternate();
        let mut rx = dp
            .USART1
            .rx(
                rx_pin,
                serial::Config::default()
                    .baudrate(100_000.bps())
                    .dma(serial::config::DmaConfig::Rx),
                &clocks,
            )
            .unwrap();
        rx.listen_idle();
        let sbus_buffer = cortex_m::singleton!(: [u8; SBUS_BUFFER_SIZE] = [0; SBUS_BUFFER_SIZE]);
        let mut sbus_transfer = dma::Transfer::init_peripheral_to_memory(
            dma::StreamsTuple::new(dp.DMA2).2,
            rx,
            cortex_m::singleton!(: [u8; SBUS_BUFFER_SIZE] = [0; SBUS_BUFFER_SIZE]).unwrap(),
            None,
            dma::config::DmaConfig::default()
                .memory_increment(true)
                .fifo_enable(true)
                .fifo_error_interrupt(true)
                .transfer_complete_interrupt(true),
        );
        sbus_transfer.start(|_rx| {});
        // SBUS UART

        (
            Shared { sbus_transfer },
            Local {
                _mem: mem,
                sbus_buffer,
            },
        )
    }

    #[task(priority=3, binds = DMA2_STREAM2, shared = [sbus_transfer], local = [sbus_buffer])]
    fn sbus_data(mut cx: sbus_data::Context) {
        cx.shared.sbus_transfer.lock(|xfer: &mut RxTransfer| {
            if xfer.is_idle() {
                let new_buf = cx.local.sbus_buffer.take().unwrap();
                let (buf, _current) = xfer.next_transfer(new_buf).unwrap();
                let chan_data = io::sbus::SbusData::new(*buf).parse();
                // free the buffer
                cx.local.sbus_buffer.replace(buf);
                match chan_task::spawn(chan_data) {
                    Ok(_) => (),
                    Err(e) => {
                        defmt::error!("Error spawning task: {:?}", e);
                        panic!();
                    }
                };
            }
        });
    }

    #[task()]
    async fn chan_task(_cx: chan_task::Context, chan_data: io::sbus::SbusChannels) {
        let inputs = chan_data.get_input();

        defmt::info!(
            "throttle: {}\t aileron: {}\t elevator: {}\t rudder: {}\t, arm: {}\t, enable: {}\t, record: {}\t",
            inputs.throttle,
            inputs.aileron,
            inputs.elevator,
            inputs.rudder,
            inputs.arm,
            inputs.enable,
            inputs.record
        );
    }
}
