#![deny(unsafe_code)]
#![no_main]
#![no_std]

/// Configure your radio as follows:
///
/// - Channel 1: Throttle
/// - Channel 2: Aileron
/// - Channel 3: Elevator
/// - Channel 4: Rudder
/// - Channel 5: Arm Switch
/// - Channel 6: Enable Switch
/// - Channel 7: Record Switch
///
use defmt_rtt as _;
use panic_probe as _;

use rtic_monotonics::systick_monotonic;
systick_monotonic!(Mono, 1000);

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [TIM2])]
mod app {
    use super::*;
    use hal::{dma, pac, prelude::*, serial};

    use ollyfc::io::sbus::SbusData;
    use stm32f4xx_hal as hal;
    const BUFFER_SIZE: usize = 25;
    type RxTransfer = dma::Transfer<
        dma::Stream2<pac::DMA2>,
        4,
        serial::Rx<pac::USART1>,
        dma::PeripheralToMemory,
        &'static mut [u8; BUFFER_SIZE],
    >;

    #[shared]
    struct Shared {
        sbus_transfer: RxTransfer,
    }

    #[local]
    struct Local {
        sbus_buffer: Option<&'static mut [u8; BUFFER_SIZE]>,
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
        Mono::start(cx.core.SYST, sysclk.to_Hz());

        let gpiob = dp.GPIOB.split();

        // initialize uart
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

        // dma
        let dma2 = dma::StreamsTuple::new(dp.DMA2);
        let buffer1 = cortex_m::singleton!(: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE]).unwrap();
        let buffer2 = cortex_m::singleton!(: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE]).unwrap();

        // Initialize and start DMA stream with the first buffer
        let mut rx_transfer = dma::Transfer::init_peripheral_to_memory(
            dma2.2,
            rx,
            buffer1,
            None,
            dma::config::DmaConfig::default()
                .memory_increment(true)
                .fifo_enable(true)
                .fifo_error_interrupt(true)
                .transfer_complete_interrupt(true),
        );
        rx_transfer.start(|_rx| {});

        (
            Shared {
                sbus_transfer: rx_transfer,
            },
            Local {
                sbus_buffer: Some(buffer2),
            },
        )
    }

    #[task(binds = DMA2_STREAM2, shared = [sbus_transfer], local = [sbus_buffer])]
    fn sbus_data(mut cx: sbus_data::Context) {
        cx.shared.sbus_transfer.lock(|xfer: &mut RxTransfer| {
            if xfer.is_idle() {
                let new_buf = cx.local.sbus_buffer.take().unwrap();
                let (buf, _current) = xfer.next_transfer(new_buf).unwrap();
                let chan_data = SbusData::new(*buf).parse();
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
    async fn chan_task(_cx: chan_task::Context, chan_data: ollyfc::io::sbus::SbusChannels) {
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
