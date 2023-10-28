#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use defmt_rtt as _;
use panic_probe as _;

use defmt::{debug, info};

// Crate
use ollyfc::flightlogger::{FlightLogData, FlightLogger, SBusInput, SensorInput, LOG_SIZE};
use ollyfc::{sbus, w25q};

// RTIC
use rtic::Mutex;
use rtic_monotonic::Monotonic;
use rtic_monotonics::systick::Systick;
use rtic_sync::{
    channel::{Receiver, Sender},
    make_channel,
};

// Sensor
use mpu6050_dmp::{
    accel::Accel, address::Address, config, quaternion::Quaternion, sensor::Mpu6050,
    yaw_pitch_roll::YawPitchRoll,
};

// HAL
use stm32f4xx_hal::{
    dma::{StreamsTuple, Transfer},
    gpio::{Edge, Output, Pin},
    i2c::I2c,
    pac::{DMA2, I2C2, SPI1, TIM10, TIM2, TIM4, TIM9, USART1},
    prelude::*,
    serial::{Config, Rx},
    spi::{Mode, Phase, Polarity, Spi},
    timer::{Delay, DelayUs, FTimer, MonoTimer},
    {dma, serial},
};

// Constants
const LOGDATA_CHAN_SIZE: usize = 128;
const LOG_FREQUENCY_MS: u32 = 500; // 500 Hz
const LOG_BUFF_SZ: usize = w25q::PAGE_SIZE as usize / LOG_SIZE;
const SBUS_BUF_SZ: usize = 25;

type SbusInTransfer = Transfer<
    dma::StreamX<DMA2, 2>,
    4,
    Rx<USART1>,
    dma::PeripheralToMemory,
    &'static mut [u8; SBUS_BUF_SZ],
>;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers=[TIM4, TIM5])]
mod app {

    use ollyfc::flightlogger::{FlightLogData, FlightLogger, SBusInput, SensorInput, LOG_SIZE};
    use ollyfc::{sbus, w25q};

    use defmt::{debug, info};
    use defmt_rtt as _;

    use mpu6050_dmp::accel;

    use rtic_monotonic::Monotonic;
    use rtic_monotonics::systick::Systick;
    use rtic_sync::{
        channel::{Receiver, Sender},
        make_channel,
    };

    use stm32f4xx_hal::serial::Config;
    use stm32f4xx_hal::timer::{FTimer, MonoTimer};
    use stm32f4xx_hal::{
        dma::{StreamsTuple, Transfer},
        gpio::{Alternate, Edge, Output, Pin},
        i2c::I2c,
        pac::{Interrupt, DMA2, I2C2, SPI1, TIM10, TIM2, TIM4, TIM9, USART1},
        prelude::*,
        serial::{Event, Rx, Serial, Tx},
        spi::{Mode, Phase, Polarity, Spi},
        timer::{Delay, DelayMs, DelayUs, SysCounterHz, Timer, Timer2},
        {dma, serial},
    };

    use mpu6050_dmp::{
        accel::Accel, address::Address, config, quaternion::Quaternion, registers, sensor::Mpu6050,
        yaw_pitch_roll::YawPitchRoll,
    };

    const LOGDATA_CHAN_SIZE: usize = 128;
    const LOG_FREQUENCY_MS: u32 = 500; // 500 Hz
    const LOG_BUFF_SZ: usize = w25q::PAGE_SIZE as usize / LOG_SIZE;
    const SBUS_BUF_SZ: usize = 25;

    type SbusInTransfer = Transfer<
        dma::StreamX<DMA2, 2>,
        4,
        Rx<USART1>,
        dma::PeripheralToMemory,
        &'static mut [u8; SBUS_BUF_SZ],
    >;

    use super::*;
    #[shared]
    struct Shared {
        // Logging
        timer10: Delay<TIM10, 1000>,
        timer2: MonoTimer<TIM2, 1_000>,
        timer9: DelayUs<TIM9>,
        sensor: SensorInput,
        sbus: SBusInput,
        // Serial
        sbus_rx_transfer: SbusInTransfer,
    }

    #[local]
    struct Local {
        // Logging
        log_grp_idx: u8,
        flight_logger: FlightLogger<w25q::W25Q>,
        log_delay: Delay<TIM4, 1000>,
        log_buffer: [FlightLogData; LOG_BUFF_SZ],
        mpu6050: Mpu6050<I2c<I2C2>>,
        mpu6050_int: Pin<'B', 8>,
        // Sbus In
        sbus_rx_buffer: Option<&'static mut [u8; SBUS_BUF_SZ]>,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        // Setup clocks
        let rcc = cx.device.RCC.constrain();
        let mut syscfg = cx.device.SYSCFG.constrain();
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 48_000_000, systick_mono_token);
        let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();
        info!("Booted. Initializing...");

        info!("Peripherals...");
        debug!("gpio...");
        // Peripherals
        let gpiob = cx.device.GPIOB.split();
        let gpioa = cx.device.GPIOA.split();

        debug!("spi1, mem...");
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

        debug!("logging...");
        let log_delay: Delay<TIM4, 1000> = cx.device.TIM4.delay(&clocks);

        let (log_ch_s, log_ch_r) = make_channel!(FlightLogData, LOGDATA_CHAN_SIZE);

        debug!("mpu6050...");
        // Configure pin for interrupt on data ready
        let mut mpu6050_int = gpiob.pb8.into_pull_down_input();
        mpu6050_int.make_interrupt_source(&mut syscfg);
        mpu6050_int.trigger_on_edge(&mut cx.device.EXTI, Edge::Falling);
        mpu6050_int.enable_interrupt(&mut cx.device.EXTI);

        let mut delay_tim5 = cx.device.TIM5.delay_us(&clocks);
        let i2c2_sda = gpiob
            .pb9
            .into_alternate()
            .internal_pull_up(true)
            .set_open_drain();
        let i2c2_scl = gpiob
            .pb10
            .into_alternate()
            .internal_pull_up(true)
            .set_open_drain();
        let i2c2 = cx.device.I2C2.i2c((i2c2_scl, i2c2_sda), 400.kHz(), &clocks);
        let mut mpu6050: Mpu6050<I2c<I2C2>> = Mpu6050::new(i2c2, Address::default()).unwrap();
        mpu6050.initialize_dmp(&mut delay_tim5).unwrap();
        mpu6050.enable_fifo().unwrap();
        mpu6050
            .set_digital_lowpass_filter(config::DigitalLowPassFilter::Filter2)
            .unwrap();
        // interrupt enable on mpu6050
        mpu6050.disable_interrupts().unwrap();
        mpu6050.interrupt_fifo_oflow_en().unwrap();

        let timer2: MonoTimer<TIM2, 1_000> = FTimer::new(cx.device.TIM2, &clocks).monotonic();

        // Sbus In: receiving flight commands
        debug!("sbus in: serial...");
        let mut sbus_in: Rx<USART1, u8> = cx
            .device
            .USART1
            .rx(
                gpioa.pa10.into_alternate(),
                Config::default()
                    .baudrate(100_000.bps())
                    .dma(serial::config::DmaConfig::Rx),
                &clocks,
            )
            .unwrap();
        sbus_in.listen_idle();

        // Sbus In:  DMA stream
        info!("sbus in: dma...");
        let dma2 = StreamsTuple::new(cx.device.DMA2);

        let sbus_in_buffer_A = cortex_m::singleton!(: [u8; 25] = [0; 25]).unwrap();
        let sbus_in_buffer_B = cortex_m::singleton!(: [u8; 25] = [0; 25]).unwrap();

        let mut sbus_in_transfer: SbusInTransfer = Transfer::init_peripheral_to_memory(
            dma2.2,
            sbus_in,
            sbus_in_buffer_A,
            None,
            dma::config::DmaConfig::default()
                .memory_increment(true)
                .fifo_enable(true)
                .fifo_error_interrupt(true)
                .transfer_complete_interrupt(true),
        );
        sbus_in_transfer.start(|_rx| {});

        // Misc
        let timer9: Delay<TIM9, 1_000_000> = cx.device.TIM9.delay_us(&clocks);

        // log_collect_task::spawn(log_ch_s.clone()).unwrap();
        // log_write_task::spawn(log_ch_r).unwrap();

        (
            Shared {
                timer10: cx.device.TIM10.delay(&clocks),
                timer2: timer2,
                timer9: timer9,
                sensor: SensorInput::default(),
                sbus: SBusInput::default(),
                sbus_rx_transfer: sbus_in_transfer,
            },
            Local {
                flight_logger: FlightLogger::new(mem),
                log_delay,
                log_buffer: [FlightLogData::default(); LOG_BUFF_SZ],
                log_grp_idx: 0,
                mpu6050: mpu6050,
                mpu6050_int,
                sbus_rx_buffer: Some(sbus_in_buffer_B),
            },
        )
    }

    #[idle]
    fn idle(cx: idle::Context) -> ! {
        loop {
            // nop
            cortex_m::asm::nop();
        }
    }

    #[task(priority = 2, shared=[timer9])]
    async fn primary_flight_loop_task(mut cx: primary_flight_loop_task::Context) {
        loop {
            cx.shared.timer9.lock(|timer| {
                timer.delay_us(1_000u32);
            });
            debug!("Primary flight loop task");
        }
    }

    #[task(local=[flight_logger, log_delay], shared=[sensor, sbus, timer2], priority=1)]
    async fn log_collect_task(
        mut cx: log_collect_task::Context,
        mut log_ch_s: Sender<'static, FlightLogData, LOGDATA_CHAN_SIZE>,
    ) {
        cx.local.flight_logger.init(w25q::BLOCK_32K_SIZE as u32);
        // get current time and sensor values
        let now = cx
            .shared
            .timer2
            .lock(|timer: &mut MonoTimer<TIM2, 1_000>| timer.now());
        let sensor = cx.shared.sensor.lock(|sensor| sensor.clone());
        let sbus = cx.shared.sbus.lock(|sbus| sbus.clone());
        let log_data = FlightLogData {
            timestamp: now.ticks(),
            sbus_input: sbus,
            sensor_input: sensor,
        };
        debug!(
            "Sent a log data to queue. Sensor: {:?} {:?} {:?}",
            sensor.roll, sensor.pitch, sensor.yaw
        );
        // send log onto the queue
        log_ch_s.send(log_data).await.unwrap();
    }

    #[task(local=[log_grp_idx, log_buffer], shared=[timer10], priority=1)]
    async fn log_write_task(
        mut cx: log_write_task::Context,
        mut log_ch_r: Receiver<'static, FlightLogData, LOGDATA_CHAN_SIZE>,
    ) {
        loop {
            // wait to get a new log
            let log_data = match log_ch_r.recv().await {
                Ok(data) => data,
                Err(e) => match e {
                    rtic_sync::channel::ReceiveError::NoSender => {
                        debug!("No sender.");
                        continue;
                    }
                    rtic_sync::channel::ReceiveError::Empty => {
                        debug!("Empty queue.");
                        continue;
                    }
                },
            };

            let buf_idx = (*cx.local.log_grp_idx).clone() as usize;
            // store log data into the buffer
            cx.local.log_buffer[buf_idx] = log_data;

            if *cx.local.log_grp_idx == LOG_BUFF_SZ as u8 - 1 {
                // TODO: write to flash. For now we'll implement a small delay to
                // simulate the flash write time
                cx.shared.timer10.lock(|t| t.delay_ms(10u32));
                debug!("Writing page flash...");

                *cx.local.log_grp_idx = 0u8;
            } else {
                // increment the index
                *cx.local.log_grp_idx += 1u8;
            }
        }
    }

    #[task(binds=EXTI9_5, local=[mpu6050, mpu6050_int], shared=[sensor])]
    fn sensor_task(mut cx: sensor_task::Context) {
        read_sensor_i2c(&mut cx);
    }

    // TASK to read SBUS in over serial. Uses DMA.
    #[task(binds = DMA2_STREAM2, priority=3, shared = [sbus_rx_transfer], local = [sbus_rx_buffer])]
    fn sbus_dma_stream(mut cx: sbus_dma_stream::Context) {
        read_sbus_stream(&mut cx);
    }
}

fn read_sensor_i2c(cx: &mut app::sensor_task::Context) {
    // Grab newest from buffer
    let mut buf = [0; 28];
    let buf = cx.local.mpu6050.read_fifo(&mut buf).unwrap();
    // Gyro
    let quat = Quaternion::from_bytes(&buf[..16]).unwrap();
    let ypr = YawPitchRoll::from(quat);
    // Acceleration
    let mut accelbytes: [u8; 6] = [0; 6];
    accelbytes.copy_from_slice(&buf[16..22]);

    let accel = Accel::from_bytes(accelbytes);
    let accel_scaled = accel.scaled(mpu6050_dmp::accel::AccelFullScale::G4);
    // store sensor input
    cx.shared.sensor.lock(|s| {
        *s = SensorInput {
            pitch: ypr.pitch,
            yaw: ypr.yaw,
            roll: ypr.roll,
            accel_x: accel_scaled.x(),
            accel_y: accel_scaled.y(),
            accel_z: accel_scaled.z(),
        };
    });
    cx.local.mpu6050_int.clear_interrupt_pending_bit();
}

fn read_sbus_stream(cx: &mut app::sbus_dma_stream::Context) {
    cx.shared
        .sbus_rx_transfer
        .lock(|transfer: &mut SbusInTransfer| {
            if transfer.is_idle() {
                // Allocate a new buffer
                let new_buf = cx.local.sbus_rx_buffer.take().unwrap();
                // Replace the new buffer with contents from the stream
                let (buffer, _current) = transfer.next_transfer(new_buf).unwrap();

                let control: sbus::SbusChannels = sbus::SbusData::new(*buffer).parse();
                let control: sbus::FlightControls = control.get_input();
                // TODO: send FlightControls to shared object...
                debug!("Sbus in: {:?}", control);
                // Free buffer
                *cx.local.sbus_rx_buffer = Some(buffer);
            }
        });
}
