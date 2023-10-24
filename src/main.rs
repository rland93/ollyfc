#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_variables)]

use panic_probe as _;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
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

    use stm32f4xx_hal::timer::{FTimer, MonoTimer};
    use stm32f4xx_hal::{
        gpio::{Edge, Output, Pin},
        i2c::I2c,
        pac::{Interrupt, I2C2, SPI2, TIM10, TIM2, TIM4},
        prelude::*,
        spi::{Mode, Phase, Polarity, Spi},
        timer::{Delay, DelayMs, DelayUs, SysCounterHz, Timer, Timer2},
    };

    use mpu6050_dmp::{
        accel::Accel, address::Address, config, quaternion::Quaternion, registers, sensor::Mpu6050,
        yaw_pitch_roll::YawPitchRoll,
    };

    const LOGDATA_CHAN_SIZE: usize = 128;
    const LOG_FREQUENCY_MS: u32 = 2; // 500 Hz

    #[shared]
    struct Shared {
        // Logging
        timer10: Delay<TIM10, 1000>,
        log_timer: MonoTimer<TIM2, 1_000>,
        sensor: SensorInput,
        sbus: SBusInput,
    }

    #[local]
    struct Local {
        // Logging
        log_grp_idx: u8,
        flight_logger: FlightLogger<w25q::W25Q>,
        log_delay: Delay<TIM4, 1000>,
        log_buffer: [FlightLogData; LOGS_IN_PAGE],
        mpu6050: Mpu6050<I2c<I2C2>>,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        // Setup clocks
        let rcc = cx.device.RCC.constrain();
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
        let log_timer: MonoTimer<TIM2, 1000> = FTimer::new(cx.device.TIM2, &clocks).monotonic();
        let log_delay: Delay<TIM4, 1000> = cx.device.TIM4.delay(&clocks);

        let (log_ch_s, log_ch_r) = make_channel!(FlightLogData, LOGDATA_CHAN_SIZE);
        log_collect_task::spawn(log_ch_s.clone()).unwrap();
        log_write_task::spawn(log_ch_r).unwrap();

        debug!("mpu6050...");
        // Configure pin for interrupt on data ready
        let mut mpu6050_int = gpiob.pb8.into_pull_up_input();
        mpu6050_int.enable_interrupt(&mut cx.device.EXTI);
        mpu6050_int.trigger_on_edge(&mut cx.device.EXTI, Edge::Rising);
        rtic::pend(mpu6050_int.interrupt());

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

        (
            Shared {
                timer10: cx.device.TIM10.delay(&clocks),
                log_timer,
                sensor: SensorInput::default(),
                sbus: SBusInput::default(),
            },
            Local {
                flight_logger: FlightLogger::new(mem),
                log_delay,
                log_buffer: [FlightLogData::default(); LOGS_IN_PAGE],
                log_grp_idx: 0,
                mpu6050: mpu6050,
            },
        )
    }

    #[task(local=[flight_logger, log_delay], shared=[log_timer, sensor, sbus])]
    async fn log_collect_task(
        mut cx: log_collect_task::Context,
        mut log_ch_s: Sender<'static, FlightLogData, LOGDATA_CHAN_SIZE>,
    ) {
        cx.local.flight_logger.init(w25q::BLOCK_32K_SIZE as u32);
        loop {
            // get current time and sensor values
            let now = cx
                .shared
                .log_timer
                .lock(|timer: &mut MonoTimer<TIM2, 1000>| timer.now());
            let sensor = cx.shared.sensor.lock(|sensor| sensor.clone());
            let sbus = cx.shared.sbus.lock(|sbus| sbus.clone());
            let log_data = FlightLogData {
                timestamp: now.ticks(),
                sbus_input: sbus,
                sensor_input: sensor,
            };

            // send log onto the queue
            log_ch_s.send(log_data).await.unwrap();

            // delay the task
            cx.local.log_delay.delay_ms(LOG_FREQUENCY_MS);
        }
    }

    #[task(local=[log_grp_idx, log_buffer], shared=[timer10])]
    async fn log_write_task(
        mut cx: log_write_task::Context,
        mut log_ch_r: Receiver<'static, FlightLogData, LOGDATA_CHAN_SIZE>,
    ) {
        loop {
            // wait to get a new log
            let log_data = log_ch_r.recv().await.unwrap();

            let buf_idx = (*cx.local.log_grp_idx).clone() as usize;
            // store log data into the buffer
            cx.local.log_buffer[buf_idx] = log_data;

            if *cx.local.log_grp_idx == 7u8 {
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

    #[task(binds=EXTI0, local=[mpu6050], shared=[sensor])]
    fn sensor_task(mut cx: sensor_task::Context) {
        loop {
            let len = cx.local.mpu6050.get_fifo_count().unwrap();
            if len >= 28 {
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
                let accel_scaled = accel.scaled(accel::AccelFullScale::G4);
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
                })
            }
        }
    }
}
