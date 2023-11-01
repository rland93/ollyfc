#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(generic_arg_infer)]

use defmt_rtt as _;
use ollyfc::w25q::W25Q;
use panic_probe as _;

use defmt::{debug, info};

// Crate
use ollyfc::flight_control::ControlPolicy;
use ollyfc::flight_logger::{FlightLogData, SBusInput, SensorInput};
use ollyfc::{sbus, w25q};

// RTIC
use rtic::Mutex;
use rtic_monotonics::systick::Systick;
use rtic_monotonics::Monotonic;
use rtic_sync::{
    channel::{Receiver, Sender},
    make_channel,
};

use stm32f4xx_hal::gpio::{Alternate, Edge, Input, OpenDrain};
use stm32f4xx_hal::{i2c, pac, rcc, syscfg};

// Sensor
use mpu6050_dmp::{
    accel::Accel, address::Address, config, quaternion::Quaternion, sensor::Mpu6050,
    yaw_pitch_roll::YawPitchRoll,
};

// HAL
use stm32f4xx_hal::{
    dma::{StreamsTuple, Transfer},
    gpio::{Output, Pin},
    pac::TIM3,
    pac::{DMA2, I2C2, USART1},
    prelude::*,
    serial::{Config, Rx},
    spi::{Mode, Phase, Polarity, Spi},
    timer::Delay,
    timer::{Channel3, PwmChannel},
    {dma, serial},
};

// Constants
const LOGDATA_CHAN_SIZE: usize = 128;
const SBUS_BUF_SZ: usize = 25;
const LOG_BUFF_SZ: usize = 4;

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
        gyro: SensorInput,
        // Serial
        sbus_rx_transfer: SbusInTransfer,
        flight_controls: sbus::FlightControls,
    }

    #[local]
    struct Local {
        elevator_channel: PwmChannel<TIM3, 2>,
        // Logging
        log_grp_idx: u8,
        log_buffer: [FlightLogData; LOG_BUFF_SZ],
        mpu6050: Mpu6050<i2c::I2c<I2C2>>,
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
        Systick::start(cx.core.SYST, 64_000_000, systick_mono_token);
        let clocks = rcc.cfgr.sysclk(64.MHz()).freeze();

        let gpiob = cx.device.GPIOB.split();
        let gpioa = cx.device.GPIOA.split();

        // Flash memory
        let mem = mem_init(
            gpioa.pa4.into_push_pull_output(),
            gpioa.pa5.into_alternate(),
            gpioa.pa6.into_alternate(),
            gpioa.pa7.into_alternate(),
            cx.device.SPI1,
            cx.device.TIM1,
            &clocks,
        );

        // Gyroscope
        let mut mpu_int = gpiob.pb8.into_pull_down_input();
        let mpu = mpu_6050_init(
            &mut mpu_int,
            gpiob
                .pb10
                .into_alternate()
                .internal_pull_up(true)
                .set_open_drain(),
            gpiob
                .pb11
                .into_alternate()
                .internal_pull_up(true)
                .set_open_drain(),
            cx.device.I2C2,
            &mut cx.device.EXTI,
            &mut syscfg,
            &clocks,
            &mut cx.device.TIM2.delay_us(&clocks),
        );

        // Channel for log data
        let (log_ch_s, log_ch_r) = make_channel!(FlightLogData, LOGDATA_CHAN_SIZE);

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

        let sbus_in_buffer_A =
            cortex_m::singleton!(: [u8; SBUS_BUF_SZ] = [0; SBUS_BUF_SZ]).unwrap();
        let sbus_in_buffer_B =
            cortex_m::singleton!(: [u8; SBUS_BUF_SZ] = [0; SBUS_BUF_SZ]).unwrap();

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

        // Set up PWM
        info!("PWM...");
        let pwm = cx
            .device
            .TIM3
            .pwm_hz(Channel3::new(gpiob.pb0), 52.Hz(), &clocks);
        let mut elevator_channel = pwm.split();
        elevator_channel.enable();

        info!("Starting tasks...");

        primary_flight_loop_task::spawn(log_ch_s).unwrap();
        log_write_task::spawn(log_ch_r).unwrap();

        (
            Shared {
                gyro: SensorInput::default(),
                sbus_rx_transfer: sbus_in_transfer,
                flight_controls: sbus::FlightControls::default(),
            },
            Local {
                elevator_channel: elevator_channel,
                log_buffer: [FlightLogData::default(); LOG_BUFF_SZ],
                log_grp_idx: 0,
                mpu6050: mpu,
                mpu6050_int: mpu_int,
                sbus_rx_buffer: Some(sbus_in_buffer_B),
            },
        )
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            // nop
            rtic::export::wfi();
        }
    }

    #[task(priority = 3, shared=[flight_controls, gyro], local=[elevator_channel])]
    async fn primary_flight_loop_task(
        cx: primary_flight_loop_task::Context,
        log_ch_s: Sender<'static, FlightLogData, LOGDATA_CHAN_SIZE>,
    ) {
        flight_loop(cx, log_ch_s).await;
    }

    #[task(local=[log_grp_idx, log_buffer], priority=1)]
    async fn log_write_task(
        cx: log_write_task::Context,
        log_ch_r: Receiver<'static, FlightLogData, LOGDATA_CHAN_SIZE>,
    ) {
        log_write(cx, log_ch_r).await;
    }

    #[task(binds=EXTI9_5, local=[mpu6050, mpu6050_int], shared=[gyro])]
    fn sensor_task(mut cx: sensor_task::Context) {
        read_sensor_i2c(&mut cx);
    }

    #[task(binds = DMA2_STREAM2, priority=3, shared = [sbus_rx_transfer, flight_controls], local = [sbus_rx_buffer])]
    fn sbus_dma_stream(mut cx: sbus_dma_stream::Context) {
        read_sbus_stream(&mut cx);
    }
}

fn mpu_6050_init(
    interrupt: &mut Pin<'B', 8, Input>,
    scl: Pin<'B', 10, Alternate<4, OpenDrain>>,
    sda: Pin<'B', 11, Alternate<4, OpenDrain>>,
    i2c2: pac::I2C2,
    exti: &mut pac::EXTI,
    syscfg: &mut syscfg::SysCfg,
    clocks: &rcc::Clocks,
    delay: &mut Delay<pac::TIM2, 1000000>,
) -> Mpu6050<i2c::I2c<pac::I2C2>> {
    // Configure pin for interrupt on data ready
    interrupt.make_interrupt_source(syscfg);
    interrupt.trigger_on_edge(exti, Edge::Falling);
    interrupt.enable_interrupt(exti);
    let i2c_dev = i2c2.i2c((scl, sda), 400.kHz(), &clocks);

    let mut mpu6050: Mpu6050<i2c::I2c<pac::I2C2>> =
        Mpu6050::new(i2c_dev, Address::default()).unwrap();

    // digitql motion processor
    mpu6050.initialize_dmp(delay).unwrap();
    mpu6050.enable_fifo().unwrap();
    mpu6050
        .set_digital_lowpass_filter(config::DigitalLowPassFilter::Filter2)
        .unwrap();
    mpu6050.disable_interrupts().unwrap();
    mpu6050.interrupt_fifo_oflow_en().unwrap();
    mpu6050
}

fn mem_init(
    cs: Pin<'A', 4, Output>,
    sck: Pin<'A', 5, Alternate<5>>,
    miso: Pin<'A', 6, Alternate<5>>,
    mosi: Pin<'A', 7, Alternate<5>>,
    spi: pac::SPI1,
    tim: pac::TIM1,
    clocks: &rcc::Clocks,
) -> W25Q {
    let mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };
    let spi = Spi::new(spi, (sck, miso, mosi), mode, 100.kHz(), clocks);
    w25q::W25Q::new(spi, cs, tim.delay_us(&clocks))
}

async fn flight_loop(
    mut cx: app::primary_flight_loop_task::Context<'_>,
    mut log_ch_s: Sender<'static, FlightLogData, LOGDATA_CHAN_SIZE>,
) {
    loop {
        let now = Systick::now();
        // Flight Controls
        let controls = cx
            .shared
            .flight_controls
            .lock(|fc: &mut sbus::FlightControls| fc.clone());
        let arm_mode = switch_mode(controls.arm);
        let enable_mode = switch_mode(controls.enable);
        let record_mode = switch_mode(controls.record);

        // Sensor
        let gyro = cx.shared.gyro.lock(|g: &mut SensorInput| g.clone());

        // control policies
        let ele = elevator_ctl(controls.elevator, arm_mode);
        let ail = aileron_ctl(controls.aileron);
        let rud = rudder_ctl(controls.rudder);
        let thr = throttle_ctl(controls.throttle);

        // Send data to logger
        let log_data = FlightLogData {
            timestamp: 0,
            sbus_input: SBusInput {
                throttle: controls.throttle,
                aileron: controls.aileron,
                elevator: controls.elevator,
                rudder: controls.rudder,
                arm: arm_mode as u16,
                enable: enable_mode as u16,
                record: record_mode as u16,
            },
            sensor_input: SensorInput {
                pitch: gyro.pitch,
                yaw: gyro.yaw,
                roll: gyro.roll,
                accel_x: gyro.accel_x,
                accel_y: gyro.accel_y,
                accel_z: gyro.accel_z,
            },
            control_policy: ControlPolicy {
                elevator: ele,
                aileron: ail,
                rudder: rud,
                throttle: thr,
            },
        };
        if !log_ch_s.is_full() {
            if let Err(_e) = log_ch_s.send(log_data).await {
                debug!("Error sending log data: No reciever.");
            }
        } else {
            debug!("Log channel full.");
        }

        let duty = scale_servo(ele, cx.local.elevator_channel.get_max_duty());
        cx.local.elevator_channel.set_duty(duty);

        Systick::delay_until(now + 20.millis()).await;
    }
}

async fn log_write(
    mut _cx: app::log_write_task::Context<'_>,
    mut log_ch_r: Receiver<'static, FlightLogData, LOGDATA_CHAN_SIZE>,
) {
    loop {
        let _data = match log_ch_r.recv().await {
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
        debug!("TODO: Write to flash");
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
    cx.shared.gyro.lock(|s| {
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
                cx.shared.flight_controls.lock(|fc| {
                    *fc = control;
                });

                // Free buffer
                *cx.local.sbus_rx_buffer = Some(buffer);
            }
        });
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum SwitchMode {
    Low,
    Neutral,
    High,
}

fn switch_mode(switch_input: u16) -> SwitchMode {
    if switch_input < 500 {
        SwitchMode::Low
    } else if switch_input > 1500 {
        SwitchMode::High
    } else {
        SwitchMode::Neutral
    }
}

fn elevator_ctl(ele: u16, en: SwitchMode) -> u16 {
    if en == SwitchMode::High {
        ele + 200
    } else {
        ele
    }
}

fn aileron_ctl(ail: u16) -> u16 {
    ail
}

fn rudder_ctl(rud: u16) -> u16 {
    rud
}

fn throttle_ctl(thr: u16) -> u16 {
    thr
}

fn scale_servo(elevator: u16, duty_max: u16) -> u16 {
    let min_pulse_width = 900; // 900 μs
    let max_pulse_width = 2100; // 2100 μs
    let pulse_width_range = max_pulse_width - min_pulse_width;
    let servo_pulse_width =
        min_pulse_width + (elevator as f32 / 2048.0 * pulse_width_range as f32) as u16;
    let pwm_duty = (servo_pulse_width as f32 / 20000.0 * duty_max as f32) as u16;
    return pwm_duty;
}
