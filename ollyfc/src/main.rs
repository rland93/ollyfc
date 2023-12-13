#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(generic_arg_infer)]

/// Hardware
///
/// TIM4 - task dispatcher
/// TIM5 - task dispatcher
/// TIM11 - task dispatcher
/// TIM1 - timer for flash memory access
/// TIM2 - timer for mpu6050
/// TIM3 - PWM timer
/// TIM10 - 1kHz Misc. Utility timer
///
///
///
use defmt::{debug, info};
use defmt_rtt as _;
use panic_probe as _;

// RTIC
use rtic_monotonics::systick::Systick;
use rtic_sync::{
    channel::{Receiver, Sender},
    make_channel,
};

use stm32f4xx_hal::gpio::Alternate;
use stm32f4xx_hal::{i2c, pac, rcc};

// Sensor
use mpu6050_dmp::sensor::Mpu6050;

// HAL
use stm32f4xx_hal::{
    dma::{StreamsTuple, Transfer},
    gpio::{Output, Pin},
    otg_fs::{UsbBus, UsbBusType, USB},
    pac::{DMA2, I2C1, USART1},
    pac::{TIM10, TIM11, TIM3},
    prelude::*,
    serial::{Config, Rx},
    spi::{Mode, Phase, Polarity, Spi},
    timer::Delay,
    timer::{Channel3, PwmChannel},
    {dma, serial},
};

// USB
use usb_device::{class_prelude::UsbBusAllocator, prelude::*};
use usbd_serial::SerialPort;

use ollyfc_common::{FlightLogData, SensorInput};

/******************************************************************************/

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

/// The program can have 2 modes. Flight mode is the default, and is used when
/// we want the device to act as a flight controller.
///
/// Data mode is used when we want to connect the device to be connected to the
/// PC via USB. In this mode, the device will not run any of the flight
/// controller tasks.
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
enum ProgramMode {
    // Flight computer running, USB not connected
    FlightMode,
    // Flight computer not running, USB connected
    DataMode,
}

mod flight_control;
mod flight_logger;
mod gyro;
mod sbus;
mod usb;
mod w25q;

// Crate
use w25q::W25Q;

type FlashPage = [u8; 256];
const USB_CH_SZ: usize = 4;

/******************************************************************************/

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers=[TIM4, TIM5])]
mod app {

    use super::*;
    #[shared]
    struct Shared {
        // Logging
        gyro: SensorInput,
        // Serial
        sbus_rx_transfer: SbusInTransfer,
        flight_controls: sbus::FlightControls,
        // USB
        usb_dev: UsbDevice<'static, UsbBus<USB>>,
        usb_ser: SerialPort<'static, UsbBus<USB>>,
        // Memory
        mem: W25Q,
    }

    #[local]
    struct Local {
        elevator_channel: PwmChannel<TIM3, 2>,
        // Logging
        log_grp_idx: u8,
        log_buffer: [FlightLogData; LOG_BUFF_SZ],
        mpu6050: Mpu6050<i2c::I2c<I2C1>>,
        mpu6050_int: Pin<'B', 8>,
        // Sbus In
        sbus_rx_buffer: Option<&'static mut [u8; SBUS_BUF_SZ]>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        // Device Peripherals
        let mut dp = cx.device;

        // Clock Setup
        let rcc = dp.RCC.constrain();
        let hse = 25.MHz();
        let sysclk = 64.MHz();
        let clocks = rcc
            .cfgr
            .use_hse(hse)
            .sysclk(sysclk)
            .require_pll48clk()
            .freeze();

        let mut syscfg = dp.SYSCFG.constrain();
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 64_000_000, systick_mono_token);

        // Utility timer
        let mut util_timer: Delay<TIM10, 1000> = dp.TIM10.delay_ms(&clocks);

        // GPIO
        let gpiob = dp.GPIOB.split();
        let gpioa = dp.GPIOA.split();

        // Flash memory
        let mem = mem_init(
            gpioa.pa4.into_push_pull_output(),
            gpioa.pa5.into_alternate(),
            gpioa.pa6.into_alternate(),
            gpioa.pa7.into_alternate(),
            dp.SPI1,
            dp.TIM1,
            &clocks,
        );

        // Gyroscope
        let mut mpu_int = gpiob.pb8.into_pull_down_input();
        let mpu = gyro::mpu_6050_init(
            &mut mpu_int,
            gpiob
                .pb6
                .into_alternate()
                .internal_pull_up(true)
                .set_open_drain(),
            gpiob
                .pb7
                .into_alternate()
                .internal_pull_up(true)
                .set_open_drain(),
            dp.I2C1,
            &mut dp.EXTI,
            &mut syscfg,
            &clocks,
            &mut dp.TIM2.delay_us(&clocks),
        );

        // Channel for log data
        let (log_ch_s, log_ch_r) = make_channel!(FlightLogData, LOGDATA_CHAN_SIZE);

        // Channel for USB data
        let (usb_ch_s, usb_ch_r) = make_channel!(FlashPage, USB_CH_SZ);

        // Sbus In: receiving flight commands
        debug!("sbus in: serial...");
        let mut sbus_in: Rx<USART1, u8> = dp
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
        let dma2 = StreamsTuple::new(dp.DMA2);

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
        let pwm = dp.TIM3.pwm_hz(Channel3::new(gpiob.pb0), 52.Hz(), &clocks);
        let mut elevator_channel = pwm.split();
        elevator_channel.enable();

        // USB -
        info!("USB Button Setup...");
        let usb_btn = gpioa.pa0.into_pull_up_input();
        // Wait for gpio setup
        util_timer.delay_ms(5u32);

        // Check if blue button is pressed and set flight mode
        let mode: ProgramMode;
        if usb_btn.is_high() {
            mode = ProgramMode::FlightMode;
        } else {
            mode = ProgramMode::DataMode;
        }
        info!("Program mode is set: {:?}", mode);

        // USB - Device setup
        info!("USB Device Setup...");
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;

        let usb = USB {
            usb_global: dp.OTG_FS_GLOBAL,
            usb_device: dp.OTG_FS_DEVICE,
            usb_pwrclk: dp.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into(),
            pin_dp: gpioa.pa12.into(),
            hclk: clocks.hclk(),
        };
        let (mut usb_dev, mut usb_ser) =
            crate::usb::usb_setup(usb, unsafe { &mut USB_BUS }, unsafe { &mut EP_MEMORY });

        // USB Mode only! Enter polling loop until host device is connected.
        if mode == ProgramMode::DataMode {
            info!("Data mode initiated. Waiting for USB connection...");
            loop {
                usb_dev.poll(&mut [&mut usb_ser]);
                match usb_dev.state() {
                    UsbDeviceState::Configured => {
                        break;
                    }
                    _ => {
                        continue;
                    }
                }
            }
            info!("USB connection established.");
            usb_send_task::spawn(usb_ch_r).unwrap();
            usb_task::spawn(usb_ch_s).unwrap();
        } else {
            info!("Flight mode initiated. Ignoring USB connection.");
            primary_flight_loop_task::spawn(log_ch_s).unwrap();
            log_write_task::spawn(log_ch_r).unwrap();
        }

        (
            Shared {
                gyro: SensorInput::default(),
                sbus_rx_transfer: sbus_in_transfer,
                flight_controls: sbus::FlightControls::default(),
                usb_dev: usb_dev,
                usb_ser: usb_ser,
                mem: mem,
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
            // wait for interrupt. This will put cortex_m to a low power mode
            // until an interrupt occurs. Since RTIC uses hardware interrupt
            // for scheduling, this basically sleeps until the next task is
            // ready to run.
            rtic::export::wfi();
        }
    }

    /// Primary task for managing USB reading and writing
    #[task(priority = 1, shared=[usb_dev, usb_ser, mem])]
    async fn usb_task(
        mut cx: usb_task::Context,
        mut usb_ch_s: Sender<'static, FlashPage, USB_CH_SZ>,
    ) {
        usb::usb_task_fn(&mut cx, &mut usb_ch_s).await;
    }

    /// Task for sending data to host over USB
    #[task(priority = 1, shared=[usb_dev, usb_ser])]
    async fn usb_send_task(
        mut cx: usb_send_task::Context,
        usb_ch_r: Receiver<'static, FlashPage, USB_CH_SZ>,
    ) {
        usb::usb_send_task_fn(&mut cx, usb_ch_r).await;
    }

    #[task(priority = 3, shared=[flight_controls, gyro], local=[elevator_channel])]
    async fn primary_flight_loop_task(
        cx: primary_flight_loop_task::Context,
        log_ch_s: Sender<'static, FlightLogData, LOGDATA_CHAN_SIZE>,
    ) {
        crate::flight_control::flight_loop(cx, log_ch_s).await;
    }

    #[task(local=[log_grp_idx, log_buffer], priority=1)]
    async fn log_write_task(
        cx: log_write_task::Context,
        log_ch_r: Receiver<'static, FlightLogData, LOGDATA_CHAN_SIZE>,
    ) {
        crate::flight_logger::log_write_task_fn(cx, log_ch_r).await;
    }

    #[task(binds=EXTI9_5, local=[mpu6050, mpu6050_int], shared=[gyro])]
    fn sensor_task(mut cx: sensor_task::Context) {
        crate::gyro::read_sensor_i2c(&mut cx);
    }

    #[task(binds = DMA2_STREAM2, priority=3, shared = [sbus_rx_transfer, flight_controls], local = [sbus_rx_buffer])]
    fn sbus_dma_stream(mut cx: sbus_dma_stream::Context) {
        crate::sbus::read_sbus_stream(&mut cx);
    }
}

/******************************************************************************/

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
