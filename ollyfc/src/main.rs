#![no_std]
#![no_main]

// configuration
use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

// system
use rtic_monotonics::systick::Systick;
use rtic_monotonics::Monotonic;

// drivers
use bmi088::interface::SpiInterface;
use bmi088::Bmi088;
use bmp388::{Blocking, BMP388};
use embedded_hdc1080_rs::Hdc1080;
use usb_device::device;

// crate
use ollyfc::sbus;
use ollyfc::sensor::imu;
use ollyfc::sensor::pressure::BMP388_CFG;
use ollyfc::usb::protocol::FcDevice;

// HAL
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::{dma, gpio, i2c, otg_fs, pac, serial, spi, timer};

use embedded_hal_bus as bus;

// device driver types
type GyroDev = bus::spi::AtomicDevice<
    'static,
    spi::Spi<pac::SPI3>,
    gpio::Pin<'B', 5, gpio::Output>,
    bus::spi::NoDelay,
>;
type AccelDev = bus::spi::AtomicDevice<
    'static,
    spi::Spi<pac::SPI3>,
    gpio::Pin<'D', 2, gpio::Output>,
    bus::spi::NoDelay,
>;
type Spi3Bus = bus::util::AtomicCell<spi::Spi<pac::SPI3>>;

type I2C3Bus = bus::util::AtomicCell<i2c::I2c<pac::I2C3>>;
type PressureSensorDev<'a> = BMP388<bus::i2c::AtomicDevice<'a, i2c::I2c<pac::I2C3>>, Blocking>;
type TempSensorDev<'b> =
    Hdc1080<bus::i2c::AtomicDevice<'b, i2c::I2c<pac::I2C3>>, timer::Delay<pac::TIM4, 1000>>;
type MagnetometerDev<'c> = lis3mdl::Lis3mdl<bus::i2c::AtomicDevice<'c, i2c::I2c<pac::I2C3>>>;
const BUZZER_CH2: u8 = timer::C2;
type BuzzerDev = timer::PwmHz<pac::TIM1, timer::ChannelBuilder<pac::TIM1, BUZZER_CH2, true>>;

const SBUS_BUFFER_SIZE: usize = 25;
type SbusRxTransfer = dma::Transfer<
    dma::Stream2<pac::DMA2>,
    4,
    serial::Rx<pac::USART1>,
    dma::PeripheralToMemory,
    &'static mut [u8; SBUS_BUFFER_SIZE],
>;

const SENSOR_LOOP_READ_DELAY: u32 = 100;
#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [USART1, I2C2_ER, I2C2_EV, SPI4])]
mod app {

    use super::*;

    #[shared]
    struct Shared {
        dev_fc_usb: FcDevice<'static, otg_fs::UsbBus<otg_fs::USB>>,
        dev_accel: Bmi088<SpiInterface<AccelDev>>,
        dev_gyro: Bmi088<SpiInterface<GyroDev>>,
        dev_temp_sensor: TempSensorDev<'static>,
        sbus_transfer: SbusRxTransfer,
    }

    #[local]
    struct Local {
        pin_imu_drdy: gpio::Pin<'C', 0, gpio::Input>,
        dev_pressure_sensor: PressureSensorDev<'static>,
        dev_magnetometer: MagnetometerDev<'static>,
        tim_10: timer::Delay<pac::TIM10, 1000>,
        dev_buzzer: BuzzerDev,
        sbus_buffer: Option<&'static mut [u8; SBUS_BUFFER_SIZE]>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<otg_fs::UsbBusType>> = None;
        static mut SPI3BUS: Option<Spi3Bus> = None;
        static mut I2C3BUS: Option<I2C3Bus> = None;

        let mut dp = cx.device;
        let rcc = dp.RCC.constrain();
        let hse = 16.MHz();
        let sysclk = 64.MHz();
        let clocks = rcc
            .cfgr
            .use_hse(hse)
            .sysclk(sysclk)
            .require_pll48clk()
            .freeze();

        let mut syscfg = dp.SYSCFG.constrain();
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, sysclk.to_Hz(), systick_mono_token);

        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();
        let gpiod = dp.GPIOD.split();

        // USB
        let usb_pin_dm = gpioa.pa11.into_alternate();
        let usb_pin_dp = gpioa.pa12.into_alternate();
        let usb_dev = otg_fs::USB {
            usb_global: dp.OTG_FS_GLOBAL,
            usb_device: dp.OTG_FS_DEVICE,
            usb_pwrclk: dp.OTG_FS_PWRCLK,
            pin_dm: gpio::alt::otg_fs::Dm::PA11(usb_pin_dm),
            pin_dp: gpio::alt::otg_fs::Dp::PA12(usb_pin_dp),
            hclk: clocks.hclk(),
        };
        #[allow(static_mut_refs)]
        let dev_fc_usb = FcDevice::new(usb_dev, unsafe { &mut USB_BUS }, unsafe { &mut EP_MEMORY });
        // USB

        // I2C3 Bus
        let i2c3_sda = gpioc.pc9.into_alternate_open_drain();
        let i2c3_scl = gpioa.pa8.into_alternate_open_drain();
        let dev_i2c3 = dp.I2C3.i2c((i2c3_scl, i2c3_sda), 400.kHz(), &clocks);

        let i2c3_cell = unsafe {
            I2C3BUS = Some(bus::util::AtomicCell::new(dev_i2c3));
            I2C3BUS.as_ref().unwrap()
        };
        // I2C3 Bus

        // Pressure Sensor
        let mut tim4_delay = dp.TIM4.delay_ms(&clocks);
        let cfg = BMP388_CFG;
        let dev_pressure_sensor: PressureSensorDev = cfg
            .setup_blocking(bus::i2c::AtomicDevice::new(i2c3_cell), &mut tim4_delay)
            .unwrap();
        // Pressure Sensor

        // Temperature + Humidity Sensor
        let dev_temp_sensor =
            Hdc1080::new(bus::i2c::AtomicDevice::new(&i2c3_cell), tim4_delay).unwrap();
        // Temperature + Humidity Sensor

        // Magnetometer
        let dev_magnetometer = lis3mdl::Lis3mdl::new(
            bus::i2c::AtomicDevice::new(i2c3_cell),
            lis3mdl::Address::Addr1C,
        )
        .unwrap();

        // Magnetometer

        // IMU
        let spi: spi::Spi3 = spi::Spi3::new(
            dp.SPI3,
            (
                gpioc.pc10.into_alternate(),
                gpioc.pc11.into_alternate(),
                gpioc.pc12.into_alternate(),
            ),
            spi::Mode {
                polarity: spi::Polarity::IdleLow,
                phase: spi::Phase::CaptureOnFirstTransition,
            },
            8.MHz(),
            &clocks,
        );
        let mut acc_cs = gpiod.pd2.into_push_pull_output();
        let mut gyr_cs = gpiob.pb5.into_push_pull_output();
        acc_cs.set_high();
        gyr_cs.set_high();
        // TODO clean this up, like i2c3 above.
        let (mut dev_accel, mut dev_gyro) = unsafe {
            SPI3BUS = Some(bus::util::AtomicCell::new(spi));
            let bus = SPI3BUS.as_ref().unwrap();
            let acc = bus::spi::AtomicDevice::new_no_delay(&bus, acc_cs).unwrap();
            let gyro = bus::spi::AtomicDevice::new_no_delay(&bus, gyr_cs).unwrap();
            (
                bmi088::Bmi088::new_with_spi(acc),
                bmi088::Bmi088::new_with_spi(gyro),
            )
        };
        dev_accel.acc_chipid().unwrap();
        dev_gyro.gyro_chipid().unwrap();
        let mut pin_imu_drdy = gpioc.pc0.into_pull_up_input();
        pin_imu_drdy.make_interrupt_source(&mut syscfg);
        pin_imu_drdy.trigger_on_edge(&mut dp.EXTI, gpio::Edge::Falling);
        pin_imu_drdy.enable_interrupt(&mut dp.EXTI);
        unsafe {
            pac::NVIC::unmask(pin_imu_drdy.interrupt());
        }
        // IMU

        // Buzzer
        let tim_10 = dp.TIM10.delay_ms(&clocks);
        let channels = timer::Channel2::new(gpioa.pa9).with_complementary(gpiob.pb0);
        let mut dev_buzzer: BuzzerDev = dp.TIM1.pwm_hz(channels, 1500.Hz(), &clocks);
        let max_duty: u16 = dev_buzzer.get_max_duty();
        dev_buzzer.set_polarity(timer::Channel::C2, timer::Polarity::ActiveHigh);
        dev_buzzer.set_complementary_polarity(timer::Channel::C2, timer::Polarity::ActiveHigh);
        dev_buzzer.set_duty(timer::Channel::C2, max_duty / 2);
        // Buzzer

        // SBUS DMA
        let sbus_rx_pin = gpiob.pb7.into_alternate();
        let mut usart1_rx = dp
            .USART1
            .rx(
                sbus_rx_pin,
                serial::Config::default()
                    .baudrate(100_000.bps())
                    .dma(serial::config::DmaConfig::Rx),
                &clocks,
            )
            .unwrap();
        usart1_rx.listen_idle();

        let sbus_dma2 = dma::StreamsTuple::new(dp.DMA2);
        // buffer 1 becomes owned by the transfer
        let sbus_buffer1 =
            cortex_m::singleton!(: [u8; SBUS_BUFFER_SIZE] = [0; SBUS_BUFFER_SIZE]).unwrap();
        // buffer 2 goes into task local and is swapped on each transfer
        let sbus_buffer2 =
            cortex_m::singleton!(: [u8; SBUS_BUFFER_SIZE] = [0; SBUS_BUFFER_SIZE]).unwrap();
        let mut sbus_transfer = dma::Transfer::init_peripheral_to_memory(
            sbus_dma2.2,
            usart1_rx,
            sbus_buffer1,
            None,
            dma::config::DmaConfig::default()
                .memory_increment(true)
                .fifo_enable(true)
                .fifo_error_interrupt(true)
                .transfer_complete_interrupt(true),
        );
        sbus_transfer.start(|_rx| {});
        // SBUS DMA

        // Status LED
        let led_channel = timer::Channel3::new(gpiob.pb10);
        let mut led_pwm = dp.TIM2.pwm_us(led_channel, 100.micros(), &clocks);
        led_pwm.enable(timer::Channel::C3);
        led_pwm.set_polarity(timer::Channel::C3, timer::Polarity::ActiveHigh);
        // Status LED

        // Tasks
        // imu
        imu_config::spawn().unwrap();
        read_pressure_sensor::spawn().unwrap();
        read_magnetometer::spawn().unwrap();
        read_temp_sensor::spawn().unwrap();
        // buzz_task::spawn().unwrap();

        (
            Shared {
                dev_fc_usb,
                dev_accel,
                dev_gyro,
                dev_temp_sensor,
                sbus_transfer,
            },
            Local {
                pin_imu_drdy,
                dev_pressure_sensor,
                tim_10,
                dev_buzzer,
                dev_magnetometer,
                sbus_buffer: Some(sbus_buffer2),
            },
        )
    }

    // Interrupts

    #[task(priority=3, binds=OTG_FS, shared=[dev_fc_usb], local=[poll_count: u32 = 0])]
    fn usb_fs(mut cx: usb_fs::Context) {
        cx.shared.dev_fc_usb.lock(|dev| {
            *cx.local.poll_count += 1;
            if *cx.local.poll_count % 1000 == 0 {
                defmt::debug!("USB interrupt fired {} times", *cx.local.poll_count);
            }

            let poll_result = dev.poll();
            let current_state = dev.state();

            defmt::debug!(
                "USB State: {:?}, Poll result: {}",
                match current_state {
                    device::UsbDeviceState::Default => "Default",
                    device::UsbDeviceState::Addressed => "Addressed",
                    device::UsbDeviceState::Configured => "Configured",
                    device::UsbDeviceState::Suspend => "Suspend",
                },
                poll_result
            );

            if poll_result {
                defmt::debug!("USB device polled successfully in interrupt");
                // Additional processing if needed
            }
        });
    }

    #[task(priority=3, binds = EXTI0, local = [pin_imu_drdy])]
    fn imu_drdy(cx: imu_drdy::Context) {
        cx.local.pin_imu_drdy.clear_interrupt_pending_bit();
        defmt::debug!("IMU DRDY interrupt fired");
        match read_imu::spawn() {
            Ok(_) => defmt::debug!("Read IMU task spawned"),
            Err(_) => defmt::error!("Read IMU task spawn failed"),
        };
    }

    #[task(priority=2, shared=[dev_fc_usb])]
    async fn handle_incoming_msg(mut cx: handle_incoming_msg::Context) {
        info!("Handling incoming messages task spawned.");

        cx.shared.dev_fc_usb.lock(|dev| {
            if dev.poll() {
                while let Some(msg) = dev.recv_msg() {
                    defmt::info!("Received message: {:?}", msg.as_bytes());
                }
            }
        });
    }

    // Task to configure the IMU
    #[task(priority=2, shared=[dev_accel, dev_gyro])]
    async fn imu_config(cx: imu_config::Context) {
        let acc = cx.shared.dev_accel;
        let gyro = cx.shared.dev_gyro;

        // reset both sensors
        (acc, gyro).lock(|a, g| {
            imu::acc_enable(a).unwrap();
            imu::gyro_enable(g).unwrap();
            imu::configure_acc(a).unwrap();
            imu::configure_gyro(g).unwrap();
        });
    }

    // Task to read data from the IMU
    #[task(priority=2, shared=[dev_accel, dev_gyro])]
    async fn read_imu(cx: read_imu::Context) {
        let mut acc = cx.shared.dev_accel;
        let mut gyro = cx.shared.dev_gyro;
        let (accdata, time) = acc.lock(|a| {
            let data = a.acc_data().unwrap();
            let time = a.sensor_time_24bit().unwrap();

            (data, time)
        });
        let gyrodata = gyro.lock(|g| g.gyro_read_rate().unwrap());
        defmt::info!(
            "{}: [{:02}, {:02}, {:02}], [{:02}, {:02}, {:02}]",
            time,
            accdata.x,
            accdata.y,
            accdata.z,
            gyrodata.x,
            gyrodata.y,
            gyrodata.z
        );
    }

    #[task(priority=2, local=[dev_pressure_sensor])]
    async fn read_pressure_sensor(cx: read_pressure_sensor::Context) {
        let sensor = cx.local.dev_pressure_sensor;
        loop {
            let now = rtic_monotonics::systick::Systick::now();

            let data = sensor.sensor_values().unwrap();
            defmt::info!("Pressure: {} Pa", data.pressure);
            defmt::info!("Temperature: {} C", data.temperature);

            rtic_monotonics::systick::Systick::delay_until(now + SENSOR_LOOP_READ_DELAY.millis())
                .await;
        }
    }

    #[task(priority=2, shared = [dev_temp_sensor])]
    async fn read_temp_sensor(mut cx: read_temp_sensor::Context) {
        loop {
            let now = rtic_monotonics::systick::Systick::now();

            let mut temp: f32 = 0.0;
            let mut humidity: f32 = 0.0;
            cx.shared.dev_temp_sensor.lock(|s| {
                temp = s.temperature().unwrap();
                humidity = s.humidity().unwrap();
            });

            let scaled_temp = (temp * 100.0) as i32;
            let int_part = (scaled_temp / 100) as i16;
            let frac_part = (scaled_temp % 100) as u8;
            let scaled_hum = (humidity * 100.0) as i32;
            let int_hum = (scaled_hum / 100) as i16;
            let frac_hum = (scaled_hum % 100) as u8;
            defmt::info!("Temperature: {}.{} C", int_part, frac_part);
            defmt::info!("Humidity: {}.{} %", int_hum, frac_hum);

            rtic_monotonics::systick::Systick::delay_until(now + SENSOR_LOOP_READ_DELAY.millis())
                .await;
        }
    }

    #[task (priority=1, local = [tim_10, dev_buzzer], shared=[])]
    async fn buzz_task(cx: buzz_task::Context) {
        cx.local.dev_buzzer.enable_complementary(timer::Channel::C2);

        loop {
            info!("Playing arm sequence");
            play_sequence(cx.local.tim_10, cx.local.dev_buzzer, &ARM_SEQUENCE);
            cx.local.tim_10.delay_ms(500);

            info!("Playing error sequence");
            play_sequence(cx.local.tim_10, cx.local.dev_buzzer, &ERROR_SEQUENCE);
            cx.local.tim_10.delay_ms(500);

            info!("Playing nominal sequence");
            play_sequence(cx.local.tim_10, cx.local.dev_buzzer, &NOMINAL_SEQUENCE);
            cx.local.tim_10.delay_ms(500);

            info!("Playing USB connect sequence");
            play_sequence(cx.local.tim_10, cx.local.dev_buzzer, &USB_CONNECT_SEQUENCE);
            cx.local.tim_10.delay_ms(500);

            info!("Playing boot sequence");
            play_sequence(cx.local.tim_10, cx.local.dev_buzzer, &BOOT_SEQUENCE);
            cx.local.tim_10.delay_ms(500);

            info!("Playing power swap sequence");
            play_sequence(cx.local.tim_10, cx.local.dev_buzzer, &POWER_SWAP_SEQUENCE);
            cx.local.tim_10.delay_ms(500);
        }
    }

    #[task(priority=2, local=[dev_magnetometer])]
    async fn read_magnetometer(cx: read_magnetometer::Context) {
        let mag = cx.local.dev_magnetometer;
        loop {
            let now = rtic_monotonics::systick::Systick::now();
            let raw = mag.get_raw_mag_axes().unwrap();
            defmt::info!("x: {}, y: {}, z: {}", raw.x, raw.y, raw.z);
            rtic_monotonics::systick::Systick::delay_until(now + SENSOR_LOOP_READ_DELAY.millis())
                .await;
        }
    }

    #[task(priority=3, binds = DMA2_STREAM2, shared = [sbus_transfer], local = [sbus_buffer])]
    fn sbus_data(mut cx: sbus_data::Context) {
        defmt::info!("SBUS data");
        cx.shared.sbus_transfer.lock(|xfer: &mut SbusRxTransfer| {
            if xfer.is_idle() {
                let new_buf = cx.local.sbus_buffer.take().unwrap();
                let (buf, _current) = xfer.next_transfer(new_buf).unwrap();
                let chan_data = sbus::SbusData::new(*buf).parse();
                // free the buffer
                cx.local.sbus_buffer.replace(buf);
                match flight_chan_task::spawn(chan_data) {
                    Ok(_) => defmt::debug!("Flight channel task spawned"),
                    Err(_) => defmt::error!("Flight channel task spawn failed"),
                };
            }
        });
    }

    #[task(priority = 1)]
    async fn flight_chan_task(
        _cx: flight_chan_task::Context,
        chan_data: ollyfc::sbus::SbusChannels,
    ) {
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

fn play_sequence(
    tim10: &mut timer::Delay<pac::TIM10, 1000>,
    pwm: &mut BuzzerDev,
    sequence: &Sequence,
) {
    for (i, (freq, on_time, dead_time)) in sequence.iter().enumerate() {
        info!(
            "Tone {}: Frequency = {} Hz, On Time = {} ms, Dead Time = {} ms",
            i + 1,
            freq,
            on_time,
            dead_time
        );

        pwm.set_period(freq.Hz());
        pwm.set_duty(timer::Channel::C2, pwm.get_max_duty() / 2);
        tim10.delay_ms(*on_time);
        pwm.set_duty(timer::Channel::C2, 0);
        tim10.delay_ms(*dead_time);
    }
}

type Sequence = [(u32, u32, u32); 3];
const ARM_SEQUENCE: Sequence = [(1000, 300, 50), (1200, 100, 25), (1600, 50, 500)];
const ERROR_SEQUENCE: Sequence = [(800, 100, 50), (800, 100, 50), (800, 100, 500)];
const NOMINAL_SEQUENCE: Sequence = [(1500, 150, 75), (1700, 150, 75), (1900, 150, 500)];
const USB_CONNECT_SEQUENCE: Sequence = [(2000, 100, 50), (2200, 100, 50), (2400, 100, 500)];
const BOOT_SEQUENCE: Sequence = [(1000, 50, 25), (1500, 50, 25), (2000, 50, 500)];
const POWER_SWAP_SEQUENCE: Sequence = [(1800, 50, 25), (1600, 50, 25), (1200, 50, 500)];
