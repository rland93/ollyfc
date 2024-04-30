#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use bmi088::{Bmi088, IntConfiguration};
use core::convert::Infallible;
use core::ptr::addr_of;
use defmt_rtt as _;
use embedded_hal_bus::spi::{AtomicDevice, AtomicError, DeviceError, NoDelay};
use embedded_hal_bus::util::AtomicCell;
use panic_probe as _;
use rtic_monotonics::systick::Systick;
use stm32f4xx_hal::{
    gpio, pac,
    prelude::*,
    spi,
    spi::{Mode, Phase, Polarity, Spi, Spi3},
};

type GyroDev = AtomicDevice<'static, Spi<pac::SPI3>, gpio::Pin<'B', 5, gpio::Output>, NoDelay>;
type AccelDev = AtomicDevice<'static, Spi<pac::SPI3>, gpio::Pin<'D', 2, gpio::Output>, NoDelay>;
type Spi3Bus = AtomicCell<Spi<pac::SPI3>>;
type SpiBusErr = AtomicError<DeviceError<spi::Error, Infallible>>;

use bmi088::{interface::SpiInterface, PinActive};
static mut SPI3BUS: Option<Spi3Bus> = None;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {

    use super::*;

    #[shared]
    struct Shared {
        accel: Bmi088<SpiInterface<AccelDev>>,
        gyro: Bmi088<SpiInterface<GyroDev>>,
        _spi3bus: &'static Option<Spi3Bus>,
    }

    #[local]
    struct Local {
        imu_drdy: gpio::Pin<'C', 0, gpio::Input>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        // Setup clocks
        let mut dp = cx.device;
        let mut syscfg = dp.SYSCFG.constrain();
        let rcc = dp.RCC.constrain();
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 64_000_000, systick_mono_token);
        let clocks = rcc.cfgr.sysclk(64.MHz()).freeze();

        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();
        let gpiod = dp.GPIOD.split();

        let spi: Spi3 = Spi3::new(
            dp.SPI3,
            (
                gpioc.pc10.into_alternate(),
                gpioc.pc11.into_alternate(),
                gpioc.pc12.into_alternate(),
            ),
            Mode {
                polarity: Polarity::IdleLow,
                phase: Phase::CaptureOnFirstTransition,
            },
            8.MHz(),
            &clocks,
        );

        let mut acc_cs = gpiod.pd2.into_push_pull_output();
        let mut gyr_cs = gpiob.pb5.into_push_pull_output();
        acc_cs.set_high();
        gyr_cs.set_high();

        let (mut accel_dev, mut gyro_dev) = unsafe {
            SPI3BUS = Some(AtomicCell::new(spi));
            let bus = SPI3BUS.as_ref().unwrap();
            let acc = AtomicDevice::new_no_delay(&bus, acc_cs).unwrap();
            let gyro = AtomicDevice::new_no_delay(&bus, gyr_cs).unwrap();
            (Bmi088::new_with_spi(acc), Bmi088::new_with_spi(gyro))
        };
        // do dummy chipid read for both sensors
        accel_dev.acc_chipid().unwrap();
        gyro_dev.gyro_chipid().unwrap();

        let mut imu_drdy = gpioc.pc0.into_pull_up_input();
        imu_drdy.make_interrupt_source(&mut syscfg);
        imu_drdy.trigger_on_edge(&mut dp.EXTI, gpio::Edge::Falling);
        imu_drdy.enable_interrupt(&mut dp.EXTI);
        unsafe {
            pac::NVIC::unmask(imu_drdy.interrupt());
        }

        imu_config::spawn().unwrap();

        defmt::info!("setup done");

        (
            Shared {
                _spi3bus: unsafe { addr_of!(SPI3BUS).as_ref().unwrap() },
                accel: accel_dev,
                gyro: gyro_dev,
            },
            Local { imu_drdy },
        )
    }

    // Interrupt
    #[task(binds = EXTI0, local = [imu_drdy])]
    fn imu_drdy(cx: imu_drdy::Context) {
        cx.local.imu_drdy.clear_interrupt_pending_bit();
        read_imu::spawn().unwrap();
    }

    // Task to configure the IMU
    #[task(shared=[accel, gyro])]
    async fn imu_config(cx: imu_config::Context) {
        let acc = cx.shared.accel;
        let gyro = cx.shared.gyro;

        // reset both sensors
        (acc, gyro).lock(|a, g| {
            acc_enable(a).unwrap();
            gyro_enable(g).unwrap();
            configure_acc(a).unwrap();
            configure_gyro(g).unwrap();
        });
    }

    #[task(shared=[accel, gyro])]
    async fn read_imu(cx: read_imu::Context) {
        let mut acc = cx.shared.accel;
        let mut gyro = cx.shared.gyro;
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
}

fn acc_enable(a: &mut Bmi088<SpiInterface<AccelDev>>) -> Result<(), bmi088::Error<SpiBusErr>> {
    a.acc_soft_reset()?;
    Systick::delay_ms(&mut Systick, 1);
    // dummy read, necessary for some reason.
    a.acc_chipid()?;

    let power = bmi088::AccOffOn::On;
    a.acc_enable_write(power)?;
    Systick::delay_us(&mut Systick, 450);

    let wake = bmi088::AccWakeSuspend::Active;
    a.acc_wake_suspend_write(wake)?;

    // check matching chipID
    let expected: u8 = 0x1E;
    let acid = a.acc_chipid()?;
    if acid != expected {
        panic!("bad chipid accel: {:02}, expected {:02}", acid, expected);
    }
    Ok(())
}

fn gyro_enable(g: &mut Bmi088<SpiInterface<GyroDev>>) -> Result<(), bmi088::Error<SpiBusErr>> {
    g.acc_soft_reset().unwrap();
    Systick::delay_ms(&mut Systick, 30);

    // power modes
    g.gyro_power_mode_write(bmi088::GyroPowerMode::Normal)?;

    // check matching chipid
    let gcid = g.gyro_chipid()?;
    let expected: u8 = 0x0F;
    if gcid != expected {
        panic!("bad chipid gyro: {:02x}, expected {:02x}", gcid, expected);
    }
    Ok(())
}

fn configure_acc(a: &mut Bmi088<SpiInterface<AccelDev>>) -> Result<(), bmi088::Error<SpiBusErr>> {
    // configuration
    let configuration = bmi088::AccelerometerConfig {
        conf: bmi088::AccConf {
            acc_bwp: bmi088::AccBandwidth::X1,
            acc_odr: bmi088::AccDataRate::Hz100,
        },
        acc_range: bmi088::AccRange::G3,
    };

    a.acc_configuration_write(configuration)?;
    let read_conf = a.acc_configuration_read()?;
    assert!(read_conf == configuration);

    // interrupts

    // int 2 -- data ready output
    let int2_conf = IntConfiguration {
        int_pin: bmi088::IntPin::Output,
        int_od: bmi088::PinBehavior::OpenDrain,
        int_lvl: PinActive::ActiveLow,
    };
    a.int2_io_conf_write(int2_conf)?;
    let read_conf = a.int2_io_conf_read()?;
    assert!(read_conf == int2_conf);

    // int 1 -- set interrupt input on gyro data active low
    let int1_conf = IntConfiguration {
        int_pin: bmi088::IntPin::Input,
        int_od: bmi088::PinBehavior::PushPull,
        int_lvl: PinActive::ActiveHigh,
    };
    a.int1_io_conf_write(int1_conf)?;
    let read_conf = a.int1_io_conf_read()?;
    assert!(read_conf == int1_conf);

    // map data ready to int2
    let drdy_int2 = bmi088::AccDrdyMap::Int2;
    a.acc_map_drdy_write(drdy_int2)?;
    let read_drdy_int2 = a.acc_map_drdy_read()?;
    assert!(read_drdy_int2 == drdy_int2);

    Ok(())
}

fn configure_gyro(g: &mut Bmi088<SpiInterface<GyroDev>>) -> Result<(), bmi088::Error<SpiBusErr>> {
    // configuration
    let bandwidth = bmi088::GyroBandwidth::Hz32;
    let range = bmi088::GyroRange::Dps2000;

    g.gyro_bandwidth_write(bandwidth)?;
    g.gyro_range_write(range)?;

    // interrupts
    // int3 -- active low
    g.gyro_conf_int3_write(bmi088::PinActive::ActiveHigh, bmi088::PinBehavior::PushPull)?;
    // int3 map to gyro data ready
    g.gyro_drdy_map_write(bmi088::GyroDrdyMap::Int3)?;
    g.gyro_drdy_en(true)?;
    Ok(())
}
