use bmi088::{Bmi088, IntConfiguration};
use core::convert::Infallible;
use defmt_rtt as _;
use embedded_hal_bus::spi::{AtomicDevice, AtomicError, DeviceError, NoDelay};
use panic_probe as _;
use rtic_monotonics::systick::Systick;
use stm32f4xx_hal::{gpio, pac, prelude::*, spi, spi::Spi};

type GyroDev = AtomicDevice<'static, Spi<pac::SPI3>, gpio::Pin<'B', 5, gpio::Output>, NoDelay>;
type AccelDev = AtomicDevice<'static, Spi<pac::SPI3>, gpio::Pin<'D', 2, gpio::Output>, NoDelay>;
type SpiBusErr = AtomicError<DeviceError<spi::Error, Infallible>>;

use bmi088::{interface::SpiInterface, PinActive};

pub fn acc_enable(a: &mut Bmi088<SpiInterface<AccelDev>>) -> Result<(), bmi088::Error<SpiBusErr>> {
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

pub fn gyro_enable(g: &mut Bmi088<SpiInterface<GyroDev>>) -> Result<(), bmi088::Error<SpiBusErr>> {
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

pub fn configure_acc(
    a: &mut Bmi088<SpiInterface<AccelDev>>,
) -> Result<(), bmi088::Error<SpiBusErr>> {
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

pub fn configure_gyro(
    g: &mut Bmi088<SpiInterface<GyroDev>>,
) -> Result<(), bmi088::Error<SpiBusErr>> {
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
