#![no_std]
#![no_main]

use core::ops::Range;

use rtt_target::{rprint, rprintln, rtt_init_print};

use cortex_m_rt::entry;

use stm32f4xx_hal::{i2c, pac, prelude::*};

const VALID_ADDR_RANGE: Range<u8> = 0x08..0x78;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let dp = pac::Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();

    // Configure I2C3
    let scl = gpioa.pa8;
    let sda = gpioc.pc9;
    let mut i2c = i2c::I2c::new(dp.I2C3, (scl, sda), i2c::Mode::standard(100.kHz()), &clocks);

    rprintln!("Start i2c scanning...");
    rprintln!();

    for addr in 0x00_u8..0x80 {
        // Write the empty array and check the slave response.
        let byte: [u8; 1] = [0; 1];
        if VALID_ADDR_RANGE.contains(&addr) && i2c.write(addr, &byte).is_ok() {
            rprint!("{:02x}", addr);
        } else {
            rprint!("..");
        }
        if addr % 0x10 == 0x0F {
            rprintln!();
        } else {
            rprint!(" ");
        }
    }

    rprintln!();
    rprintln!("Done!");

    // In main():
    test_sensor(&mut i2c, 0x40, 0xFF); // HDC1080 Manufacturer ID
    test_sensor(&mut i2c, 0x77, 0x00); // BMP388 Chip ID
    test_sensor(&mut i2c, 0x1C, 0x0F); // LIS3MDL Who Am I

    #[allow(clippy::empty_loop)]
    loop {}
}

use core::panic::PanicInfo;
#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {} // You might need a compiler fence in here.
}

fn test_sensor(i2c: &mut i2c::I2c<pac::I2C3>, address: u8, register: u8) {
    let mut buffer = [0u8; 2];
    match i2c.write_read(address, &[register], &mut buffer) {
        Ok(_) => rprintln!(
            "Sensor 0x{:02X} read successful: 0x{:02X}{:02X}",
            address,
            buffer[0],
            buffer[1]
        ),
        Err(e) => rprintln!("Sensor 0x{:02X} read failed: {:?}", address, e),
    }
}
