use crate::app::sensor_task;

use mpu6050_dmp::{
    accel::Accel, address::Address, config::DigitalLowPassFilter, quaternion::Quaternion,
    sensor::Mpu6050, yaw_pitch_roll::YawPitchRoll,
};
use ollyfc_common::SensorInput;
use rtic::Mutex;
use stm32f4xx_hal::{
    gpio::{Alternate, Edge, Input, OpenDrain, Pin},
    i2c::I2c,
    pac::{EXTI, I2C1, TIM2},
    prelude::*,
    rcc::Clocks,
    syscfg::SysCfg,
    timer::Delay,
};

pub fn read_sensor_i2c(cx: &mut sensor_task::Context) {
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

pub fn mpu_6050_init(
    interrupt: &mut Pin<'B', 8, Input>,
    scl: Pin<'B', 6, Alternate<4, OpenDrain>>,
    sda: Pin<'B', 7, Alternate<4, OpenDrain>>,
    i2c1: I2C1,
    exti: &mut EXTI,
    syscfg: &mut SysCfg,
    clocks: &Clocks,
    delay: &mut Delay<TIM2, 1000000>,
) -> Mpu6050<I2c<I2C1>> {
    // Configure pin for interrupt on data ready
    interrupt.make_interrupt_source(syscfg);
    interrupt.trigger_on_edge(exti, Edge::Falling);
    interrupt.enable_interrupt(exti);
    let i2c_dev = i2c1.i2c((scl, sda), 400u32.kHz(), &clocks);
    let mut mpu6050: Mpu6050<I2c<I2C1>> =
        Mpu6050::new(i2c_dev, Address::default()).expect("Could not initialize MPU6050!");
    // digital motion processor
    mpu6050.initialize_dmp(delay).unwrap();
    mpu6050.enable_fifo().unwrap();
    mpu6050
        .set_digital_lowpass_filter(DigitalLowPassFilter::Filter2)
        .unwrap();
    mpu6050.disable_interrupts().unwrap();
    mpu6050.interrupt_fifo_oflow_en().unwrap();
    mpu6050
}
