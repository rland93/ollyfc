use crate::app::gyro_task;
use bmi160::Sensor3DData;
use defmt::{debug, error, info};
use rtic::Mutex;
use rtic_monotonics::{systick::ExtU32, Monotonic};

const PI: f32 = 3.141592653589793238462643383279502884;
const ALPHA: f32 = 0.98;
const DEG_TO_RAD: f32 = PI / 180.0;
const SENS: f32 = 500.0 / i16::MAX as f32;

pub async fn gyro_task_fn(cx: &mut gyro_task::Context<'_>) {
    info!("Gyro task started");
    let mut t0 = 0u32;
    let mut angle: (f32, f32, f32) = (0.0, 0.0, 0.0);
    loop {
        let now = rtic_monotonics::systick::Systick::now();

        let driver = match cx.local.gyro.as_mut() {
            Some(g) => g,
            None => {
                error!("No gyro");
                return;
            }
        };

        // acquire data from the driver
        let selector = bmi160::SensorSelector::new().accel().gyro().time();

        let data = match driver.data(selector) {
            Ok(d) => d,
            Err(e) => {
                error!("Error reading data");
                continue;
            }
        };

        let default_3ddata = bmi160::Sensor3DData {
            x: -1,
            y: -1,
            z: -1,
        };
        let t1 = match data.time {
            Some(t) => t,
            None => {
                defmt::warn!("No time data");
                0
            }
        };
        let dt = (t1 - t0) as f32 * 39.0 / 1e6 as f32;
        t0 = t1;

        let accel = match data.accel {
            Some(a) => a,
            None => {
                defmt::warn!("No accel data");
                default_3ddata
            }
        };
        let gyro = match data.gyro {
            Some(g) => g,
            None => {
                defmt::warn!("No gyro data");
                default_3ddata
            }
        };

        let gyro_ang = (rad_s(gyro.x), rad_s(gyro.y), rad_s(gyro.z));

        // complementary filter

        debug!("dt: {}", dt);

        angle = (
            gyro_ang.0 * dt as f32 + angle.0,
            gyro_ang.1 * dt as f32 + angle.1,
            gyro_ang.2 * dt as f32 + angle.2,
        );

        let acc_ang = accel_angle(accel);

        angle.0 = ALPHA * angle.0 + (1.0 - ALPHA) * acc_ang.0;
        angle.1 = ALPHA * angle.1 + (1.0 - ALPHA) * acc_ang.1;
        angle.2 = ALPHA * angle.2 + (1.0 - ALPHA) * acc_ang.2;

        debug!(
            "{} {} {}",
            angle.0 * 180.0 / PI,
            angle.1 * 180.0 / PI,
            angle.2 * 180.0 / PI,
        );

        // update shared sensor object
        cx.shared.sensor.lock(|sensor| {
            sensor.pitch = angle.0;
            sensor.roll = angle.1;
            sensor.yaw = angle.2;
        });

        // yielde
        rtic_monotonics::systick::Systick::delay_until(now + 50u32.millis()).await;
    }
}

pub fn rad_s(x: i16) -> f32 {
    x as f32 * SENS * DEG_TO_RAD
}

pub fn accel_angle(accel: Sensor3DData) -> (f32, f32, f32) {
    (
        libm::atan2f(accel.x as f32, accel.z as f32),
        libm::atan2f(accel.y as f32, accel.z as f32),
        0.0, // no yaw
    )
}
