pub mod flight_log;

use crate::{pwm, sbus, sensor};

pub enum Log {
    SbusInputRaw(pwm::servo::ServoPwmOut),
    ServoPwmOutput(sbus::SbusChannels),
    ImuFrameRaw(sensor::imu::ImuDataRaw),
}

impl Log {
    fn log_size(&self) -> usize {
        match self {
            Log::SbusInputRaw(_) => size_of::<pwm::servo::ServoPwmOut>(),
            Log::ServoPwmOutput(_) => size_of::<sbus::SbusChannels>(),
            Log::ImuFrameRaw(_) => size_of::<sensor::imu::ImuDataRaw>(),
        }
    }

    pub fn as_bytes(&self) -> &[u8] {
        unsafe {
            match self {
                Log::SbusInputRaw(data) => core::slice::from_raw_parts(
                    (data as *const pwm::servo::ServoPwmOut) as *const u8,
                    size_of::<pwm::servo::ServoPwmOut>(),
                ),
                Log::ServoPwmOutput(data) => core::slice::from_raw_parts(
                    (data as *const sbus::SbusChannels) as *const u8,
                    size_of::<sbus::SbusChannels>(),
                ),
                Log::ImuFrameRaw(data) => core::slice::from_raw_parts(
                    (data as *const sensor::imu::ImuDataRaw) as *const u8,
                    size_of::<sensor::imu::ImuDataRaw>(),
                ),
            }
        }
    }

    /// panic if the chunk_size is greater than the log size
    pub fn split_log(&self, chunk_size: usize) -> (&[u8], &[u8]) {
        let bytes = self.as_bytes();
        bytes.split_at(chunk_size.min(bytes.len()))
    }
}
