use defmt::{debug, error, Format};

use crate::sbus::FlightControls;
use crate::w25q::{FlashMem, MemError};

// size of the log item, in bytes
pub const LOG_SIZE: usize = 64;
pub const LOG_PAGE_SIZE: usize = crate::w25q::PAGE_SIZE as usize;
pub const LOGS_IN_PAGE: usize = LOG_PAGE_SIZE / LOG_SIZE;

pub struct FlightLogger<T: FlashMem> {
    pub mem: T,
    addr_ptr: u32,
}

impl<T: FlashMem> FlightLogger<T> {
    pub fn new(mem: T) -> Self {
        Self {
            mem: mem,
            addr_ptr: 0u32,
        }
    }

    pub fn init(&mut self, start: u32) {
        debug!("Initializing logger at {:?}", start);
        self.addr_ptr = start;
    }

    pub fn log(&mut self, data: &[FlightLogData; 8]) {
        let mut buf = [0u8; 256];
        for (i, log) in data.iter().enumerate() {
            buf[i * LOG_SIZE..(i + 1) * LOG_SIZE].copy_from_slice(&log.to_bytes());
        }
        match self.mem.page_program(self.addr_ptr, &buf) {
            Ok(_) => (),
            Err(e) => match e {
                MemError::SpiError(_) => error!("SPI error"),
                MemError::NotAlignedError => error!("Page not aligned"),
                MemError::OutOfBoundsError => error!("Data out of bounds"),
            },
        };
        self.incr_addr_ptr();
    }

    fn incr_addr_ptr(&mut self) {
        self.addr_ptr += self.mem.page_size() as u32;
    }
}

#[derive(Clone, Copy, Debug, Format)]
pub struct SensorInput {
    pub accel_x: f32,
    pub accel_y: f32,
    pub accel_z: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
}

impl SensorInput {
    pub fn default() -> Self {
        Self {
            accel_x: 0.0,
            accel_y: 0.0,
            accel_z: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            roll: 0.0,
        }
    }
}
#[derive(Clone, Copy, Debug, Format)]
pub struct SBusInput {
    pub throttle: u16,
    pub aileron: u16,
    pub elevator: u16,
    pub rudder: u16,
    pub arm: u16,
    pub enable: u16,
    pub record: u16,
}

impl SBusInput {
    // Create a new SBusInput from a FlightControls struct
    pub fn from(sbus_input: FlightControls) -> Self {
        SBusInput {
            throttle: sbus_input.throttle,
            aileron: sbus_input.aileron,
            elevator: sbus_input.elevator,
            rudder: sbus_input.rudder,
            arm: sbus_input.arm,
            enable: sbus_input.enable,
            record: sbus_input.record,
        }
    }
    pub fn default() -> Self {
        Self {
            throttle: 0,
            aileron: 0,
            elevator: 0,
            rudder: 0,
            arm: 0,
            enable: 0,
            record: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, Format)]
pub struct FlightLogData {
    pub timestamp: u32,
    pub sbus_input: SBusInput,
    pub sensor_input: SensorInput,
}

impl FlightLogData {
    pub fn to_bytes(&self) -> [u8; LOG_SIZE] {
        let mut bytes = [0u8; LOG_SIZE];

        // timestamp
        bytes[0..4].copy_from_slice(&self.timestamp.to_be_bytes());

        // sbus
        bytes[4..6].copy_from_slice(&self.sbus_input.throttle.to_be_bytes());
        bytes[6..8].copy_from_slice(&self.sbus_input.aileron.to_be_bytes());
        bytes[8..10].copy_from_slice(&self.sbus_input.elevator.to_be_bytes());
        bytes[10..12].copy_from_slice(&self.sbus_input.rudder.to_be_bytes());
        bytes[12..14].copy_from_slice(&self.sbus_input.arm.to_be_bytes());
        bytes[14..16].copy_from_slice(&self.sbus_input.enable.to_be_bytes());
        bytes[16..18].copy_from_slice(&self.sbus_input.record.to_be_bytes());

        // sensor
        bytes[18..22].copy_from_slice(&self.sensor_input.accel_x.to_be_bytes());
        bytes[22..26].copy_from_slice(&self.sensor_input.accel_y.to_be_bytes());
        bytes[30..34].copy_from_slice(&self.sensor_input.accel_z.to_be_bytes());
        bytes[34..38].copy_from_slice(&self.sensor_input.pitch.to_be_bytes());
        bytes[38..42].copy_from_slice(&self.sensor_input.yaw.to_be_bytes());
        bytes[42..46].copy_from_slice(&self.sensor_input.roll.to_be_bytes());

        bytes
    }

    pub fn default() -> Self {
        Self {
            timestamp: 0,
            sbus_input: SBusInput::default(),
            sensor_input: SensorInput::default(),
        }
    }
}
