#![cfg_attr(feature = "no_std", no_std)]

pub mod cmd;
pub mod log;

#[cfg(feature = "std")]
use serde::{Deserialize, Serialize};

pub const LOG_SIZE: usize = 64;

#[cfg_attr(feature = "no_std", derive(defmt::Format))]
#[cfg_attr(feature = "std", derive(Serialize, Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
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

#[cfg_attr(feature = "no_std", derive(defmt::Format))]
#[cfg_attr(feature = "std", derive(Serialize, Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
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

#[cfg_attr(feature = "no_std", derive(defmt::Format))]
#[cfg_attr(feature = "std", derive(Serialize, Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlPolicy {
    pub elevator: u16,
    pub aileron: u16,
    pub rudder: u16,
    pub throttle: u16,
}

impl ControlPolicy {
    pub fn default() -> Self {
        Self {
            elevator: 0,
            aileron: 0,
            rudder: 0,
            throttle: 0,
        }
    }
}

#[cfg_attr(feature = "no_std", derive(defmt::Format))]
#[cfg_attr(feature = "std", derive(Serialize, Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlightLogData {
    pub timestamp: u32,
    pub sbus_input: SBusInput,
    pub sensor_input: SensorInput,
    pub control_policy: ControlPolicy,
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
        bytes[26..30].copy_from_slice(&self.sensor_input.accel_z.to_be_bytes());
        bytes[30..34].copy_from_slice(&self.sensor_input.pitch.to_be_bytes());
        bytes[34..38].copy_from_slice(&self.sensor_input.yaw.to_be_bytes());
        bytes[38..42].copy_from_slice(&self.sensor_input.roll.to_be_bytes());

        // control policy
        bytes[42..44].copy_from_slice(&self.control_policy.elevator.to_be_bytes());
        bytes[44..46].copy_from_slice(&self.control_policy.aileron.to_be_bytes());
        bytes[46..48].copy_from_slice(&self.control_policy.rudder.to_be_bytes());
        bytes[48..50].copy_from_slice(&self.control_policy.throttle.to_be_bytes());

        bytes
    }

    pub fn from_bytes(bytes: &[u8; LOG_SIZE]) -> Self {
        let timestamp = u32::from_be_bytes(bytes[0..4].try_into().unwrap());

        let sbus_input = SBusInput {
            throttle: u16::from_be_bytes(bytes[4..6].try_into().unwrap()),
            aileron: u16::from_be_bytes(bytes[6..8].try_into().unwrap()),
            elevator: u16::from_be_bytes(bytes[8..10].try_into().unwrap()),
            rudder: u16::from_be_bytes(bytes[10..12].try_into().unwrap()),
            arm: u16::from_be_bytes(bytes[12..14].try_into().unwrap()),
            enable: u16::from_be_bytes(bytes[14..16].try_into().unwrap()),
            record: u16::from_be_bytes(bytes[16..18].try_into().unwrap()),
        };

        let sensor_input = SensorInput {
            accel_x: f32::from_be_bytes(bytes[18..22].try_into().unwrap()),
            accel_y: f32::from_be_bytes(bytes[22..26].try_into().unwrap()),
            accel_z: f32::from_be_bytes(bytes[26..30].try_into().unwrap()),
            pitch: f32::from_be_bytes(bytes[30..34].try_into().unwrap()),
            yaw: f32::from_be_bytes(bytes[34..38].try_into().unwrap()),
            roll: f32::from_be_bytes(bytes[38..42].try_into().unwrap()),
        };

        let control_policy = ControlPolicy {
            elevator: u16::from_be_bytes(bytes[42..44].try_into().unwrap()),
            aileron: u16::from_be_bytes(bytes[44..46].try_into().unwrap()),
            rudder: u16::from_be_bytes(bytes[46..48].try_into().unwrap()),
            throttle: u16::from_be_bytes(bytes[48..50].try_into().unwrap()),
        };

        Self {
            timestamp,
            sbus_input,
            sensor_input,
            control_policy,
        }
    }

    pub fn default() -> Self {
        Self {
            timestamp: 0,
            sbus_input: SBusInput::default(),
            sensor_input: SensorInput::default(),
            control_policy: ControlPolicy::default(),
        }
    }
}
