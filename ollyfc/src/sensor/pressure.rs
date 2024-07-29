use bmp388::config::{InterruptConfig, OversamplingConfig};
use bmp388::Config;

// BST-BMP388-DS001-07 Rev 1.7 112020
// Section 3.5, Table 10, "Filter Selection"
// Use Case - Drone
//  - Mode: Normal
//  - Oversampling: Standard Resolution
//  - Pressure: x8
//  - Temperature: x1
//  - IIR Coef.: 2 (c3)
//  - Sampling Rate: 20ms (ODR 50Hz)
//  - RMS Noise: 11 cm
//
pub const BMP388_CFG: Config = Config {
    address: bmp388::Addr::Secondary as u8,
    oversampling: OversamplingConfig {
        osr_pressure: bmp388::Oversampling::x8,
        osr_temperature: bmp388::Oversampling::x1,
    },
    sampling_rate: bmp388::SamplingRate::ms20,
    filter: bmp388::Filter::c3,
    interrupt_config: InterruptConfig {
        output: bmp388::OutputMode::PushPull,
        active_high: true,
        latch: false,
        data_ready_interrupt_enable: true,
    },
    power_control: bmp388::PowerControl {
        pressure_enable: true,
        temperature_enable: true,
        mode: bmp388::PowerMode::Normal,
    },
};
