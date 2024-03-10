use bmp388::config::{InterruptConfig, OversamplingConfig};
use bmp388::Config;
use bmp388::SamplingRate;

pub const BMP388_ADDR: u8 = 0x76;
pub const BMP388_CFG: Config = Config {
    address: BMP388_ADDR,
    oversampling: OversamplingConfig {
        osr_pressure: bmp388::Oversampling::x4,
        osr_temperature: bmp388::Oversampling::x1,
    },
    sampling_rate: SamplingRate::ms5,
    filter: bmp388::Filter::c7,
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
