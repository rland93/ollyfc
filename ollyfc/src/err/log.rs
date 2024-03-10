use defmt::error;
use stm32f4xx_hal::i2c;

// enum containing all possible error types.
pub enum ErrLog {
    I2C(u8),
    BMP388(u8),
    LIS3MDL(u8),
    OS(u8),
}

// handler for logging i2c errors
pub fn err_i2c_h(err: i2c::Error) -> ErrLog {
    let e: u8 = match err {
        i2c::Error::Overrun => 0x01,
        i2c::Error::Timeout => 0x02,
        i2c::Error::Bus => 0x03,
        i2c::Error::ArbitrationLoss => 0x04,
        i2c::Error::NoAcknowledge(source) => match source {
            i2c::NoAcknowledgeSource::Address => 0x05,
            i2c::NoAcknowledgeSource::Unknown => 0x06,
            i2c::NoAcknowledgeSource::Data => 0x07,
        },
        i2c::Error::Crc => 0x08,
        _ => 0x09,
    };
    ErrLog::I2C(e)
}

// handler for bmp388 errors
pub fn err_pres_h(err: bmp388::Error) -> ErrLog {
    if err.fatal {
        ErrLog::BMP388(0x01)
    } else if err.config {
        ErrLog::BMP388(0x02)
    } else {
        ErrLog::BMP388(0x03)
    }
}

// handler for lis3mdl errors
pub fn err_mag_h(err: lis3mdl::Error) -> ErrLog {
    match err {
        lis3mdl::Error::InvalidValue => ErrLog::LIS3MDL(0x01),
        lis3mdl::Error::IncorrectDeviceIdFound => ErrLog::LIS3MDL(0x02),
        lis3mdl::Error::CommunicationError => ErrLog::I2C(0x03),
    }
}

// os errors
pub enum RticErr {
    SpawnFail,
}

// handler for os errors
pub fn err_os_h(err: RticErr) -> ErrLog {
    ErrLog::OS(err as u8)
}

// For now, just print to the RTT console. TODO: add datalog specifically for
// events and errors.
pub fn log_err(err: ErrLog) {
    match err {
        ErrLog::I2C(e) => error!("I2C {:x}", e),
        ErrLog::BMP388(e) => error!("BMP388 {:x}", e),
        ErrLog::OS(e) => error!("OS {:x}", e),
        ErrLog::LIS3MDL(e) => error!("LIS3MDL {:x}", e),
    }
}
