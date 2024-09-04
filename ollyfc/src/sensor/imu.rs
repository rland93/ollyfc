use bmi088::Sensor3DData;

#[derive(Debug, Clone, Copy)]
pub struct StandardAccFrame {
    pub header: (bool, bool),
    pub data: bmi088::Sensor3DData,
}

impl StandardAccFrame {
    pub fn default() -> StandardAccFrame {
        StandardAccFrame {
            header: (false, false),
            data: bmi088::Sensor3DData::default(),
        }
    }

    pub fn from_le_slice(data: &[u8]) -> StandardAccFrame {
        let mut frame = StandardAccFrame::default();
        frame.header = (data[0] & 0x01 != 0, data[0] & 0x00 != 0);
        frame.data.x = i16::from_le_bytes([data[1], data[2]]);
        frame.data.y = i16::from_le_bytes([data[3], data[4]]);
        frame.data.z = i16::from_le_bytes([data[5], data[6]]);
        frame
    }
}

pub struct ImuDataRaw {
    pub accel: Sensor3DData,
    pub gyro: Sensor3DData,
}
