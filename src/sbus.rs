pub struct SbusData {
    raw: [u8; 25],
}

pub struct SbusChannels {
    channels: [u16; 16],
    digital_channels: [bool; 8],
    lost_frame: bool,
    failsafe_activated: bool,
}

#[derive(defmt::Format)]
pub struct FlightControls {
    pub throttle: u16,
    pub aileron: u16,
    pub elevator: u16,
    pub rudder: u16,
    pub arm: u16,
    pub enable: u16,
    pub record: u16,
}

impl SbusChannels {
    pub fn get_channel(&self, channel: usize) -> u16 {
        self.channels[channel - 1]
    }

    pub fn get_digital_channel(&self, channel: usize) -> bool {
        self.digital_channels[channel - 1]
    }

    pub fn is_lost_frame(&self) -> bool {
        self.lost_frame
    }

    pub fn is_failsafe_activated(&self) -> bool {
        self.failsafe_activated
    }

    pub fn get_input(&self) -> FlightControls {
        FlightControls {
            throttle: self.get_channel(1),
            aileron: self.get_channel(2),
            elevator: self.get_channel(3),
            rudder: self.get_channel(4),
            arm: self.get_channel(5) as u16,
            enable: self.get_channel(6) as u16,
            record: self.get_channel(7) as u16,
        }
    }
}

impl SbusData {
    pub fn new(data: [u8; 25]) -> Self {
        SbusData { raw: data }
    }

    pub fn parse(&self) -> SbusChannels {
        let mut channels = [0u16; 16];
        let mut digital_channels = [false; 8];

        channels[0] = ((self.raw[1] as u16 | (self.raw[2] as u16) << 8) & 0x07FF) as u16;
        channels[1] = ((self.raw[2] as u16 >> 3 | (self.raw[3] as u16) << 5) & 0x07FF) as u16;
        channels[2] =
            ((self.raw[3] as u16 >> 6 | (self.raw[4] as u16) << 2 | (self.raw[5] as u16) << 10)
                & 0x07FF) as u16;
        channels[3] = ((self.raw[5] as u16 >> 1 | (self.raw[6] as u16) << 7) & 0x07FF) as u16;
        // ... Continue for all 16 channels

        for i in 0..8 {
            digital_channels[i] = (self.raw[23] >> i) & 0x01 != 0;
        }

        let lost_frame = (self.raw[23] >> 2) & 0x01 != 0;
        let failsafe_activated = (self.raw[23] >> 3) & 0x01 != 0;

        SbusChannels {
            channels,
            digital_channels,
            lost_frame,
            failsafe_activated,
        }
    }
}
