use rtic::Mutex;
use stm32f4xx_hal::serial::RxISR;

pub struct SbusData {
    raw: [u8; 25],
}

#[allow(dead_code)]
pub struct SbusChannels {
    channels: [u16; 16],
    digital_channels: [bool; 8],
    lost_frame: bool,
    failsafe_activated: bool,
}

#[derive(defmt::Format, Clone)]
pub struct FlightControls {
    pub throttle: u16,
    pub aileron: u16,
    pub elevator: u16,
    pub rudder: u16,
    pub arm: u16,
    pub enable: u16,
    pub record: u16,
}

impl FlightControls {
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

impl SbusChannels {
    pub fn get_channel_by_no(&self, channel: usize) -> u16 {
        self.channels[channel - 1]
    }

    pub fn get_input(&self) -> FlightControls {
        FlightControls {
            throttle: self.get_channel_by_no(1),
            aileron: self.get_channel_by_no(2),
            elevator: self.get_channel_by_no(3),
            rudder: self.get_channel_by_no(4),
            arm: self.get_channel_by_no(5) as u16,
            enable: self.get_channel_by_no(6) as u16,
            record: self.get_channel_by_no(7) as u16,
        }
    }
}

impl SbusData {
    pub fn new(data: [u8; 25]) -> Self {
        SbusData { raw: data }
    }

    pub fn parse(&self) -> SbusChannels {
        let mut digital_channels = [false; 8];

        let channels = parse_channels_from_raw(self.raw);

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

fn parse_channels_from_raw(raw: [u8; 25]) -> [u16; 16] {
    let mut channels: [u16; 16] = [0; 16];
    channels[0] = ((raw[1] as u16 | (raw[2] as u16) << 8) & 0x07FF) as u16;
    channels[1] = ((raw[2] as u16 >> 3 | (raw[3] as u16) << 5) & 0x07FF) as u16;
    channels[2] =
        ((raw[3] as u16 >> 6 | (raw[4] as u16) << 2 | (raw[5] as u16) << 10) & 0x07FF) as u16;
    channels[3] = ((raw[5] as u16 >> 1 | (raw[6] as u16) << 7) & 0x07FF) as u16;
    channels[4] = ((raw[6] as u16 >> 4 | (raw[7] as u16) << 4) & 0x07FF) as u16;
    channels[5] =
        ((raw[7] as u16 >> 7 | (raw[8] as u16) << 1 | (raw[9] as u16) << 9) & 0x07FF) as u16;
    channels[6] = ((raw[9] as u16 >> 2 | (raw[10] as u16) << 6) & 0x07FF) as u16;
    channels[7] = ((raw[10] as u16 >> 5 | (raw[11] as u16) << 3) & 0x07FF) as u16;
    channels[8] = ((raw[12] as u16 | (raw[13] as u16) << 8) & 0x07FF) as u16;
    channels[9] = ((raw[13] as u16 >> 3 | (raw[14] as u16) << 5) & 0x07FF) as u16;
    channels[10] =
        ((raw[14] as u16 >> 6 | (raw[15] as u16) << 2 | (raw[16] as u16) << 10) & 0x07FF) as u16;
    channels[11] = ((raw[16] as u16 >> 1 | (raw[17] as u16) << 7) & 0x07FF) as u16;
    channels[12] = ((raw[17] as u16 >> 4 | (raw[18] as u16) << 4) & 0x07FF) as u16;
    channels[13] =
        ((raw[18] as u16 >> 7 | (raw[19] as u16) << 1 | (raw[20] as u16) << 9) & 0x07FF) as u16;
    channels[14] = ((raw[20] as u16 >> 2 | (raw[21] as u16) << 6) & 0x07FF) as u16;
    channels[15] = ((raw[21] as u16 >> 5 | (raw[22] as u16) << 3) & 0x07FF) as u16;

    channels
}

pub fn read_sbus_stream(cx: &mut crate::app::sbus_dma_stream::Context) {
    cx.shared
        .sbus_rx_transfer
        .lock(|transfer: &mut crate::SbusInTransfer| {
            if transfer.is_idle() {
                // Allocate a new buffer
                let new_buf = cx.local.sbus_rx_buffer.take().unwrap();
                // Replace the new buffer with contents from the stream
                let (buffer, _current) = transfer.next_transfer(new_buf).unwrap();

                let control: SbusChannels = SbusData::new(*buffer).parse();
                let control: FlightControls = control.get_input();
                cx.shared.flight_controls.lock(|fc| {
                    *fc = control;
                });

                // Free buffer
                *cx.local.sbus_rx_buffer = Some(buffer);
            }
        });
}
