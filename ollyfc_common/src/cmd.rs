pub const PACKET_SIZE: usize = 80;
pub const BAUD_RATE: usize = 115200;

#[cfg_attr(feature = "use_defmt", derive(defmt::Format))]
#[cfg_attr(not(feature = "use_defmt"), derive(Debug, Clone, Copy, PartialEq))]

pub enum Command {
    Acknowledge,
    GetFlashData,
    GetFlashDataByBlock,
    EraseFlash,
    Invalid,
}

impl Command {
    pub fn from_byte(byte: u8) -> Self {
        match byte {
            0x06 => Command::Acknowledge,
            0x07 => Command::GetFlashData,
            0x08 => Command::GetFlashDataByBlock,
            0x09 => Command::EraseFlash,
            0x00 => Command::Invalid,
            _ => Command::Invalid,
        }
    }
    pub fn to_byte(&self) -> u8 {
        match self {
            Command::Acknowledge => 0x06,
            Command::GetFlashData => 0x07,
            Command::GetFlashDataByBlock => 0x08,
            Command::EraseFlash => 0x09,
            Command::Invalid => 0x00,
        }
    }
    pub fn from_str(cmdstr: &str) -> Option<Self> {
        match cmdstr {
            "ack" => Some(Command::Acknowledge),
            "getflash" => Some(Command::GetFlashData),
            "getbyblock" => Some(Command::GetFlashDataByBlock),
            "erase" => Some(Command::EraseFlash),
            "invalid" => Some(Command::Invalid),
            _ => None,
        }
    }
    pub fn to_str(&self) -> &'static str {
        match self {
            Command::Acknowledge => "ack",
            Command::GetFlashData => "getflash",
            Command::GetFlashDataByBlock => "getbyblock",
            Command::EraseFlash => "erase",
            Command::Invalid => "invalid",
        }
    }
}
