use core::fmt::Display;

pub const PACKET_SIZE: usize = 80;
pub const BAUD_RATE: usize = 115200;

#[cfg_attr(feature = "use_defmt", derive(defmt::Format))]
#[cfg_attr(not(feature = "use_defmt"), derive(Debug, Clone, Copy, PartialEq))]
pub enum Command {
    Acknowledge,
    GetFlashDataInfo,
    GetLogData,
    GetFlashDataByBlock,
    EraseFlash,
    Invalid,
}

impl Display for Command {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let cmdstr = match self {
            Command::Acknowledge => "ack",
            Command::GetFlashDataInfo => "getflashinfo",
            Command::GetLogData => "getflash",
            Command::GetFlashDataByBlock => "getbyblock",
            Command::EraseFlash => "erase",
            Command::Invalid => "invalid",
        };
        write!(f, "{}", cmdstr)
    }
}

impl Command {
    pub fn from_byte(byte: u8) -> Self {
        match byte {
            0x06 => Command::Acknowledge,
            0x07 => Command::GetFlashDataInfo,
            0x08 => Command::GetLogData,
            0x09 => Command::GetFlashDataByBlock,
            0x0A => Command::EraseFlash,
            0x00 => Command::Invalid,
            _ => Command::Invalid,
        }
    }
    pub fn to_byte(&self) -> u8 {
        match self {
            Command::Acknowledge => 0x06,
            Command::GetFlashDataInfo => 0x07,
            Command::GetLogData => 0x08,
            Command::GetFlashDataByBlock => 0x09,
            Command::EraseFlash => 0x0A,
            Command::Invalid => 0x00,
        }
    }
    pub fn from_str(cmdstr: &str) -> Option<Self> {
        match cmdstr {
            "ack" => Some(Command::Acknowledge),
            "getflashinfo" => Some(Command::GetFlashDataInfo),
            "getflash" => Some(Command::GetLogData),
            "getbyblock" => Some(Command::GetFlashDataByBlock),
            "erase" => Some(Command::EraseFlash),
            "invalid" => Some(Command::Invalid),
            _ => None,
        }
    }
    pub fn to_str(&self) -> &'static str {
        match self {
            Command::Acknowledge => "ack",
            Command::GetFlashDataInfo => "getflashinfo",
            Command::GetLogData => "getflash",
            Command::GetFlashDataByBlock => "getbyblock",
            Command::EraseFlash => "erase",
            Command::Invalid => "invalid",
        }
    }
}
