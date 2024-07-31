#[cfg(feature = "std")]
use serde::{Deserialize, Serialize};

use core::fmt::Display;
pub const PACKET_SIZE: usize = 80;
pub const BAUD_RATE: usize = 115200;

#[cfg_attr(feature = "no_std", derive(defmt::Format))]
#[cfg_attr(feature = "std", derive(Serialize, Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum Command {
    Default,
    Acknowledge,         // acknowledge
    GetFlashDataInfo,    // get flash data info
    GetLogData,          // get flash data
    GetLogDataTerminate, // terminate a get flash data command
    GetFlashDataByBlock, // get flash data by block
    EraseFlash,          // erase the entire flash region
    Invalid,             // invalid command
    BadFlashRead,        // abort an operation in progress on the device.
}

impl Display for Command {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let cmdstr = match self {
            Command::Default => "default",
            Command::Acknowledge => "ack",
            Command::GetFlashDataInfo => "getflashinfo",
            Command::GetLogData => "getflash",
            Command::GetLogDataTerminate => "getflashterm",
            Command::GetFlashDataByBlock => "getbyblock",
            Command::EraseFlash => "erase",
            Command::Invalid => "invalid",
            Command::BadFlashRead => "abort",
        };
        write!(f, "{}", cmdstr)
    }
}

impl Command {
    pub fn default() -> Self {
        Command::Default
    }

    pub fn from_byte(byte: u8) -> Self {
        match byte {
            0x00 => Command::Default,
            0x06 => Command::Acknowledge,
            0x07 => Command::GetFlashDataInfo,
            0x08 => Command::GetLogData,
            0xC8 => Command::GetLogDataTerminate,
            0x09 => Command::GetFlashDataByBlock,
            0x0A => Command::EraseFlash,
            0xFF => Command::Invalid,
            0xA6 => Command::BadFlashRead,
            _ => Command::Invalid,
        }
    }
    pub fn to_byte(&self) -> u8 {
        match self {
            Command::Default => 0x00,
            Command::Acknowledge => 0x06,
            Command::GetFlashDataInfo => 0x07,
            Command::GetLogData => 0x08,
            Command::GetLogDataTerminate => 0xC8,
            Command::GetFlashDataByBlock => 0x09,
            Command::EraseFlash => 0x0A,
            Command::Invalid => 0x00,
            Command::BadFlashRead => 0xA6,
        }
    }
    pub fn from_str(cmdstr: &str) -> Option<Self> {
        match cmdstr {
            "default" => Some(Command::Default),
            "ack" => Some(Command::Acknowledge),
            "getflashinfo" => Some(Command::GetFlashDataInfo),
            "getflash" => Some(Command::GetLogData),
            "getflashterm" => Some(Command::GetLogDataTerminate),
            "getbyblock" => Some(Command::GetFlashDataByBlock),
            "erase" => Some(Command::EraseFlash),
            "invalid" => Some(Command::Invalid),
            "abort" => Some(Command::BadFlashRead),
            _ => None,
        }
    }
    pub fn to_str(&self) -> &'static str {
        match self {
            Command::Default => "default",
            Command::Acknowledge => "ack",
            Command::GetFlashDataInfo => "getflashinfo",
            Command::GetLogData => "getflash",
            Command::GetLogDataTerminate => "getflashterm",
            Command::GetFlashDataByBlock => "getbyblock",
            Command::EraseFlash => "erase",
            Command::Invalid => "invalid",
            Command::BadFlashRead => "abort",
        }
    }
}
