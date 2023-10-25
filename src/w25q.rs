#![allow(dead_code)]

use defmt::Format;
use stm32f4xx_hal::{
    gpio::{Output, Pin, PushPull},
    pac::{SPI1, TIM3},
    prelude::*,
    spi::{Error as SpiE, Spi},
    timer::Delay,
};

#[repr(u8)]
enum Command {
    PageProgram = 0x02,
    ReadData = 0x03,
    FastRead = 0x0B,
    ReadStatusRegister1 = 0x05,
    ReadStatusRegister2 = 0x35,
    ReadStatusRegister3 = 0x15,
    WriteStatusRegister1 = 0x01,
    WriteStatusRegister2 = 0x31,
    WriteStatusRegister3 = 0x11,
    WriteEnable = 0x06,
    SectorErase = 0x20,
    UniqueId = 0x4B,
    Block32Erase = 0x52,
    Block64Erase = 0xD8,
    ChipErase = 0xC7,
    EnableReset = 0x66,
    Reset = 0x99,
}
pub const PAGE_SIZE: u32 = 256;
pub const N_PAGES: u32 = 16384;
pub const CAPACITY: u32 = PAGE_SIZE * N_PAGES;
pub const SECTOR_SIZE: u32 = PAGE_SIZE * 16;
pub const N_SECTORS: u32 = N_PAGES / 16;
pub const BLOCK_32K_SIZE: u32 = SECTOR_SIZE * 8;
pub const N_BLOCKS_32K: u32 = N_SECTORS / 8;
pub const BLOCK_64K_SIZE: u32 = BLOCK_32K_SIZE * 2;
pub const N_BLOCKS_64K: u32 = N_BLOCKS_32K / 2;

pub const DELAY: u32 = 10;

pub const SECTOR_ERASE_DELAY: u32 = 50; // ms
pub const BLOCK32_ERASE_DELAY: u32 = 120; // ms
pub const BLOCK64_ERASE_DELAY: u32 = 150; // ms
pub const CHIP_ERASE_DELAY: u32 = 2000; // ms

#[derive(Debug, PartialEq, Format)]
pub enum SR {
    SR1(SR1),
    SR2(SR2),
    SR3(SR3),
}

#[derive(Debug, PartialEq, Default, Format)]
pub struct SR1 {
    pub srp0: bool,
    pub sec: bool,
    pub tb: bool,
    pub bp: u8,
    pub wel: bool,
    pub busy: bool,
}

impl SR1 {
    fn from_byte(byte: u8) -> Self {
        Self {
            srp0: (byte & 0b1000_0000) != 0,
            sec: (byte & 0b0100_0000) != 0,
            tb: (byte & 0b0010_0000) != 0,
            bp: (byte & 0b0001_1100) >> 2u8,
            wel: (byte & 0b0000_0010) != 0,
            busy: (byte & 0b0000_0001) != 0,
        }
    }
    fn to_byte(&self) -> u8 {
        (self.srp0 as u8) << 7u8
            | (self.sec as u8) << 6u8
            | (self.tb as u8) << 5u8
            | (self.bp & 0b111) << 2u8
            | (self.wel as u8) << 1u8
            | self.busy as u8
    }
}

#[derive(Debug, PartialEq, Default, Format)]
pub struct SR2 {
    pub sus: bool,
    pub cmp: bool,
    pub lb: u8,
    pub qe: bool,
    pub srp1: bool,
}
impl SR2 {
    fn from_byte(byte: u8) -> Self {
        Self {
            sus: byte & 0b1000_0000 != 0,
            cmp: byte & 0b0100_0000 != 0,
            lb: (byte >> 5u8) & 0b111,
            qe: byte & 0b0000_0010 != 0,
            srp1: byte & 0b0000_0001 != 0,
        }
    }

    fn to_byte(&self) -> u8 {
        (self.sus as u8) << 7u8
            | (self.cmp as u8) << 6u8
            | (self.lb & 0b111) << 5u8
            | (self.qe as u8) << 1u8
            | self.srp1 as u8
    }
}

#[derive(Debug, PartialEq, Default, Format)]
pub struct SR3 {
    pub hold_or_reset: bool,
    pub driver_strength: u8,
    pub wps: bool,
}
impl SR3 {
    fn from_byte(byte: u8) -> Self {
        Self {
            hold_or_reset: byte & 0b1000_0000 != 0,
            driver_strength: (byte >> 5) & 0b11, // Get S22 and S21
            wps: byte & 0b0000_0100 != 0,
        }
    }

    fn to_byte(&self) -> u8 {
        (self.hold_or_reset as u8) << 7 | (self.driver_strength & 0b11) << 5 | (self.wps as u8) << 2
    }
}

#[derive(Debug, Format)]
pub enum MemError<SpiE> {
    SpiError(SpiE),
    NotAlignedError,
    OutOfBoundsError,
}
impl From<SpiE> for MemError<SpiE> {
    fn from(e: SpiE) -> Self {
        Self::SpiError(e)
    }
}

pub trait FlashMem {
    fn page_size(&self) -> u32;
    fn read(&mut self, address: u32, buf: &mut [u8]) -> Result<(), MemError<SpiE>>;
    fn page_program(&mut self, address: u32, buf: &[u8; 256]) -> Result<(), MemError<SpiE>>;
    fn sector_erase(&mut self, address: u32) -> Result<(), MemError<SpiE>>;
    fn block32_erase(&mut self, address: u32) -> Result<(), MemError<SpiE>>;
    fn block64_erase(&mut self, address: u32) -> Result<(), MemError<SpiE>>;
    fn chip_erase(&mut self) -> Result<(), MemError<SpiE>>;
}

pub struct W25Q {
    spi: Spi<SPI1>,
    cs: Pin<'A', 4, Output<PushPull>>,
    pub timer: Delay<TIM3, 1000000>,
}

impl W25Q {
    pub fn new(spi: Spi<SPI1>, cs: Pin<'A', 4, Output>, timer: Delay<TIM3, 1000000>) -> Self {
        Self { spi, cs, timer }
    }

    pub fn page_size(&self) -> u32 {
        PAGE_SIZE
    }

    pub fn wait_busy(&mut self) -> Result<(), MemError<SpiE>> {
        let mut buf = [0u8; 1];
        self.timer.delay_us(5u32);
        loop {
            self.cs.set_low();
            self.spi.write(&[Command::ReadStatusRegister1 as u8])?;
            self.spi.read(&mut buf)?;
            self.cs.set_high();
            if buf[0] & 0b0000_0001 == 0 {
                break;
            }
            self.timer.delay_us(20u32);
        }
        Ok(())
    }

    fn cmd_and_address(&self, command: Command, address: u32) -> [u8; 4] {
        [
            command as u8,
            ((address & 0xFF0000) >> 16) as u8,
            ((address & 0x00FF00) >> 8) as u8,
            ((address & 0x0000FF) >> 0) as u8,
        ]
    }

    pub fn unique_id(&mut self) -> Result<(u32, u32), MemError<SpiE>> {
        self.cs.set_low();
        self.spi.write(&[Command::UniqueId as u8])?;
        for _ in 0..4 {
            self.spi.write(&[0xFF])?;
        }
        let mut buf = [0u8; 8];
        self.spi.read(&mut buf)?;
        self.cs.set_high();

        return Ok((
            u32::from_be_bytes([buf[0], buf[1], buf[2], buf[3]]),
            u32::from_be_bytes([buf[4], buf[5], buf[6], buf[7]]),
        ));
    }

    pub fn read_status_register1(&mut self) -> Result<SR, MemError<SpiE>> {
        self.cs.set_low();
        self.spi.write(&[Command::ReadStatusRegister1 as u8])?;
        let mut buf = [0u8; 3];
        self.spi.read(&mut buf)?;
        self.cs.set_high();

        Ok(SR::SR1(SR1::from_byte(buf[0])))
    }

    pub fn can_write(&mut self) -> Result<bool, MemError<SpiE>> {
        let sr = self.read_status_register1()?;
        Ok(sr
            == SR::SR1(SR1 {
                wel: true,
                ..Default::default()
            }))
    }

    pub fn write_enable(&mut self) -> Result<(), MemError<SpiE>> {
        self.cs.set_low();
        self.spi.write(&[Command::WriteEnable as u8])?;
        self.cs.set_high();

        self.wait_busy()?;

        Ok(())
    }

    pub fn read(&mut self, address: u32, buf: &mut [u8]) -> Result<(), MemError<SpiE>> {
        self.cs.set_low();
        self.spi
            .write(&self.cmd_and_address(Command::ReadData, address))?;
        self.spi.read(buf)?;
        self.cs.set_high();

        self.wait_busy()?;

        Ok(())
    }

    pub fn page_program(&mut self, address: u32, buf: &[u8; 256]) -> Result<(), MemError<SpiE>> {
        // check alignment (256 bytes)
        if 0x000000 != (address & 0x0000FF) {
            return Err(MemError::NotAlignedError);
        }

        if !self.can_write()? {
            self.write_enable()?;
        } else {
            self.wait_busy()?;
        }

        self.cs.set_low();
        self.spi
            .write(&self.cmd_and_address(Command::PageProgram, address))?;
        self.spi.write(buf)?;
        self.cs.set_high();

        self.wait_busy()?;
        Ok(())
    }

    pub fn sector_erase(&mut self, address: u32) -> Result<(), MemError<SpiE>> {
        // check address alignment (4kb)
        if 0x000000 != (address & 0x000FFF) {
            return Err(MemError::NotAlignedError);
        }

        if !self.can_write()? {
            self.write_enable()?;
        } else {
            self.wait_busy()?;
        }

        self.cs.set_low();
        self.spi
            .write(&self.cmd_and_address(Command::SectorErase, address))?;
        self.cs.set_high();

        self.timer.delay_ms(SECTOR_ERASE_DELAY);

        self.wait_busy()?;
        Ok(())
    }

    pub fn block32_erase(&mut self, address: u32) -> Result<(), MemError<SpiE>> {
        // check address alignment (32kb)
        if 0x000000 != (address & 0x007FFF) {
            return Err(MemError::NotAlignedError);
        }

        if !self.can_write()? {
            self.write_enable()?;
        } else {
            self.wait_busy()?;
        }

        self.cs.set_low();
        self.spi
            .write(&self.cmd_and_address(Command::Block32Erase, address))?;
        self.cs.set_high();

        self.timer.delay_ms(BLOCK32_ERASE_DELAY);

        self.wait_busy()?;
        Ok(())
    }

    pub fn block64_erase(&mut self, address: u32) -> Result<(), MemError<SpiE>> {
        // check address alignment (64kb)
        if 0x000000 != (address & 0x00FFFF) {
            return Err(MemError::NotAlignedError);
        }

        if !self.can_write()? {
            self.write_enable()?;
        } else {
            self.wait_busy()?;
        }

        self.cs.set_low();
        self.spi
            .write(&self.cmd_and_address(Command::Block64Erase, address))?;
        self.cs.set_high();

        self.timer.delay_ms(BLOCK64_ERASE_DELAY);

        self.wait_busy()?;
        Ok(())
    }

    pub fn chip_erase(&mut self) -> Result<(), MemError<SpiE>> {
        if !self.can_write()? {
            self.write_enable()?;
        } else {
            self.wait_busy()?;
        }

        self.cs.set_low();
        self.spi.write(&[Command::ChipErase as u8])?;
        self.cs.set_high();

        self.timer.delay_ms(CHIP_ERASE_DELAY);

        self.wait_busy()?;
        Ok(())
    }
}

impl FlashMem for W25Q {
    fn page_size(&self) -> u32 {
        self.page_size()
    }

    fn read(&mut self, address: u32, buf: &mut [u8]) -> Result<(), MemError<SpiE>> {
        self.read(address, buf)
    }

    fn page_program(&mut self, address: u32, buf: &[u8; 256]) -> Result<(), MemError<SpiE>> {
        self.page_program(address, buf)
    }

    fn sector_erase(&mut self, address: u32) -> Result<(), MemError<SpiE>> {
        self.sector_erase(address)
    }

    fn block32_erase(&mut self, address: u32) -> Result<(), MemError<SpiE>> {
        self.block32_erase(address)
    }

    fn block64_erase(&mut self, address: u32) -> Result<(), MemError<SpiE>> {
        self.block64_erase(address)
    }

    fn chip_erase(&mut self) -> Result<(), MemError<SpiE>> {
        self.chip_erase()
    }
}
