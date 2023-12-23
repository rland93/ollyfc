use crc32fast::hash;
use log::{debug, error, info, warn};
use rusb::Error;
use rusb::{Device, GlobalContext};
use serialport::SerialPort;

pub const ACK: u8 = 0xAA;
pub const NACK: u8 = 0xAB;
pub const HEADER_LEN: usize = 6;

pub struct FcUsbDevice {
    pub dev: Device<GlobalContext>,
    pub ser: Box<dyn SerialPort>,
}

impl FcUsbDevice {
    pub fn new(mut device: Device<GlobalContext>) -> Result<Self, Error> {
        info!("Creating new FcUsbDevice");
        let port = open_serialport(&mut device)?;

        Ok(Self {
            dev: device,
            ser: port,
        })
    }

    pub fn send(&mut self, data: &[u8]) -> Result<bool, Error> {
        // create a packet
        let mut packet: Vec<u8> = vec![0u8; HEADER_LEN + data.len()];

        // Construct packet header & copy into
        let crc = hash(data) as u32;
        let len = data.len() as u16;
        packet[0..4].copy_from_slice(&crc.to_le_bytes());
        packet[4..HEADER_LEN].copy_from_slice(&len.to_le_bytes());

        debug!("created packet. crc: crc={:x}, len={}", crc, len);

        // copy data into packet
        packet[HEADER_LEN..HEADER_LEN + data.len()].copy_from_slice(data);

        // write the packet
        debug!("sending {} bytes...", packet.len());
        match self.ser.write_all(&packet) {
            Ok(_) => {}
            Err(e) => {
                error!("Error at serial CRC write: {e}");
                return Err(Error::Io);
            }
        }

        // wait for ack
        debug!("wait for receiver ack...");
        let ack = self.wait_for_ack()?;
        debug!("receiver ack={}", ack);

        Ok(ack)
    }

    /// wait for an ack. Expects 1 byte of data to be read.
    fn wait_for_ack(&mut self) -> Result<bool, Error> {
        loop {
            // wait for host to send ack
            if let Ok(ct) = self.ser.bytes_to_read() {
                if ct == 0 {
                    continue;
                } else {
                    break;
                }
            } else {
                error!("Error in bytes_to_read in wait for ack.");
                return Err(Error::Io);
            }
        }

        let mut ack: Vec<u8> = vec![0u8; 1];

        // read
        match self.ser.read_exact(&mut ack) {
            Ok(_) => {}
            Err(e) => {
                error!("Error at serial read: {e}");
                return Err(Error::Io);
            }
        };

        // check value of ack
        if ack[0] == ACK {
            return Ok(true);
        } else if ack[0] == NACK {
            return Ok(false);
        } else {
            error!("Bad bytecode read for ack: {:x?}", ack[0]);
            return Err(Error::Io);
        }
    }

    pub fn recv(&mut self) -> Result<Vec<u8>, Error> {
        debug!("wait to receive packet...");
        let bytes_to_recv: u32;
        loop {
            if let Ok(ct) = self.ser.bytes_to_read() {
                if ct == 0 {
                    continue;
                } else {
                    bytes_to_recv = ct;
                    break;
                }
            } else {
                error!("Error in recv waiting for packet.");
                return Err(Error::Io);
            }
        }

        let mut packet: Vec<u8> = vec![0u8; bytes_to_recv as usize];

        // receive packet
        let recvd = match self.ser.read(&mut packet) {
            Ok(n) => n,
            Err(e) => {
                error!("Error at serial read: {e}");
                return Err(Error::Io);
            }
        };
        debug!("read {} bytes", recvd);

        // unpack header
        let crc = u32::from_le_bytes(packet[0..4].try_into().unwrap());
        let len = u16::from_le_bytes(packet[4..HEADER_LEN].try_into().unwrap());

        // validate CRC
        let valid = crc == hash(&packet[HEADER_LEN..HEADER_LEN + len as usize]);
        debug!("crc: crc={:x}, len={} -- valid={}", crc, len, valid);
        // send ack
        debug!("sending ack...");
        self.send_ack(valid)?;

        let recvd_data = packet[HEADER_LEN..HEADER_LEN + len as usize].to_vec();
        debug!("received {} bytes", len);
        return Ok(recvd_data);
    }

    fn send_ack(&mut self, valid: bool) -> Result<(), Error> {
        // send ack
        match self.ser.write_request_to_send(true) {
            Ok(_) => {}
            Err(e) => {
                error!("Error at serial RTS: {e}");
                return Err(Error::Io);
            }
        }

        match self.ser.write(&[if valid { ACK } else { NACK }]) {
            Ok(_) => {}
            Err(e) => {
                error!("Error at serial write: {e}");
                return Err(Error::Io);
            }
        }

        Ok(())
    }
}

/// Search for COM ports matching the device.
fn find_device_com_port(vid: u16, pid: u16) -> Option<String> {
    info!(
        "Attempting to find COM Port for device {:04x}:{:04x}",
        vid, pid
    );
    match serialport::available_ports() {
        Ok(ports) => {
            for port in ports {
                info!("Found port: {}", port.port_name);
                if let serialport::SerialPortType::UsbPort(info) = port.port_type {
                    info!("Found USB port: {:04x}:{:04x}", info.vid, info.pid);
                    if info.vid == vid && info.pid == pid {
                        info!("Found COM port for {vid}:{pid} at: {}", port.port_name);
                        return Some(port.port_name);
                    }
                }
            }
            None
        }
        Err(_) => None,
    }
}

/// open serialport of the device
fn open_serialport(device: &mut Device<GlobalContext>) -> Result<Box<dyn SerialPort>, Error> {
    // open descriptor
    let desc = match device.device_descriptor() {
        Ok(desc) => desc,
        Err(e) => {
            error!("Error getting device descriptor: {}", e);
            return Err(Error::NotFound);
        }
    };

    // match vid, pid
    let vid: u16;
    let pid: u16;
    if desc.vendor_id() == 0x1209 && desc.product_id() == 0x6EF1 {
        info!("Found FC device");
        vid = desc.vendor_id();
        pid = desc.product_id();
    } else {
        error!("Attempted to create serialport on non-FC device");
        return Err(Error::NotFound);
    }

    if let Some(port_name) = find_device_com_port(vid, pid) {
        info!("Opening COM port...");
        match serialport::new(&port_name, 9600).open() {
            Ok(port) => {
                info!("Opened COM port!");
                return Ok(port);
            }
            Err(e) => {
                warn!("Failed to open serial port: {}", e);
                return Err(Error::NotFound);
            }
        }
    } else {
        warn!("Device not found");
        return Err(Error::NotFound);
    }
}
