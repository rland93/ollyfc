/// Data protocol for serial-over-USB
///
/// For example, a message from device->host:
///
/// Device -> Host
/// - device sends header + packet
/// - device waits for ack/nack
///
/// Host -> Device
/// - host receives header + packet
/// - host validates crc
/// - host sends ack/nack
///
use crc32fast::hash;
use defmt::{debug, error, info, warn};
use rtic_monotonics::systick::{ExtU32, Systick};
use stm32f4xx_hal::otg_fs::UsbBusType;
use usb_device::device::UsbDevice;
use usb_device::UsbError;

pub struct Xfer {
    pub dev: UsbDevice<'static, UsbBusType>,
    pub ser: usbd_serial::SerialPort<'static, UsbBusType>,
    tx_buf: [u8; PACKET_SIZE],
}

// Delay that we wait for RTS
pub const RTS_DELAY: u32 = 1;
pub const ACK: u8 = 0xAA;
pub const NACK: u8 = 0xAB;
pub const HEADER_LEN: usize = 6;
pub const PACKET_SIZE: usize = 262;

impl Xfer {
    pub fn new(
        dev: UsbDevice<'static, UsbBusType>,
        ser: usbd_serial::SerialPort<'static, UsbBusType>,
    ) -> Self {
        Self {
            dev,
            ser,
            tx_buf: [0u8; PACKET_SIZE],
        }
    }

    pub async fn send(&mut self, data: &[u8]) -> Result<bool, UsbError> {
        // raise runtime error of data is too large
        if data.len() > self.tx_buf.len() - HEADER_LEN {
            error!(
                "Data too large to send {} > {}",
                data.len(),
                self.tx_buf.len() - HEADER_LEN
            );
            return Err(UsbError::BufferOverflow);
        }

        // Construct packet header
        let crc = hash(data) as u32;
        let len = data.len() as u16;

        // copy into send buffer
        self.tx_buf[0..4].copy_from_slice(&crc.to_le_bytes());
        self.tx_buf[4..HEADER_LEN].copy_from_slice(&len.to_le_bytes());
        self.tx_buf[HEADER_LEN..HEADER_LEN + len as usize].copy_from_slice(&data);

        debug!("created packet. crc: crc={:x}, len={}", crc, len);

        // wait for ready to send
        while !self.ser.rts() {
            Systick::delay(RTS_DELAY.millis()).await;
        }

        // send packet
        let mut nbytes = 0;
        let end = len as usize + HEADER_LEN;

        debug!("sending {} bytes...", end);
        while nbytes < end {
            nbytes += match self.ser.write(&self.tx_buf[nbytes..end]) {
                Ok(n) => n,
                Err(e) => match e {
                    UsbError::WouldBlock => 0,
                    _ => {
                        warn!("Error at serial header write.");
                        return Err(e);
                    }
                },
            };
        }

        // wait for ack
        debug!("wait for receiver ack...");
        let ack = self.wait_for_ack().await?;
        debug!("receiver ack={}", ack);
        return Ok(ack);
    }

    async fn wait_for_ack(&mut self) -> Result<bool, UsbError> {
        let mut ack = [0u8; 1];
        loop {
            // wait for host to send ack
            if self.dev.poll(&mut [&mut self.ser]) {
                if let Ok(ct) = self.ser.read(&mut ack) {
                    if ct == 0 {
                        continue;
                    } else if ct > 1 {
                        warn!("Invalid ack byte count: {}", ct);
                        return Err(UsbError::BufferOverflow);
                    } else {
                        // validate the ack
                        if ack[0] == ACK {
                            return Ok(true);
                        } else if ack[0] == NACK {
                            return Ok(false);
                        } else {
                            warn!("Invalid ack byte: {}", ack[0]);
                            return Err(UsbError::InvalidState);
                        }
                    }
                } else {
                    // continue waiting for data to be sent
                    Systick::delay(1u32.micros()).await;
                    continue;
                }
            }
        }
    }

    /// receive bytes over serial into a buffer and return the number of bytes
    /// received. The receive buffer must be large enough to contain the number
    /// of bytes received.
    async fn recv(&mut self, rx_buf: &mut [u8]) -> Result<usize, UsbError> {
        loop {
            if self.dev.poll(&mut [&mut self.ser]) {
                match self.ser.read(rx_buf) {
                    Ok(n) => {
                        // read more bytes than can fit into 1 packet.
                        if n > rx_buf.len() {
                            error!(
                                "Read of {} into {} would cause buffer overflow!",
                                n,
                                rx_buf.len()
                            );
                            return Err(UsbError::BufferOverflow);
                        }
                        // empty read
                        else if n == 0 {
                            warn!("Empty read");
                            continue;
                        }
                        // read
                        else {
                            return Ok(n);
                        }
                    }
                    Err(e) => match e {
                        UsbError::WouldBlock => {
                            continue;
                        }
                        _ => {
                            error!("Error at serial read.");
                            return Err(e);
                        }
                    },
                }
            }
            Systick::delay(1u32.micros()).await;
        }
    }

    /// receive a packet from the host, into a buffer owned by the caller of
    /// this function. The caller must ensure that the buffer is large enough
    /// to contain the packet received, plus 6 bytes of header.
    pub async fn receive(&mut self, rx_buf: &mut [u8]) -> Result<usize, UsbError> {
        // min. length check
        if rx_buf.len() < HEADER_LEN {
            error!("Receive buffer too small");
            return Err(UsbError::BufferOverflow);
        }

        // receive message
        debug!("wait to receive packet...");
        let nbytes = self.recv(rx_buf).await?;
        debug!("read {} bytes", nbytes);
        if nbytes < HEADER_LEN {
            error!("Received less than header length");
            return Err(UsbError::InvalidState);
        };

        // unpack header
        let crc = u32::from_le_bytes(rx_buf[0..4].try_into().unwrap());
        let len = u16::from_le_bytes(rx_buf[4..HEADER_LEN].try_into().unwrap());

        // validate CRC
        let data = &rx_buf[HEADER_LEN..HEADER_LEN + len as usize];
        let valid = crc == hash(data);
        debug!("crc: crc={:x}, len={} -- valid={}", crc, len, valid);

        // wait until we can send
        while !self.ser.rts() {
            Systick::delay(1u32.micros()).await;
        }

        // send either ack or nack
        let ack: [u8; 1];
        if valid {
            ack = [ACK];
        } else {
            ack = [NACK];
        }

        // write ack/nack
        debug!("sending ack...");
        match self.ser.write(&ack) {
            Ok(n) => {
                if n != 1 {
                    error!("Ack write failed, wrote {} bytes", n);
                    return Err(UsbError::InvalidState);
                }
            }
            Err(e) => {
                error!("Ack write failed");
                return Err(e);
            }
        }

        // send out ref to the data
        debug!("received {} bytes", len);
        Ok(len as usize)
    }
}
