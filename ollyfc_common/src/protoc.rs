/// Data protocol for serial-over-USB
/// 
/// Device -> Host
/// - device sends packet data
/// - device sends 32 bit CRC
/// - device waits for ack
/// 
/// Host -> Device
/// - host listens for serial in
/// - host recieves serial bytes
/// - moves received into buffer
/// - checks crc bytes
/// - send ack

#![cfg_attr(feature = "no_std", no_std)]

// both include
use crc32fast::hash;

// host-side include
#[cfg(feature = "std")]
use serde::{Deserialize, Serialize};

// device-side include
#[cfg(feature = "no_std")]
use usb_device::device::UsbDevice;
#[cfg(feature = "no_std")]
stm32f4xx_hal::otg_fs::UsbBusType;




// device-side send
fn send(
  ser: usbd_serial::SerialPort<'static, UsbBusType>,
  dev: UsbDevice<'static, UsbBusType>
) {

}



Command::GetLogData => {
  info!("Received get log data");
  let mut ct: u32 = 0;
  let mut data: [u8; LOG_SIZE] = [0u8; LOG_SIZE];

  let info = cx.shared.logger.lock(|logger| logger.read_info_page());

  // send out the total number of logs that will be gotten
  let total: u32 = info.n_logs_in_region();
  while !ser.rts() {
      Systick::delay(1u32.millis()).await;
  }
  info!("sending total...");
  match ser.write(&total.to_le_bytes()) {
      Ok(n) => {
          info!("Sent {} bytes", total);
      }
      Err(e) => {
          error!("Write buffer is full");
          continue;
      }
  }

  // wait for the acknowledge
  debug!("wait for ack.");
  let mut ack = [0u8; 1];
  loop {
      let mut n = 0;

      // poll for usb event -- ack or nack
      if dev.poll(&mut [ser]) {
          // events
          if let Ok(ct) = ser.read(&mut ack) {
              n = ct;
          } else {
              n = 0;
          }
      }

      // continue if no data
      if n != 1 {
          continue;
      }

      // check ack
      if ack[0] == 0xAA {
          debug!("ackd");
          break;
      } else {
          error!("nackd");
      }
  }

  // now loop through the logs and send

  while ct < total {
      debug!("{}: {}/{}", Systick::now().ticks(), ct, total);

      // read the flash page corresponding to the current log
      match cx.shared.logger.lock(|logger| {
          let addr = logger.block_start_ptr() + ct * LOG_SIZE as u32;
          logger.mem.read(addr, &mut data)?;
          Ok(())
      }) {
          Ok(_) => (),
          Err(e) => match e {
              MemError::SpiError(_) => error!("SPI error"),
              MemError::NotAlignedError => error!("Page not aligned"),
              MemError::OutOfBoundsError => error!("Data out of bounds"),
          },
      }
      // "data" now has the log in it. it's length LOG_SIZE.

      // delay until serial line is ready to send.
      while !ser.rts() {
          Systick::delay(1u32.millis()).await;
      }

      debug!("sending packet");
      // send the packet.
      let nbytes = match ser.write(&data) {
          Ok(n) => n,
          Err(e) => {
              error!("Write buffer is full");
              continue;
          }
      };

      debug!("sent packet");

      // wait for the acknowledge
      debug!("wait for ack.");
      let mut ack = [0u8; 1];
      loop {
          let mut n = 0;

          // poll for usb event -- ack or nack
          if dev.poll(&mut [ser]) {
              // events
              if let Ok(ct) = ser.read(&mut ack) {
                  n = ct;
              } else {
                  n = 0;
              }
          }

          // continue if no data
          if n != 1 {
              continue;
          }

          // check ack
          if ack[0] == 0xAA {
              debug!("ackd");
              break;
          } else {
              error!("nackd");
          }
      }

      // ready to send next packet
      ct += 1;
  }

  // done with all packets, send eof
  while !ser.rts() {
      Systick::delay(1u32.millis()).await;
  }
  let eof = [0x04u8; 2];
  let nbytes = match ser.write(&eof) {
      Ok(n) => {
          if n != 2 {
              error!("EOF not sent");
          } else {
              debug!("EOF sent");
          }
      }
      Err(e) => {
          error!("Write buffer is full");
          continue;
      }
  };

  // eof sent.
}