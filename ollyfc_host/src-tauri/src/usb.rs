use log::{info, warn};
use ollyfc_common::cmd::{Command, BAUD_RATE};
use rusb::{Device, DeviceDescriptor, GlobalContext};
use serialport::SerialPort;
use serialport::{available_ports, SerialPortType};
use tauri::State;

// from crate
use crate::UsbDeviceState;

/******************************************************************************/

pub struct FcUsbDevice {
    pub device: Device<GlobalContext>,
    pub desc: DeviceDescriptor,
    pub serial_port: Option<Box<dyn SerialPort>>,
}
impl FcUsbDevice {
    pub fn new(device: Device<GlobalContext>, desc: DeviceDescriptor) -> Self {
        info!("Creating new FcUsbDevice");
        let mut dev = FcUsbDevice {
            device,
            desc,
            serial_port: None,
        };

        // open serialport if we can
        if let Some(sp) = connect_to_com_port(dev.desc.vendor_id(), dev.desc.product_id()) {
            dev.serial_port = Some(sp);
        }

        dev
    }
}

/******************************************************************************/

#[tauri::command]
pub fn send_usb_command(usb_state: State<UsbDeviceState>, cmd: &str) -> Result<(), String> {
    let mut usb_device = usb_state.0.lock().unwrap();
    info!("Sending USB command: {}", cmd);
    if let Some(dev) = usb_device.as_mut() {
        match Command::from_str(cmd) {
            Some(cmd) => match cmd {
                Command::Acknowledge => {
                    info!("Sending acknowledge");
                    if let Some(port) = &mut dev.serial_port {
                        let cmd = Command::Acknowledge.to_byte();
                        port.write(&[cmd]).unwrap();
                        port.flush().unwrap();
                        Ok(())
                    } else {
                        Err("No serial port".to_string())
                    }
                }
                _ => Err("Invalid command".to_string()),
            },
            _ => Err("Invalid command data".to_string()),
        }
    } else {
        Err("No USB device connected".to_string())
    }
}

#[tauri::command]
pub fn search_for_usb(usb_state: State<UsbDeviceState>) -> Result<bool, String> {
    info!("Searching for USB device...");
    match find_fc() {
        Some(usb_dev) => {
            info!("Found device.");
            let mut usb_device = (*usb_state).0.lock().unwrap();
            *usb_device = Some(usb_dev);
            Ok(true)
        }
        None => {
            info!("No device found.");
            Ok(false)
        }
    }
}

/******************************************************************************/

fn find_fc() -> Option<FcUsbDevice> {
    match rusb::devices() {
        Ok(device_list) => {
            for device in device_list.iter() {
                let desc = device.device_descriptor().expect("Couldn't get descriptor");
                info!(
                    "Found vid: pid: {:04x}:{:04x}",
                    desc.vendor_id(),
                    desc.product_id()
                );
                if desc.vendor_id() == 0x1209 && desc.product_id() == 0x6EF1 {
                    info!("Found FC device");
                    return Some(FcUsbDevice::new(device, desc));
                }
            }
        }
        Err(e) => eprintln!("Error obtaining USB devices: {}", e),
    }
    None
}

/// Search COM port for device.
fn find_device_com_port(vid: u16, pid: u16) -> Option<String> {
    info!(
        "Attempting to find COM Port for device {:04x}:{:04x}",
        vid, pid
    );
    match available_ports() {
        Ok(ports) => {
            for port in ports {
                info!("Found port: {}", port.port_name);
                if let SerialPortType::UsbPort(info) = port.port_type {
                    info!("Found USB port: {:04x}:{:04x}", info.vid, info.pid);
                    if info.vid == vid && info.pid == pid {
                        // found
                        info!("Found COM port. It's at: {}", port.port_name);
                        return Some(port.port_name);
                    }
                }
            }
            None
        }
        Err(_) => None,
    }
}

/// Connect to com port
fn connect_to_com_port(vid: u16, pid: u16) -> Option<Box<dyn serialport::SerialPort>> {
    info!(
        "Attempting to connect to COM port for device {:04x}:{:04x}",
        vid, pid
    );
    if let Some(port_name) = find_device_com_port(vid, pid) {
        info!("Opening COM port...");
        match serialport::new(&port_name, 9600).open() {
            Ok(port) => {
                info!("Opened COM port!");
                Some(port)
            }
            Err(e) => {
                warn!("Failed to open serial port: {}", e);
                None
            }
        }
    } else {
        warn!("Device not found");
        None
    }
}
