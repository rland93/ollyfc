// lib.rs

use log::{debug, error, info, warn};
use ollyfc_common::msg::{Message, MessageHeader, MAX_PAYLOAD_SIZE};
use ollyfc_common::{USB_PID, USB_VID};
use once_cell::sync::Lazy;
use rusb::{Context, Device, DeviceHandle, UsbContext};
use serde::Serialize;
use std::sync::Mutex;
use std::time::Duration;
const MAX_MSG_SIZE: usize = size_of::<MessageHeader>() + MAX_PAYLOAD_SIZE;

pub struct UsbDevice {
    device: Device<Context>,
    handle: DeviceHandle<Context>,
}

#[derive(Serialize, Clone)]
pub struct UsbDeviceInfo {
    name: String,
    vid: u16,
    pid: u16,
    serial_number: Option<String>,
    is_compatible: bool,
    port_number: u8,
}

#[derive(Serialize, Clone)]
pub struct ConnectionStatus {
    is_connected: bool,
    error_message: Option<String>,
}

static SELECTED_DEVICE: Lazy<Mutex<Option<UsbDeviceInfo>>> = Lazy::new(|| Mutex::new(None));
static CONNECTION_STATUS: Lazy<Mutex<ConnectionStatus>> = Lazy::new(|| {
    Mutex::new(ConnectionStatus {
        is_connected: false,
        error_message: None,
    })
});

impl UsbDevice {
    pub fn new() -> Result<Self, rusb::Error> {
        let context = Context::new()?;
        let (device, handle) = open_device(&context)?;
        Ok(Self { device, handle })
    }

    pub fn log_device_info(&mut self) -> Result<(), rusb::Error> {
        let device_desc = self.device.device_descriptor()?;
        debug!("Device Descriptor:");
        debug!("  bNumConfigurations: {}", device_desc.num_configurations());

        for config_num in 0..device_desc.num_configurations() {
            let config_desc = self.device.config_descriptor(config_num)?;
            debug!("Configuration {}:", config_num);
            debug!("NumInterfaces: {}", config_desc.num_interfaces());

            for interface in config_desc.interfaces() {
                for interface_desc in interface.descriptors() {
                    debug!("Interface {}:", interface_desc.interface_number());
                    debug!("NumEndpoints: {}", interface_desc.num_endpoints());

                    for endpoint_desc in interface_desc.endpoint_descriptors() {
                        debug!("Endpoint {:02x}:", endpoint_desc.address());
                        debug!("Type: {:?}", endpoint_desc.transfer_type());
                        debug!("Direction: {:?}", endpoint_desc.direction());
                    }
                }
            }
        }

        // Try to set the first configuration
        self.handle.set_active_configuration(1)?;
        info!("Set active configuration to 1");

        // Try to claim the first interface
        self.handle.claim_interface(0)?;
        info!("Claimed interface 0");

        Ok(())
    }

    pub fn send_message(&mut self, message: &Message) -> Result<usize, rusb::Error> {
        let timeout = Duration::from_secs(1);
        let (header, body) = message.as_bytes();
        let mut bytes_written = self.handle.write_bulk(0x01, header, timeout)?;
        bytes_written += self.handle.write_bulk(0x01, body, timeout)?;
        Ok(bytes_written)
    }
}

fn open_device(context: &Context) -> Result<(Device<Context>, DeviceHandle<Context>), rusb::Error> {
    let devices = context.devices()?;
    debug!("Found {} devices:", devices.len());
    for d in devices.iter() {
        debug!("\t{:?}", d);
    }

    for device in devices.iter() {
        let device_desc = device.device_descriptor()?;

        if device_desc.vendor_id() == USB_VID && device_desc.product_id() == USB_PID {
            info!(
                "Found device: vid={:x}:{:x}",
                device_desc.vendor_id(),
                device_desc.product_id()
            );
            let handle = device.open()?;
            return Ok((device, handle));
        }
    }

    error!("No matching USB device found");
    Err(rusb::Error::NoDevice)
}

pub fn get_usb_device_list() -> Result<Vec<UsbDeviceInfo>, rusb::Error> {
    let context = Context::new()?;
    let devices = context.devices()?;
    let mut device_list = Vec::new();

    for device in devices.iter() {
        if let Ok(device_info) = get_device_info(&device) {
            device_list.push(device_info);
        }
    }

    // Automatically select the first compatible device
    if SELECTED_DEVICE.lock().unwrap().is_none() {
        if let Some(first_compatible) = device_list.iter().find(|d| d.is_compatible) {
            *SELECTED_DEVICE.lock().unwrap() = Some(first_compatible.clone());
        }
    }

    Ok(device_list)
}

fn get_device_info(device: &Device<Context>) -> Result<UsbDeviceInfo, rusb::Error> {
    let device_desc = device.device_descriptor()?;
    let handle = device.open()?;

    let name = handle
        .read_product_string_ascii(&device_desc)
        .unwrap_or_else(|_| "Unknown".to_string());

    let serial_number = handle.read_serial_number_string_ascii(&device_desc).ok();

    let is_compatible = device_desc.vendor_id() == USB_VID && device_desc.product_id() == USB_PID;

    Ok(UsbDeviceInfo {
        name,
        vid: device_desc.vendor_id(),
        pid: device_desc.product_id(),
        serial_number,
        is_compatible,
        port_number: device.port_number(),
    })
}

#[tauri::command]
pub fn connect_to_device() -> Result<(), String> {
    let mut status = CONNECTION_STATUS.lock().unwrap();
    let selected_device = SELECTED_DEVICE.lock().unwrap().clone();

    if let Some(device_info) = selected_device {
        if !device_info.is_compatible {
            return Err("Selected device is not compatible".to_string());
        }

        // Simulate connection process (replace with actual connection logic)
        std::thread::sleep(Duration::from_millis(240));

        // For demonstration, let's randomly succeed or fail
        if rand::random::<u8>() > 5 {
            status.is_connected = true;
            status.error_message = None;
            Ok(())
        } else {
            status.is_connected = false;
            status.error_message = Some("Could not connect.".to_string());
            Err("Failed to connect to the device".to_string())
        }
    } else {
        Err("No device selected".to_string())
    }
}

#[tauri::command]
pub fn disconnect_device() -> Result<(), String> {
    let mut status = CONNECTION_STATUS.lock().unwrap();

    if status.is_connected {
        // Simulate disconnection process (replace with actual disconnection logic)
        std::thread::sleep(Duration::from_secs(1));

        status.is_connected = false;
        status.error_message = None;
        Ok(())
    } else {
        Err("No device is currently connected".to_string())
    }
}

#[tauri::command]
pub fn get_usb_devices() -> Result<Vec<UsbDeviceInfo>, String> {
    let device_list = get_usb_device_list().map_err(|e| e.to_string());
    device_list
}

#[tauri::command]
pub fn get_selected_device() -> Option<UsbDeviceInfo> {
    SELECTED_DEVICE.lock().unwrap().clone()
}

#[tauri::command]
pub fn select_device(port_number: u8) -> Result<(), String> {
    info!("Selecting device with port number: {}", port_number);
    let devices = get_usb_device_list().map_err(|e| e.to_string())?;

    if let Some(device) = devices.into_iter().find(|d| d.port_number == port_number) {
        *SELECTED_DEVICE.lock().unwrap() = Some(device);
        Ok(())
    } else {
        Err("Device not found".to_string())
    }
}

#[tauri::command]
pub fn get_connection_status() -> ConnectionStatus {
    CONNECTION_STATUS.lock().unwrap().clone()
}
