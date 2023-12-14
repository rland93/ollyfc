// Prevents additional console window on Windows in release, DO NOT REMOVE!!
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use log::{debug, error, info, warn};
use ollyfc_common::cmd::Command;
use ollyfc_common::{FlightLogData, LOG_SIZE};

use serde::Serialize;
use serialport::SerialPort;
use std::{
    sync::{Arc, Mutex},
    thread,
};
use tauri::{AppHandle, Manager};
// Modules
mod usb;

#[derive(Clone, Serialize)]
pub struct UsbDataDisplay {
    pub cmd: String,
    pub data: String,
}

#[derive(Clone)]
pub struct UsbDeviceState(Arc<Mutex<Option<usb::FcUsbDevice>>>);

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("debug")).init();

    // the usb device, including handlers
    let usb_device_state = UsbDeviceState(Arc::new(Mutex::new(None::<usb::FcUsbDevice>)));

    // the current command that was just sent out
    let current_cmd: Arc<Mutex<Command>> = Arc::new(Mutex::new(Command::Invalid));

    // ref to usb device, for the listener thread
    let listener_dev = usb_device_state.clone();
    let listener_current_cmd = current_cmd.clone();

    tauri::Builder::default()
        .manage(usb_device_state)
        .manage(current_cmd)
        .plugin(tauri_plugin_window::init())
        .plugin(tauri_plugin_shell::init())
        .invoke_handler(tauri::generate_handler![
            usb::search_for_usb,
            usb::disconnect_usb,
            usb::send_usb_command,
        ])
        .setup(|app| {
            let app_handle = app.app_handle(); // Obtain the AppHandle

            let listener_app_handle = app_handle.clone();
            // Spawn listener
            thread::spawn(move || {
                let mut read_buf = [0u8; 128];
                loop {
                    std::thread::sleep(std::time::Duration::from_millis(50));

                    let mut maybe_device = listener_dev.0.lock().unwrap();
                    let current = listener_current_cmd.lock().unwrap().clone();

                    if let Some(ref mut usb_device) = *maybe_device {
                        // Read from `usb_device.serial_port` if available
                        if let Some(ref mut ser) = usb_device.serial_port {
                            // read bytes. could be 0 if no bytes to be read but this way we handle
                            // the serialport disconnection cases so that the ui is updated in a
                            // timely manner.
                            let read_count =
                                ser_read_bytes(&listener_app_handle, ser, current, &mut read_buf);

                            // continue if no bytes are to be read.
                            if read_count == 0 {
                                continue;
                            }

                            // we have bytes
                            match current {
                                Command::Acknowledge => {
                                    warn!("No implementation");
                                }
                                Command::GetLogData => {
                                    deser_log_data(
                                        read_count,
                                        &read_buf,
                                        &listener_app_handle,
                                        current,
                                    );
                                }
                                Command::GetFlashDataInfo => {
                                    warn!("No implementation");
                                }
                                _ => {
                                    warn!("No implementation");
                                }
                            }
                        }
                        // usb disconnected
                    }
                }
            });

            Ok(())
        })
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}

fn ser_read_bytes(
    app_handle: &AppHandle,
    ser: &mut Box<dyn SerialPort>,
    cmd: Command,
    read_buf: &mut [u8; 128],
) -> usize {
    let count = match ser.bytes_to_read() {
        Ok(c) => c,
        Err(e) => {
            warn!("Error getting bytes to read: {}", e);
            match app_handle.emit("usb-disconnect", None::<String>) {
                Ok(_) => (),
                Err(e) => error!("Error emitting event: {}", e),
            };
            return 0;
        }
    };
    if count == 0 {
        return 0;
    }
    info!("Bytes to read: {}", count);

    let read_count = match ser.read(read_buf) {
        Ok(c) => c,
        Err(e) => {
            warn!("Error reading from serialport: {}", e);
            match app_handle.emit("usb-disconnect", None::<String>) {
                Ok(_) => (),
                Err(e) => error!("Error emitting event: {}", e),
            };
            return 0;
        }
    };

    info!("Read {} bytes", read_count);
    return read_count;
}

fn deser_log_data(read_count: usize, read_buf: &[u8], app_handle: &AppHandle, cmd: Command) {
    if read_count % LOG_SIZE != 0 {
        warn!(
            "Read count {} not an integer multiple of LOG_SIZE {}",
            read_count, LOG_SIZE
        );
        return;
    }

    for i in 0..read_count / LOG_SIZE {
        info!("Read log {} of {} in buffer", i, read_count / LOG_SIZE);

        let data: &[u8; LOG_SIZE] = &read_buf[i * LOG_SIZE..(i + 1) * LOG_SIZE]
            .try_into()
            // should never hit
            .expect("Buffer size mismatch.");

        debug!("Data: {:?}", data);
        let logdata = FlightLogData::from_bytes(data);
        let logdata_json = match serde_json::to_string(&logdata) {
            Ok(s) => s,
            Err(e) => {
                error!(
                    "Serialization of FlightLogData type to json failed: {:?}",
                    logdata
                );
                error!("Error serializing LogData: {}", e);
                return;
            }
        };

        match app_handle.emit(
            "usb-data",
            UsbDataDisplay {
                cmd: cmd.to_string(),
                data: logdata_json,
            },
        ) {
            Ok(_) => {
                info!("Emitted usb-data event with cmd: {}", cmd);
                ()
            }
            Err(e) => error!("Error emitting event: {}", e),
        };
    }
}
