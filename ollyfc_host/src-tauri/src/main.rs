// Prevents additional console window on Windows in release, DO NOT REMOVE!!
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use log::{debug, error, info, warn};
use ollyfc_common::cmd::Command;
use ollyfc_common::log::{LogInfoPage, LOG_INFO_SIZE};
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

#[derive(Debug, Clone, Serialize)]
pub struct EmitEvent {
    pub cmd: String,
    pub data: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct LogDumpProgress {
    pub current: usize,
    pub total: usize,
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

                // number of logs we've read so far
                let mut log_progress: usize = 0;

                // info about the log region
                let mut info = LogInfoPage::default();

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
                                    log_progress += read_count / LOG_SIZE;
                                    send_progress_event(
                                        &LogDumpProgress {
                                            current: log_progress,
                                            total: info.n_logs_in_region() as usize,
                                        },
                                        &listener_app_handle,
                                        current,
                                    );
                                }
                                Command::GetFlashDataInfo => {
                                    info!("Get Info {:?}", read_buf);
                                    info = deser_read_info(
                                        read_count,
                                        &read_buf[0..LOG_INFO_SIZE],
                                        &listener_app_handle,
                                        current,
                                    );
                                    send_info_event(&info, &listener_app_handle, current);
                                    send_progress_event(
                                        &LogDumpProgress {
                                            current: log_progress,
                                            total: info.n_logs_in_region() as usize,
                                        },
                                        &listener_app_handle,
                                        current,
                                    );
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
            EmitEvent {
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

fn deser_read_info(
    read_count: usize,
    read_buf: &[u8],
    app_handle: &AppHandle,
    cmd: Command,
) -> LogInfoPage {
    if read_count != LOG_INFO_SIZE {
        warn!(
            "Read count {} not equal to expected info size 20",
            read_count
        );
        return LogInfoPage::default();
    }

    let bytes: &[u8; LOG_INFO_SIZE] = &read_buf[0..LOG_INFO_SIZE]
        .try_into()
        // should never hit
        .expect("Buffer size mismatch.");

    debug!("Data: {:?}", bytes);
    LogInfoPage::from_bytes(bytes)
}

fn send_info_event(info: &LogInfoPage, app_handle: &AppHandle, cmd: Command) {
    let info_json = match serde_json::to_string(info) {
        Ok(s) => s,
        Err(e) => {
            error!(
                "Serialization of LogInfoPage type to json failed: {:?}",
                info
            );
            error!("Error serializing LogInfoPage: {}", e);
            return;
        }
    };

    match app_handle.emit(
        "info-data",
        EmitEvent {
            cmd: cmd.to_string(),
            data: info_json,
        },
    ) {
        Ok(_) => {
            info!("Emitted info-data event with cmd: {}", cmd);
            ()
        }
        Err(e) => error!("Error emitting event: {}", e),
    };
}

fn send_progress_event(progress: &LogDumpProgress, app_handle: &AppHandle, cmd: Command) {
    let progress_json = match serde_json::to_string(&progress) {
        Ok(s) => s,
        Err(e) => {
            error!(
                "Serialization of LogDumpProgress type to json failed: {:?}",
                progress
            );
            error!("Error serializing LogDumpProgress: {}", e);
            return;
        }
    };

    match app_handle.emit(
        "progress-data",
        EmitEvent {
            cmd: cmd.to_string(),
            data: progress_json,
        },
    ) {
        Ok(_) => {
            info!("Emitted progress-data event with cmd: {}", cmd);
            ()
        }
        Err(e) => error!("Error emitting event: {}", e),
    };
}
