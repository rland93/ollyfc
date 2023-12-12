// Prevents additional console window on Windows in release, DO NOT REMOVE!!
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use log::{debug, error, info, warn};
use std::{
    sync::{Arc, Mutex},
    thread,
};
use tauri::Manager;

// Modules
mod usb;

#[derive(Clone)]
pub struct UsbDeviceState(Arc<Mutex<Option<usb::FcUsbDevice>>>);

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("debug")).init();

    let usb_device_state = UsbDeviceState(Arc::new(Mutex::new(None::<usb::FcUsbDevice>)));
    let listener_dev = usb_device_state.clone();

    tauri::Builder::default()
        .manage(usb_device_state)
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
                    let mut maybe_device = listener_dev.0.lock().unwrap();
                    if let Some(ref mut usb_device) = *maybe_device {
                        // Read from `usb_device.serial_port` if available
                        if let Some(ref mut ser) = usb_device.serial_port {
                            let count = match ser.bytes_to_read() {
                                Ok(c) => c,
                                Err(e) => {
                                    warn!("Error getting bytes to read: {}", e);
                                    match listener_app_handle.emit("usb-disconnect", None::<String>)
                                    {
                                        Ok(_) => (),
                                        Err(e) => error!("Error emitting event: {}", e),
                                    };
                                    continue;
                                }
                            };
                            if count == 0 {
                                continue;
                            }
                            info!("Bytes to read: {}", count);

                            let read_count = match ser.read(&mut read_buf) {
                                Ok(c) => c,
                                Err(e) => {
                                    warn!("Error reading from serialport: {}", e);
                                    match listener_app_handle.emit("usb-disconnect", None::<String>)
                                    {
                                        Ok(_) => (),
                                        Err(e) => error!("Error emitting event: {}", e),
                                    };
                                    continue;
                                }
                            };

                            info!("Read {} bytes", read_count);

                            if read_count > 0 {
                                let data = &read_buf[..read_count];
                                debug!("Data: {:?}", data);
                                let data_str = format!("{:?}", data);
                                match listener_app_handle
                                    .emit("usb-data", Some(&data_str.to_string()))
                                {
                                    Ok(_) => {
                                        info!("Emitted event {}", data_str);
                                        ()
                                    }
                                    Err(e) => error!("Error emitting event: {}", e),
                                };
                            }
                        }
                        // usb disconnected
                    }
                    // } else {
                    //     match listener_app_handle.emit("usb-disconnect", None::<String>) {
                    //         Ok(_) => (),
                    //         Err(e) => error!("Error emitting event: {}", e),
                    //     };
                    // }
                    // Add a small sleep to prevent this loop from hogging the CPU
                    std::thread::sleep(std::time::Duration::from_millis(100));
                }
            });

            Ok(())
        })
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
