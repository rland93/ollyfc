// Prevents additional console window on Windows in release, DO NOT REMOVE!!
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::sync::{Arc, Mutex};

// Modules
mod usb;

pub struct UsbDeviceState(Arc<Mutex<Option<usb::FcUsbDevice>>>);

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("debug")).init();

    let usb_device_state = UsbDeviceState(Arc::new(Mutex::new(None::<usb::FcUsbDevice>)));

    tauri::Builder::default()
        .manage(usb_device_state)
        .plugin(tauri_plugin_window::init())
        .plugin(tauri_plugin_shell::init())
        .invoke_handler(tauri::generate_handler![
            usb::search_for_usb,
            usb::send_usb_command,
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
