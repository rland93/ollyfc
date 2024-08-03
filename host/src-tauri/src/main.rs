// Prevents additional console window on Windows in release, DO NOT REMOVE!!
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use log::LevelFilter;

mod usb;

fn main() {
    tauri::Builder::default()
        .plugin(
            tauri_plugin_log::Builder::default()
                .level(LevelFilter::Debug)
                .build(),
        ) // <-- this line here
        .invoke_handler(tauri::generate_handler![
            usb::get_usb_devices,
            usb::get_selected_device,
            usb::select_device,
            usb::connect_to_device,
            usb::disconnect_device,
            usb::get_connection_status,
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
