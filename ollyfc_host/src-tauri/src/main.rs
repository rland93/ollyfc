// Prevents additional console window on Windows in release, DO NOT REMOVE!!
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use ollyfc_common::cmd::Command;
use ollyfc_common::FlightLogData;

use serde::Serialize;
use std::sync::{Arc, Mutex};
// Modules
mod usb;
mod utils;
mod xfer_protoc;

#[derive(Debug, Clone, Serialize)]
pub struct EmitEvent {
    pub cmd: String,
    pub data: String,
}

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("debug")).init();

    // usb device
    let dev = Arc::new(Mutex::new(None::<xfer_protoc::FcUsbDevice>));

    // the current command that was just sent out
    let current_cmd: Arc<Mutex<Command>> = Arc::new(Mutex::new(Command::Invalid));

    // the loaded flight logs
    let flight_logs: Arc<Mutex<Vec<FlightLogData>>> =
        Arc::new(Mutex::new(Vec::<FlightLogData>::new()));

    tauri::Builder::default()
        .manage(dev.clone())
        .manage(current_cmd.clone())
        .manage(flight_logs.clone())
        .plugin(tauri_plugin_dialog::init())
        .plugin(tauri_plugin_window::init())
        .plugin(tauri_plugin_shell::init())
        .invoke_handler(tauri::generate_handler![
            usb::search_for_usb,
            usb::disconnect_usb,
            usb::save_logs_dialog,
            usb::download_logs,
            usb::get_logs,
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
