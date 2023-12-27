use log::{debug, error, info, warn};
use ollyfc_common::cmd::Command;
use ollyfc_common::log::{LogInfoPage, LOG_INFO_SIZE};
use ollyfc_common::{FlightLogData, LOG_SIZE};
use rusb::{Device, DeviceDescriptor, GlobalContext};
use serde::Serialize;
use serialport::SerialPort;
use serialport::{available_ports, SerialPortType};
use std::sync::{Arc, Mutex};
use tauri::{AppHandle, Manager, State};
use tauri_plugin_dialog::DialogExt;

// from crate
use crate::xfer_protoc::FcUsbDevice;

#[derive(Debug, Clone, Serialize)]
pub struct LogDumpProgress {
    pub current: usize,
    pub total: usize,
}

/******************************************************************************/

/// open a dialog to save to file the logs
#[tauri::command]
pub async fn save_logs_dialog(
    app: tauri::AppHandle,
    logs: State<'_, Arc<Mutex<Vec<FlightLogData>>>>,
) -> Result<String, String> {
    let file_path = app.dialog().file().blocking_save_file();
    let file_path = match file_path {
        Some(file_path) => file_path,
        None => {
            info!("No file path selected, user closed dialog.");
            return Ok(String::from("No file saved"));
        }
    };

    info!("Saving logs to file: {:?}", file_path);
    let logs = logs.lock().unwrap().clone();

    let csvtext = match FlightLogData::export_to_csv(&logs) {
        Ok(csvtext) => csvtext,
        Err(e) => {
            warn!("Failed to convert logs to CSV: {}", e);
            String::from("Could not convert files to CSV.")
        }
    };

    match std::fs::write(&file_path, csvtext) {
        Ok(_) => info!("Saved logs to file: {:?}", file_path),
        Err(e) => warn!("Failed to save logs to file: {}", e),
    }
    Ok(String::from(format!(
        "Saved {}",
        file_path.to_string_lossy()
    )))
}

/// "log download" sequence.
#[tauri::command]
pub async fn download_logs(
    app: tauri::AppHandle,
    logs: State<'_, Arc<Mutex<Vec<FlightLogData>>>>,
    usb_dev: State<'_, Arc<Mutex<Option<FcUsbDevice>>>>,
) -> Result<String, String> {
    info!("download_logs()");
    let mut maybe_dev = usb_dev.lock().unwrap();
    if let Some(ref mut dev) = *maybe_dev {
        // send command to get log info
        match dev.send(&[Command::GetFlashDataInfo.to_byte()]) {
            Ok(ack) => {
                if !ack {
                    warn!("Failed to get log info.");
                    return Err("Failed to get log info.".to_string());
                } else {
                    debug!("Sent {}", Command::GetFlashDataInfo);
                }
            }
            Err(e) => {
                warn!("Encountered error at send {e}");
                return Err("Error sending command.".to_string());
            }
        };

        // wait for log info response
        let loginfo_buf = match dev.recv() {
            Ok(r) => {
                debug!("Received log info response {} bytes", r.len());
                r
            }
            Err(e) => {
                warn!("Encountered error at recv {e}");
                return Err("Error receiving command.".to_string());
            }
        };
        let linfo = LogInfoPage::from_bytes(&loginfo_buf);
        info!("Log info response: {:?}", linfo);

        // no. of logs to download
        let num_logs = linfo.n_logs_in_region() as usize;
        let logs_per_page = linfo.page_size as usize / LOG_SIZE;

        // emit progress
        emit_log_download_progress(&app, 0, num_logs, 10);
        let npages = (linfo.block_end_ptr - linfo.block_start_ptr) / linfo.page_size;
        info!("reading npages={}...", npages);

        // send command to get log data
        match dev.send(&[Command::GetLogData.to_byte()]) {
            Ok(ack) => {
                if !ack {
                    warn!("Failed to get log data.");
                    return Err("Failed to get log data.".to_string());
                } else {
                    debug!("Sent {}", Command::GetFlashDataInfo);
                }
            }
            Err(e) => {
                warn!("Encountered error at send {e}");
                return Err("Error sending command.".to_string());
            }
        };

        // read out each page
        for i in 0..npages {
            let addr: u32 = linfo.block_start_ptr + (i * linfo.page_size);
            info!("page@{:4x?} ({i}/{npages}", addr);

            // send the address
            match dev.send(&addr.to_le_bytes()) {
                Ok(ack) => {
                    if !ack {
                        warn!("Failed to get log data.");
                        return Err("Failed to get log data.".to_string());
                    } else {
                        debug!("Sent {}", addr);
                    }
                }
                Err(e) => {
                    warn!("Encountered error at send {e}");
                    return Err("Error sending command.".to_string());
                }
            };

            // wait for log data response
            let page = match dev.recv() {
                Ok(r) => {
                    debug!("Received log data response {} bytes", r.len());
                    r
                }
                Err(e) => {
                    warn!("Encountered error at recv {e}");
                    return Err("Error receiving command.".to_string());
                }
            };

            // get logs out of page
            for j in 0..(linfo.page_size as usize / LOG_SIZE) {
                info!(
                    "log@{:4x?} ({j}/{}",
                    addr as usize + (j * LOG_SIZE),
                    linfo.page_size
                );

                let log_bytes: &[u8; LOG_SIZE] = &page[(j * LOG_SIZE)..((j + 1) * LOG_SIZE)]
                    .try_into()
                    .expect("Buffer size mismatch.");

                // push to store
                logs.lock()
                    .unwrap()
                    .push(FlightLogData::from_bytes(&log_bytes));

                // emit progress

                let current = i as usize * logs_per_page + j;
                emit_log_download_progress(&app, current, num_logs, logs_per_page);
            }
        }
        // send commmand to end log read
        let end = 0xFFFFFFFFu32;
        match dev.send(&end.to_le_bytes()) {
            Ok(ack) => {
                if !ack {
                    warn!("Termination command not acknowledged.");
                    return Err("Failed to send direct read termination command!".to_string());
                } else {
                    debug!("Sent termination command {:08x?}", end);
                }
            }
            Err(e) => {
                warn!("Encountered error at sending termination command {e}");
                return Err("Error sending command.".to_string());
            }
        };
        emit_log_download_progress(&app, num_logs, num_logs, 1)
    }
    Ok("".to_string())
}

fn info_page_from_bytes(read_buf: &[u8]) -> LogInfoPage {
    let bytes: &[u8; LOG_INFO_SIZE] = &read_buf[0..LOG_INFO_SIZE]
        .try_into()
        // should never hit
        .expect("Buffer size mismatch.");
    debug!("Data: {:?}", bytes);
    LogInfoPage::from_bytes(bytes)
}

/// Get logs from cache for display
#[tauri::command]
pub async fn get_logs(
    app: tauri::AppHandle,
    logs: State<'_, Arc<Mutex<Vec<FlightLogData>>>>,
) -> Result<Vec<String>, String> {
    let logs = logs.lock().unwrap().clone();
    info!("get-logs from cache: {:?}", logs.len());
    Ok(logdata2json(&logs))
}

/// search for a FC connected via USB
#[tauri::command]
pub fn search_for_usb(
    usb_state: State<'_, Arc<Mutex<Option<FcUsbDevice>>>>,
) -> Result<bool, String> {
    info!("Searching for USB device...");
    match find_fc() {
        Some(usb_dev) => {
            let found_dev = match FcUsbDevice::new(usb_dev) {
                Ok(d) => d,
                Err(e) => {
                    info!("Found a device but failed to open it.");
                    return Ok(false);
                }
            };

            let mut usb_device = (*usb_state).lock().unwrap();
            *usb_device = Some(found_dev);
            Ok(true)
        }
        None => {
            info!("No device found.");
            Ok(false)
        }
    }
}

#[tauri::command]
pub fn disconnect_usb(
    usb_state: State<'_, Arc<Mutex<Option<FcUsbDevice>>>>,
) -> Result<bool, String> {
    info!("Disconnecting USB device...");
    let mut usb_device = (*usb_state).lock().unwrap();
    *usb_device = None;
    Ok(false)
}

/******************************************************************************/

fn find_fc() -> Option<Device<GlobalContext>> {
    match rusb::devices() {
        Ok(device_list) => {
            for device in device_list.iter() {
                let desc = match device.device_descriptor() {
                    Ok(d) => d,
                    Err(e) => {
                        error!("Error getting device descriptor: {}", e);
                        continue;
                    }
                };
                info!(
                    "Found vid: pid: {:04x}:{:04x}",
                    desc.vendor_id(),
                    desc.product_id()
                );
                if desc.vendor_id() == 0x1209 && desc.product_id() == 0x6EF1 {
                    info!("Found FC device");
                    return Some(device);
                } else {
                    continue;
                }
            }
        }
        Err(e) => {
            error!("Error getting device list: {}", e);
            return None;
        }
    }
    None
}

/// convert a vec of log data to JSON string
fn logdata2json(logdata: &Vec<FlightLogData>) -> Vec<String> {
    let logs: Vec<String> = logdata
        .iter()
        .map(|log| match serde_json::to_string(log) {
            Ok(s) => s,
            Err(e) => {
                error!(
                    "Serialization of FlightLogData type to json failed: {:?}",
                    log
                );
                error!("Error serializing LogData: {}", e);
                String::new()
            }
        })
        .collect();
    logs
}

/// Emit progress of log download.
fn emit_log_download_progress(app: &tauri::AppHandle, count: usize, total: usize, every: usize) {
    if count % every == 0 {
        info!("{}/{}", count, total);
        match app.emit(
            "progress-data",
            LogDumpProgress {
                current: count,
                total: total,
            },
        ) {
            Ok(_) => {}
            Err(_) => {
                warn!("couldn't emit dump progress.");
            }
        }
    }
}
