use log::{debug, error, info, warn};
use ollyfc_common::cmd::Command;
use ollyfc_common::log::LogInfoPage;
use ollyfc_common::{FlightLogData, LOG_SIZE};
use rusb::{Device, GlobalContext};
use serde::Serialize;
use std::sync::{Arc, Mutex};
use tauri::{Manager, State};
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
    flight_logs: State<'_, Arc<Mutex<Vec<FlightLogData>>>>,
) -> Result<String, String> {
    info!("save_logs_dialog()");

    let file_path = match app.dialog().file().blocking_save_file() {
        Some(p) => {
            info!("File path selected: {:?}", p);
            p
        }
        None => {
            info!("No file path selected, user closed dialog.");
            return Err(String::from("No file selected"));
        }
    };
    info!("Saving logs to file: {:?}", file_path);
    let mut logs: Vec<FlightLogData> = flight_logs.lock().unwrap().clone();

    logs.sort_by_key(|log| log.timestamp);

    let csvtext = match crate::utils::export_to_csv(&logs) {
        Ok(csvtext) => csvtext,
        Err(e) => {
            warn!("Failed to convert logs to CSV: {}", e);
            return Err(String::from("Could not convert files to CSV."));
        }
    };
    match std::fs::write(&file_path, csvtext) {
        Ok(_) => info!("Saved logs to file: {:?}", file_path),
        Err(e) => {
            warn!("Failed to save logs to file: {}", e);
            return Err(String::from("Could not save files to CSV."));
        }
    }
    Ok(String::from(format!("{}", file_path.to_string_lossy())))
}

/// "log download" sequence.
#[tauri::command]
pub async fn download_logs(
    app: tauri::AppHandle,
    logs: State<'_, Arc<Mutex<Vec<FlightLogData>>>>,
    usb_dev: State<'_, Arc<Mutex<Option<FcUsbDevice>>>>,
) -> Result<String, String> {
    info!("download_logs()");
    logs.lock().unwrap().clear();
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
            info!("page@{:4x?} ({i}/{npages})", addr);

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

                // if we read an empty page, the log will appear as all 0xFF
                if !log_bytes.iter().all(|&b| b == 0xFF) {
                    // push to store
                    logs.lock()
                        .unwrap()
                        .push(FlightLogData::from_bytes(&log_bytes));
                }
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

/// Get logs from cache for display
#[tauri::command]
pub async fn get_logs(
    logs: State<'_, Arc<Mutex<Vec<FlightLogData>>>>,
) -> Result<Vec<String>, String> {
    let logs: Vec<String> = logs
        .lock()
        .unwrap()
        .clone()
        .iter()
        .map(|l| serde_json::to_string(l).unwrap())
        .collect();
    info!("get-logs from cache: {:?}", logs.len());
    Ok(logs)
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
                    info!("Found a device but failed to open it. {e}");
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
