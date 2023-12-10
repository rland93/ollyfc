use rusb::{Device, DeviceDescriptor, GlobalContext};

pub struct FcUsbDevice {
    pub device: Device<GlobalContext>,
    pub device_descriptor: DeviceDescriptor,
}

pub fn find_fc() -> Option<FcUsbDevice> {
    match rusb::devices() {
        Ok(device_list) => {
            for device in device_list.iter() {
                let desc = device.device_descriptor().expect("Couldn't get descriptor");
                if desc.vendor_id() == 0x1209 && desc.product_id() == 0x6EF1 {
                    return Some(FcUsbDevice {
                        device: device,
                        device_descriptor: desc,
                    });
                }
            }
        }
        Err(e) => eprintln!("Error obtaining USB devices: {}", e),
    }
    None
}
