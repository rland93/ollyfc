use stm32f4xx_hal::otg_fs::{UsbBus, UsbBusType, USB};
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid},
};

#[allow(non_snake_case)]
pub fn usb_setup(
    usb: USB,
    USB_BUS: &'static mut Option<UsbBusAllocator<UsbBusType>>,
    EP_MEMORY: &'static mut [u32; 1024],
) -> (
    UsbDevice<'static, UsbBusType>,
    usbd_serial::SerialPort<'static, UsbBusType>,
) {
    USB_BUS.replace(UsbBus::new(usb, EP_MEMORY));
    let usb_ser = usbd_serial::SerialPort::new(&USB_BUS.as_ref().unwrap());
    let usb_dev = UsbDeviceBuilder::new(&USB_BUS.as_ref().unwrap(), UsbVidPid(0x1209, 0x6EF1))
        .device_class(usbd_serial::USB_CLASS_CDC)
        .strings(&[StringDescriptors::default()
            .manufacturer("OllyFC")
            .product("OllyFC Flight Computer")
            .serial_number("0001")])
        .unwrap()
        .build();

    (usb_dev, usb_ser)
}

pub async fn usb_task_fn(_cx: crate::app::usb_task::Context<'_>) {}
