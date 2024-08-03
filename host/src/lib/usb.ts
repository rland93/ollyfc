interface UsbDeviceInfo {
    name: string;
    vid: number;
    pid: number;
    serial_number: string | null;
    is_compatible: boolean;
    port_number: number;
}