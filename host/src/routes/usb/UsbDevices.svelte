<script lang="ts">
    import { onMount, onDestroy } from "svelte";
    import { invoke } from "@tauri-apps/api/tauri";
    import SelectedUsbDevice from "./SelectedUsbDevice.svelte";
    import Connect from "./Connect.svelte";

    interface UsbDeviceInfo {
        name: string;
        vid: number;
        pid: number;
        serial_number: string | null;
        is_compatible: boolean;
        port_number: number;
    }

    let devices: UsbDeviceInfo[] = [];
    let selectedDevice: UsbDeviceInfo | null = null;
    let intervalId: number;

    async function fetchDevices() {
        try {
            devices = await invoke("get_usb_devices");
            selectedDevice = await invoke("get_selected_device");
        } catch (error) {
            console.error("Error fetching USB devices:", error);
        }
    }

    async function selectDevice(portNumber: number) {
        try {
            await invoke("select_device", { portNumber });
            await fetchDevices();
        } catch (error) {
            console.error("Error selecting device:", error);
        }
    }

    onMount(() => {
        fetchDevices();
        intervalId = setInterval(fetchDevices, 1000);
    });

    onDestroy(() => {
        clearInterval(intervalId);
    });
</script>

<div class="usb-device-manager grid">
    {#if devices.length === 0}
        <p>No USB devices detected.</p>
    {:else}
        <div class="usb-device-list">
            {#each devices as device}
                <button
                    class="outline"
                    class:contrast={device.is_compatible &&
                        !(selectedDevice?.port_number === device.port_number)}
                    class:secondary={!device.is_compatible}
                    on:click={() => selectDevice(device.port_number)}
                >
                    <span class="name">{device.name}</span>
                    <span class="vidpid">
                        {device.vid.toString(16)}:{device.pid.toString(16)}
                    </span>
                    <span class="serialno">
                        {#if device.serial_number}
                            {device.serial_number}
                        {/if}
                    </span>
                    <span class="port">Port: {device.port_number}</span>
                </button>
            {/each}
        </div>
    {/if}

    <SelectedUsbDevice device={selectedDevice} />

    <Connect {selectedDevice} />
</div>

<style>
    .usb-device-list {
        display: flex;
        flex-direction: column;
        gap: var(--pico-spacing);
    }
</style>
