<script lang="ts">
    export let device: UsbDeviceInfo | null = null;
</script>

<div class="selected-device">
    {#if device}
        <div class="device-info">
            <p><strong>Name:</strong> {device.name}</p>
            <p>
                <strong>VID:PID:</strong>
                {device.vid.toString(16)}:{device.pid.toString(16)}
            </p>
            <p>
                <strong>Serial Number:</strong>
                {device.serial_number || "N/A"}
            </p>
            <p><strong>Port Number:</strong> {device.port_number}</p>
        </div>
    {:else}
        <p class="device-info">No device selected</p>
    {/if}
</div>

<style>
    .device-info {
        background-color: var(--pico-background-color);
        border: 1px solid var(--pico-primary);
        border-radius: var(--pico-border-radius);
        padding: var(--pico-spacing);
        margin-bottom: var(--pico-spacing);
    }

    .device-info {
        font-family: var(--pico-font-family-monospace);
    }
</style>
