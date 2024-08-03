<script lang="ts">
    import { onMount, onDestroy } from "svelte";
    import { connectionStore } from "../../lib/connectionStore";

    export let selectedDevice: UsbDeviceInfo | null = null;

    let isLoading = false;
    let intervalId: number;

    $: connectionStatus = $connectionStore;

    async function handleConnect() {
        if (!selectedDevice || !selectedDevice.is_compatible) return;

        isLoading = true;
        try {
            await connectionStore.connect();
        } finally {
            isLoading = false;
        }
    }

    async function handleDisconnect() {
        isLoading = true;
        try {
            await connectionStore.disconnect();
        } finally {
            isLoading = false;
        }
    }

    onMount(() => {
        connectionStore.updateStatus();
        intervalId = setInterval(() => connectionStore.updateStatus(), 1000);
    });

    onDestroy(() => {
        clearInterval(intervalId);
    });
</script>

<div class="connect-button-container">
    {#if connectionStatus.is_connected}
        <button
            class="secondary"
            on:click={handleDisconnect}
            disabled={isLoading}
        >
            {isLoading ? "Disconnecting..." : "Disconnect"}
        </button>
    {:else}
        <button
            on:click={handleConnect}
            disabled={!selectedDevice ||
                !selectedDevice.is_compatible ||
                isLoading}
        >
            {isLoading ? "Connecting..." : "Connect"}
        </button>
    {/if}

    {#if connectionStatus.error_message}
        <div class="error-message">
            <h3>Connection Error:</h3>
            <p>{connectionStatus.error_message}</p>
        </div>
    {/if}
</div>

<style>
    .connect-button-container {
    }

    button {
        width: 100%;
    }

    .error-message {
        margin-top: var(--pico-spacing);
        padding: var(--pico-spacing);
        background-color: var(--pico-background-color);
        border: 1px solid var(--pico-error-color);
        border-radius: var(--pico-border-radius);
        color: var(--pico-error-color);
    }
</style>
