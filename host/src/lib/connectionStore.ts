import { writable } from 'svelte/store';
import { invoke } from "@tauri-apps/api/tauri";

interface ConnectionStatus {
    is_connected: boolean;
    error_message: string | null;
}

function createConnectionStore() {
    const { subscribe, set, update } = writable<ConnectionStatus>({
        is_connected: false,
        error_message: null
    });

    const updateStatus = async () => {
        const status = await invoke<ConnectionStatus>("get_connection_status");
        set(status);
    };
    return {
        subscribe,
        updateStatus: updateStatus,
        connect: async () => {
            try {
                await invoke("connect_to_device");
                await updateStatus();
            } catch (error) {
                console.error("Failed to connect:", error);
                update(state => ({ ...state, error_message: String(error) }));
            }
        },
        disconnect: async () => {
            try {
                await invoke("disconnect_device");
                await updateStatus();
            } catch (error) {
                console.error("Failed to disconnect:", error);
                update(state => ({ ...state, error_message: String(error) }));
            }
        }
    };
}

export const connectionStore = createConnectionStore();