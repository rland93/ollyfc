import { writable } from 'svelte/store';
import { invoke } from "@tauri-apps/api/tauri";

interface Settings {
    dark_mode: boolean;
}

function createSettingsStore() {
    const { subscribe, set, update } = writable<Settings>({
        dark_mode: false
    });

    return {
        subscribe,
        toggleDarkMode: () => update(state => ({ ...state, dark_mode: !state.dark_mode }))
    };
}

export const settingsStore = createSettingsStore();