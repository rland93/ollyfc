<script lang="ts">
    import { settingsStore } from "$lib/settingsStore";

    $: dark_mode = $settingsStore.dark_mode;
</script>

<fieldset>
    <label>
        <input
            name="dark-mode"
            on:click={settingsStore.toggleDarkMode}
            type="checkbox"
            role="switch"
            checked={dark_mode}
        />
        Dark Mode
    </label>
</fieldset>

<style>
</style>
