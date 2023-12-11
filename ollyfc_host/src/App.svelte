<script lang="ts">
  import DeviceCmd from "./lib/DeviceCmd.svelte";
  import UsbState from "./lib/UsbState.svelte";
</script>

<main class="ui-container">
  <div class="top-area">
    <UsbState />
  </div>
  <div class="main-window">
    <DeviceCmd />
  </div>
</main>

<style>
  .ui-container {
    display: flex;
    flex-direction: column;
    height: calc(100vh - 2 * var(--ui-padding1));
    gap: var(--ui-padding1);
  }

  .main-window {
    flex-grow: 1;
    display: flex;
  }

  .top-area {
    display: flex;
    justify-content: flex-start;
  }
</style>
