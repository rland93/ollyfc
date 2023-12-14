<script lang="ts">
  import DeviceCmd from "./lib/DeviceCmd.svelte";
  import UsbState from "./lib/UsbState.svelte";
  import DeviceIo from "./lib/DeviceIO.svelte";
  import Dump from "./lib/Dump.svelte";
</script>

<main class="ui-container">
  <div class="top-area">
    <UsbState />
  </div>
  <Dump />
  <div class="main-window">
    <DeviceCmd />
    <DeviceIo />
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
    flex-direction: row;
    gap: var(--ui-padding1);
  }

  .top-area {
    display: flex;
    justify-content: flex-start;
  }
</style>
