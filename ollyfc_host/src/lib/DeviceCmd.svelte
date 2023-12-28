<script lang="ts">
  import { invoke } from "@tauri-apps/api/primitives";

  import { usb, searchUSB } from "./usb_state";

  let res: string = "";
  let downloadLogs = async function () {
    res = await invoke("download_logs");
  };
</script>

<div class="ui-element1 dev-cmd">
  <div class="ui-heading">
    <h3>Device Commands</h3>
  </div>
  <div class="ui-element2">
    {#if $usb}
      <button on:click={downloadLogs}>load from flash</button>
      {res}
    {:else}
      <p>Not connected</p>
    {/if}
  </div>
</div>

<style>
  .dev-cmd {
    width: 7em;
    flex-shrink: 0;
  }
</style>
