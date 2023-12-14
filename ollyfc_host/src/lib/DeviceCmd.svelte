<script lang="ts">
  import { invoke } from "@tauri-apps/api/primitives";

  import { usb, searchUSB } from "./usb_state";

  let sendCmd = async function (cmd: string) {
    // await searchUSB();
    let res: string = await invoke("send_usb_command", {
      cmd: cmd,
    });
  };
</script>

<div class="ui-element1 dev-cmd">
  <div class="ui-heading">
    <h3>Device Commands</h3>
  </div>
  <div class="ui-element2">
    {#if $usb}
      <button
        on:click={() => {
          sendCmd("ack");
        }}>Ack</button
      >
      <button
        on:click={() => {
          sendCmd("getflashinfo");
        }}>Get Info</button
      >
      <button
        on:click={() => {
          sendCmd("getflash");
        }}>Get Flash</button
      >
    {:else}
      <p>Not connected</p>
    {/if}
  </div>
</div>

<style>
  .dev-cmd {
    width: 12em;
    flex-shrink: 0;
  }
</style>
