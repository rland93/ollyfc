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

<div class="ui-element dev-cmd">
  {#if $usb}
    <button
      on:click={() => {
        sendCmd("ack");
      }}>Send Acknowledge</button
    >
  {:else}
    <p>Not connected</p>
  {/if}

  device commands
</div>

<style>
  .dev-cmd {
    flex-grow: 1;
  }
</style>
