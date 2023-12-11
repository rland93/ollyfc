<script lang="ts">
  import { onMount } from "svelte";
  import { usb, searchUSB } from "./usb_state";

  // Show connected status
  let connected: string;
  $: connected = $usb ? "Connected" : "Not Connected";
</script>

<div class="ui-element usb-state">
  <button on:click={searchUSB}>Search for FC</button>
  <p class:yes={$usb} class:no={!$usb}>{connected}</p>
</div>

<style>
  .usb-state {
    flex-grow: 1;
  }

  .yes {
    color: var(--accent1);
  }
  .no {
    color: var(--accent2);
  }
</style>
