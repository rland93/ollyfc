<script lang="ts">
  import { usb, searchUSB, disconnect } from "./usb_state";
  import { emit, listen, type Event } from "@tauri-apps/api/event";
  import { onMount } from "svelte";

  let usb_disconnected: boolean = false;
  onMount(async () => {
    const unListenDisconnect = await listen(
      "usb-disconnect",
      (event: Event<string>) => {
        usb_disconnected = true;
      },
    );
  });

  // Show connected status
  let connected: string;
  $: connected = $usb ? "Connected" : "Not Connected";
</script>

<div class="ui-element1 usb-state">
  <div class="ui-heading">
    <h3>USB State</h3>
  </div>
  <div class="state-items">
    <div class="ui-element2 state-option state-display">
      <p>
        USB: <span class:yes={$usb} class:no={!$usb}>{connected}</span>
      </p>
      <p>
        {#if usb_disconnected}
          <span class="no"
            >USB was disconnected! <button
              class="clear"
              on:click={() => {
                usb_disconnected = false;
              }}>x</button
            ></span
          >
        {/if}
      </p>
    </div>
    <div class="ui-element2 state-option">
      <button on:click={searchUSB}>Search for FC</button>
    </div>
    <div class="ui-element2 state-option">
      <button on:click={disconnect}>Disconnect</button>
    </div>
  </div>
</div>

<style>
  .clear {
    background-color: transparent;
    box-shadow: none;
    border: 1px solid var(--accent2);
    color: inherit;
    padding: 2px;
    margin: 0;
    font: inherit;
    cursor: pointer;
    outline: inherit;
  }
  .usb-state {
    display: flex;
    flex-direction: column;
    align-items: stretch;
    justify-content: flex-start;
    flex-grow: 1;
  }

  .state-items {
    flex-grow: 1;
    display: flex;
    flex-direction: row;
    justify-items: flex-start;
    gap: var(--ui-padding2);
    align-items: stretch;
  }

  .state-option p {
    margin: 0;
  }

  .yes {
    color: var(--accent1);
  }
  .no {
    color: var(--accent2);
  }

  .state-display {
    flex-grow: 1;
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 2rem;
    font-size: var(--ui-font-size1);
  }
</style>
