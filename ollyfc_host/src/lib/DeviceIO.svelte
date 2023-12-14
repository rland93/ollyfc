<script lang="ts">
  import { invoke } from "@tauri-apps/api/primitives";
  import { usb, searchUSB, type UsbDataDisplay } from "./usb_state";
  import { emit, listen, type Event } from "@tauri-apps/api/event";
  import { onMount } from "svelte";
  import type { FlightLogData } from "./flightTypes";
  import type { EmitEvent } from "./appTypes";

  let recvd: string[] = [];

  // onmount create event listner
  onMount(async () => {
    const unlistenSerial = await listen(
      "usb-data",
      (event: Event<EmitEvent>) => {
        // store data into recvd in a reactive way.
        const emitted: EmitEvent = event.payload;
        if (emitted.cmd === "getflash") {
          let flightLogData: FlightLogData = JSON.parse(emitted.data);
          console.log(emitted.data);
        }

        let disp = `${emitted.cmd}: ${emitted.data}`;

        recvd = [...recvd, disp];
      },
    );
  });

  const clearIoScreen = async () => {
    recvd = [];
  };
</script>

<div class="ui-element1 dev-cmd">
  <div class="ui-heading">
    <h3>Device IO</h3>
  </div>
  <div class="ui-element2 io-box">
    {#if $usb}
      <!-- list events in recvd-->
      {#each recvd as e}
        <p>{e}</p>
      {/each}
    {/if}
  </div>
  <div class="ui-element2">
    <button
      on:click={() => {
        clearIoScreen();
      }}>Clear</button
    >
  </div>
</div>

<style>
  .io-box {
    flex: 1 1 1px;
    overflow-y: scroll;
  }
  .dev-cmd {
    flex-grow: 3;
    display: flex;
    gap: var(--ui-padding2);
    flex-direction: column;
    min-height: 12;
  }
</style>
