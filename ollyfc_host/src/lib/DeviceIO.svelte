<script lang="ts">
  import { invoke } from "@tauri-apps/api/primitives";
  import { usb, searchUSB, type UsbDataDisplay } from "./usb_state";
  import { emit, listen, type Event } from "@tauri-apps/api/event";
  import { onMount } from "svelte";
  import type { FlightLogData } from "./flightTypes";
  import type { EmitEvent } from "./appTypes";

  let view = false;
  let recvd: FlightLogData[] = [];

  // onmount create event listner
  onMount(async () => {
    const unlistenSerial = await listen(
      "usb-data",
      (event: Event<EmitEvent>) => {
        // store data into recvd in a reactive way.
        const emitted: EmitEvent = event.payload;
        if (emitted.cmd === "getflash") {
          let flightLogData: FlightLogData = JSON.parse(emitted.data);
        }
        recvd = [...recvd];
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
      {#if view}
        {#each recvd as d}
          <div class="log-item">
            <span>{d.sensorInput.pitch} </span>
            <span>{d.sensorInput.roll} </span>
            <span>{d.sensorInput.yaw} </span>
            <span>{d.sbusInput.throttle} </span>
            <span>{d.sbusInput.elevator} </span>
            <span>{d.sbusInput.aileron} </span>
            <span>{d.sbusInput.rudder} </span>
          </div>
        {/each}
      {/if}
    {/if}
  </div>
  <div class="ui-element2">
    <button
      on:click={() => {
        clearIoScreen();
      }}
    >
      clear
    </button>
  </div>
</div>

<style>
  .io-box {
    flex: 1 1 1px;
    overflow-y: scroll;

    display: grid;
    grid-template-columns: repeat(7, 1fr);
    grid-gap: 10px;
  }
  .dev-cmd {
    flex-grow: 3;
    display: flex;
    gap: var(--ui-padding2);
    flex-direction: column;
    min-height: 12;
  }
  .log-item {
    border-bottom: 1px solid var(--med3);
  }
</style>
