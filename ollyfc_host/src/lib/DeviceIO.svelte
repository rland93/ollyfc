<script lang="ts">
  import { invoke } from "@tauri-apps/api/primitives";
  import { usb, searchUSB, type UsbDataDisplay } from "./usb_state";
  import { emit, listen, type Event } from "@tauri-apps/api/event";
  import { onMount } from "svelte";
  import type { FlightLogData } from "./flightTypes";
  import type { EmitEvent } from "./appTypes";
  import { get } from "svelte/store";

  let recvd: FlightLogData[] = [];

  async function handleSave() {
    // const result = await invoke("save_logs");
    // console.log(result);
    console.log(get(usb));
  }

  let logs: FlightLogData[] = [];
  async function getLogs() {
    let res: string[] = await invoke("get_logs");
    console.log(res);
    logs = res.map((l) => JSON.parse(l));
    console.log(logs);
  }

  const clearIoScreen = async () => {
    recvd = [];
  };
</script>

<div class="ui-element1 dev-cmd">
  <div class="ui-heading">
    <h3>Device IO</h3>
  </div>
  <div class="ui-element2 io-box">
    {#each logs as d}
      <span style="grid-column: 1">{(d.timestamp / 1000).toFixed(2)}</span>
      <span style="grid-column: 2">{d.sensor_input.pitch.toFixed(2)} </span>
      <span style="grid-column: 3">{d.sensor_input.roll.toFixed(2)} </span>
      <span style="grid-column: 4">{d.sensor_input.yaw.toFixed(2)} </span>
      <span style="grid-column: 5"
        >{d.control_policy.throttle.toFixed(2)}
      </span>
      <span style="grid-column: 6"
        >{d.control_policy.elevator.toFixed(2)}
      </span>
      <span style="grid-column: 7">{d.control_policy.aileron.toFixed(2)} </span>
      <span style="grid-column: 8">{d.control_policy.rudder.toFixed(2)} </span>
    {/each}
  </div>
  <div class="ui-element2">
    <button
      on:click={() => {
        clearIoScreen();
      }}
    >
      clear
    </button>

    <button on:click={handleSave}> save </button>

    <button on:click={getLogs}> get </button>
  </div>
</div>

<style>
  .io-box {
    flex: 1 1 1px;
    overflow-y: scroll;

    display: grid;
    grid-template-columns: repeat(8, 1fr);
  }
  .io-box span {
    text-align: right;
    font-family: monospace;
    font-size: 0.8rem;
    display: block;
  }

  .dev-cmd {
    flex-grow: 3;
    display: flex;
    gap: var(--ui-padding2);
    flex-direction: column;
    min-height: 12;
  }
  /* .log-item {
    border-bottom: 1px solid var(--med3);
  } */
</style>
