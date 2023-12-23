<script lang="ts">
  import { invoke } from "@tauri-apps/api/primitives";
  import { emit, listen, type Event } from "@tauri-apps/api/event";
  import { onMount } from "svelte";
  import Progress from "./Progress.svelte";
  import type { EmitEvent } from "./appTypes";
  import type { LogDumpProgress } from "./usb_state";

  let lowerBound: number = 0;
  let upperBound: number = 100;
  let currentValue: number = 50;

  onMount(async () => {
    const unlistenSerial = await listen(
      "progress-data",
      (event: Event<LogDumpProgress>) => {
        // store data into recvd in a reactive way.
        // let progress: LogDumpProgress = JSON.parse(event);
        currentValue = event.payload.current;
        upperBound = event.payload.total;
      },
    );
  });
</script>

<div class="ui-element1">
  <Progress {lowerBound} {upperBound} {currentValue} />
</div>

<style>
</style>
