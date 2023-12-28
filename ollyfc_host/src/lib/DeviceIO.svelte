<script lang="ts">
  import { invoke } from "@tauri-apps/api/primitives";
  import type { FlightLogData } from "./flightTypes";
  import { get } from "svelte/store";
  import { logdata } from "./usb_state";
  import { hasNull } from "./utils";
  import type { LogDumpProgress } from "./usb_state";

  import Chart from "chart.js/auto";
  import { onDestroy, onMount } from "svelte";
  import { listen, type Event } from "@tauri-apps/api/event";

  let chart: Chart | null = null;
  let canvas: HTMLCanvasElement;

  let showPitch = true;
  let showYaw = true;
  let showRoll = true;

  let chartFontSize = 14;

  // handle the save function
  async function handleSave() {
    console.log("save called");
    await invoke("save_logs_dialog");
  }

  // get the logs from tauri backend
  async function getLogs() {
    let res: string[] = await invoke("get_logs");
    console.log(`got ${res.length} logs`);
    logdata.set([]);
    let logs: FlightLogData[] = res
      .map((r) => JSON.parse(r))
      .filter((r) => !hasNull(r));
    console.log(logs[0]);
    console.log(`will display ${res.length} logs`);
    logdata.set(logs);
  }

  // update visibility of chart series
  function updateSeriesVis(
    showPitch: boolean,
    showYaw: boolean,
    showRoll: boolean,
  ) {
    if (!chart) return;
    chart.data.datasets[0].hidden = !showPitch;
    chart.data.datasets[1].hidden = !showYaw;
    chart.data.datasets[2].hidden = !showRoll;
    chart.update();
  }

  // update the chart with new logs from `logs`
  function updateChart(logs: FlightLogData[]) {
    if (!chart) return;

    const timestamps = logs.map((log) => log.timestamp / 1000);
    const pitch = logs.map((log) => log.pitch);
    const yaw = logs.map((log) => log.yaw);
    const roll = logs.map((log) => log.roll);

    chart.data.labels = timestamps;
    chart.data.datasets[0].data = pitch;
    chart.data.datasets[1].data = yaw;
    chart.data.datasets[2].data = roll;

    chart.update();
  }

  function updateChartAppearance(base: number) {
    if (!chart) return;

    let opts = {
      // Global font settings
      plugins: {
        tooltip: {
          titleFont: { size: base - 1 }, // Tooltip title
          bodyFont: { size: base - 1 }, // Tooltip body
        },
        legend: {
          labels: {
            font: { size: base }, // Legend labels
          },
        },
      },
      scales: {
        x: {
          ticks: {
            font: { size: base }, // X-axis tick labels
          },
        },
        y: {
          ticks: {
            font: { size: base }, // Y-axis tick labels
          },
        },
      },
    };
    chart.options = { ...chart.options, ...opts };
    chart.update();
  }
  // update chart font size
  $: updateChartAppearance(chartFontSize);

  // update chart based on changes
  $: if (logdata) {
    updateChart($logdata);
  }

  $: updateSeriesVis(showPitch, showYaw, showRoll), showPitch;
  $: updateSeriesVis(showPitch, showYaw, showRoll), showYaw;
  $: updateSeriesVis(showPitch, showYaw, showRoll), showRoll;
  // clear the logs that are on the UI
  const clearLogs = async () => {
    logdata.set([]);
  };

  onMount(async () => {
    const ctx = canvas.getContext("2d");
    if (!ctx) {
      console.error("Failed to get canvas context");
      return;
    }

    chart = new Chart(ctx, {
      type: "line",
      data: {
        labels: [],
        datasets: [
          {
            label: "pitch",
            data: [],
            borderColor: "rgb(38	185	230)",
            fill: false,
          },
          {
            label: "yaw",
            data: [],
            borderColor: "rgb(144	170	236)",
            fill: false,
          },
          {
            label: "roll",
            data: [],
            borderColor: "rgb(203	154	216)",
            fill: false,
          },
        ],
      },

      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: {
          duration: 50,
        },
        plugins: {
          tooltip: {
            animation: false,
          },
        },
      },
    });
  });

  onDestroy(() => {
    chart?.destroy();
  });

  window.addEventListener("resize", () => {
    chart?.resize();
  });

  $logdata = get(logdata);
</script>

<div class="ui-element1 dev-cmd">
  <div class="ui-heading">
    <h3>Device IO</h3>
  </div>
  <div class="ui-element2">
    <button
      on:click={() => {
        clearLogs();
      }}
    >
      clear
    </button>
    <button on:click={handleSave}> save </button>
    <button on:click={getLogs}> get </button>
  </div>
  <div class="ui-element2 canvas">
    <div>
      <canvas style="height: 100%!important;" bind:this={canvas}></canvas>
    </div>
  </div>

  <div class="ui-element2 chart-controls">
    <label>
      <input type="checkbox" bind:checked={showPitch} />
      Pitch
    </label>
    <label>
      <input type="checkbox" bind:checked={showYaw} />
      Yaw
    </label>
    <label>
      <input type="checkbox" bind:checked={showRoll} />
      Roll
    </label>
    <label
      ><input
        type="range"
        bind:value={chartFontSize}
        min="14"
        max="18"
        step="1"
      />
      Font Size
    </label>
  </div>
</div>

<style>
  .chart-controls {
    display: flex;
    justify-content: center;
    align-items: center;
  }

  .chart-controls label {
    padding: var(--ui-padding2);
  }
  .dev-cmd {
    flex-grow: 3;
    display: flex;
    gap: var(--ui-padding2);
    flex-direction: column;
    min-height: 12;
  }

  .canvas {
    background-color: #ecece8;
    padding: var(--ui-padding2);
  }

  .canvas {
    position: relative;
    flex-grow: 1;
  }

  .canvas > div {
    position: absolute;
    top: 2%;
    left: 2%;
    right: 2%;
    bottom: 2%;
    height: 96%;
  }
</style>
