<script lang="ts">
  export let lowerBound: number;
  export let upperBound: number;
  export let currentValue: number;

  $: percentage =
    ((currentValue - lowerBound) / (upperBound - lowerBound)) * 100;
</script>

<div class="bar-container">
  Load Progress:
  <div class="progress-bar">
    <div class="progress" style={`width: ${percentage}%;`}></div>
  </div>
  <div class="bytes">
    <span style="display:inline-block;width:4em;text-align:right;"
      >{currentValue - lowerBound + 1}
    </span>
    / {upperBound - lowerBound}
  </div>
</div>

<style>
  .bar-container {
    display: flex;
    flex-direction: row;
    align-items: center;
    gap: var(--ui-padding1);
  }

  .bytes {
    font-size: 0.8rem;
  }

  .progress-bar {
    flex-grow: 1;
    height: 20px;
    background-color: var(--dark1);
    overflow: hidden;
  }

  .progress {
    height: 100%;
    background-color: var(--accent1);
    transition: width 0.3s ease;
  }
</style>
