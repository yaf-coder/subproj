import uPlot from "uplot";
import type { Snapshot } from "./api";

const MAX_POINTS = 600;

interface RingChart {
  plot: uPlot;
  t: number[];
  series: number[][];
}

function makeChart(container: HTMLElement, seriesLabels: string[], colors: string[]): RingChart {
  const data: uPlot.AlignedData = [
    [],
    ...seriesLabels.map(() => [] as number[]),
  ];
  const opts: uPlot.Options = {
    width: container.clientWidth || 340,
    height: 130,
    cursor: { drag: { x: false, y: false } },
    legend: { show: false },
    scales: { x: { time: false }, y: { auto: true } },
    axes: [
      { stroke: "#8da3b8", grid: { stroke: "#2a3a4a40" }, ticks: { stroke: "#2a3a4a" } },
      { stroke: "#8da3b8", grid: { stroke: "#2a3a4a40" }, ticks: { stroke: "#2a3a4a" } },
    ],
    series: [
      { label: "t" },
      ...seriesLabels.map((label, i) => ({
        label,
        stroke: colors[i],
        width: 1.5,
      })),
    ],
  };
  const plot = new uPlot(opts, data, container);
  return { plot, t: [], series: seriesLabels.map(() => []) };
}

export class Telemetry {
  private depthChart: RingChart;
  private powerChart: RingChart;
  private connStatus: HTMLElement;

  constructor(depthEl: HTMLElement, powerEl: HTMLElement, connEl: HTMLElement) {
    this.depthChart = makeChart(depthEl, ["depth"], ["#4cc4ff"]);
    this.powerChart = makeChart(powerEl, ["power_W", "soc_pct"], ["#ffa000", "#88e07d"]);
    this.connStatus = connEl;
  }

  setConnected(state: "ok" | "warn" | "bad", text: string) {
    this.connStatus.className = `status ${state}`;
    this.connStatus.textContent = text;
  }

  update(s: Snapshot) {
    this.updateTextOnly(s);
    this.updateChartOnly(s);
  }

  // Update the right-hand numeric rows without touching charts. Used when
  // scrubbing through history.
  updateTextOnly(s: Snapshot) {
    setText("tel-time", fmtTime(s.t));
    setText("tel-depth", `${s.depth_m.toFixed(2)} m`);
    setText("tel-speed", `${s.speed_m_s.toFixed(2)} m/s`);
    setText("tel-hdg", `${normDeg(s.heading_deg).toFixed(1)}°`);
    setText("tel-pr", `${s.pitch_deg.toFixed(1)}° / ${s.roll_deg.toFixed(1)}°`);
    setText("tel-wp", `${s.wp} / ${s.wp_total}`);
    setText("tel-state", stateLabel(s));
    setText("tel-soc", `${(s.soc * 100).toFixed(1)}%`);
    setText("tel-volt", `${s.voltage_V.toFixed(2)} V`);
    setText("tel-curr", `${s.current_A.toFixed(2)} A`);
    setText("tel-pow", `${s.power_W.toFixed(1)} W`);
    setText("tel-en", `${(s.energy_J / 3600).toFixed(1)} Wh`);
  }

  // Push a sample onto the charts only. Used so the charts keep growing with
  // live data while the user is scrubbing.
  updateChartOnly(s: Snapshot) {
    push(this.depthChart, s.t, [s.depth_m]);
    push(this.powerChart, s.t, [s.power_W, s.soc * 100]);
  }

  setPlanSummary(distance_m: number, duration_s: number, energy_J: number, wp: number) {
    setText("plan-dist", `${(distance_m / 1000).toFixed(2)} km`);
    setText("plan-dur", fmtTime(duration_s));
    setText("plan-en", `${(energy_J / 3600).toFixed(1)} Wh`);
    setText("plan-wp", `${wp}`);
  }

  reset() {
    this.depthChart.t.length = 0;
    this.depthChart.series.forEach(s => s.length = 0);
    this.depthChart.plot.setData([this.depthChart.t, ...this.depthChart.series] as uPlot.AlignedData);

    this.powerChart.t.length = 0;
    this.powerChart.series.forEach(s => s.length = 0);
    this.powerChart.plot.setData([this.powerChart.t, ...this.powerChart.series] as uPlot.AlignedData);
  }
}

function push(c: RingChart, t: number, vals: number[]) {
  c.t.push(t);
  vals.forEach((v, i) => c.series[i].push(v));
  while (c.t.length > MAX_POINTS) {
    c.t.shift();
    c.series.forEach(s => s.shift());
  }
  c.plot.setData([c.t, ...c.series] as uPlot.AlignedData);
}

function setText(id: string, text: string) {
  const el = document.getElementById(id);
  if (el) el.textContent = text;
}

function fmtTime(seconds: number): string {
  const s = Math.max(0, Math.floor(seconds));
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  const sec = s % 60;
  if (h) return `${h}h ${m}m ${sec}s`;
  if (m) return `${m}m ${sec}s`;
  return `${sec}s`;
}

function normDeg(d: number): number {
  let r = d % 360;
  if (r < 0) r += 360;
  return r;
}

function stateLabel(s: Snapshot): string {
  if (!s.plan_loaded) return "no plan";
  if (s.grounded) return "GROUNDED";
  if (s.finished) return s.soc <= 0.03 ? "brownout" : "complete";
  if (s.running) return "running";
  return "paused";
}
