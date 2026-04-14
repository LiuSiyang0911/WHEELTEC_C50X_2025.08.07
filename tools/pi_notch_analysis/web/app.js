const form = document.getElementById("analysis-form");
const statusEl = document.getElementById("status");
const summaryEl = document.getElementById("summary-cards");
const traceCheckboxes = Array.from(document.querySelectorAll("[data-trace]"));
const spectrumCheckboxes = Array.from(document.querySelectorAll("[data-spectrum]"));
let latestPayload = null;

function setStatus(message, isError = false) {
  statusEl.textContent = message;
  statusEl.dataset.error = isError ? "true" : "false";
}

function renderSummary(summary) {
  const items = [
    ["Source", summary.source_name],
    ["Motor", summary.motor.toUpperCase()],
    ["Disturbance", `${summary.dominant_disturbance_hz.toFixed(3)} Hz`],
    ["Recommended Gain", summary.recommended_gain.toFixed(3)],
    ["Bandwidth", `${summary.recommended_bandwidth_hz.toFixed(3)} Hz`],
    ["Time Constant", `${summary.recommended_time_constant_s.toFixed(3)} s`],
    ["Error Reduction", `${summary.error_reduction_pct.toFixed(1)} %`],
    ["PI Reduction", `${summary.pi_reduction_pct.toFixed(1)} %`],
    ["Steady Window", `${summary.steady_start_s.toFixed(2)} - ${summary.steady_stop_s.toFixed(2)} s`],
  ];

  summaryEl.classList.remove("empty");
  summaryEl.innerHTML = items
    .map(([label, value]) => `<div class="card"><span>${label}</span><strong>${value}</strong></div>`)
    .join("");
}

function selectedValues(elements, key) {
  return elements.filter((element) => element.checked).map((element) => element.dataset[key]);
}

function renderTimeSeries(chart) {
  const enabled = new Set(selectedValues(traceCheckboxes, "trace"));
  const traces = [
    ["feedback", { name: "Feedback Speed", mode: "lines" }],
    ["notched_feedback", { name: "Notched Feedback", mode: "lines" }],
    ["raw_error", { name: "Raw Error", mode: "lines" }],
    ["filtered_error", { name: "Notched Error", mode: "lines" }],
    ["raw_pi", { name: "PI Output (Raw Error)", mode: "lines", yaxis: "y2" }],
    ["filtered_pi", { name: "PI Output (Notched Error)", mode: "lines", yaxis: "y2" }],
  ]
    .filter(([key]) => enabled.has(key))
    .map(([key, meta]) => ({ x: chart[key].x, y: chart[key].y, ...meta }));

  Plotly.newPlot(
    "time-series-chart",
    traces,
    {
      margin: { t: 20, r: 50, b: 40, l: 50 },
      xaxis: { title: "Time (s)" },
      yaxis: { title: "Speed / Error (m/s)" },
      yaxis2: { title: "PI Output (PWM)", overlaying: "y", side: "right" },
      legend: { orientation: "h" },
    },
    { responsive: true }
  );
}

function renderSpectrum(chart, summary) {
  const enabled = new Set(selectedValues(spectrumCheckboxes, "spectrum"));
  const traces = [];
  if (enabled.has("feedback")) {
    traces.push({ x: chart.feedback.frequency_hz, y: chart.feedback.amplitude, name: "Feedback Spectrum", mode: "lines" });
  }
  if (enabled.has("notched_feedback")) {
    traces.push({ x: chart.notched_feedback.frequency_hz, y: chart.notched_feedback.amplitude, name: "Notched Feedback Spectrum", mode: "lines" });
  }
  if (enabled.has("error")) {
    traces.push({ x: chart.error.frequency_hz, y: chart.error.amplitude, name: "Error Spectrum", mode: "lines" });
  }
  const maxAmplitude = Math.max(
    1e-6,
    ...traces.flatMap((trace) => trace.y)
  );
  traces.push({
    x: [summary.dominant_disturbance_hz, summary.dominant_disturbance_hz],
    y: [0, maxAmplitude],
    name: "Dominant Disturbance",
    mode: "lines",
    line: { dash: "dash" },
  });

  Plotly.newPlot(
    "spectrum-chart",
    traces,
    {
      margin: { t: 20, r: 20, b: 40, l: 50 },
      xaxis: { title: "Frequency (Hz)" },
      yaxis: { title: "Amplitude" },
      legend: { orientation: "h" },
    },
    { responsive: true }
  );
  document.getElementById("spectrum-chart").on("plotly_click", (event) => {
    const point = event?.points?.[0];
    if (!point || typeof point.x !== "number") return;
    const input = form.querySelector('input[name="speed_notch_freq_hz"]');
    input.value = point.x.toFixed(3);
    setStatus(`Selected ${point.x.toFixed(3)} Hz from spectrum. Adjust if needed, then Analyze.`);
  });
}

function renderGainSweep(chart) {
  Plotly.newPlot(
    "gain-chart",
    [
      { x: chart.bandwidth_hz, y: chart.error_reduction_pct, name: "Error Reduction", mode: "lines+markers" },
      { x: chart.bandwidth_hz, y: chart.pi_reduction_pct, name: "PI Reduction", mode: "lines+markers" },
    ],
    {
      margin: { t: 20, r: 20, b: 40, l: 50 },
      xaxis: { title: "Bandwidth (Hz)" },
      yaxis: { title: "Reduction (%)" },
      legend: { orientation: "h" },
    },
    { responsive: true }
  );
}

form.addEventListener("submit", async (event) => {
  event.preventDefault();
  const formData = new FormData(form);
  const speedNotchEnabled = form.querySelector('input[name="speed_notch_enabled"]').checked;
  formData.set("speed_notch_enabled", speedNotchEnabled ? "true" : "false");
  setStatus("Analyzing CSV...");

  try {
    const response = await fetch("/api/analyze", { method: "POST", body: formData });
    const payload = await response.json();
    if (!response.ok) {
      throw new Error(payload.detail || "Analysis failed");
    }
    latestPayload = payload;
    renderSummary(payload.summary);
    renderTimeSeries(payload.charts.time_series);
    renderSpectrum(payload.charts.spectrum, payload.summary);
    renderGainSweep(payload.charts.gain_sweep);
    setStatus(`Analysis complete. Recommended gain = ${payload.summary.recommended_gain.toFixed(3)}.`);
  } catch (error) {
    setStatus(error.message, true);
  }
});

function rerenderFromLatest() {
  if (!latestPayload) return;
  renderTimeSeries(latestPayload.charts.time_series);
  renderSpectrum(latestPayload.charts.spectrum, latestPayload.summary);
}

traceCheckboxes.forEach((element) => element.addEventListener("change", rerenderFromLatest));
spectrumCheckboxes.forEach((element) => element.addEventListener("change", rerenderFromLatest));
