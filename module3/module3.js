// Module 3: the system equations and measurements are ours; Plotly draws the graphs.
const $ = id => document.getElementById(id);
const setText = (id, value) => { $(id).textContent = value; };
const css = name => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const massSlider = $("mass"), dampingSlider = $("damping"), stiffnessSlider = $("stiffness");
const STEP_FORCE = 1, SETTLING_FRACTION = 0.05;
let currentResponse, prediction;

// Exact step response and its derivative, starting at zero position and velocity.
function createSystem(m, b, k) {
  const omegaN = Math.sqrt(k / m), zeta = b / (2 * Math.sqrt(k * m));
  const alpha = b / (2 * m), finalPosition = STEP_FORCE / k;
  let kind, omegaD = null, normalizedPosition, normalizedVelocity, poles;
  if (Math.abs(zeta - 1) < 1e-9) {
    kind = "Critically damped";
    normalizedPosition = t => 1 - (1 + omegaN * t) * Math.exp(-omegaN * t);
    normalizedVelocity = t => omegaN ** 2 * t * Math.exp(-omegaN * t);
    poles = [{ real: -omegaN, imaginary: 0 }, { real: -omegaN, imaginary: 0 }];
  } else if (zeta < 1) {
    kind = b === 0 ? "Undamped" : "Underdamped";
    omegaD = omegaN * Math.sqrt(1 - zeta * zeta);
    normalizedPosition = t => 1 - Math.exp(-alpha * t) *
      (Math.cos(omegaD * t) + alpha / omegaD * Math.sin(omegaD * t));
    normalizedVelocity = t => omegaN ** 2 / omegaD * Math.exp(-alpha * t) * Math.sin(omegaD * t);
    poles = [{ real: -alpha, imaginary: omegaD }, { real: -alpha, imaginary: -omegaD }];
  } else {
    kind = "Overdamped";
    const root = Math.sqrt(zeta * zeta - 1);
    const slow = -omegaN / (zeta + root), fast = -omegaN * (zeta + root);
    normalizedPosition = t => 1 + (fast * Math.exp(slow * t) - slow * Math.exp(fast * t)) / (slow - fast);
    normalizedVelocity = t => slow * fast * (Math.exp(slow * t) - Math.exp(fast * t)) / (slow - fast);
    poles = [{ real: slow, imaginary: 0 }, { real: fast, imaginary: 0 }];
  }
  return { m, b, k, omegaN, zeta, alpha, omegaD, kind, finalPosition, normalizedPosition, normalizedVelocity, poles };
}

// Eigenvalues are calculated from A itself, independently of the transfer-function poles.
function createStateSpace(system) {
  const A = [[0, 1], [-system.k / system.m, -system.b / system.m]], B = [[0], [1 / system.m]];
  const eigenvalues = eigenvaluesOfA(A);
  const match = system.poles.every((p, i) => Math.hypot(p.real - eigenvalues[i].real,
    p.imaginary - eigenvalues[i].imaginary) < 1e-8 * Math.max(1, Math.hypot(p.real, p.imaginary)));
  return { A, B, eigenvalues, match };
}

function eigenvaluesOfA(A) {
  const halfTrace = (A[0][0] + A[1][1]) / 2;
  const determinant = A[0][0] * A[1][1] - A[0][1] * A[1][0];
  const discriminant = halfTrace ** 2 - determinant;
  if (Math.abs(-halfTrace / Math.sqrt(determinant) - 1) < 1e-9)
    return [{ real: halfTrace, imaginary: 0 }, { real: halfTrace, imaginary: 0 }];
  if (discriminant < 0) return [1, -1].map(sign => ({ real: halfTrace, imaginary: sign * Math.sqrt(-discriminant) }));
  const fast = halfTrace - Math.sqrt(discriminant);
  return [{ real: determinant / fast, imaginary: 0 }, { real: fast, imaginary: 0 }];
}

// Bisection finds an upward or downward crossing of a chosen position.
function crossingTime(positionAt, target, left, right) {
  let leftError = positionAt(left) - target;
  if (Math.abs(leftError) < 1e-14) return left;
  for (let i = 0; i < 65; i++) {
    const middle = (left + right) / 2, error = positionAt(middle) - target;
    if (Math.abs(error) < 1e-13) return middle;
    if (Math.sign(error) === Math.sign(leftError)) { left = middle; leftError = error; }
    else right = middle;
  }
  return (left + right) / 2;
}

function measureResponse(system) {
  const positionAt = system.normalizedPosition;
  const peakTime = system.omegaD === null ? null : Math.PI / system.omegaD;
  const overshoot = peakTime === null ? 0 : Math.exp(-system.alpha * peakTime);
  let right = peakTime === null ? 1 / system.omegaN : peakTime;
  while (positionAt(right) < 0.95) right *= 2;
  const t10 = crossingTime(positionAt, 0.1, 0, right), t90 = crossingTime(positionAt, 0.9, 0, right);
  let settlingTime = null;
  if (system.b !== 0) {
    if (peakTime === null) settlingTime = crossingTime(positionAt, 1 - SETTLING_FRACTION, 0, right);
    else {
      // Find the last extremum outside the band, then its crossing back inside.
      const halfPeriod = Math.PI / system.omegaD;
      const last = Math.max(0, Math.ceil(Math.log(1 / SETTLING_FRACTION) / (system.alpha * halfPeriod)) - 1);
      const target = 1 + (last % 2 === 0 ? -SETTLING_FRACTION : SETTLING_FRACTION);
      settlingTime = crossingTime(positionAt, target, last * halfPeriod, (last + 1) * halfPeriod);
    }
  }
  return { t10, t90, riseTime: t90 - t10, peakTime, settlingTime,
    peakPosition: peakTime === null ? null : system.finalPosition * (1 + overshoot), overshootPercent: 100 * overshoot };
}

// Both graphs sample the same formula. Phase samples also include velocity.
function sampleCurve(system, endTime, phase = false) {
  const cycles = system.omegaD === null ? 0 : endTime * system.omegaD / (2 * Math.PI);
  const count = Math.max(1600, Math.min(20000, Math.ceil(cycles * 40)));
  let times = Array.from({ length: count + 1 }, (_, i) => endTime * i / count);
  if (phase) {
    const fastest = Math.max(system.omegaN, ...system.poles.map(p => Math.hypot(p.real, p.imaginary)));
    const earlyEnd = Math.min(endTime, 6 / fastest);
    times.push(...Array.from({ length: 601 }, (_, i) => earlyEnd * i / 600));
    times = [...new Set(times)].sort((a, b) => a - b);
  }
  return times.map(t => {
    const point = { t, x: system.finalPosition * system.normalizedPosition(t) };
    if (phase) point.velocity = system.finalPosition * system.normalizedVelocity(t);
    return point;
  });
}

function niceStep(value) {
  const power = 10 ** Math.floor(Math.log10(value)), fraction = value / power;
  return (fraction <= 1 ? 1 : fraction <= 2 ? 2 : fraction <= 5 ? 5 : 10) * power;
}

function makePlotData(system, metrics) {
  const end = metrics.settlingTime === null ? 6 * Math.PI / system.omegaN
    : 1.25 * Math.max(metrics.settlingTime, metrics.t90, metrics.peakTime || 0);
  const step = niceStep(end / 5), endTime = Math.ceil(end / step) * step;
  return { endTime, points: sampleCurve(system, endTime) };
}

function makePhaseData(system, responseEnd) {
  const slowest = Math.min(...system.poles.map(p => -p.real));
  return sampleCurve(system, system.b === 0 ? responseEnd : Math.max(responseEnd, 8 / slowest), true);
}

const formatScalar = value => (Math.abs(value) < 0.0005 ? 0 : value).toFixed(3);
const formatTime = value => value.toFixed(3) + " s";
function formatComplex(value) {
  return formatScalar(value.real) + (Math.abs(value.imaginary) < 1e-12 ? ""
    : (value.imaginary >= 0 ? " + j" : " − j") + formatScalar(Math.abs(value.imaginary)));
}

// All text updates are kept together so the displayed numbers follow the model
function updateParameters() {
  const m = Number(massSlider.value), b = Number(dampingSlider.value), k = Number(stiffnessSlider.value);
  const system = createSystem(m, b, k), metrics = measureResponse(system), stateSpace = createStateSpace(system);
  const plotData = makePlotData(system, metrics), f = system.finalPosition;
  currentResponse = { system, metrics, stateSpace, ...plotData, phasePoints: makePhaseData(system, plotData.endTime) };
  const outputs = {
    massOut: m.toFixed(1) + " kg", dampingOut: b.toFixed(1) + " N·s/m", stiffnessOut: k.toFixed(1) + " N/m",
    omegaNOut: system.omegaN.toFixed(3) + " rad/s", zetaOut: system.zeta.toFixed(3), responseCase: system.kind,
    finalValueOut: f.toFixed(3) + " m", riseTimeOut: formatTime(metrics.riseTime),
    riseDetails: "10% at " + formatTime(metrics.t10) + "; 90% at " + formatTime(metrics.t90) + ".",
    peakTimeOut: metrics.peakTime === null ? "N/A" : formatTime(metrics.peakTime),
    peakDetails: metrics.peakTime === null ? "Monotonic response; there is no finite overshoot peak."
      : "Peak position: " + metrics.peakPosition.toFixed(3) + " m.",
    overshootOut: metrics.overshootPercent.toFixed(3) + "%",
    settlingTimeOut: metrics.settlingTime === null ? "Never" : formatTime(metrics.settlingTime),
    settlingDetails: metrics.settlingTime === null ? "With zero damping, the oscillations do not decay."
      : "The last crossing into the band, rather than its first entry.",
    a21Out: formatScalar(stateSpace.A[1][0]), a22Out: formatScalar(stateSpace.A[1][1]), b2Out: formatScalar(stateSpace.B[1][0]),
    accelerationEquation: `ẋ₂ = −${formatScalar(k / m)} x₁ − ${formatScalar(b / m)} x₂ + ${formatScalar(1 / m)} F(t)`,
    eigenMatchOut: stateSpace.match ? "Poles = eigenvalues of A. Both descriptions give the same natural motion."
      : "The pole and eigenvalue calculations do not agree.",
    responseSummary: `${system.kind}. The settling band is ${(f * 0.95).toFixed(4)}–${(f * 1.05).toFixed(4)} m. ` +
      (metrics.settlingTime === null ? "The plot shows three cycles." : "The time axis adapts to include the first peak and settling time.")
  };
  Object.entries(outputs).forEach(([id, value]) => setText(id, value));
  system.poles.forEach((p, i) => {
    setText(`pole${i + 1}Out`, formatComplex(p));
    setText(`eigen${i + 1}Out`, formatComplex(stateSpace.eigenvalues[i]));
  });
  const descriptions = {
    Undamped: "The poles lie on the imaginary axis. Oscillations continue without decay.",
    Underdamped: "Complex-conjugate poles with negative real parts produce decaying oscillations.",
    "Critically damped": "The two poles coincide at one negative real location. The ×2 label indicates a double pole.",
    Overdamped: "Two different negative real poles produce a response without oscillation."
  };
  setText("poleSummary", descriptions[system.kind]);
  const restingPoint = `(${formatScalar(f)} m, 0 m/s)`;
  setText("phaseSummary", b === 0 ? `With zero damping, the trajectory follows a closed orbit around ${restingPoint}. It does not settle.`
    : system.omegaD !== null ? `The trajectory spirals toward ${restingPoint} as the oscillations decay.`
      : `The trajectory approaches ${restingPoint} without spiraling.`);
  updatePrediction();
  drawAllPlots();
}

// Plotly handles axes, ticks, clipping, sizing, and the actual drawing.
const line = (points, color) => ({ type: "scatter", mode: "lines", x: points.map(p => p[0]),
  y: points.map(p => p[1]), line: { color, width: 2.5 }, hoverinfo: "skip" });
const dots = (points, color, symbol = "circle", size = 9) => ({ ...line(points, color), mode: "markers",
  marker: { color, symbol, size, line: { color, width: 2 } } });
const segment = (x0, y0, x1, y1, color) => ({ type: "line", x0, y0, x1, y1, line: { color, dash: "dash", width: 1.5 } });

function chart(id, data, xLabel, yLabel, extra = {}) {
  const axis = title => ({ title: { text: title, standoff: 8 }, color: css("--muted"), gridcolor: css("--line"),
    zerolinecolor: css("--muted"), linecolor: css("--line"), showline: true, mirror: true,
    fixedrange: true, automargin: true, nticks: $(id).clientWidth < 400 ? 4 : 6 });
  return Plotly.react(id, data, { margin: { l: 64, r: 16, t: 20, b: 48 },
    font: { family: "system-ui, Arial, sans-serif", size: 12, color: css("--ink") },
    paper_bgcolor: "rgba(0,0,0,0)", plot_bgcolor: "rgba(0,0,0,0)", showlegend: false, ...extra,
    xaxis: { ...axis(xLabel), ...extra.xaxis }, yaxis: { ...axis(yLabel), ...extra.yaxis }
  }, { responsive: true, staticPlot: true, displayModeBar: false });
}

function drawResponsePlot() {
  const { system: s, metrics: r, points, endTime } = currentResponse, f = s.finalPosition;
  const yMax = Math.max(r.peakPosition || f, f * 1.05) * 1.16;
  // A marker contains [time, position, color, full-height line].
  const markers = [[r.t10, f * 0.1, css("--focus")], [r.t90, f * 0.9, css("--focus")]];
  if (r.peakTime !== null) markers.push([r.peakTime, r.peakPosition, css("--peak")]);
  if (r.settlingTime !== null) markers.push([r.settlingTime, f * s.normalizedPosition(r.settlingTime), css("--settle"), true]);
  const band = { type: "rect", x0: 0, x1: endTime, y0: f * 0.95, y1: f * 1.05,
    fillcolor: css("--band"), line: { width: 0 }, layer: "below" };
  return chart("responsePlot", [line(points.map(p => [p.t, p.x]), css("--curve")),
    ...markers.map(([t, x, color]) => dots([[t, x]], color))], "Time (s)", "Position x (m)", {
    xaxis: { range: [0, endTime] }, yaxis: { range: [0, yMax] },
    shapes: [band, segment(0, f, endTime, f, css("--muted")),
      ...markers.map(([t, x, color, full]) => segment(t, 0, t, full ? yMax : x, color))]
  });
}

function paddedDomain(values) {
  const low = Math.min(...values), high = Math.max(...values), span = high - low || Math.max(Math.abs(low), 1);
  return [low - span * 0.12, high + span * 0.12];
}

function drawPolePlot() {
  const { system: s, stateSpace } = currentResponse;
  const poles = s.poles.map(p => [p.real, p.imaginary]), eigenvalues = stateSpace.eigenvalues.map(p => [p.real, p.imaginary]);
  const repeated = Math.hypot(poles[0][0] - poles[1][0], poles[0][1] - poles[1][1]) < 1e-10 * Math.max(1, s.omegaN);
  return chart("polePlot", [dots(eigenvalues, css("--settle"), "circle-open", 15), dots(poles, css("--curve"), "x", 8)],
    "Real part Re(s) (1/s)", "Imaginary part Im(s) (1/s)", {
      xaxis: { range: paddedDomain([0, ...poles.map(p => p[0])]) },
      yaxis: { range: paddedDomain([0, ...poles.map(p => p[1])]), scaleanchor: "x", scaleratio: 1 },
      annotations: repeated ? [{ x: poles[0][0], y: poles[0][1], text: "×2", showarrow: false, xshift: 18, yshift: 14 }] : []
    });
}

// Arrowheads follow increasing time at three positions along the phase path.
function directionArrows(points, xRange, yRange) {
  const lengths = [0], dx = xRange[1] - xRange[0], dy = yRange[1] - yRange[0];
  for (let i = 1; i < points.length; i++) lengths.push(lengths[i - 1] +
    Math.hypot((points[i].x - points[i - 1].x) / dx, (points[i].velocity - points[i - 1].velocity) / dy));
  const total = lengths[lengths.length - 1];
  return [0.2, 0.5, 0.8].map(fraction => {
    const head = points[lengths.findIndex(length => length >= fraction * total)];
    const tail = points[lengths.findIndex(length => length >= (fraction - 0.015) * total)];
    return { x: head.x, y: head.velocity, ax: tail.x, ay: tail.velocity, xref: "x", yref: "y", axref: "x", ayref: "y",
      text: "", showarrow: true, arrowhead: 2, arrowsize: 1.2, arrowwidth: 1.5, arrowcolor: css("--curve") };
  });
}

function drawPhasePlot() {
  const { system: s, phasePoints: points } = currentResponse;
  const xRange = paddedDomain([0, s.finalPosition, ...points.map(p => p.x)]), yRange = paddedDomain([0, ...points.map(p => p.velocity)]);
  return chart("phasePlot", [line(points.map(p => [p.x, p.velocity]), css("--curve")),
    dots([[0, 0]], css("--focus")), dots([[s.finalPosition, 0]], css("--settle"), "circle-open", 14)],
    "Position x₁ (m)", "Velocity x₂ (m/s)", {
      xaxis: { range: xRange }, yaxis: { range: yRange }, annotations: directionArrows(points, xRange, yRange)
    });
}

function drawAllPlots() {
  return Promise.all([drawResponsePlot(), drawPolePlot(), drawPhasePlot()]);
}

// Save the baseline before the user moves one slider
const parameterNames = ["m", "b", "k"];
const parameterValues = () => [massSlider, dampingSlider, stiffnessSlider].map(slider => Number(slider.value));
const roundedOvershoot = metrics => Number(metrics.overshootPercent.toFixed(3));
function updatePrediction() {
  if (!prediction) return;
  const changes = parameterValues().filter((value, i) => value !== prediction.values[i]).length;
  $("checkPrediction").disabled = changes !== 1;
  setText("predictionResult", "");
  setText("predictionHint", changes === 0 ? "Now change one slider above."
    : changes === 1 ? "One parameter changed. Check your prediction when ready."
      : "More than one parameter changed. Restore the others to the saved values, or save a new prediction.");
}

$("predictionForm").addEventListener("submit", event => {
  event.preventDefault();
  const choice = new FormData(event.currentTarget).get("prediction");
  if (!choice) return;
  prediction = { choice, values: parameterValues(), overshoot: roundedOvershoot(currentResponse.metrics), zeta: currentResponse.system.zeta };
  setText("predictionBaseline", `Saved: ${choice === "same" ? "the same" : choice} overshoot. Baseline: ` +
    prediction.values.map((value, i) => `${parameterNames[i]} = ${value.toFixed(1)}`).join(", ") +
    `. Overshoot = ${prediction.overshoot.toFixed(3)}%.`);
  updatePrediction();
});

$("checkPrediction").addEventListener("click", () => {
  if (!prediction || $("checkPrediction").disabled) return;
  const values = parameterValues(), index = values.findIndex((value, i) => value !== prediction.values[i]);
  const overshoot = roundedOvershoot(currentResponse.metrics);
  const actual = overshoot > prediction.overshoot ? "higher" : overshoot < prediction.overshoot ? "lower" : "same";
  setText("predictionResult", `${actual === prediction.choice ? "Correct!" : "Try again."} Overshoot is ${actual === "same" ? "the same" : actual}: ` +
    `${prediction.overshoot.toFixed(3)}% → ${overshoot.toFixed(3)}%. ` +
    `${parameterNames[index]}: ${prediction.values[index].toFixed(1)} → ${values[index].toFixed(1)}; ` +
    `ζ: ${prediction.zeta.toFixed(3)} → ${currentResponse.system.zeta.toFixed(3)}. ` +
    "Below critical damping, a larger ζ means less overshoot. At ζ ≥ 1, overshoot is zero.");
});


function runValidationChecks() {
  const under = createSystem(1, 2, 4), critical = createSystem(1, 4, 4), state = createStateSpace(under);
  const underOvershoot = measureResponse(under).overshootPercent, criticalOvershoot = measureResponse(critical).overshootPercent;
  const expectedPoles = [{ real: -1, imaginary: 1.732 }, { real: -1, imaginary: -1.732 }];
  const closeToExpected = values => values.every((value, i) => Math.abs(value.real - expectedPoles[i].real) < 0.0005 &&
    Math.abs(value.imaginary - expectedPoles[i].imaginary) < 0.0005);
  const checks = [
    ["4. Overshoot at ζ = 0.5\nm = 1, b = 2, k = 4", "16.3%", underOvershoot.toFixed(3) + "%", Math.abs(underOvershoot - 16.3) < 0.05],
    ["5. Overshoot at ζ = 1\nm = 1, b = 4, k = 4", "0% (no overshoot)", criticalOvershoot.toFixed(3) + "%", Math.abs(criticalOvershoot) < 1e-9],
    ["6. Poles and eigenvalues\nm = 1, b = 2, k = 4", "Both: −1 ± j1.732", "Poles: " + under.poles.map(formatComplex).join("; ") +
      "\nEigenvalues: " + state.eigenvalues.map(formatComplex).join("; "), closeToExpected(under.poles) && closeToExpected(state.eigenvalues) && state.match]
  ];
  $("validationBody").replaceChildren(...checks.map(([name, expected, calculated, pass]) => {
    const row = document.createElement("tr");
    [name, expected, calculated, pass ? "Pass" : "Fail"].forEach((text, i) => {
      const cell = document.createElement(i === 0 ? "th" : "td");
      cell.textContent = text;
      if (i === 0) cell.scope = "row";
      if (i === 3) cell.className = pass ? "pass" : "fail";
      row.append(cell);
    });
    return row;
  }));
}

[massSlider, dampingSlider, stiffnessSlider].forEach(slider => slider.addEventListener("input", updateParameters));
document.querySelectorAll("[data-damping]").forEach(button => button.addEventListener("click", () => {
  massSlider.value = "1"; stiffnessSlider.value = "4"; dampingSlider.value = button.dataset.damping;
  updateParameters();
}));
window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", drawAllPlots);
updateParameters();
runValidationChecks();
