/* P2's fixed-step RK4 model. This file also exports the model for Node tests.
   State: [true output y, velocity v, integral I, sensor output ym, filtered error q]. */
(function (root) {
  'use strict';
  const TF = .02; // The same derivative filter time constant used in P1, in seconds.
  const TAW = .1; // Anti-windup tracking time, in seconds.
  const DT = .00005; // Fixed RK4 step: 0.05 ms, including the fastest slider settings.
  const defaults = { type: 1, K: 1, tau: 1, wn: 2, zeta: .5, Kp: 1, Ki: 5, Kd: 0,
    sensorTau: .05, r: 1, umin: -100, umax: 100, T: 30, aw: true };

  function parameters(input) {
    const p = Object.assign({}, defaults, input);
    const keys = ['type', 'K', 'tau', 'wn', 'zeta', 'Kp', 'Ki', 'Kd', 'sensorTau', 'r', 'umin', 'umax', 'T'];
    if (keys.some(k => !Number.isFinite(p[k])) || ![1, 2].includes(p.type) ||
        p.K <= 0 || p.tau <= 0 || p.wn <= 0 || p.zeta < 0 || p.Kp < 0 || p.Ki < 0 || p.Kd < 0 ||
        p.sensorTau < 0 || p.r <= 0 || p.umin >= p.umax || p.T <= 0 || p.T > 120) {
      throw new RangeError('Invalid plant, sensor, controller, or simulation parameters.');
    }
    return p;
  }

  function signals(s, p) {
    // Zero sensor lag is an ideal measurement, not division by a tiny time constant.
    const ym = p.sensorTau === 0 ? s[0] : s[3];
    const e = p.r - ym;
    const raw = p.Kp * e + p.Ki * s[2] + p.Kd * (e - s[4]) / TF;
    return { ym, e, raw, u: Math.min(p.umax, Math.max(p.umin, raw)) };
  }

  function rhs(s, p) {
    const a = signals(s, p);
    // Back-calculation unwinds the integral when the requested command is
    // outside the actuator's range. The tracking time is TAW seconds.
    const dI = p.Ki === 0 ? 0 : a.e + (p.aw ? (a.u - a.raw) / (p.Ki * TAW) : 0);
    return [
      p.type === 1 ? (p.K * a.u - s[0]) / p.tau : s[1],
      p.type === 1 ? 0 : p.wn * p.wn * (p.K * a.u - s[0]) - 2 * p.zeta * p.wn * s[1],
      dI,
      p.sensorTau === 0 ? 0 : (s[0] - s[3]) / p.sensorTau,
      p.Kd === 0 ? 0 : (a.e - s[4]) / TF
    ];
  }

  function rk4(s, h, p) {
    const shifted = (k, amount) => s.map((v, j) => v + amount * k[j]);
    const a = rhs(s, p), b = rhs(shifted(a, h / 2), p);
    const c = rhs(shifted(b, h / 2), p), d = rhs(shifted(c, h), p);
    return s.map((v, j) => v + h * (a[j] + 2 * b[j] + 2 * c[j] + d[j]) / 6);
  }

  function simulate(input, dt = DT) {
    const p = parameters(input);
    if (!Number.isFinite(dt) || dt <= 0) throw new RangeError('Invalid fixed integration step.');
    const n = Math.ceil(p.T / dt);
    if (!Number.isSafeInteger(n) || n > 5000000) throw new RangeError('Invalid integration step: too many steps requested.');
    // Draw at most 3,001 samples, but calculate metrics at every RK4 step.
    const stride = Math.max(1, Math.ceil(n / 3000));
    const out = { t: [], y: [], ym: [], u: [], status: 'ok', steps: 0, minU: Infinity, maxU: -Infinity };
    let s = [0, 0, 0, 0, 0], time = 0;
    let t10 = null, t90 = null, peak = 0, gap = 0, settling = null, previous = { t: 0, y: 0 };

    // Metrics use every RK4 step, not just the points drawn on canvas.
    function observe(t, state) {
      const a = signals(state, p), y = state[0];
      const crossing = fraction => {
        const target = fraction * p.r;
        return y === previous.y ? t : previous.t + (t - previous.t) * (target - previous.y) / (y - previous.y);
      };
      if (t10 === null && y >= .1 * p.r) t10 = crossing(.1);
      if (t90 === null && y >= .9 * p.r) t90 = crossing(.9);
      peak = Math.max(peak, y); gap = Math.max(gap, Math.abs(y - a.ym));
      out.minU = Math.min(out.minU, a.u); out.maxU = Math.max(out.maxU, a.u);
      if (Math.abs(y - p.r) > .02 * p.r) settling = null;
      else if (settling === null) settling = t;
      previous = { t, y };
    }
    function sample() {
      const a = signals(s, p);
      out.t.push(time); out.y.push(s[0]); out.ym.push(a.ym); out.u.push(a.u);
    }
    observe(0, s); sample();

    for (let i = 1; i <= n; i++) {
      // Every step is dt; only a final fractional step is trimmed to end at T.
      const h = Math.min(dt, p.T - (i - 1) * dt);
      const next = rk4(s, h, p);
      if (next.some(v => !Number.isFinite(v) || Math.abs(v) > 1e7 * Math.max(1, p.r))) {
        out.status = 'limit'; break;
      }
      s = next; time = Math.min(p.T, i * dt); out.steps = i;
      observe(time, s);
      if (i % stride === 0 || i === n) sample();
    }
    if (out.t[out.t.length - 1] !== time) sample();
    const final = signals(s, p);
    out.metrics = {
      rise: t10 === null || t90 === null ? null : t90 - t10,
      overshoot: Math.max(0, (peak - p.r) / p.r * 100),
      settling: out.status === 'ok' && settling !== null && time - settling >= .5 ? settling : null,
      error: (p.r - s[0]) / p.r * 100,
      measuredError: (p.r - final.ym) / p.r * 100,
      gap, peak, endTime: time
    };
    return out;
  }

  function validationChecks() {
    const rows = [];
    const add = (name, expected, value, pass) => rows.push({ name, expected, value, pass });
    const worst = (r, key, exact) => Math.max(...r[key].map((v, i) => Math.abs(v - exact(r.t[i]))));
    let r = simulate({ Kp: 9, Ki: 0, sensorTau: 0, T: 2 });
    let error = worst(r, 'y', t => .9 * (1 - Math.exp(-10 * t)));
    add('Ideal sensor: P-only output matches the exact response', 'Max difference < 0.00001', error.toExponential(2), error < 1e-5);
    add('Ideal sensor: P-only final tracking error', '10.00 %', r.metrics.error.toFixed(2) + ' %', Math.abs(r.metrics.error - 10) < .01);
    r = simulate({ Kp: 1, Ki: 0, sensorTau: 1, T: 4 });
    error = worst(r, 'y', t => .5 + .5 * Math.exp(-t) * (Math.sin(t) - Math.cos(t)));
    add('Lagged feedback: true output matches the exact response', 'Max difference < 0.00001', error.toExponential(2), error < 1e-5);
    error = worst(r, 'ym', t => .5 * (1 - Math.exp(-t) * (Math.cos(t) + Math.sin(t))));
    add('Lagged feedback: sensor output matches the exact response', 'Max difference < 0.00001', error.toExponential(2), error < 1e-5);
    const fast = simulate({ sensorTau: .05 }), slow = simulate({ sensorTau: .5 });
    add('Demonstration gains: a slower sensor increases true overshoot', 'Slow > fast', fast.metrics.overshoot.toFixed(1) + ' % → ' + slow.metrics.overshoot.toFixed(1) + ' %', slow.metrics.overshoot > fast.metrics.overshoot);
    const p = { K: 5, tau: .1, Kp: 9, Ki: 0, Kd: 1, sensorTau: 0, T: 6 };
    r = simulate(p);
    const finer = simulate(p, DT / 2);
    error = Math.max(...r.y.map((v, i) => Math.abs(v - finer.y[i])));
    add('Fast plant with D: smaller steps preserve the output', 'Max difference < 0.001', error.toExponential(2), error < .001);
    return rows;
  }

  const api = { simulate, validationChecks, DT, TF, TAW };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.SensorPID = api;
})(typeof globalThis === 'undefined' ? this : globalThis);

// DOM controls run only in the page; Node tests use the model above.
if (typeof document !== 'undefined') {
  (function initPage() {
    /* Page controls and canvas rendering. The numerical model is above. */
    'use strict';
    const $ = id => document.getElementById(id);
    const SL = {
      K: ['Plant gain K', .1, 5, .1, 1, 1, '', 'Output per unit input at steady state.'],
      tau: ['Time constant τ', .1, 5, .05, 1, 2, ' s', 'Bigger τ is a slower plant.'],
      K2: ['Plant gain K', .1, 5, .1, 1, 1, '', 'Output per unit input at steady state.'],
      wn: ['Natural frequency ωn', .5, 10, .1, 2, 1, ' rad/s', 'Higher ωn is a faster plant.'],
      zeta: ['Damping ratio ζ', .1, 2, .05, .5, 2, '', 'Below 1 the uncontrolled plant rings.'],
      sensorTau: ['Sensor time constant τs', 0, 3, .01, .05, 2, ' s', 'Bigger τs means the measurement is further behind the true output.'],
      Kp: ['Kp (proportional)', 0, 20, .1, 1, 1, '', 'Acts on the present measured error.'],
      Ki: ['Ki (integral)', 0, 20, .1, 5, 1, '', 'Accumulates measured error; can remove a steady offset in a stable loop.'],
      Kd: ['Kd (derivative)', 0, 5, .05, 0, 2, '', 'Reacts to how quickly the measured error changes.'],
      r: ['Setpoint r', .1, 10, .1, 1, 1, '', 'Size of the step at t = 0.'],
      umax: ['u max', 1, 200, 1, 10, 0, '', 'Upper limit of the control signal.'],
      umin: ['u min', -200, 0, 1, -10, 0, '', 'Lower limit of the control signal.'],
      T: ['Simulation time', 2, 60, 1, 30, 0, ' s', 'Allow enough time to see whether the oscillations decay or grow.']
    };
    const demo = { K: 1, tau: 1, K2: 1, wn: 2, zeta: .5, Kp: 1, Ki: 5, Kd: 0,
      sensorTau: .05, r: 1, umin: -10, umax: 10, T: 30 };
    let type = 1, last = null, active = null, timer;
    const value = id => +$(id).value;
    const fmt = (n, digits = 2) => n === null || !Number.isFinite(n) ? '—' : (Math.abs(n) < .5 * 10 ** -digits ? 0 : n).toFixed(digits);
    const slider = id => {
      const d = SL[id];
      return `<div class="sl"><label for="${id}">${d[0]} <output id="${id}Out" for="${id}"></output></label><input id="${id}" type="range" min="${d[1]}" max="${d[2]}" step="${d[3]}" value="${d[4]}" aria-describedby="${id}Hint"><small id="${id}Hint">${d[7]}</small></div>`;
    };
    $('p1sl').innerHTML = ['K', 'tau'].map(slider).join('');
    $('p2sl').innerHTML = ['K2', 'wn', 'zeta'].map(slider).join('');
    $('sensorSl').innerHTML = slider('sensorTau');
    $('ctlsl').innerHTML = ['Kp', 'Ki', 'Kd', 'r', 'umax', 'umin', 'T'].map(slider).join('');
    
    function params() {
      const p = { type, K: type === 1 ? value('K') : value('K2'), aw: $('aw').checked };
      ['tau', 'wn', 'zeta', 'sensorTau', 'Kp', 'Ki', 'Kd', 'r', 'umin', 'umax', 'T'].forEach(id => p[id] = value(id));
      return p;
    }
    function readouts() {
      Object.keys(SL).forEach(id => $(id + 'Out').textContent = value(id).toFixed(SL[id][5]) + SL[id][6]);
    }
    function plantType(t) {
      type = t;
      $('b1').setAttribute('aria-pressed', String(t === 1));
      $('b2').setAttribute('aria-pressed', String(t === 2));
      $('p1sl').hidden = t !== 1; $('p2sl').hidden = t !== 2;
    }
    function loadDemo(lag) {
      Object.entries({ ...demo, sensorTau: lag }).forEach(([key, v]) => $(key).value = v);
      $('aw').checked = true; plantType(1); requestDraw(true);
    }
    
    function requestDraw(immediate = false) {
      readouts(); clearTimeout(timer);
      timer = setTimeout(compute, immediate ? 0 : 65);
    }
    function compute() {
      const p = params();
      $('compute').textContent = 'Updating the simulation…';
      $('warn').textContent = '';
      $('c1').parentElement.setAttribute('aria-busy', 'true');
      try {
        last = SensorPID.simulate(p); active = p;
        render();
        $('compute').textContent = 'Simulation updated. The PID uses the measured output ym.';
      } catch (error) {
        $('warn').textContent = error.message;
        $('compute').textContent = 'No new result. Check the settings and try again.';
      } finally {
        $('c1').parentElement.setAttribute('aria-busy', 'false');
      }
    }

    function colours() {
      const style = getComputedStyle(document.documentElement), get = key => style.getPropertyValue(key).trim();
      return { ink: get('--ink'), muted: get('--muted'), line: get('--line'), y: get('--curve'), ym: get('--mark'), limit: get('--mark2') };
    }
    function plot(canvas, series, reference, label) {
      if (!last || !canvas.clientWidth) return;
      const C = colours(), ratio = window.devicePixelRatio || 1;
      const w = canvas.clientWidth, h = canvas.clientHeight, L = 54, R = 16, top = 18, bottom = 46;
      canvas.width = Math.round(w * ratio); canvas.height = Math.round(h * ratio);
      const c = canvas.getContext('2d'); c.setTransform(ratio, 0, 0, ratio, 0, 0);
      let lo = 0, hi = 0;
      for (const s of series) for (const v of s.values) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
      for (const s of reference) { lo = Math.min(lo, s.value); hi = Math.max(hi, s.value); }
      const pad = Math.max(hi - lo, .1) * .08;
      hi += pad; if (lo < 0) lo -= pad;
      const pw = w - L - R, ph = h - top - bottom;
      const X = v => L + v / active.T * pw, Y = v => top + (hi - v) / (hi - lo) * ph;
      c.font = '12px system-ui,Arial,sans-serif'; c.lineWidth = 1;
      for (let i = 0; i <= 5; i++) {
        const t = active.T * i / 5;
        c.strokeStyle = C.line; c.beginPath(); c.moveTo(X(t), top); c.lineTo(X(t), top + ph); c.stroke();
        c.fillStyle = C.muted; c.textAlign = 'center'; c.fillText(String(+t.toFixed(1)), X(t), top + ph + 16);
      }
      for (let i = 0; i <= 4; i++) {
        const v = lo + (hi - lo) * i / 4;
        c.strokeStyle = C.line; c.beginPath(); c.moveTo(L, Y(v)); c.lineTo(L + pw, Y(v)); c.stroke();
        c.fillStyle = C.muted; c.textAlign = 'right';
        c.fillText(Math.abs(v) >= 10000 ? v.toExponential(0) : v.toFixed(hi - lo < 5 ? 2 : 0), L - 7, Y(v) + 4);
      }
      c.fillStyle = C.ink; c.textAlign = 'center'; c.fillText('Time (s)', L + pw / 2, top + ph + 36);
      c.save(); c.translate(12, top + ph / 2); c.rotate(-Math.PI / 2); c.fillText(label, 0, 0); c.restore();
      c.save(); c.beginPath(); c.rect(L, top, pw, ph); c.clip();
      for (const s of reference) {
        c.strokeStyle = s.colour; c.lineWidth = 2; c.setLineDash([6, 5]);
        c.beginPath(); c.moveTo(L, Y(s.value)); c.lineTo(L + pw, Y(s.value)); c.stroke();
      }
      for (const s of series) {
        c.strokeStyle = s.colour; c.lineWidth = s.dashed ? 2 : 3; c.setLineDash(s.dashed ? [5, 4] : []);
        c.beginPath(); s.values.forEach((v, i) => i ? c.lineTo(X(last.t[i]), Y(v)) : c.moveTo(X(last.t[i]), Y(v))); c.stroke();
      }
      c.restore();
    }
    function renderPlots() {
      if (!last) return;
      const C = colours();
      plot($('c1'), [{ values: last.y, colour: C.y }, { values: last.ym, colour: C.ym, dashed: true }], [{ value: active.r, colour: C.muted }], 'Output');
      plot($('c2'), [{ values: last.u, colour: C.ym }], [{ value: active.umax, colour: C.limit }, { value: active.umin, colour: C.limit }], 'Control signal u');
    }
    function render() {
      renderPlots();
      const m = last.metrics, limited = last.maxU >= active.umax - 1e-8 || last.minU <= active.umin + 1e-8;
      $('warn').textContent = last.status === 'ok' ? '' : 'This run exceeded the numerical safety limit. The curves stop early; reduce the gains or shorten the run.';
      $('controlNote').textContent = 'Control range reached: ' + fmt(last.minU) + ' to ' + fmt(last.maxU) + (limited ? '. The actuator hits a limit.' : '. No actuator saturation in this run.');
      const cards = [
        ['a', 'Rise time (10–90%)', m.rise === null ? 'not reached' : fmt(m.rise, 3) + ' s', 'True output, relative to the setpoint'],
        ['b', 'True overshoot', fmt(m.overshoot, 1) + ' %', 'Peak above the setpoint'],
        ['c', 'Settling time (2%)', m.settling === null ? 'not confirmed' : fmt(m.settling) + ' s', 'Within the band through the rest of this run'],
        ['a', 'Final tracking error', fmt(m.error) + ' %', 'Signed true error at the end; negative means above target']
      ];
      $('cards').innerHTML = cards.map(x => `<div class="${x[0]}"><span>${x[1]}</span><b>${x[2]}</b><small>${x[3]}</small></div>`).join('');
      $('measurementNote').textContent = 'At t = ' + fmt(m.endTime, 1) + ' s: y = ' + fmt(last.y.at(-1), 3) + ', ym = ' + fmt(last.ym.at(-1), 3) + '. Measured error = ' + fmt(m.measuredError) + ' %. Largest |y − ym| during the run = ' + fmt(m.gap, 3) + '.';
      $('cursorReadout').textContent = 'Move over the plot to read a point. At τs = 0, the blue and orange curves overlap.';
      $('bdp').textContent = active.type === 1 ? 'K/(τ·s + 1)' : 'Kωn²/(s²+2ζωn·s+ωn²)';
      $('plantEquation').textContent = active.type === 1 ? 'ẏ = (K·u − y)/τ' : 'ẏ = v, v̇ = ωn²(K·u − y) − 2ζωn·v';
      let note = '';
      if (active.type === 1 && active.Kd === 0 && active.Ki > 0) {
        const a = 1 + active.K * active.Kp, denominator = active.tau * active.K * active.Ki - a;
        note = denominator > 0 ? 'PI stability guide: for this first-order plant, the unsaturated stability boundary is τs = ' + fmt(active.tau * a / denominator, 3) + ' s. Above it, small oscillations grow. Actuator limits can change their later behaviour.' : 'PI stability guide: these gains keep the unsaturated first-order model stable for every finite positive sensor time constant; a slow sensor can still cause ringing.';
      }
      $('stabilityNote').textContent = note;
      renderPredictionObservation();
    }
    
    const questions = [
      { from: .05, to: .3, question: 'With the demo gains fixed, if τs increases from 0.05 s to 0.30 s, the true overshoot will…',
        options: ['decrease', 'increase', 'stay exactly the same'], answer: 1,
        explanation: 'The sensor catches up later, so the controller keeps pushing while the true output is already approaching or passing the target.' },
      { from: .3, to: 0, question: 'If τs becomes zero, how will the measured and true output curves compare?',
        options: ['The measurement still lags.', 'They overlap: ym = y.', 'The measurement becomes zero.'], answer: 1,
        explanation: 'An ideal sensor reports the true output immediately. The measurement gap becomes zero, and the feedback loop reduces to the P1 model.' },
      { from: .05, to: 1, question: 'With the demo gains fixed and τs = 1 s, what happens to small oscillations before actuator limits matter?',
        options: ['They decay to zero.', 'They grow.', 'They disappear immediately.'], answer: 1,
        explanation: 'These gains cross the unsaturated PI stability boundary at τs = 2/3 s. A 1 s sensor is beyond it. The growing oscillations are caused by sensor lag; actuator limits eventually affect them.' }
    ];
    let questionIndex = 0, baseline = null;
    function showQuestion() {
      const q = questions[questionIndex]; baseline = null;
      $('pQn').textContent = 'Question ' + (questionIndex + 1) + ' of ' + questions.length;
      $('pSetup').textContent = 'Experiment: first-order K = 1, τ = 1 s; Kp = 1, Ki = 5, Kd = 0; r = 1; u between −10 and 10; run for 30 s.';
      $('pQ').textContent = q.question;
      $('pOpts').innerHTML = q.options.map((option, i) => `<label><input type="radio" name="pred" value="${i}"> ${option}</label>`).join('');
      $('pResult').textContent = ''; $('pGo').disabled = false; $('pNext').hidden = true; $('pLive').hidden = true;
    }
    function renderPredictionObservation() {
      if (!baseline || !last) return;
      $('pLive').hidden = false;
      $('pLive').innerHTML = '<div><b>Before the experiment:</b> τs = ' + fmt(baseline.lag) + ' s; true overshoot ' + fmt(baseline.metrics.overshoot, 1) + ' %; largest measurement gap ' + fmt(baseline.metrics.gap, 3) + '.</div>' +
        '<div><b>Current settings:</b> τs = ' + fmt(active.sensorTau) + ' s; true overshoot ' + fmt(last.metrics.overshoot, 1) + ' %; largest measurement gap ' + fmt(last.metrics.gap, 3) + '.</div>';
    }
    $('pGo').addEventListener('click', () => {
      const chosen = document.querySelector('input[name="pred"]:checked');
      if (!chosen) { $('pResult').textContent = 'Pick an answer first.'; return; }
      const q = questions[questionIndex];
      const before = SensorPID.simulate({ ...demo, type: 1, sensorTau: q.from, aw: true });
      baseline = { lag: q.from, metrics: before.metrics };
      $('pResult').textContent = (+chosen.value === q.answer ? 'Correct. ' : 'Not quite. ') + q.explanation + ' The experiment is now loaded above.';
      document.querySelectorAll('input[name="pred"]').forEach(input => input.disabled = true);
      $('pGo').disabled = true; $('pNext').hidden = false; loadDemo(q.to);
    });
    $('pNext').addEventListener('click', () => { questionIndex = (questionIndex + 1) % questions.length; showQuestion(); });
    
    function runChecks() {
      $('runChecks').disabled = true;
      $('validationStatus').textContent = 'Running reference cases…';
      setTimeout(() => {
        try {
          const rows = SensorPID.validationChecks(), passed = rows.filter(r => r.pass).length;
          $('vbody').innerHTML = rows.map(r => `<tr><th scope="row">${r.name}</th><td>${r.expected}</td><td>${r.value}</td><td><span class="pill ${r.pass ? 'pass' : 'fail'}">${r.pass ? 'Pass' : 'Fail'}</span></td></tr>`).join('');
          $('validationStatus').textContent = passed + ' of ' + rows.length + ' checks pass. These checks do not change your settings.';
        } catch (error) {
          $('validationStatus').textContent = 'Checks could not finish: ' + error.message;
        } finally {
          $('runChecks').disabled = false;
        }
      }, 0);
    }

    Object.keys(SL).forEach(id => $(id).addEventListener('input', () => requestDraw()));
    $('aw').addEventListener('change', () => requestDraw(true));
    $('b1').addEventListener('click', () => { plantType(1); requestDraw(true); });
    $('b2').addEventListener('click', () => { plantType(2); requestDraw(true); });
    document.querySelectorAll('[data-lag]').forEach(button => button.addEventListener('click', () => loadDemo(+button.dataset.lag)));
    $('reset').addEventListener('click', () => { questionIndex = 0; showQuestion(); loadDemo(.05); });
    $('runChecks').addEventListener('click', runChecks);
    $('c1').addEventListener('pointermove', event => {
      if (!last) return;
      const rect = $('c1').getBoundingClientRect(), fraction = Math.max(0, Math.min(1, (event.clientX - rect.left - 54) / (rect.width - 70)));
      const i = Math.round(fraction * (last.t.length - 1));
      $('cursorReadout').textContent = 't = ' + fmt(last.t[i], 3) + ' s · true y = ' + fmt(last.y[i], 3) + ' · measured ym = ' + fmt(last.ym[i], 3) + ' · setpoint r = ' + fmt(active.r, 3);
    });
    window.addEventListener('resize', renderPlots);
    matchMedia('(prefers-color-scheme: dark)').addEventListener('change', renderPlots);
    showQuestion(); requestDraw(true); runChecks();
  })();
}
