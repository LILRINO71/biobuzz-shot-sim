/* BIOBUZZ Shot Sim — UI. Needs window.SHOT_DATA, window.ShotEngine and (optionally) window.SHOT_WORKER_SRC / window.THREE. */
(function () {
  'use strict';

  var D = window.SHOT_DATA, E = window.ShotEngine;
  E.init(D);
  var F = D.field, SH = D.shooter, MODEL = SH.model;
  var $ = function (id) { return document.getElementById(id); };
  var DEG = Math.PI / 180;

  function nf(n, d) {
    if (n == null || !isFinite(n)) return '—';
    return Number(n).toLocaleString('en-US', { minimumFractionDigits: d || 0, maximumFractionDigits: d || 0 });
  }
  function pct(x, d) { return x == null || !isFinite(x) ? '—' : nf(x * 100, d || 0) + '%'; }
  function clamp(v, lo, hi) { return v < lo ? lo : v > hi ? hi : v; }

  // ------------------------------------------------------------------ state
  var def = SH.defaults || {};
  var wheelById = {};
  SH.wheels.forEach(function (w) { wheelById[w.id] = w; });
  var LIMIT = F.field.half - 9;
  var state = {
    ballId: 'pollen',
    target: 'red',
    hive: { red: F.hive.upSideStart.red, blue: F.hive.upSideStart.blue },
    robot: { x: F.hive.hiveX.red, y: -48 },
    h0: def.h0 || 16,
    precision: def.precision || 'typical',
    motorId: (D.motors.motors.find(function (m) { return m.freeRpm === 6000; }) || D.motors.motors[0]).id,
    shooter: {
      type: def.type || 'single', wheelId: wheelById[def.wheelId] ? def.wheelId : SH.wheels[0].id,
      motorsPerWheel: def.motorsPerWheel || 1, gear: def.gear || 1, topRatio: def.topRatio != null ? def.topRatio : 0.6,
      inertiaPreset: def.inertiaPreset || 'medium', shotInterval: def.shotInterval || 0.5,
      etaSingle: MODEL.etaSingle, etaDual: MODEL.etaDual
    },
    heat: false
  };

  function params(robot) {
    var pr = SH.precision[state.precision], s = state.shooter;
    var inert = SH.inertiaPresets.find(function (p) { return p.id === s.inertiaPreset; });
    return {
      ballId: state.ballId, target: state.target, hiveState: { red: state.hive.red, blue: state.hive.blue },
      robot: robot || { x: state.robot.x, y: state.robot.y }, h0: state.h0,
      precision: { sigThetaDeg: pr.sigThetaDeg, sigYawDeg: pr.sigYawDeg, sigShooter: pr.sigShooter },
      motorId: state.motorId,
      shooter: {
        type: s.type, wheelDiameterMm: wheelById[s.wheelId].diameterMm, motorsPerWheel: s.motorsPerWheel, gear: s.gear,
        topRatio: s.topRatio, inertiaKgM2: inert ? inert.inertiaKgM2 : 4e-4, shotInterval: s.shotInterval,
        etaSingle: s.etaSingle, etaDual: s.etaDual
      }
    };
  }

  // ------------------------------------------------------------------ palette
  var P = {};
  var PALETTE_KEYS = ['ground', 'panel', 'panel-2', 'ink', 'ink-2', 'ink-3', 'rule', 'tile', 'tile-seam', 'pollen', 'pollen-ink',
    'red', 'blue', 'good-mark', 'warn-mark', 'bad-mark', 'robot'];
  function readPalette() {
    var cs = getComputedStyle(document.documentElement);
    PALETTE_KEYS.forEach(function (k) { P[k] = cs.getPropertyValue('--' + k).trim() || '#888'; });
  }
  function rgba(hex, a) {
    var h = hex.replace('#', '');
    if (h.length === 3) h = h.split('').map(function (c) { return c + c; }).join('');
    var n = parseInt(h, 16);
    return 'rgba(' + ((n >> 16) & 255) + ',' + ((n >> 8) & 255) + ',' + (n & 255) + ',' + a + ')';
  }
  var VCLASS = { 'POSSIBLE': 'good', 'NOT CONSISTENT': 'warn', "WON'T WORK": 'bad' };
  var ICON = {
    good: '<svg viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="2.3" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M3 8.6l3.2 3.2L13 5"/></svg>',
    warn: '<svg viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="2.3" stroke-linecap="round" aria-hidden="true"><path d="M8 3v6.2M8 12.6v.4"/></svg>',
    bad: '<svg viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="2.3" stroke-linecap="round" aria-hidden="true"><path d="M4 4l8 8M12 4l-8 8"/></svg>',
    alert: '<svg viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M8 1.8 15 14H1Z"/><path d="M8 6.3v3.4M8 11.8v.2"/></svg>'
  };

  // ------------------------------------------------------------------ backend (worker or main thread)
  function mainBackend() {
    return {
      kind: 'main thread',
      evaluate: function (p, mode) { return new Promise(function (res) { setTimeout(function () { res(E.evaluate(p, mode)); }, 0); }); },
      motors: function (p) { return new Promise(function (res) { setTimeout(function () { res(E.compareMotors(p)); }, 0); }); },
      scan: function (p, onCell, onDone) {
        var pts = E.scanPoints(6), i = 0, stop = false;
        (function chunk() {
          if (stop) return;
          var t = performance.now();
          while (i < pts.length && performance.now() - t < 12) {
            var q = pts[i++];
            var r = E.evaluate(Object.assign({}, p, { robot: q }), 'coarse');
            onCell(q.x, q.y, r.verdict, i, pts.length);
          }
          if (i < pts.length) setTimeout(chunk, 0); else onDone();
        })();
        return function () { stop = true; };
      }
    };
  }

  function spawnWorker() {
    return new Promise(function (resolve, reject) {
      var url = URL.createObjectURL(new Blob([window.SHOT_WORKER_SRC], { type: 'text/javascript' }));
      var w = new Worker(url);
      var timer = setTimeout(function () { w.terminate(); reject(new Error('worker did not answer')); }, 2000);
      w.onmessage = function (e) { if (e.data && e.data.type === 'ready') { clearTimeout(timer); resolve(w); } };
      w.onerror = function (err) { clearTimeout(timer); w.terminate(); reject(err); };
      w.postMessage({ type: 'init', data: D });
    });
  }

  function makeBackend() {
    var noWorker = /[?&]noworker\b/.test(location.search);
    if (noWorker || !window.Worker || !window.SHOT_WORKER_SRC || !window.Blob || !window.URL) return Promise.resolve(mainBackend());
    var p;
    try { p = Promise.all([spawnWorker(), spawnWorker()]); } catch (e) { return Promise.resolve(mainBackend()); }
    return p.then(function (ws) {
      var live = ws[0], scanner = ws[1], pending = {}, seq = 0, scanId = 0, scanCbs = null;
      live.onmessage = function (e) {
        var m = e.data, cb = pending[m.id];
        if (!cb) return;
        delete pending[m.id];
        if (m.type === 'error') cb.reject(new Error(m.message)); else cb.resolve(m.type === 'motors' ? m.rows : m.result);
      };
      scanner.onmessage = function (e) {
        var m = e.data;
        if (!scanCbs || m.id !== scanId) return;
        if (m.type === 'scanCell') scanCbs.onCell(m.x, m.y, m.verdict, m.done, m.total);
        else if (m.type === 'scanDone' || m.type === 'error') scanCbs.onDone();
      };
      function call(msg) {
        return new Promise(function (resolve, reject) {
          var id = ++seq;
          pending[id] = { resolve: resolve, reject: reject };
          msg.id = id;
          live.postMessage(msg);
        });
      }
      return {
        kind: 'web worker',
        evaluate: function (pp, mode) { return call({ type: 'evaluate', params: pp, mode: mode }); },
        motors: function (pp) { return call({ type: 'motors', params: pp }); },
        scan: function (pp, onCell, onDone) {
          var my = ++scanId;
          scanCbs = { onCell: onCell, onDone: onDone };
          scanner.postMessage({ type: 'scan', id: my, params: pp, step: 6 });
          return function () { if (scanId === my) { scanCbs = null; scanner.postMessage({ type: 'cancelScan' }); } };
        }
      };
    }).catch(function () { return mainBackend(); });
  }

  var backend = null;

  // ------------------------------------------------------------------ jobs
  var current = null;          // last applied result
  var currentParams = null;
  var queued = null, running = false, jobSeq = 0, motorsSeq = 0, fullTimer = 0;

  function request(mode) {
    queued = { mode: mode, params: params(), seq: ++jobSeq };
    pump();
  }
  function pump() {
    if (running || !queued || !backend) return;
    var job = queued;
    queued = null;
    running = true;
    setBusy(true);
    backend.evaluate(job.params, job.mode).then(function (r) {
      apply(r, job.params, job.mode);
    }).catch(function (err) {
      $('reason').textContent = 'Something went wrong while calculating: ' + err.message;
    }).then(function () {
      running = false;
      if (queued) pump(); else setBusy(false);
    });
  }
  function requestFull(delay) {
    clearTimeout(fullTimer);
    fullTimer = setTimeout(function () {
      request('full');
      requestMotors();
      if (state.heat) startScan();
    }, delay == null ? 150 : delay);
  }
  function requestMotors() {
    if (!backend) return;
    var my = ++motorsSeq;
    backend.motors(params()).then(function (rows) { if (my === motorsSeq) renderMotors(rows); }).catch(function () {});
  }
  function setBusy(on) { $('verdictPanel').classList.toggle('busy', !!on); }

  function apply(r, p, mode) {
    current = r;
    currentParams = p;
    renderVerdict(r);
    renderArc(r, p);
    renderMotor(r, p);
    drawField();
    if (mode === 'full') { drawSide(); three.update(); }
  }

  // ------------------------------------------------------------------ verdict + readouts
  function renderVerdict(r) {
    var cls = VCLASS[r.verdict] || 'bad';
    var badge = $('verdictBadge');
    badge.className = 'verdict-badge v-' + cls;
    badge.innerHTML = ICON[cls];
    badge.appendChild(document.createTextNode(r.verdict));
    var rate = Math.floor(r.hitRate * 100 + 1e-9);
    $('hitRate').textContent = rate + '%';
    var fill = $('rateFill');
    fill.style.width = clamp(r.hitRate * 100, 0, 100) + '%';
    fill.style.background = P[cls + '-mark'];
    $('reason').textContent = r.reason;
    var box = $('warnings');
    box.replaceChildren();
    (r.warnings || []).forEach(function (w) {
      var c = document.createElement('span');
      c.className = 'chip';
      c.innerHTML = ICON.alert;
      c.appendChild(document.createTextNode(w.text));
      box.appendChild(c);
    });
  }

  function fillKV(dl, items) {
    dl.replaceChildren();
    items.forEach(function (it) {
      var d = document.createElement('div');
      if (it.wide) d.className = 'wide';
      var dt = document.createElement('dt');
      dt.textContent = it.label;
      var dd = document.createElement('dd');
      dd.textContent = it.value;
      if (it.small) {
        var sm = document.createElement('small');
        sm.textContent = it.small;
        dd.appendChild(sm);
      }
      d.appendChild(dt);
      d.appendChild(dd);
      dl.appendChild(d);
    });
  }

  function aimWords(off) {
    if (Math.abs(off) < 0.05) return 'straight at the mouth center';
    return nf(Math.abs(off), 1) + '° ' + (off > 0 ? 'left' : 'right') + ' of the mouth center';
  }

  function renderArc(r) {
    var b = r.best, dl = $('arcReadout');
    if (!b) {
      var cm = r.closestMiss;
      fillKV(dl, [
        { label: 'Launch angle', value: '—' }, { label: 'Exit speed', value: '—' },
        { label: 'Closest try', value: cm ? nf(cm.thetaDeg, 0) + '° · ' + nf(cm.v, 2) + ' m/s' : '—', small: cm && cm.cause ? 'misses: ' + causeWord(cm.cause) : '', wide: true }
      ]);
      return;
    }
    var w = b.windows, note = b.reachable === false ? ' (needed; motor can\'t reach it)' : '';
    fillKV(dl, [
      { label: 'Launch angle', value: nf(b.thetaDeg, 1) + '°', small: nf(90 - b.thetaDeg, 1) + '° off vertical' },
      { label: 'Exit speed' + note, value: nf(b.v, 2) + ' m/s', small: nf(b.vFtS, 1) + ' ft/s' },
      { label: 'Aim', value: nf(b.yawOffsetDeg, 1) + '°', small: aimWords(b.yawOffsetDeg) },
      { label: 'Apex', value: nf(b.apexIn, 0) + ' in', small: nf(b.apexIn / 12, 1) + ' ft above the tiles' },
      { label: 'Time to CELL', value: b.tToCell != null ? nf(b.tToCell, 2) + ' s' : '—', small: b.entrySpeed != null ? 'enters at ' + nf(b.entrySpeed, 1) + ' m/s' : '' },
      { label: 'Distance', value: nf(r.distIn, 0) + ' in', small: 'robot to mouth center, flat' },
      { label: 'Scoring window', wide: true,
        value: nf(w.thetaDeg[0], 1) + '–' + nf(w.thetaDeg[1], 1) + '° · ' + nf(w.v[0], 2) + '–' + nf(w.v[1], 2) + ' m/s',
        small: 'aim ' + nf(w.yawDeg[0] - b.yawDeg, 1) + '° to +' + nf(w.yawDeg[1] - b.yawDeg, 1) + '° · each range holds the other two at the best value' }
    ]);
  }

  function causeWord(c) {
    return ({ short: 'falls short', long: 'flies past', lip: 'clips the bottom lip', roof: 'hits the CELL rim or roof', cell: 'hits another CELL',
      frame: 'hits the HIVE frame or arm', wall: 'hits the wall', floor: 'hits the floor' })[c] || c;
  }

  function renderMotor(r, p) {
    var m = r.motor, dl = $('motorReadout');
    var mo = D.motors.motors.find(function (x) { return x.id === p.motorId; });
    var fill = $('meterFill');
    if (!m) {
      fillKV(dl, [{ label: 'Motor', value: mo ? mo.label : '—', wide: true, small: 'No arc from here, so there is no speed to check' }]);
      fill.style.width = '0';
      return;
    }
    var bp = E.ballProps(p.ballId);
    var spinRpm = m.S0 * (r.best ? r.best.v : 0) / (bp.rIn * 0.0254) * 60 / (2 * Math.PI);
    var sg = r.sigma || {};
    fillKV(dl, [
      { label: 'Wheel speed', value: nf(m.wheelRpm, 0) + ' rpm', small: nf(p.shooter.wheelDiameterMm, 0) + ' mm wheel' },
      { label: 'Motor speed', value: nf(m.motorRpm, 0) + ' rpm', small: 'of ' + nf(m.usableFreeRpm, 0) + ' usable (' + pct(m.headroom) + ')' },
      { label: 'Dip per shot', value: pct(m.dip, 1), small: isFinite(m.recoveryMs) ? 'back to speed in ' + nf(m.recoveryMs, 0) + ' ms' : 'never recovers' },
      { label: 'Spin-up', value: isFinite(m.spinUpMs) ? nf(m.spinUpMs / 1000, 2) + ' s' : '—', small: 'from a standstill' },
      { label: 'Speed scatter', value: sg.v != null ? '±' + pct(sg.v, 1) : '—',
        small: sg.v != null ? 'shots ' + pct(sg.shooter, 1) + ' · motor ' + pct(sg.motor, 1) + ' · recovery ' + pct(sg.recovery, 1) : '' },
      { label: 'Backspin', value: nf(spinRpm, 0) + ' rpm', small: 'spin ratio ' + nf(m.S0, 2) }
    ]);
    var h = m.headroom;
    fill.style.width = clamp(h * 100, 0, 100) + '%';
    fill.className = 'meter-fill' + (h > 1 ? ' bad' : h > 0.8 ? ' warn' : '');
  }

  function renderMotors(rows) {
    var body = $('motorRows'), fsf = MODEL.freeSpeedFactor || 1, g = state.shooter.gear;
    body.replaceChildren();
    rows.forEach(function (row) {
      var tr = document.createElement('tr');
      tr.tabIndex = 0;
      if (row.motorId === state.motorId) { tr.className = 'sel'; tr.setAttribute('aria-selected', 'true'); }
      var mo = D.motors.motors.find(function (x) { return x.id === row.motorId; });
      var td0 = document.createElement('td');
      td0.textContent = row.label;
      var sku = document.createElement('small');
      sku.textContent = mo ? mo.sku : '';
      td0.appendChild(sku);
      tr.appendChild(td0);
      var need = row.motorRpm != null ? row.motorRpm / (row.freeRpm * fsf * 0.8) * g : null;
      var cells = [
        nf(row.freeRpm, 0) + ' rpm',
        row.motorRpm != null ? nf(row.motorRpm, 0) + ' rpm' : '—',
        row.headroom != null ? pct(row.headroom) : '—',
        need == null ? '—' : need <= g + 1e-9 ? 'not needed' : nf(need, 1) + ' : 1',
        row.recoveryMs != null && isFinite(row.recoveryMs) ? nf(row.recoveryMs, 0) + ' ms' : '—',
        pct(row.hitRate)
      ];
      cells.forEach(function (c) { var td = document.createElement('td'); td.textContent = c; tr.appendChild(td); });
      var tdv = document.createElement('td');
      var cls = VCLASS[row.verdict] || 'bad';
      var chip = document.createElement('span');
      chip.className = 'vchip v-' + cls;
      chip.innerHTML = ICON[cls];
      chip.appendChild(document.createTextNode(row.verdict));
      chip.title = row.reason;
      tdv.appendChild(chip);
      tr.appendChild(tdv);
      function pick() { state.motorId = row.motorId; syncControls(); requestFull(0); }
      tr.addEventListener('click', pick);
      tr.addEventListener('keydown', function (e) { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); pick(); } });
      body.appendChild(tr);
    });
    $('motorsCaption').textContent = 'Same wheel (' + nf(wheelById[state.shooter.wheelId].diameterMm, 0) + ' mm), gearing (' + g + ' : 1) and shooter. "Gear-up for 80%" is the wheel : motor ratio that would run the motor at 80% of its free speed. Click a row to use that motor.';
  }

  // ------------------------------------------------------------------ field view
  var fieldCanvas = $('field'), fctx = fieldCanvas.getContext('2d');
  var FV = { size: 600, ext: F.field.half + 9, dpr: 1 };
  var hiveCache = { key: '', model: null };
  function hive() {
    var key = state.hive.red + ',' + state.hive.blue;
    if (hiveCache.key !== key) { hiveCache.key = key; hiveCache.model = E.hiveModel(state.hive); }
    return hiveCache.model;
  }
  function layoutField() {
    var wrap = $('fieldWrap');
    var avail = wrap.clientWidth;
    var maxH = Math.max(340, window.innerHeight - 140);
    var size = Math.max(260, Math.floor(Math.min(avail, maxH)));
    var dpr = Math.min(2.5, window.devicePixelRatio || 1);
    FV.size = size; FV.dpr = dpr;
    fieldCanvas.style.width = size + 'px';
    fieldCanvas.style.height = size + 'px';
    fieldCanvas.width = Math.round(size * dpr);
    fieldCanvas.height = Math.round(size * dpr);
  }
  function sx(x) { return (x + FV.ext) / (2 * FV.ext) * FV.size; }
  function sy(y) { return (FV.ext - y) / (2 * FV.ext) * FV.size; }
  function k() { return FV.size / (2 * FV.ext); }
  function toWorld(px, py) { return { x: px / FV.size * 2 * FV.ext - FV.ext, y: FV.ext - py / FV.size * 2 * FV.ext }; }

  function hull2(pts) {
    pts = pts.slice().sort(function (a, b) { return a[0] - b[0] || a[1] - b[1]; });
    if (pts.length < 3) return pts;
    function cross(o, a, b) { return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]); }
    var lo = [], up = [];
    pts.forEach(function (p) { while (lo.length >= 2 && cross(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); });
    for (var i = pts.length - 1; i >= 0; i--) { var p = pts[i]; while (up.length >= 2 && cross(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); }
    up.pop(); lo.pop();
    return lo.concat(up);
  }
  function poly(ctx, pts) {
    ctx.beginPath();
    pts.forEach(function (p, i) { if (i) ctx.lineTo(p[0], p[1]); else ctx.moveTo(p[0], p[1]); });
    ctx.closePath();
  }
  function line(ctx, a, b) { ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke(); }

  var heatCells = {}, heatKey = '', heatStop = null, heatCount = 0;

  function drawField() {
    var c = fctx, s = FV.size, K = k(), half = F.field.half;
    c.setTransform(FV.dpr, 0, 0, FV.dpr, 0, 0);
    c.clearRect(0, 0, s, s);
    c.fillStyle = P.ground;
    c.fillRect(0, 0, s, s);

    // alliance areas (outside the walls)
    ['red', 'blue'].forEach(function (al) {
      var a = F.allianceAreas[al];
      var x0 = Math.max(a.x0, -FV.ext), x1 = Math.min(a.x1, FV.ext);
      c.fillStyle = rgba(P[al], 0.16);
      c.fillRect(sx(x0), sy(a.y1), (x1 - x0) * K, (a.y1 - a.y0) * K);
      c.save();
      c.translate(sx(al === 'red' ? -half - 4.5 : half + 4.5), sy(0));
      c.rotate(al === 'red' ? -Math.PI / 2 : Math.PI / 2);
      c.fillStyle = P['ink-2'];
      c.font = '600 ' + Math.max(10, Math.round(3.4 * K)) + 'px "Barlow Semi Condensed", sans-serif';
      c.textAlign = 'center';
      c.textBaseline = 'middle';
      c.fillText(al.toUpperCase() + ' ALLIANCE', 0, 0);
      c.restore();
    });
    c.fillStyle = P['ink-3'];
    c.font = '600 ' + Math.max(10, Math.round(3.2 * K)) + 'px "Barlow Semi Condensed", sans-serif';
    c.textAlign = 'center';
    c.textBaseline = 'middle';
    c.fillText('AUDIENCE', sx(0), sy(-half - 4.5));

    // tiles
    c.fillStyle = P.tile;
    c.fillRect(sx(-half), sy(half), 2 * half * K, 2 * half * K);
    c.strokeStyle = P['tile-seam'];
    c.lineWidth = 1;
    (F.field.tileSeams || []).forEach(function (v) {
      line(c, [sx(v), sy(half)], [sx(v), sy(-half)]);
      line(c, [sx(-half), sy(v)], [sx(half), sy(v)]);
    });

    // heat map
    if (state.heat) {
      var step = 6;
      Object.keys(heatCells).forEach(function (key) {
        var cell = heatCells[key], cls = VCLASS[cell.v];
        c.fillStyle = rgba(P[cls + '-mark'], 0.5);
        c.fillRect(sx(cell.x - step / 2) + 0.5, sy(cell.y + step / 2) + 0.5, step * K - 1, step * K - 1);
      });
    }

    // zones
    ['red', 'blue'].forEach(function (al) {
      var z = F.loadingZones[al];
      c.strokeStyle = P[al];
      c.lineWidth = Math.max(2, 1.2 * K);
      c.strokeRect(sx(z.x0) + c.lineWidth / 2, sy(z.y1) + c.lineWidth / 2, (z.x1 - z.x0) * K - c.lineWidth, (z.y1 - z.y0) * K - c.lineWidth);
      var g = F.gardens[al];
      c.fillStyle = P[al];
      c.fillRect(sx(g.x0), sy(g.y1), (g.x1 - g.x0) * K, Math.max(2, (g.y1 - g.y0) * K));
    });

    // walls
    c.strokeStyle = P['ink-2'];
    c.lineWidth = Math.max(2, (F.field.wallThickness || 1) * K);
    c.strokeRect(sx(-half) - c.lineWidth / 2, sy(half) - c.lineWidth / 2, 2 * half * K + c.lineWidth, 2 * half * K + c.lineWidth);

    // flowers
    F.flowers.forEach(function (fl) {
      var alongX = fl.wall === '+y' || fl.wall === '-y';
      var wx = (alongX ? fl.alongWall : fl.depth) * K, wy = (alongX ? fl.depth : fl.alongWall) * K;
      c.fillStyle = P['panel-2'];
      c.strokeStyle = P['ink-3'];
      c.lineWidth = 1.2;
      c.fillRect(sx(fl.x) - wx / 2, sy(fl.y) - wy / 2, wx, wy);
      c.strokeRect(sx(fl.x) - wx / 2, sy(fl.y) - wy / 2, wx, wy);
      c.beginPath();
      c.arc(sx(fl.x), sy(fl.y), Math.max(2, fl.openingDia / 2 * K), 0, 2 * Math.PI);
      c.stroke();
    });

    // HIVE frame
    var hm = hive(), fr = hm.frame;
    c.strokeStyle = P['ink-3'];
    c.lineCap = 'round';
    c.lineWidth = Math.max(1.5, 2 * fr.tubeRadius * K);
    if (fr.footBars) fr.footBars.segments.forEach(function (sg) { line(c, [sx(sg[0][0]), sy(sg[0][1])], [sx(sg[1][0]), sy(sg[1][1])]); });
    fr.legs.forEach(function (sg) { line(c, [sx(sg[0][0]), sy(sg[0][1])], [sx(sg[1][0]), sy(sg[1][1])]); });
    line(c, [sx(fr.crossbar[0][0]), sy(fr.crossbar[0][1])], [sx(fr.crossbar[1][0]), sy(fr.crossbar[1][1])]);

    // CELLs (down first, then up, target last)
    var order = [];
    hm.hives.forEach(function (h) { h.cells.forEach(function (cell) { order.push({ h: h, cell: cell }); }); });
    order.sort(function (a, b) {
      var ra = (a.cell.role === 'up' ? 1 : 0) + (a.h.alliance === state.target && a.cell.role === 'up' ? 1 : 0);
      var rb = (b.cell.role === 'up' ? 1 : 0) + (b.h.alliance === state.target && b.cell.role === 'up' ? 1 : 0);
      return ra - rb;
    });
    order.forEach(function (o) {
      var isT = o.h.alliance === state.target && o.cell.role === 'up';
      var pts = hull2(o.cell.vertices.map(function (v) { return [sx(v[0]), sy(v[1])]; }));
      poly(c, pts);
      c.fillStyle = rgba(P[o.h.alliance], o.cell.role === 'up' ? (isT ? 0.34 : 0.22) : 0.10);
      c.fill();
      c.strokeStyle = isT ? P.ink : rgba(P[o.h.alliance], 0.9);
      c.lineWidth = isT ? 2 : 1.2;
      c.stroke();
    });
    hm.hives.forEach(function (h) {
      c.strokeStyle = P['ink-3'];
      c.lineWidth = Math.max(1.5, 2 * h.armRadius * K);
      for (var i = 0; i + 1 < h.armBar.length; i++) line(c, [sx(h.armBar[i][0]), sy(h.armBar[i][1])], [sx(h.armBar[i + 1][0]), sy(h.armBar[i + 1][1])]);
    });
    // target mouth lip
    var th = hm.hives.find(function (h) { return h.alliance === state.target; });
    var mouth = th.cells[0].mouth;
    c.strokeStyle = P.pollen;
    c.lineWidth = Math.max(3, 1.3 * K);
    line(c, [sx(mouth[0][0]), sy(mouth[0][1])], [sx(mouth[1][0]), sy(mouth[1][1])]);
    var mc = hm.mouthCentroid[state.target];
    c.fillStyle = P['ink-2'];
    c.font = '700 ' + Math.max(10, Math.round(3.4 * K)) + 'px "Barlow Semi Condensed", sans-serif';
    c.textAlign = 'center';
    c.textBaseline = 'middle';
    var lx = th.hx + (th.alliance === 'red' ? -1 : 1) * 17;
    c.fillText('UP', sx(lx), sy(mc[1]));

    // ground track + aim
    var r = current, heading = r ? (r.best ? r.best.yawDeg : r.psi0Deg) : 90;
    if (r && r.trajectory && r.best && r.best.reachable !== false) {
      c.strokeStyle = P.pollen;
      c.lineWidth = 3;
      c.lineJoin = 'round';
      c.beginPath();
      r.trajectory.forEach(function (q, i) { if (i) c.lineTo(sx(q[0]), sy(q[1])); else c.moveTo(sx(q[0]), sy(q[1])); });
      c.stroke();
    } else {
      c.strokeStyle = rgba(P['ink-2'], 0.6);
      c.lineWidth = 1.5;
      line(c, [sx(state.robot.x), sy(state.robot.y)], [sx(mc[0]), sy(mc[1])]);
    }
    c.beginPath();
    c.arc(sx(mc[0]), sy(mc[1]), Math.max(3, 1.2 * K), 0, 2 * Math.PI);
    c.fillStyle = P.ink;
    c.fill();

    // robot
    var rx = sx(state.robot.x), ry = sy(state.robot.y), hs = 9 * K;
    c.save();
    c.translate(rx, ry);
    c.rotate(-heading * DEG);
    c.fillStyle = P.robot;
    c.strokeStyle = P.ink;
    c.lineWidth = 2;
    c.beginPath();
    if (c.roundRect) c.roundRect(-hs, -hs, 2 * hs, 2 * hs, 2 * K); else c.rect(-hs, -hs, 2 * hs, 2 * hs);
    c.fill();
    c.stroke();
    c.strokeStyle = P.pollen;
    c.lineWidth = Math.max(3, 1.4 * K);
    line(c, [hs - 1.2 * K, -hs * 0.72], [hs - 1.2 * K, hs * 0.72]);
    c.fillStyle = P.ink;
    c.beginPath();
    c.arc(0, 0, Math.max(3, 1.3 * K), 0, 2 * Math.PI);
    c.fill();
    c.restore();
    if (document.activeElement === fieldCanvas) {
      c.strokeStyle = P.pollen;
      c.lineWidth = 1.5;
      c.beginPath();
      c.arc(rx, ry, hs * 1.6, 0, 2 * Math.PI);
      c.stroke();
    }
  }

  // pointer + keyboard
  var dragging = false, dragFrame = 0;
  function moveRobotTo(x, y, mode) {
    state.robot.x = Math.round(clamp(x, -LIMIT, LIMIT) * 2) / 2;
    state.robot.y = Math.round(clamp(y, -LIMIT, LIMIT) * 2) / 2;
    $('rx').value = state.robot.x;
    $('ry').value = state.robot.y;
    drawField();
    if (mode === 'coarse') {
      if (!dragFrame) dragFrame = requestAnimationFrame(function () { dragFrame = 0; request('coarse'); });
    } else {
      request('coarse');
      requestFull(250);
    }
  }
  function eventWorld(e) {
    var rect = fieldCanvas.getBoundingClientRect();
    return toWorld(e.clientX - rect.left, e.clientY - rect.top);
  }
  fieldCanvas.addEventListener('pointerdown', function (e) {
    if (e.button !== undefined && e.button !== 0) return;
    dragging = true;
    fieldCanvas.setPointerCapture(e.pointerId);
    fieldCanvas.classList.add('dragging');
    fieldCanvas.focus({ preventScroll: true });
    var w = eventWorld(e);
    moveRobotTo(w.x, w.y, 'coarse');
    e.preventDefault();
  });
  fieldCanvas.addEventListener('pointermove', function (e) {
    if (!dragging) return;
    var w = eventWorld(e);
    moveRobotTo(w.x, w.y, 'coarse');
  });
  function endDrag(e) {
    if (!dragging) return;
    dragging = false;
    fieldCanvas.classList.remove('dragging');
    try { fieldCanvas.releasePointerCapture(e.pointerId); } catch (err) { /* already released */ }
    requestFull(0);
  }
  fieldCanvas.addEventListener('pointerup', endDrag);
  fieldCanvas.addEventListener('pointercancel', endDrag);
  fieldCanvas.addEventListener('keydown', function (e) {
    var d = e.shiftKey ? 6 : 1, dx = 0, dy = 0;
    if (e.key === 'ArrowLeft') dx = -d; else if (e.key === 'ArrowRight') dx = d;
    else if (e.key === 'ArrowUp') dy = d; else if (e.key === 'ArrowDown') dy = -d;
    else return;
    e.preventDefault();
    moveRobotTo(state.robot.x + dx, state.robot.y + dy, 'key');
  });
  fieldCanvas.addEventListener('focus', drawField);
  fieldCanvas.addEventListener('blur', drawField);

  // heat map
  function heatParamsKey(p) { var q = Object.assign({}, p); delete q.robot; return JSON.stringify(q); }
  function startScan() {
    if (!backend) return;
    var p = params(), key = heatParamsKey(p);
    if (key === heatKey && heatStop === null && heatCount > 0) return; // finished for these settings
    if (heatStop) heatStop();
    heatKey = key;
    heatCells = {};
    heatCount = 0;
    var prog = $('scanProgress');
    prog.hidden = false;
    var drawQueued = false;
    heatStop = backend.scan(p, function (x, y, verdict, done, total) {
      heatCells[x + ',' + y] = { x: x, y: y, v: verdict };
      heatCount = done;
      prog.textContent = 'Map ' + Math.round(done / total * 100) + '%';
      if (!drawQueued) { drawQueued = true; requestAnimationFrame(function () { drawQueued = false; drawField(); }); }
    }, function () {
      heatStop = null;
      prog.textContent = 'Map ready';
      drawField();
    });
  }
  $('scanBtn').addEventListener('click', function () {
    state.heat = !state.heat;
    this.setAttribute('aria-pressed', String(state.heat));
    $('heatLegend').hidden = !state.heat;
    if (state.heat) startScan();
    else {
      if (heatStop) { heatStop(); heatStop = null; }
      heatKey = ''; heatCells = {}; heatCount = 0;
      $('scanProgress').hidden = true;
      drawField();
    }
  });

  // ------------------------------------------------------------------ side view
  var sideCanvas = $('side'), sctx = sideCanvas.getContext('2d');
  function drawSide() {
    var r = current, p = currentParams;
    if (!r || !p) return;
    var cssW = sideCanvas.clientWidth || 500, cssH = Math.round(Math.max(250, Math.min(420, cssW * 0.72)));
    var dpr = Math.min(2.5, window.devicePixelRatio || 1);
    sideCanvas.style.height = cssH + 'px';
    sideCanvas.width = Math.round(cssW * dpr);
    sideCanvas.height = Math.round(cssH * dpr);
    var c = sctx;
    c.setTransform(dpr, 0, 0, dpr, 0, 0);
    c.fillStyle = P['panel-2'];
    c.fillRect(0, 0, cssW, cssH);

    var sv = E.sideView(p, r);
    var arc = sv.arcs.trajectory || (r.closestMiss ? r.closestMiss.path.map(function (q) {
      return [(q[0] - sv.origin[0]) * sv.uAxis[0] + (q[1] - sv.origin[1]) * sv.uAxis[1], q[2]];
    }) : null);
    var s0 = -14, s1 = 14, zTop = 80;
    function grow(q) { if (q[0] < s0) s0 = q[0]; if (q[0] > s1) s1 = q[0]; if (q[1] + 8 > zTop) zTop = q[1] + 8; }
    sv.polys.forEach(function (pl) { if (pl.role === 'target' || pl.role === 'cell') pl.pts.forEach(grow); });
    if (arc) arc.forEach(grow);
    s0 -= 6; s1 += 6;
    var m = { l: 34, r: 10, t: 10, b: 26 };
    var kk = Math.min((cssW - m.l - m.r) / (s1 - s0), (cssH - m.t - m.b) / zTop);
    var ox = m.l + ((cssW - m.l - m.r) - (s1 - s0) * kk) / 2;
    function X(s) { return ox + (s - s0) * kk; }
    function Y(z) { return cssH - m.b - z * kk; }

    c.strokeStyle = P.rule;
    c.lineWidth = 1;
    c.font = '500 11px "JetBrains Mono", monospace';
    c.fillStyle = P['ink-3'];
    c.textAlign = 'right';
    c.textBaseline = 'middle';
    for (var z = 0; z <= zTop; z += 12) { line(c, [X(s0), Y(z)], [X(s1), Y(z)]); c.fillText(String(z), m.l - 6, Y(z)); }
    c.textAlign = 'center';
    c.textBaseline = 'top';
    var stepS = (s1 - s0) > 160 ? 48 : 24;
    for (var sVal = Math.ceil(s0 / stepS) * stepS; sVal <= s1; sVal += stepS) c.fillText(String(sVal), X(sVal), Y(0) + 6);
    c.strokeStyle = P['ink-2'];
    c.lineWidth = 1.5;
    line(c, [X(s0), Y(0)], [X(s1), Y(0)]);

    sv.polys.forEach(function (pl) {
      var pts = pl.pts.map(function (q) { return [X(q[0]), Y(q[1])]; });
      poly(c, pts);
      if (pl.role === 'target') { c.fillStyle = rgba(P.pollen, 0.16); c.strokeStyle = P.ink; c.lineWidth = 1.5; }
      else if (pl.role === 'cell') { c.fillStyle = rgba(P[pl.alliance] || P['ink-3'], 0.12); c.strokeStyle = rgba(P['ink-3'], 0.8); c.lineWidth = 1; }
      else { c.fillStyle = rgba(P['ink-3'], 0.35); c.strokeStyle = 'transparent'; c.lineWidth = 0; }
      c.fill();
      if (c.lineWidth) c.stroke();
    });
    if (sv.mouth) {
      var mh = hull2(sv.mouth.map(function (q) { return [X(q[0]), Y(q[1])]; }));
      c.strokeStyle = P.pollen;
      c.lineWidth = 3;
      poly(c, mh);
      c.stroke();
    }

    // robot + mast
    var h0 = p.h0;
    c.fillStyle = P.robot;
    c.strokeStyle = P['ink-2'];
    c.lineWidth = 1.5;
    c.fillRect(X(-9), Y(Math.min(h0, 12)), 18 * kk, Math.min(h0, 12) * kk);
    c.strokeRect(X(-9), Y(Math.min(h0, 12)), 18 * kk, Math.min(h0, 12) * kk);
    if (h0 > 12) { c.lineWidth = 3; line(c, [X(0), Y(12)], [X(0), Y(h0)]); }
    c.fillStyle = P.ink;
    c.beginPath();
    c.arc(X(0), Y(h0), 3.5, 0, 2 * Math.PI);
    c.fill();

    (sv.arcs.ghosts || []).forEach(function (g) {
      c.strokeStyle = P['ink-3'];
      c.lineWidth = 1.5;
      c.beginPath();
      g.forEach(function (q, i) { if (i) c.lineTo(X(q[0]), Y(q[1])); else c.moveTo(X(q[0]), Y(q[1])); });
      c.stroke();
    });
    if (arc) {
      var ok = !!sv.arcs.trajectory && r.best && r.best.reachable !== false;
      c.strokeStyle = ok ? P.pollen : P['bad-mark'];
      c.lineWidth = ok ? 3 : 2;
      c.lineJoin = 'round';
      c.beginPath();
      arc.forEach(function (q, i) { if (i) c.lineTo(X(q[0]), Y(q[1])); else c.moveTo(X(q[0]), Y(q[1])); });
      c.stroke();
      var end = arc[arc.length - 1], rIn = E.ballProps(p.ballId).rIn;
      c.beginPath();
      c.arc(X(end[0]), Y(end[1]), Math.max(3, rIn * kk), 0, 2 * Math.PI);
      c.fillStyle = ok ? P.pollen : P['bad-mark'];
      c.fill();
      var apex = arc.reduce(function (a, q) { return q[1] > a[1] ? q : a; }, arc[0]);
      c.fillStyle = P.ink;
      c.font = '600 12px "Barlow Semi Condensed", sans-serif';
      c.textAlign = 'center';
      c.textBaseline = 'bottom';
      c.fillText((ok ? 'apex ' : 'closest try · apex ') + Math.round(apex[1]) + ' in', X(apex[0]), Y(apex[1]) - 6);
    }
    c.fillStyle = P['ink-2'];
    c.font = '500 11px "JetBrains Mono", monospace';
    c.textAlign = 'left';
    c.textBaseline = 'top';
    c.fillText('in · along the shot', m.l, 4);
    $('viewsCaption').textContent = r.best ? 'Launch ' + nf(r.best.thetaDeg, 1) + '° at ' + nf(r.best.v, 2) + ' m/s from ' + nf(p.h0, 0) + ' in up. Side view and 3-D view are to scale.' : 'No clean arc from this spot — the red line is the closest try.';
  }

  // ------------------------------------------------------------------ 3-D view
  var three = (function () {
    var host = $('threeHost'), T = window.THREE, api = { update: function () {}, rebuild: function () {}, resize: function () {} };
    if (!T) {
      host.classList.add('no3d');
      host.textContent = '3-D view unavailable (three.js did not load). Everything else works.';
      return api;
    }
    var renderer;
    try { renderer = new T.WebGLRenderer({ antialias: true }); } catch (e) {
      host.classList.add('no3d');
      host.textContent = '3-D view unavailable (WebGL is off). Everything else works.';
      return api;
    }
    renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
    host.appendChild(renderer.domElement);
    var scene = new T.Scene(), camera = new T.PerspectiveCamera(38, 4 / 3, 1, 3000);
    var hemi = new T.HemisphereLight(0xffffff, 0x777777, 0.62), sun = new T.DirectionalLight(0xffffff, 0.5);
    sun.position.set(-120, 220, 160);
    scene.add(hemi, sun);
    var staticGroup = new T.Group(), dynGroup = new T.Group();
    scene.add(staticGroup, dynGroup);
    var orbit = { az: 2.2, el: 0.5, dist: 235, target: new T.Vector3(0, 22, 0) };
    var needs = true, userOrbited = false;

    function V(p) { return new T.Vector3(p[0], p[2], -p[1]); }
    function col(k) { return new T.Color(P[k]); }
    function clear(g) {
      while (g.children.length) {
        var o = g.children.pop();
        o.traverse(function (n) { if (n.geometry) n.geometry.dispose(); if (n.material) n.material.dispose(); });
      }
    }
    function rod(a, b, r, color, g) {
      var va = V(a), vb = V(b), len = va.distanceTo(vb);
      var mesh = new T.Mesh(new T.CylinderGeometry(r, r, len, 10), new T.MeshStandardMaterial({ color: color, roughness: 0.7 }));
      mesh.position.copy(va).add(vb).multiplyScalar(0.5);
      mesh.quaternion.setFromUnitVectors(new T.Vector3(0, 1, 0), vb.clone().sub(va).normalize());
      g.add(mesh);
    }
    function cellMesh(cell, color, opacity, edgeColor, g) {
      var v = cell.vertices, pos = [], idx = [];
      v.forEach(function (p) { var q = V(p); pos.push(q.x, q.y, q.z); });
      for (var i = 0; i < 5; i++) { var j = (i + 1) % 5; idx.push(i, j, 5 + j, i, 5 + j, 5 + i); }
      idx.push(0, 1, 2, 0, 2, 3, 0, 3, 4);
      var geo = new T.BufferGeometry();
      geo.setAttribute('position', new T.Float32BufferAttribute(pos, 3));
      geo.setIndex(idx);
      geo.computeVertexNormals();
      g.add(new T.Mesh(geo, new T.MeshStandardMaterial({ color: color, transparent: true, opacity: opacity, side: T.DoubleSide, depthWrite: false })));
      var lines = [];
      function seg(a, b) { var qa = V(v[a]), qb = V(v[b]); lines.push(qa.x, qa.y, qa.z, qb.x, qb.y, qb.z); }
      for (var k2 = 0; k2 < 5; k2++) { seg(k2, (k2 + 1) % 5); seg(5 + k2, 5 + (k2 + 1) % 5); seg(k2, 5 + k2); }
      var lg = new T.BufferGeometry();
      lg.setAttribute('position', new T.Float32BufferAttribute(lines, 3));
      g.add(new T.LineSegments(lg, new T.LineBasicMaterial({ color: edgeColor })));
    }
    function tube(path, radius, color, g) {
      if (!path || path.length < 2) return;
      var pts = [], stride = Math.max(1, Math.floor(path.length / 120));
      for (var i = 0; i < path.length; i += stride) pts.push(V(path[i]));
      pts.push(V(path[path.length - 1]));
      var curve = new T.CatmullRomCurve3(pts);
      g.add(new T.Mesh(new T.TubeGeometry(curve, Math.min(400, pts.length * 3), radius, 8, false), new T.MeshStandardMaterial({ color: color, roughness: 0.4 })));
    }

    function rebuild() {
      clear(staticGroup);
      scene.background = col('panel-2');
      var half = F.field.half;
      var floor = new T.Mesh(new T.PlaneGeometry(2 * half, 2 * half), new T.MeshStandardMaterial({ color: col('tile'), roughness: 1 }));
      floor.rotation.x = -Math.PI / 2;
      staticGroup.add(floor);
      var seams = [];
      (F.field.tileSeams || []).forEach(function (s) { seams.push(s, 0.05, -half, s, 0.05, half, -half, 0.05, s, half, 0.05, s); });
      var sg = new T.BufferGeometry();
      sg.setAttribute('position', new T.Float32BufferAttribute(seams, 3));
      staticGroup.add(new T.LineSegments(sg, new T.LineBasicMaterial({ color: col('tile-seam') })));
      var wallMat = new T.MeshStandardMaterial({ color: col('ink-3'), transparent: true, opacity: 0.55 });
      var wh = F.field.wallHeight, wt = F.field.wallThickness || 1;
      [[0, half + wt / 2, 2 * half + 2 * wt, wt], [0, -half - wt / 2, 2 * half + 2 * wt, wt], [half + wt / 2, 0, wt, 2 * half], [-half - wt / 2, 0, wt, 2 * half]].forEach(function (w) {
        var m = new T.Mesh(new T.BoxGeometry(w[2], wh, w[3]), wallMat);
        m.position.set(w[0], wh / 2, -w[1]);
        staticGroup.add(m);
      });
      ['red', 'blue'].forEach(function (al) {
        var z = F.loadingZones[al];
        var m = new T.Mesh(new T.PlaneGeometry(z.x1 - z.x0, z.y1 - z.y0), new T.MeshBasicMaterial({ color: col(al), transparent: true, opacity: 0.35 }));
        m.rotation.x = -Math.PI / 2;
        m.position.set((z.x0 + z.x1) / 2, 0.1, -(z.y0 + z.y1) / 2);
        staticGroup.add(m);
      });
      var hm = hive(), fr = hm.frame, frameCol = col('ink-3');
      fr.legs.forEach(function (s) { rod(s[0], s[1], fr.tubeRadius, frameCol, staticGroup); });
      rod(fr.crossbar[0], fr.crossbar[1], fr.tubeRadius, frameCol, staticGroup);
      hm.hives.forEach(function (h) {
        for (var i = 0; i + 1 < h.armBar.length; i++) rod(h.armBar[i], h.armBar[i + 1], h.armRadius, frameCol, staticGroup);
        h.cells.forEach(function (cell) {
          var isT = h.alliance === state.target && cell.role === 'up';
          cellMesh(cell, col(h.alliance), cell.role === 'up' ? 0.32 : 0.18, isT ? col('ink') : col(h.alliance), staticGroup);
          if (isT) tube(cell.mouth.concat([cell.mouth[0]]), 0.45, col('pollen'), staticGroup);
        });
      });
      needs = true;
    }

    function update() {
      clear(dynGroup);
      var r = current, p = currentParams;
      if (!r || !p) return;
      var heading = r.best ? r.best.yawDeg : r.psi0Deg;
      if (!userOrbited) {
        orbit.az = Math.atan2(Math.sin(heading * DEG), -Math.cos(heading * DEG)) + 0.55;
        orbit.target.set((p.robot.x + F.hive.hiveX[p.target]) / 2, 22, -p.robot.y / 2);
      }
      var bodyH = Math.min(p.h0, 12);
      var robot = new T.Mesh(new T.BoxGeometry(18, bodyH, 18), new T.MeshStandardMaterial({ color: col('pollen'), roughness: 0.8 }));
      robot.position.set(p.robot.x, bodyH / 2, -p.robot.y);
      robot.rotation.y = heading * DEG;
      dynGroup.add(robot);
      var edges = new T.LineSegments(new T.EdgesGeometry(robot.geometry), new T.LineBasicMaterial({ color: col('ink') }));
      edges.position.copy(robot.position);
      edges.rotation.copy(robot.rotation);
      dynGroup.add(edges);
      if (p.h0 > bodyH) rod([p.robot.x, p.robot.y, bodyH], [p.robot.x, p.robot.y, p.h0], 0.8, col('ink-2'), dynGroup);
      var path = r.trajectory || (r.closestMiss && r.closestMiss.path);
      var ok = !!r.trajectory && r.best && r.best.reachable !== false;
      if (path) {
        tube(path, 0.7, ok ? col('pollen') : col('bad-mark'), dynGroup);
        var end = path[path.length - 1];
        var ball = new T.Mesh(new T.SphereGeometry(E.ballProps(p.ballId).rIn, 20, 14), new T.MeshStandardMaterial({ color: ok ? col('pollen') : col('bad-mark') }));
        ball.position.copy(V(end));
        dynGroup.add(ball);
      }
      needs = true;
    }

    function place() {
      var d = orbit.dist, ce = Math.cos(orbit.el);
      camera.position.set(orbit.target.x + d * ce * Math.cos(orbit.az), orbit.target.y + d * Math.sin(orbit.el), orbit.target.z + d * ce * Math.sin(orbit.az));
      camera.lookAt(orbit.target);
    }
    function resize() {
      var w = host.clientWidth || 400, h = host.clientHeight || 300;
      renderer.setSize(w, h, false);
      renderer.domElement.style.width = '100%';
      renderer.domElement.style.height = '100%';
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      needs = true;
    }
    (function loop() {
      if (needs) { place(); renderer.render(scene, camera); needs = false; }
      requestAnimationFrame(loop);
    })();

    var ptrs = {}, lastPinch = 0;
    host.addEventListener('pointerdown', function (e) { host.setPointerCapture(e.pointerId); ptrs[e.pointerId] = [e.clientX, e.clientY]; userOrbited = true; });
    host.addEventListener('pointermove', function (e) {
      if (!ptrs[e.pointerId]) return;
      var ids = Object.keys(ptrs);
      if (ids.length === 2) {
        ptrs[e.pointerId] = [e.clientX, e.clientY];
        var a = ptrs[ids[0]], b = ptrs[ids[1]], dd = Math.hypot(a[0] - b[0], a[1] - b[1]);
        if (lastPinch) orbit.dist = clamp(orbit.dist * lastPinch / dd, 60, 900);
        lastPinch = dd;
      } else {
        var prev = ptrs[e.pointerId];
        orbit.az += (e.clientX - prev[0]) * 0.008;
        orbit.el = clamp(orbit.el + (e.clientY - prev[1]) * 0.006, 0.05, 1.5);
        ptrs[e.pointerId] = [e.clientX, e.clientY];
      }
      needs = true;
    });
    function up(e) { delete ptrs[e.pointerId]; lastPinch = 0; }
    host.addEventListener('pointerup', up);
    host.addEventListener('pointercancel', up);
    host.addEventListener('wheel', function (e) { e.preventDefault(); userOrbited = true; orbit.dist = clamp(orbit.dist * Math.exp(e.deltaY * 0.001), 60, 900); needs = true; }, { passive: false });

    api.update = update;
    api.rebuild = function () { rebuild(); update(); };
    api.resize = resize;
    return api;
  })();

  // ------------------------------------------------------------------ controls
  var motorSel = $('motor');
  D.motors.motors.forEach(function (m) {
    var o = document.createElement('option');
    o.value = m.id;
    o.textContent = m.label + ' — ' + m.sku;
    motorSel.appendChild(o);
  });
  SH.wheels.forEach(function (w) {
    var o = document.createElement('option');
    o.value = w.id;
    o.textContent = w.label;
    $('wheel').appendChild(o);
  });
  SH.inertiaPresets.forEach(function (ip) {
    var o = document.createElement('option');
    o.value = ip.id;
    o.textContent = ip.label + ' · ' + (ip.inertiaKgM2 * 1e4).toFixed(1) + '×10⁻⁴ kg·m²';
    $('inertia').appendChild(o);
  });

  function syncControls() {
    document.querySelectorAll('.seg[data-key]').forEach(function (seg) {
      var key = seg.getAttribute('data-key'), val;
      if (key === 'ballId') val = state.ballId;
      else if (key === 'target') val = state.target;
      else if (key === 'upSide') val = String(state.hive[state.target]);
      else if (key === 'motorsPerWheel') val = String(state.shooter.motorsPerWheel);
      else if (key === 'type') val = state.shooter.type;
      else if (key === 'precision') val = state.precision;
      seg.querySelectorAll('button').forEach(function (b) { b.setAttribute('aria-pressed', String(b.getAttribute('data-v') === String(val))); });
    });
    $('h0').value = state.h0;
    $('h0Out').textContent = state.h0 + ' in';
    motorSel.value = state.motorId;
    $('gear').value = String(state.shooter.gear);
    $('wheel').value = state.shooter.wheelId;
    $('inertia').value = state.shooter.inertiaPreset;
    $('interval').value = state.shooter.shotInterval;
    $('intervalOut').textContent = nf(state.shooter.shotInterval, 2) + ' s';
    var dual = state.shooter.type === 'dual';
    $('topRatioCtl').hidden = !dual;
    $('topRatio').value = state.shooter.topRatio;
    $('topOut').textContent = Math.round(state.shooter.topRatio * 100) + '% of bottom · spin ratio ' + nf((1 - state.shooter.topRatio) / (1 + state.shooter.topRatio), 2);
    var eta = $('eta'), range = dual ? (MODEL.etaDualRange || [0.8, 0.95]) : (MODEL.etaSingleRange || [0.3, 0.5]);
    eta.min = range[0];
    eta.max = range[1];
    eta.value = dual ? state.shooter.etaDual : state.shooter.etaSingle;
    $('etaOut').textContent = nf(Number(eta.value), 2);
    $('etaWhat').textContent = dual ? 'average wheel surface speed' : 'wheel surface speed';
    $('etaHelp').textContent = dual ? 'Two wheels ideally launch at their average surface speed; slip and compression lose a little.' :
      'A hooded shooter ideally launches at half the wheel\'s surface speed. Measured shooters land around 0.30–0.45, so lower this if your real shots come out slow.';
    var pr = SH.precision[state.precision];
    $('precHelp').textContent = 'Each shot is off by about ±' + pr.sigThetaDeg + '° in angle, ±' + pr.sigYawDeg + '° in aim and ±' + nf(pr.sigShooter * 100, 1) + '% in speed.';
    $('rx').value = state.robot.x;
    $('ry').value = state.robot.y;
    $('rx').min = $('ry').min = -LIMIT;
    $('rx').max = $('ry').max = LIMIT;
    $('tipLabel').textContent = 'TIP ' + state.target + ' HIVE';
  }

  function changed(opts) {
    syncControls();
    if (opts && opts.hive) { hiveCache.key = ''; three.rebuild(); }
    drawField();
    request('coarse');
    requestFull();
  }

  document.querySelectorAll('.seg[data-key]').forEach(function (seg) {
    seg.addEventListener('click', function (e) {
      var b = e.target.closest('button');
      if (!b) return;
      var key = seg.getAttribute('data-key'), v = b.getAttribute('data-v');
      if (key === 'ballId') state.ballId = v;
      else if (key === 'target') { state.target = v; return changed({ hive: true }); }
      else if (key === 'upSide') { state.hive[state.target] = Number(v); return changed({ hive: true }); }
      else if (key === 'motorsPerWheel') state.shooter.motorsPerWheel = Number(v);
      else if (key === 'type') state.shooter.type = v;
      else if (key === 'precision') state.precision = v;
      changed();
    });
  });
  $('tipBtn').addEventListener('click', function () { state.hive[state.target] *= -1; changed({ hive: true }); });
  $('h0').addEventListener('input', function () { state.h0 = Number(this.value); changed(); });
  motorSel.addEventListener('change', function () { state.motorId = this.value; changed(); });
  $('gear').addEventListener('change', function () { state.shooter.gear = Number(this.value); changed(); });
  $('topRatio').addEventListener('input', function () { state.shooter.topRatio = Number(this.value); changed(); });
  $('wheel').addEventListener('change', function () { state.shooter.wheelId = this.value; changed(); });
  $('inertia').addEventListener('change', function () { state.shooter.inertiaPreset = this.value; changed(); });
  $('interval').addEventListener('input', function () { state.shooter.shotInterval = Number(this.value); changed(); });
  $('eta').addEventListener('input', function () {
    if (state.shooter.type === 'dual') state.shooter.etaDual = Number(this.value); else state.shooter.etaSingle = Number(this.value);
    changed();
  });
  ['rx', 'ry'].forEach(function (id) {
    $(id).addEventListener('change', function () {
      var x = Number($('rx').value), y = Number($('ry').value);
      if (!isFinite(x) || !isFinite(y)) return syncControls();
      moveRobotTo(x, y, 'key');
    });
  });

  // ------------------------------------------------------------------ theme + resize
  function themeChanged() { readPalette(); drawField(); drawSide(); three.rebuild(); if (current) renderVerdict(current); }
  if (window.matchMedia) {
    var mq = window.matchMedia('(prefers-color-scheme: dark)');
    if (mq.addEventListener) mq.addEventListener('change', themeChanged); else if (mq.addListener) mq.addListener(themeChanged);
  }
  new MutationObserver(themeChanged).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme', 'class'] });
  var resizeTimer = 0;
  function onResize() {
    clearTimeout(resizeTimer);
    resizeTimer = setTimeout(function () { layoutField(); drawField(); drawSide(); three.resize(); }, 60);
  }
  window.addEventListener('resize', onResize);
  if (window.ResizeObserver) new ResizeObserver(onResize).observe($('fieldWrap'));

  // ------------------------------------------------------------------ start
  readPalette();
  syncControls();
  layoutField();
  drawField();
  three.resize();
  three.rebuild();
  makeBackend().then(function (b) {
    backend = b;
    $('engineState').textContent = 'Engine: ' + b.kind;
    request('full');
    requestMotors();
  });
  window.__shotSim = { state: state, get current() { return current; }, get backend() { return backend && backend.kind; }, get heatCount() { return heatCount; }, get heatRunning() { return !!heatStop; } };
})();
