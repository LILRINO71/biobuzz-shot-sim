/*
 * BIOBUZZ Shot Sim - physics, geometry, search, verdict and motor engine.
 * No DOM, no dependencies. UMD: module.exports (Node) / self.ShotEngine (worker) / window.ShotEngine.
 *
 * Units: positions in inches (field frame, origin at field centre on the tile top, +x toward BLUE,
 * +y away from the audience, +z up), speeds in m/s, angles in degrees, headings CCW from +x.
 * Ball flight is integrated in SI and stored as Int16 millimetres.
 */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.ShotEngine = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  // ------------------------------------------------------------------ constants
  var IN_M = 0.0254;
  var LB_KG = 0.45359237;
  var IPM = 1 / 25.4;            // inches per millimetre
  var MPI = 25.4;                // millimetres per inch
  var DEG = Math.PI / 180;

  // Air + aerodynamics (SPEC 4). Not in the data files; data.shooter.aero may override.
  var AERO_DEFAULT = { rho: 1.20, g: 9.81, cd: 0.45, clSlope: 0.20, clSMax: 1.0 };

  // Balls (SPEC 4). data.shooter.balls may override.
  var BALLS_DEFAULT = {
    pollen: { id: 'pollen', label: 'POLLEN', Din: 2.80, massLb: 0.055, wallIn: 0.070 },
    nectar: { id: 'nectar', label: 'NECTAR', Din: 91.948 / 25.4, massLb: 0.091, wallIn: 2.159 / 25.4 }
  };

  // Trajectory table (SPEC 4)
  var TBL = { th0: 10, th1: 89, v0: 2.0, vRatio: 1.02, vMax: 16, dt: 0.0025, storeEvery: 2,
    tMax: 2.6, rhoMax: 5.6, zStopDesc: 0.55 };
  var MAX_SAMPLES = Math.ceil(TBL.tMax / (TBL.dt * TBL.storeEvery)) + 3;

  // Miss causes
  var C_HIT = 0, C_SHORT = 1, C_LONG = 2, C_LIP = 3, C_ROOF = 4, C_CELL = 5, C_FRAME = 6, C_WALL = 7, C_FLOOR = 8;
  var CAUSE_NAMES = ['hit', 'short', 'long', 'lip', 'roof', 'cell', 'frame', 'wall', 'floor'];

  // Obstacle kinds
  var K_SHELL = 1, K_PRISM = 2, K_ARM = 3, K_CAPSULE = 4;

  var VERDICT = { OK: 'POSSIBLE', MID: 'NOT CONSISTENT', BAD: "WON'T WORK" };

  function now() {
    return (typeof performance !== 'undefined' && performance.now) ? performance.now() : Date.now();
  }

  // ------------------------------------------------------------------ math helpers
  // erf: W. J. Cody-style rational approximation would be more exact; Abramowitz-Stegun 7.1.26
  // (|error| <= 1.5e-7) is plenty for cell masses.
  function erf(x) {
    var sign = x < 0 ? -1 : 1;
    var ax = x < 0 ? -x : x;
    var t = 1 / (1 + 0.3275911 * ax);
    var y = 1 - (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - 0.284496736) * t + 0.254829592) * t * Math.exp(-ax * ax);
    return sign * y;
  }
  function Phi(x) { return 0.5 * (1 + erf(x / Math.SQRT2)); }

  function clamp(v, lo, hi) { return v < lo ? lo : (v > hi ? hi : v); }

  function fmtInt(n) {
    var s = String(Math.round(Math.abs(n)));
    var out = '';
    while (s.length > 3) { out = ',' + s.slice(-3) + out; s = s.slice(0, -3); }
    return (n < 0 ? '-' : '') + s + out;
  }
  function fmt(n, d) { return Number(n).toFixed(d); }
  function pct(x) { return Math.round(x * 100) + '%'; }

  // point-segment squared distance in 3-D
  function segDist2(px, py, pz, ax, ay, az, bx, by, bz) {
    var dx = bx - ax, dy = by - ay, dz = bz - az;
    var L2 = dx * dx + dy * dy + dz * dz;
    var t = L2 > 0 ? ((px - ax) * dx + (py - ay) * dy + (pz - az) * dz) / L2 : 0;
    if (t < 0) t = 0; else if (t > 1) t = 1;
    var ex = px - ax - t * dx, ey = py - ay - t * dy, ez = pz - az - t * dz;
    return ex * ex + ey * ey + ez * ez;
  }
  function seg2Dist2(px, py, ax, ay, bx, by) {
    var dx = bx - ax, dy = by - ay;
    var L2 = dx * dx + dy * dy;
    var t = L2 > 0 ? ((px - ax) * dx + (py - ay) * dy) / L2 : 0;
    if (t < 0) t = 0; else if (t > 1) t = 1;
    var ex = px - ax - t * dx, ey = py - ay - t * dy;
    return ex * ex + ey * ey;
  }

  function convexHull(pts) {
    var p = pts.slice().sort(function (a, b) { return a[0] - b[0] || a[1] - b[1]; });
    if (p.length < 3) return p;
    function cross(o, a, b) { return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]); }
    var lower = [], upper = [], i;
    for (i = 0; i < p.length; i++) {
      while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], p[i]) <= 0) lower.pop();
      lower.push(p[i]);
    }
    for (i = p.length - 1; i >= 0; i--) {
      while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], p[i]) <= 0) upper.pop();
      upper.push(p[i]);
    }
    upper.pop(); lower.pop();
    return lower.concat(upper);
  }

  // ------------------------------------------------------------------ state
  var S = {
    ready: false, data: null, G: null, aero: null, balls: null, model: null, shooterData: null,
    motors: [], motorById: {}, tables: {}, tableKeys: [], obsCache: {}, gridCache: [], ctxCache: []
  };

  function num(v, d) { return (typeof v === 'number' && isFinite(v)) ? v : d; }

  // ------------------------------------------------------------------ init
  function init(data) {
    if (!data || !data.field || !data.motors || !data.shooter) throw new Error('ShotEngine.init needs {motors, field, shooter}');
    var F = data.field, H = F.hive, C = H.cell, fr = H.frame;
    var armRad = H.armDeg * DEG;
    var G = {
      half: F.field.half,
      wallTop: F.field.wallHeight,
      leave: F.field.half + 6,
      robotLimit: F.field.half - 9,
      pz: H.pivotZ, ca: Math.cos(armRad), sa: Math.sin(armRad), armDeg: H.armDeg,
      hx: { red: H.hiveX.red, blue: H.hiveX.blue },
      upStart: { red: H.upSideStart.red, blue: H.upSideStart.blue },
      aIn: C.aIn, aOut: C.aOut, depth: C.aOut - C.aIn,
      b0: C.b0, W2: C.width / 2, SH: C.shoulder, HT: C.height,
      swingWarnZ: num(H.swingClearanceWarnZ, 25.5),
      armR: H.armBar.radius, armAB: H.armBar.ab.map(function (p) { return [p[0], p[1]]; }),
      tubeR: fr.tubeRadius,
      legs: fr.legs.map(function (s) { return [s[0].slice(), s[1].slice()]; }),
      crossbar: [fr.crossbar[0].slice(), fr.crossbar[1].slice()],
      cornerR: fr.cornerBlocks ? fr.cornerBlocks.radius : fr.tubeRadius,
      corners: fr.cornerBlocks ? fr.cornerBlocks.segments.map(function (s) { return [s[0].slice(), s[1].slice()]; }) : [],
      frameRaw: fr
    };
    // roof segment (right half, in (q = |w|, B = b - b0)): (W2, SH) -> (0, HT)
    G.RX = -G.W2; G.RY = G.HT - G.SH; G.RL2 = G.RX * G.RX + G.RY * G.RY;
    G.roofK = (G.HT - G.SH) / G.W2;
    // pentagon area centroid (B above base)
    var aRect = 2 * G.W2 * G.SH, aTri = 0.5 * 2 * G.W2 * (G.HT - G.SH);
    G.centroidB = (aRect * G.SH / 2 + aTri * (G.SH + (G.HT - G.SH) / 3)) / (aRect + aTri);
    // derived mouth numbers
    G.lipZ = G.pz + G.aOut * G.sa + G.b0 * G.ca;
    G.lipU = G.aOut * G.ca - G.b0 * G.sa;
    G.topZ = G.pz + G.aOut * G.sa + (G.b0 + G.HT) * G.ca;
    G.topU = G.aOut * G.ca - (G.b0 + G.HT) * G.sa;
    S.G = G;
    S.data = data;
    S.shooterData = data.shooter;
    var m = data.shooter.model || {};
    S.model = {
      etaSingle: num(m.etaSingle, 0.45), etaDual: num(m.etaDual, 0.9), lossFactor: num(m.lossFactor, 2.0),
      sm: m.sigmaMotor || { h1: 0.8, h2: 0.95, s0: 0.005, s1: 0.03, s2: 0.06 },
      downgrade: num(m.downgradeHeadroom, 0.92),
      thPossible: m.thresholds ? num(m.thresholds.possible, 0.8) : 0.8,
      thMid: m.thresholds ? num(m.thresholds.notConsistent, 0.4) : 0.4,
      freeSpeedFactor: num(m.freeSpeedFactor, 1.0)
    };
    var ae = data.shooter.aero || {};
    S.aero = {
      rho: num(ae.rho, AERO_DEFAULT.rho), g: num(ae.g, AERO_DEFAULT.g), cd: num(ae.cd, AERO_DEFAULT.cd),
      clSlope: num(ae.clSlope, AERO_DEFAULT.clSlope), clSMax: num(ae.clSMax, AERO_DEFAULT.clSMax)
    };
    S.balls = {};
    Object.keys(BALLS_DEFAULT).forEach(function (k) {
      var o = (data.shooter.balls && data.shooter.balls[k]) || {};
      var d = BALLS_DEFAULT[k];
      S.balls[k] = { id: k, label: o.label || d.label, Din: num(o.Din, d.Din), massLb: num(o.massLb, d.massLb), wallIn: num(o.wallIn, d.wallIn) };
    });
    S.motors = data.motors.motors.slice();
    S.motorById = {};
    S.motors.forEach(function (mo) { S.motorById[mo.id] = mo; });
    S.tables = {}; S.tableKeys = []; S.obsCache = {}; S.gridCache = []; S.ctxCache = [];
    // HIVE bounding box: union of every obstacle over all tilt states and both targets
    var hb = [Infinity, Infinity, Infinity, -Infinity, -Infinity, -Infinity];
    [-1, 1].forEach(function (sr) {
      [-1, 1].forEach(function (sb) {
        var ob = obstaclesFor({ red: sr, blue: sb }, 'red');
        for (var i = 0; i < ob.list.length; i++) {
          var bx = ob.list[i].box;
          for (var j = 0; j < 3; j++) { if (bx[j] < hb[j]) hb[j] = bx[j]; if (bx[j + 3] > hb[j + 3]) hb[j + 3] = bx[j + 3]; }
        }
      });
    });
    G.hiveBox = hb;
    // HIVE swing footprint (robot warning): x within any CELL width, |y| within the farthest CELL reach
    var maxU = 0;
    [[G.aIn, G.aOut], [-G.aOut, -G.aIn]].forEach(function (ab) {
      penta().forEach(function (wb) {
        [ab[0], ab[1]].forEach(function (a) {
          var u = Math.abs(a * G.ca - wb[1] * G.sa);
          if (u > maxU) maxU = u;
        });
      });
    });
    G.swingMaxU = maxU;
    S.ready = true;
    return { ok: true, motors: S.motors.length };
  }

  function need() { if (!S.ready) throw new Error('ShotEngine.init(data) has not been called'); }

  // ------------------------------------------------------------------ balls
  function ballProps(ballId) {
    need();
    var b = S.balls[ballId];
    if (!b) throw new Error('unknown ball ' + ballId);
    var R = b.Din / 2 * IN_M, Ri = R - b.wallIn * IN_M, m = b.massLb * LB_KG;
    var I = 0.4 * m * (Math.pow(R, 5) - Math.pow(Ri, 5)) / (Math.pow(R, 3) - Math.pow(Ri, 3));
    return { id: b.id, label: b.label, Din: b.Din, rIn: b.Din / 2, massKg: m, I: I, areaM2: Math.PI * R * R };
  }

  function terminalSpeed(ballId) {
    var bp = ballProps(ballId), A = S.aero;
    return Math.sqrt(2 * bp.massKg * A.g / (A.rho * A.cd * bp.areaM2));
  }

  // ------------------------------------------------------------------ HIVE geometry
  function penta() {
    var G = S.G;
    return [[-G.W2, G.b0], [G.W2, G.b0], [G.W2, G.b0 + G.SH], [0, G.b0 + G.HT], [-G.W2, G.b0 + G.SH]];
  }

  // world (x,y,z) -> body (w,a,b) for a HIVE centred at hx with up side sigma
  function worldToBody(x, y, z, hx, sigma) {
    var G = S.G, u = sigma * y, dz = z - G.pz;
    return [x - hx, u * G.ca + dz * G.sa, -u * G.sa + dz * G.ca];
  }
  function bodyToWorld(w, a, b, hx, sigma) {
    var G = S.G, u = a * G.ca - b * G.sa;
    return [hx + w, sigma * u, G.pz + a * G.sa + b * G.ca];
  }

  // squared distance from (q = |w|, B = b - b0) to the pentagon boundary
  function dEdge2qb(q, B) {
    var G = S.G, W2 = G.W2, SH = G.SH;
    var dx = q > W2 ? q - W2 : 0;
    var d = dx * dx + B * B;
    var ex = q - W2, ey = B < 0 ? -B : (B > SH ? B - SH : 0);
    var d2 = ex * ex + ey * ey;
    if (d2 < d) d = d2;
    var px = q - W2, py = B - SH;
    var t = (px * G.RX + py * G.RY) / G.RL2;
    if (t < 0) t = 0; else if (t > 1) t = 1;
    var fx = px - t * G.RX, fy = py - t * G.RY;
    d2 = fx * fx + fy * fy;
    return d2 < d ? d2 : d;
  }
  function insideqb(q, B) {
    var G = S.G;
    return B >= 0 && q <= G.W2 && B <= G.HT - G.roofK * q;
  }
  // public (w, b) versions
  function dEdge(w, b) { return Math.sqrt(dEdge2qb(Math.abs(w), b - S.G.b0)); }
  function dPoly(w, b) { var q = Math.abs(w), B = b - S.G.b0; return insideqb(q, B) ? 0 : Math.sqrt(dEdge2qb(q, B)); }
  function insidePentagon(w, b) { return insideqb(Math.abs(w), b - S.G.b0); }
  function dInt(a, lo, hi) { return a < lo ? lo - a : (a > hi ? a - hi : 0); }
  function prismDist(w, a, b, aMin, aMax) { return Math.hypot(dPoly(w, b), dInt(a, aMin, aMax)); }
  function shellDist(w, a, b) {
    var G = S.G, f = a - G.aOut;
    if (f > 0) return Math.hypot(dEdge(w, b), f);
    if (f >= -G.depth) return insidePentagon(w, b) ? dEdge(w, b) : dPoly(w, b);
    return Math.hypot(dPoly(w, b), f + G.depth);
  }
  function capsuleDist(p, s0, s1, R) {
    return Math.sqrt(segDist2(p[0], p[1], p[2], s0[0], s0[1], s0[2], s1[0], s1[1], s1[2])) - R;
  }

  function normState(hs) {
    var G = S.G;
    hs = hs || {};
    return { red: hs.red === 1 || hs.red === -1 ? hs.red : G.upStart.red, blue: hs.blue === 1 || hs.blue === -1 ? hs.blue : G.upStart.blue };
  }

  function mouthCentroid(alliance, sigma) {
    var G = S.G;
    return bodyToWorld(0, G.aOut, G.b0 + G.centroidB, G.hx[alliance], sigma);
  }

  function cellWorld(hx, sigma, up) {
    var G = S.G, P = penta();
    var aClosed = up ? G.aIn : -G.aIn, aOpen = up ? G.aOut : -G.aOut;
    var verts = [], mouth = [];
    P.forEach(function (wb) { verts.push(bodyToWorld(wb[0], aClosed, wb[1], hx, sigma)); });
    P.forEach(function (wb) { var p = bodyToWorld(wb[0], aOpen, wb[1], hx, sigma); verts.push(p); mouth.push(p); });
    return { role: up ? 'up' : 'down', vertices: verts, mouth: mouth };
  }

  function hiveModel(hiveState) {
    need();
    var G = S.G, hs = normState(hiveState);
    var hives = ['red', 'blue'].map(function (al) {
      var sg = hs[al], hx = G.hx[al];
      return {
        alliance: al, hx: hx, sigma: sg,
        cells: [cellWorld(hx, sg, true), cellWorld(hx, sg, false)],
        armBar: G.armAB.map(function (ab) { return bodyToWorld(0, ab[0], ab[1], hx, sg); }),
        armRadius: G.armR
      };
    });
    var fr = G.frameRaw;
    function cp(o) { return JSON.parse(JSON.stringify(o)); }
    return {
      hives: hives,
      frame: {
        legs: cp(fr.legs), crossbar: cp(fr.crossbar),
        cornerBlocks: fr.cornerBlocks ? cp(fr.cornerBlocks) : null,
        footBars: fr.footBars ? cp(fr.footBars) : null,
        logoPanels: fr.logoPanels ? cp(fr.logoPanels) : null,
        tubeRadius: fr.tubeRadius
      },
      mouthCentroid: { red: mouthCentroid('red', hs.red), blue: mouthCentroid('blue', hs.blue) },
      lipZ: G.lipZ, topZ: G.topZ
    };
  }

  // ------------------------------------------------------------------ obstacles
  function boxOfPoints(pts, R) {
    var b = [Infinity, Infinity, Infinity, -Infinity, -Infinity, -Infinity];
    pts.forEach(function (p) {
      for (var j = 0; j < 3; j++) { if (p[j] - R < b[j]) b[j] = p[j] - R; if (p[j] + R > b[j + 3]) b[j + 3] = p[j] + R; }
    });
    return b;
  }

  // Obstacle list for a tilt state and target alliance. Index 0 is always the target up-CELL shell.
  function obstaclesFor(hiveState, target) {
    var G = S.G, hs = normState(hiveState);
    var key = hs.red + '|' + hs.blue + '|' + target;
    if (S.obsCache[key]) return S.obsCache[key];
    var list = [];
    var tIdx = target === 'blue' ? 1 : 0;
    var hives = [{ al: 'red', hx: G.hx.red, sg: hs.red }, { al: 'blue', hx: G.hx.blue, sg: hs.blue }];
    var T = hives[tIdx], O = hives[1 - tIdx];
    list.push({ kind: K_SHELL, hive: tIdx, hx: T.hx, sg: T.sg, aMin: G.aIn, aMax: G.aOut, cause: C_ROOF,
      box: boxOfPoints(cellWorld(T.hx, T.sg, true).vertices, 0), name: 'target up-CELL' });
    list.push({ kind: K_PRISM, hive: tIdx, hx: T.hx, sg: T.sg, aMin: -G.aOut, aMax: -G.aIn, cause: C_CELL,
      box: boxOfPoints(cellWorld(T.hx, T.sg, false).vertices, 0), name: 'target down-CELL' });
    list.push({ kind: K_PRISM, hive: 1 - tIdx, hx: O.hx, sg: O.sg, aMin: G.aIn, aMax: G.aOut, cause: C_CELL,
      box: boxOfPoints(cellWorld(O.hx, O.sg, true).vertices, 0), name: 'other up-CELL' });
    list.push({ kind: K_PRISM, hive: 1 - tIdx, hx: O.hx, sg: O.sg, aMin: -G.aOut, aMax: -G.aIn, cause: C_CELL,
      box: boxOfPoints(cellWorld(O.hx, O.sg, false).vertices, 0), name: 'other down-CELL' });
    [T, O].forEach(function (h, i) {
      var pts = G.armAB.map(function (ab) { return bodyToWorld(0, ab[0], ab[1], h.hx, h.sg); });
      list.push({ kind: K_ARM, hive: i === 0 ? tIdx : 1 - tIdx, hx: h.hx, sg: h.sg, R: G.armR, cause: C_FRAME,
        box: boxOfPoints(pts, G.armR), name: 'arm bar' });
    });
    function capsule(s, R, name) {
      list.push({ kind: K_CAPSULE, p0: s[0], p1: s[1], R: R, cause: C_FRAME, box: boxOfPoints([s[0], s[1]], R), name: name });
    }
    G.legs.forEach(function (s) { capsule(s, G.tubeR, 'frame leg'); });
    capsule(G.crossbar, G.tubeR, 'crossbar');
    G.corners.forEach(function (s) { capsule(s, G.cornerR, 'corner block'); });
    var res = { key: key, list: list, hs: hs, target: target, T: T, O: O };
    S.obsCache[key] = res;
    return res;
  }

  // ------------------------------------------------------------------ ball flight (SPEC 4)
  function aeroK(bp, opts) {
    var A = S.aero;
    opts = opts || {};
    var rho = num(opts.rho, A.rho), cd = num(opts.cd, A.cd), cls = num(opts.clSlope, A.clSlope);
    return {
      kd: 0.5 * rho * cd * bp.areaM2 / bp.massKg,
      kl0: 0.5 * rho * bp.areaM2 / bp.massKg * cls,
      smax: num(opts.clSMax, A.clSMax),
      g: num(opts.g, A.g)
    };
  }

  /**
   * Integrate one shot in its vertical plane (RK4, dt 2.5 ms), storing (rho, z) in integer millimetres
   * every 5 ms relative to the shooter exit. info <- [n, apexIdx, monoIdx, preMaxRho].
   *   apexIdx  = last index of the highest stored z (samples after it are "descending")
   *   monoIdx  = first index from which rho never decreases again (backspin can pull steep lobs back)
   *   preMaxRho = max rho over [0, monoIdx]
   * lim = {tMax, rhoMax, zStop} (truncation, SPEC 4); defaults are the table limits.
   */
  function fly(K, S0, thetaDeg, v0, buf, info, lim) {
    var th = thetaDeg * DEG;
    var x = 0, z = 0, vx = v0 * Math.cos(th), vz = v0 * Math.sin(th);
    var wr = S0 * v0;
    var kd = K.kd, kl0 = K.kl0, smax = K.smax, g = K.g;
    var dt = TBL.dt, h2 = dt / 2, h6 = dt / 6;
    var tMax = lim && lim.tMax != null ? lim.tMax : TBL.tMax;
    var rhoMax = lim && lim.rhoMax != null ? lim.rhoMax : TBL.rhoMax;
    var zStop = lim && lim.zStop != null ? lim.zStop : TBL.zStopDesc;
    var maxN = (buf.length >> 1);
    var nStepsMax = Math.round(tMax / dt);
    buf[0] = 0; buf[1] = 0;
    var n = 1, apexIdx = 0, apexZ = 0, step = 0;
    var V, Sr, kl, a1x, a1z, a2x, a2z, a3x, a3z, a4x, a4z, vx2, vz2, vx3, vz3, vx4, vz4;
    for (;;) {
      V = Math.sqrt(vx * vx + vz * vz); Sr = V > 1e-9 ? wr / V : 0; if (Sr > smax) Sr = smax; kl = kl0 * Sr;
      a1x = -kd * V * vx - kl * V * vz; a1z = -g - kd * V * vz + kl * V * vx;
      vx2 = vx + h2 * a1x; vz2 = vz + h2 * a1z;
      V = Math.sqrt(vx2 * vx2 + vz2 * vz2); Sr = V > 1e-9 ? wr / V : 0; if (Sr > smax) Sr = smax; kl = kl0 * Sr;
      a2x = -kd * V * vx2 - kl * V * vz2; a2z = -g - kd * V * vz2 + kl * V * vx2;
      vx3 = vx + h2 * a2x; vz3 = vz + h2 * a2z;
      V = Math.sqrt(vx3 * vx3 + vz3 * vz3); Sr = V > 1e-9 ? wr / V : 0; if (Sr > smax) Sr = smax; kl = kl0 * Sr;
      a3x = -kd * V * vx3 - kl * V * vz3; a3z = -g - kd * V * vz3 + kl * V * vx3;
      vx4 = vx + dt * a3x; vz4 = vz + dt * a3z;
      V = Math.sqrt(vx4 * vx4 + vz4 * vz4); Sr = V > 1e-9 ? wr / V : 0; if (Sr > smax) Sr = smax; kl = kl0 * Sr;
      a4x = -kd * V * vx4 - kl * V * vz4; a4z = -g - kd * V * vz4 + kl * V * vx4;
      x += h6 * (vx + 2 * vx2 + 2 * vx3 + vx4);
      z += h6 * (vz + 2 * vz2 + 2 * vz3 + vz4);
      vx += h6 * (a1x + 2 * a2x + 2 * a3x + a4x);
      vz += h6 * (a1z + 2 * a2z + 2 * a3z + a4z);
      step++;
      if ((step & 1) === 0) {
        var zm = Math.round(z * 1000), xm = Math.round(x * 1000);
        if (zm > 32767) zm = 32767; else if (zm < -32768) zm = -32768;
        if (xm > 32767) xm = 32767; else if (xm < -32768) xm = -32768;
        buf[2 * n] = xm; buf[2 * n + 1] = zm;
        if (zm >= apexZ) { apexZ = zm; apexIdx = n; }
        n++;
        if ((vz < 0 && z < zStop) || step >= nStepsMax || x > rhoMax || n >= maxN) break;
      }
    }
    var m = n - 1;
    while (m > 0 && buf[2 * (m - 1)] <= buf[2 * m]) m--;
    var pm = -32768;
    for (var i = 0; i <= m; i++) if (buf[2 * i] > pm) pm = buf[2 * i];
    info[0] = n; info[1] = apexIdx; info[2] = m; info[3] = pm;
    return n;
  }

  // float path for tests/drawing without quantisation (same integrator maths), SI units
  function flyFloat(ballId, S0, thetaDeg, v0, opts) {
    need();
    opts = opts || {};
    var bp = ballProps(ballId), K = aeroK(bp, opts);
    var dt = num(opts.dt, TBL.dt), tEnd = num(opts.tEnd, 1.0);
    var th = thetaDeg * DEG, x = 0, z = 0, vx = v0 * Math.cos(th), vz = v0 * Math.sin(th), wr = S0 * v0;
    function acc(ux, uz) {
      var V = Math.sqrt(ux * ux + uz * uz), Sr = V > 1e-9 ? wr / V : 0; if (Sr > K.smax) Sr = K.smax;
      var kl = K.kl0 * Sr;
      return [-K.kd * V * ux - kl * V * uz, -K.g - K.kd * V * uz + kl * V * ux];
    }
    var steps = Math.round(tEnd / dt), out = [[0, 0, vx, vz]];
    for (var s = 0; s < steps; s++) {
      var a1 = acc(vx, vz);
      var b2x = vx + dt / 2 * a1[0], b2z = vz + dt / 2 * a1[1], a2 = acc(b2x, b2z);
      var b3x = vx + dt / 2 * a2[0], b3z = vz + dt / 2 * a2[1], a3 = acc(b3x, b3z);
      var b4x = vx + dt * a3[0], b4z = vz + dt * a3[1], a4 = acc(b4x, b4z);
      x += dt / 6 * (vx + 2 * b2x + 2 * b3x + b4x);
      z += dt / 6 * (vz + 2 * b2z + 2 * b3z + b4z);
      vx += dt / 6 * (a1[0] + 2 * a2[0] + 2 * a3[0] + a4[0]);
      vz += dt / 6 * (a1[1] + 2 * a2[1] + 2 * a3[1] + a4[1]);
      out.push([x, z, vx, vz]);
    }
    return out;
  }

  // ------------------------------------------------------------------ trajectory table
  function tableSpeeds() {
    var v = [];
    for (var j = 0; ; j++) {
      var s = TBL.v0 * Math.pow(TBL.vRatio, j);
      if (s > TBL.vMax + 1e-9) break;
      v.push(s);
    }
    return v;
  }

  function buildTable(ballId, S0) {
    need();
    S0 = num(+S0, 1);
    var key = ballId + '|' + S0.toFixed(4);
    if (S.tables[key]) { S.tables[key].hits = (S.tables[key].hits || 0) + 1; return S.tables[key]; }
    var t0 = now();
    var bp = ballProps(ballId), K = aeroK(bp);
    var th = [];
    for (var a = TBL.th0; a <= TBL.th1; a++) th.push(a);
    var sp = tableSpeeds();
    var nT = th.length, nV = sp.length, N = nT * nV;
    var trajs = new Array(N), lens = new Int32Array(N), apex = new Int32Array(N), mono = new Int32Array(N), preMax = new Int32Array(N);
    var buf = new Int16Array(2 * MAX_SAMPLES), info = new Int32Array(4);
    var samples = 0;
    for (var i = 0; i < nT; i++) {
      for (var j = 0; j < nV; j++) {
        var idx = i * nV + j;
        fly(K, S0, th[i], sp[j], buf, info);
        trajs[idx] = buf.slice(0, 2 * info[0]);
        lens[idx] = info[0]; apex[idx] = info[1]; mono[idx] = info[2]; preMax[idx] = info[3];
        samples += info[0];
      }
    }
    var tbl = {
      key: key, ballId: ballId, S0: S0, rIn: bp.rIn, K: K,
      thetas: Float64Array.from(th), speeds: Float64Array.from(sp), thStep: 1, lnStep: Math.log(TBL.vRatio),
      nT: nT, nV: nV, trajs: trajs, lens: lens, apex: apex, mono: mono, preMax: preMax,
      samples: samples, bytes: samples * 4, buildMs: 0, hits: 0
    };
    tbl.buildMs = now() - t0;
    S.tables[key] = tbl;
    S.tableKeys.push(key);
    while (S.tableKeys.length > 4) delete S.tables[S.tableKeys.shift()];
    return tbl;
  }

  // ------------------------------------------------------------------ params
  function presetInertia(id) {
    var ps = (S.shooterData.inertiaPresets || []);
    for (var i = 0; i < ps.length; i++) if (ps[i].id === id) return ps[i].inertiaKgM2;
    return null;
  }

  function shooterNorm(sh) {
    need();
    sh = sh || {};
    var def = S.shooterData.defaults || {};
    var wheelD = 96;
    (S.shooterData.wheels || []).forEach(function (w) { if (w.id === def.wheelId) wheelD = w.diameterMm; });
    var I = num(sh.inertiaKgM2, NaN);
    if (!isFinite(I) || I <= 0) I = presetInertia(sh.inertiaPreset) || presetInertia(def.inertiaPreset) || 4e-4;
    return {
      type: sh.type === 'dual' ? 'dual' : 'single',
      wheelDiameterMm: Math.max(10, num(sh.wheelDiameterMm, wheelD)),
      motorsPerWheel: Math.max(1, Math.round(num(sh.motorsPerWheel, num(def.motorsPerWheel, 1)))),
      gear: Math.max(0.05, num(sh.gear, num(def.gear, 1))),
      topRatio: clamp(num(sh.topRatio, num(def.topRatio, 0.6)), 0, 1),
      inertiaKgM2: I,
      shotInterval: clamp(num(sh.shotInterval, num(def.shotInterval, 0.5)), 0.01, 60),
      etaSingle: clamp(num(sh.etaSingle, S.model.etaSingle), 0.01, 1),
      etaDual: clamp(num(sh.etaDual, S.model.etaDual), 0.01, 1.5)
    };
  }

  function shooterSpin(shooter) {
    var sh = shooterNorm(shooter);
    if (sh.type === 'dual') return (1 - sh.topRatio) / (1 + sh.topRatio);
    return 1;
  }

  function defaultMotorId() {
    var def = S.shooterData.defaults || {};
    for (var i = 0; i < S.motors.length; i++) if (S.motors[i].freeRpm === def.motorFreeRpm) return S.motors[i].id;
    return S.motors[0].id;
  }

  function normParams(params) {
    need();
    params = params || {};
    var G = S.G, sd = S.shooterData, def = sd.defaults || {};
    var ballId = params.ballId === 'nectar' ? 'nectar' : 'pollen';
    var target = params.target === 'blue' ? 'blue' : 'red';
    var hs = normState(params.hiveState);
    var rob = params.robot || {};
    var rx = num(rob.x, G.hx.red), ry = num(rob.y, -48);
    var L = G.robotLimit;
    var x = clamp(rx, -L, L), y = clamp(ry, -L, L);
    var prDef = (sd.precision && sd.precision[def.precision || 'typical']) || { sigThetaDeg: 1, sigYawDeg: 1, sigShooter: 0.015 };
    var pr = params.precision || {};
    return {
      ballId: ballId, target: target, hiveState: hs,
      robot: { x: x, y: y }, robotIn: { x: rx, y: ry }, clamped: (x !== rx || y !== ry),
      h0: clamp(num(params.h0, num(def.h0, 16)), 4, 29),
      precision: {
        sigThetaDeg: Math.max(1e-3, num(pr.sigThetaDeg, prDef.sigThetaDeg)),
        sigYawDeg: Math.max(1e-3, num(pr.sigYawDeg, prDef.sigYawDeg)),
        sigShooter: Math.max(0, num(pr.sigShooter, prDef.sigShooter))
      },
      motorId: S.motorById[params.motorId] ? params.motorId : defaultMotorId(),
      shooter: shooterNorm(params.shooter)
    };
  }

  // ------------------------------------------------------------------ motor + flywheel (SPEC 6, 10)
  function sigmaMotorOf(h) {
    var sm = S.model.sm;
    if (!(h > sm.h1)) return sm.s0;
    if (h <= sm.h2) return sm.s0 + (sm.s1 - sm.s0) * (h - sm.h1) / (sm.h2 - sm.h1);
    if (h <= 1) return sm.s1 + (sm.s2 - sm.s1) * (h - sm.h2) / (1 - sm.h2);
    return sm.s2;
  }

  // exit speed per unit bottom-wheel surface speed
  function exitFactor(sh) {
    return sh.type === 'dual' ? sh.etaDual * (1 + sh.topRatio) / 2 : sh.etaSingle;
  }

  function motorCore(mo, sh, bp, vExit) {
    var M = S.model;
    var fx = exitFactor(sh);
    var D = sh.wheelDiameterMm / 1000;
    var free = mo.freeRpm * M.freeSpeedFactor;
    var vWheel = vExit / fx;
    var nW = vWheel / (Math.PI * D) * 60;
    var nM = nW / sh.gear;
    var h = nM / free;
    var vCap = fx * (free * sh.gear) * Math.PI * D / 60;
    var S0 = sh.type === 'dual' ? (1 - sh.topRatio) / (1 + sh.topRatio) : 1;
    var I = sh.inertiaKgM2;
    var w0 = nW * 2 * Math.PI / 60;
    var wwf = free * sh.gear * 2 * Math.PI / 60;
    var Tws = sh.motorsPerWheel * mo.stallTorqueNm / sh.gear;
    var tau = I * wwf / Tws;
    var share = sh.type === 'dual' ? 1 / (1 + sh.topRatio) : 1;
    var rM = bp.rIn * IN_M;
    var spinW = S0 * vExit / rM;
    var E = M.lossFactor * (0.5 * bp.massKg * vExit * vExit + 0.5 * bp.I * spinW * spinW) * share;
    var w1 = Math.sqrt(Math.max(0, w0 * w0 - 2 * E / I));
    var dip = w0 > 0 ? 1 - w1 / w0 : 0;
    var rec = w0 < wwf ? tau * Math.log((wwf - w1) / (wwf - w0)) : Infinity;
    var wdt = wwf - (wwf - w1) * Math.exp(-sh.shotInterval / tau);
    var residual = w0 > 0 ? Math.max(0, 1 - wdt / w0) : 0;
    var spin = w0 < wwf ? tau * Math.log(wwf / (wwf - w0)) : Infinity;
    return {
      motorId: mo.id, label: mo.label, freeRpm: mo.freeRpm, usableFreeRpm: free,
      vExit: vExit, wheelRpm: nW, motorRpm: nM, headroom: h, reachable: h <= 1, vCap: vCap, S0: S0,
      dip: dip, recoveryMs: rec * 1000, spinUpMs: spin * 1000, residual: residual,
      sigmaMotor: sigmaMotorOf(h), sigmaRecovery: residual / 2, tauS: tau, shotEnergyJ: E
    };
  }

  function motorModel(motorId, shooter, ballId, vExit) {
    need();
    var mo = S.motorById[motorId];
    if (!mo) throw new Error('unknown motor ' + motorId);
    return motorCore(mo, shooterNorm(shooter), ballProps(ballId === 'nectar' ? 'nectar' : 'pollen'), vExit);
  }

  function sigmaVOf(mc, sigShooter) {
    return Math.sqrt(sigShooter * sigShooter + mc.sigmaMotor * mc.sigmaMotor + mc.sigmaRecovery * mc.sigmaRecovery);
  }

  // ------------------------------------------------------------------ hit-test context
  function makeCtx(p, rIn) {
    var G = S.G, ob = obstaclesFor(p.hiveState, p.target), list = ob.list, nO = list.length, r = rIn;
    var ebox = new Float64Array(6 * nO), rr2 = new Float64Array(nO);
    for (var i = 0; i < nO; i++) {
      var b = list[i].box;
      ebox[6 * i] = b[0] - r; ebox[6 * i + 1] = b[1] - r; ebox[6 * i + 2] = b[2] - r;
      ebox[6 * i + 3] = b[3] + r; ebox[6 * i + 4] = b[4] + r; ebox[6 * i + 5] = b[5] + r;
      var R = list[i].R || 0;
      rr2[i] = (R + r) * (R + r);
    }
    var hb = G.hiveBox;
    var T = ob.T;
    var cen = mouthCentroid(p.target, T.sg);
    var x0 = p.robot.x, y0 = p.robot.y;
    var mouth = cellWorld(T.hx, T.sg, true).mouth;
    // horizontal distance range from the robot to the mouth pentagon (projected, convex)
    var dmax = 0, dmin = Infinity, inside = true, sgn = 0;
    for (var k = 0; k < 5; k++) {
      var P0 = mouth[k], P1 = mouth[(k + 1) % 5];
      var dv = Math.hypot(P0[0] - x0, P0[1] - y0);
      if (dv > dmax) dmax = dv;
      var d2 = seg2Dist2(x0, y0, P0[0], P0[1], P1[0], P1[1]);
      if (d2 < dmin) dmin = d2;
      var cr = (P1[0] - P0[0]) * (y0 - P0[1]) - (P1[1] - P0[1]) * (x0 - P0[0]);
      if (cr !== 0) { var sg = cr > 0 ? 1 : -1; if (sgn === 0) sgn = sg; else if (sg !== sgn) inside = false; }
    }
    dmin = inside ? 0 : Math.sqrt(dmin);
    return {
      p: p, r: r, r2: r * r, subStep: 0.4 * r,
      x0: x0, y0: y0, h0: p.h0,
      obs: list, nO: nO, ebox: ebox, rr2: rr2,
      hbox: [hb[0] - r, hb[1] - r, hb[2] - r, hb[3] + r, hb[4] + r, hb[5] + r],
      T: T, O: ob.O,
      lipLimit: G.lipZ - r,
      cen: cen,
      psi0: Math.atan2(cen[1] - y0, cen[0] - x0) / DEG,
      dist: Math.hypot(cen[0] - x0, cen[1] - y0),
      dminMM: dmin * MPI - 1, dmaxMM: dmax * MPI + 1,
      zCantMM: (G.lipZ - r - p.h0) * MPI,
      zBandLoMM: (G.lipZ - p.h0) * MPI - 1,
      zBandHiMM: (G.topZ - p.h0) * MPI + 1,
      zNearLoMM: (cen[2] - 12 - p.h0) * MPI, zNearHiMM: (cen[2] + 12 - p.h0) * MPI,
      wallIn: G.half - r, wallTopR: G.wallTop + r, leave: G.leave
    };
  }

  function exitDist(x0, y0, c, s, L) {
    var tx = c > 1e-12 ? (L - x0) / c : (c < -1e-12 ? (-L - x0) / c : Infinity);
    var ty = s > 1e-12 ? (L - y0) / s : (s < -1e-12 ? (-L - y0) / s : Infinity);
    return tx < ty ? tx : ty;
  }

  function yawCtx(ctx, psiDeg) {
    var G = S.G, c = Math.cos(psiDeg * DEG), s = Math.sin(psiDeg * DEG);
    var x0 = ctx.x0, y0 = ctx.y0, hb = ctx.hbox;
    var t0 = -Infinity, t1 = Infinity, has = true, ta, tb;
    if (Math.abs(c) < 1e-12) { if (x0 < hb[0] || x0 > hb[3]) has = false; }
    else { ta = (hb[0] - x0) / c; tb = (hb[3] - x0) / c; t0 = Math.max(t0, Math.min(ta, tb)); t1 = Math.min(t1, Math.max(ta, tb)); }
    if (Math.abs(s) < 1e-12) { if (y0 < hb[1] || y0 > hb[4]) has = false; }
    else { ta = (hb[1] - y0) / s; tb = (hb[4] - y0) / s; t0 = Math.max(t0, Math.min(ta, tb)); t1 = Math.min(t1, Math.max(ta, tb)); }
    if (t1 < t0 || t1 < 0) has = false;
    var T = ctx.T;
    return {
      psi: psiDeg, c: c, s: s, has: has,
      rin: t0 * MPI, rout: t1 * MPI,
      rWall: exitDist(x0, y0, c, s, ctx.wallIn) * MPI,
      rLeave: exitDist(x0, y0, c, s, ctx.leave) * MPI,
      sMouth: ((ctx.cen[0] - x0) * c + (ctx.cen[1] - y0) * s) * MPI,
      fA: T.sg * G.ca * s * IPM, fB: G.sa * IPM, fC: T.sg * G.ca * y0 + G.sa * (ctx.h0 - G.pz) - G.aOut
    };
  }

  // binary searches over P[2*i + off], i in [lo, hi); return hi when not found
  function firstGE(P, off, lo, hi, v) { while (lo < hi) { var m = (lo + hi) >> 1; if (P[2 * m + off] >= v) hi = m; else lo = m + 1; } return lo; }
  function firstGT(P, off, lo, hi, v) { while (lo < hi) { var m = (lo + hi) >> 1; if (P[2 * m + off] > v) hi = m; else lo = m + 1; } return lo; }
  function firstLE(P, off, lo, hi, v) { while (lo < hi) { var m = (lo + hi) >> 1; if (P[2 * m + off] <= v) hi = m; else lo = m + 1; } return lo; }
  function firstLT(P, off, lo, hi, v) { while (lo < hi) { var m = (lo + hi) >> 1; if (P[2 * m + off] < v) hi = m; else lo = m + 1; } return lo; }

  // index of the first descending sample below the "can't score any more" height (n if none)
  function cantIndex(ctx, P, n, apexIdx) {
    return firstLT(P, 1, apexIdx + 1, n, ctx.zCantMM);
  }

  // checks at a stored sample outside the HIVE box: floor, walls, left the field, can't score
  function pointOut(ctx, yc, x, y, z, desc, rho) {
    if (z < ctx.r) return C_FLOOR;
    var ax = x < 0 ? -x : x, ay = y < 0 ? -y : y;
    if ((ax > ctx.wallIn || ay > ctx.wallIn) && z < ctx.wallTopR) return C_WALL;
    if (ax > ctx.leave || ay > ctx.leave) return C_LONG;
    if (desc && z < ctx.lipLimit) return rho < yc.sMouth ? C_SHORT : C_LONG;
    return -1;
  }

  // full check at a (sub-)sample inside the HIVE box; cand = bitmask of obstacles whose box the segment touches
  function pointIn(ctx, yc, cand, px, py, pz, desc, rho) {
    var G = S.G, r2 = ctx.r2, obs = ctx.obs, nO = ctx.nO, rr2 = ctx.rr2;
    var tInside = false, f = 1, u, dz, a, b, q, B, d2;
    if (cand & 1) {
      var T = obs[0];
      u = T.sg * py; dz = pz - G.pz;
      a = u * G.ca + dz * G.sa; b = -u * G.sa + dz * G.ca;
      q = px - T.hx; if (q < 0) q = -q;
      B = b - G.b0; f = a - G.aOut;
      var ins = insideqb(q, B);
      if (f > 0) d2 = dEdge2qb(q, B) + f * f;
      else if (f >= -G.depth) d2 = dEdge2qb(q, B);
      else { var gg = f + G.depth; d2 = (ins ? 0 : dEdge2qb(q, B)) + gg * gg; }
      if (d2 < r2) return B < 1.5 ? C_LIP : C_ROOF;
      tInside = ins && f < 0 && f >= -G.depth;
    }
    for (var i = 1; i < nO; i++) {
      if (!(cand & (1 << i))) continue;
      var o = obs[i];
      if (o.kind === K_PRISM) {
        u = o.sg * py; dz = pz - G.pz;
        a = u * G.ca + dz * G.sa; b = -u * G.sa + dz * G.ca;
        q = px - o.hx; if (q < 0) q = -q;
        B = b - G.b0;
        var da = a < o.aMin ? o.aMin - a : (a > o.aMax ? a - o.aMax : 0);
        d2 = (insideqb(q, B) ? 0 : dEdge2qb(q, B)) + da * da;
        if (d2 < r2) return C_CELL;
      } else if (o.kind === K_ARM) {
        var w = px - o.hx, w2 = w * w;
        if (w2 >= rr2[i]) continue;
        u = o.sg * py; dz = pz - G.pz;
        a = u * G.ca + dz * G.sa; b = -u * G.sa + dz * G.ca;
        var ab = G.armAB, best = Infinity;
        for (var j = 0; j + 1 < ab.length; j++) {
          var s2 = seg2Dist2(a, b, ab[j][0], ab[j][1], ab[j + 1][0], ab[j + 1][1]);
          if (s2 < best) best = s2;
        }
        if (best + w2 < rr2[i]) return C_FRAME;
      } else {
        var p0 = o.p0, p1 = o.p1;
        if (segDist2(px, py, pz, p0[0], p0[1], p0[2], p1[0], p1[1], p1[2]) < rr2[i]) return C_FRAME;
      }
    }
    if (pz < ctx.r) return C_FLOOR;
    if (tInside && f < -ctx.r) return C_HIT;
    if (desc && pz < ctx.lipLimit && !tInside) return rho < yc.sMouth ? C_SHORT : C_LONG;
    return -1;
  }

  function setOut(out, code, k, t) { if (out) { out.code = code; out.k = k; out.t = t; } return code; }

  /**
   * SPEC 5 hit test of one stored trajectory at one heading. Returns the cause code (0 = scores) and
   * fills out.k / out.t (segment index and fraction where the decision was made).
   * Only the stretch whose heading distance overlaps the HIVE box is sub-sampled; before it nothing can
   * happen (robot inside the field, ball rising from h0 > r), after it only walls / leaving / can't-score.
   */
  function runTest(ctx, yc, P, n, apexIdx, mono, preMax, kz, out) {
    var kFrom = 1, kTo = 0, k, code;
    if (yc.has) {
      if (preMax < yc.rin) kFrom = firstGE(P, 0, mono > 1 ? mono : 1, n, yc.rin);
      if (kFrom < 1) kFrom = 1;
      var kE = firstGT(P, 0, kFrom > mono ? kFrom : mono, n, yc.rout);
      kTo = kE < n - 1 ? kE : n - 1;
      if (kTo > kz) kTo = kz;
    }
    if (kz < kFrom && kz < n) {
      return setOut(out, P[2 * kz] < yc.sMouth ? C_SHORT : C_LONG, kz, 1);
    }
    var x0 = ctx.x0, y0 = ctx.y0, h0 = ctx.h0, c = yc.c * IPM, s = yc.s * IPM;
    var hb = ctx.hbox, eb = ctx.ebox, nO = ctx.nO, step = ctx.subStep;
    for (k = kFrom; k <= kTo; k++) {
      var ra = P[2 * k - 2], za = P[2 * k - 1], rb = P[2 * k], zb = P[2 * k + 1];
      var xa = x0 + ra * c, ya = y0 + ra * s, zaw = h0 + za * IPM;
      var xb = x0 + rb * c, yb = y0 + rb * s, zbw = h0 + zb * IPM;
      var desc = k > apexIdx;
      var mnx, mxx, mny, mxy, mnz, mxz;
      if (xa < xb) { mnx = xa; mxx = xb; } else { mnx = xb; mxx = xa; }
      if (ya < yb) { mny = ya; mxy = yb; } else { mny = yb; mxy = ya; }
      if (zaw < zbw) { mnz = zaw; mxz = zbw; } else { mnz = zbw; mxz = zaw; }
      var cand = 0;
      if (!(mxx < hb[0] || mnx > hb[3] || mxy < hb[1] || mny > hb[4] || mxz < hb[2] || mnz > hb[5])) {
        for (var i = 0; i < nO; i++) {
          var o6 = 6 * i;
          if (mxx < eb[o6] || mnx > eb[o6 + 3] || mxy < eb[o6 + 1] || mny > eb[o6 + 4] || mxz < eb[o6 + 2] || mnz > eb[o6 + 5]) continue;
          cand |= (1 << i);
        }
      }
      if (cand === 0) {
        code = pointOut(ctx, yc, xb, yb, zbw, desc, rb);
        if (code >= 0) return setOut(out, code, k, 1);
        continue;
      }
      var dx = xb - xa, dy = yb - ya, dzz = zbw - zaw;
      var m = Math.ceil(Math.sqrt(dx * dx + dy * dy + dzz * dzz) / step);
      if (m < 1) m = 1;
      for (var j = 1; j <= m; j++) {
        var t = j / m;
        code = pointIn(ctx, yc, cand, xa + t * dx, ya + t * dy, zaw + t * dzz, desc, ra + t * (rb - ra));
        if (code >= 0) return setOut(out, code, k, t);
      }
    }
    // after the HIVE stretch
    var kS = kTo + 1;
    if (kS < 1) kS = 1;
    var mStart = kS > mono ? kS : mono;
    for (k = kS; k < mStart && k < n; k++) {
      code = pointOut(ctx, yc, x0 + P[2 * k] * c, y0 + P[2 * k] * s, h0 + P[2 * k + 1] * IPM, k > apexIdx, P[2 * k]);
      if (code >= 0) return setOut(out, code, k, 1);
    }
    if (mStart < n) {
      var ev = [kz, firstGE(P, 0, mStart, n, yc.rWall), firstGT(P, 0, mStart, n, yc.rLeave)];
      ev.sort(function (p1, p2) { return p1 - p2; });
      for (var e = 0; e < 3; e++) {
        k = ev[e];
        if (k < mStart || k >= n) continue;
        code = pointOut(ctx, yc, x0 + P[2 * k] * c, y0 + P[2 * k] * s, h0 + P[2 * k + 1] * IPM, k > apexIdx, P[2 * k]);
        if (code >= 0) return setOut(out, code, k, 1);
      }
    }
    return setOut(out, P[2 * (n - 1)] < yc.sMouth ? C_SHORT : C_LONG, n - 1, 1);
  }

  /**
   * Test one trajectory at every heading of a yaw fan. A heading can only score if the ball centre
   * crosses the mouth plane inward inside the pentagon with >= 0.5 r clearance (a necessary condition
   * implied by the SPEC 5 test with <= 0.4 r sub-steps); only those get the full runTest. The others get
   * a cheap explanatory cause. hits/causes are written at base + y.
   */
  var _out = { code: 0, k: 0, t: 0 };
  function testTraj(ctx, ycs, P, n, apexIdx, mono, preMax, hits, causes, base) {
    var G = S.G, nY = ycs.length, y;
    var kz = cantIndex(ctx, P, n, apexIdx);
    var zLo = ctx.zBandLoMM, zHi = ctx.zBandHiMM;
    if (P[2 * apexIdx + 1] < zLo) {
      for (y = 0; y < nY; y++) { hits[base + y] = 0; causes[base + y] = C_SHORT; }
      return 0;
    }
    // segments (k-1 -> k) whose z range overlaps the mouth band, rising and falling runs
    var a0 = firstGE(P, 1, 1, apexIdx + 1, zLo);
    var g1 = firstGT(P, 1, 0, apexIdx + 1, zHi);
    var a1 = g1 < apexIdx ? g1 : apexIdx;
    var d0 = firstLE(P, 1, apexIdx + 1, n, zHi);
    var e1 = firstLT(P, 1, apexIdx, n, zLo);
    var d1 = e1 < n - 1 ? e1 : n - 1;
    if (a1 > kz) a1 = kz;
    if (d1 > kz) d1 = kz;
    var dmin = ctx.dminMM, dmax = ctx.dmaxMM, any = false, firstRho = null, k;
    for (k = a0; k <= a1 && !any; k++) {
      var r0 = P[2 * k - 2], r1 = P[2 * k];
      if (firstRho === null) firstRho = r0;
      if ((r0 > r1 ? r0 : r1) >= dmin && (r0 < r1 ? r0 : r1) <= dmax) any = true;
    }
    for (k = d0; k <= d1 && !any; k++) {
      var q0 = P[2 * k - 2], q1 = P[2 * k];
      if (firstRho === null) firstRho = q0;
      if ((q0 > q1 ? q0 : q1) >= dmin && (q0 < q1 ? q0 : q1) <= dmax) any = true;
    }
    if (!any) {
      var cz = (firstRho === null || firstRho < dmin) ? C_SHORT : C_LONG;
      for (y = 0; y < nY; y++) { hits[base + y] = 0; causes[base + y] = cz; }
      return 0;
    }
    var nh = 0, x0 = ctx.x0, y0 = ctx.y0, h0 = ctx.h0, T = ctx.T, r2 = ctx.r2;
    for (y = 0; y < nY; y++) {
      var yc = ycs[y], fA = yc.fA, fB = yc.fB, fC = yc.fC;
      var cand = false, cheap = -1, lastF = 0, run, kk, kEnd;
      for (run = 0; run < 2 && !cand; run++) {
        kk = run === 0 ? a0 : d0;
        kEnd = run === 0 ? a1 : d1;
        if (kk > kEnd) continue;
        var fp = fA * P[2 * kk - 2] + fB * P[2 * kk - 1] + fC;
        for (; kk <= kEnd; kk++) {
          var fk = fA * P[2 * kk] + fB * P[2 * kk + 1] + fC;
          lastF = fk;
          if (fp >= 0 && fk < 0) {
            var t = fp / (fp - fk);
            var rr = P[2 * kk - 2] + t * (P[2 * kk] - P[2 * kk - 2]);
            var zz = P[2 * kk - 1] + t * (P[2 * kk + 1] - P[2 * kk - 1]);
            var px = x0 + rr * IPM * yc.c, py = y0 + rr * IPM * yc.s, pz = h0 + zz * IPM;
            var u = T.sg * py, dz = pz - G.pz;
            var bb = -u * G.sa + dz * G.ca, B = bb - G.b0;
            var q = px - T.hx; if (q < 0) q = -q;
            var ins = insideqb(q, B), de2 = dEdge2qb(q, B);
            if (ins && de2 >= 0.25 * r2) { cand = true; break; }
            if (cheap < 0) {
              if (de2 < r2) cheap = B < 1.5 ? C_LIP : C_ROOF;
              else if (B < 0) cheap = C_SHORT;
              else if (q > G.W2 && B <= G.HT) cheap = de2 < 9 ? C_ROOF : C_LONG;
              else cheap = C_LONG;
            }
          }
          fp = fk;
        }
      }
      if (cand) {
        var code = runTest(ctx, yc, P, n, apexIdx, mono, preMax, kz, _out);
        hits[base + y] = code === C_HIT ? 1 : 0;
        causes[base + y] = code;
        if (code === C_HIT) nh++;
      } else {
        hits[base + y] = 0;
        causes[base + y] = cheap >= 0 ? cheap : (lastF > 0 ? C_SHORT : C_LONG);
      }
    }
    return nh;
  }

  // ------------------------------------------------------------------ coarse grid (SPEC 7.1)
  function coarseYaws(ctx) {
    var d = Math.atan2(14, ctx.dist) / DEG / 8;
    var dPsi = Math.max(0.5, d), ys = [];
    for (var k = -8; k <= 8; k++) ys.push(ctx.psi0 + k * dPsi);
    return { dPsi: dPsi, yaws: ys };
  }

  function gridKey(p, S0) {
    return [p.ballId, S0.toFixed(4), p.target, p.hiveState.red, p.hiveState.blue,
      p.robot.x.toFixed(3), p.robot.y.toFixed(3), p.h0.toFixed(3)].join('|');
  }

  function getGrid(p, table) {
    var key = gridKey(p, table.S0);
    for (var i = 0; i < S.gridCache.length; i++) if (S.gridCache[i].key === key) return S.gridCache[i];
    var t0 = now();
    var ctx = makeCtx(p, table.rIn);
    var fan = coarseYaws(ctx);
    var ycs = fan.yaws.map(function (psi) { return yawCtx(ctx, psi); });
    var nT = table.nT, nV = table.nV, nY = ycs.length, N = nT * nV;
    var hits = new Uint8Array(N * nY), causes = new Uint8Array(N * nY);
    var nHits = 0, minHitJ = nV;
    for (var idx = 0; idx < N; idx++) {
      var nh = testTraj(ctx, ycs, table.trajs[idx], table.lens[idx], table.apex[idx], table.mono[idx], table.preMax[idx], hits, causes, idx * nY);
      if (nh > 0) { nHits += nh; var j = idx % nV; if (j < minHitJ) minHitJ = j; }
    }
    var grid = {
      key: key, table: table, ctx: ctx, ycs: ycs, yaws: fan.yaws, dPsi: fan.dPsi,
      nA: nT, nB: nV, nC: nY, hits: hits, causes: causes, nHits: nHits,
      minHitV: minHitJ < nV ? table.speeds[minHitJ] : null,
      ms: 0
    };
    grid.ms = now() - t0;
    S.gridCache.unshift(grid);
    while (S.gridCache.length > 3) S.gridCache.pop();
    return grid;
  }

  // ------------------------------------------------------------------ hit rate (SPEC 7.4)
  var _pool = {};
  function pool(name, size) {
    var b = _pool[name];
    if (!b || b.length < size) { b = new Float32Array(size); _pool[name] = b; }
    return b;
  }

  // per-axis cell masses for an aim at offset d cells (cells span half a step either side)
  function kernel(step, sigma) {
    if (!(sigma > 0)) sigma = 1e-9;
    var h = Math.ceil(4 * sigma / step);
    if (h > 400) h = 400;
    var k = new Float64Array(2 * h + 1);
    var prev = Phi((-h - 0.5) * step / sigma);
    for (var d = -h; d <= h; d++) {
      var cur = Phi((d + 0.5) * step / sigma);
      k[d + h] = cur - prev;
      prev = cur;
    }
    return { h: h, k: k };
  }

  /**
   * Separable Gaussian hit rate over a (theta, speed, yaw) boolean grid. Cells with speed index > cellMaxB
   * are misses; aims are only evaluated up to aimMaxB. KBs[b] is the ln-speed kernel for an aim at b
   * (sigma_v depends on the aim speed). Returns the rate field (pooled buffer) and the evaluated box.
   */
  function rateField(v, cellMaxB, aimMaxB, KA, KC, KBs, tag) {
    var H = v.hits, nA = v.nA, nB = v.nB, nC = v.nC;
    var amin = nA, amax = -1, bmin = nB, bmax = -1, cmin = nC, cmax = -1, a, b, c, d, i;
    var bTop = Math.min(cellMaxB, nB - 1);
    for (a = 0; a < nA; a++) {
      for (b = 0; b <= bTop; b++) {
        var base = (a * nB + b) * nC;
        for (c = 0; c < nC; c++) {
          if (H[base + c]) {
            if (a < amin) amin = a; if (a > amax) amax = a;
            if (b < bmin) bmin = b; if (b > bmax) bmax = b;
            if (c < cmin) cmin = c; if (c > cmax) cmax = c;
          }
        }
      }
    }
    if (amax < 0) return null;
    var size = nA * nB * nC;
    var T1 = pool('T1' + tag, size), T2 = pool('T2' + tag, size), R = pool('R' + tag, size);
    var hA = KA.h, ka = KA.k, hC = KC.h, kc = KC.k;
    var A0 = Math.max(0, amin - hA), A1 = Math.min(nA - 1, amax + hA);
    var C0 = Math.max(0, cmin - hC), C1 = Math.min(nC - 1, cmax + hC);
    var nBC = nB * nC;
    // theta pass
    for (a = A0; a <= A1; a++) {
      var dlo = Math.max(-hA, amin - a), dhi = Math.min(hA, amax - a);
      for (b = bmin; b <= bmax; b++) {
        for (c = cmin; c <= cmax; c++) {
          var s = 0;
          for (d = dlo; d <= dhi; d++) { if (H[((a + d) * nB + b) * nC + c]) s += ka[d + hA]; }
          T1[(a * nB + b) * nC + c] = s;
        }
      }
    }
    // yaw pass
    for (a = A0; a <= A1; a++) {
      for (b = bmin; b <= bmax; b++) {
        var row = (a * nB + b) * nC;
        for (c = C0; c <= C1; c++) {
          var lo = Math.max(-hC, cmin - c), hi = Math.min(hC, cmax - c), s2 = 0;
          for (d = lo; d <= hi; d++) s2 += kc[d + hC] * T1[row + c + d];
          T2[row + c] = s2;
        }
      }
    }
    // speed pass (aim-dependent kernel)
    var hBmax = 0;
    for (b = 0; b <= Math.min(aimMaxB, nB - 1); b++) if (KBs[b] && KBs[b].h > hBmax) hBmax = KBs[b].h;
    var B0 = Math.max(0, bmin - hBmax), B1 = Math.min(aimMaxB, nB - 1, bmax + hBmax);
    for (b = B0; b <= B1; b++) {
      var K = KBs[b], hB = K.h, kb = K.k;
      var e0 = Math.max(-hB, bmin - b), e1 = Math.min(hB, bmax - b);
      for (a = A0; a <= A1; a++) {
        var ab = a * nBC;
        for (c = C0; c <= C1; c++) {
          var s3 = 0;
          for (d = e0; d <= e1; d++) s3 += kb[d + hB] * T2[ab + (b + d) * nC + c];
          R[ab + b * nC + c] = s3;
        }
      }
    }
    if (B0 > B1) return null;
    return { R: R, A0: A0, A1: A1, B0: B0, B1: B1, C0: C0, C1: C1 };
  }

  function wrap180(d) { d = ((d + 180) % 360 + 360) % 360 - 180; return d; }

  /**
   * Best aim on a grid view {hits, causes, nA, nB, nC, thetas[], speeds[], yaws[], thStep, lnStep, yawStep}.
   * ignoreCap: evaluate as if the motor could reach any speed (used to explain "too slow").
   */
  function selectAim(v, p, mo, sh, bp, ignoreCap) {
    var vCap = motorCore(mo, sh, bp, 1).vCap;
    var maxB = -1;
    for (var b = 0; b < v.nB; b++) if (v.speeds[b] <= vCap * (1 + 1e-12)) maxB = b;
    var cellMaxB = ignoreCap ? v.nB - 1 : maxB;
    if (cellMaxB < 0) return null;
    var sigV = new Float64Array(v.nB), KBs = new Array(v.nB), KBs2 = new Array(v.nB), mcs = new Array(v.nB);
    for (b = 0; b <= cellMaxB; b++) {
      var mc = motorCore(mo, sh, bp, v.speeds[b]);
      mcs[b] = mc;
      sigV[b] = sigmaVOf(mc, p.precision.sigShooter);
      KBs[b] = kernel(v.lnStep, sigV[b]);
    }
    var KA = kernel(v.thStep, p.precision.sigThetaDeg), KC = kernel(v.yawStep, p.precision.sigYawDeg);
    var F = rateField(v, cellMaxB, cellMaxB, KA, KC, KBs, 'p');
    if (!F) return null;
    var R = F.R, nB = v.nB, nC = v.nC, best = -1, bi = -1, a, c, i;
    for (a = F.A0; a <= F.A1; a++) for (b = F.B0; b <= F.B1; b++) for (c = F.C0; c <= F.C1; c++) {
      i = (a * nB + b) * nC + c;
      if (R[i] > best) { best = R[i]; bi = i; }
    }
    if (bi < 0) return null;
    // tie-break a plateau with a wider (2 sigma) scatter so the aim sits in the middle of the window
    var tol = 0.002, ties = [];
    for (a = F.A0; a <= F.A1; a++) for (b = F.B0; b <= F.B1; b++) for (c = F.C0; c <= F.C1; c++) {
      i = (a * nB + b) * nC + c;
      if (R[i] >= best - tol) { ties.push(i); if (ties.length > 200000) break; }
    }
    if (ties.length > 1) {
      var tieVals = ties.map(function (ix) { return R[ix]; });
      for (b = 0; b <= cellMaxB; b++) KBs2[b] = kernel(v.lnStep, 2 * sigV[b]);
      var F2 = rateField(v, cellMaxB, cellMaxB, kernel(v.thStep, 2 * p.precision.sigThetaDeg), kernel(v.yawStep, 2 * p.precision.sigYawDeg), KBs2, 's');
      if (F2) {
        var b2 = -1;
        for (var t = 0; t < ties.length; t++) {
          var val = F2.R[ties[t]];
          if (val > b2) { b2 = val; bi = ties[t]; best = tieVals[t]; }
        }
      }
    }
    c = bi % nC; b = ((bi - c) / nC) % nB; a = ((bi - c) / nC - b) / nB;
    return {
      rate: Math.max(0, Math.min(1, best)), a: a, b: b, c: c,
      theta: v.thetas[a], v: v.speeds[b], yaw: v.yaws[c],
      sigV: sigV[b], mc: mcs[b], vCap: vCap, maxB: maxB, cellMaxB: cellMaxB,
      KA: KA, KC: KC, KB: KBs[b]
    };
  }

  // contiguous scoring run through the aim along each axis
  function windowsOf(v, sel) {
    var H = v.hits, nB = v.nB, nC = v.nC, a = sel.a, b = sel.b, c = sel.c;
    function hit(ai, bi, ci) { return bi <= sel.cellMaxB && H[(ai * nB + bi) * nC + ci] === 1; }
    var w = {};
    if (!hit(a, b, c)) {
      w.thetaDeg = [sel.theta, sel.theta]; w.v = [sel.v, sel.v]; w.yawDeg = [sel.yaw, sel.yaw];
      w.idx = { a: [a, a], b: [b, b], c: [c, c] }; w.aimHit = false;
      return w;
    }
    var lo = a, hi = a;
    while (lo > 0 && hit(lo - 1, b, c)) lo--;
    while (hi < v.nA - 1 && hit(hi + 1, b, c)) hi++;
    w.thetaDeg = [v.thetas[lo], v.thetas[hi]]; var ia = [lo, hi];
    lo = b; hi = b;
    while (lo > 0 && hit(a, lo - 1, c)) lo--;
    while (hi < v.nB - 1 && hit(a, hi + 1, c)) hi++;
    w.v = [v.speeds[lo], v.speeds[hi]]; var ib = [lo, hi];
    lo = c; hi = c;
    while (lo > 0 && hit(a, b, lo - 1)) lo--;
    while (hi < v.nC - 1 && hit(a, b, hi + 1)) hi++;
    w.yawDeg = [v.yaws[lo], v.yaws[hi]]; var ic = [lo, hi];
    w.idx = { a: ia, b: ib, c: ic }; w.aimHit = true;
    return w;
  }

  // kernel-weighted most common miss cause around the aim
  function nearMissCause(v, sel) {
    var H = v.hits, Cz = v.causes, nB = v.nB, nC = v.nC, w = new Float64Array(9), tot = 0;
    var KA = sel.KA, KB = sel.KB, KC = sel.KC;
    for (var da = -KA.h; da <= KA.h; da++) {
      var a = sel.a + da; if (a < 0 || a >= v.nA) continue;
      for (var db = -KB.h; db <= KB.h; db++) {
        var b = sel.b + db; if (b < 0 || b >= nB) continue;
        for (var dc = -KC.h; dc <= KC.h; dc++) {
          var c = sel.c + dc; if (c < 0 || c >= nC) continue;
          var i = (a * nB + b) * nC + c;
          var m = KA.k[da + KA.h] * KB.k[db + KB.h] * KC.k[dc + KC.h];
          if (H[i] && b <= sel.cellMaxB) continue;
          w[Cz[i] === C_HIT ? C_LONG : Cz[i]] += m; tot += m;
        }
      }
    }
    var best = -1, bc = 0;
    for (var k = 1; k < 9; k++) if (w[k] > best) { best = w[k]; bc = k; }
    return tot > 1e-4 ? CAUSE_NAMES[bc] : null;
  }

  // ------------------------------------------------------------------ fine stage (SPEC 7.5)
  var _fbuf = null, _finfo = new Int32Array(4);
  function fineStage(p, table, grid, sel, mo, sh, bp) {
    if (!_fbuf) _fbuf = new Int16Array(2 * MAX_SAMPLES);
    var ctx = grid.ctx, pr = p.precision;
    var Wt = Math.max(3, 3 * pr.sigThetaDeg), Wp = Math.max(3, 3 * pr.sigYawDeg), Wv = Math.max(0.06, 3 * sel.sigV);
    var ths = [], yaws = [], sps = [], i, n;
    n = Math.ceil(Wt / 0.25 - 1e-9);
    for (i = -n; i <= n; i++) { var th = sel.theta + i * 0.25; if (th >= 1 && th <= 89.75) ths.push(th); }
    n = Math.ceil(Wp / 0.25 - 1e-9);
    for (i = -n; i <= n; i++) yaws.push(sel.yaw + i * 0.25);
    n = Math.ceil(Wv / 0.005 - 1e-9);
    var lv0 = Math.log(sel.v);
    for (i = -n; i <= n; i++) { var vv = i === 0 ? sel.v : Math.exp(lv0 + i * 0.005); if (vv >= 0.5 && vv <= 40) sps.push(vv); }
    var nA = ths.length, nB = sps.length, nC = yaws.length;
    var hits = new Uint8Array(nA * nB * nC), causes = new Uint8Array(nA * nB * nC);
    var ycs = yaws.map(function (psi) { return yawCtx(ctx, psi); });
    var nHits = 0;
    for (var a = 0; a < nA; a++) {
      for (var b = 0; b < nB; b++) {
        fly(table.K, table.S0, ths[a], sps[b], _fbuf, _finfo);
        nHits += testTraj(ctx, ycs, _fbuf, _finfo[0], _finfo[1], _finfo[2], _finfo[3], hits, causes, (a * nB + b) * nC);
      }
    }
    var view = { hits: hits, causes: causes, nA: nA, nB: nB, nC: nC, thetas: ths, speeds: sps, yaws: yaws,
      thStep: 0.25, lnStep: 0.005, yawStep: 0.25, nHits: nHits };
    var fsel = nHits ? selectAim(view, p, mo, sh, bp, false) : null;
    return { view: view, sel: fsel };
  }

  function coarseView(grid) {
    var t = grid.table;
    return { hits: grid.hits, causes: grid.causes, nA: grid.nA, nB: grid.nB, nC: grid.nC,
      thetas: t.thetas, speeds: t.speeds, yaws: grid.yaws, thStep: t.thStep, lnStep: t.lnStep, yawStep: grid.dPsi, nHits: grid.nHits };
  }

  // ------------------------------------------------------------------ single shot (cross-check + drawing)
  var _cbuf = null, _cinfo = new Int32Array(4), _ctxCache = { key: null, ctx: null };
  function ctxFor(p, rIn) {
    var key = gridKey(p, 0) + '|' + rIn;
    if (_ctxCache.key !== key) { _ctxCache.key = key; _ctxCache.ctx = makeCtx(p, rIn); }
    return _ctxCache.ctx;
  }

  function classifyCore(p, ctx, K, S0, thetaDeg, v, yawDeg) {
    if (!_cbuf) _cbuf = new Int16Array(2 * MAX_SAMPLES);
    var P = _cbuf, info = _cinfo, G = S.G;
    fly(K, S0, thetaDeg, v, P, info);
    var n = info[0], apexIdx = info[1];
    var kz = cantIndex(ctx, P, n, apexIdx);
    var yc = yawCtx(ctx, yawDeg);
    var out = { code: 0, k: 0, t: 1 };
    var code = runTest(ctx, yc, P, n, apexIdx, info[2], info[3], kz, out);
    var c = yc.c * IPM, s = yc.s * IPM, x0 = ctx.x0, y0 = ctx.y0, h0 = ctx.h0;
    var kEnd = Math.max(1, Math.min(out.k, n - 1)), tEnd = out.t;
    var path = [], apex = -Infinity, i;
    for (i = 0; i < kEnd; i++) {
      var zi = h0 + P[2 * i + 1] * IPM;
      path.push([x0 + P[2 * i] * c, y0 + P[2 * i] * s, zi]);
      if (zi > apex) apex = zi;
    }
    var rE = P[2 * kEnd - 2] + tEnd * (P[2 * kEnd] - P[2 * kEnd - 2]);
    var zE = P[2 * kEnd - 1] + tEnd * (P[2 * kEnd + 1] - P[2 * kEnd - 1]);
    path.push([x0 + rE * c, y0 + rE * s, h0 + zE * IPM]);
    if (h0 + zE * IPM > apex) apex = h0 + zE * IPM;
    // last inward crossing of the mouth plane inside the pentagon, before the decision point
    var tTo = null, vIn = null, T = ctx.T;
    var fp = yc.fA * P[0] + yc.fB * P[1] + yc.fC;
    for (i = 1; i <= kEnd; i++) {
      var fk = yc.fA * P[2 * i] + yc.fB * P[2 * i + 1] + yc.fC;
      if (fp >= 0 && fk < 0) {
        var tc = fp / (fp - fk);
        if (!(i === kEnd && tc > tEnd)) {
          var rr = P[2 * i - 2] + tc * (P[2 * i] - P[2 * i - 2]), zz = P[2 * i - 1] + tc * (P[2 * i + 1] - P[2 * i - 1]);
          var bw = worldToBody(x0 + rr * c, y0 + rr * s, h0 + zz * IPM, T.hx, T.sg);
          if (insidePentagon(bw[0], bw[2])) {
            tTo = (i - 1 + tc) * TBL.dt * TBL.storeEvery;
            vIn = Math.hypot(P[2 * i] - P[2 * i - 2], P[2 * i + 1] - P[2 * i - 1]) / 1000 / (TBL.dt * TBL.storeEvery);
          }
        }
      }
      fp = fk;
    }
    return { hit: code === C_HIT, cause: code === C_HIT ? null : CAUSE_NAMES[code], path: path, apexIn: apex, tToCell: tTo, entrySpeed: vIn, code: code };
  }

  function classifyShot(params, thetaDeg, v, yawDeg) {
    need();
    var p = normParams(params), bp = ballProps(p.ballId), S0 = shooterSpin(p.shooter);
    var ctx = ctxFor(p, bp.rIn);
    var r = classifyCore(p, ctx, aeroK(bp), S0, thetaDeg, v, yawDeg);
    delete r.code;
    return r;
  }

  // ------------------------------------------------------------------ why nothing scores (SPEC 7.2)
  var CAUSE_PRIORITY = [C_LIP, C_ROOF, C_CELL, C_FRAME, C_SHORT, C_LONG, C_WALL, C_FLOOR];
  function dominantCause(grid) {
    if (grid.dom) return grid.dom;
    var ctx = grid.ctx, t = grid.table, ycs = grid.ycs, nY = ycs.length, nV = t.nV;
    var counts = new Float64Array(9), total = 0, out = { code: 0, k: 0, t: 1 };
    var cx = ctx.cen[0], cy = ctx.cen[1], cz = ctx.cen[2], x0 = ctx.x0, y0 = ctx.y0, h0 = ctx.h0;
    var bestD = Infinity, bestPair = null;
    for (var idx = 0; idx < t.nT * nV; idx++) {
      var P = t.trajs[idx], n = t.lens[idx], ap = t.apex[idx];
      if (P[2 * ap + 1] < ctx.zNearLoMM) continue;
      var kz = cantIndex(ctx, P, n, ap);
      var r0 = firstGE(P, 1, 0, ap + 1, ctx.zNearLoMM), r1 = firstGT(P, 1, 0, ap + 1, ctx.zNearHiMM) - 1;
      var f0 = firstLE(P, 1, ap + 1, n, ctx.zNearHiMM), f1 = firstLT(P, 1, ap + 1, n, ctx.zNearLoMM) - 1;
      if (r1 > kz) r1 = kz; if (f1 > kz) f1 = kz;
      for (var y = 0; y < nY; y++) {
        var yc = ycs[y], c = yc.c * IPM, s = yc.s * IPM, dmin = Infinity;
        for (var run = 0; run < 2; run++) {
          var ka = run ? f0 : r0, kb = run ? f1 : r1;
          for (var k = ka; k <= kb; k++) {
            var dx = x0 + P[2 * k] * c - cx, dy = y0 + P[2 * k] * s - cy, dz = h0 + P[2 * k + 1] * IPM - cz;
            var d2 = dx * dx + dy * dy + dz * dz;
            if (d2 < dmin) dmin = d2;
          }
        }
        if (dmin < 144) {
          var code = runTest(ctx, yc, P, n, ap, t.mono[idx], t.preMax[idx], kz, out);
          counts[code === C_HIT ? C_LONG : code] += 1; total++;
        }
        if (dmin < bestD) { bestD = dmin; bestPair = [idx, y]; }
      }
    }
    if (total === 0) {
      for (var j = 0; j < grid.causes.length; j++) counts[grid.causes[j] || C_LONG] += 1;
    }
    var bc = C_SHORT, bv = -1;
    CAUSE_PRIORITY.forEach(function (cd) { if (counts[cd] > bv) { bv = counts[cd]; bc = cd; } });
    var cnt = {};
    for (var q = 1; q < 9; q++) if (counts[q]) cnt[CAUSE_NAMES[q]] = counts[q];
    grid.dom = { cause: CAUSE_NAMES[bc], code: bc, counts: cnt, near: total, closest: bestPair };
    return grid.dom;
  }

  // ------------------------------------------------------------------ verdicts and reasons (SPEC 7.6)
  var NO_ARC_TEXT = {
    lip: 'Every arc from here clips the bottom lip of the CELL',
    roof: 'Every arc from here hits the rim or roof of the CELL',
    cell: 'Every arc from here runs into another CELL before it reaches the target',
    frame: 'The HIVE frame or arm bar blocks every arc from here',
    short: 'Every arc from here drops below the CELL mouth before it gets there',
    long: 'Every arc from here sails past the CELL mouth',
    wall: 'Every arc from here hits the field wall',
    floor: 'Every arc from here hits the floor first'
  };
  function noArcReason(cause) { return NO_ARC_TEXT[cause] || 'No arc from here gets the ball cleanly into the CELL'; }

  function tooSlowReason(vMin, mcMin) {
    return 'Needs at least ' + fmt(vMin, 1) + ' m/s — ' + fmtInt(mcMin.motorRpm) + ' motor rpm; this motor tops out at ' +
      fmtInt(mcMin.usableFreeRpm) + ' rpm';
  }

  function tightPhrase(v, sel, w, p) {
    var pr = p.precision;
    var mt = (Math.min(sel.theta - w.thetaDeg[0], w.thetaDeg[1] - sel.theta) + v.thStep / 2) / pr.sigThetaDeg;
    var my = (Math.min(sel.yaw - w.yawDeg[0], w.yawDeg[1] - sel.yaw) + v.yawStep / 2) / pr.sigYawDeg;
    var mv = (Math.min(Math.log(sel.v / w.v[0]), Math.log(w.v[1] / sel.v)) + v.lnStep / 2) / Math.max(1e-9, sel.sigV);
    if (mv <= mt && mv <= my) {
      var vHalf = 50 * (Math.log(w.v[1] / w.v[0]) + v.lnStep);
      return { axis: 'v', margin: mv, text: w.v[1] - w.v[0] < 0.02 ?
        'exit speed has to be within about ±' + fmt(Math.max(vHalf, 0.1), 1) + '% of ' + fmt(sel.v, 2) + ' m/s' :
        'only exit speeds from ' + fmt(w.v[0], 2) + ' to ' + fmt(w.v[1], 2) + ' m/s go in' };
    }
    if (mt <= my) {
      var tW = w.thetaDeg[1] - w.thetaDeg[0] + v.thStep;
      return { axis: 'theta', margin: mt, text: w.thetaDeg[1] - w.thetaDeg[0] < 0.5 ?
        'the launch angle has to be within a ' + fmt(tW, 1) + '° window around ' + fmt(sel.theta, 1) + '°' :
        'only launch angles from ' + fmt(w.thetaDeg[0], w.thetaDeg[0] % 1 ? 1 : 0) + '° to ' +
        fmt(w.thetaDeg[1], w.thetaDeg[1] % 1 ? 1 : 0) + '° go in' };
    }
    var width = w.yawDeg[1] - w.yawDeg[0] + v.yawStep;
    return { axis: 'yaw', margin: my, text: 'the left-right aim has only a ' + fmt(width, 1) + '° window' };
  }

  function decide(p, v, sel) {
    var M = S.model, rate = sel.rate, h = sel.mc.headroom;
    var w = windowsOf(v, sel), tight = tightPhrase(v, sel, w, p);
    var rp = Math.floor(rate * 100 + 1e-9), hp = Math.ceil(h * 100 - 1e-9), room = Math.max(0, Math.round((1 - h) * 100));
    var d = { windows: w, tight: tight };
    if (rate >= M.thPossible) {
      if (h > M.downgrade) {
        d.verdict = VERDICT.MID; d.code = 'LOW_HEADROOM';
        d.reason = 'Scores ' + rp + '% of shots, but the motor runs at ' + hp + '% of free speed — no headroom as the battery drains';
      } else {
        d.verdict = VERDICT.OK; d.code = 'OK';
        d.reason = 'Scores ' + rp + '% of shots with ' + room + '% motor headroom';
      }
    } else if (rate >= M.thMid) {
      d.verdict = VERDICT.MID; d.code = 'MISSES_OFTEN';
      d.reason = 'Scores ' + rp + '% of shots — ' + tight.text + (h > M.downgrade ? '; the motor is also at ' + hp + '% of free speed' : '');
    } else {
      d.verdict = VERDICT.BAD; d.code = 'RARELY_SCORES';
      d.reason = 'Only ' + rp + '% of shots score — ' + tight.text;
    }
    return d;
  }

  // ------------------------------------------------------------------ warnings
  var ROBOT_HALF = 9;
  function warningsFor(p, headingDeg) {
    var G = S.G, W = [], x = p.robot.x, y = p.robot.y;
    if (p.clamped) {
      W.push({ code: 'CLAMPED', text: 'Robot moved back inside the field — its centre can be at most ' + fmt(G.robotLimit, 1) + ' in from the middle in x and y' });
    }
    var c = Math.cos(headingDeg * DEG), s = Math.sin(headingDeg * DEG), onLeg = false;
    G.legs.forEach(function (leg) {
      var base = leg[0][2] <= leg[1][2] ? leg[0] : leg[1];
      var dx = base[0] - x, dy = base[1] - y;
      var lx = dx * c + dy * s, ly = -dx * s + dy * c;
      var ex = Math.max(Math.abs(lx) - ROBOT_HALF, 0), ey = Math.max(Math.abs(ly) - ROBOT_HALF, 0);
      if (Math.hypot(ex, ey) < G.tubeR) onLeg = true;
    });
    if (onLeg) W.push({ code: 'ON_FRAME_LEG', text: 'Robot overlaps a HIVE frame leg — it cannot sit here' });
    var under = ['red', 'blue'].some(function (al) { return Math.abs(x - G.hx[al]) <= G.W2 && Math.abs(y) <= G.swingMaxU; });
    if (p.h0 > G.swingWarnZ && under) {
      W.push({ code: 'TALL_UNDER_HIVE', text: 'Shooter exit is above ' + fmt(G.swingWarnZ, 1) + ' in while under the HIVE — a TIP could hit it' });
    }
    return W;
  }

  // ------------------------------------------------------------------ evaluate
  function evaluate(params, mode) {
    need();
    var tAll = now();
    mode = mode === 'full' ? 'full' : 'coarse';
    var p = normParams(params), sh = p.shooter, bp = ballProps(p.ballId), mo = S.motorById[p.motorId];
    var S0 = shooterSpin(sh);
    var t0 = now();
    var table = buildTable(p.ballId, S0);
    var tableMs = now() - t0;
    var t1 = now();
    var grid = getGrid(p, table), ctx = grid.ctx, view = coarseView(grid);
    var pr = p.precision;
    var res = {
      verdict: VERDICT.BAD, code: 'NO_ARC', hitRate: 0, reason: '',
      psi0Deg: ctx.psi0, distIn: ctx.dist,
      best: null, motor: null,
      sigma: { thetaDeg: pr.sigThetaDeg, yawDeg: pr.sigYawDeg, v: null, shooter: pr.sigShooter, motor: null, recovery: null },
      trajectory: null, ghosts: [], missCause: null, warnings: [],
      timings: { tableMs: tableMs, coarseMs: 0, fineMs: 0, totalMs: 0 },
      mode: mode, robot: { x: p.robot.x, y: p.robot.y }, h0: p.h0, S0: S0, gridHits: grid.nHits, closestMiss: null
    };
    var coarseMs, fineMs = 0;
    if (grid.nHits === 0) {
      var dom = dominantCause(grid);
      res.reason = noArcReason(dom.cause);
      res.missCause = dom.cause;
      res.causeCounts = dom.counts;
      coarseMs = now() - t1;
      if (dom.closest) {
        var ci = dom.closest[0], th = table.thetas[Math.floor(ci / table.nV)], vv = table.speeds[ci % table.nV], yw = grid.yaws[dom.closest[1]];
        var cm = classifyCore(p, ctx, table.K, S0, th, vv, yw);
        res.closestMiss = { thetaDeg: th, v: vv, yawDeg: yw, cause: cm.cause, path: cm.path };
      }
    } else {
      var sel = selectAim(view, p, mo, sh, bp, false), uncapped = false, v = view, d = null;
      if (!sel) {
        uncapped = true;
        sel = selectAim(view, p, mo, sh, bp, true);
        var mcMin = motorCore(mo, sh, bp, grid.minHitV);
        res.code = 'TOO_SLOW';
        res.reason = tooSlowReason(grid.minHitV, mcMin);
        res.minSpeed = grid.minHitV;
      }
      coarseMs = now() - t1;
      if (!uncapped && mode === 'full') {
        var tf = now();
        var fine = fineStage(p, table, grid, sel, mo, sh, bp);
        if (fine.sel) { sel = fine.sel; v = fine.view; }
        fineMs = now() - tf;
      }
      if (sel) {
        if (!uncapped) {
          d = decide(p, v, sel);
          res.verdict = d.verdict; res.code = d.code; res.reason = d.reason; res.hitRate = sel.rate;
          res.tightAxis = d.tight.axis;
        }
        var w = d ? d.windows : windowsOf(v, sel);
        var shot = classifyCore(p, ctx, table.K, S0, sel.theta, sel.v, sel.yaw);
        res.best = {
          thetaDeg: sel.theta, yawDeg: sel.yaw, yawOffsetDeg: wrap180(sel.yaw - ctx.psi0),
          v: sel.v, vFtS: sel.v / 0.3048,
          apexIn: shot.apexIn, tToCell: shot.tToCell, entrySpeed: shot.entrySpeed,
          windows: { thetaDeg: w.thetaDeg.slice(), v: w.v.slice(), yawDeg: w.yawDeg.slice() },
          hit: shot.hit, cause: shot.cause, reachable: !uncapped
        };
        res.motor = sel.mc;
        res.sigma.v = sel.sigV; res.sigma.motor = sel.mc.sigmaMotor; res.sigma.recovery = sel.mc.sigmaRecovery;
        res.trajectory = shot.path;
        if (w.v[0] < sel.v) res.ghosts.push(classifyCore(p, ctx, table.K, S0, sel.theta, w.v[0], sel.yaw).path);
        if (w.v[1] > sel.v) res.ghosts.push(classifyCore(p, ctx, table.K, S0, sel.theta, w.v[1], sel.yaw).path);
        res.missCause = uncapped ? null : nearMissCause(v, sel);
      }
    }
    res.warnings = warningsFor(p, res.best ? res.best.yawDeg : ctx.psi0);
    res.timings.coarseMs = coarseMs;
    res.timings.fineMs = fineMs;
    res.timings.totalMs = now() - tAll;
    return res;
  }

  // ------------------------------------------------------------------ motor comparison (SPEC 7.8)
  function compareMotors(params) {
    need();
    var p = normParams(params), sh = p.shooter, bp = ballProps(p.ballId), S0 = shooterSpin(sh);
    var table = buildTable(p.ballId, S0), grid = getGrid(p, table), view = coarseView(grid);
    var dom = grid.nHits === 0 ? dominantCause(grid) : null;
    return S.motors.map(function (mo) {
      var row = { motorId: mo.id, label: mo.label, freeRpm: mo.freeRpm, motorRpm: null, headroom: null, recoveryMs: null,
        hitRate: 0, verdict: VERDICT.BAD, reason: '', code: 'NO_ARC' };
      if (dom) { row.reason = noArcReason(dom.cause); return row; }
      var sel = selectAim(view, p, mo, sh, bp, false);
      if (!sel) {
        var mcMin = motorCore(mo, sh, bp, grid.minHitV);
        row.code = 'TOO_SLOW'; row.reason = tooSlowReason(grid.minHitV, mcMin);
        row.motorRpm = mcMin.motorRpm; row.headroom = mcMin.headroom; row.recoveryMs = mcMin.recoveryMs;
        return row;
      }
      var d = decide(p, view, sel);
      row.motorRpm = sel.mc.motorRpm; row.headroom = sel.mc.headroom; row.recoveryMs = sel.mc.recoveryMs;
      row.hitRate = sel.rate; row.verdict = d.verdict; row.reason = d.reason; row.code = d.code; row.v = sel.v;
      return row;
    });
  }

  // ------------------------------------------------------------------ field scan points (SPEC 7.9, 10)
  function scanPoints(stepIn, limitIn) {
    need();
    var step = num(stepIn, 6);
    if (!(step > 0)) step = 6;
    var lim = num(limitIn, S.G.half - 10.5);
    var n = Math.floor(lim / step + 1e-9), pts = [];
    for (var j = -n; j <= n; j++) for (var i = -n; i <= n; i++) pts.push({ x: i * step, y: j * step });
    return pts;
  }

  // ------------------------------------------------------------------ side view (SPEC 8)
  function sideView(params, result) {
    need();
    var p = normParams(params), G = S.G;
    var ox = p.robot.x, oy = p.robot.y, heading;
    if (result && result.best) heading = result.best.yawDeg;
    else if (result && typeof result.psi0Deg === 'number') heading = result.psi0Deg;
    else {
      var cen = mouthCentroid(p.target, p.hiveState[p.target]);
      heading = Math.atan2(cen[1] - oy, cen[0] - ox) / DEG;
    }
    var ux = Math.cos(heading * DEG), uy = Math.sin(heading * DEG);
    function pr(pt) { return [(pt[0] - ox) * ux + (pt[1] - oy) * uy, pt[2]]; }
    var model = hiveModel(p.hiveState), polys = [], mouth = null;
    model.hives.forEach(function (h) {
      h.cells.forEach(function (cell) {
        var isT = h.alliance === p.target && cell.role === 'up';
        polys.push({ role: isT ? 'target' : 'cell', alliance: h.alliance, cell: cell.role, pts: convexHull(cell.vertices.map(pr)) });
        if (isT) mouth = cell.mouth.map(pr);
      });
    });
    function thick(a, b, R, role, name) {
      var pa = pr(a), pb = pr(b), pts = [];
      for (var k = 0; k < 12; k++) {
        var an = k * Math.PI / 6, dx = R * Math.cos(an), dz = R * Math.sin(an);
        pts.push([pa[0] + dx, pa[1] + dz], [pb[0] + dx, pb[1] + dz]);
      }
      polys.push({ role: role, part: name, pts: convexHull(pts) });
    }
    model.hives.forEach(function (h) {
      for (var k = 0; k + 1 < h.armBar.length; k++) thick(h.armBar[k], h.armBar[k + 1], G.armR, 'arm', 'arm bar');
    });
    G.legs.forEach(function (s) { thick(s[0], s[1], G.tubeR, 'frame', 'leg'); });
    thick(G.crossbar[0], G.crossbar[1], G.tubeR, 'frame', 'crossbar');
    G.corners.forEach(function (s) { thick(s[0], s[1], G.cornerR, 'frame', 'corner block'); });
    var minS = 0, maxS = 0;
    polys.forEach(function (pl) { pl.pts.forEach(function (q) { if (q[0] < minS) minS = q[0]; if (q[0] > maxS) maxS = q[0]; }); });
    var arcs = { trajectory: null, ghosts: [] };
    if (result && result.trajectory) {
      arcs.trajectory = result.trajectory.map(pr);
      arcs.trajectory.forEach(function (q) { if (q[0] > maxS) maxS = q[0]; if (q[0] < minS) minS = q[0]; });
    }
    if (result && result.ghosts) arcs.ghosts = result.ghosts.map(function (g) { return g.map(pr); });
    return {
      uAxis: [ux, uy], origin: [ox, oy], headingDeg: heading,
      polys: polys, mouth: mouth, floor: [minS - 6, maxS + 6],
      arcs: arcs, lipZ: G.lipZ, topZ: G.topZ
    };
  }

  function clearCaches() {
    S.tables = {}; S.tableKeys = []; S.gridCache = []; S.obsCache = {}; _ctxCache.key = null; _pool = {};
  }

  return {
    version: '1.0.0',
    init: init,
    ballProps: ballProps,
    shooterSpin: shooterSpin,
    buildTable: buildTable,
    hiveModel: hiveModel,
    motorModel: motorModel,
    classifyShot: classifyShot,
    evaluate: evaluate,
    compareMotors: compareMotors,
    scanPoints: scanPoints,
    sideView: sideView,
    CAUSES: CAUSE_NAMES.slice(1),
    VERDICTS: [VERDICT.OK, VERDICT.MID, VERDICT.BAD],
    _internal: {
      erf: erf, Phi: Phi, worldToBody: worldToBody, bodyToWorld: bodyToWorld,
      dEdge: dEdge, dPoly: dPoly, insidePentagon: insidePentagon, dInt: dInt,
      prismDist: prismDist, shellDist: shellDist, capsuleDist: capsuleDist,
      flyFloat: flyFloat, terminalSpeed: terminalSpeed, normParams: normParams, shooterNorm: shooterNorm,
      obstaclesFor: obstaclesFor, mouthCentroid: mouthCentroid, penta: penta,
      geometry: function () { return S.G; }, tableSpec: TBL, clearCaches: clearCaches,
      kernel: kernel
    }
  };
});
