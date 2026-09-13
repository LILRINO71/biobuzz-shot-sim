import test from 'node:test';
import assert from 'node:assert/strict';
import { loadEngine, loadData, BASE } from './load-data.mjs';

const E = loadEngine();
const I = E._internal;
const D = loadData();
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: ${a} vs ${b} (±${tol})`);
const S = (o) => ({ ...BASE, ...o, shooter: { ...BASE.shooter, ...(o.shooter || {}) } });

test('CELL mouth maps to the published lip / top heights', () => {
  const G = I.geometry(), c = D.field.hive.cell;
  for (const sigma of [-1, 1]) {
    const lip = I.bodyToWorld(0, c.aOut, c.b0, -12.75, sigma);
    const top = I.bodyToWorld(0, c.aOut, c.b0 + c.height, -12.75, sigma);
    near(lip[2], 53.5, 0.02, 'lip z'); near(Math.abs(lip[1]), 19.27, 0.02, 'lip u');
    near(top[2], 65.62, 0.02, 'top z'); near(Math.abs(top[1]), 12.27, 0.02, 'top u');
    assert.equal(Math.sign(lip[1]), sigma);
  }
  const m = E.hiveModel({ red: -1, blue: 1 }).mouthCentroid.red;
  near(m[2], 58.31, 0.03, 'centroid z'); near(-m[1], 16.49, 0.03, 'centroid u');
  assert.ok(G);
});

test('world <-> body round trip', () => {
  for (let k = 0; k < 200; k++) {
    const x = Math.sin(k) * 60, y = Math.cos(k * 1.3) * 60, z = 5 + (k % 70), hx = k % 2 ? 12.75 : -12.75, s = k % 3 ? 1 : -1;
    const b = I.worldToBody(x, y, z, hx, s);
    const w = I.bodyToWorld(b[0], b[1], b[2], hx, s);
    near(w[0], x, 1e-9, 'x'); near(w[1], y, 1e-9, 'y'); near(w[2], z, 1e-9, 'z');
  }
});

test('pentagon distances', () => {
  const b0 = D.field.hive.cell.b0;
  assert.equal(I.dPoly(0, b0 + 5), 0);
  near(I.dEdge(0, b0 + 5), 5, 1e-9, 'inside to base');
  near(I.dPoly(0, b0 - 1), 1, 1e-9, 'below base');
  near(I.dPoly(15, b0 + 3), 5, 1e-9, 'beside wall');
  near(I.dPoly(0, b0 + 16), 2, 1e-9, 'above apex');
  near(I.dPoly(-12, b0 - 3), Math.hypot(2, 3), 1e-9, 'corner');
});

test('prism distance matches brute-force sampling of the solid', () => {
  const c = D.field.hive.cell;
  let worst = 0;
  for (let k = 0; k < 60; k++) {
    const w = -16 + (k * 7.3) % 32, a = 4 + (k * 5.1) % 24, b = c.b0 - 5 + (k * 3.7) % 24;
    const analytic = I.prismDist(w, a, b, c.aIn, c.aOut);
    let brute = Infinity;
    for (let aa = c.aIn; aa <= c.aOut + 1e-9; aa += 0.25)
      for (let ww = -10; ww <= 10 + 1e-9; ww += 0.25)
        for (let bb = c.b0; bb <= c.b0 + c.height + 1e-9; bb += 0.25)
          if (I.insidePentagon(ww, bb)) brute = Math.min(brute, Math.hypot(w - ww, a - aa, b - bb));
    worst = Math.max(worst, Math.abs(brute - analytic));
  }
  assert.ok(worst < 0.2, `worst diff ${worst}`);
});

test('no-air flight is a parabola; pollen terminal speed is ~15.1 m/s', () => {
  const tr = I.flyFloat('pollen', 0, 60, 8, { cd: 0, clSlope: 0, tEnd: 1.0 });
  const [rho, z] = tr.at(-1);
  near(rho, 8 * Math.cos(Math.PI / 3), 0.001, 'rho at 1 s');
  near(z, 8 * Math.sin(Math.PI / 3) - 0.5 * 9.81, 0.001, 'z at 1 s');
  near(I.terminalSpeed('pollen'), 15.1, 0.1, 'terminal speed');
});

test('in-plane windows match the 2-D reference model (SPEC §9)', () => {
  const p = S({});
  const rowV = [], colT = [];
  for (let v = 4.4; v <= 5.7; v += 0.01) if (E.classifyShot(p, 70.5, v, 90).hit) rowV.push(v);
  for (let t = 60; t <= 80; t += 0.1) if (E.classifyShot(p, t, 5.05, 90).hit) colT.push(t);
  near(rowV[0], 4.80, 0.05, 'u48 row low'); near(rowV.at(-1), 5.31, 0.05, 'u48 row high');
  near(colT[0], 65.5, 0.6, 'u48 col low'); near(colT.at(-1), 75.5, 0.6, 'u48 col high');
  const p30 = S({ robot: { x: -12.75, y: -30 } });
  const row30 = [];
  for (let v = 4.2; v <= 5.6; v += 0.01) if (E.classifyShot(p30, 81.5, v, 90).hit) row30.push(v);
  near(row30[0], 4.57, 0.1, 'u30 row low'); near(row30.at(-1), 5.20, 0.1, 'u30 row high');
});

test('motor model reference numbers', () => {
  const m = E.motorModel('yj6000', BASE.shooter, 'pollen', 5.05);
  near(m.wheelRpm, 2233, 2, 'wheel rpm');
  near(m.headroom, 0.384, 0.003, 'headroom');
  near(m.dip, 0.049, 0.002, 'dip');
  near(m.recoveryMs, 50, 5, 'recovery ms');
  assert.equal(m.reachable, true);
  assert.equal(E.motorModel('yj30', BASE.shooter, 'pollen', 5.05).reachable, false);
});

test('verdicts: under the pivot, good in-plane spot, too-slow motor', () => {
  const under = E.evaluate(S({ robot: { x: -12.75, y: 0 } }), 'full');
  assert.equal(under.verdict, "WON'T WORK");
  const good = E.evaluate(S({}), 'full');
  assert.equal(good.verdict, 'POSSIBLE');
  assert.ok(good.hitRate >= 0.8);
  const slow = E.evaluate(S({ motorId: 'yj30' }), 'full');
  assert.equal(slow.verdict, "WON'T WORK");
  assert.match(slow.reason, /tops out/);
});

test('only the three verdict labels are ever produced', () => {
  const allowed = new Set(['POSSIBLE', 'NOT CONSISTENT', "WON'T WORK"]);
  for (const q of E.scanPoints(24)) {
    const r = E.evaluate(S({ robot: q }), 'coarse');
    assert.ok(allowed.has(r.verdict), r.verdict);
    assert.ok(Number.isFinite(r.hitRate));
  }
});

test('mirror symmetry: red σ=-1 at (x,y) equals blue σ=+1 at (-x,-y)', () => {
  for (const [x, y] of [[-12.75, -48], [-40, -50], [-58, -10], [-5, -35]]) {
    const a = E.evaluate(S({ robot: { x, y } }), 'full');
    const b = E.evaluate(S({ target: 'blue', robot: { x: -x, y: -y } }), 'full');
    assert.equal(a.verdict, b.verdict, `${x},${y}`);
    near(a.hitRate, b.hitRate, 0.02, `hit rate ${x},${y}`);
  }
});

test('battery voltage sets the top speed; PID does not raise it', () => {
  const lo = E.motorModel('yj6000', { ...BASE.shooter, batteryV: 12 }, 'pollen', 5.05);
  const hi = E.motorModel('yj6000', { ...BASE.shooter, batteryV: 13.2 }, 'pollen', 5.05);
  near(hi.usableFreeRpm / lo.usableFreeRpm, 13.2 / 12, 1e-9, 'free speed scales with voltage');
  near(hi.vCap / lo.vCap, 13.2 / 12, 1e-9, 'speed cap scales with voltage');
  const pid = E.motorModel('yj6000', { ...BASE.shooter, control: 'pid' }, 'pollen', 5.05);
  const pow = E.motorModel('yj6000', { ...BASE.shooter, control: 'power' }, 'pollen', 5.05);
  assert.equal(pid.usableFreeRpm, pow.usableFreeRpm, 'PID and fixed power share the same ceiling');
});

test('fixed power recovers slower and scatters more than PID', () => {
  const sh = { ...BASE.shooter, shotInterval: 0.3 };
  const pid = E.motorModel('yj6000', { ...sh, control: 'pid' }, 'pollen', 5.05);
  const pow = E.motorModel('yj6000', { ...sh, control: 'power' }, 'pollen', 5.05);
  assert.ok(pow.recoveryMs > 10 * pid.recoveryMs, `recovery ${pow.recoveryMs} vs ${pid.recoveryMs}`);
  assert.ok(pow.residual > pid.residual);
  assert.ok(pow.sigmaMotor > pid.sigmaMotor);
  const p = S({ robot: { x: -12.75, y: -30 }, shooter: { shotInterval: 0.3 } });
  const rPid = E.evaluate({ ...p, shooter: { ...p.shooter, control: 'pid' } }, 'full');
  const rPow = E.evaluate({ ...p, shooter: { ...p.shooter, control: 'power' } }, 'full');
  assert.ok(rPow.hitRate < rPid.hitRate, `hit ${rPow.hitRate} vs ${rPid.hitRate}`);
});

test('heavier flywheel dips less; bigger wheel needs fewer rpm', () => {
  const light = E.motorModel('yj6000', { ...BASE.shooter, inertiaKgM2: 2e-4 }, 'pollen', 5.05);
  const heavy = E.motorModel('yj6000', { ...BASE.shooter, inertiaKgM2: 8e-4 }, 'pollen', 5.05);
  assert.ok(heavy.dip < light.dip && heavy.spinUpMs > light.spinUpMs);
  const w72 = E.motorModel('yj6000', { ...BASE.shooter, wheelDiameterMm: 72 }, 'pollen', 5.05);
  near(w72.wheelRpm / E.motorModel('yj6000', BASE.shooter, 'pollen', 5.05).wheelRpm, 96 / 72, 1e-9, 'rpm scales with 1/D');
});
