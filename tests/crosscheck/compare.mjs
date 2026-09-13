// Cross-check: engine.classifyShot vs the independent Python oracle.
//   node tests/crosscheck/compare.mjs [--shots 150]
// Writes tests/crosscheck/out/{cases,out,boundary}.json. Exit code 1 on any non-boundary disagreement.
import fs from 'node:fs';
import path from 'node:path';
import { execFileSync } from 'node:child_process';
import { ROOT, loadEngine, BASE } from '../load-data.mjs';

const E = loadEngine();
const OUT = path.join(ROOT, 'tests', 'crosscheck', 'out');
fs.mkdirSync(OUT, { recursive: true });
const nShots = Number(process.argv[process.argv.indexOf('--shots') + 1]) || 150;

let seed = 12345;
const rnd = () => ((seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648);

const S = (o) => ({ ...BASE, ...o, shooter: { ...BASE.shooter, ...(o.shooter || {}) } });
const situations = [
  ['red in-plane u30', S({ robot: { x: -12.75, y: -30 } })],
  ['red in-plane u48', S({})],
  ['red in-plane u60', S({ robot: { x: -12.75, y: -60 } })],
  ['blue in-plane u48', S({ target: 'blue', robot: { x: 12.75, y: 48 } })],
  ['oblique (-40,-50)', S({ robot: { x: -40, y: -50 } })],
  ['oblique (40,-50) red', S({ robot: { x: 40, y: -50 } })],
  ['oblique (-40,50) blue', S({ target: 'blue', robot: { x: -40, y: 50 } })],
  ['side (58,-20) red', S({ robot: { x: 58, y: -20 } })],
  ['far side red (-12.75,48)', S({ robot: { x: -12.75, y: 48 } })],
  ['under red pivot', S({ robot: { x: -12.75, y: 0 } })],
  ['between hives (0,-10)', S({ robot: { x: 0, y: -10 } })],
  ['corner (-60,-60)', S({ robot: { x: -60, y: -60 } })],
  ['corner (60,60) blue', S({ target: 'blue', robot: { x: 60, y: 60 } })],
  ['wall (0,-61)', S({ robot: { x: 0, y: -61 } })],
  ['red after TIP (-12.75,48)', S({ hiveState: { red: 1, blue: 1 }, robot: { x: -12.75, y: 48 } })],
  ['blue after TIP (12.75,-40)', S({ target: 'blue', hiveState: { red: -1, blue: -1 }, robot: { x: 12.75, y: -40 } })],
  ['nectar u48', S({ ballId: 'nectar' })],
  ['nectar oblique (-30,-45)', S({ ballId: 'nectar', robot: { x: -30, y: -45 } })],
  ['h0 6 u40', S({ h0: 6, robot: { x: -12.75, y: -40 } })],
  ['h0 28 u26', S({ h0: 28, robot: { x: -12.75, y: -26 } })],
  ['dual k=1 (S0=0) u48', S({ shooter: { type: 'dual', topRatio: 1 } })],
  ['dual k=0.3 oblique', S({ shooter: { type: 'dual', topRatio: 0.3 }, robot: { x: -35, y: -40 } })],
];

const cases = situations.map(([id, p]) => {
  const S0 = E.shooterSpin(p.shooter);
  const r = E.evaluate(p, 'full');
  const psi0 = r.psi0Deg;
  const shots = [];
  const aim = r.best;
  const nDense = aim ? Math.floor(nShots * 0.6) : 0;
  for (let i = 0; i < nDense; i++) {
    shots.push({ thetaDeg: aim.thetaDeg + (rnd() - 0.5) * 12, v: aim.v * Math.exp((rnd() - 0.5) * 0.24), yawDeg: aim.yawDeg + (rnd() - 0.5) * 12 });
  }
  while (shots.length < nShots) {
    shots.push({ thetaDeg: 20 + rnd() * 69, v: 2 + rnd() * 10, yawDeg: psi0 + (rnd() - 0.5) * 50 });
  }
  return { id, params: p, S0, oracle: { id, ballId: p.ballId, target: p.target, hiveState: p.hiveState, robot: p.robot, h0: p.h0, S0, shots } };
});

function runOracle(list, name) {
  const cf = path.join(OUT, name + '-cases.json');
  const of = path.join(OUT, name + '-out.json');
  fs.writeFileSync(cf, JSON.stringify(list));
  execFileSync('python', [path.join(ROOT, 'tests', 'crosscheck', 'oracle.py'), cf, of, '--quiet'], { stdio: 'inherit' });
  return JSON.parse(fs.readFileSync(of, 'utf8'));
}

const t0 = Date.now();
const oracleOut = runOracle(cases.map((c) => c.oracle), 'main');
let total = 0, agree = 0, causeAgree = 0;
const dis = [];
cases.forEach((c, ci) => {
  c.oracle.shots.forEach((s, si) => {
    const e = E.classifyShot(c.params, s.thetaDeg, s.v, s.yawDeg);
    const o = oracleOut[ci].results[si];
    total++;
    if (e.hit === o.hit) { agree++; if (e.cause === o.cause) causeAgree++; }
    else dis.push({ ci, si, s, e: { hit: e.hit, cause: e.cause }, o: { hit: o.hit, cause: o.cause } });
  });
});

// Boundary test: does the oracle itself flip within ±0.25° / ±0.5 %?
let nonBoundary = [];
if (dis.length) {
  const probe = dis.map((d, k) => {
    const c = cases[d.ci].oracle, s = d.s;
    const shots = [[0.25, 1], [-0.25, 1], [0, 1.005], [0, 0.995]].map(([dt, fv]) => ({ thetaDeg: s.thetaDeg + dt, v: s.v * fv, yawDeg: s.yawDeg }));
    return { ...c, id: 'probe' + k, shots };
  });
  const pr = runOracle(probe, 'boundary');
  dis.forEach((d, k) => {
    d.boundary = pr[k].results.some((r) => r.hit !== d.o.hit);
    if (!d.boundary) nonBoundary.push(d);
  });
}

const report = {
  shots: total, agreementPct: +(100 * agree / total).toFixed(3), causeAgreementPct: +(100 * causeAgree / agree).toFixed(2),
  disagreements: dis.length, nonBoundary: nonBoundary.length, seconds: (Date.now() - t0) / 1000,
  nonBoundaryDetail: nonBoundary.slice(0, 20).map((d) => ({ situation: cases[d.ci].id, shot: d.s, engine: d.e, oracle: d.o })),
};
fs.writeFileSync(path.join(OUT, 'report.json'), JSON.stringify(report, null, 1));
console.log(JSON.stringify(report, null, 1));
process.exit(nonBoundary.length ? 1 : 0);
