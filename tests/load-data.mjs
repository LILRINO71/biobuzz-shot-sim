// Loads data/*.json and an initialised ShotEngine for Node scripts and tests.
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';

export const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const require = createRequire(import.meta.url);

export function loadData() {
  const rd = (f) => JSON.parse(fs.readFileSync(path.join(ROOT, 'data', f), 'utf8'));
  return { motors: rd('motors.json'), field: rd('field.json'), shooter: rd('shooter.json') };
}

export function loadEngine() {
  const E = require(path.join(ROOT, 'src', 'engine.js'));
  E.init(loadData());
  return E;
}

export const BASE = {
  ballId: 'pollen', target: 'red', hiveState: { red: -1, blue: 1 }, robot: { x: -12.75, y: -48 }, h0: 16,
  precision: { sigThetaDeg: 1, sigYawDeg: 1, sigShooter: 0.015 }, motorId: 'yj6000',
  shooter: { type: 'single', wheelDiameterMm: 96, motorsPerWheel: 1, gear: 1, topRatio: 0.6, inertiaKgM2: 4e-4, shotInterval: 0.5 },
};
