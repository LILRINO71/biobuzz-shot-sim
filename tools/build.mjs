// Builds dist/biobuzz-shot-sim.html (publishable fragment) and dist/preview.html (standalone page).
//   node tools/build.mjs
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const rd = (...p) => fs.readFileSync(path.join(ROOT, ...p), 'utf8');
const json = (f) => JSON.parse(rd('data', f));

const data = { motors: json('motors.json'), field: json('field.json'), shooter: json('shooter.json') };
const dataJs = `window.SHOT_DATA = ${JSON.stringify(data)};\n`;
fs.writeFileSync(path.join(ROOT, 'src', 'data.js'), dataJs, 'utf8');

const engine = rd('src', 'engine.js');
const worker = rd('src', 'worker.js');
const app = rd('src', 'app.js');
const css = rd('src', 'styles.css');
const markup = rd('src', 'markup.html');

// Inline scripts must never contain a closing script tag.
const safe = (s) => s.replace(/<\/(script)/gi, '<\/$1');
const workerSrc = `window.SHOT_WORKER_SRC = ${JSON.stringify(engine + '\n' + worker)};\n`;

const fragment = [
  '<title>BIOBUZZ Shot Sim</title>',
  '<meta name="description" content="Drag a robot anywhere on the FTC BIOBUZZ field, pick a goBILDA motor and flywheel, and see whether the shot into the HIVE is possible.">',
  '<link rel="preconnect" href="https://fonts.googleapis.com">',
  '<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>',
  '<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Barlow+Semi+Condensed:wght@500;600;700&family=Barlow:wght@400;500;600&family=JetBrains+Mono:wght@400;500;600&display=swap">',
  `<style>\n${css}\n</style>`,
  markup,
  '<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>',
  `<script>\n${safe(dataJs)}</script>`,
  `<script>\n${safe(engine)}\n</script>`,
  `<script>\n${safe(workerSrc)}</script>`,
  `<script>\n${safe(app)}\n</script>`,
].join('\n');

fs.mkdirSync(path.join(ROOT, 'dist'), { recursive: true });
fs.writeFileSync(path.join(ROOT, 'dist', 'biobuzz-shot-sim.html'), fragment, 'utf8');
const preview = `<!doctype html>\n<html lang="en">\n<head>\n<meta charset="utf-8">\n<meta name="viewport" content="width=device-width, initial-scale=1">\n</head>\n<body>\n${fragment}\n</body>\n</html>\n`;
fs.writeFileSync(path.join(ROOT, 'dist', 'preview.html'), preview, 'utf8');
console.log(`built dist/biobuzz-shot-sim.html (${(fragment.length / 1024).toFixed(0)} KB) and dist/preview.html`);
