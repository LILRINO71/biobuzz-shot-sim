# BIOBUZZ Shot Sim — build spec

An in-browser simulator for FTC 2026-27 BIOBUZZ. The user drags a robot **anywhere on the
field**, picks a **goBILDA 8 mm REX (hex) shaft motor** and a flywheel setup, and the sim
finds the arc needed to put POLLEN or NECTAR into their alliance's upward HIVE CELL, then
gives exactly one of three verdicts:

| Verdict | Meaning |
|---|---|
| **POSSIBLE** | makes the shot reliably (hit rate ≥ 80 %) and the motor has headroom |
| **NOT CONSISTENT** | the shot exists but misses often (40–80 %), or the motor is near its limit |
| **WON'T WORK** | no clear arc, motor can't reach the speed, or hit rate < 40 % |

Only those three labels, ever. Everything else (hit %, reasons) is supporting detail.

Paths used below:
- Project root: `C:/Users/hiheo/Claude/biobuzz-shot-sim`
- Reference scratch (read-only, never committed): `SCR = C:/Users/hiheo/AppData/Local/Temp/claude/C--Users-hiheo-Claude/0efbfc1b-5386-4b88-ba3d-a83b0a03bd21/scratchpad`
  - `SCR/manual.txt` — full text of Competition Manual V1 (page-delimited `===== PAGE n =====`)
  - `SCR/img/*.png|jpg` — figures extracted from the manual (p69–p74 HIVE/CELL/FLOWER, p83 staging)
  - `SCR/pdfimg.py`, `SCR/pdftext.py` — pure-Python PDF text/image extractors (manual at `SCR/manual.pdf`)
  - `SCR/pollen.step`, `SCR/nectar.step` — AndyMark CAD
  - `SCR/shooter_out.txt` — output of the earlier 2-D in-plane Python model
- Earlier 2-D reference model: `C:/Users/hiheo/Claude/ftc-base/shooter/shooter_math.py`

Nothing in this project may be committed with FIRST manual text/images or vendor CAD files.

---------------------------------------------------------------------------------------
## 1. Files

```
data/motors.json       goBILDA 5203 Yellow Jacket lineup (research output)
data/field.json        field element layout + HIVE geometry constants (research output)
data/shooter.json      flywheel wheel presets, inertia presets, model constants (research output)
src/engine.js          physics + geometry + search + verdict + motor model (no DOM).
                       UMD: window.ShotEngine / self.ShotEngine (worker) / module.exports (Node)
src/worker.js          worker wrapper around the engine (protocol in §7)
src/app.js             UI
src/styles.css         UI styles
src/markup.html        page markup (tools/build.mjs wraps it with CSS, data and scripts)
tools/build.mjs        writes src/data.js from data/*.json and inlines data + css + js (+ engine and
                       worker source as a string for a Blob worker) into dist/biobuzz-shot-sim.html
tests/engine.test.mjs  node --test suite (tests/load-data.mjs loads data + engine for Node)
tests/crosscheck/      independent Python oracle + comparison harness
dist/biobuzz-shot-sim.html   single self-contained page (this is what gets published)
README.md
```

No npm dependencies. Node 24 and Python 3.14 (numpy, scipy, matplotlib) are available.
The built page may load only: Google Fonts CSS, and scripts from
`https://cdnjs.cloudflare.com/ajax/libs/...` (three.js r128 UMD is allowed:
`https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js`). No fetch/XHR at runtime;
all data is inlined. The page must still work (minus the 3-D view) if that script fails to load.

---------------------------------------------------------------------------------------
## 2. Units and coordinate frame

- Geometry in **inches**, physics in SI internally; UI shows in, m/s and ft/s, rpm, degrees.
- Field frame: origin at field center on the tile surface. **+x** toward the BLUE alliance wall
  (right, seen from the audience). **+y** away from the audience. **+z** up.
  Field interior is |x| ≤ 72, |y| ≤ 72. Tiles are a 6×6 grid of 24 in squares.
- Headings ψ are measured in the x-y plane from +x, counter-clockwise, degrees.

---------------------------------------------------------------------------------------
## 3. Field geometry (from Manual V1; research refines `data/field.json`)

### 3.1 HIVE structure (center of field)
- Two HIVEs share one frame. Pivot axis is parallel to **x**, at y = 0, z = **43.95**.
- RED HIVE pivot center x = **−12.75**, BLUE HIVE x = **+12.75** (HIVE center-to-center 25.5 in).
- Each HIVE is a rigid bistable body tilted **30°**. Its two CELLs swing in the y-z plane.
- **upSide** σ ∈ {−1, +1}: the sign of y on which that HIVE's upward CELL sits.
  Match start (Fig 10-2): **red σ = −1** (audience side), **blue σ = +1**. Each TIP flips σ.
- HIVE body coordinates for a HIVE with center x = hx and upSide σ:
  - w = x − hx  (across the CELL width)
  - u = σ·y     (horizontal distance from the pivot line toward the up side)
  - a =  u·cos30 + (z − 43.95)·sin30   (along the arm, toward the up CELL)
  - b = −u·sin30 + (z − 43.95)·cos30   (perpendicular to the arm, "up" in the body)
  - inverse: u = a·cos30 − b·sin30, z = 43.95 + a·sin30 + b·cos30
- CELL = right pentagonal prism, axis along the arm.
  - Up CELL: a ∈ [A_IN, A_OUT] = [**9.42, 21.46**] (18.84/2 and 18.84/2 + 12.04). Open end at a = A_OUT.
  - Down CELL: a ∈ [−21.46, −9.42]. Open end at a = −21.46 (treated as a solid obstacle).
  - Cross-section in (w, b), identical for both cells: b0 = **−1.3625** (base below the arm line),
    vertices (−10, b0), (10, b0), (10, b0+7.61), (0, b0+14), (−10, b0+7.61).
    (20 in wide, 14 in tall, 7.61 in to the shoulders.)
  - Check (must hold, unit-tested): the up-CELL mouth lip (a = 21.46, b = b0) maps to z = 53.50,
    u = 19.27; the mouth apex (a = 21.46, b = b0+14) maps to z = 65.62, u = 12.27.
    Published: lip 53.5 in, top 65.6 in.
  - Mouth outward normal (world) = σ·ŷ·cos30 + ẑ·sin30. A ball scores by entering the open end
    moving inward (toward the pivot and/or down).
  - Mouth centroid (aim reference): (w, b) = (0, b0 + 5.5597) at a = A_OUT → u ≈ 16.49, z ≈ 58.31.
  - Manual Fig 9-10 labels "bottom of HIVE 25.5 in" but its own drawing and the lip/top numbers
    give ≈ 32 in for the down CELL's lowest corner. Use the self-consistent geometry for ball
    collisions; use 25.5 in (conservative) only for the "robot under the HIVE" warning.
- HIVE connecting assembly (arm bar) per HIVE: capsule polyline in body coords at w = 0, radius 1.0:
  (a, b) = (−21.46, −2.4) → (−5, −2.4) → (0, 0) → (5, −2.4) → (21.46, −2.4).
- Frame (A-frame "sawhorse", ~1.5 in tube → capsule radius 0.75):
  - 4 legs: (±24.73, ±19.475, 0) → (±13.25, 0, 43.95) (top x has the same sign as the base x)
  - crossbar: (−13.25, 0, 43.95) → (13.25, 0, 43.95)
  - base footprint 49.46 (x) × 38.95 (y). Under-tile strips are not obstacles.
  - BIOBUZZ logo panels: ignore unless research gives trustworthy geometry.

### 3.2 Perimeter and other elements (drawing + robot placement)
- Walls at |x| = 72, |y| = 72, height **12.25** (ball collision if z < 12.25 + r beyond 72 − r).
- FLOWERs on the walls (approx centers): (−24, +72), (+72, +24), (+24, −72), (−72, −24);
  top opening Ø4 in at 21.5 in; draw with an ~8 in footprint.
- LOADING ZONE (23 × 11 in, against the wall): red x ∈ [−72, −61], y ∈ [24, 47];
  blue x ∈ [61, 72], y ∈ [−47, −24].
- GARDEN (23 × 2 in strip): red y ∈ [−72, −70], x ∈ [−72, −49]; blue y ∈ [70, 72], x ∈ [49, 72].
- ALLIANCE AREA (outside the field, 97 wide × 54 deep): red x < −72, blue x > 72, y ∈ [−48.5, 48.5].
- Tile seams every 24 in.

---------------------------------------------------------------------------------------
## 4. Ball flight

- Balls (hollow polyethylene, 26 holes):
  - POLLEN: D = 2.80 in, m = 0.055 lb (24.948 g), wall 0.070 in
  - NECTAR: D = 91.948 mm (3.620 in), m = 0.091 lb (41.277 g), wall 2.159 mm
  - I = 0.4·m·(R⁵ − Ri⁵)/(R³ − Ri³)
- Air ρ = 1.20 kg/m³, g = 9.81, C_D = 0.45, C_L(S) = 0.20·min(S, 1) for backspin.
- Spin: launch spin ratio S0 (from the shooter config, §6). ω = S0·v0/r held constant in flight;
  S = ω·r/|v| each step.
- 2-D flight in the vertical plane of the launch heading (drag, lift and gravity all lie in that
  plane, so the path never leaves it). State (ρ, z, vρ, vz), relative to the exit point:
  - a = −kd·|v|·v + kl·|v|·(−vz, vρ) − g·ẑ,  kd = ½ρC_D·A/m,  kl = ½ρC_L·A/m
  - RK4, dt = 2.5 ms.
- 3-D position: P(t) = (x0 + ρ·cos ψ, y0 + ρ·sin ψ, h0 + z) with shooter exit (x0, y0, h0).
  The shooter exit is at the robot center; h0 is a user setting (4–29 in; 29 in = R105 height limit).
- Trajectory table (per ball + S0): pitch θ = 10°…89° step 1° (80), speed v = 2.0 × 1.02^j m/s up to
  16 m/s (≈106). Store positions as Int16 millimetres every 5 ms. Truncate when descending and
  z < 0.55 m (no h0 ≤ 29 in can still reach the mouth), or t > 2.6 s, or ρ > 5.6 m.
  Rebuild only when ball or S0 changes. Must take < 1.5 s on a desktop.

---------------------------------------------------------------------------------------
## 5. Hit test (per trajectory, per yaw)

Sphere radius r = D/2 (inches). A shot **scores** only if the ball passes cleanly into the target
up-CELL: the centre crosses the open end moving inward with (w, b) inside the pentagon, and reaches
f < −r (f = a − A_OUT, outward positive) without the sphere ever touching any obstacle.
Any contact anywhere = miss (no bounce-ins).

Distances (inches), collision when distance < r:
- **Solid prism** (down CELL of the target HIVE, both CELLs of the other HIVE):
  d = hypot(dPoly(w, b), dInt(a, [amin, amax])); dPoly = distance to the pentagon region (0 inside),
  dInt = distance to the interval (0 inside). Exact for a right prism.
- **Target up-CELL shell** (open at a = A_OUT):
  - f > 0: d = hypot(dEdge(w, b), f)   (dEdge = distance to the pentagon boundary polyline)
  - −12.04 ≤ f ≤ 0: if (w, b) inside the pentagon, d = dEdge(w, b); else d = dPoly(w, b)
  - f < −12.04: d = hypot(dPoly(w, b), f + 12.04)
- **Capsules**: distance to segment − radius (arm bars, legs, crossbar).
- **Floor**: z < r → miss. **Walls**: beyond 72 − r in x or y with z < 12.25 + r → miss.
  Beyond ±78 → miss (left the field).
- **Can't score any more**: descending and z < 53.50 − r and not inside the target → miss.
- Sub-sample between stored samples so the ball moves ≤ 0.4·r per check, but only while the segment's
  bounding box touches the HIVE bounding box (|x| ≤ 27, |y| ≤ 27, z ≤ 68, expanded by r).
  Outside that box only the floor / wall / can't-score checks run.
- Miss cause (for explanations): `short` (fell below the mouth before reaching it along the heading),
  `long` (passed beyond it), `lip` (target shell contact near the base edge, b < b0 + 1.5),
  `roof` (target shell contact elsewhere: rim, roof, sides), `cell` (another CELL),
  `frame` (legs, crossbar, arm bar), `wall`, `floor`.

---------------------------------------------------------------------------------------
## 6. Motor + flywheel model

`data/motors.json`: every goBILDA 5203 Series Yellow Jacket planetary gear motor with the 8 mm REX
shaft (all ratios; 24 mm / 48 mm shaft lengths share specs): `{id, sku, ratio, freeRpm,
stallTorqueKgCm, stallTorqueNm, stallCurrentA, freeCurrentA, encoderPPR, voltage, url}`.

Shooter config (UI):
- `type`: `single` (one wheel + fixed hood) or `dual` (bottom + top wheels, ball between them).
- `wheelDiameterMm` (presets from shooter.json + custom), `motorsPerWheel` (1 or 2),
  `gear` G = wheel rpm ÷ motor rpm (1 = direct drive; 1.5 = geared up …).
- `dual` only: `topRatio` k = top wheel surface speed ÷ bottom (0…1). The bottom wheel is the fast one.
- `inertiaPreset` → flywheel inertia per wheel shaft, kg·m² (shooter.json).
- `shotInterval` Δt seconds between consecutive shots (0.2–2.0, default 0.5).

Exit speed and spin:
- single: v_exit = η_single·v_wheel (η_single = 0.45 default; ideal rolling on a hood = 0.5), S0 = 1.
- dual: v_exit = η_dual·(v_bottom + v_top)/2 (η_dual = 0.90), v_top = k·v_bottom, S0 = (1 − k)/(1 + k).
- Required wheel rpm n_w = v_wheel/(π·D)·60 (the bottom wheel for dual). Motor rpm n_m = n_w / G.
- **Headroom** h = n_m / freeRpm. h > 1.00 → unreachable.

Speed consistency (fractions, combined in quadrature):
- σ_shooter from the precision preset (§8).
- σ_motor(h): h ≤ 0.80 → 0.005; 0.80→0.95 linear 0.005→0.030; 0.95→1.00 linear 0.030→0.060.
- Flywheel dip and recovery (wheel shaft driven by n = motorsPerWheel motors through G):
  - ω0 = n_w·2π/60, ω_wf = freeRpm·G·2π/60, T_ws = n·stallTorqueNm/G, τ = I·ω_wf/T_ws
  - E = lossFactor(2.0)·(½mv² + ½I_ball·(S0·v/r)²)·share (share = 1 single; v_b/(v_b+v_t) dual)
  - ω1 = sqrt(max(0, ω0² − 2E/I)), dip d = 1 − ω1/ω0
  - recovery t_rec = τ·ln((ω_wf − ω1)/(ω_wf − ω0))  (∞ if ω0 ≥ ω_wf)
  - after Δt: ω(Δt) = ω_wf − (ω_wf − ω1)·e^(−Δt/τ); residual d_res = max(0, 1 − ω(Δt)/ω0)
  - σ_recovery = d_res/2
  - spin-up from rest: t_spin = τ·ln(ω_wf/(ω_wf − ω0))
- σ_v = sqrt(σ_shooter² + σ_motor² + σ_recovery²). σ_v depends on the aim speed (through h).

---------------------------------------------------------------------------------------
## 7. Search, hit rate and verdict

Scatter model: independent Gaussians on pitch (σθ), yaw (σψ) and ln(speed) (σ_v).

1. **Coarse**: ψ0 = heading from the robot (x0, y0) to the mouth centroid.
   Yaws ψ0 + k·Δψ, k = −8…8, Δψ = max(0.5°, atan2(14, dist)/8) (dist = horizontal distance to the
   centroid, inches). Run every table trajectory at every yaw → boolean hit grid H[θ, v, ψ]
   plus the miss cause per cell.
2. If H is empty → **WON'T WORK**, reason from the most common miss cause among trajectories that
   came within 12 in of the mouth centroid (fallback: all).
3. Speed cap: v_cap = exit speed at h = 1.00 for the chosen motor setup. Cells with v > v_cap are
   misses. If nothing remains → **WON'T WORK** ("needs ≥ X m/s = N motor rpm; this motor tops out
   at M rpm").
4. Hit rate for an aim point (θa, va, ψa) = Σ over hit cells of the Gaussian probability mass of
   that cell (per-axis erf differences; each cell spans half a step either side; mass outside the
   grid counts as a miss). σ_v evaluated at va.
5. **Fine** (full mode only): around the coarse best aim point, a local grid θ ± max(3°, 3σθ) step
   0.25°, ψ ± max(3°, 3σψ) step 0.25°, ln v ± max(0.06, 3σ_v) step 0.005, integrated on the fly with
   the same integrator. Best aim = argmax hit rate on the fine grid (speed cap applied).
6. Verdict (only three):
   - hit rate ≥ 0.80 → POSSIBLE; 0.40–0.80 → NOT CONSISTENT; < 0.40 → WON'T WORK
   - if the best aim needs h > 0.92, POSSIBLE is downgraded to NOT CONSISTENT
     ("motor at X % of free speed — no headroom as the battery drains")
   - the reason string always says why in plain words.
7. The result also reports: best θ, ψ (absolute heading and offset from ψ0), v (m/s, ft/s), hit rate,
   angle / speed / yaw windows (contiguous scoring range through the aim point along each axis),
   apex height, time to CELL, entry speed, wheel rpm, motor rpm, h, dip %, recovery ms, spin-up ms,
   σ breakdown, the 3-D trajectory polyline, and ghost trajectories at the speed-window edges.
8. Motor comparison: for the current position and settings, coarse hit rate + verdict for every
   motor in motors.json (same gearing and wheel), so the UI shows all goBILDA ratios side by side.
9. Field scan (heat map): coarse verdict at robot centres on a 6 in grid, |x|, |y| ≤ 63, progressive.

Performance budget (desktop, single thread): table build < 1.5 s, coarse evaluate < 120 ms,
full evaluate < 400 ms, full field scan < 45 s (progressive; use 1–4 workers when available).

Worker protocol (`src/worker.js`): messages `{type:'init', data}`, `{type:'table', ball, S0}`,
`{type:'evaluate', id, params, mode:'coarse'|'full'}`, `{type:'motors', id, params}`,
`{type:'scan', id, params}` → replies `{type:'result'|'motors'|'scanCell'|'scanDone'|'error', id, ...}`.
The app must fall back to main-thread execution (chunked with setTimeout) if a Blob worker can't
be created or doesn't answer within 2 s (the published page runs under a strict CSP).

---------------------------------------------------------------------------------------
## 8. UI

Precision presets (σθ, σψ, σ_shooter): **Dialed-in** (0.5°, 0.5°, 1.0 %), **Typical** (1.0°, 1.0°,
1.5 %, default), **Rough** (2.0°, 2.0°, 3.0 %).

Controls: ball (Pollen / Nectar) · target alliance (Red / Blue) · up-CELL side for that HIVE
(audience side / far side) + a "TIP" button that flips it · motor (all goBILDA 5203 ratios, labels like
"6000 rpm · 1:1") · motors per wheel · shooter type · wheel diameter · gearing G · top-wheel
ratio (dual only) · flywheel inertia preset · shooter exit height · robot precision · time between
shots · robot x / y number inputs.

Views:
- **Field (top-down)** — the main surface, square, drawn to scale: tiles, perimeter, alliance areas,
  loading zones, gardens, flowers, HIVE frame footprint, both HIVEs' CELL footprints (target up-CELL
  highlighted, mouth edge marked), the 18 × 18 in robot (rotated to face the shot), aim line and the
  trajectory's ground track. Drag the robot anywhere (pointer + touch); arrow keys nudge 1 in
  (Shift = 6 in) when the robot is focused. While dragging run coarse; on release run full.
  Toggle: verdict heat map overlay (field scan) with a progress indicator.
- **Side view** — orthographic projection onto the shot's vertical plane: floor, the recommended
  arc, ghost arcs at the speed-window edges, projected outlines of the HIVE parts (convex hulls of
  projected prism vertices; target CELL emphasised, mouth rim highlighted), apex label. To scale.
- **3-D view** (three.js r128, simple custom orbit: drag rotates, wheel/pinch zooms): tiles, walls,
  HIVE frame, CELLs (translucent), robot box, trajectory line, ball at the mouth. Hidden if THREE
  is unavailable.
- **Verdict panel** — the one bold element: POSSIBLE / NOT CONSISTENT / WON'T WORK with an icon
  (check / exclamation / cross) + hit-rate % + one-sentence reason. aria-live polite.
- **Required arc readout** — launch angle, exit speed (m/s + ft/s), heading, apex, time to CELL,
  scoring windows.
- **Motor readout** — wheel rpm, motor rpm, % of free speed (meter), dip per shot, recovery ms,
  spin-up time, σ breakdown.
- **All goBILDA motors table** — one row per ratio: free rpm, motor rpm needed, % of free speed,
  recovery ms, hit %, verdict chip. Selected motor highlighted; clicking a row selects it.
- Warnings as chips: robot overlaps a HIVE frame leg; shooter taller than the 25.5 in HIVE swing
  clearance while under the HIVE; robot outside the field (clamped).
- A short "How this works / assumptions" disclosure.

Design (follow exactly):
- Subject world = FTC field: grey foam tiles, alliance red/blue tape, pollen yellow. The user's
  existing FTC tool uses Barlow Semi Condensed + Barlow + JetBrains Mono — use the same families
  (Google Fonts, with fallbacks `'Arial Narrow', system-ui` / `system-ui, 'Segoe UI'` /
  `ui-monospace, Consolas`). Numbers in JetBrains Mono, tabular-nums in tables and readouts.
- Tokens on bare `:root` (light), redefined under
  `@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) {…} }` and again under
  `:root[data-theme="dark"]`. body background from a token. No color defined only inside a theme block.
  Canvas/SVG drawing code reads colors from the CSS tokens (getComputedStyle) and redraws when the
  theme changes (matchMedia listener + MutationObserver on data-theme).
- Light: ground #EEF1EE, panel #FAFBF9, panel-2 #F3F5F2, ink #151A17, ink-2 #48524C, ink-3 #6B756F,
  rule #D3D9D2, tile #DADFD9, tile-seam #C4CBC3. Pollen accent #D69A00 (marks) / #7A5300 (text).
  Red alliance #C8352E, blue alliance #2D6FD6. Status good #1C7A33 on rgba(12,163,12,.10),
  warn #855700 on rgba(250,178,25,.20), bad #AD312A on rgba(208,59,59,.09).
- Dark: ground #0E1210, panel #161B18, panel-2 #1B211D, ink #E3E9E4, ink-2 #A9B3AD, ink-3 #7F8A84,
  rule #2A312D, tile #202622, tile-seam #2D352F. Pollen #E0A800 / text #E8B84A. Red #E0574F,
  blue #5A98EE. Status good #4CC56A on rgba(12,163,12,.16), warn #F2C253 on rgba(250,178,25,.14),
  bad #F07A72 on rgba(208,59,59,.17).
- Layout: app shell, max-width ~1440 px. Left: field (square, dominant). Right column (≈380 px):
  verdict panel, required-arc readout, motor readout, controls grouped (Shot / Motor & flywheel /
  Robot). Below: side view + 3-D view (tabs or side-by-side), then the all-motors table.
  Stacks to one column under 960 px; the field stays square; no horizontal page scroll.
- Borders/radius by role (panels 6 px radius, hairline rules; not everything a card). No emoji.
  Uppercase micro-labels with letter-spacing. Visible focus rings. prefers-reduced-motion respected.
- `<title>BIOBUZZ Shot Sim</title>`. Opens in a realistic working state: Pollen, RED target
  (σ = −1), 6000 rpm motor, single wheel 96 mm hood, robot at (−12.75, −48), Typical precision.
- Text colors always come from ink tokens (never the alliance/verdict color on body text);
  alliance and verdict meaning always carry a label or icon too.

---------------------------------------------------------------------------------------
## 9. Tests (node --test) — minimum set
- Geometry: lip / top / centroid mapping (§3.1) within 0.02 in; world↔body transform round trip.
- Distance functions: pentagon inside / outside / edge cases; prism distance vs brute-force sampling.
- Flight: the no-air limit (C_D = C_L = 0) matches the analytic parabola within 1 mm at 1 s;
  POLLEN terminal speed 15.1 m/s ± 0.1.
- In-plane cross-check vs the 2-D Python model (`ftc-base/shooter/shooter_math.py`, pickleball lift,
  S0 = 1): robot at x = hx, y = σ·u0 aiming along ψ toward the pivot. POLLEN, h0 = 16 in:
  u0 = 48 → the θ = 70.5° row scores for v in ≈ [4.80, 5.31] m/s and the v = 5.05 m/s column scores
  for θ in ≈ [65.5°, 75.5°]; u0 = 30 → θ = 81.5° row ≈ [4.57, 5.20]. Agreement within one grid step
  (numbers from `SCR/shooter_out.txt`; the 3-D model adds side walls and the frame, which these
  in-plane shots do not touch).
- Motor model: 6000 rpm, 96 mm, single hood, v_exit 5.05 m/s → wheel ≈ 2233 rpm, h ≈ 0.372;
  I = 4e-4 kg·m², POLLEN → dip ≈ 4.9 %, recovery ≈ 50 ms (1 motor).
- Verdict: robot directly under the target pivot (hx, 0) → WON'T WORK; (hx, σ·48) with the
  6000 rpm motor, Typical precision → POSSIBLE; the slowest motor at G = 1 → WON'T WORK with a
  "tops out" reason.

---------------------------------------------------------------------------------------
## 10. Decisions after research (these override earlier sections where they conflict)

- **Field size:** use the measured field in `data/field.json`: `field.half` = 70.5 (inside wall
  faces, 141 in field), tile pitch 23.5, wall height 11.6 above the tile top. Everywhere this spec
  says 72, use `field.half`. "Left the field" = beyond half + 6. Robot centre clamp:
  |x|, |y| ≤ half − 9 (61.5). Field scan grid: 6 in step, centres −60 … +60 (21 × 21).
- **Zones, gardens, alliance areas, FLOWERs:** from `data/field.json`. FLOWERs are drawn, not ball obstacles.
- **Frame obstacles:** legs, crossbar and cornerBlocks from `data/field.json` (capsules with the listed
  radii). footBars and logoPanels are drawn only (floor-level / may be absent at events).
- **Arm bar:** the 6-point polyline and radius 0.8 from `data/field.json`.
- **CELL:** ringThickness and lugBelowBase are drawing details; ball collisions use the
  zero-thickness shell of §5.
- **Motors:** `data/motors.json` (11 × 5203 series, 24 mm REX shaft; the 5204 series with the 80 mm
  REX shaft has identical specs — say so in the UI help). Usable free speed =
  freeRpm × `shooter.model.freeSpeedFactor` (0.97, goBILDA's tested 5800 vs 6000 theoretical); use it for
  headroom h, v_cap and ω_wf. The §9 motor test therefore expects h ≈ 0.384 (2233 / 5820).
- **Exit efficiency:** η_single default 0.45, exposed as an "advanced" control 0.30–0.50
  (`etaSingleRange`; published hooded-shooter data spans 0.30–0.45). η_dual default 0.90, advanced
  0.80–0.95.
- **Flywheel:** wheel and inertia presets from `data/shooter.json`; default inertia preset "medium" (4.0e-4 kg·m²).
- **Scan API default:** `ShotEngine.scanPoints(6, field.half − 10.5)` → centres −60 … +60.

## 11. Additions (v1.1)

- **Battery voltage** (`shooter.batteryV`, default 12.0, UI 11.0–13.5): usable free speed and stall torque scale with V/12.
- **Speed control** (`shooter.control`): `pid` keeps the §6 model (full-power recovery, σ_motor(h), 92 % downgrade);
  `power` = open loop: recovery toward the setpoint with the same τ and no boost (residual = dip·e^(−Δt/τ)),
  σ_motor = openLoopSwingV / (V·√12), no headroom downgrade. PID never raises the top speed.
- **Flywheel inertia** from the chosen wheel: wheels-per-shaft × wheel inertia + optional goBILDA steel flywheel(s) + hubs
  (replaces the fixed inertia presets in the UI).
- **Show the math** dialog: every formula with this spot's numbers (sections: rpm chain, battery/PID, flywheel,
  wheel comparison table, flight, hit rate/verdict). `motorModel(...).details` and `ShotEngine.constants()` feed it.
