# AI agent tools: CLI-Anything and REA for biobuzz-shot-sim (web shot simulator)

Research note, 2026-11-17. It covers two agent toolkits:

- **[CLI-Anything](https://github.com/HKUDS/CLI-Anything)** (Apache-2.0) wraps real desktop programs (FreeCAD, Blender, 3MF and others) in CLIs that agents can drive.
- **[REA](https://github.com/morluto/rea)** (MIT) is an MCP server and CLI that gives agents reverse-engineering tools.

This file covers only what matters for **this** repo.

How to read the labels:

- **Confirmed**: checked against the tool's own repo or docs.
- **Recommendation**: a suggestion, not tested.
- Nothing below was run end to end on this project yet.

## What could help here

| Tool | Use in this repo | Value |
|---|---|---|
| REA **JS/Electron analysis** | Run `rea analyze-javascript-application <abs path>/dist --json` on our own build to see which modules and source maps ship to GitHub Pages. **Confirmed**: it reads the files without running them, and it handles Webpack/Rspack bundles | Medium |
| REA **`environment-build`** | Record a HAR of the sim page, then build an agent environment (portfolio material). This works only if the app makes API calls; this sim is mostly client-side | Low |
| CLI-Anything **Blender** harness | Headless renders or decimation of ball and goal models | Low |

## Proposed changes (not made yet)

1. Before each Pages deploy, check that `dist/` doesn't ship source maps or unused vendor code.

## Status on the dev PC (2026-11-17)

- **Installed:** the agent skills `cli-hub-meta-skill` and `reverse-engineer-anything`, plus `cli-anything-hub` 0.4.1 (pip).
- **Not usable yet:**
  - The REA CLI/MCP needs Node 24.22.1 or newer, and the PC has 24.15.0.
  - The FreeCAD/Blender harnesses need those programs installed. Neither is.

## Rules for REA

Use REA only on code and builds we made, or on devices we own. Reverse engineering commercial software can break its EULA or DMCA §1201, and §1201(f) covers only narrow interoperability work. REA's tool results are sent to the AI model provider, so never point it at keys, `.env` files or credentials.
