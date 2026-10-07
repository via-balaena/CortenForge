# How to read this book

**Status:** draft for stress test, 2026-10-06. Base `main` @ `3520544e`. Two PRs, split by layer — Rigid-physics, then Rigid-loading — with commits separated by area.

**What it is.** The spec for the "Rigid" PRs of the 0.10.0 arc. An outside user reviewed the published 0.9.0 `sim-core` and `sim-mjcf` crates; planning then compared both crates with MuJoCo 3.5.0, first row by row and then over the whole in-tree MJCF corpus (the parity census), and found far more than the user reported. This book holds the rule that decides every case, the decisions, and the commits that carry them out.

**How it is organised.**

- **Parts 0–3 are the spec.** They settle every question the research left open, order the work into commit series, and say how each commit is checked. Where they disagree with Part 4, they win.
- **Part 4 is the research** — twenty-one sections written by read-only researchers at `3520544e`, each with probes, measurements against `mujoco==3.5.0`, and MuJoCo source citations. Paths under `$SCRATCH` are the planning session's scratch and are not in the repo; every claim is re-established by the tests its commit adds.

**Row ids.** `core-*` and `mjcf-*` are the outside user's findings (triaged); `P-*` are items found while planning (the 0.10 carried-items ledger). Scope lists them all.

| research | area |
|---|---|
| [A1](research/a01-core-state.md) | sim-core state, lifecycle, BatchSim, docs |
| [A2](research/a02-core-callbacks.md) | callbacks, finite differences, the bad-ctrl check |
| [A3](research/a03-mjcf-errors-defaults.md) | `MjcfError`, default classes, where validation runs |
| [A4](research/a04-mjcf-parser.md) | schema pre-pass, attribute reader, allowlist |
| [A5](research/a05-mjcf-validation.md) | compiler-side checks at MuJoCo 3.5.0 |
| [A6](research/a06-determinism-verification.md) | build determinism, verification protocol, flip table |
| [A7](research/a07-delay-history.md) | actuator and sensor delay |
| [A8](research/a08-sleep-parity.md) | sleeping |
| [A9](research/a09-flex-composite.md) | flex orientation, flaps, cables, `curve` |
| [A10](research/a10-mesh-inertia-hull.md) | STL, principal axes, mesh inertia, hull |
| [A11](research/a11-lengthrange.md) | actuator `lengthrange` |
| [A12](research/a12-multijoint-bias.md) | multi-joint bias force (P-L32) |
| [A13](research/a13-isolate-dynamics.md) | connect/weld impedance, implicit `qacc`, flex edge rows |
| [A14](research/a14-isolate-deriv-geom.md) | position derivatives, `fromto`, ball `xaxis` |
| [A15](research/a15-isolate-fallthrough.md) | primitives falling through a box |
| [A16](research/a16-parity-census.md) | the parity census |
| [A17](research/a17-collision-convex-mesh-hfield.md) | native convex collision, MULTICCD, height fields |
| [A18](research/a18-constraint-solver.md) | Newton, CG, elliptic/noslip, equality, weld |
| [A19](research/a19-sensors-kinematics.md) | accelerometer, `cfrc_ext`, `ref`/`qpos0`, FD tangent, sleep under RK4 |
| [A20](research/a20-model-clusters-and-census-gate.md) | model clusters, flex comparability, the census gate |
| [A21](research/a21-collision-primitives.md) | primitive collision ports, contact frame and inclusion, GJK distance |
