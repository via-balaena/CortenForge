# The commit series

> ⚠ Being rewritten into two PR series (Rigid-physics, then Rigid-loading; decided 2026-10-06) once the round-3 research lands. The single series below predates that decision.

Core first, then MJCF. Titles carry no `!` (the commit-msg hook refuses it); a breaking change says so in its body. Each commit compiles, is green alone, carries the tests it flips, its must-fail test(s), and (MJCF) its MuJoCo 3.5.0 citation. "Must-fail" = fails at the parent, passes at the commit. **The MJCF harness baseline is taken at the head of the core series, not at `main`** (K13 changes multi-joint trajectories if it lands).

### Core series

| # | commit | rows | detail | depends |
|---|---|---|---|---|
| K1 | `fix(sim-core): one pre-step check on every Result entry point; thermostat refuses h ≤ 0` | core-L3, core-D3 (display), P-L24 | A1 §1–2 | — |
| K2 | `fix(sim-core): hybrid sensor derivatives run every stage` | P-L33 | A2 §7 | — |
| K3 | `fix(sim-core): implicitspringdamper forward() leaves qvel unchanged` | core-L6 | A1 §3 | K1 |
| K4 | `fix(sim-core): reset_to_keyframe resets fully; energy_initial captured once per reset` | core-L2, core-L1 | A1 §4–5 | K1 |
| K5 | `fix(sim-core): RK4 advances plugins; Model::try_make_data` | core-L7 | A1 §6 | — |
| K6 | `feat(sim-core): kinematic trees in sim-core; Model::recompute_derived` | core-L5, P-L31 | A1 §7 (+ radii, tendon lengths, cf-design) | — |
| K7 | `feat(sim-core): callbacks fire in MuJoCo's order and counts` | core-C1, C2, C3 | A2 §1–2 | K2 (else hybrid `D` ships wrong, measured) |
| K8 | `feat(sim-core): finite differences refuse RK4 and history` | core-C1 (FD) | A2 §3 | K7 |
| K9 | `feat(sim-core): sensor derivatives at the current state` | rule (A2 Q4) | to be specified from A2 §4 at implementation; FD per-column `forward()` goes | K8 |
| K10 | `fix(sim-core): the bad-ctrl check reads the clamped input and leaves ctrl alone` | core-L4 | A2 §5 | — |
| K11 | `feat(sim-core): per-env BatchSim returns errors; model_of, forward_all` | core-B3, P-L19 | A1 §8 | K1 |
| K12 | `refactor: delete SimulationConfig, SolverConfig, Gravity` | core-D4 | A1 §9 + A3 §5 (sim-mjcf `config.rs`) | — |
| K13 | `fix(sim-core): multi-joint bodies match MuJoCo's bias force` | P-L32 | §5 | if Jon puts it in Rigid |
| K14 | `docs(sim-core): …` | core-C1, C4, B1, B2, D1, D2, D3, P-L25, P-L31 (FD `B`) | A1 §10, A2 §2, §4, §6, §8 | all K |

### MJCF series

| # | commit | rows | detail | depends |
|---|---|---|---|---|
| M1 | `feat(sim-mjcf): typed MjcfError with locations` | mjcf-E1, E2 | A3 §1–2 | — |
| M2 | `fix(cf-geometry): convex hull no longer depends on hash order` | P-L27 | A6 §1, gate | — |
| M3 | `fix(sim-mjcf): flex edges in MuJoCo's order` | P-L27 | A6 §1 (+ A6-Q2 refusal) | M1 |
| M4 | `fix(sim-mjcf): a non-finite inertia is an error, not a hang` | mjcf-H1 (hang) | A5 §3.1 `principal_axes` | M1 |
| M5 | `fix(sim-mjcf): depth-safe traversals on a large stack` | mjcf-H2 (part 1), P-L35 (`all_bodies`) | A4 §1.4, commit 1 | M1 |
| M6 | `feat(sim-mjcf): one attribute reader with MuJoCo's rules` | mjcf-S2 (numbers, lengths, required), P-L28 friction, mjcf-S8 (zero quat, multiple orientations), P-L35 (zero-quat hang) | A4 §2, commit 2 | M1 |
| M7 | `feat(sim-mjcf): every non-finite number is refused` | mjcf-S9, mjcf-H1 (decision) | A5 §3.1 | M6 |
| M8 | `feat(sim-mjcf): keywords exact; composite curve keywords as MuJoCo` | mjcf-S2 (keywords), curve map | A4 commit 3 | M6 |
| M9 | `fix(sim-mjcf): self-closing elements are not dropped` | mjcf-S4 | A4 §8.2 | M1 |
| M10 | `fix(sim-mjcf): repeated <compiler>/<option> merge` | mjcf-S5 | A4 §8.3 | M1 |
| M11 | `feat(sim-mjcf): defaults resolved in one pass before frames and composites` | mjcf-S10, P-L26, frame/composite/compiler-pass order bugs | A3 §3–4 (`validate_resolved` home) | M9 |
| M12 | `feat(sim-mjcf): explicit values survive their class` | mjcf-S11 | A3 §3 | M6, M8, M11 |
| M13 | `feat(sim-mjcf): default geom size, element-wise overlay, size must be positive` | mjcf-S3 | A3 §3 | M8, M11 |
| M14 | `feat(sim-mjcf): MuJoCo forms` | mjcf-S1 (`intvelocity`), mjcf-S8 (xyaxes/zaxis, box fromto, plane size), mjcf-S15, P-L28 (connect `site1`/`site2`) | A4 commit 6 | M6 |
| M15 | `feat(sim-mjcf): schema pre-pass at MuJoCo 3.5.0 with the extension allowlist` | mjcf-S1, H2 (depth), S14, decision 19, §4 item 1 | A4 §1, §3–5, commit 7 | M14 |
| M16 | `feat(sim-mjcf): joint layout as MuJoCo` | mjcf-H3 | A5 §3.3 | M11 |
| M17 | `feat(sim-mjcf): joint axis too small is an error` | mjcf-S9 | A5 §3.2 | M11 |
| M18 | `feat(sim-mjcf): one limit helper for joints, tendons, actuators` | mjcf-H4, S6, P-L1 | A5 §3.4 | M12 |
| M19 | `feat(sim-mjcf): sizes, masses and inertias as MuJoCo; sim-urdf writes inertial pos` | mjcf-S7 | A5 §3.5 | M13 |
| M20 | `feat(sim-mjcf): duplicate names are errors; sim-urdf world link` | mjcf-S12 | A5 §3.6 | M11 |
| M21 | `fix(sim-mjcf): frame contents are validated` | mjcf-S13 | A5 §3.7 | M11 |
| M22 | `fix(sim-mjcf): condim, <contact/> appends, repeated pairs, .mjb limit` | mjcf-D2 | A5 §3.8 | M11 |
| M23 | `feat(sim-mjcf): a vertex-only mesh gets its convex hull` | mjcf-D1 | A5 §3.8 | M2 |
| M24 | `fix(sim-mjcf): timestep > 1 loads` | P-L24 (mjcf half) | A5 §3.9 | M1 |
| M25 | `feat(sim-mjcf): delay and history` | P-L34 | §4 item 3 | M11 |
| M26 | `docs: MuJoCo divergences, sim-mjcf docs, conformance status` | D-docs, the divergences table, the stale "Real-World Model Loading ✅ COMPLETE" (its tests were deleted in `b56753d1`; 1 of the 16 listed Menagerie models loads) | A1 §10, A4, A5 | all |
