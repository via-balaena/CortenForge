# Solvers Example Spec

**Status:** Implemented — 7/7 checks pass
**Date:** 2026-03-27
**Parent:** `examples/COVERAGE_SPEC.md` Track 1, item 6

## Goal

Exercise all 3 constraint solver implementations (PGS, CG, Newton) on an
identical contact-rich scene, measuring convergence speed, constraint
satisfaction, and stability as the primary comparison metrics.

## Why This Matters

Every existing example uses Newton (the default). PGS and CG have **zero**
visual examples. The constraint solver is the most performance-critical
subsystem in any contact simulation — understanding the tradeoffs is
essential for users tuning their own models.

## Scene

Two boxes stacked on a ground plane with friction. Simple enough to be
deterministic, complex enough to exercise the solver (multi-contact with
friction coupling, bilateral + unilateral constraints).

```
         ┌───┐
         │ B │  upper box (free body, mass 0.5 kg)
         └───┘
         ┌───┐
         │ A │  lower box (free body, mass 1.0 kg)
         └───┘
    ═══════════  ground plane
```

This matches COVERAGE_SPEC.md item 6 exactly. The stack exercises:
- Multi-contact (4+ contact points per box-plane pair)
- Friction coupling (condim=3, sliding friction)
- Gravity loading (steady-state constraint forces)
- Stack stability (force transmission through intermediate body)

## Solvers Under Test

| # | MJCF string  | Enum variant | Method                                       |
|---|-------------|--------------|----------------------------------------------|
| 1 | `PGS`       | `PGS`        | Projected Gauss-Seidel (dual-space, iterative) |
| 2 | `CG`        | `CG`         | Conjugate Gradient (primal, M⁻¹ preconditioner) |
| 3 | `Newton`    | `Newton`     | Newton (primal, H⁻¹ preconditioner, line search) |

**Fallback behavior:** Newton and CG fall back to PGS on non-convergence.
The example tracks fallback rate as a metric.

**Fallback details:** When Newton falls back to PGS, PGS **overwrites**
`data.solver_niter` and `data.solver_stat` — the original Newton stats are
lost. Detect Newton fallbacks via `data.newton_solved == false` (this flag
is only set to `true` when Newton converges without fallback). For CG,
there is no equivalent flag — CG fallback is not tracked separately.

## Example Structure

**Headless comparison only** — no per-solver visual examples (unlike
integrators, where drift is visual). Solver differences are in convergence
stats, not visual motion. All three produce the same stable stack.

```
solvers/
  comparison/               # cargo run -p example-solver-comparison (headless)
  SOLVERS_SPEC.md           # this file
```

The comparison example runs all 3 solvers on the same scene, steps each for
5 seconds, and prints a formatted comparison table + pass/fail checks.

### MJCF (identical for all, only solver= differs)

```xml
<mujoco model="solver-{name}">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002" solver="{name}"
          iterations="100" tolerance="1e-10">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <geom name="ground" type="plane" size="2 2 0.01"
          friction="0.5 0.005 0.001" rgba="0.35 0.35 0.38 1"/>

    <body name="box_a" pos="0 0 0.1">
      <joint type="free" name="jnt_a"/>
      <geom name="box_a" type="box" size="0.1 0.1 0.1" mass="1.0"
            friction="0.5 0.005 0.001" rgba="0.82 0.22 0.15 1"/>
    </body>

    <body name="box_b" pos="0 0 0.3">
      <joint type="free" name="jnt_b"/>
      <geom name="box_b" type="box" size="0.1 0.1 0.1" mass="0.5"
            friction="0.5 0.005 0.001" rgba="0.15 0.45 0.82 1"/>
    </body>
  </worldbody>
</mujoco>
```

Initial condition: both boxes at rest, positioned so box_a base is at
z=0.0 (touching ground, box center at z=0.1) and box_b base is at
z=0.2 (touching box_a top, box center at z=0.3). Box half-size is 0.1m,
so boxes start exactly at their rest positions with minimal transient.

**Note:** `<freejoint/>` is a MuJoCo shorthand that CortenForge does **not**
support. Use `<joint type="free"/>` instead.

### Timestep Choice

`dt = 0.002` — fine enough for stable contact, coarse enough that solver
iteration count is non-trivial and differences are measurable.

### Simulation Duration

5 seconds (2500 steps). Long enough to accumulate meaningful solver
statistics and verify steady-state stability. Short enough to run quickly.

## Metrics

For each solver, track per-step:

| Metric | Source | Description |
|--------|--------|-------------|
| `solver_niter` | `data.solver_niter` | Iterations to convergence this step |
| `improvement` | `data.solver_stat[last].improvement` | Final cost improvement |
| `gradient` | `data.solver_stat[last].gradient` | Final gradient norm |
| `nactive` | `data.solver_stat[last].nactive` | Active constraints at convergence |
| `nchange` | `data.solver_stat[last].nchange` | Constraint state changes last iter |

Aggregate over all steps:

| Aggregate | Computation |
|-----------|-------------|
| Mean iterations | sum(solver_niter) / nsteps |
| Max iterations | max(solver_niter) |
| Stack drift | max z-displacement of box_a from rest |
| Constraint violation | max penetration depth across all contacts |
| Energy stability | total energy drift over 5s |
| Fallback count (Newton only) | # steps where `data.newton_solved == false` |

## Console Output

```
Running 3 solvers for 5s each (dt=0.002, 2500 steps)...

=== Solver Comparison (t = 5s, dt = 0.002) ===

  Solver       Avg iter  Max iter  Drift(mm)  Max depth(mm)  E drift(%)  Fallback
  ────────────────────────────────────────────────────────────────────────────────
  PGS            47.4       100      0.000       0.0256      +0.0000%    n/a
  CG             31.1       100      0.000       0.0256      -0.0000%    n/a
  Newton          0.2         2      0.000       0.0256      -0.0000%    0/2500

  [PASS] All solvers stable: max drift < 1mm
  [PASS] Newton converges in <= 5 avg iterations
  [PASS] PGS converges in <= 50 avg iterations
  [PASS] CG faster than PGS: avg_iter_CG < avg_iter_PGS
  [PASS] Newton faster than CG: avg_iter_Newton < avg_iter_CG
  [PASS] Constraint violation < 1mm for all solvers
  [PASS] Energy stable: |drift| < 1% for all solvers

  PASS: 7/7 checks passed
```

## Pass/Fail Criteria (7 checks)

1. **All solvers stable**: max z-drift of box_a COM < 1mm over 5 seconds.
   All three must produce a stable stack — if any solver can't keep a
   two-box stack, that's a bug, not a feature difference.

2. **Newton converges fast**: average iterations per step <= 5.
   Newton should converge in 2-3 iterations on this simple scene (quadratic
   convergence near the solution).

3. **PGS converges**: average iterations per step <= 50.
   PGS is the workhorse — it should converge within the iteration budget
   (100) on every step.

4. **CG faster than PGS**: `avg_iter_CG < avg_iter_PGS`.
   CG uses a preconditioner (M⁻¹) and should need fewer iterations than
   PGS's coordinate-wise descent.

5. **Newton faster than CG**: `avg_iter_Newton < avg_iter_CG`.
   Newton uses full Hessian information — it should converge in fewer
   iterations than CG.

6. **Constraint violation bounded**: max penetration depth < 1mm for all.
   Solver accuracy target — contacts should be resolved cleanly.

7. **Energy stable**: total energy drift < 1% for all solvers (with energy
   flag enabled). The stack reaches steady state — energy should be
   approximately conserved after initial settling.

## Implementation Notes

- Use `sim_mjcf::load_model()` to build each model (inline MJCF with
  parameterized solver name)
- Use `data.step(&model)` loop — no Bevy, pure sim-core + sim-mjcf
- After each step, record `data.solver_niter` and final `data.solver_stat`
- Track box_a position via `data.qpos[0..7]` (free joint: 3 pos + 4 quat)
  — specifically `qpos[2]` (z-position)
- Penetration: use `data.contacts[i].depth` (positive = penetrating) for
  each of `data.ncon` active contacts. Alternatively, `data.efc_pos[i]`
  gives constraint violation per row (negative = penetration for contacts)
- Energy: `data.total_energy()` (with `<flag energy="enable"/>`)
- Initial `data.forward()` before first step to compute initial contacts
- Let the stack settle for ~0.5s before measuring drift (initial transient)
- Fallback detection (Newton): check `!data.newton_solved` after each step.
  When Newton falls back to PGS, PGS overwrites `solver_niter`/`solver_stat`
  — the Newton stats are lost. `data.newton_solved` is the only reliable
  indicator. CG has no equivalent flag; CG fallback is not tracked.
- All 3 solvers populate `solver_stat` with per-iteration `SolverStat`
  entries. PGS fills `improvement` only (other fields zeroed); CG and Newton
  fill all fields (`improvement`, `gradient`, `lineslope`, `nactive`,
  `nchange`, `nline`)

## Non-Goals

- No per-solver visual examples (all three look identical — a stable stack)
- No warmstart/noslip tuning (save for advanced contact-tuning example)
- No multi-scenario comparison (stacking is sufficient for first example)
- No wall-clock timing (too noisy, platform-dependent — iteration count is
  the meaningful portable metric)
- No solver parameter sweeps (iterations, tolerance — one config per solver)
