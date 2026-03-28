# Solvers — Convergence Comparison

Runs all 3 constraint solvers on the same two-box stacking scene and compares
convergence speed, constraint satisfaction, and stability. This is the
definitive test: identical physics, identical initial conditions, only the
solver method differs.

## What it does

Creates 3 independent simulations — one per solver — and steps each for
5 seconds (2500 steps at dt = 0.002). Measures iteration count, contact
penetration, position drift, and energy stability, then prints a comparison
table with 7 pass/fail checks.

No window — headless analysis. The output is the table.

## The 3 solvers

| Solver | Method | Convergence |
|--------|--------|-------------|
| **PGS** | Projected Gauss-Seidel (dual-space, iterative) | Linear |
| **CG** | Conjugate Gradient (primal, M^-1 preconditioner) | Superlinear |
| **Newton** | Newton (primal, H^-1 preconditioner, line search) | Quadratic |

**Fallback behavior:** Newton and CG fall back to PGS on non-convergence.
When Newton falls back, PGS overwrites `solver_niter`/`solver_stat` — the
original Newton stats are lost. Detect Newton fallbacks via
`data.newton_solved == false`.

## What to expect

```
=== Solver Comparison (t = 5s, dt = 0.002) ===

  Solver       Avg iter  Max iter  Drift(mm)  Max depth(mm)  E drift(%)  Fallback
  ────────────────────────────────────────────────────────────────────────────────
  PGS            47.4       100      0.000       0.0256      +0.0000%    n/a
  CG             31.1       100      0.000       0.0256      -0.0000%    n/a
  Newton          0.2         2      0.000       0.0256      -0.0000%    0/2500
```

**Key insight:** Newton converges in 0-2 iterations on this simple scene
(quadratic convergence near the solution). CG needs ~31 iterations with its
M^-1 preconditioner. PGS needs ~47 iterations with coordinate-wise descent.
All three produce an identical, rock-solid stack — the difference is purely
in how fast they get there.

## Scene

```
         +---+
         | B |  upper box (free body, 0.5 kg)
         +---+
         +---+
         | A |  lower box (free body, 1.0 kg)
         +---+
    ===========  ground plane
```

Two 0.2m boxes stacked on a ground plane. Multi-contact (4+ contact points
per box-plane pair), friction coupling (condim=3), gravity loading. Boxes
start at their rest positions (box_a center z=0.1, box_b center z=0.3) to
minimize initial transient.

## Metrics

| Metric | Source | Description |
|--------|--------|-------------|
| Avg iterations | `data.solver_niter` | Mean iterations per step over 2500 steps |
| Max iterations | `data.solver_niter` | Peak iteration count (100 = budget exhausted) |
| Stack drift | `data.qpos[2]` | Max z-displacement of box_a after 0.5s settling |
| Max depth | `data.contacts[i].depth` | Max penetration depth (post-settling) |
| Energy drift | `data.total_energy()` | Drift from settled energy baseline |
| Fallback | `data.newton_solved` | Steps where Newton fell back to PGS |

## Validation (7 checks)

| # | Check | Criterion |
|---|-------|-----------|
| 1 | All solvers stable | Max z-drift of box_a < 1mm |
| 2 | Newton converges fast | Avg iterations <= 5 |
| 3 | PGS converges | Avg iterations <= 50 |
| 4 | CG faster than PGS | avg_iter_CG < avg_iter_PGS |
| 5 | Newton faster than CG | avg_iter_Newton < avg_iter_CG |
| 6 | Constraint violation bounded | Max penetration < 1mm |
| 7 | Energy stable | \|drift\| < 1% for all solvers |

## Run

```
cargo run -p example-solver-comparison --release
```
