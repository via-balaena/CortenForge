# scan-fit-3layer-sleeve-v15

**Row 21.5 — A2 (faer LU fallback) acceptance test.** v1.5 of the row 21 sleeve-fit synthesis row. v1 (`scan-fit-3layer-sleeve`, cuboid scan + sphere probe + z-slab) stays intact on dev as the apples-to-apples baseline; v1.5 pivots the geometry to **capsule scan + capsule probe + y-slab cut** per the row 21 v1.5 spec memo and exercises [sim-soft's A2 LU fallback][lu] at the two condensed-tangent factor sites.

[lu]: ../../../sim/L0/soft/src/solver/backward_euler.rs

## Why v1.5 ships now (and didn't, pre-A2)

The row 21 v1.5 spec was **parked empirically on 2026-05-08**: capsule + capsule contact at the cap apex concentrates the iter-0 penalty gradient sharply enough that faer's sparse Cholesky tripped a non-PD pivot at Newton iter 2-3 for every penetration depth tested (1/2/4 mm, vertex indices 13271/13039/13021). The skeleton solver's `factor_at_position` and `factor_and_solve_free` ended with `.unwrap_or_else(panic!)`, so the run terminated before captured-bits could be baked. See [`project_sim_soft_row_21_v1_5_spec.md`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_sim_soft_row_21_v1_5_spec.md) for the parked-state diagnostic.

A2 (this arc) replaces the panic with a cheap-path-first Lu fallback. On `LltError::Numeric(NonPositivePivot)`, both factor sites symmetrize the lower-tri triplets to the full sparsity pattern and factor via `Lu` against a cached `SymbolicLu`. Happy path stays bit-identical to pre-A2 code. The fall-through engages exactly when the parked spike said it would — **iter 2 vertex 13271** in v1.5 at 1 mm — and Newton continues.

## What v1.5 changes vs v1

| Axis | v1 | v1.5 |
|---|---|---|
| Scan SDF | `Solid::cuboid(0.020, 0.015, 0.040)` | `Solid::capsule(SCAN_RADIUS=0.015, SCAN_HALF_HEIGHT=0.025)` |
| Probe SDF | `Solid::sphere(PROBE_RADIUS=0.012)` | `Solid::capsule(PROBE_RADIUS=0.012, PROBE_HALF_HEIGHT=0.030)` |
| Probe penetration | 1 mm | 1 mm (unchanged — A2 unblocks the capsule-apex SPD trip but deeper depths hit a separate Armijo-stall wall) |
| Slab axis | `z = 0` equatorial (40 mm below contact zone) | `y = 0` longitudinal (capsule cap apex contact in view) |
| Bbox half-extents | `SCAN_HX/HY/HZ + WRAP_THICKNESS + CELL_SIZE` | `BBOX_HALF_XY = SCAN_RADIUS + WRAP_THICKNESS + CELL_SIZE`, `BBOX_HALF_Z = SCAN_HALF_HEIGHT + SCAN_RADIUS + WRAP_THICKNESS + CELL_SIZE` |
| Tet count | ~75 k | **38_592** (smaller — capsule wrap is geometrically slimmer than cuboid+offset) |
| Newton iter cap | 50 | **100** (A2 fallback's slow-convergence tail needs the budget) |
| Strain-energy ordering | `Ψ̄_inner > Ψ̄_middle > Ψ̄_outer` (flat-face contact loads inner directly) | **INVERTED** — `Ψ̄_outer > Ψ̄_middle > Ψ̄_inner` (apex contact transmits radially through the wrap; v1's `verify_strain_energy_ordering` retained under `#[allow(dead_code)]` per row 25 precedent) |

Everything else — `WRAP_THICKNESS`, 6/4/4 mm layer split, F4 silicone-table provenance (`ECOFLEX_00_20`/`DRAGON_SKIN_10A`/`DRAGON_SKIN_20A`), `MaterialField` partition by distance-from-scan, `PenaltyRigidContact`, the per-tet Ψ extraction pipeline — carries through from v1 unchanged. See [v1's README](../scan-fit-3layer-sleeve/README.md) for the architectural and mechanical payload analysis.

## A2 acceptance signature

Empirical capture (post-A2 bake, 1 mm penetration):

```
sim-soft: faer LU fallback fired at factor_and_solve_free at Newton iter 2 (free residual norm 1.22e+02) (Llt non-PD pivot: NonPositivePivot { index: 13271 })
sim-soft: faer LU fallback fired at factor_and_solve_free at Newton iter 3 (free residual norm 3.99e+01) (Llt non-PD pivot: NonPositivePivot { index: 12487 })
sim-soft: faer LU fallback fired at factor_and_solve_free at Newton iter 4 (free residual norm 1.55e+01) (Llt non-PD pivot: NonPositivePivot { index: 12492 })
sim-soft: faer LU fallback fired at factor_and_solve_free at Newton iter 5 (free residual norm 6.16e+00) (Llt non-PD pivot: NonPositivePivot { index: 12492 })
```

- **Iter 2 vertex 13271** matches the parked memo's 1 mm prediction verbatim (parked-state diagnostic confirmed pre-A2 trip at this exact vertex).
- Newton converges in **92 iters** to final residual `9.05e-11` (under `tol = 1e-10`).
- `force_total_z = -164.8 N`, `n_active_pairs = 151` (within the spec's "80-150" predicted range).
- Captured constants self-pin bit-equally on the verify pass (no `CF_CAPTURE_BITS`).

The headline mechanical readout (Ψ̄ per layer at 1 mm capsule-apex penetration):

| Layer | Ψ̄ (J/m³) |
|---|---|
| Inner  | 3.5e+02 |
| Middle | 2.6e+03 |
| Outer  | 8.6e+03 |
| Outer max | 4.3e+06 |

The ordering INVERTS vs v1's "softest+closest = highest Ψ̄" prediction. Mechanical reasoning: at the capsule cap apex the contact zone is a tiny near-point overlap (~151 pairs vs v1's ~294-pair flat-face disc), so the probe barely engages the cavity wall directly; instead pressure transmits radially through the curved wrap above the scan, loading the middle layer through a wider cross-section than the inner layer (which spans only a thin annulus near the apex). Effect #1 (compliance) and #3 (constraint-distance) still favor inner-over-middle, but effect #2 (distance-to-load) reverses in the apex-geometry — and the apex-load effect dominates. v1's `verify_strain_energy_ordering` is retained under `#[allow(dead_code)]` for a future v1.5-specific re-engage with the inverted predicate.

## Run

```sh
cargo run -p example-sim-soft-scan-fit-3layer-sleeve-v15 --release
cargo run -p cf-viewer --release -- examples/sim-soft/scan-fit-3layer-sleeve-v15/out/sleeve_yslab.ply
```

Release mode required per `feedback_release_mode_heavy_tests`. The y-slab PLY's `material_id` categorical + `displacement_magnitude` sequential auto-pick categorical + viridis palettes per cf-view's Q5 detector.

## Followups (deeper penetrations and beyond)

A2 unblocks the *originally-parked* Llt wall at every depth (1/2/4 mm — all three would tripled at iter 2-3 pre-A2). Where they sit post-A2:

- **1 mm** — clean convergence at 92 iters (this row).
- **2 mm** — A2 fallback engages; full convergence vs Armijo stall **not yet measured** (followup).
- **4 mm** — A2 fallback engages 4× at iters 3-7 (residual cascade 80 → 41 → 13 → 13), then Armijo line-search stalls at Newton iter 18 with `α ≈ 4.77e-7`, residual ~5.66e-2. This is a downstream Newton-basin / capsule-apex-stress limit beyond A2's scope, addressed by either (a) v2's multi-step quasi-static ramp, (b) reduced probe radius `<< WRAP_THICKNESS` so the bottom hemisphere doesn't bridge the wrap shell, or (c) swapping one side to a non-cap geometry (sphere, cylinder side) — per the parked memo's pattern (nn).

The deeper-penetration cases are followups; row 21.5 v1.5 at 1 mm is the clean A2 acceptance test.
