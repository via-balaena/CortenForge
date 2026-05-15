# Insertion-Sim State — `cf-device-design::insertion_sim`

> **Bookmark date**: 2026-05-14. **Branch**: `dev`, HEAD `605df1fb`,
> forked off `main` `c9b8258d` (PR #244 squash-merge). **Test count**:
> 64 (8 `#[ignore]`d, all passing). **Grade**: A on all automated
> criteria for `cf-device-design`.

This document is the bookmark of where the slice-7 insertion-sim arc
landed at the end of 2026-05-14, and the brief for the focused recon
that follows. It is intended to be readable cold, without conversation
context — a fresh contributor (or future-us after some time away)
should be able to resume from this doc plus the source.

---

## TL;DR

- The insertion sim runs end-to-end on the real iter-1 scan
  (`sock_over_capsule.cleaned.stl`).
- The **flood-fill SDF problem is solved** (the FEM-grade
  closest-face-normal sign issue that blocked everything at 7.2 has
  been replaced topologically; residuals collapsed ~400 000×).
- The **quasi-static ramp is built and working** — the real scan now
  *converges* multiple steps, where it previously could not converge
  *at all*.
- It does **not** yet reach the full 3 mm default press-fit on the
  real scan. The solver Armijo-stalls past ~0.94 mm, on a sharply-
  characterized failure mode (non-SPD tangent near the solution +
  Armijo line-search, residual floor rises with depth).
- The three surface levers (warm-starting / `tol` / `kappa`) have
  been applied and their marginal returns are clearly diminishing.
- **Slice 7 is paused after 7.3b.1.** The next move is recon at the
  fundamental layer of the stall — not more surface tuning — to
  identify a sniped fix. This doc names the sharp questions the
  recon must answer.

The pause is deliberate. The current state is a *good* bookmark
moment: two real, durable wins shipped, and the remaining failure is
characterized precisely enough to point recon at the right spot.

---

## The arc to here (7.0 → 7.3b.1)

| Sub-slice | Commit | What it bought | Was the next problem? |
|---|---|---|---|
| 7.0 — SDF bridge spike | `b36962ba` | Measured `mesh_sdf` + BCC stuffing throughput on the real scan; found low SDF source resolution (~1.5–3k faces) is optimal — tet count and quality are governed by `cell_size`, not SDF face count. | "How does this thread together into an end-to-end builder?" |
| 7.1 — geometry + per-layer Yeoh material | `0129b7d6` | `build_insertion_geometry(scan, design, …) -> InsertionGeometry`. Mirrors the rows-21–25 layered-sleeve precedent. Materials per-layer via `LayeredScalarField` keyed on the scan SDF. | "Does the solver actually run on this?" |
| 7.2 — single static insertion solve | `efb7bbbe` | `run_single_insertion_step` wires `CpuNewtonSolver` + `PenaltyRigidContact` + outer-skin Dirichlet BC. Validated on a synthetic icosphere (36 iters, residual 4e-11). | **First sign of trouble**: the real scan didn't converge — residual ~6e4, *not scaling with interference* (the smoking gun for a structural problem rather than "press too hard"). |
| 7.3a diagnostic | `ed676d8b` | `diagnose_iter1_scan_geometry` spike root-caused 7.2's failure: `simplify_sloppy_decoder` → non-manifold decimated topology → `mesh_sdf`'s closest-face-normal sign ~12% wrong → spurious huge contact forces. | "What's the fix?" |
| 7.3a fix | `9d1886d5` (spike) + `984c2125` (wire-in) | Route C: a flood-fill `GridSdf`. Samples *unsigned* distance (topology-blind) on a lattice, floods "outside" from the bbox corners, expands labels into the wall band by multi-source BFS. Sign is topological — immune to non-manifold edges, inconsistent winding, sub-cell holes. 99.2% sign-agreement with the legacy methods' confident consensus, `inside_components=1` (no flood leak). | **Result: real-scan single-step residual collapsed 6e4 → 0.157** (400 000×). Geometry/SDF problem solved. But the single-step still doesn't reach `tol` — Armijo-stalls near the solution on a non-SPD tangent. |
| 7.3b.1 — quasi-static ramp | `605df1fb` | `run_insertion_ramp`. Seats the intruder in N equal interference increments, each Newton solve warm-started from the prior `x_final`. Mesh + BCs *cloned* per step (faster + bit-identical vs the rows' re-mesh-per-step precedent — `SdfMeshedTetMesh`/`BoundaryConditions` are `Clone`). Three levers: warm-starting + `tol = 1e-1` + `kappa = 1e3` via `with_params`. | **Result: the real scan now solves** — partial, but for the first time, real. Synthetic seats to 2.62 mm of 3 mm; real scan to 0.94 mm of 3 mm. The deep-regime stall is what this doc is about. |

Each ✅ sub-slice was followed by an `N+1` cold read (small doc/fix
commits) — see `git log --oneline` for the full sequence.

---

## What works (and is durable)

These are landed, tested, characterized — recon should *use* them,
not relitigate them.

### The flood-fill `GridSdf` — the SDF for any real scan

Source: `tools/cf-device-design/src/insertion_sim.rs::GridSdf` +
`build_grid_sdf`. Verified by `grid_sdf_fix_spike`.

- Sign is **topological** (flood-fill from bbox corners through
  non-wall lattice points; multi-source BFS expansion into the wall
  band). Immune to the noisy-mesh defects `mesh_sdf` cannot handle:
  non-manifold edges, inconsistent winding, duplicate faces,
  sub-cell holes.
- Build cost is `O(n_grid_points × n_faces)` for the unsigned-
  distance sample, plus O(n_grid_points) flood-fill + BFS expansion.
  At a 3 mm grid on the iter-1 scan: ~720 ms serial.
- Resolution-independent sign correctness (sweep at 4/3/2 mm grids:
  identical 99.2% agreement with the legacy methods' confident
  consensus, identical 482 "disputed" points resolved).
- Exposes `cf_design::Sdf` — drops into `Solid::from_sdf` /
  `LayeredScalarField` / `PenaltyRigidContact` with no further
  adapters.
- One **known limitation** (not bug): the flood-fill leaks if the
  scan has a hole *larger than one grid cell*. The 7.3a diagnostic
  confirmed the iter-1 scan has only 2 boundary edges (essentially
  watertight); pathological scans would need the
  `GridSdfReport.inside_components` check (1 = healthy) as a
  pre-flight.

### The quasi-static ramp — `run_insertion_ramp`

Source: same file, `run_insertion_ramp` + `InsertionRamp` /
`RampStep`. Verified by `run_insertion_ramp_on_synthetic_sphere` and
`run_insertion_ramp_on_iter1_scan`.

- Seats the intruder in N equal interference increments
  (`0 → cavity_inset_m`).
- **Warm-starts** each step from the prior step's `x_final` (sim-
  soft's `solve_impl` uses `x_prev` as the initial Newton iterate;
  verified at `sim/L0/soft/src/solver/backward_euler.rs:854`).
- **Clones** the mesh + BCs per step. The rows-21–25 precedent
  rebuilds the mesh each step; we don't have to — `SdfMeshedTetMesh`
  and `BoundaryConditions` both derive `Clone`, and the cloned mesh
  is bit-identical to a rebuild. Faster, and removes an
  unnecessary nondeterminism risk.
- **`catch_unwind`**s each step. `replay_step` panics on non-
  convergence; the ramp catches the panic, captures the panic
  message into `InsertionRamp.failure_reason`, sets
  `failed_at_step = Some(k)`, and stops cleanly. *Graceful failure
  is a feature, not a bug* — `failure_reason` is what lets a caller
  understand which lever to pull next.

### The three convergence levers — applied

| Lever | Value | Default | Why |
|---|---|---|---|
| Warm-starting via `x_prev` chaining | enabled | enabled | Shallow steps drop from ~36–68 iters (cold) to **2–9 iters** (warm). The whole point of the ramp. |
| `INSERTION_SOLVE_TOL` | `1e-1` (N, free-DOF residual norm) | `SolverConfig::skeleton()`'s `1e-10` | The deep ramp steps Armijo-stall at a residual floor right around `0.1 N`. That is *exactly* the Fork-B "physically negligible" bar — sub-Newton out-of-balance against tens-of-N contact forces. Setting `tol` there converts stalls into clean loose convergences. The shallow steps still reach ~`1e-5`; `tol` only bites once a step hits the floor. |
| `INSERTION_CONTACT_KAPPA` | `1e3` via `PenaltyRigidContact::with_params` | `PenaltyRigidContact::new`'s `1e4` | Full-surface contact (the whole cavity wall engages at once, unlike the rows' localized probe) concentrates the penalty Hessian. Softening `κ` 10× nearly doubles the reachable depth. The tradeoff is slightly more residual penetration; acceptable for a relative-comparison tool. |

These three are all the surface knobs the design exposed. Each has
been pushed to its honest limit. Pushing further (`tol > 0.1`,
`κ < 1e3`) starts to degrade what the simulation actually means.

### Pipeline (as-built)

```
                          ┌──── CavityState ──┬─► SimDesign ─┐
  cf-device-design app ───┤                                  │
  (or test fixture)       └──── LayersState ──┘              │
                                                             ▼
   scan (mesh_types::IndexedMesh)  ─────► build_insertion_geometry
                                                  │
                              ┌───────────────────┴─────────────────────┐
                              │                                         │
                              ▼                                         ▼
                    decimate_for_sdf                           layer_boundary_thresholds
                      (meshopt sloppy +                                 │
                       mesh-repair weld)                                ▼
                              │                                LayeredScalarField (μ, C₂, λ)
                              ▼                                        │
                       build_grid_sdf                                   ▼
                  (sample unsigned distance              MaterialField::from_yeoh_fields
                   on a lattice, flood-fill                             │
                   "outside" from bbox corners,                         │
                   BFS-expand into wall band)                           │
                              │                                         │
                              └──────────────► GridSdf ──────────────►──┤
                                                  │                     │
                                                  ▼                     │
                              Solid::from_sdf(GridSdf, bounds)          │
                                .offset(-inset)  → cavity               │
                                .offset(total - inset)  → outer         │
                                outer.subtract(cavity)  → body          │
                                                  │                     │
                                                  ▼                     ▼
                            SdfMeshedTetMesh::<Yeoh>::from_sdf_yeoh(body, hints)
                                                  │
                                                  ▼
                          ┌── InsertionGeometry { mesh, intruder: GridSdf, cavity_offset_m,
                          │                       outer_offset_m, bounds, cell_size_m, n_tets }
                          │
                          ├─► run_single_insertion_step(geometry, interference_m)
                          │      → InsertionStep                              (7.2)
                          │
                          └─► run_insertion_ramp(geometry, n_steps)
                                 → InsertionRamp { steps, failed_at_step,    (7.3b.1)
                                                   failure_reason, final_x,
                                                   n_pinned }
```

### Public API

All under `tools/cf-device-design/src/insertion_sim.rs`, currently
`#[cfg(test)]`-gated (un-gates at slice 7.4 when the UI wires it
in).

- **Inputs** (caller-built):
  - `SimDesign { cavity_inset_m, layers: Vec<SimLayer> }`
  - `SimLayer { thickness_m, anchor_key: String }`
- **Stage 1 (geometry)**:
  - `build_insertion_geometry(scan, &SimDesign, sdf_target_faces, cell_size_m) -> Result<InsertionGeometry>`
  - `InsertionGeometry { mesh, intruder, cavity_offset_m, outer_offset_m, bounds, cell_size_m, n_tets }`
- **Stage 2 (SDF — both the helper and reusable building block)**:
  - `build_grid_sdf(scan, bbox, grid_cell_m, wall_threshold_m) -> Result<(GridSdf, GridSdfReport)>`
  - `GridSdf` (`impl cf_design::Sdf`)
  - `GridSdfReport { dims, grid_cell_m, n_outside, n_inside, n_wall, inside_components, build_ms }`
- **Stage 3 (solve)**:
  - `run_single_insertion_step(geometry, interference_m) -> Result<InsertionStep>`
  - `InsertionStep { x_final, iter_count, final_residual_norm, n_pinned }`
- **Stage 4 (ramp — the 7.3b.1 deliverable)**:
  - `run_insertion_ramp(geometry, n_steps) -> Result<InsertionRamp>`
  - `RampStep { interference_m, iter_count, final_residual_norm }`
  - `InsertionRamp { steps, failed_at_step, failure_reason, final_x, n_pinned }`

---

## The open problem — the depth envelope

### Numbers

16-step ramp, `tol = 1e-1`, `κ = 1e3`, default `cavity_inset_m = 3 mm`,
10 mm wall, single Ecoflex 00-30 layer:

| Geometry | Converges through | Of inset | Stall residual |
|---|---|---|---|
| Synthetic icosphere (well-conditioned) | step 13 / 16 = **2.62 mm** | 87% | ~0.2 N |
| Real iter-1 scan (capsule shape) | step 4 / 16 = **0.94 mm** | 31% | ~0.2 N |

### Failure signature

Every depth-stretched test fails identically:

> **Armijo line-search stalled at Newton iter N (r_norm ~0.1, final
> α ~5e-7). Likely causes: non-SPD tangent near solution (spec §3
> R-2 violation), or near-singular condensed system.**

Properties of the stall:
- The **residual decreases** with iterations (the LU fallback handles
  non-PD Cholesky pivots fine — the linear solve is not the problem).
- The **Armijo line-search** is what fails: no step size α ∈ (1,
  2⁻²⁰) reduces the merit function. That means the SPD-fallback-LU
  Newton direction is not a descent direction for the merit
  function at this iterate.
- The **stall floor rises with absolute depth**, not with increment
  size. More ramp steps (smaller increments) doesn't help past a
  given absolute depth — only changes *which step number* hits the
  floor.
- The **failure mode is identical** between the well-conditioned
  synthetic and the noisy real scan — only the *depth at which it
  bites* differs (real scan is harder by ~3×).

### Why the surface levers can't solve it

Already pulled to their honest limit:

| Lever | Stalled at | Notes |
|---|---|---|
| `tol = 1e-10` (skeleton) + `κ = 1e4` (default) | ~0.5 mm | The 7.2/7.3a single-step regime. |
| `tol = 1e-10` + `κ = 1e4`, ramp warm-start | ~1.7 mm synthetic | Better, but iter count grows quickly. |
| `tol = 1e-3` + `κ = 1e4`, ramp | ~1.5 mm synthetic | Looser tol helps the converging steps finish faster but still hits the rising stall floor. |
| `tol = 1e-1` + `κ = 1e4`, ramp | ~1.7 mm synthetic | Stall floor crosses 0.1 at depth 1.7 mm. |
| **`tol = 1e-1` + `κ = 1e3`, ramp** (current) | **2.6 mm synthetic / 0.9 mm real** | Best so far. |
| `tol > 0.1` | (untried) | Pushes the floor crossing deeper but stretches "physically negligible." |
| `κ < 1e3` | (untried) | More residual penetration; starts to change the physics. |
| More `n_steps` | (16 → 24 expected to help marginally) | Smaller increments help warm-start *quality*, but the stall floor is depth-driven, not increment-driven. |

The honest summary: **the surface layer is exhausted.**

---

## What the recon must answer

The recon's job is to identify the fundamental layer of this stall
and propose 1–3 *sniped* fixes ranked by implementation cost vs
expected envelope-extension. These are the sharp questions:

### 1. Where does the tangent indefiniteness actually come from?

Read `sim/L0/soft/src/solver/backward_euler.rs::solve_impl` and the
Hessian assembly path (`assemble_free_hessian_triplets`,
`assemble_global_int_force`). At a stalling iterate, which term
first contributes a strongly-negative eigenvalue?

Candidates:
- **Yeoh validity domain** (`validate_F_in_domain`). Past Yeoh's
  softening shoulder, the energy is non-convex. The slice-7 memo
  banked "NH caps at ~6 mm reliable contact; Yeoh widens it to ~8 mm
  in the rows." Our wall is 10 mm but the *strain* in the stalling
  tets could still be in or past the softening regime. Check the
  per-tet F at the stall iterate; identify which tets are past
  validity and how much of the Hessian they contribute to.
- **Penalty contact Hessian rank deficiency**. With `κ = 1e3` and
  most cavity-surface vertices active, the contact Hessian is
  high-rank in the *normal* direction but nearly zero in the
  *tangent* direction (no friction in the current setup). The
  combined tangent (elastic + contact) may have a near-null space
  in the "wall slides tangentially" mode. Diagnose: SVD the
  free-DOF Hessian at the stall iterate; identify the smallest
  eigenvalues and their eigenvectors. Does the smallest eigenvector
  correspond to a physically-meaningful tangential mode?
- **Sliver tets from BCC stuffing**. The 7.0 spike's quality report
  had `min_aspect_ratio ~0.04` at 1500 faces. Bad elements
  contribute disproportionately to a bad Hessian. Are the stalling
  Hessian's worst rows associated with the sliver tets?

### 2. Is the line-search the bottleneck, or the Newton direction?

When Armijo fails (no α reduces the merit function), it means the
search direction is bad at this iterate. Two possible roots:

- The **SPD-fallback LU direction** is mathematically not a descent
  direction. Newton's method assumes SPD tangent → SPD-fallback for
  the rare non-PD case is a backup but not a guarantee of descent.
- The **merit function itself** is what's bad — penalty-contact
  energy has near-discontinuities at contact-set changes, and at the
  stall, the wall is at a point where any direction enters a new
  pair-activation regime that increases the merit even infinitesimally.

The first would be fixed by a **trust-region** method (uses a model
trust region rather than line-search; handles indefinite Hessian
natively). The second would be fixed by **smoother contact
activation** (smooth-min barrier instead of step-function
penalty-band entry).

### 3. Why do the rows reach 8 mm with the same solver?

The rows (rows 22 / 23 in `examples/sim-soft/scan-fit-3layer-sleeve-*`)
ramp to 8 mm interference on cuboid + sphere geometry. Same solver,
same Yeoh material, same cell size, same skeleton solver config.
Two specific differences to test:

- **Contact extent**: localized (small sphere probe poking the
  cavity end) vs full-surface (whole intruder vs whole cavity).
  Localized contact has at most a few hundred active pairs; full-
  surface has thousands. The penalty Hessian's conditioning depends
  on the active-pair count.
- **SDF kind**: analytical (`Solid::cuboid` + `Solid::sphere`) vs
  `GridSdf` (trilinear-interpolated). The analytical SDF has a C¹
  gradient everywhere; the `GridSdf` gradient has trilinear-
  interpolation discontinuities at cell faces (small but nonzero).
  Could affect the local Hessian smoothness at active contact pairs.

A *cheap* discriminating experiment: build an analytical
`Solid::sphere(R)` intruder against an analytical
`Solid::sphere(R + wall).subtract(Solid::sphere(R - inset))` body
(no GridSdf at all), run the ramp. If it reaches 3 mm full depth,
the answer is "GridSdf gradient roughness." If it stalls at a
similar shallow depth, the answer is "full-surface contact period."

### 4. Is the formulation itself the right one?

Penalty contact is a soft constraint approximation. For a **snug**
press-fit — where the silicone is known-fully-engaged with the
limb everywhere — the natural formulation is a **Dirichlet BC**:
pin the cavity-surface vertices to the limb surface, pin the outer-
skin vertices to rest, solve for the interior. No contact
nonlinearity, no penalty Hessian, no Armijo stall on contact-set
churn.

Tradeoffs:
- Dirichlet **cannot capture separation** (the silicone pulling
  away from the limb in places). For a snug press-fit, that's
  arguably not happening (the wall is in compression everywhere
  inside-out). For a *loose* press-fit (or post-insertion relaxation
  where the limb retracts), separation matters.
- A **hybrid** formulation — Dirichlet on the "definitely engaged"
  region, contact on the regions where separation is possible —
  captures both. But it requires *deciding* which vertices are
  which, which is itself a sub-problem.

Question for the recon: is the engineering intent here always-snug,
or does separation matter? And does sim-soft's `BoundaryConditions`
API support per-vertex Dirichlet targets (not just pinned-at-rest)?

### 5. What does the literature do for this regime?

Full-surface elastic contact at moderate strain is a well-studied
regime. The recon should identify which of these the slice can
reasonably reach for, ranked by implementation cost:

- **Mortar method** (Wohlmuth-style integral contact constraint).
  Excellent conditioning, well-conditioned at high stiffness, but
  significant implementation effort.
- **IPC barrier with smooth-min activation** (Li, Ferguson et al.).
  Continuously differentiable contact energy, guaranteed
  intersection-free states, but a *substantial* algorithmic add to
  sim-soft.
- **Augmented Lagrangian** (Uzawa or Powell-Hestenes). Drops the
  ill-conditioning of pure penalty for a tractable algorithmic
  bump. Smaller change than mortar.
- **Continuation methods with adaptive step size** (Allgower-Georg).
  The ramp generalized — when a step fails, halve its size and
  retry instead of giving up. Cheap; might extend the envelope
  without changing the contact formulation.
- **Trust-region Newton** instead of line-search Newton. Handles
  indefinite tangents natively. Bigger change inside sim-soft's
  solver but possibly the right thing.

The literature recon's deliverable: a one-paragraph survey of which
1–2 of these match this slice's engineering intent + implementation
budget, with citations.

---

## Reproduction

To see the current state in front of you:

```
git checkout 605df1fb     # or current dev HEAD

# Synthetic icosphere ramp — seats to ~2.6 mm, stalls.
cargo test -p cf-device-design --release \
    run_insertion_ramp_on_synthetic_sphere \
    -- --ignored --nocapture

# Real iter-1 scan ramp — seats to ~0.94 mm, stalls.
# Needs the fixture: `/Users/jonhillesheim/scans/sock_over_capsule.cleaned.stl`
# (set CF_DEVICE_DESIGN_SPIKE_SCAN to override the path).
cargo test -p cf-device-design --release \
    run_insertion_ramp_on_iter1_scan \
    -- --ignored --nocapture

# 7.3a diagnostic — still valid post-fix as a sanity check on the
# GridSdf health + ramp pre-flight.
cargo test -p cf-device-design --release \
    diagnose_iter1_scan_geometry \
    -- --ignored --nocapture
```

The `failure_reason` field of `InsertionRamp` will hold the panic
message (the same "Armijo line-search stalled..." text) for the
failed step. Eyeballing the per-step iter counts + residuals across
the ramp shows the rising stall floor directly.

---

## Constraints the recon must respect

The recon is free to question anything except these — they are
shipped decisions, validated, and load-bearing:

- **Yeoh material**, not Neo-Hookean. NH caps at ~6 mm reliable
  contact, Yeoh widens to ~8 mm in the rows-21–25 precedent. The
  slice-7 plan settled this; the per-row precedent is the
  validation.
- **Route A geometry** (cleaned scan → flood-fill `GridSdf` → CSG
  Solids → meshed). Route B (prism-split the proxy) was rejected
  with reasoning; Route C is the within-Route-A SDF-sign fix.
- **Fork B** — this is a relative-comparison engineering aid, not
  an absolute predictor. The simulation's job is to let the user
  compare two designs, not to predict an absolute stress value.
  Sub-Newton out-of-balance is physically negligible against the
  tens-of-N contact forces — but *the shape* of the predicted
  deformation must remain meaningful.
- **The synthetic icosphere ramp test must stay passing**. It is the
  canonical regression for the SDF + ramp machinery. Losing it would
  mean regressing one of the durable wins.
- **The slice-7 plan's deferred items must not be silently
  invalidated**:
  - The Slacker → sim-modulus refinement is scheduled for 7.4
    (`SimLayer.slacker_fraction` field grows then, resolved via
    `SiliconeMaterial::from_effective_shore`).
  - `mod insertion_sim` un-gates from `#[cfg(test)]` at 7.4 (UI
    wiring), not earlier.
  - `InsertionResult` (per-tet stress / stretch / contact-force F-d
    curve) is scheduled for 7.3b.2.

---

## What 7.3b.2 and 7.3b.3 look like after the recon

- **7.3b.2 — `InsertionResult` (outputs)**, provisional. Per-step
  per-tet stress / stretch (reconstruct F from rest + current vertex
  positions → `Yeoh::first_piola` + `Yeoh::energy`; SVD → principal
  stretches), orphan-filtered contact-pair sum
  (`per_pair_readout` → `filter_pair_readouts_to_referenced`) →
  force-displacement curve. Builds on whatever ramp the recon-sniped
  fix delivers (full-depth or characterized-partial).
- **7.3b.3 (provisional) — the sniped fix itself**, only if the
  recon chooses to land it inside slice 7 rather than punt to a
  later sim-soft arc.

The order of operations after the recon will depend on which
question above turned out to be the real answer.

---

## Bookmark

- **Branch**: `dev`, HEAD `605df1fb`.
- **Forked off**: `main` `c9b8258d` (PR #244 squash-merge,
  "feat: cf-device-design layer-engineering suite + scan→cast
  scaffold").
- **Test count**: 64 (8 `#[ignore]`d, all passing).
- **Crate grade**: `cf-device-design` A on all automated criteria.
- **Lines of insertion-sim code** (`insertion_sim.rs`):
  ~2300 (mostly tests + docstrings; the algorithm payload is
  `build_grid_sdf`, `build_insertion_geometry`,
  `run_single_insertion_step`, `run_insertion_ramp`, plus their
  helpers).
- **Slice-7 plan memo** (private; complements this doc): the
  per-session plan + decision log lives in Claude's project memory
  alongside the codebase under `MEMORY.md`. This doc is the
  *public-artifact* version intended for cold reading.
