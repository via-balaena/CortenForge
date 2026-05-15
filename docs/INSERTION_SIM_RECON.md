# Insertion-Sim Recon — diagnosis and revised plan

> **Recon date**: 2026-05-14. **Branch**: `dev`, forked off `main`
> `c9b8258d` (PR #244 squash-merge). **Reads from**:
> `docs/INSERTION_SIM_STATE.md` (the bookmark this recon answers).
> **Adds**: the [`run_insertion_ramp_on_analytical_sphere_shell`]
> discriminating experiment in `tools/cf-device-design/src/insertion_sim.rs`
> + algorithm-neutral gradient-contract tests in
> `design/cf-geometry/src/sdf.rs`.
> **Records**: empirical falsification of an earlier draft of this
> recon's "Sniped Fix #1 Option (a)" recommendation. The fix landed,
> was measured, gave only marginal improvement, and was reverted.
> See §"What didn't work, and why" below — the lesson reshapes the
> next session's plan.
>
> **Iter 3 update (this commit, 2026-05-14)**: N1 / N2 / N3 measured.
> N1 (finer grid) *regressed* both envelopes — falsifies the recon's
> "trilinear cell-face kink scales with grid spacing" mental model
> for polyhedral inputs. N2 (10× SDF source mesh resolution) doubled
> the iter-1 envelope (31 % → 62 %); the iter-1 scan was severely
> under-resolved at 2 500 faces on a 50 cm geometry. N3 (Gaussian
> pre-smooth, σ = 0.5 cell) won decisively on both: synthetic 87 %
> → **100 %**, iter-1 31 % → **75 %**, and changed the failure mode
> from Armijo-stall (contact side) to Yeoh-stretch-validity
> (material side). **Structural-fix recommendation locked**: ship
> N3 in `cf-device-design::build_grid_sdf` next session. See
> §"Recon iter 3 results" below.

This document is the recon at the fundamental layer of the
slice-7.3b.1 depth-envelope stall. It is intended to be readable
cold alongside the bookmark.

---

## TL;DR

- **The stall is a contact-side SDF-smoothness problem**, not a
  contact-formulation problem. The bookmark's hypothesis
  "full-surface penalty contact is the wall" is **falsified** by
  the analytical-sphere experiment (full 3 mm inset, no stall,
  same κ / tol / n_steps / Yeoh material as the GridSdf ramps).
- **Recon iter 3 (measured) settled the structural-fix question
  with three discriminating experiments** (N1 / N2 / N3 — see
  §"Recon iter 3 results"):
  - **N1 (finer GridSdf grid) regressed both envelopes** —
    falsifies recon-iter-2's "cell-face kink scales with grid
    spacing" mental model. The polyhedral source mesh is itself
    C⁰; finer grid resolves its facet edges more faithfully, not
    less. Demoted as a fix.
  - **N2 (10× `sdf_target_faces` on the iter-1 scan)** doubled
    the iter-1 envelope (31 → 62 %). The decimated-scan proxy
    was severely under-resolved at 2 500 faces (~10 mm spacing
    on a 50 cm geometry). Recommended follow-up.
  - **N3 (Gaussian pre-smooth on the GridSdf signed buffer,
    σ = 0.5 cell) is the win** — synthetic icosphere 87 → **100 %**
    (full depth), iter-1 scan 31 → **75 %**, plus the per-step
    iter-count explosion of the FD baseline (8/24/56/58/61) is
    gone (replaced by steady 2-iter convergence). Failure mode
    on both ramps shifts from contact-side Armijo stall to
    material-side Yeoh-stretch-validity violation. **The contact-
    formulation is no longer the binding constraint after N3.**
- **Next step**: session 4 ships N3 as ~95 LOC in
  `cf-device-design::build_grid_sdf` (the prototype was built
  this session and reverted). User's autonomous-architect
  authority covers it. Tricubic / analytical-primitive fits and
  finer-grid options are all demoted — see §"Sniped fixes — final
  ranking."

---

## The discriminating experiment (preserved from the original recon)

`tools/cf-device-design/src/insertion_sim.rs::run_insertion_ramp_on_analytical_sphere_shell`
(commit `509f77dc`) — mirrors `run_insertion_ramp_on_synthetic_sphere`
one-for-one except the body and the contact intruder are pure
analytical `Solid::sphere` CSG, never routed through `GridSdf`.

Parameters held identical to the icosphere baseline (κ = 1e3,
tol = 1e-1, 16-step ramp, single-layer ECOFLEX_00_30 Yeoh, 10 mm
wall, 4 mm cell, 3 mm inset, 40 mm radius).

### Result

```
analytical sphere-shell ramp — 46584 tets, 4090 pinned, 16 requested steps:
  step interference 0.19 mm — 4 Newton iters, residual 2.39e-3
  step interference 0.38 mm — 1 Newton iters, residual 3.28e-2
  step interference 0.56 mm — 1 Newton iters, residual 3.45e-2
  ... (most steps 1 Newton iter, residual drifts 0.033 → 0.065)
  step interference 2.81 mm — 2 Newton iters, residual 2.30e-3
  step interference 3.00 mm — 2 Newton iters, residual 5.00e-3
  → converged all 16 steps to the full 3.00 mm inset
```

Full 7.5-second wall run (release).

### What this datapoint robustly tells us

The hierarchy of ramp envelopes:

| Setup | Contact extent | SDF kind | Reaches |
|---|---|---|---|
| Row 22/23/24 sphere probe | Localized | Analytical | 6–8 mm |
| **Analytical sphere shell (this experiment)** | **Full-surface** | **Analytical** | **3 mm (100%)** |
| Synthetic icosphere ramp | Full-surface | `GridSdf` | 2.62 mm (87%) |
| Real iter-1 scan ramp | Full-surface | `GridSdf` | 0.94 mm (31%) |

The "full-surface vs analytical" row falsifies "full-surface
contact is the wall." The remaining variable is "SDF kind" —
specifically, smoothness of the contact-side SDF. Full-surface
contact with a truly C¹ analytical SDF reaches 100%; full-surface
contact with a C⁰ trilinear-interpolant SDF stalls.

---

## What didn't work, and why (the lesson from this session)

The original recon recommended **Sniped Fix #1 Option (a)**:
replace `SdfGrid::gradient`'s centered-FD-on-`distance_clamped`
with the **analytical derivative of the trilinear basis**. The
mental model: FD's `eps = 0.5·cell_size` stencil straddles cell
faces, picking up trilinear-discontinuity artifacts in both the
`+eps` and `-eps` samples — "two cell-face direction jumps per
axis as the query moves, not one." Analytical-derivative-of-
trilinear was predicted to halve the kink count and yield
"icosphere 87 → ~100%, real scan 31 → 80%+."

### The implementation (and what it actually produced)

I implemented Option (a) cleanly — analytical partial derivatives
of the trilinear basis, bit-exact on linear fields, ~60 lines in
`design/cf-geometry/src/sdf.rs::SdfGrid::gradient`. Compile clean,
all cf-geometry unit tests passed (195 / 195). Then I measured on
the actual insertion-sim ramps:

| Ramp | Pre-fix | Post-fix | Δ |
|---|---|---|---|
| Synthetic icosphere | 13 / 16 (2.62 mm, 87%) | **14 / 16 (2.62 mm, 87%)** | +1 step |
| Real iter-1 scan | 4 / 16 (0.94 mm, 31%) | **5 / 16 (1.12 mm, 37%)** | +1 step, +0.18 mm |
| Analytical sphere (control) | 16 / 16 (100%) | 16 / 16 (100%) | bit-identical, doesn't go through `SdfGrid` |

**Per-step Newton-iter cost exploded on the synthetic ramp**:
steps 10–14 took 8 / 24 / 56 / 58 / 61 iters respectively (vs
pre-fix's mostly-1-iter / max-2-iter pattern). Wall time went
30 s → 85 s. Real scan iter counts also grew (3 → 5 → 5 → 9)
before stalling.

### What the smoothness test revealed

The cf-geometry unit test I wrote to pin the smoothness improvement
(`analytical_gradient_smoother_than_centered_fd_on_smooth_field`)
**failed**: on a smooth non-singular field
`f(p) = p.x + 0.05·sin(2π·p.y)`, the analytical-trilinear gradient
produced a per-sample max angular step of **0.098 rad**, while the
centered-FD gradient on the same sweep produced **0.013 rad** —
roughly 8× *smaller* per-step jumps for FD.

This is the empirical evidence that the original mental model was
wrong. The correct picture:

- **Trilinear interpolation is C⁰ but not C¹ at cell faces.** This
  is the actual root of the cell-face gradient artifacts. Neither
  FD nor analytical-derivative can remove it; both inherit some
  form of it.
- **Analytical derivative concentrates each cell-face kink at the
  cell boundary itself** — one step jump of magnitude
  ≈ `cell_size × second-difference-of-distance` per cell per axis.
- **Centered-FD with `eps = 0.5·cell_size` spreads the same total
  angular variation across a half-cell-wide transition** on each
  side of the cell boundary. The FD's "straddling error" the
  original recon flagged as a defect is, in practice, a *smoothing*
  effect — Newton iterates that step by sub-cell distances see a
  gradually-rotating gradient instead of a concentrated jump.

Total angular variation per cell is similar between the two
algorithms. Per-step max-jump (which is what the Newton solver's
warm-start residual sensitivity scales with) is actually **larger
for analytical** — exactly the opposite of the original
prediction.

### Why Newton's iterations exploded post-fix

Pre-fix, the warm-started residual at each ramp step satisfied the
loose tol = 1e-1 outright — the gradient field was smooth enough
that the per-step interference increment barely perturbed
equilibrium. The solver converged in 1 Newton iter per step.

Post-fix, the analytical gradient produces a *different* deformed
state (because the contact normals differ from FD's), and that
state has higher warm-start residuals (per the per-step `residual`
column rising from ~0.03 to ~0.08–0.1 across the same depths).
Once the residual is above tol, Newton enters its iteration loop,
where the concentrated cell-face jumps cause the assembled tangent
to mismatch the local Jacobian more sharply per step — driving the
iter count up exactly when the depth grows.

Net: the fix produced a different equilibrium (1 more step
reachable on each ramp before catastrophic Armijo failure), but
the per-step Newton convergence got *worse*, not better.

### The revert

Commit reverted on `dev`. cf-geometry `SdfGrid::gradient` back to
centered-FD; WGSL shader comments restored to "Matches CPU"; GPU
spec doc reverted. The new gradient-contract tests in
`design/cf-geometry/src/sdf.rs` were **kept** — they pass for the
FD path too (and for any future higher-order replacement), and
they harden the gradient invariants that the original test suite
did not pin (bit-exactness on linear fields, outward orientation
within trilinear accuracy, bounded max-step on smooth sweeps,
fallback behavior).

---

## Revised diagnosis

The slice-7.3b.1 ramp stalls past ~0.94 mm (real scan) / ~2.62 mm
(synthetic) because **the trilinear interpolant of the SDF is
C⁰-but-not-C¹**. The contact normal `n = ∇φ / ‖∇φ‖` carries
cell-face direction artifacts that are intrinsic to the
interpolant, not to how its gradient is sampled. The Newton solver
sees these as a Jacobian-vs-assembled-tangent mismatch on the
contact-active subset:

- Centered-FD samples the artifact smoothly across half-cell-wide
  transitions → smoother local gradient, smaller per-step kinks,
  smaller per-iter Newton-direction defect. This is the
  pre-existing implementation.
- Analytical-derivative-of-trilinear samples the artifact as a
  concentrated step at the cell boundary → sharper local gradient
  changes, larger per-step kinks, larger per-iter Newton-direction
  defect. This is what Option (a) implemented.

Both algorithms inherit the same fundamental limitation. **The
durable fix has to make the underlying interpolant smoother, not
move the FD stencil around.**

### Where the limitation comes from, concretely

`SdfGrid::distance` interpolates 8 corner values with weights
`L_a(fx) L_b(fy) L_c(fz)`, which is a tri-linear function within
each cell — linear in each coordinate at fixed other-coords. The
partial derivatives `∂f/∂fx` are bilinear in (fy, fz) and
piecewise-*constant* in fx within a cell; at the next cell, they
take a different piecewise-constant value (the next cell's
forward-difference of distance). The gradient direction
therefore varies *bilinearly* within a cell and *jumps* at the
cell boundary.

For a fully-engaged contact set, the Newton iterate moves dozens
of vertices by sub-cell increments per step. Each iterate
crossing a cell face injects a contact-normal direction jump into
the assembled Hessian, accumulated across thousands of active
pairs. The assembled tangent at `x_k` mismatches the actual
Jacobian at `x_{k+1}` proportional to the *fraction of active
pairs whose nearest cell face was crossed during the step*. As
depth grows, more pairs activate and more cross faces per step;
the cumulative mismatch grows until the Armijo line-search can't
find a descent step.

The discriminating experiment shows this is exactly the
mechanism: a `Solid::sphere` SDF has no cell-face artifacts at
all (it is `r - ‖p - c‖`, C^∞ off the medial axis), and the
solver sails to full depth.

### What about the empirical post-fix +1 step?

Both ramps gained one converged step after the fix. The +1 is
real, but small enough that it doesn't change the qualitative
story: the wall is still there, just shifted by ~one increment.
The analytical-gradient equilibrium is a slightly different
deformed state from the FD equilibrium — at the cost of higher
per-iter cost. For a Fork-B relative-comparison tool, the +1 is
not worth the 3× wall-time regression.

---

## Next-recon experiments (before any structural fix)

Three cheap, sharp discriminating experiments. Each is ≤ 10 lines
in `cf-device-design`, runs the existing
`run_insertion_ramp_on_synthetic_sphere` +
`run_insertion_ramp_on_iter1_scan` in ≤ 15 min wall total, and
produces a data point that constrains which structural fix is the
right one. Run all three in one session, then write the resulting
fix-recommendation in this doc.

### Experiment N1 — finer grid, same algorithm

**Where**: `tools/cf-device-design/src/insertion_sim.rs::build_insertion_geometry`,
line `let grid_cell_m = 0.75 * cell_size_m`. Try `0.25 * cell_size_m`
(1 mm grid at the default 4 mm cell), then `0.125` (0.5 mm grid).

**What it tests**: does halving the cell-face-kink amplitude
proportionally extend the envelope? The trilinear-interpolation
kink is linear in `cell_size`; halving the grid spacing should
halve the per-step normal-direction jumps.

**Cost / time**: 1-line change, 5–10 min wall (the 0.5 mm grid
build will be ~10–30× slower than 3 mm but the solve dominates
for the icosphere; for the real scan, the SDF build itself becomes
the bottleneck — that's an informative finding).

**Predicted outcome**:
- If kink-amplitude is the binding constraint, envelopes scale
  linearly: icosphere from 87 → ~95% at half-cell, real scan from
  31 → ~50–60% at half-cell.
- If there is a depth wall *independent* of grid resolution (e.g.,
  a Yeoh-validity or contact-formulation issue we missed), the
  envelopes don't move. That would mean kink amplitude is a red
  herring and the structural fix has to address something else.

**What we'd learn**:
- If linear scaling: **Gaussian pre-smooth or tricubic**
  interpolation are the right structural fixes, and we can predict
  their payoff before implementing.
- If no scaling: the recon needs another iteration — there is a
  contact-side or solver-side issue the trilinear-kink hypothesis
  has been masking.

### Experiment N2 — finer SDF source mesh, same grid

**Where**: same builder call, `sdf_target_faces` parameter. Try
10× the current value (so the iter-1 scan goes from ~2500-face
proxy to ~25000-face). Synthetic icosphere is unchanged
(icosphere(0.040, 3) has 1280 faces and is already smoothly
sampled).

**What it tests**: is the real-scan-vs-icosphere gap (31% vs 87%)
driven by SDF source resolution, or by the trilinear-interpolation
limit, or both? The 7.0 spike concluded "tet quality is governed
by `cell_size`, not SDF face count" — but that was about *meshing*
quality, not gradient quality.

**Cost / time**: 1-line change in the iter-1 test, ~2–5 min wall.

**Predicted outcome**:
- If real-scan envelope grows with face count, the underlying
  surface noise (rotating-table scan capture) is being aliased into
  the SDF values. A pre-decimation smoothing pass on the scan
  itself would help.
- If it doesn't grow, the cell-face trilinear-interpolation kink
  dominates over scan noise at the current cell_size — N1's finer
  grid is the right axis.

### Experiment N3 — Gaussian pre-smooth on the GridSdf scalar buffer

**Where**: `tools/cf-device-design/src/insertion_sim.rs::build_grid_sdf`,
*after* the flood-fill labels the wall band. One pass of a
separable 3D Gaussian convolution on `grid_values: Vec<f64>` with
σ ≈ 0.5–1.0 cell. ~40–80 lines.

**What it tests**: does smoothing the underlying distance values
(so the interpolant approximates a C¹ function) extend the
envelope? This is Sniped Fix #1 Option (b) from the original
recon, now promoted in priority by the analysis above.

**Cost / time**: ~80 lines, ~5 min wall on each ramp.

**Predicted outcome**:
- If envelope extends materially (icosphere → 95+%, real scan
  → 60+%), Option (b) is the right structural fix.
- If it extends only marginally, the structural fix needs to be
  tricubic interpolation (a literal C² interpolant has C¹ gradient
  with no kinks of any kind) or analytical-primitive fitting.

**Side-effects to characterize**:
- Surface-position bias: Gaussian smoothing shrinks features
  by ~σ. At σ = 0.5 cell on a 3 mm grid, that's ~1.5 mm — large
  enough that the cleaned-scan inset offset accounting needs to
  account for it, or the Fork-B relative-comparison interpretation
  needs to acknowledge it.
- Wall-band preservation: the flood-fill's wall-band must remain
  intact post-smoothing or the GridSdf's sign correctness regresses.
  A smoothing pass that respects the wall-outside-inside labeling
  is the safe variant; an unrestricted smooth would leak the
  flood-fill assignment.

### Cross-experiment interpretation (specified, pre-measurement)

The three experiments combined give a 2×2 (or 2×3) discrimination
table:

|  | N1 grid helps | N1 grid doesn't help |
|---|---|---|
| N3 smooth helps | The fix is grid + smoothing | The fix is smoothing alone |
| N3 smooth doesn't help | The fix is grid alone (or something else) | Trilinear isn't the binding constraint — recon iter 3 |

N2 (face count) is an orthogonal axis — answers whether scan
surface noise contributes on top of trilinear-kink (additive) or
not.

---

## Recon iter 3 results (2026-05-14, this session)

The three experiments were measured on this branch. Each
experiment ran the two existing ramps (synthetic icosphere +
iter-1 scan) in release mode with `--ignored --nocapture`; logs
are checked in under `target/recon-iter-3/` for the session and
discarded after the commit.

### The baseline (post-revert FD on trilinear, current `dev` HEAD)

| Ramp | Steps converged | Last depth | Per-step Newton iters |
|---|---|---|---|
| Synthetic icosphere | **14 / 16** (87 %) | 2.62 mm | 3/1/1/2/2/1/2/2/2/8/24/56/58/61 → Armijo stall at 2.81 mm |
| Real iter-1 scan | **5 / 16** (31 %) | 0.94 mm | 4/3/5/5/9 → Armijo stall at 1.12 mm |

> *Bias-debiaser correction to recon-iter-2*: the recon-iter-2
> draft reported pre-fix synthetic baseline as `13/16` and pre-fix
> iter counts as "mostly-1 / max-2." The actual current-`dev`
> baseline shows 14/16 and the 8/24/56/58/61 iter explosion near
> 1.88 mm. The earlier numbers were either from an older state
> or imprecisely transcribed. **The iter-count explosion is
> present at the FD baseline, not introduced by Option (a)** —
> the recon-iter-2 §"Why Newton's iterations exploded post-fix"
> narrative misattributed it. The actual difference Option (a)
> made was a +1 step in envelope and ~3× wall-time regression,
> both small.

### N1 — finer grid (0.25× cell_size, 1 mm grid spacing)

One-line change: `let grid_cell_m = 0.25 * cell_size_m` in
`build_insertion_geometry`.

| Ramp | Steps converged | Last depth | Δ from baseline |
|---|---|---|---|
| Synthetic icosphere | 11 / 16 (69 %) | 2.06 mm | **−3 steps, regressed** |
| Real iter-1 scan | 4 / 16 (25 %) | 0.75 mm | **−1 step, regressed** |

**Falsification of the recon-iter-2 prediction "envelopes scale
linearly with kink amplitude."** Both envelopes regressed; the
finer grid made things *worse*, not better. Per-step iter counts
also exploded earlier (synthetic 11-iter stall at 2.06 mm with
prior steps already at 5/9/11; iter-1 already at 11 / 5 / 9 / 15
in the first four steps).

**Mental-model correction**: the input mesh (icosphere or
decimated scan) is itself C⁰ — its faces are the source of the
gradient discontinuities. A coarser grid was incidentally
low-pass-filtering the polyhedral facet edges; a finer grid
*resolves the polyhedral C⁰-ness more faithfully*. The cell-face
kink amplitude isn't a function of grid spacing; the kink is
inherited from the source mesh and the grid samples it.

This puts us decisively in the **"N1 doesn't help"** column of
the discrimination table.

### N2 — sdf_target_faces 10× on the iter-1 scan (2 500 → 25 000)

Synthetic icosphere is fixed at 1 280 faces (subdivision-3
icosphere); N2 doesn't apply.

| Ramp | Steps converged | Last depth | Δ from baseline |
|---|---|---|---|
| Real iter-1 scan | **10 / 16** (62 %) | 1.88 mm | **+5 steps, +0.94 mm** |

Per-step iters: `7/4/4/5/8/7/21/40/97/102` → Yeoh
stretch-validity violation at tet 3690, `max_stretch_deviation =
1.014`, `F = [2.014, 0.914, 0.520]`. **The failure mode changed**
from contact-side Armijo stall to material-side principal-
stretch validity.

**Interpretation**: at 2 500 faces, the iter-1 scan has an
average face spacing ≈ √(scan_surface_area / face_count) ≈
10 mm on a 50 cm leg — *more than twice the BCC cell_size
(4 mm)*. The grid is over-sampling the source mesh, picking up
its under-resolution as aliasing artifacts. At 25 000 faces,
face spacing drops to ≈ 3 mm, matching the grid_cell, and the
scan-side surface representation becomes faithful.

The recon's "may help (additive)" prediction *understated* N2 —
it was a 2× envelope improvement, more powerful than the
N3 smoothing on iter-1 alone.

### N3 — Gaussian pre-smooth on the GridSdf signed-distance buffer

Separable 3D Gaussian, σ = 0.5 cell, kernel radius 1 (3-tap),
clamp-edge boundary, applied to `signed` between step 6 and
`SdfGrid::new` in `build_grid_sdf`. Prototype in this session
was ~95 LOC; reverted on commit per "no implementation this
session" constraint.

| Ramp | Steps converged | Last depth | Δ from baseline |
|---|---|---|---|
| Synthetic icosphere | **16 / 16** (100 %) | 3.00 mm | **+2 steps, +0.38 mm** — full depth |
| Real iter-1 scan | **12 / 16** (75 %) | 2.25 mm | **+7 steps, +1.31 mm** |

Per-step iters synthetic: `3/1/1/2/2/1/2/2/2/2/2/2/15/14/47/14`
— the explosion pattern is *gone* through step 12, replaced by
steady 2-iter convergence; only the last four steps grow, and
the final step (3.00 mm) lands in 14 iters.

Per-step iters iter-1: `4/2/2/3/6/4/4/5/8/11/13/18` → Yeoh
stretch-validity violation at tet 4130, `F = [2.027, 1.068,
0.325]`. Smooth growth through step 12, then material wall.
Like N2, **the failure mode changed** from Armijo-stall to
Yeoh-validity.

**Side-effect characterization (surface-position bias)**:
Gaussian-smoothed signed-distance shifts the zero isosurface
by ≈ σ²·κ / 2 where κ is local mean curvature. With σ = 0.5 cell
= 1.5 mm on a 40 mm-radius sphere: bias ≈ 1.5² / (2·40) ≈
0.028 mm. For sharper device features (radii ~5 mm): bias
≈ 1.5² / (2·5) = 0.225 mm. Both are well within Fork-B
relative-comparison tolerance and below the BCC cell_size (4 mm).
Wall-band sign correctness preserved automatically — the
smoothing of a piecewise-`±|unsigned|` field with the kernel
radius ≤ wall-band width does not change region labels.

### Cross-experiment interpretation (populated, post-measurement)

|  | N1 grid helps | **N1 grid doesn't help** |
|---|---|---|
| **N3 smooth helps** | n/a | **← we are here. The fix is smoothing alone.** |
| N3 smooth doesn't help | n/a | (Would have meant: structural fix is something else — contact-formulation, Yeoh-validity preconditioning.) |

N2 axis (orthogonal): the iter-1 scan's source-mesh resolution
*also* matters, additively. The largest single-axis win on iter-1
is N3 alone (+7 steps); N2 alone is +5 steps; **N2 and N3 combined
were not measured this session** (session budget — listed as
session-4 verification) but the failure-mode change of both
experiments to Yeoh-validity suggests the combination would reach
deeper before hitting the material wall.

### Settled structural-fix recommendation

**Implement N3** (Gaussian pre-smooth on `GridSdf` signed buffer)
as the slice-7 fix. Specifically:

1. Re-introduce `gaussian_smooth_3d_separable` in
   `tools/cf-device-design/src/insertion_sim.rs` (prototype from
   this session is durable — kernel build + three separable
   passes + clamp-edge, ~95 LOC).
2. Call it after step 6 of `build_grid_sdf`, just before
   `SdfGrid::new`, with `sigma_cells = 0.5` as the default.
3. Update `build_grid_sdf`'s docstring with the σ²·κ
   surface-position-bias note and the Fork-B-relative caveat.
4. Tighten the `run_insertion_ramp_on_synthetic_sphere`
   regression assertion from `>= 10 steps` to `== 16 steps`
   (full depth) — earned now.
5. The iter-1 ramp test stays unchanged (it does not assert a
   step count); document the post-fix 12 / 16 envelope in its
   docstring.
6. **Keep N2 (`sdf_target_faces = 25_000` on iter-1) as a
   follow-up commit** if the 12 / 16 envelope isn't enough for
   slice-8/9 downstream use. Likely yes — the Yeoh-validity wall
   is the *next* recon target if higher inset depths are
   required, and N2 + N3 together would be a clean reach toward
   that wall.

**Why N3 and not tricubic**: Tricubic / B-spline would buy the
last few percent (synthetic already at 100 %, iter-1 already at
75 % with a material-side wall blocking the rest). The ~200–400
LOC `cf-geometry` upgrade isn't justified by remaining gain. If
slice-8 Save/Open or slice-9 wiring surface a regime where N3
isn't enough, revisit then.

**Why N3 in `cf-device-design`, not `cf-geometry`**: the σ²·κ
surface-position bias is acceptable for Fork-B insertion FEM but
*not* for cast-mold geometry (cf-cast-cli sizes molds from the
exact scan SDF) or scan-prep visualization. Scoping the smoothing
to insertion-sim's `GridSdf` preserves the engineering meaning of
the geometry elsewhere.

---

## Sniped fixes — final ranking (post-N1/N2/N3 measurement)

The recon-iter-2 ranking was conditional on the experiments. Now
that they are run, the ranking collapses to a single
implementation directive (#1) with one optional follow-up (#2)
and the rest demoted.

### #1 — Gaussian pre-smooth on the GridSdf scalar buffer ← SHIP

**Measured envelope-extension**: synthetic 87 → 100 %, iter-1
31 → 75 %. **Failure-mode shift** from Armijo-stall to
Yeoh-validity on both ramps — the contact-formulation is no
longer the binding constraint after the fix.

**Cost**: ~95 LOC inside `cf-device-design::build_grid_sdf`,
prototyped in this session and reverted per the no-implementation
constraint. Re-introduction in session 4 is a copy from the
recon-iter-3 spike.

**Risk**: surface-position bias ≈ σ²·κ / 2 ≈ 0.03 mm on smooth
features and ≈ 0.2 mm on 5 mm-radius features. Documented as a
Fork-B-relative tolerance; below the 4 mm cell_size budget;
auditable.

**Why scope to `cf-device-design`, not `cf-geometry`**: see
recommendation above.

### #2 — Raise `sdf_target_faces` on real-scan inputs (follow-up)

**Measured envelope-extension on iter-1**: 31 → 62 % at 25 k
faces alone; predicted ≥ 75–90 % when combined with #1.

**Cost**: 1-line change in the iter-1 ramp test + a constant
documented in `build_insertion_geometry` (recommend
`sdf_target_faces ≥ 25_000` for cleaned-scan inputs). Trades SDF
build time linearly with face count — the iter-1 build at 25 k
took ~30 s of additional setup, well within the ramp budget.

**When to ship**: if slice-8/9 downstream needs deeper inset than
75 % on real scans, combine with #1. Otherwise hold until a
real engineering case demands it.

### Demoted — Finer GridSdf grid (N1 falsified)

Empirically *regresses* both envelopes (synthetic 87 → 69 %,
iter-1 31 → 25 %). The cell-face-kink-vs-grid-spacing mental
model was wrong on polyhedral inputs — the kink is inherited
from the source mesh, not produced by the grid. Should not be
revisited as a fix; the N1 experiment itself stays useful as
historical evidence.

### Demoted — Tricubic / Catmull-Rom upgrade in `cf-geometry`

Not needed. The contact wall is no longer the binding constraint
post-N3. ~200–400 LOC `cf-geometry` upgrade would buy at most
the last 25 % of iter-1 envelope, and that gap is on the
material-validity side, not the gradient-smoothness side.
Revisit only if slice-8/9 surfaces a regime where N3 + N2 are
insufficient.

### Demoted — Analytical-primitive fitting for the contact intruder

Not needed for the same reason. The N3 fix preserves the scan
geometry verbatim (modulo the σ²·κ bias) and reaches deeper than
analytical-primitive fitting would (the iter-1 75 % envelope is
limited by Yeoh validity, not by intruder smoothness).

### Demoted (carryover) — Analytical-derivative-of-trilinear

Already falsified in recon-iter-2. Reaffirmed by recon-iter-3:
the cell-face kink it would have addressed isn't from the
trilinear interpolant per se — it's from the source mesh — and
N3 attacks it at the right layer.

### Punted (carryover) — Adaptive-step continuation

Still punted. The N3 ramp doesn't show a depth wall in the
"can't take another step" sense — it hits a material-side
validity check the solver enforces unconditionally. Continuation
won't help there.

---

## Constraints honored (unchanged from original recon)

- **Yeoh material kept.** Empirical: Yeoh tangent indefiniteness
  is not the binding constraint at the depths we're reaching
  (1–3 mm interference on a 10 mm wall gives principal stretches
  well inside Yeoh's `5–9` envelope).
- **Route A geometry kept.** The cleaned scan → GridSdf → CSG
  Solids path stays. The Gaussian-pre-smooth fix changes the
  GridSdf *values* without changing the architecture; the
  tricubic fix changes the interpolation algorithm without
  changing the architecture; the analytical-primitive fit changes
  *only the contact-intruder* side, leaving the body unchanged.
- **Fork B kept.** All candidate fixes preserve relative-comparison
  semantics. The Gaussian pre-smooth introduces a small
  surface-position bias that needs to be characterized but doesn't
  invalidate the comparison.
- **Synthetic icosphere ramp test preserved.** The post-fix
  envelope must reach at least the current 13 / 16 floor (the
  test's regression assertion).
- **Slice-7 deferred items honored.** 7.4 Slacker → modulus,
  un-gate of `mod insertion_sim`, `InsertionResult` — all still
  scheduled as the bookmark plans.

---

## Bookmark — implementation slice for next session

Recon iter 3 closed the loop. The next session is the **N3
implementation commit** (session 4 in the bookmark numbering):

1. **Reintroduce `gaussian_smooth_3d_separable`** in
   `tools/cf-device-design/src/insertion_sim.rs` (the prototype
   from this session — kernel build via `(-r..=r).map(|i|
   exp(-i²/(2σ²)))` then three separable passes with clamp-edge
   boundary handling; ~95 LOC). Document σ²·κ surface-position
   bias.
2. **Wire it into `build_grid_sdf`** between step 6 (signed
   buffer) and `SdfGrid::new`, with `sigma_cells = 0.5` as the
   default. Update the function's docstring.
3. **Validate**: run both insertion-sim ramps in `--release
   --ignored`. Synthetic must hit `16 / 16` (full depth);
   iter-1 must hit `≥ 12 / 16`.
4. **Regression assertion**: tighten
   `run_insertion_ramp_on_synthetic_sphere` from `>= 10 steps`
   to `== 16 steps`. The iter-1 ramp test asserts only ramp
   mechanics already; leave that alone but update its docstring
   to record the post-fix envelope.
5. **Optional within the same commit** — bump
   `sdf_target_faces` from `2_500` to `25_000` on the iter-1
   ramp test (recon-iter-3 §N2 finding) to characterize the
   combined #1 + #2 envelope. If it cleanly hits `≥ 14 / 16`,
   keep the bump; else split into a separate commit.
6. **Add a small cf-device-design unit test** verifying the
   Gaussian smoother is bit-exact on a constant field and is a
   no-op (within fp-tol) for `sigma_cells = 0.0`. Algorithm-
   neutral contract test for the helper.
7. **Cold-read** the updated `build_grid_sdf` + insertion-sim
   ramp tests as the N+1 pass. Then `cargo xtask grade
   cf-device-design` and `xtask grade cf-geometry` (neither
   should regress).

User's "autonomous architect" authority covers this commit.

Slice 7 closes after this commit, modulo deferred items —
`mod insertion_sim` un-gate (7.4), `InsertionResult` UI surface
(7.5), and the slacker-effective-modulus wiring (7.6) — which
the bookmark plans for slice 7.4+.

---

## Bookmark (commit + state)

- **Branch**: `dev`, HEAD ⟨this commit⟩ (recon-iter-3 doc-only
  update; cf-device-design code unchanged from recon-iter-2).
- **Preserves**:
  - The discriminating experiment
    `run_insertion_ramp_on_analytical_sphere_shell` (introduced in
    `509f77dc`, the original recon commit).
  - The algorithm-neutral gradient-contract tests in
    `design/cf-geometry/src/sdf.rs::tests` (introduced in
    recon-iter-2, kept on revert).
- **Reverts (carryover from recon-iter-2)**:
  - `SdfGrid::gradient` from analytical-trilinear back to centered-
    FD-on-`distance_clamped`.
- **Reverts (this session — no code change committed)**:
  - The N1 one-line edit (`grid_cell_m = 0.25 * cell_size_m`),
    measured + reverted.
  - The N2 one-line edit (`sdf_target_faces = 25_000` in the
    iter-1 test), measured + reverted.
  - The N3 ~95-LOC prototype (`gaussian_smooth_3d_separable` +
    its call site in `build_grid_sdf`), measured + reverted.
    Will be reintroduced in session 4 per the bookmark above.
- **Next**: session 4 — implement N3. See §"Bookmark —
  implementation slice for next session" above.

---

## Post-implementation note (slice 7.3c shipped, 2026-05-15)

The N3 fix shipped as slice 7.3c. Synthetic ramp reaches
`16 / 16` (full 3 mm); iter-1 ramp reaches `12 / 16` (~2.25 mm) —
both bit-exact reproductions of the recon-iter-3 measurements.
Synthetic-ramp assertion tightened from `>= 10 steps` to
`== 16 steps`. cf-device-design grew from 56 → 59 tests (the
three smoother contract tests).

### Surprise: N3 + N2 combo *regresses* iter-1, doesn't compose

The bookmark item 5 conditional "if combined ≥ 14 / 16, keep the
bump" was measured this session. Result: **the combo regresses to
9 / 16 (1.69 mm, 56 %)**, worse than N3-alone's 12 / 16. The
failure mode also flipped back from Yeoh-validity (N3-alone) to
Armijo line-search stall (combo).

Hypothesis (post-hoc — needs recon-iter-4 to confirm before any
re-attempt): at 2 500 faces the decimated proxy's facet length
scale is ~10 mm, far above the σ = 0.5 cell × 3 mm = 1.5 mm
smoothing bandwidth — the smoother completely eliminates the
proxy's polyhedral kinks. At 25 000 faces the proxy's facet
scale drops to ~3 mm, matching the smoothing bandwidth — but the
real scan-capture noise (rotating-table artifacts, sub-mm
amplitude features) is *exposed* by the faithful proxy. σ = 0.5
cell is enough to suppress one source of high-frequency content
or the other, but not both at the same time.

A clean N2 follow-up therefore needs σ retuning — likely σ ≥ 1.0
cell when `sdf_target_faces ≥ 25_000`. That's a recon-iter-4
project, gated on slice 8/9 surfacing a deeper-iter-1-envelope
*requirement*. Slice 7.3c shipped at 75 % iter-1 envelope; no
downstream consumer needs more right now.

`sdf_target_faces = 2_500` stays the iter-1 ramp test default;
the test's docstring records the combo-regression so the next
recon doesn't re-tread it without retuning σ first.
