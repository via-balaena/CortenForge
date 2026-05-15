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

This document is the recon at the fundamental layer of the
slice-7.3b.1 depth-envelope stall. It is intended to be readable
cold alongside the bookmark.

---

## TL;DR

- **The stall is a trilinear-interpolation problem**, not a
  contact-formulation problem. The bookmark's hypothesis
  "full-surface penalty contact is the wall" is **falsified** by
  the discriminating experiment (`Solid::sphere` analytical SDF
  reaches the full 3 mm inset, no stall, with the same κ, tol,
  n_steps, Yeoh material).
- The stall depth on `GridSdf`-mediated contact tracks how rough
  the contact-side SDF is — but the relevant roughness is *the
  C⁰-but-not-C¹ trilinear interpolant itself*, not just how its
  gradient is sampled. **Both centered-FD-of-trilinear and
  analytical-derivative-of-trilinear inherit similar total
  cell-face angular variation** (different distributions, similar
  totals). Replacing one with the other does not unlock the
  envelope — empirically validated by trying it (see §"What
  didn't work").
- The actual fix must make the underlying interpolant **smoother
  than trilinear**: either pre-smoothing the distance buffer
  before trilinear interpolation (so the interpolant approximates
  a C¹ function), upgrading to tricubic / B-spline interpolation
  (literally C¹ or C²), or feeding the contact path an analytical
  primitive fitted to the scan instead of `GridSdf`.
- **Next step: more recon, not implementation.** Three cheap
  discriminating experiments are listed in §"Next-recon
  experiments" — finer grid, larger SDF face budget, Gaussian
  pre-smooth on the distance buffer. Each is 1–10 lines of
  code, runs in ≤ 10 min wall time on the existing ramps, and
  gives a sharp data point. Run them before committing to a
  specific structural fix.

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

### Cross-experiment interpretation

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

## Sniped fixes — revised ranking (post-revert)

Each fix's expected envelope-extension depends on the N1–N3
discriminating-experiment outcomes, so the ranking is *conditional*
on those experiments having been run. Until they are, this is the
informed-prior ranking:

### Conditional #1 — Gaussian pre-smooth on the GridSdf scalar buffer

(This was the original recon's Option (b), now promoted by the
falsification of (a).)

**Effect**: shrinks the cell-face kink amplitude proportional to
the smoothing strength. A C¹-approximating interpolant produces
near-C¹ gradients with both FD and analytical-derivative methods.
This is the **single most-targeted fix** for the trilinear-
interpolant-is-C⁰ root cause.

**Cost**: ~80 lines in `cf-device-design::build_grid_sdf` (or
`cf-geometry::SdfGrid::new` if we want it to be universal). The
smoothing pass is separable 3D Gaussian, well-understood.

**Risk**: surface-position bias proportional to smoothing σ;
characterizable + correctable. Wall-band-preservation needs care.

**Predicted envelope-extension** (conditional on N3 confirming):
icosphere 87 → 95+%, real scan 31 → 60+%. Higher than Option (a)
delivered because this attacks the actual root.

### Conditional #2 — Tricubic / Catmull-Rom interpolation in `SdfGrid`

**Effect**: the literal C¹ (Catmull-Rom) or C² (B-spline)
upgrade. No cell-face kinks of any kind — the interpolant and its
gradient are smooth functions everywhere.

**Cost**: heavier, ~200–400 lines in `cf-geometry`. The
interpolation has to be implemented carefully (and the existing
trilinear-bit-exact tests need to be amended — Catmull-Rom is
not bit-exact on linear fields the way trilinear is, since it
uses 4 corners per axis).

**Risk**: Catmull-Rom can overshoot near discontinuities — the
flood-fill wall-band's hard distance transition would need
handling. Probably constrains to the "clamped" cubic variants.
Higher implementation complexity than Option #1.

**Predicted envelope-extension** (conditional): the most
generous — likely icosphere → 100%, real scan → 80–90%. But
the cost is also the highest of the three.

### Conditional #3 — Analytical-primitive fitting for the contact intruder

**Effect**: keep `GridSdf` for the body (so layered materials,
inset offsets, etc. all stay working), but fit a smooth
analytical primitive (capsule, ellipsoid, cylinder + caps) to the
*scan* and use that as the contact intruder in
`intruder_contact_at`. The contact normal is then C^∞ smooth —
exactly the regime the discriminating experiment showed reaches
full depth.

**Cost**: ~150–250 lines. Need a fit algorithm (least-squares fit
to scan vertices, with a chosen primitive family). The cf-cast arc
has prior art for capsule fits.

**Risk**: changes the engineering meaning — the contact is now
against an *approximation* of the limb, not the actual cleaned
scan. For Fork-B relative comparison this is acceptable (and is
how many commercial socket-fit tools work), but it does lose
fidelity.

**Predicted envelope-extension**: likely 100% on both ramps if the
fit is reasonable. The clearest path to "full depth on the real
scan", at the cost of changing what the simulation is actually
solving.

### Demoted — Analytical-derivative-of-trilinear (original Option (a))

**Empirically falsified** in the session that produced this recon
update. Reverted on `dev`. Should not be revisited unless paired
with N3 (Gaussian pre-smooth) — on top of a smoother underlying
field, the analytical-derivative variant might match FD's per-step
smoothness while preserving its per-cell consistency. But there is
no current evidence that this combined variant would beat
N3-alone.

### Punted — Finer SDF grid (Option #2 from original recon)

Useful as the N1 *experiment*, but not durable as a *fix*. The
memory and build-time cost of a 1 mm grid (let alone 0.5 mm) on
larger scans is real, and it doesn't address the kink — only
shrinks it.

### Punted — Adaptive-step continuation (Option #3 from original recon)

Still punted. The bookmark's observation that the stall is
depth-driven, not increment-driven, remains valid. Adaptive
continuation can't push past a depth wall.

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

The next session is *another recon session*, not an implementation
session. The deliverable is:

1. **Run N1, N2, N3** as branched experiments on `dev`. Each is a
   1-commit measurement (the `cf-device-design` change is small;
   run the ramps; record the numbers in a `cf-device-design` test
   docstring or an appended section of this doc).
2. **Update this doc** with the experimental results in
   §"Cross-experiment interpretation."
3. **Settle on a single structural fix** (Gaussian pre-smooth,
   tricubic, or analytical primitive) based on the data.
4. **Implement the chosen fix** as a dedicated commit (or 2 if
   needed). Validate with both insertion-sim ramps.

This is one or two sessions away. The user's authority to commit
the chosen fix autonomously is granted by the project-memory
"autonomous architect" memo; the choice of *which* fix has now
been gated on N1–N3 specifically because the original recon's
hypothesis was wrong about the FD-vs-analytical tradeoff.

---

## Bookmark (commit + state)

- **Branch**: `dev`, HEAD ⟨this commit⟩.
- **Preserves**:
  - The discriminating experiment
    `run_insertion_ramp_on_analytical_sphere_shell` (introduced in
    `509f77dc`, the original recon commit).
  - The algorithm-neutral gradient-contract tests in
    `design/cf-geometry/src/sdf.rs::tests` (introduced this
    session, kept on revert).
- **Reverts**:
  - `SdfGrid::gradient` from analytical-trilinear back to centered-
    FD-on-`distance_clamped`. WGSL shader comments + GPU spec
    doc warnings rolled back. cf-geometry test framings made
    algorithm-neutral.
- **Next**: more recon — run N1 (finer grid), N2 (more SDF faces),
  N3 (Gaussian pre-smooth) on the existing ramps; update this
  doc; settle the structural fix.
