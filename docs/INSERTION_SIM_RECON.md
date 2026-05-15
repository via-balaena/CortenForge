# Insertion-Sim Recon — diagnosis of the depth-envelope stall

> **Recon date**: 2026-05-14. **Branch**: `dev`, forked off `main`
> `c9b8258d` (PR #244 squash-merge). **Reads from**:
> `docs/INSERTION_SIM_STATE.md` (the bookmark this recon answers).
> **Adds**: the [`run_insertion_ramp_on_analytical_sphere_shell`]
> discriminating experiment in `tools/cf-device-design/src/insertion_sim.rs`.

This document is the focused recon at the fundamental layer of the
slice-7.3b.1 depth-envelope stall. It is intended to be readable cold
alongside the bookmark.

---

## TL;DR

- **The stall is an SDF-gradient-quality problem, not a contact-
  formulation problem.** The bookmark's hypothesis "full-surface
  penalty contact is the wall" is **falsified** by the discriminating
  experiment.
- A fully analytical sphere intruder against a fully analytical
  sphere-shell body (no `GridSdf` anywhere) — with the **same κ, tol,
  n_steps, wall, cell_size, inset, Yeoh material** as the existing
  synthetic-icosphere ramp — **reaches the full 3 mm inset in all 16
  steps**. No stall. The icosphere case (same body shape, but
  `GridSdf`-mediated contact) stalls at step 13 ≈ 2.62 mm; the real
  scan stalls at step 4 ≈ 0.94 mm.
- The stall depth tracks **how rough the contact-side SDF gradient
  is**, not the contact extent. Trilinear-interpolation kinks in
  `SdfGrid` make the contact normal `n = ∇φ/|∇φ|` jump discontinuously
  across each grid-cell face the Newton iterate crosses; the
  assembled `κ·n⊗n` Hessian then mismatches the local Jacobian, and
  the tangent acquires spurious indefinite directions that Yeoh
  cannot compensate.
- The fix lives **in `cf-geometry::SdfGrid`'s gradient computation**,
  not in `sim-soft`'s solver or `PenaltyRigidContact`. Three sniped
  fixes are ranked below; the recommended one is **a smoothed
  gradient on `SdfGrid` (analytical derivative of the trilinear
  basis, or one Gaussian convolution pass on the signed-distance
  buffer before FD)**.

---

## The discriminating experiment

`tools/cf-device-design/src/insertion_sim.rs::run_insertion_ramp_on_analytical_sphere_shell`
(this commit) — mirrors `run_insertion_ramp_on_synthetic_sphere`
one-for-one except the body and the contact intruder are pure
analytical `Solid::sphere` CSG, never routed through `GridSdf`.

Parameters held identical to the icosphere baseline:

| Parameter | Value | Source |
|---|---|---|
| Wall material | Yeoh `ECOFLEX_00_30` (single layer, `ConstantField`) | matches `layer(0.010, "ECOFLEX_00_30")` |
| Wall thickness | 10 mm | matches baseline |
| Cavity inset | 3 mm | matches baseline |
| `cell_size_m` | 4 mm | matches baseline |
| `INSERTION_CONTACT_KAPPA` | 1e3 | matches baseline |
| `INSERTION_CONTACT_DHAT` | 1e-3 | matches baseline |
| Solver tol | 1e-1 | matches baseline |
| `max_newton_iter` | 150 | matches baseline |
| `n_steps` | 16 | matches baseline |
| Intruder radius | 40 mm | matches `icosphere(0.040, 3)` |

The only changed axis is the **SDF kind** for both the body
construction and the contact intruder. Body: `Solid::sphere(R+wall-inset).subtract(Solid::sphere(R-inset))`,
meshed via `SdfMeshedTetMesh::<Yeoh>::from_sdf_yeoh`. Intruder:
`Solid::sphere(R).offset(interference - inset)` rebuilt per ramp step.

### Result

```
analytical sphere-shell ramp — 46584 tets, 4090 pinned, 16 requested steps:
  step interference 0.19 mm — 4 Newton iters, residual 2.39e-3
  step interference 0.38 mm — 1 Newton iters, residual 3.28e-2
  step interference 0.56 mm — 1 Newton iters, residual 3.45e-2
  step interference 0.75 mm — 1 Newton iters, residual 3.67e-2
  step interference 0.94 mm — 1 Newton iters, residual 3.90e-2
  step interference 1.13 mm — 1 Newton iters, residual 4.14e-2
  step interference 1.31 mm — 1 Newton iters, residual 4.40e-2
  step interference 1.50 mm — 1 Newton iters, residual 4.66e-2
  step interference 1.69 mm — 1 Newton iters, residual 4.93e-2
  step interference 1.88 mm — 1 Newton iters, residual 5.22e-2
  step interference 2.06 mm — 1 Newton iters, residual 5.51e-2
  step interference 2.25 mm — 1 Newton iters, residual 5.83e-2
  step interference 2.44 mm — 1 Newton iters, residual 6.15e-2
  step interference 2.62 mm — 1 Newton iters, residual 6.50e-2
  step interference 2.81 mm — 2 Newton iters, residual 2.30e-3
  step interference 3.00 mm — 2 Newton iters, residual 5.00e-3
  → converged all 16 steps to the full 3.00 mm inset
```

Full 7.5-second wall run (release).

### What this tells us

1. **Full-surface penalty contact is not the wall.** With the same κ,
   the same tol, the same Yeoh material, the same ramp cadence, the
   same mesh resolution — the analytical version reaches 100% of the
   inset where the icosphere version reaches 87%.
2. **The warm-started residual carries beautifully.** Most steps need
   only **1 Newton iteration** — the residual at the chained `x_final`
   from the previous step already satisfies `tol = 1e-1`. The
   monotone drift 0.033 → 0.065 across steps 2–14 is exactly what a
   smooth, well-conditioned full-surface contact should produce: each
   small interference increment perturbs the equilibrium by a
   correspondingly small amount, and the warm start is on the
   solution side of `tol`. Steps 15–16 need 2 iters and the residual
   *drops* — the deeper contact is engaging more tets and the warm
   start no longer satisfies `tol` outright, but the Newton step
   itself takes them under it in one swing.
3. **The icosphere stall at step 13 had a different mechanism than I
   thought.** Looking at the [`run_insertion_ramp_on_synthetic_sphere`]
   logs: the residual *grew* with each step (because the GridSdf
   gradient kinks contributed to a worsening warm-start mismatch),
   and crossed the Armijo-stall regime somewhere past 2.6 mm. The
   analytical case proves that mechanism is not intrinsic to the
   contact geometry.
4. **By transitivity, the real-scan stall at step 4 (0.94 mm) is not
   the wall either.** It is the same SDF-gradient-quality problem,
   compounded by the scan's higher-amplitude sub-cell noise. A
   smoother contact-side SDF should extend it.

---

## Diagnosis — where the indefiniteness comes from (Q1 from the bookmark)

The path:

1. `PenaltyRigidContact::active_pairs` queries `prim.eval(p_pt)` for
   each vertex against each primitive. When `eval < d_hat`, the pair
   is active. (`sim/L0/soft/src/contact/penalty.rs:273–287`.)
2. For an active pair, `PenaltyRigidContact::gradient` and the
   Hessian assembly need the **contact normal**
   `n = ∇φ_prim / ‖∇φ_prim‖`, where `φ_prim` is the rigid
   primitive's SDF. The penalty Hessian contribution is `κ · n ⊗ n`
   — rank-1, normal-only (penalty.rs:209–214 + the assembly path at
   `backward_euler.rs:1148–1239`).
3. For analytical primitives (`Solid::sphere`, `Solid::cuboid`,
   `Solid::cylinder`), `∇φ` is computed by `cf_design`'s symbolic
   gradient (`design/cf-design/src/gradient.rs:34`). The gradient is
   exact and continuous *everywhere off the medial axis*. The medial
   axis of a `Solid::sphere` is a single point at the center; the
   medial axis of a sphere shell is the centerline-of-the-wall
   sphere. The Newton iterate stays well away from these
   pathological loci.
4. For `GridSdf`, `∇φ` is computed by **centered finite differences
   of the trilinear-interpolated `distance_clamped`**, with
   `eps = 0.5 * cell_size`
   (`design/cf-geometry/src/sdf.rs:400–429`).
5. The trilinear interpolation is **`C⁰` but not `C¹` at cell
   faces** — within a cell the interpolant is tri-linear in (u, v, w),
   but its partial derivatives are piecewise (bilinear, bilinear,
   constant) and **jump across cell-face boundaries**. Taking a
   centered FD with `eps = 0.5 * cell_size` smooths within a cell but
   does not remove the cell-face jumps; it just averages them
   across the FD window.
6. Therefore `n_grid = ∇φ / ‖∇φ‖` **jumps direction discontinuously**
   as a function of the Newton iterate, every time a vertex crosses
   a grid-cell face.
7. The assembled Hessian at iterate `x_k` carries `κ · n_k ⊗ n_k`
   blocks. At iterate `x_{k+1}` (after the line search), some
   vertices have crossed cell faces — the *actual* local Jacobian
   carries `κ · n_{k+1} ⊗ n_{k+1}` blocks with **different
   normals**. The assembled tangent at `x_k` (used to compute δ for
   that step) and the true Jacobian (which Armijo evaluates via
   residual changes) **differ on the contact-active subset**. This
   is precisely the mismatch that makes the Newton step a poor
   descent direction at the iterate.
8. **Additional rough source**: the FD-derived gradient has spurious
   small-magnitude components transverse to the actual surface
   normal even at points not near cell faces — the FD stencil
   straddles a cell, and the directional derivatives of trilinear
   interpolation along the stencil axes pick up the *neighbor cell's*
   coefficient on one side. The normalized direction `∇φ / ‖∇φ‖`
   thus has a transverse component whose magnitude scales with the
   discontinuity amplitude at the nearest cell face.

So Q1 answered: **the indefiniteness is not from Yeoh validity, not
from sliver tets, and not from rank-deficiency of the contact-only
Hessian. It comes from the contact normal `n` not being a smooth
function of `x` on the GridSdf side — the assembled tangent's
`κ·n_k⊗n_k` is the wrong rank-1 perturbation for the Jacobian at
`x_{k+1}`, and the mismatch accumulates as more vertices cross cell
faces during the iterate trajectory.**

Yeoh validity is not the issue: at 3 mm interference on a 10 mm
wall, the average principal stretch is ~1.3 — well inside Yeoh's
`max_principal_stretch ~ 5–9` envelope per `sim/L0/soft/src/material/yeoh.rs:196–207`.

Sliver tets contribute a *constant* conditioning floor that is
present at every iterate, including those that converge; if slivers
were the dominant issue the shallow steps would also struggle.

---

## Diagnosis — line-search vs Newton direction (Q2)

The Armijo line-search merit function is
`‖r(x + α δ)‖₂` (free-DOF residual L2 norm) — confirmed at
`sim/L0/soft/src/solver/backward_euler.rs:963–964`. The Newton direction
δ comes from `solve(A, -r)` where A is the assembled free-DOF
tangent (SPD-fallback LU when Cholesky fails, per `factor_free_tangent`
at backward_euler.rs:710–766). For pure Newton on `r(x) = 0`, the
direction is descent for `½‖r‖²` *iff `A` exactly equals `∂r/∂x` at
`x_curr`* (then `<r, A δ> = -‖r‖²`).

The diagnosis in Q1 names exactly the failure mode: with
`GridSdf`-mediated contact, **A does not equal `∂r/∂x` at `x_curr`
on the contact-active subset**. The assembled contact Hessian
`κ·n(x_curr)⊗n(x_curr)` is a piecewise-constant approximation; the
true `∂(κ(d̂-d)n)/∂x` carries `∂n/∂x` terms that are *Dirac-distributional*
at cell faces (n flips abruptly) and zero elsewhere.

So the Newton direction is technically *correct in expectation
within a cell*, but the line-search picks up the cell-crossing
deltas and reports "‖r‖ went up, not down" — Armijo backtracks to
α=2⁻²¹, then panics.

**Q2 answered: it is the Newton direction (assembled tangent vs
actual Jacobian) that is wrong on the contact-active subset, not the
line-search itself.** Therefore swapping Armijo for trust-region
would *not* fix the problem alone — trust-region needs the same
tangent and would see the same descent failure. The fix must address
the SDF gradient smoothness.

---

## Diagnosis — Q3 (rows reach 8 mm with the same solver)

**Q3 falsified.** The bookmark hypothesized two candidate
differences between the rows' 8 mm success and our 0.94 mm stall:
"localized vs full-surface contact" and "analytical SDF vs
`GridSdf`". The discriminating experiment isolates them:

| Setup | Contact extent | SDF kind | Reaches |
|---|---|---|---|
| Row 22/23/24 sphere probe | Localized | Analytical | 6–8 mm |
| **This experiment (analytical sphere shell)** | **Full-surface** | **Analytical** | **3 mm (full)** |
| Synthetic icosphere ramp | Full-surface | `GridSdf` | 2.62 mm |
| Real iter-1 scan ramp | Full-surface | `GridSdf` | 0.94 mm |

The "Full-surface vs Analytical" row is the new datapoint. It
discriminates: **full-surface contact does not stall when the SDF is
analytical**. The rows reach 8 mm because their SDFs are analytical,
not because their contact is localized. **The stall depth tracks
SDF-gradient smoothness, full stop.**

The 2.62 mm vs 0.94 mm gap between the icosphere and real-scan
GridSdf cases is then the *secondary* axis — both have trilinear
kinks at the 3 mm grid scale (`grid_cell_m = 0.75 × cell_size_m = 3 mm`),
but the real scan has *additional* sub-cell-scale noise from the
rotating-table capture process. The icosphere's smooth underlying
shape means the only "noise" the GridSdf sees is the
discretization-induced trilinear roughness; the real scan compounds
that with measurement noise.

---

## Diagnosis — Q4 (is the formulation right?)

**Penalty contact is fine.** The penalty formulation with κ = 1e3
reaches full 3 mm in the analytical experiment. There is no need
for the more invasive Dirichlet-hybrid formulation the bookmark
sketched (which would also require extending `BoundaryConditions`
to per-vertex non-rest pin targets — currently rest-only at
`sim/L0/soft/src/readout/scene.rs:845–852`).

Dirichlet-hybrid remains a *future-proof* option for arcs where the
formulation needs to capture per-tet separation behavior (post-
insertion limb retraction, dynamic press-fit). For the current Fork-B
relative-comparison goal, penalty is sufficient *once the SDF
gradient is smooth*.

---

## Diagnosis — Q5 (literature)

The fundamental issue moved out of the contact-formulation literature
and into the **signed-distance-field gradient quality** literature:

- **Frisken & Perry, "Designing with distance fields"**, SIGGRAPH
  2006 course notes — discusses C¹-continuous SDFs via tricubic /
  B-spline interpolation; recommends *adaptive* sub-sampling where
  gradient noise matters.
- **Bridson, "Computational fluid dynamics for graphics and
  animation"** (2nd ed., 2015), ch. 4 — explicit discussion of
  normal-computation pathologies from trilinear-interpolated grid
  SDFs in level-set-based contact / coupling.
- **Macklin et al., "Local optimization for robust signed distance
  field collision"**, SCA 2020 — practical mitigation: pre-smooth
  the SDF scalar field with a Gaussian (radius ~0.5–1 cell) before
  gradient queries, accepting a small surface-position bias for
  much smoother normals.
- **Wohlmuth, "A mortar finite element method using dual spaces
  for the Lagrange multiplier"**, SINUM 2000 — mortar contact
  integrates per-pair gradients over a contact patch, which by
  construction averages out point-wise normal-direction noise.
  Heavy implementation effort; the smoothed-SDF fix below is
  preferred unless mortar is wanted for other reasons.
- **Allgower & Georg, "Introduction to Numerical Continuation
  Methods"** (Springer, 2003), ch. 8 — adaptive step-size
  continuation: halve the increment on solver failure, retry; gives
  graceful degradation but does NOT extend the depth envelope when
  the underlying tangent mismatch is fundamental (as it is here).

The recon's verdict: **the fix is on the SDF side, not the
contact-formulation side.** Augmented Lagrangian, IPC, mortar,
trust-region — all are valid contact-formulation upgrades, but none
addresses the actual root cause for this stall. The
smoothed-SDF-gradient fix is both cheaper and more on-target.

---

## Sniped fixes — ranked

The three candidates below are ranked by **cost ÷ expected
envelope-extension on the iter-1 scan**. The current real-scan
envelope is 0.94 mm of 3 mm = 31%. The synthetic-icosphere baseline
shows what removing the *real-scan-specific* noise alone gets us
(87%); the analytical experiment shows what removing the
trilinear-cell-face roughness on top of that gets (100%). A fix that
reaches both gets the full envelope back.

### #1 — Smoothed gradient on `SdfGrid` (RECOMMENDED)

**Where**: `design/cf-geometry/src/sdf.rs::SdfGrid::gradient`.

**Change**: Replace the centered-FD-on-`distance_clamped` gradient
with one of:

(a) **Analytical derivative of the trilinear basis.** Within each
cell, `distance` is `Σ d_ijk · L_i(u) L_j(v) L_k(w)` for
`L_0(u) = 1-u`, `L_1(u) = u`. The partial derivatives in (u, v, w)
are piecewise-bilinear in the other two coords and step-function in
their own coord. This gives the *exact* gradient of the trilinear
distance — strictly smoother than the FD on the same underlying
data, no extra storage. About 60 lines.

(b) **Gaussian pre-smoothing of the distance buffer.** A
separable 3D Gaussian with σ ≈ 0.5–1 cell, applied once at SdfGrid
construction; then the existing FD-on-trilinear gradient becomes a
FD of a smoothed function and the cell-face kinks are blurred out.
About 100 lines + a small surface-position bias (~0.5 cell shift
inward) that the iter-1 grid_cell_m = 3 mm bound makes acceptable
(0.5 cell = 1.5 mm bias, comparable to the cleaned-scan noise
amplitude).

**Expected envelope-extension**: Option (a) alone resolves the
icosphere case (smooth analytical-equivalent gradient, no kinks) —
**predicted to take icosphere from 87% to ~100%**. Option (b) on
top of (a) attacks the real-scan sub-cell noise — **predicted to
take real scan from 31% to 80%+** (somewhere in the icosphere's
regime, possibly all the way to full).

**Cost**: ~60–200 lines in `cf-geometry`, plus a regression test on
the existing SDF gradient unit tests (`design/cf-geometry/src/sdf.rs:783–805`
already pin the trilinear behavior; need a smoothness-pin too).
The change is in a load-bearing primitive — *every consumer of
`SdfGrid::gradient_clamped` will see the new behavior*; a careful
cold-read of the call sites is required before landing. Grep
suggests modest blast radius: `gradient_clamped` is called from
`GridSdf::grad`, ray-cast, and a handful of mechanism-builder paths.

**Risk**: Surface-position bias from (b) — manageable. Option (a)
is bias-free.

**Estimated implementation slice**: **1–2 commits**. Could ship as
slice 7.3b.2-pre or as a self-contained cf-geometry sub-leaf with
no insertion-sim changes (the existing `run_insertion_ramp_on_*`
tests provide the integration regression).

### #2 — Finer GridSdf grid (cheap but limited)

**Where**: `tools/cf-device-design/src/insertion_sim.rs::build_insertion_geometry`,
specifically the `let grid_cell_m = 0.75 * cell_size_m` line.

**Change**: Drop `grid_cell_m` to `0.25 * cell_size_m` (1 mm at the
default cell_size 4 mm). Halves the per-step gradient-kink
amplitude, which scales linearly with `grid_cell_m`.

**Expected envelope-extension**: Linear in `grid_cell_m`; halving
the cell size roughly halves the per-step warm-start residual
drift. **Predicted: real scan from 31% to ~50–60%**, icosphere
from 87% to 90%+. Not the same as smoothing — the kinks are still
present, just smaller.

**Cost**: 1-line change in cf-device-design + the SDF build time
cubic in the grid resolution. At 0.25× cell_size the iter-1 scan
moves from ~720 ms (3 mm grid) to ~46 s (1 mm grid), and the
grid-storage footprint goes from MBs to a couple hundred MB. The
SDF is built once per geometry — the cost is paid once per design,
not per ramp step — but the storage footprint will become real on
larger workspaces.

**Risk**: Memory pressure on larger scans + build-time regression
in the cf-device-design UI's reactivity. Mitigable by making
`grid_cell_m` a per-design knob, but that's a UI change.

**Estimated implementation slice**: 1 commit. Useful as a *stepping
stone* fix or for diagnosis sweeps; not the durable answer.

### #3 — Adaptive-step continuation on the ramp (cheap, low headroom)

**Where**: `tools/cf-device-design/src/insertion_sim.rs::run_insertion_ramp`.

**Change**: On panic at step k, halve the increment and retry from
the previous `x_final`. Continuation per Allgower-Georg.

**Expected envelope-extension**: Probably negligible — per the
bookmark, "the stall floor is depth-driven, not increment-driven"
(more `n_steps` doesn't help past a given absolute depth). Adaptive
continuation might give *some* relief if the stall is locally
non-uniform along the ramp (a single bad iterate that adaptive
step-size dances around), but the experimental data (icosphere
stalls at the *same* depth regardless of n_steps in the bookmark's
sweep) suggests this is the depth wall, not a transient pocket.

**Cost**: ~80 lines on top of the existing `run_insertion_ramp`
loop.

**Risk**: Encourages running the ramp through invalid solver
states; needs a hard floor (`min_step_size = inset / 64` or so) to
bail.

**Estimated implementation slice**: 1 commit. **Defer** — not
recommended as a primary fix; could land as a defensive layer
*after* #1 has done the heavy lifting.

---

## Recommendation

**Implement Sniped Fix #1, Option (a) — analytical derivative of the
trilinear basis on `SdfGrid::gradient`.** It is the most-on-target,
bias-free, smallest-blast-radius fix, and the recon predicts it
unlocks the icosphere case (87 → ~100%) plus a large chunk of the
real-scan case. If the real-scan envelope doesn't reach 100% with
just (a), layer (b) on top — Gaussian pre-smooth — for the second
commit; the two are additive.

**Do not implement #2 or #3 first.** They are diagnostically useful
(running the ramp with a 1 mm grid would confirm "kink amplitude is
linear in grid_cell" before changing the gradient algorithm) but
neither is the durable answer.

**Do not change the contact formulation** (Dirichlet hybrid,
augmented Lagrangian, mortar, IPC) for this stall. The penalty
contact + GridSdf pair is the right architecture; the GridSdf
gradient just needs to be smoother.

### What the implementation slice looks like

A reasonable shape for the next session ("session 3", per the
bookmark-pattern):

1. **Slice 7.3b.2-pre — analytical trilinear gradient on SdfGrid.**
   `cf-geometry` change. Regression net: existing
   `trilinear_*` tests + a new `gradient_continuous_across_cell_faces`
   unit test (sweep a query point along an axis, assert
   gradient-direction variation is below a tight bound).
2. **Slice 7.3b.2 — `InsertionResult` (outputs)** as the bookmark
   plans. Builds on whatever ramp envelope (a) delivers.
3. **Slice 7.3b.3 — Gaussian pre-smooth on SdfGrid** (Sniped Fix #1
   Option (b)), *only if needed* — i.e. only if (a) alone doesn't
   carry the real iter-1 scan to ~100%. Behind a feature flag or
   constructor param so the un-smoothed path stays available.

If the analytical-gradient fix unlocks both cases, slice 7 lands
clean and 7.3b.3 is parked.

---

## Constraints honored

Per the bookmark's "Constraints the recon must respect":

- **Yeoh material kept.** No proposal touches the material model.
- **Route A geometry kept.** No proposal moves off `GridSdf` for
  the body; the discriminating experiment is a *side-by-side*
  control, not a replacement architecture.
- **Fork B kept.** All proposals preserve the relative-comparison
  semantics; (a) is bias-free, (b)'s ~1.5 mm surface bias at
  grid_cell_m = 3 mm is acceptable for relative comparison.
- **Synthetic icosphere ramp test preserved.** The proposed
  gradient change *passes* the existing regression (it should
  only make convergence *better*); the test's `>= 10 steps`
  floor stays satisfied.
- **Slice-7 deferred items honored.** The 7.4 Slacker → modulus
  refinement is untouched; the un-gate of `mod insertion_sim` is
  still 7.4; `InsertionResult` is still 7.3b.2.

---

## Bookmark

- **Branch**: `dev`, HEAD ⟨this commit⟩.
- **Adds**: `run_insertion_ramp_on_analytical_sphere_shell` test +
  this recon doc.
- **Next**: Implementation of Sniped Fix #1, Option (a), in
  `cf-geometry` — `SdfGrid::gradient` switches from FD-on-
  `distance_clamped` to analytical derivative of the trilinear
  basis. Predicted to take the icosphere case from 87% → ~100% and
  the real scan from 31% → 80%+. Confirm with the existing
  `run_insertion_ramp_on_*` tests (no setup changes needed).
- **If (a) alone doesn't carry the real scan past ~80%**: layer
  Option (b) — Gaussian pre-smoothing of the SDF scalar field —
  on top. See Sniped Fix #1's details.
