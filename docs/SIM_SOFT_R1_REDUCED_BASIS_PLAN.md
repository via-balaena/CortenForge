# R1 — Linear Reduced Basis: scope and plan

**Status**: 2026-08-11, v4. **R1.0, R1.1 and R1.2 COMPLETE. R1 is done.** R1.0/R1.1
CONFIRM; **R1.2 is a SPLIT verdict** — the gradient algebra is exact (§13, G1) and the
gradient's *accuracy* does not follow from the state's (§13, G2). Parent:
`docs/SIM_SOFT_REALTIME_RECON.md` §7 rung R1. Predecessor R0 landed (`ecf4cfef`).

**What R1 is**: the cheap kill-or-confirm. It answers one question — *is this material's
deformation low-dimensional?* — and nothing else. No contact, no coupling, no
hyper-reduction.

**Reading order**: §1 (what R1 is and is not) → §2 (where the code goes) → §3 (**the
fixture decision — the one thing that needs your call**) → §4 (cost arithmetic, and why
it changes §3) → §5 (sub-rungs R1.0–R1.2) → §6 (gates) → §7 (kill conditions) → §8
(risks) → §9 (decisions for the head engineer).

---

## 1. What R1 is, and what it is deliberately not

**Is**: a POD basis `Φ` (`n_free × r`) built from full-order snapshots, a reduced Newton
solve on `q ∈ ℝʳ` with a dense `r × r` direct factorization, and the reduced IFT
gradient — wired at the same time, not retrofitted (recon §6).

**Is not**: hyper-reduced. Every element is still swept to build `f_int` and `K`; only
the *solve* is reduced. That is the point — R1 isolates "does a subspace represent this
deformation" from "can we make the element sweep cheap", which is R3.

**Not gated on wall time.** A fast-but-wrong subspace is worthless, and §4 shows the
speedup at R1 is size-dependent anyway. The gate is accuracy.

## 2. Where the code goes

`sim/L0/soft/src/solver/backward_euler/reduced/` — inside the `backward_euler` module.

**This needs no API change and no new dependency**, which is why it is the right home:

| need | already available as |
|---|---|
| internal force at `x` | `assembly.rs::assemble_global_int_force` — `pub(super)` |
| free tangent at `x` | `assembly.rs::assemble_free_hessian_triplets` — `pub(super)` |
| residual assembly | `helpers.rs::residual_into` — `pub(super)` |
| tangent matvec | `assembly.rs::internal_force_tangent_matvec` — `pub(super)` |
| thin SVD / symmetric eigen | `nalgebra` — already a dependency |

A sibling crate would force all five to become `pub`, widening `sim-soft`'s public
surface for an experiment that may be deleted. L0 tier rules are unaffected: no new
crates, dependency count unchanged at 110/200, no C toolchain, wasm32 unaffected.

**Basis size convention**: `r ≤ min(n_free / 50, 200)`. Stated up front so the gate is
falsifiable — without a ceiling on `r` you can always add modes until the error looks
acceptable, which makes "the basis works" unfalsifiable.

## 3. THE FIXTURE DECISION — this needs your call

The recon proposes R1 on the **cantilever at 3 000 free DOF**. I think that is the wrong
choice on its own, for a reason worth stating plainly:

**The cantilever's deformation is large-rotation bending, and linear subspaces represent
rotation badly.** A rotated configuration is not a linear combination of
small-displacement fields; this is the textbook failure mode of linear POD for
geometrically nonlinear structures, and it is exactly why modal derivatives exist
(recon §4a defers them to R3). At 43 % tip deflection the beam tip rotates a lot.

**So R1-on-cantilever has a high prior probability of failing — and failing for a motion
class the product may not care about.** The mission's deformation is tissue under a
device: localized indentation and bulk compression, with little rigid rotation. Killing
the whole MOR ladder on a cantilever result would be a false negative with large
consequences.

The converse risk is real too: the capstone is an *exoskeleton*, and limbs rotate. A
basis validated only on indentation would be a false positive for the eventual coupled
problem.

**Recommendation: run R1 on both, and treat them as two verdicts, not one.**

| fixture | deformation class | why |
|---|---|---|
| **`indentation_prescribed`** (new) | localized indentation + bulk compression | Mission-relevant. The bonded-layer geometry from `bonded_layer_indentation`, driven by a **prescribed displacement** on a top-surface patch instead of an IPC contact — same kinematics, no contact model, so it stays inside R1's no-contact scope. |
| **`cantilever`** (existing) | large-rotation bending | The hard case. Expected to stress the basis hardest, and the place modal derivatives would first pay. |

The contrast is itself the result. "Linear POD holds for indentation and breaks on
bending" is a precise, actionable answer that routes R3's basis work; a single blended
verdict is not.

## 4. Predicted cost — and why it moves the fixture size

Without hyper-reduction, each Newton iteration must still form `A_r = ΦᵀAΦ`. That costs
roughly `2·nnz_full·r` (sparse × dense) + `2·n·r²` (dense GEMM) — **linear in `n`**,
where the sparse factorization it replaces grows superlinearly. Using §2a's measured
`pattern nnz` and `numF`, at `r = 50`:

| free DOF | `ΦᵀAΦ` ≈ MFLOP | measured `numF` | verdict |
|---:|---:|---:|---|
| 3 000 | 26 | 4.58 ms | **projection is comparable or slower** |
| 19 440 | 174 | 85.7 ms | projection starts to win |
| 70 644 | 653 | 958.8 ms | projection wins decisively |

Two consequences:

1. **At 3 000 free DOF R1 will likely show no speedup at all** — which is fine (the gate
   is accuracy) but means that size cannot inform the performance question.
2. **Recon §4b needs a size qualifier.** Its v1.6 text says a basis alone "attacks the
   larger half" because factorization is 57–72 % of the frame. True asymptotically,
   false at 3 000 free DOF where forming `ΦᵀAΦ` costs about what factorizing `A` costs.
   Estimating the end-to-end effect at r = 50: ~1.1× at 19 440 free DOF, ~2.5× at
   70 644, ~1× or worse at 3 000. **A §4b amendment is part of this plan's first
   commit**, since leaving it unqualified would set a false expectation for R1's result.

⚠ Those MFLOP figures are **arithmetic, not measurements**, and the throughput
assumption behind the "verdict" column (~3 GFLOP/s for the memory-bound sparse-dense
half) is a guess. R1.1 measures it. They are here to size the fixture, not to be quoted.

**So: run the accuracy gate at 3 000 free DOF (fast to iterate), and repeat the winning
configuration at ≥ 19 440 to get a trustworthy first speedup datapoint.**

## 5. Sub-rungs

Deliberately three PRs, not one. **R1.0 can kill the whole ladder on its own**, before
any solver code exists.

### R1.0 — snapshots + basis, no solver

Snapshot collection from the f64 oracle; POD via the Gram-matrix route (`m × m`
symmetric eigendecomposition, since `n ≫ m`); singular-value spectrum and projection
error reported.

- **Deliverable**: `SnapshotSet`, `PodBasis`, spectrum + projection-error report.
- **Gate**: on a **held-out** trajectory (§6), projection error
  `‖u − ΦΦᵀu‖ / ‖u‖ < 1 %` at `r` within the §2 ceiling, on the indentation fixture.
- **This is the kill-or-confirm.** If the spectrum does not decay, stop here — no solver
  is written, and the cost of the answer was one module.

### R1.1 — reduced forward Newton

`r_r(q) = Φᵀ r(x_rest + Φq)`, `A_r = ΦᵀAΦ`, dense `r × r` Cholesky, Armijo unchanged.

- **Gate**: reduced trajectory vs the oracle — tip / indentation depth within 1 % over a
  held-out trajectory; Newton converges every step.
- **Also measures**: the §4 arithmetic, at both 3 000 and ≥ 19 440 free DOF.

### R1.2 — reduced gradient

Reduced IFT: `A_r λ = Φᵀ g_free`, contracted against `Φᵀ ∂r/∂θ`. `Φ` held **constant**
(recon §6 decision — the basis's parameter dependence belongs to the validity domain,
not to the chain rule).

- **Gate** (⚠ **amended before building — the original was one gate doing two jobs**):

  | | gate | tolerance |
  |---|---|---|
  | **G1** (kill) | reduced adjoint vs finite difference **on the reduced model** | crate gradcheck, 1e-5 rel |
  | **G2** (measured, loosely bounded) | reduced gradient vs the **oracle's** | pilot-set; expected at basis-error scale, *not* gradcheck |

  The original single gate — "reduced gradient vs the oracle's, to gradcheck tolerance" —
  cannot mean what it says. It conflates a statement about the constant-`Φ` IFT algebra
  (G1) with a statement about the basis (G2). R1.0/R1.1 already measured that the two
  models' *states* differ by 0.8–1.1 %; demanding 5-digit agreement between the
  derivatives of two functions whose outputs differ in the third digit would fail for
  basis truncation, a cost already priced, and not for anything about differentiability.
  G1 is the one that can fail for the reason §7 names, so G1 is the kill.

## 6. Gates, in detail

**Held-out trajectories are mandatory, and the recon's R1 gate is too weak without
them.** Recon §7 says "projection error vs the oracle < 1 % … over the training
trajectory". A basis trained and tested on the same trajectory reproduces it almost by
construction; that measures nothing. Every gate here is stated on a trajectory the basis
did **not** see.

Ensemble design, per fixture:

- **Train**: ≥ 5 trajectories varying load magnitude and direction (for indentation:
  indenter position, depth, and rate; for the cantilever: gravity direction and tip
  load).
- **Hold out**: ≥ 2 trajectories with parameters *inside* the training box but not on
  it, plus **1 deliberately outside** it — the latter is not a pass/fail gate but the
  first evidence about how the validity domain behaves at its edge, which R3 has to
  formalise.

Three quantities, all against the f64 oracle:

1. **projection error** — the basis's own ceiling, independent of the solver;
2. **trajectory error in a physical observable** — indentation reaction force, or tip
   displacement — *not* DOF-space L2, which flatters a basis that gets the bulk right
   and the surface wrong;
3. **gradient error** (R1.2 only).

## 7. Kill conditions, stated in advance

So they cannot be renegotiated once numbers exist:

- **R1.0 kill**: projection error on held-out data does not reach 1 % at any `r` within
  the §2 ceiling, **on the indentation fixture**. → linear POD is wrong for this
  material class; revert to R0's win and re-recon. Widening `r` past the ceiling to
  reach the number is the failure mode this condition exists to prevent.
- **Cantilever-only failure is NOT a ladder kill.** If indentation passes and bending
  fails, that is the expected signature of linear-subspace rotation weakness, and it
  routes R3 toward modal derivatives rather than killing MOR.
- **R1.1 kill**: reduced Newton fails to converge on trajectories where the oracle does.
- **R1.2 kill** (amended in lockstep with §5): the **G1** gradient error exceeds
  gradcheck tolerance — the reduced adjoint is not the derivative of the reduced model,
  so the IFT algebra is wrong and the differentiability design needs revisiting *before*
  R3. **G2 is not a kill**: it prices the surrogate, and a large G2 routes the basis work
  rather than stopping it.
- ⚠ **Neither gate tests whether holding `Φ` constant is the right modelling choice**,
  and R1.2 must not be read as having validated it. G1 finite-differences a constant-`Φ`
  model with `Φ` held constant on both sides, so it is exact by construction and blind to
  the question. G2 is the end-to-end price, against the model with no basis at all. For
  the load parameter the term is genuinely zero — the basis depends on the *training
  box*, not on the θ being differentiated — so this reservation binds on the material
  channel, where the training trajectories were run at one `μ`.

## 8. Risks

1. **A single-trajectory ensemble gives a false confirm.** A gravity release is nearly
   one-dimensional; a basis will look superb and mean nothing. §6's ensemble design is
   the mitigation and is not optional.
2. **Large rotation defeats linear POD** (§3). Expected on the cantilever; the plan
   treats it as information rather than failure.
3. **`ΦᵀAΦ` may dominate** (§4), making R1 slower than full-order at small `n`. Mitigated
   by not gating on wall time, and by measuring at two sizes.
4. **Snapshot storage.** `m` snapshots × `n_free` × 8 B — at 70 644 free DOF and 500
   snapshots that is 283 MB. Fine in-process; do not naively serialise per-Newton-iterate
   snapshots at that size.
5. **The indentation fixture is new code**, and a prescribed-displacement top face is a
   roller/Dirichlet configuration the existing `BoundaryConditions` supports
   (`roller_vertices` with a driven axis) — but it has not been exercised in a *dynamic*
   scene. Expect a rung-0.5 of just building and validating that fixture against the
   oracle before any basis work.

## 9. Decisions for the head engineer

1. **Fixture scope (§3)** — indentation only, cantilever only, or both? My
   recommendation is **both, as two verdicts**. This is the decision that most changes
   what R1 means.
2. **Does R1.0's kill condition bind on indentation alone**, as §7 proposes, or on both
   fixtures? Binding it to both makes a rotation failure kill the ladder, which I think
   is wrong.
3. **Sizes**: 3 000 free DOF for accuracy iteration plus ≥ 19 440 for one speedup
   datapoint, or accuracy only at 3 000?

## 13. R1.2 result — SPLIT. The algebra is exact; the gradient is not the state.

Gates: `sim/L0/soft/tests/reduced_gradient.rs`. Implementation:
`reduced/sensitivity.rs`.

### G1 — CONFIRM, by a margin of 350x

The reduced adjoint **is** the exact gradient of the reduced model. Worst disagreement
with a finite difference on `ReducedNewtonSolver::step`, across three cotangents and
three parameter channels (a directional derivative over all 867 θ components, individual
θ components, and the material `μ`): **2.85e-8**, against a 1e-5 gate.

That covers both parameter classes, which is why both were built rather than only the
load the plan named. The load's `∂r/∂θ = −e_k` is trivial and exercises only the adjoint
solve; the material's `∂r/∂p` is a genuinely assembled field, and it reuses
`assemble_material_residual_grad` — the same assembly the full-order forward sensitivity
and VJP use — so the reduced path has no second copy of the stress derivative to drift
from.

The structure that makes this cheap is worth stating: with `μ = Φλ_r` the reduced
gradient is the **full-order formula with `λ` replaced by `μ`**, for every parameter.
Nothing else changes.

### G2 — the finding. Gradient accuracy does NOT follow from state accuracy.

Three held-out trajectories, `r = 40`, worst case per cotangent:

| cotangent | rel. L2 err | adjoint projection err | cos(g_red, g_oracle) | ‖g_red‖/‖g_or‖ |
|---|---:|---:|---:|---:|
| `face-z` (in-family) | 0.246 | 0.138 | **0.972** | 0.90–0.91 |
| `node-z` (localized) | 0.642 | 0.486 | 0.766 | 0.76–0.77 |
| `Σx*` (out-of-family) | 0.816 | 0.583 | 0.692 | 0.31–0.32 |

against a **displacement error of 0.4–1.1 %** at the same steps. **The gradient is
21.5–186× less accurate than the state that produced it**, forming that ratio within each
trajectory rather than across them. The gate asserts the low end of it (`MIN_AMPLIFICATION`).

Three things pin the diagnosis:

1. **It is not the reduced trajectory.** Taking the reduced adjoint at the *oracle's*
   state changes the numbers in the fourth digit (0.8164 → 0.8163). The state error
   contributes essentially nothing.
2. **It is the adjoint's representability.** The adjoint field's own projection error
   tracks the gradient error across all three cotangents at a near-constant **1.27–1.79**
   over all nine (trajectory, cotangent) pairs — the same "Galerkin overhead over a
   projection floor" structure R1.1 found, one level up. A POD basis fitted to
   *displacement* snapshots was never asked to span an *adjoint*, and it does not.
3. **The shortfall is systematic.** `‖g_red‖ < ‖g_or‖` in **all 9 measured cases**, by
   0.31x to 0.91x. What motivates expecting this: Galerkin on an SPD `A` makes `μ` the
   best approximation to `λ` in the **energy** norm over `span(Φ)`, and a projection is
   never longer than what it projects — so `‖μ‖_A ≤ ‖λ‖_A`. ⚠ That is *not* a proof of
   what is measured here, which is a **Euclidean** norm of the gradient restricted to the
   loaded DOFs; the argument motivates the expectation, the 9/9 observation is the
   evidence. Either way the practical consequence is the one that matters: the reduced
   gradient **understates** sensitivity rather than overstating it. The gate asserts
   `ratio ≤ 1` because the other direction is the dangerous failure — a co-design loop
   reading an overstated sensitivity has no warning.

### What this means for a consumer, and why L2 was the wrong number to decide on

A surrogate gradient fails in two separable ways: too short costs a line search;
mis-aimed costs the descent property. Splitting them changes the verdict. On the
in-family objective the direction is **0.972 cosine** — perfectly usable for descent —
while its 25 % L2 error is almost entirely the magnitude shortfall. On the out-of-family
one, cosine 0.69 and magnitude 0.31 is a gradient that still descends but badly mis-scales.

So the validity domain R1 owes its consumers has an axis the recon and this plan both
missed: it constrains **what you differentiate**, not only where you evaluate. A reduced
model qualified for its parameter box is *not* thereby qualified for an arbitrary
objective on it.

### The prediction inside G2 that was wrong

The original two cotangents (`Σx*` and `node-z`) were chosen expecting **localization**
to be the hard axis
— a point-load adjoint being Green's-function-like where a displacement POD is smooth. It
is not the ranking that came out. `Σx*` is the worst, and its distinguishing feature is
*direction*: it applies unit forces along `x` and `y`, which the training ensemble (`z`
tractions only) never explored. `node-z` — in-family direction, out-of-family sharpness —
sits between. **What the basis has never been loaded along costs more than what it has
never resolved.** The `face-z` control is what makes that readable; without it the result
would have been dismissible as an exotic objective.

### Two knobs, one of which was measured not to matter

The first G1 pilot failed at 1.05e-5 on one θ component. Two candidate causes: the
solver's stopping criterion leaving residual slack, and an under-resolved FD step. An
`h`-sweep settled it — the error falls **monotonically as `1/h`** (1.05e-5 → 6.59e-7 →
2.30e-8 → 8.26e-10 over `h/‖θ‖` of 1e-6 → 1e-3), which is the signature of noise, not
truncation, and shows the difference converging *to* the analytic value.

A negative control then re-ran G1 at the **loose** 1e-6 tolerance with the widened step:
worst disagreement **9.68e-7**, still inside the gate. **The tolerance did none of the
work; the step size did all of it.** The tolerance stays tightened for margin (350× vs
10×), but the reasoning that first justified it was wrong and is recorded as such in the
test's own constant docs — a future reader tightening a tolerance to fix an FD should
know this knob was tried and measured not to be the one.

### What R1.2 does not answer

Whether a basis *refitted per parameter value* would close the gap — i.e. what the
dropped `dΦ/dp` term is actually worth. For the load channel that term is exactly zero
(the basis depends on the training box, not the evaluation θ), so the question binds only
on the material channel. It is not worth answering yet: G2 shows the error is dominated
by the adjoint's representability at a *fixed* parameter, which refitting would not
touch. The lead this result actually generates is **enriching `Φ` with adjoint snapshots**
(goal-oriented / dual-weighted-residual bases) — and that belongs with R3's basis work,
not behind it.

## 12. R1.1 result — CONFIRM. The Galerkin solve tracks the basis floor.

Gate: `sim/L0/soft/tests/reduced_newton_trajectory.rs`. Implementation:
`reduced/newton.rs`.

**Galerkin overhead is 1.45×–1.91× of the basis's own projection floor**, across 15
(trajectory, step) pairs at `r = 40`; 13 of 15 fall in 1.45–1.63. Worst absolute
trajectory error 1.14 %, against a floor of 0.79 %. **Reduced Newton needs exactly the
same iteration count as the oracle** (15 per trajectory) and converges every step.

So the error is dominated by basis truncation, which R1.0 already priced — the *solve*
adds well under a factor of two on top. That is what R1.1 existed to separate, and it is
why the gate asserts the **ratio** rather than the absolute error: asserting absolute
error would re-test R1.0 with more machinery.

### §4's cost arithmetic was wrong, in the optimistic direction this time

§4 predicted the reduced path would be "comparable or slower" at ~3 000 free DOF and
only win above ~10–20 k. Measured at **5 202** free DOF, the reduced trajectory is
**faster than the oracle** at every `r` tried:

| `r` | reduced | oracle | speedup |
|---:|---:|---:|---:|
| 10 | 162 ms | 412 ms | **2.55×** |
| 20 | 200 ms | 421 ms | **2.11×** |
| 40 | 288 ms | 437 ms | **1.52×** |

The crossover is below 5 202, not above 10 000. (These are post-fix; a first cut
rebuilt `Φ` from unit vectors inside `project_tangent` on every Newton iteration —
`O(n·r²)` of pure waste — and measured 307 ms at `r = 40`.)

⚠ That is the third time an estimate in this arc has missed (the first two are in the
recon's v1.7 and v1.6 entries). The pattern is consistent — **flop-count arithmetic over
sparse/dense mixes has not once predicted a measured ratio in this codebase.** Treat §4's
remaining projections as motivation for what to measure, never as a result.

### The bug this rung produced, and the API distinction behind it

The reduced solve failed to converge on step 1 of every trajectory. Cause: the residual
was projected with `PodBasis::project`, which computes `q = ΦᵀMu` — correct for the
*coordinates of a displacement* in a mass-orthonormal basis, wrong for a residual. The
Galerkin condition on a force is plain `Φᵀr = 0`. With the mass weight in, the effective
Jacobian became `ΦᵀMAΦ` while the code factored `ΦᵀAΦ`, so the search direction stopped
matching the residual it descended and the line search stalled immediately.

Displacements are vectors; residuals are covectors; they do not project the same way.
`PodBasis::project_covector` now exists for the second, with both docstrings pointing at
the distinction. **This is the second mass-weighting bug in this arc** — the first was
R1.0's re-weighted modes — which is a signal about where this code is easy to get wrong.

### Also worth knowing

`‖Φᵀr‖ / ‖r‖` at convergence measures **1e-7 to 1e-10**: the basis sees almost none of
the full residual, and the reduced state leaves a large full-order residual while its
displacement error stays near 1 %. Expected for Galerkin, and the reason displacement —
not residual — is the accuracy metric. Reported, not asserted; no useful bound is known.

⚠ The pilot's overhead figure (1.45–1.55×) was computed as worst-total over
worst-floor. The gate takes the worst *per-step* ratio, which is the stricter and correct
measure, and reads 1.91×. The gate bound is 2.5× — ~30 % headroom over the observed
maximum, because a gate that passes by 5 % flakes.

## 11. R1.0 result — CONFIRM, and one prediction falsified

Gate: `sim/L0/soft/tests/reduced_pod_basis.rs`. Implementation:
`sim/L0/soft/src/solver/backward_euler/reduced/`.

**Indentation fixture (the mission-relevant one): PASSES.** 5 202 free DOF, 48 training
trajectories, `r = 40` (a **130×** reduction, inside §2's ceiling of 104):
**held-out interpolation error 0.79 %**, against the 1 % gate. Extrapolation outside the
training box measures 12–39 % and is reported, not asserted — that is the validity
domain being real, exactly as §4c of the recon anticipated.

**Cantilever fixture: PASSES far more easily** — 0.14 % at `r = 20`, **0.018 % at
`r = 40`**, at a deflection of **97 % of span**.

### §3's central argument was wrong

§3 predicted the cantilever would be the hard case, because linear subspaces represent
rotation badly. It is the *easy* case, by two orders of magnitude, at near-total
deflection. The real discriminator is not rotation but **whether the load's spatial
support moves**: a localized bump at a new position is nearly orthogonal to one
elsewhere — the moving-feature problem POD shares with travelling waves — whereas large
but globally smooth rotation from a low-dimensional load family is easy.

⚠ **The comparison is confounded** and the conclusion is correspondingly limited: the
indentation ensemble varies 4 parameters and the cantilever 2, so what is demonstrated
is that *parameter dimensionality and moving support dominate rotation*, not that
rotation is free. Isolating rotation needs a matched-dimensionality ensemble, which was
not run.

**Consequence for R3**: modal derivatives (recon §4a) address geometric nonlinearity,
which is measurably **not** the bottleneck here. The bottleneck is parameter-space
coverage of a moving load, which points instead at local/piecewise bases, basis
interpolation over parameter space, or simply larger ensembles. R3's basis work should
be re-planned on that basis.

### Two traps caught, both now defended in the gate

1. **A one-parameter ensemble gives a false confirm.** Varying only load magnitude
   yields `r = 2` at 100.0000 % retained energy — a perfect-looking, meaningless basis,
   because a load ramp produces nearly proportional fields. **Effective rank tracks
   trajectory count, not snapshot count** (10 steps/trajectory measured 0.81 %, 5 steps
   0.79 %). Had R1.0 been run on a single trajectory, as the recon's original sketch
   allowed, it would have "confirmed" on a 2-dimensional artefact.
2. **Training energy does not predict generalisation.** The 99.99 %-energy criterion
   picks `r = 6`, where held-out error is 2.7 %. Reaching 1 % needs `r = 40` **and** a
   dense ensemble — 4 trajectories plateau at 33 %, 16 at 2.1 %, 48 at 0.79 %, and
   raising `r` alone never gets there. `r` is pinned in the gate rather than chosen by
   an energy threshold.

**A near-miss worth recording**: the first ensemble design used a *prescribed
displacement* indenter patch per §3. Moving the patch changes which DOFs are
Dirichlet-constrained, hence `free_dof_indices`, hence the snapshot width — trajectories
could not have shared a basis at all. Fixing the patch would have left depth as the only
parameter, i.e. trap 1. Loading through `theta` instead keeps every loaded vertex free
and the free-DOF map identical across the ensemble.

**A bug the pilot caught**: the mass-inner-product modes were re-weighted by `M^{1/2}`
after fitting, leaving them orthonormal in neither product; every mass projection
collapsed to zero and the error metric read an exact `1.000`. `ΦᵀMΦ = I` is now asserted
in the gate (measured `5.2e-12`), which is the invariant that would have caught it on
the first run.

## 10. Version history

- **v4 (2026-08-11)** — R1.2 built, piloted and gated; §13 records the split verdict, the
  §5/§7 gate amendment that preceded it, and the finding that gradient accuracy does not
  follow from state accuracy.
- **v3 (2026-08-11)** — R1.1 built and gated; §12 records CONFIRM (Galerkin overhead
  1.45–1.91× of the basis floor, identical iteration counts, faster than the oracle at
  5 202 free DOF) plus the `project`-vs-`project_covector` bug and the second miss by
  §4's cost arithmetic.
- **v2 (2026-08-11)** — R1.0 built, piloted and gated; §11 records CONFIRM on both
  fixtures and the falsification of §3's rotation argument.
- **v1 (2026-08-11)** — first issue. Adds three things the recon's R1 sketch did not
  carry: the fixture argument (§3), the `ΦᵀAΦ` cost arithmetic and the §4b amendment it
  forces (§4), and mandatory held-out trajectories (§6).
