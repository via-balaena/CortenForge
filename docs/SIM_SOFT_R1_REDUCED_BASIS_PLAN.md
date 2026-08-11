# R1 — Linear Reduced Basis: scope and plan

**Status**: 2026-08-11, v2. **R1.0 COMPLETE — CONFIRM.** R1.1 unblocked. §11 records
the result, including a prediction of mine it falsified. Parent: `docs/SIM_SOFT_REALTIME_RECON.md`
§7 rung R1. Predecessor R0 landed (`ecf4cfef`); nothing else is queued ahead of this.

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

- **Gate**: reduced gradient vs the oracle's, to the crate's existing gradcheck
  tolerance; plus a finite-difference check on the reduced model in its own right.

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
- **R1.2 kill**: gradient error exceeds gradcheck tolerance with `Φ` constant → the
  constant-basis decision (recon §6) is wrong and the whole differentiability design
  needs revisiting *before* R3, not after.

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

- **v2 (2026-08-11)** — R1.0 built, piloted and gated; §11 records CONFIRM on both
  fixtures and the falsification of §3's rotation argument.
- **v1 (2026-08-11)** — first issue. Adds three things the recon's R1 sketch did not
  carry: the fixture argument (§3), the `ΦᵀAΦ` cost arithmetic and the §4b amendment it
  forces (§4), and mandatory held-out trajectories (§6).
