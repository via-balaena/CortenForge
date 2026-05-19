# Candidate C — C.2 sweep falsification bookmark

> **STATUS — RESOLVED case A (C′.a shipped).** 2026-05-18
> LATE-EVENING.  C′.a ε-bisection in (0, 0.1) mm found a **narrow
> converging window at ε ≈ 0.075 mm** for cavity = 5 mm (4-sample
> sweep: 0.025/0.05 stall at r_norm ≈ 0.2, 0.075 converges 16/16,
> 0.1 stalls at 0.384).  Pinned at 0.075 mm + cap raised 4 → 5 mm
> (one notch above gated-A).  3 mm sanity gate (16/16, ZERO LM
> rescues) confirms gated-A baseline preserved.  C.3 6 mm probe
> gate at the pinned ε stalls at r_norm 0.536 → cap pinned to 5 mm
> (no point testing 7 mm).  See §9 below for the post-resolution
> data.  Original bookmark retained as audit trail.

> **STATUS — BOOKMARK** (historical, 2026-05-18 LATE-EVENING).
> Halted the F3 recon B C.2 + C.3 ladder mid-arc; C.1 sim-soft
> primitive + C.2 cf-device-design wire-up KEPT (dormant via
> `ε = 0.0`); cap scaffolding (8 mm) REVERTED to gated-A 4 mm
> baseline.  Recon next session picks the next mechanism per the
> three-session pattern
> ([[feedback-bookmark-when-surface-levers-exhaust]]).

**Predecessor docs**:

- `docs/CANDIDATE_C_SMOOTHED_CONTACT_SPEC.md` — the C.0 spec this
  arc was executing.  §5 falsifier matrix anticipated outcomes A
  (ship-it), B (partial), C (regression), D (worse-than-gated-A).
  Empirical sweep landed in a **new outcome the spec did not
  anticipate**: substantial improvement at every ε > 0 (so NOT
  outcomes C or D) but **non-monotonic** ε response + no ε
  converges 16/16 (so NOT outcomes A or B).
- `docs/CAVITY_5MM_CHATTERING_BOOKMARK.md` — the parent arc whose
  class-2 diagnosis this sweep was supposed to confirm.  Diagnosis
  is now PARTIALLY FALSIFIED: smoothing the contact Hessian helps
  significantly (r_norm floor 1.78 → 0.38 at ε = 0.1 mm) but does
  NOT close the gap to convergence + the response is non-monotonic
  past a small ε.
- `docs/F3_FALSIFICATION_BOOKMARK.md` — the grandparent arc + 3-
  failure-class mental model.  Class 2 (active-set discontinuity)
  remains a real contributor (smoothing measurably helps) but is
  **not the sole binding pathology** at cavity = 5 mm.
- `docs/F4_FALSIFICATION_POSTMORTEM.md` — sibling falsification
  postmortem (banks the F2 + F4 arcs; this C.2 falsification is the
  next entry in that lineage).

**Predecessor memory**: [[project-cavity-5mm-chattering-bookmark]],
[[project-f3-recon-a-gated-lm-shipped]],
[[project-f3-falsification-bookmark]],
[[project-f4-falsification-postmortem]],
[[project-cavity-inset-stall-bookmark]],
[[project-sl-4-arc-shipped]].

---

## TL;DR

The C.2 sliding-ramp sweep at cavity = 5 mm, layers 10+3 mm tested
ε ∈ {0.1, 0.25} mm before halting on a non-monotonic signal:

| ε (mm) | steps converged | Newton iters at step 0 | r_norm floor | stall mode | LM rescues |
|---|---|---|---|---|---|
| 0 (gated-A baseline) | 0/16 | 61 | **1.784** | Armijo stall | 1 (iter 57) |
| 0.1 | 0/16 | 126 | **0.384** | Armijo stall | 2 (iter 89, 119) |
| 0.25 | 0/16 | 150 (iter cap) | **0.753** | iter cap | 2 (iter 66, 137) |

Smoothing **measurably helps** (~4.6× r_norm reduction at ε = 0.1
vs gated-A baseline) but ε = 0.25 mm is **WORSE than ε = 0.1 mm**
— the C.0 spec predicted monotonic improvement with ε, so the
class-2-chattering-as-sole-binding-pathology mental model is
falsified at the (0.1, 0.25) ε interval.

**Hypotheses for the upstream cause** (any one of these — or
several — could be binding past the chattering envelope):

1. **SDF normal discontinuities** — C.0 spec §7 open question, never
   tested.  For mesh-derived SDFs (`mesh_sdf::SignedDistanceField`),
   the contact normal `n = ∂d/∂p` jumps at closest-triangle partition
   boundaries.  Smoothing the energy ramp leaves the normal-direction
   chattering unaddressed.  ε = 0.25's chronic non-PD pivots stuck at
   one tet (index 1671) for ~20 iters smell like this: one vertex's
   contact normal is chattering across a triangle-partition boundary
   even though the gap function is smoothed.
2. **Step-0 cold-start initial guess** — the sliding ramp starts at
   step 0 with `x_prev = rest pose`.  At cavity = 5 mm, the rest
   pose is far from the converged state; F4.3 already falsified a
   warmup ramp at this regime.  Maybe Newton's initial-state
   convexity envelope, not contact-Hessian regularity, is the
   binding constraint at step 0.  All sweep gates failed at step 0
   so we never tested k > 0.
3. **Band-widening backfire** — widening ε from 0.1 → 0.25 mm
   brings ~2.5× more pairs into the tapered `(d̂, d̂+ε)` regime.
   Tapered pairs have smaller per-pair Hessian contributions (the
   `r(τ) = 1 − 10τ³ + 15τ⁴ − 6τ⁵` ramp goes to 0 at the upper
   boundary, with `r' < 0` strictly).  If the active set becomes
   dominated by mid-taper pairs, the assembled Hessian's
   eigenstructure degrades in a new way that the C.0 spec's
   "smoother is strictly better" intuition misses.

**Hyp 1** is the most worrying — it predicts smoothing alone can
never close the gap, regardless of ε.  **Hyp 3** predicts a small
optimum at ε well below 0.25 mm (matches the data shape).
**Hyp 2** predicts every step-0 failure mode is more about Newton
initialization than the assembled contact tangent.

---

## 1. Sweep data — verbatim (cavity = 5 mm, layers 10+3 mm)

### ε = 0.1 mm

Final iter trace (Armijo stall iter 126):

```
sim-soft: LM seeded λ = 6.670275203126426e-3 at factor_and_solve_free at Newton iter 89 (free residual norm 4.8554482003308985e-1) (Llt non-PD pivot: NonPositivePivot { index: 13228 })
sim-soft: LM converged in 7 retries to λ = 6.670275203126425e3 at factor_and_solve_free at Newton iter 89
sim-soft: LM converged in 1 retries to λ = 8.337844003908032e3 at factor_and_solve_free at Newton iter 119 (free residual norm 3.8603761033334133e-1)
...
sim-soft: faer LU fallback fired at factor_and_solve_free at Newton iter 126 (free residual norm 3.843060848110731e-1) (Llt non-PD pivot: NonPositivePivot { index: 8535 })
Error: sliding ramp failed at step 0 — no converged step. Armijo line-search stalled at Newton iter 126, r_norm 3.843e-1
```

Pattern: progressive descent from r_norm ~77 (iter 0) to 0.384
(iter 126).  Two LM rescues at iter 89 + 119 (high λ ~6.67e3 +
8.34e3 — substantially indefinite).  Final pivot stuck at index
8535 for the last ~10 iters.

### ε = 0.25 mm

Final iter trace (Newton iter cap 150 hit):

```
sim-soft: LM seeded λ = 6.671007881875142e-3 at factor_and_solve_free at Newton iter 66 (free residual norm 1.097675003076348e0) (Llt non-PD pivot: NonPositivePivot { index: 7216 })
sim-soft: LM converged in 7 retries to λ = 6.671007881875141e3 at factor_and_solve_free at Newton iter 66
sim-soft: LM bumped λ = 8.338759852343926e3 at factor_and_solve_free at Newton iter 137 (free residual norm 8.007218057995591e-1)
...
sim-soft: faer LU fallback fired at factor_and_solve_free at Newton iter 148 (free residual norm 7.749290442921445e-1) (Llt non-PD pivot: NonPositivePivot { index: 1671 })
sim-soft: faer LU fallback fired at factor_and_solve_free at Newton iter 149 (free residual norm 7.534352288333006e-1) (Llt non-PD pivot: NonPositivePivot { index: 1671 })
Error: sliding ramp failed at step 0 — no converged step. Newton iter cap 150 reached without convergence, last r_norm 7.534e-1
```

Pattern: descent stalls at r_norm ~0.775 for ~13 iters (iter 137
post-LM through 148), then **sudden 0.022 drop at iter 149**
right before the iter cap.  Chronic non-PD pivot at index 1671 for
~20 iters (vs ε = 0.1's 8535) — different tet, different active-set
partition.

### Cross-comparison

| metric | gated-A baseline | ε = 0.1 mm | ε = 0.25 mm |
|---|---|---|---|
| r_norm floor | 1.784 | **0.384** | 0.753 |
| Newton iters | 61 | 126 | 150 (cap) |
| LM rescues | 1 (mild λ=6.67e-3) | 2 (stiff λ ~6.67e3 + 8.34e3) | 2 (mild + stiff) |
| stuck pivot | 33374 → 8535 | 8535 (sustained) | 1671 (sustained) |
| stall mode | Armijo | Armijo | iter cap (still descending) |

Two notable patterns the C.0 spec did NOT predict:

A. **LM rescue λ scales 10⁶× from ε = 0 to ε > 0** (6.67e-3 at gated
   A → 6.67e3 at smoothed).  The smoothing widens the active band,
   bringing more pairs in, increasing the indefiniteness Newton has
   to absorb when an active set flips.  LM has to work much harder.
B. **Stuck-pivot index changes with ε** (33374/8535 → 8535 → 1671).
   The chronic non-PD pivot is moving across the mesh as ε changes
   — different ε values pull different tets to the chattering edge.
   If a single problematic tet stayed across ε, hyp 1 would predict
   it; the moving pivot is more consistent with hyp 3.

---

## 2. Decisions banked from the sweep

### 2.1 Sweep halted early

The C.0 spec planned a 5-ε sweep (`{0.1, 0.25, 0.5, 1.0, 2.0} mm`)
to find the smallest converging ε.  After 2 samples the
non-monotonicity surfaced + the user (on bandwidth) opted for
bookmark + recon over collecting 3 more data points.  Rationale:
the (0.1, 0.25) signal is sharp enough to characterize "smoothing
alone is insufficient"; 3 more samples might narrow the optimum
within the smoothed family but would not change the headline
conclusion that **another mechanism is needed in addition to or
instead of smoothing**.

### 2.2 What's KEPT vs what's REVERTED

> **SUPERSEDED by §9** — this subsection captured the BOOKMARK-
> state revert (ε → 0.0, cap → 4 mm) as of the falsification
> commit `f0f22fb8`.  C′.a then shipped (case A) the same evening
> and re-pinned ε = 0.075 mm + cap = 5 mm.  The KEPT list below is
> still accurate; the REVERTED list captures the bookmark-state
> revert, not the current pin.

Per [[feedback-spec-falsified-revert-opt-in-keep-surface]] — the
1-line opt-in reverts, the surface plumbing survives:

**KEPT** (binding for next-recon candidates; still accurate):

- C.1 sim-soft `PenaltyRigidContact::with_params_and_smoothing` +
  `with_params_and_smoothing_and_interior_cutoff` constructors +
  the `smoothing_eps_m` field + the quintic-Hermite `quintic_ramp`
  helper + the 15 unit tests in `penalty_smoothing.rs`.
- The C.2 cf-device-design wire-up: `intruder_contact_at` +
  `intruder_contact_sliding_at` STILL route through C.1's
  smoothing constructors.  At `ε = 0.0` the C.1 fast-path
  short-circuit makes this bit-equal to pre-C.1 hard penalty.
- The sentinel test
  `insertion_contact_smoothing_eps_m_sentinel` (now pinning
  ε = 0.075 mm per §9.1 — was pinning ε = 0.0 at bookmark time).
- Gated A's `try_solve_impl` + `try_gated_factor_solve_armijo`
  (class-1 LM rescue, orthogonal to whatever C-recon picks).
- The `catch_unwind` belt-and-suspenders in
  `run_sliding_insertion_ramp` (class-3 Yeoh safety,
  belt-and-suspenders per
  [[feedback-workaround-removal-verification]]).

**REVERTED** (as of `f0f22fb8`, SUPERSEDED by §9.1 + §9.5):

- ~~`INSERTION_CONTACT_SMOOTHING_EPS_M`: 0.25e-3 → **0.0**~~.  Now
  re-pinned to **0.075e-3** per §9.1.
- ~~UI cavity slider cap: **8 mm → 4 mm**~~.  Now **5 mm** per
  §9.5 (cap raised one notch above gated-A baseline as a net
  product-knob win).

### 2.3 What is NOT reverted

- The C.0 spec itself stays in the repo as the design-of-record
  for any future ε-tuning work.  Annotated SHIPPED→PARTIAL-FALSE
  in a follow-up doc polish if needed.
- C.0 + C.1 commits are not amended or reverted at the git level.
  The C.2 commit `a55f48f0` stays; the revert is a NEW commit on
  top.  Audit trail preserved.

---

## 3. C-recon ladder candidates for the next session

In rough order of "how invasive" + "how likely to address the
falsification signal":

### Candidate C′ — re-spec smoothing (smallest deviation from C)

Stay inside the smoothed-penalty family but explore the parameter
+ polynomial space the C.0 spec foreclosed.  Cheapest if the
non-monotonicity is purely the band-widening backfire (hyp 3).

- **C′.a — ε bisection in (0, 0.1) mm**.  If ε = 0.1 mm beats
  ε = 0.25 mm, the optimum may be at ε ≪ 0.1 mm.  Test
  ε ∈ {0.025, 0.05, 0.075} mm.  Cost: 3 more sweep cycles
  (~5 min user time).
- **C′.b — alternative polynomial (cubic Hermite C¹ only)**.
  C.0 spec recommended quintic for Hessian continuity but allowed
  cubic.  Cubic is cheaper per evaluation + has different
  active-band geometry; falsification gate per spec §2.2.
- **C′.c — symmetric ramp `[d̂−ε, d̂+ε]`** (the bookmark §3
  variant C.0 spec §2.2 explicitly rejected as "polluting the
  active-side regime").  If hyp 3 is right, symmetric smoothing
  might give the active-side regime a softer pre-penalty entry
  that helps Newton's quadratic convergence near `d̂`.

### Candidate E — SDF normal smoothing (addresses hyp 1)

If hyp 1 binds — mesh-derived SDF normals discontinuous at
triangle-partition boundaries — the right mechanism is
normal-direction smoothing, not gap-function smoothing.  Approaches:

- **E.a — Gaussian convolution of the SDF**.  Replace
  `mesh_sdf::SignedDistanceField`'s exact distance with a kernel-
  blurred field; normals become C∞.  Computationally cheap (grid
  convolution at build time) but changes the geometric primitive
  the contact sees.
- **E.b — Per-vertex normal averaging at the contact site**.
  Less invasive: when computing `n` for a contact pair, average
  over the k nearest triangles.  Localizes the smoothing but adds
  per-query cost.
- **E.c — Move to `parry3d` TriMesh** with its own normal
  continuity guarantees.  Already used in cf-cast +
  cf-cast-cli per [[project-mesh-sdf-parry-accel-spec]]; would
  reuse that integration cost.

### Candidate F — Warmup at step 0 (addresses hyp 2)

If hyp 2 binds — step 0 is a cold-start Newton problem regardless
of contact-Hessian regularity — the right mechanism is a warmup
substep ramp at step 0.  F4.3 already falsified one warmup
approach but the failure mode there was different (introduced a
load class the solver couldn't handle).  A more conservative
warmup:

- **F.a — interference homotopy with κ ramp**: at step 0, start
  with κ = 1e1 (10× softer than `INSERTION_CONTACT_KAPPA = 1e3`)
  + ramp to 1e3 over N substeps.  Each substep starts from the
  prior substep's converged state.
- **F.b — slide-pose homotopy substep**: at step 0, interpolate
  the slide pose from identity to the target `t = 1/16` over N
  substeps.  Each substep is closer to the prior one than the
  rest pose is to the step-0 target.

### Candidate D — Mesh refinement (deferred, OK as fallback)

The C.0 spec's outcome-D fallback.  Halves the per-tet diameter
in the high-strain region; cuts the per-pair Hessian magnitude
(more pairs, each contributing less) which softens the active-set
discontinuity geometrically.  Higher implementation cost (~500
LOC mesh-refinement primitive); defer unless E + F also falsify.

### Recommended next-session order

1. **C′.a (ε bisection in (0, 0.1))** — cheapest probe of hyp 3.
   ~5 min user time + 3 small commits per the §3 C′.a cost estimate
   (3 rebuild-and-scrub cycles at ε ∈ {0.025, 0.05, 0.075} mm).
   Falsifies/confirms the band-widening hypothesis directly.
2. **E.b (per-vertex normal averaging)** — if C′.a doesn't find
   16/16 ε, this is the next-most-targeted probe of hyp 1.
3. **F.a (κ ramp at step 0)** — if E.b also doesn't help.
4. **Candidate D** — last resort fallback.

The user picks per their bandwidth + risk appetite.  Pre-recon
session, **none** of these has a binding theoretical argument
better than the others — the sweep data + 3 hypotheses are the
input; the recon picks based on user judgment + cost estimate.

---

## 4. What this bookmark does NOT do

- Pre-commit to any candidate above.  Recon picks empirically
  with the bookmark + 3 hypotheses as input.
- Revert the C.1 sim-soft primitive.  C.1's surface stays as a
  building block; future candidates can use it (e.g., C′ + E
  composed: smoothed contact at small ε + smoothed SDF normals).
- Revert the C.2 cf-device-design wire-up.  The const +
  constructor routings stay; a future recon flips
  `INSERTION_CONTACT_SMOOTHING_EPS_M` + nothing else.
- Touch gated A's `try_solve_impl` / `try_gated_factor_solve_armijo`.
  Class-1 LM rescue still fires + still works; orthogonal to
  whichever C-recon mechanism comes next.
- Modify the `catch_unwind` belt-and-suspenders in
  `run_sliding_insertion_ramp`.  Class-3 Yeoh safety stays.
- Speculate on the C-recon outcome.  Three hypotheses; recon
  picks one based on user judgment + cost estimate.

---

## 5. Falsification mechanics — what the spec missed

Two structural lessons banked from this arc:

### 5.1 "Smoothing strictly improves Hessian regularity" is wrong

C.0 spec §2.2 reasoning:

> The polynomial taper `(1, 0, 0) → (0, 0, 0)` makes the per-pair
> Hessian C² at the boundary.  Sum of C² functions is C².
> Therefore the assembled `H_contact(x) = Σ H_pair(x)` is C².

The argument is correct **per pair** but the assembled tangent's
**eigenstructure** depends on more than the per-pair smoothness:

- **Magnitude** of the per-pair contribution.  Tapered pairs (in
  `(d̂, d̂+ε)`) contribute smaller-magnitude Hessian blocks than
  full-penalty pairs (`sd < d̂`).  If the active set is dominated
  by tapered pairs (which is what large ε produces), the assembled
  tangent has many small-magnitude rank-1 contributions that don't
  add to a well-conditioned positive-definite block.
- **Direction** of the per-pair contribution.  All `H_pair = κ ·
  ramp(sd) · n ⊗ n` contributions are rank-1.  Their sum is
  rank-up-to-3 per vertex.  Smoothing the magnitude doesn't change
  the rank structure or the alignment of `n` across pairs.

The C.0 spec implicitly assumed "C² assembled tangent → good Newton
convergence".  The empirical sweep shows that **C² regularity
without conditioning bound is insufficient**.  This is a class of
spec mistake worth banking as a pattern: regularity arguments
need to compose with conditioning arguments, not just continuity.

### 5.2 "ε is a free knob that can be tuned monotonically" is wrong

C.0 spec §6 C.2 sweep protocol assumed the optimum ε would be
"find the smallest ε that converges 16/16".  This presupposed a
monotonic response — larger ε strictly better up to where the
penetration tolerance bites.  Empirically the response has a
LOCAL OPTIMUM somewhere below ε = 0.25 mm (since 0.1 beats 0.25).
The protocol should have anticipated bisection rather than
monotonic search.

Bank for future recon specs: **never assume monotonic response
to a smoothing parameter** without an explicit conditioning
argument that proves it.  Default to bisection-friendly sweep
protocols.

---

## 6. Anchors

**Commits on `sim-arc/sl-4-intruder-render`** (all local-only, not
pushed):

- `9a1433b8` + `61b31f9b` + `59997c41` + `c16fe375` + `a1c26105` —
  gated-A spec + impl + outcome-B follow-up + cold-read polish.
- `edba9f48` + `f1dc082f` + `7a52d2a1` — C.0 spec + cold-read
  polish + §8 placeholder fill.
- `002a3372` + `e99cafc2` — C.1 sim-soft primitive + cold-read
  polish.
- `a55f48f0` — C.2 cf-device-design opt-in (initial ε = 0.5 mm
  pending sweep) — the commit this bookmark closes the loop on.
- `f0f22fb8` — C.2 sweep falsification revert + this bookmark.
- `69d9f6dc` — C.2 sweep falsification bookmark cold-read polish
  (5 findings pass-1 + pass-2).
- `bb6292a5` — C′.a sweep scaffolding (cap 4 → 8 mm).
- `bfbb4569` — C′.a case A ship (ε = 0.075 mm + cap 4 → 5 mm + 3
  mm baseline preserved + §9 added).
- (this commit) — case A cold-read polish (5 findings pass-1 +
  pass-2).

**Docs**:

- `docs/CANDIDATE_C_SMOOTHED_CONTACT_SPEC.md` (~867 lines, C.0
  recon spec — partially falsified, retained as audit trail).
- `docs/CAVITY_5MM_CHATTERING_BOOKMARK.md` (~250 lines, parent
  arc bookmark — class-2 diagnosis now partially falsified).
- `docs/F3_FALSIFICATION_BOOKMARK.md` (~grandparent arc).
- `docs/F4_FALSIFICATION_POSTMORTEM.md` — sibling falsification
  postmortem (banks the F2 + F4 falsification arcs; this C.2
  falsification is the next entry in that lineage).

**Memory pointers** (project memories; feedback memories are cited
inline above where they apply):

- [[project-cavity-5mm-chattering-bookmark]] — direct predecessor.
- [[project-f3-recon-a-gated-lm-shipped]] — sibling arc that
  generated this arc's input data (the cavity = 5 mm Gate B trace).
- [[project-f3-falsification-bookmark]] — grandparent.
- [[project-f4-falsification-postmortem]] — sibling falsification
  pattern.
- [[project-cavity-inset-stall-bookmark]] — original cavity-inset
  stall arc this whole F3+F4+C lineage hangs from.
- [[project-sl-4-arc-shipped]] — SL.4 sliding-intruder rendering
  arc baseline the visual gates ride on.

**Code sites — preserved C.2 wire-up (DO NOT revert)**:

- `tools/cf-device-design/src/insertion_sim.rs:941-973` —
  `INSERTION_CONTACT_SMOOTHING_EPS_M` const (now `0.0`) + the
  falsification-evidence docstring.
- `tools/cf-device-design/src/insertion_sim.rs:~991` —
  `intruder_contact_at` routed through
  `with_params_and_smoothing(...)` (4-arg, dormant at ε = 0.0).
- `tools/cf-device-design/src/insertion_sim.rs:~2335` —
  `intruder_contact_sliding_at` routed through
  `with_params_and_smoothing_and_interior_cutoff(...)` (5-arg,
  dormant at ε = 0.0).
- `tools/cf-device-design/src/insertion_sim.rs` `mod tests` —
  `insertion_contact_smoothing_eps_m_sentinel` pins ε = 0.0 + the
  sweep falsification table.

**Code sites — preserved C.1 sim-soft surface (DO NOT revert)**:

- `sim/L0/soft/src/contact/penalty.rs:91-120` — `quintic_ramp`
  free helper (module scope, NOT in the impl).
- `sim/L0/soft/src/contact/penalty.rs:136-197` —
  `PenaltyRigidContact` struct + `smoothing_eps_m` field.
- `sim/L0/soft/src/contact/penalty.rs:215-517` — `impl
  PenaltyRigidContact` with **5 constructors** (`new`, `with_params`,
  `with_params_and_interior_cutoff`, `with_params_and_smoothing`,
  `with_params_and_smoothing_and_interior_cutoff`) + per-pair
  helpers (`pair_is_active`, `pair_contribution`).
- `sim/L0/soft/tests/penalty_smoothing.rs` — 15 unit tests pinning
  the bit-equal-when-dormant contract + Hessian continuity gates.

**Code sites — REVERTED scaffolding**:

- `tools/cf-device-design/src/main.rs:225-280` —
  `CavityState::inset_slider_range_m` returns `(0.0, 0.004)` again.
- `tools/cf-device-design/src/main.rs:~2090-2115` — egui label
  reads "capped at 4 mm" again.
- `tools/cf-device-design/src/main.rs:~2980-3015` — sentinel test
  `cavity_inset_slider_range_zero_to_four_mm` asserts max_m =
  0.004 again.

---

## 7. Implementation expectations for the next session

- Per [[feedback-autonomous-architecture]] — drive the C-recon
  autonomously.  Pick a candidate from §3, implement,
  measure-or-revert.  Flag only big departures.
- Per [[feedback-bookmark-when-surface-levers-exhaust]] —
  three-session pattern.  This bookmark is the bookmark session;
  next is recon (pick a candidate); the session after that is
  implementation.  If recon turns up clear no-go on the picked
  candidate, drop it + pick the next.
- Per [[feedback-implement-measure-revert-pattern]] — for the
  cheap C′ candidates (≲ 200 LOC), just implement → measure →
  keep-or-revert.  The empirical data from a 1-hour cycle is more
  valuable than 3 hours of theoretical recon for these.
- Per [[feedback-cold-read-review-post-ship]] +
  [[feedback-cold-read-two-passes-for-non-trivial-diffs]] —
  cold-read on every non-trivial commit.  The C-recon bookmarks
  + implementations are exactly the parallel-surfaces + spec-doc
  diff pattern that pass-2 catches structural findings in.
- Per [[feedback-strip-the-knob-when-default-works]] — if a
  C-recon candidate ships, do NOT expose ε / κ / mesh-refinement-
  level / etc as user-facing sliders unless empirical data shows
  multi-modal response.  Const-only by default.

---

## 8. Falsifier matrix (when next recon doesn't work, what we learn)

If the C-recon next session picks a candidate + it fails, the
falsifier outcomes:

| Candidate | Fails how | What it means | Next |
|---|---|---|---|
| C′.a (ε bisection) | No ε in (0, 0.1) mm converges 16/16 | Smoothing alone is insufficient at any ε in the explored band.  Hyp 3 falsified (or local optimum is below numerical noise floor). | Move to E (SDF normal) or F (warmup). |
| C′.b (cubic Hermite) | Cubic chatters same as quintic or worse | C¹ vs C² polynomial doesn't matter for this geometry; the binding pathology is bigger than polynomial order. | Move to E or F. |
| C′.c (symmetric ramp) | Symmetric chatters worse than one-sided | Active-side regime is fine; the bookmark §3 instinct to keep one-sided was correct. | Move to E or F. |
| E.a (Gaussian SDF) | Smooth SDF still chatters at 5 mm | Hyp 1 falsified; normal-direction discontinuity wasn't the binding contributor.  Or the kernel width is wrong. | Try E.b (per-vertex avg) or F. |
| E.b (per-vertex normal avg) | Avg'd normal still chatters | Hyp 1 falsified or insufficient.  Combine with C′? | Move to F. |
| F.a (κ ramp warmup) | Warmup substeps don't help | Hyp 2 falsified; step 0 isn't a cold-start issue.  Probably outcome D / mesh refinement is next. | Move to D (mesh refinement) or escalate to a different solver entirely (sub-step time integration, IPC barrier). |

If ALL of C′ + E + F fail, candidate D (mesh refinement) is the
last in-spec option; past that the recon escalates to a different
solver architecture entirely.

---

## 9. §C′.a-RESOLVED — case A shipped 2026-05-18 LATE-EVENING

Per [[feedback-implement-measure-revert-pattern]] C′.a was
implement → measure → keep (case A) — no separate recon session
needed because the ε edit was a 1-line change + the data answered
the hypothesis directly.

### 9.1 Full C.2 + C′.a sweep table

Cavity = 5 mm, layers 10 + 3 mm, sock_over_capsule.cleaned.stl,
sliding-intruder ramp 1 → 16:

| ε (mm) | steps converged | r_norm floor | stall mode | LM rescues | stuck pivot |
|---|---|---|---|---|---|
| 0 (gated-A baseline) | 0/16 | 1.784 | Armijo iter 61 | 1 mild λ ≈ 6.67e-3 | 33374 → 8535 |
| 0.025 (C′.a) | 0/16 | 0.231 | Armijo iter 108 | 3 stiff λ ≈ 6.67e3 | 8113 (sustained) |
| 0.05 (C′.a) | 0/16 | 0.200 | Armijo iter 147 | 4 moderate λ ≈ 5.21e3 | 13311 → 19203 → 19060 |
| **0.075 (C′.a)** | **16/16** | **converges seated 83.35 mm** | — | **0** | — |
| 0.1 (C.2) | 0/16 | 0.384 | Armijo iter 126 | 2 stiff λ ≈ 8.34e3 | 8535 (sustained) |
| 0.25 (C.2) | 0/16 | 0.753 | iter cap 150 | 2 stiff λ ≈ 6.67e3+ | 1671 (sustained) |

### 9.2 Mental model update

The response is **U-shaped with a narrow converging window at
ε ≈ 0.075 mm**.  Both sides of the optimum:

- **Too small** (ε ≤ 0.05 mm): smoothing window doesn't cover
  enough chattering pairs to suppress the active-set
  discontinuity; r_norm floor plateaus around ≈ 0.2 (0.025 →
  0.231, 0.05 → 0.200 — essentially flat).
- **Too large** (ε ≥ 0.1 mm): band-widening backfire dominates
  (hyp 3 confirmed) — too many pairs in the tapered regime
  degrades the assembled tangent's eigenstructure, and the
  r_norm floor climbs sharply (0.1 → 0.384, 0.25 → 0.753).

Hypothesis 3 (band-widening backfire) is **CONFIRMED** as a real
contributor on the upper side of the optimum.  Hypotheses 1
(SDF normal discontinuity) + 2 (step-0 cold-start) remain
uninvestigated but the chosen ε achieves 16/16 without engaging
LM at all, so they are not BINDING at cavity ≤ 5 mm.

### 9.3 Sanity gate (cavity = 3 mm) result

ε = 0.075 mm at cavity = 3 mm: **16/16 converged**, seated
83.35 mm, ZERO LM rescues + ZERO Yeoh failures + ZERO panics.
46 LU fallback firings (handled cleanly without escalation).
Gated-A 3 mm baseline preserved — gated A's class-1 LM rescue
mechanism and smoothed contact compose orthogonally.

### 9.4 C.3 probe gate (cavity = 6 mm) result

ε = 0.075 mm at cavity = 6 mm: **0/16**, Newton iter cap 150
hit, r_norm 0.536. Two LM rescues (iter 81 mild λ ≈ 7.91e3,
iter 139 bumped to λ ≈ 9.89e3 with new stuck pivot at index
25077).  7 mm gate skipped — strictly worse than 6 mm by the
chattering-envelope monotonicity argument.

The C′.a-pinned ε **does not generalize past 5 mm**.  Cavity > 5
mm needs either:

- A ε that varies with cavity (would need a per-cavity sweep or a
  UI slider — DO NOT add unless empirical evidence shows
  multi-modal need per [[feedback-strip-the-knob-when-default-works]]).
- A composed mechanism — e.g., smoothed contact + SDF normal
  smoothing per hyp 1, or smoothed contact + step-0 warmup per
  hyp 2.

### 9.5 Cap decision

Cap raised **4 → 5 mm** (one notch above gated-A baseline).
Three-surface mirror:

- `CavityState::inset_slider_range_m` returns `(0.0, 0.005)` +
  docstring rewritten with the C′.a + C.3 rationale.
- egui label in `render_cavity_section` reads "capped at 5 mm —
  highest cavity the C′.a-pinned ε converges 16/16 at; C.3 6 mm
  probe gate stalled at r_norm 0.536".
- Sentinel test renamed
  `cavity_inset_slider_range_zero_to_five_mm` + asserts 0.005 +
  comment carries the C′.a ship rationale + C.3 stall.

The cap **lower-bounds the chattering envelope** at cavity 6 mm
+ stays well inside Yeoh material validity (which doesn't bite
until ≥ 8 mm).  Future C-recon mechanisms can re-raise the cap
up to ~7 mm before material validity has to be re-evaluated.

### 9.6 What stayed dormant + can be re-activated cheaply

- C′.b (cubic Hermite polynomial) — orthogonal to C′.a's ε
  bisection; not informative now that ε = 0.075 mm works.
- C′.c (symmetric ramp) — same.
- E candidates (SDF normal smoothing) — would extend the
  converging window past cavity 5 mm if hyp 1 also binds there.
- F candidates (step-0 warmup) — would extend past 5 mm if hyp
  2 binds.
- D (mesh refinement) — geometric attack on the chattering
  envelope; still the deferred last-resort fallback.

The bookmark §3 + §8 ladder + falsifier matrix are still valid
guides for any of those if a future product requirement pushes
cavity past 5 mm.
