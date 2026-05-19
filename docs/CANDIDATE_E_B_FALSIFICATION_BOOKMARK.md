# Candidate E.b — falsification bookmark (case E)

> **STATUS — RESOLVED case E (E.b consts reverted to disabled).**
> 2026-05-19.  The cavity = 6 mm sweep at `(k=7, r ∈ {0.5, 1.0,
> 2.0} mm)` cracked the chattering envelope at step 0 (vs the
> `(1, 0)` baseline that stalled at `r_norm 0.536` in C.3) but ALL
> three samples hit a Yeoh material-validity wall at step 2 —
> seated 5.21 mm of 83.35 mm only.  Sanity gate at the candidate
> pin `(7, 1.0 mm)` confirmed the regression as **case E** per the
> spec's falsifier matrix: cavity = 3 mm holds 16/16 clean but
> cavity = 5 mm regresses to 1/16 (step 2 Yeoh wall at tet 3258,
> `max_stretch_deviation = 1.002`).  E.b's averaging shifts the
> equilibrium toward the Yeoh validity bound at every cavity — at
> 3 mm there's margin to absorb it, at 5 mm there isn't.  The
> cavity = 6 mm "1/16 win" was Yeoh wall surfacing, not E.b
> succeeding.
>
> Consts reverted to disabled state `(k=1, r=0)` per
> [[feedback-spec-falsified-revert-opt-in-keep-surface]]; **all
> surface plumbing KEPT** (sim-soft constructors + helper + 9
> unit tests + cf-device-design routing + sentinel test).  Cap
> scaffolding 6 → 5 mm reverted.  **Yeoh wall is the new binding
> constraint at cavity > 5 mm** — superseding the chattering
> diagnosis the C-arc was pursuing.  Recon-next-session at
> `docs/CAVITY_6MM_YEOH_WALL_BOOKMARK.md` (TBD; this commit only
> documents the falsification).

**Predecessor docs**:

- `docs/CANDIDATE_E_B_NORMAL_AVERAGING_SPEC.md` — the E.b spec
  (§5 falsifier matrix anticipated case E; this falsification
  data lands in §9 case-E branch).
- `docs/CANDIDATE_C_SWEEP_FALSIFICATION_BOOKMARK.md` — parent
  C-arc bookmark with the 3 hypotheses; hyp 1 (SDF normal
  discontinuity) is what E.b tested.
- `docs/CAVITY_5MM_CHATTERING_BOOKMARK.md` — grandparent
  bookmark; class-2 chattering diagnosis at cavity = 5 mm is now
  known to be PARTIALLY-RESOLVED (C′.a fixes 5 mm), and cavity
  > 5 mm is now known to be Yeoh-wall-binding, NOT chattering-
  binding.

**Predecessor memory**: [[project-c-prime-a-shipped]],
[[project-c2-sweep-falsification-bookmark]],
[[project-cavity-5mm-chattering-bookmark]],
[[project-f3-recon-a-gated-lm-shipped]],
[[project-f3-falsification-bookmark]],
[[project-f4-falsification-postmortem]],
[[project-sl-4-arc-shipped]].

---

## TL;DR

**E.b mechanism**: at per-pair contact query, replace
`n = prim.grad(p)` with `n_avg(p) = normalize(prim.grad(p) +
Σ prim.grad(p ± r·e_{x,y,z}))` over 6 axis-aligned offset samples
at radius `r`.  Targets hyp 1 (SDF normal slope discontinuity at
grid cell faces in `GridSdf::gradient_clamped`).

**Sweep result at cavity = 6 mm** (layers 10+3 mm,
sock_over_capsule.cleaned.stl):

| `(k, r)` | step 1 | F (N) | iters | step 2 outcome |
|---|---|---|---|---|
| (1, 0) baseline | 0/16 r_norm 0.536 chattering stall | — | 150 cap | — |
| (7, 0.5 mm) | 1/16 seated 5.21 mm | 15.23 | 34 | Yeoh tet 1458, max_stretch 1.141 |
| (7, 1.0 mm) | 1/16 seated 5.21 mm | 8.1 | ~26 | Yeoh tet 3206, max_stretch 1.051 |
| (7, 2.0 mm) | 1/16 seated 5.21 mm | 8.15 | 25 | Yeoh tet (likely) |

**Sanity gate** at the candidate pin `(7, 1.0 mm)`:

| cavity | result | binding constraint |
|---|---|---|
| 3 mm | 16/16 ✓ | none (clean F-d, peak ~3.1 N) |
| 5 mm | 1/16 ✗ regression | Yeoh tet 3258, max_stretch 1.002 |
| 6 mm | 1/16 | Yeoh tet 3206, max_stretch 1.051 |

The 5 mm regression makes this **case E** per spec §5.  3 mm
holds because there's enough margin between the perturbed
equilibrium and the Yeoh bound; 5 mm sits right at the bound and
the perturbation pushes one tet over.

---

## 1. What we learned (3 structural findings)

### 1.1 The "cavity 6 mm 1/16 win" was illusory

The C.3 cavity = 6 mm probe gate at C′.a's ε stalled at step 0
with `r_norm 0.536` — a chattering pathology.  Three hypotheses
in the C.2 falsification bookmark predicted what was binding;
hyp 1 (SDF normal discontinuity) was the candidate E.b tested.

E.b at `(7, r)` for r ∈ {0.5, 1.0, 2.0} mm successfully passes
step 0 at cavity = 6 mm — confirms hyp 1 IS a contributor on the
chattering side.  But step 1 converges to a state at d = 5.21 mm
(1/16) and step 2 hits the Yeoh wall.

**The Yeoh wall is what would have fired anyway** if C′.a alone
could have gotten past step 0.  The "1/16 progress" at cavity =
6 mm is not new product capability — it's just unmasking a
preexisting constraint that the chattering bottleneck was hiding.

### 1.2 E.b shifts equilibrium toward the Yeoh bound at every cavity

The 5 mm sanity gate failing is the smoking gun.  At cavity = 5
mm:

- C′.a alone (k=1, r=0): 16/16 ZERO LM, no Yeoh panic across
  16 steps (so per-tet max_stretch_deviation stayed below
  1.000 the whole ramp).
- E.b (7, 1.0 mm): 1/16, Yeoh wall at step 2 tet 3258
  max_stretch = 1.002 (barely over the bound).

E.b's averaged normal isn't a strict superset of `prim.grad(p)` —
it perturbs the contact force direction slightly even on smooth
SDF regions where `prim.grad` was already a good direction.
That perturbation shifts the equilibrium state in a way that, at
cavity = 5 mm where the existing baseline has tight margin, pushes
one tet just past the strain bound.

This is a structural finding banked for the next recon: **any
contact-side smoothing mechanism whose default-on is intended
must pass a "doesn't shift max_stretch_deviation in already-
working regimes" gate**.  E.b fails that gate.

### 1.3 Yeoh wall at step 2 is the NEW binding constraint at cavity > 5 mm

C.2 falsification bookmark §1 enumerated three hypotheses for
what binds at cavity > 5 mm (after C′.a partially-resolved the
class-2 chattering at cavity ≤ 5 mm):

1. SDF normal discontinuity (hyp 1) — E.b tested + CONFIRMED as
   a contributor, but bandwidth gain absorbed by Yeoh wall.
2. Step-0 cold-start (hyp 2) — F.a candidate, untested.
3. Band-widening backfire (hyp 3) — C′.a confirmed on upper-ε
   side at cavity = 5 mm.

E.b's data reframes the cavity > 5 mm problem: it's NO LONGER a
chattering/solver problem (E.b breaks through the chattering at
step 0).  It's a **material-validity/geometry problem** —
deeper insertion creates Yeoh-strain at some tet that exceeds
validity at the current 5 mm slide-step granularity.

The right next sub-arc is the **Yeoh wall** itself, not another
solver-side smoothing mechanism.  See §3 for the recon ladder.

---

## 2. What's KEPT vs REVERTED

Per [[feedback-spec-falsified-revert-opt-in-keep-surface]] — the
1-line opt-in reverts, the surface plumbing survives:

**KEPT** (binding for next-recon candidates; still accurate):

- sim-soft `PenaltyRigidContact::with_params_and_smoothing_and_normal_averaging`
  + `..._and_interior_cutoff` constructors.
- sim-soft `normal_avg_k` + `normal_avg_radius_m` fields.
- sim-soft `averaged_normal` private helper with the `k ∈ {1, 7}`
  assertion + the `r > 0 iff k > 1` invariant.
- sim-soft `tests/penalty_normal_averaging.rs` (9 unit tests:
  bit-equal-when-disabled at `k=1` regardless of `r`, k=7 plane
  preserves constant normal, k=7 sphere preserves radial
  symmetry, 4 constructor-validation panics, with-cutoff happy
  path).
- cf-device-design `intruder_contact_at` + `intruder_contact_sliding_at`
  ROUTING through the new constructors.  At `(k=1, r=0)` the
  helper short-circuits to `prim.grad(p)` bit-equal pre-E.b.
- cf-device-design `INSERTION_CONTACT_NORMAL_AVG_K` +
  `INSERTION_CONTACT_NORMAL_AVG_RADIUS_M` consts (now pinned at
  `(1, 0)` — disabled state, bit-equal-when-dormant).
- cf-device-design `insertion_contact_normal_avg_sentinel` test
  (still asserts `(1, 0)` + bounds + composition invariant).
- C′.a-pinned `INSERTION_CONTACT_SMOOTHING_EPS_M = 0.075e-3` +
  the C′.a 3-surface mirror (orthogonal axis; unaffected).
- Gated-A class-1 LM rescue (orthogonal).
- `catch_unwind` belt-and-suspenders in
  `run_sliding_insertion_ramp` (class-3 Yeoh safety — Yeoh wall
  panic was caught + UI rendered the partial-step report).
- `docs/CANDIDATE_E_B_NORMAL_AVERAGING_SPEC.md` (spec stays as
  audit trail; §5 falsifier matrix's case-E branch matched
  empirically).

**REVERTED**:

- `INSERTION_CONTACT_NORMAL_AVG_K`: 7 → **1**.
- `INSERTION_CONTACT_NORMAL_AVG_RADIUS_M`: 1.0e-3 → **0.0**.
- cavity-cap scaffolding 6 → **5 mm** (3-surface mirror in
  `tools/cf-device-design/src/main.rs`: `inset_slider_range_m`
  + egui label + sentinel test renamed back to
  `cavity_inset_slider_range_zero_to_five_mm`).

---

## 3. Next-session recon — Yeoh wall sub-arc

The cavity > 5 mm problem is now known to be **Yeoh wall at step 2**,
not chattering.  The right recon ladder reorients accordingly:

### 3.1 Hypothesis ranking for the Yeoh wall

In rough order of how cheaply each is testable:

**H1 — Slide-step-size artifact**.  `DEFAULT_SLIDE_STEP_SIZE_M =
5 mm` (sliding step length).  At cavity ≥ 5 mm step 2's slide
brings the intruder from 5.21 mm to ~10.4 mm of inserted depth
in a single quasi-static step — local strain at the contact
zone jumps by 5 mm of geometry mismatch.  If we halve the slide
step (2.5 mm, 32 substeps), each step's strain delta is smaller
and might stay within Yeoh validity.

**H2 — Mesh resolution**.  BCC lattice cell size = 4 mm.  The
Yeoh strain is computed per-tet; at coarse mesh density a single
tet absorbs the full local strain.  Finer mesh would distribute
the strain across more tets, reducing per-tet max_stretch.
~500 LOC mesh-refinement primitive (per C-arc spec §3 candidate
D).

**H3 — True material limit**.  Even with smaller slide steps +
finer mesh, max_stretch_deviation might genuinely exceed 1.000
at deeper cavity values because the engineered geometry
literally requires that strain.  Then the cavity cap is
material-bound, not solver-bound — and 5 mm is the right
permanent cap.

### 3.2 Recommended next-session order

1. **H1 first** (~30 min user time, ≲ 50 LOC).  Reduce
   `DEFAULT_SLIDE_STEP_SIZE_M` to 2.5 mm (or add a sweep over
   {5.0, 2.5, 1.0} mm).  Re-test cavity = 6 mm + cavity = 5 mm.
   If smaller steps avoid Yeoh wall at 5 mm but not 6 mm,
   confirms H1 partial; if avoids both, H1 binding; if avoids
   neither, H1 falsified → H2.
2. **H2 if H1 falsifies** (~500 LOC).  Mesh refinement is the
   bigger investment; defer until H1's verdict is in.
3. **H3 if H1 + H2 both fail**.  Accept the 5 mm cap as a
   material-bound permanent posture; document the constraint;
   move to next product priority.

### 3.3 Composition with E.b

If H1 succeeds (smaller slide steps make cavity > 5 mm work
under C′.a alone), E.b stays disabled — its only function was
to break the cavity-6mm step-0 chattering, and smaller slide
steps may make that step easier directly (less mismatch per
step → less chattering pressure on the active set).

If H1 partially succeeds (e.g., cavity 5 mm works at 2.5 mm
slide step under C′.a alone, but cavity 6 mm still chatters at
step 0), THEN E.b might compose orthogonally — but only with a
**cavity-conditional** opt-in (per spec §7.3), NOT default-on,
because the case-E regression at cavity = 5 mm remains a
concern.

---

## 4. What this bookmark does NOT do

- Pre-commit to any of H1 / H2 / H3.  Recon picks empirically.
- Revert the sim-soft E.b surface.  All constructors + helper +
  tests + cf-device-design routing stay; only the 2 consts +
  cap scaffolding revert.
- Revert C′.a's pin (orthogonal axis; cavity ≤ 5 mm still works
  16/16 ZERO LM at the C′.a ε pin).
- Modify gated A.  Class-1 LM rescue stays orthogonal +
  unchanged.
- Modify the `catch_unwind` belt-and-suspenders.  Class-3 Yeoh
  safety is exactly what surfaced the wall as a graceful UI
  report rather than a crash.
- Speculate on the Yeoh-wall fix beyond the §3 hypothesis
  ranking.  Next-session recon picks based on empirical
  evidence.

---

## 5. Falsification mechanics — what the spec correctly predicted

The spec §5 falsifier matrix anticipated **case E** verbatim:

> Case E — 3 or 5 mm sanity gate at the chosen `(k, r)` drops
> below 16/16 converged | E.b composes badly with C′.a's gap
> smoothing — hard regression of the existing baseline. | E.b.4
> case E: revert opt-in + bookmark + escalate to F.a.

The empirical data matched: 3 mm holds, 5 mm regresses 1/16.
The recommended action (revert + bookmark) is what this commit
ships.  Escalation target is reframed from F.a (step-0 warmup
for chattering) to the Yeoh-wall sub-arc — because E.b's data
showed that the cavity > 5 mm problem isn't chattering anymore.

**Banked**: spec falsifier matrices that include "regression at
existing baseline" branches are load-bearing — they prevent
shipping a knob whose default-on costs more than its non-default
behavior gains.

---

## 6. Anchors

**Commits on `sim-arc/sl-4-intruder-render`** (all local):

- `0fd0cec2` — E.b normal-averaging spec.
- `a8146001` + `96907778` — E.b.1 sim-soft per-pair normal
  averaging + cold-read polish.
- `43151fb1` + `db27379c` — E.b.2 cf-device-design opt-in
  (disabled state) + cold-read polish.
- `e13767dd` — E.b.3 sweep scaffolding (cavity cap 5 → 6 mm).
- (this commit) — E.b.4 case-E falsification: consts revert
  to `(1, 0)`, cap scaffolding 6 → 5 mm reverted, this bookmark
  + const-docstring sweep table addendum + egui label revert.

**Docs**:

- `docs/CANDIDATE_E_B_NORMAL_AVERAGING_SPEC.md` — E.b design
  (retained as audit trail; falsifier matrix §5 case E was
  empirically matched).
- `docs/CANDIDATE_C_SWEEP_FALSIFICATION_BOOKMARK.md` — parent
  bookmark; hyp 1 confirmed as a chattering contributor but
  unmasked by Yeoh wall as the actual cavity > 5 mm bottleneck.
- `docs/CAVITY_5MM_CHATTERING_BOOKMARK.md` — grandparent
  bookmark; cavity > 5 mm story reframed (Yeoh wall, not
  chattering).
- `docs/F3_FALSIFICATION_BOOKMARK.md` — F3 grandparent's
  3-class failure mental model.  Class 2 (active-set
  discontinuity) is RESOLVED at cavity ≤ 5 mm by C′.a; class 2
  at cavity > 5 mm is REPLACED by Yeoh-wall (geometry/material)
  as the new binding constraint.

**Memory pointers**:

- [[project-c-prime-a-shipped]] — direct predecessor; "What
  stayed dormant" §E candidates is now updated by this commit's
  finding (E.b shipped + falsified, not dormant anymore).
- [[project-c2-sweep-falsification-bookmark]] — RESOLVED case A
  for the C-arc; now extended by this commit's finding that
  the cavity > 5 mm problem is Yeoh wall, not chattering.
- [[project-cavity-5mm-chattering-bookmark]] — class-2
  partially-resolved at cavity ≤ 5 mm by C′.a; cavity > 5 mm
  story reframed by E.b case-E.
- [[project-f3-falsification-bookmark]] +
  [[project-f3-recon-a-gated-lm-shipped]] — class-1 LM rescue +
  Fork-B LmConfig are orthogonal, unchanged.
- [[project-f4-falsification-postmortem]] — sibling falsification
  pattern (F2 + F4 dead; F3 + E.b joining the dead-arc list).

**Code sites — preserved E.b surface (DO NOT revert)**:

- `sim/L0/soft/src/contact/penalty.rs:136-200` —
  `PenaltyRigidContact` struct + `normal_avg_k` /
  `normal_avg_radius_m` fields.
- `sim/L0/soft/src/contact/penalty.rs:215-` — 5 existing
  constructors + 2 new
  (`with_params_and_smoothing_and_normal_averaging` +
  `..._and_interior_cutoff`) + the `averaged_normal` private
  helper + MAINTENANCE NOTE.
- `sim/L0/soft/src/contact/penalty.rs:` per_pair_readout +
  `ContactModel::gradient` + `ContactModel::hessian` consumer
  sites all route `prim.as_ref()` through `averaged_normal`.
- `sim/L0/soft/tests/penalty_normal_averaging.rs` — 9 unit
  tests pinning bit-equal-when-disabled + averaging-on-plane +
  averaging-on-sphere + 4 constructor-validation panics +
  with-cutoff happy path.
- `tools/cf-device-design/src/insertion_sim.rs:~1000-1050` —
  `INSERTION_CONTACT_NORMAL_AVG_K = 1` +
  `INSERTION_CONTACT_NORMAL_AVG_RADIUS_M = 0.0` consts (pinned
  disabled) + their sweep-table-bearing docstrings.
- `tools/cf-device-design/src/insertion_sim.rs:~1043` —
  `intruder_contact_at` routes through
  `with_params_and_smoothing_and_normal_averaging` (6-arg,
  dormant at `(1, 0)`).
- `tools/cf-device-design/src/insertion_sim.rs:~2406` —
  `intruder_contact_sliding_at` routes through
  `with_params_and_smoothing_and_normal_averaging_and_interior_cutoff`
  (7-arg, dormant at `(1, 0)`).
- `tools/cf-device-design/src/insertion_sim.rs` `mod tests` —
  `insertion_contact_normal_avg_sentinel` pins `(1, 0)` +
  bounds + composition invariant.

**Code sites — REVERTED scaffolding** (3-surface mirror):

- `tools/cf-device-design/src/main.rs:~282-284` —
  `CavityState::inset_slider_range_m` returns `(0.0, 0.005)`.
- `tools/cf-device-design/src/main.rs:~2110-2120` — egui label
  reads "capped at 5 mm — highest cavity the C′.a-pinned ε
  converges 16/16 at; E.b.4 case-E falsification ...".
- `tools/cf-device-design/src/main.rs:~3000-3030` — sentinel
  test `cavity_inset_slider_range_zero_to_five_mm` asserts
  `max_m = 0.005`.

---

## 7. Implementation expectations for the next session

- Per [[feedback-bookmark-when-surface-levers-exhaust]] —
  three-session pattern.  This commit is the bookmark.  Next
  session is recon (pick a Yeoh-wall hypothesis from §3).  The
  session after that is implementation.
- Per [[feedback-autonomous-architecture]] — drive the
  Yeoh-wall recon autonomously; flag only big departures.
- Per [[feedback-implement-measure-revert-pattern]] — H1 is a
  ≲ 50 LOC slide-step edit; implement-measure-revert directly
  without a separate spec session if the change scope holds.
- Per [[feedback-cold-read-review-post-ship]] +
  [[feedback-cold-read-two-passes-for-non-trivial-diffs]] —
  cold-read on every non-trivial commit.

---

## 10. §POST-RESOLUTION — N_STEPS sweep reframes the wall as Newton overshoot

2026-05-19 LATE-EVENING.  After the E.b case-E ship, an N_STEPS
slide-fraction sweep at cavity = 6 mm WITHOUT E.b (consts at
`(1, 0)` per case E) revealed that **the "Yeoh wall" diagnosis in
§1.3 was incomplete**.  The wall is real but it's a **Newton-path-
overshoot at step 2+**, not a true material limit.

### 10.1 Sweep data (cavity = 6 mm, E.b disabled, layers 10+3 mm)

| N | step 1 d | step 1 outcome | iters | F (N) | step 2 outcome |
|---|---|---|---|---|---|
| 8 | 10.42 mm | ✓ | 41 | 8.14 | Yeoh tet 3206 ms=1.057 (d=20.84) |
| 12 | 6.95 mm | ✓ | 23 | 8.16 | Yeoh tet 3206 ms=1.052 (d=13.90) |
| **16** | **5.21 mm** | **✗ chatter r=0.536** | 150 cap | — | — |
| 20 | 4.17 mm | ✓ | 36 | 8.07 | Yeoh tet 3206 ms=1.052 (d=8.33) |
| 24 | 3.47 mm | ✓ | 20 | 8.06 | Yeoh tet 3206 ms=1.051 (d=6.94) |
| 32 (cavity=5mm) | 2.60 mm | ✗ chatter r=0.131 | 150 cap | — | — |

### 10.2 Two structural reframes

**Reframe A — N=16 is uniquely pathological at cavity = 6 mm.**
The C-arc's "chattering envelope at cavity > 5 mm" diagnosis was
N=16-specific.  N=8, 12, 20, 24 all converge step 1 cleanly at
cavity = 6 mm WITHOUT E.b.  Only N=16 chatters at step 0.  The
neighbors N=12 and N=20 bracket N=16 on both sides + both converge;
this isn't a slide-fraction-monotonic threshold but a specific
resonance.  C′.a's ε = 0.075 mm and E.b's `(7, 1 mm)` were both
tuned to fix the N=16 step-0 chattering — but the fix wasn't
necessary at other N values.

**Reframe B — Yeoh wall is Newton-overshoot, not material limit.**
Compare two N values reaching the same step-2 target depth:

- N=12 step 1 reaches d=6.95 mm from rest CLEANLY (max_stretch
  nowhere near 1.0 — no Yeoh panic).
- N=24 step 2 tries to reach d=6.94 mm from a warm-started state
  at d=3.47 mm.  Yeoh fires at tet 3206 ms=1.051.

**Same target depth, different paths, different outcomes.**  The
geometry CAN handle d=6.95 mm (proven by N=12 step 1).  Newton's
iteration scheme at step 2+, starting from a warm-started deformed
state, makes a trial step that overshoots tet 3206's deformation
gradient past validity — Armijo gets no chance to backtrack
because the validity check fires DURING the energy/gradient eval
on the trial state.

### 10.3 Universal Yeoh stub at step 2+ regardless of N

Across N values that survive step 1, **step 2 always hits Yeoh at
tet 3206 with max_stretch ≈ 1.051 - 1.057**.  Step-2 target
depths varied from 6.94 mm (N=24) to 20.84 mm (N=8) — a 3× range
— but the Yeoh ms barely budges.  Pattern strongly suggests
Newton's first-trial iteration at step 2 extrapolates a similar
displacement pattern regardless of step size, hitting the same
geometric overshoot near tet 3206.

### 10.4 What this means for the recon ladder

§3's H1/H2/H3 hypothesis ranking was framed around "the Yeoh wall
is the new binding constraint" — that framing is now superseded:

- H1 (slide-step-size artifact): tested + falsified for
  Yeoh-wall-avoidance (N=32 chatters at step 0 even at cavity = 5
  mm where N=16 was clean).  But H1's spirit (smaller steps) IS
  what produces step-1 convergence at cavity = 6 mm via N=12, 20,
  24 — just at the cost of step-2 Yeoh overshoot.
- H2 (mesh refinement): less directly relevant now.  Tet 3206's
  validity violation is a Newton-trial-state issue; finer mesh
  would distribute the overshoot across more tets but Newton would
  still overshoot.
- H3 (true material limit): FALSIFIED by reframe B.  d=6.94 mm IS
  reachable (N=12 proves it from rest).  The limit is solver
  implementation, not material.

**New recon path** — sub-arc on Newton overshoot at step 2+ of the
sliding ramp:

- (a) Damped first-iter step at sliding-ramp step 2+ — limit the
  Newton trial state to within validity by scaling the first
  iteration's step size.
- (b) Per-tet Yeoh-aware trial limiter — pre-screen Newton's trial
  state against `max_stretch_deviation` before energy eval, scale
  back if any tet exceeds 1.0.
- (c) Micro-substep at sliding-ramp step 2+ — split each step 2+
  into 2 micro-substeps so Newton's trial states stay smaller.

(a) and (c) are sim-soft-side fixes; (b) requires Yeoh-internal
plumbing.  Start with (a) ~50 LOC implement-measure-revert.

### 10.5 What gets reverted from this commit

The sweep scaffolding (cap raise 5 → 6 mm + temp comments) is
reverted in this commit.  All sim-soft E.b surface plumbing
stays.  `DEFAULT_N_STEPS = 16` stays as the production default
(other N values introduce per-step regressions that aren't
sweep-tested against the full cavity-≤-5 mm baseline).

### 10.6 Updated next-session pointer

Sub-arc: Newton overshoot at sliding-ramp step 2+.  Cheap probe
candidate (a) damped first-iter step; if successful at cavity = 6
mm without regressing cavity ≤ 5 mm, ship + raise cap to 6 mm
(real this time, not scaffolding).

---

End of bookmark.
