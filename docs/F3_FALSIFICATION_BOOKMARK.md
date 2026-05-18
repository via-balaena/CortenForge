# F3 — Levenberg-Marquardt regularization — falsification bookmark

**Status**: F3 design (`docs/F3_LM_REGULARIZATION_SPEC.md`) shipped through
F3.1 → F3.4 on `sim-arc/sl-4-intruder-render`; visual gate on iter-1
`sock_over_capsule.cleaned.stl` 2026-05-18 EVENING falsified the spec's
mental model. **F3.4 LM opt-in reverted on dev** (single-line revert in
`tools/cf-device-design/src/insertion_sim.rs::insertion_solver_config`);
F3.1–F3.3 sim-soft plumbing + F3.4 surface plumbing (`try_replay_step` /
`SolverFailure` / `catch_unwind` belt-and-suspenders) **KEPT** — those
are net architectural wins independent of F3's outcome.

**Three-session pattern**: this is the bookmark session. Recon next; the
recon's implementation third.

**Predecessor docs**:
- `docs/F3_LM_REGULARIZATION_SPEC.md` — the F3 spec (annotated with
  FALSIFIED 2026-05-18 EVENING header in the same commit).
- `docs/CAVITY_INSET_STALL_BOOKMARK.md` — the original cavity-inset stall
  bookmark F3 was attempting to close.
- `docs/F4_FALSIFICATION_POSTMORTEM.md` — the F4 + F2 falsification this
  bookmark mirrors structurally.

**Predecessor memory**: [[project-f3-lm-regularization-spec]],
[[project-cavity-inset-stall-bookmark]],
[[project-f4-falsification-postmortem]], [[project-sl-4-arc-shipped]].

---

## 1. TL;DR

F3.4 visual gate on `sock_over_capsule` produced a **non-monotone**
outcome that maps cleanly to none of the spec §3 A/B/C/D categories:

| Cavity inset | Pre-F3 (SL.4.3) | Post-F3.4 | F3's effect |
|---|---|---|---|
| **3 mm** | **16/16 converged** | **0/16**, Armijo stall at Newton iter 10, r_norm 1.048e-1 (5% above tol = 1e-1) | **broke baseline** — LM trajectory diverges from LU trajectory enough to miss loose-tol convergence |
| **5 mm** | stalled, r_norm floor ~0.55 N | 0/16, Armijo stall at Newton iter 34, r_norm 2.707e-1 (**halved floor**) | helped eigenstructure but didn't bridge to tol — Outcome C semantics |
| **8 mm** | stalled at step 0 | **1/16** (seated to 5.21 of 83.35 mm), then Yeoh material validity panic at step 2 (tet 1324: `max_stretch_deviation = 1.209`, F = [2.209, 0.842, 0.624]) | enabled deeper Newton walk, exposed a SEPARATE failure class F3 doesn't address |

F3's premise (indefinite tangent → LM rescues → Armijo converges) is
**partially correct**: it helps where the tangent was severely indefinite
(5 mm floor halved, 8 mm got one step in) but hurts where the LU fallback
was already adequate (3 mm regression), and at deep insets exposes a
material-validity failure class that's orthogonal to F3.

---

## 2. The three failure classes (mental-model update)

The spec assumed ONE failure class (indefinite tangent / non-descent
direction). Empirically cf-device-design's sliding-intruder sim has at
least THREE:

### 2.1 Indefinite-tangent / non-descent direction — F3 partial success

Spec §1's framing: the assembled tangent `A = M/dt² + K(x) + H_contact(x)`
is indefinite at the Newton iterate, the LU fallback step
`δ = A⁻¹ · (-r)` is not guaranteed a descent direction, Armijo can only
shrink along the supplied direction.

F3's lever (+λI to make `A + λI` SPD, the LM step is guaranteed descent)
**does** help when this is the dominant failure mode — at cavity = 5 mm
the r_norm floor moved from ~0.55 N (pre-F3) to 0.27 N (F3.4), and at
cavity = 8 mm step 0 converged where pre-F3 it did not.

### 2.2 LM-vs-LU trajectory drift at well-conditioned regions — NEW class created by F3

At cavity = 3 mm, the pre-F3 LU fallback step was sufficient for
loose-tol convergence in ~30–40 Newton iters. F3 replaces that LU step
with an LM step every time Llt is non-PD, and those LM steps push Newton
on a DIFFERENT trajectory. For cavity = 3 mm specifically, that
trajectory leads to Armijo stall at iter 10, r_norm = 1.048e-1 — just
**5% above tol = 1e-1**.

The trajectory drift is not a bug in F3; it's a structural consequence
of "LM step is guaranteed descent BUT slow descent at large λ" combined
with a finite Newton iter budget + loose tol. The pre-F3 LU step
happened to be aligned with the residual well enough that 30 iters got
r_norm below 0.1; the LM step's slower descent rate doesn't reach the
loose tol within the iter cap.

**Implication**: F3's mental model is wrong about "LU fallback always
fails on indefinite tangents". Sometimes LU fallback is the LEAST BAD
option, even at the cost of theoretical-soundness. F3's premise of
"always rescue LU with LM when Llt is non-PD" is too aggressive.

### 2.3 Yeoh material validity at deep insets — pre-existing class, exposed by F3

At cavity = 8 mm, F3 enabled Newton to converge step 1 (depth 5.21 mm).
Step 2 then panicked from `sim/L0/soft/src/solver/backward_euler.rs:678`
(Phase 4 Decision Q fail-closed):

```
validity violation at tet 1324: max_stretch_deviation = 1.209 exceeds bound
1.000 (singular values of F = [2.209, 0.842, 0.624]).
```

This is a CONSTITUTIVE-MODEL panic — the deformation gradient F at tet
1324 has principal stretches `[2.209, 0.842, 0.624]`, and the Yeoh
material model fails-closed when stretch exceeds its calibrated validity
envelope (`max_stretch_deviation > 1.000` ≈ `min(|λᵢ|, |1/λᵢ|) > 2.0`).

F3 didn't CAUSE this — the geometry inherently requires the cavity wall
to stretch 2.2× somewhere when seating a body-derived intruder at 8 mm
inset. Pre-F3 the Armijo stall at deep insets prevented Newton from
walking far enough to expose it. F3 enabled deeper walks → exposed the
material-validity bound.

**Implication**: cavity ≥ 5 mm has a SECOND failure class that even a
perfect indefinite-tangent solver wouldn't address. The cf-device-design
UI's "capped at 8 mm — past this, material strain exceeds Yeoh validity"
warning was already in place; we now know the bound bites at 8 mm even
INSIDE the cap, not just past it.

---

## 3. What the F3 plumbing earned independent of the LM opt-in

Even with the LM opt-in reverted, F3.1 → F3.4 left durable architectural
surface improvements:

- **`SolverFailure` enum** (3 variants) — richer-than-string error context
  for the three documented non-recoverable Newton-loop failures. Pre-F3
  these were all `panic!`s with string-parsed messages.
- **`Solver::try_step` + `try_replay_step`** REQUIRED trait methods — a
  graceful-failure API surface for the documented failures.
  `SaturationPolicy::PanicOnStall` preserves pre-F3 panic semantics when
  consumers don't explicitly opt in.
- **`solver_failure_message(&SolverFailure) -> String`** in
  cf-device-design — formats SolverFailure into the same shape
  `panic_message` produced, so the SL.4 viewport's "stalled at step N:
  <reason>" surface reads consistently regardless of which surface
  tripped.
- **Belt-and-suspenders `catch_unwind` + `try_replay_step` pattern** in
  `run_sliding_insertion_ramp` — F3.4 hotfix (`3f988e21`) after the Gate
  C visual gate. `try_replay_step` handles documented SolverFailures
  with richer context; `catch_unwind` catches the undocumented panics
  (Yeoh material validity, debug_asserts, OOM). Strictly better than
  either alone. The MAINTENANCE NOTE comments cross-link the two
  formatter sites (`solver_failure_message` ↔ inline match arms in
  `run_single_insertion_step`).

These survive the recon because they're orthogonal to F3's LM mental
model — they describe the failure surface, not the rescue mechanism.

---

## 4. Future-arc candidates

Ranked by implementation cost ascending. The recon next session picks
between them (or some combination).

### A. Gated LM (F3-modified, smallest-cost variant of F3 itself)

**Premise**: F3's LM rescue is theoretically sound but too eager. Engage
LM only when LU fallback step + Armijo ALSO stalls — not on every Llt
non-PD detection.

**Mechanism**: extend `LmState` with a "Armijo-failed-on-LU-step" flag.
On Llt non-PD, first try the LU fallback step (pre-F3 behavior); only
if Armijo can't accept that step, escalate to LM retries. The successful
LM step then replaces the rejected LU step for the rest of that Newton
iter; LmState resets on the next Newton iter.

**Falsifier**:
- Cavity = 3 mm baseline must restore bit-equally (LU step would have
  worked in pre-F3, so the LU+Armijo path succeeds, no LM escalation).
- Cavity = 5 mm: at iters where pre-F3 stalled, LM now escalates and
  produces a descent step. If the same r_norm-floor improvement (0.55 →
  0.27 N) is achieved with the gated variant, we know the spec's
  mechanism was right, just the gate was wrong.
- Cavity = 8 mm: same as 5 mm; if step 1 converges again the gating is
  validated; step 2 will still hit Yeoh validity (orthogonal class).

**Implementation cost**: ~80 LOC in `sim/L0/soft/src/solver/backward_euler.rs`
(extend `LmState` + thread the Armijo-failed-on-LU flag through
`armijo_backtrack` → `factor_free_tangent`). cf-device-design re-enables
LM in `insertion_solver_config` — same one-liner. No new SolverFailure
variants needed.

**Risk**: the "first try LU, escalate on Armijo failure" gate increases
per-iter cost when LM is needed (factor twice: LU once for the rejected
step, then LM retry loop for the replacement). Acceptable if it's the
only path to preserving the baseline.

### B. Material-validity safe-step

**Premise**: the Yeoh validity panic at cavity = 8 mm is a Phase 4
fail-closed assertion intended to prevent garbage-out solutions. The fix
is a NEWTON STEP PROJECTION: before applying `x_curr + α · δ`, check
each tet's resulting F and clip α to keep all tets within the
material-valid region.

**Mechanism**: a new sim-soft primitive
`max_alpha_preserving_validity(x_curr, δ, mesh, material) -> f64` called
inside the Armijo loop. The first backtrack-α tried becomes
`min(α_init, α_validity)` where `α_validity` is the largest α keeping
all tets valid. Newton's step magnitude is bounded by material validity,
not just the line-search; over multiple Newton iters the deformation
walks toward the solution in validity-feasible increments.

**Falsifier**: cavity = 8 mm step 2 no longer panics. If step 2 converges
(possibly slowly) the candidate is validated. If step 2 instead stalls
(Newton runs out of iters before walking far enough due to α-clipping),
the candidate partially helps but exposes a Newton iter budget tension.

**Implementation cost**: moderate (~150 LOC in sim-soft) — the validity
check is per-tet per-α-trial, which adds cost to every Armijo backtrack
but is O(n_tets) vs the O(n_tets · iters) of Newton itself, so the
overhead is bounded.

**Risk**: clipping α may dramatically slow Newton convergence (small
steps + finite iter budget = no convergence). The validity-safe step
might preserve material integrity at the cost of Outcome-C-like Armijo
stalls everywhere.

### C. Smoothed contact (the spec's F5)

**Premise**: F3 spec §1 already calls out active-set chattering as F3's
structural limit (the part F3 can't address). At deep insets the active
contact set switches between Newton iters, making `H_contact(x)`
effectively discontinuous; no second-order method converges quadratically
through chattering boundaries.

**Mechanism**: replace the hard penalty contact with a mollified version
(e.g., smoothed barrier function: `c(sd) = κ · (sd - d̂)² / d̂² · smooth(sd)`).
The smoothing window de-chatters the active set boundary at the cost of
penetration tolerance.

**Falsifier**: cavity = 5 mm + 8 mm convergence improves dramatically
(active-set chattering was the dominant cause). Cavity = 3 mm should be
unchanged (no chattering at shallow insets).

**Implementation cost**: high (~300+ LOC, new contact primitive in
sim-soft + threading through `PenaltyRigidContact`). Plus calibration
work on the smoothing window vs. penetration tolerance.

**Risk**: smoothing widens the contact-affected region → may worsen
material strain at deep insets (more tets engaged → more stretch
concentration) → re-exposes the Yeoh validity class.

### D. Mesh refinement at high-stretch regions

**Premise**: if material validity is concentrated at specific tets (tet
1324 in our case), adaptive mesh refinement reduces per-tet stretch by
spreading the deformation across more elements.

**Mechanism**: detect high-stretch tets pre-solve, subdivide them
(longest-edge bisection) before meshing the FEM problem.

**Implementation cost**: high (adaptive mesher + remeshing logic). Plus
likely calibration of the refinement criterion.

**Risk**: refined mesh = more DOFs = bigger linear solve per iter. May
trade material-validity violations for runtime + memory pressure.

---

## 5. Recommendation

Three-session pattern:

1. **This session** (bookmark): three-gate data captured, F3.4 LM opt-in
   reverted, F3.1–F3.4 surface plumbing kept, this doc written.
2. **Next session** (recon): pick a candidate (A is the cheapest first
   step; B addresses an orthogonal class and could be combined with A).
   Spec the recon's chosen candidate in a new design doc.
3. **Third session** (implementation): build the recon outcome.

The recon should also consider: maybe cavity = 8 mm is just OUT OF SCOPE
for this material + geometry. The UI already capped the slider there;
maybe the right answer is to LOWER the cap (e.g., 5 mm) and document
that the cavity-inset design space is bounded by material validity, not
solver convergence.

---

## 6. Anchors

### This session's commits (on `sim-arc/sl-4-intruder-render`, pushed to origin)

- `6dc51959` — F3.4 LM opt-in (cf-device-design `try_replay_step` swap +
  surface plumbing).
- `7aad8488` — F3.4 cold-read polish (M1 maintenance note).
- `3f988e21` — F3.4 hotfix: restore `catch_unwind` around
  `try_replay_step` (Yeoh validity panic at Gate C revealed
  catch_unwind was load-bearing for undocumented panic classes).
- *(this commit)* — F3 falsification bookmark + 1-line LM opt-in revert
  + spec FALSIFIED annotation.

### Predecessor sim-soft commits (the F3 spec + plumbing — KEPT, not reverted)

- `3b4ec375` + 3× cold-read polish — F3 spec.
- `60db1cb2` — F3.1 LM types (`LmConfig` + `SaturationPolicy`).
- `97da606f` — F3.2 LM retry loop + LmState plumbing.
- `bce67ac9` — F3.3 SolverFailure enum + `try_step` / `try_replay_step`
  trait methods.
- `00e5e0dc` — F3.2+F3.3 cold-read pass-2 polish.

These survive the recon because the `SolverFailure` surface is correct
independent of whether LM is the right rescue mechanism. The recon
candidates above all consume the same surface.

### Visual-gate observations (verbatim from
`tools/cf-device-design`'s stderr, iter-1 sock_over_capsule.cleaned.stl)

**Gate A — cavity = 3 mm, default 2-layer design (Ecoflex 00-30 + Slacker / Dragon Skin 20A)**:
```
sim-soft: LM seeded λ = 3.17e-2 at factor_and_solve_free at Newton iter 1
  (free residual norm 10.94) (Llt non-PD pivot: NonPositivePivot { index: 14734 })
sim-soft: LM converged in 6 retries to λ = 3.17e3 at Newton iter 1
[iters 2–9 silent — Llt-PD throughout]
Error: sliding ramp failed at step 0 — no converged step.
Armijo line-search stalled at Newton iter 10, r_norm 1.048e-1
```

Pre-F3 baseline (SL.4.3 documented at [[project-sl-4-arc-shipped]]):
**16/16 converged** with the same defaults.

**Gate B — cavity = 5 mm**:
```
sim-soft: LM seeded λ = 3.17e-2 at Newton iter 1 (r_norm 10.94)
sim-soft: LM converged in 6 retries to λ = 3.17e3 at Newton iter 1
sim-soft: LM seeded λ = 7.22e-3 at Newton iter 2 (r_norm 56.65)
  [r_norm rose from 10.94 → 56.65 — the LM step at iter 1 over-shot]
sim-soft: LM converged in 4 retries to λ = 7.22e0 at Newton iter 2
sim-soft: LM bumped λ = 3.61e1 at Newton iter 3 (r_norm 33.47)
sim-soft: LM converged in 1 retries to λ = 3.61e1
sim-soft: LM bumped λ = 1.80e2 at Newton iter 4 (r_norm 32.78)
sim-soft: LM bumped λ = 4.51e2 at Newton iter 6 (r_norm 12.18)
sim-soft: LM bumped λ = 1.13e3 at Newton iter 8 (r_norm 4.55)
sim-soft: LM bumped λ = 8.81e1 at Newton iter 15 (r_norm 9.82e-1)
  [λ decayed from 1.13e3 → 8.81e0 over iters 9–14, then bumped again]
Error: sliding ramp failed at step 0 — no converged step.
Armijo line-search stalled at Newton iter 34, r_norm 2.707e-1
```

Pre-F3 baseline (from [[project-cavity-inset-stall-bookmark]]):
stalled with r_norm floor ~0.55 N. F3 halved the floor (0.55 → 0.27 N)
but didn't bridge to tol = 0.1 N.

**Gate C — cavity = 8 mm (after F3.4 hotfix `3f988e21` — pre-hotfix
this CRASHED the Bevy app)**:
```
[LM activity at iters 4, 15, 17, 22 etc. — see /tmp/f34_gui.log for full
trace; λ trajectory walked up to ~1.5e4 at iter 4 then decayed]
Converged: 1 of 16 steps — seated to 5.21 of 83.35 mm
Solver reason: validity violation at tet 1324: max_stretch_deviation = 1.209
exceeds bound 1.000 (singular values of F = [2.209, 0.842, 0.624]).
Phase 4 scope memo Decision Q fail-closed semantics.
```

Pre-F3 Gate C is undocumented (the UI cap was 8 mm post-F4.1 specifically
because pre-F4 Newton stalled at deep insets), but per the F4
postmortem's framing it would have stalled at step 0. F3 enabled step 1
to converge, exposed step 2's material validity.

### Per-layer state at Gate C abort (from the Validations + Per-layer
panel readout)

- Layer 0 (innermost, Ecoflex 00-30 + 50% Slacker, 10 mm thick): 46634
  tets, mean strain energy density 4.86e2 J/m³, max first Piola
  Frobenius 2.66e5 Pa, principal stretch range **0.26..2.30** (the
  outlier 2.30 stretch is what tripped Yeoh validity).
- Layer 1 (outermost, Dragon Skin 20A, 3 mm Δ from prior layer): 19653
  tets, mean Ψ 7.13e1 J/m³, max P 4.09e5 Pa, stretch range 0.54..1.34.

---

## 7. What this bookmark does NOT do

- It does NOT revert F3.1–F3.4's sim-soft + cf-device-design surface
  plumbing (`SolverFailure`, `try_step`/`try_replay_step`, LmConfig,
  `solver_failure_message`, `catch_unwind` belt-and-suspenders). Those
  are independent of the LM mental model.
- It does NOT recommend a specific recon candidate. The recon session
  picks; the cheapest (A — gated LM) is the natural first stop but
  isn't mandatory.
- It does NOT touch the F3 spec doc beyond adding a FALSIFIED status
  header pointing here. The spec's design content survives as audit
  trail; it's a correctly-designed implementation of an
  incorrectly-modeled failure class.
