# Cavity 5 mm class-2 chattering — bookmark (candidate C entry point)

**Status**: surfaced by F3 recon A's outcome-B visual gate 2026-05-18
EVENING (`docs/F3_RECON_A_GATED_LM_SPEC.md` §6 A.4). Three-session
pattern per [[feedback-bookmark-when-surface-levers-exhaust]]: this is
the bookmark session. Recon next; recon's implementation third.

**Branch**: `sim-arc/sl-4-intruder-render`. Gated-A shipped as commits
`9a1433b8` (spec) + `61b31f9b` (cold-read polish) + `59997c41` (impl).
This bookmark + UI cap to 4 mm land in the same outcome-B follow-up
commit.

**Predecessor docs**:
- `docs/F3_RECON_A_GATED_LM_SPEC.md` — gated-A spec; §1 inherited
  3-failure-class mental model identifies class 2 as the orthogonal
  arc this bookmark addresses.
- `docs/F3_FALSIFICATION_BOOKMARK.md` — F3 falsification + §8
  RESOLUTION footer pointing here.
- `docs/CAVITY_INSET_STALL_BOOKMARK.md` — original arc; §8 Run 2 is
  the pre-F3 cavity-only-change baseline this bookmark's evidence
  references.
- `docs/F4_FALSIFICATION_POSTMORTEM.md` — F4 + F2 falsification context;
  F5 (smoothed contact) appears in §10 candidate-set scoring as a
  high-cost alternative — that's what candidate C builds on.

**Predecessor memory**: [[project-f3-recon-a-gated-lm-shipped]]
(direct predecessor — gated-A's outcome-B Gate B is the empirical
surface this bookmark addresses), [[project-f3-falsification-bookmark]],
[[project-cavity-inset-stall-bookmark]],
[[project-f4-falsification-postmortem]], [[project-sl-4-arc-shipped]].

---

## 1. TL;DR

Gated-A's outcome-B visual gate empirically confirms what the F3
falsification bookmark §2.2 hypothesized: at cavity ≥ 5 mm, the
sliding-ramp insertion sim has a **structural r_norm floor** that
Newton+Armijo cannot cross, **independent of LM regularization**. The
floor is driven by **class 2 — active-set discontinuity** in the
penalty-contact Hessian (`PenaltyRigidContact::active_pairs`): each
Newton step at cavity ≥ 5 mm can cross an active-pair on/off boundary,
producing a worse residual at the new x_curr than the linearized
Newton model predicted. Armijo accepts each step (linearized prediction
satisfied at the SOURCE x_curr), but the post-step assembly at the
NEW x_curr lands in a different active-set partition.

**Candidate C** (smoothed-contact penalty barrier): mollify the
penalty function so the active-pair on/off boundary becomes a
smooth transition (e.g., barrier with a Gaussian-smoothed threshold,
or per-pair sigmoid weighting near `sd ≈ d̂`). De-chatters the active
set at the cost of penetration tolerance. Estimated cost ~300+ LOC in
sim-soft + calibration window vs. penetration depth.

---

## 2. The empirical evidence

### 2.1 Gate B verbatim trace (gated-A, cavity = 5 mm, default layers 10+3 mm)

From `/tmp/recon_a_gui.log` 2026-05-18 EVENING:

```
sim-soft: faer LU fallback fired at factor_and_solve_free at Newton iter 2 (free residual norm 5.665118902894688e1) (Llt non-PD pivot: NonPositivePivot { index: 13387 })
sim-soft: faer LU fallback fired at factor_and_solve_free at Newton iter 3 (free residual norm 3.8031427348765526e1) (Llt non-PD pivot: NonPositivePivot { index: 13764 })
... [iters 4-56: progressive r_norm decrease ~38 → 1.78, LU+Armijo accepts every iter] ...
sim-soft: faer LU fallback fired at factor_and_solve_free at Newton iter 57 (free residual norm 1.7836752010550596e0) (Llt non-PD pivot: NonPositivePivot { index: 33374 })
sim-soft: LM seeded λ = 6.669388036959534e-3 at factor_and_solve_free at Newton iter 57 (free residual norm 1.7836752010550596e0)
sim-soft: LM converged in 1 retries to λ = 6.669388036959534e-3 at factor_and_solve_free at Newton iter 57
sim-soft: faer LU fallback fired at factor_and_solve_free at Newton iter 58 (free residual norm 1.7836728780581803e0)
sim-soft: faer LU fallback fired at factor_and_solve_free at Newton iter 60 (free residual norm 1.783667081522099e0)
sim-soft: faer LU fallback fired at factor_and_solve_free at Newton iter 61 (free residual norm 1.7836667564733832e0)
Error: sliding ramp failed at step 0 — no converged step. Armijo line-search stalled at Newton iter 61 (r_norm 1.784).
```

### 2.2 The class-2 mental model in evidence

| Iter | r_norm | Mechanism |
|---|---|---|
| 0 | unobserved (Llt-PD, no log) | initial residual at x_prev = rest position |
| 1 | unobserved (Llt-PD, no log) | iter 0's LU step + Armijo accepted; gated A's first-pass succeeded → no LM activity, no stderr log line |
| 2 | **56.65** | first iter where Llt non-PD detection fires (LU-fallback log line emitted); IF iter 1's r_norm was modest (per F3.4 Gate B's always-on-LM data, iter 1 saw r_norm = 10.94 at the SAME geometry — strong evidence iter 1 → iter 2 crossed an active-pair boundary), this r_norm jump is the active-set CROSSING signature. **Caveat**: gated A's iter 1 r_norm is unobservable from stderr (PD iters don't log); the iter-1 → iter-2 chattering jump inference is by-geometry-analogy with F3.4 Gate B, not direct evidence. |
| 3-56 | 38 → 5 → ~1.79 | progressive descent — LU + Armijo accepts every iter despite Llt-non-PD throughout (LU fallback fires per iter; Armijo accepts every step) |
| 57 | 1.7836 | first iter where LU + Armijo Armijo-stalls — gated A escalates |
| 57 (LM) | 1.7836 | LM seeded at λ = 6.67e-3 (very small — the tangent is only mildly indefinite); 1 retry; Armijo accepts the LM step (proven by iter 58's existence — Newton advanced) |
| 58, 60, 61 | 1.7837 → 1.7837 → 1.7837 | LU + Armijo accepts again at iter 58 (LM dormant; persistent λ not consumed because first-pass succeeded); iters 60-61 similar; r_norm decreasing by ~1e-5 per iter — converging on a structural floor |
| 61 (Armijo) | 1.7837 → stall | iter 61's factor + solve succeeded (last LU-fallback log line shown above) but armijo_backtrack failed: `trial_norm ≤ (1 - α·c1) · 1.7836` couldn't be satisfied within `max_line_search_backtracks = 20` backtracks — even a 5e-4 absolute decrease would satisfy Armijo, so the trial residual stayed > 1.7836 for every α tried. THIS IS THE CHATTERING FLOOR. |

The r_norm decreases by `O(1e-5)` per iter from iter 57 onward — the
Newton iteration is computing valid descent steps, the LM rescue
succeeds when needed, **but the residual asymptotes to ~1.7837** because
each step crosses just enough active-pair boundaries that the gain
from the descent step is offset by the active-set discontinuity. The
final Armijo stall fires when the residual decrease per step falls
below Armijo's c1 = 1e-4 tolerance.

### 2.3 Why F3's `+λI` cannot smooth this

F3's lever regularizes the assembled tangent `A = M/dt² + K(x) +
H_contact(x)` by adding `λI`. This dominates negative eigenvalues of
`A` to make `A + λI` SPD → the LM step is guaranteed a descent
direction.

But `H_contact(x)` is **discontinuous** at active-pair boundaries: when
a pair flips active/inactive, the corresponding rows/columns of
`H_contact` jump (add/remove the penalty contribution). The
discontinuity is in the FUNCTION VALUE, not just in the eigenstructure
— no diagonal additive can smooth a value discontinuity.

Gate B's evidence: even with LM successfully rescuing the iter-57
escalation (`λ = 6.67e-3` was tiny — barely any regularization needed),
the residual still floored at 1.78. The mechanism the spec described
is **structurally insufficient** for this failure class.

---

## 3. Candidate-C mechanism preview

**Smoothed contact** (the F3 spec §1 reference + the F4 postmortem §10
F5 entry): replace `PenaltyRigidContact`'s hard penalty with a
mollified version:

```
hard:    c(sd) = κ · max(0, d̂ - sd)²         (active when sd < d̂; sharp on/off at sd = d̂)
smooth:  c(sd) = κ · (d̂ - sd)² · smooth_step(d̂ - sd, ε)
                where smooth_step(u, ε) =
                    0 for u < -ε
                    cubic or sigmoid transition for -ε ≤ u ≤ ε
                    1 for u > ε
```

The smoothing window `ε` de-chatters the active-set boundary at the
cost of penetration tolerance (the smoothed penalty doesn't go to 0
exactly at `sd = d̂` — it tapers over a band of width `ε`).

**Falsifier**: at cavity = 5 mm, the smoothed-contact sim's Newton
trajectory should:
1. Eliminate the iter-1 → iter-2 r_norm jump (10.94 → 56.65) — the
   active set evolves continuously rather than jumping.
2. Converge below tol = 0.1 (since the structural r_norm floor at 1.78
   was driven by chattering — removing chattering removes the floor).
3. Preserve cavity = 3 mm baseline 16/16 (the smoothed contact should
   converge to a similar fixed point at shallow insets).

**Calibration tradeoff**: too-tight `ε` → smoothing has no effect,
chattering persists. Too-loose `ε` → penetration tolerance grows,
contact-force readouts become inaccurate. Iter-1 calibration likely
needs an empirical sweep over `ε ∈ [0.1, 1.0] · d̂` (where `d̂ = 1 mm`).

---

## 4. Sub-leaf ladder (preliminary; recon refines)

Pre-recon scope sketch. Don't commit; the recon picks the actual ladder.

### C.0 — recon spec (next session)
- Read PenaltyRigidContact internals; identify the active-pair gather
  + Hessian assembly sites.
- Design the smoothed penalty: cubic vs sigmoid? `ε` parameter
  surface (per-pair or global)? Fork-A vs Fork-B applicability?
- Document falsifier matrix at cavity = 3 / 4 / 5 / 6 / 7 mm.
- Bit-equal-when-dormant contract: smoothed contact must reduce to
  hard contact when `ε = 0` (so cf-cast + sim-hard production callers
  stay bit-equal pre-opt-in).
- Per-pair on_saturation / failure-class implications (per the F3
  lesson — bake in mirror-on-change MAINTENANCE NOTES from day 1).

### C.1 — sim-soft `SmoothedPenaltyRigidContact` primitive
- New file `sim/L0/soft/src/contact/smoothed_penalty.rs` (or modify
  existing `penalty.rs` with a smoothing-window field — gated by
  zero `ε`).
- ~150-300 LOC depending on whether per-tet or per-DOF smoothing.
- Unit tests: smoothing window behavior at exact `sd = d̂`, transition
  monotonicity, gradient continuity vs hard penalty at small-`ε`.

### C.2 — cf-device-design opt-in + calibration sweep
- `INSERTION_CONTACT_SMOOTHING_M` const (alongside
  `INSERTION_CONTACT_KAPPA` + `INSERTION_CONTACT_DHAT`); pass through
  to the contact builder.
- User-driven visual gate sweep: cavity = 5 mm × `ε` ∈ {0.1, 0.5, 1.0,
  2.0} mm. Find the smallest `ε` that converges 16/16.

### C.3 — UI cap restoration (outcome-dependent)
- If C.2 ships at `ε = 0.5 mm` and cavity ∈ [0, 7 mm] converges →
  raise UI cap to 7 mm (or back to 8 mm if Yeoh-validity bounds hold).
- Update sentinel test + docstring per the gated-A precedent.

### C.4 — F3 falsification bookmark resolution update
- Append §9 "RESOLVED full by candidate C" to F3 falsification bookmark.
- Update this bookmark with resolution status + final UI cap chosen.

---

## 5. What this bookmark does NOT do

- **Pick the actual recon scope**. Three-session pattern: bookmark
  (this) → recon → implementation. Don't pre-commit to the §4 ladder
  scope.
- **Address class 3 (Yeoh material validity at cavity ≥ 8 mm)**. That's
  a separate arc (candidate B — material-validity safe-step, from the
  F3 falsification bookmark §4). Candidate C addresses class 2 only.
  Both may be needed to restore cavity ≥ 8 mm design space.
- **Touch gated-A's `try_solve_impl` mechanism**. Gated A is the right
  baseline-restoration lever and stays as-is. Candidate C is an
  orthogonal contact-model improvement.
- **Speculate on the smoothing-window value**. C.0 recon picks
  empirically. The hard penalty's `d̂ = 1 mm` is the natural reference
  scale; `ε` will likely be in `[0.1, 2.0] · d̂`.

---

## 6. Anchors

**Predecessor commits** (on `sim-arc/sl-4-intruder-render`):
- `9a1433b8` — F3 recon A spec.
- `61b31f9b` — F3 recon A spec cold-read polish.
- `59997c41` — F3 recon A implementation (A.1 + A.2 + A.3).
- *(this commit)* — F3 recon A outcome-B follow-up (UI cap to 4 mm +
  F3 falsification §8 RESOLUTION footer + recon spec STATUS header +
  this candidate-C bookmark).

**Code sites for C.0 recon**:
- `sim/L0/soft/src/contact/penalty.rs` — `PenaltyRigidContact` (the
  hard-penalty contact primitive C smooths).
- `sim/L0/soft/src/contact/mod.rs` — contact trait + active-pair
  surface.
- `tools/cf-device-design/src/insertion_sim.rs:923-928` —
  `INSERTION_CONTACT_KAPPA = 1.0e3` + `INSERTION_CONTACT_DHAT = 1e-3`
  (the existing contact tunables; the smoothing-window const lands
  alongside).
- `tools/cf-device-design/src/insertion_sim.rs::intruder_contact_at`
  (lines ~937-950) + `intruder_contact_sliding_at` — contact-builder
  sites.

**Memory**:
- [[project-f3-falsification-bookmark]] — pointed to "RESOLVED partial
  by gated A" — references this bookmark for the class-2 next-arc.
- [[project-sl-4-arc-shipped]] — establishes the SL.4 sliding-intruder
  rendering arc baseline.
- [[project-cavity-inset-stall-bookmark]] — the original arc; §8 Run 2
  baseline `r_norm = 1.78` is the empirical floor candidate C must
  cross.