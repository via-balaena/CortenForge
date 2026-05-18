# F4 — interference-homotopy continuation — FALSIFIED

**Status**: F4.3 SHIPPED `573e56fe` 2026-05-18 LATE-EVENING + REVERTED `c0443df4` same session after user-driven visual gate at the **default** cavity = 3 mm baseline (previously known-converging at 16/16 per [[project-sl-4-arc-shipped]]) failed at the first warmup substep. F4.2 (`26e83fa4` — pure API refactor) KEPT — the `interference_m` parameter on `intruder_contact_sliding_at` is a useful scaffold for any future homotopy attempt; today's behavior preserved bit-equal at `interference_m = 0`. Branch `sim-arc/sl-4-intruder-render` ends in the same observable state as before the F4.3 attempt (cavity = 3 mm converges 16/16, cavity = 5 mm step-0 stalls), with F4.0 docs + F4.1 UI cap + F4.2 API scaffold landed.

---

## 1. What F4.3 did

Two coupled changes (`573e56fe`):

1. **Warmup loop in `run_sliding_insertion_ramp`** before the main slide ramp. At the step-0 slide pose, run `K = ceil(cavity_inset_m / 1.5 mm).clamp(0, 8)` homotopy substeps ramping `interference_m` from `cavity_inset_m / K` to `cavity_inset_m`, warmstarting each.
2. **Main loop switched** from `interference_m = 0` (pre-F4 shrunk-scan model) to `interference_m = cavity_inset_m` (bare-scan model — body geometry overlapping the un-deformed cavity by `cavity_inset_m` everywhere the surfaces contact).

## 2. Empirical falsification

`cargo run --release -p cf-device-design -- ~/scans/sock_over_capsule.cleaned.stl` at the design's autoloaded defaults (cavity = 3 mm, layers 10 + 3 mm — the same params that converged 16/16 pre-F4 per the SL.4.3 visual gate):

> Error: sliding ramp failed at step 0 — no converged step.
> F4 warmup stalled at homotopy substep **1/2** (interference **1.500 mm** at step-0 pose):
> Armijo line-search stalled at Newton iter 18 (r_norm 3.899e-1, final α 4.77e-7).
> Likely causes: non-SPD tangent near solution (spec §3 R-2 violation), or near-singular condensed system.

The **first** warmup substep stalled. Cavity = 3 mm → K = 2 → first substep applies 1.5 mm of uniform engineered interference at the step-0 slide pose (intruder at `t = 1/16`, just past cap mouth). That alone is enough to push the Newton tangent non-SPD. No amount of additional warmup substeps can help if substep 1 already fails.

Sim-soft log: ~10 `faer LU fallback fired` lines with `NonPositivePivot` indices around 14–18 k, residual descending smoothly `3.05 → 0.39` over Newton iters 2–18 before Armijo gives up at α = 4.77e-7 = `0.5^21` (the documented `max_line_search_backtracks = 20` cap). Same α-collapse signature as the pre-v5 SL.3 stall and the original `cavity = 5 mm` step-0 stall, with a different r_norm floor (~0.39 vs 1.78 vs 0.572).

## 3. Root cause — the F4 mental model itself

The [[project-cavity-inset-stall-bookmark]] §9 upstream chain claimed:

> 2. **Sliding ramp's t=0 starts with full engineered interference** at every active vertex. Initial condition is the lever.

**This claim is false for pre-F4 sliding.** Pre-F4 `intruder_contact_sliding_at` produces `Solid::from_sdf(transformed_scan, bounds).offset(cavity_offset_m)` — the transformed scan SHRUNK by `cavity_inset_m`. At rest pose the intruder coincides with the un-deformed cavity wall surface (engineered interference = 0); at non-rest poses the only interference comes from pose-induced geometric mismatch (curvature-driven, small, NOT scaled by `cavity_inset_m`).

F4 took this misdiagnosis and built a "fix" that **introduced** the load class the bookmark thought was already there. The bare-scan main-loop model applies uniform engineered interference at every contact pair — a load class the sliding-mode FEM has never been calibrated against. Even **1.5 mm** of this load at the step-0 pose blows past the κ = 1 × 10³ convergence envelope edge — much smaller than the 3 mm default cavity inset the pre-F4 model was already converging through.

The empirical reading: **pre-F4 sliding mode's convergence depends on the intruder being the shrunk-scan, not the bare scan**. The shrunk-scan model is what makes step-0 tractable. F4 broke that invariant.

## 4. Where the original cavity-inset stall *actually* lives

Q1 bisection (bookmark §8) established that cavity-inset drives the step-0 stall via something. F4 guessed "engineered interference magnitude" and was wrong. The remaining mechanisms in `intruder_contact_sliding_at` that DO scale with `cavity_inset_m`:

- **`interior_cutoff = 2 × cavity_inset_m`** — the active-set filter band. Larger cavity inset → wider band → MORE contact pairs pass through to the Newton assembly. At cavity = 5 mm, the cutoff is 10 mm wide vs 6 mm at default; the additional pairs in the band have larger composed-sd magnitudes (deeper local penetrations from pose mismatch), generating higher per-pair penalty contributions, and the total Hessian-concentration in the contact zone grows. This is the same κ-concentration mechanism documented at `insertion_sim.rs:907-915` for the growing ramp.
- **Active-set composition shifts**: at the step-0 pose with the shrunk-scan intruder, the pairs that DO activate are at the cap-mouth region where the moving intruder's curvature differs from the un-deformed cavity. Larger cavity_inset_m → larger geometric mismatch in this region → some pairs see local interference at the κ-envelope edge even though the model has no "uniform engineered interference".

So cavity-inset still drives the stall, but through the **interior-cutoff width** + **pose-mismatch-magnitude scaling**, NOT through "uniform engineered interference at active vertices". The candidate fixes need to be re-scored against this corrected diagnosis.

## 5. Re-scoring §5 Q3 candidates with the corrected diagnosis

| # | What it changes | Corrected verdict |
|---|---|---|
| F1 | Adaptive `tol` scaled with cavity_inset_m | Still a band-aid, but now matches the actual mechanism: stall-floor scales with cutoff-width-induced pair-count, so adaptive tol with the same scaling could land. ~30 LOC. |
| F2 | Lower κ further (1e3 → 1e2 or 5e2) | Still tactical, still cf-device-design-only, and now better-targeted: directly attacks the Hessian concentration the larger interior_cutoff produces. The κ = 1e3 doc explicitly says the envelope edge was sized for the default 3 mm cavity; lowering κ for larger cavities is consistent with the existing tuning's logic. **Top candidate** for cheap implement-measure-revert. |
| F3 | Levenberg-Marquardt `+λI` in `factor_and_solve_free` | Sim-soft architectural. Now the strongest candidate for "fix the class of pathology, not the specific case" — would help every consumer with non-PD pivot patterns, regardless of whether the load is shrunk-scan pose-mismatch or any future bare-scan load. Bigger blast radius + sim-soft review. **Right call for next-next-session arc** if F2 doesn't reach cavity = 5/8 mm. |
| F4 | ~~Interference homotopy continuation~~ | **DEAD**. Falsified empirically; the load class it adds is the load class the sliding solver can't handle. |
| F5 | Smaller `cell_size_m` for tight designs | Untested. Indirect — finer mesh may help or hurt conditioning. Lower priority. |
| F6 | UI warning + design-space gating | Already partially landed via F4.1's 8 mm cap. Doesn't answer the mold-size question for the cavity ∈ [4, 8] mm range. |

**The next architectural ladder candidate is F2** (cheap implement-measure-revert per [[feedback-implement-measure-revert-pattern]] — single constant change, ≲ 5 LOC). If F2 doesn't reach cavity = 5/8 mm, F3 (sim-soft LM regularization) is the next-next-session arc.

## 6. What stays from the F4 attempt

- **F4.0** (`5c653b57`) — bookmark doc. KEEP. Captures the Q1 bisection + design-space framing. §9 + §10 need annotation pointing to this postmortem; §5 Q3 candidate table needs the rescored verdicts from §5 above.
- **F4.1** (`3f7691b5`) — UI slider cap at 8 mm + material-validity sub-label. KEEP. Independent of F4's solver story; the 8 mm cap is a sound bound regardless of which fix candidate eventually lands.
- **F4.2** (`26e83fa4`) — `interference_m` parameter on `intruder_contact_sliding_at`. KEEP. Pure API refactor; today's behavior preserved bit-equal at `interference_m = 0` (all 4 call sites pass 0). The parameter is dormant scaffold — useful if any future fix needs to ramp interference, costs nothing if no such fix ever lands. Removing it is a one-line edit if a cold-read later decides the scaffold isn't worth keeping.
- **F4.3** (`573e56fe`) — warmup loop + main-loop switch to bare-scan. **REVERTED** (`c0443df4`). 170 tests + clippy clean post-revert.

## 7. Three-session pattern earned its keep

The user's F4.4 falsifier rule fired exactly as designed: "if F4.3 visual gate fails, do NOT iterate in-session". Without that rule the temptation would have been to tweak the warmup pose, the K formula, the warmup pose interpolation, etc. — none of which would have helped (the failure mode is upstream: the load class itself is wrong, not the homotopy increment). The clean revert + postmortem + handoff to a fresh recon session is exactly the pattern [[feedback-bookmark-when-surface-levers-exhaust]] describes.

The implement-measure-revert per [[feedback-implement-measure-revert-pattern]] also earned its keep: ~150 LOC of F4.3 implementation produced the highest-information result possible — empirical falsification of the F4 mental model itself, with the differential signal (Newton stalls at 1.5 mm uniform interference vs converges through pose-mismatch interference at 3 mm cavity inset) reshaping the recon for the next session.

## 8. Next-session pickup

1. **Read this postmortem + [[project-cavity-inset-stall-bookmark]] §5 Q3 with §5 above's rescored verdicts**. The candidate ranking changed materially — F4 is dead, F2 is the new top candidate, F3 is the architectural backstop.
2. **Implement F2 first** (single-constant κ ramp, ≲ 5 LOC implement-measure-revert per [[feedback-implement-measure-revert-pattern]]). Try κ = 5e2 and κ = 1e2 against the cavity = 5 mm failure case. If either converges, run the visual gate at cavity 3/5/8.
3. **If F2 doesn't reach cavity = 5/8 mm**, the failure is more fundamental than a single tuning constant — escalate to F3 (sim-soft LM regularization) as the next-next-session arc. F3 design-spec session before implementation per [[feedback-bookmark-when-surface-levers-exhaust]].

## 9. Anchors

- F4.3 falsification commit ladder:
  - `c0443df4` — revert of F4.3
  - `573e56fe` — F4.3 (the falsified attempt; reverted, history preserved)
  - `26e83fa4` — F4.2 (KEPT)
  - `3f7691b5` — F4.1 (KEPT)
  - `5c653b57` — F4.0 bookmark (KEPT, needs annotation)
- User's terminal log of the failure: see chat transcript for this session.
- Original bookmark: `docs/CAVITY_INSET_STALL_BOOKMARK.md` (needs §11 annotation pointing here + §5 Q3 verdict rewrite).
- Memory entry to update: [[project-cavity-inset-stall-bookmark]] status changes from "F4.0 + F4.1 SHIPPED, F4.2 + F4.3 next session" to "F4.0 + F4.1 + F4.2 SHIPPED, F4.3 FALSIFIED + REVERTED, F2 candidate next session".
