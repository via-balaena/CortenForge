# Cavity-inset / layer-thickness stall — recon bookmark

**Status**: Q1 BISECTION RESOLVED IN-SESSION 2026-05-18 LATE-EVENING. Bookmark drafted; bisection ran inline with user driving the UI; architectural fix (F4 — interference homotopy continuation) chosen with bounded 8 mm target. See §8 for the resolved-state addendum. Implementation arc (F4.0 → F4.5 ladder) starts next session per the three-session pattern; F4.0 is this docs commit + F4.1 (slider cap) which lands same session as a low-risk decision-locking change.

**Branch**: `sim-arc/sl-4-intruder-render` (SL.4 arc just shipped; no PR yet). The stall is an existing latent that SL.4 surfaced by getting a render reliable enough to spot the slider scrub problem the moment the user tried to evaluate a non-default design.

**Surface area**: cf-device-design's sliding-mode insertion sim. Default cavity-inset + layer thickness pair converges 16/16; non-default tweaks stall at step 0 with a different failure profile than pre-v5 SL.3.

---

## 1. What was observed

### 1a. Working baseline

**Params** (auto-loaded from `~/scans/sock_over_capsule.design.toml` at tool launch):
- Cavity inset: **3 mm**
- Layer 0 (Ecoflex 00-30 + 50% Slacker → Shore 000-20): thickness **10 mm** from cavity
- Layer 1 (Dragon Skin 20A): **3 mm** Δ from prior layer
- Wall total: 13 mm
- Insertion Sim: 16 steps · 4 mm cell · Yeoh material · tol = 1e-1 · κ = 1e3

**Result**: "Converged: 16 of 16 steps — seated to 83.35 mm (full depth)". Peak contact force ~3 N at full seating. Wall-clock ~3-4 min. The 16/16 converge matches the recon-arc CR.4 record.

User-driven SL.4.3 visual gate PASS on these params.

### 1b. Failing tweak

**Params** (UI-tweaked from the working baseline):
- Cavity inset: **5 mm** (3 → 5)
- Layer 0: thickness **5 mm** (10 → 5)
- Layer 1: **5 mm** (3 → 5)
- Wall total: 10 mm (13 → 10)
- Same `16 steps · 4 mm cell · Yeoh · tol = 1e-1 · κ = 1e3`

**Result** (UI error banner):
> Error: sliding ramp failed at step 0 — no converged step. Armijo line-search stalled at Newton iter 63 (r_norm 5.721815018853527e-1, final α 4.76837158203125e-7). Likely causes: non-SPD tangent near solution (spec §3 R-2 violation), or near-singular condensed system.

The terminal also shows ~30 `sim-soft: faer LU fallback fired at factor_and_solve_free` lines per Newton iter, with `NonPositivePivot { index: ... }` — the condensed system was repeatedly non-SPD throughout the run, not just at the final iter.

### 1c. Stack contrast with pre-v5 SL.3 stall

| Signal | Pre-v5 SL.3 (`f2de9a8d`, fixed by recon-arc) | This stall (defaults vs tweaked) |
| --- | --- | --- |
| Failure step | 0 | 0 |
| Newton iter at stall | 41 | 63 |
| `r_norm` at stall | 4.14 | 0.572 |
| `α` collapse | 4.77e-7 | 4.77e-7 |
| Active-set firing | Body-bulk vertices through transformed-SDF (gross interference) | Real near-cavity contact (the v5 interior_cutoff is doing its job) |
| Cause | Spec §3 R-2 transformed-SDF semantics ignored deep interior | NEW — small residual + non-SPD tangent → tol=1e-1 floor sits ~6× below the stall floor |

The α=4.77e-7 coincidence is just the Armijo backtrack count: `0.5^20 ≈ 9.5e-7` and `0.5^21 ≈ 4.77e-7` — `max_line_search_backtracks = 20` (sim-soft default) means α traces the same geometric sequence regardless of root cause. The DIFFERENTIAL is the r_norm at which Armijo gives up: 4.14 (pre-v5) vs 0.572 (now). 7.6× smaller, meaning we're MUCH closer to a feasible Newton solution but the tangent is non-SPD enough that the line search can't make local progress.

---

## 2. What the codebase already knows

The exact failure mode is *documented* in cf-device-design as a known pathology at the existing tuning's tol floor:

- `tools/cf-device-design/src/insertion_sim.rs:878-893` — `INSERTION_SOLVE_TOL = 1e-1`:
  > `1e-1` is not arbitrary — the 7.3b.1 finding is that the deeper ramp steps Armijo-stall (non-SPD tangent near the solution; the capsule geometry's secondary pathology) at a residual floor right around `0.1 N`. Setting `tol` at that floor converts those stalls into clean (loose-but-physically-exact) convergences, which is what lets the ramp seat the intruder to a meaningful depth.

- `tools/cf-device-design/src/insertion_sim.rs:907-915` — `INSERTION_CONTACT_KAPPA = 1.0e3`:
  > 7.3b.1 found `PenaltyRigidContact::new`'s default `1e4` keeps the Newton tangent non-SPD near the solution past ~1.5 mm interference — full-surface contact (the *whole* cavity wall engages at once, unlike the rows' localized probe) concentrates the penalty Hessian. A gentler `1e3` widens the convergeable depth envelope; the tradeoff is slightly more residual penetration, acceptable for this relative-comparison tool (Fork B).

- `sim/L0/soft/src/solver/backward_euler.rs:969-974` — the panic message that fired:
  > "Armijo line-search stalled … non-SPD tangent near solution (spec §3 R-2 violation), or near-singular condensed system."

### What this implies

The 7.3b.1 tuning was done against the **default params** (3 mm cavity inset, default layers). The tol-at-stall-floor trick (tol = 1e-1) is a knife-edge calibration: r_norm must drop UNDER 1e-1 *before* Armijo stalls for the run to converge.

**Hypothesis Z (frontrunner)**: tweaking cavity inset 3 → 5 mm (or possibly the layer composition) shifts the stall floor higher than 1e-1, so the trick no longer catches the stall and Newton runs out the Armijo cap. Concretely r_norm ≈ 0.572 > 0.1.

This is *consistent* with the docstring's own warning that the κ=1e3 envelope was widened "past ~1.5 mm interference" but doesn't promise an unlimited interference range. Cavity inset 5 mm ≡ 5 mm engineered interference — well past 1.5 mm.

---

## 3. What changed between baseline and failure

Three params moved simultaneously:

| Param | Baseline | Failure | Δ |
| --- | --- | --- | --- |
| `cavity_inset_m` | 3 mm | 5 mm | +67% |
| Layer 0 `thickness_m` (innermost) | 10 mm | 5 mm | -50% |
| Layer 1 `thickness_m` (outermost) | 3 mm | 5 mm | +67% |

These are three independent levers in the design space and we don't yet know which one(s) cause the stall. **Bisection is the next experiment**.

Knobs *not* changed: cell size (4 mm), Yeoh material, κ (1e3), d̂ (1 mm), max_newton_iter (150), max_line_search_backtracks (20), Newton tol (1e-1), intruder geometry, centerline, cap planes, layer materials.

---

## 4. Why this matters (product framing)

The user's stated goal for the sim is "tell us the exact size of each mold for each layer." That goal requires the sim to converge across the full design space they want to explore — at minimum:
- cavity_inset_m ∈ [2 mm, 6 mm]-ish (engineered interference for fit)
- Layer count ∈ {1, 2, 3+}
- Layer thickness ∈ [2 mm, 15 mm]-ish per layer
- Material × Slacker combos across the supported list

A sim that only converges at one specific design point doesn't give the user any decision-making power. Fixing this is on the critical path to workshop iter-1's mold-size question being answerable.

---

## 5. Recon questions for the fresh session

### Q1 (sequenced FIRST — gating the rest): which param change drives the stall?

Run two more sims to bisect:
- **Test A**: cavity_inset = 5 mm + layers 10+3 mm (cavity-only change). If A stalls, cavity inset is the driver and layer composition is a passenger.
- **Test B**: cavity_inset = 3 mm + layers 5+5 mm (layers-only change). If B converges, layer composition is innocent. If B stalls, layers contribute too.

Best case (one-driver): A stalls, B converges → cavity inset is the lone culprit; recon narrows to "raise stall-floor envelope past 1.5 mm interference."

Worst case (interaction): A converges, B stalls → layer composition is the driver. Or A stalls AND B stalls → both contribute (need a 2D map of the converge/stall boundary).

The 7.3b.1 docstring strongly implicates cavity inset as the primary lever (it's the "interference past ~1.5 mm" knob the κ=1e3 widening was sized against), so the prior is one-driver-cavity. But Q1 should be empirical, not theoretical.

Wall-clock: ~3-4 min per test on iter-1 sock.

### Q2: is the stall floor monotonic in cavity inset?

If A from Q1 is the stall driver, sweep cavity inset 3 → 6 mm at 0.5 mm step, watch r_norm at stall. Three possibilities:
- Smooth monotonic increase → simple "extend the κ-vs-tol calibration" recipe.
- Step discontinuity at some inset → topology change (more contact pairs, new mesh feature), different intervention.
- Non-monotonic / chaotic → the pathology is fragile to fixture-specific geometry and would force per-fixture tuning.

### Q3: candidate fix surfaces (don't pick yet — let Q1+Q2 narrow first)

Several levers exist; not all are equal in blast radius:

| Candidate | Lever | Blast radius | Trust |
| --- | --- | --- | --- |
| **F1 — adaptive tol** | Pick `tol = max(1e-1, 0.5 × estimated_stall_floor)` where stall-floor scales with cavity_inset_m. | cf-device-design-only (one constant becomes a function). | Medium — relies on understanding scaling; if non-monotonic, recipe is fragile. |
| **F2 — lower κ further** | Drop to `5e2` or `1e2` for tight-inset designs; widens convergence envelope per the existing docstring's logic. | cf-device-design-only (one constant). Trades residual penetration. | Medium — proven mechanism at the 1e4→1e3 step; question is how far it can go. |
| **F3 — Levenberg-Marquardt-style regularization in Newton step** | Add `+ λI` to the assembled tangent to ensure SPD; `λ` adapts to non-PD pivot count from the LU fallback. | sim-soft (touches `factor_and_solve_free`). | High mechanism, but it's a real Newton-method change with implications for every consumer. |
| **F4 — interference-ramp warmstart** | If the engineered interference at step 0 is "too large", first sub-step to a smaller interference, warmstart from that solution. | cf-device-design-only. Doubles wall-clock on hard cases. | High mechanism, but adds a control loop. |
| **F5 — smaller cell_size_m for tight designs** | Halve `cell_size_m` for finer mesh resolution when cavity_inset_m is below a threshold. | cf-device-design-only. Increases wall-clock and memory ~8×. | Medium — finer mesh sometimes helps Newton conditioning, sometimes hurts. |
| **F6 — accept the limit + UI warning** | Document the convergence envelope in the tool, gray out unsupported design-space regions. | cf-device-design-only. No solver work; punts the question to the user. | Low value — doesn't actually answer the mold-size question. |

The recon session should pick 1-2 candidates after Q1+Q2 narrow the failure surface, then a third session implements.

### Q4: is `max_line_search_backtracks = 20` the binding constraint?

At α = 4.77e-7 = `0.5^21`, the backtrack hit 20 ÷ 21 iterations. If we raised the cap to 40 (`0.5^40 ≈ 9e-13`), would the search find a working α? Probably not — the Armijo condition `trial_norm ≤ (1 - α·c1)·r_norm` reduces to `trial_norm ≤ r_norm` as α → 0, which means the search is genuinely stuck against a non-PD Hessian, not slowness. But this is a 1-line config tweak to verify cheaply.

### Q5: is the LU fallback's non-PD pivot count informative?

The terminal log showed `sim-soft: faer LU fallback fired at factor_and_solve_free at Newton iter N` repeated for every iter, with `NonPositivePivot { index: M }` — the system is non-PD at every iter, not just the failing one. If we could surface the pivot-failure count as a `SlideRampStep.readout` signal, the user could see the conditioning degrade in real time before the Armijo cap hits. This is an observability lever, not a fix, but useful for Q2's sweep.

---

## 6. Anchors for the recon session

### Critical files

- `tools/cf-device-design/src/insertion_sim.rs:876-920` — solver tuning constants + their rationale comments. The `INSERTION_SOLVE_TOL` and `INSERTION_CONTACT_KAPPA` docstrings are 7.3b.1's reasoning preserved in-tree.
- `tools/cf-device-design/src/insertion_sim.rs:1040-1080`-ish — `run_sliding_insertion_ramp` call site (where solver_config gets composed).
- `tools/cf-device-design/src/insertion_sim_ui.rs:780-815` — `poll_simulation_task` (where `last_error` gets populated from the panic).
- `sim/L0/soft/src/solver/backward_euler.rs:81-110` — `SolverConfig::skeleton()` defaults (max_newton_iter=10, max_line_search_backtracks=20, etc.).
- `sim/L0/soft/src/solver/backward_euler.rs:870-910` — Newton main loop.
- `sim/L0/soft/src/solver/backward_euler.rs:912-975` — Armijo backtrack + panic site.
- `sim/L0/soft/src/contact/penalty.rs:273` (approx) — `PenaltyRigidContact::active_pairs`.

### Relevant memories

- [[project-v2-cf-cast-arc-complete]] — 7.3b.1 lives in this history; the κ + tol calibration is the cited finding.
- [[project-sliding-intruder-contact-recon-shipped]] — v5 interior_cutoff fix; the SL.3 stall pattern that this stall now resembles in failure mode (Armijo cap) but differs in mechanism.
- [[project-sl-4-arc-shipped]] — context for why this stall is now visible (SL.4 made the sim's user-facing surface scrubable; user immediately tried a non-default design).
- [[feedback-implement-measure-revert-pattern]] — for the candidate fixes in §5 Q3 that are ≲200 LOC, implement→measure→keep-or-revert is faster than over-analyzing.
- [[feedback-bookmark-when-surface-levers-exhaust]] — the three-session pattern this bookmark is starting.

### Terminal artifact (preserve for recon)

```
sim-soft: faer LU fallback fired at factor_and_solve_free at Newton iter 1 (free residual norm 6.877863561106156e1)
  non-PD pivot: NonPositivePivot { index: 8531 }
sim-soft: faer LU fallback fired at factor_and_solve_free at Newton iter 2 (free residual norm 5.890434559732861e1)
  non-PD pivot: NonPositivePivot { index: 8202 }
[... ~60 iters ...]
sim-soft: faer LU fallback fired at factor_and_solve_free at Newton iter 63 (free residual norm 5.721815018853527e-1)
  non-PD pivot: NonPositivePivot { index: 8766 }
thread 'Async Compute Task Pool (2)' panicked at sim/L0/soft/src/solver/backward_euler.rs:969:9
Armijo line-search stalled at Newton iter 63 (r_norm 5.721815018853527e-1, final α 4.76837158203125e-7).
```

Residual drops smoothly from ~69 → 0.57 over 62 Newton iters (no oscillation), then Armijo can't proceed. Classic "near a non-smooth feasible region" signature — not divergence, not chaos, just locally non-descent.

---

## 7. What to do next session

1. **Run the bisection** (Q1) — two ~3-4 min sims, then we know which lever dominates.
2. **Pick 1-2 candidates** from §5 Q3 informed by Q1.
3. **Either implement-measure-revert** ([[feedback-implement-measure-revert-pattern]]) if pick is ≲200 LOC, or write a recon-spec doc if blast radius is bigger.
4. If candidate fix lands, **re-verify SL.4 visual gate doesn't regress** at default params (the v5 interior_cutoff regression sentinel at `insertion_sim.rs:4878` catches gross regressions automatically).

Bisection is the highest-information cheap action. Until we know whether cavity inset or layer composition is the driver, the §5 Q3 candidates are too unconstrained.

---

## 8. Q1 bisection — RESOLVED IN-SESSION

The user ran Q1 inline this session. Two ~3-4 min sims:

| Run | Cavity | L0 | L1 | Result | Stall r_norm | Peak F | Notes |
|-----|--------|----|----|--------|--------------|--------|-------|
| Baseline | 3 mm | 10 mm | 3 mm | **16/16 OK** | n/a | 3.0 N | SL.4.3 visual gate baseline. |
| Run 1 (L0 only) | 3 mm | **5 mm** | 3 mm | **15/16** | n/a (stalls last step) | **8.4 N** | Partial converge; stalls on the FINAL step (full seating). |
| Run 2 (cavity only) | **5 mm** | 10 mm | 3 mm | **stall step 0** | **1.78** | n/a | r_norm trajectory: ~57 → 30 → 10 → 2 → plateau at 1.78, no oscillation. Clean stall floor. |
| Original failure (for reference) | **5 mm** | **5 mm** | **5 mm** | stall step 0 | 0.572 | n/a | Both pathologies stacked. Step-0 stall wins. |

### Two independent pathologies confirmed

1. **Cavity inset → step-0 stall** (the load-bearing one). When engineered interference exceeds the κ=1e3 convergence envelope (~1.5 mm per the in-tree docstring at `insertion_sim.rs:907-915`), Newton's tangent goes non-SPD at *initial contact*. Doesn't matter what the layers do — the contact pressure overwhelms the solver before any silicone deformation matters. This is the §2 Hypothesis Z, confirmed.

2. **Layer thinning → step-N stall** (a separate, milder issue). Thinner Layer 0 → less material to absorb interference → higher reaction force at deep seating → eventual stall at the last step under load. Banked as a follow-up sim arc; doesn't need to block F4.

When BOTH are present (original failure), the cavity pathology wins because it fails first. Surprising finding: Run 2's r_norm (1.78) is **3× the original failure's** (0.572) — thicker Layer 0 means MORE silicone tets in active contact at step 0 = more residual contributions. The "easier" thin-layer case was actually easier because fewer pairs were active. Useful intuition for predicting which configs will be hardest.

### Banked render artifact

Run 1's spiky-yellow halo around the silicone in the viewport (5 mm L0) was GONE in Run 2 (10 mm L0). Confirms it's a Layer-0-thinning render artifact (BCC tets visible through the thinner shell), not a sim-side issue. Bank as low-priority cf-device-design render polish for later.

---

## 9. Architectural decision — bounded design surface + F4

### "Is F2 the most upstream fix?" — explicit chain

The user asked the architectural question. Honest answer: F2 (lower κ) is the most TACTICAL fix, not the most upstream.

Upstream chain:
1. **Cavity inset is an engineering design knob with up to N mm interference at every contact vertex.** Product, not bug.
2. **Sliding ramp's t=0 starts with full engineered interference** at every active vertex. Initial condition is the lever.
3. **Penalty contact + large interference + many concentrated pairs → combined tangent non-SPD** (spec §3 R-2 violation).
4. Newton's search direction from non-PD tangent isn't descent → Armijo can't reduce residual → stall.

The candidates re-scored on "upstream-ness":

| # | What it changes | Layer | Verdict |
|---|---|---|---|
| F1 | Newton's convergence threshold | Layer 1 (give up sooner) | Pure band-aid |
| F2 | Penalty stiffness κ | Layer 3 (model parameter) | Tactical patch |
| F3 | `+λI` Levenberg-Marquardt on the assembled tangent | Layer 4 (Newton Hessian) | Sim-soft architectural — fixes non-PD class generally; bigger blast radius |
| **F4** | **Sub-step cavity_inset 0 → target via warmstart chain** | **Layer 2 (initial condition)** | **cf-device-design architectural — treats engineered interference as solver-conditionable input** |
| F5 | Mesh resolution | n/a | Indirect, may regress as much as helps |
| F6 | UI warning | n/a | Punt, doesn't answer mold-size question |

**F4 chosen** as the architectural fix for this specific problem (homotopy continuation — textbook nonlinear FEM technique). Contained to cf-device-design; doesn't touch sim-soft; generalizes to any cavity_inset within material-validity bounds.

F3 (LM regularization) banked as future sim-soft work — would help every consumer's non-PD pathology, but bigger scope + bigger review.

### Bounded design surface — 8 mm cap

Original cavity_inset slider goes to 15 mm. That ceiling wasn't engineering-derived; it was generous UI default. For iter-1 sock (71 mm diameter):

| Inset | Strain | What it means physically |
|-------|--------|--------------------------|
| 2-3 mm | ~5% | Comfortable compression fit (medical compression sock territory) |
| 4-6 mm | ~10% | Tight engineered grip, normal product range |
| 7-8 mm | ~15-20% | Aggressive interference fit, edge of comfort |
| 10 mm | 28% | Approaching circulation-impact zone |
| 15 mm | **42%** | Tourniquet territory — not a wearable design |

At ≥10 mm, **Yeoh material model validity** is also exceeded for typical layer thicknesses — the silicone stretch ratio leaves the published Yeoh constants' physically meaningful range. Even if FEM converges, engineering scalars are unreliable (math fiction, not fit prediction).

**Decision**: cap UI slider at **8 mm**. Covers all reasonable product designs + room for aggressive-grip exploration; stays inside Yeoh validity for typical layer thicknesses; gives F4 a bounded design space to be reliable against (~3× the current default, not 5×).

Future: if physical experimentation surfaces a need for >8 mm cavity shrink, revisit. The slider cap is conservative-bounded, not architecturally fixed.

---

## 10. F4 — implementation ladder

### Sub-leaves

| # | Headline | Crate | Est LOC |
|---|----------|-------|---------|
| **F4.0** | Commit this bookmark + Run 1/2 results + 8 mm cap architectural decision (docs only) | docs | this commit |
| **F4.1** | Cap UI slider at 8 mm + add sub-label explaining material-validity reasoning | cf-device-design | ~10 + tests |
| **F4.2** | Thread `interference_m` homotopy parameter through `intruder_contact_sliding_at` (analogous to existing growing-mode builder at `insertion_sim.rs:922`). At `interference_m = cavity_inset_m` reproduces today's behavior bit-equal | cf-device-design | ~30 + tests |
| **F4.3** | Warmup loop in `run_sliding_insertion_ramp`. Before t=0, run K sub-steps with `interference_m_k = k · cavity_inset_m / K`, warmstarting each. `K = ceil(cavity_inset_m / 1.5 mm).clamp(0, 8)`. The 1.5 mm step size derives from the documented κ=1e3 envelope edge | cf-device-design | ~60 + tests |
| **F4.4** | User-driven visual gate at cavity 3 / 5 / 8 mm. All three must converge 16/16. Falsifier: if 5 mm stalls with F4, escalate to F3 (LM regularization) as next-session arc | runtime check | n/a |
| **F4.5** | Cold-read polish per [[feedback-cold-read-review-post-ship]] | cf-device-design | bundle |

### Session split

- **This session**: F4.0 (this commit) + F4.1 (slider cap, low-risk UI bound). Locks the architectural decisions; doesn't touch sim core.
- **Next session (fresh eyes)**: F4.2 + F4.3 + F4.4 — the load-bearing mechanism + visual gate. Bookmark + slider cap from session 1 serve as in-tree anchors.
- **Session after**: F4.5 cold-read polish.

Rationale for split: per the three-session pattern, doing implementation in the same session as the recon risks same-blind-spots. F4.3 (warmup loop) is where homotopy-continuation surprises would hide — wants fresh eyes. F4.0 + F4.1 are low-cognitive-load and don't carry that risk.

### Expected wall-clock cost of F4.3

- Each warmup sub-step solves a 1.5 mm-incremental contact problem warmstarted from the previous solution.
- K = 2 at cavity_inset = 3 mm (default) → +1-2 min wall-clock
- K = 4 at cavity_inset = 5 mm → +2-3 min
- K = 6 at cavity_inset = 8 mm → +3-5 min

Acceptable — and the warmstart should mean each sub-step converges in few Newton iters, much faster than a cold solve.

### F4 falsification plan

If F4.3 visual gate fails at cavity = 5 mm (the original failure case), do NOT iterate inside the session. Bookmark + escalate to F3 (LM regularization, sim-soft side) as next-session arc. The three-session pattern earns its keep precisely when "the obvious fix doesn't work."
