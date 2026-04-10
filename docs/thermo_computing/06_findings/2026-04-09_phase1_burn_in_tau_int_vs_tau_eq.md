# Phase 1 finding — burn-in heuristic conflates τ_int with τ_eq (Crack 4)

> **Status**: open finding, fix plan ready, executing in this session.
> **Discovered**: 2026-04-09 during the Phase 1 fix execution session, on the first run of the §7.4 sweep test under `--ignored` after Cracks 1+2+3 were fixed.
> **Branch**: `feature/thermo-doc-review`
> **Affected files**: `03_phases/01_langevin_thermostat.md` (§7.2 parameter table + statistical accounting, §7.4 sweep), `sim/L0/thermostat/tests/langevin_thermostat.rs` (test_equipartition_central_parameter_set + test_equipartition_sweep_gamma_t)

## TL;DR

The Phase 1 spec §7.2 parameter table sets `n_burn_in = 25_000 steps (= 5·τ_int)` with `τ_int ≈ M/(2γh)`. The chassis derives `τ_int` correctly as the **steady-state autocorrelation time of `½v²`** (line 1463 of `02_foundations/chassis_design.md`), which is the right quantity for computing N_eff in a Welford-based estimator. **It is NOT the right quantity for choosing burn-in length.** Burn-in adequacy is governed by the equilibration time `τ_eq = M/γ`, which is **2× longer than τ_int** for an underdamped Langevin SHO.

For the central case (γ=0.1), `n_burn_in = 25_000 steps = 25 time units = 2.5·τ_eq`. Residual energy fraction missing after burn-in: `exp(−2.5) ≈ 8.2%`. Time-averaged over the 100,000-step measurement window, this gives a systematic bias of **0.82% of ½kT** — below the 4.5% std error tolerance, so the test passes, but at *threshold*, not with margin.

For the sweep at γ=0.01, `τ_eq = 100 time units` and `n_burn_in = 25 time units = 0.25·τ_eq`. The system is only 22% equilibrated when measurement starts, and the measurement window itself is only 1·τ_eq long, so most of the window is during continued relaxation. Predicted bias: **49% of ½kT**. Observed: 50.2% (measured = 0.1256, expected = 0.25). **Quantitative match to 1% relative.**

The chassis is correct as written. The conflation is at the spec level: §7.2 transcribed "5·τ_int" as the burn-in heuristic without recognizing that τ_int (autocorrelation) and τ_eq (equilibration) are different quantities that happen to differ by a factor of 2 for the underdamped Langevin SHO.

**Fix is structural**: name `τ_eq = M/γ` explicitly in §7.2, scale n_burn_in and n_measure to τ_eq instead of τ_int, and parameterize the §7.4 sweep per combo.

## The three time scales

An underdamped Langevin SHO with `M=1`, `k=1`, damping `γ`, and `kT` has **three distinct time scales** that must not be conflated:

| Quantity | Rate | Time constant | Where it's used |
|---|---|---|---|
| Amplitude decay (`x(t)`, `v(t)` envelope) | `γ/(2M)` | `2M/γ` | §10 stochastic gating decay test (Crack 3 fix) |
| `⟨v²⟩` equilibration from cold start | `γ/M` | `τ_eq = M/γ` | **§7 burn-in adequacy** |
| `½v²` autocorrelation in steady state | `2γ/M` | `τ_int = M/(2γ)` | §7 N_eff for std error |

The factor of 2 between energy decay (`γ/M`) and amplitude decay (`γ/(2M)`) is because energy ∝ amplitude². Crack 3 was about not conflating amplitude decay with energy decay; this finding is about not conflating energy *equilibration* with `½v²` *autocorrelation*. They both equal `M/(2γ)` numerically when written one way and `M/γ` the other, but their physical meanings are distinct.

For γ=0.1, h=0.001:
- Amplitude time constant: `2M/γ = 20 time units = 20,000 steps`
- Equilibration time constant `τ_eq = M/γ = 10 time units = 10,000 steps`
- `½v²` autocorrelation time `τ_int = M/(2γ) = 5 time units = 5,000 steps`

For γ=0.01, h=0.001:
- Amplitude time constant: `2M/γ = 200 time units = 200,000 steps`
- Equilibration time constant `τ_eq = M/γ = 100 time units = 100,000 steps`
- `½v²` autocorrelation time `τ_int = M/(2γ) = 50 time units = 50,000 steps`

## Where the conflation lives

### Chassis (CORRECT)

`02_foundations/chassis_design.md:1463-1469`:

> "the velocity-squared autocorrelation decays at rate `2γ/M`, giving an integrated autocorrelation time of approximately `M / (2γh)` steps. With M=1, γ=0.1, h=0.001:
> ```
> τ_int ≈ 1 / (2 · 0.1 · 0.001) = 5000 steps
> ```
> So 10⁵ steps gives an *effective* sample count of:
> ```
> N_eff ≈ 10⁵ / (1 + 2·5000) ≈ 10
> ```"

The chassis derives `τ_int` as the v² autocorrelation time and uses it correctly in the N_eff formula. **No conflation in the chassis.** The chassis text never claims this quantity governs burn-in adequacy.

### Spec §7.2 (CONFLATED)

`03_phases/01_langevin_thermostat.md` parameter table (pre-fix):
> `n_burn_in | 25_000 steps (= 5·τ_int) | τ_int ≈ M/(2γh) = 5000 (recon log part 2)`

The "(= 5·τ_int)" annotation uses τ_int as the burn-in scale. **This is where the conflation enters.** A 5·τ_int burn-in is `5·M/(2γ) = 2.5·τ_eq` time units of equilibration. For γ=0.1 the system is only 91.8% equilibrated when measurement starts (`1 − exp(−2.5) = 0.918`), and the measurement window has to drag the running average the rest of the way to ½kT.

The spec author may have transcribed "5·τ_int" as a generic "discard initial transients" multiplier without distinguishing autocorrelation time from equilibration time. Both quantities are real, both are derived in the chassis or in standard Langevin theory, and the names are similar enough that the conflation is easy to make.

### Spec §7.4 sweep (PROPAGATED)

The sweep test description says: "Smaller per-combination trajectory count (`n_traj = 30`) keeps total cost bounded; the central combination is already proven by §7.3 so the sweep is verification, not the gate."

It does not parameterize `n_burn_in` or `n_measure` by `γ`. The implementation hardcodes the same 25,000 / 100,000 step counts as the central case. For γ=0.01, where `τ_eq = 100,000 steps = 4× n_burn_in`, this leaves the system catastrophically un-equilibrated.

### Implementation (FAITHFUL)

`tests/langevin_thermostat.rs::test_equipartition_sweep_gamma_t` translates §7.4 verbatim into code with the same hardcoded burn-in. The bug is not in the implementation; the implementation is a faithful translation of a spec that conflates two time scales.

## Quantitative model

For burn-in time `t_b` and measurement time `t_m`, with energy relaxation from cold start `E(t) = kT·(1 − exp(−t/τ_eq))`:

```
Mean of ½v² over measurement window [t_b, t_b+t_m]
   = (kT/2) · {1 − (τ_eq/t_m) · exp(−t_b/τ_eq) · [1 − exp(−t_m/τ_eq)]}
```

The term in braces is the equilibration efficiency. Define the **bias fraction**:

```
b = (τ_eq/t_m) · exp(−t_b/τ_eq) · (1 − exp(−t_m/τ_eq))
```

`b` is the relative shortfall of the measurement-averaged `½v²` from the equilibrium target `½kT`.

### Central case (γ=0.1) prediction

`τ_eq = 10`, `t_b = 25`, `t_m = 100`:

```
b = (10/100) · exp(−2.5) · (1 − exp(−10))
  = 0.1 · 0.0821 · 1.0
  = 0.00821 ≈ 0.82%
```

Predicted measured grand mean: `0.5 · (1 − 0.00821) = 0.4959`.

### Probe F observation (central case, 6 seeds, from propagation-chain finding)

From `06_findings/2026-04-09_phase1_statistical_propagation_chain.md`:

| seed_base | mean |
|---|---|
| 0xC0FFEE | 0.5029616596 |
| 0xBADF00D | 0.4972875317 |
| 0xCAFEBABE | 0.4907776035 |
| 0xDEADBEEF | 0.4652127813 |
| 0xFEEDFACE | 0.5132946297 |
| 0xFACEF00D | 0.5097218707 |

6-seed average: `0.4965`. Distance from prediction `0.4959`: **+0.0006**, or about `0.06σ` of the 6-seed std-of-mean (`std/√6 ≈ 0.024/2.45 ≈ 0.010`).

**The burn-in bias model is more consistent with the observed Probe F average than a zero-bias model is.** A zero-bias prediction would put the 6-seed average at `0.5000` exactly; the observed `0.4965` is `−0.34σ` from that. The burn-in model puts it at `0.4959`; the observed is `+0.06σ` from that. Neither is statistically distinguishable from the data with only 6 seeds, but the systematic *direction* and *magnitude* match the burn-in model with surprising precision.

The previous session's Probe F analysis correctly characterized the data as "consistent with zero — pure sampling noise" but lacked the τ_int/τ_eq distinction in the model space, so it could not identify the small systematic offset as a load-bearing signal.

### Sweep case (γ=0.01, kT=0.5) prediction

`τ_eq = 100`, `t_b = 25`, `t_m = 100`:

```
b = (100/100) · exp(−0.25) · (1 − exp(−1))
  = 1 · 0.7788 · 0.6321
  = 0.4923 ≈ 49%
```

Predicted measured grand mean: `(½ · 0.5) · (1 − 0.4923) = 0.25 · 0.5077 = 0.1269`.

### Sweep observation (γ=0.01, kT=0.5)

```
measured  = 0.1256
expected  = 0.25
deviation = 0.1244
std_error = 0.0155
n_sigma   = 8.04
```

Predicted: `0.1269`. Observed: `0.1256`. **Difference: 0.0013, or `0.08σ` of the test's reported std error.** Quantitative match to 1% relative.

This is overwhelming confirmation that the diagnosis is correct. The sweep is not failing because of bad statistics or a flake — it is failing because the system has not equilibrated, and the magnitude of failure is exactly what the equilibration physics predicts.

## Why the previous session didn't catch it

The previous session ran Probes A–F focused on the central test (γ=0.1) and the §10 decay test. The §7.4 sweep is `#[ignore]`'d so was never executed. The Probe F 6-seed analysis showed `mean of deviations = −3.46e−3` with the conclusion "consistent with zero — pure sampling noise." This conclusion is *statistically correct* given the data — but the *direction* of the deviation (negative) and the *magnitude* (~0.7% relative) match a real burn-in residual, and the previous session had no model for that residual in mind.

The handoff-vindication finding (`06_findings/2026-04-09_phase1_handoff_principle_vindication.md`) anticipated this exact pattern:

> "Be ready for chassis-level breaks, especially during closing review."

The current finding is not a chassis break (the chassis is correct), but it is a spec-level break that hides in the gap between two correctly-derived but distinct quantities. The pattern is the same as Cracks 1+2: a conceptual conflation propagates through paper passes because each pass reads the text and recopies the inference without re-deriving the physics.

**This is the second iteration of the recon-to-iteration handoff principle on the thermo line.** Phase 1 has now produced two independent confirmations that paper passes systematically miss external-correctness errors and that running code is the only reliable check.

## Fix plan

### Layer A — sweep test (definitely needed)

Per-combo τ_eq-scaled parameters:

| γ | τ_eq (steps) | n_burn_in (5·τ_eq) | n_measure (20·τ_eq) | per-traj steps | per-row cost |
|---|---|---|---|---|---|
| 0.01 | 100,000 | 500,000 | 2,000,000 | 2,500,000 | 30 traj × 3 kT × 2.5M = 225M |
| 0.1 | 10,000 | 50,000 | 200,000 | 250,000 | 30 × 3 × 250k = 22.5M |
| 1.0 | 1,000 | 5,000 | 20,000 | 25,000 | 30 × 3 × 25k = 2.25M |
| **Total** | | | | | **~250M sim steps** |

Estimated runtime in release mode: ~25 seconds (slow row dominates). Within the docstring's "order-of-minutes" budget.

After the fix, residual bias for every combo:
```
b = (1/20) · exp(−5) · (1 − exp(−20))
  = 0.05 · 0.00674 · 1
  = 3.4e−4 ≈ 0.034%
```

That's `<< 1%` for all combos and `<< std_error` for any reasonable n_traj. **The sweep would pass with margin at every combination.**

### Layer B — central test (recommended)

Update the central case to use the same τ_eq-based scaling:

| Parameter | Old | New | Rationale |
|---|---|---|---|
| `n_burn_in` | 25,000 | 50,000 | `5·τ_eq` instead of `5·τ_int`; residual bias 0.82% → 0.034% |
| `n_measure` | 100,000 | 200,000 | `20·τ_eq` instead of `20·τ_int`; doubles N_eff per traj from ~10 to ~20 |
| `n_traj` | 100 | 100 | unchanged |

With the new parameters, std error of grand mean improves from `~4.5% of ½kT` to `~3.2% of ½kT` (factor of `√2`), and the systematic bias drops from 0.82% to 0.034%. **The test now passes with real margin** instead of by accidental cancellation.

Runtime cost: roughly 2× the current ~5 seconds, so ~10 seconds in release mode. Still order-of-seconds, still acceptable for CI.

The §7.2 parameter table and statistical-accounting paragraph need updates to reflect the new numbers and to name `τ_eq` explicitly as the equilibration time (distinct from `τ_int`).

### Layer C — chassis (NOT needed)

The chassis text at lines 1463-1469 is correct. It derives `τ_int` as the v² autocorrelation time and uses it correctly for N_eff. **No chassis edits required.** Optional polish: a forward link from chassis Decision 5 to this finding so future spec authors don't repeat the conflation. That polish is deferred per the chassis paper-lock principle.

## Open questions

### Should the chassis grow a forward link to this finding?

Same open question as Cracks 1+2 (option A vs option B in the propagation-chain finding). Same recommendation: **option A — chassis stays paper-locked, this finding is the canonical correction.** A future session deciding to fold corrections into the chassis would amend Decision 5 to add a "see also" pointer at line ~1469.

### Should the central test's seed_base be revisited?

Currently `0x00C0_FFEE_u64`. After the burn-in fix, the deviation will shift toward zero (bias 0.82% → 0.034%). The 6-seed Probe F analysis was for the *old* parameters; the new central test's seed-to-seed behavior is technically unverified.

**Recommendation: keep `0xC0FFEE`.** If the new test fails on the first run with the corrected parameters, that's an immediate signal that the diagnosis is incomplete and we should investigate. Don't pre-emptively change the seed to avoid an outcome we don't expect.

### Does this affect any future phase?

Yes — every phase that runs equipartition or any other equilibration-based test will need to choose burn-in based on `τ_eq`, not `τ_int`. The two quantities should be derived separately in any future spec that uses them. Phase 2 (free + articulated body equipartition) is the immediate next consumer; the Phase 2 spec should cite this finding when locking its burn-in choice.

## Reproducibility

The Probe F data quoted above is from `06_findings/2026-04-09_phase1_statistical_propagation_chain.md`, which preserves the original 6-seed measurements verbatim. The sweep failure is reproducible by running:

```
cargo test -p sim-thermostat --release --test langevin_thermostat \
    -- --ignored test_equipartition_sweep_gamma_t
```

against the `c1b24ad` commit (post-Cracks 1+2+3 fix, pre-Crack 4 fix). The first failing combination is `γ=0.01, kT=0.5` with `measured = 0.1256, n_sigma = 8.04`.

After this finding's fixes are applied (a future commit on this branch), the same command should pass cleanly with all 9 combinations within ±3σ of their expected values.

## Closing observation

This is the second time on this branch that a quantitative model derived from first principles has predicted a code-level outcome to within 1% relative error. The first was Crack 3 (Probe A's eigenvalue analysis matching observed amplitude decay to ~2.5%). This is the second (burn-in bias model matching observed sweep failure to ~1%).

The pattern: when the physics is well-understood and the implementation is correct, **code-level failures are loud, deterministic, and quantitatively explicable**. The previous session's vindication finding called this out; this finding is its second confirmation.

The handoff principle continues to deliver. The cracks keep being foundational and load-bearing, not peripheral. **Phase 1 has now produced two cracks the previous session missed (the merge std error error and the §10 time constant) plus one crack this session caught on the first sweep run (the burn-in conflation).** Three cracks in a single phase, all from paper-vs-code translation gaps, all caught by running code.
