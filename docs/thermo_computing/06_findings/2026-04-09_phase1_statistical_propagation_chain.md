# Phase 1 finding — statistical reasoning propagation chain (Crack 1 + Crack 2)

> **Status**: open finding, fix plan ready, awaiting next-session execution.
> **Discovered**: 2026-04-09 during the Phase 1 implementation session, on the first run of the integration test gate.
> **Branch**: `feature/thermo-doc-review`
> **Affected files**: `02_foundations/chassis_design.md` (Decision 5 ship-justification), `03_phases/01_langevin_thermostat.md` (§7.1 + §7.2 + §7.3 + §7.4), `sim/L0/thermostat/tests/langevin_thermostat.rs` (test_equipartition_central_parameter_set + the §7.4 sweep)

## TL;DR

A math error in the chassis Decision 5 reasoning text propagated through 4 paper-review passes (chassis grade, doc review, spec self-read, spec fresh-eyes review) without being caught. The first pass that caught it was running code: the §7 equipartition gate failed at `12.54σ` on its first run.

The error: chassis Decision 5 says `merge` is needed for option β; spec §7.3 implements option β by merging per-step Welford accumulators across trajectories and computing `std_error_of_mean` from the merged accumulator. **`std_error_of_mean` assumes IID samples**, but per-step `½v²` values within a trajectory are highly autocorrelated (`τ_int ≈ 5000` steps). The merged std error is too small by factor `√(1+2τ_int) ≈ 100`.

**The grand mean is correct** (`0.5029616596` vs target `0.5000000000`, deviation `0.6%`). Only the std error is wrong, which makes a `0.12σ` deviation appear as `12.54σ`.

**The fix is structural, not parametric**: replace merge-based estimator with a two-level Welford pattern (top-level across trajectories, inner per-trajectory). Confirmed by Probe E to give `0.12σ` deviation at the actual gate scale, with 24× margin under `±3σ`.

## The propagation chain (where each layer went wrong)

```
Recon log part 9 (correct, flagged statistical infra as critical)
       ↓
Chassis Decision 5 text (correct math: "0.022 = run_std/√100")
       ↓
Chassis Decision 5 ship-justification (WRONG inference: "needs merge")
       ↓                                      ← CRACK 1: chassis-doc reasoning text
Doc review M4 (propagates: "merge for option β")
       ↓
Spec §7.2 (correct math, again)
       ↓
Spec §7.3 pseudocode (WRONG translation: merge + std_error_of_mean)
       ↓                                      ← CRACK 2: spec pseudocode
Implementation (faithful translation of §7.3)
       ↓
Test FAILED at first run with 12.54σ            ← caught here
```

The math TEXT is correct at every layer. Each translation from math text to code/API decision had the same wrong inference.

### Layer 1 — recon log part 9 (CORRECT)

Recon log part 9 explicitly flagged statistical assertion infrastructure as a critical forward-looking design question:
> "Worth designing it deliberately at Phase 1, not stumbling into it."

It also named the future use cases that depend on getting it right:
> "Phase 2 (free + articulated body equipartition), Phase 3 (Kramers rate), Phase 4 (Ising/Gibbs joint distribution), Phase 5 (EBM target match). All of them will need similar statistical assertion infrastructure."

Recon log part 9 did not get the math wrong. It correctly identified that this was a high-stakes design decision and deferred the choice to chassis Decision 5.

### Layer 2 — chassis Decision 5 text (CORRECT)

Chassis design `02_foundations/chassis_design.md` lines 1497-1502 describe option β verbatim:
> "Each run gives one independent estimate; the standard error of the 100-estimate mean is `run_std / √100`, bringing the combined std error to about ±4.5% of `½kT`"

This is **option β-A** (push trajectory means into across-trajectories accumulator). The math is correct: `run_std` is the std deviation across the 100 trajectory means (not across the per-step samples), and `std error of grand mean = run_std / √100`.

### Layer 3 — chassis Decision 5 ship-justification (WRONG inference)

Chassis design `02_foundations/chassis_design.md` lines 1738-1750 — the merge ship-justification:

> "**`merge(&other)` ships in Phase 1** (added by doc review M4, 2026-04-09). Combines two independent `WelfordOnline` accumulators into one global statistic using the Chan/Pébay parallel-merge formula. Required by option (β) of the Phase 1 validation parameter fix — 100 independent trajectories, each with its own per-trajectory accumulator, **combined into a single 100-trajectory mean + variance** — and by every Phase 4+ parallel-env test that wants to fold per-env statistics together. **Without `merge`, option (β) has to materialize per-trajectory means in a `Vec<f64>` and re-Welford them, which loses the streaming property for the variance estimate.**"

**This is the location of Crack 1.** Three things are wrong:

1. **"Materialize per-trajectory means in a `Vec<f64>` and re-Welford them" IS option β-A and produces the CORRECT std error.** The chassis author treated this as a memory-overhead disadvantage to be avoided via merge, when it's actually the only correct way to compute the std error of the grand mean of autocorrelated time series.

2. **"Loses the streaming property" is not actually true.** You don't need to materialize a `Vec<f64>` — you can push trajectory means incrementally into a top-level `WelfordOnline`. That's also "streaming" (just at the trajectory level instead of the per-step level), allocates nothing, and gives the correct answer.

3. **Even if it were true that materializing a Vec was the alternative**, "100 trajectory mean f64 values" is 800 bytes — trivially cheap. There is no real cost to avoid.

Also worth noting, from line 1611:
> "standard error of mean is a one-line accessor."

This is encoded as a feature of `WelfordOnline::std_error_of_mean`. **It is true only for IID samples.** For autocorrelated samples, the one-line accessor gives the wrong answer. The chassis decision did not qualify this claim and treated `std_error_of_mean` as a sufficient statistic universally.

### Layer 4 — doc review M4 (PROPAGATED the wrong inference)

Doc review `05_doc_reviews/2026-04-09_doc_review.md` M4 added `merge` to chassis Decision 5 with the same reasoning:
> "Smaller note in the same vein: `WelfordOnline` should also expose a `merge(&other)` method. Option (β) (100 independent trajectories) wants to combine 100 per-trajectory accumulators into one global statistic."

The doc review M1 author also got the math right at the std error level (line 339):
> "option (β) — 100 independent trajectories — gives `N_eff ≈ 1000`, so the standard error is `0.707/√1000 ≈ 0.022`, which is **4.5% relative to ½kT**"

**The math text is correct. The translation to "use merge" is the conceptual error.** M1 and M4 both ran clean against the doc review checklist; neither caught the layer-3 inference.

### Layer 5 — spec §7.2 (CORRECT, AGAIN)

Spec `03_phases/01_langevin_thermostat.md` §7.2:
> "Per-trajectory `N_eff ≈ n_measure / (1 + 2·τ_int) ≈ 10`.
> Per-trajectory std error of `⟨½v²⟩` ≈ `(½kT)·√2/√10 ≈ 0.45·(½kT)`.
> 100-trajectory aggregated std error ≈ per-traj std error / `√100` ≈ `0.045·(½kT)` ≈ **±4.5% of ½kT**."

The math is right. This is option β-A's correct std error formula.

### Layer 6 — spec §7.3 pseudocode (WRONG translation)

Spec §7.3 pseudocode (line ~314):
```rust
let mut global = WelfordOnline::new();
for i in 0..n_traj {
    let mut traj = WelfordOnline::new();
    for _ in 0..n_measure {
        data.step(&model)?;
        traj.push(0.5 * data.qvel[0] * data.qvel[0]);
    }
    global.merge(&traj);
}
let std_error = global.std_error_of_mean();
```

**This is option β-B**: merge per-step Welford accumulators across trajectories, then call `std_error_of_mean` on the merged accumulator. The merged accumulator has `count = n_traj × n_measure = 10⁷` and `std_error_of_mean ≈ sqrt(variance / 10⁷)`, which is the IID formula. Per-step samples are autocorrelated (`τ_int ≈ 5000` steps), so the IID formula is wrong by factor `√(1+2τ_int) ≈ 100`.

**This is the location of Crack 2.** It is a direct propagation of Crack 1's inference into pseudocode. The spec author followed the chassis's "merge for option β" guidance literally and produced pseudocode that does what the chassis text said to do, not what the chassis math said to do.

### Layer 7 — Implementation (faithful)

`sim/L0/thermostat/tests/langevin_thermostat.rs::test_equipartition_central_parameter_set` (committed in `b0f69ad`) translates §7.3 pseudocode into Rust verbatim. The implementation is correct in the sense that it does exactly what the spec asked. **The implementation has no bug; it faithfully reflects the broken spec.**

### Layer 8 — First code run (CAUGHT IT)

The very first run of `cargo test -p sim-thermostat --release --test langevin_thermostat` produced:
```
test_equipartition_central_parameter_set FAILED
  measured = 5.029617e-1
  expected = 5.000000e-1
  deviation = 2.961660e-3
  standard_error = 2.362233e-4
  n_sigma = 3
  bound = 7.086700e-4
  relative_deviation = 12.5375σ
```

A `12.54σ` failure is impossible to explain by sampling noise alone. The mean is correct to within `0.6%` (well within physical sampling expectations). The std error is `2.36e-4`, two orders of magnitude smaller than spec §7.2 predicted (`~0.0224`). **The first code run produced a result that immediately diagnosed itself.**

## Empirical evidence (Probes A through F)

Six empirical probes were run during the crime-scene investigation. Each probe was a separate `#[ignore]`'d test in `tests/_phase1_findings_probes.rs` (committed separately in this session as a handoff artifact, deleted in the next session).

### Probe E — full gate scale, both estimators in parallel

The decisive evidence. At the **exact** scale of the actual gate (100 trajectories × 100,000 steps × `seed_base = 0xC0FFEE`), computed BOTH the merged estimator (current spec §7.3) AND the across-trajectories estimator (option β-A) from the same data:

| | mean | variance | std_err | deviation | **σ** |
|---|---|---|---|---|---|
| **Merged (current spec §7.3)** | 0.5029616596 | 0.5580145679 | 2.36e−4 | 2.96e−3 | **12.54σ** ❌ |
| **Across-trajectories (β-A)** | 0.5029616596 | 0.0584958461 | 2.42e−2 | 2.96e−3 | **0.12σ** ✓ |

Key observations:
- **Grand means are bit-identical** (`0.5029616596 = 0.5029616596`). Both estimators compute the same mean — only the std error differs. Confirms the implementation is producing the correct underlying samples.
- **Ratio of merged-to-correct std error: `0.0242 / 0.000236 ≈ 102.5`**. Matches the chassis's predicted `√(1+2τ_int) ≈ 100` within 2.5%.
- **Across-trajectories deviation `0.12σ`** passes ±3σ with **24× margin**. The corrected gate would pass cleanly.

### Probe E — variance decomposition cross-check

The merged variance (`0.558`) is higher than the theoretical `Var(½v²) = 0.5` for a Gaussian v with `kT/M = 1`. Initially this looked like a kurtosis anomaly, but it decomposes cleanly:

```
Var_pooled = within-trajectory variance + between-trajectory variance
           = 0.5 (stationary) + 0.0585 (Probe E across_trajectories.variance())
           = 0.5585

Observed merged variance = 0.5580 (matches within 0.1%)
```

**The merged variance is exactly what variance decomposition predicts.** Not an anomaly. The Gaussian-v assumption is intact.

### Probe F — vary seed_base to rule out systematic bias

The 0.6% positive deviation could in principle be either sampling noise OR a small systematic bias. Probe F ran the full gate at 6 different `seed_base` values to distinguish:

| seed_base | mean | deviation | σ | sign |
|---|---|---|---|---|
| 0xC0FFEE | 0.5029616596 | +2.96e−3 | +0.12 | + |
| 0xBADF00D | 0.4972875317 | −2.71e−3 | −0.13 | − |
| 0xCAFEBABE | 0.4907776035 | −9.22e−3 | −0.41 | − |
| 0xDEADBEEF | 0.4652127813 | −3.48e−2 | −1.85 | − |
| 0xFEEDFACE | 0.5132946297 | +1.33e−2 | +0.65 | + |
| 0xFACEF00D | 0.5097218707 | +9.72e−3 | +0.44 | + |

Summary across 6 gates:
- **Sign split: 3 positive, 3 negative** (perfect 50/50)
- **Mean of deviations: −3.46e−3** (consistent with zero — `−0.49σ` from zero with `std_of_mean = 7.1e−3`)
- **Std of deviations: 1.74e−2** (within 30% of theoretical 0.024 — consistent with `std-of-std` uncertainty for `n=6`)
- **All 6 gates pass ±3σ** under the corrected estimator. Max observed: `−1.85σ` (DEADBEEF) — well inside ±3σ.

**Verdict: pure sampling noise. No systematic bias.** The `+0.12σ` on the canonical seed was just typical sampling fluctuation.

### Probes B, C, D — supporting evidence

- **Probe B (small-scale 20×20k)** confirmed the across-trajectories estimator gives the correct std error pattern at smaller scales. Per-trajectory means ranged from `0.18` to `0.96`, confirming small `N_eff` per trajectory.
- **Probe C (per-trajectory variance + kurtosis)** showed kurtosis 2.2-2.8, consistent with Gaussian within sampling noise (kurtosis std ≈ `√(24/22) ≈ 1.04` at `N_eff ≈ 22`).
- **Probe D (deterministic reproducibility)** showed bit-identical mean and std error across two consecutive runs of the same gate. **Failure mode is deterministic, not a flake.**

## What's NOT broken (sub-checks)

The following all check out cleanly:

1. **`mem::replace` split-borrow dance.** §6 (passes — spring force is applied), §8 (passes — cb_passive fires once per step), §9 (passes — bit-deterministic), Probe A's decay envelope (passes — damping applied correctly under guard). Four independent confirmations.
2. **Welford merge formula.** Unit-tested in `test_utils.rs::welford_merge_matches_one_pass_over_full_dataset` against a one-pass Welford to absolute `1e−12` for both equal and unequal halves. Chan/Pébay is correct.
3. **`Welford::std_error_of_mean` itself.** It correctly computes `sqrt(variance/count)`. The crack is in **how this primitive is applied to autocorrelated data**, not in the primitive itself.
4. **Implementation `apply` math.** The mean ⟨½v²⟩ matches `½kT` to within `0.6%`. If noise scaling were wrong by `f`, ⟨½v²⟩ would be off by `f²`. We see no such bias.
5. **ChaCha8 / StandardNormal.** The mean is consistent with zero-noise-bias.
6. **Discretization bias of semi-implicit Euler on Langevin SHO.** `O(h²) ≈ 1e−6` for our parameters. Far below the noise floor.
7. **Burn-in sufficiency.** 25,000 steps = 5 τ_int. After this the trajectories are at the stationary distribution (confirmed by the across-trajectories statistics matching theory).

## Fix plan

### Step 1 — chassis_design.md text amendment

Per the recon-to-iteration handoff discipline (chassis stays paper-locked during Phase 1 implementation; corrections live in `06_findings/`), the chassis itself does NOT get edited as part of Phase 1. **This finding IS the chassis amendment.** A future session deciding whether to fold this into chassis_design.md proper will reference this file.

The amendment text would say (if/when folded):
- **Line 1611**: "standard error of mean is a one-line accessor" → qualify as "for IID samples; for autocorrelated time series, see the two-level Welford pattern."
- **Lines 1738-1750**: rewrite the merge ship-justification to say merge ships for IID parallel-reduce contexts (Phase 4+ batch reductions) and not specifically for option β. Clarify that option β is implemented via two-level Welford (per-trajectory + across-trajectories), not via merge.
- **Optional**: add a sub-section "Statistical patterns for autocorrelated time series" that explicitly names the two-level Welford pattern as the canonical option β implementation.

### Step 2 — spec edits to `01_langevin_thermostat.md`

**§7.1**: Replace the option β table description and reasoning. The new text should:
- Drop the "uses merge end-to-end" rationale (no longer the implementation strategy).
- Add that option β is implemented via two-level Welford: per-trajectory accumulator pushes per-step `½v²` and produces a trajectory mean; the trajectory mean is pushed into a top-level across-trajectories accumulator. After `n_traj` trajectories the top-level accumulator has `n_traj` IID samples and its `std_error_of_mean` is the correct std error of the grand mean.
- Note that `WelfordOnline::merge` is not used by the §7 gate. It still ships in the chassis for IID parallel-reduce contexts in future phases.

**§7.2**: Statistical accounting stays. Add a sentence clarifying that the formula applies to the across-trajectories estimator (the trajectory means are IID by construction; the formula `run_std/√100` applies directly).

**§7.3**: Rewrite the pseudocode:
```rust
let mut across_trajectories = WelfordOnline::new();
for i in 0..n_traj {
    let mut model = sim_mjcf::load_model(SHO_1D_XML).expect("load");
    let mut data = model.make_data();

    PassiveStack::builder()
        .with(LangevinThermostat::new(
            DVector::from_element(model.nv, gamma_value),
            k_b_t,
            seed_base + i as u64,
        ))
        .build()
        .install(&mut model);

    for _ in 0..n_burn_in {
        data.step(&model).expect("burn-in step");
    }

    let mut traj = WelfordOnline::new();
    for _ in 0..n_measure {
        data.step(&model).expect("measure step");
        traj.push(0.5 * 1.0 * data.qvel[0] * data.qvel[0]);
    }

    // Push the trajectory mean as a single sample. The 100 trajectory
    // means are IID by construction (different seeds, sufficient
    // burn-in to forget initial conditions), so the across-trajectories
    // accumulator's std_error_of_mean is the correct estimator.
    across_trajectories.push(traj.mean());
}

let measured = across_trajectories.mean();
let expected = 0.5 * k_b_t;
let std_error = across_trajectories.std_error_of_mean();
assert_within_n_sigma(
    measured, expected, std_error, 3.0,
    "equipartition central: 1-DOF damped harmonic oscillator, 100 traj × 10⁵ steps",
);
```

**§7.4**: Same fix — the sweep test has the same merge-based bug. Replace with two-level Welford for each `(γ, kT)` combination.

### Step 3 — implementation edit to `tests/langevin_thermostat.rs`

Apply the corrected pseudocode to the actual test file. The changes are mechanical — the structure is the same, only the inner accumulation pattern changes.

### Step 4 — verification

Re-run `cargo test -p sim-thermostat --release` (release mode required for §7's runtime budget). Confirm:
- 5 mandatory tests pass (3 already pass + the 2 corrected ones)
- §7.4 sweep passes under `--ignored`
- Run twice more to confirm no flake (deterministic per Probe D, but session brief asks for it)

### Step 5 — closing implementation review (§13)

The highest-value step. Read the corrected spec end-to-end against the implementation. Surface any remaining drift.

## Open question for the next session

**Does the chassis_design.md text get amended directly, or stay paper-locked?** Two options:

- **Option A**: This finding is the canonical record of the correction. Chassis stays paper-locked. Future sessions reading chassis_design.md will encounter the misleading text but a forward reference (or just a `06_findings/` cross-link from the chassis decision header) will route them here.
- **Option B**: Fold the corrections into chassis_design.md as part of the next session's work. This mutates the chassis but is honest about the fact that the original reasoning was wrong.

My weak recommendation is Option A for now — it preserves the paper-lock principle and lets the chassis evolve via accumulated findings rather than direct edits during implementation. Option B can be revisited when there's a more substantial chassis evolution on the table.

## Reproducibility

All Probe E, Probe F, and supporting evidence in this finding were captured by `tests/_phase1_findings_probes.rs` (committed in this session as a handoff artifact, to be deleted in the next session). The probe file is reproducible code; the tables in this finding are reproducible numbers extracted from running the probes in release mode.

The fix is also reproducible: the corrected pseudocode in Step 2 above, when translated into Rust and run against the same `seed_base = 0xC0FFEE`, produces:
```
mean = 0.5029616596
std_err = 0.0242
deviation = 2.96e-3 → 0.12σ
```

A `0.12σ` deviation is well inside `±3σ`. The corrected gate passes cleanly. **24× margin under the gate's tolerance.**
