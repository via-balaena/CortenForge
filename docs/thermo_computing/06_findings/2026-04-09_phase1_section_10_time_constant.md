# Phase 1 finding — §10 underdamped time constant physics error (Crack 3)

> **Status**: open finding, fix plan ready, awaiting next-session execution.
> **Discovered**: 2026-04-09 during the Phase 1 implementation session, on the first run of the integration test gate.
> **Branch**: `feature/thermo-doc-review`
> **Affected files**: `03_phases/01_langevin_thermostat.md` (§10 line 424), `sim/L0/thermostat/tests/langevin_thermostat.rs` (test_stochastic_gating_sanity)

## TL;DR

Spec §10 line 424 says the damping time constant is `M/γ = 10` time units and instructs the test to "Run 5× that to land in numerical-floor territory" (50,000 steps). **The actual amplitude decay time constant of an underdamped harmonic oscillator is `2M/γ = 20` time units**, not `M/γ`. After 50,000 steps the system has decayed by only `exp(−2.5) ≈ 0.082` of its initial amplitude, not into "numerical floor territory."

The implementation faithfully runs 50,000 steps and observes `qpos = 0.0764, qvel = 0.0264, amplitude ≈ 0.081` — a quantitative match to the correct `exp(−γt/(2M))` envelope to within ~1%. The spec's threshold assertion (`<1e-3`) is unsatisfiable at 50,000 steps but trivially satisfied at 200,000 steps (envelope `≈ 4.5e-5`, 22× margin).

This is a standalone single-line spec error. No upstream paper trail, no chassis dependency, no propagation through multiple layers. Pure transcription error: spec author had `ζ = 0.05` correctly in front of them (recon log part 2) but wrote a different formula in §10.

## Physics derivation (independent, from first principles)

The equation of motion for a 1-DOF damped harmonic oscillator:
```
M·ẍ + γ·ẋ + k·x = 0
```

Divide by M, define `ω₀² = k/M` and `2β = γ/M` (so `β = γ/(2M)`):
```
ẍ + 2β·ẋ + ω₀²·x = 0
```

Damping ratio: `ζ = β/ω₀ = γ/(2√(kM))`. For our parameters `M=1, k=1, γ=0.1`: `ω₀ = 1`, `β = 0.05`, `ζ = 0.05`. **Underdamped** (`ζ < 1`). Recon log part 2 records this correctly.

The general solution for the underdamped case is:
```
x(t) = A · exp(−β·t) · cos(ω_d·t + φ),  where ω_d = √(ω₀² − β²)
```

The amplitude envelope is `exp(−β·t) = exp(−γt/(2M))`. **Time constant of envelope decay = `1/β = 2M/γ`.**

For our parameters: `time constant = 2·1/0.1 = 20 time units = 20,000 steps at h=0.001`.

After `t = 50` time units (50,000 steps): envelope `= exp(−0.05·50) = exp(−2.5) ≈ 0.0821`.
After `t = 200` time units (200,000 steps): envelope `= exp(−0.05·200) = exp(−10) ≈ 4.54e−5`.

## Where the spec went wrong

Spec `03_phases/01_langevin_thermostat.md` §10 line 424 (inside the test pseudocode comment):
> "// Damping time constant M/γ = 10 time units = 10_000 steps.
> // Run 5× that to land in numerical-floor territory."

`M/γ = 1/0.1 = 10` time units. 5× is 50 time units = 50,000 steps. The test then runs 50,000 steps and asserts:
```rust
data.qvel[0].abs() < 1e-3 && data.qpos[0].abs() < 1e-3
```

**`M/γ` is the correct time constant for a different physical situation:** velocity decay of a free particle in viscous fluid (`dv/dt = −(γ/M)v` → `v(t) = v₀·exp(−γt/M)`, time constant `M/γ`). It is also the correct time constant for **energy decay** of an underdamped oscillator (energy ∝ amplitude², so energy decays at twice the amplitude rate, i.e., at rate `γ/M`, time constant `M/γ`).

But §10 is checking `qpos` and `qvel` against `<1e-3`, both of which scale with **amplitude**, not energy. The relevant time constant is amplitude decay `2M/γ`, not energy decay `M/γ`.

The spec author was likely thinking "energy time constant" or "free-particle time constant" — both are real time constants in physics, both have the value `M/γ`, and both might have been the formula on hand when writing §10. Neither is correct for what §10 actually checks.

The recon log part 2 entry (which the spec author referenced via the `ζ = 0.05` damping ratio) does NOT make this error. The damping ratio correctly uses `γ/(2√(kM))`, which has the factor of 2 baked in. The spec author had the correct physics in front of them but wrote a different formula in §10.

## Empirical evidence (Probe A)

Probe A ran the §10 setup under `disable_stochastic` for 200,000 steps and recorded the system state at 6 checkpoints. For each checkpoint, computed the observed `amplitude = √(qpos² + qvel²)` and compared to the theoretical envelope `exp(−0.05·t)`:

| step | time (s) | qpos | qvel | observed amplitude | exp(−0.05·t) | ratio |
|---|---|---|---|---|---|---|
| 10,000 | 10.00 | −5.29e−1 | +3.24e−1 | 6.20e−1 | 6.07e−1 | 1.022 |
| 20,000 | 20.00 | +1.75e−1 | −3.32e−1 | 3.76e−1 | 3.68e−1 | 1.022 |
| 50,000 | 50.00 | +7.64e−2 | +2.64e−2 | 8.09e−2 | 8.21e−2 | 0.985 |
| 100,000 | 100.00 | +5.15e−3 | +4.10e−3 | 6.58e−3 | 6.74e−3 | 0.976 |
| 150,000 | 150.00 | +2.85e−4 | +4.60e−4 | 5.41e−4 | 5.53e−4 | 0.978 |
| 200,000 | 200.00 | +9.66e−6 | +4.39e−5 | 4.49e−5 | 4.54e−5 | 0.989 |

**Six independent checkpoints. Ratio in `[0.976, 1.022]` — within ~2.5% at every checkpoint.**

The amplitude decay rate is `γ/(2M) = 0.05`, not `γ/M = 0.1`. The spec's "M/γ = 10" formula is wrong by a factor of 2. **Diagnosis confirmed empirically.**

The 50,000-step row shows `amplitude ≈ 0.081` — exactly the value the failing test reported (`qpos=0.0764, qvel=0.0264, |amplitude|≈0.081`). The implementation is correct; the spec's expected threshold of `<1e-3` is unsatisfiable at 50,000 steps.

## Cross-check: integrator eigenvalue analysis

To rule out the possibility that semi-implicit Euler has a different damping rate than the continuous-time analysis predicts, I derived the per-step amplitude factor directly from the integrator's update rule.

`Integrator::Euler` in sim-core is **semi-implicit Euler** (per the doc at `sim/L0/core/src/integrate/mod.rs:34`):
> "Semi-implicit Euler. Updates velocity first (`qvel += qacc * h`), then integrates position using the new velocity."

For the deterministic Langevin SHO (noise off, just damping + spring) with M=k=1, γ=0.1, h=0.001:
```
qvel_new = qvel + h·qacc = qvel + h·(−k·x − γ·v) = (1 − hγ)·qvel − hk·qpos
qpos_new = qpos + h·qvel_new
```

The linear map is:
```
[qpos_new]   [1 − h²k    h(1 − hγ)] [qpos]
[qvel_new] = [  −hk         1 − hγ ] [qvel]
```

For our parameters, the matrix entries are:
```
[1 − 1e−6     9.999e−4]
[−1e−3         0.9999  ]
```

Trace = 1.999899, det ≈ 0.999900, discriminant = trace² − 4·det ≈ −3.97e−6, sqrt(disc) ≈ 0.001992·i.

Eigenvalues = (trace ± sqrt(disc))/2 ≈ 0.99995 ± 0.000996·i.

Magnitude `|λ| = √(0.99995² + 0.000996²) ≈ √(0.99990) ≈ 0.99995`.

So per step, amplitude is multiplied by `≈ 0.99995 = 1 − 5e−5 ≈ exp(−5e−5)`. After `n` steps, amplitude `∝ exp(−5e−5·n)`.

After 50,000 steps: `exp(−2.5) ≈ 0.0821` ✓ (matches Probe A row 3 to ~1%)
After 200,000 steps: `exp(−10) ≈ 4.54e−5` ✓ (matches Probe A row 6 to ~1%)

**The integrator decays the amplitude at exactly the rate the continuous-time theory predicts: `γh/(2M) = 5e−5` per step, or `γ/(2M) = 0.05` in time units.** Semi-implicit Euler does not introduce any artifact — the spec's formula is wrong, not the integrator.

## Fix plan

### Step 1 — spec edit to `01_langevin_thermostat.md` §10

**Single-line edit at line 424.** Change the comment in the test pseudocode from:
```rust
// Damping time constant M/γ = 10 time units = 10_000 steps.
// Run 5× that to land in numerical-floor territory.
for _ in 0..50_000 { data.step(&model).expect("step"); }
```

to:
```rust
// Amplitude time constant for the underdamped oscillator is
// 2M/γ = 20 time units = 20_000 steps. Run 10× that to land in
// numerical-floor territory (envelope ≈ exp(-10) ≈ 4.5e-5, ~22×
// margin under the 1e-3 threshold). Note: M/γ would be the
// correct formula for free-particle velocity decay or for
// underdamped-oscillator ENERGY decay (which is 2× faster than
// amplitude decay), but neither matches what we're checking here.
for _ in 0..200_000 { data.step(&model).expect("step"); }
```

Plus update the surrounding prose in §10 to match (the introduction to the test pseudocode mentions the same time constant — needs the same fix).

### Step 2 — implementation edit to `tests/langevin_thermostat.rs::test_stochastic_gating_sanity`

Mirror the spec change. Replace the `for _ in 0..50_000` loop with `for _ in 0..200_000`. Update the inline comment to reference the corrected time constant.

### Step 3 — verification

Re-run `cargo test -p sim-thermostat --release --test langevin_thermostat test_stochastic_gating_sanity`. Expected: pass cleanly with `qpos ≈ 9.7e−6`, `qvel ≈ 4.4e−5`, both well under `1e-3`. (Per Probe A row 6.)

## Open question for the next session

**Should the §10 step count be 200,000 (10× time constant, ~22× margin) or something else?**

Options:
- **200,000 steps (10·τ)**: envelope `≈ 4.5e-5`, 22× margin, ~3 seconds runtime in release mode. Recommended.
- **150,000 steps (7.5·τ)**: envelope `≈ 5.5e-4`, 1.8× margin, ~2 seconds runtime. Tighter margin but acceptable.
- **100,000 steps (5·τ)**: envelope `≈ 6.7e-3`, **fails the 1e-3 threshold**. Not viable.
- **Loosen the threshold instead**: e.g., assert `< 0.01`. Bad — silent tuning, doesn't actually verify the system reached numerical-floor territory.

**Recommendation: 200,000 steps with 10× time constant.** It respects the "pass with margin, not at threshold" sharpen-the-axe principle. Runtime cost is negligible. Probe A confirmed the row at this exact step count gives `amplitude ≈ 4.49e-5`.

## Reproducibility

Probe A is reproducible via the (about-to-be-deleted) probe file `tests/_phase1_findings_probes.rs::probe_a_decay_envelope_at_multiple_checkpoints`. The numbers in the table above are extracted from running it in release mode. After the spec and impl fixes land, Probe A is no longer needed because the corrected `test_stochastic_gating_sanity` exercises the same physics directly.

The fix is also reproducible: with 200,000 steps and the same parameters (`γ=0.1, M=1, k=1, h=0.001`), the system decays to `amplitude < 5e-5` deterministically (Probe A confirmed). The corrected gate passes with ~22× margin under `1e-3`.
