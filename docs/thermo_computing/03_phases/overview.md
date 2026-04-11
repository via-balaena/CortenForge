# Phases — The Gap (working both ways toward the middle)

> Extracted from `MASTER_PLAN.md` §4 *The Gap* during the 2026-04-09
> doc-tree refactor. Per-phase spec files (`01_langevin_thermostat.md`,
> etc.) will be added next to this overview as each phase matures from
> sketch to implementable.

Phases ordered by dependency. Each phase has a validation gate; subsequent
phases do not start until the previous gate is green. This implements the
"baby steps for new physics" + "fix gaps before continuing" rules.

| #   | Phase                              | Status   | Validation gate                                                       | Spec |
| --- | ---------------------------------- | -------- | --------------------------------------------------------------------- | ---- |
| 1   | Langevin thermostat (1-DOF)        | spec'd   | Equipartition `⟨½mv²⟩ = ½k_B·T` on damped harmonic oscillator         | [`01_langevin_thermostat.md`](./01_langevin_thermostat.md) |
| 2   | Thermostat on free + articulated   | **done**  | Equipartition holds on 6-DOF free body; on a 2-link hinge chain      | [`02_multi_dof_equipartition.md`](./02_multi_dof_equipartition.md) |
| 3   | Single bistable element            | **done**  | Double-well switching rate matches Kramers' formula                   | [`03_bistable_kramers.md`](./03_bistable_kramers.md) |
| 4   | Coupled bistable array             | **done**  | Pairwise correlations match exact Ising model on same coupling topology | [`04_coupled_bistable_array.md`](./04_coupled_bistable_array.md) |
| 5   | Boltzmann learning on physical Ising sampler | **done** | KL convergence + parameter recovery on fully-connected 4-element graph | [`05_boltzmann_learning.md`](./05_boltzmann_learning.md) |
| 6   | Native Gibbs sampler + three-way comparison | **done** | TV(Gibbs,exact)<0.02; TV(Langevin,exact)<0.15; Gibbs≪Langevin; Phase 4 §11.4 closed | [`06_gibbs_sampler.md`](./06_gibbs_sampler.md) |
| 7   | RL on the sampler                  | pending  | Agent improves sample quality (ESS / autocorrelation / KL to target)  | —    |
| D1  | Brownian ratchet motor (RL harvests thermal noise) | **done** | CEM displacement −40 periods (140× random); REINFORCE noise-inflation finding | [`d1_brownian_ratchet.md`](./d1_brownian_ratchet.md) |

## Phase 1 — Langevin thermostat (1-DOF)

> **Spec'd 2026-04-09**: the inline sketch that previously lived here has
> been replaced by a full per-phase spec at
> [`01_langevin_thermostat.md`](./01_langevin_thermostat.md). That file
> is the canonical Phase 1 source: API surface, MJCF model, validation
> tests (equipartition gate + callback firing-count + reproducibility +
> Stochastic gating sanity), acceptance criteria, and forwarded Phase 5+
> caveats. Read it before implementing or reviewing Phase 1 work.

The historical sketch and the older `qfrc_applied`-based revision notes
are preserved verbatim in the recon log: see parts
[02](../04_recon_log/2026-04-09_part_02_forward_step.md),
[03](../04_recon_log/2026-04-09_part_03_qfrc_applied.md), and
[04](../04_recon_log/2026-04-09_part_04_cb_passive.md) for the design
trail.

<!-- prior inline sketch removed; see 01_langevin_thermostat.md -->

<details>
<summary>Original inline sketch (collapsed; preserved for historical reference)</summary>

The minimum viable first move. Implement a `LangevinThermostat` struct
that wraps in a `PassiveStack` (chassis Decision 2) and installs as a
`cb_passive` callback on the model. Each step, the thermostat contributes
both an explicit damping force *and* a stochastic force to its per-DOF
scratch buffer (chassis M5 contract); the stack folds the buffer into
`qfrc_passive` once per step:

```
qfrc_out[i] += −γ_i · qvel[i]  +  sqrt(2 · γ_i · k_B · T / h) · randn()
                └── damping ──┘   └────── FDT-paired noise ──────┘
```

After every component in the stack has run, the stack does
`data.qfrc_passive += qfrc_out` once. The thermostat itself never sees
or touches `data.qfrc_passive` directly.

where `γ_i` is the thermostat damping coefficient on DOF `i`, owned by the
thermostat instance (not by the model — see Q4).

**Scheme**: Explicit Langevin (Euler-Maruyama). Damping and noise are
both *custom passive forces*, sharing the existing `qfrc_passive`
aggregation and projection pipeline. No BAOAB splitting, no integrator
bypass, no model mutation, no split-step API needed. The discretization
temperature error scales as `O(h · γ / M)` and is below the validation
test's sampling-error tolerance at the chosen parameters (see Recon Log,
2026-04-09 part 2). Upgrade path to BAOAB / Eulerdamp is clear if higher
accuracy is later needed.

**Why `cb_passive` and not `qfrc_applied`**: Recon item 3 first finding
(2026-04-09 part 4). `cb_passive` is the documented hook for "custom
passive forces (e.g., viscous drag, spring models)" — exactly the
category Langevin damping + FDT-paired noise belongs to. `qfrc_passive`
is auto-cleared every step at the start of `mj_fwd_passive()`
(`passive.rs:368`), so accumulation is safe by construction — no silent
compounding possible. The thermostat is conceptually a passive thermal
force, so this is also the correct ontological category. Item 2's
original Option A (sole-writer overwrite of `qfrc_applied`) is superseded
and the forward-looking Option C (dedicated `qfrc_thermostat` field) is
no longer needed — `cb_passive` already achieves composability with RL /
controller writes to `qfrc_applied`.

**Where it lives**: A small `thermostat` submodule of sim-core (likely
alongside `forward/passive.rs`, since the integration point is in
`mj_fwd_passive`). A `LangevinThermostat` struct + an
`install(self: Arc<Self>, model: &mut Model)` method that wires up the
callback. Opt-in, default off (the model's `cb_passive` field defaults
to `None`).

**RNG ownership**: `cb_passive` is `Fn`, not `FnMut`, so the closure
cannot capture mutable RNG state directly. The `LangevinThermostat`
struct owns its RNG behind interior mutability (`Mutex<PRNG>`); the
callback closure captures `Arc<LangevinThermostat>` and dereferences it
each call. This gives thread-safety (matches the `Send + Sync` bound on
the callback), reproducibility (RNG is per-instance, seedable), and
local state (no `Data` mutation needed). PRNG choice is recon item 4
(still open) — placeholder.

**API sketch**:
```rust
let thermostat = Arc::new(LangevinThermostat::new(
    /* gamma  */ DVector::from_element(model.nv, 0.1),
    /* k_b_t  */ 1.0,
    /* seed   */ 42,
));
thermostat.install(&mut model);
loop { data.step(&model)?; }   // plain step(), no split-step needed
```

**Model setup**: Set `dof_damping = 0` on the thermostatted DOFs (the
thermostat owns damping; model damping would compound). Use
`Integrator::Euler`. The thermostat is integrator-agnostic in principle —
ImplicitFast / Implicit will work via the same `qfrc_passive` path —
but Phase 1 only validates against Euler.

**Validation** (sketch — *the Phase 1 spec will revise the parameter set*):
1-DOF damped harmonic oscillator with `M=1`, `k_spring=1`, `γ=0.1`,
`k_B·T=1`, `h=0.001`. Measure `⟨½ M qvel²⟩` and assert it equals
`½ k_B·T` within sampling-error tolerance. Then sweep
`γ ∈ {0.01, 0.1, 1.0}` and `k_B·T ∈ {0.5, 1.0, 2.0}` to confirm
linear scaling in `T` and γ-independence of the stationary
temperature. Passing this is the gate to all of Phase 2+.

> **Correction (chassis design Decision 5, 2026-04-09; tolerance
> wording corrected 2026-04-09 doc review M1)**: the original
> "10⁵ steps gives ~10⁻² sampling tolerance" calculation from
> recon log part 2 was *too optimistic by ~20×*. It assumed
> independent samples; actual samples from the Markov chain are
> correlated. For γ=0.1, M=1, h=0.001, the velocity-squared
> autocorrelation time is `τ_int ≈ M/(2γh) = 5000 steps`, giving
> an *effective* sample count of `N_eff ≈ 10⁵ / (1 + 2·5000) ≈ 10`
> for a 10⁵-step run. With `½Mv²` chi-squared distributed (mean
> `½kT`, standard deviation `(½kT)·√2`), the standard error of
> the sample mean over `N_eff = 10` effective samples is
> `(½kT)·√2/√10 ≈ 0.45 · (½kT)` — about **±45% relative to the
> expected mean ½kT**, not ±2%. **Tolerance convention** (locked
> in by this correction): in this document and all downstream
> validation specs, sampling-error tolerances are always expressed
> as a fraction of the expected mean (here `½kT`), not as a
> fraction of `kT`. The earlier "±22%" wording confused the two
> by a factor of 2. The Phase 1 spec will choose between three
> fixes: **(α)** ~100× longer trajectories, **(β)** ~100
> *independent* trajectories of 10⁵ steps each averaged across
> runs — avoids autocorrelation analysis entirely; weak lean;
> brings the std error to ~4.5% of `½kT`, **(γ)** loosen tolerance
> to ~5-10% of `½kT`. See [`../02_foundations/chassis_design.md`](../02_foundations/chassis_design.md)
> Decision 5 for the full reasoning.

**Phase 5+ caveats** (flagged here, addressed in later phases):
1. **Derivatives / FD perturbation** — `cb_passive` fires inside
   `forward_skip()` because `mj_fwd_passive` is called from `forward_acc`,
   which `forward_skip` calls. **RESOLVED at the chassis level by
   doc review M2 → Decision 7 (2026-04-09)**: `PassiveStack` exposes
   `disable_stochastic() -> StochasticGuard<'_>`, an RAII guard that
   sets every `Stochastic` component to deterministic-mode for the
   guard's lifetime and restores prior state on drop. Phase 5 FD
   loops wrap their perturbation block in the guard; the perturbed
   and baseline runs both produce zero noise, so the FD difference
   recovers `∂F_det/∂qpos` exactly. See
   [`../02_foundations/chassis_design.md`](../02_foundations/chassis_design.md)
   Decision 7 for the full reasoning and the Scheme A vs. Scheme B
   trade-off.
2. **Plugin passive forces fire after `cb_passive`** (`passive.rs:723-731`).
   If a model has plugins contributing passive forces, the thermostat's
   noise will be present when the plugin runs. Not a Phase 1 issue (no
   plugins) but worth being aware of.
3. **BatchSim parallel envs** need *one* `LangevinThermostat` instance
   per env, not a shared instance. Avoids lock contention on the RNG
   mutex and keeps RNG streams independent across envs. **Resolved at
   the chassis level by Decision 3** (`install_per_env` factory +
   defensive clear).

</details>

## Phases 2–7

Sketches only at this point. Each will be fleshed into a child spec when the
preceding gate is green.

- **Phase 2**: Test the thermostat on a free body (3 translational + 3
  rotational DOF) and on a 2-link articulated chain with a hinge. The
  articulated case is where constraint-projection effects on noise become
  real (see Open Question Q1).
- **Phase 3**: Build a single bistable element in cf-design — start with a
  buckling beam or a parameterized double-well potential well. Measure
  switching rates as a function of barrier height and `T`; compare against
  Kramers' escape-rate formula.
- **Phase 4**: Couple 4–8 bistable elements via elastic linkages. Measure the
  joint state distribution; compare against analytical Ising or against a CPU
  Gibbs sampler on the same energy.
- **Phase 5**: Make the geometry/coupling differentiable through cf-design and
  hook the autograd engine to it. Train the array as an EBM to match a target
  distribution (start with a 2D Gaussian mixture). **Gates on Q5** (cf-design
  end-to-end differentiability) — escalated to active foreground recon by doc
  review M3, scheduled in parallel with the Phase 1 spec drafting so the
  build order past Phase 1 is informed by the answer rather than blind to it.
- **Phase 6**: Bridge to THRML or implement native. **Q3 RESOLVED 2026-04-09
  (doc review S1)**: original `extropic-ai/thrml` (JAX) exists; two community
  Rust ports exist on GitHub (`SashimiSaketoro/thrml-rs`,
  `Pingasmaster/thrml-rs`), neither officially maintained by Extropic, neither
  apparently on crates.io. Three Phase 6 options now in scope: (A) depend on
  a community port, (B) implement a minimal native block-Gibbs sampler in
  Rust (leading direction, ~few hundred LOC, sharpen-the-axe consistent),
  (C) vendor / fork a port into a `sim-thrml` sibling crate. See Q3 in §5
  for the full trade-off.
- **Phase 7**: Wrap the sampler as an ml-bridge environment. Reward = sample
  quality. Train a controller to improve mixing.
