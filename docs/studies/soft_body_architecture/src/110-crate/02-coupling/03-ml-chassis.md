# With sim-ml-chassis

`sim-ml-chassis` is the shared autograd substrate under `sim-soft`, `sim-rl`, and `sim-opt`. It owns the generic reverse-mode tape, the `Tensor<f64>` type every cross-crate gradient signal flows through, the `Var` / `Node` / `BackwardOp` machinery for elementary-op VJPs, and — as [Part 8 Ch 04](../../80-gpu/04-chassis-extension.md) commits — the parallel GPU backend (`GpuTensor<f32>` + `GpuTape` + `VjpRegistry::register`) that Phase E brings online. `sim-soft` is a consumer: it registers physics-fused VJPs against the chassis's tape, emits a `ForwardMap`-shaped surface through which optimizers query reward and gradient, and never owns the tape itself. This sub-leaf specifies the cross-crate boundary at that level — what `sim-soft` exports to anything running on the chassis, and what the chassis guarantees back.

## What crosses the boundary

| Field | Direction | Representation | Required? |
|---|---|---|---|
| Design-parameter vector $\theta$ | consumer → `sim-soft` via `ForwardMap::evaluate` | `Tensor<f64>` (chassis-owned type) | yes |
| `(RewardBreakdown, EditResult)` | `sim-soft` → consumer | γ-locked struct pair from [Part 10 Ch 00](../../100-optimization/00-forward.md) | yes |
| `(Tensor<f64>, GradientEstimate)` | `sim-soft` → consumer via `ForwardMap::gradient` | gradient plus exact/noisy flag | yes |
| `Tape` — reverse-mode tape with physics-fused VJP entries | bidirectional across the chassis API | chassis-owned; `sim-soft::autograd/` registers VJPs against it | yes |
| `GpuTensor` / `GpuTape` + `VjpRegistry` registration | chassis → `sim-soft::gpu/` | chassis-owned tensor backend + global kernel-pair registry | Phase E onward |
| `Observable` trait implementations | `sim-soft` → consumer | readout surface (stress, pressure, temperature, reward_breakdown) | yes |

The chassis exposes `Tape` and `Tensor<f64>` as its core contract; `sim-soft` adds three physics-fused VJPs (FEM assembly, IPC barrier, IFT adjoint) to the chassis's registry and otherwise lets the chassis do its job. This matches the `autograd/` module claim 1 commitment: [the tape is the chassis's; `sim-soft::autograd/` registers against it](../00-module-layout/07-autograd.md). No autograd framework dependency; the tape surface is narrow enough to audit in an afternoon.

## The `ForwardMap` trait is the user-facing boundary

The γ-locked [`ForwardMap` trait](../../100-optimization/00-forward.md) is the single trait any consumer (`sim-opt`, `sim-rl`, a custom optimizer stack, a CI regression harness) calls to evaluate a design:

```rust
use sim_ml_chassis::{Tape, Tensor};
use sim_soft::{EditResult, GradientEstimate, RewardBreakdown};

pub trait ForwardMap {
    fn evaluate(
        &mut self,
        theta: &Tensor<f64>,
        tape:  &mut Tape,
    ) -> (RewardBreakdown, EditResult);

    fn gradient(
        &mut self,
        theta: &Tensor<f64>,
        tape:  &Tape,
    ) -> (Tensor<f64>, GradientEstimate);
}
```

The trait is exported from [`sim-soft::readout/`](../00-module-layout/09-readout.md) because `readout/` is the module that assembles `RewardBreakdown` from a converged Newton step. [Ch 00 `readout/` claim 1](../00-module-layout/09-readout.md) commits `RewardBreakdown` is γ-locked — per-component (pressure-uniformity, coverage, peak-pressure barrier, effective-stiffness bound), not a pre-composed scalar — so downstream consumers that want only a scalar call `breakdown.score_with(&weights)` under [Part 1 Ch 01 §04](../../10-physical/01-reward/04-composition.md), and consumers that want per-term breakdown (preference GP updates on per-term weights, residual-GP per-component corrections in `sim-opt`) read the struct directly.

`EditResult` carries the SDF edit classification (`ParameterOnly` / `MaterialChanging` / `TopologyChanging`) and wall-time-ms for the evaluation, per [Part 7 Ch 04](../../70-sdf-pipeline/04-live-remesh.md). `GradientEstimate` is `{Exact, Noisy { variance }}` per [Part 6 Ch 05](../../60-differentiability/05-diff-meshing.md). Both cross the chassis boundary as part of the `ForwardMap` return payload because downstream cost-aware and noise-tolerant machinery reads them directly.

## The chassis gains a GPU backend to support this coupling

The parent Ch 02 spine commits "chassis is extended with a GPU tensor backend and a VJP registration API to support custom solver-level gradients." This is the [Part 8 Ch 04](../../80-gpu/04-chassis-extension.md) deliverable, sized to what [`sim-soft::gpu/`](../00-module-layout/06-gpu.md) consumes:

- **`GpuTensor<f32>`** — the chassis's GPU-resident tensor type, wgpu-backed, parallel to the existing CPU `Tensor<f64>`. Phase E default; generic bound admits future f16 / f64 variants.
- **`GpuTape`** — the chassis's GPU-side tape, recording kernel-call graphs the backward pass replays in reverse. Parallel to the existing CPU `Tape`; computation graphs with both CPU and GPU entries are supported via automatic-readback dispatch.
- **`VjpRegistry::register(KernelId, VjpKernelHandle)`** — chassis-init-time global registry for GPU kernel pairs (forward + VJP, both pre-compiled WGSL). `sim-soft::gpu/` registers its solver-specific kernels — per-element neo-Hookean assembly, IPC barrier, sparse MVP, preconditioned CG — at chassis init. [Part 8 Ch 04 §02](../../80-gpu/04-chassis-extension/02-vjp-api.md) commits the API surface.

The extension is **extension, not replacement** per [Part 8 Ch 04](../../80-gpu/04-chassis-extension.md): existing CPU tape / tensor types are not deprecated, renamed, or reshaped. `sim-rl`'s baselines and non-GPU consumers continue to work unchanged. The GPU extension ships as a new `sim_ml_chassis::gpu` module alongside the existing CPU surface; the two backends interop at the tape level.

`sim-soft` is the primary consumer of the GPU extension, but not the only one — if a future `sim-rl` algorithm wants GPU batch training, it gets the GPU backend for free. The commitment from [Part 8 Ch 04 claim 3](../../80-gpu/04-chassis-extension.md) is that GPU autograd is a chassis-level capability, not a `sim-soft`-local hack, and the API is specified at a level where any chassis consumer can adopt it.

## Determinism-in-θ and the cached-tape contract

The [γ-locked `ForwardMap` determinism-in-θ contract](../../100-optimization/00-forward.md) is load-bearing on this coupling. Repeated `evaluate` calls at the same $\theta$ produce the same `RewardBreakdown` modulo cache state — no stochastic sim-parameter updates between calls, no RNG inside `sim-soft`, no wall-clock sensitivity. [Ch 00 `readout/` claim 2](../00-module-layout/09-readout.md) cashes this out for the reward side (closed-form stress, closed-form pressure, closed-form reward terms); [Ch 00 `autograd/` claim 3](../00-module-layout/07-autograd.md) cashes it out for the gradient side (tape replay is deterministic given recorded nodes, checkpointed re-computation is deterministic by construction).

The reason the chassis boundary cares: [Part 10 Ch 02's BayesOpt GP cache](../../100-optimization/02-bayesopt.md) maintains cached $(\theta_i, R_\text{sim}(\theta_i), \nabla_\theta R_\text{sim}(\theta_i))$ training data across the entire optimization run, and [Part 10 Ch 05's residual-GP updates](../../100-optimization/05-sim-to-real/01-online.md) slot underneath it as a separate correction layer. Determinism-in-θ is what keeps the cached training data valid as the sim-to-real correction layer updates; without it, every residual-GP update would force a cache invalidation sweep through the BayesOpt inner loop's training data and the loop's performance model from [Part 10 Ch 00](../../100-optimization/00-forward.md) breaks. The chassis boundary surfaces this contract verbatim in the `ForwardMap` docstring.

## The `Observable` trait is the shared readout surface

Beyond `ForwardMap`, the chassis-coupled consumer also sees the [`Observable` trait from Ch 01 §00](../01-traits/00-core.md) — the trait behind `ForwardMap::evaluate`'s `RewardBreakdown` return. `Observable` has an associated `type Step` that threads the concrete `NewtonStep<Tape>` type from the active `Solver` impl (CPU or GPU) into readout code without committing readout to one tape variant. Consumer crates outside `sim-soft` — [`sim-opt`](#where-sim-opt-sits) and [`sim-bevy`](02-bevy.md) — interact with whichever `Observable` the chosen backend delivers.

`sim-opt` reads `reward_breakdown` + `stress_field` for its optimizer state; `sim-bevy` reads `stress_field` + `pressure_field` + `temperature_field` for the visual snapshot. One trait, two consumers, same backend-polymorphic `Step` threading. This is the one public trait `sim-soft` exports that is wider than `Solver`'s `dyn`-safe single-trait surface — and even then, only by the per-consumer narrow slice each cares about. No consumer reaches through `Observable` into internal `sim-soft` structures; the trait is the bottleneck.

## Where `sim-opt` sits

`sim-opt` is the downstream optimizer crate that builds on `sim-ml-chassis` plus `sim-soft::readout/`. It owns the full γ-locked optimizer state surface from [Part 10](../../100-optimization/00-forward.md):

- **`GradientEnhancedGP`** ([Part 10 Ch 02](../../100-optimization/02-bayesopt.md)) — the gradient-enhanced GP over $(\theta, R, \nabla_\theta R)$ triples; Matérn-5/2 + ARD; cached `faer::sparse::linalg::solvers::Llt<f64>` factor.
- **`Acquisition` enum** ([Part 10 Ch 02 / Ch 04](../../100-optimization/02-bayesopt.md)) — `ExpectedImprovement`, `UpperConfidenceBound { beta }` (form $\mu + \beta_t \sigma$), `Thompson`, `MaxValueEntropySearch { n_ystar_samples }`, `BALDRegression`. Plus **`CostAwareAcquisition`** wrapping a base `Acquisition` with a `CostModel`.
- **`PreferenceGP`** / **`BradleyTerryGP`** / **`DuelingBanditPolicy`** ([Part 10 Ch 03](../../100-optimization/03-preference.md)) — probit-likelihood single-designer preference GP, logit-likelihood tournament Bradley-Terry GP, online preference-acquisition policy. Both GPs cache `Llt<f64>` at the Laplace mode.
- **`SimToRealCorrection`** / **`MeasurementReport`** / **`OnlineUpdateOutcome`** / **`MeasurementProtocol`** ([Part 10 Ch 05](../../100-optimization/05-sim-to-real.md)) — sim-to-real residual-GP correction, physical-print measurement record, online-update state-machine outcome, measurement-protocol descriptor.

**`sim-soft` does not see any of these types.** `MeasurementReport` crosses the `sim-opt` boundary only — not `sim-soft`, not `sim-ml-chassis`. `SimToRealCorrection::correct(theta, r_sim) -> RewardBreakdown` is the caller of [Ch 00 `readout/`'s `RewardBreakdown::apply_residuals`](../00-module-layout/09-readout.md), and the only `sim-soft`-visible type that crosses is `RewardBreakdown` itself. This is the crate-boundary discipline [Part 10 Ch 05 §01](../../100-optimization/05-sim-to-real/01-online.md) commits to: `sim-opt` is the seam between optimization-stack state (residual GP, BayesOpt GP, preference GP) and physical-world state (print records, measurements, printer configuration).

The dependency direction is `sim-opt` → `sim-ml-chassis` + `sim-soft`; no direction of flow goes the other way. This is what keeps `sim-soft` headless-runnable in CI without a printer attached and lets the GP machinery be swapped, extended, or bypassed without forcing changes upstream.

## Phase A–I deferral discipline — the physical-print loop is post-Phase-I

The [Part 12 Ch 07 open-questions "priority-2"](../../120-roadmap/07-open-questions.md) commitment is explicit: **the full design-print-rate loop — `MeasurementReport` ingestion, residual-GP online updates per [Part 10 Ch 05 §01](../../100-optimization/05-sim-to-real/01-online.md), batch-rating workflow, cross-print drift detection — is post-Phase-I.** These are not Phase A–I deliverables. [Part 12 Ch 06](../../120-roadmap/06-optimization.md)'s optimization milestone ships the inner BayesOpt loop against $R_\text{sim}(\theta)$; the outer loop that closes around physical prints is deferred to post-Phase-I because it requires a specific printer, measurement instruments, and a measurement protocol that the simulation crate does not commit to acquiring.

The chassis coupling honors this: the `ForwardMap` surface ships in Phase D; `GradientEnhancedGP` / `Acquisition` / `CostAwareAcquisition` ship in `sim-opt` on a schedule Parts 10 Ch 02 and Ch 04 commit to; `PreferenceGP` / `BradleyTerryGP` / `DuelingBanditPolicy` and `SimToRealCorrection` / `MeasurementReport` / `OnlineUpdateOutcome` / `MeasurementProtocol` are deferred to post-Phase-I per [Part 10 Ch 03 "not on the Phase A–I critical path"](../../100-optimization/03-preference.md) and [Part 10 Ch 05 "What this does NOT commit"](../../100-optimization/05-sim-to-real.md). Part 12 Ch 07 is the committed home for the physical-print loop; the chassis coupling does not absorb it into the Phase A–I roadmap by back-door.

## What the chassis coupling does not carry

- **No optimizer state in `sim-soft`.** `GradientEnhancedGP`, `PreferenceGP`, `BradleyTerryGP`, `Acquisition`, `CostAwareAcquisition`, `DuelingBanditPolicy`, `SimToRealCorrection` all live in `sim-opt`. `sim-soft` produces the inputs those optimizers consume; it does not consume them itself. ([`readout/` "does not carry"](../00-module-layout/09-readout.md) commits the same discipline at the module level.)
- **No elementary-op VJPs in `sim-soft::autograd/`.** Matmul, add, broadcast, elementwise-nonlinearity — those are chassis hot-path, already recorded by the chassis's tape. `sim-soft` registers the *physics-fused* VJPs (FEM assembly, IPC barrier, IFT adjoint) per [Ch 00 `autograd/`](../00-module-layout/07-autograd.md) and relies on the chassis for everything below them.
- **No reward-weight learning.** Reward-composition weights $w$ are part of $\theta$ per [Part 10 Ch 00](../../100-optimization/00-forward.md). `PreferenceGP` / `BradleyTerryGP` update them; `sim-soft` does not. The coupling passes the current weights through with $\theta$; no decision about what they should be lives in `sim-soft`.
- **No printer-state plumbing.** `MeasurementReport` / `OnlineUpdateOutcome` / `PrinterId` / `ResinBatchId` never cross into `sim-soft` or `sim-ml-chassis`. They are `sim-opt`-only types, deferred to post-Phase-I.

## What this sub-leaf commits the crate to

- **`sim-ml-chassis` is the chassis; `sim-soft` consumes it.** The generic tape, `Tensor<f64>`, `Var` / `Node` / `BackwardOp`, and (post-Phase-E) `GpuTensor<f32>` / `GpuTape` / `VjpRegistry` all live in the chassis. `sim-soft` registers physics-fused VJPs against the chassis's tape via [`autograd/`](../00-module-layout/07-autograd.md) (CPU) and [`gpu/`](../00-module-layout/06-gpu.md) (GPU).
- **`ForwardMap` and `Observable` are the two public traits crossing the chassis boundary.** Both are γ-locked; their signatures and semantics are fixed.
- **`RewardBreakdown` is the per-component return type.** `score_with(&weights)` composes to scalar; `apply_residuals(...)` composes per-component residuals from `SimToRealCorrection::correct` per [Ch 00 `readout/` claim 3](../00-module-layout/09-readout.md).
- **`sim-opt` owns the full optimizer state surface.** `GradientEnhancedGP`, `PreferenceGP`, `BradleyTerryGP`, `Acquisition` enum variants, `CostAwareAcquisition`, `DuelingBanditPolicy`, `SimToRealCorrection`, `MeasurementReport`, `OnlineUpdateOutcome`, `MeasurementProtocol`. `sim-soft` does not see these types.
- **Phase A–I deferral discipline is enforced.** Physical-print loop machinery (`MeasurementReport` ingestion, residual-GP online updates, batch-rating, cross-print drift detection) is post-Phase-I per [Part 12 Ch 07 "priority-2"](../../120-roadmap/07-open-questions.md), not a Phase A–I deliverable under [Part 12 Ch 06](../../120-roadmap/06-optimization.md).
- **Determinism-in-θ is the contract the chassis coupling depends on.** `ForwardMap::evaluate` returns the same `RewardBreakdown` at the same $\theta$ modulo cache state. Residual-GP updates in `sim-opt` slot underneath without invalidating BayesOpt training data.
- **Phase D is when this coupling closes.** Phases A–C run `sim-soft` without the `sim-ml-chassis` reward seam live; [Phase D — "first working sim-soft"](../03-build-order.md#the-committed-order) is when a synthetic 2-parameter design space optimizes end-to-end through the `ForwardMap` surface.
