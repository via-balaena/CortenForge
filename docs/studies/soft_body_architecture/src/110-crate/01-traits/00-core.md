# Core traits

Seven traits form `sim-soft`'s public API surface — one per externally visible responsibility from [Ch 00 module-layout](../00-module-layout.md)'s ten modules. The three modules without a core trait ([`coupling/`](../00-module-layout/05-coupling.md), [`gpu/`](../00-module-layout/06-gpu.md), [`sdf_bridge/`](../00-module-layout/08-sdf-bridge.md)) are infrastructure: `coupling/` is a sibling-crate bridge, `gpu/` delivers a `Solver` impl rather than its own trait, and `sdf_bridge/` is a design-boundary concern internal to the `cf-design` handshake.

The table lists each trait with the minimal associated-type surface the [parent Ch 01 claim 1](../01-traits.md) commits to. The hot-path / cold-path column cashes out **trait generics on hot paths, trait objects on cold paths** — the central performance pattern the crate stakes the Newton loop's inlining on.

| Trait | Module | Minimal surface | Hot-path form | Cold-path form |
|---|---|---|---|---|
| `Material` | [`material/`](../00-module-layout/00-material.md) | `energy`, `first_piola`, `tangent`, `validity` (4 items, [Part 2 Ch 00 §00](../../20-materials/00-trait-hierarchy/00-trait-surface.md)) | generic `M: Material` per-Gauss-point | n/a — always generic |
| `Element` | [`element/`](../00-module-layout/01-element.md) | `N(xi)`, `grad_N(xi)`, `gauss_points`, `n_dof` | generic `E: Element` per-tet | n/a — always generic |
| `Mesh` | [`mesh/`](../00-module-layout/02-mesh.md) | `n_tets`, `tet_vertices`, `adjacency`, `quality`, `equals_structurally` | concrete `TetMesh` on hot path | `dyn Mesh` on change-detection path |
| `ContactModel` | [`contact/`](../00-module-layout/03-contact.md) | `active_pairs`, `energy`, `gradient`, `hessian`, `ccd_toi` | generic `C: ContactModel` per-pair | `dyn ContactModel` at scene construction |
| `Solver` | [`solver/`](../00-module-layout/04-solver.md) | `step(tape, ...) -> NewtonStep`, `replay_step(...) -> NewtonStep`, associated `type Tape` | n/a — always `dyn` on the public boundary | `dyn Solver` for CPU/GPU runtime selection |
| `Differentiable` | [`autograd/`](../00-module-layout/07-autograd.md) | `register_vjp`, `ift_adjoint`, `time_adjoint`, `fd_wrapper` | generic hooks at VJP registration | n/a — registry, no hot path |
| `Observable` | [`readout/`](../00-module-layout/09-readout.md) | `stress_field`, `pressure_field`, `temperature_field`, `reward_breakdown` | closed-form per-tet / per-pair / per-vertex | n/a — closed-form |

Each trait's signature is pinned below.

## `Material` — 4 items, cited back

```rust
pub trait Material: Send + Sync {
    fn energy(&self, f: &Matrix3<f64>) -> f64;
    fn first_piola(&self, f: &Matrix3<f64>) -> Matrix3<f64>;
    fn tangent(&self, f: &Matrix3<f64>) -> SMatrix<f64, 9, 9>;
    fn validity(&self) -> ValidityDomain;
}
```

The full derivation — why four items, why `Send + Sync`, why `validity` on the trait — lives in [Part 2 Ch 00 §00 trait-surface](../../20-materials/00-trait-hierarchy/00-trait-surface.md). Part 11 Ch 01 cites it back; it does not restate.

## `Element` — per-element geometry

```rust
pub trait Element<const N: usize, const G: usize>: Send + Sync {
    fn shape_functions(&self, xi: Vec3) -> SVector<f64, N>;
    fn shape_gradients(&self, xi: Vec3) -> SMatrix<f64, N, 3>;
    fn gauss_points(&self) -> [(Vec3, f64); G];
    fn n_dof(&self) -> usize { 3 * N }
}

// Concrete impls:
//   impl Element<4, 1>  for Tet4  — 4 nodes, 1 Gauss point, constant strain
//   impl Element<10, 4> for Tet10 — 10 nodes, 4 Gauss points, linear strain
```

`Element` is const-generic in node and Gauss-point counts so per-element stack-allocated matrices avoid heap allocation on the hot path. [`element/`](../00-module-layout/01-element.md)'s per-tet assembly loop is `fn assemble<const N: usize, const G: usize, M: Material, E: Element<N, G>>(mat: &M, elem: &E, ...)`, monomorphized per `(N, G, M)` triple. Tet4 (`N=4, G=1`) and Tet10 (`N=10, G=4`) are different concrete impls; the element-type choice per region ([Part 3 Ch 00 §04](../../30-discretization/00-element-choice/04-tradeoff.md)) is a compile-time decision per assembly codepath, not runtime dispatch.

## `Mesh` — storage abstraction

```rust
pub trait Mesh: Send + Sync {
    fn n_tets(&self) -> usize;
    fn n_vertices(&self) -> usize;
    fn tet_vertices(&self, tet: TetId) -> [VertexId; 4];
    fn positions(&self) -> &[Vec3];
    fn adjacency(&self) -> &MeshAdjacency;
    fn quality(&self) -> &QualityMetrics;

    /// The mesh-equality predicate from [Ch 00 §02 mesh claim 3](../00-module-layout/02-mesh.md).
    fn equals_structurally(&self, other: &dyn Mesh) -> bool;
}
```

Phase A ships `TetMesh` as the one concrete impl; the trait exists for the forward compatibility argument in the [parent Ch 01 claim 2](../01-traits.md) — consumer modules (`solver/`, `contact/`, `sdf_bridge/`) compile against `M: Mesh` before Phase A's `TetMesh` exists, and a future GPU-native or hex-variant mesh representation lands as an additional impl without changing callers. On the hot path the concrete `TetMesh` is used directly (zero vtable overhead); `dyn Mesh` shows up only at change-detection and scene-construction boundaries.

## `ContactModel` — energy-term interface

```rust
pub trait ContactModel: Send + Sync {
    fn active_pairs(&self, mesh: &dyn Mesh, positions: &[Vec3]) -> Vec<ContactPair>;
    fn energy(&self, pair: &ContactPair, positions: &[Vec3]) -> f64;
    fn gradient(&self, pair: &ContactPair, positions: &[Vec3]) -> ContactGradient;
    fn hessian(&self, pair: &ContactPair, positions: &[Vec3]) -> ContactHessian;
    fn ccd_toi(&self, pair: &ContactPair, x0: &[Vec3], x1: &[Vec3]) -> f64;
}
```

[`contact/`](../00-module-layout/03-contact.md) ships `IpcBarrierModel` as the one concrete impl; penalty and impulse are rejected at the module level per Ch 00 §03 claim 1. The trait exists for the same forward-compatibility reason `Mesh` does — future contact refinements (e.g., mortared-interface contact for glued layers) would land as additional impls, but are not Phase A–I work.

## `Solver` — the only `dyn`-safe public trait

```rust
pub trait Solver: Send + Sync {
    type Tape;

    fn step(
        &mut self,
        tape: &mut Self::Tape,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta: &Tensor<f64>,
        dt: f64,
    ) -> NewtonStep<Self::Tape>;

    fn replay_step(
        &self,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta: &Tensor<f64>,
        dt: f64,
    ) -> NewtonStep<Self::Tape>;

    fn current_dt(&self) -> f64;
    fn convergence_tol(&self) -> f64;
}
```

Two concrete impls: `CpuNewtonSolver<Tape = CpuTape>` (Phase B) and `GpuNewtonSolver<Tape = GpuTape>` (Phase E). Both are selectable at scene construction via `Box<dyn Solver<Tape = _>>`; the runtime dispatch cost is paid once per step, not once per Newton inner iteration. The associated `Tape` type is what lets CPU and GPU solvers each use the tape representation their VJPs are registered against — [`autograd/`](../00-module-layout/07-autograd.md) carries separate registrations for CPU closed-form and GPU compute-kernel VJPs, and the `Solver` impl picks one.

`Solver::step`'s `&mut self` admits *intra-call* mutability for logging, diagnostic accumulation, and internal scratch — line-search iteration counts, Newton residual histories, per-step telemetry that does not affect the returned `NewtonStep`. *Cross-call* output-affecting state — adaptive tolerance learning from previous steps, warm-start iterates persisting across `step` invocations, cached line-search history influencing backtracking decisions — is forbidden. A `step(&mut self, ...)` impl whose return value at the same `(x_prev, v_prev, theta, dt)` depends on prior calls violates the [γ-locked `ForwardMap` determinism-in-θ contract](../../100-optimization/00-forward.md) and breaks the [checkpointed-step replay pattern from Part 6 Ch 04](../../60-differentiability/04-checkpointing.md) that the time-adjoint machinery rests on. Same mutation-discipline template as `Differentiable` (§below) and the chassis [`Preconditioner`](../../80-gpu/02-sparse-solvers/03-preconditioning.md) trait — uniform across every impl-side trait the autograd substrate consumes.

`Solver::replay_step` is the pure-function counterpart called from the [checkpointed-step VJP](../../60-differentiability/04-checkpointing/00-uniform.md)'s `&self` context during backward-pass replay. Given the same `(x_prev, v_prev, theta, dt)`, `replay_step` produces the same `NewtonStep<Self::Tape>` as the original forward `step` — this is the [checkpointed-step replay contract](../../60-differentiability/04-checkpointing/02-tradeoff.md)'s "Replay is bit-reproducible given the stored primal $(x, v, \Delta t)$" commitment. No tape is written; `replay_step` has no `&mut Tape` parameter. The `factor` and `dr_dtheta` fields in the returned `NewtonStep` are the artifacts the VJP back-substitutes against; they are released by end-of-scope RAII before the VJP returns — the [tape-lifetime factor-release contract from Part 6 Ch 02 §01](../../60-differentiability/02-implicit-function/01-linear-solve.md) refined to per-VJP-call scope, since `replay_step`'s rebuilt factor is not persisted on any tape. Any `&self`-state on the impl that `replay_step` reads is constructor-state (solver config, scene references) — nothing that depends on prior call history.

## `Differentiable` — VJP registration

```rust
pub trait Differentiable {
    type Tape;

    fn register_vjp(&mut self, forward_key: TapeNodeKey, vjp: Box<dyn VjpOp>);

    fn ift_adjoint(
        &self,
        tape: &Self::Tape,
        step: &NewtonStep<Self::Tape>,
        upstream: &Tensor<f64>,
    ) -> Tensor<f64>;

    fn time_adjoint(&self, tape: &Self::Tape, rollout: &[NewtonStep<Self::Tape>], upstream: &Tensor<f64>) -> Tensor<f64>;

    fn fd_wrapper(&self, forward: &dyn Fn(&Tensor<f64>) -> Tensor<f64>, theta: &Tensor<f64>) -> (Tensor<f64>, GradientEstimate);
}
```

`Differentiable` is a registry, not a hot-path trait. It hands out `TapeNodeKey`s that `Material`, `Element`, and `ContactModel` VJPs register against; it replays them in `ift_adjoint` and `time_adjoint`. The associated `Tape` type matches the `Solver::Tape` type so forward and backward share one tape representation.

`Differentiable`'s `&mut self` on `forward` / `backward` admits *intra-call* mutability: the expected pattern is `forward` factoring the Hessian and stashing it in impl fields, `backward` reusing the factor for the IFT back-substitute, the tape dropping the whole chain at end of scope. Impl-field state that is scratch-across-one-call — factor stash, cached Jacobian pattern, line-search diagnostics, [time-adjoint checkpoint payloads](../../60-differentiability/03-time-adjoint.md) — is expected and load-bearing.

*Cross-call* state — anything readable in a later `evaluate` call and affecting output — is forbidden. A preconditioner cached across calls "for performance," a warm-start iterate persisting from a previous θ, per-call statistics accumulating into output-affecting decisions: all violate the [γ-locked `ForwardMap` determinism-in-θ contract](../../100-optimization/00-forward.md) and invalidate the [BayesOpt cache from Part 10 Ch 02](../../100-optimization/02-bayesopt.md). The `Differentiable` trait docstring carries this distinction as canonical contract — the chassis [`VjpOp` docstring](../../80-gpu/04-chassis-extension/02-vjp-api.md) covers below-the-tape purity; `Differentiable` impls live *above* the tape, where cross-call state is admitted by the `&mut self` signature but forbidden by contract.

## `Observable` — readout surface

```rust
pub trait Observable {
    type Step;   // matches the caller's Solver::Tape-parameterized NewtonStep type

    fn stress_field(&self, step: &Self::Step) -> StressField;
    fn pressure_field(&self, step: &Self::Step) -> PressureField;
    fn temperature_field(&self, step: &Self::Step) -> TemperatureField;

    /// The γ-locked RewardBreakdown from [Part 10 Ch 00](../../100-optimization/00-forward.md).
    fn reward_breakdown(&self, step: &Self::Step, theta: &Tensor<f64>) -> RewardBreakdown;
}
```

`Observable` is the trait behind the [`ForwardMap` γ-locked API](../../100-optimization/00-forward.md)'s `evaluate` method. The associated `Step` type threads the concrete `NewtonStep<Tape>` from the active `Solver` impl into the readout code without committing readout to one tape variant. Concrete impls: `SimSoftCpuObservable { type Step = NewtonStep<CpuTape>; }` and `SimSoftGpuObservable { type Step = NewtonStep<GpuTape>; }`, both held by [`readout/`](../00-module-layout/09-readout.md); consumers outside `sim-soft` ([`sim-opt`](../02-coupling/03-ml-chassis.md), [`sim-bevy`](../02-coupling/02-bevy.md)) interact with whichever `Observable` the chosen backend delivers. Determinism-in-θ per [Ch 00 §09 readout claim 2](../00-module-layout/09-readout.md) is the trait-level contract every impl must honor.

## What this sub-leaf commits the crate to

- **Seven traits, one per externally visible responsibility.** No additions in Phase A–I without cross-crate review; an eighth trait would widen the audit surface that "A-grade or it doesn't ship" binds against.
- **`Solver` is the only trait with runtime dispatch on the public boundary.** CPU vs GPU selection is a `dyn Solver` call; every other public trait monomorphizes through. The [Part 11 Ch 01 parent claim 1](../01-traits.md) commits to this split.
- **Associated `Tape` type threads from `Solver` through `Differentiable`.** One tape per solver impl, VJPs registered against that tape, forward and backward see the same type. No cross-tape bridging, no dynamic tape dispatch.
- **Trait-surface changes are not local edits.** Adding a method to `Material` or `Element` ripples through [`element/`](../00-module-layout/01-element.md) assembly codegen, the [gradcheck suite](../04-testing/03-gradcheck.md), and every impl. [Parent Ch 01 claim 2](../01-traits.md)'s "trait-stable-before-impls" commitment binds on every trait in this table.
