# Composition rules

[`00-core.md`](00-core.md) pinned the seven traits. This leaf names the patterns by which those traits compose — how multi-material meshes compose `Mesh` with per-region `Material` references, how CPU and GPU `Solver` impls share a `Differentiable` registry, how [`element/`](../00-module-layout/01-element.md) and [`contact/`](../00-module-layout/03-contact.md) VJPs stack onto one `Differentiable` without leaking into each other. Material-specific composition (viscoelasticity as `Viscoelastic<M>`, anisotropy as `Anisotropic<M, R>`, thermal coupling as `Thermal<M>`) is already committed in [Part 2 Ch 00 §01 composition rules](../../20-materials/00-trait-hierarchy/01-composition.md); this leaf cites that commitment rather than restating it and covers the cross-trait patterns that Part 2 does not.

## Material compositions are cited, not duplicated

[Part 2 Ch 00 §01](../../20-materials/00-trait-hierarchy/01-composition.md) commits the Material-side composition rules: `Viscoelastic<M: Material>` for rate dependence, `Anisotropic<M: Material, R: FiberDirection>` for fiber reinforcement, `Thermal<M: Material>` for temperature-dependent modulus and thermal expansion, canonical outside-in order `Thermal<Viscoelastic<Anisotropic<Iso>>>` enforced at construction via `Material::compose`. Per-element history (Prony hereditary state) lives in [`element/`](../00-module-layout/01-element.md)'s per-tet storage, not on the trait.

Part 11 Ch 01 honors this. No Part-11-side rule below contradicts it; any proposed change to the decorator set or the composition order would require a Part 2 Ch 00 §01 edit first — a deliberate edit at that chapter's scope, not an incidental downstream Part-11 change.

## Multi-material meshes — `Mesh` + `Field<MaterialRef>`

The parent [Ch 01 shorthand "multi-material meshes as `Mesh<M: Material>`"](../01-traits.md) refers to the single-material specialization case. The general multi-material pattern uses [`mesh/`](../00-module-layout/02-mesh.md)'s per-region tag plus a `Field` lookup keyed on the tag:

```rust
pub struct MultiMaterialScene {
    mesh: TetMesh,
    material_by_region: Vec<Box<dyn Material>>,
}

impl MultiMaterialScene {
    pub fn material_at_tet(&self, tet: TetId) -> &dyn Material {
        let region = self.mesh.region_tag(tet);
        self.material_by_region[region as usize].as_ref()
    }
}
```

On the hot path [`element/`](../00-module-layout/01-element.md)'s assembly loop looks up the material per-tet and calls through `dyn Material`, not a monomorphized `M: Material` — the one place the "trait generics on hot paths" default bends. The cost is a vtable call per tet; at Phase B throughput this measures below the sparse-factorization fraction of step wall time. Single-material scenes opt into the monomorphized fast path via a separate `UniformMaterialScene<M: Material>` type that the same `Solver::step` entry point is also generic over.

[Part 3 Ch 03 — multi-material interfaces](../../30-discretization/03-interfaces.md) commits to bonded and frictional-sliding interfaces between regions, both of which compose with this pattern: `bonded` requires no extra modeling (the same `Mesh` + per-region lookup), `frictional-sliding` requires an interface element in [`element/`](../00-module-layout/01-element.md) that spans two regions — a Phase-H concern, not a trait-composition concern.

## Solver backends — one trait, two `Tape` types

`Solver` is the only `dyn`-safe public trait ([Ch 01 §00 core traits](00-core.md)). CPU and GPU backends are two impls:

```rust
pub struct CpuNewtonSolver { /* faer factor, Armijo line search, ... */ }
impl Solver for CpuNewtonSolver {
    type Tape = CpuTape;   // Records elementary ops + hand-written VJPs
    // ...
}

pub struct GpuNewtonSolver { /* wgpu kernels, CG preconditioner, ... */ }
impl Solver for GpuNewtonSolver {
    type Tape = GpuTape;   // Records kernel-call graph + GPU VJPs
    // ...
}
```

The associated `Tape` type is what keeps the two backends cleanly separated. VJPs registered against `CpuTape` do not accidentally get called during a GPU solve, and vice versa. [`autograd/`](../00-module-layout/07-autograd.md)'s `Differentiable` impls are parameterized by the tape type — `CpuDifferentiable: Differentiable<Tape = CpuTape>` and `GpuDifferentiable: Differentiable<Tape = GpuTape>` — so the registry's identity is tied to the backend. There is no hybrid CPU+GPU solve in the Phase A–I roadmap; if a future phase splits the mechanical solve across devices, the composition pattern will change.

Selection happens at scene construction:

```rust
let solver: Box<dyn Solver<Tape = _>> = match backend {
    Backend::Cpu => Box::new(CpuNewtonSolver::new(...)),
    Backend::Gpu => Box::new(GpuNewtonSolver::new(...)),
};
```

One `dyn Solver` call per step, the rest of the Newton-loop inner iterations run inside the concrete impl without further dispatch. This is the cold-path / hot-path split the [parent Ch 01 claim 1](../01-traits.md) commits to, made concrete.

## VJPs stack onto `Differentiable` — no per-module registries

Each of `Material`, `Element`, and `ContactModel` has hand-written VJPs per [Part 6 Ch 01 custom-VJPs](../../60-differentiability/01-custom-vjps.md). All of them register against the *same* `Differentiable` instance — there is one VJP registry per `Solver` instance, keyed by `TapeNodeKey`:

```rust
impl CpuNewtonSolver {
    pub fn new(...) -> Self {
        let mut diff = CpuDifferentiable::new();

        // Material::tangent and Material::first_piola closed-form VJPs
        material::register_vjps(&mut diff);

        // Element's fused per-tet assembly VJP per Part 6 Ch 01 §01
        element::register_vjps(&mut diff);

        // ContactModel's fused barrier VJP per Part 6 Ch 01 §02
        contact::register_vjps(&mut diff);

        Self { differentiable: diff, ... }
    }
}
```

No per-module `Differentiable` instance, no cross-registry lookup. The IFT adjoint's one back-substitution on `solver/`'s cached `Llt<f64>` consumes upstream gradients that the stacked per-module VJPs produced during the forward pass. This is what keeps [Part 6 Ch 02 IFT](../../60-differentiability/02-implicit-function.md)'s "one factor, many RHSes" cost model honest — if each module had its own `Differentiable`, the adjoint would have to stitch registries and the single-registry guarantee from [Ch 00 §07 autograd claim 2](../00-module-layout/07-autograd.md) would leak.

## Element-type composition — homogeneous regions, deferred fully-mixed

Tet4 and Tet10 can coexist in one scene via per-region element-type tagging: regions marked for Tet10 use a Tet10-specific assembly codepath, regions marked for Tet4 use a Tet4-specific one. Conforming interfaces between Tet4 and Tet10 regions require constraint enforcement at the shared edges (the Tet10 edge's midpoint node must lie on the Tet4 edge); this lands in [`element/`](../00-module-layout/01-element.md) as an interface-element impl, not as a trait composition.

Fully-adaptive mixed-element refinement — per-tet element-type choice driven by stress feedback — is deferred past Phase H per [Part 3 Ch 00 §04](../../30-discretization/00-element-choice/04-tradeoff.md). The trait-level composition pattern is ready for it (per-tet `dyn Element` dispatch via the same vtable-on-hot-path cost as multi-material above), but the assembly-layer complexity is what the deferral addresses, not the trait design.

## Two claims

**1. Composition is either monomorphized decorators or runtime-dispatched `Box<dyn …>`.** No middle ground. `Viscoelastic<Anisotropic<NeoHookean>>` is a three-layer generic struct that monomorphizes to a single concrete type at compile time. Multi-material scenes, CPU-vs-GPU solver selection, and future fully-mixed-element composition all route through `Box<dyn Trait>` at runtime. The crate does not use type-erasure tricks (hand-rolled enum dispatch over a closed set of impls, runtime downcasts via `Any`, interior-mutability tricks that hide dispatch from the trait) that sit between the two extremes.

**2. Trait impls compose; traits do not.** Adding a new capability is a composition over existing traits (a new `Material` decorator, a new `Solver` impl, a new `Element` implementor), not a new trait. The seven traits from [Ch 01 §00 core traits](00-core.md) are the complete public trait surface. If a future phase needs capability outside this set, the path is: is it a composition over an existing trait? If yes, add the composition. If no, the crate's responsibility boundary is wrong and the trait surface must be re-audited at the cross-crate level, not grown locally.

## What this sub-leaf commits the crate to

- **Material, Element, and ContactModel VJPs share one `Differentiable` instance per `Solver`.** No per-module registries; no cross-registry stitching. The IFT adjoint's cost model depends on this.
- **Multi-material composition uses `dyn Material` per-tet dispatch on the hot path.** Single-material scenes opt into monomorphized assembly via a separate scene type. The two paths share the same `Solver::step` entry point.
- **Composition order for Material decorators is non-negotiable.** `Thermal<Viscoelastic<Anisotropic<Iso>>>` outside-in, enforced at construction by `Material::compose` per [Part 2 Ch 00 §01](../../20-materials/00-trait-hierarchy/01-composition.md). Part 11 does not rescind or loosen this ordering.
- **Backend selection is a one-time `dyn Solver` dispatch at scene construction.** The per-step cost is one vtable call; the per-inner-iteration cost is zero. CPU-and-GPU hybrid solves are not in the Phase A–I roadmap.
