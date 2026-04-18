# Trait surfaces

This chapter names the traits that form `sim-soft`'s public API surface. Every cross-module interaction routes through one of them; every external crate (`sim-mjcf`, `sim-thermostat`, `sim-bevy`, `sim-ml-chassis`, `cf-design`) sees `sim-soft` as a set of trait implementations, not as a pile of structs.

| Section | What it covers |
|---|---|
| [Core traits](01-traits/00-core.md) | `Material`, `Element`, `Mesh`, `ContactModel`, `Solver`, `Differentiable`, `Observable` — one trait per responsibility, with minimal associated-type surface |
| [Composition rules](01-traits/01-composition.md) | How traits compose — multi-material meshes as `Mesh<M: Material>`, viscoelasticity as a `Material` decorator on a hyperelastic core, anisotropy as `Material` with a fiber-direction associated type, GPU backends as a `Solver` with its own `Tape` associated type |

Two claims Ch 01 rests on:

1. **Trait generics on hot paths, trait objects on cold paths.** Per-element stiffness assembly, per-contact-pair barrier evaluation, and per-vertex material sampling are generic in the material and element types so the compiler monomorphizes and inlines. Heterogeneous solver composition and runtime-dispatched multi-material assignment use trait objects. The split is deliberate: `Material` is generic, `Solver` is `dyn`-safe on the public boundary. Part 11 Ch 01's sub-chapter on core traits pins the signature of each.
2. **Trait surface is public-stable before implementations are.** The point of pinning the trait surface early is that consumer crates (`sim-mjcf` coupling, `cf-design` bridge, `sim-ml-chassis` reward hookup) can be designed against the trait signatures before `sim-soft`'s internal implementations exist. [Part 11 Ch 03 build order](03-build-order.md) and [Part 12's milestones](../120-roadmap/00-dependencies.md) both rely on this separability — traits ship first, implementations fill in phase-by-phase.
