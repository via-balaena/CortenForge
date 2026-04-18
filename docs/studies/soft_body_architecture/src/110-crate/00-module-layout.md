# sim-soft module layout

`sim-soft` is organized into ten modules, each with a single responsibility. The organization inverts the common FEM-library shape — where `material`, `element`, and `mesh` are collapsed into one "fem" module — by pulling each concept into its own module so the trait boundaries are visible at the filesystem level. A new reader navigating the crate should be able to read any one module in isolation and understand what it does; cross-module concerns are routed through the trait surfaces defined in [Ch 01](01-traits.md).

The ten modules and their responsibilities:

| Module | Responsibility | Depends on |
|---|---|---|
| [`material/`](00-module-layout/00-material.md) | Constitutive laws, strain measures, hyperelastic / viscoelastic / anisotropic models, spatial material fields | Standalone — pure math |
| [`element/`](00-module-layout/01-element.md) | Shape functions, quadrature, element-level stiffness assembly, Tet4 / Tet10 / Hex8 | `material/` |
| [`mesh/`](00-module-layout/02-mesh.md) | Tet and hex meshes, adjacency, adaptive refinement, quality metrics | Standalone |
| [`contact/`](00-module-layout/03-contact.md) | IPC barrier, CCD, smoothed Coulomb friction, self-contact, broad-phase BVH | `mesh/` |
| [`solver/`](00-module-layout/04-solver.md) | Backward-Euler Newton loop, line-search, sparse linear solve on total potential energy (faer on CPU, wgpu-native on GPU) | All above |
| [`coupling/`](00-module-layout/05-coupling.md) | Co-simulation with `sim-mjcf` rigid bodies and `sim-thermostat` temperature field | `solver/`, plus sibling crates |
| [`gpu/`](00-module-layout/06-gpu.md) | wgpu compute kernels, sparse matrix layouts, GPU autograd tape | `material/`, `element/`, `solver/` |
| [`autograd/`](00-module-layout/07-autograd.md) | VJP registration, implicit-function-theorem bridge, time adjoint, checkpointing | `solver/`, `gpu/` |
| [`sdf_bridge/`](00-module-layout/08-sdf-bridge.md) | SDF → tet mesh tetrahedralization, material field sampling, live re-mesh | `mesh/`, plus `cf-design` |
| [`readout/`](00-module-layout/09-readout.md) | Observation extraction: stress per tet, contact pressure, temperature field, reward computation | `solver/` |

Two claims Ch 00 rests on:

1. **The module boundary is the trait boundary.** Every cross-module interaction goes through a trait defined in [Ch 01](01-traits.md). No module reaches into another's internal types; no module exports its concrete structs as public API. The consequence is that any module can be re-implemented in isolation (e.g., a CPU and GPU implementation of `solver/` coexisting, selected by feature flag) without touching its callers. This is the same design principle that the [`sim-ml-chassis`](../110-crate/02-coupling/03-ml-chassis.md) separation pays off in the existing tree.
2. **The layout is informed by the thesis, not by convention.** Most FEM libraries collapse `material/`, `element/`, and `mesh/` into one module because historically they were written that way; `sim-soft` separates them because the [Part 1 Ch 03 thesis](../10-physical/03-thesis.md) commits to SDF-valued material fields that must be sampled independently of element geometry, and to multi-material meshes where material and mesh compose independently. The separation pays for itself in the SDF pipeline and falls out naturally for single-material cases.
