# SDF as universal design primitive

The [Part 1 Ch 03 thesis](../10-physical/03-thesis.md) commits `sim-soft` to working directly on hyperelastic material with IPC contact on FEM tetrahedra; the [canonical problem](../10-physical/00-canonical.md) commits the book to a compliant-cavity-plus-probe system where the cavity's geometry is *the thing being designed*. This chapter names what the designer hands to `sim-soft` at the front door: **a signed distance field plus a material field, not a mesh and not a B-rep.** The mesh is derived from the SDF on ingest ([Ch 01](01-tet-strategies.md)) and re-derived on every design edit ([Ch 04](04-live-remesh.md)). The material parameters are sampled from the field per-tet ([Ch 02](02-material-assignment.md)). Nothing in the user-facing design surface is mesh-valued, and nothing is CAD-B-rep-valued. Both alternatives are rejected, explicitly, with reasons.

| Section | What it covers |
|---|---|
| [Why SDFs over meshes](00-sdf-primitive/00-why-sdf.md) | The structural argument: SDFs are topology-agnostic, CSG-closed, differentiable by construction; meshes are topology-rigid, require manual surgery for composition, and break differentiability at boolean operations |
| [SDF operations — CSG, smooth blends, displacement](00-sdf-primitive/01-operations.md) | The operator algebra: union as $\min$, intersection as $\max$, difference as $\max(-\cdot, \cdot)$; smoothed variants via log-sum-exp; displacement via additive perturbation; all operations are differentiable w.r.t. their operands |
| [Integration with cf-design](00-sdf-primitive/02-cf-design.md) | `cf-design` is the authoring layer: designer edits primitive parameters, composition tree, material-field parameters; `sim-soft::sdf_bridge` consumes the resolved SDF+material field. The boundary is an in-memory `SdfField` + `MaterialField` pair, not a file format |

Four claims Ch 00 rests on.

## 1. SDF is the right abstraction because geometry, material, and differentiability all want the same thing

The argument for SDFs in `sim-soft` is not a taste argument (*SDFs are cooler than meshes*); it is that three otherwise-independent concerns each converge on the SDF abstraction.

**Geometry composition.** The canonical problem mixes primitives — cylindrical cavity, probe, mount, optional layered sleeves. Composing these as meshes requires robust boolean operations on polygon meshes, which is a famously hard problem (self-intersections, T-junctions, numerical-epsilon flips) that mesh libraries like libigl, CGAL, and OpenMesh get right most of the time but not all of the time. Composing them as SDFs requires evaluating $\min$, $\max$, or weighted blends of scalar fields at each query point — three lines of arithmetic, robust by construction, no topology surgery. A designer who wants to subtract a spherical cavity from a cylindrical shell writes $\phi = \max(\phi_\text{shell}, -\phi_\text{sphere})$ and is done. Mesh boolean composition with the same result would require a CGAL corefine-and-compute-difference call plus manual fix-up for the rim band where the sphere grazes the cylinder — sometimes, and sometimes not.

**Material fields.** The thesis commits to [multi-material meshes with spatial material fields](../10-physical/03-thesis.md) — stiffness, fiber direction, viscoelastic relaxation time all varying across the body. "Vary across the body" means "scalar-valued function of position," which *is* the SDF formulation applied to material parameters rather than to geometry. A designer who wants a carbon-black-loaded stiff skin over a softer silicone core writes a radial SDF for the skin boundary and a stiffness field that interpolates between two values across that boundary. The same machinery that stores and composes geometric SDFs stores and composes material SDFs. Having two abstractions for "scalar field over the body" would be a failure of architectural economy; `sim-soft` has one.

**Differentiability.** [Part 6 Ch 05's differentiable-meshing open problem](../60-differentiability/05-diff-meshing.md) names the core obstruction: meshing operations (marching cubes, Delaunay, fTetWild) are piecewise-constant in their output topology and therefore not differentiable through the topology change. SDFs sidestep the topology question at the *design* level — the designer's edit is a scalar-field perturbation, which *is* differentiable. The non-differentiable step is confined to one well-defined boundary (SDF → tet mesh, [Ch 01](01-tet-strategies.md)), and the FD-wrapper pattern Part 6 Ch 05 commits to is applied exactly there. If the design surface were mesh-valued, differentiability would bleed into every vertex-move and every topology edit, and the FD wrapper would have nothing clean to wrap. The SDF abstraction is what makes the non-differentiable step localized and therefore containable.

Three concerns, one abstraction. That is the argument.

## 2. CSG plus smooth blends gives the whole operator algebra

`sim-soft`'s SDF representation supports a closed operator algebra sufficient for every geometric composition the canonical problem and its extensions need. Concretely, eight operators, all cheap, all differentiable away from operator corners.

```rust
pub trait Sdf {
    fn eval(&self, p: Vec3) -> f64;
    fn grad(&self, p: Vec3) -> Vec3;    // autodiff or analytic
}

// Composition operators — all preserve the Sdf trait
pub fn union(a: impl Sdf, b: impl Sdf) -> impl Sdf;
pub fn intersection(a: impl Sdf, b: impl Sdf) -> impl Sdf;
pub fn difference(a: impl Sdf, b: impl Sdf) -> impl Sdf;

// Smoothed variants — parameterized blend radius k
pub fn smooth_union(a: impl Sdf, b: impl Sdf, k: f64) -> impl Sdf;
pub fn smooth_intersection(a: impl Sdf, b: impl Sdf, k: f64) -> impl Sdf;
pub fn smooth_difference(a: impl Sdf, b: impl Sdf, k: f64) -> impl Sdf;

// Spatial transforms
pub fn translate(a: impl Sdf, t: Vec3) -> impl Sdf;
pub fn rotate(a: impl Sdf, r: UnitQuaternion<f64>) -> impl Sdf;
pub fn scale(a: impl Sdf, s: f64) -> impl Sdf;

// Displacement — add a scalar perturbation field
pub fn displace(a: impl Sdf, d: impl Fn(Vec3) -> f64) -> impl Sdf;
```

The sharp operators $\min / \max / \max(-\cdot, \cdot)$ are $C^0$ at their own boundaries — the union of two spheres has a creased equator. Smoothed variants use the log-sum-exp trick:

$$ \phi_\cup^k(p) = -k\, \log\!\left(e^{-\phi_a(p)/k} + e^{-\phi_b(p)/k}\right) $$

which converges to $\min(\phi_a, \phi_b)$ as $k \to 0^+$ and is $C^\infty$ for $k > 0$. The blend radius $k$ is a first-class design parameter — the designer controls it, `cf-design` exposes it, `sim-soft` differentiates through it. Rounded-corner geometry is a design dimension, not a post-hoc fairing step, and the differentiability of the blend radius is what lets the optimizer move a corner from sharp to rounded and back.

This operator algebra is closed under SDF composition: feed in `Sdf` trait objects, get out `Sdf` trait objects. No intermediate mesh, no conversion, no accumulated numerical-precision loss. Designs can compose dozens of operators deep without topology failures.

## 3. The alternatives are both structurally wrong

Two alternatives dominate the existing design-tool landscape, and `sim-soft` rejects both for specific reasons.

**Mesh-first design (Blender, Maya, polygon modeling).** The designer authors a mesh directly — vertex positions, edge topology, face assignments. This is the native format for most 3D content pipelines and is well-tooled. It is wrong for `sim-soft` because (a) the mesh composition operators are not robust, so edits that mix primitives can fail; (b) the material-field story is bolted-on (vertex-attribute painting) and does not compose with the geometry; (c) differentiability through mesh edits is the exact problem Part 6 Ch 05 names as open. Mesh-first is a good fit for content creation where the final mesh *is* the artifact; it is a bad fit for generative design where the mesh is a transient intermediate between an SDF spec and a simulation.

**CAD B-rep (STEP, IGES, Onshape/OpenCascade kernel).** The designer authors a boundary representation — parametric surfaces (planes, cylinders, NURBS patches) stitched along trimmed-edge boundaries with explicit topology. This is the native format for mechanical CAD and is the right answer for manufacturing-ready parts: dimensioned, toleranced, manufacturable by known processes. It is wrong for `sim-soft` at the *exploratory-design* stage because (a) B-rep composition goes through heavy CAD kernels (OpenCascade, Parasolid) that add enormous dependency surface for marginal design-exploration gain; (b) converting B-rep to a simulation-ready tet mesh is a two-step process (B-rep → surface mesh → tet mesh) that loses the parametric connection to the original design; (c) smooth-blend parameters like the $k$ in §2 have no clean B-rep analog — you have to commit to specific fillet radii at specific edges, which is a manufacturing-artifact decision, not a design-exploration degree of freedom.

Nothing stops a `sim-soft` user from *exporting* an SDF to a mesh and then to a STEP file for manufacturing; that pipeline exists and is not re-built here. What the canonical problem and the `cf-design` integration commit to is that the *design-exploration* loop operates on SDFs, and the mesh / B-rep appears only at the boundaries (to `sim-soft` as a tet mesh, to downstream manufacturing as an STL or STEP).

## 4. The cf-design ↔ sim-soft boundary is an SdfField + MaterialField pair

`cf-design` owns authoring: the designer edits primitive parameters (probe radius, cavity length, wall thickness), composes them via the operator algebra from §2, and paints material fields over the composed geometry. `sim-soft`'s [`sdf_bridge/` module](../110-crate/00-module-layout/08-sdf-bridge.md) consumes the resolved output — two in-memory objects:

```rust
pub struct SdfField {
    pub eval: Box<dyn Sdf>,             // the composed geometric SDF
    pub bbox: Aabb3,                     // for meshing budget
    pub resolution_hint: f64,           // target tet edge length
}

pub struct MaterialField {
    pub stiffness: Box<dyn Field<Output = f64>>,    // Shore-A or Young's modulus
    pub density: Box<dyn Field<Output = f64>>,
    pub fiber_dir: Option<Box<dyn Field<Output = Vec3>>>,
    pub prony_terms: Vec<Box<dyn Field<Output = f64>>>,
    pub thermal_conductivity: Option<Box<dyn Field<Output = f64>>>,
    pub layer_id: Option<Box<dyn Field<Output = u8>>>,                      // multi-material layer ID — see [Part 9 Ch 00](../90-visual/00-sss.md)
    pub diffusion_profile: Option<Box<dyn Field<Output = DiffusionProfile>>>, // SSS — see [Part 9 Ch 00](../90-visual/00-sss.md); renderer-only, no solver consumers
}
```

The handshake is in-memory because the design loop runs at interactive rates (design-mode: one re-solve per parameter edit, target ≤50 ms on the parameter-only hot path; [Ch 04](04-live-remesh.md) breaks down the cost regimes), and a file-format round-trip would blow the budget. Serialization to disk exists for persistence but is not on the hot path of the interactive loop.

The `Field<Output = T>` trait lets the material fields be any composition the SDF algebra supports — constant, primitive-based, SDF-distance-interpolated, sampled from volume data. The trait is defined in [Part 2 Ch 09](../20-materials/09-spatial-fields.md) and shared across material and SDF abstractions, which is the one-abstraction-for-scalar-fields payoff from §1.

Two of the fields — `layer_id` and `diffusion_profile` — are renderer-facing and do not feed the FEM solver. They are introduced in [Part 9 Ch 00](../90-visual/00-sss.md); `layer_id` partitions multi-material bodies for per-pixel SSS composition, and `diffusion_profile` stores the per-channel scattering-coefficient-vs-distance curve. Both are optional; single-material bodies and physics-only runs leave them `None` and pay no cost for their presence in the struct.

## What this commits downstream

- [Ch 01 (tet strategies)](01-tet-strategies.md) operates on the `SdfField` + `bbox` + `resolution_hint` triple, not on a mesh input. The chosen tet generator (fTetWild by default) queries the SDF at its own sample points.
- [Ch 02 (material assignment)](02-material-assignment.md) samples the `MaterialField` per-tet at the centroid or via Gauss-point quadrature; no mesh-attached material state is assumed coming in.
- [Ch 04 (live re-mesh)](04-live-remesh.md) detects design edits by hashing the `SdfField` + `MaterialField` pair and re-runs [Ch 01](01-tet-strategies.md) when the hash changes. Topology-preserving edits (parameter tweaks within the same composition tree) are warm-started.
- [Part 2 Ch 09 (spatial material fields)](../20-materials/09-spatial-fields.md) and [Ch 02](02-material-assignment.md) both sit behind the same `Field` trait; a material defined by `cf-design` composes with a material defined by `sim-soft::material` without conversion.
- [Part 6 Ch 05 (diff-meshing open problem)](../60-differentiability/05-diff-meshing.md)'s FD wrapper applies at the `sdf_bridge/` boundary specifically. Gradients w.r.t. SDF parameters flow through the `cf-design` authoring layer (differentiable by construction); gradients w.r.t. geometry *as it appears in the tet mesh* go through the FD wrapper. The clean localization is what this chapter is defending.

## What this does NOT commit

- **SDF data structure.** This chapter commits to SDFs as the design primitive, not to a specific representation (grid-sampled volume, procedural tree, neural SDF, sparse voxel hash). `cf-design` may use any; `sim-soft` queries via the `Sdf::eval` trait method and is agnostic. The trade-offs among representations are in [`cf-design`'s own documentation](../110-crate/02-coupling/04-cf-design.md), not here.
- **Surface extraction algorithm.** Whether the tet generator uses marching cubes, dual contouring, or direct SDF-conforming tetrahedralization is [Ch 01](01-tet-strategies.md)'s decision; Ch 00 just commits that the input is an SDF.
- **Manufacturing export.** STL/STEP/manufacturing-output pipelines exist but are outside `sim-soft`'s scope. `cf-design` handles export; `sim-soft` handles simulation.
