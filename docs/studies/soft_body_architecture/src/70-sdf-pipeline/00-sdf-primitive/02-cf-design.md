# Integration with cf-design

The [Ch 00 parent's Claim 4](../00-sdf-primitive.md) commits the `cf-design` ↔ `sim-soft` boundary to an in-memory `SdfField` + `MaterialField` pair. This leaf writes out who owns what on each side of the boundary, names what crosses it and what does not, and clarifies the one coordination point — content hashing — that [Ch 04 live re-mesh](../04-live-remesh.md) consumes.

## Authoring vs consuming, line by line

`cf-design` owns authoring. Concretely: primitive parameter state (probe radius, cavity length, wall thickness, layer positions), composition tree structure (which primitives participate, which operators connect them, blend radii and other operator parameters), material-field composition (per-parameter `Field<Output = T>` construction from primitives and operators), UI or programmatic surface for editing, persistence format for designs, the export pipeline to STL / STEP / manufacturing outputs.

`sim-soft::sdf_bridge/` owns consumption. Concretely: receiving the resolved `SdfField` and `MaterialField` at the boundary, computing content hashes for [Ch 04's change detection](../04-live-remesh/00-change-detection.md), dispatching to [Ch 01's meshing pipelines](../01-tet-strategies.md) on topology changes, dispatching to [Ch 02's material-assignment pass](../02-material-assignment.md) on material-only or full-re-mesh changes, reporting the resulting `EditResult` back across the boundary.

The two sides communicate through three types at the API boundary: `SdfField`, `MaterialField`, `EditResult`. Nothing else crosses. In particular: `cf-design`'s composition tree is not exposed to `sim-soft`, `sim-soft`'s tet mesh is not exposed to `cf-design`, and neither side sees the other's internal state.

## What crosses the boundary

```rust
// cf-design → sim-soft
pub struct SdfField {
    pub eval: Box<dyn Sdf>,
    pub bbox: Aabb3,
    pub resolution_hint: f64,
}

pub struct MaterialField {
    pub stiffness: Box<dyn Field<Output = f64>>,
    pub density: Box<dyn Field<Output = f64>>,
    pub fiber_dir: Option<Box<dyn Field<Output = Vec3>>>,
    pub prony_terms: Vec<Box<dyn Field<Output = f64>>>,
    pub thermal_conductivity: Option<Box<dyn Field<Output = f64>>>,
}

// sim-soft → cf-design
pub enum EditResult {
    Warm { iters: u32, wall_ms: f64 },
    Cold { iters: u32, wall_ms: f64 },
}
```

The `Box<dyn Sdf>` and `Box<dyn Field<Output = T>>` trait objects are the opacity mechanism. `cf-design` resolves its composition tree into a single `Sdf` trait object whose `eval` method evaluates the composed expression and whose `grad` method evaluates the composed derivative (either analytically per [§01](01-operations.md) or via `cf-design`'s autodiff). `sim-soft` calls `eval` and `grad` at its own sample points and is agnostic to how the composition is internally represented — whether `cf-design` uses a procedural tree, a grid-sampled volume, a neural SDF, or a sparse-voxel hash, `sim-soft` sees only the trait surface.

This is the content-hash precondition [Ch 04](../04-live-remesh/00-change-detection.md) relies on. Because `sim-soft` is agnostic to the composition tree, it cannot walk the tree to detect changes — instead, it hashes the `Sdf`'s output at a fixed sparse sample set and compares hashes frame-to-frame. The opacity is what keeps the boundary thin; the content-hashing is what makes the thin boundary workable for change detection.

## What does not cross the boundary

- **`cf-design`'s composition tree.** Internal to `cf-design`; [§01 operations](01-operations.md) defines the operator algebra but `cf-design` may represent the tree as a typed Rust AST, a serialized JSON graph, a node-editor scene graph, or anything else.
- **`sim-soft`'s tet mesh.** Produced by [Ch 01](../01-tet-strategies.md) downstream of the boundary; `cf-design` does not see or manipulate it.
- **Intermediate solver state.** The per-step Newton iterates, the [factor-on-tape Cholesky](../../50-time-integration/00-backward-euler.md), the [contact broadphase](../../40-contact/03-self-contact/00-bvh.md), the per-tet material parameters: all internal to `sim-soft`.
- **Time-series state.** `cf-design` edits geometry and materials; the designer does not directly edit positions, velocities, or stresses. Those are dynamics, owned by `sim-soft`.
- **Optimization machinery.** `cf-design` produces designs; [Part 10's optimizer](../../100-optimization/00-forward.md) sweeps designs; `sim-soft` evaluates designs. Each boundary is clean; `cf-design` does not know about the optimizer, and the optimizer does not know about `cf-design`'s internal tree.

## Why in-memory and not a file format

The design loop runs at interactive rates — [Ch 04](../04-live-remesh.md)'s parameter-only hot path targets a **≤50 ms end-to-end budget** per edit. A file-format round-trip (serialize the design, write to disk, read from disk, deserialize, rebuild trait objects) would blow that budget on the serialization step alone. The boundary is in-memory because interactive latency demands it.

Persistence exists (the designer saves designs, the optimizer logs them, regression tests snapshot them), but persistence goes through `cf-design`'s own serialization; `sim-soft`'s `sdf_bridge/` never touches the on-disk format. From `sim-soft`'s perspective, every design is an in-memory `SdfField` + `MaterialField` produced freshly by `cf-design`.

## What this does not commit

- **The specific `Field<Output = T>` trait definition.** The trait is shared with [Part 2 Ch 09 spatial material fields](../../20-materials/09-spatial-fields.md); its exact method signature and default-impl set lives there. This sub-leaf commits only that the boundary carries trait objects, not the trait's full definition.
- **The `SdfField` internal representation.** A grid-sampled volume, procedural tree, neural SDF, or sparse voxel hash: any of these is admissible behind the `Sdf` trait. The trade-offs are [`cf-design`'s concern](../../110-crate/02-coupling/04-cf-design.md), not `sim-soft`'s.
- **The change-detection hashing algorithm.** [Ch 04 §00](../04-live-remesh/00-change-detection.md) names the probe points and the hash function; this sub-leaf commits only that hashing is the mechanism and `cf-design`'s opacity is what motivates it.
- **The `cf-design` UI.** Sliders, node graph, Python bindings, WYSIWYG tool, programmatic API only — any is admissible. `sim-soft`'s only contract is the `SdfField` + `MaterialField` pair at the boundary.

## What this sub-leaf commits the book to

- **Three types cross the boundary: `SdfField`, `MaterialField`, `EditResult`.** Nothing else. `cf-design`'s composition tree and `sim-soft`'s tet mesh are invisible across the boundary.
- **The boundary uses trait objects for opacity.** `Box<dyn Sdf>` and `Box<dyn Field<Output = T>>` let `cf-design` choose its internal representation without `sim-soft` knowing.
- **Change detection is content-hash-based.** Because `sim-soft` cannot see the composition tree, it hashes the `Sdf`'s output at fixed probe points. [Ch 04 §00](../04-live-remesh/00-change-detection.md) specifies the probe-point policy and hash function.
- **The boundary is in-memory.** Interactive-rate latency ([Ch 04](../04-live-remesh.md)'s ≤50 ms budget) prohibits a file-format round-trip on the hot path. Persistence is owned by `cf-design` and does not touch `sim-soft`.
