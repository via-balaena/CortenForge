# Future Work 16 — Phase 3D: Mesh + Plugin Infrastructure (Items #65–66)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

Infrastructure items for mesh processing and extensibility.

---

### 65. Mesh Convex Hull Auto-Computation
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State

No convex hull auto-computation for mesh geoms. Users must pre-process meshes or
use triangle mesh collision (slower). MuJoCo automatically computes convex hulls
from mesh geometry for fast collision detection.

#### Objective

Automatically compute convex hulls from mesh geometry when needed for collision
detection.

#### Specification

1. **Algorithm**: Quickhull (or equivalent) convex hull computation from mesh
   vertex data. Standard algorithm with O(n log n) average case.
2. **When to compute**: During model building, for mesh geoms used in collision
   (not visual-only). Store the convex hull alongside the original mesh.
3. **Storage**: Add `mesh_convex_hull: Option<ConvexHull>` to mesh data. The
   convex hull contains vertices and face normals.
4. **Collision integration**: Use the convex hull for GJK/EPA collision detection
   instead of per-triangle testing. Fall back to triangle mesh for non-convex
   collision pairs (if needed).
5. **MuJoCo behavior**: MuJoCo always computes convex hulls for mesh geoms used
   in collision. The `<mesh>` element does not have an opt-out flag.

#### Acceptance Criteria

1. A complex mesh (100+ triangles) gets a convex hull with fewer vertices.
2. GJK collision using convex hull is faster than triangle mesh for convex-ish
   meshes.
3. Collision results are physically correct (conservative — convex hull
   contains the original mesh).
4. Non-mesh geoms unaffected (regression).

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — convex hull computation, collision
  integration
- Possibly a new utility in the mesh crate family

---

### 66. `<plugin>` / `<extension>` Support
**Status:** Not started | **Effort:** XL | **Prerequisites:** None

#### Current State

No plugin infrastructure exists. No mechanism for custom physics behaviors.

#### Objective

Design and implement a plugin system that allows users to extend the physics
pipeline with custom behaviors.

#### Specification

1. **Plugin types** (MuJoCo plugin API taxonomy):
   - **Actuator plugins**: custom force generation (replaces gain/bias formulas).
   - **Sensor plugins**: custom sensor computation.
   - **Passive plugins**: custom passive forces (e.g., drag, elasticity).
   - **SDF plugins**: custom signed distance field collision geometry.
2. **Rust trait-based design**:
   ```rust
   pub trait ActuatorPlugin: Send + Sync {
       fn compute(&self, model: &Model, data: &Data, id: usize) -> f64;
   }
   pub trait SensorPlugin: Send + Sync {
       fn compute(&self, model: &Model, data: &Data, id: usize, out: &mut [f64]);
   }
   ```
3. **Registration**: Plugins registered at model build time via a plugin registry.
   MJCF `<plugin>` elements reference plugins by name.
4. **Pipeline integration**: At each relevant pipeline stage, check for registered
   plugins and call them. Plugins receive read-only `Model` + `Data` references
   and return computed values.
5. **MJCF parsing**: Parse `<extension>` (plugin declaration) and `<plugin>`
   (plugin instance on an element). Store plugin configuration data.

#### Acceptance Criteria

1. A custom actuator plugin can replace the gain/bias formula for an actuator.
2. A custom sensor plugin can compute an arbitrary sensor reading.
3. Plugin registration and MJCF parsing work end-to-end.
4. Models without plugins are unaffected (regression).
5. Plugin trait is `Send + Sync` for compatibility with batched simulation.

#### Files

- `sim/L0/core/src/plugin.rs` — new module for plugin traits and registry
- `sim/L0/mjcf/src/model_builder.rs` — parse plugin MJCF elements
- `sim/L0/core/src/mujoco_pipeline.rs` — plugin dispatch at pipeline stages
