# Future Work 11 — Phase 3A: Cleanup + Conformance Test Suite (Items #43–45)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

Items #43–#44 are the final correctness gaps before the conformance suite. Item #45
is the conformance test suite that validates all preceding correctness work (#19–#44).

---

### 43. Mesh `inertia` Attribute (Shell Inertia + Inertia Mode Enum)
**Status:** Not started | **Effort:** S–M | **Prerequisites:** None

#### Current State

All mesh geoms compute inertia via the signed tetrahedron decomposition (Mirtich 1996)
in `compute_mesh_inertia()` (`builder/mod.rs`). This treats the mesh as a
solid volume. The `exactmeshinertia` compiler attribute is parsed but effectively
no-op — CortenForge always uses the exact algorithm.

No `shellinertia` or `inertia` attribute is parsed on either `<geom>` or `<mesh>`.
Thin-walled objects (cups, shells, tubes) get incorrect inertia tensors.

#### MuJoCo Reference (v3.2.5+, targeting 3.4.0)

MuJoCo 3.2.5 refactored shell inertia from `<geom shellinertia="true">` (deprecated)
to `<mesh inertia="...">` with a 4-mode enum (`mjtMeshInertia` in `mjspec.h`):

| Mode | Constant | Behavior |
|------|----------|----------|
| `convex` | `mjMESH_INERTIA_CONVEX` | Compute volume/inertia from the convex hull (recommended for non-watertight meshes) |
| `exact` | `mjMESH_INERTIA_EXACT` | Signed tetrahedron decomposition; requires watertight oriented mesh |
| `legacy` | `mjMESH_INERTIA_LEGACY` | Legacy algorithm; overcounts volume for non-convex meshes |
| `shell` | `mjMESH_INERTIA_SHELL` | Mass distributed on surface (area-weighted); density = mass/area |

Default: `exact` (MuJoCo 3.4.0). The `<compiler exactmeshinertia>` attribute is
now deprecated in favor of the `<mesh inertia>` attribute.

**Shell inertia algorithm** (from `user_mesh.cc`, `ComputeInertia`):

For each triangle face `(A, B, C)`:
1. Compute area: `area = 0.5 * |(B-A) × (C-A)|`
2. Use `area` as the triangle's mass contribution (instead of signed volume)
3. Compute centroid: `center = (A + B + C) / 3`
4. Accumulate second-moment products with divisor `C = 12` (vs `C = 20` for solid):
   ```
   For coordinate pairs (xx, yy, zz, xy, xz, yz):
   P[j] += (area / 12) * Σ vertex_products(A, B, C)
   ```
5. Center of mass: area-weighted centroid of all triangles
6. Convert to principal moments via parallel axis theorem:
   ```
   Ixx = P_yy + P_zz,  Iyy = P_xx + P_zz,  Izz = P_xx + P_yy
   ```

**`geom/shellinertia`**: Still accepted for backward compatibility but deprecated.
For primitive geoms (sphere, box, capsule), it still computes analytical hollow-shell
inertia. For mesh geoms, `<mesh inertia="shell">` takes precedence.

#### Objective

Add the `inertia` attribute to `<mesh>` with the 4-mode enum, implementing the shell
inertia algorithm. Also parse `geom/shellinertia` for backward compatibility.

#### Specification

##### S1. MJCF types

Add to `MjcfMesh`:
```rust
pub inertia: Option<MeshInertia>,  // default: None → exact
```

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MeshInertia {
    /// Convex hull volume/inertia (recommended for non-watertight).
    Convex,
    /// Signed tetrahedron decomposition (requires watertight oriented mesh).
    #[default]
    Exact,
    /// Legacy overcounting algorithm.
    Legacy,
    /// Surface-area weighted (shell/hollow objects).
    Shell,
}
```

Also add to `MjcfGeom`:
```rust
pub shellinertia: Option<bool>,  // deprecated backward compat
```

##### S2. Parsing

Parse `inertia` attribute on `<mesh>` element:
```xml
<mesh name="cup" file="cup.obj" inertia="shell"/>
```

Parse `shellinertia` on `<geom>` element (deprecated, for backward compat):
```xml
<geom type="mesh" mesh="cup" shellinertia="true"/>
```

Resolution order: `<mesh inertia>` takes precedence over `<geom shellinertia>`.
If only `shellinertia="true"` is set, treat as `MeshInertia::Shell`.

##### S3. Shell inertia algorithm

Add `compute_mesh_inertia_shell()` alongside existing `compute_mesh_inertia()`:

```rust
fn compute_mesh_inertia_shell(mesh: &TriangleMeshData) -> (f64, Vector3<f64>, Matrix3<f64>)
```

Returns `(total_area, area_weighted_com, inertia_tensor_at_unit_surface_density)`.

Algorithm per triangle `(A, B, C)`:
1. `area = 0.5 * |(B-A) × (C-A)|`
2. `center = (A + B + C) / 3`
3. Accumulate total area and area-weighted centroid
4. Accumulate second-moment products with divisor 12:
   - For each axis pair, accumulate vertex coordinate products
     (same structure as the solid algorithm but with `area/12` replacing
     `signed_volume/20`)
5. Shift inertia to COM via parallel axis theorem

The returned `total_area` replaces `volume` in the density→mass calculation:
`mass = density * total_area` (surface density interpretation).

##### S4. Mode dispatch in `compute_geom_inertia()`

```rust
let (measure, com, inertia_unit) = match mesh_inertia_mode {
    MeshInertia::Exact => compute_mesh_inertia(mesh),       // existing
    MeshInertia::Shell => compute_mesh_inertia_shell(mesh),  // new
    MeshInertia::Convex => compute_mesh_inertia(&convex_hull(mesh)),  // defer to #65
    MeshInertia::Legacy => compute_mesh_inertia_legacy(mesh),  // low priority
};
```

For this item, implement `Exact` (already done) and `Shell`. `Convex` depends on
#65 (mesh convex hull auto-computation). `Legacy` is low priority. Both can return
a clear error if used before implemented.

##### S5. Primitive geom shell inertia (backward compat)

When `shellinertia="true"` on a primitive geom, use analytical hollow-shell formulas:

| Geom Type | Solid Inertia | Shell Inertia |
|-----------|--------------|---------------|
| Sphere | `2/5 mr²` | `2/3 mr²` |
| Box | `m/12 (b²+c²)` etc. | `m/6 (b²+c²)` etc. (faces as rectangular shells) |
| Capsule | Standard solid | Cylindrical shell + hemispherical shell caps |
| Cylinder | `m/12 (3r²+h²)` etc. | Cylindrical shell (open ends) |
| Ellipsoid | `m/5 (b²+c²)` etc. | `m/3 (b²+c²)` etc. |

These are straightforward closed-form substitutions. MuJoCo computes them
analytically when `shellinertia` is set on primitives.

#### Acceptance Criteria

1. **Default unchanged**: `<mesh>` without `inertia` attribute uses exact solid
   inertia (existing behavior, regression test).

2. **Shell mesh — analytical validation**: `<mesh inertia="shell">` on an
   icosphere mesh (tessellated sphere, radius 1.0, ~320 triangles) produces
   inertia matching the analytical hollow sphere `I = 2/3 mr²` within `1e-3`
   relative error (mesh tessellation introduces small discretization error).

3. **Shell mesh — area-weighted COM**: A mesh hemisphere with `inertia="shell"`
   has its COM on the surface centroid (not at the volumetric center). Verify
   COM differs from the solid case.

4. **Density interpretation**: With `density="1000"` and `inertia="shell"`,
   `mass = 1000 * total_surface_area` (not `1000 * volume`).

5. **Explicit mass override**: `<geom mass="5.0" ...>` uses the explicit mass
   regardless of inertia mode (mass scaling ratio changes, not mass itself).

6. **Backward compat**: `<geom shellinertia="true" type="mesh" mesh="cup"/>`
   produces the same result as `<mesh name="cup" inertia="shell"/>`.

7. **Primitive geom**: `<geom type="sphere" size="1" shellinertia="true"/>`
   produces `I = 2/3 mr²` (hollow sphere). Default (`shellinertia="false"`)
   produces `I = 2/5 mr²` (solid sphere).

8. **MuJoCo conformance**: Compare inertia tensor (6 components) against
   MuJoCo 3.4.0 for a non-trivial mesh (Stanford bunny or similar) with
   `inertia="shell"`. Tolerance: `1e-6` relative.

9. **Default class inheritance**: `shellinertia` inherits from `<default>`
   class on `<geom>`. `inertia` on `<mesh>` is per-mesh (not inheritable via
   default class — meshes don't have default classes in MuJoCo).

10. **Degenerate mesh**: A mesh with zero surface area falls back gracefully
    (AABB-based inertia, matching the existing solid degenerate path).

#### Files

- `sim/L0/mjcf/src/types.rs` — `MeshInertia` enum, add `inertia` to `MjcfMesh`,
  add `shellinertia` to `MjcfGeom`
- `sim/L0/mjcf/src/parser.rs` — parse `inertia` on `<mesh>`, `shellinertia` on `<geom>`
- `sim/L0/mjcf/src/builder/` — `compute_mesh_inertia_shell()`, mode dispatch
  in `compute_geom_inertia()`, primitive shell inertia formulas
- `sim/L0/tests/integration/` — shell inertia tests (new file or extend
  `exactmeshinertia.rs`)

---

### 44. Legacy Crate Deprecation
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

Four standalone crates duplicate functionality that is fully implemented in the
pipeline (`sim-core`):

| Crate | Lines | Pipeline equivalent | Used by pipeline? |
|-------|-------|--------------------|--------------------|
| `sim-constraint` | 10,359 | PGS/CG/Newton in `constraint/` modules | **No** |
| `sim-muscle` | 2,550 | MuJoCo FLV in `mj_fwd_actuation()` | **No** |
| `sim-tendon` | 3,919 | `mj_fwd_tendon_fixed/spatial()` | **No** |
| `sim-sensor` | — | 32 sensor types in `mj_sensor_*()` | **No** |

These crates are re-exported by `sim-physics` (the umbrella crate) but have zero
callers in the pipeline. They confuse contributors ("which PGS?"), inflate
compile times, and create a false impression of coverage.

#### Objective

Deprecate standalone crates that are fully superseded by pipeline implementations.

#### Specification

1. **sim-tendon**: Mark as `#[deprecated]`. Add top-level doc comment directing
   users to pipeline `mj_fwd_tendon_*()`. Remove from default workspace members.
2. **sim-muscle**: Mark as `#[deprecated]`. Note the standalone Hill model is
   richer than pipeline FLV but not MuJoCo-compatible. Keep available for
   biomechanics users who don't need MuJoCo conformance.
3. **sim-constraint**: Keep crate but audit public API. Remove re-exports of
   deleted types (PGSSolver, NewtonSolver, etc.). Document which types are
   standalone-only vs pipeline-compatible.
4. **sim-sensor**: Keep — provides standalone hardware sensor API independent of
   pipeline.

#### Acceptance Criteria

1. Deprecated crates emit compiler warnings on use.
2. `cargo doc` shows clear deprecation notices with migration guidance.
3. No change to pipeline behavior (regression).
4. Workspace compiles cleanly (no dead-code warnings in deprecated crates).

#### Files
- `sim/L0/tendon/src/lib.rs` — deprecation attributes
- `sim/L0/muscle/src/lib.rs` — deprecation attributes
- `sim/L0/constraint/src/lib.rs` — API audit
- `Cargo.toml` (workspace) — optional default-members adjustment

---

### 45. MuJoCo Conformance Test Suite
**Status:** Not started | **Effort:** XL | **Prerequisites:** #19–#44

#### Current State

Testing is ad-hoc: analytical FK/CRBA ground truth (9 tests in `validation.rs`),
finite-difference derivative checks (30+ in `derivatives.rs`), and hardcoded
MuJoCo 3.4.0 reference values for spatial tendons (18 in `spatial_tendons.rs`).
No systematic per-stage comparison, no trajectory-level validation, no
self-consistency or property-based tests.

#### Objective

Build a four-layer conformance test suite that validates CortenForge against
MuJoCo 3.4.0 at every pipeline stage, catches regressions via trajectory
comparison, and enforces physical invariants. The spec will be written fresh
once all 26 prerequisite items (#19–#44) are complete.

#### Specification

1. **Layer A — Self-consistency** (no MuJoCo dependency): forward/inverse
   equivalence, island/monolithic solver equivalence, determinism,
   integrator energy ordering, sparse/dense mass matrix equivalence.
2. **Layer B — Per-stage reference**: compare each pipeline stage output
   (FK, CRBA, RNE, passive, collision, constraint, actuation, sensors,
   tendons, integration) against MuJoCo 3.4.0 hardcoded values.
3. **Layer C — Trajectory comparison**: multi-step trajectory tests with
   per-step per-field diagnostics to isolate which stage first diverges.
4. **Layer D — Property/invariant tests** (no MuJoCo dependency): momentum
   conservation, energy conservation, quaternion normalization, contact
   force feasibility, mass matrix properties.
5. **Reference generation**: Python script pinned to `mujoco==3.4.0`,
   outputs checked into repo. No MuJoCo build dependency for Rust tests.
6. **Detailed spec deferred**: models, tolerances, test models, and
   acceptance criteria will be written against the post-fix codebase when
   all prerequisites land. Previous detailed spec was removed — it was
   written when only 14 of the current 26 gaps were known.

#### Acceptance Criteria

1. Self-consistency tests (Layer A) pass without MuJoCo installed.
2. Per-stage reference tests (Layer B) pass at algorithm-appropriate tolerances.
3. Trajectory tests (Layer C) pass for contact and non-contact scenarios.
4. Property tests (Layer D) verify conservation laws and invariants.
5. `cargo test -p sim-conformance-tests` works without MuJoCo installed.
6. No existing tests in `integration/` broken or moved.

#### Files

- `sim/L0/tests/conformance/` — test suite directory
- `sim/L0/tests/conformance/generate_references.py` — Python reference generator
- `sim/L0/tests/integration/` — unchanged (existing tests stay)
