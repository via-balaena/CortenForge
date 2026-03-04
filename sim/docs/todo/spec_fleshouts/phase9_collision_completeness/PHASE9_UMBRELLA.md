# Phase 9 — Collision Completeness: Umbrella Spec

**Status:** Draft
**Phase:** Roadmap Phase 9
**Tasks:** 6 (§43, §50, §54, §57, §65, DT-70)
**Deliverables:** 1 T1 session + 5 sub-specs (Spec A, B, C, D, E)
**Test baseline:** 1,900+ domain tests (post-Phase 8)

---

## Scope

Phase 9 closes all collision pipeline completeness gaps in the v1.0 roadmap.
**The goal is MuJoCo conformance** — after Phase 9, CortenForge computes
convex hulls from mesh geometry at build time, computes mesh inertia in all
four MuJoCo modes, detects heightfield-mesh/plane/hfield contacts, prevents
tunneling via conservative-advancement CCD, resolves deformable-vs-complex-geom
contacts, and wires SDF solver parameters from `<option>`.

Phase 9 tasks span two pipeline stages:
- **Build-time mesh processing:** §65 (convex hull computation), §43 (mesh
  inertia modes) — these run during model compilation, before any simulation
  step.
- **Runtime collision:** §50 (CCD), §54 (heightfield pairs), DT-70
  (deformable-vs-complex-geom), §57 (SDF option wiring) — these execute
  during the position-stage collision pass in `forward()`.

Unlike Phase 8 (single-subsystem constraint pipeline), Phase 9 has broader
scope — five sub-specs plus one T1 item — but the tasks converge on the
collision pipeline in `sim-core`. The key structural feature is the
**hard dependency** from Spec B (mesh inertia) to Spec A (convex hull):
the `Convex` inertia mode delegates to Spec A's convex hull computation.

> **Conformance mandate for sub-spec authors:** Each sub-spec must cite the
> exact MuJoCo C source (function, file, line range) for every behavior it
> implements. Acceptance criteria must include expected values derived from
> MuJoCo's output — not from hand-calculation or intuition. The MuJoCo C
> source code is the single source of truth. When the MuJoCo docs and the
> C source disagree, the C source wins.

---

## Task Assignment

Every Phase 9 task is assigned to exactly one deliverable. No orphans, no
overlaps. Each task closes a specific MuJoCo conformance gap.

| Task | Description | Deliverable | Status | Rationale |
|------|-------------|-------------|--------|-----------|
| §57 | `sdf_iterations`/`sdf_initpoints` from `<option>` — wire to SDF solver | **T1** | Pending | Pure plumbing: parse 2 int attributes, thread to existing SDF functions |
| §65 | Mesh convex hull auto-computation (Quickhull) | **Spec A** | Pending | Foundational: convex hull is consumed by Spec B (inertia) and collision dispatch |
| §43 | Mesh inertia modes (exact/shell/convex/legacy) | **Spec B** | Pending | Depends on Spec A for `Convex` mode |
| §54 | Missing heightfield collision pairs (hfield-mesh, hfield-plane, hfield-hfield) | **Spec C** | Pending | Independent runtime collision pairs |
| §50 | Continuous Collision Detection (conservative advancement) | **Spec D** | Pending | Largest algorithmic task; requires GJK distance query extension |
| DT-70 | Deformable-vs-mesh/hfield/SDF narrowphase | **Spec E** | Pending | Three new flex collision pair types |

### Sub-spec scope statements

Each sub-spec must identify the exact MuJoCo C functions it is porting, cite
them with source file and line ranges, and produce acceptance criteria with
MuJoCo-verified expected values.

**Spec A — Mesh Convex Hull Auto-Computation** (§65):
Build-time convex hull computation from mesh vertices using Quickhull (or
equivalent incremental algorithm). MuJoCo always computes convex hulls for
mesh geoms used in collision — the hull is stored alongside the original mesh
and used by the GJK/EPA narrowphase for mesh-vs-* collision pairs.

MuJoCo reference:
- `user_mesh.cc` → `mjCMesh::MakeConvex()` — Quickhull construction from
  mesh vertex set. Computes convex hull vertices and face normals.
- `user_mesh.cc` → `mjCMesh::CopyVert()` / `mjCMesh::CopyFace()` — hull
  vertex/face storage format.
- `mjmodel.h` → `mesh_convexhull` — per-mesh boolean controlling whether
  hull is computed (default: true for collision geoms).
- The convex hull is the collision geometry; the original mesh is used for
  rendering and optionally for exact-mode inertia.

Primary changes in `sim/L0/core/src/mesh.rs` (new `ConvexHull` struct +
Quickhull algorithm), `sim/L0/mjcf/src/builder/mesh.rs` (invoke hull
computation during model build), and `sim/L0/core/src/collision_shape.rs`
(use hull for `CollisionShape::ConvexMesh`).

**Spec B — Mesh Inertia Modes** (§43):
Four-mode `MeshInertia` enum on `<mesh>` element: `exact` (signed tetrahedron
decomposition — existing), `shell` (area-weighted surface distribution),
`convex` (exact on convex hull — delegates to Spec A), `legacy` (backward
compat). Also backward-compatible `shellinertia` on `<geom>` for primitive
geom shell formulas.

MuJoCo reference:
- `user_mesh.cc` → `ComputeInertia()` — shell inertia algorithm:
  area-weighted triangle decomposition with divisor 12 (vs 20 for solid).
  `mass = density * total_surface_area`.
- `mjspec.h` → `mjtMeshInertia` enum: `mjMESH_INERTIA_EXACT`,
  `mjMESH_INERTIA_SHELL`, `mjMESH_INERTIA_CONVEX`, `mjMESH_INERTIA_LEGACY`.
- `user_mesh.cc` → `SetGeom()` — shell inertia for primitive geoms (sphere:
  `2/3 mr^2`, box: `m/6 (b^2+c^2)`, etc.).
- Default: `exact`. `shellinertia` on `<geom>` is deprecated but still
  accepted for backward compatibility.

**Depends on Spec A (§65):** The `Convex` inertia mode calls
`compute_mesh_inertia(&convex_hull)` — it runs the exact volumetric algorithm
on Spec A's convex hull output. Spec A must be complete before implementing
the Convex path.

Primary changes in `sim/L0/mjcf/src/types.rs` (`MeshInertia` enum on
`MjcfMesh`, `shellinertia` on `MjcfGeom`), `sim/L0/mjcf/src/builder/mesh.rs`
(shell inertia algorithm, mode dispatch in `compute_geom_inertia()`),
`sim/L0/mjcf/src/builder/body.rs` (primitive shell formulas for
`shellinertia`).

**Spec C — Heightfield Collision Pairs** (§54):
Three missing heightfield collision pairs: hfield-mesh (highest priority),
hfield-plane, and hfield-hfield. Currently heightfield collision supports
sphere, capsule, and box directly; cylinder approximates as capsule, ellipsoid
approximates as sphere.

MuJoCo reference:
- `engine_collision_primitive.c` → heightfield collision dispatch. MuJoCo
  uses `mjCOLLISION_TABLE` to determine supported pairs.
- `engine_collision_primitive.c` → `mjc_PlaneHField()` — plane-heightfield
  contact generation.
- `engine_collision_primitive.c` → `mjc_MeshHField()` — mesh-heightfield
  triangle-cell intersection.
- `engine_collision_primitive.c` → `mjc_HFieldHField()` — dual-grid
  heightfield intersection (if supported — verify in source).

Primary changes in `sim/L0/core/src/collision/hfield.rs` (new collision pair
functions), `sim/L0/core/src/collision/narrow.rs` (dispatch routing for new
pairs), `sim/L0/core/src/heightfield.rs` (grid traversal utilities for new
pair types).

**Spec D — Continuous Collision Detection** (§50):
Conservative-advancement CCD for convex geom pairs. This is the largest
algorithmic task in Phase 9.

**Hidden prerequisite:** The existing GJK in `gjk_epa.rs` is
intersection-only — it returns `bool` + EPA penetration depth but has NO
separating distance query for non-overlapping shapes. Conservative
advancement requires the minimum separation distance at each iteration.
Spec D must first extend `gjk_epa.rs` with a `gjk_distance()` function
(or equivalent closest-point query) before implementing CCD.

MuJoCo reference:
- `engine_collision_driver.c` → CCD dispatch after broadphase. Conservative
  advancement: iterative time-of-impact estimation using upper-bound velocity
  and GJK minimum separation distance.
- `mjOption` → `ccd_iterations` (default 50, already parsed + stored in
  CortenForge as `MjcfOption.ccd_iterations`).
- `mjOption` → `ccd_tolerance` — distance threshold for CCD contact. **NOT
  currently parsed** in CortenForge (only exists in dm_control schema). Must
  be parsed as part of this spec or explicitly deferred.
- `mjOption` → `nativeccd` / `multiccd` enable flags — already wired as
  no-op guards with `tracing::warn!` in `builder/mod.rs`.
- Scope: convex-convex only (sphere, capsule, box, ellipsoid, convex mesh).
  Non-convex pairs use discrete detection.
- Multi-point CCD (`MULTICCD` flag) for flat surfaces.

Primary changes in `sim/L0/core/src/gjk_epa.rs` (GJK distance query),
new `sim/L0/core/src/collision/ccd.rs` (conservative advancement algorithm),
`sim/L0/core/src/collision/mod.rs` (CCD integration into `mj_collision()`
pipeline — the broadphase + narrowphase entry point).

**Spec E — Deformable Complex-Geom Narrowphase** (DT-70):
Three new flex collision pair types: flex-vs-mesh, flex-vs-hfield,
flex-vs-SDF. Currently `flex_collide.rs` only supports primitive geom types
(plane, sphere, box, capsule, cylinder, ellipsoid).

MuJoCo reference:
- `engine_collision_primitive.c` → flex collision dispatch. Each flex vertex
  is treated as a sphere with radius `flex_radius`; the narrowphase computes
  distance from vertex sphere to the complex geom.
- Flex-vs-mesh: vertex sphere against triangle mesh (or convex hull).
- Flex-vs-hfield: vertex sphere against heightfield grid — may reuse existing
  `collide_with_hfield()` sphere-hfield path.
- Flex-vs-SDF: vertex sphere against SDF — may reuse existing
  `sdf_sphere_contact()`.

**Soft dependency on Spec A (§65):** Flex-vs-mesh *may* use Spec A's convex
hull for vertex-sphere-vs-convex-mesh queries. If Spec A is not yet complete,
the spec should define a brute-force per-triangle fallback.

Primary changes in `sim/L0/core/src/collision/flex_collide.rs` (extend
`narrowphase_sphere_geom()` dispatch with Mesh/Hfield/Sdf arms).

**T1 — SDF Option Wiring** (§57):
Parse `sdf_iterations` (int, default 10) and `sdf_initpoints` (int, default
40) from `<option>`. Thread `sdf_iterations` to `closest_surface_point()` in
`sdf.rs` (replacing hardcoded `max_iterations = 10`). Thread `sdf_initpoints`
to the appropriate SDF contact functions — audit each `sdf_*_contact` function
to determine which should use the configurable init point count vs. fixed
geometric sampling (e.g., box corners are always 8).

MuJoCo reference: `mjOption` struct fields `sdf_iterations` (default 10)
and `sdf_initpoints` (default 40) in `mjmodel.h`.

---

## Dependency Graph

```
Session 1 (Umbrella)
    |
    +-- Session 2 (T1: §57 SDF options)                     <- independent
    |
    +-- Sessions 3-7 (Spec A: §65 Convex Hull)              <- independent
    |       |
    |       v (hard dep)
    |   Sessions 8-12 (Spec B: §43 Mesh Inertia)            <- depends on Spec A
    |       :
    |       v (soft dep -- flex-vs-mesh may use convex hull)
    |   Sessions 23-27 (Spec E: DT-70 Deformable NP)        <- can proceed without Spec A
    |
    +-- Sessions 13-17 (Spec C: §54 Hfield Pairs)           <- independent
    |
    +-- Sessions 18-22 (Spec D: §50 CCD)                    <- independent
```

### Dependency edges

| From -> To | Type | Specific dependency |
|-----------|------|---------------------|
| Spec A -> Spec B | **Hard** | §43's `Convex` inertia mode calls `compute_mesh_inertia(&convex_hull)` using Spec A's `ConvexHull` struct. Spec B cannot implement Convex mode without Spec A. |
| Spec A -> Spec E | **Soft** | DT-70 flex-vs-mesh *may* use Spec A's convex hull for vertex-sphere-vs-convex-mesh queries. Fallback: brute-force per-triangle distance. |

All other deliverables are independent — T1, Spec A, Spec C, and Spec D
can proceed in parallel. Spec E can proceed without Spec A (using per-triangle
fallback) but benefits from it.

### Why Spec D is independent of Spec A

CCD operates on convex geom pairs using GJK distance queries on
`CollisionShape` objects. The existing `CollisionShape::ConvexMesh` variant
already exists and represents a convex vertex set. CCD uses the GJK support
function on whatever shape is provided — it does not need to *compute* the
convex hull itself. If Spec A is not yet complete, CCD tests can use
non-mesh convex shapes (sphere, capsule, box, ellipsoid). Mesh-vs-mesh CCD
is a valid later integration but not a prerequisite for the CCD algorithm.

### GJK distance query is a blocking prerequisite for CCD

Spec D's S1 (GJK distance query for non-overlapping convex shapes) must
land BEFORE S2+ (conservative advancement). The existing GJK in `gjk_epa.rs`
is intersection-only (`gjk_intersection()` → `bool`, `gjk_epa_contact()` →
`Option<GjkContact>` for overlapping shapes only). Conservative advancement
requires the *separating distance* between non-overlapping shapes at each
iteration step. This is a hard internal prerequisite within Spec D — the CCD
algorithm cannot be tested or validated without it.

---

## File Ownership Matrix

Files touched by 2+ deliverables, with ownership sequence and handoff state.
Single-owner files are not listed.

### `sim/L0/core/src/mesh.rs` (2,147 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | Spec A (§65) | Add `ConvexHull` struct + Quickhull algorithm; add `convex_hull: Option<ConvexHull>` field to `TriangleMeshData` | Meshes can have convex hulls |
| 2 | Spec B (§43) | Read `convex_hull` from mesh to compute inertia in Convex mode | No additional changes to mesh.rs |

**Conflict risk: None.** Spec A adds the hull; Spec B reads it.

### `sim/L0/core/src/collision_shape.rs` (1,734 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | Spec A (§65) | Modify `CollisionShape::ConvexMesh` construction to use convex hull vertices instead of original mesh vertices | Hull-based collision shapes |
| 2 | Spec D (§50) | Add GJK distance support function on `CollisionShape` (or in gjk_epa.rs using existing support function) | Distance-queryable shapes |

**Conflict risk: Low.** Spec A modifies shape construction; Spec D adds
distance query capability. Different operations on the same type.

### `sim/L0/core/src/gjk_epa.rs` (1,367 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | Spec D (§50) | Add `gjk_distance()` function for non-overlapping convex shapes (separating distance + closest points) | GJK supports both intersection and distance |

**Conflict risk: None.** Single-owner in Phase 9.

### `sim/L0/core/src/collision/narrow.rs` (338 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| any | Spec C (§54) | Add hfield-mesh, hfield-plane, hfield-hfield dispatch arms in `collide_geoms()` | New pair routing |
| any | Spec E (DT-70) | No change expected (flex collision dispatches through `flex_collide.rs`, not `narrow.rs`) | Unchanged |

**Conflict risk: None.** Only Spec C modifies this file.

### `sim/L0/core/src/collision/hfield.rs` (93 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | Spec C (§54) | Add `collide_hfield_mesh()`, `collide_hfield_plane()`, `collide_hfield_hfield()` | Three new collision pair functions |
| 2 | Spec E (DT-70) | May reuse hfield sphere contact for flex-vs-hfield (calls into existing functions, not modifying hfield.rs) | Unchanged |

**Conflict risk: None.** Spec C is the sole modifier.

### `sim/L0/core/src/collision/flex_collide.rs` (221 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | Spec E (DT-70) | Extend `narrowphase_sphere_geom()` with Mesh, Hfield, Sdf match arms | Three new flex collision paths |

**Conflict risk: None.** Single-owner in Phase 9.

### `sim/L0/core/src/heightfield.rs` (941 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | Spec C (§54) | May add grid traversal utilities for new pair types (hfield-mesh cell iteration) | New traversal helpers |
| 2 | Spec E (DT-70) | Read-only consumer — flex-vs-hfield reuses existing `heightfield_sphere_contact()` | Unchanged |

**Conflict risk: None.** Spec C is the sole modifier; Spec E only reads.

### `sim/L0/core/src/collision/mod.rs` (1,065 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| any | Spec D (§50) | Add CCD pass integration (after broadphase, for qualifying pairs) | CCD wired into pipeline |

**Conflict risk: None.** Single-owner in Phase 9.

### `sim/L0/core/src/sdf.rs` (2,793 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | T1 (§57) | Replace hardcoded `max_iterations = 10` with configurable `sdf_iterations`; thread `sdf_initpoints` to appropriate contact functions | SDF solver parameters configurable |

**Conflict risk: None.** Single-owner in Phase 9.

### `sim/L0/mjcf/src/types.rs`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| any | T1 (§57) | Add `sdf_iterations: usize` and `sdf_initpoints: usize` to `MjcfOption` | Two new option fields |
| any | Spec A (§65) | Add `convexhull: Option<bool>` to `MjcfMesh` (if MuJoCo has this attribute) | Convex hull control |
| any | Spec B (§43) | Add `inertia: Option<MeshInertia>` to `MjcfMesh`; add `shellinertia: Option<bool>` to `MjcfGeom` | Inertia mode + backward compat |

**Conflict risk: None.** All additions are new fields on different structs
or independent fields on the same struct.

### `sim/L0/mjcf/src/builder/mesh.rs` (829 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | Spec A (§65) | Invoke convex hull computation during mesh build; store hull on `TriangleMeshData` | Meshes have hulls at build time |
| 2 | Spec B (§43) | Add shell inertia algorithm; add mode dispatch in `compute_geom_inertia()` (or equivalent) | Four inertia modes available |

**Conflict risk: Low.** Spec A adds hull computation (new code section).
Spec B adds inertia mode dispatch (modifies existing inertia computation).
Different code sections within the same file.

### `sim/L0/core/src/types/model.rs` (~1,300 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| any | T1 (§57) | Add `sdf_iterations: usize` and `sdf_initpoints: usize` to Model options (or `MjOption` struct) | Two new option fields |
| any | Spec D (§50) | Add `ccd_tolerance: f64` to Model options (if not deferred) | CCD tolerance field |

**Conflict risk: None.** New independent fields.

### `sim/L0/core/src/forward/position.rs` (601 lines)

Not directly modified by Phase 9. The broadphase (`SweepAndPrune`) runs
in `collision/mod.rs::mj_collision()`, not in `position.rs`. CCD
integrates into `collision/mod.rs` as an additional pass within
`mj_collision()`, after broadphase pair generation and before/after
narrowphase — Spec D decides the exact placement.

---

## API Contracts

Cross-spec API dependencies. Sub-specs must be written against these
contracts, not against the current (pre-Phase 9) codebase.

### Contract 1: `ConvexHull` struct (Spec A -> Spec B, Spec E)

**After Spec A — expected API:**
```rust
/// Convex hull of a triangle mesh, computed via Quickhull.
pub struct ConvexHull {
    /// Hull vertices (subset of or derived from original mesh vertices).
    pub vertices: Vec<Vector3<f64>>,
    /// Hull faces as index triples into `vertices`.
    pub faces: Vec<[usize; 3]>,
    /// Outward face normals (one per face).
    pub normals: Vec<Vector3<f64>>,
}

impl TriangleMeshData {
    /// Returns the convex hull, computing it if not yet cached.
    pub fn convex_hull(&self) -> &ConvexHull;
}
```

**What Spec B needs:** `convex_hull.vertices` and `convex_hull.faces` to
construct a temporary `TriangleMeshData` representing the hull surface, then
call the existing `compute_mesh_inertia()` on it. Alternatively, Spec B can
call `compute_mesh_inertia()` directly on the hull's vertex/face data.

**What Spec E needs:** The `ConvexHull` for flex-vs-mesh collision (optional
optimization). If Spec A is not complete, Spec E falls back to per-triangle
brute-force distance.

### Contract 2: GJK distance query (Spec D internal)

**After Spec D S1 — expected API:**
```rust
/// Result of GJK distance query for non-overlapping convex shapes.
pub struct GjkDistanceResult {
    /// Minimum separating distance (> 0 if non-overlapping).
    pub distance: f64,
    /// Closest point on shape A (in world frame).
    pub closest_a: Vector3<f64>,
    /// Closest point on shape B (in world frame).
    pub closest_b: Vector3<f64>,
}

/// Compute minimum separating distance between two non-overlapping convex shapes.
/// Returns None if shapes are overlapping (use gjk_epa_contact instead).
pub fn gjk_distance(
    shape_a: &CollisionShape,
    pose_a: &Pose,
    shape_b: &CollisionShape,
    pose_b: &Pose,
) -> Option<GjkDistanceResult>;
```

This is internal to Spec D — no other spec depends on it. The function is
used by the conservative advancement inner loop.

### Contract 3: SDF option threading (T1 -> SDF collision)

**After T1 — expected change:**
```rust
// Before (current): hardcoded in sdf.rs
let max_iterations = 10;

// After T1: parameter threaded from model options
fn closest_surface_point(
    sdf: &SdfCollisionData,
    point: Vector3<f64>,
    sdf_iterations: usize,  // from model.opt.sdf_iterations
) -> ...
```

No other deliverable depends on this. The SDF collision functions
(`sdf_collide.rs`) will thread the parameters from the model through to
`sdf.rs`.

---

## Shared Convention Registry

Conventions decided once here. Sub-specs reference this section instead of
inventing their own.

### 1. Collision pair dispatch pattern

New collision pairs follow the existing dispatch pattern in `narrow.rs`.
CortenForge uses if-statement dispatch (not MuJoCo's table-based
`mjCOLLISION_TABLE`):

```rust
// In collide_geoms(): special-case dispatch before analytical pairs
if type1 == GeomType::Hfield || type2 == GeomType::Hfield {
    return collide_with_hfield(model, geom1, geom2, pos1, mat1, pos2, mat2, margin);
}
```

The function signature for new collision pair functions follows the existing
convention. **Critical:** parameters are by-value `Vector3<f64>` and
`Matrix3<f64>` (rotation matrix, NOT quaternion), and the return type is
`Option<Contact>` (single contact or None):

```rust
pub fn collide_with_hfield(
    model: &Model,
    geom1: usize,          // first geom index
    geom2: usize,          // second geom index
    pos1: Vector3<f64>,    // by-value, NOT reference
    mat1: Matrix3<f64>,    // rotation MATRIX, NOT quaternion
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    margin: f64,
) -> Option<Contact>       // single contact or None
```

Some collision functions (e.g., `collide_sphere_sphere` in `pair_convex.rs`)
also take `size1: Vector3<f64>` and `size2: Vector3<f64>` for geom
dimensions. Sub-specs should match the exact parameter set of the nearest
existing function for their pair type.

New pairs that generate *multiple* contacts (e.g., hfield-mesh may produce
contacts at multiple grid cells) should use a `Vec<Contact>` return or
internal accumulation — the sub-spec decides the pattern. The existing
heightfield contact functions in `heightfield.rs` (e.g.,
`heightfield_box_contact`) return `Option<HeightFieldContact>` for a single
contact point per query; `collide_with_hfield` currently dispatches one call
per geom type and converts to `Option<Contact>` via `.map()`. Spec C may
extend this to iterate over multiple grid cells for hfield-mesh pairs.

**Layer distinction:** This `pos: Vector3<f64>` + `mat: Matrix3<f64>`
convention applies to **narrowphase dispatch functions** (in `collision/`).
Lower-level algorithmic functions (GJK/EPA in `gjk_epa.rs`) use `&Pose`
with `UnitQuaternion<f64>` internally — see API Contract §2. These are
intentionally different conventions at different abstraction layers.

### 2. Contact generation signature

All new contact generation code creates `Contact` structs following the
existing pattern in `contact_types.rs`:

```rust
Contact {
    pos: contact_point,       // world-frame contact position
    normal: contact_normal,   // world-frame normal (from geom_a to geom_b)
    depth: penetration_depth, // positive = penetrating
    geom1: geom_a,
    geom2: geom_b,
    // ... solver params from contact_param()
}
```

Normal convention: contact normal points from geom_a toward geom_b (outward
from geom_a's surface). This matches MuJoCo's convention.

### 3. GeomType enum stability

No new `GeomType` variants are added in Phase 9. All existing variants
(Plane, Sphere, Capsule, Cylinder, Box, Ellipsoid, Mesh, Hfield, Sdf) are
sufficient for the Phase 9 work. The collision dispatch table expands to
cover new *pairs* of existing types, not new types.

### 4. New model field naming

| Field | Type | Default | Deliverable |
|-------|------|---------|-------------|
| `sdf_iterations` | `usize` | `10` | T1 (§57) |
| `sdf_initpoints` | `usize` | `40` | T1 (§57) |
| `ccd_tolerance` | `f64` | `1e-6` (verify) | Spec D (§50, if not deferred) |

These fields live on the model options struct (alongside existing
`ccd_iterations: usize`).

### 5. Mesh data storage convention

Spec A's `ConvexHull` is stored on `TriangleMeshData` (not on `Model`
directly). This follows the existing pattern where `TriangleMeshData` owns
all mesh geometry data and is shared via `Arc<TriangleMeshData>` in
`Model.mesh_data`.

```rust
pub struct TriangleMeshData {
    // ... existing fields ...
    /// Convex hull for collision (computed at build time via Quickhull).
    pub convex_hull: Option<ConvexHull>,
}
```

The hull is `Option` because visual-only meshes (not used in collision) may
skip hull computation for performance.

### 6. Build-time vs runtime convention

- **Build-time computations** (Spec A convex hull, Spec B inertia): execute
  during model building in `sim/L0/mjcf/src/builder/`. The results are stored
  on `Model` or its sub-structures (`TriangleMeshData`, etc.) and are
  immutable after build.
- **Runtime computations** (T1 SDF wiring, Spec C hfield pairs, Spec D CCD,
  Spec E flex-vs-complex): execute during `forward()` in
  `sim/L0/core/src/collision/`. They read from `Model` and write to `Data`.

### 7. Contact parameter combination (mechanism-2)

New collision pairs in Phase 9 generate `Contact` structs that flow through
the existing contact parameter combination pipeline in `collision/mod.rs`:
`contact_param()` → `solmix_weight()` → `apply_pair_overrides()` →
`apply_global_override()`. No changes to this pipeline are needed — new
pair types produce contacts with the same structure as existing pairs, and
mechanism-2 `<contact><pair>` overrides apply automatically based on geom
indices. Sub-specs should NOT modify contact parameter combination logic.

---

## Cross-Spec Blast Radius

### Behavioral interactions between specs

| Interaction | Analysis |
|------------|----------|
| Spec A (convex hull) + Spec B (mesh inertia) | **Dependency, not conflict.** Spec B's `Convex` mode consumes Spec A's output. Spec A must land first. No conflicting code changes — Spec A adds hull to mesh data, Spec B reads hull for inertia computation. |
| Spec A (convex hull) + Spec D (CCD) | **No conflict.** CCD operates on `CollisionShape` objects, which already have a `ConvexMesh` variant. Spec A changes how `ConvexMesh` shapes are constructed (from hull vertices instead of original vertices), but this is transparent to CCD — the GJK support function works on whatever vertices are provided. |
| Spec A (convex hull) + Spec E (flex-vs-mesh) | **Soft dependency.** Spec E may use the convex hull for more efficient flex-vs-mesh distance queries. If Spec A is not complete, Spec E uses per-triangle brute-force. No code conflict. |
| Spec C (hfield pairs) + Spec E (flex-vs-hfield) | **Complementary, not dependent.** Spec C adds rigid-rigid hfield-mesh/plane/hfield pairs. Spec E adds flex-vs-hfield pairs. Both modify heightfield collision but in different dispatch paths: Spec C in `narrow.rs`/`hfield.rs`, Spec E in `flex_collide.rs`. The heightfield grid traversal utilities in `heightfield.rs` may be shared but as read-only consumers. |
| Spec D (CCD) + all other specs | **Independent.** CCD is a post-broadphase pass for high-velocity convex pairs. It does not modify narrowphase dispatch, heightfield collision, flex collision, or mesh inertia. It adds a new pipeline stage but does not alter existing stages. |
| T1 (SDF options) + all other specs | **Independent.** T1 modifies SDF solver internals only. No other spec touches `sdf.rs` or `sdf_collide.rs`. |
| Spec B (mesh inertia) + Spec C/D/E | **No conflict.** Spec B is build-time inertia computation. Specs C/D/E are runtime collision. Different pipeline stages, different code paths, no shared mutable state. |

### Existing test impact (cross-spec)

| Test area | Touched by | Conflict risk |
|-----------|-----------|--------------|
| Mesh collision tests | Spec A (hull changes collision shape construction) | **Low.** Existing tests use analytical shapes (sphere, box); mesh collision tests may change if hull is now used instead of raw vertices. Spec A must verify no regression. |
| Heightfield collision tests | Spec C (new pair tests) | **None.** Adds new tests for new pairs. Existing sphere/capsule/box-on-hfield tests unchanged. |
| GJK/EPA tests | Spec D (new distance query tests) | **None.** Adds new test functions. Existing intersection tests unchanged. |
| Flex collision tests | Spec E (new pair tests) | **None.** Adds new tests. Existing primitive flex tests unchanged. |
| SDF collision tests | T1 (parameter threading tests) | **Low.** Existing tests use hardcoded parameters that match defaults (10, 40). With threading, default values are the same, so existing tests still pass. |
| Inertia computation tests | Spec B (new mode tests) | **None.** Adds new tests for shell/convex/legacy modes. Existing exact mode tests unchanged. |
| Phase 8 regression suite | None expected | **None.** Phase 9 does not modify constraint/solver code from Phase 8. |

### Test count changes

| Deliverable | Estimated new tests | Net change |
|-------------|-------------------|------------|
| T1 (§57 SDF options) | 3-5 (parse, defaults, threading) | +3-5 |
| Spec A (§65 convex hull) | 6-10 (Quickhull correctness, degenerate cases, GJK integration) | +6-10 |
| Spec B (§43 mesh inertia) | 8-12 (shell algorithm, primitive shell, convex mode, backward compat) | +8-12 |
| Spec C (§54 hfield pairs) | 6-10 (hfield-mesh, hfield-plane, hfield-hfield, regression) | +6-10 |
| Spec D (§50 CCD) | 8-12 (GJK distance, conservative advancement, tunneling prevention, flag wiring) | +8-12 |
| Spec E (DT-70 flex NP) | 4-6 (flex-vs-mesh, flex-vs-hfield, flex-vs-SDF) | +4-6 |
| **Total** | **35-55** | **+35-55** |

---

## Phase-Level Acceptance Criteria

These are the aggregate gates that determine "Phase 9 complete." Individual
sub-specs have their own ACs for technical correctness. **The overarching
criterion: CortenForge's collision pipeline is functionally complete for
every feature implemented in Phase 9, matching MuJoCo's behavior.**

### PH9-AC1: All 6 tasks ship-complete
Every task in the assignment table (§43, §50, §54, §57, §65, DT-70) has
landed and its sub-spec ACs (or T1 criteria) are met. Every sub-spec AC
that asserts a numerical value has that value verified against MuJoCo's
actual output.

### PH9-AC2: No regression in existing test suite
All 1,900+ domain tests from the post-Phase 8 baseline pass. Zero test
failures attributable to Phase 9 changes.

### PH9-AC3: Quality gate passes
`cargo xtask check` passes (formatting, clippy, all tests).

### PH9-AC4: Aggregate test growth
Domain test count increases by at least 35 (lower bound of per-deliverable
estimates). At least one MuJoCo conformance test (expected value from running
MuJoCo) per sub-spec.

### PH9-AC5: Collision conformance coverage
After Phase 9, the following MuJoCo collision features are covered:

| Feature | Pre-Phase 9 | Post-Phase 9 |
|---------|------------|-------------|
| Mesh convex hull | No auto-computation; `ConvexMesh` uses raw vertices | Quickhull at build time; `ConvexMesh` uses hull vertices |
| Mesh inertia modes | Exact only (signed tetrahedron) | Exact + Shell + Convex + Legacy |
| Heightfield collision pairs | Sphere, capsule, box (cylinder/ellipsoid approximated) | + hfield-mesh, hfield-plane, hfield-hfield |
| Continuous collision detection | Parsed flags, no implementation | Conservative-advancement CCD for convex pairs |
| Flex-vs-complex-geom | Primitive geoms only (plane, sphere, box, capsule, cylinder, ellipsoid) | + mesh, hfield, SDF |
| SDF solver parameters | Hardcoded (iterations=10, initpoints=varies) | Configurable from `<option>` |

### PH9-AC6: Cross-spec dependency satisfied
Spec B's `Convex` inertia mode correctly delegates to Spec A's `ConvexHull`
computation. A mesh with `inertia="convex"` produces inertia values matching
MuJoCo's convex-hull-based inertia for the same mesh.

---

## Out of Scope

Explicitly excluded from Phase 9. Each exclusion states its conformance
impact.

- **GPU collision** (§68, §69) — Post-v1.0 GPU pipeline. *Conformance
  impact: none — GPU acceleration does not change physics results.*

- **XPBD collision re-detect** (DT-26) — Post-v1.0 extension.
  *Conformance impact: none — XPBD is not part of MuJoCo's core solver.*

- **Flex self-collision** (§42A-iv) — Phase 10 Flex Pipeline. *Conformance
  impact: deferred to Phase 10 — flex self-collision is a distinct
  subsystem from rigid collision.*

- **Flex-flex cross-object collision filtering** (§42A-v) — Phase 10.
  *Conformance impact: deferred to Phase 10.*

- **SAP for flex broadphase** (DT-69) — Performance optimization.
  *Conformance impact: none — brute-force produces identical contacts.*

- **Flex adhesion contacts** (DT-72) — Low priority MuJoCo compat.
  *Conformance impact: minimal — affects models using flex adhesion.*

- **Ray-geom intersection for sensors** (DT-119, DT-122) — Sensor-layer
  mesh/hfield/SDF distance queries, not collision pipeline. *Conformance
  impact: affects distance sensors for complex geom types.*

- **Non-convex mesh-mesh collision** — MuJoCo uses convex hulls for mesh
  collision, not per-triangle mesh-mesh testing. CortenForge's existing
  `TriangleMesh` collision shape is a CortenForge extension; Phase 9 does
  not remove it but the convex hull path becomes the default for MuJoCo
  conformance. *Conformance impact: none — convex hull is the conformant
  path.*

- **`<composite>` element** (§46) — Phase 13. Procedural body generation
  is an MJCF preprocessing feature, not collision. *Conformance impact:
  none for collision.*

- **Collision performance optimizations** (DT-18, DT-24) — Post-v1.0.
  *Conformance impact: none — optimizations do not change physics results.*
