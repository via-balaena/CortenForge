# Spec A — Mesh Convex Hull Auto-Computation: Quality Rubric

Grades the Spec A spec on 11 criteria. Target: A+ on every criterion
before implementation begins. A+ means "an implementer could build this
without asking a single clarifying question — and the result would produce
numerically identical output to MuJoCo."

**MuJoCo conformance is the cardinal goal.** Every criterion in this rubric
ultimately serves conformance. P1 (MuJoCo Reference Fidelity) is the most
important criterion — grade it first and hardest. A spec that scores A+ on
all other criteria but has P1 wrong is worse than useless: it would produce
a clean, well-tested implementation of the wrong behavior.

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).
Anything below B does not ship.

---

## Scope Adjustment

The umbrella spec (PHASE9_UMBRELLA.md) contains three factual errors, one
file ownership gap, and one API contract omission discovered during
verification. These must be corrected.

| Umbrella claim | Reality | Action |
|----------------|---------|--------|
| `mjCMesh::MakeConvex()` — Quickhull construction | Function does not exist. Actual function: `mjCMesh::MakeGraph()` in `user_mesh.cc`, which wraps the **Qhull library** (external C library), not a custom Quickhull implementation. | Correct: cite `MakeGraph()` + Qhull delegation |
| `mjmodel.h` → `mesh_convexhull` per-mesh boolean | Field does not exist. Hull presence is indicated by `mesh_graphadr[i] != -1`. Hull computation is triggered internally by `needhull_` flag (set via `SetNeedHull(true)` when any collision geom references the mesh). | Correct: cite `mesh_graphadr`, `mesh_graph`, `needhull_` |
| `convexhull` attribute on `<mesh>` controls hull computation | No such attribute. The historical `convexhull` was on `<compiler>` (MuJoCo 2.x, removed). Modern MuJoCo always computes hulls for collision meshes. User control is only via `maxhullvert` on `<mesh>` (vertex count limit, default -1 = no limit). | Correct: cite `maxhullvert` only |
| `gjk_epa.rs` File Ownership: "Single-owner in Phase 9" (Spec D only) | Spec A scope item 5 requires hill-climbing support in the GJK support function, which lives in `gjk_epa.rs`. Spec A must modify the `support()` match arm for `ConvexMesh` and add warm-start infrastructure (EGT-11). This makes `gjk_epa.rs` dual-owned: Spec A (hill-climbing support) + Spec D (distance query). | Spec must acknowledge dual ownership; coordinate with Spec D to avoid merge conflicts |
| Umbrella `ConvexHull` struct omits adjacency graph | Umbrella API Contract 1 defines `ConvexHull { vertices, faces, normals }` — three fields, no adjacency graph. But hill-climbing (scope item 5, EGT-3, P10) requires adjacency data. | Spec must either add a graph field to `ConvexHull`, define a separate `HullGraph` struct, or use a wrapper type |

**Final scope:**
1. Convex hull computation from mesh vertices at build time (algorithm choice:
   Quickhull in Rust — see EGT-5 for rationale)
2. Adjacency graph data structure for O(√n) hill-climbing GJK support queries
3. `maxhullvert` attribute parsing from `<mesh>` (int, default -1, min 4)
4. Mesh collision integration: hull used for collision dispatch (either via
   `CollisionShape::ConvexMesh` through GJK/EPA, or internally within
   `collide_with_mesh()` — see EGT-10 for the dispatch routing decision)
5. GJK support function: hill-climbing for hulls with ≥10 vertices,
   exhaustive scan for smaller hulls (matching `mjMESH_HILLCLIMB_MIN = 10`)
6. Hull computed only for collision meshes — visual-only meshes skip hull

---

## Empirical Ground Truth

### EGT-1: Hull computation trigger (`needhull_`)

**MuJoCo source:** `user_mesh.cc` → `mjCMesh::Process()`

```cpp
if (needhull_ || face_.empty()) {
    MakeGraph();
}
```

`needhull_` is set to `true` by `SetNeedHull(true)`, called from
`user_model.cc` (around line 5027) when a geom referencing the mesh has
contact properties (`contype`, `conaffinity`, pair membership) or uses
`mjMESH_INERTIA_CONVEX` inertia mode. This is NOT the same as
`SetNotVisual()` (which only sets `visual_ = false`). **MuJoCo computes
hulls for meshes used in collision or convex inertia — not merely for all
non-visual meshes.** Visual-only meshes skip hull computation.

**CortenForge mapping:** Currently all meshes are stored as
`Arc<TriangleMeshData>` in `Model.mesh_data`. There is no visual-only vs
collision distinction at the mesh storage level. The spec must decide how
to determine which meshes need hull computation. Two options:
- **Option A (simple):** Compute hull for all meshes unconditionally.
  Slightly wasteful for visual-only meshes but avoids tracking which geoms
  reference which meshes during build.
- **Option B (conformant):** Track which meshes are referenced by collision
  geoms (geoms with `contype > 0` or `conaffinity > 0`) and compute hull
  only for those. Matches MuJoCo's `SetNeedHull()` behavior.

The scope requires "hull computed only for collision meshes" (scope item 6),
so the spec must either implement option B or justify option A as a
conformance-neutral simplification (since computing an unused hull does not
change collision behavior).

### EGT-2: MakeGraph() — Qhull delegation

**MuJoCo source:** `user_mesh.cc` → `mjCMesh::MakeGraph()`

MuJoCo does NOT implement a custom Quickhull. It delegates to the **Qhull
library** (external C library by Barber, Dobkin, Huhdanpaa):

```cpp
std::string qhopt = "qhull Qt";  // Qt = triangulate all facets
if (maxhullvert_ > -1) {
    qhopt += " TA" + std::to_string(maxhullvert_ - 4);
    // TA = terminate after N vertices added beyond initial simplex
}
// ...
qh_initflags(&qh, qhopt.c_str());
qh_init_B(&qh, points, numpoints, dim, ismalloc);
qh_qhull(&qh);
qh_triangulate(&qh);          // Ensure all faces are triangles
qh_vertexneighbors(&qh);      // Build vertex-to-face adjacency
```

**CortenForge approach:** Since CortenForge is pure Rust, we cannot use the
C Qhull library directly. The spec must specify a Rust convex hull algorithm
(Quickhull is the standard choice). The algorithm must produce an equivalent
convex hull — same vertex set (as a set, ignoring order), same face count,
and identical GJK support function results for any direction vector. Face
triangulation order is implementation-dependent and need not match Qhull's
output exactly.

### EGT-3: Graph data structure layout

**MuJoCo source:** `user_mesh.cc` → `MakeGraph()` graph construction

The graph is a flat `int` array with layout:
```
[0]            = numvert         (hull vertex count)
[1]            = numface         (hull face count)
[2 .. 2+nv-1]           = vert_edgeadr[]    (edge list start per vertex)
[2+nv .. 2+2*nv-1]      = vert_globalid[]   (local hull → original mesh vertex)
[2+2*nv .. 2+3*nv+3*nf-1] = edge_localid[]  (adjacency lists, -1 separated)
[2+3*nv+3*nf .. end]    = face_globalid[]   (3 indices per triangle face)
```

Size formula: `szgraph = 2 + 3*numvert + 6*numface`

**Purpose:** Enables O(√n) hill-climbing support queries. Without this graph,
GJK degrades to O(n) per support call — unacceptable for large meshes.

**CortenForge mapping:** The spec should use a Rust-idiomatic adjacency
structure rather than the flat int array. The key requirement is that the
support function can walk vertex neighbors efficiently.

### EGT-4: GJK support function strategies

**MuJoCo source:** `engine_collision_convex.c`

Two strategies, selected in `mjc_initCCDObj()`:

```c
if (m->mesh_graphadr[m->geom_dataid[g]] < 0 ||
    m->mesh_vertnum[m->geom_dataid[g]] < mjMESH_HILLCLIMB_MIN) {
    obj->support = mjc_meshSupport;       // Exhaustive O(n)
} else {
    obj->support = mjc_hillclimbSupport;  // Hill-climbing O(sqrt(n))
}
```

`mjMESH_HILLCLIMB_MIN = 10` (defined in `engine_collision_convex.h`).

**Exhaustive support** (`mjc_meshSupport`): Linear scan all hull vertices,
find max dot product with direction. Warm-starts from cached `vertindex`.

**Hill-climbing support** (`mjc_hillclimbSupport`): Start from cached vertex
(`meshindex`), walk to neighbors with better dot product, repeat until
convergence. Uses `vert_edgeadr` and `edge_localid` from the graph.

Both functions:
- Rotate direction to local frame before scanning
- Warm-start from previously cached vertex index
- Transform result back to global frame

### EGT-5: Why Quickhull in Rust (not Qhull FFI)

MuJoCo uses Qhull (C library). CortenForge alternatives:
1. **Qhull FFI via `qhull-rs` crate** — possible but adds C dependency,
   complicates build, may have unsafe code
2. **Pure Rust Quickhull** — maintains the pure-Rust property of the sim
   crates, no unsafe FFI, full control over degenerate handling
3. **Incremental convex hull** — simpler than Quickhull but O(n²)

Decision for the spec: **Pure Rust Quickhull** (option 2). Rationale:
- CortenForge sim crates are pure Rust — no C/C++ dependencies
- Quickhull is well-documented (Barber et al. 1996, same paper as Qhull)
- Performance is O(n log n) average, matching Qhull
- Existing Rust crate `convex_hull` or hand-rolled implementation

The spec must decide whether to use an existing crate or implement from
scratch. Either way, the algorithm must handle all degenerate cases that
Qhull handles.

### EGT-6: Degenerate case handling

**MuJoCo source:** `user_mesh.cc`

| Case | MuJoCo behavior |
|------|----------------|
| `nvert < 4` | `MakeGraph()` returns early — no hull computed |
| Duplicate vertices | Removed by `ProcessVertices()` (hash-map dedup) before hull |
| Coplanar points | Handled by Qhull internally (`Qt` triangulation option) |
| Collinear points | Handled by Qhull (produces degenerate hull or error) |
| Qhull failure | Caught via `setjmp`/`longjmp`; graph freed; `szgraph_` zeroed |
| Invalid point IDs | `ok=0` set; graph freed |
| Non-triangular faces | Impossible after `qh_triangulate()` but validated (`cnt == 3`) |

### EGT-7: `maxhullvert` attribute

**MuJoCo source:** `xml_native_reader.cc` + `mjspec.h`

```cpp
// Parsing:
if (ReadAttrInt(elem, "maxhullvert", &n)) {
    if (n != -1 && n < 4) throw mjXError(elem, "maxhullvert must be larger than 3");
    mesh->maxhullvert = n;
}
```

- Default: `-1` (no limit)
- Valid: `-1` or `> 3`
- Converted to Qhull `TA` option as `maxhullvert - 4` (counts vertices
  beyond initial simplex of 4)

**CortenForge mapping:** Parse as `Option<i32>` (None = no limit, Some(n) =
limit, validated n > 3 or n == -1). The Quickhull algorithm must support
early termination at N hull vertices.

### EGT-8: Vertex deduplication before hull

**MuJoCo source:** `user_mesh.cc` → `ProcessVertices()`

Uses `std::unordered_map<VertexKey, int, VertexKey>` to deduplicate
vertices. Face indices are remapped to deduplicated vertex set. This runs
BEFORE `MakeGraph()`.

**CortenForge mapping:** `TriangleMeshData` may already have deduplicated
vertices (check mesh loading pipeline). If not, deduplication must be added
before hull computation.

### EGT-9: Codebase context — files and match sites

**Files that will change:**

| File | Lines | Change |
|------|-------|--------|
| `sim/L0/core/src/mesh.rs` | 2,147 | Add `ConvexHull` struct, `HullGraph` adjacency struct, Quickhull algorithm |
| `sim/L0/core/src/collision_shape.rs` | 1,734 | Modify `ConvexMesh` variant to use hull data, OR keep ConvexMesh and change construction |
| `sim/L0/core/src/gjk_epa.rs` | 1,367 | Update `support()` for ConvexMesh to use hill-climbing with graph |
| `sim/L0/mjcf/src/types.rs` | 4,195 | Add `maxhullvert: Option<i32>` to `MjcfMesh` |
| `sim/L0/mjcf/src/builder/mesh.rs` | 829 | Invoke hull computation during mesh build |

**Exhaustive match sites in `collision_shape.rs`:**

| Function | Lines | Arms | Impact |
|----------|-------|------|--------|
| `is_convex()` | 349–357 | 6 | May need update if ConvexMesh changes |
| `is_mesh_based()` | 418–422 | 3 | May need update |
| `bounding_radius()` | 487–524 | 10 exhaustive | Must update if variant changes |
| `local_aabb()` | 539–606 | 10 exhaustive | Must update if variant changes |

**Exhaustive match site in `gjk_epa.rs`:**

| Function | Lines | Arms | Impact |
|----------|-------|------|--------|
| `support()` | 218–316 | 10 exhaustive | Must update ConvexMesh arm for hill-climbing |

**Current `ConvexMesh` variant:**
```rust
ConvexMesh { vertices: Vec<Point3<f64>> }
```
Only stores vertices — no faces, no normals, no graph. The spec must extend
this to include the adjacency graph for hill-climbing support.

**Current `TriangleMeshData` struct:**
```rust
pub struct TriangleMeshData {
    vertices: Vec<Point3<f64>>,
    triangles: Vec<Triangle>,
    aabb_min: Point3<f64>,
    aabb_max: Point3<f64>,
    bvh: Option<Bvh>,  // skipped during serde
}
```
No `convex_hull` field. The umbrella spec contract says to add
`convex_hull: Option<ConvexHull>`.

### EGT-10: Mesh collision dispatch routing (current architecture)

**CortenForge source:** `sim/L0/core/src/collision/narrow.rs`

Mesh collision is dispatched BEFORE `geom_to_collision_shape()` is called:

```rust
// narrow.rs:87-88 — early return for mesh geoms
if type1 == GeomType::Mesh || type2 == GeomType::Mesh {
    return collide_with_mesh(model, geom1, geom2, pos1, mat1, pos2, mat2, margin);
}

// narrow.rs:172-199 — GJK/EPA fallback only reached if above checks fail
let shape1 = geom_to_collision_shape(type1, size1);  // Returns None for Mesh
let shape2 = geom_to_collision_shape(type2, size2);
```

`geom_to_collision_shape()` (narrow.rs:222) returns `None` for
`GeomType::Mesh`. This is never reached for mesh geoms because the early
return at line 87-88 dispatches to `collide_with_mesh()` first.

**`collide_with_mesh()`** lives in `sim/L0/core/src/collision/mesh_collide.rs`
(192 lines). It uses **per-triangle BVH-accelerated collision** — NOT
GJK/EPA. It dispatches to specialized handlers: `mesh_sphere_contact`,
`mesh_capsule_contact`, `mesh_box_contact`, `mesh_mesh_deepest_contact`,
etc. These are analytical algorithms on the original triangle mesh, not
the convex hull.

**Architectural consequence:** After Spec A, the spec must choose:
- **(a)** Change dispatch: make `geom_to_collision_shape()` return
  `Some(ConvexMesh)` for mesh geoms with a hull, routing through GJK/EPA.
  This **bypasses** `collide_with_mesh()` and its BVH-accelerated
  per-triangle algorithms entirely. The `CollisionShape::ConvexMesh`
  support function fires instead.
- **(b)** Keep dispatch: mesh collision continues through
  `collide_with_mesh()`, which uses the hull internally (e.g., for
  mesh-mesh pairs) while keeping specialized per-triangle handlers
  for mesh-primitive pairs.

No production code currently **constructs** `CollisionShape::ConvexMesh`.
The variant is handled in exhaustive matches in production code
(`raycast.rs`, `bevy/components.rs`, `bevy/mesh.rs`) but all actual
instance construction occurs only in tests (`collision_shape.rs` tests,
`gjk_epa.rs` tests). The support function lives at `gjk_epa.rs:372`.

### EGT-11: GJK warm-starting (current state)

**CortenForge source:** `sim/L0/core/src/gjk_epa.rs`

The current `support_convex_mesh()` (gjk_epa.rs:372-397) performs a
**fresh O(n) scan** of all vertices every call via `find_max_dot()`.
There is no warm-starting mechanism:

```rust
fn support_convex_mesh(
    pose: &Pose,
    vertices: &[Point3<f64>],
    direction: &Vector3<f64>,
) -> Point3<f64> {
    let local_dir = pose.rotation.inverse() * direction;
    let vertex_vectors: Vec<Vector3<f64>> = vertices.iter().map(|p| p.coords).collect();
    let (best_idx, _max_dot) = find_max_dot(&vertex_vectors, &local_dir);
    pose.transform_point(&vertices[best_idx])
}
```

The `Simplex` struct has no field to cache the last-used vertex index.
Each GJK iteration calls `support()` independently with no state
persistence.

**MuJoCo comparison:** MuJoCo caches `vertindex` (for exhaustive) and
`meshindex` (for hill-climbing) on the `mjCCDObj` struct, warm-starting
each support call from the previous result.

**Hidden prerequisite for Spec A:** If the spec adds hill-climbing
support (scope item 5), the GJK framework must be extended to store
cache state. This requires either:
1. Adding a cache field to the support function closure or wrapper
2. Extending `Simplex` to carry per-shape cache state
3. Using a separate cache struct threaded through GJK iterations

The spec must address this architectural change as a prerequisite for
hill-climbing, not assume the infrastructure exists.

---

## Criteria

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does — exact function names, field
> names, calling conventions, and edge cases. No hand-waving. This is the
> single most important criterion: everything else in the spec is downstream
> of getting the MuJoCo reference right.

| Grade | Bar |
|-------|-----|
| **A+** | Every MuJoCo function cited with source file and exact behavior: `mjCMesh::MakeGraph()` in `user_mesh.cc` (Qhull wrapping, graph construction), `mjc_meshSupport()` and `mjc_hillclimbSupport()` in `engine_collision_convex.c` (GJK support strategies), `mjc_initCCDObj()` (strategy selection with `mjMESH_HILLCLIMB_MIN = 10`), `ProcessVertices()` (dedup before hull), `SetNeedHull()` (hull trigger based on `contype`/`conaffinity`/pair membership/convex inertia — NOT `SetNotVisual()`). The `maxhullvert` attribute documented with validation rules (default -1, min 4) and the `-4` offset mapping to Qhull's `TA` option (i.e., `maxhullvert = N` means at most N total hull vertices; Qhull receives `TA(N-4)` because the initial simplex contributes 4). Graph layout documented: `[numvert, numface, vert_edgeadr[], vert_globalid[], edge_localid[], face_globalid[]]` with size formula `2 + 3*numvert + 6*numface` — graph is purely topological (int arrays only, no face normals). All degenerate cases (nvert < 4, coplanar, collinear, duplicates, Qhull failure) documented with MuJoCo's exact behavior. The spec explicitly states that MuJoCo delegates to Qhull, NOT a custom Quickhull. C code snippets included for graph population and support function inner loops. The spec defines what "identical hull output" means for conformance: for **non-truncated hulls** (`maxhullvert = -1`), the hull vertex SET (as a set, ignoring order) and face COUNT must match MuJoCo; face triangulation order is implementation-dependent and need not match; the GJK support function MUST return the same support point as MuJoCo for any direction vector. For **`maxhullvert`-truncated hulls**, vertex set match with Qhull is best-effort (truncation order is implementation-dependent since Quickhull and Qhull traverse vertices in different orders); the conformance requirement relaxes to: hull has at most `maxhullvert` vertices, is a valid convex hull of a subset of the input vertices, and GJK support results are consistent with its own vertex set. |
| **A** | MuJoCo behavior described correctly from C source. Minor gaps in edge-case coverage or graph layout details. |
| **B** | Correct at high level (e.g., "MuJoCo computes convex hulls") but missing function names, graph layout, or support function strategies. |
| **C** | Description based on docs/intuition rather than C source. Claims `MakeConvex()` exists or cites `mesh_convexhull` field. |

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. No "see MuJoCo source"
> or "TBD" gaps. Rust code is line-for-line implementable.

| Grade | Bar |
|-------|-----|
| **A+** | Quickhull algorithm fully specified in Rust pseudocode: initial simplex construction from 4 extreme/non-coplanar points, conflict list management (which unprocessed points are "above" which face), iterative expansion (pick farthest conflict point, find horizon edges, create new cone of faces, redistribute conflict points), termination when no conflicts remain. Degenerate case guards specified with Rust code: coplanar detection (volume test < ε), collinear detection (cross product magnitude < ε), duplicate removal. `maxhullvert` early-termination logic specified. Graph construction from hull output specified. Any intentional deviation from Qhull's algorithm explicitly justified with proof that it produces a valid convex hull. |
| **A** | Algorithm complete and correct. One or two minor details left implicit (e.g., exact epsilon values). |
| **B** | Algorithm structure clear but degenerate handling hand-waved, or graph construction deferred. |
| **C** | Skeleton only — "implement Quickhull." |

### P3. Convention Awareness

> Spec explicitly addresses our codebase's conventions where they differ from
> MuJoCo and provides the correct translation.

| Grade | Bar |
|-------|-----|
| **A+** | Every MuJoCo → CortenForge translation called out: MuJoCo's flat `int*` graph → CortenForge's Rust-idiomatic adjacency struct; MuJoCo's `float*` vertex storage → CortenForge's `Vec<Point3<f64>>`; MuJoCo's `mesh_graphadr`/`mesh_graph` global arrays → CortenForge's per-mesh `Option<ConvexHull>` on `TriangleMeshData`; MuJoCo's `geom_dataid` → CortenForge's `geom_mesh: Vec<Option<usize>>`; MuJoCo's rotation matrix in support function → CortenForge's `UnitQuaternion<f64>` in `Pose`. Umbrella contract reconciliation: (1) the umbrella's `ConvexHull` struct includes `normals: Vec<Vector3<f64>>` but MuJoCo's graph stores NO face normals (purely topological int arrays) — the spec must justify precomputing normals as a CortenForge convenience (for Spec B inertia computation) or remove them from the contract; (2) the umbrella's `ConvexHull` struct omits the adjacency graph entirely (only has `vertices`, `faces`, `normals`) but hill-climbing support (scope item 5, EGT-3, P10) requires adjacency data — the spec must either add a graph field to `ConvexHull`, define a separate `HullGraph` struct, or store adjacency on a wrapper type. Convention difference table present. Existing `CollisionShape::ConvexMesh` variant documented with exact current fields and how it changes. |
| **A** | Major conventions documented. Minor field-name mappings left to implementer. |
| **B** | Some conventions noted, others not — risk of silent mismatch during implementation. |
| **C** | MuJoCo code pasted without adaptation to our conventions. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. Contains concrete values,
> tolerances, and model configurations. No "should work correctly."

| Grade | Bar |
|-------|-----|
| **A+** | Every AC has the three-part structure: (1) concrete input mesh (e.g., unit cube with 8 vertices, tetrahedron with 4 vertices, icosphere with 42 vertices), (2) exact expected hull vertex/face count, (3) what to assert. At least one AC verifies hull correctness against known geometry (e.g., cube hull has 8 vertices and 12 triangular faces). At least one AC verifies the GJK support function produces the same support point with and without hill-climbing for a small mesh. At least one AC verifies `maxhullvert` limits the hull vertex count — including boundary case `maxhullvert = 4` (minimum valid value, maps to Qhull `TA0`, result should be exactly a 4-vertex tetrahedron from the initial simplex). Edge case ACs: nvert < 4 → no hull, duplicate vertices → hull computed on deduplicated set. The spec defines what "hull identical to MuJoCo" means: vertex set matches (ignoring order), face count matches, GJK support function result matches for any direction. Code-review ACs explicitly labeled. |
| **A** | ACs are testable. Some lack exact numerical expectations. |
| **B** | ACs directionally correct but vague ("hull should be computed"). |
| **C** | ACs are aspirational statements, not tests. |

### P5. Test Plan Coverage

> Tests cover happy path, edge cases, regressions, and interactions.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present. Edge case inventory: fewer than 4 vertices, exactly 4 (minimal tetrahedron), coplanar points (flat quad), collinear points, duplicate vertices, large mesh (>100 vertices), `maxhullvert` limiting (including boundary `maxhullvert = 4`), visual-only mesh (no hull), already-convex mesh (all vertices on hull — hull vertex count equals input vertex count), near-degenerate geometry (thin sliver mesh with nearly-coplanar clusters). Negative cases: hull NOT computed for visual-only mesh, hull NOT computed for nvert < 4. At least one test verifies hill-climbing support and exhaustive support produce identical results for the same hull and direction. At least one test uses a non-trivial mesh (icosphere, Stanford bunny subset) to catch bugs in large hull computation. Serde round-trip test: serialize `TriangleMeshData` with hull → deserialize → verify hull is available (either preserved or rebuilt). Supplementary tests justified. |
| **A** | Good coverage. Minor edge-case gaps. |
| **B** | Happy path covered. Edge cases sparse. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and interactions with other specs are
> explicitly stated.

| Grade | Bar |
|-------|-----|
| **A+** | Execution order unambiguous. S1 (ConvexHull struct + Quickhull algorithm in mesh.rs), S2 (graph construction + hill-climbing support in gjk_epa.rs), S3 (`maxhullvert` parsing in types.rs + builder), S4 (CollisionShape::ConvexMesh construction using hull), S5 (integration tests). Each section states what it requires from prior sections. Spec A's output contracts documented for downstream consumption: (1) `ConvexHull` struct for Spec B (inertia from hull) and Spec E (flex-mesh pairs may use hull), (2) the dispatch routing decision (EGT-10 option a vs b) which cascades to Spec C (heightfield-mesh pairs) and Spec D (CCD — which collision path fires). `gjk_epa.rs` dual ownership with Spec D acknowledged (see Scope Adjustment). No cross-spec blocking — Spec A is independent (does not depend on other specs). |
| **A** | Order clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Spec identifies every file touched, every behavior that changes, and every
> existing test that might break.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description. Exhaustive match sites in `collision_shape.rs` (`is_convex()` at 349–357, `is_mesh_based()` at 418–422, `bounding_radius()` at 487–524, `local_aabb()` at 539–606) and `gjk_epa.rs` (`support()` at 218–316) enumerated with expected changes. Behavioral change: `CollisionShape::ConvexMesh` now uses hull vertices instead of original mesh vertices — this moves TOWARD MuJoCo conformance. Existing mesh collision tests identified: any test that creates a ConvexMesh from raw vertices must still work (hull vertices ARE the ConvexMesh vertices after Spec A). ConvexMesh tests in `collision_shape.rs` verified: `is_convex()` returns true for ConvexMesh (test at line 976) — this should still hold. Predicate test section at lines 1511–1559. No data staleness guards affected. |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. Some regression risk unaddressed. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions within the spec. Shared concepts use identical
> terminology throughout.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology uniform: "ConvexHull" (the struct), "hull" (the computed result), "graph" (the adjacency data), "support function" (GJK query). Every cross-reference accurate. File paths in Specification match Files Affected. AC numbers match Traceability Matrix. Edge case lists consistent between MuJoCo Reference and Test Plan. The spec's ConvexHull definition reconciles the umbrella's API contract per the Scope Adjustment table (adding adjacency graph, justifying or removing normals). |
| **A** | Consistent. One or two minor terminology inconsistencies. |
| **B** | Some sections use different names for the same concept. |
| **C** | Contradictions between sections. |

### P9. Geometric Correctness

> The convex hull algorithm is mathematically rigorous: produces a valid
> convex polytope for all non-degenerate inputs, handles all degenerate
> inputs gracefully, and maintains numerical stability.

*Boundary with P1:* P1 grades whether the spec correctly describes MuJoCo's
behavior (what Qhull does). P9 grades whether the specified Rust algorithm
is mathematically sound and produces equivalent results.

| Grade | Bar |
|-------|-----|
| **A+** | Quickhull correctness argued: initial simplex is non-degenerate (4 non-coplanar points selected via extremal heuristic — e.g., maximizing coordinate spread or simplex volume to avoid near-degenerate starting configurations; Qhull uses extremal coordinate spread, other valid heuristics exist). Conflict graph correctly partitions remaining points, horizon edge detection produces a simple closed polygon on the hull surface, new cone faces have consistent outward winding. Epsilon values justified (e.g., volume test ε = 1e-10 for coplanarity). Proof sketch that all interior points are below all hull faces upon termination. Degenerate cases produce defined fallback behavior (not panic or undefined hull). Near-degenerate robustness addressed: behavior for closely-spaced vertex clusters, thin-sliver initial simplices (nearly-flat tetrahedra), and conflict points very close to face planes (distance ≈ ε). |
| **A** | Algorithm is correct. Epsilon values chosen but not fully justified. |
| **B** | Algorithm seems correct but no argument for degenerate cases. |
| **C** | Algorithm correctness assumed without analysis. |

### P10. GJK Support Integration

> The convex hull integrates correctly with the GJK/EPA collision system:
> support function is efficient, warm-starting works, and the result is
> numerically identical to exhaustive search.

*Boundary with P2:* P2 grades algorithm completeness (all steps specified).
P10 grades integration correctness (the support function works correctly
within the GJK framework).

| Grade | Bar |
|-------|-----|
| **A+** | Hill-climbing support function specified with: adjacency walk from cached vertex, convergence guarantee (convex hull graph is connected, objective is unimodal on convex surface), warm-start from cached vertex index. **Hidden prerequisite acknowledged:** CortenForge's current GJK has NO warm-start infrastructure (see EGT-11) — the spec must design a caching mechanism (e.g., per-shape cache state threaded through GJK iterations, or an extended `Simplex` struct) before hill-climbing can work. Exhaustive fallback for hull vertex count < 10 (matching `mjMESH_HILLCLIMB_MIN`). Direction-to-local-frame rotation specified (matching existing `support()` convention in `gjk_epa.rs`). The spec proves that hill-climbing produces the same support point as exhaustive search for any convex hull (unimodality argument). |
| **A** | Support function correct. Hill-climbing/exhaustive threshold noted. Minor gap in warm-start details. |
| **B** | Support function described but no hill-climbing, or hill-climbing without convergence argument. |
| **C** | Support function not discussed, or assumes O(n) exhaustive is sufficient. |

### P11. Build-Time Pipeline Integration

> The hull computation is correctly wired into the mesh processing pipeline:
> computed at the right time, stored in the right place, consumed by the
> right downstream code.

*Boundary with P7:* P7 grades blast radius (what files change, what might
break). P11 grades pipeline correctness (the computation happens at the
right stage and flows to the right consumers).

| Grade | Bar |
|-------|-----|
| **A+** | Build-time pipeline documented: `convert_mjcf_mesh()` or `process_mesh()` in `builder/mesh.rs` → compute hull → store `ConvexHull` on `TriangleMeshData` → `Arc<TriangleMeshData>` registered in `Model.mesh_data`. **Runtime dispatch routing explicitly decided:** currently `geom_to_collision_shape()` in `narrow.rs` returns `None` for `GeomType::Mesh`, routing mesh collision through the dedicated `collide_with_mesh()` path in `mesh_collide.rs`. The spec must unambiguously choose between: (a) changing `geom_to_collision_shape()` to return `Some(ConvexMesh)` for mesh geoms with a hull (routing through GJK/EPA, bypassing `mesh_collide.rs`), or (b) keeping the dispatch unchanged and having `collide_with_mesh()` use the hull internally. This decision has cascade effects on which collision path fires, whether `mesh_collide.rs` specialized pairs are bypassed, and what existing tests break. Visual-only mesh detection specified: which meshes skip hull computation and how this is determined. Serde strategy for `ConvexHull` on `TriangleMeshData` explicitly decided: either derives Serialize/Deserialize (hull persists across serde round-trips) or uses `#[serde(skip)]` with rebuild-on-demand (matching the existing BVH pattern). `maxhullvert` threading: parsed from MJCF → stored on `MjcfMesh` → passed to Quickhull at build time. Original mesh vertices/faces confirmed preserved (hull is additive, not a replacement) — original data remains available for rendering, inertia computation (Spec B), and non-convex queries. |
| **A** | Pipeline correct. Minor gap in visual-only detection or maxhullvert threading. |
| **B** | Pipeline described at high level but missing specific function calls or data flow. |
| **C** | Pipeline not discussed. |

---

## Rubric Self-Audit

### Self-audit checklist

- [x] **Specificity:** Every A+ bar names specific MuJoCo functions
      (`MakeGraph()`, `mjc_meshSupport()`, `mjc_hillclimbSupport()`,
      `mjc_initCCDObj()`), specific CortenForge files and line ranges
      (`collision_shape.rs:349-357`, `gjk_epa.rs:218-316`), specific
      degenerate cases (nvert < 4, coplanar, collinear, duplicates), and
      specific constants (`mjMESH_HILLCLIMB_MIN = 10`). Two independent
      reviewers could assign the same grade by checking these specifics.

- [x] **Non-overlap:** P1 vs P9 boundary: P1 grades MuJoCo reference
      accuracy, P9 grades mathematical correctness of the Rust algorithm.
      P2 vs P10 boundary: P2 grades algorithm completeness (all steps
      specified), P10 grades GJK integration correctness (support function
      works within GJK framework). P6 vs P11 boundary: P6 grades inter-spec
      dependency documentation (output contracts, downstream cascades), P11
      grades pipeline correctness (right stage, right consumers). P7 vs P11
      boundary: P7 grades blast radius (files changed, tests broken), P11
      grades pipeline correctness. Each pair grades the same content from
      a different angle.

- [x] **Completeness:** 11 criteria cover: MuJoCo reference (P1), algorithm
      (P2), conventions (P3), ACs (P4), tests (P5), dependencies (P6), blast
      radius (P7), consistency (P8), geometric correctness (P9), GJK
      integration (P10), build pipeline (P11). No meaningful gap — a spec
      that is A+ on all 11 but still has a problem would require a 12th
      criterion not yet imagined.

- [x] **Gradeability:** P1 → MuJoCo Reference section. P2 → Specification
      (S1..SN). P3 → Convention Notes. P4 → Acceptance Criteria. P5 → Test
      Plan + Traceability Matrix. P6 → Prerequisites + Execution Order. P7 →
      Risk & Blast Radius. P8 → cross-cutting. P9 → Algorithm section
      (mathematical argument). P10 → GJK support function section. P11 →
      Build pipeline section.

- [x] **Conformance primacy:** P1 is tailored with exact C function names,
      source files, graph layout, and degenerate cases. P4 requires
      hull vertex/face counts for known geometries. P5 requires support
      function equivalence tests. The rubric cannot produce an A+ spec that
      diverges from MuJoCo's hull behavior.

- [x] **Empirical grounding:** EGT-1 through EGT-11 filled in with verified
      MuJoCo C source analysis and CortenForge codebase audit. Every A+ bar
      that references MuJoCo behavior has a corresponding EGT entry.
      CortenForge-side facts (dispatch routing, warm-start state, existing
      ConvexMesh usage) are documented in EGT-9, EGT-10, and EGT-11. No
      criterion bar was written from assumptions — all verified against
      `user_mesh.cc`, `engine_collision_convex.c`, and the CortenForge source.

### Criterion → Spec section mapping

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P1 | MuJoCo Reference, Key Behaviors table, Convention Notes |
| P2 | Specification (S1: ConvexHull + Quickhull, S2: Graph + Support) |
| P3 | Convention Notes, Specification code |
| P4 | Acceptance Criteria |
| P5 | Test Plan, AC→Test Traceability Matrix, Edge Case Inventory |
| P6 | Prerequisites, Execution Order |
| P7 | Risk & Blast Radius (Behavioral Changes, Files Affected, Existing Test Impact) |
| P8 | *Cross-cutting — all sections checked for mutual consistency* |
| P9 | Specification S1 (Quickhull algorithm, degenerate handling) |
| P10 | Specification S2 (GJK support function, hill-climbing, warm-start, EGT-11) |
| P11 | Specification S3/S4 (build pipeline, dispatch routing per EGT-10) |

---

## Scorecard

| Criterion | Grade | Evidence |
|-----------|-------|----------|
| P1. MuJoCo Reference Fidelity | | |
| P2. Algorithm Completeness | | |
| P3. Convention Awareness | | |
| P4. Acceptance Criteria Rigor | | |
| P5. Test Plan Coverage | | |
| P6. Dependency Clarity | | |
| P7. Blast Radius & Risk | | |
| P8. Internal Consistency | | |
| P9. Geometric Correctness | | |
| P10. GJK Support Integration | | |
| P11. Build-Time Pipeline Integration | | |

**Overall: — (Rev 5)**

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| R1 | P1 | Umbrella cites `MakeConvex()` — does not exist; actual function is `MakeGraph()` | Rubric Phase 1 (MuJoCo C source read) | Scope Adjustment table added; P1 A+ bar requires correct function name | Rubric Rev 1 |
| R2 | P1 | Umbrella cites `mesh_convexhull` field — does not exist; actual indicator is `mesh_graphadr != -1` | Rubric Phase 1 | Scope Adjustment table added; EGT-1 documents actual trigger | Rubric Rev 1 |
| R3 | P1 | Umbrella cites `convexhull` attribute on `<mesh>` — does not exist; actual attribute is `maxhullvert` | Rubric Phase 1 | Scope Adjustment table added; EGT-7 documents `maxhullvert` | Rubric Rev 1 |
| R4 | P2 | MuJoCo uses Qhull (external library), not custom Quickhull — spec must choose Rust algorithm | Rubric Phase 1 | EGT-2 documents Qhull delegation; EGT-5 documents Rust algorithm choice; P2 A+ bar updated | Rubric Rev 1 |
| R5 | P10 | Graph data structure essential for O(√n) hill-climbing — not just vertices/faces | Rubric Phase 1 | EGT-3 documents graph layout; P10 A+ bar requires hill-climbing specification | Rubric Rev 1 |
| R6 | P11 | Visual-only mesh skip behavior needs specification | Rubric Phase 1 | EGT-1 documents `needhull_` trigger; P11 A+ bar requires visual-only detection | Rubric Rev 1 |
| R7 | P1, EGT-1 | `needhull_` is NOT set by `SetNotVisual()` — actually set by `SetNeedHull(true)` based on geom contact properties (`contype`, `conaffinity`, pair membership, `mjMESH_INERTIA_CONVEX`) | Stress test (MuJoCo source verification) | EGT-1 corrected; P1 A+ bar updated to cite `SetNeedHull()` | Rubric Rev 2 |
| R8 | P11 | **CRITICAL:** Dispatch routing for mesh geoms after hull not addressed — `geom_to_collision_shape()` returns `None` for Mesh, so spec must decide whether mesh collision routes through GJK/EPA (ConvexMesh) or stays in `collide_with_mesh()` | Stress test (blind spot analysis) | P11 A+ bar updated to require explicit dispatch routing decision | Rubric Rev 2 |
| R9 | P3 | Umbrella contract includes `normals: Vec<Vector3<f64>>` on ConvexHull but MuJoCo's graph stores NO face normals (purely topological int arrays) | Stress test (blind spot analysis) | P3 A+ bar updated to require reconciliation with umbrella contract | Rubric Rev 2 |
| R10 | P9 | Near-degenerate geometry (thin slivers, closely-spaced clusters) not addressed — distinct failure mode from strictly degenerate cases | Stress test (blind spot analysis) | P9 A+ bar updated to require near-degenerate robustness discussion and initial simplex quality heuristic | Rubric Rev 2 |
| R11 | P4 | `maxhullvert = 4` boundary case not required — maps to Qhull `TA0`, should produce exactly 4-vertex tetrahedron | Stress test (blind spot analysis) | P4 A+ bar updated to require boundary AC; P1 A+ bar documents `-4` offset mapping | Rubric Rev 2 |
| R12 | P11 | Serde strategy for `ConvexHull` on `TriangleMeshData` not addressed — must choose serialize vs skip+rebuild | Stress test (blind spot analysis) | P11 A+ bar updated to require explicit serde decision | Rubric Rev 2 |
| R13 | P1, P4 | Hull "identical" definition missing — different valid hulls can have different face triangulations | Stress test (blind spot analysis) | P1 A+ bar defines conformance: vertex set matches, face count matches, GJK support result matches; face order is implementation-dependent | Rubric Rev 2 |
| R14 | P5 | Missing edge cases: already-convex mesh, near-degenerate geometry, serde round-trip, `maxhullvert=4` boundary | Stress test (blind spot analysis) | P5 A+ bar updated with all four edge cases | Rubric Rev 2 |
| R15 | EGT-2 | "identical hull topology" contradicts P1's conformance definition (allows different face triangulation order) | Stress test round 2 (internal consistency) | EGT-2 rewritten: "equivalent convex hull — same vertex set, same face count, identical GJK support results" | Rubric Rev 3 |
| R16 | EGT-1 | EGT-1 suggests "compute hull for all meshes" but Scope item 6 says "collision only" — contradiction | Stress test round 2 (internal consistency) | EGT-1 rewritten to present both options and note scope requires collision-only or justification for all-meshes | Rubric Rev 3 |
| R17 | P11 | No EGT entry for dispatch routing facts (`geom_to_collision_shape()` → None, `collide_with_mesh()` uses per-triangle BVH not GJK) | Stress test round 2 (dispatch deep dive) | Added EGT-10 documenting full dispatch architecture with code snippets | Rubric Rev 3 |
| R18 | P10 | GJK has NO warm-start infrastructure — `support_convex_mesh()` does fresh O(n) scan every call, no cache fields | Stress test round 2 (dispatch deep dive) | Added EGT-11 documenting current GJK warm-start state; P10 A+ bar updated to acknowledge hidden prerequisite | Rubric Rev 3 |
| R19 | P9 | "maximizes simplex volume" is not what Qhull does — Qhull uses extremal coordinate spread heuristic | Stress test round 2 (internal consistency) | P9 A+ bar updated: "maximizing coordinate spread or simplex volume... Qhull uses extremal coordinate spread" | Rubric Rev 3 |
| R20 | Scope | Scope item 4 biased toward dispatch option (a); P11 allows option (b) | Stress test round 2 (internal consistency) | Scope item 4 rewritten to be dispatch-neutral, references EGT-10 | Rubric Rev 3 |
| R21 | EGT-10 | File path `collision_mesh.rs` is wrong — actual file is `collision/mesh_collide.rs` (192 lines) | Stress test round 3 (EGT-10/11 source verification) | EGT-10 file path corrected | Rubric Rev 4 |
| R22 | Scope, P6 | `gjk_epa.rs` ownership conflict: umbrella says single-owner (Spec D), but Spec A scope item 5 requires hill-climbing support modifications in that file | Stress test round 3 (criterion coverage gaps) | Scope Adjustment table expanded with 4th umbrella correction; P6 A+ bar updated to acknowledge dual ownership | Rubric Rev 4 |
| R23 | P3, Scope | Umbrella `ConvexHull` struct omits adjacency graph (only has vertices, faces, normals) but P10 requires adjacency data for hill-climbing — P3 reconciliation only covered `normals`, not the missing graph | Stress test round 3 (umbrella contract alignment) | P3 A+ bar expanded to require reconciliation of both `normals` discrepancy AND missing adjacency graph; Scope Adjustment table expanded with 5th row (API contract omission) | Rubric Rev 4 |
| R24 | EGT-10 | ConvexMesh "only used in tests (sdf.rs tests)" is wrong — variant IS matched in production code (raycast.rs, bevy); sdf.rs tests claim is fabricated | Stress test round 3 (EGT-10/11 source verification) | EGT-10 rewritten: "No production code constructs ConvexMesh; variant is matched in production (raycast.rs, bevy)" | Rubric Rev 4 |
| R25 | P6 | Dispatch routing decision (EGT-10) is a dependency output that cascades to Specs C, D, E — P6 only mentioned ConvexHull struct as output contract | Stress test round 3 (criterion coverage gaps) | P6 A+ bar updated to list both output contracts (ConvexHull struct + dispatch routing decision) | Rubric Rev 4 |
| R26 | P8 | P8 A+ bar says umbrella fields "match" spec's ConvexHull definition, but after R23 the umbrella requires reconciliation (missing adjacency graph, extra normals) — stale claim | Stress test round 4 (internal consistency) | P8 A+ bar reworded: "reconciles the umbrella's API contract per the Scope Adjustment table" | Rubric Rev 5 |
| R27 | P1 | `maxhullvert`-truncated hulls: rubric requires "vertex set matches MuJoCo" but Quickhull and Qhull traverse vertices differently, producing different N-vertex subsets upon early termination — tension between P1 and P2/P9 | Stress test round 4 (adversarial spec author) | P1 A+ bar split conformance: non-truncated hulls require vertex set match; truncated hulls relax to "at most N vertices, valid convex hull, GJK support consistent with own vertex set" | Rubric Rev 5 |
| R28 | P6 | P6 says dispatch routing cascades to Spec E (flex-mesh), but Spec E depends on ConvexHull struct (output 1), not dispatch routing (output 2) — Spec E dispatches through flex_collide.rs, not narrow.rs | Stress test round 4 (cross-reference verification) | P6 A+ bar corrected: Spec E listed under output contract 1 (ConvexHull struct), not output contract 2 (dispatch routing) | Rubric Rev 5 |
| R29 | P7 | "Predicate tests (lines 986–1558)" is misleading — range covers entire test module; actual predicate tests are 1511–1559; ConvexMesh `is_convex()` test is at line 976 | Stress test round 4 (source verification) | P7 A+ bar corrected: cites ConvexMesh test at line 976, predicate section at 1511–1559 | Rubric Rev 5 |
| R30 | Self-audit | Non-overlap analysis missing P6/P11 boundary pair created by R25 — both criteria now reference dispatch routing decision | Stress test round 4 (internal consistency) | Self-audit Non-overlap item expanded with P6/P11 boundary (inter-spec dependency vs pipeline correctness) | Rubric Rev 5 |
| R31 | Gap log | R23's resolution column omitted that Scope Adjustment row 5 was added — only mentioned P3 A+ bar update | Stress test round 4 (gap log completeness) | R23 resolution updated to include Scope Adjustment table expansion | Rubric Rev 5 |
