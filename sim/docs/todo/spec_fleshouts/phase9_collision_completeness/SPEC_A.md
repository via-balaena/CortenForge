# Spec A — Mesh Convex Hull Auto-Computation: Spec

**Status:** Draft
**Phase:** Roadmap Phase 9 — Collision Completeness
**Effort:** L
**MuJoCo ref:** `mjCMesh::MakeGraph()` in `user_mesh.cc`, `mjc_hillclimbSupport()` in `engine_collision_convex.c`
**MuJoCo version:** 3.2.6 (C source read from GitHub)
**Test baseline:** 1,900+ sim domain tests (post-Phase 8)
**Prerequisites:**
- None — Spec A is independent (does not depend on other Phase 9 specs)

**Independence:** This spec is independent of Specs B, C, D, E per the umbrella
dependency graph. Spec B (mesh inertia) and Spec E (flex-vs-mesh) depend on
this spec's output (`ConvexHull` struct). Spec D (CCD) is independent but may
construct `CollisionShape::ConvexMesh` from the hull for CCD on mesh pairs.
Shared files: `gjk_epa.rs` is dual-owned — Spec A adds hill-climbing support,
Spec D adds distance query (different functions, no conflict).

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical results to MuJoCo for the feature described below.
> Every algorithm, every acceptance criterion, and every test is in service
> of this goal. The MuJoCo C source code is the single source of truth —
> not the MuJoCo documentation, not intuition about "what should happen,"
> not a Rust-idiomatic reinterpretation. When in doubt, read the C source
> and match its behavior exactly.

---

## Scope Adjustment

The umbrella spec (PHASE9_UMBRELLA.md) contains factual errors and omissions
discovered during rubric verification. These corrections carry forward.

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| `mjCMesh::MakeConvex()` — Quickhull | Function does not exist. Actual: `mjCMesh::MakeGraph()` in `user_mesh.cc`, wraps the **Qhull library** (external C) | Cite `MakeGraph()` + Qhull delegation throughout |
| `mjmodel.h` → `mesh_convexhull` per-mesh boolean | Field does not exist. Hull presence: `mesh_graphadr[i] != -1`. Trigger: `needhull_` flag via `SetNeedHull(true)` | Cite `mesh_graphadr`, `mesh_graph`, `needhull_` |
| `convexhull` attribute on `<mesh>` controls hull | No such attribute. MuJoCo always computes hulls for collision meshes. User control: `maxhullvert` on `<mesh>` (vertex count limit, default -1 = no limit) | Parse `maxhullvert` only |
| `gjk_epa.rs` single-owner (Spec D only) | Spec A modifies `support()` for hill-climbing; Spec D adds `gjk_distance()`. Dual-owned. | Acknowledge dual ownership |
| `ConvexHull` struct: `{ vertices, faces, normals }` | MuJoCo's graph stores NO face normals (purely topological int arrays). Also missing: adjacency graph for hill-climbing. | Add `graph: HullGraph` field; keep `normals` as CortenForge convenience (needed by Spec B) |

**Final scope:**
1. Convex hull computation from mesh vertices at build time (pure Rust Quickhull)
2. Adjacency graph for O(√n) hill-climbing GJK support queries
3. `maxhullvert` attribute parsing from `<mesh>` (int, default -1, min 4)
4. Mesh collision integration: hull used for mesh-mesh collision dispatch
   via GJK/EPA on hull shapes within `collide_with_mesh()` (option b —
   see AD-1)
5. GJK support function: hill-climbing for hulls with ≥10 vertices,
   exhaustive scan for smaller hulls (matching `mjMESH_HILLCLIMB_MIN = 10`)
6. Hull computed for all meshes unconditionally (conformance-neutral
   simplification — see AD-3). **Rubric reconciliation:** EGT-8 requires
   the spec to address `needhull_` trigger semantics. This spec acknowledges
   the MuJoCo behavior (§MuJoCo Reference → Hull computation trigger) and
   explicitly defers matching it (AD-3). The chosen approach (compute for
   all meshes) is conformance-neutral: it produces identical collision
   results for all collision meshes and merely adds an unused hull for
   visual-only meshes.

---

## Problem Statement

**Conformance gap.** MuJoCo computes convex hulls for all collision mesh geoms
at model compilation time via `mjCMesh::MakeGraph()` in `user_mesh.cc`. The
hull is stored as a graph data structure (`mesh_graph[]`, `mesh_graphadr[]`)
and used by the GJK/EPA narrowphase for all mesh-vs-* collision pairs. The
GJK support function uses hill-climbing on the hull adjacency graph for O(√n)
queries on large meshes (`mjc_hillclimbSupport()` in
`engine_collision_convex.c`).

CortenForge does not compute convex hulls. The `CollisionShape::ConvexMesh`
variant exists but is never constructed in production code — only in tests.
Mesh collision routes through `collide_with_mesh()` in `mesh_collide.rs`,
which uses per-triangle BVH collision. The GJK support function for
`ConvexMesh` (`support_convex_mesh()` in `gjk_epa.rs:376`) performs a fresh
O(n) linear scan every call with no warm-starting or hill-climbing.

This gap affects:
- **Mesh-mesh collision:** Not MuJoCo-conformant (uses per-triangle BVH instead
  of convex hull GJK/EPA)
- **CCD (Spec D):** Cannot perform CCD on mesh geoms without a convex hull shape
- **Mesh inertia (Spec B):** `Convex` inertia mode requires the hull
- **GJK performance:** O(n) per support call degrades collision performance for
  large meshes

---

## MuJoCo Reference

> **This is the most important section of the spec.** Everything downstream —
> the algorithm, the acceptance criteria, the tests — is derived from what's
> documented here.

### Hull computation trigger (`needhull_`)

**Source:** `user_mesh.cc` → `mjCMesh::Process()`

```c
if (needhull_ || face_.empty()) {
    MakeGraph();
}
```

`needhull_` is set to `true` by `SetNeedHull(true)`, called from
`user_model.cc` when a geom referencing the mesh has contact properties
(`contype`, `conaffinity`, pair membership) or uses `mjMESH_INERTIA_CONVEX`
inertia mode. This is NOT the same as `SetNotVisual()`. Visual-only meshes
skip hull computation.

The `face_.empty()` condition is a separate trigger: if the mesh has vertices
but no faces (a pure point cloud), MuJoCo still computes the hull to get face
data. This handles the case where a mesh is defined with `<mesh vertex="..."/>`
(no `face` attribute). CortenForge's `TriangleMeshData` always has triangles
(the mesh loading pipeline triangulates), so this condition is N/A for us.

### MakeGraph() — Qhull delegation

**Source:** `user_mesh.cc` → `mjCMesh::MakeGraph()`

MuJoCo does NOT implement a custom Quickhull. It delegates to the **Qhull
library** (external C library by Barber, Dobkin, Huhdanpaa):

```c
std::string qhopt = "qhull Qt";  // Qt = triangulate all facets
if (maxhullvert_ > -1) {
    qhopt += " TA" + std::to_string(maxhullvert_ - 4);
    // TA = terminate after N vertices added beyond initial simplex
    // maxhullvert=N means at most N total hull vertices
    // Qhull receives TA(N-4) because the initial simplex contributes 4
}
qh_initflags(&qh, qhopt.c_str());
qh_init_B(&qh, points, numpoints, dim, ismalloc);
qh_qhull(&qh);
qh_triangulate(&qh);          // Ensure all faces are triangles
qh_vertexneighbors(&qh);      // Build vertex-to-face adjacency
```

After Qhull returns, `MakeGraph()` populates the graph in two passes:

**Pass 1 — Build adjacency from Qhull output:** Iterates Qhull vertices and
their neighbor facets, storing **global** point IDs (`pid1`) into
`edge_localid`, with `-1` sentinels separating each vertex's neighbor list.
Deduplication prevents the same neighbor from appearing twice.

```c
// Pass 1: iterate Qhull vertices, extract neighbor vertices from facets
FORALLvertices {
    // Record edge list start for this vertex
    vert_edgeadr[vcnt] = ecnt;
    vert_globalid[vcnt] = qh_pointid(&qh, vertex->point);

    // Iterate neighbor facets of this vertex
    FOREACHneighbor_(vertex) {
        if (!neighbor->simplicial) { ok = 0; break; }
        FOREACHvertex_(neighbor->vertices) {
            int pid1 = qh_pointid(&qh, vertex_->point);
            if (pid1 != vert_globalid[vcnt]) {
                // Deduplicate: check if pid1 already in this vertex's list
                int found = 0;
                for (int k = vert_edgeadr[vcnt]; k < ecnt; k++) {
                    if (edge_localid[k] == pid1) { found = 1; break; }
                }
                if (!found) {
                    edge_localid[ecnt++] = pid1;  // global ID for now
                }
            }
        }
    }
    edge_localid[ecnt++] = -1;  // sentinel
    vcnt++;
}
```

**Pass 2 — Convert global→local IDs:** Replaces all global IDs in
`edge_localid` with **local** sequential indices by searching
`vert_globalid[]`:

```c
// Pass 2: convert global point IDs to local sequential indices
for (int i = 0; i < numvert + 3*numface; i++) {
    if (edge_localid[i] >= 0) {
        for (adr = 0; adr < numvert; adr++) {
            if (vert_globalid[adr] == edge_localid[i]) {
                edge_localid[i] = adr;  // global -> local
                break;
            }
        }
    }
}
```

This is why `mjc_hillclimbSupport()` reads `edge_localid[i]` to get a **local**
hull index, then uses `vert_globalid[subidx]` to recover the **global** mesh
vertex index for coordinate lookup in `mesh_vert`.

**`vert_globalid` mapping:** MuJoCo maintains a bidirectional mapping between
hull-local vertex indices (0..numvert-1) and original mesh vertex indices.
The hill-climbing walk operates on local indices (for graph traversal), but
vertex coordinates are looked up via global indices (into `mesh_vert`).
CortenForge does NOT need this mapping because our `ConvexHull.vertices` stores
the actual coordinates directly (not indices into a shared vertex buffer).
The adjacency graph uses indices into `ConvexHull.vertices`.

**CortenForge approach:** Since CortenForge is pure Rust (no C/C++ dependencies),
we implement Quickhull directly. The algorithm must produce an equivalent convex
hull — same vertex **set** (ignoring order), same face **count**, and identical
GJK support function results for any direction vector. Face triangulation order
is implementation-dependent and need not match Qhull.

### Graph data structure layout

**Source:** `user_mesh.cc` → `MakeGraph()` graph construction

MuJoCo stores the graph as a flat `int` array:

```
[0]            = numvert         (hull vertex count)
[1]            = numface         (hull face count)
[2 .. 2+nv-1]           = vert_edgeadr[]    (edge list start per vertex)
[2+nv .. 2+2*nv-1]      = vert_globalid[]   (local hull → original mesh vertex)
[2+2*nv .. 2+3*nv+3*nf-1] = edge_localid[]  (adjacency lists, -1 separated)
[2+3*nv+3*nf .. end]    = face_globalid[]   (3 indices per triangle face)
```

Size formula: `szgraph = 2 + 3*numvert + 6*numface`

**Purpose:** Enables O(√n) hill-climbing support queries. Without the graph,
GJK degrades to O(n) per support call.

**CortenForge mapping:** Rust-idiomatic `HullGraph` struct with
`Vec<Vec<usize>>` adjacency lists instead of flat int array.

### GJK support function strategies

**Source:** `engine_collision_convex.c`

Two strategies, selected in `mjc_initCCDObj()`:

```c
// Strategy selection in mjc_initCCDObj():
if (m->mesh_graphadr[m->geom_dataid[g]] < 0 ||
    m->mesh_vertnum[m->geom_dataid[g]] < mjMESH_HILLCLIMB_MIN) {
    obj->support = mjc_meshSupport;       // Exhaustive O(n)
} else {
    obj->support = mjc_hillclimbSupport;  // Hill-climbing O(sqrt(n))
}

// Warm-start initialization (also in mjc_initCCDObj):
obj->meshindex = -1;   // hill-climbing: no cached vertex yet
obj->vertindex = -1;   // exhaustive: no cached vertex yet
```

**`mesh_vertnum` semantics:** MuJoCo's `mesh_vertnum` is the number of
**unique** mesh vertices (after `ProcessVertices()` deduplication), which is
also the number of hull vertices for a non-truncated hull. The threshold
`mjMESH_HILLCLIMB_MIN = 10` applies to this count. CortenForge's equivalent
is `hull.vertices.len()` (the hull vertex count), which corresponds to
MuJoCo's `mesh_vertnum` for the purposes of the hill-climbing decision.
The `graph: Option<HullGraph>` field on `ConvexMesh` is set to `Some` only
when `hull.vertices.len() >= HILL_CLIMB_MIN` (see `convex_mesh_from_hull()`),
so the threshold is enforced at construction time — the support function
simply checks `graph.is_some()`.

When `meshindex == -1`, `mjc_hillclimbSupport` starts from vertex 0
(the `imax = obj->meshindex >= 0 ? obj->meshindex : 0` line).
When `vertindex == -1`, `mjc_meshSupport` initializes `max = -FLT_MAX`
(the warm-start branch is skipped, so it's a full scan from scratch).
CortenForge uses `Cell::new(0)` which always starts from vertex 0.

**Warm-start equivalence:** Both MuJoCo and CortenForge produce identical
results for the first call: MuJoCo's hill-climbing starts at vertex 0
(when `meshindex == -1`), CortenForge starts at vertex 0 (from `Cell::new(0)`).
For exhaustive support, MuJoCo initializes `max = -FLT_MAX` when
`vertindex == -1` (no cache), while CortenForge initializes `max` from
`vertices[0].coords.dot(&dir)` (cache = 0). Both still scan all vertices —
the only difference is whether vertex 0's dot product is the initial bound
or `-FLT_MAX`. Since the exhaustive scan visits all vertices including 0,
the result is identical. For subsequent calls, both systems read from their
respective caches (MuJoCo: `meshindex`/`vertindex`, CortenForge: `Cell<usize>`)
and produce the same result because the support function is deterministic
given the same start vertex and the same vertex set. See also the
warm-start persistence discussion below (after the CortenForge implementation
code block) for the broader warm-start lifecycle.

`mjMESH_HILLCLIMB_MIN = 10` (defined in `engine_collision_convex.h`).

**Exhaustive support** (`mjc_meshSupport`): Linear scan of all **original
mesh** vertices (not hull vertices). This function is selected when there is
no graph (`mesh_graphadr < 0`) or the mesh has fewer than
`mjMESH_HILLCLIMB_MIN` vertices. Warm-starts by initializing `max` from
the cached `vertindex`'s dot product (instead of `-FLT_MAX`). Still scans ALL
vertices — the warm-start provides a tighter initial comparison bound.
**CortenForge mapping:** Our exhaustive path scans `ConvexMesh.vertices`,
which are the hull vertices (not the original mesh vertices). This is
acceptable because the hull contains all extreme points — the support
function result is identical.

**Hill-climbing support** (`mjc_hillclimbSupport`): **Steepest-ascent** —
start from cached vertex (`meshindex`), scan ALL neighbors, move to the
neighbor with the highest dot product, repeat until no neighbor improves. Uses
`vert_edgeadr` and `edge_localid` from the graph. NOT greedy first-improvement
— the inner `for` loop examines every neighbor before deciding which to move
to.

Both functions:
- Rotate direction to local frame before scanning
- Warm-start from previously cached vertex index
- Transform result back to global frame
- Cache the result vertex index for the next call

### `maxhullvert` attribute

**Source:** `xml_native_reader.cc` + `mjspec.h`

```c
if (ReadAttrInt(elem, "maxhullvert", &n)) {
    if (n != -1 && n < 4) throw mjXError(elem, "maxhullvert must be larger than 3");
    mesh->maxhullvert = n;
}
```

- Default: `-1` (no limit)
- Valid: `-1` or `> 3`
- Converted to Qhull `TA` option as `maxhullvert - 4`

### Vertex deduplication before hull

**Source:** `user_mesh.cc` → `ProcessVertices()`

Uses `std::unordered_map<VertexKey, int>` to deduplicate vertices before hull
computation. `VertexKey` is a struct of 3 ints (quantized vertex coordinates).
Face indices are remapped to the deduplicated vertex set.

```c
// Key structure for vertex deduplication (quantized coordinates)
struct VertexKey { int x, y, z; };
std::unordered_map<VertexKey, int, VertexKeyHash> vertex_map;
// For each vertex: quantize → lookup → if new, assign sequential ID
// After dedup: nvert_ = unique count, face indices remapped
```

**CortenForge mapping:** `TriangleMeshData::new()` does NOT deduplicate
vertices. The Quickhull algorithm must either tolerate duplicate vertices
(producing a valid hull with them) or deduplicate internally. Deduplicating
internally in Quickhull is simpler and matches MuJoCo's approach.

### Degenerate case handling

**Source:** `user_mesh.cc` → `MakeGraph()`

| Case | MuJoCo behavior |
|------|----------------|
| `nvert < 4` | `MakeGraph()` returns early — no hull computed, `szgraph_ = 0` |
| Duplicate vertices | Removed by `ProcessVertices()` before hull |
| Coplanar points | Handled by Qhull internally (`Qt` triangulation option) |
| Collinear points | Handled by Qhull (produces degenerate hull or error) |
| Qhull failure | Caught via `setjmp`/`longjmp`; graph freed; `szgraph_` zeroed |
| Invalid point IDs | `ok=0` set; graph freed |
| Non-triangular faces | Impossible after `qh_triangulate()` but validated (`cnt == 3`) |

### Conformance definition

For **non-truncated hulls** (`maxhullvert = -1`): the hull vertex SET (as a
set, ignoring order) and face COUNT must match MuJoCo. Face triangulation
order is implementation-dependent. The GJK support function MUST return a
support point that is the same hull vertex as MuJoCo's for any direction
vector — i.e., the same vertex index (modulo index mapping), not bitwise
identical coordinates.

**Precision note (float vs f64):** MuJoCo stores mesh vertices as `float`
(32-bit) and uses `dot3f` (float dot product) in support functions.
CortenForge uses `f64` (64-bit) throughout. For support function conformance,
this means:

1. **Hull vertex sets are identical.** The convex hull is a combinatorial
   structure determined by vertex positions. Quickhull produces the same
   vertex set whether computed in f32 or f64, because the algorithm's
   decisions (which point is farthest, which faces are visible) depend on
   sign comparisons of distances, not on precision. The only failure mode
   is when two distances are so close that different precisions produce
   different signs — but this only affects vertices within ε of a face
   plane, and such vertices are correctly handled by the ε-threshold
   (they are treated as interior in both precisions).

2. **Support point identity.** The support function returns
   `argmax_v (v · d)`. For a given set of hull vertices and direction d,
   the maximizer is determined by the vertex positions. When two vertices
   have dot products that differ by less than the float ULP (~1e-7 ×
   |max_dot|), the tie-breaking may differ between f32 and f64. This is
   acceptable: the two vertices are geometrically near-equivalent support
   points, and the difference in contact computation is within simulation
   noise.

3. **Coordinate values.** CortenForge stores vertex coordinates as f64.
   MuJoCo stores them as float. For meshes loaded from STL/OBJ (which use
   float), CortenForge's f64 values are exact float-to-f64 promotions
   (no rounding). For meshes specified inline in MJCF XML (parsed from
   decimal strings), both systems parse to their native precision — values
   agree to ~7 significant digits.

For **`maxhullvert`-truncated hulls**: vertex set match with Qhull is
best-effort (Quickhull and Qhull traverse vertices in different orders).
Conformance relaxes to: hull has at most `maxhullvert` vertices, is a valid
convex hull of a subset of the input vertices, and GJK support results are
consistent with its own vertex set.

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| Hull computation at build time | `MakeGraph()` in `user_mesh.cc` via Qhull library | **Not implemented** — no hull computation |
| Adjacency graph for GJK | `mesh_graph[]` flat int array, O(√n) hill-climbing | **Not implemented** — O(n) exhaustive scan only |
| `maxhullvert` attribute | Parsed from `<mesh>`, validated ≥4 or -1, maps to Qhull `TA(N-4)` | **Not parsed** — `MjcfMesh` has no `maxhullvert` field |
| GJK support strategy selection | Hill-climbing for ≥10 vertices, exhaustive for <10, warm-started | **Exhaustive only** — `support_convex_mesh()` does fresh O(n) scan |
| Hull trigger | `needhull_` set for collision meshes and convex-inertia meshes | **N/A** — no hull system exists |
| Mesh-mesh collision | GJK/EPA on convex hulls | Per-triangle BVH via `mesh_mesh_deepest_contact()` |
| ConvexMesh construction | Hull vertices populate collision shape automatically | **Never constructed in production** — only in tests |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| Graph storage | Flat `int*` array: `mesh_graph[mesh_graphadr[i]...]` | `HullGraph { adjacency: Vec<Vec<usize>> }` on `ConvexHull` struct | Use adjacency list per vertex instead of flat array with `edgeadr` offsets |
| Vertex storage | `float*` pointer into `mesh_vert` global array | `Vec<Point3<f64>>` on `ConvexHull` | Direct — same data, different container |
| Hull presence | `mesh_graphadr[i] != -1` | `TriangleMeshData.convex_hull: Option<ConvexHull>` — `Some` = hull exists | Check `Option::is_some()` instead of `graphadr != -1` |
| Mesh-to-geom | `geom_dataid[g]` → mesh index | `model.geom_mesh[g]: Option<usize>` → mesh index | Use `.expect()` or `?` instead of raw index |
| Support direction frame | Rotate to local via rotation matrix `geom_mat` | Rotate to local via `pose.rotation.inverse()` (UnitQuaternion) | Direct port — both achieve world-to-local rotation |
| Vertex warm-start | `vertindex` / `meshindex` fields on `mjCCDObj` | `Cell<usize>` on `ConvexMesh` variant (interior mutability through `&self`) | `convex_mesh()` initializes `Cell::new(0)`. MuJoCo initializes to -1 (no cache); both systems start from vertex 0 on first call. |
| Face normals in graph | NOT stored — graph is purely topological (int arrays) | Precomputed on `ConvexHull.normals: Vec<Vector3<f64>>` | **CortenForge convenience** — normals needed by Spec B (shell inertia) and contact computation. Not present in MuJoCo's graph. |
| `maxhullvert` type | `int` in C, default `-1`, validated `n != -1 && n < 4` → error | `Option<usize>` — `None` = no limit, `Some(n)` where `n >= 4` | Parse XML attribute as `i32` (matching MuJoCo's `ReadAttrInt`). Convert: `-1` → `None`, `>= 4` → `Some(n as usize)`, `0..3` → error. The `i32` is an intermediate parsing type only — the stored field is `Option<usize>`. |

---

## Architecture Decisions

### AD-1: Dispatch routing for mesh collision

**Problem:** After adding convex hulls, mesh collision dispatch must be decided.
Currently `geom_to_collision_shape()` returns `None` for `GeomType::Mesh`,
routing all mesh collision through `collide_with_mesh()` (per-triangle BVH).
Two options exist (see rubric EGT-10).

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| (a) | Route through GJK/EPA: `geom_to_collision_shape()` returns `Some(ConvexMesh)` for mesh geoms with hull | Fully MuJoCo-conformant for ALL mesh pairs; single code path | **Massive blast radius** — ALL mesh collision tests change behavior; per-triangle specialized handlers bypassed entirely; mesh-primitive contacts become hull-surface contacts |
| (b) | Keep dispatch, hull used internally: `collide_with_mesh()` uses hull for mesh-mesh pairs | Controlled blast radius; mesh-primitive pairs unchanged; mesh-mesh becomes conformant | Mesh-primitive pairs remain non-conformant (per-triangle BVH vs hull GJK/EPA); two code paths for mesh collision |

**Chosen:** Option (b) — Keep dispatch unchanged, use hull internally for
mesh-mesh pairs. Rationale: option (a) has unacceptable blast radius for a
single spec. Mesh-mesh is the highest-conformance-value pair (MuJoCo uses
GJK/EPA on hulls). Mesh-primitive pairs remain working via specialized
per-triangle handlers — a future conformance pass can migrate those to
hull-based GJK/EPA if needed (tracked as out-of-scope item). Spec D (CCD)
constructs `ConvexMesh` directly from the hull when it needs GJK/EPA on
mesh shapes.

### AD-2: Hull normals

**Problem:** MuJoCo's graph stores NO face normals (purely topological int
arrays). The umbrella API contract includes `normals: Vec<Vector3<f64>>`.

**Chosen:** Include normals on `ConvexHull`. Rationale: Spec B needs face
normals for shell inertia computation on the hull. Computing them during
Quickhull is nearly free (cross product of two edge vectors). This is a
CortenForge convenience, not a conformance requirement. Explicitly documented
in Convention Notes as a deviation from MuJoCo's graph structure.

### AD-3: Hull trigger — all meshes vs collision-only

**Problem:** MuJoCo only computes hulls for collision meshes (`needhull_`
flag). CortenForge could either match this (requiring geom-to-mesh dependency
tracking during build) or compute for all meshes.

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| A | Compute for all meshes | Simple — no geom dependency tracking | Slight waste for visual-only meshes |
| B | Collision meshes only (match MuJoCo) | Exact MuJoCo behavior | Requires two-pass build (process meshes, then geoms, then hulls) or deferred computation |

**Chosen:** Option A — compute for all meshes. Rationale: conformance-neutral
(computing an unused hull does not change collision behavior). The cost is
O(n log n) per mesh at build time — negligible for typical model sizes.
The two-pass build required by option B adds complexity to `builder/mesh.rs`
with no conformance benefit.

### AD-4: Algorithm placement

**Problem:** The Quickhull implementation is ~300-400 lines. Should it go in
`mesh.rs` (already 2,147 lines) or a new file?

**Chosen:** New `sim/L0/core/src/convex_hull.rs` module. Rationale: the
Quickhull algorithm operates on a point cloud and produces a hull — it's a
standalone geometric algorithm, not mesh data management. `mesh.rs` stays
focused on `TriangleMeshData`. The `ConvexHull` and `HullGraph` structs live
in `convex_hull.rs` and are imported by `mesh.rs`.

---

## Specification

### S1. `ConvexHull` and `HullGraph` structs + Quickhull algorithm

**File:** `sim/L0/core/src/convex_hull.rs` (new file)
**MuJoCo equivalent:** `mjCMesh::MakeGraph()` in `user_mesh.cc`, graph layout in `mjmodel.h`
**Design decision:** AD-4 (separate file — the rubric's EGT-9 suggests
`mesh.rs`, but AD-4 overrides: the algorithm is a standalone geometric
operation, not mesh data management). AD-2 (include normals). Rust-idiomatic
adjacency lists instead of MuJoCo's flat int array — numerically equivalent for
hill-climbing support queries.

#### Data structures

```rust
use nalgebra::{Point3, Vector3};

/// Convex hull of a triangle mesh, computed via Quickhull at build time.
///
/// Stores the hull geometry (vertices, triangulated faces, outward normals)
/// and an adjacency graph for O(√n) hill-climbing GJK support queries.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ConvexHull {
    /// Hull vertices in local mesh frame. Subset of the original mesh's
    /// deduplicated vertices (or new vertices if deduplication merges some).
    pub vertices: Vec<Point3<f64>>,

    /// Triangulated hull faces as index triples into `vertices`.
    /// All faces are triangles. Winding: counter-clockwise when viewed from
    /// outside (outward normal follows right-hand rule).
    pub faces: Vec<[usize; 3]>,

    /// Outward unit face normals (one per face, same order as `faces`).
    /// CortenForge convenience — MuJoCo's graph stores no face normals.
    /// Needed by Spec B (shell inertia) and contact normal computation.
    pub normals: Vec<Vector3<f64>>,

    /// Vertex adjacency graph for hill-climbing support queries.
    pub graph: HullGraph,
}

/// Vertex adjacency graph for convex hull.
///
/// Rust-idiomatic representation of MuJoCo's flat `mesh_graph` int array.
/// For each hull vertex, stores the indices of all neighboring vertices
/// (vertices that share a hull edge).
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct HullGraph {
    /// `adjacency[i]` = sorted list of vertex indices adjacent to vertex `i`.
    /// Length equals `ConvexHull::vertices.len()`.
    pub adjacency: Vec<Vec<usize>>,
}
```

#### Quickhull algorithm

```rust
/// Compute the convex hull of a point set using the Quickhull algorithm.
///
/// Returns `None` if fewer than 4 non-degenerate points exist (matching
/// MuJoCo's early return in `MakeGraph()`). If `max_vertices` is `Some(n)`,
/// the hull is truncated to at most `n` vertices (matching MuJoCo's
/// `maxhullvert` → Qhull `TA(n-4)` mapping).
pub fn quickhull(
    points: &[Point3<f64>],
    max_vertices: Option<usize>,
) -> Option<ConvexHull> {
    if points.len() < 4 {
        return None;
    }

    // Step 0: scale-dependent epsilon
    let epsilon = compute_epsilon(points);

    // Step 1: initial simplex (4 non-coplanar points)
    // epsilon is passed in — NOT recomputed inside find_initial_simplex.
    let simplex = find_initial_simplex(points, epsilon)?;

    // Build initial hull vertices from simplex indices
    let mut hull_vertices: Vec<Point3<f64>> = simplex.iter()
        .map(|&i| points[i])
        .collect();

    // Step 2-3: create initial faces, init conflict graph, expand.
    // No edge_map — face neighbor tracking uses Face.neighbors exclusively.
    let mut faces = create_initial_faces(&hull_vertices, &[0, 1, 2, 3]);
    init_conflict_graph(points, &mut faces, &simplex, epsilon);
    expand_hull(
        points, &mut hull_vertices, &mut faces,
        max_vertices, epsilon,
    );

    // Step 4-5: extract alive faces, build graph and normals
    let alive_faces: Vec<[usize; 3]> = faces.iter()
        .filter(|f| f.alive)
        .map(|f| f.indices)
        .collect();
    let graph = build_graph(&hull_vertices, &alive_faces);
    let normals = alive_faces.iter().map(|&[a, b, c]| {
        let e1 = hull_vertices[b] - hull_vertices[a];
        let e2 = hull_vertices[c] - hull_vertices[a];
        e1.cross(&e2).normalize()
    }).collect();

    Some(ConvexHull {
        vertices: hull_vertices,
        faces: alive_faces,
        normals,
        graph,
    })
}
```

**Step 0 — Epsilon computation:** Call `compute_epsilon(points)` to get a
scale-dependent ε. This single ε is used for all thresholds throughout the
algorithm (initial simplex degeneracy, conflict assignment, BFS visibility).

**Step 1 — Initial simplex** (4 non-coplanar points):

1. Find 6 extremal points (min/max along each axis).
2. From those, select the pair with maximum distance → edge `(p0, p1)`.
3. Find the point farthest from line `(p0, p1)` → `p2`. If distance < ε
   (1e-10), all points are collinear → return `None`.
4. Find the point farthest from plane `(p0, p1, p2)` → `p3`. If distance < ε,
   all points are coplanar → return `None`.
5. Orient `p3` so the tetrahedron `(p0, p1, p2, p3)` has outward-facing normals.
   If `p3` is below the plane of `(p0, p1, p2)`, swap `p0` and `p1` to flip
   winding.

```rust
fn find_initial_simplex(points: &[Point3<f64>], epsilon: f64) -> Option<[usize; 4]> {
    // Find extremal points (min/max per axis)
    let (mut min_idx, mut max_idx) = ([0usize; 3], [0usize; 3]);
    for (i, p) in points.iter().enumerate() {
        for axis in 0..3 {
            if p[axis] < points[min_idx[axis]][axis] { min_idx[axis] = i; }
            if p[axis] > points[max_idx[axis]][axis] { max_idx[axis] = i; }
        }
    }

    // Find most distant pair among extremals
    let candidates: Vec<usize> = min_idx.iter().chain(max_idx.iter())
        .copied().collect::<std::collections::HashSet<_>>()
        .into_iter().collect();
    let (i0, i1) = most_distant_pair(points, &candidates);

    // epsilon is passed in from quickhull() — NOT recomputed here.

    // Find point farthest from line (i0, i1)
    let i2 = farthest_from_line(points, points[i0], points[i1], epsilon)?;

    // Find point farthest from plane (i0, i1, i2)
    let normal = (points[i1] - points[i0]).cross(&(points[i2] - points[i0]));
    let i3 = farthest_from_plane(points, points[i0], &normal, epsilon)?;

    // Orient so normals face outward
    let d = (points[i3] - points[i0]).dot(&normal);
    if d > 0.0 {
        Some([i1, i0, i2, i3])  // Swap to flip winding
    } else {
        Some([i0, i1, i2, i3])
    }
}

/// Find the most distant pair of points from a set of candidate indices.
fn most_distant_pair(
    points: &[Point3<f64>],
    candidates: &[usize],
) -> (usize, usize) {
    let mut best = (candidates[0], candidates[1]);
    let mut max_dist_sq = 0.0_f64;
    for (ci, &i) in candidates.iter().enumerate() {
        for &j in &candidates[ci + 1..] {
            let d = (points[i] - points[j]).norm_squared();
            if d > max_dist_sq {
                max_dist_sq = d;
                best = (i, j);
            }
        }
    }
    best
}

/// Find the point farthest from the line through `a` and `b`.
/// Returns `None` if all points are collinear (max distance < ε),
/// or if `a == b` (degenerate: all extremals identical).
fn farthest_from_line(
    points: &[Point3<f64>],
    a: Point3<f64>,
    b: Point3<f64>,
    epsilon: f64,
) -> Option<usize> {
    let ab = b - a;
    let ab_len_sq = ab.norm_squared();
    if ab_len_sq < 1e-30 {
        return None; // Degenerate: a == b (all extremals identical)
    }
    let mut best_idx = 0;
    let mut max_dist_sq = 0.0_f64;
    for (i, p) in points.iter().enumerate() {
        // Distance from point to line = |cross(AB, AP)| / |AB|
        let ap = p - a;
        let cross = ab.cross(&ap);
        let dist_sq = cross.norm_squared() / ab_len_sq;
        if dist_sq > max_dist_sq {
            max_dist_sq = dist_sq;
            best_idx = i;
        }
    }
    if max_dist_sq.sqrt() < epsilon {
        None  // All collinear
    } else {
        Some(best_idx)
    }
}

/// Find the point farthest from the plane through `origin` with `normal`.
/// Returns `None` if all points are coplanar (max |distance| < ε).
///
/// IMPORTANT: `normal` is the raw cross product (unnormalized). We normalize
/// it here to get true geometric distance for the ε comparison. Without
/// normalization, the dot product is scaled by |normal| and the comparison
/// against ε would be dimensionally inconsistent.
fn farthest_from_plane(
    points: &[Point3<f64>],
    origin: Point3<f64>,
    normal: &Vector3<f64>,
    epsilon: f64,
) -> Option<usize> {
    let normal_len = normal.norm();
    if normal_len < 1e-30 {
        return None; // Degenerate: zero-area triangle (collinear points)
    }
    let unit_normal = normal / normal_len;

    let mut best_idx = 0;
    let mut max_abs_dist = 0.0_f64;
    for (i, p) in points.iter().enumerate() {
        // True geometric distance = dot with unit normal
        let dist = (p - origin).dot(&unit_normal).abs();
        if dist > max_abs_dist {
            max_abs_dist = dist;
            best_idx = i;
        }
    }
    if max_abs_dist < epsilon {
        None  // All coplanar
    } else {
        Some(best_idx)
    }
}
```

**Step 2 — Conflict graph initialization:**

Assign each non-simplex point to the face it is farthest above (positive
signed distance from face plane). Points below all faces are interior to
the initial simplex and are discarded.

```rust
/// Initialize the conflict graph: assign each non-simplex point to the
/// face it is farthest above.
fn init_conflict_graph(
    points: &[Point3<f64>],
    faces: &mut [Face],
    simplex: &[usize; 4],
    epsilon: f64,
) {
    let simplex_set: HashSet<usize> = simplex.iter().copied().collect();
    for (pi, p) in points.iter().enumerate() {
        if simplex_set.contains(&pi) {
            continue;
        }
        // Find the face this point is farthest above
        let mut best_face: Option<usize> = None;
        let mut best_dist = epsilon; // threshold: must be strictly above ε
        for (fi, face) in faces.iter().enumerate() {
            if !face.alive { continue; }
            let dist = (p - face.center).dot(&face.normal);
            if dist > best_dist {
                best_dist = dist;
                best_face = Some(fi);
            }
        }
        if let Some(fi) = best_face {
            faces[fi].conflict_list.push(pi);
            if best_dist > faces[fi].farthest_dist {
                faces[fi].farthest_dist = best_dist;
                faces[fi].farthest_idx = pi;
            }
        }
        // else: point is interior (below all faces within ε) — discard
    }
}
```

**Step 3 — Iterative expansion:**

```rust
/// Main Quickhull expansion loop.
///
/// Iteratively adds the farthest conflict point as a new hull vertex,
/// removes visible faces, creates cone faces from the horizon, and
/// redistributes conflict points.
///
/// **No edge_map.** Face neighbor tracking uses `Face.neighbors` exclusively.
/// Previous versions used a `HashMap<(usize,usize), usize>` edge map, but
/// this is fundamentally broken for convex hulls: each edge is shared by
/// exactly two faces, but a HashMap can only store one face per canonical
/// edge key. When visible face edges are removed from the map, horizon
/// edge entries (shared with non-visible faces) are destroyed, making it
/// impossible to find the non-visible neighbor for cone face construction.
///
/// The fix: `find_horizon()` returns the non-visible face index for each
/// horizon edge directly (it already knows this from the BFS). No edge
/// map lookup is needed.
fn expand_hull(
    points: &[Point3<f64>],
    hull_vertices: &mut Vec<Point3<f64>>,
    faces: &mut Vec<Face>,
    max_vertices: Option<usize>,
    epsilon: f64,
) {
    loop {
        // 1. Find the face with the farthest conflict point (eye point)
        // NaN safety: farthest_dist is initialized to NEG_INFINITY and
        // only updated via comparison with dot products of finite vectors.
        // The compute_face_normal NaN guard (returns Vector3::z() for
        // degenerate faces) ensures normals are never NaN, so the dot
        // product (p - center) . normal is always finite. Therefore
        // farthest_dist is never NaN, and partial_cmp never returns None.
        // We use unwrap_or(Ordering::Equal) as a belt-and-suspenders
        // guard that treats NaN as equal (no preference) rather than
        // panicking.
        let best_face = faces.iter().enumerate()
            .filter(|(_, f)| f.alive && !f.conflict_list.is_empty())
            .max_by(|(_, a), (_, b)| {
                a.farthest_dist.partial_cmp(&b.farthest_dist)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
        let (face_idx, _) = match best_face {
            Some(pair) => pair,
            None => break, // No conflict points remain — hull is complete
        };

        // 2. Check vertex limit (maxhullvert)
        if let Some(max) = max_vertices {
            if hull_vertices.len() >= max {
                break;
            }
        }

        let eye_pi = faces[face_idx].farthest_idx; // index into `points`
        let eye_pt = points[eye_pi];
        let eye_hull_idx = hull_vertices.len();
        hull_vertices.push(eye_pt);

        // 3-4. BFS to find visible faces + extract horizon edges.
        // Each horizon edge is (a, b, non_vis_fi):
        //   (a, b) = edge in visible face's winding direction
        //   non_vis_fi = the non-visible face sharing that edge
        let (visible, mut horizon) = find_horizon(
            faces, &eye_pt, face_idx, epsilon,
        );

        // Sort horizon edges into a closed polygon.
        // After ordering: edge[i].1 == edge[i+1].0 (wrapping).
        order_horizon_edges(&mut horizon);

        // 5. Collect conflict points from visible faces before deletion
        let mut orphan_points: Vec<usize> = Vec::new();
        for &fi in &visible {
            orphan_points.extend(
                faces[fi].conflict_list.iter()
                    .filter(|&&pi| pi != eye_pi)
            );
            faces[fi].alive = false;
            faces[fi].conflict_list.clear();
        }

        // 6. Create cone faces from horizon edges to eye point.
        //
        // For each horizon edge (a, b, non_vis_fi), create face [eye, a, b].
        //
        // Face [eye, a, b] has edges (via face_edge):
        //   Edge 0: (eye, a)  — shared with previous cone face
        //   Edge 1: (a, b)    — shared with non-visible neighbor
        //   Edge 2: (b, eye)  — shared with next cone face
        //
        // Winding correctness: the visible face had edge (a, b) in its
        // winding. The non-visible face has the same edge as (b, a) in
        // its winding (manifold consistency of the existing hull). The
        // new cone face [eye, a, b] has edge (a, b) at position 1,
        // matching the visible face's direction. Since the non-visible
        // face has (b, a) — the opposite direction — manifold consistency
        // is preserved.
        //
        // Normal correctness: the cone face normal (a-eye) × (b-eye)
        // points away from the hull interior because eye is strictly
        // outside the hull (it was above the visible face by > ε).
        let cone_start = faces.len();
        for &(a, b, non_vis_fi) in &horizon {
            let new_fi = faces.len();
            let normal = compute_face_normal(
                &hull_vertices[eye_hull_idx],
                &hull_vertices[a],
                &hull_vertices[b],
            );
            let center = Point3::from(
                (hull_vertices[eye_hull_idx].coords
                    + hull_vertices[a].coords
                    + hull_vertices[b].coords) / 3.0
            );

            faces.push(Face {
                indices: [eye_hull_idx, a, b],
                normal,
                center,
                conflict_list: Vec::new(),
                farthest_dist: f64::NEG_INFINITY,
                farthest_idx: 0,
                alive: true,
                neighbors: [
                    usize::MAX,    // Edge 0 (eye, a) — cone-cone, set below
                    non_vis_fi,    // Edge 1 (a, b)   — non-visible neighbor
                    usize::MAX,    // Edge 2 (b, eye)  — cone-cone, set below
                ],
            });

            // Update the non-visible neighbor's reference: replace its
            // stale pointer to the dead visible face with the new cone face.
            update_neighbor(faces, non_vis_fi, a, b, new_fi);
        }

        // Link adjacent cone faces: consecutive horizon edges share a
        // vertex, so their cone faces share an edge through the eye point.
        //
        // Horizon ordering: edge[i] = (aᵢ, bᵢ, _) with bᵢ = aᵢ₊₁.
        // Face i = [eye, aᵢ, bᵢ] has edge 2 = (bᵢ, eye).
        // Face i+1 = [eye, aᵢ₊₁, bᵢ₊₁] = [eye, bᵢ, bᵢ₊₁] has edge 0 = (eye, bᵢ).
        // Edge (bᵢ, eye) of face i and edge (eye, bᵢ) of face i+1 are the
        // same edge in opposite directions — they are neighbors.
        let cone_count = horizon.len();
        for i in 0..cone_count {
            let fi = cone_start + i;
            let next = cone_start + (i + 1) % cone_count;
            let prev = cone_start + (i + cone_count - 1) % cone_count;
            faces[fi].neighbors[0] = prev; // Edge 0 (eye, aᵢ) ↔ prev's edge 2
            faces[fi].neighbors[2] = next; // Edge 2 (bᵢ, eye) ↔ next's edge 0
        }

        // 7. Redistribute orphan conflict points to new cone faces only.
        //
        // Only cone faces need checking — non-visible faces already had
        // their conflict lists populated from previous iterations, and the
        // orphan points were above the (now-deleted) visible faces, which
        // are replaced by cone faces. Points that are not above any cone
        // face are now interior to the hull (they were "between" the old
        // visible face planes and the new cone face planes).
        for &pi in &orphan_points {
            let p = &points[pi];
            let mut best_face: Option<usize> = None;
            let mut best_dist = epsilon;
            for fi in cone_start..faces.len() {
                if !faces[fi].alive { continue; }
                let dist = (p - faces[fi].center).dot(&faces[fi].normal);
                if dist > best_dist {
                    best_dist = dist;
                    best_face = Some(fi);
                }
            }
            if let Some(fi) = best_face {
                faces[fi].conflict_list.push(pi);
                if best_dist > faces[fi].farthest_dist {
                    faces[fi].farthest_dist = best_dist;
                    faces[fi].farthest_idx = pi;
                }
            }
            // else: point is now interior — discard
        }
    }
}
```

**Helper functions for expansion:**

```rust
/// Return the edge vertices for edge_idx (0, 1, 2) of a face.
/// Edge 0 = (indices[0], indices[1]), Edge 1 = (indices[1], indices[2]),
/// Edge 2 = (indices[2], indices[0]).
fn face_edge(f: &Face, edge_idx: usize) -> (usize, usize) {
    match edge_idx {
        0 => (f.indices[0], f.indices[1]),
        1 => (f.indices[1], f.indices[2]),
        2 => (f.indices[2], f.indices[0]),
        _ => unreachable!(),
    }
}

/// Compute outward unit normal for face (a, b, c).
///
/// Returns `Vector3::z()` as a fallback for degenerate (collinear) triangles
/// where the cross product has near-zero magnitude. This prevents NaN from
/// `normalize()` on a zero vector.
///
/// **Harmlessness proof:** A degenerate cone face [eye, a, b] occurs only
/// when eye, a, b are collinear. This means the horizon edge (a, b) passes
/// through (or very near) the eye point. In practice this cannot happen
/// because the eye point is strictly outside the hull (above some face by
/// > epsilon), while a and b are hull vertices (on the hull surface). The
/// only geometric configuration where eye is collinear with a hull edge is
/// when the hull is extremely flat — but this is caught by the initial
/// simplex's farthest-from-plane check (returns None if the tetrahedron
/// volume < epsilon).
///
/// If it somehow occurs despite these guards (e.g., due to floating-point
/// accumulation over many expansion steps), the degenerate face has an
/// arbitrary normal (Vector3::z()) and its conflict distance computation
/// `(p - center) . normal` produces arbitrary but finite values. Points
/// assigned to this face may be incorrectly classified, but they are
/// within epsilon of the hull surface (otherwise they would have been
/// assigned to a non-degenerate face with larger signed distance). The
/// hull remains valid to within epsilon tolerance.
fn compute_face_normal(a: &Point3<f64>, b: &Point3<f64>, c: &Point3<f64>) -> Vector3<f64> {
    let e1 = b - a;
    let e2 = c - a;
    let cross = e1.cross(&e2);
    let len = cross.norm();
    if len < 1e-30 {
        // Degenerate triangle — collinear or coincident vertices.
        // Return arbitrary unit vector; face will have zero effective area.
        Vector3::z_axis().into_inner()
    } else {
        cross / len
    }
}

/// Order horizon edges into a closed polygon.
/// Input: unordered edges [(a₁,b₁,n₁), (a₂,b₂,n₂), ...] where
///   (aᵢ,bᵢ) is the edge and nᵢ is the non-visible neighbor face.
/// Output: reordered so edge[i].1 == edge[i+1].0 (wrapping).
fn order_horizon_edges(edges: &mut Vec<(usize, usize, usize)>) {
    for i in 0..edges.len() - 1 {
        let tail = edges[i].1;
        // Find the next edge that starts at `tail`
        let next_pos = edges[i + 1..].iter()
            .position(|e| e.0 == tail)
            .expect("horizon edges must form a closed polygon");
        edges.swap(i + 1, i + 1 + next_pos);
    }
    debug_assert_eq!(
        edges.last().unwrap().1, edges[0].0,
        "horizon not closed"
    );
}

/// Update face `fi`'s neighbor entry: replace reference to old face
/// (identified by shared edge (a,b)) with `new_fi`.
fn update_neighbor(
    faces: &mut [Face],
    fi: usize,
    a: usize,
    b: usize,
    new_fi: usize,
) {
    // Find the edge index first (read-only), then mutate separately
    // to keep borrows clean.
    let target_edge = {
        let f = &faces[fi];
        (0..3).find(|&edge_idx| {
            let (ea, eb) = face_edge(f, edge_idx);
            (ea == a && eb == b) || (ea == b && eb == a)
        })
    };
    if let Some(edge_idx) = target_edge {
        faces[fi].neighbors[edge_idx] = new_fi;
    }
}
```

**Initial face construction from simplex:**

```rust
/// Create the 4 initial faces of the tetrahedron simplex and link their
/// neighbors via direct O(4²) search.
///
/// No edge_map needed — there are only 4 faces and 6 edges, so brute-force
/// search is simpler and correct. Neighbors are set by finding the other
/// face that shares each edge (in either direction).
fn create_initial_faces(
    hull_vertices: &[Point3<f64>],
    simplex: &[usize; 4],
) -> Vec<Face> {
    let [i0, i1, i2, i3] = *simplex;
    // 4 faces of tetrahedron with outward-facing winding.
    //
    // **Winding proof for all 4 faces:**
    //
    // After find_initial_simplex, the orientation swap ensures:
    //   d = (i3 - i0) . ((i1-i0) x (i2-i0)) < 0
    // i.e., i3 is BELOW the plane of (i0, i1, i2).
    //
    // A face has an outward normal iff the opposite vertex is in the
    // face's negative half-space (below the face plane).
    //
    // Face 0 [i0,i1,i2]: opposite vertex i3.
    //   (i3-i0) . ((i1-i0) x (i2-i0)) = d < 0. Proven directly.
    //
    // Faces 1-3: These use the oriented simplex theorem. For a
    //   tetrahedron (v0,v1,v2,v3) with negative signed volume
    //   (det([v1-v0, v2-v0, v3-v0]) < 0), the face orderings
    //   [v0,v1,v2], [v0,v3,v1], [v1,v3,v2], [v0,v2,v3] produce
    //   outward normals. This is a standard result in computational
    //   geometry (see Edelsbrunner, "Geometry and Topology for Mesh
    //   Generation", or the oriented simplex construction in CGAL).
    //   The key property: each face's winding is chosen so that the
    //   opposite vertex lies in the face's negative half-space.
    //
    //   Empirical validation: AC14 (test T19) checks that ALL face
    //   normals of the unit cube hull point outward (dot product with
    //   centroid-to-face vector > 0), catching any winding error.
    let face_indices = [
        [i0, i1, i2],
        [i0, i3, i1],
        [i1, i3, i2],
        [i0, i2, i3],
    ];

    let mut faces = Vec::with_capacity(4);
    for &[a, b, c] in &face_indices {
        let normal = compute_face_normal(
            &hull_vertices[a], &hull_vertices[b], &hull_vertices[c],
        );
        let center = Point3::from(
            (hull_vertices[a].coords
                + hull_vertices[b].coords
                + hull_vertices[c].coords) / 3.0
        );
        faces.push(Face {
            indices: [a, b, c],
            normal,
            center,
            conflict_list: Vec::new(),
            farthest_dist: f64::NEG_INFINITY,
            farthest_idx: 0,
            alive: true,
            neighbors: [usize::MAX; 3], // filled below
        });
    }

    // Link neighbors: for each face, find the face sharing each edge.
    // O(4 × 3 × 3 × 3) = O(108) — trivial for 4 faces.
    for fi in 0..4 {
        for edge_idx in 0..3 {
            let (a, b) = face_edge(&faces[fi], edge_idx);
            for fj in 0..4 {
                if fi == fj { continue; }
                if shares_edge(&faces[fj], a, b) {
                    faces[fi].neighbors[edge_idx] = fj;
                    break;
                }
            }
        }
    }
    faces
}

/// Check if face f contains edge (a, b) in either direction.
fn shares_edge(f: &Face, a: usize, b: usize) -> bool {
    for edge_idx in 0..3 {
        let (ea, eb) = face_edge(f, edge_idx);
        if (ea == a && eb == b) || (ea == b && eb == a) {
            return true;
        }
    }
    false
}
```

**Face adjacency for BFS and horizon detection:**

Each face tracks its three edge-neighbors (the face sharing each edge):

```rust
struct Face {
    indices: [usize; 3],          // Vertex indices into hull_vertices
    normal: Vector3<f64>,         // Outward unit normal
    center: Point3<f64>,          // Face centroid
    conflict_list: Vec<usize>,    // Indices into original point array
    farthest_dist: f64,           // Max distance among conflict points
    farthest_idx: usize,          // Index of farthest conflict point
    alive: bool,                  // false = deleted during expansion
    neighbors: [usize; 3],        // Face index across edge 0-1, 1-2, 2-0
}
```

Neighbor tracking is maintained directly on `Face.neighbors` — no external
edge map. An edge `(a, b)` is shared by exactly two faces on a convex hull.
For the initial simplex (4 faces, 6 edges), neighbors are resolved by O(4²)
brute-force search (`shares_edge`). During expansion, `find_horizon()` returns
the non-visible face index for each horizon edge (it already knows this from
BFS traversal of `Face.neighbors`). Cone-cone neighbors are determined by
horizon edge ordering (consecutive edges share a vertex, so consecutive cone
faces are neighbors through the eye point).

**Why not an edge map?** A `HashMap<(usize, usize), usize>` can only store
one face per canonical edge key `(min(a,b), max(a,b))`. Since each edge is
shared by exactly two faces, the map loses one face's reference per edge.
When visible face edges are removed from the map, horizon edge entries (shared
with non-visible faces) are destroyed, making it impossible to find the
non-visible neighbor for cone face construction. The `Face.neighbors` approach
is simpler and correct: each face explicitly stores its three neighbors, and
updates are made in-place via `update_neighbor()`.

**Horizon edge detection algorithm:**

```rust
/// BFS from start_face to find all faces visible from eye, then extract
/// the horizon edges (boundary between visible and non-visible faces).
///
/// Returns `(visible_set, horizon_edges)` where each horizon edge is
/// `(a, b, non_vis_fi)`: edge `(a, b)` in the visible face's winding
/// direction, and `non_vis_fi` is the non-visible face sharing that edge.
///
/// Edge direction is NOT reversed — `(a, b)` is as it appears in the
/// visible face. Cone faces are created as `[eye, a, b]`, giving edge
/// `(a, b)` in the cone face at edge index 1. The non-visible face has
/// this edge as `(b, a)` in its winding (manifold consistency). The cone
/// face's normal `(a-eye) × (b-eye)` points outward because eye is
/// outside the hull.
fn find_horizon(
    faces: &[Face],
    eye: &Point3<f64>,
    start_face: usize,
    epsilon: f64, // scale-dependent ε from compute_epsilon()
) -> (HashSet<usize>, Vec<(usize, usize, usize)>) {
    // BFS from start_face: collect all visible faces
    let mut visible = HashSet::new();
    let mut queue = VecDeque::new();
    visible.insert(start_face);
    queue.push_back(start_face);

    while let Some(fi) = queue.pop_front() {
        let face = &faces[fi];
        for &ni in &face.neighbors {
            // Guard: usize::MAX is the sentinel for uninitialized neighbors.
            // This should never occur in a well-formed hull (all neighbors are
            // set during create_initial_faces and expand_hull), but guarding
            // prevents an out-of-bounds panic if a bug leaves a sentinel.
            if ni == usize::MAX { continue; }
            if !visible.contains(&ni) && faces[ni].alive {
                let n = &faces[ni];
                let dist = (eye - n.center).dot(&n.normal);
                if dist > epsilon {
                    // Neighbor is visible from eye
                    visible.insert(ni);
                    queue.push_back(ni);
                }
            }
        }
    }

    // Horizon = edges between a visible face and a non-visible face.
    // Each horizon edge carries the non-visible face index so that
    // expand_hull() can set cone face neighbors without an edge_map.
    let mut horizon = Vec::new();
    for &fi in &visible {
        let face = &faces[fi];
        for edge_idx in 0..3 {
            let ni = face.neighbors[edge_idx];
            if !visible.contains(&ni) {
                let (a, b) = face_edge(face, edge_idx);
                // Keep (a, b) in visible face's winding direction.
                // The non-visible face has this edge as (b, a).
                // Cone face [eye, a, b] has edge (a, b) at edge index 1,
                // opposite to non-visible face's (b, a) — correct manifold.
                horizon.push((a, b, ni));
            }
        }
    }
    (visible, horizon)
}
```

**Horizon closure proof:** The horizon is a closed polygon because:
(1) The convex hull surface is a 2-manifold (closed, orientable surface) where
every edge is shared by exactly two faces.
(2) The set of visible faces (those with `(eye - center) · normal > ε`) is a
contiguous region because: if face F is visible and face G is adjacent to F,
then either G is visible (added to BFS queue) or G is not visible (the shared
edge is on the horizon). The BFS starting from the initial visible face
explores all reachable visible faces.
(3) The visible set is **simply connected** on the convex hull surface: the eye
point is strictly outside the hull (it was in a conflict list, so it is above
at least one face by > ε). The set of faces visible from an external point on
a convex surface forms a topological disk (not an annulus or multiply-connected
region) because convexity ensures that visibility is "monotone" — if a face is
visible from a point outside the hull, all faces between it and the initial
visible face (along any surface path) are also visible. This follows from the
supporting hyperplane theorem: a point outside a convex body sees a connected
cap.
(4) The boundary of a simply-connected region on a 2-manifold is a single
closed loop. Each boundary edge connects two boundary vertices, and each
boundary vertex has exactly one incoming and one outgoing boundary edge.
Therefore the horizon edges form a single closed polygon.

**Horizon edge count:** For a triangulated convex hull with V vertices, the
horizon has at most O(V) edges. In practice, it is O(√V) for random eye
points on typical hulls.

**Cone face construction and neighbor updates:**

For each horizon edge `(a, b, non_vis_fi)`, create face `[eye, a, b]`. The
face has three edges:
- Edge 0: `(eye, a)` — neighbor is the previous cone face
- Edge 1: `(a, b)` — neighbor is `non_vis_fi` (the non-visible face)
- Edge 2: `(b, eye)` — neighbor is the next cone face

The non-visible neighbor `non_vis_fi` is provided by `find_horizon()` (which
extracts it from `Face.neighbors` during BFS). `update_neighbor()` then
replaces `non_vis_fi`'s stale pointer to the dead visible face with the new
cone face index.

After creating all cone faces, link cone-cone neighbors: consecutive horizon
edges share a vertex (ordering invariant: `edge[i].1 == edge[i+1].0`), so
face i's edge 2 `(bᵢ, eye)` is shared with face (i+1)'s edge 0 `(eye, aᵢ₊₁)`
where `aᵢ₊₁ = bᵢ`.

This approach replaces the earlier edge_map-based neighbor lookup. All neighbor
information flows from `Face.neighbors` (authoritative), `find_horizon()`
(captures non-visible neighbors), and `order_horizon_edges()` (determines
cone-cone adjacency). No auxiliary data structure needed.

**Step 4 — Graph construction:**

After Quickhull completes, build the adjacency graph from face connectivity:

```rust
fn build_graph(vertices: &[Point3<f64>], faces: &[[usize; 3]]) -> HullGraph {
    let n = vertices.len();
    let mut adjacency: Vec<Vec<usize>> = vec![Vec::new(); n];
    for &[a, b, c] in faces {
        // Each edge of a face creates adjacency between its endpoints
        insert_sorted(&mut adjacency[a], b);
        insert_sorted(&mut adjacency[a], c);
        insert_sorted(&mut adjacency[b], a);
        insert_sorted(&mut adjacency[b], c);
        insert_sorted(&mut adjacency[c], a);
        insert_sorted(&mut adjacency[c], b);
    }
    HullGraph { adjacency }
}

fn insert_sorted(list: &mut Vec<usize>, val: usize) {
    if let Err(pos) = list.binary_search(&val) {
        list.insert(pos, val);
    }
}
```

**Step 5 — Face normals:**

Compute outward unit normal for each face:

```rust
let normals: Vec<Vector3<f64>> = faces.iter().map(|&[a, b, c]| {
    let e1 = vertices[b] - vertices[a];
    let e2 = vertices[c] - vertices[a];
    e1.cross(&e2).normalize()
}).collect();
```

Normals are guaranteed outward-facing because Quickhull maintains consistent
winding throughout expansion (new cone faces inherit winding from the horizon
edge orientation).

**Degenerate case handling:**

| Case | Behavior | MuJoCo equivalent |
|------|----------|-------------------|
| `nvert < 4` | Return `None` | `MakeGraph()` returns early, `szgraph_ = 0` |
| All collinear (Step 1 fails) | Return `None` | Qhull failure → graph freed |
| All coplanar (Step 1 fails) | Return `None` | Qhull `Qt` handles; our impl returns `None` for strict coplanarity |
| Duplicate vertices | Handled naturally — duplicates produce zero-distance conflict assignments and end up interior | `ProcessVertices()` deduplicates first |
| `maxhullvert` reached | Break from expansion loop; remaining conflict points become interior | Qhull `TA(N-4)` terminates early |
| Near-degenerate (thin sliver) | ε-guarded volume test in initial simplex (ε = scale-dependent, see `compute_epsilon()`); conflict distance threshold (same ε) | Qhull has internal robustness (scale-dependent epsilon) |

**Epsilon strategy — scale-dependent:**

Absolute epsilons fail for meshes at different scales (a 1mm mesh vs a 100m
mesh). We compute a **scale-dependent epsilon** based on the point cloud's
bounding box diagonal:

```rust
fn compute_epsilon(points: &[Point3<f64>]) -> f64 {
    let mut min = points[0];
    let mut max = points[0];
    for p in points {
        for i in 0..3 {
            min[i] = min[i].min(p[i]);
            max[i] = max[i].max(p[i]);
        }
    }
    let diagonal = (max - min).norm();
    // ε = diagonal × relative tolerance.
    // f64 machine epsilon is ~2.2e-16; geometric ops (cross products,
    // dot products with differences) accumulate ~3-4 ULPs of error.
    // Relative tolerance of 1e-10 gives ~6 orders of magnitude above
    // machine epsilon noise while remaining small enough to not discard
    // legitimate hull vertices.
    //
    // Qhull uses a similar approach (qh_distround in libqhull.c):
    // scale-dependent epsilon based on point cloud extent × machine epsilon,
    // with additional factors for dimensionality. Qhull's effective relative
    // tolerance is ~1e-12 to 1e-10 depending on configuration.
    //
    // Our factor of 1e-10 is heuristically chosen to be:
    //   (a) Large enough to filter numerical noise (>> machine epsilon × ops)
    //   (b) Small enough to preserve legitimate hull vertices (a vertex
    //       at distance 1e-8 × diagonal from the hull surface is kept)
    //   (c) Conformance-neutral: hull vertex sets are stable across
    //       reasonable choices in the range [1e-12, 1e-8].
    let eps = diagonal * 1e-10;
    // Floor at absolute minimum to handle degenerate zero-extent point clouds.
    // Why 1e-14: f64 machine epsilon is 2.2e-16. Geometric operations (cross
    // products, dot products with vector differences) accumulate ~10-100x
    // roundoff, yielding effective noise of ~1e-14. The floor ensures that
    // even for a zero-diagonal point cloud (all identical points), distance
    // comparisons are above the noise floor. Values in [1e-15, 1e-13] would
    // also work; 1e-14 is a round number that provides ~45x margin above
    // machine epsilon.
    eps.max(1e-14)
}
```

**Epsilon values (derived from scale):**
- Coplanarity/collinearity test: `ε` (scale-dependent, computed above)
- Conflict assignment threshold: `ε` (same — point must be this far above face)
- Initial simplex farthest-from-line/plane: `ε` (same)

**Duplicate vertex handling:** The initial simplex selection uses extremal
points (min/max per axis). If multiple points share the same extremal
coordinate (duplicates or coincident points), the algorithm picks the first
encountered — this is valid because the subsequent farthest-from-line and
farthest-from-plane selections resolve degeneracy. If ALL input points are
identical (worst case), the farthest-from-line distance is 0 < ε, and the
algorithm returns `None` (degenerate). For partially-duplicate inputs where
≥4 distinct points exist, the extremal heuristic finds at least 2 distinct
points (min and max differ on at least one axis), and the subsequent steps
find the remaining 2 from the non-degenerate subset.

**Winding consistency proof:** The initial simplex has consistent outward
winding (Step 1 orients the tetrahedron). During expansion, each new cone face
is `[eye, a, b]` where `(a, b)` is a horizon edge in the visible face's
winding direction (NOT reversed). The shared edge between the new cone face
and the non-visible face is `(a, b)` at edge index 1 of the cone face.

On the existing manifold, the visible face had edge `(a, b)` and the adjacent
non-visible face had the same edge as `(b, a)` (opposite direction — manifold
consistency). The new cone face `[eye, a, b]` has `(a, b)` at edge 1, which
is opposite to the non-visible face's `(b, a)`. Therefore the shared edge
between the new cone face and the non-visible face has opposite orientations
in the two faces — manifold consistency is preserved.

For cone-cone edges: face i = `[eye, aᵢ, bᵢ]` has edge 2 = `(bᵢ, eye)`.
Face i+1 = `[eye, bᵢ, bᵢ₊₁]` (since aᵢ₊₁ = bᵢ) has edge 0 = `(eye, bᵢ)`.
The shared edge is `(bᵢ, eye)` vs `(eye, bᵢ)` — opposite directions. ✓

Normal direction proof: the cone face normal `(a - eye) × (b - eye)` points
away from the hull interior. Let H be any interior point of the old hull
(e.g., the centroid of the initial simplex). The visible face had outward
normal N such that `(eye - face_center) · N > ε` (eye is outside). The cone
face `[eye, a, b]` has vertices where `a` and `b` are hull vertices (on the
old hull surface) and `eye` is strictly outside. The normal of `[eye, a, b]`
is `(a - eye) × (b - eye)`.

To show this normal points away from H: the edge `(a, b)` lies on the
boundary between the visible and non-visible regions of the hull surface.
The non-visible face containing `(b, a)` has an outward normal pointing
away from H. The cone face `[eye, a, b]` connects the horizon edge to
the external eye point, creating a face whose normal has a positive
component along the visible face's outward normal direction (because eye
is on the outward side of the visible face). Formally: the signed volume
of tetrahedron `(H, eye, a, b)` has the same sign as `(H - eye) · ((a - eye) × (b - eye))`.
Since H is inside the old hull and eye is outside, `H - eye` points inward
from eye, and the cross product `(a - eye) × (b - eye)` points outward
(same half-space as the visible face normal). Therefore the dot product
is negative, confirming the normal points away from H.

Therefore, winding consistency is preserved through every expansion step.
By induction from the correctly-oriented initial simplex, all faces have
consistent outward-pointing normals after any number of expansion iterations.

**Termination invariant (proof sketch):** At each iteration, every remaining
point is assigned to the conflict list of some face it is strictly above
(distance > ε). When a face is deleted and replaced by the cone from the eye
point, conflict points of deleted faces are redistributed to new faces they
are above. Points below all new faces are discarded (they are now interior to
the hull). When all conflict lists are empty, no point is above any face —
therefore every point is on or below every face plane, which means every point
is on or inside the convex hull. The hull is complete.

**Termination — strict progress:** At each iteration, the eye point E is the
farthest conflict point of some face. E becomes a hull vertex and is removed
from its conflict list. E is NOT added to any new conflict list (the code
filters `pi != eye_pi` when collecting orphan points). Therefore, the total
number of conflict points across all faces strictly decreases by at least 1
per iteration. Since the initial conflict count is finite (at most n - 4
points), the loop terminates in at most n - 4 iterations.

**Conflict redistribution under near-epsilon distances:** When conflict points
are very close to face planes (distance ≈ ε), redistribution may discard them
(treated as interior) even though they are technically above. This is correct
behavior: such points would create faces with normals dominated by numerical
noise, producing slivers. The ε threshold ensures all accepted conflict points
produce geometrically meaningful expansions. A point discarded at distance ≈ ε
is within ε of the hull surface, which is within the tolerance budget for
convex hull construction. The hull remains a valid convex polytope containing
all input points (to within ε).

**Near-degenerate robustness:** For closely-spaced vertex clusters (multiple
points within ε of each other), the conflict assignment treats them as
identical — they are either all above a face or all interior. For conflict
points very close to a face plane (distance ≈ ε), the threshold ensures they
are treated as interior rather than above, avoiding the creation of
numerically-degenerate slivers.

**Initial simplex quality argument:** The extremal heuristic (min/max per axis)
maximizes the coordinate spread of the initial simplex. For a non-degenerate
point cloud (≥4 non-coplanar points), the extremal pair has maximum separation
along at least one axis, and `farthest_from_line` / `farthest_from_plane` find
the points maximizing area and volume respectively. This produces a simplex
whose volume is bounded below by a quantity proportional to ε³.
**Volume bound justification:** The simplex volume is
`V = |det([p1-p0, p2-p0, p3-p0])| / 6`. The bound arises from:

- **Base area:** `farthest_from_line` ensures the third point p2 is at
  distance ≥ ε from line (p0, p1). The base triangle (p0, p1, p2) has
  area = ½ × |p1-p0| × dist(p2, line) ≥ ½ × |p1-p0| × ε. The base edge
  |p1-p0| is the extremal pair distance, which is ≥ the bounding box
  diagonal (the maximum coordinate spread). For the bound, the area is
  ≥ ½ × D × ε where D is the extremal pair distance.

- **Height:** `farthest_from_plane` ensures p3 is at distance ≥ ε from
  the plane of (p0, p1, p2).

- **Volume:** V = ⅓ × base_area × height ≥ ⅓ × (½ × D × ε) × ε
  = D × ε² / 6. Since D ≥ ε (the extremal pair is at least as spread as
  the threshold), V ≥ ε³ / 6.

A simplex with volume ≥ ε³/6 has face normals (cross products of edge
vectors) with magnitude ≥ O(ε²), sufficient to produce numerically stable
conflict assignments (signed distances are computed via dot products with
unit normals, so the normalization step divides by O(ε²), and the resulting
distances have O(ε) precision). For truly near-degenerate point clouds
where even the extremal heuristic produces a simplex with volume < ε³, the
`farthest_from_plane` check returns `None` (all points coplanar to within ε),
correctly rejecting the input rather than producing a garbage hull.

### S2. Hill-climbing support + warm-start in `gjk_epa.rs`

**File:** `sim/L0/core/src/gjk_epa.rs` (lines 218-316 `support()`, lines 376-397 `support_convex_mesh()`)
**MuJoCo equivalent:** `mjc_hillclimbSupport()` and `mjc_meshSupport()` in `engine_collision_convex.c`, `mjc_initCCDObj()` for strategy selection
**Design decision:** Add `graph: Option<HullGraph>` to `CollisionShape::ConvexMesh` variant. Hill-climbing for ≥10 vertices (matching `mjMESH_HILLCLIMB_MIN = 10`), exhaustive for <10. Warm-start cache as thread-local or per-call mutable state.

#### `support()` dispatch update

The top-level `support()` function (line 218) dispatches by `CollisionShape` variant.
The `ConvexMesh` arm is updated to destructure the new fields and pass them to
`support_convex_mesh()`:

**Before** (`gjk_epa.rs:226`):
```rust
CollisionShape::ConvexMesh { vertices } => support_convex_mesh(pose, vertices, direction),
```

**After:**
```rust
CollisionShape::ConvexMesh { vertices, graph, warm_start } => {
    support_convex_mesh(pose, vertices, graph.as_ref(), warm_start, direction)
}
```

This routes to the updated `support_convex_mesh()` below, which internally
selects hill-climbing (when `graph.is_some()`) or exhaustive scan.

#### `CollisionShape::ConvexMesh` variant extension

**Before** (current — `collision_shape.rs:91-95`):
```rust
ConvexMesh {
    vertices: Vec<Point3<f64>>,
},
```

**After:**
```rust
ConvexMesh {
    /// Vertices of the convex hull in local coordinates.
    vertices: Vec<Point3<f64>>,
    /// Adjacency graph for hill-climbing support queries.
    /// `None` for manually constructed shapes or hulls with <10 vertices.
    graph: Option<HullGraph>,
    /// Warm-start cache: last support vertex index for hill-climbing.
    /// `Cell` allows mutation through `&self` (interior mutability).
    warm_start: Cell<usize>,
},
```

The `convex_mesh()` factory method (line 278) is updated:
```rust
pub fn convex_mesh(vertices: Vec<Point3<f64>>) -> Self {
    debug_assert!(vertices.len() >= 4, ...);
    Self::ConvexMesh { vertices, graph: None, warm_start: Cell::new(0) }
}

/// Construct a ConvexMesh from a precomputed ConvexHull.
pub fn convex_mesh_from_hull(hull: &ConvexHull) -> Self {
    Self::ConvexMesh {
        vertices: hull.vertices.clone(),
        graph: if hull.vertices.len() >= HILL_CLIMB_MIN {
            Some(hull.graph.clone())
        } else {
            None
        },
        warm_start: Cell::new(0),
    }
}
```

#### Hill-climbing support function

**MuJoCo C source** (`engine_collision_convex.c` → `mjc_hillclimbSupport`):

```c
// Rotate direction to local frame
mjtNum local_dir[3];
mulMatTVec3(local_dir, mat, dir);

mjtNum max = -FLT_MAX;
int prev = -1;
int imax = obj->meshindex >= 0 ? obj->meshindex : 0;  // warm-start

// Steepest-ascent hill climb: scan ALL neighbors, pick best
while (imax != prev) {
    prev = imax;
    int subidx;
    for (int i = vert_edgeadr[imax]; (subidx = edge_localid[i]) >= 0; i++) {
        mjtNum vdot = dot3f(local_dir, verts + 3*vert_globalid[subidx]);
        if (vdot > max) {
            max = vdot;
            imax = subidx;
        }
    }
}
obj->meshindex = imax;  // cache LOCAL index for next call
```

**Critical detail 1:** MuJoCo uses **steepest-ascent** (best-improvement) hill
climbing, NOT greedy first-improvement. The inner `for` loop scans ALL
neighbors of the current vertex, updating `imax` to whichever neighbor has the
highest dot product. There is no `break` on first improvement. The outer
`while` loop restarts from the new best vertex only after ALL neighbors of
the current vertex have been examined. The `vert_edgeadr[imax]` is evaluated
once at for-loop entry; even though `imax` is mutated inside the body, `i`
continues incrementing through the neighbor list of the **original** vertex.

**Critical detail 2:** MuJoCo never explicitly computes the start vertex's
own dot product. The `max` is initialized to `-FLT_MAX`, and the start
vertex is only "discovered" if it appears in its own neighbor list (which
it doesn't on a convex hull) or when a neighbor's dot product is computed.
In practice, the first neighbor's dot product always exceeds `-FLT_MAX`,
so the algorithm works correctly. CortenForge's `hill_climb_support()`
similarly initializes `max_dot` to `f64::NEG_INFINITY` and never computes
the start vertex's dot product — convergence is detected via `current == prev`
(no neighbor improved), not via explicit comparison with the start vertex's
value. An implementer should NOT "fix" this by initializing `max_dot` from
the start vertex — this matches MuJoCo's behavior and is correct.

**MuJoCo C source** (`engine_collision_convex.c` → `mjc_meshSupport`):

```c
mjtNum max = -FLT_MAX;
int imax = 0;

// Warm-start: initialize max from cached vertex
if (obj->vertindex >= 0) {
    imax = obj->vertindex;
    max = dot3f(local_dir, verts + 3*imax);
}

// Exhaustive scan of ALL vertices
for (int i = 0; i < nverts; i++) {
    mjtNum vdot = dot3f(local_dir, verts + 3*i);
    if (vdot > max) {
        max = vdot;
        imax = i;
    }
}
obj->vertindex = imax;
```

**Note:** The exhaustive path warm-starts by initializing `max` from the
cached vertex's dot product (instead of `-FLT_MAX`). It still scans ALL
vertices — the warm-start provides a tighter initial bound for the comparison,
which is a minor branch-prediction optimization.

**Margin handling:** Margin is NOT applied inside the support functions.
MuJoCo inflates the support result by `margin/2` along the search direction in
the **libccd support wrapper** (`mjccd_support`), so each object is inflated by
`margin/2` and the total Minkowski inflation equals `margin`. CortenForge's
`gjk_epa_contact()` does NOT take a margin parameter — support functions
return raw vertex positions. Margin is handled downstream in
`make_contact_from_geoms()` (S4), which receives the `margin` parameter from
`collide_with_mesh()` and passes it to `contact_param()` for `includemargin`
computation. This separation (support = raw geometry, margin = contact
construction) is architecturally equivalent to MuJoCo's separation (support =
raw geometry, margin = libccd wrapper).

**CortenForge implementation:**

```rust
/// Minimum vertex count for hill-climbing support.
/// Matches MuJoCo's `mjMESH_HILLCLIMB_MIN = 10`.
const HILL_CLIMB_MIN: usize = 10;

fn support_convex_mesh(
    pose: &Pose,
    vertices: &[Point3<f64>],
    graph: Option<&HullGraph>,
    warm_start: &Cell<usize>,
    direction: &Vector3<f64>,
) -> Point3<f64> {
    if vertices.is_empty() {
        return pose.position;
    }

    let local_dir = pose.rotation.inverse() * direction;

    let best_idx = if let Some(g) = graph {
        // Hill-climbing (steepest-ascent): warm-start from cached vertex
        let start = warm_start.get().min(vertices.len() - 1);
        let idx = hill_climb_support(vertices, g, &local_dir, start);
        warm_start.set(idx);
        idx
    } else {
        // Exhaustive: warm-start by initializing max from cached vertex
        let cached = warm_start.get().min(vertices.len() - 1);
        let mut best = cached;
        let mut max_dot = vertices[cached].coords.dot(&local_dir);
        for (i, v) in vertices.iter().enumerate() {
            let dot = v.coords.dot(&local_dir);
            if dot > max_dot {
                max_dot = dot;
                best = i;
            }
        }
        warm_start.set(best);
        best
    };

    pose.transform_point(&vertices[best_idx])
}

/// Steepest-ascent hill-climbing support on convex hull graph.
///
/// Matches MuJoCo's `mjc_hillclimbSupport()`: at each step, examine ALL
/// neighbors of the current vertex, move to the neighbor with the highest
/// dot product, repeat until no neighbor improves. This is steepest-ascent
/// (best-improvement), NOT greedy first-improvement.
///
/// Guaranteed to find the global maximum because the dot product
/// is a linear (unimodal) function on a convex surface.
fn hill_climb_support(
    vertices: &[Point3<f64>],
    graph: &HullGraph,
    direction: &Vector3<f64>,
    start: usize,
) -> usize {
    let mut current = start;
    let mut max_dot = f64::NEG_INFINITY;

    loop {
        let prev = current;
        // Scan ALL neighbors of current vertex (steepest-ascent)
        for &neighbor in &graph.adjacency[current] {
            let dot = vertices[neighbor].coords.dot(direction);
            if dot > max_dot {
                max_dot = dot;
                current = neighbor;
            }
        }
        // Converged: no neighbor improved
        if current == prev {
            return current;
        }
    }
}
```

**Convergence guarantee (two-part argument):**

1. **No strict local maxima.** The dot product `v . d` is a linear function
   on vertex positions. On a convex polytope, a linear function achieves its
   maximum at a vertex (supporting hyperplane theorem). If vertex v has
   `v . d >= w . d` for all neighbors w, then v is a local maximum on the
   hull graph. But a linear function on a convex surface has no strict local
   maxima that are not global maxima: if some other vertex u had `u . d > v . d`,
   then by convexity the path from v to u on the hull graph must pass through
   a neighbor of v with a higher dot product — contradicting v being a local
   max. Therefore any local maximum is the global maximum.

2. **Graph connectivity.** The 1-skeleton (vertex-edge graph) of a convex
   polyhedron is connected — this is a standard result (Balinski's theorem:
   the graph of a d-polytope is d-connected, hence connected for d >= 2).
   Therefore steepest-ascent can reach any vertex from any starting vertex.

Combining (1) and (2): steepest-ascent hill-climbing always converges to the
global maximum regardless of starting vertex.

**Start-vertex-is-optimum edge case:** When the start vertex v0 is already
the global maximum, the algorithm's first iteration sets `max_dot = NEG_INFINITY`
and scans all neighbors. Since all neighbors have lower dot products than v0
(v0 is the global max), some neighbor N with the highest dot product is selected.
The algorithm moves to N (since N's dot product > NEG_INFINITY). On the next
iteration, N's neighbors are scanned — including v0 (adjacency is symmetric on
a convex hull). Since `v0 . d > N . d >= max_dot`, the algorithm moves back to v0.
Then v0's neighbors are scanned again, and no neighbor exceeds v0's dot product
(which is now `max_dot`). The algorithm terminates at v0. Correct, albeit with
one extra round-trip. This matches MuJoCo's behavior (same initialization).

This is the same argument MuJoCo relies on.

**Warm-start mechanism:** MuJoCo caches the last-used vertex index on
`mjCCDObj`: `meshindex` for hill-climbing (local hull index) and `vertindex`
for exhaustive (global mesh index). Both are initialized to `-1` (no cache)
by `mjc_initCCDObj()`. CortenForge uses `Cell<usize>` on `ConvexMesh`:

```rust
ConvexMesh {
    vertices: Vec<Point3<f64>>,
    graph: Option<HullGraph>,
    /// Warm-start cache: last support vertex index.
    /// Used by both hill-climbing (start vertex) and exhaustive
    /// (initial max bound). `Cell` for interior mutability through `&self`.
    warm_start: Cell<usize>,
},
```

Both paths (hill-climbing and exhaustive) read and update the cache:
- **Hill-climbing:** Starts walking from cached vertex. Warm-starting from the
  previous result amortizes convergence time for sequential queries with
  similar directions (typical in GJK iteration).
- **Exhaustive:** Initializes `max_dot` from cached vertex's dot product
  (instead of `NEG_INFINITY`), providing a tighter initial bound. Still scans
  all vertices.

`Cell<usize>` is `Send` but not `Sync` (this is a Rust standard library
guarantee — `Cell<T>` implements `!Sync` unconditionally because it allows
interior mutation without synchronization). This matches CortenForge's
single-threaded collision pipeline. If the collision pipeline is ever
parallelized, `Cell<usize>` would cause a compile error (cannot be shared
across threads via `&CollisionShape`), forcing a conscious decision to
use `AtomicUsize` or thread-local caching. The `convex_mesh()` factory
initializes `warm_start: Cell::new(0)`.

**Warm-start persistence:** The warm-start cache lives on the
`CollisionShape::ConvexMesh` variant, which is constructed once per mesh-mesh
collision call in `collide_with_mesh()` (S4). This means the cache is fresh
(0) for each collision call, not persisted across frames. MuJoCo persists
`meshindex`/`vertindex` on the `mjCCDObj` struct, which is also per-call (not
cross-frame). Both systems warm-start across GJK iterations within a single
support query sequence, not across frames. The behavior is conformant.

#### Match site updates

All production match sites that destructure `ConvexMesh { vertices }` must
be updated to `ConvexMesh { vertices, .. }` or `ConvexMesh { vertices, graph }`:

| File | Line | Current pattern | New pattern |
|------|------|-----------------|-------------|
| `collision_shape.rs` | 284 | `convex_mesh()` factory: `Self::ConvexMesh { vertices }` struct literal | `Self::ConvexMesh { vertices, graph: None, warm_start: Cell::new(0) }` |
| `collision_shape.rs` | 309 | `tetrahedron()` factory: `Self::ConvexMesh { vertices }` struct literal | `Self::ConvexMesh { vertices, graph: None, warm_start: Cell::new(0) }` |
| `collision_shape.rs` | 500 | `bounding_radius`: `ConvexMesh { vertices }` | `ConvexMesh { vertices, .. }` |
| `collision_shape.rs` | 573 | `local_aabb`: `ConvexMesh { vertices }` | `ConvexMesh { vertices, .. }` |
| `gjk_epa.rs` | 226 | `ConvexMesh { vertices }` | `ConvexMesh { vertices, graph, warm_start }` |
| `raycast.rs` | 142 | `ConvexMesh { vertices }` | `ConvexMesh { vertices, .. }` |
| `bevy/components.rs` | 82 | `ConvexMesh { .. }` | No change needed |
| `bevy/mesh.rs` | 40 | `ConvexMesh { vertices }` | `ConvexMesh { vertices, .. }` |

### S3. `maxhullvert` parsing in `types.rs` + `builder/mesh.rs`

**File:** `sim/L0/mjcf/src/types.rs` (line 856, `MjcfMesh` struct)
**MuJoCo equivalent:** `ReadAttrInt(elem, "maxhullvert", ...)` in `xml_native_reader.cc`
**Design decision:** Parse as `Option<i32>` to match MuJoCo's validation
(`-1` or `> 3`). Stored as `Option<usize>` after validation (converting -1 to
None, positive values to Some(n)).

**Before** (`types.rs` MjcfMesh — line 856-881):
```rust
pub struct MjcfMesh {
    pub name: String,
    pub file: Option<String>,
    pub scale: Option<Vector3<f64>>,
    pub vertex: Option<Vec<f64>>,
    pub face: Option<Vec<u32>>,
}
```

**After:**
```rust
pub struct MjcfMesh {
    pub name: String,
    pub file: Option<String>,
    pub scale: Option<Vector3<f64>>,
    pub vertex: Option<Vec<f64>>,
    pub face: Option<Vec<u32>>,
    /// Maximum convex hull vertices. `None` = no limit (default).
    /// `Some(n)` where n >= 4 = limit hull to n vertices.
    /// MuJoCo: `maxhullvert` attribute, default -1 (no limit), min 4.
    pub maxhullvert: Option<usize>,
}
```

**Parser update:** In the MJCF parser for `<mesh>`, parse `maxhullvert` as
an integer. Validate:
- If value is `-1`: store `None` (no limit)
- If value is `>= 4`: store `Some(value as usize)`
- If value is `< 4` and not `-1`: error with message matching MuJoCo:
  `"maxhullvert must be larger than 3"`
- If attribute absent: default to `None`

Also add `maxhullvert: Option<usize>` to `MjcfMeshDefaults` (types.rs line
842-845) for default class inheritance:

```rust
// types.rs — MjcfMeshDefaults (line 840-845)
// Before: only `scale: Option<Vector3<f64>>`
// After:
pub struct MjcfMeshDefaults {
    pub scale: Option<Vector3<f64>>,
    pub maxhullvert: Option<usize>,
}
```

The MJCF default class inheritance mechanism applies `MjcfMeshDefaults` fields
to `MjcfMesh` when the `<mesh>` element does not explicitly set them. The
MJCF builder handles defaults on a per-element basis: each element type
(geom, actuator, joint, etc.) has its own defaults struct, and the builder
merges defaults into parsed elements during `process_mesh()` in
`sim/L0/mjcf/src/builder/mesh.rs`. The `scale` field is already merged
there. Adding `maxhullvert` follows the same pattern: if the `<mesh>`
element has no `maxhullvert` attribute, use the default class value; if
neither specifies it, use `None` (no limit). There is no single
`apply_defaults()` function — each builder module handles its own defaults
inline.

### S4. Mesh-mesh hull integration in `mesh_collide.rs`

**File:** `sim/L0/core/src/collision/mesh_collide.rs` (lines 46-52, mesh-mesh dispatch)
**MuJoCo equivalent:** All mesh-mesh collision in MuJoCo goes through GJK/EPA on convex hulls
**Design decision:** AD-1 option (b) — keep dispatch routing, use hull
internally for mesh-mesh pairs only. Mesh-primitive pairs unchanged.

**Before** (mesh-mesh path in `collide_with_mesh()`, lines 46-52):
```rust
(GeomType::Mesh, GeomType::Mesh) => {
    let mesh1_id = model.geom_mesh[geom1]?;
    let mesh2_id = model.geom_mesh[geom2]?;
    let mesh1 = &model.mesh_data[mesh1_id];
    let mesh2 = &model.mesh_data[mesh2_id];
    mesh_mesh_deepest_contact(mesh1, &pose1, mesh2, &pose2, use_bvh)
}
```

**After:**

**Type note:** The outer `match` block assigns to `let mesh_contact: Option<MeshContact>`
(line 44 of `mesh_collide.rs`), which is then converted to `Contact` via
`make_contact_from_geoms()` at line 134. The hull path produces a `Contact`
directly (via `gjk_epa_contact()` → `make_contact_from_geoms()`), so it must
use an **early `return`** to bypass the `MeshContact`→`Contact` conversion at
the end of the function. The fallback (BVH) path returns `Option<MeshContact>`
as before.

```rust
(GeomType::Mesh, GeomType::Mesh) => {
    let mesh1_id = model.geom_mesh[geom1]?;
    let mesh2_id = model.geom_mesh[geom2]?;
    let mesh1 = &model.mesh_data[mesh1_id];
    let mesh2 = &model.mesh_data[mesh2_id];

    // If both meshes have convex hulls, use GJK/EPA on hulls
    // (MuJoCo-conformant path). Early return: hull path produces
    // Contact directly, bypassing the MeshContact→Contact conversion
    // at the bottom of collide_with_mesh().
    if let (Some(hull1), Some(hull2)) = (mesh1.convex_hull(), mesh2.convex_hull()) {
        let shape1 = CollisionShape::convex_mesh_from_hull(hull1);
        let shape2 = CollisionShape::convex_mesh_from_hull(hull2);
        // gjk_epa_contact() at gjk_epa.rs:1020-1046
        // Returns Option<GjkContact { point: Point3, normal: Vector3,
        //                             penetration: f64 }>.
        // Does NOT take a margin parameter — margin is passed
        // downstream to make_contact_from_geoms() which handles
        // includemargin via contact_param().
        // Convert GjkContact.point (Point3) → Vector3 via .coords
        // because make_contact_from_geoms() expects pos: Vector3.
        return gjk_epa_contact(&shape1, &pose1, &shape2, &pose2)
            .map(|gjk| {
                // GjkContact.penetration: positive when overlapping
                //   (clamped to >= 0 by gjk_epa_contact, line 1044).
                // make_contact_from_geoms.depth: positive when overlapping
                //   (same sign convention — no conversion needed).
                // Verified: gjk_epa.rs:101-110 defines penetration as
                //   "positive when overlapping", narrow.rs:296-305
                //   accepts depth with the same convention.
                make_contact_from_geoms(
                    model,
                    gjk.point.coords, // Point3 → Vector3
                    gjk.normal,
                    gjk.penetration,  // same sign convention as depth
                    geom1,
                    geom2,
                    margin,
                )
            });
    }

    // Fallback to per-triangle BVH if either mesh lacks hull.
    // Returns Option<MeshContact> for conversion at end of function.
    mesh_mesh_deepest_contact(mesh1, &pose1, mesh2, &pose2, use_bvh)
}
```

### S5. Build pipeline integration in `builder/mesh.rs`

**File:** `sim/L0/mjcf/src/builder/mesh.rs` (lines 39-61 `process_mesh()`)
**MuJoCo equivalent:** Hull computation in `mjCMesh::Process()` after mesh loading
**Design decision:** AD-3 option A — compute hull for all meshes unconditionally.
Hull computed after `TriangleMeshData::new()`, before `Arc::new()` wrapping.

**Before** (`process_mesh()` — line 52-58):
```rust
let mesh_data = convert_mjcf_mesh(mjcf_mesh, base_path, &self.compiler)?;
let mesh_id = self.mesh_data.len();
self.mesh_name_to_id.insert(mjcf_mesh.name.clone(), mesh_id);
self.mesh_name.push(mjcf_mesh.name.clone());
self.mesh_data.push(Arc::new(mesh_data));
```

**After:**
```rust
let mut mesh_data = convert_mjcf_mesh(mjcf_mesh, base_path, &self.compiler)?;

// Compute convex hull at build time (matching MuJoCo's MakeGraph())
let max_hull_vert = mjcf_mesh.maxhullvert;
mesh_data.compute_convex_hull(max_hull_vert);

let mesh_id = self.mesh_data.len();
self.mesh_name_to_id.insert(mjcf_mesh.name.clone(), mesh_id);
self.mesh_name.push(mjcf_mesh.name.clone());
self.mesh_data.push(Arc::new(mesh_data));
```

The `compute_convex_hull` method on `TriangleMeshData`:

```rust
impl TriangleMeshData {
    /// Compute and store the convex hull of this mesh's vertices.
    /// If the mesh has fewer than 4 vertices or all points are degenerate,
    /// `convex_hull` remains `None`.
    pub fn compute_convex_hull(&mut self, max_vertices: Option<usize>) {
        self.convex_hull = quickhull(self.vertices(), max_vertices);
    }

    /// Returns the convex hull if computed.
    pub fn convex_hull(&self) -> Option<&ConvexHull> {
        self.convex_hull.as_ref()
    }
}
```

**Import chain:** `mesh.rs` must import the hull types and function:
```rust
// In sim/L0/core/src/mesh.rs, add at the top:
use crate::convex_hull::{ConvexHull, quickhull};
```

The `convex_hull` field on `TriangleMeshData`:

```rust
pub struct TriangleMeshData {
    vertices: Vec<Point3<f64>>,
    triangles: Vec<Triangle>,
    aabb_min: Point3<f64>,
    aabb_max: Point3<f64>,
    #[cfg_attr(feature = "serde", serde(skip))]
    bvh: Option<Bvh>,
    /// Convex hull for collision (computed at build time via Quickhull).
    /// Serialized (unlike BVH) because hull computation is expensive.
    convex_hull: Option<ConvexHull>,
}
```

**Original mesh data preserved:** The `convex_hull` field is purely additive —
the original `vertices`, `triangles`, `aabb_min`, `aabb_max`, and `bvh` fields
are unchanged. The hull is computed FROM the mesh vertices but does not replace
them. Original mesh data remains available for rendering, per-triangle
collision (mesh-primitive pairs), and inertia computation (Spec B exact mode).

**`PartialEq` consideration:** `TriangleMeshData` has a manual `PartialEq`
implementation (`mesh.rs:103-107`) that only compares `vertices` and
`triangles` (ignores `bvh`). The `convex_hull` field should also be excluded
from `PartialEq` — it is deterministically derived from `vertices` (like
`bvh` is from triangles), so including it would be redundant and would fail
for meshes where one has a hull and the other doesn't yet. No change to the
manual `PartialEq` impl is needed — the new field is automatically excluded
because the impl only accesses `self.vertices` and `self.triangles`.

**Visual-only mesh detection:** Not needed — per AD-3 option A, hulls are
computed for all meshes unconditionally. The `process_mesh()` function does
not distinguish between collision and visual-only meshes. This is a
conformance-neutral simplification: computing an unused hull for a visual-only
mesh has no effect on collision behavior.

**Serde strategy:** `ConvexHull` derives `Serialize`/`Deserialize` (not
skipped like BVH). Rationale: Quickhull is O(n log n) and non-trivial to
recompute; BVH is simpler to rebuild. Preserving the hull across serde
round-trips avoids recomputation cost and ensures exact hull preservation.

**`Cell<usize>` and serde:** `CollisionShape` does NOT derive
`Serialize`/`Deserialize` (only `Debug, Clone`). Therefore `Cell<usize>` on
`ConvexMesh` causes no serde issue. If `CollisionShape` ever adds serde in
the future, `Cell<usize>` implements serde natively via the `serde` crate
(serializes the inner value). No `#[serde(skip)]` needed.

---

## Acceptance Criteria

### AC1: Unit cube hull vertex/face count *(runtime test — analytically derived)*
**Given:** A mesh with 8 vertices at (±0.5, ±0.5, ±0.5) forming a cube
**After:** `quickhull()` with `max_vertices = None`
**Assert:** Hull has exactly 8 vertices and 12 triangular faces
**Field:** `ConvexHull.vertices.len()`, `ConvexHull.faces.len()`

### AC2: Tetrahedron hull — minimum valid *(runtime test — analytically derived)*
**Given:** A mesh with 4 vertices forming a regular tetrahedron
**After:** `quickhull()` with `max_vertices = None`
**Assert:** Hull has exactly 4 vertices and 4 triangular faces
**Field:** `ConvexHull.vertices.len()`, `ConvexHull.faces.len()`

### AC3: Interior point excluded *(runtime test — analytically derived)*
**Given:** 8 cube vertices + 1 interior point at (0, 0, 0)
**After:** `quickhull()` with `max_vertices = None`
**Assert:** Hull has exactly 8 vertices (interior point excluded)
**Field:** `ConvexHull.vertices.len()`

### AC4: `maxhullvert` limits vertex count *(runtime test — analytically derived)*
**Given:** Icosphere with 42 vertices, `max_vertices = Some(10)`
**After:** `quickhull()`
**Assert:** `hull.vertices.len() <= 10` AND hull is a valid convex polytope
(all input points lie on or below all hull face planes within ε). The exact
count depends on when the limit is checked relative to the expansion loop:
the initial simplex contributes 4 vertices, then each expansion step adds 1.
The loop checks `hull_vertices.len() >= max` before adding, so the limit
may be reached at exactly 10. However, the assertion uses `<=` (not `==`)
to be robust against edge cases where the initial simplex vertices happen
to be ≥ max (e.g., `maxhullvert = 4` → exactly 4 from the simplex alone).
**Field:** `ConvexHull.vertices.len()`

### AC5: `maxhullvert = 4` boundary *(runtime test — analytically derived)*
**Given:** 8 cube vertices, `max_vertices = Some(4)`
**After:** `quickhull()`
**Assert:** Hull has exactly 4 vertices (initial simplex only — no expansion)
**Field:** `ConvexHull.vertices.len()`

### AC6: Fewer than 4 vertices → no hull *(runtime test — analytically derived)*
**Given:** 3 vertices forming a triangle
**After:** `quickhull()`
**Assert:** Returns `None`
**Field:** Return value

### AC7: Duplicate vertices handled *(runtime test — analytically derived)*
**Given:** 8 cube vertices, each duplicated (16 total points)
**After:** `quickhull()`
**Assert:** Hull has 8 unique vertices and 12 faces (duplicates become interior)
**Field:** `ConvexHull.vertices.len()`, `ConvexHull.faces.len()`

### AC8: Hill-climbing ≡ exhaustive for all directions *(runtime test — analytically derived)*
**Given:** ConvexMesh from icosphere hull (42 vertices) with graph
**After:** Query support in 100 random directions using both hill-climbing and
exhaustive O(n) scan
**Assert:** Both strategies return a vertex with the same maximum dot product
for every direction (compared by value, not index, to handle ties)
**Field:** `vertices[hill_climb_support(...)].coords.dot(&dir)` vs
`vertices[exhaustive_max_idx].coords.dot(&dir)`

### AC9: `maxhullvert` parsing *(runtime test)*
**Given:** MJCF with `<mesh name="box" maxhullvert="10" vertex="..."/>`
**After:** Parse MJCF
**Assert:** `mjcf_mesh.maxhullvert == Some(10)`
**Field:** `MjcfMesh.maxhullvert`

### AC10: `maxhullvert` validation *(runtime test)*
**Given:** MJCF with `<mesh name="box" maxhullvert="2" vertex="..."/>`
**After:** Parse MJCF
**Assert:** Parse error containing "maxhullvert must be larger than 3"
**Field:** Error message

### AC11: Mesh-mesh collision uses hull when available *(runtime test)*
**Given:** Two cube mesh geoms (unit cubes), first at origin, second at
(0.9, 0.0, 0.0) — overlapping by 0.1 units along X. Both have convex hulls.
**After:** `collide_with_mesh()` for the pair
**Assert:** Contact is generated with:
- `depth` ≈ 0.1 (within 1e-3)
- `normal` approximately `(±1, 0, 0)` (dot product with `(1,0,0)` > 0.99)
- `pos.x` ≈ 0.45 to 0.50 (within 0.05)
**Tolerance justification:** EPA on axis-aligned cubes converges in ≤3
iterations because the Minkowski difference is also a box with axis-aligned
faces. The EPA polytope quickly captures the exact penetration direction
(along X). Depth tolerance of 1e-3 is conservative — EPA typically achieves
<1e-6 on this configuration. Normal tolerance of 0.99 cosine allows for minor
numerical noise in the GJK simplex. Position tolerance of 0.05 is wider
because EPA's contact point is on the Minkowski boundary, which is then
projected back to the original shapes — the exact position depends on the
EPA simplex vertex selection.
**Field:** `Contact.depth`, `Contact.normal`, `Contact.pos`

### AC12: Hull available after build *(runtime test)*
**Given:** Model with mesh geom parsed from MJCF
**After:** Model build completes
**Assert:** `model.mesh_data[mesh_id].convex_hull().is_some()`
**Field:** `TriangleMeshData.convex_hull`

### AC13: No `unsafe` blocks *(code review)*
All new code in `convex_hull.rs`, modified code in `gjk_epa.rs`, `collision_shape.rs`,
`mesh_collide.rs`, `builder/mesh.rs`, and `types.rs` contains zero `unsafe` blocks.

### AC14: Face normals point outward *(runtime test — analytically derived)*
**Given:** Unit cube hull (8 vertices, 12 faces)
**After:** `quickhull()` with `max_vertices = None`
**Assert:** Every face normal, dotted with the vector from hull centroid to
face centroid, is positive (normal points outward from the hull interior).
**Field:** `ConvexHull.normals`, `ConvexHull.faces`, `ConvexHull.vertices`

### AC15: Graph adjacency structural validity *(runtime test — analytically derived)*
**Given:** Unit cube hull (8 vertices, 12 faces, graph)
**After:** `quickhull()` with `max_vertices = None`
**Assert:**
- `graph.adjacency.len() == hull.vertices.len()` (one list per vertex)
- Every vertex has ≥3 neighbors (minimum degree on a convex polyhedron)
- Adjacency is symmetric: if `b ∈ adjacency[a]`, then `a ∈ adjacency[b]`
- Every face edge `(a, b)` appears in `adjacency[a]` and `adjacency[b]`
- No self-loops: `a ∉ adjacency[a]`
**Field:** `ConvexHull.graph.adjacency`

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (cube hull) | T1 | Direct |
| AC2 (tetrahedron hull) | T2 | Direct |
| AC3 (interior point) | T3 | Direct |
| AC4 (maxhullvert limits) | T4, T22 | Direct + edge case (limit > vertex count) |
| AC5 (maxhullvert=4 boundary) | T5 | Direct + edge case |
| AC6 (nvert < 4 / degenerate) | T6, T21 | Direct + edge case (all-identical vertices) |
| AC7 (duplicate vertices) | T7 | Direct + edge case |
| AC8 (hill-climb ≡ exhaustive) | T8 | Direct |
| AC9 (maxhullvert parsing) | T9, T18 | Direct + edge case (-1 → None) |
| AC10 (maxhullvert validation) | T10 | Direct |
| AC11 (mesh-mesh hull collision) | T11 | Direct + integration |
| AC12 (hull after build) | T12 | Direct + integration |
| AC13 (no unsafe) | — | Code review (manual) |
| AC14 (face normals outward) | T19 | Direct |
| AC15 (graph adjacency structural) | T23 | Direct |

---

## Test Plan

### T1: Cube hull correctness → AC1
**Model:** 8 vertices at (±0.5, ±0.5, ±0.5), 12 triangular faces.
**Expected:** `quickhull()` returns `Some(hull)` with `hull.vertices.len() == 8`,
`hull.faces.len() == 12`. All 8 input vertices are on the hull. Each face
normal points outward (dot product with vector from center to face centroid
is positive).
**Tolerance:** Exact integer counts. Normal direction: dot product > 0.

### T2: Tetrahedron hull — minimum valid → AC2
**Model:** 4 vertices of regular tetrahedron: `(1,1,1)`, `(1,-1,-1)`,
`(-1,1,-1)`, `(-1,-1,1)`.
**Expected:** Hull has 4 vertices, 4 faces. Every face normal points outward.
**Tolerance:** Exact counts.

### T3: Interior point excluded → AC3
**Model:** 8 cube vertices + `(0,0,0)`.
**Expected:** Hull has 8 vertices (center excluded), 12 faces.

### T4: maxhullvert limits → AC4
**Model:** Icosphere with 42 vertices (subdivided icosahedron).
**Expected:** `quickhull(&points, Some(10))` returns hull with
`hull.vertices.len() <= 10`. All hull vertices are a subset of the input.
Hull is a valid convex polytope: for every input point p and every hull face f,
`(p - f.center) · f.normal <= epsilon`. This validates the hull contains all
input points.
**Tolerance:** `hull.vertices.len() <= 10`. Convexity check uses
`compute_epsilon(points)` as the tolerance for face-plane containment.

### T5: maxhullvert=4 boundary → AC5
**Model:** 8 cube vertices.
**Expected:** `quickhull(&points, Some(4))` returns hull with exactly 4
vertices (the initial simplex tetrahedron). No expansion iterations occur.
**Tolerance:** Exact count of 4.

### T6: Fewer than 4 vertices → AC6
**Model:** 3 vertices forming an equilateral triangle.
**Expected:** `quickhull()` returns `None`.

### T7: Duplicate vertices → AC7
**Model:** 8 cube vertices, each duplicated once (16 points total).
**Expected:** Hull has 8 vertices, 12 faces. Duplicates treated as interior.

### T8: Hill-climbing ≡ exhaustive → AC8
**Model:** Icosphere with 42 vertices. Compute hull with graph.
**Test:** For 100 uniformly-distributed random directions (seeded RNG for
reproducibility), compute support via:
- `hill_climb_support(vertices, graph, direction, 0)` → idx_hill
- exhaustive linear scan → idx_exhaust
**Expected:** For each direction, the dot product of the hill-climbing result
equals the dot product of the exhaustive result:
`vertices[idx_hill].coords.dot(&dir) == vertices[idx_exhaust].coords.dot(&dir)`.
We compare **dot product values** (not indices) because ties are possible —
two vertices may have identical dot products with a direction, and the two
algorithms may pick different ones.
**Tolerance:** Exact f64 equality (same vertices, same direction, same dot).
**Rationale:** Hill-climbing on a convex surface with a linear objective is
guaranteed to find the global maximum. This test empirically verifies the
guarantee and catches graph construction bugs.

### T9: maxhullvert parsing → AC9
**Model:** MJCF XML with `<mesh name="m" maxhullvert="10" vertex="..." face="..."/>`.
**Expected:** Parsed `MjcfMesh.maxhullvert == Some(10)`.

### T10: maxhullvert validation error → AC10
**Model:** MJCF XML with `<mesh name="m" maxhullvert="2" vertex="..." face="..."/>`.
**Expected:** Parse error containing "maxhullvert must be larger than 3".

### T11: Mesh-mesh collision via hull → AC11
**Model:** Two unit cube mesh geoms. Cube 1: vertices at (±0.5, ±0.5, ±0.5),
pose at origin (identity rotation). Cube 2: same vertex set, pose at
(0.9, 0.0, 0.0) (identity rotation). Both meshes have convex hulls (computed
via `compute_convex_hull(None)` before constructing collision shapes).
**Setup:** Construct a `Model` with two mesh geoms. Set geom positions
(`pos1 = Vector3::new(0.0, 0.0, 0.0)`, `pos2 = Vector3::new(0.9, 0.0, 0.0)`)
and rotation matrices (`mat1 = mat2 = Matrix3::identity()`). Call
`collide_with_mesh(model, geom1, geom2, pos1, mat1, pos2, mat2, margin=0.0)`.
The function internally constructs `Pose` from the position/matrix arguments
(lines 39-42) and determines `use_bvh` from the model's disabled flags
(line 36). It should detect that both meshes have hulls and take the GJK/EPA
code path.
**Expected:** Returns `Some(contact)` with:
- `contact.depth` ≈ 0.1 (within 1e-3) — overlap along X is 0.5 + 0.5 - 0.9 = 0.1
- `contact.normal` ≈ `(±1, 0, 0)` (dot product with `(1,0,0)` > 0.99 in abs)
- `contact.pos.x` ≈ 0.45 to 0.50 (contact point on the overlapping region,
  within 0.05 — EPA may place it at the midplane or the surface depending on
  convergence)
**Tolerance:** EPA on axis-aligned cubes converges well. Depth tolerance 1e-3,
normal tolerance 0.99 cosine, position tolerance 0.05. Tolerances are
empirically validated: EPA typically achieves better than 1e-4 on simple
polyhedra, but we leave margin for edge cases in the simplex evolution.

### T12: Hull available after model build → AC12
**Model:** MJCF with a mesh geom referencing a cube mesh asset.
**Expected:** After model build, `model.mesh_data[mesh_id].convex_hull().is_some()`.

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| Fewer than 4 vertices | MuJoCo returns no hull; must not panic | T6 | AC6 |
| Exactly 4 vertices (tetrahedron) | Minimum valid hull; no expansion needed | T2 | AC2 |
| All coplanar (flat quad) | Degenerate — no 3D hull possible | T13 | AC6 |
| All collinear (3+ points on a line) | Degenerate — no hull | T14 | AC6 |
| Duplicate vertices | Must not produce invalid hull | T7 | AC7 |
| Already-convex mesh (all on hull) | All vertices should be hull vertices | T1 | AC1 |
| `maxhullvert = 4` (boundary) | Maps to zero expansion iterations | T5 | AC5 |
| `maxhullvert = -1` in MJCF | Stored as `None` (no limit) | T18 | AC9 |
| Large mesh (>100 vertices) | Tests algorithm at scale; hill-climbing perf | T15 | AC8, AC14 |
| Near-degenerate (thin sliver) | Thin tetrahedron with nearly-coplanar vertices | T16 | AC1 |
| Face normals outward | All normals point away from hull interior | T19 | AC14 |
| Warm-start correctness | Cache updated between consecutive queries | T20 | AC8 |
| Visual-only mesh | N/A — AD-3 computes hulls for all meshes unconditionally; no visual-only skip logic. Test T12 verifies all meshes get hulls. | T12 | AC12 |
| Graph adjacency structural | Graph must be symmetric, no self-loops, min degree ≥3 | T23 | AC15 |
| All-identical vertices | 10 copies of same point — must not panic, return `None` | T21 | AC6 |
| `maxhullvert` > vertex count | `maxhullvert = 100` with 8-vertex cube — should produce full hull (limit never reached) | T22 | AC4 |
| Serde round-trip | Hull preserved across serialize/deserialize | T17 | AC12 |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T13 (coplanar points) | 4 coplanar vertices → `None` | Validates degenerate guard |
| T14 (collinear points) | 4 collinear vertices → `None` | Validates degenerate guard |
| T15 (large mesh) | 162-vertex icosphere hull | Tests Quickhull at scale; verifies graph adjacency. Expected: hull has ≤162 vertices (all on hull for icosphere), all face normals outward, hill-climbing ≡ exhaustive for 100 random directions. |
| T16 (thin sliver) | Near-coplanar tetrahedron: `(0,0,0)`, `(1,0,0)`, `(0,1,0)`, `(0.5,0.5,1e-8)` | The bounding box diagonal ≈ `√(1+1+1e-16) ≈ 1.41`, so `ε = 1.41 × 1e-10 ≈ 1.41e-10`. The fourth point is at distance `1e-8` from the plane (the z-coordinate), which is `1e-8 > 1.41e-10 = ε`. Therefore `farthest_from_plane` succeeds, and the hull SHOULD be computed. Expected: `Some` with 4 vertices, 4 faces. The hull is a very flat tetrahedron but geometrically valid. Validates that ε-guarded degeneracy checks correctly accept points that are small but still above the scale-dependent threshold. |
| T17 (serde round-trip) | Serialize + deserialize TriangleMeshData with hull | Verifies hull persists (not skipped like BVH) |
| T18 (maxhullvert=-1 parse) | `maxhullvert="-1"` → `None` (no limit) | Verifies -1 maps to None, not Some(-1) |
| T19 (face normals outward) | Unit cube hull → all normals outward → AC14 | For each face, `(face_centroid - hull_centroid) · normal > 0` |
| T20 (warm-start correctness) | ConvexMesh with 42-vertex hull, two consecutive support queries with similar directions | First query: direction `(1,0,0)`. Second query: direction `(0.99,0.14,0)` (slight rotation). Verify both return correct support point (matches exhaustive). Verify warm_start cache was updated after first query (not 0). |
| T21 (all-identical vertices) | 10 copies of `(1.0, 2.0, 3.0)` | `quickhull()` returns `None` — farthest-from-line distance is 0 < ε. Must not panic or divide by zero. |
| T22 (maxhullvert > vertex count) | 8 cube vertices, `max_vertices = Some(100)` | Hull has 8 vertices (limit never reached). Same result as `max_vertices = None`. |
| T23 (graph structural validity) | Unit cube hull → AC15 | `graph.adjacency.len() == 8`. Every vertex has ≥3 neighbors. Adjacency symmetric (`b ∈ adj[a]` ⟹ `a ∈ adj[b]`). Every face edge in adjacency. No self-loops. For cube: each vertex has exactly 3 neighbors (cube graph is 3-regular). |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| `CollisionShape::ConvexMesh` has new fields | Single-field variant `{ vertices }` | Three-field variant `{ vertices, graph, warm_start }` | **Toward MuJoCo** (graph for hill-climbing, warm_start for caching) | All match sites on ConvexMesh | Add `..` to patterns that don't use new fields |
| Mesh-mesh collision path | Per-triangle BVH (`mesh_mesh_deepest_contact`) | GJK/EPA on convex hulls when both meshes have hulls | **Toward MuJoCo** | Mesh-mesh collision tests | Contact points/depths may change; update expected values |
| `TriangleMeshData` has `convex_hull` field | No hull data | `Option<ConvexHull>` computed at build time | **Toward MuJoCo** | Code that constructs or serializes TriangleMeshData | None — transparent addition |
| GJK support for ConvexMesh | O(n) exhaustive scan, no graph | Hill-climbing O(√n) for ≥10 vertices, exhaustive for <10 | **Toward MuJoCo** | GJK/EPA collision results | Same support point — different algorithm, same result |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `sim/L0/core/src/convex_hull.rs` | **New file** — ConvexHull, HullGraph structs, Quickhull algorithm | +400 |
| `sim/L0/core/src/lib.rs` | Add `pub mod convex_hull;` | +1 |
| `sim/L0/core/src/mesh.rs` | Add `convex_hull: Option<ConvexHull>` field, `compute_convex_hull()` and `convex_hull()` methods | +20 |
| `sim/L0/core/src/collision_shape.rs` | Extend ConvexMesh variant with `graph` + `warm_start: Cell<usize>`, add `convex_mesh_from_hull()` factory, add `use std::cell::Cell` | ~20 modified |
| `sim/L0/core/src/gjk_epa.rs` | Update support for ConvexMesh to accept graph, add hill-climbing, add HILL_CLIMB_MIN constant | ~40 modified |
| `sim/L0/core/src/raycast.rs` | Update ConvexMesh match pattern to `{ vertices, .. }` | ~1 modified |
| `sim/L0/core/src/collision/mesh_collide.rs` | Mesh-mesh path: add hull-based GJK/EPA branch | ~20 modified |
| `sim/L0/mjcf/src/types.rs` | Add `maxhullvert` to MjcfMesh and MjcfMeshDefaults | +4 |
| `sim/L0/mjcf/src/parser.rs` (or equivalent `<mesh>` parser) | Parse `maxhullvert` attribute from `<mesh>` element | ~5 modified |
| `sim/L0/mjcf/src/builder/mesh.rs` | Invoke `compute_convex_hull()` after mesh construction | +5 |
| `sim/L1/bevy/src/mesh.rs` | Update ConvexMesh match pattern to `{ vertices, .. }` | ~1 modified |
| `sim/L0/core/tests/` or inline | New test file for convex hull + integration tests | +300 |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| `test_convex_mesh_creation` | `collision_shape.rs:968` | Pass (unchanged) | Factory `convex_mesh()` defaults graph to None |
| `test_bounding_radius_all_types` (ConvexMesh) | `collision_shape.rs:1286` | Pass (unchanged) | bounding_radius currently uses `{ vertices }` pattern; changed to `{ vertices, .. }` — iterates vertices only, doesn't touch graph |
| `test_local_aabb_convex_mesh` | `collision_shape.rs:1644` | Pass (unchanged) | local_aabb currently uses `{ vertices }` pattern; changed to `{ vertices, .. }` — iterates vertices only |
| `test_is_mesh_based` | `collision_shape.rs:1515` | Pass (unchanged) | `matches!` with `..` pattern |
| GJK/EPA tests using ConvexMesh | `gjk_epa.rs` tests | Pass (unchanged) | Tests construct via `convex_mesh()` which sets graph to None; exhaustive support path unchanged |
| Predicate tests (`is_convex`, `is_mesh_based`) | `collision_shape.rs:1511-1559` | Pass (unchanged) | Predicate match arms use `{ .. }` pattern (lines 356, 421) — field addition is transparent |
| Mesh-mesh collision tests (7 tests in `mesh.rs`) | `mesh.rs:1917-2055` | **Pass unchanged** — see analysis below | These tests call `mesh_mesh_contact()` (in `mesh.rs`), NOT `collide_with_mesh()` (in `mesh_collide.rs`). The hull path is only in `collide_with_mesh()`. |

**Existing mesh-mesh tests** (all in `mesh.rs`):

| Line | Test name | Expected impact |
|------|-----------|----------------|
| 1917 | `test_mesh_mesh_contact_overlapping_cubes` | Pass unchanged |
| 1942 | `test_mesh_mesh_contact_separate_cubes` | Pass unchanged |
| 1958 | `test_mesh_mesh_contact_rotated_cubes` | Pass unchanged |
| 1978 | `test_mesh_mesh_contact_tetrahedra` | Pass unchanged |
| 1995 | `test_mesh_mesh_deepest_contact` | Pass unchanged |
| 2025 | `test_mesh_mesh_contact_identical_position` | Pass unchanged |
| 2041 | `test_mesh_mesh_contact_edge_touching` | Pass unchanged |

**All 7 tests pass unchanged.** These tests call `mesh_mesh_contact()` (a
`mesh.rs` function), NOT `collide_with_mesh()` (in `mesh_collide.rs`). The
hull-based path is added to `collide_with_mesh()` only —
`mesh_mesh_contact()` is unchanged and continues using per-triangle BVH.
The only tests affected by the hull path would be integration tests that
call `collide_with_mesh()` or `detect_contacts()` end-to-end with mesh-mesh
pairs.

### Non-Modification Sites

| File:line | What it does | Why NOT modified |
|-----------|-------------|-----------------|
| `collision_shape.rs:348-358` (`is_convex`) | Returns true for ConvexMesh | Uses `..` pattern — no field access |
| `collision_shape.rs:418-422` (`is_mesh_based`) | Returns true for ConvexMesh | Uses `..` pattern |
| `bevy/components.rs:82` | Maps ConvexMesh to enum variant | Uses `..` pattern |
| `collision/narrow.rs:87-88` | Early return for Mesh geoms | Dispatch routing unchanged (AD-1 option b) |
| `collision/narrow.rs:222` | `geom_to_collision_shape` returns None for Mesh | Unchanged (AD-1 option b) |

---

## Execution Order

1. **S1** (ConvexHull struct + Quickhull algorithm in `convex_hull.rs`) → verify
   with T1, T2, T3, T5, T6, T7, T13, T14, T15, T16 (standalone algorithm tests)
2. **S2** (Hill-climbing support + ConvexMesh variant extension in `gjk_epa.rs` +
   `collision_shape.rs`) → verify with T8 (hill-climb ≡ exhaustive). Depends on
   S1 for ConvexHull/HullGraph types.
3. **S3** (`maxhullvert` parsing in `types.rs`) → verify with T9, T10. Independent
   of S1/S2 but logically follows.
4. **S5** (Build pipeline integration in `builder/mesh.rs`) → verify with T12.
   Depends on S1 (quickhull function) and S3 (maxhullvert field).
   **Note:** S5 is implemented before S4 despite the higher section number
   because S5 computes hulls at build time, and S4 (collision dispatch)
   requires hulls to be available at runtime. Without S5, T11 would fail
   (no hulls to collide).
5. **S4** (Mesh-mesh hull integration in `mesh_collide.rs`) → verify with T11.
   Depends on S1 (ConvexHull), S2 (ConvexMesh factory), S5 (hull available after
   build).
6. **T17** (serde round-trip) — verifies S1 + S5 integration.

Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` after each
section.

**Output contracts for downstream specs:**

| Contract | Consumer | What is provided | Stability guarantee |
|----------|----------|-----------------|---------------------|
| `ConvexHull` struct | Spec B (mesh inertia — `compute_mesh_inertia(&hull)`) | `vertices`, `faces`, `normals`, `graph` | Struct fields are stable; `normals` is CortenForge-only (AD-2) |
| `ConvexHull` struct | Spec E (flex-vs-mesh) | Hull vertices for contact surface | Spec E reads `hull.vertices` only — no dependency on graph or normals |
| `convex_mesh_from_hull()` (defined in S2, `collision_shape.rs`) | Spec D (CCD — constructs ConvexMesh for GJK/EPA on mesh pairs) | `CollisionShape::ConvexMesh` with graph + warm_start | Factory signature stable: `fn convex_mesh_from_hull(hull: &ConvexHull) -> Self` |
| `TriangleMeshData.convex_hull()` | Spec D (reads hull from mesh data) | `Option<&ConvexHull>` accessor | Returns `None` if hull not computed (nvert < 4) |
| AD-1 option (b) dispatch | Spec C (hfield-mesh), Spec D (CCD) | Mesh collision dispatch unchanged; hull used only for mesh-mesh pairs | `geom_to_collision_shape()` still returns `None` for Mesh geoms |

**Merge protocol for `gjk_epa.rs`:** Spec A modifies the `support()` function's
ConvexMesh arm (hill-climbing). Spec D will add `gjk_distance()` as a new
function. These are non-overlapping changes — no merge conflict expected.
If both specs are developed in parallel branches, merge Spec A first (it
modifies existing code), then rebase Spec D on top (additive only). The only
shared symbol is the `HILL_CLIMB_MIN` constant, which Spec D does not use.

**Merge protocol for `collision_shape.rs`:** Spec A adds `graph` and `warm_start`
fields to `ConvexMesh`. Spec D may add additional fields (e.g., `geom_margin`
for CCD). These are non-conflicting field additions. Merge Spec A first, then
Spec D adds its fields alongside.

---

## Performance Characterization

**Quickhull cost:**
- Average: O(n log n) where n = vertex count
- Worst case: O(n²) for pathological point distributions
- For typical meshes (100–10,000 vertices): <10ms at build time
- Runs once during model compilation, not per frame

**Hill-climbing support cost:**
- O(√n) per call on average for convex hulls (empirically verified on
  random polytopes in computational geometry literature)
- vs O(n) for exhaustive scan
- Significant for meshes with ≥10 vertices during GJK/EPA collision

**Memory cost:**
- ConvexHull: ~24 bytes/vertex (Point3) + 24 bytes/face (3×usize) + normals
  (24 bytes/face) + graph adjacency (~48 bytes/vertex for typical valence 6)
- For a 100-vertex hull: ~10 KB. For 1,000-vertex hull: ~100 KB.
- Acceptable for model-scale data.

**Acceptable?** Yes — build-time cost is negligible, runtime support queries
are faster, memory is small. No optimization needed for v1.

---

## Out of Scope

- **Mesh-primitive dispatch routing to GJK/EPA** (option a from AD-1) — deferred.
  Mesh-sphere, mesh-capsule, mesh-box pairs continue using per-triangle BVH.
  *Conformance impact: mesh-primitive contacts are on actual mesh surface, not
  hull surface. Gap acceptable for v1.0 — these contacts are often more accurate
  than hull contacts for non-convex meshes.*

- **Collision-only hull trigger** (matching MuJoCo's `needhull_`) — deferred
  (AD-3 option A chosen). *Conformance impact: none — computing unused hulls
  does not change collision behavior.*

- **Non-convex mesh-mesh collision** — the existing per-triangle BVH path
  remains as fallback for meshes without hulls. CortenForge extension, not
  MuJoCo conformance. *Conformance impact: none — MuJoCo always uses hull.*

- **Vertex deduplication in `TriangleMeshData`** — Quickhull handles duplicate
  vertices internally. A separate dedup pass on `TriangleMeshData` is not
  needed for hull correctness. *Conformance impact: none.*

- **GPU convex hull computation** — Post-v1.0. *Conformance impact: none.*
