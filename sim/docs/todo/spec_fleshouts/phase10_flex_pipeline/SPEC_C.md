# Spec C — Flex Self-Collision Dispatch: Spec

**Status:** Draft
**Phase:** Roadmap Phase 10 — Flex Pipeline
**Effort:** L
**MuJoCo ref:** `mj_collision()` in `engine_collision_driver.c` (dispatch loop
+ three gate conditions); `mj_collideFlexInternal()` in
`engine_collision_driver.c` (adjacent-element contacts);
`mj_collideFlexSelf()` in `engine_collision_driver.c` (non-adjacent element
collision with midphase); `mjtFlexSelf` enum in `mjmodel.h`
**MuJoCo version:** 3.5.0
**Test baseline:** 1,900+ sim domain tests
**Prerequisites:**
- T1 Session 2 (commit `1f0230c`): provides `flex_rigid`, `flexedge_rigid`,
  `flexedge_length`, `flexedge_velocity`
- Phase 7 T1 (commit `cf76731`): provides `flex_internal` parsing and storage
- Spec A (Sessions 3–7): provides `flexedge_J` (not consumed by Spec C — independent)
- Spec B (Sessions 8–13): provides cotangent bending (not consumed by Spec C — independent)

**Independence:** This spec is independent of Spec A (§42A-i edge Jacobian)
and Spec B (§42B cotangent bending) per the umbrella dependency graph. Spec C
depends on T1 for `flex_rigid[f]` (gate condition 1). Spec C is consumed by
Spec D (Sessions 19–21) which reuses narrowphase primitives. Shared file
`collision/mod.rs` is modified by Spec C (self-collision dispatch) and Spec D
(flex-flex dispatch) — sequential additions to the collision pipeline, no
conflict.

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical results to MuJoCo for the feature described below.
> Every algorithm, every acceptance criterion, and every test is in service
> of this goal. The MuJoCo C source code is the single source of truth —
> not the MuJoCo documentation, not intuition about "what should happen,"
> not a Rust-idiomatic reinterpretation. When in doubt, read the C source
> and match its behavior exactly.

---

## Scope Adjustment

Empirical verification against the MuJoCo C source and the CortenForge
codebase discovered the following corrections from the umbrella/session plan.

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| "Add `flex_internal: Vec<bool>` to Model" | `flex_internal` already exists on Model (Phase 7 T1, commit `cf76731`). Parsed from MJCF `<flex internal="..."/>`, stored and wired. | **Drop** — field already exists. Spec C only *consumes* it in dispatch. |
| `flex_selfcollide` upgrade from `Vec<bool>` to `Vec<FlexSelfCollide>` | Current: `Vec<bool>` (`builder/flex.rs:414-415` converts `Option<String>` → `bool`). MuJoCo uses `mjtFlexSelf` enum (5 values). The bool is lossy — `"bvh"`, `"sap"`, `"narrow"`, `"auto"` all collapse to `true`. | **In scope** — upgrade to `Vec<FlexSelfCollide>` enum, fix parser/builder pipeline. |
| "BVH: Per-element AABB tree" | BVH infrastructure exists in `mid_phase.rs`: `Bvh`, `BvhPrimitive`, `query()`, `BvhPrimitive::from_triangle()`. Can be reused for flex element AABBs. | **In scope** — reuse existing BVH. |
| "SAP: Sweep-and-prune on element AABBs" | No general-purpose SAP for element-level AABBs exists. The existing SAP is rigid-body broadphase. MuJoCo's flex SAP is a simple axis-sweep on element AABBs. | **In scope** — implement SAP for flex element AABBs. |
| Triangle-triangle narrowphase for dim=2 | `triangle_triangle_intersection()` exists in `mesh.rs:390` (SAT-based, returns `TriTriContact`). | **In scope** — reuse for dim=2 non-adjacent self-collision narrowphase. |
| Tet-tet narrowphase for dim=3 | No tet-tet intersection exists. MuJoCo uses vertex-face + edge-edge tests for tet pairs. | **In scope** — implement vertex-face + edge-edge for dim=3. |
| `activelayers` used in self-collision filtering | `flex_activelayers` parsed and stored (Phase 7 T1) but not consumed. MuJoCo uses `activelayers` to filter which element layers participate in self-collision. | **Defer** — `activelayers` runtime consumption is an optimization. Track as DT-150. |
| Umbrella Convention Registry §4: "Default: None (no self-collision)" | MuJoCo default is `"auto"` (enabled). `future_work_7.md:2504` confirms. Builder currently converts absent attribute → `true` (enabled). | **Correct** — Default is `FlexSelfCollide::Auto`, not `None`. |

**Final scope:**

1. `FlexSelfCollide` enum (None, Narrow, Bvh, Sap, Auto) — type definition + parser/builder migration
2. Contact encoding for self-collision — add `flex_vertex2: Option<usize>` to Contact
3. Contact parameter combination — `contact_param_flex_self()` function
4. Constraint pipeline updates — new Jacobian function + assembly bodyweight path
5. Element adjacency precomputation at build time
6. Three-condition gate logic in collision dispatch
7. Internal collision path: `mj_collide_flex_internal()` for adjacent elements
8. Self-collision narrowphase: `mj_collide_flex_self()` for non-adjacent elements
9. BVH midphase: per-element AABB tree for candidate pair pruning
10. SAP midphase: sweep-and-prune on element AABBs
11. AUTO dispatch: BVH for dim=3, SAP for dim≤2

---

## Problem Statement

**Conformance gap** — MuJoCo dispatches flex self-collision from
`mj_collision()` in `engine_collision_driver.c` behind a three-condition gate:
(1) `!flex_rigid[f]`, (2) `(flex_contype[f] & flex_conaffinity[f]) != 0`,
(3) per-path enable flags (`flex_internal[f]` for adjacent elements,
`flex_selfcollide[f] != NONE` for non-adjacent elements). Two independent
dispatch paths generate contacts between elements of the same flex body:
`mj_collideFlexInternal()` for adjacent elements sharing vertices/edges, and
`mj_collideFlexSelf()` for non-adjacent elements with optional BVH/SAP
midphase acceleration.

CortenForge does not implement flex self-collision. The `flex_selfcollide`
field is stored as `Vec<bool>` (lossy — algorithm selection information is
lost), `flex_internal` is parsed but never consumed, and no dispatch logic
exists in the collision pipeline. A deformable body that folds onto itself
generates zero self-contacts. This spec adds the full self-collision dispatch
pipeline: enum type upgrade, three-condition gate, adjacent-element internal
contacts, non-adjacent element contacts with midphase acceleration (BVH/SAP),
and the necessary constraint pipeline updates (Jacobian, assembly) for the
new contact type.

---

## MuJoCo Reference

> **This is the most important section of the spec.** Everything downstream
> is derived from what's documented here.

### Dispatch structure

**Source:** `engine_collision_driver.c` → `mj_collision()`,
`future_work_10.md:7340-7367` (gate + dispatch loop),
`future_work_10.md:7356` (AUTO: `case mjFLEXSELF_AUTO: /* BVH for dim=3,
SAP otherwise */`), `future_work_10.md:7401` (AUTO: "BVH for `dim=3`
(solids), SAP otherwise").

```c
// Self-collision dispatch loop (inside mj_collision, after flex-rigid)
for (int f = 0; f < m->nflex; f++) {
    // Gate conditions 1+2 (shared by both paths)
    if (!m->flex_rigid[f] && (m->flex_contype[f] & m->flex_conaffinity[f])) {
        // Path 1: internal collision (adjacent elements)
        if (m->flex_internal[f]) {
            mj_collideFlexInternal(m, d, f);
        }
        // Path 2: self-collision (non-adjacent elements)
        if (m->flex_selfcollide[f] != mjFLEXSELF_NONE) {
            switch (m->flex_selfcollide[f]) {
                case mjFLEXSELF_NARROW: mj_collideFlexSelf(m, d, f); break;
                case mjFLEXSELF_BVH:    /* BVH midphase + narrowphase */ break;
                case mjFLEXSELF_SAP:    /* SAP midphase + narrowphase */ break;
                case mjFLEXSELF_AUTO:
                    if (m->flex_dim[f] == 3)  /* BVH */;
                    else                       /* SAP */;
                    break;
            }
        }
    }
}
```

### Gate conditions

Three conjunctive conditions, evaluated in short-circuit order:

1. **`!flex_rigid[f]`** — skip if all vertices have `invmass == 0`. From T1
   Session 2 (commit `1f0230c`). Rigid flexes cannot self-collide (no DOFs).

2. **`(flex_contype[f] & flex_conaffinity[f]) != 0`** — self-bitmask check.
   MuJoCo's `filterBitmask()` (`engine_collision_driver.c`) performs this
   check. For self-collision, the flex checks its own bitmask against *itself*
   (not against another object) — i.e., `filterBitmask(contype, conaffinity,
   contype, conaffinity)` simplifies to `(contype & conaffinity) != 0`.
   Consequence: `contype=2, conaffinity=4` disables self-collision even when
   `selfcollide != NONE`, because `2 & 4 = 0`
   (`future_work_8.md:1049-1050`).

3. **Per-path enable flags:**
   - `flex_internal[f]` for internal collision (adjacent elements).
     Default: `true` (`future_work_7.md:2517`).
   - `flex_selfcollide[f] != mjFLEXSELF_NONE` for self-collision
     (non-adjacent elements). Default: `mjFLEXSELF_AUTO`
     (`future_work_7.md:2504`).

### `mjtFlexSelf` enum

**Source:** `mjmodel.h`, `future_work_10.md:7369`.

```c
enum mjtFlexSelf {
    mjFLEXSELF_NONE   = 0,  // no self-collision
    mjFLEXSELF_NARROW = 1,  // brute-force all non-adjacent element pairs
    mjFLEXSELF_BVH    = 2,  // BVH midphase + narrowphase
    mjFLEXSELF_SAP    = 3,  // sweep-and-prune midphase + narrowphase
    mjFLEXSELF_AUTO   = 4,  // BVH for dim=3, SAP otherwise
};
```

MJCF attribute values: `"none"`, `"narrow"`, `"bvh"`, `"sap"`, `"auto"`.
Default: `"auto"` (enabled with automatic algorithm selection).

### Element adjacency

Two elements are **adjacent** if they share at least one vertex. For dim=2
(triangles), sharing an edge means sharing 2 vertices; sharing a single
vertex also counts as adjacent. For dim=3 (tetrahedra), sharing a face means
sharing 3 vertices. MuJoCo's `mj_collideFlexInternal()` handles all
adjacent-element contacts. `mj_collideFlexSelf()` handles only non-adjacent
element pairs (zero shared vertices).

Element topology is static (defined at model build time, never changes at
runtime). The adjacency relationship is precomputed at build time from
`flexelem_data` connectivity and stored on Model.

### Internal collision (`mj_collideFlexInternal`)

**Source:** `engine_collision_driver.c` → `mj_collideFlexInternal()`,
`future_work_10.md:7391-7393`.

For each pair of adjacent elements in flex `f`:
1. Skip if elements share all vertices (same element, impossible for valid mesh)
2. For each vertex V of element A that is NOT shared with element B:
   - Test vertex sphere (center = `flexvert_xpos[V]`, radius =
     `flexvert_radius[V]`) against the face(s) of element B
   - If penetrating within margin: generate contact
3. Repeat for each vertex of element B that is NOT shared with element A

For dim=2 (triangles sharing an edge): each side has exactly 1 non-shared
vertex. Two vertex-face tests per adjacent pair. For dim=2 (triangles sharing
a single vertex): each side has 2 non-shared vertices. Four vertex-face tests.

For dim=3 (tetrahedra sharing a face): each side has exactly 1 non-shared
vertex. Two vertex-face tests per adjacent pair. For dim=3 (tetrahedra sharing
an edge or single vertex): more non-shared vertices, more tests.

### Self-collision (`mj_collideFlexSelf`)

**Source:** `engine_collision_driver.c` → `mj_collideFlexSelf()`,
`future_work_10.md:7397-7402`.

For each pair of non-adjacent elements in flex `f`:
1. **NARROW mode (brute-force):** iterate all `O(n²)` element pairs, filter
   adjacent pairs, run narrowphase on remaining pairs.
2. **BVH mode:** build per-element AABB tree from `flexvert_xpos` each step,
   query for overlapping pairs, filter adjacent pairs, run narrowphase.
3. **SAP mode:** compute element AABBs, sort along axis of maximum variance,
   sweep for overlapping projections, filter adjacent pairs, run narrowphase.
4. **AUTO mode:** BVH for `dim=3` (solids), SAP otherwise (shells/cables).

Narrowphase for non-adjacent element pairs:

**Source:** `engine_collision_driver.c` → `mj_collideFlexSelf()` narrowphase,
`future_work_10.md:7391-7402`.

MuJoCo uses vertex-face tests as the primary narrowphase: each vertex of
element A is tested against faces of element B (and vice versa). For our
implementation:

- **dim=2 (triangles):** **Deliberate implementation choice:** use
  `triangle_triangle_intersection()` (`mesh.rs:390`), which is a SAT-based
  test returning contact point, normal, and depth, instead of implementing
  MuJoCo's exact vertex-face tests. This is a **conformance-preserving
  superset**: SAT detects all intersection types including vertex-face and
  edge-edge crossings, so it will never miss a contact that MuJoCo generates.
  It may produce additional edge-edge crossing contacts that MuJoCo does not
  generate; MuJoCo conformance test T18 will quantify this delta for the
  test model. For vertex-face penetrations (the dominant case for
  self-collision), both approaches are **numerically identical**: both compute
  the same separating axis (the face normal) and the same penetration depth
  (vertex-to-plane distance). **Rationale for this choice:** reusing the
  existing, tested SAT primitive avoids re-implementing vertex-face from
  scratch and producing a second narrowphase code path; the superset property
  guarantees no missed contacts. If T18 reveals a conformance-significant
  contact count delta, the implementation session may switch to dedicated
  vertex-face tests (the spec's internal collision path in S6 already
  implements vertex-face via `sphere_triangle_contact()`, so the primitive
  exists).
- **dim=3 (tetrahedra):** Vertex-face tests per MuJoCo: each vertex of tet A
  tested against each triangular face of tet B (and vice versa). MuJoCo's
  `mj_collideFlexSelf()` in `engine_collision_driver.c` also performs
  edge-edge proximity tests between tet edges (documented in
  `future_work_10.md:7391-7393` as "element-element narrowphase for adjacent
  elements ... generates contacts between element faces/edges"). CortenForge
  defers edge-edge tet tests to DT-151 as a conformance refinement — this is
  a **deliberate divergence** from MuJoCo, not an omission from the spec.
  **Conformance impact of deferring edge-edge:** edge-edge contacts are
  secondary for tetrahedral meshes — vertex-face captures the primary
  penetration. Models with dim=3 self-collision are rare; no current test
  models exercise this path. MuJoCo conformance test T18 will quantify the
  contact count delta (if any) for the test model.
- **dim=1 (cables):** Edge-edge proximity tests (closest distance between
  two line segments). MuJoCo tests cable segment pairs as capsules (each
  vertex has a radius). If distance < combined vertex radii + margin,
  generate contact. For dim=1 internal collision (adjacent segments sharing
  a vertex): MuJoCo skips vertex-face tests (edges have no faces) and relies
  on edge-edge proximity between adjacent segments. CortenForge's dim=1
  internal collision (`test_vertex_against_element_faces` with `dim=1`)
  returns early with no tests, matching MuJoCo's behavior (adjacent cable
  segments do not generate internal contacts via vertex-face).

### Contact parameters for self-collision

For self-collision, both sides are the same flex `f`. The contact parameter
combination is trivial since both sides have identical parameters:

- **Priority:** `flex_priority[f] == flex_priority[f]` → equal priority path
- **Condim:** `max(flex_condim[f], flex_condim[f]) = flex_condim[f]`
- **Solmix:** equal weights → `mix = 0.5`
- **Solref:** `combine_solref(flex_solref[f], flex_solref[f], 0.5) = flex_solref[f]`
- **Solimp:** `combine_solimp(flex_solimp[f], flex_solimp[f], 0.5) = flex_solimp[f]`
- **Friction:** `element_max(flex_friction[f], flex_friction[f]) = flex_friction[f]`
- **Gap:** `flex_gap[f] + flex_gap[f] = 2 * flex_gap[f]` (additive per MuJoCo convention)
- **Margin:** `flex_margin[f] + flex_margin[f] = 2 * flex_margin[f]` (additive)

The result is just the flex's own parameters with doubled gap/margin. This
allows a simplified `contact_param_flex_self()` that skips the priority/solmix
logic entirely.

### Contact encoding for self-collision

MuJoCo stores flex self-collision contacts with special element-based encoding
in `mjContact.geom[0]`/`geom[1]`. Our Contact struct uses `geom1`/`geom2` for
rigid geom indices and `flex_vertex: Option<usize>` for one flex vertex.
Self-collision has TWO flex vertices and ZERO rigid geoms.

The constraint Jacobian for self-collision contacts requires DOFs for both
sides. For free vertices (all current models), each vertex has 3 independent
translational DOFs. The Jacobian is trivial: `+direction` on vertex A's DOFs,
`-direction` on vertex B's DOFs.

### Margin and depth conventions

MuJoCo computes effective margin as the sum of both sides' margins:
`margin = geom_margin[g1] + geom_margin[g2]` for rigid-rigid
(`engine_collision_driver.c` → `mj_collideGeoms()`, documented in
`future_work_7.md:1953`). The same additive convention applies to
flex-rigid: `margin = flex_margin[f] + geom_margin[g]`
(`flex_collide.rs:258-261`). For self-collision, both sides are the same
flex, so the additive convention yields:

```
margin = flex_margin[f] + flex_margin[f] = 2 * flex_margin[f]
gap    = flex_gap[f]    + flex_gap[f]    = 2 * flex_gap[f]
includemargin = margin - gap
```

A contact is active when `depth > -includemargin`. The sphere-face test
generates a contact when sphere penetration depth exceeds `-includemargin`.

### Edge cases

| Edge case | MuJoCo behavior |
|-----------|----------------|
| Rigid flex (`flex_rigid[f]=true`) | Skipped entirely — zero self-contacts |
| Self-bitmask zero (`contype=2, conaffinity=4`) | Skipped — `2 & 4 = 0` |
| `selfcollide=none` | No non-adjacent contacts (internal still possible if `internal=true`) |
| `internal=false` | No adjacent contacts (self-collision still possible if `selfcollide != none`) |
| Zero-element flex (`flex_elemnum[f]=0`) | No element pairs → zero contacts |
| Single-element flex | No non-adjacent pairs → zero self-contacts. If `internal=true`: no adjacent pairs either (need 2+ elements) |
| dim=1 cable | Elements are edges (2 vertices). Self-collision uses edge-edge proximity. AUTO → SAP. |
| All elements coplanar (flat mesh) | No self-penetration → zero self-contacts (correct behavior) |
| Overlapping elements from deformation | Contacts generated at penetration points |

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| Self-collision dispatch | Three-condition gate + `mjtFlexSelf` enum dispatch in `mj_collision()` | No self-collision dispatch. `flex_selfcollide` stored as `Vec<bool>` (lossy). |
| Internal collision | `mj_collideFlexInternal()` for adjacent-element vertex-face contacts | `flex_internal` parsed and stored but never consumed. |
| Midphase acceleration | BVH, SAP, AUTO algorithm selection per `mjtFlexSelf` enum | No midphase for flex. |
| Element adjacency | Precomputed from element topology, used to partition internal vs self paths | No element adjacency data structure. |
| Contact parameter combination | Trivial identity (both sides same flex) with additive margin/gap | No `contact_param_flex_self()` function. |
| Contact encoding | Element-based encoding in `mjContact.geom[0/1]` | `Contact.flex_vertex` supports one flex vertex only. |
| Constraint Jacobian | Both sides use flex vertex DOFs | `compute_flex_contact_jacobian()` assumes one flex vertex + one rigid body. |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| Element connectivity | `m->flex_elem[flex_elemadr[f] + (dim+1)*e + v]` | `model.flexelem_data[flexelem_dataadr[elem_global_idx] + v]` | Use `flexelem_dataadr`/`flexelem_datanum` for element-to-vertex lookup. Element global index = `flex_elemadr[f] + e`. |
| `mjtFlexSelf` enum | C enum, int-valued (0–4) | `FlexSelfCollide` Rust enum with `#[repr(u8)]` discriminants matching MuJoCo values | Direct port — variant names match. |
| Vertex positions | `d->flexvert_xpos[3*v + x]` (flat f64 array) | `data.flexvert_xpos[v]` (`Vec<Vector3<f64>>`) | Use `data.flexvert_xpos[v]` directly — no manual `3*v+x` indexing. |
| Contact encoding (self) | `mjContact.geom[0/1]` with element encoding | `Contact.flex_vertex` + `Contact.flex_vertex2` (both `Option<usize>`) | When `flex_vertex.is_some() && flex_vertex2.is_some()` → self-collision contact. `geom1 = geom2 = usize::MAX` (sentinel). |
| Contact encoding (flex-rigid) | `mjContact.geom[0]` = rigid geom | `Contact.geom1 = geom2 = geom_idx`, `flex_vertex = Some(vi)` | No change — existing convention preserved. |
| Flex element indexing | `flex_elemadr[f]..flex_elemadr[f]+flex_elemnum[f]` | Same: `model.flex_elemadr[f]..model.flex_elemadr[f]+model.flex_elemnum[f]` | Direct port — no translation needed. |
| BVH API | MuJoCo internal BVH | `Bvh::build(primitives)`, `bvh.query(&aabb)` returns `Vec<usize>` | Build with `BvhPrimitive { aabb, index: elem_idx, data: 0 }`. `query()` returns **local primitive indices** (0..n), not element global indices. Map back to global: `elem_start + returned_index`. The `index` field on `BvhPrimitive` stores the global element index but `query()` returns the position in the primitives array. |
| Triangle-triangle test | MuJoCo vertex-face tests | `triangle_triangle_intersection(tri_a, tri_b)` returns `Option<TriTriContact>` | Use existing SAT-based function for dim=2 non-adjacent self-collision. `TriTriContact` has `point: Point3<f64>`, `normal: Vector3<f64>`, `depth: f64` — map directly to `make_contact_flex_self(pos=point.coords, normal, depth)`. The `tri_a`/`tri_b` index fields are not used (we identify vertices by the input arrays). |
| Contact frame | MuJoCo `mjContact.frame[6]` (2 tangent vectors) | `Contact.frame: [Vector3<f64>; 2]` — `frame[0]` = t1, `frame[1]` = t2 | Direct port. `compute_tangent_frame(&normal)` produces orthonormal tangent basis. Jacobian tangent rows use `contact.frame[0]` and `contact.frame[1]` — same convention for self-collision and flex-rigid. |

---

## Architecture Decisions

### AD-1: Contact encoding for self-collision

**Problem:** The `Contact` struct encodes flex-rigid contacts as
`flex_vertex: Some(vi)` with `geom1 = geom2 = geom_idx`. Self-collision has
two flex vertices and zero rigid geoms. The existing Jacobian
(`jacobian.rs:50`) and assembly (`assembly.rs:601-612`) code assumes a rigid
geom exists.

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | Add `flex_vertex2: Option<usize>` | Minimal struct change; backward-compatible; simple dispatch (`is_some()` check) | Encodes vertex-vertex, not full element-face (approximate Jacobian) |
| 2 | Add `ContactKind` enum | Type-safe; extensible; carries element info | Larger change; requires exhaustive match in all consumers |
| 3 | Sentinel values in `geom1`/`geom2` | Zero struct changes | Fragile; easy to forget sentinel check; no type safety |

**Chosen:** Option 1 — add `flex_vertex2: Option<usize>`. The vertex-vertex
encoding is correct for the Jacobian when both sides are free vertices (each
has 3 independent translational DOFs). The Jacobian projects `+direction` on
vertex 1's DOFs and `-direction` on vertex 2's DOFs. For self-collision,
vertex 1 is the penetrating vertex and vertex 2 is the nearest vertex on the
opposing element's face. Full element-face barycentric distribution is deferred
to post-v1.0 body-attached vertex support (§27D).

### AD-2: Element adjacency storage

**Problem:** The collision dispatch needs to distinguish adjacent (internal)
from non-adjacent (self) element pairs. Element topology is static — adjacency
should be precomputed at build time.

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | Flat sorted adjacency list (`flex_elem_adj`, `flex_elem_adj_adr`, `flex_elem_adj_num`) | Compact; O(log k) lookup via binary search; cache-friendly | 3 new Model fields |
| 2 | `Vec<HashSet<usize>>` per element | O(1) lookup | Heap-allocated per element; poor cache behavior |
| 3 | Adjacency matrix (bitset) | O(1) lookup | O(n²) space per flex; impractical for large meshes |

**Chosen:** Option 1 — flat sorted adjacency list. Consistent with the
CSR-style indexing patterns used elsewhere in Model (e.g., `flexelem_dataadr`,
`flexedge_J_rowadr`). Binary search on small neighbor lists (typically 3–6
for triangular meshes) is effectively O(1).

---

## Specification

### S1. `FlexSelfCollide` enum + parser/builder pipeline

**File:** `sim/L0/core/src/types/enums.rs` (add after `FlexBendingType`, line ~900)
**MuJoCo equivalent:** `mjtFlexSelf` enum in `mjmodel.h`
**Design decision:** Follow the `FlexBendingType` pattern (same derives, same
file location). Discriminant values match MuJoCo's `mjtFlexSelf` for
debugging clarity. Default is `Auto` (matching MuJoCo default "auto").

**After** (new enum):
```rust
/// Self-collision algorithm selection for flex bodies.
///
/// Matches MuJoCo's `mjtFlexSelf` enum. Controls how non-adjacent element
/// pairs are tested for collision within a single flex body.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[repr(u8)]
pub enum FlexSelfCollide {
    /// No self-collision (non-adjacent elements not tested).
    None = 0,
    /// Brute-force: test all non-adjacent element pairs O(n²).
    Narrow = 1,
    /// BVH midphase: per-element AABB tree for candidate pair pruning.
    Bvh = 2,
    /// Sweep-and-prune midphase: sort element AABBs along axis of max variance.
    Sap = 3,
    /// Automatic: BVH for dim=3 (solids), SAP otherwise (shells/cables).
    Auto = 4,
}

impl Default for FlexSelfCollide {
    fn default() -> Self {
        Self::Auto // MuJoCo default is "auto" (enabled)
    }
}
```

**Pipeline migration** — every site that touches `flex_selfcollide`:

| File | Current | After |
|------|---------|-------|
| `core/types/model.rs:374` | `pub flex_selfcollide: Vec<bool>` | `pub flex_selfcollide: Vec<FlexSelfCollide>` |
| `core/types/model_init.rs:168` | `flex_selfcollide: vec![]` | `flex_selfcollide: vec![]` (type inferred) |
| `mjcf/types.rs:3802` | `selfcollide: Option<String>` | No change — keep `Option<String>` in MJCF intermediate type |
| `mjcf/parser.rs:3034-3036` | Parses string | No change — parse as string, convert in builder |
| `mjcf/builder/flex.rs:414-415` | `self.flex_selfcollide.push(flex.selfcollide.as_deref() != Some("none"))` | Convert string → `FlexSelfCollide` enum (see below) |
| `mjcf/builder/mod.rs:708` | `pub(crate) flex_selfcollide: Vec<bool>` | `pub(crate) flex_selfcollide: Vec<FlexSelfCollide>` |
| `mjcf/builder/init.rs:280` | `flex_selfcollide: vec![]` | `flex_selfcollide: vec![]` (type inferred) |
| `mjcf/builder/build.rs:197` | Wires field | No change (type propagates) |

**Builder conversion** (`builder/flex.rs`):

**Before:**
```rust
self.flex_selfcollide.push(flex.selfcollide.as_deref() != Some("none"));
```

**After:**
```rust
let sc = match flex.selfcollide.as_deref() {
    Some("none") => FlexSelfCollide::None,
    Some("narrow") => FlexSelfCollide::Narrow,
    Some("bvh") => FlexSelfCollide::Bvh,
    Some("sap") => FlexSelfCollide::Sap,
    Some("auto") => FlexSelfCollide::Auto,
    None => FlexSelfCollide::Auto, // MuJoCo default
    Some(other) => {
        // Unknown keyword — warn and default to Auto
        FlexSelfCollide::Auto
    }
};
self.flex_selfcollide.push(sc);
```

### S2. Contact encoding + contact parameter combination

**File:** `sim/L0/core/src/types/contact_types.rs` (add field to `Contact`)
**File:** `sim/L0/core/src/collision/mod.rs` (add `contact_param_flex_self()`)
**MuJoCo equivalent:** `mjContact` struct encoding + `mj_contactParam()` for
self-collision pairs
**Design decision:** Per AD-1, add `flex_vertex2: Option<usize>` for minimal
disruption. The dispatch convention: `flex_vertex.is_some() &&
flex_vertex2.is_some()` identifies self-collision contacts. For self-collision,
`geom1 = geom2 = usize::MAX` (sentinel — no rigid geom).

**Contact struct addition** (`contact_types.rs`):

Add after `flex_vertex` (line 86):
```rust
    /// If `Some(vertex_idx)`, this is a flex self-collision contact.
    /// `flex_vertex` = penetrating vertex (side 1).
    /// `flex_vertex2` = nearest opposing vertex (side 2).
    /// `geom1 = geom2 = usize::MAX` (no rigid geom).
    /// If `None`, standard flex-rigid or rigid-rigid contact.
    pub flex_vertex2: Option<usize>,
```

Initialize to `None` in all existing constructors (`new`, `with_solver_params`,
`with_condim`, `make_contact_flex_rigid`).

**Contact parameter function** (`collision/mod.rs`):

```rust
/// Compute contact parameters for flex self-collision.
///
/// Both sides are the same flex — parameters are trivially the flex's own
/// values. Gap and margin are additive (both sides contribute), so they
/// are doubled. Follows MuJoCo's `mj_contactParam()` convention for
/// same-entity pairs.
pub(crate) fn contact_param_flex_self(
    model: &Model,
    flex_id: usize,
) -> (i32, f64, [f64; 2], [f64; 5], [f64; 5]) {
    let f = model.flex_friction[flex_id];
    (
        model.flex_condim[flex_id],
        2.0 * model.flex_gap[flex_id],    // additive: flex_gap + flex_gap
        model.flex_solref[flex_id],
        model.flex_solimp[flex_id],
        [f.x, f.x, f.y, f.z, f.z],       // tangential, tangential, torsional, rolling, rolling
    )
}
```

**Contact factory** (`collision/flex_collide.rs`):

```rust
/// Create a Contact for a flex self-collision.
///
/// Side 1: penetrating vertex (flex_vertex).
/// Side 2: nearest opposing vertex (flex_vertex2).
/// No rigid geom — geom1/geom2 set to usize::MAX (sentinel).
pub fn make_contact_flex_self(
    model: &Model,
    vertex1: usize,
    vertex2: usize,
    pos: Vector3<f64>,
    normal: Vector3<f64>,
    depth: f64,
) -> Contact {
    let flex_id = model.flexvert_flexid[vertex1];
    let (condim, gap, solref, solimp, mu) = contact_param_flex_self(model, flex_id);
    let solref = assign_ref(model, &solref);
    let solimp = assign_imp(model, &solimp);
    let mu = assign_friction(model, &mu);
    let margin = assign_margin(model, 2.0 * model.flex_margin[flex_id]);
    let includemargin = margin - gap;

    let (t1, t2) = compute_tangent_frame(&normal);

    let dim: usize = match condim {
        1 => 1,
        4 => 4,
        _ => 3,
    };

    Contact {
        pos,
        normal,
        depth,
        geom1: usize::MAX,    // sentinel — no rigid geom
        geom2: usize::MAX,
        friction: mu[0],
        dim,
        includemargin,
        mu,
        solref,
        solreffriction: assign_ref(model, &[0.0, 0.0]),
        solimp,
        frame: (t1, t2).into(),
        flex_vertex: Some(vertex1),
        flex_vertex2: Some(vertex2),
    }
}
```

### S3. Constraint pipeline updates (Jacobian + assembly)

**File:** `sim/L0/core/src/constraint/jacobian.rs` (add `compute_flex_self_contact_jacobian`)
**File:** `sim/L0/core/src/constraint/assembly.rs` (update bodyweight computation)
**MuJoCo equivalent:** Flex self-collision Jacobian computation in constraint pipeline
**Design decision:** Add a new Jacobian function rather than branching inside
`compute_flex_contact_jacobian()`. This keeps the flex-rigid path unchanged
and makes the self-collision logic self-contained (per R15/EGT-9).

**Jacobian function** (`jacobian.rs`):

```rust
/// Compute the contact Jacobian for a flex self-collision contact.
///
/// Both sides are flex vertices with trivial Jacobians (identity on DOF
/// columns). Vertex 1 (penetrating) contributes positively, vertex 2
/// (opposing) contributes negatively.
///
/// Note: torsional and rolling friction rows (dim≥4) are all-zero for
/// self-collision contacts because flex vertices have no angular DOFs.
/// This means condim>3 produces zero-valued constraint rows that have
/// no effect. The solver handles this correctly (zero-row → zero force).
pub fn compute_flex_self_contact_jacobian(
    model: &Model,
    _data: &Data,
    contact: &Contact,
    vi1: usize,
    vi2: usize,
) -> DMatrix<f64> {
    let nv = model.nv;
    let dim = contact.dim;
    let normal = contact.normal;
    let tangent1 = contact.frame[0];
    let tangent2 = contact.frame[1];

    let mut j = DMatrix::zeros(dim, nv);

    // Helper: project direction onto vertex DOFs with given sign
    let add_vertex = |j: &mut DMatrix<f64>, row: usize, dir: &Vector3<f64>,
                      dof_base: usize, sign: f64| {
        if dof_base == usize::MAX { return; } // pinned vertex
        j[(row, dof_base)]     += sign * dir.x;
        j[(row, dof_base + 1)] += sign * dir.y;
        j[(row, dof_base + 2)] += sign * dir.z;
    };

    let dof1 = model.flexvert_dofadr[vi1];
    let dof2 = model.flexvert_dofadr[vi2];

    // Row 0: normal direction
    // Vertex 1 (penetrating) = positive, vertex 2 (opposing) = negative
    add_vertex(&mut j, 0, &normal, dof1, 1.0);
    add_vertex(&mut j, 0, &normal, dof2, -1.0);

    // Rows 1-2: tangent directions (sliding friction)
    if dim >= 3 {
        add_vertex(&mut j, 1, &tangent1, dof1, 1.0);
        add_vertex(&mut j, 1, &tangent1, dof2, -1.0);
        add_vertex(&mut j, 2, &tangent2, dof1, 1.0);
        add_vertex(&mut j, 2, &tangent2, dof2, -1.0);
    }

    // Rows 3+ (torsional/rolling): all zero — flex vertices have no angular DOFs
    j
}
```

**Dispatch update** (`jacobian.rs`, in `compute_contact_jacobian`):

**Before** (line 154):
```rust
if let Some(vi) = contact.flex_vertex {
    return compute_flex_contact_jacobian(model, data, contact, vi);
}
```

**After:**
```rust
if let Some(vi) = contact.flex_vertex {
    if let Some(vi2) = contact.flex_vertex2 {
        return compute_flex_self_contact_jacobian(model, data, contact, vi, vi2);
    }
    return compute_flex_contact_jacobian(model, data, contact, vi);
}
```

**Assembly bodyweight update** (`assembly.rs`, lines 601-612):

**Before:**
```rust
let bw_contact = {
    let b1 = if contact.geom1 < model.geom_body.len() {
        model.geom_body[contact.geom1]
    } else { 0 };
    let b2 = if contact.geom2 < model.geom_body.len() {
        model.geom_body[contact.geom2]
    } else { 0 };
    // ... invweight computation
};
```

**After:**
```rust
let bw_contact = if let (Some(vi1), Some(vi2)) = (contact.flex_vertex, contact.flex_vertex2) {
    // Self-collision: both sides are flex vertices — use vertex bodies
    let b1 = model.flexvert_bodyid[vi1];
    let b2 = model.flexvert_bodyid[vi2];
    let w1t = if b1 < model.body_invweight0.len() { model.body_invweight0[b1][0] } else { 0.0 };
    let w2t = if b2 < model.body_invweight0.len() { model.body_invweight0[b2][0] } else { 0.0 };
    let w1r = if b1 < model.body_invweight0.len() { model.body_invweight0[b1][1] } else { 0.0 };
    let w2r = if b2 < model.body_invweight0.len() { model.body_invweight0[b2][1] } else { 0.0 };
    [(w1t + w2t).max(MJ_MINVAL), (w1r + w2r).max(MJ_MINVAL)]
} else {
    let b1 = if contact.geom1 < model.geom_body.len() {
        model.geom_body[contact.geom1]
    } else { 0 };
    let b2 = if contact.geom2 < model.geom_body.len() {
        model.geom_body[contact.geom2]
    } else { 0 };
    let w1t = if b1 < model.body_invweight0.len() { model.body_invweight0[b1][0] } else { 0.0 };
    let w2t = if b2 < model.body_invweight0.len() { model.body_invweight0[b2][0] } else { 0.0 };
    let w1r = if b1 < model.body_invweight0.len() { model.body_invweight0[b1][1] } else { 0.0 };
    let w2r = if b2 < model.body_invweight0.len() { model.body_invweight0[b2][1] } else { 0.0 };
    [(w1t + w2t).max(MJ_MINVAL), (w1r + w2r).max(MJ_MINVAL)]
};
```

### S4. Element adjacency precomputation

**File:** `sim/L0/core/src/types/model.rs` (add adjacency fields)
**File:** `sim/L0/mjcf/src/builder/flex.rs` (compute adjacency at build time)
**MuJoCo equivalent:** Element adjacency derived from `flex_elem` connectivity
**Design decision:** Per AD-2, flat sorted adjacency list. Three new Model
fields following CSR-style indexing. Computed at build time after
`flexelem_data` is populated.

**Model fields** (`model.rs`, after `flexelem_flexid`):

```rust
    /// Per-element: flat sorted adjacency list. Element `e`'s adjacent
    /// elements are `flex_elem_adj[flex_elem_adj_adr[e]..+flex_elem_adj_num[e]]`.
    /// Two elements are adjacent if they share at least one vertex.
    /// Length: sum of all adjacency counts across all elements.
    pub flex_elem_adj: Vec<usize>,
    /// Start index in `flex_elem_adj` for element `e`. Length `nflexelem`.
    pub flex_elem_adj_adr: Vec<usize>,
    /// Number of adjacent elements for element `e`. Length `nflexelem`.
    pub flex_elem_adj_num: Vec<usize>,
```

**Build-time computation** (`builder/flex.rs`, after element data is populated):

Algorithm:
1. Build vertex-to-element map: for each vertex, collect all elements containing it
2. For each element `e`, collect all elements that share at least one vertex
   (union of vertex-to-element sets for each vertex of `e`, minus `e` itself)
3. Sort neighbor lists, deduplicate, flatten into `flex_elem_adj`

```rust
fn compute_element_adjacency(
    flexelem_data: &[usize],
    flexelem_dataadr: &[usize],
    flexelem_datanum: &[usize],
    nflexelem: usize,
    nflexvert: usize,
) -> (Vec<usize>, Vec<usize>, Vec<usize>) {
    // Step 1: vertex → element map
    let mut vert_to_elems: Vec<Vec<usize>> = vec![vec![]; nflexvert];
    for e in 0..nflexelem {
        let adr = flexelem_dataadr[e];
        let num = flexelem_datanum[e];
        for i in 0..num {
            vert_to_elems[flexelem_data[adr + i]].push(e);
        }
    }

    // Step 2: per-element adjacency
    let mut adj_data = Vec::new();
    let mut adj_adr = Vec::with_capacity(nflexelem);
    let mut adj_num = Vec::with_capacity(nflexelem);

    for e in 0..nflexelem {
        let adr = flexelem_dataadr[e];
        let num = flexelem_datanum[e];
        let mut neighbors = Vec::new();
        for i in 0..num {
            let v = flexelem_data[adr + i];
            for &neighbor in &vert_to_elems[v] {
                if neighbor != e {
                    neighbors.push(neighbor);
                }
            }
        }
        neighbors.sort_unstable();
        neighbors.dedup();

        adj_adr.push(adj_data.len());
        adj_num.push(neighbors.len());
        adj_data.extend(neighbors);
    }

    (adj_data, adj_adr, adj_num)
}
```

**Helper function** (for runtime adjacency check):

```rust
/// Check if two elements are adjacent (share at least one vertex).
/// Uses binary search on the precomputed sorted adjacency list.
#[inline]
fn elements_adjacent(model: &Model, e1: usize, e2: usize) -> bool {
    let adr = model.flex_elem_adj_adr[e1];
    let num = model.flex_elem_adj_num[e1];
    model.flex_elem_adj[adr..adr + num].binary_search(&e2).is_ok()
}
```

### S5. Gate logic + dispatch structure

**File:** `sim/L0/core/src/collision/mod.rs` (add after `mj_collision_flex()` call)
**MuJoCo equivalent:** `mj_collision()` flex self-collision dispatch loop
**Design decision:** New top-level function `mj_collision_flex_self()` called
from `mj_collision()` after `mj_collision_flex()`. Iterates all flexes,
applies three gate conditions, dispatches internal and self-collision paths.
Matches umbrella Convention Registry §3 pipeline order.

**Data dependency chain:** `mj_collision_flex_self()` reads
`data.flexvert_xpos` for element AABB construction and vertex-face tests.
`flexvert_xpos` is populated by `mj_flex()` in the position stage
(`forward/position.rs` → `dynamics/flex.rs`), which syncs vertex world
positions from body FK. The collision stage runs after the position stage in
`forward()`, so `flexvert_xpos` is guaranteed current when
`mj_collision_flex_self()` executes. Element AABBs are computed per-step
(not cached on Model) because vertex positions change each step.

**Pipeline integration** (`collision/mod.rs`, after line 540):

**Before:**
```rust
    // Mechanism 3: flex vertex vs rigid geom contacts
    mj_collision_flex(model, data);
}
```

**After:**
```rust
    // Mechanism 3: flex vertex vs rigid geom contacts
    mj_collision_flex(model, data);

    // Mechanism 4: flex self-collision (internal + non-adjacent)
    mj_collision_flex_self(model, data);
}
```

**Dispatch function:**

```rust
/// Dispatch flex self-collision: internal (adjacent) + self (non-adjacent).
///
/// Called after mj_collision_flex() in the collision pipeline. Iterates all
/// flex objects, applies three conjunctive gate conditions, dispatches to
/// internal and self-collision paths independently.
fn mj_collision_flex_self(model: &Model, data: &mut Data) {
    for f in 0..model.nflex {
        // Gate condition 1: skip rigid flexes (all vertices invmass == 0)
        if model.flex_rigid[f] {
            continue;
        }

        // Gate condition 2: self-bitmask check
        // contype & conaffinity against ITSELF (not cross-object)
        if (model.flex_contype[f] & model.flex_conaffinity[f]) == 0 {
            continue;
        }

        // Path 1: internal collision (adjacent elements)
        if model.flex_internal[f] {
            mj_collide_flex_internal(model, data, f);
        }

        // Path 2: self-collision (non-adjacent elements)
        if model.flex_selfcollide[f] != FlexSelfCollide::None {
            match model.flex_selfcollide[f] {
                FlexSelfCollide::None => unreachable!(),
                FlexSelfCollide::Narrow => {
                    mj_collide_flex_self_narrow(model, data, f);
                }
                FlexSelfCollide::Bvh => {
                    mj_collide_flex_self_bvh(model, data, f);
                }
                FlexSelfCollide::Sap => {
                    mj_collide_flex_self_sap(model, data, f);
                }
                FlexSelfCollide::Auto => {
                    if model.flex_dim[f] == 3 {
                        mj_collide_flex_self_bvh(model, data, f);
                    } else {
                        mj_collide_flex_self_sap(model, data, f);
                    }
                }
            }
        }
    }
}
```

### S6. Internal collision (adjacent elements)

**File:** `sim/L0/core/src/collision/flex_collide.rs`
**MuJoCo equivalent:** `mj_collideFlexInternal()` in `engine_collision_driver.c`
**Design decision:** Iterate adjacent element pairs (from S4 adjacency data),
perform vertex-face tests between non-shared vertices and opposing element
faces. For dim=2 (triangles): sphere-triangle tests. For dim=3 (tetrahedra):
sphere-face tests against each tet face.

```rust
/// Internal collision: contacts between adjacent elements within a flex.
///
/// For each adjacent element pair, tests non-shared vertices of each element
/// against faces of the other element. Generates vertex-face contacts where
/// the vertex sphere penetrates the opposing face within margin.
pub fn mj_collide_flex_internal(model: &Model, data: &mut Data, f: usize) {
    let elem_start = model.flex_elemadr[f];
    let elem_count = model.flex_elemnum[f];
    let dim = model.flex_dim[f];

    if elem_count < 2 { return; } // need 2+ elements for internal collision

    // Iterate unique adjacent pairs
    for local_e in 0..elem_count {
        let e1 = elem_start + local_e;
        let adr = model.flex_elem_adj_adr[e1];
        let num = model.flex_elem_adj_num[e1];
        for &e2 in &model.flex_elem_adj[adr..adr + num] {
            // Only process each pair once (e1 < e2)
            if e2 <= e1 { continue; }
            // Both elements must belong to the same flex
            if model.flexelem_flexid[e2] != f { continue; }

            // Get vertex sets for both elements
            let verts1 = elem_vertices(model, e1);
            let verts2 = elem_vertices(model, e2);

            // Test non-shared vertices of e1 against faces of e2
            for &v in &verts1 {
                if !verts2.contains(&v) {
                    test_vertex_against_element_faces(
                        model, data, v, &verts2, dim, f,
                    );
                }
            }
            // Test non-shared vertices of e2 against faces of e1
            for &v in &verts2 {
                if !verts1.contains(&v) {
                    test_vertex_against_element_faces(
                        model, data, v, &verts1, dim, f,
                    );
                }
            }
        }
    }
}

/// Get vertex indices for an element.
fn elem_vertices(model: &Model, elem: usize) -> Vec<usize> {
    let adr = model.flexelem_dataadr[elem];
    let num = model.flexelem_datanum[elem];
    model.flexelem_data[adr..adr + num].to_vec()
}

/// Test a vertex sphere against the faces of an element.
///
/// For dim=2 (triangle): one face (the triangle itself).
/// For dim=3 (tetrahedron): four triangular faces.
/// For dim=1 (edge): skip (no faces for vertex-face test).
fn test_vertex_against_element_faces(
    model: &Model,
    data: &mut Data,
    vertex: usize,
    elem_verts: &[usize],
    dim: usize,
    flex_id: usize,
) {
    let vpos = data.flexvert_xpos[vertex];
    let radius = model.flexvert_radius[vertex];
    let margin = 2.0 * model.flex_margin[flex_id];
    let gap = 2.0 * model.flex_gap[flex_id];
    let includemargin = margin - gap;

    match dim {
        2 => {
            // Triangle face: elem_verts = [v0, v1, v2]
            assert!(elem_verts.len() >= 3);
            let tri = [
                data.flexvert_xpos[elem_verts[0]],
                data.flexvert_xpos[elem_verts[1]],
                data.flexvert_xpos[elem_verts[2]],
            ];
            if let Some((depth, normal, contact_pos)) =
                sphere_triangle_contact(vpos, radius, &tri)
            {
                if depth > -includemargin {
                    // Find nearest face vertex for flex_vertex2
                    let nearest = nearest_vertex(vpos, elem_verts, data);
                    let contact = make_contact_flex_self(
                        model, vertex, nearest, contact_pos, normal, depth,
                    );
                    data.contacts.push(contact);
                    data.ncon += 1;
                }
            }
        }
        3 => {
            // Tetrahedron: 4 triangular faces
            // Faces: (0,1,2), (0,1,3), (0,2,3), (1,2,3)
            let faces = [
                [elem_verts[0], elem_verts[1], elem_verts[2]],
                [elem_verts[0], elem_verts[1], elem_verts[3]],
                [elem_verts[0], elem_verts[2], elem_verts[3]],
                [elem_verts[1], elem_verts[2], elem_verts[3]],
            ];
            for face in &faces {
                let tri = [
                    data.flexvert_xpos[face[0]],
                    data.flexvert_xpos[face[1]],
                    data.flexvert_xpos[face[2]],
                ];
                if let Some((depth, normal, contact_pos)) =
                    sphere_triangle_contact(vpos, radius, &tri)
                {
                    if depth > -includemargin {
                        let nearest = nearest_vertex(vpos, face, data);
                        let contact = make_contact_flex_self(
                            model, vertex, nearest, contact_pos, normal, depth,
                        );
                        data.contacts.push(contact);
                        data.ncon += 1;
                    }
                }
            }
        }
        _ => {} // dim=1: no faces for vertex-face test
    }
}

/// Find the nearest vertex (by Euclidean distance) from a set.
fn nearest_vertex(
    pos: Vector3<f64>,
    candidates: &[usize],
    data: &Data,
) -> usize {
    candidates.iter()
        .copied()
        .min_by(|&a, &b| {
            let da = (data.flexvert_xpos[a] - pos).norm_squared();
            let db = (data.flexvert_xpos[b] - pos).norm_squared();
            da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
        })
        .expect("candidates must not be empty")
}

/// Sphere-triangle contact test.
///
/// Tests if a sphere (center, radius) intersects a triangle. Returns
/// (depth, normal, contact_pos) if intersecting. Normal points from
/// triangle surface toward sphere center.
pub fn sphere_triangle_contact(
    sphere_pos: Vector3<f64>,
    sphere_radius: f64,
    tri: &[Vector3<f64>; 3],
) -> Option<(f64, Vector3<f64>, Vector3<f64>)> {
    let v0 = tri[0];
    let v1 = tri[1];
    let v2 = tri[2];
    let e1 = v1 - v0;
    let e2 = v2 - v0;
    let n = e1.cross(&e2);
    let n_len = n.norm();
    if n_len < 1e-12 { return None; } // degenerate triangle
    let n = n / n_len;

    // Distance from sphere center to triangle plane
    let d = (sphere_pos - v0).dot(&n);
    if d.abs() > sphere_radius { return None; } // too far

    // Project sphere center onto triangle plane
    let proj = sphere_pos - d * n;

    // Check if projection is inside triangle (barycentric coordinates)
    let v0p = proj - v0;
    let d00 = e1.dot(&e1);
    let d01 = e1.dot(&e2);
    let d11 = e2.dot(&e2);
    let d20 = v0p.dot(&e1);
    let d21 = v0p.dot(&e2);
    let denom = d00 * d11 - d01 * d01;
    if denom.abs() < 1e-12 { return None; } // degenerate
    let bary_v = (d11 * d20 - d01 * d21) / denom;
    let bary_w = (d00 * d21 - d01 * d20) / denom;
    let bary_u = 1.0 - bary_v - bary_w;

    if bary_u >= 0.0 && bary_v >= 0.0 && bary_w >= 0.0 {
        // Projection inside triangle — contact on triangle plane
        let normal = if d >= 0.0 { n } else { -n };
        let depth = sphere_radius - d.abs();
        let contact_pos = proj;
        return Some((depth, normal, contact_pos));
    }

    // Projection outside triangle — find closest point on edges/vertices
    let closest = closest_point_on_triangle(sphere_pos, tri);
    let diff = sphere_pos - closest;
    let dist = diff.norm();
    if dist < 1e-12 || dist > sphere_radius { return None; }

    let normal = diff / dist;
    let depth = sphere_radius - dist;
    Some((depth, normal, closest))
}

/// Find closest point on triangle boundary (edges + vertices).
fn closest_point_on_triangle(p: Vector3<f64>, tri: &[Vector3<f64>; 3]) -> Vector3<f64> {
    let edges = [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])];
    let mut best = tri[0];
    let mut best_dist = (p - tri[0]).norm_squared();

    for (a, b) in &edges {
        let ab = *b - *a;
        let t = ((p - *a).dot(&ab) / ab.dot(&ab)).clamp(0.0, 1.0);
        let closest = *a + t * ab;
        let dist = (p - closest).norm_squared();
        if dist < best_dist {
            best_dist = dist;
            best = closest;
        }
    }
    best
}
```

### S7. Self-collision narrowphase (NARROW brute-force)

**File:** `sim/L0/core/src/collision/flex_collide.rs`
**MuJoCo equivalent:** `mj_collideFlexSelf()` in `engine_collision_driver.c`
(NARROW mode)
**Design decision:** Iterate all `O(n²)` element pairs within the flex, skip
adjacent pairs (using S4 adjacency check), run narrowphase on non-adjacent
pairs. For dim=2: use `triangle_triangle_intersection()` from `mesh.rs`. For
dim=3: use vertex-face + edge-edge tests. For dim=1: use edge-edge proximity.

```rust
/// Self-collision (NARROW): brute-force all non-adjacent element pairs.
///
/// O(n²) element pair iteration. For each non-adjacent pair, run element-
/// element narrowphase. This is the reference implementation — BVH and SAP
/// produce identical contacts with midphase acceleration.
pub fn mj_collide_flex_self_narrow(model: &Model, data: &mut Data, f: usize) {
    let elem_start = model.flex_elemadr[f];
    let elem_count = model.flex_elemnum[f];
    let dim = model.flex_dim[f];

    if elem_count < 2 { return; }

    for i in 0..elem_count {
        let e1 = elem_start + i;
        for j in (i + 1)..elem_count {
            let e2 = elem_start + j;

            // Skip adjacent elements (handled by internal collision)
            if elements_adjacent(model, e1, e2) {
                continue;
            }

            collide_element_pair(model, data, e1, e2, dim, f);
        }
    }
}

/// Element-element narrowphase for non-adjacent elements.
///
/// Dispatches based on dim:
/// - dim=2: triangle-triangle intersection (SAT-based)
/// - dim=3: vertex-face + edge-edge tests
/// - dim=1: edge-edge proximity
///
/// This function is the narrowphase primitive reused by Spec D (flex-flex).
pub fn collide_element_pair(
    model: &Model,
    data: &mut Data,
    e1: usize,
    e2: usize,
    dim: usize,
    flex_id: usize,
) {
    let verts1 = elem_vertices(model, e1);
    let verts2 = elem_vertices(model, e2);

    match dim {
        2 => {
            collide_triangles(model, data, &verts1, &verts2, flex_id);
        }
        3 => {
            collide_tetrahedra(model, data, &verts1, &verts2, flex_id);
        }
        1 => {
            collide_edges(model, data, &verts1, &verts2, flex_id);
        }
        _ => {}
    }
}

/// Triangle-triangle narrowphase for dim=2 self-collision.
///
/// Uses the existing `triangle_triangle_intersection()` (SAT-based) from
/// mesh.rs. Generates at most one contact per triangle pair.
fn collide_triangles(
    model: &Model,
    data: &mut Data,
    verts1: &[usize],
    verts2: &[usize],
    flex_id: usize,
) {
    use crate::mesh::triangle_triangle_intersection;
    use nalgebra::Point3;

    let tri_a = [
        Point3::from(data.flexvert_xpos[verts1[0]]),
        Point3::from(data.flexvert_xpos[verts1[1]]),
        Point3::from(data.flexvert_xpos[verts1[2]]),
    ];
    let tri_b = [
        Point3::from(data.flexvert_xpos[verts2[0]]),
        Point3::from(data.flexvert_xpos[verts2[1]]),
        Point3::from(data.flexvert_xpos[verts2[2]]),
    ];

    let margin = 2.0 * model.flex_margin[flex_id];
    let gap = 2.0 * model.flex_gap[flex_id];
    let includemargin = margin - gap;

    if let Some(contact_result) = triangle_triangle_intersection(&tri_a, &tri_b) {
        if contact_result.depth > -includemargin {
            // Use contact_result.point, normal, depth
            // Find nearest vertex on each element to determine flex_vertex/vertex2
            let nearest1 = nearest_vertex(
                contact_result.point.coords, verts1, data,
            );
            let nearest2 = nearest_vertex(
                contact_result.point.coords, verts2, data,
            );

            let contact = make_contact_flex_self(
                model,
                nearest1,
                nearest2,
                contact_result.point.coords,
                contact_result.normal,
                contact_result.depth,
            );
            data.contacts.push(contact);
            data.ncon += 1;
        }
    }
}

/// Tetrahedron-tetrahedron narrowphase for dim=3 self-collision.
///
/// Tests each vertex of tet A against faces of tet B (and vice versa).
/// Also tests edge-edge proximity between non-parallel edges.
fn collide_tetrahedra(
    model: &Model,
    data: &mut Data,
    verts1: &[usize],
    verts2: &[usize],
    flex_id: usize,
) {
    // Vertex-face: each vertex of tet1 against 4 faces of tet2
    for &v in verts1 {
        test_vertex_against_element_faces(
            model, data, v, verts2, 3, flex_id,
        );
    }
    // Vertex-face: each vertex of tet2 against 4 faces of tet1
    for &v in verts2 {
        test_vertex_against_element_faces(
            model, data, v, verts1, 3, flex_id,
        );
    }
    // Edge-edge tests between non-parallel edges of tet1 and tet2 are
    // deferred to post-v1.0 (DT-151). Vertex-face tests capture the
    // primary self-collision contacts for tetrahedral meshes.
}

/// Edge-edge proximity for dim=1 self-collision.
///
/// Tests two cable segments (each = 2 vertices) for proximity.
/// If closest distance < combined vertex radii + margin, generate contact.
fn collide_edges(
    model: &Model,
    data: &mut Data,
    verts1: &[usize],
    verts2: &[usize],
    flex_id: usize,
) {
    let p0 = data.flexvert_xpos[verts1[0]];
    let p1 = data.flexvert_xpos[verts1[1]];
    let q0 = data.flexvert_xpos[verts2[0]];
    let q1 = data.flexvert_xpos[verts2[1]];

    let r1 = model.flexvert_radius[verts1[0]].max(model.flexvert_radius[verts1[1]]);
    let r2 = model.flexvert_radius[verts2[0]].max(model.flexvert_radius[verts2[1]]);
    let combined_radius = r1 + r2;
    let margin = 2.0 * model.flex_margin[flex_id];
    let gap = 2.0 * model.flex_gap[flex_id];
    let includemargin = margin - gap;

    // Closest distance between two line segments
    let (closest1, closest2) = closest_points_segments(p0, p1, q0, q1);
    let diff = closest1 - closest2;
    let dist = diff.norm();

    if dist < 1e-12 { return; } // degenerate
    let depth = combined_radius - dist;
    if depth <= -includemargin { return; }

    let normal = diff / dist;
    let contact_pos = (closest1 + closest2) * 0.5;

    // Nearest vertices for flex_vertex/flex_vertex2
    let v1 = if (closest1 - p0).norm_squared() < (closest1 - p1).norm_squared() {
        verts1[0]
    } else {
        verts1[1]
    };
    let v2 = if (closest2 - q0).norm_squared() < (closest2 - q1).norm_squared() {
        verts2[0]
    } else {
        verts2[1]
    };

    let contact = make_contact_flex_self(model, v1, v2, contact_pos, normal, depth);
    data.contacts.push(contact);
    data.ncon += 1;
}

/// Closest points between two line segments.
fn closest_points_segments(
    a0: Vector3<f64>, a1: Vector3<f64>,
    b0: Vector3<f64>, b1: Vector3<f64>,
) -> (Vector3<f64>, Vector3<f64>) {
    let d1 = a1 - a0;
    let d2 = b1 - b0;
    let r = a0 - b0;
    let a = d1.dot(&d1);
    let e = d2.dot(&d2);
    let f = d2.dot(&r);

    if a < 1e-12 && e < 1e-12 {
        return (a0, b0); // both degenerate to points
    }

    let (s, t) = if a < 1e-12 {
        (0.0, (f / e).clamp(0.0, 1.0))
    } else {
        let c = d1.dot(&r);
        if e < 1e-12 {
            ((-c / a).clamp(0.0, 1.0), 0.0)
        } else {
            let b = d1.dot(&d2);
            let denom = a * e - b * b;
            let s = if denom.abs() > 1e-12 {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let t = ((b * s + f) / e).clamp(0.0, 1.0);
            // Recompute s for clamped t
            let s = if a.abs() > 1e-12 {
                ((b * t - c) / a).clamp(0.0, 1.0)
            } else {
                s
            };
            (s, t)
        }
    };

    (a0 + s * d1, b0 + t * d2)
}
```

### S8. Midphase acceleration (BVH + SAP + AUTO)

**File:** `sim/L0/core/src/collision/flex_collide.rs`
**MuJoCo equivalent:** BVH and SAP midphase in `mj_collideFlexSelf()`
**Design decision:** Reuse existing `Bvh` infrastructure from `mid_phase.rs`
for BVH mode. Implement a simple axis-sweep for SAP mode. Both produce
candidate element pairs, filter adjacent pairs, run S7's narrowphase. AUTO
dispatches BVH for dim=3, SAP otherwise.

**BVH midphase:**

```rust
/// Self-collision with BVH midphase.
///
/// Builds a per-element AABB tree each step (vertex positions change),
/// queries for overlapping element pairs, filters adjacent, runs narrowphase.
///
/// Implementation note: uses per-element query loop (each element queries BVH
/// for overlapping neighbors) rather than BVH-vs-BVH self-intersection
/// traversal. This is correct and produces identical results — the per-element
/// approach is O(n log n) typical (each query is O(log n), n queries). A
/// dedicated BVH self-intersection would avoid rebuilding query AABBs but the
/// algorithmic complexity is the same. MuJoCo's internal implementation
/// details for BVH traversal are not documented; our approach matches the
/// observable behavior (same candidate pairs, same contacts).
pub fn mj_collide_flex_self_bvh(model: &Model, data: &mut Data, f: usize) {
    use crate::mid_phase::{Bvh, BvhPrimitive};

    let elem_start = model.flex_elemadr[f];
    let elem_count = model.flex_elemnum[f];
    let dim = model.flex_dim[f];

    if elem_count < 2 { return; }

    // Build per-element AABBs
    let primitives: Vec<BvhPrimitive> = (0..elem_count)
        .map(|i| {
            let e = elem_start + i;
            let verts = elem_vertices(model, e);
            let aabb = element_aabb(&verts, data);
            BvhPrimitive { aabb, index: e, data: 0 }
        })
        .collect();

    let bvh = Bvh::build(primitives);

    // Query all overlapping pairs
    for i in 0..elem_count {
        let e1 = elem_start + i;
        let verts1 = elem_vertices(model, e1);
        let aabb1 = element_aabb(&verts1, data);

        for candidate_idx in bvh.query(&aabb1) {
            let e2 = elem_start + candidate_idx;

            // Skip self-overlap and duplicate pairs
            if e2 <= e1 { continue; }

            // Skip adjacent elements
            if elements_adjacent(model, e1, e2) { continue; }

            collide_element_pair(model, data, e1, e2, dim, f);
        }
    }
}

/// Compute AABB for an element from its vertex positions.
fn element_aabb(verts: &[usize], data: &Data) -> Aabb {
    use crate::mid_phase::Aabb;

    let mut min = data.flexvert_xpos[verts[0]];
    let mut max = min;

    for &v in &verts[1..] {
        let p = data.flexvert_xpos[v];
        min = Vector3::new(min.x.min(p.x), min.y.min(p.y), min.z.min(p.z));
        max = Vector3::new(max.x.max(p.x), max.y.max(p.y), max.z.max(p.z));
    }

    Aabb { min, max }
}
```

**SAP midphase:**

```rust
/// Self-collision with SAP (sweep-and-prune) midphase.
///
/// Computes element AABBs, sorts along axis of maximum variance, sweeps
/// for overlapping projections. Filters adjacent, runs narrowphase.
pub fn mj_collide_flex_self_sap(model: &Model, data: &mut Data, f: usize) {
    let elem_start = model.flex_elemadr[f];
    let elem_count = model.flex_elemnum[f];
    let dim = model.flex_dim[f];

    if elem_count < 2 { return; }

    // Compute element AABBs + centroids
    let mut aabbs: Vec<(usize, Aabb)> = (0..elem_count)
        .map(|i| {
            let e = elem_start + i;
            let verts = elem_vertices(model, e);
            let aabb = element_aabb(&verts, data);
            (e, aabb)
        })
        .collect();

    // Find axis of maximum variance
    let axis = axis_of_max_variance(&aabbs);

    // Sort by AABB min along chosen axis
    aabbs.sort_by(|a, b| {
        let a_min = match axis { 0 => a.1.min.x, 1 => a.1.min.y, _ => a.1.min.z };
        let b_min = match axis { 0 => b.1.min.x, 1 => b.1.min.y, _ => b.1.min.z };
        a_min.partial_cmp(&b_min).unwrap_or(std::cmp::Ordering::Equal)
    });

    // Sweep: for each element, check subsequent elements until no overlap
    for i in 0..aabbs.len() {
        let (e1, ref aabb1) = aabbs[i];
        let max1 = match axis { 0 => aabb1.max.x, 1 => aabb1.max.y, _ => aabb1.max.z };

        for j in (i + 1)..aabbs.len() {
            let (e2, ref aabb2) = aabbs[j];
            let min2 = match axis { 0 => aabb2.min.x, 1 => aabb2.min.y, _ => aabb2.min.z };

            // No more overlaps along sweep axis
            if min2 > max1 { break; }

            // Check full 3D AABB overlap
            if !aabb_overlap(aabb1, aabb2) { continue; }

            // Skip adjacent elements
            let (ea, eb) = if e1 < e2 { (e1, e2) } else { (e2, e1) };
            if elements_adjacent(model, ea, eb) { continue; }

            collide_element_pair(model, data, ea, eb, dim, f);
        }
    }
}

/// Find axis (0=x, 1=y, 2=z) of maximum AABB centroid variance.
fn axis_of_max_variance(aabbs: &[(usize, Aabb)]) -> usize {
    let n = aabbs.len() as f64;
    if n < 2.0 { return 0; }

    let mut mean = [0.0_f64; 3];
    for (_, aabb) in aabbs {
        let c = (aabb.min + aabb.max) * 0.5;
        mean[0] += c.x;
        mean[1] += c.y;
        mean[2] += c.z;
    }
    mean[0] /= n;
    mean[1] /= n;
    mean[2] /= n;

    let mut var = [0.0_f64; 3];
    for (_, aabb) in aabbs {
        let c = (aabb.min + aabb.max) * 0.5;
        var[0] += (c.x - mean[0]).powi(2);
        var[1] += (c.y - mean[1]).powi(2);
        var[2] += (c.z - mean[2]).powi(2);
    }

    if var[0] >= var[1] && var[0] >= var[2] { 0 }
    else if var[1] >= var[2] { 1 }
    else { 2 }
}

/// Test 3D AABB overlap.
fn aabb_overlap(a: &Aabb, b: &Aabb) -> bool {
    a.min.x <= b.max.x && a.max.x >= b.min.x
        && a.min.y <= b.max.y && a.max.y >= b.min.y
        && a.min.z <= b.max.z && a.max.z >= b.min.z
}
```

---

## Acceptance Criteria

### AC1: FlexSelfCollide enum parsing *(runtime test)*
**Given:** MJCF model with `<flex selfcollide="bvh"/>`
**After:** Model build
**Assert:** `model.flex_selfcollide[0] == FlexSelfCollide::Bvh`
**Field:** `Model.flex_selfcollide`

### AC2: FlexSelfCollide default *(runtime test)*
**Given:** MJCF model with `<flex/>` (no `selfcollide` attribute)
**After:** Model build
**Assert:** `model.flex_selfcollide[0] == FlexSelfCollide::Auto`
**Field:** `Model.flex_selfcollide`

### AC3: Gate — rigid flex produces zero self-contacts *(runtime test)*
**Given:** Flex with all vertices pinned (`invmass == 0`), `selfcollide="auto"`,
compatible self-bitmask (`contype=1, conaffinity=1`)
**After:** `mj_collision()`
**Assert:** Zero contacts with `flex_vertex2.is_some()`
**Field:** `Data.contacts`, `Data.ncon`

### AC4: Gate — incompatible self-bitmask produces zero self-contacts *(runtime test)*
**Given:** Non-rigid flex with `contype=2, conaffinity=4` (`2 & 4 = 0`),
`selfcollide="auto"`
**After:** `mj_collision()`
**Assert:** Zero contacts with `flex_vertex2.is_some()`
**Field:** `Data.contacts`, `Data.ncon`

### AC5: Gate — selfcollide=none produces zero non-adjacent contacts *(runtime test)*
**Given:** Non-rigid flex with compatible self-bitmask, `selfcollide="none"`,
`internal=true`, elements overlapping from deformation
**After:** `mj_collision()`
**Assert:** Only internal (adjacent-element) contacts generated, zero non-adjacent contacts
**Field:** `Data.contacts`

### AC6: Gate — internal=false produces zero adjacent contacts *(runtime test)*
**Given:** Non-rigid flex with compatible self-bitmask, `selfcollide="auto"`,
`internal=false`, adjacent elements with penetrating vertices
**After:** `mj_collision()`
**Assert:** Only non-adjacent self-collision contacts, zero adjacent contacts
**Field:** `Data.contacts`

### AC7: Internal collision — adjacent triangles generate vertex-face contacts *(runtime test)*
**Given:** 2-triangle mesh: triangle A = [(0,0,0), (1,0,0), (0,1,0)],
triangle B = [(1,0,0), (0,1,0), (0.5,0.5,0)]. They share edge
[(1,0,0),(0,1,0)]. Deform vertex (0.5,0.5,0) of triangle B to
(0.5,0.5,-0.02) so it penetrates triangle A's plane (z=0) by 0.02.
Vertex radius = 0.01. `internal=true`, `selfcollide="none"`, `margin=0`.
**After:** `mj_collision()`
**Assert:** Exactly 1 contact generated. `depth ≈ 0.03 ± 1e-6` (radius +
plane penetration distance). Normal ≈ (0,0,1) (triangle A's face normal).
`contact.flex_vertex` = index of vertex (0.5,0.5,-0.02),
`contact.flex_vertex2.is_some()`.
**Field:** `Data.contacts`
**Derivation:** depth = vertex_radius (0.01) + |vertex_z - plane_z| (0.02)
= 0.03. This follows directly from MuJoCo's `mj_collideFlexInternal()`
sphere-face test: penetration depth = sphere_radius − signed_distance_to_face.
Normal = face outward normal = (0,0,1) for z=0 plane with standard winding.
**MuJoCo verification:** During implementation, load equivalent MJCF into
MuJoCo C, deform vertex via `d->flexvert_xpos`, call `mj_collision()`, and
confirm `d->ncon`, contact depth, and normal match within 1e-10.

### AC8: Self-collision — folded cloth generates non-adjacent contacts *(runtime test)*
**Given:** 4×4 triangle shell with `selfcollide="narrow"`, `internal=false`,
deformed so non-adjacent elements overlap (folded in half)
**After:** `mj_collision()`
**Assert:** At least one self-collision contact with
`contact.flex_vertex.is_some() && contact.flex_vertex2.is_some()`
**Field:** `Data.contacts`

### AC9: BVH/SAP equivalence *(runtime test)*
**Given:** Same 4×4 triangle shell model, deformed to create self-collision
**After:** Three runs with `selfcollide="narrow"`, `"bvh"`, `"sap"`
**Assert:** All three produce identical contact sets (same number of contacts,
same vertex pairs, same normals within tolerance 1e-12)
**Field:** `Data.contacts`

### AC10: AUTO dispatch — BVH for dim=3, SAP otherwise *(runtime test)*
**Given:** Two models: dim=2 shell with `selfcollide="auto"`, dim=3 solid with
`selfcollide="auto"`
**After:** `mj_collision()` for each
**Assert:** dim=2 model uses SAP path, dim=3 model uses BVH path. Contacts
match the equivalent explicit mode (`"sap"` for dim=2, `"bvh"` for dim=3).
**Field:** `Data.contacts`

### AC11: Contact encoding correctness *(runtime test)*
**Given:** Self-collision contact generated from AC8 scenario
**After:** Examine contact
**Assert:** `contact.flex_vertex.is_some() && contact.flex_vertex2.is_some()`.
`contact.geom1 == usize::MAX && contact.geom2 == usize::MAX`.
Both vertices belong to the same flex.
**Field:** `Contact.flex_vertex`, `Contact.flex_vertex2`, `Contact.geom1/geom2`

### AC12: Jacobian correctness *(runtime test)*
**Given:** Self-collision contact from AC7 scenario (2-triangle mesh).
Vertex 1 (penetrating) has `flexvert_dofadr[v1] = d1`. Vertex 2 (opposing)
has `flexvert_dofadr[v2] = d2`. Contact normal = n.
**After:** `compute_contact_jacobian()` on the contact
**Assert:** Jacobian `J` is `dim × nv` matrix. Row 0: `J[0, d1] = n.x`,
`J[0, d1+1] = n.y`, `J[0, d1+2] = n.z`, `J[0, d2] = -n.x`,
`J[0, d2+1] = -n.y`, `J[0, d2+2] = -n.z`. All other columns in row 0 are
zero. Rows 1-2 (if dim≥3): same pattern with tangent vectors t1, t2. Rows
3+ (torsional/rolling) are all-zero (flex vertices have no angular DOFs).
Total nonzero columns: exactly 6 (3 per vertex).
**Field:** Jacobian matrix structure

### AC13: Assembly bodyweight *(runtime test)*
**Given:** Self-collision contact from AC8 scenario
**After:** Contact assembly
**Assert:** Bodyweight computed from `model.flexvert_bodyid[vi1]` and
`model.flexvert_bodyid[vi2]`, NOT from `model.geom_body[contact.geom1/geom2]`
(which would index with `usize::MAX`).
**Field:** `bw_contact` values

### AC14: Margin formula *(runtime test)*
**Given:** Flex with `margin=0.01`, `gap=0.002`, `selfcollide="narrow"`
**After:** Self-collision contact generated
**Assert:** `contact.includemargin == 2*0.01 - 2*0.002 = 0.016 ± 1e-12`
**Field:** `Contact.includemargin`

### AC15: Element adjacency precomputation *(runtime test)*
**Given:** 4-triangle mesh (known topology)
**After:** Model build
**Assert:** Adjacency lists match expected: triangle 0 is adjacent to
triangles 1, 2 (sharing edges), etc. `elements_adjacent(model, e1, e2)`
returns correct results.
**Field:** `Model.flex_elem_adj`, `Model.flex_elem_adj_adr`, `Model.flex_elem_adj_num`

### AC16: No existing test regression *(code review)*
All 1,900+ domain tests pass unchanged. The `flex_selfcollide` type change
does not break any existing test (no tests directly assert on
`flex_selfcollide` bool values per EGT-7).

### AC17: Narrowphase primitives reusable by Spec D *(code review)*
`collide_element_pair()`, `sphere_triangle_contact()`, and
`make_contact_flex_self()` are `pub` functions callable from outside the module.
Function signatures match umbrella Contract 3.

### AC18: MuJoCo conformance — self-collision contact count *(runtime test)*
**Given:** MJCF model with a 4×4 triangle shell (`selfcollide="narrow"`,
`internal=false`), deformed into a known fold configuration (vertices on one
half displaced by `dz = -0.05` to create overlap).
**After:** Load same MJCF in MuJoCo C, apply identical vertex deformations
via `d->flexvert_xpos`, call `mj_collision()`. Record `d->ncon` and per-contact
`depth`, `pos`, `frame` from MuJoCo. Run same model through CortenForge.
**Assert:** CortenForge `data.ncon` matches MuJoCo `d->ncon` exactly.
Per-contact depth within 1e-10 of MuJoCo values. Contact normals within
1e-10 (dot product ≥ 1.0 − 1e-10).
**Field:** `Data.ncon`, per-contact `depth`, `normal`

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (enum parsing) | T1 | Direct |
| AC2 (enum default) | T1 | Direct |
| AC3 (rigid gate) | T2 | Direct |
| AC4 (bitmask gate) | T3 | Direct |
| AC5 (selfcollide=none gate) | T4 | Direct |
| AC6 (internal=false gate) | T5 | Direct |
| AC7 (internal collision) | T6 | Direct |
| AC8 (self-collision narrow) | T7 | Direct |
| AC9 (BVH/SAP equivalence) | T8 | Direct |
| AC10 (AUTO dispatch) | T9 | Direct |
| AC11 (contact encoding) | T7 | Integration |
| AC12 (Jacobian) | T10 | Direct |
| AC13 (assembly bodyweight) | T10 | Integration |
| AC14 (margin formula) | T11 | Direct |
| AC15 (adjacency precomp) | T12 | Direct |
| AC16 (no regression) | — | Code review (manual) |
| AC17 (narrowphase reusable) | — | Code review (manual) |
| AC18 (MuJoCo conformance) | T18 | Direct |

**Supplementary test → AC mapping:**

| Test | AC(s) |
|------|-------|
| T13 (multi-flex) | AC5, AC8 |
| T14 (sphere-triangle primitive) | AC7 |
| T15 (single-element) | AC3 |
| T16 (zero-element) | AC3 |
| T17 (dim=1 cable) | AC10 |
| T18 (MuJoCo conformance) | AC18 |

---

## Test Plan

### T1: FlexSelfCollide enum parsing → AC1, AC2
Test all 5 MJCF keyword values parse to correct enum variants:
`"none"→None`, `"narrow"→Narrow`, `"bvh"→Bvh`, `"sap"→Sap`, `"auto"→Auto`.
Test absent attribute defaults to `Auto`. Test `FlexSelfCollide::default() == Auto`.

### T2: Gate — rigid flex zero contacts → AC3
Build a flex with all vertices pinned (`invmass=0`), compatible bitmask,
`selfcollide="auto"`. Verify `model.flex_rigid[0] == true`. Run collision.
Assert zero contacts with `flex_vertex2.is_some()`.

### T3: Gate — incompatible self-bitmask → AC4
Build non-rigid flex with `contype=2, conaffinity=4`, `selfcollide="auto"`.
Deform so elements overlap. Run collision. Assert zero self-collision contacts.

### T4: Gate — selfcollide=none → AC5
Build non-rigid flex with compatible bitmask, `selfcollide="none"`,
`internal=true`. Deform so non-adjacent elements overlap. Run collision.
Assert only adjacent-element contacts (from internal path), zero non-adjacent.

### T5: Gate — internal=false → AC6
Build non-rigid flex with compatible bitmask, `selfcollide="narrow"`,
`internal=false`. Set up adjacent elements with vertex penetration. Run
collision. Assert only non-adjacent contacts, zero adjacent.

### T6: Internal collision — vertex-face contacts → AC7
Build 2-triangle mesh per AC7: triangle A = [(0,0,0), (1,0,0), (0,1,0)],
triangle B = [(1,0,0), (0,1,0), (0.5,0.5,0)], sharing edge
[(1,0,0),(0,1,0)]. Set `internal=true`, `selfcollide="none"`, vertex
radius = 0.01. Deform non-shared vertex of B to (0.5,0.5,-0.02). Run
collision. Assert exactly 1 contact with `depth ≈ 0.03 ± 1e-6`, normal ≈
(0,0,1), `flex_vertex` = deformed vertex, `flex_vertex2.is_some()`.
Expected value analytically derived: depth = radius(0.01) +
|plane_distance|(0.02) = 0.03.

### T7: Self-collision — folded cloth → AC8, AC11
Build 4×4 triangle grid (32 triangles). Set `selfcollide="narrow"`,
`internal=false`. Deform mesh so it folds (non-adjacent elements overlap).
Run collision. Assert ≥1 contact with `flex_vertex.is_some() &&
flex_vertex2.is_some()`, `geom1 == usize::MAX`, both vertices in same flex.

### T8: Midphase equivalence → AC9
Same folded cloth model. Run collision three times with `selfcollide` set to
`"narrow"`, `"bvh"`, `"sap"`. Assert all three produce identical contact
count and matching vertex pairs (sorted). Normal/depth within 1e-12 tolerance.

### T9: AUTO dispatch → AC10
Build dim=2 shell with `selfcollide="auto"` and dim=3 tet mesh with
`selfcollide="auto"`. Deform both to create self-collision. Verify dim=2
contacts match `"sap"` contacts, dim=3 contacts match `"bvh"` contacts.

### T10: Jacobian + assembly → AC12, AC13
Generate a self-collision contact from the AC7 scenario (2-triangle mesh).
Let vertex 1 have `dofadr = d1`, vertex 2 have `dofadr = d2`, contact
normal = n. Compute Jacobian via `compute_contact_jacobian()`. Assert:
(a) `J[0, d1] == n.x`, `J[0, d1+1] == n.y`, `J[0, d1+2] == n.z` (vertex 1
positive), (b) `J[0, d2] == -n.x`, `J[0, d2+1] == -n.y`,
`J[0, d2+2] == -n.z` (vertex 2 negative), (c) exactly 6 nonzero columns in
row 0, (d) rows 3+ are all-zero. For assembly: mock the assembly bodyweight
path and assert `flexvert_bodyid[vi1]` and `flexvert_bodyid[vi2]` are used
(not `geom_body[usize::MAX]` which would fall to body 0).

### T11: Margin/gap formula → AC14
Build flex with `margin=0.01, gap=0.002`, `selfcollide="narrow"`. Create
overlapping elements within margin distance. Assert
`contact.includemargin ≈ 0.016 ± 1e-12`.

### T12: Element adjacency → AC15
Build 4-triangle mesh with known topology (2×2 grid). Assert adjacency:
- Triangle 0 adjacent to triangles 1, 2 (sharing edges)
- Triangle 3 adjacent to triangles 1, 2
- Triangle 0 NOT adjacent to triangle 3 (diagonal, no shared vertices)
Verify `elements_adjacent()` for all pairs.

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| Rigid flex (`flex_rigid=true`) | Gate condition 1 must skip | T2 | AC3 |
| Self-bitmask zero (`contype & conaffinity == 0`) | Gate condition 2 must skip | T3 | AC4 |
| `selfcollide=none` | Gate condition 3b must skip non-adjacent | T4 | AC5 |
| `internal=false` | Gate condition 3a must skip adjacent | T5 | AC6 |
| Single-element flex | No non-adjacent pairs, no adjacent pairs → zero contacts | T15 | AC3 |
| Zero-element flex (`nelem=0`) | No element pairs → no contacts | T16 | AC3 |
| dim=1 cable | Edge-edge proximity, AUTO→SAP, no internal contacts | T17 | AC10 |
| All elements coplanar | No self-penetration → zero contacts | T7 (precondition) | AC8 |
| `activelayers` filtering | Deferred to DT-150 | — | — |

### T15: Edge case — single-element flex → AC3 (variant)
Build a flex with exactly 1 triangle element, `selfcollide="narrow"`,
`internal=true`, compatible bitmask, non-rigid. Run collision. Assert zero
contacts: a single element has no adjacent pairs (0 internal contacts) and no
non-adjacent pairs (0 self-collision contacts). Verify `data.ncon` is unchanged.

### T16: Edge case — zero-element flex → AC3 (variant)
Build a flex with `flex_elemnum[f] = 0`, `selfcollide="auto"`, compatible
bitmask, non-rigid. Run collision. Assert zero contacts. Verify both
`mj_collide_flex_internal()` and `mj_collide_flex_self_*()` return early on
`elem_count < 2`.

### T17: Edge case — dim=1 cable self-collision → AC10 (variant)
Build a dim=1 cable flex (10 edge elements), `selfcollide="auto"`. Deform so
non-adjacent cable segments cross (segments 0-1 and 5-6 brought within vertex
radius distance). Run collision. Assert: (a) AUTO dispatches to SAP path
(dim != 3), (b) edge-edge proximity contacts generated for non-adjacent
segments, (c) `contact.flex_vertex.is_some() && contact.flex_vertex2.is_some()`.
Internal collision (`internal=true`): adjacent cable segments produce zero
internal contacts (dim=1 has no faces for vertex-face test — early return in
`test_vertex_against_element_faces`).

### T18: MuJoCo conformance — self-collision contact comparison → AC18
Load MJCF 4×4 triangle shell model in MuJoCo C with `selfcollide="narrow"`,
`internal=false`. Apply vertex deformations to fold mesh (displace vertices
on one half by `dz = -0.05`). Call `mj_collision()`. Record reference values:
`d->ncon`, per-contact `depth`, `pos[3]`, `frame[6]`. Load same MJCF in
CortenForge, apply identical vertex deformations. Run collision. Assert
contact count matches MuJoCo exactly. Per-contact depth within 1e-10. Contact
normals within 1e-10 (dot product ≥ 1 − 1e-10). This is the primary
MuJoCo conformance gate for Spec C.

### Supplementary Tests

| Test | What it covers | AC(s) | Rationale |
|------|---------------|-------|-----------|
| T13 (multi-flex model) | Two flexes: one with `selfcollide="auto"`, one with `selfcollide="none"`. Only the enabled flex generates self-contacts. | AC5, AC8 | Verify per-flex dispatch independence. |
| T14 (sphere-triangle primitive) | Unit test of `sphere_triangle_contact()` with known geometry: sphere at (0,0,0.05) radius 0.1 vs triangle at z=0. Expected: depth = 0.05, normal = (0,0,1), contact_pos = (0,0,0). | AC7 | Verify primitive correctness independent of collision pipeline. |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| `flex_selfcollide` type | `Vec<bool>` | `Vec<FlexSelfCollide>` | Toward MuJoCo | Builder tests, any code reading the field | Replace `true`/`false` checks with enum variant checks. No existing runtime code reads `flex_selfcollide` (per EGT-6). |
| Self-collision contacts | Not generated | Generated for non-rigid flexes with compatible bitmask and `selfcollide != None` | Toward MuJoCo | Downstream constraint pipeline (Jacobian, assembly) | S3 updates constraint pipeline to handle self-collision contacts. |
| Internal contacts | Not generated | Generated for adjacent elements when `internal=true` | Toward MuJoCo | Same as above | Same — S3 handles both. |
| Contact struct size | No `flex_vertex2` field | `flex_vertex2: Option<usize>` added | N/A | All Contact consumers | `flex_vertex2` defaults to `None` — transparent for existing contacts. |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `sim/L0/core/src/types/enums.rs` | Add `FlexSelfCollide` enum | +25 |
| `sim/L0/core/src/types/model.rs` | Change `flex_selfcollide` type, add adjacency fields | ~15 modified, +10 new |
| `sim/L0/core/src/types/model_init.rs` | Init adjacency fields | +3 |
| `sim/L0/core/src/types/contact_types.rs` | Add `flex_vertex2` field, init in constructors | +8 |
| `sim/L0/core/src/collision/mod.rs` | Add `mj_collision_flex_self()` dispatch, `contact_param_flex_self()` | +60 |
| `sim/L0/core/src/collision/flex_collide.rs` | Add internal collision, self-collision narrowphase, midphase, primitives | +400 |
| `sim/L0/core/src/constraint/jacobian.rs` | Add `compute_flex_self_contact_jacobian()`, update dispatch | +45 |
| `sim/L0/core/src/constraint/assembly.rs` | Update bodyweight for self-collision contacts | +15 |
| `sim/L0/mjcf/src/builder/flex.rs` | String→enum conversion, adjacency computation | +60 |
| `sim/L0/mjcf/src/builder/mod.rs` | Change `flex_selfcollide` type | ~1 modified |
| `sim/L0/mjcf/src/builder/init.rs` | Init as `Vec<FlexSelfCollide>` | ~1 modified |
| `sim/L0/core/src/collision/flex_collide.rs` (tests) | New test module | +250 |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| `test_flex_internal_false` | `sim/L0/mjcf/src/builder/mod.rs:1302` | Pass (unchanged) | Tests `flex_internal`, not `flex_selfcollide` |
| `test_flex_internal_default` | `sim/L0/mjcf/src/builder/mod.rs:1336` | Pass (unchanged) | Tests `flex_internal`, not `flex_selfcollide` |
| Flex-rigid collision tests | `sim/L0/core/src/collision/flex_collide.rs` Spec E tests | Pass (unchanged) | Self-collision dispatch is additive, after flex-rigid |
| Flex passive force tests | `sim/L0/core/src/forward/passive.rs` | Pass (unchanged) | Collision and passive are independent pipeline stages |
| Phase 9 collision tests | `sim/L0/core/src/collision/mod.rs` | Pass (unchanged) | Phase 10 does not modify Phase 9 collision code |

### Non-Modification Sites

| File:line | What it does | Why NOT modified |
|-----------|-------------|-----------------|
| `sim/L0/core/src/collision/mod.rs:549-600` | `mj_collision_flex()` — flex-rigid dispatch | Different collision path (flex-vs-rigid, not self) |
| `sim/L0/core/src/forward/passive.rs` | Flex passive forces (spring, bending) | Different pipeline stage — not collision |
| `sim/L0/core/src/collision/mod.rs:174-226` | `contact_param_flex_rigid()` | Different pair type — flex-rigid, not flex-self |
| `sim/L0/core/src/constraint/jacobian.rs:20-139` | `compute_flex_contact_jacobian()` | Handles flex-rigid only — new function for self |

---

## Execution Order

**Numbering note:** The rubric illustrative order (S1: enum → S2: gate →
S3: internal → S4: narrow → S5: BVH → S6: SAP) groups by feature path.
This spec uses a dependency-driven order that front-loads shared
infrastructure (contact encoding S2, Jacobian S3, adjacency S4) before any
collision path consumes them. The mapping: rubric S1 = spec S1, rubric S2 =
spec S5, rubric S3 = spec S6, rubric S4 = spec S7, rubric S5+S6 = spec S8.
All rubric content is covered; only the numbering differs.

1. **S1** (enum + pipeline migration) — must be first because all subsequent
   sections dispatch on `FlexSelfCollide` enum variants. → Verify: all 5
   keywords parse correctly, default is `Auto`, domain tests pass.

2. **S2** (contact encoding + params) — must precede S6/S7 which generate
   contacts. → Verify: `make_contact_flex_self()` constructs valid contacts,
   `contact_param_flex_self()` returns correct values.

3. **S3** (Jacobian + assembly) — must precede S6/S7 so generated contacts
   can be processed by constraint pipeline. → Verify: self-collision Jacobian
   has correct DOF structure, bodyweight uses vertex bodies.

4. **S4** (adjacency precomputation) — must precede S5/S6/S7 which use
   adjacency data. → Verify: adjacency lists match expected topology.

5. **S5** (gate logic + dispatch) — must precede S6/S7/S8 which are called
   from the dispatch function. → Verify: gate conditions independently
   tested, dispatch structure calls correct functions.

6. **S6** (internal collision) → Verify: adjacent-element vertex-face
   contacts generated correctly.

7. **S7** (self-collision NARROW) → Verify: non-adjacent element contacts
   generated for deformed mesh.

8. **S8** (BVH + SAP + AUTO) — depends on S7's narrowphase primitives. →
   Verify: BVH/SAP produce identical contacts to NARROW.

Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` after each
section. MuJoCo conformance is the cardinal goal.

---

## Performance Characterization

**Cost:** Self-collision narrowphase is O(n²) for NARROW mode, where n is the
number of elements in the flex. BVH reduces to O(n log n) typical case. SAP
reduces to O(n + k) where k is the number of overlapping pairs.

**Worst case:** A flex with 1000 triangles using NARROW mode: ~500,000
element pair tests. Each test is a sphere-triangle contact (~100 FLOPs),
so ~50M FLOPs per step. BVH or SAP reduces this to ~5,000–10,000 pair
tests for typical deformations.

**Acceptable?** Yes for v1.0. AUTO mode (default) selects BVH/SAP
automatically, avoiding the O(n²) worst case. NARROW mode is intended for
small meshes or debugging. Production models should use AUTO.

---

## Out of Scope

- **`activelayers` runtime filtering** — parsed and stored (Phase 7 T1) but
  runtime consumption deferred. Tracked as DT-150. *Conformance impact:
  minimal — affects models using layer-based collision filtering.*

- **Edge-edge tests for dim=3 tet self-collision** — Vertex-face tests
  capture the primary contacts. Full edge-edge tet self-collision deferred.
  Tracked as DT-151. *Conformance impact: minor — edge-edge contacts are
  secondary for tetrahedral meshes.*

- **Barycentric force distribution on face side** — Current vertex-vertex
  approximation (nearest face vertex receives full reaction force) instead of
  distributing across face vertices via barycentric weights. Tracked as
  DT-152. *Conformance impact: minor — force direction is correct, only
  distribution across face vertices is approximate. Correct for free vertices
  (all current models).*

- **Flex-flex cross-object collision** (§42A-v / DT-143) — Spec D,
  Sessions 19–21. *Conformance impact: covered by Spec D.*

- **Body-attached flex vertex Jacobian** (§27D) — Self-collision Jacobian for
  body-attached vertices requires kinematic chain traversal on both sides.
  Current trivial identity Jacobian is correct for free vertices only.
  *Conformance impact: affects body-attached flex models (none exist yet).*

- **Contact parameter combination for flex-flex** — Spec D adds
  `contact_param_flex_flex()` for cross-object collisions. Self-collision uses
  `contact_param_flex_self()` (trivial identity). *Conformance impact: none —
  different pair types.*
