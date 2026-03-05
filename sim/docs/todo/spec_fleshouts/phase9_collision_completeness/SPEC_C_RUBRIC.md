# Spec C — Heightfield Collision Pairs: Quality Rubric

Grades the Spec C spec on 10 criteria. Target: A+ on every criterion
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

The umbrella spec (PHASE9_UMBRELLA.md) and session plan (SESSION_PLAN.md)
assumed three missing heightfield pairs: hfield-mesh, hfield-plane, and
hfield-hfield. Empirical verification against MuJoCo's collision function
table reveals that **MuJoCo does not support hfield-plane or hfield-hfield
either**. Additionally, MuJoCo uses a single unified function
(`mjc_ConvexHField`) for ALL convex-vs-hfield pairs — the same function
handles sphere, capsule, cylinder, box, ellipsoid, and mesh.

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| hfield-mesh (highest priority) | Supported via `mjc_ConvexHField` in `engine_collision_convex.c` — prism decomposition + CCD/GJK penetration testing. Uses mesh's convex hull as the convex shape. | **In scope** (primary deliverable) |
| hfield-plane | **NOT supported.** `mjCOLLISIONFUNC[mjGEOM_HFIELD][mjGEOM_PLANE] = 0` (null function pointer). MuJoCo silently produces no contacts for this pair. | **Drop** — implementing would be a CortenForge extension, not conformance |
| hfield-hfield | **NOT supported.** `mjCOLLISIONFUNC[mjGEOM_HFIELD][mjGEOM_HFIELD] = 0`. MuJoCo silently produces no contacts. | **Drop** — implementing would be a CortenForge extension, not conformance |
| hfield-cylinder "approximated as capsule" | MuJoCo handles cylinder **exactly** via `mjc_ConvexHField` (same prism approach as all convex types — no capsule approximation) | **In scope** — if implementing the general prism approach, cylinder becomes exact automatically |
| hfield-ellipsoid "approximated as sphere" | MuJoCo handles ellipsoid **exactly** via `mjc_ConvexHField` (no sphere approximation) | **In scope** — same rationale as cylinder |
| Multi-contact per hfield pair | MuJoCo's `mjc_ConvexHField` returns up to `mjMAXCONPAIR` (50) contacts per geom pair. CortenForge returns `Option<Contact>` (single contact). | **In scope** — multi-contact is required for MuJoCo conformance |
| Dispatch ordering (mesh before hfield) | CortenForge's `narrow.rs` checks mesh before hfield (line 87 vs 92). A mesh-hfield pair enters `collide_with_mesh`, which has `unreachable!` for Hfield. This is a latent panic. | **In scope** — fix dispatch ordering |

**Final scope:**
1. Implement `mjc_ConvexHField`-equivalent prism-based collision for
   hfield-vs-convex pairs (replacing current point-sampling approach)
2. Add hfield-mesh support (the missing pair — uses mesh convex hull
   from Spec A as the convex shape)
3. Fix dispatch ordering in `narrow.rs` (move hfield check before mesh
   check, or add explicit mesh-hfield routing)
4. Support multi-contact returns for hfield pairs (up to
   `mjMAXCONPAIR` contacts per pair)
5. As a consequence of (1), existing hfield-cylinder and
   hfield-ellipsoid pairs become exact (no more capsule/sphere
   approximation) — conformance improvement at zero additional cost
6. Defer hfield-SDF to existing `collide_with_sdf` path (SDF check
   fires first in dispatch; no change needed)

**Items NOT in scope:**
- hfield-plane: not in MuJoCo → not conformance work
- hfield-hfield: not in MuJoCo → not conformance work
- hfield-SDF: already handled by `collide_with_sdf` dispatch path

---

## Empirical Ground Truth

### MuJoCo behavioral verification

MuJoCo version: 3.x (source from `google-deepmind/mujoco` main branch).
Verified by reading C source directly — `engine_collision_convex.c`,
`engine_collision_driver.c`, `mjmodel.h`.

### Codebase context

| File | Lines | Role | Spec C impact |
|------|-------|------|---------------|
| `sim/L0/core/src/collision/narrow.rs` | 338 | Narrowphase dispatch | Fix dispatch order: move hfield before mesh (lines 87-93). Change `collide_geoms` return type or add multi-contact path. |
| `sim/L0/core/src/collision/hfield.rs` | 93 | Hfield dispatch | Major rewrite: replace point-sampling dispatch with prism-based `mjc_ConvexHField` equivalent. Change return type or dispatch pattern to support multi-contact (see EGT-7 options). |
| `sim/L0/core/src/heightfield.rs` | 941 | HeightFieldData + contact functions | Add prism construction helpers. Existing `heightfield_sphere_contact` etc. may be deprecated or kept as fallback. |
| `sim/L0/core/src/collision/mesh_collide.rs` | 214 | Mesh collision dispatch | Remove `unreachable!` for Hfield at lines 107, 140 (becomes dead code after dispatch fix). |
| `sim/L0/core/src/collision/mod.rs` | 1065 | Broadphase + collision loop | Change loop from single-contact push to multi-contact extend for hfield pairs. |
| `sim/L0/core/src/gjk_epa.rs` | ~1425 | GJK/EPA | Used by prism-vs-convex testing (existing `gjk_epa_contact` on prism shape). |
| `sim/L0/core/src/collision_shape.rs` | ~1771 | CollisionShape enum | May need `CollisionShape::Prism` variant or construct prisms as `ConvexMesh`. |

**Exhaustive match sites that will break or need updating:**
- `narrow.rs:87-93` — dispatch ordering (mesh before hfield)
- `mesh_collide.rs:107` — `GeomType::Hfield => unreachable!`
- `mesh_collide.rs:140` — `GeomType::Hfield => unreachable!`
- `hfield.rs:48-72` — match on `model.geom_type[other_geom]` (catch-all `_ => return None`)
- `mod.rs:443-448` — single-contact `if let Some(contact)` push loop (mechanism 1)
- `mod.rs:486-499` — single-contact `if let Some(contact)` push loop (mechanism 2, with pair overrides)

### EGT-1: `mjc_ConvexHField` algorithm

**MuJoCo source:** `engine_collision_convex.c` → `mjc_ConvexHField()`

**Signature:**
```c
int mjc_ConvexHField(const mjModel* m, const mjData* d,
                     mjContact* con, int g1, int g2, mjtNum margin)
```

Returns: number of contacts generated (0 to `mjMAXCONPAIR`).

**Algorithm (6 phases):**

1. **Frame setup:** Express geom2 (convex shape) position and orientation
   in hfield's local frame. `pos = mat1^T * (pos2 - pos1)`,
   `mat = mat1^T * mat2`.

2. **Early exit (sphere-box):** Test bounding sphere of geom2 (radius =
   `geom_rbound[g2] + margin`) against hfield extents
   `[-size[0], +size[0]] × [-size[1], +size[1]] × [-size[3], +size[2]]`.
   If no overlap, return 0.

   **CortenForge convention note:** MuJoCo uses `+size[2]` as the upper Z
   bound (correct for normalized [0,1] elevation data). CortenForge stores
   pre-scaled heights, so the actual max height is `HeightFieldData.max_height`.
   For conformance, use `hfield_size[2]` (matches MuJoCo); for tighter
   culling with non-normalized data, use `HeightFieldData.max_height`.
   The spec should choose one and document the rationale.

3. **AABB computation:** Query geom2's support function along ±X, ±Y, ±Z
   in hfield-local frame to get tight AABB `[xmin,xmax] × [ymin,ymax] ×
   [zmin,zmax]`. Perform AABB-AABB test against hfield extents. If no
   overlap, return 0.

   **CortenForge frame note:** `gjk_epa::support()` operates in world
   space — it takes a world-space direction and returns a world-space
   point. To query in hfield-local directions: (1) transform the
   hfield-local direction to world: `world_dir = hf_rotation * local_dir`,
   (2) call `support(shape, &geom2_pose, &world_dir)` → world point,
   (3) transform result to hfield-local:
   `hf_pose.inverse_transform_point(&world_point)`. Alternatively,
   construct a relative pose for geom2 in hfield-local frame and call
   `support` with local-frame directions directly (the function handles
   internal transforms via `pose.rotation.inverse() * direction`).

4. **Sub-grid clipping:** Map AABB to grid indices:
   ```
   cmin = floor((xmin + size[0]) / (2*size[0]) * (ncol-1))
   cmax = ceil ((xmax + size[0]) / (2*size[0]) * (ncol-1))
   rmin = floor((ymin + size[1]) / (2*size[1]) * (nrow-1))
   rmax = ceil ((ymax + size[1]) / (2*size[1]) * (nrow-1))
   ```
   Clamp to `[0, ncol-1]` and `[0, nrow-1]`.

5. **Prism iteration:** For each grid cell `(r, c)` in the sub-grid,
   construct two triangular prisms (one per diagonal triangle). Each prism
   has 6 vertices: 3 top vertices at heightfield surface height (+ margin)
   and 3 bottom vertices at base elevation `-size[3]`. The `addPrismVert`
   helper rotates a sliding window of 6 vertices:
   ```c
   // Shift old vertices
   prism[0] = prism[1]; prism[1] = prism[2];  // bottom
   prism[3] = prism[4]; prism[4] = prism[5];  // top (surface)
   // Add new vertex
   prism[2].x = prism[5].x = dx*c - size[0];
   prism[2].y = prism[5].y = dy*(r+dr) - size[1];
   prism[5].z = hfield_data[(r+dr)*ncol + c] * size[2] + margin;
   ```
   Grid step: `dx = 2*size[0]/(ncol-1)`, `dy = 2*size[1]/(nrow-1)`.

6. **Per-prism collision:** For each prism, skip if all 3 top vertices
   are below `zmin` (geom2's lowest point). Otherwise call
   `mjc_penetration(&prism, &geom2, ...)` — this is MuJoCo's CCD/GJK
   penetration solver treating the prism as a convex shape. Transform
   resulting contact from hfield-local to world frame. Increment contact
   count; break if `ncon >= mjMAXCONPAIR`.

**Post-processing:** If using libccd mode (not native CCD), fix contact
normals via `mjc_fixNormal()`.

### EGT-2: Collision function table

**MuJoCo source:** `engine_collision_driver.c` → `mjCOLLISIONFUNC`

The HFIELD row of the collision function table (geom types in order:
PLANE=0, HFIELD=1, SPHERE=2, CAPSULE=3, ELLIPSOID=4, CYLINDER=5,
BOX=6, MESH=7, SDF=8):

```
HFIELD row: {0, 0, mjc_ConvexHField, mjc_ConvexHField,
             mjc_ConvexHField, mjc_ConvexHField, mjc_ConvexHField,
             mjc_ConvexHField, mjc_HFieldSDF}
```

- `HFIELD-PLANE = 0` (not supported)
- `HFIELD-HFIELD = 0` (not supported)
- `HFIELD-{SPHERE,CAPSULE,ELLIPSOID,CYLINDER,BOX,MESH} = mjc_ConvexHField`
- `HFIELD-SDF = mjc_HFieldSDF` (separate function, not in Spec C scope)

**Key insight:** MuJoCo uses ONE function for all convex-vs-hfield pairs.
There are no separate `mjc_SphereHField`, `mjc_BoxHField`, etc. The prism
approach handles all convex shapes uniformly via the CCD support function.

### EGT-3: `hfield_size` 4-element convention

**MuJoCo source:** `mjmodel.h` → `hfield_size`

```c
mjtNum* hfield_size;  // (x, y, z_top, z_bottom)
```

- `size[0]` = half-extent in X (center-origin: hfield spans `[-size[0], +size[0]]`)
- `size[1]` = half-extent in Y (spans `[-size[1], +size[1]]`)
- `size[2]` = elevation scaling factor (height = `data[i] * size[2]`)
- `size[3]` = base elevation (bottom of prisms at `-size[3]`)

**CortenForge:** Uses `hfield_size: Vec<[f64; 4]>` — same 4-element
layout. The `hfield.rs` dispatch already reads `hf_size` and applies the
centering offset:
```rust
let offset = hf_mat * Vector3::new(-hf_size[0], -hf_size[1], 0.0);
let hf_pose = Pose::from_position_rotation(Point3::from(hf_pos + offset), quat);
```

However, `HeightFieldData` uses corner-origin `[0, extent_x] × [0, extent_y]`
internally. The centering transform in `hfield.rs` maps between these
conventions. The prism approach must use center-origin coordinates
(matching MuJoCo) for the grid math in EGT-1.

### EGT-4: `addPrismVert` helper

**MuJoCo source:** `engine_collision_convex.c` → `addPrismVert()`

```c
static inline void addPrismVert(mjCCDObj* obj, int r, int c, int i,
                                mjtNum dx, mjtNum dy, mjtNum margin) {
  // Shift old vertices (sliding window of 3)
  mji_copy3(obj->prism[0], obj->prism[1]);  // bottom
  mji_copy3(obj->prism[1], obj->prism[2]);
  mji_copy3(obj->prism[3], obj->prism[4]);  // top (surface)
  mji_copy3(obj->prism[4], obj->prism[5]);

  int dr = 1 - i;  // i=0 → dr=1 (next row); i=1 → dr=0 (same row)

  obj->prism[2][0] = obj->prism[5][0] = dx*c - obj->size[0];
  obj->prism[2][1] = obj->prism[5][1] = dy*(r + dr) - obj->size[1];
  obj->prism[5][2] = obj->hfield_data[(r+dr)*obj->hfield_ncol + c] * obj->size[2];
  obj->prism[5][2] += margin;
}
```

The bottom vertices (`prism[0..2]`) have fixed Z = `-size[3]`. The top
vertices (`prism[3..5]`) have Z = `hfield_data[row*ncol + col] * size[2] + margin`.

**Initialization:** The bottom Z = `-size[3]` is NOT set by `addPrismVert` —
it only updates X/Y for slot [2] and X/Y/Z for slot [5]. The bottom Z
must be initialized to `-size[3]` for all prism[0..2] **before the loop
begins**. The sliding window preserves this Z through `mji_copy3` shifts
(which copy all 3 coordinates), and the subsequent X/Y assignment to
prism[2] does not touch its Z.

**Data flow:** Prism construction requires data from TWO model fields:
- `model.hfield_data[hfield_id]` (`HeightFieldData`) — provides heights
  (pre-scaled in CortenForge) and grid dimensions (`width`, `depth`,
  `cell_size`)
- `model.hfield_size[hfield_id]` (`[f64; 4]`) — provides `size[0..1]`
  for centering offsets and `size[3]` for prism base elevation

The `i` parameter alternates between the two triangles of each grid cell:
- `i=0`: triangle with vertex at `(c, r+1)` — lower-left diagonal
- `i=1`: triangle with vertex at `(c, r)` — upper-right diagonal

Each call shifts the window by one vertex position, so after 3 calls the
prism contains 3 fresh vertices forming one triangle.

### EGT-5: `mjMAXCONPAIR` = 50

**MuJoCo source:** `mjmodel.h`

```c
#define mjMAXCONPAIR    50    // maximum number of contacts per geom pair
```

The prism loop breaks when `ncon >= 50`. This is a hard limit per geom
pair — not per frame.

### EGT-6: CortenForge dispatch ordering bug

**File:** `sim/L0/core/src/collision/narrow.rs` lines 82-94

Current dispatch order:
```rust
// line 82: SDF first
if type1 == GeomType::Sdf || type2 == GeomType::Sdf { ... }
// line 87: Mesh second
if type1 == GeomType::Mesh || type2 == GeomType::Mesh { ... }
// line 92: Hfield third
if type1 == GeomType::Hfield || type2 == GeomType::Hfield { ... }
// line 97: Plane fourth
if type1 == GeomType::Plane || type2 == GeomType::Plane { ... }
```

For a Mesh-Hfield pair, the mesh check fires first (line 87). Inside
`collide_with_mesh`, the match on the other geom type has:
```rust
GeomType::Hfield => unreachable!("handled by collide_with_hfield")
```
at lines 107 and 140 of `mesh_collide.rs`.

This is currently "safe" because the broadphase never generates
mesh-hfield pairs (no test coverage). But it's a latent panic that WILL
fire once hfield-mesh collision is enabled.

**Fix:** Move hfield check before mesh check, OR add an explicit
mesh-hfield guard before the mesh check. MuJoCo's collision table treats
hfield as the "owner" of hfield-convex pairs (the hfield decomposes into
prisms), so moving hfield before mesh is the conformant routing.

### EGT-7: CortenForge single-contact architecture

**File:** `sim/L0/core/src/collision/mod.rs`

Two call sites use the single-contact `Option<Contact>` return:

**Mechanism 1 (broadphase loop, lines 443-448):**
```rust
if let Some(contact) = collide_geoms(...) {
    data.contacts.push(contact);
    data.ncon += 1;
}
```

**Mechanism 2 (explicit contact pairs, lines 486-499):**
```rust
if let Some(mut contact) = collide_geoms(...) {
    apply_pair_overrides(&mut contact, pair);
    ...
    data.contacts.push(contact);
    data.ncon += 1;
}
```

Both call sites assume `collide_geoms` returns `Option<Contact>` — at
most one contact per geom pair. MuJoCo's `mjc_ConvexHField` returns up
to 50 contacts. To support multi-contact hfield pairs, the pipeline must
change to allow multiple contacts per `collide_geoms` invocation. Both
mechanism 1 and mechanism 2 loops must be updated (mechanism 2 also
applies pair overrides to each contact).

**Design options:**
- **Option A:** Change `collide_geoms` return to `SmallVec<[Contact; 4]>`
  or `Vec<Contact>`. Simple but changes signature for all pairs.
- **Option B:** Have `collide_with_hfield` take `&mut Vec<Contact>` and
  push directly. Breaks functional style but avoids changing `collide_geoms`.
- **Option C:** Add a separate `collide_geoms_multi` path for hfield
  pairs in the broadphase loop, keeping single-contact path for others.
- **Option D:** Return `Option<Contact>` for single contact (existing
  behavior), and have a separate `collide_geoms_hfield_multi` that
  returns `Vec<Contact>`. The broadphase loop checks geom type and
  dispatches accordingly.

The spec must choose one and justify it.

### EGT-8: HeightFieldData coordinate system

**File:** `sim/L0/core/src/heightfield.rs`

CortenForge's `HeightFieldData` uses corner-origin coordinates:
- X spans `[0, (width-1) * cell_size]`
- Y spans `[0, (depth-1) * cell_size]`
- Heights are **pre-scaled by `size[2]`** in the builder
  (`builder/mesh.rs:238`: `elevation.iter().map(|&e| e * z_top).collect()`).
  The prism approach must NOT multiply by `size[2]` again — heights
  from `HeightFieldData` are already in world-scale meters.
- Index convention: `heights[y * width + x]` (row-major, Y outer loop).
  MuJoCo uses `hfield_data[r * ncol + c]` — mapping is `r` = Y index,
  `c` = X index.

MuJoCo's heightfield uses center-origin coordinates:
- X spans `[-size[0], +size[0]]`
- Y spans `[-size[1], +size[1]]`
- Heights scaled by `size[2]` at runtime (`data[i] * size[2]`)

**Cell size uniformity:** `HeightFieldData` has a single `cell_size`
field (forced square). The builder computes `dx = 2*size[0]/(ncol-1)`
and `dy = 2*size[1]/(nrow-1)`. If they differ by >1%, the builder
resamples to a uniform grid, potentially changing `width`/`depth` from
the original MJCF `ncol`/`nrow`. The prism sub-grid clipping must use
`HeightFieldData.width`/`depth`, not the original MJCF dimensions. With
uniform cells, `dx = dy = cell_size`, simplifying the prism math.

The existing `hfield.rs` dispatch (line 39) applies a centering offset:
```rust
let offset = hf_mat * Vector3::new(-hf_size[0], -hf_size[1], 0.0);
let hf_pose = Pose::from_position_rotation(Point3::from(hf_pos + offset), quat);
```

The prism construction (EGT-4) uses center-origin coordinates directly.
The spec must clarify whether the prism approach works in MuJoCo
center-origin coordinates (with the centering offset applied at the
dispatch level) or in `HeightFieldData` corner-origin coordinates (with
conversion in the prism helper).

### EGT-9: CCD penetration function

**MuJoCo source:** `engine_collision_convex.c` → `mjc_penetration()`

The per-prism collision test uses `mjc_penetration()`, which is MuJoCo's
unified convex-convex penetration solver. It supports both libccd and
native CCD modes.

**CortenForge equivalent:** `gjk_epa_contact()` in `gjk_epa.rs`. This
function takes two `CollisionShape` references and two `Pose` references,
returning `Option<GjkContact>` with penetration depth, contact point, and
normal.

For the prism approach, each hfield prism must be represented as a
`CollisionShape`. Options:
- **`CollisionShape::ConvexMesh`** with 6 vertices (the prism). This
  works because prisms are convex. The GJK support function for
  `ConvexMesh` iterates all vertices (fine for 6 vertices).
- **New `CollisionShape::Prism` variant** with specialized support
  function. More efficient but adds enum complexity.

`ConvexMesh` with 6 vertices is likely sufficient — the per-vertex scan
is O(6) which is negligible. No `HullGraph` needed (below the 10-vertex
hill-climbing threshold).

**Pose pairing:** `gjk_epa_contact` takes `(shape_a, pose_a, shape_b,
pose_b)` and operates in world space. Two valid approaches for the
prism-vs-convex call:
- **Option 1 (hfield-local prisms):** Construct prism vertices in
  center-origin hfield-local coords (matching MuJoCo's grid math).
  Pass the hfield's world `Pose` as `pose_a`. Pass geom2's world `Pose`
  as `pose_b`. The `support()` function handles all frame transforms
  internally.
- **Option 2 (world-space prisms):** Transform prism vertices to world
  space during construction. Pass identity `Pose` for the prism.
  Pass geom2's world `Pose` as `pose_b`.

Option 1 is simpler (prism construction stays in grid math, no per-vertex
world transform) and matches MuJoCo's pattern (collision in hfield
frame, contact transformed to world at the end). The spec must choose
one and state it.

---

## Criteria

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does — exact function names, field
> names, calling conventions, and edge cases. No hand-waving.

| Grade | Bar |
|-------|-----|
| **A+** | Every MuJoCo function cited with source file and exact behavior: `mjc_ConvexHField` (engine_collision_convex.c), `addPrismVert` helper, `mjc_penetration` penetration solver, `mjCOLLISIONFUNC` collision table. The 6-phase algorithm (EGT-1) is described step-by-step. `hfield_size` 4-element semantics (EGT-3) documented. Collision table entries for HFIELD row verified — hfield-plane=0 and hfield-hfield=0 explicitly noted as NOT supported. Prism vertex construction from `addPrismVert` (EGT-4) reproduced with coordinate formulas. Edge cases addressed: empty sub-grid (no contacts), all prism tops below zmin (skip), mjMAXCONPAIR limit (50), margin addition to prism top vertices, base elevation at -size[3]. |
| **A** | MuJoCo behavior described correctly from C source. Minor gaps in edge-case coverage. |
| **B** | Correct at high level, but missing specifics (e.g., prism approach described without vertex formulas, or collision table not verified). |
| **C** | Partially correct. Some MuJoCo behavior misunderstood. Hfield-plane or hfield-hfield incorrectly described as supported. |

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. No "see MuJoCo source"
> gaps. An implementer can build the prism-based hfield collision without
> reading MuJoCo source.

| Grade | Bar |
|-------|-----|
| **A+** | The complete `mjc_ConvexHField` algorithm is written out in Rust pseudocode: frame transformation, AABB computation via support queries, sub-grid clipping formulas, `addPrismVert` sliding window, prism-vs-convex penetration test via `gjk_epa_contact`, contact coordinate transform, `mjMAXCONPAIR` break. Every formula from EGT-1 and EGT-4 is present. The prism `CollisionShape` representation is defined (ConvexMesh with 6 vertices or dedicated Prism variant). Contact normal direction convention is explicit. |
| **A** | Algorithm is complete. One or two minor details left implicit. |
| **B** | Algorithm structure is clear but prism construction or sub-grid formulas are hand-waved. |
| **C** | Skeleton only — "decompose hfield into prisms and test." |

### P3. Convention Awareness

> Spec explicitly addresses coordinate system conventions, normal
> direction, and the CortenForge ↔ MuJoCo translation layer.

| Grade | Bar |
|-------|-----|
| **A+** | MuJoCo center-origin vs CortenForge corner-origin coordinate systems documented with explicit conversion rules (EGT-8). `hfield_size[0..3]` mapping to CortenForge's `HeightFieldData` fields documented. Contact normal convention: MuJoCo's prism normal direction vs CortenForge's "from geom_a toward geom_b" convention — translation rule stated. Prism vertex Z: MuJoCo uses `data[i] * size[2]`; CortenForge stores heights pre-scaled by `size[2]` in the builder — no double-scaling rule stated. Index mapping: MuJoCo `(r, c)` = CortenForge `(y_index, x_index)` — `heights[y * width + x]` vs `hfield_data[r * ncol + c]`. Pose convention: MuJoCo modifies `mat2`/`pos2` in-place (relative frame); CortenForge must compute relative transform without mutation. |
| **A** | Major conventions documented. Minor coordinate mapping details left implicit. |
| **B** | Some conventions noted, others not — risk of wrong coordinates in prism construction. |
| **C** | MuJoCo code pasted without adaptation to CortenForge conventions. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. Contains concrete values
> and model configurations.

| Grade | Bar |
|-------|-----|
| **A+** | Every runtime AC has the three-part structure: (1) concrete model (hfield dimensions, geom type/size/position), (2) exact expected contact count and approximate contact positions/normals/depths, (3) what field to check. At least one AC for hfield-mesh with expected values. At least one AC for hfield-sphere using the new prism approach verifying contacts are still generated (regression guard — values may differ from point-sampling, but contacts must exist for same scenarios). Code-review ACs for dispatch ordering fix and multi-contact pipeline change. AC for mjMAXCONPAIR=50 limit. AC for hfield-cylinder/ellipsoid now being exact (no approximation). AC for hfield-mesh where mesh has no convex hull (expected: 0 contacts). |
| **A** | ACs are testable. Some lack exact numerical expectations. |
| **B** | ACs are directionally correct but vague ("contacts should be generated"). |
| **C** | ACs are aspirational statements, not tests. |

### P5. Test Plan Coverage

> Tests cover happy path, edge cases, regressions, and interactions.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present. Edge case inventory: flat hfield (all same height), single-cell hfield (2×2 grid), geom entirely outside hfield bounds (no contacts), geom straddling hfield edge (partial sub-grid), geom smaller than one cell (single prism test), geom larger than entire hfield (full grid), degenerate prism (coplanar top vertices), tilted hfield (non-identity hfield pose), mesh-on-hfield (the primary missing pair). Negative cases: hfield-plane returns no contacts, hfield-hfield returns no contacts. Regression: existing sphere/capsule/box-on-hfield tests still pass (or updated to match new multi-contact behavior). At least one multi-contact test verifying >1 contact from a single hfield pair. |
| **A** | Good coverage. Minor edge-case gaps. |
| **B** | Happy path covered. Edge cases sparse. No multi-contact verification. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and interactions with other specs
> are explicitly stated.

| Grade | Bar |
|-------|-----|
| **A+** | Execution order is unambiguous. Spec A (convex hull) dependency stated: hfield-mesh uses mesh's convex hull — if `mesh.convex_hull()` returns `None`, return 0 contacts (MuJoCo always computes hulls for mesh geoms, so absent hull = no collision is conformant). No dependency on Spec D (CCD) or Spec E (flex). Dispatch ordering fix (EGT-6) must land before hfield-mesh tests can run. Multi-contact pipeline change (EGT-7) must land before multi-contact tests. Section ordering reflects these dependencies. |
| **A** | Order is clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Spec identifies every file touched, every behavior that changes, and
> every existing test that might break.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description (EGT codebase context table). Behavioral changes documented: (1) hfield-sphere/capsule/box may return different contact positions/normals due to prism approach replacing point-sampling — regression risk assessed, (2) hfield-cylinder/ellipsoid contacts change from approximation to exact — expected value changes documented, (3) dispatch reordering makes mesh-hfield go to hfield handler instead of mesh handler — `unreachable!` in mesh_collide.rs becomes dead code. Existing test impact: hfield tests in `heightfield.rs` (11 tests) may need expected-value updates. Test count in `sim-core` (current baseline from domain test run). |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. Regression risk from prism approach unaddressed. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions within the spec. Shared concepts use identical
> terminology throughout.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology is uniform: "prism" always means triangular prism (6 vertices), "sub-grid" always means the clipped cell range, "center-origin" vs "corner-origin" used consistently. File paths in Specification sections match Files Affected table. AC numbers match between AC section and Test Plan / Traceability Matrix. Edge cases in MuJoCo Reference appear in Test Plan. |
| **A** | Consistent. One or two minor terminology inconsistencies. |
| **B** | Some sections use different names for the same concept. |
| **C** | Contradictions between sections. |

### P9. Dispatch Architecture

> The spec correctly handles the narrowphase dispatch routing for hfield
> pairs, including the ordering fix, the multi-contact return type change,
> and the interaction with the existing single-contact dispatch for
> non-hfield pairs.

| Grade | Bar |
|-------|-----|
| **A+** | Dispatch fix is precisely specified: which check moves where in `narrow.rs`, what happens to the `unreachable!` in `mesh_collide.rs` (removed or replaced with routing to hfield). Multi-contact return strategy chosen with justification (EGT-7 options evaluated). The chosen strategy preserves backward compatibility for non-hfield pairs (they still use `Option<Contact>`). The broadphase loop in `mod.rs` is updated to handle both single and multi-contact returns. No double-dispatch risk (a pair never enters both mesh and hfield handlers). |
| **A** | Dispatch fix is clear. Multi-contact strategy chosen but minor integration details missing. |
| **B** | Dispatch fix mentioned but not fully specified. Multi-contact strategy hand-waved. |
| **C** | Dispatch issue not addressed. |

**Boundary with P1:** P1 grades whether the spec correctly describes
MuJoCo's collision table and function ownership. P9 grades whether the
CortenForge dispatch implementation correctly routes pairs to match
MuJoCo's table.

### P10. Multi-Contact Conformance

> The spec correctly handles the transition from single-contact to
> multi-contact for hfield pairs, including the contact limit, the
> pipeline integration, and the effect on downstream constraint solving.

| Grade | Bar |
|-------|-----|
| **A+** | Multi-contact generation is fully specified: how many contacts per prism (1 from `gjk_epa_contact`), how contacts accumulate across prisms (up to `mjMAXCONPAIR=50`), how the contact limit is enforced (break from prism loop). Pipeline integration: how multi-contact results flow into `data.contacts` and `data.ncon` for BOTH mechanism 1 (broadphase, `mod.rs:443`) and mechanism 2 (explicit pairs, `mod.rs:486`) loops. Mechanism 2 must apply `apply_pair_overrides` to each contact in the multi-contact set. Downstream impact: constraint solver already handles multiple contacts per geom pair (each contact is independent) — no solver changes needed. Contact ordering: prisms iterated in row-major order matching MuJoCo's `(r, c)` loop — contact order may affect solver behavior with contact limits. |
| **A** | Multi-contact generation complete. Minor downstream impact details missing. |
| **B** | Multi-contact mentioned but implementation details incomplete. |
| **C** | Single-contact assumption preserved. |

**Boundary with P9:** P9 grades dispatch routing and return type. P10
grades the algorithmic and conformance aspects of generating multiple
contacts from a single hfield pair.

---

## Rubric Self-Audit

### Self-audit checklist

- [x] **Specificity:** Every A+ bar names specific functions
      (`mjc_ConvexHField`, `addPrismVert`, `gjk_epa_contact`), specific
      files (`narrow.rs:87-93`, `hfield.rs:48-72`, `mod.rs:443-448` +
      `mod.rs:486-499`), and specific values (`mjMAXCONPAIR=50`,
      `hfield_size[0..3]`). Two independent reviewers could assign
      consistent grades.

- [x] **Non-overlap:** P1 (MuJoCo reference) vs P9 (dispatch routing) —
      P1 grades correctness of MuJoCo description, P9 grades correctness
      of CortenForge routing to match. P9 (dispatch) vs P10 (multi-contact)
      — P9 grades return type and routing, P10 grades contact generation
      algorithm and pipeline integration. Boundaries documented in each
      criterion.

- [x] **Completeness:** The 10 criteria cover: MuJoCo reference (P1),
      algorithm (P2), conventions (P3), ACs (P4), tests (P5),
      dependencies (P6), blast radius (P7), consistency (P8), dispatch
      architecture (P9), multi-contact conformance (P10). No meaningful
      gap could exist outside these dimensions for this task.

- [x] **Gradeability:** P1 → MuJoCo Reference + Key Behaviors. P2 →
      Specification sections. P3 → Convention Notes. P4 → Acceptance
      Criteria. P5 → Test Plan. P6 → Prerequisites + Execution Order.
      P7 → Risk & Blast Radius. P8 → cross-cutting. P9 → Dispatch fix
      section. P10 → Multi-contact section.

- [x] **Conformance primacy:** P1 is tailored with specific MuJoCo C
      function names and source files. The A+ bar for P1 requires
      verification of the collision table entries. P4 requires
      MuJoCo-verified expected values. P5 requires conformance tests.
      The scope adjustment drops non-MuJoCo features (hfield-plane,
      hfield-hfield).

- [x] **Empirical grounding:** EGT-1 through EGT-9 are filled in from
      C source reading. Every A+ bar that references specific MuJoCo
      behavior has a corresponding EGT-N entry. No bars were written
      from header-file assumptions — the collision table was verified
      from `engine_collision_driver.c`, the algorithm from
      `engine_collision_convex.c`.

### Criterion → Spec section mapping

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P1 | MuJoCo Reference, Key Behaviors table, Convention Notes |
| P2 | Specification (S1, S2, ...) |
| P3 | Convention Notes, Specification code |
| P4 | Acceptance Criteria |
| P5 | Test Plan, AC→Test Traceability Matrix, Edge Case Inventory |
| P6 | Prerequisites, Execution Order |
| P7 | Risk & Blast Radius (Behavioral Changes, Files Affected, Existing Test Impact) |
| P8 | *Cross-cutting — all sections checked for mutual consistency* |
| P9 | Dispatch fix section (within Specification or dedicated section) |
| P10 | Multi-contact section (within Specification or dedicated section) |

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
| P9. Dispatch Architecture | | |
| P10. Multi-Contact Conformance | | |

**Overall: — (Rev 4)**

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| R1 | Scope | Umbrella assumed hfield-plane and hfield-hfield are MuJoCo features | Rubric Phase 1 (EGT-2: collision table verification) | Dropped both — MuJoCo collision table has 0 (null) for both pairs | Rubric Rev 1 |
| R2 | Scope | Umbrella assumed separate hfield functions per geom type | Rubric Phase 1 (EGT-1: mjc_ConvexHField) | Corrected: MuJoCo uses one unified function for all convex-vs-hfield pairs | Rubric Rev 1 |
| R3 | P9 | Dispatch ordering bug not in umbrella | Rubric Phase 1 (EGT-6: narrow.rs line 87-93) | Added to scope: dispatch fix is prerequisite for hfield-mesh | Rubric Rev 1 |
| R4 | P10 | Multi-contact architecture not in umbrella | Rubric Phase 1 (EGT-5 + EGT-7) | Added to scope: multi-contact returns required for conformance | Rubric Rev 1 |
| R5 | Scope | Umbrella noted cylinder/ellipsoid approximations but didn't flag as conformance gap | Rubric Phase 1 (EGT-2) | Added: prism approach makes all convex types exact — conformance improvement | Rubric Rev 1 |
| R6 | P3 | EGT-8 claimed CortenForge stores raw heights — actually pre-scaled by `size[2]` in builder | Stress test (builder/mesh.rs:238) | Fixed: EGT-8 now documents pre-scaling, warns against double-scaling | Rubric Rev 2 |
| R7 | P2 | EGT-4 code block comments swapped top/bottom vertex groups | Stress test (prism[5].z = surface height → prism[3..5] is top) | Fixed: inline comments corrected | Rubric Rev 2 |
| R8 | P10 | EGT-7 only cited mechanism 1 call site (mod.rs:443); missed mechanism 2 (mod.rs:486) | Stress test (mod.rs:486-499 also uses Option\<Contact\>) | Fixed: both call sites documented in EGT-7 and P10 | Rubric Rev 2 |
| R9 | P6 | No fallback behavior specified for hfield-mesh when mesh has no convex hull | Stress test (mesh.convex_hull() can return None) | Fixed: P6 specifies 0 contacts when hull absent; P4 requires AC for this case | Rubric Rev 2 |
| R10 | P3 | HeightFieldData cell_size uniformity and resampling implications not documented | Stress test (builder/mesh.rs:235-272) | Fixed: EGT-8 documents cell_size constraint and resampling | Rubric Rev 2 |
| R11 | P3 | Row/column index mapping between MuJoCo (r,c) and CortenForge (y,x) not documented | Stress test (heightfield.rs:86 access pattern) | Fixed: EGT-8 and P3 document index mapping | Rubric Rev 2 |
| R12 | Codebase | gjk_epa.rs and collision_shape.rs line counts stale (~1367/~1734 vs actual 1425/1771) | Stress test (wc -l) | Fixed: updated approximate counts | Rubric Rev 2 |
| R13 | P2 | EGT-1 Phase 5 code block still had swapped top/bottom comments (second instance missed in Rev 2) | Stress test round 2 | Fixed: comments corrected to match EGT-4 | Rubric Rev 3 |
| R14 | P7 | Exhaustive match sites list only mentioned mod.rs:443 (mechanism 1), not mod.rs:486 (mechanism 2) | Stress test round 2 | Fixed: added mechanism 2 entry | Rubric Rev 3 |
| R15 | P2 | Prism bottom vertex Z = -size[3] initialization before loop not documented in EGT-4 | Stress test round 2 (addPrismVert never sets prism[2].z) | Fixed: added initialization note and data flow section to EGT-4 | Rubric Rev 3 |
| R16 | P2 | Pose pairing for gjk_epa_contact prism-vs-convex call not discussed | Stress test round 2 (support() operates in world space) | Fixed: added pose pairing options to EGT-9 | Rubric Rev 3 |
| R17 | P2 | AABB support query frame management underspecified — support() is world-space but EGT-1 says "in hfield-local frame" | Stress test round 2 (gjk_epa.rs:219 signature) | Fixed: added CortenForge frame note to EGT-1 Phase 3 | Rubric Rev 3 |
| R18 | Self-audit | Specificity bullet referenced mod.rs:443-448 but not mod.rs:486-499 | Stress test round 2 | Fixed: updated self-audit | Rubric Rev 3 |
| R19 | P2 | hfield_size[3] (base elevation) stored in model.hfield_size, not HeightFieldData — data flow gap | Stress test round 2 (HeightFieldData.aabb() has no base elevation) | Fixed: added data flow note to EGT-4 | Rubric Rev 3 |
| R20 | P3 | EGT-1 Phase 2 Z extent uses +size[2] but CortenForge pre-scales heights — max_height may differ | Stress test round 2 (HeightFieldData.max_height vs hfield_size[2]) | Fixed: added convention translation note to EGT-1 Phase 2 | Rubric Rev 3 |
| R21 | P8 | Codebase context table pre-decided hfield.rs return type as `Vec<Contact>` while EGT-7 presents 4 open options | Stress test round 3 (line 74 vs EGT-7 lines 339-351) | Fixed: table now says "see EGT-7 options" | Rubric Rev 4 |
| R22 | P8 | P4 "matching existing behavior" contradicts P7 "may return different values" — ambiguous regression guard | Stress test round 3 (P4 vs P7 A+ bars) | Fixed: P4 now says "verifying contacts are still generated" | Rubric Rev 4 |
