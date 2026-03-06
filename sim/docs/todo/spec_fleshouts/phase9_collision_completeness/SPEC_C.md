# Spec C — Heightfield Collision Pairs: Spec

**Status:** Draft
**Phase:** Roadmap Phase 9 — Collision Completeness
**Effort:** L
**MuJoCo ref:** `mjc_ConvexHField()` in `engine_collision_convex.c`
**MuJoCo version:** 3.x (source from `google-deepmind/mujoco` main branch)
**Test baseline:** 2,489 sim domain tests
**Prerequisites:**
- Spec A (§65 convex hull) landed — hfield-mesh uses mesh convex hull
  (`ba6c261`)
- Phase 9 umbrella (`ade7750`)

**Independence:** This spec is independent of Specs B, D, E per the umbrella
dependency graph. Shared files: `narrow.rs` (dispatch ordering fix is
contained within this spec — no other Phase 9 spec modifies dispatch
ordering). `heightfield.rs` is read-only for Spec E (flex-vs-hfield reuses
existing sphere contact); Spec C is the sole modifier.

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical results to MuJoCo for the feature described below.
> Every algorithm, every acceptance criterion, and every test is in service
> of this goal. The MuJoCo C source code is the single source of truth —
> not the MuJoCo documentation, not intuition about "what should happen,"
> not a Rust-idiomatic reinterpretation. When in doubt, read the C source
> and match its behavior exactly.

---

## Scope Adjustment

The rubric (SPEC_C_RUBRIC.md) performed empirical verification against
MuJoCo's collision function table. Key corrections from the umbrella:

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| hfield-mesh (highest priority) | Supported via `mjc_ConvexHField` — prism decomposition + GJK penetration. Uses mesh's convex hull as the convex shape. | **In scope** (primary deliverable) |
| hfield-plane | **NOT supported.** `mjCOLLISIONFUNC[mjGEOM_HFIELD][mjGEOM_PLANE] = 0`. | **Drop** — not conformance work |
| hfield-hfield | **NOT supported.** `mjCOLLISIONFUNC[mjGEOM_HFIELD][mjGEOM_HFIELD] = 0`. | **Drop** — not conformance work |
| hfield-cylinder "approximated as capsule" | MuJoCo handles cylinder **exactly** via `mjc_ConvexHField` (same prism approach as all convex types) | **In scope** — prism approach makes cylinder exact |
| hfield-ellipsoid "approximated as sphere" | MuJoCo handles ellipsoid **exactly** via `mjc_ConvexHField` | **In scope** — prism approach makes ellipsoid exact |
| Multi-contact per hfield pair | MuJoCo returns up to `mjMAXCONPAIR` (50) contacts per pair. CortenForge returns `Option<Contact>` (single). | **In scope** — multi-contact required |
| Dispatch ordering | Mesh check fires before hfield in `narrow.rs` — latent panic for mesh-hfield pairs | **In scope** — fix dispatch ordering |

**Final scope:**
1. Replace current point-sampling hfield collision with MuJoCo-conformant
   prism-based `mjc_ConvexHField` equivalent for ALL convex-vs-hfield pairs
2. Add hfield-mesh support (uses mesh convex hull from Spec A)
3. Fix dispatch ordering in `narrow.rs` (move hfield before mesh)
4. Support multi-contact returns (up to 50 contacts per hfield pair)
5. Existing hfield-cylinder and hfield-ellipsoid become exact (no more
   capsule/sphere approximation)

**Not in scope:**
- hfield-plane: not in MuJoCo
- hfield-hfield: not in MuJoCo
- hfield-SDF: already handled by `collide_with_sdf` dispatch path

---

## Problem Statement

**Conformance gap:** MuJoCo implements heightfield collision for ALL convex
geom types (sphere, capsule, cylinder, ellipsoid, box, mesh) using a single
unified function `mjc_ConvexHField()` in `engine_collision_convex.c`. This
function decomposes the heightfield into triangular prisms and tests each
prism against the convex shape using GJK/EPA penetration. It returns up to
50 contacts per geom pair.

CortenForge does not implement this. Instead, it uses ad-hoc point-sampling
functions per geom type (`heightfield_sphere_contact`,
`heightfield_capsule_contact`, `heightfield_box_contact`), each returning a
single contact. Cylinder and ellipsoid are approximated (cylinder → capsule,
ellipsoid → sphere with max radius). Mesh-hfield pairs are completely
unsupported — a mesh-hfield pair triggers `unreachable!` in
`mesh_collide.rs` due to dispatch ordering.

The conformance gaps are:
1. Wrong algorithm: point-sampling vs prism decomposition produces different
   contact positions, normals, and penetration depths
2. Missing pair: hfield-mesh not supported (latent panic)
3. Single-contact: MuJoCo generates up to 50 contacts per hfield pair;
   CortenForge generates at most 1
4. Approximated pairs: cylinder/ellipsoid are approximated instead of exact

---

## MuJoCo Reference

> **This is the most important section of the spec.** Everything downstream
> is derived from what's documented here.

### `mjc_ConvexHField` — unified convex-vs-heightfield collision

**Source:** `engine_collision_convex.c` → `mjc_ConvexHField()`

**Signature:**
```c
int mjc_ConvexHField(const mjModel* m, const mjData* d,
                     mjContact* con, int g1, int g2, mjtNum margin)
```

Returns: number of contacts generated (0 to `mjMAXCONPAIR` = 50).

**Algorithm (6 phases):**

**Phase 1 — Frame setup:** The convex geom (geom2, the non-hfield) is
expressed in the hfield's local frame:
```c
pos = mat1^T * (pos2 - pos1)    // hfield-local position
mat = mat1^T * mat2              // hfield-local orientation
```
All subsequent work happens in hfield-local coordinates until the final
contact transform.

**Phase 2 — Bounding sphere early exit:** Test the bounding sphere of
geom2 (radius = `geom_rbound[g2] + margin`) against the hfield extents
box `[-size[0], +size[0]] × [-size[1], +size[1]] × [-size[3], +size[2]]`.
If the sphere does not overlap the hfield's extent box, return 0.

**Phase 3 — AABB computation via support queries:** Query geom2's support
function along ±X, ±Y, ±Z in hfield-local frame to get a tight AABB
`[xmin, xmax] × [ymin, ymax] × [zmin, zmax]`. Test this AABB against the
hfield extents. If no overlap, return 0.

**Phase 4 — Sub-grid clipping:** Map AABB to grid cell indices:
```
cmin = floor((xmin + size[0]) / (2*size[0]) * (ncol-1))
cmax = ceil ((xmax + size[0]) / (2*size[0]) * (ncol-1))
rmin = floor((ymin + size[1]) / (2*size[1]) * (nrow-1))
rmax = ceil ((ymax + size[1]) / (2*size[1]) * (nrow-1))
```
Clamp to `[0, ncol-1]` and `[0, nrow-1]`.

**Phase 5 — Prism iteration:** For each grid cell `(r, c)` in the sub-grid,
construct two triangular prisms (one per diagonal triangle). Each prism has
6 vertices: 3 bottom vertices at base elevation `-size[3]` and 3 top
vertices at heightfield surface height + margin.

The `addPrismVert` helper uses a sliding window:
```c
static inline void addPrismVert(mjCCDObj* obj, int r, int c, int i,
                                mjtNum dx, mjtNum dy, mjtNum margin) {
  // Shift old vertices (sliding window of 3 per layer)
  mji_copy3(obj->prism[0], obj->prism[1]);  // bottom
  mji_copy3(obj->prism[1], obj->prism[2]);
  mji_copy3(obj->prism[3], obj->prism[4]);  // top (surface)
  mji_copy3(obj->prism[4], obj->prism[5]);

  int dr = 1 - i;  // i=0 → dr=1 (next row); i=1 → dr=0 (same row)

  obj->prism[2][0] = obj->prism[5][0] = dx*c - size[0];
  obj->prism[2][1] = obj->prism[5][1] = dy*(r + dr) - size[1];
  obj->prism[5][2] = hfield_data[(r+dr)*ncol + c] * size[2] + margin;
  // prism[2][2] remains at -size[3] (initialized before loop, preserved by copy)
}
```

Grid step sizes: `dx = 2*size[0]/(ncol-1)`, `dy = 2*size[1]/(nrow-1)`.

Bottom vertex Z = `-size[3]` is initialized for all `prism[0..2]` before
the loop begins. The sliding window copy (`mji_copy3`) preserves this Z,
and the X/Y assignment to `prism[2]` does not modify Z.

The `i` parameter alternates between the two triangles of each grid cell:
- `i=0`: triangle with vertex at `(c, r+1)` — uses `dr=1`
- `i=1`: triangle with vertex at `(c, r)` — uses `dr=0`

Each call shifts the window by one vertex, so after 3 calls the prism
contains 3 fresh vertices forming one triangle (top and bottom layers).

**Phase 6 — Per-prism collision:** For each prism, skip if all 3 top
vertices are below `zmin` (geom2's lowest support point). Otherwise call
`mjc_penetration(&prism, &geom2, ...)` — MuJoCo's unified CCD/GJK
penetration solver treating the prism as a convex shape. Transform
resulting contact from hfield-local to world frame. Increment contact
count; break if `ncon >= mjMAXCONPAIR` (50).

### Collision function table — HFIELD row

**Source:** `engine_collision_driver.c` → `mjCOLLISIONFUNC`

```
HFIELD row: {0, 0, mjc_ConvexHField, mjc_ConvexHField,
             mjc_ConvexHField, mjc_ConvexHField, mjc_ConvexHField,
             mjc_ConvexHField, mjc_HFieldSDF}
```

- `HFIELD-PLANE = 0` (not supported)
- `HFIELD-HFIELD = 0` (not supported)
- `HFIELD-{SPHERE,CAPSULE,ELLIPSOID,CYLINDER,BOX,MESH} = mjc_ConvexHField`
- `HFIELD-SDF = mjc_HFieldSDF` (separate function, not in Spec C scope)

**Key insight:** ONE function for all convex-vs-hfield pairs. No separate
per-type functions.

### `hfield_size` — 4-element convention

**Source:** `mjmodel.h` → `hfield_size`

```c
mjtNum* hfield_size;  // (x, y, z_top, z_bottom)
```

- `size[0]` = half-extent in X (hfield spans `[-size[0], +size[0]]`)
- `size[1]` = half-extent in Y (spans `[-size[1], +size[1]]`)
- `size[2]` = elevation scaling factor (height = `data[i] * size[2]`)
- `size[3]` = base elevation (bottom of prisms at `-size[3]`)

### `mjMAXCONPAIR` = 50

**Source:** `mjmodel.h`

Hard limit per geom pair. The prism loop breaks when `ncon >= 50`.

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| Convex-hfield collision algorithm | `mjc_ConvexHField` — prism decomposition + GJK penetration, returns 0..50 contacts | Point-sampling per type, returns 0..1 contact |
| Hfield-mesh | Supported via `mjc_ConvexHField` using mesh convex hull | `unreachable!` panic — dispatch sends to mesh handler |
| Hfield-cylinder | Exact via `mjc_ConvexHField` | Approximated as capsule |
| Hfield-ellipsoid | Exact via `mjc_ConvexHField` | Approximated as sphere (max radius) |
| Multi-contact | Up to 50 contacts per hfield pair | At most 1 contact |
| Hfield-plane | Not supported (null entry) | Returns `None` (correct) |
| Hfield-hfield | Not supported (null entry) | Returns `None` (correct) |
| Prism base elevation | `-size[3]` | Not applicable (no prisms) |
| Margin handling | Added to prism top vertex Z | Not threaded to hfield helpers |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| Heightfield origin | Center-origin: X in `[-size[0], +size[0]]`, Y in `[-size[1], +size[1]]` | Corner-origin: X in `[0, extent_x]`, Y in `[0, extent_y]`. Old code applied centering offset in `hfield.rs:39` | Prism construction uses MuJoCo center-origin coords directly (`dx * c - hf_x_half`). The hfield Pose uses `hf_pos` WITHOUT centering offset — the offset is baked into prism vertex positions. Heights accessed from `HeightFieldData` using corner-origin grid indices (MuJoCo `c` = CortenForge `x`, MuJoCo `r` = CortenForge `y`) |
| Height scaling | `data[i] * size[2]` at runtime | Pre-scaled by `size[2]` in builder (`builder/mesh.rs:238`). Heights in `HeightFieldData` are already in world-scale meters | **Do NOT multiply by `size[2]` again.** Read heights directly from `HeightFieldData.get(x, y)` |
| Grid indices | `hfield_data[r * ncol + c]` where r=row (Y), c=col (X) | `heights[y * width + x]` where y=row, x=col | Direct mapping: MuJoCo `r` = CortenForge `y`, MuJoCo `c` = CortenForge `x` |
| Grid dimensions | `nrow` × `ncol` | `depth` × `width` | MuJoCo `nrow` = CortenForge `depth`, MuJoCo `ncol` = CortenForge `width` |
| Cell size | `dx = 2*size[0]/(ncol-1)`, `dy = 2*size[1]/(nrow-1)` — may differ | Single `cell_size` (forced uniform). Builder resamples if `dx != dy` | Compute `dx = 2*hf_size[0]/(width-1)` and `dy = 2*hf_size[1]/(depth-1)` from `hf_size` and grid dimensions (matches MuJoCo formula exactly, ensures vertices span `[-size, +size]`). Use `HeightFieldData.width()`/`depth()` for grid dimensions. **Do NOT use `cell_size()` for prism math** — after resampling, `cell_size` may differ from `2*size[0]/(width-1)` due to rounding in `new_ncol = round(2*size[0]/cell_size) + 1`. |
| Base elevation | `-size[3]` from `hfield_size[3]` | Not stored in `HeightFieldData`. Available from `model.hfield_size[hfield_id][3]` | Read `hfield_size[3]` from model for prism bottom Z |
| Penetration test | `mjc_penetration()` — CCD/GJK solver | `gjk_epa_contact()` in `gjk_epa.rs` — GJK/EPA solver | Direct substitute: `gjk_epa_contact(prism_shape, prism_pose, convex_shape, convex_pose)` |
| Contact normal | Prism normal from GJK/EPA, may be corrected by `mjc_fixNormal()` in libccd mode | GJK/EPA normal points from shape A to shape B | Direct port — CortenForge's GJK/EPA uses native mode (no libccd), so no fixNormal needed |
| Pose mutation | MuJoCo modifies `pos2`/`mat2` in-place (relative frame computation) | CortenForge must compute relative transform without mutation | Compute `hf_local_pos = hf_mat^T * (other_pos - hf_pos)` and `hf_local_mat = hf_mat^T * other_mat` as new variables |
| Upper Z extent | `+size[2]` (elevation scaling factor used as extent — works because MuJoCo's elevation data is normalized to [0,1], so max height = 1.0 × size[2] = size[2]) | `HeightFieldData.max_height()` (pre-scaled actual maximum = `max(data) * size[2]`) | Use `max_height()` for bounding sphere/AABB tests. **Rationale:** MuJoCo uses `size[2]` as Z extent because it assumes `max(data) == 1.0` (normalized data). In CortenForge, heights are pre-scaled, so `max_height()` gives the actual maximum. When data is normalized, `max_height == size[2]` (identical). When data is not fully normalized (e.g., `max(data) < 1.0`), `max_height < size[2]` gives a tighter bound — strictly better culling, never misses a contact. |

---

## Architecture Decisions

### AD-1: Multi-contact return type

**Problem:** `collide_geoms` returns `Option<Contact>` (single contact).
The prism approach generates up to 50 contacts per hfield pair. Both
mechanism 1 (broadphase, `mod.rs:443`) and mechanism 2 (explicit pairs,
`mod.rs:486`) loops consume this return type.

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| A | Change `collide_geoms` return to `SmallVec<[Contact; 4]>` | Uniform interface for all pairs | Changes signature for ALL pairs; every single-contact pair must wrap in SmallVec |
| B | `collide_with_hfield` takes `&mut Vec<Contact>` and pushes directly | Avoids changing `collide_geoms` signature | Breaks functional style; hfield handler needs access to contact list |
| C | Separate `collide_geoms_multi` function for hfield pairs | Clean separation; existing code unchanged | Two dispatch paths; mechanism 2 overrides must be duplicated |
| D | `collide_geoms` returns `Option<Contact>` as before; new `collide_geoms_hfield_multi` returns `Vec<Contact>` dispatched by type check in broadphase loop | Minimal change to existing code; only the calling loops need a type check | Two function entry points; slight code duplication in dispatch |

**Chosen:** Option D — The broadphase and mechanism-2 loops check geom
types before calling. If either geom is Hfield, they call a dedicated
`collide_hfield_multi` that returns `Vec<Contact>`. Otherwise they call
`collide_geoms` as before. This preserves the existing `Option<Contact>`
interface for all non-hfield pairs (zero behavioral change) and isolates
the multi-contact logic to hfield-specific code.

**Rationale:** Option A imposes SmallVec overhead on every collision pair
(sphere-sphere, box-box, etc.) for a feature only hfield needs. Option B
breaks the functional return-value pattern. Option C duplicates the full
dispatch table. Option D adds one type check per pair in the loop (cheap)
and keeps the hfield multi-contact logic self-contained.

**Downstream impact:** The constraint solver already handles multiple contacts
per geom pair — each `Contact` in `data.contacts` is treated independently
regardless of which geom pair produced it. No solver changes are needed.
Multi-contact hfield pairs simply contribute more entries to the flat contact
list, same as if multiple single-contact pairs had been generated.

### AD-2: Prism coordinate frame

**Problem:** Prism vertices can be constructed in hfield-local center-origin
coordinates (matching MuJoCo's grid math) or in world-space coordinates.

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | Hfield-local prisms | Matches MuJoCo's algorithm directly. Pass hfield world Pose as prism pose to GJK/EPA | Need to compute geom2's relative pose in hfield frame |
| 2 | World-space prisms | No relative pose computation | Every prism vertex must be world-transformed. Differs from MuJoCo pattern |

**Chosen:** Option 1 — Construct prism vertices in hfield-local
center-origin coordinates. Pass the hfield's world `Pose` as the prism's
pose to `gjk_epa_contact`. This matches MuJoCo's pattern exactly (collision
computed in hfield frame, contacts transformed to world at the end).

---

## Specification

### S1. Dispatch ordering fix in `narrow.rs`

**File:** `core/src/collision/narrow.rs` lines 86-94
**MuJoCo equivalent:** `mjCOLLISIONFUNC` table — hfield "owns" hfield-convex pairs
**Design decision:** Move the hfield check before the mesh check. MuJoCo's
collision table treats hfield as the owner of all hfield-convex pairs
(including hfield-mesh). The mesh handler's `unreachable!` for Hfield
becomes dead code (we can remove it for cleanliness, but the key fix is
that hfield-mesh pairs never reach it).

**Before** (current `narrow.rs:86-94`):
```rust
// Special case: mesh collision (has its own BVH-accelerated path)
if type1 == GeomType::Mesh || type2 == GeomType::Mesh {
    return collide_with_mesh(model, geom1, geom2, pos1, mat1, pos2, mat2, margin);
}

// Special case: height field collision
if type1 == GeomType::Hfield || type2 == GeomType::Hfield {
    return collide_with_hfield(model, geom1, geom2, pos1, mat1, pos2, mat2, margin);
}
```

**After:**
```rust
// Special case: height field collision (before mesh — hfield owns
// hfield-mesh pairs per MuJoCo's collision table)
if type1 == GeomType::Hfield || type2 == GeomType::Hfield {
    return collide_with_hfield(model, geom1, geom2, pos1, mat1, pos2, mat2, margin);
}

// Special case: mesh collision (has its own BVH-accelerated path)
if type1 == GeomType::Mesh || type2 == GeomType::Mesh {
    return collide_with_mesh(model, geom1, geom2, pos1, mat1, pos2, mat2, margin);
}
```

Also remove the `unreachable!("handled by collide_with_hfield")` arms in
`mesh_collide.rs` at lines 107 and 140. Replace with a comment noting that
hfield pairs are routed before reaching this function.

### S2. Multi-contact dispatch in broadphase loops

**File:** `core/src/collision/mod.rs` lines 440-499
**MuJoCo equivalent:** `mjc_ConvexHField` returns up to 50 contacts; all
contacts are pushed into `mjData.contact`.
**Design decision:** AD-1 Option D. Add type checks in both mechanism 1
and mechanism 2 loops. For hfield pairs, call a new `collide_hfield_multi`
function that returns `Vec<Contact>`. For non-hfield pairs, call
`collide_geoms` as before.

**After (mechanism 1, ~line 440):**
```rust
// Check if either geom is a heightfield — use multi-contact path.
// EXCLUDE SDF-Hfield pairs: MuJoCo uses mjc_HFieldSDF (not mjc_ConvexHField)
// for that pair, and CortenForge's collide_with_sdf already handles it.
// If we intercepted SDF-Hfield here, collide_hfield_multi would fail to
// construct a CollisionShape for SDF (returns None) → 0 contacts → regression.
let is_hfield_pair = (model.geom_type[geom1] == GeomType::Hfield
    || model.geom_type[geom2] == GeomType::Hfield)
    && model.geom_type[geom1] != GeomType::Sdf
    && model.geom_type[geom2] != GeomType::Sdf;

if is_hfield_pair {
    let contacts = collide_hfield_multi(model, geom1, geom2,
        pos1, mat1, pos2, mat2, margin);
    for contact in contacts {
        data.contacts.push(contact);
        data.ncon += 1;
    }
} else if let Some(contact) =
    collide_geoms(model, geom1, geom2, pos1, mat1, pos2, mat2, margin)
{
    data.contacts.push(contact);
    data.ncon += 1;
}
```

**After (mechanism 2, ~line 486):**
```rust
let is_hfield_pair = (model.geom_type[geom1] == GeomType::Hfield
    || model.geom_type[geom2] == GeomType::Hfield)
    && model.geom_type[geom1] != GeomType::Sdf
    && model.geom_type[geom2] != GeomType::Sdf;

if is_hfield_pair {
    let contacts = collide_hfield_multi(model, geom1, geom2,
        pos1, mat1, pos2, mat2, margin);
    for mut contact in contacts {
        apply_pair_overrides(&mut contact, pair);
        apply_global_override(model, &mut contact, pair.gap);
        if !enabled(model, ENABLE_OVERRIDE) {
            contact.mu = assign_friction(model, &contact.mu);
            contact.friction = contact.mu[0];
        }
        data.contacts.push(contact);
        data.ncon += 1;
    }
} else if let Some(mut contact) =
    collide_geoms(model, geom1, geom2, pos1, mat1, pos2, mat2, margin)
{
    apply_pair_overrides(&mut contact, pair);
    // ... existing override logic ...
}
```

The `collide_hfield_multi` function is defined in `hfield.rs` and
re-exported through `mod.rs`.

### S3. Prism-based `collide_hfield_multi` — core algorithm

**File:** `core/src/collision/hfield.rs` (major rewrite)
**MuJoCo equivalent:** `mjc_ConvexHField()` in `engine_collision_convex.c`
**Design decision:** Replace the existing `collide_with_hfield` single-contact
dispatch with a new `collide_hfield_multi` function implementing the full
6-phase prism algorithm. The old `collide_with_hfield` is removed entirely
(its callers are replaced in S2). The existing per-type point-sampling
functions in `heightfield.rs` are left intact for potential use by other
code (flex-vs-hfield in Spec E), but are no longer called from the
narrowphase dispatch.

```rust
/// Maximum contacts per heightfield-geom pair (MuJoCo `mjMAXCONPAIR`).
const MAX_CONTACTS_PER_PAIR: usize = 50;

/// Prism-based heightfield collision for all convex geom types.
///
/// Implements MuJoCo's `mjc_ConvexHField`: decomposes the heightfield into
/// triangular prisms in the sub-grid overlapping the convex geom, then tests
/// each prism against the geom using GJK/EPA penetration.
///
/// Returns 0..50 contacts.
pub fn collide_hfield_multi(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    margin: f64,
) -> Vec<Contact> {
    let mut contacts = Vec::new();

    // --- Phase 0: Identify hfield and convex geom ---
    let (hf_geom, conv_geom, hf_pos, hf_mat, conv_pos, conv_mat) =
        if model.geom_type[geom1] == GeomType::Hfield {
            (geom1, geom2, pos1, mat1, pos2, mat2)
        } else {
            (geom2, geom1, pos2, mat2, pos1, mat1)
        };

    let hfield_id = match model.geom_hfield[hf_geom] {
        Some(id) => id,
        None => return contacts,
    };
    let hfield = &model.hfield_data[hfield_id];
    let hf_size = &model.hfield_size[hfield_id];

    // Build convex collision shape
    let conv_type = model.geom_type[conv_geom];
    let conv_size = model.geom_size[conv_geom];
    let conv_shape = match conv_type {
        GeomType::Mesh => {
            // Use mesh's convex hull (from Spec A)
            let mesh_id = match model.geom_mesh[conv_geom] {
                Some(id) => id,
                None => return contacts,
            };
            let hull = match model.mesh_data[mesh_id].convex_hull() {
                Some(h) => h,
                None => return contacts, // No hull = no collision (conformant)
            };
            CollisionShape::convex_mesh_from_hull(hull)
        }
        _ => match geom_to_collision_shape(conv_type, conv_size) {
            Some(s) => s,
            None => return contacts,
        },
    };

    // Hfield pose: world position WITHOUT centering offset — the offset is
    // baked into prism vertex positions via `dx * c - hf_x_half`.
    let hf_quat = UnitQuaternion::from_matrix(&hf_mat);
    let hf_pose = Pose::from_position_rotation(
        Point3::from(hf_pos),
        hf_quat,
    );

    // Convex geom pose
    let conv_quat = UnitQuaternion::from_matrix(&conv_mat);
    let conv_pose = Pose::from_position_rotation(
        Point3::from(conv_pos),
        conv_quat,
    );

    // --- Phase 1: Express convex geom in hfield-local frame ---
    // MuJoCo center-origin: X in [-size[0], +size[0]], Y in [-size[1], +size[1]]
    let local_pos = hf_mat.transpose() * (conv_pos - hf_pos);
    let local_mat = hf_mat.transpose() * conv_mat;

    // --- Phase 2: Bounding sphere early exit ---
    let rbound = model.geom_rbound[conv_geom] + margin;
    let hf_x_half = hf_size[0];
    let hf_y_half = hf_size[1];
    let hf_z_top = hfield.max_height(); // Pre-scaled — do NOT use hf_size[2]
    let hf_z_bot = -hf_size[3];

    if local_pos.x + rbound < -hf_x_half || local_pos.x - rbound > hf_x_half
        || local_pos.y + rbound < -hf_y_half || local_pos.y - rbound > hf_y_half
        || local_pos.z + rbound < hf_z_bot || local_pos.z - rbound > hf_z_top
    {
        return contacts;
    }

    // --- Phase 3: AABB via support queries ---
    // Query support function along ±X, ±Y, ±Z in hfield-local frame
    let local_quat = UnitQuaternion::from_matrix(&local_mat);
    let local_pose = Pose::from_position_rotation(
        Point3::from(local_pos),
        local_quat,
    );

    let (aabb_min, aabb_max) = compute_local_aabb(&conv_shape, &local_pose);

    // Test AABB against hfield extents
    if aabb_max.x < -hf_x_half || aabb_min.x > hf_x_half
        || aabb_max.y < -hf_y_half || aabb_min.y > hf_y_half
        || aabb_max.z < hf_z_bot || aabb_min.z > hf_z_top
    {
        return contacts;
    }

    // --- Phase 4: Sub-grid clipping ---
    let ncol = hfield.width();
    let nrow = hfield.depth();
    let dx = 2.0 * hf_x_half / (ncol - 1) as f64;
    let dy = 2.0 * hf_y_half / (nrow - 1) as f64;

    let cmin = ((aabb_min.x + hf_x_half) / dx).floor().max(0.0) as usize;
    let cmax = ((aabb_max.x + hf_x_half) / dx).ceil().min((ncol - 1) as f64) as usize;
    let rmin = ((aabb_min.y + hf_y_half) / dy).floor().max(0.0) as usize;
    let rmax = ((aabb_max.y + hf_y_half) / dy).ceil().min((nrow - 1) as f64) as usize;

    let zmin = aabb_min.z;

    // --- Phase 5 & 6: Prism iteration and collision ---
    for r in rmin..rmax {
        for c in cmin..cmax {
            // Two triangles per cell (i=0 and i=1)
            for i in 0..2u8 {
                let prism_verts = build_prism(
                    hfield, hf_size, r, c, i, dx, dy, margin,
                );

                // Skip if all 3 top vertices are below zmin
                if prism_verts[3].z < zmin
                    && prism_verts[4].z < zmin
                    && prism_verts[5].z < zmin
                {
                    continue;
                }

                // Build prism as ConvexMesh (6 vertices, no HullGraph needed)
                let prism_shape = CollisionShape::convex_mesh(
                    prism_verts.iter().map(|v| Point3::from(*v)).collect(),
                );

                // Test prism vs convex geom using GJK/EPA
                // Prism is in hfield-local coords → use hfield world pose
                // Convex geom uses its own world pose
                if let Some(result) = gjk_epa_contact(
                    &prism_shape, &hf_pose, &conv_shape, &conv_pose,
                ) {
                    // Contact normal: GJK/EPA returns normal from shape_a
                    // (prism/hfield) toward shape_b (convex geom).
                    // When hfield is geom1, normal points from geom1 to geom2
                    // (correct convention). When hfield is geom2, negate.
                    let normal = if hf_geom == geom1 {
                        result.normal
                    } else {
                        -result.normal
                    };

                    contacts.push(make_contact_from_geoms(
                        model,
                        result.point.coords,
                        normal,
                        result.penetration,
                        geom1,
                        geom2,
                        margin,
                    ));

                    if contacts.len() >= MAX_CONTACTS_PER_PAIR {
                        return contacts;
                    }
                }
            }
        }
    }

    contacts
}
```

### S3a. `build_prism` helper

**File:** `core/src/collision/hfield.rs` (new function)
**MuJoCo equivalent:** `addPrismVert()` in `engine_collision_convex.c`
**Design decision:** Instead of using MuJoCo's sliding-window approach
(which is a C optimization for reusing vertices across iterations), we
construct each prism's 6 vertices directly. This is simpler in Rust and
the cost of constructing 6 vertices per prism is negligible. The
mathematical result is identical.

```rust
/// Build a triangular prism for one triangle of a heightfield grid cell.
///
/// Returns 6 vertices: [bottom0, bottom1, bottom2, top0, top1, top2].
/// Bottom vertices are at Z = -size[3] (base elevation).
/// Top vertices are at Z = height + margin (surface elevation).
///
/// `i=0`: lower-left triangle (vertices at (c,r), (c+1,r), (c,r+1))
/// `i=1`: upper-right triangle (vertices at (c+1,r), (c+1,r+1), (c,r+1))
fn build_prism(
    hfield: &HeightFieldData,
    hf_size: &[f64; 4],
    r: usize,
    c: usize,
    i: u8,
    dx: f64,
    dy: f64,
    margin: f64,
) -> [Vector3<f64>; 6] {
    let base_z = -hf_size[3];
    let hf_x_half = hf_size[0];
    let hf_y_half = hf_size[1];

    // Grid vertex positions in MuJoCo center-origin coords
    let (v0, v1, v2) = if i == 0 {
        // Lower-left triangle: (c,r), (c+1,r), (c,r+1)
        (
            (c, r),
            (c + 1, r),
            (c, r + 1),
        )
    } else {
        // Upper-right triangle: (c+1,r), (c+1,r+1), (c,r+1)
        (
            (c + 1, r),
            (c + 1, r + 1),
            (c, r + 1),
        )
    };

    let x0 = dx * v0.0 as f64 - hf_x_half;
    let y0 = dy * v0.1 as f64 - hf_y_half;
    let x1 = dx * v1.0 as f64 - hf_x_half;
    let y1 = dy * v1.1 as f64 - hf_y_half;
    let x2 = dx * v2.0 as f64 - hf_x_half;
    let y2 = dy * v2.1 as f64 - hf_y_half;

    // Heights from HeightFieldData — already pre-scaled by size[2]
    // MuJoCo (c, r) maps to CortenForge (x=c, y=r) via heights[y*width+x]
    let h0 = hfield.get(v0.0, v0.1).unwrap_or(0.0) + margin;
    let h1 = hfield.get(v1.0, v1.1).unwrap_or(0.0) + margin;
    let h2 = hfield.get(v2.0, v2.1).unwrap_or(0.0) + margin;

    [
        // Bottom layer (at base elevation)
        Vector3::new(x0, y0, base_z),
        Vector3::new(x1, y1, base_z),
        Vector3::new(x2, y2, base_z),
        // Top layer (at surface + margin)
        Vector3::new(x0, y0, h0),
        Vector3::new(x1, y1, h1),
        Vector3::new(x2, y2, h2),
    ]
}
```

### S3b. `compute_local_aabb` helper

**File:** `core/src/collision/hfield.rs` (new function)
**MuJoCo equivalent:** Support queries along ±X/±Y/±Z in `mjc_ConvexHField`
**Design decision:** Use the GJK support function to compute a tight AABB
of the convex shape in hfield-local coordinates. The `support()` function
in `gjk_epa.rs` operates in world space, but since we've already computed
the convex geom's pose in hfield-local frame (`local_pose`), we can query
support in local-frame directions directly.

```rust
/// Compute tight AABB of a collision shape at a given pose.
///
/// Queries the GJK support function along ±X, ±Y, ±Z to get exact
/// extremes of the convex shape.
fn compute_local_aabb(
    shape: &CollisionShape,
    pose: &Pose,
) -> (Vector3<f64>, Vector3<f64>) {
    use crate::gjk_epa::support;

    let dirs = [
        Vector3::x(), -Vector3::x(),
        Vector3::y(), -Vector3::y(),
        Vector3::z(), -Vector3::z(),
    ];

    let mut min = Vector3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max = Vector3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);

    for dir in &dirs {
        let pt = support(shape, pose, dir);
        min.x = min.x.min(pt.x);
        min.y = min.y.min(pt.y);
        min.z = min.z.min(pt.z);
        max.x = max.x.max(pt.x);
        max.y = max.y.max(pt.y);
        max.z = max.z.max(pt.z);
    }

    (min, max)
}
```

Note: The `support()` function in `gjk_epa.rs` takes a `CollisionShape`,
a `Pose`, and a direction. It returns a `Point3<f64>` in the **same frame
as the pose maps to** — i.e., the direction and output are in the pose's
"target" frame. When `pose` is `local_pose` (mapping shape-local →
hfield-local), the directions are hfield-local axes and the returned points
are in hfield-local coordinates — exactly what we need for the AABB
comparison against hfield extents. (In contrast, when `support()` is called
with a world-frame pose during `gjk_epa_contact`, it returns world-space
points.)

### S4. Remove old single-contact `collide_with_hfield`

**File:** `core/src/collision/hfield.rs` + `core/src/collision/narrow.rs`
**MuJoCo equivalent:** N/A — replacing non-conformant implementation
**Design decision:** The old `collide_with_hfield` function is removed
from `hfield.rs`. In `narrow.rs`, the hfield check within `collide_geoms`
is kept (moved before mesh per S1) but now returns `None` as a safety
fallback. Hfield pairs are intercepted in the broadphase loops (S2)
before `collide_geoms` is called, so this branch should never execute in
normal flow. Keeping it prevents the `unreachable!` in `mesh_collide.rs`
from being reachable if someone calls `collide_geoms` directly.

The `use super::hfield::collide_with_hfield` import in `narrow.rs` is
replaced with the `None` return path. The `collide_hfield_multi` function
is only called from the broadphase loops in `mod.rs`.

**After (in `collide_geoms`, before the mesh check):**
```rust
// Height field collision handled via collide_hfield_multi at the
// broadphase loop level (multi-contact). Return None here as a safety
// fallback — hfield pairs should never reach collide_geoms in normal flow.
if type1 == GeomType::Hfield || type2 == GeomType::Hfield {
    return None;
}
```

---

## Acceptance Criteria

### AC1: Hfield-sphere contact via prism approach *(runtime test)*
**Given:** 5×5 heightfield (flat at z=0.5), sphere (radius=0.3) at position
(0, 0, 0.7) — sphere bottom at z=0.4, below surface at z=0.5.
**After:** `collide_hfield_multi()`
**Assert:** At least 1 contact returned. Contact depth ≈ 0.1 (± 0.05).
Contact normal approximately +Z.
**Field:** Returned `Vec<Contact>` — count, depth, normal.

### AC2: Hfield-mesh contact (previously missing pair) *(runtime test)*
**Given:** 5×5 heightfield (flat at z=0.0), cube mesh (1×1×1) with convex
hull, positioned at (0, 0, 0.4) — bottom face at z=-0.1, penetrating
surface.
**After:** `collide_hfield_multi()`
**Assert:** At least 1 contact returned with positive penetration depth.
No panic (previously `unreachable!`).
**Field:** Returned `Vec<Contact>` — non-empty.

### AC3: Hfield-cylinder exact (no capsule approximation) *(runtime test)*
**Given:** 5×5 heightfield (flat at z=0.0), cylinder (radius=0.5,
half_length=0.5) lying on its side (axis along X, rotated 90°),
positioned so its curved surface penetrates the terrain.
**After:** `collide_hfield_multi()`
**Assert:** Contacts generated using exact cylinder shape (not capsule
approximation). At least 1 contact with reasonable depth. Compare: old
code used capsule approximation which would produce different contact
geometry for a horizontal cylinder.
**Field:** Returned `Vec<Contact>`.

### AC4: Hfield-ellipsoid exact (no sphere approximation) *(runtime test)*
**Given:** 5×5 heightfield (flat at z=0.0), ellipsoid (radii 0.5, 0.3, 0.2)
positioned so the z=0.2 bottom penetrates terrain.
**After:** `collide_hfield_multi()`
**Assert:** At least 1 contact. Contact depth uses true ellipsoid geometry,
not sphere with max_r=0.5.
**Field:** Returned `Vec<Contact>`.

### AC5: Multi-contact generation *(runtime test)*
**Given:** 10×10 heightfield (sinusoidal terrain with 0.5 amplitude), large
box (2×2×0.5) positioned to span multiple grid cells and penetrate terrain
at multiple points.
**After:** `collide_hfield_multi()`
**Assert:** At least 4 contacts returned (the 2×2 box spans ~4 cells on the
10×10 grid; each cell has 2 prisms; penetrating terrain at multiple peaks
guarantees multiple prism hits). Single-contact behavior was the old code's
limitation.
**Field:** `contacts.len() >= 4`.

### AC6: `mjMAXCONPAIR` = 50 limit *(runtime test)*
**Given:** Large heightfield (50×50), large box spanning the entire grid,
positioned to penetrate every cell.
**After:** `collide_hfield_multi()`
**Assert:** At most 50 contacts returned.
**Field:** `contacts.len() <= 50`.

### AC7: No contacts when geom outside hfield bounds *(runtime test)*
**Given:** 5×5 heightfield, sphere positioned far outside bounds (x=100).
**After:** `collide_hfield_multi()`
**Assert:** 0 contacts returned (bounding sphere/AABB early exit).
**Field:** `contacts.is_empty()`.

### AC8: Hfield-mesh with no convex hull *(runtime test)*
**Given:** 5×5 heightfield, mesh geom whose `TriangleMeshData` has
`convex_hull() == None`.
**After:** `collide_hfield_multi()`
**Assert:** 0 contacts returned (no hull = no collision, conformant).
**Field:** `contacts.is_empty()`.

### AC9: Dispatch ordering safety *(runtime test)*
**Given:** Model with a mesh geom and an hfield geom, broadphase generates
a mesh-hfield pair.
**After:** Running collision detection
**Assert:** No panic. The pair is routed to hfield handler, not mesh handler.
**Field:** No `unreachable!` triggered.

### AC10: Mechanism-2 hfield pair overrides *(runtime test)*
**Given:** Model with `<contact><pair>` specifying a hfield-sphere pair with
custom solref/solimp.
**After:** Running full `mj_collision()`
**Assert:** Contacts have the overridden solver parameters, not the default
geom-combined parameters. Each contact in the multi-contact set has the
override applied.
**Field:** `contact.solref`, `contact.solimp` match pair values.

### AC11: Hfield-plane returns no contacts *(runtime test)*
**Given:** Model with a plane geom and an hfield geom.
**After:** `collide_hfield_multi()` or `collide_geoms()`
**Assert:** 0 contacts (conformant — MuJoCo does not support this pair).
**Field:** Empty result.

### AC12: SDF-Hfield routing preserved *(runtime test)*
**Given:** Model with an SDF geom and an hfield geom, broadphase generates
the pair.
**After:** Running full `mj_collision()`
**Assert:** The SDF-Hfield pair is handled by `collide_with_sdf` (via the
SDF check in `collide_geoms`), NOT by `collide_hfield_multi`. Contacts
are generated if the SDF and hfield geometries overlap. The `is_hfield_pair`
check in the broadphase loop excludes SDF-Hfield pairs.
**Field:** Contacts generated (non-zero if overlapping); no routing to
`collide_hfield_multi`.

### AC13: No regression in existing tests *(runtime test)*
**Given:** Full sim domain test suite.
**After:** All changes applied.
**Assert:** All 2,489 existing tests pass.
**Field:** `cargo test` output.

### AC14: Dispatch and pipeline structure *(code review)*
- `narrow.rs`: hfield check occurs before mesh check; hfield branch returns `None`
- `mesh_collide.rs`: no `unreachable!("handled by collide_with_hfield")` arms
- `mod.rs` mechanism 1: `is_hfield_pair` check excludes SDF; hfield pairs go
  through `collide_hfield_multi`
- `mod.rs` mechanism 2: same `is_hfield_pair` check; `apply_pair_overrides`
  applied to each contact in the multi-contact set
- `hfield.rs`: `collide_with_hfield` removed; `collide_hfield_multi` present

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (hfield-sphere prism) | T1 | Direct |
| AC2 (hfield-mesh) | T2 | Direct |
| AC3 (hfield-cylinder exact) | T3 | Direct |
| AC4 (hfield-ellipsoid exact) | T4 | Direct |
| AC5 (multi-contact) | T5 | Direct |
| AC6 (MAX_CONTACTS limit) | T6 | Direct |
| AC7 (outside bounds) | T7 | Direct + edge case |
| AC8 (mesh no hull) | T8 | Edge case |
| AC9 (dispatch safety) | T9 | Integration |
| AC10 (mechanism-2 overrides) | T10 | Integration |
| AC11 (hfield-plane no contacts) | T11 | Negative |
| AC12 (SDF-hfield routing) | T13 | Integration |
| AC13 (no regression) | T12 | Regression |
| AC14 (code review) | — | Code review (manual) |

---

## Test Plan

### T1: Hfield-sphere prism contact → AC1

Model: 5×5 flat heightfield at z=0.5 (cell_size=1.0, pre-scaled heights).
Sphere radius=0.3 at position (0, 0, 0.7).

Expected: At least 1 contact with depth ≈ 0.1 (tolerance ±0.05). Contact
normal should be approximately +Z (the flat terrain normal). This test
verifies the prism approach generates contacts for the basic sphere case.

### T2: Hfield-mesh contact (new pair) → AC2

Model: 5×5 flat heightfield at z=0.0. Unit cube mesh (1×1×1) with computed
convex hull, positioned at (0, 0, 0.4) so bottom face (z=-0.1) penetrates
the surface.

Expected: At least 1 contact with positive depth. This test verifies the
previously-missing hfield-mesh pair works. Must not panic.

### T3: Hfield-cylinder exact → AC3

Model: 5×5 flat heightfield at z=0.0. Cylinder (radius=0.5, half_length=1.0)
rotated to lie on its side (axis along X), positioned at z=0.4 so the
curved surface penetrates terrain.

Expected: At least 1 contact. The contact uses the exact cylinder shape
(CollisionShape::Cylinder), not a capsule approximation. Verify by checking
that the collision shape constructed is Cylinder, not Capsule.

### T4: Hfield-ellipsoid exact → AC4

Model: 5×5 flat heightfield at z=0.0. Ellipsoid (radii: 0.5, 0.3, 0.2)
at position (0, 0, 0.15) so bottom (z=-0.05) penetrates.

Expected: At least 1 contact. Depth uses true ellipsoid shape.

### T5: Multi-contact generation → AC5

Model: 10×10 heightfield with sinusoidal terrain (amplitude=0.5). Large box
(2×2×0.5) positioned to span many cells and penetrate at multiple peaks.

Expected: At least 4 contacts returned from a single hfield pair call (box spans
~4 cells; each has 2 prisms; sinusoidal peaks ensure multiple penetrations).

### T6: MAX_CONTACTS_PER_PAIR limit → AC6

Model: 50×50 heightfield. Very large box spanning the entire grid, positioned
to penetrate deeply across all cells.

Expected: Exactly 50 contacts returned (the `MAX_CONTACTS_PER_PAIR` limit).

### T7: Geom outside hfield bounds → AC7

Model: 5×5 heightfield. Sphere at position (100, 100, 0).

Expected: 0 contacts. Bounding sphere early exit fires.

### T8: Mesh with no convex hull → AC8

Model: 5×5 heightfield. Mesh geom with `TriangleMeshData` that has no convex
hull (`convex_hull() == None`).

Expected: 0 contacts returned. No panic.

### T9: Dispatch ordering — mesh-hfield pair → AC9

Integration test: Build a Model with both a mesh geom and an hfield geom.
Run full collision detection (`mj_collision`).

Expected: No panic (the `unreachable!` in `mesh_collide.rs` is never reached).
The pair is handled by the hfield multi-contact path.

### T10: Mechanism-2 pair overrides for hfield → AC10

Integration test: Model with `<contact><pair>` entry for a hfield-sphere pair
with custom `solref=[0.01, 0.5]` and `solimp=[0.95, 0.99, 0.001]`.

Expected: All contacts from the hfield pair have the custom solref/solimp
values, not the default geom-combined values.

### T11: Hfield-plane negative case → AC11

Model with a plane geom and an hfield geom (if broadphase generates this pair).

Expected: 0 contacts. Either broadphase doesn't pair them, or
`collide_hfield_multi` returns empty (convex shape construction for Plane
returns `None`).

### T12: Full regression suite → AC13

Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests`. All
2,489+ existing tests pass.

### T13: SDF-Hfield routing preserved → AC12

Integration test: Model with an SDF geom overlapping an hfield geom.

Expected: `collide_with_sdf` handles the pair (via `collide_geoms`'s SDF
check). The `is_hfield_pair` guard in the broadphase loop excludes this
pair (because one geom is SDF). Contacts are generated if geometries
overlap. This verifies that S2's hfield interception does not break the
existing SDF-Hfield path.

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| Flat hfield (all same height) | Simplest case — verifies basic prism construction | T1, T2 | AC1, AC2 |
| Single-cell hfield (2×2 grid) | Minimum valid hfield — 1 cell = 2 prisms | TS1 | — |
| Geom entirely outside bounds | Tests bounding sphere and AABB early exits | T7 | AC7 |
| Geom straddling hfield edge | Partial sub-grid — only some cells overlap | TS2 | — |
| Geom smaller than one cell | Only 1-2 prisms tested | TS3 | — |
| Geom larger than entire hfield | Full grid iterated, hits MAX_CONTACTS limit | T6 | AC6 |
| Mesh with no convex hull | Edge case for hfield-mesh | T8 | AC8 |
| Tilted hfield (non-identity pose) | Tests hfield-local frame transform correctness | TS4 | — |
| Degenerate prism (coplanar top verts) | All 3 heights same — prism is a flat slab | TS5 | — |
| Hfield-plane (negative case) | Not in MuJoCo — must return 0 contacts | T11 | AC11 |
| Hfield-hfield (negative case) | Not in MuJoCo — must not be attempted | TS6 | — |
| SDF-Hfield (routing preservation) | MuJoCo uses `mjc_HFieldSDF` (not `mjc_ConvexHField`). S2's `is_hfield_pair` must exclude SDF to preserve existing `collide_with_sdf` routing | T13 | AC12 |
| Contact limit (50) | Hard cap per MuJoCo | T6 | AC6 |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| TS1 (single-cell hfield) | 2×2 grid with sphere contact | Minimum valid hfield — ensures algorithm works at boundary |
| TS2 (edge-straddling geom) | Sphere at hfield boundary | Sub-grid clipping correctness |
| TS3 (small geom, one cell) | Tiny sphere in single cell | Verifies sub-grid converges to 1-2 prisms |
| TS4 (tilted hfield) | Rotated hfield with sphere contact | Frame transform correctness (hfield-local → world) |
| TS5 (flat prism) | All vertices at same height | Degenerate prism handling in GJK |
| TS6 (hfield-hfield no crash) | Two hfield geoms in model | Verifies graceful return, no dispatch errors |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| Hfield-sphere contacts | Single contact via point-sampling | 0-50 contacts via prism GJK/EPA | Toward MuJoCo | Models with sphere-on-hfield | Contact positions/normals/depths may differ. Multi-contact is more physically accurate. |
| Hfield-capsule contacts | Single contact via point-sampling | 0-50 contacts via prism GJK/EPA | Toward MuJoCo | Models with capsule-on-hfield | Same as above |
| Hfield-box contacts | Single contact via point-sampling | 0-50 contacts via prism GJK/EPA | Toward MuJoCo | Models with box-on-hfield | Same as above |
| Hfield-cylinder contacts | Approximated as capsule, single contact | Exact cylinder via prism GJK/EPA, multi-contact | Toward MuJoCo | Models with cylinder-on-hfield | Contacts now use true cylinder geometry |
| Hfield-ellipsoid contacts | Approximated as sphere(max_r), single contact | Exact ellipsoid via prism GJK/EPA, multi-contact | Toward MuJoCo | Models with ellipsoid-on-hfield | Contacts now use true ellipsoid geometry |
| Hfield-mesh contacts | `unreachable!` panic | 0-50 contacts via prism GJK/EPA | Toward MuJoCo (new pair) | Models with mesh-on-hfield | Previously crashed; now works |
| Dispatch ordering | Mesh check before hfield in `collide_geoms` | Hfield check before mesh | Toward MuJoCo | Internal dispatch only | Transparent — same behavior, no panic risk |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `core/src/collision/hfield.rs` | Major rewrite: replace `collide_with_hfield` with `collide_hfield_multi`, `build_prism`, `compute_local_aabb` | ~+200 / ~-80 |
| `core/src/collision/narrow.rs` | Swap hfield/mesh check order; remove hfield call (return None); remove import | ~10 modified |
| `core/src/collision/mod.rs` | Add hfield multi-contact path in mechanism 1 and mechanism 2 loops; import changes | ~+30 / ~5 modified |
| `core/src/collision/mesh_collide.rs` | Remove `unreachable!("handled by collide_with_hfield")` at lines 107, 140 | ~-2 |
| `core/src/heightfield.rs` tests | New tests for prism-based collision | ~+300 |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| `test_heightfield_flat` | `heightfield.rs:748` | Pass (unchanged) | Tests HeightFieldData construction, not collision dispatch |
| `test_heightfield_from_fn` | `heightfield.rs:763` | Pass (unchanged) | Tests HeightFieldData construction |
| `test_heightfield_bilinear_interpolation` | `heightfield.rs:774` | Pass (unchanged) | Tests sample() |
| `test_heightfield_normal` | `heightfield.rs:797` | Pass (unchanged) | Tests normal() |
| `test_heightfield_sphere_contact` | `heightfield.rs:814` | Pass (unchanged) | Tests `heightfield_sphere_contact` directly — this function is preserved (used by flex-vs-hfield in Spec E) |
| `test_heightfield_sphere_contact_on_slope` | `heightfield.rs:838` | Pass (unchanged) | Same — tests function directly |
| `test_heightfield_capsule_contact` | `heightfield.rs:855` | Pass (unchanged) | Tests `heightfield_capsule_contact` directly |
| `test_heightfield_box_contact` | `heightfield.rs:875` | Pass (unchanged) | Tests `heightfield_box_contact` directly |
| `test_heightfield_cells_in_aabb` | `heightfield.rs:894` | Pass (unchanged) | Tests grid utilities |
| `test_heightfield_aabb` | `heightfield.rs:911` | Pass (unchanged) | Tests AABB computation |
| `test_heightfield_with_pose` | `heightfield.rs:924` | Pass (unchanged) | Tests sphere contact with pose |
| Integration tests using hfield | `sim-conformance-tests` | May see different contact counts/values | Multi-contact replaces single-contact. Expected values may need updating. |

### Non-Modification Sites

| File:line | What it does | Why NOT modified |
|-----------|-------------|-----------------|
| `heightfield.rs:457` (`heightfield_sphere_contact`) | Point-sampling sphere-hfield contact | Preserved — still used by flex-vs-hfield (Spec E), and by direct tests. Not called from narrowphase dispatch anymore. |
| `heightfield.rs` (capsule, box contacts) | Point-sampling contact functions | Preserved — same rationale |
| `collision/sdf_collide.rs` | SDF collision dispatch | SDF check fires before hfield in `collide_geoms` — no change needed |
| `collision/flex_collide.rs` | Flex collision dispatch | Uses `heightfield_sphere_contact` directly (not through `collide_geoms`) — no change needed |
| `gjk_epa.rs` | GJK/EPA algorithm | Read-only consumer. `support()` and `gjk_epa_contact()` used by prism collision. No modifications. |

---

## Execution Order

1. **S1** (dispatch ordering fix in `narrow.rs` + remove `unreachable!` in
   `mesh_collide.rs`) → verify no test regressions. This is a prerequisite:
   without the fix, adding hfield-mesh tests would be blocked by the
   existing `unreachable!`.

2. **S3, S3a, S3b** (`collide_hfield_multi`, `build_prism`,
   `compute_local_aabb` in `hfield.rs`) → verify prism construction and
   GJK/EPA contact generation with unit tests for basic cases (flat hfield
   + sphere).

3. **S2** (multi-contact dispatch in `mod.rs` broadphase loops) → verify
   hfield pairs route through `collide_hfield_multi` in both mechanism 1
   and mechanism 2.

4. **S4** (remove old `collide_with_hfield`, update `collide_geoms` hfield
   branch) → verify all tests pass with the new pipeline.

5. **Full test suite** → verify AC13 (no regression), all new tests pass.

---

## Performance Characterization

**Cost:** For each hfield-convex pair, the algorithm iterates over a sub-grid
of heightfield cells. Each cell produces 2 prisms, each tested via
GJK/EPA (O(1) for 6-vertex prisms). Total: O(k) where k = number of
cells in the sub-grid (typically 4-20 for normal geom sizes).

**Worst case:** A very large box spanning a 50×50 heightfield: 2500 cells ×
2 prisms × GJK/EPA (~50μs each) ≈ 250ms. This hits the 50-contact limit
early, so actual cost is bounded by `MAX_CONTACTS_PER_PAIR * gjk_cost`.

**Acceptable?** Yes. Hfield collision is already an expensive operation.
The prism approach matches MuJoCo's exact algorithm and cost profile.
For typical use cases (robot on terrain), k ≈ 4-20 cells, well within
real-time budgets.

---

## Out of Scope

- **Hfield-plane collision** — MuJoCo does not support this pair
  (`mjCOLLISIONFUNC[HFIELD][PLANE] = 0`). Implementing it would be a
  CortenForge extension. *Conformance impact: none.*

- **Hfield-hfield collision** — MuJoCo does not support this pair.
  *Conformance impact: none.*

- **Hfield-SDF collision** — Handled by existing `collide_with_sdf` dispatch
  path. The SDF check fires before hfield in `collide_geoms`.
  *Conformance impact: none — already handled.*

- **Optimized prism BVH** — The prism approach iterates cells linearly.
  An acceleration structure (quadtree) could speed up large hfield queries.
  Deferred to post-v1.0. *Conformance impact: none — optimization only.*

- **Flex-vs-hfield via prism approach** — Spec E handles flex-vs-hfield
  using the existing point-sampling `heightfield_sphere_contact` (each flex
  vertex is a sphere). Migrating flex-vs-hfield to the prism approach is
  out of scope. *Conformance impact: minimal — flex vertices are small
  spheres, point-sampling is adequate.*
