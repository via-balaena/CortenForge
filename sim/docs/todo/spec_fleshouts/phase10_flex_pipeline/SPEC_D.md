# Spec D — Flex-Flex Cross-Object Collision (§42A-v / DT-143): Spec

**Status:** Draft
**Phase:** Roadmap Phase 10 — Flex Pipeline
**Effort:** M
**MuJoCo ref:** `canCollide2()` in `engine_collision_driver.c` (broadphase
pair enumeration + `filterBitmask()` bitmask filtering); `mj_contactParam()`
in `engine_collision_driver.c` (contact parameter combination for flex-flex
path); element-element narrowphase dispatch (reuses Spec C primitives)
**MuJoCo version:** 3.5.0
**Test baseline:** 2,100+ sim domain tests (post-Spec C)
**Prerequisites:**
- Spec C (Sessions 14–18, commit `772b8a3`): provides element-element
  narrowphase primitives (`collide_element_pair()`, `collide_triangles()`,
  `collide_tetrahedra()`, `collide_edges()`), `make_contact_flex_self()`,
  `element_aabb()`, `FlexSelfCollide` enum, BVH/SAP midphase, self-collision
  dispatch
- T1 Session 2 (commit `1f0230c`): provides `flex_rigid`, `flexedge_rigid`,
  `flexedge_length`, `flexedge_velocity`
- Spec A (Sessions 3–7): provides `flexedge_J` (not consumed by Spec D —
  independent)
- Spec B (Sessions 8–13): provides cotangent bending (not consumed by Spec D —
  independent)

**Independence:** This spec depends on Spec C (§42A-iv self-collision) for
narrowphase primitives. It is independent of Spec A (§42A-i edge Jacobian)
and Spec B (§42B cotangent bending). Shared file `collision/mod.rs` is
modified by Spec D (flex-flex dispatch) as an additive extension after Spec C's
self-collision dispatch — sequential additions, no conflict. Shared file
`collision/flex_collide.rs` is modified by Spec D (narrowphase generalization +
`mj_collide_flex_flex()`) as an extension of Spec C's primitives.

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical results to MuJoCo for the feature described below.
> Every algorithm, every acceptance criterion, and every test is in service
> of this goal. The MuJoCo C source code is the single source of truth —
> not the MuJoCo documentation, not intuition about "what should happen,"
> not a Rust-idiomatic reinterpretation. When in doubt, read the C source
> and match its behavior exactly.

---

## Scope Adjustment

Verified against MuJoCo 3.5.0 via empirical tests (SPEC_D_RUBRIC.md EGT-1
through EGT-8). The umbrella's Spec D scope statement is accurate with the
following corrections:

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| `flex_rigid` gates flex-flex collision | **Not true.** `flex_rigid` only gates self-collision. MuJoCo produces 32 flex-flex contacts when flex 1 is fully rigid (`flex_rigid[0]=1`). EGT-4 verified. | **Correct**: `flex_rigid` is NOT a gate for flex-flex |
| 3-session compressed cycle | Session plan updated to full 5-session cycle (rubric → spec → implement → review-create → review-execute). | **Adjust**: 5 sessions |
| Reuses Spec C narrowphase primitives directly | Spec C's `collide_element_pair()` takes a single `flex_id`. Must generalize to accept two flex IDs for cross-object margin/gap/contact factory. | **In scope** with narrowphase adaptation |
| Umbrella Convention Registry §4 default: `FlexSelfCollide::None` | MuJoCo default is `Auto`. Already corrected by Spec C. | **N/A** — Spec C already handled |
| Sentinel `geom1/geom2 = usize::MAX` safe in downstream consumers | **Not safe.** Two CRITICAL sites will PANIC: `acceleration.rs:510-511` and `sleep.rs:557-558`. Three MEDIUM sites have wrong logic. Pre-existing latent bugs for flex-self too, newly exposed by flex-flex active constraints. | **In scope** — must fix before flex-flex contacts can produce constraint forces |

**Final scope:**

1. `contact_param_flex_flex()` — contact parameter combination (priority + solmix protocol, both sides `flex_*` arrays)
2. `make_contact_flex_flex()` — contact factory (sentinel geoms, dual flex vertex tracking)
3. Narrowphase generalization — adapt `collide_element_pair()` + underlying functions to accept two flex IDs
4. Cross-object midphase — BVH-based element pair pruning between two flex objects
5. `mj_collide_flex_flex()` — broadphase pair loop with `filterBitmask()` bitmask filter
6. Pipeline integration — dispatch after flex-self in `mj_collision()`
7. Sentinel-safe downstream fixes — fix all unguarded `geom_body[contact.geom*]` sites

---

## Problem Statement

**Conformance gap** — MuJoCo dispatches flex-flex cross-object collision from
`mj_collision()` in `engine_collision_driver.c`. For every pair `(f1, f2)`
where `f1 < f2`, it applies `filterBitmask()` on `flex_contype`/
`flex_conaffinity`, then runs element-element narrowphase between the two flex
objects. Contact parameters are combined via `mj_contactParam()` following the
standard priority + solmix protocol.

CortenForge does not implement flex-flex collision. The collision pipeline
dispatches flex-rigid and flex-self contacts but has no path for contacts
between elements of two different flex objects. Additionally, five downstream
sites that index `model.geom_body[contact.geom1]` are unsafe for flex contacts
that use the `usize::MAX` sentinel — two will panic, three have wrong logic.
These are pre-existing latent bugs for flex-self contacts that will be exposed
by flex-flex contacts producing active constraint forces.

---

## MuJoCo Reference

> **This is the most important section of the spec.** Everything downstream
> is derived from what's documented here.

### Broadphase: `canCollide2()` and `filterBitmask()`

**Source:** `engine_collision_driver.c` → `canCollide2()`.

MuJoCo uses a unified bodyflex index space where bodies occupy `[0, nbody)`
and flexes occupy `[nbody, nbody+nflex)`. The `canCollide2()` function
enumerates all entity pairs `(i, j)` where `i < j` and checks:

```c
// Flex-flex pair: both indices >= nbody
int f1 = i - nbody;
int f2 = j - nbody;
// filterBitmask: (ct1 & ca2) != 0 || (ct2 & ca1) != 0
if (filterBitmask(flex_contype[f1], flex_conaffinity[f1],
                  flex_contype[f2], flex_conaffinity[f2])) {
    // Dispatch element-element narrowphase
}
```

Key behaviors:
- **Pair enumeration:** `f1 < f2` — no duplicate pairs, no self-pairs.
  Three flex objects produce pairs `(0,1)`, `(0,2)`, `(1,2)` (EGT-6).
- **`flex_rigid` is NOT checked.** Only self-collision gates on `flex_rigid`.
  A fully rigid flex (`flex_rigid[f]=true`) still collides with other flex
  objects (EGT-4: 32 contacts when flex 1 all-pinned).
- **`flex_selfcollide` is NOT checked.** Self-collision mode is irrelevant
  for cross-object collision.
- **`flex_internal` is NOT checked.** Internal collision is for same-flex
  adjacent elements only.

### `filterBitmask()` protocol

**Source:** `engine_collision_driver.c` → `filterBitmask()`.

```c
// Returns true if pair can collide
static int filterBitmask(int ct1, int ca1, int ct2, int ca2) {
    return (ct1 & ca2) || (ct2 & ca1);
}
```

For flex-flex: reads `flex_contype[f1]`, `flex_conaffinity[f1]`,
`flex_contype[f2]`, `flex_conaffinity[f2]`.

Empirically verified (EGT-2):
- Incompatible (`ct=1/ca=1` vs `ct=2/ca=2`): `(1&2)=0` and `(2&1)=0` → **0 contacts**
- Compatible (`ct=1/ca=3` vs `ct=2/ca=1`): `(1&1)=1` → **32 contacts**

### Contact parameter combination: `mj_contactParam()`

**Source:** `engine_collision_driver.c` → `mj_contactParam()` flex-flex path.

For flex-flex pairs, `mj_contactParam()` reads `flex_*` arrays for both sides
and applies the standard priority + solmix protocol:

1. **Priority check:** If `flex_priority[f1] != flex_priority[f2]`, the
   higher-priority flex's parameters win entirely (condim, solref, solimp,
   friction). Gap is still additive.

2. **Equal priority — combine:**
   - `condim = max(flex_condim[f1], flex_condim[f2])` (EGT-7: `max(1,6)=6`)
   - `gap = flex_gap[f1] + flex_gap[f2]` (additive)
   - `solmix_weight = flex_solmix[f1] / (flex_solmix[f1] + flex_solmix[f2])`
   - `solref = mix * solref1 + (1-mix) * solref2` (standard ref)
   - `solimp = mix * solimp1 + (1-mix) * solimp2`
   - `friction = element_wise_max(friction1, friction2)`

Empirically verified (EGT-3):
- **Priority test:** flex1 priority=1, flex2 priority=0 → solref=`[0.05, 2.0]`
  (flex1's), friction=`0.5` (flex1's).
- **Solmix test:** flex1 (solmix=0.5, solref=`[0.02,1.0]`, friction=0.8),
  flex2 (solmix=1.5, solref=`[0.04,2.0]`, friction=0.2), equal priority →
  mix=0.25, solref=`[0.035, 1.75]`, friction=`0.8` (max).

### Margin and gap combination

**Source:** `mj_contactParam()` and narrowphase detection threshold.

Margin and gap are additive for flex-flex (same as flex-self):
- `margin = flex_margin[f1] + flex_margin[f2]`
- `gap = flex_gap[f1] + flex_gap[f2]`
- `includemargin = margin - gap`

Empirically verified (EGT-5): flex1 margin=0.05/gap=0.01, flex2
margin=0.03/gap=0.02 → `includemargin = (0.05+0.03) - (0.01+0.02) = 0.05`.

### Element-element narrowphase

**Source:** `engine_collision_driver.c` → flex-flex narrowphase dispatch.

For each overlapping element pair (one from each flex), MuJoCo runs the same
element-element collision test as self-collision:
- dim=2: triangle-triangle intersection (SAT-based), at most 1 contact
- dim=3: vertex-face + edge-edge tests between tetrahedra
- dim=1: edge-edge proximity between cable segments

**Mixed flex dimensions:** When `flex_dim[f1] != flex_dim[f2]`, MuJoCo
dispatches narrowphase based on the effective dimension of the collision pair.
In practice, mixed-dim flex-flex collision (e.g., shell vs solid, cable vs
shell) is rare — most models use the same dim for all flex objects. This spec
handles mixed dim by using `min(dim1, dim2)` for narrowphase dispatch, which
selects the lower-dimensional test (e.g., edge-edge for cable-vs-shell).
This is conservative — it may miss some contacts that a full dim-aware
narrowphase would catch, but it avoids mismatched element topology (a triangle
cannot be tested against a tetrahedron with the tet-tet algorithm).

Contact attribution: contacts are element-level, not per-vertex. MuJoCo
stores `vert=[-1,-1]` (no per-vertex attribution) but CortenForge stores
nearest vertices for Jacobian DOF lookup.

Element pair count (EGT-8): Two 3×3 grids (8 triangles each) → 64 possible
pairs (8×8), 32 intersecting → 32 contacts. Each intersecting pair produces
exactly 1 contact.

### Contact encoding

**Source:** `mj_contactParam()` output fields.

```
contact.geom1 = -1   (sentinel: no rigid geom)
contact.geom2 = -1
contact.flex  = [f1, f2]   (flex indices)
contact.elem  = [e1, e2]   (element indices within each flex)
contact.vert  = [-1, -1]   (no per-vertex attribution)
```

CortenForge mapping:
- `geom1 = geom2 = usize::MAX` (Rust sentinel, same as flex-self)
- `flex_vertex = Some(nearest_vertex_from_flex1)` (for Jacobian DOF lookup)
- `flex_vertex2 = Some(nearest_vertex_from_flex2)` (for Jacobian DOF lookup)
- Flex ID derived from vertex via `model.flexvert_flexid[vertex]`

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| Broadphase pair enumeration | `canCollide2()` enumerates all `(f1,f2)` with `f1<f2` | **Not implemented** — no flex-flex dispatch path |
| Bitmask filtering | `filterBitmask(flex_contype[f1], flex_conaffinity[f1], flex_contype[f2], flex_conaffinity[f2])` | **Not implemented** |
| Contact parameter combination | `mj_contactParam()` with priority + solmix on `flex_*` arrays for both sides | **Not implemented** |
| `flex_rigid` gate | Does NOT gate flex-flex (only gates self-collision) | N/A |
| Element-element narrowphase | Same primitives as self-collision (tri-tri, tet-tet, edge-edge) | Self-collision primitives exist but take single `flex_id` — no cross-object adaptation |
| Margin/gap | Additive: `flex_margin[f1] + flex_margin[f2]`, `flex_gap[f1] + flex_gap[f2]` | **Not implemented** |
| Contact encoding | `geom=-1`, `flex[0]/flex[1]`, `vert=[-1,-1]` | flex-self uses `geom1=geom2=usize::MAX`, `flex_vertex`, `flex_vertex2` — pattern reusable |
| Force distribution | Distributes constraint forces to bodies via `geom_body[]` or flex vertex bodies | **BUG:** `acceleration.rs:510-511` indexes `geom_body[usize::MAX]` — will PANIC |
| Sleep/wake detection | Checks contact bodies for sleep state transitions | **BUG:** `sleep.rs:557-558` indexes `geom_body[usize::MAX]` — will PANIC |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| Sentinel geom | `contact.geom = -1` (C int) | `contact.geom1 = usize::MAX` (Rust usize) | Direct port — same sentinel concept, different integer representation |
| Flex pair IDs | `contact.flex[0]`, `contact.flex[1]` (flex indices) | No separate `flex[]` field. Derive from `flexvert_flexid[flex_vertex]` and `flexvert_flexid[flex_vertex2]` | Derive flex ID from vertex index: `model.flexvert_flexid[contact.flex_vertex.unwrap()]` |
| Element indices | `contact.elem[0]`, `contact.elem[1]` | Not stored on Contact — element identity used only during narrowphase, not needed downstream | No action — Contact struct does not store element indices |
| Vertex attribution | `contact.vert = [-1, -1]` (no vertex attribution) | `flex_vertex = Some(nearest_v1)`, `flex_vertex2 = Some(nearest_v2)` — stores nearest vertices for Jacobian DOF lookup | CortenForge stores vertex IDs (needed for DOF address lookup); MuJoCo doesn't (uses different Jacobian path) |
| Friction unpacking | `Vector3(tangential, torsional, rolling)` → 5-element array | `[f.x, f.x, f.y, f.z, f.z]` — `[tan1, tan2, torsional, roll1, roll2]` | Use same unpacking as `contact_param_flex_rigid()` and `contact_param_flex_self()` |
| Margin/gap | `margin = flex_margin[f1] + flex_margin[f2]` | Same formula | Direct port — no translation needed |
| Gap additive | `gap = flex_gap[f1] + flex_gap[f2]` | Same formula | Direct port |
| Condim combination | `max(condim1, condim2)` | Same formula | Direct port |

---

## Architecture Decisions

### AD-1: Narrowphase adaptation strategy

**Problem:** Spec C's `collide_element_pair()` accepts a single `flex_id`
parameter, used to compute `2*flex_margin[flex_id]` and `2*flex_gap[flex_id]`
and to call `make_contact_flex_self()`. Cross-object collision needs two flex
IDs with different margin/gap sources and a different contact factory.

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | Generalize to accept two flex IDs (`flex_id1`, `flex_id2`) | One code path for both self and cross; `margin = flex_margin[f1] + flex_margin[f2]` naturally produces `2*flex_margin[f]` when `f1==f2`; no code duplication | Requires updating all self-collision call sites to pass `(f, f)` |
| 2 | Create parallel `collide_element_pair_flex_flex()` variant | No changes to existing self-collision code | Duplicates all narrowphase geometry code (triangles, tets, edges) |
| 3 | Extract contact factory as callback parameter | Maximum flexibility | Closure captures conflict with `&mut Data` borrow in Rust |

**Chosen:** Option 1 — generalize to two flex IDs. When `flex_id1 == flex_id2`,
the function produces `make_contact_flex_self()`; when different, it produces
`make_contact_flex_flex()`. Margin becomes `flex_margin[f1] + flex_margin[f2]`
which equals `2*flex_margin[f]` for self-collision. Zero geometry code
duplication. All self-collision call sites updated from `(..., f)` to
`(..., f, f)` — a mechanical change verified by compiler (arity change).

### AD-2: Cross-object midphase strategy

**Problem:** Self-collision uses a single BVH built from all elements of one
flex, then queries each element against the same BVH. Cross-object needs to
find overlapping elements between two different flex objects.

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | Build BVH from flex2, query with each element of flex1 | Simple, asymmetric; O(n1 * log(n2)) | Must choose which flex gets the BVH (pick larger) |
| 2 | Build BVH from both, do bipartite query | Handles both directions | More complex, not needed for correctness |
| 3 | Brute-force (no midphase) | Simplest | O(n1 * n2) — poor for large meshes |

**Chosen:** Option 1 — build BVH from the larger flex (more elements), query
with each element of the smaller flex. This matches MuJoCo's asymmetric
approach. For small element counts where BVH overhead exceeds brute-force,
the brute-force path is trivially fast anyway.

---

## Specification

### S1. `contact_param_flex_flex()` — Contact parameter combination

**File:** `sim/L0/core/src/collision/mod.rs` (after `contact_param_flex_self()`, ~line 249)
**MuJoCo equivalent:** `mj_contactParam()` in `engine_collision_driver.c` (flex-flex path)
**Design decision:** Mirrors `contact_param_flex_rigid()` (lines 177–229) but
reads `flex_*` arrays for both sides instead of `flex_*` + `geom_*`. Same
priority + solmix protocol. The function signature and return type match the
existing `contact_param_*` family.

**After** (new implementation):
```rust
/// Contact parameter combination for flex-flex collision pairs.
///
/// Both sides read `flex_*` arrays. Follows MuJoCo's `mj_contactParam()`
/// with the same priority + solmix protocol as geom-geom and flex-rigid.
/// Gap is additive: `flex_gap[f1] + flex_gap[f2]`.
pub(crate) fn contact_param_flex_flex(
    model: &Model,
    flex_id1: usize,
    flex_id2: usize,
) -> (i32, f64, [f64; 2], [f64; 5], [f64; 5]) {
    let priority1 = model.flex_priority[flex_id1];
    let priority2 = model.flex_priority[flex_id2];
    let gap = model.flex_gap[flex_id1] + model.flex_gap[flex_id2];

    if priority1 > priority2 {
        let f = model.flex_friction[flex_id1];
        return (
            model.flex_condim[flex_id1],
            gap,
            model.flex_solref[flex_id1],
            model.flex_solimp[flex_id1],
            [f.x, f.x, f.y, f.z, f.z],
        );
    }
    if priority2 > priority1 {
        let f = model.flex_friction[flex_id2];
        return (
            model.flex_condim[flex_id2],
            gap,
            model.flex_solref[flex_id2],
            model.flex_solimp[flex_id2],
            [f.x, f.x, f.y, f.z, f.z],
        );
    }

    // Equal priority — combine
    let condim = model.flex_condim[flex_id1].max(model.flex_condim[flex_id2]);

    let s1 = model.flex_solmix[flex_id1];
    let s2 = model.flex_solmix[flex_id2];
    let mix = solmix_weight(s1, s2);

    let solref = combine_solref(
        model.flex_solref[flex_id1],
        model.flex_solref[flex_id2],
        mix,
    );
    let solimp = combine_solimp(
        model.flex_solimp[flex_id1],
        model.flex_solimp[flex_id2],
        mix,
    );

    let f1 = model.flex_friction[flex_id1];
    let f2 = model.flex_friction[flex_id2];
    let fri = [
        f1.x.max(f2.x),
        f1.x.max(f2.x), // tangential1, tangential2
        f1.y.max(f2.y), // torsional
        f1.z.max(f2.z),
        f1.z.max(f2.z), // rolling1, rolling2
    ];

    (condim, gap, solref, solimp, fri)
}
```

### S2. `make_contact_flex_flex()` — Contact factory

**File:** `sim/L0/core/src/collision/flex_collide.rs` (after `make_contact_flex_self()`, ~line 351)
**MuJoCo equivalent:** Contact construction in flex-flex narrowphase dispatch
**Design decision:** Mirrors `make_contact_flex_self()` (lines 310–351) but
calls `contact_param_flex_flex()` instead of `contact_param_flex_self()`.
Uses additive margin from two different flex objects. Same sentinel convention
(`geom1 = geom2 = usize::MAX`), same `flex_vertex`/`flex_vertex2` encoding.

**After** (new implementation):
```rust
/// Create a Contact for a flex-flex cross-object collision.
///
/// Side 1: nearest vertex from flex1 (`flex_vertex`).
/// Side 2: nearest vertex from flex2 (`flex_vertex2`).
/// No rigid geom — `geom1`/`geom2` set to `usize::MAX` (sentinel).
/// Margin = flex_margin[f1] + flex_margin[f2] (additive, not doubled).
pub fn make_contact_flex_flex(
    model: &Model,
    vertex1: usize,
    vertex2: usize,
    pos: Vector3<f64>,
    normal: Vector3<f64>,
    depth: f64,
) -> Contact {
    let flex_id1 = model.flexvert_flexid[vertex1];
    let flex_id2 = model.flexvert_flexid[vertex2];
    let (condim, gap, solref, solimp, mu) =
        contact_param_flex_flex(model, flex_id1, flex_id2);
    let solref = assign_ref(model, &solref);
    let solimp = assign_imp(model, &solimp);
    let mu = assign_friction(model, &mu);
    let margin = assign_margin(
        model,
        model.flex_margin[flex_id1] + model.flex_margin[flex_id2],
    );
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
        geom1: usize::MAX, // sentinel — no rigid geom
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

### S3. Narrowphase generalization — dual flex ID support

**File:** `sim/L0/core/src/collision/flex_collide.rs`
**MuJoCo equivalent:** Element-element narrowphase in flex-flex and flex-self paths
**Design decision:** Per AD-1, generalize `collide_element_pair()` and all
underlying narrowphase functions (`collide_triangles()`, `collide_tetrahedra()`,
`collide_edges()`, `test_vertex_against_element_faces()`) to accept two flex
IDs. When `flex_id1 == flex_id2`, produce `make_contact_flex_self()`; when
different, produce `make_contact_flex_flex()`. Margin formula becomes
`flex_margin[f1] + flex_margin[f2]` which equals `2*flex_margin[f]` for
self-collision. All self-collision callers updated from `(..., f)` to
`(..., f, f)` — arity change caught by compiler.

**Before** (current signature):
```rust
pub fn collide_element_pair(
    model: &Model, data: &mut Data,
    e1: usize, e2: usize, dim: usize, flex_id: usize,
) { ... }

fn collide_triangles(
    model: &Model, data: &mut Data,
    verts1: &[usize], verts2: &[usize], flex_id: usize,
) { ... }
```

**After** (generalized):
```rust
pub fn collide_element_pair(
    model: &Model, data: &mut Data,
    e1: usize, e2: usize, dim: usize,
    flex_id1: usize, flex_id2: usize,
) {
    let verts1 = elem_vertices(model, e1);
    let verts2 = elem_vertices(model, e2);

    match dim {
        2 => collide_triangles(model, data, &verts1, &verts2, flex_id1, flex_id2),
        3 => collide_tetrahedra(model, data, &verts1, &verts2, flex_id1, flex_id2),
        1 => collide_edges(model, data, &verts1, &verts2, flex_id1, flex_id2),
        _ => {}
    }
}
```

Inside each narrowphase function, the changes are:

```rust
fn collide_triangles(
    model: &Model, data: &mut Data,
    verts1: &[usize], verts2: &[usize],
    flex_id1: usize, flex_id2: usize,
) {
    // ... triangle setup unchanged ...

    // Margin/gap: additive from both flex IDs
    let margin = model.flex_margin[flex_id1] + model.flex_margin[flex_id2];
    let gap = model.flex_gap[flex_id1] + model.flex_gap[flex_id2];
    let includemargin = margin - gap;

    if let Some(contact_result) = triangle_triangle_intersection(&tri_a, &tri_b) {
        if contact_result.depth > -includemargin {
            let nearest1 = nearest_vertex(contact_result.point.coords, verts1, data);
            let nearest2 = nearest_vertex(contact_result.point.coords, verts2, data);

            // Choose contact factory based on self vs cross-object
            let contact = if flex_id1 == flex_id2 {
                make_contact_flex_self(model, nearest1, nearest2, ...)
            } else {
                make_contact_flex_flex(model, nearest1, nearest2, ...)
            };
            data.contacts.push(contact);
            data.ncon += 1;
        }
    }
}
```

The same pattern applies to `collide_tetrahedra()`, `collide_edges()`, and
`test_vertex_against_element_faces()`.

**Self-collision call site updates** (compiler-verified arity change):

```rust
// In mj_collide_flex_self_narrow():
collide_element_pair(model, data, e1, e2, dim, f, f);  // was: ..., f);

// In mj_collide_flex_self_bvh():
collide_element_pair(model, data, e1, e2, dim, f, f);  // was: ..., f);

// In mj_collide_flex_self_sap():
collide_element_pair(model, data, ea, eb, dim, f, f);   // was: ..., f);

// In mj_collide_flex_internal() → test_vertex_against_element_faces():
test_vertex_against_element_faces(model, data, v, &verts2, dim, f, f);  // was: ..., f);
```

**Verification:** All self-collision tests still pass after this change because
`flex_margin[f] + flex_margin[f] == 2*flex_margin[f]` and `flex_id1 == flex_id2`
selects `make_contact_flex_self()`.

### S4. Cross-object midphase — BVH between two flex objects

**File:** `sim/L0/core/src/collision/flex_collide.rs` (new function)
**MuJoCo equivalent:** BVH-accelerated element pair enumeration in flex-flex path
**Design decision:** Per AD-2, build BVH from the larger flex object's elements,
query with each element of the smaller flex. Reuses `element_aabb()` and
`Bvh::build()` from Spec C. No adjacency filtering needed for cross-object
pairs (different flex objects cannot share elements).

**After** (new implementation):
```rust
/// Flex-flex element pair enumeration with BVH midphase.
///
/// Builds BVH from the larger flex's elements, queries with each element
/// of the smaller flex. No adjacency filtering (cross-object elements
/// cannot be adjacent). Dispatches to `collide_element_pair()` for each
/// AABB-overlapping pair.
fn mj_collide_flex_pair(model: &Model, data: &mut Data, f1: usize, f2: usize) {
    let elem_start1 = model.flex_elemadr[f1];
    let elem_count1 = model.flex_elemnum[f1];
    let elem_start2 = model.flex_elemadr[f2];
    let elem_count2 = model.flex_elemnum[f2];

    if elem_count1 == 0 || elem_count2 == 0 {
        return;
    }

    // Determine which flex provides BVH (larger element count) and which
    // provides query elements (smaller). Dim must match for collision.
    let dim1 = model.flex_dim[f1];
    let dim2 = model.flex_dim[f2];

    // Mixed dimensions: use the minimum dim for narrowphase dispatch.
    // MuJoCo dispatches based on the collision pair's effective dimensionality.
    let dim = dim1.min(dim2);

    // Build BVH from the flex with more elements (better tree balance)
    let (bvh_start, bvh_count, query_start, query_count, bvh_flex, query_flex) =
        if elem_count2 >= elem_count1 {
            (elem_start2, elem_count2, elem_start1, elem_count1, f2, f1)
        } else {
            (elem_start1, elem_count1, elem_start2, elem_count2, f1, f2)
        };

    // Build BVH from larger flex's elements
    let primitives: Vec<BvhPrimitive> = (0..bvh_count)
        .map(|i| {
            let e = bvh_start + i;
            let verts = elem_vertices(model, e);
            let aabb = element_aabb(&verts, data);
            BvhPrimitive {
                aabb,
                index: e,
                data: 0,
            }
        })
        .collect();

    let bvh = Bvh::build(primitives);

    // Query each element of smaller flex against BVH
    for i in 0..query_count {
        let e_query = query_start + i;
        let verts_query = elem_vertices(model, e_query);
        let aabb_query = element_aabb(&verts_query, data);

        for candidate_idx in bvh.query(&aabb_query) {
            let e_bvh = bvh_start + candidate_idx;

            // No adjacency filtering — cross-object elements are never adjacent
            // No duplicate filtering — each (query, bvh) pair is unique

            collide_element_pair(model, data, e_query, e_bvh, dim, query_flex, bvh_flex);
        }
    }
}
```

### S5. `mj_collide_flex_flex()` — Broadphase pair loop

**File:** `sim/L0/core/src/collision/mod.rs` (new function, after `mj_collision_flex_self()`)
**MuJoCo equivalent:** `canCollide2()` flex-flex path in `engine_collision_driver.c`
**Design decision:** Direct port of MuJoCo's pair enumeration with
`filterBitmask()`. No `flex_rigid` check (EGT-4 verified). Iterates all
`(f1, f2)` pairs with `f1 < f2` to avoid duplicate pairs. Calls
`mj_collide_flex_pair()` for each bitmask-compatible pair.

**After** (new implementation):
```rust
/// Dispatch flex-flex cross-object collision.
///
/// Iterates all flex pair combinations (f1 < f2), applies bitmask filter,
/// dispatches to element-element narrowphase with BVH midphase. No
/// `flex_rigid` check — rigid flexes still participate in cross-object
/// collision (MuJoCo behavior, EGT-4 verified).
fn mj_collision_flex_flex(model: &Model, data: &mut Data) {
    if model.nflex < 2 {
        return;
    }

    for f1 in 0..model.nflex {
        let ct1 = model.flex_contype[f1];
        let ca1 = model.flex_conaffinity[f1];

        for f2 in (f1 + 1)..model.nflex {
            // filterBitmask: (ct1 & ca2) != 0 || (ct2 & ca1) != 0
            let ct2 = model.flex_contype[f2];
            let ca2 = model.flex_conaffinity[f2];
            if (ct1 & ca2) == 0 && (ct2 & ca1) == 0 {
                continue;
            }

            mj_collide_flex_pair(model, data, f1, f2);
        }
    }
}
```

### S6. Pipeline integration

**File:** `sim/L0/core/src/collision/mod.rs` (inside `mj_collision()`, after line 566)
**MuJoCo equivalent:** Flex-flex dispatch placement in `mj_collision()` after flex-self
**Design decision:** Add `mj_collide_flex_flex()` call after
`mj_collision_flex_self()` in the collision pipeline. This matches MuJoCo's
dispatch order: rigid-rigid → flex-rigid → flex-self → flex-flex. The import
for `mj_collide_flex_pair` is added to the existing import block from
`flex_collide`.

**Before** (current `mj_collision()` end):
```rust
    // Mechanism 3: flex vertex vs rigid geom contacts
    mj_collision_flex(model, data);

    // Mechanism 4: flex self-collision (internal + non-adjacent)
    mj_collision_flex_self(model, data);
}
```

**After:**
```rust
    // Mechanism 3: flex vertex vs rigid geom contacts
    mj_collision_flex(model, data);

    // Mechanism 4: flex self-collision (internal + non-adjacent)
    mj_collision_flex_self(model, data);

    // Mechanism 5: flex-flex cross-object collision
    mj_collide_flex_flex(model, data);
}
```

The new dispatch function `mj_collide_flex_flex()` is defined in `mod.rs`
(same file) and calls `mj_collide_flex_pair()` which is in `flex_collide.rs`.
Add `mj_collide_flex_pair` to the import block at the top of `mod.rs`:

```rust
use self::flex_collide::{
    make_contact_flex_rigid, mj_collide_flex_internal, mj_collide_flex_pair,
    mj_collide_flex_self_bvh, mj_collide_flex_self_narrow,
    mj_collide_flex_self_sap, narrowphase_sphere_geom,
};
```

### S7. Sentinel-safe downstream fixes

**File:** Multiple files (see below)
**MuJoCo equivalent:** N/A — these are CortenForge-specific bugs caused by
the sentinel `geom1/geom2 = usize::MAX` convention for flex contacts
**Design decision:** Fix all unguarded `model.geom_body[contact.geom*]`
access sites to handle flex contacts correctly. The fix pattern: check
`flex_vertex`/`flex_vertex2` first to derive bodies from vertex indices,
fall back to `geom_body[]` only for rigid-rigid contacts. These fixes are
required for flex-flex contacts to produce active constraint forces without
panicking. They also fix pre-existing latent bugs for flex-self contacts.

#### S7a. Force distribution — `forward/acceleration.rs:510–511` (CRITICAL)

**File:** `sim/L0/core/src/forward/acceleration.rs`, lines 510–511

**Before** (current — will PANIC for flex contacts):
```rust
let contact = &data.contacts[ci];
let body1 = model.geom_body[contact.geom1];
let body2 = model.geom_body[contact.geom2];
```

**After:**
```rust
let contact = &data.contacts[ci];
let (body1, body2) = contact_bodies(model, contact);
```

Where `contact_bodies()` is a shared helper defined in `types/contact_types.rs`:

```rust
impl Contact {
    /// Derive the two body indices for this contact.
    ///
    /// For rigid-rigid: `(geom_body[geom1], geom_body[geom2])`.
    /// For flex-rigid: `(flexvert_bodyid[vertex], geom_body[geom2])`.
    /// For flex-self/flex-flex: `(flexvert_bodyid[v1], flexvert_bodyid[v2])`.
    #[inline]
    pub fn bodies(&self, model: &Model) -> (usize, usize) {
        match (self.flex_vertex, self.flex_vertex2) {
            (Some(vi1), Some(vi2)) => {
                // Flex self-collision or flex-flex
                (model.flexvert_bodyid[vi1], model.flexvert_bodyid[vi2])
            }
            (Some(vi), None) => {
                // Flex-rigid: vertex body + rigid geom body
                (model.flexvert_bodyid[vi], model.geom_body[self.geom2])
            }
            _ => {
                // Rigid-rigid
                (model.geom_body[self.geom1], model.geom_body[self.geom2])
            }
        }
    }
}
```

#### S7b. Sleep/wake detection — `island/sleep.rs:557–558` (CRITICAL)

**File:** `sim/L0/core/src/island/sleep.rs`, lines 557–558

**Before** (current — will PANIC):
```rust
let body1 = model.geom_body[contact.geom1];
let body2 = model.geom_body[contact.geom2];
```

**After:**
```rust
let (body1, body2) = contact.bodies(model);
```

#### S7c. Impedance bodyweight — `constraint/impedance.rs:489–498` (MEDIUM)

**File:** `sim/L0/core/src/constraint/impedance.rs`, lines 489–498

The current code has bounds-checking but falls back to body 0 (world body,
`invweight0[0] = 0.0`) for flex contacts with sentinel geoms. This produces
wrong bodyweight — world body has zero inverse weight, so impedance
computation gets `0.0` instead of the flex vertex body's inverse weight.

**Before** (wrong logic):
```rust
let b1 = if contact.geom1 < model.geom_body.len() {
    model.geom_body[contact.geom1]
} else {
    0
};
```

**After:**
```rust
let (b1, b2) = contact.bodies(model);
```

This replaces the entire `b1`/`b2` derivation block with the shared helper.

#### S7d. Island construction — `island/mod.rs:61–70` (MEDIUM)

**File:** `sim/L0/core/src/island/mod.rs`, lines 61–70

The current code has bounds-checking but `continue`s for flex contacts,
which means flex contacts never create island connectivity. Two flex objects
in contact should be in the same island for correct sleep grouping.

**Before** (flex contacts skipped):
```rust
let body1 = if contact.geom1 < model.geom_body.len() {
    model.geom_body[contact.geom1]
} else {
    continue;
};
```

**After:**
```rust
let (body1, body2) = contact.bodies(model);
```

Remove the bounds-check conditionals and use the shared helper directly.

#### S7e. Touch sensor — `sensor/acceleration.rs:171–180` (MEDIUM)

**File:** `sim/L0/core/src/sensor/acceleration.rs`, lines 171–180

The current code has bounds-checking but falls back to `usize::MAX` for flex
contacts, which means touch sensors never fire on flex-flex contacts because
the body match `body_id == geom1_body` uses `geom_body[usize::MAX]`.

**Before** (touch never fires for flex contacts):
```rust
let geom1_body = if c.geom1 < model.ngeom {
    model.geom_body[c.geom1]
} else {
    usize::MAX
};
let geom2_body = if c.geom2 < model.ngeom {
    model.geom_body[c.geom2]
} else {
    usize::MAX
};
```

**After:**
```rust
let (geom1_body, geom2_body) = c.bodies(model);
```

This allows touch sensors to correctly detect forces from flex contacts
(both flex-self and flex-flex).

---

## Acceptance Criteria

### AC1: Bitmask filter — incompatible → zero contacts *(runtime test — EGT-2 verified)*
**Given:** Two dim=2 flexcomp grids (3×3), overlapping positions. flex1
`contype=1, conaffinity=1`. flex2 `contype=2, conaffinity=2`.
**After:** `mj_collision()`
**Assert:** `data.ncon` for flex-flex contacts = 0 (bitmask: `(1&2)=0` and `(2&1)=0`)
**Field:** `data.contacts` (no contacts with both `flex_vertex` and `flex_vertex2` from different flexes)

### AC2: Bitmask filter — compatible → contacts generated *(runtime test — EGT-2 verified)*
**Given:** Two dim=2 flexcomp grids (3×3), overlapping positions. flex1
`contype=1, conaffinity=3`. flex2 `contype=2, conaffinity=1`.
**After:** `mj_collision()`
**Assert:** `data.ncon` for flex-flex contacts = 32 (bitmask: `(1&1)=1`)
**Field:** `data.contacts`, filtered to contacts where both `flex_vertex` and `flex_vertex2` are `Some` and vertices belong to different flex objects

### AC3: Contact param — priority winner takes all *(runtime test — EGT-3 verified)*
**Given:** Two overlapping flexes. flex1 `priority=1, solref=[0.05,2.0], friction=0.5`. flex2 `priority=0, solref=[0.02,1.0], friction=0.3`.
**After:** `mj_collision()`
**Assert:** All flex-flex contacts have `solref=[0.05, 2.0]` and `friction=0.5` (flex1 wins)
**Field:** `contact.solref`, `contact.friction`

### AC4: Contact param — solmix weighted blend *(runtime test — EGT-3 verified)*
**Given:** Two overlapping flexes, equal priority. flex1 `solmix=0.5, solref=[0.02,1.0], friction=0.8`. flex2 `solmix=1.5, solref=[0.04,2.0], friction=0.2`.
**After:** `mj_collision()`
**Assert:** `solref=[0.035, 1.75] ± 1e-12`, `friction=0.8` (element-wise max)
**Field:** `contact.solref`, `contact.friction`

### AC5: Margin/gap additive *(runtime test — EGT-5 verified)*
**Given:** Two overlapping flexes. flex1 `margin=0.05, gap=0.01`. flex2 `margin=0.03, gap=0.02`.
**After:** `mj_collision()`
**Assert:** `contact.includemargin = 0.05 ± 1e-12` (i.e., `(0.05+0.03) - (0.01+0.02)`)
**Field:** `contact.includemargin`

### AC6: `flex_rigid` does NOT gate flex-flex *(runtime test — EGT-4 verified)*
**Given:** Two overlapping flexes. flex1 has ALL 9 vertices pinned (`invmass=0`, so `flex_rigid[0]=true`). flex2 unpinned.
**After:** `mj_collision()`
**Assert:** flex-flex contacts > 0 (identical count to unpinned case)
**Field:** `data.ncon`, filtered to flex-flex

### AC7: Multi-flex all-pairs enumeration *(runtime test — EGT-6 verified)*
**Given:** Three overlapping dim=2 flexcomp grids (2×2, 2 triangles each), all with compatible bitmasks.
**After:** `mj_collision()`
**Assert:** Flex-flex contacts exist for all three pairs: `(0,1)`, `(0,2)`, `(1,2)`. No duplicate or missed pairs.
**Field:** `data.contacts`, grouped by `(flexvert_flexid[flex_vertex], flexvert_flexid[flex_vertex2])`

### AC8: Condim combination — max *(runtime test — EGT-7 verified)*
**Given:** Two overlapping flexes. flex1 `condim=1`. flex2 `condim=6`.
**After:** `mj_collision()`
**Assert:** All flex-flex contacts have `dim = 6` (i.e., `max(1,6)=6`)
**Field:** `contact.dim`

### AC9: Contact encoding *(runtime test — EGT-1 verified)*
**Given:** Two overlapping dim=2 flexes.
**After:** `mj_collision()`
**Assert:** All flex-flex contacts have: `geom1 = usize::MAX`, `geom2 = usize::MAX`, `flex_vertex = Some(v1)` where `flexvert_flexid[v1]` is one flex, `flex_vertex2 = Some(v2)` where `flexvert_flexid[v2]` is the other flex.
**Field:** `contact.geom1`, `contact.geom2`, `contact.flex_vertex`, `contact.flex_vertex2`

### AC10: Full forward step without panic — sentinel safety *(runtime test — integration)*
**Given:** Two overlapping dim=2 flexes with compatible bitmasks, non-zero mass, gravity enabled. Flex-flex contacts generated with `geom1 = geom2 = usize::MAX` sentinel.
**After:** `mj_step()` equivalent (full forward step including constraint solving, producing active flex-flex constraint forces)
**Assert:**
1. No panic in `mj_fwd_constraint()` force distribution loop (`acceleration.rs:510-511`) — `contact.bodies()` returns valid body indices, not `geom_body[usize::MAX]`.
2. No panic in `mj_wake_collision()` wake-contact check (`sleep.rs:557-558`) — `contact.bodies()` returns valid body indices.
3. Island construction (`island/mod.rs:61-70`) completes — flex-flex contacts create island connectivity between flex object bodies.
4. Impedance bodyweight (`impedance.rs:489-498`) uses flex vertex body's `invweight0`, not world body fallback.
5. `data.qfrc_constraint` has non-zero entries at DOFs of vertices involved in flex-flex contacts (confirms active constraint forces flow through the full pipeline).
**Field:** `data.qfrc_constraint`, `data.cfrc_ext`

### AC11: S10 global override applied *(runtime test)*
**Given:** Two overlapping flexes with `ENABLE_OVERRIDE` active, `o_margin=0.1, o_solref=[0.05,2.0], o_solimp=[0.95,0.99,0.001,0.5,2.0], o_friction=[1.0,1.0,0.01,0.001,0.001]`.
**After:** `mj_collision()`
**Assert:** All flex-flex contacts have `solref=o_solref`, `solimp=o_solimp`, friction from `o_friction`, margin = `o_margin`.
**Field:** `contact.solref`, `contact.solimp`, `contact.mu`, `contact.includemargin`

### AC12: Self-collision regression *(runtime test)*
**Given:** Single flex with `selfcollide="auto"`, folding mesh that generates self-contacts.
**After:** `mj_collision()`
**Assert:** Self-collision contact count and values identical to pre-Spec-D implementation. Narrowphase generalization (f, f) produces same results as original (f).
**Field:** `data.ncon`, `contact.depth`, `contact.normal`

### AC13: Existing flex-rigid regression *(runtime test)*
**Given:** Flex-rigid model (flex vertex colliding with a geom).
**After:** `mj_collision()`
**Assert:** Flex-rigid contact count and values identical to pre-Spec-D implementation. S7 changes do not affect rigid path.
**Field:** `data.ncon`, `contact.depth`

### AC14: Sentinel-safe body derivation *(code review)*
**Assert:** No `model.geom_body[contact.geom1]` or `model.geom_body[contact.geom2]` access anywhere in the codebase without prior validation that the geom index is valid (i.e., not `usize::MAX`). All contact body derivation uses `Contact::bodies()` or equivalent safe pattern.
**Sites to verify:**
- `forward/acceleration.rs:510-511` — uses `contact.bodies()`
- `island/sleep.rs:557-558` — uses `contact.bodies()`
- `constraint/impedance.rs:489-498` — uses `contact.bodies()`
- `island/mod.rs:61-70` — uses `contact.bodies()`
- `sensor/acceleration.rs:171-180` — uses `contact.bodies()`

### AC15: Constraint Jacobian works for flex-flex *(runtime test — verification)*
**Given:** Two overlapping flexes producing flex-flex contacts.
**After:** Full constraint assembly
**Assert:** `compute_flex_self_contact_jacobian()` is invoked for flex-flex contacts (via `flex_vertex2.is_some()` dispatch at `jacobian.rs:208-209`). Jacobian has non-zero entries at DOFs of both flex objects' vertices. No modification to Jacobian code needed.
**Field:** Constraint Jacobian rows for flex-flex contacts

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (incompatible bitmask) | T1 | Direct |
| AC2 (compatible bitmask) | T2 | Direct |
| AC3 (priority winner) | T3 | Direct |
| AC4 (solmix blend) | T4 | Direct |
| AC5 (margin/gap additive) | T5 | Direct |
| AC6 (flex_rigid non-gate) | T6 | Direct |
| AC7 (multi-flex all-pairs) | T7 | Direct |
| AC8 (condim max) | T8 | Direct |
| AC9 (contact encoding) | T2 | Direct (encoding verified within compatible test) |
| AC10 (full step no panic) | T9 | Integration |
| AC11 (S10 override) | T10 | Direct |
| AC12 (self-collision regression) | T11 | Regression |
| AC13 (flex-rigid regression) | T12 | Regression |
| AC14 (sentinel safety) | — | Code review (manual) |
| AC15 (Jacobian works) | T9 | Integration (verified as part of full step) |

---

## Test Plan

### T1: Incompatible bitmask → zero flex-flex contacts → AC1
Two overlapping 3×3 dim=2 flexcomp grids. flex1 `contype=1, conaffinity=1`.
flex2 `contype=2, conaffinity=2`. After `mj_collision()`, assert zero flex-flex
contacts. Flex-rigid contacts may exist if rigid geoms present. Filter contacts
by checking both `flex_vertex` and `flex_vertex2` are `Some` with vertices from
different flex objects. MuJoCo 3.5.0 verified: 0 flex-flex contacts (EGT-2).

### T2: Compatible bitmask → 32 contacts with correct encoding → AC2, AC9
Two overlapping 3×3 dim=2 flexcomp grids (8 triangles each). flex1 `contype=1,
conaffinity=3`. flex2 `contype=2, conaffinity=1`. After `mj_collision()`,
assert 32 flex-flex contacts. Each contact has `geom1 = usize::MAX`,
`geom2 = usize::MAX`, `flex_vertex = Some(v)` with `flexvert_flexid[v]` = one
flex, `flex_vertex2 = Some(v2)` with `flexvert_flexid[v2]` = other flex.
MuJoCo 3.5.0 verified: 32 contacts (EGT-2, EGT-8).

### T3: Priority winner takes all → AC3
Two overlapping flexes. flex1 `priority=1`, flex2 `priority=0`. flex1
`solref=[0.05, 2.0], friction=Vector3(0.5, 0.005, 0.0001)`. After collision,
all flex-flex contacts have `solref=[0.05, 2.0]` and `friction=0.5`.
MuJoCo 3.5.0 verified (EGT-3).

### T4: Solmix weighted blend → AC4
Two overlapping flexes, equal priority. flex1 `solmix=0.5, solref=[0.02, 1.0],
friction=Vector3(0.8, 0.005, 0.0001)`. flex2 `solmix=1.5, solref=[0.04, 2.0],
friction=Vector3(0.2, 0.005, 0.0001)`. mix = 0.5/(0.5+1.5) = 0.25. After
collision: `solref=[0.035, 1.75]`, `friction=0.8` (max). Tolerance: 1e-12.
MuJoCo 3.5.0 verified (EGT-3).

### T5: Margin/gap additive → AC5
Two overlapping flexes. flex1 `margin=0.05, gap=0.01`. flex2 `margin=0.03,
gap=0.02`. After collision: `contact.includemargin = 0.05 ± 1e-12`
(= (0.05+0.03) - (0.01+0.02)). MuJoCo 3.5.0 verified (EGT-5).

### T6: flex_rigid does NOT gate flex-flex → AC6
Two overlapping 3×3 flexes. flex1 has ALL 9 vertices pinned (invmass=0,
`flex_rigid[0]=true`). flex2 unpinned. After collision: flex-flex contact count
is identical to the unpinned case (32). MuJoCo 3.5.0 verified (EGT-4).

### T7: Three-flex all-pairs enumeration → AC7
Three overlapping 2×2 dim=2 grids (2 triangles each), all compatible bitmasks.
After collision: flex-flex contacts exist for pairs (0,1), (0,2), (1,2). Group
contacts by flex pair (derived from `flexvert_flexid[]` on both vertices).
Each pair has at least 1 contact. No duplicate pairs. MuJoCo 3.5.0 verified
(EGT-6).

### T8: Condim max combination → AC8
Two overlapping flexes. flex1 `condim=1`. flex2 `condim=6`. After collision:
all flex-flex contacts have `dim = 6`. MuJoCo 3.5.0 verified (EGT-7).

### T9: Full forward step integration — active constraint forces, no panic → AC10, AC15
Two overlapping dim=2 flexes with compatible bitmasks, non-zero mass, gravity.
Run full `mj_step()` equivalent (kinematics → collision → constraint assembly →
constraint solve → force distribution → integration). The flex-flex contacts
must produce **active constraint forces** (non-zero `data.qfrc_constraint`
at contact vertex DOFs). Assert:
1. No panic at `acceleration.rs:510-511` (`mj_fwd_constraint()` force
   distribution loop) — verifies S7a fix handles sentinel `geom1/geom2`.
2. No panic at `sleep.rs:557-558` (`mj_wake_collision()` wake-contact check)
   — verifies S7b fix.
3. `data.qfrc_constraint` has non-zero entries at DOFs of vertices involved
   in flex-flex contacts — confirms forces flow through the full constraint
   pipeline (Jacobian → assembly → solve → distribution).
4. `data.cfrc_ext` for flex vertex bodies is non-zero — confirms wrench
   distribution reaches body level.

This exercises the full pipeline including:
- Force distribution (`acceleration.rs` S7a fix)
- Sleep/wake detection (`sleep.rs` S7b fix)
- Island construction (`island/mod.rs` S7d fix)
- Impedance computation (`impedance.rs` S7c fix)
- Constraint Jacobian (`jacobian.rs` — AC15 verified here)

### T10: S10 global override → AC11
Two overlapping flexes with `ENABLE_OVERRIDE` active. Set `o_margin=0.1`,
`o_solref=[0.05, 2.0]`, `o_solimp=[0.95, 0.99, 0.001, 0.5, 2.0]`,
`o_friction=[1.0, 1.0, 0.01, 0.001, 0.001]`. After collision: all flex-flex
contacts use override values.

### T11: Self-collision regression → AC12
Single flex with `selfcollide="auto"` and a mesh that generates self-contacts
(e.g., folded sheet). After collision: self-contact count and `contact.depth`
values match pre-Spec-D behavior. Verify the generalization from
`collide_element_pair(..., f)` to `collide_element_pair(..., f, f)` is
transparent.

### T12: Flex-rigid regression → AC13
Flex-rigid model (flex vertex sphere colliding with a rigid box). After
collision: contact count and values match pre-Spec-D behavior. S7 changes
to body derivation do not affect the rigid path.

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| Incompatible bitmasks → zero contacts | Ensures bitmask filter is applied, not bypassed | T1 | AC1 |
| `flex_rigid[f]=true` on one flex → still collides | MuJoCo does NOT gate flex-flex on flex_rigid (EGT-4) | T6 | AC6 |
| Same-flex pair excluded (f1 < f2) | Pair loop must not test flex against itself — that's self-collision | T7 (3 flexes, verify no self-pairs in flex-flex contacts) | AC7 |
| Zero-element flex | flex with `elemnum=0` — `mj_collide_flex_pair()` must early-return | T2 (implicit — if test model is well-formed, zero-elem path not triggered. Explicit unit test for early return.) | AC2 |
| Single-element flex pair | Minimal valid case: one triangle vs one triangle | Covered by T7 (2×2 grids = 2 elements each) | AC7 |
| Three+ flex objects | All-pairs enumeration produces C(n,2) pairs | T7 | AC7 |
| Condim mismatch (max) | `condim=1` vs `condim=6` → result is `6` | T8 | AC8 |
| S10 global override | Override replaces per-flex contact params | T10 | AC11 |
| Full forward step with active constraints | Exercises sentinel-guarded downstream: force distribution, sleep/wake, island, impedance | T9 | AC10, AC15 |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T13 (contact_param_flex_flex unit) | Unit test for `contact_param_flex_flex()` in isolation | Validates parameter combination logic independently from collision dispatch — catches bugs in the function itself without requiring model setup |
| T14 (zero-element flex early return) | `mj_collide_flex_pair()` with one flex having 0 elements | Boundary condition for the midphase — must not panic or produce spurious contacts |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| Flex-flex contacts generated | No flex-flex contacts — two flex objects pass through each other | Element-element contacts generated for bitmask-compatible flex pairs | Toward MuJoCo | Models with 2+ flex objects | None — transparent addition |
| Narrowphase function signatures | `collide_element_pair(..., flex_id)` — single flex ID | `collide_element_pair(..., flex_id1, flex_id2)` — two flex IDs | N/A (internal refactor) | Internal callers (self-collision dispatch) | Update call sites to pass `(f, f)` — compiler catches all |
| Force distribution body derivation | `model.geom_body[contact.geom1]` — panics for sentinel | `contact.bodies(model)` — safe for all contact types | Bug fix | All contacts — fixes pre-existing flex-self bug | None — transparent fix |
| Sleep/wake body derivation | `model.geom_body[contact.geom1]` — panics for sentinel | `contact.bodies(model)` — safe for all contact types | Bug fix | All contacts | None |
| Impedance bodyweight | Falls back to world body (invweight=0) for flex contacts | Uses flex vertex body's invweight | Bug fix — toward MuJoCo | Flex self-collision (pre-existing) + flex-flex | Impedance values change for flex contacts |
| Island construction | Flex contacts skipped — no island connectivity | Flex contacts create island connectivity between flex object bodies | Bug fix — toward MuJoCo | Sleep grouping for flex objects in contact | Flex objects in contact now sleep together |
| Touch sensor body match | Touch sensors never fire on flex contacts | Touch sensors correctly detect forces from flex contacts | Bug fix — toward MuJoCo | Touch sensors attached to flex vertex bodies | Touch sensor readings change for flex models |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `sim/L0/core/src/collision/mod.rs` | Add `contact_param_flex_flex()`, `mj_collide_flex_flex()`, import `mj_collide_flex_pair`, add dispatch call | +60 |
| `sim/L0/core/src/collision/flex_collide.rs` | Add `make_contact_flex_flex()`, `mj_collide_flex_pair()`. Generalize `collide_element_pair()`, `collide_triangles()`, `collide_tetrahedra()`, `collide_edges()`, `test_vertex_against_element_faces()` to accept two flex IDs. Update self-collision callers. | ~120 modified, +80 new |
| `sim/L0/core/src/types/contact_types.rs` | Add `Contact::bodies()` method | +20 |
| `sim/L0/core/src/forward/acceleration.rs` | Replace `model.geom_body[contact.geom*]` with `contact.bodies(model)` | ~4 modified |
| `sim/L0/core/src/island/sleep.rs` | Replace `model.geom_body[contact.geom*]` with `contact.bodies(model)` | ~4 modified |
| `sim/L0/core/src/constraint/impedance.rs` | Replace `geom_body` bounds-check block with `contact.bodies(model)` | ~12 modified |
| `sim/L0/core/src/island/mod.rs` | Replace `geom_body` bounds-check block with `contact.bodies(model)` | ~12 modified |
| `sim/L0/core/src/sensor/acceleration.rs` | Replace `geom_body` bounds-check block with `contact.bodies(model)` (c.bodies(model)) | ~8 modified |
| `sim/L0/core/tests/flex_flex_collision_tests.rs` | New test file | +400 |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| `test_flex_self_collision_*` | `flex_self_collision_tests.rs` | **Pass (unchanged)** | Narrowphase generalization produces identical results for `(f, f)` calls |
| `test_flex_internal_collision_*` | `flex_self_collision_tests.rs` | **Pass (unchanged)** | `test_vertex_against_element_faces` generalization preserves behavior |
| `test_flex_rigid_*` | Flex conformance tests | **Pass (unchanged)** | `contact.bodies()` returns `(flexvert_bodyid, geom_body)` for flex-rigid — same as current |
| `test_rigid_rigid_*` | Contact param tests, collision tests | **Pass (unchanged)** | `contact.bodies()` returns `(geom_body, geom_body)` for rigid-rigid — same as current |
| Impedance tests with flex contacts | `impedance_tests.rs` | **Value change** — bodyweight values change for flex contacts (from world body 0 to actual flex vertex body). This is a **conformance improvement**, not a regression. | S7c fix: impedance now uses correct body inverse weight instead of world body |
| Island tests with flex contacts | `island_tests.rs` | **Behavior change** — flex contacts now create island connectivity. If any test asserts specific island structure for flex models, values may change. | S7d fix: flex contacts participate in island grouping |

### Non-Modification Sites

| File:line | What it does | Why NOT modified |
|-----------|-------------|-----------------|
| `constraint/jacobian.rs:149-192` | `compute_flex_self_contact_jacobian()` | Already works for flex-flex: reads `flexvert_dofadr[vi1]` and `flexvert_dofadr[vi2]` — DOF addresses are independent of flex ID |
| `constraint/jacobian.rs:207-211` | `flex_vertex2.is_some()` dispatch | Already dispatches to `compute_flex_self_contact_jacobian()` for any contact with both `flex_vertex` and `flex_vertex2` — works for flex-flex |
| `constraint/assembly.rs:603-624` | `(Some(vi1), Some(vi2))` bodyweight path | Already handles dual flex vertex contacts via `flexvert_bodyid` — works for flex-flex |
| `forward/actuation.rs:239` | `flex_vertex.is_some()` skip | Already skips flex contacts before `geom_body` access — safe |
| `collision/flex_collide.rs:250-303` | `make_contact_flex_rigid()` | Not modified — handles flex-rigid contacts only |

---

## Execution Order

1. **S7 (sentinel fixes) first** — Fix all downstream `geom_body[contact.geom*]`
   sites. This is prerequisite for flex-flex contacts producing active
   constraints. Define `Contact::bodies()` in `contact_types.rs`, then update
   all five consumer sites. → Run domain tests to verify no regression in
   rigid-rigid and flex-rigid paths.

2. **S1 (contact_param_flex_flex)** — Add the parameter combination function.
   No callers yet, but unit-testable in isolation. → Test with T13.

3. **S2 (make_contact_flex_flex)** — Add the contact factory. No callers yet.

4. **S3 (narrowphase generalization)** — Generalize `collide_element_pair()` and
   all underlying functions to accept two flex IDs. Update all self-collision
   callers from `(..., f)` to `(..., f, f)`. → Run domain tests to verify
   self-collision regression (T11, AC12). This is the highest-risk step because
   it modifies existing narrowphase code.

5. **S4 (cross-object midphase)** — Add `mj_collide_flex_pair()`. No callers
   yet.

6. **S5 + S6 (dispatch + pipeline)** — Add `mj_collide_flex_flex()` and wire
   it into `mj_collision()`. → Run full test suite (T1–T12).

---

## Out of Scope

- **Body-attached flex vertices** (§27D) — Vertices attached to multi-DOF
  bodies. Flex-flex collision works for free vertices (current models); §27D
  would change the Jacobian path. *Conformance impact: affects body-attached
  flex models. Deferred to post-v1.0.*

- **`activelayers` runtime consumption** (DT-150) — `flex_activelayers` is
  parsed and stored but not consumed at runtime for collision filtering.
  *Conformance impact: minimal — optimization for layered collision filtering.*

- **GPU flex collision** (DT-67) — Post-v1.0 GPU acceleration.
  *Conformance impact: none — physics results unchanged.*

- **SAP midphase for flex-flex** — MuJoCo may use SAP for certain flex-flex
  configurations. This spec uses BVH for all flex-flex midphase. BVH produces
  identical contact sets to SAP (and brute-force). *Conformance impact: none
  — contact results are identical regardless of midphase algorithm.*

- **Flex-flex with mixed dim** — Two flex objects with different `flex_dim`
  values (e.g., shell vs solid). Currently handled by using `min(dim1, dim2)`
  for narrowphase dispatch. Full MuJoCo conformance for mixed-dim flex-flex
  collision is not verified empirically. *Conformance impact: edge case —
  most models use same dim for all flex objects.*

- **Tet-tet edge-edge tests** (DT-151) — Tetrahedron-tetrahedron collision
  only uses vertex-face tests (inherited from Spec C). Edge-edge tests
  deferred. *Conformance impact: may miss some tet-tet contacts where edges
  intersect without vertex-face penetration.*
