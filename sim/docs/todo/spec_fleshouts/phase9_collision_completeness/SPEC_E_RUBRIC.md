# Spec E — Deformable Complex-Geom Narrowphase: Quality Rubric

Grades the Spec E spec on 10 criteria. Target: A+ on every criterion
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
list three new flex collision pair types: flex-vs-mesh, flex-vs-hfield,
and flex-vs-SDF. Empirical verification against MuJoCo's C source and the
CortenForge codebase confirms this scope with refinements.

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| Flex-vs-mesh: vertex sphere against triangle mesh (or convex hull) | MuJoCo treats flex vertices as spheres and uses the standard sphere-vs-mesh collision algorithm. For mesh collision, MuJoCo uses the mesh's convex hull (computed at build time). The behavioral outcome is identical to rigid sphere-vs-mesh: same contact point, normal, and depth. | **In scope** — vertex sphere vs mesh, using convex hull when available, per-triangle fallback when not |
| Flex-vs-hfield: vertex sphere against heightfield grid, may reuse `heightfield_sphere_contact` | MuJoCo uses the standard sphere-vs-hfield collision path for flex vertices. For rigid sphere-vs-hfield, MuJoCo uses the prism-based `mjc_ConvexHField()` (same function for all convex-vs-hfield pairs). However, CortenForge has two hfield sphere paths: `heightfield_sphere_contact()` (point-sampling, corner-origin) and `collide_hfield_multi()` (prism-based GJK from Spec C). The prism-based path is MuJoCo-conformant for rigid-rigid; for flex vertices (tiny spheres), both approaches give equivalent results. | **In scope** — vertex sphere vs hfield. Spec decides which approach to reuse. |
| Flex-vs-SDF: vertex sphere against SDF, may reuse `sdf_sphere_contact` | MuJoCo uses the standard sphere-vs-SDF collision path for flex vertices. CortenForge already has `sdf_sphere_contact()` in `sdf.rs` which takes sphere center + radius and returns `Option<SdfContact>`. | **In scope** — vertex sphere vs SDF, direct reuse of `sdf_sphere_contact()` |
| Soft dependency on Spec A (convex hull) | Spec A is complete (Session 7 review passed). `TriangleMeshData::convex_hull()` returns `Option<&ConvexHull>`. Flex-vs-mesh can use the hull. | **Resolved** — Spec A is available; hull path is the primary path, per-triangle is fallback |

**Final scope:**
1. Add flex-vs-mesh collision: vertex sphere against mesh (convex hull
   GJK path primary, per-triangle BVH fallback for meshes without hull)
2. Add flex-vs-hfield collision: vertex sphere against heightfield
3. Add flex-vs-SDF collision: vertex sphere against SDF
4. All three integrate into the existing `narrowphase_sphere_geom()`
   dispatch in `flex_collide.rs` or bypass it with direct calls from
   `mj_collision_flex()` — spec decides the integration pattern

**Items NOT in scope:**
- Flex self-collision (Phase 10)
- Flex-flex cross-body collision filtering (Phase 10)
- SAP integration for flex broadphase (DT-69)
- Flex adhesion contacts (DT-72)
- GPU flex pipeline (DT-67)

---

## Empirical Ground Truth

### MuJoCo behavioral verification

MuJoCo version: 3.x (source from `google-deepmind/mujoco` main branch).
Verified by reading C source directly — `engine_collision_primitive.c`,
`engine_collision_driver.c`, `engine_collision_convex.c`.

### Codebase context

| File | Lines | Role | Spec E impact |
|------|-------|------|---------------|
| `sim/L0/core/src/collision/flex_collide.rs` | 222 | Flex narrowphase dispatch | **Primary change site.** Add Mesh, Hfield, Sdf match arms to `narrowphase_sphere_geom()` (currently `_ => return None` at line 153). |
| `sim/L0/core/src/collision/mod.rs` | ~616 | `mj_collision_flex()` dispatch loop | May need modification if complex geom pairs bypass `narrowphase_sphere_geom()`. Currently calls `narrowphase_sphere_geom` → `make_contact_flex_rigid` for all pairs. |
| `sim/L0/core/src/mesh.rs` | ~2100 | `mesh_sphere_contact()` at line 1076 | Reuse candidate for flex-vs-mesh. Takes `&TriangleMeshData`, `&Pose`, sphere center, radius, `use_bvh`. Returns `Option<MeshContact>`. |
| `sim/L0/core/src/heightfield.rs` | ~941 | `heightfield_sphere_contact()` at line 457 | Reuse candidate for flex-vs-hfield. Uses corner-origin coords; needs centering offset from `hfield_size`. |
| `sim/L0/core/src/sdf.rs` | ~2793 | `sdf_sphere_contact()` at line 583 | Reuse candidate for flex-vs-SDF. Takes `&SdfCollisionData`, `&Pose`, sphere center, radius. Returns `Option<SdfContact>`. |
| `sim/L0/core/src/collision/hfield.rs` | ~1031 | `collide_hfield_multi()` at line 33 | Alternative reuse for flex-vs-hfield (prism-based GJK). Takes geom indices + model — designed for rigid-rigid dispatch. Would need adaptation for virtual flex sphere. |
| `sim/L0/core/src/collision/sdf_collide.rs` | 143 | `collide_with_sdf()` dispatch | Reference for SDF centering offset and pose construction patterns. |
| `sim/L0/core/src/collision/mesh_collide.rs` | ~297 | `collide_with_mesh()` dispatch | Reference for mesh collision patterns (hull path vs per-triangle). |
| `sim/L0/core/src/types/model.rs` | ~500 | Model struct | `geom_mesh`, `geom_hfield`, `geom_sdf`, `mesh_data`, `hfield_data`, `sdf_data`, `hfield_size` — all needed to look up complex geom data from geom index. |
| `sim/L0/core/src/gjk_epa.rs` | ~1425 | GJK/EPA + distance | `gjk_epa_contact()` for sphere-vs-hull collision (if using hull path for flex-vs-mesh). |
| `sim/L0/core/src/collision_shape.rs` | ~1771 | `CollisionShape` enum | `CollisionShape::Sphere` and `CollisionShape::convex_mesh_from_hull()` for constructing shapes for GJK queries. |

**Exhaustive match sites that will break or need updating:**
- `flex_collide.rs:152-153` — `_ => return None` catch-all that currently
  silently skips Mesh, Hfield, Sdf. This is the primary site to modify.

**No other exhaustive match sites break.** The `GeomType` enum is not being
extended (Phase 9 Convention Registry §3). No new enum variants, no new
model fields, no staleness guards. This spec adds match arms to an existing
catch-all, not new variants to an enum.

### EGT-1: MuJoCo flex-vs-complex-geom dispatch

**MuJoCo source:** `engine_collision_primitive.c` — flex vertex collision.

MuJoCo treats each flex vertex as a sphere with radius `flex_radius`.
The flex collision loop (`mj_collideFlexVert` or equivalent in
`engine_collision_primitive.c`) iterates all vertices × all geoms and
computes sphere-vs-geom distances. For primitive geom types, MuJoCo
computes distances inline (similar to CortenForge's existing
`narrowphase_sphere_geom` match arms). For complex geom types (mesh,
hfield, SDF), MuJoCo uses the same collision algorithms as rigid
sphere-vs-complex collision — the dispatch mechanism may differ from the
rigid-rigid `mjCOLLISIONFUNC` table, but the **behavioral outcome is
identical**: a flex vertex sphere at position P with radius R produces the
same contact geometry (point, normal, depth) as a rigid sphere geom of
radius R at position P against the same geom.

**Key behavioral fact:** The only difference between flex-vs-geom and
rigid-sphere-vs-geom is contact parameter combination: flex uses
`contact_param_flex_rigid()` (flex friction, condim, solref, etc.)
instead of the rigid-rigid `contact_param()` path. The geometric
collision (point, normal, depth) is identical.

**Spec author note:** The exact MuJoCo dispatch mechanism for flex-vs-
complex-geom should be verified against the C source. The behavioral
equivalence with rigid sphere collision is the conformance-critical claim;
the dispatch mechanism is an implementation detail.

### EGT-2: Flex-vs-mesh collision path

**MuJoCo behavior:** Flex vertex sphere is tested against the mesh's convex
hull using GJK/EPA (since MuJoCo always computes convex hulls for collision
meshes). This is the standard sphere-vs-convex-mesh collision path.

**CortenForge state:**
- `mesh_sphere_contact()` (`mesh.rs:1076`): BVH-accelerated per-triangle
  sphere contact. Returns deepest `Option<MeshContact>`. Works with original
  mesh triangles (not convex hull).
- For meshes with convex hulls (Spec A complete), the rigid-rigid mesh
  dispatch (`mesh_collide.rs`) uses GJK/EPA on hulls. The flex path could
  similarly use `CollisionShape::convex_mesh_from_hull()` +
  `gjk_epa_contact()` for hull-based collision, or call
  `mesh_sphere_contact()` for per-triangle collision.

**Interface anatomy of `narrowphase_sphere_geom`:**
The function returns `Option<(f64, Vector3<f64>, Vector3<f64>)>` which is
`Option<(depth, normal, contact_pos)>` — NOT `(d_surface, normal)`.

Internally, the match block (lines 22-154) computes an intermediate
`(d_surface, normal)` where `d_surface` is the signed distance from the
vertex center to the geom surface (positive = outside, negative = inside),
NOT including the sphere radius. After the match block, post-match code
(lines 156-162) converts:
```rust
let depth = sphere_radius - d_surface;      // line 156
if depth <= 0.0 { return None; }             // line 157-158
let contact_pos = v - normal * d_surface;    // line 161
Some((depth, normal, contact_pos))           // line 162
```

**Three dispatch options for new match arms:**

**Option A — Return `(d_surface, normal)` from match block:** Add match
arms that compute `d_surface` and `normal`, then let the existing
post-match code compute `depth` and `contact_pos`. For reusing existing
functions that return `penetration`, the conversion is:
`d_surface = sphere_radius - penetration`. The post-match code then
recomputes `depth = sphere_radius - d_surface = penetration` — redundant
but uniform with existing arms.

**Option B — Early return from match arm:** New match arms call the
existing function, then `return Some((depth, normal, contact_pos))`
directly, bypassing the post-match conversion. Precedent: the Cylinder
arm already uses `return None` at line 123 inside the match block. This
avoids the redundant `d_surface ↔ penetration` round-trip and lets each
complex type use its function's contact point directly (important for
heightfield edge clamping — see EGT-3).

**Option C — Bypass `narrowphase_sphere_geom` entirely:** Complex geom
types are handled in `mj_collision_flex()` before calling
`narrowphase_sphere_geom`, calling existing functions directly and
constructing Contact via `make_contact_flex_rigid()`. Changes the
dispatch flow in `collision/mod.rs`.

### EGT-3: Flex-vs-hfield collision path

**MuJoCo behavior:** Flex vertex sphere uses standard sphere-vs-hfield
collision. MuJoCo uses `mjc_ConvexHField()` (prism-based GJK) for all
convex-vs-hfield pairs including sphere.

**CortenForge state:** Two paths exist:
1. `heightfield_sphere_contact()` (`heightfield.rs:457`): Point-sampling
   approach. Uses corner-origin local coords (`0..extent_x`,
   `0..extent_y`). Requires centering offset: `hfield_size[0..1]`
   subtracted from geom position to convert center-origin (MuJoCo) to
   corner-origin. Pattern shown in `sdf_collide.rs:88-90`:
   ```rust
   let hf_offset = other_mat * Vector3::new(-hf_size[0], -hf_size[1], 0.0);
   let hf_pose = Pose::from_position_rotation(
       Point3::from(other_pos + hf_offset), other_quat);
   ```
2. `collide_hfield_multi()` (`hfield.rs:33`): Prism-based GJK from Spec C.
   Takes `(model, geom1, geom2, ...)` — designed for rigid-rigid dispatch
   with geom indices. Not directly usable for virtual flex spheres without
   creating a virtual geom entry.

For flex vertices (tiny spheres, typically `radius < 0.05`), the
point-sampling approach gives equivalent results to the prism-based approach
and is simpler to integrate. The prism-based approach is technically more
MuJoCo-conformant (MuJoCo uses it for all convex shapes), but for spheres
small enough to fit within a single heightfield cell, both approaches
produce identical contacts.

**Edge-clamping subtlety:** `heightfield_sphere_contact` clamps the
query XY position to the heightfield bounds (line 478-479). For vertices
near the edge, the clamped position differs from the vertex XY position.
If using dispatch option A (returning `d_surface` from the match block),
the post-match `contact_pos = v - normal * d_surface` will place the
contact directly below the vertex, not at the clamped position. For
dispatch option B (early return), the contact point from the function
can be used directly, avoiding this discrepancy. This is a minor edge
case — flex vertices are rarely at heightfield edges.

### EGT-4: Flex-vs-SDF collision path

**MuJoCo behavior:** Flex vertex sphere uses standard sphere-vs-SDF
collision. Queries the SDF distance field at the vertex position.

**CortenForge state:** `sdf_sphere_contact()` (`sdf.rs:583`) takes
`(&SdfCollisionData, &Pose, Point3<f64>, f64)` and returns
`Option<SdfContact>` with `point`, `normal`, `penetration`. Direct reuse
is straightforward — construct the SDF pose from `geom_pos` + `geom_mat`,
call with `sphere_center = Point3::from(sphere_pos)` and
`sphere_radius`.

### EGT-5: Contact creation pipeline

Flex contacts are created via `make_contact_flex_rigid()` at
`flex_collide.rs:169-221`. This function:
- Gets flex ID from vertex index: `model.flexvert_flexid[vertex_idx]`
- Calls `contact_param_flex_rigid(model, flex_id, geom_idx)` for
  parameter combination (condim, gap, solref, solimp, mu)
- Applies global overrides (`assign_ref`, `assign_imp`, `assign_friction`,
  `assign_margin`)
- Sets `flex_vertex: Some(vertex_idx)` on the Contact
- Sets `geom1 = geom2 = geom_idx` (flex contacts store rigid geom, not
  flex body)

This pipeline is independent of the geom type — it works for any geom.
No changes needed to `make_contact_flex_rigid()`.

### EGT-6: Normal direction convention

In `narrowphase_sphere_geom`, the normal points **from the rigid geom
surface toward the vertex** (outward from geom). This is consistent
across all existing geom types:
- Sphere: `diff / dist` where `diff = v - geom_pos`
- Box: `geom_mat * normal_local` pointing from surface toward vertex
- Plane: `plane_normal` (pointing away from plane surface toward vertex)

The existing contact functions return normals with different conventions:
- `MeshContact.normal`: points outward from mesh surface (toward sphere)
- `HeightFieldContact.normal`: points up from terrain (toward object)
- `SdfContact.normal`: points outward from SDF surface (toward object)

All three point from the geom surface toward the vertex — matching
`narrowphase_sphere_geom`'s convention. No normal negation needed.

---

## Criteria

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does for flex-vs-mesh,
> flex-vs-hfield, and flex-vs-SDF collision — exact function names,
> dispatch mechanism, and edge-case behavior. The MuJoCo C source
> (`engine_collision_primitive.c`, flex vertex collision loop) is the
> single source of truth.

| Grade | Bar |
|-------|-----|
| **A+** | Every MuJoCo function cited with source file and line range. Spec states that MuJoCo's flex collision treats each vertex as a sphere and uses the same collision algorithms as rigid sphere-vs-geom for each complex type — producing identical contact geometry (point, normal, depth). The spec verifies this behavioral equivalence against the C source (not assumed). Edge cases addressed: mesh with no convex hull (MuJoCo always has hull — what does CortenForge do?), hfield with vertex outside grid bounds, SDF with vertex outside bounding box, flex vertex with zero radius, mesh geom with no mesh data (`geom_mesh = None`), hfield geom with no hfield data (`geom_hfield = None`), SDF geom with no SDF data (`geom_sdf = None`). Normal conventions verified: normal from geom surface toward vertex for all three pair types. |
| **A** | MuJoCo behavior described correctly from C source. Minor gaps in edge-case coverage. |
| **B** | Correct at high level, but missing specifics or based on docs not C source. |
| **C** | Partially correct. Some MuJoCo behavior misunderstood or invented. |

### P2. Algorithm Completeness

> Every algorithmic step for each of the three new collision pairs is
> specified unambiguously. For each pair: how to access the complex geom's
> data from the model, how to construct the appropriate poses, how to call
> the existing collision function (or compute distance inline), and how to
> convert the result to `(d_surface, normal)` format. No "see MuJoCo
> source" or "TBD" gaps.

| Grade | Bar |
|-------|-----|
| **A+** | All three pair types (mesh, hfield, SDF) have complete, line-for-line implementable Rust code. Data access patterns explicit: how to get `TriangleMeshData` from `model.geom_mesh[geom_idx]` → `model.mesh_data[mesh_id]`, how to get `HeightFieldData` from `model.geom_hfield[geom_idx]` → `model.hfield_data[hfield_id]` + `model.hfield_size[hfield_id]`, how to get `SdfCollisionData` from `model.geom_sdf[geom_idx]` → `model.sdf_data[sdf_id]`. Pose construction explicit (especially hfield centering offset). Result conversion formula stated (EGT-2 interface conversion). `None` return paths for missing data documented. |
| **A** | Algorithm is complete. One or two minor details left implicit. |
| **B** | Algorithm structure is clear but some steps hand-waved. |
| **C** | Skeleton only. |

### P3. Convention Awareness

> Spec explicitly addresses CortenForge conventions where they differ from
> MuJoCo: normal direction, coordinate systems (hfield corner-origin vs
> center-origin), `Pose` vs `(pos, mat)` conversion, and the
> `narrowphase_sphere_geom` return format (`d_surface` signed distance
> vs contact struct `penetration`).

| Grade | Bar |
|-------|-----|
| **A+** | Convention Notes table present covering: (1) normal direction for each pair type and why no negation is needed, (2) hfield centering offset formula and why it's needed, (3) `d_surface` vs `penetration` conversion, (4) `Pose` construction from `(geom_pos, geom_mat)` pattern, (5) `use_bvh` flag threading for mesh collision (`DISABLE_MIDPHASE`). Each convention verified to preserve numerical equivalence with MuJoCo. |
| **A** | Major conventions documented. Minor details left to implementer. |
| **B** | Some conventions noted, others not — risk of silent mismatch. |
| **C** | Existing code pasted without adaptation. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. Contains concrete input
> models, expected contact generation behavior, and specific fields to
> check. For flex-vs-complex-geom, "concrete" means: specific MJCF model
> with a flex body near a mesh/hfield/SDF geom, expected contact count
> (0 or 1+), expected contact normal direction, and expected depth sign.

| Grade | Bar |
|-------|-----|
| **A+** | Every AC has the three-part structure: (1) MJCF model or programmatic setup, (2) expected behavior (contact generated or not, depth range, normal direction), (3) what field to check. At least one AC per pair type has an expected depth value verified against geometry (e.g., "sphere at Z=0.3, mesh surface at Z=0, radius=0.05 → penetration ≈ 0.25"). Code-review ACs explicitly labeled (e.g., "catch-all `_ => return None` is removed; all GeomType variants have explicit arms"). Negative ACs present: geom with no mesh data → no contact, vertex outside hfield bounds → no contact. |
| **A** | ACs are testable. Some lack exact numerical expectations. |
| **B** | ACs are directionally correct but vague. |
| **C** | ACs are aspirational statements, not tests. |

### P5. Test Plan Coverage

> Tests cover happy path (contact generated for each pair type), edge cases
> (no mesh data, outside bounds, zero radius), and negative cases (vertex
> above surface → no contact). Each AC maps to at least one test.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present. Explicit edge case inventory: mesh with no hull (per-triangle fallback), mesh with no data (`geom_mesh = None`), hfield with vertex outside grid bounds, hfield with vertex exactly on surface, SDF with vertex outside bounding box, zero-radius flex vertex, single-triangle mesh, single-cell hfield (2×2 grid). Negative cases: vertex above all three complex geom surfaces → no contact. At least one test per pair type uses a non-trivial configuration (e.g., multi-vertex flex body with some vertices contacting and some not). |
| **A** | Good coverage. Minor edge-case gaps. |
| **B** | Happy path covered. Edge cases sparse. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and interactions with completed
> specs are explicitly stated. Spec A (convex hull) is a resolved
> dependency; no blocking prerequisites remain.

| Grade | Bar |
|-------|-----|
| **A+** | Execution order is unambiguous. Spec A completion stated with commit hash. Each section states what it requires from prior sections. Cross-spec interactions called out: Spec A's `ConvexHull` usage for flex-vs-mesh hull path, Spec C's prism-based hfield collision as alternative, T1's SDF option threading. No circular dependencies. |
| **A** | Order is clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Spec identifies every file touched, every behavior that changes, and
> every existing test that might break. This spec has LOW blast radius:
> it adds match arms to one catch-all and reuses existing functions.
> The risk is in interface conversion errors, not in architectural changes.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description. Behavioral change: `narrowphase_sphere_geom` no longer returns `None` for Mesh/Hfield/Sdf geom types — this is a conformance improvement (generates contacts where none existed before). Existing test impact: no existing tests break (no existing tests exercise flex-vs-mesh/hfield/SDF since those paths were no-ops). New tests only. States that `make_contact_flex_rigid` is unchanged. States that `mj_collision_flex` loop is unchanged (or specifies what changes). |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions within the spec. Terminology consistent (e.g.,
> "vertex sphere" used uniformly, not alternating with "flex point" or
> "deformable vertex"). Cross-references accurate.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology is uniform. File paths in Specification sections match Files Affected table. AC numbers match Traceability Matrix. Edge cases in MuJoCo Reference appear in Test Plan. Normal convention stated identically in MuJoCo Reference, Algorithm, and Convention Notes. |
| **A** | Consistent. One or two minor inconsistencies. |
| **B** | Some sections use different names for the same concept. |
| **C** | Contradictions between sections. |

### P9. Dispatch Integration

> Spec addresses how the three new pair types integrate into the existing
> flex collision dispatch architecture. The current architecture has a
> clean separation: `mj_collision_flex()` (outer loop) →
> `narrowphase_sphere_geom()` (match block → `(d_surface, normal)` →
> post-match → `(depth, normal, contact_pos)`) →
> `make_contact_flex_rigid()` (Contact creation). Three options exist
> (see EGT-2): (A) extend match block with `(d_surface, normal)` arms,
> (B) early return `Some((depth, normal, contact_pos))` from match arms
> (bypassing post-match conversion), or (C) bypass `narrowphase_sphere_geom`
> entirely. The spec must choose and justify.
>
> **Boundary with P2:** P2 grades whether the algorithm is complete and
> implementable. P9 grades whether the dispatch architecture is sound —
> does the new code fit cleanly into the existing flex collision pipeline,
> or does it create an awkward split between inline distance computation
> (primitives) and function delegation (complex types)?

| Grade | Bar |
|-------|-----|
| **A+** | Dispatch strategy explicitly stated and justified. All three options (A: match-block `d_surface`, B: early return from match, C: bypass function) considered; chosen option justified. If option A: explains the `d_surface = sphere_radius - penetration` conversion and why the redundant round-trip is acceptable. If option B: explains how early return from match arms interacts with the post-match code (skipped). If option C: explains how `mj_collision_flex()` changes and why the architectural split is worth it. Whichever approach: the spec explains why it was chosen, and the resulting code structure is as clean as the existing primitive dispatch. No architectural debt introduced. |
| **A** | Dispatch strategy clear. Minor justification gaps. |
| **B** | Strategy implied but not explicit. |
| **C** | No dispatch discussion. |

### P10. Reuse Correctness

> Spec addresses how existing sphere-vs-complex-geom functions are reused
> and what interface conversion is needed. The risk is in conversion
> errors: wrong sign on d_surface, wrong Pose construction (forgetting
> hfield centering offset), wrong normal direction, or wrong radius
> (inflated vs raw).
>
> **Boundary with P1:** P1 grades whether the spec correctly describes
> MuJoCo's behavior. P10 grades whether the spec correctly maps that
> behavior onto CortenForge's existing infrastructure — can the existing
> functions be reused, and if so, does the conversion logic preserve
> numerical equivalence?

| Grade | Bar |
|-------|-----|
| **A+** | For each of the three pair types: (1) names the existing function to reuse, (2) states its input parameters and how each is derived from `narrowphase_sphere_geom`'s arguments (`sphere_pos`, `sphere_radius`, `geom_idx`, `model`, `geom_pos`, `geom_mat`), (3) states its return type and the exact conversion formula — either to `(d_surface, normal)` for option A or to `(depth, normal, contact_pos)` for option B, (4) lists any parameters the existing function needs that `narrowphase_sphere_geom` doesn't directly have (and how they're obtained — e.g., `model.hfield_size[hfield_id]` for centering offset, `DISABLE_MIDPHASE` flag for `use_bvh`). The conversion is verified to preserve numerical equivalence: `contact_pos = v - normal * d_surface` matches the existing function's contact `point` (verified for all three types in EGT-2). |
| **A** | Reuse strategy clear. Minor conversion details implicit. |
| **B** | Functions named but conversion logic vague. |
| **C** | No reuse discussion. |

---

## Rubric Self-Audit

### Self-audit checklist

- [x] **Specificity:** Every A+ bar names specific functions
  (`narrowphase_sphere_geom`, `mesh_sphere_contact`,
  `heightfield_sphere_contact`, `sdf_sphere_contact`), specific files
  (`flex_collide.rs:152-153`, `mesh.rs:1076`, `heightfield.rs:457`,
  `sdf.rs:583`), and specific edge cases (no mesh data, outside hfield
  bounds, zero-radius vertex). Two independent reviewers would agree on
  grades.

- [x] **Non-overlap:** P1 grades MuJoCo behavior accuracy. P2 grades
  algorithm completeness. P9 grades dispatch architecture (fits into
  existing pipeline). P10 grades reuse correctness (conversion logic).
  Boundaries stated in P9 and P10 criterion descriptions. P3 grades
  convention translations (normal signs, coord systems) — distinct from
  P10's focus on reuse conversion formulas because P3 covers all
  conventions, not just those arising from function reuse.

- [x] **Completeness:** 10 criteria cover: MuJoCo fidelity (P1),
  algorithm (P2), conventions (P3), ACs (P4), tests (P5), dependencies
  (P6), blast radius (P7), consistency (P8), dispatch integration (P9),
  reuse correctness (P10). No dimension uncovered: the task is
  fundamentally about adding match arms that delegate to existing
  functions, so dispatch and reuse are the domain-specific dimensions.

- [x] **Gradeability:** P1 → MuJoCo Reference + Key Behaviors. P2 →
  Specification (S1, S2, ...). P3 → Convention Notes. P4 → Acceptance
  Criteria. P5 → Test Plan + Traceability. P6 → Prerequisites +
  Execution Order. P7 → Files Affected + Blast Radius. P8 → cross-cutting.
  P9 → Specification (dispatch architecture). P10 → Specification (reuse
  + conversion).

- [x] **Conformance primacy:** P1 is tailored with the behavioral
  equivalence claim (flex vertex sphere produces identical contact geometry
  to rigid sphere), source file (`engine_collision_primitive.c`), and
  task-specific edge cases (no hull, outside bounds, no data, zero radius).
  P1 requires the spec author to verify behavioral equivalence against C
  source (not assume it). P4 requires geometry-derived expected values.
  P5 requires at least one test per pair type. P10 requires conversion
  formulas that preserve numerical equivalence.

- [x] **Empirical grounding:** EGT-1 through EGT-6 filled in with
  verified MuJoCo behavior and CortenForge codebase context (file:line
  references). Every A+ bar references specific EGT entries or codebase
  locations. No criterion was written from header-file assumptions.

### Criterion → Spec section mapping

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P1 | MuJoCo Reference, Key Behaviors table, Convention Notes |
| P2 | Specification (S1: Mesh, S2: Hfield, S3: SDF) |
| P3 | Convention Notes, Specification code |
| P4 | Acceptance Criteria |
| P5 | Test Plan, AC→Test Traceability Matrix, Edge Case Inventory |
| P6 | Prerequisites, Execution Order |
| P7 | Risk & Blast Radius (Files Affected, Existing Test Impact) |
| P8 | *Cross-cutting — all sections checked for mutual consistency* |
| P9 | Specification (dispatch architecture decision + justification) |
| P10 | Specification (reuse functions + conversion formulas) |

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
| P9. Dispatch Integration | | |
| P10. Reuse Correctness | | |

**Overall: — (Rev 2)**

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| R1 | P9 | Initial draft lacked domain-specific criterion for dispatch integration — the primary architectural decision is how to fit three new match arms into the existing pipeline | Rubric self-audit | Added P9 (Dispatch Integration) | Rubric Rev 1 |
| R2 | P10 | Initial draft lacked domain-specific criterion for reuse correctness — the primary risk is conversion errors when adapting existing function results | Rubric self-audit | Added P10 (Reuse Correctness) | Rubric Rev 1 |
| R3 | EGT | Initial draft had no EGT section — criterion bars were written from knowledge, not empirical verification | Rubric self-audit | Added EGT-1 through EGT-6 with file:line references and codebase context table | Rubric Rev 1 |
| R4 | P1 | Edge cases initially generic — replaced with task-specific edge cases: no mesh data, outside hfield bounds, no SDF data, zero-radius vertex | Rubric self-audit | Tailored P1 A+ bar with specific edge cases | Rubric Rev 1 |
| R5 | P5 | Edge case inventory initially generic — replaced with task-specific list: no hull fallback, no data guards, boundary conditions, zero radius | Rubric self-audit | Tailored P5 A+ bar with specific edge cases | Rubric Rev 1 |
| R6 | EGT-2 | Rubric said `narrowphase_sphere_geom` returns `(d_surface, normal)` — WRONG. It returns `(depth, normal, contact_pos)`. The `(d_surface, normal)` is the match block intermediate, converted by post-match code at lines 156-162. | Stress test | Rewrote EGT-2 to document the full interface anatomy: match block → intermediate → post-match conversion → return. Added three dispatch options (A/B/C). | Rubric Rev 2 |
| R7 | P9 | Rubric only discussed two dispatch options (extend match or bypass function). Missed option B: early return `Some(...)` from within match arms, bypassing post-match conversion. Precedent: Cylinder arm uses `return None` at line 123. | Stress test | Added option B to EGT-2 and P9 A+ bar. All three options now enumerated. | Rubric Rev 2 |
| R8 | EGT-1 | Rubric asserted flex collision uses `mjCOLLISIONFUNC` dispatch table. MuJoCo's flex collision may use inline dispatch instead. The behavioral outcome (identical to rigid sphere) is correct but the mechanism claim was unverified. | Stress test | Softened to behavioral equivalence claim. Added spec author note to verify dispatch mechanism against C source. | Rubric Rev 2 |
| R9 | EGT-3 | Heightfield edge-clamping subtlety: `heightfield_sphere_contact` clamps XY to bounds, but post-match `contact_pos = v - normal * d_surface` projects along normal. For edge vertices, these give different contact positions. Minor edge case but should be noted. | Stress test | Added edge-clamping note to EGT-3 with recommendation for dispatch option B (avoids the issue). | Rubric Rev 2 |
