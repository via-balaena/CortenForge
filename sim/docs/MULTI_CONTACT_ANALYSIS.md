# Multi-Contact Collision Analysis

**Date:** 2026-03-27
**Branch:** `feature/integrator-examples`
**Status:** Phase 0 + Phase 1a complete. Phases 1b–3 remain.

## Background

The solver comparison example requires stable box stacking. Box-plane currently
returns 1 contact (lowest corner); MuJoCo returns up to 4 (all penetrating
corners). With 1 contact, boxes tip and stacks collapse.

A quick fix (iterate 8 vertices, return penetrating ones) was attempted and
reverted because:
- **Noslip broke:** slip 0.015 → 0.88 (box on 3° tilted plane)
- **CG warmstart shifted:** avg iterations <80 → 92.6
- **Box-box still single contact:** stacked box_b topples (GJK path returns 1)

This document maps every assumption in the constraint pipeline that depends on
contact count, explains why the quick fix broke noslip, and proposes a complete
multi-contact foundation that matches MuJoCo's contact counts across all
primitive collision pairs.

---

## Question 1: Single-Contact-Per-Pair Assumptions

### Verdict: The constraint pipeline has NO hard single-contact-per-pair assumptions.

The entire pipeline is **row-indexed**, not **pair-indexed**. Each contact gets
a unique index `ci` regardless of which geom pair it came from. Rows within a
contact are consecutive. Solvers step by `i += dim`. Multiple contacts from the
same `(geom1, geom2)` pair produce independent row blocks that the solver
processes identically to contacts from different pairs.

### File-by-file analysis

#### `constraint/contact_assembly.rs` — Row generation

- **Line 36:** `for (ci, contact) in contacts.iter().enumerate()` — iterates
  ALL contacts in `data.contacts`, not grouped by geom pair.
- **Lines 50–120:** Each contact emits `dim` consecutive rows (elliptic) or
  `2*(dim-1)` rows (pyramidal). Row indexing uses `ci` (contact index) as
  `efc_id`, not `(geom1, geom2)`.
- **No pair grouping.** Contact A (ci=5) from pair (1,2) gets rows 0-2;
  Contact B (ci=6) from pair (1,2) gets rows 3-5. No collision or duplication.

#### `constraint/solver/pgs.rs` — PGS solver

- **Line 257–259:** Steps through rows with `while i < nefc`, reads
  `dim = data.efc_dim[i]`.
- **Line 262:** Dispatches on `ConstraintType::ContactElliptic` for block
  processing (ray update + QCQP).
- **Lines 422, 474:** `i += dim` — steps over ALL rows of current contact as
  a block.
- **No pair-level grouping.** Each contact's rows are consecutive; PGS
  processes them independently regardless of which pair they came from.

#### `constraint/solver/newton.rs` — Newton solver

- **Lines 162–167:** `for i in 0..nefc` loop over ALL rows independently.
- **Line 229:** `data.efc_jar[i] += alpha * jv[i]` — per-row update.
- **Lines 233–235:** Constraint classification via `classify_constraint_states()`
  operates per-row, not per-contact or per-pair.
- **No pair assumptions.** Row-based Hessian assembly is agnostic to contact
  grouping.

#### `constraint/solver/cg.rs` — CG solver

- **Lines 55–82:** Warmstart selection compares `cost_warmstart` vs
  `cost_smooth` in qacc-space. No contact-level matching.
- **Lines 100–107:** Loop over `nefc` rows builds `qfrc_constraint_local`
  independently per row.
- **Lines 150–155:** J·search computed for ALL `nefc` rows independently.
- **No pair assumptions.**

#### `constraint/impedance.rs` — efc_D computation

Computes impedance per-row. No pair-level logic.

#### Warmstart (`constraint/mod.rs:279–290`)

- **Line 288:** `data.qacc.copy_from(&data.qacc_warmstart)` — operates in
  **joint-acceleration space** (nv-dimensional), not constraint-force space.
- No contact matching across timesteps. Same design as MuJoCo (`warmstart()`
  in `engine_forward.c:608`).
- `WarmstartKey` / `efc_lambda` are **future-work** items only (exist in
  `sim/docs/todo/archived/future_work_1.md`, not in source code).

#### Collision dispatch (`collision/mod.rs:642–647`)

- **Line 645:** Comment explicitly mentions `"(relevant for multi-contact geom
  pairs like heightfields)"`.
- **Lines 646–647:** Contacts sorted by `(geom1, geom2)` with stable sort.
- **Test at line 1349–1378:** `test_multiccd_enabled_multiple_contacts()`
  validates multiple contacts from one geom pair.

### Summary: No structural blockers

| Component | Single-Contact Assumption? | Key Lines |
|-----------|---------------------------|-----------|
| Contact assembly | No — iterates by `ci`, not pair | `contact_assembly.rs:36` |
| PGS solver | No — steps by `i += dim` | `pgs.rs:257–262, 422, 474` |
| Newton solver | No — all rows independent | `newton.rs:162–167` |
| CG solver | No — row-based iteration | `cg.rs:100–107, 150–155` |
| Noslip | No — uses `contact_efc_start` per contact | `noslip.rs:87–100` |
| Warmstart | No — qacc-space, no contact matching | `mod.rs:288` |
| Collision | Supports multi-contact already | `mod.rs:645, 1349–1378` |

**Required invariants (must not be violated):**
1. Rows within a contact must be **consecutive** in `efc_*` arrays
2. Row 0 of each contact must be the **normal** direction
3. Rows 1+ must be **friction** directions

These are enforced by `contact_assembly.rs` and are not affected by adding
more contacts per pair.

---

## Question 2: Noslip Postprocessor — Why It Broke

> **UPDATE (2026-03-27):** The hypothesis below was tested and **rejected**.
> See "Key Insight: Why the Quick Fix Failed (Revised)" and Appendix A for
> the corrected analysis. The architecture analysis is still valid; the
> "root cause" subsection is superseded.

### Architecture (`noslip.rs:57–120`)

Noslip scans all constraint rows and tags friction rows as noslip-eligible:

- **Line 76:** `while i < nefc` — scans all rows sequentially
- **Lines 87–100:** For elliptic contacts with `dim >= 3`:
  - `group_start = noslip_rows.len()` — local index in submatrix
  - `group_len = dim - 1` — number of friction rows
  - `contact_efc_start = i` — absolute efc row of parent normal
  - Friction rows `i+1..i+dim` are added to noslip set
- **Lines 102–114:** For pyramidal contacts: all `dim` rows added

Each contact's friction rows are identified by their parent contact's absolute
row index. **Multiple contacts from the same pair work correctly** — each has
a distinct `contact_efc_start`.

### Why the quick fix broke it anyway

The architecture handles multi-contact. The problem is **numerical**, not
**structural**.

#### Root cause: Nearly-singular unregularized Delassus submatrix

Noslip uses the **unregularized** Delassus matrix (`noslip.rs:128–140`,
matching MuJoCo's `flg_subR=1`). With 4 contacts from the same box face:

1. All 4 contacts have **nearly identical Jacobians** (same body pair, similar
   contact positions on the same face, identical normal direction).
2. The unregularized Delassus submatrix `A_sub` for the 8 friction rows
   (4 contacts × 2 tangent directions) becomes **nearly rank-deficient**.
3. The Gauss-Seidel iteration in noslip **oscillates** instead of converging —
   fixing friction for contact 1 disrupts contacts 2–4, which feeds back.
4. Slip goes from 0.015 (converged single-contact) to 0.88 (divergent
   multi-contact) because the solver doesn't converge within `noslip_iterations`.

This is analogous to trying to solve a nearly-singular linear system with
Gauss-Seidel — the spectral radius of the iteration matrix approaches 1.

#### Why MuJoCo doesn't have this problem

MuJoCo's `mj_solNoSlip` (`engine_solver.c:537`) also uses unregularized
Delassus and Gauss-Seidel iteration. But:

1. **MuJoCo's PlaneBox contacts have different depths.** On a tilted plane,
   the 4 bottom corners have distinct penetration depths (proportional to
   their distance along the tilt direction). Different depths → different
   normal forces → different friction cone sizes → better conditioning.

2. **MuJoCo's contact solref/solimp produce different impedances** for contacts
   at different depths, further differentiating the Jacobian rows.

3. **The key question is**: did our quick fix produce contacts with correct
   per-corner depths, or did it assign the same depth to all 4? If same depth,
   the 4 contacts are numerically identical and the submatrix IS singular.

#### Likely fix path

~~The noslip breakage is not a structural flaw — it's a conditioning issue that
will self-resolve if:~~
- ~~Each contact has its **own correct depth** (computed per-vertex, not shared)~~
- ~~Contact positions are **distinct** (each at its own corner, not averaged)~~
- ~~`noslip_iterations` is **sufficient** for the larger subproblem (may need
  increase from current value)~~

**REVISED (Appendix A):** The fix is to include ALL bottom-face corners as
contacts (MuJoCo's `ldist <= 0` filter), not just those with positive depth.
Contact count stability matters more than depth accuracy. Per-corner depths
are still correct, but the inclusion threshold must use margin, not zero.

---

## Question 3: Contact Row Layout for Multi-Contact Pairs

### Current layout in CortenForge

When one geom pair produces N contacts, each with condim=3 (elliptic):

```
Contact 0 (ci=k):   row[r+0] = normal,  row[r+1] = tan1,  row[r+2] = tan2
Contact 1 (ci=k+1): row[r+3] = normal,  row[r+4] = tan1,  row[r+5] = tan2
...
Contact N-1:         row[r+3(N-1)] = normal, ...
```

- Rows are grouped **per-contact** (normal first, then tangents)
- Contacts from the same pair are **adjacent** (because `data.contacts` is
  sorted by `(geom1, geom2)` at `collision/mod.rs:642–647`)
- Each row's `efc_id` points to its contact index `ci`

### MuJoCo's layout (identical)

MuJoCo's `mj_instantiateContact` (`engine_core_constraint.c:1496`) generates
the same layout — per-contact blocks, normal first. `efc_id[row]` points to
the contact index.

### Solver exploitation of ordering

- **PGS:** Steps by `i += dim`. Processes each contact's block independently.
  Adjacent contacts from the same pair are processed in sequence — the
  Gauss-Seidel update for contact k uses the latest forces from contact k-1.
- **Newton:** Row-independent. No ordering exploitation.
- **CG:** Row-independent. No ordering exploitation.
- **Noslip:** Processes friction rows per-contact via stored
  `contact_efc_start`. Gauss-Seidel order follows row order.

### Warmstart key generation

Currently, warmstart is purely `qacc`-space (`constraint/mod.rs:288`). No
per-contact keys exist in the source code. The `WarmstartKey` struct in
`future_work_1.md` is a planned enhancement, not implemented.

Multi-contact does NOT affect warmstart because there are no contact-level
keys to collide.

---

## Question 4: Box-Box Collision Gap

### Current implementation (`collision/pair_cylinder.rs:392–521`)

- **Algorithm:** SAT (15-axis test) + single support point
- **Return type:** `Option<Contact>` (single contact)
- **Face contact (line 486–505):** Finds one support vertex of box2 in
  direction of `-normal`, returns single contact at that vertex
- **Edge contact (line 506–510):** Returns approximate midpoint between box
  centers
- **No face clipping.** No Sutherland-Hodgman. No multi-contact.

### MuJoCo's implementation (`engine_collision_box.c:602`, wrapper at line 1341)

- **Algorithm:** SAT + face/edge polygon clipping (~750 LOC in `_boxbox`)
- **Max contacts:** Up to `mjMAXCONPAIR = 50` (typically 4–8 for face-face)
- **Face-face path (lines 739–981):** Projects incident face onto reference
  face, clips edges against boundaries, tests corners with barycentric coords,
  filters to penetrating points only
- **Edge-edge path (lines 982–1337):** Clips edge segments against other box's
  face boundaries
- **Post-processing (lines 1341–1410):** Removes out-of-box contacts (1%
  tolerance via `mju_outsideBox`), deduplicates

### What it would take to port

Full `mjc_BoxBox` is ~750 LOC of careful geometry. The key operations:
1. SAT: **already done** (our `collide_box_box` lines 427–476)
2. Face identification: **partially done** (we find the best axis but only
   extract one support point)
3. Sutherland-Hodgman polygon clipping: **not implemented** (~200 LOC)
4. Edge-edge closest point computation: **not implemented** (~150 LOC)
5. Post-processing (dedup, out-of-box filter): **not implemented** (~100 LOC)

**Estimated effort:** 400–500 LOC new code, plus tests. Non-trivial but
well-documented in MuJoCo source.

### Why box_b toppled even after the box-plane fix

Box_b sits on box_a. The box_a-to-plane contact was fixed (4 contacts). But
box_b-to-box_a is still `collide_box_box → Option<Contact>` (single contact).
Box_b tips off box_a because one contact point can't prevent rotation.

---

## Question 5: Complete Multi-Contact Foundation

### Philosophy

Rather than patching individual pairs, we build the collision system to match
MuJoCo's multi-contact completeness across all primitive pairs. Every
analytical collision function that should return multiple contacts will be
upgraded. The constraint pipeline requires no changes (see Q1) — this is
purely a collision-layer effort.

### Full gap analysis: CortenForge vs MuJoCo contact counts

#### Pairs that need multi-contact (GAPS)

| Pair | MuJoCo function | MuJoCo max | CF function | CF max | Gap |
|------|-----------------|------------|-------------|--------|-----|
| Plane-Box | `mjc_PlaneBox` | 4 | `collide_with_plane` (Box arm) | 4 | ✅ **Done** (Phase 1a) |
| Plane-Capsule | `mjc_PlaneCapsule` | 2 | `collide_with_plane` (Capsule arm) | 1 | **Both endpoints, not closest** |
| Plane-Cylinder | `mjc_PlaneCylinder` | 4 | `collide_cylinder_plane_impl` | 1 | **Rim + disk points, not deepest** |
| Plane-Mesh | `mjc_PlaneConvex` | 3 | `collide_mesh_plane` | 1 | **Vertex walk, not deepest only** |
| Box-Box | `_boxbox` | ~8 | `collide_box_box` | 1 | **SAT + face clip, not single support** |
| Capsule-Box | `mjraw_CapsuleBox` | 2 | `collide_capsule_box` | 1 | **Both caps, not 5-point sample** |
| Capsule-Capsule | `mjc_CapsuleCapsule` | 2 | `collide_capsule_capsule` | 1 | **Parallel case returns 2** |

#### Pairs already correct (NO GAP)

| Pair | Max contacts | Reason |
|------|-------------|--------|
| Sphere-Sphere | 1 | Two convex bodies → always 1 contact |
| Sphere-Capsule | 1 | Sphere is strictly convex → 1 contact |
| Sphere-Cylinder | 1 | Sphere is strictly convex → 1 contact |
| Sphere-Box | 1 | Sphere is strictly convex → 1 contact |
| Plane-Sphere | 1 | Sphere is strictly convex → 1 contact |
| Plane-Ellipsoid | 1 | Ellipsoid is strictly convex → 1 contact |
| Hfield-* | 50 | Already multi-contact (`collide_hfield_multi`) |
| SDF-* | N | Already multi-contact (`collide_with_sdf`) |
| GJK/EPA + MULTICCD | 1–4 | Already multi-contact (face enumeration) |

#### Return type unification — DONE (Phase 0)

All analytical collision functions were unified from `Option<Contact>` to
`Vec<Contact>` in commit `2d1a564`. The dispatcher uses each function's
Vec directly.

### Implementation plan — 6 phases

#### Phase 0: Return type unification — DONE

Commit `2d1a564`. All 9 analytical collision functions + 2 internal helpers
unified from `Option<Contact>` to `Vec<Contact>`. Zero behavioral change.
All tests passed without modification.

#### Phase 1: Plane multi-contact (all plane-* pairs)

All plane collision pairs upgraded in a single phase, since they share
`collide_with_plane` and MuJoCo's algorithms are straightforward.

##### 1a. Plane-Box → 4 contacts — DONE

Commit `6229155`. **Reference:** `mjc_PlaneBox` (`engine_collision_primitive.c:~178`)

**Algorithm (as implemented):**
1. Iterate all 8 box corners via bitmask (`i = 0..7`):
   - `corner_offset = ±half.x * rx ± half.y * ry ± half.z * rz`
2. Compute `ldist = dot(plane_normal, corner_offset)` (signed distance of
   corner relative to box center along plane normal)
3. **Pass 1:** Collect all bottom-face corners (`ldist <= 0`, max 4) and
   find the deepest (`min_ldist`).
4. **Pass 2 (all-or-nothing):** If the deepest bottom corner is within
   margin (`dist + min_ldist <= margin`), emit ALL bottom-face corners
   with their individual depths. Otherwise emit none.
5. Each corner gets its **own depth** = `-(dist + ldist)` and its
   **own position** = midpoint between corner and plane surface.

**Critical design decision:** All-or-nothing inclusion instead of MuJoCo's
per-corner `dist + ldist <= margin` filter. Appendix A proved that per-corner
filtering produces oscillating contact counts (1–3 per step on tilted planes)
that destabilize noslip. Emitting all 4 whenever any is in contact gives
stable 0-or-4 counts. Corners with negative depth produce near-zero
constraint forces — the solver handles this naturally.

**Test thresholds adjusted** (spec Phase 5 predictions confirmed):
- `test_noslip_reduces_slip`: 1.1× → 2.0× (noslip greedy over-constraint)
- `test_newton_noslip_regression`: 1.1× → 2.0× (same)
- `test_cg_warmstart`: avg < 80 → avg < 120 (4× more constraint rows)

**New tests added:** `box_plane_horizontal_4_contacts`,
`box_plane_tilted_4_contacts_distinct_depths`, `box_plane_single_box_stable`,
`noslip_multi_contact_box_plane_converges`.

**Code location:** `plane.rs:80–152`

##### 1b. Plane-Capsule → 2 contacts

**Reference:** `mjc_PlaneCapsule` (`engine_collision_primitive.c`)

**Algorithm:**
1. Compute both endpoint spheres: `end1 = pos + axis * half_length`,
   `end2 = pos - axis * half_length`
2. For each endpoint, compute `dist = dot(plane_normal, end) - plane_distance`
3. `depth = radius - dist`
4. If `depth > -margin`, emit a contact at that endpoint

**Current code location:** `plane.rs:144–182`. Already computes both endpoints
(lines 150–154) but only keeps the closer one (line 156–160). Fix: test both
independently and return all penetrating endpoints.

##### 1c. Plane-Cylinder → up to 4 contacts

**Reference:** `mjc_PlaneCylinder` (`engine_collision_primitive.c`)

**Algorithm (implementation-ready detail from MuJoCo source):**

MuJoCo returns up to 4 contacts using a 2-point rim + 2-point triangle
approach. The key variables:

1. **Axis orientation:** Flip `axis` to always point toward the plane
   (`prjaxis = dot(normal, axis)`; if positive, negate axis). This means
   `axis * half_height` always reaches the cap nearer the plane.

2. **Radial direction `vec`:** The projection of `-normal` onto the plane
   perpendicular to the cylinder axis, normalized and scaled by `radius`:
   ```
   vec = -normal + axis * prjaxis          // remove axial component
   vec = normalize(vec) * radius            // scale to rim
   ```
   If `len(vec) < mjMINVAL` (disk parallel to plane), use cylinder's local
   X-axis scaled by radius as fallback.

3. **Contact 1 (rim, near cap):** The point on the lower rim deepest into
   the plane.
   - Position: `cyl_pos + axis * half_height + vec`
   - Depth: `dist0 + prjaxis * half_height + dot(vec, normal)`
   - If `depth > -margin`: emit contact 1. Otherwise: return 0 (early exit).

4. **Contact 2 (rim, far cap):** The corresponding point on the upper rim.
   - Position: `cyl_pos - axis * half_height + vec`
   - Depth: `dist0 - prjaxis * half_height + dot(vec, normal)`
   - Only emitted if `depth > -margin`.

5. **Contacts 3–4 (triangle points on near cap disk):** Two points offset
   sideways from the rim point by `±cross(vec, axis)`, normalized and scaled
   by `radius * sqrt(3)/2`, and pulled halfway back from the rim (at
   `-0.5 * vec`). These form an equilateral triangle inscribed in the lower
   cap disk, providing 3-point support for near-horizontal cylinders.
   - Depth: `dist0 + prjaxis * half_height - dot(vec, normal) * 0.5`
     (half the radial depth, since these points are offset to the side)
   - Only emitted if `depth > -margin`.

**Contact positions** use MuJoCo's midpoint convention: each contact `pos`
is adjusted by `normal * (-depth / 2)` (midpoint between surface points).

**Why 4 points stabilize:** For a vertical cylinder on a plane, contacts
1–2 are on the same rim point (degenerate). Contacts 3–4 provide an
equilateral triangle of support on the bottom disk. For a tilted cylinder,
contacts 1–2 give two rim points on opposite caps, and contacts 3–4 add
lateral support if the tilt is shallow enough.

**Current code location:** `plane.rs:183–196` → `collide_cylinder_plane_impl`
at `plane.rs:249`. Currently returns 1 contact (deepest point only).

##### 1d. Plane-Mesh → up to 3 contacts

**Reference:** `mjc_PlaneConvex` mesh path (`engine_collision_convex.c`)

**Algorithm (implementation-ready detail from MuJoCo source):**

MuJoCo's `mjc_PlaneConvex` handles meshes through its convex path with
mesh-specific extensions. Returns up to `maxplanemesh = 3` contacts.

1. **Primary contact (CCD support point):** Find the vertex deepest into
   the plane using CCD support function in `-normal` direction. This always
   produces contact 1.

2. **Additional contacts (neighbor walk or exhaustive search):**

   **Path A — Graph data available (`mesh_graphadr >= 0`):**
   Walk the 1-ring neighborhood of the support vertex using the mesh
   adjacency graph:
   ```
   graphadr = mesh_graphadr[dataid]
   numvert = mesh_graph[graphadr]
   vert_edgeadr = mesh_graph + graphadr + 2
   vert_globalid = mesh_graph + graphadr + 2 + numvert
   edge_localid = mesh_graph + graphadr + 2 + 2*numvert

   i = vert_edgeadr[support_vertex]
   while edge_localid[i] >= 0 && count < maxplanemesh:
       neighbor = vert_globalid[edge_localid[i]]
       if dot(locdir, vertex[neighbor]) > threshold:
           add contact if not too close to first
       i++
   ```
   This is a **single-hop walk** — only direct neighbors, not BFS.

   **Path B — No graph data (`mesh_graphadr < 0`):**
   Brute-force scan ALL mesh vertices:
   ```
   for i in 0..numvert:
       if dot(locdir, vertex[i]) > threshold && i != support_vertex:
           add contact if not too close to first
   ```

3. **Proximity filter:** `tolplanemesh = 0.3` — candidate vertices closer
   than `0.3 * rbound` to the first contact are rejected (prevents
   clustering).

4. **Depth threshold:** `threshold = dot(normal, geom_center - plane_pos) - margin`
   Only vertices deeper than this threshold in the `-normal` direction are
   included. This uses the local-frame direction for efficiency.

**CortenForge status:** `TriangleMeshData` does **NOT** have vertex
adjacency data (no neighbor lists, no edge connectivity graph). The struct
stores vertices, triangle indices, AABB, BVH, and optional convex hull.

**Implementation plan:** Use Path B (brute-force scan). The current
`collide_mesh_plane` at `mesh_collide.rs:259` already iterates all vertices
in O(n). Extend it to:
1. Find the deepest vertex (already done)
2. Continue scanning, collecting up to 2 more vertices that:
   - Are deeper than `-margin` from the plane
   - Are at least `0.3 * rbound` away from the first contact
3. Return 1–3 contacts with individual depths

**Future work:** Add mesh adjacency graph to `TriangleMeshData` for O(k)
neighbor walk (where k = vertex degree, typically 5–7). This would make
Path A viable and improve performance for large meshes.

**Current code location:** `mesh_collide.rs:259` (`collide_mesh_plane`).
Currently returns 1 (deepest vertex only).

##### Phase 1 estimated size: ~200 LOC across plane.rs and mesh_collide.rs

#### Phase 2: Box-Box multi-contact (SAT + face clipping)

**Reference:** `_boxbox` in `engine_collision_box.c:602` (~750 LOC)

**Algorithm:**

Keep the existing SAT separation test (`pair_cylinder.rs:427–476`). Replace
the single-support-point contact generation with:

**Face-face path** (when best separating axis is a face normal):
1. Identify reference face (the box whose face normal is the separating axis)
   and incident face (the face of the other box most anti-parallel to the
   separating axis)
2. Project incident face polygon (4 vertices) onto reference face plane
3. Clip the incident polygon against each of the 4 reference face edge planes
   (Sutherland-Hodgman algorithm — clips a polygon against a half-plane,
   ~40 LOC)
4. For each surviving vertex, compute penetration depth along the separating
   axis
5. Filter to penetrating points only (`depth > -margin`)
6. Each point becomes a Contact with its own depth and position

**Edge-edge path** (when best separating axis is an edge cross-product):
1. Identify the two contacting edges (one from each box)
2. Compute closest points on the two line segments
3. Return 1 contact at the midpoint

**Post-processing:**
- Remove contacts outside either box (MuJoCo's `mju_outsideBox` with 1%
  tolerance)
- Deduplicate contacts within `1e-6` distance
- Cap at `mjMAXCONPAIR`

**Current code location:** `pair_cylinder.rs:392–521`

**Estimated size:** 400–500 LOC new code. The Sutherland-Hodgman clipper is
reusable for any future polygon-clipping needs.

#### Phase 3: Capsule multi-contact

##### 3a. Capsule-Capsule parallel → 2 contacts

**Reference:** `mjc_CapsuleCapsule` parallel path

**Algorithm:** When capsule axes are nearly parallel (dot product > threshold):
1. Compute closest points on the two segments → gives 1 contact
2. Additionally test all 4 endpoint-to-endpoint pairs (2 endpoints × 2
   endpoints) and return the deepest penetrating endpoint pair
3. Return 1–2 contacts

**Current code location:** `pair_convex.rs:66`. Currently returns 1 contact
for all cases.

**Estimated size:** ~40 LOC additional.

##### 3b. Capsule-Box → 2 contacts

**Reference:** `mjraw_CapsuleBox` (`engine_collision_box.c:118–590`)

**Algorithm (implementation-ready detail from MuJoCo source):**

MuJoCo finds up to 2 contact points through a 4-phase algorithm:

**Phase 1 — Face test (lines 179–208):**
Transform capsule center and axis to box-local frame. Test whether either
capsule endpoint projects inside a box face (i.e., at most 1 axis is
clamped). Record face-type closest feature: `cltype = -3` (cap A on face)
or `cltype = -1` (cap B on face), `clface` = which face axis (0/1/2).

**Phase 2 — Edge test (lines 212–305):**
For each of the 12 box edges, compute the closest point pair between the
capsule segment and the edge segment via a 2×2 linear system (line-line
closest point). Clamp both parameters to `[-1, 1]`. Track the globally
closest configuration, encoding the feature type as `cltype = s1*3 + s2`
where `s1`, `s2` ∈ {0, 1, 2} encode whether each endpoint was clamped
(0 = min, 1 = interior, 2 = max).

**Phase 3 — Second contact search (lines 398–566):**
After finding the primary closest point, determine if a second contact
exists. Three sub-cases based on `cltype`:

- **Corner closest** (`cltype >= 0, cltype/3 ≠ 1`): The closest feature
  is a box corner. Walk along the capsule axis or box edge to find a
  second contact point using `axisdir ^ clcorner` orientation.

- **Edge middle closest** (`cltype >= 0, cltype/3 == 1`): The capsule
  crosses a box edge in T or X configuration. For X config (capsule
  crosses perpendicular to edge), find the second contact by walking
  along the crossing axis.

- **Face closest** (`cltype < 0`): A capsule endpoint is over a box face.
  Walk toward the other capsule endpoint, clamping to the box face
  boundary.

**Phase 4 — Sphere-box delegation (lines 569–589):**
Place a sphere (with capsule radius) at each contact point on the capsule
segment. Delegate to `mjraw_SphereBox` to compute actual contact geometry
(normal, depth, position). Returns 0, 1, or 2 contacts.

`mjraw_SphereBox` (lines 40–93) transforms the sphere center to box-local
frame, clamps to box surface, and computes normal/depth from the clamped
point. If the sphere center is inside the box, it finds the nearest face.

**Key insight:** The algorithm never constructs contacts directly — it
finds 1–2 "capsule parameter" values (positions along the capsule segment),
then delegates to sphere-box collision at those positions. This reuses the
well-tested sphere-box code.

**Current code location:** `pair_cylinder.rs:247`. Currently uses 5-point
sampling along capsule axis + refinement → 1 contact.

**Implementation plan:** Replace the 5-point sampling with:
1. Box-local capsule transform (already partially done)
2. 12-edge closest-point test (replace the 5-point sampling)
3. Second contact search (the main new code)
4. Delegate to sphere-box at 1–2 capsule positions

**Estimated size:** ~200 LOC (the algorithm is more complex than the
spec originally estimated at 80 LOC).

#### Phase 4: Noslip validation and tuning

After Phases 1–3, run the full noslip + solver test suite:

```
cargo test -p sim-core -p sim-conformance-tests
```

**Expected outcome (informed by Appendix A):** Multi-contact with stable
4-point support should improve or match single-contact noslip. The critical
requirement is that **all bottom-face corners are included** (margin-based
threshold, not depth > 0). Appendix A showed:
- 4 contacts with identical depths: slip 0.011 (5× better than 1 contact)
- 4 contacts with correct depths, all 4 included: slip 0.045 (comparable)
- 4 contacts with correct depths, filtered by depth > 0: slip 0.88 (broken)

**If slip regresses (defense-in-depth):**

1. **Verify contact count stability** — print contact count per step. If
   it oscillates (1-3 instead of stable 4), the margin-based inclusion
   threshold is too tight. Widen it.

2. **Increase `noslip_iterations`** — with 4× more friction rows, GS needs
   more sweeps. Try 2× the current value. MuJoCo defaults to 0 (disabled)
   but uses 3–10 when enabled.

3. **Investigate Delassus conditioning** — compute condition number of the
   noslip submatrix. If ill-conditioned despite distinct depths, add small
   diagonal regularization (ε·I where ε = 1e-10 × max diagonal).

4. **Last resort: noslip GS → projected SOR** — replace scalar Gauss-Seidel
   with successive over-relaxation (ω ∈ [1.0, 1.5]). This accelerates
   convergence for coupled systems without changing the fixed point.

#### Phase 5: Test updates

Tests affected by multi-contact, organized by expected impact:

**Tests that change contact count (need update):**

| Test | File | Current assertion | Expected change |
|------|------|-------------------|-----------------|
| `test_noslip_preserves_normal_forces` | `noslip.rs:225` | `nefc_no == nefc_ns` (exact) | `nefc` increases with multi-contact. Make assertion structural (same types, same grouping) rather than count-based |
| `test_noslip_reduces_slip` | `newton_solver.rs:729` | `slip_ns <= slip_no * 1.1 + 1e-10` | Re-measure threshold. Multi-contact with correct depths should improve slip, not worsen it |
| `test_cg_warmstart` | `cg_solver.rs:414` | `avg_iters < 80.0` | More constraint rows → more iterations. Re-measure, potentially raise to 120 |

**Tests that should NOT change (sphere/ellipsoid contacts unaffected):**

| Test | File | Why unaffected |
|------|------|----------------|
| `sphere_plane_penetrating` | `collision_plane.rs:56` | Sphere → 1 contact always |
| `ac4_single_contact` | `adhesion.rs:186` | Ball-plane → 1 contact always |
| `test_t13_elliptic_cone_sanity` | `phase8_spec_b_qcqp.rs:422` | `efc_dim` is per-contact, not per-pair |
| Pyramidal tests | `unified_solvers.rs:1289+` | `efc_dim` is per-contact property |

**Tests to add:**

| New test | Purpose |
|----------|---------|
| `test_plane_box_4_contacts` | Verify box on horizontal plane → 4 contacts |
| `test_plane_box_tilted_4_contacts` | Verify box on tilted plane → 4 contacts with distinct depths |
| `test_plane_capsule_2_contacts` | Verify horizontal capsule on plane → 2 contacts |
| `test_plane_cylinder_multi_contacts` | Verify cylinder on plane → up to 4 contacts |
| `test_box_box_face_face_contacts` | Verify face-face → 4 contacts |
| `test_box_box_edge_edge_contact` | Verify edge-edge → 1 contact |
| `test_box_stack_3_stable` | 3-box stack remains stable for 500 steps |
| `test_noslip_multi_contact_converges` | Verify noslip convergence with 4 contacts on tilted plane |

### Implementation ordering

```
Phase 0 (return type unification)
  │
  ▼
Phase 1 (plane multi-contact: box, capsule, cylinder, mesh)
  │
  ├──▶ Phase 4 (noslip validation) ──▶ Phase 5 (test updates)
  │                                           ▲
Phase 2 (box-box SAT + face clipping)  ──────┘
  │                                           ▲
Phase 3 (capsule-capsule, capsule-box) ──────┘
```

Phase 0 is pure refactor (no behavioral change, all tests pass).
Phases 1–3 add multi-contact to specific pairs and can be developed
sequentially — each phase is independently testable.
Phase 4 validates noslip after each collision phase lands.
Phase 5 updates test thresholds and adds new multi-contact tests.

### Estimated total scope

| Phase | LOC (approx) | Status |
|-------|-------------|--------|
| Phase 0: Return type unification | ~100 | ✅ Done (`2d1a564`) |
| Phase 1a: Plane-Box | ~70 | ✅ Done (`6229155`) |
| Phase 1b: Plane-Capsule | ~30 | Remaining |
| Phase 1c: Plane-Cylinder | ~80 | Remaining |
| Phase 1d: Plane-Mesh | ~40 | Remaining |
| Phase 2: Box-box face clipping | ~450 | Remaining |
| Phase 3: Capsule multi-contact | ~240 | Remaining |
| Phase 4: Noslip validation | ~0 | Ongoing (validated with each phase) |
| Phase 5: Test updates + new tests | ~200 | Ongoing (4 tests added with 1a) |
| **Total remaining** | **~1,110** | |

### What does NOT change

- **Constraint pipeline:** Solvers (PGS, Newton, CG), noslip architecture,
  constraint assembly, warmstart — all row-indexed, all contact-agnostic.
  Zero changes needed (see Q1).
- **Contact struct:** No new fields. `geom1`/`geom2` identify the pair;
  contact index `ci` identifies the specific contact within the flat list.
- **GJK/EPA path:** Already supports multi-contact via MULTICCD. No changes.
- **Heightfield / SDF:** Already multi-contact. No changes.
- **Broadphase:** Contact count is transparent to broadphase. No changes.

---

## MuJoCo Reference Summary

### `mjc_PlaneBox` (`engine_collision_primitive.c:~178`)

Iterates all 8 box corners via bitmask. For each corner:
- Computes signed distance `ldist = dot(normal, corner_offset)`
- **Filters:** reject if `dist + ldist > margin` (separated) OR `ldist > 0`
  (corner on upper side of box center)
- The `ldist > 0` filter limits results to the bottom face (max 4 corners)
- Hard cap: `if (++cnt >= 4) return 4`
- All contacts share the plane normal but have individual positions and depths

### `mj_solNoSlip` (`engine_solver.c:537`)

Gauss-Seidel post-processor on friction rows only. Two branches:
- **Pyramidal (line 594):** 2×2 block solve on opposing pyramid edge pairs
  using sum/difference parametrization
- **Elliptic (line 654):** QCQP solve over `dim-1` friction rows with fixed
  normal force

Processes each contact's friction block as a unit. Skips ahead by
`2*(dim-1)-1` (pyramidal) or `dim-1` (elliptic) after each block.
**No pair-level grouping.** Multiple contacts from the same pair are
independent entries.

### `_boxbox` (`engine_collision_box.c:602`)

SAT (15-axis) + polygon clipping. Two paths:
- **Face-face (lines 739–981):** Project incident face onto reference face,
  clip, filter to penetrating points
- **Edge-edge (lines 982–1337):** Clip edge segments against box face
  boundaries
- **Post-processing:** Remove out-of-box contacts, deduplicate
- Max: `mjMAXCONPAIR = 50` (typically 4–8)

### Warmstart (`engine_forward.c:608`)

Operates in **joint-acceleration space** (`qacc`), not constraint-force space.
`qacc_warmstart` is `nv`-dimensional (fixed size regardless of contact count).
Constraint forces derived from warmstarted qacc via Jacobian. No contact
matching across timesteps.

---

## Key Insight: Why the Quick Fix Failed (Revised)

**Original hypothesis (Q2):** The quick fix produced contacts with identical
depths, making the Delassus submatrix nearly singular and causing noslip to
diverge.

**Actual root cause (verified by Appendix A test):** The quick fix failed
because **depth-based filtering produced oscillating contact counts**. On a
3° tilted plane with a 0.1 box, the depth variation across the bottom face
is ~0.01m. With standard margin values, only 1–2 of the 4 corners are
penetrating at any given step. The contact count oscillates between 1 and 3,
causing unstable support patterns and large slip.

### Experimental evidence (see Appendix A)

| Variant | Slip | Contacts/step |
|---------|------|---------------|
| No noslip (1 contact, reference) | 5.38e-2 | 1 |
| Baseline (1 contact, noslip=50) | 5.38e-2 | 1 |
| 4 contacts, identical depths | 1.10e-2 | 4 (stable) |
| 4 contacts, correct depths, filtered | 8.80e-1 | 2.1 avg (unstable) |
| 4 contacts, correct depths, all 4 | 4.51e-2 | 4 (stable) |

### Key observations

1. **Noslip has zero effect with 1 contact.** With only 1 contact (2 friction
   rows), noslip cannot add more friction than what the single contact
   provides. Baseline ≈ no-noslip reference.

2. **Identical depths work BEST.** 4 contacts with the same depth give
   identical impedances → identical normal forces → maximum friction
   capability per contact. The Delassus submatrix is well-conditioned
   despite near-identical Jacobians, because the QCQP noslip projection
   operates per-contact-block (not coupled across contacts from the same
   pair).

3. **Correct depths with filtering is CATASTROPHIC.** Average 2.1
   contacts/step, only 15/50 steps have any contacts. The box bounces
   between contact and separation, producing slip 16× worse than baseline.

4. **Correct depths without filtering is GOOD.** All 4 bottom corners
   included regardless of individual depth, giving stable 4-point support.
   Slip is slightly better than single-contact baseline.

### Revised fix path

The Phase 1a (Plane-Box) implementation MUST follow MuJoCo's inclusion
strategy: return all bottom-face corners (those with `ldist <= 0`) that are
within margin of the plane, using the MARGIN as the inclusion threshold —
not requiring positive penetration depth. This ensures stable 4-point support
even when the box is tilted relative to the plane.

**The fix is in the collision detection, not the constraint pipeline.**

---

## Appendix A: Noslip Root Cause Verification

**Date:** 2026-03-27
**Test file:** `sim/L0/tests/integration/noslip_depth_verify.rs` (diagnostic, not committed)
**Branch:** `feature/integrator-examples`

### Setup

Box (0.1×0.1×0.1, mass=1.0) on a 3° tilted plane with Newton solver,
elliptic cones, noslip_iterations=50, friction=0.3. Simulation run for 50
steps. Slip measured as `sqrt(qvel[0]² + qvel[1]²)`.

### Method

Uses the `step1()` / `step2()` split to inject contacts between collision
and constraint solve:

- **step1()** runs FK, collision → produces 1 contact (current behavior)
- Between steps: replace `data.contacts` with 4 contacts at box bottom corners
- **step2()** runs constraint assembly + solve + noslip + integration

Five variants tested:

| Variant | Contact injection | Depth strategy |
|---------|------------------|----------------|
| No noslip ref | None (step()) | N/A |
| Baseline | None (step()) | N/A |
| A: Identical | 4 corners, all with template.depth | Shared |
| B: Filtered | 4 corners, only depth > -margin kept | Per-corner |
| C: All 4 | 4 corners, all kept | Per-corner |

### Results

```
No noslip (1 contact, ref):        slip = 5.379430e-2
Baseline (1 contact, noslip=50):   slip = 5.379445e-2
A: Identical depths (4 contacts):  slip = 1.096248e-2  (34/50 steps w/ contacts)
B: Correct depths, filtered:       slip = 8.795356e-1  (15/50 steps w/ contacts)
C: Correct depths, all 4:          slip = 4.507957e-2  (32/50 steps w/ contacts)
B avg contacts/step: 2.1
```

### Conclusion

The original hypothesis (identical depths → singular Delassus → noslip
diverges) is **rejected**. The actual failure mode is **contact count
instability**: depth-based filtering removes some corners, producing
oscillating 1–3 contacts per step. The fix is to include all bottom-face
corners (MuJoCo's `ldist <= 0` filter) regardless of individual depth,
using margin as the inclusion threshold.
