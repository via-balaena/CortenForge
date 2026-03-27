# Multi-Contact Collision Analysis

**Date:** 2026-03-27
**Branch:** `feature/integrator-examples` (clean)
**Status:** Recon complete — no code changes

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

The noslip breakage is not a structural flaw — it's a conditioning issue that
will self-resolve if:
- Each contact has its **own correct depth** (computed per-vertex, not shared)
- Contact positions are **distinct** (each at its own corner, not averaged)
- `noslip_iterations` is **sufficient** for the larger subproblem (may need
  increase from current value)

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
| Plane-Box | `mjc_PlaneBox` | 4 | `collide_with_plane` (Box arm) | 1 | **4 corners, not 1** |
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

#### Return type unification needed

All 7 gap functions currently return `Option<Contact>`. The top-level
dispatcher `collide_geoms` (`narrow.rs:63`) already returns `Vec<Contact>`.
Every analytical function must be unified to `Vec<Contact>` so the dispatcher
can aggregate contacts naturally.

### Implementation plan — 6 phases

#### Phase 0: Return type unification

**Scope:** Change ALL analytical collision functions from `Option<Contact>` to
`Vec<Contact>`. This is a mechanical refactor with zero behavioral change —
`Some(c)` becomes `vec![c]`, `None` becomes `vec![]`.

**Files changed:**

| Function | File | Current | New |
|----------|------|---------|-----|
| `collide_with_plane` | `plane.rs:16` | `Option<Contact>` | `Vec<Contact>` |
| `collide_sphere_sphere` | `pair_convex.rs:13` | `Option<Contact>` | `Vec<Contact>` |
| `collide_capsule_capsule` | `pair_convex.rs:66` | `Option<Contact>` | `Vec<Contact>` |
| `collide_sphere_capsule` | `pair_convex.rs:129` | `Option<Contact>` | `Vec<Contact>` |
| `collide_sphere_box` | `pair_convex.rs:219` | `Option<Contact>` | `Vec<Contact>` |
| `collide_cylinder_sphere` | `pair_cylinder.rs:17` | `Option<Contact>` | `Vec<Contact>` |
| `collide_cylinder_capsule` | `pair_cylinder.rs:147` | `Option<Contact>` | `Vec<Contact>` |
| `collide_capsule_box` | `pair_cylinder.rs:247` | `Option<Contact>` | `Vec<Contact>` |
| `collide_box_box` | `pair_cylinder.rs:392` | `Option<Contact>` | `Vec<Contact>` |

**Dispatcher update** (`narrow.rs`): Remove `.into_iter().collect()` wrappers.
Each specialized function's Vec is used directly or concatenated.

**Internal helpers** (`collide_cylinder_plane_impl`, `collide_ellipsoid_plane_impl`):
Also convert to `Vec<Contact>`.

**Behavioral change:** None. All functions still return 0 or 1 contact. Tests
pass without modification.

**Why first:** Every subsequent phase adds multi-contact to functions that
already return `Vec<Contact>`, so the API is stable from Phase 1 onward.

#### Phase 1: Plane multi-contact (all plane-* pairs)

All plane collision pairs upgraded in a single phase, since they share
`collide_with_plane` and MuJoCo's algorithms are straightforward.

##### 1a. Plane-Box → 4 contacts

**Reference:** `mjc_PlaneBox` (`engine_collision_primitive.c:~178`)

**Algorithm:**
1. Iterate all 8 box corners via bitmask (`i = 0..7`):
   - `corner_offset = ±half.x * rx ± half.y * ry ± half.z * rz`
2. Compute `ldist = dot(plane_normal, corner_offset)` (signed distance of
   corner relative to box center along plane normal)
3. **Filter:** keep corner if BOTH:
   - `dist + ldist <= margin` (corner is within margin of plane)
   - `ldist <= 0` (corner is on the bottom side of the box center)
4. Each passing corner gets its **own depth** = `-(dist + ldist)` and its
   **own position** = midpoint between corner and plane surface
5. Hard cap at 4 contacts

**Critical detail:** The `ldist <= 0` filter ensures only the bottom face's
corners are returned (max 4 of 8). Each corner has a DISTINCT depth — this is
what differentiates the Jacobian rows and keeps noslip well-conditioned.

**Current code location:** `plane.rs:80–143`

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

**Algorithm:** MuJoCo returns up to 4 contacts:
1. The deepest rim point (nearest point on the bottom circle to the plane)
2. Up to 3 triangle-sample points on the lower disk (for near-horizontal
   cylinders, these stabilize the resting contact)

**Current code location:** `plane.rs:183–196` → `collide_cylinder_plane_impl`
at `plane.rs:249`. Currently returns 1 contact.

##### 1d. Plane-Mesh → up to 3 contacts

**Reference:** `mjc_PlaneConvex` mesh path

**Algorithm:** MuJoCo walks neighboring vertices from the support vertex
(deepest into the plane) and returns up to `maxplanemesh = 3` contacts below
margin.

**Current code location:** `mesh_collide.rs:259` (`collide_mesh_plane`).
Currently returns 1 (deepest vertex only).

**Note:** Requires access to mesh adjacency data (vertex neighbors). If not
available, fall back to projecting all vertices and keeping the N deepest.

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

**Reference:** `mjraw_CapsuleBox` in MuJoCo

**Algorithm:** MuJoCo performs segment-box closest-point analysis, then tests
2 key points (typically the capsule endpoints projected onto the box). Returns
up to 2 contacts.

**Current code location:** `pair_cylinder.rs:247`. Currently uses 5-point
sampling + refinement → 1 contact.

**Estimated size:** ~80 LOC refactor.

#### Phase 4: Noslip validation and tuning

After Phases 1–3, run the full noslip + solver test suite:

```
cargo test -p sim-core -p sim-conformance-tests
```

**Expected outcome with correct per-corner depths:** Noslip should converge
because:
- Each corner contact has a distinct depth → distinct impedance → distinct
  Jacobian contributions → well-conditioned Delassus submatrix
- The Gauss-Seidel iteration has well-separated eigenvalues

**If slip regresses (defense-in-depth):**

1. **Verify per-corner depths** — print contact depths on the tilted-plane
   test. On a 3° tilt with 0.1m box, depth variation should be ~0.005m.
   If all 4 are identical → bug in depth computation.

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

| Phase | LOC (approx) | Complexity |
|-------|-------------|------------|
| Phase 0: Return type unification | ~100 | Mechanical refactor |
| Phase 1: Plane multi-contact | ~200 | Straightforward (MuJoCo reference clear) |
| Phase 2: Box-box face clipping | ~450 | Most complex (Sutherland-Hodgman) |
| Phase 3: Capsule multi-contact | ~120 | Moderate |
| Phase 4: Noslip validation | ~0 (diagnostic) | Investigation, not code |
| Phase 5: Test updates + new tests | ~300 | Straightforward |
| **Total** | **~1,170** | |

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

## Key Insight: Why the Quick Fix Failed

The quick fix likely produced 4 contacts with **identical or near-identical
depths** (reusing the single lowest-corner depth for all corners, or computing
depth from the box center rather than per-corner).

On a 3° tilted plane with a 0.1×0.1×0.1 box:
- Corner depth variation should be ~0.1 × sin(3°) ≈ 0.005m across the face
- If all 4 get the same depth → same impedance → same Jacobian columns →
  nearly-singular Delassus → noslip diverges
- If each gets its correct depth → distinct impedance values → well-conditioned
  Delassus → noslip converges

**The fix is in the collision detection, not the constraint pipeline.**
