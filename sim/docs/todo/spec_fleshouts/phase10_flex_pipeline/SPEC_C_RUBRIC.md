# Spec C — Flex Self-Collision Dispatch: Spec Quality Rubric

Grades the Spec C spec on 11 criteria. Target: A+ on every criterion
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

Empirical verification against the MuJoCo C source documentation and the
CortenForge codebase discovered the following scope corrections from the
umbrella/session plan claims.

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| "Add `flex_internal: Vec<bool>` to Model" | `flex_internal` already exists on Model (Phase 7 T1, commit `cf76731`). Parsed from MJCF `<flex internal="..."/>`, stored and wired. | **Drop** — field already exists. Spec C only needs to *consume* it in dispatch. |
| `flex_selfcollide` upgrade from `Vec<bool>` to `Vec<FlexSelfCollide>` | Current: `Vec<bool>` (builder/flex.rs:414-415 converts `Option<String>` → `bool`). MuJoCo uses `mjtFlexSelf` enum (5 values). The bool is lossy — `"bvh"`, `"sap"`, `"narrow"`, `"auto"` all collapse to `true`. | **In scope** — upgrade to `Vec<FlexSelfCollide>` enum, fix parser/builder pipeline. |
| "BVH: Per-element AABB tree" | BVH infrastructure exists in `mid_phase.rs`: `Bvh`, `BvhPrimitive`, `query()`, `BvhPrimitive::from_triangle()`. Can be reused for flex element AABBs. | **In scope** — reuse existing BVH infrastructure. |
| "SAP: Sweep-and-prune on element AABBs" | No general-purpose SAP infrastructure exists for element-level AABBs. The existing SAP in the collision pipeline is rigid-body broadphase. MuJoCo's flex SAP is a simple axis-sweep on element AABBs. | **In scope** — implement SAP for flex element AABBs. |
| Triangle-triangle narrowphase for dim=2 | `triangle_triangle_intersection()` exists in `mesh.rs:390` (SAT-based, returns `TriTriContact`). Can be reused for flex element-element collision. | **In scope** — reuse existing triangle-triangle primitive. |
| Tetrahedron-tetrahedron narrowphase for dim=3 | No tet-tet intersection exists. MuJoCo uses vertex-face + edge-edge tests for tet pairs. | **In scope** — implement tet-tet narrowphase. |
| "Element-element contact generation" | MuJoCo uses vertex-face tests for flex self-collision: each vertex of element A tested against faces of element B (and vice versa). For dim=2, this is vertex-triangle. For dim=3, vertex-tetrahedron-face + edge-edge. | **In scope** — spec must clarify exact narrowphase approach per dim. |
| `activelayers` used in self-collision | `flex_activelayers` parsed and stored (Phase 7 T1) but not consumed. MuJoCo uses `activelayers` to filter which element layers participate in self-collision. | **Defer** — `activelayers` runtime consumption is an optimization, not required for basic self-collision correctness. Track as deferred item. |

**Final scope:**

1. `FlexSelfCollide` enum (None, Narrow, Bvh, Sap, Auto) — type upgrade
2. Parser/builder pipeline migration from `Option<String>` → `FlexSelfCollide`
3. Three-condition gate logic in collision dispatch
4. Internal collision path: `mj_collide_flex_internal()` for adjacent elements
5. Self-collision path: `mj_collide_flex_self()` for non-adjacent elements
6. Brute-force narrowphase (NARROW mode): all non-adjacent element pairs
7. BVH midphase: per-element AABB tree for candidate pair pruning
8. SAP midphase: sweep-and-prune on element AABBs
9. AUTO dispatch: BVH for dim=3, SAP for dim=2 (matching MuJoCo)
10. Element-element narrowphase: triangle-triangle (dim=2), vertex-face (dim=3)
11. Contact generation consistent with existing flex-rigid contacts

---

## Empirical Ground Truth

### EGT-1: Three conjunctive gate conditions

**MuJoCo source:** `engine_collision_driver.c` → `mj_collision()`.
**Documented in:** `future_work_10.md:7340-7367`, `future_work_8.md:1030-1047`.

MuJoCo's self-collision dispatch is gated behind three conjunctive conditions:

```c
for (int f = 0; f < m->nflex; f++) {
    if (!m->flex_rigid[f] && (m->flex_contype[f] & m->flex_conaffinity[f])) {
        if (m->flex_internal[f]) {
            mj_collideFlexInternal(m, d, f);
        }
        if (m->flex_selfcollide[f] != mjFLEXSELF_NONE) {
            // dispatch based on selfcollide enum
        }
    }
}
```

Three conditions:
1. `!flex_rigid[f]` — rigid flexes skip entirely (all vertices invmass == 0)
2. `(flex_contype[f] & flex_conaffinity[f]) != 0` — self-bitmask check
3. Per-path enable: `flex_internal[f]` for internal, `flex_selfcollide[f] != NONE` for self

**Consequence:** Setting `contype=2, conaffinity=4` disables self-collision
even when `selfcollide != NONE`, because `2 & 4 = 0`. This is the
self-bitmask gate documented in `future_work_8.md:1049-1050`.

Both `internal` and `selfcollide` paths are independently gated behind
conditions 1+2. They are independent concepts: `internal` controls
adjacent-element collision, `selfcollide` controls non-adjacent element
collision (`future_work_8.md:1052-1055`).

### EGT-2: `mjtFlexSelf` enum values

**MuJoCo source:** `engine_collision_driver.c`.
**Documented in:** `future_work_10.md:7369`.

```c
enum mjtFlexSelf {
    mjFLEXSELF_NONE   = 0,  // no self-collision
    mjFLEXSELF_NARROW = 1,  // brute-force all element pairs
    mjFLEXSELF_BVH    = 2,  // BVH midphase + narrowphase
    mjFLEXSELF_SAP    = 3,  // sweep-and-prune midphase + narrowphase
    mjFLEXSELF_AUTO   = 4,  // BVH for dim=3, SAP otherwise
};
```

MJCF attribute values: `"none"`, `"narrow"`, `"bvh"`, `"sap"`, `"auto"`.
MuJoCo default: `"auto"` (documented in `future_work_7.md:2504`,
`future_work_8.md:1038`).

### EGT-3: Dispatch structure — internal vs self-collision paths

**MuJoCo source:** `engine_collision_driver.c`.
**Documented in:** `future_work_10.md:7340-7360`.

Two independent dispatch paths behind the shared gate:

1. **Internal collision** (`flex_internal[f]`): `mj_collideFlexInternal(m, d, f)`
   — contacts between elements that share vertices or edges (adjacent elements).
   MuJoCo default `internal=true`.

2. **Self-collision** (`flex_selfcollide[f] != NONE`): dispatches to midphase
   based on enum value:
   - `NARROW`: `mj_collideFlexSelf(m, d, f)` — brute-force all non-adjacent pairs
   - `BVH`: BVH midphase + narrowphase
   - `SAP`: sweep-and-prune midphase + narrowphase
   - `AUTO`: BVH for dim=3 (solids), SAP otherwise (shells/cables)

### EGT-4: Element adjacency definition

Two elements are **adjacent** if they share at least one vertex. For dim=2
(triangles), sharing an edge means sharing 2 vertices. For dim=3 (tetrahedra),
sharing a face means sharing 3 vertices. MuJoCo's `mj_collideFlexInternal()`
handles all adjacent-element contacts.

**Non-adjacent** elements share zero vertices. These are candidates for
self-collision via `mj_collideFlexSelf()`.

The adjacency test requires building an element-element adjacency map from
`flexelem_data` connectivity: for each pair of elements in the same flex,
check if they share any vertices.

### EGT-5: Element-element narrowphase

**MuJoCo narrowphase for flex self-collision:**

For dim=2 (shells): vertex-triangle tests + edge-edge tests between
non-adjacent triangular elements. MuJoCo generates contacts at vertex-face
penetrations and edge-edge crossings.

For dim=3 (solids): vertex-face tests + edge-edge tests between non-adjacent
tetrahedral elements. Contacts at vertex-face penetrations of tet faces.

**CortenForge existing primitives:**
- `triangle_triangle_intersection()` in `mesh.rs:390` — SAT-based, returns
  `TriTriContact { point, normal, depth }`. Could be reused for dim=2
  element-element narrowphase.
- `BvhPrimitive::from_triangle()` in `mid_phase.rs:72` — creates AABB from
  triangle vertices. Can be reused for element BVH construction.
- `Bvh::query()` in `mid_phase.rs:302` — AABB overlap query. Can be reused
  for midphase.

### EGT-6: Codebase context — files to modify

| File | Line range | Current state | Spec C change |
|------|-----------|---------------|---------------|
| `sim/L0/core/src/types/model.rs` | 374 | `flex_selfcollide: Vec<bool>` | Upgrade to `Vec<FlexSelfCollide>` |
| `sim/L0/core/src/types/model_init.rs` | 168 | `flex_selfcollide: vec![]` | Init as `Vec<FlexSelfCollide>` |
| `sim/L0/core/src/types/enums.rs` | ~894 | `FlexBendingType` enum exists | Add `FlexSelfCollide` enum |
| `sim/L0/core/src/collision/mod.rs` | 540-541 | `mj_collision_flex()` called, no self-collision | Add self-collision dispatch after flex-rigid |
| `sim/L0/core/src/collision/flex_collide.rs` | 1-698 | Flex-rigid narrowphase only | Add `mj_collide_flex_internal()`, `mj_collide_flex_self()`, midphase |
| `sim/L0/mjcf/src/types.rs` | 3800-3802 | `selfcollide: Option<String>` on `MjcfFlex` | Change to `FlexSelfCollide` (or keep string, convert in builder) |
| `sim/L0/mjcf/src/parser.rs` | 3034-3036 | Parses `selfcollide` as `Option<String>` | Parse string → `FlexSelfCollide` enum |
| `sim/L0/mjcf/src/builder/flex.rs` | 414-415 | Converts string → bool (lossy) | Convert string → `FlexSelfCollide` enum |
| `sim/L0/mjcf/src/builder/mod.rs` | 708 | `flex_selfcollide: Vec<bool>` | Upgrade to `Vec<FlexSelfCollide>` |
| `sim/L0/mjcf/src/builder/init.rs` | 280 | `flex_selfcollide: vec![]` | Init as `Vec<FlexSelfCollide>` |
| `sim/L0/mjcf/src/builder/build.rs` | 197 | Wires `flex_selfcollide` to Model | No change (type propagates) |

**Match sites (exhaustive enum arms and pattern matches):**

The `flex_selfcollide` field is currently `Vec<bool>`. Upgrading to
`Vec<FlexSelfCollide>` will break any pattern match or conditional that
reads the field. Grep results show the field is:
- Read in `model.rs:374` (doc comment only)
- Set in `builder/flex.rs:414-415` (push)
- Init in `builder/init.rs:280` (empty vec)
- Wired in `builder/build.rs:197` (struct field transfer)
- Init in `model_init.rs:168` (empty vec)
- Referenced in 2 builder tests (`builder/mod.rs:1302, 1336`)

No runtime code currently reads `flex_selfcollide` — it is parsed and stored
but never consumed in the collision pipeline. The upgrade is safe: no existing
runtime behavior changes.

### EGT-7: Builder tests that reference flex_selfcollide/flex_internal

**`builder/mod.rs:1302`:** `assert!(!model.flex_internal[0], "internal should be false");`
**`builder/mod.rs:1336`:** `assert!(model.flex_internal[0], "internal default should be true");`

These tests verify parsing of `internal` attribute. After Spec C upgrades
`flex_selfcollide` from bool to enum, any test asserting
`model.flex_selfcollide[i] == true/false` must be updated to assert the
enum variant. Currently no tests directly assert on `flex_selfcollide` values
(the builder tests only check `flex_internal`).

### EGT-8: Contact generation convention

**Existing flex-rigid contacts** (`flex_collide.rs:243-295`,
`make_contact_flex_rigid()`):
- `contact.geom1` = rigid geom index
- `contact.geom2` = rigid geom index (same)
- `contact.flex_vertex` = `Some(vertex_idx)`
- Contact params via `contact_param_flex_rigid()` (priority + solmix)
- Margin: `flex_margin[f] + geom_margin[g]`
- Override: `assign_ref()`, `assign_imp()`, `assign_friction()`, `assign_margin()`

For flex self-collision contacts, the spec must define:
- What goes in `geom1`/`geom2` (no rigid geom involved)
- What goes in `flex_vertex` (two flex vertices/elements involved)
- How contact params combine (both sides are the same flex)
- Whether to use `contact_param_flex_self()` or reuse existing functions

### EGT-9: Downstream consumers of Contact.geom1/geom2

**Constraint Jacobian** (`jacobian.rs`):
- `compute_contact_jacobian()` (line 148): if `contact.flex_vertex` is `Some`,
  dispatches to `compute_flex_contact_jacobian()`. Otherwise falls through to
  `model.geom_body[contact.geom1]` / `model.geom_body[contact.geom2]` (line 158-159).
- `compute_flex_contact_jacobian()` (line 20): uses `contact.geom2` at line 50
  to look up the rigid body (`model.geom_body[contact.geom2]`). The flex vertex
  DOFs come from `model.flexvert_dofadr[vertex_idx]` via `contact.flex_vertex`.
  For self-collision, both sides are flex vertices — there is no rigid body.
  A **new Jacobian function** is needed: `compute_flex_self_contact_jacobian()`
  that fills both sides using `flexvert_dofadr`.

**Constraint Assembly** (`assembly.rs`):
- Lines 601-612: computes bodyweight `bw_contact` from
  `model.geom_body[contact.geom1]` and `model.geom_body[contact.geom2]`.
  Has bounds checks (falls back to body 0 if out of range), so won't crash,
  but will compute **wrong bodyweight** for self-collision contacts.
  For self-collision, bodyweight should come from the flex's parent body
  (or vertex bodies), not from geom_body.

**Contact encoding options for self-collision:**
1. Add `flex_vertex2: Option<usize>` to Contact — when both are Some, it's
   flex-self; when only `flex_vertex` is Some, it's flex-rigid.
2. Add a `ContactKind` enum (RigidRigid, FlexRigid, FlexSelf) that carries
   the relevant indices.
3. Repurpose `geom1`/`geom2` — set them to sentinel values (e.g., `usize::MAX`)
   for self-collision contacts.

The spec must choose one approach and update all downstream consumers.

---

## Criteria

> **Criterion priority:** P1 (MuJoCo Reference Fidelity) is the cardinal
> criterion. Grade P1 first and hardest. If P1 is not A+, do not proceed.

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does — exact function names, field
> names, calling conventions, and edge cases. No hand-waving.

| Grade | Bar |
|-------|-----|
| **A+** | Every MuJoCo function cited with source file and exact behavior: `mj_collision()` in `engine_collision_driver.c` for dispatch loop and three gate conditions, `mj_collideFlexInternal()` for adjacent-element contact generation, `mj_collideFlexSelf()` for non-adjacent element collision with midphase, `mjtFlexSelf` enum with exact values (NONE=0, NARROW=1, BVH=2, SAP=3, AUTO=4), `filterBitmask()` for self-bitmask check convention. Edge cases addressed: rigid flex skip (`flex_rigid[f]=true` skips entire flex), self-bitmask zero (`contype & conaffinity == 0` skips even with selfcollide enabled — per EGT-1), zero-element flex (`nelem=0` → no element pairs), single-element flex (no non-adjacent pairs possible), boundary between adjacent and non-adjacent elements (shared vertex count), dim=1 flex (cables — no triangle elements, what does MuJoCo do?), `selfcollide="none"` (no dispatch), `internal=false` (skip internal path), AUTO mode selection logic (dim=3→BVH, otherwise→SAP). Narrowphase approach per dim documented with MuJoCo C source citations. C code snippets included for gate conditions and dispatch structure. |
| **A** | MuJoCo behavior described correctly from C source. Minor gaps in edge-case coverage. |
| **B** | Correct at high level but missing specifics — or based on docs rather than C source. |
| **C** | Partially correct. Some MuJoCo behavior misunderstood or invented. |

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. No "see MuJoCo source"
> or "TBD" gaps.

| Grade | Bar |
|-------|-----|
| **A+** | Complete algorithms for: (1) element adjacency determination from `flexelem_data` connectivity — how to build the adjacency set, **explicitly stated as build-time precomputation** (topology is static) with Model field(s) for adjacency storage defined (per R12), (2) internal collision: iterate adjacent element pairs, run narrowphase, generate contacts, (3) self-collision brute-force (NARROW): iterate all non-adjacent pairs `O(n²)`, (4) BVH midphase: build per-element AABB tree from `flexvert_xpos`, query for overlapping pairs, filter adjacent, run narrowphase, (5) SAP midphase: sort element AABBs along axis of maximum variance, sweep for overlapping pairs, filter adjacent, run narrowphase, (6) AUTO dispatch logic, (7) element-element narrowphase per dim: triangle-triangle for dim=2 (reusing `triangle_triangle_intersection` or specifying vertex-face approach), vertex-face for dim=3, (8) **contact parameter combination for self-collision**: both sides are the same flex — spec must define how `flex_solref`, `flex_solimp`, `flex_friction`, `flex_condim` combine (trivial identity since both sides identical, or per MuJoCo's approach) (per R13), (9) **margin/gap formula for self-collision**: `flex_margin[f] + flex_margin[f]` or `flex_margin[f]`? Must cite MuJoCo C source (per R14). An implementer can type each algorithm without reading MuJoCo source. Each algorithm matches MuJoCo's computational steps. |
| **A** | Algorithm is complete and MuJoCo-conformant. One or two minor details left implicit. |
| **B** | Algorithm structure is clear but some steps are hand-waved. |
| **C** | Skeleton only. |

### P3. Convention Awareness

> Spec explicitly addresses our codebase's conventions where they differ
> from MuJoCo.

| Grade | Bar |
|-------|-----|
| **A+** | Convention difference table present with explicit porting rules: MuJoCo `flex_elem[elembase + dim*e + v]` vs CortenForge `flexelem_data[flexelem_dataadr[e] + v]` element connectivity indexing; MuJoCo `mjtFlexSelf` (C enum, int-valued) vs CortenForge `FlexSelfCollide` (Rust enum, discriminant values); MuJoCo `flexvert_xpos[3*v + x]` vs CortenForge `data.flexvert_xpos[v]` (Vector3 vs flat array); **`Contact` struct encoding for self-collision contacts explicitly defined** — spec must choose and fully specify one of: (a) add `flex_vertex2: Option<usize>`, (b) add `ContactKind` enum, or (c) sentinel values in `geom1`/`geom2` — and show how downstream consumers (`compute_flex_contact_jacobian` at `jacobian.rs:50`, bodyweight computation at `assembly.rs:601-612`) are updated (per R11/EGT-9); **contact parameter combination for self-collision**: explicit formula since both sides are same flex (per R13); MuJoCo per-flex element indexing `flex_elemadr[f]..flex_elemadr[f]+flex_elemnum[f]` vs CortenForge same convention; existing BVH API (`Bvh::build()`, `query()`, `BvhPrimitive::from_triangle()`) reuse conventions; `triangle_triangle_intersection()` reuse conventions (tri_a/tri_b index fields need caller fill-in, per mesh.rs:388). Each porting rule verified to preserve numerical equivalence. |
| **A** | Major conventions documented. Minor field-name mappings left to implementer. |
| **B** | Some conventions noted, others not. |
| **C** | MuJoCo code pasted without adaptation. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. Contains concrete values,
> tolerances, and model configurations.

| Grade | Bar |
|-------|-----|
| **A+** | Every runtime AC has three-part structure: (1) concrete input model/state, (2) exact expected value or tolerance, (3) field to check. Gate condition ACs: "For a rigid flex (`flex_rigid[f]=true`), zero self-contacts generated even with `selfcollide=auto`." Self-bitmask AC: "For flex with `contype=2, conaffinity=4`, zero self-contacts even with `selfcollide=auto`." Internal AC: "For a 4-triangle sheet with `internal=true`, adjacent element pairs generate contacts when vertices penetrate." Self-collision AC: "For a folded cloth (non-adjacent elements overlapping), `selfcollide=narrow` generates contacts at overlapping elements." BVH/SAP equivalence AC: "BVH and SAP produce identical contact sets to brute-force NARROW for the same model." At least one AC has expected values from MuJoCo. Enum upgrade AC: "`selfcollide='bvh'` parses to `FlexSelfCollide::Bvh`, not collapsed to bool `true`." **Margin/gap AC**: "For self-collision contacts, the effective margin equals [specified formula from MuJoCo C source], verified by checking `contact.includemargin` against expected value" (per R14). **Contact encoding AC**: "Self-collision contacts have correct vertex/element indices — constraint Jacobian produces correct DOF-column entries for both flex vertices" (per R11). |
| **A** | ACs are testable. Some lack exact numerical expectations. |
| **B** | ACs are directionally correct but vague. |
| **C** | ACs are aspirational statements. |

### P5. Test Plan Coverage

> Tests cover happy path, edge cases, regressions, and interactions.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present. Edge case inventory: rigid flex (zero contacts), self-bitmask zero (zero contacts), single-element flex (no non-adjacent pairs), zero-element flex (no contacts), dim=1 cable (no triangle elements), `internal=false` + `selfcollide=auto` (only non-adjacent contacts), `internal=true` + `selfcollide=none` (only adjacent contacts), all elements coplanar (degenerate geometry), overlapping elements from deformation. Negative cases: rigid flex produces no self-contacts; incompatible self-bitmask produces no self-contacts; `selfcollide=none` produces no non-adjacent contacts. Midphase equivalence: BVH and SAP produce identical contacts to NARROW for the same model. At least one MuJoCo conformance test per major path (gate logic, internal, self-collision). Non-trivial model: multi-flex model where one has self-collision enabled and another does not. |
| **A** | Good coverage. Minor edge-case gaps. |
| **B** | Happy path covered. Edge cases sparse. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and interactions with other specs.

| Grade | Bar |
|-------|-----|
| **A+** | Prerequisites stated: T1 Session 2 (commit `1f0230c`) provides `flex_rigid`. Phase 7 T1 (commit `cf76731`) provides `flex_internal`. Spec A (Sessions 3-7) provides `flexedge_J` (not consumed by Spec C — independent). Spec B (Sessions 8-13) provides cotangent bending (not consumed by Spec C — independent). Spec C is consumed by Spec D (Sessions 19-21) which reuses narrowphase primitives. Execution order unambiguous: S1 (enum upgrade + parser) → S2 (gate logic + dispatch) → S3 (internal collision) → S4 (self-collision narrowphase NARROW) → S5 (BVH midphase) → S6 (SAP midphase + AUTO). |
| **A** | Order is clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Spec identifies every file touched, every behavior that changes, and every
> existing test that might break.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description (per EGT-6 table: 11 files **plus** downstream consumers per R15). Behavioral changes: `flex_selfcollide` type changes from `Vec<bool>` to `Vec<FlexSelfCollide>` — this is a **breaking type change** but no runtime code currently reads the field (per EGT-6), so zero behavioral regression. New behavior: self-collision contacts generated where none existed before (new feature, not regression). **Downstream Contact consumers that must handle self-collision contacts**: (a) `jacobian.rs` — `compute_flex_contact_jacobian()` assumes rigid body at `contact.geom2` (line 50) — needs new flex-self Jacobian path, (b) `assembly.rs:601-612` — bodyweight from `geom_body[contact.geom1/geom2]` — needs flex-self bodyweight path, (c) `contact_types.rs` — Contact struct needs new field(s) or variant for two-vertex encoding (per R11/EGT-9). Builder test impact: tests asserting `flex_selfcollide` values need update (currently none assert on selfcollide directly, per EGT-7). Existing flex-rigid collision tests (`flex_collide.rs` Spec E tests, `collision/mod.rs` tests): **must remain unchanged** — self-collision dispatch is additive, inserted after flex-rigid. All 1,900+ domain tests must pass. Spec D dependency: narrowphase primitives must be public/reusable (function signature is an API contract, per umbrella Contract 3). |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions within the spec. Shared concepts use identical
> terminology throughout.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology uniform: "self-collision" (not "self-contact" in one place and "intra-flex collision" in another), "adjacent elements" (not "neighboring" or "connected" interchangeably), `FlexSelfCollide` (not `FlexSelf` or `SelfCollideMode`). Gate condition count consistent: 3 conditions stated in MuJoCo Reference, 3 tested in ACs, 3 in test plan. File paths match between Specification and Files Affected. AC numbers match between AC section and traceability matrix. Edge case lists consistent across MuJoCo Reference and Test Plan. Enum variant names identical everywhere: `None`, `Narrow`, `Bvh`, `Sap`, `Auto`. |
| **A** | Consistent. One or two minor terminology inconsistencies. |
| **B** | Some sections use different names for the same concept. |
| **C** | Contradictions between sections. |

### P9. Type Architecture (FlexSelfCollide enum)

> The `FlexSelfCollide` enum is correctly defined, correctly replaces
> `Vec<bool>`, and correctly propagates through the entire
> parse→build→store→dispatch pipeline.

**Boundary with P1:** P1 grades whether the spec correctly identifies
MuJoCo's `mjtFlexSelf` enum values and semantics. P9 grades whether the
CortenForge type system migration is complete — every site that touches
`flex_selfcollide` is identified and updated.

| Grade | Bar |
|-------|-----|
| **A+** | Enum definition with discriminant values matching MuJoCo (`None=0, Narrow=1, Bvh=2, Sap=3, Auto=4`). Every file that touches `flex_selfcollide` identified (per EGT-6): `model.rs` (field type), `model_init.rs` (init), `enums.rs` (definition), `types.rs` (MJCF intermediate — keep `Option<String>` or change?), `parser.rs` (parse to string), `builder/flex.rs` (string→enum conversion), `builder/mod.rs` (builder field type), `builder/init.rs` (init), `builder/build.rs` (wire). Migration path for each site explicitly stated. Default value documented: absent attribute → `FlexSelfCollide::Auto` (matching MuJoCo default "auto"). `#[derive]` traits specified (at minimum: Clone, Copy, Debug, PartialEq, Default). Exhaustive `match` arms in dispatch ensure compile-time safety when new variants are added. |
| **A** | Enum defined correctly. Most sites updated. Minor init/default gaps. |
| **B** | Enum defined but migration path incomplete — some sites missed. |
| **C** | Enum not properly specified. |

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P9 | Specification S1 (enum definition), Files Affected, Convention Notes |

### P10. Gate Condition Completeness

> All three conjunctive gate conditions are correctly specified, tested
> independently, and shown to interact correctly (any one failing skips
> dispatch).

**Boundary with P1:** P1 grades whether the spec correctly identifies what
MuJoCo's gate conditions are. P10 grades whether the spec comprehensively
specifies the gate logic implementation and tests each condition independently
and in combination.

| Grade | Bar |
|-------|-----|
| **A+** | All three conditions specified with exact code: (1) `model.flex_rigid[f]` — skip if true, (2) `(model.flex_contype[f] & model.flex_conaffinity[f]) != 0` — skip if zero, (3a) `model.flex_internal[f]` for internal path, (3b) `model.flex_selfcollide[f] != FlexSelfCollide::None` for self path. Each condition tested independently: rigid-but-compatible, non-rigid-but-incompatible-bitmask, non-rigid-compatible-but-selfcollide-none. Compound test: all three conditions satisfied → contacts generated. Order of evaluation specified (short-circuit: rigid checked first for performance). The self-bitmask convention documented: `contype & conaffinity` checks against *itself* (not against another object — this is self-collision, both sides are the same flex). |
| **A** | All three conditions specified and tested. Minor interaction gaps. |
| **B** | Conditions listed but not all independently tested. |
| **C** | Gate conditions incomplete or wrong. |

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P10 | Specification S2 (gate logic), Acceptance Criteria (gate ACs), Test Plan (gate tests) |

### P11. Pipeline Integration

> Self-collision dispatch is placed in the correct position in the collision
> pipeline, with correct data dependencies and ordering relative to existing
> collision paths.

**Boundary with P6:** P6 grades cross-spec dependencies. P11 grades
intra-pipeline ordering (where self-collision dispatch sits relative to
broadphase, narrowphase, flex-rigid, and future flex-flex).

| Grade | Bar |
|-------|-----|
| **A+** | Pipeline placement matches MuJoCo and the umbrella convention (Convention Registry §3): self-collision dispatch occurs **after** `mj_collision_flex()` (flex-rigid) and **before** flex-flex collision (Spec D). Data dependency chain explicit: `flexvert_xpos` must be current (populated by `mj_flex()` in position stage) before element AABBs are computed. Element AABB construction is per-step (not cached on Model) because vertex positions change each step. Contact generation appends to `data.contacts` and increments `data.ncon`, same as flex-rigid. Dispatch function signature: `fn mj_collision_flex_self(model: &Model, data: &mut Data)` — iterates all flexes, applies gate, dispatches per-flex. Internal and self-collision paths both inside this function (not separate top-level calls). Narrowphase primitives (`collide_flex_elements` or equivalent) designed for reuse by Spec D (per umbrella Contract 3). **Constraint pipeline compatibility**: spec must define how self-collision contacts flow through `compute_contact_jacobian()` → `assemble_contact_system()`. The existing flex-rigid Jacobian path (`compute_flex_contact_jacobian` at `jacobian.rs:20`) assumes one flex vertex + one rigid body — self-collision needs both sides as flex vertices. The assembly bodyweight computation (`assembly.rs:601-612`) uses `geom_body[contact.geom1/geom2]` — self-collision needs flex body lookup instead. Spec must either extend the existing functions with branching or define new flex-self variants (per R11/R15/EGT-9). |
| **A** | Pipeline placement correct. Minor ordering justification gaps. |
| **B** | Computation placed in correct stage but data dependencies unclear. |
| **C** | Pipeline placement wrong or unspecified. |

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P11 | Specification (dispatch placement), Execution Order, Prerequisites |

---

## Rubric Self-Audit

### Self-audit checklist

- [x] **Specificity:** Every A+ bar names specific MuJoCo functions
      (`mj_collision`, `mj_collideFlexInternal`, `mj_collideFlexSelf`,
      `filterBitmask`, `mjtFlexSelf`), specific CortenForge files/lines
      (`model.rs:374`, `builder/flex.rs:414-415`, `collision/mod.rs:540`,
      `mesh.rs:390`), and specific edge cases (rigid flex, self-bitmask
      zero, single-element flex, dim=1 cable, zero-element flex).
      Two reviewers would agree on grades by pointing to spec lines.

- [x] **Non-overlap:** P1 vs P9 boundary: P1 grades MuJoCo reference
      accuracy (what MuJoCo's `mjtFlexSelf` enum does), P9 grades
      CortenForge type migration completeness (every file updated). P1 vs
      P10 boundary: P1 grades what MuJoCo's gate conditions are, P10 grades
      completeness of gate logic implementation and testing. P6 vs P11
      boundary: P6 grades cross-spec deps, P11 grades intra-pipeline
      ordering.

- [x] **Completeness:** 11 criteria cover: MuJoCo fidelity (P1), algorithm
      (P2), conventions (P3), ACs (P4), tests (P5), deps (P6), blast
      radius (P7), consistency (P8), type architecture (P9), gate
      conditions (P10), pipeline integration (P11). No meaningful gap
      dimension remains. The three domain-specific criteria (P9-P11) cover
      the three novel aspects of Spec C: enum refactoring, compound gate
      logic, and collision pipeline positioning. **Rev 2 stress test**
      verified end-to-end: contacts generated by self-collision must flow
      through Jacobian computation and constraint assembly — gaps R11-R15
      ensure the rubric catches Contact encoding, parameter combination,
      margin, and downstream consumer issues.

- [x] **Gradeability:** Criterion→section mapping provided for P9-P11.
      P1-P8 use standard mapping from rubric template.

- [x] **Conformance primacy:** P1 cites 5 specific C source functions/enums.
      A+ bar requires verification against C source. P4 requires
      MuJoCo-derived expected values for gate conditions. P5 requires
      MuJoCo conformance tests per major path. Scope Adjustment corrects
      umbrella claim about adding `flex_internal` (already exists).

- [x] **Empirical grounding:** 9 EGT entries covering: gate conditions
      (EGT-1), enum values (EGT-2), dispatch structure (EGT-3), adjacency
      (EGT-4), narrowphase (EGT-5), codebase context (EGT-6), builder
      tests (EGT-7), contact conventions (EGT-8), downstream Contact
      consumers (EGT-9). All A+ bars reference EGT findings.

- [x] **Stress test (Rev 2):** Traced self-collision contacts downstream
      through the full pipeline: Contact struct → Jacobian computation
      (`jacobian.rs`) → constraint assembly (`assembly.rs`). Found 5 gaps
      (R11-R15) that would let a spec pass without addressing Contact
      encoding, parameter combination, margin formula, or downstream
      consumer compatibility. All gaps now have corresponding A+ bar
      requirements.

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
| P9 | Specification S1 (enum definition), Files Affected, Convention Notes |
| P10 | Specification S2 (gate logic), Acceptance Criteria, Test Plan |
| P11 | Specification (dispatch placement), Execution Order, Prerequisites |

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
| P9. Type Architecture | | |
| P10. Gate Condition Completeness | | |
| P11. Pipeline Integration | | |

**Overall: Rubric Rev 2 — stress-tested, ready for spec grading**

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| R1 | Scope | Umbrella claims "Add `flex_internal: Vec<bool>` to Model" — already exists (Phase 7 T1, commit `cf76731`) | Rubric Phase 1 (codebase grep) | Scope Adjustment documents: field already exists, Spec C only consumes it. | Rubric Rev 1 |
| R2 | P9 | `flex_selfcollide` is `Vec<bool>` on Model AND on ModelBuilder — both need upgrade | Rubric Phase 1 (EGT-6 codebase context) | EGT-6 table lists all 11 files. P9 A+ bar requires every file identified. | Rubric Rev 1 |
| R3 | P1 | dim=1 flex (cables) — do cables have self-collision? They have no triangle elements. | Rubric self-audit | P1 A+ bar requires spec to address dim=1 edge case. MuJoCo likely skips (no elements = no element pairs). | Rubric Rev 1 |
| R4 | P10 | Self-bitmask check is `contype & conaffinity` against *itself* — easy to confuse with cross-object `filterBitmask()` which checks both directions | Rubric self-audit | P10 A+ bar explicitly calls out that self-collision uses self-bitmask (same flex, both sides identical), not cross-object bitmask. | Rubric Rev 1 |
| R5 | P11 | Element AABB construction: per-step or cached? Vertices move each step, so AABBs must be rebuilt. | Rubric self-audit | P11 A+ bar states AABBs are per-step (not cached on Model). | Rubric Rev 1 |
| R6 | EGT-5 | `triangle_triangle_intersection()` in mesh.rs may not match MuJoCo's vertex-face approach. Spec must choose: reuse existing SAT-based tri-tri, or implement MuJoCo's vertex-face approach? | Rubric self-audit | P2 A+ bar requires spec to specify narrowphase approach per dim and justify choice. EGT-5 documents both options. | Rubric Rev 1 |
| R7 | P7 | `flex_selfcollide` type change from `Vec<bool>` to `Vec<FlexSelfCollide>` is a breaking change — any code reading the field must be updated. Grep confirms no runtime code reads it currently. | Rubric Phase 1 (codebase grep) | P7 A+ bar explicitly states this is a breaking type change with zero runtime impact. EGT-6 confirms no runtime reads. | Rubric Rev 1 |
| R8 | EGT-8 | Contact encoding for self-collision unclear: `geom1`/`geom2` fields on `Contact` struct expect geom indices, but self-collision has no rigid geom. | Rubric self-audit | EGT-8 documents the gap. P3 A+ bar requires convention difference for contact encoding. | Rubric Rev 1 |
| R9 | P5 | `activelayers` runtime consumption deferred (Scope Adjustment) — test plan should note this as explicitly out-of-scope to prevent confusion | Rubric self-audit | Scope Adjustment explicitly defers `activelayers`. P5 edge case inventory should list "activelayers filtering" as out-of-scope. | Rubric Rev 1 |
| R10 | P9 | Umbrella Convention Registry §4 says `Default: None (no self-collision)` — MuJoCo's actual default is `"auto"` (enabled). `future_work_7.md:2555-2558` confirms: "MuJoCo default is 'auto' (enabled)". `MjcfFlex::default()` at `types.rs:3886` uses `selfcollide: None` with comment "MuJoCo default is 'auto' (enabled)". Builder converts absent attribute → `true` (enabled). | Rubric Phase 1 (cross-reference umbrella vs documentation) | P9 A+ bar requires `Default` trait on `FlexSelfCollide` to return `Auto` (not `None`). Spec must correct umbrella's claim and use `Auto` as default. | Rubric Rev 1 |
| R11 | P3, P7, P11 | **Contact struct encoding for self-collision is undefined.** The `Contact` struct (`contact_types.rs:40-87`) uses `geom1`/`geom2` for rigid geom indices and `flex_vertex: Option<usize>` for one flex vertex. Self-collision has TWO flex vertices (or elements) and ZERO rigid geoms. (a) `compute_flex_contact_jacobian()` (`jacobian.rs:50`) does `model.geom_body[contact.geom2]` for the rigid side — would index into geom_body with an invalid index for self-collision. (b) `assembly.rs:603-612` uses `contact.geom1`/`geom2` to look up body indices via `model.geom_body[]` — for self-collision, these indices would be meaningless (silently falls back to body 0 due to bounds check). The spec MUST define: what goes in `geom1`/`geom2` for self-contacts, how to encode two flex vertex indices, and whether a new Jacobian function or Contact variant is needed. | Stress test (jacobian.rs:50, assembly.rs:603) | P3 A+ bar strengthened to require explicit Contact encoding for self-collision. P7 A+ bar strengthened to identify jacobian.rs and assembly.rs as files requiring changes. P11 A+ bar strengthened to require Jacobian and assembly compatibility. New EGT-9 added. | Rubric Rev 2 |
| R12 | P2 | **Element adjacency precomputation timing.** Element topology is static (defined at model build time, never changes at runtime). The adjacency map (which element pairs share vertices) should be precomputed at build time and stored on Model, not recomputed each step. The spec must explicitly state this is a build-time computation and define the Model field(s) for adjacency storage. | Stress test (topology analysis) | P2 A+ bar strengthened to require adjacency precomputation timing specification. | Rubric Rev 2 |
| R13 | P2, P3 | **Contact parameter combination for self-collision.** For flex-rigid, `contact_param_flex_rigid()` (`collision/mod.rs:174-226`) combines flex_* and geom_* parameters via priority + solmix. For self-collision, both sides are the same flex — `flex_solref[f]` vs `flex_solref[f]`, same priority, same solmix. The spec must define: use flex params directly (trivial combination since both sides identical), or call a `contact_param_flex_self()` function? MuJoCo source documentation needed. | Stress test (collision/mod.rs:174-226) | P2 and P3 A+ bars require explicit contact parameter combination for self-collision. | Rubric Rev 2 |
| R14 | P2, P4 | **Margin and gap for self-collision.** Flex-rigid uses `flex_margin[f] + geom_margin[g]` (`flex_collide.rs:260`). For self-collision (both sides same flex), is it `flex_margin[f] + flex_margin[f]` (doubled) or just `flex_margin[f]`? MuJoCo C source must be cited. The spec must specify the exact formula and provide an AC testing the correct margin behavior. | Stress test (flex_collide.rs:257-260) | P2 and P4 A+ bars require margin/gap formula for self-collision. | Rubric Rev 2 |
| R15 | P7, P11 | **Constraint assembly and Jacobian require changes for self-collision contacts.** This is the downstream consequence of R11. Files that assume `Contact.geom1`/`geom2` reference valid rigid geoms: (a) `jacobian.rs:50` — `compute_flex_contact_jacobian()` uses `geom2` for rigid body lookup, (b) `jacobian.rs:158` — `compute_contact_jacobian()` falls through to rigid-rigid path if `flex_vertex` is None, (c) `assembly.rs:601-612` — bodyweight computation uses `geom1`/`geom2` → `geom_body[]`. The spec's blast-radius analysis MUST list these files and describe how they will handle self-collision contacts (new Jacobian function for flex-self, or branching on a new Contact field). | Stress test (jacobian.rs, assembly.rs) | P7 and P11 A+ bars require jacobian.rs and assembly.rs in blast radius. | Rubric Rev 2 |
