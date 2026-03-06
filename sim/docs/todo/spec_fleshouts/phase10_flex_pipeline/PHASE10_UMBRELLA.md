# Phase 10 — Flex Pipeline: Umbrella Spec

**Status:** Draft
**Phase:** Roadmap Phase 10
**Tasks:** 6 (§42A-i, §42A-ii, §42A-iii, §42A-iv / DT-142, §42A-v / DT-143, §42B)
**Deliverables:** 1 T1 session + 4 sub-specs (Spec A, B, C, D)
**Test baseline:** 1,900+ domain tests (post-Phase 9)

---

## Scope

Phase 10 closes all flex (deformable body) pipeline gaps in the v1.0 roadmap.
**The goal is MuJoCo conformance** — after Phase 10, CortenForge pre-computes
sparse edge Jacobians for correct force projection through body Jacobians,
stores `flexedge_length`/`flexedge_velocity` as Data fields matching MuJoCo's
layout, computes MuJoCo-conformant cotangent Laplacian bending forces (with
Bridson preserved as an alternative), dispatches flex self-collision via
BVH/SAP midphase with element-element narrowphase, and filters flex-flex
cross-object collisions through `contype`/`conaffinity` bitmasks.

Phase 10 tasks span two pipeline stages:
- **Passive force computation:** §42A-i (sparse edge Jacobian), §42A-ii
  (`flex_rigid`/`flexedge_rigid` arrays), §42A-iii (`flexedge_length`/
  `flexedge_velocity` Data fields), §42B (cotangent Laplacian bending) —
  these execute during `mj_fwd_passive()` in the forward pipeline.
- **Collision detection:** §42A-iv (flex self-collision dispatch), §42A-v
  (flex-flex cross-object collision) — these execute during the collision
  pass in `mj_collision()`.

Both pipeline stages converge on the flex subsystem in `sim-core`. The key
structural features are:
1. **Two dependency chains:** T1 → Spec C → Spec D (hard deps); Spec A and
   Spec B are independent of each other and of the collision chain.
2. **First trait boundary:** §42B introduces `FlexBendingModel` — the first
   trait abstraction in the sim pipeline, validating the pattern for the
   post-v1.0 trait architecture roadmap.
3. **Split implementation:** Spec B's implementation is split across two
   sessions at the build-time/runtime seam (precomputation vs force
   application + trait architecture).
4. **Compressed cycle:** Spec D uses a 3-session cycle (M effort with heavy
   Spec C narrowphase reuse).

> **Conformance mandate for sub-spec authors:** Each sub-spec must cite the
> exact MuJoCo C source (function, file, line range) for every behavior it
> implements. Acceptance criteria must include expected values derived from
> MuJoCo's output — not from hand-calculation or intuition. The MuJoCo C
> source code is the single source of truth. When the MuJoCo docs and the
> C source disagree, the C source wins.

---

## Task Assignment

Every Phase 10 task is assigned to exactly one deliverable. No orphans, no
overlaps. Each task closes a specific MuJoCo conformance gap.

| Task | Description | Deliverable | Status | Rationale |
|------|-------------|-------------|--------|-----------|
| §42A-ii | `flex_rigid`/`flexedge_rigid` boolean arrays — skip rigid bodies/edges | **T1** | Pending | S effort, mechanical pre-computed Model fields. Downstream: `flex_rigid` is gate condition for Spec C. |
| §42A-iii | `flexedge_length`/`flexedge_velocity` pre-computed Data fields | **T1** | Pending | S effort, mechanical pre-computed Data fields. Replace inline computation in all consumers. |
| §42A-i | Sparse flex edge Jacobian (`flexedge_J`) — CSR format, body Jacobian projection | **Spec A** | Pending | L effort, algorithmic. Pre-computed sparse Jacobian replaces 3 inline `±direction` patterns. |
| §42B | Cotangent Laplacian bending — Wardetzky/Garg precomputation + trait abstraction | **Spec B** | Pending | L effort, largest algorithmic task. 6 sessions (split implementation at build-time/runtime seam). |
| §42A-iv / DT-142 | Flex self-collision dispatch — BVH/SAP midphase + element-element narrowphase | **Spec C** | Pending | L effort. `FlexSelfCollide` enum, internal + self-collision paths, midphase acceleration. |
| §42A-v / DT-143 | Flex-flex cross-object collision filtering — `canCollide2()` broadphase | **Spec D** | Pending | M effort, reuses Spec C narrowphase primitives. 3 sessions (compressed cycle). |

### Sub-spec scope statements

Each sub-spec must identify the exact MuJoCo C functions it is porting, cite
them with source file and line ranges, and produce acceptance criteria with
MuJoCo-verified expected values.

**Spec A — Sparse Flex Edge Jacobian** (§42A-i):
Pre-computed CSR-format edge Jacobian (`flexedge_J`) for projecting edge
forces through body Jacobians via `J^T * force`. Replaces three inline
`±direction` force application patterns (edge spring-damper, bending, Newton
penalty) with a single unified code path. For free vertices, the sparse
Jacobian degenerates to `±direction` on 3 translational DOFs. For
body-attached vertices (future §27D), it uses `mj_jac()` at the vertex
position for the attached body.

MuJoCo reference:
- `engine_passive.c` — edge/bending/penalty force application using
  `flexedge_J` sparse Jacobian for `J^T * force` projection.
- `engine_forward.c` — `mj_flex()` computes `flexedge_J` after forward
  kinematics, before passive force computation.
- CSR format: `flex_edge_J_rownnz`, `flex_edge_J_rowadr`,
  `flex_edge_J_colind`, `flexedge_J` (data array).
- Each edge has 2 rows (one per endpoint vertex), each row spans the
  vertex's DOFs.

Primary changes: `sim/L0/core/src/types/model.rs` (CSR metadata),
`sim/L0/core/src/types/data.rs` (Jacobian values),
`sim/L0/core/src/forward/passive.rs` (unified `J^T * force` consumer
migration), `sim/L0/mjcf/src/builder/flex.rs` (CSR structure allocation).

**Spec B — Cotangent Laplacian Bending** (§42B, 6 sessions — split impl):
MuJoCo's actual bending formulation: Wardetzky et al. "Discrete Quadratic
Curvature Energies" (cotangent Laplacian) with Garg et al. "Cubic Shells"
curved reference correction. Replaces the current Bridson dihedral angle
spring model as the default, with Bridson preserved via a `FlexBendingModel`
trait — the first trait boundary in the sim pipeline.

MuJoCo reference:
- `user_mesh.cc` → `ComputeBending()` (lines ~3740-3784) — precomputation
  of 17-coefficient bending matrix per edge: 4x4 cotangent stiffness
  matrix + 1 curved reference coefficient.
- `engine_passive.c` (lines ~206-268) — runtime bending force application
  via matrix-vector multiply: `spring[3*i+x] += b[4*i+j] * xpos[3*v[j]+x]`.
- `flex_bending` model array: 17 `f64` per edge, flat layout.
- `flexedge_flap` topology: 2 vertices per edge (opposite vertices in
  adjacent triangles forming the diamond stencil).

Implementation split:
- Sessions 10 scope (build-time): `flexedge_flap` topology, cotangent weight
  precomputation (Wardetzky operator), material stiffness, 4x4 stiffness
  matrix (outer product), curved reference coefficient (Garg correction).
- Session 11 scope (runtime): cotangent force application (matrix-vector
  spring + damper), `FlexBendingModel` trait definition and dispatch,
  `BridsonBending` refactor (move current dihedral code into trait impl).

Primary changes: `sim/L0/core/src/forward/passive.rs` (trait definition,
`CotangentBending`, `BridsonBending`, runtime dispatch),
`sim/L0/mjcf/src/builder/flex.rs` (`flex_bending` precomputation,
`flexedge_flap` topology), `sim/L0/core/src/types/model.rs` (storage),
`sim/L0/mjcf/src/types.rs` (`FlexBendingType` enum, `bending_model`
attribute), `sim/L0/mjcf/src/parser.rs` (parse `bending_model`).

**Spec C — Flex Self-Collision Dispatch** (§42A-iv / DT-142):
Self-collision detection within a single flex body: BVH/SAP midphase
acceleration + element-element narrowphase for non-adjacent elements.
Upgrades `flex_selfcollide` from `Vec<bool>` to `Vec<FlexSelfCollide>`
enum with NONE/NARROW/BVH/SAP/AUTO variants matching MuJoCo's
`mjtFlexSelf`. Two independent dispatch paths behind a shared three-condition
gate: internal collision (adjacent elements) and self-collision (non-adjacent
elements with midphase).

MuJoCo reference:
- `engine_collision_driver.c` — flex self-collision dispatch from
  `mj_collision()`. Three conjunctive gate conditions: `!flex_rigid[f]`,
  `(flex_contype[f] & flex_conaffinity[f]) != 0`, per-path enable flags.
- `mjtFlexSelf` enum: `NONE=0, NARROW=1, BVH=2, SAP=3, AUTO=4`.
- `mj_collideFlexInternal()` — adjacent element contact generation.
- `mj_collideFlexSelf()` — non-adjacent element collision with midphase.

**Depends on T1 (§42A-ii):** The first gate condition `!flex_rigid[f]`
requires T1's `flex_rigid` array. Cannot dispatch self-collision without
the rigid-flex skip flag.

Primary changes: `sim/L0/core/src/collision/flex_collide.rs` (dispatch,
`FlexSelfCollide` enum, `mj_collide_flex_internal()`,
`mj_collide_flex_self()`, BVH/SAP midphase),
`sim/L0/core/src/collision/mod.rs` (pipeline integration),
`sim/L0/core/src/types/model.rs` (`flex_selfcollide` type upgrade),
`sim/L0/mjcf/src/builder/flex.rs` (wire `FlexSelfCollide` enum values).

**Spec D — Flex-Flex Cross-Object Collision** (§42A-v / DT-143, 3 sessions — compressed):
Cross-object collision filtering and narrowphase between two different flex
objects. Broadphase filtering via `contype`/`conaffinity` bitmasks in a
unified bodyflex index space. Element-element narrowphase reuses Spec C's
collision primitives. Contact parameter combination via
`contact_param_flex_flex()` following the priority + solmix protocol.

MuJoCo reference:
- `engine_collision_driver.c` → `canCollide2()` — unified bodyflex index
  space where bodies occupy `[0, nbody)` and flexes occupy
  `[nbody, nbody+nflex)`. Bitmask filter:
  `filterBitmask(contype1, conaffinity1, contype2, conaffinity2)`.
- Flex-flex narrowphase: element-element collision tests
  (triangle-triangle for dim=2, tetrahedron-tetrahedron for dim=3).
- Contact parameter combination: priority + solmix protocol matching
  flex-rigid contact params.

**Depends on Spec C (§42A-iv):** Reuses Spec C's element-element narrowphase
primitives (`mj_collide_flex_internal`, `mj_collide_flex_self`). Cannot
implement flex-flex narrowphase without these building blocks.

Primary changes: `sim/L0/core/src/collision/flex_collide.rs`
(`mj_collide_flex_flex()`), `sim/L0/core/src/collision/mod.rs`
(`contact_param_flex_flex()`, pipeline integration).

**T1 — Pre-computed Boolean Arrays + Data Fields** (§42A-ii + §42A-iii):
Two S-effort items combined into a single session.

§42A-ii: Pre-compute `flex_rigid[f]` (true if ALL vertices have
`invmass == 0`) and `flexedge_rigid[e]` (true if BOTH endpoints have
`invmass == 0`) as Model fields during build. Replace per-vertex invmass
checks in flex loops with boolean lookups. MuJoCo ref:
`engine_passive.c` loop structure, `flex_rigid` gate condition.

§42A-iii: Pre-compute `flexedge_length[e]` (Euclidean distance between
endpoints) and `flexedge_velocity[e]` (rate of length change) as Data
fields after forward kinematics. Replace inline computation in all consumers
(edge spring-damper, Newton penalty). MuJoCo ref: `engine_passive.c` field
access patterns, `d->flexedge_length[e]`, `d->flexedge_velocity[e]`.

Both items are optimization + MuJoCo Data layout parity — no behavior change.

---

## Dependency Graph

```
Session 1 (Umbrella)
    |
    +-- Session 2 (T1: §42A-ii + §42A-iii)              <- independent
    |       |
    |       v (hard dep: Spec C needs flex_rigid from §42A-ii)
    |   Sessions 14-18 (Spec C: §42A-iv Self-Collision)
    |       |
    |       v (hard dep: Spec D reuses Spec C narrowphase primitives)
    |   Sessions 19-21 (Spec D: §42A-v Flex-Flex)
    |
    +-- Sessions 3-7 (Spec A: §42A-i Edge Jacobian)     <- independent
    |
    +-- Sessions 8-13 (Spec B: §42B Bending)             <- independent (6 sessions)
```

### Dependency edges

| From -> To | Type | Specific dependency |
|-----------|------|---------------------|
| T1 (§42A-ii) -> Spec C (§42A-iv) | **Hard** | §42A-iv's self-collision gate requires `flex_rigid[f]` from §42A-ii. Cannot dispatch self-collision without the rigid-flex skip flag. |
| Spec C (§42A-iv) -> Spec D (§42A-v) | **Hard** | §42A-v reuses element-element narrowphase primitives from §42A-iv (`mj_collide_flex_internal`, `mj_collide_flex_self`). |
| Spec A (§42A-i) -> Spec B (§42B) | **Soft** | §42B's cotangent bending force application *could* use §42A-i's edge Jacobian for body-attached vertex projection. For free vertices (current test cases), inline application is correct. |

All other deliverables are independent — T1, Spec A, and Spec B can
proceed in parallel. Spec C must wait for T1. Spec D must wait for Spec C.

### Spec B's split implementation: why Sessions 10 and 11 must execute in order

Spec B is split across two implementation sessions at the build-time/runtime
seam. Session 10 (S1-S5) computes `flexedge_flap` topology, cotangent
weights, material stiffness, the 4x4 stiffness matrix, and the curved
reference coefficient — all stored in `Model.flex_bending`. Session 11
(S6-S8) implements the runtime force application loop that reads
`flex_bending`, defines the `FlexBendingModel` trait, and refactors Bridson
into a trait impl.

Session 11 cannot proceed without Session 10: the runtime force loop
(`spring[3*i+x] += b[4*i+j] * xpos[3*v[j]+x]`) reads the precomputed
`flex_bending` coefficients that Session 10 populates. Without populated
coefficients, there is nothing to test. This is a hard internal ordering
within Spec B, analogous to Phase 9 Spec D's GJK distance prerequisite
for conservative advancement.

### Why Spec A and Spec B are independent

§42A-i (edge Jacobian) provides force projection infrastructure for
body-attached flex vertices. §42B (cotangent bending) replaces the bending
force model with MuJoCo's cotangent Laplacian. Both apply forces to
`qfrc_passive` but through different mechanisms:

- §42A-i changes *how* forces are projected onto DOFs (sparse `J^T * F`)
- §42B changes *what* forces are computed (cotangent matrix vs dihedral angle)

For free vertices (all current models), both inline `±direction` and sparse
Jacobian produce identical results. §42B can apply cotangent forces directly
to vertex DOFs without the edge Jacobian. When body-attached vertices are
added (future §27D), both mechanisms combine: cotangent computes forces,
edge Jacobian projects them.

---

## File Ownership Matrix

Files touched by 2+ deliverables, with ownership sequence and handoff state.
Single-owner files are not listed.

### `sim/L0/core/src/types/model.rs` (~1,250 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | T1 (§42A-ii) | Add `flex_rigid: Vec<bool>` and `flexedge_rigid: Vec<bool>` | Rigid skip flags available |
| 2 | Spec A (§42A-i) | Add CSR metadata fields: `flex_edge_J_rownnz`, `flex_edge_J_rowadr`, `flex_edge_J_colind` | CSR structure for edge Jacobian |
| 3 | Spec B (§42B) | Add `flex_bending: Vec<f64>` (17 per edge), `flexedge_flap: Vec<[usize; 2]>`, `flex_bending_model: Vec<FlexBendingType>` | Cotangent bending storage |
| 4 | Spec C (§42A-iv) | Upgrade `flex_selfcollide: Vec<bool>` to `Vec<FlexSelfCollide>` | Enum-based self-collision dispatch |

**Conflict risk: None.** All deliverables add independent fields. Spec C
modifies an existing field type but no other deliverable reads
`flex_selfcollide` before Spec C.

### `sim/L0/core/src/types/data.rs` (~1,084 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | T1 (§42A-iii) | Add `flexedge_length: Vec<f64>` and `flexedge_velocity: Vec<f64>` | Pre-computed edge metrics |
| 2 | Spec A (§42A-i) | Add `flexedge_J: Vec<f64>` (sparse Jacobian data array) | Pre-computed edge Jacobian |

**Conflict risk: None.** Independent new fields.

### `sim/L0/core/src/forward/passive.rs` (~922 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | T1 (§42A-ii) | Replace per-vertex invmass checks with `flex_rigid[f]` / `flexedge_rigid[e]` boolean lookups in edge spring and bending loops | Boolean-gated skip |
| 1 | T1 (§42A-iii) | Replace inline length/velocity computation (lines ~505-545) with `flexedge_length[e]` / `flexedge_velocity[e]` reads | Pre-computed Data reads |
| 2 | Spec A (§42A-i) | Replace inline `±direction` force application (3 sites) with unified `J^T * force` using `flexedge_J` | Sparse Jacobian force projection |
| 3 | Spec B (§42B) | Major rewrite of bending section (lines ~581-699): add `FlexBendingModel` trait, `CotangentBending` impl (matrix-vector), move Bridson code into `BridsonBending` impl | Trait-dispatched bending |

**Conflict risk: Medium.** Three deliverables modify the same flex loops
in `passive.rs`. Ordered execution prevents conflicts: T1 simplifies the
loops (boolean gates + Data reads), Spec A changes force projection, Spec B
rewrites the bending section. Each builds on the previous state.

### `sim/L0/core/src/collision/flex_collide.rs` (~698 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | Spec C (§42A-iv) | Add `FlexSelfCollide` enum, `mj_collide_flex_internal()`, `mj_collide_flex_self()`, BVH/SAP midphase, self-collision dispatch loop | Self-collision paths |
| 2 | Spec D (§42A-v) | Add `mj_collide_flex_flex()` — reuses Spec C's element-element narrowphase primitives | Cross-object collision |

**Conflict risk: None.** Spec D extends Spec C's work. Different functions
in the same file.

### `sim/L0/core/src/collision/mod.rs` (~1,358 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | Spec C (§42A-iv) | Add self-collision dispatch after `mj_collision_flex()` (flex-rigid). Add `contact_param_flex_self()` or reuse `contact_param_flex_rigid()` | Self-collision in pipeline |
| 2 | Spec D (§42A-v) | Add flex-flex dispatch loop after self-collision. Add `contact_param_flex_flex()` | Cross-object collision in pipeline |

**Conflict risk: None.** Spec D extends the dispatch sequence Spec C
establishes. Sequential additions to the collision pipeline.

### `sim/L0/mjcf/src/builder/flex.rs` (~729 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | T1 (§42A-ii) | Compute `flex_rigid` and `flexedge_rigid` after `flexvert_invmass` is populated | Rigid flags set at build time |
| 2 | Spec A (§42A-i) | Allocate CSR metadata (rownnz, rowadr, colind) based on edge/vertex topology | CSR structure ready |
| 3 | Spec B (§42B) | Compute `flexedge_flap` topology from element connectivity, compute `flex_bending` (17 coefficients per edge) from rest geometry + material properties | Cotangent precomputation complete |
| 4 | Spec C (§42A-iv) | Wire `FlexSelfCollide` enum values from parsed `selfcollide` attribute (replace current `bool` push) | Enum-valued self-collision config |

**Conflict risk: Low.** All deliverables add new code sections within the
builder. T1 adds a post-processing step, Spec A adds CSR allocation,
Spec B adds topology + precomputation, Spec C modifies one existing push
statement. Different code sections within `process_flex_bodies()`.

### `sim/L0/core/src/forward/position.rs` (~601 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | T1 (§42A-iii) | Compute `flexedge_length` and `flexedge_velocity` after `mj_flex()` syncs `flexvert_xpos` (kinematics phase) | Pre-computed edge metrics in Data |

**Conflict risk: None.** Single modifier in Phase 10. The computation must
occur after `mj_flex()` (which populates `flexvert_xpos`) and before
`mj_fwd_passive()` (which consumes the pre-computed values).

### `sim/L0/core/src/dynamics/flex.rs` (~19 lines)

Not directly modified by Phase 10. `mj_flex()` syncs `flexvert_xpos` from
body FK — this runs before T1's edge metric computation in `position.rs`.
Listed for reference since `flexvert_xpos` is the input to T1's computation.

### `sim/L0/mjcf/src/types.rs` (~4,247 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| any | Spec B (§42B) | Add `FlexBendingType` enum, add `bending_model` attribute to `MjcfFlex` | Bending model selection |
| any | Spec C (§42A-iv) | Add `FlexSelfCollide` enum (if defined in mjcf types rather than core) | Self-collision algorithm selection |

**Conflict risk: None.** Independent new types/fields.

### `sim/L0/mjcf/src/parser.rs` (~6,025 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| any | Spec B (§42B) | Parse `bending_model` attribute from `<flex>` element | Bending model configurable |
| any | Spec C (§42A-iv) | Update `selfcollide` parsing to produce `FlexSelfCollide` enum (currently produces `Option<String>`) | Enum-valued parsing |

**Conflict risk: None.** Different attributes in the same `<flex>` element.

---

## API Contracts

Cross-spec API dependencies. Sub-specs must be written against these
contracts, not against the current (pre-Phase 10) codebase.

### Contract 1: `flex_rigid` / `flexedge_rigid` (T1 -> Spec C)

**After T1 — expected API:**
```rust
// Model fields (pre-computed at build time)

/// Per-flex: true if ALL vertices have invmass == 0 (entire flex is rigid).
pub flex_rigid: Vec<bool>,

/// Per-edge: true if BOTH endpoint vertices have invmass == 0.
pub flexedge_rigid: Vec<bool>,
```

**What Spec C needs:** `flex_rigid[f]` as the first gate condition in the
self-collision dispatch loop. If `flex_rigid[f]` is true, the entire flex
body is skipped for self-collision — no element-element tests are performed.

**What T1 guarantees:** `flex_rigid[f]` is populated for all `f` in
`0..model.nflex` during model build, after `flexvert_invmass` is set.
The invariant `flex_rigid[f] == true` implies all vertex invmasses are zero
for flex `f`.

### Contract 2: `flexedge_length` / `flexedge_velocity` (T1 -> passive consumers)

**After T1 — expected API:**
```rust
// Data fields (computed after forward kinematics each step)

/// Pre-computed edge lengths (Euclidean distance between endpoints).
pub flexedge_length: Vec<f64>,

/// Pre-computed edge elongation velocities (rate of length change).
pub flexedge_velocity: Vec<f64>,
```

**What passive force consumers need:** Direct read of `data.flexedge_length[e]`
and `data.flexedge_velocity[e]` instead of computing from `flexvert_xpos`
and `qvel`. This is consumed by Spec A (edge spring force magnitude) and
Spec B (bending damper velocity term), but neither has a hard dependency —
they can read the Data fields that T1 establishes.

### Contract 3: Element-element narrowphase primitives (Spec C -> Spec D)

**After Spec C — expected API:**
```rust
/// Narrowphase collision between two triangular elements from different
/// flex bodies (or non-adjacent elements within the same flex body).
/// Returns contact(s) if elements intersect.
fn collide_flex_elements(
    model: &Model,
    data: &Data,
    flex_a: usize,
    elem_a: usize,
    flex_b: usize,
    elem_b: usize,
) -> Vec<Contact>;
```

**What Spec D needs:** Spec D's `mj_collide_flex_flex()` iterates over
element pairs from two different flex bodies and calls Spec C's
narrowphase primitives for each candidate pair. Spec D does NOT re-implement
element-element collision — it reuses Spec C's implementation.

### Contract 4: Contact parameter combination (Spec D convention)

**After Spec D — expected API:**
```rust
/// Combine contact parameters for a flex-flex collision pair.
/// Follows the same priority + solmix protocol as contact_param_flex_rigid().
fn contact_param_flex_flex(
    model: &Model,
    flex1: usize,
    flex2: usize,
) -> (i32, f64, [f64; 2], [f64; 5], Vector3<f64>);
// Returns: (condim, gap, solref, solimp, friction)
```

This is internal to Spec D. No other spec depends on it. The function
mirrors `contact_param_flex_rigid()` (lines 174-216 in `collision/mod.rs`)
but reads `flex_*` arrays for both sides instead of `flex_*` + `geom_*`.

---

## Shared Convention Registry

Conventions decided once here. Sub-specs reference this section instead of
inventing their own.

### 1. Flex data field naming

All new flex fields follow MuJoCo's naming convention with Rust adaptations:

| MuJoCo C field | CortenForge field | Type | Location |
|---------------|-------------------|------|----------|
| `m->flex_rigid` | `model.flex_rigid` | `Vec<bool>` | Model |
| `m->flexedge_rigid` | `model.flexedge_rigid` | `Vec<bool>` | Model |
| `d->flexedge_length` | `data.flexedge_length` | `Vec<f64>` | Data |
| `d->flexedge_velocity` | `data.flexedge_velocity` | `Vec<f64>` | Data |
| `d->flexedge_J` | `data.flexedge_J` | `Vec<f64>` | Data |
| `m->flex_edge_J_rownnz` | `model.flex_edge_J_rownnz` | `Vec<usize>` | Model |
| `m->flex_edge_J_rowadr` | `model.flex_edge_J_rowadr` | `Vec<usize>` | Model |
| `m->flex_edge_J_colind` | `model.flex_edge_J_colind` | `Vec<usize>` | Model |
| `m->flex_bending` | `model.flex_bending` | `Vec<f64>` | Model |
| `m->flexedge_flap` | `model.flexedge_flap` | `Vec<[usize; 2]>` | Model |

**Convention:** Model fields use MuJoCo's exact names (snake_case). Data
fields use MuJoCo's exact names. Index types use `usize` (not `i32`).
Boolean arrays use `Vec<bool>` (not bitfields).

### 2. Force application pattern

All flex passive forces (edge spring, bending, penalty) follow this pattern
in `mj_fwd_passive()`:

```rust
// 1. Check disable flags
if model.opt.disableflags.contains(DisableFlag::SPRING) { /* skip spring */ }
if model.opt.disableflags.contains(DisableFlag::DAMPER) { /* skip damper */ }

// 2. Skip rigid flex/edge
if model.flex_rigid[f] { continue; }
if model.flexedge_rigid[e] { continue; }

// 3. Read pre-computed Data fields
let length = data.flexedge_length[e];
let velocity = data.flexedge_velocity[e];

// 4. Compute forces
let spring_force = ...;
let damper_force = ...;

// 5. Apply via J^T * force (Spec A) or inline ±direction (pre-Spec A)
// Spring -> qfrc_spring
// Damper -> qfrc_damper
```

Sub-specs must use `qfrc_spring` for spring forces and `qfrc_damper` for
damping forces (not `qfrc_passive` directly). This was established in
Phase 5 (S4.7-prereq) and applies to all new force contributions.

### 3. Collision dispatch convention

New flex collision paths follow the existing dispatch pattern in
`collision/mod.rs`. The collision pipeline order is:

```
mj_collision()
    ├── broadphase (SweepAndPrune for rigid-rigid)
    ├── narrowphase (rigid-rigid pairs)
    ├── mj_collision_flex() (flex-rigid pairs)       <- existing
    ├── flex self-collision (Spec C)                   <- Phase 10
    └── flex-flex collision (Spec D)                   <- Phase 10
```

Each collision path generates `Contact` structs following the existing
convention:
```rust
Contact {
    pos: contact_point,       // world-frame contact position
    normal: contact_normal,   // world-frame normal
    depth: penetration_depth, // positive = penetrating
    geom1: geom_a,            // geom index (or flex-encoded index)
    geom2: geom_b,
    // ... solver params from contact_param_*()
}
```

### 4. FlexSelfCollide enum values

```rust
pub enum FlexSelfCollide {
    None   = 0,  // no self-collision
    Narrow = 1,  // brute-force narrowphase (all element pairs)
    Bvh    = 2,  // BVH midphase + narrowphase
    Sap    = 3,  // sweep-and-prune midphase + narrowphase
    Auto   = 4,  // BVH for dim=3, SAP otherwise
}
```

Matches MuJoCo's `mjtFlexSelf` enum. MJCF attribute values: `"none"`,
`"narrow"`, `"bvh"`, `"sap"`, `"auto"`. Default: `None` (no self-collision).

### 5. Bending model selection

```rust
pub enum FlexBendingType {
    /// Wardetzky/Garg cotangent Laplacian (MuJoCo-conformant, default).
    Cotangent,
    /// Bridson dihedral angle springs (nonlinear, large-deformation accurate).
    Bridson,
}
```

MJCF attribute: `bending_model` on `<flex>` element. Default: `Cotangent`
(MuJoCo conformance). This is a CortenForge extension attribute — MuJoCo
only implements the cotangent model.

### 6. Build-time vs runtime convention

- **Build-time computations** (T1 rigid flags, Spec A CSR metadata, Spec B
  cotangent precomputation, Spec C enum wiring): execute during model
  building in `sim/L0/mjcf/src/builder/`. Results are stored on `Model`
  and are immutable after build.
- **Runtime computations** (T1 length/velocity, Spec A Jacobian values,
  Spec B force application, Spec C/D collision dispatch): execute during
  `forward()` in `sim/L0/core/src/`. They read from `Model` and write to
  `Data`.

### 7. Contact parameter combination (flex-flex)

Spec D's `contact_param_flex_flex()` follows the same priority + solmix
protocol as `contact_param_flex_rigid()` (established in Phase 9):

```rust
// If priorities differ: winner takes all parameters
// If priorities equal: solmix-weighted blend of solref/solimp
//   friction: element-wise max
//   condim: max
//   gap: min
```

No changes to the combination protocol — only the input sources change
(both sides read `flex_*` arrays instead of one `flex_*` + one `geom_*`).

---

## Cross-Spec Blast Radius

### Behavioral interactions between specs

| Interaction | Analysis |
|------------|----------|
| T1 (rigid flags) + Spec A (edge Jacobian) | **No conflict.** T1 adds boolean skip gates to flex loops. Spec A changes force projection within those same loops. Both modify `passive.rs` but at different code locations (gate checks vs force application). T1 must land first so Spec A's loops use the optimized gates. |
| T1 (rigid flags) + Spec B (bending) | **No conflict.** T1 adds `flexedge_rigid[e]` skip in edge loops. Spec B rewrites the bending section entirely. T1's bending loop changes are overwritten by Spec B's rewrite — Spec B must incorporate the `flex_rigid[f]` gate. |
| T1 (rigid flags) + Spec C (self-collision) | **Dependency, not conflict.** Spec C consumes T1's `flex_rigid[f]` as gate condition 1. T1 must land before Spec C. No conflicting code changes. |
| T1 (Data fields) + Spec A (edge Jacobian) | **Complementary.** T1 pre-computes `flexedge_length`/`flexedge_velocity`. Spec A reads these when computing edge spring forces via `J^T * force`. Spec A benefits from T1 but doesn't require it (can compute inline if needed). |
| Spec A (edge Jacobian) + Spec B (bending) | **Soft dependency.** Spec B's cotangent force application *could* use Spec A's edge Jacobian for force projection. For free vertices, inline application is correct. The soft dependency is documented and deferred to when body-attached vertices (§27D) are implemented. |
| Spec C (self-collision) + Spec D (flex-flex) | **Dependency, not conflict.** Spec D extends Spec C's dispatch path and reuses Spec C's narrowphase primitives. Spec C must land before Spec D. No conflicting code changes — different dispatch paths in the collision pipeline. |
| Spec A/B (passive) + Spec C/D (collision) | **Independent pipeline stages.** Passive forces and collision detection run in different phases of `forward()`. No shared mutable state, no code overlap. |

### Existing test impact (cross-spec)

| Test area | Touched by | Conflict risk |
|-----------|-----------|--------------|
| Flex passive force tests | T1 (optimization), Spec A (force projection), Spec B (bending rewrite) | **Low.** All three produce identical simulation results. T1 and Spec A are optimization-only (bit-identical). Spec B's cotangent default changes bending values but Bridson regression path preserves old behavior. |
| Flex collision tests | Spec C (new self-collision tests), Spec D (new flex-flex tests) | **None.** Adds new tests. Existing flex-rigid collision tests unchanged. |
| Flex conformance tests | Spec B (cotangent bending changes default) | **Medium.** Any conformance test that checks `qfrc_spring` bending components will see different values under cotangent vs Bridson. Tests that use Bridson-specific expected values must be updated or gated on `bending_model`. |
| Phase 9 regression suite | None expected | **None.** Phase 10 does not modify collision pipeline code from Phase 9. |

### Test count changes

| Deliverable | Estimated new tests | Net change |
|-------------|-------------------|------------|
| T1 (§42A-ii + §42A-iii) | 4-6 (rigid skip, Data field accuracy, regression) | +4-6 |
| Spec A (§42A-i edge Jacobian) | 6-10 (CSR construction, free vertex identity, body-attached projection, consumer migration, regression) | +6-10 |
| Spec B (§42B cotangent bending) | 8-12 (cotangent conformance, Bridson regression, stability, curved reference, boundary edge, trait dispatch, mixed model) | +8-12 |
| Spec C (§42A-iv self-collision) | 8-12 (gate conditions, FlexSelfCollide enum, internal collision, self-collision, BVH/SAP correctness, midphase equivalence) | +8-12 |
| Spec D (§42A-v flex-flex) | 4-6 (bitmask filtering, element-element contacts, contact params, regression) | +4-6 |
| **Total** | **30-46** | **+30-46** |

---

## Phase-Level Acceptance Criteria

These are the aggregate gates that determine "Phase 10 complete." Individual
sub-specs have their own ACs for technical correctness. **The overarching
criterion: CortenForge's flex pipeline is functionally complete for every
feature implemented in Phase 10, matching MuJoCo's behavior.**

### PH10-AC1: All 6 tasks ship-complete
Every task in the assignment table (§42A-i, §42A-ii, §42A-iii, §42A-iv /
DT-142, §42A-v / DT-143, §42B) has landed and its sub-spec ACs (or T1
criteria) are met. Every sub-spec AC that asserts a numerical value has that
value verified against MuJoCo's actual output.

### PH10-AC2: No regression in existing test suite
All 1,900+ domain tests from the post-Phase 9 baseline pass. Zero test
failures attributable to Phase 10 changes.

### PH10-AC3: Quality gate passes
`cargo xtask check` passes (formatting, clippy, all tests).

### PH10-AC4: Aggregate test growth
Domain test count increases by at least 30 (lower bound of per-deliverable
estimates). At least one MuJoCo conformance test (expected value from running
MuJoCo) per sub-spec.

### PH10-AC5: Flex pipeline conformance coverage
After Phase 10, the following MuJoCo flex features are covered:

| Feature | Pre-Phase 10 | Post-Phase 10 |
|---------|-------------|--------------|
| Edge force projection | Inline `±direction` on 3 DOFs | Sparse `J^T * force` via CSR `flexedge_J` |
| Rigid flex skip | Per-vertex invmass check in inner loop | Pre-computed `flex_rigid` / `flexedge_rigid` boolean arrays |
| Edge length/velocity | Inline computation at each consumer | Pre-computed `flexedge_length` / `flexedge_velocity` Data fields |
| Bending model | Bridson dihedral angle springs (diverges from MuJoCo) | Wardetzky/Garg cotangent Laplacian (MuJoCo-conformant default) + Bridson preserved via trait |
| Flex self-collision | Parsed `selfcollide` flag, no dispatch | Full dispatch: FlexSelfCollide enum, internal + self-collision paths, BVH/SAP midphase |
| Flex-flex collision | No flex-flex collision path | `canCollide2()` broadphase filtering + element-element narrowphase |

### PH10-AC6: Cross-spec dependency integrity
- Spec C's self-collision gate correctly uses T1's `flex_rigid[f]` as the
  first gate condition.
- Spec D's `mj_collide_flex_flex()` correctly reuses Spec C's
  element-element narrowphase primitives (not a copy).
- Spec B's `FlexBendingModel` trait dispatches correctly between
  `CotangentBending` and `BridsonBending` implementations.

---

## Out of Scope

Explicitly excluded from Phase 10. Each exclusion states its conformance
impact.

- **GPU flex pipeline** (DT-67) — Post-v1.0 GPU acceleration of flex
  constraint solve. *Conformance impact: none — GPU acceleration does not
  change physics results.*

- **Volume constraints** (DT-73) — No MuJoCo equivalent; emergent from
  continuum model. *Conformance impact: none — not a MuJoCo feature.*

- **NeoHookean / hyperelastic materials** — MuJoCo uses linear elasticity
  for flex. CortenForge matches this. *Conformance impact: none.*

- **Flex equality constraints** (DT-66) — `<equality><flex>` coupling via
  equality constraints. Low priority. *Conformance impact: minimal — affects
  models using flex-flex equality coupling.*

- **Shared-body flex vertices** (DT-87) — Multiple vertices referencing the
  same body's DOFs. Requires §27D (body-attached flex vertices) first.
  *Conformance impact: affects multi-vertex-per-body flex models.*

- **Body-attached flex vertices** (§27D) — Vertices attached to multi-DOF
  bodies. Spec A's edge Jacobian is forward-looking for this. *Conformance
  impact: affects body-attached flex models. Spec A provides the
  infrastructure; §27D provides the vertex attachment mechanism.*

- **Per-vertex material variation** (DT-68) — All vertices share single
  material. Low priority. *Conformance impact: minimal.*

- **Flex adhesion contacts** (DT-72) — Low priority MuJoCo compat.
  *Conformance impact: minimal — affects models using flex adhesion.*

- **SAP for flex broadphase** (DT-69) — Performance optimization for
  flex-rigid broadphase. *Conformance impact: none — brute-force produces
  identical contacts.*

- **`elastic2d` keyword** (DT-86) — Model selection on `<flex><elasticity>`.
  *Conformance impact: minimal — affects models selecting specific 2D
  elasticity modes.*

- **`<composite>` element** (§46) — Phase 13. Procedural body generation.
  *Conformance impact: none for flex pipeline.*

- **Deformable-vs-complex-geom narrowphase** (DT-70) — Phase 9 Spec E
  covers flex-vs-mesh/hfield/SDF. Not part of Phase 10 scope.
  *Conformance impact: covered by Phase 9.*

- **Behavioral friction tests for deformable** (DT-71) — Additional
  friction coverage beyond Phase 8's verification pass. *Conformance
  impact: minimal — existing friction tests cover core behavior.*

- **Flexcomp deferred rendering attributes** (DT-89) — `flatskin`,
  `material`, `rgba` on `<flexcomp>`. *Conformance impact: none —
  rendering attributes do not affect physics.*

- **Collision performance optimizations** (DT-18, DT-24) — Post-v1.0.
  *Conformance impact: none — optimizations do not change physics results.*
