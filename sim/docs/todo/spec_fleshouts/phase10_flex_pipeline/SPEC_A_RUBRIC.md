# Spec A — Sparse Flex Edge Jacobian: Spec Quality Rubric

Grades the Spec A spec on 11 criteria. Target: A+ on every criterion
before implementation begins. A+ means "an implementer could build this
without asking a single clarifying question — and the result would produce
numerically identical output to MuJoCo."

**MuJoCo conformance is the cardinal goal.** Every criterion in this rubric
ultimately serves conformance. P1 (MuJoCo Reference Fidelity) is the most
important criterion — grade it first and hardest. A spec that scores A+ on
all other criteria but has P1 wrong is worse than useless: it would produce
a clean, well-tested implementation of the wrong behavior.

---

## Scope Adjustment

Empirical verification against the MuJoCo C source discovered scope
corrections from the umbrella/future_work claims.

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| "Each edge has 2 rows (one per endpoint vertex)" | Each edge has **ONE row** in `flexedge_J`. The row encodes the differential Jacobian projected along the edge direction. | **Correct in spec** — 1 row per edge, not 2. |
| "Three inline ±direction patterns: edge spring-damper, bending, Newton penalty" | Bending forces do NOT use `flexedge_J` — they apply per-vertex forces directly via `body_dofadr`/`body_dofnum`. No "Newton penalty" consumer exists. | **Drop bending and Newton penalty as consumers.** Actual consumers: (1) edge spring-damper, (2) velocity computation, (3) equality constraint Jacobian. |
| Edge velocity as separate §42A-iii item | MuJoCo computes `flexedge_velocity = J * qvel` using the sparse Jacobian (`mju_mulMatVecSparse` in `mj_fwdVelocity`). T1 already implemented inline velocity computation. Spec A should migrate this to `J * qvel`. | **Add velocity migration** as a consumer site. |
| Constraint Jacobian not mentioned as consumer | MuJoCo's `mj_instantiateEquality` (engine_core_constraint.c) uses `flexedge_J` rows directly as the `efc_J` constraint Jacobian for `mjEQ_FLEX` constraints. Our `assembly.rs:340-349` currently uses inline `±direction`. | **Add constraint assembly** as a consumer site. |

**Final scope:**

1. CSR sparse structure on Model (`flexedge_J_rownnz`, `flexedge_J_rowadr`, `flexedge_J_colind`)
2. Runtime Jacobian computation on Data (`flexedge_J`) — one row per edge, `vec^T * jacdif`
3. Consumer migration: edge spring-damper (passive.rs), velocity (flex.rs), constraint Jacobian (assembly.rs)
4. Free-vertex identity: sparse Jacobian degenerates to `[-vec^T, +vec^T]` on 6 DOFs, bit-identical to current inline
5. Body-attached vertex forward-compatibility: infrastructure ready, deferred to §27D

---

## Empirical Ground Truth

### EGT-1: CSR Structure — ONE row per edge

**MuJoCo source:** `engine_setconst.c` → `makeFlexSparse()`.
**MuJoCo version:** 3.5.0 (empirically verified).

Each edge `e` has exactly **one row** in the sparse Jacobian matrix. The
row encodes the edge-direction-projected differential Jacobian between the
two endpoint bodies. The CSR metadata:

- `flexedge_J_rowadr[e]` — start index in `colind`/`J` arrays for edge `e`
- `flexedge_J_rownnz[e]` — number of non-zeros for edge `e`
- `flexedge_J_colind[j]` — DOF column index for non-zero entry `j`

The sparsity pattern is determined by calling `mj_jacDifPair(m, d, chain,
b1, b2, pos1, pos2, ..., issparse=1)`, which returns the union of DOFs
affecting the relative motion between the two endpoint bodies.

For two free flex vertices (each body has 3 translational DOFs), `NV = 6`
(3 DOFs per body). The `colind` array contains the 6 DOF indices in
kinematic chain order.

Total non-zeros across all edges: `sum(flexedge_J_rownnz[e] for e in 0..nflexedge)`.

**Empirical verification** (MuJoCo 3.5.0, 3-vertex cable with pin on v0):

```
Model: nflexvert=3, nflexedge=2, nv=6
  v0: body=worldbody, dofnum=0 (pinned)
  v1: body=1, dofadr=0, dofnum=3 (free)
  v2: body=2, dofadr=3, dofnum=3 (free)

CSR Structure:
  rownnz = [3, 6]
  rowadr = [0, 3]
  colind = [0, 1, 2, 0, 1, 2, 3, 4, 5]
  Total nnz = 9

Edge 0 (pinned v0 → free v1): rownnz=3 (only v1's DOFs)
Edge 1 (free v1 → free v2):   rownnz=6 (both vertices' DOFs)
```

All-pinned flex: `flex_rigid=[1]`, `flexedge_rigid=[1,1]`, `rownnz=[0,0]`.

### EGT-2: Runtime Jacobian computation — `vec^T * jacdif`

**MuJoCo source:** `engine_core_smooth.c` → `mj_flex()` (lines ~1290-1330).

For each edge `e` with endpoints `v1`, `v2`:

```c
// 1. Compute edge direction + length
vec = pos2 - pos1;
flexedge_length[e] = mju_normalize3(vec);  // normalizes in-place, returns length

// 2. Get differential body Jacobian (3×NV)
NV = mj_jacDifPair(m, d, chain, b1, b2, pos1, pos2,
                    jac1, jac2, jacdif, NULL, NULL, NULL, 1);

// 3. Project along edge direction: J_edge = vec^T * jacdif (1×NV)
mju_mulMatTVec(flexedge_J + rowadr[e], jacdif, vec, 3, NV);
```

**Meaning:** `J_edge[j] = vec · jacdif[:,j]` — the dot product of the unit
edge direction with each column of the 3×NV differential Jacobian.

**For free vertices:**

- `jac1` = 3×3 identity at v1's 3 translational DOFs (body with 3 slide joints)
- `jac2` = 3×3 identity at v2's 3 translational DOFs
- `jacdif = jac2 - jac1` = 3×6 matrix: `[-I, +I]`
- `J_edge = vec^T * [-I, +I]` = `[-vec^T, +vec^T]` (1×6)
- This is exactly the `±direction` pattern our code currently uses inline

**Empirical verification** (MuJoCo 3.5.0, same 3-vertex cable):

```
Edge 0 (pinned v0 → free v1): dir=[1,0,0]
  J = [+1, 0, 0]  (only v1 DOFs; jacdif = jac(v1) since jac(worldbody)=0)

Edge 1 (free v1 → free v2): dir=[1,0,0]
  J = [-1, 0, 0, +1, 0, 0]  ([-vec^T, +vec^T] pattern confirmed)

J*qvel verification (qvel=[0.1, 0.2, -0.3, 0.5, -0.1, 0.4]):
  Edge 0: J*qvel = 0.1 = flexedge_velocity[0] ✓
  Edge 1: J*qvel = 0.4 = flexedge_velocity[1] ✓

J^T*force verification (v1 displaced +0.05 in x):
  Edge 0: len=0.55, frc=-5.0, J^T*(-5) adds [-5,0,0] to DOFs [0,1,2]
  Edge 1: len=0.45, frc=+5.0, J^T*(+5) adds [-5,0,0,+5,0,0] to DOFs [0..5]
  qfrc_spring = [-10, 0, 0, 5, 0, 0] ✓ (matches MuJoCo output exactly)
```

**For body-attached vertices (§27D, future):**

- `jac1` = `mj_jac(b1, pos1)` — full body Jacobian at vertex position
- `jac2` = `mj_jac(b2, pos2)`
- `jacdif = jac2 - jac1` — captures full kinematic chain difference
- `J_edge = vec^T * jacdif` — correct projection through multi-DOF bodies

### EGT-3: Skip conditions

**MuJoCo source:** `engine_core_smooth.c` → `mj_flex()`.

J computation is skipped for:

1. `flex_rigid[f]` — entire flex is rigid (all vertices have invmass == 0)
2. `flex_interp[f]` — interpolated flex (we don't have this field; irrelevant)
3. `skipjacobian` — no passive force needed: `!flex_edgeequality[f] && !flex_edgedamping[f] && !flex_edgestiffness[f] && !flex_damping[f]`

Note: `flex_edgeequality` doesn't exist in CortenForge (we have edge
constraints enabled per-flex via `flex_edge_solref`/`flex_edge_solimp`).
The skip condition may need adaptation.

**Empirical verification** (MuJoCo 3.5.0, 2D shell with `flex_edgestiffness=0`,
`flex_edgedamping=0`, `flex_damping=0.1`): J is still computed for all edges
(all `flexedge_J` values nonzero). Confirms that `flex_damping` (vertex-level
damping, used for bending) triggers J computation even though bending itself
doesn't read `flexedge_J`. This is a conservative optimization in MuJoCo —
our skip condition should match.

### EGT-4: Consumer site 1 — Edge spring-damper passive forces

**MuJoCo source:** `engine_passive.c` → `mj_springdamper()` (lines ~671-679).

```c
// J^T * force for edge spring-damper
int end = m->flexedge_J_rowadr[e] + m->flexedge_J_rownnz[e];
for (int j = m->flexedge_J_rowadr[e]; j < end; j++) {
    int colind = m->flexedge_J_colind[j];
    mjtNum J = d->flexedge_J[j];
    d->qfrc_spring[colind] += J * frc_spring;
    d->qfrc_damper[colind] += J * frc_damper;
}
```

**Current CortenForge:** `passive.rs:541-564` — applies `±direction * force`
to `dof0+ax` and `dof1+ax` (3 DOFs per vertex). This is the inline pattern
that Spec A replaces.

### EGT-5: Consumer site 2 — Edge velocity computation

**MuJoCo source:** `engine_forward.c` → `mj_fwdVelocity()`.

```c
mju_mulMatVecSparse(d->flexedge_velocity, d->flexedge_J, d->qvel,
                    m->nflexedge,
                    m->flexedge_J_rownnz, m->flexedge_J_rowadr,
                    m->flexedge_J_colind, NULL);
```

`flexedge_velocity[e] = J[e,:] · qvel` — sparse matrix-vector product.

**Current CortenForge:** `dynamics/flex.rs:27-57` (`mj_flex_edge()`) computes
velocity inline from `flexvert_xpos` and `qvel`. After Spec A, this should
use the sparse Jacobian: `velocity[e] = sum(J[j] * qvel[colind[j]])`.

### EGT-6: Consumer site 3 — Edge equality constraint Jacobian

**MuJoCo source:** `engine_core_constraint.c` → `mj_instantiateEquality()`,
case `mjEQ_FLEX`.

```c
// Sparse mode: pass J row directly as constraint Jacobian
mj_addConstraint(m, d, d->flexedge_J + m->flexedge_J_rowadr[e],
                 cpos, 0, 0, 1, mjCNSTR_EQUALITY, i,
                 m->flexedge_J_rownnz[e],
                 m->flexedge_J_colind + m->flexedge_J_rowadr[e]);
```

**Current CortenForge:** `constraint/assembly.rs:340-349` — populates
`efc_J` inline using `±direction[k]` at `dof0+k` and `dof1+k`. After
Spec A, this should read from `flexedge_J` directly.

### EGT-7: Bending does NOT use flexedge_J

**MuJoCo source:** `engine_passive.c` bending section (lines ~520-530).

Bending forces are applied directly per-vertex using `body_dofadr`/`body_dofnum`:

```c
for (int i = 0; i < 4; i++) {
    int bid = bodyid[v[i]];
    int body_dofadr = m->body_dofadr[bid];
    for (int x = 0; x < body_dofnum; x++) {
        d->qfrc_spring[body_dofadr+x] -= spring[3*i+x];
    }
}
```

Bending operates on a 4-vertex diamond stencil, not on edge forces. The
edge Jacobian projects scalar edge forces, not 4-vertex stencil forces.
**Bending is NOT a consumer of `flexedge_J`.**

### Codebase Context

**Files to modify (with line ranges):**

| File | Line range | Current state | Spec A change |
|------|-----------|---------------|---------------|
| `sim/L0/core/src/types/model.rs` | 438-450 | Per-edge arrays section | Add `flexedge_J_rownnz`, `flexedge_J_rowadr`, `flexedge_J_colind` |
| `sim/L0/core/src/types/data.rs` | 124-130 | Flex edge pre-computed section | Add `flexedge_J: Vec<f64>` |
| `sim/L0/core/src/types/model_init.rs` | 490-491 | `flexedge_length`/`velocity` init | Add `flexedge_J` init with correct size |
| `sim/L0/core/src/dynamics/flex.rs` | 21-57 | `mj_flex_edge()` inline velocity | Rewrite: compute `flexedge_J`, then `velocity = J * qvel` |
| `sim/L0/core/src/forward/passive.rs` | 496-565 | Edge spring-damper with inline `±direction` | Replace force application with sparse `J^T * force` loop |
| `sim/L0/core/src/constraint/assembly.rs` | 336-349 | Constraint Jacobian with inline `±direction` | Read from `flexedge_J`/`colind` instead of computing inline |
| `sim/L0/mjcf/src/builder/flex.rs` | ~300+ | Edge processing in `process_flex_bodies()` | Allocate CSR structure (compute rownnz, rowadr, colind) |

**Existing tests that exercise these paths:**

- `sim/L0/tests/integration/flex_unified.rs` — 43 flex tests covering edge
  springs, bending, constraint assembly, vertex dynamics
- Conformance tests in `sim-conformance-tests` that use flex models
- Any test asserting `qfrc_spring`, `qfrc_damper`, or `efc_J` values for
  flex models

**Match sites (exhaustive enum arms, manual Clone/Default, size guards):**

- `model_init.rs:190` — `flexedge_length0: vec![]` pattern; new CSR fields
  must follow same init pattern
- `data.rs:658-659` — Clone impl for flex Data fields; add `flexedge_J`
- `data.rs:943-944` — reset/zero for flex Data fields; add `flexedge_J`
- Builder `process_flex_bodies()` — CSR allocation must occur after edge
  extraction (edges are extracted before material assignment)

---

## Criteria

> **Criterion priority:** P1 (MuJoCo Reference Fidelity) is the cardinal
> criterion. Grade P1 first and hardest. If P1 is not A+, do not proceed.

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does — exact function names, field
> names, calling conventions, and edge cases. No hand-waving.

| Grade | Bar |
|-------|-----|
| **A+** | Every MuJoCo function cited with source file and exact behavior: `mj_flex()` in `engine_core_smooth.c` for J computation, `makeFlexSparse()` in `engine_setconst.c` for CSR structure, `mj_springdamper()` in `engine_passive.c` for J^T*force, `mj_fwdVelocity()` in `engine_forward.c` for J*qvel, `mj_instantiateEquality()` in `engine_core_constraint.c` for efc_J. Edge cases addressed: zero-length edges (dist < epsilon → skip J), rigid flex skip (`flex_rigid[f]`), pinned vertices (`dofadr=usize::MAX` → rownnz excludes pinned body's DOFs, per EGT-1 empirical data), degenerate edge with both vertices on same body (§27D-only, impossible for current free-vertex models — noted as forward-compat). Scope correction from EGT-7 (bending NOT a consumer) explicitly documented. One-row-per-edge structure confirmed against source and empirically (not the umbrella's "2 rows" claim). |
| **A** | MuJoCo behavior described correctly from C source. Minor gaps in edge-case coverage. |
| **B** | Correct at high level but missing specifics or based on docs rather than C source. |
| **C** | Partially correct. Some MuJoCo behavior misunderstood or invented. |

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. No "see MuJoCo source"
> or "TBD" gaps.

| Grade | Bar |
|-------|-----|
| **A+** | CSR allocation algorithm fully specified: per-edge `rownnz` from DOF count of two endpoint bodies, `rowadr` as cumulative sum, `colind` from body DOF ranges. Skip condition specified: CortenForge equivalent of MuJoCo's `skipjacobian = !flex_edgeequality[f] && !flex_edgedamping[f] && !flex_edgestiffness[f] && !flex_damping[f]`, with explicit mapping for `flex_edgeequality` (which doesn't exist in CortenForge). J computation fully specified: `vec = normalize(pos2-pos1)`, `jacdif = jac(b2,pos2) - jac(b1,pos1)`, `J_edge = vec^T * jacdif`. Length and J share the direction normalization (no double computation). Free-vertex degenerate case proven: `J_edge = [-vec^T, +vec^T]` when both bodies are free 3-DOF. Force application loop specified: `qfrc[colind[j]] += J[j] * frc`. Velocity loop specified: `velocity[e] = sum(J[j] * qvel[colind[j]])`. Constraint consumer specified: scatter flexedge_J values at colind positions into dense efc_J row. An implementer can type each algorithm without reading MuJoCo source. |
| **A** | Algorithm is complete and MuJoCo-conformant. One or two minor details left implicit. |
| **B** | Algorithm structure is clear but some steps are hand-waved. |
| **C** | Skeleton only. |

### P3. Convention Awareness

> Spec explicitly addresses our codebase's conventions where they differ
> from MuJoCo.

| Grade | Bar |
|-------|-----|
| **A+** | Convention difference table present with explicit porting rules: MuJoCo `flex_edge[2*(ebase+e)]` vs CortenForge `flexedge_vert[e]` vertex indexing; MuJoCo `flex_vertbodyid[vbase+v]` vs CortenForge `flexvert_bodyid[v]` (global vs flex-local indexing); MuJoCo `body_dofadr`/`body_dofnum` vs CortenForge `flexvert_dofadr` (vertex DOF address); MuJoCo `mj_jacDifPair()` vs CortenForge `mj_jac()` (our `mj_jac` returns dense `DMatrix<f64>`, needs sparse adaptation or inline computation); nalgebra `Vector3<f64>` vs MuJoCo `mjtNum*` pointer arithmetic; `usize::MAX` sentinel for pinned vertices vs MuJoCo negative indices. Each porting rule verified to preserve numerical equivalence. |
| **A** | Major conventions documented. Minor field-name mappings left to implementer. |
| **B** | Some conventions noted, others not. |
| **C** | MuJoCo code pasted without adaptation. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. Contains concrete values,
> tolerances, and model configurations.

| Grade | Bar |
|-------|-----|
| **A+** | Every runtime AC has three-part structure: (1) concrete input model, (2) exact expected value or tolerance, (3) field to check. Free-vertex identity AC: "For a 2-vertex cable with edge stiffness, `qfrc_spring` values after Spec A are bit-identical to pre-Spec-A values." CSR structure AC: "For a 4-vertex square shell, `rownnz` = [6, 6, 6, 6, 6] (each edge between two 3-DOF free vertices)." Velocity AC: "J*qvel produces identical `flexedge_velocity` to inline computation for a cable under gravity after 10 steps." Constraint AC: "efc_J row for edge e matches flexedge_J values at colind positions." At least one AC states MuJoCo-derived expected value. |
| **A** | ACs are testable. Some lack exact numerical expectations. |
| **B** | ACs are directionally correct but vague. |
| **C** | ACs are aspirational statements. |

### P5. Test Plan Coverage

> Tests cover happy path, edge cases, regressions, and interactions.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present. Edge case inventory: zero-length edge (skip J), pinned vertex (zero J contribution, `dofadr=usize::MAX`), fully rigid flex (skip entire flex), mixed rigid/non-rigid edges within one flex, single-vertex flex (no edges), `nflexedge=0` (no edges at all), both vertices on same body (future §27D). Negative cases: rigid flex produces no J computation; disabled spring+damper still computes J for constraint consumer. Free-vertex regression test: simulation results bit-identical before/after Spec A. At least one MuJoCo conformance test (compare J*qvel velocity against running MuJoCo on same model). Non-trivial model test: multi-flex model with different edge counts. |
| **A** | Good coverage. Minor edge-case gaps. |
| **B** | Happy path covered. Edge cases sparse. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and interactions with other specs.

| Grade | Bar |
|-------|-----|
| **A+** | Prerequisites stated: T1 (Session 2, commit `1f0230c`) provides `flex_rigid`, `flexedge_rigid`, `flexedge_length`, `flexedge_velocity`. Spec A is independent of Spec B (bending) and Spec C/D (collision). Cross-spec interaction: Spec A's `flexedge_J` is available for Spec B's cotangent bending to use for body-attached vertices (soft dep, not required for free vertices). Execution order unambiguous: S1 (CSR structure on Model) → S2 (J computation on Data) → S3 (consumer migration: passive) → S4 (consumer migration: velocity) → S5 (consumer migration: constraint). |
| **A** | Order is clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Spec identifies every file touched, every behavior that changes, and every
> existing test that might break.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description (7 files, per Codebase Context table). Behavioral changes: **none** — for free vertices, sparse J^T*force produces identical results to inline `±direction`. This is a structural refactor, not a behavior change. All 43 flex_unified.rs tests must pass unchanged. All conformance tests must pass unchanged. No expected value changes. Data staleness: `model_init.rs` must allocate `flexedge_J` with correct size (total non-zeros from CSR structure). `data.rs` reset must zero `flexedge_J`. Builder must compute CSR metadata after edge extraction. Assembly.rs migration from inline to J-read is numerically equivalent. |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions within the spec. Shared concepts use identical
> terminology throughout.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology uniform: "edge Jacobian" (not "edge J" in one place and "flex Jacobian" in another). Consumer count consistent: 3 consumers stated in MuJoCo Reference, 3 sections in Specification, 3 migration ACs. File paths match between Specification and Files Affected. AC numbers match between AC section and Test Plan traceability. Edge case lists consistent across MuJoCo Reference and Test Plan. CSR field names identical everywhere: `flexedge_J_rownnz`, `flexedge_J_rowadr`, `flexedge_J_colind`, `flexedge_J`. |
| **A** | Consistent. One or two minor terminology inconsistencies. |
| **B** | Some sections use different names for the same concept. |
| **C** | Contradictions between sections. |

### P9. Consumer Migration Completeness

> Every inline `±direction` force/velocity/Jacobian application site is
> identified and migrated to use the sparse Jacobian. No orphan sites.

**Boundary with P1:** P1 grades whether the spec correctly identifies what
MuJoCo does (which sites use `flexedge_J`). P9 grades whether the spec
comprehensively migrates all CortenForge inline sites to use the new
infrastructure.

| Grade | Bar |
|-------|-----|
| **A+** | All 3 consumer sites identified with exact file:line: (1) `passive.rs:541-564` edge spring-damper → sparse `J^T * force` loop, (2) `dynamics/flex.rs:27-57` velocity → `velocity[e] = sum(J[j] * qvel[colind[j]])`, (3) `assembly.rs:340-349` constraint Jacobian → scatter `flexedge_J` values at `colind` positions into dense `efc_J` row (our assembly uses dense `efc_J`, not sparse). Each site has a before/after code comparison showing the migration. Proof that no other inline `±direction` sites exist (grep for `direction[ax]` or `dof0 + ax` patterns in flex code). Migration preserves numerical identity for free vertices. Each migrated site has a dedicated test verifying equivalence. |
| **A** | All sites identified and migrated. Minor before/after gaps. |
| **B** | Most sites identified. One site missed or migration incomplete. |
| **C** | Incomplete consumer enumeration. |

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P9 | Specification S3-S5 (consumer migration sections), Files Affected |

### P10. Sparse Matrix Format Correctness

> The CSR sparse format is specified correctly: allocation, indexing,
> degenerate cases, and interaction with CortenForge's data layout.

**Boundary with P2:** P2 grades algorithmic completeness (every step specified).
P10 grades the CSR-specific correctness (indexing, sizing, edge cases in
sparse format).

| Grade | Bar |
|-------|-----|
| **A+** | CSR allocation formula: `total_nnz = sum(rownnz)`, `rowadr[0] = 0`, `rowadr[e] = rowadr[e-1] + rownnz[e-1]`. Sizing for free vertices proven: each edge has `rownnz = dof_count(b1) + dof_count(b2)`, for two 3-DOF free bodies this is 6. Degenerate cases: edge with both vertices pinned (`rownnz = 0`), edge with one pinned vertex (`rownnz = dof_count(other_body)`), edge where both vertices belong to same body (`rownnz = dof_count(body)`, jacdif = jac(pos2) - jac(pos1)` on same body). `colind` ordering: DOFs in kinematic chain order (matching `mj_jacDifPair` output). Data array `flexedge_J` sized to `total_nnz`. When `nflexedge = 0`, all CSR arrays are empty. |
| **A** | CSR format correct. Minor degenerate-case gaps. |
| **B** | CSR structure described but sizing or indexing incomplete. |
| **C** | CSR format not properly specified. |

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P10 | Specification S1 (CSR structure), Model fields, Builder allocation |

### P11. Pipeline Integration

> The Jacobian computation is placed in the correct pipeline stage, with
> correct data dependencies and ordering relative to FK, velocity, and
> passive force computation.

**Boundary with P6:** P6 grades cross-spec dependencies. P11 grades
intra-pipeline ordering (which stage computes what, data flow between
stages).

| Grade | Bar |
|-------|-----|
| **A+** | Pipeline placement matches MuJoCo: (1) CSR structure computed once at model build time in builder (Model field), (2) J values and `flexedge_length` computed together in position stage after `mj_flex()` populates `flexvert_xpos` — length and J share the direction normalization step (no double computation, per R10), (3) `flexedge_velocity = J * qvel` computed after J (can remain in position stage since `qvel` is available, or move to velocity stage to match MuJoCo's `mj_fwdVelocity` placement — spec must state which and justify), (4) passive force consumers read J in `mj_fwd_passive()`, (5) constraint consumer reads J in constraint assembly. Data dependency chain explicit: `mj_flex()` → `flexvert_xpos` → `mj_flex_edge()` → `flexedge_J` + `flexedge_length` → `mj_fwd_passive()` reads J. |
| **A** | Pipeline placement correct. Minor ordering justification gaps. |
| **B** | Computation placed in correct stage but data dependencies unclear. |
| **C** | Pipeline placement wrong or unspecified. |

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P11 | Specification (computation placement), Execution Order, Prerequisites |

---

## Rubric Self-Audit

### Self-audit checklist

- [x] **Specificity:** Every A+ bar names specific MuJoCo functions
      (`mj_flex`, `makeFlexSparse`, `mj_springdamper`, `mj_fwdVelocity`,
      `mj_instantiateEquality`), specific CortenForge files/lines
      (`passive.rs:541-564`, `assembly.rs:340-349`, `dynamics/flex.rs:27-57`),
      and specific edge cases (zero-length, pinned, rigid, both-on-same-body).
      Two reviewers would agree on grades by pointing to spec lines.

- [x] **Non-overlap:** P1 vs P9 boundary: P1 grades MuJoCo reference
      accuracy (what MuJoCo does), P9 grades CortenForge migration
      completeness (every inline site found and migrated). P2 vs P10
      boundary: P2 grades algorithmic steps, P10 grades CSR-specific
      format correctness. P6 vs P11 boundary: P6 grades cross-spec deps,
      P11 grades intra-pipeline ordering.

- [x] **Completeness:** 11 criteria cover: MuJoCo fidelity (P1), algorithm
      (P2), conventions (P3), ACs (P4), tests (P5), deps (P6), blast
      radius (P7), consistency (P8), consumer migration (P9), sparse
      format (P10), pipeline (P11). No meaningful gap dimension remains.

- [x] **Gradeability:** Criterion→section mapping provided for P9-P11.
      P1-P8 use standard mapping from rubric template.

- [x] **Conformance primacy:** P1 cites 5 specific C source files.
      A+ bar requires verification against C source. P4 requires
      MuJoCo-derived expected values. P5 requires MuJoCo conformance
      tests. Scope Adjustment section corrects umbrella claims against
      actual C source (EGT-7: bending NOT a consumer).

- [x] **Empirical grounding:** 7 EGT entries verified against C source:
      CSR structure (EGT-1), J computation (EGT-2), skip conditions
      (EGT-3), 3 consumer sites (EGT-4/5/6), bending non-consumer
      (EGT-7). Codebase context with file:line references. All A+ bars
      reference EGT findings.

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
| P9 | Specification S3-S5 (consumer migration), Files Affected |
| P10 | Specification S1 (CSR structure), Model fields, Builder allocation |
| P11 | Specification (computation placement), Execution Order, Prerequisites |

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
| P9. Consumer Migration Completeness | | |
| P10. Sparse Matrix Format Correctness | | |
| P11. Pipeline Integration | | |

**Overall: Rubric Rev 1 — ready for spec grading**

> **Grading note:** The Evidence column is not optional. For each grade, cite
> the specific spec content that justifies it.

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| R1 | Scope | Umbrella claims "2 rows per edge" — MuJoCo has 1 row per edge | Rubric Phase 1 (C source verification) | Scope Adjustment table documents correction. P1 A+ bar requires 1-row confirmation. | Rubric Rev 1 |
| R2 | P9/Scope | Umbrella claims 3 consumers: "edge spring-damper, bending, Newton penalty" — bending and Newton penalty do NOT use flexedge_J | Rubric Phase 1 (EGT-7) | Scope Adjustment drops bending and Newton penalty. Actual consumers: edge spring-damper, velocity, constraint Jacobian. | Rubric Rev 1 |
| R3 | P9 | Constraint assembly (assembly.rs:340-349) not mentioned as consumer in umbrella | Rubric Phase 1 (EGT-6 from engine_core_constraint.c) | Added as consumer site 3. P9 A+ bar requires 3 sites. | Rubric Rev 1 |
| R4 | P10 | CSR degenerate cases not covered: pinned vertex, same-body vertices, nflexedge=0 | Rubric self-audit | P10 A+ bar explicitly lists degenerate cases. | Rubric Rev 1 |
| R5 | P11 | Pipeline placement needs to specify whether velocity moves to velocity stage | Rubric self-audit | P11 A+ bar requires explicit placement decision with justification. | Rubric Rev 1 |
| R6 | P3 | MuJoCo uses mj_jacDifPair (sparse differential Jacobian) — CortenForge has only dense mj_jac(). Need convention mapping. | Rubric Phase 1 (codebase context) | P3 A+ bar requires mj_jacDifPair → mj_jac() porting rule. | Rubric Rev 1 |
| R7 | P1 | "Both vertices on same body (NV=0 → skip)" is impossible for free vertices — each vertex has its own body. Only applies to future §27D body-attached vertices. | Stress-test audit | P1 A+ bar edge case list should clarify "both on same body" is §27D-only. Spec should still handle it for forward-compat but mark as untestable until §27D. | Rubric Rev 2 |
| R8 | P2 | `flex_edgeequality` exists in MuJoCo (confirmed empirically: `flex_edgeequality=[0]`) but not in CortenForge. Skip condition adaptation needed: spec must define what CortenForge uses in place of `flex_edgeequality` (e.g., check `flex_edge_solref` or always compute J). | Stress-test audit | P2 A+ bar should require spec to specify CortenForge's skip condition with explicit mapping from MuJoCo's `skipjacobian` condition. | Rubric Rev 2 |
| R9 | P9 | Constraint assembly consumer migration: should the spec specify dense scatter (copy sparse J into dense efc_J row) matching our current dense constraint assembly? Our `assembly.rs` uses dense `efc_J` matrix, not sparse. | Stress-test audit | P9 A+ bar should require before/after for constraint consumer, noting that CortenForge uses dense efc_J and the migration is "scatter flexedge_J values at colind positions into efc_J row." | Rubric Rev 2 |
| R10 | P11 | `flexedge_length` is computed alongside J in MuJoCo's `mj_flex()` (shared normalization). T1 already computes length in `mj_flex_edge()`. After Spec A, length and J should share the normalization step. P11 bar should note this coupling. | Stress-test audit | P11 A+ bar updated to require that length and J share the direction computation (avoid double normalization). | Rubric Rev 2 |
| R11 | P4 | Empirical values now available from MuJoCo 3.5.0 stress tests: (1) CSR rownnz=[3,6] for pinned+free cable, (2) J=[-0.7071,-0.7071,0,+0.7071,+0.7071,0] for diagonal edge, (3) qfrc_spring=[-10,0,0,5,0,0] for stretched cable. P4 A+ bar should require spec to include these exact values as AC expected values. | Stress-test audit | Empirical values added to EGT-1 and EGT-2. P4 bar already requires MuJoCo-derived expected values. | Rubric Rev 2 |
