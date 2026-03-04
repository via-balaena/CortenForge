# Phase 8 — Constraint & Solver Gaps: Umbrella Spec

**Status:** Draft
**Phase:** Roadmap Phase 8
**Tasks:** 6 active (DT-19, DT-23, DT-25, DT-28, DT-33, DT-39) + 1 pre-completed (DT-32)
**Deliverables:** 1 T1 session + 2 sub-specs (Spec A, B) + 1 verification pass (DT-25)
**Test baseline:** 1,900+ domain tests (post-Phase 7)

---

## Scope

Phase 8 closes all constraint formulation and solver completeness gaps in the
v1.0 roadmap. **The goal is MuJoCo conformance** — after Phase 8, CortenForge
correctly wires per-DOF friction loss solver parameters with defaults cascade,
activates tendon limit constraints with the correct margin, validates multi-DOF
joints in fixed tendons, computes the body-weight diagonal approximation for
constraint regularization, projects friction forces onto the QCQP cone surface,
and verifies deformable-rigid friction correctness.

Unlike Phases 5–6 (single-domain depth passes) or Phase 7 (heterogeneous
parsing breadth), Phase 8 is a **constraint/solver depth pass**: all tasks
converge on the unified constraint pipeline (`assembly.rs`, `solver/*.rs`).
Tasks are grouped by design complexity and shared code paths:

- **Spec A** (DT-23, DT-33): Solver param wiring and constraint activation —
  "finish items" that close remaining gaps in the parameter lifecycle
  (parse → defaults cascade → model field → constraint assembly).
- **Spec B** (DT-19): QCQP cone projection — verifying and completing
  MuJoCo-conformant per-contact friction cone enforcement during solver
  iterations. Substantial infrastructure exists (`project_elliptic_cone`,
  `noslip_qcqp2/3`); Spec B verifies conformance and closes any gaps.
- **T1 session** (DT-28, DT-39): Two algorithmic tasks — tendon DOF mapping
  validation and body-weight diagonal approximation.
- **DT-25 verification** (1 session): Deformable-rigid friction correctness
  audit, downgraded from full spec after infrastructure review showed the
  friction pipeline already exists.

**Pre-completed:** DT-32 (per-tendon `solref_limit`/`solimp_limit` naming
conformance) was completed before this umbrella as a naming-only refactor.
Commit: `9f9cf9f`.

> **Conformance mandate for sub-spec authors:** Each sub-spec must cite the
> exact MuJoCo C source (function, file, line range) for every behavior it
> implements. Acceptance criteria must include expected values derived from
> MuJoCo's output — not from hand-calculation or intuition. The MuJoCo C
> source code is the single source of truth. When the MuJoCo docs and the
> C source disagree, the C source wins.

---

## Task Assignment

Every Phase 8 task is assigned to exactly one deliverable. No orphans, no
overlaps. Each task closes a specific MuJoCo conformance gap.

| Task | Description | Deliverable | Status | Rationale |
|------|-------------|-------------|--------|-----------|
| ~~DT-32~~ | Per-tendon `solref_limit`/`solimp_limit` naming conformance | Pre-spec | **Done** (`9f9cf9f`) | Naming-only; no functional change |
| DT-23 | Per-DOF friction loss solver params — verify multi-DOF fan-out + test coverage | **Spec A** | Pending | Finish item: fields, parsing, defaults cascade, builder wiring, and assembly wiring all exist. Verify multi-DOF fan-out (ball/free) + add non-default tests. |
| DT-33 | Tendon `margin` attribute — limit activation distance | **Spec A** | Pending | Finish item: parsing + defaults cascade exist. Add model field + builder wiring + replace 4 hardcoded `< 0.0` in assembly.rs:149,153,541,564. |
| DT-28 | Ball/Free joints in fixed tendons — validation + DOF mapping | **T1** | Pending | Validate `qvel` DOF index mapping for multi-DOF joints in `compute_fixed_tendon_length()` |
| DT-39 | Body-weight diagonal approximation (`diagApprox`) | **T1** | Pending | Add `body_invweight0`/`dof_invweight0` to Model + `mj_diagApprox()` |
| DT-19 | QCQP-based cone projection for normal+friction force projection | **Spec B** | Pending | Infrastructure exists (`project_elliptic_cone`, `noslip_qcqp2/3`); verify conformance against `mj_projectConstraint()`, close gaps |
| DT-25 | Deformable-rigid friction — correctness verification | **Verification** | Pending | Downgraded from T3 spec; infrastructure exists, verify + document gaps |

### Sub-spec scope statements

Each sub-spec must identify the exact MuJoCo C functions it is porting, cite
them with source file and line ranges, and produce acceptance criteria with
MuJoCo-verified expected values.

**Spec A — Solver Param & Margin Completeness** (DT-23, DT-33):
Two constraint parameter lifecycle items that share the same parse → defaults →
model → assembly pipeline:

- **DT-23 (Per-DOF friction loss solver params):** This is a **finish item**.
  The full pipeline is substantially implemented:
  - **Model fields**: `dof_solref: Vec<[f64; 2]>` (model.rs:226) and
    `dof_solimp: Vec<[f64; 5]>` (model.rs:229) exist.
  - **Assembly wiring**: Already wired into `finalize_row!` in assembly.rs
    (lines 380-381) for DOF friction loss rows.
  - **MJCF parsing**: `solreffriction`/`solimpfriction` are already parsed
    from `<joint>` elements (parser.rs:1787-1798) and stored as
    `Option<[f64; 2]>` / `Option<[f64; 5]>` on `MjcfJoint` (types.rs:614-616).
  - **Defaults cascade**: `MjcfJointDefaults` has `solreffriction` /
    `solimpfriction` fields, parsed in defaults (parser.rs:669-673), cascaded
    via `apply_to_joint()` (defaults.rs:209-213).
  - **Builder wiring**: `builder/joint.rs:160-163` populates `dof_solref`/
    `dof_solimp` from `joint.solreffriction.unwrap_or(DEFAULT_SOLREF)`.

  What remains is **verification and test coverage**:
  1. **Multi-DOF joints**: The builder currently does a single `push()` per
     joint call. Verify ball joint (3 DOFs) and free joint (6 DOFs) all
     receive identical values from their parent joint — MuJoCo copies to
     all DOFs of the joint.
  2. **Test coverage**: Add tests with non-default `solreffriction` values and
     verify different friction loss dynamics (no such tests exist currently).
  3. **Tendon friction solref**: Verify `tendon_solref_fri`/`tendon_solimp_fri`
     are similarly wired end-to-end (parsing → builder → assembly).
  MuJoCo reference: `getsolparam()` in `engine_core_constraint.c` — case
  `mjCNSTR_FRICTION_DOF` reads `m->dof_solref[mjNREF*id]` directly. Compile-
  time default chain: `mj_defaultSolRefImp()` → `<default><joint>` →
  `<joint solreffriction="...">` → `CopyTree()` per-DOF copy.

- **DT-33 (Tendon `margin`):** This is a **finish item** — parsing and
  defaults already exist:
  - **MJCF parsing**: `margin` is already parsed from `<tendon>` elements
    (parser.rs:3646) as `MjcfTendon.margin: Option<f64>` (types.rs:2808).
  - **Defaults cascade**: `MjcfTendonDefaults.margin` exists (types.rs:805),
    parsed (parser.rs:986), cascaded (defaults.rs:644-646).

  What remains:
  1. **Model field**: Add `tendon_margin: Vec<f64>` to Model (does not exist).
  2. **Builder wiring**: Wire parsed `margin` through the tendon builder to
     populate the model field.
  3. **Assembly modification**: Replace 4 hardcoded `< 0.0` activation checks
     in assembly.rs with margin-aware checks:
     - Line 149: `if length - limit_min < 0.0` → `< tendon_margin[t]`
     - Line 153: `if limit_max - length < 0.0` → `< tendon_margin[t]`
     - Line 541: `if dist_lower < 0.0` → `< tendon_margin[t]`
     - Line 564: `if dist_upper < 0.0` → `< tendon_margin[t]`
  4. **finalize_row! margin arg**: Pass `tendon_margin[t]` instead of
     hardcoded `0.0` to `finalize_row!`.

  Phase 7 Spec B (§64a) implemented the identical pattern for joints
  (`jnt_margin` at model.rs:208, used at 4+ sites in assembly.rs). This is
  the direct reference pattern.
  MuJoCo reference: `tendon_margin[ntendon]` in `mjModel` (`mjmodel.h`);
  `mj_instantiateLimit()` in `engine_core_constraint.c` — tendon limit
  activation uses `dist < margin` check.

**Spec B — QCQP Cone Projection** (DT-19):
The friction cone enforcement algorithm used during PGS/CG/Newton solver
iterations. When a contact has condim > 1, the solver must jointly project the
normal force and friction forces onto the friction cone surface.

**Existing infrastructure** (substantial — this is a refinement spec, not
greenfield):
- `project_elliptic_cone()` in `noslip.rs:15-57` — projects forces onto the
  elliptic friction cone by scaling friction components when `||λ_i/μ_i|| > λ_n`.
  Used by PGS (pgs.rs:148) for group projection of elliptic contacts.
- `noslip_qcqp2()` in `noslip.rs:76-166` — 2-DOF QCQP solver for condim=3
  constrained friction in the noslip post-processor.
- `noslip_qcqp3()` in `noslip.rs:172-286` — 3-DOF QCQP solver for condim=4.
- PGS handles pyramidal contacts with per-row `.max(0.0)` clamping (pgs.rs:185-192).

**What Spec B must verify/implement** against MuJoCo's `mj_projectConstraint()`:
- Verify `project_elliptic_cone()` matches MuJoCo's projection exactly (the
  current implementation may differ in edge cases: zero normal force, all-
  friction scenarios, QCQP vs simple scaling).
- Verify the noslip QCQP solvers match MuJoCo's noslip post-processor.
- Verify integration across all three solvers (PGS, CG, Newton).
- Handle condim=6 (full elliptic with rolling friction): verify the existing
  projection handles 5 friction components correctly.
- Condim dispatch: frictionless (condim=1) → unilateral clamp; pyramidal
  (condim=3) → per-row box constraints; elliptic (condim=4/6) → group
  QCQP projection.

MuJoCo reference: `mj_projectConstraint()` in `engine_solver.c` — called
per-contact during each solver iteration. The projection depends on condim
and uses different formulations for pyramidal vs elliptic cones.

**DT-25 Verification Pass** (1 session, NOT a full spec):
The deformable-rigid friction infrastructure already exists: condim support
in collision, Jacobian construction in `jacobian.rs`, unified projection in
assembly. This session:
1. Audits the existing deformable contact + friction code path end-to-end
2. Writes targeted tests for deformable-rigid friction with condim=3
3. Verifies R-scaling with asymmetric Jacobians (zero angular on flex side)
4. Documents any real gaps (condim=4/6 for deformable, Jacobian issues) as
   deferred items — does NOT implement fixes

---

## Dependency Graph

```
Session 1 (Umbrella — rescoped)
    |
    +-- Session 2 (T1: DT-28, DT-39)                <- independent
    |
    +-- Sessions 3-7 (Spec A: DT-23 + DT-33)        <- independent
    |
    +-- Sessions 8-12 (Spec B: DT-19 QCQP)          <- independent
    |
    +-- Session 13 (DT-25 verification)              <- independent
```

**Flat dependency graph.** No cross-spec dependencies. All deliverables are
independent of each other — ordering is for clarity, not necessity.

### Why no cross-spec dependencies?

| Spec pair | Interaction analysis |
|-----------|---------------------|
| Spec A (solver params) vs Spec B (cone projection) | **No conflict.** Spec A modifies constraint parameter wiring (`solref`/`solimp` fields, `tendon_margin`). Spec B modifies force projection in solver iterations. Different stages of the constraint pipeline: Spec A is at row assembly time, Spec B is at solver iteration time. |
| Spec A vs T1 (DT-28, DT-39) | **No conflict.** Spec A touches DOF friction solver params and tendon margin. T1 touches tendon DOF mapping (DT-28) and body-weight approximation (DT-39). Different model fields, different code paths. |
| Spec A vs DT-25 verification | **Minimal overlap.** Spec A modifies tendon limit activation (DT-33). DT-25 audits deformable contact friction. The only shared file is `assembly.rs`, but they modify different sections (tendon limits vs deformable contacts). |
| Spec B vs T1 (DT-39 diagApprox) | **Complementary, not dependent.** DT-39 adds `body_invweight0`/`dof_invweight0` and `mj_diagApprox()`. Spec B's cone projection reads the regularized diagonal `R` but does not depend on how it was computed (exact vs approximate). |
| Spec B vs DT-25 verification | **No conflict.** Spec B implements cone projection for rigid contacts. DT-25 verifies deformable friction which uses the same projection function but with different Jacobian structure. DT-25 is read-only audit + tests. |
| T1 vs DT-25 verification | **No conflict.** DT-28 validates tendon DOF mapping. DT-39 adds weight fields. Neither touches deformable contact code. |

---

## File Ownership Matrix

Files touched by 2+ deliverables, with ownership sequence and handoff state.
Single-owner files are not listed.

### `sim/L0/core/src/constraint/assembly.rs` (38.5 KB)

| Order | Deliverable | Section | Change | State after |
|-------|-------------|---------|--------|-------------|
| any | Spec A (DT-33) | tendon limits (~lines 141-154, 525-580) | Replace 4 hardcoded `< 0.0` with `< tendon_margin[t]`; pass margin to `finalize_row!` | Tendon limits margin-aware |
| any | Spec A (DT-23) | DOF friction (~lines 377-385) | **Verify** existing `dof_solref`/`dof_solimp` wiring; no code change expected | Already wired (verification only) |
| any | Spec B (DT-19) | -- | No direct change to assembly.rs; cone projection is in solver/*.rs | Unchanged |
| any | DT-25 | deformable contacts | **Read-only audit** + new tests; no code change expected | Unchanged |

**Conflict risk: Low.** Spec A modifies tendon limit sections. DT-23 is
verification-only on the friction loss section. Spec B and DT-25 do not modify
this file.

### `sim/L0/core/src/types/model.rs`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| any | Spec A (DT-33) | Add `tendon_margin: Vec<f64>` | 1 new tendon field |
| any | T1 (DT-39) | Add `body_invweight0: Vec<[f64; 2]>`, `dof_invweight0: Vec<f64>`, `tendon_invweight0: Vec<f64>` | 3 new weight fields |

**Conflict risk: None.** All additions are new fields — no modification of
existing fields.

### `sim/L0/core/src/types/model_init.rs`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| any | Spec A (DT-33) | Init `tendon_margin` | 1 new default |
| any | T1 (DT-39) | Init `body_invweight0`, `dof_invweight0`, `tendon_invweight0` | 3 new defaults |

**Conflict risk: None.** Append-only pattern.

### `sim/L0/core/src/constraint/solver/pgs.rs` (488 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| any | Spec B (DT-19) | Add/modify cone projection logic in PGS iteration | Cone-aware PGS |
| any | DT-25 | **Read-only audit** of projection for deformable rows | Unchanged |

**Conflict risk: Low.** Spec B modifies projection logic. DT-25 only reads.

### `sim/L0/core/src/constraint/solver/noslip.rs` (748 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| any | Spec B (DT-19) | Verify/update cone projection in noslip post-processor | Cone-aware noslip |

**Conflict risk: None.** Single-owner in Phase 8.

### `sim/L0/mjcf/src/types.rs`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| any | Spec A (DT-23) | **Already exists**: `solreffriction`/`solimpfriction` on `MjcfJoint` (line 614-616) and `MjcfJointDefaults`. Verify only. | No change expected |
| any | Spec A (DT-33) | **Already exists**: `margin: Option<f64>` on `MjcfTendon` (line 2808) and `MjcfTendonDefaults` (line 805). Verify only. | No change expected |

**Conflict risk: None.** Both items are verification of existing parsing.

### `sim/L0/mjcf/src/builder/` (joint and tendon building)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| any | Spec A (DT-23) | **Already exists**: `builder/joint.rs:160-163` populates `dof_solref`/`dof_solimp` from `solreffriction`. Verify multi-DOF fan-out. | Verification (may need fix for ball/free) |
| any | Spec A (DT-33) | Wire `tendon_margin` through builder to model field (parsing exists, model field + builder wiring does not) | New field populated |

**Conflict risk: None.** Different builder files (joint vs tendon).

---

## API Contracts

No cross-spec API dependencies exist in Phase 8. Each deliverable produces
independent additions or modifications:

- **Spec A** modifies constraint parameter wiring and tendon limit activation —
  no other deliverable reads `tendon_margin` or depends on `dof_solref` wiring.
- **Spec B** adds/modifies cone projection in solver iteration — no other
  deliverable calls `mj_projectConstraint()`.
- **T1 (DT-28)** validates tendon DOF mapping — no other deliverable modifies
  tendon computation.
- **T1 (DT-39)** adds weight fields and `mj_diagApprox()` — Spec B's cone
  projection reads the regularized diagonal `R` but is agnostic to how `R`
  was computed. The exact-diagonal path (currently used) continues to work;
  `diagApprox` is an alternative path selectable at build time.
- **DT-25** is read-only audit + new tests — no code modification expected.

This is structurally similar to Phase 7's flat dependency graph. The key
difference is that Phase 8 tasks converge on the same subsystem
(constraint pipeline) rather than being spread across unrelated subsystems.
The flat graph holds because each task touches a different *stage* of the
pipeline: parameter wiring (Spec A) → diagonal approximation (T1-b) →
row assembly (unchanged) → solver projection (Spec B) → verification (DT-25).

---

## Shared Convention Registry

Conventions decided once here. Sub-specs reference this section instead of
inventing their own.

### 1. New model array naming

All new model arrays follow the existing `{element}_{name}` pattern:

| Field | Type | Default | Deliverable |
|-------|------|---------|-------------|
| `tendon_margin` | `Vec<f64>` | `0.0` | Spec A (DT-33) |
| `body_invweight0` | `Vec<[f64; 2]>` | `[0.0, 0.0]` | T1 (DT-39) |
| `dof_invweight0` | `Vec<f64>` | `0.0` | T1 (DT-39) |
| `tendon_invweight0` | `Vec<f64>` | `0.0` | T1 (DT-39) |

**Existing fields used (not created) by Phase 8:**

| Field | Type | Location | Used by |
|-------|------|----------|---------|
| `dof_solref` | `Vec<[f64; 2]>` | model.rs:226 | Spec A (DT-23, verify) |
| `dof_solimp` | `Vec<[f64; 5]>` | model.rs:229 | Spec A (DT-23, verify) |
| `tendon_solref_fri` | `Vec<[f64; 2]>` | model.rs:688 | Already wired |
| `tendon_solimp_fri` | `Vec<[f64; 5]>` | model.rs:690 | Already wired |
| `tendon_solref_lim` | `Vec<[f64; 2]>` | model.rs:680 | Already wired |
| `tendon_solimp_lim` | `Vec<[f64; 5]>` | model.rs:683 | Already wired |
| `jnt_margin` | `Vec<f64>` | model.rs:208 | Pattern for DT-33 |

### 2. Solver parameter naming convention

MuJoCo uses a consistent naming pattern for solver parameters on MJCF
elements. CortenForge must match exactly:

| MJCF attribute | Model field | Indexed by | Constraint type |
|----------------|-------------|-----------|-----------------|
| `solreffriction` | `dof_solref` | DOF | Friction loss DOF |
| `solimpfriction` | `dof_solimp` | DOF | Friction loss DOF |
| `solreflimit` | `jnt_solref` | Joint | Joint limits |
| `solimplimit` | `jnt_solimp` | Joint | Joint limits |
| `margin` (on `<joint>`) | `jnt_margin` | Joint | Joint limit activation |
| `margin` (on `<tendon>`) | `tendon_margin` | Tendon | Tendon limit activation |

**Critical:** `solreffriction` on `<joint>` maps to per-DOF fields (not
per-joint). A ball joint's `solreffriction` is copied to all 3 of its DOFs.

### 3. Constraint type ordering

The unified constraint assembly emits rows in this order (matching MuJoCo):

1. Equality constraints (connect, weld, joint, distance, tendon) + FlexEdge → `ne` rows
2. DOF friction loss + tendon friction loss → `nf` rows
3. Joint limits + tendon limits → `nl` rows
4. Contacts → `nc` rows

This ordering is stable across Phase 8. Spec A modifies parameter wiring for
rows in groups 2 and 3. Spec B modifies projection for rows in group 4 (and
potentially group 2 for friction). Neither changes the ordering.

### 4. Default values for solver parameters

Match MuJoCo's compile-time defaults exactly:

| Parameter | Default value | Source |
|-----------|--------------|--------|
| `solref` (all types) | `[0.02, 1.0]` | `mj_defaultSolRefImp()` in `engine_setconst.c` |
| `solimp` (all types) | `[0.9, 0.95, 0.001, 0.5, 2.0]` | `mj_defaultSolRefImp()` in `engine_setconst.c` |
| `margin` (joint/tendon) | `0.0` | `mjmodel.h` field default |

### 5. Weight field computation convention

`body_invweight0` and `dof_invweight0` are computed at **build time** from the
mass matrix at `qpos0`. They are static model properties, not runtime state.

- `body_invweight0[b][0]` = 1 / (total mass of subtree rooted at body `b`)
- `body_invweight0[b][1]` = 1 / (trace of subtree rotational inertia at body `b`)
- `dof_invweight0[dof]` = derived from `body_invweight0` using joint type

MuJoCo reference: `setInertia()` inside `mj_setConst()` in `engine_setconst.c`.

---

## Cross-Spec Blast Radius

### Behavioral interactions between specs

| Interaction | Analysis |
|------------|----------|
| Spec A `tendon_margin` + Spec B cone projection | **No conflict.** `tendon_margin` affects constraint *activation* (whether a row is emitted). Cone projection affects constraint *force* (how the solver projects forces during iteration). Different pipeline stages. |
| Spec A `dof_solref` verification + Spec B solver iteration | **No conflict.** Spec A verifies that the correct `solref` values reach assembly. Spec B uses the regularized diagonal `R` derived from `solref`/`solimp` but does not modify how `R` is computed. |
| T1 `diagApprox` + Spec B cone projection | **Complementary.** `diagApprox` provides an alternative way to compute the diagonal of `A = J·M⁻¹·J^T`. Cone projection reads `R` which depends on `diagApprox`. If `diagApprox` changes the diagonal values, cone projection results change numerically but the algorithm is identical. **Resolution:** T1 adds `diagApprox` as an option alongside the existing exact diagonal. Spec B's cone projection works with either. |
| T1 `diagApprox` + Spec A `tendon_margin` | **No conflict.** `diagApprox` adds `tendon_invweight0` to Model. Spec A adds `tendon_margin` to Model. Different fields, different consumers. |
| DT-25 verification + Spec B cone projection | **Complementary.** DT-25 verifies deformable friction uses cone projection correctly. Spec B implements/improves that projection. If Spec B lands first, DT-25 verifies the new projection. If DT-25 runs first, it verifies the current projection and documents gaps that Spec B may address. **Resolution:** Order doesn't matter — DT-25 documents the state of the projection at verification time. |
| DT-28 tendon validation + Spec A `tendon_margin` | **No conflict.** DT-28 validates tendon DOF mapping for ball/free joints. Spec A adds tendon margin for limit activation. Different code paths (tendon length computation vs constraint activation). |

### Existing test impact (cross-spec)

| Test area | Touched by | Conflict risk |
|-----------|-----------|--------------|
| Constraint assembly tests (assembly.rs) | Spec A (DT-33: new margin tests) | **Low.** Adds new tests, doesn't modify existing. |
| PGS solver tests | Spec B (cone projection) | **Low.** May modify existing projection tests. |
| Newton solver tests | Spec B (verify cone projection) | **Low.** Verification only. |
| Tendon tests (sim-tendon) | T1 (DT-28: ball/free validation) | **Low.** Adds new tests. |
| Deformable tests | DT-25 (new friction tests) | **None.** Adds tests only. |
| Phase 7 regression suite (1,900+ tests) | None expected | **None.** Phase 8 does not modify parsing or defaults code paths from Phase 7. |

### Test count changes

| Deliverable | Estimated new tests | Net change |
|-------------|-------------------|------------|
| T1-a (DT-28: ball/free tendons) | 3–5 (ball + free joint validation, DOF mapping) | +3–5 |
| T1-b (DT-39: diagApprox) | 4–6 (body_invweight0 computation, dof_invweight0, diagApprox vs exact) | +4–6 |
| Spec A (DT-23 + DT-33) | 6–10 (solreffriction defaults cascade, non-default friction params, tendon margin activation at 2 limit types) | +6–10 |
| Spec B (DT-19: QCQP) | 6–10 (cone projection for condim=1/3/4/6, edge cases: zero normal, all-friction) | +6–10 |
| DT-25 verification | 2–4 (deformable-rigid friction condim=3, R-scaling audit) | +2–4 |
| **Total** | **21–35** | **+21–35** |

---

## Phase-Level Acceptance Criteria

These are the aggregate gates that determine "Phase 8 complete." Individual
sub-specs have their own ACs for technical correctness. **The overarching
criterion: CortenForge's constraint formulation and solver behavior is
numerically identical to MuJoCo's for every feature implemented in Phase 8.**

### PH8-AC1: All 6 active tasks ship-complete
Every task in the assignment table (DT-19, DT-23, DT-25, DT-28, DT-33, DT-39)
has landed and its sub-spec ACs (or T1/verification criteria) are met. Every
sub-spec AC that asserts a numerical value has that value verified against
MuJoCo's actual output.

### PH8-AC2: No regression in existing test suite
All 1,900+ domain tests from the post-Phase 7 baseline pass. Zero test
failures attributable to Phase 8 changes.

### PH8-AC3: Quality gate passes
`cargo xtask check` passes (formatting, clippy, all tests).

### PH8-AC4: Aggregate test growth
Domain test count increases by at least 21 (lower bound of per-deliverable
estimates). At least one MuJoCo conformance test (expected value from running
MuJoCo) per sub-spec.

### PH8-AC5: Constraint conformance coverage
After Phase 8, the following MuJoCo constraint/solver features are covered:

| Feature | Pre-Phase 8 | Post-Phase 8 |
|---------|------------|-------------|
| Per-DOF friction loss solver params | Full pipeline exists (parsing, defaults, builder, assembly); multi-DOF fan-out + tests unverified | Fully verified: multi-DOF fan-out correct, non-default tests pass |
| Tendon `margin` | Parsed from MJCF + defaults cascade, but no model field; hardcoded `< 0.0` at 4 assembly sites | `tendon_margin` model field wired; `< tendon_margin[t]` at all 4 sites + margin in `finalize_row!` |
| Ball/Free joints in fixed tendons | Unvalidated DOF index mapping | Validated + tested for ball (nv=3) and free (nv=6) |
| Body-weight diagonal approximation | Exact diagonal only (O(nv) per row) | `diagApprox` via `body_invweight0`/`dof_invweight0` (O(1) per row) |
| QCQP cone projection | Elliptic cone projection exists (`project_elliptic_cone`, `noslip_qcqp2/3`); unverified against MuJoCo | Verified conformance: projection matches `mj_projectConstraint()` for all condim values |
| Deformable-rigid friction | Infrastructure exists, unverified | Verified for condim=3; gaps documented as deferred |

---

## Out of Scope

Explicitly excluded from Phase 8. Each exclusion states its conformance impact.

- **DT-18** (Zero-friction condim downgrade optimization) — Performance
  optimization, not conformance. *Conformance impact: none — produces
  identical results, just faster.*

- **DT-20** (Unify `J^T * lambda` vs chain-walk force application) —
  Architectural cleanup, not conformance. *Conformance impact: none — both
  paths produce identical forces.*

- **DT-22** (`efc_impP` impedance derivative field) — API introspection field,
  not used by solvers. *Conformance impact: minimal — external API surface
  only, no physics effect.*

- **DT-24** (Incremental collision detection on tree wake) — Performance
  optimization. *Conformance impact: none.*

- **DT-26/DT-27** (XPBD improvements) — Post-v1.0 extensions.
  *Conformance impact: none — XPBD is not part of MuJoCo's core solver.*

- **DT-30** (Compound pulley physics) — Post-v1.0 extension.
  *Conformance impact: none — capstan friction is not in MuJoCo core.*

- **DT-31** (`WrapType::Joint` in spatial tendons) — Low priority MuJoCo
  compat. *Conformance impact: minor — affects models using `mjWRAP_JOINT`,
  which is uncommon.*

- **DT-101** (`mj_contactPassive()` viscous damping) — Post-v1.0.
  *Conformance impact: affects models with contact viscosity, which is
  uncommon in standard models.*

- **Condim=4/6 for deformable contacts** — If DT-25 verification discovers
  gaps in deformable elliptic friction, these are documented as deferred items,
  not fixed in Phase 8. *Conformance impact: affects deformable models with
  torsional/rolling friction, which are rare.*

- **`solreffriction` on contacts** — Per-direction solver params on contact
  friction rows (`solreffriction` attribute on `<geom>` or `<pair>`) are a
  separate scope from per-DOF friction loss params. The contact-level
  `solreffriction` affects elliptic friction row parameters and is tracked
  separately. *Conformance impact: affects models with per-contact
  friction solver tuning.*
