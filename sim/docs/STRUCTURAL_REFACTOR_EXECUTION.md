# Structural Refactor — Execution Strategy

> **Purpose**: Risk-mitigated execution discipline for the structural refactor.
> Complements [STRUCTURAL_REFACTOR.md](./STRUCTURAL_REFACTOR.md) (the plan) and
> [STRUCTURAL_REFACTOR_RUBRIC.md](./STRUCTURAL_REFACTOR_RUBRIC.md) (the grading
> criteria). This document answers: *how do we actually do this safely?*

---

## Core Principle

**Never have more than one function in flight.** Move it, compile, move the
next. Tedious but zero-risk. The spec is detailed enough that this is mostly
mechanical — the hard thinking is already done.

---

## 1. Session Granularity

The biggest real risk is **context window exhaustion mid-extraction**. A
half-moved function leaves the monolith in an uncompilable state.

**Rule**: Each session targets **one sub-module file** (e.g.,
`constraint/equality.rs`, not all of Phase 6). The session ends with a commit.
If context runs low, stop at the last clean commit — never mid-move.

**Small-file grouping**: Trivially small modules (under ~100 lines) that share
a parent directory may be grouped into one session — e.g., `tendon/mod.rs`
(~91 lines) + `tendon/fixed.rs` (~63 lines), or `dynamics/mod.rs` (re-exports)
+ `dynamics/spatial.rs`. The grouping is noted in the progress table.

**Implication for Phase 6**: The constraint extraction (12 files, ~5,612 lines)
spans multiple sessions. That's 12 commits — each independently revertible.

---

## 2. Phase Ordering

The [Phase Dependencies DAG](./STRUCTURAL_REFACTOR.md#phase-dependencies) allows
phases 3–7 in parallel, but **sequential execution is safer**:

- Each phase shrinks the monolith, making the next extraction easier to reason about
- Fewer simultaneous moving parts to track
- If something goes wrong, the blast radius is one phase

**Recommended order**:

```
0 → 1 → 2 → 5 → 4 → 3 → 7 → 6 → 8a → 8b → 8c → 10 → 12
```

**Rationale for the 3–7 ordering**:

| Order | Phase | Lines | Why this position |
|-------|-------|------:|-------------------|
| 1st | 5 (tendon) | ~1,150 | Small, self-contained. Good warmup after the foundational phases. |
| 2nd | 4 (sensor + energy) | ~1,200 | Also self-contained, moderate size. |
| 3rd | 3 (collision) | ~2,700 | Larger but no solver complexity. |
| 4th | 7 (dynamics) | ~930 | Small. Fills out the `dynamics/` tree created in Phase 2. |
| 5th | 6 (constraint) | ~5,612 | Largest, most sub-modules (12), most internal dependencies. By this point we've completed 5 extraction phases and have the rhythm. |

**Phase 10** (sim-mjcf) is independent and runs after sim-core phases complete.
This avoids mental context-switching between two codebases.

---

## 3. The Move Protocol

### Per-function (repeat for each function being moved):

```
1. Create target file (if new) with //! doc + use imports
2. Cut function from monolith → paste into target file
   - Preserve #[inline], #[inline(always)], #[inline(never)] attributes
3. Add pub(crate) re-import in monolith top
   (e.g., pub(crate) use crate::types::*)
4. Update lib.rs to re-export moved pub symbols from the new module path
   (e.g., pub use types::Model;). Keep existing monolith wildcard re-export
   (pub use mujoco_pipeline::*;) until Phase 12.
5. cargo check -p sim-core
6. If error → fix imports, repeat step 5
7. Move to next function in same sub-module
```

### Per-sub-module (after all functions for a sub-module are moved):

```
 8. Run the lazy import check:
    - Comment out monolith re-imports
    - cargo check -p sim-core
    - Fix any errors in non-monolith files (point to real module paths)
    - Uncomment re-imports
 9. Update all use imports in existing files that reference moved symbols
    (derivatives.rs, batch.rs, lib.rs — see spec step 1)
10. Update code comments that reference mujoco_pipeline.rs or
    model_builder.rs by filename to reference the new module
11. Update doc references in sim/docs/ for functions moved in this sub-module
12. Run S6 stale-reference grep for monolith filenames
13. cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics
14. cargo clippy -p sim-core -- -D warnings
15. Verify pass count matches baseline
    (see spec Phase 0 table — currently 1,526 passed, 0 failed, 15 ignored)
16. Commit (with approval)
```

### Per-phase (after all sub-modules for a phase are committed):

```
17. Grade all modules touched in this phase against rubric S1–S8
18. Verify every new file has a //! module doc comment
```

**Monolith re-imports** (`pub(crate) use crate::types::*;` etc.) accumulate
throughout phases 1–8c. They are removed in Phase 12 when the monolith shim is
deleted. Do not remove them during intermediate phases.

---

## 4. Pre-Flight for Each Phase

Before starting any phase, read:

1. The phase's checklist in [STRUCTURAL_REFACTOR.md](./STRUCTURAL_REFACTOR.md)
2. The target module sizes + line ranges from the Doc Reference Mapping Table
3. The ["every phase includes" steps](./STRUCTURAL_REFACTOR.md#every-phase-includes-non-negotiable)
   (lines 989–1022)
4. Any MARGIN WARNINGs for modules in this phase (flagged in the progress table
   with `(MARGIN)`)

This avoids guessing about what goes where.

---

## 5. Phase 0 — First Session

Before any code moves:

- [ ] Verify the test baseline still matches the spec's Phase 0 table
      (STRUCTURAL_REFACTOR.md lines 1034–1045)
- [ ] Audit the test helper categorization (shared vs. local) in the monolith's
      `#[cfg(test)]` section (L21476–L26722)
- [ ] Create branch: `refactor/structural-decompose`
- [ ] Commit Phase 0 as the starting point

---

## 6. Phase-Specific Hazards

### Phase 1: MARGIN WARNINGs

- `types/model.rs` (~780 lines): 20 lines from limit. Fallback: move
  `is_ancestor()` (~14 lines) to `types/model_init.rs`.
- `types/model_init.rs` (~774 lines): raw source ~930 before whitespace
  removal. Fallback: move `compute_body_lengths` + `compute_dof_lengths`
  (~64 lines) to a separate file.
- `types/data.rs` (~710 lines): raw span ~805. Fallback: move
  `reset_to_keyframe` (~40 lines) to `types/data_keyframe.rs`.

### Phase 6: MARGIN WARNINGs

- `constraint/assembly.rs` (~750 lines): With `use` imports + `//!` doc may
  approach 800. Fallback: move `populate_efc_island` (62 lines) to
  `constraint/mod.rs`.
- `constraint/solver/hessian.rs` (~685 lines): With overhead may approach 800.
  Fallback: split `SparseHessian` into `constraint/solver/sparse_hessian.rs`.

### Phase 8a: Line range hazard

**`forward/passive.rs`**: The tendon implicit K/D helpers at L12690–L12816 sit
**between** the two passive force ranges (L12108–L12690 and L12818–L12899) but
belong to `integrate/implicit.rs` (Phase 8b), **NOT** to `forward/passive.rs`.
Only take L12108–L12690 and L12818–L12899 for passive.

---

## 7. Phase 12 — Doc Sweep

The ~694 doc reference updates are mechanical find-and-replace using the
[Doc Reference Mapping Table](./STRUCTURAL_REFACTOR.md#doc-reference-mapping-table).

**Strategy**: Batch by target module to minimize errors. For each target module
(e.g., `constraint/assembly.rs`), find all `future_work_*.md` lines citing
source ranges that map to that module and update them in one pass. Diff-review
the changes before committing.

---

## 8. Abort Criteria

Stop and reassess if any of these occur:

- Test count changes (any test appearing or disappearing)
- A module exceeds 800 production lines after extraction (trigger the documented
  fallback in the spec)
- A circular import that wasn't anticipated in the audit findings
- Any physics output change (same inputs must produce bitwise-identical outputs)

If an abort criterion is triggered, see the
[Rollback Procedure](./STRUCTURAL_REFACTOR.md#rollback-procedure) in the spec
for revert protocol.

---

## 9. Progress Tracking

After each commit, update this table. Sub-module order within each phase
respects internal call dependencies (matches the spec's extraction orders).

| Phase | Sub-module | Notes | Status | Commit | Session |
|-------|-----------|-------|--------|--------|---------|
| 0 | Preparation | | | | |
| 1 | types/enums.rs | Extract first — other types/ modules depend on enums | | | |
| 1 | types/model.rs | **(MARGIN)** ~780 lines | | | |
| 1 | types/model_init.rs | **(MARGIN)** ~774 lines | | | |
| 1 | types/model_factories.rs | `#[cfg(test)]`-gated | | | |
| 1 | types/data.rs | **(MARGIN)** ~710 lines (raw ~805) | | | |
| 1 | types/contact_types.rs | | | | |
| 1 | types/keyframe.rs | | | | |
| 1 | lib.rs re-exports | Route pub API through types/ | | | |
| 2 | linalg.rs | | | | |
| 2 | dynamics/mod.rs + spatial.rs | Grouped — mod.rs is pure re-exports | | | |
| 2 | joint_visitor.rs | | | | |
| 5 | tendon/mod.rs + fixed.rs | Grouped — mod.rs ~91 lines, fixed.rs ~63 lines | | | |
| 5 | tendon/spatial.rs | | | | |
| 5 | tendon/wrap_math.rs | | | | |
| 4 | sensor/mod.rs + position.rs | Grouped — mod.rs ~17 lines | | | |
| 4 | sensor/velocity.rs | | | | |
| 4 | sensor/acceleration.rs | | | | |
| 4 | sensor/postprocess.rs | | | | |
| 4 | sensor/derived.rs | | | | |
| 4 | energy.rs | | | | |
| 3 | collision/mod.rs | | | | |
| 3 | collision/narrow.rs | | | | |
| 3 | collision/pair_convex.rs | | | | |
| 3 | collision/pair_cylinder.rs | | | | |
| 3 | collision/plane.rs | | | | |
| 3 | collision/mesh_collide.rs | | | | |
| 3 | collision/hfield.rs | | | | |
| 3 | collision/sdf_collide.rs | | | | |
| 3 | collision/flex_collide.rs | | | | |
| 7 | dynamics/crba.rs | | | | |
| 7 | dynamics/rne.rs | | | | |
| 7 | dynamics/factor.rs | | | | |
| 7 | dynamics/flex.rs | | | | |
| 6 | constraint/mod.rs | | | | |
| 6 | constraint/impedance.rs | | | | |
| 6 | constraint/jacobian.rs | | | | |
| 6 | constraint/equality.rs | | | | |
| 6 | constraint/assembly.rs | **(MARGIN)** ~750 lines | | | |
| 6 | constraint/solver/mod.rs | | | | |
| 6 | constraint/solver/primal.rs | | | | |
| 6 | constraint/solver/pgs.rs | | | | |
| 6 | constraint/solver/cg.rs | | | | |
| 6 | constraint/solver/hessian.rs | **(MARGIN)** ~685 lines | | | |
| 6 | constraint/solver/newton.rs | | | | |
| 6 | constraint/solver/noslip.rs | | | | |
| 8a | forward/mod.rs | | | | |
| 8a | forward/position.rs | | | | |
| 8a | forward/velocity.rs | | | | |
| 8a | forward/passive.rs | **HAZARD**: see Phase 8a notes above | | | |
| 8a | forward/actuation.rs | | | | |
| 8a | forward/muscle.rs | | | | |
| 8a | forward/acceleration.rs | | | | |
| 8a | forward/check.rs | | | | |
| 8a | jacobian.rs | Top-level, not under forward/ | | | |
| 8b | integrate/mod.rs | | | | |
| 8b | integrate/euler.rs | | | | |
| 8b | integrate/implicit.rs | Receives L12690–L12816 from Phase 8a hazard | | | |
| 8b | integrate/rk4.rs | | | | |
| 8c | island/mod.rs | | | | |
| 8c | island/sleep.rs | | | | |
| 10 | builder/mod.rs | | | | |
| 10 | builder/init.rs | | | | |
| 10 | builder/orientation.rs | | | | |
| 10 | builder/asset.rs | | | | |
| 10 | builder/fluid.rs | | | | |
| 10 | builder/compiler.rs | | | | |
| 10 | builder/frame.rs | | | | |
| 10 | builder/mesh.rs | | | | |
| 10 | builder/geom.rs | | | | |
| 10 | builder/joint.rs | | | | |
| 10 | builder/body.rs | | | | |
| 10 | builder/mass.rs | | | | |
| 10 | builder/actuator.rs | | | | |
| 10 | builder/sensor.rs | | | | |
| 10 | builder/tendon.rs | | | | |
| 10 | builder/contact.rs | | | | |
| 10 | builder/equality.rs | | | | |
| 10 | builder/flex.rs | | | | |
| 10 | builder/build.rs | Must be last — references all process_* outputs | | | |
| 10 | builder/ inline tests | ~4,152 lines across 151 fn definitions | | | |
| 12 | Monolith deletion + shim removal | | | | |
| 12 | Stale reference sweep (grep) | | | | |
| 12 | future_work_*.md doc updates (~694) | | | | |
| 12 | ARCHITECTURE.md rewrite | | | | |
| 12 | Other doc + test comment updates | GAP_ANALYSIS, CONFORMANCE, REFERENCE, test files, gjk_epa.rs | | | |
| 12 | Final workspace verification + grading | All 15 final-state checks from rubric | | | |
