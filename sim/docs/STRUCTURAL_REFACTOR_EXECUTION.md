# Structural Refactor — Execution Strategy

> **Purpose**: Risk-mitigated execution discipline for the structural refactor.
> Complements [STRUCTURAL_REFACTOR.md](./STRUCTURAL_REFACTOR.md) (the plan) and
> [STRUCTURAL_REFACTOR_RUBRIC.md](./STRUCTURAL_REFACTOR_RUBRIC.md) (the grading
> criteria). This document answers: *how do we actually do this safely?*

---

## Core Principle

**Never have more than one item in flight.** Move it, compile, move the next.
Tedious but zero-risk. The spec is detailed enough that this is mostly
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

### Per-item (repeat for each function, struct, enum, const, or impl block):

```
0. Declare module (once per new directory):
   - Add `mod <dir_name>;` to lib.rs — or to the parent mod.rs for
     nested directories (e.g., `mod solver;` in `constraint/mod.rs`)
   - Create <dir_name>/mod.rs with //! doc and
     `pub(crate) mod <submodule>;` declarations for planned sub-modules
   - Add `pub use <submodule>::*;` in mod.rs for each sub-module
     (so that `crate::types::Model` resolves — keeps re-export chain
     to 2 hops per rubric anti-pattern #2)
   - cargo check to verify the empty module structure compiles
   Note: When mod.rs is itself an extraction target (e.g., `builder/mod.rs`
   holds the struct definition + orchestration functions), create it with the
   //! doc and sub-module declarations, then immediately proceed to step 2
   to populate it — there is no separate "target file" to create in step 1.
1. Create target file (if new) with //! doc
2. Cut item from monolith → paste into target file
   - Copy only the `use` imports that the item needs from the monolith
     preamble. cargo check will flag any you miss.
   - Preserve ALL attributes exactly as they appear: #[derive(...)],
     #[repr(...)], #[non_exhaustive], #[must_use], #[allow(...)],
     #[inline]/[inline(always)]/[inline(never)], and /// doc comments.
   - Do NOT add new attributes — only preserve existing ones.
   - Do NOT copy the monolith's file-level #![allow(...)] block into
     new files. Rely on cargo clippy (step 14) to surface issues.
   - Set visibility: pub if the symbol is in the crate's lib.rs
     re-export list (for sim-core: `pub use mujoco_pipeline::{...}`
     at lib.rs lines 111–173; for sim-mjcf: `pub use model_builder::{...}`
     at lib.rs line 197), pub(crate) otherwise.
     Private items only called within the same target file stay private.
   - If the moved item calls functions still in the monolith, add a
     temporary `use crate::mujoco_pipeline::{needed_fn};` in the target
     file. This will be updated when the callee is extracted later.
3. Add named pub(crate) re-import in monolith top
   (e.g., pub(crate) use crate::types::Model;)
   Use named imports during the per-item loop — NOT wildcards.
   A wildcard (use crate::types::*) would shadow items still in the
   monolith if the target module re-exports them. After all items for
   a sub-module are moved, you may collapse to a glob.
4. Update lib.rs re-exports (only if the moved symbol appears in the
   `pub use mujoco_pipeline::{...}` list in lib.rs):
   - Remove it from the monolith re-export list
   - Add a new re-export from the real module path
     (e.g., pub use types::Model;)
   - Rust does not allow the same name in two `pub use` re-exports
   - Keep the remaining monolith re-export list until Phase 12
   - When a single `pub use <monolith>::{A, B, C}` block has some
     symbols moving and others staying, split it: remove the moving
     symbols from the monolith block and add separate `pub use`
     lines from the new module path. The remaining monolith block
     keeps only the not-yet-moved symbols.

   Skip this step for:
   - Private or pub(crate) items (not in the re-export list)
   - `impl` methods (accessible through the type's own re-export)
   - Trait impls (automatically available when type + trait are in scope)
5. cargo check -p <crate>  (sim-core for phases 1–8c, sim-mjcf for phase 10)
6. If error → fix imports, repeat step 5
7. Move to next item in same sub-module
```

**Move order within a sub-module**: When two items moving to the same target
file have a call dependency, move the **callee first** so the caller's import
points to the final location immediately. If there is a circular call (both
call each other), move both in the same step and fix imports together before
running `cargo check`.

**Inherent `impl` blocks and trait impls**: When moving an enum, struct, or
const, treat its inherent `impl` block and all trait impls (`Display`, `Error`,
`From`, etc.) as part of the same item — move them together in a single step.

**Detached `impl` methods**: Methods on a type that implement a distinct
subsystem go in the module of their **primary computation**, not the module
that defines the type. For example, `Data::step()` goes in `forward/mod.rs`,
not `types/data.rs`; `ModelBuilder::new()` goes in `builder/init.rs`, not
`builder/mod.rs`. See the [`impl` Block Split
Strategy](./STRUCTURAL_REFACTOR_RUBRIC.md#impl-block-split-strategy) in the
rubric for the sim-core mapping; for sim-mjcf, the Phase 10 checklist in the
spec lists every split explicitly.

**Split-impl imports**: When an `impl TypeName` block lives in a child module
(e.g., `builder/init.rs`), that file needs `use super::TypeName;` to bring the
type into scope. This is standard Rust — `super::` refers to the parent module
(`builder/mod.rs` in this case). Add this import alongside the other `use`
lines copied from the monolith preamble.

**`#[cfg(test)]`-gated modules**: For test-only modules like
`types/model_factories.rs`, place `#[cfg(test)]` on the `mod` declaration in
the parent (e.g., `#[cfg(test)] mod model_factories;` in `types/mod.rs`), not
on the file contents. The file itself uses normal `pub(crate)` visibility.

### Per-sub-module (after all items for a sub-module are moved):

```
 8. Run the lazy import check:
    - Comment out monolith re-imports
    - cargo check -p <crate>
    - Expect errors inside the monolith itself — ignore those
    - Fix any errors in non-monolith files (point to real module paths)
    - Uncomment re-imports
 9. Update all use imports in existing files that reference moved symbols
    (phases 1–8c: derivatives.rs, batch.rs, lib.rs — the external
     consumers listed in the spec's "every phase includes" checklist;
     phase 10: sim-mjcf/src/lib.rs)
10. Update code comments that reference mujoco_pipeline.rs or
    model_builder.rs by filename to reference the new module
11. Update doc references in sim/docs/ for functions moved in this sub-module
12. Run S6 stale-reference grep for monolith filenames
13. cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics
    If tests fail: the most common causes are (a) a visibility change
    (pub → pub(crate)) that broke an external test, (b) a use import
    routing through the monolith shim instead of the real module, or
    (c) a test helper that wasn't moved with its code. Bisect by
    restoring items to the monolith one at a time until the failure is
    isolated. Do not commit a sub-module with failing tests.
14. cargo clippy -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics -- -D warnings
15. Verify pass count matches baseline
    (see spec Phase 0 table — currently 1,526 passed, 0 failed, 15 ignored)
16. Commit (with approval)
```

### Per-phase (after all sub-modules for a phase are committed):

```
17. Grade all modules touched in this phase against rubric S1–S8
18. Verify every new file has a //! module doc comment
```

**Monolith re-imports** (`pub(crate) use crate::types::Model;` etc.) accumulate
throughout phases 1–8c (and phase 10 for `model_builder.rs`). They are removed
in Phase 12 when the monolith shims are deleted. Do not remove them during
intermediate phases.

### Phase 10 Adaptations

The move protocol above is written from the sim-core perspective. For Phase 10
(sim-mjcf), substitute:

| Protocol reference | Phases 1–8c (sim-core) | Phase 10 (sim-mjcf) |
|--------------------|------------------------|---------------------|
| "the monolith" | `mujoco_pipeline.rs` | `model_builder.rs` |
| `cargo check -p` | `sim-core` | `sim-mjcf` |
| Re-import pattern (step 3) | `pub(crate) use crate::types::Model;` (named, per item) | `pub(crate) use crate::builder::orientation::resolve_orientation;` (named, per item) |
| Intra-module type access | `use crate::types::Model;` (types extracted first) | `use super::ModelBuilder;` in sub-modules (struct defined in `builder/mod.rs`) |
| `lib.rs` re-exports | `sim-core/src/lib.rs`: move symbols from `pub use mujoco_pipeline::{...}` to new module paths | `sim-mjcf/src/lib.rs`: add `mod builder;` alongside `mod model_builder;` (both coexist until Phase 12), update `pub use` |
| Test command (step 13) | `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics` | `cargo test -p sim-mjcf -p sim-conformance-tests -p sim-physics -p sim-urdf` |
| Clippy command (step 14) | `cargo clippy -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics -- -D warnings` | `cargo clippy -p sim-mjcf -p sim-conformance-tests -p sim-physics -p sim-urdf -- -D warnings` |
| Feature gate test (step 13a) | N/A | `cargo test -p sim-mjcf --features mjb` (once per session) |
| Pass count baseline (step 15) | 1,526 passed, 0 failed, 15 ignored (spec Phase 0 table) | Extend baseline to include `sim-urdf` — record its pass count during Phase 0 |
| Files to check for imports | `derivatives.rs`, `batch.rs`, `lib.rs` | `sim-mjcf/src/lib.rs` (only external consumer) |

---

## 4. Pre-Flight for Each Phase

Before starting any phase, read:

1. The phase's checklist in [STRUCTURAL_REFACTOR.md](./STRUCTURAL_REFACTOR.md)
2. The target module sizes + line ranges from the Doc Reference Mapping Table
3. The ["every phase includes" steps](./STRUCTURAL_REFACTOR.md#every-phase-includes-non-negotiable)
4. The [`impl` Block Split Strategy](./STRUCTURAL_REFACTOR_RUBRIC.md#impl-block-split-strategy)
   for any `impl` methods moving in this phase
5. Any MARGIN WARNINGs or HAZARDs for modules in this phase (flagged in Section 6
   and annotated in the progress table)

This avoids guessing about what goes where.

---

## 5. Phase 0 — First Session

Before any code moves:

- [ ] Verify the test baseline still matches the spec's Phase 0 table
- [ ] Record `sim-urdf` pass count (needed for Phase 10 baseline —
      run `cargo test -p sim-urdf` and add the result to the baseline)
- [ ] Audit the test helper categorization (shared vs. local) in
      `mujoco_pipeline.rs` `#[cfg(test)]` section (L21476–L26722)
- [ ] Create branch: `refactor/structural-decompose`
- [ ] Commit Phase 0 as the starting point

---

## 6. Phase-Specific Hazards

### Phase 1: MARGIN WARNINGs + scatter hazard

- `types/model.rs` (~780 lines): 20 lines from limit. Fallback: move
  `is_ancestor()` (~14 lines) to `types/model_init.rs`.
- `types/model_init.rs` (~774 lines): raw source ~930 before whitespace
  removal. Fallback: move `compute_body_lengths` + `compute_dof_lengths`
  (~64 lines) to a separate file.
- `types/data.rs` (~710 lines): raw span ~805. Fallback: move
  `reset_to_keyframe` (~40 lines) to `types/data_keyframe.rs`.
- **Scatter**: `compute_body_lengths` and `compute_dof_lengths` (L12901–L12964)
  are physically located ~9,000 lines from the other `model_init` functions
  (L2890–L3744). Don't miss them.

### Phase 6: MARGIN WARNINGs + macro dependency

- `constraint/assembly.rs` (~750 lines): With `use` imports + `//!` doc may
  approach 800. Fallback: move `populate_efc_island` (62 lines) to
  `constraint/mod.rs`.
- `constraint/solver/hessian.rs` (~685 lines): With overhead may approach 800.
  Fallback: split `SparseHessian` into `constraint/solver/sparse_hessian.rs`.
- **Macro**: `assemble_unified_constraints` (→ `assembly.rs`) contains a local
  `macro_rules! finalize_row` (L15469) that calls 5 functions from
  `constraint/impedance.rs`. The macro moves with the function body, but
  impedance must be extracted **before** assembly. The Phase 6 extraction order
  already enforces this (impedance = step 2, assembly = step 5). Do not reorder.

### Phase 8a: Line range hazard

**`forward/passive.rs`**: The tendon implicit K/D helpers at L12690–L12816 sit
**between** the two passive force ranges (L12108–L12689 and L12818–L12899) but
belong to `integrate/implicit.rs` (Phase 8b), **NOT** to `forward/passive.rs`.
Only take L12108–L12689 and L12818–L12899 for passive. (L12690 is the first
line of `tendon_all_dofs_sleeping` — it belongs to implicit.rs.)

### Phase 8b: Naming hazard

**`mj_integrate_pos`** → `integrate/euler.rs` (Euler position integration).
**`mj_integrate_pos_explicit`** → `jacobian.rs` (Phase 8a — Jacobian utility
for finite-difference derivatives). The names differ by one suffix. Don't
confuse them.

### Phase 10: MARGIN WARNING + feature gate + shared constants

- `builder/mod.rs` (~732 lines): Only under 800 **because** `new()` (~264
  lines) is split out to `builder/init.rs`. If `init.rs` is not split, mod.rs
  would be ~996 lines. Extract `init.rs` immediately after `mod.rs`.
- **Feature gate**: `sim-mjcf` has an `mjb` feature (`mjb.rs`). Run
  `cargo test -p sim-mjcf --features mjb` at least once per Phase 10 session
  to verify no regression behind the feature gate.
- **Shared constants**: `DEFAULT_SOLREF` and `DEFAULT_SOLIMP` (L41, L49) are
  used by 6+ target modules (joint, geom, tendon, equality, flex, contact).
  Place them in `builder/mod.rs` as `pub(crate) const` items so all
  sub-modules can access them via `super::DEFAULT_SOLREF`. Move them during
  the first session (builder/mod.rs extraction).

---

## 7. Phase 12 — Doc Sweep

The ~692 doc reference updates are mechanical find-and-replace using the Doc
Reference Mapping Tables (one for
[sim-core](./STRUCTURAL_REFACTOR.md#doc-reference-mapping-table), one for
sim-mjcf immediately below it in the same file).

**Strategy**: Batch by target module to minimize errors. For each target module
(e.g., `constraint/assembly.rs`), find all `future_work_*.md` lines citing
source ranges that map to that module and update them in one pass. Diff-review
the changes before committing.

---

## 8. Abort Criteria

Stop and reassess if any of these occur:

- Test count changes (any test appearing or disappearing)
- A module exceeds 800 production lines after extraction — first try
  consolidating `use` imports (one grouped `use` instead of many single lines);
  if still over, trigger the documented per-module fallback in the spec
- A circular import that wasn't anticipated in the audit findings
- A moved function contains a `macro_rules!` that references symbols from a
  module not yet extracted — move the dependency module first, or move both in
  the same step
- Any physics output change (same inputs must produce bitwise-identical outputs)

If an abort criterion is triggered, see the
[Rollback Procedure](./STRUCTURAL_REFACTOR.md#rollback-procedure) in the spec
for revert protocol.

---

## 9. Progress Tracking

After each commit, update this table. Sub-module order within each phase
respects internal call dependencies (matches the spec's extraction orders).
The **Session** column records which context window performed the extraction
(e.g., "S1", "S2") — useful for tracking progress across multiple sessions
and correlating with commit history.

| Phase | Sub-module | Notes | Status | Commit | Session |
|-------|-----------|-------|--------|--------|---------|
| 0 | Preparation | Baseline verified: 1,526/0/15; sim-urdf: 34/0/1; 13 test helpers all Local | done | — | S1 |
| 1 | types/mod.rs + enums.rs | Grouped — mod.rs is re-exports; extract enums first | | | |
| 1 | types/model.rs | **(MARGIN)** ~780 lines | | | |
| 1 | types/model_init.rs | **(MARGIN)** ~774 lines; **SCATTER**: L12901–L12964 | | | |
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
| 8a | forward/passive.rs | **(HAZARD)** skip L12690–L12816 (→ Phase 8b); take up to L12689 | | | |
| 8a | forward/actuation.rs | | | | |
| 8a | forward/muscle.rs | | | | |
| 8a | forward/acceleration.rs | | | | |
| 8a | forward/check.rs | | | | |
| 8a | jacobian.rs | **(HAZARD)** Top-level; includes `mj_integrate_pos_explicit` | | | |
| 8b | integrate/mod.rs | | | | |
| 8b | integrate/euler.rs | **(HAZARD)** `mj_integrate_pos` (NOT `_explicit`) | | | |
| 8b | integrate/implicit.rs | Extracts L12690–L12816 (skipped in Phase 8a) | | | |
| 8b | integrate/rk4.rs | | | | |
| 8c | island/mod.rs | | | | |
| 8c | island/sleep.rs | | | | |
| 10 | builder/mod.rs | **(MARGIN)** ~732 lines — requires init.rs split | | | |
| 10 | builder/init.rs | Split from mod.rs for MARGIN compliance | | | |
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
| 12 | future_work_*.md doc updates (~692) | | | | |
| 12 | ARCHITECTURE.md rewrite | | | | |
| 12 | Other doc + test comment updates | GAP_ANALYSIS, CONFORMANCE, REFERENCE, CLAUDE.md, test files, gjk_epa.rs (TRAIT_ARCHITECTURE has 0 references — no update needed) | | | |
| 12 | Final workspace verification + grading | All 15 final-state checks from rubric | | | |
