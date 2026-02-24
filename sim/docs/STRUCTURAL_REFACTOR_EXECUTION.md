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
   - **Annotate monolith imports**: Every `use crate::mujoco_pipeline::...`
     in a new file must have a trailing comment indicating when it will be
     removed (e.g., `// monolith: removed in Phase 8a`). This makes
     audit rounds faster — the annotation proves the import was
     intentional, not a lazy routing mistake.
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

**`#[cfg(test)]`-gated modules**: If a module is truly test-only (not imported
by external test crates), place `#[cfg(test)]` on the `mod` declaration in the
parent, not on the file contents. Note: `types/model_factories.rs` is **NOT**
`#[cfg(test)]`-gated because it is used by `sim-conformance-tests` and other
external test crates — it must be unconditionally compiled.

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
12a. Move inline tests. If the monolith has a `#[cfg(test)] mod <name>_tests`
    block whose tests exercise functions that just moved to this sub-module,
    move the entire test module (including local helper functions) into the
    new file. Consult the test helper audit table in the spec (Phase 0)
    to verify all helpers in the block are Local. Do not leave test modules
    behind in the monolith when their production code has moved.
13. cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics
    If tests fail: the most common causes are (a) a visibility change
    (pub → pub(crate)) that broke an external test, (b) a use import
    routing through the monolith shim instead of the real module, or
    (c) a test helper that wasn't moved with its code. Bisect by
    restoring items to the monolith one at a time until the failure is
    isolated. Do not commit a sub-module with failing tests.
14. cargo clippy -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics -- -D warnings
15. Verify pass count matches baseline
    (see spec Phase 0 table — original 4-crate baseline: 1,526 passed, 0 failed,
    15 ignored; current 11-crate baseline: 2,007 passed, 0 failed, 20 ignored)
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
| Pass count baseline (step 15) | Original 4-crate: 1,526/0/15; current 11-crate: 2,007/0/20 (see step 15) | Extend baseline to include `sim-urdf` — record its pass count during Phase 0 |
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

- [x] Verify the test baseline still matches the spec's Phase 0 table
- [x] Record `sim-urdf` pass count (needed for Phase 10 baseline —
      run `cargo test -p sim-urdf` and add the result to the baseline)
- [x] Audit the test helper categorization (shared vs. local) in
      `mujoco_pipeline.rs` `#[cfg(test)]` section (L21476–L26722)
- [x] Create branch: `refactor/structural-decompose`
- [x] Commit Phase 0 as the starting point

---

## 6. Phase-Specific Hazards

> **Note on line numbers**: All `L____` references in this document are
> pre-refactor snapshot positions (before commit `d018c7f`). Each extraction
> phase shifts the remaining monolith lines. Use **function names** to locate
> code; line numbers are approximate locator aids, not exact positions.

### Phase 1: MARGIN WARNINGs + scatter hazard **(RESOLVED)**

All Phase 1 hazards were handled during extraction. Doc-heavy exemption
(rubric S1) applied to model.rs (809 awk / 306 code), model_init.rs
(842 awk / 627 code), and data.rs (837 awk / 382 code). Scatter functions
were located and moved correctly.

- ~~`types/model.rs` (~780 lines): 20 lines from limit.~~ Fallback not needed.
- ~~`types/model_init.rs` (~774 lines): raw source ~930.~~ Fallback not needed.
- ~~`types/data.rs` (~710 lines): raw span ~805.~~ Fallback not needed.
- ~~**Scatter**: `compute_body_lengths` and `compute_dof_lengths`.~~ Found and moved.

### Phase 6: MARGIN WARNINGs + macro dependency **(RESOLVED)**

All Phase 6 hazards were handled during extraction. Both MARGIN files
landed well under 800 lines without needing fallbacks (assembly.rs: 735,
hessian.rs: 721). Macro dependency order enforced correctly (impedance
step 2 before assembly step 5).

- ~~`constraint/assembly.rs` (~750 lines): With `use` imports + `//!` doc may
  approach 800.~~ Actual: 735 lines. Fallback not needed.
- ~~`constraint/solver/hessian.rs` (~685 lines): With overhead may approach 800.~~
  Actual: 721 lines. Fallback not needed.
- ~~**Macro**: `finalize_row` in assembly.rs calls impedance functions.~~
  Extraction order enforced: impedance = step 2, assembly = step 5.

### Phase 8a: Line range hazard **(RESOLVED)**

All Phase 8a hazards were handled during extraction. Tendon implicit K/D
helpers correctly left in monolith for Phase 8b. `mj_integrate_pos_explicit`
extracted to `jacobian.rs`; `mj_integrate_pos` left for Phase 8b. `compute_muscle_params`
left in monolith (heavy deps). ~3,100 lines moved; monolith 8,275 → 5,364.

- ~~**`forward/passive.rs`**: The tendon implicit K/D helpers at L12690–L12816 sit
  **between** the two passive force ranges (L12108–L12689 and L12818–L12899) but
  belong to `integrate/implicit.rs` (Phase 8b), **NOT** to `forward/passive.rs`.~~
  Hazard avoided: only L12108–L12689 and L12818–L12899 taken for passive.

### Phase 8b: Naming hazard **(RESOLVED)**

All Phase 8b hazards were handled during extraction. `mj_integrate_pos`
correctly placed in `integrate/euler.rs`; `mj_integrate_pos_explicit` already
in `jacobian.rs` (Phase 8a). ~566 lines moved; monolith 5,364 → 4,798.

- ~~**`mj_integrate_pos`** → `integrate/euler.rs` (Euler position integration).~~
  Hazard avoided: placed correctly, not confused with `_explicit` variant.
- ~~**`mj_integrate_pos_explicit`** → `jacobian.rs` (Phase 8a).~~
  Already extracted; no action needed.

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
| 0 | Preparation | Baseline verified: 1,526/0/15; sim-urdf: 34/0/1; 15 test helpers all Local (corrected from 13 — 2 missed in initial scan) | done | — | S1 |
| 1 | types/mod.rs + enums.rs | Grouped — mod.rs is re-exports; extract enums first | done | d018c7f | S1 |
| 1 | types/model.rs | **(MARGIN)** 809 awk / 306 code-only — doc-heavy type def exempt per rubric S1 | done | — | S1 |
| 1 | types/model_init.rs | **(MARGIN)** 842 awk / 627 code-only — doc-heavy type def exempt per rubric S1; **SCATTER** handled | done | 5912e68 | S2 |
| 1 | **Phase 1 audit** | Rounds 1–7: fixed A–K. Round 8: independent deep audit of all 8 types/ files + integration points — zero findings, all S1–S8 criteria A-grade. Phase 1 closed. 1,526/0/15 baseline preserved. | done | — | S7 |
| 1 | types/model_factories.rs | NOT cfg(test): used by sim-conformance-tests; 317 lines | done | 5912e68 | S2 |
| 1 | types/data.rs | **(MARGIN)** 837 awk / 382 code-only — doc-heavy type def exempt per rubric S1 | done | 5912e68 | S2 |
| 1 | types/contact_types.rs | | done | 5912e68 | S2 |
| 1 | types/keyframe.rs | | done | 5912e68 | S2 |
| 1 | lib.rs re-exports | Route pub API through types/ | done | 5912e68 | S2 |
| 2 | linalg.rs | 488 lines (419 prod + 69 test); cholesky_tests moved with code | done | d1c6e89 | S8 |
| 2 | dynamics/mod.rs + spatial.rs | Grouped — mod.rs is pure re-exports; spatial.rs 221 lines | done | d1c6e89 | S8 |
| 2 | joint_visitor.rs | 196 lines; JointContext/JointVisitor kept pub (visit_joints is pub on Model) | done | d1c6e89 | S8 |
| 2 | **Phase 2 audit** | Independent audit: all S1–S8 A-grade. One finding fixed (data.rs SpatialVector import routed through monolith shim → pointed to crate::dynamics). 1,526/0/15 baseline preserved. | done | 8db4b78 | S8 |
| 5 | tendon/mod.rs + fixed.rs | Grouped — mod.rs 79 lines, fixed.rs 36 lines | done | 54c86d8 | S9 |
| 5 | tendon/spatial.rs | 458 prod + 79 test lines; includes subquat + subquat_tests | done | 54c86d8 | S9 |
| 5 | tendon/wrap_math.rs | 590 prod + 90 test lines; includes wrap_inside_tests | done | 54c86d8 | S9 |
| 5 | **Phase 5 audit** | Independent audit: all S1–S8 A-grade. Zero findings. 2,007/0/20 (11-crate scope); 1,526/0/15 baseline preserved. | done | — | S9 |
| 4 | sensor/mod.rs | 56 prod + 1330 test lines; sensor_body_id + sensor_tests (32 tests) | done | 5670d4e | S8 |
| 4 | sensor/position.rs | 323 lines; mj_sensor_pos; imports geom_to_collision_shape from collision/narrow (updated in Phase 3) | done | 5670d4e | S8 |
| 4 | sensor/velocity.rs | 218 lines; mj_sensor_vel | done | 5670d4e | S8 |
| 4 | sensor/acceleration.rs | 216 lines; mj_sensor_acc; Touch/Force/Torque/Accelerometer | done | 5670d4e | S8 |
| 4 | sensor/postprocess.rs | 73 lines; sensor_write helpers + mj_sensor_postprocess | done | 5670d4e | S8 |
| 4 | sensor/derived.rs | 346 lines; 7 derived computation functions (subtree COM/momentum/angmom, body accel, force/torque) | done | 5670d4e | S8 |
| 4 | energy.rs | 111 lines; mj_energy_pos, mj_energy_vel, Data::total_energy() | done | 5670d4e | S8 |
| 4 | **Phase 4 audit** | Independent audit: all S1–S8 A-grade. Two cosmetic findings fixed (duplicate/missing monolith annotation comments). 2,007/0/20 (11-crate scope). Clippy clean. | done | — | S8 |
| 3 | collision/mod.rs | 472 prod + 426 test; check_collision_affinity, contact_param, contact_param_flex_rigid, solmix_weight, combine_solref, combine_solimp, mj_collision, mj_collision_flex; contact_param_tests (25 tests) | done | 7b2a841 | S11 |
| 3 | collision/narrow.rs | 305 lines; collide_geoms, geom_to_collision_shape, apply_pair_overrides, make_contact_from_geoms; GEOM_EPSILON + 3 constants | done | 7b2a841 | S11 |
| 3 | collision/pair_convex.rs | 314 lines; collide_sphere_sphere, collide_capsule_capsule, collide_sphere_capsule, collide_sphere_box | done | 7b2a841 | S11 |
| 3 | collision/pair_cylinder.rs | 548 lines; collide_cylinder_sphere, collide_cylinder_capsule, collide_capsule_box, collide_box_box, test_sat_axis | done | 7b2a841 | S11 |
| 3 | collision/plane.rs | 429 prod + 544 test; collide_with_plane, collide_cylinder_plane_impl, collide_ellipsoid_plane_impl; primitive_collision_tests (11 tests) | done | 7b2a841 | S11 |
| 3 | collision/mesh_collide.rs | 185 lines; collide_with_mesh, collide_mesh_plane | done | 7b2a841 | S11 |
| 3 | collision/hfield.rs | 93 lines; collide_with_hfield | done | 7b2a841 | S11 |
| 3 | collision/sdf_collide.rs | 131 lines; collide_with_sdf | done | 7b2a841 | S11 |
| 3 | collision/flex_collide.rs | 210 lines; narrowphase_sphere_geom, make_contact_flex_rigid | done | 7b2a841 | S11 |
| 3 | **Phase 3 audit** | Independent audit: all S1–S8 A-grade. Zero findings. Lazy import check passed (errors only in monolith). sensor/position.rs import updated. 2,007/0/20 (11-crate scope). Clippy clean. | done | — | S11 |
| 7 | dynamics/crba.rs | 395 lines; mj_crba, cache_body_effective_mass, MIN_INERTIA_THRESHOLD, DEFAULT_MASS_FALLBACK | done | 1f69392 | S12 |
| 7 | dynamics/rne.rs | 362 lines; mj_rne, mj_gravcomp (imports mj_apply_ft from monolith until Phase 8a) | done | 1f69392 | S12 |
| 7 | dynamics/factor.rs | 237 lines; mj_factor_sparse, mj_factor_sparse_selective, Model::compute_qld_csr_metadata (split impl block) | done | 1f69392 | S12 |
| 7 | dynamics/flex.rs | 19 lines; mj_flex | done | 1f69392 | S12 |
| 7 | **Phase 7 audit** | Independent audit: all S1–S8 A-grade. Zero findings. Lazy import check passed (errors only in monolith). model_init.rs updated to import directly from dynamics::crba. 2,007/0/20 (11-crate scope). Clippy clean. | done | — | S12 |
| 6 | constraint/mod.rs | 426 lines; mj_fwd_constraint_islands, mj_fwd_constraint, compute_qacc_smooth, build_m_impl_for_newton, compute_qfrc_smooth_implicit, compute_point_velocity | done | 2a12dfe | S13 |
| 6 | constraint/impedance.rs | 331 lines; compute_impedance, compute_kbip, compute_aref, compute_regularization, etc. | done | 264a33a | S13 |
| 6 | constraint/jacobian.rs | 330 lines; compute_contact_jacobian, compute_flex_contact_jacobian, add_angular_jacobian | done | fba1dbd | S13 |
| 6 | constraint/equality.rs | 661 lines; all extract_*_jacobian, add_body_*_jacobian_row, get_min_* helpers | done | 0478367 | S13 |
| 6 | constraint/assembly.rs | **(MARGIN)** 735 lines; assemble_unified_constraints, tendon_deadband_displacement (populate_efc_island deferred — uses island data, moved in Phase 8c) | done | 64159d5 | S13 |
| 6 | constraint/solver/mod.rs | 77 lines; decode_pyramid, compute_qfrc_constraint_from_efc, extract_qfrc_frictionloss | done | f960b7d | S13 |
| 6 | constraint/solver/primal.rs | 676 lines; compute_gradient_and_search(_sparse), PrimalQuad, PrimalPoint, primal_prepare/eval/search, evaluate_cost_at | done | f20d719 | S13 |
| 6 | constraint/solver/pgs.rs | 488 lines; pgs_solve_unified, pgs_cost_change, classify_constraint_states, compute_delassus_regularized | done | c6a15cc | S13 |
| 6 | constraint/solver/cg.rs | 310 lines; cg_solve_unified | done | 7e7744d | S13 |
| 6 | constraint/solver/hessian.rs | **(MARGIN)** 721 lines; assemble_hessian, SparseHessian struct + impl, hessian_incremental, hessian_cone | done | 4ed8179 | S13 |
| 6 | constraint/solver/newton.rs | 352 lines; NewtonResult, newton_solve, recover_newton | done | 2247036 | S13 |
| 6 | constraint/solver/noslip.rs | 748 lines; project_elliptic_cone, noslip_qcqp2/3, NoslipRowKind, noslip_postprocess | done | a17d28d | S13 |
| 6 | **Phase 6 audit** | Independent audit: all S1–S8 A-grade. One finding (F1 — stale monolith comment in constraint/mod.rs, fixed). DAG annotations corrected in rubric. 2,007/0/20 (11-crate scope). Clippy clean. | done | — | S13 |
| 8a | forward/mod.rs | 238 lines; step(), forward(), forward_core(), forward_skip_sensors() orchestration | done | b62d746 | S14 |
| 8a | forward/position.rs | 601 lines; mj_fwd_position, aabb_from_geom, SweepAndPrune, closest_point_segment, closest_points_segments | done | b62d746 | S14 |
| 8a | forward/velocity.rs | 125 lines; mj_fwd_velocity, tendon velocities | done | b62d746 | S14 |
| 8a | forward/passive.rs | 715 lines; fluid/aero helpers, mj_fwd_passive, PassiveForceVisitor. **(HAZARD OK)** tendon implicit K/D helpers left in monolith for Phase 8b | done | b62d746 | S14 |
| 8a | forward/actuation.rs | 461 lines; mj_transmission_site, mj_transmission_body_dispatch, mj_actuator_length, mj_fwd_actuation | done | b62d746 | S14 |
| 8a | forward/muscle.rs | 122 lines; muscle F-L-V curves, sigmoid, muscle_activation_dynamics. compute_muscle_params left in monolith (heavy deps) | done | b62d746 | S14 |
| 8a | forward/acceleration.rs | 331 lines; mj_fwd_acceleration dispatch, explicit, implicit, implicitfast, implicit_full, ImplicitSpringVisitor | done | b62d746 | S14 |
| 8a | forward/check.rs | 51 lines; mj_check_pos, mj_check_vel, mj_check_acc | done | b62d746 | S14 |
| 8a | jacobian.rs | 455 lines; mj_jac, mj_jac_site/body/point/body_com/geom, mj_apply_ft, mj_differentiate_pos, mj_integrate_pos_explicit. **(HAZARD OK)** mj_integrate_pos (Phase 8b) left in monolith | done | b62d746 | S14 |
| 8a | **Phase 8a audit** | Independent audit: all S1–S8 A-grade. Zero code findings. Checklist and progress table updated. 2,007/0/20 (11-crate scope). Clippy clean. | done | 6a4724a | S14 |
| 8b | integrate/euler.rs | 180 lines; mj_integrate_pos, mj_normalize_quat, PositionIntegrateVisitor, QuaternionNormalizeVisitor. **(HAZARD OK)** mj_integrate_pos (NOT `_explicit`) correctly placed | done | 2d8bb9f | S15 |
| 8b | integrate/implicit.rs | 117 lines; tendon_all_dofs_sleeping, tendon_active_stiffness, accumulate_tendon_kd. Extracts tendon K/D helpers skipped in Phase 8a | done | 2d8bb9f | S15 |
| 8b | integrate/rk4.rs | 191 lines; mj_runge_kutta (classic RK4 Butcher tableau) | done | 2d8bb9f | S15 |
| 8b | integrate/mod.rs | 140 lines; Data::integrate() + integrate_without_velocity() dispatch | done | 2d8bb9f | S15 |
| 8b | **Phase 8b audit** | Independent audit: all S1–S8 A-grade. Zero findings. Lazy import check passed. 2,007/0/20 (11-crate scope). Clippy clean. | done | — | S15 |
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
