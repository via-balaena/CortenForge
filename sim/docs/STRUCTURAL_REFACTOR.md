# sim-core Structural Refactor

> **Status**: Spec draft — 2026-02-22
> **Scope**: Decompose `mujoco_pipeline.rs` (26,722 lines) and `model_builder.rs`
> (10,184 lines total; ~6,032 production + ~4,152 tests) into navigable module
> trees. Zero physics changes. Zero API changes.
> Every test passes identically before and after.
>
> **Grading**: [STRUCTURAL_REFACTOR_RUBRIC.md](./STRUCTURAL_REFACTOR_RUBRIC.md) —
> 8 structural criteria (S1–S8), all must be A-grade per phase.

---

## Why

`mujoco_pipeline.rs` is a 26,722-line file containing ~457 functions (named `fn` definitions; `grep 'fn '` returns ~459 including function pointer types and closures) and 149
inline tests. It holds the Model struct, the Data struct, all enums, forward
kinematics, collision detection, three constraint solvers, island discovery,
sleeping, five integrators, actuation, sensors, tendons, flex bodies, Jacobians,
spatial algebra, muscle dynamics, and linear algebra utilities — in one file.

A newcomer opens `sim-core/src/` and sees 12 files, one of which is impenetrable.
An AI assistant loses context partway through the file. A `grep` for any function
name returns hits buried in a 27K-line wall. The crate-level architecture is sound
(13 crates, clean L0/L1 split), but the two biggest crates are internally flat.

MuJoCo itself splits its engine across ~15 C files:

| MuJoCo file | Scope |
|-------------|-------|
| `engine_forward.c` | `mj_step`, `mj_forward`, top-level orchestration |
| `engine_core_smooth.c` | FK, velocity, CRBA, RNE |
| `engine_core_constraint.c` | Constraint assembly, solver dispatch |
| `engine_solver.c` | PGS/CG/Newton implementations |
| `engine_collision_*.c` | Broad phase, narrow phase per pair type |
| `engine_passive.c` | Springs, dampers, fluid, flex bending |
| `engine_sensor.c` | All sensor evaluation |
| `engine_island.c` | Island discovery, sleeping |
| `engine_derivative.c` | Analytical derivatives |
| `engine_io.c` | Model/Data I/O |
| `engine_util_*.c` | Linear algebra, spatial algebra |

We ended up *more* monolithic than the C codebase we're reimplementing. This
refactor brings us to parity with MuJoCo's own file organization, adapted to
Rust's module system.

### What this unlocks

1. **Navigability** — each file has a clear, bounded purpose
2. **AI assistability** — any module fits in a context window; Claude can read
   one module and understand it completely without needing the other 20K lines
3. **Parallel development** — two people (or two sessions) can work on
   `solver/newton.rs` and `collision/narrow.rs` without merge conflicts
4. **Incremental understanding** — a beginner can read `forward/mod.rs` (~90
   lines of orchestration) and understand the pipeline stages without drowning
   in implementation details
5. **Future refactors** — the trait architecture (TRAIT_ARCHITECTURE.md) needs
   seams; this creates them

---

## Audit Findings and Resolutions

The proposed module boundaries were stress-tested against the actual call graph,
shared mutable state patterns, and function sizes. Eight risks were identified.
Here is the resolution for each.

### Finding 1: `assemble_unified_constraints` is 685 lines

This single function would consume almost the entire 800-line budget of
`constraint/assembly.rs`.

**Resolution**: `assemble_unified_constraints` has clear internal phases
(count rows → allocate → populate equality/friction/limits/contacts → count
types). It stays as one function (splitting it would lose the unified
assembly semantics), but `constraint/assembly.rs` gets only this function plus
`populate_efc_island` (~59 lines), totaling **~750 lines** — under the limit.
The equality Jacobian extraction functions (extract_connect_jacobian, etc.)
move to a separate `constraint/equality.rs` (626 lines).

### Finding 2: RK4 calls back into `forward_core` (circular dependency)

`mj_runge_kutta` (integration) calls `data.forward_skip_sensors(model)` which
calls `forward_core()` — the entire forward pipeline re-runs 3 times.

**Resolution**: This is not a module-level circular dependency. Both
`forward_core` and `mj_runge_kutta` are methods on `impl Data`. In Rust,
`impl` blocks for the same type can be split across files in the same crate.
`forward_core` lives in `forward/mod.rs` as an `impl Data` method.
`mj_runge_kutta` lives in `integrate/rk4.rs` as a free function taking
`&mut Data`. It calls `data.forward_skip_sensors(model)` as a method call on
the `Data` receiver — no module import needed, just a method call on `self`/`data`.

### Finding 3: `mj_fwd_position` embeds `mj_fwd_tendon`

Tendon kinematics runs inside the position stage (after site poses, before
subtree COM), not as a separate pipeline step.

**Resolution**: `mj_fwd_tendon` lives in `tendon/mod.rs` as a `pub(crate)`
free function. `forward/position.rs` calls `crate::tendon::mj_fwd_tendon(model, data)`.
The dependency direction is `forward/position → tendon`, which is correct:
tendon computation depends on site poses (computed earlier in position), and
position orchestration calls tendon. No cycle.

The `forward/mod.rs` pipeline orchestration example in the rubric must be
corrected: tendon is called from WITHIN `mj_fwd_position`, not as a separate
top-level call in `forward_core`.

### Finding 4: Bidirectional imports between pipeline and `derivatives.rs`

`mj_fwd_acceleration_implicitfast` and `_implicit_full` import from
`crate::derivatives`, while `derivatives.rs` imports 15 symbols (12
`pub(crate)` + 3 `pub`) from the pipeline.

**Resolution**: This bidirectional dependency already exists and works —
Rust modules in the same crate can freely import from each other. After the
split, `forward/acceleration.rs` imports from `crate::derivatives`, and
`derivatives.rs` imports from `crate::dynamics::spatial`, `crate::linalg`,
etc. The dependency is diffused across smaller modules but functionally
identical. No cycle at the crate level.

### Finding 5: `mj_crba` modifies constraint-domain fields

`mj_crba` writes to `body_min_mass`, `body_min_inertia` (via
`cache_body_effective_mass`) and `qLD_*` (via `mj_factor_sparse_selective`).

**Resolution**: Module boundaries organize **code**, not **data access**. The
`Data` struct is shared mutable state passed as `&mut Data`. Any function
that receives `&mut Data` can write to any field. This is by design — MuJoCo
works the same way. `mj_crba` lives in `dynamics/crba.rs` because it
implements the Composite Rigid Body Algorithm. The fact that it also triggers
factorization and caches effective mass for constraints is part of the CRBA
stage, not a violation.

### Finding 6: `mj_fwd_acceleration_implicit` writes `data.qvel`

In implicit integration mode, velocity update happens inside the acceleration
computation, not in the separate integration step.

**Resolution**: Same as Finding 5 — module boundaries organize code, not
data ownership. `mj_fwd_acceleration_implicit` lives in
`forward/acceleration.rs` because it computes acceleration (and, for implicit
modes, simultaneously updates velocity). The `integrate/` module handles the
explicit integration paths. This split matches MuJoCo's own conceptual
separation.

### Finding 7: 15 functions (12 `pub(crate)` + 3 `pub`) span 6 domains

`derivatives.rs` imports symbols from spatial algebra, Jacobians, kinematics,
fluid, sleep, and linear algebra.

**Resolution**: Each module re-exports its `pub(crate)` symbols in its
`mod.rs`. `derivatives.rs` uses targeted imports from each module:

```rust
// pub(crate) imports — these change path when their source module is extracted
use crate::dynamics::spatial::{spatial_cross_motion, spatial_cross_force, object_velocity_local};
use crate::forward::passive::{fluid_geom_semi_axes, ellipsoid_moment, norm3};
use crate::joint_visitor::joint_motion_subspace;
use crate::jacobian::{mj_jac_body_com, mj_jac_geom};
use crate::linalg::{cholesky_solve_in_place, lu_solve_factored};
use crate::integrate::implicit::tendon_all_dofs_sleeping;

// pub imports — these route through lib.rs re-exports and don't need
// pub(crate) import path changes during extraction
use crate::{mj_differentiate_pos, mj_integrate_pos_explicit, mj_solve_sparse};
```

This is more explicit than the current `use crate::mujoco_pipeline::*` and
is better for readability — you can see exactly which modules derivatives
depends on. During the refactor, only the 12 `pub(crate)` imports change
path as their source modules are extracted; the 3 `pub` functions
(`mj_differentiate_pos`, `mj_integrate_pos_explicit`, `mj_solve_sparse`)
route through `lib.rs` re-exports and are unaffected.

### Finding 8: JointVisitor impls are scattered across 4 domains

Five `impl JointVisitor` blocks exist: in CRBA (MassCacheVisitor), passive
forces (PassiveForceVisitor), implicit acceleration (ImplicitSpringVisitor),
and integration (PositionIntegrateVisitor, QuaternionNormalizeVisitor).

**Resolution**: This is correct Rust design. The `JointVisitor` trait and
`JointContext` struct and `Model::visit_joints()` driver live in a shared
`joint_visitor.rs` module. Each visitor implementation is a local struct
defined inside the function that uses it (e.g., `MassCacheVisitor` is defined
inside `cache_body_effective_mass`). These implementations stay with their
host functions when those functions move to new modules. The visitor pattern
is specifically designed for this — the trait is shared, the impls are local.

### Constraint/Solver Module Revised Structure

The original spec proposed a simple `pgs.rs` / `cg.rs` / `newton.rs` split.
The audit revealed the constraint block is **~5,612 lines** with 45+ functions,
requiring a finer decomposition. Revised structure (all modules ≤800 lines):

```
constraint/
  mod.rs              ~400 lines — mj_fwd_constraint, mj_fwd_constraint_islands,
  │                               compute_qacc_smooth, build_m_impl_for_newton,
  │                               compute_qfrc_smooth_implicit, compute_point_velocity
  │                               Line ranges: compute_qacc_smooth (L15275–L15315, 41 lines),
  │                               mj_fwd_constraint_islands (L19686–L19691, 6 lines),
  │                               build_m_impl_for_newton (L19715–L19742, 28 lines),
  │                               compute_qfrc_smooth_implicit (L19759–L19842, 84 lines),
  │                               mj_fwd_constraint (L19850–L20006, 157 lines),
  │                               compute_point_velocity (L20013–L20028, 16 lines).
  │                               Total named functions: 332 lines; the ~400 estimate includes
  │                               whitespace, comments, and local structs between functions.
  assembly.rs        ~750 lines  — assemble_unified_constraints (~685 body), populate_efc_island
  │                               (~59 body), totaling ~750 with signatures + doc comments.
  │                               **MARGIN WARNING**: ~750 is function bodies only. The extracted
  │                               module will need ~20–30 lines of `use` imports + `//!` doc
  │                               comment. If the total exceeds 800, the fallback is to move
  │                               `populate_efc_island` (62 lines) to `constraint/mod.rs`.
  equality.rs         626 lines  — extract_{connect,weld,joint,tendon,distance}_jacobian,
  │                               add_body_{point,angular}_jacobian_row,
  │                               get_min_{diagonal_mass,translational_mass,rotational_inertia}
  jacobian.rs        ~337 lines  — compute_flex_contact_jacobian, compute_contact_jacobian,
  │                               add_angular_jacobian (L14269–L14606 — contact Jacobian
  │                               construction for constraint assembly)
  impedance.rs       ~343 lines  — compute_impedance, quaternion_to_axis_angle, compute_kbip,
  │                               compute_aref, normalize_quat4, ball_limit_axis_angle,
  │                               compute_diag_approx_exact, mj_solve_sparse_vec,
  │                               compute_regularization (L14932–L15274 — constraint impedance,
  │                               stiffness/damping precomputation, diagonal approximation)
  solver/
    mod.rs            ~130 lines  — compute_delassus_regularized, compute_qfrc_constraint_from_efc,
    │                              extract_qfrc_frictionloss, decode_pyramid, solver dispatch
    │                              (named functions ~114 lines; ~130 with imports/docs/dispatch)
    pgs.rs           ~400 lines  — pgs_solve_unified, pgs_cost_change, classify_constraint_states
    cg.rs            ~280 lines  — cg_solve_unified
    newton.rs        ~313 lines  — newton_solve, recover_newton
    hessian.rs       ~685 lines  — assemble_hessian, SparseHessian struct + 7 impl methods
    │                              (assemble, refactor, fill_numeric, find_entry,
    │                              symbolic_factor, numeric_factor, solve),
    │                              hessian_incremental, hessian_cone
    │                              **MARGIN WARNING**: ~685 is function/struct bodies only.
    │                              With `use` imports, `//!` doc, `NV_SPARSE_THRESHOLD`
    │                              const, and whitespace, the extracted module may approach
    │                              800. If it exceeds, split `SparseHessian` into
    │                              `constraint/solver/sparse_hessian.rs`.
    primal.rs        ~653 lines  — compute_gradient_and_search{,_sparse}, primal_prepare,
    │                              primal_eval, primal_search, evaluate_cost_at,
    │                              PrimalQuad + PrimalPoint struct definitions
    noslip.rs        ~695 lines  — project_elliptic_cone (L14607), noslip_qcqp2 (L14661),
    │                              noslip_qcqp3 (L14757), noslip_postprocess (L18555–L18972,
    │                              ~418 lines)
```

**Total: ~5,612 lines across 12 files. Largest: ~750 lines (assembly.rs). All under 800.**

---

## Principles

1. **Zero behavioral change.** Same physics, same API, same test results. This
   is pure code movement.

2. **Module boundaries follow MuJoCo's file boundaries.** Not arbitrary — each
   module maps to a well-understood stage of the MuJoCo pipeline.

3. **`pub(crate)` by default.** Only types/functions already in `lib.rs`
   re-exports become `pub`. Everything else is `pub(crate)` or private. This
   prevents accidental API surface expansion.

4. **Tests move with their code.** Inline `#[cfg(test)]` blocks move to the
   module they test. Integration tests in `sim-conformance-tests` don't move
   (they test the public API, not internals).

5. **One PR per major module extraction.** Not one giant PR. Each extraction is
   independently reviewable, independently revertible, and independently
   testable.

6. **No new features, no cleanups, no "while we're here."** The temptation to
   fix something while moving it is strong. Resist it. Physics changes go in
   separate commits after the refactor is complete.

---

## Intermediate State Compilation

During phases 1–8c, the sim-core monolith (`mujoco_pipeline.rs`) and the new modules
coexist. The monolith shrinks each phase. During Phase 10, the sim-mjcf monolith
(`model_builder.rs`) and the new `builder/` modules coexist. To keep every intermediate
state compilable:

- The monolith continues to `pub(crate)` export everything it still contains.
- New modules import from `crate::mujoco_pipeline::*` for symbols not yet extracted.
- As each symbol moves to its final module, imports in *all* files (including other
  new modules) are updated to point to the real location — never through the shim.
- Phase 12 removes both shims entirely. Any remaining `crate::mujoco_pipeline::` or
  `crate::model_builder::` import is a bug.
- **Monolith re-imports**: After extracting symbols, the shrinking monolith
  needs `pub(crate) use crate::types::*;` (etc.) re-imports at its top so
  remaining code can still use bare names. Without these, every remaining
  function referencing an extracted symbol fails to compile. These re-imports
  are removed in Phase 12 when the monolith is deleted.

**`lib.rs` complexity peak**: During intermediate phases, `lib.rs` will
simultaneously re-export from both the monolith (`mujoco_pipeline::*`) and
from new modules (`types::Model`, `constraint::mj_fwd_constraint`, etc.).
This duplication is expected and correct — the monolith re-imports ensure
remaining monolith code compiles, while `lib.rs` re-exports ensure the
public API routes through real module paths. The duplication resolves in
Phase 12 when the monolith is deleted. Do not try to "clean up" `lib.rs`
during intermediate phases.

**Import update scope**: "All files" means both newly created modules AND
existing files (`derivatives.rs`, `batch.rs`, `lib.rs`, etc.) that import
symbols whose home module changed in this phase. The monolith re-imports
(`pub(crate) use crate::types::*;` etc.) provide a compilation safety net,
but imports in existing files should be updated to point to the real location
in the same phase that moves the symbol — don't defer to Phase 12.

This means at any commit during phases 1–8c and 10:
1. `cargo check -p sim-core` passes
2. `cargo test -p sim-core` passes
3. No import points through the re-export shim to a symbol that already has a real home

---

## Target Structure: sim-core

### Current (12 files, total lines incl. tests)

```
sim-core/src/
  lib.rs                   255 lines
  mujoco_pipeline.rs    26,722 lines  ← THE PROBLEM
  derivatives.rs         2,746 lines
  collision_shape.rs     1,734 lines
  contact.rs             1,503 lines
  gjk_epa.rs             1,367 lines
  mesh.rs                2,021 lines
  mid_phase.rs           1,178 lines
  heightfield.rs           941 lines
  sdf.rs                 2,793 lines
  raycast.rs             1,150 lines
  batch.rs                 (batched sim)
```

### Target (~61 new files in module tree, ~71 total with existing files)

```
sim-core/src/
  lib.rs                      Module declarations + pub re-exports (unchanged API)
  │
  ├── types/
  │   ├── mod.rs              Re-exports
  │   ├── enums.rs            MjJointType, GeomType, SolverType, Integrator, SleepPolicy,
  │   │                       SleepState, ConstraintType, ConstraintState, EqualityType,
  │   │                       ActuatorDynamics, ActuatorTransmission, GainType, BiasType,
  │   │                       WrapType, TendonType, MjSensorType, MjSensorDataType,
  │   │                       MjObjectType, StepError, ResetError, SleepError
  │   ├── model.rs            Model struct definition + field accessors +
  │   │                       is_ancestor() + joint_qpos0() + qld_csr() (~780 lines)
  │   │                       (visit_joints → joint_visitor.rs,
  │   │                        compute_qld_csr_metadata → dynamics/factor.rs,
  │   │                        compute_muscle_params → forward/muscle.rs,
  │   │                        compute_spatial_tendon_length0 → tendon/mod.rs)
  │   ├── model_init.rs       empty(), make_data(), compute_ancestors(),
  │   │                       compute_implicit_params(), compute_stat_meaninertia(),
  │   │                       compute_body_lengths(), compute_dof_lengths()
  │   │                       (~774 lines — Model construction + precomputation)
  │   ├── model_factories.rs  factory helpers (NOT cfg(test) — used by external test crates):
  │   │                       n_link_pendulum(), double_pendulum(), spherical_pendulum(),
  │   │                       free_body() (~280 lines — model constructors for tests)
  │   ├── data.rs             Data struct + Clone impl + field accessors
  │   │                       (qld_diag, reset, reset_to_keyframe)
  │   │                       — NOT step/forward/integrate
  │   │                       (total_energy → energy.rs;
  │   │                        sleep_state, tree_awake, nbody_awake, nisland →
  │   │                        island/sleep.rs)
  │   ├── contact_types.rs    Contact, ContactPair, compute_tangent_frame
  │   └── keyframe.rs         Keyframe struct
  │
  ├── forward/
  │   ├── mod.rs              step() (dispatches on integrator: RK4 → mj_runge_kutta,
  │   │                       others → integrate; then sleep transitions + qacc_warmstart),
  │   │                       forward(), forward_skip_sensors(), forward_core()
  │   │                       — the top-level orchestration (calls into sub-modules)
  │   ├── position.rs         mj_fwd_position (FK from qpos → xpos/xquat/xmat),
  │   │                       aabb_from_geom, SweepAndPrune
  │   ├── velocity.rs         mj_fwd_velocity (body spatial velocities from qvel)
  │   ├── passive.rs          mj_fwd_passive (springs, dampers, friction loss,
  │   │                       fluid/aerodynamic forces, flex bending + vertex damping)
  │   ├── actuation.rs        mj_fwd_actuation, mj_actuator_length,
  │   │                       mj_transmission_site, mj_transmission_body,
  │   │                       mj_transmission_body_dispatch
  │   ├── muscle.rs           compute_muscle_params, muscle_* helpers (~259 lines)
  │   ├── acceleration.rs     mj_fwd_acceleration (dispatch), mj_fwd_acceleration_explicit,
  │   │                       _implicit, _implicitfast, _implicit_full,
  │   │                       ImplicitSpringVisitor — all four acceleration paths
  │   └── check.rs            mj_check_pos, mj_check_vel, mj_check_acc
  │
  ├── dynamics/
  │   ├── mod.rs              Re-exports
  │   ├── crba.rs             mj_crba, cache_body_effective_mass (CRBA + mass matrix)
  │   ├── rne.rs              mj_rne, mj_gravcomp (Recursive Newton-Euler + gravity comp)
  │   ├── factor.rs           mj_factor_sparse, mj_factor_sparse_selective,
  │   │                       compute_qld_csr_metadata
  │   │                       (sparse LDL factorization + structural precomputation;
  │   │                       sparse solvers → linalg.rs)
  │   ├── spatial.rs          spatial_cross_motion, spatial_cross_force,
  │   │                       compute_body_spatial_inertia, shift_spatial_inertia,
  │   │                       object_velocity_local, rotate_spatial_to_world, etc.
  │   └── flex.rs             mj_flex (flex vertex position sync, ~10 lines)
  │
  ├── constraint/
  │   ├── mod.rs              mj_fwd_constraint, mj_fwd_constraint_islands,
  │   │                       compute_qacc_smooth, build_m_impl_for_newton,
  │   │                       compute_qfrc_smooth_implicit, compute_point_velocity (~400 lines)
  │   ├── assembly.rs         assemble_unified_constraints (~685 lines),
  │   │                       populate_efc_island (~750 lines total)
  │   ├── equality.rs         extract_{connect,weld,joint,tendon,distance}_jacobian,
  │   │                       add_body_{point,angular}_jacobian_row,
  │   │                       get_min_diagonal_mass/translational/rotational (626 lines)
  │   ├── jacobian.rs         compute_flex_contact_jacobian, compute_contact_jacobian,
  │   │                       add_angular_jacobian (~337 lines — contact Jacobian
  │   │                       construction for constraint assembly)
  │   ├── impedance.rs        compute_impedance, quaternion_to_axis_angle, compute_kbip,
  │   │                       compute_aref, normalize_quat4, ball_limit_axis_angle,
  │   │                       compute_diag_approx_exact, mj_solve_sparse_vec,
  │   │                       compute_regularization (~343 lines — constraint impedance,
  │   │                       stiffness/damping precomputation, diagonal approximation)
  │   └── solver/
  │       ├── mod.rs          compute_delassus_regularized, compute_qfrc_constraint_from_efc,
  │       │                  extract_qfrc_frictionloss, decode_pyramid, solver dispatch (~130 lines)
  │       ├── pgs.rs          PGS solver, pgs_cost_change, classify_constraint_states (~400 lines)
  │       ├── cg.rs           CG solver (~280 lines)
  │       ├── newton.rs       Newton solver, recover_newton (~313 lines)
  │       ├── hessian.rs      assemble_hessian, SparseHessian struct + 7 impl methods,
  │       │                   hessian_incremental, hessian_cone (~685 lines)
  │       ├── primal.rs       Shared CG/Newton infrastructure — gradient, linesearch,
  │       │                   cost evaluation, PrimalQuad/PrimalPoint structs (~653 lines)
  │       └── noslip.rs       project_elliptic_cone, noslip_qcqp2, noslip_qcqp3,
  │                          noslip_postprocess (~695 lines)
  │
  ├── collision/
  │   │  NOTE: The collision/ module tree contains dispatch and primitive pair
  │   │  functions extracted from mujoco_pipeline.rs. The existing top-level files
  │   │  (collision_shape.rs, mesh.rs, heightfield.rs, sdf.rs, gjk_epa.rs,
  │   │  mid_phase.rs, raycast.rs) remain at the top level unchanged — they are
  │   │  independent libraries. The new collision/mesh_collide.rs,
  │   │  collision/hfield.rs, and collision/sdf_collide.rs are thin dispatch
  │   │  wrappers (~80-120 lines each) that route from the pipeline's
  │   │  geometry-type matching into those libraries.
  │   ├── mod.rs              mj_collision (broad + narrow dispatch), mj_collision_flex,
  │   │                       check_collision_affinity, contact parameter mixing
  │   ├── narrow.rs           collide_geoms dispatch, geom_to_collision_shape,
  │   │                       apply_pair_overrides, make_contact_from_geoms, GEOM_EPSILON
  │   ├── pair_convex.rs      sphere_sphere, capsule_capsule, sphere_capsule,
  │   │                       sphere_box, closest_point_segment, closest_points_segments (~310 lines)
  │   ├── pair_cylinder.rs    cylinder_sphere, cylinder_capsule, capsule_box,
  │   │                       box_box (SAT), test_sat_axis (~620 lines)
  │   ├── plane.rs            collide_with_plane, collide_cylinder_plane_impl,
  │   │                       collide_ellipsoid_plane_impl (ground plane collisions)
  │   ├── mesh_collide.rs     collide_with_mesh, collide_mesh_plane
  │   ├── hfield.rs           collide_with_hfield
  │   ├── sdf_collide.rs      collide_with_sdf
  │   └── flex_collide.rs     narrowphase_sphere_geom (flex vertex), make_contact_flex_rigid
  │
  ├── sensor/
  │   ├── mod.rs              Top-level sensor dispatch
  │   ├── position.rs         mj_sensor_pos (position-stage sensors)
  │   ├── velocity.rs         mj_sensor_vel (velocity-stage sensors)
  │   ├── acceleration.rs     mj_sensor_acc (acceleration-stage sensors)
  │   ├── postprocess.rs      mj_sensor_postprocess, sensor_write helpers
  │   └── derived.rs          compute_subtree_com, compute_subtree_momentum,
  │                           compute_body_acceleration, compute_body_angular_acceleration,
  │                           compute_site_force_torque, is_body_in_subtree,
  │                           compute_subtree_angmom
  │
  ├── tendon/
  │   ├── mod.rs              mj_fwd_tendon (dispatch), Model::compute_spatial_tendon_length0
  │   ├── fixed.rs            mj_fwd_tendon_fixed
  │   ├── spatial.rs          mj_fwd_tendon_spatial, accumulate_point_jacobian,
  │   │                       apply_tendon_force, subquat, WrapResult enum
  │   └── wrap_math.rs        sphere_tangent_point, compute_tangent_pair, circle_tangent_2d,
  │                           sphere_wrapping_plane, wrap_inside_2d,
  │                           directional_wrap_angle, segments_intersect_2d,
  │                           sphere_wrap, cylinder_wrap
  │
  ├── island/
  │   ├── mod.rs              mj_island, mj_flood_fill, equality_trees,
  │   │                       constraint_tree (island discovery + graph utilities)
  │   └── sleep.rs            mj_wake, mj_wake_collision, mj_wake_tendon,
  │                           mj_wake_equality, mj_wake_tree, mj_sleep,
  │                           mj_sleep_cycle, mj_update_sleep_arrays,
  │                           mj_check_qpos_changed, sensor_body_id,
  │                           tree_can_sleep, sleep_trees, sync_tree_fk,
  │                           reset_sleep_state
  │
  ├── integrate/
  │   ├── mod.rs              integrate() dispatch (Euler/Implicit*/RK4),
  │   │                       integrate_without_velocity() (GPU integration path)
  │   ├── euler.rs            Euler integration, mj_integrate_pos,
  │   │                       mj_normalize_quat, PositionIntegrateVisitor,
  │   │                       QuaternionNormalizeVisitor
  │   ├── implicit.rs         Tendon implicit K/D helpers shared by
  │   │                       forward/acceleration.rs and constraint/mod.rs:
  │   │                       tendon_all_dofs_sleeping, tendon_all_dofs_sleeping_fields,
  │   │                       tendon_deadband_displacement, tendon_active_stiffness,
  │   │                       accumulate_tendon_kd
  │   └── rk4.rs              mj_runge_kutta (4-stage Runge-Kutta)
  │
  ├── jacobian.rs             mj_jac, mj_jac_site, mj_jac_body, mj_jac_point,
  │                           mj_jac_body_com, mj_jac_geom, mj_apply_ft,
  │                           compute_contact_normal_jacobian,
  │                           mj_differentiate_pos, mj_integrate_pos_explicit
  │
  ├── energy.rs               mj_energy_pos, mj_energy_vel
  │                           (Standalone because energy is a query — called from
  │                           forward_core and from Data::total_energy — not a
  │                           pipeline stage. Matches MuJoCo's separation of energy
  │                           computation from integration.)
  │
  ├── linalg.rs               cholesky_in_place, cholesky_solve_in_place,
  │                           cholesky_rank1_update, cholesky_rank1_downdate,
  │                           lu_factor_in_place, lu_solve_factored,
  │                           mj_solve_sparse, mj_solve_sparse_batch, UnionFind
  │
  ├── joint_visitor.rs        JointVisitor trait, JointContext,
  │                           joint_motion_subspace, Model::visit_joints()
  │                           (visitor impls stay local to their host functions:
  │                           MassCacheVisitor in crba.rs, PassiveForceVisitor in
  │                           passive.rs, ImplicitSpringVisitor in acceleration.rs,
  │                           Position/QuatNormalize visitors in integrate/euler.rs)
  │
  │  ── (existing files, unchanged) ──
  ├── collision_shape.rs      CollisionShape, Aabb (already separate)
  ├── contact.rs              ContactPoint, ContactManifold, ContactForce (already separate)
  ├── gjk_epa.rs              GJK/EPA (already separate)
  ├── mesh.rs                 Triangle mesh collisions (already separate)
  ├── mid_phase.rs            BVH (already separate)
  ├── heightfield.rs          Height field (already separate)
  ├── sdf.rs                  SDF (already separate)
  ├── raycast.rs              Raycasting (already separate)
  ├── derivatives.rs          Transition derivatives (already separate)
  └── batch.rs                BatchSim (already separate)
```

### Module size estimates

> **Note on line numbers**: All `L____` references throughout this document are
> pre-refactor snapshot positions (before commit `d018c7f`). Each extraction
> phase shifts the remaining monolith lines. Use **function names** to locate
> code; line numbers are approximate locator aids, not exact positions.

| Module | Est. lines | Source (line ranges in current file) |
|--------|-----------|--------------------------------------|
| `types/enums.rs` | ~710 | L325–L383 + L455–L1100 (enums, error types — UnionFind moves to linalg.rs; JointContext/JointVisitor at L384–L454 → `joint_visitor.rs`) |
| `types/model.rs` | ~780 | L1142–L1870 (Model struct + accessors) + L3648–L3659 (joint_qpos0) + L4063–L4076 (is_ancestor) + qld_csr (inline accessor in Model impl). **MARGIN WARNING**: ~780 is 20 lines from limit. Fallback: move `is_ancestor()` (~14 lines) to `types/model_init.rs`. |
| `types/model_init.rs` | ~774 | L2890–L3646 + L3661–L3744 + L4033–L4062 + L12901–L12964 (empty, make_data, compute_ancestors, compute_implicit_params, compute_stat_meaninertia, compute_body_lengths, compute_dof_lengths — see doc reference mapping table for precise per-function ranges). **MARGIN WARNING**: raw source spans ~930 lines before whitespace/comment removal. Fallback: move `compute_body_lengths` + `compute_dof_lengths` (~64 lines) to a separate `types/model_precompute.rs` or into the target module they serve. |
| `types/model_factories.rs` | ~280 | Factory helpers (n_link_pendulum, double_pendulum, etc.) — NOT `#[cfg(test)]`: used by sim-conformance-tests |
| `types/data.rs` | ~710 | L2185–L2890 (Data struct + Clone) + L4385–L4483 (qld_diag, reset, reset_to_keyframe — the Data accessors that stay here; total_energy at L4485–L4489 → `energy.rs`, sleep_state/tree_awake/nbody_awake/nisland at L4491–L4519 → `island/sleep.rs`). Source ranges total ~805 lines; the ~710 estimate assumes whitespace/comment removal. **MARGIN WARNING**: raw span (~805) is near the 800 limit. Fallback: move `reset_to_keyframe` (~40 lines) to a separate `types/data_keyframe.rs`. |
| `types/contact_types.rs` | ~302 | L1870–L2171 (ContactPair, Contact struct, impl Contact, compute_tangent_frame) |
| `types/keyframe.rs` | ~20 | L1105–L1124 (Keyframe struct) |
| `forward/mod.rs` | ~200 | L4521–L4700 (step (doc comment at L4521, fn at L4537), forward, forward_skip_sensors, forward_core + mod declarations/re-exports/docs — the full orchestration is ~90 lines of call sequences) |
| `forward/position.rs` | ~500 | L4876–L5383 (mj_fwd_position + aabb + SAP) |
| `forward/velocity.rs` | ~114 | L9244–L9358 (body spatial velocities from qvel) |
| `forward/passive.rs` | ~660 | L12108–L12690 (fluid helpers + mj_fwd_passive) + L12818–L12899 (PassiveForceVisitor) |
| `forward/actuation.rs` | ~566 | L10682–L11247 (transmission + actuation, excluding L10795–L10829 which goes to jacobian.rs). Originally estimated at ~760 with compute_muscle_params included, but raw range totaled ~825 — exceeding the 800-line S1 limit. Split: muscle params moved to `forward/muscle.rs`. |
| `forward/muscle.rs` | ~259 | L3745–L4003 (compute_muscle_params + muscle_* helpers, joins in Phase 8a) |
| `forward/check.rs` | ~30 | L4822–L4876 (mj_check_pos, mj_check_vel, mj_check_acc — ~30 lines of function bodies within a 54-line range that includes comments/whitespace. These are trivial validation guards. May be inlined into `forward/mod.rs` if the separate file feels like overhead.) |
| `forward/acceleration.rs` | ~310 | L20029–L20084 (mj_fwd_acceleration dispatch) + L20085–L20104 + L20559–L20827 (4 accel paths) |
| `dynamics/crba.rs` | ~350 | L11247–L11598 |
| `dynamics/rne.rs` | ~350 | L11677–L12038 (includes doc comment starting at L11677; fn mj_rne at L11704) |
| `dynamics/factor.rs` | ~200 | L20276–L20444 (mj_factor_sparse, mj_factor_sparse_selective) + compute_qld_csr_metadata (~30 lines, joins from Model impl in Phase 7). Factorization + structural precomputation; sparse solvers go to `linalg.rs`. |
| `dynamics/spatial.rs` | ~300 | L106–L325 (spatial algebra) + L12038–L12108 |
| `dynamics/flex.rs` | ~10 | L9236–L9244 (mj_flex — flex vertex position sync, 6-line function + imports/doc) |
| `constraint/mod.rs` | ~400 | mj_fwd_constraint + dispatch (verified) |
| `constraint/assembly.rs` | ~750 | assemble_unified_constraints (~685 body) + populate_efc_island (~59 body) |
| `constraint/equality.rs` | 626 | Jacobian extraction for equality constraints (verified) |
| `constraint/jacobian.rs` | ~337 | L14269–L14606 (compute_flex_contact_jacobian, compute_contact_jacobian, add_angular_jacobian) |
| `constraint/impedance.rs` | ~343 | L14932–L15274 (compute_impedance, quaternion_to_axis_angle, compute_kbip, compute_aref, normalize_quat4, ball_limit_axis_angle, compute_diag_approx_exact, mj_solve_sparse_vec, compute_regularization) |
| `constraint/solver/pgs.rs` | ~400 | PGS + classify_constraint_states |
| `constraint/solver/cg.rs` | ~280 | CG solver |
| `constraint/solver/newton.rs` | ~313 | Newton solver + recover |
| `constraint/solver/hessian.rs` | ~685 | assemble_hessian, SparseHessian struct + 7 impl methods (L16882–L17408, ~527 lines), hessian_incremental, hessian_cone (verified) |
| `constraint/solver/primal.rs` | ~653 | Shared CG/Newton linesearch + cost + PrimalQuad/PrimalPoint structs |
| `constraint/solver/noslip.rs` | ~695 | L14607–L14883 (project_elliptic_cone, noslip_qcqp2, noslip_qcqp3) + L18555–L18972 (noslip_postprocess, ~418 lines) |
| `constraint/solver/mod.rs` | ~130 | Delassus, qfrc recovery, decode_pyramid, dispatch |
| `collision/mod.rs` | ~465 | L5383–L5639 (check_collision_affinity, mj_collision, mj_collision_flex) + L6068–L6240 (contact parameter mixing) |
| `collision/pair_convex.rs` | ~310 | L7095–L7406 (sphere/capsule/box pairwise) |
| `collision/pair_cylinder.rs` | ~620 | L7406–L8028 (cylinder pairs + box-box SAT) |
| `collision/plane.rs` | ~415 | L6673–L7088 (collide_with_plane, collide_cylinder_plane_impl, collide_ellipsoid_plane_impl) |
| `collision/narrow.rs` | ~272 | L5844–L6068 + L6241–L6288 (collide_geoms dispatch, geom_to_collision_shape, apply_pair_overrides, make_contact_from_geoms, constants — flex helpers at L5640–L5842 carved out to `collision/flex_collide.rs`) |
| `collision/hfield.rs` | ~87 | L6290–L6376 (collide_with_hfield) |
| `collision/sdf_collide.rs` | ~119 | L6378–L6496 (collide_with_sdf) |
| `collision/mesh_collide.rs` | ~174 | L6498–L6671 (collide_with_mesh + collide_mesh_plane) |
| `collision/flex_collide.rs` | ~202 | L5640–L5842 (narrowphase_sphere_geom (~156 lines), make_contact_flex_rigid (~46 lines) — flex-rigid collision helpers called by mj_collision_flex) |
| `sensor/mod.rs` | ~17 | L8118–L8134 (dispatch + mod declarations) |
| `sensor/position.rs` | ~289 | L8135–L8423 (mj_sensor_pos) |
| `sensor/velocity.rs` | ~190 | L8437–L8626 (mj_sensor_vel) |
| `sensor/acceleration.rs` | ~188 | L8638–L8825 (mj_sensor_acc) |
| `sensor/postprocess.rs` | ~48 | L8833–L8895 (sensor_write helpers + mj_sensor_postprocess) |
| `sensor/derived.rs` | ~304 | L8897–L9229 (subtree_com, subtree_momentum, body_acceleration, body_angular_acceleration, site_force_torque, is_body_in_subtree, subtree_angmom) — energy at L8028–L8116 is separate, see `energy.rs` |
| `tendon/*.rs` | ~1,100 total | tendon/mod.rs + fixed.rs: L9367–L9429 (~63) + L4004–L4032 (~28, compute_spatial_tendon_length0); tendon/spatial.rs: L9430–L9829 (~400) + L10042–L10113 (~72); tendon/wrap_math.rs: L10114–L10674 (~561) |
| `island/*.rs` | ~1,324 total | island/sleep.rs: L12966–L13409 (444) + L14005–L14268 (264) + L4491–L4519 (29, Data sleep methods) = ~737; island/mod.rs: L13411–L13997 = ~587 |
| `integrate/mod.rs` | ~120 | L4701–L4821 (integrate() dispatch, integrate_without_velocity() — already mapped above) |
| `integrate/euler.rs` | ~166 | L20900–L21066 (mj_integrate_pos, PositionIntegrateVisitor, mj_normalize_quat, QuaternionNormalizeVisitor) |
| `integrate/implicit.rs` | ~127 | L12690–L12816 (tendon K/D helpers only: tendon_all_dofs_sleeping, tendon_all_dofs_sleeping_fields, tendon_deadband_displacement, tendon_active_stiffness, accumulate_tendon_kd) |
| `integrate/rk4.rs` | ~169 | L21302–L21470 (mj_runge_kutta) |
| `jacobian.rs` | ~465 | L9830–L10041 + L10795–L10829 + L21085–L21283 (Jacobian utilities + compute_contact_normal_jacobian + mj_differentiate_pos + mj_integrate_pos_explicit; doc comment at L21068–L21084) |
| `linalg.rs` | ~340 | L20106–L20253 (Cholesky) + L20445–L20557 (mj_solve_sparse, mj_solve_sparse_batch) + L20829–L20897 (LU). Decision: sparse solvers belong in `linalg.rs` (they are solve routines, not factorization); factorization stays in `dynamics/factor.rs`. |
| `joint_visitor.rs` | ~130 | L384–L454 (JointContext + JointVisitor trait) + L11599–L11669 (joint_motion_subspace + boundary lines from CRBA) |
| `energy.rs` | ~94 | L8028–L8116 + L4485–L4489 (Data::total_energy) |
| **Inline tests** | ~5,246 | L21476–L26722 (move with their modules) |

**Production code**: ~21,475 lines → distributed across ~61 modules
**Largest module**: `constraint/assembly.rs` at ~750 lines
**Average module**: ~420 lines (ideal for comprehension)
**All modules**: ≤800 lines of production code (verified for constraint block)

### Doc Reference Mapping Table

When updating `future_work_*.md` references that cite `mujoco_pipeline.rs` by line
number (e.g., `mujoco_pipeline.rs:L15334`), use this table to determine the target module.
Line ranges are pre-refactor snapshot positions (before commit `d018c7f`) and
shift as functions are extracted. Use function names to locate code.

| Line range (approx) | Target module | Contents |
|---------------------|---------------|----------|
| L1–L105 | (preamble) | File header, `use` imports, module-level items |
| L106–L325 | `dynamics/spatial.rs` | Spatial algebra (spatial_cross_motion, spatial_cross_force, etc.) |
| L325–L383 | `types/enums.rs` | Enums part 1 (MjJointType through end of preceding items — L384 starts JointContext's doc comment) |
| L384–L454 | `joint_visitor.rs` | JointContext (doc comment at L384, #[derive] at L388) + JointVisitor trait |
| L455–L1100 | `types/enums.rs` | Enums part 2 (GeomType, SolverType, Integrator, ... ResetError) |
| L1105–L1124 | `types/keyframe.rs` | Keyframe struct |
| L1125–L1141 | (whitespace/derives) | Model doc comment + `#[derive]` — precedes Model struct |
| L1142–L1870 | `types/model.rs` | Model struct + accessors |
| L1870–L2171 | `types/contact_types.rs` | ContactPair, Contact struct, impl Contact, compute_tangent_frame |
| L2172–L2184 | (whitespace/comments) | Section boundary — no production code |
| L2185–L2890 | `types/data.rs` | Data struct + Clone impl |
| L2890–L3646 | `types/model_init.rs` | empty() (L2893), make_data() (L3143–L3646) |
| L3648–L3659 | `types/model.rs` | `joint_qpos0()` (doc comment at L3648, fn at L3652) |
| L3661–L3696 | `types/model_init.rs` | `compute_ancestors()` (doc comment at L3661, fn at L3671) |
| L3697–L3704 | (whitespace/comments) | Section boundary — no production code |
| L3705–L3744 | `types/model_init.rs` | `compute_implicit_params()` (doc comment at L3698, fn at L3705) |
| L3745–L4003 | `forward/muscle.rs` | `compute_muscle_params()` — stays in monolith until Phase 8a |
| L4004–L4032 | `tendon/mod.rs` | `compute_spatial_tendon_length0()` — stays in monolith until Phase 5 |
| L4033–L4062 | `types/model_init.rs` | `compute_stat_meaninertia()` |
| L4063–L4076 | `types/model.rs` | `is_ancestor()` |
| L4078–L4383 | `types/model_factories.rs` | Factory methods (n_link_pendulum, double_pendulum, spherical_pendulum, free_body) |
| L4385–L4483 | `types/data.rs` | Data accessors (qld_diag (L4391), reset (L4397), reset_to_keyframe (L4444–L4483)) |
| L4485–L4489 | `energy.rs` | `Data::total_energy()` (L4487) — moves in Phase 4 with energy.rs |
| L4491–L4519 | `island/sleep.rs` | `Data::sleep_state()` (L4499), `tree_awake()` (L4505), `nbody_awake()` (L4511), `nisland()` (L4517) — moves in Phase 8c |
| L4521–L4700 | `forward/mod.rs` | step (doc comment at L4521, fn at L4537), forward, forward_skip_sensors, forward_core |
| L4701–L4821 | `integrate/mod.rs` | integrate(), integrate_without_velocity() |
| L4822–L4876 | `forward/check.rs` | mj_check_pos, mj_check_vel, mj_check_acc (~24 lines + whitespace) |
| L4876–L5383 | `forward/position.rs` | mj_fwd_position + AABB + SAP |
| L5383–L5639 | `collision/mod.rs` | check_collision_affinity (L5383), mj_collision (L5454), mj_collision_flex (L5584) |
| L5640–L5842 | `collision/flex_collide.rs` | narrowphase_sphere_geom (L5640), make_contact_flex_rigid (L5797) |
| L5844–L6068 | `collision/narrow.rs` | Narrow-phase dispatch + geom_to_collision_shape |
| L6068–L6240 | `collision/mod.rs` | Contact parameter mixing (contact_param (L6068), contact_param_flex_rigid (L6129), solmix_weight (L6188), combine_solref (L6206), combine_solimp (L6218)) |
| L6241–L6265 | `collision/narrow.rs` | End of make_contact_from_geoms |
| L6267–L6288 | `collision/narrow.rs` | Constants (GEOM_EPSILON, etc.) |
| L6290–L6376 | `collision/hfield.rs` | collide_with_hfield |
| L6378–L6496 | `collision/sdf_collide.rs` | collide_with_sdf |
| L6498–L6671 | `collision/mesh_collide.rs` | collide_with_mesh + collide_mesh_plane |
| L6673–L7088 | `collision/plane.rs` | collide_with_plane + collide_cylinder_plane_impl + collide_ellipsoid_plane_impl |
| L7095–L7406 | `collision/pair_convex.rs` | Sphere/capsule/box pairs |
| L7406–L8028 | `collision/pair_cylinder.rs` | Cylinder pairs + box-box SAT |
| L8028–L8116 | `energy.rs` | mj_energy_pos, mj_energy_vel |
| L8118–L8134 | `sensor/mod.rs` | Sensor pipeline section header, dispatch (mod declarations + re-exports) |
| L8135–L8423 | `sensor/position.rs` | mj_sensor_pos (289 lines) |
| L8437–L8626 | `sensor/velocity.rs` | mj_sensor_vel (190 lines) |
| L8638–L8825 | `sensor/acceleration.rs` | mj_sensor_acc (188 lines) |
| L8833–L8895 | `sensor/postprocess.rs` | sensor_write (6), sensor_write3 (5), sensor_write4 (6), mj_sensor_postprocess (31) |
| L8897–L9229 | `sensor/derived.rs` | compute_subtree_com (30), compute_subtree_momentum (27), compute_body_acceleration (59), compute_body_angular_acceleration (38), compute_site_force_torque (96), is_body_in_subtree (12), compute_subtree_angmom (42) |
| L9236–L9244 | `dynamics/flex.rs` | mj_flex (flex vertex sync, ~6 lines) |
| L9244–L9358 | `forward/velocity.rs` | Body spatial velocities from qvel |
| L9358–L9367 | (whitespace/comments) | Section boundary between velocity and tendon — no production code |
| L9367–L9429 | `tendon/mod.rs` + `tendon/fixed.rs` | mj_fwd_tendon (L9367), mj_fwd_tendon_fixed (L9387) |
| L9430–L9829 | `tendon/spatial.rs` | mj_fwd_tendon_spatial (L9430), accumulate_point_jacobian (L9736) |
| L9830–L10041 | `jacobian.rs` | Jacobian functions (mj_jac, mj_jac_site, mj_jac_body, etc.) |
| L10042–L10113 | `tendon/spatial.rs` | apply_tendon_force (L10042), subquat (L10073), WrapResult enum (L10095) |
| L10114–L10674 | `tendon/wrap_math.rs` | segments_intersect_2d (L10114), directional_wrap_angle (L10131), sphere_tangent_point (L10156), compute_tangent_pair (L10170), circle_tangent_2d (L10187), sphere_wrapping_plane (L10219), wrap_inside_2d (L10246), sphere_wrap (L10369), cylinder_wrap (L10509) |
| L10675–L10681 | (whitespace/comments) | Section boundary between tendon wrapping and actuation |
| L10795–L10829 | `jacobian.rs` | compute_contact_normal_jacobian (~35 lines) — private, called only by `mj_transmission_body`. Despite sitting in the actuation section of the monolith, logically belongs with Jacobian utilities. |
| L10682–L11247 | `forward/actuation.rs` | Transmission + actuation (excluding L10795–L10829 which goes to jacobian.rs; muscle_* helpers referenced in this range call compute_muscle_params from `forward/muscle.rs`) |
| L11247–L11598 | `dynamics/crba.rs` | CRBA + mass matrix |
| L11599–L11608 | `joint_visitor.rs` | (end of CRBA range / start of joint_motion_subspace — boundary overlap resolved) |
| L11609–L11669 | `joint_visitor.rs` | joint_motion_subspace (motion subspace computation) |
| L11670–L11676 | (whitespace/comments) | Section boundary between joint_motion_subspace and mj_rne |
| L11677–L12038 | `dynamics/rne.rs` | RNE + gravity comp (doc comment starts at L11677, fn mj_rne at L11704) |
| L12038–L12108 | `dynamics/spatial.rs` | Additional spatial algebra |
| L12108–L12401 | `forward/passive.rs` | Fluid model helpers (mj_inertia_box_fluid, mj_ellipsoid_fluid, mj_fluid dispatch) + mj_fwd_passive doc comment |
| L12402–L12690 | `forward/passive.rs` | mj_fwd_passive (L12402) — spring/damper/frictionloss/flex bending |
| L12690–L12816 | `integrate/implicit.rs` | **WARNING: These lines sit between the two `forward/passive.rs` ranges (L12108–L12690 and L12818–L12899) but belong to `integrate/implicit.rs`, NOT to `forward/passive.rs`.** Tendon implicit K/D helpers: tendon_all_dofs_sleeping (L12690), tendon_all_dofs_sleeping_fields (L12700), tendon_deadband_displacement (L12713), tendon_active_stiffness (L12743), accumulate_tendon_kd (L12755). Used by acceleration and constraint code. `tendon_all_dofs_sleeping` also imported by `island/sleep.rs` as `pub(crate)`. |
| L12818–L12899 | `forward/passive.rs` | PassiveForceVisitor struct + impl JointVisitor (joint visitor for passive spring/damper forces) |
| L12901–L12964 | `types/model_init.rs` | compute_body_lengths (L12901), compute_dof_lengths (L12937) — model construction helpers for §16.14 mechanism lengths |
| L12966–L13409 | `island/sleep.rs` | mj_sleep (L12966), tree_can_sleep (L13035), sleep_trees (L13082), sync_tree_fk (L13130), reset_sleep_state (L13197), mj_update_sleep_arrays (L13297), mj_check_qpos_changed (L13386) |
| L13411–L13997 | `island/mod.rs` | mj_flood_fill (L13411), mj_island (L13461), equality_trees (L13774), constraint_tree (L13883) |
| L14005–L14268 | `island/sleep.rs` | mj_wake (L14005), mj_wake_collision (L14038), mj_sleep_cycle (L14073), mj_wake_tendon (L14092), mj_wake_equality (L14153), mj_wake_tree (L14201), sensor_body_id (L14239) |
| L14269–L14534 | `constraint/jacobian.rs` | compute_flex_contact_jacobian (L14282), compute_contact_jacobian (L14410) — contact Jacobian construction for constraint assembly |
| L14536–L14606 | `constraint/jacobian.rs` | add_angular_jacobian (L14541) — angular Jacobian helper for torsional/rolling constraints |
| L14607–L14660 | `constraint/solver/noslip.rs` | project_elliptic_cone (L14607) — elliptic cone projection for noslip |
| L14661–L14883 | `constraint/solver/noslip.rs` | noslip_qcqp2 (L14661), noslip_qcqp3 (L14757) — QCQP solvers for noslip friction |
| L14884–L14931 | `constraint/solver/mod.rs` | decode_pyramid (L14884) — public API for pyramid → force conversion |
| L14932–L15043 | `constraint/impedance.rs` | compute_impedance (L14932) — constraint impedance model |
| L15044–L15082 | `constraint/impedance.rs` | quaternion_to_axis_angle (L15044) — used in weld/ball constraint assembly |
| L15083–L15118 | `constraint/impedance.rs` | compute_kbip (L15083), compute_aref (L15116) — constraint stiffness/damping/reference |
| L15119–L15122 | (whitespace) | Section boundary — no production code |
| L15123–L15164 | `constraint/impedance.rs` | normalize_quat4 (L15123), ball_limit_axis_angle (L15143) — quaternion helpers for ball joint limits |
| L15165–L15171 | (whitespace) | Section boundary — no production code |
| L15172–L15274 | `constraint/impedance.rs` | compute_diag_approx_exact (L15172), mj_solve_sparse_vec (L15208), compute_regularization (L15255) — diagonal approximation and regularization |
| L15275–L15315 | `constraint/mod.rs` | compute_qacc_smooth (~41 lines) |
| L15334–L16018 | `constraint/assembly.rs` | assemble_unified_constraints (685 lines) |
| L16028–L16062 | `constraint/solver/mod.rs` | compute_delassus_regularized (35 lines) |
| L16074–L16220 | `constraint/solver/pgs.rs` | pgs_solve_unified (147 lines) |
| L16237–L16516 | `constraint/solver/cg.rs` | cg_solve_unified (280 lines) |
| L16523–L16558 | `constraint/solver/pgs.rs` | pgs_cost_change (36 lines) |
| L16561–L16591 | `constraint/solver/mod.rs` | compute_qfrc_constraint_from_efc, extract_qfrc_frictionloss (25 lines) |
| L16608–L16824 | `constraint/solver/pgs.rs` | classify_constraint_states (217 lines) |
| L16837–L17521 | `constraint/solver/hessian.rs` | assemble_hessian (46), SparseHessian struct+impl (527), hessian_incremental (44), hessian_cone (50) |
| L17529–L17602 | `constraint/solver/primal.rs` | compute_gradient_and_search (36), compute_gradient_and_search_sparse (36) |
| L17603–L17654 | `constraint/solver/primal.rs` | PrimalQuad, PrimalPoint struct definitions |
| L17655–L18181 | `constraint/solver/primal.rs` | primal_prepare (101), primal_eval (133), primal_search (155), evaluate_cost_at (109) |
| L18204–L18514 | `constraint/solver/newton.rs` | newton_solve (303), recover_newton (8) |
| L18555–L18972 | `constraint/solver/noslip.rs` | noslip_postprocess (~418 lines) |
| L18998–L19086 | `constraint/equality.rs` | get_min_diagonal_mass (68), get_min_translational_mass (3), get_min_rotational_inertia (3) |
| L19110–L19482 | `constraint/equality.rs` | extract_{connect,weld,joint,tendon,distance}_jacobian |
| L19490–L19609 | `constraint/equality.rs` | add_body_point_jacobian_row (68), add_body_angular_jacobian_row (47) |
| L19624–L19682 | `constraint/assembly.rs` | populate_efc_island (59 lines) |
| L19686–L19691 | `constraint/mod.rs` | mj_fwd_constraint_islands (~6 lines) |
| L19691–L19715 | (whitespace/comments) | Section boundary — no production code |
| L19715–L19742 | `constraint/mod.rs` | build_m_impl_for_newton (~28 lines) |
| L19742–L19759 | (whitespace/comments) | Section boundary — no production code |
| L19759–L19842 | `constraint/mod.rs` | compute_qfrc_smooth_implicit (~84 lines) |
| L19842–L19850 | (whitespace/comments) | Section boundary — no production code |
| L19850–L20006 | `constraint/mod.rs` | mj_fwd_constraint (~157 lines) |
| L20006–L20013 | (whitespace/comments) | Section boundary — no production code |
| L20013–L20028 | `constraint/mod.rs` | compute_point_velocity (~16 lines) |
| L20029–L20083 | `forward/acceleration.rs` | mj_fwd_acceleration (L20066) — dispatch to explicit/implicit/implicitfast/implicit_full (L20084 starts mj_fwd_acceleration_explicit) |
| L20085–L20104 | `forward/acceleration.rs` | mj_fwd_acceleration_explicit |
| L20106–L20253 | `linalg.rs` | Cholesky factorization, solve, rank-1 update/downdate |
| L20255–L20275 | `dynamics/factor.rs` | (doc comment for mj_factor_sparse) |
| L20276–L20444 | `dynamics/factor.rs` | mj_factor_sparse, mj_factor_sparse_selective (sparse LDL factorization) |
| L20445–L20557 | `linalg.rs` | mj_solve_sparse, mj_solve_sparse_batch (sparse triangular solves) |
| L20559–L20827 | `forward/acceleration.rs` | Implicit acceleration paths (3 variants + ImplicitSpringVisitor) |
| L20828 | (whitespace) | Section boundary |
| L20829–L20897 | `linalg.rs` | LU factorization + solve |
| L20898 | (whitespace) | Section boundary |
| L20900–L21066 | `integrate/euler.rs` | mj_integrate_pos (12), PositionIntegrateVisitor (98), mj_normalize_quat (6), QuaternionNormalizeVisitor (44) |
| L21085–L21283 | `jacobian.rs` | mj_differentiate_pos (101), mj_integrate_pos_explicit (81) — primary callers are `derivatives.rs` for finite-difference Jacobians |
| L21302–L21470 | `integrate/rk4.rs` | mj_runge_kutta (169 lines) |
| L21476–L26722 | (inline tests) | Move with their host modules |

For `model_builder.rs` references, map to `builder/<submodule>.rs` per the
sim-mjcf target structure table above.

---

## Target Structure: sim-mjcf

### Current (total lines incl. tests)

```
sim-mjcf/src/
  lib.rs
  model_builder.rs    10,184 lines  ← SECOND PROBLEM
  parser.rs            5,273 lines
  types.rs             3,883 lines
  defaults.rs          1,310 lines
  config.rs
  error.rs
  include.rs
  mjb.rs
  validation.rs
```

### Target

```
sim-mjcf/src/
  lib.rs
  parser.rs                  (unchanged — 3,470 production lines, exempt from S1
                              for this refactor, tracked for future decomposition)
  types.rs                   (unchanged — 3,775 production lines, exempt from S1
                              for this refactor, tracked for future decomposition)
  defaults.rs                (unchanged — 871 production lines, exempt from S1
                              for this refactor, tracked for future decomposition)
  config.rs                  (unchanged)
  error.rs                   (unchanged)
  include.rs                 (unchanged)
  mjb.rs                     (unchanged)
  validation.rs              (unchanged)
  │
  └── builder/
      ├── mod.rs             ModelBuilder struct, load_model(),
      │                      load_model_from_file(), model_from_mjcf(),
      │                      set_options(), resolve_keyframe() — top-level orchestration
      ├── init.rs            ModelBuilder::new() — field initialization (~264 lines,
      │                      split from mod.rs to stay under 800; split-impl-block pattern)
      ├── build.rs           build(self) -> Model — model assembly (~675 lines,
      │                      split from mod.rs via Rust's split-impl-block pattern)
      ├── body.rs            process_body, process_body_with_world_frame,
      │                      process_worldbody_geoms_and_sites
      ├── joint.rs           process_joint
      ├── geom.rs            process_geom, process_site, compute_geom_mass,
      │                      compute_geom_inertia, geom_effective_com, geom_size_to_vec3,
      │                      compute_fromto_pose
      ├── actuator.rs        process_actuator, parse_gaintype, parse_biastype,
      │                      parse_dyntype, floats_to_array
      ├── sensor.rs          process_sensors, resolve_sensor_object,
      │                      convert_sensor_type, sensor_datatype
      ├── tendon.rs          process_tendons
      ├── contact.rs         process_contact, compute_initial_geom_distance,
      │                      geom_world_position
      ├── equality.rs        process_equality_constraints
      ├── flex.rs            process_flex_bodies, compute_flexedge_crosssection,
      │                      compute_flex_address_table, compute_flex_count_table,
      │                      compute_vertex_masses, compute_dihedral_angle,
      │                      compute_edge_solref, compute_bend_*
      ├── mass.rs            apply_mass_pipeline, compute_inertia_from_geoms,
      │                      extract_inertial_properties
      ├── mesh.rs            process_mesh, process_hfield, load_mesh_file,
      │                      convert_mjcf_mesh, convert_embedded_mesh,
      │                      convert_mjcf_hfield, compute_mesh_inertia, resolve_mesh
      ├── frame.rs           expand_frames, expand_single_frame, frame_accum_child,
      │                      validate_childclass_references, validate_frame_childclass_refs
      ├── compiler.rs        apply_discardvisual, apply_fusestatic, fuse_static_body,
      │                      remove_visual_geoms, collect_mesh_refs
      ├── orientation.rs     quat_from_wxyz, quat_to_wxyz, euler_seq_to_quat,
      │                      resolve_orientation
      ├── fluid.rs           compute_geom_fluid, geom_semi_axes, get_added_mass_kappa
      └── asset.rs           resolve_asset_path
```

### Module size estimates

| Module | Est. lines | Source (line ranges in current file) |
|--------|-----------|--------------------------------------|
| `builder/mod.rs` | ~732 | ModelConversionError + Display + Error (L50–L65, ~16), resolve_keyframe (L69–L205, ~137), model_from_mjcf (L206–L317, ~112), load_model (L318–L345, ~28), load_model_from_file (L346–L423, ~78), struct ModelBuilder (L516–L822, ~307), set_options (L1089–L1142, ~54). **Note**: `new()` (L824–L1087, ~264) is split out to `builder/init.rs` (see margin warning below). |
| `builder/init.rs` | ~264 | ModelBuilder::new() (L824–L1087, ~264 lines of field initialization) — split from mod.rs to stay under 800. |
| `builder/build.rs` | ~676 | build(self) -> Model (L3505–L4180, ~676 lines — model assembly; split via Rust's split-impl-block pattern) |
| `builder/body.rs` | ~304 | process_worldbody_geoms_and_sites (L1194–L1232, ~39), process_body (L1233–L1262, ~30), process_body_with_world_frame (L1263–L1497, ~235) |
| `builder/joint.rs` | ~172 | process_joint (L1498–L1669, ~172) |
| `builder/geom.rs` | ~503 | process_geom (L1670–L1868, ~199), process_site (L1869–L1933, ~65), geom_effective_com (L5387–L5405, ~19), compute_geom_mass (L5493–L5545, ~53), compute_geom_inertia (L5546–L5640, ~95), compute_fromto_pose (L5641–L5680, ~40), geom_size_to_vec3 (L5681–L5712, ~32) |
| `builder/actuator.rs` | ~419 | process_actuator (L2114–L2484, ~371), parse_gaintype (L4186–L4196, ~11), parse_biastype (L4197–L4207, ~11), parse_dyntype (L4208–L4224, ~17), floats_to_array (L4225–L4233, ~9) |
| `builder/sensor.rs` | ~300 | process_sensors (L2485–L2538, ~54), resolve_sensor_object (L2672–L2827, ~156), convert_sensor_type (L5713–L5750, ~38), sensor_datatype (L5751–L5802, ~52) |
| `builder/tendon.rs` | ~180 | process_tendons (L1934–L2113, ~180) |
| `builder/contact.rs` | ~160 | process_contact (L2539–L2671, ~133), compute_initial_geom_distance (L3071–L3081, ~11), geom_world_position (L3082–L3097, ~16) |
| `builder/equality.rs` | ~243 | process_equality_constraints (L2828–L3070, ~243) |
| `builder/flex.rs` | ~581 | process_flex_bodies (L3154–L3504, ~351), compute_flexedge_crosssection (L5803–L5853, ~51), compute_flex_address_table (L5854–L5867, ~14), compute_flex_count_table (L5868–L5883, ~16), compute_vertex_masses (L5884–L5957, ~74), compute_dihedral_angle (L5958–L5997, ~40), compute_edge_solref (L5998–L6008, ~11), compute_bend_stiffness_from_material (L6009–L6025, ~17), compute_bend_damping_from_material (L6026–L6032, ~7) |
| `builder/mass.rs` | ~198 | apply_mass_pipeline (L3098–L3153, ~56), extract_inertial_properties (L5081–L5135, ~55), compute_inertia_from_geoms (L5406–L5492, ~87) |
| `builder/mesh.rs` | ~439 | process_mesh (L1143–L1167, ~25), process_hfield (L1168–L1193, ~26), load_mesh_file (L4719–L4780, ~62), convert_mjcf_hfield (L4781–L4842, ~62), convert_mjcf_mesh (L4843–L4873, ~31), convert_embedded_mesh (L4874–L4960, ~87), compute_mesh_inertia (L4961–L5080, ~120), resolve_mesh (L5361–L5386, ~26) |
| `builder/frame.rs` | ~242 | frame_accum_child (L4367–L4381, ~15), validate_childclass_references (L4382–L4405, ~24), validate_frame_childclass_refs (L4406–L4434, ~29), expand_frames (L4435–L4464, ~30), expand_single_frame (L4465–L4608, ~144) |
| `builder/compiler.rs` | ~216 | apply_discardvisual + nested remove_visual_geoms + collect_mesh_refs (L5136–L5183, ~48), apply_fusestatic (L5184–L5245, ~62), fuse_static_body (L5255–L5360, ~106) |
| `builder/orientation.rs` | ~142 | quat_from_wxyz (L4234–L4248, ~15), euler_seq_to_quat (L4249–L4275, ~27), resolve_orientation (L4276–L4366, ~91), quat_to_wxyz (L5246–L5254, ~9) |
| `builder/fluid.rs` | ~91 | get_added_mass_kappa (L424–L441, ~18), geom_semi_axes (L442–L454, ~13), compute_geom_fluid (L455–L514, ~60) |
| `builder/asset.rs` | ~110 | AssetKind enum (L4609–L4632, ~24), resolve_asset_path (L4633–L4718, ~86) |
| **Inline tests** | ~4,152 | L6033–L10184 (move with their modules) |

**Production code**: ~6,032 lines → distributed across 19 modules
**Largest module**: `builder/mod.rs` at ~732 lines (after splitting `new()` to `init.rs`)
**All modules**: ≤800 lines of production code

> **MARGIN WARNING — `builder/mod.rs`**: Without the `new()` split, mod.rs
> would be ~996 lines (struct definition 307 + new 264 + set_options 54 +
> orchestration 371). The split moves `new()` to `builder/init.rs` (~264
> lines), reducing mod.rs to ~732. This is the recommended approach. If the
> split feels like overhead, an alternative fallback is to move the struct
> definition (307 lines) + new() (264) + set_options() (54) = 625 lines to
> `builder/types.rs`, leaving mod.rs as pure orchestration (~371 lines).

> Module body estimates sum to ~5,972. The file has ~6,032 production lines.
> The ~60-line gap is the file preamble (`use` imports, ~49 lines) plus
> minor inter-function whitespace (~11 lines).
> Verify actual sizes during extraction — estimates are approximate.

### Doc Reference Mapping Table

When updating `future_work_*.md` references that cite `model_builder.rs` by line
number (e.g., `model_builder.rs:L2114`), use this table to determine the target module.
Line ranges are snapshot positions from the pre-refactor source and shift as
functions are extracted. Use function names to locate code.

| Line range (approx) | Target module | Contents |
|---------------------|---------------|----------|
| L1–L49 | (preamble) | File header, `use` imports |
| L50–L65 | `builder/mod.rs` | ModelConversionError struct + Display + Error impls |
| L66–L68 | (whitespace) | |
| L69–L205 | `builder/mod.rs` | resolve_keyframe() (~137 lines) |
| L206–L317 | `builder/mod.rs` | model_from_mjcf() (~112 lines) |
| L318–L345 | `builder/mod.rs` | load_model() (~28 lines) |
| L346–L423 | `builder/mod.rs` | load_model_from_file() (~78 lines) |
| L424–L441 | `builder/fluid.rs` | get_added_mass_kappa() (~18 lines) |
| L442–L454 | `builder/fluid.rs` | geom_semi_axes() (~13 lines) |
| L455–L514 | `builder/fluid.rs` | compute_geom_fluid() (~60 lines) |
| L515 | (whitespace) | |
| L516–L822 | `builder/mod.rs` | struct ModelBuilder (~307 lines of fields) |
| L823 | (whitespace) | |
| L824–L1087 | `builder/init.rs` | ModelBuilder::new() (~264 lines of field init) |
| L1089–L1142 | `builder/mod.rs` | set_options() (~54 lines) |
| L1143–L1167 | `builder/mesh.rs` | process_mesh() (~25 lines) |
| L1168–L1193 | `builder/mesh.rs` | process_hfield() (~26 lines) |
| L1194–L1232 | `builder/body.rs` | process_worldbody_geoms_and_sites() (~39 lines) |
| L1233–L1262 | `builder/body.rs` | process_body() (~30 lines) |
| L1263–L1497 | `builder/body.rs` | process_body_with_world_frame() (~235 lines) |
| L1498–L1669 | `builder/joint.rs` | process_joint() (~172 lines) |
| L1670–L1868 | `builder/geom.rs` | process_geom() (~199 lines) |
| L1869–L1933 | `builder/geom.rs` | process_site() (~65 lines) |
| L1934–L2113 | `builder/tendon.rs` | process_tendons() (~180 lines) |
| L2114–L2484 | `builder/actuator.rs` | process_actuator() (~371 lines) |
| L2485–L2538 | `builder/sensor.rs` | process_sensors() (~54 lines) |
| L2539–L2671 | `builder/contact.rs` | process_contact() (~133 lines) |
| L2672–L2827 | `builder/sensor.rs` | resolve_sensor_object() (~156 lines) |
| L2828–L3070 | `builder/equality.rs` | process_equality_constraints() (~243 lines) |
| L3071–L3081 | `builder/contact.rs` | compute_initial_geom_distance() (~11 lines) |
| L3082–L3097 | `builder/contact.rs` | geom_world_position() (~16 lines) |
| L3098–L3153 | `builder/mass.rs` | apply_mass_pipeline() (~56 lines) |
| L3154–L3504 | `builder/flex.rs` | process_flex_bodies() (~351 lines) |
| L3505–L4180 | `builder/build.rs` | build(self) -> Model (~676 lines — model assembly) |
| L4181–L4185 | (whitespace/comments) | Section boundary |
| L4186–L4196 | `builder/actuator.rs` | parse_gaintype() (~11 lines) |
| L4197–L4207 | `builder/actuator.rs` | parse_biastype() (~11 lines) |
| L4208–L4224 | `builder/actuator.rs` | parse_dyntype() (~17 lines) |
| L4225–L4233 | `builder/actuator.rs` | floats_to_array() (~9 lines) |
| L4234–L4248 | `builder/orientation.rs` | quat_from_wxyz() (~15 lines) |
| L4249–L4275 | `builder/orientation.rs` | euler_seq_to_quat() (~27 lines) |
| L4276–L4366 | `builder/orientation.rs` | resolve_orientation() (~91 lines) |
| L4367–L4381 | `builder/frame.rs` | frame_accum_child() (~15 lines) |
| L4382–L4405 | `builder/frame.rs` | validate_childclass_references() (~24 lines) |
| L4406–L4434 | `builder/frame.rs` | validate_frame_childclass_refs() (~29 lines) |
| L4435–L4464 | `builder/frame.rs` | expand_frames() (~30 lines) |
| L4465–L4608 | `builder/frame.rs` | expand_single_frame() (~144 lines) |
| L4609–L4632 | `builder/asset.rs` | AssetKind enum (~24 lines) |
| L4633–L4718 | `builder/asset.rs` | resolve_asset_path() (~86 lines) |
| L4719–L4780 | `builder/mesh.rs` | load_mesh_file() (~62 lines) |
| L4781–L4842 | `builder/mesh.rs` | convert_mjcf_hfield() (~62 lines) |
| L4843–L4873 | `builder/mesh.rs` | convert_mjcf_mesh() (~31 lines) |
| L4874–L4960 | `builder/mesh.rs` | convert_embedded_mesh() (~87 lines) |
| L4961–L5080 | `builder/mesh.rs` | compute_mesh_inertia() (~120 lines) |
| L5081–L5135 | `builder/mass.rs` | extract_inertial_properties() (~55 lines) |
| L5136–L5183 | `builder/compiler.rs` | apply_discardvisual() + nested remove_visual_geoms + collect_mesh_refs (~48 lines) |
| L5184–L5245 | `builder/compiler.rs` | apply_fusestatic() (~62 lines) |
| L5246–L5254 | `builder/orientation.rs` | quat_to_wxyz() (~9 lines) |
| L5255–L5360 | `builder/compiler.rs` | fuse_static_body() (~106 lines) |
| L5361–L5386 | `builder/mesh.rs` | resolve_mesh() (~26 lines) |
| L5387–L5405 | `builder/geom.rs` | geom_effective_com() (~19 lines) |
| L5406–L5492 | `builder/mass.rs` | compute_inertia_from_geoms() (~87 lines) |
| L5493–L5545 | `builder/geom.rs` | compute_geom_mass() (~53 lines) |
| L5546–L5640 | `builder/geom.rs` | compute_geom_inertia() (~95 lines) |
| L5641–L5680 | `builder/geom.rs` | compute_fromto_pose() (~40 lines) |
| L5681–L5712 | `builder/geom.rs` | geom_size_to_vec3() (~32 lines) |
| L5713–L5750 | `builder/sensor.rs` | convert_sensor_type() (~38 lines) |
| L5751–L5802 | `builder/sensor.rs` | sensor_datatype() (~52 lines) |
| L5803–L5853 | `builder/flex.rs` | compute_flexedge_crosssection() (~51 lines) |
| L5854–L5867 | `builder/flex.rs` | compute_flex_address_table() (~14 lines) |
| L5868–L5883 | `builder/flex.rs` | compute_flex_count_table() (~16 lines) |
| L5884–L5957 | `builder/flex.rs` | compute_vertex_masses() (~74 lines) |
| L5958–L5997 | `builder/flex.rs` | compute_dihedral_angle() (~40 lines) |
| L5998–L6008 | `builder/flex.rs` | compute_edge_solref() (~11 lines) |
| L6009–L6025 | `builder/flex.rs` | compute_bend_stiffness_from_material() (~17 lines) |
| L6026–L6032 | `builder/flex.rs` | compute_bend_damping_from_material() (~7 lines) |
| L6033–L10184 | (inline tests) | ~4,152 lines — move with their host modules |

---

## Standalone Crate Decision

Four standalone crates have functionality that's been re-implemented inside the
pipeline: `sim-constraint`, `sim-tendon`, `sim-muscle`, `sim-sensor`. The
current state (duplicated logic in two places) is the worst option.

### Decision: Keep as reference libraries, document clearly, do NOT try to unify now

**Rationale:**

1. **Unification is a physics change, not a structural refactor.** Making the
   pipeline call into `sim-tendon` instead of using its own tendon code would
   change the call graph, potentially the numerics, and require extensive
   re-validation. That's a separate project.

2. **The standalone crates have independent value.** `sim-constraint`'s CG solver,
   `sim-tendon`'s spatial routing, and `sim-muscle`'s Hill model are usable
   outside the MuJoCo pipeline. They're libraries in their own right.

3. **Post-v1.0 item #44 already tracks deprecation.** The roadmap has this. We
   don't need to solve it during the structural refactor.

4. **Clear documentation is enough for now.** Each standalone crate's README
   should say: "This is a standalone reference library. The MuJoCo pipeline in
   sim-core has its own implementation. See ARCHITECTURE.md for details."

This decision should be revisited post-v1.0 when the trait architecture lands
and there's a clean way to compose standalone implementations into the pipeline.

---

## Execution Plan

### Every phase includes (non-negotiable):

Every extraction phase (1–10) ends with these steps. They are not optional.

1. **Update all `use` imports** in moved code AND in all existing files that
   import moved symbols to point to new module paths (not through the
   re-export shim — point to the real location). Key files to check each
   phase: `derivatives.rs` (15 cross-module imports), `batch.rs` (3 imports),
   `lib.rs` (re-exports).
2. **Update all code comments** that reference `mujoco_pipeline.rs` or
   `model_builder.rs` by filename to reference the new module.
3. **Update doc references** in `sim/docs/` for any functions that moved
   in this phase.
4. **Add `//!` module doc** to every new `.rs` file created in this phase.
5. **Run rubric S6 stale-reference check** (grep for old filenames).
5a. **Verify no lazy imports.** After adding monolith re-imports, temporarily
   comment them out and run `cargo check -p sim-core`. Any errors in files
   OTHER than the monolith indicate an import that should have been updated
   to point to the new module. Fix those imports, then uncomment the
   re-imports. (This is a manual check — can't be automated in the
   verification script because it requires temporary code changes.)
6. **Run full test suite** — must match baseline.
7. **Run `cargo clippy -- -D warnings`** — zero warnings.
8. **Preserve `#[inline]` attributes.** When moving a function that has
   `#[inline]`, `#[inline(always)]`, or `#[inline(never)]`, keep the
   attribute. Cross-module calls may not be inlined without it (LTO
   notwithstanding). Do not add new `#[inline]` attributes — only preserve
   existing ones.
9. **Grade against rubric S1–S8** for all modules touched in this phase.
10. **Compile incrementally.** Move one function (or one logical group) at a time.
   Run `cargo check -p sim-core` after each move. Do not batch-move an entire
   sub-module's worth of functions and hope it compiles at the end. Commit after
   each sub-module is complete (e.g., commit after `constraint/assembly.rs` is done,
   before starting `constraint/equality.rs`).

### Phase 0: Preparation

- [x] **Verify test baseline.** Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics` and record the pass count. This is the
      invariant: every phase must produce the same pass count.
      (Note: Other sim crates like `sim-constraint`, `sim-tendon`, `sim-muscle`,
      `sim-sensor`, `sim-urdf`, `sim-types`, `sim-simd` are not included because
      they don't depend on `sim-core` internal module paths — only on its public
      API, which doesn't change. If in doubt, run the full CLAUDE.md sim test
      command as a final sanity check.)

      **Pre-refactor baseline snapshot (2026-02-23):**
      | Crate | Passed | Failed | Ignored |
      |-------|--------|--------|---------|
      | `sim-core` (unit) | 402 | 0 | 0 |
      | `sim-core` (doc, runner 1) | 2 | 0 | 9 |
      | `sim-core` (doc, runner 2) | 0 | 0 | 2 |
      | `sim-mjcf` (unit) | 281 | 0 | 0 |
      | `sim-mjcf` (doc) | 1 | 0 | 3 |
      | `sim-conformance-tests` | 831 | 0 | 1 |
      | `sim-physics` (unit) | 6 | 0 | 0 |
      | `sim-physics` (doc) | 3 | 0 | 0 |
      | **Total** | **1,526** | **0** | **15** |

      **sim-urdf baseline (for Phase 10):**
      | Crate | Passed | Failed | Ignored |
      |-------|--------|--------|---------|
      | `sim-urdf` (unit) | 32 | 0 | 0 |
      | `sim-urdf` (doc) | 2 | 0 | 1 |
      | **Total** | **34** | **0** | **1** |
- [x] **Audit test helpers.** Grep the monolith's `#[cfg(test)]` section
      (L21476–L26722) for shared helper functions (non-`#[test]` `fn`
      definitions). Categorize each as:
      - **Local**: used by tests of one domain → moves with those tests
      - **Shared**: used by tests across domains → moves to a
        `#[cfg(test)] pub(crate) mod test_helpers` in the most relevant
        parent module, or to `types/model_factories.rs` if it constructs
        test models
      Record the categorization before beginning Phase 1.

      **Test helper audit results (Session S1, corrected S7):**
      All 15 helpers are **Local** — no shared helpers exist.
      (Original audit found 13; 2 were added by DT-74/DT-75 before Phase 0
      but missed in the initial scan. Corrected here.)

      | Helper | Line | Test module | Moves with |
      |--------|------|-------------|------------|
      | `make_collision_test_model` | 21483 | primitive_collision_tests | collision tests |
      | `make_sensor_test_model` | 22180 | sensor_tests | sensor tests |
      | `add_sensor` | 22270 | sensor_tests | sensor tests |
      | `make_ball_joint_model` | 23210 | sensor_tests | sensor tests |
      | `random_spd` | 23509 | cholesky_tests | linalg tests |
      | `setup_sparse` | 23585 | sparse_factorization_tests | linalg tests |
      | `assert_solve_matches` | 23608 | sparse_factorization_tests | linalg tests |
      | `build_muscle_model_joint` | 23931 | muscle_tests | forward/muscle tests |
      | `quat_from_axis_angle_deg` | 24580 | ball_limit_tests | constraint tests |
      | `make_single_joint_site_model` | 24683 | jac_site_tests | jacobian tests |
      | `make_single_joint_model` | 24843 | mj_jac_tests | jacobian tests |
      | `make_free_hinge_hinge_chain` | 25298 | mj_jac_tests | jacobian tests |
      | `make_free_body_contact_model` | 25707 | contact_jac_free_joint_tests | constraint tests |
      | `make_two_free_body_contact_model` | 25789 | contact_jac_free_joint_tests | constraint tests |
      | `make_two_geom_model` | 26217 | contact_param_tests | collision tests |
- [x] **Create a branch:** `refactor/structural-decompose`

### Phase 1: Extract types from `mujoco_pipeline.rs`

The safest first move. Types have no function bodies to worry about — just
struct/enum definitions.

- [ ] Create `src/types/mod.rs`, `enums.rs`, `model.rs`, `model_init.rs`,
      `model_factories.rs`, `data.rs`, `contact_types.rs`, `keyframe.rs`
- [ ] Move all enums (MjJointType, GeomType, SolverType, etc.) → `types/enums.rs`
- [ ] Move Model struct definition + field accessors + `is_ancestor()` +
      `joint_qpos0()` + `qld_csr()` → `types/model.rs` (~780 lines)
- [ ] Move `empty()`, `make_data()`, `compute_ancestors()`,
      `compute_implicit_params()`, `compute_stat_meaninertia()`,
      `compute_body_lengths()`, `compute_dof_lengths()` →
      `types/model_init.rs` (~774 lines — construction + precomputation).
      Note: compute_body_lengths + compute_dof_lengths are at L12901–L12964,
      physically distant from the other model_init functions.
- [ ] Move `n_link_pendulum()`, `double_pendulum()`, `spherical_pendulum()`,
      `free_body()` → `types/model_factories.rs` (~280 lines, NOT `#[cfg(test)]` — used by external test crates)
- [ ] Methods destined for other modules (`visit_joints` → `joint_visitor.rs`,
      `compute_qld_csr_metadata` → `dynamics/factor.rs`,
      `compute_muscle_params` → `forward/muscle.rs`,
      `compute_spatial_tendon_length0` → `tendon/mod.rs`) stay in the monolith
      until their target module is created in a later phase.
- [ ] Move Data struct + Clone impl + accessors → `types/data.rs`, plus only
      the `impl Data` methods that belong there: `reset()`, `reset_to_keyframe()`,
      `qld_diag()`, and field accessors. Pipeline methods (`step`, `forward`,
      `integrate`, etc.) stay in the monolith until Phase 8.
- [ ] Move Contact/ContactPair + compute_tangent_frame → `types/contact_types.rs`
- [ ] Move Keyframe → `types/keyframe.rs`
- [ ] Update `lib.rs` re-exports (same public API)
- [ ] Run full test suite — must match baseline

**Estimated size**: ~3,500 lines moved

### Phase 2: Extract linear algebra + spatial algebra

Pure math functions with no pipeline state dependencies.

- [ ] Create `src/linalg.rs` — move Cholesky, LU, sparse solve, UnionFind
- [ ] Create `src/dynamics/mod.rs` + `src/dynamics/spatial.rs` — move
      spatial_cross_motion/force, compute_body_spatial_inertia,
      shift_spatial_inertia (remaining dynamics/ modules filled in Phase 7)
- [ ] Create `src/joint_visitor.rs` — move JointVisitor trait, JointContext,
      joint_motion_subspace, Model::visit_joints()
- [ ] Run full test suite

**Estimated size**: ~900 lines moved

### Phase 3: Extract collision into module tree

The collision code is already partially separate (gjk_epa.rs, mesh.rs, etc.)
but the dispatch and primitives live in the monolith.

- [ ] Create `src/collision/` module tree
- [ ] Move mj_collision, collision filters → `collision/mod.rs`
- [ ] Move collide_geoms dispatch, geom_to_collision_shape → `collision/narrow.rs`
- [ ] Move pairwise primitives → `collision/pair_convex.rs` + `collision/pair_cylinder.rs`
- [ ] Move plane collisions → `collision/plane.rs`
- [ ] Move mesh collision dispatch → `collision/mesh_collide.rs`
- [ ] Move hfield collision → `collision/hfield.rs`
- [ ] Move sdf collision → `collision/sdf_collide.rs`
- [ ] Move flex collision → `collision/flex_collide.rs`
- [ ] Move contact parameter mixing (contact_param, solmix, combine_*) →
      `collision/mod.rs` (or `collision/params.rs`)
- [ ] Run full test suite

**Estimated size**: ~2,700 lines moved (module sizes sum to ~2,664; overhead for
imports/docs/whitespace in new files ~36)

### Phase 4: Extract sensors + energy queries

Self-contained: reads from Model/Data, writes to sensordata.
Energy is not a sensor but is extracted here because it's physically adjacent
in the monolith (L8028–L8116) and has no dependencies on later pipeline stages.

- [x] Create `src/sensor/` module tree
- [x] Move mj_sensor_pos → `sensor/position.rs`
- [x] Move mj_sensor_vel → `sensor/velocity.rs`
- [x] Move mj_sensor_acc → `sensor/acceleration.rs`
- [x] Move mj_sensor_postprocess + sensor_write helpers → `sensor/postprocess.rs`
- [x] Move subtree_com, subtree_momentum, body_acceleration, etc. → `sensor/derived.rs`
- [x] Move mj_energy_pos, mj_energy_vel, Data::total_energy() → `src/energy.rs`
- [x] Run full test suite — 2,007 passed / 0 failed / 20 ignored

**Estimated size**: ~1,200 lines moved (sensor modules ~1,036 + energy.rs ~94 +
inter-function whitespace/doc comments ~70)

### Phase 5: Extract tendon pipeline

- [x] Create `src/tendon/` module tree
- [x] Move mj_fwd_tendon, compute_spatial_tendon_length0 → `tendon/mod.rs`
- [x] Move mj_fwd_tendon_fixed → `tendon/fixed.rs`
- [x] Move mj_fwd_tendon_spatial + accumulate_point_jacobian + apply_tendon_force →
      `tendon/spatial.rs`
- [x] Move wrapping math (sphere_tangent_point, wrap_inside_2d, etc.) →
      `tendon/wrap_math.rs`
- [x] Update the call site in the still-monolithic `mj_fwd_position` to use
      `crate::tendon::mj_fwd_tendon(model, data)` instead of a direct call
- [x] Run full test suite

**Estimated size**: ~1,150 lines moved (tendon modules ~1,124 + overhead ~26)

### Phase 6: Extract constraint system

The largest extraction. ~5,612 lines across 45+ functions → 12 files.
See "Constraint/Solver Module Revised Structure" in the Audit Findings section.

- [ ] Create `src/constraint/mod.rs` — move mj_fwd_constraint, mj_fwd_constraint_islands,
      compute_qacc_smooth, build_m_impl_for_newton, compute_qfrc_smooth_implicit,
      compute_point_velocity (~400 lines)
- [ ] Create `src/constraint/assembly.rs` — move assemble_unified_constraints,
      populate_efc_island (~750 lines)
- [ ] Create `src/constraint/equality.rs` — move all extract_*_jacobian functions,
      add_body_*_jacobian_row, get_min_* helpers (626 lines)
- [ ] Create `src/constraint/jacobian.rs` — move compute_flex_contact_jacobian,
      compute_contact_jacobian, add_angular_jacobian (~337 lines)
- [ ] Create `src/constraint/impedance.rs` — move compute_impedance,
      quaternion_to_axis_angle, compute_kbip, compute_aref, normalize_quat4,
      ball_limit_axis_angle, compute_diag_approx_exact, mj_solve_sparse_vec,
      compute_regularization (~343 lines)
- [ ] Create `src/constraint/solver/mod.rs` — compute_delassus_regularized,
      compute_qfrc_constraint_from_efc, extract_qfrc_frictionloss,
      decode_pyramid (~130 lines)
- [ ] Create `src/constraint/solver/pgs.rs` — pgs_solve_unified, pgs_cost_change,
      classify_constraint_states (~400 lines)
- [ ] Create `src/constraint/solver/cg.rs` — cg_solve_unified (~280 lines)
- [ ] Create `src/constraint/solver/newton.rs` — newton_solve, recover_newton (~313 lines)
- [ ] Create `src/constraint/solver/hessian.rs` — assemble_hessian,
      SparseHessian struct + 7 impl methods, hessian_incremental,
      hessian_cone (~685 lines)
- [ ] Create `src/constraint/solver/primal.rs` — shared CG/Newton infrastructure:
      compute_gradient_and_search, primal_prepare/eval/search, evaluate_cost_at,
      PrimalQuad/PrimalPoint structs (~653 lines)
- [ ] Create `src/constraint/solver/noslip.rs` — project_elliptic_cone,
      noslip_qcqp2, noslip_qcqp3, noslip_postprocess (~695 lines)

**Extraction order within Phase 6** (respects internal call dependencies):

1. `constraint/mod.rs` — entry points (mj_fwd_constraint, etc.)
2. `constraint/impedance.rs` — standalone helpers (no constraint-internal deps)
3. `constraint/jacobian.rs` — standalone (contact Jacobian construction)
4. `constraint/equality.rs` — standalone (equality constraint Jacobians)
5. `constraint/assembly.rs` — calls equality, jacobian, impedance
6. `constraint/solver/mod.rs` — dispatch + Delassus + qfrc recovery (calls solver sub-modules; initially imports from monolith until they're extracted in steps 7–12)
7. `constraint/solver/primal.rs` — shared infrastructure (used by CG + Newton)
8. `constraint/solver/pgs.rs` — uses primal
9. `constraint/solver/cg.rs` — uses primal
10. `constraint/solver/hessian.rs` — used by Newton
11. `constraint/solver/newton.rs` — uses primal + hessian
12. `constraint/solver/noslip.rs` — independent

- [ ] Run full test suite

**Estimated size**: ~5,612 lines moved. All 12 files under 800 lines.

### Phase 7: Extract dynamics (CRBA, RNE, factorization)

- [ ] Extend `src/dynamics/` module tree (created in Phase 2 with `spatial.rs`)
- [ ] Move mj_crba + cache_body_effective_mass → `dynamics/crba.rs`
- [ ] Move mj_rne + mj_gravcomp → `dynamics/rne.rs`
- [ ] Move mj_factor_sparse*, compute_qld_csr_metadata → `dynamics/factor.rs`
- [ ] Move mj_flex → `dynamics/flex.rs` (~10 lines — only `mj_flex` exists)
- [ ] Run full test suite

**Estimated size**: ~930 lines moved (crba ~350 + rne ~350 + factor ~200 + flex ~10 + overhead ~20)

### Phase 8a: Extract forward pipeline

The forward pipeline is the most interconnected extraction. Splitting it from
integration and island/sleep follows Principle #5 (one PR per major module).

- [ ] Create `src/forward/` module tree
- [ ] Move step/forward/forward_skip_sensors/forward_core → `forward/mod.rs`
- [ ] Move mj_fwd_position + aabb_from_geom + SweepAndPrune → `forward/position.rs`
- [ ] Move mj_fwd_velocity → `forward/velocity.rs`
- [ ] Move passive forces + PassiveForceVisitor → `forward/passive.rs`
      **WARNING**: L12690–L12816 (tendon implicit K/D helpers) sits between the
      two passive ranges but goes to `integrate/implicit.rs` in Phase 8b, NOT
      to `forward/passive.rs`. Only take L12108–L12690 and L12818–L12899.
- [ ] Move actuation pipeline → `forward/actuation.rs` (~566 lines)
- [ ] Move compute_muscle_params + muscle_* helpers → `forward/muscle.rs` (~259 lines)
- [ ] Move acceleration paths → `forward/acceleration.rs`
- [ ] Move check functions → `forward/check.rs`
- [ ] Move Jacobian functions → `src/jacobian.rs` (includes `mj_differentiate_pos`
      and `mj_integrate_pos_explicit` — primary callers are `derivatives.rs`)
- [ ] Run full test suite

**Estimated size**: ~3,050 lines moved (forward/* modules ~2,574 + jacobian.rs ~465 + overhead ~11)

### Phase 8b: Extract integration

- [ ] Create `src/integrate/` module tree
- [ ] Move integrate() dispatch + integrate_without_velocity() → `integrate/mod.rs`
- [ ] Move Euler integration + mj_integrate_pos + mj_normalize_quat +
      PositionIntegrateVisitor + QuaternionNormalizeVisitor →
      `integrate/euler.rs` (NOT mj_differentiate_pos or
      mj_integrate_pos_explicit — those go to `jacobian.rs` in Phase 8a)
- [ ] Move tendon implicit K/D helpers (tendon_all_dofs_sleeping,
      tendon_all_dofs_sleeping_fields, tendon_deadband_displacement,
      tendon_active_stiffness, accumulate_tendon_kd) →
      `integrate/implicit.rs` (ONLY tendon K/D helpers — implicit
      acceleration functions stay in `forward/acceleration.rs`)
- [ ] Move mj_runge_kutta → `integrate/rk4.rs`
- [ ] Run full test suite

**Estimated size**: ~600 lines moved (integrate modules sum: 120 + 166 + 127 + 169 = 582)

### Phase 8c: Extract island/sleep

- [ ] Create `src/island/` module tree
- [ ] Move mj_island, mj_flood_fill, equality_trees, constraint_tree → `island/mod.rs`
- [ ] Move mj_wake*, mj_sleep, mj_sleep_cycle, mj_update_sleep_arrays,
      mj_check_qpos_changed, mj_wake_tree, sensor_body_id, tree_can_sleep,
      sleep_trees, sync_tree_fk, reset_sleep_state,
      Data::sleep_state(), Data::tree_awake(), Data::nbody_awake(),
      Data::nisland() → `island/sleep.rs`
- [ ] Run full test suite

**Estimated size**: ~1,324 lines moved (island/sleep.rs: ~737 lines, island/mod.rs: ~587 lines)

> **Accounting note**: The per-phase "Estimated size" values sum to ~20,966
> lines. The file's production code is ~21,475 lines. The ~509-line
> undercount is the file preamble (`use` imports, ~105 lines) plus
> inter-function whitespace, section boundaries, and conservative rounding
> (~404 lines across all phases — most per-module estimates use function-body
> sizes which exclude surrounding whitespace and doc comments). The
> per-*module* estimates in the size table sum to ~20,707, leaving a
> ~768-line gap. This gap is distributed across: the file preamble (~105
> lines) and inter-function whitespace, section boundaries, and conservative
> rounding (~663 lines across all modules — most estimates use round numbers
> below the actual).
> Verify actual module sizes during extraction — estimates are approximate.

### Phase 10: Extract model_builder.rs into module tree

The second-largest extraction. 10,184 lines total (~6,032 production + ~4,152
tests; ~73 production functions (47 free functions + 24 impl methods + 2 nested:
`remove_visual_geoms` and `collect_mesh_refs`, both inside `apply_discardvisual` —
go to `builder/compiler.rs`) + 151 fn definitions in test
section (149 `#[test]` + 2 test helpers)) across 19 target modules (including
`builder/build.rs` and `builder/init.rs`). Same strategy as sim-core: move functions, move tests,
update imports, verify.

- [ ] Create `sim-mjcf/src/builder/mod.rs` — move `ModelBuilder` struct,
      `set_options()`, `resolve_keyframe()`, plus top-level orchestration functions:
      `model_from_mjcf()`, `load_model()`, `load_model_from_file()`
- [ ] Create `builder/init.rs` — move `ModelBuilder::new()` (~264 lines of field
      initialization; split from mod.rs via Rust's split-impl-block pattern to
      keep mod.rs under 800 lines)
- [ ] Create `builder/build.rs` — move `build(self) -> Model` method (~675 lines
      of model assembly; split from mod.rs via Rust's split-impl-block pattern)
- [ ] Create `builder/body.rs` — move `process_body()`, `process_body_with_world_frame()`,
      `process_worldbody_geoms_and_sites()`
- [ ] Create `builder/joint.rs` — move `process_joint()`
- [ ] Create `builder/geom.rs` — move `process_geom()`, `process_site()`,
      `compute_geom_mass()`, `compute_geom_inertia()`, `geom_effective_com()`,
      `geom_size_to_vec3()`, `compute_fromto_pose()`
- [ ] Create `builder/actuator.rs` — move `process_actuator()`, `parse_gaintype()`,
      `parse_biastype()`, `parse_dyntype()`, `floats_to_array()`
- [ ] Create `builder/sensor.rs` — move `process_sensors()`, `resolve_sensor_object()`,
      `convert_sensor_type()`, `sensor_datatype()`
- [ ] Create `builder/tendon.rs` — move `process_tendons()`
- [ ] Create `builder/contact.rs` — move `process_contact()`,
      `compute_initial_geom_distance()`, `geom_world_position()`
- [ ] Create `builder/equality.rs` — move `process_equality_constraints()`
- [ ] Create `builder/flex.rs` — move `process_flex_bodies()`,
      `compute_flexedge_crosssection()`, `compute_flex_address_table()`,
      `compute_flex_count_table()`, `compute_vertex_masses()`,
      `compute_dihedral_angle()`, `compute_edge_solref()`,
      `compute_bend_stiffness_from_material()`, `compute_bend_damping_from_material()`
- [ ] Create `builder/mass.rs` — move `apply_mass_pipeline()`,
      `compute_inertia_from_geoms()`, `extract_inertial_properties()`
- [ ] Create `builder/mesh.rs` — move `process_mesh()`, `process_hfield()`,
      `load_mesh_file()`, `convert_mjcf_mesh()`, `convert_embedded_mesh()`,
      `convert_mjcf_hfield()`, `compute_mesh_inertia()`, `resolve_mesh()`
- [ ] Create `builder/frame.rs` — move `expand_frames()`, `expand_single_frame()`,
      `frame_accum_child()`, `validate_childclass_references()`,
      `validate_frame_childclass_refs()`
- [ ] Create `builder/compiler.rs` — move `apply_discardvisual()`,
      `remove_visual_geoms()`, `collect_mesh_refs()`, `apply_fusestatic()`,
      `fuse_static_body()`
- [ ] Create `builder/orientation.rs` — move `quat_from_wxyz()`, `quat_to_wxyz()`,
      `euler_seq_to_quat()`, `resolve_orientation()`
- [ ] Create `builder/fluid.rs` — move `compute_geom_fluid()`, `geom_semi_axes()`,
      `get_added_mass_kappa()`
- [ ] `resolve_keyframe()` stays in `builder/mod.rs` (part of orchestration layer)
- [ ] Create `builder/asset.rs` — move `resolve_asset_path()`
- [ ] Update `sim-mjcf/src/lib.rs` — change `mod model_builder;` to `mod builder;`,
      update all `pub use` re-exports to route through `builder::`
- [ ] Move all 151 fn definitions in test section (149 `#[test]` + 2 test helpers)
      to their respective builder sub-modules
- [ ] Run full test suite

**Extraction order within Phase 10** (respects internal call dependencies):

1.  `builder/mod.rs` — ModelBuilder struct + set_options() +
    orchestration stubs (model_from_mjcf, load_model, load_model_from_file,
    resolve_keyframe). All process_* calls remain as `self.process_*()` —
    they resolve via Rust's impl-across-files once extracted.
1b. `builder/init.rs` — ModelBuilder::new() (~264 lines of field initialization).
    Split from mod.rs immediately to stay under 800 lines.
2.  `builder/orientation.rs` — leaf helper (resolve_orientation, quat_from_wxyz,
    quat_to_wxyz, euler_seq_to_quat). No internal deps.
3.  `builder/asset.rs` — leaf helper (resolve_asset_path). No internal deps.
4.  `builder/fluid.rs` — leaf helper (compute_geom_fluid, geom_semi_axes,
    get_added_mass_kappa). No internal deps.
5.  `builder/compiler.rs` — preprocessing (apply_discardvisual, apply_fusestatic,
    fuse_static_body, remove_visual_geoms, collect_mesh_refs). No internal deps.
6.  `builder/frame.rs` — preprocessing (expand_frames, expand_single_frame,
    frame_accum_child, validate_childclass_references,
    validate_frame_childclass_refs). No internal deps.
7.  `builder/mesh.rs` — asset loading (process_mesh, process_hfield,
    load_mesh_file, convert_mjcf_mesh, convert_embedded_mesh,
    convert_mjcf_hfield, compute_mesh_inertia, resolve_mesh).
    Deps: asset.rs (resolve_asset_path).
8.  `builder/geom.rs` — geometry processing (process_geom, process_site,
    compute_geom_mass, compute_geom_inertia, geom_effective_com,
    geom_size_to_vec3, compute_fromto_pose).
    Deps: orientation.rs, mesh.rs (resolve_mesh).
9.  `builder/joint.rs` — joint processing (process_joint).
    Deps: orientation.rs.
10. `builder/body.rs` — body tree traversal (process_body,
    process_body_with_world_frame, process_worldbody_geoms_and_sites).
    Deps: orientation.rs, geom.rs, joint.rs.
11. `builder/mass.rs` — mass pipeline (apply_mass_pipeline,
    compute_inertia_from_geoms, extract_inertial_properties).
    Deps: geom.rs (compute_geom_mass, compute_geom_inertia).
12. `builder/actuator.rs` — actuator wiring (process_actuator, parse_gaintype,
    parse_biastype, parse_dyntype). No internal deps beyond name lookups.
13. `builder/sensor.rs` — sensor wiring (process_sensors,
    resolve_sensor_object, convert_sensor_type, sensor_datatype).
14. `builder/tendon.rs` — tendon processing (process_tendons).
15. `builder/contact.rs` — contact pair processing (process_contact,
    compute_initial_geom_distance, geom_world_position).
16. `builder/equality.rs` — equality constraints (process_equality_constraints).
17. `builder/flex.rs` — flex body processing (process_flex_bodies and compute_*
    helpers). Deps: none beyond ModelBuilder fields.
18. `builder/build.rs` — model assembly (build(self) -> Model). Must be LAST
    because it references fields populated by all process_* methods.
    Deps: flex.rs (compute_flex_address_table, compute_flex_count_table,
    compute_flexedge_crosssection).
19. Tests — move inline tests to their respective modules after all production
    code is extracted.

**Estimated size**: ~10,184 lines total (~6,032 production + ~4,152 tests)
reorganized across 19 modules. Largest expected: `builder/build.rs` at ~675
production lines.

### Phase 12: Final verification, reference sweep, documentation rewrite

This is the most important phase. The code is already modular — now we make
sure every trace of the monolith is gone from documentation and comments.

- [ ] **Run full workspace tests**: `cargo test`
- [ ] **Run clippy**: `cargo clippy -- -D warnings`
- [ ] **Run fmt**: `cargo fmt --all -- --check`
- [ ] **Run quality gate**: `cargo xtask check`
- [ ] **Verify monolith is gone**:
  - `mujoco_pipeline.rs` ≤100 lines (re-export shim) or deleted
  - `model_builder.rs` ≤100 lines (re-export shim) or deleted
- [ ] **Exhaustive stale reference sweep** (must return zero matches):
  ```bash
  grep -rn 'mujoco_pipeline\.rs' sim/ --include='*.rs' --include='*.md' \
    | grep -v 'STRUCTURAL_REFACTOR\|CHANGELOG'
  grep -rn 'model_builder\.rs' sim/ --include='*.rs' --include='*.md' \
    | grep -v 'STRUCTURAL_REFACTOR\|CHANGELOG'
  ```
- [ ] **Rewrite `sim/docs/ARCHITECTURE.md`** — replace the current "The
      simulation engine lives in `mujoco_pipeline.rs`" section with the new
      module tree structure. This is the primary document newcomers read.
- [ ] **Update all `sim/docs/todo/future_work_*.md`** files — ~692 references
      across 27 files (432 `mujoco_pipeline.rs` refs + 262 `model_builder.rs` refs).
      Use the **Doc Reference Mapping Table** (in the Module Size Estimates section) to map line-range citations
      (e.g., `mujoco_pipeline.rs:L15334`) to target modules. Bulk find-and-replace
      with module-specific paths (e.g., `constraint/solver/newton.rs`, `forward/passive.rs`).
- [ ] **Update `sim/docs/MUJOCO_GAP_ANALYSIS.md`** — fix module references
- [ ] **Verify `sim/docs/TRAIT_ARCHITECTURE.md`** — 0 references (verified 2026-02-22,
      likely no update needed; re-verify at execution time)
- [ ] **Update `sim/docs/MUJOCO_CONFORMANCE.md`** — 4 references to fix
- [ ] **Update `sim/docs/MUJOCO_REFERENCE.md`** — 1 reference to fix
- [ ] **Update test file comments** in `sim/L0/tests/integration/`:
  - `mjcf_sensors.rs:4` — update module references
  - `spatial_tendons.rs:1171` — update test location reference
  - `sensors.rs:6` — update module reference
  - `equality_constraints.rs:1017` — update module reference
  - `mod.rs:57` — update module reference
- [ ] **Update `sim-core/src/gjk_epa.rs`** lines 264, 291 — comments reference
      `mujoco_pipeline.rs`
- [ ] **Update `CLAUDE.md`** if any workflow changes
- [ ] **Run the rubric verification script** from STRUCTURAL_REFACTOR_RUBRIC.md
- [ ] **Grade all 15 final-state checks** from the rubric

---

## What stays in `mujoco_pipeline.rs`

After all phases complete, `mujoco_pipeline.rs` should either:

**Option A: Delete entirely.** All code moved to modules. `lib.rs` declares
the modules directly.

**Option B: Thin re-export file.** `mujoco_pipeline.rs` becomes a ~50-line
file that re-exports from the new modules, preserving the `pub use
mujoco_pipeline::*` pattern in `lib.rs`. This avoids changing downstream
import paths.

**Recommendation: Option B.** It's the smallest diff for downstream code
(sim-mjcf, sim-conformance-tests, sim-bevy all import from
`sim_core::mujoco_pipeline::*` or `sim_core::Model`). We can delete the
shim later when convenient.

---

## Risk Assessment

| Risk | Mitigation |
|------|-----------|
| Visibility errors after move (`pub` vs `pub(crate)`) | Each phase runs full test suite; compiler catches this |
| Circular module dependencies | Module tree follows the pipeline's data flow (types → dynamics → forward → constraint → integrate) — no cycles |
| Merge conflicts with in-progress feature work | Do this before next feature phase. No concurrent physics changes. |
| Performance regression from module boundaries | Zero-cost: Rust modules have no runtime overhead. `#[inline]` on hot paths if needed. |
| Import path changes breaking downstream | Option B (re-export shim) prevents this entirely |
| "While we're here" scope creep | Principle #6. Enforce in review. |

### Rollback Procedure

Each phase produces one or more commits (one per sub-module extraction, as
specified in the "Every phase includes" checklist). To revert phase N:

1. Identify the commits belonging to phase N (they will be contiguous on the
   branch and labeled with the phase number in commit messages).
2. `git revert` the phase-N commits in reverse chronological order.
3. Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics`
   to verify the revert restored baseline.

Phases should be reverted in reverse chronological order. Reverting phase N
may require also reverting phases that depend on N (see Phase Dependencies
below) — for example, if Phase 3 (collision) and Phase 4 (sensor) are both
complete, sensor modules may import from `crate::collision::*`, so reverting
Phase 3 would break Phase 4. Within a single phase, commits can be reverted
in reverse order independently.

---

## Success Criteria

See [STRUCTURAL_REFACTOR_RUBRIC.md](./STRUCTURAL_REFACTOR_RUBRIC.md) for the
full 8-criterion grading rubric. Every criterion must be **A-grade** before
each phase is committed.

**Summary of hard gates**:

1. **Same test count, same test results.** `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics` produces
   identical pass/fail/skip counts before and after.
2. **No file over 800 lines of production code.** (Tests can push individual
   files higher, but production logic stays under 800.) See rubric S1.
3. **`mujoco_pipeline.rs` either deleted or ≤100 lines** (re-export shim).
4. **`model_builder.rs` either deleted or ≤100 lines**.
5. **A newcomer can understand the pipeline stages** by reading `forward/mod.rs`
   (~90 lines of orchestration) without opening any other file. See rubric S5.
6. **`cargo clippy -- -D warnings` passes.**
7. **`cargo fmt --all -- --check` passes.**
8. **Zero stale references** to `mujoco_pipeline.rs` or `model_builder.rs`
   anywhere in the codebase. See rubric S6.
9. **Every module has a `//!` doc comment** — one sentence describing its
   scope and the MuJoCo C file it corresponds to. See rubric S7.
10. **3-click discoverability** — any function reachable from `lib.rs` by
    navigating at most 3 levels of `mod.rs`. See rubric S7.

---

## Timeline and Ordering

This refactor should happen **before** the next batch of v1.0 feature work
(Phase 1 correctness bugs). Reasons:

1. Every future feature adds more code to the monolith, making the refactor
   harder.
2. The refactor creates clean seams that make future features easier to
   implement and review.
3. No physics knowledge is needed for most of the extraction — it's mechanical
   code movement.

Estimated effort: 13 phases (0–8c, 10, 12; 9 and 11 are intentionally skipped
to separate sim-core extraction from sim-mjcf extraction and final cleanup),
each independently committable. The whole refactor can be done incrementally
over multiple sessions with commits between each phase.

## Phase Dependencies

The phases are not all strictly sequential. This DAG shows the required
ordering — any phase can begin once all its predecessors are complete.

```
Phase 0 (preparation)
  │
  v
Phase 1 (types)
  │
  v
Phase 2 (linalg + spatial + joint_visitor)
  │
  ├──────────┬──────────┬──────────┬──────────┐
  v          v          v          v          v
Phase 3    Phase 4    Phase 5    Phase 6    Phase 7
(collision) (sensor)  (tendon) (constraint) (dynamics)
  │          │          │          │          │
  └──────────┴──────────┴──────────┴──────────┘
                        │
                        v
                    Phase 8a (forward pipeline)
                        │
                    ┌───┴───┐
                    v       v
                Phase 8b  Phase 8c
              (integrate) (island/sleep)

Phase 10 (model_builder) ← independent of sim-core phases;
  │                         requires only Phase 0
  v
Phase 12 (final verification) ← requires ALL above phases
```

**Key constraints**:
- Phases 0 → 1 → 2 are strictly sequential (each builds on the prior)
- Phases 3, 4, 5, 6, 7 are mutually independent after Phase 2
- Phase 6 requires Phase 2 (constraint code imports from linalg/spatial)
- Phase 8a requires {2, 3, 4, 5, 6, 7} — it is the bottleneck
- Phases 8b and 8c require Phase 8a
- Phase 10 (sim-mjcf) is independent of sim-core phases
- Phase 12 requires all above phases

---

## Relationship to Other Plans

| Document | Relationship |
|----------|-------------|
| `ROADMAP_V1.md` | This refactor is a **prerequisite** inserted before Phase 1. No roadmap items are changed. |
| `TRAIT_ARCHITECTURE.md` | The trait boundaries (FlexBendingModel, etc.) need module seams to slot into. This refactor creates those seams. |
| `future_work_11.md #44` | Legacy crate deprecation. Independent of this refactor but becomes easier once module boundaries are clear. |
| `ARCHITECTURE.md` | Must be updated in Phase 12 to reflect new structure. |
