# Structural Refactor — Grading Rubric

> **Purpose**: Enforce A-grade readability and organization throughout the
> `sim-core` and `sim-mjcf` structural refactor. Every phase is graded against
> this rubric before it can be committed. The rubric measures what matters:
> can a human or AI read this code and immediately understand where they are,
> what this module does, and how it fits into the whole?
>
> **Relationship to STANDARDS.md**: The project-wide 7-criterion system
> (coverage, docs, clippy, safety, deps, bevy-free, API design) continues to
> apply. This rubric adds **8 structural criteria** specific to the refactor
> that STANDARDS.md doesn't cover — module size, naming, dependency flow,
> discoverability, etc. Both must pass.

---

## Grading Scale

| Grade | Meaning | Action |
|-------|---------|--------|
| **A** | Exemplary — a newcomer can navigate it cold | Ship it |
| **B** | Good — minor readability issues | Fix before committing |
| **C** | Functional but confusing | Rework the module boundaries |
| **F** | Monolith in disguise | Redo the extraction |

**Every criterion must be A for the phase to be committed. No exceptions.**

---

## The 8 Structural Criteria

### S1. Module Size — "Can I read this in one sitting?"

| Grade | Threshold |
|-------|-----------|
| **A** | Every `.rs` file has **≤800 lines** of production code (excluding `#[cfg(test)]` blocks) |
| **B** | Every `.rs` file has **≤1,200 lines** of production code |
| **C** | No file exceeds **1,500 lines** of production code |
| **F** | Any file exceeds **1,500 lines** of production code |

**Why 800**: That's roughly what fits in a single focused reading session — or
in an AI's working context without losing the thread. MuJoCo's C files average
~600-1,200 lines each. 800 lines of Rust (more expressive per line than C) is
the sweet spot.

**How to measure**:
```bash
# Count production lines (exclude test blocks)
# For each .rs file in sim-core/src/:
#
# Note: This assumes all #[cfg(test)] blocks are module-level (mod tests { ... }),
# not individual function attributes. Verified for mujoco_pipeline.rs and
# model_builder.rs. If a file has #[cfg(test)] on individual functions, this
# will undercount production lines.
awk '/#\[cfg\(test\)\]/{ stop=1 } !stop{ count++ } END{ print count }' file.rs
```

**Edge case**: This awk script stops at the FIRST `#[cfg(test)]`, which could
be a `#[cfg(test)] pub(crate) mod test_helpers` at the top of a file rather
than the actual `mod tests` block at the bottom. After the refactor, verify
that no extracted module has `#[cfg(test)]` attributes above the final
`mod tests` block. If a module needs `#[cfg(test)]` on individual items
(e.g., test-only imports), place them inside the `mod tests` block or
immediately before it.

**Inline tests** (`#[cfg(test)]` blocks) don't count toward the limit. They
can push a file over 800 total lines, and that's fine — the goal is that the
production code you need to understand fits in your head.

#### Pre-existing File Exemptions

The following files already exceed 800 production lines **before** this refactor
begins. They are **exempt from S1** during this refactor because it targets only
`mujoco_pipeline.rs` and `model_builder.rs` — these files are not being touched.

| File | Production lines (excl. `#[cfg(test)]` blocks) |
|------|-----------------|
| `sim-mjcf/src/types.rs` | 3,775 |
| `sim-mjcf/src/parser.rs` | 3,470 |
| `sim-core/src/derivatives.rs` | 2,393 |
| `sim-core/src/sdf.rs` | 1,565 |
| `sim-core/src/mesh.rs` | 1,344 |
| `sim-core/src/gjk_epa.rs` | 1,051 |
| `sim-core/src/contact.rs` | 1,044 |
| `sim-core/src/raycast.rs` | 924 |
| `sim-core/src/collision_shape.rs` | 883 |
| `sim-mjcf/src/defaults.rs` | 871 |
| `sim-core/src/mid_phase.rs` | 850 |

**Follow-up mandate**: These files should be decomposed in a future refactor
pass. Do not add new code to them. Tracked as:
- `parser.rs` (3,470 lines) → `sim/docs/todo/future_work_17.md` or
  equivalent post-refactor ticket
- `types.rs` (3,775 lines) → same
- `defaults.rs` (871 lines) → same
- `derivatives.rs` (2,393 lines) → same
- Other sim-core files (sdf, mesh, gjk_epa, contact, raycast,
  collision_shape, mid_phase) → same

After the structural refactor lands, add a `# [S1 EXEMPT]` comment to the
top of each file as a visible reminder. Any PR that increases the production
line count of an exempt file must include justification.

---

### S2. Module Naming — "Can I guess what's in the file from its name?"

| Grade | Threshold |
|-------|-----------|
| **A** | Every module name maps 1:1 to a recognizable pipeline stage or concept. A newcomer who knows MuJoCo (or any physics engine) can predict the contents from the name alone. |
| **B** | Names are accurate but require domain knowledge to decode |
| **C** | Some names are ambiguous or overlap in scope |
| **F** | Names are misleading or a module contains unrelated concerns |

**A-grade naming checklist**:

- [ ] Module name = the thing it computes, not the data it touches
  - `crba.rs` (what it does) not `mass_matrix.rs` (what it writes to)
  - `pgs.rs` (the algorithm) not `dual_solver.rs` (a vague category)
- [ ] No `utils.rs`, `util.rs`, `helpers.rs`, `misc.rs`, or `common.rs` —
      these are code smell names that become dumping grounds
- [ ] No `pipeline.rs` — that's what we're escaping
- [ ] Nested modules use the parent as namespace:
  - `constraint/solver/pgs.rs` reads as "the PGS constraint solver"
  - `forward/position.rs` reads as "forward kinematics position stage"
- [ ] The `mod.rs` in each directory is the **orchestration** — it calls the
      sub-modules. It is not a dumping ground for leftover code.

**The test**: If you told a physics PhD student "find where the Newton solver
lives", could they navigate to the right file in under 10 seconds by reading
directory names alone?

---

### S3. Single Responsibility — "Does this module do exactly one thing?"

| Grade | Threshold |
|-------|-----------|
| **A** | Every module has a one-sentence description that fully captures its scope. No function in the module falls outside that sentence. |
| **B** | One or two functions are tangentially related but defensible |
| **C** | Module mixes two distinct pipeline stages |
| **F** | Module is a grab bag of unrelated functions |

**The one-sentence test**: Can you write a `//!` doc comment for the module
that is a single sentence, not a list? If it requires "and" between unrelated
concepts, the module does too much.

Good:
```rust
//! Recursive Newton-Euler algorithm for computing bias forces.
```

Bad:
```rust
//! Bias force computation, gravity compensation, and some linear algebra helpers.
```

If you catch yourself writing a list, split the module.

---

### S4. Dependency Direction — "Does the data flow make sense?"

| Grade | Threshold |
|-------|-----------|
| **A** | Module dependencies form a clean DAG with no cycles. The direction matches the physics pipeline flow (types → dynamics → forward → constraint → integrate). |
| **B** | DAG is clean but one module reaches backward for a utility function |
| **C** | Two modules are mutually dependent through `pub(crate)` |
| **F** | Circular dependencies requiring workarounds |

**The rule**: Modules later in the pipeline can depend on modules earlier in
the pipeline. Never the reverse. `linalg.rs` and `types/` are at the root —
everything can depend on them. `integrate/` is at the leaf — nothing depends
on it.

```
                    types/
                      │
               ┌──────┼──────────┬──────────┐
               │      │          │          │
            linalg  joint_     energy   jacobian
               │   visitor   (utility —    (utility — used by constraint,
               │      │     standalone      sensor, derivatives, tendon)
               │      │     query, called
               │      │     from forward_core
               │      │     and Data::total_energy)
           dynamics/   │
        ┌──┬──┼──┬──┐  │
        │  │  │  │  │  │
      crba rne spatial factor flex
          │    │    │
          └────┼────┘
               │
         ┌─────┼──────┐
         │     │      │
      forward/ │   tendon/ ←── forward/position calls into tendon
         │     │      │
    ┌────┼─────┼──┐   │
    │    │     │  │   │
 position velocity passive actuation
    │
    ├── collision/
    │
 constraint/
    │
 island/
    │
 integrate/  (leaf — nothing depends on this)
    │
 sensor/  (leaf — reads Data, writes sensordata)
```

**Constraint sub-module dependency DAG** (see Phase 6 extraction order in
STRUCTURAL_REFACTOR.md for the full rationale):

```
constraint/mod.rs
  ├── impedance.rs  (standalone)
  ├── jacobian.rs   (standalone)
  ├── equality.rs   (standalone)
  ├── assembly.rs   (calls equality, jacobian, impedance)
  └── solver/
      ├── mod.rs        (dispatch)
      ├── primal.rs     (shared infra)
      ├── pgs.rs        (uses primal)
      ├── cg.rs         (uses primal)
      ├── hessian.rs    (used by newton)
      ├── newton.rs     (uses primal + hessian)
      └── noslip.rs     (standalone)
```

**Known cross-cuts (validated by audit, all are same-crate imports)**:
- `forward/acceleration.rs` imports from `crate::derivatives` (implicit integrators)
- `derivatives.rs` imports from `dynamics::spatial`, `linalg`, `jacobian`, etc.
- `forward/position.rs` calls `crate::tendon::mj_fwd_tendon`
- `integrate/rk4.rs` calls `data.forward_skip_sensors()` (method on Data, not module import)

**How to verify**: After each extraction, check that no module you just created
has `use crate::forward::*` when it lives in `dynamics/`. If it does, you moved
the function to the wrong module.

---

### S5. `mod.rs` Clarity — "Can I read just the orchestration?"

| Grade | Threshold |
|-------|-----------|
| **A** | Every `mod.rs` file reads as a **table of contents** for its directory. The public function in `mod.rs` calls sub-module functions in pipeline order, and that call sequence tells the story of what this module does. A reader who reads only `mod.rs` understands the architecture. |
| **B** | `mod.rs` has clear orchestration but some helper logic mixed in |
| **C** | `mod.rs` contains significant implementation detail |
| **F** | `mod.rs` is just another big file — the module tree is cosmetic |

**The ideal `forward/mod.rs`** (illustrative — matches the actual `step` and
`forward_core` implementations structurally from `mujoco_pipeline.rs`):

```rust
//! Forward dynamics pipeline — top-level orchestration.
//!
//! This module implements `step`, `forward`, and `forward_core` on `Data`,
//! which call sub-modules in physics pipeline order. Corresponds to
//! MuJoCo's `engine_forward.c`.

mod acceleration;
mod actuation;
mod check;
mod passive;
mod position;
mod velocity;

pub(crate) use acceleration::mj_fwd_acceleration;
pub(crate) use position::mj_fwd_position;
pub(crate) use velocity::mj_fwd_velocity;

impl Data {
    pub fn step(&mut self, model: &Model) -> Result<(), StepError> {
        if model.timestep <= 0.0 || !model.timestep.is_finite() {
            return Err(StepError::InvalidTimestep);
        }

        check::mj_check_pos(model, self)?;
        check::mj_check_vel(model, self)?;

        match model.integrator {
            Integrator::RungeKutta4 => {
                // RK4: forward() evaluates initial state (with sensors).
                // mj_runge_kutta() then calls forward_skip_sensors() 3 more times.
                self.forward(model)?;
                check::mj_check_acc(model, self)?;
                crate::integrate::mj_runge_kutta(model, self)?;
            }
            Integrator::Euler
            | Integrator::ImplicitSpringDamper
            | Integrator::ImplicitFast
            | Integrator::Implicit => {
                self.forward(model)?;
                check::mj_check_acc(model, self)?;
                self.integrate(model);
            }
        }

        // Sleep update (§16.12): Phase B island-aware sleep transition.
        let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
        if sleep_enabled {
            crate::island::mj_sleep(model, self);
            crate::island::mj_update_sleep_arrays(model, self);
        }

        // Save qacc for next-step warmstart (§15.9).
        self.qacc_warmstart.copy_from(&self.qacc);

        Ok(())
    }

    pub fn forward(&mut self, model: &Model) -> Result<(), StepError> {
        self.forward_core(model, true)
    }

    pub(crate) fn forward_skip_sensors(&mut self, model: &Model) -> Result<(), StepError> {
        self.forward_core(model, false)
    }

    fn forward_core(&mut self, model: &Model, compute_sensors: bool) -> Result<(), StepError> {
        let sleep = model.enableflags & ENABLE_SLEEP != 0;

        // ===== Pre-pipeline: Wake detection (§16.4) =====
        if sleep && crate::island::mj_wake(model, self) {
            crate::island::mj_update_sleep_arrays(model, self);
        }

        // ========== Position Stage ==========
        // Note: tendon kinematics runs INSIDE mj_fwd_position (after site
        // poses, before subtree COM). It is not a separate pipeline call.
        position::mj_fwd_position(model, self);
        crate::dynamics::flex::mj_flex(model, self);

        // §16.15: If FK detected external qpos changes on sleeping bodies, wake them
        if sleep && crate::island::mj_check_qpos_changed(model, self) {
            crate::island::mj_update_sleep_arrays(model, self);
        }

        actuation::mj_transmission_site(model, self);

        // §16.13.2: Tendon wake — multi-tree tendons with active limits
        if sleep && crate::island::mj_wake_tendon(model, self) {
            crate::island::mj_update_sleep_arrays(model, self);
        }

        crate::collision::mj_collision(model, self);

        // Wake-on-contact (§16.5c): re-run collision for newly-awake tree's geoms
        if sleep && crate::island::mj_wake_collision(model, self) {
            crate::island::mj_update_sleep_arrays(model, self);
            crate::collision::mj_collision(model, self);
        }

        // §16.13.3: Equality constraint wake — cross-tree equality coupling
        if sleep && crate::island::mj_wake_equality(model, self) {
            crate::island::mj_update_sleep_arrays(model, self);
        }

        // §36: Body transmission — requires contacts from mj_collision()
        actuation::mj_transmission_body_dispatch(model, self);

        if compute_sensors {
            crate::sensor::mj_sensor_pos(model, self);
        }
        crate::energy::mj_energy_pos(model, self);

        // ========== Velocity Stage ==========
        velocity::mj_fwd_velocity(model, self);
        actuation::mj_actuator_length(model, self);
        if compute_sensors {
            crate::sensor::mj_sensor_vel(model, self);
        }

        // ========== Acceleration Stage ==========
        actuation::mj_fwd_actuation(model, self);
        crate::dynamics::crba::mj_crba(model, self);
        crate::dynamics::rne::mj_rne(model, self);
        crate::energy::mj_energy_vel(model, self);
        passive::mj_fwd_passive(model, self);

        // §16.11: Island discovery before constraint solve
        if sleep {
            crate::island::mj_island(model, self);
        }
        crate::constraint::mj_fwd_constraint_islands(model, self);

        if !self.newton_solved {
            acceleration::mj_fwd_acceleration(model, self)?;
        }

        // ========== Sensors (optional) ==========
        if compute_sensors {
            crate::sensor::mj_sensor_acc(model, self);
            crate::sensor::mj_sensor_postprocess(model, self);
        }

        Ok(())
    }
}
```

This matches the **actual** `step` and `forward_core` implementations from
`mujoco_pipeline.rs` (as of 2026-02-22). A reader who sees only this
understands the **entire simulation pipeline** in ~90 lines. That's the goal.

---

### S6. Reference Completeness — "Did we update everything?"

| Grade | Threshold |
|-------|-----------|
| **A** | Zero stale references. Every mention of `mujoco_pipeline.rs` or `model_builder.rs` in the codebase is updated or intentionally preserved with a comment explaining why. |
| **B** | Code references are all correct; 1-2 stale doc references remain |
| **C** | Some code imports reference old paths (but compile via re-exports) |
| **F** | Broken imports, dead references, or misleading documentation |

**The exhaustive reference list** (must be checked after every phase):

> *Path shorthand: `sim-core` = `sim/L0/core`, `sim-mjcf` = `sim/L0/mjcf`.*

#### Code references (MUST be correct — compiler enforces):

| File | Reference | What to update |
|------|-----------|----------------|
| `sim-core/src/lib.rs:80` | `pub mod mujoco_pipeline;` | Change to new module declarations |
| `sim-core/src/lib.rs:111` | `pub use mujoco_pipeline::{...}` | Route through new modules |
| `sim-core/src/derivatives.rs:56` | `use crate::mujoco_pipeline::{...}` | Point to new module paths |
| `sim-core/src/derivatives.rs:2402` | `use crate::mujoco_pipeline::{ellipsoid_moment, norm3}` | Point to `forward::passive` (fluid/aero helpers) |
| `sim-core/src/batch.rs:42` | `use crate::mujoco_pipeline::{Data, Model, StepError}` | Point to `types::` |
| `sim-mjcf/src/lib.rs:176` | `mod model_builder;` | Change to `mod builder;` |
| `sim-mjcf/src/lib.rs:197` | `pub use model_builder::{...}` | Route through `builder::` |
| `sim-core/src/mujoco_pipeline.rs:2096` | Comment referencing `model_builder.rs` | Update path |
| `sim-core/src/gjk_epa.rs:264,291` | Comments referencing `mujoco_pipeline.rs` | Update to new module path |
| `sim-core/src/derivatives.rs:70` | Comment referencing `mujoco_pipeline` | Update to new module path |

#### Documentation references (MUST be updated — humans and AI read these):

| File | Approximate references | Action |
|------|----------------------|--------|
| `sim/docs/ARCHITECTURE.md` | ~7 references total | Rewrite to describe new module structure |
| `sim/docs/MUJOCO_GAP_ANALYSIS.md` | ~35 references | Update module paths |
| `sim/docs/todo/future_work_*.md` | ~704 references across 19 files (436 `mujoco_pipeline.rs` + 268 `model_builder.rs`) | Bulk find-and-replace using Doc Reference Mapping Table in STRUCTURAL_REFACTOR.md |
| `sim/docs/STRUCTURAL_REFACTOR.md` | Many references (this is the spec) | Update to reflect actual final structure |
| `sim/docs/MUJOCO_CONFORMANCE.md` | 4 references | Update module paths |
| `sim/docs/MUJOCO_REFERENCE.md` | 1 reference | Update module path |
| `sim/docs/TRAIT_ARCHITECTURE.md` | 0 references (verified 2026-02-22, no update needed) | No action required |

#### Test file references (should be updated for accuracy):

| File | Reference |
|------|-----------|
| `sim/L0/tests/integration/mjcf_sensors.rs:4` | Comment about `model_builder.rs` and `mujoco_pipeline.rs` |
| `sim/L0/tests/integration/spatial_tendons.rs:1171` | Comment about tests in `mujoco_pipeline.rs` |
| `sim/L0/tests/integration/sensors.rs:6` | Comment about `mujoco_pipeline.rs` |
| `sim/L0/tests/integration/equality_constraints.rs:1017` | Comment about `model_builder` |
| `sim/L0/tests/integration/mod.rs:57` | Comment about `model_builder` |

**Verification command** (run after every phase):
```bash
# Should return ZERO matches after the refactor is complete
grep -rn 'mujoco_pipeline\.rs' sim/ --include='*.rs' --include='*.md' \
  | grep -v 'STRUCTURAL_REFACTOR' \
  | grep -v 'CHANGELOG'

grep -rn 'model_builder\.rs' sim/ --include='*.rs' --include='*.md' \
  | grep -v 'STRUCTURAL_REFACTOR' \
  | grep -v 'CHANGELOG'
```

**Lazy import check** (manual, per phase — not in the automated script):

The S6 grep catches `.rs` filename strings in comments, but it does NOT catch
`use crate::mujoco_pipeline::` import paths that route through the re-export
shim instead of pointing to the real module. To catch those: after adding
monolith re-imports, temporarily comment them out and run `cargo check -p sim-core`.
Any errors in files OTHER than the monolith indicate an import that should have
been updated. Fix those imports, then uncomment. See step 5a in the
STRUCTURAL_REFACTOR.md phase checklist.

---

### S7. Discoverability — "Can I find what I need without grep?"

| Grade | Threshold |
|-------|-----------|
| **A** | The module tree structure alone tells you where any piece of functionality lives. You can navigate to the constraint CG solver, the flex collision code, or the muscle dynamics without searching — just by reading directory names and `mod.rs` files. |
| **B** | Most things are findable by navigation; one or two require a search |
| **C** | You need grep to find more than half the functions |
| **F** | The module tree doesn't help — might as well be one file |

**A-grade discoverability checklist**:

- [ ] **3-click rule**: From `sim-core/src/lib.rs`, any function is reachable
      by navigating at most 3 levels of `mod.rs` files
      (e.g., `lib.rs` → `constraint/mod.rs` → `constraint/solver/mod.rs` → `newton.rs`)
- [ ] **Module-level doc comments** (`//!`) on every module explain:
  1. What pipeline stage this implements
  2. What the key public functions are
  3. What MuJoCo C file this corresponds to (where applicable)
- [ ] **Re-exports in `mod.rs`** surface the most important symbols — a reader
      doesn't have to guess which sub-module contains `mj_fwd_constraint`
- [ ] **No hidden coupling**: If module A calls a function in module B, that
      function is visible in B's `mod.rs` re-exports. No reaching into
      `B::internal_submodule::private_helper` from outside.
- [ ] **Alphabetical ordering** of `mod` declarations and `use` statements
      within each file (unless pipeline-order is more informative)

**The discoverability test**: Hand the `sim-core/src/` directory listing to
someone who's never seen the codebase. Ask them: "Where would you look to
understand how contacts are solved?" If they say `constraint/solver/` without
hesitation, that's an A.

---

### S8. Test Colocation — "Are tests near the code they test?"

| Grade | Threshold |
|-------|-----------|
| **A** | Unit tests live in `#[cfg(test)]` blocks in the same file as the code they test. Integration tests stay in `sim-conformance-tests`. No test is orphaned from its module. |
| **B** | Tests are colocated but a few test helpers are duplicated across modules |
| **C** | Some unit tests ended up in the wrong module |
| **F** | Tests are in a separate file with no clear mapping to the code they test |

**Rules**:

1. **Unit tests** (`#[cfg(test)] mod tests { ... }`) go in the same `.rs` file
   as the functions they test. When moving `collide_sphere_sphere` from the
   monolith to `collision/pair_convex.rs`, its tests move too.

2. **Integration tests** in `sim-conformance-tests` stay put. They test the
   public API (`Model`, `Data`, `step`, `forward`), not internal module
   boundaries. The refactor doesn't change the public API, so these tests
   don't move.

3. **Test helpers** shared across multiple test modules get their own
   `#[cfg(test)] pub(crate) mod test_utils` module in the most appropriate
   parent, or use `#[cfg(test)]` helper functions in `mod.rs`.

4. **No orphaned tests**: After extraction, `mujoco_pipeline.rs` should have
   **zero** `#[cfg(test)]` blocks remaining (since all code moved out).

**Verification**:
```bash
# After final phase, this should return nothing:
grep -c '#\[cfg(test)\]' sim/L0/core/src/mujoco_pipeline.rs
# Expected: 0 (file is either deleted or a thin re-export)
```

---

## Phase Completion Checklist

Before committing any phase of the refactor, run through this entire checklist.
Every box must be checked.

### Automated (must pass):

- [ ] `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics`
      — **same pass count as baseline**
- [ ] `cargo clippy -p sim-core -p sim-mjcf -- -D warnings` — **zero warnings**
- [ ] `cargo fmt --all -- --check` — **no formatting issues**
- [ ] `RUSTDOCFLAGS="-D warnings" cargo doc --no-deps -p sim-core -p sim-mjcf`
      — **builds without errors or warnings** (catches broken intra-doc links)

### Structural criteria (grade each, all must be A):

| # | Criterion | Grade | Notes |
|---|-----------|-------|-------|
| S1 | Module Size (≤800 lines prod) | | |
| S2 | Module Naming (predictable) | | |
| S3 | Single Responsibility (one sentence) | | |
| S4 | Dependency Direction (clean DAG) | | |
| S5 | `mod.rs` Clarity (table of contents) | | |
| S6 | Reference Completeness (zero stale) | | |
| S7 | Discoverability (3-click rule) | | |
| S8 | Test Colocation (tests move with code) | | |

### Manual review:

- [ ] Read every new `mod.rs` — does it tell a clear story?
- [ ] Read every new `//!` module doc — is it one sentence, accurate?
- [ ] Spot-check 3 random functions — can you find them by navigation alone?
- [ ] Check that `lib.rs` public API is **identical** to before the phase

---

## Automated Verification Script

This script should be run after every phase. It checks the measurable criteria
automatically.

**Usage**: Save as `sim/scripts/verify_refactor.sh` and run with
`bash sim/scripts/verify_refactor.sh` (requires bash for `globstar`).

```bash
#!/usr/bin/env bash
set -euo pipefail
shopt -s globstar

echo "=== STRUCTURAL REFACTOR VERIFICATION ==="
echo ""

# S1: Module size check
# Pre-existing files exempt from S1 (see "Pre-existing File Exemptions" above)
EXEMPT_FILES=(
    "sim/L0/mjcf/src/types.rs"
    "sim/L0/mjcf/src/parser.rs"
    "sim/L0/core/src/derivatives.rs"
    "sim/L0/core/src/sdf.rs"
    "sim/L0/core/src/mesh.rs"
    "sim/L0/core/src/gjk_epa.rs"
    "sim/L0/core/src/contact.rs"
    "sim/L0/core/src/raycast.rs"
    "sim/L0/core/src/collision_shape.rs"
    "sim/L0/mjcf/src/defaults.rs"
    "sim/L0/core/src/mid_phase.rs"
)
is_exempt() {
    local file="$1"
    for exempt in "${EXEMPT_FILES[@]}"; do
        [ "$file" = "$exempt" ] && return 0
    done
    return 1
}
echo "--- S1: Module Size ---"
OVER_LIMIT=0
for f in sim/L0/core/src/**/*.rs sim/L0/mjcf/src/**/*.rs; do
    [ -f "$f" ] || continue
    is_exempt "$f" && continue
    # Count lines before first #[cfg(test)]
    PROD_LINES=$(awk '/#\[cfg\(test\)\]/{ exit } { count++ } END{ print count+0 }' "$f")
    if [ "$PROD_LINES" -gt 800 ]; then
        echo "  WARN: $f has $PROD_LINES production lines (limit: 800)"
        OVER_LIMIT=$((OVER_LIMIT + 1))
    fi
done
if [ "$OVER_LIMIT" -eq 0 ]; then
    echo "  PASS: All modules ≤800 production lines"
else
    echo "  FAIL: $OVER_LIMIT modules exceed 800 lines"
fi
echo ""

# S2: No forbidden module names
echo "--- S2: Module Naming ---"
BAD_NAMES=$(find sim/L0/core/src sim/L0/mjcf/src -name 'utils.rs' -o -name 'util.rs' \
    -o -name 'helpers.rs' -o -name 'misc.rs' -o -name 'common.rs' \
    -o -name 'pipeline.rs' 2>/dev/null | wc -l)
if [ "$BAD_NAMES" -eq 0 ]; then
    echo "  PASS: No forbidden module names"
else
    echo "  FAIL: Found $BAD_NAMES files with vague names (utils/helpers/misc/common/pipeline)"
fi
echo ""

# S6: Stale reference check
echo "--- S6: Stale References ---"
STALE_RS=$(grep -rn 'mujoco_pipeline\.rs\|model_builder\.rs' sim/ \
    --include='*.rs' 2>/dev/null \
    | grep -v 'STRUCTURAL_REFACTOR\|CHANGELOG\|target/' | wc -l)
STALE_MD=$(grep -rn 'mujoco_pipeline\.rs\|model_builder\.rs' sim/ \
    --include='*.md' 2>/dev/null \
    | grep -v 'STRUCTURAL_REFACTOR\|CHANGELOG' | wc -l)
STALE_TOTAL=$((STALE_RS + STALE_MD))
if [ "$STALE_TOTAL" -eq 0 ]; then
    echo "  PASS: Zero stale references to monolith filenames"
else
    echo "  FAIL: $STALE_TOTAL stale references remain ($STALE_RS in .rs, $STALE_MD in .md)"
    echo "  Run: grep -rn 'mujoco_pipeline\\.rs\\|model_builder\\.rs' sim/ --include='*.rs' --include='*.md'"
fi
echo ""

# S8: No tests left in monolith
echo "--- S8: Test Colocation ---"
if [ -f sim/L0/core/src/mujoco_pipeline.rs ]; then
    TEST_BLOCKS=$(grep -c '#\[cfg(test)\]' sim/L0/core/src/mujoco_pipeline.rs 2>/dev/null || echo "0")
    PROD_LINES=$(awk '/#\[cfg\(test\)\]/{ exit } { count++ } END{ print count+0 }' sim/L0/core/src/mujoco_pipeline.rs)
    if [ "$PROD_LINES" -gt 100 ]; then
        echo "  FAIL: mujoco_pipeline.rs still has $PROD_LINES production lines"
    elif [ "$TEST_BLOCKS" -gt 0 ]; then
        echo "  FAIL: mujoco_pipeline.rs still has $TEST_BLOCKS test blocks"
    else
        echo "  PASS: mujoco_pipeline.rs is a thin re-export shim"
    fi
else
    echo "  PASS: mujoco_pipeline.rs has been removed"
fi
echo ""

# Compiler checks
echo "--- Compiler Checks ---"
echo "Running cargo test..."
cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics 2>&1 | tail -5

echo ""
echo "Running cargo clippy..."
cargo clippy -p sim-core -p sim-mjcf -- -D warnings 2>&1 | tail -3

echo ""
echo "Running cargo doc (strict)..."
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps -p sim-core -p sim-mjcf 2>&1 | tail -5

echo ""
echo "Running cargo fmt check..."
cargo fmt --all -- --check 2>&1 | tail -3

echo ""
echo "=== VERIFICATION COMPLETE ==="
```

---

## MuJoCo C → CortenForge Rust Module Mapping

For reference — this is the correspondence between MuJoCo's C files and our
target module structure. This helps verify S2 (naming) and S3 (single
responsibility) by showing the authoritative decomposition.

| MuJoCo C file | CortenForge module | Scope |
|---------------|-------------------|-------|
| `engine_forward.c` | `forward/mod.rs`, `forward/acceleration.rs`, `forward/check.rs`, `integrate/` | `mj_step`, `mj_forward`, acceleration, state checks, integration |
| `engine_core_smooth.c` | `forward/position.rs`, `forward/velocity.rs`, `dynamics/crba.rs`, `dynamics/rne.rs`, `tendon/`, `forward/actuation.rs` | FK, velocity, CRBA, RNE, tendon kinematics, actuation/transmission |
| `engine_passive.c` | `forward/passive.rs` | Springs, dampers, fluid, flex bending |
| `engine_core_constraint.c` | `constraint/mod.rs`, `constraint/assembly.rs` | Constraint row construction |
| `engine_solver.c` | `constraint/solver/` | PGS, CG, Newton |
| `engine_collision_driver.c` | `collision/mod.rs` | Broad phase + dispatch |
| `engine_collision_convex.c` | `collision/pair_convex.rs`, `collision/pair_cylinder.rs` | Pairwise narrow phase |
| `engine_collision_primitive.c` | `collision/plane.rs`, `collision/narrow.rs` | Plane collisions, narrow-phase dispatch |
| (no 1:1 C file) | `collision/flex_collide.rs`, `collision/mesh_collide.rs`, `collision/hfield.rs`, `collision/sdf_collide.rs` | Dispatch wrappers for flex/mesh/hfield/SDF collisions (route into standalone libraries) |
| `engine_sensor.c` | `sensor/` | All sensor evaluation |
| `engine_island.c` | `island/` | Island discovery, sleeping |
| `engine_derivative.c` | `derivatives.rs` (already separate) | Analytical derivatives |
| `engine_util_sparse.c` | `linalg.rs`, `dynamics/factor.rs` | Sparse solves (`linalg.rs`), sparse LDL factorization + CSR metadata (`dynamics/factor.rs`) |
| `engine_util_spatial.c` | `dynamics/spatial.rs` | Spatial algebra |
| `engine_io.c` | `types/model.rs`, `types/model_init.rs`, `types/data.rs` | Model/Data struct definitions + construction |
| (no 1:1 C file) | `energy.rs`, `joint_visitor.rs` | Energy query (scattered in MuJoCo), joint visitor pattern (Rust-specific abstraction) |

---

## `impl` Block Split Strategy

The audit found one `impl Model` block (16 methods spanning 5+ domains) and
one `impl Data` block (14 methods spanning state management, queries, and
the core pipeline orchestrators). Both must be split across files.

**Rust allows this**: Multiple `impl` blocks for the same type in the same
crate. Each module file can have its own `impl Model { ... }` or
`impl Data { ... }` block containing only the methods that belong to that
module's domain.

**Rules for splitting**:

| Method | Goes in | Rationale |
|--------|---------|-----------|
| `Model` struct def, field accessors, `is_ancestor()`, `joint_qpos0()`, `qld_csr()` | `types/model.rs` | Struct definition + accessors (~780 lines) |
| `Model::empty()`, `make_data()`, `compute_ancestors()`, `compute_stat_meaninertia()` | `types/model_init.rs` | Construction + precomputation (~760 lines) |
| `Model::compute_implicit_params()` | `types/model_init.rs` | Model precomputation (no pipeline deps; called at construction alongside `empty()` and `make_data()`) |
| `Model::n_link_pendulum()`, `double_pendulum()`, etc. | `types/model_factories.rs` | `#[cfg(test)]`-gated test factories (~280 lines) |
| `Model::visit_joints()` | `joint_visitor.rs` | Joint iteration |
| `Model::compute_qld_csr_metadata()` | `dynamics/factor.rs` | Sparse factorization metadata |
| `Model::compute_muscle_params()` | `forward/actuation.rs` | Muscle precomputation |
| `Model::compute_spatial_tendon_length0()` | `tendon/mod.rs` | Tendon precomputation |
| `Data::reset()`, `reset_to_keyframe()` | `types/data.rs` | State management |
| `Data::total_energy()` | `energy.rs` | Energy query |
| `Data::sleep_state()`, `tree_awake()`, etc. | `island/sleep.rs` | Sleep queries |
| `Data::step()` | `forward/mod.rs` | Top-level orchestrator |
| `Data::forward()`, `forward_core()` | `forward/mod.rs` | Pipeline orchestration |
| `Data::integrate()` | `integrate/mod.rs` | Integration dispatch |
| `Data::integrate_without_velocity()` | `integrate/mod.rs` | GPU integration |

**The principle**: A method goes in the module that implements its **primary
computation**. `Data::step()` goes in `forward/mod.rs` because it orchestrates
the forward pipeline, not in `types/data.rs` just because it's on `Data`.

---

## Anti-Patterns to Watch For

These are the traps that turn a "refactor" into "rearranging deck chairs."
If you catch any of these, stop and fix before proceeding.

### 1. The God `mod.rs`
You moved the functions into sub-files but left all the logic in `mod.rs`.
Now `mod.rs` is 2,000 lines and the sub-files are just type definitions.
**Fix**: The `mod.rs` should be the orchestrator, not the implementor.

### 2. The Re-export Maze
You have `pub use submodule::subsubmodule::thing` chains 5 levels deep, and
figuring out where `thing` actually lives requires following a trail of
re-exports. **Fix**: Re-exports should be at most 2 levels deep. If a
symbol is public, it should be findable in at most 2 hops from `lib.rs`.

### 3. The Hidden Dependency
Module A reaches into `crate::collision::pair_convex::internal_helper()`
because it needs one function for a special case. Now `pair_convex.rs` can
never be refactored without checking all callers. **Fix**: If a function is
used outside its module, it goes in the `mod.rs` re-exports.

### 4. The "Utilities" Dumping Ground
You created `linalg.rs` and now everything that doesn't fit anywhere else
goes there. It grows to 2,000 lines. **Fix**: `linalg.rs` contains only
matrix factorization and solve routines (plus UnionFind, which is used by
the sparse solver for island partitioning). Spatial algebra has its own home
in `dynamics/spatial.rs`. If something doesn't fit, it means the module tree
needs another leaf, not that `linalg.rs` needs another function.

### 5. The Cosmetic Module
You created `constraint/solver/mod.rs` that just has `pub mod pgs; pub mod cg; pub mod newton;` and nothing else. The dispatch logic is still in `constraint/mod.rs`. Now reading `constraint/solver/mod.rs` tells you nothing about how solvers are chosen or what they share. **Fix**: The solver dispatch function lives in `constraint/solver/mod.rs`.

### 6. The Copy-Paste Move
You moved code but didn't update the references. Now `derivatives.rs` still
says `use crate::mujoco_pipeline::norm3` and it compiles only because the
re-export shim exists. **Fix**: Update every import to point to the actual
module, not the shim.

---

## Final State Verification

When the entire refactor is complete (all 13 phases (0–8c, 10, 12; 9 and 11 are intentionally
skipped to separate sim-core extraction from sim-mjcf extraction and final cleanup)), this is the definitive
acceptance test:

```
1. cargo test                               → same total pass count as baseline
2. cargo clippy -- -D warnings              → zero warnings
3. cargo fmt --all -- --check               → clean
4. cargo xtask check                        → passes
5. S1 check (module size)                   → all files ≤800 production lines
6. S2 check (naming)                        → zero forbidden names
7. S6 check (stale references)              → zero matches
8. S8 check (monolith remnant)              → mujoco_pipeline.rs ≤100 lines or deleted
9. model_builder.rs                         → ≤100 lines or deleted
10. Read forward/mod.rs                     → tells the complete pipeline story in ~90 lines
11. A newcomer navigate-only test           → 3 out of 3 targets found by directory browsing
12. Every module has //! doc comment        → verified
13. `RUSTDOCFLAGS="-D warnings" cargo doc`  → zero broken intra-doc links
14. Every mod.rs has clear re-exports       → verified
15. ARCHITECTURE.md reflects new structure  → verified
```

**If all 15 checks pass, the refactor is done.**
