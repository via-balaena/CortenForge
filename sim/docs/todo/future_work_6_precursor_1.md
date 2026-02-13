# Future Work 6 Precursor — MuJoCo Alignment (Items #19a–19e, #20–27, #30)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

Every item here is a prerequisite to #19 (MuJoCo Conformance Test Suite): writing
conformance tests against known-wrong or missing code is backwards.

**Section A — Diverged Implementations (#19a–19e):** Places where CortenForge
produces numerically different results from MuJoCo due to architectural shortcuts
taken during earlier phases. Items are ordered by dependency: friction combination
has no dependencies, friction loss requires the constraint assembly infrastructure,
and the PGS/CG unified constraints build on both. Legacy crate cleanup is independent.

**Section B — Missing/Incomplete Features (#20–27, #30):** Features that are parsed
but not wired at runtime, silently ignored, or stubbed. These produce incorrect or
missing results for any model that uses them. All have Medium or higher correctness
impact.

---

### 19a. Friction Combination Rule: Geometric Mean → Element-Wise Max
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

`make_contact_from_geoms()` (`mujoco_pipeline.rs:5690`) combines friction
coefficients from two contacting geoms using **geometric mean**:

```rust
let sliding = (f1.x * f2.x).sqrt();
let torsional = (f1.y * f2.y).sqrt();
let rolling = (f1.z * f2.z).sqrt();
```

The same pattern is used for deformable-rigid contacts (`mujoco_pipeline.rs:19230`).

MuJoCo uses **element-wise maximum** (when geom priorities are equal):

```c
// engine_collision_driver.c — mj_contactParam()
for (int i=0; i < 3; i++) {
    fri[i] = mju_max(friction1[i], friction2[i]);
}
```

This divergence was documented as "known non-conformance" in `future_work_2.md`
(Decision D3) and explicitly deferred: "better done as a deliberate conformance
task with before/after validation, not buried in a condim refactor."

**Impact:** Every contact between geoms with asymmetric friction values produces
wrong friction coefficients. This affects every trajectory comparison test.

#### Objective

Switch friction combination from geometric mean to element-wise max, matching
MuJoCo's `mj_contactParam()` behavior for equal-priority geoms.

#### Specification

1. **`make_contact_from_geoms()`** (`mujoco_pipeline.rs:5690`): Change three lines:
   ```rust
   let sliding = f1.x.max(f2.x);
   let torsional = f1.y.max(f2.y);
   let rolling = f1.z.max(f2.z);
   ```

2. **Deformable-rigid contacts** (`mujoco_pipeline.rs:19230`): Same change pattern.
   Deformable material has a single `friction: f64` combined with rigid geom's
   per-axis friction:
   ```rust
   let sliding = deform_friction.max(rigid_friction.x);
   let torsional = deform_friction.max(rigid_friction.y);
   let rolling = deform_friction.max(rigid_friction.z);
   ```

3. **`<contact><pair>` override**: When a `<pair>` element specifies explicit
   `friction`, it overrides the combination rule entirely — no change needed here.

4. **Geom priority**: MuJoCo uses `geom/@priority` to select which geom's
   friction wins outright (higher priority geom's friction used directly). Our
   parser does not read `priority`. This is out of scope — `priority` is a
   separate feature (future 3B item). For equal-priority geoms (the only case
   we handle), element-wise max is correct.

5. **Doc correction**: `future_work_6.md` line ~1146 claims geometric mean
   "matching MuJoCo's combination rule in `mj_contactForce`" — this is factually
   wrong. Update to state element-wise max.

#### Acceptance Criteria

1. `make_contact_from_geoms()` uses `f1.max(f2)` per component.
2. Deformable-rigid contact friction uses `max()` combination.
3. Existing contact tests updated — any test that relied on geometric mean
   behavior must be re-validated. Expected: tolerance tightening, not loosening.
4. New test: two geoms with `friction="0.3"` and `friction="0.7"` produce
   contact with `sliding = 0.7` (not `sqrt(0.21) ≈ 0.458`).
5. `future_work_6.md` D7 friction claim corrected. (Already done in spec
   refactoring — verify the correction persists.)

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `make_contact_from_geoms()`, deformable contacts
- `sim/L0/tests/integration/` — contact friction tests
- `sim/docs/todo/future_work_6.md` — D7 correction

---

### 19b. Friction Loss Migration: Tanh Passive Force → Huber Constraint Rows
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State

CortenForge computes friction loss as a **passive force** in `mj_fwd_passive()`:

```rust
// mujoco_pipeline.rs:10922
let smooth_sign = (qvel * self.model.friction_smoothing).tanh();
let fl_force = frictionloss * smooth_sign;
self.data.qfrc_passive[dof_idx] -= fl_force;
self.data.qfrc_frictionloss[dof_idx] -= fl_force;
```

MuJoCo treats friction loss as **constraint rows** (`mjCNSTR_FRICTION_DOF`,
`mjCNSTR_FRICTION_TENDON`) with Huber cost in the constraint solver. It is
**never** part of `qfrc_passive` in MuJoCo.

The infrastructure for migration already exists:
- `Data.qfrc_frictionloss: DVector<f64>` — separate accumulator (approach (b)
  from §15, already implemented)
- Newton unified constraint assembly (`assemble_unified_constraints()`) already
  has the Jacobian structure and Huber state classification
- `ConstraintType::FrictionLoss` with states `Quadratic`, `LinearPos`,
  `LinearNeg` defined in §15

What's missing: the actual constraint row assembly for friction loss is not
wired into `assemble_unified_constraints()`, and the tanh path is still active
for the Newton solver.

**Impact:** Every model with `frictionloss > 0` produces different dynamics from
MuJoCo. The force profile near zero velocity differs qualitatively (smooth tanh
vs Huber kink).

#### Objective

Implement friction loss as Huber-cost constraint rows in the unified constraint
system, matching MuJoCo's `mjCNSTR_FRICTION_DOF` / `mjCNSTR_FRICTION_TENDON`.

#### Specification

##### Row assembly

In `assemble_unified_constraints()`, after equality constraint rows and before
joint limit rows, emit friction loss rows:

1. **DOF friction loss**: For each DOF with `model.dof_frictionloss[i] > 0`:
   - Emit 1 row with Jacobian: 1×nv sparse, `J[row, i] = 1.0`
   - `efc_type[row] = ConstraintType::FrictionLoss`
   - `efc_pos[row] = 0.0` (friction loss has no position error)
   - `efc_floss[row] = model.dof_frictionloss[i]`
   - `solref` / `solimp`: from the joint's solver parameters (or model defaults)
   - `aref[row] = -B * efc_vel - K * imp * 0 = -B * efc_vel`
     (position term is zero; only velocity damping matters)

2. **Tendon friction loss**: For each tendon with `model.tendon_frictionloss[t] > 0`:
   - Emit 1 row with Jacobian: 1×nv dense, `J[row, :] = ten_J[t, :]`
   - Same metadata as DOF friction, but `efc_floss = tendon_frictionloss[t]`

##### Huber cost classification

Friction loss rows use the three-state Huber classification (from §15):

```
let threshold = R_i * floss_i;  // R_i = regularization, floss_i = friction loss value
if jar_i.abs() <= threshold {
    state = Quadratic;          // smooth quadratic zone near zero
} else if jar_i > threshold {
    state = LinearPos;          // linear positive zone
} else {
    state = LinearNeg;          // linear negative zone
}
```

This produces dry-friction-like behavior: quadratic resistance near zero
velocity, constant force at high velocity — matching MuJoCo's Huber cost
semantics.

##### Newton path changes

- `mj_fwd_passive()`: **Keep** friction loss in `qfrc_passive` (for PGS/CG
  fallback compatibility). Continue populating `qfrc_frictionloss` separately.
- `newton_solve()`: Subtract `qfrc_frictionloss` from RHS (already implemented).
  Friction loss is now handled by the constraint rows in J.
- The existing `data.newton_solved` branch in force balance already accounts for
  this (implemented in previous §19 spec work).

##### PGS/CG path changes

For this item, PGS/CG continues using the tanh passive force path unchanged.
The friction loss constraint rows are only active when Newton is the solver.
PGS/CG migration to unified constraints is item #19d.

##### Removal scope

- Do NOT remove the tanh computation from `mj_fwd_passive()` yet — PGS/CG
  still needs it until #19d.
- Do NOT remove `friction_smoothing` from Model — it becomes unused after #19d
  but removing it is a breaking API change for later.

#### Acceptance Criteria

1. `assemble_unified_constraints()` emits friction loss rows for all DOFs/tendons
   with `frictionloss > 0`.
2. Newton solver produces Huber-cost friction loss forces (not tanh).
3. `efc_force` contains friction loss constraint forces after Newton solve.
4. Force balance identity holds: `M * qacc == rhs` where RHS excludes
   `qfrc_frictionloss` (Newton path).
5. Regression: PGS/CG path unchanged — `qfrc_passive` still includes tanh
   friction loss.
6. New test: single hinge with `frictionloss=1.0`, Newton solver. Compare
   `efc_force` friction loss component against MuJoCo 3.4.0 reference.
7. Existing `test_frictionloss_scaling` updated to reflect Newton path behavior.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `assemble_unified_constraints()`,
  friction loss row emission
- `sim/L0/tests/integration/passive_forces.rs` — update friction loss tests
- `sim/L0/tests/integration/newton_solver.rs` — Newton friction loss validation

#### Origin

Migrated from scattered notes in:
- `future_work_5.md` §15 "Friction loss migration" (lines 130–166) — design
  context, approach (a)/(b), compatibility notes
- `future_work_6.md` §19 B4 "Frictionloss divergence" (lines 720–734) —
  conformance test deferral note

---

### 19c. Friction Loss Migration: PGS/CG Passive Force → Constraint Rows
**Status:** Not started | **Effort:** M | **Prerequisites:** #19b

#### Current State

After #19b, friction loss is handled correctly for the Newton solver path via
Huber constraint rows. However, PGS/CG still uses the tanh passive force
approximation in `mj_fwd_passive()`. MuJoCo handles friction loss as constraint
rows for ALL solver types — not just Newton.

#### Objective

Extend friction loss constraint rows to the PGS/CG solver path, eliminating the
tanh passive force approximation entirely.

#### Specification

1. **Constraint assembly**: Friction loss rows (from #19b) must also be emitted
   when PGS/CG is the solver. Currently `assemble_unified_constraints()` is only
   called for Newton — either extend its use to PGS/CG, or add friction loss row
   assembly to the PGS/CG contact-only path.

2. **PGS solver**: PGS must handle friction loss rows. These are single-row
   constraints with Huber cost — PGS projects each row independently. The
   projection for Huber cost:
   ```
   if state == Quadratic:  f = clamp(f_unclamped, -threshold, +threshold)
   if state == LinearPos:  f = max(f_unclamped, 0)   // or threshold boundary
   if state == LinearNeg:  f = min(f_unclamped, 0)
   ```
   (Exact projection depends on MuJoCo's PGS implementation — verify against
   `engine_solver.c:mj_solPGS`.)

3. **CG solver**: Same treatment — friction loss rows participate in the CG
   solve alongside contact rows.

4. **Remove tanh path**: Once PGS/CG handles friction loss as constraint rows:
   - Remove the friction loss computation from `mj_fwd_passive()` entirely
   - `qfrc_passive` no longer includes friction loss for ANY solver
   - `qfrc_frictionloss` accumulator can be removed (or repurposed for
     diagnostics — extract from `efc_force` instead)
   - `friction_smoothing` on Model becomes dead — deprecate

5. **Force balance update**: With friction loss removed from `qfrc_passive`,
   the force balance identity simplifies for all solver paths:
   `M * qacc == qfrc_applied + qfrc_actuator + qfrc_passive + qfrc_constraint - qfrc_bias`
   No more `qfrc_frictionloss` subtraction needed (friction loss is now in
   `qfrc_constraint` via the solver).

#### Acceptance Criteria

1. PGS solver handles friction loss constraint rows.
2. CG solver handles friction loss constraint rows.
3. `mj_fwd_passive()` no longer computes friction loss.
4. `qfrc_passive` matches MuJoCo's `qfrc_passive` (no friction loss component).
5. Force balance identity is uniform across all solver types.
6. `friction_smoothing` field deprecated on Model.
7. PGS friction loss forces match MuJoCo 3.4.0 reference within solver tolerance.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `mj_fwd_passive()`, `mj_fwd_constraint()`,
  PGS/CG solver paths
- `sim/L0/tests/integration/passive_forces.rs` — remove tanh-specific tests
- `sim/L0/tests/integration/newton_solver.rs` — verify uniform behavior

#### Origin

Continuation of #19b. Completes the friction loss migration started in
`future_work_5.md` §15 "Compatibility" section (lines 162–166).

---

### 19d. PGS/CG Unified Constraints: Penalty Limits + Equality → Solver Rows
**Status:** Not started | **Effort:** L | **Prerequisites:** #19c

#### Current State

The Newton solver path handles ALL constraint types (equality, limits, contacts)
through the unified constraint assembly (`assemble_unified_constraints()`). The
PGS/CG path uses a different architecture:

- **Contacts**: Go through the PGS/CG solver (correct)
- **Joint limits**: Penalty forces with Baumgarte stabilization
  (`mujoco_pipeline.rs:18423–18473`). Comment: "MuJoCo uses solver-based
  approach, but penalty is acceptable for most robotics."
- **Tendon limits**: Same penalty approach (`mujoco_pipeline.rs:18475–18526`)
- **Equality constraints**: Penalty forces via `apply_equality_constraints()`
  (`mujoco_pipeline.rs:18528–18536`). Comment: "Using penalty method with
  Baumgarte stabilization (like joint limits). MuJoCo uses solver-based approach
  via PGS, but penalty is robust and simpler."

MuJoCo passes ALL constraint types through the solver for ALL solver types.
The penalty approach produces different force magnitudes, stability
characteristics, and impedance scaling.

Related divergences that this item eliminates:
- Default penalty parameters 4× stiffer than MuJoCo (`default_eq_stiffness:
  10000.0` vs MuJoCo's `solref=[0.02, 1.0]` → k≈2500)
- Impedance consumption formula differs: penalty scales both position and
  velocity terms by impedance; MuJoCo KBIP applies impedance only to position

#### Objective

Route joint limits, tendon limits, and equality constraints through the PGS/CG
solver as constraint rows, matching MuJoCo's architecture for all solver types.

#### Specification

1. **Extend constraint assembly to PGS/CG**: Reuse
   `assemble_unified_constraints()` (or a shared subset) to emit equality, limit,
   and contact rows into the unified Jacobian when PGS/CG is the solver.

2. **PGS projection rules** (per constraint type):
   - **Equality**: Unclamped (no projection needed — always Quadratic)
   - **Joint/tendon limits**: Unilateral clamp: `f = max(f, 0)` (force must push
     away from limit, never pull)
   - **Contacts**: Existing cone projection (unchanged)
   - **Friction loss**: Huber projection (from #19c)

3. **CG solver**: Same unified system. CG inherently handles mixed constraint
   types through the KKT system.

4. **Remove penalty code paths**:
   - Remove `solref_to_penalty()` and all callers
   - Remove `apply_equality_constraints()` penalty implementation
   - Remove `apply_connect_constraint()`, `apply_weld_constraint()`,
     `apply_joint_equality_constraint()` penalty implementations
   - Remove penalty limit code at lines 18423–18526
   - Remove `default_eq_stiffness`, `default_eq_damping` from Model

5. **Impedance**: Switch from penalty-style `F = -imp * k * pos - imp * b * vel`
   to MuJoCo KBIP: `aref = -B * vel - K * imp * pos`. This is already
   implemented in the Newton path's `compute_aref()` — reuse it.

6. **solref/solimp**: All constraints now use the standard solref/solimp
   parameterization through the solver (no more `solref_to_penalty()` conversion
   layer). Direct mode (`solref[0] ≤ 0`) works for all constraint types.

#### Acceptance Criteria

1. PGS/CG solver handles equality, limit, and contact constraints through
   unified row assembly.
2. `solref_to_penalty()` and all penalty force code removed.
3. `apply_equality_constraints()` penalty path removed.
4. Joint limit forces match MuJoCo 3.4.0 PGS reference for `T13_joint_limits`
   model.
5. Weld constraint forces match MuJoCo 3.4.0 PGS reference for
   `T14_equality_weld` model.
6. `default_eq_stiffness` / `default_eq_damping` removed from Model.
7. Force balance identity uniform across all solver types and constraint types.
8. No regressions on Newton path (already unified).

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `mj_fwd_constraint()` PGS/CG path,
  `solref_to_penalty()`, equality/limit penalty code
- `sim/L0/tests/integration/equality_constraints.rs` — update tests
- `sim/L0/tests/integration/passive_forces.rs` — limit force tests

#### Origin

This divergence was acknowledged in code comments at `mujoco_pipeline.rs:18428`
and `18530` but never tracked as a work item.

---

### 19e. Legacy Crate Deprecation
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

Four standalone crates duplicate functionality that is fully implemented in the
pipeline (`sim-core`):

| Crate | Lines | Pipeline equivalent | Used by pipeline? |
|-------|-------|--------------------|--------------------|
| `sim-constraint` | 10,359 | PGS/CG/Newton in `mujoco_pipeline.rs` | **No** |
| `sim-muscle` | 2,550 | MuJoCo FLV in `mj_fwd_actuation()` | **No** |
| `sim-tendon` | 3,919 | `mj_fwd_tendon_fixed/spatial()` | **No** |
| `sim-sensor` | — | 32 sensor types in `mj_sensor_*()` | **No** |

These crates are re-exported by `sim-physics` (the umbrella crate) but have zero
callers in the pipeline. They confuse contributors ("which PGS?"), inflate
compile times, and create a false impression of coverage.

This item was previously tracked as #34 in Phase 3D. It is being moved here
because legacy crate confusion is a correctness hazard for #19: test authors
might accidentally test the wrong implementation.

#### Objective

Deprecate standalone crates that are fully superseded by pipeline implementations.

#### Specification

(Specification carried over from former #34 — see `future_work_9.md`)

1. **sim-tendon**: Mark as `#[deprecated]`. Add top-level doc comment directing
   users to pipeline `mj_fwd_tendon_*()`. Remove from default workspace members.
2. **sim-muscle**: Mark as `#[deprecated]`. Note the standalone Hill model is
   richer than pipeline FLV but not MuJoCo-compatible. Keep available for
   biomechanics users who don't need MuJoCo conformance.
3. **sim-constraint**: Keep crate but audit public API. Remove re-exports of
   deleted types (PGSSolver, NewtonSolver, etc.). Document which types are
   standalone-only vs pipeline-compatible.
4. **sim-sensor**: Keep — provides standalone hardware sensor API independent of
   pipeline.

#### Acceptance Criteria

1. Deprecated crates emit compiler warnings on use.
2. `cargo doc` shows clear deprecation notices with migration guidance.
3. No change to pipeline behavior (regression).
4. Workspace compiles cleanly (no dead-code warnings in deprecated crates).

#### Files
- `sim/L0/tendon/src/lib.rs` — deprecation attributes
- `sim/L0/muscle/src/lib.rs` — deprecation attributes
- `sim/L0/constraint/src/lib.rs` — API audit
- `Cargo.toml` (workspace) — optional default-members adjustment

#### Origin

Migrated from #34 in `future_work_9.md`.

---

### 20. Contact Margin/Gap Runtime Effect
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

`geom_margin` and `geom_gap` are stored on `Model` as `Vec<f64>` (one per geom,
initialized to 0.0). `ContactPair` has `margin` and `gap` fields, correctly parsed
from `<contact><pair>` MJCF elements. However, margin and gap have **no runtime
effect** on contact activation or constraint assembly:

- **Narrow-phase**: All collision functions (e.g., `collide_sphere_sphere`) generate
  contacts only when `penetration > 0.0`. Margin is not checked.
  (`mujoco_pipeline.rs`, `collide_sphere_sphere`, line ~6509.)
- **Constraint assembly**: `margin` is hardcoded to `0.0` in `assemble_contact_system`
  (`mujoco_pipeline.rs`, line ~14600: `let margin = 0.0_f64;`). The `compute_aref()`
  function accepts a `margin` parameter but always receives 0.0.
- **`apply_pair_overrides`**: Applies condim, friction, solref, solimp from
  `ContactPair` but skips margin/gap. Comment at line ~5664: "margin/gap are NOT
  applied here."
- **Broad-phase**: `pair.margin` IS used in the bounding-sphere distance cull for
  explicit pairs (line ~5431). This is correct and should be preserved.
- **Geom-level parsing**: `<geom margin="..." gap="...">` attributes are NOT parsed.
  `model_builder.rs` comment: "geom-level not yet parsed, default to 0.0."
- **`efc_margin`**: The `Data` struct has an `efc_margin: Vec<f64>` field (per
  constraint row), currently always populated with 0.0.

MuJoCo's margin/gap system involves three distinct locations: contact detection
(wider activation), constraint violation computation (shifted reference), and the
`aref` stabilization formula. All three must be wired up.

#### Objective

Wire `margin` and `gap` into the full pipeline — MJCF parsing, contact detection,
and constraint assembly — so that contacts activate at the correct distance and
Baumgarte stabilization uses the correct reference point.

#### Specification

##### S1. Margin/gap combination rule (MuJoCo semantics)

MuJoCo **sums** margins and gaps from both geoms (source:
`engine_collision_driver.c`, `mj_collideGeoms`):

```
effective_margin = geom_margin[g1] + geom_margin[g2]
effective_gap    = geom_gap[g1]    + geom_gap[g2]
includemargin    = effective_margin - effective_gap
```

For explicit `<contact><pair>` contacts, the pair's `margin` and `gap` attributes
**override** the geom-summed values entirely (pair takes precedence).

The `includemargin` value is the distance threshold below which a contact generates
a constraint. It is stored on the `Contact` struct and propagated to `efc_margin`
during constraint assembly.

##### S2. Geom-level MJCF parsing

Parse `margin` and `gap` attributes from `<geom>` elements. These cascade through
the defaults system (existing infrastructure handles attribute cascading):

```xml
<geom type="sphere" size="0.1" margin="0.002" gap="0.001"/>
```

Default values: `margin = 0.0`, `gap = 0.0` (preserves current behavior when
attributes are absent).

Update `Model.geom_margin` and `Model.geom_gap` arrays, which are already
allocated (currently zeroed). Wire through `model_builder.rs` → `process_geom()`.

##### S3. Narrow-phase contact activation

Change the activation condition in all narrow-phase collision functions from:

```rust
if penetration > 0.0 { ... }  // current
```

to:

```rust
if penetration > -margin { ... }  // with margin
```

where `margin` is the effective margin computed per S1. When a contact is generated
with `penetration ∈ (-margin, 0]`, the contact's `depth` field stores the
actual geometric penetration (which is negative — surfaces are separated). The
`includemargin` field on the Contact stores the margin value.

This means the contact exists in the constraint system but may produce zero force
(if `depth > includemargin`, the constraint violation `r > 0` so no corrective
force is applied). This is MuJoCo's mechanism for "pre-contact" — smooth force
onset rather than sudden activation.

Affected functions: `collide_sphere_sphere`, `collide_sphere_plane`,
`collide_capsule_plane`, `collide_box_plane`, `collide_sphere_capsule`,
and all other `collide_*` variants. Each function must receive the effective
margin for the geom pair.

Implementation approach: pass `margin` as a parameter to each `collide_*`
function. Compute `margin` in the dispatch function (`collide_geoms` or explicit
pair loop) using S1, and thread it through.

##### S4. Contact struct: store `includemargin`

The `Contact` struct already has a `pub includemargin: bool` field (currently
always `false`). Replace this with a numeric field:

```rust
/// Distance threshold used for constraint activation.
/// `includemargin = effective_margin - effective_gap`.
/// Constraint violation: `r = depth - includemargin`.
/// Contact exists when `depth > -effective_margin` (see S3).
pub includemargin: f64,
```

Set by `make_contact_from_geoms()` or `apply_pair_overrides()`.

##### S5. `apply_pair_overrides`: apply margin/gap from ContactPair

In `apply_pair_overrides()`, set `contact.includemargin = pair.margin - pair.gap`
when the contact comes from an explicit `<contact><pair>`. Remove the existing
comment "margin/gap are NOT applied here."

For automatic (Mechanism 1) contacts, set
`contact.includemargin = (geom_margin[g1] + geom_margin[g2]) - (geom_gap[g1] + geom_gap[g2])`
in `make_contact_from_geoms()`.

##### S6. Constraint assembly: populate `efc_margin`

In `assemble_contact_system` (line ~14598), replace the hardcoded
`let margin = 0.0_f64` with:

```rust
let margin = contact.includemargin;
```

This value flows into:
1. `data.efc_margin[row] = margin` — per constraint row (normal direction only;
   friction rows get `efc_margin = 0.0`, matching MuJoCo).
2. `compute_aref(k, b, imp, pos, margin, vel)` — the existing function already
   computes `aref = -b * vel - k * imp * (pos - margin)`. With nonzero margin,
   the constraint equilibrium shifts to `pos = margin` instead of `pos = 0`.

No changes to `compute_aref()` itself — the function is already correct; it just
needs a nonzero margin input.

##### S7. Interaction with global `<option>` override

MuJoCo supports `<option o_margin="...">` which overrides all per-geom and
per-pair margins globally. This is a low-priority extension — defer unless existing
models require it. Note in code with a TODO.

#### Acceptance Criteria

1. **Margin activation**: A sphere at height 1mm above a plane with
   `geom_margin="0.002"` (each geom) generates a contact. Effective margin =
   `0.002 + 0.002 = 0.004 > 0.001` (distance). Contact `depth ≈ -0.001`
   (negative, surfaces separated), `includemargin = 0.004`, constraint
   `r = -0.001 - 0.004 = -0.005 < 0` → constraint force engages.
2. **Gap buffer**: With `margin="0.004"` and `gap="0.003"`, `includemargin = 0.001`.
   A sphere at distance 0.002 above the plane generates a contact (within margin)
   but `r = -0.002 - 0.001 = -0.003 < 0` → force engages. At distance 0.0005,
   still in contact, stronger force. Verify force magnitude scales with
   `|r|` (deeper violation → stronger force).
3. **Pair override**: An explicit `<contact><pair margin="0.01" gap="0.005">`
   uses `includemargin = 0.005`, ignoring geom-level margins.
4. **Zero margin/gap regression**: All existing tests pass unchanged (default
   margin/gap = 0.0 preserves `penetration > 0.0` activation and
   `efc_margin = 0.0` in assembly).
5. **MuJoCo reference**: Compare contact count and forces against MuJoCo 3.4.0
   for a sphere-on-plane test at 5 separation distances spanning
   `[-0.002, +0.003]` with `margin=0.004, gap=0.001`. Contact count and
   `qfrc_constraint` must match within tolerance `1e-8`.
6. **`efc_margin` populated**: After `forward()`, `data.efc_margin[normal_row]`
   equals `contact.includemargin` for every active contact.

#### Implementation Notes

**Narrow-phase function signature change.** Every `collide_*` function currently
takes `(model, geom1, geom2, pos1, pos2, size1, size2) -> Option<Contact>`. Add
a `margin: f64` parameter. The dispatch function computes the effective margin
(S1) before calling the specific collider.

**Existing broad-phase is correct.** The bounding-sphere cull at line ~5431
already adds `pair.margin` to the distance check. For automatic contacts, add
`geom_margin[g1] + geom_margin[g2]` to the broad-phase distance threshold.

**Breaking change to Contact struct.** Replacing `includemargin: bool` with
`includemargin: f64` is a breaking change. Any existing code that checks
`contact.includemargin` as a bool must be updated. Search for all uses.

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — all `collide_*` functions,
  `make_contact_from_geoms()`, `apply_pair_overrides()`,
  `assemble_contact_system()` (margin wiring), broad-phase distance check
- `sim/L0/mjcf/src/parser.rs` — parse `margin`/`gap` from `<geom>` elements
- `sim/L0/mjcf/src/model_builder.rs` — wire geom-level margin/gap into
  `Model.geom_margin`/`Model.geom_gap` in `process_geom()`
- `sim/L0/mjcf/src/defaults.rs` — ensure margin/gap cascade through defaults
- `sim/L0/tests/integration/` — new test file `contact_margin_gap.rs`

#### Origin

Migrated from `future_work_6.md` #20.

---

### 21. Noslip Post-Processor for PGS/CG Solvers
**Status:** Newton done, PGS/CG missing | **Effort:** S | **Prerequisites:** None

#### Current State

The noslip post-processor is **fully implemented for the Newton solver** as
`noslip_postprocess()` (`mujoco_pipeline.rs`, lines ~16521–16694, ~170 LOC).
Three integration tests verify it (`newton_solver.rs`, lines 656–777):
- `test_noslip_zero_iterations_is_noop` — regression test
- `test_noslip_produces_finite_results` — basic sanity
- `test_noslip_reduces_slip` — quantitative slip reduction on box-on-slope

The Newton implementation:
1. Identifies friction rows from `efc_type`/`efc_dim` (contact rows with `dim ≥ 3`,
   skipping the normal row 0).
2. Builds a friction-only Delassus submatrix `A_fric` with **no regularizer** on
   the diagonal (`1/A[i,i]` instead of `1/(A[i,i] + R[i])`).
3. Runs PGS iterations on friction rows with elliptic cone projection.
4. Writes back updated `efc_force` and recomputes `qfrc_constraint` and `qacc`.

**What's missing:** The PGS and CG solver paths bypass noslip entirely. The solver
dispatch at line ~18364 calls `noslip_postprocess()` only after
`NewtonResult::Converged`. The PGS/CG code path (which runs when Newton is not
selected, or Newton falls through) does not invoke noslip. This means models using
`solver="PGS"` or `solver="CG"` with `noslip_iterations > 0` silently ignore the
noslip setting.

MJCF parsing is complete: `noslip_iterations` and `noslip_tolerance` are read from
`<option>` and stored on `Model` (defaults: 0 iterations, 1e-6 tolerance).

#### Objective

Wire `noslip_postprocess()` into the PGS and CG solver paths so all three solver
types support noslip when `noslip_iterations > 0`.

#### Specification

##### S1. Refactor `noslip_postprocess()` to be solver-agnostic

The existing `noslip_postprocess()` function is already solver-agnostic internally —
it operates on `efc_force`, `efc_type`, `efc_dim`, and the Jacobians, none of which
are Newton-specific. The only coupling is the **call site**.

Verify that the function does not access any Newton-specific state (it should not).
If it does, factor out any Newton dependencies.

##### S2. Add noslip call after PGS/CG solve

In `mj_fwd_constraint()`, after the PGS or CG solver finishes (the code path that
handles penalty contacts + iterative solve), add:

```rust
// After PGS/CG solve completes and efc_force is populated:
if model.noslip_iterations > 0 {
    noslip_postprocess(model, data);
}
```

The noslip function already handles the `qacc` recomputation internally
(`qfrc_constraint = J^T · efc_force`, then `qacc = M^{-1} · (qfrc_smooth +
qfrc_constraint)`). No additional post-processing needed.

##### S3. Constraint row handling

The existing noslip implementation processes **contact friction rows only** (rows
where `efc_type == ContactElliptic || ContactNonElliptic` and `dim ≥ 3`, skipping
the normal row). MuJoCo's `mj_solNoSlip` also processes:
- Equality constraint rows (indices `0..ne`)
- Dry friction rows (indices `ne..ne+nf`) with interval clamping `[-floss, +floss]`

These are currently absent from our `noslip_postprocess()`. For PGS/CG conformance:
- **Phase 1 (this spec):** Keep current behavior — contact friction rows only.
  This covers the primary use case (suppress tangential slip in contact).
- **Phase 2 (stretch):** Add equality and dry friction row processing. Mark with
  TODO in code.

##### S4. Cone projection modes

The existing implementation uses elliptic cone projection. MuJoCo supports both
pyramidal and elliptic cones in noslip. Since CortenForge uses elliptic cones
exclusively (the `model.cone` field controls this, and pyramidal is currently only
a fallthrough from Newton), the elliptic projection is sufficient for now.

If pyramidal cone support is added later, `noslip_postprocess()` must dispatch to
the correct projection based on `model.cone`.

#### Acceptance Criteria

1. **PGS + noslip**: A box on a 20° incline with `solver="PGS"` and
   `noslip_iterations=10` shows reduced tangential slip compared to
   `noslip_iterations=0`. Measure `qvel` tangential component after 100 steps.
2. **CG + noslip**: Same benchmark with `solver="CG"` — noslip reduces slip.
3. **Newton regression**: Existing Newton noslip tests continue to pass
   (no behavior change for Newton path).
4. **`noslip_iterations=0` is no-op**: For all solver types, zero iterations
   produces identical output to the no-noslip code path.
5. **Normal forces unchanged**: After noslip, contact normal force components
   (`efc_force[normal_row]`) are identical to pre-noslip values (bit-exact).
6. **MuJoCo reference**: Compare `efc_force` after noslip against MuJoCo 3.4.0
   for a box-on-slope benchmark with `solver="PGS" noslip_iterations=10`.
   Tolerance: `1e-4` (PGS tolerance, same as main solve).

#### Implementation Notes

**Minimal diff.** The implementation is a 2–5 line change at the PGS/CG solver
call site. The `noslip_postprocess()` function is already written and tested.
The primary risk is verifying it works correctly with PGS/CG constraint assembly
output (which uses a slightly different `efc_*` layout than Newton).

**Verify `efc_type`/`efc_dim` layout.** The Newton path and PGS path may
populate `efc_type` and `efc_dim` differently (Newton uses
`assemble_unified_constraints`, PGS uses the penalty + PGS code path). Verify
that `noslip_postprocess()` correctly identifies friction rows in both layouts.

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — add noslip call after PGS/CG solve
  (2–5 lines); verify `noslip_postprocess()` compatibility with PGS efc layout
- `sim/L0/tests/integration/` — new tests for PGS+noslip, CG+noslip in existing
  `newton_solver.rs` (rename to `constraint_solver.rs`?) or new file

#### Origin

Migrated from `future_work_6.md` #21.

---

### 22. Body-Transmission Actuators (Adhesion)
**Status:** Parsing done, transmission missing | **Effort:** M | **Prerequisites:** None

#### Current State

**What exists:**
- **MJCF parsing**: `<actuator><adhesion>` is fully parsed. `MjcfActuatorType::Adhesion`
  sets `gaintype = Fixed` (with user-specified gain), `biastype = None`,
  `dyntype = None`, `ctrllimited = true`. The `body` attribute is parsed and stored
  as `actuator.body: Option<String>`. (`parser.rs`, lines ~1685–1689;
  `model_builder.rs`, lines ~1972–1976.)
- **Force generation**: Adhesion uses standard gain/bias: `force = gain * ctrl`.
  Clamped by `forcerange`. No activation dynamics. This already works through the
  standard `mj_fwd_actuation()` Phase 2 force computation.
- **Error on body transmission**: `model_builder.rs` line ~1794 returns
  `ModelConversionError` when `actuator.body` is `Some(...)`: "Actuator '...' uses
  body transmission '...' which is not yet supported." This is the **only** blocker.

**What's missing:**
- `ActuatorTransmission::Body(usize)` variant in the enum (currently: Joint/Tendon/Site).
- `mj_transmission_body()` — computes moment arm from contact normal Jacobians.
- Force application via `qfrc_actuator += moment_arm * force` in Phase 3 of
  `mj_fwd_actuation()`.

**Existing transmission patterns** (for reference):
- **Joint**: `qfrc_actuator[dof] += gear * force` (direct DOF application).
- **Tendon**: `apply_tendon_force()` with cached `ten_J` Jacobian.
- **Site**: `qfrc_actuator[dof] += actuator_moment[i][dof] * force` (pre-computed
  moment Jacobian from `mj_transmission_site()`).

Body transmission follows the **Site pattern** — pre-compute a moment arm vector
in a transmission function, then apply it in Phase 3.

#### Objective

Implement body transmission (`ActuatorTransmission::Body`) so adhesion actuators
load and function. The adhesion force pulls/pushes the target body toward/away from
surfaces via contact normal Jacobians.

#### Specification

##### S1. `ActuatorTransmission::Body` enum variant

Add a new variant to the transmission enum:

```rust
pub enum ActuatorTransmission {
    Joint,
    Tendon,
    Site,
    /// Body transmission — force applied via average of contact normal Jacobians.
    /// The usize is the target body index.
    Body(usize),
}
```

##### S2. `mj_transmission_body()` — moment arm computation

**Algorithm** (matches MuJoCo `engine_core_smooth.c`):

```rust
fn mj_transmission_body(
    model: &Model,
    data: &Data,
    actuator_id: usize,
) -> DVector<f64> {
    let body_id = match model.actuator_transmission[actuator_id] {
        ActuatorTransmission::Body(id) => id,
        _ => unreachable!(),
    };
    let gear = model.actuator_gear[actuator_id];
    let mut moment = DVector::zeros(model.nv);
    let mut count = 0usize;

    for c in 0..data.ncon {
        let contact = &data.contacts[c];
        let body1 = model.geom_body[contact.geom1];
        let body2 = model.geom_body[contact.geom2];

        if body1 != body_id && body2 != body_id {
            continue;
        }

        // Compute contact Jacobian difference (J_body1 - J_body2)
        // projected along contact normal (first axis of contact frame).
        // This gives the normal-direction relative velocity Jacobian.
        let j_normal = compute_contact_normal_jacobian(
            model, data, contact,
        );

        // Accumulate (sign convention: negative = attractive)
        moment += &j_normal;
        count += 1;
    }

    if count > 0 {
        // Average and negate: positive ctrl → attractive force
        moment *= -gear / (count as f64);
    }

    moment
}
```

The `compute_contact_normal_jacobian()` helper computes `J_diff · n` where
`J_diff = J(body1, contact_pos) - J(body2, contact_pos)` is the relative
velocity Jacobian at the contact point, and `n` is the contact normal.
This is the same Jacobian row used for the normal constraint direction.

**Key detail — negative sign:** The `-1/count` scaling means positive actuator
force produces a generalized force that pulls the body *toward* the surface
(attractive adhesion). This is MuJoCo's convention. The constraint solver
still clamps normal force ≥ 0 — adhesion does NOT allow negative constraint
forces. Instead, the adhesion generalized force in `qfrc_actuator` counteracts
gravity and other forces, keeping the body in contact.

##### S3. Call site in `mj_fwd_actuation()`

Call `mj_transmission_body()` during the **transmission computation phase** of
`mj_fwd_actuation()`, before the force application loop. Store the result in
`data.actuator_moment[actuator_id]`.

Then in Phase 3 (force application), handle the Body case like Site:

```rust
ActuatorTransmission::Body(_) => {
    for dof in 0..model.nv {
        let m = data.actuator_moment[i][dof];
        if m != 0.0 {
            data.qfrc_actuator[dof] += m * force;
        }
    }
}
```

Note: `actuator_length[i]` for body transmission is always 0 (adhesion has no
length, unlike tendons or joints).

##### S4. Model builder: remove error, resolve body name to ID

In `model_builder.rs`, replace the error at line ~1794 with body name resolution:

```rust
} else if let Some(ref body_name) = actuator.body {
    let body_id = self.resolve_body_id(body_name)?;
    transmission = ActuatorTransmission::Body(body_id);
    trnid = body_id;
}
```

##### S5. Interaction with `forcerange`

The adhesion model builder already sets up `forcerange` correctly (lines
~1876–1892). The scalar force `gain * ctrl` is clamped to `forcerange` in Phase 2
of `mj_fwd_actuation()` (line ~9975). No changes needed — the existing clamp
happens before the force is mapped through the moment arm.

The default adhesion setup uses `ctrl ∈ [0, 1]` with positive gain, so the scalar
force is always non-negative. The negative sign in the moment arm (S2) makes the
generalized force attractive.

##### S6. Contact count edge cases

- **No contacts**: If the target body has no active contacts, `moment` is zero,
  and the actuator produces no force. This is correct — adhesion requires contact.
- **Multiple contacts**: Forces are averaged across all contacts involving the body.
  This prevents the adhesion force from scaling with contact count.
- **Excluded contacts (gap zone)**: MuJoCo accumulates contributions from contacts
  in the margin gap zone into a separate `moment_exclude` buffer. Defer this to a
  stretch goal — it requires #20 (margin/gap runtime effect) to be implemented first.

#### Acceptance Criteria

1. **Loading**: `<actuator><adhesion body="box" gain="100"/>` loads without error
   for a model with a body named "box".
2. **Zero force when no contact**: With `ctrl=1.0` but the body not in contact,
   `qfrc_actuator` contribution from the adhesion actuator is zero.
3. **Attractive force**: A sphere resting on a plane with adhesion `ctrl=1.0`
   produces a downward `qfrc_actuator` (pulling sphere toward plane). Verify
   `qfrc_actuator` has the correct sign by checking that the sphere's equilibrium
   position is lower (more penetrated) than without adhesion.
4. **Force magnitude**: For a single contact, the generalized force equals
   `-(gain * ctrl) * J_normal`. Compare against MuJoCo 3.4.0 `qfrc_actuator`
   for a sphere-on-plane adhesion scenario. Tolerance: `1e-10`.
5. **Multiple contacts**: With 4 contacts (box on plane), moment arm is the
   average of 4 normal Jacobians. Compare against MuJoCo 3.4.0.
6. **Regression**: Models without adhesion actuators produce identical results
   (no force application code path is touched).
7. **`actuator_moment` populated**: After `forward()`, `data.actuator_moment[i]`
   for a body-transmission actuator is nonzero when contacts exist and zero
   when no contacts exist.

#### Implementation Notes

**Contact normal Jacobian reuse.** The constraint assembly already computes contact
normal Jacobians for the constraint system (`efc_J`). However, `mj_transmission_body()`
runs during `mj_fwd_actuation()`, which is called *before* constraint assembly in the
pipeline. So we cannot reuse `efc_J` — we must compute the Jacobian independently.
This matches MuJoCo's approach (transmission runs in the smooth phase, before
constraints).

Use the existing `mj_jac()` or `jac_body_com()` functions to compute the body
Jacobians at the contact point, then difference them and project along the contact
normal.

**Pipeline ordering.** MuJoCo's pipeline: `mj_fwd_position` → `mj_fwd_velocity` →
`mj_fwd_actuation` → `mj_fwd_acceleration` → `mj_fwd_constraint`. Body transmission
runs in `mj_fwd_actuation`, which means it uses contacts from the *previous* step
(contacts are detected in `mj_fwd_position` → `mj_collision`). This is the correct
order — contacts are already populated when `mj_fwd_actuation` runs.

**Gear parameter.** The adhesion shortcut sets `gear[0] = 1.0` by default. The
`gear` value scales the moment arm in `mj_transmission_body()`. For multi-axis
gear (not needed for adhesion), only `gear[0]` is used.

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — add `Body(usize)` to
  `ActuatorTransmission`, implement `mj_transmission_body()`, add Body case
  to Phase 3 force application in `mj_fwd_actuation()`
- `sim/L0/mjcf/src/model_builder.rs` — replace error with body name resolution
  in `process_actuator()`
- `sim/L0/tests/integration/` — new test file `adhesion_actuator.rs` or add to
  existing `actuator_tests.rs`

#### Origin

Migrated from `future_work_6.md` #22.

---

### 23. Tendon Equality Constraints
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State
Pipeline emits a runtime warning and silently ignores `EqualityType::Tendon`
constraints. Any model using `<equality><tendon>` gets incorrect behavior — the
constraint is simply not enforced. Standalone `TendonConstraint`/`TendonNetwork`
exists in sim-constraint but is not wired into the pipeline.

#### Objective
Implement tendon equality constraints in the pipeline constraint assembly.

#### Specification

1. **Constraint type**: `tendon1.length - tendon2.length = polycoef(tendon1.length)`
   where `polycoef` is a polynomial (up to degree 4) mapping reference tendon length
   to target offset (MuJoCo semantics).
2. **Assembly**: In `mj_fwd_constraint()`, for each `EqualityType::Tendon`:
   - Compute constraint violation: `c = L1 - polycoef(L1) - L2`
   - Compute Jacobian: `J = J_tendon1 - dpolycoef/dL1 * J_tendon1 - J_tendon2`
   - Add row to equality constraint block (before contact constraints)
3. **Solver**: Equality constraints are unclamped in PGS/Newton (no projection needed).
4. **solref/solimp**: Apply per-constraint solver parameters for stabilization.

#### Acceptance Criteria
1. Two tendons coupled by `<equality><tendon>` maintain the specified length relationship.
2. `polycoef` polynomial evaluation matches MuJoCo for degree 0–4.
3. Removing the equality constraint reproduces uncoupled behavior (regression).

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `mj_fwd_constraint()`, equality block assembly

#### Origin

Migrated from `future_work_7.md` #23.

---

### 24. `solreffriction` Per-Direction Solver Parameters
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State
`Contact` has a single `solref` field applied to all constraint rows. MuJoCo supports
`solreffriction` — separate solver parameters for friction directions (tangent,
torsional, rolling). Comment at `mujoco_pipeline.rs:5662`: "solreffriction is NOT
applied here."

#### Objective
Apply per-direction solver reference parameters to friction constraint rows.

#### Specification

1. **MJCF parsing**: Parse `solreffriction` from `<geom>`, `<pair>`, and `<default>`
   elements. Store on `Contact` as `solreffriction: [f64; 2]`.
2. **Constraint assembly**: When building the RHS vector `b`:
   - Normal row: use `solref` (existing behavior)
   - Friction rows (tangent, torsional, rolling): use `solreffriction` if specified,
     otherwise fall back to `solref`
3. **Default**: `solreffriction = [0, 0]` means "use solref" (MuJoCo convention —
   zero values indicate "inherit from solref").

#### Acceptance Criteria
1. `solreffriction="0.05 1"` produces different friction damping than `solref`.
2. `solreffriction="0 0"` matches current behavior (regression).
3. Per-direction stabilization is numerically stable.

#### Files
- `sim/L0/mjcf/src/model_builder.rs` — parse `solreffriction`
- `sim/L0/core/src/mujoco_pipeline.rs` — constraint assembly RHS

#### Origin

Migrated from `future_work_7.md` #24.

---

### 25. Fluid / Aerodynamic Forces
**Status:** Not started | **Effort:** L | **Prerequisites:** None

#### Current State
`density`, `viscosity`, and `wind` fields are parsed from MJCF `<option>` and stored
on `Model`, but no fluid drag or lift computation exists in the pipeline. Any model
that relies on these (swimming, flying, falling with drag) produces incorrect results.

MuJoCo computes viscous and inertial drag in `mj_passive()` using ellipsoid
approximations of geom shapes. The force model has two components:
- **Viscous drag**: `F = -6π μ r v` (Stokes drag, scaled by equivalent sphere radius)
- **Inertial drag**: `F = -½ ρ C_d A |v| v` (form drag with ellipsoid cross-section)

#### Objective
Implement MuJoCo-compatible fluid forces in the passive force computation.

#### Specification

1. **Equivalent ellipsoid**: For each geom, compute equivalent ellipsoid semi-axes
   based on geom type and size (MuJoCo uses `mju_geom2Ellipsoid()`).
2. **Viscous drag** (per-geom): `F_visc = -6π * viscosity * equiv_radius * vel_fluid`
   where `vel_fluid = vel_body - wind`.
3. **Inertial drag** (per-geom): `F_inertia = -½ * density * C_d * A_proj * |vel| * vel`
   with projected area `A_proj` from ellipsoid cross-section normal to velocity.
4. **Torque**: Both components produce torques via `r × F` from geom center to body COM.
5. **Application**: Add forces in `mj_passive()` before constraint solving.

#### Acceptance Criteria
1. A falling sphere in a viscous medium reaches terminal velocity.
2. Wind produces lateral force on a stationary body.
3. Zero density/viscosity/wind produces zero fluid force (regression).
4. Force magnitudes match MuJoCo within 1% for a reference test case.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `mj_passive()` or new `mj_fluid()` helper

#### Origin

Migrated from `future_work_7.md` #25.

---

### 26. `<flex>` / `<flexcomp>` MJCF Deformable Body Parsing
**Status:** Not started | **Effort:** L | **Prerequisites:** None

#### Current State
The MJCF parser skips `<flex>` and `<flexcomp>` elements inside `<deformable>` — only
`<skin>` is parsed. MuJoCo's `<flex>` is the native MJCF way to define deformable
bodies (soft bodies, cloth, ropes), and `<flexcomp>` is its procedural counterpart
(similar to `<composite>` but for flex bodies).

CortenForge has a functional deformable pipeline (`XpbdSolver`, `DeformableBody`,
`mj_deformable_collision()`, `mj_deformable_step()`) wired into the main pipeline,
but it's only accessible via the programmatic API (`register_deformable()`). Models
that define deformable bodies through MJCF `<flex>` elements silently lose them.

#### Objective
Parse `<flex>` and `<flexcomp>` MJCF elements and wire them into the existing
deformable pipeline.

#### Specification

1. **`<flex>` parsing**: Parse `<flex>` elements from `<deformable>`:
   - `name`, `dim` (1=cable, 2=shell, 3=solid)
   - `<vertex>` — vertex positions
   - `<element>` — element connectivity (edges/triangles/tetrahedra)
   - `<body>` — which body each vertex belongs to
   - Material properties: `young`, `poisson`, `damping`, `thickness`
   - Collision: `selfcollide`, `radius`, `margin`
2. **`<flexcomp>` parsing**: Parse procedural flex definitions:
   - `type` (grid, box, cylinder, etc.)
   - `count`, `spacing` — procedural dimensions
   - Expand to equivalent `<flex>` before model building
3. **Wiring**: Convert parsed flex data to `DeformableBody` instances and register
   them with the pipeline's `deformable_solvers` during model construction.
4. **`<equality><flex>`**: Parse flex equality constraints (flex-flex coupling).

#### Acceptance Criteria
1. A `<flex dim="2">` cloth element loads and simulates correctly.
2. `<flexcomp type="grid">` generates the expected deformable mesh.
3. Programmatic `register_deformable()` path unchanged (regression).

#### Files
- `sim/L0/mjcf/src/parser.rs` — parse `<flex>`, `<flexcomp>` elements
- `sim/L0/mjcf/src/model_builder.rs` — convert to `DeformableBody`, register with pipeline

#### Origin

Migrated from `future_work_7.md` #26.

---

### 27. Ball / Free Joint Limits (Swing-Twist Cones)
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State
Limit enforcement in `mj_fwd_constraint()` silently skips ball and free joints with
`_ => {}`. MuJoCo supports cone limits (swing-twist) for ball joints — the joint's
rotation is decomposed into swing (away from reference axis) and twist (around
reference axis), each independently limited. Models with `limited="true"` on ball
joints get no limit enforcement.

#### Objective
Implement swing-twist cone limits for ball joints and rotation limits for free joints.

#### Specification

1. **Ball joint limits**: MuJoCo parameterizes ball joint limits as:
   - `range[0]`: Maximum swing angle (rotation away from reference axis) in radians
   - `range[1]`: Maximum twist angle (rotation around reference axis) in radians
   - Constraint is `swing ≤ range[0]` and `|twist| ≤ range[1]`
2. **Swing-twist decomposition**: Given quaternion `q` representing joint rotation:
   - `twist = atan2(2*(q.w*q.z + q.x*q.y), q.w² + q.x² - q.y² - q.z²)`
   - `swing = acos(clamp(2*(q.w² + q.z²) - 1, -1, 1))`
   (or equivalent formulation matching MuJoCo's `mju_ball2Limit()`)
3. **Constraint assembly**: When limit is violated, add constraint row(s) to the
   equality/limit block with appropriate Jacobian (derivative of swing/twist w.r.t.
   angular velocity).
4. **Free joint limits**: Apply same logic to the quaternion DOFs of free joints
   (indices 3-6 of the 7-DOF free joint).

#### Acceptance Criteria
1. A ball joint with `range="30 45"` (degrees, if `<compiler angle="degree"/>`)
   limits swing to 30° and twist to 45°.
2. Limit forces push the joint back within the cone.
3. Unlimited ball joints (`limited="false"`) unchanged (regression).
4. Swing-twist decomposition matches MuJoCo's `mju_ball2Limit()` output.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `mj_fwd_constraint()`, limit enforcement block

#### Origin

Migrated from `future_work_7.md` #27.

---

### 30. Pyramidal Friction Cones
**Status:** Stub | **Effort:** L | **Prerequisites:** None

#### Current State
`Model.cone` field is stored (0=pyramidal, 1=elliptic). When pyramidal is requested,
the solver emits a warning and falls back to elliptic cones. MuJoCo supports both;
pyramidal cones linearize the friction constraint into facets, producing a different
(larger) constraint system.

Most models use the default elliptic cones. Pyramidal cones are mainly relevant for
legacy compatibility and for solvers that cannot handle second-order cone constraints.

#### Objective
Implement pyramidal friction cone constraints as an alternative to elliptic cones.

#### Specification

1. **Linearization**: For a contact with friction μ and condim=3, pyramidal cones
   replace the single elliptic cone `||f_t|| ≤ μ f_n` with 4 linear inequalities:
   `f_t1 ± f_t2 ≤ μ f_n` (4 constraint rows instead of 3).
2. **Constraint sizing**: `nefc` changes based on cone type:
   - Elliptic condim=3: 3 rows per contact (normal + 2 tangent)
   - Pyramidal condim=3: 4 rows per contact (4 facets)
   - Higher condim scales similarly
3. **PGS projection**: Pyramidal constraints use simple non-negativity projection
   (each facet force ≥ 0) instead of cone projection.
4. **Newton solver**: Pyramidal cones require different Hessian blocks. Currently
   Newton falls back to PGS for pyramidal — this item should make both paths work.

#### Acceptance Criteria
1. `<option cone="pyramidal"/>` uses linearized friction (no warning).
2. Friction force magnitude matches MuJoCo pyramidal output within 5%.
3. `<option cone="elliptic"/>` (default) is unchanged (regression).
4. Newton solver handles pyramidal cones without PGS fallback.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — constraint assembly, PGS projection,
  Newton Hessian blocks

#### Origin

Migrated from `future_work_8.md` #30.
