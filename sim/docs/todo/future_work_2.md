# Future Work 2 — Correctness: Model Fidelity (Items #1–5)

Part of [Simulation Phase 2 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

---

### 1. `<default>` Class Resolution
**Status:** Complete | **Effort:** S | **Prerequisites:** None

#### Current State
`DefaultResolver` is fully implemented in `defaults.rs` (1,052 lines, 13 tests).
It builds inheritance hierarchies from nested `<default>` elements, resolves
attributes with class → parent → root lookup, and provides `apply_to_joint()`,
`apply_to_geom()`, `apply_to_actuator()`, `apply_to_tendon()`, `apply_to_sensor()`,
`apply_to_site()` methods for every element type.

**The resolver is never called.** `model_builder.rs` has zero references to
`DefaultResolver`. Defaults are parsed from MJCF, stored in `MjcfModel.defaults`,
and silently dropped. Every element receives hardcoded defaults instead of the
values specified in `<default>` classes.

**Impact:** Any MJCF model that relies on `<default>` classes (standard practice in
MuJoCo Menagerie, DM Control, Gymnasium) produces wrong simulation parameters —
wrong joint damping, wrong geom friction, wrong actuator gains. This is a
correctness bug, not a missing feature.

#### Objective
Wire `DefaultResolver` into the model builder so that `<default class="...">`
attributes are applied to elements before per-element attributes override them.

#### Specification

**Import.** `model_builder.rs` does not reference `DefaultResolver` today. Add:

```rust
use crate::defaults::DefaultResolver;
```

**Integration point:** `model_from_mjcf()` (`model_builder.rs:94`). This is a
free function that creates a local `ModelBuilder`. After construction, set the
resolver before any element processing begins:

```rust
let mut builder = ModelBuilder::new();           // existing (line 98)
builder.resolver = DefaultResolver::from_model(&mjcf);  // ← add
```

This single line makes defaults available to every subsequent `self.resolver`
call inside the builder's process methods.

**Element resolution.** For each element type, apply defaults before the existing
`process_*` call reads fields. The `apply_to_*` methods return a new element with
defaults merged — explicit attributes win, missing attributes filled from class
defaults. The resolver already handles class → parent → root lookup internally.

| Element type | Resolver method | Where applied | `class` field? |
|-------------|----------------|---------------|----------------|
| `MjcfJoint` | `apply_to_joint()` | Before `process_joint()` call in `process_body_with_world_frame()` (:779) | Yes |
| `MjcfGeom` | `apply_to_geom()` | Before `compute_inertia_from_geoms()` (:738) and `process_geom()` (:797) in `process_body_with_world_frame()`, and in `process_worldbody_geoms_and_sites()` (:635). See ordering constraint below. | Yes |
| `MjcfActuator` | `apply_to_actuator()` | Before `process_actuator()` call in `model_from_mjcf()` (:132) | Yes |
| `MjcfTendon` | `apply_to_tendon()` | Inside `process_tendons()` loop (:1124), Pattern B | Yes |
| `MjcfSensor` | `apply_to_sensor()` | Inside `process_sensors()` loop (:1485), Pattern B | Yes |
| `MjcfSite` | `apply_to_site()` | Before `process_site()` calls in `process_body_with_world_frame()` (:803) and `process_worldbody_geoms_and_sites()` (:641) | No — applies root defaults only |

**Resolution patterns.** Three distinct patterns arise depending on how the
caller iterates elements:

*Pattern A — caller loop (joints, geoms, sites, actuators).* The caller
iterates elements and calls a `process_*` method for each one. Resolution
happens in the loop body, before the call:

```rust
// Before (process_body_with_world_frame, line 779):
for joint in &body.joints {
    self.process_joint(joint, body_id, current_last_dof, world_pos, world_quat)?;
}

// After:
for joint in &body.joints {
    let joint = self.resolver.apply_to_joint(joint);
    self.process_joint(&joint, body_id, current_last_dof, world_pos, world_quat)?;
}
```

Same pattern applies to the actuator loop in `model_from_mjcf()` (line 132),
except that `model_from_mjcf` is a free function — use `builder.resolver`
instead of `self.resolver`:

```rust
for actuator in &mjcf.actuators {
    let actuator = builder.resolver.apply_to_actuator(actuator);
    builder.process_actuator(&actuator)?;
}
```

Same pattern applies to worldbody geoms and sites in
`process_worldbody_geoms_and_sites()` (line 627). The geom loop (line 635) and
site loop (line 641) each get a resolver call before the existing `process_*`
call — identical to Pattern A above.

*Pattern B — internal loop (tendons, sensors).* `process_tendons(&[MjcfTendon])`
and `process_sensors(&[MjcfSensor])` receive a slice and iterate internally.
Resolution happens inside these methods at the top of their `for` loop:

```rust
// Inside process_tendons (line 1124):
for (t_idx, tendon) in tendons.iter().enumerate() {
    let tendon = self.resolver.apply_to_tendon(tendon);  // ← add
    // ... existing code uses `tendon.stiffness`, `tendon.damping`, etc.
}
```

Same pattern for `process_sensors` (line 1485). The shadowed `tendon`/`sensor`
binding replaces the borrowed reference with an owned resolved copy.

*Pattern C — geom pre-resolution (inertia ordering).* See ordering constraint
below — geoms are resolved once into a `Vec` before both the inertia call and
the geom processing loop.

**Borrow analysis.** No conflicts in any pattern:
`self.resolver.apply_to_*(elem)` borrows `self` immutably, returns an owned
element, and the borrow ends. The subsequent `self.process_*(...)` borrows
`self` mutably — but the immutable borrow has already been released (NLL).
All `process_*` methods already take references to MJCF types, so no signature
changes are needed.

**Ordering constraint: geom defaults must resolve before inertia computation.**
`process_body_with_world_frame()` calls `compute_inertia_from_geoms(&body.geoms)`
at line 738, *before* the geom processing loop at line 797. This function reads
`geom.density`, `geom.mass`, `geom.geom_type`, and `geom.size` to compute body
mass and inertia. If defaults change any of these fields, inertia will be wrong.

Fix: resolve geom defaults before the inertia call:

```rust
// Resolve geom defaults for inertia computation
let resolved_geoms: Vec<MjcfGeom> = body.geoms.iter()
    .map(|g| self.resolver.apply_to_geom(g))
    .collect();

let (mass, inertia, ipos, iquat) = if let Some(ref inertial) = body.inertial {
    extract_inertial_properties(inertial)
} else {
    compute_inertia_from_geoms(&resolved_geoms)
};

// ... later, use resolved_geoms in the geom processing loop:
for geom in &resolved_geoms {
    self.process_geom(geom, body_id)?;
}
```

This resolves each geom once (not twice) and ensures both inertia computation
and geom processing see the same default-applied values. The subsequent
`body_geom_num.push(body.geoms.len())` at line 800 still uses the original
slice length, which is correct since `resolved_geoms.len() == body.geoms.len()`.

**How "explicit vs default" detection works.** The `apply_to_*` methods use
sentinel values to detect whether a field was explicitly set or is still at its
parser-initialized default. For example, `apply_to_joint()` treats
`damping == 0.0` as "not explicitly set" and fills it from the class default.
The parser initializes `MjcfJoint` fields to MuJoCo's documented default values
(e.g., `damping: 0.0`, `armature: 0.0`, `stiffness: 0.0`), so zero-default
fields work correctly — a user who never specifies damping gets class defaults
applied.

**Doc-comment fix (required).** `get_defaults()` (`defaults.rs:72`) has a
misleading doc comment claiming it "returns the root defaults if the class
doesn't exist." The implementation actually returns `None` for unknown class
names (line 74: `self.resolved_defaults.get(class_name)`). This is safe —
`apply_to_*` methods handle `None` by returning the element unchanged — but an
implementer reading the doc comment might expect different behavior. Fix the doc
comment to say "Returns `None` if the class name is not found" as part of this
task.

**Known limitation: sentinel-based detection diverges from MuJoCo.** MuJoCo
internally initializes attributes in a special "undefined state" distinct from
any valid value, allowing it to always distinguish "explicitly set" from "not
set" — even when the explicit value equals the default. Our sentinel approach
cannot make this distinction. This manifests in two ways:

1. **Zero-default fields** (damping, stiffness, armature): a user who explicitly
   writes `damping="0"` on a joint with a class that specifies `damping="5"`
   will get `5` instead of `0`. In MuJoCo, the explicit `0` would win. In
   practice this rarely matters — explicitly setting a value to its default is
   unusual.

2. **Non-zero-default fields** (geom density=1000, friction=[1,0.005,0.0001],
   joint type=Hinge, joint axis=[0,0,1]): `apply_to_geom()` checks
   `density == 1000.0` using epsilon comparison. A user who explicitly writes
   `density="1000"` on a geom in a class with `density="500"` gets `500` — the
   class default wins over the explicit value. Same for `type="hinge"` on a
   joint whose class specifies `type="slide"`.

This is a pre-existing design issue in `DefaultResolver`, not introduced by
this wiring task. The correct fix is switching MJCF struct fields to `Option<T>`
to distinguish "parsed from XML" from "at default." That refactor is out of
scope here — it touches the parser, all MJCF types, and all `apply_to_*`
methods. Wiring the resolver as-is is still a large net improvement: models
that use `<default>` classes go from completely broken (all defaults silently
dropped) to working correctly for the vast majority of real-world cases.

**Not in scope: `childclass` attribute.** MuJoCo supports
`<body childclass="X">` which sets the default class for all child elements
that don't specify their own `class`. Neither the parser nor `MjcfBody` has a
`childclass` field. This is a separate parser enhancement — the resolver's
class lookup already supports it (just pass the inherited class name when
calling `apply_to_*`). Tracked as a follow-up, not required for this task.

**Not in scope: mesh defaults.** `DefaultResolver` has `mesh_defaults()` and
merges `MjcfMeshDefaults` (containing `scale: Option<Vector3<f64>>`) through
the inheritance chain. However, `MjcfMesh` has no `class` field — mesh defaults
can only come from the root default class. `convert_mjcf_mesh()` reads
`mjcf_mesh.scale` directly (lines 2295, 2331). Applying root-only mesh scale defaults
is low-value (few real models rely on it) and would require special-casing since
there's no `apply_to_mesh()` method. Deferred.

**Not in scope: equality constraint defaults.** The four equality constraint
types (Connect, Weld, Joint, Distance) all have `class` fields in their MJCF
structs, but `DefaultResolver` has no `apply_to_equality()` method and
`MjcfDefault` has no equality defaults. Similarly, `solref`/`solimp` are not
included in any of the defaults structs (`MjcfGeomDefaults`,
`MjcfJointDefaults`, etc.). These are resolver gaps, not wiring gaps — they
require extending `DefaultResolver` and the defaults types, which is a separate
task.

**Threading the resolver.** The resolver must be accessible from five call
sites across `ModelBuilder`:

| Call site | Elements resolved |
|-----------|------------------|
| `model_from_mjcf()` (:132) | actuators |
| `process_body_with_world_frame()` (:779, :797, :803) | joints, geoms, sites |
| `process_worldbody_geoms_and_sites()` (:635, :641) | worldbody geoms, sites |
| `process_tendons()` (:1124) | tendons |
| `process_sensors()` (:1485) | sensors |

**Approach: store on `ModelBuilder`.** Add a `resolver: DefaultResolver` field
to the `ModelBuilder` struct (defined at `model_builder.rs:190`), initialized to
`DefaultResolver::default()` in `ModelBuilder::new()` (:382) and set to
`DefaultResolver::from_model(&mjcf)` at the start of `model_from_mjcf()` before
any processing begins. All methods access `self.resolver`. This avoids threading
a parameter through six method signatures and the recursive
`process_body` → `process_body_with_world_frame` call chain.

#### Acceptance Criteria

**Core functionality:**
1. A model with `<default><joint damping="0.5"/></default>` and a joint that
   does not specify `damping` produces `jnt_damping[0] = 0.5` in the built
   Model.
2. Per-element attributes override defaults: a joint with `class="heavy"`
   and explicit `damping="1.0"`, where class "heavy" specifies
   `damping="0.5"`, produces `jnt_damping = 1.0`.
3. Nested class inheritance: root sets `damping=0.1`, class "robot" sets
   `damping=0.5`, class "arm" (child of "robot") adds `armature=0.01` — a
   joint with `class="arm"` gets `jnt_damping=0.5` and `jnt_armature=0.01`.
4. Geom defaults apply: `<default><geom friction="0.8 0.01 0.001"/></default>`
   affects `geom_friction` in the built Model.
5. Actuator defaults apply: `<default><actuator gear="100"/></default>` affects
   `actuator_gear` in the built Model.
6. Tendon defaults apply: `<default><tendon stiffness="1000"/></default>` affects
   `tendon_stiffness` in the built Model.
7. Sensor defaults apply: `<default><sensor noise="0.01"/></default>` affects
   `sensor_noise` in the built Model.

**Inertia correctness:**
8. A body with no explicit `<inertial>` and a geom whose density comes from a
   default class (e.g., `<default><geom density="500"/></default>`) has
   `body_mass` computed from `density=500`, not the parser default `1000`.

**Edge cases:**
9. Elements without a `class` attribute receive root defaults (empty-string
   class lookup).
10. Elements referencing a nonexistent class fall through gracefully (no panic;
    element keeps its own values unchanged). Note: the resolver already handles
    this — `get_defaults()` returns `None` for unknown classes, and `apply_to_*`
    skips all defaults when the lookup returns `None`. No warning is logged
    today; adding one is optional but recommended (a `tracing::warn!` in
    `get_defaults()` when `class` is `Some` but not found in the map).
11. A model with no `<default>` block at all builds identically to today (the
    resolver is constructed but has no effect — `DefaultResolver::new(&[])` is
    a no-op).

**Regression:**
12. All existing integration tests pass unchanged — current tests don't use
    `<default>` classes, so they must be unaffected.

#### Files
- `sim/L0/mjcf/src/model_builder.rs` — modify: add `DefaultResolver` field to
  `ModelBuilder`, construct in `model_from_mjcf()`, apply before each
  `process_*` call
- `sim/L0/mjcf/src/defaults.rs` — minor: fix misleading `get_defaults()` doc
  comment (:70), optionally add `tracing::warn!` for unknown class names
- `sim/L0/tests/integration/default_classes.rs` — new test file for default
  class verification covering criteria 1–11. Register in
  `sim/L0/tests/integration/mod.rs`. Tests use `sim_mjcf::load_model(xml)`
  and assert on `model.jnt_damping`, `model.geom_friction`, etc.

---

### 2. Contact Condim (1/4/6) + Friction Cones
**Status:** Complete (Phase 1 + Phase 2) | **Effort:** M | **Prerequisites:** None

#### Implementation Status

**Phase 1 — Complete (PR #46)**

Infrastructure for variable contact dimensions is in place:

| Component | Status | Notes |
|-----------|--------|-------|
| `Model.geom_condim: Vec<i32>` | ✅ | Parsed from MJCF, validated to {1,3,4,6} with round-up |
| `Contact.mu: [f64; 5]` | ✅ | `[sliding1, sliding2, torsional, rolling1, rolling2]` |
| `Contact::with_condim()` | ✅ | Sets `dim` directly from condim |
| `make_contact_from_geoms()` | ✅ | Geometric mean per friction type, `max(condim1, condim2)` |
| `compute_contact_jacobian()` | ✅ | Returns `dim×nv` matrix, uses `contact.frame[]` |
| `add_angular_jacobian()` | ✅ | Helper for torsional/rolling rows |
| `compute_efc_offsets()` | ✅ | Tracks per-contact row offsets |
| `assemble_contact_system()` | ✅ | Uses `contact.frame[]`, variable-sized Delassus |
| Tangent basis unification | ✅ | All call sites use `contact.frame[]` (Pre-req 0) |

**Phase 2 — Complete**

All solver functions updated for variable-dimension contacts:

| Function | Status | Implementation |
|----------|--------|----------------|
| `pgs_solve_with_system()` | ✅ | Uses `efc_offsets[i]` for indexing |
| `pgs_solve_contacts()` | ✅ | Returns `Vec<DVector<f64>>` |
| `cg_solve_contacts()` | ✅ | Uses sum of `contact.dim` for `nefc` |
| `Data.efc_lambda` | ✅ | `HashMap<_, Vec<f64>>` for variable condim |
| `compute_block_jacobi_preconditioner()` | ✅ | Returns `Vec<DMatrix<f64>>` for variable `dim×dim` blocks |
| `apply_preconditioner()` | ✅ | Handles variable-dim blocks with `efc_offsets` |
| `extract_forces()` | ✅ | Returns `Vec<DVector<f64>>` |
| `project_elliptic_cone()` | ✅ | Two-step projection (unilateral + friction scaling) |
| `project_friction_cone()` | ✅ | Dispatches to `project_elliptic_cone()` for dim 3/4/6 |
| `apply_contact_torque()` | ✅ | New function for torsional/rolling torques |
| `mj_fwd_constraint()` force loop | ✅ | Applies torques for dim ≥ 4/6 |
| `Model::empty().cone` | ✅ | Default changed to 1 (elliptic) |

**All 565 tests pass (323 unit + 242 integration).**

**Pre-req 0 (tangent basis unification)** is complete — all three call sites
(`compute_contact_jacobian()`, `assemble_contact_system()`, `mj_fwd_constraint()`)
now use `contact.frame[]` instead of recomputing via `build_tangent_basis()`.
The redundant `build_tangent_basis()` function was removed.

**Elliptic cone projection** is fully implemented. Pyramidal cones are not
supported — if `cone == 0` is specified in MJCF, a warning is emitted and
elliptic behavior is used.

**Current behavior:**
- **condim 1** (frictionless): Solver handles correctly (normal force only)
- **condim 3** (sliding): Fully functional with elliptic cone projection
- **condim 4** (torsional): Jacobian + solver + torque application working
- **condim 6** (rolling): Jacobian + solver + torque application working

#### Objective
Support the full range of MuJoCo contact dimensionalities (1, 3, 4, 6) and
both friction cone types (elliptic, pyramidal) so that the solver generates
correct constraint forces for all contact configurations.

#### Specification

This task is organized into seven sub-tasks. **All sub-tasks complete.**

**Dependency order:**

```
Pre-0 (tangent basis) ─→ A (model plumbing) ──→ B (Jacobian) ──→ C (system assembly) ──→ E (solvers)
        ✅                      ✅                   ✅                   ✅                  ✅
                                                                                            ↗
                                                 D (cone projection) ──────────────────────┘
                                                         ✅
                                                 F (force application) ← E
                                                         ✅
                                                 G (cone validation) — independent
                                                         ✅
```

**Phase 2 completed sub-tasks:**
- **D** (cone projection): Two-step `project_elliptic_cone()` for condim 1/3/4/6
- **E** (solvers): PGS/CG use `efc_offsets`, variable-dim blocks, `Vec<f64>` warmstart
- **F** (force application): `apply_contact_torque()` for torsional/rolling
- **G** (cone validation): Pyramidal cone warning, default to elliptic (`Model.cone = 1`)

**Phase 1 completed sub-tasks (for reference only — see code):**
- Pre-0: Tangent basis unification (all call sites use `contact.frame[]`)
- A: Model plumbing (`geom_condim`, `Contact.mu: [f64; 5]`)
- B: Variable-dimension Jacobian (`dim×nv` matrix with angular rows)
- C: System assembly (`assemble_contact_system()` uses `efc_offsets`)

---

**Pre-requisite 0: Unify tangent basis functions** ✅ COMPLETE

All three call sites now use `contact.frame[]` instead of `build_tangent_basis()`.
The redundant function was removed. See `compute_contact_jacobian()`,
`assemble_contact_system()`, and `mj_fwd_constraint()` force loop in current code.

---

<details>
<summary><b>Sub-task A: Model plumbing</b> ✅ COMPLETE (click to expand)</summary>

**A.1.** Add `geom_condim: Vec<i32>` to `Model` (`mujoco_pipeline.rs`). Initialize
in `Model::empty()` as an empty `Vec` (the struct derives `Clone` and `Debug`, so
adding a `Vec<i32>` field requires no trait changes). Populate in
`model_builder.rs:process_geom()` from `MjcfGeom.condim` — push immediately after
the `geom_friction` push (`:1050`), adjacent to the other per-geom solver data.
Validate in `process_geom()` (not the parser — the parser should faithfully
represent the XML): clamp to `{1, 3, 4, 6}` — if a geom specifies an invalid
condim, map to the nearest valid value and emit `tracing::warn!`:
- `condim ≤ 0` → `1` (frictionless)
- `condim = 2` → `3` (round up)
- `condim = 5` → `6` (round up)
- `condim > 6` → `6` (cap at max)

**A.2.** Condim resolution per contact. MuJoCo rule: when two geoms collide,
contact condim = `max(condim1, condim2)` (both geoms have equal priority; geom
priority override is not parsed today and is out of scope). In
`make_contact_from_geoms()` (`:3580`), compute:

```rust
let condim = model.geom_condim[geom1].max(model.geom_condim[geom2]) as usize;
```

**A.3.** Friction coefficient combination. MuJoCo stores 5 friction coefficients
per contact: `[sliding1, sliding2, torsional, rolling1, rolling2]`. Per-geom
friction has 3 values: `[sliding, torsional, rolling]`. The per-contact 5-tuple
is formed by:

- `sliding1 = sliding2 = sqrt(geom1.friction.x * geom2.friction.x)` (isotropic
  sliding — anisotropic sliding requires per-pair override, out of scope)
- `torsional = sqrt(geom1.friction.y * geom2.friction.y)`
- `rolling1 = rolling2 = sqrt(geom1.friction.z * geom2.friction.z)`

**Known non-conformance:** MuJoCo uses element-wise max for friction combination
(when geom priorities are equal). We use geometric mean for all three components,
matching our existing sliding friction convention (`make_contact_from_geoms` line
3591). This is a pre-existing divergence from MuJoCo — not introduced by this
task. Switching to element-wise max would be a one-line change per component but
would alter every existing simulation's behavior — this is better done as a
deliberate conformance task with before/after validation, not buried in a condim
refactor.

**A.4.** Expand `Contact.mu` from `[f64; 2]` to `[f64; 5]`:

```rust
pub mu: [f64; 5],  // [sliding1, sliding2, torsional, rolling1, rolling2]
```

Update `Contact::with_solver_params()` signature to accept a `condim: usize`
parameter and a `mu: [f64; 5]` array. Set `self.dim = condim`. Remove the
`dim: if friction > 0.0 { 3 } else { 1 }` heuristic — condim is now an
explicit input from model data.

`Contact.friction` is retained for API compatibility (used by `Contact::new()`
and debug display). It equals `mu[0]` (sliding1). After this refactor, the solver
projection reads from `mu`, not `friction`.

**Edge case: condim > 1 with zero friction.** If a geom specifies `condim="3"`
but `friction="0 0 0"`, the contact has `dim=3` with `mu=[0,0,0,0,0]`. The
projection's Step 0 clamp zeroes all friction lambdas — physically correct
(frictionless behavior) but wasteful (3 constraint rows for a 1D problem). A
future optimization could detect `mu[0..dim-1] == 0` in `make_contact_from_geoms()`
and downgrade to `condim=1`.

**A.5.** Update `make_contact_from_geoms()` to pass the resolved condim and
full 5-element friction array:

```rust
fn make_contact_from_geoms(
    model: &Model,
    pos: Vector3<f64>,
    normal: Vector3<f64>,
    depth: f64,
    geom1: usize,
    geom2: usize,
) -> Contact {
    let condim = model.geom_condim[geom1].max(model.geom_condim[geom2]) as usize;

    // Sliding friction (geometric mean, existing convention)
    let sliding = (model.geom_friction[geom1].x * model.geom_friction[geom2].x).sqrt();
    let torsional = (model.geom_friction[geom1].y * model.geom_friction[geom2].y).sqrt();
    let rolling = (model.geom_friction[geom1].z * model.geom_friction[geom2].z).sqrt();

    let mu = [sliding, sliding, torsional, rolling, rolling];

    let (solref, solimp) = combine_solver_params(
        model.geom_solref[geom1], model.geom_solimp[geom1],
        model.geom_solref[geom2], model.geom_solimp[geom2],
    );

    Contact::with_solver_params(pos, normal, depth, geom1, geom2, sliding,
                                condim, mu, solref, solimp)
}
```

**A.6.** `Contact::new()` (used by unit tests and direct callers at `:11551`,
`:13855`) retains its existing signature. Internally it calls
`with_solver_params()` with inferred defaults:
`condim = if friction > 0.0 { 3 } else { 1 }` and `mu = [friction, friction,
friction * 0.005, friction * 0.0001, friction * 0.0001]`. The torsional/rolling
ratios (0.005, 0.0001) match MuJoCo's per-geom defaults (`types.rs:857`:
`friction: Vector3::new(1.0, 0.005, 0.0001)`) scaled by the sliding coefficient.
This preserves backward compatibility — all existing test code compiles
unchanged and produces identical contact parameters.

**Struct literal callers.** Three test functions construct `Contact` via struct
literal syntax (bypassing constructors), all in `mujoco_pipeline.rs`:
- `test_cg_solve_single_contact` (`:13641`) — `mu: [0.5, 0.0]`
- `test_project_friction_cone_unit` (`:13680`) — `mu: [0.5, 0.0]`
- `test_project_friction_cone_zero_mu` (`:13825`) — `mu: [0.0, 0.0]`

All three must be updated to `mu: [f64; 5]` format when `Contact.mu` changes.
Update to e.g. `mu: [0.5, 0.5, 0.0, 0.0, 0.0]` or refactor to use
`Contact::new()`. No other struct literal callers exist in the codebase.

</details>

---

<details>
<summary><b>Sub-task B: Variable-dimension Jacobian</b> ✅ COMPLETE (click to expand)</summary>

The contact Jacobian must produce `dim` rows instead of 3. The first 3 rows
are unchanged (normal, tangent1, tangent2). Rows 4–6 are angular Jacobian rows
in the contact frame.

**B.1.** Function signature is unchanged:
`fn compute_contact_jacobian(model: &Model, data: &Data, contact: &Contact) -> DMatrix<f64>`.
Change `DMatrix::zeros(3, nv)` to `DMatrix::zeros(dim, nv)` where
`dim = contact.dim`. Return type stays `DMatrix<f64>` (already dynamic).
For condim 1, the result is a 1×nv matrix — only the normal row. For condim
3, identical to today (3×nv). For condim 4/6, rows 3+ are angular.

**B.2.** Rows 0–2 (unchanged for dim ≥ 3; only row 0 for dim = 1): linear
relative velocity Jacobian in the contact frame directions (normal, tangent1,
tangent2). These map joint velocities to the relative linear velocity at the
contact point, projected onto each direction. For condim 1, rows 1–2 are
absent — the function conditionally skips tangent row construction:

```rust
// Row 0: normal (always)
add_body_jacobian(&mut j, 0, &normal, body2, 1.0);
add_body_jacobian(&mut j, 0, &normal, body1, -1.0);

if dim >= 3 {
    // Rows 1–2: tangent (condim ≥ 3)
    add_body_jacobian(&mut j, 1, &tangent1, body2, 1.0);
    add_body_jacobian(&mut j, 1, &tangent1, body1, -1.0);
    add_body_jacobian(&mut j, 2, &tangent2, body2, 1.0);
    add_body_jacobian(&mut j, 2, &tangent2, body1, -1.0);
}
```

**B.3.** Row 3 (torsional, condim ≥ 4): angular relative velocity about the
contact normal. For a joint in the kinematic chain:

```
J_torsional[dof] = sign * (contact_normal · ω_basis)
```

where `ω_basis` is the angular velocity basis for the joint:
- Hinge: `axis` (the joint axis direction)
- Slide: `0` (prismatic joints produce no angular velocity)
- Ball: `e_i` for each of the 3 DOFs (local frame basis vectors)
- Free (angular DOFs only): `e_x, e_y, e_z` for DOFs 3,4,5

This is the projection of relative angular velocity onto the contact normal.
The `r × F` terms that appear in linear rows are absent here — torsional
friction is a pure torque about the contact normal.

**B.4.** Rows 4–5 (rolling, condim = 6): angular relative velocity in the
tangent plane. Same structure as row 3 but projected onto tangent1 and tangent2:

```
J_rolling1[dof] = sign * (tangent1 · ω_basis)
J_rolling2[dof] = sign * (tangent2 · ω_basis)
```

**B.5.** Implementation: refactor the inner `add_body_jacobian` closure. The
current closure takes `(j, row, direction, body_id, sign)` and computes linear
velocity Jacobian entries. Add a second closure `add_angular_jacobian` that
computes angular velocity Jacobian entries for a given direction. The angular
Jacobian is simpler than the linear one — no `r = contact_point - jpos` cross
product, just direct projection of the angular velocity basis:

```rust
let add_angular_jacobian =
    |j: &mut DMatrix<f64>, row: usize, direction: &Vector3<f64>,
     body_id: usize, sign: f64| {
        if body_id == 0 { return; }
        let mut current_body = body_id;
        while current_body != 0 {
            let jnt_start = model.body_jnt_adr[current_body];
            let jnt_end = jnt_start + model.body_jnt_num[current_body];
            for jnt_id in jnt_start..jnt_end {
                let dof_adr = model.jnt_dof_adr[jnt_id];
                let jnt_body = model.jnt_body[jnt_id];
                match model.jnt_type[jnt_id] {
                    MjJointType::Hinge => {
                        let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                        j[(row, dof_adr)] += sign * direction.dot(&axis);
                    }
                    MjJointType::Slide => { /* no angular contribution */ }
                    MjJointType::Ball => {
                        let rot = data.xquat[jnt_body].to_rotation_matrix();
                        for i in 0..3 {
                            let omega = rot * Vector3::ith(i, 1.0);
                            j[(row, dof_adr + i)] += sign * direction.dot(&omega);
                        }
                    }
                    MjJointType::Free => {
                        // Only angular DOFs (3,4,5) contribute
                        j[(row, dof_adr + 3)] += sign * direction.x;
                        j[(row, dof_adr + 4)] += sign * direction.y;
                        j[(row, dof_adr + 5)] += sign * direction.z;
                    }
                }
            }
            current_body = model.body_parent[current_body];
        }
    };
```

**DOF conventions for angular Jacobian.** The angular Jacobian computes the
projection of relative angular velocity onto a direction (normal, tangent1, or
tangent2). The mapping from DOFs to world-frame angular velocity depends on
joint type:

- **Hinge:** Single DOF. Angular velocity = `qvel[dof] * axis`, where `axis` is
  the joint axis rotated to world frame. Jacobian column = `direction · axis`.
- **Slide:** No angular contribution. Prismatic joints produce zero angular
  velocity.
- **Ball:** 3 DOFs representing angular velocity in **body-local** coordinates.
  The rotation `rot * e_i` maps body-frame basis vector `e_i` to its world-frame
  angular velocity contribution. Jacobian column for DOF `i` =
  `direction · (rot * e_i)`. This is consistent with the existing linear
  Jacobian (`:7670–7675`).
- **Free:** Angular DOFs 3–5 represent world-frame angular velocity
  `(ωx, ωy, ωz)` directly. The angular Jacobian column for DOF `3+k` is simply
  `direction[k]` — the identity projection from world-frame angular velocity
  onto the contact direction. Note: this differs from the *linear* Jacobian's
  angular DOF entries (which compute `direction · (eₖ × r)` — the linear
  velocity at the contact point due to rotation). The difference is correct:
  the angular Jacobian maps DOFs to angular velocity (not linear velocity at
  a point).

Then for condim ≥ 4, add row 3 (torsional):
```rust
if dim >= 4 {
    add_angular_jacobian(&mut j, 3, &normal, body2, 1.0);
    add_angular_jacobian(&mut j, 3, &normal, body1, -1.0);
}
```

And for condim = 6, add rows 4–5 (rolling):
```rust
if dim >= 6 {
    add_angular_jacobian(&mut j, 4, &tangent1, body2, 1.0);
    add_angular_jacobian(&mut j, 4, &tangent1, body1, -1.0);
    add_angular_jacobian(&mut j, 5, &tangent2, body2, 1.0);
    add_angular_jacobian(&mut j, 5, &tangent2, body1, -1.0);
}
```

</details>

---

<details>
<summary><b>Sub-task C: Variable-dimension system assembly</b> ✅ COMPLETE (click to expand)</summary>

**C.1.** Extract `efc_offsets` computation into a shared helper:

```rust
/// Compute per-contact row offsets and total constraint count for variable-dim contacts.
/// Returns (offsets, nefc) where offsets[i] = starting row index for contact i.
fn compute_efc_offsets(contacts: &[Contact]) -> (Vec<usize>, usize) {
    let mut offsets = Vec::with_capacity(contacts.len());
    let mut nefc = 0;
    for c in contacts {
        offsets.push(nefc);
        nefc += c.dim;
    }
    (offsets, nefc)
}
```

This replaces `nefc = ncon * 3` in `assemble_contact_system()` (`:7806`),
`pgs_solve_with_system()` (`:8062`), and `cg_solve_contacts()` (`:8257`).

In `mj_fwd_constraint()`, compute offsets once and pass to all solver
functions via an added parameter:

```rust
// In mj_fwd_constraint(), before solver dispatch:
let (efc_offsets, nefc) = compute_efc_offsets(&data.contacts);

// Compute Jacobians (per-contact, variable row count):
let jacobians: Vec<DMatrix<f64>> = data.contacts.iter()
    .map(|c| compute_contact_jacobian(model, data, c))
    .collect();

// Assemble Delassus system (Jacobians passed in, not returned):
let (a, b) = assemble_contact_system(
    model, data, &data.contacts, &jacobians, &efc_offsets, nefc,
);

// Thread to solver dispatch (PGS or CG):
let (forces, niter) = pgs_solve_contacts(..., &efc_offsets, ...);
// or: cg_solve_contacts(..., &efc_offsets, ...);

// Thread to force application loop:
for (i, lambda) in forces.iter().enumerate() {
    let base = efc_offsets[i]; // used for warmstart store
    // ...
}
```

All three solver entry points (`pgs_solve_contacts`, `pgs_solve_with_system`,
`cg_solve_contacts`) need to use `efc_offsets` for variable-dimension indexing.

**Phase 1 implemented signature** (already in codebase):
```rust
fn assemble_contact_system(
    model: &Model,
    data: &Data,
    contacts: &[Contact],
    jacobians: &[DMatrix<f64>],
) -> (DMatrix<f64>, DVector<f64>, Vec<usize>)  // Returns efc_offsets
```

The function computes `efc_offsets` internally via `compute_efc_offsets()` and
returns it as the third tuple element. Callers extract offsets from the return
value rather than passing them in.

The return type is unchanged — a 2-tuple of `(A, b)` (Delassus matrix and RHS
vector). Jacobians are passed **in** as a parameter (computed separately in
`mj_fwd_constraint()`), not returned. The new `efc_offsets` and `nefc`
parameters replace the internal `ncon * 3` computation.

**C.2.** `M⁻¹ * Jᵀ` computation (`:7823–7834`). The inner loop currently
iterates `for col in 0..3`. Change to `for col in 0..contact_dim` where
`contact_dim = contacts[i].dim`. The `minv_jt_contact` matrix becomes
`nv × dim` instead of `nv × 3`.

**C.3.** Diagonal block assembly (`:7856–7863`). Currently `for ri in 0..3 { for
ci in 0..3 }`. Change to `for ri in 0..dim_i { for ci in 0..dim_i }` where
`dim_i = contacts[i].dim`. Indexing changes from `i * 3 + ri` to
`efc_offsets[i] + ri`. The same CFM (derived from per-contact solimp) applies to
all `dim` diagonal entries, matching MuJoCo's per-row impedance which uses
identical solimp for all rows of a single contact.

**C.4.** Off-diagonal block assembly (`:7936–7942`). Same change: `for ri in
0..dim_i { for ci in 0..dim_j }` with offset-based indexing.

**C.5.** RHS construction (`:7946–7985`). **Requires sub-task B to be complete**
so that `jac_i` has `dim` rows and `j_qacc_smooth = jac_i * &qacc_smooth`
produces a `dim`-element vector. The current code computes `vn, vt1, vt2` from
linear relative velocity. For condim ≥ 4, also compute angular relative velocity
at the contact point and project onto the torsional/rolling directions:

```rust
let base = efc_offsets[i];
let dim = contacts[i].dim;

// Rows 0–2: linear relative velocity (unchanged)
b[base]     = j_qacc_smooth[0] + velocity_damping * vn + depth_correction * depth;
if dim >= 3 {
    b[base + 1] = j_qacc_smooth[1] + vt1;
    b[base + 2] = j_qacc_smooth[2] + vt2;
}

// Rows 3+: angular relative velocity
// The angular relative velocity is computed by the Jacobian rows 3+,
// so j_qacc_smooth[3..] already contains the correct acceleration terms.
// The velocity terms come from the angular Jacobian applied to qvel.
if dim >= 4 {
    let j_qvel_torsional = jacobians[i].row(3).dot(&data.qvel);
    b[base + 3] = j_qacc_smooth[3] + j_qvel_torsional;
}
if dim >= 6 {
    let j_qvel_roll1 = jacobians[i].row(4).dot(&data.qvel);
    let j_qvel_roll2 = jacobians[i].row(5).dot(&data.qvel);
    b[base + 4] = j_qacc_smooth[4] + j_qvel_roll1;
    b[base + 5] = j_qacc_smooth[5] + j_qvel_roll2;
}
```

**Condim-1 safety.** For condim 1, `jac_i` is 1×nv and `j_qacc_smooth` is a
1-element vector. The `if dim >= 3` guard prevents out-of-bounds access at
`j_qacc_smooth[1]`. The `vt1`/`vt2` computation (`:7962–7964`) is still
executed but harmless — the values are unused. Similarly, `contact.frame`
tangent vectors are computed but unused for condim 1 (~20 FLOPs per
frictionless contact). Both are micro-optimizations — skip only if profiling
shows this path is hot (unlikely — contact detection dominates).

Note: the normal row includes Baumgarte position correction
(`depth_correction * depth`). The friction/torsional/rolling rows do not — they
are velocity-only constraints, matching MuJoCo's formulation.

**Velocity computation paths.** The linear velocity terms (`vn`, `vt1`, `vt2`)
are computed via `compute_point_velocity()` (`:7955–7964`) — body-relative
velocity projected onto the contact frame. The angular velocity terms use
`J[row,:] · qvel` instead (the Jacobian). These are mathematically equivalent;
`compute_point_velocity()` computes relative linear velocity at a point, while
an angular analogue would compute relative angular velocity — a different
quantity that would need a separate function. Using the Jacobian rows directly
is simpler since they are already computed.

</details>

---

## Phase 2 Sub-tasks (COMPLETE)

---

**Sub-task D: Friction cone projection — `project_friction_cone()`** ✅ COMPLETE

This is the core mathematical change. The projection depends on both `condim`
and `Model.cone`.

**D.1. Elliptic cone projection.**

For condim `d`, the feasible set is a second-order cone (SOC):

```
K = { λ ∈ ℝ^d : λ₀ ≥ 0,  λ₀² ≥ Σᵢ₌₁^{d-1} (λᵢ / μᵢ)² }
```

where `μᵢ = contact.mu[i-1]` (the `i`-th friction coefficient).

**Lambda-to-mu index mapping:**

| lambda index | Physical meaning | mu index | mu field |
|:---:|:---|:---:|:---|
| 0 | Normal force | — | (no mu; clamped ≥ 0) |
| 1 | Sliding friction 1 | 0 | sliding1 |
| 2 | Sliding friction 2 | 1 | sliding2 |
| 3 | Torsional friction | 2 | torsional |
| 4 | Rolling friction 1 | 3 | rolling1 |
| 5 | Rolling friction 2 | 4 | rolling2 |

**MuJoCo's approach (for reference).** MuJoCo's PGS solver does a coupled
QCQP projection per contact block: it solves
`min 0.5 x'Ax + x'b  s.t. Σ(xᵢ/μᵢ)² ≤ r²` using Newton iteration on the
Lagrange multiplier (functions `mju_QCQP`, `mju_QCQP2`, `mju_QCQP3` in
`engine_util_solve.c`). This produces the exact minimum of the local QP
subject to the cone constraint — the resulting friction direction can differ
from the unconstrained update direction because the off-diagonal Delassus
coupling is accounted for in the projection.

**Our approach: Two-step physical projection.** Instead of the pure SOC
projection (which would project negative normal forces to the cone boundary),
we use a physically-motivated two-step approach that matches MuJoCo's
constraint-level semantics:

1. **Unilateral constraint** (`λ₀ < 0`): contact is separating → release
   completely (all forces = 0). A negative normal force is physically
   impossible — it would mean the contact is pulling the bodies together.
2. **Friction cone** (`s > λ₀`): friction exceeds cone boundary → scale
   friction components by `λ₀/s` to project to the boundary. The normal
   force is preserved.

Where `s = √(Σ (λᵢ/μᵢ)²)` is the weighted friction norm.

This differs from the pure SOC projection which would handle Case 1 differently
(projecting to the cone boundary rather than the origin). The two-step approach
is physically correct: when the solver produces a negative normal force, the
contact should release entirely rather than producing non-zero friction.

```rust
fn project_elliptic_cone(lambda: &mut [f64], mu: &[f64; 5], dim: usize) {
    // Step 1: Enforce unilateral constraint (normal force must be non-negative)
    // Negative normal force = separating contact = release completely
    if lambda[0] < 0.0 {
        for l in lambda.iter_mut().take(dim) {
            *l = 0.0;
        }
        return;
    }

    // Step 2: Clamp friction components where mu ≈ 0 (infinite resistance = no sliding)
    for i in 1..dim {
        if mu[i - 1] <= 1e-12 {
            lambda[i] = 0.0;
        }
    }

    // Step 3: Compute weighted friction norm (elliptic cone radius)
    // s = sqrt( Σ (λ_i / μ_i)² ) for i = 1..dim-1
    let mut s_sq = 0.0;
    for i in 1..dim {
        if mu[i - 1] > 1e-12 {
            s_sq += (lambda[i] / mu[i - 1]).powi(2);
        }
    }
    let s = s_sq.sqrt();

    // Step 4: If friction exceeds cone boundary, scale to boundary
    // Cone constraint: s ≤ λ_n, i.e., ||(λ_i/μ_i)|| ≤ λ_n
    if s > lambda[0] && s > 1e-10 {
        let scale = lambda[0] / s;
        for l in lambda.iter_mut().take(dim).skip(1) {
            *l *= scale;
        }
    }
}
```

**Why two-step over pure SOC?** The pure SOC projection has a third case:
when `λ₀ < 0` but `λ₀ > -s` (negative normal but not in the polar cone), it
projects to the cone *boundary* at `t = (λ₀ + s)/2`, producing non-zero forces.
This is mathematically the nearest point on the cone, but physically incorrect:
a separating contact should not produce any force. The two-step approach
ensures that any separating contact (λ₀ < 0) releases completely.

When `μ₁ = μ₂` (isotropic sliding), the friction scaling reduces exactly to
the circular cone projection (`‖λ_t‖ ≤ μ * λ_n`).

**Divergence from MuJoCo's QCQP.** MuJoCo's QCQP projection solves a local QP
that can rotate the friction direction toward the optimum. Our projection
preserves the friction *direction* in μ-weighted space and only scales the
magnitude. For diagonal-dominant Delassus matrices (typical in practice —
contacts on independent or weakly coupled bodies), the off-diagonal terms are
small and the sequential projection is an excellent approximation. For strongly
coupled contacts (multiple contacts on a single small body), the QCQP would
converge in fewer iterations. Upgrading to QCQP projection is a separate
optimization task.

**Anisotropic scaling approximation.** The uniform scaling `lambda[i] *= scale`
projects to the cone boundary by preserving the friction direction in μ-weighted
space, not the nearest Euclidean point in physical space. For the exact
projection when `μᵢ` differ, one would solve a scalar nonlinear equation
(Newton on the KKT Lagrange multiplier) per contact. The error is proportional
to the anisotropy ratio `max(μ)/min(μ) - 1`; for typical torsional/rolling
coefficients (≪ sliding), the approximation is excellent. When all `μᵢ` are
equal (isotropic), the uniform scaling is exact.

**Note: when mu[i] ≈ 0.** If a friction coefficient is near zero (≤ 1e-12),
`lambda[i]` is clamped to zero in Step 2 (before computing `s`). This is
necessary because any nonzero `λᵢ` with `μᵢ → 0` makes `λᵢ/μᵢ → ∞`,
violating the cone constraint. This handles degenerate cases like condim 4
with zero torsional friction — the torsional row exists in the Jacobian/Delassus
but the cone constraint forces it to zero.

**D.2. Pyramidal cone projection (`cone == 0`, MuJoCo default).**

The pyramidal formulation is fundamentally different from elliptic. MuJoCo's
pyramidal cone uses `2(d-1)` non-negative variables instead of `d` cone-
projected variables. Each non-negative variable corresponds to one edge of the
pyramid. The solver state is:

```
f₊₁, f₋₁, f₊₂, f₋₂, ...   (all ≥ 0)
```

The physical force is recovered as:
```
λ_normal = Σ fₖ              (all edge forces contribute to normal)
λ_friction_i = f₊ᵢ - f₋ᵢ    (net friction in each direction)
```

Projection is trivial: clamp each `fₖ ≥ 0`. However, this changes the system
size — a condim-3 contact produces 4 non-negative variables (not 3 cone
variables), condim-4 produces 6, condim-6 produces 10.

**Decision: defer pyramidal cones.** Pyramidal cones require a different system
size, different Jacobian basis construction, and different warmstart format.
This is a substantial architectural change on top of the already-large variable-
dimension refactor. Since MuJoCo's default is `pyramidal` but the elliptic cone
is mathematically cleaner and our solver already uses cone projection (not LP),
we implement elliptic cones first. Pyramidal support is tracked as a follow-up.
`Model.cone` is validated: if `cone == 0` (pyramidal), emit a
`tracing::warn!` and fall back to elliptic behavior.

**D.3.** Condim-1 projection: simply `lambda[0] = lambda[0].max(0.0)`. No
friction components.

**D.4.** Update `project_friction_cone()` signature:

```rust
fn project_friction_cone(
    lambda: &mut DVector<f64>,
    contacts: &[Contact],
    efc_offsets: &[usize],
) {
    for (i, contact) in contacts.iter().enumerate() {
        let base = efc_offsets[i];
        let dim = contact.dim;
        match dim {
            1 => {
                lambda[base] = lambda[base].max(0.0);
            }
            3 | 4 | 6 => {
                project_elliptic_cone(
                    &mut lambda.as_mut_slice()[base..base + dim],
                    &contact.mu,
                    dim,
                );
            }
            _ => {
                // Invalid condim — treat as frictionless for safety
                lambda[base] = lambda[base].max(0.0);
                for j in 1..dim {
                    lambda[base + j] = 0.0;
                }
            }
        }
    }
}
```

---

**Sub-task E: Solver updates — PGS and CG** ✅ COMPLETE

**E.1. `pgs_solve_with_system()`**

Compute `efc_offsets` and `nefc` from contacts (same as sub-task C). Replace
all `i * 3` indexing with `efc_offsets[i]`. The existing `diag_inv`
precomputation (`1/A[i,i]` for each of the `nefc` rows, with zero fallback for
singular diagonals) is unchanged — it already operates on the full
`nefc`-length diagonal. The per-contact GS sweep changes:

```rust
for i in 0..ncon {
    let base = efc_offsets[i];
    let dim = contacts[i].dim;

    // Compute residuals for all dim rows of this contact
    let mut residuals = vec![0.0; dim];
    for r in 0..dim {
        residuals[r] = b[base + r];
        for j in 0..nefc {
            residuals[r] += a[(base + r, j)] * lambda[j];
        }
    }

    // GS update
    let old: Vec<f64> = (0..dim).map(|r| lambda[base + r]).collect();
    for r in 0..dim {
        lambda[base + r] -= residuals[r] * diag_inv[base + r];
    }

    // Project this contact onto its friction cone
    match dim {
        1 => { lambda[base] = lambda[base].max(0.0); }
        3 | 4 | 6 => {
            project_elliptic_cone(
                &mut lambda.as_mut_slice()[base..base + dim],
                &contacts[i].mu,
                dim,
            );
        }
        _ => { /* unreachable after A.1 validation */ }
    }

    // Track convergence
    for r in 0..dim {
        max_delta = max_delta.max((lambda[base + r] - old[r]).abs());
    }
}
```

**E.1b. `pgs_solve_contacts()` wrapper.** This is a pass-through
wrapper that calls `assemble_contact_system()` then `pgs_solve_with_system()`.
Changes: use `efc_offsets` from the return value of `assemble_contact_system()`.
Return type changes from `(Vec<Vector3<f64>>, usize)` to
`(Vec<DVector<f64>>, usize)` per the type propagation table. No logic changes
beyond indexing updates.

**E.2. Warmstart format.** Change `efc_lambda: HashMap<WarmstartKey, [f64; 3]>`
(`Data` struct) to `HashMap<WarmstartKey, Vec<f64>>`. The `Vec` length
matches the contact's dim. On warmstart load, if the stored dim differs from
the current contact's dim (e.g., condim changed between frames), discard the
warmstart for that contact (use zero initialization). On store, save
`lambda[base..base+dim].to_vec()`.

**Callers that must update:**

| Location | Current usage | Change |
|----------|--------------|--------|
| `pgs_solve_with_system()` warmstart load | `[f64; 3]` into `lambda[base..base+3]` | `Vec<f64>`, check len == dim |
| `pgs_solve_with_system()` warmstart store | `[lambda[base], ...]` | `lambda[base..base+dim].to_vec()` |
| `cg_solve_contacts()` warmstart load | `[f64; 3]` | Same as PGS |
| `cg_solve_contacts()` single-contact store | `[lam[0], lam[1], lam[2]]` | `lam.as_slice().to_vec()` |
| `cg_solve_contacts()` multi-contact store | `[lambda[i*3], ...]` | `lambda[base..base+dim].to_vec()` |
| Touch sensor | `lambda[0]` via `[f64; 3]` | `lambda[0]` via `Vec<f64>` (logic unchanged) |
| RK4 integrator | `data.efc_lambda.clone()` | Works unchanged (`HashMap<K, Vec>` is `Clone`) |
| Test code | `HashMap<WarmstartKey, [f64; 3]>` | Update to `Vec<f64>` |

The touch sensor reads `lambda[0]` which is the normal force
regardless of condim — no logic change needed, only the type annotation.

**E.3. `compute_block_jacobi_preconditioner()`.** Replace `Matrix3`
blocks with `DMatrix` blocks of size `dim × dim`:

```rust
fn compute_block_jacobi_preconditioner(
    a: &DMatrix<f64>,
    contacts: &[Contact],
    efc_offsets: &[usize],
) -> Vec<DMatrix<f64>> {
    contacts.iter().enumerate().map(|(i, contact)| {
        let base = efc_offsets[i];
        let dim = contact.dim;
        let block = a.view((base, base), (dim, dim)).clone_owned();
        if let Some(chol) = block.cholesky() {
            chol.inverse()
        } else {
            // Scalar Jacobi fallback
            let mut inv = DMatrix::zeros(dim, dim);
            for r in 0..dim {
                let d = block[(r, r)];
                inv[(r, r)] = if d.abs() > 1e-12 { 1.0 / d } else { 0.0 };
            }
            inv
        }
    }).collect()
}
```

**E.4. `apply_preconditioner()`.** Change from `Vector3` blocks to
`DVector` blocks, indexing via `efc_offsets`:

```rust
fn apply_preconditioner(
    precond: &[DMatrix<f64>],
    v: &DVector<f64>,
    contacts: &[Contact],
    efc_offsets: &[usize],
) -> DVector<f64> {
    let mut result = DVector::zeros(v.len());
    for (i, contact) in contacts.iter().enumerate() {
        let base = efc_offsets[i];
        let dim = contact.dim;
        let block = v.rows(base, dim);
        let applied = &precond[i] * block; // dim × dim inverse block * dim vector
        result.rows_mut(base, dim).copy_from(&applied);
    }
    result
}
```

**E.5. `cg_solve_contacts()`.** Note: despite the `cg_` name, this
solver implements Preconditioned Projected Gradient Descent (PGD) with
Barzilai-Borwein adaptive step size, not conjugate gradient. The "CG" name is
historical. The following changes preserve the PGD algorithm.

- Accept `efc_offsets: &[usize]` parameter; use for `nefc` and all indexing
- Single-contact direct solve (`:8278–8306`): replace `Matrix3` / `Vector3`
  with `DMatrix` / `DVector` of size `dim × dim` / `dim`. The existing
  `a_block.try_inverse()` becomes `a.view((0,0),(dim,dim)).clone_owned()
  .try_inverse()`. Cone projection uses `project_elliptic_cone()` on a
  `dim`-element slice instead of manual inline clamping. Warmstart store
  saves `lam.as_slice().to_vec()` instead of `[lam[0], lam[1], lam[2]]`
- `project_friction_cone()` call passes `efc_offsets`
- Warmstart load: if `prev.len() != contact.dim`, skip (zero init)
- `b_norm` check (`:8322`): unchanged — operates on the full `DVector`
- `apply_preconditioner()` call: uses updated variable-dim version (E.4)
- Return type: `Result<(Vec<DVector<f64>>, usize), (DMatrix<f64>, DVector<f64>)>`.
  The `Err` variant carries the pre-assembled Delassus system `(A, b)` so the
  CG→PGS fallback in `mj_fwd_constraint()` can call `pgs_solve_with_system()`
  directly without re-assembling the matrix. This pattern is unchanged — only
  the `Ok` forces type changes from `Vec<Vector3>` to `Vec<DVector>`.
  The fallback call passes `efc_offsets`:
  ```rust
  Err((a, b)) => {
      let (forces, niter) = pgs_solve_with_system(
          &data.contacts, &a, &b, &efc_offsets,
          clamped_iters, clamped_tol, &mut efc_lambda,
      );
      // ...
  }
  ```

**E.6. `extract_forces()`.** Change return type from
`Vec<Vector3<f64>>` to `Vec<DVector<f64>>`:

```rust
fn extract_forces(
    lambda: &DVector<f64>,
    contacts: &[Contact],
    efc_offsets: &[usize],
) -> Vec<DVector<f64>> {
    contacts.iter().enumerate().map(|(i, c)| {
        let base = efc_offsets[i];
        lambda.rows(base, c.dim).clone_owned()
    }).collect()
}
```

**E.7. Consolidated signature changes.**

All function signatures that change in Phase 2, with line references to current
implementation in `mujoco_pipeline.rs`:

```rust
// CURRENT (line 8236):
fn project_friction_cone(lambda: &mut DVector<f64>, contacts: &[Contact], ncon: usize)

// AFTER:
fn project_friction_cone(
    lambda: &mut DVector<f64>,
    contacts: &[Contact],
    efc_offsets: &[usize],
)
```

```rust
// CURRENT (line 8291):
fn pgs_solve_with_system(
    contacts: &[Contact],
    a: &DMatrix<f64>,
    b: &DVector<f64>,
    max_iterations: usize,
    tolerance: f64,
    efc_lambda: &mut HashMap<WarmstartKey, [f64; 3]>,
) -> (Vec<Vector3<f64>>, usize)

// AFTER:
fn pgs_solve_with_system(
    contacts: &[Contact],
    a: &DMatrix<f64>,
    b: &DVector<f64>,
    efc_offsets: &[usize],                              // NEW
    max_iterations: usize,
    tolerance: f64,
    efc_lambda: &mut HashMap<WarmstartKey, Vec<f64>>,   // CHANGED
) -> (Vec<DVector<f64>>, usize)                         // CHANGED
```

```rust
// CURRENT (line 8270):
fn pgs_solve_contacts(
    model: &Model,
    data: &Data,
    contacts: &[Contact],
    jacobians: &[DMatrix<f64>],
    max_iterations: usize,
    tolerance: f64,
    efc_lambda: &mut HashMap<WarmstartKey, [f64; 3]>,
) -> (Vec<Vector3<f64>>, usize)

// AFTER:
fn pgs_solve_contacts(
    model: &Model,
    data: &Data,
    contacts: &[Contact],
    jacobians: &[DMatrix<f64>],
    max_iterations: usize,
    tolerance: f64,
    efc_lambda: &mut HashMap<WarmstartKey, Vec<f64>>,   // CHANGED
) -> (Vec<DVector<f64>>, usize)                         // CHANGED
```

```rust
// CURRENT (line 8486):
fn cg_solve_contacts(
    model: &Model,
    data: &Data,
    contacts: &[Contact],
    jacobians: &[DMatrix<f64>],
    max_iterations: usize,
    tolerance: f64,
    efc_lambda: &mut HashMap<WarmstartKey, [f64; 3]>,
) -> Result<(Vec<Vector3<f64>>, usize), (DMatrix<f64>, DVector<f64>)>

// AFTER:
fn cg_solve_contacts(
    model: &Model,
    data: &Data,
    contacts: &[Contact],
    jacobians: &[DMatrix<f64>],
    max_iterations: usize,
    tolerance: f64,
    efc_lambda: &mut HashMap<WarmstartKey, Vec<f64>>,   // CHANGED
) -> Result<(Vec<DVector<f64>>, usize), (DMatrix<f64>, DVector<f64>)>  // CHANGED
```

```rust
// CURRENT (line 8252):
fn extract_forces(lambda: &DVector<f64>, ncon: usize) -> Vec<Vector3<f64>>

// AFTER:
fn extract_forces(
    lambda: &DVector<f64>,
    contacts: &[Contact],
    efc_offsets: &[usize],
) -> Vec<DVector<f64>>
```

```rust
// CURRENT (line ~8400, inside cg_solve_contacts):
fn compute_block_jacobi_preconditioner(a: &DMatrix<f64>, ncon: usize) -> Vec<Matrix3<f64>>

// AFTER:
fn compute_block_jacobi_preconditioner(
    a: &DMatrix<f64>,
    contacts: &[Contact],
    efc_offsets: &[usize],
) -> Vec<DMatrix<f64>>
```

```rust
// CURRENT (line ~8420, inside cg_solve_contacts):
fn apply_preconditioner(precond: &[Matrix3<f64>], v: &DVector<f64>) -> DVector<f64>

// AFTER:
fn apply_preconditioner(
    precond: &[DMatrix<f64>],
    v: &DVector<f64>,
    contacts: &[Contact],
    efc_offsets: &[usize],
) -> DVector<f64>
```

```rust
// CURRENT (line 1677, Data struct):
pub efc_lambda: HashMap<WarmstartKey, [f64; 3]>,

// AFTER:
pub efc_lambda: HashMap<WarmstartKey, Vec<f64>>,
```

---

**Sub-task F: Force application — `mj_fwd_constraint()`** ✅ COMPLETE

The force application loop in `mj_fwd_constraint()` currently converts lambda
(`Vector3<f64>`) to a world-frame linear force. For condim ≥ 4, torsional and
rolling components are torques, not forces. They require a different
application path.

**F.1.** Add `apply_contact_torque()` alongside `apply_contact_force()`. This
function traverses the kinematic chain and applies the torque contribution to
each joint's DOF — the angular Jacobian transpose:

```rust
fn apply_contact_torque(
    model: &Model,
    data: &mut Data,
    body_id: usize,
    torque: Vector3<f64>,
) {
    if body_id == 0 { return; }
    let mut current_body = body_id;
    while current_body != 0 {
        let jnt_start = model.body_jnt_adr[current_body];
        let jnt_end = jnt_start + model.body_jnt_num[current_body];
        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let jnt_body = model.jnt_body[jnt_id];
            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    data.qfrc_constraint[dof_adr] += axis.dot(&torque);
                }
                MjJointType::Slide => { /* no angular DOF */ }
                MjJointType::Ball => {
                    let body_torque = data.xquat[jnt_body].inverse() * torque;
                    data.qfrc_constraint[dof_adr] += body_torque.x;
                    data.qfrc_constraint[dof_adr + 1] += body_torque.y;
                    data.qfrc_constraint[dof_adr + 2] += body_torque.z;
                }
                MjJointType::Free => {
                    data.qfrc_constraint[dof_adr + 3] += torque.x;
                    data.qfrc_constraint[dof_adr + 4] += torque.y;
                    data.qfrc_constraint[dof_adr + 5] += torque.z;
                }
            }
        }
        current_body = model.body_parent[current_body];
    }
}
```

**F.2.** Update the force application loop in `mj_fwd_constraint()`:

```rust
for (i, lambda) in constraint_forces.iter().enumerate() {
    let contact = &data.contacts[i];
    let normal = contact.normal;
    let (tangent1, tangent2) = (contact.frame[0], contact.frame[1]); // Pre-req 0
    let dim = contact.dim;

    // Linear force (rows 0–2, when dim ≥ 3)
    let world_force = if dim >= 3 {
        normal * lambda[0] + tangent1 * lambda[1] + tangent2 * lambda[2]
    } else {
        normal * lambda[0]  // condim 1: normal only
    };

    let body1 = model.geom_body[contact.geom1];
    let body2 = model.geom_body[contact.geom2];
    let pos = contact.pos;

    apply_contact_force(model, data, body1, pos, -world_force);
    apply_contact_force(model, data, body2, pos, world_force);

    // Torsional torque (row 3, when dim ≥ 4)
    if dim >= 4 {
        let torsional_torque = normal * lambda[3];
        apply_contact_torque(model, data, body1, -torsional_torque);
        apply_contact_torque(model, data, body2, torsional_torque);
    }

    // Rolling torques (rows 4–5, when dim = 6)
    if dim >= 6 {
        let rolling_torque = tangent1 * lambda[4] + tangent2 * lambda[5];
        apply_contact_torque(model, data, body1, -rolling_torque);
        apply_contact_torque(model, data, body2, rolling_torque);
    }
}
```

---

**Sub-task G: MuJoCo conformance — cone type validation** ✅ COMPLETE

**G.1.** Change `Model::empty()` default from `cone: 0` (pyramidal) to
`cone: 1` (elliptic). Since we don't implement pyramidal cones, our default
should match what we support. Additionally, in `model_from_mjcf()`, if the
MJCF explicitly specifies `cone == 0` (pyramidal), emit
`tracing::warn!("pyramidal friction cones not yet supported — using elliptic")`
and set `cone = 1`. This prevents silent incorrect behavior without warning
noise on every default-configured model.

**G.2.** When `cone == 1` (elliptic), use the `project_elliptic_cone()` from
sub-task D.1.

**G.3.** When full elliptic semantics are active and `mu[0] != mu[1]`
(anisotropic sliding), the projection in D.1 already handles this correctly
via the per-coefficient scaling `(λᵢ / μᵢ)²`.

---

**Borrow / ownership analysis.**

- `efc_offsets` is a `Vec<usize>` computed once in `mj_fwd_constraint()` from
  `&data.contacts` (immutable borrow ends before solver call). Passed as
  `&[usize]` to all solver functions — no borrow conflicts.
- `project_friction_cone()` borrows `lambda` mutably and `contacts`/
  `efc_offsets` immutably. No overlap with the A matrix or b vector.
- `compute_contact_jacobian()` borrows `model` and `data` immutably, returns
  owned `DMatrix`. No change to borrow semantics.
- The existing `std::mem::take(&mut data.efc_lambda)` pattern (`:9642`) still
  works — `HashMap<K, Vec<f64>>` implements `Default` via `HashMap::new()`.
- Warmstart `HashMap<WarmstartKey, Vec<f64>>` uses heap-allocated values, which
  increases allocation pressure vs the current `[f64; 3]`. For typical contact
  counts (< 100), this is negligible. If profiling shows issues, a
  `SmallVec<[f64; 6]>` can be substituted without API changes.
- `data.efc_lambda.clone()` in RK4 (`:10566`) remains valid — both `HashMap`
  and `Vec<f64>` are `Clone`.

**Performance considerations.**

- For the common condim-3 case, the variable-dimension code should not be
  measurably slower than the current fixed-3 code. The `efc_offsets` lookup
  replaces a multiply by 3, and the `dim`-bounded loops have the same trip
  count. The `DMatrix`-based preconditioner blocks are heap-allocated where
  `Matrix3` was stack-allocated — for condim 3, this could be optimized with
  `SmallVec` or `enum { Fixed3(Matrix3), Dynamic(DMatrix) }` if profiling
  warrants it.
- For mixed-condim scenes, the variable block sizes mean the Delassus matrix
  has irregular structure. No special sparse format is needed — the existing
  dense `DMatrix` representation handles this. The off-diagonal sparsity
  optimization (`:7900, bodies_share_chain`) still works unchanged.

---

**Type change propagation.**

The `Contact.mu` expansion and variable-dim lambda propagate type changes
through many function signatures. This section consolidates all changes for
implementer reference.

**`[f64; 3] → Vec<f64>` warmstart chain:**

| Location | Current | After |
|:---|:---|:---|
| `Data.efc_lambda` (`:1573`) | `HashMap<WarmstartKey, [f64; 3]>` | `HashMap<WarmstartKey, Vec<f64>>` |
| `pgs_solve_contacts()` (`:8028`) param | `efc_lambda: &mut HashMap<..., [f64; 3]>` | `&mut HashMap<..., Vec<f64>>` |
| `pgs_solve_contacts()` return | `(Vec<Vector3<f64>>, usize)` | `(Vec<DVector<f64>>, usize)` |
| `pgs_solve_with_system()` (`:8049`) param + return | same as above | same as above |
| `cg_solve_contacts()` (`:8244`) param | same efc_lambda change | same |
| `cg_solve_contacts()` return | `Result<(Vec<Vector3<f64>>, usize), (DMatrix, DVector)>` | `Result<(Vec<DVector<f64>>, usize), (DMatrix, DVector)>` — error type unchanged |
| Touch sensor (`:5805`) | `lambda[0]` via `[f64; 3]` | `lambda[0]` via `Vec<f64>` — logic unchanged |
| RK4 (`:10566`) | `data.efc_lambda.clone()` | Works unchanged — `HashMap<K, Vec>` is `Clone` |
| Test code (`:13596`, `:13657`) | `HashMap<WarmstartKey, [f64; 3]>` | Update to `Vec<f64>` |

**`Vector3 → DVector` forces chain:**

| Location | Current | After |
|:---|:---|:---|
| `extract_forces()` (`:8010`) return | `Vec<Vector3<f64>>` | `Vec<DVector<f64>>` |
| `mj_fwd_constraint()` force loop (`:9731`) | `lambda.x / .y / .z` | `lambda[0] / [1] / [2]` |
| `SolverType::CGStrict` fallback (`:9714–9716`) | `vec![Vector3::zeros(); n]` | `data.contacts.iter().map(\|c\| DVector::zeros(c.dim)).collect()` |

---

**Known limitation: `J^T * λ` vs manual force application.** The current code
applies contact forces by manually traversing the kinematic chain
(`apply_contact_force()`), not by computing `J^T * λ`. This decision is
documented at `:9724–9730`. For condim 4/6 we add `apply_contact_torque()`
which is the angular analogue. An alternative would be to use the pre-computed
Jacobian: `qfrc_constraint += Jᵢᵀ * λᵢ` for each contact. This would be
mathematically equivalent and avoid the separate torque function, but would
couple force application to the Jacobian representation. The manual approach
is retained for consistency with the existing design and readability.

**Not in scope: pyramidal friction cones.** See sub-task D.2 rationale.
Pyramidal cones require a fundamentally different variable count per contact
(2(d-1) vs d), different Jacobian basis construction, and changes to every
solver loop. This is better implemented as a separate task once elliptic cones
are working.

**Not in scope: per-pair condim override.** `<contact><pair condim="...">` is
task #3 (contact pair/exclude). This task reads condim from `geom_condim` only.

**Not in scope: anisotropic sliding from MJCF.** MuJoCo supports different
`mu[0]` and `mu[1]` (sliding1 ≠ sliding2) per contact, but only via
`<contact><pair>` overrides. Per-geom friction has a single sliding value that
fills both slots. The elliptic projection handles anisotropic `mu` correctly
if provided by a future pair override.

**Not in scope: geom priority.** MuJoCo uses `geom/@priority` to break ties
in condim/friction combination. Our parser does not read `priority`. We use
`max(condim1, condim2)` unconditionally.

**Not in scope: QCQP-based cone projection.** MuJoCo's PGS solver uses a
per-contact QCQP (quadratically constrained QP) to jointly project normal and
friction forces onto the cone. Our solver uses sequential SOC projection
(clamp normal, then scale friction). See D.1 divergence analysis. Upgrading
to QCQP projection would improve convergence for strongly coupled contacts
but is a separate optimization task.

**Not in scope: friction combination method.** MuJoCo uses element-wise max
for combining friction between equal-priority geoms. We use geometric mean.
See A.3 rationale.

#### Acceptance Criteria

**Pre-requisite regression (1 criterion):**
0. Pre-req 0 regression: condim-3 solver forces are bit-identical before and
   after switching from `build_tangent_basis()` to `contact.frame`. Run the
   existing contact test suite — no output changes.

**Condim correctness (7 criteria):**
1. condim 1 contact produces zero tangential force — only `λ_n ≥ 0`.
2. condim 3 contact matches current behavior exactly (regression).
3. condim 4 contact limits torsional moment: `|λ_torsional| ≤ μ_torsional *
   λ_n`. A spinning top on a surface with `condim=4` decelerates; with
   `condim=3` it spins forever.
4. condim 6 contact limits rolling moment: `|λ_rolling| ≤ μ_rolling * λ_n`.
   A ball rolling on a surface with `condim=6` decelerates; with `condim=3`
   it rolls forever.
5. Mixed condim contacts in the same scene produce correct forces for each
   contact independently. Two contacts with condim 1 and condim 6 coexist.
6. Contact condim = `max(geom1.condim, geom2.condim)` — a condim-1 geom
   touching a condim-3 geom produces a condim-3 contact.
7. A model with no condim attributes (all defaults = 3) behaves identically
   to current behavior.

**Friction cone correctness (3 criteria):**
8. Elliptic cone projection with isotropic friction (`mu[0] == mu[1]`) matches
   current circular cone behavior (regression).
9. Elliptic cone projection with anisotropic friction
   (`mu[0] = 1.0, mu[1] = 0.5`) produces the correct elliptical boundary.
10. `Model.cone = 0` (pyramidal) emits a warning and falls back to elliptic.

**Solver correctness (3 criteria):**
11. PGS solver converges for mixed-condim scenes.
12. CG solver converges for mixed-condim scenes.
13. CG → PGS fallback works with variable block sizes. Test by setting
    `max_iterations=1` for CG (force non-convergence), verify PGS fallback
    produces physically reasonable forces for a mixed-condim scene.

**Jacobian correctness (2 criteria):**
14. Contact Jacobian row 3 (torsional) maps joint velocities to relative
    angular velocity about the contact normal. Verified by finite difference:
    perturb a hinge joint angle by ε, recompute contact angular velocity
    about normal, compare `(ω(q+ε) - ω(q))/ε` to `J[3, dof]`.
15. Contact Jacobian rows 4–5 (rolling) map to angular velocity in tangent
    plane. Same finite-difference verification.

**Edge cases (3 criteria):**
16. A scene with only condim-1 contacts (all frictionless) produces correct
    normal-only forces and the system `nefc = ncon`.
17. Warmstart works across frames when condim is constant; warmstart is
    discarded when a contact's condim changes between frames. *(Unit test on
    warmstart load logic: verify `prev.len() != contact.dim` triggers discard,
    not integration test — dynamic condim changes are unusual in simulation.)*
18. Zero contacts still produces an empty solution (no crash from
    `efc_offsets` being empty).

#### Test Models

Concrete MJCF models for acceptance criteria tests. All use
`sim_mjcf::load_model(xml)` and step via `data.step()`.

**Frictionless contact (criteria 1, 16):**
```xml
<mujoco>
  <option timestep="0.001"/>
  <worldbody>
    <body pos="0 0 0.1">
      <freejoint/>
      <geom type="sphere" size="0.05" condim="1"/>
    </body>
    <geom type="plane" size="1 1 0.1" condim="1"/>
  </worldbody>
</mujoco>
```
- Apply lateral force via `qfrc_applied`
- Assert: zero tangential constraint force in `efc_lambda` (only normal)
- Assert: `nefc == ncon` (1 row per contact)

**Torsional friction (criteria 3):**
```xml
<mujoco>
  <option timestep="0.001"/>
  <worldbody>
    <body pos="0 0 0.06">
      <freejoint/>
      <geom type="cylinder" size="0.05 0.01" condim="4"
            friction="1.0 0.1 0.0001"/>
    </body>
    <geom type="plane" size="1 1 0.1" condim="4"
          friction="1.0 0.1 0.0001"/>
  </worldbody>
</mujoco>
```
- Initialize with angular velocity about z-axis: `qvel[5] = 10.0` (spinning)
- Step 200 times
- Assert: `qvel[5]` magnitude decreases (torsional friction decelerates spin)
- Control: same model with `condim="3"` — `qvel[5]` unchanged (no torsional)

**Rolling friction (criteria 4):**
```xml
<mujoco>
  <option timestep="0.001"/>
  <worldbody>
    <body pos="0 0 0.06">
      <freejoint/>
      <geom type="sphere" size="0.05" condim="6"
            friction="1.0 0.005 0.01"/>
    </body>
    <geom type="plane" size="1 1 0.1" condim="6"
          friction="1.0 0.005 0.01"/>
  </worldbody>
</mujoco>
```
- Initialize with linear velocity: `qvel[0] = 1.0` (rolling along x)
- Step 200 times
- Assert: `qvel[0]` magnitude decreases (rolling friction decelerates)
- Control: same model with `condim="3"` — velocity unchanged (no rolling)

**Mixed condim (criteria 5, 6):**
```xml
<mujoco>
  <option timestep="0.001"/>
  <worldbody>
    <body name="ball" pos="-0.3 0 0.06">
      <freejoint/>
      <geom type="sphere" size="0.05" condim="6"
            friction="1.0 0.005 0.01"/>
    </body>
    <body name="ice" pos="0.3 0 0.06">
      <freejoint/>
      <geom type="sphere" size="0.05" condim="1"/>
    </body>
    <geom type="plane" size="1 1 0.1" condim="3"
          friction="1.0 0.005 0.0001"/>
  </worldbody>
</mujoco>
```
- Assert: ball-plane contact has condim = max(6, 3) = 6
- Assert: ice-plane contact has condim = max(1, 3) = 3
- Assert: both contacts coexist and produce correct forces

**Elliptic cone projection unit tests (criteria 8, 9, 10):**

These are unit tests on `project_elliptic_cone()` directly (no MJCF model needed).

- **Criterion 8 (isotropic regression):** Call `project_elliptic_cone` with
  isotropic `mu = [0.5, 0.5, 0.0, 0.0, 0.0]`, `dim = 3` for several input
  vectors: inside cone `(2, 0.5, 0.5)`, outside cone `(1, 2, 2)`, negative
  normal `(-1, 0.5, 0.5)`, zero friction `(1, 0, 0)`. Verify output matches
  the existing `project_friction_cone()` for each case.
- **Criterion 9 (anisotropic elliptic):** Call with `mu = [1.0, 0.5, 0.0, 0.0, 0.0]`,
  `dim = 3`, input `(1, 0.8, 0.4)`. Check: `(0.8/1.0)² + (0.4/0.5)² = 1.28 > 1`
  → outside cone. Projected point must satisfy
  `(λ₁/μ₁)² + (λ₂/μ₂)² = λ₀²` (on boundary). Also verify the projection is
  tighter in the smaller-μ direction.
- **Criterion 10 (pyramidal warning):** Set `Model.cone = 0`, load any model,
  verify `tracing::warn!` is emitted and solver uses elliptic projection.

#### Jacobian Finite-Difference Verification (criteria 14–15)

**Setup:**
- Model: single hinge joint, body with sphere geom resting on plane, condim=4
- ε = 1e-7 (joint angle perturbation)
- Tolerance: `|J_fd - J_analytical| / max(|J_analytical|, 1e-8) < 1e-4`

**Procedure:**
1. Set joint angle `q`, run `mj_fwd_position()` to get body poses
2. Detect contact, compute analytical Jacobian via `compute_contact_jacobian()`
3. Extract row 3 (torsional): `J_analytical = J[3, :]`
4. Set joint angle `q + ε`, run `mj_fwd_position()` again
5. Compute angular velocity of body about contact normal at both poses:
   `ω_n(q) = (contact_normal · body_angular_velocity(q))`
6. Finite-difference: `J_fd[dof] = (ω_n(q+ε) - ω_n(q)) / ε`
7. Compare element-wise: `|J_fd[dof] - J_analytical[dof]| < tol`

Repeat for rows 4–5 (rolling) with condim=6, projecting onto tangent1/tangent2
instead of normal.

#### Files

**Phase 2 changes — `sim/L0/core/src/mujoco_pipeline.rs`:**

| Item | Sub-task | Line | Change |
|------|----------|:----:|--------|
| `project_elliptic_cone()` | D.1 | new | New function for SOC projection (insert near `:8236`) |
| `project_friction_cone()` | D.4 | :8236 | Update: condim dispatch via `efc_offsets` |
| `pgs_solve_with_system()` | E.1 | :8291 | Update: use `efc_offsets`, variable-dim GS sweep |
| `pgs_solve_contacts()` | E.1b | :8270 | Update: thread `efc_offsets`, return `Vec<DVector>` |
| `cg_solve_contacts()` | E.5 | :8486 | Update: variable-dim, return `Vec<DVector>` |
| `compute_block_jacobi_preconditioner()` | E.3 | :8400~ | Update: `DMatrix` blocks instead of `Matrix3` |
| `apply_preconditioner()` | E.4 | :8420~ | Update: `DVector` blocks instead of `Vector3` |
| `extract_forces()` | E.6 | :8252 | Update: return `Vec<DVector<f64>>` |
| `Data.efc_lambda` | E.2 | :1677 | Update: `HashMap<K, [f64;3]>` → `HashMap<K, Vec<f64>>` |
| `apply_contact_torque()` | F.1 | new | New function (insert near `apply_contact_force` `:9600~`) |
| `mj_fwd_constraint()` force loop | F.2 | :9731~ | Update: add torque application for condim ≥ 4 |
| `Model::empty()` | G.1 | :1400~ | Update: `cone: 0` → `cone: 1` (default to elliptic) |
| Touch sensor | E.2 | :5805~ | Update: type annotation for `Vec` access |

**Phase 2 changes — `sim/L0/mjcf/src/model_builder.rs`:**
- `model_from_mjcf()` — pyramidal cone warning (sub-task G.1)

**Phase 2 changes — `sim/L0/tests/integration/`:**
- New file `condim_friction.rs` with acceptance criteria tests
- Register in `mod.rs`

**Already completed in Phase 1 (no changes needed):**
- `compute_contact_jacobian()` — uses `contact.frame[]`, returns `dim×nv`
- `assemble_contact_system()` — uses `contact.frame[]`, variable-dim blocks
- `compute_efc_offsets()` — already implemented
- `add_angular_jacobian()` — already implemented
- `Model.geom_condim` — already added
- `Contact.mu: [f64; 5]` — already expanded
- `Contact::with_condim()` — already implemented
- `make_contact_from_geoms()` — already updated
- Test struct literals — already updated to `[f64; 5]`
- `process_geom()` condim validation — already done

#### Design Decisions

| # | Question | Decision | Rationale |
|:---:|:---|:---|:---|
| D1 | Unify tangent basis functions as pre-req? | **Yes** | Eliminates consistency risk; zero marginal cost since condim touches all call sites |
| D2 | Default `cone` to elliptic? | **Yes** | Our default should match what we implement; pyramidal warn only on explicit MJCF request |
| D3 | Friction combination: keep geometric mean? | **Yes** | Defer to separate conformance task — changing behavior is orthogonal to condim |
| D4 | `Contact.friction` field: keep or remove? | **Keep** | Retained for backward compat; documented as `== mu[0]`; solver reads `mu` |
| D5 | Warmstart: `Vec<f64>` or `SmallVec<[f64; 6]>`? | **`Vec<f64>`** | Optimize later if profiling shows heap allocation pressure |
| D6 | `contact.frame` fields: use everywhere? | **Yes** | See D1; pre-computed frame ensures consistency across Jacobian/assembly/forces |

---

### 3. `<contact>` Pair/Exclude
**Status:** Done | **Effort:** M–L | **Prerequisites:** None

#### Current State

The two-mechanism contact architecture is fully implemented:

- `<contact><pair>` and `<contact><exclude>` are parsed from MJCF
- Mechanism 1 (automatic pipeline): `check_collision_affinity` checks body-pair
  excludes and explicit pair-set before same-body/parent-child/bitmask filters
- Mechanism 2 (explicit pairs): dedicated loop in `mj_collision` bypasses all
  kinematic and bitmask filters, applies rbound+margin distance cull, narrow-phase,
  and `apply_pair_overrides` for per-pair parameter overrides
- Defaults class resolution via `DefaultResolver::apply_to_pair` and
  `merge_pair_defaults`
- Geom-combination fallbacks for unspecified attributes in `process_contact`
- Duplicate pair deduplication (last-wins, canonical key)
- Body-pair excludes via `HashSet` for O(1) lookup

#### Objective

Parse `<contact><pair>` and `<contact><exclude>` from MJCF and apply them in the
collision pipeline, matching MuJoCo's two-mechanism architecture.

#### Background: MuJoCo's Two-Mechanism Architecture

MuJoCo generates contact candidates via two **independent** mechanisms:

1. **Automatic pipeline:** Broad-phase (sweep-and-prune) → body-pair excludes →
   same-body filter → parent-child filter → contype/conaffinity bitmask → narrow-phase.
2. **Explicit pair pipeline:** `<pair>` entries are injected directly as additional
   candidates. They bypass the same-body filter, parent-child filter, and bitmask
   filter. They still go through a bounding-sphere distance cull and narrow-phase.

These mechanisms are **additive/independent**: an `<exclude>` removes body pairs
from mechanism 1 only. An explicit `<pair>` naming geoms on those same bodies still
produces contacts via mechanism 2. Excludes do not suppress explicit pairs.

#### Specification

##### A. MJCF Parsing

**`<contact>` element** — grouping container with no attributes of its own.

**`<pair>` sub-element** — references two geom names:

| Attribute | Type | Required | Default | Semantics |
|-----------|------|----------|---------|-----------|
| `name` | string | no | — | Identifier for this pair |
| `class` | string | no | — | Defaults class (inherits `<default><pair .../>`) |
| `geom1` | string | **yes** | — | First geom (by name) |
| `geom2` | string | **yes** | — | Second geom (by name) |
| `condim` | int | no | from geoms† | Contact dimensionality (1, 3, 4, 6) |
| `friction` | real(5) | no | from geoms† | 5D friction `[tan1, tan2, torsional, roll1, roll2]` |
| `solref` | real(2) | no | from geoms† | Solver reference (normal direction) |
| `solreffriction` | real(2) | no | = pair solref | Solver reference (friction directions) |
| `solimp` | real(5) | no | from geoms† | Solver impedance |
| `margin` | real | no | from geoms† | Distance threshold for contact activation |
| `gap` | real | no | from geoms† | Contact included if distance < margin - gap |

**†Divergence from MuJoCo defaults:** MuJoCo's `<pair>` has hard-coded defaults
independent of the referenced geoms (condim=3, friction={1,1,0.005,0.0001,0.0001},
solref={0.02,1}, solimp={0.9,0.95,0.001,0.5,2}, margin=0, gap=0). We instead
fall back to geom-combination rules when an attribute is unspecified. This means
a bare `<pair geom1="A" geom2="B"/>` with no overrides produces the same contact
parameters that the automatic pipeline would compute — which is more intuitive
for users who want to force a pair without changing its physics. MuJoCo's
approach means an override-free `<pair>` silently changes contact parameters
relative to the automatic pipeline.

**Array length handling:** The parser uses `parse_float_array` + `len() >= N`
checks, matching the existing pattern for `solimp`/`solref`/`o_friction`. If
fewer than the required number of elements are provided (e.g., `friction="0.5"`
instead of 5 values), the attribute is treated as if it were not specified
(falls through to geom-combination fallback). **Minor divergence:** MuJoCo's
pair friction is natively 5D — when fewer values are given, only the provided
positions are overwritten and the remaining positions keep their struct defaults
(`{1, 1, 0.005, 0.0001, 0.0001}`), e.g., `friction="0.5"` yields
`{0.5, 1, 0.005, 0.0001, 0.0001}`. Our approach instead falls through to the
geom-combination fallback, which produces reasonable values but differs from
MuJoCo for partial-array inputs. This edge case is rare in practice and our
behavior is safe (no silent corruption).

**Fallback rule:** Each attribute is independent. When unspecified (and not
inherited from a defaults class), it is computed from the two referenced geoms
using our existing combination rules (from `make_contact_from_geoms` +
`combine_solver_params`):
- `condim` → `max(geom1.condim, geom2.condim)` (matches MuJoCo for equal-priority
  geoms; we don't implement `geom/@priority` — see §I)
- `friction` → each geom's 3-element `[sliding, torsional, rolling]` is expanded
  to 5D as `[sliding, sliding, torsional, rolling, rolling]`, then per-element
  geometric mean across the two expanded vectors. **Known divergence:** MuJoCo
  uses element-wise max on 3D friction, then expands to 5D. We use geometric
  mean, matching our existing convention (see §2, "Known non-conformance").
  Fixing this is a separate conformance task.
- `solref` → element-wise minimum. **Known divergence:** MuJoCo uses
  `solmix`-weighted average for positive solref values (element-wise min only
  for non-positive/direct format). Since we don't implement `solmix` (see §I),
  element-wise min is our standing approximation.
- `solreffriction` → falls back to the pair's resolved `solref` (no geom-level
  counterpart exists; matches MuJoCo where `{0,0}` default means "use solref")
- `solimp` → element-wise maximum. **Known divergence:** MuJoCo uses
  `solmix`-weighted average. Same rationale as solref above.
- `margin` → `max(geom1.margin, geom2.margin)` (currently always 0.0; see §H.
  Matches MuJoCo — margin/gap always use max regardless of priority.)
- `gap` → `max(geom1.gap, geom2.gap)` (currently always 0.0; see §H)

**`<exclude>` sub-element** — references two body names:

| Attribute | Type | Required | Default | Semantics |
|-----------|------|----------|---------|-----------|
| `name` | string | no | — | Identifier for this exclusion |
| `body1` | string | **yes** | — | First body (by name) |
| `body2` | string | **yes** | — | Second body (by name) |

When specified, **all** geom pairs where one geom belongs to `body1` and the other
belongs to `body2` are excluded from the automatic collision pipeline.

**Example MJCF:**

```xml
<contact>
  <pair geom1="left_hand" geom2="right_hand" condim="1"/>
  <pair geom1="finger_tip" geom2="table_top"
        friction="0.8 0.8 0.01 0.001 0.001"
        solref="0.01 0.5"/>
  <exclude body1="upper_arm" body2="forearm"/>
</contact>
```

**Parser implementation (`parse_contact`):** Follow the `parse_sensors`/`parse_actuators`
container pattern (line ~1963). Loop on `Event::Start`/`Event::Empty`/`Event::End`:

- `b"pair"` — parse attributes into `MjcfContactPair`. Required attributes
  (`geom1`, `geom2`) use the existing `get_attribute_opt(...).ok_or_else(||
  MjcfError::missing_attribute("geom1", "pair"))?` pattern (see `parse_connect_attrs`
  line ~1411). Optional attributes use `parse_int_attr` / `parse_float_attr` /
  `get_attribute_opt` + `parse_float_array` + length check, matching existing
  geom/tendon parsing. For `Event::Start`, call `skip_element` after parsing
  (to consume any unexpected children); for `Event::Empty`, no skip needed.
- `b"exclude"` — parse `body1`/`body2` (required, same `missing_attribute` pattern)
  and optional `name`.
- `_` — `skip_element` for `Event::Start`, ignore for `Event::Empty`.
- Break on `Event::End(b"contact")`.

**Parser implementation (`parse_pair_defaults`):** Follow the `parse_tendon_defaults`
pattern (line ~472). Parse optional attributes from `BytesStart` into
`MjcfPairDefaults`. Uses `parse_int_attr` for `condim`, `parse_float_attr` for
`margin`/`gap`, and `get_attribute_opt` + `parse_float_array` + `len() >= N`
for `friction`/`solref`/`solreffriction`/`solimp`. Add `b"pair"` to both
`Event::Start` and `Event::Empty` branches in `parse_default` (line ~324/359),
since `<pair .../>` is typically self-closing.

##### B. MJCF Types (`sim/L0/mjcf/src/types.rs`)

All new types follow the existing crate convention for derives:
`#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]`.

```rust
/// Parsed `<contact><pair>` element.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfContactPair {
    pub name: Option<String>,
    pub class: Option<String>,
    pub geom1: String,
    pub geom2: String,
    pub condim: Option<i32>,
    /// 5-element friction: [tan1, tan2, torsional, roll1, roll2].
    pub friction: Option<[f64; 5]>,
    pub solref: Option<[f64; 2]>,
    pub solreffriction: Option<[f64; 2]>,
    pub solimp: Option<[f64; 5]>,
    pub margin: Option<f64>,
    pub gap: Option<f64>,
}

/// Parsed `<contact><exclude>` element.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfContactExclude {
    pub name: Option<String>,
    pub body1: String,
    pub body2: String,
}

/// Parsed `<contact>` element (grouping container).
#[derive(Debug, Clone, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfContact {
    pub pairs: Vec<MjcfContactPair>,
    pub excludes: Vec<MjcfContactExclude>,
}
```

Add `pub contact: MjcfContact` to `MjcfModel`. `MjcfModel` has a **manual
`Default` impl** (line ~2786, not `#[derive(Default)]`), so also add
`contact: MjcfContact::default(),` to the struct literal in
`impl Default for MjcfModel`.

##### C. Defaults Class Extension (`MjcfDefault`)

Add `pub pair: Option<MjcfPairDefaults>` to `MjcfDefault`:

```rust
/// Default pair parameters (from `<default><pair .../>`)
#[derive(Debug, Clone, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MjcfPairDefaults {
    pub condim: Option<i32>,
    pub friction: Option<[f64; 5]>,
    pub solref: Option<[f64; 2]>,
    pub solreffriction: Option<[f64; 2]>,
    pub solimp: Option<[f64; 5]>,
    pub margin: Option<f64>,
    pub gap: Option<f64>,
}
```

**`merge_defaults` update:** `DefaultResolver::merge_defaults` (line ~503) constructs
a `MjcfDefault` with merged fields for every default type. Add a `pair` field using
`merge_pair_defaults`:

```rust
pair: Self::merge_pair_defaults(parent.pair.as_ref(), child.pair.as_ref()),
```

`merge_pair_defaults` follows the same `(None, None) | (Some, None) | (None, Some) | (Some, Some)` pattern as `merge_joint_defaults` etc., using `c.field.or(p.field)` for
each `Option` field on `MjcfPairDefaults`. Without this, class inheritance chains
that set pair defaults on a parent class will silently fail to propagate to children.

##### D. Model Data Structures (`mujoco_pipeline.rs` → `Model`)

```rust
/// Explicit contact pair: geom indices + per-pair overrides.
/// All fields are fully resolved at build time (no Options).
#[derive(Debug, Clone)]
pub struct ContactPair {
    pub geom1: usize,
    pub geom2: usize,
    pub condim: i32,
    /// 5-element friction: [tan1, tan2, torsional, roll1, roll2].
    pub friction: [f64; 5],
    pub solref: [f64; 2],
    pub solreffriction: [f64; 2],
    pub solimp: [f64; 5],
    pub margin: f64,
    pub gap: f64,
}
```

Add to `Model`:

```rust
/// Explicit contact pairs from `<contact><pair>`.
/// Processed in mechanism 2 (bypass kinematic and bitmask filters).
pub contact_pairs: Vec<ContactPair>,
/// Explicit pair geom-pair set for O(1) lookup during automatic pipeline.
/// Canonical key: `(min(geom1, geom2), max(geom1, geom2))`.
/// Used to suppress automatic-pipeline contacts for pairs that have explicit overrides.
pub contact_pair_set: HashSet<(usize, usize)>,
/// Excluded body-pair set from `<contact><exclude>`.
/// Canonical key: `(min(body1, body2), max(body1, body2))`.
pub contact_excludes: HashSet<(usize, usize)>,
```

Initialize in `Model::empty()`:

```rust
contact_pairs: vec![],
contact_pair_set: HashSet::new(),
contact_excludes: HashSet::new(),
```

This is backward-compatible with all existing test Model constructors (they use
`Model::empty()` + field assignment and will inherit the empty defaults).

Import uses `use std::collections::{HashMap, HashSet};`.

**Why `HashSet` for excludes:** The exclude check runs in the hot loop for every
candidate pair from the automatic pipeline. `O(1)` lookup vs `O(n)` linear scan
on a `Vec`. Canonical ordering `(min, max)` ensures symmetry: excluding (A, B)
also excludes (B, A).

**Why `contact_pair_set`:** Prevents duplicate contacts. When a geom pair appears
both in the automatic pipeline (via bitmask match) and as an explicit `<pair>`,
only the explicit pair should produce a contact (it may carry overridden
parameters). Without deduplication, both mechanisms would produce a contact for
the same geom pair. The `contact_pair_set` is populated alongside `contact_pairs`
in the builder; the automatic pipeline checks it to skip pairs that will be
handled by mechanism 2.

##### E. Model Builder (`model_builder.rs`)

Two-stage resolution follows the existing pattern (e.g., geoms, tendons):

**Stage 1 — Defaults class merge (in `DefaultResolver`):**

Add `apply_to_pair(&self, pair: &MjcfContactPair) -> MjcfContactPair`. The
defaults class is resolved via `pair.class.as_deref()`, following the same
pattern as `apply_to_joint` / `apply_to_tendon`:

```rust
pub fn apply_to_pair(&self, pair: &MjcfContactPair) -> MjcfContactPair {
    let mut result = pair.clone();
    if let Some(defaults) = self.pair_defaults(pair.class.as_deref()) {
        if result.condim.is_none() { result.condim = defaults.condim; }
        if result.friction.is_none() { result.friction = defaults.friction; }
        if result.solref.is_none() { result.solref = defaults.solref; }
        if result.solreffriction.is_none() { result.solreffriction = defaults.solreffriction; }
        if result.solimp.is_none() { result.solimp = defaults.solimp; }
        if result.margin.is_none() { result.margin = defaults.margin; }
        if result.gap.is_none() { result.gap = defaults.gap; }
    }
    result
}
```

**Stage 2 — Geom/body fallback resolution (new `process_contact` method):**

Add `builder.process_contact(&mjcf.contact)?;` to `model_from_mjcf()` after
body tree processing (which populates `geom_name_to_id` and `body_name_to_id`)
and before `builder.build()`. The only dependency is on name-to-id maps from
body tree processing, so it can go anywhere after that; the natural insertion
point is after `process_sensors` (line ~141) and before `build()` (line ~147):

```rust
// in model_from_mjcf(), after process_sensors (line ~141):
builder.process_contact(&mjcf.contact)?;

// Build final model
Ok(builder.build())
```

`process_contact` iterates:

1. For each `MjcfContactPair`:
   - Apply defaults: `let pair = self.resolver.apply_to_pair(mjcf_pair);`
   - Resolve `geom1`/`geom2` names via `geom_name_to_id`. Error if not found.
   - For each still-`None` attribute, compute from the two referenced geoms using
     the standard combination rules on the builder's geom arrays:
     - `condim` → `max(geom_condim[g1], geom_condim[g2])`
     - `friction` → `geom_friction[g]` is `Vector3<f64>` = `[sliding, torsional,
       rolling]`. Combine in 3D via geometric mean (`sqrt(f1[i] * f2[i])`), then
       expand to 5D: `[s, s, t, r, r]` where `s = sqrt(f1.x*f2.x)`,
       `t = sqrt(f1.y*f2.y)`, `r = sqrt(f1.z*f2.z)` (matches
       `make_contact_from_geoms` line ~3799 + `with_condim` line ~1377)
     - `solref` → `element-wise min(geom_solref[g1], geom_solref[g2])` (our
       `combine_solver_params`; see §A fallback rules for divergence note)
     - `solimp` → `element-wise max(geom_solimp[g1], geom_solimp[g2])` (same)
     - `margin` → `0.0` (geom-level margin is not yet parsed; `build()` initializes
       `geom_margin` as `vec![0.0; ngeom]`, and `ModelBuilder` does not store it
       as a field, so it is unavailable during `process_contact()`)
     - `gap` → `0.0` (same rationale as margin)
     - `solreffriction` → falls back to the pair's resolved `solref` value
       (there is no geom-level `solreffriction`; MuJoCo defaults it to solref)
   - Compute canonical key `let key = (g1.min(g2), g1.max(g2));`
   - **Deduplicate:** If `contact_pair_set` already contains `key`, find and
     replace the existing entry in `contact_pairs` (last-wins semantics,
     matching MuJoCo's signature-based merge). Use
     `contact_pairs.iter().position(|p| ...).ok_or_else(|| ...)` to locate
     the existing entry and overwrite it (returns `ModelConversionError` on
     invariant violation). Otherwise push a new entry and insert `key` into
     `contact_pair_set`.

2. For each `MjcfContactExclude`:
   - Resolve `body1`/`body2` names via `body_name_to_id`. Error if not found.
   - Insert `(min(id1, id2), max(id1, id2))` into `contact_excludes`.

**`build()` struct literal:** The `build()` function (line ~2105) constructs
`Model` via struct literal. Add the three new fields:

```rust
contact_pairs: self.contact_pairs,
contact_pair_set: self.contact_pair_set,
contact_excludes: self.contact_excludes,
```

These fields must also be declared on `ModelBuilder` and initialized empty in
`ModelBuilder::new()`.

##### F. Collision Pipeline Changes (`mj_collision`)

**Automatic pipeline (mechanism 1) — modify `check_collision_affinity`:**

Add exclude and explicit-pair checks before the existing filters:

```rust
fn check_collision_affinity(model: &Model, geom1: usize, geom2: usize) -> bool {
    let body1 = model.geom_body[geom1];
    let body2 = model.geom_body[geom2];

    // NEW: Check body-pair exclude list
    let exclude_key = (body1.min(body2), body1.max(body2));
    if model.contact_excludes.contains(&exclude_key) {
        return false;
    }

    // NEW: Skip if this geom pair has an explicit <pair> entry —
    // mechanism 2 will handle it with its overridden parameters.
    let pair_key = (geom1.min(geom2), geom1.max(geom2));
    if model.contact_pair_set.contains(&pair_key) {
        return false;
    }

    // Existing: same-body, parent-child, contype/conaffinity ...
}
```

This prevents duplicate contacts: a geom pair that appears both in the automatic
pipeline (via bitmask match) and as an explicit `<pair>` is only processed by
mechanism 2, which applies the correct overridden parameters.

**Explicit pair pipeline (mechanism 2) — add to `mj_collision`:**

After the existing automatic loop, process explicit pairs. These bypass the
SAP broad-phase, so we use a direct `geom_rbound`-based distance cull instead:

```rust
// Mechanism 2: explicit contact pairs (bypass kinematic + bitmask filters)
for pair in &model.contact_pairs {
    let geom1 = pair.geom1;
    let geom2 = pair.geom2;

    // Distance cull using bounding radii (replaces SAP broad-phase for pairs).
    // geom_rbound is the bounding sphere radius, pre-computed per geom.
    // For planes, rbound = INFINITY (line 402) so this check always passes.
    // Margin is added to match MuJoCo's mj_filterSphere: rbound1 + rbound2 + margin.
    // Currently pair.margin defaults to 0.0 (§H) so this has no runtime effect yet,
    // but keeps the code correct when margin-aware collision is implemented.
    let dist = (data.geom_xpos[geom1] - data.geom_xpos[geom2]).norm();
    if dist > model.geom_rbound[geom1] + model.geom_rbound[geom2] + pair.margin {
        continue;
    }

    // Narrow-phase collision detection
    let pos1 = data.geom_xpos[geom1];
    let mat1 = data.geom_xmat[geom1];
    let pos2 = data.geom_xpos[geom2];
    let mat2 = data.geom_xmat[geom2];

    if let Some(mut contact) = collide_geoms(model, geom1, geom2, pos1, mat1, pos2, mat2) {
        // Apply pair overrides to the contact
        apply_pair_overrides(&mut contact, pair);
        data.contacts.push(contact);
        data.ncon += 1;
    }
}
```

**No `geom_types_compatible` extraction needed.** `collide_geoms` already returns
`Option<Contact>` — it returns `None` for any geom type combination it can't
handle (unsupported types, non-penetrating geometry). The mechanism-2 loop
simply calls `collide_geoms` directly; if it returns `None`, no contact is
produced.

**`apply_pair_overrides(contact: &mut Contact, pair: &ContactPair)`:**

Overwrites the contact fields that `collide_geoms` populated from geom defaults:

```rust
fn apply_pair_overrides(contact: &mut Contact, pair: &ContactPair) {
    // condim → dim mapping (same logic as Contact::with_condim)
    contact.dim = match pair.condim {
        1 => 1, 3 => 3, 4 => 4, 6 => 6,
        0 | 2 => 3, 5 => 6, _ => 6,
    } as usize;
    // 5D friction: directly from pair (already fully resolved)
    contact.mu = pair.friction;
    contact.friction = pair.friction[0]; // legacy scalar = tan1
    // Solver params
    contact.solref = pair.solref;
    contact.solimp = pair.solimp;
    // NOTE: solreffriction is NOT applied here — Contact has a single solref
    // field; per-direction solver params require solver changes (see §G).
    // NOTE: margin/gap are NOT applied here — no runtime effect yet (see §H).
}
```

Note: `collide_geoms` internally calls `make_contact_from_geoms`, which combines
friction/condim/solref/solimp from the two geoms. For mechanism-2 contacts, this
combination is computed and then immediately overwritten by `apply_pair_overrides`.
This is intentional — keeping `collide_geoms` unchanged avoids threading pair
context through the entire narrow-phase dispatch tree. The redundant combination
is negligible cost (a few arithmetic ops per contact, dwarfed by GJK/EPA).

##### G. `solreffriction` Handling

The `Contact` struct currently has a single `solref` field. For now, store
`solreffriction` in the `ContactPair` but do **not** add it to `Contact` — it
only takes effect when the solver processes friction constraint rows, and the
current solver uses a single `solref` for all rows. When the solver is upgraded
to support per-direction solver params, `Contact` can be extended. Document this
as a known divergence.

**Out of scope (defer):** Applying `solreffriction` in the constraint solver.
Parse and store it so the data is available, but do not change solver behavior.

##### H. Margin/Gap: Parse and Store, No Runtime Effect Yet

Our collision pipeline does not currently use `geom_margin` or `geom_gap` anywhere:
`aabb_from_geom` builds AABBs from pure geometry (no margin inflation), and
`collide_geoms` detects only geometric penetration (no margin-expanded envelope).
Both fields default to 0.0 in all existing models.

For this task we **parse and store** pair-level `margin`/`gap` for data
completeness. Unspecified pair margin/gap fallback to 0.0 (geom-level
`margin`/`gap` are not yet parsed from MJCF — `ModelBuilder` does not store
them as fields, and `build()` initializes them as `vec![0.0; ngeom]`). Both
fields have **no runtime effect** until margin-aware collision detection is
implemented. That is a separate future task affecting both the automatic and
explicit pair pipelines equally.

##### I. Not in Scope

- **Geom `priority` attribute.** Our combination rules use `max(condim)` and
  geometric-mean friction unconditionally. This is documented in future_work_2
  §2 ("Not in scope: geom priority").
- **`solmix` attribute.** MuJoCo uses `solmix`-weighted averaging for
  solref/solimp combination (with element-wise min as fallback for non-positive
  solref). Our `combine_solver_params` uses element-wise min for solref and
  element-wise max for solimp. This is a pre-existing divergence in the
  automatic pipeline — not introduced by this task. The pair fallback rules
  (§A) use the same combination functions for consistency.
- **Margin-aware collision detection.** See §H above.

##### J. Implementation Notes

- **Duplicate `<pair>` deduplication (implemented).** If the same geom pair
  appears in multiple `<pair>` entries, `process_contact` deduplicates by
  canonical key `(min(g1, g2), max(g1, g2))`, keeping the **last** entry
  (last-wins, matching MuJoCo's behavior where signature-based merging in
  `engine_collision_driver.c` ensures at most one contact per geom pair).
  Without deduplication, duplicate entries would produce duplicate contacts at
  the same point — doubling contact force and over-constraining the pair. The
  `contact_pair_set` already uses `HashSet` so the suppression of
  automatic-pipeline duplicates is inherently idempotent. Duplicate `<exclude>`
  entries for the same body pair are likewise idempotent (`HashSet` insert is a
  no-op). The dedup lookup uses `.ok_or_else(|| ModelConversionError { .. })?`
  rather than `if let Some(pos)` to catch invariant violations between the
  `contact_pair_set` and `contact_pairs` vec.
- **`ContactPair` re-export.** `ContactPair` is re-exported from `sim_core`
  via `lib.rs` so that `sim_mjcf`'s `model_builder` can reference it.

**Edge cases (valid behavior, no special handling needed):**

- **Empty `<contact>` element:** `<contact></contact>` (parsed via `Event::Start`
  → `parse_contact()` returns empty `MjcfContact`) and `<contact/>` (parsed via
  `Event::Empty` → directly assign `MjcfContact::default()`) both produce an
  empty `MjcfContact`. `parse_mujoco` dispatches `b"contact"` in both the
  `Event::Start` branch (calling `parse_contact()`) and the `Event::Empty`
  branch (assigning `MjcfContact::default()`).
- **Self-pair (`geom1 == geom2`):** Allowed. `collide_geoms(g, g, ...)` returns
  `None` for all current geom types (zero depth / degenerate normal), so no
  contact is produced. No validation needed.
- **Unknown child elements inside `<contact>`:** Follow existing parser pattern:
  `skip_element` for `Event::Start`, ignore for `Event::Empty`. No error.
- **`<pair>` order preservation:** The order of `<pair>` entries in MJCF is
  preserved in `model.contact_pairs`. Mechanism-2 processes them in order, so
  contact order in `data.contacts` matches MJCF document order (after automatic
  contacts).

#### Acceptance Criteria

1. `<exclude>` prevents all contacts between geoms on the specified body pair,
   via the automatic pipeline.
2. `<exclude>` does **not** suppress an explicit `<pair>` between geoms on the
   same bodies (two-mechanism independence).
3. `<pair>` produces contacts between the specified geoms, bypassing same-body,
   parent-child, and contype/conaffinity filters.
4. `<pair>` still goes through bounding-sphere distance cull and narrow-phase
   (no contacts when geoms are far apart or non-penetrating).
5. No duplicate contacts: a geom pair that matches both the automatic pipeline
   and an explicit `<pair>` produces exactly one contact (from mechanism 2).
6. Per-pair `condim` override changes contact dimensionality for that pair.
7. Per-pair 5D `friction` override replaces the geom-combined friction
   (both `mu` array and legacy `friction` scalar on `Contact`).
8. Per-pair `solref`/`solimp` overrides replace the geom-combined solver params.
9. Per-pair `margin`/`gap` are parsed and stored but have no runtime effect (see §H).
10. Per-pair `solreffriction` defaults to the pair's resolved `solref` when
    unspecified (no geom-level fallback exists).
11. When pair attributes are unspecified, they fall back to the standard geom
    combination rules (identical behavior to automatic contacts).
12. Existing contype/conaffinity filtering is unchanged for non-paired/excluded
    geoms (regression test).
13. Unknown geom/body names in `<pair>`/`<exclude>` produce a clear
    `ModelConversionError` naming the bad reference and the element
    (e.g., `"<pair> references unknown geom 'foo' in geom1"`).
14. `<pair>` `class` attribute inherits from the referenced defaults class.
15. `<exclude body1="X" body2="X"/>` is accepted without error (no-op; same-body
    contacts are already filtered by the automatic pipeline).
16. Duplicate `<pair>` entries for the same geom pair produce exactly one
    `ContactPair` (last-wins deduplication). Only one contact is generated,
    not two.
17. Missing required attributes (`geom1`/`geom2` on `<pair>`,
    `body1`/`body2` on `<exclude>`) produce `MjcfError::MissingAttribute`
    at parse time.

#### Conformance Tests

- **Exclude basic:** Two bodies with geoms that normally collide. Add `<exclude>`.
  Verify zero contacts.
- **Exclude + pair independence:** Exclude bodies A/B. Add explicit `<pair>` for
  geoms on A/B. Verify the pair still produces contacts.
- **Exclude self-body:** `<exclude body1="A" body2="A"/>`. Verify no error (no-op,
  same-body contacts are already filtered). *(AC 15)*
- **Pair bypass bitmask:** Two geoms with `contype=0 conaffinity=0` (would never
  collide automatically). Add `<pair>`. Verify contacts are produced. *(AC 3)*
- **Pair bypass parent-child:** Parent and child body geoms (normally filtered).
  Add `<pair>`. Verify contacts are produced. *(AC 3)*
- **Pair bypass same-body:** Two geoms on the same body (normally filtered by
  same-body check). Add `<pair>`. Verify contacts are produced. *(AC 3)*
- **Pair no duplicate:** Two geoms with matching bitmasks (would collide
  automatically) plus an explicit `<pair>` with `condim="1"`. Verify exactly
  one contact with `dim == 1` (mechanism-2 only, not both).
- **Pair condim override:** Pair with `condim="1"` between two `condim="3"` geoms.
  Verify contact `dim == 1`.
- **Pair friction override:** Pair with explicit 5D friction. Verify contact `mu`
  matches the pair, not the geom combination.
- **Pair partial override:** Pair with only `condim` specified. Verify other
  params (friction, solref, solimp) match the geom-combined values.
- **Pair with class:** Pair with `class="custom"` referencing a defaults class
  that sets `condim="4"`. Verify contact `dim == 4`.
- **Pair solref/solimp override:** Pair with explicit `solref="0.05 0.8"` and
  `solimp="0.85 0.9 0.002 0.4 1.5"`. Verify contact `solref` and `solimp`
  match the pair values, not the geom-combined values. *(AC 8)*
- **Pair margin/gap stored:** Parse a `<pair margin="0.1" gap="0.05" .../>`.
  Verify the `ContactPair` in `model.contact_pairs` has `margin == 0.1` and
  `gap == 0.05`. Verify no runtime effect on contact generation (contacts are
  produced identically with or without margin/gap). *(AC 9)*
- **Pair solreffriction default:** Parse a `<pair solref="0.05 0.8" .../>` with
  no `solreffriction` attribute. Verify `ContactPair.solreffriction == [0.05, 0.8]`
  (matches the pair's resolved solref). *(AC 10)*
- **Pair distant geoms:** Two geoms far apart with explicit `<pair>`. Verify no
  contact produced (bounding-sphere cull works). *(AC 4)*
- **Parse error — bad geom name:** `<pair geom1="nonexistent" .../>`. Verify
  error is `ModelConversionError` and message contains `"nonexistent"` and
  `"geom1"`. *(AC 13)*
- **Parse error — bad body name:** `<exclude body1="nonexistent" .../>`. Verify
  error is `ModelConversionError` and message contains `"nonexistent"` and
  `"body1"`. *(AC 13)*
- **Duplicate pair dedup:** Two `<pair>` entries for the same geom pair, first
  with `condim="1"`, second with `condim="4"`. Verify `model.contact_pairs`
  contains exactly one entry for that geom pair with `condim == 4` (last wins).
  Verify exactly one contact is produced, with `dim == 4`. *(AC 16)*
- **Parse error — missing geom1:** `<pair geom2="foo"/>` (no `geom1`). Verify
  error is `MjcfError::MissingAttribute` with `attribute == "geom1"`. *(AC 17)*
- **Parse error — missing body1:** `<exclude body2="bar"/>` (no `body1`). Verify
  error is `MjcfError::MissingAttribute` with `attribute == "body1"`. *(AC 17)*
- **Regression:** Run existing contact test suite unchanged. No output changes.
  *(AC 12)*

#### Files

| File | Change | Details |
|------|--------|---------|
| `sim/L0/mjcf/src/types.rs` | modify | Add `MjcfContactPair`, `MjcfContactExclude`, `MjcfContact`, `MjcfPairDefaults` (all with serde cfg_attr); add `contact` field to `MjcfModel` + update manual `Default` impl; add `pair` field to `MjcfDefault` |
| `sim/L0/mjcf/src/parser.rs` | modify | Add `b"contact"` arm in `parse_mujoco` dispatch in **both** `Event::Start` (line ~67, calls `parse_contact()`) **and** `Event::Empty` (line ~111, assigns `MjcfContact::default()` for self-closing `<contact/>`); new `parse_contact()` function for `<pair>`/`<exclude>` children; add `b"pair"` arm in `parse_default` (both `Event::Start` and `Event::Empty` branches, since `<pair .../>` is typically self-closing) |
| `sim/L0/mjcf/src/defaults.rs` | modify | Add `MjcfContactPair`, `MjcfPairDefaults` to imports; add `pair_defaults()` accessor, `apply_to_pair()` method, and `merge_pair_defaults()` to `DefaultResolver`; add `pair` field to `merge_defaults()` struct literal |
| `sim/L0/mjcf/src/model_builder.rs` | modify | Add `HashSet` import (line ~21); add `ContactPair` import from sim_core; add `MjcfContact` to types import; add `contact_pairs`, `contact_pair_set`, `contact_excludes` fields to `ModelBuilder`; new `process_contact()` method (resolve names, apply defaults, compute geom fallbacks incl. `solreffriction` → `solref`); call `process_contact()` in `model_from_mjcf()` after body tree; move fields into `Model` in `build()` struct literal |
| `sim/L0/core/src/mujoco_pipeline.rs` | modify | Add `ContactPair` struct and `contact_pairs`/`contact_pair_set`/`contact_excludes` to `Model`; add `HashSet` import; initialize new fields in `Model::empty()`; add exclude + pair-set checks in `check_collision_affinity`; add mechanism-2 loop in `mj_collision` (rbound cull + `collide_geoms` + `apply_pair_overrides`) |
| `sim/L0/core/src/lib.rs` | modify | Add `ContactPair` to re-exports from `mujoco_pipeline` |

---

### 4. Spatial Tendons + Wrapping
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State
Fixed tendons are fully integrated into the pipeline (`mujoco_pipeline.rs:7384–7431`).
Spatial tendons are scaffolded — type dispatch exists but produces zeros at runtime.

sim-tendon crate has standalone implementations:

| Component | File | Status |
|-----------|------|--------|
| `SpatialTendon` | `spatial.rs` | Standalone — path computation, force transmission |
| `SphereWrap` | `wrapping.rs` | Standalone — geodesic wrapping |
| `CylinderWrap` | `wrapping.rs` | Standalone — pulley-style wrapping |
| `PulleySystem` | `pulley.rs` | Standalone — compound pulleys |
| `TendonPath` | `path.rs` | Standalone — segment caching |

The pipeline has placeholder stubs:
- `ActuatorTransmission::Site => {}` at lines 2180, 2222, 5486, 6449, 6688

#### Objective
Wire spatial tendon length/velocity computation and wrapping geometry into the
pipeline so that spatial tendons produce correct forces and site-transmission
actuators work.

#### Specification

1. **Register spatial tendons** in `Model` — store wrap site references, attachment
   body indices, wrap geometry (sphere/cylinder) parameters.
2. **Compute spatial tendon length** in `mj_fwd_tendon()` — call into sim-tendon's
   path computation for each spatial tendon. Wrap geometry evaluated against current
   body poses.
3. **Compute spatial tendon velocity** — finite difference or analytical Jacobian
   of tendon length w.r.t. joint velocities.
4. **Force transmission** — spatial tendon forces applied to attached bodies via
   path Jacobian (same pattern as fixed tendons but with wrap-dependent routing).

#### Acceptance Criteria
1. Spatial tendon length matches MuJoCo for a tendon wrapped around a sphere.
2. Spatial tendon length matches MuJoCo for a tendon wrapped around a cylinder.
3. Actuator force transmitted through spatial tendon produces correct joint torques.
4. Zero-wrap-site tendons (straight-line spatial) match fixed tendon equivalent.
5. Tendon velocity is correct (verified by finite-difference comparison).

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (spatial tendon computation in
  `mj_fwd_tendon()`, Model fields for wrap geometry)
- `sim/L0/tendon/src/` — reference (existing spatial/wrapping implementation)
- `sim/L0/mjcf/src/model_builder.rs` — modify (spatial tendon MJCF → Model wiring)

---

### 5. Site-Transmission Actuators
**Status:** Not started | **Effort:** M | **Prerequisites:** #4

#### Current State
`ActuatorTransmission::Site` enum variant exists and is parsed from MJCF.
Five dispatch points in the pipeline are stubs (`=> {}`):

- `mujoco_pipeline.rs:2180` — length computation
- `mujoco_pipeline.rs:2222` — velocity computation
- `mujoco_pipeline.rs:5486` — Jacobian computation
- `mujoco_pipeline.rs:6449` — force application
- `mujoco_pipeline.rs:6688` — Phase 2 actuation

#### Objective
Implement site-based actuator transmission so that actuator forces can be applied
at arbitrary body sites.

#### Specification
Site transmission computes actuator length as the distance (or projected distance)
from the site to a reference. Force is applied at the site frame and transmitted
to the parent body via the site Jacobian (3×nv or 6×nv matrix mapping joint
velocities to site linear/angular velocity).

Requires spatial tendon infrastructure (#4) for the Jacobian computation — a
site-transmission actuator is effectively a degenerate spatial tendon with a
single attachment point.

#### Acceptance Criteria
1. Site actuator length equals site position projected onto actuator axis.
2. Site actuator force produces correct joint torques via Jacobian transpose.
3. Site actuators work with all gain/bias types (Fixed, Affine, Muscle).

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (fill in 5 `Site => {}` stubs)
