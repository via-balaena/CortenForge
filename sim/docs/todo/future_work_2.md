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
**Status:** Not started | **Effort:** L | **Prerequisites:** None

#### Current State
The contact solver is hardcoded to condim 3 (normal + 2D tangential friction).
Twelve functions and data structures must change.

**What exists (solver — `mujoco_pipeline.rs`):**
- `nefc = ncon * 3` hardcoded at three locations (`:7806`, `:8062`, `:8257`)
- `compute_contact_jacobian()` (`:7619`) returns a fixed 3×nv matrix (rows:
  normal, tangent1, tangent2) — no angular rows exist
- `assemble_contact_system()` (`:7799`) builds 3×3 blocks for the Delassus
  matrix. `M⁻¹ * Jᵀ` solve loop (`:7828`) iterates `for col in 0..3`
- `project_friction_cone()` (`:7994`) implements circular cone only
  (`‖λ_t‖ ≤ μ * λ_n`)
- `compute_block_jacobi_preconditioner()` (`:8158`) extracts `Matrix3` blocks
- `apply_preconditioner()` (`:8200`) operates on `Vector3` blocks
- `pgs_solve_with_system()` (`:8049`) uses `base = i * 3`, inline 3-row
  residual/update/projection
- `cg_solve_contacts()` (`:8244`) uses `nefc = ncon * 3`, single-contact direct
  solve with `Matrix3` / `Vector3`, warmstart as `[f64; 3]`
- `extract_forces()` (`:8010`) returns `Vec<Vector3<f64>>`
- Force application in `mj_fwd_constraint()` (`:9731`) converts lambda to a
  world-frame `Vector3` force and calls `apply_contact_force()` — no torque path

**What exists (contact creation):**
- `make_contact_from_geoms()` (`:3580`) only reads `geom_friction[i].x` (sliding)
  and combines via `sqrt(μ₁ * μ₂)`. Torsional (`.y`) and rolling (`.z`) are
  **never read**
- `Contact::with_solver_params()` (`:1277`) sets `dim: if friction > 0.0 { 3 }
  else { 1 }` and `mu: [friction, friction * 0.005]` — dim is stored but never
  consumed by the solver
- `Contact.mu` is `[f64; 2]` — MuJoCo uses 5 friction coefficients per contact

**What exists (model / MJCF):**
- `MjcfGeom.condim` (`types.rs:836`) is parsed from XML (`parser.rs:921`) with
  default 3, but `model_builder.rs` **never propagates it to `Model`** — there
  is no `Model.geom_condim` field
- `Model.cone: u8` (`:1112`) is parsed and stored (0=pyramidal, 1=elliptic)
  but **ignored** by the solver
- `geom_friction: Vec<Vector3<f64>>` stores all three friction components per
  geom but only `.x` is consumed

**Net result:**
- **condim 1** (frictionless): Not supported — all contacts get friction rows
- **condim 3** (sliding friction): Only working mode
- **condim 4** (+ torsional): Torsional friction silently dropped
- **condim 6** (+ rolling): Rolling friction silently dropped
- **Elliptic/pyramidal cones**: Parsed, ignored — circular cone always used

#### Objective
Support the full range of MuJoCo contact dimensionalities (1, 3, 4, 6) and
both friction cone types (elliptic, pyramidal) so that the solver generates
correct constraint forces for all contact configurations.

#### Specification

This task is organized into seven sub-tasks. Each is independently testable.
Dependency order:

```
A (model plumbing) ──→ B (Jacobian) ──→ C (system assembly) ──→ E (solvers)
                                                              ↗
                       D (cone projection) ──────────────────┘
                       F (force application) ← E
                       G (cone validation) — independent
```

A must land first (all downstream code reads `Contact.dim` and `Contact.mu`).
B and D can be developed in parallel. C depends on B (Jacobian sizes).
E depends on C and D. F depends on E (return type change).

---

**Sub-task A: Model plumbing — `geom_condim` + friction propagation**

**A.1.** Add `geom_condim: Vec<i32>` to `Model` (`mujoco_pipeline.rs`). Initialize
in `Model::empty()` as an empty `Vec`. Populate in `model_builder.rs:process_geom()`
from `MjcfGeom.condim`. Validate at parse time: clamp to `{1, 3, 4, 6}` — if a
geom specifies an invalid condim (2, 5, or > 6), round up to the next valid
value (2→3, 5→6, >6→6) and emit `tracing::warn!`.

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

MuJoCo uses element-wise max for friction combination (when geom priorities are
equal). We use geometric mean for all three components, matching our existing
sliding friction convention (`make_contact_from_geoms` line 3591). Switching to
element-wise max would be a one-line change per component but would alter every
existing simulation's behavior — this is better done as a deliberate conformance
task with before/after validation, not buried in a condim refactor.

**A.4.** Expand `Contact.mu` from `[f64; 2]` to `[f64; 5]`:

```rust
pub mu: [f64; 5],  // [sliding1, sliding2, torsional, rolling1, rolling2]
```

Update `Contact::with_solver_params()` signature to accept a `condim: usize`
parameter and a `mu: [f64; 5]` array. Set `self.dim = condim`. Remove the
`dim: if friction > 0.0 { 3 } else { 1 }` heuristic — condim is now an
explicit input from model data.

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

---

**Sub-task B: Variable-dimension Jacobian — `compute_contact_jacobian()`**

The contact Jacobian must produce `dim` rows instead of 3. The first 3 rows
are unchanged (normal, tangent1, tangent2). Rows 4–6 are angular Jacobian rows
in the contact frame.

**B.1.** Change `DMatrix::zeros(3, nv)` to `DMatrix::zeros(dim, nv)` where
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

---

**Sub-task C: Variable-dimension system assembly — `assemble_contact_system()`**

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
let (efc_offsets, _nefc) = compute_efc_offsets(&data.contacts);
// Pass &efc_offsets to pgs_solve_contacts / cg_solve_contacts
```

All three solver entry points (`pgs_solve_contacts`, `pgs_solve_with_system`,
`cg_solve_contacts`) gain an `efc_offsets: &[usize]` parameter.
`assemble_contact_system` also gains this parameter (it currently recomputes
`ncon * 3` locally).

**C.2.** `M⁻¹ * Jᵀ` computation (`:7823–7834`). The inner loop currently
iterates `for col in 0..3`. Change to `for col in 0..contact_dim` where
`contact_dim = contacts[i].dim`. The `minv_jt_contact` matrix becomes
`nv × dim` instead of `nv × 3`.

**C.3.** Diagonal block assembly (`:7856–7863`). Currently `for ri in 0..3 { for
ci in 0..3 }`. Change to `for ri in 0..dim_i { for ci in 0..dim_i }` where
`dim_i = contacts[i].dim`. Indexing changes from `i * 3 + ri` to
`efc_offsets[i] + ri`.

**C.4.** Off-diagonal block assembly (`:7936–7942`). Same change: `for ri in
0..dim_i { for ci in 0..dim_j }` with offset-based indexing.

**C.5.** RHS construction (`:7946–7985`). The current code computes `vn, vt1,
vt2` from linear relative velocity. For condim ≥ 4, also compute angular
relative velocity at the contact point and project onto the torsional/rolling
directions:

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
executed but harmless — the values are unused. Skipping the tangent basis for
condim 1 would be a micro-optimization, not required.

Note: the normal row includes Baumgarte position correction
(`depth_correction * depth`). The friction/torsional/rolling rows do not — they
are velocity-only constraints, matching MuJoCo's formulation.

---

**Sub-task D: Friction cone projection — `project_friction_cone()`**

This is the core mathematical change. The projection depends on both `condim`
and `Model.cone`.

**D.1. Elliptic cone projection.**

For condim `d`, the feasible set is a second-order cone (SOC):

```
K = { λ ∈ ℝ^d : λ₀ ≥ 0,  λ₀² ≥ Σᵢ₌₁^{d-1} (λᵢ / μᵢ)² }
```

where `μᵢ = contact.mu[i-1]` (the `i`-th friction coefficient).

**MuJoCo's approach (for reference).** MuJoCo's PGS solver does a coupled
QCQP projection per contact block: it solves
`min 0.5 x'Ax + x'b  s.t. Σ(xᵢ/μᵢ)² ≤ r²` using Newton iteration on the
Lagrange multiplier (functions `mju_QCQP`, `mju_QCQP2`, `mju_QCQP3` in
`engine_util_solve.c`). This produces the exact minimum of the local QP
subject to the cone constraint — the resulting friction direction can differ
from the unconstrained update direction because the off-diagonal Delassus
coupling is accounted for in the projection.

**Our approach: sequential SOC projection.** We use the simpler projection
that our solver already employs for condim 3: clamp the normal force
non-negative, then scale the friction vector to the cone boundary if needed.
This is the standard SOC projection `Π_K(λ)`:

```rust
fn project_elliptic_cone(lambda: &mut [f64], mu: &[f64; 5], dim: usize) {
    // Step 1: clamp normal force non-negative
    lambda[0] = lambda[0].max(0.0);

    // Step 2: compute weighted friction magnitude in the scaled cone
    // s = sqrt( Σ (λ_i / μ_i)² ) for i = 1..dim-1
    let mut s_sq = 0.0;
    for i in 1..dim {
        if mu[i - 1] > 1e-12 {
            s_sq += (lambda[i] / mu[i - 1]).powi(2);
        }
    }
    let s = s_sq.sqrt();

    // Step 3: if outside cone, scale friction to boundary
    if s > lambda[0] && s > 1e-10 {
        let scale = lambda[0] / s;
        for i in 1..dim {
            lambda[i] *= scale;
        }
    }
}
```

When `μ₁ = μ₂` (isotropic sliding), this reduces exactly to the current
circular cone projection (`‖λ_t‖ ≤ μ * λ_n`).

**Divergence analysis.** The sequential projection differs from MuJoCo's QCQP
in two ways: (1) it clamps normal before projecting friction, rather than
jointly optimizing both; (2) it preserves the friction direction from the GS
update, rather than rotating toward the QP optimum. For diagonal-dominant
Delassus matrices (typical in practice — contacts on independent or weakly
coupled bodies), the off-diagonal terms are small and the sequential
projection is an excellent approximation. For strongly coupled contacts
(multiple contacts on a single small body), the QCQP would converge in fewer
iterations. This is a pre-existing property of our condim-3 solver, not
introduced by this task. Upgrading to QCQP projection is a separate
optimization task.

**Note: when mu[i] ≈ 0.** If a friction coefficient is near zero (< 1e-12),
that direction's contribution to `s` is skipped and `lambda[i]` is clamped
to zero after projection. This handles degenerate cases like condim 4 with
zero torsional friction — the torsional row exists in the Jacobian/Delassus
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

**Sub-task E: Solver updates — PGS and CG**

**E.1. `pgs_solve_with_system()` (`:8049`).**

Compute `efc_offsets` and `nefc` from contacts (same as sub-task C). Replace
all `i * 3` indexing with `efc_offsets[i]`. The per-contact GS sweep changes:

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
    project_elliptic_cone(
        &mut lambda.as_mut_slice()[base..base + dim],
        &contacts[i].mu,
        dim,
    );

    // Track convergence
    for r in 0..dim {
        max_delta = max_delta.max((lambda[base + r] - old[r]).abs());
    }
}
```

**E.2. Warmstart format.** Change `efc_lambda: HashMap<WarmstartKey, [f64; 3]>`
(`Data` struct, `:1573`) to `HashMap<WarmstartKey, Vec<f64>>`. The `Vec` length
matches the contact's dim. On warmstart load, if the stored dim differs from
the current contact's dim (e.g., condim changed between frames), discard the
warmstart for that contact (use zero initialization). On store, save
`lambda[base..base+dim].to_vec()`.

**Callers that must update:**

| Location | Current usage | Change |
|----------|--------------|--------|
| `pgs_solve_with_system()` `:8068–8072` | Load `[f64; 3]` into `lambda[base..base+3]` | Load `Vec<f64>`, check len == dim |
| `pgs_solve_with_system()` `:8144–8148` | Store `[lambda[base], ...]` | Store `lambda[base..base+dim].to_vec()` |
| `cg_solve_contacts()` `:8266–8272` | Load `[f64; 3]` | Same as PGS |
| `cg_solve_contacts()` `:8302–8304` | Store `[lam[0], lam[1], lam[2]]` | Store `lam.as_slice().to_vec()` |
| `cg_solve_contacts()` `:8378–8381` | Store `[lambda[i*3], ...]` | Store `lambda[base..base+dim].to_vec()` |
| Touch sensor `:5805` | `data.efc_lambda.get(&key)` → `lambda[0]` | `lambda[0]` still correct (normal force) |
| RK4 integrator `:10566` | `data.efc_lambda.clone()` | Works unchanged (`HashMap<K, Vec>` is `Clone`) |
| Test code `:13596`, `:13657` | `HashMap<WarmstartKey, [f64; 3]>` | Update to `Vec<f64>` |

The touch sensor (`:5805`) reads `lambda[0]` which is the normal force
regardless of condim — no logic change needed, only the type annotation.

**E.3. `compute_block_jacobi_preconditioner()` (`:8158`).** Replace `Matrix3`
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

**E.4. `apply_preconditioner()` (`:8200`).** Change from `Vector3` blocks to
`DVector` blocks, indexing via `efc_offsets`.

**E.5. `cg_solve_contacts()` (`:8244`).**

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

**E.6. `extract_forces()` (`:8010`).** Change return type from
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

---

**Sub-task F: Force application — `mj_fwd_constraint()`**

The force application loop (`:9731–9746`) currently converts lambda
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
    let (tangent1, tangent2) = build_tangent_basis(&normal);
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

**Sub-task G: MuJoCo conformance — cone type validation**

**G.1.** In `Model` construction (or `model_from_mjcf()`), if `cone == 0`
(pyramidal), emit `tracing::warn!("pyramidal friction cones not yet
supported — using elliptic")` and set `cone = 1`. This prevents silent
incorrect behavior.

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
13. CG → PGS fallback works with variable block sizes.

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
    discarded when a contact's condim changes between frames.
18. Zero contacts still produces an empty solution (no crash from
    `efc_offsets` being empty).

#### Files

**`sim/L0/core/src/mujoco_pipeline.rs`** — modify (15 functions + 2 structs):

| Item | Sub-task | Change |
|------|----------|--------|
| `Model` struct | A.1 | Add `geom_condim: Vec<i32>` field |
| `Model::empty()` | A.1 | Init `geom_condim` as empty Vec |
| `Contact` struct | A.4 | `mu: [f64; 2]` → `[f64; 5]` |
| `Contact::with_solver_params()` | A.4 | Add `condim`, `mu` params |
| `Contact::new()` | A.6 | Infer condim/mu from friction (backward compat) |
| `make_contact_from_geoms()` | A.5 | Resolve condim, combine all 3 friction components |
| `Data` struct | E.2 | `efc_lambda: HashMap<K, [f64;3]>` → `HashMap<K, Vec<f64>>` |
| `compute_efc_offsets()` | C.1 | New helper function |
| `compute_contact_jacobian()` | B | `dim×nv` matrix, angular rows |
| `assemble_contact_system()` | C | Variable-dim blocks, `efc_offsets` |
| `project_elliptic_cone()` | D.1 | New function |
| `project_friction_cone()` | D.4 | Condim dispatch, `efc_offsets` |
| `pgs_solve_contacts()` | E.1 | Pass `efc_offsets` |
| `pgs_solve_with_system()` | E.1 | Variable-dim GS sweep |
| `compute_block_jacobi_preconditioner()` | E.3 | `DMatrix` blocks |
| `apply_preconditioner()` | E.4 | `DVector` blocks |
| `cg_solve_contacts()` | E.5 | Variable-dim, `DMatrix` direct solve |
| `extract_forces()` | E.6 | `Vec<DVector<f64>>` return |
| `apply_contact_torque()` | F.1 | New function (angular J^T) |
| `mj_fwd_constraint()` | F.2 | Torque application, `efc_offsets` |
| Touch sensor (`:5805`) | E.2 | Type annotation only (`Vec` access) |

**`sim/L0/mjcf/src/model_builder.rs`** — modify:
- `process_geom()` — push `geom.condim` to `Model.geom_condim`
- `model_from_mjcf()` — condim validation (clamp to {1,3,4,6}), pyramidal
  cone warning (sub-task G.1)

**`sim/L0/tests/integration/`** — new file `condim_friction.rs`:
- 18 acceptance criteria tests
- Register in `mod.rs`
- Tests use `sim_mjcf::load_model(xml)` with MJCF snippets specifying
  different condim values per geom
- Jacobian tests (criteria 14–15) use finite-difference verification against
  analytical Jacobian rows

---

### 3. `<contact>` Pair/Exclude
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State
Contact filtering uses `contype`/`conaffinity` bitmasks only
(`mujoco_pipeline.rs:3278–3328`). The `<contact>` MJCF element with `<pair>` and
`<exclude>` sub-elements is not parsed. Users cannot:

- Force contacts between specific geom pairs regardless of bitmasks
- Exclude contacts between specific geom pairs (e.g., self-collision filtering)
- Override solref/solimp per pair

#### Objective
Parse `<contact><pair>` and `<contact><exclude>` from MJCF and apply them in
collision filtering.

#### Specification

**Data structures in Model:**

```rust
pub contact_pairs: Vec<ContactPair>,   // force contact between specific geoms
pub contact_excludes: Vec<(usize, usize)>,  // exclude contact between body pairs
```

**`ContactPair`:** Stores `(geom1, geom2)` indices plus per-pair overrides for
`condim`, `friction`, `solref`, `solimp`, `margin`, `gap`.

**Collision filtering update in `can_collide()`:**

1. Check exclude list first — if body pair is excluded, skip
2. Check pair list — if geom pair has explicit pair, force include (ignore bitmasks)
3. Fall through to existing contype/conaffinity bitmask check

**MJCF parsing:**

```xml
<contact>
  <pair geom1="left_hand" geom2="right_hand" condim="1"/>
  <exclude body1="upper_arm" body2="forearm"/>
</contact>
```

#### Acceptance Criteria
1. `<exclude>` prevents contacts between specified body pairs.
2. `<pair>` forces contacts between specified geom pairs, overriding bitmasks.
3. Per-pair condim/friction/solref/solimp overrides apply to paired contacts.
4. Existing contype/conaffinity filtering is unchanged for non-paired/excluded geoms.

#### Files
- `sim/L0/mjcf/src/parser.rs` — modify (parse `<contact>` block)
- `sim/L0/mjcf/src/types.rs` — modify (add `MjcfContactPair`, `MjcfContactExclude`)
- `sim/L0/mjcf/src/model_builder.rs` — modify (build pair/exclude lists)
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (`can_collide()`, Model fields)

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
