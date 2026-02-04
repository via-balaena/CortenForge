# Future Work 2 — Correctness: Model Fidelity (Items #1–5)

Part of [Simulation Phase 2 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

---

### 1. `<default>` Class Resolution
**Status:** Not started | **Effort:** S | **Prerequisites:** None

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
| `MjcfJoint` | `apply_to_joint()` | Before `process_joint()` call in `process_body_with_world_frame()` (:685) | Yes |
| `MjcfGeom` | `apply_to_geom()` | Before `compute_inertia_from_geoms()` (:733) and `process_geom()` (:782) in `process_body_with_world_frame()`, and in `process_worldbody_geoms_and_sites()` (:620). See ordering constraint below. | Yes |
| `MjcfActuator` | `apply_to_actuator()` | Before `process_actuator()` call in `model_from_mjcf()` (:130) | Yes |
| `MjcfTendon` | `apply_to_tendon()` | Inside `process_tendons()` loop (:1108), Pattern B | Yes |
| `MjcfSensor` | `apply_to_sensor()` | Inside `process_sensors()` loop (:1468), Pattern B | Yes |
| `MjcfSite` | `apply_to_site()` | Before `process_site()` calls | No — applies root defaults only |

**Resolution patterns.** Three distinct patterns arise depending on how the
caller iterates elements:

*Pattern A — caller loop (joints, geoms, sites, actuators).* The caller
iterates elements and calls a `process_*` method for each one. Resolution
happens in the loop body, before the call:

```rust
// Before (process_body_with_world_frame, line 765):
for joint in &body.joints {
    self.process_joint(joint, body_id, current_last_dof, world_pos, world_quat)?;
}

// After:
for joint in &body.joints {
    let joint = self.resolver.apply_to_joint(joint);
    self.process_joint(&joint, body_id, current_last_dof, world_pos, world_quat)?;
}
```

Same pattern applies to the actuator loop in `model_from_mjcf()` (line 130),
except that `model_from_mjcf` is a free function — use `builder.resolver`
instead of `self.resolver`:

```rust
for actuator in &mjcf.actuators {
    let actuator = builder.resolver.apply_to_actuator(actuator);
    builder.process_actuator(&actuator)?;
}
```

*Pattern B — internal loop (tendons, sensors).* `process_tendons(&[MjcfTendon])`
and `process_sensors(&[MjcfSensor])` receive a slice and iterate internally.
Resolution happens inside these methods at the top of their `for` loop:

```rust
// Inside process_tendons (line 1108):
for (t_idx, tendon) in tendons.iter().enumerate() {
    let tendon = self.resolver.apply_to_tendon(tendon);  // ← add
    // ... existing code uses `tendon.stiffness`, `tendon.damping`, etc.
}
```

Same pattern for `process_sensors` (line 1468). The shadowed `tendon`/`sensor`
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
at line 733, *before* the geom processing loop at line 782. This function reads
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
`body_geom_num.push(body.geoms.len())` at line 785 still uses the original
slice length, which is correct since `resolved_geoms.len() == body.geoms.len()`.

**How "explicit vs default" detection works.** The `apply_to_*` methods use
sentinel values to detect whether a field was explicitly set or is still at its
parser-initialized default. For example, `apply_to_joint()` treats
`damping == 0.0` as "not explicitly set" and fills it from the class default.
The parser initializes `MjcfJoint` fields to MuJoCo's documented default values
(e.g., `damping: 0.0`, `armature: 0.0`, `stiffness: 0.0`), so zero-default
fields work correctly — a user who never specifies damping gets class defaults
applied.

**Note:** `get_defaults()` (`defaults.rs:72`) has a misleading doc comment
claiming it "returns the root defaults if the class doesn't exist." The
implementation actually returns `None` for unknown class names (line 74:
`self.resolved_defaults.get(class_name)`). This is safe — `apply_to_*` methods
handle `None` by returning the element unchanged — but an implementer reading
the doc comment might expect different behavior. Consider fixing the doc comment
as part of this task.

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
| `model_from_mjcf()` (:130) | actuators |
| `process_body_with_world_frame()` (:765, :782, :788) | joints, geoms, sites |
| `process_worldbody_geoms_and_sites()` (:628, :633) | worldbody geoms, sites |
| `process_tendons()` (:1108) | tendons |
| `process_sensors()` (:1468) | sensors |

**Approach: store on `ModelBuilder`.** Add a `resolver: DefaultResolver` field,
initialized to `DefaultResolver::default()` in `ModelBuilder::new()` and set to
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
The contact solver is hardcoded to condim 3 (normal + 2D tangential friction):

- `nefc = ncon * 3` at three locations (`mujoco_pipeline.rs:7806`, `8062`, `8257`)
- `project_friction_cone()` implements circular cone only (`|λ_t| ≤ μ * λ_n`)
- `Model.cone` field (0=pyramidal, 1=elliptic) is parsed and stored but **ignored**
  by the solver
- `geom_friction[i].y` (torsional) and `.z` (rolling) are stored but **never read**
  — only `.x` (sliding) is extracted for contacts
- `Contact.dim` stores the condim value but the solver never checks it
- `Contact::new()` already computes a meaningful `dim` value
  (`dim: if friction > 0.0 { 3 } else { 1 }` at `mujoco_pipeline.rs:1307`), so
  part of the plumbing exists — the solver simply ignores it

This means:
- **condim 1** (frictionless): Not supported — all contacts get friction
- **condim 3** (sliding friction): The only mode that works
- **condim 4** (+ torsional): Torsional friction silently dropped
- **condim 6** (+ rolling): Rolling friction silently dropped
- **Elliptic cones**: Parsed, ignored — circular cone used regardless
- **Pyramidal cones**: Parsed, ignored — circular cone used regardless

#### Objective
Support the full range of MuJoCo contact dimensionalities (1, 3, 4, 6) and
friction cone types (elliptic, pyramidal, circular).

#### Specification

**Variable-size constraint blocks:**

Replace `nefc = ncon * 3` with `nefc = sum(contact[i].dim)`. The Delassus matrix
becomes block-structured with variable block sizes. `assemble_contact_system()`
must build blocks of size `dim × dim` per contact instead of fixed 3×3.

**Condim dispatch:**

| condim | DOFs per contact | Description |
|--------|-----------------|-------------|
| 1 | 1 | Normal only (frictionless) |
| 3 | 3 | Normal + 2D tangential (current) |
| 4 | 4 | Normal + 2D tangential + torsional |
| 6 | 6 | Normal + 2D tangential + torsional + 2D rolling |

For condim 1: Skip friction rows entirely. Constraint is scalar `λ_n ≥ 0`.

For condim 4: Add torsional friction row. Torsional moment `τ_t ≤ μ_t * λ_n`
where `μ_t = geom_friction[i].y`. Requires spin velocity computation at contact.

For condim 6: Add 2 rolling friction rows. Rolling moment
`|τ_r| ≤ μ_r * λ_n` where `μ_r = geom_friction[i].z`.

**Friction cone types:**

| Type | Projection |
|------|-----------|
| Circular (current) | `‖λ_t‖ ≤ μ * λ_n` |
| Elliptic (MuJoCo default) | `(λ_t1/μ₁)² + (λ_t2/μ₂)² ≤ λ_n²` |
| Pyramidal | Linearized: `|λ_t1| + |λ_t2| ≤ μ * λ_n` (or N-face approximation) |

Dispatch on `Model.cone` in `project_friction_cone()`.

**Impact on CG solver:**

Both PGS and CG (`cg_solve_contacts()`, `pgs_solve_with_system()`) must handle
variable block sizes. The preconditioner (`compute_block_jacobi_preconditioner()`)
changes from 3×3 diagonal blocks to `dim × dim` blocks.

#### Acceptance Criteria
1. condim 1 contact produces zero tangential force.
2. condim 3 contact matches current behavior (regression).
3. condim 4 contact limits torsional moment by `μ_t * λ_n`.
4. condim 6 contact limits rolling moment by `μ_r * λ_n`.
5. Elliptic cone projection matches MuJoCo for anisotropic friction.
6. Mixed condim contacts in the same scene work (different contacts have
   different dim values).
7. `Model.cone` selects the friction cone type at model load time.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (`assemble_contact_system()`,
  `project_friction_cone()`, `pgs_solve_with_system()`, `cg_solve_contacts()`,
  `compute_block_jacobi_preconditioner()`)

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
