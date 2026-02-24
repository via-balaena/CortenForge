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

**The resolver is never called.** `builder/` has zero references to
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

**Import.** `builder/` does not reference `DefaultResolver` today. Add:

```rust
use crate::defaults::DefaultResolver;
```

**Integration point:** `model_from_mjcf()` (`builder/`). This is a
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

**Not in scope (now implemented): `childclass` attribute.** MuJoCo supports
`<body childclass="X">` which sets the default class for all child elements
that don't specify their own `class`. This was implemented as part of item #19
(`<frame>` element parsing + body/frame `childclass`) in `future_work_6.md`.
`MjcfBody` now has a `childclass` field, and `process_body_with_world_frame()`
threads `inherited_childclass` through the body hierarchy.

**Not in scope: mesh defaults.** `DefaultResolver` has `mesh_defaults()` and
merges `MjcfMeshDefaults` (containing `scale: Option<Vector3<f64>>`) through
the inheritance chain. However, `MjcfMesh` has no `class` field — mesh defaults
can only come from the root default class. `convert_mjcf_mesh()` reads
`mjcf_mesh.scale` directly (lines 2295, 2331). Applying root-only mesh scale defaults
is low-value (few real models rely on it) and would require special-casing since
there's no `apply_to_mesh()` method. Deferred.
Tracked in [future_work_10b.md](./future_work_10b.md) §DT-1.

**Not in scope: equality constraint defaults.** The four equality constraint
types (Connect, Weld, Joint, Distance) all have `class` fields in their MJCF
structs, but `DefaultResolver` has no `apply_to_equality()` method and
`MjcfDefault` has no equality defaults. Similarly, `solref`/`solimp` are not
included in any of the defaults structs (`MjcfGeomDefaults`,
`MjcfJointDefaults`, etc.). These are resolver gaps, not wiring gaps — they
require extending `DefaultResolver` and the defaults types, which is a separate
task.
Tracked in [future_work_10b.md](./future_work_10b.md) §DT-2.

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
to the `ModelBuilder` struct (defined at `builder/`), initialized to
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
- `sim/L0/mjcf/src/builder/` — modify: add `DefaultResolver` field to
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

**A.1.** Add `geom_condim: Vec<i32>` to `Model` (`types/model.rs`). Initialize
in `Model::empty()` as an empty `Vec` (the struct derives `Clone` and `Debug`, so
adding a `Vec<i32>` field requires no trait changes). Populate in
`builder/:process_geom()` from `MjcfGeom.condim` — push immediately after
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

**Known non-conformance:** ⚠️ **Tracked as [#24](./future_work_7.md)
(Friction Combination Rule).** MuJoCo uses element-wise max for friction
combination (when geom priorities are equal). We use geometric mean. The fix
is tracked as #24, a prerequisite to the conformance test suite (#45).

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
Tracked in [future_work_10c.md](./future_work_10c.md) §DT-18.

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
literal syntax (bypassing constructors), all in sim-core test modules:
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
implementation across sim-core modules:

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
Tracked in [future_work_10c.md](./future_work_10c.md) §DT-20.

**Not in scope: pyramidal friction cones.** See sub-task D.2 rationale.
Pyramidal cones require a fundamentally different variable count per contact
(2(d-1) vs d), different Jacobian basis construction, and changes to every
solver loop. This is better implemented as a separate task once elliptic cones
are working.

**Not in scope: per-pair condim override.** `<contact><pair condim="...">` is
task #3 (contact pair/exclude, now implemented — see §3). This task reads
condim from `geom_condim` only.

**Not in scope: anisotropic sliding from MJCF.** MuJoCo supports different
`mu[0]` and `mu[1]` (sliding1 ≠ sliding2) per contact, but only via
`<contact><pair>` overrides (now implemented in task #3 — see §3). Per-geom
friction has a single sliding value that fills both slots. The elliptic
projection handles anisotropic `mu` correctly when provided by a pair override.

**Not in scope: geom priority.** MuJoCo uses `geom/@priority` to break ties
in condim/friction combination. Our parser does not read `priority`. We use
`max(condim1, condim2)` unconditionally.

**Not in scope: QCQP-based cone projection.** MuJoCo's PGS solver uses a
per-contact QCQP (quadratically constrained QP) to jointly project normal and
friction forces onto the cone. Our solver uses sequential SOC projection
(clamp normal, then scale friction). See D.1 divergence analysis. Upgrading
to QCQP projection would improve convergence for strongly coupled contacts
but is a separate optimization task.
Tracked in [future_work_10c.md](./future_work_10c.md) §DT-19.

**Not in scope: friction combination method.** ⚠️ Now tracked as
[#24](./future_work_7.md) (Friction Combination Rule).

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

**Phase 2 changes — `sim/L0/core/src/` (multiple modules):**

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

**Phase 2 changes — `sim/L0/mjcf/src/builder/`:**
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
| D3 | Friction combination: keep geometric mean? | **Deferred → #24** | Now tracked in [future_work_7.md](./future_work_7.md) #24 |
| D4 | `Contact.friction` field: keep or remove? | **Keep** | Retained for backward compat; documented as `== mu[0]`; solver reads `mu` |
| D5 | Warmstart: `Vec<f64>` or `SmallVec<[f64; 6]>`? | **`Vec<f64>`** | Optimize later if profiling shows heap allocation pressure → [future_work_10j.md](./future_work_10j.md) §DT-91 |
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

##### D. Model Data Structures (`types/data.rs` → `Model`)

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

##### E. Model Builder (`builder/`)

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
| `sim/L0/mjcf/src/builder/` | modify | Add `HashSet` import (line ~21); add `ContactPair` import from sim_core; add `MjcfContact` to types import; add `contact_pairs`, `contact_pair_set`, `contact_excludes` fields to `ModelBuilder`; new `process_contact()` method (resolve names, apply defaults, compute geom fallbacks incl. `solreffriction` → `solref`); call `process_contact()` in `model_from_mjcf()` after body tree; move fields into `Model` in `build()` struct literal |
| `sim/L0/core/src/types/model_init.rs` | modify | Add `ContactPair` struct and `contact_pairs`/`contact_pair_set`/`contact_excludes` to `Model`; add `HashSet` import; initialize new fields in `Model::empty()`; add exclude + pair-set checks in `check_collision_affinity`; add mechanism-2 loop in `mj_collision` (rbound cull + `collide_geoms` + `apply_pair_overrides`) |
| `sim/L0/core/src/lib.rs` | modify | Add `ContactPair` to re-exports from `types` |

---

### 4. Spatial Tendons + Wrapping
**Status:** Complete (steps 1-9 done) | **Effort:** L | **Prerequisites:** None

#### Current State

**Fixed tendons** are fully integrated: `mj_fwd_tendon_fixed()` computes length as
`L = Σ cᵢ qᵢ`, populates `ten_J[t]` with coefficients, and force transmission
uses the simple `qfrc += coef * force` pattern through `wrap_objid` (DOF address)
and `wrap_prm` (coefficient).

**Spatial tendons** are parsed and stored but produce zeros at runtime:
- `mj_fwd_tendon()` dispatches on `TendonType::Spatial` → sets `ten_length=0`,
  `ten_J=zeros`.
- Three force-mapping sites use the fixed-tendon wrap-array pattern (`wrap_objid`
  as DOF address), which is semantically wrong for spatial tendons where
  `wrap_objid` holds site IDs:
  - `mj_fwd_passive()` — tendon spring/damper/friction → `qfrc_passive`
  - `mj_fwd_constraint()` — tendon limit penalty → `qfrc_constraint`
  - `mj_fwd_actuation()` — `ActuatorTransmission::Tendon` → `qfrc_actuator`
- Two model-build-time computations also use the wrap-array pattern and will
  produce wrong results for spatial tendons:
  - `compute_muscle_params()` — builds J vector from `wrap_objid`/`wrap_prm`
    for `acc0` computation. (Comment: "Tendon evaluation is not needed because we
    construct the moment vector directly from wrap_objid/wrap_prm (constant for
    fixed tendons)." This is wrong for spatial tendons where J is
    configuration-dependent.)
  - `actuator_lengthrange` estimation for unlimited tendons — iterates wrap
    objects and looks up joint ranges by DOF address.

**sim-tendon crate** (`sim/L0/tendon/src/`) has standalone implementations of the
geometric primitives:

| Component | File | Status |
|-----------|------|--------|
| `SphereWrap` | `wrapping.rs` | Standalone — geodesic arc wrapping |
| `CylinderWrap` | `wrapping.rs` | Standalone — 2D radial + axial wrapping |
| `TendonPath` | `path.rs` | Standalone — multi-segment path with via points |
| `SpatialTendon` | `spatial.rs` | Standalone — wraps TendonPath + force model |
| `PulleySystem` | `pulley.rs` | Standalone — compound pulleys, capstan equation |

These are **reference implementations** — the pipeline will reimplement the core
algorithms inline (see Design Decision below), not call into sim-tendon directly.

**MJCF parser bug:** `parse_tendon()` stores sites and wrapping geoms in separate
`Vec<String>` fields (`tendon.sites`, `tendon.wrapping_geoms`), **losing their
interleaved ordering** from the XML. MuJoCo spatial tendon semantics require
ordered path elements: `<site> <geom> <site> <geom> <site>`. The parser must
preserve this ordering.

**Missing Model fields:**
- No `wrap_sidesite` array — the sidesite attribute from `<geom>` wrapping
  elements is not stored anywhere after parsing. Needed for wrapping side
  disambiguation.
- `tendon_length0` is declared on `Model` but never computed — MuJoCo computes
  tendon length at `qpos0` during compilation and uses it as the default
  `lengthspring` when none is specified.

#### Objective

Wire spatial tendon length, Jacobian, and force transmission into the pipeline so
that spatial tendons produce correct kinematics (length, velocity) and dynamics
(passive forces, limit forces, actuator forces) — verified against MuJoCo.

#### Design Decision: Inline vs. sim-tendon Dependency

**Decision: Inline reimplementation in the pipeline.**

Rationale:
- The pipeline uses flat array data structures (`wrap_type[]`, `wrap_objid[]`,
  `wrap_prm[]`, `ten_J[]`, `site_xpos[]`, `geom_xpos[]`, `geom_xmat[]`). The
  sim-tendon crate uses rich struct hierarchies (`TendonPath`, `AttachmentPoint`,
  `WrappingGeometry`). Bridging these would require allocation and copying each
  timestep.
- MuJoCo computes spatial tendon Jacobians via body Jacobian projection, which
  requires access to the full kinematic tree (`jnt_type`, `jnt_axis`, `body_parent`,
  etc.). The sim-tendon crate does not have this interface.
- The sim-tendon crate's wrapping geometry (`SphereWrap::compute_wrap`,
  `CylinderWrap::compute_wrap`) can be used as **reference implementations** for
  verification, but the pipeline should implement wrapping directly using the
  `geom_xpos`/`geom_xmat`/`geom_size` arrays that are already available.
- This matches how fixed tendons work: `mj_fwd_tendon_fixed()` is fully inline
  in the pipeline, not a wrapper around sim-tendon's `FixedTendon`.

#### Specification

##### 4.1 MJCF Parser: Preserve Path Element Ordering

The parser must store spatial tendon path elements in their original XML order.
Replace the separate `sites: Vec<String>` and `wrapping_geoms: Vec<String>` fields
with an ordered path representation.

**New type in `types.rs`:**
```rust
/// A single element in a spatial tendon path, preserving MJCF ordering.
#[derive(Debug, Clone, PartialEq)]
pub enum SpatialPathElement {
    /// Via-point: tendon passes through this site.
    Site { site: String },
    /// Wrapping surface: tendon wraps around this geom (sphere or cylinder only).
    /// `sidesite` determines which side of the geometry the tendon wraps around,
    /// preventing discontinuous jumps when multiple wrapping solutions exist.
    Geom { geom: String, sidesite: Option<String> },
    /// Pulley: scales subsequent path length and Jacobian contributions by
    /// 1/divisor until the next pulley element or end of tendon.
    Pulley { divisor: f64 },
}
```

**Modified `MjcfTendon`:**
```rust
pub struct MjcfTendon {
    // ... existing fields (name, class, tendon_type, range, limited,
    //     stiffness, damping, frictionloss, width, rgba) ...
    /// Ordered path elements for spatial tendons (replaces `sites` + `wrapping_geoms`).
    pub path_elements: Vec<SpatialPathElement>,
    /// Joint coefficients for fixed tendons (unchanged).
    pub joints: Vec<(String, f64)>,
}
```

Remove the `sites: Vec<String>` and `wrapping_geoms: Vec<String>` fields.

**Parser changes in `parser.rs`:** `parse_tendon()` pushes `SpatialPathElement`
variants for each child element in document order:
- `<site site="...">` → `SpatialPathElement::Site { site }`
- `<geom geom="..." sidesite="...">` → `SpatialPathElement::Geom { geom, sidesite }`
- `<pulley divisor="...">` → `SpatialPathElement::Pulley { divisor }` (default 1.0)
- `<joint ...>` → emit error: "Joint elements inside spatial tendons are not
  supported (tendon '{name}'). Use a fixed tendon for joint coupling."

##### 4.2 Model Builder: Spatial Tendon Path → Wrap Arrays

`process_tendons()` in `builder/` translates `path_elements` into the
flat wrap arrays, **preserving element order**.

**Existing types:** `WrapType::Geom` and `WrapType::Pulley` already exist in
the `WrapType` enum (line 471) but are currently unused by the model builder —
only `WrapType::Joint` is emitted for fixed tendons and `WrapType::Site` for
spatial tendon sites. This spec activates `WrapType::Geom` and
`WrapType::Pulley` for spatial tendon wrap arrays.

**New Model field:**
```rust
/// Sidesite ID for wrapping geom wrap objects. `usize::MAX` if no sidesite.
/// Indexed in parallel with `wrap_type`/`wrap_objid`/`wrap_prm`.
pub wrap_sidesite: Vec<usize>,
```

**Parallel array invariant:** `wrap_sidesite` must be pushed for **every** wrap
entry — both spatial and fixed tendons — to maintain the same length as
`wrap_type`/`wrap_objid`/`wrap_prm`. Fixed tendon wrap entries always push
`wrap_sidesite.push(usize::MAX)` (they have no wrapping geometry). Failure to
maintain this invariant causes index-out-of-bounds panics when
`mj_fwd_tendon_spatial()` reads `wrap_sidesite[adr + j + 1]`.

For each `SpatialPathElement`:
- `Site { site }` → `wrap_type.push(WrapType::Site)`, `wrap_objid.push(site_id)`,
  `wrap_prm.push(0.0)`, `wrap_sidesite.push(usize::MAX)`.
- `Geom { geom, sidesite }` → `wrap_type.push(WrapType::Geom)`,
  `wrap_objid.push(geom_id)`, `wrap_prm.push(0.0)`,
  `wrap_sidesite.push(resolved_sidesite_id or usize::MAX)`. Validate that the
  referenced geom has type `GeomType::Sphere` or `GeomType::Cylinder`; emit
  error otherwise.
- `Pulley { divisor }` → `wrap_type.push(WrapType::Pulley)`,
  `wrap_objid.push(0)`, `wrap_prm.push(divisor)`, `wrap_sidesite.push(usize::MAX)`.

**Validation rules:**
1. Path must start and end with a `Site` element.
2. Every `Geom` element must be immediately followed by a `Site` element.
   This prevents three dangerous sequences: (a) consecutive `Geom` elements
   (MuJoCo requires wrapping geoms to be separated by sites), (b) `Geom`
   followed by `Pulley` (the Site-Geom-Site branch reads `wrap_objid[j+2]`
   as a site ID — if position `j+2` is a Pulley with `objid=0`, it silently
   reads `site_xpos[0]`, producing wrong position data), and (c) `Geom` as
   the last element (out-of-bounds read). Combined with rule 1
   (path ends with Site), this ensures the pairwise loop's Site-Geom-Site
   branch always has a valid Site at `j+2`.
3. A tendon with fewer than 2 `Site` elements is an error.
4. All site names, geom names, and sidesite names referenced in `path_elements`
   must resolve to existing model objects. Emit an error with the tendon name
   and unresolved name otherwise.
5. Wrapping geoms must have `geom_size[0] > 0` (MuJoCo convention for the
   primary radius). In the codebase, `geom_size` is `Vec<Vector3<f64>>`
   (declared at line 878), so `geom_size[0]` in MuJoCo terminology corresponds
   to `model.geom_size[geom_id].x` in Rust. A zero-radius wrapping geom
   causes division by zero in the arc angle computation.
6. A `Pulley` element must not be immediately followed by a `Geom` element.
   The pairwise loop assumes `type0` is always a `Site` after pulley handling.
   A `Pulley, Geom` sequence would cause `type0 = Geom`, which then indexes
   `site_xpos` with a geom ID — producing silent data corruption (wrong
   position data), not a panic. Add `debug_assert!(type0 == WrapType::Site)`
   after the pulley check as a defense-in-depth guard. MuJoCo requires pulleys
   to appear between site groups: `Site [Geom Site]* [Pulley Site [Geom Site]*]*`.
7. Pulley `divisor` must be positive (`divisor > 0.0`). A zero or negative
   divisor causes division by zero in length and Jacobian accumulation.
8. Each pulley-delimited branch must contain at least 2 `Site` elements.
   A branch is the sequence of elements between the start of the path and
   the first `Pulley`, between two consecutive `Pulley` elements, or between
   the last `Pulley` and the end of the path. A single-site branch produces
   no segment and is a degenerate configuration that wastes a pulley division.
   Emit a warning (not an error, for MuJoCo compatibility) with the tendon
   name and branch index.
9. ~~Wrapping geom sidesite must be **outside** the wrapping geometry surface.~~
   **Retired** — sidesites inside wrapping geometry are now handled by the
   `wrap_inside` algorithm (§39, `future_work_10.md`). The build-time
   validation panic has been removed.
   ~~**Validation timing:** This rule requires computing the sidesite position in
   the wrapping geom's local frame. When the sidesite and geom are on the
   **same body**, both positions are in the same body-local frame and the
   transform is static (no FK needed). When they are on **different bodies**,
   the relative transform requires FK. Therefore, this validation is performed
   inside `compute_spatial_tendon_length0()` (which runs `mj_fwd_position()`
   first), not in `process_tendons()`. MuJoCo similarly validates at compile
   time after computing initial body transforms from `qpos0`.~~
10. A `<joint>` element inside a `<spatial>` tendon is an error. MuJoCo
    supports `mjWRAP_JOINT` within spatial tendons, but this is uncommon and
    not implemented. Emit an error: "Joint elements inside spatial tendons are
    not supported (tendon '{name}'). Use a fixed tendon for joint coupling."
    This is enforced at parse time (§4.1) but also validated here as
    defense-in-depth.

Remove the spatial tendon warning log.

**Compute `tendon_length0` for spatial tendons:** The model builder already
computes `tendon_length0` and defaults `lengthspring` for fixed tendons
(`builder/`, in the `build()` method, loop at ~line 2319). This loop
only handles `TendonType::Fixed` via the wrap-array DOF pattern. For spatial
tendons, extend this by adding a new public `Model` method:

```rust
pub fn compute_spatial_tendon_length0(&mut self) {
    // Early return if no spatial tendons — avoid unnecessary FK + Data allocation.
    // Mirrors compute_muscle_params() which returns early when there are no muscles.
    let has_spatial = (0..self.ntendon).any(|t| self.tendon_type[t] == TendonType::Spatial);
    if !has_spatial {
        return;
    }

    let mut data = self.make_data();
    mj_fwd_position(self, &mut data);  // runs FK + mj_fwd_tendon

    // ~~Rule 9 validation (sidesite outside wrapping geometry) — RETIRED per §39.~~
    // The `wrap_inside` algorithm is now implemented. Sidesites inside wrapping
    // geometry are handled at runtime by the `wrap_inside_2d()` Newton solver
    // in `sphere_wrap()` and `cylinder_wrap()`. The build-time panic has been
    // removed from the actual implementation.

    // Compute tendon_length0 and default lengthspring for spatial tendons.
    for t in 0..self.ntendon {
        if self.tendon_type[t] == TendonType::Spatial {
            self.tendon_length0[t] = data.ten_length[t];
            if self.tendon_lengthspring[t] == 0.0 && self.tendon_stiffness[t] > 0.0 {
                self.tendon_lengthspring[t] = data.ten_length[t];
            }
        }
    }
}
```

**Build ordering:** In `builder/`'s `build()` method, the current
order is:

1. Assemble `Model` struct literal (line ~2105)
2. `compute_ancestors()` (line 2295)
3. `compute_implicit_params()` (line 2298)
4. `compute_muscle_params()` (line 2301)
5. Geom bounding radii loop (line 2304)
6. Fixed tendon `tendon_length0` loop (line 2319)

`compute_spatial_tendon_length0()` must run **before** `compute_muscle_params()`
because it populates `tendon_length0` for spatial tendons (needed for
`lengthspring` defaulting) and validates sidesite positions (which requires FK).
It does **not** depend on any step after (3), so the correct insertion point is
between (3) and (4). The fixed tendon loop (6) stays where it is — it handles
only `TendonType::Fixed` and uses simple linear algebra (no FK needed).

The new order becomes:

1. Assemble `Model` struct literal
2. `compute_ancestors()`
3. `compute_implicit_params()`
4. **`compute_spatial_tendon_length0()`** — new; requires FK via `mj_fwd_position`
5. `compute_muscle_params()` — now sees valid `tendon_length0` for all types
6. Geom bounding radii loop — unchanged
7. Fixed tendon `tendon_length0` loop — unchanged; overwrites spatial values
   for `TendonType::Fixed` tendons (which is correct: fixed tendon length0 is
   analytic, not FK-derived)

Note: the current `compute_muscle_params()` does not read spatial tendon FK
data directly (it uses the wrap-array shortcut for fixed tendons), but the
spatial tendon `tendon_length0` must be valid before any downstream code that
reads `tendon_lengthspring`.

No circular dependency: `wrap_sidesite` is static model data populated by
`process_tendons()` before `build()` returns. `mj_fwd_tendon_spatial()` reads
it at runtime. `tendon_length0` is derived data computed after the model
structure is complete.

**Borrow safety:** The `&mut self` method passes `self` to `mj_fwd_position(self,
&mut data)` — Rust auto-reborrows `self` as `&Model` for the call. After
`mj_fwd_position` returns, `self` is writeable again for `self.tendon_length0[t]
= data.ten_length[t]`. `data` is an owned `Data` value (from `make_data()`,
which clones `qpos0`), so it does not hold references into `self`. This is the
same borrow pattern used by `compute_muscle_params()` (line 2336-2337).

**Note on `springlength`:** MuJoCo's MJCF attribute `springlength` is actually
a `real(2)` pair `(low, high)` with default `(-1, -1)`. When both are -1,
MuJoCo uses `tendon_length0` as the rest length. When specified as a pair with
`low < high`, it creates a spring deadband (zero force when length is between
`low` and `high`). Our current model stores `tendon_lengthspring` as a single
`f64`. Upgrading to a pair for deadband semantics is out of scope for this spec
— the parser does not currently parse `springlength`, and the single-float
default-from-length0 behavior is sufficient for initial spatial tendon support.
Add `springlength` parsing + deadband semantics as a follow-up item.

##### 4.3 Spatial Tendon Length + Jacobian: `mj_fwd_tendon_spatial()`

New function called from `mj_fwd_tendon()` for `TendonType::Spatial`.

**Dispatch point:** In `mj_fwd_tendon()` (line 6599), replace the
`TendonType::Spatial` stub (lines 6611-6614, which sets `ten_length=0` and
`ten_J.fill(0.0)`) with a call to `mj_fwd_tendon_spatial(model, data, t)`.
This parallels the `TendonType::Fixed` arm which calls
`mj_fwd_tendon_fixed(model, data, t)`.

**`ten_J` storage:** `Data.ten_J` is a `Vec<DVector<f64>>` — one dense vector
of length `nv` per tendon (declared at line 1684, initialized at line 2103 as
`vec![DVector::zeros(self.nv); self.ntendon]`). `mj_fwd_tendon_spatial()` zeros
`data.ten_J[t]` with `.fill(0.0)` at entry and accumulates Jacobian
contributions via `data.ten_J[t][dof] += scale * direction.dot(&j_col)` through
`accumulate_point_jacobian()`. This matches the fixed-tendon pattern in
`mj_fwd_tendon_fixed()` (lines 6632/6645). The dense storage is acceptable for
correctness; a sparse representation is a follow-up optimization (see §4.5
performance note).

**Algorithm overview.** MuJoCo processes spatial tendons using a pairwise loop
over the wrap array (`engine_core_smooth.c`, function `mj_tendon`). At each
iteration, it reads `type0 = wrap_type[j]` and `type1 = wrap_type[j+1]`:

- **Pulley:** If either `type0` or `type1` is a pulley, handle it and advance
  `j += 1`. When `type0` is the pulley, update the divisor. When `type1` is the
  pulley (but `type0` is not), just skip — the divisor will be updated on the
  next iteration when the pulley becomes `type0`.
- **Site–Site:** `type0` and `type1` are both sites. Compute straight-line
  distance. Advance `j += 1`.
- **Site–Geom–Site:** `type0` is a site, `type1` is a geom (sphere/cylinder).
  Read the site at `j+2`. Call wrapping to get tangent points. Advance `j += 2`.

For each segment, the wrapping function (`mju_wrap`) produces a 4-point waypoint
array `[site0, t1, t2, site1]`. When wrapping occurs, 3 sub-segments are produced.
When the path is clear, only 1 segment `[site0, site1]` is produced. The Jacobian
loop iterates over `k in 0..(wrapped ? 3 : 1)`, computing the Jacobian for each
straight sub-segment by projecting the body Jacobian difference through the segment
direction vector. The arc (t1→t2) contributes to length, and its Jacobian
contribution is zero because both tangent points are on the same body.

MuJoCo does **not** compute derivatives of the wrapping function itself — the
tangent points are treated as fixed on the geom body for Jacobian purposes.

**New type for wrapping results:**
```rust
/// Result of a wrapping geometry computation.
/// Unlike MuJoCo's `mju_wrap` (which transforms to world frame internally),
/// our `sphere_wrap`/`cylinder_wrap` return tangent points in geom-local frame.
/// The caller (`mj_fwd_tendon_spatial`) transforms them to world frame.
enum WrapResult {
    /// Straight path is shorter — no wrapping around the obstacle.
    NoWrap,
    /// Path wraps around the obstacle, producing two tangent points and an arc.
    /// Tangent points are in the **geom-local frame** (caller transforms to world).
    /// `arc_length` is a scalar distance (frame-independent: same in local and world).
    Wrapped {
        tangent_point_1: Vector3<f64>,
        tangent_point_2: Vector3<f64>,
        arc_length: f64,
    },
}
```

**Pseudocode** (follows MuJoCo's pairwise loop structure):

```
fn mj_fwd_tendon_spatial(model, data, t):
    let adr = model.tendon_adr[t]
    let num = model.tendon_num[t]
    data.ten_J[t].fill(0.0)
    let mut total_length = 0.0
    let mut divisor: f64 = 1.0

    if num < 2:
        data.ten_length[t] = 0.0
        return  // degenerate — validation should prevent this

    let mut j = 0
    while j < num - 1:
        let type0 = model.wrap_type[adr + j]
        let type1 = model.wrap_type[adr + j + 1]

        // ---- Pulley handling ----
        // MuJoCo processes pulleys as a pair-skip: when a pulley appears as
        // type0 or type1, advance j by 1 without processing a segment.
        // Divisor is updated only when the pulley is type0.
        if type0 == WrapType::Pulley || type1 == WrapType::Pulley:
            if type0 == WrapType::Pulley:
                divisor = model.wrap_prm[adr + j]
            j += 1
            continue

        // ---- At this point, type0 must be a Site ----
        debug_assert!(type0 == WrapType::Site, "type0 must be Site after pulley check")
        let id0 = model.wrap_objid[adr + j]
        let p0 = data.site_xpos[id0]
        let body0 = model.site_body[id0]

        if type1 == WrapType::Site:
            // ---- Site–Site: straight segment ----
            let id1 = model.wrap_objid[adr + j + 1]
            let p1 = data.site_xpos[id1]
            let body1 = model.site_body[id1]

            let diff = p1 - p0
            let dist = diff.norm()
            // Length: accumulated unconditionally (matching MuJoCo).
            total_length += dist / divisor
            // Jacobian: guarded by distance threshold to avoid bogus direction.
            // MuJoCo uses mju_normalize3 which falls back to (1,0,0) for
            // near-zero vectors (mjMINVAL = 1e-15); we skip entirely for
            // distances below 1e-10 — a deliberate safety improvement.
            if dist > 1e-10 && body0 != body1:
                let dir = diff / dist
                accumulate_point_jacobian(model, &data.xpos, &data.xquat, &mut data.ten_J[t],
                    body1, p1, &dir, 1.0 / divisor)
                accumulate_point_jacobian(model, &data.xpos, &data.xquat, &mut data.ten_J[t],
                    body0, p0, &dir, -1.0 / divisor)

            j += 1  // advance to site1 (it becomes type0 on next iteration)

        else if type1 == WrapType::Geom:
            // ---- Site–Geom–Site: wrapping segment ----
            // Safety: j+2 < num is guaranteed by validation rules 1-2:
            //   - Rule 1: path ends with Site (so a Geom cannot be last)
            //   - Rule 2: every Geom must be followed by a Site
            // Together these ensure every Geom at position j+1 has a Site at j+2.
            debug_assert!(j + 2 < num, "Geom at j+1 must be followed by Site at j+2")
            let geom_id = model.wrap_objid[adr + j + 1]
            let geom_body = model.geom_body[geom_id]

            // The site AFTER the geom is at j+2
            let id1 = model.wrap_objid[adr + j + 2]
            let p1 = data.site_xpos[id1]
            let body1 = model.site_body[id1]

            // Transform site positions into geom-local frame
            let geom_pos = data.geom_xpos[geom_id]
            let geom_mat = data.geom_xmat[geom_id]
            let p0_local = geom_mat.transpose() * (p0 - geom_pos)
            let p1_local = geom_mat.transpose() * (p1 - geom_pos)

            // Resolve sidesite (if specified) in geom-local frame
            let sidesite_local = if model.wrap_sidesite[adr + j + 1] != usize::MAX:
                let ss_id = model.wrap_sidesite[adr + j + 1]
                Some(geom_mat.transpose() * (data.site_xpos[ss_id] - geom_pos))
            else:
                None

            // Dispatch to wrapping geometry
            let wrap_result = match model.geom_type[geom_id]:
                Sphere   => sphere_wrap(p0_local, p1_local,
                               model.geom_size[geom_id].x, sidesite_local)
                Cylinder => cylinder_wrap(p0_local, p1_local,
                               model.geom_size[geom_id].x, sidesite_local)
                _        => unreachable (validated at model build)

            match wrap_result:
              Wrapped { tangent_point_1, tangent_point_2, arc_length } =>
                // Transform tangent points back to world frame
                let t1 = geom_pos + geom_mat * tangent_point_1
                let t2 = geom_pos + geom_mat * tangent_point_2

                // Waypoints: [p0, t1, t2, p1], bodies: [body0, geom_body, geom_body, body1]
                // 3 sub-segments, each contributing length and Jacobian.

                // Length: all three components accumulated unconditionally.
                // MuJoCo computes this as a single expression:
                //   L += (dist1 + wlen + dist3) / divisor
                let d1 = t1 - p0;  let dist1 = d1.norm()
                let d3 = p1 - t2;  let dist3 = d3.norm()
                total_length += (dist1 + arc_length + dist3) / divisor

                // Sub-segment 1: p0 (body0) → t1 (geom_body)
                if dist1 > 1e-10 && body0 != geom_body:
                    let dir1 = d1 / dist1
                    accumulate_point_jacobian(model, &data.xpos, &data.xquat, &mut data.ten_J[t], geom_body, t1, &dir1, 1.0/divisor)
                    accumulate_point_jacobian(model, &data.xpos, &data.xquat, &mut data.ten_J[t], body0, p0, &dir1, -1.0/divisor)

                // Sub-segment 2: t1 → t2 (arc on geom surface)
                // Both endpoints on geom_body → Jacobian difference is zero.
                // No Jacobian contribution; length already included above.

                // Sub-segment 3: t2 (geom_body) → p1 (body1)
                if dist3 > 1e-10 && geom_body != body1:
                    let dir3 = d3 / dist3
                    accumulate_point_jacobian(model, &data.xpos, &data.xquat, &mut data.ten_J[t], body1, p1, &dir3, 1.0/divisor)
                    accumulate_point_jacobian(model, &data.xpos, &data.xquat, &mut data.ten_J[t], geom_body, t2, &dir3, -1.0/divisor)

              NoWrap =>
                // No wrapping — straight segment p0 → p1
                let diff = p1 - p0;  let dist = diff.norm()
                total_length += dist / divisor
                if dist > 1e-10 && body0 != body1:
                    let dir = diff / dist
                    accumulate_point_jacobian(model, &data.xpos, &data.xquat, &mut data.ten_J[t], body1, p1, &dir, 1.0/divisor)
                    accumulate_point_jacobian(model, &data.xpos, &data.xquat, &mut data.ten_J[t], body0, p0, &dir, -1.0/divisor)

            j += 2  // advance past the geom to site1 (becomes type0 next)

        else:
            unreachable!("type0 must be Site after pulley check; \
                          validation ensures path starts with Site and \
                          geoms are always followed by sites")

    data.ten_length[t] = total_length
```

**`accumulate_point_jacobian()`** — walks from a body to the root, projecting each
joint's velocity contribution through a direction vector and accumulating into a
tendon Jacobian row. This is the same kinematic chain walk as
`compute_contact_jacobian`'s inner `add_body_jacobian` closure
(`constraint/jacobian.rs`), but operating on a `DVector<f64>` (1×nv) instead
of a `DMatrix` row. All joint type formulas (Hinge, Slide, Ball, Free) are
verified to exactly match the existing `add_body_jacobian` implementation.
**Note:** `compute_body_jacobian_at_point()` was dead code with a broken
implementation — it has been deleted and replaced by the canonical `mj_jac()`
API (DT-74). Use `mj_jac()` for full `(jacp 3×nv, jacr 3×nv)` Jacobians.

**Borrow safety:** The function takes `ten_j: &mut DVector<f64>` as a direct
mutable reference (not through `data`), plus immutable references `xpos` and
`xquat` for the body poses. The caller obtains these via field-level borrow
splitting on `Data`:
```rust
accumulate_point_jacobian(model, &data.xpos, &data.xquat,
    &mut data.ten_J[t], body_id, &point, &dir, scale);
```
This is safe because `data.ten_J`, `data.xpos`, and `data.xquat` are disjoint
fields of `Data`. The alternative — implementing as a closure that captures
`data` (like `add_body_jacobian`) — would work too, but a standalone function
is more testable.

```
fn accumulate_point_jacobian(model: &Model, xpos: &[Vector3<f64>],
                             xquat: &[UnitQuaternion<f64>],
                             ten_j: &mut DVector<f64>,
                             body_id: usize, point: &Vector3<f64>,
                             direction: &Vector3<f64>, scale: f64):
    if body_id == 0: return  // world body has no DOFs

    let mut current = body_id
    while current != 0:
        let jnt_start = model.body_jnt_adr[current]
        let jnt_end = jnt_start + model.body_jnt_num[current]
        for jnt_id in jnt_start..jnt_end:
            let dof = model.jnt_dof_adr[jnt_id]
            let jnt_body = model.jnt_body[jnt_id]
            match model.jnt_type[jnt_id]:
                Hinge =>
                    // jnt_axis and jnt_pos are body-local (declared at line 823);
                    // rotated to world frame by xquat[body].
                    let axis = xquat[jnt_body] * model.jnt_axis[jnt_id]
                    let jpos = xpos[jnt_body]
                            + xquat[jnt_body] * model.jnt_pos[jnt_id]
                    let r = point - jpos
                    ten_j[dof] += scale * direction.dot(&axis.cross(&r))
                Slide =>
                    let axis = xquat[jnt_body] * model.jnt_axis[jnt_id]
                    ten_j[dof] += scale * direction.dot(&axis)
                Ball =>
                    let jpos = xpos[jnt_body]
                            + xquat[jnt_body] * model.jnt_pos[jnt_id]
                    let r = point - jpos
                    let rot = xquat[jnt_body].to_rotation_matrix()
                    for i in 0..3:
                        let omega = rot * Vector3::ith(i, 1.0)
                        ten_j[dof+i] += scale * direction.dot(&omega.cross(&r))
                Free =>
                    // Translational DOFs: direction projects directly.
                    ten_j[dof+0] += scale * direction.x
                    ten_j[dof+1] += scale * direction.y
                    ten_j[dof+2] += scale * direction.z
                    // Rotational DOFs: same formula as Ball — body-frame axes
                    // rotated to world frame. MuJoCo's cdof for free joint
                    // angular DOFs uses xmat columns (= R * e_i), identical
                    // to the Ball joint case.
                    // BUG NOTE: The existing `add_body_jacobian` (line 7923)
                    // uses world-frame unit vectors (e_x, e_y, e_z) instead of
                    // body-frame (R*e_i). This is a pre-existing bug that
                    // produces incorrect angular Jacobian columns for free
                    // bodies with non-identity orientation. We implement the
                    // correct formula here (matching MuJoCo and the Ball case).
                    // The `add_body_jacobian` bug should be fixed separately.
                    let jpos = xpos[jnt_body]
                    let r = point - jpos
                    let rot = xquat[jnt_body].to_rotation_matrix()
                    for i in 0..3:
                        let omega = rot * Vector3::ith(i, 1.0)
                        ten_j[dof+3+i] += scale * direction.dot(&omega.cross(&r))
        current = model.body_parent[current]
```

**Convention note:** `model.jnt_pos` stores the joint anchor position in body
frame (declared at line 823: `/// Joint anchor position in body frame.`). The
world-frame anchor is computed as `xpos[body] + xquat[body] * jnt_pos[jnt]`
(using the function's `xpos`/`xquat` parameters, which are `&data.xpos` and
`&data.xquat` at the call site). This matches the existing `add_body_jacobian`
closure (lines 7889, 7900) and all other joint anchor references in the
pipeline (`mj_fwd_position` line 7244, `mj_rne` line 7342). Similarly,
`model.jnt_axis` is body-local and rotated to world via
`xquat[body] * jnt_axis[jnt]`. The codebase uses `UnitQuaternion`
multiplication (`xquat * vec`) as the standard rotation operation; `xmat`
(stored as `Matrix3<f64>`) is available but `xquat` is preferred for rotating
individual vectors.

##### 4.4 Tendon Velocity

No changes required. `mj_fwd_velocity()` already computes:
```rust
data.ten_velocity[t] = data.ten_J[t].dot(&data.qvel);
```
Once `ten_J[t]` is correctly populated by 4.3, velocity is automatically correct.

##### 4.5 Tendon Force Transmission

**Note on force computation:** The force formula in `mj_fwd_passive()`
(`force = -stiffness * (length - springlength) - damping * velocity -
frictionloss * tanh(velocity * smoothing)`) is tendon-type-agnostic — it
operates on `ten_length[t]`, `ten_velocity[t]`, and produces `ten_force[t]`.
No modification is needed for spatial tendons. Only the **transmission**
(mapping `ten_force` to generalized forces via `J^T`) changes.

Three locations map tendon forces to joint-space generalized forces using the
fixed-tendon wrap-array pattern. All three must be fixed for spatial tendons.

**Pattern:** For `TendonType::Fixed`, the existing wrap-array loop is correct and
efficient. For `TendonType::Spatial`, use the already-computed `ten_J[t]` via
`J^T * force`:

```
fn apply_tendon_force(model: &Model, ten_j: &DVector<f64>, tendon_type: TendonType,
                      t: usize, force: f64, target: &mut DVector<f64>):
    match tendon_type:
        TendonType::Fixed =>
            let adr = model.tendon_adr[t]
            let num = model.tendon_num[t]
            for w in adr..(adr + num):
                let dof_adr = model.wrap_objid[w]
                let coef = model.wrap_prm[w]
                if dof_adr < model.nv:
                    target[dof_adr] += coef * force
        TendonType::Spatial =>
            for dof in 0..model.nv:
                let j = ten_j[dof]
                if j != 0.0:
                    target[dof] += j * force
```

**Borrow safety:** The function takes `ten_j: &DVector<f64>` (an immutable
reference to `data.ten_J[t]`) and `target: &mut DVector<f64>` (a mutable
reference to `data.qfrc_passive` / `data.qfrc_constraint` / `data.qfrc_actuator`).
These are disjoint fields of `Data`, so Rust's borrow checker allows
simultaneous access via field-level splitting:
```rust
apply_tendon_force(model, &data.ten_J[t], model.tendon_type[t],
                   t, force, &mut data.qfrc_passive);
```
This pattern avoids passing `&mut data` (which would prevent reading `ten_J`).

**Performance note:** The spatial branch iterates all `nv` DOFs. A spatial
tendon typically has 6-20 nonzero Jacobian entries, so this is wasteful for
large models (nv > 100). MuJoCo avoids this by computing the Jacobian
contribution inline during the chain walk, never materializing a full `ten_J`
row. For initial correctness, the O(nv) scan is acceptable. A follow-up
optimization could cache nonzero DOF indices during `mj_fwd_tendon_spatial()`
and iterate only those indices in `apply_tendon_force`.
Tracked in [future_work_10d.md](./future_work_10d.md) §DT-34.

**Apply this pattern at all three sites:**

1. **`mj_fwd_passive()`** — after computing `ten_force[t]`:
   `apply_tendon_force(model, &data.ten_J[t], model.tendon_type[t], t, force, &mut data.qfrc_passive)`.
2. **`mj_fwd_constraint()`** — tendon limit penalty forces (lines 9999-10050).
   **Sign convention** (verified against the existing code): for lower-limit
   violations (length < min), the code uses `+= coef * force` (line 10028);
   for upper-limit violations (length > max), it uses `-= coef * force`
   (line 10046). The `force` variable is always non-negative — computed as
   `stiffness * penetration + damping * vel_into` where `penetration` is
   positive when violated and `vel_into` is clamped to `max(0.0)`. With
   `apply_tendon_force`:
   - Lower limit: `apply_tendon_force(model, &data.ten_J[t], ..., t, +force, &mut data.qfrc_constraint)`
     (force pushes tendon length up, away from lower bound)
   - Upper limit: `apply_tendon_force(model, &data.ten_J[t], ..., t, -force, &mut data.qfrc_constraint)`
     (force pushes tendon length down, away from upper bound)
3. **`mj_fwd_actuation()`** — `ActuatorTransmission::Tendon` branch (the
   local variable is `tendon_id` at line 6907, aliased from `trnid`):
   `apply_tendon_force(model, &data.ten_J[tendon_id], ..., tendon_id, gear * force, &mut data.qfrc_actuator)`.

##### 4.6 Model-Build-Time Fixes

Two model-build-time computations use the wrap-array pattern and need spatial
tendon handling:

**A. `compute_muscle_params()` Phase 2 — Jacobian vector for `acc0`:**

Currently builds `j_vec[dof] = gear * coef` from wrap arrays (lines 2356-2368).
For spatial tendons, the Jacobian is configuration-dependent — it must be
computed from the actual tendon path at `qpos0`.

The existing `mj_fwd_position(self, &mut data)` call at line 2337 already
invokes `mj_fwd_tendon()` internally, which populates `data.ten_J`. No
additional FK or tendon evaluation call is needed — just read `data.ten_J[tid]`
after `mj_fwd_position` returns.

Replace the `ActuatorTransmission::Tendon` branch in Phase 2 (lines 2356-2368)
with a tendon-type dispatch:

```
ActuatorTransmission::Tendon => {
    let tid = self.actuator_trnid[i];
    if tid < self.ntendon {
        match self.tendon_type[tid] {
            TendonType::Fixed => {
                // Existing wrap-array pattern (constant J for fixed tendons).
                let adr = self.tendon_adr[tid];
                let num = self.tendon_num[tid];
                for w in adr..(adr + num) {
                    let dof_adr = self.wrap_objid[w];
                    let coef = self.wrap_prm[w];
                    if dof_adr < self.nv {
                        j_vec[dof_adr] = gear * coef;
                    }
                }
            }
            TendonType::Spatial => {
                // Configuration-dependent J — use ten_J from FK at qpos0.
                // mj_fwd_position() already called mj_fwd_tendon() which
                // populated data.ten_J[tid].
                for dof in 0..self.nv {
                    j_vec[dof] = gear * data.ten_J[tid][dof];
                }
            }
        }
    }
}
```

**B. `compute_muscle_params()` Phase 1 — `actuator_lengthrange` for unlimited
spatial-tendon actuators:**

Currently estimates length range from joint ranges via wrap-array DOF lookup
(lines 2294-2317). For spatial tendons, the tendon length is a nonlinear
function of joint positions — the linear estimation is wrong. The limited
branch (line 2291) already uses `tendon_range` directly, which works for
spatial tendons. The unlimited branch (lines 2294-2317) must be guarded.

Insert a spatial tendon guard at the top of the unlimited branch (after
`} else {` on line 2294):

```
} else {
    // For unlimited spatial tendons, wrap-array DOF lookup is
    // semantically wrong (wrap_objid holds site/geom IDs, not DOFs).
    // Skip estimation and leave lengthrange = (0, 0).
    if self.tendon_type[tid] == TendonType::Spatial {
        log::warn!("Unlimited spatial tendon {}: cannot estimate \
                    actuator_lengthrange from joint ranges. Specify \
                    explicit lengthrange in MJCF for muscle actuators.", tid);
        continue;  // leave lengthrange = (0, 0)
    }
    // ... existing fixed-tendon wrap-array estimation ...
```

Fix: if the spatial tendon has declared `tendon_range` limits, use those (this
path already works). If unlimited, skip the range estimation and leave
`lengthrange = (0, 0)` — muscle actuators on unlimited spatial tendons will
need explicit `lengthrange` in MJCF. Log a warning.
Tracked in [future_work_10j.md](./future_work_10j.md) §DT-78.

##### 4.7 Wrapping Geometry: Sphere

Compute the shortest path around a sphere between two points. The algorithm
works in the geom-local frame where the sphere is centered at the origin.

**Sidesite selection (applies to both sphere and cylinder):** MuJoCo computes
**both** candidate wrapping solutions and selects using a goodness heuristic:
with sidesite, goodness = dot(normalized midpoint, sidesite direction); without
sidesite, goodness = negative squared chord distance (shorter chord → shorter
arc). Self-intersecting candidates are penalized (goodness = -10000) during
evaluation. If even the selected candidate self-intersects, wrapping is rejected
entirely. This two-phase approach (penalize then reject) avoids discarding a
valid candidate when only one of the two self-intersects.

```
fn sphere_wrap(p1: Vector3, p2: Vector3, radius: f64, sidesite: Option<Vector3>)
    -> WrapResult
{
    // 1. Early exits.
    if radius <= 0.0:
        return WrapResult::NoWrap  // degenerate geom
    if p1.norm() < radius || p2.norm() < radius:
        return WrapResult::NoWrap  // endpoint inside sphere

    // 2. Check if the straight-line path misses the sphere.
    let d = p2 - p1
    if d.norm_squared() < 1e-20:
        return WrapResult::NoWrap  // coincident sites
    let t_param = -(p1.dot(&d)) / d.norm_squared()
    let closest = p1 + t_param.clamp(0.0, 1.0) * d
    if closest.norm() > radius:
        // Straight line clears the sphere. MuJoCo still allows wrapping if
        // a sidesite is present AND the closest point is on the opposite side
        // from the sidesite (sidesite-forced wrapping: the tendon is pulled
        // around the geometry even though the direct path would clear it).
        // Note: this test uses the raw `closest` and `sidesite` vectors in 3D.
        // Since `closest` lies in the p1-p2 plane (a linear combination of p1
        // and p2), the out-of-plane component of `sidesite` is automatically
        // orthogonal to `closest` and contributes zero to the dot product.
        // The result is equivalent to MuJoCo's 2D projected test.
        if sidesite.is_none() || closest.dot(&sidesite.unwrap()) >= 0.0:
            return WrapResult::NoWrap
        // else: sidesite forces wrapping — fall through

    // 3. Compute the wrapping plane normal.
    //    The plane contains the sphere center (origin), p1, and p2.
    let mut plane_normal = p1.cross(&p2)
    if plane_normal.norm() < 1e-10 * p1.norm() * p2.norm():
        // Degenerate: p1, origin, p2 are collinear.
        // Construct an arbitrary perpendicular by crossing with the
        // least-aligned cardinal axis (achieves the same goal as MuJoCo's
        // fallback, which uses a different construction but produces an
        // equally valid arbitrary perpendicular).
        let u = p1.normalize()
        let min_axis = if u.x.abs() <= u.y.abs() && u.x.abs() <= u.z.abs():
            Vector3::x()
        else if u.y.abs() <= u.z.abs():
            Vector3::y()
        else:
            Vector3::z()
        plane_normal = u.cross(&min_axis)
    plane_normal = plane_normal.normalize()

    // 4. Construct 2D coordinate system in the wrapping plane.
    //    MuJoCo projects all points into a 2D basis within the wrapping plane
    //    for tangent computation and intersection testing.
    //    axis0 = normalized p1 direction (the "x" axis of the 2D frame)
    //    axis1 = plane_normal × axis0 (the "y" axis of the 2D frame)
    let axis0 = p1.normalize()
    let axis1 = plane_normal.cross(&axis0).normalize()

    // Project endpoints and sidesite into the 2D wrapping plane.
    // Note: p1_2d.y is always 0 because axis0 = p1.normalize(), so
    // p1.dot(axis1) = 0 (axis1 is perpendicular to axis0 by construction).
    // This is expected — p1 defines the "x" axis of the 2D frame.
    let p1_2d = Vector2::new(p1.dot(&axis0), p1.dot(&axis1))  // = (||p1||, 0)
    let p2_2d = Vector2::new(p2.dot(&axis0), p2.dot(&axis1))
    let ss_2d = sidesite.map(|ss| {
        let v = Vector2::new(ss.dot(&axis0), ss.dot(&axis1))
        // Normalize to unit direction. The goodness heuristic (step 5) compares
        // dot products with the midpoint direction — only the sidesite direction
        // matters, not its magnitude. Both candidates use the same sidesite, so
        // any uniform scaling would cancel in the comparison.
        if v.norm() > 1e-10 { v.normalize() } else { v }
    })

    // 4b. Compute both candidate tangent-point pairs (±normal).
    //     Tangent point from external point p to circle of radius r:
    //       cos_theta = r / ||p||, sin_theta = sqrt(1 - cos_theta^2)
    //       Tangent = rotate unit(p) by theta in the wrapping plane.
    //     3D tangent points for the final result:
    let (t1_a, t2_a) = compute_tangent_pair(p1, p2, radius, plane_normal)
    let (t1_b, t2_b) = compute_tangent_pair(p1, p2, radius, -plane_normal)
    //     2D projections for intersection testing:
    let t1a_2d = Vector2::new(t1_a.dot(&axis0), t1_a.dot(&axis1))
    let t2a_2d = Vector2::new(t2_a.dot(&axis0), t2_a.dot(&axis1))
    let t1b_2d = Vector2::new(t1_b.dot(&axis0), t1_b.dot(&axis1))
    let t2b_2d = Vector2::new(t2_b.dot(&axis0), t2_b.dot(&axis1))

    // 5. Select the best candidate using MuJoCo's goodness heuristic.
    //    Phase 1: Compute goodness score.
    let (mut good_a, mut good_b) = if let Some(sd) = ss_2d:
        // With sidesite: goodness = dot(normalized 2D midpoint, sidesite direction).
        let sum_a = t1a_2d + t2a_2d
        let sum_b = t1b_2d + t2b_2d
        let ga = if sum_a.norm() > 1e-10 { sum_a.normalize().dot(&sd) }
                 else { -1e10 }
        let gb = if sum_b.norm() > 1e-10 { sum_b.normalize().dot(&sd) }
                 else { -1e10 }
        (ga, gb)
    else:
        // No sidesite: goodness = negative squared chord distance.
        let chord_a = (t1a_2d - t2a_2d).norm_squared()
        let chord_b = (t1b_2d - t2b_2d).norm_squared()
        (-chord_a, -chord_b)

    // Phase 1b: Penalize self-intersecting candidates.
    if segments_intersect_2d(p1_2d, t1a_2d, p2_2d, t2a_2d):
        good_a = -10000.0
    if segments_intersect_2d(p1_2d, t1b_2d, p2_2d, t2b_2d):
        good_b = -10000.0

    // Phase 2: Select the better candidate with ind tracking.
    //          +normal → MuJoCo candidate i=1 → ind=1
    //          -normal → MuJoCo candidate i=0 → ind=0
    //          On tie (good_a == good_b), select candidate A (ind=1) to match
    //          MuJoCo's `i = (good[0] > good[1] ? 0 : 1)` which defaults to i=1.
    let (t1, t2, t1_2d, t2_2d, ind) = if good_a >= good_b:
        (t1_a, t2_a, t1a_2d, t2a_2d, 1) else: (t1_b, t2_b, t1b_2d, t2b_2d, 0)

    // Phase 3: Reject wrapping if the selected candidate self-intersects.
    if segments_intersect_2d(p1_2d, t1_2d, p2_2d, t2_2d):
        return WrapResult::NoWrap

    // 6. Arc length via directional wrap angle (same as cylinder wrapping).
    //    MuJoCo uses `length_circle` with `ind` for both sphere and cylinder.
    //    With the crossed tangent pairing, the arc can be > π when sidesite
    //    forces wrapping the long way around, so plain acos is insufficient.
    let wrap_angle = directional_wrap_angle(t1_2d, t2_2d, ind)
    let arc_length = radius * wrap_angle

    WrapResult::Wrapped { tangent_point_1: t1, tangent_point_2: t2, arc_length }
}
```

**`compute_tangent_pair(p1, p2, radius, normal)` helper:**

Computes the tangent points from two external points to a sphere of the given
radius, using `normal` to select which of the two tangent lines to use. Each
tangent point is computed independently.

**IMPORTANT:** MuJoCo's `wrap_circle` applies the rotation in **opposite
directions** for the two endpoints within a single candidate. This is because
a consistent wrapping path requires the tangent at the entry point and the
tangent at the exit point to be on opposite sides of the circle. To match
MuJoCo, `p2` uses the negated `normal`, producing a crossed tangent pair.

```
fn sphere_tangent_point(p: Vector3, radius: f64, normal: Vector3) -> Vector3:
    // Tangent point from external point p to sphere of radius r at origin.
    // The tangent line is in the half-plane defined by (origin, p, normal).
    //
    // Geometric reasoning: `normal` is the wrapping plane normal (perpendicular
    // to the plane containing origin, p1, p2). Since p lies in this plane,
    // u = p/||p|| also lies in it. Therefore `normal × u` produces a vector
    // that is (a) in the wrapping plane (perpendicular to `normal`) and
    // (b) perpendicular to `u`. This gives us a 2D basis {u, v} within the
    // wrapping plane for constructing the tangent point via rotation.
    //
    // Precondition: ||p|| >= radius (caller guards via early exit in sphere_wrap).
    // Precondition: ||normal|| ≈ 1 (caller normalizes plane_normal before calling).
    let d = p.norm()
    debug_assert!(d >= radius, "sphere_tangent_point: p is inside sphere")
    let cos_theta = radius / d
    let sin_theta = (1.0 - cos_theta * cos_theta).max(0.0).sqrt()
    let u = p / d                        // unit vector toward p (in wrapping plane)
    let v = normal.cross(&u).normalize() // perpendicular to u, in wrapping plane
    // Tangent point on sphere surface at angle theta from the center-to-p line:
    radius * (u * cos_theta + v * sin_theta)

fn compute_tangent_pair(p1, p2, radius, normal) -> (Vector3, Vector3):
    (sphere_tangent_point(p1, radius, normal),
     sphere_tangent_point(p2, radius, -normal))  // negated for p2 (crossed pairing)
```

Reference: `sim-tendon/src/wrapping.rs` `SphereWrap::compute_wrap()`.
Reference: MuJoCo `engine_util_misc.c` `mju_wrap()` — dual-candidate selection.

##### 4.8 Wrapping Geometry: Cylinder

Compute the shortest path around an infinite cylinder. The algorithm works in
the geom-local frame where the cylinder axis is Z and center is the origin.

The cylinder wrapping problem reduces to a 2D circle-tangent problem in the XY
plane (perpendicular to the cylinder axis), plus Z-interpolation for the axial
(helical) component.

```
fn cylinder_wrap(p1: Vector3, p2: Vector3, radius: f64, sidesite: Option<Vector3>)
    -> WrapResult
{
    // 1. Project onto XY plane (perpendicular to cylinder axis).
    let p1_xy = Vector2::new(p1.x, p1.y)
    let p2_xy = Vector2::new(p2.x, p2.y)

    // 2. Early exits.
    if radius <= 0.0:
        return WrapResult::NoWrap  // degenerate geom
    if p1_xy.norm() < radius || p2_xy.norm() < radius:
        return WrapResult::NoWrap  // endpoint inside cylinder cross-section

    // 3. Compute sidesite XY projection once (used in both early-exit and
    //    candidate selection). MuJoCo computes this once in mju_wrap and
    //    passes it to wrap_circle as the `side` parameter.
    let ss_dir = sidesite.map(|ss| {
        let ss_xy = Vector2::new(ss.x, ss.y);
        if ss_xy.norm() > 1e-10 { ss_xy.normalize() } else { ss_xy }
    });

    // 4. Check if the 2D line segment misses the cylinder cross-section.
    let d_xy = p2_xy - p1_xy
    if d_xy.norm_squared() < 1e-20:
        return WrapResult::NoWrap  // coincident sites in XY projection
    let t_param = -(p1_xy.dot(&d_xy)) / d_xy.norm_squared()
    let closest = p1_xy + t_param.clamp(0.0, 1.0) * d_xy
    if closest.norm() > radius:
        // Straight line clears the cylinder. Same sidesite-forced wrapping
        // logic as sphere (see §4.7 step 2): only skip wrapping if there is
        // no sidesite, or the closest point is on the sidesite's side.
        // MuJoCo normalizes the sidesite projection so the test is purely
        // directional (not affected by sidesite distance from axis).
        if ss_dir.is_none() || closest.dot(&ss_dir.unwrap()) >= 0.0:
            return WrapResult::NoWrap
        // else: sidesite forces wrapping — fall through

    // 5. Compute both candidate 2D tangent-point pairs (±wrap direction).
    //    Same dual-candidate approach as sphere wrapping (see §4.7).
    //
    //    IMPORTANT sign-to-ind mapping: our `circle_tangent_2d` with `sign=+1`
    //    produces the same tangent points as MuJoCo's `wrap_circle` with `sgn=-1`
    //    (candidate `i=1`). This is because MuJoCo's formula places the `sgn` term
    //    on the opposite perpendicular component. Therefore:
    //      sign=+1  →  MuJoCo candidate i=1  →  ind=1
    //      sign=-1  →  MuJoCo candidate i=0  →  ind=0
    //    The `ind` value must match the MuJoCo candidate index for
    //    `directional_wrap_angle` to compute the correct arc direction.
    let (t1a_xy, t2a_xy) = compute_tangent_pair_2d(p1_xy, p2_xy, radius, +1)  // MuJoCo i=1
    let (t1b_xy, t2b_xy) = compute_tangent_pair_2d(p1_xy, p2_xy, radius, -1)  // MuJoCo i=0

    // 6. Select the best candidate using MuJoCo's goodness heuristic.
    //    MuJoCo evaluates candidates in two phases:
    //    Phase 1: Compute goodness score (sidesite alignment or chord distance).
    //    Phase 1b: Penalize self-intersecting candidates (good = -10000).
    //    Phase 2: Pick the candidate with higher goodness.
    //    Phase 3: Reject wrapping entirely if the selected candidate still
    //             self-intersects (both candidates were bad).
    let (mut good_a, mut good_b) = if let Some(sd) = ss_dir:
        // With sidesite: goodness = dot(normalized midpoint, sidesite direction).
        let sum_a = t1a_xy + t2a_xy
        let sum_b = t1b_xy + t2b_xy
        let ga = if sum_a.norm() > 1e-10 { sum_a.normalize().dot(&sd) }
                 else { -1e10 }  // degenerate midpoint
        let gb = if sum_b.norm() > 1e-10 { sum_b.normalize().dot(&sd) }
                 else { -1e10 }
        (ga, gb)
    else:
        // No sidesite: goodness = negative squared chord distance between
        // tangent points. Shorter chord → shorter arc → higher goodness.
        // This is MuJoCo's `good[i] = -dd` heuristic.
        let chord_a = (t1a_xy - t2a_xy).norm_squared()
        let chord_b = (t1b_xy - t2b_xy).norm_squared()
        (-chord_a, -chord_b)

    // Phase 1b: Penalize self-intersecting candidates.
    if segments_intersect_2d(p1_xy, t1a_xy, p2_xy, t2a_xy):
        good_a = -10000.0
    if segments_intersect_2d(p1_xy, t1b_xy, p2_xy, t2b_xy):
        good_b = -10000.0

    // Phase 2: Select the better candidate with correct ind mapping.
    //          On tie (good_a == good_b), select candidate A (ind=1) to match
    //          MuJoCo's `i = (good[0] > good[1] ? 0 : 1)` which defaults to i=1.
    let (t1_xy, t2_xy, ind) = if good_a >= good_b:
        (t1a_xy, t2a_xy, 1)   // sign=+1 → MuJoCo ind=1
    else:
        (t1b_xy, t2b_xy, 0)   // sign=-1 → MuJoCo ind=0

    // Phase 3: Reject wrapping if the selected candidate self-intersects.
    //          (Both candidates were penalized, but the "less bad" one won.)
    if segments_intersect_2d(p1_xy, t1_xy, p2_xy, t2_xy):
        return WrapResult::NoWrap

    // 7. Compute directional wrap angle (MuJoCo's length_circle algorithm).
    //    The base angle from acos is in [0, π]. A 2D cross product of the
    //    tangent points determines rotational direction. The candidate index
    //    `ind` determines whether to flip to the reflex angle (2π − θ).
    //    This correctly handles wrapping > 180° when sidesite forces the
    //    "long way" around.
    let wrap_angle = directional_wrap_angle(t1_xy, t2_xy, ind)

    // 8. Z-interpolation: path-length-proportional (MuJoCo formula).
    //    All coordinates (p1, p2, t1_xy, t2_xy) are in the geom-local frame
    //    where Z is the cylinder axis. The returned tangent points are also
    //    in geom-local frame — the caller (`mj_fwd_tendon_spatial`) transforms
    //    them to world frame via `geom_pos + geom_mat * tangent_point`.
    //    The axial (Z) coordinate is linearly interpolated based on the
    //    fraction of total 2D path length (straight1 + arc + straight2).
    //    This produces a helical path with uniform axial velocity.
    let L0 = (p1_xy - t1_xy).norm()         // 2D straight: p1 → tangent1
    let wlen = radius * wrap_angle            // 2D arc on cylinder surface (angle is in [0, 2π))
    let L1 = (p2_xy - t2_xy).norm()         // 2D straight: tangent2 → p2
    let total_2d = L0 + wlen + L1
    if total_2d < 1e-10:
        return WrapResult::NoWrap
    let t1_z = p1.z + (p2.z - p1.z) * L0 / total_2d
    let t2_z = p1.z + (p2.z - p1.z) * (L0 + wlen) / total_2d

    // 9. Arc length of the helical path on the cylinder surface.
    let circ_arc = wlen                       // circumferential arc
    let axial_disp = t2_z - t1_z             // axial travel on surface
    let arc_length = (circ_arc * circ_arc + axial_disp * axial_disp).sqrt()

    let t1 = Vector3::new(t1_xy.x, t1_xy.y, t1_z)
    let t2 = Vector3::new(t2_xy.x, t2_xy.y, t2_z)

    WrapResult::Wrapped { tangent_point_1: t1, tangent_point_2: t2, arc_length }
}
```

**`compute_tangent_pair_2d(p1_xy, p2_xy, radius, sign)` helper:**

2D analog of `compute_tangent_pair`. Computes tangent points from two external
points to a circle of the given radius at the origin in 2D. `sign` (+1 or -1)
selects which of the two tangent lines to use (clockwise vs counterclockwise).

```
fn circle_tangent_2d(p: Vector2, radius: f64, sign: i32) -> Vector2:
    let d = p.norm()
    let cos_theta = radius / d
    let sin_theta = (1.0 - cos_theta * cos_theta).max(0.0).sqrt()
    let u = p / d
    let v = Vector2::new(-u.y, u.x) * sign as f64  // perpendicular (rotated 90°)
    radius * (u * cos_theta + v * sin_theta)

fn compute_tangent_pair_2d(p1_xy, p2_xy, radius, sign) -> (Vector2, Vector2):
    (circle_tangent_2d(p1_xy, radius, sign),
     circle_tangent_2d(p2_xy, radius, -sign))  // negated for p2 (crossed pairing)
```

**`directional_wrap_angle(t1, t2, ind)` — MuJoCo's `length_circle` algorithm:**

Computes the directional arc angle between two tangent points on a circle.
Unlike a simple `acos` (which returns [0, π]), this function can return angles
in [0, 2π) by using the 2D cross-product sign and the candidate index to
determine whether to take the reflex angle. This is critical for wrapping
> 180° when a sidesite forces the "long way" around.

```
fn directional_wrap_angle(t1: Vector2, t2: Vector2, ind: usize) -> f64:
    // Base angle via acos: [0, π]
    let t1n = t1.normalize()
    let t2n = t2.normalize()
    let base_angle = t1n.dot(&t2n).clamp(-1.0, 1.0).acos()

    // 2D cross product determines rotational direction of t1 → t2.
    let cross = t1.y * t2.x - t1.x * t2.y

    // MuJoCo's convention (from length_circle in engine_util_misc.c):
    // If (cross > 0 && ind == 1) || (cross < 0 && ind == 0):
    //     angle = 2π - base_angle   (take the reflex arc)
    // This means: candidate 0 wraps in the "natural" direction for the
    // tangent arrangement; candidate 1 wraps the "other way."
    let angle = if (cross > 0.0 && ind == 1) || (cross < 0.0 && ind == 0):
        2.0 * PI - base_angle
    else:
        base_angle

    angle  // in [0, 2π)
```

**Note on sign-to-ind mapping and crossed pairing:** Our
`circle_tangent_2d(p, r, sign)` uses the perpendicular `v = (-u.y, u.x) * sign`,
which is the **negated** convention compared to MuJoCo's `wrap_circle` (where
`sgn` multiplies the opposite component). Therefore `sign=+1` corresponds to
MuJoCo candidate `i=1` (with `ind=1`), and `sign=-1` corresponds to MuJoCo
candidate `i=0` (with `ind=0`). Additionally, `compute_tangent_pair_2d` and
`compute_tangent_pair` apply **crossed pairing**: p1 uses `+sign`/`+normal`
while p2 uses `-sign`/`-normal`, matching MuJoCo's opposite-sign-per-endpoint
convention in `wrap_circle`.
The `ind` parameter passed to `directional_wrap_angle` must always equal the
MuJoCo candidate index for the selected tangent pair — NOT the spec's `sign`
parameter. For the no-sidesite case, MuJoCo selects via the chord-distance
goodness heuristic (shorter chord → higher goodness), which typically picks
the shorter arc but is not guaranteed to always be `ind=0`.

**Both sphere and cylinder use `directional_wrap_angle` with `ind`.** MuJoCo
calls `length_circle` (which `directional_wrap_angle` implements) for both
geometries. With the crossed tangent pairing (p2 uses negated normal/sign),
the arc between tangent points CAN exceed π when sidesite forces wrapping the
long way around. The `ind` parameter is essential for determining the correct
arc direction in both cases.

Reference: `sim-tendon/src/wrapping.rs` `CylinderWrap::compute_wrap()`.
Reference: MuJoCo `engine_util_misc.c` `length_circle()` — directional arc.
Reference: MuJoCo `engine_util_misc.c` `mju_wrap()` — Z-interpolation formula.
Reference: MuJoCo `engine_util_misc.c` `is_intersect()` — segment intersection test.

**`segments_intersect_2d(a1, a2, b1, b2)` — 2D segment intersection test:**

Returns true if line segment a1→a2 crosses line segment b1→b2. Used by both
sphere and cylinder wrapping to reject self-intersecting tendon paths (MuJoCo's
`is_intersect` check). Uses non-strict inequalities (`>=`, `<=`) matching
MuJoCo's implementation — endpoint-touching segments are considered
intersecting. For sphere wrapping, the 3D tangent points are projected
into the wrapping plane's 2D coordinate system before this test (see §4.7
step 4).

```
fn segments_intersect_2d(a1, a2, b1, b2) -> bool:
    let da = a2 - a1
    let db = b2 - b1
    let denom = da.x * db.y - da.y * db.x  // 2D cross product
    if denom.abs() < 1e-20:
        return false  // parallel or degenerate
    let t = ((b1.x - a1.x) * db.y - (b1.y - a1.y) * db.x) / denom
    let u = ((b1.x - a1.x) * da.y - (b1.y - a1.y) * da.x) / denom
    t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0  // MuJoCo uses non-strict (>=, <=)
```

##### 4.9 Scope Boundary: What This Spec Does NOT Cover

- **`ActuatorTransmission::Site` stubs** — these are a separate feature (site-
  transmission actuators, spec #5). Spatial tendons use `ActuatorTransmission::Tendon`,
  not `Site`. Site-transmission actuators are orthogonal and have their own spec.
- **Pulley systems** — `WrapType::Pulley` divisor scaling is included in the path
  algorithm (4.3), but compound pulley physics (capstan friction, pulley inertia)
  from `sim-tendon/src/pulley.rs` are out of scope.
Tracked in [future_work_10d.md](./future_work_10d.md) §DT-30.
- **Tendon equality constraints** — implemented in §37 (`extract_tendon_equality_jacobian()`).
- **Wrapping function derivatives** — MuJoCo does not compute derivatives of the
  wrapping function (∂tangent_points/∂q). The tangent points are treated as fixed
  on the geom body for Jacobian purposes. This is an approximation that works
  well in practice. We follow the same approach.
- **`springlength` deadband semantics** — MuJoCo's `springlength` attribute is a
  `real(2)` pair supporting spring deadband (zero force between `low` and `high`).
  Our model currently stores `tendon_lengthspring` as a single `f64`. Upgrading to
  the pair representation and parsing `springlength` from MJCF is a follow-up.
- **`WrapType::Joint`** — MuJoCo supports `<joint>` elements inside spatial tendons
  (`mjWRAP_JOINT`), which create fixed-tendon-like contributions within a spatial
  tendon path. This wrap type is uncommon and not implemented. The parser rejects
  `<joint>` inside `<spatial>` at parse time (§4.1), and validation rule 10 (§4.2)
  provides defense-in-depth.
Tracked in [future_work_10d.md](./future_work_10d.md) §DT-31.
- ~~**`wrap_inside` code path (sidesite inside wrapping geometry)**~~ ✅ — Implemented
  in §39 (`future_work_10.md`). The `wrap_inside` Newton solver handles sidesites
  inside wrapping geometry for both spheres and cylinders. Validation rule 9 is retired.
- ~~**Tendon visualization data (`wrap_xpos`, `wrap_obj`)**~~ ✅ — Implemented
  in §40b (`future_work_10.md`). `Data` now stores `wrap_xpos`, `wrap_obj`,
  `ten_wrapadr`, and `ten_wrapnum`, populated during `mj_fwd_tendon_spatial()`
  with path points for all segment types (straight, wrapped, pulley).
- **`solref_limit`/`solimp_limit` MJCF attributes** — MuJoCo's `<spatial>` and
  `<fixed>` tendon elements accept per-tendon constraint solver parameters. These
  are not parsed (pre-existing gap shared with fixed tendons). Model-default
  solver params (`tendon_solref`, `tendon_solimp`) are used for all tendon limits.
Tracked in [future_work_10d.md](./future_work_10d.md) §DT-32.
- **Tendon `margin` attribute** — MuJoCo tendons have a `margin` attribute that
  creates an activation distance for limit constraints (constraint activates when
  `|length - limit| < margin`). Not implemented — limits activate at the exact
  boundary. This is a pre-existing gap shared with fixed tendons.
Tracked in [future_work_10d.md](./future_work_10d.md) §DT-33.
- **Pre-existing `add_body_jacobian` free joint bug** — The existing
  ~~`add_body_jacobian` closure uses world-frame unit vectors for free
  joint angular DOFs instead of body-frame `R*e_i`.~~ **FIXED** in DT-75 —
  all three affected locations (`compute_contact_jacobian`,
  `compute_flex_contact_jacobian`, `add_angular_jacobian`) now use body-frame
  `R*e_i` and `xpos`-based lever arm. See [future_work_10j.md](./future_work_10j.md) §DT-75.

#### Acceptance Criteria

All tests compare against MuJoCo reference values. Tolerance: `1e-6` for lengths,
`1e-5` for forces (matching existing conformance test standards).

| # | Criterion | Test Method |
|---|-----------|-------------|
| 1 | **Straight-line spatial tendon** — two sites, no wrapping geom. Length equals Euclidean distance between site world positions. | Unit test: 2-body chain with hinge joint, site on each body. Compare `ten_length` against hand-computed `\|\|site_xpos[1] - site_xpos[0]\|\|` at several joint angles. |
| 2 | **Multi-site spatial tendon** — 3+ sites, no wrapping. Length equals sum of consecutive segment distances. | Unit test: 3-body chain, site per body. Vary joint angles, compare total path length. |
| 3 | **Sphere wrapping** — tendon routed `site → geom(sphere) → site`. When straight path intersects sphere, length includes geodesic arc. When clear, straight-line path used. | Unit test with known geometry. Verify wrapping/non-wrapping transition. Cross-check against `sim-tendon::SphereWrap::compute_wrap()`. |
| 4 | **Cylinder wrapping** — tendon routed `site → geom(cylinder) → site`. Helical path around cylinder. | Unit test with known geometry. Cross-check against `sim-tendon::CylinderWrap::compute_wrap()`. |
| 5 | **Jacobian correctness** — `ten_J[t]` is the gradient of tendon length w.r.t. joint positions. | Finite-difference test: for each DOF `i`, perturb `qpos` by `±ε` (ε=1e-7), recompute `ten_length`, verify `(L(q+ε) - L(q-ε)) / 2ε ≈ ten_J[i]` within `1e-5`. Run for straight and wrapped tendons. **Note:** At the wrapping transition boundary (where the tendon switches between wrapping and non-wrapping), the Jacobian is discontinuous because MuJoCo does not compute wrapping derivatives. Finite-difference tests must use configurations well inside the wrapping or non-wrapping regime, not at the transition. |
| 6 | **Velocity correctness** — `ten_velocity[t] = ten_J[t] · qvel`. | Set known `qvel`, verify `ten_velocity` matches `J · qvel`. Also verify against finite-difference of `ten_length` across two timesteps. |
| 7 | **Passive force transmission** — spring/damper tendon forces produce correct generalized forces via `J^T`. | Unit test: spatial tendon with `stiffness > 0`. After `mj_fwd_passive()`, verify `qfrc_passive[dof] == Σ_t ten_J[t][dof] * ten_force[t]` (summed over spatial tendons). |
| 8 | **Tendon limit forces** — length limit violations produce correct constraint forces via `J^T`. | Unit test: set `tendon_limited = true`, drive tendon past limit, verify `qfrc_constraint` direction and magnitude match `J^T * penalty_force`. |
| 9 | **Actuator through spatial tendon** — `ActuatorTransmission::Tendon` with a spatial tendon target. Actuator force transmitted correctly. | Unit test: motor actuator on spatial tendon. After `mj_fwd_actuation()`, verify `qfrc_actuator` includes `gear * ten_J[tid]^T * actuator_force[i]`. |
| 10 | **Pulley divisor** — path segments after a `<pulley divisor="D">` element contribute `length/D` and `J/D`. Divisor persists until next pulley or end of tendon. | Unit test: tendon with pulley element. Verify length and Jacobian scaling. |
| 11 | **Wrapping transition** — tendon smoothly transitions between wrapping and non-wrapping as configuration changes. No length discontinuity at the transition. | Unit test: sweep joint angle through the wrapping transition. Verify `ten_length` is continuous (no jump > 1e-8 between adjacent angle steps of 0.001 rad). |
| 12 | **Model build: `tendon_length0`** — computed at `qpos0`, used as default `lengthspring`. | Unit test: build model with spatial tendon, verify `tendon_length0` matches `ten_length` after FK at `qpos0`. |
| 13 | **Model build: `acc0` for muscle on spatial tendon** — `compute_muscle_params()` uses configuration-dependent Jacobian from `ten_J`, not wrap-array shortcut. | Unit test: muscle actuator on spatial tendon. Verify `actuator_acc0` is non-zero and physically reasonable. |
| 14 | **Sidesite disambiguation** — wrapping with a `sidesite` attribute selects the correct wrapping direction. Covers three sub-cases: (a) sphere sidesite (Model E), (b) cylinder sidesite (Model I), (c) sidesite-forced wrapping when straight path clears the geometry (Model J). | Unit test: sphere wrapping with sidesite on each side. Verify the two cases produce different tangent points and arc lengths. Verify the selected path passes on the sidesite's side of the geometry. Test cylinder sidesite separately (XY-projected direction code path). Test sidesite-forced wrapping: verify wrapping occurs when `closest.norm() > radius` but sidesite is on the opposite side. |
| 15 | **Degenerate wrapping plane** — sphere wrapping when p1, center, p2 are collinear does not produce NaN. An arbitrary wrapping plane is chosen. The crossed tangent pairing places both tangent points on the same side of the sphere, so the arc angle is `acos(7/9) ≈ 0.6797 rad` (not π) for the symmetric d=3r case. | Unit test: place sites on opposite sides of a sphere along a line through the center (Model H). Verify `ten_length` is finite and equals `2×√(d²−r²) + r×acos(7/9)` where d=0.3, r=0.1. Expected: `0.63365281` (**confirmed against MuJoCo 3.4.0**). MuJoCo's tangent points at qpos=0 are both on the same side of the sphere (y=z≈0.067), confirming crossed tangent pairing with arc angle `acos(7/9) ≈ 0.6797`. |
| 16 | **MuJoCo conformance** — end-to-end test with representative MJCF models. Compare `ten_length`, `ten_velocity`, `ten_force` against MuJoCo output at multiple timesteps. | Conformance test: load each test model, step 100 frames, compare state vectors against MuJoCo reference data. |
| 17 | **Parser/validation rejection** — invalid spatial tendon MJCF is rejected with clear errors. Covers: fewer than 2 sites, path not starting/ending with Site, Geom not followed by Site (includes consecutive Geoms and Geom-Pulley sequences), Pulley immediately followed by Geom, unsupported geom type (e.g., box), zero-radius wrapping geom, zero/negative pulley divisor, unresolved site/geom/sidesite names, ~~sidesite inside wrapping geometry~~ (no longer rejected — handled by `wrap_inside` per §39), `<joint>` element inside spatial tendon. | Unit tests: for each invalid pattern, parse the MJCF and assert that the appropriate error is returned. |
| 18 | **Free-joint angular Jacobian** — `accumulate_point_jacobian` uses body-frame axes (`R*e_i`) for free joint angular DOFs, not world-frame unit vectors. | Unit test (Model K): free body rotated 45° about Y with spatial tendon. Attach site at `pos="0.2 0.1 0"` — the Y-offset is critical for discrimination (without it, DOFs 3 and 5 are zero in both conventions). Verify `ten_J` angular columns (DOFs 3-5) via finite difference. MuJoCo reference: `J = [0.989, 0.087, -0.122, +0.061, -0.122, -0.061]`. The test must fail if world-frame unit vectors are used: world-frame would give `J[3]=0.0, J[5]=-0.087` vs correct `J[3]=+0.061, J[5]=-0.061`. |
| 19 | **Mixed straight + wrapping segments** — spatial tendon with both site-site (straight) and site-geom-site (wrapped) segments in sequence. | Unit test (Model L): 4-site tendon with a wrapping geom between sites 2 and 3 but not between sites 1 and 2. Verify total length = straight_segment_1 + wrapped_segment_2. Verify Jacobian via finite difference covers all DOFs across both segment types. |

#### Test MJCF Models

**Model A — Straight-line spatial tendon (no wrapping):**

MuJoCo reference (3.4.0): `ten_length = 0.70000000` at qpos=0.

```xml
<mujoco>
  <worldbody>
    <body name="upper" pos="0 0 1">
      <joint name="shoulder" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5"/>
      <site name="s1" pos="0 0 0"/>
      <body name="lower" pos="0 0 -0.5">
        <joint name="elbow" type="hinge" axis="0 1 0"/>
        <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.4"/>
        <site name="s2" pos="0 0 -0.2"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t1" stiffness="100" damping="10">
      <site site="s1"/>
      <site site="s2"/>
    </spatial>
  </tendon>
  <actuator>
    <motor name="m1" tendon="t1" gear="1"/>
  </actuator>
  <sensor>
    <tendonpos name="tpos" tendon="t1"/>
    <tendonvel name="tvel" tendon="t1"/>
  </sensor>
</mujoco>
```

**Model B — Sphere wrapping:**

MuJoCo reference (3.4.0): `ten_length = 0.60843263` at qpos=0.

Sites are offset in Y (`y=0.05`) so the straight-line path intersects the sphere
(closest approach = 0.05 < radius = 0.1) without being collinear through the
center at qpos=0. This tests non-degenerate sphere wrapping (criterion 3).

```xml
<mujoco>
  <worldbody>
    <body name="b1" pos="-0.3 0.05 0">
      <joint name="j1" type="hinge" axis="0 0 1"/>
      <geom type="sphere" size="0.02"/>
      <site name="origin" pos="0 0 0"/>
    </body>
    <body name="wrap_body" pos="0 0 0">
      <geom name="wrap_sphere" type="sphere" size="0.1"/>
    </body>
    <body name="b2" pos="0.3 0.05 0">
      <joint name="j2" type="hinge" axis="0 0 1"/>
      <geom type="sphere" size="0.02"/>
      <site name="insertion" pos="0 0 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_wrap" stiffness="200">
      <site site="origin"/>
      <geom geom="wrap_sphere"/>
      <site site="insertion"/>
    </spatial>
  </tendon>
</mujoco>
```

**Model C — Cylinder wrapping:**

MuJoCo reference (3.4.0): `ten_length = 0.74507207` at qpos=0.

Sites have non-zero Y coordinates so the XY projection does not pass through
the cylinder axis, producing a non-degenerate arc angle at qpos=0. Slide
joints move along X, varying the wrap angle as sites move.

```xml
<mujoco>
  <worldbody>
    <body name="b1" pos="-0.3 0.1 0.2">
      <joint name="j1" type="slide" axis="1 0 0"/>
      <site name="s1" pos="0 0 0"/>
    </body>
    <body name="cylinder_body" pos="0 0 0">
      <geom name="wrap_cyl" type="cylinder" size="0.08 0.3"/>
    </body>
    <body name="b2" pos="0.3 -0.05 -0.2">
      <joint name="j2" type="slide" axis="1 0 0"/>
      <site name="s2" pos="0 0 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_cyl">
      <site site="s1"/>
      <geom geom="wrap_cyl"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
```

**Model D — Pulley with mechanical advantage:**

MuJoCo reference (3.4.0): `ten_length = 0.75000000`, `ten_J = [1.0, -0.5]`.

MuJoCo pulley semantics: a pulley element divides the tendon into "branches."
Segments before the pulley use divisor=1, segments after use divisor=D.
Each branch must have at least two sites to form a segment.

Expected length: `||s1-s2||/1 + ||s3-s4||/2`.

```xml
<mujoco>
  <worldbody>
    <body name="b1" pos="0 0 0.5">
      <joint name="j1" type="slide" axis="0 0 1"/>
      <site name="s1" pos="0 0 0"/>
    </body>
    <body name="b2" pos="0 0 0">
      <site name="s2" pos="0 0 0"/>
      <site name="s3" pos="0 0 0"/>
    </body>
    <body name="b3" pos="0 0 -0.5">
      <joint name="j2" type="slide" axis="0 0 1"/>
      <site name="s4" pos="0 0 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_pulley" stiffness="100">
      <site site="s1"/>
      <site site="s2"/>
      <pulley divisor="2"/>
      <site site="s3"/>
      <site site="s4"/>
    </spatial>
  </tendon>
</mujoco>
```

**Model E — Sphere wrapping with sidesite:**

MuJoCo reference (3.4.0): `ten_length = 0.64131210` at qpos=0.

Exercises dual-candidate sidesite selection. The asymmetric Y-placement
(origin at y=0.15, insertion at y=-0.05) breaks X-symmetry so the two
candidate wrapping paths have clearly different arc lengths. The sidesite
at `(0, 0.3, 0)` forces the tendon to wrap around the +Y side of the sphere,
selecting the longer arc (which passes near the sidesite) over the shorter one.

Both sites' straight-line path intersects the sphere (closest approach to the
origin < radius = 0.1), ensuring wrapping occurs at qpos=0.

```xml
<mujoco>
  <worldbody>
    <body name="b1" pos="-0.3 0.15 0">
      <joint name="j1" type="hinge" axis="0 0 1"/>
      <geom type="sphere" size="0.02"/>
      <site name="origin" pos="0 0 0"/>
    </body>
    <body name="wrap_body" pos="0 0 0">
      <geom name="wrap_sphere" type="sphere" size="0.1"/>
      <site name="side_hint" pos="0 0.3 0"/>
    </body>
    <body name="b2" pos="0.3 -0.05 0">
      <joint name="j2" type="hinge" axis="0 0 1"/>
      <geom type="sphere" size="0.02"/>
      <site name="insertion" pos="0 0 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_side">
      <site site="origin"/>
      <geom geom="wrap_sphere" sidesite="side_hint"/>
      <site site="insertion"/>
    </spatial>
  </tendon>
</mujoco>
```

**Model F — Multi-site spatial tendon (3+ sites, no wrapping):**

MuJoCo reference (3.4.0): `ten_length = 1.05000000` at qpos=0
(= 0.70 + 0.35, sum of segment distances).

Three-body chain with a site on each body and a single spatial tendon
connecting all three. Tests criterion 2 (multi-site length = sum of segments).

```xml
<mujoco>
  <worldbody>
    <body name="b1" pos="0 0 1">
      <joint name="j1" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5"/>
      <site name="s1" pos="0 0 0"/>
      <body name="b2" pos="0 0 -0.5">
        <joint name="j2" type="hinge" axis="0 1 0"/>
        <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.4"/>
        <site name="s2" pos="0 0 -0.2"/>
        <body name="b3" pos="0 0 -0.4">
          <joint name="j3" type="hinge" axis="0 1 0"/>
          <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.3"/>
          <site name="s3" pos="0 0 -0.15"/>
        </body>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_multi" stiffness="100" damping="10">
      <site site="s1"/>
      <site site="s2"/>
      <site site="s3"/>
    </spatial>
  </tendon>
</mujoco>
```

**Model G — Tendon limits + muscle actuator:**

MuJoCo reference (3.4.0): `ten_length = 0.70000000` at qpos=0. Note: the
muscle actuator requires explicit `lengthrange="0.35 0.9"` in MJCF — MuJoCo's
automatic estimation fails for this 2-site straight spatial tendon because
`compute_muscle_params` finds identical min/max lengths.

Exercises tendon limit constraints (criterion 8) and muscle `acc0` computation
(criterion 13). The tendon has `limited="true"` with `range="0.35 0.9"`. At
qpos=0, tendon length is 0.7 (within range). The minimum achievable distance
with elbow at pi is 0.3, which violates the lower limit (0.35). The upper
limit (0.9) can be exceeded with shoulder rotation. The muscle actuator on
the limited tendon gets `actuator_lengthrange` from the tendon's range,
which tests `compute_muscle_params()` with the spatial-tendon Jacobian fix.

```xml
<mujoco>
  <worldbody>
    <body name="b1" pos="0 0 1">
      <joint name="shoulder" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5"/>
      <site name="s1" pos="0 0 0"/>
      <body name="b2" pos="0 0 -0.5">
        <joint name="elbow" type="hinge" axis="0 1 0"/>
        <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.4"/>
        <site name="s2" pos="0 0 -0.2"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_lim" stiffness="100" damping="10"
             limited="true" range="0.35 0.9">
      <site site="s1"/>
      <site site="s2"/>
    </spatial>
  </tendon>
  <actuator>
    <muscle name="muscle1" tendon="t_lim" force="100" lengthrange="0.35 0.9"/>
  </actuator>
</mujoco>
```

**Model H — Collinear degenerate sphere wrapping:**

Sites are placed on the X-axis, directly opposite each other through the sphere
center. This forces the collinear fallback in `sphere_wrap` (step 3 — arbitrary
perpendicular via least-aligned cardinal axis). The dual-candidate selection
(step 5) evaluates both ±normal solutions; for the collinear case they produce
the same arc by symmetry, so the test exercises the arbitrary-plane fallback
path without sidesite involvement.

Expected length at qpos=0: **`0.63365281`** (**confirmed against MuJoCo 3.4.0**).
Formula: `2×√(d²−r²) + r×acos(7/9)` where d=0.3, r=0.1.

MuJoCo's tangent points: `t1 = (-0.0333, 0.0667, 0.0667)`,
`t2 = (0.0333, 0.0667, 0.0667)` — both on the same side of the sphere,
confirming crossed tangent pairing. The arbitrary wrapping plane chosen by
MuJoCo includes a Z component (not pure XY), but the arc angle is
plane-invariant: `θ = acos(t1·t2/r²) = acos(7/9) ≈ 0.6797 rad`.

Note: the sim-tendon reference implementation (`wrapping.rs`) produces
`θ = π` for this case because it uses independent `find_perpendicular`
per point (which places tangent points on opposite sides). This is a
known algorithmic divergence — the pipeline must implement the crossed
pairing algorithm to match MuJoCo.

Algebraic derivation:
- `cos_θ = r/d = 1/3`, `sin_θ = 2√2/3`
- t1 = r×(u₁×cos_θ + v₁×sin_θ), t2 = r×(u₂×cos_θ + v₂×sin_θ)
- Crossed pairing: v₁ = n×u₁, v₂ = (−n)×u₂ — both perpendiculars point the
  same way, so t1 and t2 end up on the same side of the sphere
- `cos(θ) = t1·t2 / (r²) = 7/9` → `θ = acos(7/9)`

```xml
<mujoco>
  <worldbody>
    <body name="b1" pos="-0.3 0 0">
      <joint name="j1" type="slide" axis="0 1 0"/>
      <geom type="sphere" size="0.02"/>
      <site name="s1" pos="0 0 0"/>
    </body>
    <body name="wrap_body" pos="0 0 0">
      <geom name="wrap_sphere" type="sphere" size="0.1"/>
    </body>
    <body name="b2" pos="0.3 0 0">
      <joint name="j2" type="slide" axis="0 1 0"/>
      <geom type="sphere" size="0.02"/>
      <site name="s2" pos="0 0 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_collinear">
      <site site="s1"/>
      <geom geom="wrap_sphere"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
```

**Model I — Cylinder wrapping with sidesite:**

MuJoCo reference (3.4.0): `ten_length = 0.74507207` at qpos=0 (same geometry
as Model C; sidesite selects same wrapping direction as no-sidesite default).

Exercises cylinder sidesite disambiguation (the XY-projected sidesite direction
code path, which is distinct from the sphere sidesite path). Sites offset in Y
ensure wrapping occurs. The sidesite at `(0, 0.2, 0)` forces the tendon to
wrap around the +Y side of the cylinder. Slide joints along X allow varying
the wrap angle.

```xml
<mujoco>
  <worldbody>
    <body name="b1" pos="-0.3 0.1 0.2">
      <joint name="j1" type="slide" axis="1 0 0"/>
      <site name="s1" pos="0 0 0"/>
    </body>
    <body name="cylinder_body" pos="0 0 0">
      <geom name="wrap_cyl" type="cylinder" size="0.08 0.3"/>
      <site name="cyl_side" pos="0 0.2 0"/>
    </body>
    <body name="b2" pos="0.3 -0.05 -0.2">
      <joint name="j2" type="slide" axis="1 0 0"/>
      <site name="s2" pos="0 0 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_cyl_side">
      <site site="s1"/>
      <geom geom="wrap_cyl" sidesite="cyl_side"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
```

**Model J — Sidesite-forced wrapping (straight path clears sphere):**

MuJoCo reference (3.4.0): `ten_length = 0.86662782` at qpos=0 (wrapping
confirmed: `ten_wrapnum = 4`).

Sites are placed far from the X-axis (Y=0.2) so the straight-line path
**clears** the sphere (closest approach = 0.2 > radius = 0.1). The sidesite
at `(0, -0.3, 0)` is on the **opposite** side, forcing wrapping even though
the direct path would not intersect the sphere. This tests the sidesite-forced
wrapping code path (spec §4.7 step 2, lines 3022-3033).

```xml
<mujoco>
  <worldbody>
    <body name="b1" pos="-0.3 0.2 0">
      <joint name="j1" type="hinge" axis="0 0 1"/>
      <geom type="sphere" size="0.02"/>
      <site name="s1" pos="0 0 0"/>
    </body>
    <body name="wrap_body" pos="0 0 0">
      <geom name="wrap_sphere" type="sphere" size="0.1"/>
      <site name="side_hint" pos="0 -0.3 0"/>
    </body>
    <body name="b2" pos="0.3 0.2 0">
      <joint name="j2" type="hinge" axis="0 0 1"/>
      <geom type="sphere" size="0.02"/>
      <site name="s2" pos="0 0 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_forced">
      <site site="s1"/>
      <geom geom="wrap_sphere" sidesite="side_hint"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
```

**Model K — Free-joint spatial tendon:**

Exercises `accumulate_point_jacobian` with a free joint (6 DOFs: 3 translational
+ 3 rotational). The spatial tendon connects a site on the world body to a site
on the free-floating body. The body is initially rotated 45° about Y via
`euler="0 45 0"` to ensure the angular Jacobian uses body-frame axes (`R*e_i`),
not world-frame unit vectors. The attach site is at `pos="0.2 0.1 0"` — the
Y-offset is critical: without it (e.g., `pos="0.2 0 0"`), the lever arm `r`
lies in the XZ rotation plane, making angular DOFs 3 and 5 identically zero
in **both** body-frame and world-frame conventions. The Y=0.1 offset breaks
this symmetry so that world-frame axes produce `J[3]=0.0, J[5]=-0.087` while
body-frame axes produce `J[3]=+0.061, J[5]=-0.061` — a clear discriminator.

MuJoCo reference (MuJoCo 3.4.0): `ten_length = 1.15448807`,
`ten_J = [0.98868, 0.08662, -0.12250, +0.06125, -0.12250, -0.06125]`.

```xml
<mujoco>
  <worldbody>
    <site name="anchor" pos="0 0 0"/>
    <body name="floating" pos="1 0 0" euler="0 45 0">
      <freejoint name="free"/>
      <geom type="sphere" size="0.05"/>
      <site name="attach" pos="0.2 0.1 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_free" stiffness="100">
      <site site="anchor"/>
      <site site="attach"/>
    </spatial>
  </tendon>
</mujoco>
```

**Model L — Mixed straight + wrapping segments:**

A 3-body chain where the first segment (s1→s2) is a straight site-site path and
the second segment (s2→geom→s3) wraps around a sphere. This exercises the
pairwise loop's `j` advancement logic across different segment types within
a single tendon: first `j += 1` (site-site), then `j += 2` (site-geom-site).
Sites s1 and s2 are offset in Y (0.3 and 0.15) to keep the first segment well
clear of the wrapping sphere. Site s3 has Y=0 so the s2→s3 line passes through
the sphere cross-section at qpos=0 (closest approach ≈ 0.070 < radius 0.08),
ensuring wrapping is active at the home configuration.

```xml
<mujoco>
  <worldbody>
    <body name="b1" pos="0 0 1">
      <joint name="j1" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.5"/>
      <site name="s1" pos="0 0.3 0"/>
      <body name="b2" pos="0 0 -0.5">
        <joint name="j2" type="hinge" axis="0 1 0"/>
        <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.4"/>
        <site name="s2" pos="0 0.15 0"/>
        <body name="wrap_body" pos="0 0 -0.2">
          <geom name="wrap_sphere" type="sphere" size="0.08"/>
        </body>
        <body name="b3" pos="0 0 -0.4">
          <joint name="j3" type="hinge" axis="0 1 0"/>
          <geom type="capsule" size="0.02" fromto="0 0 0 0 0 -0.3"/>
          <site name="s3" pos="0 0 0"/>
        </body>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_mixed" stiffness="100">
      <site site="s1"/>
      <site site="s2"/>
      <geom geom="wrap_sphere"/>
      <site site="s3"/>
    </spatial>
  </tendon>
</mujoco>
```

#### Implementation Order

**Atomicity constraint:** Steps 3 and 4 **must ship in the same commit**.
`mj_fwd_tendon_spatial()` populates non-zero `ten_length`/`ten_J`, which the
three force-mapping sites (`mj_fwd_passive`, `mj_fwd_constraint`,
`mj_fwd_actuation`) consume. These sites currently use the fixed-tendon
wrap-array pattern where `wrap_objid[w]` is a DOF address. For spatial tendons,
`wrap_objid` stores site/geom IDs, not DOF addresses. Deploying spatial
kinematics (step 3) without the force-mapping fix (step 4) would cause these
sites to write forces to garbage DOF indices — producing silent data corruption,
not just wrong results.

1. ~~**MJCF parser + types** — `SpatialPathElement` enum, ordered path parsing,
   `sidesite` and `divisor` attribute parsing. Update `validation.rs`.
   Write parser/validation tests (test 17, parser-side cases).~~ **DONE**
2. ~~**Model builder** — `process_tendons()` spatial path → wrap arrays with
   `wrap_sidesite`. Validation rules (including rule 9: sidesite outside
   geometry). Remove spatial tendon warning.
   Write builder validation tests (test 17, builder-side cases).~~ **DONE**
3. ~~**`mj_fwd_tendon_spatial()` + `accumulate_point_jacobian()`** — straight-line
   path only (sites, no wrapping geoms). `accumulate_point_jacobian` is a standalone
   helper testable in isolation against `compute_contact_jacobian`'s
   `add_body_jacobian`. Write acceptance tests 1, 2, 5, 6, 18.
   **Note:** `accumulate_point_jacobian` uses body-frame axes (`R*e_i`) for free
   joint angular DOFs, matching MuJoCo's `cdof` convention. The existing
   `add_body_jacobian` (line 7923) has a pre-existing bug using world-frame unit
   vectors instead — do NOT match that bug. See the Free joint case in the
   pseudocode above.~~ **DONE**
4. ~~**Force transmission fix** — `apply_tendon_force()` helper. Update
   `mj_fwd_passive()`, `mj_fwd_constraint()`, and `mj_fwd_actuation()`.
   **Must be in the same commit as step 3** (see atomicity constraint above).
   Write tests 7, 8, 9.~~ **DONE**
5. ~~**Model-build fixes** — `tendon_length0` computation for spatial tendons,
   `compute_muscle_params()` spatial tendon Jacobian, `actuator_lengthrange`
   warning for unlimited spatial tendons. Write tests 12, 13.~~ **DONE**
6. ~~**Sphere wrapping** — `sphere_wrap()` + `WrapResult` type + dual-candidate
   sidesite selection + sidesite-forced wrapping (when straight path clears
   the geometry but sidesite prevents early exit) + collinear degenerate
   fallback + self-intersection rejection (`segments_intersect_2d`) +
   integration into `mj_fwd_tendon_spatial()`. Write tests 3, 11, 14
   (sub-cases a and c: Models E, J), 15 (Model H), 19 (Model L: mixed
   straight + wrapping segments).~~ **DONE**
7. ~~**Cylinder wrapping** — `cylinder_wrap()` + Z-interpolation + integration.
   Write test 4. Write test 14 sub-case b (Model I: cylinder sidesite).~~ **DONE**
8. ~~**Pulley divisor** — already in the pairwise loop structure; verify with
   dedicated test. Write test 10.~~ **DONE**
9. ~~**MuJoCo conformance test** — end-to-end comparison. Test 16.~~ **DONE**

#### Files

| File | Action | Changes |
|------|--------|---------|
| `sim/L0/mjcf/src/types.rs` | modify | Add `SpatialPathElement` enum. Replace `sites`+`wrapping_geoms` fields on `MjcfTendon` with `path_elements: Vec<SpatialPathElement>`. |
| `sim/L0/mjcf/src/parser.rs` | modify | `parse_tendon()`: add `b"pulley"` match arm (currently missing — `<pulley>` elements are silently ignored by the `_ => {}` default). Push `SpatialPathElement` variants in document order. Parse `sidesite` attr on `<geom>` children, `divisor` attr on new `<pulley>` children. |
| `sim/L0/mjcf/src/builder/` | modify | `process_tendons()`: iterate `path_elements`, populate `wrap_sidesite`, add geom type validation. Remove spatial tendon warning. Call `compute_spatial_tendon_length0()` before `compute_muscle_params()` in `build()`. Skip `actuator_lengthrange` estimation for unlimited spatial-tendon actuators (log warning; see 4.6B). |
| `sim/L0/mjcf/src/validation.rs` | modify | Update spatial tendon validation: replace `tendon.sites.len() < 2` check with `path_elements` counting of `Site` variants. Update site/geom/sidesite name-existence checks to iterate `path_elements`. Add validation for Geom-must-be-followed-by-Site rule, start/end-with-Site rule, geom `size[0] > 0` rule, Pulley-Geom adjacency rule, pulley `divisor > 0` rule, per-branch minimum 2 sites (rule 8, warning), ~~sidesite-outside-geometry rule (rule 9)~~ (retired per §39 — `wrap_inside` now handles this at runtime), and `<joint>` inside spatial tendon rejection (rule 10). |
| `sim/L0/mjcf/src/defaults.rs` | verify | `apply_to_tendon()` does not reference `sites`/`wrapping_geoms` — only sets scalar fields (stiffness, damping, etc.). No changes needed, but verify after field removal. |
| `sim/L0/core/src/constraint/mod.rs` | modify | Add `Model::wrap_sidesite` field. Add `mj_fwd_tendon_spatial()`, `accumulate_point_jacobian()`, `sphere_wrap()` + `compute_tangent_pair()` + `sphere_tangent_point()`, `cylinder_wrap()` + `compute_tangent_pair_2d()` + `circle_tangent_2d()` + `directional_wrap_angle()` + `segments_intersect_2d()`, `apply_tendon_force()`, `compute_spatial_tendon_length0()`. Fix force mapping in `mj_fwd_passive()`, `mj_fwd_constraint()`, `mj_fwd_actuation()`. Fix `compute_muscle_params()` for spatial tendons. |
| `sim/L0/tendon/src/wrapping.rs` | reference | Use `SphereWrap::compute_wrap()` and `CylinderWrap::compute_wrap()` as **rough sanity checks** in tests. These implementations have significant algorithmic divergences from MuJoCo (no sidesite, no dual-candidate selection, no self-intersection check, angle-based Z-interpolation instead of path-length-proportional, 1% margin hack on sphere clearance). They cannot serve as exact verification oracles — use MuJoCo reference data for conformance testing. |
| `sim/conformance-tests/` | create | Acceptance tests 1-19 and MJCF test models A-L. Parser/validation rejection tests (test 17). Free-joint Jacobian test (test 18, Model K). Mixed straight + wrapping segments test (test 19, Model L). MuJoCo conformance tests (test 16). |

---

### 5. Site-Transmission Actuators
**Status:** ✅ Complete (criteria 1–22 implemented; criterion 22 cross-validated against MuJoCo 3.5.0) | **Effort:** L | **Prerequisites:** #4 (complete)

#### Objective
Implement MuJoCo-conformant site-based actuator transmission. Site transmissions
apply Cartesian forces/torques at a site frame, enabling jet/propeller modelling
(without refsite) and task-space end-effector control (with refsite). This
requires upgrading the gear representation from scalar to 6D and adding a refsite
slot to the transmission target.

#### MuJoCo Reference Semantics
MuJoCo site transmissions have two modes, selected by the presence or absence of
the `refsite` attribute. In both modes, the moment vector and velocity are related
by the same universal formula used for all transmission types:

```
velocity_i  = moment_i · qvel                              (always)
qfrc_actuator += moment_i * force_i                        (always)
```

**Mode A — No refsite (jets/propellers):**
```
length_i(q) = 0                            (constant zero)
moment_i    = J_site^T @ R_site @ gear     (nv-vector)
```
The 6D gear vector `[tx ty tz rx ry rz]` is expressed in the site's local frame.
`R_site` rotates it to world frame. `J_site` is the 6×nv stacked
translational/rotational Jacobian of the site. Force/torque is fixed to the site
frame and moves with the body — useful for thrusters and propellers.

Note: `length` is formally zero, but `velocity = moment · qvel` is generally
**nonzero** — it captures the instantaneous power conjugate of the actuator
wrench. This is physically correct and required for velocity-feedback actuators
(`dyntype=integrator`) and damping terms to function on site transmissions.

**Mode B — With refsite (Cartesian control):**
```
length_i(q) = (R_ref^T @ (p_site - p_ref)) · gear[0:3]     (translational)
            + subquat(q_site, q_ref)       · gear[3:6]      (rotational)

moment_i    = (J_site - J_ref)^T @ R_ref @ gear             (nv-vector)
              with common-ancestor DOF columns zeroed
```
Where:
- `p_site`, `p_ref` are world-frame site positions
- `R_ref` is the refsite rotation matrix (3×3)
- `q_site`, `q_ref` are world-frame site quaternions
- `subquat(a, b)` is the quaternion difference expressed as axis-angle (expmap)
- `J_site`, `J_ref` are 6×nv site Jacobians (3×nv translational + 3×nv rotational)
- Common-ancestor DOFs: DOFs on the shared kinematic chain between site and refsite
  bodies are zeroed in the difference Jacobian to avoid double-counting

The moment vector (nv-dimensional) maps scalar actuator force to generalized
forces: `qfrc_actuator += moment * force`.

#### Implementation Summary
All five pipeline stubs have been resolved and all data-model changes applied.
The implementation spans two commits: `828df4d` (core implementation) and
`24b8a88` (A+ test coverage + conformance placeholder). Key additions:

- `mj_jac_site()` — full 3×nv site Jacobian (translational + rotational)
- `mj_transmission_site()` — pipeline step for Site length + moment
- `subquat()` — quaternion difference as axis-angle 3-vector
- Common-ancestor DOF zeroing for Mode B difference Jacobians
- `ActuatorFrc` sensor bug fix (reads `actuator_force[objid]`)
- Refsite warning for non-Site transmissions (Joint/Tendon branches)

**Test coverage:** 29 integration tests + 9 unit tests (7 subquat + 2 jac_site).
Criteria 1–22 all have passing tests. Criterion 22 (MuJoCo conformance ≤ 1e-8)
verified with 7 sub-tests against MuJoCo 3.5.0 — see `MUJOCO_CONFORMANCE.md` §5.

#### Data-Model Changes

**1. Gear: scalar → 6D**

| Layer | Current | Target |
|-------|---------|--------|
| `MjcfActuator.gear` (types.rs:1814) | `f64` | `[f64; 6]` |
| `MjcfActuatorDefaults.gear` (types.rs:492) | `Option<f64>` | `Option<[f64; 6]>` |
| MJCF parser (parser.rs:1240) | `parse_float_attr` → single f64 | `parse_float_array`, pad to 6 with zeros |
| `Model.actuator_gear` (types/model.rs) | `Vec<f64>` | `Vec<[f64; 6]>` |
| model_builder (builder/) | `push(actuator.gear)` | `push(actuator.gear)` (type changes) |
| `MjcfActuator::default()` (types.rs:1879) | `gear: 1.0` | `gear: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]` |

MuJoCo default: `gear="1 0 0 0 0 0"`. When the user writes `gear="50"`, parse as
`[50, 0, 0, 0, 0, 0]`. Parse up to 6 floats; ignore extras beyond 6; pad with
zeros if fewer than 6. Existing Joint and Tendon transmissions only use `gear[0]`
(the scalar they use today), so all current behaviour is preserved.

**2. Refsite: add optional second transmission target**

| Layer | Current | Target |
|-------|---------|--------|
| `MjcfActuator` (types.rs) | no `refsite` field | `pub refsite: Option<String>` |
| MJCF parser (parser.rs) | — | `get_attribute_opt(e, "refsite")` |
| `Model.actuator_trnid` (types/model.rs) | `Vec<usize>` (1 per actuator) | `Vec<[usize; 2]>` — `[site_id, refsite_id]` where `refsite_id = usize::MAX` means absent |
| model_builder | pushes single usize | pushes `[site_id, refsite_id_or_MAX]` |

All existing Joint/Tendon code reads `actuator_trnid[i]` — change to
`actuator_trnid[i][0]`. The `[1]` slot is `usize::MAX` for Joint and Tendon
transmissions regardless of MJCF content. This is a mechanical refactor.

Validation: if `refsite` is specified on a non-Site transmission (e.g.
`<general joint="j1" refsite="s1">`), ignore it and emit a warning. `refsite`
is only meaningful for `trntype=Site`.

**3. World-frame site quaternions**

The rotational length computation (Mode B) requires world-frame site quaternions.
`Data` currently has `site_xmat: Vec<Matrix3<f64>>` (world rotation matrix) but
no world-frame quaternion. `Model.site_quat` is in **body frame**, not world.

Add to `Data`:
```
pub site_xquat: Vec<UnitQuaternion<f64>>   // length nsite, world frame
```
Computed during FK alongside `site_xmat`, matching MuJoCo's approach:
```
site_xquat[s] = xquat[site_body[s]] * model.site_quat[s]
```

**4. Site Jacobian function: `mj_jac_site`**

New function in `jacobian.rs`:
```
fn mj_jac_site(
    model: &Model,
    data: &Data,
    site_id: usize,
) -> (DMatrix<f64>, DMatrix<f64>)   // (jac_trans 3×nv, jac_rot 3×nv)
```

Computes the full 3×nv translational and 3×nv rotational Jacobian for a site by
walking the kinematic chain from `model.site_body[site_id]` to root. For each
joint on the chain:

- `axis` = world-frame joint axis = `xquat[jnt_body] * jnt_axis[jid]`
- `joint_anchor` = `xpos[jnt_body] + xquat[jnt_body] * jnt_pos[jid]`
  (for Free joints: `joint_anchor = xpos[jnt_body]`, i.e. body origin)
- `r` = `site_xpos[site_id] - joint_anchor`

Per joint type:

- **Hinge:**
  `jac_trans.column_mut(dof) += cross(axis, r)` ;
  `jac_rot.column_mut(dof) += axis`
- **Slide:**
  `jac_trans.column_mut(dof) += axis`
  (no rotational contribution)
- **Ball** (k = 0, 1, 2):
  `let omega_k = R_body * e_k` ;
  `jac_trans.column_mut(dof+k) += cross(omega_k, r)` ;
  `jac_rot.column_mut(dof+k) += omega_k`
- **Free:**
  Translational DOFs (dof+0..dof+3):
    `jac_trans.column_mut(dof+k) += e_k` (identity columns)
    (no rotational contribution from translational DOFs)
  Rotational DOFs (dof+3..dof+6, k = 0, 1, 2):
    `let omega_k = R_body * e_k` ;
    `jac_trans.column_mut(dof+3+k) += cross(omega_k, r)` ;
    `jac_rot.column_mut(dof+3+k) += omega_k`

This function is analogous to MuJoCo's `mj_jacSite`. It builds on the pattern in
`accumulate_point_jacobian` but returns full 3-row matrices instead of projecting
onto a single direction.

**5. Actuator moment cache and `mj_transmission_site` pipeline step**

Add to `Data`:
```
pub actuator_moment: Vec<DVector<f64>>   // length nu, each DVector is nv
```

New function `mj_transmission_site(model, data)` computes `actuator_length` and
`actuator_moment` for **Site transmissions only** (velocity is deferred to
`mj_actuator_length` at step 7):

- **Site (Mode A):** moment from `mj_jac_site` + wrench projection ;
  `length = 0`
- **Site (Mode B):** moment from difference Jacobian + wrench projection ;
  `length` from position/quaternion differences

Joint and Tendon transmissions do **not** use `mj_transmission_site`. Their
length/velocity computation remains in `mj_actuator_length` exactly as today.
Their force application remains as direct dispatch (Joint → single DOF write,
Tendon → `apply_tendon_force`). This minimizes regression risk and preserves
the existing sparse/direct-write performance for Joint/Tendon paths.

The `actuator_moment` cache is populated for Site transmissions only. Joint and
Tendon transmissions do not need it — their force mapping is already efficient
via the existing per-type dispatch.

#### Pipeline Placement (implemented)

The `forward()` pipeline with site-transmission integration:

```
 1. mj_fwd_position()          — FK: site_xpos, site_xmat, site_xquat computed
 2. mj_transmission_site()     — NEW: sets actuator_length + actuator_moment
                                  for Site transmissions only (needs FK output)
 3. mj_collision()
 4. mj_sensor_pos()            — can now read actuator_length (stub 3 resolved)
 5. mj_energy_pos()
 6. mj_fwd_velocity()          — body/tendon velocities
 7. mj_actuator_length()       — UNCHANGED: Joint/Tendon length+velocity as today;
                                  Site case adds: velocity = moment · qvel
 8. mj_sensor_vel()
 9. mj_fwd_actuation()         — Joint/Tendon force dispatch unchanged;
                                  Site uses cached moment (stub 5)
10. mj_crba()                  — (unchanged, shown for completeness)
11. mj_rne()                   — (unchanged)
12. mj_energy_vel()            — (unchanged)
13. mj_fwd_passive()           — (unchanged)
14. mj_fwd_constraint()        — (unchanged)
15. mj_fwd_acceleration()      — (unchanged)
16. mj_sensor_acc()            — ActuatorFrc sensor fix reads actuator_force here
17. mj_sensor_postprocess()    — (unchanged)
```

`mj_transmission_site` must run after `mj_fwd_position` (needs site poses) and
before `mj_sensor_pos` (which reads `actuator_length`). This matches MuJoCo's
ordering where `mj_transmission` runs inside `mj_fwdPosition`, before
`mj_sensorPos`.

Site velocity (`actuator_velocity = moment · qvel`) is computed inside
`mj_actuator_length` (step 7, the existing Stub 4 location) rather than at
step 2, because `qvel` is semantically part of the velocity stage. While `qvel`
doesn't change during the position/velocity stages (it's an input), placing the
velocity write at step 7 keeps it adjacent to the Joint/Tendon velocity writes
and matches MuJoCo's pipeline ordering.

#### Stub Resolutions (all complete)

**Stub 4 — `mj_actuator_length` (length & velocity):** ✅

Site branch reads pre-computed length from `mj_transmission_site`, computes
velocity as `moment · qvel`. Joint/Tendon logic unchanged.

**Stub 5 — `mj_fwd_actuation` (force application):** ✅

Site branch applies force via cached moment vector (`qfrc += moment * force`).
Joint (O(1) direct write) and Tendon (`apply_tendon_force`) branches unchanged.

**Stub 3 — `mj_sensor_pos` (ActuatorPos sensor):** ✅

Site case reads `data.actuator_length[objid]`, which was already populated by
`mj_transmission_site` (step 2) before `mj_sensor_pos` (step 4).

**Stub 2 — `compute_muscle_params` (muscle acc0 Jacobian):** ✅

Uses `mj_jac_site` at build time. Mode A: wrench rotated by site frame.
Mode B: difference Jacobian with common-ancestor zeroing, wrench in refsite frame.

**Stub 1 — `compute_muscle_params` (length-range estimation):** ✅

No-op for site transmissions — explicit `lengthrange` required in MJCF for
muscle actuators with site transmission (matches MuJoCo behavior).
Tracked in [future_work_10j.md](./future_work_10j.md) §DT-77.

#### Bug Fix: `ActuatorFrc` Sensor (done)

The `ActuatorFrc` sensor in `mj_sensor_acc` previously read
`qfrc_actuator[dof_adr]` using `actuator_trnid` as a joint ID — a pre-existing
bug that only worked for Joint transmissions and produced incorrect values for
Tendon and Site transmissions.

Fixed to read `data.actuator_force[objid]` (transmission-independent scalar
actuator force, after gain/bias/clamp). This matches MuJoCo's semantics where
the `actuatorfrc` sensor reports the scalar actuator force, not the generalized
force at a specific DOF.

#### Common-Ancestor DOF Zeroing (Mode B only)
When site and refsite share part of their kinematic chain, the difference
Jacobian `J_site - J_ref` has non-zero entries for DOFs that move both sites
identically — these must be zeroed. Algorithm:

1. Let `b0 = site_body[sid]`, `b1 = site_body[refid]`
2. Walk `b0` and `b1` up via `body_parent` to find their lowest common ancestor
   body `bca`
3. All DOFs belonging to `bca` and its ancestors are "common" DOFs — collect
   via `body_jnt_adr[b]`/`body_jnt_num[b]` → `jnt_dof_adr[j]`/`jnt_type[j].nv()`
   for each body on the ancestor chain
4. Zero those columns in the difference Jacobian before projecting onto the
   wrench vector

Note: MuJoCo uses `dof_parentid` to walk the DOF tree directly at DOF
granularity. We walk the body tree via `body_parent` and collect DOFs per body.
This is equivalent when each body has at most one joint (the common case). For
multi-joint bodies (e.g. a body with both hinge and slide), both approaches zero
all DOFs on that body — the result is identical because the body-level walk
captures all joints on the ancestor chain. If a body has joints that belong to
different kinematic sub-chains (not possible in MuJoCo's tree topology), the
body-level walk is strictly more conservative (zeros more DOFs), which is safe.

#### Quaternion Subtraction (`subquat`)
MuJoCo's `mju_subQuat(res, qa, qb)` computes the orientation difference as an
axis-angle 3-vector satisfying `qb * quat(res) = qa`:
```
dq = conjugate(qb) * qa              // relative quaternion in qb's frame
axis = normalize(dq.xyz)             // unit rotation axis
sin_half = ||dq.xyz||                // before normalization
angle = 2 * atan2(sin_half, dq.w)   // full-range angle via atan2
if angle > π: angle -= 2π            // shortest path
res = axis * angle                   // axis-angle 3-vector
// small-angle limit: when sin_half ≈ 0, axis is undefined;
// atan2(0, 1) = 0 so res = 0-vector (correct)
```
Implemented as `subquat()` in `sensor/mod.rs` with 7 unit tests covering
identity, cardinal axes, 180°, shortest-path wrapping, relative rotation,
and small-angle guard. It is the rotational analogue of the translational
position difference `p_site - p_ref`.

#### Implementation Order (all complete)
1. ✅ **Data-model changes** — 6D gear, 2-slot trnid, refsite parsing,
   `Data.site_xquat`, `Data.actuator_moment`, `subquat` utility. Mechanical
   refactor of all existing Joint/Tendon code to use `gear[0]` and `trnid[i][0]`.
2. ✅ **`mj_jac_site`** — new function, unit-tested against `accumulate_point_jacobian`
   (single-direction projection must agree).
3. ✅ **`mj_transmission_site`** — new pipeline step computing `actuator_length` +
   `actuator_moment` for Site transmissions only. Wired into `forward()` after
   `mj_fwd_position`, before `mj_collision`.
4. ✅ **Stub 4** — Site velocity in `mj_actuator_length`: `moment · qvel`.
5. ✅ **Stub 5** — Site force application in `mj_fwd_actuation` via cached moment.
   Joint/Tendon force paths unchanged.
6. ✅ **Stub 3** — Site sensor read in `mj_sensor_pos` (reads `actuator_length`).
7. ✅ **Stub 2** — muscle acc0 (uses `mj_jac_site` at build time).
8. ✅ **Stub 1** — length-range (no-op for sites; explicit `lengthrange` required
   for muscle site actuators).
9. ✅ **`ActuatorFrc` sensor fix** — `mj_sensor_acc` reads
   `data.actuator_force[objid]` instead of `qfrc_actuator[dof_adr]`. Fixes
   pre-existing bug for Tendon transmissions.
10. ✅ **Validation** — site/refsite reference validation in `validation.rs`.
    Mutual exclusivity of transmission targets (joint/tendon/site/body).

#### Acceptance Criteria

All criteria 1–22 have passing integration tests in
`sim/L0/tests/integration/site_transmission.rs`. Criterion 22 cross-validated
against MuJoCo 3.5.0 with 7 sub-tests (modes A/B, translational/rotational/mixed
gear, free joint, position actuator).

**Correctness — Mode A (no refsite):**
1. `actuator_length[i] == 0.0` regardless of configuration.
2. `actuator_velocity[i] == actuator_moment[i] · qvel` — generally nonzero when
   the site is moving. Verified numerically: for a hinge joint at constant
   `qvel`, `actuator_velocity` equals the rate of change of the wrench's power
   direction.
3. A site actuator with `gear="0 0 1 0 0 0"` on a body with a hinge joint
   produces `qfrc_actuator[dof] = force * dot(cross(world_joint_axis, r), z_site)`
   where `r` is the lever arm from joint anchor to site position (world frame)
   and `z_site` is the site's local z-axis rotated to world frame
   (`site_xmat * [0,0,1]`).
4. Rotational gear component (`gear="0 0 0 1 0 0"`) produces a pure torque about
   the site's local x-axis, with correct Jacobian mapping to joint torques:
   `qfrc_actuator[dof] = force * dot(world_joint_axis, x_site)` for a hinge.
5. Mixed translational + rotational gear produces the correct superposition.

**Correctness — Mode B (with refsite):**
6. Translational length: for two sites separated by `[dx, dy, dz]` in the refsite
   frame, `actuator_length == gear[0]*dx + gear[1]*dy + gear[2]*dz`.
7. Rotational length: for a 90° rotation about the refsite's x-axis,
   `actuator_length` includes `gear[3] * π/2` (axis-angle magnitude).
8. Velocity: `actuator_velocity == actuator_moment · qvel` (verified numerically
   against finite-difference of length over a small dt).
9. Force application: `qfrc_actuator == actuator_moment * force` (verified by
   energy conservation: `force * velocity == qfrc_actuator · qvel`).
10. Common-ancestor zeroing: when site and refsite are on the same body,
    `actuator_moment` is identically zero (force on a rigid body against itself
    produces no net generalized force).
11. When site and refsite share a partial chain, only the shared-chain DOFs are
    zeroed; DOFs unique to either branch retain their Jacobian contributions.

**Backward compatibility:**
12. All existing Joint and Tendon actuator tests pass without modification (the
    6D gear change uses only `gear[0]` for these transmission types; their
    force-application and length/velocity paths are structurally unchanged).
13. MJCF models with scalar `gear="50"` parse as `[50, 0, 0, 0, 0, 0]`.
    Models with `gear="1 2 3 4 5 6 7"` parse as `[1, 2, 3, 4, 5, 6]` (extras
    ignored).

**Gain/bias compatibility:**
14. Site actuators work with all gain/bias types: Fixed, Affine, Muscle.
15. Muscle actuators with site transmission produce correct `acc0` (validated
    against manually-computed `||M^{-1} * moment||` where `moment` is the
    nv-vector from `mj_jac_site` wrench projection).

**Sensor correctness:**
16. `ActuatorFrc` sensor reports `data.actuator_force[i]` (scalar actuator force)
    for all transmission types, not `qfrc_actuator[dof_adr]`.
17. `ActuatorPos` sensor for Site transmissions returns the same value as
    `data.actuator_length[i]`.
18. `ActuatorVel` sensor for Site transmissions returns the same value as
    `data.actuator_velocity[i]`.

**Validation:**
19. MJCF with `site="nonexistent"` produces a parse/validation error.
20. MJCF with `refsite="nonexistent"` produces a parse/validation error.
21. MJCF with both `joint="j1"` and `site="s1"` on the same actuator produces a
    validation error (mutually exclusive transmission targets).

**MuJoCo conformance:**
22. For a representative model (e.g. 3-link arm with site actuators), actuator
    length, velocity, and generalized forces match MuJoCo ≤ 1e-8 relative
    tolerance. Test both Mode A and Mode B.

#### Files (all modified)
- ✅ `sim/L0/mjcf/src/types.rs` — `MjcfActuator.gear` → `[f64; 6]`, `refsite: Option<String>`, defaults updated
- ✅ `sim/L0/mjcf/src/parser.rs` — 6D gear parsing (1–6 floats, zero-padded), `refsite` attribute
- ✅ `sim/L0/mjcf/src/defaults.rs` — 6D gear defaults propagation
- ✅ `sim/L0/mjcf/src/builder/` — refsite → `actuator_trnid[i][1]`, 6D gear push, refsite warning for non-Site
- ✅ `sim/L0/mjcf/src/validation.rs` — site/refsite reference validation, mutual exclusivity enforcement
- ✅ `sim/L0/core/src/sensor/mod.rs` — `actuator_gear` → `Vec<[f64; 6]>`, `actuator_trnid` → `Vec<[usize; 2]>`, `Data.site_xquat`, `Data.actuator_moment`, `mj_jac_site()`, `mj_transmission_site()`, 5 stubs filled, `ActuatorFrc` sensor fix, `subquat()`, common-ancestor zeroing, `subquat_tests` (7), `jac_site_tests` (2)
- ✅ `sim/L0/tests/integration/site_transmission.rs` — 29 integration tests (criteria 1–22 all active, criterion 22 cross-validated against MuJoCo 3.5.0)
