# Future Work 13 — Phase 3C: API + Pipeline Completeness (Items #51–55)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

These items add missing API surface, data fields, and runtime behaviors needed
for full MuJoCo API parity.

---

### 51. `cacc` / `cfrc_int` / `cfrc_ext` — Body Force Accumulators
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State

`Data` struct has NO `cacc`, `cfrc_int`, or `cfrc_ext` fields. Derivative scratch
buffers exist (`deriv_Dcacc`, `deriv_Dcfrc`) but those are Jacobian matrices for
analytical derivatives, not the actual per-body force accumulators.

#### Objective

Add per-body 6D force/acceleration accumulators to `Data` and populate them
during the forward pass.

#### Specification

1. **Data fields**:
   - `cacc: Vec<[f64; 6]>` — per-body 6D acceleration (angular 3 + linear 3),
     computed during forward dynamics.
   - `cfrc_int: Vec<[f64; 6]>` — per-body internal/constraint forces, computed
     via inverse dynamics pass after constraint solve.
   - `cfrc_ext: Vec<[f64; 6]>` — per-body external forces (applied + actuator),
     accumulated during force assembly.
2. **Population**: Compute during `forward()`:
   - `cacc`: after computing `qacc`, propagate body accelerations via FK Jacobians.
   - `cfrc_int`: backward pass from `qacc` and `cacc` to compute internal forces
     at each joint (reaction forces).
   - `cfrc_ext`: sum of `xfrc_applied` and actuator-generated forces per body.
3. **MuJoCo convention**: Forces are in Cartesian (6D spatial vectors in world
   frame), not generalized coordinates.

#### Acceptance Criteria

1. `cacc[0]` (world body) is zero (fixed body, no acceleration).
2. For a free-falling body, `cacc` equals `[0,0,0, 0,0,-g]` (gravity only).
3. `cfrc_int` at the root joint matches total applied + gravity forces.
4. `cfrc_ext` reflects applied forces and actuator contributions.
5. Fields accessible via `data.cacc[body_id]`.

#### Files

- `sim/L0/core/src/types/data.rs` — Data fields
- `sim/L0/core/src/forward/` — accumulator computation

---

### 52. `mj_inverse()` — Inverse Dynamics API
**Status:** Not started | **Effort:** M | **Prerequisites:** #51 (body force accumulators)

#### Current State

Building blocks exist: subtree inverse dynamics for Force/Torque sensors via
`compute_site_force_torque()`, RNE for bias forces, CRBA for mass matrix. But no
public `mj_inverse()` API that computes `qfrc_inverse` from `(qpos, qvel, qacc)`.

#### Objective

Expose a public inverse dynamics function that computes the generalized forces
needed to produce given accelerations.

#### Specification

1. **API**: `pub fn inverse(model: &Model, data: &mut Data)` that reads `qpos`,
   `qvel`, `qacc` from `data` and computes `qfrc_inverse`.
2. **Algorithm**: `qfrc_inverse = M * qacc + qfrc_bias - qfrc_passive`. The
   building blocks (CRBA, RNE, passive forces) already exist. This is primarily
   an assembly and API wiring task.
3. **Body accumulators**: Full `mj_inverse` also populates `cacc`, `cfrc_int`,
   `cfrc_ext` (item #51). These should be populated during the inverse pass.
4. **Data field**: Add `qfrc_inverse: DVector<f64>` to `Data`.

#### Acceptance Criteria

1. `inverse()` produces forces that, when applied, reproduce the given `qacc`.
2. For a system at equilibrium (qacc=0), `qfrc_inverse` equals gravity + passive.
3. Round-trip: `forward()` → read `qacc` → `inverse()` → `qfrc_inverse` matches
   the total applied forces.
4. `ENABLE_FWDINV` guard wired: when set, compute forward/inverse comparison
   statistics (§41 S5.3).
5. `ENABLE_INVDISCRETE` guard wired: when set, use discrete-time inverse
   dynamics (§41 S5.4).

#### Files

- `sim/L0/core/src/forward/mod.rs` — `inverse()` function
- `sim/L0/core/src/types/data.rs` — `qfrc_inverse` field

---

### 53. `mj_step1` / `mj_step2` — Split Stepping API
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

Only `step()` (forward + integrate) and `forward()` are exposed. No separate
`step2()` for the integration-only phase.

#### Objective

Expose `step1()` and `step2()` methods for split stepping, allowing users to
inject forces between the forward pass and integration.

#### Specification

1. **`step1()`**: Equivalent to `forward()` — runs position-dependent computations
   (FK, collision, constraint solve, sensor evaluation). After `step1()`, all
   `Data` fields except `qpos`/`qvel` reflect the current state.
2. **`step2()`**: Runs integration only — updates `qpos` and `qvel` based on
   computed `qacc`. Users can modify `qfrc_applied` or `xfrc_applied` between
   `step1()` and `step2()`.
3. **`step()` unchanged**: `step()` remains `step1()` + `step2()` for convenience.
4. **No state flags**: No internal flag needed to track which step phase was last
   called. `step2()` simply integrates whatever state is current.

#### Acceptance Criteria

1. `step1()` followed by `step2()` produces identical results to `step()`.
2. Forces injected between `step1()` and `step2()` affect the integration.
3. `step()` behavior unchanged (regression).

#### Files

- `sim/L0/core/src/forward/mod.rs` — `step1()`, `step2()` methods

---

### 54. Heightfield Collision Gaps
**Status:** Not started | **Effort:** L | **Prerequisites:** None

#### Current State

Explicitly documented as "not supported" in `collision/` modules:
`// Hfield↔Hfield, Hfield↔Plane, Hfield↔Mesh: not supported`.
Heightfield collision works for sphere, capsule, and box geoms only.

#### Objective

Implement missing heightfield collision pairs: hfield↔mesh, hfield↔plane,
hfield↔hfield.

#### Specification

1. **Hfield↔Mesh**: For each mesh triangle, test against the heightfield grid
   cells it overlaps. Generate contacts at triangle-terrain intersections.
   This is the highest-priority pair (mesh objects on terrain).
2. **Hfield↔Plane**: Clip the heightfield to the plane. Generate contacts at
   grid vertices below the plane. Lower priority (unusual scenario).
3. **Hfield↔Hfield**: Grid-to-grid overlap testing. Lowest priority (rare).
4. **Algorithm**: For hfield↔mesh, use the existing heightfield grid traversal
   (used for sphere/capsule) but with triangle-cell intersection tests instead
   of point-cell distance tests.

#### Acceptance Criteria

1. A mesh object dropped onto a heightfield terrain generates contacts.
2. Existing sphere/capsule/box on heightfield unchanged (regression).
3. Contact normals and depths are physically reasonable.

#### Files

- `sim/L0/core/src/collision/hfield.rs` — heightfield collision functions

---

### 55. `*_user` Custom Data Fields
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

Zero references in codebase. Not parsed, not stored. MuJoCo provides `nuser_*`
fields for arbitrary per-element user data arrays.

#### Objective

Parse and store per-element user data arrays from MJCF.

#### Specification

1. **MJCF parsing**: Parse `user` attribute from `<body>`, `<geom>`, `<joint>`,
   `<tendon>`, `<actuator>`, `<sensor>`, `<site>` elements. The attribute is a
   space-separated list of floats.
2. **`<size>` nuser fields**: Parse `nuser_body`, `nuser_geom`, `nuser_jnt`,
   `nuser_tendon`, `nuser_actuator`, `nuser_sensor` from `<size>`. These set
   the expected array length (MuJoCo pads/truncates to this length).
3. **Model storage**: Add `body_user: Vec<Vec<f64>>`, `geom_user: Vec<Vec<f64>>`,
   etc. to `Model`.
4. **Runtime**: User data is read-only from the physics perspective — no pipeline
   effect. Accessible via `Model` API.

#### Acceptance Criteria

1. `<body user="1.0 2.0 3.0">` stores `[1.0, 2.0, 3.0]` on the body.
2. `<size nuser_body="5"/>` pads shorter `user` arrays with zeros.
3. No simulation behavior change (regression).
4. User data accessible via `model.body_user[body_id]`.

#### Files

- `sim/L0/mjcf/src/builder/` — parse user attributes
- `sim/L0/core/src/types/model.rs` — Model storage fields
