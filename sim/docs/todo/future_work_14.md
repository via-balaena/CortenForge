# Future Work 14 — Phase 3C: Data Fields + Derivatives + API (Items #56–59)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

These items add missing Data fields, analytical derivatives, and API surface
for full MuJoCo parity.

---

### ~~56. `subtree_linvel` / `subtree_angmom` as Data Fields~~
**Status:** ✅ Done (Phase 4, commit `503ac6d`) | **Effort:** S | **Prerequisites:** None

#### Current State

`subtree_com` exists as a Data field. But `subtree_linvel` and `subtree_angmom` are
computed on-demand only when sensors request them (via helper functions). Not stored
persistently.

#### Objective

Promote `subtree_linvel` and `subtree_angmom` to persistent Data fields, recomputed
each forward pass.

#### Specification

1. **Data fields**:
   - `subtree_linvel: Vec<[f64; 3]>` — per-body subtree linear velocity of COM.
   - `subtree_angmom: Vec<[f64; 3]>` — per-body subtree angular momentum about
     subtree COM.
2. **Computation**: Move the existing on-demand computation (used by
   SubtreeLinVel/SubtreeAngMom sensors) into the forward pass, storing results
   persistently.
3. **Sensors**: SubtreeLinVel and SubtreeAngMom sensors read from the persistent
   fields instead of recomputing.

#### Acceptance Criteria

1. `subtree_linvel[0]` (world body) is the whole-body COM velocity.
2. Sensor readings unchanged (regression — same values, just different read path).
3. Fields accessible without sensor evaluation.

#### Files

- `sim/L0/core/src/types/data.rs` — Data fields
- `sim/L0/core/src/forward/` — forward-pass computation

---

### 57. `sdf_iterations` / `sdf_initpoints` Option Attributes
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

Not parsed. SDF collision uses hardcoded parameters.

#### Objective

Parse `sdf_iterations` and `sdf_initpoints` from `<option>` and use them to
control SDF collision quality.

#### Specification

1. **MJCF parsing**: Parse `sdf_iterations` (int, default 10) and
   `sdf_initpoints` (int, default 40) from `<option>`.
2. **Storage**: Add to `MjcfOption` / `Model.opt`.
3. **Runtime**: Pass these values to the SDF collision solver instead of
   hardcoded constants.
   - `sdf_iterations`: maximum Newton iterations for SDF contact refinement.
   - `sdf_initpoints`: number of initial sample points for SDF collision search.

#### Acceptance Criteria

1. Default values match MuJoCo defaults (10, 40).
2. Increasing `sdf_initpoints` finds more contacts on complex SDF geometries.
3. Reducing `sdf_iterations` produces coarser but faster collision detection.
4. Non-SDF collision unaffected (regression).

#### Files

- `sim/L0/mjcf/src/builder/` — parse option attributes
- `sim/L0/core/src/collision/sdf_collide.rs` — pass to SDF collision

---

### 58. `mjd_smooth_pos` — Analytical Position Derivatives
**Status:** Not started | **Effort:** L | **Prerequisites:** None

#### Current State

Only `mjd_smooth_vel` (velocity derivatives) is analytical. Position columns of the
transition derivative A matrix use finite differencing. This makes transition
derivatives ~2× slower than necessary.

#### Objective

Implement analytical position derivatives to enable fully analytical transition
Jacobians.

#### Specification

1. **Algorithm**: Analytical derivatives of smooth dynamics (FK + RNE + passive)
   with respect to positions. This involves:
   - FK position derivatives: `∂xpos/∂qpos`, `∂xquat/∂qpos` — chain rule
     through joint transforms.
   - RNE position derivatives: `∂qfrc_bias/∂qpos` — position-dependent gravity
     torques and centrifugal terms.
   - Passive force position derivatives: `∂qfrc_passive/∂qpos` — spring terms.
2. **Integration with existing derivatives**: The existing `mjd_smooth_vel` and
   finite-difference position derivatives produce the transition matrix:
   ```
   A = [∂f/∂qpos  ∂f/∂qvel]
   ```
   Replace the FD `∂f/∂qpos` columns with the analytical version.
3. **MuJoCo reference**: MuJoCo 3.x provides `mjd_smooth_pos` which computes
   these analytically. Match the formulation.

#### Acceptance Criteria

1. Analytical position derivatives match finite-difference derivatives within
   `1e-6` relative tolerance.
2. Transition derivative computation is ≥1.5× faster with analytical position
   derivatives.
3. Existing velocity derivatives unchanged (regression).
4. `mjd_transitionFD` results match `mjd_transition` (analytical) within
   tolerance.

#### Files

- `sim/L0/core/src/derivatives.rs` — analytical position derivative functions

---

### ~~59. `mj_name2id` / `mj_id2name` — Public Name Lookup API~~
**Status:** ✅ Done (Phase 3 commit) | **Effort:** S | **Prerequisites:** None

#### Current State

`ModelBuilder` has internal `HashMap` lookups (`joint_name_to_id`,
`body_name_to_id`, etc.) but these are **private to the builder** and not exposed
on the final `Model` struct. `Model` has `body_name: Vec<Option<String>>` for
id-to-name, but no name-to-id lookup API. Users must manually iterate.

#### Objective

Expose O(1) bidirectional name↔index lookup on `Model`.

#### Specification

1. **Name-to-ID**: Add `HashMap<String, usize>` lookup tables for each element
   type (body, joint, geom, site, tendon, actuator, sensor) on `Model`.
   Populate during model building.
2. **API**:
   ```rust
   impl Model {
       pub fn name2id(&self, elem_type: ElementType, name: &str) -> Option<usize>;
       pub fn id2name(&self, elem_type: ElementType, id: usize) -> Option<&str>;
   }
   ```
3. **Element types**: body, joint, geom, site, tendon, actuator, sensor, mesh.
   Camera, light, material, and texture are deferred (rendering-layer, post-v1.0).
4. **Existing id2name**: The existing `body_name: Vec<Option<String>>` etc. fields
   already provide id-to-name. `id2name()` wraps these.

#### Acceptance Criteria

1. `model.name2id(ElementType::Body, "torso")` returns the correct body index.
2. `model.id2name(ElementType::Joint, 0)` returns the first joint's name.
3. Lookup is O(1) (HashMap-based).
4. Unnamed elements return `None`.

#### Files

- `sim/L0/core/src/types/model.rs` — Model fields and API methods
- `sim/L0/mjcf/src/builder/` — populate lookup tables during build
