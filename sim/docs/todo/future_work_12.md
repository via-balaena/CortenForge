# Future Work 12 — Phase 3B: Format Completeness + Performance (Items #46–50)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

SIMD audit first (informs hot-path optimization), then format gaps, then CCD last
(largest effort item, benefits from SIMD findings). Lower priority than Phase 3A
(MuJoCo alignment) but needed for full parity.

---

### 46. `<composite>` Element (Procedural Body Generation)
**Status:** Not started | **Effort:** XL | **Prerequisites:** #18 (`<include>` + `<compiler>`)

#### Current State
The MJCF parser does not handle the `<composite>` element. This is a macro-expansion
feature in MuJoCo that procedurally generates body/geom/joint/tendon trees from
templates (grid, rope, cable, cloth, box, cylinder, ellipsoid). It is widely used for
soft-body and deformable setups. Many composite models also use `<include>`, making
#18 a practical prerequisite.

#### Objective
Implement `<composite>` as a pre-processing expansion that generates equivalent
explicit MJCF elements before model building.

#### Specification

1. **Composite types** (MuJoCo supports 7):
   - `grid` — 2D grid of bodies (cloth, membranes)
   - `rope` / `cable` — 1D chain of bodies (ropes, cables)
   - `cloth` — 2D deformable mesh
   - `box` / `cylinder` / `ellipsoid` — 3D volumetric soft bodies
2. **Expansion**: For each `<composite>`, generate:
   - Body tree with specified count/spacing
   - Geoms (spheres or capsules) on each body
   - Joints (ball or hinge) connecting adjacent bodies
   - Tendons (if `tendon` sub-element specified)
   - Skin (if `skin` sub-element specified)
3. **Pre-processing**: Expand composites in the XML DOM before `model_builder`
   processes the tree. The builder never sees `<composite>` — only expanded elements.

#### Acceptance Criteria
1. `<composite type="rope" count="10"/>` generates a 10-body chain.
2. `<composite type="grid" count="5 5"/>` generates a 5×5 body grid.
3. Generated models match MuJoCo's composite expansion (body count, joint types,
   connectivity).
4. Models without `<composite>` are unaffected (regression).

#### Files
- `sim/L0/mjcf/src/composite.rs` — new module for composite expansion
- `sim/L0/mjcf/src/lib.rs` — pre-processing hook before model building

---

### 47. URDF Loader Completeness
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State
URDF loading works for basic models but has known gaps:
- **Mesh geometry**: Parsed but not converted to collision shapes — mesh geoms are
  silently dropped.
- **Friction**: `<dynamics friction="...">` parsed but not applied to contacts.
- **Planar joint**: Approximated as a single hinge, losing 2 of 3 DOF.
- **`<mimic>` joints**: Not supported (coupled joint motion).
- **Kinematic loops**: Not supported (tree structures only).

#### Objective
Close the URDF→Model conversion gaps so standard ROS URDF models load faithfully.

#### Specification

1. **Mesh geometry**: Convert URDF mesh references (STL/OBJ/DAE file paths) to
   `GeomType::Mesh` with loaded vertex data. Reuse existing mesh loading
   infrastructure from MJCF path.
2. **Friction**: Map URDF `<dynamics friction="...">` to `geom.friction[0]`
   (sliding friction coefficient). Apply to generated contacts.
3. **Planar joint**: Map to 2 slide joints + 1 hinge (3 DOF) matching ROS
   convention: translation in XY plane + rotation about Z.
4. **`<mimic>` joints**: Parse `<mimic joint="..." multiplier="..." offset="..."/>`
   and generate `<equality><joint>` coupling constraints.

#### Acceptance Criteria
1. A URDF with mesh collision geoms generates contacts.
2. URDF friction values affect contact solver behavior.
3. Planar joint provides 3 DOF movement.
4. Simple URDF models (no meshes, no planar) unchanged (regression).

#### Files
- `sim/L0/urdf/src/` — URDF→Model conversion

---

### 48. SIMD Utilization Audit + Wiring
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State
`sim-simd` crate (2,015 lines) provides batch SIMD operations: `Vec3x4`, `Vec3x8`,
`batch_aabb_overlap_4`, `batch_normal_force_4`, `batch_integrate_*`, `find_max_dot`.
Only `find_max_dot()` has production callers (in GJK/EPA support function search).
All other batch ops have zero callers outside benchmarks.

#### Objective
Audit hot paths in the pipeline for SIMD opportunities and wire in existing batch
operations where they provide measurable speedup.

#### Specification

1. **Audit targets** (profile first, optimize second):
   - Broad-phase AABB overlap testing — candidate for `batch_aabb_overlap_4`
   - Contact force accumulation — candidate for `batch_normal_force_4`
   - Position/velocity integration — candidate for `batch_integrate_*`
   - Jacobian-transpose force mapping — candidate for `Vec3x4` batch dot products
2. **Wiring**: For each candidate, benchmark SIMD vs scalar on realistic workloads
   (≥100 contacts, ≥30 bodies). Only wire SIMD if speedup > 1.3×.
3. **Cleanup**: Remove unused batch ops from `sim-simd` if no production use case
   materializes after audit.

#### Acceptance Criteria
1. Profile data for top-5 hot paths in pipeline step.
2. SIMD wired for ≥2 hot paths with measured speedup > 1.3×.
3. Unused batch ops removed or documented as "benchmark-only".
4. No correctness regressions (bit-exact where possible, tolerance where not).

#### Files
- `sim/L0/simd/src/` — existing batch ops
- `sim/L0/core/src/forward/` — hot path integration points

---

### 49. Non-Physics MJCF Elements
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State
Several MJCF elements are not parsed. None affect physics simulation, but their
absence causes warnings or limits interoperability:
- `<visual>` — rendering parameters (L1/sim-bevy concern)
- `<statistic>` — auto-computed model statistics (center, extent, meansize)
- `<custom>` — user-defined numeric/text data
- `<size>` — memory size hints (may not apply to Rust allocation model)

#### Objective
Parse and store these elements to suppress warnings and improve MJCF round-trip
fidelity. No simulation effect.

#### Specification

1. **`<statistic>`**: Parse `center`, `extent`, `meaninertia`, `meanmass`,
   `meansize`. Store on `Model`. Auto-compute from model geometry if not specified
   (MuJoCo behavior).
2. **`<custom>`**: Parse `<numeric>` and `<text>` children. Store in
   `Model.custom_numeric: HashMap<String, Vec<f64>>` and
   `Model.custom_text: HashMap<String, String>`.
3. **`<size>`**: Parse and store. No runtime effect (Rust manages memory
   dynamically).
4. **`<visual>`**: Parse top-level visual parameters (global, quality, headlight,
   map, scale, rgba). Store on `Model` for L1 consumption. Do not implement
   rendering (that's sim-bevy's job).

#### Acceptance Criteria
1. Models with these elements load without warnings.
2. Stored values accessible via `Model` API.
3. No simulation behavior change (regression).

#### Files
- `sim/L0/mjcf/src/builder/` — parse + store
- `sim/L0/core/src/types/model.rs` — `Model` struct fields (if needed)

---

### 50. CCD (Continuous Collision Detection)
**Status:** Parsed only | **Effort:** L | **Prerequisites:** None

#### Current State
`ccd_iterations` is parsed from `<option>` and stored on `MjcfOption`.
`nativeccd` and `multiccd` enable flags are stored in `MjcfFlag`. But no CCD
implementation exists — these fields have no runtime effect.
Contacts are detected via discrete overlap testing only, which can miss fast-moving
thin objects (tunneling).

#### Objective
Implement conservative-advancement CCD for convex geom pairs.

#### Specification

1. **Algorithm**: Conservative advancement — iteratively advance the time-of-impact
   estimate using upper-bound velocity and minimum separation distance.
2. **Scope**: Convex-convex pairs only (sphere, capsule, box, ellipsoid, convex mesh).
   Non-convex pairs (mesh-mesh, hfield) use discrete detection.
3. **Integration**: After broad-phase, for pairs with relative velocity exceeding a
   threshold, run CCD to find earliest time-of-impact. Generate contact at TOI
   configuration.
4. **Parameters**: `ccd_iterations` (max bisection steps), `ccd_tolerance` (distance
   threshold for contact).
5. **SIMD opportunity**: The GJK distance query in the conservative advancement inner
   loop is a candidate for SIMD batch support function evaluation (see #48 audit).

#### Acceptance Criteria
1. A fast-moving sphere does not tunnel through a thin wall.
2. `ccd_iterations=0` disables CCD (current behavior, regression).
3. CCD contacts produce forces consistent with discrete contacts at TOI.

#### Files
- `sim/L0/core/src/collision/` — post-broadphase CCD pass
- `sim/L0/core/src/gjk_epa.rs` — GJK distance query for conservative advancement
