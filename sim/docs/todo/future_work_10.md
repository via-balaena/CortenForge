# Future Work 10 — Phase 3A: Constraint/Joint Features + Physics + Pipeline + Trait Architecture (Items #38–42, #42B–F)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

Items #38–42B are prerequisites to #45 (MuJoCo Conformance Test Suite). This file
covers constraint/joint features (ball/free limits, tendon wrapping), physics
(fluid forces), pipeline flags, deformable body MJCF parsing, and the trait
architecture (§42C–F) described in [TRAIT_ARCHITECTURE.md](../TRAIT_ARCHITECTURE.md).

§42B–F form the trait architecture rollout: §42B (Phase A) proves the pattern with
`FlexBendingModel`, then §42C/D/E extend it to elasticity, actuators, and contact
solvers, and §42F assembles them into the composable `SimBuilder`. §42C–F are NOT
prerequisites to #45 — they add non-MuJoCo extensions and can proceed in parallel
with Batches 5–9 once §42B lands.

---

### 38. Ball / Free Joint Limits (Swing-Twist Cones)
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

---

### 39. `wrap_inside` Algorithm (Tendon Wrapping)
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State

When a sidesite is inside a wrapping geometry, the code **panics** at
`mujoco_pipeline.rs:3848` with "The wrap_inside algorithm is not implemented."
This is a runtime crash on valid models.

#### Objective

Implement the `wrap_inside` resolution algorithm so that tendon wrapping handles
the sidesite-inside-geometry case gracefully instead of panicking.

#### Specification

1. **Detection**: The current code already detects the inside case (distance from
   sidesite to wrapping geometry center < radius). The `todo!()` panic needs to
   be replaced with the resolution algorithm.
2. **Algorithm (sphere wrapping)**: When a sidesite is inside the wrapping sphere:
   - Project the sidesite position onto the sphere surface along the line from
     center to sidesite.
   - Use the projected point as the effective sidesite for wrapping calculations.
   - If sidesite is exactly at center (degenerate), use the previous frame's
     wrap point or a default direction.
3. **Algorithm (cylinder wrapping)**: When a sidesite is inside the wrapping
   cylinder:
   - Project radially outward to the cylinder surface.
   - Maintain the axial component.
4. **MuJoCo reference**: MuJoCo's `mju_wrap` handles the inside case in
   `engine_core_smooth.c`. The key behavior: when inside, the tendon path goes
   through the wrapping geometry center (zero-length wrap segment) rather than
   crashing.
5. **No panic**: Replace `todo!()` with the computed wrap-inside path. Never
   panic on valid MJCF input.

#### Acceptance Criteria

1. A model with sidesite inside wrapping sphere does not panic.
2. A model with sidesite inside wrapping cylinder does not panic.
3. Tendon length is continuous as sidesite transitions from outside to inside
   the wrapping geometry.
4. Existing wrapping tests (18 spatial tendon tests) pass unchanged.
5. New test: sidesite deliberately placed inside wrap sphere, verify tendon
   length matches MuJoCo.

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — replace `todo!()` at the wrap_inside
  call site with proper algorithm

---

### 40. Fluid / Aerodynamic Forces
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

---

### 41. `disableflags` — Runtime Disable Flag Effects
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State

MJCF parser parses all 21 `<flag>` attributes. But only `DISABLE_ISLAND` has a
runtime constant and check in the pipeline. Flags like `contact="disable"`,
`gravity="disable"`, `limit="disable"`, `equality="disable"` are parsed but have
NO runtime effect.

#### Objective

Wire parsed disable flags into the pipeline so each flag conditionally skips its
corresponding stage.

#### Specification

1. **Flag constants**: Define constants for each disable flag (some may already
   exist as parsed values — wire them to runtime checks).
2. **Pipeline gates**: Add flag checks at each relevant pipeline stage:
   - `DISABLE_GRAVITY`: skip gravity contribution in `mj_rne`.
   - `DISABLE_CONTACT`: skip collision detection and contact constraint assembly.
   - `DISABLE_LIMIT`: skip joint/tendon limit enforcement.
   - `DISABLE_EQUALITY`: skip equality constraint assembly.
   - `DISABLE_FRICTIONLOSS`: skip friction loss passive force.
   - `DISABLE_PASSIVE`: skip all passive forces.
   - `DISABLE_ACTUATION`: skip actuator force computation.
   - `DISABLE_SENSOR`: skip sensor evaluation.
   - `DISABLE_REFSAFE`: skip reference safety clamping.
   - (remaining flags as documented in MuJoCo API reference)
3. **Enable flags**: Similarly wire enable flags (`ENABLE_ENERGY`, etc.) —
   `ENABLE_ENERGY` is already wired. Check others.

#### Acceptance Criteria

1. `<flag contact="disable"/>` prevents contact detection.
2. `<flag gravity="disable"/>` zeros gravitational contribution to bias forces.
3. `<flag limit="disable"/>` prevents joint limit enforcement.
4. All 21 parsed flags have runtime effect.
5. Default flag values match MuJoCo defaults (all disabled flags off, all
   enabled flags per MuJoCo spec).

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — flag checks at each pipeline stage

---

### 42. `<flex>` / `<flexcomp>` MJCF Deformable Body Parsing
**Status:** Subsumed by §6B | **Effort:** L | **Prerequisites:** None

> **Note:** This item has been subsumed by [§6B Flex Solver Unification](./future_work_6b_precursor_to_7.md),
> which implements full `<flex>`/`<flexcomp>` parsing along with the unified flex
> constraint pipeline. See `future_work_6b_precursor_to_7.md` §P9 for the
> implemented parsing specification.

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

---

### 42B. Flex Bending Discretization: Cotangent Laplacian + Trait Abstraction
**Status:** Not started | **Effort:** L | **Prerequisites:** §6B ✅

#### Background

§6B implemented flex bending as Bridson et al. 2003 dihedral angle springs in
`mj_fwd_passive()`. MuJoCo uses Wardetzky et al. "Discrete Quadratic Curvature
Energies" (cotangent Laplacian) with Garg et al. "Cubic Shells" curved
reference correction — a precomputed 4×4 stiffness matrix per edge.

These are fundamentally different:

| | **Bridson (ours)** | **Wardetzky/Garg (MuJoCo)** |
|-|---|---|
| **Force model** | `F = -k * (θ - θ₀) * grad_i` | `F_i = Σ_j B[i,j] * x_j + B[16] * frc_i` |
| **Gradient norms** | Geometry-dependent, change each step | Constant (precomputed cotangent weights) |
| **Large deformation** | Correct — uses actual dihedral angle | Linearized — accurate only for small deflections |
| **Stability** | Requires per-vertex force clamp | Unconditionally stable (constant matrix) |
| **Runtime cost** | Per-hinge: atan2 + cross products + 4 gradient vectors | Per-edge: 4×4 matrix-vector (16 muls) + cross products |
| **Precomputation** | None (rest angle stored) | 17 coefficients per edge |
| **Conformance** | Diverges from MuJoCo | Exact match |

Neither is strictly superior — Bridson captures nonlinear bending faithfully
(cloth draping, sheet metal forming), while the cotangent Laplacian is efficient,
stable, and matches MuJoCo exactly. We should support both via a trait.

#### Current State

Bending force computation in `mj_fwd_passive()` (`mujoco_pipeline.rs:11370+`):
- Bridson dihedral angle springs with per-vertex force clamping
- `Model.flex_bend_stiffness: Vec<f64>` (scalar per flex body)
- `Model.flex_bend_damping: Vec<f64>` (scalar per flex body)
- `Model.flexhinge_vert: Vec<[usize; 4]>` — hinge topology `[e0, e1, a, b]`
- `Model.flexhinge_angle0: Vec<f64>` — rest dihedral angles
- Force clamp: `fm_max = 1/(dt² * |grad| * invmass)` per vertex per hinge

MuJoCo (`engine_passive.c:206–268`):
- `flex_bending` array: 17 `f64` per edge (4×4 stiffness matrix + 1 curved ref coeff)
- `b[17*e + 4*i + j]` = stiffness coupling between vertex `i` and vertex `j`
- `b[17*e + 16]` = curved reference contribution coefficient
- Force: `spring[3*i+x] += b[4*i+j] * xpos[3*v[j]+x] + b[16] * frc[i][x]`
- Damper: `damper[3*i+x] += b[4*i+j] * vel[j][x]` (same matrix)
- No stability clamp — constant matrix guarantees linear force growth

Precomputation (`user_mesh.cc:3740–3784`, `ComputeBending`):
```
mu = young / (2 * (1 + poisson))
stiffness = 3 * mu * thickness³ / (24 * volume)
```
Uses Wardetzky cotangent operator: `c[i]` weights from cotangent of dihedral
angles at rest configuration. The 4×4 outer product `c[i] * c[j] * cos_theta * stiffness`
gives the stiffness matrix entries. The `b[16]` curved reference term handles
non-flat rest configurations.

#### Objective

1. Implement the Wardetzky/Garg cotangent Laplacian bending model
2. Abstract bending computation behind a `FlexBendingModel` trait
3. Default to cotangent (MuJoCo conformance), allow Bridson via configuration
4. Precompute `flex_bending` coefficients at model compile time

#### Specification

##### S1. `FlexBendingModel` trait

```rust
/// Bending force model for 2D flex bodies (dim=2).
pub trait FlexBendingModel {
    /// One-time precomputation from rest geometry and material properties.
    /// Called during model compilation.
    fn precompute(
        &mut self,
        model: &Model,
        flex_id: usize,
    );

    /// Accumulate bending forces into qfrc_passive.
    /// Called each step from mj_fwd_passive().
    fn apply_forces(
        &self,
        model: &Model,
        data: &mut Data,
        flex_id: usize,
    );
}
```

Two implementations:
- `CotangentBending` — precomputes 17-coefficient matrix per edge, applies via
  matrix-vector multiply. No stability clamp needed.
- `BridsonBending` — current implementation (dihedral angle springs + per-vertex
  force clamp). No precomputation beyond rest angles.

##### S2. Model storage

Add to `Model`:
```rust
/// Bending stiffness matrix (17 f64 per edge, Wardetzky cotangent Laplacian).
/// Layout: [4×4 vertex coupling matrix, 1 curved reference coeff] per edge.
/// Only populated when using cotangent bending model.
pub flex_bending: Vec<f64>,

/// Which bending model each flex body uses.
pub flex_bending_model: Vec<FlexBendingType>,
```

```rust
pub enum FlexBendingType {
    /// Wardetzky/Garg cotangent Laplacian (MuJoCo-conformant, default).
    Cotangent,
    /// Bridson dihedral angle springs (nonlinear, large-deformation accurate).
    Bridson,
}
```

##### S3. Precomputation (cotangent model)

During model compilation in `model_builder.rs`, for each flex body with
`bending_model == Cotangent`:

1. For each interior edge (edge where both adjacent triangles exist):
   - Compute 4 cotangent weights `c[0..4]` from rest vertex positions using
     Wardetzky's formula (cotangent of dihedral angles in the diamond stencil)
   - Compute `volume` as sum of adjacent triangle areas
   - Compute `mu = young / (2 * (1 + poisson))` (shear modulus)
   - Compute `stiffness = 3 * mu * thickness³ / (24 * volume)`
   - Compute edge vector `e0`, flap vectors `e1, e2`, tangent vectors `t0, t1`
   - Compute `cos_theta = -dot(t0, t1) / |e0|²`
   - Fill 4×4 matrix: `bending[4*i + j] = c[i] * c[j] * cos_theta * stiffness`
   - Compute curved reference coefficient `bending[16]` for non-flat rest shapes
2. For boundary edges: zero the 17 coefficients (boundary has no bending)

##### S4. Runtime force application (cotangent model)

In `mj_fwd_passive()`, replace the current per-hinge loop with a per-edge loop
for cotangent bending:

```rust
for e in 0..model.nflexedge {
    let flex_id = model.flexedge_flexid[e];
    if model.flex_bending_model[flex_id] != FlexBendingType::Cotangent {
        continue;
    }
    let b = &model.flex_bending[17 * e..17 * (e + 1)];
    let [v0, v1] = model.flexedge_vert[e];
    let [va, vb] = model.flexedge_flap[e]; // adjacent vertices
    if vb == usize::MAX { continue; } // boundary edge

    let v = [v0, v1, va, vb];
    // spring: F_i += Σ_j b[4*i+j] * x_j + b[16] * curved_ref_i
    // damper: F_i += damping * Σ_j b[4*i+j] * vel_j
    // (See MuJoCo engine_passive.c:240–266 for exact loop)
}
```

No stability clamp — the constant matrix produces forces linear in positions.

##### S5. Bridson model (preserve current implementation)

Move the current dihedral angle loop into `BridsonBending::apply_forces()`.
No changes to the algorithm — keep the per-vertex force clamp.

##### S6. MJCF configuration

Option A: Custom CortenForge extension attribute on `<flex>`:
```xml
<flex name="cloth" dim="2" young="1e4" bending_model="bridson"/>
```

Default: `cotangent` (MuJoCo conformance).

Option B: Global option:
```xml
<option cortenforge:bending_model="bridson"/>
```

Prefer Option A — per-flex-body granularity is more useful.

##### S7. Edge topology: `flexedge_flap`

MuJoCo stores `flex_edgeflap` — for each edge, the two "flap" vertices (one
from each adjacent triangle, opposite the shared edge). We need the same:

```rust
/// For each edge: the two vertices opposite the edge in adjacent triangles.
/// [va, vb] where va is in triangle 1, vb is in triangle 2.
/// Boundary edges: vb = usize::MAX.
pub flexedge_flap: Vec<[usize; 2]>,
```

This is computed during model compilation from element connectivity.

#### Acceptance Criteria

1. **Cotangent conformance**: Bending forces match MuJoCo 3.x for a 4×4 grid
   shell with `young=1e4, poisson=0.3, thickness=0.01, density=1000`. Compare
   `qfrc_spring` (MuJoCo) vs `qfrc_passive` (ours) after 1 step with gravity.
   Tolerance: `1e-10` per component.
2. **Bridson equivalence**: Switching to `bending_model="bridson"` produces the
   same forces as the current implementation (regression test).
3. **Stability without clamp**: Cotangent model with `young=1e12, dt=0.01`
   (the AC20 test scenario) remains stable for 500 steps without any force
   clamping. The stability comes from the constant matrix, not from clamps.
4. **Curved reference**: A non-flat rest mesh (e.g., a curved shell) produces
   correct bending forces via the `b[16]` curved reference term.
5. **Boundary edges**: Boundary edges (only one adjacent triangle) produce zero
   bending force.
6. **Trait dispatch**: Swapping `FlexBendingType` on a model changes force
   computation without touching any other pipeline code.
7. **Mixed-model dispatch**: A model with two flex bodies — one cotangent, one
   Bridson — dispatches correctly per flex body and produces correct forces for
   both simultaneously.
8. **Zero trait overhead**: `mj_fwd_passive()` throughput with cotangent bending
   through the trait is within 1% of a hardcoded implementation (no performance
   regression from the trait boundary). This validates the pattern for Phases B–E
   of the [trait architecture](../TRAIT_ARCHITECTURE.md).

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `FlexBendingModel` trait,
  `CotangentBending`, `BridsonBending`, `mj_fwd_passive()` dispatch
- `sim/L0/mjcf/src/model_builder.rs` — `flex_bending` precomputation,
  `flexedge_flap` topology, `bending_model` attribute parsing
- `sim/L0/mjcf/src/parser.rs` — parse `bending_model` attribute from `<flex>`
- `sim/L0/tests/integration/flex_unified.rs` — conformance tests, stability
  tests, regression tests for Bridson path

---

### 42C. `FlexElasticityModel` Trait — Phase B (Membrane / Edge Elasticity)
**Status:** Not started | **Effort:** L | **Prerequisites:** §42B

#### Background

The flex edge constraint currently uses a soft equality constraint with solref/solimp
(matching MuJoCo's `mjEQ_FLEX` mechanism). This is `LinearElastic` — small-strain
response expressed as constraint rows in the unified Jacobian.

Nonlinear hyperelastic materials (Neo-Hookean, Mooney-Rivlin, Ogden) require
strain-energy-based forces computed from deformation gradients. These cannot be
expressed as constraint rows — they produce direct passive forces from the strain
energy density function.

The divergence:

| Regime | Model | Mechanism | Use Case |
|--------|-------|-----------|----------|
| Small strain | Linear elastic (current) | Constraint row (`FlexEdge`) | Robotics, stiff bodies, MuJoCo conformance |
| Large strain | Neo-Hookean / SVK | Passive force from strain energy | Soft tissue, surgical sim, rubber |
| Hyperelastic | Mooney-Rivlin, Ogden | Passive force from strain energy invariants | Biomechanics, material characterization |

#### Specification

##### S1. `FlexElasticityModel` trait

```rust
pub trait FlexElasticityModel {
    /// Per-flex precomputed data (opaque to the pipeline).
    type Precomputed;

    /// Precompute coefficients at model build time.
    fn precompute(
        flex_id: usize,
        model: &Model,
    ) -> Self::Precomputed;

    /// Apply elasticity forces/constraints for this flex body.
    /// Constraint-based models add rows to the Jacobian.
    /// Force-based models accumulate into qfrc_passive.
    fn apply(
        flex_id: usize,
        pre: &Self::Precomputed,
        model: &Model,
        data: &mut Data,
    );
}
```

##### S2. `LinearElastic` (extract current implementation)

Move the existing `FlexEdge` constraint row assembly behind the trait:
- `precompute()`: Store rest edge lengths (already computed at build time).
- `apply()`: Assemble edge equality constraint rows into the Jacobian, exactly
  as the current code does. No behavioral change.

##### S3. `NeoHookean` (new implementation)

Strain-energy-based passive force computation:
- **Energy**: `W = μ/2 (I₁ - 3) - μ ln(J) + λ/2 (ln J)²`
  where `I₁ = tr(FᵀF)`, `J = det(F)`, `F` is the deformation gradient.
- **Force**: `f = -∂W/∂x` computed per element from the deformation gradient.
- `precompute()`: Compute rest-state inverse reference matrices (`Dm⁻¹`) per element,
  rest volumes, and material parameters (Lamé: `μ = E / (2(1+ν))`,
  `λ = Eν / ((1+ν)(1-2ν))`).
- `apply()`: For each element, compute deformation gradient `F = Ds * Dm⁻¹`,
  compute first Piola-Kirchhoff stress `P = ∂W/∂F`, accumulate nodal forces
  `f = -V₀ * P * Dm⁻ᵀ` into `qfrc_passive`.

For dim=2 (shells): Use the in-plane deformation gradient (2×2 → 3×2 map)
with thickness integrated out. For dim=3 (solids): Full 3×3 deformation gradient.

##### S4. Per-flex dispatch

```rust
pub enum FlexElasticityType {
    /// Linear elastic constraint rows (MuJoCo-conformant, default).
    Linear,
    /// Neo-Hookean hyperelastic passive forces.
    NeoHookean,
}
```

Stored per flex body in `Model.flex_elasticity_model: Vec<FlexElasticityType>`.

##### S5. MJCF configuration

```xml
<flex name="tissue" dim="3" young="5e3" poisson="0.45"
      elasticity_model="neo_hookean"/>
```

Default: `linear` (MuJoCo conformance).

#### Acceptance Criteria

1. `LinearElastic` through the trait produces identical constraint rows as the
   current direct implementation (bit-exact regression).
2. `NeoHookean` on a dim=3 cube under gravity produces physically plausible
   large-deformation response (cube deforms, doesn't explode, conserves volume
   approximately for ν → 0.5).
3. `NeoHookean` on a dim=2 shell under gravity produces membrane stretch forces.
4. Mixed model: one flex body linear, one neo-Hookean, both simulate correctly.
5. Neo-Hookean with small strain converges to linear elastic response
   (validation: compare forces at 1% strain, should agree within 5%).

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `FlexElasticityModel` trait,
  `LinearElastic`, `NeoHookean`, dispatch in constraint assembly + passive forces
- `sim/L0/mjcf/src/model_builder.rs` — `elasticity_model` attribute, precomputation
- `sim/L0/mjcf/src/parser.rs` — parse `elasticity_model` from `<flex>`
- `sim/L0/tests/integration/flex_unified.rs` — hyperelastic tests, regression tests

---

### 42D. `ActuatorGainModel` Trait — Phase C (Actuator Gain Models)
**Status:** Not started | **Effort:** M | **Prerequisites:** §42B

#### Background

MuJoCo has three gain types dispatched by enum: `Fixed`, `Affine`, `Muscle`. The set
is closed — adding a new gain model requires modifying the enum. Real actuator modeling
needs an open set: series elastic actuators (SEA compliance), pneumatic actuators
(nonlinear pressure-volume), hydraulic actuators (valve dynamics), cable-driven actuators
(cable elasticity + friction).

The existing enum dispatch is correct for the MuJoCo-compatible types and should remain.
The trait extends it for user-defined models that can't be expressed as
`force = gainprm[0] + gainprm[1]*length + gainprm[2]*velocity`.

#### Specification

##### S1. `ActuatorGainModel` trait

```rust
pub trait ActuatorGainModel {
    type Params;

    /// Compute actuator force contribution.
    fn gain(
        length: f64,
        velocity: f64,
        activation: f64,
        ctrl: f64,
        params: &Self::Params,
    ) -> f64;

    /// Compute ∂force/∂velocity for implicit integration.
    fn dgain_dvel(
        length: f64,
        velocity: f64,
        activation: f64,
        ctrl: f64,
        params: &Self::Params,
    ) -> f64;
}
```

##### S2. Built-in implementations

Extract the existing gain computation into trait implementations:
- `FixedGain`: `force = gainprm[0] * ctrl`
- `AffineGain`: `force = gainprm[0] + gainprm[1]*length + gainprm[2]*velocity`
- `MuscleGain`: Hill-type muscle model (existing `compute_muscle_gain()`)

These must produce identical results to the current enum-dispatched code.

##### S3. `SeriesElasticGain` (new, proof-of-concept)

A simple SEA model as the second non-MuJoCo implementation:
- `force = k_spring * (x_motor - x_joint) + d * (v_motor - v_joint)`
- Where `x_motor = ctrl * gear_ratio`, `v_motor = d(ctrl)/dt * gear_ratio`
- `Params`: `{ k_spring: f64, damping: f64, gear_ratio: f64 }`

This is a common actuator model in legged robotics that cannot be expressed as
MuJoCo's affine gain.

##### S4. Dispatch

The existing `GainType` enum stays for MuJoCo-compatible types (zero-cost match
dispatch in the hot loop). Custom gain models use trait dispatch:

```rust
pub enum GainDispatch {
    /// Built-in MuJoCo types (match dispatch, zero-cost).
    Builtin(GainType),
    /// Custom gain model (trait dispatch).
    Custom(Box<dyn ActuatorGainModel<Params = CustomGainParams>>),
}
```

For monomorphized dispatch (no vtable), the `SimBuilder` approach from §42F
would eliminate the `Box<dyn>`. §42D can use dynamic dispatch as a first step
since actuator force computation is not the innermost hot loop.

##### S5. MJCF configuration

Custom gain models are configured via `<general>` actuator with a custom attribute:

```xml
<general name="sea_hip" joint="hip" gaintype="custom"
         cortenforge:gain_model="series_elastic"
         cortenforge:gain_params="1000 10 50"/>
```

Or via the Rust API:
```rust
model.set_actuator_gain(actuator_id, SeriesElasticGain {
    k_spring: 1000.0, damping: 10.0, gear_ratio: 50.0,
});
```

#### Acceptance Criteria

1. `FixedGain`, `AffineGain`, `MuscleGain` through the trait produce identical
   forces as the current enum dispatch (bit-exact regression).
2. `SeriesElasticGain` produces correct spring-damper forces for a 1-DOF test case.
3. `dgain_dvel` is correct for all implementations (verify numerically with finite
   differences).
4. Models without custom gain types see zero overhead (existing enum path unchanged).
5. Implicit integrator works correctly with custom gain models (uses `dgain_dvel`).

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `ActuatorGainModel` trait, built-in
  implementations, `SeriesElasticGain`, dispatch in `mj_fwd_actuation()`
- `sim/L0/mjcf/src/parser.rs` — parse custom gain attributes
- `sim/L0/tests/` — regression tests for built-in gains, SEA model tests

---

### 42E. Contact Solver Trait — Phase E (Contact Formulation Extensibility)
**Status:** Not started | **Effort:** XL | **Prerequisites:** §42B

#### Background

The current architecture has enum dispatch for contact solvers (`SolverType::PGS | CG
| Newton`). This works because all three solve the same LCP formulation — they differ
in numerics, not physics.

Future solvers may differ in **formulation**, not just solution strategy:

| Solver | Formulation | Physics | Use Case |
|--------|------------|---------|----------|
| PGS/CG/Newton | LCP (complementarity) | Rigid contact, Coulomb friction | Robotics, MuJoCo conformance |
| XPBD | Position-based constraints | Compliant contact, regularized | Real-time, games, animation |
| Impulse-based | Velocity-level impulses | Event-driven, exact collision | Billiards, granular media |
| Compliant contact | Kelvin-Voigt / Hunt-Crossley | Viscoelastic contact | Soft robotics, grasping |

These aren't different algorithms for the same problem — they're different **problem
formulations**. An enum can't capture this because the input/output types differ:
LCP solvers consume a Delassus matrix + constraint bounds, XPBD consumes position
constraints + compliance, impulse-based solvers consume collision events + restitution.

#### Specification

##### S1. `ContactSolver` trait

```rust
pub trait ContactSolver {
    /// Solver-specific configuration.
    type Config;

    /// Solver-specific per-step workspace.
    type Workspace;

    /// Allocate workspace for this step's contacts.
    fn prepare(
        config: &Self::Config,
        model: &Model,
        data: &Data,
        contacts: &[Contact],
    ) -> Self::Workspace;

    /// Solve for contact forces / impulses / position corrections.
    /// Modifies data.qacc (or data.qpos for position-level solvers).
    fn solve(
        config: &Self::Config,
        workspace: &mut Self::Workspace,
        model: &Model,
        data: &mut Data,
        contacts: &[Contact],
    );
}
```

##### S2. `LcpSolver` (extract current implementation)

Wraps the existing PGS/CG/Newton solver behind the trait:
- `Config`: `{ solver_type: SolverType, iterations: usize, tolerance: f64, noslip_iterations: usize }`
- `Workspace`: The existing `ConstraintState` (Delassus matrix, lambda, residuals)
- `prepare()`: Assemble Jacobian + Delassus (existing `mj_fwd_constraint()` logic)
- `solve()`: Dispatch to PGS/CG/Newton based on `solver_type` (existing solver code)

The internal PGS/CG/Newton enum dispatch stays — this is numerics-level dispatch within
a single formulation.

##### S3. `XpbdSolver` (new implementation)

Extended Position-Based Dynamics for compliant contact:
- `Config`: `{ iterations: usize, substeps: usize }`
- `Workspace`: Position constraint data, compliance matrices
- `prepare()`: Generate position-level contact constraints from penetration depths
- `solve()`: Iterative constraint projection with compliance:
  `Δx = -C(x) / (∇C·M⁻¹·∇Cᵀ + α/dt²)` where `α` is compliance

XPBD is the standard real-time physics formulation (Macklin et al. 2016). It's
simpler than LCP, faster per iteration, but less physically accurate.

##### S4. Dispatch

```rust
pub enum ContactSolverType {
    /// LCP solver (PGS/CG/Newton). MuJoCo-conformant.
    Lcp(SolverType),
    /// XPBD compliant contact. Real-time, position-based.
    Xpbd,
}
```

Default: `Lcp(SolverType::Newton)`.

##### S5. MJCF configuration

```xml
<option solver="newton"/>  <!-- existing, unchanged -->
<option cortenforge:solver="xpbd" cortenforge:xpbd_substeps="4"/>
```

#### Acceptance Criteria

1. `LcpSolver` through the trait produces identical results as the current direct
   implementation for PGS, CG, and Newton (bit-exact regression).
2. `XpbdSolver` produces stable contact for a box resting on a plane.
3. `XpbdSolver` resolves interpenetration within the specified iterations.
4. Models using default solver see zero overhead (LCP path unchanged).
5. Switching solver type at runtime (between steps) works correctly.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `ContactSolver` trait, `LcpSolver`,
  `XpbdSolver`, dispatch in constraint solve stage
- `sim/L0/tests/` — regression tests for LCP, XPBD stability tests

---

### 42F. `SimBuilder` Composition — Phase D (Trait Assembly)
**Status:** Not started | **Effort:** L | **Prerequisites:** §42C, §42D, §42E

#### Background

§42B (Phase A) and §42C/D/E (Phases B, C, E) each introduce a trait boundary.
Phase D assembles them into a composable builder that produces a fully monomorphized
simulation type — no vtable overhead in the inner loop.

#### Specification

##### S1. Generic `Sim` type

```rust
pub struct Sim<B, E, A, C>
where
    B: FlexBendingModel,
    E: FlexElasticityModel,
    A: ActuatorGainModel,
    C: ContactSolver,
{
    model: Model,
    data: Data,
    bending: B,
    elasticity: E,
    actuators: A,
    contact_solver: C,
}
```

##### S2. `SimBuilder`

```rust
pub struct SimBuilder<B = CotangentBending, E = LinearElastic, A = DefaultGain, C = LcpSolver> {
    bending: B,
    elasticity: E,
    actuators: A,
    contact_solver: C,
}

impl SimBuilder {
    pub fn new() -> Self { /* defaults: MuJoCo-conformant config */ }
}

impl<B, E, A, C> SimBuilder<B, E, A, C> {
    pub fn bending<B2: FlexBendingModel>(self, b: B2) -> SimBuilder<B2, E, A, C> { ... }
    pub fn elasticity<E2: FlexElasticityModel>(self, e: E2) -> SimBuilder<B, E2, A, C> { ... }
    pub fn actuators<A2: ActuatorGainModel>(self, a: A2) -> SimBuilder<B, E, A2, C> { ... }
    pub fn contact_solver<C2: ContactSolver>(self, c: C2) -> SimBuilder<B, E, A, C2> { ... }
    pub fn build(self, model: Model) -> Result<Sim<B, E, A, C>, Error> { ... }
}
```

##### S3. Type aliases for common configurations

```rust
/// MuJoCo-conformant defaults. This is what `Model::make_data()` + `Data::step()` uses.
pub type MujocoSim = Sim<CotangentBending, LinearElastic, DefaultGain, LcpSolver>;

/// Large-deformation cloth/animation preset.
pub type AnimationSim = Sim<BridsonBending, LinearElastic, DefaultGain, XpbdSolver>;

/// Soft robotics / biomechanics preset.
pub type SoftBodySim = Sim<CotangentBending, NeoHookean, DefaultGain, LcpSolver>;
```

##### S4. Backward compatibility

The existing `Model`/`Data` API does not change:
- `load_model()` + `make_data()` + `step()` continues to work exactly as before.
- Internally, this uses `MujocoSim` (the default configuration).
- `SimBuilder` is the power-user API for non-default configurations.

##### S5. `Sim::step()` pipeline

`Sim::step()` calls the same pipeline stages as `Data::step()`, but dispatches
trait calls through the generic parameters instead of hardcoded implementations:

```rust
impl<B, E, A, C> Sim<B, E, A, C>
where
    B: FlexBendingModel,
    E: FlexElasticityModel,
    A: ActuatorGainModel,
    C: ContactSolver,
{
    pub fn step(&mut self) -> Result<(), Error> {
        mj_step_common(&self.model, &mut self.data)?;
        self.bending.apply_forces(...);
        self.elasticity.apply(...);
        self.actuators.gain(...);
        self.contact_solver.solve(...);
        mj_step_integrate(&self.model, &mut self.data)?;
        Ok(())
    }
}
```

All dispatch is monomorphized — the compiler sees concrete types, not trait objects.

#### Acceptance Criteria

1. `SimBuilder::new().build(model)` produces a `MujocoSim` that passes all existing
   conformance tests.
2. `SimBuilder::new().bending(BridsonBending).build(model)` produces an `AnimationSim`
   that uses Bridson bending.
3. Mixing traits: `SimBuilder::new().elasticity(NeoHookean::new(mu, lambda)).contact_solver(XpbdSolver::new(4)).build(model)` compiles and runs.
4. `Model::make_data()` + `Data::step()` is unchanged — no regression, no API break.
5. Compile-time monomorphization: no vtables in the hot path (verify via assembly
   inspection or `cargo-asm`).
6. Domain randomization example: a training loop that randomly selects bending model
   per episode compiles and produces distinct dynamics.

#### Files
- `sim/L0/core/src/sim_builder.rs` — new module: `Sim<B, E, A, C>`, `SimBuilder`,
  type aliases
- `sim/L0/core/src/lib.rs` — re-export `SimBuilder`, `MujocoSim`, etc.
- `sim/L0/core/src/mujoco_pipeline.rs` — refactor pipeline stages to accept trait
  parameters
- `sim/L0/tests/` — builder composition tests, regression tests, domain randomization
  example test
