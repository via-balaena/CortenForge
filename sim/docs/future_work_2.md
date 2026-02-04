# Simulation — Future Work (Phase 2)

Continuation of [FUTURE_WORK.md](./FUTURE_WORK.md). Phase 1 completed tasks 1–8, 12,
plus cleanup items C1–C2. This document covers the remaining roadmap.

## Priority Framework

Same axes as Phase 1:

| Axis | Definition |
|------|-----------|
| **RL Impact** | How directly this unblocks reinforcement-learning training workflows (batch stepping, observation fidelity, GPU throughput). |
| **Correctness** | Whether this fixes a simulation bug, stub, or semantic mismatch that produces wrong results today. |
| **Effort** | Implementation size: **S** (< 200 LOC), **M** (200–800 LOC), **L** (800–2 000 LOC), **XL** (> 2 000 LOC). |

Priority is **Correctness + RL Impact**, tie-broken by inverse Effort.

| # | Item | RL Impact | Correctness | Effort | Prerequisites |
|---|------|-----------|-------------|--------|---------------|
| 1 | `<default>` class resolution | High | **Critical** | S | None |
| 2 | Contact condim (1/4/6) + friction cones | Medium | **Critical** | L | None |
| 3 | `<contact>` pair/exclude | Medium | High | M | None |
| 4 | Spatial tendons + wrapping | Low | High | M | None |
| 5 | Site-transmission actuators | Low | High | M | #4 |
| 6 | Height field + SDF collision (MJCF wiring) | Low | Medium | S | None |
| 7 | Deferred sensors (JointLimitFrc, TendonLimitFrc) | Low | Medium | S | None |
| 8 | `<general>` actuator MJCF attributes | Low | Medium | S | None |
| 9 | Batched simulation | **High** | Low | L | None |
| 10 | GPU acceleration | **High** | Low | XL | #9 |
| 11 | Deformable body integration | Medium | Low | XL | None |
| 12 | Analytical derivatives (mjd_*) | **High** | Low | XL | None |
| 13 | Full implicit integrator | Low | Medium | L | None |
| 14 | Keyframes, mocap bodies, user callbacks | Medium | Low | L | None |
| 15 | Newton solver | Low | Low | XL | None |
| 16 | Sleeping / body deactivation | Low | Low | M | None |
| 17 | SOR relaxation for PGS | Low | Low | S | None |

## Dependency Graph

```
   ┌───┐
   │ 4 │ Spatial Tendons
   └─┬─┘
     │
     ▼
   ┌───┐
   │ 5 │ Site-Transmission Actuators
   └───┘

   ┌───┐
   │ 9 │ Batched Simulation
   └─┬─┘
     │
     ▼
   ┌────┐
   │ 10 │ GPU Acceleration
   └────┘

   ┌───┐  ┌───┐  ┌───┐  ┌───┐  ┌───┐  ┌───┐  ┌────┐  ┌────┐  ┌────┐  ┌────┐  ┌────┐
   │ 1 │  │ 2 │  │ 3 │  │ 6 │  │ 7 │  │ 8 │  │ 11 │  │ 12 │  │ 13 │  │ 14 │  │ 15 │
   └───┘  └───┘  └───┘  └───┘  └───┘  └───┘  └────┘  └────┘  └────┘  └────┘  └────┘
   (all independent)
```

---

## Group A — Correctness (Model Fidelity)

### 1. `<default>` Class Resolution
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State
`DefaultResolver` is fully implemented in `defaults.rs` (1,052 lines, 13 tests).
It builds inheritance hierarchies from nested `<default>` elements, resolves
attributes with class → parent → root lookup, and provides `apply_to_joint()`,
`apply_to_geom()`, `apply_to_actuator()`, `apply_to_tendon()`, `apply_to_sensor()`
methods for every element type.

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
In `model_builder.rs`, before each element processing loop (joints, geoms,
actuators, tendons, sensors):

1. Construct `DefaultResolver::new(&mjcf_model.defaults)` once per model build
2. For each element, look up its `class` attribute
3. Call `resolver.apply_to_*(&mut element, class)` before reading element fields
4. Per-element explicit attributes override defaults (already the case — defaults
   fill in missing values, explicit values win)

#### Acceptance Criteria
1. A model with `<default><joint damping="0.5"/></default>` and a bare
   `<joint type="hinge"/>` produces `joint_damping = 0.5`.
2. Per-element attributes override defaults: `<joint damping="1.0"/>` in a class
   that specifies `damping="0.5"` produces `1.0`.
3. Nested class inheritance works: child class inherits parent, overrides specific
   attributes.
4. All existing integration tests pass (no regressions — current tests don't use
   `<default>` classes, so they are unaffected).
5. New integration tests verify at least: joint defaults, geom defaults, actuator
   defaults.

#### Files
- `sim/L0/mjcf/src/model_builder.rs` — modify (wire `DefaultResolver` into
  `process_joints()`, `process_geoms()`, `process_actuators()`, etc.)
- `sim/L0/mjcf/src/defaults.rs` — reference (already complete)
- `sim/L0/tests/integration/` — new test file for default class verification

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

---

### 6. Height Field + SDF Collision (MJCF Wiring)
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State
sim-core has full collision support for height fields (`CollisionShape::HeightField`)
and SDFs (`CollisionShape::Sdf`). The mesh-sdf crate provides `SignedDistanceField`
with 10 shape-pair collision combinations.

The MJCF model builder does **not wire these geometry types** — `hfield` and `sdf`
geom types fall back to `CollisionShape::Box` during model construction.

#### Objective
Wire `<geom type="hfield">` and SDF geom types to their sim-core
`CollisionShape` variants in the model builder.

#### Specification
In the geom type dispatch in `model_builder.rs`, add arms for `hfield` and `sdf`:

- `"hfield"` → load the referenced `<hfield>` asset, construct
  `CollisionShape::HeightField` with the elevation grid and spatial dimensions.
- `"sdf"` → construct `CollisionShape::Sdf` backed by `mesh-sdf`'s
  `SignedDistanceField`.

Both shape types already have full collision detection in sim-core — only the
builder dispatch is missing.

#### Acceptance Criteria
1. `<geom type="hfield" hfield="terrain"/>` produces `CollisionShape::HeightField`.
2. SDF geoms produce `CollisionShape::Sdf`.
3. Collision detection between height field and sphere/capsule/box works.

#### Files
- `sim/L0/mjcf/src/model_builder.rs` — modify (geom type dispatch)

---

### 7. Deferred Sensors (JointLimitFrc, TendonLimitFrc)
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State
32 sensor types are parsed from MJCF; 30 are wired into the pipeline via
`process_sensors()`. Two are recognized but skipped at runtime with a warning:

- `JointLimitFrc` — joint limit constraint force
- `TendonLimitFrc` — tendon limit constraint force

Both require reading constraint forces from `qfrc_constraint` and mapping them
back to the specific joint/tendon limit that generated them.

#### Objective
Wire the two deferred sensors into the pipeline.

#### Specification
In `process_sensors()`, add evaluation arms for `JointLimitFrc` and
`TendonLimitFrc`:

- **JointLimitFrc:** After constraint solving, identify the constraint force
  contribution from the sensor's target joint limit. The force is the component
  of `qfrc_constraint` attributable to the joint's limit constraint (stored
  during PGS/CG solve). Return scalar force magnitude.

- **TendonLimitFrc:** Same pattern but for tendon limit constraints. The tendon
  limit force is the constraint force projected onto the tendon length direction.

Both sensors return 0 when the joint/tendon is within its limit range.

#### Acceptance Criteria
1. `JointLimitFrc` sensor reads the constraint force contribution from its
   joint's limit.
2. `TendonLimitFrc` sensor reads the constraint force contribution from its
   tendon's limit.
3. Zero force when joint/tendon is within limits.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (sensor evaluation in
  `process_sensors()`)

---

### 8. `<general>` Actuator MJCF Attributes
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State
The gain/bias runtime is fully general (any combination of Fixed/Affine/None
works). Only the MJCF parser-to-builder wiring for `<general>` actuators is
missing. Currently `<general>` actuators are treated as Motor-like
(gaintype=Fixed, gainprm=[1,0,0], biastype=None).

Deferred in Phase 1 (#12) because no RL models in common use require it. Included
here for completeness.

#### Objective
Parse explicit `gaintype`, `biastype`, `dyntype`, `gainprm`, `biasprm`, `dynprm`
attributes on `<general>` actuator elements.

#### Specification
In `parser.rs`, when parsing `<general>` actuators, read optional attributes:

- `gaintype` → `GainType` (Fixed, Affine, Muscle)
- `biastype` → `BiasType` (None, Affine, Muscle)
- `dyntype` → `ActuatorDynamics` (None, Filter, FilterExact, Integrator, Muscle)
- `gainprm`, `biasprm`, `dynprm` → `[f64; 10]` parameter arrays

In `model_builder.rs`, when the actuator type is `General` and explicit types are
provided, use them directly instead of the Motor-like defaults.

#### Acceptance Criteria
1. `<general gaintype="affine" gainprm="0 0 -5" biastype="none"/>` produces a
   damper-like actuator with `GainType::Affine`.
2. `<general>` without explicit type attributes still defaults to Motor-like
   (Fixed/None) — backward compatible.
3. All valid combinations of gaintype/biastype produce correct force output.

#### Files
- `sim/L0/mjcf/src/parser.rs` — modify (parse attributes on `<general>`)
- `sim/L0/mjcf/src/model_builder.rs` — modify (wire parsed types)

---

## Group B — Scaling & Performance

### 9. Batched Simulation
**Status:** Not started | **Effort:** L | **Prerequisites:** None

*Transferred from [FUTURE_WORK.md](./FUTURE_WORK.md) #10.*

#### Current State
Single-environment execution. `Data::step(&mut self, &Model)` (`mujoco_pipeline.rs:2599`)
steps one simulation. `Model` is immutable after construction, uses
`Arc<TriangleMeshData>` for shared mesh data (`mujoco_pipeline.rs:912`). `Data` is
fully independent — no shared mutable state, no interior mutability, derives `Clone`
(`mujoco_pipeline.rs:1391`).

#### Objective
Step N independent environments in parallel on CPU. Foundation for GPU acceleration
(#10) and large-scale RL training.

#### Specification

```rust
pub struct BatchSim {
    model: Arc<Model>,
    envs: Vec<Data>,
}

impl BatchSim {
    pub fn new(model: Arc<Model>, n: usize) -> Self;
    pub fn step_all(&mut self) -> BatchResult;
    pub fn reset(&mut self, env_idx: usize);
    pub fn reset_where(&mut self, mask: &[bool]);
}
```

`step_all()` uses rayon `par_iter_mut` over `envs`. Each `Data` steps independently
against the shared `Arc<Model>`. rayon 1.10 is already a workspace dependency;
sim-core declares it optional under the `parallel` feature flag
(`core/Cargo.toml:19,33`).

```rust
pub struct BatchResult {
    pub states: DMatrix<f64>,       // (n_envs, nq + nv) row-major
    pub rewards: DVector<f64>,      // (n_envs,)
    pub terminated: Vec<bool>,      // per-env episode termination
    pub truncated: Vec<bool>,       // per-env time limit
    pub errors: Vec<Option<StepError>>, // None = success
}
```

`states` is a contiguous matrix for direct consumption by RL frameworks (numpy
interop via row-major layout). Reward computation is user-defined:

```rust
pub trait RewardFn: Send + Sync {
    fn compute(&self, model: &Model, data: &Data) -> f64;
}
```

**Design constraint — single Model:** All environments share the same `Arc<Model>`
(same nq, nv, geom set). The `states: DMatrix<f64>` layout (n_envs, nq + nv)
requires uniform state dimensions. Multi-model batching (different robots in the
same batch) would require a fundamentally different design and is explicitly
out of scope.

**Error handling:** Environments that fail (e.g., `CholeskyFailed`,
`SingularMassMatrix`) are recorded in `errors`, flagged in `terminated`, and
auto-reset on the next `step_all()` call. The batch never aborts due to a single
environment failure.

**SIMD integration:** sim-simd provides within-environment acceleration:
`batch_dot_product_4()`, `batch_aabb_overlap_4()`, `batch_normal_force_4()`,
`batch_friction_force_4()`, `batch_integrate_position_4()`,
`batch_integrate_velocity_4()`. These accelerate the inner loop of each
environment's step. Cross-environment parallelism comes from rayon, not SIMD.

#### Acceptance Criteria
1. `BatchSim::step_all()` produces identical results to calling `Data::step()` on each environment sequentially — parallelism does not change simulation output.
2. `states` matrix layout is stable: row = env, cols = qpos ++ qvel.
3. Failed environments do not affect healthy environments in the same batch.
4. Linear throughput scaling up to available CPU cores (verified by benchmark with 1, 2, 4, 8 threads).
5. Zero-copy state extraction — `states` is filled directly from `Data` fields without intermediate allocations.
6. `reset_where()` resets only flagged environments without touching others.

#### Files
- `sim/L0/core/src/` — create `batch.rs` module
- `sim/L0/core/Cargo.toml` — modify (enable rayon under `parallel` feature)

---

### 10. GPU Acceleration
**Status:** Not started | **Effort:** XL | **Prerequisites:** #9

*Transferred from [FUTURE_WORK.md](./FUTURE_WORK.md) #11.*

#### Current State
CPU-only. No GPU infrastructure exists in the simulation pipeline. The `mesh-gpu`
crate provides wgpu context and buffer management for rendering but has no compute
shader infrastructure.

#### Objective
wgpu compute shader backend for batch simulation. Thousands of parallel environments
on a single GPU for RL training at scale.

#### Specification

Port the inner loop of `Data::step()` (FK, collision, PGS, integration) to compute
shaders via wgpu. The `BatchSim` API from #9 defines the memory layout that the
GPU backend fills.

This item is intentionally kept sparse — it is blocked on #9 and should not be
over-specified until the CPU batching API stabilizes. Key design decisions to be
made at implementation time:

- Which pipeline stages move to GPU first (integration is simplest, collision is
  most impactful).
- Data transfer strategy (host-pinned memory, persistent GPU buffers, double-buffering).
- Whether to use wgpu compute shaders or a lower-level API (Vulkan compute, Metal).
- Fallback path for systems without GPU support.

#### Acceptance Criteria
1. GPU-batched simulation produces identical results to CPU-batched (#9) for the same inputs.
2. For >= 1024 environments, GPU throughput exceeds CPU throughput on supported hardware.
3. Graceful fallback to CPU batching when no GPU is available.

#### Files
- `sim/L0/core/src/` — create `gpu.rs` module or new `sim-gpu` crate
- `mesh-gpu/` — reference for wgpu context management

---

## Group C — Physics Completeness

### 11. Deformable Body Pipeline Integration
**Status:** Not started | **Effort:** XL | **Prerequisites:** None

*Transferred from [FUTURE_WORK.md](./FUTURE_WORK.md) #9.*

#### Current State
sim-deformable is a standalone 7,733-line crate (86 tests):

| Component | Location | Description |
|-----------|----------|-------------|
| `XpbdSolver` | `solver.rs:134` | XPBD constraint solver. `step(&mut self, body: &mut dyn DeformableBody, gravity: Vector3<f64>, dt: f64)` (`solver.rs:196`). Configurable substeps, damping, sleeping. |
| `DeformableBody` trait | `lib.rs:173` | Common interface for Cloth, SoftBody, CapsuleChain. |
| `Cloth` | `cloth.rs` | Triangle meshes with distance + dihedral bending constraints. `thickness` field (`cloth.rs:50`). Presets: cotton, silk, leather, rubber, paper, membrane. |
| `SoftBody` | `soft_body.rs` | Tetrahedral meshes with distance + volume constraints. Presets: rubber, gelatin, soft tissue, muscle, foam, stiff. |
| `CapsuleChain` | `capsule_chain.rs` | 1D particle chains with distance + bending constraints. `radius` field (`capsule_chain.rs:41`). Presets: rope, steel cable, hair, chain. |
| `Material` | `material.rs` | Young's modulus, Poisson's ratio, density, `friction` (`material.rs:94`). 14 presets. |
| `ConstraintType::Collision` | `constraints.rs:42-43` | Enum variant defined but unimplemented — no constraint implements it. |
| `FlexEdge` | `constraints.rs` | Stretch, shear, twist constraint variants. |

The crate has zero coupling to the MuJoCo pipeline. sim-physics re-exports it behind
the `deformable` feature flag (`physics/src/lib.rs:109-110`). `Material.friction` and
per-body `radius`/`thickness` fields are declared but unused by the collision system
(which doesn't exist yet).

#### Objective
Deformable bodies interact with rigid bodies through the same contact solver and
step in the same simulation loop.

#### Specification

**Collision detection** (greenfield — no infrastructure exists):

1. **Broadphase:** Deformable vertex AABBs vs rigid geom AABBs. Vertex AABBs are
   point + radius/thickness margin. Use `batch_aabb_overlap_4()` (`simd/src/batch_ops.rs:251`)
   for batched broadphase queries.
2. **Narrowphase:** Vertex-vs-geom closest point computation. Produces `Contact` structs
   identical to rigid-rigid contacts (same normal, depth, friction, solref/solimp).
3. **Friction:** Combine `Material.friction` (deformable side) with geom friction
   (rigid side) using the same combination rule as rigid-rigid contacts.

**Contact solver coupling:**

Deformable-rigid contacts feed into PGS (or CG) alongside rigid-rigid
contacts. Each deformable vertex is a 3-DOF point mass. Its inverse mass comes from
the XPBD solver's per-particle mass. Contact Jacobians for deformable vertices are
3x3 identity blocks (point mass — no rotational DOFs).

**Force feedback:**

Contact impulses from PGS apply to deformable vertex velocities directly and to
rigid body `qfrc_constraint` through the standard Jacobian transpose. XPBD
constraint projection runs after contact resolution within the same timestep.

**Substep iteration (XPBD/contact ordering):**

A single pass (contact solve -> XPBD) may leave contacts invalid because XPBD
constraint projection moves vertices after contacts are computed. For stiff
deformable bodies or deep penetrations, this causes jitter.

Options (to be chosen at implementation time):
- **Option A (simple):** Single pass, accept minor inaccuracy. Sufficient for cloth
  and rope where deformation is small relative to contact depth.
- **Option B (robust):** Iterate contact-detection + solve + XPBD for
  `n_substep_iterations` (default 1, configurable up to 4). Each iteration
  re-detects contacts at updated vertex positions. More expensive but handles
  stiff soft bodies contacting rigid surfaces.

The choice should be configurable per-model via an MJCF option.

**Pipeline integration in `Data::step()`:**

```
1. Rigid: forward kinematics, collision, forces
2. Deformable-rigid collision detection -> Contact list
3. Combined contact solve (rigid + deformable contacts)
4. Apply contact impulses to rigid bodies and deformable vertices
5. XpbdSolver::step() for each registered deformable body
6. (Optional) Repeat steps 2-5 for substep iterations > 1
7. Rigid: position integration
```

#### Acceptance Criteria
1. A rigid body resting on a deformable surface experiences the same contact forces as resting on a rigid surface of equivalent geometry.
2. XPBD internal constraints (distance, bending, volume) are satisfied after contact resolution — contact forces do not violate deformable material properties.
3. `Material.friction` is used in deformable-rigid contacts (not hardcoded).
4. `ConstraintType::Collision` is implemented in the constraint system.
5. Zero-deformable-body configurations have zero overhead (no broadphase, no substep).
6. Cloth draped over a rigid sphere reaches stable equilibrium without jitter.
7. Substep iteration count is configurable; default (1) works for cloth/rope use cases.

#### Files
- `sim/L0/deformable/src/` — modify (collision detection, `ConstraintType::Collision` implementation)
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (pipeline integration in `Data::step()`, deformable body registration)
- `sim/L0/simd/src/batch_ops.rs` — reference (`batch_aabb_overlap_4()`)

---

### 12. Analytical Derivatives (mjd_*)
**Status:** Not started | **Effort:** XL | **Prerequisites:** None

#### Current State
No analytical derivative infrastructure exists. The pipeline computes contact
Jacobians (`compute_contact_jacobian()` at line 7619) and body-point Jacobians
(`compute_body_jacobian_at_point()` at line 7534) for the constraint solver, but
these are not exposed as a general derivative API.

MuJoCo provides `mjd_transitionFD` (finite-difference derivatives of the full
transition function) and analytical derivatives for specific pipeline stages. These
are critical for model-based RL (iLQR, DDP), trajectory optimization, and system
identification.

#### Objective
Provide derivatives of the simulation transition function
`(q_{t+1}, v_{t+1}) = f(q_t, v_t, u_t)` with respect to state and control.

#### Specification

This item is intentionally sparse — the implementation strategy (analytical vs
finite-difference vs automatic differentiation) should be chosen at design time:

**Option A (pragmatic):** Finite-difference derivatives via `mjd_transitionFD`
pattern — step the simulation with perturbed inputs and compute centered
differences. Easy to implement, works with the existing pipeline, O(nv + nu) cost
per derivative computation.

**Option B (performant):** Analytical derivatives for each pipeline stage. Requires
deriving and implementing gradients through FK, contact, constraint solver, and
integration. Matches MuJoCo's `mjd_*` functions. Much more complex but O(1)
additional cost.

**Option C (modern):** Automatic differentiation via dual numbers or tape-based AD.
Would require making the pipeline generic over scalar type or using an AD library.

#### Acceptance Criteria
1. `d(q_{t+1})/d(q_t)`, `d(q_{t+1})/d(v_t)`, `d(q_{t+1})/d(u_t)` are computable.
2. Derivatives agree with finite differences to within numerical precision.
3. Derivative computation cost is documented (FD: ~O(nv * step_cost), analytical: ~O(step_cost)).

#### Files
- `sim/L0/core/src/` — new module (derivatives)

---

### 13. Full Implicit Integrator
**Status:** Not started | **Effort:** L | **Prerequisites:** None

#### Current State
`Integrator::ImplicitSpringDamper` implements diagonal-only implicit integration:

- Per-DOF stiffness K and damping D from joint properties
- Solves `(M + h*D + h^2*K) * v_new = M*v_old + h*f_ext - h*K*(q - q_eq)`
- Diagonal K and D — no off-diagonal coupling
- Tendon springs/dampers explicitly skipped in implicit mode
  (`mujoco_pipeline.rs:7403–7406`, comment: "Tendon springs/dampers couple multiple
  joints (non-diagonal K/D), so they cannot be absorbed into the existing diagonal
  implicit modification.")

MuJoCo's full implicit integrator handles the complete mass-matrix coupling
including off-diagonal stiffness and damping terms from tendons and other
multi-joint coupling elements.

#### Objective
Extend the implicit integrator to handle full (non-diagonal) stiffness and damping
matrices, including tendon spring/damper coupling across joints.

#### Specification

Replace diagonal K/D vectors with sparse K/D matrices. In `mj_fwd_passive()`,
compute off-diagonal entries from tendon Jacobians:

```
K_ij += J_tendon[i] * k_tendon * J_tendon[j]
D_ij += J_tendon[i] * d_tendon * J_tendon[j]
```

The implicit solve becomes `(M + h*D + h^2*K) * v = rhs` with sparse M+hD+h^2K.
Reuse the existing sparse Cholesky infrastructure from Task #2 (Phase 1).

#### Acceptance Criteria
1. With tendon springs, implicit integrator produces stable integration where
   explicit Euler diverges.
2. Diagonal-only case (no tendons) matches current `ImplicitSpringDamper` output
   (regression test).
3. Off-diagonal K/D terms are sparse — zero overhead for joints without coupling.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (`mj_fwd_passive()`,
  `mj_fwd_acceleration_implicit()`, implicit parameter storage)

---

### 14. Keyframes, Mocap Bodies, User Callbacks
**Status:** Not started | **Effort:** L | **Prerequisites:** None

#### Current State
None of these features exist in the pipeline. The only reference to "keyframe" in
the codebase is a doc comment in sim-constraint's equality module.

#### Objective
Support MJCF `<keyframe>` state snapshots for quick reset, `mocap` bodies for
externally driven poses, and (deferred) user callback hooks for custom logic
injection during simulation.

#### Specification

**Keyframes:**

Store named state snapshots in `Model`:

```rust
pub struct Keyframe {
    pub name: String,
    pub qpos: DVector<f64>,
    pub qvel: DVector<f64>,
    pub act: DVector<f64>,
    pub ctrl: DVector<f64>,
}
```

`Data::reset_to_keyframe(&mut self, &Model, keyframe_idx: usize)` restores
state. Parse from MJCF `<keyframe><key name="..." qpos="..."/></keyframe>`.

**Mocap bodies:**

Bodies with `mocap="true"` have externally driven position/orientation. They
participate in collision but are not integrated — their pose is set directly
via `data.mocap_pos[i]` and `data.mocap_quat[i]`.

```rust
// In Data:
pub mocap_pos: Vec<Vector3<f64>>,
pub mocap_quat: Vec<UnitQuaternion<f64>>,
```

Pipeline skips FK and integration for mocap bodies; collision uses the
mocap-set pose directly.

**User callbacks (deferred):**

MuJoCo's `mjcb_*` hooks (passive force, control, sensor, collision filter) are
useful but require careful API design for Rust's ownership model. Defer to a
follow-up — keyframes and mocap are higher priority.

#### Acceptance Criteria
1. `Data::reset_to_keyframe()` restores exact state from MJCF `<keyframe>`.
2. Mocap body pose is externally settable and affects collision but not integration.
3. Mocap bodies have zero mass contribution to the system.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (Model fields, Data fields,
  `Data::step()` skip for mocap, `reset_to_keyframe()`)
- `sim/L0/mjcf/src/parser.rs` — modify (parse `<keyframe>`, `mocap` attribute)
- `sim/L0/mjcf/src/model_builder.rs` — modify (build keyframe/mocap data)

---

### 15. Newton Solver
**Status:** Not started | **Effort:** XL | **Prerequisites:** None

#### Current State
The standalone Newton solver was deleted during Phase 3 crate consolidation (commit
`a5cef72`). No remnants exist. The pipeline has PGS and CG (PGD+BB) solvers.

MuJoCo's Newton solver uses analytical second-order derivatives of the constraint
cost and typically converges in 2–3 iterations, making it faster than PGS for
stiff problems. It is the recommended solver for most MuJoCo applications.

#### Objective
Implement a Newton contact solver in the pipeline, operating on the same
`assemble_contact_system()` infrastructure as PGS and CG.

#### Specification

Intentionally sparse — this is a large effort and should be fully specified at
implementation time. Key design points:

- Reuse `assemble_contact_system()` for Delassus matrix + RHS
- Newton direction: solve `H * delta = -g` where H is the Hessian of the
  constraint cost and g is the gradient
- Line search or trust region for step size
- Add `SolverType::Newton` variant
- Projected Newton (respect `lambda_n >= 0` bounds)

#### Acceptance Criteria
1. Newton solver converges to same solution as PGS/CG (within tolerance).
2. Newton solver converges in fewer iterations than PGS for stiff contacts.
3. Falls back to PGS if Newton fails to converge.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (new solver function, SolverType
  variant, dispatch in `mj_fwd_constraint()`)

---

## Group D — Quality of Life

### 16. Sleeping / Body Deactivation
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State
No sleep/deactivation system. Every body is simulated every step regardless of
whether it is stationary. MuJoCo deactivates bodies whose velocity is below a
threshold for a configurable duration, skipping their dynamics until an external
force or contact wakes them.

#### Objective
Bodies at rest are automatically deactivated, reducing computation for scenes with
many stationary objects.

#### Specification

Per-body sleep state:

```rust
pub body_sleep_time: Vec<f64>,   // time at zero velocity (in Model or Data)
pub body_asleep: Vec<bool>,      // deactivation flag
```

In `Data::step()`, after integration:
1. For each body, if `|v| < sleep_threshold` for `sleep_duration` steps, set
   `body_asleep = true`.
2. Asleep bodies skip FK, force computation, and integration.
3. Wake on: external force applied, contact with awake body, `ctrl` change on
   attached actuator.

#### Acceptance Criteria
1. Stationary bodies deactivate after configurable duration.
2. Contact with an active body wakes sleeping bodies.
3. Scene with 100 resting bodies and 1 active body runs faster than all-active.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (sleep tracking, skip logic)

---

### 17. SOR Relaxation for PGS
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State
PGS uses plain Gauss-Seidel (relaxation factor omega = 1.0). MuJoCo's PGS uses SOR
with configurable omega for accelerated convergence.

#### Objective
Add SOR relaxation parameter to PGS solver.

#### Specification

In `pgs_solve_with_system()` (`mujoco_pipeline.rs:8049`), after the GS update:

```rust
lambda_new = (1 - omega) * lambda_old + omega * lambda_gs;
```

where `omega` is read from `Model.solver_sor` (default 1.0, parsed from MJCF
`<option sor="..."/>`).

#### Acceptance Criteria
1. omega = 1.0 matches current behavior (regression).
2. omega = 1.3 converges in fewer iterations for a stiff contact benchmark.
3. omega < 1.0 (under-relaxation) is stable for pathological cases.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (`pgs_solve_with_system()`,
  Model field)

---

## Appendix: Deferred / Low Priority

Items acknowledged but not prioritized for Phase 2:

| Item | Reason for deferral |
|------|-------------------|
| `dampratio` attribute (Position actuators) | Builder-only change, no RL models use it. Deferred in Phase 1 #12. |
| Explicit Euler integrator | MuJoCo supports it but semi-implicit Euler is strictly superior. No use case. |
| Velocity Verlet integrator | Not in MuJoCo. Was in standalone system (deleted). No pipeline need. |
| Implicit-fast (no Coriolis) | Optimization variant. ImplicitSpringDamper covers primary use case. |
| Planar/Cylindrical joints in pipeline | In sim-constraint standalone. No MJCF models use them. |
| `<custom>` element (user data) | Low priority — no simulation effect. |
| `<extension>` element (plugins) | Large design effort, no immediate need. |
| `<visual>` element | L1 concern (sim-bevy), not core physics. |
| `<statistic>` element | Auto-computed model stats. Informational only. |
| Sleeping bodies in deformable | Depends on both #11 and #16. |
| Sparse mass matrix (deeper MuJoCo parity) | Phase 1 #1/#2 cover the main path. Full sparse pipeline is diminishing returns. |
| MuJoCo conformance test suite | Important but orthogonal to features — can be built incrementally. |
| SIMD utilization (unused batch ops) | sim-simd exists; utilization will come naturally with #9/#10. |
| Tendon equality constraints | Standalone in sim-constraint. Pipeline tendons work; equality coupling is rare. |

---

## Cross-Reference: Phase 1 Mapping

| Phase 1 # | Phase 2 # | Notes |
|-----------|-----------|-------|
| #9 (Deformable Body) | #11 | Transferred verbatim |
| #10 (Batched Simulation) | #9 | Transferred verbatim |
| #11 (GPU Acceleration) | #10 | Transferred verbatim |
