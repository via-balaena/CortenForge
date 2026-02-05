# Future Work 3 — Correctness Remainder + Scaling (Items #6–10)

Part of [Simulation Phase 2 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

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
32 MJCF sensor types are parsed (`MjcfSensorType` in `types.rs`). Of these, 30
are converted to pipeline sensor types (`MjSensorType` in `mujoco_pipeline.rs`)
via `convert_sensor_type()` in `model_builder.rs:2694–2725` and fully evaluated
in the pipeline. Two are recognized at parse time but return `None` from
conversion and are skipped with a warning at model build time:

- `Jointlimitfrc` — joint limit constraint force
- `Tendonlimitfrc` — tendon limit constraint force

Neither has a corresponding `MjSensorType` variant. Both require reading
constraint forces from `qfrc_constraint` and mapping them back to the specific
joint/tendon limit that generated them.

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

*Transferred from [future_work_1.md](./future_work_1.md) #10.*

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

*Transferred from [future_work_1.md](./future_work_1.md) #11.*

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
