# GPU Physics Pipeline Spec

## 1. Problem

The current physics step runs entirely on CPU, with one exception:
`sim-gpu` accelerates SDF-SDF narrowphase collision via a WGSL compute
shader. But this creates a **half-GPU** pattern — upload poses, dispatch
shader, readback contacts, continue on CPU. At VR rates (90 Hz × 2–4
substeps = 180–360 steps/sec), the per-step transfer latency dominates.

```
Current (per substep):
  CPU: FK, CRBA, RNE, actuation, passive, broadphase
    → CPU→GPU: upload poses
       → GPU: trace_surface.wgsl
          → GPU→CPU: readback contacts
             → CPU: assembly, solver, integration
```

**The fix is architectural:** move the *entire* physics step to GPU.
Data stays on-device. CPU uploads control inputs once per frame,
readback body poses once per frame for rendering. No CPU stages
in the inner loop. No stale data between substeps.

## 2. Goal

A GPU physics pipeline where **every stage** of the physics step —
FK, CRBA, RNE, collision, constraint solve, integration — runs as
sequential compute dispatches in a single command buffer. The CPU
submits one command buffer per frame and reads back only what Bevy
needs for rendering.

```
Per frame:
  CPU → GPU:  ctrl, qfrc_applied, xfrc_applied       (~100 bytes)

Per substep (all on GPU, no readback):
  fk.wgsl             → body_xpos, body_xquat, geom_xpos
  crba.wgsl           → M (mass matrix)
  rne.wgsl            → qfrc_bias
  actuation.wgsl      → qfrc_actuator
  passive.wgsl        → qfrc_passive
  smooth.wgsl         → qfrc_smooth, qacc_smooth
  aabb.wgsl           → geom_aabb
  broadphase.wgsl     → pair_buffer
  narrowphase.wgsl    → contact_buffer
  assemble.wgsl       → efc_J, efc_D, efc_aref
  newton_solve.wgsl   → qacc
  integrate.wgsl      → qpos, qvel

After final substep:
  GPU → CPU:  qpos, qvel, ncon                        (~few KB)
```

**No split-step. No stale data. Every substep gets fresh FK, fresh M,
correct body poses.**

## 3. Reference architecture: MJX

MJX (MuJoCo's JAX backend) is the reference implementation. We adopt
its proven design and adapt it from JAX to wgpu/WGSL.

| Aspect | MJX | CortenForge GPU | Rationale |
|--------|-----|-----------------|-----------|
| Solver | Newton/CG (primal) | Newton (nv ≤ 60), CG (nv > 60) | Proven, 1–3 iterations |
| Solver variable | qacc | qacc | Primal = unconstrained optimization |
| Friction model | Both pyramidal + elliptic | Pyramidal first | Simpler GPU path |
| FK/CRBA/RNE | GPU (level-order tree scan) | GPU (level-order tree scan) | Full GPU, no split-step |
| Mass matrix | GPU Cholesky (dense/sparse) | GPU Cholesky in shared mem (dense) | nv ≤ 60 fits in 16KB |
| Batching | jax.vmap over environments | n_env dimension on all buffers | Architecture-ready from day 1 |
| Tree scan | jax.lax.scan + vmap | wgpu compute pass per tree level | Manual but equivalent |
| Data layout | JAX arrays (SoA by nature) | GPU-side SoA, CPU-side stays AoS | Convert at boundary |

**Key architectural decision:** GPU-side buffers are SoA (struct of
arrays) with an `n_env` leading dimension from day 1. With `n_env=1`,
this is single-environment interactive. The same shaders and buffer
layout support `n_env=1000` for batch RL — just wider buffers, same
dispatches.

## 4. Non-goals

- **Rewriting the CPU pipeline.** The CPU `Data::step()` pipeline is
  unchanged. The GPU pipeline is a parallel code path. CPU remains for
  correctness validation, platforms without GPU, and f64 reference.
- **GPU mesh collision / GJK-EPA.** Branchy, poor GPU utilization.
  Remain CPU-only.
- **Elliptic friction cones on GPU.** Pyramidal first. Elliptic cones
  require per-contact Hessian blocks and are a follow-up.
- **Implicit spring/damper on GPU.** The GPU path uses semi-implicit
  Euler. Implicit integrators (ImplicitSpringDamper, Implicit) that
  modify M remain CPU-only.
- **Sensors, energy, sleep on GPU.** These are diagnostic/bookkeeping
  stages that run once per frame on CPU after readback. Not
  performance-critical.

## 5. Fundamental operations

Every stage of the physics pipeline decomposes into 7 primitive
operations. The GPU pipeline implements each as a WGSL pattern:

| Operation | WGSL pattern | Used by |
|-----------|-------------|---------|
| **Tree scan (forward)** | One compute pass per depth level. All bodies at depth d in parallel. | FK, RNE forward |
| **Tree scan (backward)** | One compute pass per depth level (reverse). Atomic accumulate into parent. | CRBA, RNE backward |
| **Per-element map** | One thread per element. No dependencies. | AABB, integration, force map, actuation, passive |
| **Sparse MV product** | Per-row parallel. | J^T·D·J·x, force mapping |
| **Dense factorization** | Single workgroup, shared memory. | Newton Cholesky (nv ≤ 60), qacc_smooth solve |
| **Reduction** | Workgroup tree reduction + atomic. | Dot products, Hessian accumulation |
| **Atomic scatter** | atomicAdd to shared or global buffer. | Broadphase, contact emit, backward tree scan |

The **only fundamentally sequential operation** is the tree scan: O(depth)
sequential dispatches, each with O(bodies_per_level × n_env) parallelism.
Everything else is embarrassingly parallel.

## 6. GPU buffer architecture

### 6.1 Design principles

**SoA (struct of arrays):** All GPU buffers store one field across all
elements, not one element with all fields. This maximizes memory
coalescing when threads access the same field across different elements.

**n_env dimension:** Every per-state buffer has a leading `n_env`
dimension. For `n_env=1` (interactive), this is just an offset of 0.
For `n_env=1000` (batch), each thread computes
`env_id * stride + element_id`. Same shader, wider dispatch.

**f32 throughout:** All GPU physics uses f32. Conversion from f64
happens at the CPU→GPU boundary (upload) and GPU→CPU boundary
(readback). MJX proves f32 is sufficient for physics.

**Pre-allocated, fixed-size:** All buffers allocated at model creation.
No dynamic growth on GPU. Atomic counters track active elements
(contacts, pairs, constraints) within pre-allocated capacity.

### 6.2 Static buffers (uploaded once at model creation)

These encode the model structure. Shared across all environments.

**Tree structure:**

| Buffer | Size | Contents |
|---|---|---|
| `body_parent` | nbody × 4 | Parent body index (u32). body_parent[0] = 0 (world). |
| `body_depth` | nbody × 4 | Tree depth per body (u32). body_depth[0] = 0. |
| `body_jnt_adr` | nbody × 4 | Index of first joint for this body (u32). |
| `body_jnt_num` | nbody × 4 | Number of joints on this body (u32). |
| `body_dof_adr` | nbody × 4 | Index of first DOF for this body (u32). |
| `body_dof_num` | nbody × 4 | Number of DOFs on this body (u32). |
| `body_qpos_adr` | nbody × 4 | Index into qpos for this body (u32). |
| `body_pos` | nbody × 12 | Body position offset in parent frame (vec3<f32>). |
| `body_quat` | nbody × 16 | Body orientation offset in parent frame (vec4<f32>). |
| `body_ipos` | nbody × 12 | Inertial frame offset (vec3<f32>). |
| `body_iquat` | nbody × 16 | Inertial frame orientation (vec4<f32>). |
| `body_mass` | nbody × 4 | Body mass (f32). |
| `body_inertia` | nbody × 12 | Diagonal inertia in body frame (vec3<f32>). |
| `max_depth` | 4 | Maximum tree depth (u32). Controls dispatch count. |

**Joint structure:**

| Buffer | Size | Contents |
|---|---|---|
| `jnt_type` | njnt × 4 | Joint type enum: Free=0, Ball=1, Hinge=2, Slide=3 (u32). |
| `jnt_axis` | njnt × 12 | Joint axis in parent frame (vec3<f32>). |
| `jnt_pos` | njnt × 12 | Joint anchor in parent frame (vec3<f32>). |
| `jnt_body` | njnt × 4 | Body index for this joint (u32). |
| `jnt_qpos_adr` | njnt × 4 | Index into qpos (u32). |
| `jnt_dof_adr` | njnt × 4 | Index into qvel/qacc (u32). |
| `dof_body` | nv × 4 | Body index per DOF (u32). |
| `dof_parent` | nv × 4 | Parent DOF index for LDL sparsity (u32, 0xFFFFFFFF = none). |
| `jnt_armature` | njnt × 4 | Armature inertia per joint (f32). |

**Geometry:**

| Buffer | Size | Contents |
|---|---|---|
| `geom_type` | ngeom × 4 | GeomType enum (u32). |
| `geom_body` | ngeom × 4 | Body index per geom (u32). |
| `geom_pos` | ngeom × 12 | Geom offset in body frame (vec3<f32>). |
| `geom_quat` | ngeom × 16 | Geom orientation in body frame (vec4<f32>). |
| `geom_size` | ngeom × 12 | Geom dimensions for AABB (vec3<f32>). |
| `geom_shape_id` | ngeom × 4 | SDF grid index (u32). |
| `geom_friction` | ngeom × 12 | Per-geom friction [slide, torsion, roll] (vec3<f32>). |
| `sdf_grid[i]` | W×H×D × 4 | SDF distance values per shape (f32). |
| `sdf_meta[i]` | 32 | Grid dimensions, cell_size, origin. |

**Solver parameters:**

| Buffer | Size | Contents |
|---|---|---|
| `solver_params` | 128 | dt, gravity, solver_iterations, tolerance, nv, nq, nbody, ngeom, njnt, max_depth, condim, cone_type. |

**Actuator structure (for GPU actuation):**

| Buffer | Size | Contents |
|---|---|---|
| `actuator_trntype` | nu × 4 | Transmission type (u32). |
| `actuator_trnid` | nu × 8 | Transmission target [id0, id1] (u32 × 2). |
| `actuator_gear` | nu × 24 | Gear ratio (6 × f32). |
| `actuator_gainprm` | nu × 40 | Gain parameters (10 × f32). |
| `actuator_biasprm` | nu × 40 | Bias parameters (10 × f32). |

### 6.3 Per-environment state buffers (n_env × size)

These hold the simulation state. Each environment has its own copy.

**Generalized coordinates:**

| Buffer | Size per env | Contents |
|---|---|---|
| `qpos` | nq × 4 | Generalized positions (f32). |
| `qvel` | nv × 4 | Generalized velocities (f32). |
| `qacc` | nv × 4 | Generalized accelerations (f32). |
| `qacc_smooth` | nv × 4 | Unconstrained acceleration M⁻¹·qfrc_smooth (f32). |
| `ctrl` | nu × 4 | Control inputs (f32). Uploaded from CPU per frame. |

**Per-body computed state:**

| Buffer | Size per env | Contents |
|---|---|---|
| `body_xpos` | nbody × 12 | World-frame position (vec3<f32>). |
| `body_xquat` | nbody × 16 | World-frame quaternion (vec4<f32>). |
| `body_xmat` | nbody × 36 | World-frame rotation matrix (mat3x3<f32>). |
| `body_xipos` | nbody × 12 | COM position in world frame (vec3<f32>). |
| `body_cinert` | nbody × 40 | Spatial inertia in world frame (10 unique f32). |
| `body_crb` | nbody × 40 | Composite rigid body inertia (10 unique f32). |
| `body_cvel` | nbody × 24 | Body spatial velocity (6 × f32). |
| `body_cacc` | nbody × 24 | Body bias acceleration (6 × f32). |
| `body_cfrc` | nbody × 24 | Body bias force (6 × f32). |
| `subtree_mass` | nbody × 4 | Subtree total mass (f32). |
| `subtree_com` | nbody × 12 | Subtree COM position (vec3<f32>). |

**Per-geom computed state:**

| Buffer | Size per env | Contents |
|---|---|---|
| `geom_xpos` | ngeom × 12 | World-frame position (vec3<f32>). |
| `geom_xmat` | ngeom × 36 | World-frame rotation (mat3x3<f32>). |
| `geom_aabb` | ngeom × 24 | World AABB [min, max] (2 × vec3<f32>). |

**Force accumulators:**

| Buffer | Size per env | Contents |
|---|---|---|
| `qfrc_applied` | nv × 4 | User-applied forces (f32). Uploaded per frame. |
| `xfrc_applied` | nbody × 24 | Body-frame Cartesian forces (6 × f32). Uploaded per frame. |
| `qfrc_bias` | nv × 4 | Coriolis + gravity bias (f32). Written by RNE. |
| `qfrc_passive` | nv × 4 | Spring + damper forces (f32). Written by passive. |
| `qfrc_actuator` | nv × 4 | Actuator forces (f32). Written by actuation. |
| `qfrc_smooth` | nv × 4 | Total smooth force (f32). Written by smooth. |
| `qfrc_constraint` | nv × 4 | Constraint forces (f32). Written by map_forces. |

**Mass matrix (dense, nv ≤ 60):**

| Buffer | Size per env | Contents |
|---|---|---|
| `qM` | nv × nv × 4 | Mass matrix (f32). Written by CRBA. |
| `qM_factor` | nv × nv × 4 | Cholesky factor of M (f32). Written by smooth (for qacc_smooth). |

**Motion subspace (precomputed per substep by FK):**

| Buffer | Size per env | Contents |
|---|---|---|
| `cdof` | nv × 24 | Joint motion subspace in world frame (6 × f32 per DOF). |

**Constraint working set:**

| Buffer | Size per env | Contents |
|---|---|---|
| `pair_buffer` | max_pairs × 8 | Broadphase pairs [geom_a, geom_b] (u32 × 2). |
| `pair_count` | 4 | Active pair count (atomic u32). |
| `contact_buffer` | max_contacts × 48 | Contacts (see struct below). |
| `contact_count` | 4 | Active contact count (atomic u32). |
| `efc_J` | max_constraints × nv × 4 | Constraint Jacobian (f32). |
| `efc_D` | max_constraints × 4 | Constraint stiffness (f32). |
| `efc_aref` | max_constraints × 4 | Reference acceleration (f32). |
| `efc_type` | max_constraints × 4 | Constraint type enum (u32). |
| `efc_force` | max_constraints × 4 | Solver output forces (f32). |
| `constraint_count` | 4 | Active constraint count (atomic u32). |

**Contact struct (48 bytes):**
```wgsl
struct GpuContact {
    point:    vec3<f32>,   // World-space contact position
    depth:    f32,         // Penetration depth (≥ 0)
    normal:   vec3<f32>,   // World-space contact normal
    geom1:    u32,         // Geom index A
    friction: vec3<f32>,   // Mixed friction [slide, torsion, roll]
    geom2:    u32,         // Geom index B
};
```

### 6.4 Pre-allocation sizes

| Buffer | Max size | Rationale |
|---|---|---|
| `max_contacts` | 32,768 | ~100 SDF pairs × ~300 contacts. |
| `max_pairs` | 4,096 | 50 geoms → C(50,2) = 1225 worst case. 3× headroom. |
| `max_constraints` | 196,608 | 6 × max_contacts (pyramidal condim=4: 6 rows per contact). |

### 6.5 Memory budget

**Per-environment state (hockey scene: nbody=4, ngeom=4, nv=13, nq=16):**

| Category | Size |
|---|---|
| Generalized coords (qpos, qvel, qacc, etc.) | ~500 bytes |
| Per-body state (poses, inertias, velocities) | ~3 KB |
| Per-geom state (poses, AABBs) | ~300 bytes |
| Force accumulators | ~500 bytes |
| Mass matrix (2 copies) | ~1.4 KB |
| Constraint working set | ~13 MB (pre-allocated) |
| **Total per env** | **~13 MB** |

For `n_env=1`: 13 MB. For `n_env=1000`: 13 GB — feasible on 5070 Ti
(16 GB VRAM), though constraint pre-allocation would need tuning.

**Static buffers (shared across envs):** ~2 MB for SDF grids + model
structure. Negligible.

## 7. Tree scan primitive

The tree scan is the single most important GPU primitive in this spec.
FK, CRBA, and RNE all use it. Build it once, parameterize it.

### 7.1 Algorithm

```
Given: body_parent[nbody], body_depth[nbody], max_depth

Forward scan (root → leaves):
  for d in 0..=max_depth:
    dispatch: process all bodies where body_depth[body] == d
    each thread: read parent result → compute own result → write

Backward scan (leaves → root):
  for d in max_depth..=0:
    dispatch: process all bodies where body_depth[body] == d
    each thread: compute own result → atomicAdd into parent
```

Each depth level is one compute pass. wgpu guarantees a storage buffer
barrier between passes. No explicit synchronization needed.

### 7.2 WGSL implementation pattern

```wgsl
// Uniform: current depth level being processed
@group(0) @binding(0) var<uniform> params: ScanParams;

struct ScanParams {
    current_depth: u32,
    n_env: u32,
    nbody: u32,
    // ...
};

@compute @workgroup_size(64)
fn fk_forward(@builtin(global_invocation_id) gid: vec3<u32>) {
    let body_id = gid.x;
    let env_id  = gid.y;
    if (body_id >= params.nbody || env_id >= params.n_env) { return; }
    if (body_depth[body_id] != params.current_depth) { return; }

    let parent = body_parent[body_id];
    let env_offset = env_id * params.nbody;

    // Read parent's world pose (written in previous depth pass)
    let parent_xpos = body_xpos[env_offset + parent];
    let parent_xquat = body_xquat[env_offset + parent];

    // Compute this body's world pose
    // ... (joint-dependent FK logic)

    // Write result
    body_xpos[env_offset + body_id] = world_pos;
    body_xquat[env_offset + body_id] = world_quat;
}
```

**Dispatch (Rust side):**
```rust
for depth in 0..=max_depth {
    update_depth_uniform(depth);
    let mut pass = encoder.begin_compute_pass(&desc);
    pass.set_pipeline(&fk_pipeline);
    pass.set_bind_group(0, &fk_bind_group, &[]);
    // Dispatch enough threads to cover all bodies × all envs
    pass.dispatch_workgroups(
        ceil(nbody, 64),   // X: bodies
        n_env,             // Y: environments
        1,
    );
}
```

### 7.3 Parallelism analysis

| Tree shape | Depth | Bodies/level | Parallelism (n_env=1) | Parallelism (n_env=1000) |
|---|---|---|---|---|
| Flat (hockey) | 1 | all N at level 1 | N (embarrassingly parallel) | 1000×N |
| Serial chain (7-DOF arm) | 7 | 1 per level | 1 (purely sequential) | 1000 |
| Humanoid (30 bodies) | 5 | ~6 per level | ~6 | ~6000 |
| Many free bodies (100) | 1 | 100 at level 1 | 100 | 100,000 |

**Key insight:** For single-environment interactive (n_env=1), tree scan
parallelism is limited by bodies_per_level. For SDF physics scenes
(mostly free-floating bodies), this is embarrassingly parallel. For
articulated robots, it's sequential — but the tree stages are cheap
(~microseconds of arithmetic), so the sequential dispatches are bounded
by wgpu dispatch overhead (~5μs each), not compute.

**For n_env > 1 (batch RL):** Even a serial 7-DOF arm has 1000×
parallelism at every level. GPU is fully saturated.

## 8. Shader specifications

### 8.1 `fk.wgsl` — Forward kinematics (NEW)

**Estimated:** ~250 lines WGSL.
**Pattern:** Tree scan (forward), one pass per depth level.
**Reference:** MJX `kinematics.py:fwd_position()`, `scan.py:body_tree()`

**Per body (at its depth level):**
1. Read parent world pose: `(parent_xpos, parent_xquat)`
2. Apply body offset: `pos = parent_xpos + rotate(parent_xquat, body_pos[i])`
3. Apply body orientation: `quat = parent_xquat × body_quat[i]`
4. Apply joint transformation (depends on joint type):
   - **Free:** pos = qpos[0..3], quat = qpos[3..7] (absolute, not relative)
   - **Hinge:** quat = quat × axis_angle(jnt_axis, qpos[jnt_qpos_adr])
   - **Ball:** quat = quat × qpos[jnt_qpos_adr..+4]
   - **Slide:** pos += rotate(quat, jnt_axis) × qpos[jnt_qpos_adr]
5. Compute rotation matrix: `xmat = quat_to_mat3(quat)`
6. Compute inertial frame: `xipos, ximat` from body offsets
7. Compute geom poses: `geom_xpos, geom_xmat` for all geoms on this body
8. Compute motion subspace `cdof` for each DOF on this body

**World body (depth 0):** Identity pose, no joints. Written as
initialization before the scan starts (or hardcoded).

**Subtree COM (backward pass after FK):** A separate backward tree scan
accumulates `subtree_mass` and `subtree_com` from leaves to root.
Used by RNE gravity computation.

### 8.2 `crba.wgsl` — Composite Rigid Body Algorithm (NEW)

**Estimated:** ~300 lines WGSL.
**Pattern:** Backward tree scan + per-DOF parallel M assembly.
**Reference:** MJX `smooth.py:crb()`, `support.py:make_m()`

**Phase 1: Initialize composite inertias (per-body parallel)**
```
crb[body_id] = cinert[body_id]   // copy from FK
```

**Phase 2: Backward tree scan (leaves → root)**
```
for d in max_depth..=1:
  per body at depth d:
    parent = body_parent[body_id]
    atomicAdd(crb[parent], shift_inertia(crb[body_id], xpos[body_id] - xpos[parent]))
```

The `shift_inertia` applies the parallel axis theorem to transport
the child's composite inertia to the parent's origin.

**Phase 3: Build mass matrix (per-DOF parallel)**
```
for each DOF i:
  body_i = dof_body[i]
  // Diagonal: M[i,i] = cdof[i]^T · crb[body_i] · cdof[i] + armature[i]
  // Off-diagonal: walk dof_parent chain
  j = dof_parent[i]
  while j != NONE:
    M[j,i] = cdof[j]^T · crb[body_i] · cdof[i]
    M[i,j] = M[j,i]  // symmetric
    j = dof_parent[j]
```

For flat trees (hockey), dof_parent depth is 1. M assembly is
embarrassingly parallel — each DOF's column is independent.

**Phase 4: Cholesky factorization (single workgroup, shared memory)**
```
M_factor = cholesky(M)   // nv×nv in shared memory
```
For nv=13: ~366 flops. For nv=60: ~36K flops. Single thread within
the workgroup. Needed by `smooth.wgsl` for qacc_smooth.

### 8.3 `rne.wgsl` — Recursive Newton-Euler (NEW)

**Estimated:** ~300 lines WGSL.
**Pattern:** Forward tree scan + backward tree scan.
**Reference:** MJX `smooth.py:rne()`

**Phase 1: Gravity (per-joint parallel, no tree dependency)**
```
for each joint:
  qfrc_bias[dof] += gravity_torque(subtree_mass, subtree_com, jnt_axis)
```

**Phase 2a: Forward scan — bias accelerations (root → leaves)**
```
for d in 0..=max_depth:
  per body at depth d:
    cacc[body] = transport(cacc[parent]) + velocity_product(cvel[body])
```

**Phase 2b: Backward scan — bias forces (leaves → root)**
```
for d in max_depth..=0:
  per body at depth d:
    cfrc[body] = cinert[body] · cacc[body] + gyroscopic(cvel[body], cinert[body])
    atomicAdd(cfrc[parent], transport(cfrc[body]))
```

**Phase 2c: Project to joint space (per-DOF parallel)**
```
for each DOF:
  qfrc_bias[dof] += dot(cdof[dof], cfrc[dof_body])
```

### 8.4 `actuation.wgsl` — Actuator forces (NEW)

**Estimated:** ~150 lines WGSL.
**Pattern:** Per-actuator parallel map.

**Per actuator:**
1. Read `ctrl[i]` (uploaded from CPU)
2. Compute force from gain/bias: `force = gain(ctrl) + bias(qpos, qvel)`
3. Apply transmission: `qfrc_actuator[dof] += gear × force`

**Scope:** Covers the common actuator types (motor, position, velocity).
Complex actuators (muscle, cylinder) remain CPU-only — the GPU path
asserts supported actuator types at pipeline creation.

### 8.5 `passive.wgsl` — Passive forces (NEW)

**Estimated:** ~100 lines WGSL.
**Pattern:** Per-DOF parallel map.

**Per DOF:**
1. Gravity: `qfrc_passive[dof] = gravity_force(dof)` (from subtree COM)
2. Joint damping: `qfrc_passive[dof] -= damping[dof] × qvel[dof]`
3. Joint stiffness: `qfrc_passive[dof] -= stiffness[dof] × (qpos - qpos_spring)`

**Scope:** Covers joint-level springs/dampers. Fluid forces and tendon
passive forces remain CPU-only for the initial implementation.

### 8.6 `smooth.wgsl` — Smooth forces + qacc_smooth (NEW)

**Estimated:** ~120 lines WGSL.
**Pattern:** Per-DOF parallel + single-workgroup Cholesky solve.

**Phase 1: Compute qfrc_smooth (per-DOF parallel)**
```
qfrc_smooth[dof] = qfrc_applied[dof] + qfrc_actuator[dof]
                 + qfrc_passive[dof] - qfrc_bias[dof]
```

Plus xfrc_applied projection: for each body with non-zero xfrc,
project body-frame [torque, force] into joint space via cdof.

**Phase 2: Solve qacc_smooth = M⁻¹ · qfrc_smooth (single workgroup)**

Uses the Cholesky factor from crba.wgsl:
```
qacc_smooth = cholesky_solve(M_factor, qfrc_smooth)
```
Forward + backward substitution in shared memory. For nv=13: ~338 flops.

### 8.7 `aabb.wgsl` — Bounding box computation (NEW)

**Estimated:** ~80 lines WGSL.
**Pattern:** Per-geom parallel map.

Per geom: read `geom_xpos`, `geom_xmat`, `geom_size` → compute world
AABB with margin → write `geom_aabb`. Uses fresh body poses from FK
(no stale data).

### 8.8 `broadphase.wgsl` — Spatial hash broadphase (NEW)

**Estimated:** ~300 lines WGSL (3 passes).
**Reference:** GPU Gems 3, Chapter 32.

Pass 1: Hash + count. Pass 2: Blelloch prefix sum. Pass 3: Scatter +
AABB overlap test → emit pairs via atomic.

### 8.9 `trace_surface.wgsl` — SDF-SDF narrowphase (EXISTS)

**Status:** Done, tested, 267 lines. Workgroup 8×8×4.
**Change:** Output feeds into assemble.wgsl instead of CPU readback.
Contact struct extended to 48 bytes.

### 8.10 `sdf_plane.wgsl` — SDF-plane narrowphase (NEW)

**Estimated:** ~150 lines WGSL. Workgroup 8×8×4.
Per grid cell: surface filter → world transform → plane distance →
emit contact. Simpler than SDF-SDF (no second grid).

### 8.11 `assemble.wgsl` — Constraint assembly (NEW)

**Estimated:** ~350 lines WGSL.
**Pattern:** Per-contact parallel map.
**Reference:** MJX `constraint.py:make_constraint()`

Per contact: compute contact frame, emit pyramidal rows (6 rows for
condim=4), compute impedance/stabilization, write efc_J/D/aref.

**Jacobian for free bodies:** 12 non-zero entries per row (6 per body).
Uses fresh body poses from FK for correct contact-to-COM offsets.

**diagApprox:** `J · M⁻¹ · J^T` computed per-body from `body_mass` and
`body_inertia` — no full M⁻¹ needed for the diagonal approximation.

### 8.12 `newton_solve.wgsl` — Primal Newton solver (NEW)

**Estimated:** ~500 lines WGSL.
**Pattern:** Single workgroup, shared memory.
**Reference:** MJX `solver.py:_newton()`

Primal formulation: minimize cost(qacc) = Gauss term + constraint
penalties. Newton iterations: evaluate cost + gradient + Hessian →
Cholesky solve → line search → update qacc. 1–3 iterations.

H = M + J^T · diag(D · active) · J (nv × nv, in shared memory).
256 threads cooperate on constraint evaluation and H accumulation.
Thread 0 does Cholesky. Fits in shared memory for nv ≤ 60.

**For nv > 60:** Switch to CG solver (see §12).

### 8.13 `map_forces.wgsl` — Force mapping (NEW)

**Estimated:** ~60 lines WGSL.
**Pattern:** Per-DOF parallel reduction.

`qfrc_constraint[dof] = Σ_row efc_force[row] × efc_J[row][dof]`

### 8.14 `integrate.wgsl` — Semi-implicit Euler (NEW)

**Estimated:** ~120 lines WGSL.
**Pattern:** Per-body parallel map.
**Reference:** MJX `math.py:quat_integrate()`

Per body: `qvel += dt × qacc`, then position integration with
exponential map for quaternions. Matches CPU `integrate_quaternion()`.

## 9. Command buffer structure

One command buffer per frame, encoding all substeps:

```rust
let mut encoder = device.create_command_encoder();

// Upload per-frame: ctrl, qfrc_applied, xfrc_applied (~100 bytes)
upload_controls(&encoder, &staging, &gpu);

for _substep in 0..num_substeps {
    // Reset atomic counters
    encoder.clear_buffer(&pair_count, 0, 4);
    encoder.clear_buffer(&contact_count, 0, 4);
    encoder.clear_buffer(&constraint_count, 0, 4);

    // FK: level-order forward scan (max_depth + 1 passes)
    for depth in 0..=max_depth {
        update_uniform(&encoder, depth);
        compute_pass(&encoder, &fk_pipeline, ceil(nbody, 64), n_env);
    }
    // Subtree COM: backward scan
    for depth in (0..=max_depth).rev() {
        update_uniform(&encoder, depth);
        compute_pass(&encoder, &subtree_com_pipeline, ceil(nbody, 64), n_env);
    }

    // CRBA: init (parallel) + backward scan + M assembly + Cholesky
    compute_pass(&encoder, &crba_init_pipeline, ceil(nbody, 64), n_env);
    for depth in (0..=max_depth).rev() {
        update_uniform(&encoder, depth);
        compute_pass(&encoder, &crba_accum_pipeline, ceil(nbody, 64), n_env);
    }
    compute_pass(&encoder, &crba_build_m_pipeline, ceil(nv, 64), n_env);
    compute_pass(&encoder, &crba_factor_pipeline, 1, n_env);  // Cholesky

    // RNE: gravity (parallel) + forward scan + backward scan + project
    compute_pass(&encoder, &rne_gravity_pipeline, ceil(njnt, 64), n_env);
    for depth in 0..=max_depth {
        update_uniform(&encoder, depth);
        compute_pass(&encoder, &rne_forward_pipeline, ceil(nbody, 64), n_env);
    }
    for depth in (0..=max_depth).rev() {
        update_uniform(&encoder, depth);
        compute_pass(&encoder, &rne_backward_pipeline, ceil(nbody, 64), n_env);
    }
    compute_pass(&encoder, &rne_project_pipeline, ceil(nv, 64), n_env);

    // Actuation + passive + smooth
    compute_pass(&encoder, &actuation_pipeline, ceil(nu, 64), n_env);
    compute_pass(&encoder, &passive_pipeline, ceil(nv, 64), n_env);
    compute_pass(&encoder, &smooth_pipeline, 1, n_env);  // includes Cholesky solve

    // Collision
    compute_pass(&encoder, &aabb_pipeline, ceil(ngeom, 64), n_env);
    // broadphase (3 passes) ...
    // narrowphase (per-pair dispatches) ...

    // Constraint solve
    compute_pass(&encoder, &assemble_pipeline, ceil(max_contacts, 256), n_env);
    for _iter in 0..solver_iterations {
        compute_pass(&encoder, &newton_pipeline, 1, n_env);  // single workgroup per env
    }
    compute_pass(&encoder, &map_forces_pipeline, ceil(nv, 64), n_env);

    // Integration
    compute_pass(&encoder, &integrate_pipeline, ceil(nbody, 64), n_env);
}

// Readback: final qpos, qvel, ncon
encoder.copy_buffer_to_buffer(&gpu_qpos, &staging_qpos, ...);
queue.submit([encoder.finish()]);
staging_qpos.slice(..).map_async(MapMode::Read, ...);
device.poll(Maintain::Wait);
```

**Dispatch count per substep (hockey, max_depth=1):**

| Stage | Dispatches |
|---|---|
| FK (2 levels + subtree COM 2 levels) | 4 |
| CRBA (init + 2 levels + build M + Cholesky) | 5 |
| RNE (gravity + 2 forward + 2 backward + project) | 6 |
| Actuation + passive + smooth | 3 |
| AABB + broadphase (3) + narrowphase | 5 |
| Assemble + Newton (×2 iter) + map_forces | 4 |
| Integration | 1 |
| **Total** | **~28** |

At ~5μs per dispatch: ~140μs overhead. Well within the 1ms/substep
budget. The compute work per dispatch is small for 4 bodies but grows
with scene complexity.

## 10. Integration with sim-core

### 10.1 New trait: `GpuPhysicsPipeline`

```rust
pub trait GpuPhysicsPipeline: Send + Sync {
    /// Run N substeps on GPU. Returns final state for rendering.
    fn step_gpu(
        &self,
        model: &Model,
        data: &Data,           // Reads: qpos, qvel, ctrl, qfrc_applied, xfrc_applied
        num_substeps: u32,
    ) -> GpuStepResult;
}

pub struct GpuStepResult {
    pub qpos: Vec<f64>,     // f32→f64 widened
    pub qvel: Vec<f64>,
    pub ncon: usize,
}
```

### 10.2 Pipeline creation

```rust
pub fn enable_gpu_pipeline(model: &mut Model) -> Result<(), GpuError> {
    // Validate: model must be GPU-compatible
    assert!(model.integrator == Integrator::Euler, "GPU requires Euler");
    assert!(model.nv <= 60, "GPU Newton requires nv ≤ 60");
    // Assert supported joint types, actuator types, etc.

    let ctx = GpuContext::new()?;
    let pipeline = GpuPipelineImpl::new(&ctx, model)?;
    // Uploads all static buffers, creates all compute pipelines
    model.gpu_pipeline = Some(Arc::new(pipeline));
    Ok(())
}
```

### 10.3 Dispatch in `Data::step()`

```rust
pub fn step(&mut self, model: &Model) -> Result<(), StepError> {
    if let Some(gpu) = &model.gpu_pipeline {
        let result = gpu.step_gpu(model, self, model.gpu_substeps);
        self.qpos.copy_from_slice(&result.qpos);
        self.qvel.copy_from_slice(&result.qvel);
        self.ncon = result.ncon;
        self.time += model.timestep * model.gpu_substeps as f64;

        // Re-run CPU FK for Bevy rendering (sensor/sleep need body poses)
        self.forward_pos_vel(model, true);
        return Ok(());
    }
    // CPU fallback (unchanged)
    // ...
}
```

### 10.4 Relationship to existing code

| Existing code | Role in GPU pipeline |
|---|---|
| `trace_surface.wgsl` | Directly reused as narrowphase SDF-SDF |
| `GpuContext` (device, queue) | Shared across all pipeline stages |
| `GpuSdfGrid` (grid upload) | Reused for static SDF buffers |
| CPU `Data::step()` | Unchanged. CPU fallback + validation reference |
| `integrate_without_velocity()` | Not needed — full GPU handles everything |
| `GpuSdfCollision` trait | Deprecated by full pipeline. Kept for standalone testing |

## 11. Cleanup of current half-GPU path

1. **Phase 1 (now):** Keep `GpuSdfCollision` trait as-is. Working, tested.
2. **Phase 2:** Add `GpuPhysicsPipeline` as parallel path. Both coexist.
3. **Phase 3:** Deprecate `GpuSdfCollision` dispatch in sdf_collide.rs.
   Move to `gpu_standalone` module for test/validation only.
4. **Phase 4:** Remove per-step readback path. `GpuSdfCollision` remains
   for unit tests only.

## 12. Validation strategy

Each shader group is validated independently:

| Shader group | Validation method |
|---|---|
| FK (fk.wgsl) | Compare GPU body_xpos/xquat vs CPU mj_fwd_position() |
| CRBA (crba.wgsl) | Compare GPU qM vs CPU mj_crba() |
| RNE (rne.wgsl) | Compare GPU qfrc_bias vs CPU mj_rne() |
| Smooth (smooth.wgsl) | Compare GPU qacc_smooth vs CPU compute_qacc_smooth() |
| Collision (aabb + broadphase + narrowphase) | Compare GPU contacts vs CPU mj_collision() |
| Assembly (assemble.wgsl) | Compare GPU efc_J/D/aref vs CPU (pyramidal mode) |
| Solver (newton_solve.wgsl) | Compare GPU qacc vs CPU Newton output |
| Integration (integrate.wgsl) | Compare GPU qpos/qvel vs CPU integrate() |

**End-to-end:** Run hockey scene GPU vs CPU for 5 seconds. Body
trajectories should remain bounded (f32 drift is expected, divergence
is not). Pyramidal vs elliptic friction causes expected behavioral
differences.

**Tree scan validation:** Run FK on a 7-body chain (depth 7). Compare
GPU body poses against CPU FK at each level. Validates the level-order
scan is correct for non-flat trees.

## 13. Performance targets

| Metric | Target | Measurement |
|---|---|---|
| GPU substep (hockey, n_env=1) | < 1ms | wgpu timestamp queries |
| GPU substep (100 bodies, n_env=1) | < 2ms | wgpu timestamp queries |
| GPU substep (hockey, n_env=100) | < 2ms | wgpu timestamp queries |
| CPU→GPU transfer (per frame) | < 0.05ms | Staging buffer map |
| GPU→CPU readback (per frame) | < 0.05ms | Staging buffer map |
| 90 Hz frame budget | < 11ms total | Frame timer |
| Newton iterations | 1–3 | Solver convergence log |

## 14. Implementation order

1. **`fk.wgsl` + tree scan primitive** — Foundation. Validates
   level-order dispatch, body pose computation, n_env dimension.
   Test against CPU FK for flat tree (hockey) and chain (7-DOF arm).

2. **`crba.wgsl`** — Backward tree scan + M assembly + Cholesky.
   Validates backward scan with atomic accumulation. Compare M against
   CPU CRBA.

3. **`rne.wgsl`** — Forward + backward scan (reuses tree scan).
   Compare qfrc_bias against CPU RNE.

4. **`actuation.wgsl` + `passive.wgsl` + `smooth.wgsl`** — Per-element
   maps + Cholesky solve. Validates qacc_smooth against CPU.

5. **`integrate.wgsl`** — Per-body map. Validates qpos/qvel update.
   At this point: full FK→CRBA→RNE→smooth→integrate loop works on GPU.
   Can run gravity-only simulation (no contacts).

6. **`sdf_plane.wgsl`** — Completes narrowphase. Builds on existing
   trace_surface.wgsl patterns.

7. **`aabb.wgsl` + `broadphase.wgsl`** — Collision detection pipeline.
   Validates spatial hashing + pair emission.

8. **`assemble.wgsl`** — Constraint assembly. Validates pyramidal
   Jacobian computation on GPU.

9. **`newton_solve.wgsl`** — Primal solver. The hardest shader.
   Validates against CPU Newton.

10. **`map_forces.wgsl`** — Force mapping. Simple J^T·f reduction.

11. **Pipeline orchestration** — Chain all shaders in single command
    buffer. Implement `GpuPhysicsPipeline` trait. End-to-end validation.

12. **Cleanup** — Deprecate half-GPU dispatch.

## 15. Risks

| Risk | Mitigation |
|---|---|
| Tree scan dispatch overhead dominates | Hockey has depth 1 (2 passes). Overhead is ~10μs. Profile early. |
| Atomic accumulation contention in backward scan | At depth 1 (flat tree), all children atomicAdd into world body. Use f32 atomics (supported by wgpu). For deep trees, contention is low (few children per parent). |
| Newton Cholesky fails (non-SPD H) | Regularize: H += ε·I. Fall back to CPU PGS for that frame. |
| f32 precision | MJX proves f32 Newton works. Regularization in H diagonal. Kahan summation in reductions. |
| nv > 60 exceeds shared memory for H | CG solver (iterative, no dense H). Not needed for hockey. |
| Complex actuators not supported on GPU | Assert supported types at pipeline creation. Unsupported → CPU fallback. |
| n_env > 1 memory pressure | Tune max_contacts/max_constraints per use case. 13 MB/env is baseline. |
| wgpu atomicAdd f32 not available | Use atomicAdd u32 with float reinterpretation, or per-thread local accumulation + workgroup reduction. |

## 16. Future extensions

- **CG solver for nv > 60:** Iterative, no dense Hessian. Needs M·v
  product (streamable) and J^T·D·J·v (sparse). MJX uses CG as
  alternative solver.

- **Elliptic friction cones:** Add cone Hessian blocks to Newton H.
  Per-contact dim×dim blocks via J^T·C_m·J. MJX supports both.

- **Multi-environment batching:** Set n_env > 1. Same shaders, wider
  dispatch. Requires: batch-friendly contact allocation, per-env
  constraint counts.

- **SoA refactor of CPU Data struct:** Align CPU and GPU layouts for
  zero-copy upload. Eliminates f64→f32 conversion at boundary.

- **Noslip post-processor:** Secondary Jacobi pass on friction rows
  to eliminate residual slip. Matches CPU noslip_postprocess().

- **Implicit integrators on GPU:** Modify M in crba.wgsl to include
  h·D + h²·K terms. Requires tendon K/D on GPU.

- **GPU sensors:** Position/velocity/acceleration sensors on GPU.
  Eliminates post-readback CPU FK for sensor evaluation.
