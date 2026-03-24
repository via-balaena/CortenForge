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
  fk.wgsl             → body_xpos, body_xquat, body_cinert, geom_xpos, cdof
  crba.wgsl           → M (mass matrix)
  velocity_fk.wgsl    → body_cvel (spatial velocities from qvel)
  rne.wgsl            → qfrc_bias (gravity + Coriolis + gyroscopic)
  actuation.wgsl      → qfrc_actuator
  passive.wgsl        → qfrc_passive (springs, dampers — NOT gravity)
  smooth.wgsl         → qfrc_smooth, qacc_smooth
  aabb.wgsl           → geom_aabb
  broadphase.wgsl     → pair_buffer
  narrowphase.wgsl    → contact_buffer
  assemble.wgsl       → efc_J, efc_D, efc_aref
  newton_solve.wgsl   → qacc, efc_force
  map_forces.wgsl     → qfrc_constraint
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
| **Tree scan (forward)** | One compute pass per depth level. All bodies at depth d in parallel. | FK, velocity FK, RNE forward |
| **Tree scan (backward)** | One compute pass per depth level (reverse). Accumulate into parent via workgroup reduction (not raw f32 atomics — see §7.3 note). | CRBA, RNE backward, subtree COM |
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

**Packed structs, not SoA:** The original design called for SoA (struct
of arrays) — one buffer per field. In practice, WebGPU's
`maxStorageBuffersPerShaderStage` default of 8 makes SoA infeasible for
model data (~18 fields). **Session 1 established the actual pattern:**
pack related per-element fields into struct arrays (AoS on GPU).

This is the standard approach in production GPU physics (MJX uses JAX
pytrees which are effectively the same). Memory coalescing is preserved
because all threads at a given depth level access the same struct fields
in sequence.

See `gpu-physics-pipeline/LIMITATIONS.md` §1 for full rationale.

**n_env dimension:** Every per-state buffer has a leading `n_env`
dimension. For `n_env=1` (interactive), this is just an offset of 0.
For `n_env=1000` (batch), each thread computes
`env_id * stride + element_id`. Same shader, wider dispatch.

**Note:** Session 1 hardcodes `n_env=1`. Batch support (n_env > 1)
requires adding env-offset math to qpos indexing and CAS atomics for
backward scans. Deferred to a future session.

**f32 throughout:** All GPU physics uses f32. Conversion from f64
happens at the CPU→GPU boundary (upload) and GPU→CPU boundary
(readback). MJX proves f32 is sufficient for physics.

**Pre-allocated, fixed-size:** All buffers allocated at model creation.
No dynamic growth on GPU. Atomic counters track active elements
(contacts, pairs, constraints) within pre-allocated capacity.

**vec4 alignment:** All vec3 fields are padded to vec4 (w=0) for
16-byte alignment. Cinert uses 12 floats (3×vec4) instead of 10, and
cdof uses 8 floats (2×vec4) instead of 6. The wasted padding is
negligible and enables clean `array<vec4<f32>>` access in WGSL.

### 6.2 Static buffers (uploaded once at model creation)

These encode the model structure. Shared across all environments.

**Implementation note (Session 1):** Static model data is packed into
3 GPU struct arrays, not individual buffers:

| GPU buffer | Rust type | WGSL type | Added |
|---|---|---|---|
| `bodies` | `array<BodyModelGpu>` | `array<BodyModel>` | Session 1 |
| `joints` | `array<JointModelGpu>` | `array<JointModel>` | Session 1 |
| `geoms` | `array<GeomModelGpu>` | `array<GeomModel>` | Session 1 (extended Session 4) |
| `dofs` | `array<DofModelGpu>` | `array<DofModel>` | Session 2 |
| `sdf_values` | `array<f32>` | `array<f32>` | Session 4 |
| `sdf_metas` | `array<SdfMetaGpu>` | `array<SdfMeta>` | Session 4 |
| `body_invweight0` | `array<[f32; 4]>` | `array<vec4<f32>>` | Session 5 |

**BodyModel struct** (112 bytes, 16-byte aligned):

| Field | Type | Notes |
|---|---|---|
| `parent` | u32 | Parent body index. body 0 → 0 (self-referential world). |
| `depth` | u32 | Tree depth. Computed at upload from `body_parent`. |
| `jnt_adr` | u32 | Index of first joint for this body. |
| `jnt_num` | u32 | Number of joints on this body. |
| `dof_adr` | u32 | Index of first DOF for this body. |
| `dof_num` | u32 | Number of DOFs on this body. |
| `mocap_id` | u32 | Mocap index (0xFFFFFFFF = not mocap). |
| `_pad` | u32 | Alignment padding. |
| `pos` | vec4\<f32\> | Body position offset in parent frame (w=0). |
| `quat` | vec4\<f32\> | Body orientation offset in parent frame (x,y,z,w). |
| `ipos` | vec4\<f32\> | Inertial frame offset (w=0). |
| `iquat` | vec4\<f32\> | Inertial frame orientation (x,y,z,w). |
| `inertia` | vec4\<f32\> | xyz = diagonal inertia in body frame, w = mass. |

**JointModel struct** (48 bytes, 16-byte aligned):

| Field | Type | Notes |
|---|---|---|
| `jtype` | u32 | Joint type: Free=0, Ball=1, Hinge=2, Slide=3. |
| `qpos_adr` | u32 | Index into qpos. |
| `dof_adr` | u32 | Index into qvel/qacc. |
| `body_id` | u32 | Body that owns this joint (`model.jnt_body[j]`). Added Session 3 (was `_pad`). |
| `axis` | vec4\<f32\> | Joint axis in parent frame (w=0). |
| `pos` | vec4\<f32\> | Joint anchor in parent frame (w=0). |

**GeomModel struct** (96 bytes, 16-byte aligned) — extended in Session 4:

| Field | Type | Notes |
|---|---|---|
| `body_id` | u32 | Body index for this geom. |
| `geom_type` | u32 | GeomType enum (Plane=0..Sdf=8). Session 4. |
| `contype` | u32 | Collision type bitmask. Session 4. |
| `conaffinity` | u32 | Collision affinity bitmask. Session 4. |
| `pos` | vec4\<f32\> | Geom offset in body frame (w=0). |
| `quat` | vec4\<f32\> | Geom orientation in body frame (x,y,z,w). |
| `size` | vec4\<f32\> | xyz = type-specific dims, w = bounding radius. Session 4. |
| `friction` | vec4\<f32\> | x=slide, y=torsion, z=roll, w=0. Session 4. |
| `sdf_meta_idx` | u32 | Index into `sdf_metas` (0xFFFFFFFF = not SDF). Session 4. |
| `condim` | u32 | Contact dimensionality (1, 3, 4, or 6). Session 4. |
| `_pad` | u32[2] | Alignment padding. |

**Note (Session 4):** The GeomModel struct was extended from 48 to 96 bytes
to pack collision-related fields (type, contype, conaffinity, size, friction,
sdf_meta_idx) into the same struct instead of separate buffers. All shaders
that reference `array<GeomModel>` (currently `fk.wgsl`, `aabb.wgsl`,
`sdf_sdf_narrow.wgsl`, `sdf_plane_narrow.wgsl`) must use the 96-byte struct
definition for correct array stride.

**DofModel struct** (16 bytes, 16-byte aligned) — added in Session 2:

| Field | Type | Notes |
|---|---|---|
| `body_id` | u32 | Body that owns this DOF (`model.dof_body[i]`). |
| `parent` | u32 | Parent DOF in kinematic tree (0xFFFFFFFF = none). |
| `armature` | f32 | Combined `jnt_armature[jnt] + dof_armature[i]`, pre-computed at upload. |
| `_pad` | u32 | Alignment padding. |

| GPU buffer | Rust type | WGSL type | Bind group |
|---|---|---|---|
| `dofs` | `array<DofModelGpu>` | `array<DofModel>` | CRBA Group 1, binding 1 |

**SDF grid data** (Session 4) — unified buffer approach:

| Buffer | Size | Contents |
|---|---|---|
| `sdf_values` | Σ(W×H×D) × 4 | All SDF grids concatenated (f32). |
| `sdf_metas` | nshape × 32 | Per-shape metadata: `SdfMeta` (width, height, depth, cell_size, origin, values_offset). |

Each narrowphase shader reads `sdf_metas[idx]` to get grid dimensions and
the offset into `sdf_values` where that shape's data starts. This replaces
per-grid buffer bindings with dynamic indexing into one large buffer.

**SdfMeta struct** (32 bytes, 16-byte aligned):

| Field | Type | Notes |
|---|---|---|
| `width` | u32 | Grid X samples. |
| `height` | u32 | Grid Y samples. |
| `depth` | u32 | Grid Z samples. |
| `cell_size` | f32 | Uniform cell size (meters). |
| `origin` | vec3\<f32\> | Grid minimum corner (local frame). |
| `values_offset` | u32 | Element offset into `sdf_values` buffer. |

**Per-dispatch uniform parameters (separate structs for separate concerns):**

| Buffer | Size | Used by | Contents |
|---|---|---|---|
| `FkParams` | 32 | FK, CRBA, velocity FK | current_depth, nbody, njnt, ngeom, nv, n_env, nq, _pad. |
| `PhysicsParams` | 48 | RNE, smooth, integrate | gravity (vec4), timestep, nbody, njnt, nv, nq, n_env, current_depth, nu. |
| `AabbParams` | 16 | AABB | ngeom, _pad[3]. Session 4. |
| `NarrowphaseParams` | 48 | SDF-SDF, SDF-plane narrowphase | geom1, geom2, src/dst sdf_meta_idx, surface_threshold, margin, flip_normal, friction. Session 4. |

`FkParams` (Session 1) serves kinematics shaders. `PhysicsParams` (Session 3)
adds gravity and timestep for dynamics shaders. `NarrowphaseParams` (Session 4)
is written per-pair dispatch — each narrowphase invocation gets its own pair's
metadata via `queue.write_buffer` before the dispatch.

**Per-body inverse weight** (Session 5) — for bodyweight diagonal approximation:

| Buffer | Size | Contents |
|---|---|---|
| `body_invweight0` | nbody × 16 | Per-body `vec4<f32>`: x=translational (1/mass), y=rotational (3/trace(I)), z=0, w=0. |

Used by `assemble.wgsl` for O(1) per-row diagApprox: `bw = invweight0[b1].x + invweight0[b2].x`.
World body (id=0) must have invweight0 = (0, 0, 0, 0).

**Constraint solve parameters** (Session 5) — separate uniforms for assembly and solver:

| Buffer | Rust type | Size | Used by |
|---|---|---|---|
| `assembly_params` | `AssemblyParams` | 64 | `assemble.wgsl` — nv, timestep, solref, solimp, impratio, max counts. |
| `solver_params` | `SolverParams` | 32 | `newton_solve.wgsl`, `map_forces.wgsl` — nv, max_iter, tolerance, meaninertia. |

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
| `mocap_pos` | nmocap × 12 | Mocap body positions (vec3<f32>). Uploaded per frame. |
| `mocap_quat` | nmocap × 16 | Mocap body orientations (vec4<f32>). Uploaded per frame. |

**Per-body computed state:**

| Buffer | Size per env | Contents |
|---|---|---|
| `body_xpos` | nbody × 16 | World-frame position (vec4<f32>, w=0). |
| `body_xquat` | nbody × 16 | World-frame quaternion (vec4<f32>, x,y,z,w). |
| `body_xipos` | nbody × 16 | COM position in world frame (vec4<f32>, w=0). |
| `body_cinert` | nbody × 48 | Spatial inertia (12 f32 = 3×vec4). See §8.1 for layout. |
| `body_crb` | nbody × 48 | Composite rigid body inertia (same 12-float format). Written by CRBA. |
| `body_cvel` | nbody × 32 | Body spatial velocity (2×vec4: [ω,0; v,0]). Written by velocity_fk. |
| `body_cacc` | nbody × 24 | Body bias acceleration (6 × f32). |
| `body_cfrc` | nbody × 24 | Body bias force (6 × f32). |
| `subtree_mass` | nbody × 4 | Subtree total mass (f32). |
| `subtree_com` | nbody × 16 | Subtree COM position (vec4<f32>, w=0). |

**Per-geom computed state:**

| Buffer | Size per env | Contents |
|---|---|---|
| `geom_xpos` | ngeom × 16 | World-frame position (vec4<f32>, w=0). |
| `geom_xmat` | ngeom × 48 | World-frame rotation (3×vec4<f32>, column-major). |
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
| `cdof` | nv × 32 | Joint motion subspace in world frame (2×vec4 = 8 f32 per DOF; 6 used, 2 padding). |

**Collision output (Session 4):**

| Buffer | Size per env | Contents |
|---|---|---|
| `geom_aabb` | ngeom × 32 | World AABB per geom: 2×vec4 [min, max]. Written by `aabb.wgsl`. |
| `contact_buffer` | max_contacts × 48 | Pipeline contacts (see struct below). Written by narrowphase. |
| `contact_count` | 4 | Active contact count (atomic u32). Reset to 0 before collision. |

**Note (Session 4):** `pair_buffer` and `pair_count` are NOT allocated.
The broadphase was replaced by a pre-computed pair plan — pairs are determined
at model upload time and dispatched directly, not written to a GPU buffer.

**Constraint working set (Session 5):**

| Buffer | Size per env | Contents |
|---|---|---|
| `efc_J` | max_constraints × nv × 4 | Constraint Jacobian, row-major (f32). Written by `assemble.wgsl`. |
| `efc_D` | max_constraints × 4 | Constraint stiffness / regularization (f32). Written by `assemble.wgsl`. |
| `efc_aref` | max_constraints × 4 | Reference acceleration (f32). Written by `assemble.wgsl`. |
| `efc_force` | max_constraints × 4 | Solver output forces (f32). Written by `newton_solve.wgsl`. |
| `constraint_count` | 4 | Active constraint row count (atomic u32). Written by `assemble.wgsl`. |
| `qfrc_constraint` | nv × 4 | Constraint forces in joint space (f32). Written by `map_forces.wgsl`. |

**Note (Session 5):** `efc_type` is NOT allocated on GPU. The Newton solver
classifies constraints on-the-fly by checking `jar = J·qacc - aref`: active
if `jar < 0`, satisfied otherwise. No explicit type enum is needed.

**PipelineContact struct (48 bytes) — implemented in Session 4:**
```wgsl
struct PipelineContact {
    point:    vec3<f32>,   // World-space contact position
    depth:    f32,         // Penetration depth (≥ 0)
    normal:   vec3<f32>,   // World-space contact normal (geom1 → geom2)
    geom1:    u32,         // Geom index A
    friction: vec3<f32>,   // Combined friction [slide, torsion, roll]
    geom2:    u32,         // Geom index B
};
```

Friction is combined at pair plan construction time: element-wise max of
both geoms' friction vectors. Written into `NarrowphaseParams` and copied
into each contact by the narrowphase shader.

**Note:** The standalone `GpuContact` (32 bytes, in `collision.rs`) used by
the half-GPU path (`GpuSdfCollision` trait) is a separate type and is
unchanged. `PipelineContact` (48 bytes) is used only by the pipeline path.

### 6.4 Pre-allocation sizes

| Buffer | Max size | Rationale |
|---|---|---|
| `max_contacts` | 32,768 | ~100 SDF pairs × ~300 contacts. Implemented in Session 4. |
| `max_pairs` | — | Not allocated. Pre-computed pair plan replaces GPU pair buffer. |
| `max_constraints` | 196,608 | 6 × max_contacts (pyramidal condim=4: 6 rows per contact). Session 5. |

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
    each thread: compute own result → accumulate into parent
```

Each depth level is one compute pass. wgpu guarantees a storage buffer
barrier between passes. No explicit synchronization needed.

### 7.3 Backward scan accumulation (no f32 atomics)

WGSL only supports `atomicAdd` on `atomic<u32>` and `atomic<i32>` —
**not f32**. The backward tree scan must accumulate floating-point
values (inertias, forces) into parent bodies without f32 atomics.

**Strategy (implemented in Session 2): global CAS atomics.**

The accumulation buffer (e.g., `body_crb`) is declared as
`array<atomic<u32>>`. Each child thread shifts its contribution,
then CAS-atomic-adds each f32 element into the parent's slots:

```wgsl
fn atomic_add_f32_crb(idx: u32, val: f32) {
    var old = atomicLoad(&body_crb[idx]);
    loop {
        let new_val = bitcast<u32>(bitcast<f32>(old) + val);
        let result = atomicCompareExchangeWeak(&body_crb[idx], old, new_val);
        if result.exchanged { break; }
        old = result.old_value;
    }
}
```

**WGSL limitation (LIMITATIONS.md §8):** Storage pointers cannot be
passed as function arguments. Each buffer that needs CAS-atomic f32
addition requires its own dedicated function accessing the global
variable by name (e.g., `atomic_add_f32_crb` for `body_crb`).

**For flat trees (hockey):** All children have parent 0 (world body).
Since the world body has no DOFs, the `parent == 0` guard in
`crba_backward` makes the entire backward scan a no-op. The M matrix
is block-diagonal — each free body's 6×6 block is independent.

**For deeper trees:** Typically 1–3 children per parent at any depth
level → near-zero CAS contention. If profiling shows contention is
a bottleneck, workgroup-local shared memory reduction can be added
as an optimization layer on top of the CAS pattern.

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

**Dispatch (Rust side) — dynamic uniform offsets:**

The depth-level `FkParams` uniform is NOT re-uploaded between passes.
Instead, all depth slots are pre-written into a single uniform buffer
(256-byte aligned slots), and `set_bind_group()` selects the slot via
a dynamic offset. This avoids multiple `queue.submit()` calls.

```rust
// Pre-write all depth slots before encoding
for depth in 0..=max_depth {
    let offset = u64::from(depth) * 256;  // minUniformBufferOffsetAlignment
    queue.write_buffer(&params_buffer, offset, bytemuck::bytes_of(&params));
}

// Encode all passes in one encoder, one submit
for depth in 0..=max_depth {
    let dyn_offset = depth * 256;
    let mut pass = encoder.begin_compute_pass(&desc);
    pass.set_pipeline(&fk_pipeline);
    pass.set_bind_group(0, &params_bind_group, &[dyn_offset]);
    pass.set_bind_group(1, &model_bind_group, &[]);
    pass.set_bind_group(2, &state_bind_group, &[]);
    pass.set_bind_group(3, &geom_bind_group, &[]);
    pass.dispatch_workgroups(
        nbody.div_ceil(64), // X: bodies
        n_env,              // Y: environments
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

**Estimated:** ~350 lines WGSL.
**Pattern:** Tree scan (forward), one pass per depth level.
**Reference:** MJX `kinematics.py:fwd_position()`, CPU `position.rs`

**Per body (at its depth level):**

**Step 0 — Mocap check:**
If `body_mocap_id[body] != 0xFFFFFFFF`: read pose directly from
`mocap_pos[mocap_id]` / `mocap_quat[mocap_id]`, skip steps 1–4.
VR controllers use this path.

**Step 1 — Parent frame:**
```
pos  = body_xpos[parent]
quat = body_xquat[parent]
```

**Step 2 — Body offset:**
```
pos  += rotate(quat, body_pos[body])
quat  = quat × body_quat[body]
```

**Step 3 — Joint loop** (a body may have multiple joints):
```
for j in body_jnt_adr[body] .. body_jnt_adr[body] + body_jnt_num[body]:
    match jnt_type[j]:
        Free:
            // Absolute — overwrites pos/quat, body offset is ignored
            pos  = vec3(qpos[adr], qpos[adr+1], qpos[adr+2])
            quat = normalize(vec4(qpos[adr+3..adr+7]))  // w,x,y,z
        Hinge:
            // Rotate around joint anchor in world frame
            world_axis   = rotate(quat, jnt_axis[j])
            world_anchor = pos + rotate(quat, jnt_pos[j])
            rot          = axis_angle_to_quat(world_axis, qpos[adr])
            quat         = rot × quat              // LEFT multiply (world frame)
            pos          = world_anchor + rotate(rot, pos - world_anchor)
        Ball:
            // qpos is [w,x,y,z] → swizzle to (x,y,z,w) for GPU quaternion layout
            dq           = normalize(vec4(qpos[adr+1], qpos[adr+2], qpos[adr+3], qpos[adr]))
            quat         = quat × dq              // RIGHT multiply (local frame)
            // NOTE: The original spec had MJX-style LEFT multiply with conjugate sandwich.
            // Session 1 matched the CPU (position.rs:90-98) which uses RIGHT multiply.
            // This is simpler and correct — Ball joints rotate in the body's local frame.
        Slide:
            pos += rotate(quat, jnt_axis[j]) × qpos[adr]
```

**Critical hinge details (verified against CPU `position.rs:61-82`):**
- `jnt_axis` is in local frame → must be rotated to world.
- Rotation is LEFT-multiplied (`rot × quat`), not right-multiplied.
  The rotation is in world frame (axis was rotated to world).
- Position is rotated around the joint **anchor point** (`jnt_pos`),
  not around the body origin. The pivot-point formula is:
  `pos = anchor + rot × (pos - anchor)`.

**Step 4 — Normalize quaternion:**
```
quat = normalize(quat)   // Prevent f32 drift. Critical for Free/Ball.
```

**Step 5 — Derived quantities:**
```
xmat  = quat_to_mat3(quat)
xipos = pos + rotate(quat, body_ipos[body])
ximat = quat_to_mat3(quat × body_iquat[body])
```

**Step 6 — Spatial inertia (cinert):**
Compute the 6×6 spatial inertia in world frame, stored as 12 floats
(3 × vec4, padded from 10 for alignment — see LIMITATIONS.md §3):
```
h = xipos - pos                    // COM offset from body origin (world frame)
I_COM = ximat × diag(body_inertia[body]) × ximat^T   // rotated inertia
cinert[body*3 + 0] = vec4(mass, h.x, h.y, h.z)
cinert[body*3 + 1] = vec4(I_COM[0,0], I_COM[0,1], I_COM[0,2], I_COM[1,1])
cinert[body*3 + 2] = vec4(I_COM[1,2], I_COM[2,2], 0, 0)  // 2 padding floats
```

**Reconstructing the 6×6 from 12 floats** (needed by CRBA and RNE):
```
v0 = cinert[body*3 + 0]    // (mass, h.x, h.y, h.z)
v1 = cinert[body*3 + 1]    // (I_00, I_01, I_02, I_11)
v2 = cinert[body*3 + 2]    // (I_12, I_22, _, _)
m = v0.x; h = v0.yzw; I_COM = symmetric3(v1.xyzw, v2.xy)
I_rot = I_COM + m × (dot(h,h) × I₃ - outer(h,h))   // parallel axis
coupling = m × skew(h)
I_spatial = [[I_rot, coupling], [coupling^T, m × I₃]]
```

**Step 7 — Geom poses:**
For each geom on this body:
```
geom_xpos[g] = pos + rotate(quat, geom_pos[g])
geom_xmat[g] = quat_to_mat3(quat × geom_quat[g])
```
Alternatively: emit geom poses in a separate per-geom dispatch after
FK completes (better load balance when bodies have unequal geom counts).

**Step 8 — Motion subspace (cdof):**
Per DOF on this body, compute the 6-vector spatial motion subspace
in world frame (GPU optimization — CPU computes this inside CRBA/RNE):
```
Hinge DOF: cdof = [world_axis; cross(world_axis, pos - world_anchor)]
Slide DOF: cdof = [0; world_axis]
Ball DOFs: cdof = [R[:,k]; 0]  for k=0,1,2  (columns of xmat)
Free linear DOFs: cdof = [0; e_k]  for k=0,1,2  (world basis)
Free angular DOFs: cdof = [R[:,k]; 0]  for k=0,1,2
```

**World body (depth 0):** Identity pose, zero cinert. Written as
initialization before the scan starts.

**Subtree COM (backward pass after FK):**
A separate backward tree scan accumulates weighted COM:
```
Phase 1 (per-body): subtree_mass[b] = mass[b];
                     weighted_com[b] = mass[b] × xipos[b]
Phase 2 (backward): accumulate into parent (see §7.3 for f32 workaround)
Phase 3 (per-body): subtree_com[b] = weighted_com[b] / subtree_mass[b]
                     Guard: if subtree_mass < 1e-10, use xipos[b]
```

### 8.2 `crba.wgsl` — Composite Rigid Body Algorithm — COMPLETE (Session 2)

**Actual:** ~300 lines WGSL. 4 entry points.
**Pattern:** Format conversion + backward tree scan + per-DOF parallel M assembly + Cholesky.
**Reference:** CPU `crba.rs`, `spatial.rs:shift_spatial_inertia()`
**Spec:** `sim/docs/gpu-physics-pipeline/SESSION_2_CRBA_VELOCITY_FK.md`

**Bindings (9 total, 4 bind groups):**
- Group 0: `FkParams` uniform (dynamic offset) — reuses FK params struct
- Group 1: `bodies` (read), `dofs` (read) — static model
- Group 2: `body_xpos`, `body_cinert`, `cdof` (read) — FK outputs
- Group 3: `body_crb` (atomic read/write), `qM`, `qM_factor` (read/write)

**Phase 1: `crba_init` — convert cinert to crb format (per-body parallel)**

cinert from FK stores `(m, h, I_COM)` — NOT additive (h doesn't sum
correctly). CRBA needs an additive format for accumulation.

crb format: `(m, m·h, I_ref)` where `I_ref = I_COM + m·(|h|²·I₃ − h⊗h)`.
All 10 useful values are element-wise additive when both operands share
the same reference point.

```
crb[body*3+0] = (m, m·h_x, m·h_y, m·h_z)
crb[body*3+1] = (I_ref_00, I_ref_01, I_ref_02, I_ref_11)
crb[body*3+2] = (I_ref_12, I_ref_22, 0, 0)
```

Stored in `array<atomic<u32>>` via `atomicStore` + `bitcast<u32>`.

**Phase 2: `crba_backward` — CAS atomic accumulation (per depth, leaves→root)**
```
for d in max_depth..=1:
  per body at depth d:
    if parent == 0: skip     // world body has no DOFs
    d = body_xpos[body] - body_xpos[parent]
    shifted = shift_crb(crb[body], d)
    CAS-atomic-add 10 floats into crb[parent]
```

The `shift_crb` operates on the crb format directly:
```
h = mh / m; h_new = h + d; mh_new = mh + m·d
I_ref_new[r,c] = I_ref_old[r,c] + m·((|h_new|²−|h|²)·δ_rc − (h_new[r]·h_new[c] − h[r]·h[c]))
```

**Phase 3: `crba_mass_matrix` — per-DOF parallel M assembly**

qM is zeroed via `queue.write_buffer` before this dispatch. Each DOF i
computes its column independently. The 6×6 multiply uses an **implicit
formulation** — no matrix materialization:

```
buf_angular = I_ref · ω + cross(mh, v)
buf_linear  = cross(ω, mh) + m · v
```

Off-diagonal entries use the `dof_parent` walk with spatial force
transport at body boundaries (`buf.angular += cross(r, buf.linear)`).
Verified against CPU `crba.rs:196-225`.

For flat trees (hockey), all DOFs share the same body so no cross-body
transport occurs. For articulated chains, the transport is essential.

Armature is pre-combined at upload: `dofs[i].armature = jnt_armature + dof_armature`.

**Phase 4: `crba_cholesky` — dense Cholesky (single thread, global memory)**

Standard Cholesky-Banachiewicz: lower triangular L such that M = L·L^T.
Single thread within one workgroup. Reads/writes global memory (not
shared memory — unnecessary optimization for nv ≤ 60).

For nv=13: ~366 flops. For nv=60: ~36K flops. Guard: `sqrt(max(sum, 1e-10))`.
Needed by `smooth.wgsl` for `qacc_smooth`.

### 8.3 `velocity_fk.wgsl` — Velocity forward kinematics — COMPLETE (Session 2)

**Actual:** ~100 lines WGSL. 1 entry point.
**Pattern:** Tree scan (forward), one dispatch per depth level.
**Reference:** CPU `velocity.rs:mj_fwd_velocity()`
**Spec:** `sim/docs/gpu-physics-pipeline/SESSION_2_CRBA_VELOCITY_FK.md`

**Bindings (6 total, 3 bind groups):**
- Group 0: `FkParams` uniform (dynamic offset)
- Group 1: `bodies` (read) — static model (no joints needed)
- Group 2: `body_xpos`, `cdof`, `qvel` (read), `body_cvel` (write)

Computes body spatial velocities (`cvel`) by propagating `qvel` from
root to leaves. **Must run after FK (needs cdof, body_xpos) and before
RNE (which reads cvel for Coriolis/centrifugal/gyroscopic forces).**

**Per body (at its depth level):**
```
// Transport parent velocity to this body's origin
omega = parent_omega
v_lin = parent_v_lin + cross(parent_omega, r)   // lever arm

// Add joint velocity contributions using cdof (no joint-type branching)
for each DOF d on this body:
    omega += cdof[d].angular × qvel[d]
    v_lin += cdof[d].linear  × qvel[d]
```

**Key deviation from CPU:** The CPU `velocity.rs` branches on joint type
to compute velocity contributions. The GPU uses `cdof[d] × qvel[d]`
directly, which is mathematically equivalent (cdof encodes the motion
subspace) and eliminates all branching. For zero-offset joints
(`jnt_pos = 0`, which includes all current test models), the results
match exactly. For offset joints, the GPU cdof approach would be more
correct than the CPU's angular-only hinge addition.

**World body (depth 0):** `cvel[0] = [0; 0; 0; 0; 0; 0]`.

**cvel layout:** 2×vec4 per body (32 bytes):
- `cvel[body*2+0]` = `(ω_x, ω_y, ω_z, 0)` — angular velocity
- `cvel[body*2+1]` = `(v_x, v_y, v_z, 0)` — linear velocity at body origin

**Why this stage is critical:** Without cvel, RNE produces zero
Coriolis, centrifugal, and gyroscopic forces. At the hockey stick's
swing speed (~25 rad/s), the missing centrifugal term visibly affects
dynamics.

### 8.4 `rne.wgsl` — Recursive Newton-Euler — COMPLETE (Session 3)

**Actual:** ~300 lines WGSL. 5 entry points.
**Pattern:** Per-joint map + forward tree scan + per-body map + backward tree scan + per-DOF map.
**Reference:** CPU `rne.rs`, MJX `smooth.py:rne()`
**Prerequisite:** `velocity_fk.wgsl` must have written `body_cvel`.
**Spec:** `sim/docs/gpu-physics-pipeline/SESSION_3_RNE_FORCES_INTEGRATION.md`

**Bindings (16 total, max 9 per entry point — under 16-per-stage limit):**
- Group 0: `PhysicsParams` uniform (dynamic offset)
- Group 1: `bodies`, `joints`, `dofs` (read) — static model
- Group 2: `body_xpos`, `body_xquat`, `body_cinert`, `cdof`, `body_cvel`,
  `subtree_mass`, `subtree_com`, `qvel`, `qpos` (read)
- Group 3: `body_cacc`, `body_cfrc` (atomic), `qfrc_bias` (read-write)

**Phase 1: `rne_gravity` — per-joint parallel (no tree dependency)**

Uses `subtree_mass[body]` and `subtree_com[body]` from FK. Joint-type
dependent gravity projection (hinge=axis·torque, slide=axis·force,
ball=body-local rotation, free=direct force + world-frame torque).
Requires `JointModelGpu.body_id` (added Session 3).
```
for each joint:
  gravity_force = -subtree_mass[body] * gravity
  qfrc_bias[dof] = project(gravity_force, jnt_type, subtree_com, jnt_axis, ...)
```
Note: gravity is computed ONLY here, not in passive forces.

**Phase 2a: `rne_forward` — bias accelerations (root → leaves)**
```
for d in 0..=max_depth:
  per body at depth d:
    // Transport parent bias acceleration
    r = body_xpos[body] - body_xpos[parent]
    cacc[body].angular = cacc[parent].angular
    cacc[body].linear  = cacc[parent].linear + cross(cacc[parent].angular, r)
    // Velocity-product acceleration (Coriolis) using cdof:
    for each DOF d on body:
      vj = cdof[d] * qvel[d]
      cacc += spatial_cross_motion(cvel[parent], vj)
    // Free joint correction: subtract [0; ω × v_linear]
    if dof_num == 6: cacc.linear -= cross(omega, v_linear)
```
`spatial_cross_motion(v, s)` = `[ω×s_ang; ω×s_lin + v_lin×s_ang]`

**Phase 2b: `rne_cfrc_init` — per-body parallel (no depth ordering)**

**CRITICAL: Must be a separate dispatch from the backward accumulation.**
See LIMITATIONS.md §9. Computes each body's own cfrc, writes via
`atomicStore` into the pre-zeroed `body_cfrc` buffer.
```
per body (all parallel):
  I = cinert[body]  (NOT body_crb — needs single-body, not composite)
  cfrc[body] = I × cacc[body] + cvel[body] ×* (I × cvel[body])
```
The gyroscopic term is `v ×* (I·v)` where `×*` is the spatial force
cross product: `[ω×f_ang + v_lin×f_lin; ω×f_lin]`.

Uses `cinert_mul_spatial()` — converts cinert `(m, h, I_COM)` to I_ref
inline via parallel axis theorem, then does implicit 6×6 multiply (same
math as CRBA's `crb_mul_cdof`).

**Phase 2c: `rne_backward` — accumulate cfrc (leaves → root)**

Only CAS-accumulates into parent — does NOT write own cfrc (that was
done in `rne_cfrc_init`). Uses buffer-specific `atomic_add_f32_cfrc()`
(LIMITATIONS.md §8).
```
for d in max_depth..=1:
  per body at depth d:
    if parent == 0: skip  (world body has no DOFs)
    CAS-atomic-add cfrc[body] into cfrc[parent]
```

**No spatial transport in backward pass:** The CPU (`rne.rs:271-278`)
does plain `cfrc[parent] += cfrc[child]` — no frame change — because
all quantities are in world frame.

**Phase 2d: `rne_project` — per-DOF parallel**
```
for each DOF d:
  qfrc_bias[d] += dot(cdof[d], cfrc[dof_body])
```
Uses `cdof` to avoid joint-type branching — `S^T · f = dot(cdof, cfrc)`.

### 8.5 `actuation.wgsl` — Actuator forces (DEFERRED)

**Status:** Not implemented in Session 3. Hockey has `nu=0` (no actuators).
Building untestable code violates "A-grade or it doesn't ship."
`qfrc_actuator` buffer is allocated and zeroed instead. Shader will be
added when a test model with actuators exists for validation.

**Estimated:** ~150 lines WGSL.
**Pattern:** Per-actuator parallel map.

**Per actuator (when implemented):**
1. Read `ctrl[i]` (uploaded from CPU)
2. Compute force from gain/bias: `force = gain(ctrl) + bias(qpos, qvel)`
3. Apply transmission: `qfrc_actuator[dof] += gear × force`

**Scope:** Covers the common actuator types (motor, position, velocity).
Complex actuators (muscle, cylinder) remain CPU-only — the GPU path
asserts supported actuator types at pipeline creation.

### 8.6 `passive.wgsl` — Passive forces (DEFERRED)

**Status:** Not implemented in Session 3. Hockey free bodies have no
springs or dampers. `qfrc_passive` buffer is allocated and zeroed.
Shader will be added when a test model with springs/dampers exists.

**Estimated:** ~100 lines WGSL.
**Pattern:** Per-DOF parallel map.

**Per DOF (when implemented):**
1. Joint damping: `qfrc_passive[dof] = -damping[dof] × qvel[dof]`
2. Joint stiffness: `qfrc_passive[dof] -= stiffness[dof] × (qpos - qpos_spring)`

**Gravity is NOT computed here.** Gravity is computed exclusively in
`rne.wgsl` Phase 1 (via subtree COM). The CPU passive force function
(`mj_fwd_passive`) also does not compute gravity — it handles springs,
dampers, fluid forces, and gravity compensation. Computing gravity in
both RNE and passive would double-count it.

**Scope:** Covers joint-level springs/dampers. Fluid forces and tendon
passive forces remain CPU-only for the initial implementation.

### 8.7 `smooth.wgsl` — Smooth forces + qacc_smooth — COMPLETE (Session 3)

**Actual:** ~98 lines WGSL. 2 entry points.
**Pattern:** Per-DOF parallel + single-thread Cholesky solve.
**Spec:** `sim/docs/gpu-physics-pipeline/SESSION_3_RNE_FORCES_INTEGRATION.md`

**Bindings (8 total, 4 bind groups):**
- Group 0: `PhysicsParams` uniform (dynamic offset)
- Group 1: `qfrc_bias`, `qfrc_applied`, `qfrc_actuator`, `qfrc_passive` (read)
- Group 2: `qM_factor` (read)
- Group 3: `qfrc_smooth`, `qacc_smooth` (read-write)

**Phase 1: `smooth_assemble` — per-DOF parallel**
```
qfrc_smooth[dof] = qfrc_applied[dof] + qfrc_actuator[dof]
                 + qfrc_passive[dof] - qfrc_bias[dof]
```

**Note on xfrc_applied projection (DEFERRED):** The CPU
(`constraint/mod.rs:89-107`) projects body-frame Cartesian forces
`xfrc_applied[B]` into joint space by walking ancestor DOFs via
`mj_apply_ft`. For gravity-only (Session 3), `xfrc_applied` is zeroed.
The projection will be added when external forces are needed. The spec
below describes the full algorithm for future implementation:
```
for each body B with non-zero xfrc_applied[B]:
  lever = xipos[B] - body_xpos[dof_body[d]]
  qfrc_smooth[d] += dot(cdof[d], [torque; force])
                  + dot(cdof[d].angular, cross(lever, force))
```

**Phase 2: `smooth_solve` — single thread (Cholesky forward/back substitution)**

Uses the Cholesky factor L from `crba.wgsl`:
```
// Forward substitution: L · y = qfrc_smooth
// Backward substitution: L^T · x = y
// Result: qacc_smooth = M⁻¹ · qfrc_smooth
```
Single thread on global memory (same approach as `crba_cholesky`).
For nv=13: ~338 flops. Trivially fast.

### 8.8 `aabb.wgsl` — Bounding box computation — COMPLETE (Session 4)

**Actual:** ~98 lines WGSL. 1 entry point: `compute_aabb`.
**Pattern:** Per-geom parallel map.
**Spec:** `sim/docs/gpu-physics-pipeline/SESSION_4_COLLISION_PIPELINE.md`

**Bindings (5 total, 3 bind groups):**
- Group 0: `AabbParams` uniform (ngeom)
- Group 1: `geoms` (read) — GeomModel array (includes type, size)
- Group 2: `geom_xpos`, `geom_xmat` (read), `geom_aabb` (write)

Per geom: read `geom_xpos`, `geom_xmat`, `GeomModel.size`,
`GeomModel.geom_type` → compute local half-extents per type → rotate
OBB to world → write axis-aligned envelope. Uses fresh FK poses (no
stale data). AABB is NOT margin-inflated — the narrowphase AABB guard
handles margin.

### 8.9 `broadphase.wgsl` — DEFERRED (replaced by pre-computed pair plan)

**Status:** Not implemented. The original spec called for a 3-pass
spatial hash (GPU Gems 3). Hockey has 4 collision-active geoms →
C(4,2) = 6 possible pairs. A spatial hash is untestable at this scale
and violates "A-grade or it doesn't ship."

**Replacement (Session 4):** Pre-computed pair plan at model upload time.
`GpuCollisionPipeline::new()` enumerates all valid collision pairs
(filtered by contype/conaffinity and same-body exclusion), classifies
by narrowphase type (SDF-SDF, SDF-plane, or skip), and creates dispatch
descriptors. Each narrowphase dispatch includes an AABB overlap guard —
if AABBs don't overlap, all threads return immediately. Fully
GPU-resident, no CPU readback needed.

**Future upgrade path:** When scenes grow to 1000+ geoms, add a
`broadphase.wgsl` spatial hash that writes to `pair_buffer`. The
narrowphase then reads `pair_buffer` instead of using the pre-computed
plan. The AABB shader and narrowphase shaders are reused as-is.

### 8.10 `sdf_sdf_narrow.wgsl` — SDF-SDF narrowphase — COMPLETE (Session 4)

**Actual:** ~310 lines WGSL. 1 entry point: `sdf_sdf_narrow`.
**Pattern:** Per-voxel parallel (workgroup 8×8×4), same as trace_surface.wgsl.
**Spec:** `sim/docs/gpu-physics-pipeline/SESSION_4_COLLISION_PIPELINE.md`

**Note:** This is a **new file** adapted from `trace_surface.wgsl`, not a
modification of it. The standalone `trace_surface.wgsl` (267 lines) is
preserved unchanged for the existing `GpuSdfCollider` / `GpuSdfCollision`
trait used by the half-GPU path.

**Key changes from standalone trace_surface.wgsl:**
1. **AABB guard** — first operation reads `geom_aabb` for the pair, all
   threads return if no overlap.
2. **Unified SDF buffer** — reads from `sdf_values[offset + idx]` via
   `sdf_metas[params.src_sdf_meta_idx]` instead of separate per-grid buffers.
3. **Poses from FK output** — constructs mat4x4 from `geom_xpos`/`geom_xmat`
   directly, no CPU-uploaded pose matrices.
4. **Extended contact output** — writes 48-byte `PipelineContact` with geom
   indices + combined friction (from `NarrowphaseParams`).

**Bindings (9 total, 4 bind groups):**
- Group 0: `NarrowphaseParams` uniform (pair metadata, friction, margin)
- Group 1: `sdf_metas` (read), `sdf_values` (read) — unified SDF data
- Group 2: `geom_xpos`, `geom_xmat`, `geom_aabb` (read) — FK output
- Group 3: `contact_buffer` (rw), `contact_count` (atomic)

**Symmetric dispatch:** For each SDF-SDF pair, two dispatches are encoded
(A→B with `flip_normal=0`, B→A with `flip_normal=1`), matching the
existing trace_surface.wgsl pattern. Both write to the same contact buffer.

**No GPU-side dedup.** The symmetric dispatch produces ~2× contacts.
Dedup is deferred to Session 5's constraint assembly (contact capping).

### 8.11 `sdf_plane_narrow.wgsl` — SDF-plane narrowphase — COMPLETE (Session 4)

**Actual:** ~213 lines WGSL. 1 entry point: `sdf_plane_narrow`.
**Pattern:** Per-voxel parallel (workgroup 8×8×4).
**Spec:** `sim/docs/gpu-physics-pipeline/SESSION_4_COLLISION_PIPELINE.md`

**Bindings:** Same bind group layout as `sdf_sdf_narrow.wgsl`.

Per grid cell: surface filter → gradient → surface reconstruction →
transform to world → compute signed distance to plane → contact test →
emit `PipelineContact` with plane normal as contact normal.

**Plane equation** extracted from FK output: plane normal = Z-axis of
`geom_xmat[plane_geom]` (column 2), plane offset = `dot(normal, geom_xpos[plane_geom])`.

**Contact normal** points away from plane (UP), matching CPU convention.

### 8.12 `assemble.wgsl` — Constraint assembly — IMPLEMENTED

**Actual:** ~490 lines WGSL. Entry point: `assemble_constraints`.
**Pattern:** Per-contact parallel, `@workgroup_size(256)`.
**Reference:** CPU `contact_assembly.rs:assemble_pyramidal_contact()`

**Bindings:** 13 total (1 uniform + 12 storage), 4 bind groups.
- Group 0: `AssemblyParams` uniform (nv, timestep, solref, solimp)
- Group 1: geoms, bodies, body_invweight0 (static model, read)
- Group 2: contact_buffer (raw f32), contact_count (atomic rw), body_xpos, body_xquat, qvel (read)
- Group 3: efc_J, efc_D, efc_aref, constraint_count (output, rw)

**Algorithm per thread (one contact):**
1. Read `PipelineContact` as 12 raw f32s (vec3 storage alignment workaround, LIMITATIONS §12)
2. Look up geom → body mapping, effective condim = min(geom1.condim, geom2.condim)
3. **Body swap:** If body1 > body2, swap body indices (no normal negate — normal already
   points from lower body toward higher body). Ensures body2 = moving body (+1 sign).
4. Compute tangent frame from normal via cross-product with reference axis
5. Allocate rows via `atomicAdd(constraint_count)`: condim=1→1, condim=3→4, condim=4→6
6. Per row: zero J row, write Jacobian, compute impedance/diagApprox/D/aref

**Jacobian (free joints only):**
- Translational DOFs: `J[dof+0..3] = sign × facet_dir`
- Angular DOFs: `J[dof+3+i] = sign × dot(facet_dir, cross(R·eᵢ, r))`
- `r = stabilize_lever_arm(contact_point - body_xpos, normal)` — projects out sub-physical perpendicular drift
- Pyramidal facets: `facet_dir = normal ± mu × tangent_d`
- Torsional facets: translational = normal; angular adds `± mu_torsion × dot(normal, omega_i)`

**diagApprox (bodyweight, O(1) per row):**
`bw = (1 + mu²) × (invweight0[body1].x + invweight0[body2].x)` for pyramidal.
World body (id=0) contributes 0. No M⁻¹ solve needed.

**Impedance:** Sigmoid from solimp params: `imp = d0 + y × (dwidth - d0)` where `y` is
power-law interpolation of `violation / width`. Clamped to [0.0001, 0.9999].
`R = (1-imp)/imp × diag`, `D = 1/R`.

**KBIP:** `K = 1/(dmax² × tc² × dr²)`, `B = 2/(dmax × tc)`, `aref = -B×vel - K×imp×pos`.

### 8.13 `newton_solve.wgsl` — Primal Newton solver — IMPLEMENTED

**Actual:** ~390 lines WGSL. Entry point: `newton_solve`.
**Pattern:** Single workgroup (256 threads), all iterations in one dispatch.
**Reference:** CPU `newton.rs:newton_solve()`, `primal.rs`

**Bindings:** 10 total (1 uniform + 9 storage), 4 bind groups.
- Group 0: `SolverParams` uniform (nv, max_iter, tolerance, meaninertia)
- Group 1: qM, qacc_smooth, qfrc_smooth (read)
- Group 2: efc_J, efc_D, efc_aref, constraint_count (read, but atomic requires rw — LIMITATIONS §13)
- Group 3: qacc, efc_force (output, rw)

**Shared memory (exactly 16,384 bytes at MAX_NV=60 — LIMITATIONS §11):**

| Array | Size |
|---|---|
| `H_atomic` (60×60 atomic u32) | 14,400 B |
| `qacc_sh`, `qacc_sm_sh`, `grad_sh`, `search_sh` (60 f32 each) | 960 B |
| `reduction_sh` (256 f32) | 1,024 B |
| **Total** | **16,384 B** |

Ma and qfrc_smooth read from global memory on-the-fly (no shared memory budget).

**Algorithm (per Newton iteration):**
1. **Init H = M**: 256 threads cooperatively copy `qM → H_atomic`
2. **Classify + H + J^T·force**: Each thread processes constraint row stripe.
   Active rows (jar < 0) contribute `D·J^T·J` to H via CAS atomics (LIMITATIONS §4)
   and accumulate partial J^T·force in per-thread registers (`partial_jtf[MAX_NV]`).
   Merged single pass — no second iteration over constraints for gradient.
3. **Cholesky**: Thread 0 factorizes H in shared memory. `sqrt(max(s, 1e-10))` guard.
4. **Gradient**: Per-DOF parallel reduction of `partial_jtf` across 256 threads.
   Thread 0 adds `Ma[k] - qfrc_smooth[k]` to complete `grad = Ma - qfrc_smooth - J^T·force`.
5. **Search**: Thread 0 solves `search = -H⁻¹·grad` via Cholesky forward/backward substitution.
6. **Line search (backtracking, not bracket-narrowing Newton):**
   Evaluates cost at 4 alphas {1.0, 0.5, 0.25, 0.125} plus alpha=0 (current).
   All 256 threads participate in all evaluations (LIMITATIONS §14 — barriers require
   uniform control flow). Thread 0 picks the alpha minimizing total cost.
   Cost = Gauss (½·u^T·M·u) + Σ_active ½·D·jar².
7. **Update**: `qacc += best_alpha × search`. If alpha=0, set convergence flag.

**Cold start:** Every substep initializes `qacc = qacc_smooth`. Warm start (reusing
previous substep's qacc) is not yet implemented — deferred to Session 6 or later.

**Finalize:** Writes final qacc and efc_force (= -D·jar for active, 0 for satisfied)
to global memory.

**For nv > 60:** Shared memory insufficient for dense H. Future: sparse Hessian or tiled approach.

### 8.14 `map_forces.wgsl` — Force mapping — IMPLEMENTED

**Actual:** ~56 lines WGSL. Entry point: `map_forces`.
**Pattern:** Per-DOF parallel, `@workgroup_size(64)`.

**Bindings:** 5 total (1 uniform + 4 storage), 3 bind groups.

`qfrc_constraint[dof] = Σ_row efc_force[row] × efc_J[row × nv + dof]`

Reads `constraint_count` via `atomicLoad` (requires rw access — LIMITATIONS §13).
Each thread handles one DOF, iterates over all constraint rows.

### 8.15 `integrate.wgsl` — Semi-implicit Euler — COMPLETE (Session 3)

**Actual:** ~130 lines WGSL. 1 entry point.
**Pattern:** Per-joint parallel map (one thread per joint).
**Reference:** CPU `euler.rs`, MJX `math.py:quat_integrate()`
**Spec:** `sim/docs/gpu-physics-pipeline/SESSION_3_RNE_FORCES_INTEGRATION.md`

**Bindings (5 total, 3 bind groups):**
- Group 0: `PhysicsParams` uniform (non-dynamic, single dispatch)
- Group 1: `joints` (read) — static model
- Group 2: `qpos` (read-write), `qvel` (read-write), `qacc` (read)

Per joint: semi-implicit Euler (`qvel += dt × qacc`, then `qpos += dt × f(qvel)`).
Hinge/slide: scalar integration. Ball/free: quaternion exponential map
(`q_new = q_old × axis_angle(ω/|ω|, |ω|×dt)`) with normalization. Handles
qpos quaternion layout swizzle `(w,x,y,z) ↔ (x,y,z,w)` (LIMITATIONS.md §6).

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

    // Velocity FK: forward scan (needs cdof from FK, writes cvel)
    for depth in 0..=max_depth {
        update_uniform(&encoder, depth);
        compute_pass(&encoder, &velocity_fk_pipeline, ceil(nbody, 64), n_env);
    }

    // RNE: gravity + forward scan + cfrc_init + backward accumulation + project
    // (reads cvel from velocity_fk for Coriolis/gyroscopic terms)
    // NOTE: cfrc_init and backward MUST be separate dispatches (LIMITATIONS.md §9)
    compute_pass(&encoder, &rne_gravity_pipeline, ceil(njnt, 64), n_env);
    for depth in 0..=max_depth {
        update_uniform(&encoder, depth);
        compute_pass(&encoder, &rne_forward_pipeline, ceil(nbody, 64), n_env);
    }
    compute_pass(&encoder, &rne_cfrc_init_pipeline, ceil(nbody, 64), n_env);
    for depth in (1..=max_depth).rev() {
        update_uniform(&encoder, depth);
        compute_pass(&encoder, &rne_backward_pipeline, ceil(nbody, 64), n_env);
    }
    compute_pass(&encoder, &rne_project_pipeline, ceil(nv, 64), n_env);

    // Actuation + passive (DEFERRED — buffers zeroed for gravity-only)
    // compute_pass(&encoder, &actuation_pipeline, ceil(nu, 64), n_env);
    // compute_pass(&encoder, &passive_pipeline, ceil(nv, 64), n_env);

    // Smooth: assemble qfrc_smooth + Cholesky solve for qacc_smooth
    compute_pass(&encoder, &smooth_assemble_pipeline, ceil(nv, 64), n_env);
    compute_pass(&encoder, &smooth_solve_pipeline, 1, n_env);

    // Collision (Session 4)
    encoder.clear_buffer(&contact_count, 0, 4);  // reset atomic counter
    compute_pass(&encoder, &aabb_pipeline, ceil(ngeom, 64), n_env);
    // Narrowphase: pre-computed pair plan (no broadphase shader)
    // For each pair (determined at model upload):
    //   - Write NarrowphaseParams for this pair
    //   - Dispatch sdf_sdf_narrow (×2 symmetric) or sdf_plane_narrow (×1)
    //   - Shader includes AABB overlap guard (returns if no overlap)
    collision_pipeline.encode(&mut encoder, &ctx, &model_bufs, &state_bufs);

    // Constraint solve (Session 5) — replaces gravity-only bridge
    // GpuConstraintPipeline::encode() dispatches 3 compute passes:
    //   1. clear constraint_count atomic
    //   2. assemble_constraints: per-contact → pyramidal efc_J/D/aref
    //   3. newton_solve: single workgroup, internal iteration loop (NOT multiple dispatches)
    //   4. map_forces: per-DOF J^T · efc_force → qfrc_constraint
    // Newton writes qacc directly — no copy needed.
    constraint_pipeline.encode(&mut encoder, &state_bufs);

    // Integration (per-joint, not per-body)
    compute_pass(&encoder, &integrate_pipeline, ceil(njnt, 64), n_env);
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
| Velocity FK (2 levels) | 2 |
| RNE (gravity + 2 forward + cfrc_init + 1 backward + project) | 6 |
| Smooth (assemble + solve) | 2 |
| AABB (1) + narrowphase (9 = 3 SDF-SDF×2 + 3 SDF-plane) | 10 |
| Constraint solve (clear + assemble + newton + map_forces) | 4 |
| Integration | 1 |
| **Total** | **~34** |

At ~5μs per dispatch: ~150μs overhead. Well within the 1ms/substep
budget.

**Note (Session 5):** The Newton solver runs ALL iterations in a single
dispatch with an internal `for` loop — NOT multiple dispatches as
originally spec'd. This eliminates inter-dispatch overhead and keeps
shared memory state persistent across iterations.

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
| FK (fk.wgsl) | Compare GPU body_xpos/xquat/cinert vs CPU mj_fwd_position(). Test hinge pivot-point rotation with non-zero jnt_pos. |
| CRBA (crba.wgsl) | Compare GPU qM vs CPU mj_crba(). Test off-diagonal entries on a 3-body chain (depth 2+) to verify spatial force transport. |
| Velocity FK (velocity_fk.wgsl) | Compare GPU body_cvel vs CPU mj_fwd_velocity(). Verify Coriolis terms at high angular velocity. |
| RNE (rne.wgsl) | Compare GPU qfrc_bias vs CPU mj_rne(). Verify gyroscopic terms are non-zero when cvel is non-zero. |
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

## 14. Implementation sessions

Implementation is organized into 6 sessions. Each session is designed
to be self-contained: start by reading the spec sections listed, read
the CPU reference files, implement, validate against CPU, commit. Use
`/clear` between sessions — the spec provides full continuity.

### Session 1: FK + tree scan primitive — COMPLETE (2026-03-24)

**Goal:** Body poses computed on GPU match CPU FK within f32 tolerance.

**Spec:** `sim/docs/gpu-physics-pipeline/SESSION_1_FK_TREE_SCAN.md`

**Implemented:**
1. Tree scan dispatch with dynamic uniform offsets (one submit, not one per depth)
2. `fk.wgsl` — 4 entry points: fk_forward, fk_geom_poses, fk_subtree_backward, fk_subtree_normalize
3. Packed struct buffers (`BodyModelGpu`, `JointModelGpu`, `GeomModelGpu`) — §6.2 updated
4. `GpuModelBuffers` — computes `body_depth`/`max_depth` at upload, packs structs
5. `GpuStateBuffers` — per-env state with n_env=1 (batch support deferred)
6. `GpuFkPipeline` — 4 pipelines, unified layout (4 bind groups), dispatch loop
7. Readback utilities (`readback_vec4s`, `readback_f32s`)

**Validated (7 tests, all passing):**
- T1: Free body xpos/xquat (1e-5)
- T2: 7-body hinge chain xpos/xquat (1e-4)
- T3: Free body at offset (1e-5)
- T4: Cinert for free body (1e-4)
- T5: cdof for 3-link pendulum (1e-4)
- T6: Subtree COM for 3-link pendulum (1e-4)
- T7: Cinert mass for hinge chain (1e-3)

**Deviations from original spec:**
- Buffer architecture: packed structs (AoS) instead of SoA individual buffers (wgpu limit)
- Ball joint: RIGHT multiply matching CPU, not LEFT multiply from MJX (§8.1 updated)
- Cinert: 12 floats (3×vec4) not 10; cdof: 8 floats (2×vec4) not 6 (alignment)
- Subtree COM backward: plain f32 addition, not CAS atomics (single-workgroup safe)
- `max_storage_buffers_per_shader_stage` raised to 16 in `GpuContext`

**Constraints documented:** `sim/docs/gpu-physics-pipeline/LIMITATIONS.md`

**Milestone:** `cargo test -p sim-gpu` — 11/11 tests pass (4 existing + 7 new).

---

### Session 2: CRBA + velocity FK — COMPLETE (2026-03-24)

**Goal:** Mass matrix M and body velocities cvel match CPU within f32 tolerance.

**Spec:** `sim/docs/gpu-physics-pipeline/SESSION_2_CRBA_VELOCITY_FK.md`

**Implemented:**
1. `crba.wgsl` — 4 entry points: `crba_init`, `crba_backward`, `crba_mass_matrix`, `crba_cholesky`
2. `velocity_fk.wgsl` — 1 entry point: `velocity_fk_forward`
3. `DofModelGpu` packed struct (body_id, parent, armature) — §6.2 updated
4. `GpuCrbaPipeline` — 4 pipelines, 4 bind groups (9 bindings total), CAS atomics
5. `GpuVelocityFkPipeline` — 1 pipeline, 3 bind groups (6 bindings total)
6. New state buffers: `body_crb`, `qM`, `qM_factor`, `qvel`, `body_cvel`

**Validated (5 tests, all passing):**
- T8:  qM diagonal for free body (1e-4)
- T9:  Full qM for 3-link pendulum — off-diagonal + spatial force transport (1e-3)
- T10: qM for flat tree free body (1e-4)
- T11: cvel for free body with non-zero qvel (1e-5)
- T12: cvel for 3-link pendulum at various qvel (1e-4)

**Deviations from original spec:**
- crb format: cinert `(m, h, I_COM)` is NOT additive; `crba_init` converts to
  `(m, m·h, I_ref)` via parallel axis theorem. Original spec showed direct copy.
- Backward scan: global CAS atomics directly, not workgroup-local shared memory
  reduction. Simpler and correct for all tree sizes. For hockey (flat tree),
  the `parent == 0` guard makes the entire backward scan a no-op.
- Cholesky: single thread on global memory, not shared memory. Trivially fast
  for nv ≤ 60 (~36K flops max). Shared memory optimization unnecessary.
- Velocity FK: uses `cdof × qvel` instead of per-joint-type branching.
  Mathematically equivalent, eliminates all branching, more GPU-friendly.
- `body_cvel`: 2×vec4 (32 bytes/body) not 6×f32 (24 bytes) — vec4 alignment.
- Armature pre-combined at upload: `dofs[i].armature = jnt_armature + dof_armature`.

**Constraints documented:** `LIMITATIONS.md` §8 (WGSL storage pointer function args)

**Milestone:** `cargo test -p sim-gpu` — 16/16 tests pass (11 existing + 5 new).

---

### Session 3: RNE + forces + integration (gravity-only sim) — COMPLETE (2026-03-24)

**Goal:** A complete GPU physics loop (no contacts) that drops objects
under gravity and matches CPU trajectories.

**Spec:** `sim/docs/gpu-physics-pipeline/SESSION_3_RNE_FORCES_INTEGRATION.md`

**Implemented:**
1. `rne.wgsl` — 5 entry points: `rne_gravity`, `rne_forward`, `rne_cfrc_init`, `rne_backward`, `rne_project`
2. `smooth.wgsl` — 2 entry points: `smooth_assemble`, `smooth_solve`
3. `integrate.wgsl` — 1 entry point: `integrate_euler`
4. `PhysicsParams` uniform struct (gravity, timestep, counts) — separate from `FkParams`
5. `GpuRnePipeline` — 5 pipelines, 4 bind groups (16 bindings max, 9 per entry point)
6. `GpuSmoothPipeline` — 2 pipelines, 4 bind groups (8 bindings)
7. `GpuIntegratePipeline` — 1 pipeline, 3 bind groups (5 bindings)
8. New state buffers: body_cacc, body_cfrc, qfrc_bias, qfrc_applied, qfrc_actuator, qfrc_passive, qfrc_smooth, qacc_smooth, qacc
9. Gravity-only bridge: `encoder.copy_buffer_to_buffer(qacc_smooth → qacc)`
10. `JointModelGpu.body_id` (was `_pad`) — needed by RNE gravity + integration

**Validated (6 tests, all passing):**
- T13: qfrc_bias gravity for free body (1e-4)
- T14: qfrc_bias Coriolis for spinning free body — verifies non-zero gyroscopic terms (1e-3)
- T15: qfrc_bias for 3-link pendulum — forward/backward scan + CAS accumulation (1e-3)
- T16: qacc_smooth for free body under gravity — end-to-end FK→CRBA→vel_FK→RNE→smooth (1e-4)
- T17: Gravity drop trajectory — 2 seconds, GPU vs CPU position match (<0.05m)
- T18: Quaternion stability — 1000 steps at 50 rad/s, norm stays within [0.99, 1.01]

**Deviations from original spec:**
- No `actuation.wgsl` or `passive.wgsl` — hockey has `nu=0` and no springs/dampers.
  Building untestable code violates "A-grade or it doesn't ship." Buffers zeroed instead.
  Shaders will be added when a scene with actuators/springs exists for validation.
- `xfrc_applied` projection deferred — zeroed for gravity-only.
- RNE backward split into two entry points: `rne_cfrc_init` (per-body parallel,
  computes own cfrc) + `rne_backward` (depth-ordered CAS accumulation into parent).
  The original single-pass design had a bug: `atomicStore` of own cfrc overwrote
  CAS additions from deeper children. The CPU separates these into two loops
  (`rne.rs:252-268` then `rne.rs:271-278`), and the GPU must do the same.
- RNE uses `cinert` directly (not `body_crb`) — after CRBA, body_crb contains
  composite inertia, but RNE needs single-body inertia. Converts cinert → I_ref inline.
- `body_cacc`/`body_cfrc` use 2×vec4 (32 bytes/body), not 6×f32 (24 bytes) — vec4 alignment.
- `PhysicsParams` (48 bytes) is separate from `FkParams` (32 bytes) — keeps kinematics
  and dynamics concerns separate. Contains gravity vec4 and timestep.

**Milestone:** `cargo test -p sim-gpu` — 22/22 tests pass (16 existing + 6 new).
First full GPU physics loop: objects fall under gravity correctly.

---

### Session 4: Collision pipeline — COMPLETE (2026-03-24)

**Goal:** GPU collision detection (AABB + narrowphase) produces contacts
that match CPU collision within f32 tolerance.

**Spec:** `sim/docs/gpu-physics-pipeline/SESSION_4_COLLISION_PIPELINE.md`

**Implemented:**
1. `aabb.wgsl` — 1 entry point: `compute_aabb` (per-geom parallel)
2. `sdf_sdf_narrow.wgsl` — 1 entry point: `sdf_sdf_narrow` (adapted from trace_surface.wgsl)
3. `sdf_plane_narrow.wgsl` — 1 entry point: `sdf_plane_narrow`
4. Extended `GeomModelGpu` (48 → 96 bytes): type, contype, conaffinity, size, friction, sdf_meta_idx, condim
5. `SdfMetaGpu` (32 bytes): per-shape grid metadata pointing into unified buffer
6. `PipelineContact` (48 bytes): contact with geom indices + combined friction
7. `NarrowphaseParams` (48 bytes): per-pair dispatch uniform
8. Unified SDF grid buffer: all grids concatenated in `sdf_values`, indexed via `sdf_metas`
9. `GpuCollisionPipeline`: pre-computed pair plan, AABB guard, dispatch orchestration
10. New state buffers: `geom_aabb`, `contact_buffer` (48×32768), `contact_count`

**Validated (4 tests, all passing):**
- T19: AABB computation for plane + SDF sphere (center/half-extent correct)
- T20: SDF-SDF contacts — two overlapping spheres produce contacts with valid normals
- T21: SDF-plane contacts — sphere penetrating ground, normals point up
- T22: Full collision pipeline — mixed SDF-SDF + SDF-plane pairs, valid geom indices + friction

**Deviations from original spec:**
- No `broadphase.wgsl` (spatial hash). Hockey has 4 collision-active geoms →
  C(4,2) = 6 possible pairs. A 3-pass spatial hash is untestable at this scale.
  Replaced by pre-computed pair plan at model upload time. Each narrowphase
  dispatch includes an AABB overlap guard — if AABBs don't overlap, all threads
  return immediately. Fully GPU-resident, no readback. Spatial hash can be added
  later when scenes grow to 1000+ geoms.
- Narrowphase shaders read `geom_xpos`/`geom_xmat` directly from FK output
  instead of receiving pre-computed mat4x4 poses via params. Eliminates per-pair
  CPU pose upload, keeps poses on GPU.
- `sdf_sdf_narrow.wgsl` is a clean rewrite adapted from `trace_surface.wgsl`,
  not a modification of it. The standalone `trace_surface.wgsl` is preserved
  unchanged for the existing `GpuSdfCollider` (used by `GpuSdfCollision` trait).
- Contact deduplication not implemented on GPU. SDF-SDF symmetric dispatch
  produces ~2× contacts. Dedup deferred to Session 5's constraint assembly
  (contact capping per pair).
- `meta` is a WGSL reserved keyword — grid metadata parameter renamed to `gm`.

**Milestone:** `cargo test -p sim-gpu` — 26/26 tests pass (22 existing + 4 new).
Collision contacts feed into Session 5's constraint assembly via `contact_buffer`.

---

### Session 5: Constraint solve — COMPLETE (2026-03-24)

**Goal:** GPU constraint solver (assembly + Newton + force mapping)
produces qacc that matches CPU Newton solver within f32 tolerance.

**Spec:** `sim/docs/gpu-physics-pipeline/SESSION_5_CONSTRAINT_SOLVE.md`

**Implemented:**
1. `assemble.wgsl` — per-contact parallel: PipelineContact → pyramidal constraint rows (efc_J/D/aref). Free joints only. condim=1,3,4. Bodyweight diagApprox.
2. `newton_solve.wgsl` — single workgroup (256 threads): shared-memory Hessian (MAX_NV=60, 16 KB), Cholesky factorization, backtracking line search (4 candidates: {1.0, 0.5, 0.25, 0.125}). Cold start from qacc_smooth.
3. `map_forces.wgsl` — per-DOF parallel: qfrc_constraint = J^T · efc_force.
4. `GpuConstraintPipeline` — Rust orchestration: assemble + newton + map_forces.
5. New state buffers: efc_J, efc_D, efc_aref, efc_force, constraint_count, qfrc_constraint.
6. New model buffer: body_invweight0 (per-body translational + rotational inverse weight).
7. Gravity-only bridge replaced by constraint solver output.

**Validated (4 tests, all passing):**
- T23: Assembly produces constraint rows (1376 rows from SDF-plane contacts), all efc_D > 0
- T24: Newton solver pushes sphere upward (qacc.z = 5217.9 > qacc_smooth.z = -9.81)
- T25: Zero contacts → qacc == qacc_smooth (exact match, cold start)
- T26: Multi-substep stability — 10 steps, sphere at z=4.75, no explosion

**Deviations from original spec:**
- Free joints only — general joint kinematic chain traversal deferred.
- Cold start (no warm start) — qacc initialized from qacc_smooth each substep.
- Backtracking line search, not bracket-narrowing Newton 1D — 4 fixed alpha candidates.
- condim=1,3,4 only — condim=6 (rolling friction) deferred.
- Pyramidal only (no elliptic cone) — matches MJX GPU convention.
- Body swap without normal negate — SDF-plane outputs geom1=SDF (free body), geom2=plane (world). Assembly swaps body indices so body2 = moving body (gets +1 sign). The contact normal (plane normal, upward) already points from lower body (world) to higher body (free body) — no negate needed.
- Default solref/solimp only — per-contact params not yet on GPU.

**Milestone:** `cargo test -p sim-gpu` — 30/30 tests pass (26 existing + 4 new). Constraint solve replaces gravity-only bridge. Contacts now affect body motion.

---

### Session 6: Pipeline orchestration + end-to-end — COMPLETE (2026-03-24)

**Goal:** Unified GPU physics pipeline with single-submit command buffer.
All substeps encoded in one command buffer per frame.

**Spec:** `sim/docs/gpu-physics-pipeline/SESSION_6_PIPELINE_ORCHESTRATION.md`

**Implemented:**
1. Sub-pipeline refactoring: split `dispatch()` into `write_params()` + `encode()` for all 6 pipelines (FK, CRBA, velocity FK, RNE, smooth, integrate). Existing `dispatch()` preserved as backward-compat wrapper.
2. Collision pipeline fix: pre-allocated per-pair params buffers + bind groups in `new()`. Fixed latent bug where `queue.write_buffer` per-pair produced "last write wins" behavior. Pre-created AABB and shared narrowphase bind groups.
3. `GpuPhysicsPipeline` orchestrator (`orchestrator.rs`): unified struct holding all 8 sub-pipelines + staging buffers. Model validation (free joints only, nv ≤ 60).
4. `encode_substep()`: per-substep buffer zeroing via `encoder.clear_buffer()` + 8 pipeline `encode()` calls.
5. `step()`: uploads state, writes params once, encodes N substeps in one command buffer, single `queue.submit()`, staging buffer readback, f32→f64 widening.
6. `GpuPipelineError` error type for validation failures.

**Validated (5 tests, all passing):**
- T28: Model validation — free-body accepted, hinge rejected
- T29: Single substep via orchestrator — z decreases under gravity
- T30: Multi-substep single-submit matches serial submits (10 steps)
- T31: GPU vs CPU trajectory comparison — 5 seconds, z divergence < 1.0m
- T32: Sustained multi-substep stress test — 2 seconds (48 batches × 10 substeps)

**Deviations from original spec:**
- GPU-compatible hockey example deferred to separate PR (requires Bevy integration, all-free-joint variant)
- Bevy plugin integration (`step_physics_realtime` GPU path) deferred
- Collision per-pair bug fix was not in original spec — discovered during implementation. The `queue.write_buffer` per-pair pattern appeared to work on Metal (writes may be sequenced on this backend) but was architecturally incorrect.
- Buffer lifetime: wgpu bind groups may not reliably prevent premature buffer drop — explicitly stored buffer references in `NarrowphaseDispatch` and `GpuCollisionPipeline`.

**Milestone:** `cargo test -p sim-gpu` — 35/35 tests pass (30 existing + 5 new). Full GPU physics pipeline available via `GpuPhysicsPipeline::new()` + `step()`.

## 15. Risks

| Risk | Mitigation |
|---|---|
| Tree scan dispatch overhead dominates | Hockey has depth 1 (~30 dispatches/substep × ~5μs = ~150μs). Profile early on 5070 Ti. |
| Backward scan f32 accumulation (no WGSL f32 atomics) | WGSL only supports `atomic<u32/i32>`. Use workgroup-local shared memory reduction for flat trees. For deep trees, use CAS loop: `atomicCompareExchangeWeak` on `u32` with `bitcast<f32>`. See §7.3. |
| Newton Cholesky fails (non-SPD H) | Regularize: H += ε·I. Fall back to CPU PGS for that frame. |
| f32 precision | MJX proves f32 Newton works. Regularization in H diagonal. Kahan summation in reductions. |
| nv > 60 exceeds shared memory for H | CG solver (iterative, no dense H). nv=60 uses ~15KB of 16KB minimum. Query device limit; NVIDIA 5070 Ti likely supports 48KB+. |
| Complex actuators not supported on GPU | Assert supported types at pipeline creation. Unsupported → CPU fallback. |
| n_env > 1 memory pressure | Tune max_contacts/max_constraints per use case. 13 MB/env is baseline. |
| Hinge FK correctness | Three critical details: left-multiply rotation (world frame), pivot-point position adjustment around jnt_pos, axis rotation to world frame. All verified against CPU position.rs. |

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
