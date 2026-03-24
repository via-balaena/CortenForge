# Session 3: RNE + Forces + Integration (Gravity-Only Sim)

**Branch:** `feature/vr-hockey`
**Parent spec:** `sim/docs/GPU_PHYSICS_PIPELINE_SPEC.md` (§8.4, §8.5, §8.6, §8.7, §8.15, §14 Session 3)
**Prereq:** Sessions 1+2 complete — FK, CRBA (M + Cholesky), velocity FK (cvel)
**Date:** 2026-03-24

## 1. Scope

Build the GPU RNE (bias forces: gravity + Coriolis + gyroscopic), smooth
force assembly, Cholesky solve for qacc_smooth, semi-implicit Euler
integration with quaternion exponential map, and a gravity-only bridge
(`qacc = qacc_smooth`). After this session, a free body dropped from
height follows the same trajectory on GPU as on CPU — the first full
GPU physics loop.

### In scope

- `rne.wgsl` — 4 entry points: gravity, forward scan (bias acc), backward scan (bias forces), project
- `smooth.wgsl` — 2 entry points: assemble qfrc_smooth, Cholesky solve for qacc_smooth
- `integrate.wgsl` — 1 entry point: semi-implicit Euler + quaternion exponential map
- `PhysicsParams` uniform struct (gravity, timestep, nv, nbody, etc.)
- New state buffers: qacc, qacc_smooth, qfrc_bias, qfrc_passive, qfrc_actuator, qfrc_smooth, qfrc_applied, body_cacc, body_cfrc
- Gravity-only bridge: `copy_buffer_to_buffer(qacc_smooth → qacc)`
- Command buffer chaining: FK → CRBA → vel_FK → RNE → smooth → integrate
- Validation tests comparing GPU vs CPU (6 tests: T13–T18)

### Out of scope

- `actuation.wgsl` — hockey has `nu=0`. Buffers zeroed. Shader deferred until actuators exist in a test scene.
- `passive.wgsl` — hockey has no joint springs/dampers. Buffers zeroed. Shader deferred similarly.
- Newton constraint solver (Session 5) — replaced by gravity-only bridge
- Collision pipeline (Session 4)
- `xfrc_applied` projection in smooth — zeroed for gravity-only. Full projection deferred.
- Batch environments (`n_env > 1`) — buffers are n_env-ready, tests use `n_env=1`
- Sleep filtering — GPU assumes all bodies awake

### Gravity-only simplification

With no actuators, no springs/dampers, and no contacts:
```
qfrc_actuator = 0
qfrc_passive  = 0
qfrc_applied  = 0
xfrc_applied  = 0
qfrc_smooth   = 0 + 0 + 0 - qfrc_bias = -qfrc_bias
qacc_smooth   = M⁻¹ · (-qfrc_bias)
qacc          = qacc_smooth   (no constraints)
```
The entire force pathway simplifies to: RNE → negate → Cholesky solve → integrate.

## 2. File structure

```
sim/L0/gpu/src/
├── pipeline/
│   ├── mod.rs                    # Add: pub mod rne; pub mod smooth; pub mod integrate;
│   ├── types.rs                  # Add: PhysicsParams
│   ├── state_buffers.rs          # Add: 9 new buffers
│   ├── rne.rs                    # NEW: GpuRnePipeline
│   ├── smooth.rs                 # NEW: GpuSmoothPipeline
│   ├── integrate.rs              # NEW: GpuIntegratePipeline
│   └── tests.rs                  # Add: T13–T18
├── shaders/
│   ├── rne.wgsl                  # NEW: ~350 lines
│   ├── smooth.wgsl               # NEW: ~150 lines
│   └── integrate.wgsl            # NEW: ~150 lines
```

## 3. GPU-side types (`types.rs` additions)

### 3.1 `PhysicsParams` — per-dispatch uniform for dynamics shaders

```rust
#[repr(C)]
#[derive(Debug, Copy, Clone, Pod, Zeroable)]
pub struct PhysicsParams {
    pub gravity: [f32; 4],    // (gx, gy, gz, 0) — vec4 for alignment
    pub timestep: f32,        // model.timestep (dt)
    pub nbody: u32,
    pub njnt: u32,
    pub nv: u32,
    pub nq: u32,
    pub n_env: u32,
    pub current_depth: u32,
    pub nu: u32,              // number of actuators (0 for hockey)
}
```

48 bytes, 16-byte aligned. Fits in a single 256-byte uniform slot.

The `current_depth` field is reused by the forward/backward tree scans
in RNE (same as `FkParams.current_depth`). The gravity vector and
timestep are new fields not present in `FkParams`.

WGSL struct:
```wgsl
struct PhysicsParams {
    gravity: vec4<f32>,
    timestep: f32,
    nbody: u32,
    njnt: u32,
    nv: u32,
    nq: u32,
    n_env: u32,
    current_depth: u32,
    nu: u32,
};
```

### 3.2 Why not extend FkParams

`FkParams` (32 bytes) is used by FK, CRBA, and velocity FK — shaders that
don't need gravity or timestep. Adding fields would waste uniform space in
those shaders and break the existing bind group layouts. A separate struct
keeps concerns separate: `FkParams` for kinematics, `PhysicsParams` for
dynamics.

## 4. State buffers additions (`state_buffers.rs`)

### 4.1 New fields

```rust
pub struct GpuStateBuffers {
    // ... existing fields (Session 1+2) ...

    // RNE state (Session 3)
    /// Bias accelerations: 2×vec4 per body [angular, linear] (like cvel).
    pub body_cacc: wgpu::Buffer,
    /// Bias forces: `atomic<u32>` × 8 per body (2×vec4 as u32, for CAS backward scan).
    pub body_cfrc: wgpu::Buffer,
    /// Bias force vector: f32 × nv (gravity + Coriolis + gyroscopic).
    pub qfrc_bias: wgpu::Buffer,

    // Force accumulators (Session 3 — zeroed for gravity-only)
    /// Applied generalized forces: f32 × nv (uploaded from CPU, zero for gravity-only).
    pub qfrc_applied: wgpu::Buffer,
    /// Actuator forces: f32 × nv (zero when nu=0).
    pub qfrc_actuator: wgpu::Buffer,
    /// Passive forces: f32 × nv (zero when no springs/dampers).
    pub qfrc_passive: wgpu::Buffer,

    // Smooth dynamics (Session 3)
    /// Total smooth force: f32 × nv.
    pub qfrc_smooth: wgpu::Buffer,
    /// Unconstrained acceleration: f32 × nv (M⁻¹ · qfrc_smooth).
    pub qacc_smooth: wgpu::Buffer,
    /// Final acceleration: f32 × nv (= qacc_smooth for gravity-only; solver output for Session 5+).
    pub qacc: wgpu::Buffer,
}
```

### 4.2 Allocation sizes

```rust
// body_cacc: 2×vec4 per body = 32 bytes/body (matching cvel layout)
let body_cacc = alloc(ctx, "body_cacc", nbody * 32, usage_inout);

// body_cfrc: atomic<u32> × 8 per body = 32 bytes/body
// 8 elements: 6 spatial force floats + 2 padding (vec4 aligned)
let body_cfrc = alloc(ctx, "body_cfrc", nbody * 32, usage_inout);

// Generalized force/acceleration vectors: nv f32 each
let nv_bytes = nv.max(1) * 4;
let qfrc_bias = alloc(ctx, "qfrc_bias", nv_bytes, usage_inout);
let qfrc_applied = alloc(ctx, "qfrc_applied", nv_bytes, usage_inout);
let qfrc_actuator = alloc(ctx, "qfrc_actuator", nv_bytes, usage_inout);
let qfrc_passive = alloc(ctx, "qfrc_passive", nv_bytes, usage_inout);
let qfrc_smooth = alloc(ctx, "qfrc_smooth", nv_bytes, usage_inout);
let qacc_smooth = alloc(ctx, "qacc_smooth", nv_bytes, usage_inout);
let qacc = alloc(ctx, "qacc", nv_bytes, usage_inout);
```

### 4.3 Zero initialization

For gravity-only, `qfrc_applied`, `qfrc_actuator`, and `qfrc_passive` are
zeroed at allocation (wgpu `mapped_at_creation: false` gives zeroed memory
on most drivers, but we zero explicitly before each substep via
`queue.write_buffer` for correctness).

## 5. RNE algorithm — GPU adaptation

### 5.1 Overview

The CPU RNE (`rne.rs`) has two parts:
1. **Gravity** — O(n) per-joint using precomputed subtree mass/COM
2. **Coriolis/gyroscopic** — O(n) Featherstone forward-backward

The GPU implementation maps these to 4 entry points dispatched sequentially.

### 5.2 Phase 1: Gravity (per-joint parallel)

Uses `subtree_mass[body]` and `subtree_com[body]` (already computed by FK
subtree backward scan in Session 1).

**Per joint (no depth dependency — fully parallel):**
```
gravity_force = -subtree_mass[body] * gravity

match jnt_type:
  Hinge:
    world_axis = rotate(body_xquat[body], jnt_axis)
    world_anchor = body_xpos[body] + rotate(body_xquat[body], jnt_pos)
    r = subtree_com[body] - world_anchor
    torque = cross(r, gravity_force)
    qfrc_bias[dof_adr] = dot(torque, world_axis)

  Slide:
    world_axis = rotate(body_xquat[body], jnt_axis)
    qfrc_bias[dof_adr] = dot(gravity_force, world_axis)

  Ball:
    world_anchor = body_xpos[body] + rotate(body_xquat[body], jnt_pos)
    r = subtree_com[body] - world_anchor
    torque = cross(r, gravity_force)
    body_torque = rotate(conjugate(body_xquat[body]), torque)
    qfrc_bias[dof_adr + 0..3] = body_torque.xyz

  Free:
    // Linear: direct gravity force
    qfrc_bias[dof_adr + 0..3] = gravity_force.xyz
    // Angular: torque from subtree COM
    jpos = vec3(qpos[qpos_adr], qpos[qpos_adr+1], qpos[qpos_adr+2])
    r = subtree_com[body] - jpos
    torque = cross(r, gravity_force)
    qfrc_bias[dof_adr + 3..6] = torque.xyz
```

**Why joint-type branching here:** Unlike velocity FK (which uses cdof to
avoid branching), gravity projection is fundamentally type-dependent. The
torque arm formula differs per joint type. Hinge needs axis projection,
slide needs force projection, ball needs body-local rotation, free needs
direct force + world-frame torque. cdof doesn't encode this.

**Buffer bindings needed:** bodies, joints, body_xpos, body_xquat, qpos,
subtree_mass, subtree_com, qfrc_bias.

### 5.3 Phase 2a: Forward scan — bias accelerations (root → leaves)

One dispatch per depth level (0 → max_depth). Uses `body_cvel` from
velocity FK.

```
per body at depth d:
  if body_id == 0:
    cacc[0] = (0,0,0,0,0,0)   // world body: zero bias acceleration
    return

  // Transport parent bias acceleration
  r = body_xpos[body] - body_xpos[parent]
  cacc.angular = cacc[parent].angular
  cacc.linear  = cacc[parent].linear + cross(cacc[parent].angular, r)

  // Velocity-product acceleration (Coriolis): v_parent ×_m v_joint
  // For each DOF on this body:
  for d in dof_adr..dof_adr+dof_num:
    v_joint_ang = cdof[d*2+0].xyz * qvel[d]
    v_joint_lin = cdof[d*2+1].xyz * qvel[d]

    // spatial_cross_motion(v_parent, v_joint):
    //   result.angular = ω_parent × v_joint_ang
    //   result.linear  = ω_parent × v_joint_lin + v_parent_lin × v_joint_ang
    let v_parent = body_cvel[parent]
    cacc.angular += cross(v_parent.angular, v_joint_ang)
    cacc.linear  += cross(v_parent.angular, v_joint_lin)
                  + cross(v_parent.linear, v_joint_ang)

  // Free joint correction: subtract ω × v_linear
  // (spatial vs. world-frame acceleration convention difference)
  if body has a free joint (dof_num == 6):
    let omega = body_cvel[body].angular
    let v_lin = vec3(qvel[dof_adr], qvel[dof_adr+1], qvel[dof_adr+2])
    cacc.linear -= cross(omega, v_lin)

  body_cacc[body] = cacc
```

**Free joint detection:** Instead of checking `jnt_type`, use
`body.dof_num == 6` as a proxy. A body with 6 DOFs starting at its
`dof_adr` is a free body. This avoids reading the joints buffer in the
forward scan.

**Buffer bindings needed:** bodies, body_xpos, body_cvel, cdof, qvel, body_cacc.

### 5.4 Phase 2b: Backward — bias forces (init + accumulation)

**IMPORTANT: Two separate dispatches, not one combined pass.**
See LIMITATIONS.md §9 for the general rule.

The CPU (`rne.rs:252-278`) does this in two separate loops:
1. Compute `cfrc[body] = I·a + v ×* (I·v)` for ALL bodies (forward loop)
2. Accumulate `cfrc[parent] += cfrc[child]` (backward loop)

On GPU, combining these in a single depth-ordered pass causes a bug:
the `atomicStore` of body d's cfrc at depth level d overwrites CAS
additions from children at depth d+1. The fix is two dispatches:

**Dispatch 1: `rne_cfrc_init` — per-body parallel (no depth ordering)**

Computes each body's own cfrc and writes via `atomicStore`. Buffer is
pre-zeroed, so `atomicStore` is safe — no prior CAS additions exist yet.

```
per body (all parallel):
  if body_id == 0: return

  I = cinert[body]
  v = body_cvel[body]
  a = body_cacc[body]

  Ia = spatial_inertia_mul(I, a)
  Iv = spatial_inertia_mul(I, v)

  // v ×* (I × v) — gyroscopic
  gyro_ang = cross(v.angular, Iv.angular) + cross(v.linear, Iv.linear)
  gyro_lin = cross(v.angular, Iv.linear)

  cfrc[body] = Ia + gyro   // atomicStore
```

**Dispatch 2: `rne_backward` — per depth level (max_depth → 1)**

Only does CAS accumulation — reads own cfrc (via `atomicLoad`), CAS-adds
into parent. Does NOT write own cfrc.

```
per body at depth d:
  if body_id == 0: return
  if body.parent == 0: return  // world body has no DOFs

  // Read own cfrc (includes contributions from deeper children
  // already CAS-added in previous depth dispatches)
  cfrc_ang = atomicLoad(cfrc[body].angular)
  cfrc_lin = atomicLoad(cfrc[body].linear)

  // CAS-atomic-add into parent
  atomic_add_f32_cfrc(parent, cfrc_ang)
  atomic_add_f32_cfrc(parent, cfrc_lin)
```

**spatial_inertia_mul:** Converts cinert `(m, h, I_COM)` to I_ref inline
via parallel axis theorem (~15 flops), then does implicit 6×6 multiply.
Same math as `crb_mul_cdof` in crba.wgsl but reads cinert directly (NOT
`body_crb`, which contains composite inertia after CRBA, not single-body).

**CAS atomic pattern for cfrc:** Exactly like `atomic_add_f32_crb` but
for the `body_cfrc` buffer (LIMITATIONS.md §8 — buffer-specific functions):
```wgsl
fn atomic_add_f32_cfrc(idx: u32, val: f32) {
    var old = atomicLoad(&body_cfrc[idx]);
    loop {
        let new_val = bitcast<u32>(bitcast<f32>(old) + val);
        let result = atomicCompareExchangeWeak(&body_cfrc[idx], old, new_val);
        if result.exchanged { break; }
        old = result.old_value;
    }
}
```

**Buffer bindings needed:** bodies, body_cinert, body_cvel,
body_cacc, body_cfrc (atomic).

### 5.5 Phase 2c: Project to joint space (per-DOF parallel)

```
per DOF d:
  body = dofs[d].body_id
  cfrc_ang = load_cfrc_f32(body, angular)
  cfrc_lin = load_cfrc_f32(body, linear)
  qfrc_bias[d] += dot(cdof[d].angular, cfrc_ang) + dot(cdof[d].linear, cfrc_lin)
```

Uses `cdof` to avoid joint-type branching — `S^T · f` = `dot(cdof, cfrc)`.
The `+=` adds the Coriolis contribution to the gravity term already
written in Phase 1.

**Buffer bindings needed:** dofs, cdof, body_cfrc, qfrc_bias.

### 5.6 cfrc buffer initialization

`body_cfrc` must be zeroed before the backward scan because the CAS
pattern reads existing values. Zero via `queue.write_buffer` before
encoding the backward dispatch (same pattern as qM zeroing in CRBA).

`body_cacc` does NOT need zeroing — every body is written exactly once
in the forward scan.

`qfrc_bias` does NOT need zeroing before gravity — Phase 1 (gravity)
writes each DOF directly (not +=). Phase 2c (project) adds to it (+=).
**Wait** — the CPU code initializes `qfrc_bias.fill(0.0)` before gravity.
And gravity uses `+=` for ball joint DOFs (3 components). So we MUST zero
`qfrc_bias` before Phase 1.

**Correction:** Zero `qfrc_bias` before the gravity dispatch.

## 6. Shader specifications

### 6.1 `rne.wgsl` — 4 entry points

**Bindings:**
```wgsl
// Group 0: physics params (dynamic uniform offset)
@group(0) @binding(0) var<uniform> params: PhysicsParams;

// Group 1: static model (read-only)
@group(1) @binding(0) var<storage, read> bodies: array<BodyModel>;
@group(1) @binding(1) var<storage, read> joints: array<JointModel>;
@group(1) @binding(2) var<storage, read> dofs: array<DofModel>;

// Group 2: FK / velocity FK outputs (read-only)
@group(2) @binding(0) var<storage, read> body_xpos: array<vec4<f32>>;
@group(2) @binding(1) var<storage, read> body_xquat: array<vec4<f32>>;
@group(2) @binding(2) var<storage, read> body_cinert: array<vec4<f32>>;
@group(2) @binding(3) var<storage, read> cdof: array<vec4<f32>>;
@group(2) @binding(4) var<storage, read> body_cvel: array<vec4<f32>>;
@group(2) @binding(5) var<storage, read> subtree_mass: array<f32>;
@group(2) @binding(6) var<storage, read> subtree_com: array<vec4<f32>>;
@group(2) @binding(7) var<storage, read> qvel: array<f32>;
@group(2) @binding(8) var<storage, read> qpos: array<f32>;

// Group 3: RNE outputs (read-write)
@group(3) @binding(0) var<storage, read_write> body_cacc: array<vec4<f32>>;
@group(3) @binding(1) var<storage, read_write> body_cfrc: array<atomic<u32>>;
@group(3) @binding(2) var<storage, read_write> qfrc_bias: array<f32>;
```

Total: 1 + 3 + 9 + 3 = 16 bindings. **At the limit.** If this exceeds
the 16-per-stage limit, move `qpos` out of Group 2 (only needed by
gravity, not by forward/backward/project) and into a sub-dispatch or
pack differently.

**Mitigation:** Each entry point only uses a subset of bindings. WGSL/naga
counts bindings used by the entry point, not all bindings in the module.
Gravity uses ~12, forward uses ~10, backward uses ~10, project uses ~7.
All under 16.

**Entry point 1: `rne_gravity`** — per-joint parallel
```
@compute @workgroup_size(64)
fn rne_gravity(gid):
    let jnt_id = gid.x
    if jnt_id >= njnt || env_id >= n_env: return

    let jnt = joints[jnt_id]
    let body_id = ... // need jnt_body — currently not in JointModelGpu!
```

**Problem:** `JointModelGpu` doesn't have `jnt_body`. It has `jtype`,
`qpos_adr`, `dof_adr`, `axis`, `pos`. We need to know which body a
joint belongs to.

**Solution:** Add `body_id: u32` to `JointModelGpu`. This replaces
`_pad` (offset 12). The struct remains 48 bytes, 16-byte aligned:
```rust
pub struct JointModelGpu {
    pub jtype: u32,
    pub qpos_adr: u32,
    pub dof_adr: u32,
    pub body_id: u32,      // was _pad
    pub axis: [f32; 4],
    pub pos: [f32; 4],
}
```

This is a backward-compatible change — the fk.wgsl shader declares
`_pad` at offset 12 but never reads it. Renaming it to `body_id` in
the WGSL struct is safe.

**Entry point 2: `rne_forward`** — per depth level (0 → max_depth)
```
@compute @workgroup_size(64)
fn rne_forward(gid):
    // Transport parent bias acc, add Coriolis, free joint correction
    // (See §5.3)
```

**Entry point 3: `rne_backward`** — per depth level (max_depth → 1)
```
@compute @workgroup_size(64)
fn rne_backward(gid):
    // Compute cfrc = I·a + v ×* (I·v), CAS accumulate into parent
    // (See §5.4)
```

**Entry point 4: `rne_project`** — per-DOF parallel
```
@compute @workgroup_size(64)
fn rne_project(gid):
    // qfrc_bias[d] += dot(cdof[d], cfrc[body])
    // (See §5.5)
```

### 6.2 `smooth.wgsl` — 2 entry points

**Bindings:**
```wgsl
// Group 0: physics params (dynamic uniform offset)
@group(0) @binding(0) var<uniform> params: PhysicsParams;

// Group 1: force inputs (read-only)
@group(1) @binding(0) var<storage, read> qfrc_bias: array<f32>;
@group(1) @binding(1) var<storage, read> qfrc_applied: array<f32>;
@group(1) @binding(2) var<storage, read> qfrc_actuator: array<f32>;
@group(1) @binding(3) var<storage, read> qfrc_passive: array<f32>;

// Group 2: mass matrix factor (read-only)
@group(2) @binding(0) var<storage, read> qM_factor: array<f32>;

// Group 3: outputs (read-write)
@group(3) @binding(0) var<storage, read_write> qfrc_smooth: array<f32>;
@group(3) @binding(1) var<storage, read_write> qacc_smooth: array<f32>;
```

Total: 1 + 4 + 1 + 2 = 8 bindings. Comfortable.

**Entry point 1: `smooth_assemble`** — per-DOF parallel
```
@compute @workgroup_size(64)
fn smooth_assemble(gid):
    let d = gid.x
    if d >= nv || env_id >= n_env: return

    qfrc_smooth[d] = qfrc_applied[d] + qfrc_actuator[d]
                    + qfrc_passive[d] - qfrc_bias[d]
```

Note: `xfrc_applied` projection is deferred (zeroed for gravity-only).

**Entry point 2: `smooth_solve`** — single thread (Cholesky solve)
```
@compute @workgroup_size(64)
fn smooth_solve(gid):
    if gid.x != 0 || env_id >= n_env: return

    let nv = params.nv
    let env_off = env_id * nv
    let env_qm_off = env_id * nv * nv

    // Copy qfrc_smooth → qacc_smooth (will be overwritten by solve)
    for i in 0..nv:
        qacc_smooth[env_off + i] = qfrc_smooth[env_off + i]

    // Forward substitution: L · y = qfrc_smooth
    // (L is lower triangular in qM_factor)
    for i in 0..nv:
        var sum = qacc_smooth[env_off + i]
        for k in 0..i:
            sum -= qM_factor[env_qm_off + i*nv + k] * qacc_smooth[env_off + k]
        qacc_smooth[env_off + i] = sum / qM_factor[env_qm_off + i*nv + i]

    // Backward substitution: L^T · x = y
    for i in (0..nv).rev():
        var sum = qacc_smooth[env_off + i]
        for k in (i+1)..nv:
            sum -= qM_factor[env_qm_off + k*nv + i] * qacc_smooth[env_off + k]
        qacc_smooth[env_off + i] = sum / qM_factor[env_qm_off + i*nv + i]
```

For nv=13: ~338 flops. Trivially fast on single GPU thread.

### 6.3 `integrate.wgsl` — 1 entry point

**Bindings:**
```wgsl
// Group 0: physics params
@group(0) @binding(0) var<uniform> params: PhysicsParams;

// Group 1: static model (read-only)
@group(1) @binding(0) var<storage, read> joints: array<JointModel>;

// Group 2: state (read-write)
@group(2) @binding(0) var<storage, read_write> qpos: array<f32>;
@group(2) @binding(1) var<storage, read_write> qvel: array<f32>;
@group(2) @binding(2) var<storage, read> qacc: array<f32>;
```

Total: 1 + 1 + 3 = 5 bindings. Light.

**Entry point: `integrate_euler`** — per-joint parallel
```
@compute @workgroup_size(64)
fn integrate_euler(gid):
    let jnt_id = gid.x
    if jnt_id >= njnt || env_id >= n_env: return

    let jnt = joints[jnt_id]
    let dt = params.timestep

    match jnt.jtype:
      Hinge | Slide:
        // Velocity: qvel[d] += dt * qacc[d]
        qvel[jnt.dof_adr] += dt * qacc[jnt.dof_adr]
        // Position: qpos[q] += dt * qvel[d]
        qpos[jnt.qpos_adr] += dt * qvel[jnt.dof_adr]

      Ball:
        // Velocity update: 3 DOFs
        for k in 0..3:
          qvel[jnt.dof_adr + k] += dt * qacc[jnt.dof_adr + k]
        // Quaternion integration via exponential map
        let omega = vec3(qvel[jnt.dof_adr], qvel[jnt.dof_adr+1], qvel[jnt.dof_adr+2])
        quat_integrate(&qpos, jnt.qpos_adr, omega, dt)

      Free:
        // Velocity update: 6 DOFs (linear 0-2, angular 3-5)
        for k in 0..6:
          qvel[jnt.dof_adr + k] += dt * qacc[jnt.dof_adr + k]
        // Position: linear (qpos 0-2 from qvel 0-2)
        for k in 0..3:
          qpos[jnt.qpos_adr + k] += dt * qvel[jnt.dof_adr + k]
        // Quaternion: angular (qpos 3-6 from qvel 3-5)
        let omega = vec3(qvel[jnt.dof_adr+3], qvel[jnt.dof_adr+4], qvel[jnt.dof_adr+5])
        quat_integrate(&qpos, jnt.qpos_adr + 3, omega, dt)
```

**Quaternion exponential map:**
```wgsl
fn quat_integrate(qpos_adr: u32, omega: vec3<f32>, dt: f32) {
    let angle = length(omega) * dt;
    if angle < 1e-10 { return; }

    let axis = omega / length(omega);
    let half_angle = angle * 0.5;
    let s = sin(half_angle);
    let c = cos(half_angle);
    // dq = (cos(θ/2), sin(θ/2) * axis) in (w,x,y,z)
    let dq = vec4(s * axis.x, s * axis.y, s * axis.z, c);  // (x,y,z,w) GPU layout

    // Read current quaternion from qpos (w,x,y,z) → swizzle to (x,y,z,w)
    let q_old = vec4(qpos[adr+1], qpos[adr+2], qpos[adr+3], qpos[adr+0]);

    // q_new = q_old * dq (right-multiply — local frame rotation)
    let q_new = quat_mul(q_old, dq);

    // Normalize
    let n = length(q_new);
    let q_norm = q_new / max(n, 1e-10);

    // Write back to qpos (x,y,z,w) → (w,x,y,z)
    qpos[adr + 0] = q_norm.w;
    qpos[adr + 1] = q_norm.x;
    qpos[adr + 2] = q_norm.y;
    qpos[adr + 3] = q_norm.z;
}
```

**Quaternion convention reminder (LIMITATIONS.md §6):**
- qpos stores `(w,x,y,z)` — MuJoCo convention
- GPU internal uses `(x,y,z,w)` — nalgebra memory layout
- FK already handles this swizzle. Integration must match.

## 7. Rust pipeline modules

### 7.1 `GpuRnePipeline` (`rne.rs`)

```rust
pub struct GpuRnePipeline {
    gravity_pipeline: wgpu::ComputePipeline,
    forward_pipeline: wgpu::ComputePipeline,
    backward_pipeline: wgpu::ComputePipeline,
    project_pipeline: wgpu::ComputePipeline,

    params_bind_group: wgpu::BindGroup,
    model_bind_group: wgpu::BindGroup,
    state_bind_group: wgpu::BindGroup,
    output_bind_group: wgpu::BindGroup,

    params_buffer: wgpu::Buffer,
    max_depth: u32,
    nbody: u32,
    njnt: u32,
    nv: u32,
}
```

**Dispatch sequence:**
```rust
pub fn dispatch(&self, ..., encoder) {
    // 0. Zero qfrc_bias and body_cfrc
    queue.write_buffer(&state.qfrc_bias, 0, &zero_nv);
    queue.write_buffer(&state.body_cfrc, 0, &zero_cfrc);

    // 1. rne_gravity: one dispatch, all joints parallel
    // 2. rne_forward: one dispatch per depth (0 → max_depth)
    // 3. rne_backward: one dispatch per depth (max_depth → 1)
    // 4. rne_project: one dispatch, all DOFs parallel
}
```

### 7.2 `GpuSmoothPipeline` (`smooth.rs`)

```rust
pub struct GpuSmoothPipeline {
    assemble_pipeline: wgpu::ComputePipeline,
    solve_pipeline: wgpu::ComputePipeline,

    params_bind_group: wgpu::BindGroup,
    forces_bind_group: wgpu::BindGroup,
    factor_bind_group: wgpu::BindGroup,
    output_bind_group: wgpu::BindGroup,

    params_buffer: wgpu::Buffer,
    nv: u32,
}
```

**Dispatch sequence:**
```rust
pub fn dispatch(&self, ..., encoder) {
    // 1. smooth_assemble: one dispatch, all DOFs parallel
    // 2. smooth_solve: one dispatch, single workgroup
}
```

### 7.3 `GpuIntegratePipeline` (`integrate.rs`)

```rust
pub struct GpuIntegratePipeline {
    euler_pipeline: wgpu::ComputePipeline,

    params_bind_group: wgpu::BindGroup,
    model_bind_group: wgpu::BindGroup,
    state_bind_group: wgpu::BindGroup,

    params_buffer: wgpu::Buffer,
    njnt: u32,
}
```

**Dispatch sequence:**
```rust
pub fn dispatch(&self, ..., encoder) {
    // 1. integrate_euler: one dispatch, all joints parallel
}
```

## 8. Model buffer changes

### 8.1 `JointModelGpu` — add `body_id`

```rust
pub struct JointModelGpu {
    pub jtype: u32,
    pub qpos_adr: u32,
    pub dof_adr: u32,
    pub body_id: u32,       // NEW (was _pad)
    pub axis: [f32; 4],
    pub pos: [f32; 4],
}
```

Upload change in `model_buffers.rs`:
```rust
JointModelGpu {
    jtype: jnt_type_to_gpu(model.jnt_type[j]),
    qpos_adr: model.jnt_qpos_adr[j] as u32,
    dof_adr: model.jnt_dof_adr[j] as u32,
    body_id: model.jnt_body[j] as u32,   // was: _pad: 0
    axis: [...],
    pos: [...],
}
```

Update `fk.wgsl` JointModel struct: rename `_pad` → `body_id`. The FK
shader never reads this field — no behavioral change.

## 9. Dispatch order (full pipeline)

```
Per substep:
  FK dispatches                  (Session 1)
    → body_xpos, body_xquat, body_cinert, cdof, subtree_com, subtree_mass

  CRBA dispatches                (Session 2)
    crba_init → crba_backward → zero qM → crba_mass_matrix → crba_cholesky
    → qM, qM_factor

  Velocity FK dispatches         (Session 2)
    velocity_fk_forward × depth  → body_cvel

  RNE dispatches                 (Session 3 — this session)
    zero qfrc_bias, zero body_cfrc
    rne_gravity                  → qfrc_bias (gravity component)
    rne_forward × depth          → body_cacc
    rne_backward × depth         → body_cfrc (CAS accumulated)
    rne_project                  → qfrc_bias (Coriolis added)

  Smooth dispatches              (Session 3 — this session)
    smooth_assemble              → qfrc_smooth
    smooth_solve                 → qacc_smooth

  Gravity-only bridge            (Session 3 — this session)
    copy_buffer_to_buffer(qacc_smooth → qacc)

  Integration dispatch           (Session 3 — this session)
    integrate_euler              → qpos, qvel (updated in-place)
```

All dispatches encode into a single command encoder. One `queue.submit()`
per substep during validation (will be one per frame in Session 6).

## 10. Gravity-only bridge

After `smooth_solve` writes `qacc_smooth`, and before `integrate_euler`
reads `qacc`:

```rust
// In the orchestration code:
encoder.copy_buffer_to_buffer(
    &state.qacc_smooth, 0,
    &state.qacc, 0,
    (nv as u64) * 4,
);
```

This is a single GPU-side buffer copy — no shader, no CPU readback. When
the Newton solver arrives (Session 5), this copy is replaced by the
solver's output writing to `qacc`.

**Note:** `copy_buffer_to_buffer` requires both buffers to have
`COPY_SRC` / `COPY_DST` usage flags. Ensure `qacc_smooth` has
`COPY_SRC` and `qacc` has `COPY_DST`.

## 11. Validation tests

### T13: qfrc_bias gravity for free body

Create a single free body (mass=2.5) at position (0, 0, 5) with
default gravity (0, 0, -9.81). Run CPU `mj_rne()` and GPU RNE. Compare
`qfrc_bias` within 1e-4.

**Expected:** `qfrc_bias[0..3]` = `(0, 0, -mass*g)` = `(0, 0, -24.525)`.
`qfrc_bias[3..6]` = `(0, 0, 0)` (COM coincides with body origin for
symmetric body → zero gravity torque).

**Why this test:** Validates gravity path for free joints. Catches sign
errors, missing `-subtree_mass * gravity` convention.

### T14: qfrc_bias Coriolis for spinning free body

Create a free body (mass=1.0, inertia=(0.1, 0.2, 0.3)). Set
`qvel = [0, 0, 0, 10, 5, 3]` (high angular velocity). Run CPU and GPU
RNE. Compare `qfrc_bias` within 1e-3.

**Expected:** Non-zero Coriolis/gyroscopic terms on rotational DOFs
(ω × Iω ≠ 0 when ω is not aligned with a principal axis).

**Why this test:** Validates the Coriolis/gyroscopic path. If `body_cvel`
is zero or the gyroscopic term is missing, Coriolis forces will be zero.

### T15: qfrc_bias for 3-link pendulum (gravity + Coriolis)

Create a 3-link hinge pendulum. Set non-trivial joint angles and
velocities. Run CPU and GPU RNE. Compare `qfrc_bias` within 1e-3.

**Why this test:** Validates forward scan (bias acceleration transport
through chain), backward scan (CAS accumulation of cfrc), and projection.
Also validates the lever arm effect in the Coriolis acceleration.

### T16: qacc_smooth for free body under gravity

Create a free body (mass=2.5). No velocities. Run full pipeline through
`smooth_solve`. Compare GPU `qacc_smooth` vs CPU `compute_qacc_smooth()`.

**Expected:** `qacc_smooth` = `M⁻¹ · (-qfrc_bias)`. For a free body
with no angular velocity: `qacc_smooth = [0, 0, -9.81, 0, 0, 0]`
(pure gravitational acceleration, no rotation).

**Why this test:** End-to-end validation from FK through Cholesky solve.
Any error in gravity sign, force assembly, or Cholesky solve will show up.

### T17: Gravity drop trajectory (2 seconds)

Create a free body (mass=1.0) at position (0, 0, 10). Run both CPU and
GPU for 2 seconds at dt=0.005 (400 steps). Compare position trajectories.

**Expected:** `z(t) = 10 - ½gt²`. After 2 seconds: z ≈ 10 - 19.62 = -9.62.
GPU and CPU should match within bounded f32 drift (NOT divergence). The
maximum position error at t=2s should be < 0.01 (1cm over 400 steps).

**Why this test:** First full end-to-end GPU physics loop. Validates that
the command buffer chain (FK → CRBA → vel_FK → RNE → smooth → bridge →
integrate) produces correct dynamics. Catches subtle errors in buffer
chaining, state update ordering, and quaternion integration.

**Implementation:** Loop on CPU: for each step, dispatch GPU pipeline,
readback qpos, compare. After all steps, compare final positions.

### T18: Quaternion stability under high angular velocity

Create a free body. Set `qvel = [0, 0, 0, 0, 0, 50]` (50 rad/s around z).
Run GPU pipeline for 1000 steps at dt=0.002. After each step, readback
quaternion from qpos. Verify quaternion norm stays within [0.99, 1.01].

**Why this test:** Quaternion exponential map must be correct. Without
normalization, quaternion drift over 1000 steps at high ω will denormalize.
Without correct integration, the body will gain/lose energy.

### Tolerance rationale

- T13 (single free body gravity): 1e-4 (minimal f32 error)
- T14 (single body Coriolis): 1e-3 (gyroscopic involves multiple cross products)
- T15 (3-link chain): 1e-3 (cascading f32 errors through chain)
- T16 (qacc_smooth): 1e-4 (Cholesky solve adds minimal error)
- T17 (trajectory): 0.01 absolute position error over 400 steps
- T18 (quat norm): 0.01 deviation from unit (tight)

## 12. Storage buffer count analysis

**rne.wgsl:** 16 bindings total, but per-entry-point:
- `rne_gravity`: params(1) + bodies,joints(2) + xpos,xquat,subtree_mass,subtree_com,qpos(5) + qfrc_bias(1) = 9
- `rne_forward`: params(1) + bodies(1) + xpos,cvel,cdof,qvel(4) + cacc(1) = 7
- `rne_backward`: params(1) + bodies(1) + xpos,cinert,cvel(3) + cacc,cfrc(2) = 7
- `rne_project`: params(1) + dofs(1) + cdof(1) + cfrc,qfrc_bias(2) = 5

All under 16 per entry point. No storage buffer limit risk.

**smooth.wgsl:** 8 bindings. Safe.

**integrate.wgsl:** 5 bindings. Safe.

## 13. Risk register

| Risk | Mitigation |
|------|------------|
| 16-binding limit for rne.wgsl | Counted per entry point — all under 16. If naga counts all module bindings, split into 2 shader files |
| CAS contention on body_cfrc for deep chains | Same mitigation as CRBA — near-zero contention for typical trees. Hockey (flat) skips backward entirely |
| Gravity double-counting | Gravity computed ONLY in rne_gravity. passive.wgsl and smooth.wgsl do not add gravity |
| Free joint correction sign error | Test T14 specifically validates Coriolis at high ω. Compare all 6 DOFs |
| Quaternion drift in integration | Test T18 runs 1000 steps at high ω. Exponential map + normalization prevents drift |
| Command buffer ordering | Validate T17 end-to-end. Any misordering would produce divergent trajectories |

## 14. Deviations from master spec

### 14.1 No actuation.wgsl or passive.wgsl

The master spec §8.5 and §8.6 describe `actuation.wgsl` and `passive.wgsl`
as Session 3 deliverables. This session defers both:

- **actuation.wgsl:** Hockey has `nu=0` (no actuators). Building and
  testing an actuation shader with zero actuators produces untestable code.
  `qfrc_actuator` is zeroed instead. The shader will be added in a future
  session when an actuator-bearing scene exists for validation.

- **passive.wgsl:** Hockey free bodies have zero joint stiffness and zero
  damping. Building a passive force shader that always returns zero is
  untestable. `qfrc_passive` is zeroed instead. The shader will be added
  when a scene with springs/dampers exists.

- **xfrc_applied projection:** `smooth.wgsl` Phase 1 in the master spec
  (§8.7) includes projection of `xfrc_applied` (body-frame Cartesian
  forces) into joint space. This requires walking ancestor DOFs via
  `mj_apply_ft`. For gravity-only validation, `xfrc_applied` is zero.
  The projection will be added when external forces are needed.

This follows the project principle: "A-grade or it doesn't ship." Code
that can't be validated doesn't ship — it ships when it can be tested.

### 14.2 Gravity-only bridge replaces Newton solver

The master spec §14 Session 3 notes this explicitly: "copy
`qacc_smooth → qacc` after smooth.wgsl (no constraints means
`qacc = qacc_smooth`)." Implemented as `encoder.copy_buffer_to_buffer` —
no extra shader.

### 14.3 RNE uses cinert directly, not body_crb

The master spec §8.4 Phase 2b shows `I = reconstruct_6x6(cinert[body])`.
After CRBA, `body_crb[body]` contains the *composite* rigid body inertia
(accumulated from subtree), not the single-body inertia. RNE needs the
single-body inertia, so it reads `body_cinert` and converts to I_ref
inline (same parallel axis math as `crba_init`, but local variables only).

### 14.4 body_cacc/body_cfrc use 2×vec4 layout

The master spec §6.3 shows `body_cacc` and `body_cfrc` as 6×f32 (24
bytes). GPU alignment requires vec4, so these use 2×vec4 (32 bytes/body)
matching `body_cvel` layout. The 2 padding floats per body are wasted.

### 14.5 JointModelGpu gains body_id

`JointModelGpu._pad` replaced with `body_id` (= `model.jnt_body[j]`).
Required by `rne_gravity` (per-joint dispatch needs to know which body)
and `integrate_euler` (per-joint dispatch). The struct stays 48 bytes.
Existing shaders (fk.wgsl) are updated to rename the field but never
read it — no behavioral change.

## 15. Implementation order

Phase A — Foundation:
1. Add `body_id` to `JointModelGpu` + update upload + update fk.wgsl struct
2. Add `PhysicsParams` to `types.rs`
3. Add 9 new state buffers to `state_buffers.rs`

Phase B — RNE:
4. Write `rne.wgsl` (4 entry points)
5. Write `rne.rs` (dispatch + bind groups)
6. Tests T13–T15 (validate RNE in isolation)

Phase C — Smooth dynamics:
7. Write `smooth.wgsl` (2 entry points)
8. Write `smooth.rs` (dispatch + bind groups)
9. Test T16 (validate qacc_smooth)

Phase D — Integration + end-to-end:
10. Write `integrate.wgsl` (1 entry point)
11. Write `integrate.rs` (dispatch + bind groups)
12. Wire gravity-only bridge (copy qacc_smooth → qacc)
13. Tests T17–T18 (end-to-end trajectory + quaternion stability)
