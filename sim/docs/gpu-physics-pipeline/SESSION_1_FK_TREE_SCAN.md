# Session 1: FK + Tree Scan Primitive

**Branch:** `feature/vr-hockey`
**Parent spec:** `sim/docs/GPU_PHYSICS_PIPELINE_SPEC.md` (§6, §7, §8.1, §14 Session 1)
**Date:** 2026-03-24

## 1. Scope

Build the GPU forward kinematics pipeline: tree scan infrastructure,
`fk.wgsl` shader, SoA buffer upload, and GPU↔CPU readback. After this
session, GPU-computed body poses match CPU `mj_fwd_position()` within
f32 tolerance.

### In scope

- Tree scan dispatch loop (forward: root→leaves, backward: leaves→root)
- `fk.wgsl` — body poses, cinert (10-float), geom poses, cdof, subtree COM
- `common.wgsl` — shared quaternion/matrix math
- Static model buffer upload (body_parent, jnt_type, etc.)
- Per-env state buffers (qpos, body_xpos, body_xquat, etc.)
- GPU→CPU readback for validation
- Validation tests comparing GPU vs CPU FK

### Out of scope

- CRBA, velocity FK, RNE, forces, solver, integration (Sessions 2–5)
- Batch environments (n_env > 1) — buffers are n_env-ready but tests use n_env=1
- Site poses (not needed on GPU — rendering uses geom poses)
- Tendon kinematics (CPU-only)
- Sleep detection (CPU-only)

## 2. File structure

```
sim/L0/gpu/src/
├── lib.rs                        # Add: pub mod pipeline;
├── pipeline/
│   ├── mod.rs                    # Pipeline module, re-exports
│   ├── types.rs                  # GPU-side Pod structs
│   ├── model_buffers.rs          # Static buffer upload from Model
│   ├── state_buffers.rs          # Per-env mutable state buffers
│   ├── fk.rs                     # FK pipeline: compile, bind, dispatch, readback
│   └── tests.rs                  # FK validation tests (cfg(test))
├── shaders/
│   ├── common.wgsl               # Shared quaternion/matrix utilities
│   └── fk.wgsl                   # Forward kinematics compute shader
```

Existing files (`context.rs`, `buffers.rs`, `collision.rs`,
`shaders/trace_surface.wgsl`) are untouched.

## 3. GPU-side types (`types.rs`)

All types are `#[repr(C)]`, `Pod`, `Zeroable` for direct GPU upload.

### 3.1 `FkParams` — per-dispatch uniform

```rust
#[repr(C)]
struct FkParams {
    current_depth: u32,   // Tree depth being processed this dispatch
    nbody: u32,           // Total body count
    njnt: u32,            // Total joint count
    ngeom: u32,           // Total geom count
    nv: u32,              // Total DOF count
    n_env: u32,           // Number of environments (1 for interactive)
    _pad: [u32; 2],       // 16-byte alignment
}
```

### 3.2 Joint type enum (GPU mapping)

The CPU enum order is `Hinge=0, Slide=1, Ball=2, Free=3` (Rust
discriminant order). The spec §6.2 uses `Free=0, Ball=1, Hinge=2,
Slide=3`. To avoid confusion, we use explicit constants in both Rust
upload and WGSL:

```
GPU_JNT_FREE  = 0u32
GPU_JNT_BALL  = 1u32
GPU_JNT_HINGE = 2u32
GPU_JNT_SLIDE = 3u32
```

Conversion at upload: `match cpu_type { Free => 0, Ball => 1, Hinge => 2, Slide => 3 }`.

### 3.3 Cinert compact format (10 floats)

```
[mass, h.x, h.y, h.z, I_00, I_01, I_02, I_11, I_12, I_22]
```

Where `h = xipos - xpos` (COM offset in world frame) and `I` is the
upper triangle of the rotational inertia about the COM in world frame
(`ximat * diag(body_inertia) * ximat^T`). This is NOT the parallel-axis
shifted version — CRBA reconstructs the full 6×6 from these 10 floats
by applying parallel axis theorem with `h`.

GPU buffer: `nbody * 10 * 4` bytes. Padded to `nbody * 12 * 4` (3×vec4)
for 16-byte alignment. The last 2 floats per body are unused padding.

**Decision: use 12 floats per body (3×vec4), not 10.** WGSL storage
buffers require 16-byte aligned access for vec4. Packing 10 floats
creates misalignment headaches. 12 floats (3×vec4) wastes 8 bytes/body
(negligible) but enables clean `array<vec4<f32>>` access:
- `cinert[body*3 + 0]` = `(mass, h.x, h.y, h.z)`
- `cinert[body*3 + 1]` = `(I_00, I_01, I_02, I_11)`
- `cinert[body*3 + 2]` = `(I_12, I_22, _pad, _pad)`

## 4. Static model buffers (`model_buffers.rs`)

### 4.1 Derived fields (computed at upload, not stored on Model)

**`body_depth: Vec<u32>`** — tree depth per body.
```
body_depth[0] = 0  (world body)
for b in 1..nbody:
    body_depth[b] = body_depth[body_parent[b]] + 1
```

**`max_depth: u32`** — `body_depth.iter().max()`. Controls the number
of forward/backward dispatch passes.

**`body_qpos_adr: Vec<u32>`** — first qpos index for this body's joints.
```
for b in 0..nbody:
    if body_jnt_num[b] > 0:
        body_qpos_adr[b] = jnt_qpos_adr[body_jnt_adr[b]]
    else:
        body_qpos_adr[b] = 0  // unused, body has no joints
```

**`body_dof_adr: Vec<u32>`** / **`body_dof_num: Vec<u32>`** — already
on Model. Upload as-is (cast to u32).

### 4.2 `GpuModelBuffers` struct

Owns all static GPU buffers. Created once at model upload.

```rust
struct GpuModelBuffers {
    // Tree structure
    body_parent: wgpu::Buffer,      // u32 × nbody
    body_depth: wgpu::Buffer,       // u32 × nbody
    body_jnt_adr: wgpu::Buffer,     // u32 × nbody
    body_jnt_num: wgpu::Buffer,     // u32 × nbody
    body_dof_adr: wgpu::Buffer,     // u32 × nbody
    body_dof_num: wgpu::Buffer,     // u32 × nbody
    body_pos: wgpu::Buffer,         // vec3<f32> × nbody (padded to 16B: vec4)
    body_quat: wgpu::Buffer,        // vec4<f32> × nbody
    body_ipos: wgpu::Buffer,        // vec3<f32> × nbody (padded to vec4)
    body_iquat: wgpu::Buffer,       // vec4<f32> × nbody
    body_mass: wgpu::Buffer,        // f32 × nbody
    body_inertia: wgpu::Buffer,     // vec3<f32> × nbody (padded to vec4)
    body_mocap_id: wgpu::Buffer,    // u32 × nbody (0xFFFFFFFF = not mocap)

    // Joint structure
    jnt_type: wgpu::Buffer,         // u32 × njnt
    jnt_axis: wgpu::Buffer,         // vec3<f32> × njnt (padded to vec4)
    jnt_pos: wgpu::Buffer,          // vec3<f32> × njnt (padded to vec4)
    jnt_body: wgpu::Buffer,         // u32 × njnt
    jnt_qpos_adr: wgpu::Buffer,     // u32 × njnt
    jnt_dof_adr: wgpu::Buffer,      // u32 × njnt

    // Geometry
    geom_body: wgpu::Buffer,        // u32 × ngeom
    geom_pos: wgpu::Buffer,         // vec3<f32> × ngeom (padded to vec4)
    geom_quat: wgpu::Buffer,        // vec4<f32> × ngeom

    // Counts (as uniform)
    max_depth: u32,
    nbody: u32,
    njnt: u32,
    ngeom: u32,
    nv: u32,
    nq: u32,
}
```

### 4.3 vec3 padding strategy

WGSL `array<vec4<f32>>` requires 16-byte stride. For vec3 fields
(body_pos, body_ipos, body_inertia, jnt_axis, jnt_pos, geom_pos),
upload as vec4 with w=0.0. This avoids misalignment and allows direct
`array<vec4<f32>>` indexing in WGSL.

Upload code pattern:
```rust
fn vec3s_to_padded_vec4s(vecs: &[Vector3<f64>]) -> Vec<[f32; 4]> {
    vecs.iter()
        .map(|v| [v.x as f32, v.y as f32, v.z as f32, 0.0])
        .collect()
}
```

### 4.4 Quaternion convention

nalgebra stores quaternions as `(x, y, z, w)` internally but constructs
as `Quaternion::new(w, x, y, z)`. WGSL will use `vec4<f32>` as
`(x, y, z, w)` — matching nalgebra's memory layout. Upload directly
from `quat.as_vector()` which gives `[x, y, z, w]`.

**Wait — verify this.** nalgebra `Quaternion` stores `[x, y, z, w]` in
memory (`.coords` field is `Vector4` with `[x, y, z, w]`). Confirmed.

Upload:
```rust
fn unit_quats_to_f32(quats: &[UnitQuaternion<f64>]) -> Vec<[f32; 4]> {
    quats.iter()
        .map(|q| {
            let c = q.as_ref().coords;  // [x, y, z, w]
            [c.x as f32, c.y as f32, c.z as f32, c.w as f32]
        })
        .collect()
}
```

WGSL quaternion ops will use `(x, y, z, w)` layout consistently.

**CRITICAL: qpos quaternion layout.**
MuJoCo/CortenForge qpos stores quaternions as `[w, x, y, z]` (w-first).
The GPU qpos buffer preserves this layout. The FK shader must swizzle
when reading: `let q = vec4(qpos[adr+1], qpos[adr+2], qpos[adr+3], qpos[adr+0])`.

## 5. Per-env state buffers (`state_buffers.rs`)

### 5.1 `GpuStateBuffers` struct

```rust
struct GpuStateBuffers {
    // Input (uploaded from CPU)
    qpos: wgpu::Buffer,            // f32 × n_env × nq

    // Output (written by FK shader)
    body_xpos: wgpu::Buffer,       // vec4<f32> × n_env × nbody
    body_xquat: wgpu::Buffer,      // vec4<f32> × n_env × nbody
    body_xmat: wgpu::Buffer,       // mat3x3 as 12 f32 × n_env × nbody (3×vec4)
    body_xipos: wgpu::Buffer,      // vec4<f32> × n_env × nbody
    body_cinert: wgpu::Buffer,     // 12 f32 × n_env × nbody (3×vec4)
    geom_xpos: wgpu::Buffer,       // vec4<f32> × n_env × ngeom
    geom_xmat: wgpu::Buffer,       // 12 f32 × n_env × ngeom (3×vec4)
    cdof: wgpu::Buffer,            // 8 f32 × n_env × nv (2×vec4, 6 used + 2 pad)
    subtree_mass: wgpu::Buffer,    // f32 × n_env × nbody
    subtree_com: wgpu::Buffer,     // vec4<f32> × n_env × nbody

    // Mocap (uploaded from CPU per frame)
    mocap_pos: wgpu::Buffer,       // vec4<f32> × n_env × nmocap
    mocap_quat: wgpu::Buffer,      // vec4<f32> × n_env × nmocap

    n_env: u32,
}
```

### 5.2 Indexing convention

All per-env buffers use: `index = env_id * stride + element_id`

For n_env=1, env_id is always 0 and the offset is zero. The n_env
dimension is architectural — present in buffer sizing and shader math,
but with n_env=1 the Y dispatch dimension is 1 and the multiply is
optimized away.

### 5.3 Buffer usages

| Buffer | Usages |
|---|---|
| qpos | STORAGE \| COPY_DST (CPU upload) |
| body_xpos, body_xquat, etc. | STORAGE \| COPY_SRC (GPU write, CPU readback) |
| mocap_pos, mocap_quat | STORAGE \| COPY_DST (CPU upload per frame) |

All output buffers also get COPY_SRC for readback during validation.
In production, only body_xpos/xquat need COPY_SRC (for Bevy rendering).

## 6. `common.wgsl` — shared math

Quaternion and matrix utilities used by fk.wgsl and all future shaders.
Quaternion layout: `vec4<f32>` = `(x, y, z, w)` throughout.

```wgsl
// ── Quaternion operations (layout: x, y, z, w) ────────────────

fn quat_mul(a: vec4<f32>, b: vec4<f32>) -> vec4<f32>
    // Hamilton product: a × b
    // Returns (x, y, z, w)

fn quat_conjugate(q: vec4<f32>) -> vec4<f32>
    // Returns (-x, -y, -z, w)

fn quat_normalize(q: vec4<f32>) -> vec4<f32>
    // Returns q / |q|, safe for near-zero (returns identity)

fn quat_rotate(q: vec4<f32>, v: vec3<f32>) -> vec3<f32>
    // Rotate vector v by quaternion q: q * (0,v) * q*
    // Optimized: avoids full quaternion multiply

fn quat_from_axis_angle(axis: vec3<f32>, angle: f32) -> vec4<f32>
    // axis must be unit length. Returns (sin(a/2)*axis, cos(a/2))

fn quat_identity() -> vec4<f32>
    // Returns (0, 0, 0, 1)

// ── Matrix operations ─────────────────────────────────────────

fn quat_to_mat3(q: vec4<f32>) -> mat3x3<f32>
    // Convert unit quaternion to 3×3 rotation matrix

fn mat3_transpose(m: mat3x3<f32>) -> mat3x3<f32>
    // WGSL mat3x3 is column-major. transpose() swaps columns/rows.
    // (WGSL has built-in transpose, but explicit for clarity)
```

## 7. `fk.wgsl` — forward kinematics shader

### 7.1 Bindings

```wgsl
// Group 0: uniform params
@group(0) @binding(0) var<uniform> params: FkParams;

// Group 1: static model (read-only storage)
@group(1) @binding(0) var<storage, read> body_parent: array<u32>;
@group(1) @binding(1) var<storage, read> body_depth: array<u32>;
@group(1) @binding(2) var<storage, read> body_jnt_adr: array<u32>;
@group(1) @binding(3) var<storage, read> body_jnt_num: array<u32>;
@group(1) @binding(4) var<storage, read> body_pos: array<vec4<f32>>;
@group(1) @binding(5) var<storage, read> body_quat: array<vec4<f32>>;
@group(1) @binding(6) var<storage, read> body_ipos: array<vec4<f32>>;
@group(1) @binding(7) var<storage, read> body_iquat: array<vec4<f32>>;
@group(1) @binding(8) var<storage, read> body_mass: array<f32>;
@group(1) @binding(9) var<storage, read> body_inertia: array<vec4<f32>>;
@group(1) @binding(10) var<storage, read> body_mocap_id: array<u32>;
@group(1) @binding(11) var<storage, read> jnt_type: array<u32>;
@group(1) @binding(12) var<storage, read> jnt_axis: array<vec4<f32>>;
@group(1) @binding(13) var<storage, read> jnt_pos: array<vec4<f32>>;
@group(1) @binding(14) var<storage, read> jnt_qpos_adr: array<u32>;
@group(1) @binding(15) var<storage, read> body_dof_adr: array<u32>;
@group(1) @binding(16) var<storage, read> body_dof_num: array<u32>;
@group(1) @binding(17) var<storage, read> jnt_dof_adr: array<u32>;
@group(1) @binding(18) var<storage, read> jnt_body: array<u32>;

// Group 2: per-env state (read-write storage)
@group(2) @binding(0) var<storage, read> qpos: array<f32>;
@group(2) @binding(1) var<storage, read_write> body_xpos: array<vec4<f32>>;
@group(2) @binding(2) var<storage, read_write> body_xquat: array<vec4<f32>>;
@group(2) @binding(3) var<storage, read_write> body_xipos: array<vec4<f32>>;
@group(2) @binding(4) var<storage, read_write> body_cinert: array<vec4<f32>>;
@group(2) @binding(5) var<storage, read_write> geom_xpos: array<vec4<f32>>;
@group(2) @binding(6) var<storage, read_write> geom_xmat: array<vec4<f32>>;
@group(2) @binding(7) var<storage, read_write> cdof: array<vec4<f32>>;
@group(2) @binding(8) var<storage, read_write> subtree_mass: array<f32>;
@group(2) @binding(9) var<storage, read_write> subtree_com: array<vec4<f32>>;
@group(2) @binding(10) var<storage, read> mocap_pos: array<vec4<f32>>;
@group(2) @binding(11) var<storage, read> mocap_quat: array<vec4<f32>>;

// Group 3: geometry model (read-only)
@group(3) @binding(0) var<storage, read> geom_body: array<u32>;
@group(3) @binding(1) var<storage, read> geom_pos: array<vec4<f32>>;
@group(3) @binding(2) var<storage, read> geom_quat: array<vec4<f32>>;
```

**Rationale for 4 bind groups:**
- Group 0: Updated every dispatch (depth level changes)
- Group 1: Set once (static model data, shared across dispatches)
- Group 2: Set once (state, read-write across dispatches)
- Group 3: Set once (geom model, only used in geom pose pass)

wgpu allows setting groups independently — only group 0 changes per
dispatch, avoiding redundant bind group switches.

### 7.2 Entry points

Three entry points, dispatched in sequence:

**`fk_forward`** — one dispatch per depth level (d=0 to max_depth):
```
@compute @workgroup_size(64)
fn fk_forward(@builtin(global_invocation_id) gid: vec3<u32>)
    body_id = gid.x
    env_id = gid.y
    if body_id >= nbody || env_id >= n_env: return
    if body_depth[body_id] != current_depth: return

    // Depth 0: write identity and zero cinert, return
    if body_id == 0:
        write identity pose, zero cinert
        return

    // Read parent pose
    parent = body_parent[body_id]
    pos = body_xpos[parent].xyz
    quat = body_xquat[parent]

    // Check mocap
    mocap_id = body_mocap_id[body_id]
    if mocap_id != 0xFFFFFFFF:
        pos = mocap_pos[mocap_id].xyz
        quat = quat_normalize(mocap_quat[mocap_id])
    else:
        // Apply body offset
        pos += quat_rotate(quat, body_pos[body_id].xyz)
        quat = quat_mul(quat, body_quat[body_id])

        // Joint loop
        for j in body_jnt_adr[body_id] .. + body_jnt_num[body_id]:
            adr = jnt_qpos_adr[j]
            switch jnt_type[j]:
                case HINGE:
                    // See §7.3 below
                case SLIDE:
                    pos += quat_rotate(quat, jnt_axis[j].xyz) * qpos[adr]
                case BALL:
                    // qpos is [w, x, y, z] → swizzle to (x, y, z, w)
                    let q = quat_normalize(vec4(qpos[adr+1], qpos[adr+2],
                                                qpos[adr+3], qpos[adr+0]))
                    quat = quat_mul(quat, q)  // RIGHT multiply (local frame)
                case FREE:
                    pos = vec3(qpos[adr], qpos[adr+1], qpos[adr+2])
                    quat = quat_normalize(vec4(qpos[adr+4], qpos[adr+5],
                                               qpos[adr+6], qpos[adr+3]))

        quat = quat_normalize(quat)

    // Write pose
    body_xpos[body_id] = vec4(pos, 0.0)
    body_xquat[body_id] = quat

    // Derived quantities
    xipos = pos + quat_rotate(quat, body_ipos[body_id].xyz)
    body_xipos[body_id] = vec4(xipos, 0.0)

    // Cinert (10 floats in 12-float buffer)
    compute_cinert(body_id, pos, quat, xipos)

    // cdof — motion subspace for each DOF on this body
    compute_cdof(body_id, pos, quat)

    // Subtree COM init
    subtree_mass[body_id] = body_mass[body_id]
    subtree_com[body_id] = vec4(body_mass[body_id] * xipos, 0.0)
```

**`fk_geom_poses`** — one dispatch after FK completes (all geoms parallel):
```
@compute @workgroup_size(64)
fn fk_geom_poses(@builtin(global_invocation_id) gid: vec3<u32>)
    geom_id = gid.x
    env_id = gid.y
    if geom_id >= ngeom || env_id >= n_env: return

    bid = geom_body[geom_id]
    pos = body_xpos[bid].xyz
    quat = body_xquat[bid]

    geom_xpos[geom_id] = vec4(pos + quat_rotate(quat, geom_pos[geom_id].xyz), 0.0)
    geom_xmat[geom_id * 3 + 0] = ... // 3×vec4 from quat_to_mat3(quat_mul(quat, geom_quat[geom_id]))
```

**`fk_subtree_com`** — backward tree scan (d=max_depth down to 1),
then a normalization pass:
```
@compute @workgroup_size(64)
fn fk_subtree_backward(@builtin(global_invocation_id) gid: vec3<u32>)
    body_id = gid.x
    if body_depth[body_id] != current_depth || body_id == 0: return

    parent = body_parent[body_id]
    // Accumulate into parent — for flat trees (hockey), all children
    // share parent 0, no contention since each body_id is unique and
    // this is a sequential accumulation pattern per parent.
    //
    // For deeper trees where multiple children share a parent at the
    // same depth, use atomicAdd-via-CAS pattern (§7.3 of parent spec).
    // For Session 1, we use a simple serial accumulation since n_env=1
    // and tree depths are small. Optimize in Session 2 if needed.
    subtree_mass[parent] += subtree_mass[body_id]
    subtree_com[parent] += subtree_com[body_id]

@compute @workgroup_size(64)
fn fk_subtree_normalize(@builtin(global_invocation_id) gid: vec3<u32>)
    body_id = gid.x
    if body_id >= nbody: return
    if subtree_mass[body_id] > 1e-10:
        subtree_com[body_id] /= subtree_mass[body_id]
    else:
        subtree_com[body_id] = body_xipos[body_id]
```

### 7.3 Hinge joint — critical details

Verified against CPU `position.rs:61-82`. The GPU must do exactly:

```
world_axis = quat_rotate(quat, jnt_axis[j].xyz)
world_anchor = pos + quat_rotate(quat, jnt_pos[j].xyz)
rot = quat_from_axis_angle(normalize(world_axis), qpos[adr])
quat = quat_mul(rot, quat)           // LEFT multiply (world-frame rotation)
pos = world_anchor + quat_rotate(rot, pos - world_anchor)  // pivot-point
```

Key points that can silently break:
1. **LEFT multiply** `rot × quat`, not `quat × rot`. The axis is in
   world frame, so rotation is applied in world frame.
2. **Axis normalization.** The CPU uses `Unit::try_new(world_axis, 1e-10)`.
   The GPU should normalize with a degenerate guard.
3. **Pivot-point formula.** Position rotates around the world-space
   anchor, not body origin.

### 7.4 Ball joint — match CPU, not spec

The parent spec §8.1 shows a world-frame LEFT multiply with conjugates
for Ball joints. This is the MJX convention. Our CPU (`position.rs:90-98`)
does a simpler RIGHT multiply: `quat *= q` (local-frame rotation).

**Decision: match the CPU exactly.** The GPU does:
```
q = quat_normalize(swizzle_wxyz_to_xyzw(qpos[adr..adr+4]))
quat = quat_mul(quat, q)   // RIGHT multiply
```

No anchor rotation for Ball (CPU doesn't apply jnt_pos for Ball joints
in FK — the joint anchor only matters for cdof computation).

### 7.5 Cinert computation

```wgsl
fn compute_cinert(body_id: u32, pos: vec3<f32>, quat: vec4<f32>, xipos: vec3<f32>) {
    let iquat = quat_mul(quat, body_iquat[body_id]);
    let ximat = quat_to_mat3(iquat);

    let inertia = body_inertia[body_id].xyz;  // diagonal

    // I_world = ximat * diag(inertia) * ximat^T
    // Compute upper triangle only (symmetric)
    var i_rot: mat3x3<f32>;
    for (var row = 0u; row < 3u; row++) {
        for (var col = row; col < 3u; col++) {
            i_rot[col][row] = ximat[0][row] * inertia.x * ximat[0][col]
                            + ximat[1][row] * inertia.y * ximat[1][col]
                            + ximat[2][row] * inertia.z * ximat[2][col];
        }
    }

    let h = xipos - pos;
    let mass = body_mass[body_id];

    // Pack into 12 floats (3 × vec4)
    body_cinert[body_id * 3u + 0u] = vec4(mass, h.x, h.y, h.z);
    body_cinert[body_id * 3u + 1u] = vec4(i_rot[0][0], i_rot[1][0], i_rot[2][0], i_rot[1][1]);
    body_cinert[body_id * 3u + 2u] = vec4(i_rot[2][1], i_rot[2][2], 0.0, 0.0);
}
```

### 7.6 cdof computation

Per DOF on this body, compute the 6-float motion subspace in world frame.
Stored as 2×vec4 per DOF (6 floats used, 2 padding).

```wgsl
fn compute_cdof(body_id: u32, pos: vec3<f32>, quat: vec4<f32>) {
    let dof_start = body_dof_adr[body_id];
    let n_dof = body_dof_num[body_id];
    let jnt_start = body_jnt_adr[body_id];
    let n_jnt = body_jnt_num[body_id];

    var dof_idx = 0u;
    for (var j = jnt_start; j < jnt_start + n_jnt; j++) {
        let jtype = jnt_type[j];
        let dof_base = dof_start + dof_idx;

        switch jtype {
            case HINGE: {
                let axis = quat_rotate(quat, jnt_axis[j].xyz);
                let anchor = pos + quat_rotate(quat, jnt_pos[j].xyz);
                let r = pos - anchor;
                let lin = cross(axis, r);
                cdof[dof_base * 2u + 0u] = vec4(axis, 0.0);      // angular
                cdof[dof_base * 2u + 1u] = vec4(lin, 0.0);        // linear
                dof_idx += 1u;
            }
            case SLIDE: {
                let axis = quat_rotate(quat, jnt_axis[j].xyz);
                cdof[dof_base * 2u + 0u] = vec4(0.0, 0.0, 0.0, 0.0);
                cdof[dof_base * 2u + 1u] = vec4(axis, 0.0);
                dof_idx += 1u;
            }
            case BALL: {
                let rot = quat_to_mat3(quat);
                for (var k = 0u; k < 3u; k++) {
                    cdof[(dof_base + k) * 2u + 0u] = vec4(rot[k], 0.0);  // angular = column k
                    cdof[(dof_base + k) * 2u + 1u] = vec4(0.0, 0.0, 0.0, 0.0);
                }
                dof_idx += 3u;
            }
            case FREE: {
                // Linear DOFs (0-2): world basis
                for (var k = 0u; k < 3u; k++) {
                    cdof[(dof_base + k) * 2u + 0u] = vec4(0.0, 0.0, 0.0, 0.0);
                    var e = vec3(0.0, 0.0, 0.0);
                    e[k] = 1.0;
                    cdof[(dof_base + k) * 2u + 1u] = vec4(e, 0.0);
                }
                // Angular DOFs (3-5): rotated body axes
                let rot = quat_to_mat3(quat);
                for (var k = 0u; k < 3u; k++) {
                    cdof[(dof_base + 3u + k) * 2u + 0u] = vec4(rot[k], 0.0);
                    cdof[(dof_base + 3u + k) * 2u + 1u] = vec4(0.0, 0.0, 0.0, 0.0);
                }
                dof_idx += 6u;
            }
            default: {}
        }
    }
}
```

## 8. Dispatch sequence (`fk.rs`)

### 8.1 `GpuFkPipeline` struct

```rust
struct GpuFkPipeline {
    fk_pipeline: wgpu::ComputePipeline,
    geom_pipeline: wgpu::ComputePipeline,
    subtree_backward_pipeline: wgpu::ComputePipeline,
    subtree_normalize_pipeline: wgpu::ComputePipeline,

    // Bind group layouts (one per group)
    params_layout: wgpu::BindGroupLayout,    // group 0
    model_layout: wgpu::BindGroupLayout,     // group 1
    state_layout: wgpu::BindGroupLayout,     // group 2
    geom_model_layout: wgpu::BindGroupLayout, // group 3

    // Bind groups (created once)
    model_bind_group: wgpu::BindGroup,
    state_bind_group: wgpu::BindGroup,
    geom_model_bind_group: wgpu::BindGroup,

    // Per-depth params uniform buffer (rewritten each dispatch)
    params_buffer: wgpu::Buffer,
}
```

### 8.2 Dispatch loop

```rust
fn dispatch_fk(
    &self,
    ctx: &GpuContext,
    encoder: &mut wgpu::CommandEncoder,
    model: &GpuModelBuffers,
    state: &GpuStateBuffers,
) {
    let ceil64 = |n: u32| -> u32 { (n + 63) / 64 };

    // Forward FK: one pass per depth level
    for depth in 0..=model.max_depth {
        // Update params uniform
        let params = FkParams {
            current_depth: depth,
            nbody: model.nbody,
            njnt: model.njnt,
            ngeom: model.ngeom,
            nv: model.nv,
            n_env: state.n_env,
            _pad: [0; 2],
        };
        ctx.queue.write_buffer(&self.params_buffer, 0, bytemuck::bytes_of(&params));

        let mut pass = encoder.begin_compute_pass(&default_desc("fk_forward"));
        pass.set_pipeline(&self.fk_pipeline);
        pass.set_bind_group(0, &self.params_bind_group(depth), &[]);
        pass.set_bind_group(1, &self.model_bind_group, &[]);
        pass.set_bind_group(2, &self.state_bind_group, &[]);
        pass.dispatch_workgroups(ceil64(model.nbody), state.n_env, 1);
    }

    // Geom poses: one dispatch (all geoms × all envs)
    {
        let mut pass = encoder.begin_compute_pass(&default_desc("fk_geom"));
        pass.set_pipeline(&self.geom_pipeline);
        // groups 0-3 already set, only need group 0 updated for ngeom
        pass.dispatch_workgroups(ceil64(model.ngeom), state.n_env, 1);
    }

    // Subtree COM: backward scan
    for depth in (1..=model.max_depth).rev() {
        // Update params with current_depth = depth
        // ...
        let mut pass = encoder.begin_compute_pass(&default_desc("subtree_backward"));
        pass.set_pipeline(&self.subtree_backward_pipeline);
        pass.dispatch_workgroups(ceil64(model.nbody), state.n_env, 1);
    }

    // Subtree COM: normalize
    {
        let mut pass = encoder.begin_compute_pass(&default_desc("subtree_normalize"));
        pass.set_pipeline(&self.subtree_normalize_pipeline);
        pass.dispatch_workgroups(ceil64(model.nbody), state.n_env, 1);
    }
}
```

### 8.3 Params uniform update strategy

The `FkParams` uniform is 32 bytes. Between dispatches we need to update
`current_depth`. Two options:

**Option A: `queue.write_buffer` between passes.** Simple but requires
the encoder to be submitted between writes (write_buffer is a queue op,
not an encoder op). This means one `queue.submit()` per depth level —
too many submissions.

**Option B: Dynamic offset into a pre-allocated uniform buffer.** Allocate
`(max_depth + 1) * 256` bytes (256 = min uniform buffer offset alignment).
Write all depth params at creation. Use `set_bind_group(0, &bg, &[offset])`
with dynamic offsets. One encoder, one submit.

**Decision: Option B.** Pre-allocate a uniform buffer with one 256-byte
slot per depth level. Write all slots at FK dispatch time (before
encoding). Use dynamic offsets to select the slot per dispatch.

```rust
// Pre-allocate: (max_depth + 1) slots × 256 bytes each
let params_buf_size = (max_depth as u64 + 1) * 256;

// Write all slots before encoding
for depth in 0..=max_depth {
    let params = FkParams { current_depth: depth, ... };
    let offset = depth as u64 * 256;
    ctx.queue.write_buffer(&self.params_buffer, offset, bytemuck::bytes_of(&params));
}

// During encoding, use dynamic offset
pass.set_bind_group(0, &params_bind_group, &[depth * 256]);
```

## 9. Readback (`fk.rs`)

For validation, readback body_xpos and body_xquat to CPU:

```rust
fn readback_body_poses(
    ctx: &GpuContext,
    state: &GpuStateBuffers,
    nbody: u32,
) -> (Vec<[f32; 4]>, Vec<[f32; 4]>) {
    // Create staging buffers, copy, map, read — same pattern as
    // GpuSdfGrid::readback() in buffers.rs.
}
```

Also readback cinert (12 floats × nbody) and cdof (8 floats × nv) for
validation.

## 10. Validation tests (`tests.rs`)

All tests skip gracefully if no GPU is available (`GpuContext::new()`
returns Err).

### T1: Flat tree — hockey scene

Build a hockey-like model: world body + 2 free bodies (puck + paddle).
Set qpos to non-origin values. Run CPU FK, run GPU FK, compare:
- `body_xpos`: max component error < 1e-5
- `body_xquat`: max component error < 1e-5
- `geom_xpos`: max component error < 1e-5

### T2: Deep chain — 7-body serial hinge chain

Build a 7-body chain: body 0 → body 1 (hinge) → body 2 (hinge) → ... → body 7.
Each hinge has non-zero `jnt_pos` (anchor offset). Set qpos angles to
various values (0, π/4, π/2). Run CPU FK, run GPU FK, compare:
- `body_xpos`: max error < 1e-4 (f32 accumulation through 7 levels)
- Validates: level-order scan, hinge pivot-point, LEFT multiply, axis rotation

### T3: Mixed joints

Build a model with Free + Hinge + Ball + Slide joints. Verify GPU
matches CPU for all joint types.

### T4: Cinert validation

For the T1 and T2 models, readback GPU cinert (12 floats per body).
Reconstruct the full 6×6 spatial inertia on CPU and compare against
`data.cinert[body_id]`:
- Max element error < 1e-4

### T5: cdof validation

For T2 (hinge chain), readback GPU cdof (6 floats per DOF). Compare
against CPU `joint_motion_subspace()` output:
- Max element error < 1e-5

### T6: Subtree COM

For a model with non-uniform masses, compare GPU subtree_com vs CPU:
- Max error < 1e-4

### T7: Mocap body

Build a model with one mocap body. Set mocap_pos/mocap_quat. Verify
GPU FK reads mocap arrays correctly.

## 11. Tolerance rationale

**1e-5 for single-level operations** (flat tree, direct pose computation):
f32 has ~7 decimal digits. One level of FK arithmetic (rotate, add)
introduces ~1 ULP of error per operation. With ~10 operations, error is
~10 ULP ≈ 1e-6 for values of order 1. Using 1e-5 gives 10× headroom.

**1e-4 for deep chains** (7 levels): Error accumulates through the tree.
Each level adds ~1e-6 relative error. After 7 levels: ~7e-6. Using 1e-4
gives >10× headroom for values of order 1–10.

**1e-4 for cinert**: The cinert computation involves matrix multiply
(quat→mat→R·diag·R^T) which has more floating-point operations than
basic FK. 1e-4 is appropriate.

## 12. Implementation order

1. `common.wgsl` — quaternion/matrix math (test with a trivial compute
   shader that rotates a vector and readback)
2. `types.rs` — Pod structs
3. `model_buffers.rs` — compute derived fields, upload static data
4. `state_buffers.rs` — allocate per-env buffers, upload qpos
5. `fk.wgsl` — shader (FK forward only, no geom/subtree yet)
6. `fk.rs` — pipeline creation, dispatch loop, readback
7. T1 + T2 tests (body poses only)
8. Add geom poses to fk.wgsl, T1 geom test
9. Add cinert to fk.wgsl, T4 test
10. Add cdof to fk.wgsl, T5 test
11. Add subtree COM (backward scan), T6 test
12. T3 (mixed joints), T7 (mocap)

## 13. Open questions

1. **Subtree COM backward scan contention.** For flat trees, all children
   accumulate into parent 0. With `@workgroup_size(64)` and multiple
   threads targeting the same parent, we need atomic accumulation or
   serial fallback. For Session 1 with small models (n < 64), a single
   workgroup handles all bodies and we can use workgroup-local shared
   memory. Defer the CAS pattern to Session 2 (CRBA backward scan).

2. **wgpu bind group limit.** wgpu has a default limit of 4 bind groups
   (matching WebGPU). Our design uses exactly 4. If we need more for
   future shaders, we'll pack buffers.

3. **geom_xmat storage.** Storing as 3×vec4 (12 floats) instead of
   mat3x3 (9 floats) wastes 12 bytes per geom. Negligible for hockey
   (4 geoms × 12 bytes = 48 bytes). Worth the alignment simplicity.
