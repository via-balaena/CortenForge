# Session 2: CRBA + Velocity FK

**Branch:** `feature/vr-hockey`
**Parent spec:** `sim/docs/GPU_PHYSICS_PIPELINE_SPEC.md` (§7.3, §8.2, §8.3, §14 Session 2)
**Prereq:** Session 1 complete — FK produces body_xpos, body_xquat, body_cinert, cdof
**Date:** 2026-03-24

## 1. Scope

Build the GPU CRBA (mass matrix M + dense Cholesky) and velocity FK
(body spatial velocities cvel from qvel). After this session, GPU qM
matches CPU `mj_crba()` and GPU cvel matches CPU `mj_fwd_velocity()`
within f32 tolerance.

### In scope

- `crba.wgsl` — 4 entry points: init, backward scan, M assembly, Cholesky
- `velocity_fk.wgsl` — 1 entry point: forward scan for body cvel
- CAS-atomic backward scan for composite inertia accumulation (§7.3)
- `DofModelGpu` packed struct + upload (dof_body, dof_parent, armature)
- New state buffers: body_crb, qvel, body_cvel, qM, qM_factor
- Validation tests comparing GPU vs CPU (5 tests: T8–T12)

### Out of scope

- RNE, forces, solver, integration (Sessions 3–5)
- Sparse LDL factorization (CPU-only; GPU uses dense Cholesky)
- Sleep filtering (GPU assumes all bodies awake)
- Body effective mass caching (constraint limiting — Session 5)
- Batch environments (n_env > 1) — buffers are n_env-ready, tests use n_env=1

## 2. File structure

```
sim/L0/gpu/src/
├── pipeline/
│   ├── mod.rs                    # Add: pub mod crba; pub mod velocity_fk;
│   ├── types.rs                  # Add: DofModelGpu, CrbaParams
│   ├── model_buffers.rs          # Add: dofs buffer upload
│   ├── state_buffers.rs          # Add: body_crb, qvel, body_cvel, qM, qM_factor
│   ├── crba.rs                   # NEW: GpuCrbaPipeline
│   ├── velocity_fk.rs            # NEW: GpuVelocityFkPipeline
│   └── tests.rs                  # Add: T8–T12
├── shaders/
│   ├── crba.wgsl                 # NEW: 4 entry points (~400 lines)
│   └── velocity_fk.wgsl          # NEW: 1 entry point (~150 lines)
```

## 3. GPU-side types (`types.rs` additions)

### 3.1 `DofModelGpu` — per-DOF static model data

```rust
#[repr(C)]
#[derive(Debug, Copy, Clone, Pod, Zeroable)]
pub struct DofModelGpu {
    pub body_id: u32,      // model.dof_body[i]
    pub parent: u32,       // model.dof_parent[i] (0xFFFFFFFF = none)
    pub armature: f32,     // jnt_armature[jnt] + dof_armature[i] (pre-combined)
    pub _pad: u32,
}
```

16 bytes per DOF, 16-byte aligned. For hockey nv=13: 208 bytes.

Upload at `GpuModelBuffers::upload()`:
```rust
for dof in 0..nv {
    let jnt = model.dof_jnt[dof];
    let arm = model.jnt_armature[jnt] + model.dof_armature.get(dof).copied().unwrap_or(0.0);
    dofs_cpu.push(DofModelGpu {
        body_id: model.dof_body[dof] as u32,
        parent: model.dof_parent[dof].map_or(0xFFFF_FFFFu32, |p| p as u32),
        armature: arm as f32,
        _pad: 0,
    });
}
```

WGSL struct:
```wgsl
struct DofModel {
    body_id: u32,
    parent: u32,       // 0xFFFFFFFF = no parent
    armature: f32,
    _pad: u32,
};
```

### 3.2 `CrbaParams` — per-dispatch uniform

Reuse `FkParams` (already has current_depth, nbody, njnt, ngeom, nv,
n_env, nq). All fields needed by CRBA and velocity FK are present.
No new params struct needed — just alias `FkParams` in WGSL.

### 3.3 DOF parent sentinel

```rust
pub const DOF_PARENT_NONE: u32 = 0xFFFF_FFFF;
```

## 4. crb storage format

### 4.1 Why cinert format doesn't work for accumulation

cinert from FK stores `(m, h, I_COM)` where h = xipos − xpos (COM
offset from body origin) and I_COM = rotational inertia about COM in
world frame. This format is NOT additive: `h₁ + h₂` does not give the
combined COM offset.

### 4.2 Accumulation-friendly format: (m, m·h, I_ref)

```
crb[body*3 + 0] = (m, m·h_x, m·h_y, m·h_z)
crb[body*3 + 1] = (I_ref_00, I_ref_01, I_ref_02, I_ref_11)
crb[body*3 + 2] = (I_ref_12, I_ref_22, 0, 0)
```

Where `I_ref = I_COM + m·(|h|²·I₃ − h⊗h)` is rotational inertia about
the body origin (parallel axis theorem applied).

**All 10 useful values are additive** when both operands share the same
reference point:
- `m₁ + m₂` ✓
- `m₁·h₁ + m₂·h₂` ✓ (weighted COM sum)
- `I_ref₁ + I_ref₂` ✓ (both about same origin)

### 4.3 Init conversion (cinert → crb)

Per body, in `crba_init`:
```wgsl
let m = cinert[b*3 + 0].x;
let h = cinert[b*3 + 0].yzw;
let mh = m * h;

// I_COM upper triangle from cinert
let I_COM_00 = cinert[b*3+1].x;  let I_COM_01 = cinert[b*3+1].y;
let I_COM_02 = cinert[b*3+1].z;  let I_COM_11 = cinert[b*3+1].w;
let I_COM_12 = cinert[b*3+2].x;  let I_COM_22 = cinert[b*3+2].y;

// Parallel axis: I_ref = I_COM + m*(|h|²·δ - h⊗h)
let hh = dot(h, h);
let I_ref_00 = I_COM_00 + m * (hh - h.x*h.x);
let I_ref_01 = I_COM_01 + m * (0.0 - h.x*h.y);
let I_ref_02 = I_COM_02 + m * (0.0 - h.x*h.z);
let I_ref_11 = I_COM_11 + m * (hh - h.y*h.y);
let I_ref_12 = I_COM_12 + m * (0.0 - h.y*h.z);
let I_ref_22 = I_COM_22 + m * (hh - h.z*h.z);
```

Store via `atomicStore` (crb buffer is `array<atomic<u32>>`).

### 4.4 Shift operation (child origin → parent origin)

Given child crb about child origin, d = xpos[child] − xpos[parent]:

```
h = mh / m                         // recover COM offset from child origin
h_new = h + d                       // COM offset from parent origin
mh_new = mh + m·d                   // = m·h_new

// I_ref shift: I_ref_new[r][c] = I_ref_old[r][c]
//   + m·((|h_new|²−|h|²)·δ_rc − (h_new[r]·h_new[c] − h[r]·h[c]))
```

6 upper-triangle elements, ~30 flops per child.

### 4.5 6×6 reconstruction for M assembly

The full 6×6 spatial inertia from crb = (m, mh, I_ref):

```
Upper-left 3×3:  I_ref (stored directly)
Lower-right 3×3: m·I₃
Off-diagonal:    skew(mh)
```

The matrix-vector product `buf = 6×6 · cdof` without materializing 6×6:
```wgsl
// cdof = (ω, v) stored as 2×vec4
let omega = cdof[i*2 + 0].xyz;
let v     = cdof[i*2 + 1].xyz;

// I_ref · ω (3×3 symmetric matmul)
let Iw = vec3(
    I00*omega.x + I01*omega.y + I02*omega.z,
    I01*omega.x + I11*omega.y + I12*omega.z,
    I02*omega.x + I12*omega.y + I22*omega.z,
);

buf_angular = Iw + cross(mh, v);
buf_linear  = cross(omega, mh) + m * v;
```

Dot product: `dot(cdof[j], buf) = dot(ω_j, buf_angular) + dot(v_j, buf_linear)`

## 5. CAS atomic pattern

### 5.1 WGSL implementation

```wgsl
fn atomic_add_f32(p: ptr<storage, atomic<u32>, read_write>, val: f32) {
    var old = atomicLoad(p);
    loop {
        let new_val = bitcast<u32>(bitcast<f32>(old) + val);
        let result = atomicCompareExchangeWeak(p, old, new_val);
        if result.exchanged { break; }
        old = result.old_value;
    }
}
```

### 5.2 Usage in backward scan

Each child at depth d shifts its crb to parent's frame, then
CAS-atomic-adds 10 floats into parent's crb slots. The 2 padding
floats per body are skipped (no atomic needed).

Contention analysis for hockey (flat tree, 4 bodies at depth 1):
- All 4 bodies have parent 0 → `parent == 0` guard makes backward scan
  a complete no-op. M is block-diagonal (each free body independent).

For pendulum chains: children at depth d accumulate into parent at
depth d−1. The `parent == 0` guard skips depth-1 bodies (parent is
world body). Typically 1–2 children per parent → near-zero contention.

For mixed trees: CAS contention is bounded by max fan-out at any
non-root depth level — usually 2–4 children per parent.

### 5.3 Buffer declaration

```wgsl
@group(3) @binding(0) var<storage, read_write> body_crb: array<atomic<u32>>;
```

- `crba_init`: `atomicStore(&body_crb[idx], bitcast<u32>(val))`
- `crba_backward`: `atomic_add_f32(&body_crb[parent_idx], shifted_val)`
- `crba_mass_matrix`: `bitcast<f32>(atomicLoad(&body_crb[idx]))`

## 6. Shader specifications

### 6.1 `crba.wgsl` — 4 entry points

**Bindings:**
```wgsl
// Group 0: params (uniform, dynamic offset)
@group(0) @binding(0) var<uniform> params: FkParams;

// Group 1: static model (read-only)
@group(1) @binding(0) var<storage, read> bodies: array<BodyModel>;
@group(1) @binding(1) var<storage, read> dofs: array<DofModel>;

// Group 2: FK outputs (read-only for CRBA)
@group(2) @binding(0) var<storage, read> body_xpos: array<vec4<f32>>;
@group(2) @binding(1) var<storage, read> body_cinert: array<vec4<f32>>;
@group(2) @binding(2) var<storage, read> cdof: array<vec4<f32>>;

// Group 3: CRBA outputs (read-write)
@group(3) @binding(0) var<storage, read_write> body_crb: array<atomic<u32>>;
@group(3) @binding(1) var<storage, read_write> qM: array<f32>;
@group(3) @binding(2) var<storage, read_write> qM_factor: array<f32>;
```

Total: 1 + 2 + 3 + 3 = 9 bindings (under 16-per-stage limit).

**Entry point 1: `crba_init`** — per-body parallel
```
@compute @workgroup_size(64)
fn crba_init(gid):
    if body_id >= nbody || env_id >= n_env: return
    Convert cinert[body_id] from (m, h, I_COM) to (m, m·h, I_ref)
    atomicStore 12 u32 into body_crb[body_id * 12 .. +12]
```

**Entry point 2: `crba_backward`** — per depth level (max_depth → 1)
```
@compute @workgroup_size(64)
fn crba_backward(gid):
    if body_id >= nbody || env_id >= n_env: return
    if body.depth != current_depth: return
    if body_id == 0: return
    if body.parent == 0u: return    // world body has no DOFs — skip (matches CPU)

    Load child crb (12 u32 → bitcast to f32)
    d = body_xpos[body_id] - body_xpos[parent]
    shifted = shift_crb(child_crb, d)
    CAS-atomic-add 10 floats into parent's crb slots
```

**Entry point 2b: qM zeroing** — before M assembly

The qM buffer must be zeroed before M assembly because the dof_parent
walk only writes entries for DOFs in the same kinematic tree. Cross-tree
entries (which should be zero) are never written. Zeroing is done from
Rust via `queue.write_buffer` with a zero-filled slice before encoding
the `crba_mass_matrix` dispatch. No extra shader entry point needed.

**Entry point 3: `crba_mass_matrix`** — per-DOF parallel
```
@compute @workgroup_size(64)
fn crba_mass_matrix(gid):
    let dof_i = gid.x
    if dof_i >= nv || env_id >= n_env: return

    let body_i = dofs[dof_i].body_id
    Reconstruct 6×6 from crb[body_i] (no materialization — implicit matmul)
    buf = 6×6_implicit_mul(crb[body_i], cdof[dof_i])

    // Diagonal
    qM[dof_i * nv + dof_i] = dot6(cdof[dof_i], buf) + dofs[dof_i].armature

    // Off-diagonal: walk dof_parent chain
    var current_body = body_i
    var j = dofs[dof_i].parent
    while j != DOF_PARENT_NONE:
        let body_j = dofs[j].body_id
        if body_j != current_body:
            r = body_xpos[current_body] - body_xpos[body_j]
            buf_angular += cross(r, buf_linear)   // spatial force transport
            current_body = body_j
        let m_ji = dot6(cdof[j], buf)
        qM[j * nv + dof_i] = m_ji
        qM[dof_i * nv + j] = m_ji   // symmetry
        j = dofs[j].parent
```

**Entry point 4: `crba_cholesky`** — single workgroup
```
@compute @workgroup_size(64)
fn crba_cholesky(gid):
    if gid.x != 0 || env_id >= n_env: return   // single thread

    // Copy qM → qM_factor
    for i in 0..nv:
        for j in 0..nv:
            qM_factor[i*nv + j] = qM[i*nv + j]

    // Dense Cholesky (lower triangular L such that M = L·L^T)
    for j in 0..nv:
        // Diagonal
        var sum = qM_factor[j*nv + j]
        for k in 0..j:
            sum -= qM_factor[j*nv + k] * qM_factor[j*nv + k]
        qM_factor[j*nv + j] = sqrt(max(sum, 1e-10))

        // Off-diagonal
        for i in (j+1)..nv:
            var s = qM_factor[i*nv + j]
            for k in 0..j:
                s -= qM_factor[i*nv + k] * qM_factor[j*nv + k]
            qM_factor[i*nv + j] = s / qM_factor[j*nv + j]
            qM_factor[j*nv + i] = 0.0   // zero upper triangle
```

For nv=13: ~366 flops. For nv=60: ~36K flops. Both trivially fast for a
single GPU thread.

### 6.2 `velocity_fk.wgsl` — 1 entry point

**Bindings:**
```wgsl
// Group 0: params (uniform, dynamic offset)
@group(0) @binding(0) var<uniform> params: FkParams;

// Group 1: static model (read-only)
@group(1) @binding(0) var<storage, read> bodies: array<BodyModel>;

// Group 2: state
@group(2) @binding(0) var<storage, read> body_xpos: array<vec4<f32>>;
@group(2) @binding(1) var<storage, read> cdof: array<vec4<f32>>;
@group(2) @binding(2) var<storage, read> qvel: array<f32>;
@group(2) @binding(3) var<storage, read_write> body_cvel: array<vec4<f32>>;
```

Total: 1 + 1 + 4 = 6 bindings.

**Entry point: `velocity_fk_forward`** — per depth level (0 → max_depth)
```
@compute @workgroup_size(64)
fn velocity_fk_forward(gid):
    if body_id >= nbody || env_id >= n_env: return
    if body.depth != current_depth: return

    let env_body_off = env_id * nbody
    let env_dof_off = env_id * nv

    // World body: zero velocity
    if body_id == 0:
        body_cvel[(env_body_off) * 2 + 0] = vec4(0)   // angular
        body_cvel[(env_body_off) * 2 + 1] = vec4(0)   // linear
        return

    // Transport parent velocity to this body
    let parent = body.parent
    let p_ang = body_cvel[(env_body_off + parent) * 2 + 0].xyz
    let p_lin = body_cvel[(env_body_off + parent) * 2 + 1].xyz
    let r = body_xpos[env_body_off + body_id].xyz - body_xpos[env_body_off + parent].xyz

    var omega = p_ang
    var v_lin = p_lin + cross(p_ang, r)   // lever arm effect

    // Add joint velocity contributions using cdof
    let dof_start = body.dof_adr
    let dof_count = body.dof_num
    for d in 0..dof_count:
        let dof_idx = env_dof_off + dof_start + d
        let qv = qvel[dof_idx]
        omega += cdof[dof_idx * 2 + 0].xyz * qv
        v_lin += cdof[dof_idx * 2 + 1].xyz * qv

    body_cvel[(env_body_off + body_id) * 2 + 0] = vec4(omega, 0.0)
    body_cvel[(env_body_off + body_id) * 2 + 1] = vec4(v_lin, 0.0)
```

**Key insight:** Using cdof (already computed by FK) eliminates all
joint-type branching. `cdof[d]` encodes the 6D motion subspace column
for DOF d, so `cdof[d] × qvel[d]` gives the correct velocity
contribution regardless of joint type (hinge, slide, ball, free).

## 7. Model buffers additions (`model_buffers.rs`)

### 7.1 `GpuModelBuffers` struct additions

```rust
pub struct GpuModelBuffers {
    // ... existing fields (bodies, joints, geoms, max_depth, nbody, etc.)

    /// Packed per-DOF data: `array<DofModel>` (storage, read).
    pub dofs: wgpu::Buffer,
}
```

### 7.2 Upload logic

In `GpuModelBuffers::upload()`, after existing joint/geom packing:

```rust
let dofs_cpu: Vec<DofModelGpu> = (0..nv)
    .map(|dof| {
        let jnt = model.dof_jnt[dof];
        let arm_jnt = model.jnt_armature[jnt];
        let arm_dof = model.dof_armature.get(dof).copied().unwrap_or(0.0);
        DofModelGpu {
            body_id: model.dof_body[dof] as u32,
            parent: model.dof_parent[dof].map_or(DOF_PARENT_NONE, |p| p as u32),
            armature: (arm_jnt + arm_dof) as f32,
            _pad: 0,
        }
    })
    .collect();
let dofs = upload_structs(ctx, "dofs", &dofs_cpu);
```

## 8. State buffers additions (`state_buffers.rs`)

### 8.1 New fields

```rust
pub struct GpuStateBuffers {
    // ... existing fields ...

    // CRBA state
    pub body_crb: wgpu::Buffer,    // atomic<u32> × 12 per body (3×vec4 as u32)
    pub qM: wgpu::Buffer,          // f32 × nv × nv (dense mass matrix)
    pub qM_factor: wgpu::Buffer,   // f32 × nv × nv (dense Cholesky factor)

    // Velocity FK state
    pub qvel: wgpu::Buffer,        // f32 × nv (uploaded from CPU)
    pub body_cvel: wgpu::Buffer,   // vec4<f32> × 2 per body (spatial velocity)
}
```

### 8.2 Allocation sizes

```rust
// body_crb: 12 u32 per body = 48 bytes/body (3×vec4 worth of atomic<u32>)
let body_crb = alloc(ctx, "body_crb", nbody * 48, usage_inout);

// qM: n_env × nv × nv f32 (n_env-ready, n_env=1 for Session 2)
let nv_sq = (nv * nv).max(1);
let qm_size = u64::from(n_env) * nv_sq * 4;
let qM = alloc(ctx, "qM", qm_size, usage_out);
let qM_factor = alloc(ctx, "qM_factor", qm_size, usage_out);

// qvel: nv f32
let qvel_f32 = f64s_to_f32s(data.qvel.as_slice());
let qvel = upload_init(ctx, "qvel", bytemuck::cast_slice(&qvel_f32), usage_in | COPY_SRC);

// body_cvel: 2 × vec4 per body = 32 bytes/body
let body_cvel = alloc(ctx, "body_cvel", nbody * 32, usage_inout);
```

### 8.3 Upload method

```rust
pub fn upload_qvel(&self, ctx: &GpuContext, data: &Data) {
    let qvel_f32 = f64s_to_f32s(data.qvel.as_slice());
    ctx.queue.write_buffer(&self.qvel, 0, bytemuck::cast_slice(&qvel_f32));
}
```

## 9. Rust pipeline modules

### 9.1 `GpuCrbaPipeline` (`crba.rs`)

Follows `GpuFkPipeline` pattern. 4 compute pipelines (one per entry
point), shared pipeline layout (4 bind groups), pre-allocated bind
groups + uniform buffer.

```rust
pub struct GpuCrbaPipeline {
    init_pipeline: wgpu::ComputePipeline,
    backward_pipeline: wgpu::ComputePipeline,
    mass_matrix_pipeline: wgpu::ComputePipeline,
    cholesky_pipeline: wgpu::ComputePipeline,

    params_bind_group: wgpu::BindGroup,
    model_bind_group: wgpu::BindGroup,
    fk_state_bind_group: wgpu::BindGroup,
    crba_output_bind_group: wgpu::BindGroup,

    params_buffer: wgpu::Buffer,
    max_depth: u32,
    nbody: u32,
    nv: u32,
}
```

**Dispatch sequence:**
```rust
pub fn dispatch(&self, ctx, model, state, encoder) {
    // 1. crba_init: one dispatch, all bodies parallel
    // 2. crba_backward: one dispatch per depth (max_depth → 1)
    // 2b. Zero qM via queue.write_buffer (before encoding mass_matrix)
    // 3. crba_mass_matrix: one dispatch, all DOFs parallel
    // 4. crba_cholesky: one dispatch, single workgroup
}
```

### 9.2 `GpuVelocityFkPipeline` (`velocity_fk.rs`)

Single compute pipeline, 3 bind groups.

```rust
pub struct GpuVelocityFkPipeline {
    forward_pipeline: wgpu::ComputePipeline,

    params_bind_group: wgpu::BindGroup,
    model_bind_group: wgpu::BindGroup,
    state_bind_group: wgpu::BindGroup,

    params_buffer: wgpu::Buffer,
    max_depth: u32,
    nbody: u32,
}
```

**Dispatch sequence:**
```rust
pub fn dispatch(&self, ctx, model, state, encoder) {
    // velocity_fk_forward: one dispatch per depth (0 → max_depth)
}
```

## 10. Dispatch order (full pipeline so far)

```
Per substep:
  FK dispatches                  (Session 1 — already built)
    → body_xpos, body_xquat, body_cinert, cdof, subtree_com

  CRBA dispatches                (Session 2 — this session)
    crba_init                    → body_crb (initialized from cinert)
    crba_backward × max_depth    → body_crb (accumulated, skips parent==0)
    zero qM (queue.write_buffer) → qM cleared
    crba_mass_matrix             → qM (dense mass matrix)
    crba_cholesky                → qM_factor (dense Cholesky)

  Velocity FK dispatches         (Session 2 — this session)
    velocity_fk_forward × depth  → body_cvel

  [Session 3+: RNE, forces, solver, integration]
```

All dispatches encode into a single command encoder. One `queue.submit()`
per frame (or per substep during validation).

## 11. Validation tests

### T8: qM diagonal for free body

Create a single free body (mass=2.5, inertia=(0.1, 0.2, 0.3)).
Set a non-trivial pose. Run CPU `mj_crba()` and GPU CRBA. Compare
all nv=6 diagonal entries of qM within 1e-4.

**Why this test:** Validates crb init (cinert → crb conversion),
M assembly diagonal, and armature addition. No backward scan needed
(single body).

### T9: qM for 3-link pendulum (off-diagonal + spatial force transport)

Create a 3-link hinge pendulum. Set non-trivial joint angles.
Run CPU and GPU CRBA. Compare full nv×nv qM matrix within 1e-3.

**Why this test:** The dof_parent walk crosses body boundaries,
triggering spatial force transport (`angular += cross(r, linear)`).
Validates the cross-body M[i,j] entries that would be wrong without
the transport.

### T10: qM for hockey scene (flat tree, multiple free bodies)

Create 3 free bodies at different poses. Run CPU and GPU CRBA.
Compare full qM (should be block-diagonal: each 6×6 block independent).

**Why this test:** Validates CAS atomic accumulation with multiple
children sharing parent 0. Also validates that cross-body M entries
are zero (free bodies on world body have independent DOFs).

### T11: cvel for free body with non-zero qvel

Create a free body. Set `qvel = [1, 0, 0, 0, 0, 5]` (linear x +
angular z). Run CPU and GPU velocity FK. Compare cvel within 1e-5.

**Why this test:** Basic velocity FK validation. Catches the original
missing-velocity-FK bug (cvel was zero → missing Coriolis forces).

### T12: cvel for 3-link pendulum at various qvel

Create a 3-link pendulum. Set non-zero qvel for all joints. Run CPU
and GPU. Compare all body cvel within 1e-4.

**Why this test:** Validates lever-arm transport (`v += ω × r`) and
cdof-based velocity accumulation across a deep chain. The lever arm
effect is the mechanism that produces correct Coriolis forces in RNE.

### Tolerance rationale

- Free body (1 level): 1e-5 (minimal f32 accumulation error)
- 3-link chain (3 levels): 1e-4 (cascading f32 errors through chain)
- qM with spatial transport: 1e-3 (compound f32 error in matmul + transport)

## 12. Deviations from master spec

### 12.1 crb format conversion

The master spec §8.2 pseudocode shows `crb[body] = cinert[body]` as a
direct copy. On CPU this works because cinert is a full 6×6 matrix that
is inherently additive about the reference point. On GPU, cinert stores
`(m, h, I_COM)` which is NOT additive. The `crba_init` entry point
performs the necessary conversion to `(m, m·h, I_ref)`.

### 12.2 No shared memory reduction for backward scan

The master spec §7.3 describes workgroup-local shared memory reduction
for the backward scan. For Session 2, we use direct global CAS atomics
instead. This is simpler and correct for all tree sizes. The contention
is minimal for our target scenes (hockey: 4 children at depth 1).

Shared memory optimization can be added later if profiling shows CAS
contention is a bottleneck for deeper trees.

### 12.3 Dense Cholesky via single thread

The spec §8.2 says "single workgroup, shared memory" for Cholesky.
We use a single thread within one workgroup reading/writing global
memory. For nv ≤ 60, this is trivially fast (<36K flops). Shared
memory can be added later if the global memory latency matters.

### 12.4 Velocity FK uses cdof, not joint-type branching

The CPU `mj_fwd_velocity()` branches on joint type to compute velocity
contributions. The GPU shader uses `cdof[d] × qvel[d]` directly, which
is mathematically equivalent (cdof encodes the motion subspace) but
avoids all branching. This is both simpler and more GPU-friendly.

## 13. Risk register

| Risk | Mitigation |
|------|------------|
| CAS contention on parent 0 for flat trees | Profile. If >10 retries, add shared memory reduction |
| f32 precision in Cholesky for nv=60 | Guard `sqrt(max(sum, 1e-10))`. If insufficient, use double-pump (two f32 → pseudo-f64) |
| body_crb atomic buffer may not coalesce well | All threads in backward scan access contiguous parent slots. Memory coalescing should be fine |
| Storage buffer count approaching 16-per-stage | CRBA uses 9/16. Leaves headroom for Session 3+ |
