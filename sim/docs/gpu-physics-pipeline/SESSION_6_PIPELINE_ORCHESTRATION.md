# Session 6: Pipeline Orchestration + End-to-End

**Branch:** `feature/vr-hockey`
**Date:** 2026-03-24
**Prereq:** Sessions 1–5 complete (FK, CRBA, velocity FK, RNE, smooth, collision, constraint, integration)

---

## 1 Scope

### In scope

- `GpuPhysicsPipeline` — unified struct holding all 8 sub-pipelines + staging buffers
- `encode_substep()` — encodes one full substep into an existing `CommandEncoder`
- `step()` — uploads inputs, encodes N substeps, submits once, reads back final state
- `write_all_params()` — writes all uniform params once (constant across substeps)
- Model validation: free joints only, nv ≤ 60, SDF geoms required for collision
- Sub-pipeline refactoring: split `dispatch()` into `write_params()` + `encode()` for single-submit compatibility
- Collision pipeline fix: pre-allocate per-pair params buffers (fix `queue.write_buffer` sequencing bug)
- Per-substep buffer zeroing via `encoder.clear_buffer()` (replace `queue.write_buffer` zeroing)
- GPU vs CPU trajectory comparison test (5 seconds)
- GPU-compatible hockey variant (all free joints)
- Tests T28–T32

### Out of scope

- Non-free joint support in constraint solver (hinge, ball, slide)
- Warm start (qacc reuse between substeps)
- n_env > 1 (batch environments)
- Bevy plugin integration (GPU path in `step_physics_realtime`) — deferred to post-merge
- Actuation / passive force shaders (nu=0 for hockey)
- Performance profiling / timestamp queries

---

## 2 Architecture

### 2.1 GpuPhysicsPipeline struct

```rust
pub struct GpuPhysicsPipeline {
    ctx: GpuContext,
    model_bufs: GpuModelBuffers,
    state_bufs: GpuStateBuffers,

    fk: GpuFkPipeline,
    crba: GpuCrbaPipeline,
    velocity_fk: GpuVelocityFkPipeline,
    rne: GpuRnePipeline,
    smooth: GpuSmoothPipeline,
    collision: GpuCollisionPipeline,
    constraint: GpuConstraintPipeline,
    integrate: GpuIntegratePipeline,

    // Staging buffers for readback (MAP_READ | COPY_DST)
    staging_qpos: wgpu::Buffer,
    staging_qvel: wgpu::Buffer,

    // Cached model dimensions
    nq: u32,
    nv: u32,
}
```

### 2.2 Data flow (per frame)

```
CPU → GPU:  qpos, qvel (initial state, once per frame)

┌─── Substep loop (all on GPU, single command buffer) ──────────┐
│                                                                │
│  clear:  body_cfrc, qfrc_bias, qM,                           │
│          qfrc_applied, qfrc_actuator, qfrc_passive            │
│                                                                │
│  FK           → body_xpos, body_xquat, cinert, cdof, geom     │
│  CRBA         → qM, qM_factor                                 │
│  Velocity FK  → body_cvel                                      │
│  RNE          → qfrc_bias                                      │
│  Smooth       → qfrc_smooth, qacc_smooth                      │
│  Collision    → contact_buffer, contact_count                  │
│  Constraint   → qacc, efc_force, qfrc_constraint              │
│  Integrate    → qpos (updated), qvel (updated)                │
│                                                                │
└── (qpos/qvel feed back into next substep's FK) ───────────────┘

GPU → CPU:  qpos, qvel (final state, once per frame)
```

### 2.3 Single-submit command buffer

```rust
// 1. Upload initial state (host → device)
state_bufs.upload_qpos(ctx, data);
state_bufs.upload_qvel(ctx, data);

// 2. Write all uniform params ONCE (constant across substeps)
fk.write_params(ctx, &model_bufs, &state_bufs);
crba.write_params(ctx, &model_bufs, &state_bufs);
velocity_fk.write_params(ctx, &model_bufs, &state_bufs);
rne.write_params(ctx, &model_bufs, &state_bufs, cpu_model);
smooth.write_params(ctx, &model_bufs, &state_bufs, cpu_model);
integrate.write_params(ctx, &model_bufs, &state_bufs, cpu_model);
// collision + constraint params are write-once at creation

// 3. Encode all substeps into ONE command buffer
let mut encoder = device.create_command_encoder(...);
for _ in 0..num_substeps {
    encode_substep(&mut encoder);
}

// 4. Copy final state to staging (device → staging)
encoder.copy_buffer_to_buffer(&state_bufs.qpos, 0, &staging_qpos, 0, nq * 4);
encoder.copy_buffer_to_buffer(&state_bufs.qvel, 0, &staging_qvel, 0, nv * 4);

// 5. Single submit + synchronous readback
ctx.queue.submit([encoder.finish()]);
staging_qpos.map_async(MapMode::Read, ...);
staging_qvel.map_async(MapMode::Read, ...);
device.poll(Wait);

// 6. f32 → f64 widening, copy back to Data
```

---

## 3 Critical fix: sub-pipeline single-submit compatibility

### 3.1 Problem: `queue.write_buffer` for state zeroing

Current sub-pipeline `dispatch()` methods use `queue.write_buffer` to zero
state buffers (body_cfrc, qfrc_bias, qM, force accumulators). This works
for one-submit-per-substep but breaks for multi-substep single-submit:

`queue.write_buffer` is a host-side operation that stages data to be written
at the START of the next `queue.submit()`. All writes happen before any GPU
commands execute. For N substeps in one command buffer, the zeroing only
happens once (before substep 1), not before each substep.

### 3.2 Fix: `encoder.clear_buffer()` for per-substep zeroing

Move all per-substep buffer zeroing from `queue.write_buffer` to
`encoder.clear_buffer()`, which inserts a GPU-side fill command into the
command stream, correctly sequenced between compute passes.

**Buffers that need per-substep zeroing:**

| Buffer | Current location | Size |
|--------|-----------------|------|
| `body_cfrc` | `rne.rs` dispatch() | nbody × 32 |
| `qfrc_bias` | `rne.rs` dispatch() | nv × 4 |
| `qM` | `crba.rs` dispatch() | nv² × 4 |
| `qfrc_applied` | `smooth.rs` dispatch() | nv × 4 |
| `qfrc_actuator` | `smooth.rs` dispatch() | nv × 4 |
| `qfrc_passive` | `smooth.rs` dispatch() | nv × 4 |

`contact_count` and `constraint_count` are already zeroed via
`encoder.clear_buffer()` inside `GpuCollisionPipeline::encode()` and
`GpuConstraintPipeline::encode()` respectively. No changes needed.

### 3.3 Sub-pipeline refactoring pattern

Each sub-pipeline that uses `queue.write_buffer` for params and/or zeroing
gets split into two methods:

```rust
impl GpuFooBarPipeline {
    /// Write uniform params to GPU. Call once before encoding substeps.
    pub fn write_params(&self, ctx: &GpuContext, model: &GpuModelBuffers, ...) {
        // queue.write_buffer for params only (not zeroing)
    }

    /// Encode compute passes into encoder. Call once per substep.
    pub fn encode(&self, encoder: &mut CommandEncoder, state: &GpuStateBuffers) {
        // begin_compute_pass + set_pipeline + set_bind_group + dispatch_workgroups
        // NO queue.write_buffer calls
    }

    /// Legacy dispatch: write_params + encode. Used by existing tests.
    pub fn dispatch(&self, ctx: &GpuContext, model: &GpuModelBuffers,
                    state: &GpuStateBuffers, encoder: &mut CommandEncoder) {
        self.write_params(ctx, model, ...);
        // zero state buffers via queue.write_buffer (legacy, single-submit only)
        ...
        self.encode(encoder, state);
    }
}
```

Existing tests (T1–T27) continue using `dispatch()` unchanged.
`GpuPhysicsPipeline` calls `write_params()` once + `encode()` per substep.

**Pipelines needing this split:**

| Pipeline | Has params write | Has state zeroing | Change needed |
|----------|-----------------|-------------------|---------------|
| FK | Yes (depth params) | No | Split write_params + encode |
| CRBA | Yes (depth params) | Yes (qM) | Split write_params + encode, move qM zero |
| Velocity FK | Yes (depth params) | No | Split write_params + encode |
| RNE | Yes (physics params) | Yes (cfrc, bias) | Split write_params + encode, move zeros |
| Smooth | Yes (physics params) | Yes (force accumulators) | Split write_params + encode, move zeros |
| Integrate | Yes (physics params) | No | Split write_params + encode |
| Collision | No (params per-pair at creation) | No (uses encoder.clear) | See §3.4 |
| Constraint | No (params at creation) | No (uses encoder.clear) | No change |

### 3.4 Collision pipeline: per-pair params fix

**Bug:** `GpuCollisionPipeline::encode()` calls `queue.write_buffer` to
write `NarrowphaseParams` for each pair during encoding. Since all
`queue.write_buffer` calls execute before the command buffer, only the
last pair's params are visible to all dispatches.

**Why tests pass:** T19–T22 and T26 all create scenes with ≤1 narrowphase
pair, so the "last write wins" behavior doesn't manifest.

**Fix:** Pre-allocate one `NarrowphaseParams` uniform buffer per pair at
pipeline creation time. Write all pair params once in `new()`. Each
dispatch uses its own pre-created bind group pointing to its own buffer.

```rust
struct NarrowphaseDispatch {
    bind_group: wgpu::BindGroup,      // group 0 with this pair's params
    pipeline_idx: PipelineSelector,    // sdf_sdf or sdf_plane
    workgroup_dims: [u32; 3],          // dispatch dimensions
}

pub struct GpuCollisionPipeline {
    // ...existing fields...
    dispatches: Vec<NarrowphaseDispatch>,  // replaces pairs + narrow_params_buf
}
```

`encode()` becomes a simple loop over pre-built dispatches with no
`queue.write_buffer` calls.

---

## 4 GpuPhysicsPipeline implementation

### 4.1 `new()` — model validation + pipeline creation

```rust
pub fn new(ctx: GpuContext, model: &Model, data: &Data) -> Result<Self, GpuPipelineError> {
    // ── Validation ────────────────────────────────────────────
    if model.nv > 60 {
        return Err(GpuPipelineError::NvTooLarge(model.nv));
    }
    for j in 0..model.njnt {
        if model.jnt_type[j] != MjJointType::Free {
            return Err(GpuPipelineError::UnsupportedJointType(j, model.jnt_type[j]));
        }
    }

    // ── Upload model + state ──────────────────────────────────
    let model_bufs = GpuModelBuffers::upload(&ctx, model);
    let state_bufs = GpuStateBuffers::new(&ctx, &model_bufs, data);

    // ── Create all sub-pipelines ──────────────────────────────
    let fk = GpuFkPipeline::new(&ctx, &model_bufs, &state_bufs);
    let crba = GpuCrbaPipeline::new(&ctx, &model_bufs, &state_bufs);
    let velocity_fk = GpuVelocityFkPipeline::new(&ctx, &model_bufs, &state_bufs);
    let rne = GpuRnePipeline::new(&ctx, &model_bufs, &state_bufs, model);
    let smooth = GpuSmoothPipeline::new(&ctx, &model_bufs, &state_bufs);
    let collision = GpuCollisionPipeline::new(&ctx, model, &model_bufs);
    let constraint = GpuConstraintPipeline::new(&ctx, &model_bufs, &state_bufs, model);
    let integrate = GpuIntegratePipeline::new(&ctx, &model_bufs, &state_bufs);

    // ── Staging buffers for readback ──────────────────────────
    let nq = model.nq as u32;
    let nv = model.nv as u32;
    let staging_qpos = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("staging_qpos"),
        size: u64::from(nq.max(1)) * 4,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    let staging_qvel = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("staging_qvel"),
        size: u64::from(nv.max(1)) * 4,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    Ok(Self {
        ctx, model_bufs, state_bufs,
        fk, crba, velocity_fk, rne, smooth,
        collision, constraint, integrate,
        staging_qpos, staging_qvel,
        nq, nv,
    })
}
```

### 4.2 `encode_substep()` — one substep into encoder

```rust
fn encode_substep(&self, encoder: &mut wgpu::CommandEncoder) {
    let nv = self.nv;
    let nbody = self.model_bufs.nbody;

    // ── Zero per-substep state buffers ────────────────────────
    encoder.clear_buffer(&self.state_bufs.body_cfrc, 0, None);
    encoder.clear_buffer(&self.state_bufs.qfrc_bias, 0, None);
    if nv > 0 {
        let nv_sq_bytes = u64::from(nv) * u64::from(nv) * 4;
        encoder.clear_buffer(&self.state_bufs.qm, 0, Some(nv_sq_bytes));
        encoder.clear_buffer(&self.state_bufs.qfrc_applied, 0, None);
        encoder.clear_buffer(&self.state_bufs.qfrc_actuator, 0, None);
        encoder.clear_buffer(&self.state_bufs.qfrc_passive, 0, None);
    }

    // ── 8 pipeline stages ─────────────────────────────────────
    self.fk.encode(encoder, &self.state_bufs);
    self.crba.encode(encoder, &self.state_bufs);
    self.velocity_fk.encode(encoder, &self.state_bufs);
    self.rne.encode(encoder, &self.state_bufs);
    self.smooth.encode(encoder, &self.state_bufs);
    self.collision.encode(encoder, &self.ctx, &self.model_bufs, &self.state_bufs);
    self.constraint.encode(encoder, &self.state_bufs);
    self.integrate.encode(encoder, &self.state_bufs);
}
```

### 4.3 `step()` — N substeps + readback

```rust
pub fn step(&self, cpu_model: &Model, data: &mut Data, num_substeps: u32) {
    // 1. Upload per-frame inputs (CPU → GPU)
    self.state_bufs.upload_qpos(&self.ctx, data);
    self.state_bufs.upload_qvel(&self.ctx, data);

    // 2. Write all uniform params (constant across substeps)
    self.fk.write_params(&self.ctx, &self.model_bufs, &self.state_bufs);
    self.crba.write_params(&self.ctx, &self.model_bufs, &self.state_bufs);
    self.velocity_fk.write_params(&self.ctx, &self.model_bufs, &self.state_bufs);
    self.rne.write_params(&self.ctx, &self.model_bufs, &self.state_bufs, cpu_model);
    self.smooth.write_params(&self.ctx, &self.model_bufs, &self.state_bufs, cpu_model);
    self.integrate.write_params(&self.ctx, &self.model_bufs, &self.state_bufs, cpu_model);

    // 3. Encode all substeps
    let mut encoder = self.ctx.device.create_command_encoder(
        &wgpu::CommandEncoderDescriptor { label: Some("gpu_physics_step") },
    );
    for _ in 0..num_substeps {
        self.encode_substep(&mut encoder);
    }

    // 4. Copy final state to staging
    let nq_bytes = u64::from(self.nq.max(1)) * 4;
    let nv_bytes = u64::from(self.nv.max(1)) * 4;
    encoder.copy_buffer_to_buffer(
        &self.state_bufs.qpos, 0, &self.staging_qpos, 0, nq_bytes,
    );
    encoder.copy_buffer_to_buffer(
        &self.state_bufs.qvel, 0, &self.staging_qvel, 0, nv_bytes,
    );

    // 5. Single submit
    self.ctx.queue.submit([encoder.finish()]);

    // 6. Synchronous readback
    let qpos_f32 = map_read_f32(&self.ctx, &self.staging_qpos, self.nq as usize);
    let qvel_f32 = map_read_f32(&self.ctx, &self.staging_qvel, self.nv as usize);

    // 7. f32 → f64 widening, write back to Data
    for (i, &v) in qpos_f32.iter().enumerate() {
        data.qpos[i] = f64::from(v);
    }
    for (i, &v) in qvel_f32.iter().enumerate() {
        data.qvel[i] = f64::from(v);
    }
    data.time += cpu_model.timestep * f64::from(num_substeps);
}
```

---

## 5 File structure

```
sim/L0/gpu/src/
├── pipeline/
│   ├── mod.rs               ── add: pub mod orchestrator; pub use
│   ├── orchestrator.rs      ── NEW: GpuPhysicsPipeline, GpuPipelineError
│   ├── fk.rs                ── add: write_params() + encode() (split from dispatch)
│   ├── crba.rs              ── add: write_params() + encode() (move qM zeroing out)
│   ├── velocity_fk.rs       ── add: write_params() + encode()
│   ├── rne.rs               ── add: write_params() + encode() (move cfrc/bias zeroing out)
│   ├── smooth.rs            ── add: write_params() + encode() (move force zeroing out)
│   ├── integrate.rs         ── add: write_params() + encode()
│   ├── collision.rs         ── refactor: pre-allocate per-pair params + bind groups
│   ├── constraint.rs        ── no change (encode() already correct)
│   ├── state_buffers.rs     ── no change
│   ├── model_buffers.rs     ── no change
│   ├── types.rs             ── add: GpuPipelineError enum
│   └── tests.rs             ── add: T28–T32
```

---

## 6 Implementation phases

### Phase 1: Sub-pipeline split (write_params + encode)

For each of FK, CRBA, Velocity FK, RNE, Smooth, Integrate:
1. Extract the `queue.write_buffer` param-writing code into `write_params()`
2. Extract the compute pass encoding code into `encode()`
3. Remove state buffer zeroing from both methods (zeroing moves to orchestrator)
4. Rewrite `dispatch()` to call `write_params()` + legacy zeroing + `encode()`
5. Verify: `cargo test -p sim-gpu` — all 30 existing tests pass

### Phase 2: Collision pipeline per-pair fix

1. In `GpuCollisionPipeline::new()`:
   - Allocate one uniform buffer per narrowphase dispatch (not one shared buffer)
   - Write params into each buffer at creation time
   - Create one bind group per dispatch pointing to its own buffer
2. Store as `Vec<NarrowphaseDispatch>` with pre-built bind groups
3. `encode()` loops over dispatches — no `queue.write_buffer` calls
4. Verify: `cargo test -p sim-gpu` — T19–T22 pass

### Phase 3: GpuPhysicsPipeline + tests

1. Create `orchestrator.rs` with `GpuPhysicsPipeline`, `GpuPipelineError`
2. Implement `new()` with model validation
3. Implement `encode_substep()`, `step()`
4. Add T28–T32 tests
5. Verify: `cargo test -p sim-gpu`

### Phase 4: GPU-compatible hockey variant

1. Add a `10c-hockey-gpu/` example (or modify `10b-hockey` with a `--gpu` flag)
2. All joints are free (stick is free body, not hinge)
3. Call `GpuPhysicsPipeline::new()` + `step()` instead of CPU `Data::step()`
4. Re-run CPU FK after `step()` for Bevy rendering: `data.forward_pos_vel(model, true)`
5. Verify: hockey runs, puck moves, contacts detected

---

## 7 Sub-pipeline encode() signatures

After refactoring, the encode-only methods have minimal signatures:

```rust
// FK: needs encoder only (params pre-written, bind groups pre-created)
impl GpuFkPipeline {
    pub fn encode(&self, encoder: &mut CommandEncoder, state: &GpuStateBuffers);
}

// CRBA: same pattern
impl GpuCrbaPipeline {
    pub fn encode(&self, encoder: &mut CommandEncoder, state: &GpuStateBuffers);
}

// Velocity FK: same
impl GpuVelocityFkPipeline {
    pub fn encode(&self, encoder: &mut CommandEncoder, state: &GpuStateBuffers);
}

// RNE: same
impl GpuRnePipeline {
    pub fn encode(&self, encoder: &mut CommandEncoder, state: &GpuStateBuffers);
}

// Smooth: same
impl GpuSmoothPipeline {
    pub fn encode(&self, encoder: &mut CommandEncoder, state: &GpuStateBuffers);
}

// Integrate: same
impl GpuIntegratePipeline {
    pub fn encode(&self, encoder: &mut CommandEncoder, state: &GpuStateBuffers);
}

// Collision: already has encode() with correct signature
impl GpuCollisionPipeline {
    pub fn encode(&self, encoder: &mut CommandEncoder, ctx: &GpuContext,
                  model_bufs: &GpuModelBuffers, state_bufs: &GpuStateBuffers);
}

// Constraint: already has encode() with correct signature
impl GpuConstraintPipeline {
    pub fn encode(&self, encoder: &mut CommandEncoder, state_bufs: &GpuStateBuffers);
}
```

Note: the `encode()` methods on FK, CRBA, Velocity FK, RNE, Smooth, and
Integrate do NOT take `ctx` — they don't call `queue.write_buffer`. They
only use pre-created pipelines and bind groups. The `state` parameter is
needed by collision and constraint for `encoder.clear_buffer` calls on
`contact_count` / `constraint_count`. For other pipelines, `state` is
passed for consistency but only used if the encode method needs buffer
references (e.g., for `encoder.clear_buffer` — but those are handled by
the orchestrator, not the sub-pipelines).

Actually, looking more carefully: FK, CRBA, etc. don't need `state` at
all in their encode methods — bind groups were created at construction time
pointing to the state buffers. The compute pass encoding only uses
pre-created bind groups. So the signature simplifies to:

```rust
pub fn encode(&self, encoder: &mut CommandEncoder);
```

Collision and constraint keep their existing signatures since collision
creates bind groups dynamically (for AABB) and constraint uses
`encoder.clear_buffer`.

---

## 8 Tests

### T28: Model validation

```
Create a model with a hinge joint → GpuPhysicsPipeline::new() returns
Err(UnsupportedJointType).

Create a free-body-only model → new() succeeds.

Create a model with nv=61 → new() returns Err(NvTooLarge).
```

### T29: Single substep matches manual loop

```
Setup: free body at z=10, ground plane, SDF sphere.
Run 1 substep via GpuPhysicsPipeline::step(model, data, 1).
Compare qpos/qvel against T26-style manual pipeline loop (1 step).
Tolerance: exact match (f32 bit-identical — same operations, same order).
```

### T30: Multi-substep single-submit matches multi-submit

```
Setup: same as T29.
Run 10 substeps via GpuPhysicsPipeline::step(model, data, 10).
Run 10 substeps via 10 × step(model, data, 1).
Compare final qpos/qvel.
Tolerance: exact match (encoder.clear_buffer produces same zeros as
queue.write_buffer, dispatch order identical).
```

### T31: GPU vs CPU trajectory comparison (5 seconds)

```
Setup: free SDF sphere (r=5, mass=1) dropped from z=10 onto ground plane.
timestep = 0.002, num_substeps = 1.

CPU: 2500 steps via Data::step().
GPU: 2500 steps via GpuPhysicsPipeline::step(model, data, 1) × 2500.

Compare z-position trajectory at each step:
- First 0.5s (free fall): |GPU_z - CPU_z| < 0.01m
- After 1s (bouncing/settling): |GPU_z - CPU_z| < 0.5m
  (pyramidal friction + f32 drift causes expected divergence)
- All values finite, |z| < 100 (no explosion)
```

### T32: Multi-body free scene

```
Setup: 2 free SDF spheres (different masses/sizes) + ground plane.
Both start at z=10, offset in x.
Run 100 substeps via step(model, data, 100).

Verify:
- Both spheres' z > -1 (didn't fall through ground)
- All qpos/qvel finite
- Spheres separated in x (didn't merge)
```

---

## 9 GPU-compatible hockey variant

### 9.1 Changes from original hockey

The original hockey example uses a hinge joint for the stick. The GPU
constraint solver (Session 5) only supports free joints. The GPU-compatible
variant makes ALL joints free:

| Body | Original joint | GPU variant |
|------|---------------|-------------|
| Puck | Free | Free (unchanged) |
| Stick | Revolute (hinge) | Free |
| Goal | Free | Free (unchanged) |

The stick becomes a free body. Spacebar impulse still applies angular
velocity via `qvel` injection. The stick will drift in translation (no
hinge constraint holds it in place), but the collision dynamics work
correctly — stick hits puck, puck slides on ice, contacts detected.

### 9.2 Integration with Bevy rendering

After `GpuPhysicsPipeline::step()` returns, `data.qpos` and `data.qvel`
are updated with GPU results (f32→f64 widened). Bevy rendering needs
body/geom poses (`xpos`, `xmat`, `geom_xpos`, `geom_xmat`), which are
NOT read back from GPU (too many buffers, and CPU FK is cheap).

Instead, run CPU FK after GPU step to populate Data's pose fields:

```rust
gpu_pipeline.step(model, data, num_substeps);
data.forward_pos_vel(model, true);  // CPU FK for rendering
```

This is the approach described in the master spec §10.3.

---

## 10 Error type

```rust
#[derive(Debug)]
pub enum GpuPipelineError {
    /// No GPU device available.
    NoGpu(String),
    /// Model has nv > 60 (exceeds shared memory budget for Newton solver).
    NvTooLarge(usize),
    /// Model contains a non-free joint type.
    UnsupportedJointType(usize, MjJointType),
}

impl std::fmt::Display for GpuPipelineError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoGpu(msg) => write!(f, "GPU not available: {msg}"),
            Self::NvTooLarge(nv) => write!(f, "nv={nv} exceeds GPU limit of 60"),
            Self::UnsupportedJointType(j, jt) =>
                write!(f, "joint {j} has unsupported type {jt:?} (GPU requires Free)"),
        }
    }
}

impl std::error::Error for GpuPipelineError {}
```

---

## 11 Session 5 limitations carried forward

These limitations from Session 5 constrain Session 6:

1. **Free joints only** — model validation rejects non-free joints.
   Hockey uses all free joints for GPU mode.
2. **Cold start** — qacc initialized from qacc_smooth each substep.
   No warm start reuse between substeps.
3. **Default solref/solimp** — model-level defaults via uniform.
   Per-contact params not on GPU.
4. **condim ≤ 4** — no rolling friction.
5. **Shared memory 16 KB** — MAX_NV=60 ceiling for Newton solver.
6. **n_env = 1** — single environment only.

---

## 12 Validation criteria

| Criterion | Target |
|-----------|--------|
| T28–T32 all pass | Required |
| GPU hockey runs without crash for 10 seconds | Required |
| GPU vs CPU z-trajectory divergence < 0.5m after 5s | Required |
| All qpos/qvel finite after 5 seconds | Required |
| Single queue.submit per step() call (verified by inspection) | Required |
| No queue.write_buffer calls inside encode_substep() | Required |
| Existing tests T1–T27 unchanged and passing | Required |
