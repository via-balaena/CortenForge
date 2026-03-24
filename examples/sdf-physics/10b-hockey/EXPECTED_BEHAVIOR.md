# SDF Physics 10b — VR Hockey

## Vision

Hold a hockey stick in VR (Quest 3 controllers) and hit a puck across
ice into a goal. The full physics pipeline runs on GPU — FK through
integration in a single command buffer per frame. Data stays on-device
between substeps. CPU readback happens only once per frame for rendering.

## Current status

### GPU physics pipeline — COMPLETE (Sessions 1–6)

The full GPU physics pipeline is implemented and tested in `sim-gpu`:

| Stage | Shader(s) | Session |
|-------|-----------|---------|
| Forward kinematics | `fk.wgsl` | 1 |
| CRBA (mass matrix + Cholesky) | `crba.wgsl` | 2 |
| Velocity FK (body spatial velocities) | `velocity_fk.wgsl` | 2 |
| RNE (bias forces) | `rne.wgsl` | 3 |
| Smooth dynamics | `smooth.wgsl` | 3 |
| Integration (semi-implicit Euler) | `integrate.wgsl` | 3 |
| Collision (AABB + SDF narrowphase) | `aabb.wgsl`, `sdf_*_narrow.wgsl` | 4 |
| Constraint solve (Newton solver) | `assemble.wgsl`, `newton_solve.wgsl`, `map_forces.wgsl` | 5 |
| Pipeline orchestration | `GpuPhysicsPipeline` | 6 |

**Key facts:**
- Newton solver (not Jacobi) — primal, 1–3 iterations, shared-memory Hessian
- Pre-computed pair plan (not spatial hash broadphase) — pairs known at model creation
- Single `queue.submit()` per frame, N substeps encoded in one command buffer
- 35 GPU tests passing
- Free joints only (constraint solver limitation)
- nv ≤ 60 (shared memory budget)

**API:**
```rust
let gpu = sim_gpu::GpuPhysicsPipeline::new(&model, &data)?;
gpu.step(&model, &mut data, num_substeps);   // single submit
data.forward_pos_vel(&model, true);           // CPU FK for rendering
```

Spec: `sim/docs/GPU_PHYSICS_PIPELINE_SPEC.md`

### What remains

1. **Stick as mocap body** — controller pose drives stick kinematically
2. **VR integration** — OpenXR, stereo rendering, controller input
3. **Wire up `GpuPhysicsPipeline`** in the Bevy app

---

## Architecture

### Stick = mocap body

The stick is NOT a dynamic body with a joint. It's a **mocap body** —
kinematic, driven directly by the VR controller pose. The GPU FK shader
already handles mocap bodies (`body_mocap_id != 0xFFFFFFFF`): it reads
the pose from `mocap_pos`/`mocap_quat` buffers and skips joint FK.

This is the simplest approach:
- No PD spring-damper complexity
- No joint constraint needed
- Stick has effectively infinite mass (pushes puck, not pushed back)
- Controller → stick is 1:1 (zero lag, no floatiness)
- `state_bufs.upload_mocap()` already exists in the GPU pipeline

The puck and goal are free bodies with SDF geoms. Contacts between
stick (mocap) and puck (free) transfer momentum via the constraint solver.

### Per-frame data flow

```
Controller → CPU:  stick pose (position + orientation)
CPU → GPU:         mocap_pos, mocap_quat (once per frame, ~32 bytes)

┌─── Substep loop (all on GPU, single command buffer) ──────────┐
│  FK (mocap bodies get controller pose, free bodies get qpos)  │
│  CRBA → Velocity FK → RNE → Smooth                           │
│  Collision (AABB + SDF narrowphase)                           │
│  Constraint solve (Newton: stick contacts push puck)          │
│  Integration (puck/goal positions update)                     │
└───────────────────────────────────────────────────────────────┘

GPU → CPU:  qpos, qvel (once per frame, ~few KB)
CPU:        FK for Bevy rendering (body/geom poses)
Bevy:       stereo render to Quest 3 via OpenXR
```

### Scene setup

| Body | Type | Joint | Driven by |
|------|------|-------|-----------|
| Puck | Dynamic | Free | Physics (gravity + contacts) |
| Stick | Mocap | — | VR controller pose |
| Goal | Dynamic | Free | Physics (gravity + contacts) |
| Ground | Static (geom on world body) | — | — |

### Contact pairs

| Pair | Type | Notes |
|------|------|-------|
| Stick ↔ Puck | SDF-SDF | Blade hits puck — momentum transfer |
| Puck ↔ Ground | SDF-plane | Puck slides on ice |
| Puck ↔ Goal | SDF-SDF | Puck enters goal |
| Stick ↔ Ground | SDF-plane | Blade scrapes ice |
| Goal ↔ Ground | SDF-plane | Goal sits on ice |

### Model parameters

- **Timestep:** 0.002s (500 Hz)
- **Substeps per frame:** 4 (at 90 Hz → 360 physics steps/sec)
- **Solver:** Newton (GPU, 1–3 iterations)
- **SDF grid:** 1.0mm collision, 0.3mm visual
- **Friction:** 0.05 sliding (ice-like)
- **Scale:** millimeters (physics) → meters (VR). Factor 1000:1 (tabletop hockey).

---

## Implementation plan

### Step 1: Mocap hockey model

Replace the current hinge-based stick with a mocap body setup:
- Stick body: `body_mocapid = Some(0)` (mocap index 0)
- No joint on the stick body (it's kinematic, not dynamic)
- Puck and goal remain as free-joint bodies
- All dynamic joints are Free → GPU-compatible

This model works with `GpuPhysicsPipeline::new()` (validation passes).

### Step 2: GPU stepping in Bevy

Add `upload_mocap()` to `GpuPhysicsPipeline::step()` (currently only
uploads qpos/qvel — mocap poses must also be uploaded each frame so the
FK shader can read the controller-driven stick pose).

```rust
#[derive(Resource)]
struct GpuPhysics(sim_gpu::GpuPhysicsPipeline);

fn step_physics_gpu(
    model: Res<PhysicsModel>,
    mut data: ResMut<PhysicsData>,
    gpu: Res<GpuPhysics>,
) {
    gpu.0.step(&model.0, &mut data.0, 4);         // 4 substeps, single submit
    data.0.forward_pos_vel(&model.0, true);         // CPU FK for Bevy rendering
}
```

### Step 3: VR input (Quest 3 controllers)

Add `bevy_mod_openxr` (behind `vr` feature flag):

```toml
[features]
vr = ["dep:bevy_mod_openxr", "dep:bevy_mod_xr"]
```

Controller → mocap mapping system:
```rust
fn controller_to_mocap(
    controllers: Query<&Transform, With<RightController>>,
    mut data: ResMut<PhysicsData>,
) {
    if let Ok(transform) = controllers.single() {
        // Bevy transform → physics mocap pose (scale + axis conversion)
        data.0.mocap_pos[0] = bevy_to_physics_pos(transform.translation);
        data.0.mocap_quat[0] = bevy_to_physics_quat(transform.rotation);
    }
}
```

The GPU pipeline uploads mocap poses automatically via
`state_bufs.upload_mocap()` inside `step()`. Currently `step()` only
uploads qpos/qvel — mocap upload needs to be added.

### Step 4: Haptics (optional)

OpenXR haptic pulse on blade↔puck contact:
- Read back `ncon` from GPU (already part of readback)
- If contacts involve stick geom: trigger 20–50ms vibration
- Amplitude proportional to puck velocity change

---

## Platform

- **Development:** macOS (Metal backend)
- **Target:** Windows + NVIDIA 5070 Ti (Vulkan) + Quest 3 (Link/Air Link)
- **Fallback:** CPU physics always available if GPU init fails
- **VR runtime:** Quest 3 via Oculus Link → SteamVR → OpenXR

## Pass criteria

- Stable 90 Hz with 4 GPU substeps (no dropped frames)
- Puck slides on ice, enters goal on hit
- Stick tracks controller with zero visible lag
- All physics values finite after 60 seconds of play
- GPU physics step < 2ms per frame (leaves 9ms for rendering)

## Failure modes

- **GPU init fails:** CPU fallback (slower but works)
- **VR frame drops:** Profile each GPU stage. Single-submit eliminates
  per-substep CPU↔GPU round-trips.
- **Stick clips through puck:** SDF grid too coarse or timestep too large.
  Increase grid resolution or decrease timestep.
- **Puck jitters on ice:** Newton solver needs more iterations or
  solref/solimp need tuning.
