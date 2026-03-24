# GPU Physics Pipeline Spec

## 1. Problem

The current physics step runs entirely on CPU, with one exception:
`sim-gpu` accelerates SDF-SDF narrowphase collision via a WGSL compute
shader. But this creates a **half-GPU** pattern that is worse than
all-CPU for small scenes:

```
CPU: broadphase (find pairs)
  → CPU→GPU: upload poses (208 bytes per pair)
     → GPU: trace_surface.wgsl
        → GPU→CPU: readback contacts
           → CPU: constraint assembly, solver, integration
```

Each physics step incurs an upload + dispatch + readback cycle. At VR
rates (90 Hz × 2–4 substeps = 180–360 steps/sec), the transfer latency
dominates. The GPU sits idle during the CPU stages, and the CPU waits
for the GPU readback before it can continue.

**The fix is architectural:** move the entire physics step to GPU so data
stays on-device between stages. CPU readback happens only for rendering,
and only after the final substep — not between substeps, not between
stages.

## 2. Goal

A GPU physics pipeline where broadphase, narrowphase, constraint solver,
and integration run as sequential compute dispatches in a single command
buffer. Data flows through GPU storage buffers between stages. The CPU
submits one command buffer per physics step (or per batch of substeps)
and reads back body poses only for rendering.

```
Per frame (not per substep):
  CPU → GPU:  external forces, control inputs (~few KB)

Per substep (all on GPU, no readback):
  broadphase.wgsl  → pair_buffer
  narrowphase.wgsl → contact_buffer     (trace_surface + sdf_plane)
  jacobi_solve.wgsl → force_buffer
  integrate.wgsl   → qpos_buffer, qvel_buffer

After final substep:
  GPU → CPU:  qpos, qvel, geom_xpos (~few KB, for Bevy rendering)
```

## 3. Non-goals

- **Full MuJoCo conformance on GPU.** FK, RNE, CRBA, and tree-structured
  algorithms remain on CPU. These have parent→child dependencies that
  make them poor GPU targets. The GPU pipeline handles the "inner loop"
  stages that dominate wall-clock at high step rates.
- **Multi-environment batching.** MJX-style vmap over thousands of
  environments is a separate effort (requires SoA layout refactor).
  This spec targets single-environment interactive simulation.
- **GPU mesh collision / GJK-EPA.** These are branchy algorithms with
  poor GPU utilization. They remain CPU-only.
- **Replacing the CPU pipeline.** The GPU pipeline is an alternative code
  path. The CPU pipeline remains for correctness validation, platforms
  without GPU, and scenes where GPU dispatch overhead exceeds savings.

## 4. Current physics step (CPU)

The complete `Data::step()` pipeline (41 stages, see pipeline map below).
Stages marked **[GPU]** are candidates for the GPU pipeline:

```
step()
  ├─ FK (tree traversal)                          [CPU — sequential]
  ├─ Flex bending                                  [CPU]
  ├─ CRBA + LDL factorization                     [CPU — sequential]
  ├─ Transmissions (site, slidercrank, body)       [CPU]
  ├─ Collision                                     [GPU — broadphase + narrowphase]
  │   ├─ AABB computation                          [GPU — per-geom, parallel]
  │   ├─ Broadphase (sweep-and-prune)              [GPU — spatial hashing]
  │   ├─ Narrowphase dispatch                      [GPU — SDF-SDF, SDF-plane]
  │   └─ (GJK/EPA for convex pairs)               [CPU — branchy]
  ├─ cb_control fires                              [CPU — user callback]
  ├─ Actuation (ctrl → forces)                     [CPU — per-actuator]
  ├─ RNE bias forces                               [CPU — sequential tree]
  ├─ Passive forces                                [CPU — per-joint]
  ├─ Constraint assembly                           [GPU — per-contact, parallel]
  ├─ Unconstrained acceleration (M⁻¹·f)           [CPU — LDL solve]
  ├─ Constraint solver                             [GPU — Jacobi, parallel]
  │   ├─ (Newton — sequential)                     [CPU fallback]
  │   ├─ (CG — sequential)                         [CPU fallback]
  │   └─ PGS → replaced by Jacobi                 [GPU]
  ├─ Map efc_force → qfrc_constraint              [GPU — per-DOF reduction]
  ├─ Integration (qvel += dt·qacc, qpos += dt·qvel) [GPU — per-body, parallel]
  ├─ Sleep update                                  [CPU]
  └─ Warmstart save                                [CPU — copy]
```

## 5. GPU pipeline stages

### 5.1 What moves to GPU

| Stage | Shader | Input buffers | Output buffers | Parallelism |
|---|---|---|---|---|
| AABB computation | `aabb.wgsl` | qpos, geom_body, geom_size | geom_aabb | Per-geom |
| Broadphase | `broadphase.wgsl` | geom_aabb | pair_buffer, pair_count | 3-pass spatial hash |
| SDF-SDF narrowphase | `trace_surface.wgsl` (exists) | SDF grids, poses | contact_buffer | Per-cell |
| SDF-plane narrowphase | `sdf_plane.wgsl` (new) | SDF grids, poses, plane | contact_buffer | Per-cell |
| Constraint assembly | `assemble.wgsl` | contact_buffer, body state | efc_J, efc_b | Per-contact |
| Jacobi solver | `jacobi_solve.wgsl` | efc_J, efc_b, M_diag | efc_force, qacc | Per-constraint × iterations |
| Force mapping | `map_forces.wgsl` | efc_force, efc_J | qfrc_constraint | Per-DOF reduction |
| Integration | `integrate.wgsl` | qacc, qvel, qpos, dt | qvel, qpos (updated) | Per-body |

### 5.2 What stays on CPU

| Stage | Reason |
|---|---|
| FK (forward kinematics) | Tree traversal, parent→child dependency |
| CRBA + LDL | Tree-structured, sparse factorization |
| RNE (bias forces) | Forward-backward tree traversal |
| GJK/EPA narrowphase | Extremely branchy, poor GPU utilization |
| Transmissions | Per-actuator, few actuators, complex logic |
| Sleep / island discovery | Union-find, sequential |
| cb_control callback | User code, runs on CPU |
| Sensors | Diverse types, callback-heavy |

### 5.3 The split-step architecture

The GPU pipeline doesn't replace the full step. It replaces the **inner
loop** — the stages that run many times per frame and dominate wall-clock
at high step rates. The CPU handles setup (FK, CRBA, RNE) and teardown
(sleep, sensors) once per frame.

```
Per frame:
  CPU:  FK, CRBA, RNE, actuation, passive forces  (once)
  CPU → GPU:  body state, external forces          (once)

  GPU:  for substep in 0..N:                       (N = 2–4 for VR)
          broadphase → narrowphase → solver → integration

  GPU → CPU:  final qpos, qvel                     (once)
  CPU:  sleep update, sensors, warmstart save       (once)
```

This means FK/CRBA/RNE run at frame rate (90 Hz), not substep rate
(180–360 Hz). The GPU inner loop runs at substep rate with no CPU
round-trips between substeps.

**Trade-off:** Body poses used for broadphase/narrowphase during GPU
substeps are computed from the GPU-integrated qpos, not from full FK.
For free-floating bodies (hockey puck, stick), qpos directly encodes
world position — no FK needed. For articulated bodies with joints, the
GPU would need a simplified FK or accept slightly stale poses within a
frame. For the hockey scene (all Free joints + one Revolute), this is
a non-issue.

## 6. Buffer layout

### 6.1 Static buffers (uploaded once at model creation)

| Buffer | Size | Contents |
|---|---|---|
| `sdf_grid[shape_id]` | W×H×D × 4 bytes each | SDF distance values (f32) |
| `sdf_meta[shape_id]` | 32 bytes each | Width, height, depth, cell_size, origin |
| `geom_type` | ngeom × 4 bytes | GeomType enum (SDF, Plane, etc.) |
| `geom_body` | ngeom × 4 bytes | Body index per geom |
| `geom_size` | ngeom × 12 bytes | Geom dimensions (for AABB) |
| `geom_shape_id` | ngeom × 4 bytes | SDF grid index (for SDF geoms) |
| `body_mass` | nbody × 4 bytes | Inverse mass per body (f32) |
| `body_inertia` | nbody × 12 bytes | Diagonal inverse inertia (f32) |
| `friction` | ngeom × 12 bytes | Per-geom friction [slide, torsion, roll] |
| `solver_params` | 64 bytes | dt, gravity, solver iterations, tolerances |

### 6.2 Dynamic buffers (written per frame or per substep)

| Buffer | Size | Written by | Read by |
|---|---|---|---|
| `qpos` | nq × 4 bytes | CPU (initial), `integrate.wgsl` | `aabb.wgsl`, `integrate.wgsl` |
| `qvel` | nv × 4 bytes | CPU (initial), `integrate.wgsl` | `jacobi_solve.wgsl`, `integrate.wgsl` |
| `qacc` | nv × 4 bytes | `jacobi_solve.wgsl` | `integrate.wgsl` |
| `geom_aabb` | ngeom × 24 bytes | `aabb.wgsl` | `broadphase.wgsl` |
| `pair_buffer` | max_pairs × 8 bytes | `broadphase.wgsl` | narrowphase dispatch |
| `pair_count` | 4 bytes | `broadphase.wgsl` | narrowphase dispatch |
| `contact_buffer` | max_contacts × 32 bytes | narrowphase shaders | `assemble.wgsl` |
| `contact_count` | 4 bytes (atomic) | narrowphase shaders | `assemble.wgsl` |
| `efc_J` | max_constraints × nv × 4 bytes | `assemble.wgsl` | `jacobi_solve.wgsl` |
| `efc_b` | max_constraints × 4 bytes | `assemble.wgsl` | `jacobi_solve.wgsl` |
| `efc_force` | max_constraints × 4 bytes | `jacobi_solve.wgsl` | `map_forces.wgsl` |

### 6.3 Pre-allocation sizes

| Buffer | Max size | Rationale |
|---|---|---|
| `max_contacts` | 32,768 | Existing sim-gpu constant. Sufficient for ~100 SDF pairs. |
| `max_pairs` | 4,096 | ngeom² upper bound, but spatial hashing prunes aggressively |
| `max_constraints` | 65,536 | 2× max_contacts (friction rows double contact count) |

All buffers are pre-allocated at model creation. No dynamic growth on GPU.

## 7. Shader specifications

### 7.1 `trace_surface.wgsl` — SDF-SDF narrowphase (EXISTS)

**Status:** Done, tested, 267 lines.
**Workgroup:** 8×8×4 = 256 threads.
**Algorithm:** One thread per grid cell. Surface filter → gradient →
reconstruct → transform → cross-SDF query → contact emit via atomic.
**No changes needed** for the pipeline — just consumed differently
(output buffer feeds into `assemble.wgsl` instead of CPU readback).

### 7.2 `sdf_plane.wgsl` — SDF-plane narrowphase (NEW)

**Estimated:** ~150 lines WGSL.
**Workgroup:** 8×8×4 = 256 threads.
**Algorithm:** One thread per grid cell:
1. Read SDF value at cell
2. One-sided surface filter (skip far interior)
3. Compute gradient → surface point reconstruction
4. Transform surface point to world space
5. Compute signed distance to plane: `d = dot(point, plane_normal) - plane_offset`
6. If `d < margin`: emit contact
   - Position: the world-space surface point
   - Normal: plane normal (constant, no gradient needed)
   - Depth: `max(-d, 0)`

**Simpler than SDF-SDF:** No second grid lookup, no destination gradient.
Plane normal is a uniform, not computed per-thread.

### 7.3 `aabb.wgsl` — Bounding box computation (NEW)

**Estimated:** ~80 lines WGSL.
**Workgroup:** 64 threads (one per geom).
**Algorithm:** Per-geom:
1. Read body position and orientation from `qpos`
2. Read geom local AABB (half-extents + offset)
3. Rotate half-extents by body orientation
4. Compute world AABB: `center ± rotated_half_extents + margin`
5. Write to `geom_aabb[geom_id]`

### 7.4 `broadphase.wgsl` — Spatial hash broadphase (NEW)

**Estimated:** ~300 lines WGSL (3 passes).
**Reference:** GPU Gems 3, Chapter 32.

**Pass 1 — Count (per-geom):**
- Compute which hash cells the geom's AABB overlaps
- Atomically increment `cell_count[hash(cell)]`

**Pass 2 — Prefix sum:**
- Parallel prefix sum over `cell_count` to get `cell_offset`
- Standard Blelloch scan, workgroup size 256

**Pass 3 — Scatter + test (per-geom):**
- Scatter geom IDs into sorted `cell_geom_list` at `cell_offset`
- For each cell: test all pairs of geoms in the cell
- AABB overlap test → emit pair to `pair_buffer` via atomic

**Hash cell size:** `max(geom_AABB_extent) × 2` — tunable.

### 7.5 `assemble.wgsl` — Constraint assembly (NEW)

**Estimated:** ~250 lines WGSL.
**Workgroup:** 256 threads (one per contact).
**Algorithm:** Per-contact:
1. Read contact (position, normal, depth, geom pair)
2. Look up bodies for both geoms
3. Compute Jacobian row: `J[dof] = contact_normal · body_velocity_jacobian`
   - For free bodies: J is sparse (6 entries per body)
4. Compute RHS: `b = -depth / dt` (Baumgarte stabilization)
5. Compute diagonal: `d = J · M⁻¹ · Jᵀ + regularization`
6. Write `efc_J[row]`, `efc_b[row]`, `efc_diag[row]`

**Simplification for free-body scenes:** When all bodies are Free joints,
the Jacobian is block-diagonal (6×6 per body). No tree structure needed.
This covers the hockey scene entirely.

### 7.6 `jacobi_solve.wgsl` — Parallel Jacobi solver (NEW)

**Estimated:** ~400 lines WGSL.
**Workgroup:** 256 threads (one per constraint row).
**Algorithm:** Iterative Jacobi with mass splitting (Tonge 2012):

**Setup (once):**
- For each body, count how many contacts it participates in: `n_contacts[body]`
- Split mass: `m_split[body] = m[body] / n_contacts[body]`
- Recompute diagonal: `d_split = J · M_split⁻¹ · Jᵀ + regularization`

**Per iteration (4–8 iterations):**
1. Per-constraint: compute residual `r = b - d·f - Σ(off_diagonal · f_other)`
2. Update: `f_new = f + r / d_split`
3. Project: clamp `f_new` to friction cone (normal ≥ 0, tangent ≤ μ·normal)
4. Barrier sync between iterations

**Convergence:** Mass splitting ensures convergence even with Jacobi's
inherent instability. Tonge 2012 proves that the split preserves the
LCP complementarity structure.

**Alternative (future):** ComFree-Sim (2026) analytical contact resolution
for linear scaling. JGS2 (2025) for near-quadratic convergence.

### 7.7 `map_forces.wgsl` — Force mapping (NEW)

**Estimated:** ~60 lines WGSL.
**Workgroup:** 64 threads (one per DOF).
**Algorithm:** Per-DOF reduction:
`qfrc_constraint[dof] = Σ_row efc_force[row] × efc_J[row][dof]`

Sparse: most J entries are zero. For free bodies, at most 2 bodies
contribute per contact (6 DOFs each).

### 7.8 `integrate.wgsl` — Semi-implicit Euler (NEW)

**Estimated:** ~100 lines WGSL.
**Workgroup:** 64 threads (one per body).
**Algorithm:** Per-body:
1. `qacc = (qfrc_smooth + qfrc_constraint) / mass` (or use pre-computed qacc)
2. `qvel += dt × qacc`
3. For Free joints:
   - `qpos[0..3] += dt × qvel[0..3]` (translation)
   - `quat = quat × exp(0.5 × dt × ω)` (quaternion integration)
   - Normalize quaternion
4. For Hinge joints:
   - `qpos += dt × qvel` (scalar angle)

**Gravity:** Applied as `qacc[2] -= 9810.0` (mm/s², Z-up) before velocity update.

## 8. Command buffer structure

One command buffer per frame, encoding all substeps:

```rust
let mut encoder = device.create_command_encoder();

// Upload: external forces, control inputs (once per frame)
encoder.copy_buffer_to_buffer(&cpu_forces, &gpu_forces);

for _substep in 0..num_substeps {
    // Stage 1: AABB
    let mut pass = encoder.begin_compute_pass();
    pass.set_pipeline(&aabb_pipeline);
    pass.dispatch_workgroups(ceil(ngeom, 64), 1, 1);

    // Stage 2: Broadphase (3 sub-passes)
    // ... count, prefix_sum, scatter_and_test

    // Stage 3: Narrowphase (per SDF-SDF pair + per SDF-plane pair)
    // ... trace_surface dispatches + sdf_plane dispatches

    // Stage 4: Constraint assembly
    pass.set_pipeline(&assemble_pipeline);
    pass.dispatch_workgroups(ceil(max_contacts, 256), 1, 1);

    // Stage 5: Jacobi solver (multiple iterations)
    for _iter in 0..solver_iterations {
        pass.set_pipeline(&jacobi_pipeline);
        pass.dispatch_workgroups(ceil(max_constraints, 256), 1, 1);
    }

    // Stage 6: Force mapping
    pass.set_pipeline(&map_forces_pipeline);
    pass.dispatch_workgroups(ceil(nv, 64), 1, 1);

    // Stage 7: Integration
    pass.set_pipeline(&integrate_pipeline);
    pass.dispatch_workgroups(ceil(nbody, 64), 1, 1);

    // Reset contact_count atomic for next substep
    // (no CPU readback between substeps)
}

// Readback: final qpos for rendering (once per frame)
encoder.copy_buffer_to_buffer(&gpu_qpos, &staging_qpos);
queue.submit([encoder.finish()]);

// Async readback
staging_qpos.slice(..).map_async(MapMode::Read, ...);
device.poll(Maintain::Wait);
```

**Key property:** Between substeps, NO data leaves the GPU. The
`contact_count` atomic is reset via a small compute dispatch or
`clear_buffer`, not by CPU write.

## 9. Integration with sim-core

### 9.1 New trait: `GpuPhysicsPipeline`

```rust
pub trait GpuPhysicsPipeline: Send + Sync {
    /// Run N substeps on GPU. Returns final body state.
    fn step_gpu(
        &self,
        model: &Model,
        qpos: &[f64],       // input: current positions (f64, converted to f32 on upload)
        qvel: &[f64],       // input: current velocities
        qfrc_applied: &[f64], // input: external forces
        num_substeps: u32,
        dt: f64,
    ) -> GpuStepResult;
}

pub struct GpuStepResult {
    pub qpos: Vec<f64>,     // output: final positions (f32 on GPU, widened to f64)
    pub qvel: Vec<f64>,     // output: final velocities
    pub ncon: usize,        // output: contact count (final substep)
}
```

### 9.2 Dispatch in `Data::step()`

```rust
pub fn step(&mut self, model: &Model) -> Result<(), StepError> {
    if let Some(gpu_pipeline) = &model.gpu_pipeline {
        // GPU path: FK + RNE on CPU, inner loop on GPU
        self.forward_position(model)?;    // FK, CRBA
        self.forward_velocity(model);     // velocity FK
        self.forward_actuation(model);    // ctrl → forces
        self.forward_rne(model);          // bias forces
        self.forward_passive(model);      // springs, dampers

        let result = gpu_pipeline.step_gpu(
            model,
            self.qpos.as_slice(),
            self.qvel.as_slice(),
            self.qfrc_smooth.as_slice(), // combined applied + actuator + passive - bias
            model.gpu_substeps,
            model.timestep,
        );

        self.qpos.copy_from_slice(&result.qpos);
        self.qvel.copy_from_slice(&result.qvel);
        self.ncon = result.ncon;
        self.time += model.timestep * model.gpu_substeps as f64;

        self.forward_position(model)?;    // re-run FK for rendering
        return Ok(());
    }

    // CPU fallback (existing pipeline, unchanged)
    self.forward(model)?;
    self.integrate(model)?;
    Ok(())
}
```

### 9.3 Relationship to existing `GpuSdfCollision` trait

The existing trait (`sim-gpu/collision.rs`) handles SDF-SDF and SDF-plane
collision as standalone operations with CPU readback. It remains for:
- CPU fallback when the full GPU pipeline isn't available
- Platforms that support GPU compute but not the full pipeline
- Validation (compare GPU pipeline contacts against standalone GPU contacts)

The new `GpuPhysicsPipeline` trait is a higher-level abstraction that
encompasses the entire inner loop. It subsumes the collision trait's
functionality within its narrowphase stage.

## 10. Existing code reuse

| Existing code | Reuse in pipeline |
|---|---|
| `trace_surface.wgsl` (267 lines) | Directly reused as narrowphase SDF-SDF |
| `GpuContext` (device, queue, adapter) | Shared — single GPU device for all stages |
| `GpuSdfGrid` (grid upload) | Directly reused — grids are static buffers |
| `GridMeta` (width, height, depth, cell_size, origin) | Reused as uniform |
| Contact buffer layout | Same struct, same max size (32,768) |
| `enable_gpu_collision()` | Extended to `enable_gpu_pipeline()` |

**What changes:** The dispatch layer in `collision.rs` that does per-step
CPU→GPU→CPU round-trips. This is replaced by the command buffer chain
in §8.

## 11. Cleanup of current half-GPU path

The current `GpuSdfCollision` trait and its dispatch in
`sdf_collide.rs:173–183` can be cleaned up in phases:

1. **Phase 1 (now):** Keep as-is. It works and is tested.
2. **Phase 2 (during pipeline build):** Add `GpuPhysicsPipeline` as a
   parallel code path. Both traits coexist on `Model`.
3. **Phase 3 (after pipeline validated):** Deprecate `GpuSdfCollision`
   dispatch in `sdf_collide.rs`. The standalone GPU collision functions
   move to a `gpu_standalone` module for testing/validation only.
4. **Phase 4 (cleanup):** Remove the per-step readback path entirely.
   `GpuSdfCollision` trait remains but is only used for unit tests and
   CPU fallback validation.

## 12. Validation strategy

Each shader is validated independently before integration:

| Shader | Validation method |
|---|---|
| `trace_surface.wgsl` | Existing tests (4 passing) |
| `sdf_plane.wgsl` | Compare GPU contacts vs CPU `sdf_plane_contact()` |
| `aabb.wgsl` | Compare GPU AABBs vs CPU `compute_aabb()` |
| `broadphase.wgsl` | Compare GPU pairs vs CPU broadphase (no missed pairs) |
| `assemble.wgsl` | Compare GPU J/b vs CPU `assemble_unified_constraints()` |
| `jacobi_solve.wgsl` | Stability test: hockey scene runs 10s without explosion |
| `integrate.wgsl` | Compare GPU qpos/qvel vs CPU integration (within f32 tol) |

**End-to-end validation:** Run hockey scene with GPU pipeline and CPU
pipeline in parallel. Compare body trajectories over 5 seconds. Max
divergence should be bounded (not accumulated — f32 vs f64 drift is
expected but should not grow without bound).

## 13. Performance targets

| Metric | Target | Measurement |
|---|---|---|
| GPU step latency (hockey scene) | < 1ms | `wgpu` timestamp queries |
| CPU→GPU transfer (per frame) | < 0.1ms | Staging buffer map time |
| GPU→CPU readback (per frame) | < 0.1ms | Staging buffer map time |
| Substeps without readback | 2–4 | Verified via buffer inspection |
| 90 Hz frame budget | < 11ms total | Frame timer |
| GPU utilization during physics | > 50% | Platform profiler |

**Baseline:** Current CPU step for hockey scene (~X ms, to be measured).
GPU pipeline should be faster for scenes with > 10 SDF geom pairs.
For the 3-body hockey scene, GPU may not be faster due to dispatch
overhead — the win comes with more complex scenes.

## 14. Implementation order

Build bottom-up, validating each shader before integrating:

1. **`sdf_plane.wgsl`** — Completes narrowphase. Smallest new shader,
   builds directly on existing `trace_surface.wgsl` patterns.

2. **`integrate.wgsl`** — Simplest new stage. Per-body, no dependencies
   between bodies. Validates the buffer layout and GPU↔CPU data flow.

3. **`aabb.wgsl`** — Per-geom, trivial. Needed by broadphase.

4. **`broadphase.wgsl`** — First multi-pass shader. Validates the
   spatial hashing pattern and atomic pair emission.

5. **`assemble.wgsl`** — Connects narrowphase contacts to solver.
   Validates Jacobian computation on GPU.

6. **`jacobi_solve.wgsl`** — The hardest stage. Mass splitting,
   iterative convergence, friction cone projection. Build last because
   it depends on all previous stages' output formats.

7. **Pipeline orchestration** — Chain all shaders in a single command
   buffer. Eliminate per-substep readback. Implement `GpuPhysicsPipeline`
   trait.

8. **Cleanup** — Deprecate half-GPU dispatch. Update `enable_gpu_collision`
   to `enable_gpu_pipeline`.

## 15. Risks

| Risk | Mitigation |
|---|---|
| Jacobi convergence worse than PGS | Mass splitting (Tonge 2012). If still insufficient, fall back to CPU PGS for that frame. |
| f32 precision causes solver divergence | Kahan summation in reduction shaders. Regularization in Jacobi diagonal. |
| Broadphase spatial hash misses pairs | Validate against CPU broadphase. Tune cell size conservatively. |
| Command buffer too large for single submit | Split into multiple submits with barriers (unlikely for small scenes). |
| GPU dispatch overhead > compute savings | Only enable GPU pipeline for scenes above a geom-count threshold. |
| wgpu limitations (no f64, 16KB shared memory) | Already addressed in existing sim-gpu design. f32 throughout. |
| Barrier sync between Jacobi iterations | Use `storageBarrier()` in WGSL. Verified pattern in existing compute shaders. |

## 16. Future extensions

- **ComFree-Sim solver:** Analytical complementarity-free contact.
  Linear scaling on GPU. Drop-in replacement for Jacobi in §7.6.
- **JGS2 subspace solver:** Near-quadratic Jacobi convergence via
  precomputed perturbation subspace. Replaces mass splitting.
- **Multigrid (MGPBD):** Hierarchical Jacobi smoothing for faster
  convergence on large scenes.
- **Async double-buffer:** Pipeline substep N+1 compute while reading
  substep N results. Hides readback latency completely.
- **Multi-environment batching:** `vmap`-style parallel simulation of
  thousands of environments for RL training.
