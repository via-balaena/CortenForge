# SDF Physics 10b — GPU Hockey → VR Hockey

## Vision

A hockey game powered by a near-complete GPU physics pipeline, ultimately
playable in VR. The physics engine runs broadphase through integration on
GPU compute shaders, with data staying on-device between stages. CPU
readback happens only for rendering. VR is layered on top only after the
GPU pipeline is rock-solid.

## Phases

### Phase 1: GPU physics pipeline

Build a complete GPU physics pipeline for this scene. Data stays on GPU
from broadphase through integration. CPU readback only for rendering.

| Stage | GPU suitability | Current status | Shader |
|---|---|---|---|
| **1a. SDF-SDF collision** | Excellent | **Done** | `trace_surface.wgsl` |
| **1b. SDF-plane collision** | Excellent | Stubbed | New: `sdf_plane.wgsl` |
| **1c. Integration** | Excellent | CPU only | New: `integrate.wgsl` |
| **1d. Broadphase** | Excellent | CPU only | New: `broadphase.wgsl` |
| **1e. Jacobi solver** | Good | CPU PGS only | New: `jacobi_solve.wgsl` |

**Not on GPU (keep on CPU):**
- GJK/EPA narrowphase — too branchy, poor GPU utilization
- Joint projection — complex, low parallelism
- Forward kinematics — tree-structured, sequential

**Architecture principle:** The GPU pipeline is a sequence of compute
dispatches connected by GPU buffers. No CPU readback between stages.
The full step is: broadphase → narrowphase → solver → integration →
(optional) readback for rendering.

```
 ┌──────────────────── GPU ─────────────────────┐
 │                                               │
 │  Broadphase    Narrowphase    Solver   Integ  │
 │  (spatial      (SDF-SDF,     (Jacobi  (pos/  │
 │   hashing)      SDF-plane)    + mass   vel   │
 │                                split)  update)│
 │     ↓              ↓            ↓        ↓    │
 │  [pair_buf]   [contact_buf]  [force]  [qpos]  │
 │                                       [qvel]  │
 └───────────────────────────────────┬───────────┘
                                     │ readback
                                     ↓
                              Bevy rendering
```

**Deliverables for each stage:**

**1a. SDF-SDF collision (done)**
- `trace_surface.wgsl`: one thread per grid cell, atomic contact append
- Pre-allocated 32,768-contact buffer
- Tested: sphere-sphere + concave socket-pin

**1b. SDF-plane collision**
- New shader: `sdf_plane.wgsl`
- Per-cell: test distance to plane → reconstruct surface → contact test
- Simpler than SDF-SDF: plane normal is constant, no second grid lookup
- Estimated: ~150 lines WGSL

**1c. Integration**
- New shader: `integrate.wgsl`
- Per-body: `qvel += dt * qacc`, `qpos += dt * qvel` (semi-implicit Euler)
- Embarrassingly parallel — one thread per body
- Gravity applied here
- Estimated: ~100 lines WGSL

**1d. Broadphase (spatial hashing)**
- New shader: `broadphase.wgsl`
- Three-pass counting sort: count via atomics → prefix sum → scatter
- Input: AABB per geom (from qpos + geom extents)
- Output: candidate collision pairs (geom_i, geom_j)
- Reference: GPU Gems 3 Ch. 32 (26x speedup demonstrated)
- Estimated: ~300 lines WGSL

**1e. Jacobi solver with mass splitting**
- New shader: `jacobi_solve.wgsl`
- Replace CPU PGS with GPU-parallel Jacobi
- Mass splitting (Tonge 2012): split body mass across contacts so each
  contact is solved independently. Eliminates Jacobi jitter.
- Multiple iterations per step (4–8 typical)
- Input: contact buffer + body state
- Output: constraint forces → applied to qvel
- Reference: ComFree-Sim (2026) for linear scaling alternative
- Estimated: ~400 lines WGSL
- **This is the hardest stage.** PGS is sequential; Jacobi trades
  convergence for parallelism. Mass splitting recovers convergence.

### Phase 2: Flat-screen validation

The hockey example with spacebar input, full GPU pipeline. Validate:
- GPU contacts match CPU within f32 tolerance
- Solver produces stable stacking (goal on ground) and sliding (puck)
- Performance: measure full-step GPU wall-clock vs CPU
- 1 kHz stress test for VR readiness (0.001s timestep, stable)

### Phase 3: VR

Add `bevy_mod_openxr` (0.5.0, Bevy 0.18). Stick becomes Free joint
tracked by VR controller via PD spring-damper. GPU pipeline handles all
physics at 90 Hz × 2–4 substeps = 180–360 physics steps/second.

### Phase 4: Advanced GPU (future)

- Multigrid solver (MGPBD) for faster convergence
- Dirty-cell optimization (skip unchanged SDF regions)
- Double-buffered async readback (pipeline step N+1 while reading N)
- Flex constraints on GPU

## GPU architecture details

### Buffer lifecycle

```
Model creation (once):
  - Upload SDF grids to GPU storage buffers (per-shape, immutable)
  - Allocate contact buffer (32,768 × 32 bytes, reused each step)
  - Allocate pair buffer (broadphase output)
  - Allocate body state buffer (qpos, qvel, qacc — per body)

Per physics step:
  - Write: body poses/AABBs (from qpos, ~few KB)
  - Dispatch: broadphase → narrowphase → solver → integration
  - Read: qpos/qvel (for rendering, ~few KB)

Between substeps (2–4 per frame):
  - NO re-upload of SDF grids (static)
  - NO CPU readback (data stays on GPU)
  - Only final substep triggers readback for rendering
```

### Device strategy

`sim-gpu` owns its own `wgpu::Device`, separate from Bevy's render
device. This avoids contention between physics compute and rendering.
Both can submit work concurrently.

### Precision

- GPU shaders use f32 (WebGPU has no f64 support)
- CPU uses f64 for accumulation and solver
- Expected tolerance: contact positions within 1.5mm of CPU reference
- Solver convergence: Jacobi needs more iterations than PGS but each
  iteration is massively parallel

## Platform

- **Development:** macOS (Metal backend — GPU collision works on Metal)
- **Target:** Windows + NVIDIA 5070 Ti (Vulkan backend + VR)
- **CPU fallback:** Always available if GPU init fails
- **Feature flag:** `vr` for XR deps. Default: flat-screen + GPU.

## Scene setup

### Puck

- `Solid::cylinder(12.0, 2.0)` — 24mm diameter, 4mm thick
- PLA (1250 kg/m³), black, Free joint at `(0, 0, 2)`

### Stick

- Shaft: `Solid::pipe([(0,0,0), (0,-55,-15)], 2.5)` — ~15° angle
- Blade: `Solid::cuboid(15, 2, 2.5)` at `(13, -57, -15.5)` — L-shape
- `shaft.smooth_union(blade, 0.5)`, `with_joint_origin((0,0,0))`
- Flat-screen: Revolute at `(0, 59, 18)`, spacebar swing
- VR: Free joint, PD spring-damper

### Goal

- `cuboid(12,25,10) - cuboid(12,23,10).translate(4,0,-2)`
- Back wall + side walls + crossbar, no floor, open +X face
- Steel (7800 kg/m³), semi-transparent red, at `(-50, 0, 10)`

### Contact pairs

| Pair | Type | GPU shader |
|---|---|---|
| Blade ↔ puck | SDF-SDF | `trace_surface.wgsl` |
| Puck ↔ goal | SDF-SDF | `trace_surface.wgsl` |
| Puck ↔ ice | SDF-plane | `sdf_plane.wgsl` (Phase 1b) |
| Stick ↔ ice | SDF-plane | `sdf_plane.wgsl` (Phase 1b) |
| Goal ↔ ice | SDF-plane | `sdf_plane.wgsl` (Phase 1b) |

## Model parameters

- **Timestep:** 0.002s (500 Hz). Stress test: 0.001s (1 kHz).
- **Solver:** PGS on CPU (Phase 2), Jacobi on GPU (after Phase 1e).
- **SDF grid:** 1.0mm collision, 0.3mm visual
- **Friction:** 0.05 sliding on all geoms
- **GPU:** `sim_gpu::enable_gpu_collision(&mut model)`

## Pass criteria

### Phase 1: GPU pipeline

Each stage (1a–1e) must pass independently before integrating:

- **1a (done):** GPU SDF-SDF contacts match CPU ≥90% within 1.5mm
- **1b:** GPU SDF-plane contacts match CPU ≥90% within 1.5mm
- **1c:** GPU integration matches CPU qpos/qvel within 1e-4 (f32 limit)
- **1d:** GPU broadphase finds same collision pairs as CPU (no missed pairs)
- **1e:** GPU Jacobi solver produces stable simulation (no jitter, no
  explosion) for the hockey scene. Stacking (goal on ground) holds.
  Puck slides correctly after impact.

### Phase 2: Flat-screen validation

- Puck moves toward goal on spacebar (Δx < -10mm)
- All positions finite, puck grounded, contacts detected
- Full GPU step faster than CPU step (measured, printed)
- Stable at 1 kHz for 10+ seconds

### Phase 3: VR

- Stable 90 Hz with GPU physics (no dropped frames)
- Controller → stick tracking responsive (< 1 frame lag)
- Haptic feedback on blade↔puck contact
- Puck enters goal

## Failure modes

- **GPU init fails:** CPU fallback (always works, just slower)
- **GPU/CPU mismatch:** f32 precision → tolerance checks, not exact match
- **Jacobi jitter:** Mass splitting not applied → implement Tonge 2012
- **Broadphase missed pair:** Spatial hash cell too large → tune cell size
- **Solver divergence:** Too few iterations or too large timestep →
  increase iterations or decrease dt
- **Substep readback:** Data bouncing CPU↔GPU between substeps →
  ensure buffers stay on GPU, readback only after final substep
- **VR frame drops:** Physics > 11ms → profile each GPU stage individually

## VR details (Phase 3)

### Dependencies (behind `vr` feature flag)

```toml
[features]
vr = ["dep:bevy_mod_openxr", "dep:bevy_mod_xr", "dep:schminput"]
```

### Controller → stick mapping

PD spring-damper (Half-Life Alyx / Boneworks):
1. Kinematic anchor tracks controller 1:1
2. Dynamic stick (Free joint) connected via spring + damper
3. Velocity buffer (5 frames) for smooth impact transfer
4. Blade tip velocity: `v = v_linear + cross(ω, blade_pos - grip_pos)`

### Haptics

OpenXR `XrHapticVibration` via raw `OxrInstance`:
- 20–50ms pulse, amplitude ∝ impact speed
- Direct OpenXR API call (not wrapped by bevy_mod_openxr)

### Scale

Physics mm → VR meters. Factor 1000:1 (tabletop hockey).

## Future

- **Goal net:** dim=2 flex shell (cloth) on GPU
- **Curved blade:** `bend()` SDF operator
- **Multiplayer:** Two sticks, network physics sync
- **Multigrid solver:** MGPBD for faster Jacobi convergence
- **ComFree-Sim:** Analytical contact, linear GPU scaling
- **Async readback:** Double-buffer physics compute + rendering
