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

**The fix is architectural:** move the entire inner-loop physics step to
GPU so data stays on-device between stages. CPU readback happens only
for rendering, and only after the final substep — not between substeps,
not between stages.

## 2. Goal

A GPU physics pipeline where broadphase, narrowphase, constraint solver,
and integration run as sequential compute dispatches in a single command
buffer. Data flows through GPU storage buffers between stages. The CPU
submits one command buffer per physics step (or per batch of substeps)
and reads back body poses only for rendering.

```
Per frame (not per substep):
  CPU → GPU:  M, qfrc_smooth, external forces, control inputs

Per substep (all on GPU, no readback):
  broadphase.wgsl     → pair_buffer
  narrowphase.wgsl    → contact_buffer     (trace_surface + sdf_plane)
  assemble.wgsl       → efc_J, efc_D, efc_aref
  newton_solve.wgsl   → qacc (primal solver, MJX-style)
  integrate.wgsl      → qpos_buffer, qvel_buffer

After final substep:
  GPU → CPU:  qpos, qvel, ncon (~few KB, for Bevy rendering)
```

## 3. Reference architecture: MJX

MJX (MuJoCo's JAX backend) is the reference implementation for GPU
physics in the MuJoCo ecosystem. Key design decisions we adopt:

| Aspect | MJX | CortenForge GPU | Rationale |
|--------|-----|-----------------|-----------|
| Solver | Newton/CG in primal space | Newton (nv ≤ 60) | Proven, fast convergence (1–3 iterations) |
| Solver variable | qacc (acceleration) | qacc | Primal = no M⁻¹ needed on GPU |
| Friction model | Both pyramidal + elliptic | Pyramidal first | Simpler GPU path, no cone Hessian blocks |
| Constraint Jacobian | Dense (nefc × nv) | Dense (nefc × nv) | MJX stores dense regardless of M format |
| FK/CRBA/RNE | GPU (level-order tree scan) | CPU (split-step) | Single-env interactive, not batch RL |
| Mass matrix | GPU Cholesky | CPU LDL → upload M | Small nv, reuse existing CRBA |
| Batching | vmap over environments | Single environment | VR interactive, not RL training |

**Key MJX insight adopted:** The solver operates in acceleration space
(primal), not constraint-force space (dual). This avoids the need for
`M⁻¹` on GPU — only `M` is needed, which CPU already computes via CRBA.
Newton converges in 1–3 iterations vs 20–100+ for Jacobi.

**Key MJX decision NOT adopted:** MJX runs FK/CRBA/RNE on GPU via
level-order tree scans (`scan.body_tree`). We keep these on CPU because:
(a) the hockey scene has 4 bodies — tree scans provide zero speedup,
(b) our split-step architecture eliminates per-substep CPU round-trips,
(c) WGSL lacks JAX's `scan`/`vmap` primitives.

**Future:** GPU FK/CRBA/RNE via level-order scans is the path to
multi-environment batching and full GPU simulation (§17).

## 4. Non-goals

- **Full MuJoCo conformance on GPU.** FK, RNE, CRBA, and tree-structured
  algorithms remain on CPU for this spec. The GPU pipeline handles the
  "inner loop" stages that dominate wall-clock at high step rates.
- **Multi-environment batching.** MJX-style vmap over thousands of
  environments is a separate effort (requires SoA layout refactor +
  GPU FK/CRBA/RNE). This spec targets single-environment interactive.
- **GPU mesh collision / GJK-EPA.** These are branchy algorithms with
  poor GPU utilization. They remain CPU-only.
- **Replacing the CPU pipeline.** The GPU pipeline is an alternative code
  path. The CPU pipeline remains for correctness validation, platforms
  without GPU, and scenes where GPU dispatch overhead exceeds savings.
- **Elliptic friction cones on GPU.** The GPU solver uses pyramidal
  friction (per-row independent, no cone Hessian coupling). Elliptic
  cones remain CPU-only via the existing PGS/Newton solvers. See §7.6.

## 5. Current physics step (CPU)

The complete `Data::step()` pipeline. The actual call tree is
`step() → forward_core() → forward_pos_vel() + forward_acc()` then
`integrate()`. Every stage is listed; **[GPU]** marks candidates for
the GPU inner loop.

```
step()
  forward_pos_vel()
  │ ├─ Wake detection (user-force)                  [CPU — sleep bookkeeping]
  │ ├─ mj_fwd_position                              [CPU — FK tree traversal]
  │ │   ├─ Joint-to-body FK                          [CPU — sequential parent→child]
  │ │   ├─ Geometry pose computation                 [CPU — per-geom from body]
  │ │   └─ Subtree COM                               [CPU — tree reduction]
  │ ├─ mj_flex / mj_flex_edge                        [CPU — flex bending]
  │ ├─ mj_crba + LDL factorization                  [CPU — tree-structured sparse]
  │ ├─ Wake detection (qpos change)                  [CPU — sleep bookkeeping]
  │ ├─ Transmissions (site, slidercrank)             [CPU — per-actuator]
  │ ├─ Wake detection (tendon)                       [CPU — sleep bookkeeping]
  │ ├─ mj_collision                                  [GPU — broadphase + narrowphase]
  │ │   ├─ AABB computation                          [GPU — per-geom, parallel]
  │ │   ├─ Broadphase (sweep-and-prune)              [GPU — spatial hashing]
  │ │   ├─ SDF-SDF narrowphase                       [GPU — grid tracing]
  │ │   ├─ SDF-plane narrowphase                     [GPU — grid tracing]
  │ │   ├─ SDF-primitive narrowphase                 [CPU — analytical]
  │ │   └─ GJK/EPA convex narrowphase                [CPU — branchy]
  │ ├─ Wake detection (collision, equality)          [CPU — sleep bookkeeping]
  │ ├─ Transmission body dispatch                    [CPU — needs contacts]
  │ ├─ Sensors (position-dependent)                  [CPU — diverse types]
  │ ├─ Energy (potential)                             [CPU — diagnostic]
  │ ├─ mj_fwd_velocity                              [CPU — velocity FK]
  │ ├─ mj_actuator_length                            [CPU — per-actuator]
  │ ├─ Sensors (velocity-dependent)                  [CPU — diverse types]
  │ └─ Energy (kinetic)                               [CPU — diagnostic]
  │
  cb_control                                          [CPU — user callback]
  │
  forward_acc()
  │ ├─ mj_fwd_actuation                             [CPU — per-actuator]
  │ ├─ mj_rne (bias forces)                         [CPU — forward-backward tree]
  │ ├─ mj_fwd_passive                                [CPU — springs, dampers]
  │ ├─ mj_gravcomp_to_actuator                       [CPU — gravity compensation]
  │ ├─ mj_island (island discovery)                  [CPU — union-find]
  │ ├─ mj_fwd_constraint_islands                     [GPU — assembly + solver]
  │ │   ├─ compute_qacc_smooth (M⁻¹·f)              [CPU — LDL solve]
  │ │   ├─ assemble_unified_constraints              [GPU — per-contact, parallel]
  │ │   ├─ Warmstart                                  [CPU — cost comparison]
  │ │   ├─ Solver dispatch (Newton/CG/PGS)           [GPU — Newton on GPU]
  │ │   ├─ compute_qfrc_constraint (J^T·f)           [GPU — per-DOF reduction]
  │ │   └─ Extract friction/limit forces             [CPU — bookkeeping]
  │ ├─ mj_fwd_acceleration (M⁻¹·total_force)        [CPU — LDL solve]
  │ ├─ Sensors (acceleration-dependent)              [CPU — diverse types]
  │ └─ Forward/inverse comparison (FWDINV)           [CPU — diagnostic]
  │
  integrate()                                        [GPU — per-body, parallel]
  │ ├─ Activation update                              [CPU — per-actuator]
  │ ├─ Velocity: qvel += dt·qacc                     [GPU — per-DOF]
  │ ├─ Position: qpos integration (SO(3) manifold)   [GPU — per-body]
  │ └─ Quaternion normalization                       [GPU — per-body]
  │
  Sleep update                                        [CPU — island-aware]
  Warmstart save                                      [CPU — copy qacc]
```

## 6. GPU pipeline design

### 6.1 What moves to GPU

| Stage | Shader | Input buffers | Output buffers | Parallelism |
|---|---|---|---|---|
| AABB computation | `aabb.wgsl` | qpos, geom_body, geom_size | geom_aabb | Per-geom |
| Broadphase | `broadphase.wgsl` | geom_aabb | pair_buffer, pair_count | 3-pass spatial hash |
| SDF-SDF narrowphase | `trace_surface.wgsl` (exists) | SDF grids, poses | contact_buffer | Per-cell |
| SDF-plane narrowphase | `sdf_plane.wgsl` (new) | SDF grids, poses, plane | contact_buffer | Per-cell |
| Constraint assembly | `assemble.wgsl` | contacts, body state | efc_J, efc_D, efc_aref | Per-contact |
| Primal solver | `newton_solve.wgsl` | efc_J, efc_D, M, qacc_smooth | qacc, efc_force | Per-iteration workgroup |
| Force mapping | `map_forces.wgsl` | efc_force, efc_J | qfrc_constraint | Per-DOF reduction |
| Integration | `integrate.wgsl` | qacc, qvel, qpos, dt | qvel, qpos (updated) | Per-body |

### 6.2 What stays on CPU

| Stage | Reason |
|---|---|
| FK (forward kinematics) | Tree traversal, parent→child dependency |
| CRBA + LDL factorization | Tree-structured, sparse factorization |
| RNE (bias forces) | Forward-backward tree traversal |
| compute_qacc_smooth | Requires M⁻¹ via sparse LDL solve |
| GJK/EPA narrowphase | Extremely branchy, poor GPU utilization |
| Transmissions | Per-actuator, few actuators, complex logic |
| Flex bending | Small, model-dependent, not inner-loop |
| Island discovery | Union-find, sequential |
| cb_control callback | User code, runs on CPU |
| Sensors | Diverse types, callback-heavy |
| Energy computation | Diagnostic only |
| Sleep / warmstart | Bookkeeping |

### 6.3 The split-step architecture

The GPU pipeline replaces the **inner loop** — the stages that run
at substep rate and dominate wall-clock. The CPU handles setup
(FK, CRBA, RNE, actuation, passive forces) and teardown (sleep,
sensors) once per frame.

```
Per frame:
  CPU:  FK, CRBA, RNE, actuation, passive forces   (once, ~90 Hz)
  CPU:  compute_qacc_smooth = M⁻¹ · qfrc_smooth    (once)
  CPU → GPU:  qpos, qvel, M, qacc_smooth,           (once)
              qfrc_smooth, geom poses

  GPU:  for substep in 0..N:                        (N = 2–4, ~180–360 Hz)
          aabb → broadphase → narrowphase
          → assemble → newton_solve → map_forces
          → integrate

  GPU → CPU:  final qpos, qvel, ncon                (once)
  CPU:  re-run FK for rendering                      (once)
  CPU:  sleep update, sensors, warmstart save        (once)
```

**What the GPU solver receives from CPU each frame:**
- `M` (nv × nv, f32): Mass matrix from CRBA. Constant during substeps
  because qpos changes within a frame don't warrant re-running CRBA.
  For free bodies, M is block-diagonal (6×6 per body). For the hockey
  scene (nv=13), M is 676 bytes.
- `qacc_smooth` (nv, f32): Unconstrained acceleration = M⁻¹·qfrc_smooth.
  Precomputed on CPU via sparse LDL solve.
- `qfrc_smooth` (nv, f32): Smooth force = applied + actuator + passive − bias.

### 6.4 Articulated body handling

**Free joints** (puck, goal): qpos directly encodes world position +
quaternion. GPU integration updates qpos directly. AABB and narrowphase
use qpos-derived poses. No FK needed between substeps. **Fully correct.**

**Hinge joints** (stick): qpos is a scalar angle. The body's world-space
pose depends on `parent_pose × rotation(axis, angle)`, which is FK.
Within GPU substeps, the body pose becomes stale.

**Impact for hockey:** At 2 substeps per frame, the stick pose is stale
by 1 substep. At 500 Hz, that's 2ms. At max swing speed (~25 rad/s),
the hinge angle changes by ~0.05 rad (2.9°). The AABB margin absorbs
this for broadphase. Narrowphase contacts will have slightly wrong
positions — acceptable for interactive VR.

**Mitigation (Phase 2):** Add a lightweight `hinge_fk.wgsl` shader that
computes `body_pose = parent_pose × rotation(axis, angle)` for hinge
joints. One mat4 multiply per hinge body per substep. This makes
substep-accurate collision possible for single-DOF joints without
full FK.

**Limitation:** Multi-body articulated chains (robot arms, fingers)
require full recursive FK, which is not in scope for this spec. The
GPU path supports: Free joints (any number), single-DOF hinges
attached to world body. See §17 for future FK on GPU.

### 6.5 Mass matrix assumptions

The GPU solver uses M in two ways:
1. **Gauss cost**: `0.5·(qacc − qacc_smooth)^T·M·(qacc − qacc_smooth)`
2. **Newton Hessian**: `H = M + J^T·diag(D·active)·J`

For the hockey scene, M is 13×13 and fits entirely in workgroup shared
memory (676 bytes). The GPU receives the full dense M from CPU.

**Free bodies** have block-diagonal M: a 6×6 block per body containing
mass (3×3 diagonal) and rotational inertia (3×3, generally full for
bodies with products of inertia). The blocks are independent.

**Scaling note:** For nv > 60, M exceeds 16KB shared memory
(60×60×4 = 14.4KB). At that point, switch to CG solver which needs
only M·v products (streamable, no shared-memory H). See §17.

## 7. Buffer layout

### 7.1 Static buffers (uploaded once at model creation)

| Buffer | Size | Contents |
|---|---|---|
| `sdf_grid[shape_id]` | W×H×D × 4 bytes each | SDF distance values (f32) |
| `sdf_meta[shape_id]` | 32 bytes each | Width, height, depth, cell_size, origin |
| `geom_type` | ngeom × 4 bytes | GeomType enum (SDF, Plane, etc.) |
| `geom_body` | ngeom × 4 bytes | Body index per geom |
| `geom_size` | ngeom × 12 bytes | Geom dimensions (for AABB) |
| `geom_shape_id` | ngeom × 4 bytes | SDF grid index (for SDF geoms) |
| `friction` | ngeom × 12 bytes | Per-geom friction [slide, torsion, roll] |
| `solver_params` | 64 bytes | dt, gravity, solver iterations, tolerances |

### 7.2 Per-frame buffers (uploaded once per frame from CPU)

| Buffer | Size | Contents |
|---|---|---|
| `M` | nv × nv × 4 bytes | Mass matrix (dense, from CRBA) |
| `qacc_smooth` | nv × 4 bytes | Unconstrained acceleration (M⁻¹·qfrc_smooth) |
| `qfrc_smooth` | nv × 4 bytes | Smooth force (applied + actuator + passive − bias) |
| `qpos` | nq × 4 bytes | Initial positions (also updated by integrate.wgsl) |
| `qvel` | nv × 4 bytes | Initial velocities (also updated by integrate.wgsl) |
| `body_invmass` | nbody × 4 bytes | Inverse mass per body (f32) |
| `body_invinertia` | nbody × 36 bytes | 3×3 inverse inertia tensor per body (f32) |

**Note:** `body_invmass` and `body_invinertia` are the full inverse
inertia per body (not just diagonal), needed for free-body Jacobian
assembly. For bodies with products of inertia, the off-diagonal terms
of the 3×3 inverse inertia matter.

### 7.3 Dynamic buffers (written per substep on GPU)

| Buffer | Size | Written by | Read by |
|---|---|---|---|
| `qacc` | nv × 4 bytes | `newton_solve.wgsl` | `integrate.wgsl` |
| `geom_aabb` | ngeom × 24 bytes | `aabb.wgsl` | `broadphase.wgsl` |
| `pair_buffer` | max_pairs × 8 bytes | `broadphase.wgsl` | narrowphase dispatch |
| `pair_count` | 4 bytes | `broadphase.wgsl` | narrowphase dispatch |
| `contact_buffer` | max_contacts × 48 bytes | narrowphase shaders | `assemble.wgsl` |
| `contact_count` | 4 bytes (atomic) | narrowphase shaders | `assemble.wgsl` |
| `efc_J` | max_constraints × nv × 4 bytes | `assemble.wgsl` | `newton_solve.wgsl` |
| `efc_D` | max_constraints × 4 bytes | `assemble.wgsl` | `newton_solve.wgsl` |
| `efc_aref` | max_constraints × 4 bytes | `assemble.wgsl` | `newton_solve.wgsl` |
| `efc_type` | max_constraints × 4 bytes | `assemble.wgsl` | `newton_solve.wgsl` |
| `efc_force` | max_constraints × 4 bytes | `newton_solve.wgsl` | `map_forces.wgsl` |
| `qfrc_constraint` | nv × 4 bytes | `map_forces.wgsl` | `integrate.wgsl` |

**Contact buffer layout (48 bytes per contact):**
```
struct GpuContact {
    point:    vec3<f32>,   // World-space contact position
    depth:    f32,         // Penetration depth (≥ 0)
    normal:   vec3<f32>,   // World-space contact normal
    geom1:    u32,         // Geom index A
    friction: vec3<f32>,   // Mixed friction [slide, torsion, roll]
    geom2:    u32,         // Geom index B
}
```

### 7.4 Pre-allocation sizes

| Buffer | Max size | Rationale |
|---|---|---|
| `max_contacts` | 32,768 | Existing sim-gpu constant. ~100 SDF pairs × ~300 contacts. |
| `max_pairs` | 4,096 | 50 geoms → C(50,2) = 1225 worst case. 3× headroom. |
| `max_constraints` | 196,608 | 6 × max_contacts. Pyramidal friction: 2×(condim−1) rows per contact, condim=4 → 6 rows. |

**Derivation of max_constraints:** Each contact with condim=4 (normal +
3 friction axes) produces 6 pyramidal constraint rows: 2 opposing facets
per friction axis × 3 axes. For condim=3: 4 rows. For condim=1
(frictionless): 1 row. The 6× multiplier handles the worst case.

**Memory budget (hockey scene, nv=13):**
- `efc_J`: 196,608 × 13 × 4 = 10.2 MB
- `efc_D/aref/type/force`: 196,608 × 4 × 4 = 3.1 MB
- Total constraint buffers: ~13 MB (well within GPU limits)

**Note:** For the actual hockey scene (~50-100 contacts, ~300-600
constraint rows), only a tiny fraction of these buffers is used.
Pre-allocation avoids dynamic allocation on GPU.

## 8. Shader specifications

### 8.1 `trace_surface.wgsl` — SDF-SDF narrowphase (EXISTS)

**Status:** Done, tested, 267 lines.
**Workgroup:** 8×8×4 = 256 threads.
**Algorithm:** One thread per grid cell. Surface filter → gradient →
reconstruct → transform → cross-SDF query → contact emit via atomic.
**Change for pipeline:** Output feeds into `assemble.wgsl` instead of
CPU readback. Contact struct extended from 32 to 48 bytes to include
geom indices and friction (or these are looked up by assembly shader
from the pair buffer).

### 8.2 `sdf_plane.wgsl` — SDF-plane narrowphase (NEW)

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

### 8.3 `aabb.wgsl` — Bounding box computation (NEW)

**Estimated:** ~80 lines WGSL.
**Workgroup:** 64 threads (one per geom).
**Algorithm:** Per-geom:
1. Read body position and orientation from `qpos`
2. Read geom local AABB (half-extents + offset)
3. Rotate half-extents by body orientation
4. Compute world AABB: `center ± rotated_half_extents + margin`
5. Write to `geom_aabb[geom_id]`

**For hinge-attached geoms:** Uses the parent body pose from the
previous substep (stale by 1 substep). Phase 2 adds `hinge_fk.wgsl`
to update these poses per substep.

### 8.4 `broadphase.wgsl` — Spatial hash broadphase (NEW)

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

### 8.5 `assemble.wgsl` — Constraint assembly (NEW)

**Estimated:** ~350 lines WGSL.
**Workgroup:** 256 threads (one per contact).
**Reference:** MJX `constraint.py:make_constraint()`

**Algorithm:** Per-contact, emit pyramidal constraint rows:
1. Read contact (position, normal, depth, geom pair, friction)
2. Look up body indices for both geoms
3. Compute contact frame: normal + 2 tangent vectors (Gram-Schmidt)
4. **For each friction axis (2 axes for condim=3, 3 for condim=4):**
   Emit 2 pyramidal facet rows (positive and negative directions):
   ```
   J_facet = normal_J ± μ_i · tangent_i_J
   ```
   where `normal_J` and `tangent_i_J` are the contact-frame Jacobians
   projected into joint space.
5. Also emit 1 normal-only row (frictionless component)

**Jacobian computation for free bodies:**
```
For body A at contact point p:
  J_linear  = contact_dir              (3 entries at body_A dof_adr)
  J_angular = cross(p - body_A_pos, contact_dir)  (3 entries at body_A dof_adr + 3)
For body B: negate.
```
Each row has at most 12 non-zero entries (6 per body).

**Impedance and stabilization (matching CPU assembly):**
- `pos` = penetration depth
- `vel` = J · qvel (constraint-space velocity)
- `imp` = impedance from solimp (violation-dependent damping)
- `(k, b)` = stiffness/damping from solref
- `aref = -b · vel - k · imp · pos` (reference acceleration)
- `D = imp / (imp · diagApprox + (1 − imp))` (constraint stiffness)
- `R = 1/D` (regularization)

**diagApprox for free bodies:** `diagApprox = J · M⁻¹ · J^T` reduces to
`1/mass + angular_J^T · I⁻¹ · angular_J` per body. Uses the per-body
`body_invmass` and `body_invinertia` buffers — no full M⁻¹ needed.

### 8.6 `newton_solve.wgsl` — Primal Newton solver (NEW)

**Estimated:** ~500 lines WGSL.
**Reference:** MJX `solver.py:_newton()`, `solver.py:_cost()`

**Design:** Single-workgroup Newton solver operating in acceleration
space (primal formulation). For the hockey scene (nv=13), the entire
13×13 Hessian fits in workgroup shared memory (676 bytes).

**Formulation (MJX primal):**
```
cost(qacc) = 0.5·(qacc − qacc_smooth)^T · M · (qacc − qacc_smooth)
           + Σ_i  s_i(J_i · qacc − aref_i)
```
where `s_i()` is the pyramidal constraint penalty:
- Equality:   `s(x) = 0.5 · D · x²`
- Inequality: `s(x) = 0.5 · D · x²` if `x < 0`, else `0`

**Gradient:**
```
grad = M · (qacc − qacc_smooth) + J^T · efc_force
```
where `efc_force_i = D_i · max(0, -(J_i · qacc − aref_i))` for
inequality constraints (sign depends on type).

**Hessian (pyramidal — block-diagonal, no cone coupling):**
```
H = M + J^T · diag(D · active) · J
```
where `active_i = 1` if constraint i is in its quadratic zone, else 0.
H is nv × nv. For pyramidal friction, no coupling between rows — the
Hessian is a simple diagonal-weighted sum.

**Algorithm (per Newton iteration, 1–3 iterations):**

1. **Evaluate cost + classify constraints** (parallel, 256 threads)
   - Each thread processes a subset of constraint rows
   - Compute `Jaref_i = J_i · qacc − aref_i`
   - Classify: active if `Jaref_i < 0` (inequality) or always (equality)
   - Compute `efc_force_i = −D_i · Jaref_i · active_i`
   - Accumulate per-thread contributions to `grad` and `H` in registers

2. **Reduce into shared memory** (workgroup reduction)
   - Sum all threads' `H` contributions → shared `H[nv][nv]`
   - Sum all threads' `grad` contributions → shared `grad[nv]`
   - Add `M` to `H` (loaded from per-frame buffer)
   - Add `M · (qacc − qacc_smooth)` to `grad`

3. **Cholesky solve** (single thread, thread 0)
   - Factor `H = L · L^T` in shared memory (~nv³/6 flops = ~366 for nv=13)
   - Solve `L · L^T · Δqacc = −grad` via forward/backward substitution
   - Cost: ~nv² flops = ~169 for nv=13

4. **Line search** (parallel evaluation)
   - Test α = 1.0, 0.5, 0.25 (or bracket-narrowing Newton as in MJX)
   - Evaluate `cost(qacc + α · Δqacc)` for each α
   - Pick α with lowest cost

5. **Update** (broadcast)
   - `qacc += α · Δqacc`

**Workgroup layout:** 256 threads, one workgroup total. Thread 0 does
Cholesky; all threads cooperate on constraint evaluation and reduction.

**Shared memory budget (nv=13):**
```
H:           13 × 13 × 4 = 676 bytes
grad:        13 × 4       = 52 bytes
qacc:        13 × 4       = 52 bytes
qacc_smooth: 13 × 4       = 52 bytes
M:           13 × 13 × 4 = 676 bytes
scratch:     ~500 bytes
Total:       ~2 KB (well under 16 KB limit)
```

**Warmstart:** The solver initializes `qacc` from the previous substep's
result (already on GPU — no CPU round-trip). First substep per frame
uses `qacc_smooth` as cold start, or optionally the CPU warmstart.

**Convergence:** Newton converges quadratically. For the hockey scene,
1–2 iterations suffice (validated against CPU Newton which also does
1–3 iterations). Tolerance: `improvement < solver_tolerance` where
improvement is the relative cost decrease.

**Constraint force output:** After final iteration, `efc_force` is
written to the storage buffer for `map_forces.wgsl`.

### 8.7 `map_forces.wgsl` — Force mapping (NEW)

**Estimated:** ~60 lines WGSL.
**Workgroup:** 64 threads (one per DOF).
**Algorithm:** Per-DOF reduction:
```
qfrc_constraint[dof] = Σ_row  efc_force[row] × efc_J[row][dof]
```

This is `J^T · efc_force` — a matrix-vector transpose product.
Sparse: most J entries are zero. For free bodies, at most 2 bodies
contribute per contact (6 DOFs each).

### 8.8 `integrate.wgsl` — Semi-implicit Euler (NEW)

**Estimated:** ~120 lines WGSL.
**Workgroup:** 64 threads (one per body).
**Reference:** MJX `math.py:quat_integrate()`

**Algorithm:** Per-body:
1. Compute acceleration:
   - For Free joints: `qacc` already computed by solver (per-DOF)
   - For Hinge joints: `qacc[hinge_dof]` from solver
2. Update velocity: `qvel += dt × qacc`
3. Integrate position:
   - **Free joints (translation):** `qpos[0..3] += dt × qvel[0..3]`
   - **Free joints (rotation):** Exponential map on SO(3):
     ```
     axis = normalize(ω)
     angle = ‖ω‖ × dt
     dq = [cos(angle/2), sin(angle/2) × axis]
     quat_new = quat_old × dq
     normalize(quat_new)
     ```
   - **Hinge joints:** `qpos += dt × qvel` (scalar)
4. Apply gravity: Included in `qfrc_smooth` (uploaded from CPU), not
   applied separately in the shader.

**Matching CPU:** Uses the same quaternion integration as
`integrate/euler.rs:integrate_quaternion()` — exponential map with
quaternion right-multiplication and renormalization.

## 9. Command buffer structure

One command buffer per frame, encoding all substeps:

```rust
let mut encoder = device.create_command_encoder();

// Upload per-frame data: M, qacc_smooth, qfrc_smooth, qpos, qvel
encoder.copy_buffer_to_buffer(&staging_m, &gpu_m, ...);
encoder.copy_buffer_to_buffer(&staging_qacc_smooth, &gpu_qacc_smooth, ...);
encoder.copy_buffer_to_buffer(&staging_qfrc_smooth, &gpu_qfrc_smooth, ...);
encoder.copy_buffer_to_buffer(&staging_qpos, &gpu_qpos, ...);
encoder.copy_buffer_to_buffer(&staging_qvel, &gpu_qvel, ...);

for _substep in 0..num_substeps {
    // Reset atomic counters
    encoder.clear_buffer(&pair_count_buffer, 0, 4);
    encoder.clear_buffer(&contact_count_buffer, 0, 4);

    // Stage 1: AABB
    {
        let mut pass = encoder.begin_compute_pass(&default_desc());
        pass.set_pipeline(&aabb_pipeline);
        pass.dispatch_workgroups(ceil(ngeom, 64), 1, 1);
    }

    // Stage 2: Broadphase (3 sub-dispatches)
    {
        let mut pass = encoder.begin_compute_pass(&default_desc());
        // Pass 1: count
        pass.set_pipeline(&broadphase_count_pipeline);
        pass.dispatch_workgroups(ceil(ngeom, 64), 1, 1);
    }
    {
        let mut pass = encoder.begin_compute_pass(&default_desc());
        // Pass 2: prefix sum
        pass.set_pipeline(&broadphase_prefix_pipeline);
        pass.dispatch_workgroups(ceil(num_cells, 256), 1, 1);
    }
    {
        let mut pass = encoder.begin_compute_pass(&default_desc());
        // Pass 3: scatter + test
        pass.set_pipeline(&broadphase_scatter_pipeline);
        pass.dispatch_workgroups(ceil(ngeom, 64), 1, 1);
    }

    // Stage 3: Narrowphase
    // One dispatch per SDF-SDF pair (from pair_buffer)
    // One dispatch per SDF-plane pair
    {
        let mut pass = encoder.begin_compute_pass(&default_desc());
        pass.set_pipeline(&narrowphase_pipeline);
        // Dispatched per pair — indirect dispatch from pair_count
        pass.dispatch_workgroups_indirect(&indirect_buffer, 0);
    }

    // Stage 4: Constraint assembly
    {
        let mut pass = encoder.begin_compute_pass(&default_desc());
        pass.set_pipeline(&assemble_pipeline);
        // Dispatch enough for max_contacts (inactive threads early-exit)
        pass.dispatch_workgroups(ceil(max_contacts, 256), 1, 1);
    }

    // Stage 5: Newton solver (1–3 iterations, single workgroup)
    for _iter in 0..solver_iterations {
        let mut pass = encoder.begin_compute_pass(&default_desc());
        pass.set_pipeline(&newton_pipeline);
        pass.dispatch_workgroups(1, 1, 1);  // Single workgroup
    }

    // Stage 6: Force mapping
    {
        let mut pass = encoder.begin_compute_pass(&default_desc());
        pass.set_pipeline(&map_forces_pipeline);
        pass.dispatch_workgroups(ceil(nv, 64), 1, 1);
    }

    // Stage 7: Integration
    {
        let mut pass = encoder.begin_compute_pass(&default_desc());
        pass.set_pipeline(&integrate_pipeline);
        pass.dispatch_workgroups(ceil(nbody, 64), 1, 1);
    }
}

// Readback: final qpos, qvel, contact_count (once per frame)
encoder.copy_buffer_to_buffer(&gpu_qpos, &staging_qpos, ...);
encoder.copy_buffer_to_buffer(&gpu_qvel, &staging_qvel, ...);
encoder.copy_buffer_to_buffer(&contact_count_buffer, &staging_count, 0, 4);
queue.submit([encoder.finish()]);

// Async readback
staging_qpos.slice(..).map_async(MapMode::Read, ...);
device.poll(Maintain::Wait);
```

**Key property:** Between substeps, NO data leaves the GPU. Atomic
counters are reset via `clear_buffer`, not by CPU write. `qacc` from
the previous substep serves as warmstart for the next substep's
Newton solve.

**Each compute pass** implies a storage buffer barrier (wgpu guarantees
this between passes in the same command encoder). No explicit barriers
needed between stages.

## 10. Integration with sim-core

### 10.1 New trait: `GpuPhysicsPipeline`

```rust
pub trait GpuPhysicsPipeline: Send + Sync {
    /// Run N substeps on GPU. Returns final body state.
    fn step_gpu(
        &self,
        model: &Model,
        data: &Data,           // Full data — extracts qpos, qvel, M, qfrc_smooth
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

### 10.2 Dispatch in `Data::step()`

```rust
pub fn step(&mut self, model: &Model) -> Result<(), StepError> {
    if let Some(gpu_pipeline) = &model.gpu_pipeline {
        // CPU setup: FK, CRBA, RNE, actuation, passive forces
        self.forward_pos_vel(model, true);
        if let Some(ref cb) = model.cb_control {
            (cb.0)(model, self);
        }

        // CPU: acceleration-stage setup (actuation, RNE, passive, qacc_smooth)
        // These populate qfrc_smooth, qfrc_bias, qM, qacc_smooth.
        forward::actuation::mj_fwd_actuation(model, self);
        dynamics::rne::mj_rne(model, self);
        forward::passive::mj_fwd_passive(model, self);
        constraint::compute_qacc_smooth(model, self);

        // GPU inner loop
        let result = gpu_pipeline.step_gpu(
            model, self, model.gpu_substeps, model.timestep,
        );

        // Write back
        self.qpos.copy_from_slice(&result.qpos);
        self.qvel.copy_from_slice(&result.qvel);
        self.ncon = result.ncon;
        self.time += model.timestep * model.gpu_substeps as f64;

        // Re-run FK for Bevy rendering
        position::mj_fwd_position(model, self);

        // Sleep + warmstart
        if model.enableflags & ENABLE_SLEEP != 0 {
            island::mj_sleep(model, self);
        }
        self.qacc_warmstart.copy_from(&self.qacc);

        return Ok(());
    }

    // CPU fallback (existing pipeline, unchanged)
    // ...
}
```

### 10.3 Existing `integrate_without_velocity`

The codebase already has `Data::integrate_without_velocity()`
(`integrate/mod.rs:233`) behind the `gpu-internals` feature gate. This
performs position integration + quaternion normalization without
updating velocity (assuming velocity was updated on GPU). The GPU
pipeline's CPU-side post-readback can use this instead of reimplementing
position integration.

### 10.4 Relationship to existing `GpuSdfCollision` trait

The existing trait (`sim-gpu/collision.rs`) handles SDF-SDF and SDF-plane
collision as standalone operations with CPU readback. It remains for:
- CPU fallback when the full GPU pipeline isn't available
- Platforms that support GPU compute but not the full pipeline
- Validation (compare GPU pipeline contacts against standalone GPU contacts)

The new `GpuPhysicsPipeline` trait is a higher-level abstraction that
encompasses the entire inner loop. It subsumes the collision trait's
functionality within its narrowphase stage.

**Note:** The existing `GpuSdfCollision::sdf_plane_contacts()` method
is currently a stub returning `vec![]` (sim-gpu/collision.rs:548–557).
SDF-plane collision goes through `compute_shape_plane_contact()` on CPU
(sdf_collide.rs:141), not through the trait. The full GPU pipeline will
implement SDF-plane in `sdf_plane.wgsl` directly.

## 11. Existing code reuse

| Existing code | Reuse in pipeline |
|---|---|
| `trace_surface.wgsl` (267 lines) | Directly reused as narrowphase SDF-SDF |
| `GpuContext` (device, queue, adapter) | Shared — single GPU device for all stages |
| `GpuSdfGrid` (grid upload) | Directly reused — grids are static buffers |
| `GridMeta` (width, height, depth, cell_size, origin) | Reused as uniform |
| Contact buffer layout | Extended from 32 to 48 bytes (add geom IDs + friction) |
| `enable_gpu_collision()` | Extended to `enable_gpu_pipeline()` |
| `integrate_without_velocity()` | Used for CPU-side post-readback position integration |

**What changes:** The dispatch layer in `collision.rs` that does per-step
CPU→GPU→CPU round-trips. This is replaced by the command buffer chain
in §9.

## 12. Cleanup of current half-GPU path

The current `GpuSdfCollision` trait and its dispatch in
`sdf_collide.rs:176–227` (three-tier dispatch logic) can be cleaned up
in phases:

1. **Phase 1 (now):** Keep as-is. It works and is tested.
2. **Phase 2 (during pipeline build):** Add `GpuPhysicsPipeline` as a
   parallel code path. Both traits coexist on `Model`.
3. **Phase 3 (after pipeline validated):** Deprecate `GpuSdfCollision`
   dispatch in `sdf_collide.rs`. The standalone GPU collision functions
   move to a `gpu_standalone` module for testing/validation only.
4. **Phase 4 (cleanup):** Remove the per-step readback path entirely.
   `GpuSdfCollision` trait remains but is only used for unit tests and
   CPU fallback validation.

## 13. Validation strategy

Each shader is validated independently before integration:

| Shader | Validation method |
|---|---|
| `trace_surface.wgsl` | Existing tests (4 passing) |
| `sdf_plane.wgsl` | Compare GPU contacts vs CPU `compute_shape_plane_contact()` |
| `aabb.wgsl` | Compare GPU AABBs vs CPU `compute_aabb()` |
| `broadphase.wgsl` | Compare GPU pairs vs CPU broadphase (no missed pairs) |
| `assemble.wgsl` | Compare GPU J/D/aref vs CPU `assemble_unified_constraints()`. Note: GPU uses pyramidal, CPU may use elliptic — compare frictionless rows only, or set CPU to pyramidal mode for validation. |
| `newton_solve.wgsl` | Compare GPU qacc vs CPU Newton solver output (within f32 tolerance). Stability test: hockey scene runs 10s without explosion. |
| `map_forces.wgsl` | Compare GPU qfrc_constraint vs CPU `compute_qfrc_constraint_from_efc()` |
| `integrate.wgsl` | Compare GPU qpos/qvel vs CPU integration (within f32 tolerance) |

**End-to-end validation:** Run hockey scene with GPU pipeline and CPU
pipeline in parallel. Compare body trajectories over 5 seconds. Max
divergence should be bounded (not accumulated — f32 vs f64 drift is
expected but should not grow without bound). The pyramidal vs elliptic
friction difference will cause trajectory divergence — this is expected
and documented, not a bug.

## 14. Performance targets

| Metric | Target | Measurement |
|---|---|---|
| GPU step latency (hockey scene) | < 1ms per substep | `wgpu` timestamp queries |
| CPU→GPU transfer (per frame) | < 0.1ms | Staging buffer map time |
| GPU→CPU readback (per frame) | < 0.1ms | Staging buffer map time |
| Substeps without readback | 2–4 | Verified via buffer inspection |
| 90 Hz frame budget | < 11ms total | Frame timer |
| Newton iterations per substep | 1–3 | Solver convergence log |
| GPU utilization during physics | > 50% | Platform profiler |

**Baseline:** Current CPU step for hockey scene (~X ms, to be measured).
GPU pipeline should be faster for scenes with > 10 SDF geom pairs.
For the 3-body hockey scene, GPU may not be faster due to dispatch
overhead — the win comes with more complex scenes. The primary goal
for hockey is **eliminating per-substep CPU↔GPU latency**, not raw
compute throughput.

## 15. Implementation order

Build bottom-up, validating each shader before integrating:

1. **`sdf_plane.wgsl`** — Completes narrowphase. Smallest new shader,
   builds directly on existing `trace_surface.wgsl` patterns.

2. **`integrate.wgsl`** — Simplest new stage. Per-body, no dependencies
   between bodies. Validates the buffer layout and GPU↔CPU data flow.

3. **`aabb.wgsl`** — Per-geom, trivial. Needed by broadphase.

4. **`broadphase.wgsl`** — First multi-pass shader. Validates the
   spatial hashing pattern and atomic pair emission.

5. **`assemble.wgsl`** — Connects narrowphase contacts to solver.
   Validates pyramidal Jacobian computation on GPU. Depends on contact
   buffer format from steps 1–4.

6. **`newton_solve.wgsl`** — The core solver. Single-workgroup Newton
   in shared memory. Depends on assembly output format from step 5.
   Build last because it depends on all previous stages' output formats.

7. **`map_forces.wgsl`** — J^T · efc_force reduction. Simple, but
   depends on solver output format.

8. **Pipeline orchestration** — Chain all shaders in a single command
   buffer. Eliminate per-substep readback. Implement `GpuPhysicsPipeline`
   trait.

9. **Cleanup** — Deprecate half-GPU dispatch. Update `enable_gpu_collision`
   to `enable_gpu_pipeline`.

## 16. Risks

| Risk | Mitigation |
|---|---|
| Newton Cholesky fails on GPU (non-SPD H) | Regularize: H += ε·I. If still fails, fall back to CPU PGS for that frame. |
| f32 precision causes solver divergence | Kahan summation in Hessian accumulation. Regularization in H diagonal. MJX proves f32 Newton works. |
| Pyramidal friction differs from elliptic | Document expected behavioral difference. Validate frictionless components match exactly. |
| Broadphase spatial hash misses pairs | Validate against CPU broadphase. Tune cell size conservatively. |
| Command buffer too large for single submit | Split into multiple submits with barriers (unlikely for small scenes). |
| GPU dispatch overhead > compute savings | Only enable GPU pipeline for scenes above a geom-count threshold. |
| wgpu limitations (no f64, 16KB shared memory) | Already addressed in existing sim-gpu design. f32 throughout. Newton H fits in shared memory for nv ≤ 60. |
| Stale hinge poses during substeps | Phase 1: accept (margin absorbs). Phase 2: hinge_fk.wgsl. |
| nv > 60 exceeds shared memory for H | Switch to CG solver (§17). Not needed for hockey (nv=13). |

## 17. Future extensions

- **CG solver for large nv:** When nv > 60, the Newton Hessian exceeds
  shared memory. Switch to CG (conjugate gradient) which needs only
  M·v products (streamable) and J^T·D·J·v (sparse). MJX uses CG as
  its alternative solver. Threshold: nv > 60 (matches MJX's
  sparse/dense crossover).

- **GPU FK/CRBA/RNE via level-order scans:** MJX's `scan.body_tree()`
  processes kinematic trees on GPU by grouping bodies by depth and
  vmapping within each level. Enables full GPU step (no split-step)
  and multi-environment batching.

- **Elliptic friction cones on GPU:** Add cone Hessian blocks to the
  Newton Hessian. MJX supports this — each contact contributes a
  dim×dim Hessian block to H via `J^T · C_m · J`. Requires block-aware
  Hessian accumulation.

- **Multi-environment batching:** `vmap`-style parallel simulation of
  thousands of environments for RL training. Requires GPU FK/CRBA/RNE
  + SoA data layout. MJX's primary use case.

- **`hinge_fk.wgsl`:** Lightweight per-substep FK for single-DOF
  joints attached to world. `body_pose = parent_pose × rot(axis, angle)`.
  One mat4 multiply per hinge per substep.

- **Async double-buffer:** Pipeline substep N+1 compute while reading
  substep N results. Hides readback latency completely.

- **ComFree-Sim solver:** Analytical complementarity-free contact.
  Linear scaling on GPU. Drop-in replacement for Newton in §8.6.

- **Noslip post-processor on GPU:** After the main solver, run a
  secondary Jacobi pass on friction rows only to eliminate residual
  slip. Matches CPU's `noslip_postprocess()`.
