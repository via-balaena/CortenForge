# Future Work 10 — Phase 3E: GPU Pipeline (Items #35–39)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

Phase 10a (Euler velocity integration on GPU) is complete in `sim-gpu`. These items
progressively move more pipeline stages to GPU, targeting full GPU-resident simulation
where only `ctrl` upload and `qpos`/`qvel`/`sensordata` download cross PCIe.

All items build on the wgpu infrastructure, SoA buffer layout, and
environment-per-workgroup model established in Phase 10a. They form a strict
sequential chain: each phase depends on the previous.

---

### 35. GPU Forward Kinematics (Phase 10b)
**Status:** Not started | **Effort:** XL | **Prerequisites:** #10 (Phase 10a)

#### Current State
FK (`mj_kinematics`, `mj_com_pos`) runs entirely on CPU. This is the largest
single-stage cost for models with many bodies, as it involves tree traversal
(parent-to-child propagation of body poses).

#### Objective
Port forward kinematics to a GPU compute shader using level-set parallel traversal.

#### Specification

1. **Level-set parallelism**: Pre-compute tree levels at model build time. Bodies at
   the same depth have no parent-child dependency and can be computed in parallel.
   Dispatch one workgroup pass per tree level, synchronizing between levels.
2. **Buffers** (new SoA uploads):
   - `gpu_qpos` — generalized positions (f32)
   - `gpu_xpos`, `gpu_xquat` — output body poses (f32)
   - `GpuModelHeader` — body tree topology, joint parameters (uniform buffer,
     uploaded once at construction)
3. **Joint types**: Shader must handle hinge, slide, ball, free joint position→pose
   transforms. Dispatch per joint type or use branching within a unified kernel.
4. **COM computation**: After body poses, compute subtree COM in a second pass
   (bottom-up tree reduction).

#### Acceptance Criteria
1. GPU FK matches CPU FK within f32 tolerance for a 50-body humanoid.
2. Throughput exceeds CPU for ≥64 parallel environments.
3. CPU fallback path unchanged (regression).

#### Files
- `sim/L0/gpu/src/` — new `fk.wgsl` shader, buffer management
- `sim/L0/gpu/src/lib.rs` — integrate FK pass into `GpuBatchSim::step_all()`

---

### 36. GPU Collision Broad-Phase (Phase 10c)
**Status:** Not started | **Effort:** L | **Prerequisites:** #35 (Phase 10b)

#### Current State
Broad-phase collision (AABB overlap testing, `contype`/`conaffinity` filtering) runs
on CPU. For scenes with many geoms, this becomes a bottleneck.

#### Objective
Port AABB broad-phase to GPU using parallel sweep-and-prune or spatial hashing.

#### Specification

1. **AABB computation**: After GPU FK, compute world-space AABBs from geom poses +
   sizes on GPU (one thread per geom).
2. **Overlap detection**: Use sort-and-sweep on the x-axis (GPU radix sort + parallel
   scan) or spatial hash grid.
3. **Filtering**: Apply `contype & conaffinity` bitmask test on GPU.
4. **Output**: Compact list of candidate pairs per environment (atomic counter +
   append buffer).
5. **Download**: Only candidate pair indices cross PCIe (much smaller than full
   geom data).

#### Acceptance Criteria
1. GPU broad-phase produces the same candidate pairs as CPU for a reference scene.
2. Throughput exceeds CPU for ≥64 environments with ≥20 geoms each.
3. CPU fallback path unchanged (regression).

#### Files
- `sim/L0/gpu/src/` — new `broadphase.wgsl` shader
- `sim/L0/gpu/src/lib.rs` — integrate broad-phase into step pipeline

---

### 37. GPU Collision Narrow-Phase (Phase 10d)
**Status:** Not started | **Effort:** XL | **Prerequisites:** #36 (Phase 10c)

#### Current State
Narrow-phase collision (GJK/EPA, primitive-primitive) runs on CPU. This is the most
algorithmically complex stage to port — GJK is iterative with variable-length loops
and EPA involves dynamic polytope expansion.

#### Objective
Port narrow-phase contact generation to GPU compute shaders.

#### Specification

1. **Primitive pairs**: Start with sphere-sphere, sphere-capsule, sphere-plane,
   capsule-capsule, box-plane (closed-form or simple iterative). These cover the
   majority of contacts in typical RL models.
2. **GJK/EPA**: Port iterative GJK distance + EPA penetration to GPU. Use fixed
   iteration caps to avoid divergent warps.
3. **Contact output**: Per-pair contact struct (pos, normal, depth, friction) written
   to a dynamic contact buffer with per-environment atomic counters.
4. **Condim dispatch**: Contact dimension determined by geom pair properties on GPU.

#### Acceptance Criteria
1. GPU narrow-phase matches CPU contact generation (positions within f32 tolerance).
2. GJK convergence within fixed iteration cap for ≥99% of pairs.
3. CPU fallback for unsupported pair types (hfield, SDF, mesh-mesh).

#### Files
- `sim/L0/gpu/src/` — `narrowphase.wgsl`, `gjk_gpu.wgsl` shaders
- `sim/L0/gpu/src/lib.rs` — integrate narrow-phase, contact buffer management

---

### 38. GPU Constraint Solver (Phase 10e)
**Status:** Not started | **Effort:** XL | **Prerequisites:** #37 (Phase 10d)

#### Current State
Constraint solving (PGS, CG, Newton) runs on CPU. PGS is inherently sequential
(Gauss-Seidel updates use already-modified values). GPU requires a parallel variant.

#### Objective
Implement a Jacobi-style parallel constraint solver on GPU.

#### Specification

1. **Algorithm**: Jacobi PGS (all constraints updated simultaneously using values
   from the previous iteration, not the current one). Converges slower than GS per
   iteration but is fully parallel.
2. **Delassus assembly**: Compute `A = J M^{-1} J^T + R` on GPU. For small models,
   use dense matrix. For large models, exploit sparsity (only contacts sharing
   bodies have non-zero off-diagonal blocks).
3. **Iteration**: Each iteration is a single dispatch:
   - Compute residuals for all constraints in parallel
   - Update lambda values (Jacobi step)
   - Project onto friction cones per-contact
4. **Warmstart**: Frame-coherent warmstart buffer replaces CPU's HashMap-based
   `WarmstartKey` system.
5. **Convergence**: Check max-delta across all constraints via parallel reduction.

#### Acceptance Criteria
1. GPU solver produces contact forces within 5% of CPU solver for reference scenes.
2. Convergence within `solver_iterations` for well-conditioned problems.
3. Throughput exceeds CPU for ≥64 environments.
4. PGS fallback on CPU unchanged (regression).

#### Files
- `sim/L0/gpu/src/` — `constraint_solver.wgsl`, `delassus.wgsl` shaders
- `sim/L0/gpu/src/lib.rs` — solver dispatch, warmstart buffer management

---

### 39. Full GPU Step (Phase 10f)
**Status:** Not started | **Effort:** L | **Prerequisites:** #35–38 (Phases 10b–10e)

#### Current State
Each pipeline stage uploads/downloads data across PCIe. The CPU orchestrates stage
sequencing. This PCIe overhead dominates for small-to-medium models.

#### Objective
Chain all GPU stages into a single command buffer submission, minimizing PCIe traffic
to only `ctrl` upload and `qpos`/`qvel`/`sensordata` download.

#### Specification

1. **Single submission**: Record FK → broad-phase → narrow-phase → dynamics →
   constraint solve → integration as a single wgpu command buffer with appropriate
   barriers between stages.
2. **Persistent buffers**: All intermediate data (xpos, xquat, contacts, lambda)
   lives in GPU-local memory. Only `ctrl` crosses PCIe host→device, and
   `qpos`/`qvel`/`sensordata` cross device→host.
3. **Dynamics on GPU**: Port `mj_fwd_acceleration` (RNEA inverse dynamics, Coriolis,
   gravity) to GPU. This is the remaining CPU stage after 10b–10e.
4. **Sensor evaluation**: Compute pipeline sensors on GPU (position/velocity sensors
   read from GPU-resident body poses).

#### Acceptance Criteria
1. Full GPU step matches CPU step within f32 tolerance.
2. PCIe traffic is O(nv + nsensor) per step, not O(nbody + ngeom + ncon).
3. Throughput exceeds CPU by ≥5× for 256+ environments on representative models.
4. CPU-only path unchanged (regression).

#### Files
- `sim/L0/gpu/src/` — command buffer chaining, dynamics shader, sensor shader
- `sim/L0/gpu/src/lib.rs` — unified step dispatch
