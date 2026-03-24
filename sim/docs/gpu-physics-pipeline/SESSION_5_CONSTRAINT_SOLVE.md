# Session 5: Constraint Solve (Assembly + Newton Solver + Force Mapping)

**Branch:** `feature/vr-hockey`
**Date:** 2026-03-24
**Prereq:** Sessions 1–4 complete (FK, CRBA, velocity FK, RNE, smooth, integration, collision)

---

## 1 Scope

### In scope

- `assemble.wgsl` — per-contact parallel: PipelineContact → pyramidal constraint rows (efc_J/D/aref)
- `newton_solve.wgsl` — single-workgroup Newton solver: shared-memory Hessian, Cholesky, backtracking line search
- `map_forces.wgsl` — per-DOF parallel: qfrc_constraint = J^T · efc_force
- `GpuConstraintPipeline` — Rust orchestration: buffer allocation, bind groups, dispatch
- New state buffers: efc_J, efc_D, efc_aref, efc_force, constraint_count, qfrc_constraint
- New model buffer: body_invweight0 (for bodyweight diagApprox)
- Replace gravity-only bridge (`copy qacc_smooth → qacc`) with solver output
- Validation tests T23–T27

### Out of scope

- Non-free joint types (hinge, ball, slide) — free joints only (hockey)
- Warm start (substep-to-substep qacc reuse) — cold start from qacc_smooth
- Elliptic cone — pyramidal only
- condim=6 (rolling friction) — condim=1,3,4 only
- Per-contact solref/solimp — model-level defaults via uniform
- Noslip post-processing
- Batch environments (n_env > 1)
- Sparse Hessian path (nv > 60)

---

## 2 Architecture

### 2.1 Data flow

```
Session 3 output          Session 4 output          Session 5
─────────────────         ──────────────────         ─────────
qacc_smooth ─────────────────────────────────────┐
qfrc_smooth ─────────────────────────────────────┤
qM ──────────────────────────────────────────────┤
qvel ────────────────────────────────────────────┤
body_xpos, body_xquat ──────────────────────────┤
                                                 │
contact_buffer ──────────────────────────────────┤
contact_count ───────────────────────────────────┤
                                                 ▼
                                          ┌─────────────┐
                                          │ assemble.wgsl│
                                          └──────┬──────┘
                                                 │ efc_J, efc_D, efc_aref, constraint_count
                                                 ▼
                                          ┌──────────────────┐
                                          │ newton_solve.wgsl │
                                          └──────┬───────────┘
                                                 │ qacc (final), efc_force
                                                 ▼
                                          ┌────────────────┐
                                          │ map_forces.wgsl │
                                          └──────┬─────────┘
                                                 │ qfrc_constraint
                                                 ▼
                                          ┌──────────────────┐
                                          │ integrate.wgsl    │  (Session 3, reads qacc)
                                          └──────────────────┘
```

### 2.2 Command buffer integration

Replace the gravity-only bridge in the per-substep dispatch:

```rust
// Before (Session 3):
encoder.copy_buffer_to_buffer(&qacc_smooth, 0, &qacc, 0, nv * 4);

// After (Session 5):
constraint_pipeline.encode(&mut encoder, ctx, model_bufs, state_bufs);
// qacc is now written by Newton solver — integration reads it directly
```

### 2.3 Dispatch structure

```rust
// constraint_pipeline.encode():
encoder.clear_buffer(&state_bufs.constraint_count, 0, Some(4));

// Assembly: one thread per contact
let n_contacts = max_contacts;  // dispatch max; shader bounds-checks via contact_count
compute_pass(&encoder, &assemble_pipeline, ceil(n_contacts, 256), 1);

// Newton: single workgroup, internal iteration loop
compute_pass(&encoder, &newton_pipeline, 1, 1);

// Force mapping: one thread per DOF
compute_pass(&encoder, &map_forces_pipeline, ceil(nv, 64), 1);
```

---

## 3 File structure

```
sim/L0/gpu/src/
├── pipeline/
│   ├── mod.rs              ── add: pub mod constraint;
│   ├── types.rs            ── add: AssemblyParams, SolverParams
│   ├── model_buffers.rs    ── add: body_invweight0 buffer + upload
│   ├── state_buffers.rs    ── add: efc_J, efc_D, efc_aref, efc_force,
│   │                               constraint_count, qfrc_constraint
│   ├── constraint.rs       ── NEW: GpuConstraintPipeline
│   └── tests.rs            ── add: T23–T27
├── shaders/
│   ├── assemble.wgsl       ── NEW: ~400 lines
│   ├── newton_solve.wgsl   ── NEW: ~600 lines
│   └── map_forces.wgsl     ── NEW: ~60 lines
```

---

## 4 GPU-side types

### 4.1 AssemblyParams (uniform, 64 bytes)

```wgsl
struct AssemblyParams {
    nv: u32,
    max_contacts: u32,
    max_constraints: u32,
    nbody: u32,
    timestep: f32,
    impratio: f32,
    solref_timeconst: f32,
    solref_dampratio: f32,
    solimp_d0: f32,
    solimp_dwidth: f32,
    solimp_width: f32,
    solimp_midpoint: f32,
    solimp_power: f32,
    _pad0: f32,
    _pad1: f32,
    _pad2: f32,
};
```

```rust
#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
pub struct AssemblyParams {
    pub nv: u32,
    pub max_contacts: u32,
    pub max_constraints: u32,
    pub nbody: u32,
    pub timestep: f32,
    pub impratio: f32,
    pub solref_timeconst: f32,
    pub solref_dampratio: f32,
    pub solimp_d0: f32,
    pub solimp_dwidth: f32,
    pub solimp_width: f32,
    pub solimp_midpoint: f32,
    pub solimp_power: f32,
    pub _pad: [f32; 3],
}
```

### 4.2 SolverParams (uniform, 32 bytes)

```wgsl
struct SolverParams {
    nv: u32,
    max_iter: u32,
    max_ls: u32,
    _pad0: u32,
    tolerance: f32,
    ls_tolerance: f32,
    meaninertia: f32,
    _pad1: f32,
};
```

```rust
#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
pub struct SolverParams {
    pub nv: u32,
    pub max_iter: u32,
    pub max_ls: u32,
    pub _pad0: u32,
    pub tolerance: f32,
    pub ls_tolerance: f32,
    pub meaninertia: f32,
    pub _pad1: f32,
}
```

### 4.3 Existing types used

- `PipelineContact` (48 bytes) — from Session 4, read by assembly
- `GeomModelGpu` (96 bytes) — body_id, condim, friction
- `BodyModelGpu` (112 bytes) — dof_adr, dof_num, parent

---

## 5 Buffer additions

### 5.1 Model buffers (GpuModelBuffers)

```rust
pub body_invweight0: wgpu::Buffer,  // nbody × vec4<f32> [trans, rot, 0, 0]
```

Upload from `model.body_invweight0`:
```rust
let data: Vec<[f32; 4]> = (0..model.nbody).map(|b| {
    [
        model.body_invweight0[b][0] as f32,
        model.body_invweight0[b][1] as f32,
        0.0, 0.0,
    ]
}).collect();
```

### 5.2 State buffers (GpuStateBuffers)

| Buffer | Size | Type | Usage |
|--------|------|------|-------|
| `efc_J` | max_constraints × nv × 4 | f32 | Assembly write, Newton read, map_forces read |
| `efc_D` | max_constraints × 4 | f32 | Assembly write, Newton read |
| `efc_aref` | max_constraints × 4 | f32 | Assembly write, Newton read |
| `efc_force` | max_constraints × 4 | f32 | Newton write, map_forces read |
| `constraint_count` | 4 | atomic u32 | Assembly write (atomicAdd), Newton read |
| `qfrc_constraint` | nv × 4 | f32 | map_forces write |

Pre-allocation: `max_constraints = 196_608` (6 × 32,768).

Total new memory: 196,608 × (nv + 3) × 4 bytes. At nv=13: 196,608 × 16 × 4 ≈ 12.6 MB. At nv=60: 196,608 × 63 × 4 ≈ 49.5 MB.

---

## 6 Shader specifications

### 6.1 assemble.wgsl (~400 lines)

**Pattern:** One thread per contact, `@workgroup_size(256)`.
**Reference:** CPU `contact_assembly.rs:assemble_pyramidal_contact()` + `jacobian.rs:compute_contact_jacobian()`

#### Bindings (13 total, 4 groups)

| Group | Binding | Type | Buffer |
|-------|---------|------|--------|
| 0 | 0 | uniform | AssemblyParams |
| 1 | 0 | storage read | geoms (GeomModel array) |
| 1 | 1 | storage read | bodies (BodyModel array) |
| 1 | 2 | storage read | body_invweight0 (vec4 array) |
| 2 | 0 | storage read | contact_buffer (PipelineContact array) |
| 2 | 1 | storage read | contact_count (atomic u32) |
| 2 | 2 | storage read | body_xpos (vec4 array) |
| 2 | 3 | storage read | body_xquat (vec4 array) |
| 2 | 4 | storage read | qvel (f32 array) |
| 3 | 0 | storage rw | efc_J (f32 array) |
| 3 | 1 | storage rw | efc_D (f32 array) |
| 3 | 2 | storage rw | efc_aref (f32 array) |
| 3 | 3 | storage rw | constraint_count (atomic u32) |

#### Algorithm (per thread = per contact)

```
1. ci = global_invocation_id.x
2. if ci >= atomicLoad(contact_count): return
3. Read contact = contact_buffer[ci]
4. Look up condim = min(geoms[contact.geom1].condim, geoms[contact.geom2].condim)
5. Look up body1 = geoms[contact.geom1].body_id, body2 = geoms[contact.geom2].body_id

6. Compute tangent frame from contact.normal:
     ref = select((1,0,0), (0,1,0), |normal.x| > 0.9)
     t1 = normalize(cross(normal, ref))
     t2 = cross(normal, t1)

7. Determine row count:
     condim=1 → n_rows=1 (frictionless)
     condim=3 → n_rows=4 (2 tangent directions × 2 facets)
     condim=4 → n_rows=6 (2 tangent + 1 torsional × 2 facets)

8. Allocate rows: row_start = atomicAdd(constraint_count, n_rows)
   if row_start + n_rows > max_constraints: return

9. For each allocated row:
   a. Zero efc_J[row, 0..nv]
   b. Compute Jacobian (see §6.1.1)
   c. Compute vel = J_row · qvel
   d. Compute impedance, diagApprox, R, D, KBIP, aref (see §6.1.2)
   e. Write efc_D[row], efc_aref[row]
```

#### 6.1.1 Jacobian computation (free joints only)

For each body B in {body1 (sign=-1), body2 (sign=+1)}:
```
if B == 0: skip (world body, no DOFs)
dof_adr = bodies[B].dof_adr
r = contact.point - body_xpos[B].xyz
r = stabilize_lever_arm(r, normal)   // project out sub-physical lateral drift
R_mat = quat_to_mat3(body_xquat[B])  // 3×3 rotation matrix

For the facet direction dir:
  // Translational DOFs
  efc_J[row, dof_adr+0..3] += sign × dir

  // Angular DOFs
  for i in 0..3:
    omega_i = R_mat[i]  // i-th column = R·e_i
    efc_J[row, dof_adr+3+i] += sign × dot(dir, cross(omega_i, r))
```

**Sliding facets (condim ≥ 3):** The facet direction `dir` applies uniformly to both translational and angular DOFs:
- Positive facet: `dir = normal + mu × tangent_d`
- Negative facet: `dir = normal - mu × tangent_d`
- `mu = contact.friction[0]` (sliding friction)

**Torsional facets (condim = 4):** Translational and angular directions decouple:
- Translational: `efc_J[row, dof+0..3] += sign × normal` (same for both +/- facets)
- Angular: `efc_J[row, dof+3+i] += sign × (dot(normal, cross(omega_i, r)) ± mu_torsion × dot(normal, omega_i))`
- `mu_torsion = contact.friction[1]`

**Frictionless (condim = 1):** Single row with `dir = normal`.

#### 6.1.2 Impedance and regularization

Per constraint row:
```
pos = -contact.depth  (negative = penetration)
margin = 0.0          (contacts have zero margin on GPU)
violation = |pos - margin| = contact.depth

// Impedance sigmoid
imp = compute_impedance(solimp, violation)

// diagApprox (bodyweight, O(1))
// For sliding facets: (1 + mu²) × (invweight0[b1].x + invweight0[b2].x)
// For frictionless:   invweight0[b1].x + invweight0[b2].x
// For torsional:      (1 + mu²) × (invweight0[b1].x + invweight0[b2].x)
// (Pyramidal uses translational bodyweight for all rows, scaled by 1+mu²)
diag = max(bw_diag, 1e-15)

// Regularization
R = max((1.0 - imp) / imp × diag, 1e-15)
D = 1.0 / R

// KBIP (default solref = [0.02, 1.0], clamped by timestep)
dmax = clamp(solimp_dwidth, 0.0001, 0.9999)
timeconst = max(solref_timeconst, 2.0 × timestep)
dampratio = solref_dampratio
K = 1.0 / max(dmax² × timeconst² × dampratio², 1e-15)
B = 2.0 / max(dmax × timeconst, 1e-15)

// Reference acceleration
vel = J_row · qvel   // dot product, computed per-row
aref = -B × vel - K × imp × pos
```

#### 6.1.3 Helper functions

```wgsl
fn quat_to_mat3(q: vec4<f32>) -> mat3x3<f32>
    // q = (x,y,z,w) nalgebra convention → 3×3 rotation matrix

fn compute_tangent_frame(normal: vec3<f32>) -> array<vec3<f32>, 2>
    // Returns (tangent1, tangent2) orthonormal to normal

fn stabilize_lever_arm(r: vec3<f32>, normal: vec3<f32>) -> vec3<f32>
    // Project out sub-physical perpendicular drift (threshold 1e-6)

fn compute_impedance(solimp: SolimpParams, violation: f32) -> f32
    // Sigmoid mapping: violation → impedance in [0.0001, 0.9999]
```

---

### 6.2 newton_solve.wgsl (~600 lines)

**Pattern:** Single workgroup, `@workgroup_size(256)`. All Newton iterations in one dispatch.
**Reference:** CPU `newton.rs:newton_solve()` + `primal.rs:primal_prepare/eval/search()`

#### Bindings (10 total, 4 groups)

| Group | Binding | Type | Buffer |
|-------|---------|------|--------|
| 0 | 0 | uniform | SolverParams |
| 1 | 0 | storage read | qM (f32 array, nv×nv row-major) |
| 1 | 1 | storage read | qacc_smooth (f32 array) |
| 1 | 2 | storage read | qfrc_smooth (f32 array) |
| 2 | 0 | storage read | efc_J (f32 array) |
| 2 | 1 | storage read | efc_D (f32 array) |
| 2 | 2 | storage read | efc_aref (f32 array) |
| 2 | 3 | storage read | constraint_count (atomic u32) |
| 3 | 0 | storage rw | qacc (f32 array) |
| 3 | 1 | storage rw | efc_force (f32 array) |

#### Shared memory layout (16,384 bytes at MAX_NV=60)

```wgsl
const MAX_NV: u32 = 60u;

var<workgroup> H_atomic: array<atomic<u32>, 3600>;  // MAX_NV² = 14,400 bytes
var<workgroup> qacc_sh: array<f32, 60>;              // 240 bytes
var<workgroup> qacc_sm_sh: array<f32, 60>;           // 240 bytes
var<workgroup> grad_sh: array<f32, 60>;              // 240 bytes
var<workgroup> search_sh: array<f32, 60>;            // 240 bytes
var<workgroup> reduction_sh: array<f32, 256>;        // 1,024 bytes
                                                     // Total: 16,384 bytes
```

Ma (M × qacc) and qfrc_smooth are computed from global memory reads on-the-fly.

#### Algorithm

```
Entry: newton_solve(@builtin(local_invocation_id) lid)
  tid = lid.x
  nv = params.nv
  nefc = atomicLoad(constraint_count)

  // ── INITIALIZE ──────────────────────────────────────────
  // Cold start: qacc = qacc_smooth
  if tid < nv:
    qacc_sh[tid] = qacc_smooth_buf[tid]
    qacc_sm_sh[tid] = qacc_smooth_buf[tid]
  workgroupBarrier()

  // Early exit: no constraints → qacc = qacc_smooth
  if nefc == 0:
    if tid < nv: qacc_buf[tid] = qacc_sh[tid]
    return

  // ── NEWTON ITERATION LOOP ──────────────────────────────
  var done = 0u  (shared)
  for iter in 0..max_iter:
    if done != 0: break

    // ── PHASE 1: Initialize H = M ─────────────────────────
    // 256 threads cooperate to load M into H_atomic
    for idx in tid..nv*nv step 256:
      atomicStore(H_atomic[idx], bitcast<u32>(qM_buf[idx]))
    workgroupBarrier()

    // ── PHASE 2: Classify + accumulate H ───────────────────
    // Each thread handles rows tid, tid+256, tid+512, ...
    for i in tid..nefc step 256:
      // Compute jar[i] = J[i] · qacc - aref[i]
      jar_i = dot(efc_J[i,:], qacc_sh) - efc_aref[i]

      if jar_i >= 0.0: continue  // satisfied

      // Accumulate H += D[i] × J[i]^T × J[i]
      d_i = efc_D[i]
      for r in 0..nv:
        j_r = efc_J[i, r]
        if j_r == 0.0: continue
        d_j_r = d_i × j_r
        for c in r..nv:
          j_c = efc_J[i, c]
          if j_c == 0.0: continue
          val = d_j_r × j_c
          atomic_add_f32_H(r*nv + c, val)
          if r != c:
            atomic_add_f32_H(c*nv + r, val)
    workgroupBarrier()

    // ── PHASE 3: Cholesky factorize H (thread 0) ──────────
    if tid == 0:
      for j in 0..nv:
        sum = H_f32(j, j)
        for k in 0..j: sum -= H_f32(j, k)²
        set_H_f32(j, j, sqrt(max(sum, 1e-10)))
        ljj = H_f32(j, j)
        for i in j+1..nv:
          s = H_f32(i, j)
          for k in 0..j: s -= H_f32(i, k) × H_f32(j, k)
          set_H_f32(i, j, s / ljj)
    workgroupBarrier()

    // ── PHASE 4: Compute efc_force + gradient ──────────────
    // efc_force[i] = -D[i] × jar[i] × (jar < 0)
    // grad[k] = Ma[k] - qfrc_smooth[k] - Σ_i efc_force[i] × J[i,k]

    // 4a: Compute Ma[k] = Σ_j M[k,j] × qacc[j]  (first nv threads)
    //     and load qfrc_smooth[k]  (first nv threads)
    // These are stored into grad_sh as: grad[k] = Ma[k] - qfrc_smooth[k]
    if tid < nv:
      var ma_k = 0.0
      for j in 0..nv: ma_k += qM_buf[tid*nv + j] × qacc_sh[j]
      grad_sh[tid] = ma_k - qfrc_smooth_buf[tid]
    workgroupBarrier()

    // 4b: Compute J^T × efc_force via per-DOF reduction
    for k in 0..nv:
      var partial = 0.0
      for i in tid..nefc step 256:
        jar_i = dot(efc_J[i,:], qacc_sh) - efc_aref[i]
        if jar_i < 0.0:
          efc_force_i = -efc_D[i] × jar_i
          partial += efc_force_i × efc_J[i*nv + k]
      reduction_sh[tid] = partial
      workgroupBarrier()
      // Tree reduction
      for s in {128, 64, 32, 16, 8, 4, 2, 1}:
        if tid < s: reduction_sh[tid] += reduction_sh[tid + s]
        workgroupBarrier()
      if tid == 0:
        grad_sh[k] -= reduction_sh[0]  // grad = (Ma - qfrc) - J^T·force
      workgroupBarrier()

    // ── PHASE 5: Search direction (thread 0) ───────────────
    // search = -H⁻¹ × grad via Cholesky solve
    if tid == 0:
      // Copy grad → search
      for i in 0..nv: search_sh[i] = grad_sh[i]
      // Forward: L y = b
      for i in 0..nv:
        for k in 0..i: search_sh[i] -= H_f32(i, k) × search_sh[k]
        search_sh[i] /= H_f32(i, i)
      // Backward: L^T x = y
      for i_rev in 0..nv:
        i = nv - 1 - i_rev
        for k in i+1..nv: search_sh[i] -= H_f32(k, i) × search_sh[k]
        search_sh[i] /= H_f32(i, i)
      // Negate
      for i in 0..nv: search_sh[i] = -search_sh[i]
    workgroupBarrier()

    // ── PHASE 6: Backtracking line search ──────────────────
    // Evaluate cost at 4 candidate alphas: {1.0, 0.5, 0.25, 0.125}
    // Also evaluate cost at alpha=0 (current qacc)

    // cost(alpha) = Gauss(alpha) + Σ_active ½·D·jar²
    // where jar_trial = jar(qacc + alpha×search)
    // All 256 threads participate in each evaluation

    var best_alpha = 0.0  (shared)
    cost_0 = parallel_cost_eval(alpha=0.0)
    cost_1 = parallel_cost_eval(alpha=1.0)
    cost_05 = parallel_cost_eval(alpha=0.5)
    cost_025 = parallel_cost_eval(alpha=0.25)

    if tid == 0:
      best = cost_0
      best_alpha = 0.0
      if cost_1 < best: best = cost_1; best_alpha = 1.0
      if cost_05 < best: best = cost_05; best_alpha = 0.5
      if cost_025 < best: best = cost_025; best_alpha = 0.25
    workgroupBarrier()

    // ── PHASE 7: Update qacc ───────────────────────────────
    if best_alpha > 0.0:
      if tid < nv: qacc_sh[tid] += best_alpha × search_sh[tid]
    else:
      if tid == 0: done = 1
    workgroupBarrier()

  // ── FINALIZE ──────────────────────────────────────────────
  // Write qacc to global memory
  if tid < nv:
    qacc_buf[tid] = qacc_sh[tid]

  // Write final efc_force to global memory
  for i in tid..nefc step 256:
    jar_i = dot(efc_J[i,:], qacc_sh) - efc_aref[i]
    if jar_i < 0.0:
      efc_force_buf[i] = -efc_D[i] × jar_i
    else:
      efc_force_buf[i] = 0.0
```

#### 6.2.1 Parallel cost evaluation

```wgsl
fn parallel_cost_eval(alpha: f32) -> f32:
  // Each thread computes partial cost over its constraint stripe
  var partial = 0.0

  // Constraint cost
  for i in tid..nefc step 256:
    var jar_trial = -efc_aref_buf[i]
    for k in 0..nv:
      jar_trial += efc_J_buf[i*nv + k] × (qacc_sh[k] + alpha × search_sh[k])
    if jar_trial < 0.0:
      partial += 0.5 × efc_D_buf[i] × jar_trial × jar_trial

  // Gauss cost (first nv threads)
  if tid < nv:
    var ma_k = 0.0
    for j in 0..nv:
      ma_k += qM_buf[tid*nv + j] × (qacc_sh[j] + alpha × search_sh[j])
    let u_k = (qacc_sh[tid] + alpha × search_sh[tid]) - qacc_sm_sh[tid]
    partial += 0.5 × u_k × (ma_k - qfrc_smooth_buf[tid])

  // Reduce 256 partial sums
  reduction_sh[tid] = partial
  workgroupBarrier()
  for s in {128, 64, 32, 16, 8, 4, 2, 1}:
    if tid < s: reduction_sh[tid] += reduction_sh[tid + s]
    workgroupBarrier()

  return reduction_sh[0]  // only thread 0 reads this
```

#### 6.2.2 CAS atomic helpers

```wgsl
fn H_f32(r: u32, c: u32) -> f32 {
    return bitcast<f32>(atomicLoad(&H_atomic[r * MAX_NV + c]));
}
fn set_H_f32(r: u32, c: u32, val: f32) {
    atomicStore(&H_atomic[r * MAX_NV + c], bitcast<u32>(val));
}
fn atomic_add_f32_H(idx: u32, val: f32) {
    var old = atomicLoad(&H_atomic[idx]);
    loop {
        let new_val = bitcast<u32>(bitcast<f32>(old) + val);
        let result = atomicCompareExchangeWeak(&H_atomic[idx], old, new_val);
        if result.exchanged { break; }
        old = result.old_value;
    }
}
```

---

### 6.3 map_forces.wgsl (~60 lines)

**Pattern:** Per-DOF parallel, `@workgroup_size(64)`.
**Reference:** CPU `solver/mod.rs:compute_qfrc_constraint_from_efc()`

#### Bindings (5 total, 3 groups)

| Group | Binding | Type | Buffer |
|-------|---------|------|--------|
| 0 | 0 | uniform | SolverParams |
| 1 | 0 | storage read | efc_J |
| 1 | 1 | storage read | efc_force |
| 1 | 2 | storage read | constraint_count |
| 2 | 0 | storage rw | qfrc_constraint |

#### Algorithm

```wgsl
@compute @workgroup_size(64)
fn map_forces(@builtin(global_invocation_id) gid: vec3<u32>) {
    let d = gid.x;
    if d >= params.nv { return; }

    let nefc = atomicLoad(&constraint_count_buf[0]);
    var total = 0.0;
    for (var i = 0u; i < nefc; i++) {
        total += efc_force_buf[i] * efc_J_buf[i * params.nv + d];
    }
    qfrc_constraint_buf[d] = total;
}
```

---

## 7 Rust pipeline module: constraint.rs

### 7.1 GpuConstraintPipeline

```rust
pub struct GpuConstraintPipeline {
    assemble_pipeline: wgpu::ComputePipeline,
    newton_pipeline: wgpu::ComputePipeline,
    map_forces_pipeline: wgpu::ComputePipeline,

    assemble_layout: wgpu::BindGroupLayout,
    newton_layout: wgpu::BindGroupLayout,
    map_forces_layout: wgpu::BindGroupLayout,

    assemble_params_buf: wgpu::Buffer,
    solver_params_buf: wgpu::Buffer,

    // Pre-built bind groups
    assemble_bg: [wgpu::BindGroup; 4],
    newton_bg: [wgpu::BindGroup; 4],
    map_forces_bg: [wgpu::BindGroup; 3],

    nv: u32,
    max_contacts: u32,
    max_constraints: u32,
}
```

### 7.2 Construction

```rust
impl GpuConstraintPipeline {
    pub fn new(
        ctx: &GpuContext,
        model_bufs: &GpuModelBuffers,
        state_bufs: &GpuStateBuffers,
        nv: u32,
        max_contacts: u32,
        model: &Model,
    ) -> Self {
        // 1. Compile shaders (assemble.wgsl, newton_solve.wgsl, map_forces.wgsl)
        // 2. Create bind group layouts
        // 3. Create compute pipelines
        // 4. Create uniform buffers
        // 5. Write assembly params (solref, solimp, timestep, etc.)
        // 6. Write solver params (max_iter, tolerance, etc.)
        // 7. Create bind groups
    }
}
```

### 7.3 Dispatch

```rust
impl GpuConstraintPipeline {
    pub fn encode(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        state_bufs: &GpuStateBuffers,
    ) {
        // 1. Clear constraint_count
        encoder.clear_buffer(&state_bufs.constraint_count, 0, Some(4));

        // 2. Assembly dispatch
        {
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_pipeline(&self.assemble_pipeline);
            for (i, bg) in self.assemble_bg.iter().enumerate() {
                pass.set_bind_group(i as u32, bg, &[]);
            }
            pass.dispatch_workgroups(
                (self.max_contacts + 255) / 256, 1, 1
            );
        }

        // 3. Newton dispatch (single workgroup)
        {
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_pipeline(&self.newton_pipeline);
            for (i, bg) in self.newton_bg.iter().enumerate() {
                pass.set_bind_group(i as u32, bg, &[]);
            }
            pass.dispatch_workgroups(1, 1, 1);
        }

        // 4. Force mapping dispatch
        {
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_pipeline(&self.map_forces_pipeline);
            for (i, bg) in self.map_forces_bg.iter().enumerate() {
                pass.set_bind_group(i as u32, bg, &[]);
            }
            pass.dispatch_workgroups(
                (self.nv + 63) / 64, 1, 1
            );
        }
    }
}
```

---

## 8 Validation tests

### T23: Assembly validation

**Setup:** 1 free body (SDF sphere) partially below ground plane. condim=3.

**Verify:**
- constraint_count == ncontacts × 4 (pyramidal, condim=3)
- efc_J rows have correct nonzero pattern (6 entries per body per row for free joints, ground body = 0 entries)
- efc_D values match CPU bodyweight diagApprox → regularization pipeline
- efc_aref values match CPU KBIP computation
- Tolerance: 1e-3 (tangent frame may differ from CPU)

### T24: Newton solver — sphere on ground

**Setup:** Same scene as T23. Run full pipeline: FK → CRBA → vel FK → RNE → smooth → collision → **constraint solve**.

**Verify:**
- GPU qacc vs CPU Newton solver qacc
- Tolerance: 1e-2 (f32 vs f64, tangent frame differences, bodyweight vs exact diagApprox)
- Constraint forces push the sphere upward (qacc[2] > qacc_smooth[2] for vertical DOF)

### T25: Zero contacts — no collision

**Setup:** SDF sphere well above ground (no penetration).

**Verify:**
- constraint_count == 0
- GPU qacc == GPU qacc_smooth (exactly, no solver modification)
- qfrc_constraint == 0

### T26: Multi-body — hockey-like scene

**Setup:** 3-body scene: puck (free) + stick (free) + ground plane. Puck and stick overlap slightly.

**Verify:**
- GPU qacc vs CPU qacc for all bodies
- Both SDF-SDF (puck↔stick) and SDF-plane (puck↔ground, stick↔ground) contacts contribute
- Tolerance: 5e-2 (multiple contact patches, tangent frame variability)

### T27: Multi-substep stability

**Setup:** Puck dropped onto ground from small height. Run 10 substeps of the full pipeline (FK through integration, looped).

**Verify:**
- All qpos and qvel values are finite after 10 substeps
- Position magnitude < 100m (no explosion)
- Puck's vertical position stabilizes (not passing through ground)

---

## 9 Deviations from master spec

| Deviation | Rationale | Future upgrade |
|-----------|-----------|----------------|
| Free joints only | Hockey has only free bodies. General joints require kinematic chain traversal on GPU. | Add joint-type dispatch in assembly shader |
| Cold start (no warm start) | Simpler. Warmstart benefit is ~1 fewer Newton iteration per substep. | Compare cost at qacc vs qacc_smooth before iterating |
| Backtracking line search (not bracket-narrowing Newton) | Simpler, correct, 4 evaluations per iteration. Full Newton 1D adds complexity without clear benefit at f32 precision. | Upgrade to bracket-narrowing if convergence is slow |
| condim=1,3,4 only | condim=6 (rolling friction) rare for hockey. Assembly has clear extension point. | Add rolling facets (rows 4-5) |
| Pyramidal only (no elliptic) | Master spec mandates pyramidal for GPU (simpler, matches MJX). | N/A — pyramidal is the GPU design |
| Default solref/solimp only | Per-contact params require per-geom solref/solimp in GeomModelGpu. | Extend GeomModelGpu with solref/solimp |
| Bodyweight diagApprox | Exact diagApprox requires M⁻¹ solve per row (O(nv) each). Bodyweight is O(1). | N/A — bodyweight is the standard GPU choice (MJX uses it too) |

---

## 10 Risk assessment

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| Shared memory exceeds 16 KB | Low | Blocker | Layout verified: exactly 16,384 bytes at MAX_NV=60 |
| Storage buffer count > 16 | Low | Blocker | Assembly: 13, Newton: 10, map_forces: 5 — all under 16 |
| CAS contention on H_atomic | Medium | Performance | For nv=13, H has 169 entries. 256 threads × 7 rows each = low contention |
| f32 Cholesky numerical issues | Medium | Solver failure | `sqrt(max(sum, 1e-10))` guard. PGS fallback not on GPU — solver writes qacc_smooth if Cholesky fails |
| Tangent frame mismatch (GPU vs CPU) | Certain | Test tolerance | Validate qacc (physical result) not J (frame-dependent). Use 1e-2 tolerance for qacc |
| SDF duplicate contacts | Low | Over-provisioned rows | 196K max_constraints >> hockey max ~3600 used rows |
| Torsional Jacobian error | Low | Incorrect friction | Torsional facets tested separately (condim=4 path) |

---

## 11 Implementation order

1. **Types** — AssemblyParams, SolverParams in types.rs
2. **Model buffers** — body_invweight0 upload in model_buffers.rs
3. **State buffers** — efc_J/D/aref/force, constraint_count, qfrc_constraint in state_buffers.rs
4. **assemble.wgsl** — write shader, condim=1,3 first, then add condim=4
5. **Assembly test (T23)** — verify constraint rows against CPU
6. **newton_solve.wgsl** — write shader, test standalone
7. **map_forces.wgsl** — write shader
8. **GpuConstraintPipeline** — Rust orchestration, bind groups, encode()
9. **Integration** — replace gravity-only bridge, wire into existing pipeline
10. **Full pipeline tests (T24–T27)** — compare against CPU, stability
11. **Verify existing tests** — T1–T22 still pass after buffer additions

---

## 12 Performance estimate (hockey, nv=13, ~600 contacts, ~2400 constraint rows)

| Stage | Threads | Work per thread | Estimated time |
|-------|---------|-----------------|----------------|
| Assembly | 600 | ~200 flops + 6×13 writes | ~10 μs |
| Newton (per iter) | 256 | Phase 2: ~2400/256 × 169 CAS | ~15 μs |
| | | Phase 3: Cholesky 13³/6 ≈ 366 flops | ~1 μs |
| | | Phase 4: 13 reductions × 8 steps | ~5 μs |
| | | Phase 5: Cholesky solve ~338 flops | ~1 μs |
| | | Phase 6: 4 cost evals × ~260 flops/thread | ~5 μs |
| Newton total (3 iter) | | | ~80 μs |
| Force mapping | 13 | 2400 multiply-adds | ~5 μs |
| **Session 5 total** | | | **~95 μs** |

Well within the 1 ms per substep budget. Combined with Session 3+4 (~150 μs), total pipeline is ~245 μs per substep.
