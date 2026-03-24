# GPU Physics Pipeline — Limitations & Workarounds

Constraints imposed by wgpu, WGSL, and target hardware (Metal/Vulkan)
that shaped architectural decisions. Referenced by session specs and the
master spec.

## 1. Storage buffer limit per shader stage

**Constraint:** `WebGPU` default `maxStorageBuffersPerShaderStage = 8`.
Metal hardware supports 31, Vulkan/NVIDIA supports 1M+, but wgpu's
`Limits::default()` enforces the `WebGPU` standard.

**Impact:** The spec's §6.2 described 18+ individual buffers for static
model data (body_parent, body_depth, jnt_type, jnt_axis, etc.). This
exceeds 8 per stage even with separate bind groups, since the per-stage
limit counts ALL storage bindings used by the entry point, across all
groups.

**Workaround:** Pack related fields into struct arrays:
- `BodyModelGpu` (112 bytes): 7 u32 fields + 5 vec4 fields → 1 buffer
- `JointModelGpu` (48 bytes): 3 u32 fields + 2 vec4 fields → 1 buffer
- `GeomModelGpu` (48 bytes): 1 u32 field + 2 vec4 fields → 1 buffer

Group 1 (static model): 2 buffers instead of 18.

**Device limit raised to 16** in `GpuContext` to accommodate FK's ~12
storage bindings per shader stage (2 model + 10 state). This works on
all Metal GPUs (limit 31) and all Vulkan GPUs.

**Future risk:** If a later shader (e.g., newton_solve.wgsl) needs >16
storage buffers, we'll need to either pack more aggressively or split
the shader into multiple dispatches with fewer bindings each.

## 2. No WGSL `#include` / module system

**Constraint:** WGSL has no include mechanism. Each `.wgsl` file is a
standalone compilation unit.

**Impact:** Shared math (quaternion multiply, normalize, rotate,
axis_angle, quat_to_mat3) must be copy-pasted into every shader file.

**Workaround:** `common.wgsl` exists as a standalone reference copy.
Each shader (fk.wgsl, future crba.wgsl, etc.) inlines the math
functions it needs. A bug fix in quat_mul must be applied to all files.

**Mitigation:** We can automate concatenation via build.rs if the
duplication becomes a maintenance burden, or use naga's WGSL
composition features if/when they stabilize.

## 3. vec4 alignment for storage buffer arrays

**Constraint:** WGSL `array<T>` in storage buffers requires elements
to be aligned to `alignof(T)`. For `vec4<f32>`, that's 16 bytes.
Storing 10 floats (cinert) or 6 floats (cdof) as flat `array<f32>`
would require manual stride calculations and non-coalesced access.

**Impact:** The spec's 10-float cinert and 6-float cdof formats would
require awkward indexing in WGSL (`cinert[body * 10 + k]`), and reads
would cross cache lines non-uniformly.

**Workaround:**
- Cinert: 12 floats (3 × vec4) per body. Wastes 8 bytes/body.
- cdof: 8 floats (2 × vec4) per DOF. Wastes 8 bytes/DOF.
- All vec3 model fields (body_pos, jnt_axis, etc.) padded to vec4.

The waste is negligible (e.g., 4 bodies × 8 bytes = 32 bytes for
hockey) and the clean `array<vec4<f32>>` access pattern is worth it.

## 4. No f32 atomics in WGSL

**Constraint:** WGSL `atomicAdd` only works on `atomic<u32>` and
`atomic<i32>`. There is no `atomicAdd` for `f32`.

**Impact:** Backward tree scans (CRBA, RNE, subtree COM) where multiple
children accumulate into a shared parent cannot use simple atomic adds.

**Session 1 workaround:** Plain f32 addition (non-atomic). Safe only
when each depth level fits in a single workgroup (true for hockey's flat
tree with ~4 bodies at depth 1).

**Session 2+ solution:** CAS (compare-and-swap) loop using
`atomicCompareExchangeWeak` on `atomic<u32>` with `bitcast<f32>→u32`:
```wgsl
fn atomic_add_f32(addr: ptr<storage, atomic<u32>>, val: f32) {
    var old = atomicLoad(addr);
    loop {
        let new_val = bitcast<u32>(bitcast<f32>(old) + val);
        let result = atomicCompareExchangeWeak(addr, old, new_val);
        if result.exchanged { break; }
        old = result.old_value;
    }
}
```
This is standard practice (CUDA does the same) but adds ~3-5 CAS retries
per contended write.

## 5. Uniform buffer dynamic offset alignment

**Constraint:** `WebGPU` requires uniform buffer offsets to be multiples
of `minUniformBufferOffsetAlignment` = 256 bytes.

**Impact:** The `FkParams` struct is 32 bytes, but each depth-level slot
in the pre-allocated uniform buffer must be 256 bytes apart. For a
7-level tree, that's 7 × 256 = 1,792 bytes (vs. 7 × 32 = 224 bytes
without alignment).

**Workaround:** Pre-allocate `(max_depth + 3) × 256` bytes for the
params buffer. Use dynamic offsets in `set_bind_group()`. The waste
(~1.5 KB) is negligible and avoids multiple `queue.submit()` calls.

## 6. Quaternion memory layout mismatch

**Constraint:** nalgebra stores quaternions in memory as `[x, y, z, w]`
(vector part first, scalar last). MuJoCo/CortenForge qpos stores
quaternions as `[w, x, y, z]` (scalar first).

**Impact:** Two different conventions coexist:
- GPU struct fields (body_quat, body_iquat, geom_quat): `(x,y,z,w)` —
  uploaded directly from nalgebra's memory layout.
- qpos buffer: `(w,x,y,z)` — uploaded directly from CPU qpos.

**Workaround:** The FK shader swizzles when reading quaternions from
qpos: `vec4(qpos[adr+4], qpos[adr+5], qpos[adr+6], qpos[adr+3])`.
All internal GPU quaternion operations use `(x,y,z,w)` consistently.

## 7. wgpu bind group count limit

**Constraint:** `WebGPU` default `maxBindGroups = 4`.

**Impact:** All shader bindings must fit within 4 bind groups. The FK
pipeline uses exactly 4:
- Group 0: params (uniform, dynamic offset)
- Group 1: static model (2 storage)
- Group 2: per-env state (10 storage)
- Group 3: geom model + output (3 storage)

**Future risk:** Later shaders may need additional binding groups. If
4 is insufficient, options include:
- Packing more data into fewer buffers within existing groups
- Using push constants for small uniform data
- Restructuring bind groups to share layouts across shaders

## 8. No storage-pointer function parameters in WGSL

**Constraint:** WGSL does not allow passing `ptr<storage, ...>` as a
function argument. Naga (wgpu's shader validator) rejects it:

```
Argument 'p' at index 0 is a pointer of space Storage, which can't
be passed into functions.
```

Only `ptr<function, ...>` and `ptr<private, ...>` can be passed as
function arguments. Workgroup and storage pointers cannot.

**Impact:** Generic helper functions like `atomic_add_f32(ptr, val)`
that operate on an arbitrary storage buffer element are impossible.
Each buffer that needs CAS-atomic f32 addition requires its own
dedicated function that accesses the global variable directly by name.

**Workaround:** Instead of:
```wgsl
fn atomic_add_f32(p: ptr<storage, atomic<u32>, read_write>, val: f32) { ... }
// FAILS: storage pointers can't be passed to functions
```

Write buffer-specific functions:
```wgsl
fn atomic_add_f32_crb(idx: u32, val: f32) {
    var old = atomicLoad(&body_crb[idx]);
    loop {
        let new_val = bitcast<u32>(bitcast<f32>(old) + val);
        let result = atomicCompareExchangeWeak(&body_crb[idx], old, new_val);
        if result.exchanged { break; }
        old = result.old_value;
    }
}
```

This is a fundamental WGSL limitation (not a naga bug). The WebGPU
spec restricts pointer parameters to function and private address
spaces. If a future WGSL revision lifts this restriction, the
workaround functions can be unified into a single generic helper.

**Discovered:** Session 2 (CRBA backward scan, 2026-03-24).

## 9. Backward tree scan: init and accumulation must be separate dispatches

**Constraint:** When a backward tree scan needs to both COMPUTE a
per-body value AND ACCUMULATE that value into parents, these two
operations must be separate dispatches. They cannot be combined in a
single depth-ordered pass.

**Impact:** If the per-body computation and parent accumulation are in
the same entry point (dispatched per depth level, leaves → root), the
`atomicStore` of body d's own value at depth level d **overwrites**
CAS-atomic additions that children at depth d+1 have already
accumulated into body d during the previous depth dispatch.

Example (RNE backward scan, 3-link pendulum):
```
Depth 3: body 3 computes cfrc[3], CAS-adds into cfrc[2]  ← cfrc[2] now has child contribution
Depth 2: body 2 computes cfrc[2] via atomicStore          ← OVERWRITES child's CAS addition!
          body 2 CAS-adds (only own cfrc) into cfrc[1]    ← missing body 3's contribution
```

**Workaround:** Split into two dispatches:
1. `rne_cfrc_init` — per-body parallel (no depth ordering), computes and
   `atomicStore`s each body's own cfrc. Buffer is pre-zeroed.
2. `rne_backward` — depth-ordered (leaves → root), only CAS-atomic-adds
   into parent. Reads (not writes) own cfrc.

**Why CRBA didn't hit this:** CRBA already separates these — `crba_init`
writes all per-body crb values in a single parallel dispatch, then
`crba_backward` only does CAS accumulation. The separation was for a
different reason (cinert format conversion), but it accidentally avoided
this bug.

**General rule:** Any backward tree scan that writes per-body values
AND accumulates into parents must use the init+accumulate split pattern.
Future backward scans (e.g., constraint force propagation) should follow
this pattern from the start.

**Discovered:** Session 3 (RNE backward scan, 2026-03-24). Caused ~4%
error in pendulum qfrc_bias due to lost child contributions.

## 10. WGSL reserved keywords include common variable names

**Constraint:** WGSL reserves many common English words as keywords,
including `meta`, `override`, `enable`, `diagnostic`, and others. Using
these as variable names, function parameters, or struct field names
causes a parse error:

```
Shader parsing error: name `meta` is a reserved keyword
```

**Impact:** Natural parameter names like `meta: SdfMeta` (for grid
metadata) fail at shader compilation. This is caught at runtime (when
`create_shader_module` is called), not at Rust compile time.

**Workaround:** Use abbreviated or prefixed names for WGSL variables.
Session 4 renamed `meta` → `gm` (grid metadata) in narrowphase shaders.
Other risky names to avoid: `override`, `enable`, `requires`, `alias`,
`const_assert`, `diagnostic`.

**General rule:** When writing WGSL, avoid single common English words
as identifiers. Prefer descriptive prefixes (`src_gm`, `dst_gm`) or
abbreviations (`gm`, `np`) over plain words (`meta`, `data`, `info`).

**Discovered:** Session 4 (collision pipeline, 2026-03-24). Caused
shader compilation failure for `sdf_sdf_narrow.wgsl` and
`sdf_plane_narrow.wgsl` until parameter names were changed.

## 11. Workgroup shared memory budget: 16 KB minimum

**Constraint:** WebGPU guarantees `maxComputeWorkgroupStorageSize ≥ 16,384`
bytes (16 KB). Most Metal/Vulkan GPUs offer 32–64 KB, but portable code
must target the 16 KB floor.

**Impact:** `newton_solve.wgsl` needs a dense Hessian in shared memory
(`nv × nv × 4` bytes) plus working vectors and a reduction buffer.
At `MAX_NV = 60`, the layout is:

| Array | Size |
|-------|------|
| `H_atomic` (60 × 60 × 4) | 14,400 bytes |
| `qacc_sh` (60 × 4) | 240 bytes |
| `qacc_sm_sh` (60 × 4) | 240 bytes |
| `grad_sh` (60 × 4) | 240 bytes |
| `search_sh` (60 × 4) | 240 bytes |
| `reduction_sh` (256 × 4) | 1,024 bytes |
| **Total** | **16,384 bytes** |

This is exactly the 16 KB minimum — zero headroom. Any additional
shared memory (e.g., Ma, qfrc_smooth vectors) must be read from global
memory on-the-fly instead.

**Workaround:** Vectors `Ma` and `qfrc_smooth` are computed from global
memory reads in the gradient computation phase rather than cached in
shared memory. For `nv ≤ 13` (hockey) this is negligible; for `nv = 60`
it adds ~3,600 global memory reads per Newton iteration.

**Future risk:** If `MAX_NV > 60` is needed, either:
- Use the sparse Hessian path (CSC storage, no dense H in shared memory)
- Tile the Hessian across multiple dispatches
- Require devices with 32+ KB shared memory

**Discovered:** Session 5 (constraint solve, 2026-03-24).

## 12. vec3 in storage buffer structs: alignment mismatch

**Constraint:** WGSL `vec3<f32>` has `align(16)` but `size(12)` in
storage buffers. When used in a struct, the compiler inserts 4 bytes of
padding after each `vec3`, making the struct layout unpredictable and
mismatched with Rust's `#[repr(C)]` layout.

**Impact:** `PipelineContact` (48 bytes, Rust side) contains three
`vec3`-like fields (`point`, `normal`, `friction`) interleaved with
scalar fields. Declaring this as a WGSL struct with `vec3` members
would produce incorrect field offsets.

**Workaround:** The assembly shader reads `PipelineContact` as a raw
`array<f32>` — 12 floats per contact (48 bytes / 4). Geom indices
(`geom1`, `geom2`) stored as `u32` are read via `bitcast<u32>(f32)`.
A local `ContactData` struct (non-storage, no alignment issues) is
populated from the raw values:
```wgsl
let base = ci * 12u;
c.point   = vec3(contact_buf[base + 0u], contact_buf[base + 1u], contact_buf[base + 2u]);
c.depth   = contact_buf[base + 3u];
c.normal  = vec3(contact_buf[base + 4u], contact_buf[base + 5u], contact_buf[base + 6u]);
c.geom1   = bitcast<u32>(contact_buf[base + 7u]);
```

**General rule:** Never use `vec3` in storage buffer struct definitions.
Use `vec4` with padding, or read as flat `array<f32>` with manual
indexing. Function-local structs (address space `function`) don't have
this restriction.

**Discovered:** Session 5 (constraint assembly, 2026-03-24).

## 13. atomicLoad requires read_write access

**Constraint:** WGSL requires `var<storage, read_write>` for any buffer
containing `atomic<T>`, even when the shader only performs `atomicLoad`
(read-only operation). A `var<storage, read>` binding with
`array<atomic<u32>>` fails validation.

**Impact:** `newton_solve.wgsl` and `map_forces.wgsl` only READ
`constraint_count` (via `atomicLoad`), but the bind group layout must
declare the buffer as `read_write`. Similarly, `assemble.wgsl` only
reads `contact_count` but must bind it as `read_write`.

**Workaround:** All atomic counter buffers (`contact_count`,
`constraint_count`) are bound as `storage { read_only: false }` in
every shader that accesses them, regardless of whether the shader
modifies them. The Rust `BindGroupLayoutEntry` uses `storage_rw()`
even for semantically read-only access.

**Risk:** A shader that only intends to read could accidentally write
to the counter. No mitigation other than code review.

**Discovered:** Session 5 (constraint solve, 2026-03-24). All three
constraint shaders needed this pattern.

## 14. workgroupBarrier requires uniform control flow across all invocations

**Constraint:** All invocations in a workgroup MUST execute the same
`workgroupBarrier()` calls. If any invocation skips a barrier (e.g.,
due to an `if` branch or early `return`), the behavior is undefined
(typically a hang or deadlock).

**Impact:** The Newton solver's line search evaluates cost at 4
candidate alphas using `eval_cost()`, which contains `workgroupBarrier`
in its parallel reduction. Even though only thread 0 uses the returned
cost value, ALL 256 threads must call `eval_cost()` for all 4 alphas.
This prevents early-exit optimization (e.g., "if alpha=1.0 works, skip
evaluating 0.5 and 0.25").

**Workaround:** Always evaluate all candidates unconditionally. Thread 0
picks the best alpha after all evaluations complete. The wasted work is
minimal (~260 flops/thread per extra evaluation for hockey).

**General rule:** Functions containing `workgroupBarrier()` must be
called from uniform control flow. Never put barrier-containing calls
inside `if (tid == 0u)` or other thread-divergent branches. Structure
loops so all threads break at the same iteration (use a shared
convergence flag checked after a barrier, not before).

**Discovered:** Session 5 (Newton solver line search, 2026-03-24).
