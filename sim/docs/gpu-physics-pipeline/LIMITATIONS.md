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
