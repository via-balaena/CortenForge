# Session 4: Collision Pipeline (AABB + Narrowphase)

**Branch:** `feature/vr-hockey`
**Parent spec:** `sim/docs/GPU_PHYSICS_PIPELINE_SPEC.md` (§8.8, §8.9, §8.10, §8.11, §6.2–6.4, §14 Session 4)
**Prereq:** Sessions 1–3 complete — FK (geom_xpos, geom_xmat), CRBA (M), velocity FK (cvel), RNE (qfrc_bias), smooth (qacc_smooth), integration
**Date:** 2026-03-24

## 1. Scope

Build the GPU collision pipeline: per-geom AABB computation, SDF-SDF
narrowphase (adapting the existing `trace_surface.wgsl` for pipeline
integration), and SDF-plane narrowphase. After this session, the GPU
pipeline produces contacts in a GPU buffer that Session 5's constraint
assembly can consume directly — no CPU readback in the inner loop.

### In scope

- `aabb.wgsl` — per-geom world-frame AABB from FK poses
- `sdf_sdf_narrow.wgsl` — adapted from `trace_surface.wgsl`, writes to
  pipeline's 48-byte `PipelineContact` buffer with geom indices + friction
- `sdf_plane_narrow.wgsl` — SDF surface cells tested against plane equation
- Extended `GeomModelGpu` — add type, contype, conaffinity, size, friction,
  sdf_meta_idx (48 → 96 bytes)
- Unified SDF grid buffer — all grids concatenated, per-shape metadata
- `GpuCollisionPipeline` — pre-computed pair plan, AABB guard, dispatch
- New state buffers: `geom_aabb`, `contact_buffer`, `contact_count`
- Validation tests T19–T23

### Out of scope

- `broadphase.wgsl` (spatial hash) — see §2 deviation
- `assemble.wgsl` (constraint assembly) — Session 5
- Non-SDF narrowphase (sphere, box, capsule, mesh, GJK/EPA) — CPU only
- Contact deduplication on GPU — accept duplicates, Session 5 caps contacts
- Batch environments (`n_env > 1`)
- Heightfield collision on GPU

### Deviation: No broadphase shader

The master spec (§8.9) calls for a 3-pass spatial hash broadphase. This
session replaces it with a **pre-computed pair plan** for these reasons:

1. **Hockey has 4 collision-active geoms** → C(4,2) = 6 possible pairs.
   A spatial hash (prefix scan, hash table, scatter) is massive overhead
   for 6 AABB overlap tests.
2. **Untestable code violates "A-grade or it doesn't ship."** With 4 geoms,
   we cannot validate that a spatial hash correctly handles hash collisions,
   cell boundary cases, or memory contention — the scene is too small.
3. **No CPU readback needed.** The pre-computed plan dispatches all
   narrowphase shaders at model-upload time. Each includes an AABB overlap
   guard that checks `geom_aabb` — if no overlap, all threads return.
4. **Sufficient for <100 geoms.** For 50 geoms, the plan would produce
   C(50,2) = 1,225 narrowphase dispatches × ~5μs = ~6ms — within budget.
   Spatial hash becomes worthwhile at 1,000+ geoms.

**Future upgrade path:** When scenes grow to 1,000+ geoms, add a
`broadphase.wgsl` spatial hash that writes to `pair_buffer`. The
narrowphase then reads `pair_buffer` instead of using a pre-computed plan.
The AABB shader and narrowphase shaders from this session are reused as-is.

## 2. Architecture

### 2.1 Pre-computed pair plan

At model upload (`GpuCollisionPipeline::new()`):

1. Enumerate all geom pairs `(i, j)` where `i < j`
2. Filter by affinity: `geom_contype[i] & geom_conaffinity[j] != 0` OR
   `geom_contype[j] & geom_conaffinity[i] != 0`
3. Filter by same-body exclusion: `geom_body[i] != geom_body[j]`
4. Classify each surviving pair by narrowphase type:
   - `(Sdf, Sdf)` → SDF-SDF (2 symmetric dispatches per pair)
   - `(Sdf, Plane)` or `(Plane, Sdf)` → SDF-plane (1 dispatch per pair)
   - All other combinations → skip (CPU-only narrowphase types)
5. For each pair, create bind groups referencing the correct SDF grid
   regions and store dispatch metadata (grid dimensions, combined friction)

At runtime (per substep):
1. Dispatch `aabb.wgsl` — per-geom parallel
2. For each pre-computed pair, dispatch its narrowphase shader:
   - Shader reads `geom_aabb[a]` and `geom_aabb[b]` as first operation
   - If no AABB overlap → all threads return (barrier-free early exit)
   - If overlap → run trace, emit contacts to shared `contact_buffer`

### 2.2 Hockey pair plan

For the hockey scene (1 plane + 3 SDF, visual meshes excluded by affinity):

| Pair | Type | Dispatches |
|------|------|-----------|
| puck ↔ ground | SDF-plane | 1 |
| stick ↔ ground | SDF-plane | 1 |
| goal ↔ ground | SDF-plane | 1 |
| puck ↔ stick | SDF-SDF | 2 (symmetric A→B, B→A) |
| puck ↔ goal | SDF-SDF | 2 |
| stick ↔ goal | SDF-SDF | 2 |
| **Total** | | **9 narrowphase dispatches** |

Plus 1 AABB dispatch = 10 dispatches total. At ~5μs each = ~50μs.

### 2.3 Unified SDF grid buffer

All SDF grids are concatenated into one `sdf_values: array<f32>` storage
buffer. A per-shape metadata buffer `sdf_metas: array<SdfMeta>` stores
dimensions, cell_size, origin, and the offset into `sdf_values` where
each grid starts.

```
sdf_values: [ grid_0 values... | grid_1 values... | grid_2 values... ]
sdf_metas:  [ {w,h,d, cell_size, origin, offset=0}, {w,h,d, ..., offset=W0*H0*D0}, ... ]
```

Each narrowphase shader receives the `sdf_meta_idx` of its geoms via a
per-pair uniform. The shader reads `sdf_metas[idx]` to get grid dimensions
and offset, then indexes into `sdf_values[offset + grid_idx(x,y,z,w,h)]`.

This replaces per-grid buffer bindings with dynamic indexing into one large
buffer — solving the storage buffer limit problem for arbitrary geom counts.

## 3. File structure

```
sim/L0/gpu/src/
├── pipeline/
│   ├── mod.rs                    # Add: pub mod collision;
│   ├── types.rs                  # Add: PipelineContact, SdfMetaGpu, NarrowphaseParams
│   │                             # Modify: GeomModelGpu (48 → 96 bytes)
│   ├── model_buffers.rs          # Add: sdf_values, sdf_metas buffers + extended GeomModelGpu upload
│   ├── state_buffers.rs          # Add: geom_aabb, contact_buffer, contact_count
│   ├── collision.rs              # NEW: GpuCollisionPipeline
│   └── tests.rs                  # Add: T19–T23
├── shaders/
│   ├── fk.wgsl                   # Update GeomModel struct (48 → 96 bytes)
│   ├── aabb.wgsl                 # NEW: ~80 lines
│   ├── sdf_sdf_narrow.wgsl       # NEW: ~300 lines (adapted from trace_surface.wgsl)
│   └── sdf_plane_narrow.wgsl     # NEW: ~150 lines
```

## 4. GPU-side types (`types.rs` additions)

### 4.1 Extended `GeomModelGpu` (48 → 96 bytes)

```rust
#[repr(C)]
#[derive(Debug, Copy, Clone, Pod, Zeroable)]
pub struct GeomModelGpu {
    // Row 0 (16 bytes): identity
    pub body_id: u32,
    pub geom_type: u32,       // GeomType enum as u32
    pub contype: u32,         // Collision type bitmask
    pub conaffinity: u32,     // Collision affinity bitmask
    // Row 1 (16 bytes): position offset in body frame
    pub pos: [f32; 4],        // (x, y, z, 0)
    // Row 2 (16 bytes): orientation in body frame
    pub quat: [f32; 4],       // (x, y, z, w) — nalgebra layout
    // Row 3 (16 bytes): dimensions
    pub size: [f32; 4],       // xyz = type-specific size, w = rbound
    // Row 4 (16 bytes): friction
    pub friction: [f32; 4],   // x = slide, y = torsion, z = roll, w = 0
    // Row 5 (16 bytes): SDF reference
    pub sdf_meta_idx: u32,    // Index into sdf_metas (0xFFFFFFFF = not SDF)
    pub condim: u32,          // Contact dimensionality (1, 3, 4, or 6)
    pub _pad: [u32; 2],
}
```

96 bytes, 16-byte aligned. The stride change requires updating the WGSL
`GeomModel` struct definition in `fk.wgsl` (which only reads `body_id`,
`pos`, `quat` — the new fields are unused but the struct size must match
for correct array indexing).

WGSL struct:
```wgsl
struct GeomModel {
    body_id: u32,
    geom_type: u32,
    contype: u32,
    conaffinity: u32,
    pos: vec4<f32>,
    quat: vec4<f32>,
    size: vec4<f32>,
    friction: vec4<f32>,
    sdf_meta_idx: u32,
    condim: u32,
    _pad0: u32,
    _pad1: u32,
};
```

### 4.2 `SdfMetaGpu` (32 bytes)

```rust
#[repr(C)]
#[derive(Debug, Copy, Clone, Pod, Zeroable)]
pub struct SdfMetaGpu {
    pub width: u32,
    pub height: u32,
    pub depth: u32,
    pub cell_size: f32,
    pub origin: [f32; 3],     // Grid origin (world-frame of local SDF)
    pub values_offset: u32,   // Offset into unified sdf_values buffer
}
```

WGSL struct:
```wgsl
struct SdfMeta {
    width: u32,
    height: u32,
    depth: u32,
    cell_size: f32,
    origin: vec3<f32>,
    values_offset: u32,
};
```

### 4.3 `PipelineContact` (48 bytes)

```rust
#[repr(C)]
#[derive(Debug, Copy, Clone, Pod, Zeroable)]
pub struct PipelineContact {
    pub point: [f32; 3],      // World-space contact position
    pub depth: f32,           // Penetration depth (≥ 0)
    pub normal: [f32; 3],     // World-space contact normal (geom1 → geom2)
    pub geom1: u32,           // Geom index A
    pub friction: [f32; 3],   // Combined friction [slide, torsion, roll]
    pub geom2: u32,           // Geom index B
}
```

WGSL struct:
```wgsl
struct PipelineContact {
    point: vec3<f32>,
    depth: f32,
    normal: vec3<f32>,
    geom1: u32,
    friction: vec3<f32>,
    geom2: u32,
};
```

### 4.4 `NarrowphaseParams` (uniform, per-pair)

```rust
#[repr(C)]
#[derive(Debug, Copy, Clone, Pod, Zeroable)]
pub struct NarrowphaseParams {
    // Pose transforms (from FK-computed geom_xpos/geom_xmat)
    pub src_pose: [f32; 16],     // Source geom: local → world (mat4x4)
    pub dst_pose: [f32; 16],     // Dest geom: local → world
    pub dst_pose_inv: [f32; 16], // Dest geom: world → local
    // Pair metadata
    pub surface_threshold: f32,  // src_cell_size * 2.0
    pub contact_margin: f32,     // Model contact margin
    pub flip_normal: u32,        // 0 = negate, 1 = keep
    pub geom1: u32,              // Source geom index
    pub geom2: u32,              // Dest geom index
    pub src_sdf_meta_idx: u32,   // Index into sdf_metas for source
    pub dst_sdf_meta_idx: u32,   // Index into sdf_metas for dest (unused for plane)
    pub _pad: u32,
    // Combined friction
    pub friction: [f32; 4],      // [slide, torsion, roll, 0]
}
```

For SDF-plane, `dst_pose`/`dst_pose_inv` encode the plane equation
(plane normal = Z-axis of `dst_pose`, plane offset = dot(normal, pos)).
`dst_sdf_meta_idx` is unused.

**Note:** This is a large uniform (224 bytes). Written per-pair dispatch
via `queue.write_buffer`. With 9 pairs × 224 bytes = ~2KB per substep —
negligible bandwidth.

**Alternative considered:** Pack pair metadata into a storage buffer and
index by `pair_id`. Rejected because each pair dispatch already needs a
separate bind group (for the AABB guard check), so a per-pair uniform is
simpler.

## 5. Static buffer additions (`model_buffers.rs`)

### 5.1 New fields on `GpuModelBuffers`

```rust
pub struct GpuModelBuffers {
    // ... existing fields ...

    /// Concatenated SDF grid values: f32 array covering all shapes.
    pub sdf_values: wgpu::Buffer,
    /// Per-shape metadata: array<SdfMetaGpu>.
    pub sdf_metas: wgpu::Buffer,
    /// Number of SDF shapes (for indexing bounds).
    pub nshape: u32,
}
```

### 5.2 Upload logic

```rust
// Unified SDF grid upload
let mut all_values: Vec<f32> = Vec::new();
let mut metas: Vec<SdfMetaGpu> = Vec::new();

for shape in &model.shape_data {
    let grid = shape.sdf_grid();
    let offset = all_values.len() as u32;
    all_values.extend(grid.values().iter().map(|&v| v as f32));
    metas.push(SdfMetaGpu {
        width: grid.width() as u32,
        height: grid.height() as u32,
        depth: grid.depth() as u32,
        cell_size: grid.cell_size() as f32,
        origin: [
            grid.origin().x as f32,
            grid.origin().y as f32,
            grid.origin().z as f32,
        ],
        values_offset: offset,
    });
}
```

### 5.3 Extended `GeomModelGpu` upload

```rust
let geoms_cpu: Vec<GeomModelGpu> = (0..ngeom)
    .map(|g| {
        let gpos = &model.geom_pos[g];
        let gqc = model.geom_quat[g].as_ref().coords;
        let gsize = &model.geom_size[g];
        let gfric = &model.geom_friction[g];
        let sdf_meta_idx = model.geom_shape[g]
            .map_or(0xFFFF_FFFFu32, |s| s as u32);

        GeomModelGpu {
            body_id: model.geom_body[g] as u32,
            geom_type: geom_type_to_gpu(model.geom_type[g]),
            contype: model.geom_contype[g],
            conaffinity: model.geom_conaffinity[g],
            pos: [gpos.x as f32, gpos.y as f32, gpos.z as f32, 0.0],
            quat: [gqc.x as f32, gqc.y as f32, gqc.z as f32, gqc.w as f32],
            size: [
                gsize.x as f32, gsize.y as f32, gsize.z as f32,
                model.geom_rbound[g] as f32,
            ],
            friction: [gfric.x as f32, gfric.y as f32, gfric.z as f32, 0.0],
            sdf_meta_idx,
            condim: model.geom_condim[g] as u32,
            _pad: [0; 2],
        }
    })
    .collect();
```

### 5.4 GeomType GPU mapping

```rust
pub const GPU_GEOM_PLANE: u32 = 0;
pub const GPU_GEOM_SPHERE: u32 = 1;
pub const GPU_GEOM_CAPSULE: u32 = 2;
pub const GPU_GEOM_CYLINDER: u32 = 3;
pub const GPU_GEOM_BOX: u32 = 4;
pub const GPU_GEOM_ELLIPSOID: u32 = 5;
pub const GPU_GEOM_MESH: u32 = 6;
pub const GPU_GEOM_HFIELD: u32 = 7;
pub const GPU_GEOM_SDF: u32 = 8;
```

## 6. State buffer additions (`state_buffers.rs`)

```rust
pub struct GpuStateBuffers {
    // ... existing fields ...

    // Collision state (Session 4)
    /// Per-geom world-frame AABB: 2×vec4 per geom [min(xyz,0), max(xyz,0)].
    pub geom_aabb: wgpu::Buffer,
    /// Pipeline contacts: PipelineContact × max_contacts.
    pub contact_buffer: wgpu::Buffer,
    /// Active contact count: atomic<u32>.
    pub contact_count: wgpu::Buffer,
}
```

### 6.1 Allocation sizes

```rust
// geom_aabb: 2×vec4 per geom = 32 bytes/geom
let geom_aabb = alloc(ctx, "geom_aabb", ngeom * 32, usage_inout);

// contact_buffer: 48 bytes per contact, pre-allocated max
const MAX_PIPELINE_CONTACTS: u32 = 32_768;
let contact_buffer = alloc(
    ctx, "contact_buffer",
    u64::from(MAX_PIPELINE_CONTACTS) * 48,
    usage_inout,
);

// contact_count: single atomic u32
let contact_count = alloc(ctx, "contact_count", 4, usage_inout);
```

## 7. Shader specifications

### 7.1 `aabb.wgsl` — Per-geom AABB computation (~80 lines)

**Pattern:** Per-geom parallel map. One thread per geom.
**Input:** `geom_xpos`, `geom_xmat` (from FK), `GeomModel.size`, `GeomModel.geom_type`
**Output:** `geom_aabb` — 2×vec4 per geom [min, max]

**Bindings (5 total, 3 bind groups):**
- Group 0: `PhysicsParams` uniform (non-dynamic — single dispatch)
- Group 1: `geoms` (storage, read) — GeomModel array
- Group 2: `geom_xpos` (read), `geom_xmat` (read), `geom_aabb` (write)

**Algorithm:** Matches CPU `aabb_from_geom_aabb()` in `forward/position.rs`:
```wgsl
@compute @workgroup_size(64)
fn compute_aabb(@builtin(global_invocation_id) gid: vec3<u32>) {
    let g = gid.x;
    if g >= params.ngeom { return; }

    let geom = geoms[g];
    let pos = geom_xpos[g].xyz;
    // geom_xmat: 3×vec4 (column-major rotation matrix)
    let col0 = geom_xmat[g * 3u + 0u].xyz;
    let col1 = geom_xmat[g * 3u + 1u].xyz;
    let col2 = geom_xmat[g * 3u + 2u].xyz;

    // Local-frame AABB half-extents from geom type + size
    let half = local_half_extents(geom.geom_type, geom.size);

    // Rotate OBB to world → axis-aligned envelope
    let world_half = vec3(
        abs(col0.x) * half.x + abs(col1.x) * half.y + abs(col2.x) * half.z,
        abs(col0.y) * half.x + abs(col1.y) * half.y + abs(col2.y) * half.z,
        abs(col0.z) * half.x + abs(col1.z) * half.y + abs(col2.z) * half.z,
    );

    geom_aabb[g * 2u + 0u] = vec4(pos - world_half, 0.0);
    geom_aabb[g * 2u + 1u] = vec4(pos + world_half, 0.0);
}
```

**`local_half_extents` per geom type:**
```wgsl
fn local_half_extents(gtype: u32, size: vec4<f32>) -> vec3<f32> {
    switch gtype {
        case 0u: { return vec3(1e6); }                        // Plane (infinite)
        case 1u: { return vec3(size.x); }                     // Sphere (r)
        case 2u: { return vec3(size.x, size.x, size.x + size.y); } // Capsule (r, r, r+h)
        case 3u: { return vec3(size.x, size.x, size.y); }    // Cylinder (r, r, h)
        case 4u: { return size.xyz; }                         // Box (sx, sy, sz)
        case 5u: { return size.xyz; }                         // Ellipsoid (rx, ry, rz)
        case 8u: { return size.xyz; }                         // SDF (uses geom_size as AABB)
        default: { return size.xyz; }                         // Mesh, Hfield — conservative
    }
}
```

**Note on margin:** AABB is NOT inflated by contact margin here. The AABB
guard in narrowphase shaders handles the margin check (cheaper to inflate
during the 6-pair guard check than per-geom in the AABB pass).

### 7.2 `sdf_sdf_narrow.wgsl` — SDF-SDF narrowphase (~300 lines)

**Adapted from:** `trace_surface.wgsl` (267 lines, fully tested)

**Changes from standalone `trace_surface.wgsl`:**
1. **AABB guard** — reads `geom_aabb` for the pair, returns if no overlap
2. **Unified SDF buffer** — reads from `sdf_values[offset + idx]` instead
   of separate `src_grid`/`dst_grid` buffers
3. **Extended contact output** — writes 48-byte `PipelineContact` with
   geom1, geom2, combined friction (from params)
4. **Geom indices from params** — `params.geom1`, `params.geom2`
5. **Grid metadata from buffer** — reads `sdf_metas[params.src_sdf_meta_idx]`
   instead of uniform `src_meta`

**Bindings (7 total, 4 bind groups):**
- Group 0: `NarrowphaseParams` uniform
- Group 1: `sdf_metas` (storage, read), `sdf_values` (storage, read)
- Group 2: `geom_aabb` (storage, read)
- Group 3: `contact_buffer` (storage, read_write), `contact_count` (atomic)

**Workgroup:** `@workgroup_size(8, 8, 4)` (same as trace_surface.wgsl)

**Dispatch grid:** `(src_width.div_ceil(8), src_height.div_ceil(8), src_depth.div_ceil(4))`

**Algorithm:** Identical to trace_surface.wgsl — per-voxel surface
reconstruction, world transform, destination SDF query, contact emission.
Only the data access pattern changes (unified buffer + metadata indexing).

**AABB guard (first operation in shader):**
```wgsl
// Check AABB overlap for this pair — skip entire dispatch if no overlap
let aabb_a_min = geom_aabb[params.geom1 * 2u + 0u].xyz;
let aabb_a_max = geom_aabb[params.geom1 * 2u + 1u].xyz;
let aabb_b_min = geom_aabb[params.geom2 * 2u + 0u].xyz;
let aabb_b_max = geom_aabb[params.geom2 * 2u + 1u].xyz;

if aabb_a_min.x > aabb_b_max.x || aabb_b_min.x > aabb_a_max.x
|| aabb_a_min.y > aabb_b_max.y || aabb_b_min.y > aabb_a_max.y
|| aabb_a_min.z > aabb_b_max.z || aabb_b_min.z > aabb_a_max.z {
    return;
}
```

**Contact emission:**
```wgsl
let ci = atomicAdd(&contact_count, 1u);
if ci < arrayLength(&contact_buffer) {
    contact_buffer[ci] = PipelineContact(
        world,                                 // point
        penetration,                           // depth
        normal_world,                          // normal
        params.geom1,                          // geom1
        params.friction.xyz,                   // friction
        params.geom2,                          // geom2
    );
}
```

**Symmetric dispatch:** For each SDF-SDF pair, two dispatches are encoded
(A→B with flip_normal=0, B→A with flip_normal=1), matching the existing
trace_surface.wgsl pattern. Both write to the same contact_buffer with
the same geom1/geom2 indices (A is always geom1 regardless of dispatch
direction) to maintain consistent normal convention.

### 7.3 `sdf_plane_narrow.wgsl` — SDF-plane narrowphase (~150 lines)

**New shader.** Traces SDF surface cells against a plane equation.

**Reference:** CPU `sim/L0/core/src/collision/sdf_collide.rs` →
`compute_shape_plane_contact()`, which samples SDF grid cells and tests
distance to the plane.

**Bindings (7 total, 4 bind groups):** Same layout as `sdf_sdf_narrow.wgsl`.

**Workgroup:** `@workgroup_size(8, 8, 4)`

**Dispatch grid:** `(sdf_width.div_ceil(8), sdf_height.div_ceil(8), sdf_depth.div_ceil(4))`

**Algorithm (per grid cell):**
```wgsl
@compute @workgroup_size(8, 8, 4)
fn sdf_plane_narrow(@builtin(global_invocation_id) gid: vec3<u32>) {
    // 0. AABB guard (same as sdf_sdf_narrow)
    // ...

    // 1. Bounds check
    let x = gid.x; let y = gid.y; let z = gid.z;
    let meta = sdf_metas[params.src_sdf_meta_idx];
    if x >= meta.width || y >= meta.height || z >= meta.depth { return; }

    // 2. Read SDF value
    let idx = meta.values_offset + z * meta.width * meta.height + y * meta.width + x;
    let sdf_value = sdf_values[idx];

    // 3. Surface filter (skip far-from-surface cells)
    let threshold = meta.cell_size * 2.0;
    if sdf_value > threshold { return; }

    // 4. Compute local position
    let local = meta.origin + vec3(f32(x), f32(y), f32(z)) * meta.cell_size;

    // 5. Surface gradient (centered differences in SDF grid)
    let grad = sdf_gradient(local, meta);
    if dot(grad, grad) < 0.5 { return; }

    // 6. Surface reconstruction
    let surface_local = local - grad * sdf_value;

    // 7. Transform to world
    let world = (params.src_pose * vec4(surface_local, 1.0)).xyz;

    // 8. Plane distance test
    // Plane normal = dst_pose column 2 (Z-axis of plane geom)
    let plane_normal = params.dst_pose[2].xyz;  // column-major: column 2
    let plane_pos = params.dst_pose[3].xyz;     // column-major: column 3
    let plane_offset = dot(plane_normal, plane_pos);
    let dist = dot(plane_normal, world) - plane_offset;

    // 9. Contact test
    if dist >= params.contact_margin { return; }
    let penetration = max(-dist, 0.0);

    // 10. Emit contact
    // Normal points UP (away from plane), matching CPU convention
    let ci = atomicAdd(&contact_count, 1u);
    if ci < arrayLength(&contact_buffer) {
        contact_buffer[ci] = PipelineContact(
            world,
            penetration,
            plane_normal,           // Plane normal as contact normal
            params.geom1,
            params.friction.xyz,
            params.geom2,
        );
    }
}
```

**Plane equation extraction:** The plane geom's pose matrix encodes the
plane: column 2 = plane normal, column 3 = a point on the plane. This
matches the CPU convention where the plane normal is `mat.column(2)`.

**SDF gradient function:** Duplicated from trace_surface.wgsl's gradient
logic but adapted to read from the unified `sdf_values` buffer via
metadata offsets. Uses centered differences with half-cell-size epsilon.

## 8. `GpuCollisionPipeline` (`collision.rs`)

### 8.1 Pair descriptor

```rust
/// Pre-computed narrowphase dispatch descriptor.
struct NarrowphasePair {
    geom_a: usize,
    geom_b: usize,
    pair_type: PairType,
    /// Combined friction: element-wise max of both geoms' friction.
    friction: [f32; 3],
    /// Contact margin: max of both geoms' margin.
    margin: f32,
}

enum PairType {
    /// Two SDF geoms — dispatch sdf_sdf_narrow.wgsl twice (symmetric).
    SdfSdf {
        sdf_meta_a: u32,
        sdf_meta_b: u32,
    },
    /// SDF geom vs plane geom — dispatch sdf_plane_narrow.wgsl once.
    SdfPlane {
        sdf_geom: usize,    // Which geom is the SDF
        plane_geom: usize,  // Which geom is the plane
        sdf_meta_idx: u32,
    },
}
```

### 8.2 Pipeline structure

```rust
pub struct GpuCollisionPipeline {
    // Compute pipelines
    aabb_pipeline: wgpu::ComputePipeline,
    sdf_sdf_pipeline: wgpu::ComputePipeline,
    sdf_plane_pipeline: wgpu::ComputePipeline,

    // Bind group layouts
    aabb_layout: wgpu::BindGroupLayout,
    narrow_layout: wgpu::BindGroupLayout,

    // Pre-computed pair plan
    pairs: Vec<NarrowphasePair>,

    // Per-pair uniform buffer (written per dispatch)
    narrow_params_buf: wgpu::Buffer,
}
```

### 8.3 Dispatch method

```rust
impl GpuCollisionPipeline {
    /// Encode collision dispatches into an existing command encoder.
    ///
    /// Must be called AFTER FK (which writes geom_xpos, geom_xmat).
    /// Writes: geom_aabb, contact_buffer, contact_count.
    pub fn encode(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        ctx: &GpuContext,
        model_bufs: &GpuModelBuffers,
        state_bufs: &GpuStateBuffers,
    ) {
        // 1. Reset contact counter
        encoder.clear_buffer(&state_bufs.contact_count, 0, Some(4));

        // 2. AABB computation (one dispatch, all geoms)
        {
            let mut pass = encoder.begin_compute_pass(&desc("aabb"));
            pass.set_pipeline(&self.aabb_pipeline);
            pass.set_bind_group(0, &self.aabb_params_bg, &[]);
            pass.set_bind_group(1, &self.aabb_model_bg, &[]);
            pass.set_bind_group(2, &self.aabb_state_bg, &[]);
            pass.dispatch_workgroups(
                model_bufs.ngeom.div_ceil(64), 1, 1,
            );
        }

        // 3. Narrowphase dispatches (one per pre-computed pair)
        for pair in &self.pairs {
            // Write pair-specific params to uniform buffer
            let params = self.build_narrow_params(pair, ctx, state_bufs);
            ctx.queue.write_buffer(
                &self.narrow_params_buf, 0,
                bytemuck::bytes_of(&params),
            );

            match &pair.pair_type {
                PairType::SdfSdf { sdf_meta_a, sdf_meta_b } => {
                    // A→B dispatch
                    self.encode_sdf_sdf_pass(
                        encoder, pair, *sdf_meta_a, *sdf_meta_b,
                        0, // flip_normal = 0 (negate)
                    );
                    // B→A dispatch (symmetric)
                    self.encode_sdf_sdf_pass(
                        encoder, pair, *sdf_meta_b, *sdf_meta_a,
                        1, // flip_normal = 1 (keep)
                    );
                }
                PairType::SdfPlane { sdf_meta_idx, .. } => {
                    self.encode_sdf_plane_pass(
                        encoder, pair, *sdf_meta_idx,
                    );
                }
            }
        }
    }
}
```

**Note on params writing:** `queue.write_buffer` before each narrowphase
dispatch means the params are uploaded between dispatches. This works
because wgpu serializes writes and dispatches within a single encoder.
The GPU sees the correct params for each dispatch.

**Alternative:** Pre-write all pair params into a large uniform buffer
with 256-byte aligned slots (same pattern as FK/RNE depth params). This
avoids per-dispatch CPU work. For 9 pairs × 256 bytes = 2.3KB — trivial.
Use dynamic offset to select the slot per dispatch. This is cleaner and
will be the implementation approach.

## 9. Pose extraction for narrowphase params

The narrowphase shaders need geom-pair pose transforms as mat4x4 matrices.
The FK pipeline computes `geom_xpos` (vec4) and `geom_xmat` (3×vec4
column-major rotation). The narrowphase params need full 4×4 pose and
inverse matrices.

**Option A: CPU readback of geom poses → compute mat4x4 on CPU**
- Violates "no readback in inner loop"

**Option B: Compute mat4x4 in the narrowphase shader from geom_xpos/xmat**
- The shader reads `geom_xpos[g]` and `geom_xmat[g*3 .. g*3+3]` directly
- Constructs mat4x4 inline: rotation columns from xmat, translation from xpos
- No params needed for poses — the shader reads FK output directly

**Option B is correct.** The narrowphase shader should read FK output
buffers directly, not receive pre-computed mat4x4 poses via params. This:
- Eliminates per-pair pose upload
- Keeps poses on GPU (no readback)
- Uses fresh FK data directly

**Updated NarrowphaseParams (simplified):**
```rust
pub struct NarrowphaseParams {
    // Pair metadata (no pose data — read from geom_xpos/xmat directly)
    pub geom1: u32,
    pub geom2: u32,
    pub src_sdf_meta_idx: u32,
    pub dst_sdf_meta_idx: u32,  // 0xFFFFFFFF for plane
    pub surface_threshold: f32,
    pub contact_margin: f32,
    pub flip_normal: u32,
    pub _pad0: u32,
    pub friction: [f32; 4],     // [slide, torsion, roll, 0]
}
```

48 bytes. Much cleaner. The shader constructs pose matrices from
`geom_xpos` and `geom_xmat` buffer reads.

**Shader pose construction:**
```wgsl
fn geom_pose_mat4(g: u32) -> mat4x4<f32> {
    let col0 = geom_xmat[g * 3u + 0u];
    let col1 = geom_xmat[g * 3u + 1u];
    let col2 = geom_xmat[g * 3u + 2u];
    let pos  = geom_xpos[g];
    return mat4x4(
        vec4(col0.xyz, 0.0),
        vec4(col1.xyz, 0.0),
        vec4(col2.xyz, 0.0),
        vec4(pos.xyz, 1.0),
    );
}

fn geom_pose_inv_mat4(g: u32) -> mat4x4<f32> {
    // For a rigid transform [R|t], inverse = [R^T | -R^T·t]
    let col0 = geom_xmat[g * 3u + 0u].xyz;
    let col1 = geom_xmat[g * 3u + 1u].xyz;
    let col2 = geom_xmat[g * 3u + 2u].xyz;
    let pos  = geom_xpos[g].xyz;
    // R^T rows = R columns
    let rt0 = vec3(col0.x, col1.x, col2.x);
    let rt1 = vec3(col0.y, col1.y, col2.y);
    let rt2 = vec3(col0.z, col1.z, col2.z);
    let neg_rt_t = -vec3(dot(rt0, pos), dot(rt1, pos), dot(rt2, pos));
    return mat4x4(
        vec4(rt0, 0.0),
        vec4(rt1, 0.0),
        vec4(rt2, 0.0),
        vec4(neg_rt_t, 1.0),
    );
}
```

## 10. Bindings summary

### 10.1 AABB shader (5 bindings, 3 groups)

| Group | Binding | Type | Buffer |
|-------|---------|------|--------|
| 0 | 0 | uniform | PhysicsParams |
| 1 | 0 | storage (read) | geoms |
| 2 | 0 | storage (read) | geom_xpos |
| 2 | 1 | storage (read) | geom_xmat |
| 2 | 2 | storage (rw) | geom_aabb |

### 10.2 SDF-SDF narrowphase (9 bindings, 4 groups)

| Group | Binding | Type | Buffer |
|-------|---------|------|--------|
| 0 | 0 | uniform | NarrowphaseParams |
| 1 | 0 | storage (read) | sdf_metas |
| 1 | 1 | storage (read) | sdf_values |
| 2 | 0 | storage (read) | geom_xpos |
| 2 | 1 | storage (read) | geom_xmat |
| 2 | 2 | storage (read) | geom_aabb |
| 3 | 0 | storage (rw) | contact_buffer |
| 3 | 1 | storage (rw) | contact_count |
| 3 | 2 | storage (read) | geoms |

9 storage + 1 uniform = 10 bindings. Under the 16-per-stage limit.

### 10.3 SDF-plane narrowphase (same layout as SDF-SDF)

Same bind group layout. The shader ignores `dst_sdf_meta_idx` and uses
the plane geom's pose from `geom_xpos`/`geom_xmat` to compute the plane
equation.

## 11. Contact deduplication

**Not implemented in Session 4.** The SDF-SDF symmetric dispatch produces
~2× contacts (A→B and B→A find overlapping surface points). The CPU path
deduplicates by spatial proximity after readback (`dedup_contacts()` in
`collision.rs`).

For the pipeline, dedup options:
1. **Accept duplicates.** Session 5's constraint assembly can cap contacts
   per pair (e.g., keep deepest N contacts per geom pair). The contact
   buffer has 32K capacity — more than enough.
2. **GPU dedup pass.** Sort contacts by (geom1, geom2, depth), then
   greedy spatial filter. Complex and not needed for hockey's contact counts.

**Decision:** Accept duplicates. Contact capping in Session 5.

## 12. Integration into command buffer (§9 update)

After RNE + smooth (existing Session 3 dispatches):

```rust
// Collision (Session 4)
collision_pipeline.encode(encoder, ctx, model_bufs, state_bufs);

// Gravity-only bridge (Session 3) — will be replaced by Newton solver (Session 5)
encoder.copy_buffer_to_buffer(&state_bufs.qacc_smooth, 0, &state_bufs.qacc, 0, nv * 4);

// Integration (Session 3)
integrate_pipeline.encode(encoder, ...);
```

The collision dispatches happen AFTER FK (which provides fresh geom poses)
and BEFORE constraint solve (Session 5, which consumes contacts).

## 13. Validation tests

### T19: AABB matches CPU

Build a model with mixed geom types (plane + sphere + SDF). Run FK, then
AABB shader. Readback `geom_aabb` and compare against CPU's
`aabb_from_geom_aabb()`. Tolerance: 1e-4 (f32 precision).

### T20: SDF-SDF contacts match standalone GPU tracer

Build two overlapping SDF spheres (same as existing `trace_contacts_match_cpu`
test in `collision.rs`). Run FK + collision pipeline. Readback
`contact_buffer` + `contact_count`. Compare against standalone
`GpuTracer::trace_contacts()`. Contact positions should match within
1 cell size. Contact count should be within 2× (pipeline has no dedup).

### T21: SDF-plane contacts

Build an SDF sphere above a ground plane (slightly intersecting). Run
FK + collision pipeline. Verify contacts exist with:
- Contact normal ≈ (0, 0, 1) (plane up)
- Contact depth > 0
- Contact position on the sphere's bottom hemisphere

Compare against CPU `compute_shape_plane_contact()`.

### T22: Hockey-like full collision

Build a 4-body scene (plane + puck SDF + stick SDF + goal SDF).
Place puck slightly below ground plane (SDF-plane contact expected).
Place stick overlapping puck (SDF-SDF contact expected).
Run FK + collision pipeline. Verify:
- SDF-plane contacts for puck
- SDF-SDF contacts for puck↔stick
- No contacts for well-separated pairs
- Correct geom indices in PipelineContact

### T23: Contact buffer format

Verify that readback `PipelineContact` has correct:
- `geom1`, `geom2` indices match the pair
- `friction` values match element-wise max of both geoms
- `depth` ≥ 0
- `normal` is unit length (within f32 tolerance)

## 14. Implementation order

1. **Types first:** Extend `GeomModelGpu`, add `SdfMetaGpu`,
   `PipelineContact`, `NarrowphaseParams` to `types.rs`
2. **Update fk.wgsl:** Change `GeomModel` struct (48 → 96 bytes)
3. **Model buffers:** Extend `GpuModelBuffers` — upload extended geom
   data + unified SDF buffer + metadata
4. **State buffers:** Add `geom_aabb`, `contact_buffer`, `contact_count`
5. **AABB shader:** Write `aabb.wgsl`, build pipeline, test (T19)
6. **SDF-SDF narrowphase:** Adapt `trace_surface.wgsl` → `sdf_sdf_narrow.wgsl`,
   build pipeline, test (T20)
7. **SDF-plane narrowphase:** Write `sdf_plane_narrow.wgsl`, build pipeline,
   test (T21)
8. **Collision pipeline orchestration:** `GpuCollisionPipeline` with
   pre-computed pair plan, `encode()` method
9. **Integration tests:** T22, T23
10. **Verify existing tests still pass** (FK, CRBA, RNE — T1–T18) after
    GeomModelGpu stride change

## 15. Risk assessment

| Risk | Mitigation |
|------|-----------|
| GeomModelGpu stride change breaks FK | Update fk.wgsl GeomModel struct. Run T1–T7. |
| Unified SDF buffer too large | Hockey: 3 grids × ~32³ × 4 bytes ≈ 400KB. Negligible. |
| Storage buffer limit (9 bindings per narrowphase) | Under 16-per-stage limit. Verified. |
| SDF-plane normal convention mismatch | Match CPU: normal = plane Z-axis (mat column 2). Test T21. |
| Duplicate contacts overwhelm buffer | 32K capacity × 48 bytes = 1.5MB. Hockey max: ~3000 raw contacts. |
| queue.write_buffer between dispatches | Use pre-allocated 256-byte slots with dynamic offset instead. |
