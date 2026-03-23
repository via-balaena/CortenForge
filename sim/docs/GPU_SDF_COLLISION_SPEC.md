# GPU SDF Collision Acceleration Spec

## 1. Problem

SDF-SDF collision detection is a triple-nested loop over every cell in a 3D
grid. For each cell near the surface, it performs trilinear interpolation,
surface reconstruction, pose transforms, and a cross-SDF distance query.
This runs sequentially on a single CPU core.

**Current hot path** (`trace_surface_into_other` in `sdf/operations.rs`):

```
for z in 0..depth:
  for y in 0..height:
    for x in 0..width:
      src_value = grid[z*W*H + y*W + x]       // lookup
      if src_value > threshold: skip            // one-sided surface filter
      gradient = forward_diff(grid, point)      // 4 trilinear samples
      gradient = normalize(gradient)            // unit vector
      surface_point = point - gradient * value  // reconstruction
      world = src_pose.transform(surface_point) // rotation + translation
      dst_local = dst_pose.inverse(world)       // inverse transform
      dst_dist = trilinear(dst_grid, dst_local) // 8 lookups
      if dst_dist < margin:
        penetration = max(-dst_dist, 0)         // geometric-only depth
        emit contact
```

This loop is called **twice per SDF-SDF pair** (A→B, then B→A) every physics
step. `sdf_plane_contact` uses a similar grid iteration for SDF-vs-ground.

**Measured impact:** Example 09 (geometry-driven hinge) uses 100% of one CPU
core with 0.1% GPU. The physics timestep is 2kHz, so this loop runs 2000×/sec
per colliding pair. Activity Monitor shows the Apple GPU sitting entirely idle.

**Why it gets worse:** Finer collision grids (needed for smoother contacts and
tighter clearances) scale as O(grid_size³). A 20³ grid has 8,000 cells; a 40³
grid has 64,000. Multi-body scenes multiply this by the number of SDF pairs.

## 2. Goal

Move the grid evaluation loop to GPU compute shaders. Each grid cell is
independent — this is embarrassingly parallel. A single GPU dispatch replaces
the triple-nested loop.

**Key benefit beyond performance:** GPU acceleration enables finer collision
grids (0.5mm or 0.3mm cell size) without CPU cost. Finer grids produce
smoother contacts, tighter clearances, and more realistic constraint
emergence from geometry. This directly serves CortenForge's core value
proposition: physics that emerges from the printable shape.

## 3. Non-goals

- **Full GPU physics pipeline** — Phases 10b–10f (FK, broad-phase, constraint
  solver) are deferred. The constraint solver is a poor GPU fit (iterative,
  sparse, data-dependent).
- **GPU mesh generation** — Marching cubes for visual meshes runs once at
  startup, not per-step.
- **Multi-environment batching** — The removed Phase 10a (Euler integration)
  was designed for batched RL training. This spec targets single-environment
  interactive simulation.
- **Contact deduplication on GPU** — Dedup stays on CPU (§5.6).
- **SDF-vs-primitive contacts** — SDF-vs-Sphere, SDF-vs-Box, SDF-vs-Cylinder,
  etc. remain CPU-only. These are single-contact queries (1–40 sample points)
  where GPU dispatch overhead exceeds compute savings.
- **Analytical convex-convex contacts** — When both SDF shapes provide
  `effective_radius()`, the existing analytical single-contact path runs on
  CPU (cheap, exact). GPU only accelerates the grid-based fallback for
  non-convex shapes.

## 4. Architecture

### 4.1 Crate layering

```
cf-geometry          SdfGrid type (values, dims, cell_size, origin)
    ↓ (type)
sim-types            Pose type (position + UnitQuaternion)
    ↓ (type)
sim-core (L0)        GpuSdfCollision trait + collision pipeline dispatch
    ↑ (implements trait)
sim-gpu (L0)         wgpu backend, WGSL compute shaders (separate Device)
    ↑ (creates + injects)
sim-bevy (L1)        Creates GpuSdfCollider, sets on Model at startup
```

**sim-gpu** depends on sim-core (for trait + types). **sim-core** does NOT
depend on sim-gpu — the trait object is injected at runtime. No circular
dependencies. Both are Layer 0 (Bevy-free).

**Separate GPU device:** sim-gpu creates its own `wgpu::Device` and
`wgpu::Queue`, independent of Bevy's render device. This avoids contention
between physics compute and rendering, and works in headless (no-Bevy)
contexts.

### 4.2 Trait design

Defined in `sim-core::sdf`:

```rust
/// GPU-accelerated SDF collision backend.
///
/// Implementations upload SDF grids to GPU memory once, then dispatch
/// compute shaders each step with updated body poses. Grid indices
/// correspond to `model.shape_data` entries.
pub trait GpuSdfCollision: Send + Sync {
    /// Grid-based SDF-SDF contact detection (non-convex fallback).
    ///
    /// Traces the surface of both grids into each other (A→B and B→A),
    /// equivalent to calling `sdf_sdf_contact_raw()`.
    /// Returns contacts in world space. Dedup is done internally.
    fn sdf_sdf_contacts(
        &self,
        grid_a: usize,
        pose_a: &Pose,
        grid_b: usize,
        pose_b: &Pose,
        margin: f64,
    ) -> Vec<SdfContact>;

    /// SDF grid vs infinite plane contact detection.
    fn sdf_plane_contacts(
        &self,
        grid: usize,
        pose: &Pose,
        plane_normal: &Vector3<f64>,
        plane_offset: f64,
    ) -> Vec<SdfContact>;
}
```

### 4.3 Integration with collision pipeline

Stored on `Model`:

```rust
pub struct Model {
    // ... existing fields ...

    /// GPU-accelerated SDF collision (None = CPU fallback).
    pub gpu_collider: Option<Box<dyn GpuSdfCollision>>,
}
```

Dispatch in `collide_with_sdf()` (`collision/sdf_collide.rs`), SDF-SDF
branch. The GPU path only replaces the grid-based fallback — the analytical
convex-convex path is preserved:

```rust
// SDF-SDF pair
if model.geom_type[other_geom] == GeomType::Sdf {
    let Some(other_sdf_id) = model.geom_shape[other_geom] else {
        return vec![];
    };
    let cell_size_min = sdf
        .cell_size()
        .min(model.shape_data[other_sdf_id].sdf_grid().cell_size());
    let contact_margin = margin.max(cell_size_min * 0.5);

    let shape_a = &*model.shape_data[sdf_id];
    let shape_b = &*model.shape_data[other_sdf_id];

    // Check if both shapes are convex (analytical path — always CPU)
    let dir = separation_direction(&sdf_pose, &other_pose)
        .unwrap_or_else(Vector3::z);
    let both_convex =
        shape_a.effective_radius(&(sdf_pose.rotation.inverse() * dir)).is_some()
        && shape_b.effective_radius(&(other_pose.rotation.inverse() * (-dir))).is_some();

    let sdf_contacts = if both_convex {
        // Analytical convex-convex: exact single contact (CPU, cheap)
        compute_shape_contact(shape_a, &sdf_pose, shape_b, &other_pose, contact_margin)
    } else if let Some(gpu) = &model.gpu_collider {
        // Grid-based multi-contact: GPU
        gpu.sdf_sdf_contacts(sdf_id, &sdf_pose, other_sdf_id, &other_pose, contact_margin)
    } else {
        // Grid-based multi-contact: CPU fallback
        sdf_sdf_contact_raw(
            shape_a.sdf_grid(), &sdf_pose,
            shape_b.sdf_grid(), &other_pose,
            contact_margin,
        )
    };
    return sdf_contacts.iter().map(&convert).collect();
}

// SDF-Plane pair (similar pattern — analytical for convex, GPU for concave)
if model.geom_type[other_geom] == GeomType::Plane {
    let plane_normal = other_mat.column(2).into_owned();
    let sdf_contacts = if let Some(gpu) = &model.gpu_collider {
        gpu.sdf_plane_contacts(sdf_id, &sdf_pose, &plane_normal, plane_offset)
    } else {
        compute_shape_plane_contact(&*model.shape_data[sdf_id], &sdf_pose,
            &other_pos, &plane_normal, margin)
    };
    return sdf_contacts.iter().map(&convert).collect();
}
```

When `gpu_collider` is `None`, the existing CPU code path runs unchanged.

### 4.4 Data flow per step

```
One-time (at model creation):
  CPU → GPU:  SDF grid values (f64 → f32 via SdfGrid::values() accessor)
              Grid metadata (dims, cell_size, origin)
              Per grid: width × height × depth × 4 bytes
              Example: 20³ grid = 32 KB

Per physics step, per SDF-SDF pair:
  CPU → GPU:  src_pose  (mat4x4 f32 = 64 bytes)
              dst_pose  (mat4x4 f32 = 64 bytes)
              dst_pose_inv (mat4x4 f32 = 64 bytes)
              params    (margin, threshold, flip = 16 bytes)
              Total: ~208 bytes (×2 dispatches for symmetric trace)

  GPU:        Dispatch trace_a→b + trace_b→a (single command buffer)
              grid_a_cells + grid_b_cells threads

  GPU → CPU:  contact_count (4 bytes)
              contacts[0..N] (32 bytes each)
              Typical: < 200 contacts = 6.4 KB

Per physics step, per SDF-Plane pair:
  CPU → GPU:  pose (64 bytes) + plane (32 bytes)
  GPU → CPU:  contacts (same format)
```

**Apple Silicon note:** CPU and GPU share unified physical memory. However,
wgpu on Metal still uses `storageModeShared` for mappable buffers, which
may involve a Metal-layer copy for readback. Use `MAP_READ` staging buffers
for contact download. The overhead is still far smaller than the compute
savings.

## 5. GPU design

### 5.1 Buffer layout

**Per-grid buffers (persistent, uploaded once):**

| Buffer | Type | Size | Usage |
|--------|------|------|-------|
| `grid_values` | `storage<array<f32>>` | W×H×D × 4 bytes | SDF distance values (ZYX order) |

**Per-grid uniforms (persistent):**

| Field | Type | Bytes | Description |
|-------|------|-------|-------------|
| `width` | `u32` | 4 | Grid X dimension |
| `height` | `u32` | 4 | Grid Y dimension |
| `depth` | `u32` | 4 | Grid Z dimension |
| `cell_size` | `f32` | 4 | Uniform cell spacing |
| `origin` | `vec3<f32>` | 12 | Grid minimum corner |
| `_pad` | `f32` | 4 | Alignment to 16 bytes |
| | | **32** | |

**Per-dispatch uniforms (updated each step):**

All pose matrices are `mat4x4<f32>` (64 bytes each, no alignment issues
unlike `mat4x3` or `mat3x3` which have non-obvious WGSL padding rules).

| Field | Type | Bytes | Description |
|-------|------|-------|-------------|
| `src_pose` | `mat4x4<f32>` | 64 | Source local → world |
| `dst_pose` | `mat4x4<f32>` | 64 | Dest local → world (for normals) |
| `dst_pose_inv` | `mat4x4<f32>` | 64 | World → dest local |
| `surface_threshold` | `f32` | 4 | `src_cell_size * 2.0` |
| `contact_margin` | `f32` | 4 | Penetration detection range |
| `flip_normal` | `u32` | 4 | 0 = first call (A→B), 1 = second (B→A) |
| `_pad` | `f32` | 4 | Alignment |
| | | **208** | |

The `surface_threshold` is always computed from the **source** grid's
`cell_size`, matching the CPU code (`src_sdf.cell_size() * 2.0`).

The Rust-side struct must match this layout exactly (use `#[repr(C)]` +
`bytemuck::Pod` + explicit padding fields).

**Output buffers:**

| Buffer | Type | Size | Usage |
|--------|------|------|-------|
| `contacts` | `storage<array<GpuContact>>` | MAX_CONTACTS × 32 bytes | Contact point + normal + depth |
| `contact_count` | `storage<atomic<u32>>` | 4 bytes | Atomic counter (reset to 0 before dispatch) |

```wgsl
struct GpuContact {
    point: vec3<f32>,       // world-space contact position
    penetration: f32,       // penetration depth (≥ 0)
    normal: vec3<f32>,      // surface normal (world space)
    _pad: f32,              // alignment to 32 bytes
}
```

### 5.2 Compute shader: trace_surface kernel

The WGSL shader must match the CPU algorithm in `trace_surface_into_other()`
exactly. Key correctness constraints verified against source:

| CPU behavior | WGSL must match |
|-------------|-----------------|
| `src_value > threshold` (one-sided filter) | Skip only far-outside, process deep interior |
| `gradient()` = forward differences, normalized | `(f(x+eps) - f(x)) / eps`, return unit vector |
| `gradient()` degenerate → `+Z` | Return `vec3(0,0,1)` when `norm < 1e-5` |
| `penetration = max(-dst_dist, 0)` | Zero when touching, positive when overlapping |
| `flip_normal=false` → negate | First call (A→B): negate dst gradient |
| `flip_normal=true` → keep as-is | Second call (B→A): use dst gradient directly |

```wgsl
// trace_surface.wgsl — Exact port of trace_surface_into_other()
//
// One thread per grid cell in the source SDF.
// Tests whether the reconstructed surface point penetrates the
// destination SDF. If so, atomically appends a contact.

@group(0) @binding(0) var<storage, read> src_grid: array<f32>;
@group(0) @binding(1) var<storage, read> dst_grid: array<f32>;
@group(0) @binding(2) var<uniform> src_meta: GridMeta;
@group(0) @binding(3) var<uniform> dst_meta: GridMeta;
@group(0) @binding(4) var<uniform> params: TraceParams;
@group(0) @binding(5) var<storage, read_write> contacts: array<GpuContact>;
@group(0) @binding(6) var<storage, read_write> contact_count: atomic<u32>;

struct GridMeta {
    width: u32,
    height: u32,
    depth: u32,
    cell_size: f32,
    origin: vec3<f32>,
    _pad: f32,
}

struct TraceParams {
    src_pose: mat4x4<f32>,          // src local → world
    dst_pose: mat4x4<f32>,          // dst local → world (for normal rotation)
    dst_pose_inv: mat4x4<f32>,      // world → dst local
    surface_threshold: f32,          // src_cell_size * 2.0
    contact_margin: f32,
    flip_normal: u32,                // 0 = A→B (negate normal), 1 = B→A (keep)
    _pad: f32,
}

@compute @workgroup_size(8, 8, 4)
fn trace_surface(@builtin(global_invocation_id) gid: vec3<u32>) {
    let x = gid.x;
    let y = gid.y;
    let z = gid.z;
    if x >= src_meta.width || y >= src_meta.height || z >= src_meta.depth {
        return;
    }

    // 1. Read source SDF value
    let idx = z * src_meta.width * src_meta.height + y * src_meta.width + x;
    let src_value = src_grid[idx];

    // One-sided filter: skip far-outside points, but process deep interior.
    // Matches CPU: `if src_value > surface_threshold { continue; }`
    if src_value > params.surface_threshold {
        return;
    }

    // 2. Compute local position from grid coordinates
    let local = src_meta.origin + vec3<f32>(f32(x), f32(y), f32(z)) * src_meta.cell_size;

    // 3. Gradient via forward differences, normalized to unit vector.
    // Matches CPU: SdfGrid::gradient() in cf-geometry/src/sdf.rs
    let grad = sdf_gradient_normalized(local, src_meta);
    // sdf_gradient_normalized returns (0,0,0) on failure — check
    if dot(grad, grad) < 0.5 {
        return;  // Degenerate or out-of-bounds
    }

    // 4. Surface reconstruction: project grid point onto zero-isosurface
    let surface_local = local - grad * src_value;

    // 5. Transform: src local → world
    let world = (params.src_pose * vec4<f32>(surface_local, 1.0)).xyz;

    // 6. Transform: world → dst local
    let dst_local = (params.dst_pose_inv * vec4<f32>(world, 1.0)).xyz;

    // 7. Query destination SDF distance (trilinear interpolation)
    let dst_dist = trilinear_sample(dst_local, dst_meta);
    // Out-of-bounds → returns 1e6 → exceeds margin → skipped

    // 8. Contact test
    // Matches CPU: `if dst_dist >= margin { continue; }`
    if dst_dist >= params.contact_margin {
        return;
    }
    // Matches CPU: `let penetration = (-dst_dist).max(0.0);`
    // Zero when touching (dst_dist ≈ 0), positive when overlapping (dst_dist < 0).
    let penetration = max(-dst_dist, 0.0);

    // 9. Destination gradient for contact normal
    let dst_grad = sdf_gradient_normalized(dst_local, dst_meta);
    if dot(dst_grad, dst_grad) < 0.5 {
        return;
    }

    // 10. Normal convention — matches CPU exactly:
    //   flip_normal=0 (first call, A→B): negate dst outward gradient
    //   flip_normal=1 (second call, B→A): use dst outward gradient as-is
    // This ensures normal always points from body A toward body B.
    var normal_world = (params.dst_pose * vec4<f32>(dst_grad, 0.0)).xyz;
    if params.flip_normal == 0u {
        normal_world = -normal_world;
    }

    // 11. Atomic append to contact buffer
    let ci = atomicAdd(&contact_count, 1u);
    if ci < arrayLength(&contacts) {
        contacts[ci] = GpuContact(world, penetration, normal_world, 0.0);
    }
}
```

### 5.3 Trilinear interpolation and gradient (WGSL)

The gradient must match the CPU implementation in `SdfGrid::gradient()`:
- **Forward differences** (not central): `(f(x+eps) - f(x)) / eps`
- **Clamped forward sample**: `distance_clamped()` clamps to grid bounds
- **Normalized**: always returns a unit vector
- **Degenerate fallback**: returns `+Z` when gradient norm < threshold

```wgsl
fn grid_idx(x: u32, y: u32, z: u32, m: GridMeta) -> u32 {
    return z * m.width * m.height + y * m.width + x;
}

// Trilinear interpolation. Returns 1e6 if point is outside grid bounds.
fn trilinear_sample(point: vec3<f32>, m: GridMeta) -> f32 {
    let rel = (point - m.origin) / m.cell_size;

    // Bounds check
    if rel.x < 0.0 || rel.y < 0.0 || rel.z < 0.0 {
        return 1e6;
    }
    let i = vec3<u32>(vec3<i32>(floor(rel)));
    if i.x >= m.width - 1u || i.y >= m.height - 1u || i.z >= m.depth - 1u {
        return 1e6;
    }

    let f = fract(rel);

    // 8 corner values (ZYX storage order)
    let c000 = src_or_dst_grid[grid_idx(i.x,     i.y,     i.z,     m)];
    let c100 = src_or_dst_grid[grid_idx(i.x + 1u, i.y,     i.z,     m)];
    let c010 = src_or_dst_grid[grid_idx(i.x,     i.y + 1u, i.z,     m)];
    let c110 = src_or_dst_grid[grid_idx(i.x + 1u, i.y + 1u, i.z,     m)];
    let c001 = src_or_dst_grid[grid_idx(i.x,     i.y,     i.z + 1u, m)];
    let c101 = src_or_dst_grid[grid_idx(i.x + 1u, i.y,     i.z + 1u, m)];
    let c011 = src_or_dst_grid[grid_idx(i.x,     i.y + 1u, i.z + 1u, m)];
    let c111 = src_or_dst_grid[grid_idx(i.x + 1u, i.y + 1u, i.z + 1u, m)];

    // Trilinear interpolation (matches CPU mul_add pattern)
    let c00 = mix(c000, c100, f.x);
    let c10 = mix(c010, c110, f.x);
    let c01 = mix(c001, c101, f.x);
    let c11 = mix(c011, c111, f.x);
    let c0  = mix(c00,  c10,  f.y);
    let c1  = mix(c01,  c11,  f.y);
    return mix(c0, c1, f.z);
}

// Trilinear interpolation with clamping (for gradient forward samples).
// Clamps point to grid bounds instead of returning 1e6.
// Matches CPU: SdfGrid::distance_clamped()
fn trilinear_sample_clamped(point: vec3<f32>, m: GridMeta) -> f32 {
    let max_bound = m.origin + vec3<f32>(
        f32(m.width - 1u),
        f32(m.height - 1u),
        f32(m.depth - 1u),
    ) * m.cell_size;
    let clamped = clamp(point, m.origin, max_bound);
    // After clamping, trilinear_sample will always be in bounds
    return trilinear_sample(clamped, m);
}

// Gradient via forward differences, returning a normalized unit vector.
// Matches CPU: SdfGrid::gradient() in cf-geometry/src/sdf.rs
//
// CPU algorithm:
//   d  = distance(point)              ← base value (returns None if OOB)
//   dx = distance_clamped(point + eps_x) ← clamped to bounds
//   dy = distance_clamped(point + eps_y)
//   dz = distance_clamped(point + eps_z)
//   grad = ((dx-d)/eps, (dy-d)/eps, (dz-d)/eps)
//   return grad / grad.norm()         ← normalized
//   fallback: +Z if degenerate
fn sdf_gradient_normalized(point: vec3<f32>, m: GridMeta) -> vec3<f32> {
    let d = trilinear_sample(point, m);
    if d > 1e5 {
        return vec3(0.0, 0.0, 0.0);  // out of bounds → caller checks
    }

    let eps = m.cell_size * 0.5;
    let dx = trilinear_sample_clamped(point + vec3(eps, 0.0, 0.0), m);
    let dy = trilinear_sample_clamped(point + vec3(0.0, eps, 0.0), m);
    let dz = trilinear_sample_clamped(point + vec3(0.0, 0.0, eps), m);

    let grad = vec3((dx - d) / eps, (dy - d) / eps, (dz - d) / eps);
    let norm = length(grad);

    if norm > 1e-5 {
        return grad / norm;      // unit vector
    } else {
        return vec3(0.0, 0.0, 1.0);  // +Z fallback (matches CPU)
    }
}
```

**WGSL implementation note:** The `trilinear_sample` function above
references `src_or_dst_grid` as a placeholder. In practice, two separate
functions are needed (one reading `src_grid`, one reading `dst_grid`)
because WGSL module-scope bindings cannot be passed as function
parameters in all backends. Alternatively, use two bind groups with
identical layout and swap at dispatch time.

**Cost per thread:** The gradient uses 4 trilinear samples (1 base + 3
forward), each doing 8 grid reads = 32 reads. The kernel does 1 src
gradient + 1 dst gradient (when penetrating) = up to 64 grid reads +
~40 FLOPs of arithmetic. High arithmetic intensity — good GPU utilization.

### 5.4 Workgroup sizing

`@workgroup_size(8, 8, 4)` = 256 threads per workgroup.

| Grid size | Workgroups | Threads | GPU occupancy |
|-----------|-----------|---------|---------------|
| 10³ | 2×2×3 = 12 | 3,072 | Low (fast) |
| 20³ | 3×3×5 = 45 | 11,520 | Moderate |
| 40³ | 5×5×10 = 250 | 64,000 | Good |
| 60³ | 8×8×15 = 960 | 245,760 | Excellent |

Even 20³ grids produce enough threads for meaningful GPU acceleration.
The threshold where GPU beats CPU is approximately grid_size ≥ 10³.

### 5.5 Dispatch strategy

For SDF-SDF contacts, `sdf_sdf_contact_raw()` calls
`trace_surface_into_other()` twice (A→B, B→A). On GPU, encode both
dispatches in a single command buffer:

```rust
let mut encoder = device.create_command_encoder(&default());

// Reset atomic counters to 0
// (copy zero-buffer to contact_count_a and contact_count_b)

// Dispatch 1: trace A's surface into B (flip_normal = 0)
{
    let mut pass = encoder.begin_compute_pass(&default());
    pass.set_pipeline(&trace_pipeline);
    pass.set_bind_group(0, &trace_a_into_b_bind_group, &[]);
    pass.dispatch_workgroups(
        (grid_a.width + 7) / 8,
        (grid_a.height + 7) / 8,
        (grid_a.depth + 3) / 4,
    );
}

// Dispatch 2: trace B's surface into A (flip_normal = 1)
{
    let mut pass = encoder.begin_compute_pass(&default());
    pass.set_pipeline(&trace_pipeline);
    pass.set_bind_group(0, &trace_b_into_a_bind_group, &[]);
    pass.dispatch_workgroups(
        (grid_b.width + 7) / 8,
        (grid_b.height + 7) / 8,
        (grid_b.depth + 3) / 4,
    );
}

// Single submit — both dispatches run on GPU without CPU round-trip
queue.submit([encoder.finish()]);

// One readback: map both contact buffers, merge results
```

### 5.6 Deduplication (CPU post-process)

GPU returns all penetrating contacts without deduplication. CPU dedup
runs after download.

**Intentional deviation from CPU:** The CPU dedup is inline (order-dependent
on grid traversal: earlier cells dominate). The GPU dedup sorts by
penetration depth first, then greedily keeps non-dominated contacts.
This produces a **deterministic** contact set independent of grid iteration
order, which is preferable. The CPU-vs-GPU comparison tests use position
tolerances rather than exact contact matching.

```rust
fn dedup_contacts(contacts: &mut Vec<SdfContact>, min_dist_sq: f64) {
    // Sort deepest first — ensures most important contacts survive
    contacts.sort_by(|a, b| b.penetration.partial_cmp(&a.penetration).unwrap());
    let mut kept = Vec::with_capacity(contacts.len());
    for c in contacts.drain(..) {
        let dominated = kept.iter().any(|k: &SdfContact|
            (k.point - c.point).norm_squared() < min_dist_sq
        );
        if !dominated {
            kept.push(c);
        }
    }
    *contacts = kept;
}
```

Dedup distance: `cell_size * 0.5` (same threshold as CPU).

## 6. Precision: f64 → f32

SDF grid values and all GPU computation use **f32**. The CPU pipeline uses f64.

**Why f32 is adequate:**
- SDF values are in mm scale (±10mm typical range)
- f32 gives ~7 significant digits → 0.001mm precision at 10mm
- Collision clearances are 0.5–1.0mm — three orders of magnitude above f32 noise
- Contact positions need ~0.01mm accuracy for constraint solver — well within f32

**Where f64 stays:**
- Constraint assembly and PGS solver (CPU, existing code)
- Integration and FK (CPU, existing code)
- `SdfContact` fields are f64 — GPU f32 contacts are promoted on download

**Conversion:**
- Upload: `grid.values().iter().map(|&v| v as f32).collect()` (one-time,
  uses new `SdfGrid::values()` accessor — see §7 prerequisites)
- Download: `SdfContact { point: Point3::new(g.x as f64, ...), ... }`

## 7. Implementation phases

### Prerequisites (cf-geometry)

Before Phase 2, add a public accessor to `SdfGrid` in
`design/cf-geometry/src/sdf.rs`:

```rust
/// Raw grid values for GPU upload. ZYX storage order.
pub fn values(&self) -> &[f64] {
    &self.values
}
```

This is the only change needed outside sim-gpu and sim-core.

### Phase 1: sim-gpu crate + GPU context

**Scope:** Create the crate, initialize wgpu, verify GPU is accessible.

- Create `sim/L0/gpu/` with `Cargo.toml`
- Dependencies: `wgpu`, `bytemuck`, `pollster`, `cf-geometry`, `sim-types`,
  `sim-core`, `nalgebra`, `log`
- `GpuContext` struct: own `wgpu::Device` + `wgpu::Queue` (separate from Bevy)
- Async init with sync fallback (`pollster::block_on`)
- Graceful error handling when no GPU available (return `Result`)
- Add to workspace `Cargo.toml`

**Test:** `gpu_context_creates_on_metal` — verify device creation on macOS.

**Deliverable:** `sim-gpu` crate compiles and creates a GPU context.

### Phase 2: SDF grid upload + readback

**Scope:** Upload `SdfGrid` data to GPU, read it back, verify correctness.

- Add `SdfGrid::values()` accessor to cf-geometry (prerequisite)
- `GpuSdfGrid` struct: storage buffer + metadata uniform
- `upload_grid(grid: &SdfGrid) -> GpuSdfGrid` — converts f64→f32, creates
  buffer with `STORAGE | COPY_SRC` usage
- Readback test: upload grid, map buffer, compare to CPU values

**Test:** `grid_upload_roundtrip` — upload a known `SdfGrid`, read back f32
values, verify they match `grid.values().iter().map(|v| *v as f32)` exactly.

**Deliverable:** GPU buffer management for SDF grids.

### Phase 3: trace_surface compute shader

**Scope:** The core kernel — port `trace_surface_into_other` to WGSL.

- `trace_surface.wgsl` compute shader (§5.2–5.3)
- Bind group layout: src_grid, dst_grid, src_meta, dst_meta, params,
  contacts, contact_count
- Rust-side `TraceParams` struct with `#[repr(C)]` + `bytemuck::Pod` +
  explicit padding matching WGSL layout (§5.1)
- Pose conversion: `Pose` → `mat4x4<f32>` (quaternion to rotation matrix,
  column-major for WGSL)
- Pipeline creation and dispatch
- Contact buffer readback and f32→f64 promotion
- CPU deduplication (§5.6)

**Tests:**
- `trilinear_matches_cpu` — Compare GPU trilinear sample to CPU
  `SdfGrid::distance()` at 100 random points. Tolerance: 1e-4.
- `gradient_matches_cpu` — Compare GPU gradient to CPU
  `SdfGrid::gradient()` at 100 random points. Tolerance: 1e-3.
  Both must use forward differences and return unit vectors.
- `trace_contacts_match_cpu` — Run `sdf_sdf_contact_raw` on CPU and GPU
  with identical inputs (sphere SDF pair). Verify all GPU contacts have
  a CPU contact within 0.2mm. Count tolerance: ±20% (dedup ordering
  difference, f32 vs f64 surface reconstruction).
- `hinge_contacts_match_cpu` — Same test with the hinge geometry
  (concave socket + flanged pin).

**Deliverable:** GPU `trace_surface_into_other` produces correct contacts.

### Phase 4: trait + integration

**Scope:** Wire GPU collision into the sim-core pipeline.

- `GpuSdfCollision` trait in `sim-core::sdf` (§4.2)
- `GpuSdfCollider` struct in sim-gpu implementing the trait
- Constructor: `GpuSdfCollider::new(shape_data: &[Arc<dyn PhysicsShape>])
  -> Result<Self, GpuError>` — uploads all SDF grids
- `sdf_sdf_contacts()` — dispatches trace A→B + B→A, readback, dedup
- `Model::gpu_collider` field (§4.3)
- Dispatch logic in `collide_with_sdf()` preserving analytical convex
  path (§4.3)
- sim-bevy creates `GpuSdfCollider` at startup, sets on Model

**Tests:**
- `example_09_gpu_backend` — Example 09 runs with GPU backend, all 6
  pass/fail checks green.
- `gpu_fallback_when_unavailable` — GPU collider creation returns `Err`
  gracefully, `gpu_collider` stays `None`, CPU fallback runs.
- `convex_convex_still_analytical` — Two ShapeSphere SDFs still use
  the exact analytical path, not the GPU grid path.

**Deliverable:** Example 09 runs interactively with GPU SDF collision.

### Phase 5: SDF-plane shader

**Scope:** GPU acceleration for SDF-vs-ground-plane contacts.

- `sdf_plane.wgsl` — simpler variant (one grid, plane equation).
  Note: `sdf_plane_contact` has a different algorithm from
  `trace_surface_into_other` (see §5.7 for differences).
- `sdf_plane_contacts()` on the trait
- Dispatch in `collide_with_sdf()` for Plane case

**Tests:**
- `plane_contacts_match_cpu` — Compare GPU and CPU plane contacts.
- Stacking examples (01–08) still work with GPU backend.

**Deliverable:** All SDF grid-based collision paths accelerated on GPU.

### 5.7 SDF-Plane differences from SDF-SDF

`sdf_plane_contact()` in `primitives.rs` differs from
`trace_surface_into_other()`:

1. Uses the grid point for the initial plane distance check, then
   reconstructs the surface point only after the test passes
2. Uses the **plane normal** (constant) as contact normal, not the SDF
   gradient
3. Truncates to `MAX_SDF_PLANE_CONTACTS = 1` (deepest only)

The GPU plane shader must replicate these differences, not reuse the
trace_surface kernel.

## 8. Testing strategy

### Correctness validation

Every phase includes a CPU-vs-GPU comparison test. Because dedup ordering
intentionally differs (§5.6), tests compare contact **positions** with
spatial tolerance rather than exact matching:

```rust
#[test]
fn trace_contacts_match_cpu() {
    let (sdf_a, sdf_b) = make_test_geometry();
    let (pose_a, pose_b, margin) = make_test_poses();

    let cpu_contacts = sdf_sdf_contact_raw(&sdf_a, &pose_a, &sdf_b, &pose_b, margin);
    let gpu_contacts = gpu_collider.sdf_sdf_contacts(0, &pose_a, 1, &pose_b, margin);

    // Contact count within 20% (different dedup ordering + f32 precision)
    let count_diff = (cpu_contacts.len() as f64 - gpu_contacts.len() as f64).abs();
    assert!(count_diff / cpu_contacts.len().max(1) as f64 < 0.2,
        "contact count mismatch: CPU={}, GPU={}", cpu_contacts.len(), gpu_contacts.len());

    // Every CPU contact has a GPU contact within 0.2mm
    for cpu_c in &cpu_contacts {
        let nearest = gpu_contacts.iter()
            .map(|g| (g.point - cpu_c.point).norm())
            .min_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap_or(f64::MAX);
        assert!(nearest < 0.2,
            "CPU contact at ({:.3},{:.3},{:.3}) has no GPU match within 0.2mm",
            cpu_c.point.x, cpu_c.point.y, cpu_c.point.z);
    }
}
```

### Performance measurement

```rust
#[test]
fn gpu_faster_than_cpu() {
    // Use 30³ grid (large enough for meaningful comparison)
    let start_cpu = Instant::now();
    for _ in 0..100 {
        sdf_sdf_contact_raw(&sdf_a, &pose_a, &sdf_b, &pose_b, margin);
    }
    let cpu_time = start_cpu.elapsed();

    let start_gpu = Instant::now();
    for _ in 0..100 {
        gpu_collider.sdf_sdf_contacts(0, &pose_a, 1, &pose_b, margin);
    }
    let gpu_time = start_gpu.elapsed();

    eprintln!("CPU: {cpu_time:?}, GPU: {gpu_time:?}");
    assert!(gpu_time < cpu_time, "GPU should be faster for 30³ grid");
}
```

## 9. Performance targets

| Metric | CPU (current) | GPU (target) |
|--------|--------------|--------------|
| SDF-SDF trace (20³) | ~2–5ms | < 0.5ms |
| SDF-Plane (20³) | ~1ms | < 0.2ms |
| Per-step upload | — | < 0.01ms (poses only) |
| Per-step download | — | < 0.05ms (contacts) |
| CPU core usage | 100% | < 30% |
| GPU usage | 0% | ~10–30% |

**Note on small grids:** For grids < 10³ cells, GPU dispatch overhead
(~50–200μs on Metal) may equal or exceed compute savings. The
implementation should measure at the actual grid sizes used and fall
back to CPU if GPU provides no benefit.

**Scaling headroom:** With GPU, a 40³ collision grid costs ~1ms (vs ~40ms
on CPU). This enables 0.5mm cell size for smoother constraint geometry
without impacting real-time performance.

## 10. Key files

### Existing (CPU collision pipeline)

| File | Role |
|------|------|
| `sim/L0/core/src/sdf/operations.rs` | `trace_surface_into_other()` — THE hot loop |
| `sim/L0/core/src/sdf/primitives.rs` | `sdf_plane_contact()` — secondary hot loop |
| `sim/L0/core/src/sdf/shape.rs` | `compute_shape_contact()` — analytical/grid dispatch |
| `sim/L0/core/src/sdf/mod.rs` | `SdfContact` type, module exports |
| `sim/L0/core/src/collision/sdf_collide.rs` | `collide_with_sdf()` — GPU dispatch site |
| `design/cf-geometry/src/sdf.rs` | `SdfGrid` — grid data, trilinear, gradient |
| `sim/L0/types/src/body.rs` | `Pose` — position + UnitQuaternion |

### New (GPU acceleration)

| File | Role |
|------|------|
| `sim/L0/gpu/Cargo.toml` | Crate manifest (wgpu, bytemuck, pollster) |
| `sim/L0/gpu/src/lib.rs` | Crate root, `GpuSdfCollider` |
| `sim/L0/gpu/src/context.rs` | `GpuContext` — device/queue init (own device) |
| `sim/L0/gpu/src/buffers.rs` | `GpuSdfGrid` — buffer management, f64→f32 |
| `sim/L0/gpu/src/collision.rs` | Dispatch logic, contact readback, dedup |
| `sim/L0/gpu/src/shaders/trace_surface.wgsl` | Surface tracing compute shader |
| `sim/L0/gpu/src/shaders/sdf_plane.wgsl` | Plane contact compute shader |
| `design/cf-geometry/src/sdf.rs` | Add `SdfGrid::values()` accessor |

## 11. Dependencies

```toml
[package]
name = "sim-gpu"
version = "0.1.0"

[dependencies]
wgpu = "24"
bytemuck = { version = "1", features = ["derive"] }
pollster = "0.4"
cf-geometry = { path = "../../../design/cf-geometry" }
sim-types = { path = "../types" }
sim-core = { path = "../core" }
nalgebra = "0.33"
log = "0.4"
```

## 12. Risk assessment

| Risk | Mitigation |
|------|------------|
| f32 precision artifacts near tight clearances | Tolerance tests at 0.5mm clearance; comparison tests use 0.2mm position tolerance |
| wgpu init fails on some hardware | `GpuSdfCollider::new()` returns `Result`; Model uses `None` → CPU fallback |
| Atomic contention on contact_count | Buffer sized to grid_size³ (worst case); contention is light (< 5% of threads emit) |
| GPU dispatch overhead exceeds compute for small grids | Measure at actual grid sizes; CPU path remains for grids < 10³ cells |
| Shader compilation latency at startup | Pipeline creation is one-time; cache via wgpu pipeline cache |
| WGSL `ptr` function params rejected by naga | Use module-scope bindings directly; duplicate functions for src/dst grid |
| Bevy render device contention | sim-gpu creates its own wgpu Device + Queue (§4.1) |
| `Pose` → `mat4x4` conversion wrong (row vs column major) | WGSL uses column-major; test with known rotation + known point |
| Rust struct alignment doesn't match WGSL layout | `#[repr(C)]` + explicit padding; test by writing known values and reading in shader |

## Appendix A: Adversarial review issues resolved

This spec was stress-tested by adversarial review. All issues found have
been addressed:

| # | Severity | Issue | Resolution |
|---|----------|-------|------------|
| 1 | CRITICAL | Gradient: central vs forward differences | Fixed: WGSL uses forward differences matching CPU (§5.3) |
| 2 | CRITICAL | Gradient not normalized in WGSL | Fixed: `sdf_gradient_normalized` returns unit vector (§5.3) |
| 3 | CRITICAL | Surface filter `abs()` vs one-sided `>` | Fixed: WGSL uses `src_value > threshold` (§5.2) |
| 4 | CRITICAL | Penetration `margin - dist` vs `max(-dist, 0)` | Fixed: WGSL uses `max(-dst_dist, 0.0)` (§5.2) |
| 5 | CRITICAL | GPU bypasses analytical convex-convex path | Fixed: dispatch checks `effective_radius` first (§4.3) |
| 6 | CRITICAL | `SdfGrid.values` is private | Fixed: prerequisite adds `values()` accessor (§7) |
| 7 | MAJOR | Dedup order-dependence | Documented: intentional improvement (§5.6) |
| 8 | MAJOR | `flip_normal` logic inverted | Fixed: `flip_normal == 0u` → negate (§5.2) |
| 9 | MAJOR | `mat4x3` / `mat3x3` alignment issues | Fixed: all poses use `mat4x4` (§5.1) |
| 10 | MAJOR | Constructor takes `Box` not `Arc` | Fixed: takes `&[Arc<dyn PhysicsShape>]` (§7 Phase 4) |
| 11 | MAJOR | Shared wgpu device with Bevy | Fixed: own Device + Queue (§4.1) |
| 12 | MAJOR | Contact struct 28 vs 32 bytes | Fixed: 32 bytes throughout (§4.4, §5.1) |
| 13 | MINOR | SDF-Plane algorithm differences | Documented (§5.7) |
| 14 | MINOR | `surface_threshold` from which grid | Documented: source grid (§5.1) |
| 15 | MINOR | SDF-vs-primitive stays CPU | Documented in non-goals (§3) |
| 16 | MINOR | Unified memory nuance | Clarified staging buffer requirement (§4.4) |
| 17 | MINOR | WGSL `ptr` parameter portability | Added risk + mitigation (§12) |
| 18 | MINOR | Performance at small grid sizes | Added note (§9) |
