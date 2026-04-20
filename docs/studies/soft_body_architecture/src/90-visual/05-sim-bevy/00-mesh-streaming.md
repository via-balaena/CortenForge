# Mesh streaming API

The physics step produces a deformed mesh plus per-vertex physics attributes; the renderer consumes them to rasterize the frame. These two run at different rates — physics at a variable 1–16 ms cadence with adaptive-shrink events, renderer at a fixed vsync — and coupling them tightly would mean either stalling the renderer on a slow physics step or skipping physics to meet vsync. `sim-bevy`'s solution is a double-buffered `MeshSnapshot` record: physics writes into the inactive buffer while the renderer reads from the active one, and an atomic pointer swap on physics-step completion switches which buffer each side sees. This sub-leaf specifies the snapshot layout, the swap protocol, the GPU upload path, and the staleness bound.

## The snapshot record

A `MeshSnapshot` carries one full rendered-frame's worth of physics state:

```rust
pub struct MeshSnapshot {
    pub vertices:         Vec<Vec3>,    // deformed positions, per-vertex
    pub normals:          Vec<Vec3>,    // surface normals, per-vertex
    pub stress_per_vertex: Vec<f32>,    // von Mises stress, for diagnostic render
    pub temperature:      Vec<f32>,     // from sim-thermostat
    pub thickness:        Vec<f32>,     // SDF-distance to far surface, for SSS
    pub contact_pressure: Vec<f32>,     // per-vertex integrated contact pressure
    pub layer_id:         Vec<u8>,      // outermost-layer ID per vertex
    pub anisotropy_dir:   Vec<Vec3>,    // local anisotropy axis per vertex
    pub indices:          Vec<u32>,     // triangle list indices (from surface extraction + subdivision)
    pub timestamp:        u64,          // physics-step counter for version tracking
}
```

The fields map one-to-one onto the shader inputs specified in [§2's pipeline](01-shader-pipeline.md): positions + normals drive rasterization; `stress_per_vertex` is available for diagnostic overlays; `temperature` feeds the [thermal shader](../04-thermal-viz.md); `thickness` parameterizes the [SSS kernel width](../00-sss/02-realtime.md); `contact_pressure` drives the [wet-surface](../01-anisotropic-reflection/00-wet.md) and [wrinkle-bump](../02-micro-wrinkles/00-contact-normals.md) shaders; `layer_id` keys into the [SSS layer-stack](../00-sss/00-stacked.md); `anisotropy_dir` feeds the [GGX tangent axes](../01-anisotropic-reflection/01-ggx.md).

Size on the canonical scene (~30k-tet physics, ~12k surface triangles subdivided one level to ~48k render-triangles, ~24k unique surface vertices) is on the order of two megabytes — dominated by the `Vec3` positions and normals, with the scalar attributes adding a smaller contribution. This fits comfortably in the per-physics-step GPU-upload budget on PCIe 4.0 hardware, which is what the spine commits Phase E to.

## Double-buffering protocol

Two snapshots, one read-only-for-renderer and one write-only-for-physics, swap on physics-step completion:

```rust
pub struct SnapshotBuffers {
    a: Arc<RwLock<MeshSnapshot>>,
    b: Arc<RwLock<MeshSnapshot>>,
    reader_picks_a: AtomicBool,        // true → renderer reads a, physics writes b
}

impl SnapshotBuffers {
    pub fn physics_step_complete(&self) {
        // flip the atomic, so on the next frame the renderer reads what physics just wrote
        self.reader_picks_a.fetch_xor(true, Ordering::AcqRel);
    }

    pub fn read_active(&self) -> RwLockReadGuard<'_, MeshSnapshot> {
        if self.reader_picks_a.load(Ordering::Acquire) {
            self.a.read()
        } else {
            self.b.read()
        }
    }

    pub fn write_inactive(&self) -> RwLockWriteGuard<'_, MeshSnapshot> {
        if self.reader_picks_a.load(Ordering::Acquire) {
            self.b.write()
        } else {
            self.a.write()
        }
    }
}
```

The `RwLock` guards the snapshot's internal `Vec`s from data races during the brief overlap when a physics step starts while a render is still reading (a reader-then-writer sequence that an over-eager scheduler might produce). In practice the lock is essentially uncontended — the renderer reads a snapshot, swaps to the next snapshot on the atomic flip, and never returns to the previous one — but the locking is cheap insurance rather than a hot-path cost.

The atomic swap is release-acquire ordered: the physics side's writes to the snapshot buffer have release semantics, and the renderer's reads have acquire semantics, so the memory model guarantees that a renderer reading after the swap sees a fully-written snapshot, not a partially-updated one.

## GPU upload on each swap

On the physics-step-complete event, the renderer-side Bevy system schedules a GPU upload of the newly-active snapshot's buffers. `sim-bevy` uses wgpu's `write_buffer` for this — a single call per buffer, asynchronous, completes before the next render pass consumes the data. The upload sequence is:

```rust
fn upload_snapshot_to_gpu(
    device: &RenderDevice,
    queue: &RenderQueue,
    snapshot: &MeshSnapshot,
    gpu_buffers: &mut GpuMeshBuffers,
) {
    queue.write_buffer(&gpu_buffers.vertices,          0, bytemuck::cast_slice(&snapshot.vertices));
    queue.write_buffer(&gpu_buffers.normals,           0, bytemuck::cast_slice(&snapshot.normals));
    queue.write_buffer(&gpu_buffers.stress_per_vertex, 0, bytemuck::cast_slice(&snapshot.stress_per_vertex));
    queue.write_buffer(&gpu_buffers.thickness,         0, bytemuck::cast_slice(&snapshot.thickness));
    queue.write_buffer(&gpu_buffers.temperature,       0, bytemuck::cast_slice(&snapshot.temperature));
    queue.write_buffer(&gpu_buffers.contact_pressure,  0, bytemuck::cast_slice(&snapshot.contact_pressure));
    queue.write_buffer(&gpu_buffers.layer_id,          0, bytemuck::cast_slice(&snapshot.layer_id));
    queue.write_buffer(&gpu_buffers.anisotropy_dir,    0, bytemuck::cast_slice(&snapshot.anisotropy_dir));
    // indices re-uploaded only when subdivision topology changes (rare)
}
```

Topology changes are rare — the [Part 7 Ch 04 `TopologyChanging` edit class](../../70-sdf-pipeline/04-live-remesh.md) fires on mesh re-meshes, which happen at design-edit granularity, not physics-step granularity. Between topology changes the index buffer stays resident on the GPU; only the per-vertex attribute buffers re-upload each step.

## Staleness bound

A renderer reading a snapshot that predates the current physics step by one step sees a maximally stale snapshot — at design-mode rates (1 ms physics steps), staleness is bounded by 1 ms; at experience-mode rates (16 ms), by 16 ms. Neither is noticeable to the viewer: 1 ms is below any human perceptual threshold, and 16 ms is one vsync frame at 60 Hz, within the normal rendering delay anyway.

Under [adaptive-$\Delta t$ shrink events](../../50-time-integration/02-adaptive-dt.md), the physics step may take longer than its nominal cadence. The renderer keeps rendering from the active snapshot during the slow step; the user sees a brief visual pause (the mesh is not animating), which is the correct visualization of a slow physics step, not a bug. The renderer does not skip frames or interpolate between snapshots to hide the pause.

## What this sub-leaf commits the book to

- **MeshSnapshot is the physics-to-render interface**, carrying all per-vertex physics attributes needed by the [§1 shader pipeline](01-shader-pipeline.md).
- **Double-buffered, writer-reader pattern with atomic pointer swap.** The renderer never blocks on physics; the physics never blocks on rendering.
- **GPU upload via wgpu `write_buffer` on each swap.** Per-vertex buffers re-upload each step; indices re-upload only on topology changes.
- **Staleness bounded by one physics step.** 1 ms at design-mode resolution, 16 ms at experience-mode; both below perceptually-noticeable thresholds.
- **Adaptive-timestep shrinks pause the render.** The renderer holds the current snapshot during a slow step rather than interpolating or skipping.
