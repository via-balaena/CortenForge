# With sim-bevy

`sim-bevy` is the Bevy-side consumer of `sim-soft`'s visual layer — the crate that turns per-vertex physics output into rendered frames via the shader pipeline committed in [Part 9](../../90-visual/00-sss.md). This coupling is the narrowest of the five: it is **out-only**. `sim-soft` pushes `MeshSnapshot` records; `sim-bevy` reads and renders. No state flows back into the physics. Nothing `sim-bevy` does can change `sim-soft`'s trajectory or force the next solve into a different regime. The architectural justification lives in [Part 1 Ch 03 thesis](../../10-physical/03-thesis.md)'s "the visual configuration is a shader, not a second simulation" — a commitment this sub-leaf operationalizes on the cross-crate surface.

## What crosses the boundary

| Field | Direction | Representation | Required? |
|---|---|---|---|
| `MeshSnapshot` — deformed positions + per-vertex attributes | `sim-soft` → `sim-bevy` | struct-of-arrays with vertices, normals, stress, temperature, thickness, contact pressure, layer ID, anisotropy direction, indices, timestamp | yes when `sim-bevy` is present |
| `SimSoftConfig` — render-tier toggles | `sim-bevy` → `sim-soft` at construction only | SSS tier, wrinkle enable, thermal overlay mode, target physics timestep | opt-in; construction-time only |

The critical asymmetry is that `SimSoftConfig` crosses once at construction, not per step. Nothing the renderer does per-frame feeds back into `sim-soft`'s next step. The coupling has zero flow-into-physics, which is what lets `sim-bevy` be removed or replaced without touching the simulation stack. `sim-soft` runs headless in CI, under `sim-ml-chassis`'s optimizer loop, under any downstream consumer that does not need pixels, with no awareness of whether anyone is watching.

## The snapshot surface — `MeshSnapshot`

The snapshot type is committed in [Part 9 Ch 05 §00 mesh-streaming](../../90-visual/05-sim-bevy/00-mesh-streaming.md) and consumed in [Part 9 Ch 05](../../90-visual/05-sim-bevy.md) by the Bevy-side `SimSoftPlugin`. This sub-leaf pins the cross-crate surface:

```rust
pub struct MeshSnapshot {
    pub vertices:          Vec<Vec3>,   // deformed positions, per-vertex
    pub normals:           Vec<Vec3>,
    pub stress_per_vertex: Vec<f32>,    // von Mises scalar
    pub temperature:       Vec<f32>,    // from sim-thermostat view; zero-filled if coupling disabled
    pub thickness:         Vec<f32>,    // SDF-distance to far surface, for SSS
    pub contact_pressure:  Vec<f32>,    // per-vertex integrated contact pressure
    pub layer_id:          Vec<u8>,     // outermost layer; full stack looked up per-scene
    pub anisotropy_dir:    Vec<Vec3>,
    pub indices:           Vec<u32>,
    pub timestamp:         u64,          // physics-step counter for versioning
}
```

Every field is driven by the physics. `stress_per_vertex` is a projection of per-tet stress onto vertices from [`readout/`'s stress-field extraction](../00-module-layout/09-readout.md); `thickness` is the SDF-distance from surface vertex to far-side surface per [Part 9 Ch 00 §01 SSS bssrdf](../../90-visual/00-sss/01-bssrdf.md); `contact_pressure` is the per-vertex integrated IPC barrier pressure per [Part 4 Ch 01 §00 barrier](../../40-contact/01-ipc-internals/00-barrier.md); `temperature` is the `sim-thermostat` view from [§01](01-thermostat.md)'s `TemperatureView`, sampled per-vertex at readout time. `layer_id` and `anisotropy_dir` come from the `MaterialField`'s renderer-facing channels per [`material/`](../00-module-layout/00-material.md) and [Part 7 Ch 00 §04 MaterialField](../../70-sdf-pipeline/00-sdf-primitive.md).

The `MeshSnapshot` struct is `sim-bevy`'s canonical consumer surface. `sim-soft` produces the struct from [`readout/`'s `Observable` trait implementation](../01-traits/00-core.md) — the same trait [`sim-opt`](03-ml-chassis.md) consumes on the ML side. One extraction pass feeds both reward and rendering; this is the [`readout/` claim 4](../00-module-layout/09-readout.md) "`sim-bevy`'s visual layer reads the same attributes" commitment made concrete. The shader inputs are the physics outputs — no baked textures, no parallel attribute pipeline for rendering.

## The `Observable` trait is the publisher

The cross-crate surface is mediated by the `Observable` trait from [Ch 01 §00 core traits](../01-traits/00-core.md). `sim-bevy` consumes an `Observable` produced by whichever `Solver` backend is live — `SimSoftCpuObservable` for CPU runs, `SimSoftGpuObservable` for GPU runs — via the trait's associated `Step` type. The trait surface `sim-bevy` depends on is:

```rust
use sim_ml_chassis::Tensor;

pub trait Observable {
    type Step;
    fn stress_field(&self, step: &Self::Step) -> StressField;
    fn pressure_field(&self, step: &Self::Step) -> PressureField;
    fn temperature_field(&self, step: &Self::Step) -> TemperatureField;
    fn reward_breakdown(&self, step: &Self::Step, theta: &Tensor<f64>) -> RewardBreakdown;
}
```

`sim-bevy` uses `stress_field`, `pressure_field`, `temperature_field` for the per-vertex buffers; it does not consume `reward_breakdown` (that is [`sim-opt`](03-ml-chassis.md)'s field). The snapshot-assembly code runs inside `sim-soft::readout/`'s per-vertex projection pass and emits the `MeshSnapshot` struct at the end. `sim-bevy` sees the assembled snapshot, not the `Observable` trait directly — the trait stays a `sim-soft` implementation detail, and the coupling's public face is the snapshot struct alone.

## Double-buffered streaming — decoupled physics and render rates

Physics runs at variable rate — 1 ms in design mode, 16 ms in experience mode, with adaptive shrinks per [Part 5 Ch 02](../../50-time-integration/02-adaptive-dt.md). Render runs at fixed 60 Hz on display vsync. [Part 9 Ch 05 §00](../../90-visual/05-sim-bevy/00-mesh-streaming.md) pins the decoupling scheme: double-buffered `MeshSnapshot`, atomic writer-reader swap on physics-step completion. The physics thread writes into buffer $A$ while the renderer reads from $B$; the swap is a single atomic pointer exchange.

No locks on the fast path. The renderer always reads a consistent snapshot from a recent — possibly slightly stale — physics step. Staleness is bounded by one physics step: ≤ 1 ms in design mode, ≤ 16 ms in experience mode, well inside the 60 Hz frame budget. The snapshot upload to GPU (`wgpu::write_buffer`) is ≈2 MB for the canonical scene and completes async before the next render pass.

The coupling module this lives in is `sim-bevy::SimSoftPlugin` — a single Bevy plugin that registers the snapshot buffer as a Bevy resource, hosts the background physics-step task, and binds the 5-pass shader pipeline from [Part 9 Ch 05 §01 shader-pipeline](../../90-visual/05-sim-bevy/01-shader-pipeline.md) as a custom Bevy render-graph node. The user adds `SimSoftPlugin` to their `App`, drops a `SimSoftScene` component, and the rest is plugin-managed. `sim-soft` does not depend on Bevy; the dependency direction is `sim-bevy` → `sim-soft` only. Running `sim-soft` without Bevy is the default for the [Phase D "first working sim-soft" milestone](../03-build-order.md#the-committed-order) and the CI regression suites.

## Determinism-in-θ and the out-only discipline

The [`ForwardMap` γ-locked determinism-in-θ contract](../../100-optimization/00-forward.md) is preserved by construction: `sim-bevy` observes, does not perturb. `MeshSnapshot` is a readout of a converged Newton step, not an input to the next one. Re-rendering the same snapshot at a different frame rate, skipping frames under GPU load, or disabling the renderer entirely leaves `sim-soft`'s trajectory bit-identical. This is the architectural guarantee that lets optimization runs in `sim-opt` ignore the render budget — headless physics is the primary configuration, and the visual layer is a consumer of the same data that `sim-opt` consumes.

The snapshot's `timestamp` field is a monotonic physics-step counter, not a wall-clock timestamp, because wall-clock timestamps are non-deterministic. A rendered frame that missed its vsync is still reproducible by replaying the same snapshot; a missed wall-clock would leak non-determinism into anything downstream of the snapshot. [Part 9 Ch 05 §00](../../90-visual/05-sim-bevy/00-mesh-streaming.md) commits to step-counter timestamps for this reason.

If a future extension of `sim-bevy` introduces a back-channel — for example, a camera-driven LOD signal that reduces tet count in regions outside the viewport — the coupling surface would stop being out-only and would have to be explicitly renegotiated against [Part 10 Ch 00](../../100-optimization/00-forward.md). No such back-channel is in the Phase A–I roadmap; the out-only shape is firm.

## Optional `sim-thermostat` dependency

The snapshot's `temperature` field is populated only when [`sim-thermostat` coupling](01-thermostat.md) is live. Without it, the field is `None` (or a zero-filled placeholder — [Part 9 Ch 05 §00](../../90-visual/05-sim-bevy/00-mesh-streaming.md) commits to the representation). The thermal-overlay render pass per [Part 9 Ch 04 §01 overlays](../../90-visual/04-thermal-viz/01-overlays.md) checks for the field's presence and disables the overlay cleanly when absent. No error, no fallback cascade; the overlay simply does not render.

Similarly, `anisotropy_dir` is populated only when the material's `MaterialField` declares a `fiber_dir` ([Part 7 Ch 00 §04 MaterialField](../../70-sdf-pipeline/00-sdf-primitive.md)); isotropic materials leave it zeroed and the anisotropic-specular render pass per [Part 9 Ch 01 anisotropic-reflection](../../90-visual/01-anisotropic-reflection.md) falls back to isotropic GGX. The snapshot carries the union of all renderable channels; per-scene population is sparse, and the shader pipeline reads what is present.

## What the `sim-bevy` coupling does not carry

- **No physics feedback.** Camera pose, render-quality selection, draw-call budget — none of these feeds back into the simulation. `sim-bevy` reads; `sim-soft` does not see.
- **No custom shader authoring from `sim-soft`.** `sim-soft` produces per-vertex attributes; `sim-bevy` authors the shaders. Users who write their own Bevy `Material` impls bypass `SimSoftPlugin`'s shader pipeline (supported but not thesis-preserving — [Part 9 Ch 05 "What this does NOT commit"](../../90-visual/05-sim-bevy.md) spells out the tradeoff).
- **No VR / AR / offline render integration.** `sim-bevy-xr` and path-traced offline exports are defensible future crates; neither is in scope for this sub-leaf. `sim-bevy` is interactive Bevy only.
- **No mesh-topology change driven by the renderer.** LOD, view-frustum culling, silhouette-based tessellation — none of these are things `sim-bevy` does to the simulation mesh. The renderer subdivides for its own G-buffer pass per [Part 9 Ch 03 subdivision](../../90-visual/03-subdivision.md); the simulation mesh is fixed per timestep by the `Solver`.

## What this sub-leaf commits the crate to

- **The `sim-bevy` coupling is out-only, mediated by `MeshSnapshot`.** No state flows from the renderer back into the physics. Headless `sim-soft` runs identically with or without `sim-bevy` attached.
- **`MeshSnapshot` is the canonical cross-crate type.** Double-buffered, atomically swapped, `wgpu::write_buffer`-uploaded. [Part 9 Ch 05 §00](../../90-visual/05-sim-bevy/00-mesh-streaming.md) owns the representation; this sub-leaf commits the cross-crate surface.
- **The `Observable` trait from [Ch 01 §00](../01-traits/00-core.md) is the internal publisher.** `sim-bevy` consumes the assembled snapshot; `sim-soft::readout/` owns the per-vertex projection that produces it.
- **Determinism-in-θ is trivially preserved.** The renderer is a consumer; it cannot perturb the physics. [Part 10 Ch 00](../../100-optimization/00-forward.md)'s contract is unaffected.
- **`SimSoftPlugin` on the Bevy side is the integration surface.** [Part 9 Ch 05](../../90-visual/05-sim-bevy.md) pins the plugin API; this sub-leaf cites back.
- **Phase I is when this coupling closes.** Phases A–H run `sim-soft` headless (no shader preview, no Bevy app); [Phase I](../03-build-order.md#the-committed-order) is when the visual layer comes up.
