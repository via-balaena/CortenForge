# sim-bevy integration

This is the chapter where [Part 1 Ch 03's thesis](../10-physical/03-thesis.md) cashes out. The thesis commits to **the physics mesh being the render mesh**, with per-vertex physics quantities (stress, temperature, thickness, contact pressure, layer ID, anisotropy direction) as shader inputs, and no separate animation skeleton, skinning layer, or normal-map bakery. `sim-bevy` is the Bevy-side consumer of that architecture — the crate that receives deformed meshes and per-vertex attributes from `sim-soft` and drives the shaders from [Ch 00](00-sss.md), [Ch 01](01-anisotropic-reflection.md), [Ch 02](02-micro-wrinkles.md), [Ch 03](03-subdivision.md), and [Ch 04](04-thermal-viz.md) to produce the final rendered frame. This chapter names the streaming API, the shader pipeline, and the per-frame data flow.

| Section | What it covers |
|---|---|
| [Mesh streaming API](05-sim-bevy/00-mesh-streaming.md) | Double-buffered `MeshSnapshot` records pushed from `sim-soft` per physics step; Bevy reads the latest complete snapshot per rendered frame. Snapshot size for canonical scene (~30k DOFs + 7 per-vertex attributes) is ≈2 MB — fits in GPU upload budget per physics step |
| [Shader pipeline](05-sim-bevy/01-shader-pipeline.md) | The fragment pipeline: geometry pass (G-buffer with per-pixel normal, material, thickness, temperature, contact pressure, layer ID), SSS pass (diffuse-profile convolution), specular pass (anisotropic GGX), micro-wrinkle pass (normal perturbation from contact-pressure gradient), thermal overlay pass (optional), final composite |

Four claims this chapter commits to.

## 1. The shader inputs are per-vertex physics buffers, not baked textures

Standard game-engine shaders read material properties from *textures* — albedo map, normal map, roughness map, baked ambient occlusion, and so on, all authored at content-creation time and sampled via UV coordinates at render. `sim-soft` produces *per-vertex buffers* — stress, contact pressure, temperature, thickness, layer ID, anisotropy direction — that vary at runtime with the deformation and the contact state. The shader inputs are therefore the physics outputs, transmitted as dense vertex-attribute streams rather than texture maps.

```rust
pub struct MeshSnapshot {
    pub vertices: Vec<Vec3>,            // deformed positions, per-vertex
    pub normals: Vec<Vec3>,             // surface normals, per-vertex
    pub stress_per_vertex: Vec<f32>,    // von Mises stress, for diagnostic render
    pub temperature: Vec<f32>,          // from sim-thermostat
    pub thickness: Vec<f32>,            // SDF-distance to far surface, for SSS
    pub contact_pressure: Vec<f32>,     // per-vertex integrated contact pressure
    pub layer_id: Vec<u8>,              // outermost-layer ID per vertex; the full depth-ordered layer stack used by SSS ([Ch 00](00-sss.md)) is looked up from a precomputed per-scene table keyed on this ID
    pub anisotropy_dir: Vec<Vec3>,      // local anisotropy axis
    pub indices: Vec<u32>,              // triangle list indices (from surface extraction)
    pub timestamp: u64,                 // physics-step counter for snapshot versioning
}
```

Every field in the snapshot is driven by the physics. `thickness` is the SDF-distance from the surface vertex to the far side of the body (used by the SSS shader from [Ch 00](00-sss.md) to parameterize the scattering distance). `contact_pressure` is the per-vertex integrated pressure from [IPC's readout](../40-contact/01-ipc-internals/03-energy.md). `temperature` comes from [`sim-thermostat`'s coupling](../110-crate/02-coupling/01-thermostat.md). `anisotropy_dir` is the fiber direction from [Part 2 Ch 06](../20-materials/06-anisotropic.md) or the contact-flow direction for wet-surface shading.

UV coordinates appear in the snapshot (not shown above for brevity) only for shader inputs that the physics does not directly produce — specifically, the per-material diffusion profile lookup in SSS, where the profile depends on material ID and not on the deformation. The "no baked textures" commitment is about runtime-varying quantities; truly-static per-material parameters (like a diffusion profile curve) do live in texture-atlas form.

## 2. Double-buffered streaming decouples physics rate from render rate

The physics runs at a variable rate — 1 ms or 16 ms timesteps depending on design-mode vs experience-mode, with adaptive shrinks triggering irregular timing ([Part 5 Ch 02](../50-time-integration/02-adaptive-dt.md)). The renderer runs at a fixed 60 Hz on the display's vsync. The two cannot be tightly coupled — if rendering waited for each physics step to complete, the frame rate would stutter during adaptive shrinks.

`sim-bevy`'s solution: **double-buffer the MeshSnapshot, writer-reader pattern**. The physics thread writes into snapshot buffer $A$ while the renderer reads from buffer $B$; on physics-step completion, an atomic swap makes $A$ the reader's view and $B$ the new writer's view. No locks on the fast path; the renderer always sees a consistent snapshot from a recent (possibly slightly stale) physics step. Staleness is bounded by one physics step — at design-mode rates (1 ms steps), the render is at most 1 ms behind; at experience-mode rates (16 ms steps), the render is at most 16 ms behind, which is well inside the 60 Hz frame budget.

The per-snapshot upload to GPU is ≈2 MB for the canonical scene at design-mode resolution — in wgpu terms, a `write_buffer` call that completes async before the next frame's render pass runs. Even at the highest physics rates (design-mode 1 ms steps), the resulting upload bandwidth is well within PCIe 4.0 limits.

## 3. The shader pipeline is five passes, composed per frame

The fragment pipeline for a rendered frame proceeds as follows:

1. **Geometry (G-buffer) pass.** Rasterize the subdivided surface mesh into a G-buffer with per-pixel: world position, normal (interpolated from vertex normals and wrinkle-perturbed in the next pass), material ID, thickness (for SSS), temperature, contact pressure, layer ID, anisotropy direction (world-space after transforming per-vertex axes). This pass depends only on the current snapshot and the subdivision result.
2. **Micro-wrinkle normal-perturbation pass.** Read the G-buffer's contact pressure field, compute its spatial gradient (sobel or similar in screen space), perturb the normal in the direction perpendicular to the gradient. Per [Ch 02](02-micro-wrinkles.md). Write the perturbed normal back into the G-buffer.
3. **SSS diffuse-profile convolution pass.** Per [Ch 00](00-sss.md). Convolve the G-buffer's diffuse term with the material's diffusion profile using screen-space separable convolution. Sample per-pixel thickness and layer ID to parameterize the convolution width. One pass per layer in a multi-layer scene; typical 2-layer stack = 2 passes.
4. **Specular pass.** Anisotropic GGX shading per [Ch 01](01-anisotropic-reflection.md) using the perturbed normal, the anisotropy direction, and the material's $\alpha_x, \alpha_y$ roughness parameters. Combined with the SSS-diffused diffuse term for the full BRDF.
5. **Thermal overlay pass (optional).** Per [Ch 04](04-thermal-viz.md). Read temperature, apply color map, blend with the composited color per the user's overlay mode.

Each pass is a single compute or fragment kernel. The pipeline depth is 5 passes; the per-frame render cost on canonical scale is ≈8–12 ms on a consumer GPU at 1080p, which is compatible with interactive frame rates while leaving GPU headroom for the concurrent physics step. Higher resolutions or higher SSS quality trade off against that headroom. The Phase I deliverable does not pin a specific FPS target; Phase E's [≥30 FPS experience-mode commitment](../110-crate/03-build-order.md#the-committed-order) is the minimum, and Phase I aims higher as the visual layer matures.

## 4. The Bevy ECS integration is one `SimSoftPlugin`

`sim-bevy` provides a single Bevy plugin — `SimSoftPlugin` — that a user adds to their `App` and that handles the entire physics↔render bridge. The plugin:

- Registers the `MeshSnapshot` double-buffer as a Bevy resource.
- Registers a background task (Bevy's async task pool) that calls `sim-soft`'s step function and writes into the inactive snapshot buffer; on completion, atomically swaps which buffer is reader vs writer.
- Registers a system that runs per frame, reads the active snapshot buffer, uploads it to GPU via wgpu's `write_buffer`, and updates the entity's mesh handle to reference the new GPU data.
- Registers the 5-pass shader pipeline from §3 as a custom Bevy render graph node.
- Exposes a `SimSoftConfig` resource for the user to configure SSS tier, wrinkle enable, thermal overlay mode, and target physics timestep.

From the user's perspective, running a `sim-soft` scene in Bevy is:

```rust
use bevy::prelude::*;
use sim_bevy::SimSoftPlugin;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(SimSoftPlugin)
        .add_systems(Startup, spawn_scene)
        .run();
}

fn spawn_scene(mut commands: Commands, assets: Res<AssetServer>) {
    commands.spawn((
        SimSoftScene::from_cf_design_file("scene.cf"),
        SimSoftConfig::default(),
        Transform::default(),
    ));
}
```

The entire rest of the `sim-soft` ↔ Bevy pipeline is plugin-managed. From the `cf-design` side, authoring is the relevant UI; from the Bevy side, adding one plugin and one component is the relevant integration. This is what [Part 11 Ch 02 sub-chapter 02](../110-crate/02-coupling/02-bevy.md) specifies at the API level.

## What this commits downstream

- [Part 11 Ch 02 sub-chapter 02 (`sim-bevy` coupling)](../110-crate/02-coupling/02-bevy.md) specifies the full `SimSoftPlugin` API and the `MeshSnapshot` protocol.
- [Phase I of build-order](../110-crate/03-build-order.md#the-committed-order) is this chapter's machinery end-to-end on the canonical problem at 60 FPS, with SSS + anisotropic + wrinkle + (optional) thermal overlay composited per frame.
- [Part 11 Ch 04 (testing)](../110-crate/04-testing.md)'s [visual-regression suite](../110-crate/04-testing/02-visual.md) compares per-frame rendered output against golden images at Phase I.
- `readout/` ([Part 11 Ch 00 — readout sub-chapter](../110-crate/00-module-layout/09-readout.md)) produces the per-vertex attribute buffers the snapshot carries; any new attribute added to the snapshot requires a corresponding readout extension.

## What this does NOT commit

- **Custom `sim-bevy` shader overrides from user code.** Users can write their own `Material` implementations in Bevy and bypass the `SimSoftPlugin`'s shader pipeline if they want; the plugin provides the *default* pipeline. Bypass is supported but unsupported in the sense that the thesis's visual-physical fusion depends on the default pipeline; custom shaders can lose the fusion and silently re-introduce the games-physics failure modes.
- **VR / AR rendering.** Stereo rendering, head-tracked viewpoint, XR integration: not in Pass 1. A future `sim-bevy-xr` extension is defensible but not planned.
- **Offline-quality rendering.** The Monte-Carlo path tracer mentioned in [Ch 00](00-sss.md) is not shipped; users who want offline quality export the `MeshSnapshot` + materials to a dedicated path tracer.
- **Multi-scene composition in one render.** If a Bevy scene includes multiple `SimSoftScene` entities, each gets its own pipeline pass; compositing them into a single scene is a Bevy-render-graph concern, not a `sim-soft` architectural commitment.
