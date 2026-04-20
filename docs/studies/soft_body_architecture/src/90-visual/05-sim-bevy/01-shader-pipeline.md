# Shader pipeline

The rendered frame is the output of a sequenced shader pipeline that reads the current [`MeshSnapshot`](00-mesh-streaming.md) and produces an sRGB framebuffer for display. Every pass in the pipeline has one job and consumes only the outputs of prior passes plus the snapshot itself; no pass reaches back into the physics or invalidates downstream passes' inputs. This sub-leaf specifies the pass order, per-pass inputs and outputs, the Bevy render-graph integration, and the cost budget that makes the full pipeline fit in the Phase I frame budget on a consumer GPU.

## The five passes

The pipeline runs five passes per frame in a strict order:

1. **Geometry (G-buffer) pass.** A standard rasterization pass using the deformed mesh from the snapshot. Outputs per-pixel: world position, surface normal (the interpolated per-vertex normal), material ID, thickness (for SSS), temperature, contact pressure, layer ID, anisotropy direction (world-space after transforming per-vertex axes), and the base-material diffuse color. This is a multi-target fragment pass writing into a G-buffer; the nine logical attributes are packed across a smaller number of multi-channel textures (e.g., a single RGBA texture holding thickness + temperature + contact pressure + layer ID) to fit the 8-simultaneous-render-target limit of most consumer GPUs. Standard deferred-rendering infrastructure with packing.
2. **Micro-wrinkle normal-perturbation pass.** Reads the G-buffer's contact-pressure channel, computes the per-pixel spatial gradient via a sobel sample of the neighbouring fragments, and perturbs the fragment normal per the [Ch 02 bump construction](../02-micro-wrinkles/00-contact-normals.md). Writes the perturbed normal back into the G-buffer's normal channel. Compute pass; fast because it reads one channel and writes one channel over the G-buffer's pixel count.
3. **SSS diffuse-profile convolution pass.** Per [Ch 00's real-time tier](../00-sss/02-realtime.md). Convolves the G-buffer's diffuse-shaded texture with the per-material diffusion profile in screen space; uses the [Jimenez 2015](../../appendices/00-references/04-rendering.md) separable form (two 1D passes, horizontal then vertical). For multi-layer scenes, the compound-variance kernel from [Ch 00 §0 stacking](../00-sss/00-stacked.md) is compile-time folded; per-pixel cost is unchanged from the single-layer case. Reads G-buffer; writes a diffused-diffuse texture.
4. **Specular pass.** Anisotropic GGX shading per [Ch 01](../01-anisotropic-reflection.md) using the perturbed normal (from pass 2), the per-pixel anisotropy direction (from G-buffer), and the per-material $\alpha_x, \alpha_y$ roughness parameters — with contact-pressure-driven modulation for [wet-surface regions](../01-anisotropic-reflection/00-wet.md). Combined with the SSS-diffused diffuse term from pass 3 into the full per-pixel BRDF response. Writes the composited-lighting texture.
5. **Thermal overlay pass (optional).** Per [Ch 04's overlay modes](../04-thermal-viz/01-overlays.md). Reads temperature, applies the colormap, composites with the prior pass's output per the user-selected overlay mode (`Off` skips this pass entirely, `Replace` overwrites, `Blend` mixes with user-configurable weight). Writes to the final-framebuffer texture.

Pass 5's output is linear RGB; Bevy's standard tonemap + gamma-correct post-processing (outside `sim-bevy`'s sub-graph) converts to sRGB and writes the swapchain framebuffer for display.

Each pass has one input dependency and one output target; passes are individually portable to a different pipeline order if a future rearrangement benefits. Pass 2 (micro-wrinkle) in particular can be disabled without affecting passes 1, 3, 4, 5 — the perturbed normal is a refinement of the interpolated normal, and the downstream passes simply read the interpolated normal if pass 2 has not run. This is the mechanism the [Ch 02 §0 debug-fallback](../02-micro-wrinkles/00-contact-normals.md) uses.

## Bevy render-graph integration

`sim-bevy` registers the pipeline as a **render-graph sub-graph** rooted in Bevy's main 3D render graph. The sub-graph has one node per pass, with explicit edges defining the read-write dependency order:

```rust
use bevy::render::{render_graph::RenderGraph, RenderApp};

fn register_sim_soft_subgraph(render_app: &mut RenderApp) {
    let mut graph = render_app.world_mut().resource_mut::<RenderGraph>();
    graph.add_node("sim_soft_geometry",   GeometryPassNode::default());
    graph.add_node("sim_soft_wrinkle",    WrinklePassNode::default());
    graph.add_node("sim_soft_sss",        SssPassNode::default());
    graph.add_node("sim_soft_specular",   SpecularPassNode::default());
    graph.add_node("sim_soft_thermal",    ThermalPassNode::default());
    graph.add_node_edge("sim_soft_geometry", "sim_soft_wrinkle");
    graph.add_node_edge("sim_soft_wrinkle",  "sim_soft_sss");
    graph.add_node_edge("sim_soft_sss",      "sim_soft_specular");
    graph.add_node_edge("sim_soft_specular", "sim_soft_thermal");
}
```

Each `*PassNode` implements Bevy's `Node` trait — one `run` method that dispatches the compute or fragment-shader work into the frame's command encoder. Standard Bevy render-graph practice; no `sim-soft`-specific deviation from how Bevy's own PBR pipeline wires up its passes. The significant difference is that the passes read `sim-soft`-side buffers (the GPU-resident per-vertex buffers from [§0 streaming](00-mesh-streaming.md)) rather than Bevy's own mesh resources.

## Budget allocation on the canonical scene

The Phase I aggregate budget is 8–12 ms per frame at 1080p on a consumer GPU. Per-pass breakdown is approximate:

| Pass | Work | Approximate cost |
|---|---|---|
| 1. G-buffer | Rasterize ~48k triangles, write packed G-buffer | ~1 ms |
| 2. Wrinkle | Compute pass over G-buffer pixels, sobel + normal perturbation | ~0.5 ms |
| 3. SSS | Two 1D separable convolutions, per-pixel cost dominated by tap count | ~2 ms |
| 4. Specular | Fragment pass with anisotropic GGX + SSS diffuse compositing | ~1 ms |
| 5. Thermal (optional) | Color-lookup composite; skipped unless enabled | ~0.3 ms |

Pipeline total is roughly 5 ms for the full pipeline with thermal off at 1080p, leaving headroom in the 8–12 ms budget to absorb `HighQuality` SSS (doubled tap count, roughly doubles pass 3's cost). Scaling up to 1440p or 4K multiplies the per-pixel passes by the pixel-count ratio; 4K at full quality would push past the Phase I budget and is addressed through per-pass quality-tier adjustments rather than by increasing the budget.

The budget is a target, not a measurement of `sim-soft` — Phase I benchmarks will calibrate these to actual consumer-GPU performance, and the breakdown above will be revised when real numbers are available. What this sub-leaf commits to is the *architectural shape*: a five-pass pipeline whose aggregate cost is dominated by the SSS convolution, with the other passes being sub-millisecond or low-single-digit-millisecond contributions.

## Quality tier integration

`SimSoftConfig::sss_quality` (introduced in [Ch 00 §2](../00-sss/02-realtime.md)) affects passes 2 and 3: `HighQuality` enables [Ch 02 §1 displacement mapping](../02-micro-wrinkles/01-displacement.md) (adds a compute pass between passes 1 and 2) and doubles the SSS tap count. `Production` default runs the pipeline above as specified. `Debug` disables pass 3 (SSS) entirely and replaces it with the [Ch 00 §2 tier-(a) normal-tinted fallback](../00-sss/02-realtime.md); pass 2 (wrinkle) can optionally be disabled in `Debug` too via a separate toggle.

`SimSoftConfig::thermal_mode` toggles pass 5; `Off` omits pass 5 entirely.

## What this sub-leaf commits the book to

- **Five-pass shader pipeline, strict-ordered.** Geometry, micro-wrinkle, SSS, specular, thermal. Each pass has one read-dependency and one write-target.
- **Bevy render-graph sub-graph with per-pass nodes.** Standard Bevy-render-graph practice; integrates with Bevy's own PBR and post-processing passes through the graph's node-edge mechanism.
- **SSS convolution dominates the budget**, but the aggregate fits inside the 8–12 ms Phase I frame target on a consumer GPU at 1080p.
- **Quality tiers from the SSS and thermal sub-chapters propagate here.** `HighQuality` bumps passes 2 and 3; `Debug` disables pass 3; thermal `Off` omits pass 5.
