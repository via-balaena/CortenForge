# Stacked translucent materials

A single-material silicone body has one diffusion profile; a realistic soft-robotics assembly has several stacked together. The canonical-problem geometry includes configurations like a carbon-black-loaded stiff skin bonded to a softer silicone core, a thin sleeve sliding over a thicker liner, or a gradient material that varies from stiff near the probe contact surface to compliant at the far side. All three are cases where a photon entering through one layer scatters through material A, crosses the inter-layer interface into material B, and exits at a surface point whose visible color depends on the full stack it traversed. This sub-leaf covers how `sim-bevy`'s SSS shader pipeline composes those stacks — per-pixel, at render time, rather than per-pass in multiple render targets.

## The shader-side layer-stack attribute

Each surface fragment carries a depth-ordered list of layer IDs the viewing ray traverses from entry to exit. Concretely, the fragment's attribute bundle carries:

```rust
pub struct LayerStack {
    pub depths:  [f32; MAX_LAYERS],     // entry depth per layer (mm, from surface inward)
    pub layer_ids: [u8; MAX_LAYERS],    // lookup into per-scene profile table
    pub count:   u8,                    // active entries (1..=MAX_LAYERS)
}

pub const MAX_LAYERS: usize = 8;
```

`MAX_LAYERS` is set to 8 because the shader's per-pixel SSS pass iterates the stack entries at fixed loop bound: the cost is proportional to `MAX_LAYERS`, not to the per-pixel `count`, so raising the cap past the realistic design envelope pays inner-loop cost with no visual payoff. Realistic silicone-composite designs — skin-over-core, sleeve-over-liner — max out at two or three layers; the headroom above that is for assemblies that stack several thin layers and for future extensions not on the Phase A–I roadmap.

The `depths` array is computed per-vertex by `sim-soft` as the ray enters each distinct layer along the interpolated surface normal; the shader interpolates across the triangle and receives a per-pixel depth breakdown. For single-layer bodies (`count == 1`), the shader degenerates to the standard non-stacked SSS path described in [§2 real-time approximations](02-realtime.md).

## How layer identity is assigned per tet

Layer identity is an explicit field on [`MaterialField`](../../70-sdf-pipeline/00-sdf-primitive.md) — an optional `layer_id: Option<Box<dyn Field<Output = u8>>>` that `cf-design` populates when the designer authors a multi-material body with sharp SDF-based interfaces between materials. [Part 7 Ch 02's material-assignment sub-chapter](../../70-sdf-pipeline/02-material-assignment.md) samples the field at per-tet centroid (Tet4) or per-Gauss-point (Tet10); each tet receives its `u8` layer identifier alongside the scalar material parameters.

Bodies whose `layer_id` is `None` — single-material silicones, smoothly-varying gradient designs without explicit layer labels — render as single-layer SSS, using the body's dominant material profile. Multi-layer rendering for gradient bodies (discretizing a smooth stiffness variation into a discrete layer stack for visual purposes) is not on the Phase A–I roadmap; designers who want it supply explicit `layer_id` from `cf-design`. The sharp-interface-with-explicit-`layer_id` case is the supported multi-layer path.

The layer ID and per-layer interface depths flow to the renderer through `sim-soft`'s [`readout/`](../../110-crate/00-module-layout/09-readout.md) crate: each surface vertex carries its outermost-layer ID (as the `u8` in [`MeshSnapshot::layer_id`](../05-sim-bevy.md)) plus a depth-ordered list of interface crossings along the inward normal, computed from the per-tet `layer_id` field walked along each surface vertex's inward ray. The shader interpolates these per-vertex attributes across the triangle; no ray-marching at shader time, and no SDF evaluation in the fragment program.

## Profile composition across layers

For a two-layer stack with layer thicknesses $\ell_1, \ell_2$ and diffusion profiles $R_{d,1}(r), R_{d,2}(r)$, the per-pixel effective profile is the 2D convolution of the two profiles on the planar-surface approximation:

$$R_{d,\text{stack}} = R_{d,1} * R_{d,2}$$

A photon entering at radial offset zero scatters to radial offset $r_1$ during its traversal of layer 1 and then to $r_1 + r_2$ (as a 2D vector sum) after layer 2; integrating over the intermediate $r_1$ gives the convolution above. Closed-form results exist when each $R_d$ is a sum of radially-symmetric Gaussians — the parameterization [d'Eon & Luebke 2007](../../appendices/00-references/04-rendering.md#deon-luebke-2007) introduced and [Jimenez et al. 2009](../../appendices/00-references/04-rendering.md#jimenez-2009) inherited for screen-space SSS — because Gaussian convolution is another Gaussian with $\sigma^2_\text{out} = \sigma^2_1 + \sigma^2_2$. The [Christensen & Burley 2015](../../appendices/00-references/04-rendering.md#christensen-burley-2015) sum-of-exponentials form is more accurate for single-layer silicone but does not admit the same clean variance-addition rule under convolution.

`sim-soft`'s `DiffusionProfile` therefore carries two parameterizations: the default single-layer form is the accurate Christensen-Burley sum-of-exponentials (per [§1 BSSRDF](01-bssrdf.md)); when the scene has `layer_id != None` on the `MaterialField`, the renderer refits each profile to a sum-of-Gaussians approximation at scene-ingest time so the multi-layer compound can use variance addition. The refit introduces a small per-channel tail-accuracy hit for typical silicone-class parameters — judged worth it to keep multi-layer rendering fast and quantified against per-scene ground-truth reference renders at [visual-regression time](../../110-crate/04-testing/02-visual.md).

`sim-bevy`'s shader uses the compounded kernel at shader-compile time rather than integrating per-pixel; compounding the per-layer profiles is compile-time constant-folding and adds no per-pixel cost. For stacks of three or more layers, the compounding is associative — compound the first two, then compound the result with the third, and so on — so shader cost grows as $\mathcal{O}(N_\text{layer})$ in *compile-time* work, not per-pixel work. The per-pixel cost is still one separable-kernel convolution pass (per [Jimenez et al. 2015](../../appendices/00-references/04-rendering.md#jimenez-2015)); the stacking is absorbed into the kernel's compound variance.

## Why per-pixel, not per-pass

An alternative architecture renders each layer as its own SSS pass, then composites the results. This is what some older games-industry pipelines do. It is rejected here for three reasons.

First, per-pass over the separable screen-space convolution loses the $\mathcal{O}(N_\text{layer}) \to \mathcal{O}(1)$ compile-time fold above: each pass pays its full per-pixel convolution cost regardless. A 2-layer stack is 2× cost, a 4-layer stack is 4×, and so on. The per-pixel compound-variance approach stays at 1× per-pixel for any layer count.

Second, per-pass produces visible ghosting at layer interfaces when the per-layer passes use different kernel widths (as they must, since different materials scatter differently). The ghosting is fixable with alignment heuristics but is an ongoing source of shader bugs. The per-pixel-compound approach has no interface, by construction.

Third, per-pass requires extra framebuffer storage proportional to layer count. The per-pixel-compound approach uses the single G-buffer from [Ch 05's shader pipeline](../05-sim-bevy.md) regardless of layer count.

## Cost summary for the canonical scene

On the canonical scene (30k-tet design-mode resolution, 1080p render, 2-layer silicone-skin-over-core configuration), the stacked-SSS pass costs ≈2 ms on a consumer GPU at Phase I — dominated by the single screen-space convolution pass, not by the per-layer setup. Degrading to non-stacked (one layer) saves the layer-stack attribute-array bandwidth but does not change the convolution cost. Scaling to denser layer stacks (rare in realistic designs) adds only the compile-time-folded variance-term arithmetic to the final kernel and leaves per-pixel cost unchanged.

The [Ch 05 shader-pipeline frame budget](../05-sim-bevy.md) allocates 8–12 ms total to the render passes at 1080p; 2 ms of that for stacked SSS leaves headroom for the [anisotropic specular pass](../01-anisotropic-reflection.md), [micro-wrinkle normal perturbation](../02-micro-wrinkles.md), and [thermal overlay](../04-thermal-viz.md) to run concurrently.

## What this sub-leaf commits the book to

- **Layer identity is an explicit optional field `layer_id` on [`MaterialField`](../../70-sdf-pipeline/00-sdf-primitive.md).** Populated by `cf-design` for sharp-interface multi-material bodies; `None` for single-material or gradient bodies, which render as single-layer SSS.
- **Per-pixel layer-stack composition is the architecture.** Each fragment carries a depth-ordered `LayerStack` attribute; the shader does one convolution pass with a compound kernel folded at compile time.
- **Up to 8 layers supported, two to three typical.** Cap is shader-loop cost on consumer hardware, not a physics limit; realistic silicone-composite designs sit well inside.
- **Multi-layer runs use a sum-of-Gaussians `DiffusionProfile` refit, compound via variance addition.** Single-layer runs keep the more accurate Christensen-Burley sum-of-exponentials form per [§1](01-bssrdf.md); the refit is a scene-ingest step that trades a small tail-accuracy loss for closed-form layer compounding.
