# Multi-layer subsurface scattering

Silicone, silicone composites with loaded particles, stacked-layer elastomer assemblies, and every other semi-translucent material the canonical problem encompasses look *wrong* with only surface shading. Light that enters a silicone body is not instantly reflected — it scatters through the volume, picks up the material's color characteristic, and exits at a different position on the surface. A silicone sleeve rendered with diffuse-only shading looks like a matte plastic sleeve; a silicone sleeve rendered with subsurface scattering looks like silicone. This is not a cosmetic preference; it is the visual signature of the material class `sim-soft` targets, and the [Ch 03 thesis](../10-physical/03-thesis.md)'s commitment to visual-and-physical-together requires SSS as first-class shading infrastructure, not a post-hoc filter.

| Section | What it covers |
|---|---|
| [Stacked translucent materials](00-sss/00-stacked.md) | Multi-layer skin-over-core, sleeve-over-sleeve, and gradient-material configurations. Each layer has its own scattering profile; the rendered result composes them per-pixel |
| [BSSRDF basics](00-sss/01-bssrdf.md) | The bidirectional scattering-surface reflectance distribution function — entry point $p_i$, exit point $p_o$, angular distribution. The formalism that anything below is approximating |
| [Real-time approximations](00-sss/02-realtime.md) | Three tiers: (a) normal-tinted baseline (cheap, wrong), (b) diffuse-profile convolution in screen space (Jimenez et al. 2015, production-grade), (c) multi-bounce Monte Carlo on the volume for offline-quality reference. `sim-soft` ships (b); (c) is for regression-checking |

Four claims this chapter commits to.

## 1. SSS is the visual signature of the target material class

The canonical problem is about silicone-class elastomers (Ecoflex, Dragon Skin) and their layered composites (carbon-black-loaded stiff skin over soft silicone core, for instance). Every one of these materials is translucent — light penetrates 0.5–5 mm into the body before scattering out, depending on the material and the wavelength. Surface-only shaders (Lambert, Blinn-Phong, Cook-Torrance BRDF) cannot represent this, by construction: the exit point for an incoming photon is assumed equal to the entry point, and the transport distance within the material is zero.

The visual consequence of getting this wrong is specific and immediate. Silicone with surface-only shading has **no depth** — the material reads as an opaque plastic of the same base color. Thin features (rims, edges) appear solid rather than slightly glowing; the material does not softly scatter light around sharp features; contacts read as hard interfaces rather than compliant pressing. A viewer who has handled silicone recognizes the surface-only render as wrong within a fraction of a second, even without naming why.

This is where the thesis's "the artifacts are physics, not cosmetics" claim applies at the *rendering* layer. Missing SSS is a rendering error with a physical cause — the material's scattering transport is not being simulated — and correcting it requires the same kind of physics-aware approach that the rest of the book commits to for the mechanical solver.

## 2. Diffuse-profile screen-space SSS is the Phase I commitment

Three real-time SSS techniques dominate the current literature:

- **Normal-tinted baseline.** A single pixel-shader evaluation reads the mesh normal, samples a "scatter color" at an offset along the normal, and blends with the surface color. Cheap (one extra texture tap), and wrong — it does not respect multi-layer structure, does not vary with feature size, does not depend on incoming light direction. The silicone-looks-like-plastic failure persists.
- **Diffuse-profile convolution in screen space.** Jimenez et al. 2015 ("Separable Subsurface Scattering") and predecessors. Each material has a measured *diffusion profile* — a radially-symmetric kernel describing how much of the incoming light at a point appears at surrounding surface points — and rendering applies this kernel as a screen-space convolution over the lit-diffuse framebuffer. Cost is one extra screen-space pass per material; quality is production-grade for static silicone-like materials.
- **Volumetric / multi-bounce Monte Carlo.** Offline path tracing through the material volume with measured scattering coefficients (absorption $\sigma_a$, scattering $\sigma_s$, anisotropy $g$). Ground-truth quality at hours-per-frame cost. Used for research-grade reference comparisons, not real-time.

`sim-soft`'s Phase I commits to **diffuse-profile screen-space SSS**, the middle tier. The cost fits the Phase I frame budget (≈2–4 ms per SSS-shaded material pass on a consumer GPU at 1080p), the quality is subjectively indistinguishable from path-traced silicone at thin-feature scales, and the technique has a decade of production deployment. `sim-bevy`'s [shader pipeline ([Ch 05](05-sim-bevy.md))](05-sim-bevy.md) dispatches the SSS pass after the G-buffer is populated and before the final composite; the SSS pass reads per-vertex thickness (interpolated to per-pixel) as a shader attribute from the physics.

Normal-tinted is kept available as a *debug* fallback — when the diffuse-profile pass is disabled for performance reasons or for a regression-test reference. Volumetric Monte Carlo is **not** shipped with `sim-soft`; users who need ground-truth reference renders export the mesh and materials to a dedicated path tracer.

## 3. Stacked layers compose via per-pixel blending, not per-pass

A silicone sleeve with a carbon-black-loaded skin has two translucent layers: the thin skin (≈1–3 mm, stiff, darker-tinted) and the softer silicone core underneath (≈5–20 mm, lighter, more scattering). Rendering these naively — one SSS pass for the outer surface, one SSS pass for the inner surface — produces visible ghosting at the interface and does not capture the cross-layer scattering that the viewer sees as "depth through the skin."

`sim-soft` handles this by **per-pixel layer composition**: each surface fragment carries a *material stack* attribute (an array of `LayerId`s with depth extents), and the SSS shader evaluates the diffusion profile across all layers intersected by the ray to the surface. This requires the physics mesh to carry per-tet layer identity, which it already does via the [`MaterialField`](../70-sdf-pipeline/00-sdf-primitive.md) infrastructure — layer identity is one of the fields passed from `cf-design` to `sim-soft` and streamed to `sim-bevy`. The shader-side cost is ≈2× the single-layer case for a 2-layer stack, not ≈N× a per-layer pass.

The [stacked-sub-chapter](00-sss/00-stacked.md) covers the composition math; the key spine-level commitment is that stacking is a shader-level concern, not a mesh-authoring concern, and that the number of layers is runtime-variable up to a small cap (4–8 layers, set by GPU texture-array depth for per-pixel lookup).

## 4. The measured diffusion profile is a material parameter, not a global setting

Every silicone variant (Ecoflex 00-10 vs Dragon Skin 30 vs carbon-loaded custom) has a *different* diffusion profile, and the profile is the visible identity of the material. Ecoflex 00-10 is more transparent than Dragon Skin 30; a render that uses the same profile for both cannot distinguish them. `sim-soft`'s `MaterialField` extends with a `DiffusionProfile` field — a 1D scattering-coefficient-vs-distance curve per color channel, typically measured via integrating-sphere spectrophotometry on a flat sample.

This places the diffusion profile alongside the other measured material parameters ([stiffness, Prony terms, thermal conductivity](../10-physical/04-material-data.md)) — a physical measurement that calibrates a specific render to the specific material the user chose in `cf-design`. The [material database in the appendices](../appendices/02-material-db.md) ships with profiles for the reference material set (Ecoflex 00-10, 00-20, 00-30, 00-50, Dragon Skin 10A, 20A, 30A, and the carbon-black composites).

Users authoring custom materials in `cf-design` supply their own profile if they have measurements, or fall back to a per-color-and-translucency-class default (which is visibly less accurate but structurally correct). The default-vs-measured distinction shows up in the render: measured profiles look like the specific material, defaults look like "a silicone-like something." Both are acceptable for design exploration; only measured is acceptable for the "matches the physical reference" validation that bridges to the sim-to-real loop in [Part 10 Ch 05](../100-optimization/05-sim-to-real.md).

## What this commits downstream

- [Ch 05 (sim-bevy integration)](05-sim-bevy.md)'s shader pipeline exposes the SSS pass; per-pixel thickness, per-pixel layer-stack, and per-pixel diffusion-profile lookup are shader attributes read from `sim-soft`-produced buffers.
- [Part 2 Ch 00 (material trait hierarchy)](../20-materials/00-trait-hierarchy.md) gets a non-mechanical field added to `MaterialField`: the `DiffusionProfile`. It does not feed the FEM solver; it feeds the renderer. The trait surface gains a new member without changing how the solver consumes the existing mechanical fields.
- [Part 10 Ch 05 (sim-to-real)](../100-optimization/05-sim-to-real.md) uses diffusion-profile comparison as part of the "does this render match the physical reference" validation loop.
- [Phase I of build-order](../110-crate/03-build-order.md#the-committed-order) lists Phase I's deliverable as the `sim-bevy` visual layer reaching the ceiling defined in [Ch 01 of Part 0](../00-context/01-ceiling.md); SSS is a named requirement within that.
- The appendix material database expands to include measured diffusion profiles alongside mechanical data.

## What this does NOT commit

- **Exact convolution-kernel parameters.** The diffusion-profile sub-chapter specifies the curve shape and the per-color decomposition, but the specific separable-filter kernel `sim-bevy` uses is Phase I implementation work.
- **Photon-traced reference renderer.** Not shipped in `sim-soft`; out-of-scope.
- **Dynamic SSS profile changes.** Material profile is static per `MaterialField`; time-varying scattering (e.g., silicone that darkens when stretched) is not modeled in Pass 1. Could be added in a future phase if a use case demands it.
