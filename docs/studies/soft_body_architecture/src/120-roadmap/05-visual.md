# Milestone — visual layer

At the end of Phase I, `sim-soft` closes the thesis's visual half. A rendered silicone sleeve conforming to a rigid probe exhibits subsurface scattering with per-material diffusion profiles, anisotropic wet-surface reflection, contact-driven micro-wrinkles at the pressure boundary, thermal overlays showing dissipative heating, and subdivision-smoothed geometry reading directly off the Tet10 physics mesh. There is no skinning layer, no secondary-motion animation module, no normal-map bakery, and no post-hoc "soft-body thickness hack" — every visual cue traces back to a per-vertex or per-element attribute produced by the physics.

This is the milestone that closes the argument of [Part 1 Ch 03](../10-physical/03-thesis.md): visual and physical are not separate goals; the integrated solver produces the visual as a byproduct of producing the physics. Phase I is where that byproduct is rendered.

## Deliverables

- **Multi-layer subsurface scattering.** Per [Part 9 Ch 00](../90-visual/00-sss.md), a measured-diffusion-profile approach using real data from the material database. The `MaterialField` carries a `DiffusionProfile` per layer; the shader integrates it per-vertex at render time. Ecoflex 00-20 under-paint and Dragon Skin 10A over-paint look different because their measured profiles are different, not because a shader artist tuned them.
- **Anisotropic GGX + wet-surface BRDF.** Per [Part 9 Ch 01](../90-visual/01-anisotropic-reflection.md). Fiber-direction anisotropy from the `MaterialField` drives the specular lobe direction; wet-surface modulation lands on top.
- **Contact-driven micro-wrinkles.** Per [Part 9 Ch 02](../90-visual/02-micro-wrinkles.md). The contact-pressure field from `readout/` is the shader input for a displacement or normal perturbation; the perturbation exists only where and when the physics says contact is happening.
- **Subdivision surfaces over the physics mesh.** Per [Part 9 Ch 03](../90-visual/03-subdivision.md). The Tet10 physics mesh is the base mesh for Loop or Catmull-Clark subdivision; the render mesh has no vertices that were not derived from the physics mesh.
- **Thermal overlays.** Per [Part 9 Ch 04](../90-visual/04-thermal-viz.md). Temperature field from `sim-thermostat` maps to color via a calibrated colormap; overlays compose with the material rendering without replacing it.
- **`sim-bevy` integration.** Per [Part 9 Ch 05](../90-visual/05-sim-bevy.md). The full shader pipeline runs against the `MeshSnapshot` double-buffered streaming API; per-vertex attributes are the shader's only inputs from the physics side.

## What Phase I does not yet include

- **Per-print-coupon calibration of diffusion profile.** Per [Part 10 Ch 05](../100-optimization/05-sim-to-real.md), the `DiffusionProfile` used at render time is the material database's reference profile, not a per-print-batch calibrated profile. Per-batch calibration requires the post-Phase-I physical-print loop and is named in [Ch 07](07-open-questions.md).
- **Preference-learned visual weights from physical prints.** The preference GP from [Part 10 Ch 03](../100-optimization/03-preference.md) comes online at Phase I to learn reward-composition weights from *rendered* designs per [Part 10 Ch 06](../100-optimization/06-full-loop.md)'s Phase I stage. Preference learning over rendered-versus-physical parts remains a post-Phase-I item — again, [Ch 07](07-open-questions.md).
- **Ground-truth validation against a physical part.** The Phase I rendered preview is physically motivated — every shader input traces to a measured material property or a sim-produced physics attribute — but it has not been compared against a photograph of the equivalent printed part until the post-Phase-I sim-to-real loop closes. Phase I ships the *instrumentation* for that comparison; the comparison itself is off the roadmap.

## Exit criteria

Phase I closes when:

1. The canonical problem's rendered preview is *subjectively indistinguishable* at typical preview distances from a path-traced reference of the same configuration — the bar [Part 9 Ch 00](../90-visual/00-sss.md) sets for the SSS pipeline. Formalized for the close as a blinded pairwise-rating panel of domain-experienced raters against the path-traced reference; a side-by-side comparison against a physical-part photograph is a post-Phase-I check per [Ch 07](07-open-questions.md)'s physical-print loop, not a Phase I gate.
2. The full `sim-bevy` shader pipeline runs against both `sim-soft` and [Track 1B's `flex`](01-track-1b.md) output without backend-specific shader code paths. Any shader feature that only composes with `sim-soft`'s per-vertex attributes is flagged as a platform regression, not a feature.
3. The Phase E rate targets survive the Phase I shader load on the reference GPU. If subsurface-scattering rendering costs push design-mode below 5 Hz or experience-mode below 30 FPS, the shader is re-scoped before Phase I lands; the thesis's commitment to rendering at interactive rates is not reversed under deadline pressure.

Phase I is the book's last phase. Once it closes, the platform runs at its designed ceiling — `sim-soft` is fast, differentiable-except-through-SDF, SDF-authored, and visually ceiling-class. What remains after Phase I is the physical-print loop and the research frontier, both of which are [Ch 07](07-open-questions.md)'s concern.
