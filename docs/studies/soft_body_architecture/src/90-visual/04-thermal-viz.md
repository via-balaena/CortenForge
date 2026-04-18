# Thermal visualization

`sim-soft` integrates with [`sim-thermostat`](../110-crate/02-coupling/01-thermostat.md) to handle temperature fields on the mesh; the thermal field is a first-class per-tet attribute alongside stress and contact pressure. Rendering the temperature field — as a color overlay, as a shader-input for thermally-variable material response, as a diagnostic for thermo-dynamic-computing experiments — is Part 9's responsibility. The machinery is simple.

| Section | What it covers |
|---|---|
| [Temperature → color mapping](04-thermal-viz/00-color-map.md) | Color maps (viridis default, inferno for high-contrast, user-selectable), value ranges auto-fit or user-set, gamma-correct interpolation. Per-pixel temperature lookup from the physics mesh |
| [Thermal overlays](04-thermal-viz/01-overlays.md) | Two overlay modes: (a) replace the material color with the thermal color (diagnostic, for pure-temperature visualization), (b) blend the thermal color with the material's normal appearance (physically-motivated — warm silicone slightly shifts its surface characteristics) |

Two claims.

**The color map is viridis by default, user-configurable.** Viridis is perceptually-uniform, colorblind-friendly, and prints legibly in grayscale — matplotlib's default since 2017 and widely adopted across scientific-visualization tooling. `sim-soft` ships viridis plus inferno (higher-contrast, for visualization publications), plasma (warm-biased), and a custom "blue-cold / red-hot" physical-intuition map. The user selects via `sim-bevy`'s [material override API](../110-crate/02-coupling/02-bevy.md) per-render; the default is viridis because scientific-communication defaults to it.

**Thermal rendering does not modify the physics.** Like subdivision, thermal overlay is a post-physics render-only step. The temperature field is read from the `MeshAttributeField<f64>` buffer that `sim-thermostat` populates; the shader samples it per-pixel and applies the chosen color map. The shader does *not* write back to the temperature buffer — there is no feedback from the render into the physics. This matters for the autograd tape: thermal visualization does not enter the backward pass; gradients flow through the physics and end at the reward terms, not through the rendered pixels.
