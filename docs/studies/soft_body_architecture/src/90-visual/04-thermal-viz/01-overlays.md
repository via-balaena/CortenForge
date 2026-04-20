# Thermal overlays

The [§0 color map](00-color-map.md) turns a per-pixel temperature scalar into an RGB color, but does not specify how that color composites with the rest of the rendered frame. Two overlay modes cover the realistic use cases: a diagnostic mode where the thermal color *replaces* the body's material appearance, and a physical-hint mode where it *blends* with the material's normal appearance to convey the temperature signal visually. This sub-leaf specifies both modes, the user-selection path, and the autograd-boundary commitment that makes thermal visualization strictly a rendering concern with no feedback into physics or gradients.

## Replace mode (diagnostic)

In replace mode, the thermal color is the pixel's final color — the body's diffuse, specular, SSS, wet-look, and micro-wrinkle contributions are all dropped, and the only thing rendered for the body's pixels is the colormap output. The silhouette, shading, and normal all survive (the pixel is still correctly shaded against the camera), but the *color* is purely thermal.

The use case is thermal diagnostics — the designer cares about temperature distribution and wants the material appearance out of the way so that hot spots and cold spots are visually obvious. Replace mode is what thermal imaging cameras show (pseudo-color on an IR measurement); it is also what thermodynamic-computing experiments with [`sim-thermostat`](../../110-crate/02-coupling/01-thermostat.md)'s temperature field want when the research question is about the thermal dynamics, not about how the material looks.

```wgsl
fn composite_thermal_replace(
    thermal_rgb: vec3<f32>,
    shading_factor: f32,            // from the lighting pass
) -> vec3<f32> {
    return thermal_rgb * shading_factor;      // no material color enters
}
```

## Blend mode (physical-motivated)

In blend mode, the thermal color mixes with the body's rendered material appearance at a user-configurable weight — kept small to preserve the material's visual identity while still carrying the temperature signal, peaking at the warmest or coldest temperatures and falling to zero at ambient. The physical motivation is that warm silicone can show a subtle visual appearance shift against cool silicone; the specific mechanism is not decomposed, and the blend mode captures the effect *visually* without claiming to derive it from a physical model. Unlike the [§1 wet-surface shader](../01-anisotropic-reflection/00-wet.md), which is driven by a specific physics signal (contact pressure), blend mode is a designer hint: the thermal field hints at a temperature-dependent appearance shift, and the magnitude is a user setting rather than a measured material parameter.

```wgsl
fn composite_thermal_blend(
    material_rgb: vec3<f32>,
    thermal_rgb:  vec3<f32>,
    t_normalized: f32,              // temperature, normalized
    blend_max:    f32,              // user-configurable, typically small
) -> vec3<f32> {
    let t_offset: f32 = abs(t_normalized - 0.5) * 2.0;   // peaks at temperature extremes
    let w:        f32 = blend_max * t_offset;
    return mix(material_rgb, thermal_rgb, w);
}
```

The blend weight peaks at temperature extremes (far from ambient) and falls to zero at the ambient midpoint, so a thermally-uniform body at ambient renders identically with or without blend mode active. Only thermally-active regions show the shift.

## User selection and runtime switching

The overlay mode is a `SimSoftConfig::thermal_mode` entry with values `Off`, `Replace`, and `Blend`. `Off` is the default — the rendered frame does not invoke the thermal shader at all, no colormap lookup, no composite, no cost. `Replace` and `Blend` trigger the thermal pass; the user can switch between modes at runtime via the `sim-bevy` material-override API per [Part 11 Ch 02 sub-chapter 02](../../110-crate/02-coupling/02-bevy.md), with the next frame picking up the new mode.

For the [thermodynamic-computing experiments](../../20-materials/08-thermal-coupling.md) that make the canonical cavity+probe scene thermally interesting, the typical workflow is:

- `Off` during early mechanical-design sweeps (thermal is irrelevant)
- `Replace` when diagnosing a specific thermal hypothesis (visualize the temperature field cleanly)
- `Blend` for final renders that demonstrate the coupled mechanical+thermal behavior (thermal appears as a subtle modulation of the realistic material render)

No frame switches modes automatically based on physics; the user selects based on the current question.

## Thermal rendering does not enter the physics or the autograd tape

The thermal shader reads `MeshSnapshot::temperature` and produces pixels. It does not write to any physics buffer, does not modify the temperature field, and does not participate in the autograd tape for gradient flow. This matches the [Part 1 Ch 03 thesis](../../10-physical/03-thesis.md) commitment that visual rendering is a downstream consumer of the physics, never a feedback into it; the temperature field's dynamics come from `sim-thermostat`'s coupling to the mechanical state per [Part 2 Ch 08](../../20-materials/08-thermal-coupling.md) and its own internal dynamics, and the renderer observes but does not affect them.

For optimization workflows that want to include thermal considerations in the [Part 10 Ch 00 forward map](../../100-optimization/00-forward.md), the thermal signal has to enter the reward on the physics side (e.g., via a temperature-component addition to the reward composition from [Part 1 Ch 01](../../10-physical/01-reward.md)), not through the visual overlay. The rendered-pixel loss used in the [Part 10 Ch 03 preference-learning](../../100-optimization/03-preference.md) loop bypasses the thermal overlay by construction — preference ratings are typically on the mechanical appearance (conformity, wet-look, wrinkle realism), not on the thermal overlay mode.

## What this sub-leaf commits the book to

- **Two overlay modes**, `Replace` (diagnostic, thermal color replaces material) and `Blend` (physical hint, thermal tints the material), selected per-render by the user via the `sim-bevy` material-override API. `Off` is the default.
- **Blend weight peaks at temperature extremes and falls to zero at ambient.** Matches the physical intuition that thermally-neutral regions should look the same regardless of mode.
- **Thermal rendering does not affect physics or autograd.** The shader reads but does not write; thermal gradients enter optimization via physics-side reward terms, not visual-loss paths.
- **Diagnostic and research workflows select different modes.** Off for mechanical-only work, Replace for thermal diagnostics, Blend for coupled-behavior visualizations.
