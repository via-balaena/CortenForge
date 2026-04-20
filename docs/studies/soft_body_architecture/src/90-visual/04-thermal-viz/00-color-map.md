# Temperature → color mapping

`sim-soft`'s rendered temperature overlay is a per-pixel color lookup against a scalar temperature field. The pixel's temperature comes from the [`MeshSnapshot::temperature` buffer](../05-sim-bevy.md) (populated by [`sim-thermostat`'s coupling](../../110-crate/02-coupling/01-thermostat.md) on each physics step), interpolated across the triangle face by the rasterizer, and passed to the thermal shader as a scalar. The shader's job is to turn that scalar into an RGB color the viewer can read. This sub-leaf specifies the colormap choices, the value-range handling, and the gamma-correct interpolation commitment.

## The default is viridis

`sim-soft` ships four built-in colormaps and accepts user-supplied ones:

- **Viridis** (default) — van der Walt & Smith's perceptually-uniform colormap, introduced in the SciPy 2015 talk "A Better Default Colormap for Matplotlib" and included in matplotlib 1.5 (October 2015) as an option, then made the matplotlib default in 2.0 (January 2017). Its design has two separately-verifiable properties: **perceptual uniformity** (equal steps along the scalar axis produce equal visually-perceived steps) and **monotonic luminance** (each position along the colormap is brighter than earlier positions, so a grayscale conversion preserves the ordering). Combined, these make viridis the safe choice for any scientific visualization where the viewer might be color-blind, the print might be black-and-white, or the comparison between two temperatures has to be visually reliable rather than aesthetic.
- **Inferno** — the same design team's companion colormap, sharing the perceptually-uniform property. Higher contrast than viridis, with a darker low end that better emphasizes peak values; useful when the temperature field is dominated by a few hot spots and the low-temperature baseline is not of interest.
- **Plasma** and **Magma** — two additional matplotlib-companion colormaps from the same design team, both perceptually uniform. Plasma is warmer-biased (purple→orange→yellow); magma is darker across the range. `sim-soft` ships both for consistency with matplotlib practice but neither is the default.
- **Blue-cold / red-hot** (physical-intuition) — a classic two-ramp colormap that maps cold to blue, ambient to neutral gray, and hot to red. *Not* perceptually uniform — the blue-to-red midpoint has lower luminance than either end — but matches engineering intuition for thermal diagnostics and is familiar from infrared-camera displays. Use when the comparison is "is it hot or cold" rather than "how hot relative to other hot things."

User selection happens via `sim-bevy`'s material-override API per [Part 11 Ch 02 sub-chapter 02](../../110-crate/02-coupling/02-bevy.md); the default is viridis because scientific-communication practice is to default to a perceptually-uniform colormap unless a specific reason to deviate exists. The physical-intuition colormap is available but never the default.

Custom colormaps — supplied by the user as a 1D RGB lookup table — load through the same API; no distinction between built-in and user-supplied at the shader level.

## Value-range handling

The colormap maps a normalized scalar $t \in [0, 1]$ to an RGB value; the normalization from physical temperature to $t$ is the value-range step. Three modes:

- **Auto-fit** (default) — the renderer tracks per-frame min and max temperatures across visible fragments and rescales $t$ to $[0, 1]$ accordingly. Simple; responsive to the scene's thermal range; has the downside that the same numerical temperature can produce different colors between frames when the min/max shifts.
- **Fixed range** — the user sets $T_\text{min}, T_\text{max}$ explicitly, and out-of-range temperatures clamp to the colormap endpoints. Stable across frames; correct for comparison renders. The preferred mode for sequences of renders or video capture.
- **Material-scaled** — the range is tied to the per-material characteristic temperature ([Part 2 Ch 08 thermal-expansion reference $T_0$](../../20-materials/08-thermal-coupling/01-expansion.md)), with a material-specific span. Useful when comparing thermal behavior across materials without obscuring the per-material physics.

Value-range mode is a `SimSoftConfig` entry, changeable at runtime; switching between modes during a session does not require a mesh rebuild.

## Gamma-correct interpolation

The colormap lookup returns an sRGB-encoded RGB triple; the shader converts to linear RGB before the final composite so that the interpolation across pixel-to-pixel temperature variation respects perceptual uniformity. The alternative — interpolating in sRGB space directly — breaks viridis's perceptual-uniformity guarantee, because sRGB encoding is non-linear in luminance and equal sRGB steps do not correspond to equal perceptual steps.

The per-fragment calculation is:

```wgsl
fn thermal_color(t: f32, cmap: texture_1d<f32>) -> vec3<f32> {
    let t_clamped: f32  = clamp(t, 0.0, 1.0);
    let srgb: vec3<f32> = textureSample(cmap, samp_linear, t_clamped).rgb;
    return srgb_to_linear(srgb);
}
```

The composite pass (see [Ch 05 shader pipeline](../05-sim-bevy.md)) operates in linear RGB throughout and converts back to sRGB only at framebuffer write time; the thermal color joins that pipeline at the linear stage.

## What this sub-leaf commits the book to

- **Viridis is the default colormap** — perceptually uniform, colorblind-friendly, grayscale-legible. Correct choice unless a specific reason argues otherwise.
- **Four built-in colormaps ship** (viridis, inferno, plasma, magma, plus the non-perceptually-uniform blue-cold/red-hot). Custom colormaps load through the same API.
- **Auto-fit, fixed-range, and material-scaled are the three value-range modes.** Fixed-range is the preferred mode for comparison sequences; auto-fit is the default.
- **Color interpolation is gamma-correct.** sRGB-encoded colormap values convert to linear RGB at shader time; the composite runs in linear RGB throughout.
