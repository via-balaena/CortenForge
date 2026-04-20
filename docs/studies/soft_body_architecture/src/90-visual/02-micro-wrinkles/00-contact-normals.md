# Normal maps from contact pressure

When silicone wraps tightly around a rigid feature — a probe rim, a sharp ridge, a finger joint — the surface develops micro-scale wrinkles much finer than the simulation mesh resolves. Resolving wrinkles as geometry would require a much denser mesh at substantially higher solve cost, and the Phase E budget does not accommodate that. `sim-soft`'s alternative is to **treat wrinkles as a shading phenomenon driven by the physics contact signal, not as a geometric phenomenon to be resolved in the mesh**. This sub-leaf specifies the technique: per-pixel normal perturbation from the spatial gradient of the contact-pressure field.

## The technique has two lineages

The mathematical structure — perturb the surface normal based on a scalar *bump field* without moving geometry — is [Blinn 1978](../../appendices/00-references/04-rendering.md)'s "Simulation of Wrinkled Surfaces," the founding bump-mapping paper. Every subsequent normal-map, parallax-map, and surface-detail technique in graphics descends from it. `sim-soft` reuses the Blinn construction directly; the full formula is in the normal-perturbation section below.

The novel part is the signal. Standard graphics practice is that the bump field $h$ is a texture authored offline by an artist or generated from a photograph; the runtime role is to sample it. `sim-soft`'s $h$ is a function of per-pixel contact pressure magnitude and its spatial gradient, both produced by the physics at runtime. This coupling — bump field as live physics output rather than authored texture — has no established precedent in the rendering literature as far as the book can determine. The closest structural parallel is [Bridson, Marino & Fedkiw 2003](../../appendices/00-references/04-rendering.md)'s "Simulation of Clothing with Folds and Wrinkles," which derives cloth wrinkles from compressive deformation in the solver rather than from authored textures. `sim-soft` differs in that the wrinkle reconstruction stays on the shader side of the physics-render boundary; Bridson et al. resolve the wrinkles in the cloth mesh's dynamics. The engineering rationale is the same — physics drives wrinkles, art does not — but the implementation lives in different layers of the stack.

## The bump field

The shader reads two quantities per pixel from the [`MeshSnapshot::contact_pressure` buffer](../05-sim-bevy.md): the pressure magnitude $p(\mathbf{x})$ and its screen-space gradient $\nabla p(\mathbf{x})$ (computed via a sobel-kernel sample of the neighboring fragments' contact-pressure values). Writing $\hat{\nabla p} = \nabla p / \|\nabla p\|$ for the unit compression direction, the bump field is constructed as

$$h(\mathbf{x}) = A(p)\, \sin\!\left( 2\pi\, f(p) \cdot (\hat{\nabla p} \cdot \mathbf{x}) \right)$$

where $A(p)$ is a pressure-dependent amplitude rising from zero at no contact to a material-specific maximum at saturation, and $f(p)$ is a pressure-dependent spatial frequency capturing the narrowing of wrinkle spacing under higher compression. Both $A$ and $f$ are parameterized by material-database entries; the shader does not hardcode them. The sinusoidal wave propagates along $\hat{\nabla p}$, so the level sets (wrinkle ridges and troughs) run perpendicular to it — the physically-observed geometry of compressive wrinkles, which form transverse to the compression direction (a rug pushed from one side bunches into ridges perpendicular to the push, not parallel).

```wgsl
fn contact_bump(
    pressure: f32,
    grad_p:   vec2<f32>,
    xy:       vec2<f32>,
    amp_max:  f32,            // material-database entry
    freq_max: f32,            // material-database entry
    p_ref:    f32,            // saturation reference (shared with wet shader)
) -> f32 {
    let t: f32 = smoothstep(0.0, p_ref, pressure);
    let dir_compress: vec2<f32> = normalize(grad_p + vec2<f32>(1e-6, 0.0));  // gradient-zero guard
    let phase: f32 = 2.0 * PI * (freq_max * t) * dot(dir_compress, xy);
    return (amp_max * t) * sin(phase);
}
```

When silicone presses around a rim, the compressive gradient points radially inward (from low-pressure bulk toward the high-pressure rim contact), and the resulting wrinkles run circumferentially — tangent to the rim boundary — matching the rug-bunching geometry above.

## Normal perturbation

The perturbed normal follows [Blinn 1978](../../appendices/00-references/04-rendering.md)'s bump-mapping construction: the perturbed normal is the unperturbed normal plus a tangent-plane displacement proportional to the surface gradient of $h$, with the specific form giving a displacement in the direction $(\mathbf{n} \times \mathbf{p}_u, \mathbf{n} \times \mathbf{p}_v)$ scaled by $(\partial h / \partial v, \partial h / \partial u)$ for surface parameterization $(u, v)$ and surface-tangent vectors $\mathbf{p}_u, \mathbf{p}_v$. The book does not reproduce the full signed formula since its sign conventions vary across derivations; `sim-soft`'s implementation follows the form in the original paper. In screen space the partial derivatives are finite-differenced from the neighboring fragments' $h$ values. The perturbed normal $\mathbf{n}'$ is then re-normalized and fed to the [§1 anisotropic GGX shader](../01-anisotropic-reflection/01-ggx.md) in place of the G-buffer's interpolated normal.

The perturbation magnitude is capped: the angular deviation between $\mathbf{n}$ and $\mathbf{n}'$ does not exceed a material-specific limit read from the material database. The cap prevents shading artifacts at silhouettes, where a large normal perturbation on a fragment whose surface is nearly edge-on to the camera produces visible shading glitches that the [§1 displacement](01-displacement.md) sub-leaf addresses more thoroughly.

## What this does not simulate

The shader reconstructs the *appearance* of wrinkles; it does not change where light actually hits. Two consequences follow.

First, silhouettes are unchanged. The fragment normal perturbation does not move the geometric boundary of the rendered body; if the coarse mesh is faceted at the rim, the faceted silhouette remains. Real silicone under tight contact has a wrinkled silhouette — the wrinkles *do* change the outline visible against the background. Normal-map bumps cannot fix this; [§1 displacement](01-displacement.md) is the fix, as a Phase I+ polish.

Second, cast shadows do not wrinkle. A wrinkle-displaced normal reflects more light, but the corresponding reduction in transmitted-through-depth light (for the [SSS layer](../00-sss.md)) is not modeled by the bump field. In practice this is invisible at the scales of interest — the wrinkles are much smaller than the SSS kernel width — but it is a class of effect the technique does not claim to capture.

## What this sub-leaf commits the book to

- **Wrinkles are reconstructed as shader-side bump fields driven by contact pressure.** The mathematical construction is [Blinn 1978](../../appendices/00-references/04-rendering.md) bump mapping; the novel part is the physics-coupled bump signal, which has no established precedent in published rendering literature.
- **Wrinkle ridges run perpendicular to the contact-pressure gradient.** The sinusoidal bump propagates along the gradient (the compression direction), so ridges align with the contact boundary — matching the observed geometry of compressive wrinkles.
- **Amplitude and frequency are material-parameterized, pressure-scaled.** All coefficients live in the appendix material database; the shader hardcodes none.
- **Silhouettes and cast shadows are not wrinkle-modulated.** The technique only modifies fragment shading; [§1 displacement mapping](01-displacement.md) addresses the silhouette case as a Phase I+ polish.
