# Anisotropic GGX

`sim-soft`'s specular shader is anisotropic GGX — [Walter et al. 2007](../../appendices/00-references/04-rendering.md)'s microfacet normal-distribution function, extended to the anisotropic form with per-tangent-axis roughness $(\alpha_x, \alpha_y)$ introduced in [Burley 2012](../../appendices/00-references/04-rendering.md)'s Disney BRDF course notes. The choice is not ambitious — GGX is the industry default across Unreal, Unity, Godot, Blender Cycles/Eevee, and Disney's own production renderers — and the point of this sub-leaf is not to defend it but to specify how `sim-soft` parameterizes it and where the anisotropy signal comes from.

## The GGX distribution

The isotropic GGX normal-distribution function from [Walter 2007](../../appendices/00-references/04-rendering.md) is

$$D(\mathbf{m}) = \frac{\alpha^2}{\pi\left((\mathbf{n} \cdot \mathbf{m})^2 (\alpha^2 - 1) + 1\right)^2}$$

for microfacet normal $\mathbf{m}$, macro-surface normal $\mathbf{n}$, and roughness parameter $\alpha$. Key properties the paper proves: the distribution integrates to one over the upper hemisphere; its tail falls off as a power law rather than a Gaussian, which matches measured microfacet distributions on real surfaces better than the Beckmann distribution GGX superseded; and the formula is cheap to evaluate and to importance-sample.

The anisotropic extension from [Burley 2012](../../appendices/00-references/04-rendering.md) replaces the single $\alpha$ with two roughnesses $\alpha_x, \alpha_y$ along per-vertex tangent axes $\mathbf{t}_x, \mathbf{t}_y$:

$$D_\text{aniso}(\mathbf{m}) = \frac{1}{\pi \alpha_x \alpha_y} \cdot \frac{1}{\left(\frac{(\mathbf{t}_x \cdot \mathbf{m})^2}{\alpha_x^2} + \frac{(\mathbf{t}_y \cdot \mathbf{m})^2}{\alpha_y^2} + (\mathbf{n} \cdot \mathbf{m})^2\right)^2}$$

When $\alpha_x = \alpha_y = \alpha$ the expression reduces to the isotropic GGX above. The production parameterization Burley 2012 ships pairs a scalar perceptual roughness $r \in [0, 1]$ with a scalar anisotropy $a \in [0, 1]$ and produces $(\alpha_x, \alpha_y)$ as $\alpha_x = r^2 / \text{aspect}$ and $\alpha_y = r^2 \cdot \text{aspect}$, with $\text{aspect} = \sqrt{1 - 0.9 a}$. The $0.9$ is Burley's empirical cap: pure anisotropy ($a = 1$) would give $\alpha_y = 0$, which is singular; the cap keeps $\alpha_y > 0$ over the full $a \in [0, 1]$ parameter range.

## The tangent axes come from the physics

Industry practice for anisotropic BRDFs is that the local tangent axes $\mathbf{t}_x, \mathbf{t}_y$ are baked into the mesh at authoring time via UV directions or per-vertex tangent-vector attributes painted by an artist. `sim-soft`'s tangent axes are runtime outputs of the physics. Three sources populate them, selected per-vertex by whichever physics signals are active at that vertex:

- **Fiber direction** — for fiber-reinforced hyperelastic materials ([Part 2 Ch 06](../../20-materials/06-anisotropic.md)), the reference-frame fiber direction $\mathbf{a}$ pushes forward to the deformed frame via the deformation gradient, and the shader receives $\mathbf{t}_x = F \mathbf{a} / \|F \mathbf{a}\|$ as its principal tangent axis. $\mathbf{t}_y$ is constructed in the tangent plane perpendicular to $\mathbf{t}_x$.
- **Contact-flow direction** — for the wet-surface case from [§0](00-wet.md), the anisotropy axis is populated from the in-plane component of the contact-pressure gradient, so the specular highlight streams along the flow direction. The specific mechanism (asperity flattening, interfacial reflectance, or other) is not decomposed; the shader reconstructs the observed directional sheen from the contact-flow signal the physics already produces.
- **Molded-flow direction** — a `cf-design` authoring input for surfaces that record their mold orientation (injection-molded parts, extruded profiles). The designer supplies a surface flow field in `cf-design`; `sim-soft` passes it through to the shader without modification.

The streaming mechanism is the [`MeshSnapshot::anisotropy_dir` attribute](../05-sim-bevy.md) populated per surface vertex. A single shader attribute carries the axis regardless of its physical origin; the shader is source-agnostic. When no anisotropy source applies, the attribute carries the zero vector and the shader falls back to isotropic GGX (single $\alpha$ = mean of $\alpha_x, \alpha_y$) for that vertex.

## Why GGX, and why not Ashikhmin-Shirley 2000

The alternative anisotropic BRDF `sim-soft` evaluated and rejected is [Ashikhmin & Shirley 2000](../../appendices/00-references/04-rendering.md)'s "An Anisotropic Phong BRDF Model" from the Journal of Graphics Tools. It is anisotropic, energy-conserving, reciprocal, and cheaper than GGX — attractive properties. It is rejected for three reasons.

First, it is not microfacet-derived; it is an empirical extension of the Phong distribution. Microfacet BRDFs compose with the Smith masking-shadowing term $G$ for self-occlusion and with the Fresnel term $F$ for per-angle reflectance in a way that is theoretically clean and ubiquitously implemented — every production engine has the machinery. The Ashikhmin-Shirley model does not decompose this way, so integrating it into an existing pipeline means parallel machinery for masking and shadowing, or accepting artifacts.

Second, the tail behavior of the Phong distribution does not match measured microfacet data at high roughness, which is where silicone spends most of its time — the dry-surface baseline for typical soft silicones sits at moderately-high $\alpha$, and the [§0 wet-surface shader](00-wet.md)'s modulation only narrows it transiently under contact. GGX's power-law tail fits measured data better in this regime; it was Walter 2007's original argument.

Third, the industry ecosystem is GGX-native. Importing Ashikhmin-Shirley means forfeiting compatibility with the [`disney/brdf`](https://github.com/disney/brdf) reference implementation, the Disney principled-BRDF downstream consumers, and the large library of measured-material fits published in GGX form. For a platform that expects to be re-targetable onto downstream renderers for [offline reference workflows](../00-sss/02-realtime.md), GGX-native is the compatibility choice.

The cost comparison is not load-bearing — GGX is not much more expensive per pixel than Ashikhmin-Shirley in practice — but the ecosystem argument is decisive.

## Smith masking-shadowing and importance sampling

[Walter 2007](../../appendices/00-references/04-rendering.md) pairs GGX with the Smith form of the masking-shadowing function $G(v, m)$; [Burley 2012](../../appendices/00-references/04-rendering.md) extends $G$ to the anisotropic case via the same $(\alpha_x, \alpha_y)$-along-tangent-axes parameterization. `sim-soft`'s shader inherits both — the anisotropic Smith $G$ paired with the anisotropic GGX $D$. The energy-conserving BRDF is the standard microfacet product $f_r = (D \cdot G \cdot F) / (4\,(\mathbf{n} \cdot \mathbf{v})\,(\mathbf{n} \cdot \mathbf{l}))$ with the per-microfacet Fresnel term $F$; $G$ models the mutual self-occlusion of microfacets, without which the $D$ distribution alone would over-reflect at grazing angles.

Importance sampling of anisotropic GGX for the [offline reference path](../00-sss/02-realtime.md) uses the standard mapping from uniform $(u_1, u_2) \in [0, 1]^2$ to microfacet normal $\mathbf{m}$ published in Walter 2007 for isotropic GGX and extended to the anisotropic case in Burley 2012; `sim-soft` does not ship a path tracer but the offline reference workflow requires the importance sampler, so the mapping is implemented behind the same tangent-axis infrastructure.

## What this sub-leaf commits the book to

- **Anisotropic GGX ([Walter 2007](../../appendices/00-references/04-rendering.md) distribution + [Burley 2012](../../appendices/00-references/04-rendering.md) anisotropic parameterization) is the `sim-soft` specular shader.** Industry default; re-targetable to every major production renderer.
- **Tangent axes come from physics at runtime, not UV-painted at authoring time.** Fiber direction, contact-flow direction, or `cf-design` molded-flow direction — selected per vertex by the material's anisotropy class and streamed via [`MeshSnapshot::anisotropy_dir`](../05-sim-bevy.md).
- **Smith masking-shadowing is paired with the $D$ distribution.** Anisotropic form per Burley 2012; required for energy conservation.
- **Ashikhmin-Shirley 2000 is the considered-and-rejected alternative.** Cheaper, not microfacet-derived; the ecosystem compatibility loss outweighs the cost saving.
