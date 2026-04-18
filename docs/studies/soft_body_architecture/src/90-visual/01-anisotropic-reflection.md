# Anisotropic reflection

Real silicone surfaces are rarely isotropically rough. A wet silicone surface has a directional specular highlight that streams along the direction of wetting. A fiber-reinforced silicone composite has a reflectance that varies with the fiber direction. A smoothly-molded surface has anisotropy aligned to the molding flow direction. This chapter names the two anisotropic-reflection shaders `sim-soft` ships.

| Section | What it covers |
|---|---|
| [Wet-surface BRDF](01-anisotropic-reflection/00-wet.md) | Screen-space wetness map + anisotropic specular aligned to contact flow direction. Handles the "glove just pressed against skin" look where contact transiently alters reflectance |
| [Anisotropic GGX](01-anisotropic-reflection/01-ggx.md) | Walter et al. 2007's GGX microfacet distribution, extended to anisotropic form (Burley 2012) via roughness $\alpha_x, \alpha_y$ along local tangent axes. Industry baseline for directional specular |

Three claims.

**Anisotropic GGX is the shader baseline.** Walter et al. 2007's microfacet GGX distribution, extended to anisotropic form by Burley 2012, is the current industry consensus for directional specular highlights; it is physically-based (energy-conserving, reciprocal), tunable to measured data, and compatible with importance-sampled Monte Carlo for offline reference. `sim-soft` ships it as the default Phase I specular shader, with the tangent axes supplied per-vertex from the mesh.

**Anisotropy direction comes from the physics, not from artist-painted UVs.** Industry practice is that anisotropy direction is baked into a texture at authoring time. `sim-soft`'s anisotropy direction is a per-vertex shader attribute driven by physics — fiber direction from the [anisotropic-material field](../20-materials/06-anisotropic.md) for fiber-reinforced silicones, contact-flow direction from the [contact pressure field](../40-contact/01-ipc-internals.md) for wet-surface scenarios, material-flow direction from `cf-design` authoring for molded-flow cases. Per-vertex anisotropy axes are streamed to the shader alongside stress, temperature, and contact pressure via [Ch 05's streaming API](05-sim-bevy.md).

**Wet-surface BRDF is contact-driven, not manually painted.** When the rigid probe from the canonical problem presses against a silicone cavity, the contact interface has a transiently wet appearance (elasticity-driven interfacial sheen). `sim-soft` models this as a shader that reads contact pressure per surface vertex and adjusts the local roughness and Fresnel response based on pressure magnitude. The same shader handles "rigid body just separated" scenarios where the wetness persists for a brief period after contact ends; the decay is parameterized by a material-specific surface-tension parameter. The [wet-surface sub-chapter](01-anisotropic-reflection/00-wet.md) covers the specific parameterization.
