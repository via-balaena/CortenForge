# Wet-surface BRDF

When the rigid probe from the canonical problem presses against a silicone cavity, the contact interface has a visible sheen distinguishing the pressed region from the dry bulk surface. Designers call this the wet look — the term carries over from the rendering literature even when, as here, no moisture is involved. The actual mechanism is a combination of effects: elastic deformation smoothing surface asperities under load, interfacial reflectance at the contact interface, and any residual release-agent or surface conditioner present on a physical sample. `sim-soft`'s shader does not try to decompose these; it reconstructs the perceived effect from the per-pixel contact-pressure signal that the physics already produces.

## The engineering target

A rendered silicone cavity without any wet-look shader reads as dry-looking even under load — a designer who has squeezed silicone recognises the mismatch. The effect to reproduce is that the pressed region's specular lobe *tightens* relative to the surrounding dry surface, and the tightening scales with contact pressure. After contact releases, the tightening relaxes back to the dry-surface baseline over a short timescale.

[Jensen, Legakis & Dorsey 1999](../../appendices/00-references/04-rendering.md#jensen-legakis-dorsey-1999)'s "Rendering of Wet Materials" is the canonical reference in the rendering literature: a layered BRDF with an interfacial film that produces darkening, specular-shift, and a secondary reflection. `sim-soft`'s case is simpler — no moisture film, and the specular tightening is the only perceived shift Phase I commits to — so we inherit the structural reading from Jensen but drop the moisture-film layer.

## The shader model

The shader reads two per-pixel physics attributes from [`readout/`](../../110-crate/00-module-layout/09-readout.md): the contact-pressure magnitude (in [`MeshSnapshot::contact_pressure`](../05-sim-bevy.md), sourced from [IPC's barrier-energy contact readout](../../40-contact/01-ipc-internals/03-energy.md)) and the contact-flow direction. The flow direction is derived per surface vertex as the in-plane component of the local contact-pressure gradient and flows to the renderer via the [`MeshSnapshot::anisotropy_dir` attribute](../05-sim-bevy.md). When contact pressure is non-zero on a vertex, this attribute carries the flow direction; in dry regions it carries zero, and the material's dry-state anisotropy source (fiber direction or molded flow per [§1](01-ggx.md)) applies instead.

Under contact, the shader modulates the GGX tangent-axis roughnesses *differentially* — the axis aligned to contact flow narrows more than the perpendicular axis — producing the directional specular streaming the wet look is recognised by:

```wgsl
fn wet_modulated_roughness(
    base_alpha: vec2<f32>,
    contact_p: f32,
    p_ref: f32,
    wet_strength: f32,          // material-database entry, in [0, 1]
    aniso_fraction: f32,        // material-database entry, in [0, 1]
) -> vec2<f32> {
    let t: f32 = smoothstep(0.0, p_ref, contact_p);          // saturation in [0, 1]
    let along_scale: f32 = 1.0 - wet_strength * t;
    let perp_scale:  f32 = 1.0 - wet_strength * t * (1.0 - aniso_fraction);
    return vec2<f32>(base_alpha.x * along_scale, base_alpha.y * perp_scale);
}
```

Four parameters govern the behaviour. `p_ref` is the material-specific reference pressure at which the wet effect saturates; `wet_strength` $\in [0, 1]$ bounds the maximum fractional roughness reduction along the flow axis; `aniso_fraction` $\in [0, 1]$ controls how much of that reduction is *withheld* from the perpendicular axis — at `aniso_fraction = 0` the modulation is isotropic (both axes scale identically), at `aniso_fraction = 1` only the along-flow axis changes and the perpendicular axis stays at its dry value. All three are per-material entries in the [appendix material database](../../appendices/02-material-db.md), not hardcoded.

`base_alpha` is the dry-state $(\alpha_x, \alpha_y)$ pair the [§1 GGX shader](01-ggx.md) would otherwise consume directly. For isotropic dry materials ($\alpha_x = \alpha_y$), the wet modulation is the sole source of anisotropy; for already-anisotropic dry materials (fiber-reinforced), the wet effect stacks on top.

The spine's Ch 01 §3 claim that the wet shader also modulates Fresnel is not implemented in Phase I. The directional roughness modulation above already captures the perceived sheen-tightening against the dry baseline; Fresnel modulation is a Phase-I-plus extension if validation against physical reference images shows the roughness-only model under-captures the wet look at grazing angles.

## Post-contact decay

After contact releases, the surface asperities recover elastically with a material-dependent time constant. The shader tracks per-vertex contact pressure with one-step history and applies an exponential decay toward the dry state:

```wgsl
struct WetPerVertex {
    live_pressure: f32,           // current-frame contact pressure
    decaying_pressure: f32,       // exponential decay memory
};

fn update_wet_state(state: WetPerVertex, dt: f32, tau_wet: f32) -> WetPerVertex {
    // live pressure wins; otherwise decay toward zero
    if (state.live_pressure > 0.0) {
        return WetPerVertex(state.live_pressure, state.live_pressure);
    }
    let decay_factor: f32 = exp(-dt / tau_wet);
    return WetPerVertex(0.0, state.decaying_pressure * decay_factor);
}
```

`tau_wet` is a material-specific parameter read from the material database. Whether the surface-recovery timescale matches the bulk [Prony-series relaxation spectrum from Part 2 Ch 07](../../20-materials/07-viscoelastic.md) is an empirical question — the two mechanisms share physical origins (polymer-chain viscoelastic recovery) but the surface-specific decay may differ from the bulk rate and is measured separately. The shader treats $\tau_\text{wet}$ as an independent per-material entry rather than deriving it from the Prony fit.

The decay state lives on the shader side only; it is not recorded on the autograd tape and does not feed back into the physics. This is deliberate — the wet appearance is a visual consequence of the physics, not an input to it, and propagating gradients through shader-side state is neither meaningful for optimization nor needed for any downstream consumer.

## What this sub-leaf commits the book to

- **Wet appearance is contact-pressure-driven roughness modulation, not a moisture-film simulation.** The shader reconstructs the perceived sheen from per-pixel contact pressure plus contact-flow direction; the underlying mechanism (asperity flattening, interfacial reflectance, or other) is not decomposed.
- **Modulation is anisotropic — the axis along contact flow narrows more than the perpendicular axis.** Produces the directional specular streaming characteristic of the wet look. For dry-isotropic materials the wet shader is the sole anisotropy source; for fiber-reinforced materials the wet effect stacks on top.
- **Post-contact decay is a shader-side exponential with material-specific $\tau_\text{wet}$.** Whether the surface-recovery timescale matches the bulk [Prony spectrum from Part 2 Ch 07](../../20-materials/07-viscoelastic.md) is empirical; the shader treats $\tau_\text{wet}$ as an independent material-database entry.
- **Fresnel modulation is deferred.** The spine's Ch 01 §3 mentions Fresnel adjustment; Phase I ships roughness-only. Fresnel extension lands in Phase I+ if reference-image validation shows grazing-angle appearance under-captured.
- **No shader-side feedback into physics.** The wet state is a render-only buffer; gradients do not pass through it.
