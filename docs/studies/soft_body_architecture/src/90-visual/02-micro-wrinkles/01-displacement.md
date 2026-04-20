# Displacement mapping

[§0 normal-map wrinkles](00-contact-normals.md) handle the shading effect of micro-wrinkles at fragment granularity but leave the silhouette unchanged. For close-up shots where the wrinkled rim is rendered against a contrasting background, the smooth silhouette of the subdivided render mesh reads as too smooth — the wrinkles appear in the shading but not at the outline. Displacement mapping is the fix: actually move the render-mesh vertices according to the bump field, so the silhouette reflects the wrinkle geometry. This sub-leaf specifies the sim-soft displacement variant, its cost, and the quality-tier condition that enables it.

## The technique

Displacement mapping in the canonical sense — move mesh vertices by a height field — appears first in [Cook 1984](../../appendices/00-references/04-rendering.md#cook-1984)'s "Shade Trees" as one of the surface-shading operators a shade tree can express, and is productionized in the Reyes renderer architecture that Cook, Carpenter & Catmull built around it. The distinction from [§0 bump mapping](00-contact-normals.md) is that displacement actually moves geometry rather than perturbing shading: the vertex positions in the render mesh are shifted along the normal by $h(x, y)$ before rasterization, so the rasterizer sees the wrinkled geometry and the silhouette picks up the wrinkle profile.

`sim-soft`'s displacement mapping operates on the [subdivided render mesh from Ch 03](../03-subdivision.md), not the physics tet mesh. The subdivision pass produces a mesh dense enough that per-vertex displacement can resolve wrinkles at the target frequency — typically one subdivision level above the physics mesh, producing around 50k render-triangles for the canonical 30k-tet scene, matched to the wrinkle spatial frequency. Displacement is applied in a compute-shader pass between subdivision and rasterization:

```wgsl
@compute @workgroup_size(64)
fn displace_vertices(
    @builtin(global_invocation_id) gid: vec3<u32>,
) {
    let i: u32 = gid.x;
    if (i >= num_vertices) { return; }
    let v: SubdividedVertex     = subdivided_verts[i];
    let h: f32                  = evaluate_bump_field(v.pos, v.contact_pressure, v.grad_p);
    displaced_verts[i].pos      = v.pos + h * v.normal;
    displaced_verts[i].normal   = recompute_normal(i, h);  // post-displacement analytic normal
}
```

`evaluate_bump_field` is the same sinusoidal construction as [§0](00-contact-normals.md)'s `contact_bump` — amplitude, frequency, and perpendicular-to-gradient direction all derive from the same material parameters and physics signals. The only new piece is `recompute_normal`: after the displacement, each vertex's normal is re-derived analytically from the bump field's gradient, so that [§1 anisotropic GGX](../01-anisotropic-reflection/01-ggx.md) downstream sees the geometrically-correct normal rather than an interpolated stale one.

## Cost and tier gating

Displacement is expensive. The compute pass over every subdivided-mesh vertex adds a dispatch to the [Ch 05 shader pipeline](../05-sim-bevy.md); on the canonical scene's subdivided-mesh vertex count the pass runs in the millisecond range, not free. Beyond that, the downstream rasterization works on denser vertex data — more triangle setup, more attribute interpolation per fragment. The aggregate cost is a few milliseconds added to the 8–12 ms Phase I frame budget.

`sim-bevy`'s `SimSoftConfig` gates displacement behind the same `sss_quality` setting [§2 of the SSS sub-chapter](../00-sss/02-realtime.md) introduced. The `HighQuality` tier enables displacement; the `Production` default skips it (normal-map-only wrinkles from [§0](00-contact-normals.md)); the `Debug` tier also skips. The condition for switching up to `HighQuality` is a user choice — typically enabled for design-mode close-up previews or capture frames for documentation, left off during interactive sweeps.

The gating matters because the silhouette-correct-wrinkle improvement is visually significant only in a narrow regime: close-up shots of tightly-pressed rims, at rendering resolutions where the wrinkle spacing exceeds a few pixels. A wider-shot overview render where the entire cavity fits within a few hundred pixels does not benefit from displacement — the wrinkles are below the pixel grid anyway, and [§0's](00-contact-normals.md) fragment-shading reconstruction does the job. Ship `Production` for the common case; `HighQuality` is the escape hatch.

## Subdivision coupling

Displacement requires an adequately dense render mesh. If the subdivided mesh edge length is longer than the wrinkle wavelength, the displacement resolves aliased geometry rather than smooth wrinkles — the render shows faceted wrinkle-peaks instead of continuous wrinkles. [Ch 03 subdivision](../03-subdivision.md)'s default subdivision level is one level (each tet face becoming 4 triangles); displacement mode bumps this to two levels (16 triangles per face) to support finer wrinkles. The additional subdivision cost (per-vertex interpolation, roughly 4× the output vertex count) stacks with the displacement compute-pass cost.

Two-level subdivision is still cheap compared to the baseline render, but the combined subdivision + displacement + denser rasterization is what pushes the budget. `HighQuality` is the tier where this is acceptable; `Production` is not.

## What this sub-leaf commits the book to

- **Displacement is an optional Phase I+ quality tier**, not a default shader. Gated behind `SimSoftConfig::sss_quality = HighQuality`.
- **The bump field is the same as [§0](00-contact-normals.md)'s** — amplitude, frequency, and direction driven by contact-pressure + gradient from the same material-database parameters. Displacement reuses the bump function rather than introducing a second one.
- **Displacement runs on the subdivided render mesh, not the physics tet mesh.** The render mesh doubles the subdivision level (one → two) when `HighQuality` is active to ensure the displaced vertex density exceeds the wrinkle frequency.
- **The typical use case is close-up previews.** `Production` default skips displacement; the silhouette-correct-wrinkle improvement is visible only in narrow conditions.
