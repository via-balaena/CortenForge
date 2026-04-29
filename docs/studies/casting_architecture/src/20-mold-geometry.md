# Mold geometry from SDFs

What this part covers (when authored to depth):

## SDF cavity inversion

The foundational operation. Given a part SDF `f_part(x)` and an enclosure (typically a bounding box `B`), the mold cavity SDF is `f_mold(x) = max(-f_part(x), f_B(x))` — or, in `sim-soft::sdf_bridge` terms, `DifferenceSdf(B, part)`. The same `DifferenceSdf` the simulator already uses for hollow-shell parts is the workhorse of mold generation.

This is one place where the soft-body book's [Part 7 SDF pipeline](../../soft_body_architecture/src/70-sdf-pipeline.md) does double duty — every operator the simulator implements (CSG, smooth blends, displacement) is reusable for mold authoring.

## Enclosure design

The bounding enclosure is a design choice, not a derived quantity. Options:

- **Tight box.** Minimum bounding box plus draft margin. Smallest mold volume, cheapest to fab.
- **Functional fixture.** Mold doubles as a clamping fixture or alignment jig. Larger volume, but absorbs setup tooling.
- **Standard tray sizes.** Mold sized to standard fab tray dimensions (3D printer bed, CNC stock). Practical constraint.

The choice is downstream of the fab path (3D-printed mold? CNC'd aluminum? Lost-PLA?) — different fabs have different cost structures over enclosure dimensions.

## Wall thickness

Mold walls have a minimum thickness driven by:

- The mold material's stiffness (3D-printed PLA is more compliant than aluminum; needs more wall to keep the cavity dimensionally stable under pour pressure).
- The cast material's pour weight (taller pours = more hydrostatic pressure on mold walls).
- Demolding force (some silicones release easily, some cling — and clinging cast pulls on the mold).

Defaults: 3–5 mm for printed PLA molds at part-sizes CortenForge targets (≤ 20 cm); 2–3 mm for aluminum; thicker for parts that exert significant demolding force.

## Pour ports, vents, runners, risers

The plumbing that makes the cavity actually fill cleanly:

- **Pour port.** Where the liquid silicone enters. Position determined by gravity-feed geometry — typically the highest point on the cavity, with a funnel above.
- **Vents.** Where displaced air escapes. Without vents, air gets trapped and you cast a part with a large cavity void where the pour stalled.
- **Runners.** Channels distributing pour from the port to the cavity. Mostly relevant for large parts or thin-walled cavities where gravity feed alone leaves dry spots.
- **Risers.** Reservoirs of extra material that feed shrinkage during cure. Less critical for silicones (low shrinkage) than for polyurethanes (1–2% shrinkage).

For most CortenForge-scale parts, the pour-port-and-two-vents pattern handles 90% of cases — the rest need explicit runner design.

## Cores for internal cavities

Parts with internal voids (the layered silicone device's hollow center) need a *core* that occupies the void during pour and gets removed afterward. Three core strategies:

- **Sacrificial core.** Water-soluble PVA or wax, dissolved/melted out after demold. Works for any cavity geometry. Slow.
- **Pull-out core.** Rigid insert with a draft angle, pulled out through a port. Limits cavity geometry to demoldable shapes.
- **Two-shot inflate.** Cast a soft part around an inflatable mandrel, deflate and remove. Niche but elegant for thin-walled spheres.

The layered silicone device's cavity probably wants a sacrificial PVA core — geometry is too closed for pull-out and the device is one-off enough that PVA cost is fine.

## Programmatic surface

`sim-cast::MoldGeometry::from_part(part_sdf, enclosure, hints)` returning `(cavity_sdf, port_positions, vent_positions, core_specs)`. The `hints` parameter mirrors `sim-soft::MeshingHints` in spirit — opt-in tunables for what would otherwise be heuristic defaults.
