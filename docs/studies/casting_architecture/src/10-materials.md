# Materials and processes

What this part covers (when authored to depth):

## Cast-side materials

- **Two-part silicones.** Ecoflex 00-10 / 00-20 / 00-30 / 00-50, Dragon Skin 10A / 20A / 30A, Smooth-Sil 940 / 950, Mold Star 15 / 30. Mix ratios, working time, demold time, Shore hardness ranges. Cross-reference to [soft-body Part 1 Ch 04 material data](../../soft_body_architecture/src/10-physical/04-material-data.md) — the simulator and the caster work from the same material database.
- **Polyurethanes.** Smooth-Cast family, water-clear vs. opaque variants, the rigid-vs-flexible spectrum.
- **Loaded composites.** Carbon-black-loaded silicones (per the layered silicone device's middle shell), iron-particle-loaded for magnetic actuation, hollow-glass-microsphere-loaded for density tuning. Cure interaction with the loading agent.

## Cure dynamics

- Pot life, working time, gel time, demold time — the four stages every cast goes through.
- Temperature dependence — Arrhenius behavior, oven post-cure protocols.
- Inhibition — what kills the cure (sulfur from latex gloves, certain plasticizers, some 3D-printed mold materials). The most common failure mode for first-time casters.
- Implications for multi-shot adhesion — first shot must be cured *enough* to demold but not *so* cured the second shot won't bond. Tradeoff window per material.

## Process types

- **Open-pour.** Single-piece mold, no enclosure, gravity-fed. Simplest case, limited to parts with one upward-facing open surface.
- **Closed two-piece.** Two mold halves clamped, pour through a port. Most common for the parts CortenForge targets.
- **N-piece molds.** Three or more pieces for parts with undercuts that no two-piece split can release. Geometric complexity escalates fast.
- **Vacuum-degassing pours.** For bubble-free casts; required for any optical or fluidic surface.
- **Pressure pots.** Alternative to vacuum; bubbles compress to invisibility rather than escape.

## Defects to model

- Bubbles (entrained air, outgassing during cure), shrinkage voids, cure inhibition zones, parting-line flash, surface tackiness from incomplete cure, layer delamination in multi-shot.

## Why this part comes first

Material and process choice constrains everything downstream — mold geometry, parting strategy, sequencing. A free-form part you can cast in Ecoflex 00-10 may be uncastable in Dragon Skin 30A through the same mold (different demold force, different shrinkage). The casting domain's design space is materials × processes × geometry, in that priority order.
