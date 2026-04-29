# Manufacturing constraints

What this part covers (when authored to depth):

The constraints in this part are the things that turn a "geometrically valid mold" into a "moldable cast." They flow back into the design loop as reward terms — see [Part 6 Loop Integration](60-loop-integration.md).

## Wall thickness

- **Minimum cast wall.** Below some thickness, the cast won't fill cleanly (silicone is viscous; thin features starve), or it fills but tears during demold. Material-dependent: ~1 mm for Ecoflex 00-30 in a vacuum-degassed pour, more for stiffer/more viscous materials.
- **Minimum mold wall.** Mold material flexes under pour pressure if walls are too thin. Cavity dimensions drift.
- **Aspect ratio limits.** A 1 mm wall over a 100 mm span has a ~1:100 aspect ratio; even if the wall thickness is technically above minimum, the ratio is unstable.

## Draft angles

The slope on side walls that lets the cast release from the mold. Zero draft means the cast must shear-stretch off the mold wall on demold; positive draft means it just slides out. Required draft is material-dependent (stretchy silicones need less, rigid polyurethanes need more) and geometry-dependent (deep cavities need more than shallow ones).

Adding draft is a design-time geometric perturbation. Either:

- The designer authors the part with draft already present.
- The casting toolchain modifies the part's SDF to add draft along the parting direction. This perturbs the simulator's geometry — a manufacturability vs. functional-design tradeoff that the design loop has to know about.

## Surface area / volume considerations

- **High SA/V ratios** (thin films, intricate surface texture) are slow to cast (thin features fill slowly), prone to bubbles, and prone to demold tearing.
- **Low SA/V ratios** (chunky parts) are easy to cast but use more material per part.
- **Rapid SA/V transitions** (a thin web connecting two thick lobes) are the worst case — the thin web cures faster than the thick lobes, and the material transition creates internal stress concentrations.

## Material cost

A constraint that deserves its own thinking. Per-unit material cost varies by ~10× across the silicone family (Ecoflex 00-30 is cheap; Smooth-Sil with platinum cure is expensive). Multi-shot parts pay material cost N times. For one-off bench parts cost is mostly noise; for any iteration-heavy design loop it accumulates and matters.

The design loop should know its material budget — manufacturability isn't just "can it be made" but also "can it be made within the material budget." A simple per-shot material-volume constraint suffices for most cases.

## Cure-time budget

Casting is slow. Ecoflex 00-30 demolds in 4 hours at room temperature (faster with heat). Multi-shot parts pay this N times. A three-shot layered device is a half-day-to-overnight process. The design loop's outer cycle (per [soft-body Part 10 Ch 06](../../soft_body_architecture/src/100-optimization/06-full-loop.md)) operates at a "weeks" timescale partly *because* of this — print/cast cycle time is a binding constraint on iteration speed.

## Programmatic surface

`sim-cast::ManufacturabilityCheck::evaluate(part_sdf, cast_sequence, materials_db, budget)` returning a `ManufacturabilityReport`:

```rust
pub struct ManufacturabilityReport {
    pub min_cast_wall_thickness_violations: Vec<RegionViolation>,
    pub draft_violations: Vec<RegionViolation>,
    pub aspect_ratio_violations: Vec<RegionViolation>,
    pub material_cost_total: f64,
    pub cure_time_total: Duration,
    pub aggregate_score: f64, // smooth, differentiable; suitable for reward composition
}
```

The aggregate score is what the optimizer reads. The per-violation lists are what the designer reads.

## Open questions

- Smooth-vs-hard constraints. A wall-thickness violation isn't a soft penalty — it's a hard "this part cannot be made." But the optimizer wants smooth gradients. The right framing is probably barrier functions (à la IPC) over the constraint manifold.
- Material budget as a Pareto axis vs. a hard constraint. Same question.
