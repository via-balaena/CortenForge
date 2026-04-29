# Multi-shot casting for multi-material parts

What this part covers (when authored to depth):

## The multi-shot problem

A multi-material part — like the layered silicone device's three-shell structure — can't be cast in a single pour. Each material region is a separate cast through a topologically distinct mold. The first material is cast, demolded, located in the next mold, and the next material is cast around it. Repeat for as many materials as the part has.

This maps directly onto the simulator's `MaterialField`. Each Lamé partition is a shot. The casting domain reads the same partition the simulator reads.

## Sequencing

The order of shots is constrained:

- **Inner-to-outer for nested shells.** The layered silicone device casts inner → middle → outer. The previous shot becomes a feature of the next mold's cavity.
- **Stiff-to-soft for graded parts.** Stiffer materials cure first (their molds clamp the soft material's cavity); softer materials fill the remaining space.
- **Cure-time-driven for adhesion-critical sequences.** The previous shot must be cured enough to demold but not so cured the next shot won't bond — narrow window, material-pair-specific.

For an N-material part, the number of valid sequences is N! in the worst case, much less in practice (most pairs are constrained). For three materials, six possible orderings, typically only one or two satisfy all constraints.

## Inter-shot adhesion

The seam between shots. Three regimes:

- **Chemical bonding.** Same-family silicones with overlapping cure chemistry bond mechanically when the second shot's prepolymer interpenetrates the first shot's surface. Usually requires the first shot to be at "demold but tacky" cure state (~70% cure).
- **Mechanical interlock.** Geometric features on the first shot's surface (texture, undercuts at the seam) that the second shot fills. Works across material families that don't chemically bond.
- **Primer/adhesive.** Surface treatment between shots. Required for incompatible material pairs (silicone-to-polyurethane for example). Adds a process step and a potential failure mode.

The choice depends on materials and on whether the seam is structural (the layered silicone device's cavity wall is structural — it carries pressure load) vs. cosmetic (a soft coating over a stiff core just needs to stay attached).

## Mold-in-mold strategies

When the geometry is too complex for sequential filling, sometimes the answer is *mold-in-mold*:

- Cast the inner shape in a small mold.
- Place the cured inner cast inside a larger mold whose cavity is "outer cast minus inner cast."
- Pour the outer material; it cures around the inner.

This is exactly how the layered silicone device cavity gets cast. The inner shell becomes a feature of the middle-shell mold's cavity. The simulator's `DifferenceSdf` composition mirrors this: `MiddleShellMold::cavity = OuterRadius - InnerShellSurface`.

## Programmatic surface

`sim-cast::CastSequence::from_material_field(part_sdf, material_field, materials_db)` returning an ordered list of `Shot` records, each with:

- `shot_id: usize` — sequence position.
- `mold: MoldGeometry` — the SDF and ports for this shot's mold.
- `material: MaterialSpec` — what to pour.
- `cure_state_at_demold: CureFraction` — when to demold (drives next shot's adhesion).
- `adhesion_strategy: AdhesionStrategy` — chemical / mechanical / primer.
- `placement_in_next_mold: Transform` — how this shot's cured cast locates in the next shot's mold.

The output is a "casting plan" — a recipe a human (or a future fab robot) can execute step by step.

## Open questions

- Sequencing as an optimization problem. For three materials it's hand-solvable; for ten it's combinatorial. Algorithmic sequencing under cure-time / adhesion / draft constraints is open.
- Cure-state tracking through a multi-shot sequence. Each shot's cure clock starts when its pour starts; the system has multiple overlapping clocks. Modeling this is its own subproblem.
