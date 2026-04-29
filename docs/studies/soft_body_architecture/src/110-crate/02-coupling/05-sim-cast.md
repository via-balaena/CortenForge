# With sim-cast (casting domain)

`sim-cast` is the manufacturing-side sibling crate that turns a CortenForge-authored soft-body design into a castable part. Architecturally, it's a peer of `sim-soft`: it consumes the same `Sdf` trait surface and the same `MaterialField` partition the simulator consumes, and it emits mold geometry, a multi-shot casting plan, and a manufacturability reward term that the design loop reads alongside the physics reward.

The casting domain has its own canonical reference at [`docs/studies/casting_architecture/`](../../../../casting_architecture/src/SUMMARY.md). That book covers mold geometry from SDFs, parting-surface determination, multi-shot sequencing, manufacturability constraints, and the loop integration. This page documents only what `sim-soft` exposes to it.

## Coupling surfaces

- **`Sdf` trait.** `sim-cast` re-exports `sim_soft::sdf_bridge::Sdf` rather than defining its own. The same `SphereSdf` / `DifferenceSdf` / future custom SDFs feed both meshers (the simulator's tet mesher and the casting domain's mold-cavity inverter).
- **`MaterialField`.** Multi-shot casting sequences map directly onto the per-tet material partition. Inner shell, middle shell, outer shell — the simulator's `LayeredScalarField` partitions are also the casting domain's shot boundaries.
- **`RewardBreakdown` extension.** The casting domain contributes a `manufacturability: f64` term to the design loop's reward. Smooth where geometry is smooth (wall thickness, draft, material cost), discontinuous where topology changes (parting-surface validity). The optimizer composes the physics reward and the manufacturability reward additively; weights are preference-learned per [Ch 03](../../100-optimization/03-preference.md).
- **Tape sharing.** Open question — does `sim-cast` write its manufacturability gradient to the same `Tape` `sim-soft` writes to (one combined backward pass), or compute its own and add at the reward-scalar level (separate passes, summed scalar)? Performance vs. composability tradeoff. See casting book [Part 6](../../../../casting_architecture/src/60-loop-integration.md) for the live discussion.

## What this means for `sim-soft`

Almost nothing changes on this side. `sim-soft` does not depend on `sim-cast`; the casting domain depends on `sim-soft` (specifically on its `sdf_bridge` and `material::field` modules). `sim-soft`'s public API stays oriented to simulation; the casting domain bolts on top.

The one place this coupling surfaces in `sim-soft` itself is the `RewardBreakdown` struct's eventual extension to carry a manufacturability slot. That's a downstream change — it lands when the casting domain's Phase D (forward manufacturability) ships, not before.

## Out of scope here

This page does not cover:

- Mold geometry generation algorithms (casting book [Part 2](../../../../casting_architecture/src/20-mold-geometry.md)).
- Parting-surface determination (casting book [Part 3](../../../../casting_architecture/src/30-parting.md)).
- Multi-shot sequencing (casting book [Part 4](../../../../casting_architecture/src/40-multi-shot.md)).
- Manufacturability constraint formulations (casting book [Part 5](../../../../casting_architecture/src/50-constraints.md)).
- Cast-side material data (casting book [Part 1](../../../../casting_architecture/src/10-materials.md)).

Anything specific to casting belongs in the casting book. This page is the bridge — what `sim-soft` exposes, what `sim-cast` consumes, and the architectural commitments that hold across the boundary.
