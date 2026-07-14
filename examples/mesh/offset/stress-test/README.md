# offset/stress-test

**The `mesh-offset` domain validation superset.** One headless validator
for `mesh_offset::offset_mesh` — the SDF + marching-cubes offset
operator — exercising a unit cube in both directions and self-gating
against closed-form volume oracles and topological invariants. Folded
from the former `mesh-offset-inward` / `mesh-offset-outward` example pair
(each now a module).

`offset_mesh` samples the input's signed-distance field on a 3D grid,
shifts every sample by the offset distance (translating the
zero-isosurface along the SDF gradient), and re-extracts the isosurface
via marching cubes. A negative distance erodes (inward); a positive
distance dilates (outward).

## What it checks

Run it red-or-green with `cargo xtask run-validators` (or
`cargo run --release -p example-offset-stress-test`). Each module aborts
(exit 101) on any oracle mismatch and writes its clean mesh to `out/`
for the `cf-viewer` visual-review path.

| Module | Offset | Oracle |
|---|---|---|
| [`outward`](src/outward.rs) | `+0.1` (dilation) | Steiner-Minkowski volume `V_d = 1 + 6d + 3πd² + (4π/3)d³`, within 5% |
| [`inward`](src/inward.rs) | `-0.1` (erosion) | Exact polytope volume `V_d = (1 + 2d)³`, within 2% |

Both also assert the output is a clean, **watertight, outward-wound,
genus-0** 2-manifold (χ = 2), plus AABB placement within half a grid
cell and a PLY round-trip.

## The inward/outward asymmetry

The headline geometry: **dilation rounds, erosion stays sharp.**

| Direction | Decomposition | Volume |
|---|---|---|
| **Outward** (d > 0) | cube + 6 face slabs + 12 edge quarter-cylinders + 8 corner sphere octants | `1 + 6d + 3πd² + (4π/3)d³` (Steiner-Minkowski) |
| **Inward** (d < 0) | a smaller scaled cube with sharp 90° corners and edges | `(1 + 2d)³` (exact polytope) |

At a convex corner, dilating the body sweeps a ball across the corner and
adds a rounded sphere-octant + edge-cylinder cap — so the outward body is
*not* a scaled cube (and its volume sits strictly between the original
cube and the naive expanded box of side `1 + 2d`). Erosion has no convex
features to round: the level set simply retreats into the interior along
each face normal and the polytope stays sharp, so the inward volume is the
exact `(1 + 2d)³`. The `outward` module asserts this asymmetry directly
(Steiner value bracketed between cube and expanded box).

## The grid-alignment topology pitfall

Marching cubes classifies each grid cell by the SDF signs at its 8
corners. When the eroded cube's flat faces land **exactly on sample
planes** (a cell size that divides the offset magnitude — e.g. `0.025`
into `0.1`), the SDF is zero across a whole face and the sign test is
ambiguous; MC stitches spurious handles through those degenerate cells.
The mesh stays closed and its volume stays correct, but its **genus jumps
above 0**. Nudging the resolution off-grid (this example uses `0.03`, which
does not divide `0.1`) puts the level set *between* samples and restores a
clean genus-0 manifold.

The `inward` module gates on the clean off-grid result and *demonstrates*
the on-grid degeneracy as diagnostic output — it does **not** assert the
spurious genus, since that is a marching-cubes limitation to leave open to
a future fix, not a contract to lock in. Outward offset is immune to the
same alignment (its rounded Steiner surface never coincides with a full
sample plane), and the `outward` module asserts that robustness — the
flip side of the pitfall.

The library-level guarantee (watertight + outward-wound + genus-0 output
at an off-grid resolution) is gated in `mesh-offset` itself by
`offset_mesh_is_watertight_outward_wound_genus0_off_grid`; the winding
half is gated by `marching_cubes_produces_outward_winding_on_cube_sdf`
(the §Q-5 fix). This example is the demonstration layer on top of those.

> **History.** These examples once demonstrated a per-face winding-flip
> workaround: pre-§Q-5, `offset_mesh` emitted inside-out, vertex-soup
> (un-welded) marching-cubes output that needed flipping and welding to
> render and measure. The §Q-5 winding fix plus the MC edge-vertex cache
> retired both problems — the output is now clean directly — so that
> narrative (and its `after_flipped.ply` artifact) retired with it.

See `docs/studies/mesh_architecture/src/30-sdf-and-offset.md` for the
depth pass on the mesh→SDF bridge, and `examples/mesh/README.md` for the
domain layout convention.
