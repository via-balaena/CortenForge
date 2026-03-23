# Example 09 — Polish TODO

Current state: physics works (5 DOF constrained, pendulum swings), but
the visual product is not acceptable. Issues below, ordered by impact.

## Visual quality

- **Socket mesh jaggedness** — internal cuboid-cylinder subtract edges
  produce sharp mesh artifacts. Smooth_subtract or a rounded cuboid
  (cuboid with round()) would fix this. The pin got smooth_union already.
- **Socket transparency hard to read** — semi-transparent steel blends
  everything together. Consider wireframe, cutaway, or higher opacity
  with a visible bore opening.
- **Pendulum weight too small** — the sphere (r=3mm) is tiny relative to
  the socket (20×20×24mm). Hard to see the rotation. Needs bigger weight,
  longer arm, more dramatic swing arc.
- **Flat shading on some faces** — some mesh triangles render flat instead
  of smooth. May need smooth normals in mesh generation or Bevy material
  settings.

## Performance

- **FPS drops with smooth_union** — smooth blends increase the Lipschitz
  constant, making the grid collision path slower per step. Options:
  GPU path, coarser collision grid, or separate visual-only smooth
  geometry from collision geometry.
- **500Hz timestep with grid concave collision** — each physics step does
  a full grid scan of the concave socket. The octree path can't help
  (concave intervals too loose — falls back to grid via §5.7 heuristic).

## Physics polish

- **Pendulum motion should be smooth and rhythmic** — currently looks
  chaotic ("pencil sharpener on meth"). The weight orbits but the motion
  is jerky, not the smooth sinusoidal swing of a real pendulum.
- **Damping rate** — the pendulum stops within ~5 seconds. Some damping is
  expected (condim=1 frictionless, but solver discretization still damps).
  Should persist longer for a convincing demo.
- **Pin wobble** — the pin visibly shifts inside the bore (r_xz up to
  0.5mm). Tighter clearance or stiffer solver settings might help.

## Design / readability

- **Doesn't read as "hinge" to a viewer** — needs clearer visual language:
  visible crank arm extending out of the socket, obvious pendulum weight
  hanging below, maybe an indicator arrow showing the rotation axis.
- **Camera angle** — default orbit camera doesn't show the rotation well.
  A side view perpendicular to the bore axis would make the swing obvious.
- **Scale reference** — no sense of how big this is. A ruler, grid lines,
  or comparison object would help.
