# Composites — Cable Composite Bodies

The composite system expands a single `<composite type="cable">` MJCF element
into a chain of capsule bodies connected by ball joints. Cables are rigid-body
chains (not deformable) — useful for ropes, cables, tethers, and flexible
tubes. The engine auto-generates geoms, boundary sites, and contact exclusions
for adjacent segments.

## Examples

| Example | Concept | What you see |
|---------|---------|-------------|
| [stress-test](stress-test/) | Headless validation | 8 checks (11 assertions): body count, joint types, contact exclusions, gravity hang, segment convergence, length preservation, curve shapes, multi-cable independence. |
| [hanging-cable](hanging-cable/) | Resolution comparison | Three cables (5/10/20 segments) pinned at one end — more segments = smoother catenary curve. |
| [cable-catenary](cable-catenary/) | Both-ends-fixed sag | Gold cable spanning two pylons, pinned at both ends. Sags under gravity into a catenary shape. |
| [cable-loaded](cable-loaded/) | Midpoint force | Cyan cable spanning two pylons with a 5N downward force at the midpoint. V-shaped sag deeper than passive catenary. |

## Key ideas

- **`<composite type="cable" count="N 1 1">`:** N vertices generate N-1 bodies
  in a linear parent chain. Each body gets a capsule (or cylinder/box) geom
  and a ball joint (3 rotational DOF).
- **`initial="none"`:** First body has no joint — pinned rigidly to its parent.
  `"ball"` (default) gives it a ball joint; `"free"` gives it 6 DOF.
- **`curve="l 0 0"`:** Curve shapes control the initial vertex layout. `l` =
  line, `s` = sine, `c` = cosine, `0` = zero. Three values for x, y, z axes.
- **`size="L A F"`:** Length along the curve axis, amplitude for sin/cos,
  frequency multiplier.
- **Slack for catenary:** Rigid links summing to exactly the span can't sag.
  Use `curve="l 0 s"` with a sine bulge to give path length > span.
- **Contact exclusions:** Automatically generated for adjacent cable bodies
  (N-2 pairs for N-1 bodies) to prevent self-collision between neighbors.
- **`prefix="X"`:** All generated elements get prefix X (bodies: XB_first,
  XB_1, ..., XB_last; joints: XJ_first, ...; geoms: XG0, XG1, ...).
- **Boundary sites:** `S_first` at the cable origin, `S_last` at the cable
  tip — useful for attaching tendons, equality constraints, or sensors.
