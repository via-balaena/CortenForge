# Mesh Examples

Focused, single-concept demonstrations of the 10-crate mesh ecosystem
(`mesh-types`, `mesh-io`, `mesh-repair`, `mesh-sdf`, `mesh-offset`,
`mesh-shell`, `mesh-measure`, `mesh-printability`, `mesh-lattice`, plus
the `mesh` umbrella).

Each example writes one or more PLY artifacts to its own `out/`
directory; per-example READMEs document the locked numerical anchors
(asserted by `cargo run --release` exit-0) and the optional visuals
pass. The architecture book at
[`docs/studies/mesh_architecture/`](../../docs/studies/mesh_architecture/)
covers design rationale; this directory is the executable proof.

For a cross-domain Bevy walkthrough that strings together repair → shell
→ lattice → printability with side-by-side scene rendering, see
[`examples/fundamentals/mesh/mesh-pipeline`](../fundamentals/mesh/mesh-pipeline).
That example complements (does not replace) the focused single-concept
examples here.

## Viewing PLY artifacts

Most examples emit one or more `.ply` files to `out/` (gitignored).
Common viewers:

- **f3d** — quick desktop preview. Pass `--up=+Z` to match the
  build-plate orientation used by the printability and lattice
  examples; f3d's default `+Y`-up world rotates "up" into-screen
  otherwise.
- **MeshLab** / **ParaView** — single-sided rendering and clipping
  planes for inspecting hollow / cavity geometry. Per-example READMEs
  call out where this matters (the printability hollow-box and
  trapped-volume fixtures have **REVERSED-wound** inner shells whose
  normals point INTO the cavity; f3d back-face-culls these by default
  and the cavity becomes invisible).
- **PLY-attribute viewers** (ParaView, MeshLab) — required for the
  vertex-only artifacts that carry custom scalars
  (`extras["signed_distance"]` from `mesh-sdf-distance-query`); colour-
  map by the named scalar to render the SDF.

The measurement, SDF, and lattice examples follow a math-pass-first
cadence: every coordinate, winding, count, and bound is encoded as an
`assert_*` in `src/main.rs`, so a clean `cargo run --release` exit-0
IS the correctness signal — the visuals pass is optional pedagogy.

## Examples by mesh crate

### `mesh-types` — attributed-mesh shape

| Example | Concept |
|---------|---------|
| [`attributed-mesh-basics`](attributed-mesh-basics/) | The SoA attributed-mesh shape — built-in slots, extras, length validation (no I/O) |

### `mesh-io` — formats and persistence

| Example | Concept |
|---------|---------|
| [`ply-with-custom-attributes`](ply-with-custom-attributes/) | Per-vertex custom-attribute PLY round-trip via `save_ply_attributed` |
| [`format-conversion`](format-conversion/) | STL ↔ PLY ↔ OBJ — what each format preserves (geometry vs topology) |

### `mesh-repair` — validate, heal, re-validate

| Example | Concept |
|---------|---------|
| [`mesh-repair-walkthrough`](mesh-repair-walkthrough/) | The repair pipeline pattern from broken to clean (validate → repair → fill → re-wind → re-validate) |

### `mesh-offset` — SDF-based offset surfaces

| Example | Concept |
|---------|---------|
| [`mesh-offset-outward`](mesh-offset-outward/) | SDF + marching cubes outward offset — corner rounding; vertex-soup MC output handled with per-face flip pattern |
| [`mesh-offset-inward`](mesh-offset-inward/) | SDF + marching cubes inward offset — sharp polytope preservation (no Steiner-Minkowski rounding); pair-companion to `mesh-offset-outward` |

### `mesh-shell` — wall-thickness shell generation

| Example | Concept |
|---------|---------|
| [`shell-generation-fast`](shell-generation-fast/) | Normal-based shell on an open-topped box — vertex correspondence preserved; perpendicular wall thickness varies as `1/√k` at corners |
| [`shell-generation-high-quality`](shell-generation-high-quality/) | SDF-based shell on a closed cube — uniform perpendicular wall thickness; Steiner-Minkowski rounding at sharp creases |

### `mesh-measure` — geometric measurement

| Example | Concept |
|---------|---------|
| [`mesh-measure-bounding-box`](mesh-measure-bounding-box/) | AABB + OBB on a two-shape fixture (axis-aligned cube + 45°-rotated brick) — when do they coincide, when do they diverge? PCA recovers the brick's `(20, 10, 10)` extents on a non-cubic input |
| [`mesh-measure-cross-section`](mesh-measure-cross-section/) | Planar slicing of a 32-segment closed cylinder — mid-slice + 10-slice stack + `circumference_at_height` / `area_at_height` helpers + out-of-mesh + plane-normal auto-normalization |
| [`mesh-measure-distance-to-mesh`](mesh-measure-distance-to-mesh/) | Point-to-point + unsigned point-to-mesh + symmetric Hausdorff between two vertex-disjoint unit cubes; Hausdorff is composition over the exposed primitives, not a built-in |

### `mesh-sdf` — signed-distance queries

| Example | Concept |
|---------|---------|
| [`mesh-sdf-distance-query`](mesh-sdf-distance-query/) | Numerical SDF on a unit octahedron — `SignedDistanceField` cached queries + free-fn one-shots + direct primitives (`closest_point_on_triangle`, `ray_triangle_intersect`, `point_segment_distance_squared`) + `point_in_mesh` ray-casting + 1000-point grid PLY with `extras["signed_distance"]` |

### `mesh-printability` — manufacturability validation

| Example | Concept |
|---------|---------|
| [`printability-thin-wall`](printability-thin-wall/) | §6.1 ThinWall detector — double-walled hollow box with a 0.4 mm top wall thinned below FDM's 1.0 mm threshold; two Critical ThinWall clusters + cavity-ceiling Overhang co-flag |
| [`printability-long-bridge`](printability-long-bridge/) | §6.2 LongBridge detector — 24-vertex H-shape with a 20 mm middle bridge above FDM's 10 mm `max_bridge_span`; the two 5 mm cantilevers cluster independently and silent-drop below the threshold |
| [`printability-trapped-volume`](printability-trapped-volume/) | §6.3 TrappedVolume detector — solid cube with a sealed sphere cavity; `Info` on FDM (extrusion prints through), `Critical` on SLA / SLS / MJF (resin / powder traps) |
| [`printability-self-intersecting`](printability-self-intersecting/) | §6.4 SelfIntersecting detector — two interpenetrating cylinders without boolean cleanup; flags 100 triangle pairs (mesh-repair `max_reported` cap) + single-cylinder convex-mesh regression |
| [`printability-small-feature`](printability-small-feature/) | §6.5 SmallFeature detector — 30 mm cube + 0.2 mm hex-prism burr; Warning band (`max_extent < min_feature_size / 2`); ThinWall co-flag drives `is_printable() == false` |
| [`printability-orientation`](printability-orientation/) | `PrinterConfig::build_up_direction` parametrization — leaning cylinder revalidated under three runs (original / manual `R_Y(-60°)` rotation / build-up-direction set to the axis); the Gap L invariant is mesh-rotation ≡ up-vector-rotation when both are exact |
| [`printability-technology-sweep`](printability-technology-sweep/) | Cross-technology severity divergence — same hollow box validated under FDM / SLA / SLS / MJF; each tech fails `is_printable()` for a *different* reason |
| [`printability-showcase`](printability-showcase/) | Capstone — single bracket fixture (528 v / 1032 f, five vertex-disjoint shells) exercises all six detectors at once; surfaces five deliberate spec deviations as load-bearing pedagogical observations |

### `mesh-lattice` — TPMS, strut, and composite lattices

#### TPMS path — implicit-surface lattices via marching cubes

| Example | Concept |
|---------|---------|
| [`mesh-lattice-tpms-gyroid`](mesh-lattice-tpms-gyroid/) | TPMS lattice via the gyroid implicit surface — `gyroid` / `density_to_threshold` / `make_shell` direct anchors then `generate_lattice` for a 30 mm cube at density 0.5; locks the F10 vertex-soup MC signature (`vertex_count == 3 × triangle_count` BIT-EXACT) |
| [`mesh-lattice-shape-bounded`](mesh-lattice-shape-bounded/) | Boundary-conforming TPMS via `with_shape_sdf` — gyroid clipped to an analytical sphere of radius 12 mm; trim drops 72.8% of vertices vs the bbox-filling baseline (87 480 vs 321 084) |

#### Strut path — cylindrical beams between grid nodes

| Example | Concept |
|---------|---------|
| [`mesh-lattice-strut-cubic`](mesh-lattice-strut-cubic/) | Cubic strut lattice — `generate_strut` / `combine_struts` / `estimate_strut_volume` direct anchors then `generate_lattice` with `with_beam_export(true)` populating the 3MF `BeamLatticeData` precursor (216 deduplicated grid nodes + 540 beams) |
| [`mesh-lattice-density-gradient`](mesh-lattice-density-gradient/) | Variable-density octet-truss via `DensityMap::Gradient` — per-beam `r1 = strut_thickness/2 × √density` modulation; four discrete cell-z strata produce four discrete radius bands visible end-to-end in the output |

#### Composite (FDM infill) — shell + lattice + caps + connections

| Example | Concept |
|---------|---------|
| [`mesh-lattice-mesh-bounded-infill`](mesh-lattice-mesh-bounded-infill/) | FDM-style shell + lattice composite via `generate_infill` on a hand-authored 50 mm watertight cube — outer inward-offset shell + interior cubic lattice + bridging connections + solid caps; four PLY outputs (input / shell / lattice / composite) |

## Future examples

The mesh book's Part 10 — [Roadmap](../../docs/studies/mesh_architecture/src/100-roadmap.md)
— tracks v0.9 example candidates surfaced by the v1.0 arc (welded TPMS-
lattice MC output, 3MF beam-lattice writer consuming the
`BeamLatticeData` populated above, winding-number inside-test for the
SDF-vertex-region edge case, and others). New mesh examples should be
added there first, then promoted to a workspace member crate when a
real consumer drives them.

## Layout convention

Every example is a workspace member crate at:

```
examples/mesh/<name>/
├── Cargo.toml     # [package].name = "example-mesh-<name>"
├── README.md      # museum-plaque (template below)
├── src/main.rs    # writes PLY to out/
└── out/           # gitignored; generated artifacts
```

Names are dash-case to match the rest of the workspace
(`mesh-pipeline`, `hello-solid`, etc.).

## Per-example README template

````markdown
# <example name>

<one-paragraph concept statement — the platform capability this demonstrates>

## What it does

<2–4 sentences: input mesh, transformation applied, output written>

## Numerical anchors

- <verifiable number Claude checks (vertex count, manifold check, volume, …)>
- <…>

## Visuals

- <what the user looks for in the PLY (shape, color gradient, …)>
- <…>

## Run

```
cargo run -p example-mesh-<name> --release
```

Output written to `out/<filename>.ply`.
````

## Cadence

Two-pass review per example (per `feedback_one_at_a_time` and
`feedback_one_at_a_time_review`):

1. **Numbers pass (Claude)** — runs the example, verifies the numerical
   anchors listed in the example's README. For measurement, SDF, and
   lattice examples authored under `feedback_math_pass_first_handauthored`,
   anchors are `assert_*` calls in `src/main.rs` and a clean exit-0
   IS the correctness signal.
2. **Visuals pass (user)** — opens the PLY in an external viewer,
   confirms the visual matches expectations. Optional but recommended
   for the lattice + showcase examples where the surface geometry IS
   part of the pedagogy.

Examples are reviewed individually before the next one lands. Multiple
examples bundle into one PR per `feedback_pr_size_ci_economics`.
