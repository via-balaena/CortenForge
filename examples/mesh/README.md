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
The workspace's unified viewer is **`cf-view`** —
`cargo run -p cf-viewer --release -- <path/to/file.ply>` renders the
PLY with flat-per-triangle WYSIWYP shading + auto-detected per-vertex
scalars (colormapped via Q5's heuristic per `docs/VIEWER_DESIGN.md`)
+ orbit camera. Pass `--up=+X|+Y|+Z` to match the input frame
(workspace default is `+Z`, matching the build-plate orientation
used by the printability and lattice examples).

```text
cargo run -p cf-viewer --release -- examples/mesh/<example>/out/<file>.ply
```

cf-view handles both geometry-only meshes and per-vertex scalar PLYs
(e.g. `extras["signed_distance"]` from `mesh-sdf-distance-query`)
out of the box — the dropdown auto-discovers scalar names and the
colormap fires by distribution heuristic (divergent / sequential /
categorical). Cavity-bearing fixtures (printability hollow-box,
trapped-volume) have **REVERSED-wound** inner shells; cf-view's
double-sided PBR materials render both sides without back-face
culling, so cavities stay visible by default.

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
| [`offset/stress-test`](offset/stress-test/) | SDF + marching cubes offset (both directions) — Steiner-Minkowski corner rounding outward vs. sharp polytope preservation inward, plus the grid-alignment topology pitfall; the `mesh-offset` domain validator |

### `mesh-shell` — wall-thickness shell generation

| Example | Concept |
|---------|---------|
| [`shell-generation/stress-test`](shell-generation/stress-test/) | The `mesh-shell` validation superset (one domain → one stress-test). Two modules: **fast** (normal-based, open-topped box — vertex correspondence preserved, `1/√k` corner thickness skew) and **high_quality** (SDF + marching cubes, closed cube — uniform wall thickness, Steiner-Minkowski rounding). Both consistently wound |

### `mesh-measure` — geometric measurement

| Example | Concept |
|---------|---------|
| [`measure/stress-test`](measure/stress-test/) | The `mesh-measure` validation superset (one domain → one stress-test). Three modules: **bounding_box** (AABB + PCA-OBB on a cube + 45°-rotated brick — OBB ⊄ AABB), **cross_section** (planar slicing of a 32-segment cylinder — shoelace area/perimeter, biased centroid, out-of-mesh, plane-normal normalization), **distance** (point-to-point + point-to-mesh + symmetric Hausdorff between two vertex-disjoint cubes) |

### `mesh-sdf` — signed-distance queries

| Example | Concept |
|---------|---------|
| [`mesh-sdf-distance-query`](mesh-sdf-distance-query/) | Numerical SDF on a unit octahedron — `SignedDistanceField` cached queries + free-fn one-shots + direct primitives (`closest_point_on_triangle`, `ray_triangle_intersect`, `point_segment_distance_squared`) + `point_in_mesh` ray-casting + 1000-point grid PLY with `extras["signed_distance"]` |

### `mesh-printability` — manufacturability validation

| Example | Concept |
|---------|---------|
| [`printability/stress-test`](printability/stress-test/) | The `mesh-printability` validation superset (one domain → one stress-test). Seven modules: **long_bridge** (§6.2 H-shape emit-vs-silent-drop + cross-detector co-flag + SLS policy-skip), **trapped_volume** (§6.3 cube + sphere cavity — analytical `(4/3)πr³` volume + per-tech Info/Critical fork), **self_intersecting** (§6.4 two interpenetrating cylinders — `max_reported` cap biconditional + convex no-false-positive control), **small_feature** (§6.5 cube + 0.2 mm burr — SmallFeature↔ThinWall convergence + negative controls), **orientation** (Gap L leaning cylinder — `with_build_up_direction`/`apply_orientation` equivalence + closed-form overhang-centroid envelope), **technology_sweep** (one hollow box under FDM/SLA/SLS/MJF — cross-tech severity matrix; its FDM slice is the canonical hollow-box §6.1 ThinWall oracle), **showcase** (capstone — 528 v / 1032 f five-shell bracket exercising all six detectors at once, `>=` predicates absorbing co-flag drift) |

### `mesh-lattice` — TPMS, strut, and composite lattices

| Example | Concept |
|---------|---------|
| [`lattice/stress-test`](lattice/stress-test/) | The `mesh-lattice` validation superset (one domain → one stress-test). Five modules: **tpms_gyroid** (TPMS implicit surface — `gyroid`/`density_to_threshold`/`make_shell` anchors then `generate_lattice`, every MC vertex on the analytical shell surface), **strut_cubic** (cubic struts — closed-form strut/cone volumes, `combine_struts` no-weld, the 3MF `BeamLatticeData` precursor = 216 grid nodes + 540 beams), **density_gradient** (the full `DensityMap` enum + per-beam `r1 = strut_thickness/2 × √density` modulation on an octet-truss), **shape_bounded** (analytical-SDF clip via `with_shape_sdf` — gyroid trimmed to a sphere, `is_outside_shape` predicate + confinement invariant), **mesh_bounded_infill** (composite FDM `generate_infill` on a 50 mm cube — shell + lattice + caps + connections, resolution-independent Euler `2V−F=8` shell anchor + §Q-5 winding guard) |

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
