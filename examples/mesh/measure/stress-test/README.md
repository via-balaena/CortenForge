# measure — Stress Test (mesh-measure validation superset)

Headless validation of the full `mesh-measure` public surface against
closed-form and analytic oracles. No window, no Bevy — self-gating
assertions that abort (exit 101) on any mismatch, so `cargo xtask
run-validators` runs it red-or-green.

Folded from three former per-surface examples
(`mesh-measure-bounding-box`, `-cross-section`, `-distance-to-mesh`),
each now a module preserving its hand-authored fixture and oracle
checks verbatim. One domain → one stress-test.

## Modules

### `bounding_box` — `dimensions` + `oriented_bounding_box`
Two-shape fixture (16 verts / 24 tris): an axis-aligned 10 mm cube +
a 20×10×10 mm brick tilted 45° about Z. Validates AABB `Dimensions`
(extents, center, diagonal, `bounding_volume`, `aspect_ratio`,
`is_cubic`), the PCA-derived OBB on the combined mesh, and OBB extent
recovery on the brick alone (`(20, 10, 10)` from a non-cubic input).
Headline: the folk-intuition "OBB ⊆ AABB" is **false** — OBB corners
extend outside the AABB envelope by `half_extent · sin(θ)`.

### `cross_section` — `cross_section(s)` + `circumference_at_height` + `area_at_height`
32-segment closed UV-cylinder (66 verts / 128 tris), radius 5 mm,
height 10 mm. Validates a single mid-slice, a 10-slice stack,
convenience helpers, out-of-mesh handling (empty slice), and
plane-normal auto-normalization. Anchors target the polygon shoelace
area/perimeter (NOT analytical π·r²), and the **biased** naive-average
centroid (`sum / N` over chain-closure-duplicated points) is anchored
to literal precision — see `mesh/mesh-measure` for the shoelace-weighted
centroid candidate.

### `distance` — `measure_distance` + `closest_point_on_mesh` + `distance_to_mesh` + Hausdorff
Two vertex-disjoint unit cubes (`[0,1]³` and `[2,3]³`). Validates
point-to-point (3-4-5 Pythagorean), point-to-mesh across 6 face + 8
corner + 1 center queries, the empty-mesh edge case, and symmetric
Hausdorff = `sqrt(12)`. Headline: Hausdorff is **composition** over the
exposed primitives, not a built-in; the farthest vertex clamps to the
`(2,2,2)` corner, NOT the face-interior projection `(2,0,0)` (outside
the +X face's `(y,z) ∈ [2,3]²` bounds).

## Run

```
cargo run -p example-measure-stress-test --release
```

Expected: each module prints its `PASS` anchors and the binary exits 0.

## Visual artifacts

Each module writes a human-inspectable ASCII PLY to `out/` for the
`cf-viewer` visual-review path:

```
cargo run -p cf-viewer --release -- examples/mesh/measure/stress-test/out/mesh.ply       # bounding_box fixture
cargo run -p cf-viewer --release -- examples/mesh/measure/stress-test/out/cylinder.ply    # cross_section fixture
cargo run -p cf-viewer --release -- examples/mesh/measure/stress-test/out/two_cubes.ply   # distance fixture
```
