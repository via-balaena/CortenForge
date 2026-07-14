# printability — Stress Test (mesh-printability validation superset)

Headless validation of the v0.8 `mesh-printability` detector inventory
against closed-form, analytic, and per-technology-severity oracles. No
window, no Bevy — self-gating assertions that abort (exit 101) on any
mismatch, so `cargo xtask run-validators` runs it red-or-green.

Folded from eight former per-crate examples (`printability-long-bridge`,
`-trapped-volume`, `-self-intersecting`, `-small-feature`,
`-orientation`, `-technology-sweep`, `-showcase`), each now a module
preserving its hand-authored fixture and oracle checks verbatim. One
domain → one stress-test.

The former `printability-thin-wall` example was **dropped** in this
fold: its FDM-only `ThinWall` oracle (two clusters, centroids, 0.4 mm
top-wall thickness, Critical severity, a trapped-volume, an overhang
co-flag) is a strict subset of `technology_sweep`'s FDM slice, which
asserts the same facts more tightly (overhang `== 1` vs `!is_empty`) on
a near-identical hollow box, plus three more technologies. No coverage
was lost — the oracles are analytical, so the differing box dimensions
add nothing.

## Modules

### `long_bridge` — `LongBridge` (§6.2)
24-vertex / 44-triangle H-shape (two 5×5×18 mm pillars + a 35×5×2 mm
lintel). The 20 mm middle bridge emits `Critical` while the two 5 mm
cantilevers silent-drop below `max_bridge_span`; endpoints and span
pinned to 1e-6. Headline: the bridge midpoint `(15, 2.5, 18)`
coincides with the middle overhang centroid (cross-detector co-flag),
and the SLS pass proves a technology-policy silent-skip pushes **zero**
`DetectorSkipped` (distinct from a precondition skip).

### `trapped_volume` — `TrappedVolume` (§6.3)
20 mm cube with a sealed 5 mm-radius UV sphere cavity (490 verts / 972
tris). Validates the voxel-discretized cavity volume against analytical
`(4/3)·π·r³ ≈ 523.6 mm³` (±10% cross-platform band) and the centroid to
sub-voxel epsilon. Headline: the **same geometry** classifies `Info` on
FDM (extrusion prints through) but `Critical` on SLA/SLS/MJF (resin /
powder trap) — and the driver of `is_printable() == false` differs per
technology (overhang on FDM, trapped-volume on SLS/MJF where overhangs
silent-skip).

### `self_intersecting` — `SelfIntersecting` (§6.4)
Two interpenetrating cylinders (68 verts / 128 tris), no boolean
cleanup. Validates the `max_reported = 100` cap **biconditional**
(truncation suffix present iff the cap is hit), the `face_a < face_b`
canonical ordering on every pair, and mandatory `Critical` severity.
Paired with a convex single-cylinder control that must flag **zero** —
the no-false-positive floor.

### `small_feature` — `SmallFeature` (§6.5)
30 mm cube + a 0.2 mm hex-prism burr (22 verts / 36 tris). Headline:
the same burr is caught by two detectors from complementary angles —
`SmallFeature` (Warning, advisory, does **not** block printability) and
`ThinWall` (Critical, blocks it) — so `is_printable()` is driven by the
co-flag. The detector reads the vertex-to-vertex AABB extent (0.2 mm),
not the flat-to-flat 0.173 mm. Negative controls: the cube must **not**
flag (`== 1` feature) and overhangs are **zero**.

### `orientation` — `build_up_direction` (Gap L)
Leaning cylinder (66 verts / 128 tris) at 60° from vertical, revalidated
under three runs: original, manual `R_Y(-60°)` mesh rotation
(`apply_orientation`), and `with_build_up_direction` set to the axis.
Proves the two exact routes (rotate the mesh vs rotate the validator's
up-vector) drive overhang to exactly 0, and pins the Run-1 overhang
centroids to a closed-form envelope (radial band `[R·cos22.5°, R]` +
downhill azimuth arc).

### `technology_sweep` — cross-technology severity matrix
One 25×20×15 mm hollow box with a 0.4 mm top wall, validated under
FDM/SLA/SLS/MJF. Each technology fails `is_printable()` for a different
reason: FDM `ThinWall` Critical, SLA thin-wall not-flagged (strict
less-than boundary `0.4 < 0.4` is false), SLS/MJF `ThinWall` Warning;
trapped-volume `Info` on FDM / `Critical` elsewhere; overhang fires on
FDM/SLA and silent-skips on the support-free technologies. Its FDM slice
is the canonical hollow-box `ThinWall` oracle for the domain.

### `showcase` — capstone (all six detectors)
A 528-vertex, 1032-triangle bracket of five vertex-disjoint shells
(body + leaning wing + thin-lip slab + hex burr + reversed-wound sphere
cavity) exercises `ThinWall`, `SmallFeature`, `TrappedVolume`,
`ExcessiveOverhang`, `LongBridge`, and `NotWatertight` at once. Uses
`>=` predicates by design to absorb cross-detector co-flag drift on a
realistic multi-part mesh; a math-pass-first `verify_fixture_geometry`
locks every hand-authored vertex / normal to 1e-12 so an exit-0 run
equals a clean visual inspection.

## Run

```
cargo run -p example-printability-stress-test --release
```

Expected: each module prints its diagnostics + `PASS` anchors and the
binary exits 0.

## Visual artifacts

Each module writes human-inspectable ASCII PLY files to its own
`out/<module>/` subdirectory for the `cf-viewer` visual-review path,
e.g.:

```
cargo run -p cf-viewer --release -- examples/mesh/printability/stress-test/out/showcase/mesh.ply
cargo run -p cf-viewer --release -- examples/mesh/printability/stress-test/out/trapped_volume/mesh.ply
```
