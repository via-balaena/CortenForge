# mesh-sdf-distance-query

**Oracle-decomposition walkthrough — the two orthogonal halves of a
signed-distance query, on a well-formed fixture and a pathological one.**
Post-D-arc, `mesh-sdf` splits signed-distance into `TriMeshDistance`
(parry BVH-backed *unsigned* distance) and a *sign* oracle
(`PseudoNormalSign` or `FloodFillSign`), composed via `Signed<D, S>`.
This example builds both compositions on two contrasting fixtures and
prints what each oracle reports, so you can see where a cheap sign oracle
suffices and where it silently fails.

This is a **runnable demonstration**, not a test: it builds fixtures,
runs both oracles, prints the results, and writes cf-view PLYs. The
numerical oracles it illustrates are asserted in `mesh-sdf`'s own library
tests (see [Correctness lives in the library](#correctness-lives-in-the-library)).

## Fixture A — well-formed octahedron (both oracles agree)

A unit octahedron (6 vertices at `(±1, 0, 0)`, `(0, ±1, 0)`, `(0, 0, ±1)`,
8 triangle faces, one per octant) is the L1-unit-ball `|x| + |y| + |z| ≤ 1`.
Its signed distance has the closed form `(|x| + |y| + |z| − 1) / √3` for
any query whose closest point is in a face interior, and clamps to the
nearest vertex otherwise.

**Why an octahedron, not a sphere**: 8 triangles with FP-exact integer
vertex coordinates and an analytically-known SDF — every value is
tractable to derive and bit-stable across libms. A UV-tessellated sphere
would carry `1 − 1/√3 ≈ 0.42` chord error at face centers.

On this clean fixture both sign oracles agree everywhere. The example:

- Composes `Signed<TriMeshDistance, PseudoNormalSign>` and prints the
  distance / unsigned-distance at a generic interior point, a face-center
  direction, and a vertex-direction exterior probe.
- Composes `Signed<TriMeshDistance, FloodFillSign>` via the
  `flood_filled_sdf` convenience constructor.
- Evaluates a 10 × 10 × 10 = 1000-point grid in `[-2, 2]³` through both
  oracles and reports their inside-counts and disagreement count (0 on a
  clean mesh), writing `out/octahedron_sdf_grid.ply` for cf-view.

## Fixture B — inverted-cap pyramid (the load-bearing demo)

A square-base pyramid with the **base winding inverted** so its outward
normal points `+Z`, *into* the body — mimicking the pre-B cf-scan-prep
auto-cap failure where `auto_cap_open_boundaries` emitted inward-pointing
cap-fan triangles.

A `PseudoNormalSign` query far below the base (`(0, 0, -3)`) finds the
base as the closest face, computes `(probe − base) · (+Z) = -3 < 0`, and
reports **INSIDE** — wrong. `FloodFillSign` labels points by topological
reachability from the bounds corners and correctly reports the probe
**outside**. The example probes four points (interior, far-below,
far-above, lateral) through both oracles and prints the per-probe
comparison: pseudo-normal disagrees on exactly the far-below probe;
flood-fill is correct everywhere.

This is why `cf-cast-cli` and `cf-device-design` ship the flood-fill sign
defense on cleaned body-part scans.

## Correctness lives in the library

Every numerical/behavioral oracle this walkthrough illustrates is asserted
in `mesh-sdf`'s library tests — this example demonstrates them, it does
not re-assert:

- **Octahedron closed-form SDF** (generic interior + 6 face-centers +
  vertex probe: `distance` / `unsigned_distance` / `is_inside` /
  `closest_point`) → `mesh-sdf` `src/sdf.rs`
  `signed_distance_matches_closed_form_l1_ball_on_octahedron` (same
  fixture, same probes, same values).
- **Inverted-cap rescue** (pseudo-normal wrong / flood-fill right below an
  inverted base) → `mesh-sdf` `src/flood_fill.rs`
  `pseudo_normal_wrong_flood_fill_right_on_inward_cap`.
- **Flood-fill contracts** (one inside component on a clean mesh; both
  sign oracles agree on a well-formed mesh) → `flood_fill.rs`
  `flood_fill_sign_correct_on_closed_pyramid`,
  `flood_fill_matches_pseudo_normal_on_outward_cap`.

## Visuals

`out/octahedron.ply` (ASCII, 6 verts + 8 tris) — open in cf-view to check
the face windings:

```text
cargo run -p cf-viewer --release -- examples/mesh/mesh-sdf-distance-query/out/octahedron.ply
```

`out/octahedron_sdf_grid.ply` (1000-vertex point cloud) carries three
per-vertex scalars — `pseudo_normal_signed` and `flood_fill_signed`
(signed distance via each oracle; cf-view auto-picks a divergent
blue→red colormap) and `oracle_disagreement` (categorical 0/1, uniformly
0 on this clean fixture). The dropdown switches between them:

```text
cargo run -p cf-viewer --release -- examples/mesh/mesh-sdf-distance-query/out/octahedron_sdf_grid.ply
```

## Run

```text
cargo run -p example-mesh-mesh-sdf-distance-query --release
```

Stdout prints Fixture A's queried distances + bulk-grid inside-counts and
Fixture B's per-probe oracle comparison. Output PLYs land in `out/`.

## Cross-references

- **Oracle-decomposition background**:
  `docs/MESH_SDF_ORACLE_DECOMPOSITION_SPEC.md`.
- **Bridge consumer of the same trait**:
  `examples/sim-soft/sdf-bridge/stress-test/`'s `mesh_scan` module (a
  mesh-derived `Signed<TriMeshDistance, PseudoNormalSign>` used as a
  `cf_design::Sdf`).
- **PLY-attribute pattern**: `examples/mesh/ply-with-custom-attributes/`
  — the `save_ply_attributed` + `extras["<scalar>"]` pattern this reuses
  for the grid output.
- **Mesh book**: `docs/studies/mesh_architecture/src/80-examples.md`.
