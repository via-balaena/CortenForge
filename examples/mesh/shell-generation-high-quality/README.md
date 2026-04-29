# shell-generation-high-quality

**SDF-based shell generation around a closed cube.** Given a closed
triangle mesh and a desired wall thickness,
`mesh-shell::ShellBuilder` with the `.high_quality()` preset produces
a watertight, manifold, printable shell whose outer surface is the
SDF level set at distance `wall_thickness` from the input,
re-triangulated by marching cubes. The level-set math gives **uniform
perpendicular wall thickness** at every wall-interior point, modulo
half-voxel MC discretization noise. Companion to
[`shell-generation-fast`](../shell-generation-fast/) (commit 11),
which uses a per-vertex normal offset — fast, but with
triangulation-skewed perpendicular thickness at corners (33% thinning
at the worst-case vertex).

## What it does

Builds a closed 10mm cube (8 verts, 6 walls × 2 = 12 triangles, same
diagonal-triangulation pattern as the open-topped box in
[`shell-generation-fast`](../shell-generation-fast/) plus 2 top
faces to close it; `signed_volume == 1000` exactly), runs it through
the `ShellBuilder` fluent API with
`.high_quality().voxel_size(0.3)` and a `wall_thickness` of 2mm, and
saves two PLY artifacts:

| File | Description |
|---|---|
| `out/before.ply` | Input closed cube. 8 verts, 12 faces, AABB `[0, 10]³`, watertight, manifold, outward-wound, `signed_volume == 1000mm³` exactly. |
| `out/shell.ply`  | Generated shell. ~12.5k verts (8 inner + ~12.5k outer-MC-welded), ~25k faces (12 inner + ~25k outer + 0 rim), watertight + manifold + printable + **`has_consistent_winding == true`** (bonus property of closed-input SDF shells; strict improvement over commit 11's open-input rim-winding quirk). |

The example anchors the load-bearing pedagogy of `.high_quality()`:
**uniform perpendicular wall thickness everywhere on the shell,
regardless of input triangulation, modulo cell-scale MC discretization
noise.** Plus the design trade-off the SDF method buys it with —
vertex-soup-then-welded outer (no 1:1 vertex correspondence with the
input) and Steiner-Minkowski rounding at sharp creases.

## API surface — `ShellBuilder` `.high_quality()`

The chain in this example reads:

```rust
let result = ShellBuilder::new(&before)
    .wall_thickness(2.0)   // mm
    .high_quality()        // preset: wall_method=Sdf, voxel=0.3, validate=true
    .voxel_size(0.3)       // re-asserts the preset's default; documents intent
    .build()?;
```

The relevant fields the preset sets:

| Method | What it sets | Notes |
|---|---|---|
| `.high_quality()` (preset) | `wall_method = Sdf`, `sdf_voxel_size_mm = 0.3`, `validate = true` | Headline preset. Both wall-method AND validation are forced; downstream `.voxel_size(...)` and `.validate(...)` overrides apply only AFTER the preset (order matters; same as commit 11's `.fast().validate(true)` story). |
| `.voxel_size(f64)` | `sdf_voxel_size_mm` | Marching-cubes cell size. Smaller = finer detail + more triangles + more memory. **Default after `.high_quality()` is 0.3mm.** Applied AFTER the preset to override; if applied BEFORE, `.high_quality()` resets it to 0.3. |

`ShellBuildResult` for closed-input SDF shells has the fields below
populated. **Critical layout difference vs `WallGenerationMethod::Normal`:**
no 1:1 vertex correspondence between inner and outer surfaces. The
outer surface is independently re-triangulated by marching cubes on
the SDF, then welded (per the engine fix in commits 11.5.1 +
11.5.2), so `outer_vertex_count` is independent of `inner_vertex_count`
and depends on the voxel size + the level set's surface area:

```rust
pub struct ShellGenerationResult {
    pub inner_vertex_count: usize,        // 8 (input verts)
    pub outer_vertex_count: usize,        // ~12.5k for our 10mm cube + 2mm offset + 0.3mm voxel
    pub rim_face_count: usize,            // 0 — closed input has no rim
    pub total_face_count: usize,          // 12 (inner) + ~25k (outer) + 0 (rim)
    pub boundary_size: usize,             // 0 — closed input
    pub validation: Option<ShellValidationResult>,  // always Some(...) under .high_quality()
    pub wall_method: WallGenerationMethod, // Sdf
}
```

This example asserts: `wall_method == Sdf`, `boundary_size == 0`,
`rim_face_count == 0`, `outer_vertex_count > 10 × inner_vertex_count`
(MC re-triangulation), `outer vert/face ratio < 1.0` (welded — soup
would be 3.0; the 11.5.2 fix's `weld_vertices + remove_unreferenced`
pass gives ~0.5), `is_watertight && is_manifold && is_printable() &&
has_consistent_winding`, plus the load-bearing wall-thickness
uniformity assertion (24 sample points, drift below
`WALL_TOLERANCE_MM`).

## Uniform perpendicular wall thickness

The headline pedagogy of `.high_quality()`:

> **The SDF level set produces uniform perpendicular wall thickness
> everywhere on the shell, regardless of input triangulation, modulo
> half-voxel MC discretization noise.**

The math: the level set of a closed mesh at signed distance `d` is
the locus of all points whose closest mesh-surface point is exactly
distance `d` away. So *by construction*, every point ON the level set
is exactly `d` away from the input. Conversely, every wall-interior
point on the input mesh is exactly `d` away from the closest
level-set point along the outward normal — provided the sample is
well clear of edges/vertices where the level set rounds (see
[Steiner-Minkowski rounding](#steiner-minkowski-rounding) below).

**Concrete numerical contrast vs commit 11.** Same input triangulation,
same wall thickness, same vertex 0–7 indexing. From the
[`shell-generation-fast` per-vertex offset table](../shell-generation-fast/README.md#per-vertex-offset-derivation):

| Vert | Position | Commit 11 thinnest perpendicular | Commit 12 nearest interior sample | Commit 12 measured |
|---|---|---|---|---|
| 0 | `(0, 0, 0)`   | `2/√3 ≈ 1.155mm` to all 3 incident walls (symmetric, k=3) | bottom-wall sample at `(3, 3, 0)` | `2.0000mm` |
| 2 | `(10, 10, 0)` | **`2/3 ≈ 0.667mm`** to right wall (asymmetric, worst-case 33% thinning) | bottom-wall sample at `(7, 7, 0)` | `2.0000mm` |
| 5 | `(10, 0, 10)` | `2/√5 ≈ 0.894mm` to right wall (asymmetric) | front-wall sample at `(7, 0, 7)` | `2.0000mm` |

The full per-vertex breakdown is in commit 11's README; commit 12's
numerical pass measures distance-to-outer at 24 wall-interior sample
points (4 per wall × 6 walls, displaced 3mm from every cube edge so
each sample is well clear of the 2mm-radius fillet zone) and confirms
each one lands within `WALL_TOLERANCE_MM` of the parameterized
`WALL_THICKNESS_MM`. On the author's machine: **min = 2.0000mm,
max = 2.0000mm, max |drift| = 0.0000mm**. The MC tessellation of the
level set's flat regions (parts of the offset cube's 6 outer faces)
are bit-exact level-set planes; brute-force closest-point on the
outer mesh recovers the perpendicular distance to within IEEE-754
arithmetic precision in those regions.

The 24-sample assertion is enforced via `mesh_sdf::SignedDistanceField`
on an outer-only mesh extracted from the assembled shell (see
`extract_outer_only` in `src/main.rs`). The full `mesh_sdf` query
cost is ~24 samples × ~25k MC outer faces ≈ 600k face checks +
closest-point arithmetic — sub-second in release mode.

## Steiner-Minkowski rounding

The cost paid for uniform perpendicular thickness: the cube's sharp
creases — its 12 EDGES and 8 VERTICES — round into the level set.

**At edges:** the level set forms cylindrical fillets of radius
`wall_thickness` around each cube edge. The closest level-set point
from any point on the edge is at distance `wall_thickness` perpendicular
to the edge axis, in any direction in the plane spanned by the two
adjacent face normals. The 12 edges of our 10mm cube produce 12
quarter-cylinder fillets, each running 10mm long with radius 2mm.

**At vertices:** the level set forms sphere octants of radius
`wall_thickness` at each cube vertex. The closest level-set point
from a cube vertex is at distance `wall_thickness` along the body
diagonal, but every direction in the sphere octant is also exactly
`wall_thickness` away. The 8 vertices of our cube produce 8 sphere
octants of radius 2mm.

This rounding is **GENUINE level-set curvature, not MC discretization
artifact.** The level set ITSELF is curved at edges and vertices; the
MC tessellation discretizes the curved level set into triangle facets
at voxel resolution. Distinguish from
[`mesh-offset-inward`](../mesh-offset-inward/) (commit 10), where the
level set has SHARP corners (inward offset of a convex polytope
preserves polytope structure, no rounding) and MC chamfers them as a
tessellation artifact only.

So the corner geometry on the outer surface of `out/shell.ply` is
**filleted from the level-set math** AND **chamfered from MC
tessellation**, layered. At voxel size 0.3mm and fillet radius 2mm,
the dominant visual effect is the level-set fillet (~6.7×
larger than voxel size); the MC chamfering adds cell-scale stair-steps
at sub-millimeter scale visible only on close inspection.

## Cross-section at mid-height

A clean numerical signature of uniform thickness: extract a horizontal
cross-section through the shell at `z = side / 2 = 5mm`. At
mid-height, both inner and outer surfaces are at constant z (well
clear of the top/bottom rounding zones), so the cross-section is a
pure 2D Steiner-Minkowski offset — an inner 10×10 square ring plus
an outer 14×14-square-with-2mm-rounded-corners ring.

```
contour_count  : 2 (one inner ring + one outer ring)
perimeter (sum): 92.5491 mm
area           : 92.5079 mm²
bounds         : (-2.000, -2.000, +5.000) → (+12.000, +12.000, +5.000)
```

Analytical perimeters:

- **Inner ring** (10×10 square): `4 × side = 40mm`
- **Outer ring** (14×14 square with 2mm-radius corner fillets):
  `4 × side + 4 × (2π × wall_thickness / 4) = 40 + 4π ≈ 52.566mm`
- **Sum**: `8 × side + 2π × wall_thickness ≈ 80 + 4π ≈ 92.566mm`

(matches measured 92.5491mm within MC discretization)

Analytical annular area:

- **Outer** (14×14 bounding box minus the 4 corner squares not in
  the level set, plus the 4 quarter-circles of radius `wall_thickness`
  that ARE in the level set): `(side + 2 × wall_thickness)² - 4 ×
  wall_thickness² + π × wall_thickness² = 196 - 16 + 4π ≈ 192.566mm²`
- **Inner**: `side² = 100mm²`
- **Annular wall** = outer − inner = `4 × side × wall_thickness + π
  × wall_thickness² ≈ 80 + 4π ≈ 92.566mm²` (4 strips of length `side`
  × thickness `wall_thickness` for the wall faces, plus 4 corner
  quarter-circles of radius `wall_thickness` totalling a full circle;
  ≈ measured 92.5079mm² within MC discretization)

The wall RING is uniformly 2mm thick around the cross-section,
exactly because every point on the inner ring is exactly
`wall_thickness` away from the closest outer-ring point. The
2mm-radius corner fillet on the outer ring matches the level-set
geometry precisely.

## Numerical anchors

**Before (`closed_cube`):**

- `report.vertex_count == 8`, `report.face_count == 12`
- `report.is_manifold && report.is_watertight && !report.is_inside_out`
- `report.boundary_edge_count == 0`, `report.non_manifold_edge_count == 0`
- `aabb.min == (0, 0, 0)`, `aabb.max == (10, 10, 10)` exactly
- **`signed_volume == 1000mm³` exactly** (= `side³`; closed cube has
  clean divergence-theorem volume, unlike commit 11's open-topped box
  which gave a half-integral 666.67)

**Shell — topological structure (`ShellGenerationResult`):**

- `inner_vertex_count == 8`, `outer_vertex_count > 10 × 8` (MC
  re-triangulation; on the author's machine, `outer_vertex_count == 12546`)
- `boundary_size == 0` (closed input has no boundary loops; SDF rim
  short-circuits)
- `rim_face_count == 0` (no boundary → no rim quads)
- `total_face_count == 12 + outer_face_count` (no rim term)
- `wall_method == Sdf`
- `result.offset_applied == false` (no `.offset(...)` call in the chain)
- **Outer vert/face ratio < 1.0** (`outer_vertex_count / outer_face_count`
  ≈ 0.5 for the run on the author's machine; soup would be 3.0; this
  assertion catches any future regression that drops the 11.5.2 weld
  pass).

**Shell — validation (`shell_stats.validation`):**

- `is_watertight == true`, `is_manifold == true`, `is_printable() == true`
- `has_consistent_winding == true` — **bonus property of closed-input
  SDF shells**. Strict improvement over commit 11's open-input
  `has_consistent_winding == false` rim-winding quirk: closed input
  has no rim, so no edge-direction conflict between rim quads and
  reversed inner faces.
- `boundary_edge_count == 0`, `non_manifold_edge_count == 0`
- `issues.is_empty()` — no ShellIssue variants flagged.

**Shell — properly wound in face-normal sense:**

- `report.is_inside_out == false` — the 11.5.2 fix's per-face
  `face.swap(1, 2)` flips MC's inside-out winding before assembling
  the shell.
- `signed_volume > 0` (~1608.30 mm³ on the author's machine; matches
  Steiner-Minkowski analytical `1000 + 1200 + 120π + 32π/3 ≈
  2610.51 mm³` outer cube minus 1000 mm³ inner cube ≈ 1610.51 mm³,
  within MC discretization undershoot).

**Shell — wall thickness uniformity (THE primary anchor):**

- 24 wall-interior sample points (4 per wall × 6 walls, at `(3, 3)`,
  `(7, 3)`, `(3, 7)`, `(7, 7)` in each wall's local 2D coords; 3mm
  from every cube edge → well clear of the 2mm-radius fillet zone).
- For each sample, `mesh_sdf::SignedDistanceField::unsigned_distance`
  on the outer-only mesh returns `wall_thickness ± WALL_TOLERANCE_MM`.
- Empirically: **min = 2.0000mm, max = 2.0000mm, max |drift| = 0.0000mm**
  (tolerance 0.2mm — half-voxel + cushion). Brute-force closest-point
  on flat MC tessellation regions recovers the perpendicular distance
  to IEEE-754 arithmetic precision; the tolerance allows for cell-scale
  drift in regions adjacent to the level-set fillets, which our 3mm
  sample displacement avoids by construction.

**Cross-section at z = 5mm:**

- `contour_count == 2` (one inner ring + one outer ring)
- `perimeter ≈ 8 × side + 2π × wall_thickness ≈ 92.566mm`
- `area ≈ 4 × side × wall_thickness + π × wall_thickness² ≈ 92.566mm²`
  (printed but not tightly asserted; depends on
  contour-chain orientation)

**PLY round-trip:**

- `out/before.ply` reloads with vertex/face counts matching the
  in-memory closed cube (8v, 12f).
- `out/shell.ply` reloads with vertex/face counts matching the
  in-memory shell (~12.5k v, ~25k f; exact counts depend on voxel
  size and platform — anchored as ratio + threshold, not absolute).

These anchors are drift catchers for the `.high_quality()` preset's
contract: if `mesh-shell` ever changes the SDF voxel default,
removes the weld pass, drops the per-face flip, regresses the
boundary-precondition fallback, or breaks the level-set perpendicular
geometry, this example fails loudly and the README sections shift.

## Visuals

Open the artifacts in MeshLab, ParaView, Blender, or `f3d`:

- **`out/before.ply`** — a solid 10mm cube. Six flat 10×10 walls,
  closed (no boundary loop). Sharp 90° edges throughout. Diagonal
  triangulation visible in wireframe view (one diagonal per wall, same
  pattern as commit 11's open-topped box plus the two top faces).
- **`out/shell.ply`** — a printable hollowed-out cube with **filleted
  outer corners + edges**. What you'll see:
  - **Outer surface is the SDF level set**, not a parallel offset.
    The outer-cube faces (~10×10 each on the +z, -z, +y, -y, +x, -x
    directions) are flat planes at perpendicular distance 2mm from
    the corresponding inner faces — uniform thickness regardless of
    input triangulation. Compare commit 11's outer surface, which
    skews per vertex with diagonals visible on every "flat" outer
    face.
  - **Cube edges are filleted.** The 12 cube edges round into
    cylindrical fillets of radius 2mm (Steiner-Minkowski); each
    quarter-cylinder runs 10mm long with the rounding direction
    spanning the two adjacent face normals. This is GENUINE
    level-set curvature; the MC tessellation discretizes the curved
    fillet into many small planar facets at 0.3mm cell resolution.
  - **Cube vertices are filleted into sphere octants** of radius 2mm.
    Each octant is tessellated into many small chamfered planar
    facets at 0.3mm voxel resolution.
  - **No `_flipped.ply` companion is needed.** Unlike
    [`mesh-offset-outward`](../mesh-offset-outward/) and
    [`mesh-offset-inward`](../mesh-offset-inward/) (where the raw MC
    output IS inside-out, requiring per-face flip remediation), the
    shell here is properly wound by construction (the 11.5.2 engine
    fix applies the per-face flip internally). All viewer pipelines
    render the shell correctly: backface-culled, two-sided, lit, etc.
    The `has_consistent_winding == true` bonus also means downstream
    tools that BFS the face graph (e.g. `mesh_repair::fix_winding_order`)
    behave consistently.
  - **Cell-scale chamfering at the filleted regions.** On close
    inspection, each filleted edge/corner shows a stair-step pattern
    of small planar facets at 0.3mm resolution. This is the MC
    tessellation discretizing the curved level set; for finer
    chamfering, reduce the voxel size (e.g.
    `.high_quality().voxel_size(0.15)` for ~4× more triangles, ~2×
    finer chamfering).
  - **Inner cavity walls** are exactly the input cube's faces, with
    winding-flipped triangles so cavity normals point inward
    (= outward from wall material). 12 inner-cavity faces, sharp 90°
    corners — no rounding here, since the level-set rounding is on
    the OUTER surface only.

A future `cross-section-sweep` example (commit 14) will visualize
the uniform-thickness annular cross-section at multiple heights with
PLY contour stacks; this commit anchors the numerical readout only
(stdout printout shows the z=5 mid-height cross-section).

## Run

```
cargo run -p example-mesh-shell-generation-high-quality --release
```

Output written to `examples/mesh/shell-generation-high-quality/out/`.
