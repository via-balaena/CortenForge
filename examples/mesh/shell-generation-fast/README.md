# shell-generation-fast

**Normal-based shell generation around an open-topped box.** Given a
triangle mesh and a desired wall thickness, `mesh-shell::ShellBuilder`
with the `.fast()` preset produces a watertight, manifold, printable
shell by duplicating each input vertex along its averaged unit normal
by `wall_thickness`, flipping the inner-surface winding so cavity
normals point inward, and connecting the inner and outer boundary
edges with a rim. Fast — no marching cubes, no SDF — at the cost of
**triangulation-dependent direction of the offset vector at each
vertex**, plus a known rim-winding quirk in `mesh-shell` 0.7.x.
Companion to [`shell-generation-high-quality`](../shell-generation-high-quality/)
(commit 12), which uses an SDF-based outer surface for
triangulation-independent perpendicular thickness at the cost of
vertex-soup-then-welded outer and Steiner-Minkowski rounding at
sharp creases.

## What it does

Builds an open-topped 10mm cube (8 verts, 5 walls × 2 = 10 triangles
with outward winding, top face missing so the 4 top edges form a
single boundary loop), runs it through the `ShellBuilder` fluent API
with `.fast().validate(true)` and a `wall_thickness` of 2mm, and
saves two PLY artifacts:

| File | Description |
|---|---|
| `out/before.ply` | Input open-topped box. 8 verts, 10 faces, AABB `[0, 10]³`, NOT watertight (`boundary_edge_count == 4`), manifold, outward-wound. |
| `out/shell.ply` | Generated shell. 16 verts (8 inner + 8 outer, 1-to-1 correspondence), 28 faces (10 inner + 10 outer + 8 rim), watertight + manifold + printable. The PLY anchors what `ShellBuilder::new(...).fast().validate(true).build()` actually returns. |

The example anchors three pedagogical points the rest of this README
unpacks: **vertex correspondence under the `.fast()` preset**,
**triangulation-dependent perpendicular wall thickness at corners**,
and the **`mesh-shell` 0.7.x rim winding quirk** (the shell is
printable but `has_consistent_winding == false`).

## API surface — `ShellBuilder` fluent chain

The recommended entry point. The chain in this example reads:

```rust
let result = ShellBuilder::new(&before)
    .wall_thickness(2.0)   // mm
    .fast()                // preset: wall_method=Normal, validate=false
    .validate(true)        // override: re-enable post-build validation
    .build()?;
```

Each method on `ShellBuilder` is `const` and `#[must_use]`; the
chain composes a `ShellParams` struct internally. The presets and
overrides interact in a specific order:

| Method | What it sets | Notes |
|---|---|---|
| `.wall_thickness(f64)` | `wall_thickness_mm` | Default `2.5`. The shell wall thickness in mesh units. |
| `.min_thickness(f64)` | `min_thickness_mm` | Default `1.5`. Validation threshold for "too thin to print". |
| `.validate(bool)` | `validate` | Default `true`. Toggles whether `shell_stats.validation` is `Some(...)` or `None`. |
| `.fast_walls()` | `wall_method = Normal` | Per-vertex normal offset (this example's path). |
| `.sdf_walls()` | `wall_method = Sdf` | Marching-cubes outer surface (commit 12's path). |
| `.voxel_size(f64)` | `sdf_voxel_size_mm` | Default `0.5`. Only used by `sdf_walls`. |
| `.fast()` (preset) | `wall_method = Normal`, `validate = false` | This example's headline preset. |
| `.high_quality()` (preset) | `wall_method = Sdf`, `sdf_voxel_size_mm = 0.3`, `validate = true` | Companion (commit 12). |
| `.offset(f64)` | `offset_mm = Some(...)` | Optional pre-pass: SDF offset the input before shell generation. Routes through `mesh-offset`. Not used by this example — see [`mesh-offset-outward`](../mesh-offset-outward/) and [`mesh-offset-inward`](../mesh-offset-inward/) for the standalone offset pipeline. |

**Order matters.** `.fast()` and `.high_quality()` are fix-up methods
that set MULTIPLE fields. If you want fast walls AND post-build
validation, write `.fast().validate(true)` (this example's path) —
the `.validate(true)` overrides the `validate = false` that `.fast()`
just set. Writing `.validate(true).fast()` would be wiped because
`.fast()` resets `validate` unconditionally.

The build returns a `ShellBuildResult`:

```rust
pub struct ShellBuildResult {
    pub mesh: IndexedMesh,
    pub offset_applied: bool,           // false here (no .offset call)
    pub shell_stats: ShellGenerationResult,
}

pub struct ShellGenerationResult {
    pub inner_vertex_count: usize,      // 8
    pub outer_vertex_count: usize,      // 8 (1-to-1 correspondence under Normal method)
    pub rim_face_count: usize,          // 8 (= 2 × boundary_size)
    pub total_face_count: usize,        // 28 (= 10 inner + 10 outer + 8 rim)
    pub boundary_size: usize,           // 4 (open top edges)
    pub validation: Option<ShellValidationResult>,  // Some(...) under .validate(true)
    pub wall_method: WallGenerationMethod,          // Normal
}
```

`ShellValidationResult` (the `validation` field) is RICHER than
`mesh_repair::validate_mesh`'s `MeshReport`: it adds
`has_consistent_winding` (edge-orientation BFS), `is_printable()`
(watertight && manifold), and a typed `Vec<ShellIssue>` enumerating
specific failure modes (`NotWatertight`, `NonManifold`,
`InconsistentWinding`, `EmptyShell`, `DegenerateTriangles`).

This example asserts: `is_watertight`, `is_manifold`,
`is_printable()`, AND `!has_consistent_winding` (the rim quirk —
see ["mesh-shell 0.7.x rim winding quirk"](#mesh-shell-07x-rim-winding-quirk)
below for the diagnosis).

## The vertex-correspondence pattern

The shell `IndexedMesh` is laid out so that:

```
shell.vertices[0..n]    = inner surface (positions = input)
shell.vertices[n..2n]   = outer surface (positions = input + averaged_unit_normal × wall_thickness)
```

where `n = inner_vertex_count`. For every `i in 0..n`:

- `shell.vertices[i + n] - shell.vertices[i]` is the offset vector at
  vertex `i`, **magnitude exactly `wall_thickness`** (within `1e-12`
  modulo IEEE-754 sqrt + division ulps). This is the universal
  invariant of the normal-based method — every vertex's offset has
  the same magnitude regardless of the input's triangulation.
- The face buffer mirrors this layout: `shell.faces[0..inner_face_count]`
  are the input faces with winding REVERSED (so cavity normals point
  inward, i.e. outward from the shell wall material); the next
  `inner_face_count` are the outer faces with original winding +
  index offset; the final `rim_face_count` are the rim quads. See
  [`mesh-shell` source `shell::generate::generate_shell_normal`](../../../mesh/mesh-shell/src/shell/generate.rs)
  for the layout in code.

This pattern survives because the normal-method outer surface
preserves the input's TOPOLOGY (same face count, same connectivity).
The SDF-based path (`.high_quality()`) does not — `mesh-offset`
runs marching cubes on the SDF and emits a vertex-soup mesh whose
vertex count + face count + connectivity all differ from the input.
Commit 12 anchors that contrast.

## Per-vertex offset derivation

The headline pedagogy of `WallGenerationMethod::Normal`, more
nuanced than the simple framing might suggest:

> **The offset vector's MAGNITUDE is universal (= `wall_thickness`).
> Its DIRECTION at each vertex depends on the input's triangulation.**

`generate_shell_normal` computes one averaged unit normal per inner
vertex by summing each incident face's UNIT normal once per face
(see [`compute_vertex_normals`](../../../mesh/mesh-shell/src/shell/generate.rs)).
For an axis-aligned cuboid, every face's unit normal is one of
`±x`, `±y`, `±z`. So the unnormalized sum at each vertex is a vector
with integer components — and the integer at each axis equals the
SIGNED count of incident triangles whose face normal points along
that axis.

For our 10-face triangulation (one diagonal per wall), each wall has
TWO triangles. A vertex on a wall's diagonal is incident to BOTH
triangles of that wall ⇒ that wall's normal contributes ×2.
Off-diagonal vertices get ×1. Symmetric `1/√k` formulas only apply
when triangle-incidence counts are equal across all incident walls.

The full per-vertex breakdown:

| Vert | Position | Incident triangles | Sum of unit normals | `\|sum\|` | Offset vector | Symmetric? |
|---|---|---|---|---|---|---|
| 0 | `(0,0,0)` | 2 bot, 2 front, 2 left | `(-2, -2, -2)` | `2√3` | `(-2/√3, -2/√3, -2/√3)` ≈ `(-1.155, -1.155, -1.155)` | ✓ |
| 1 | `(10,0,0)` | 1 bot, 1 front, 2 right | `(2, -1, -1)` | `√6` | `(4/√6, -2/√6, -2/√6)` ≈ `(1.633, -0.816, -0.816)` | ✗ |
| 2 | `(10,10,0)` | 2 bot, 2 back, 1 right | `(1, 2, -2)` | `3` | `(2/3, 4/3, -4/3)` ≈ `(0.667, 1.333, -1.333)` | ✗ |
| 3 | `(0,10,0)` | 1 bot, 1 back, 1 left | `(-1, 1, -1)` | `√3` | `(-2/√3, 2/√3, -2/√3)` ≈ `(-1.155, 1.155, -1.155)` | ✓ |
| 4 | `(0,0,10)` | 1 front, 1 left | `(-1, -1, 0)` | `√2` | `(-2/√2, -2/√2, 0)` ≈ `(-1.414, -1.414, 0)` | ✓ |
| 5 | `(10,0,10)` | 2 front, 1 right | `(1, -2, 0)` | `√5` | `(2/√5, -4/√5, 0)` ≈ `(0.894, -1.789, 0)` | ✗ |
| 6 | `(10,10,10)` | 1 back, 2 right | `(2, 1, 0)` | `√5` | `(4/√5, 2/√5, 0)` ≈ `(1.789, 0.894, 0)` | ✗ |
| 7 | `(0,10,10)` | 2 back, 2 left | `(-2, 2, 0)` | `2√2` | `(-2/√2, 2/√2, 0)` ≈ `(-1.414, 1.414, 0)` | ✓ |

(Magnitude column verifies: each offset vector's `|v|` is exactly
`wall_thickness = 2.0`, regardless of which triangulation row it's
on. That's the universal invariant.)

**Symmetric vertices** (0, 3, 4, 7) are those where each incident
wall contributes equally:
- Vert 0: 2 of each → simple `(±1, ±1, ±1)/√3` direction.
- Vert 3: 1 of each → also `(±1, ±1, ±1)/√3` (equal weights even
  though the integer is different).
- Verts 4, 7: equal contributions per wall ⇒ `(±1, ±1, 0)/√2`.

For these vertices the simple formula `wall_thickness/√k` (where
`k` is the incident-perpendicular-wall count) gives the correct
per-axis component magnitude.

**Asymmetric vertices** (1, 2, 5, 6) get unequal contributions,
producing offset directions skewed toward the wall(s) with more
incident triangles. Vert 1, for example, is on the diagonal of
the right wall (2 right triangles incident) but off-diagonal on
the bottom and front walls (1 each); the +x component dominates
(`4/√6 ≈ 1.633mm`) while ±y and ±z are smaller (`2/√6 ≈ 0.816mm`
each).

**Why this matters for printability:** the slicer sees the
*perpendicular* wall thickness — the distance from outer vertex
to each incident face plane. For axis-aligned faces this equals
`|offset component along that face's normal|`. So at vert 0
(symmetric, k=3): all three perpendicular distances equal `2/√3
≈ 1.155mm`. At asymmetric vert 2 (k=3 but unequal weights):
distances are `4/3 ≈ 1.333mm` (to bottom and back walls) and
**`2/3 ≈ 0.667mm` to the right wall — the thinnest section
anywhere on the shell, only 33% of the parameterized
`wall_thickness`**. Vert 1's perpendiculars to bottom and front
walls are similarly thin at `2/√6 ≈ 0.816mm`; verts 5, 6's
perpendiculars to right and back respectively are `2/√5 ≈
0.894mm`. The asymmetric vertices ALL have at least one
perpendicular thinner than the symmetric `1/√k` baseline.

For our 2mm wall_thickness on a 10mm cube, even the thinnest
perpendicular section (0.667mm at vert 2) stays above typical
FDM nozzle widths (0.4mm). For thinner walls at production scale
(e.g. 0.6mm at 50mm part size), that `0.667/2 = 33%` thinning
factor at the worst-case vertex would produce a perpendicular
section of `0.2mm` — well below the 0.4mm nozzle width, and the
slicer would silently produce gaps. This is the failure mode
`.high_quality()` (commit 12) addresses by routing the outer
surface through marching cubes — the SDF level set keeps
PERPENDICULAR thickness uniform regardless of input
triangulation, at the cost of breaking 1-to-1 vertex correspondence
and introducing cell-scale chamfer artifacts at sharp creases.

For `.fast()`, accept the offset-direction variation as a known
geometric property of the algorithm. The thinning is deterministic,
closed-form per vertex, and easy to budget for once the
incidence-count formula is known. The example asserts each of the
8 vertices' offset vectors against the closed-form table above
within `1e-9` (see `verify_offsets` in `src/main.rs`).

## The rim closes the open boundary

Input has 4 boundary edges (the missing top face's perimeter forms
a single loop). `generate_rim` walks each boundary edge `(v0, v1)`
in the inner mesh and emits two rim triangles bridging it to the
corresponding outer edge `(v0 + n, v1 + n)`:

```
Triangle 1: [v0, v0 + n, v1 + n]   // inner v0 → outer v0 → outer v1
Triangle 2: [v0, v1 + n, v1]       // inner v0 → outer v1 → inner v1
```

So `rim_face_count == 2 × boundary_size == 8` for our 4-edge
boundary. After the rim is added, the shell has zero boundary edges
and zero non-manifold edges; `validate_shell` reports it as
watertight, manifold, and printable.

## mesh-shell 0.7.x rim winding quirk

Even though the shell is watertight, manifold, and printable,
`shell_stats.validation.has_consistent_winding == false` and
`validation.issues` contains exactly one `ShellIssue::InconsistentWinding`.
The example anchors both as platform truth in `verify_validation`.

**The diagnosis:**

`generate_rim` computes boundary edges from the ORIGINAL input
mesh's faces — `find_boundary_edges` walks `inner_mesh.faces`
BEFORE `generate_shell_normal` reverses the inner-face winding to
build the inner shell surface. The boundary edges are returned
in their original-winding direction `(v0, v1)`, then fed into
the rim quads:

```
Rim tri 2: [v0, v1+n, v1]    // edge sequence v0 → v1+n → v1 → v0
                             // ⇒ contains boundary edge (v1, v0)
                             //   traversed v1 → v0
```

But in the shell, the inner-face winding has been REVERSED. So a
boundary edge that originally ran `v0 → v1` in some inner face
now runs `v1 → v0` in the same face's reversed copy. The rim
quad's tri 2 has `v1 → v0` AND the reversed inner face also has
`v1 → v0` for the same shared boundary edge — both faces
traverse the shared edge in the SAME direction.

`check_winding_consistency` walks each interior edge with exactly
2 incident faces and checks the edge appears in OPPOSITE directions
(the standard manifold-orientation criterion). Same-direction
edges flag as inconsistent ⇒ the result.

**Why the shell is still printable:**

`is_printable()` requires only `is_watertight && is_manifold` —
both hold. Edge-direction inconsistency does NOT affect:

- `signed_volume` — still positive (~365 mm³ for our 2mm-wall box;
  outer-volume minus inner-cavity volume).
- Face-normal direction — outer faces keep input's outward winding,
  inner faces are reversed so cavity normals point inward (outward
  from wall material), rim faces' normals point outward (away from
  wall material) by quad-construction. Every face's normal points
  outward from the shell material.
- Slicer behavior — slicers look at face-normal direction and
  ray-casting, not at edge traversal direction.

What edge-direction inconsistency DOES affect:

- BFS traversal of the shell's face graph (e.g. `mesh_repair::fix_winding_order`)
  may produce unexpected results because the BFS assumes consistent
  edge directions. If you need to run such an algorithm AFTER
  `ShellBuilder`, fix the rim winding first (see remediation
  below).
- `cf-design` and other downstream tools that propagate
  per-vertex attributes via mesh adjacency.

**Remediation (not in this example):**

Two options for downstream code:

1. Run `mesh_repair::fix_winding_order` on the shell. The BFS will
   walk the face graph from face[0] and re-orient each face to
   match the BFS-root's normal direction. For our shell this would
   either flip the rim faces (making them edge-direction-consistent
   with the inner shell) or flip the inner shell (making it
   edge-direction-consistent with the rim). Either choice produces
   `has_consistent_winding == true`.
2. Patch `mesh-shell::generate_rim` to flip the boundary edge
   direction before emitting the rim quads (e.g. `(v0, v1) →
   (v1, v0)`). This is the engine-level fix.

The example DELIBERATELY does not apply either remediation. The
`.fast()` preset's contract is "build a printable shell quickly";
the example anchors that contract truthfully and surfaces the
edge-direction quirk as platform-truth. Tracked as a fixable engine
gap per `feedback_improve_mesh_crate`. Commit 12 (the SDF-based
companion) routes through `generate_rim_for_sdf_shell` (a different
code path) and may exhibit the quirk differently or not at all —
to be verified empirically when commit 12 lands.

## Numerical anchors

**Before (`open_topped_box`):**

- `report.vertex_count == 8`, `report.face_count == 10`
- `report.is_manifold && !report.is_watertight && !report.is_inside_out`
- `report.boundary_edge_count == 4` exactly (single 4-edge boundary loop)
- `report.non_manifold_edge_count == 0`
- `aabb.min == (0, 0, 0)`, `aabb.max == (10, 10, 10)` exactly
- `signed_volume` is printed (`~666.67` for the 5-walled box; the
  partial divergence integral over the 5 outward-wound faces) but
  NOT asserted — it depends on the choice of origin, which is on
  the box corner here, and isn't a robust drift catcher for an
  open mesh.

**Shell — topological structure (`ShellGenerationResult`):**

- `inner_vertex_count == 8`, `outer_vertex_count == 8` (1-to-1 vertex
  correspondence is the topological signature of `WallGenerationMethod::Normal`)
- `boundary_size == 4` (carries the input's boundary count straight through)
- `rim_face_count == 8` (`= 2 × boundary_size`; one quad split into 2 tris per boundary edge)
- `total_face_count == 28` (`= 2 × inner_face_count + rim_face_count = 10 + 10 + 8`)
- `wall_method == Normal`
- `result.offset_applied == false` (no `.offset(...)` call in the chain)
- `shell.vertices.len() == 16`, `shell.faces.len() == 28`

**Shell — vertex correspondence + offset magnitude:**

- For every `i in 0..8`:
  `(shell.vertices[i + 8] - shell.vertices[i]).norm() ≈ 2.0` within `1e-12`.
  The offset magnitude is exactly `wall_thickness`; this is the
  topological invariant of the normal-based method.

**Shell — per-vertex offset directions (closed-form table):**

Each vertex's offset vector matches `wall_thickness × s/|s|` within
`1e-9`, where `s` is the unnormalized sum of incident unit face
normals at that vertex. See [Per-vertex offset
derivation](#per-vertex-offset-derivation) above for the full table
(8 rows). Symmetric vertices (0, 3, 4, 7) hit the simple `1/√k`
formula; asymmetric vertices (1, 2, 5, 6) have triangulation-skewed
direction.

**Shell — validation (`shell_stats.validation`):**

- `is_watertight == true` (rim closes the open top; `boundary_edge_count == 0`)
- `is_manifold == true` (`non_manifold_edge_count == 0`)
- `is_printable() == true` (watertight && manifold)
- `has_consistent_winding == false` — the **mesh-shell 0.7.x rim
  winding quirk** (see [previous section](#mesh-shell-07x-rim-winding-quirk)).
- `issues.len() == 1`, with the issue being `ShellIssue::InconsistentWinding`.

**Shell — properly wound in face-normal sense:**

- `report.is_inside_out == false` — outer faces inherit input
  winding, inner faces are deliberately winding-flipped to point
  outward from the wall material.
- `signed_volume > 0` (the wall material volume; precise value is
  printed but not asserted because the outer surface is a non-trivial
  polytope under normal-based offset). On the author's machine,
  `signed_volume ≈ 364.81 mm³`.

**PLY round-trip:**

- `out/before.ply` reloads with vertex/face counts matching the
  in-memory open-topped box (8v, 10f).
- `out/shell.ply` reloads with vertex/face counts matching the
  in-memory shell (16v, 28f).

These anchors are drift catchers for the `.fast()` preset's
contract: if `mesh-shell` ever changes the inner/outer layout, adds
welding to `generate_rim`, alters the winding-flip convention,
fixes the rim winding quirk, or changes how `.fast()` interacts
with `.validate(true)`, this example fails loudly and the README
sections shift.

## Visuals

Open the artifacts in MeshLab, ParaView, Blender, or `f3d`:

- **`out/before.ply`** — an upward-facing rectangular bowl. Five
  flat 10×10 walls (bottom + four sides), top open. The four top
  edges are visible as a square boundary loop (no triangles span
  the opening). Sharp 90° edges throughout.
- **`out/shell.ply`** — a printable hollowed-out box. Walls have
  ~2mm thickness everywhere along the surface normal direction.
  What you'll see:
  - **Outer surface is NOT a parallel offset of the input.** The
    eight outer-vertex positions are non-coplanar across what was
    originally each flat 10×10 wall; in a wireframe view, the
    diagonals on each outer face reveal the resulting skew. The
    symmetric verts 0 and 3 (bottom corners on every incident
    wall's diagonal) land at perpendicular distance `2.0/√3 ≈
    1.155mm` from each incident face plane. The asymmetric
    bottom corners (verts 1 and 2) land with uneven perpendicular
    distances: vert 1 is `2/√6 ≈ 0.816mm` from the bottom and
    front but `4/√6 ≈ 1.633mm` from the right wall; vert 2 is
    `4/3 ≈ 1.333mm` from the bottom and back but only **`2/3 ≈
    0.667mm` from the right wall** (the thinnest perpendicular
    anywhere on the shell). Because the four outer-front
    vertices have four different perpendicular distances from
    the original `y=0` plane, the outer-front "face" splits
    visibly into two skewed triangles — same on every other
    wall.
  - **Top-rim corners are mixed.** Symmetric top-rim verts (4
    and 7) land at `2.0/√2 ≈ 1.414mm` perpendicular distance
    from each incident sidewall (more than bottom corners
    because only 2 walls meet at each top-rim corner). The
    asymmetric top-rim verts (5 and 6) land with uneven
    perpendiculars — vert 5 is `4/√5 ≈ 1.789mm` from the front
    and `2/√5 ≈ 0.894mm` from the right; vert 6 mirrors with
    `0.894mm` to the back and `1.789mm` to the right.
  - **The rim is a 2mm-wide ring around the top opening**,
    closing the gap between the inner-cavity edge and the
    outer-surface edge. The rim's 8 triangles are visible on
    close inspection (each of the 4 top edges produces 2 rim
    triangles forming a quad).
  - **No inside-out artifacts visible.** Outer-surface normals
    point outward; inner-cavity normals point inward (= outward
    from wall material). The shell renders correctly across every
    viewer pipeline (single-sided, two-sided, with or without
    backface culling). The `mesh-shell` 0.7.x rim winding quirk
    flagged by `has_consistent_winding == false` is INVISIBLE in
    any lighting model — it affects edge-traversal direction, not
    face-normal direction. No `_flipped.ply` companion needed —
    unlike [`mesh-offset-outward`](../mesh-offset-outward/) and
    [`mesh-offset-inward`](../mesh-offset-inward/), which DO need
    one because `mesh-offset` 0.7.x's marching cubes path emits
    inside-out winding (a normal-direction problem, visible in
    backface-culled viewers).
  - **Sharp planar surfaces, not chamfered.** The outer surface
    is a closed-form polygon mesh (no MC discretization), so
    faces are exactly planar between vertices and corners are
    angular. This is the visual contrast vs the SDF-based
    companion (commit 12), which DOES introduce cell-scale
    chamfering at sharp creases — the cost of uniform
    perpendicular thickness.

A dedicated cross-section example (`cross-section-sweep`,
commit 14) will demonstrate the perpendicular-thickness asymmetry
visually with a PLY contour stack.

## Run

```
cargo run -p example-mesh-shell-generation-fast --release
```

Output written to `examples/mesh/shell-generation-fast/out/`.
