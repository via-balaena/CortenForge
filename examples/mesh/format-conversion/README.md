# format-conversion

Three-format round-trip across STL, PLY, and OBJ. Demonstrates that
**format choice has consequences**: STL preserves a mesh's *geometry*
(vertex positions, face winding, volume, surface area) but discards its
*topology* (which faces share which vertices), while PLY and OBJ
preserve both.

## What it does

Builds a unit cube (`mesh_types::unit_cube`, 8 vertices, 12 triangles)
and saves it independently to `out/cube.stl`, `out/cube.ply`, and
`out/cube.obj` (fan-from-source, not a chain — chained tests would
conflate per-format losses). Each artifact is reloaded via the matching
`mesh_io::load_*` and compared back to the source on five invariants:
vertex count, face count, signed volume, surface area, and AABB.

The headline divergence is that `mesh_io::load_stl` does **not** dedup
vertices — STL files store three fresh vertices per triangle on disk,
and the loader honors that, so the unit cube round-trips through STL
with 36 vertices instead of 8. PLY and OBJ both store explicit vertex
sharing and recover the original 8.

## Numerical anchors

- `face_count == 12` for all three formats
- `vertex_count == 36` after STL round-trip (12 tris × 3 fresh verts)
- `vertex_count == 8` after PLY round-trip
- `vertex_count == 8` after OBJ round-trip
- For all three: `(signed_volume - 1.0).abs() < 1e-6`
- For all three: `signed_volume > 0.0` (winding sanity — STL recomputes
  normals from vertex order on save and discards them on load, so the
  round-trip survives only if winding is preserved)
- For all three: `(surface_area - 6.0).abs() < 1e-6`
- For all three: AABB min ≈ `(0, 0, 0)` and max ≈ `(1, 1, 1)` within `1e-6`

The `1e-6` tolerance documents the precision contract: STL and PLY
binaries traverse f32 on disk, OBJ traverses `:.6` ASCII. For
0.0 / 1.0 coordinates the round-trip is bit-exact in practice.

## Visuals

- Open `out/cube.stl`, `out/cube.ply`, and `out/cube.obj` in MeshLab /
  ParaView / Blender — all three render as the same closed unit cube
- No degenerate triangles, no duplicate faces, no inverted normals
  (winding sanity check beyond the numerical assertion)
- The STL file size is noticeably larger relative to its information
  content — that's the topology loss made tangible
- If your viewer renders the OBJ all-black, that's the OBJ-format
  default for "no material file referenced" — STL and PLY have no
  material concept and fall back to the viewer's default colour

## Run

```
cargo run -p example-mesh-format-conversion --release
```

Output written to `out/cube.{stl,ply,obj}` inside this crate's directory.
