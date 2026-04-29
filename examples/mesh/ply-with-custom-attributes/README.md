# ply-with-custom-attributes

Per-vertex custom-attribute PLY round-trip via `save_ply_attributed` /
`load_ply_attributed`. Load-bearing infrastructure proof for the soft-body
PLY field-data export pattern that follows in subsequent example PRs — if
extras don't survive disk, Lagrangian per-vertex scalar export doesn't
either.

## What it does

Builds a unit cube (`mesh_types::unit_cube`, 8 vertices, 12 triangles) and
wraps it as an `AttributedMesh`. Populates a single per-vertex extra
`extras["height"]` whose values are each vertex's z-coordinate cast to
`f32`. Writes the attributed mesh to `out/cube.ply` as binary PLY, then
reloads via `load_ply_attributed` and asserts every numerical anchor
below.

## Numerical anchors

- Vertex count: 8 in == 8 out
- Face count: 12 in == 12 out
- Exactly one `extras` key is recovered: `"height"`
- `extras["height"]` is bit-equal to the original `Vec<f32>` (`[0, 0, 0, 0, 1, 1, 1, 1]`)
- For each vertex `i`: `extras["height"][i] == reloaded.geometry.vertices[i].z as f32`

## Visuals

- The cube renders as a closed unit cube in MeshLab / ParaView / Blender
- Loading the file shows a `height` per-vertex scalar field
- Colormapping `height` produces a clean top-vs-bottom gradient (`z=1` vs `z=0`)

## Run

```
cargo run -p example-mesh-ply-with-custom-attributes --release
```

Output written to `out/cube.ply` inside this crate's directory.
