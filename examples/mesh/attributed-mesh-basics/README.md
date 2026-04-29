# attributed-mesh-basics

The `AttributedMesh` shape itself: how struct-of-arrays per-vertex slots,
the extensible `extras` map, and the length-validation contract relate to
the underlying `IndexedMesh`. **No I/O** — this example demonstrates the
type, not the wire format. Pairs with
[`ply-with-custom-attributes`](../ply-with-custom-attributes/) (which proves
extras survive disk).

## What it does

Builds a unit cube (`mesh_types::unit_cube`, 8 vertices, 12 triangles) and
wraps it as an `AttributedMesh`. Populates three representative slots —
one geometry-derived (`normals` via `compute_normals`), one
domain-styled (`colors` via the (x, y, z) → (R, G, B) corner mapping),
and one user-named (`extras["height"]`, mirroring the anchor in
`ply-with-custom-attributes`). Then deliberately calls `insert_extra`
with a length-3 vector to show how the platform catches
length-mismatched per-vertex attributes.

The other four built-in slots (`zone_ids`, `clearances`, `offsets`, `uvs`)
stay `None` — opt-in attribution is a core feature of the SoA layout, and
this example surfaces that explicitly rather than hiding it.

## The slot diagram

```
AttributedMesh {
    geometry:   IndexedMesh                  // 8 vertices, 12 faces
    normals:    Some(Vec<Vector3<f64>>)      // length 8, area-weighted
    colors:     Some(Vec<VertexColor>)       // length 8, RGB = (x,y,z)*255
    zone_ids:   None                         // -- unused --
    clearances: None                         // -- unused --
    offsets:    None                         // -- unused --
    uvs:        None                         // -- unused --
    extras:     BTreeMap<String, Vec<f32>> {
        "height": [0, 0, 0, 0, 1, 1, 1, 1]   // length 8 (per-vertex z)
    }
}
```

Each populated slot has length `vertex_count()`. `extras` map values share
that constraint and `insert_extra` enforces it at insertion time.

## Numerical anchors

- `vertex_count == 8`, `face_count == 12`
- `mesh.normals` is `Some` with length 8
- `mesh.colors` is `Some` with length 8
- `mesh.zone_ids`, `mesh.clearances`, `mesh.offsets`, `mesh.uvs` are all `None`
- `mesh.extras` has exactly one key, `"height"`, of length 8
- For each vertex `i`: `extras["height"][i]` bit-equals `v.z as f32`
- The bogus length-3 insertion returns `Err` with `name == "bogus"`,
  `expected == 8`, `actual == 3`
- Mesh state is unchanged after the rejected insertion: `extras.len() == 1`,
  no `"bogus"` key

## Output (no artifact)

This example writes nothing to disk — there is no PLY, no `out/`. The
"visuals pass" is the structured printout: one labeled line per slot, the
extras map in full, and the formatted error message from the
length-mismatch demo. Read down the printout and confirm:

- Each populated slot reports `Some(8 entries)`.
- Each unused slot prints `None`.
- The mismatch demo prints the `AttributeMismatchError` `Display` form
  ("``extra attribute `bogus` has 3 values but mesh has 8 vertices``") and
  the structured field values; the mesh's `extras` length remains 1
  (i.e., the rejected insertion left no trace).

## Run

```
cargo run -p example-mesh-attributed-mesh-basics --release
```

No output files are produced.
