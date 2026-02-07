# Future Work 3 — Correctness Remainder + Scaling (Items #6–10)

Part of [Simulation Phase 2 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

---

### 6a. Height Field MJCF Wiring
**Status:** Done | **Effort:** M | **Prerequisites:** None

#### Current State

**sim-core** has full height field collision support:
- `HeightFieldData` (`heightfield.rs:84–98`) — 2D grid with uniform `cell_size`,
  XY-plane with Z-up heights, origin at corner `(0, 0)`. Stores `heights` in
  row-major order: `heights[y * width + x]` where `width` = X samples (ncol),
  `depth` = Y samples (nrow).
- `CollisionShape::HeightField { data: Arc<HeightFieldData> }` with constructor
  `CollisionShape::height_field(data)` (`collision_shape.rs:314`).
- Four contact functions: `heightfield_sphere_contact`, `heightfield_capsule_contact`,
  `heightfield_box_contact`, `heightfield_point_contact` — all in `heightfield.rs`.
  No hfield↔hfield function exists (intentional — physically degenerate).
- Full AABB and bounding radius support (`collision_shape.rs:503–605`).
- **Limitation:** `HeightFieldData` uses a single uniform `cell_size` — square
  cells only. Non-square MuJoCo hfields (where `radius_x/ncol ≠ radius_y/nrow`)
  require resampling (see Step 5).

**MJCF layer** recognizes hfield at parse time but does not wire it:
- `MjcfGeomType::Hfield` parsed from `type="hfield"` (`types.rs:775`).
- `model_builder.rs:1000–1006` falls back to `GeomType::Box` with a warning.
- `parse_asset()` (`parser.rs:690`) returns `Vec<MjcfMesh>` only; skips `<hfield>`
  elements (`parser.rs:703`: "Skip other asset types (texture, material, hfield, etc.)").
- `MjcfGeom` has no `hfield: Option<String>` field (unlike `mesh: Option<String>`
  at `types.rs:860`). `MjcfGeom::default()` (`types.rs:868`) would need `hfield: None`.
- `MjcfModel` has no `hfields` field (`types.rs:2814`).
- `GeomType` enum (`mujoco_pipeline.rs:361–377`) has no `Hfield` variant.

**Collision pipeline** has no hfield dispatch:
- `collide_geoms()` (`mujoco_pipeline.rs:3792`) dispatches on `GeomType` with
  special-case branches for Mesh (`collide_with_mesh`), Plane (`collide_with_plane`),
  and analytical primitives. No height field branch exists.
- `geom_to_collision_shape()` (`mujoco_pipeline.rs:3918`) has no Hfield arm.
- `aabb_from_geom()` (`mujoco_pipeline.rs:3374`) has no Hfield arm.
- `GeomType::bounding_radius()` (`mujoco_pipeline.rs:396`) has no Hfield arm.
- `geom_size_to_vec3()` (`model_builder.rs:2969`) falls through to catch-all
  `_ => Vector3::new(0.1, 0.1, 0.1)` for Hfield — incorrect.

#### Objective

Wire `<hfield>` assets and `<geom type="hfield">` through the full pipeline:
parse → model build → collision detection. After this, height field terrain in
MJCF files produces working collision.

#### MuJoCo Reference

Source: `mjmodel.h` (hfield fields) and `engine_collision_convex.c`
(`mjc_ConvexHField`, `mjc_HFieldElem`).

**`<hfield>` asset element attributes:**

| Attribute | Type | Description |
|-----------|------|-------------|
| `name` | string | Asset name, referenced by `<geom hfield="name">` |
| `nrow` | int | Grid rows (Y-axis samples) |
| `ncol` | int | Grid columns (X-axis samples) |
| `size` | float[4] | `[x, y, z_top, z_bottom]` (see below) |
| `elevation` | float[nrow×ncol] | Normalized elevation data, row-major |
| `file` | string | External PNG file (out of scope) |

**`size` semantics** (from `mjModel.hfield_size`):
- `size[0]` (`x`): half-extent along X → field spans `[−x, +x]`
- `size[1]` (`y`): half-extent along Y → field spans `[−y, +y]`
- `size[2]` (`z_top`): vertical scale factor for elevation data
- `size[3]` (`z_bottom`): base depth below hfield origin (collision floor at `−z_bottom`)

**Vertex coordinate computation** (from `mjc_HFieldElem`):
```
dx = 2 × size[0] / (ncol − 1)
dy = 2 × size[1] / (nrow − 1)

vertex_x = dx × col − size[0]           // col ∈ [0, ncol−1] → x ∈ [−size[0], +size[0]]
vertex_y = dy × row − size[1]           // row ∈ [0, nrow−1] → y ∈ [−size[1], +size[1]]
vertex_z = elevation[row × ncol + col] × size[2]
```

The hfield is **centered at its geom origin**. Row 0 maps to `−size[1]` (minimum Y).
The base/floor prism is at `z = −size[3]`.

**Row-major ordering match:** MuJoCo's `elevation[row * ncol + col]` maps directly
to `HeightFieldData`'s `heights[y * width + x]` — both have row 0 at minimum Y,
X varying fastest. No Y-flip is needed.

**Example MJCF:**
```xml
<asset>
  <hfield name="terrain" size="5 5 1 0.1" nrow="50" ncol="50"
          elevation="0 0 0.5 0.5 1 ..."/>
</asset>
<worldbody>
  <geom type="hfield" hfield="terrain"/>
</worldbody>
```

**Scope exclusions:**
- **File-based hfield loading** (`<hfield file="terrain.png"/>`): out of scope.
  Only inline `elevation` + `nrow`/`ncol` required. File loading can be added later
  without API changes.
- **Hfield↔hfield collision:** not supported (no `heightfield_heightfield_contact`
  exists in sim-core; physically degenerate scenario).
- **Default class inheritance for hfield attribute:** MuJoCo's `<default>` system
  does not propagate asset references (`mesh`, `hfield`), so no defaults changes
  needed. Confirmed: `apply_to_geom()` (`defaults.rs:182`) does not touch `mesh`.
- **Mass/inertia:** Hfield geoms use the catch-all in `compute_geom_inertia()`
  (returns `Vector3::new(0.001, 0.001, 0.001)`), same as Mesh geoms. This is
  expected — hfield terrain is typically attached to `<worldbody>` (static body
  with infinite effective mass). No special-case inertia computation is needed.

#### Specification

**Step 1 — MJCF types** (`types.rs`)

Add `MjcfHfield` struct (following the `MjcfMesh` pattern at `types.rs:558–582`):

```rust
pub struct MjcfHfield {
    /// Asset name (referenced by <geom hfield="...">).
    pub name: String,
    /// Size: [x, y, z_top, z_bottom].
    /// x/y = half-extents; z_top = elevation scale; z_bottom = base depth.
    pub size: [f64; 4],
    /// Number of rows (Y samples).
    pub nrow: usize,
    /// Number of columns (X samples).
    pub ncol: usize,
    /// Normalized elevation data, row-major (row 0 = min Y, X varies fastest).
    /// Length: nrow × ncol.
    pub elevation: Vec<f64>,
}
```

Add `hfield: Option<String>` field to `MjcfGeom` (`types.rs:860`, alongside `mesh`).
Update `MjcfGeom::default()` (`types.rs:868`): add `hfield: None`.

Add `hfields: Vec<MjcfHfield>` to `MjcfModel` (`types.rs:2814`, alongside `meshes`).
Update `MjcfModel::default()`: add `hfields: Vec::new()`.

**Step 2 — Parser** (`parser.rs`)

Change `parse_asset()` signature from `Result<Vec<MjcfMesh>>` to return both
meshes and hfields. Two options (pick at implementation time):

*Option A — tuple return:*
```rust
fn parse_asset<R: BufRead>(reader: &mut Reader<R>) -> Result<(Vec<MjcfMesh>, Vec<MjcfHfield>)>
```
Update call site (`parser.rs:78`): `let (meshes, hfields) = parse_asset(reader)?;`

*Option B — populate MjcfModel fields directly* (if the parser already has access
to a model builder struct). Follow whichever pattern is cleanest.

In `parse_asset()` body (`parser.rs:698`), handle hfield in **both** event
branches (following the mesh pattern which handles `Event::Start` and
`Event::Empty` separately):

In the `Event::Start` arm:
```rust
b"hfield" => {
    let hfield = parse_hfield_attrs(e)?;
    hfields.push(hfield);
    // <hfield> has no child elements — skip to closing tag
    skip_element(reader, &elem_name)?;
}
```

In the `Event::Empty` arm (self-closing `<hfield ... />`):
```rust
if e.name().as_ref() == b"hfield" {
    let hfield = parse_hfield_attrs(e)?;
    hfields.push(hfield);
}
```

MuJoCo `<hfield>` is typically self-closing (`Event::Empty`), but both forms
must be handled for robustness. Unlike meshes, `<hfield>` has no child elements
so both branches call the same `parse_hfield_attrs()` function.

Implement `parse_hfield_attrs()`:
- Required attributes: `name`, `nrow`, `ncol`, `size` (4 floats), `elevation`
  (space-separated floats).
- Validation:
  - `elevation.len() == nrow * ncol` (error otherwise)
  - `nrow >= 2 && ncol >= 2` (minimum grid)
  - `size[0] > 0 && size[1] > 0 && size[2] >= 0 && size[3] >= 0` (z_top may be 0
    for flat; z_bottom may be 0 for no base depth)
  - Elevation values need not be clamped to `[0, 1]` — MuJoCo does not enforce this.

In `parse_geom_attrs()` (`parser.rs:1039`), add:
```rust
geom.hfield = get_attribute_opt(e, "hfield");
```

**Step 3 — GeomType enum** (`mujoco_pipeline.rs`)

Add `Hfield` variant to `GeomType` (`mujoco_pipeline.rs:361`):

```rust
pub enum GeomType {
    Plane, Sphere, Capsule, Cylinder, Box, Ellipsoid, Mesh,
    Hfield,  // NEW
}
```

Update `GeomType::bounding_radius()` (`mujoco_pipeline.rs:396`): add
`Self::Hfield` arm. Like `Mesh`, the bounding radius cannot be computed from
`size` alone — it depends on the actual elevation data. Return a conservative
estimate from `geom_size`: `Vector2::new(size.x, size.y).norm()` (horizontal
half-diagonal). The true bounding radius is overwritten by the model builder's
post-build pass (`model_builder.rs:2392`) using the `HeightFieldData` AABB.

**Step 4 — Model storage** (`mujoco_pipeline.rs`)

Add to `Model` (following the mesh pattern at `mujoco_pipeline.rs:908–915`):

```rust
// ==================== Height Fields (indexed by hfield_id) ====================
pub nhfield: usize,
pub hfield_name: Vec<String>,
pub hfield_data: Vec<Arc<HeightFieldData>>,
pub hfield_size: Vec<[f64; 4]>,  // original MuJoCo size for centering offset

// In geom arrays:
pub geom_hfield: Vec<Option<usize>>,  // hfield_id per geom (like geom_mesh)
```

The `hfield_size` is stored because the centering offset (`-size[0]`, `-size[1]`)
is needed at collision time to convert from `HeightFieldData`'s corner-origin
coordinate system to MuJoCo's center-origin system.

**Step 5 — Model builder** (`model_builder.rs`)

Add `ModelBuilder` fields (following the mesh pattern at `model_builder.rs:265–271`):

```rust
hfield_name_to_id: HashMap<String, usize>,
hfield_name: Vec<String>,
hfield_data: Vec<Arc<HeightFieldData>>,
hfield_size: Vec<[f64; 4]>,
geom_hfield: Vec<Option<usize>>,
```

Initialize all to empty in `ModelBuilder::new()` (`model_builder.rs:440`).

Add `process_hfield()` method (following `process_mesh()` at `model_builder.rs:617`):

```rust
fn process_hfield(&mut self, hfield: &MjcfHfield) -> Result<(), ModelConversionError> {
    if self.hfield_name_to_id.contains_key(&hfield.name) {
        return Err(ModelConversionError { message: format!("duplicate hfield '{}'", hfield.name) });
    }
    let data = convert_mjcf_hfield(hfield)?;
    let id = self.hfield_data.len();
    self.hfield_name_to_id.insert(hfield.name.clone(), id);
    self.hfield_name.push(hfield.name.clone());
    self.hfield_data.push(Arc::new(data));
    self.hfield_size.push(hfield.size);
    Ok(())
}
```

**`convert_mjcf_hfield()` — elevation → HeightFieldData conversion:**

```
dx = 2 × size[0] / (ncol − 1)     // X cell spacing
dy = 2 × size[1] / (nrow − 1)     // Y cell spacing
heights[i] = elevation[i] × size[2]   // scale by z_top
```

`HeightFieldData` requires uniform `cell_size` (square cells). Strategy:
- If `|dx − dy| / dx.max(dy) < 0.01` (within 1%): use `cell_size = (dx + dy) / 2`.
- If `dx ≠ dy` beyond tolerance: **resample** to the finer resolution. Use
  `cell_size = dx.min(dy)`. Compute new grid dimensions:
  `new_ncol = ((2 × size[0]) / cell_size).round() + 1`,
  `new_nrow = ((2 × size[1]) / cell_size).round() + 1`.
  Bilinear-interpolate from the original grid to the new grid.
  Log a `warn!` noting the resampling.

Construct `HeightFieldData::new(heights, ncol, nrow, cell_size)` where
`width = ncol` (X samples) and `depth = nrow` (Y samples). Row-major ordering
matches — no flip needed.

In `model_from_mjcf()` (`model_builder.rs:107`), call `process_hfield()` for each
hfield asset before processing bodies (like `process_mesh` at line 110–112).

In `process_geom()` (`model_builder.rs:984`):
- `MjcfGeomType::Hfield` → `GeomType::Hfield` (remove Box fallback).
- Add hfield asset linking (parallel to mesh linking at `model_builder.rs:1017–1042`):

```rust
let geom_hfield_ref = if geom_type == GeomType::Hfield {
    match &geom.hfield {
        Some(name) => {
            let hfield_id = self.hfield_name_to_id.get(name)
                .ok_or_else(|| ModelConversionError {
                    message: format!("geom '{}': references undefined hfield '{}'",
                        geom.name.as_deref().unwrap_or("<unnamed>"), name),
                })?;
            Some(*hfield_id)
        }
        None => return Err(ModelConversionError {
            message: format!("geom '{}': type is hfield but no hfield attribute specified",
                geom.name.as_deref().unwrap_or("<unnamed>")),
        }),
    }
} else {
    None
};
```

Push `geom_hfield_ref` alongside `geom_mesh_ref` (at `model_builder.rs:1122`).

**Geom size override for Hfield:** `geom_size_to_vec3()` (`model_builder.rs:2970`)
takes only `&geom.size` (the MJCF geom element's `size` attribute) and `geom_type`
— it has **no access** to hfield asset data. For hfield geoms, the geom's `size`
attribute is typically absent (defaults to `vec![0.1]`), so `geom_size_to_vec3`
would return `(0.1, 0.1, 0.1)` via the catch-all — useless.

Instead, **override `geom_size` after hfield resolution** in `process_geom()`.
After resolving `geom_hfield_ref` (which looks up the hfield asset by name),
replace the size computed by `geom_size_to_vec3`:

```rust
// Override geom_size for hfield geoms with asset dimensions
let size = if let Some(hfield_id) = geom_hfield_ref {
    let hf_size = &self.hfield_size[hfield_id];
    Vector3::new(hf_size[0], hf_size[1], hf_size[2])
} else {
    size // keep original from geom_size_to_vec3
};
```

This stores `[x_half_extent, y_half_extent, z_top]` in `geom_size`, enabling
`aabb_from_geom()` and `bounding_radius()` to produce reasonable bounds from
`geom_size` alone (no Model access needed).

No change to `geom_size_to_vec3()` is needed — the catch-all `_ => (0.1, ...)` is
dead code for Hfield since the override above always runs first.

**Bounding radius** (post-build pass at `model_builder.rs:2392`): extend the
existing `if let Some(mesh_id)` chain with an hfield branch:

```rust
model.geom_rbound[geom_id] = if let Some(mesh_id) = model.geom_mesh[geom_id] {
    // Mesh geom: half-diagonal of AABB
    let (aabb_min, aabb_max) = model.mesh_data[mesh_id].aabb();
    let half_diagonal = (aabb_max - aabb_min) / 2.0;
    half_diagonal.norm()
} else if let Some(hfield_id) = model.geom_hfield[geom_id] {
    // Hfield geom: half-diagonal of AABB (same pattern as mesh).
    // HeightFieldData::aabb() returns corner-origin bounds; offset by
    // (-size[0], -size[1], 0) to center, then compute half-diagonal.
    let (aabb_min, aabb_max) = model.hfield_data[hfield_id].aabb();
    let half_diagonal = (aabb_max.coords - aabb_min.coords) / 2.0;
    half_diagonal.norm()
} else {
    // Primitive geom: use GeomType::bounding_radius()
    model.geom_type[geom_id].bounding_radius(model.geom_size[geom_id])
};
```

Note: the half-diagonal is origin-independent (it's `(max - min) / 2`), so the
centering offset does not affect it.

Wire `geom_hfield` and hfield arrays into `build()` (`model_builder.rs:2258`).

**Step 6 — Collision pipeline** (`mujoco_pipeline.rs`)

In `collide_geoms()` (`mujoco_pipeline.rs:3792`), add a height field branch
before the GJK/EPA slow path (after Mesh and Plane checks):

```rust
// Special case: height field collision
if type1 == GeomType::Hfield || type2 == GeomType::Hfield {
    return collide_with_hfield(model, geom1, geom2, pos1, mat1, pos2, mat2);
}
```

Implement `collide_with_hfield()` (following `collide_with_mesh()` at
`mujoco_pipeline.rs:4072`):

```rust
fn collide_with_hfield(
    model: &Model, geom1: usize, geom2: usize,
    pos1: Vector3<f64>, mat1: Matrix3<f64>,
    pos2: Vector3<f64>, mat2: Matrix3<f64>,
) -> Option<Contact> {
    // Identify which geom is the hfield, which is the other
    let (hf_geom, other_geom, hf_pos, hf_mat, other_pos, other_mat) =
        if model.geom_type[geom1] == GeomType::Hfield {
            (geom1, geom2, pos1, mat1, pos2, mat2)
        } else {
            (geom2, geom1, pos2, mat2, pos1, mat1)
        };

    let hfield_id = model.geom_hfield[hf_geom]?;
    let hfield = &model.hfield_data[hfield_id];
    let hf_size = &model.hfield_size[hfield_id];

    // Build hfield pose — apply centering offset so HeightFieldData's
    // corner-origin (0,0) maps to MuJoCo's center-origin (-size[0], -size[1])
    let quat = UnitQuaternion::from_matrix(&hf_mat);
    let offset = hf_mat * Vector3::new(-hf_size[0], -hf_size[1], 0.0);
    let hf_pose = Pose::from_position_rotation(
        Point3::from(hf_pos + offset), quat,
    );

    // Build other geom's parameters
    let other_quat = UnitQuaternion::from_matrix(&other_mat);
    let other_pose = Pose::from_position_rotation(Point3::from(other_pos), other_quat);
    let other_size = model.geom_size[other_geom];

    // Dispatch on the other geom's type
    let hf_contact = match model.geom_type[other_geom] {
        GeomType::Sphere => {
            heightfield_sphere_contact(hfield, &hf_pose, other_pose.position, other_size.x)
        }
        GeomType::Capsule => {
            let axis = other_pose.rotation * Vector3::z();
            let start = other_pose.position - axis * other_size.y;
            let end = other_pose.position + axis * other_size.y;
            heightfield_capsule_contact(hfield, &hf_pose, start, end, other_size.x)
        }
        GeomType::Box => {
            // geom_size for Box geoms stores half-extents, matching the
            // `half_extents: &Vector3<f64>` parameter expected here.
            heightfield_box_contact(hfield, &hf_pose, &other_pose, &other_size)
        }
        GeomType::Cylinder => {
            // Approximate cylinder as capsule (conservative, same as collide_with_mesh)
            let axis = other_pose.rotation * Vector3::z();
            let start = other_pose.position - axis * other_size.y;
            let end = other_pose.position + axis * other_size.y;
            heightfield_capsule_contact(hfield, &hf_pose, start, end, other_size.x)
        }
        GeomType::Ellipsoid => {
            // Approximate as sphere with max radius (conservative)
            let max_r = other_size.x.max(other_size.y).max(other_size.z);
            heightfield_sphere_contact(hfield, &hf_pose, other_pose.position, max_r)
        }
        // Hfield↔Hfield, Hfield↔Plane, Hfield↔Mesh: not supported
        _ => return None,
    };

    // Convert HeightFieldContact → pipeline Contact.
    //
    // Normal direction: HeightFieldContact.normal always points "up from
    // terrain" (heightfield.rs:438). make_contact_from_geoms stores the
    // normal verbatim. Following the collide_with_mesh pattern (lines
    // 4162–4167), when the surface geometry (hfield) is geom2 and the
    // primitive is geom1, we must negate the normal so it points from
    // geom1 toward geom2.
    let swapped = hf_geom != geom1; // true when hfield was originally geom2
    hf_contact.map(|c| {
        let normal = if swapped { -c.normal } else { c.normal };
        make_contact_from_geoms(
            model,
            c.point.coords,
            normal,
            c.penetration,
            geom1, geom2,
        )
    })
}
```

Update `geom_to_collision_shape()` (`mujoco_pipeline.rs:3918`):
add `GeomType::Hfield => None` (handled by `collide_with_hfield`, not GJK/EPA).

Update `aabb_from_geom()` (`mujoco_pipeline.rs:3374`):
add `GeomType::Hfield` arm. The function signature is
`fn aabb_from_geom(geom_type: GeomType, size: Vector3<f64>, pos: Vector3<f64>, mat: Matrix3<f64>) -> Aabb`
— it has **no `Model` reference**, only `size`/`pos`/`mat`. This matches the
Mesh pattern, which also uses a conservative fallback (`MESH_DEFAULT_EXTENT`)
without accessing mesh data.

For Hfield, `geom_size` stores `[hf_size[0], hf_size[1], hf_size[2]]` (x/y
half-extents and z_top scale). Compute a conservative AABB using the rotated-box
approach:

```rust
GeomType::Hfield => {
    // Conservative: treat as a rotated box with half-extents
    // [size.x, size.y, size.z] where size.z = z_top (max possible height).
    // z_bottom is not available here (stored in hfield_size[3], not geom_size),
    // so we use size.z as a symmetric bound — conservative but correct for
    // broad-phase. The exact AABB is not critical: false positives are
    // filtered by narrow-phase, and terrain typically sits on the ground
    // plane where z_bottom ≈ 0.
    let half = size; // [x_half_extent, y_half_extent, z_top]
    let mut min = pos;
    let mut max = pos;
    for i in 0..3 {
        let axis = mat.column(i).into_owned();
        let extent = half[i] * axis.abs();
        min -= extent;
        max += extent;
    }
    Aabb::new(Point3::from(min), Point3::from(max))
}
```

This is the same rotated-box formula used for `GeomType::Box` (line 3387–3398).
It is conservative because it treats z_top as both upper and lower extent
(actual range is `[−z_bottom, max_elevation]`, but z_bottom is typically small
and z_top ≥ max_elevation by definition). False positives are harmless for
broad-phase.

#### Acceptance Criteria

1. `<hfield name="t" size="5 5 1 0.1" nrow="3" ncol="3" elevation="0 0 0 0 1 0 0 0 0"/>`
   followed by `<geom type="hfield" hfield="t"/>` produces a `Model` with
   `nhfield == 1` and `geom_type[i] == GeomType::Hfield`.
2. Elevation mapping matches MuJoCo: `z = elevation × z_top`. An elevation value
   of `0.5` with `size="1 1 10 0"` produces `z = 0.5 × 10 = 5.0`.
3. Vertex positions match MuJoCo: for `size="5 5 1 0" ncol="11"`, column 0 maps
   to `x = −5`, column 10 maps to `x = +5`, with `dx = 1.0`.
4. Hfield geom is centered at its `pos` attribute. A sphere (radius=1) at
   `pos=(0,0,0.5)` over a flat hfield (`elevation` all 1.0) at `pos=(0,0,0)`
   with `z_top=1` produces contact at `z ≈ 1.0` (top of terrain).
5. `cell_size` is derived correctly: `2 × radius / (n − 1)`. For
   `size="5 5 1 0" nrow="11" ncol="11"`, `cell_size = 1.0`.
6. Sphere, capsule, box, cylinder, and ellipsoid primitives generate contacts with
   hfield geoms in the pipeline (tested via `Data::step()`). Cylinder uses capsule
   approximation; ellipsoid uses sphere approximation.
7. Referencing an undefined hfield name produces `ModelConversionError` with a
   clear message naming the geom and the missing asset.
8. `<geom type="hfield"/>` without an `hfield` attribute produces
   `ModelConversionError`.
9. Non-square hfields (`size[0]/ncol ≠ size[1]/nrow`) produce a valid
   `HeightFieldData` via resampling, with a `warn!` log.
10. Height field bounding radius and AABB are correct (verified: broad-phase does
    not cull valid hfield↔primitive pairs).
11. Unsupported pairs (hfield↔hfield, hfield↔plane, hfield↔mesh) silently return
    no contact — no panic, no error.
12. Duplicate hfield asset names produce `ModelConversionError`.
13. Contact normal direction is correct regardless of geom ordering: whether the
    hfield is geom1 or geom2, the contact normal points from geom1 toward geom2
    (consistent with `collide_with_mesh` convention).

#### Files

- `sim/L0/mjcf/src/types.rs` — add `MjcfHfield` struct; add `hfield: Option<String>`
  on `MjcfGeom` + update `Default`; add `hfields: Vec<MjcfHfield>` on `MjcfModel`
  + update `Default`
- `sim/L0/mjcf/src/parser.rs` — `parse_hfield_attrs()`, `b"hfield"` arms in
  both `Event::Start` and `Event::Empty` branches of `parse_asset()`, change
  `parse_asset()` return type, `hfield` attr in `parse_geom_attrs()`
- `sim/L0/mjcf/src/model_builder.rs` — `process_hfield()`, `convert_mjcf_hfield()`,
  hfield geom dispatch + asset linking + geom_size override in `process_geom()`,
  bounding radius post-build, `build()` wiring
- `sim/L0/core/src/mujoco_pipeline.rs` — `GeomType::Hfield` + `bounding_radius()`
  arm, `hfield_data`/`hfield_size`/`hfield_name`/`nhfield`/`geom_hfield` on `Model`,
  `collide_with_hfield()`, `geom_to_collision_shape()` arm, `aabb_from_geom()` arm
- `sim/L0/mjcf/src/validation.rs` — validate hfield references exist in asset list
  (optional — mesh references are also not validated today; implement if time permits)

---

### 6b. SDF MJCF Wiring (CortenForge Extension)
**Status:** Done | **Effort:** S | **Prerequisites:** 6a

#### Current State

**sim-core** has full SDF collision support:
- `SdfCollisionData` (`sdf.rs:100–118`) — 3D grid with uniform `cell_size`,
  ZYX storage order, arbitrary origin. Has `aabb()` method (`sdf.rs:514`)
  returning `(Point3<f64>, Point3<f64>)`.
- `CollisionShape::Sdf { data: Arc<SdfCollisionData> }` with constructor
  `CollisionShape::sdf(data)` (`collision_shape.rs:320`).
- `SdfContact` struct (`sdf.rs:557–564`): `point: Point3<f64>`,
  `normal: Vector3<f64>`, `penetration: f64`.
- 10 shape-pair contact functions + 1 utility (`sdf_point_contact`), all in
  `sdf.rs`. ~50 unit tests.

**SDF contact function signatures** (all return `Option<SdfContact>`):

| Function | Parameters (after `sdf, sdf_pose`) |
|----------|-----------------------------------|
| `sdf_sphere_contact` | `sphere_center: Point3, sphere_radius: f64` |
| `sdf_capsule_contact` | `capsule_start: Point3, capsule_end: Point3, capsule_radius: f64` |
| `sdf_box_contact` | `box_pose: &Pose, half_extents: &Vector3` |
| `sdf_cylinder_contact` | `cylinder_pose: &Pose, half_height: f64, radius: f64` |
| `sdf_ellipsoid_contact` | `ellipsoid_pose: &Pose, radii: &Vector3` |
| `sdf_convex_mesh_contact` | `mesh_pose: &Pose, vertices: &[Point3]` |
| `sdf_triangle_mesh_contact` | `mesh: &TriangleMeshData, mesh_pose: &Pose` |
| `sdf_plane_contact` | `plane_normal: &Vector3, plane_offset: f64` |
| `sdf_heightfield_contact` | `heightfield: &HeightFieldData, heightfield_pose: &Pose` |
| `sdf_sdf_contact` | `sdf_b: &SdfCollisionData, pose_b: &Pose` |

**Normal conventions per function:**
- **Sphere, capsule, box, cylinder, ellipsoid, convex mesh, heightfield:**
  Normal points **outward from the SDF surface** (SDF gradient direction).
- **Plane:** Normal is the **plane normal** (pointing from plane toward SDF),
  not the SDF gradient (`sdf.rs:1176`).
- **SDF↔SDF:** Normal is the **outward gradient of whichever SDF the contact
  point is closer to the surface of** (`sdf.rs:1536–1542`). Not guaranteed
  to be "from sdf_a toward sdf_b".

**MJCF layer** recognizes `type="sdf"` but does not wire it:
- `MjcfGeomType::Sdf` is parsed (`types.rs:795` — enum at `types.rs:777`).
- `model_builder.rs:1044–1049` falls back to `GeomType::Box` with a warning.
- `GeomType` enum (`mujoco_pipeline.rs:365–383`) has no `Sdf` variant.

**Collision pipeline** has no SDF dispatch:
- `collide_geoms()` (`mujoco_pipeline.rs:3837`) dispatch order:
  Mesh → Hfield → Plane → analytical pairs → GJK/EPA.
- `geom_to_collision_shape()` (`mujoco_pipeline.rs:3968`): no Sdf arm (exhaustive).
- `aabb_from_geom()` (`mujoco_pipeline.rs:3404`): no Sdf arm (exhaustive).
- `bounding_radius()` (`mujoco_pipeline.rs:402`): no Sdf arm (exhaustive).
- `collide_with_mesh()` inner matches: explicit arms for each `GeomType`,
  including `Hfield => unreachable!()`. No Sdf arm — **compile error** if Sdf
  variant is added.
- `collide_with_hfield()` (`mujoco_pipeline.rs:4117`): wildcard `_ => return None`
  catches unknown types. Sdf would silently produce no contact.
- `collide_with_plane()` (`mujoco_pipeline.rs:4367`): explicit arms, no Sdf arm
  — **compile error** if Sdf variant is added.

**MuJoCo reference:** MuJoCo does **not** have a built-in `type="sdf"` geom.
MuJoCo SDF support is through its plugin system (first-party plugins define
shapes via distance/gradient callbacks). Our `MjcfGeomType::Sdf` is a
**CortenForge extension** — it has no standard MJCF counterpart. This is
acceptable; it extends the format for our use cases.

#### Objective

Wire `<geom type="sdf">` through the collision pipeline so that programmatically
constructed `SdfCollisionData` can participate in simulation. Unlike hfield (which
has an MJCF asset format), SDF geoms will be wired for programmatic construction
only — there is no `<sdf>` asset element to parse.

#### Specification

**Step 1 — GeomType enum** (`mujoco_pipeline.rs`)

Add `Sdf` variant to `GeomType` (`mujoco_pipeline.rs:365`):

```rust
pub enum GeomType {
    Plane, Sphere, Capsule, Cylinder, Box, Ellipsoid, Mesh,
    Hfield, Sdf,  // NEW
}
```

Add `bounding_radius()` arm (`mujoco_pipeline.rs:402`):

```rust
Self::Sdf => {
    // Conservative: treat as axis-aligned box with half-extents from geom_size.
    // For programmatic SDF geoms, geom_size stores SDF AABB half-extents
    // (set by post-build pass). True bounding radius is overwritten by
    // the post-build pass using SdfCollisionData::aabb().
    size.norm()
}
```

**Step 2 — Model storage** (`mujoco_pipeline.rs`)

Add to `Model` (after hfield fields at `mujoco_pipeline.rs:939`, before the
Sites section):

```rust
// ==================== SDFs (indexed by sdf_id) ====================
/// Number of SDF assets.
pub nsdf: usize,
/// SDF collision data.
/// `Arc` for cheap cloning (multiple geoms can reference the same SDF asset).
pub sdf_data: Vec<Arc<SdfCollisionData>>,

// In geom arrays (alongside geom_mesh, geom_hfield):
/// SDF index for each geom (`None` if not an SDF geom).
/// Length: ngeom. Only geoms with `geom_type == GeomType::Sdf` have `Some(sdf_id)`.
pub geom_sdf: Vec<Option<usize>>,
```

Add to `Model::empty()` (after hfield initialization at `mujoco_pipeline.rs:1918`):

```rust
// SDFs (empty)
nsdf: 0,
sdf_data: vec![],
geom_sdf: vec![],
```

**Step 3 — Model builder** (`model_builder.rs`)

Change the `MjcfGeomType::Sdf` arm in `process_geom()` (currently at
`model_builder.rs:1044–1049`) from a Box fallback to `GeomType::Sdf`:

```rust
MjcfGeomType::Sdf => GeomType::Sdf,
```

Add `geom_sdf` field to `ModelBuilder` (alongside `geom_mesh` and `geom_hfield`):

```rust
geom_sdf: Vec<Option<usize>>,
```

Initialize to `Vec::new()` in `ModelBuilder::new()`.

In `process_geom()`, push `geom_sdf` at the end (alongside `geom_mesh` and
`geom_hfield`):

```rust
self.geom_sdf.push(None);  // SDF geoms are programmatic — never set via MJCF
```

SDF geoms in MJCF have no asset element to parse — `<geom type="sdf"/>` is
legal at parse time (it is a recognized `MjcfGeomType`) but produces a geom
with `GeomType::Sdf` and `geom_sdf == None`. At collision time,
`collide_with_sdf()` returns `None` for geoms where `geom_sdf` is `None`
(no data → no collision). This matches the pattern where a mesh geom with
`geom_mesh == None` would also produce no collision.

**No error is thrown for SDF geoms in MJCF** — the geom is silently non-colliding.
This is preferable to erroring because:
- Users may load an MJCF that declares SDF geom placeholders and populate
  `sdf_data` + `geom_sdf` programmatically after loading.
- Erroring would prevent round-tripping MJCF files that contain SDF geoms.

**geom_size for SDF geoms:** `geom_size_to_vec3()` falls through to the catch-all
`_ => Vector3::new(0.1, 0.1, 0.1)` for Sdf. This is acceptable — the post-build
bounding radius pass overwrites it with the true AABB half-extents. No override
in `process_geom()` is needed (unlike Hfield, which has asset data available at
build time via `hfield_size`).

Wire `geom_sdf`, `nsdf`, `sdf_data` into `build()` (alongside mesh/hfield
fields):

```rust
geom_sdf: self.geom_sdf,
nsdf: 0,       // programmatic — always 0 from MJCF
sdf_data: vec![],  // programmatic — always empty from MJCF
```

**Post-build bounding radius pass** (`model_builder.rs:2475`): extend the
if/else chain with an SDF branch after hfield:

```rust
model.geom_rbound[geom_id] = if let Some(mesh_id) = model.geom_mesh[geom_id] {
    // Mesh geom: half-diagonal of AABB
    let (aabb_min, aabb_max) = model.mesh_data[mesh_id].aabb();
    let half_diagonal = (aabb_max - aabb_min) / 2.0;
    half_diagonal.norm()
} else if let Some(hfield_id) = model.geom_hfield[geom_id] {
    // Hfield geom: half-diagonal of AABB
    let (aabb_min, aabb_max) = model.hfield_data[hfield_id].aabb();
    let half_diagonal = (aabb_max.coords - aabb_min.coords) / 2.0;
    half_diagonal.norm()
} else if let Some(sdf_id) = model.geom_sdf[geom_id] {
    // SDF geom: half-diagonal of AABB (same pattern as mesh/hfield).
    // SdfCollisionData::aabb() returns (Point3, Point3) — origin-based bounds.
    let (aabb_min, aabb_max) = model.sdf_data[sdf_id].aabb();
    let half_diagonal = (aabb_max.coords - aabb_min.coords) / 2.0;
    half_diagonal.norm()
} else {
    // Primitive geom: use GeomType::bounding_radius()
    model.geom_type[geom_id].bounding_radius(model.geom_size[geom_id])
};
```

Note: for MJCF-loaded models, the SDF branch never fires (`geom_sdf` is always
`None` from MJCF). It activates only for programmatically constructed models
where `geom_sdf` and `sdf_data` are populated directly.

**Step 4 — Collision pipeline** (`mujoco_pipeline.rs`)

**Dispatch order.** Add SDF branch **before Mesh** in `collide_geoms()`
(`mujoco_pipeline.rs:3855`). This is critical because:
- SDF↔Mesh: if Mesh fires first, `collide_with_mesh()` has no SDF arm →
  would require adding dispatch logic inside `collide_with_mesh`. Putting SDF
  first keeps all SDF dispatch in one function.
- SDF↔Hfield: if Hfield fires first, `collide_with_hfield()` wildcard returns
  `None` — silently drops valid contacts. `sdf_heightfield_contact()` exists
  and should be used.
- SDF↔Plane: `collide_with_plane()` has no SDF arm → same issue.

New dispatch order:

```rust
// Special case: SDF collision (before mesh/hfield/plane — SDF has its own
// contact functions for all shapes including Mesh, Hfield, and Plane)
if type1 == GeomType::Sdf || type2 == GeomType::Sdf {
    return collide_with_sdf(model, geom1, geom2, pos1, mat1, pos2, mat2);
}

// Special case: mesh collision (existing)
if type1 == GeomType::Mesh || type2 == GeomType::Mesh {
    return collide_with_mesh(model, geom1, geom2, pos1, mat1, pos2, mat2);
}

// Special case: hfield collision (existing)
if type1 == GeomType::Hfield || type2 == GeomType::Hfield {
    return collide_with_hfield(model, geom1, geom2, pos1, mat1, pos2, mat2);
}

// Special case: plane collision (existing)
// ... rest unchanged ...
```

**Exhaustive match arms.** Adding `GeomType::Sdf` requires new arms in:

- `collide_with_mesh()`: two inner `match prim_type` blocks (Mesh-vs-prim and
  prim-vs-Mesh). Add `GeomType::Sdf => unreachable!("handled by collide_with_sdf")`
  to both (same pattern as `GeomType::Hfield`).
- `collide_with_plane()`: inner `match other_type` block. Add
  `GeomType::Sdf => unreachable!("handled by collide_with_sdf")`.
- `collide_with_hfield()`: uses wildcard `_ => return None` — no change needed
  (Sdf falls into wildcard). However, since SDF dispatch is now before Hfield,
  this arm is unreachable for Sdf anyway.

**Implement `collide_with_sdf()`:**

```rust
/// Handle collision between an SDF geom and any other geom.
///
/// Dispatches to the appropriate `sdf_*_contact()` function from `sdf.rs`.
/// SDF contact normals point outward from the SDF surface. The pipeline
/// convention is that the normal points from geom1 toward geom2. When the
/// SDF is geom2 (not geom1), the normal must be negated.
fn collide_with_sdf(
    model: &Model, geom1: usize, geom2: usize,
    pos1: Vector3<f64>, mat1: Matrix3<f64>,
    pos2: Vector3<f64>, mat2: Matrix3<f64>,
) -> Option<Contact> {
    // Identify which geom is the SDF, which is the other
    let (sdf_geom, other_geom, sdf_pos, sdf_mat, other_pos, other_mat) =
        if model.geom_type[geom1] == GeomType::Sdf {
            (geom1, geom2, pos1, mat1, pos2, mat2)
        } else {
            (geom2, geom1, pos2, mat2, pos1, mat1)
        };

    let sdf_id = model.geom_sdf[sdf_geom]?;
    let sdf = &model.sdf_data[sdf_id];

    // Build SDF pose (no centering offset needed — SdfCollisionData uses
    // an arbitrary origin stored internally, unlike HeightFieldData which
    // uses corner-origin requiring a centering offset)
    let sdf_quat = UnitQuaternion::from_matrix(&sdf_mat);
    let sdf_pose = Pose::from_position_rotation(Point3::from(sdf_pos), sdf_quat);

    // Build other geom's parameters
    let other_quat = UnitQuaternion::from_matrix(&other_mat);
    let other_pose = Pose::from_position_rotation(Point3::from(other_pos), other_quat);
    let other_size = model.geom_size[other_geom];

    // Dispatch on the other geom's type
    let contact = match model.geom_type[other_geom] {
        GeomType::Sphere => {
            sdf_sphere_contact(sdf, &sdf_pose, other_pose.position, other_size.x)
        }
        GeomType::Capsule => {
            let axis = other_pose.rotation * Vector3::z();
            let start = other_pose.position - axis * other_size.y;
            let end = other_pose.position + axis * other_size.y;
            sdf_capsule_contact(sdf, &sdf_pose, start, end, other_size.x)
        }
        GeomType::Box => {
            sdf_box_contact(sdf, &sdf_pose, &other_pose, &other_size)
        }
        GeomType::Cylinder => {
            // sdf_cylinder_contact takes (pose, half_height, radius)
            // geom_size for Cylinder: x = radius, y = half_length
            sdf_cylinder_contact(sdf, &sdf_pose, &other_pose, other_size.y, other_size.x)
        }
        GeomType::Ellipsoid => {
            sdf_ellipsoid_contact(sdf, &sdf_pose, &other_pose, &other_size)
        }
        GeomType::Mesh => {
            // Use sdf_triangle_mesh_contact with the mesh's TriangleMeshData
            let mesh_id = model.geom_mesh[other_geom]?;
            let mesh_data = &model.mesh_data[mesh_id];
            sdf_triangle_mesh_contact(sdf, &sdf_pose, mesh_data, &other_pose)
        }
        GeomType::Hfield => {
            // Use sdf_heightfield_contact with the hfield's HeightFieldData
            let hfield_id = model.geom_hfield[other_geom]?;
            let hfield = &model.hfield_data[hfield_id];
            let hf_size = &model.hfield_size[hfield_id];
            // Apply centering offset (same as collide_with_hfield)
            let hf_offset = other_mat * Vector3::new(-hf_size[0], -hf_size[1], 0.0);
            let hf_pose = Pose::from_position_rotation(
                Point3::from(other_pos + hf_offset), other_quat,
            );
            sdf_heightfield_contact(sdf, &sdf_pose, hfield, &hf_pose)
        }
        GeomType::Plane => {
            // sdf_plane_contact takes (plane_normal, plane_offset)
            // Plane normal = Z-axis of plane's frame; offset = normal · position
            let plane_normal = other_mat.column(2).into_owned();
            let plane_offset = plane_normal.dot(&other_pos);
            sdf_plane_contact(sdf, &sdf_pose, &plane_normal, plane_offset)
        }
        GeomType::Sdf => {
            // SDF↔SDF — dispatch to sdf_sdf_contact
            let other_sdf_id = model.geom_sdf[other_geom]?;
            let other_sdf = &model.sdf_data[other_sdf_id];
            sdf_sdf_contact(sdf, &sdf_pose, other_sdf, &other_pose)
        }
    };

    // Convert SdfContact → pipeline Contact.
    //
    // Normal direction:
    // - For most sdf_*_contact functions: SdfContact.normal points outward
    //   from the SDF surface (away from interior, toward exterior).
    // - For sdf_plane_contact: normal is the plane normal (from plane toward SDF).
    // - For sdf_sdf_contact: normal is the outward gradient of whichever SDF
    //   the contact point is closer to the surface of.
    //
    // Pipeline convention: normal must point from geom1 toward geom2.
    //
    // When the SDF is geom1: the SDF outward normal points away from geom1
    // (the SDF), which is toward geom2. This matches the pipeline convention.
    //
    // When the SDF is geom2 (swapped = true): the SDF outward normal points
    // away from geom2, which is away from geom2 toward geom1 — opposite of
    // the pipeline convention. Negate it.
    //
    // Exception — sdf_plane_contact: the normal points from the plane toward
    // the SDF. If the SDF is geom1 and the plane is geom2, the plane-toward-SDF
    // normal points from geom2 toward geom1 — wrong direction. If the SDF is
    // geom2 and the plane is geom1, the plane-toward-SDF normal points from
    // geom1 toward geom2 — correct. So for Plane, the swap logic is inverted.
    // However, the standard swap (negate when SDF is geom2) handles this:
    //   - SDF=geom1, Plane=geom2: normal points plane→SDF = geom2→geom1.
    //     swapped=false, no negate → WRONG direction.
    //   - Need special case: for Plane, negate when NOT swapped.
    //
    // Exception — sdf_sdf_contact: the normal is the outward gradient of
    // whichever SDF is nearer to the surface. This is NOT consistently from
    // sdf_a outward. The direction depends on which SDF dominates at the
    // contact point. For correctness, we would need to determine which SDF
    // contributed the normal and adjust accordingly. However, since
    // sdf_sdf_contact is symmetric and the normal is the "best separation
    // direction," we treat it the same as the standard case: the normal is
    // approximately outward from the first SDF argument (sdf_a = our SDF geom),
    // and swap when needed. This is a conservative approximation — the contact
    // solver will still produce reasonable forces even if the normal is
    // slightly off.
    let swapped = sdf_geom != geom1;
    let is_plane = model.geom_type[other_geom] == GeomType::Plane;

    contact.map(|c| {
        let normal = match (swapped, is_plane) {
            // Standard: SDF is geom1, normal points outward from SDF (geom1→geom2) ✓
            (false, false) => c.normal,
            // Standard: SDF is geom2, negate to get geom1→geom2
            (true, false) => -c.normal,
            // Plane: SDF is geom1, normal points plane→SDF (geom2→geom1), negate
            (false, true) => -c.normal,
            // Plane: SDF is geom2, normal points plane→SDF (geom1→geom2) ✓
            (true, true) => c.normal,
        };
        make_contact_from_geoms(model, c.point.coords, normal, c.penetration, geom1, geom2)
    })
}
```

**Update `geom_to_collision_shape()`** (`mujoco_pipeline.rs:3968`):

```rust
GeomType::Sdf => None,  // Handled via collide_with_sdf()
```

**Update `aabb_from_geom()`** (`mujoco_pipeline.rs:3404`):

`aabb_from_geom` has **no Model reference** — only `(geom_type, size, pos, mat)`.
For MJCF-loaded SDF geoms, `geom_size` is `(0.1, 0.1, 0.1)` (the catch-all
default from `geom_size_to_vec3`). For programmatically constructed models,
`geom_size` should be set to meaningful half-extents by the user. Use the same
conservative rotated-box approach as Hfield:

```rust
GeomType::Sdf => {
    // Conservative: rotated box with half-extents from geom_size.
    // For programmatic SDF geoms, geom_size should store meaningful
    // half-extents. For MJCF placeholders, defaults to 0.1 —
    // small AABB is acceptable since the geom has no sdf_data.
    let half = size;
    let mut min = pos;
    let mut max = pos;
    for i in 0..3 {
        let axis = mat.column(i).into_owned();
        let extent = half[i] * axis.abs();
        min -= extent;
        max += extent;
    }
    Aabb::new(Point3::from(min), Point3::from(max))
}
```

#### Scope Exclusions

- **MJCF `<sdf>` asset element:** No such element exists in standard MuJoCo.
  SDF geoms are populated programmatically. A future `<sdf>` asset element for
  inline distance grids could be added without API changes.
- **ConvexMesh dispatch via `sdf_convex_mesh_contact`:** The pipeline's `GeomType`
  does not have a `ConvexMesh` variant (meshes are `TriangleMeshData`).
  `sdf_convex_mesh_contact` is unused in the pipeline dispatch but remains
  available for direct API use.

#### Acceptance Criteria

1. A programmatically constructed `Model` with `GeomType::Sdf` and valid
   `SdfCollisionData` produces working collision with Sphere, Capsule, Box,
   Cylinder, Ellipsoid, Mesh, Hfield, Plane, and SDF geoms.
2. `<geom type="sdf"/>` in MJCF produces a `Model` with `GeomType::Sdf` and
   `geom_sdf == None`. No error is thrown; the geom is silently non-colliding
   until `sdf_data` and `geom_sdf` are populated programmatically.
3. SDF bounding radius uses `SdfCollisionData::aabb()` half-diagonal (post-build
   pass). For MJCF placeholders without `sdf_data`, the primitive
   `bounding_radius()` fallback (`size.norm()`) is used.
4. SDF AABB in `aabb_from_geom()` uses the conservative rotated-box formula
   with `geom_size`.
5. SDF↔SDF collision works for two SDF geoms in the same scene via
   `sdf_sdf_contact()`.
6. Contact normal direction is correct regardless of geom ordering: whether
   the SDF is geom1 or geom2, the contact normal points from geom1 toward
   geom2 (consistent with pipeline convention). Special handling for
   `sdf_plane_contact` (normal points plane→SDF, not SDF outward).
7. `collide_with_sdf()` is dispatched **before** Mesh/Hfield/Plane in
   `collide_geoms()` so SDF↔Mesh, SDF↔Hfield, and SDF↔Plane are all handled
   by the SDF dispatcher (not by `collide_with_mesh`/`collide_with_hfield`/
   `collide_with_plane`).
8. Adding `GeomType::Sdf` does not cause compile errors: `unreachable!()` arms
   are added in `collide_with_mesh()` (2 inner matches) and
   `collide_with_plane()` (1 inner match).
9. `Model::empty()` includes all new fields (`nsdf`, `sdf_data`, `geom_sdf`)
   with empty/zero initialization.
10. SDF↔Hfield collision applies the centering offset (`-hf_size[0]`,
    `-hf_size[1]`) to the hfield pose, same as `collide_with_hfield()`.
11. Unsupported programmatic combinations (SDF geom with `geom_sdf == None`)
    silently return no contact — no panic, no error.

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — `GeomType::Sdf` + `bounding_radius()`
  arm, `sdf_data`/`nsdf`/`geom_sdf` on `Model` + `Model::empty()`,
  `collide_with_sdf()`, SDF dispatch in `collide_geoms()`, `aabb_from_geom()`
  arm, `geom_to_collision_shape()` arm, `unreachable!()` arms in
  `collide_with_mesh()` and `collide_with_plane()`
- `sim/L0/mjcf/src/model_builder.rs` — `MjcfGeomType::Sdf → GeomType::Sdf`
  (remove Box fallback), `geom_sdf` field + push, `build()` wiring,
  post-build bounding radius SDF branch

---

### 7. Deferred Sensors (JointLimitFrc, TendonLimitFrc)
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State
32 MJCF sensor types are parsed (`MjcfSensorType` in `types.rs`). Of these, 30
are converted to pipeline sensor types (`MjSensorType` in `mujoco_pipeline.rs`)
via `convert_sensor_type()` in `model_builder.rs:2694–2725` and fully evaluated
in the pipeline. Two are recognized at parse time but return `None` from
conversion and are skipped with a warning at model build time:

- `Jointlimitfrc` — joint limit constraint force
- `Tendonlimitfrc` — tendon limit constraint force

Neither has a corresponding `MjSensorType` variant. Both require reading
constraint forces from `qfrc_constraint` and mapping them back to the specific
joint/tendon limit that generated them.

#### Objective
Wire the two deferred sensors into the pipeline.

#### Specification
In `process_sensors()`, add evaluation arms for `JointLimitFrc` and
`TendonLimitFrc`:

- **JointLimitFrc:** After constraint solving, identify the constraint force
  contribution from the sensor's target joint limit. The force is the component
  of `qfrc_constraint` attributable to the joint's limit constraint (stored
  during PGS/CG solve). Return scalar force magnitude.

- **TendonLimitFrc:** Same pattern but for tendon limit constraints. The tendon
  limit force is the constraint force projected onto the tendon length direction.

Both sensors return 0 when the joint/tendon is within its limit range.

#### Acceptance Criteria
1. `JointLimitFrc` sensor reads the constraint force contribution from its
   joint's limit.
2. `TendonLimitFrc` sensor reads the constraint force contribution from its
   tendon's limit.
3. Zero force when joint/tendon is within limits.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (sensor evaluation in
  `process_sensors()`)

---

### 8. `<general>` Actuator MJCF Attributes
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State
The gain/bias runtime is fully general (any combination of Fixed/Affine/None
works). Only the MJCF parser-to-builder wiring for `<general>` actuators is
missing. Currently `<general>` actuators are treated as Motor-like
(gaintype=Fixed, gainprm=[1,0,0], biastype=None).

Deferred in Phase 1 (#12) because no RL models in common use require it. Included
here for completeness.

#### Objective
Parse explicit `gaintype`, `biastype`, `dyntype`, `gainprm`, `biasprm`, `dynprm`
attributes on `<general>` actuator elements.

#### Specification
In `parser.rs`, when parsing `<general>` actuators, read optional attributes:

- `gaintype` → `GainType` (Fixed, Affine, Muscle)
- `biastype` → `BiasType` (None, Affine, Muscle)
- `dyntype` → `ActuatorDynamics` (None, Filter, FilterExact, Integrator, Muscle)
- `gainprm`, `biasprm`, `dynprm` → `[f64; 10]` parameter arrays

In `model_builder.rs`, when the actuator type is `General` and explicit types are
provided, use them directly instead of the Motor-like defaults.

#### Acceptance Criteria
1. `<general gaintype="affine" gainprm="0 0 -5" biastype="none"/>` produces a
   damper-like actuator with `GainType::Affine`.
2. `<general>` without explicit type attributes still defaults to Motor-like
   (Fixed/None) — backward compatible.
3. All valid combinations of gaintype/biastype produce correct force output.

#### Files
- `sim/L0/mjcf/src/parser.rs` — modify (parse attributes on `<general>`)
- `sim/L0/mjcf/src/model_builder.rs` — modify (wire parsed types)

---

## Group B — Scaling & Performance

### 9. Batched Simulation
**Status:** Not started | **Effort:** L | **Prerequisites:** None

*Transferred from [future_work_1.md](./future_work_1.md) #10.*

#### Current State
Single-environment execution. `Data::step(&mut self, &Model)` (`mujoco_pipeline.rs:2599`)
steps one simulation. `Model` is immutable after construction, uses
`Arc<TriangleMeshData>` for shared mesh data (`mujoco_pipeline.rs:912`). `Data` is
fully independent — no shared mutable state, no interior mutability, derives `Clone`
(`mujoco_pipeline.rs:1391`).

#### Objective
Step N independent environments in parallel on CPU. Foundation for GPU acceleration
(#10) and large-scale RL training.

#### Specification

```rust
pub struct BatchSim {
    model: Arc<Model>,
    envs: Vec<Data>,
}

impl BatchSim {
    pub fn new(model: Arc<Model>, n: usize) -> Self;
    pub fn step_all(&mut self) -> BatchResult;
    pub fn reset(&mut self, env_idx: usize);
    pub fn reset_where(&mut self, mask: &[bool]);
}
```

`step_all()` uses rayon `par_iter_mut` over `envs`. Each `Data` steps independently
against the shared `Arc<Model>`. rayon 1.10 is already a workspace dependency;
sim-core declares it optional under the `parallel` feature flag
(`core/Cargo.toml:19,33`).

```rust
pub struct BatchResult {
    pub states: DMatrix<f64>,       // (n_envs, nq + nv) row-major
    pub rewards: DVector<f64>,      // (n_envs,)
    pub terminated: Vec<bool>,      // per-env episode termination
    pub truncated: Vec<bool>,       // per-env time limit
    pub errors: Vec<Option<StepError>>, // None = success
}
```

`states` is a contiguous matrix for direct consumption by RL frameworks (numpy
interop via row-major layout). Reward computation is user-defined:

```rust
pub trait RewardFn: Send + Sync {
    fn compute(&self, model: &Model, data: &Data) -> f64;
}
```

**Design constraint — single Model:** All environments share the same `Arc<Model>`
(same nq, nv, geom set). The `states: DMatrix<f64>` layout (n_envs, nq + nv)
requires uniform state dimensions. Multi-model batching (different robots in the
same batch) would require a fundamentally different design and is explicitly
out of scope.

**Error handling:** Environments that fail (e.g., `CholeskyFailed`,
`SingularMassMatrix`) are recorded in `errors`, flagged in `terminated`, and
auto-reset on the next `step_all()` call. The batch never aborts due to a single
environment failure.

**SIMD integration:** sim-simd provides within-environment acceleration:
`batch_dot_product_4()`, `batch_aabb_overlap_4()`, `batch_normal_force_4()`,
`batch_friction_force_4()`, `batch_integrate_position_4()`,
`batch_integrate_velocity_4()`. These accelerate the inner loop of each
environment's step. Cross-environment parallelism comes from rayon, not SIMD.

#### Acceptance Criteria
1. `BatchSim::step_all()` produces identical results to calling `Data::step()` on each environment sequentially — parallelism does not change simulation output.
2. `states` matrix layout is stable: row = env, cols = qpos ++ qvel.
3. Failed environments do not affect healthy environments in the same batch.
4. Linear throughput scaling up to available CPU cores (verified by benchmark with 1, 2, 4, 8 threads).
5. Zero-copy state extraction — `states` is filled directly from `Data` fields without intermediate allocations.
6. `reset_where()` resets only flagged environments without touching others.

#### Files
- `sim/L0/core/src/` — create `batch.rs` module
- `sim/L0/core/Cargo.toml` — modify (enable rayon under `parallel` feature)

---

### 10. GPU Acceleration
**Status:** Not started | **Effort:** XL | **Prerequisites:** #9

*Transferred from [future_work_1.md](./future_work_1.md) #11.*

#### Current State
CPU-only. No GPU infrastructure exists in the simulation pipeline. The `mesh-gpu`
crate provides wgpu context and buffer management for rendering but has no compute
shader infrastructure.

#### Objective
wgpu compute shader backend for batch simulation. Thousands of parallel environments
on a single GPU for RL training at scale.

#### Specification

Port the inner loop of `Data::step()` (FK, collision, PGS, integration) to compute
shaders via wgpu. The `BatchSim` API from #9 defines the memory layout that the
GPU backend fills.

This item is intentionally kept sparse — it is blocked on #9 and should not be
over-specified until the CPU batching API stabilizes. Key design decisions to be
made at implementation time:

- Which pipeline stages move to GPU first (integration is simplest, collision is
  most impactful).
- Data transfer strategy (host-pinned memory, persistent GPU buffers, double-buffering).
- Whether to use wgpu compute shaders or a lower-level API (Vulkan compute, Metal).
- Fallback path for systems without GPU support.

#### Acceptance Criteria
1. GPU-batched simulation produces identical results to CPU-batched (#9) for the same inputs.
2. For >= 1024 environments, GPU throughput exceeds CPU throughput on supported hardware.
3. Graceful fallback to CPU batching when no GPU is available.

#### Files
- `sim/L0/core/src/` — create `gpu.rs` module or new `sim-gpu` crate
- `mesh-gpu/` — reference for wgpu context management
