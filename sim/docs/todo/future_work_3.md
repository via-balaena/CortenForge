# Future Work 3 — Correctness Remainder + Scaling (Items #6–10)

Part of [Simulation Phase 2 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

---

### 6a. Height Field MJCF Wiring
**Status:** Not started | **Effort:** M | **Prerequisites:** None

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
**Status:** Not started | **Effort:** S | **Prerequisites:** 6a

#### Current State

**sim-core** has full SDF collision support:
- `SdfCollisionData` (`sim-core/src/sdf.rs:100–118`) — 3D grid with uniform
  `cell_size`, ZYX storage order, arbitrary origin.
- `CollisionShape::Sdf { data: Arc<SdfCollisionData> }` with constructor
  `CollisionShape::sdf(data)` (`collision_shape.rs:320`).
- Contact functions for all 10 shape pairs: Sphere, Capsule, Box, Cylinder,
  Ellipsoid, ConvexMesh, Plane, TriangleMesh, HeightField, Sdf↔Sdf —
  all in `sdf.rs`.

**MJCF layer** recognizes `type="sdf"` but does not wire it:
- `MjcfGeomType::Sdf` is parsed (`types.rs:777`).
- `model_builder.rs:1007–1013` falls back to `GeomType::Box` with a warning.
- `GeomType` enum has no `Sdf` variant.

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

Add `Sdf` variant to `GeomType` (alongside `Hfield` from 6a):

```rust
pub enum GeomType {
    Plane, Sphere, Capsule, Cylinder, Box, Ellipsoid, Mesh,
    Hfield, Sdf,
}
```

**Step 2 — Model storage** (`mujoco_pipeline.rs`)

Add to `Model`:

```rust
pub nsdf: usize,
pub sdf_data: Vec<Arc<SdfCollisionData>>,
pub geom_sdf: Vec<Option<usize>>,  // sdf_id per geom
```

**Step 3 — Model builder** (`model_builder.rs`)

- `MjcfGeomType::Sdf` → `GeomType::Sdf` (remove fallback to `Box`).
- SDF geoms in MJCF will fail with a clear error: "SDF geoms must be constructed
  programmatically via Model API — no MJCF asset format exists." This is because
  there is no `<sdf>` asset element to parse. Users construct SDF geoms by building
  a `Model` directly or by post-processing the model after MJCF loading.

**Step 4 — Collision pipeline** (`mujoco_pipeline.rs`)

In `collide_geoms()`, add SDF branch (after hfield, before GJK/EPA):

```rust
if type1 == GeomType::Sdf || type2 == GeomType::Sdf {
    return collide_with_sdf(model, geom1, geom2, pos1, mat1, pos2, mat2);
}
```

Implement `collide_with_sdf()` dispatching to the existing `sdf_*_contact()`
functions from `sdf.rs`, converting `SdfContact` → `Contact`.

Update `geom_to_collision_shape()` — `GeomType::Sdf => None`.

Update `aabb_from_geom()` — add `GeomType::Sdf` arm. Like Hfield (see 6a),
`aabb_from_geom` has **no Model reference** — only `(geom_type, size, pos, mat)`.
Use the same conservative rotated-box approach as Hfield, with `geom_size`
storing the SDF grid half-extents. If `geom_size` is not meaningful for SDF
(programmatic construction), use a conservative fallback similar to Mesh
(`const SDF_DEFAULT_EXTENT: f64 = 10.0`).

Update the post-build bounding radius pass (line 2392) to add an SDF branch
after hfield, following the same AABB-based half-diagonal pattern using
`SdfCollisionData::aabb()` (if available) or `geom_size`.

#### Acceptance Criteria

1. A programmatically constructed `Model` with `GeomType::Sdf` and valid
   `SdfCollisionData` produces working collision with all supported primitives.
2. `<geom type="sdf"/>` in MJCF produces a clear error explaining that SDF geoms
   require programmatic construction.
3. SDF bounding radius and AABB are correct for broad-phase culling.
4. SDF↔SDF collision works for two SDF geoms in the same scene.

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — `GeomType::Sdf`, `sdf_data`/`geom_sdf`
  on `Model`, `collide_with_sdf()`, AABB/bounding radius
- `sim/L0/mjcf/src/model_builder.rs` — error on MJCF SDF geoms (no asset to wire)

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
