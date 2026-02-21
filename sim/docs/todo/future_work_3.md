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

**MJCF layer** fully wires hfield geoms:
- `MjcfGeomType::Hfield` parsed from `type="hfield"` (`types.rs:775`).
- `MjcfHfield` struct stores asset data (`types.rs`); `MjcfGeom.hfield` links geom
  to asset; `MjcfModel.hfields` stores parsed assets.
- `parse_asset()` (`parser.rs`) parses `<hfield>` elements (both `Event::Start` and
  `Event::Empty`). `parse_hfield_attrs()` validates nrow/ncol/size/elevation.
- `model_builder.rs`: `process_hfield()` converts MJCF hfield to `HeightFieldData`;
  `process_geom()` maps `MjcfGeomType::Hfield → GeomType::Hfield` with asset linking
  and geom_size override from `hfield_size`.
- `GeomType::Hfield` variant in enum (`mujoco_pipeline.rs`).

**Collision pipeline** has full hfield dispatch:
- `collide_geoms()` dispatches `GeomType::Hfield` to `collide_with_hfield()`.
- `collide_with_hfield()` handles Sphere, Capsule, Box (direct), Cylinder (capsule
  approximation), Ellipsoid (sphere approximation). Hfield↔Hfield/Plane/Mesh return None.
- `geom_to_collision_shape()`: `Hfield => None` (handled by dedicated dispatcher).
- `aabb_from_geom()`: rotated-box formula using hfield half-extents.
- `bounding_radius()`: conservative estimate from geom_size; post-build pass overwrites
  with true AABB half-diagonal from `HeightFieldData::aabb()`.

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
  Tracked in [future_work_10b.md](./future_work_10b.md) §DT-3.
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

**MJCF layer** wires `type="sdf"` for programmatic construction:
- `MjcfGeomType::Sdf` is parsed (`types.rs`).
- `model_builder.rs`: `MjcfGeomType::Sdf → GeomType::Sdf` (no Box fallback).
  `geom_sdf` always `None` from MJCF — SDF data is populated programmatically.
- `GeomType::Sdf` variant in enum (`mujoco_pipeline.rs`).
- `Model` stores `nsdf`, `sdf_data: Vec<Arc<SdfCollisionData>>`, and
  `geom_sdf: Vec<Option<usize>>`.

**Collision pipeline** has full SDF dispatch:
- `collide_geoms()` dispatch order: SDF → Mesh → Hfield → Plane → analytical → GJK/EPA.
  SDF is dispatched **first** so SDF↔Mesh, SDF↔Hfield, and SDF↔Plane are all
  handled by `collide_with_sdf()`.
- `collide_with_sdf()` dispatches to all 10 `sdf_*_contact()` functions for every
  `GeomType` (Sphere, Capsule, Box, Cylinder, Ellipsoid, Mesh, Hfield, Plane, SDF).
  Normal direction uses XOR logic (`swapped ^ is_plane`) for the plane exception.
- `geom_to_collision_shape()`: `Sdf => None` (handled by dedicated dispatcher).
- `aabb_from_geom()`: rotated-box formula using geom_size half-extents.
- `bounding_radius()`: `size.norm()` fallback; post-build pass overwrites with
  true AABB half-diagonal from `SdfCollisionData::aabb()`.
- `collide_with_mesh()` and `collide_with_plane()`: `Sdf => unreachable!()` arms.

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
  Tracked in [future_work_10b.md](./future_work_10b.md) §DT-4.
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
**Status:** ✅ Done | **Effort:** S | **Prerequisites:** None

#### Summary

All 32 MJCF sensor types are now fully wired into the pipeline. The two
formerly-deferred types — `JointLimitFrc` and `TendonLimitFrc` — read cached
penalty force magnitudes from `mj_fwd_constraint()` and report them as
scalar (1D) Acceleration-stage sensors.

#### Architecture

**Constraint solver context:** CortenForge uses a **penalty method** for
joint and tendon limit constraints (not the PGS/CG solver, which handles
contacts only). MuJoCo stores per-constraint forces in `efc_force[]` with
typed metadata (`efc_type[]`, `efc_id[]`) and sensors scan those arrays.
CortenForge has no per-constraint force arrays — limit forces are computed
inline and immediately summed into `qfrc_constraint`. To support these
sensors, the scalar penalty force is **cached** during `mj_fwd_constraint()`
before being dispersed into generalized coordinates.

**Sign convention:** The cached value is the **unsigned penalty magnitude**
`f = stiffness × penetration + damping × vel_into` (always ≥ 0), matching
MuJoCo's non-negative `efc_force` convention. The directional sign is applied
separately when accumulating into `qfrc_constraint`.

**Pipeline ordering:** `mj_fwd_constraint()` populates the caches →
`mj_fwd_acceleration()` → `mj_sensor_acc()` reads the caches.

#### Implementation

**`Data` cache fields** (`mujoco_pipeline.rs`, after `qfrc_constraint`):
- `jnt_limit_frc: Vec<f64>` — per-joint limit force cache (length `njnt`)
- `ten_limit_frc: Vec<f64>` — per-tendon limit force cache (length `ntendon`)

Both are initialized to zero in `Model::make_data()` and cleared at the top
of `mj_fwd_constraint()` each forward pass.

**Cache population** (`mj_fwd_constraint()`):
- Joint limits: in the Hinge/Slide branch, the penalty `force` is extracted
  as a local variable and cached via `data.jnt_limit_frc[jnt_id] = force`
  in both lower-limit and upper-limit branches.
- Tendon limits: `data.ten_limit_frc[t] = force` in both branches. The
  tendon code uses two `if` blocks (not `if/else if`), so in the degenerate
  case where `limit_min > limit_max`, the upper-limit value overwrites the
  lower-limit value (matching the existing `qfrc_constraint` additive
  behavior — physically meaningless configuration).

**Sensor evaluation** (`mj_sensor_acc()`):
- `JointLimitFrc` → `sensor_write(data.jnt_limit_frc[objid])`
- `TendonLimitFrc` → `sensor_write(data.ten_limit_frc[objid])`

Both are simple cache reads with bounds checking.

**Model builder wiring** (`model_builder.rs`):
- `convert_sensor_type()`: `Jointlimitfrc → JointLimitFrc`,
  `Tendonlimitfrc → TendonLimitFrc`
- `sensor_datatype()`: both in the `Acceleration` arm
- `resolve_sensor_object()`: `JointLimitFrc` in the joint arm (resolves
  `objname` via `joint_name_to_id`), `TendonLimitFrc` in the tendon arm
  (resolves via `tendon_name_to_id`)

**`MjSensorType` enum** (`mujoco_pipeline.rs`):
- `JointLimitFrc` and `TendonLimitFrc` variants after `ActuatorFrc`
- Both in the `dim() => 1` arm

#### MuJoCo Reference

Source: `engine_sensor.c`, `mj_computeSensorAcc`, cases
`mjSENS_JOINTLIMITFRC` / `mjSENS_TENDONLIMITFRC`. MuJoCo scans `efc_force[]`
in `[ne+nf, nefc)` for matching `efc_type`/`efc_id`; returns
`efc_force[j]` (non-negative constraint multiplier λ) or 0 if not active.

#### Verification

All 12 acceptance criteria verified:

1. ✅ Basic joint limit force — penalty `f = stiffness × penetration +
   damping × vel_into` cached and read correctly
2. ✅ Basic tendon limit force — same formula, tendon cache path
3. ✅ Zero when within limits — caches cleared each step, only written
   inside violation branches
4. ✅ Non-negative output — `penetration ≥ 0`, `vel_into ≥ 0`,
   `stiffness ≥ 0`, `damping ≥ 0` → `force ≥ 0` always
5. ✅ Non-limited joint/tendon — `continue` skips non-limited, cache
   remains at cleared 0.0
6. ✅ Cache cleared each step — `fill(0.0)` at top of
   `mj_fwd_constraint()`
7. ✅ MJCF round-trip — `test_joint_limit_frc_sensor` validates
   `nsensor == 1`, `nsensordata == 1`, correct `sensor_type`
8. ✅ Object resolution errors — `joint_name_to_id.get()` returns
   `ModelConversionError` for unknown names
9. ✅ Sensor data stage — `sensor_datatype() == Acceleration`
10. ✅ Multiple sensors on same joint — shared cache entry, same value
11. ✅ Postprocessing — not in positive-only list, gets real-type
    `[-cutoff, cutoff]` clamping
12. ✅ Sensor dimension — `dim() == 1`

#### Files Changed

- `sim/L0/core/src/mujoco_pipeline.rs` — `JointLimitFrc`/`TendonLimitFrc`
  enum variants + `dim()` arm; `jnt_limit_frc`/`ten_limit_frc` cache fields
  on `Data` + initialization in `Model::make_data()`; cache clearing and
  population in `mj_fwd_constraint()`; evaluation arms in `mj_sensor_acc()`
- `sim/L0/mjcf/src/model_builder.rs` — `convert_sensor_type()` mappings;
  `sensor_datatype()` Acceleration arm; `resolve_sensor_object()` joint and
  tendon arms
- `sim/L0/tests/integration/mjcf_sensors.rs` — `test_joint_limit_frc_sensor`
  (replaced former `test_unsupported_sensor_skipped`)

---

### 8. `<general>` Actuator MJCF Attributes
**Status:** Done | **Effort:** S | **Prerequisites:** None

#### Current State

**Runtime layer is fully general.** `mj_fwd_actuation()`
(`mujoco_pipeline.rs:8300`) dispatches on `GainType` (Fixed/Affine/Muscle),
`BiasType` (None/Affine/Muscle), and `ActuatorDynamics`
(None/Filter/FilterExact/Integrator/Muscle) using per-actuator parameter
arrays `gainprm: [f64; 9]`, `biasprm: [f64; 9]`, `dynprm: [f64; 3]`. The
force equation `force = gain * input + bias` is computed at line 8379. All
type combinations work correctly at runtime — no runtime changes needed.

**MJCF layer is incomplete.** The parser recognizes `<general>` as
`MjcfActuatorType::General` (`types.rs:1790`) and parses common attributes
(name, joint, gear, ctrlrange, etc.) via `parse_actuator_attrs()`
(`parser.rs:1316`). However, **six critical attributes are not parsed**:
`gaintype`, `biastype`, `dyntype`, `gainprm`, `biasprm`, `dynprm`.

**Model builder hardcodes Motor-like defaults.** `process_actuator()`
(`model_builder.rs:1668–1675`) has a TODO comment and treats all `<general>`
actuators as Motor-like: `GainType::Fixed`, `BiasType::None`,
`gainprm=[1,0,...,0]`, `biasprm=[0,...,0]`, `dynprm=[0,0,0]`. The dyntype
determination (`model_builder.rs:1521–1526`) also hardcodes General →
`ActuatorDynamics::None`, ignoring any parsed `dyntype`.

**`MjcfActuator` struct lacks fields** (`types.rs:1819–1890`) for
`gaintype`, `biastype`, `dyntype`, `gainprm`, `biasprm`, `dynprm`.

**`MjcfActuatorDefaults` struct lacks fields** (`types.rs:486–501`) for
the general actuator attributes. `apply_to_actuator()` (`defaults.rs:239–296`)
does not propagate them.

**`actlimited`/`actrange` parsing is complete** — parsed on `MjcfActuator`
and `MjcfActuatorDefaults`, with full defaults pipeline (merge + apply).
Not yet stored on `Model` or enforced at runtime. Runtime clamping is
tracked as #34 in [future_work_9.md](./future_work_9.md).

#### Objective

Parse explicit `gaintype`, `biastype`, `dyntype`, `gainprm`, `biasprm`,
`dynprm` attributes on `<general>` actuator elements and wire them through
the model builder so the existing runtime dispatch produces correct behavior
for arbitrary `<general>` actuator configurations.

#### MuJoCo Reference

Source: `mjmodel.h` (constants, enums), `user_init.c`
(`mjs_defaultActuator`), `user_api.cc` (`mjs_setToMotor` et al.),
`xml_native_reader.cc` (parser).

**`<general>` default attribute values** (from `mjs_defaultActuator`):

| Attribute | Default | Notes |
|-----------|---------|-------|
| `gaintype` | `"fixed"` (mjGAIN_FIXED) | |
| `biastype` | `"none"` (mjBIAS_NONE) | |
| `dyntype` | `"none"` (mjDYN_NONE) | |
| `gainprm` | `[1, 0, 0, 0, 0, 0, 0, 0, 0, 0]` | 10 elements (mjNGAIN=10) |
| `biasprm` | `[0, 0, 0, 0, 0, 0, 0, 0, 0, 0]` | 10 elements (mjNBIAS=10) |
| `dynprm` | `[1, 0, 0, 0, 0, 0, 0, 0, 0, 0]` | 10 elements (mjNDYN=10) |
| `gear` | `[1, 0, 0, 0, 0, 0]` | Already parsed |

These defaults mean that a bare `<general joint="j"/>` behaves identically
to `<motor joint="j"/>` — fixed unit gain, no bias, no dynamics.

**Enum string mappings** (from `xml_native_reader.cc`):

`gaintype` (gain_map):
- `"fixed"` → mjGAIN_FIXED → `GainType::Fixed`
- `"affine"` → mjGAIN_AFFINE → `GainType::Affine`
- `"muscle"` → mjGAIN_MUSCLE → `GainType::Muscle`
- `"user"` → mjGAIN_USER → **out of scope** (callback)

`biastype` (bias_map):
- `"none"` → mjBIAS_NONE → `BiasType::None`
- `"affine"` → mjBIAS_AFFINE → `BiasType::Affine`
- `"muscle"` → mjBIAS_MUSCLE → `BiasType::Muscle`
- `"user"` → mjBIAS_USER → **out of scope** (callback)

`dyntype` (dyn_map):
- `"none"` → mjDYN_NONE → `ActuatorDynamics::None`
- `"integrator"` → mjDYN_INTEGRATOR → `ActuatorDynamics::Integrator`
- `"filter"` → mjDYN_FILTER → `ActuatorDynamics::Filter`
- `"filterexact"` → mjDYN_FILTEREXACT → `ActuatorDynamics::FilterExact`
- `"muscle"` → mjDYN_MUSCLE → `ActuatorDynamics::Muscle`
- `"user"` → mjDYN_USER → **out of scope** (callback)

**Parameter array sizing — CortenForge divergence:**

MuJoCo uses 10-element arrays for all three (`mjNGAIN=10`, `mjNBIAS=10`,
`mjNDYN=10`). CortenForge uses `[f64; 9]` for gainprm/biasprm and
`[f64; 3]` for dynprm (`mujoco_pipeline.rs:1044–1055`). This is intentional:
- `gainprm`/`biasprm`: only indices 0–8 are used by Fixed/Affine/Muscle
  gain/bias (the 10th element, index 9, is only used by MuJoCo's `user`
  callback type, which is out of scope).
- `dynprm`: only indices 0–2 are used (tau_act, tau_deact, tausmooth for
  Muscle; tau for Filter/FilterExact; unused for Integrator/None).

When parsing MJCF `gainprm`/`biasprm`/`dynprm` attributes, accept up to
10 space-separated floats (matching MuJoCo) but only store the first 9/9/3
respectively. Extra values are silently ignored. If fewer values are
provided, pad with zeros (matching MuJoCo's `memset(0)` + selective
initialization).

**Shortcut actuator expansion** (for reference — these are already
implemented and should not change):

| Shortcut | gaintype | biastype | dyntype | gainprm[0..2] | biasprm[0..2] | dynprm[0] |
|----------|----------|----------|---------|---------------|---------------|-----------|
| Motor | fixed | none | none | [1, 0, 0] | [0, 0, 0] | 0 |
| Position | fixed | affine | none/filterexact | [kp, 0, 0] | [0, -kp, -kv] | tc |
| Velocity | fixed | affine | none | [kv, 0, 0] | [0, 0, -kv] | 0 |
| Damper | affine | none | none | [0, 0, -kv] | [0, 0, 0] | 0 |
| Cylinder | fixed | affine | filter | [area, 0, 0] | [b0, b1, b2] | tc |
| Adhesion | fixed | none | none | [gain, 0, 0] | [0, 0, 0] | 0 |
| Muscle | muscle | muscle | muscle | [range/F/scale/lim] | =gainprm | [τ_a, τ_d, 0] |

**Example MJCF — damper via `<general>`:**

```xml
<actuator>
  <general joint="slide" gaintype="affine" gainprm="0 0 -5"
           biastype="none" ctrllimited="true" ctrlrange="0 1"/>
</actuator>
```

This produces: `gain = 0 + 0*length + (-5)*velocity`, `bias = 0`,
`force = gain * ctrl + 0 = -5 * velocity * ctrl`. With `ctrl ∈ [0,1]`,
this is a controllable damper with maximum damping coefficient 5.

**Example MJCF — PD servo via `<general>`:**

```xml
<actuator>
  <general joint="hinge" gaintype="fixed" gainprm="100"
           biastype="affine" biasprm="0 -100 -10"
           dyntype="filterexact" dynprm="0.01"/>
</actuator>
```

This produces: `gain = 100`, `bias = 0 - 100*length - 10*velocity`,
`force = 100*act + (−100*length − 10*velocity)`. With FilterExact dynamics
(`τ = 0.01`), the activation tracks ctrl with a 10ms time constant. This
is equivalent to `<position kp="100" kv="10" timeconst="0.01"/>`.

#### Specification

**Step 1 — MJCF types** (`types.rs`)

Add fields to `MjcfActuator` (`types.rs:1889`, before the closing brace):

```rust
    // --- <general> actuator attributes ---
    // These are only meaningful for MjcfActuatorType::General.
    // For shortcut types (Motor, Position, etc.), these are ignored —
    // the model builder expands shortcuts using their own dedicated logic.

    /// Gain type: "fixed", "affine", "muscle".
    /// None means use default (Fixed). Only parsed for <general>.
    pub gaintype: Option<String>,
    /// Bias type: "none", "affine", "muscle".
    /// None means use default (None/no bias). Only parsed for <general>.
    pub biastype: Option<String>,
    /// Dynamics type: "none", "integrator", "filter", "filterexact", "muscle".
    /// None means use default (None/direct). Only parsed for <general>.
    pub dyntype: Option<String>,
    /// Gain parameters (up to 9 elements, zero-padded).
    /// None means use default [1, 0, ..., 0]. Only parsed for <general>.
    pub gainprm: Option<Vec<f64>>,
    /// Bias parameters (up to 9 elements, zero-padded).
    /// None means use default [0, ..., 0]. Only parsed for <general>.
    pub biasprm: Option<Vec<f64>>,
    /// Dynamics parameters (up to 3 elements, zero-padded).
    /// None means use default [1, 0, 0]. Only parsed for <general>.
    pub dynprm: Option<Vec<f64>>,
```

Update `MjcfActuator::default()` (`types.rs:1892`): add all six fields as
`None`.

Store types as `Option<String>` (not enums) in the MJCF layer. Enum
conversion happens in the model builder, where unknown values produce
`ModelConversionError`. This defers validation to the builder for simplicity
— `process_actuator()` has the actuator name in scope, so error messages
can include context (e.g., "actuator 'foo': unknown gaintype 'invalid'").
Note: this differs from `MjcfGeomType` and `MjcfActuatorType`, which
validate at parse time via `from_str()`. The `Option<String>` approach is
chosen here because these attributes are only meaningful for `<general>`
actuators and are ignored for all shortcut types — deferring validation
avoids parse-time errors for attributes that will never be consumed.

Add fields to `MjcfActuatorDefaults` (`types.rs:500`, before closing brace):

```rust
    /// Gain type default for <general> actuators.
    pub gaintype: Option<String>,
    /// Bias type default for <general> actuators.
    pub biastype: Option<String>,
    /// Dynamics type default for <general> actuators.
    pub dyntype: Option<String>,
    /// Gain parameters default for <general> actuators.
    pub gainprm: Option<Vec<f64>>,
    /// Bias parameters default for <general> actuators.
    pub biasprm: Option<Vec<f64>>,
    /// Dynamics parameters default for <general> actuators.
    pub dynprm: Option<Vec<f64>>,
```

`MjcfActuatorDefaults` uses `#[derive(Default)]` — the new `Option<String>`
and `Option<Vec<f64>>` fields auto-derive to `None`. No manual `Default`
update needed.

**Step 2 — Parser** (`parser.rs`)

In `parse_actuator_attrs()` (`parser.rs:1365`, after the `kp`/`kv` block,
before the cylinder-specific section), add a `<general>`-specific block:

```rust
// --- <general>-specific attributes ---
// These are only parsed for <general> actuators. For shortcut types,
// these attributes are not part of the MJCF schema and are ignored.
if actuator.actuator_type == MjcfActuatorType::General {
    actuator.gaintype = get_attribute_opt(e, "gaintype");
    actuator.biastype = get_attribute_opt(e, "biastype");
    actuator.dyntype = get_attribute_opt(e, "dyntype");
    actuator.gainprm = parse_float_array_opt(e, "gainprm")?;
    actuator.biasprm = parse_float_array_opt(e, "biasprm")?;
    actuator.dynprm = parse_float_array_opt(e, "dynprm")?;
}
```

`parse_float_array_opt()` is a new helper:

```rust
fn parse_float_array_opt(e: &BytesStart, name: &str) -> Result<Option<Vec<f64>>> {
    match get_attribute_opt(e, name) {
        Some(s) => Ok(Some(parse_float_array(&s)?)),
        None => Ok(None),
    }
}
```

Returns `Result<Option<Vec<f64>>>`: `Ok(None)` when the attribute is
absent, `Ok(Some(vec))` when present and valid, `Err` when present but
contains unparseable floats (e.g., `gainprm="1.0 abc"`). This follows the
existing `gear` parsing pattern (`parser.rs:1330–1337`) which propagates
parse errors via `?`. If zero elements are provided (empty string),
`parse_float_array` returns `Ok(vec![])` → `Ok(Some(vec![]))` —
`floats_to_array` in the model builder then returns the full default array
unchanged (e.g., `gainprm=""` silently produces `[1,0,...,0]`). This is a
harmless edge case — MuJoCo would reject an empty `gainprm` attribute, but
silently defaulting is acceptable behavior.

In `parse_actuator_defaults()` (`parser.rs:443–477`), add after the
`forcelimited` block (`parser.rs:472`):

```rust
defaults.gaintype = get_attribute_opt(e, "gaintype");
defaults.biastype = get_attribute_opt(e, "biastype");
defaults.dyntype = get_attribute_opt(e, "dyntype");
defaults.gainprm = parse_float_array_opt(e, "gainprm")?;
defaults.biasprm = parse_float_array_opt(e, "biasprm")?;
defaults.dynprm = parse_float_array_opt(e, "dynprm")?;
```

Note: `parse_actuator_defaults()` is called for **all** actuator default
element names — `<actuator>`, `<motor>`, `<position>`, `<velocity>`, and
`<general>` (`parser.rs:334`). Parsing these attributes unconditionally
(no type guard) is intentional: the single `MjcfActuatorDefaults` struct
stores defaults for all actuator element names in a class, and the model
builder ignores `gaintype`/`biastype`/`dyntype` for shortcut types. This
matches MuJoCo's defaults system where attributes are per-element-name
but stored in a shared defaults struct.

**Step 3 — Defaults** (`defaults.rs`)

In `apply_to_actuator()` (`defaults.rs:288`, after the `kv` block), add:

```rust
// Apply <general>-specific defaults.
// These only matter for <general> actuators, but we apply them
// unconditionally — the model builder ignores them for shortcut types.
if result.gaintype.is_none() {
    result.gaintype = defaults.gaintype.clone();
}
if result.biastype.is_none() {
    result.biastype = defaults.biastype.clone();
}
if result.dyntype.is_none() {
    result.dyntype = defaults.dyntype.clone();
}
if result.gainprm.is_none() {
    result.gainprm = defaults.gainprm.clone();
}
if result.biasprm.is_none() {
    result.biasprm = defaults.biasprm.clone();
}
if result.dynprm.is_none() {
    result.dynprm = defaults.dynprm.clone();
}
```

Note: the mutable variable is `result` (not `actuator`), matching the
existing function pattern where `let mut result = actuator.clone()` at
line 240 and all subsequent mutations operate on `result`.

The pattern `if result.field.is_none() { result.field = defaults.field.clone() }`
is the correct semantics: an explicit attribute on the element overrides
the class default, and an absent attribute inherits from the class default.
This matches how `ctrlrange`, `forcerange`, and `kv` are already handled.

**Step 4 — Model builder** (`model_builder.rs`)

**4a. Enum conversion helpers.** Add three private functions:

```rust
fn parse_gaintype(s: &str) -> Result<GainType, ModelConversionError> {
    match s {
        "fixed" => Ok(GainType::Fixed),
        "affine" => Ok(GainType::Affine),
        "muscle" => Ok(GainType::Muscle),
        _ => Err(ModelConversionError {
            message: format!("unknown gaintype '{s}' (valid: fixed, affine, muscle)"),
        }),
    }
}

fn parse_biastype(s: &str) -> Result<BiasType, ModelConversionError> {
    match s {
        "none" => Ok(BiasType::None),
        "affine" => Ok(BiasType::Affine),
        "muscle" => Ok(BiasType::Muscle),
        _ => Err(ModelConversionError {
            message: format!("unknown biastype '{s}' (valid: none, affine, muscle)"),
        }),
    }
}

fn parse_dyntype(s: &str) -> Result<ActuatorDynamics, ModelConversionError> {
    match s {
        "none" => Ok(ActuatorDynamics::None),
        "integrator" => Ok(ActuatorDynamics::Integrator),
        "filter" => Ok(ActuatorDynamics::Filter),
        "filterexact" => Ok(ActuatorDynamics::FilterExact),
        "muscle" => Ok(ActuatorDynamics::Muscle),
        _ => Err(ModelConversionError {
            message: format!(
                "unknown dyntype '{s}' (valid: none, integrator, filter, filterexact, muscle)"
            ),
        }),
    }
}
```

**4b. Parameter array conversion helper.** Add:

```rust
/// Convert a variable-length parsed float vector into a fixed-size array,
/// padding with the given default value. Truncates if input exceeds `N`.
fn floats_to_array<const N: usize>(input: &[f64], default: [f64; N]) -> [f64; N] {
    let mut out = default;
    for (i, &v) in input.iter().enumerate().take(N) {
        out[i] = v;
    }
    out
}
```

**4c. Dyntype determination** (`model_builder.rs:1521–1536`).

Change the General arm from the hardcoded `ActuatorDynamics::None` to use
the parsed `dyntype` attribute:

```rust
let dyntype = match actuator.actuator_type {
    MjcfActuatorType::Motor
    | MjcfActuatorType::Damper
    | MjcfActuatorType::Adhesion
    | MjcfActuatorType::Velocity => ActuatorDynamics::None,
    MjcfActuatorType::General => {
        // <general> uses explicit dyntype attribute; defaults to None
        match &actuator.dyntype {
            Some(s) => parse_dyntype(s)?,
            None => ActuatorDynamics::None,
        }
    }
    MjcfActuatorType::Position => {
        if timeconst > 0.0 {
            ActuatorDynamics::FilterExact
        } else {
            ActuatorDynamics::None
        }
    }
    MjcfActuatorType::Muscle => ActuatorDynamics::Muscle,
    MjcfActuatorType::Cylinder => ActuatorDynamics::Filter,
};
```

This is critical: dyntype must be determined **before** the activation state
count (`act_num`) computation at line 1565, because `act_num` depends on
`dyntype`. A `<general dyntype="filter">` must allocate 1 activation state.

**4d. Gain/bias/dynprm expansion** (`model_builder.rs:1668–1675`).

Replace the current General arm:

```rust
MjcfActuatorType::General => {
    // <general> uses explicit attributes; defaults match MuJoCo's
    // mjs_defaultActuator: gaintype=fixed, biastype=none,
    // gainprm=[1,0,...], biasprm=[0,...], dynprm=[1,0,0].
    let gt = match &actuator.gaintype {
        Some(s) => parse_gaintype(s)?,
        None => GainType::Fixed,
    };
    let bt = match &actuator.biastype {
        Some(s) => parse_biastype(s)?,
        None => BiasType::None,
    };
    let gp = match &actuator.gainprm {
        Some(v) => floats_to_array(v, {
            let mut d = [0.0; 9];
            d[0] = 1.0; // MuJoCo default
            d
        }),
        None => {
            let mut d = [0.0; 9];
            d[0] = 1.0;
            d
        },
    };
    let bp = match &actuator.biasprm {
        Some(v) => floats_to_array(v, [0.0; 9]),
        None => [0.0; 9],
    };
    let dp = match &actuator.dynprm {
        Some(v) => floats_to_array(v, {
            let mut d = [0.0; 3];
            d[0] = 1.0; // MuJoCo default
            d
        }),
        None => [1.0, 0.0, 0.0], // MuJoCo: dynprm[0]=1
    };
    (gt, bt, gp, bp, dp)
}
```

**Note on dynprm default.** MuJoCo defaults `dynprm[0]=1`. This is the
time constant for Filter/FilterExact dynamics. When `dyntype=none`, dynprm
is unused. When `dyntype=filter` without explicit `dynprm`, the default
`τ=1.0` gives a 1-second time constant — matching MuJoCo. The current
codebase uses `[0.0; 3]` for other shortcut types (Motor, Velocity, Damper),
which is correct for those types because they all have `dyntype=none`. The
`[1.0, 0.0, 0.0]` default only applies to the General case.

**Note on gainprm partial specification.** If the user writes
`gainprm="100"` (1 element), `floats_to_array` produces
`[100, 0, 0, 0, 0, 0, 0, 0, 0]` — the first element is overridden, the
rest keep the zero default. But the MuJoCo default for `gainprm[0]` is 1,
not 0. This is correct because MuJoCo's `ReadAttr` overwrites the entire
array from index 0 up to the number of provided values, and the remaining
elements keep their `memset(0)` initialization (the `gainprm[0]=1` default
from `mjs_defaultActuator` is overwritten by `ReadAttr`). Our
`floats_to_array` with a default array of `[1,0,...,0]` matches: index 0
is overwritten by the user's value, and indices beyond the user's input
retain the default array values (which are 0 for indices 1–8).

#### Scope Exclusions

- **`gaintype="user"`, `biastype="user"`, `dyntype="user"`:** MuJoCo's
  callback types. These require a plugin/callback system that does not exist
  in CortenForge. These values fall into the wildcard `_` arm of
  `parse_gaintype`/`parse_biastype`/`parse_dyntype` and produce
  `ModelConversionError` with message `"unknown gaintype 'user'"` (same
  error as any unrecognized string). No special-case handling is needed —
  the error message is clear enough. These can be added later without API
  changes.
  Tracked in [future_work_10b.md](./future_work_10b.md) §DT-5.
- **`actlimited` / `actrange`:** Activation clamping for stateful actuators.
  **Parsing complete** — both fields are parsed on `MjcfActuator` and
  `MjcfActuatorDefaults` with full defaults pipeline (Option<T>, merge,
  apply). Not yet stored on `Model` or enforced at runtime. Runtime clamping
  is tracked as #34 in [future_work_9.md](./future_work_9.md). Without
  runtime `actlimited` enforcement, `dyntype=integrator` actuators have
  unbounded activation — users must manage via `ctrlrange` or manual
  clamping.
- **`actearly`:** Controls whether activation dynamics are computed before
  or after the control signal. MuJoCo default is false. **Parsed** on
  `MjcfActuator` and `MjcfActuatorDefaults` with full defaults pipeline,
  but not yet wired to runtime — our pipeline always computes dynamics
  in the standard order.
  Tracked in [future_work_10b.md](./future_work_10b.md) §DT-6.
- **`actdim`:** Number of activation variables per actuator. MuJoCo default
  is -1 (auto-detect from dyntype: 0 for none, 1 for others). Our pipeline
  hardcodes the same auto-detection logic (`model_builder.rs:1565–1571`).
  Explicit `actdim` override is not needed.
  Tracked in [future_work_10b.md](./future_work_10b.md) §DT-7.
- **Additional transmission types:** `cranksite`, `slidersite`,
  `jointinparent` are not supported by the existing transmission resolver
  (`model_builder.rs:1425–1495`) and are not part of this work item.
  Tracked in [future_work_10b.md](./future_work_10b.md) §DT-8.
- **`nsample`, `interp`, `delay`:** MuJoCo 3.x additions for
  interpolation-based actuators. Out of scope.
  Tracked in [future_work_10b.md](./future_work_10b.md) §DT-9.
- **Default class for shortcut types:** If a `<default>` element sets
  `gaintype="affine"` and a `<motor>` uses that class, the `gaintype` is
  ignored for the motor (the model builder's shortcut expansion overrides
  it). This is correct MuJoCo behavior — class defaults for these attributes
  only affect `<general>` actuators.

#### Acceptance Criteria

1. **Explicit gaintype/biastype.** `<general joint="j" gaintype="affine"
   gainprm="0 0 -5" biastype="none" ctrllimited="true" ctrlrange="0 1"/>`
   produces `GainType::Affine`, `BiasType::None`, `gainprm=[0,0,-5,0,...,0]`.
   Force = `(0 + 0*length − 5*velocity) * ctrl + 0`.
2. **Explicit dyntype.** `<general joint="j" dyntype="filter"
   dynprm="0.05"/>` produces `ActuatorDynamics::Filter`, `dynprm=[0.05,0,0]`,
   and allocates 1 activation state (`act_num == 1`).
3. **Bare `<general>` backward compatibility.** `<general joint="j"/>`
   without explicit type attributes produces Motor-like behavior:
   `GainType::Fixed`, `BiasType::None`, `ActuatorDynamics::None`,
   `gainprm=[1,0,...,0]`, `biasprm=[0,...,0]`, `dynprm=[1,0,0]`. Force =
   `1 * ctrl + 0 = ctrl`. Note: `dynprm` changes from `[0,0,0]` (current
   codebase) to `[1,0,0]` (MuJoCo default). This is functionally irrelevant
   because `dyntype=none` ignores `dynprm`, but aligns the stored values
   with MuJoCo for correctness.
4. **PD servo via `<general>`.** `<general joint="j" gaintype="fixed"
   gainprm="100" biastype="affine" biasprm="0 -100 -10"
   dyntype="filterexact" dynprm="0.01"/>` produces identical force output
   to `<position joint="j" kp="100" kv="10" timeconst="0.01"/>`.
5. **Muscle via `<general>`.** `<general joint="j" gaintype="muscle"
   biastype="muscle" dyntype="muscle" gainprm="0.75 1.05 -1 200 0.5 1.6
   1.5 0.6 1.4" dynprm="0.01 0.04 0"/>` produces identical force output
   to the equivalent `<muscle>` shortcut with the same parameters. Note:
   `biasprm` is not specified here — it defaults to `[0,...,0]`. This is
   correct because `BiasType::Muscle` reads from `gainprm` at runtime
   (`mujoco_pipeline.rs:8367`: `let prm = &model.actuator_gainprm[i]`),
   not from `biasprm`. The `<muscle>` shortcut sets `biasprm = gainprm`
   (MuJoCo convention), but the actual `biasprm` values are unused for
   `BiasType::Muscle`.
6. **Default class inheritance.** A `<default class="d"><general
   gaintype="affine" gainprm="0 0 -10"/></default>` applied to
   `<general class="d" joint="j"/>` produces `GainType::Affine`,
   `gainprm=[0,0,-10,0,...,0]`. Explicit attributes on the element override
   class defaults.
7. **Partial gainprm.** `<general joint="j" gainprm="50"/>` produces
   `gainprm=[50,0,0,0,...,0]` (first element overridden, rest zero-padded).
8. **dynprm default.** `<general joint="j" dyntype="filter"/>` without
   explicit `dynprm` produces `dynprm=[1,0,0]` (MuJoCo default τ=1.0s).
9. **Invalid enum error.** `<general joint="j" gaintype="invalid"/>`
   produces `ModelConversionError` with message containing "unknown gaintype"
   and the valid options.
10. **User type error.** `<general joint="j" gaintype="user"/>` produces
    `ModelConversionError` with message containing "unknown gaintype 'user'".
11. **Shortcut types unaffected.** `gaintype`/`biastype`/`dyntype`
    attributes on `<motor>`, `<position>`, `<velocity>`, `<damper>`,
    `<cylinder>`, `<adhesion>`, or `<muscle>` elements are silently ignored
    by the parser (not parsed — the parsing block is gated by
    `actuator_type == General`). The shortcut expansion logic is unchanged.
12. **All dyntype values wire correctly.** `dyntype="none"` → 0 activation
    states; `dyntype="integrator"` → 1 state, `act_dot = ctrl`;
    `dyntype="filter"` → 1 state, `act_dot = (ctrl − act) / τ`;
    `dyntype="filterexact"` → 1 state (same act_dot, different integration);
    `dyntype="muscle"` → 1 state, muscle activation dynamics.
13. **Extra gainprm elements silently ignored.** `gainprm="1 2 3 4 5 6 7 8
    9 10"` (10 elements) stores only the first 9 — element at index 9 is
    dropped without error.

#### Files

- `sim/L0/mjcf/src/types.rs` — add `gaintype`, `biastype`, `dyntype`,
  `gainprm`, `biasprm`, `dynprm` fields on `MjcfActuator` + update
  `Default`; add same fields on `MjcfActuatorDefaults` + update `Default`
- `sim/L0/mjcf/src/parser.rs` — add `parse_float_array_opt()` helper;
  parse 6 new attributes in `parse_actuator_attrs()` (gated by
  `actuator_type == General`); parse same in `parse_actuator_defaults()`
- `sim/L0/mjcf/src/defaults.rs` — extend `apply_to_actuator()` with
  6 new `is_none()` default propagation blocks
- `sim/L0/mjcf/src/model_builder.rs` — add `parse_gaintype()`,
  `parse_biastype()`, `parse_dyntype()`, `floats_to_array()` helpers;
  replace General arm in dyntype determination (`model_builder.rs:1521`);
  replace General arm in gain/bias/dynprm expansion
  (`model_builder.rs:1668`)

---

## Group B — Scaling & Performance

### 9. Batched Simulation
**Status:** ✅ Complete | **Effort:** L | **Prerequisites:** None

*Transferred from [future_work_1.md](./future_work_1.md) #10.*

#### Current State

Single-environment execution. `Data::step(&mut self, &Model)`
(`mujoco_pipeline.rs:3032`) steps one simulation. `Model` is immutable after
construction, uses `Arc<TriangleMeshData>` for shared mesh data
(`mujoco_pipeline.rs:955`). `Data` is fully independent — no shared mutable
state, no interior mutability, derives `Clone` (`mujoco_pipeline.rs:1593`).

`Data::reset(&mut self, &Model)` (`mujoco_pipeline.rs:2993`) resets a subset
of fields to initial state:
- **Zeroed:** `qpos` (← `qpos0`), `qvel`, `qacc`, `qacc_warmstart`, `ctrl`,
  `act`, `act_dot`, `actuator_length`, `actuator_velocity`, `actuator_force`,
  `sensordata`, `time`, `ncon`, `contacts` (cleared).
- **NOT reset (user inputs):** `qfrc_applied`, `xfrc_applied` — these are
  user-applied forces that persist across `reset()`. Callers must zero them
  explicitly if a clean reset is needed.
- **NOT reset (intermediates):** FK outputs (`xpos`, `xquat`, etc.), force
  intermediates (`qfrc_bias`, `qfrc_passive`, `qfrc_constraint`,
  `qfrc_actuator`), mass matrix (`qM`, `qLD_*`), body/composite inertia,
  tendon/equality state, energy, solver state, scratch buffers. All of these
  are recomputed on the next `forward()` call, so stale values are harmless.
- **NOT reset (warmstart):** `efc_lambda` — the HashMap retains entries from
  the previous episode. Stale warmstarts are harmless (solver converges
  regardless, just with fewer saved iterations) and clearing is O(capacity)
  work with no correctness benefit.

#### Objective

Step N independent environments in parallel on CPU. This is a **pure physics
batching** API — it knows nothing about rewards, episodes, or RL semantics.
Higher-level RL wrappers (Gymnasium, etc.) are built on top by consumers, not
inside sim-core. Foundation for GPU acceleration (#10).

#### Architecture

**Parallelism model:** Task-level parallelism via rayon. Each `Data` runs its
full `step()` pipeline independently on a rayon worker thread. No data sharing
between environments during a step. This is embarrassingly parallel — the only
synchronization point is the rayon `join` at the end of `step_all()`.

**Memory layout:** `Vec<Data>` — each environment owns its own heap allocations
(scratch buffers, contact vectors, warmstart HashMap, etc.). This means N
environments produce N × (number of arrays in `Data`) separate heap
allocations. For typical use (N ≤ 4096, model nv ≤ 200), total memory is
dominated by the `nv × nv` mass matrix per env (~320 KB at nv=200), giving
~1.3 GB for 4096 envs. This is acceptable for CPU batching. A future
structure-of-arrays (SoA) layout across environments would improve cache
locality for bulk state extraction but adds significant complexity and is
explicitly deferred to the GPU acceleration work (#10).
Tracked in [future_work_10b.md](./future_work_10b.md) §DT-82.

**Why not SoA now:** The `Data::step()` pipeline reads and writes dozens of
fields on `Data` throughout a single step (FK, collision, RNE, constraint
solve, integration). Converting to SoA would require rewriting every pipeline
function to accept strided slices instead of `&mut Data`. The performance
gain for CPU batching is marginal (each thread operates on one env's contiguous
`Data`), while the implementation cost is enormous. SoA becomes essential only
when SIMD or GPU kernels need to process the same field across all envs
simultaneously.

#### Thread Safety Verification

`Data` derives `Clone` and contains only owned types (`DVector<f64>`,
`Vec<T>`, `HashMap<K,V>`, `f64`). All fields are `Send + Sync`. `Model`
contains `Arc<TriangleMeshData>`, `Arc<HeightFieldData>`,
`Arc<SdfCollisionData>` — all `Send + Sync`. `Data::step(&mut self, &Model)`
takes `&Model` (shared immutable) and `&mut self` (exclusive mutable).
rayon's `par_iter_mut` gives each closure an `&mut Data` — exclusive, no
aliasing.

**Known global state:** One `static WARN_ONCE: Once` exists in
`mj_fwd_constraint()` (`mujoco_pipeline.rs:10435`) for unimplemented tendon
equality constraints. `Once` is `Sync` — concurrent calls are safe (one thread
prints, others skip). No correctness impact.

**No other global/thread-local state.** Verified: no `thread_local!`, no
`lazy_static!`, no `static mut`, no `RefCell`, no `Mutex`/`RwLock` in
sim-core's non-test code. The `HashMap<WarmstartKey, Vec<f64>>` in `Data` is
per-instance (not shared), so no contention.

**Conclusion:** `Data` is trivially `Send`. rayon `par_iter_mut` over
`Vec<Data>` is sound without any code changes to the step pipeline.

#### Specification

```rust
/// Batched simulation: N independent environments sharing one Model.
///
/// All environments have identical physics (same Model), but independent
/// state (separate Data). Stepping is parallelized across CPU cores via
/// rayon when the `parallel` feature is enabled; sequential fallback
/// when disabled.
///
/// # Examples
///
/// ```
/// use sim_core::{BatchSim, Model};
/// use std::sync::Arc;
///
/// let model = Arc::new(load_model("humanoid.xml"));
/// let mut batch = BatchSim::new(model, 64);
///
/// // Set controls for each environment
/// for (i, env) in batch.envs_mut().enumerate() {
///     env.ctrl[0] = actions[i];
/// }
///
/// // Step all environments in parallel
/// let errors = batch.step_all();
///
/// // Read results
/// for (i, env) in batch.envs().enumerate() {
///     observations[i] = env.qpos.as_slice();
/// }
///
/// // Reset failed environments
/// let failed: Vec<bool> = errors.iter().map(|e| e.is_some()).collect();
/// batch.reset_where(&failed);
/// ```
pub struct BatchSim {
    model: Arc<Model>,
    envs: Vec<Data>,
}
```

**Construction:**

```rust
impl BatchSim {
    /// Create a batch of `n` environments, each initialized via
    /// `model.make_data()` (qpos = qpos0, qvel = 0, time = 0).
    pub fn new(model: Arc<Model>, n: usize) -> Self {
        let envs = (0..n).map(|_| model.make_data()).collect();
        Self { model, envs }
    }

    /// Number of environments.
    #[must_use]
    pub fn len(&self) -> usize { self.envs.len() }

    /// Whether the batch is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool { self.envs.is_empty() }

    /// Shared model reference.
    #[must_use]
    pub fn model(&self) -> &Model { &self.model }
}
```

**Environment access — direct `Data` references:**

```rust
impl BatchSim {
    /// Immutable access to environment `i`.
    ///
    /// # Errors
    ///
    /// Returns `None` if `i >= len()`.
    #[must_use]
    pub fn env(&self, i: usize) -> Option<&Data> {
        self.envs.get(i)
    }

    /// Mutable access to environment `i`.
    ///
    /// Use this to set `ctrl`, `qfrc_applied`, `xfrc_applied`, or
    /// any other input field before calling `step_all()`.
    ///
    /// # Errors
    ///
    /// Returns `None` if `i >= len()`.
    pub fn env_mut(&mut self, i: usize) -> Option<&mut Data> {
        self.envs.get_mut(i)
    }

    /// Iterator over all environments (immutable).
    pub fn envs(&self) -> impl ExactSizeIterator<Item = &Data> {
        self.envs.iter()
    }

    /// Iterator over all environments (mutable).
    ///
    /// Primary mechanism for setting per-env controls before `step_all()`:
    /// ```
    /// for (i, env) in batch.envs_mut().enumerate() {
    ///     env.ctrl.copy_from_slice(&actions[i]);
    /// }
    /// ```
    pub fn envs_mut(&mut self) -> impl ExactSizeIterator<Item = &mut Data> {
        self.envs.iter_mut()
    }
}
```

Users set `ctrl` (and any other inputs) directly on each `Data` via
`env_mut()` or `envs_mut()` before calling `step_all()`. This avoids
inventing a batched control-input API that would duplicate what `Data`
already provides. The `Data` struct's fields are public — this is
intentional and matches MuJoCo's `mjData` design.

**Stepping:**

```rust
impl BatchSim {
    /// Step all environments by one timestep.
    ///
    /// Returns per-environment errors. `None` = success, `Some(e)` = that
    /// environment's step failed. Failed environments are **not**
    /// auto-reset — the caller decides what to do (reset, skip, abort).
    ///
    /// When the `parallel` feature is enabled, environments are stepped
    /// in parallel via rayon `par_iter_mut`. When disabled, environments
    /// are stepped sequentially (identical results, useful for debugging).
    ///
    /// # Determinism
    ///
    /// Output is independent of thread count and scheduling order.
    /// Each environment's step is a pure function of its own `Data` and
    /// the shared `Model`. No cross-environment communication occurs.
    pub fn step_all(&mut self) -> Vec<Option<StepError>> {
        let model = &self.model;

        #[cfg(feature = "parallel")]
        {
            use rayon::iter::{IntoParallelRefMutIterator, ParallelIterator};
            self.envs
                .par_iter_mut()
                .map(|data| data.step(model).err())
                .collect()
        }

        #[cfg(not(feature = "parallel"))]
        {
            self.envs
                .iter_mut()
                .map(|data| data.step(model).err())
                .collect()
        }
    }
}
```

**No auto-reset.** The original spec had failed envs auto-resetting on the
next `step_all()`. This is removed: auto-reset bakes RL episode semantics
into the physics layer. The caller knows whether a `StepError` means "reset
this env" or "this model is broken, abort the whole batch." sim-core provides
the mechanism (`reset`/`reset_where`); the policy belongs to the consumer.

**Reset:**

`Data::reset()` resets the source-of-truth fields (qpos, qvel, ctrl, etc.)
but does **not** zero `qfrc_applied` or `xfrc_applied`. These are user
inputs that persist across reset — callers who need a fully clean state
must zero them explicitly. See the "Current State" section for the complete
field-by-field breakdown.

```rust
impl BatchSim {
    /// Reset environment `i` to initial state via `Data::reset(&Model)`.
    ///
    /// Does **not** zero `qfrc_applied` / `xfrc_applied` (see `Data::reset`
    /// documentation). Callers must zero these explicitly if needed.
    ///
    /// # Errors
    ///
    /// Returns `None` if `i >= len()`.
    pub fn reset(&mut self, i: usize) -> Option<()> {
        self.envs.get_mut(i)?.reset(&self.model);
        Some(())
    }

    /// Reset all environments where `mask[i]` is true.
    ///
    /// Environments where `mask[i]` is false are untouched.
    /// If `mask.len() < len()`, unaddressed environments are untouched.
    /// If `mask.len() > len()`, excess entries are ignored.
    pub fn reset_where(&mut self, mask: &[bool]) {
        let model = &self.model;
        for (i, data) in self.envs.iter_mut().enumerate() {
            if mask.get(i).copied().unwrap_or(false) {
                data.reset(model);
            }
        }
    }

    /// Reset all environments to initial state.
    pub fn reset_all(&mut self) {
        let model = &self.model;
        for data in &mut self.envs {
            data.reset(model);
        }
    }
}
```

**No parallel reset.** `Data::reset()` is O(nq + nv + nu + na) — a handful
of `fill(0.0)` calls on small vectors. For a humanoid (nv=28), reset takes
~100 ns. Even at 4096 envs, sequential reset is ~0.4 ms — negligible
compared to a single `step_all()` (~ms per env). Parallelizing reset via
rayon would add thread-pool overhead that exceeds the work itself. If this
becomes a bottleneck at much larger scales, parallelization can be added
trivially (the pattern is identical to `step_all()`).
Tracked in [future_work_10b.md](./future_work_10b.md) §DT-92.

**Design constraint — single Model:** All environments share the same
`Arc<Model>` (same `nq`, `nv`, body tree, geom set). Multi-model batching
(different robots in the same batch) would require per-env dimensions and
is explicitly out of scope.
Tracked in [future_work_10b.md](./future_work_10b.md) §DT-83.

**What this API does NOT include (and why):**

- **No `RewardFn` / rewards** — reward computation is an RL concern, not a
  physics concern. MuJoCo itself has no reward concept. Consumers compute
  rewards by reading `Data` fields after `step_all()`.
- **No `terminated` / `truncated`** — episode boundary semantics belong to
  the RL wrapper (Gymnasium, etc.), not the physics engine. The engine
  reports `StepError`; the wrapper interprets it.
- **No `BatchResult` / `states: DMatrix`** — a fixed `(n_envs, nq+nv)`
  matrix is too rigid (ignores `act`, `sensordata`, `xpos`, contacts) and
  too narrow (RL frameworks need different observation spaces). Instead,
  users read whatever fields they need directly from each `Data`. If
  contiguous matrix extraction is needed (e.g., for numpy interop via FFI),
  that belongs in a future Python binding layer, not in the core API.
- **No `set_ctrl_batch(matrix)`** — would duplicate `Data.ctrl` which is
  already public. Direct field access is simpler and more flexible (users
  can set `qfrc_applied`, `xfrc_applied`, `act` too).

**SIMD integration:** sim-simd provides within-environment acceleration
(`batch_dot_product_4()`, `batch_aabb_overlap_4()`, etc.). These accelerate
the inner loop of each environment's step. Cross-environment parallelism
comes from rayon, not SIMD. The two are orthogonal and compose naturally.

#### Acceptance Criteria

1. **Bit-exact determinism:** `BatchSim::step_all()` produces identical
   `Data` state (qpos, qvel, qacc, contacts, sensordata — all fields) to
   calling `Data::step()` on each environment sequentially. Verified by
   test: step N envs with distinct initial states both batched and
   sequential, assert `Data` fields are bitwise equal.

2. **Error isolation:** A `StepError` in environment `i` does not affect
   any other environment's state. Verified by test: inject a NaN into one
   env's `qpos`, step the batch, confirm all other envs stepped correctly.

3. **No auto-reset:** Failed environments retain their error state after
   `step_all()`. A subsequent `step_all()` without explicit `reset()` will
   produce the same (or another) error for that env. Verified by test.

4. **Reset correctness:** `reset(i)` delegates to `Data::reset(&Model)`,
   which zeros source-of-truth fields (qpos ← qpos0, qvel, qacc, ctrl,
   act, sensordata, time, contacts) but does **not** zero `qfrc_applied`,
   `xfrc_applied`, or `efc_lambda` (see "Current State" for full breakdown).
   Intermediate fields (FK outputs, forces, mass matrix) are stale after
   reset but recomputed on the next `forward()` — this is correct.
   `reset(out_of_bounds)` returns `None`. `reset_where([true, false, true])`
   resets envs 0 and 2, leaves env 1 untouched. Verified by test.

5. **Throughput scaling:** Benchmark with a non-trivial model (humanoid,
   nv ≥ 28, with contacts) at N = {1, 16, 64, 256, 1024} environments.
   Measure wall-clock time for 100 steps. With `parallel` feature:
   throughput (envs × steps / sec) scales ≥ 0.7× linearly up to the
   number of physical CPU cores. Rayon's work-stealing handles load
   imbalance (some envs have more contacts than others).

6. **Sequential fallback:** Without `parallel` feature, `step_all()`
   compiles and produces identical results. No `#[cfg]` pollution outside
   `batch.rs`.

7. **Feature-gated:** `BatchSim` is available unconditionally (it's useful
   even without parallelism for managing multiple envs). Only the rayon
   `par_iter_mut` path is gated behind `#[cfg(feature = "parallel")]`.
   When `parallel` is disabled, the sequential path is used — no rayon
   dependency pulled in.

8. **Zero `unwrap()` / `expect()`:** All indexing uses `.get()` returning
   `Option` or bounds-checked iteration. `step()` returns `Result`.
   `reset()` returns `Option<()>`. No panics in library code.

#### Implementation Notes

**`Data::reset()` semantics:** Fully documented in the "Current State"
section. Key point for implementors: `qfrc_applied` and `xfrc_applied`
persist across `Data::reset()`. The `BatchSim` API does not add its own
reset logic — it delegates directly to `Data::reset()`. If a future
`Data::reset()` change adds/removes fields from the reset set, `BatchSim`
inherits the change automatically.

**`static WARN_ONCE` in `mj_fwd_constraint()`:** The `Once` warning for
tendon equality constraints has been removed — tendon equality is now fully
implemented in the pipeline (§37, `extract_tendon_equality_jacobian()`).

**Rayon thread pool:** `step_all()` uses the global rayon thread pool
(default: one thread per logical core). Users who need custom thread counts
can configure it via `rayon::ThreadPoolBuilder::new().num_threads(n).build_global()`.
We do not expose thread pool configuration in `BatchSim` — that's rayon's
API surface, not ours.

#### Benchmark Specification

Create `benches/batch_benchmarks.rs` with:

```rust
// Model: humanoid with contacts (ground plane + feet)
// Envs: [1, 16, 64, 256, 1024]
// Steps per measurement: 100
// Metric: envs * steps / wall_seconds (throughput)
//
// Baseline: single-env sequential (BatchSim with parallel disabled)
// Compare: BatchSim with parallel enabled
//
// Report: throughput ratio vs single-env at each N
```

Use `criterion` (already a dev-dependency). The benchmark model should be
constructed programmatically (no external XML dependency in benchmarks).

#### Files

- `sim/L0/core/src/batch.rs` — new module, contains `BatchSim`
- `sim/L0/core/src/lib.rs` — add `pub mod batch;` and re-export `BatchSim`
- `sim/L0/core/Cargo.toml` — already correct (`parallel = ["dep:rayon"]`)
- `sim/L0/core/benches/batch_benchmarks.rs` — new benchmark
- Tests: inline `#[cfg(test)]` in `batch.rs` + integration test in
  `sim/L0/tests/integration/`

---

### 10. GPU Acceleration
**Status:** ✅ Phase 10a complete | **Effort:** XL | **Prerequisites:** #9

*Transferred from [future_work_1.md](./future_work_1.md) #11.*

**Phase 10a implementation:** New `sim-gpu` crate at `sim/L0/gpu/` providing
GPU-accelerated batched simulation via wgpu compute shaders. Euler velocity
integration (`qvel += qacc * h`) runs on GPU; all other pipeline stages remain
on CPU. `GpuBatchSim` is a drop-in replacement for `BatchSim` with same API
surface (step_all, reset, env access) plus transparent CPU fallback via
`try_new()`. 17 integration tests (5 always-run + 12 GPU-required), criterion
benchmarks, A-grade across all 7 STANDARDS.md criteria. Phases 10b–10f
(FK, collision, constraint solver, full GPU step) remain as future work.

#### Current State

`GpuBatchSim` (`sim-gpu/src/pipeline.rs`) wraps `BatchSim` with GPU-accelerated
Euler velocity integration. The 5-step pipeline per timestep:
1. CPU: `data.forward(&model)` for each env (rayon-parallel with `parallel` feature)
2. CPU→GPU: Upload `qvel`, `qacc` as f32 SoA buffers
3. GPU: `euler_integrate.wgsl` compute shader (`qvel += qacc * h`)
4. GPU→CPU: Staging buffer readback, scatter f32→f64 back to `Data.qvel`
5. CPU: Activation integration, position integration, time advance (via `integrate_without_velocity`)

`GpuSimContext` (`context.rs`) provides a `OnceLock<Option<>>` singleton for
device/queue initialization, following the `mesh-gpu` pattern. `GpuEnvBuffers`
(`buffers.rs`) manages Structure-of-Arrays GPU buffers with overflow-safe size
validation. The WGSL shader uses pipeline-overridable `wg_size` constants and
strided loops for models with nv > max workgroup size.

**Remaining phases (not yet implemented):**

**Data layout (GPU-relevant fields of `Data`):**

| Field | Type | Size (nv=30, nbody=15, ngeom=20) | GPU notes |
|-------|------|----------------------------------|-----------|
| `qpos` | `DVector<f64>` | nq × 8B (nq ≥ nv, typ. ~248B) | Flat f64 array |
| `qvel`, `qacc` | `DVector<f64>` | nv × 8B ≈ 240B ea. | Flat f64 array |
| `ctrl` | `DVector<f64>` | nu × 8B | Flat f64 array |
| `xpos` | `Vec<Vector3<f64>>` | nbody × 24B ≈ 360B | 3×f64, pad to 4×f64 for GPU |
| `xquat` | `Vec<UnitQuaternion>` | nbody × 32B ≈ 480B | 4×f64 |
| `xmat` | `Vec<Matrix3<f64>>` | nbody × 72B ≈ 1080B | 3×3 f64, pad rows to 4 |
| `qM` | `DMatrix<f64>` | nv² × 8B ≈ 7.2KB | Dense symmetric, largest per-env alloc |
| `contacts` | `Vec<Contact>` | variable (0–200) | Dynamic-length, GPU-hostile |
| `efc_lambda` | `HashMap<WarmstartKey, Vec<f64>>` | variable | Hash map, cannot port to GPU |

**Key obstacle:** `Data` uses nalgebra `DVector`/`DMatrix` (heap-allocated,
runtime-sized) and `HashMap` for warmstart. These cannot be uploaded to GPU
directly. The GPU backend requires a fixed-layout Structure-of-Arrays (SoA)
representation with compile-time-known offsets and maximum sizes.

**Reference implementation:** `mesh-gpu` (`mesh/mesh-gpu/`) provides wgpu
patterns for device init (`OnceLock` singleton), buffer management
(`bytemuck` `Pod + Zeroable` types with `#[repr(C)]`), compute pipelines,
and staging buffer readback. The xtask grading system explicitly allows
`wgpu` in L0 crates (`grade.rs:570`: "wgpu is NOT considered a Bevy
dependency").

#### Objective

New `sim-gpu` L0 crate providing a wgpu compute shader backend for batched
simulation. 1024+ parallel environments on a single GPU for RL training at
scale. The crate is a **drop-in accelerator** — same `BatchSim` semantics
(step, reset, env access), same physics results, but with GPU execution
when available and transparent CPU fallback when not.

**Phase 10a scope (this spec):** Only the Euler velocity integration
(`qvel += qacc * h`) runs on GPU. All other pipeline stages (FK,
collision, constraints, dynamics) remain on CPU. This validates the full
GPU infrastructure with minimal physics complexity. Subsequent phases
(10b–10f) progressively port more stages to GPU.

#### Architecture

**Crate placement:** New crate at `sim/L0/gpu/` (package name `sim-gpu`).
This is an L0 crate — no Bevy, no windowing. Dependencies: `sim-core`
(for `Model`, `Data`, `BatchSim`, `StepError`), `wgpu`, `bytemuck`,
`pollster`, `thiserror`, `tracing`, and optionally `rayon` (for parallel
forward pass, gated behind `parallel` feature). Non-optional direct deps =
6; with `parallel` = 7. Both are within the A-grade criterion 5 threshold
(≤ 7). `sim-core` itself does NOT gain a wgpu dependency — the GPU
backend is opt-in via `sim-gpu`.

**Why a separate crate (not `gpu.rs` in sim-core):**

1. `sim-core` stays L0-clean with zero GPU deps — usable in WASM, embedded,
   Python bindings without pulling in wgpu's transitive tree.
2. GPU feature complexity (shader compilation, device management, buffer
   pooling) warrants its own module boundary and error types.
3. Independent grading: `sim-gpu` can be graded separately with GPU-aware
   dep thresholds while `sim-core` maintains its current A grade.
4. Follows established pattern: `mesh` (core) + `mesh-gpu` (accelerator).

**Execution model: environment-per-thread → environment-per-workgroup.**

On CPU (rayon), each environment runs its full `step()` pipeline on one OS
thread. On GPU, each environment maps to one **workgroup**. Within a
workgroup, individual threads cooperate on parallelizable sub-stages (e.g.,
FK traversal per body, collision per geom pair). The total dispatch is:

```
dispatch_workgroups(num_envs, 1, 1)
// Within shader: @workgroup_size(wg_size, 1, 1)  — wg_size is an override constant
```

The workgroup size is set at pipeline creation via WGSL `override` constants
to `min(next_power_of_2(nv), limits.max_compute_workgroup_size_x)` for
Phase 10a (or `max(nbody, ngeom, nv)` for later phases). For a model with
nv=30, `wg_size = 32`. For nv=300 on a GPU with max workgroup size 256,
`wg_size = 256` and each thread processes multiple DOFs via a strided
loop in the shader (standard GPU pattern). Each env uses
`workgroup_id.x` as its environment index. This maps naturally to GPU
hardware: independent envs → independent
workgroups → can execute on different SMs/CUs without synchronization.

#### GPU Data Layout (SoA)

The GPU backend uses a **Structure-of-Arrays** (SoA) layout with fixed
maximum sizes. All N environments' data is packed into contiguous GPU
buffers with known strides.

**Phase 10a buffers** (implemented in this spec):

Only `qvel` and `qacc` are uploaded/downloaded in Phase 10a:

```
Buffer: gpu_qvel    — layout: [env0_qvel[0..nv], env1_qvel[0..nv], ...]
Buffer: gpu_qacc    — layout: [env0_qacc[0..nv], env1_qacc[0..nv], ...]
```

All values are **f32** on GPU. The CPU↔GPU boundary performs f64↔f32
conversion. GPU f64 throughput is 1/32nd of f32 on consumer GPUs. For RL
training at scale, f32 physics is standard (MuJoCo MJX, Isaac Gym, Brax
all use f32). CPU path remains f64 for offline validation and precision.

Model parameters (`nv`, `num_envs`, `timestep`) are uploaded once via the
`GpuParams` uniform buffer (see section 4 below).

---

**Future-phase buffer layout (Phase 10b+ — NOT implemented in this spec):**

<details>
<summary>Click to expand future data layout design</summary>

**GpuModelHeader** (uniform buffer, uploaded once, read-only):

```rust
/// GPU-side model constants (Phase 10b+ target).
/// All fields are f32. Layout follows WebGPU alignment rules.
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct GpuModelHeader {
    nq: u32,              // offset  0
    nv: u32,              // offset  4
    nbody: u32,           // offset  8
    njnt: u32,            // offset 12
    ngeom: u32,           // offset 16
    nu: u32,              // offset 20
    timestep: f32,        // offset 24
    _pad: u32,            // offset 28 (align gravity to 32 = 16*2)
    gravity: [f32; 4],    // offset 32, .w = padding
    // Total: 48 bytes (multiple of 16) ✓
}
```

Model arrays (body tree, joint params, geom params) use storage buffers
(exceed 65KB uniform limit for non-trivial models).

**Additional per-env SoA buffers:**

```
Buffer: gpu_qpos    — layout: [env0_qpos[0..nq], env1_qpos[0..nq], ...]
Buffer: gpu_ctrl    — layout: [env0_ctrl[0..nu], env1_ctrl[0..nu], ...]
Buffer: gpu_xpos    — layout: [env0_xpos[0..nbody*4], env1_xpos[0..nbody*4], ...]
Buffer: gpu_xquat   — layout: [env0_xquat[0..nbody*4], env1_xquat[0..nbody*4], ...]
Buffer: gpu_qM      — layout: [env0_qM[0..nv*nv], env1_qM[0..nv*nv], ...]
Buffer: gpu_qfrc_*  — layout: [env0_qfrc[0..nv], env1_qfrc[0..nv], ...]
```

**Contact buffer** (dynamic-length, pre-allocated to max):

Pre-allocate `MAX_CONTACTS_PER_ENV` (default 128) per env. Atomic counter
per env tracks actual count. Contacts exceeding cap dropped
(least-penetrating first).

```rust
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct GpuContact {
    pos: [f32; 4],        // 16B: contact point, .w = padding
    normal: [f32; 4],     // 16B: contact normal, .w = penetration depth
    friction: [f32; 4],   // 16B: [mu_slide, mu_torsion, mu_roll, condim]
    geom_ids: [u32; 2],   //  8B: geom pair
    body_ids: [u32; 2],   //  8B: body pair
    solref: [f32; 2],     //  8B: solver reference params
    solimp: [f32; 2],     //  8B: solver impedance (truncated)
}
// Size: 80 bytes, 16-byte aligned
```

**Warmstart:** Frame-coherent warmstart buffer (flat
`[f32; MAX_CONTACTS * MAX_CONDIM]` per env) replaces the CPU
`HashMap<WarmstartKey, Vec<f64>>`. Contact-to-warmstart correspondence
uses contact buffer index from previous frame.

</details>

#### Pipeline Stages — GPU Porting Strategy

The `Data::step()` pipeline has 15+ stages with sequential dependencies.
Porting strategy: **start with the simplest stages, validate correctness,
then progressively port more complex stages.** Each stage becomes a
separate compute shader (or shader entry point) dispatched sequentially.
Workgroup barriers (`workgroupBarrier()`) synchronize within an env;
separate dispatches synchronize across stages.

**Phase 10a — Integration-only GPU (this spec):**

Port only the **integration step** to GPU. This is the simplest stage
(pure arithmetic on flat arrays) and validates the entire GPU infrastructure
(device init, buffer layout, upload/download, shader dispatch, correctness
testing) without touching the hard stages (FK tree traversal, collision,
constraint solver).

Pipeline for Phase 10a:
1. **CPU:** `forward()` — all 15 stages run on CPU via existing code.
2. **GPU upload:** `qvel`, `qacc` → GPU buffers (per-env SoA).
3. **GPU dispatch:** `euler_integrate` shader: `qvel += qacc * h` for
   each DOF in each env. One workgroup per env, one thread per DOF.
4. **GPU download:** Updated `qvel` → CPU `Data.qvel`.
5. **CPU:** activation integration, `mj_integrate_pos()`,
   `mj_normalize_quat()`, time advance (via `integrate_without_velocity`).

This deliberately leaves position integration on CPU because quaternion
integration (`mj_integrate_pos`) requires joint-type dispatch and
quaternion normalization — straightforward but more complex than the
velocity update. It can be ported in a follow-up.

**Shader for Phase 10a:**

```wgsl
// euler_integrate.wgsl
struct Params {
    num_envs: u32,
    nv: u32,
    timestep: f32,
    _pad: u32,
}

// Pipeline-overridable constant: set to next_power_of_2(nv) at
// pipeline creation via wgpu's `constants` map. Default 64 covers
// models with nv ≤ 64. WebGPU guarantees max workgroup size ≥ 256.
override wg_size: u32 = 64;

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read_write> qvel: array<f32>;
@group(0) @binding(2) var<storage, read> qacc: array<f32>;

@compute @workgroup_size(wg_size, 1, 1)
fn euler_integrate(
    @builtin(workgroup_id) wg_id: vec3<u32>,
    @builtin(local_invocation_id) local_id: vec3<u32>,
) {
    let env_idx = wg_id.x;
    if env_idx >= params.num_envs { return; }

    // Strided loop: each thread processes DOFs at stride wg_size.
    // Handles nv > wg_size (e.g., nv=300 with wg_size=256).
    var dof_idx = local_id.x;
    let base = env_idx * params.nv;
    loop {
        if dof_idx >= params.nv { break; }
        let offset = base + dof_idx;
        qvel[offset] = qvel[offset] + qacc[offset] * params.timestep;
        dof_idx += wg_size;
    }
}
```

**Why integration-only first:**
1. Validates the full GPU stack (wgpu init, buffer management, shader
   compilation, dispatch, readback) with minimal physics complexity.
2. The integration shader is trivially verifiable (one multiply-add per DOF,
   strided loop for large nv).
3. Establishes the SoA buffer layout, upload/download patterns, and
   `GpuBatchSim` API that all subsequent stages will use.
4. Even this minimal GPU path provides measurable throughput data for
   planning subsequent phases.

**Future phases (not in this spec, listed for roadmap context):**

- **10b:** FK (forward kinematics) — tree traversal shader, requires
  topological sort of body tree into GPU-friendly level-set parallel
  traversal. One thread per body per level, `workgroupBarrier()` between
  levels.
- **10c:** Collision broad-phase — AABB sweep-and-prune or spatial hashing
  on GPU. Write candidate pairs to intermediate buffer.
- **10d:** Collision narrow-phase — per-pair distance/contact computation.
  Sphere-sphere and capsule-capsule first, then box, then mesh (hardest).
- **10e:** Constraint solver — Jacobi-style parallel PGS (unlike CPU's
  Gauss-Seidel PGS which is inherently sequential). Convergence properties
  differ; may require more iterations.
- **10f:** Full GPU step — all stages on GPU, only `ctrl` upload and
  `qpos`/`qvel`/`sensordata` download cross the PCIe bus.

#### Specification

##### 1. Create `sim-gpu` crate

**File:** `sim/L0/gpu/Cargo.toml`

```toml
[package]
name = "sim-gpu"
description = "GPU-accelerated batched physics simulation via wgpu compute shaders"
version.workspace = true
edition.workspace = true
license.workspace = true
repository.workspace = true
authors.workspace = true
rust-version.workspace = true

[dependencies]
sim-core = { path = "../core", features = ["gpu-internals"] }
wgpu = "24"
bytemuck = { version = "1.14", features = ["derive"] }
pollster = "0.4"
thiserror = { workspace = true }
tracing = { workspace = true }
rayon = { workspace = true, optional = true }

[dev-dependencies]
approx = { workspace = true }
criterion = { workspace = true }

[[bench]]
name = "gpu_benchmarks"
harness = false

[features]
default = []
parallel = ["dep:rayon", "sim-core/parallel"]

[lints]
workspace = true
```

**File:** `sim/L0/gpu/src/lib.rs`

```rust
#![deny(clippy::unwrap_used, clippy::expect_used)]
#![warn(missing_docs)]

//! GPU-accelerated batched physics simulation via wgpu compute shaders.
//!
//! This is a Layer 0 crate — no Bevy, no windowing, no rendering.
//! Provides [`GpuBatchSim`] as a GPU-accelerated drop-in for
//! [`sim_core::batch::BatchSim`].
//!
//! # Usage
//!
//! ```ignore
//! use sim_core::Model;
//! use sim_gpu::GpuBatchSim;
//! use std::sync::Arc;
//!
//! let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
//! let batch = GpuBatchSim::new(model, 1024)?;
//! // ... set controls, step, read results — same API as BatchSim
//! ```

pub mod context;
pub mod error;
pub mod buffers;
pub mod pipeline;

pub use context::GpuSimContext;
pub use error::{GpuSimError, GpuSimResult};
pub use pipeline::GpuBatchSim;
// Re-export buffer types for advanced usage (matches mesh-gpu pattern)
pub use buffers::{GpuEnvBuffers, GpuParams};
```

##### 2. GPU context (`context.rs`)

Reuses the `mesh-gpu` pattern: `OnceLock<Option<GpuSimContext>>` singleton.

```rust
/// GPU device context for simulation compute shaders.
///
/// Initialized lazily on first access via [`GpuSimContext::get()`].
/// Returns `None` if no compatible GPU is available.
///
/// Separate from `mesh_gpu::GpuContext` because simulation may request
/// different device features/limits in the future (e.g., f64 support
/// for validation mode).
pub struct GpuSimContext {
    device: wgpu::Device,
    queue: wgpu::Queue,
    limits: wgpu::Limits,
    /// Adapter info (GPU name, vendor, backend) for logging/debugging.
    /// Matches `mesh-gpu::GpuContext` pattern.
    adapter_info: wgpu::AdapterInfo,
}

impl GpuSimContext {
    /// Returns a reference to the singleton context, or `None` if no
    /// compatible GPU is available. Initializes on first call.
    pub fn get() -> Option<&'static Self> { ... }

    /// Convenience: `Self::get().is_some()`.
    pub fn is_available() -> bool { Self::get().is_some() }

    /// Adapter information (GPU name, vendor, device type, backend).
    /// Useful for logging which GPU is being used in RL training runs.
    #[must_use]
    pub fn adapter_info(&self) -> &wgpu::AdapterInfo { &self.adapter_info }
}
```

**Why separate from `mesh-gpu::GpuContext`:** The two contexts share the
same GPU but may diverge in requested features. Coupling them would force
`sim-gpu` to depend on `mesh-gpu` (unnecessary dep) or vice versa. Each
context is a thin wrapper around wgpu device init — the duplication is
< 50 lines and avoids an artificial dependency.

**Thread safety:** `GpuBatchSim` is `Send + Sync` on native wgpu backends
(Vulkan, Metal, DX12) because wgpu's `Device`, `Queue`, `Buffer`,
`ComputePipeline`, and `BindGroup` are all `Send + Sync` on native. On
the wgpu WebGL2 backend they are `!Send`, but WebGL2 is not a target for
batch RL training. The spec does not add manual `Send`/`Sync` impls —
the auto-derived bounds from wgpu types are correct. If RL training code
needs to move `GpuBatchSim` across threads, it works on native backends
(the primary target).

**Model immutability invariant:** `GpuBatchSim` captures model parameters
(`nv`, `timestep`, `integrator`) at construction time into the GPU
`params_buffer` and pipeline configuration. The `Model` is stored as
`Arc<Model>` inside the inner `BatchSim`, which provides only `&Model`
access (no mutation through `Arc` without sole ownership via
`Arc::get_mut`). This is safe: no code path can mutate the model while
`GpuBatchSim` holds a clone of the `Arc`. If model parameters need to
change (e.g., varying timestep for curriculum learning), the
`GpuBatchSim` must be recreated with a new `Model`.

##### 3. Error types (`error.rs`)

```rust
/// Errors from GPU-accelerated simulation.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum GpuSimError {
    /// No compatible GPU available.
    #[error("no compatible GPU available for simulation")]
    NotAvailable,

    /// Buffer exceeds `limits.max_storage_buffer_binding_size`.
    ///
    /// wgpu does not expose a "free GPU memory" API. Actual GPU OOM
    /// surfaces as [`DeviceLost`](GpuSimError::DeviceLost). This error
    /// catches the checkable limit: individual buffer size vs. the
    /// device's max storage buffer binding size.
    #[error("batch of {num_envs} envs needs {required}B buffer, limit: {limit}B")]
    BatchTooLarge {
        num_envs: usize,
        required: u64,
        limit: u64,
    },

    /// GPU device lost during computation.
    #[error("GPU device lost during simulation step")]
    DeviceLost,

    /// Shader compilation failure.
    #[error("shader compilation failed: {0}")]
    ShaderCompilation(String),

    /// Buffer readback failure.
    #[error("GPU buffer readback failed: {0}")]
    BufferReadback(String),

    /// Unsupported integrator (GPU path supports Euler only in Phase 10a).
    #[error("unsupported integrator for GPU: {0:?} (only Euler is supported)")]
    UnsupportedIntegrator(sim_core::Integrator),
}

/// Result type alias for GPU simulation operations.
pub type GpuSimResult<T> = Result<T, GpuSimError>;
```

##### 4. Buffer management (`buffers.rs`)

```rust
/// Shader parameters matching the WGSL `Params` struct layout.
///
/// Uploaded once at construction to a uniform buffer.
/// Must exactly match the WGSL struct field order and alignment:
/// ```wgsl
/// struct Params { num_envs: u32, nv: u32, timestep: f32, _pad: u32 }
/// ```
#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GpuParams {
    /// Environment count (capped at `u32::MAX` by the WGSL shader's u32
    /// indexing). `GpuBatchSim::new()` truncates `n: usize` to `u32`.
    pub num_envs: u32,
    pub nv: u32,
    pub timestep: f32,
    _pad: u32,
}

impl GpuParams {
    /// Create params from a model and env count.
    #[must_use]
    pub fn from_model(model: &Model, num_envs: u32) -> Self {
        Self {
            num_envs,
            nv: model.nv as u32,
            timestep: model.timestep as f32,
            _pad: 0,
        }
    }
}
```

The `params_buffer` on `GpuBatchSim` is created once in `new()` via
`device.create_buffer_init`:

```rust
use wgpu::util::DeviceExt;

let params = GpuParams::from_model(&model, n as u32);
let params_buffer = ctx.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
    label: Some("sim_params"),
    contents: bytemuck::cast_slice(&[params]),
    usage: wgpu::BufferUsages::UNIFORM,
});
```

```rust
/// SoA GPU buffers for N environments sharing one Model.
///
/// Allocates contiguous storage buffers with stride = per-env size.
/// All values are f32 on GPU.
pub struct GpuEnvBuffers {
    /// qvel storage: N × nv f32 values.
    /// Flags: STORAGE (shader access) | COPY_SRC (→ staging for readback) |
    ///        COPY_DST (← queue.write_buffer for upload).
    pub qvel: wgpu::Buffer,
    /// qacc storage: N × nv f32 values (read-only in integration shader).
    /// Flags: STORAGE (shader access) | COPY_DST (← queue.write_buffer for upload).
    pub qacc: wgpu::Buffer,
    /// Staging buffer for GPU→CPU download (MAP_READ | COPY_DST).
    /// Only qvel is downloaded in Phase 10a.
    pub download_staging: wgpu::Buffer,
    /// Number of environments.
    pub num_envs: u32,
    /// Degrees of freedom per env.
    pub nv: u32,
}

impl GpuEnvBuffers {
    /// Allocate SoA buffers for `num_envs` environments with `nv` DOFs each.
    ///
    /// Checks each buffer size against `limits.max_storage_buffer_binding_size`.
    /// Returns `Err(GpuSimError::BatchTooLarge)` if exceeded.
    ///
    /// When `num_envs == 0` or `nv == 0`, allocates minimum-size (4 byte)
    /// buffers to satisfy wgpu's `size > 0` requirement.
    pub fn new(ctx: &GpuSimContext, num_envs: u32, nv: u32) -> GpuSimResult<Self> {
        let limit = ctx.limits.max_storage_buffer_binding_size as u64;
        // Overflow-safe size computation. checked_mul catches u64
        // overflow; the subsequent limit check catches over-size.
        let raw_size = (num_envs as u64)
            .checked_mul(nv as u64)
            .and_then(|x| x.checked_mul(std::mem::size_of::<f32>() as u64))
            .ok_or(GpuSimError::BatchTooLarge {
                num_envs: num_envs as usize,
                required: u64::MAX, // overflowed — true size exceeds u64
                limit,
            })?;
        if raw_size > limit {
            return Err(GpuSimError::BatchTooLarge {
                num_envs: num_envs as usize,
                required: raw_size, // actual computed size for diagnostics
                limit,
            });
        }
        let buf_size = raw_size.max(4); // wgpu requires size > 0
        let qvel = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("sim_qvel"),
            size: buf_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_SRC | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let qacc = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("sim_qacc"),
            size: buf_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let download_staging = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("sim_staging"),
            size: buf_size,
            usage: BufferUsages::MAP_READ | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        Ok(Self { qvel, qacc, download_staging, num_envs, nv })
    }
}
```

**Upload path:** Gather f64 `qvel`/`qacc` from each `Data`, convert to f32,
pack into contiguous `Vec<f32>`, then
`queue.write_buffer(&self.qvel, 0, bytemuck::cast_slice(&data))` and
`queue.write_buffer(&self.qacc, 0, bytemuck::cast_slice(&data))`.
`write_buffer` takes `&[u8]`, so `bytemuck::cast_slice` converts `&[f32]`
→ `&[u8]` (same pattern as `mesh-gpu`'s `upload_values`). `write_buffer`
handles the host→device transfer internally — no explicit staging buffer
needed for uploads (wgpu manages this).

**Download path:** `copy_buffer_to_buffer(qvel, 0, download_staging, 0, size)`
via a command encoder, then `submit`. Then
`download_staging.slice(..).map_async(wgpu::MapMode::Read)`,
`device.poll(wgpu::Maintain::Wait)`, `bytemuck::cast_slice` the mapped
range to `&[f32]`, scatter f32 back to f64 `Data.qvel` per environment,
unmap.

##### 5. Compute pipeline (`pipeline.rs`)

```rust
/// GPU-accelerated batch simulation.
///
/// Drop-in replacement for [`sim_core::batch::BatchSim`] that offloads
/// the integration step to GPU compute shaders. All other pipeline stages
/// (FK, collision, constraints, dynamics) run on CPU.
///
/// # Phase 10a Scope
///
/// In this initial phase, only the Euler velocity integration
/// (`qvel += qacc * h`) runs on GPU. This validates the full GPU stack
/// while keeping physics complexity minimal. The CPU `BatchSim` is used
/// internally for the `forward()` pass, and only the integration step
/// is replaced with a GPU dispatch.
pub struct GpuBatchSim {
    /// Underlying CPU batch (owns Model + Data instances).
    cpu_batch: BatchSim,
    /// Per-env SoA buffers on GPU.
    buffers: GpuEnvBuffers,
    /// Compiled integration compute pipeline.
    integrate_pipeline: wgpu::ComputePipeline,
    /// Bind group layout for the integration shader.
    integrate_bind_group_layout: wgpu::BindGroupLayout,
    /// Bind group binding params, qvel, and qacc to the integration shader.
    /// Created once in `new()` (buffers are fixed at construction).
    integrate_bind_group: wgpu::BindGroup,
    /// Uniform buffer for shader parameters (num_envs, nv, timestep).
    /// Created once in `new()`, never mutated (model-constant).
    params_buffer: wgpu::Buffer,
}

impl GpuBatchSim {
    /// Create a GPU-accelerated batch of `n` environments.
    ///
    /// **Validation order** (checked in this order; first failure wins):
    /// 1. Returns `Err(GpuSimError::UnsupportedIntegrator)` if
    ///    `model.integrator` is not `Euler`.
    /// 2. Returns `Err(GpuSimError::NotAvailable)` if no GPU is available.
    /// 3. Returns `Err(GpuSimError::BatchTooLarge)` if a buffer exceeds
    ///    `limits.max_storage_buffer_binding_size`.
    ///
    /// Integrator checking is first so that
    /// `gpu_unsupported_integrator` and other config-validation tests
    /// can run without a GPU.
    ///
    /// **Edge cases:** `n == 0` and `model.nv == 0` are accepted without
    /// error. For `n == 0`, the inner `BatchSim` is empty and `step_all()`
    /// returns `Ok(vec![])` immediately (no GPU dispatch). For `nv == 0`,
    /// the GPU dispatch is a no-op (shader exits immediately). In both
    /// cases, GPU buffers are allocated with minimum size 4 bytes
    /// (wgpu requires `size > 0`) to avoid validation errors. The 4-byte
    /// buffers are never meaningfully read or written.
    ///
    /// The `Model` is uploaded to GPU once. Per-env buffers are allocated
    /// for the maximum batch size; they are not resized.
    ///
    /// The GPU context is obtained internally via `GpuSimContext::get()`.
    /// The pipeline, buffers, and bind group layout are stored on the
    /// struct; the context itself is NOT stored — methods that need the
    /// device/queue call `GpuSimContext::get()` each time (same pattern
    /// as `mesh-gpu::SdfPipeline`).
    ///
    /// The shader is loaded at compile time via `include_str!` and
    /// compiled into a `ShaderModule` in `new()`:
    ///
    /// ```ignore
    /// const INTEGRATE_SHADER: &str = include_str!("shaders/euler_integrate.wgsl");
    /// let shader_module = ctx.device.create_shader_module(wgpu::ShaderModuleDescriptor {
    ///     label: Some("euler_integrate"),
    ///     source: wgpu::ShaderSource::Wgsl(INTEGRATE_SHADER.into()),
    /// });
    /// ```
    ///
    /// Pipeline creation sets the WGSL `override wg_size` constant to
    /// `min(next_power_of_2(nv), max_workgroup_size)` via
    /// `PipelineCompilationOptions`:
    ///
    /// ```ignore
    /// let max_wg = ctx.limits.max_compute_workgroup_size_x as usize;
    /// let wg_size = model.nv.next_power_of_two().min(max_wg) as f64;
    /// let constants = HashMap::from([("wg_size".to_string(), wg_size)]);
    /// let pipeline = ctx.device.create_compute_pipeline(
    ///     &wgpu::ComputePipelineDescriptor {
    ///         label: Some("euler_integrate"),
    ///         layout: Some(&pipeline_layout),
    ///         module: &shader_module,
    ///         entry_point: Some("euler_integrate"),
    ///         compilation_options: wgpu::PipelineCompilationOptions {
    ///             constants: &constants,
    ///             ..Default::default()
    ///         },
    ///         cache: None,
    ///     },
    /// );
    /// ```
    ///
    /// The bind group layout matches the WGSL `@group(0)` bindings:
    ///
    /// ```ignore
    /// let bind_group_layout = ctx.device.create_bind_group_layout(
    ///     &wgpu::BindGroupLayoutDescriptor {
    ///         label: Some("euler_integrate_bgl"),
    ///         entries: &[
    ///             wgpu::BindGroupLayoutEntry {
    ///                 binding: 0,
    ///                 visibility: wgpu::ShaderStages::COMPUTE,
    ///                 ty: wgpu::BindingType::Buffer {
    ///                     ty: wgpu::BufferBindingType::Uniform,
    ///                     has_dynamic_offset: false,
    ///                     min_binding_size: None,
    ///                 },
    ///                 count: None,
    ///             },
    ///             wgpu::BindGroupLayoutEntry {
    ///                 binding: 1,
    ///                 visibility: wgpu::ShaderStages::COMPUTE,
    ///                 ty: wgpu::BindingType::Buffer {
    ///                     ty: wgpu::BufferBindingType::Storage { read_only: false },
    ///                     has_dynamic_offset: false,
    ///                     min_binding_size: None,
    ///                 },
    ///                 count: None,
    ///             },
    ///             wgpu::BindGroupLayoutEntry {
    ///                 binding: 2,
    ///                 visibility: wgpu::ShaderStages::COMPUTE,
    ///                 ty: wgpu::BindingType::Buffer {
    ///                     ty: wgpu::BufferBindingType::Storage { read_only: true },
    ///                     has_dynamic_offset: false,
    ///                     min_binding_size: None,
    ///                 },
    ///                 count: None,
    ///             },
    ///         ],
    ///     },
    /// );
    /// ```
    ///
    /// The bind group is also created once in `new()` (buffers are fixed
    /// at construction) and stored on the struct:
    ///
    /// ```ignore
    /// let bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
    ///     label: Some("euler_integrate_bg"),
    ///     layout: &bind_group_layout,
    ///     entries: &[
    ///         wgpu::BindGroupEntry { binding: 0, resource: params_buffer.as_entire_binding() },
    ///         wgpu::BindGroupEntry { binding: 1, resource: buffers.qvel.as_entire_binding() },
    ///         wgpu::BindGroupEntry { binding: 2, resource: buffers.qacc.as_entire_binding() },
    ///     ],
    /// });
    /// ```
    pub fn new(model: Arc<Model>, n: usize) -> GpuSimResult<Self> { ... }

    /// Create a GPU-accelerated batch, returning `None` only when no
    /// GPU is available (`NotAvailable`). Other errors
    /// (`UnsupportedIntegrator`, `BatchTooLarge`, etc.) are propagated
    /// so callers learn about fixable configuration problems rather
    /// than silently falling back to CPU.
    ///
    /// ```ignore
    /// let model = Arc::new(model);
    /// match GpuBatchSim::try_new(Arc::clone(&model), 1024)? {
    ///     Some(gpu) => { /* use gpu.step_all()? */ },
    ///     None      => { /* no GPU — fall back to BatchSim */ },
    /// }
    /// ```
    ///
    /// Implementation: calls `new()`, maps `NotAvailable` → `Ok(None)`,
    /// re-wraps success as `Ok(Some(self))`, propagates all other errors.
    pub fn try_new(model: Arc<Model>, n: usize) -> GpuSimResult<Option<Self>> {
        match Self::new(model, n) {
            Ok(sim) => Ok(Some(sim)),
            Err(GpuSimError::NotAvailable) => Ok(None),
            Err(e) => Err(e),
        }
    }

    /// Number of environments.
    #[must_use]
    pub fn len(&self) -> usize { ... }

    /// Whether the batch is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool { ... }

    /// Shared model reference.
    #[must_use]
    pub fn model(&self) -> &Model { ... }

    /// Immutable access to environment `i`.
    #[must_use]
    pub fn env(&self, i: usize) -> Option<&Data> { ... }

    /// Mutable access to environment `i` (for setting controls).
    pub fn env_mut(&mut self, i: usize) -> Option<&mut Data> { ... }

    /// Iterator over all environments (immutable).
    #[must_use]
    pub fn envs(&self) -> impl ExactSizeIterator<Item = &Data> { ... }

    /// Iterator over all environments (mutable).
    pub fn envs_mut(&mut self) -> impl ExactSizeIterator<Item = &mut Data> { ... }

    /// Step all environments by one timestep.
    ///
    /// # Phase 10a Pipeline
    ///
    /// 1. CPU: `data.forward(model)` for each env (rayon-parallel).
    /// 2. CPU→GPU: Upload `qvel`, `qacc` for all envs.
    /// 3. GPU: `euler_integrate` shader: `qvel += qacc * h`.
    /// 4. GPU→CPU: Copy qvel to staging + download (single encoder submit).
    /// 5. CPU: activation integration, position integration, time advance.
    ///
    /// # Errors
    ///
    /// Returns `Err(GpuSimError)` if the GPU dispatch itself fails
    /// (device lost, buffer mapping error). On success, returns
    /// per-environment errors with the same semantics as
    /// [`BatchSim::step_all()`].
    ///
    /// # Failure Semantics
    ///
    /// If `step_all()` returns `Err`, the environments are in a
    /// partially-updated state: `forward()` has already run (mutating
    /// internal FK/force/contact data) but integration has not completed.
    /// The `Data.qvel` values are stale. Callers should treat a
    /// `GpuSimError` as unrecoverable for the current batch state and
    /// call `reset_all()` before continuing, or recreate the
    /// `GpuBatchSim`. Do not retry `step_all()` without resetting.
    pub fn step_all(&mut self) -> GpuSimResult<Vec<Option<StepError>>> { ... }

    /// Reset environment `i` to initial state.
    pub fn reset(&mut self, i: usize) -> Option<()> { ... }

    /// Reset environments where `mask[i]` is true.
    pub fn reset_where(&mut self, mask: &[bool]) { ... }

    /// Reset all environments.
    pub fn reset_all(&mut self) { ... }

    /// Create a CPU-only `BatchSim` with cloned state from this GPU batch.
    ///
    /// Returns a new `BatchSim` sharing the same `Arc<Model>` (via
    /// `Arc::clone`) with deep-cloned `Data` instances reflecting the
    /// current environment state. Useful for correctness validation:
    /// step both from identical state and compare results.
    #[must_use]
    pub fn cpu_reference(&self) -> BatchSim { ... }

    // --- Private helpers (not part of public API) ---

    /// Upload qvel + qacc from all CPU envs to GPU buffers (f64→f32).
    fn upload_integration_inputs(&self, ctx: &GpuSimContext) -> GpuSimResult<()> { ... }

    /// Dispatch euler_integrate shader and download updated qvel (f32→f64).
    ///
    /// Records compute pass + copy_buffer_to_buffer in a single command
    /// encoder, submits once, then maps staging buffer for readback and
    /// scatters results to per-env `Data.qvel`.
    fn dispatch_and_download(&mut self, ctx: &GpuSimContext) -> GpuSimResult<()> { ... }
}
```

**`step_all()` implementation detail (Phase 10a):**

```rust
pub fn step_all(&mut self) -> GpuSimResult<Vec<Option<StepError>>> {
    // Obtain GPU context once at the top; pass &GpuSimContext to helpers.
    // Avoids repeated Option unwrapping and matches mesh-gpu's
    // SdfPipeline::compute(&self, ctx: &GpuContext, ...) pattern.
    let ctx = GpuSimContext::get().ok_or(GpuSimError::NotAvailable)?;
    let model = Arc::clone(self.cpu_batch.model_arc());

    // 1. CPU forward pass (parallel across envs).
    //    Calls data.forward(model) on each env. Uses rayon when the
    //    `parallel` feature is enabled on sim-core, sequential otherwise.
    //    BatchSim has no forward-only method, so we iterate envs_mut()
    //    directly. This is the same pattern BatchSim::step_all() uses
    //    internally, minus the integrate() call.
    let errors: Vec<Option<StepError>> = {
        #[cfg(feature = "parallel")]
        {
            use rayon::iter::{IntoParallelRefMutIterator, ParallelIterator};
            self.cpu_batch.envs_as_mut_slice()
                .par_iter_mut()
                .map(|data| data.forward(&model).err())
                .collect()
        }
        #[cfg(not(feature = "parallel"))]
        {
            self.cpu_batch.envs_mut()
                .map(|data| data.forward(&model).err())
                .collect()
        }
    };

    // 2. Upload qvel + qacc from ALL envs to GPU (f64→f32).
    //    Failed envs are uploaded too — their qvel will be integrated
    //    on GPU but the result is harmless because step 5 skips their
    //    position integration. Uploading all avoids sparse gather and
    //    keeps the GPU dispatch uniform.
    self.upload_integration_inputs(ctx)?;

    // 3–4. GPU dispatch + readback copy in a single command encoder.
    //       The compute dispatch and copy_buffer_to_buffer (qvel → staging)
    //       are recorded into one encoder and submitted once. This avoids
    //       a redundant submit/sync boundary between dispatch and copy
    //       (same pattern as mesh-gpu's boolean pipeline).
    //       After submit: staging.slice(..).map_async(MapMode::Read),
    //       device.poll(wgpu::Maintain::Wait), then scatter:
    //         let nv = self.buffers.nv as usize;
    //         let mapped = staging.slice(..).get_mapped_range();
    //         let gpu_qvel: &[f32] = bytemuck::cast_slice(&mapped);
    //         for i in 0..self.buffers.num_envs as usize {
    //             if let Some(data) = self.cpu_batch.env_mut(i) {
    //                 let base = i * nv;
    //                 for j in 0..nv {
    //                     data.qvel[j] = gpu_qvel[base + j] as f64;
    //                 }
    //             }
    //         }
    //         drop(mapped);
    //         staging.unmap();
    self.dispatch_and_download(ctx)?;

    // 5. CPU: activation + position integration + time advance
    //    (only for envs whose forward() succeeded)
    for (i, err) in errors.iter().enumerate() {
        if err.is_none() {
            if let Some(data) = self.cpu_batch.env_mut(i) {
                data.integrate_without_velocity(&model);
            }
        }
    }

    Ok(errors)
}
```

**Note on `envs_as_mut_slice` and `model_arc`:** The forward pass requires
parallel mutable access to the env `Vec<Data>` and shared access to the
`Arc<Model>`. `BatchSim` currently exposes `envs_mut()` (iterator) and
`model()` (borrow), but for rayon `par_iter_mut()` we need a `&mut [Data]`
slice. This requires two minor additions to `BatchSim`, also behind
`#[cfg(feature = "gpu-internals")]`:

```rust
// In batch.rs, add to `impl BatchSim`:

/// Direct mutable slice access to environments.
///
/// Used by GPU backend for rayon par_iter_mut() over envs.
#[cfg(feature = "gpu-internals")]
#[doc(hidden)]
pub fn envs_as_mut_slice(&mut self) -> &mut [Data] {
    &mut self.envs
}

/// Clone of the shared model Arc.
///
/// Used by GPU backend to pass model to parallel forward() calls.
#[cfg(feature = "gpu-internals")]
#[doc(hidden)]
#[must_use]
pub fn model_arc(&self) -> &Arc<Model> {
    &self.model
}
```

**Note on `integrate_without_velocity`:** The GPU backend performs velocity
integration (`qvel += qacc * h`) on GPU. The remaining parts of the CPU
`integrate()` method are:
1. **Activation integration** (`act += h * act_dot`, muscle clamping)
2. **Position integration** (`mj_integrate_pos` + `mj_normalize_quat`)
3. **Time advance** (`time += h`)

All three must still run on CPU. This requires a new method on `Data`
that does everything `integrate()` does EXCEPT the velocity update:

```rust
// In mujoco_pipeline.rs, add to `impl Data` block:

/// Integration step without velocity update.
///
/// Performs activation integration, position integration, quaternion
/// normalization, and time advance. Skips `qvel += qacc * h` (assumed
/// to have been done externally, e.g., on GPU).
///
/// Used by the GPU backend (`sim-gpu`) where velocity integration
/// is performed on GPU via compute shader.
///
/// # Visibility
///
/// This method is public but feature-gated behind `gpu-internals`.
/// It is not part of the stable `sim-core` API — only `sim-gpu`
/// should depend on this feature.
#[cfg(feature = "gpu-internals")]
#[doc(hidden)]
pub fn integrate_without_velocity(&mut self, model: &Model) {
    // Guard: this method assumes velocity was updated externally (GPU Euler).
    // If a future phase relaxes the Euler-only check in GpuBatchSim::new(),
    // this assert will catch misuse before silent physics errors.
    debug_assert!(
        matches!(model.integrator, Integrator::Euler),
        "integrate_without_velocity only valid for Euler integrator, got {:?}",
        model.integrator
    );
    let h = model.timestep;

    // 1. Activation integration (identical to integrate())
    for i in 0..model.nu {
        let act_adr = model.actuator_act_adr[i];
        let act_num = model.actuator_act_num[i];
        for k in 0..act_num {
            let j = act_adr + k;
            match model.actuator_dyntype[i] {
                ActuatorDynamics::FilterExact => {
                    let tau = model.actuator_dynprm[i][0].max(1e-10);
                    self.act[j] += self.act_dot[j] * tau * (1.0 - (-h / tau).exp());
                }
                _ => {
                    self.act[j] += h * self.act_dot[j];
                }
            }
        }
        if model.actuator_dyntype[i] == ActuatorDynamics::Muscle {
            for k in 0..act_num {
                self.act[act_adr + k] = self.act[act_adr + k].clamp(0.0, 1.0);
            }
        }
    }

    // 2. Skip velocity integration (done on GPU)

    // 3. Position integration + quaternion normalization + time advance
    mj_integrate_pos(model, self, h);
    mj_normalize_quat(model, self);
    self.time += h;
}
```

The method lives inside `mujoco_pipeline.rs` (same module as the private
`mj_integrate_pos()`, `mj_normalize_quat()`, and `ActuatorDynamics`), so
it can call them directly.

`sim-gpu`'s `Cargo.toml` depends on:
`sim-core = { path = "../core", features = ["gpu-internals"] }`.

This pattern works because:
- `integrate_without_velocity` is `pub fn` → visible to `sim-gpu`
  (external crate).
- It lives in `mujoco_pipeline.rs` → can call private helpers (same
  module).
- `#[cfg(feature = "gpu-internals")]` → not compiled or visible unless
  the feature is explicitly enabled.
- `#[doc(hidden)]` → excluded from `cargo doc` output.

##### 6. f64 ↔ f32 conversion and precision

**Upload (f64 → f32):** Straightforward `as f32` cast. Values outside f32
range are clamped (this should never happen for well-behaved simulations
where joint positions and velocities are bounded).

**Download (f32 → f64):** `as f64` cast. The f32 result is exact in f64.

**Precision impact:** The GPU integration `qvel += qacc * h` in f32 introduces
~7 decimal digits of precision vs. CPU's ~15 digits. For a single Euler step
with `h = 0.002` and `qacc = 100.0`, the f32 error is:

```
qvel_cpu = qvel + 100.0 * 0.002  (f64)
qvel_gpu = qvel + 100.0 * 0.002  (f32, then cast back to f64)
|error| ≤ ε_f32 * |qvel_gpu| ≈ 1.2e-7 * |qvel|
```

Over 1000 steps, error accumulates to ~O(sqrt(1000) × 1.2e-7) ≈ 4e-6 for
random-walk accumulation, or O(1000 × 1.2e-7) ≈ 1.2e-4 worst case.

This is well within RL training tolerance. For reference, MuJoCo MJX
(Google DeepMind's GPU backend) operates entirely in f32 and is the
standard tool for GPU-accelerated RL physics.

#### Scope Exclusions

1. **Full GPU pipeline** — Only integration moves to GPU in Phase 10a.
   FK, collision, constraints, and dynamics remain on CPU. This is
   intentional: validate the infrastructure before porting complex stages.

2. **f64 GPU mode** — No f64 compute shader variant. GPU f64 is 1/32nd
   throughput on consumer GPUs. If bit-exact CPU parity is needed, use
   `BatchSim` (CPU). The GPU path is for throughput, not precision.

3. **Multi-model batching** — All envs share one `Model`, same as
   `BatchSim`. Multi-model GPU batching (different robots in same dispatch)
   is architecturally harder (varying nv/nbody/ngeom per env) and is a
   separate future item.

4. **CUDA/Vulkan compute backend** — wgpu only. Cross-platform by default.
   wgpu selects the best backend per platform (Vulkan on Linux, Metal on
   macOS, DX12 on Windows). A raw CUDA backend could provide higher
   throughput on NVIDIA hardware but is not cross-platform.

5. **Dynamic batch resizing** — Buffer sizes are fixed at construction.
   To change batch size, create a new `GpuBatchSim`.

6. **Activation/muscle integration on GPU** — Activation state integration
   (`act += h * act_dot`, muscle clamping) stays on CPU in Phase 10a.
   These are small arrays (length `na`, typically < 20) and not worth
   the upload/download overhead in isolation.

7. **Non-Euler GPU integration** — Phase 10a supports Euler integrator
   only. `GpuBatchSim::new()` returns
   `Err(GpuSimError::UnsupportedIntegrator)` for both `RungeKutta4` and
   `ImplicitSpringDamper`. RK4 requires 4 forward passes per step,
   multiplying CPU↔GPU transfer cost. ImplicitSpringDamper modifies
   velocity inside `mj_fwd_acceleration_implicit()`, not in the
   integration step. Both deferred to Phase 10e+.

#### Acceptance Criteria

1. **Approximate numerical equivalence:** For the same model and initial
   conditions, GPU-batched simulation results match CPU-batched results
   within f32 precision tolerance. Error accumulates over N steps
   (each step introduces fresh rounding and feeds diverged state forward),
   so the tolerance is step-count-dependent:
   `|qvel_gpu - qvel_cpu| ≤ N * ε_f32 * |qvel_cpu| + N * ε_abs`
   where `ε_f32 = 1.2e-7` (f32 machine epsilon ≈ 2⁻²³), `ε_abs = 1e-10`
   (absolute tolerance for near-zero values), and N = number of steps.
   For the single-step test (N=1): tolerance ≈ 1.2e-7 relative.
   For the 100-step test (N=100): tolerance ≈ 1.2e-5 relative.
   Tested with the `n_link_pendulum(3, 1.0, 0.1)` model.
   *Note on chaotic dynamics:* The triple pendulum is chaotic — tiny
   perturbations grow exponentially (Lyapunov exponent λ > 0). The linear
   bound `N * ε_f32` is conservative enough to cover this at N=100 (0.4s
   simulation time), but for longer rollouts (N >> 1000) or more chaotic
   models, divergence may exceed the linear bound even with a correct GPU
   implementation. If AC1 fails at high N, verify the growth rate is
   consistent with expected Lyapunov behavior rather than a GPU bug.

2. **GPU throughput baseline (Phase 10a) / exceeds CPU (Phase 10b+):**
   In Phase 10a, the benchmark measures `GpuBatchSim::step_all()` vs
   `BatchSim::step_all()` wall-clock time for 1024+ environments on
   hardware with a discrete GPU. Measured with
   `n_link_pendulum(3, 1.0, 0.1)`, 10 steps, best of 5 runs.
   GPU is expected to be **slower** in Phase 10a (only integration runs
   on GPU; the CPU forward pass dominates). The benchmark establishes a
   baseline for tracking improvement as more stages move to GPU.
   This criterion becomes a hard pass/fail gate in Phase 10b+ when FK
   moves to GPU.

3. **Graceful CPU fallback:** `GpuBatchSim::try_new()` returns `Ok(None)`
   when no GPU is available (other errors propagate via `Err`).
   `GpuBatchSim::new()` returns `Err(GpuSimError::NotAvailable)`.
   No panic, no silent degradation. Tested via always-run fallback test.

4. **Determinism:** Same model + same initial state + same controls →
   identical GPU results across runs. No non-determinism from thread
   scheduling (unlike CPU rayon which is deterministic but order-dependent
   on rayon thread pool).

5. **Error isolation:** A `StepError` in one environment does not affect
   other environments' results. All envs are uploaded to GPU uniformly
   (including failed ones — this avoids sparse gather overhead). Failed
   envs' velocity is integrated on GPU but their position integration
   and time advance are skipped on CPU, leaving them in an error state
   that matches the CPU `BatchSim` behavior.

6. **API parity with `BatchSim`:** `GpuBatchSim` exposes the same public
   methods as `BatchSim` (`len`, `is_empty`, `model`, `env`, `env_mut`,
   `envs`, `envs_mut`, `step_all`, `reset`, `reset_where`, `reset_all`)
   plus `try_new` and `cpu_reference`. The only signature difference:
   `step_all` returns
   `GpuSimResult<Vec<Option<StepError>>>` (wrapping GPU errors) vs.
   `BatchSim`'s `Vec<Option<StepError>>`. Callers add one `?` to switch.

7. **Zero `unwrap`/`expect` in library code.** All GPU operations return
   `Result` or `Option`. Test code may use `unwrap`.

8. **Bevy-free:** `cargo tree -p sim-gpu | grep -E "(bevy|winit)"` returns
   empty.

9. **Documentation:** Every public item has a doc comment. Crate-level
   docs explain the GPU/CPU tradeoff and Phase 10a scope.

10. **Clippy clean:** Zero warnings with `-D warnings`.

**AC → Test traceability:**

| AC | Verified by | Mechanism |
|----|-------------|-----------|
| 1  | `gpu_matches_cpu_single_step`, `gpu_matches_cpu_100_steps` | f64↔f32 tolerance assertion per DOF |
| 2  | `gpu_benchmarks.rs` (criterion) | Wall-clock comparison; baseline only in 10a |
| 3  | `try_new_fallback_consistency` | Always-run; no GPU needed |
| 4  | `gpu_determinism` | 5 identical runs, bitwise `assert_eq` |
| 5  | `gpu_error_isolation` | NaN injection in one env, `is_finite` on others |
| 6  | `gpu_euler_ok`, `gpu_step_single_env`, `gpu_reset_*` | API surface coverage |
| 7  | `#![deny(clippy::unwrap_used)]` + `cargo xtask grade` | Compile-time enforcement |
| 8  | `cargo tree -p sim-gpu \| grep bevy` | CI check |
| 9  | `RUSTDOCFLAGS="-D warnings" cargo doc -p sim-gpu` | CI check |
| 10 | `cargo clippy -p sim-gpu -- -D warnings` | CI check |

Additional edge-case tests (not AC-mapped):

| Test | What it covers |
|------|---------------|
| `gpu_context_availability` | `GpuSimContext::is_available()` returns bool without panic |
| `gpu_params_layout` | `GpuParams` size/align matches WGSL `Params` struct |
| `gpu_params_from_model` | f64→f32 conversion of model constants |
| `gpu_step_empty_batch` | 0-env edge case: no GPU dispatch, empty result |
| `gpu_step_nv_zero` | 0-DOF model: GPU dispatch is a no-op |
| `gpu_batch_too_large` | Overflow-safe buffer validation rejects impossible sizes |
| `gpu_unsupported_integrator` | Config validation before GPU init (no GPU needed) |

#### Benchmark

**File:** `sim/L0/gpu/benches/gpu_benchmarks.rs`

Using `criterion`:

```rust
// Benchmark: BatchSim (CPU) vs GpuBatchSim (GPU) step throughput
// Model: n_link_pendulum(3, 1.0, 0.1) — 4 bodies (world+3), 3 hinge joints, nv=3
// Env counts: 64, 256, 1024, 4096
// Measurement: 10 consecutive step_all() calls per iteration
// Groups: "cpu/{n}" vs "gpu/{n}"
```

This benchmark establishes the Phase 10a baseline. Expected result: GPU is
**slower** than CPU for Phase 10a (because forward pass dominates and still
runs on CPU). The benchmark's value is tracking GPU overhead and
establishing the infrastructure for Phase 10b+ comparisons.

#### Tests

**File:** `sim/L0/gpu/tests/integration.rs`

All tests use the `n_link_pendulum(3, 1.0, 0.1)` model (nv=3, Euler integrator)
unless stated otherwise.

```rust
// ── Always-run tests (no GPU required) ────────────────────────────

#[test]
fn gpu_context_availability() {
    // GpuSimContext::is_available() returns bool without panic.
    let _ = GpuSimContext::is_available();
}

#[test]
fn try_new_fallback_consistency() {
    // try_new() returns Ok(Some) iff is_available(), Ok(None) otherwise.
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    let result = GpuBatchSim::try_new(model, 4);
    match result {
        Ok(Some(_)) => assert!(GpuSimContext::is_available()),
        Ok(None)    => assert!(!GpuSimContext::is_available()),
        Err(e)      => panic!("try_new returned unexpected error: {e}"),
    }
}

#[test]
fn gpu_params_layout() {
    assert_eq!(std::mem::size_of::<GpuParams>(), 16);
    assert_eq!(std::mem::align_of::<GpuParams>(), 4);
}

#[test]
fn gpu_params_from_model() {
    let model = Model::n_link_pendulum(3, 1.0, 0.1);
    // model.timestep = 1.0/240.0 ≈ 0.004167 for n_link_pendulum
    let params = GpuParams::from_model(&model, 64);
    assert_eq!(params.num_envs, 64);
    assert_eq!(params.nv, 3);
    // f64→f32 conversion: verify round-trip accuracy at this scale
    let expected_ts = (1.0_f64 / 240.0) as f32;
    assert!((params.timestep - expected_ts).abs() < f32::EPSILON);
}

#[test]
fn gpu_unsupported_integrator() {
    // Checked before GPU availability — must work without a GPU.
    let mut model = Model::n_link_pendulum(3, 1.0, 0.1);

    model.integrator = Integrator::RungeKutta4;
    let err = GpuBatchSim::new(Arc::new(model.clone()), 4).unwrap_err();
    assert!(matches!(err, GpuSimError::UnsupportedIntegrator(Integrator::RungeKutta4)));

    model.integrator = Integrator::ImplicitSpringDamper;
    let err = GpuBatchSim::new(Arc::new(model), 4).unwrap_err();
    assert!(matches!(err, GpuSimError::UnsupportedIntegrator(Integrator::ImplicitSpringDamper)));
}

// ── GPU-required tests ────────────────────────────────────────────
// Ignored in CI without GPU. Run locally with: cargo test -- --ignored

#[test] #[ignore = "Requires GPU"]
fn gpu_euler_ok() {
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    let batch = GpuBatchSim::new(model, 4);
    assert!(batch.is_ok());
    assert_eq!(batch.unwrap().len(), 4);
}

#[test] #[ignore = "Requires GPU"]
fn gpu_step_empty_batch() {
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    let mut batch = GpuBatchSim::new(model, 0).unwrap();
    let result = batch.step_all().unwrap();
    assert!(result.is_empty());
}

#[test] #[ignore = "Requires GPU"]
fn gpu_step_nv_zero() {
    // Model with nv=0: GPU dispatch is a no-op, no crash.
    let model = Arc::new(Model::empty());
    assert_eq!(model.nv, 0);
    let mut batch = GpuBatchSim::new(model, 4).unwrap();
    let result = batch.step_all().unwrap();
    assert_eq!(result.len(), 4);
}

#[test] #[ignore = "Requires GPU"]
fn gpu_matches_cpu_single_step() {
    // AC1: GPU matches CPU within f32 tolerance after 1 step.
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    let mut gpu_batch = GpuBatchSim::new(Arc::clone(&model), 4).unwrap();
    let mut cpu_batch = gpu_batch.cpu_reference();

    gpu_batch.step_all().unwrap();
    cpu_batch.step_all();

    for i in 0..4 {
        let gpu_data = gpu_batch.env(i).unwrap();
        let cpu_data = cpu_batch.env(i).unwrap();
        for j in 0..model.nv {
            let abs_err = (gpu_data.qvel[j] - cpu_data.qvel[j]).abs();
            let rel_bound = 1.2e-7 * cpu_data.qvel[j].abs() + 1e-10;
            assert!(abs_err <= rel_bound,
                "env {i} qvel[{j}]: gpu={}, cpu={}, err={abs_err}, bound={rel_bound}",
                gpu_data.qvel[j], cpu_data.qvel[j]);
        }
    }
}

#[test] #[ignore = "Requires GPU"]
fn gpu_matches_cpu_100_steps() {
    // AC1: tolerance scales linearly with step count (N=100).
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    let mut gpu_batch = GpuBatchSim::new(Arc::clone(&model), 4).unwrap();
    let mut cpu_batch = gpu_batch.cpu_reference();

    for _ in 0..100 {
        gpu_batch.step_all().unwrap();
        cpu_batch.step_all();
    }

    let n = 100.0_f64;
    for i in 0..4 {
        let gpu_data = gpu_batch.env(i).unwrap();
        let cpu_data = cpu_batch.env(i).unwrap();
        for j in 0..model.nv {
            let abs_err = (gpu_data.qvel[j] - cpu_data.qvel[j]).abs();
            let rel_bound = n * 1.2e-7 * cpu_data.qvel[j].abs() + n * 1e-10;
            assert!(abs_err <= rel_bound,
                "env {i} qvel[{j}] after 100 steps: err={abs_err}, bound={rel_bound}");
        }
    }
}

#[test] #[ignore = "Requires GPU"]
fn gpu_determinism() {
    // AC4: 5 identical runs produce bitwise-identical qvel.
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    let mut reference: Option<Vec<Vec<f64>>> = None;

    for run in 0..5 {
        let mut batch = GpuBatchSim::new(Arc::clone(&model), 4).unwrap();
        for _ in 0..10 { batch.step_all().unwrap(); }

        let qvels: Vec<Vec<f64>> = (0..4)
            .map(|i| batch.env(i).unwrap().qvel.as_slice().to_vec())
            .collect();

        if let Some(ref expected) = reference {
            assert_eq!(&qvels, expected, "Run {run} diverged from run 0");
        } else {
            reference = Some(qvels);
        }
    }
}

#[test] #[ignore = "Requires GPU"]
fn gpu_error_isolation() {
    // AC5: NaN in one env doesn't corrupt others.
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    let mut batch = GpuBatchSim::new(Arc::clone(&model), 4).unwrap();

    // Inject NaN into env 1's qpos to trigger StepError in forward()
    batch.env_mut(1).unwrap().qpos[0] = f64::NAN;

    let errors = batch.step_all().unwrap();
    assert!(errors[1].is_some(), "env 1 should report error");

    // Env 0, 2, 3 must have valid (non-NaN) qvel
    for i in [0, 2, 3] {
        let data = batch.env(i).unwrap();
        for j in 0..model.nv {
            assert!(data.qvel[j].is_finite(),
                "env {i} qvel[{j}] is not finite after error in env 1");
        }
    }
}

#[test] #[ignore = "Requires GPU"]
fn gpu_reset_single() {
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    let mut batch = GpuBatchSim::new(Arc::clone(&model), 4).unwrap();
    for _ in 0..10 { batch.step_all().unwrap(); }

    batch.reset(2);
    let data = batch.env(2).unwrap();
    // After reset: qvel = 0, time = 0
    for j in 0..model.nv { assert_eq!(data.qvel[j], 0.0); }
    assert_eq!(data.time, 0.0);
}

#[test] #[ignore = "Requires GPU"]
fn gpu_reset_where() {
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    let mut batch = GpuBatchSim::new(Arc::clone(&model), 4).unwrap();
    for _ in 0..10 { batch.step_all().unwrap(); }

    let pre_step_time = batch.env(0).unwrap().time;
    batch.reset_where(&[false, true, false, true]);

    // Env 1 and 3 reset, env 0 and 2 unchanged
    assert_eq!(batch.env(1).unwrap().time, 0.0);
    assert_eq!(batch.env(3).unwrap().time, 0.0);
    assert_eq!(batch.env(0).unwrap().time, pre_step_time);
    assert_eq!(batch.env(2).unwrap().time, pre_step_time);
}

#[test] #[ignore = "Requires GPU"]
fn gpu_reset_all() {
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    let mut batch = GpuBatchSim::new(Arc::clone(&model), 4).unwrap();
    for _ in 0..10 { batch.step_all().unwrap(); }

    batch.reset_all();
    for i in 0..4 {
        let data = batch.env(i).unwrap();
        assert_eq!(data.time, 0.0);
        for j in 0..model.nv { assert_eq!(data.qvel[j], 0.0); }
    }
}

#[test] #[ignore = "Requires GPU"]
fn gpu_step_single_env() {
    // Single-env batch: no off-by-one in buffer offsets.
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    let mut gpu = GpuBatchSim::new(Arc::clone(&model), 1).unwrap();
    let mut cpu = gpu.cpu_reference();

    gpu.step_all().unwrap();
    cpu.step_all();

    let gpu_qvel = gpu.env(0).unwrap().qvel.as_slice();
    let cpu_qvel = cpu.env(0).unwrap().qvel.as_slice();
    for j in 0..model.nv {
        let err = (gpu_qvel[j] - cpu_qvel[j]).abs();
        assert!(err <= 1.2e-7 * cpu_qvel[j].abs() + 1e-10,
            "qvel[{j}]: gpu={}, cpu={}, err={err}", gpu_qvel[j], cpu_qvel[j]);
    }
}

#[test] #[ignore = "Requires GPU"]
fn gpu_batch_too_large() {
    // Construct a batch that exceeds max_storage_buffer_binding_size.
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    // u32::MAX envs × 3 DOFs × 4 bytes = ~51 GB — exceeds any real GPU limit.
    let err = GpuBatchSim::new(model, u32::MAX as usize).unwrap_err();
    assert!(matches!(err, GpuSimError::BatchTooLarge { .. }));
}
```

#### Implementation Notes

**Recommended implementation order:**

1. **`sim-core` changes first** — add `gpu-internals` feature, `integrate_without_velocity`,
   `envs_as_mut_slice`, `model_arc`. Verify existing `sim-core` tests still pass
   (`cargo test -p sim-core` and `cargo test -p sim-core --features gpu-internals`).
2. **`sim-gpu` crate scaffold** — `Cargo.toml`, `lib.rs`, `error.rs`. Verify `cargo check -p sim-gpu`.
3. **`context.rs`** — `GpuSimContext` singleton. Test: `gpu_context_availability`.
4. **`buffers.rs`** — `GpuParams`, `GpuEnvBuffers`. Tests: `gpu_params_layout`,
   `gpu_params_from_model`.
5. **`euler_integrate.wgsl`** — shader file. No standalone test (validated via pipeline).
6. **`pipeline.rs`** — `GpuBatchSim` with `new`, `try_new`, `step_all`, reset methods.
   Tests: all GPU-required tests.
7. **Benchmarks** — `gpu_benchmarks.rs`. Run last after correctness is established.

**Known gotchas:**

- **`model.nv` is `usize`, not `u32`:** The `as u32` cast in `GpuParams::from_model`
  truncates silently if `nv > u32::MAX`. This is safe in practice (no model has 4B+ DOFs)
  but the implementor should verify `nv` fits in `u32` before casting, or document the
  assumption.

- **`wgpu::PipelineCompilationOptions::constants` map values are `f64`:** The wgpu API
  uses `f64` for all override constant values, even for WGSL `u32` overrides. The
  `wg_size` value must be cast to `f64` before insertion into the constants map.

- **Staging buffer mapping is blocking:** `device.poll(wgpu::Maintain::Wait)` blocks the
  calling thread until the GPU finishes. For Phase 10a this is acceptable (the CPU forward
  pass dominates latency). In later phases, consider double-buffering or async polling.

- **`BatchSim` internal field names:** `envs_as_mut_slice` and `model_arc` access private
  fields of `BatchSim` (`envs: Vec<Data>`, `model: Arc<Model>`). If `BatchSim`'s internal
  field names change, the `#[cfg(feature = "gpu-internals")]` methods must be updated.
  The `#[doc(hidden)]` attribute signals these are unstable internals.

- **wgpu `create_shader_module` deprecation:** wgpu 24 may deprecate `create_shader_module`
  in favor of `create_shader_module_trusted`. Check the wgpu 24 changelog at implementation
  time. If deprecated, use the non-deprecated variant — the shader source is trusted
  (`include_str!` from our crate, not user-supplied).

#### Files

**New files (sim-gpu crate):**
- `sim/L0/gpu/Cargo.toml`
- `sim/L0/gpu/src/lib.rs` — crate root, re-exports
- `sim/L0/gpu/src/context.rs` — `GpuSimContext` (wgpu device init)
- `sim/L0/gpu/src/error.rs` — `GpuSimError`, `GpuSimResult`
- `sim/L0/gpu/src/buffers.rs` — `GpuEnvBuffers` (SoA buffer management)
- `sim/L0/gpu/src/pipeline.rs` — `GpuBatchSim` (main API)
- `sim/L0/gpu/src/shaders/euler_integrate.wgsl` — integration compute shader
- `sim/L0/gpu/tests/integration.rs` — correctness + fallback tests
- `sim/L0/gpu/benches/gpu_benchmarks.rs` — CPU vs GPU throughput comparison

**Modified files (sim-core):**
- `sim/L0/core/Cargo.toml` — add `gpu-internals` feature (empty, just a gate):
  ```toml
  # In [features]:
  gpu-internals = []  # Exposes internal helpers for sim-gpu. No deps.
  ```
- `sim/L0/core/src/mujoco_pipeline.rs` — add `integrate_without_velocity()`
  method on `Data`, behind `#[cfg(feature = "gpu-internals")]`
- `sim/L0/core/src/batch.rs` — add `envs_as_mut_slice()` and `model_arc()`
  on `BatchSim`, behind `#[cfg(feature = "gpu-internals")]`

**Modified files (workspace):**
- `Cargo.toml` (workspace root) — add `"sim/L0/gpu"` to `[workspace.members]`
  and `sim-gpu = { path = "sim/L0/gpu" }` to `[workspace.dependencies]`
