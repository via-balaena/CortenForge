# Mesh Collision Support Specification

> **Status**: ✅ **MOSTLY IMPLEMENTED** — Core infrastructure complete (PR #39).
> - ✅ Model fields (`mesh_data`, `geom_mesh`, `nmesh`)
> - ✅ Embedded vertex/face mesh loading from MJCF
> - ✅ Mesh-primitive collision dispatch (`collide_with_mesh()`)
> - ✅ 47 mesh collision tests passing
> - ❌ **NOT IMPLEMENTED**: File-based mesh loading (STL, OBJ)
>
> **Todorov Standard**: Single source of truth. No duplication. O(n) where possible.
> Compute once, use everywhere. Profile before optimizing.
>
> **Last Updated**: 2026-01-28

---

## Executive Summary

This spec defines the implementation of mesh collision support for the MuJoCo-aligned
physics pipeline. The goal is to enable triangle mesh geometries (`GeomType::Mesh`) to
participate in collision detection and response.

**Key Insight**: Most infrastructure already exists. The implementation is primarily
about *wiring* existing components together through the Model/Data architecture.

---

## Current State Analysis

### What Exists (DO NOT REIMPLEMENT)

| Component | Location | Status |
|-----------|----------|--------|
| `TriangleMeshData` | `sim-core/src/mesh.rs` | ✅ Complete with BVH |
| `mesh_mesh_contact()` | `sim-core/src/mesh.rs:1249` | ✅ Dual-BVH traversal |
| `mesh_sphere_contact()` | `sim-core/src/mesh.rs:1043` | ✅ BVH + triangle tests |
| `mesh_capsule_contact()` | `sim-core/src/mesh.rs:1092` | ✅ BVH + triangle tests |
| `mesh_box_contact()` | `sim-core/src/mesh.rs:1150` | ✅ SAT-based |
| `Bvh` | `mesh-boolean/src/bvh.rs` | ✅ Parallel construction |
| `MjcfMesh` | `sim-mjcf/src/types.rs:528` | ✅ Asset parsing |
| `CollisionShape::TriangleMesh` | `sim-core/src/collision_shape.rs` | ✅ Exists |
| GJK/EPA | `sim-core/src/gjk_epa.rs` | ✅ For convex hulls |

### What's Missing (IMPLEMENT THIS)

| Component | Location | Work Required | Status |
|-----------|----------|---------------|--------|
| `Model.nmesh` | `mujoco_pipeline.rs:5734` | Add field | ✅ Done |
| `Model.mesh_data` | `mujoco_pipeline.rs:5739` | Add `Vec<Arc<TriangleMeshData>>` | ✅ Done |
| `Model.geom_mesh` | `mujoco_pipeline.rs:5730` | Add `Vec<Option<usize>>` | ✅ Done |
| `geom_to_collision_shape()` | `mujoco_pipeline.rs:7250` | Return `TriangleMesh` for mesh geoms | ✅ Done |
| Mesh-primitive collision dispatch | `mujoco_pipeline.rs:7534` | Add cases for mesh geom type | ✅ Done |
| Model builder mesh loading | `model_builder.rs:1129` | Convert `MjcfMesh` to `TriangleMeshData` | ✅ Done |
| File-based mesh loading | `model_builder.rs:1157` | Load STL/OBJ files | ❌ TODO |

---

## Data Model Changes

### Model Struct Additions

```rust
// In Model struct (mujoco_pipeline.rs)

// ==================== Meshes (indexed by mesh_id) ====================
/// Number of mesh assets.
pub nmesh: usize,
/// Mesh names (for lookup by geom).
pub mesh_name: Vec<String>,
/// Triangle mesh data with prebuilt BVH.
/// Arc for cheap cloning when building CollisionShape.
pub mesh_data: Vec<Arc<TriangleMeshData>>,

// ==================== Geom-Mesh Mapping ====================
/// Mesh index for each geom (None if not a mesh geom).
/// Length: ngeom. Only set for GeomType::Mesh geoms.
pub geom_mesh: Vec<Option<usize>>,
```

### Todorov Principle Applied

- `mesh_data` uses `Arc<TriangleMeshData>` because:
  1. Multiple geoms can reference the same mesh asset (instancing)
  2. `CollisionShape::TriangleMesh` already uses `Arc`
  3. BVH construction is expensive - do it once at model load

- `geom_mesh` is `Option<usize>` (not a separate array) because:
  1. Most geoms are primitives (sphere, box, capsule)
  2. Sparse representation is memory-efficient
  3. Single lookup path: `model.geom_mesh[geom_id]`

---

## Implementation Plan

### Phase 1: Model/Data Infrastructure

**Files**: `mujoco_pipeline.rs`, `model_builder.rs`

#### Step 1.1: Add Model Fields

```rust
// mujoco_pipeline.rs - Model struct

// Add after ngeom
pub nmesh: usize,

// Add after geom_name
pub mesh_name: Vec<String>,
pub mesh_data: Vec<Arc<TriangleMeshData>>,
pub geom_mesh: Vec<Option<usize>>,
```

#### Step 1.2: Initialize in Model::empty()

```rust
// mujoco_pipeline.rs - Model::empty()

nmesh: 0,
mesh_name: vec![],
mesh_data: vec![],
geom_mesh: vec![],
```

#### Step 1.3: Update GeomType::bounding_radius()

The current implementation returns a conservative estimate:
```rust
Self::Mesh => {
    let scale = size.x.max(size.y).max(size.z);
    if scale > 0.0 { scale * 10.0 } else { 10.0 }
}
```

This should use the actual mesh AABB. Two options:

**Option A (Recommended)**: Compute at model load time and store in `geom_rbound`
- No change to `bounding_radius()` method
- Model builder computes from mesh AABB after loading

**Option B**: Pass mesh data to method
- Requires API change, breaks Todorov's "single computation" principle

We use Option A.

---

### Phase 2: MJCF Loading

**File**: `model_builder.rs`

#### Step 2.1: Process Mesh Assets

```rust
// In model_from_mjcf(), before processing bodies

// Build mesh lookup table
let mut mesh_name_to_id: HashMap<String, usize> = HashMap::new();
let mut mesh_data_vec: Vec<Arc<TriangleMeshData>> = Vec::new();
let mut mesh_name_vec: Vec<String> = Vec::new();

for (idx, mjcf_mesh) in mjcf.meshes.iter().enumerate() {
    mesh_name_to_id.insert(mjcf_mesh.name.clone(), idx);
    mesh_name_vec.push(mjcf_mesh.name.clone());

    let mesh_data = convert_mjcf_mesh_to_triangle_data(mjcf_mesh)?;
    mesh_data_vec.push(Arc::new(mesh_data));
}
```

#### Step 2.2: Convert MjcfMesh to TriangleMeshData

```rust
fn convert_mjcf_mesh_to_triangle_data(
    mjcf_mesh: &MjcfMesh,
) -> Result<TriangleMeshData, ModelConversionError> {
    // Get vertices and faces from MjcfMesh
    let vertices: Vec<Point3<f64>> = if let Some(ref verts) = mjcf_mesh.vertex {
        // Embedded vertex data: flat [x0, y0, z0, x1, y1, z1, ...]
        verts.chunks(3)
            .map(|chunk| Point3::new(
                chunk[0] * mjcf_mesh.scale.x,
                chunk[1] * mjcf_mesh.scale.y,
                chunk[2] * mjcf_mesh.scale.z,
            ))
            .collect()
    } else if let Some(ref file) = mjcf_mesh.file {
        // Load from file (STL, OBJ, etc.)
        load_mesh_file(file, mjcf_mesh.scale)?
    } else {
        return Err(ModelConversionError {
            message: format!("Mesh '{}' has no vertex data or file", mjcf_mesh.name),
        });
    };

    let faces: Vec<[usize; 3]> = if let Some(ref face_data) = mjcf_mesh.face {
        face_data.chunks(3)
            .map(|chunk| [chunk[0] as usize, chunk[1] as usize, chunk[2] as usize])
            .collect()
    } else {
        return Err(ModelConversionError {
            message: format!("Mesh '{}' has no face data", mjcf_mesh.name),
        });
    };

    // TriangleMeshData::new() builds the BVH automatically
    Ok(TriangleMeshData::new(vertices, faces))
}
```

#### Step 2.3: Link Geoms to Meshes

```rust
// In process_geom(), when geom type is Mesh

if geom.geom_type == MjcfGeomType::Mesh {
    if let Some(ref mesh_name) = geom.mesh {
        if let Some(&mesh_id) = mesh_name_to_id.get(mesh_name) {
            builder.geom_mesh.push(Some(mesh_id));

            // Compute bounding radius from actual mesh AABB
            let mesh = &mesh_data_vec[mesh_id];
            let half_extents = (mesh.aabb_max - mesh.aabb_min) / 2.0;
            let radius = half_extents.coords.norm();
            // Store in geom_rbound during build() phase
        } else {
            return Err(ModelConversionError {
                message: format!("Geom references unknown mesh '{}'", mesh_name),
            });
        }
    } else {
        return Err(ModelConversionError {
            message: "Mesh geom must have 'mesh' attribute".to_string(),
        });
    }
} else {
    builder.geom_mesh.push(None);
}
```

---

### Phase 3: Collision Detection

**File**: `mujoco_pipeline.rs`

#### Step 3.1: Update geom_to_collision_shape()

```rust
fn geom_to_collision_shape(
    model: &Model,
    geom_id: usize,
) -> Option<CollisionShape> {
    let geom_type = model.geom_type[geom_id];
    let size = model.geom_size[geom_id];

    match geom_type {
        GeomType::Sphere => Some(CollisionShape::Sphere { radius: size.x }),
        GeomType::Box => Some(CollisionShape::Box { half_extents: size }),
        GeomType::Capsule => Some(CollisionShape::Capsule {
            half_length: size.y,
            radius: size.x,
        }),
        GeomType::Cylinder => Some(CollisionShape::Cylinder {
            half_length: size.y,
            radius: size.x,
        }),
        GeomType::Ellipsoid => Some(CollisionShape::Ellipsoid { radii: size }),
        GeomType::Plane => None, // Handled specially
        GeomType::Mesh => {
            // Get mesh data from model
            model.geom_mesh[geom_id].map(|mesh_id| {
                CollisionShape::TriangleMesh {
                    data: Arc::clone(&model.mesh_data[mesh_id]),
                }
            })
        }
    }
}
```

**Note**: Function signature changes from `(geom_type, size)` to `(model, geom_id)`.
This is necessary because mesh collision requires access to `mesh_data`.
Update all call sites.

#### Step 3.2: Add Mesh Collision Dispatch in mj_collision()

The collision detection flow in `mj_collision()` needs mesh handling:

```rust
// In mj_collision(), after filtering passes

// Check if either geom is a mesh
let type1 = model.geom_type[geom1];
let type2 = model.geom_type[geom2];

// Fast path: analytical primitives (existing code)
if type1 != GeomType::Mesh && type2 != GeomType::Mesh {
    // ... existing primitive collision code ...
}

// Mesh collision path
else {
    collide_with_mesh(model, data, geom1, geom2, type1, type2, contacts);
}
```

#### Step 3.3: Implement collide_with_mesh()

```rust
fn collide_with_mesh(
    model: &Model,
    data: &Data,
    geom1: usize,
    geom2: usize,
    type1: GeomType,
    type2: GeomType,
    contacts: &mut Vec<Contact>,
) {
    let pos1 = data.geom_xpos[geom1];
    let mat1 = data.geom_xmat[geom1];
    let pos2 = data.geom_xpos[geom2];
    let mat2 = data.geom_xmat[geom2];

    let pose1 = Pose::from_position_rotation(pos1, mat1);
    let pose2 = Pose::from_position_rotation(pos2, mat2);

    let mesh_contacts = match (type1, type2) {
        // Mesh-Mesh
        (GeomType::Mesh, GeomType::Mesh) => {
            let mesh1_id = model.geom_mesh[geom1].expect("mesh geom must have mesh");
            let mesh2_id = model.geom_mesh[geom2].expect("mesh geom must have mesh");
            let mesh1 = &model.mesh_data[mesh1_id];
            let mesh2 = &model.mesh_data[mesh2_id];

            mesh_mesh_contact(mesh1, &pose1, mesh2, &pose2)
        }

        // Mesh-Primitive (mesh is geom1)
        (GeomType::Mesh, _) => {
            let mesh_id = model.geom_mesh[geom1].expect("mesh geom must have mesh");
            let mesh = &model.mesh_data[mesh_id];
            let shape2 = geom_to_collision_shape(model, geom2)
                .expect("primitive should have shape");

            collide_mesh_primitive(mesh, &pose1, &shape2, &pose2)
        }

        // Primitive-Mesh (mesh is geom2)
        (_, GeomType::Mesh) => {
            let mesh_id = model.geom_mesh[geom2].expect("mesh geom must have mesh");
            let mesh = &model.mesh_data[mesh_id];
            let shape1 = geom_to_collision_shape(model, geom1)
                .expect("primitive should have shape");

            // Swap order and negate normals
            let mut result = collide_mesh_primitive(mesh, &pose2, &shape1, &pose1);
            for c in &mut result {
                c.normal = -c.normal;
            }
            result
        }

        _ => unreachable!("at least one geom must be mesh"),
    };

    // Convert MeshContacts to Contacts
    for mc in mesh_contacts {
        if mc.penetration > 0.0 {
            contacts.push(Contact {
                pos: mc.point.coords,
                normal: mc.normal,
                depth: mc.penetration,
                geom1,
                geom2,
                friction: combine_friction(
                    model.geom_friction[geom1],
                    model.geom_friction[geom2],
                ),
                dim: 3,
                includemargin: false,
                mu: [model.geom_friction[geom1].x, model.geom_friction[geom2].x],
                solref: combine_solref(
                    model.geom_solref[geom1],
                    model.geom_solref[geom2],
                ),
                solimp: [0.9, 0.95, 0.001, 0.5, 2.0],
                frame: build_contact_frame(&mc.normal),
            });
        }
    }
}

fn collide_mesh_primitive(
    mesh: &TriangleMeshData,
    mesh_pose: &Pose,
    shape: &CollisionShape,
    shape_pose: &Pose,
) -> Vec<MeshContact> {
    match shape {
        CollisionShape::Sphere { radius } => {
            mesh_sphere_contact(mesh, mesh_pose, shape_pose.position, *radius)
        }
        CollisionShape::Box { half_extents } => {
            mesh_box_contact(mesh, mesh_pose, shape_pose, *half_extents)
        }
        CollisionShape::Capsule { half_length, radius } => {
            mesh_capsule_contact(mesh, mesh_pose, shape_pose, *half_length, *radius)
        }
        CollisionShape::Cylinder { half_length, radius } => {
            // Approximate cylinder as capsule for now
            // TODO: Proper cylinder-mesh collision
            mesh_capsule_contact(mesh, mesh_pose, shape_pose, *half_length, *radius)
        }
        CollisionShape::Ellipsoid { radii } => {
            // Approximate ellipsoid as sphere with max radius
            let max_radius = radii.x.max(radii.y).max(radii.z);
            mesh_sphere_contact(mesh, mesh_pose, shape_pose.position, max_radius)
        }
        _ => Vec::new(), // Plane handled separately, others unsupported
    }
}
```

---

### Phase 4: Testing

**Files**: `sim/L0/tests/integration/`, `sim/L0/core/src/mujoco_pipeline.rs` (unit tests)

#### Test 1: Mesh Loading from MJCF

```rust
#[test]
fn test_mesh_loading_embedded() {
    let mjcf = r#"
    <mujoco model="mesh_test">
        <asset>
            <mesh name="cube"
                  vertex="0 0 0  1 0 0  1 1 0  0 1 0  0 0 1  1 0 1  1 1 1  0 1 1"
                  face="0 1 2  0 2 3  4 6 5  4 7 6  0 4 5  0 5 1  2 6 7  2 7 3  0 3 7  0 7 4  1 5 6  1 6 2"/>
        </asset>
        <worldbody>
            <body name="cube_body" pos="0 0 1">
                <geom type="mesh" mesh="cube"/>
            </body>
        </worldbody>
    </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    assert_eq!(model.nmesh, 1);
    assert_eq!(model.mesh_name[0], "cube");
    assert!(model.mesh_data[0].triangles.len() == 12); // Cube has 12 triangles
    assert!(model.geom_mesh[1].is_some()); // geom 0 is world, geom 1 is cube
}
```

#### Test 2: Mesh-Plane Collision

```rust
#[test]
fn test_mesh_plane_collision() {
    let mjcf = r#"
    <mujoco model="mesh_plane_test">
        <asset>
            <mesh name="tetra"
                  vertex="0 0 0  1 0 0  0.5 0.866 0  0.5 0.289 0.816"
                  face="0 1 2  0 2 3  0 3 1  1 3 2"/>
        </asset>
        <worldbody>
            <geom type="plane" size="10 10 0.1"/>
            <body name="tetra_body" pos="0 0 0.1">
                <freejoint/>
                <geom type="mesh" mesh="tetra" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Step and check for contacts
    data.forward(&model);
    mj_collision(&model, &mut data);

    assert!(data.ncon > 0, "mesh should contact plane");
}
```

#### Test 3: Mesh-Mesh Collision

```rust
#[test]
fn test_mesh_mesh_collision() {
    // Two cubes colliding
    let mjcf = r#"
    <mujoco model="mesh_mesh_test">
        <asset>
            <mesh name="cube" vertex="..." face="..."/>
        </asset>
        <worldbody>
            <body name="cube1" pos="0 0 0">
                <freejoint/>
                <geom type="mesh" mesh="cube" mass="1.0"/>
            </body>
            <body name="cube2" pos="0.5 0 0">
                <freejoint/>
                <geom type="mesh" mesh="cube" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    data.forward(&model);
    mj_collision(&model, &mut data);

    assert!(data.ncon > 0, "overlapping meshes should contact");
}
```

#### Test 4: Mesh Bounding Radius

```rust
#[test]
fn test_mesh_bounding_radius() {
    // Unit cube centered at origin
    let mjcf = r#"
    <mujoco>
        <asset>
            <mesh name="unit_cube"
                  vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5
                          -0.5 -0.5  0.5  0.5 -0.5  0.5  0.5 0.5  0.5  -0.5 0.5  0.5"
                  face="0 1 2  0 2 3  4 6 5  4 7 6  0 4 5  0 5 1  2 6 7  2 7 3  0 3 7  0 7 4  1 5 6  1 6 2"/>
        </asset>
        <worldbody>
            <body pos="0 0 0">
                <geom type="mesh" mesh="unit_cube"/>
            </body>
        </worldbody>
    </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    // Bounding radius of unit cube = sqrt(0.5² + 0.5² + 0.5²) ≈ 0.866
    let expected = (0.5_f64 * 0.5 * 3.0).sqrt();
    assert_relative_eq!(model.geom_rbound[1], expected, epsilon = 0.01);
}
```

---

## Success Criteria

### Functional

- [ ] MJCF with `<mesh>` assets loads correctly
- [ ] Mesh-plane collision produces contacts
- [ ] Mesh-sphere collision produces contacts
- [ ] Mesh-box collision produces contacts
- [ ] Mesh-capsule collision produces contacts
- [ ] Mesh-mesh collision produces contacts
- [ ] Mesh instancing works (multiple geoms reference same mesh)
- [ ] Bounding radius computed from actual mesh AABB

### Performance

- [ ] BVH construction happens once at model load (not per-step)
- [ ] Broad-phase culling uses `geom_rbound` (no mesh traversal)
- [ ] Debug mode maintains >1000 steps/sec for humanoid (no regression)

### Todorov Quality

- [ ] No code duplication with existing `mesh.rs` functions
- [ ] Single source of truth for mesh data (`model.mesh_data`)
- [ ] `Arc` used for mesh data (expensive to copy, cheap to share)
- [ ] All mesh collision goes through `collide_with_mesh()`
- [ ] No special cases in hot loops

---

## File Change Summary

| File | Changes |
|------|---------|
| `mujoco_pipeline.rs` | Add `nmesh`, `mesh_name`, `mesh_data`, `geom_mesh` to Model |
| `mujoco_pipeline.rs` | Add `collide_with_mesh()`, `collide_mesh_primitive()` |
| `mujoco_pipeline.rs` | Update `geom_to_collision_shape()` signature |
| `model_builder.rs` | Add mesh asset loading and geom-mesh linking |
| `model_builder.rs` | Compute `geom_rbound` from mesh AABB for mesh geoms |
| `validation.rs` | Add mesh collision tests |

---

## Dependencies

```toml
# sim-core Cargo.toml - no new dependencies needed
# mesh.rs already provides TriangleMeshData
# mesh-boolean provides Bvh (already in workspace)
```

---

## Implementation Order

1. **Phase 1**: Model fields (1 hour)
   - Add fields to Model
   - Initialize in `empty()` and `make_data()`

2. **Phase 2**: MJCF loading (2 hours)
   - Process mesh assets
   - Link geoms to meshes
   - Compute bounding radii

3. **Phase 3**: Collision detection (2 hours)
   - Update `geom_to_collision_shape()`
   - Add mesh dispatch in `mj_collision()`
   - Implement `collide_with_mesh()`

4. **Phase 4**: Testing (1 hour)
   - Unit tests for loading
   - Integration tests for collision
   - Performance regression test

**Total estimated effort**: 6 hours

---

## References

- Featherstone, R. (2008). "Rigid Body Dynamics Algorithms" - Chapter on collision
- Ericson, C. (2005). "Real-Time Collision Detection" - BVH and SAT
- MuJoCo Documentation: Assets chapter (mesh loading)
- Todorov, E. (2014). "Convex and analytically-invertible dynamics"
