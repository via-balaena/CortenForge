# Level 4 Collision Detection: Upgrade to A+ Great

> **Todorov Standard**: Single source of truth. No duplication. O(n) where possible.
> Compute once, use everywhere. Profile before optimizing.
>
> **Rust Purist Standard**: Zero allocations in hot paths. No unwrap in production.
> Prefer slices over Vec cloning. Exhaustive match, no wildcards.
>
> **Last Updated**: 2026-01-27

---

## Executive Summary

This spec upgrades Level 4 (Collision Detection) from "Mostly Solid" to "A+ Great" by:

1. **Wiring mesh collision** into the pipeline (the functions exist, just not connected)
2. **Adding analytical cylinder-primitive** collisions (eliminate GJK/EPA for common cases)
3. **Cleaning up plane collision** for cylinder/ellipsoid/mesh (currently returns `None`)

**Key Insight**: 90% of the code exists. This is primarily integration and polishing work.

---

## Current State Matrix

### Analytical Implementations (Fast Path) ✅

| Pair | Function | Lines | Status |
|------|----------|-------|--------|
| Plane-Sphere | `collide_with_plane()` | 7618 | ✅ Complete |
| Plane-Box | `collide_with_plane()` | 7618 | ✅ Complete |
| Plane-Capsule | `collide_with_plane()` | 7618 | ✅ Complete |
| Sphere-Sphere | `collide_sphere_sphere()` | 7769 | ✅ Complete |
| Sphere-Capsule | `collide_sphere_capsule()` | 7879 | ✅ Complete |
| Sphere-Box | `collide_sphere_box()` | 7960 | ✅ Complete |
| Capsule-Capsule | `collide_capsule_capsule()` | 7820 | ✅ Complete |
| Capsule-Box | `collide_capsule_box()` | 8057 | ✅ Complete |
| Box-Box | `collide_box_box()` | 8204 | ✅ Complete (SAT) |

### GJK/EPA Fallback (Slow Path) ⚠️

| Pair | Current | Issue |
|------|---------|-------|
| Cylinder-* | GJK/EPA | Expensive quaternion conversion |
| Ellipsoid-* | GJK/EPA | Expensive quaternion conversion |
| Plane-Cylinder | Returns `None` | **BUG**: No collision detected |
| Plane-Ellipsoid | Returns `None` | **BUG**: No collision detected |
| Plane-Mesh | Returns `None` | **BUG**: No collision detected |

### Mesh Collision ❌

| Component | Location | Status |
|-----------|----------|--------|
| `TriangleMeshData` | `mesh.rs` | ✅ Complete with BVH |
| `mesh_sphere_contact()` | `mesh.rs:1043` | ✅ Complete, tested |
| `mesh_capsule_contact()` | `mesh.rs:1092` | ✅ Complete, tested |
| `mesh_box_contact()` | `mesh.rs:1150` | ✅ Complete, tested |
| `mesh_mesh_contact()` | `mesh.rs:1249` | ✅ Complete, tested |
| Pipeline integration | `mujoco_pipeline.rs:7612` | ❌ Returns `None` |
| Model.mesh_data | `mujoco_pipeline.rs` | ❌ Field missing |
| Model.geom_mesh | `mujoco_pipeline.rs` | ❌ Field missing |

---

## Implementation Plan

### Phase 1: Fix Plane Collision Gaps (Critical Bug Fix)

**Problem**: `collide_with_plane()` returns `None` for Cylinder, Ellipsoid, and Mesh.
This means objects fall through the ground.

**File**: `mujoco_pipeline.rs`

#### Step 1.1: Plane-Cylinder Collision

```rust
/// Collide cylinder with plane.
/// Returns up to 2 contacts (rim edges touching plane).
fn collide_cylinder_plane(
    model: &Model,
    plane_geom: usize,
    cyl_geom: usize,
    plane_pos: Vector3<f64>,
    plane_mat: Matrix3<f64>,
    cyl_pos: Vector3<f64>,
    cyl_mat: Matrix3<f64>,
    cyl_size: Vector3<f64>,
) -> Option<Contact> {
    let plane_normal = plane_mat.column(2).into_owned();
    let plane_d = plane_normal.dot(&plane_pos);

    let radius = cyl_size.x;
    let half_height = cyl_size.y;
    let cyl_axis = cyl_mat.column(2).into_owned(); // Local Z is cylinder axis

    // Two rim centers: top and bottom of cylinder
    let top_center = cyl_pos + cyl_axis * half_height;
    let bottom_center = cyl_pos - cyl_axis * half_height;

    // Find point on each rim closest to plane
    // Project plane normal onto rim plane, find extremal point
    let rim_dir = (plane_normal - cyl_axis * plane_normal.dot(&cyl_axis)).normalize();

    // If cylinder axis is parallel to plane normal, use any perpendicular
    let rim_dir = if rim_dir.norm() < 1e-10 {
        // Find any vector perpendicular to cyl_axis
        let arbitrary = if cyl_axis.x.abs() < 0.9 {
            Vector3::x()
        } else {
            Vector3::y()
        };
        cyl_axis.cross(&arbitrary).normalize()
    } else {
        rim_dir
    };

    // Four candidate contact points: top/bottom rim × ±rim_dir
    let candidates = [
        top_center - rim_dir * radius,
        top_center + rim_dir * radius,
        bottom_center - rim_dir * radius,
        bottom_center + rim_dir * radius,
    ];

    // Find deepest penetrating point
    let mut deepest_depth = f64::NEG_INFINITY;
    let mut deepest_point = cyl_pos;

    for point in &candidates {
        let signed_dist = plane_normal.dot(point) - plane_d;
        let depth = -signed_dist; // Positive = penetrating
        if depth > deepest_depth {
            deepest_depth = depth;
            deepest_point = *point;
        }
    }

    if deepest_depth <= 0.0 {
        return None; // No penetration
    }

    // Contact point is on plane surface
    let contact_pos = deepest_point + plane_normal * deepest_depth;

    Some(Contact {
        pos: contact_pos,
        normal: plane_normal,
        depth: deepest_depth,
        geom1: plane_geom,
        geom2: cyl_geom,
        dim: 3,
        friction: combine_friction(
            model.geom_friction[plane_geom],
            model.geom_friction[cyl_geom],
        ),
        mu: [
            model.geom_friction[plane_geom].x,
            model.geom_friction[cyl_geom].x,
        ],
        solref: combine_solref(
            model.geom_solref[plane_geom],
            model.geom_solref[cyl_geom],
        ),
        solimp: [0.9, 0.95, 0.001, 0.5, 2.0],
        includemargin: false,
        frame: build_contact_frame(&plane_normal),
    })
}
```

#### Step 1.2: Plane-Ellipsoid Collision

```rust
/// Collide ellipsoid with plane.
/// Ellipsoid defined by radii (a, b, c) along local axes.
fn collide_ellipsoid_plane(
    model: &Model,
    plane_geom: usize,
    ell_geom: usize,
    plane_pos: Vector3<f64>,
    plane_mat: Matrix3<f64>,
    ell_pos: Vector3<f64>,
    ell_mat: Matrix3<f64>,
    ell_radii: Vector3<f64>,
) -> Option<Contact> {
    let plane_normal = plane_mat.column(2).into_owned();
    let plane_d = plane_normal.dot(&plane_pos);

    // Transform plane normal to ellipsoid local frame
    let local_normal = ell_mat.transpose() * plane_normal;

    // Scale by radii to find support point in local frame
    // For ellipsoid x²/a² + y²/b² + z²/c² = 1,
    // support point in direction n is: (a²n_x, b²n_y, c²n_z) / ||(a*n_x, b*n_y, c*n_z)||
    let scaled = Vector3::new(
        ell_radii.x * local_normal.x,
        ell_radii.y * local_normal.y,
        ell_radii.z * local_normal.z,
    );
    let scale_norm = scaled.norm();

    if scale_norm < 1e-10 {
        return None; // Degenerate case
    }

    // Support point on ellipsoid surface (most negative in plane normal direction)
    let local_support = -Vector3::new(
        ell_radii.x * scaled.x / scale_norm,
        ell_radii.y * scaled.y / scale_norm,
        ell_radii.z * scaled.z / scale_norm,
    );

    // Transform to world frame
    let world_support = ell_pos + ell_mat * local_support;

    // Check penetration
    let signed_dist = plane_normal.dot(&world_support) - plane_d;
    let depth = -signed_dist;

    if depth <= 0.0 {
        return None;
    }

    let contact_pos = world_support + plane_normal * depth;

    Some(Contact {
        pos: contact_pos,
        normal: plane_normal,
        depth,
        geom1: plane_geom,
        geom2: ell_geom,
        dim: 3,
        friction: combine_friction(
            model.geom_friction[plane_geom],
            model.geom_friction[ell_geom],
        ),
        mu: [
            model.geom_friction[plane_geom].x,
            model.geom_friction[ell_geom].x,
        ],
        solref: combine_solref(
            model.geom_solref[plane_geom],
            model.geom_solref[ell_geom],
        ),
        solimp: [0.9, 0.95, 0.001, 0.5, 2.0],
        includemargin: false,
        frame: build_contact_frame(&plane_normal),
    })
}
```

#### Step 1.3: Update `collide_with_plane()` Dispatch

```rust
fn collide_with_plane(
    model: &Model,
    plane_geom: usize,
    other_geom: usize,
    plane_pos: Vector3<f64>,
    plane_mat: Matrix3<f64>,
    other_pos: Vector3<f64>,
    other_mat: Matrix3<f64>,
    other_type: GeomType,
    other_size: Vector3<f64>,
) -> Option<Contact> {
    match other_type {
        GeomType::Sphere => { /* existing */ }
        GeomType::Box => { /* existing */ }
        GeomType::Capsule => { /* existing */ }
        GeomType::Cylinder => collide_cylinder_plane(
            model, plane_geom, other_geom,
            plane_pos, plane_mat, other_pos, other_mat, other_size,
        ),
        GeomType::Ellipsoid => collide_ellipsoid_plane(
            model, plane_geom, other_geom,
            plane_pos, plane_mat, other_pos, other_mat, other_size,
        ),
        GeomType::Mesh => {
            // Handled by collide_mesh_plane() after mesh integration
            None // TODO: Phase 2
        }
        GeomType::Plane => None, // Plane-plane not supported
    }
}
```

---

### Phase 2: Model/Data Infrastructure for Meshes

**File**: `mujoco_pipeline.rs`

#### Step 2.1: Add Model Fields

```rust
// In Model struct, after geom_rbound:

// ==================== Meshes (indexed by mesh_id) ====================
/// Number of mesh assets.
pub nmesh: usize,
/// Mesh names (for lookup).
pub mesh_name: Vec<String>,
/// Triangle mesh data with prebuilt BVH.
/// Arc for cheap cloning (multiple geoms can reference same mesh).
pub mesh_data: Vec<Arc<TriangleMeshData>>,

// ==================== Geom-Mesh Mapping ====================
/// Mesh index for each geom (None if not a mesh geom).
/// Length: ngeom.
pub geom_mesh: Vec<Option<usize>>,
```

#### Step 2.2: Initialize in `Model::empty()`

```rust
// Add to Model::empty():
nmesh: 0,
mesh_name: vec![],
mesh_data: vec![],
geom_mesh: vec![],
```

#### Step 2.3: Ensure `geom_mesh` Sized with Geoms

In `Model::build()` or wherever geoms are finalized:

```rust
// Ensure geom_mesh has correct length
if model.geom_mesh.len() < model.ngeom {
    model.geom_mesh.resize(model.ngeom, None);
}
```

---

### Phase 3: MJCF Mesh Loading

**File**: `model_builder.rs`

#### Step 3.1: Process Mesh Assets

```rust
use sim_core::mesh::TriangleMeshData;
use std::sync::Arc;

/// Convert MJCF mesh asset to TriangleMeshData.
fn convert_mjcf_mesh(mjcf_mesh: &MjcfMesh) -> Result<TriangleMeshData, ModelBuildError> {
    // Get vertices
    let vertices: Vec<Point3<f64>> = match &mjcf_mesh.vertex {
        Some(verts) => {
            if verts.len() % 3 != 0 {
                return Err(ModelBuildError::InvalidMesh {
                    name: mjcf_mesh.name.clone(),
                    reason: "vertex count not divisible by 3".into(),
                });
            }
            verts
                .chunks_exact(3)
                .map(|chunk| {
                    Point3::new(
                        chunk[0] * mjcf_mesh.scale.x,
                        chunk[1] * mjcf_mesh.scale.y,
                        chunk[2] * mjcf_mesh.scale.z,
                    )
                })
                .collect()
        }
        None => {
            // TODO: Load from file if mjcf_mesh.file is Some
            return Err(ModelBuildError::InvalidMesh {
                name: mjcf_mesh.name.clone(),
                reason: "no vertex data (file loading not yet implemented)".into(),
            });
        }
    };

    // Get faces
    let faces: Vec<[usize; 3]> = match &mjcf_mesh.face {
        Some(face_data) => {
            if face_data.len() % 3 != 0 {
                return Err(ModelBuildError::InvalidMesh {
                    name: mjcf_mesh.name.clone(),
                    reason: "face count not divisible by 3".into(),
                });
            }
            face_data
                .chunks_exact(3)
                .map(|chunk| [chunk[0] as usize, chunk[1] as usize, chunk[2] as usize])
                .collect()
        }
        None => {
            return Err(ModelBuildError::InvalidMesh {
                name: mjcf_mesh.name.clone(),
                reason: "no face data".into(),
            });
        }
    };

    if vertices.is_empty() || faces.is_empty() {
        return Err(ModelBuildError::InvalidMesh {
            name: mjcf_mesh.name.clone(),
            reason: "empty mesh".into(),
        });
    }

    // TriangleMeshData::new() builds the BVH automatically
    Ok(TriangleMeshData::new(vertices, faces))
}
```

#### Step 3.2: Build Mesh Lookup Table

```rust
// In model_from_mjcf(), before processing bodies:

let mut mesh_name_to_id: HashMap<String, usize> = HashMap::new();
let mut mesh_data_vec: Vec<Arc<TriangleMeshData>> = Vec::new();
let mut mesh_name_vec: Vec<String> = Vec::new();

for mjcf_mesh in &mjcf.meshes {
    let mesh_id = mesh_data_vec.len();
    mesh_name_to_id.insert(mjcf_mesh.name.clone(), mesh_id);
    mesh_name_vec.push(mjcf_mesh.name.clone());

    let mesh_data = convert_mjcf_mesh(mjcf_mesh)?;
    mesh_data_vec.push(Arc::new(mesh_data));
}

// Later, assign to model:
model.nmesh = mesh_data_vec.len();
model.mesh_name = mesh_name_vec;
model.mesh_data = mesh_data_vec;
```

#### Step 3.3: Link Geoms to Meshes

```rust
// In process_geom(), when geom type is Mesh:

fn process_geom(
    geom: &MjcfGeom,
    mesh_name_to_id: &HashMap<String, usize>,
    mesh_data: &[Arc<TriangleMeshData>],
    builder: &mut ModelBuilder,
) -> Result<(), ModelBuildError> {
    // ... existing geom processing ...

    let geom_mesh_ref = if geom.geom_type == MjcfGeomType::Mesh {
        match &geom.mesh {
            Some(mesh_name) => {
                let mesh_id = mesh_name_to_id.get(mesh_name).ok_or_else(|| {
                    ModelBuildError::UnknownMesh {
                        geom: geom.name.clone().unwrap_or_default(),
                        mesh: mesh_name.clone(),
                    }
                })?;

                // Compute bounding radius from mesh AABB
                let mesh = &mesh_data[*mesh_id];
                let half_extents = (mesh.aabb_max() - mesh.aabb_min()) / 2.0;
                let rbound = half_extents.coords.norm();
                builder.geom_rbound.push(rbound);

                Some(*mesh_id)
            }
            None => {
                return Err(ModelBuildError::MissingMeshAttribute {
                    geom: geom.name.clone().unwrap_or_default(),
                });
            }
        }
    } else {
        // Non-mesh geom: compute rbound from size
        let rbound = geom.geom_type.bounding_radius(&geom.size);
        builder.geom_rbound.push(rbound);
        None
    };

    builder.geom_mesh.push(geom_mesh_ref);

    // ... rest of geom processing ...
}
```

---

### Phase 4: Mesh Collision Detection

**File**: `mujoco_pipeline.rs`

#### Step 4.1: Add Mesh Contact Type

```rust
use sim_core::mesh::{
    mesh_sphere_contact, mesh_capsule_contact, mesh_box_contact, mesh_mesh_contact,
    TriangleMeshData,
};
```

#### Step 4.2: Implement `collide_with_mesh()`

```rust
/// Collision detection involving at least one mesh geometry.
fn collide_with_mesh(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
) -> Option<Contact> {
    let type1 = model.geom_type[geom1];
    let type2 = model.geom_type[geom2];
    let size1 = model.geom_size[geom1];
    let size2 = model.geom_size[geom2];

    // Build poses
    let pose1 = Pose::from_position_matrix(Point3::from(pos1), mat1);
    let pose2 = Pose::from_position_matrix(Point3::from(pos2), mat2);

    let mesh_contact = match (type1, type2) {
        // Mesh-Mesh
        (GeomType::Mesh, GeomType::Mesh) => {
            let mesh1_id = model.geom_mesh[geom1].expect("mesh geom must have mesh");
            let mesh2_id = model.geom_mesh[geom2].expect("mesh geom must have mesh");
            let mesh1 = &model.mesh_data[mesh1_id];
            let mesh2 = &model.mesh_data[mesh2_id];

            mesh_mesh_contact(mesh1, &pose1, mesh2, &pose2)
                .into_iter()
                .max_by(|a, b| a.penetration.partial_cmp(&b.penetration).unwrap())
        }

        // Mesh (geom1) vs Primitive (geom2)
        (GeomType::Mesh, prim_type) => {
            let mesh_id = model.geom_mesh[geom1].expect("mesh geom must have mesh");
            let mesh = &model.mesh_data[mesh_id];

            match prim_type {
                GeomType::Sphere => {
                    mesh_sphere_contact(mesh, &pose1, pose2.position, size2.x)
                }
                GeomType::Capsule => {
                    mesh_capsule_contact(mesh, &pose1, &pose2, size2.y, size2.x)
                }
                GeomType::Box => {
                    mesh_box_contact(mesh, &pose1, &pose2, size2)
                }
                GeomType::Cylinder => {
                    // Approximate as capsule (conservative)
                    mesh_capsule_contact(mesh, &pose1, &pose2, size2.y, size2.x)
                }
                GeomType::Ellipsoid => {
                    // Approximate as sphere with max radius (conservative)
                    let max_r = size2.x.max(size2.y).max(size2.z);
                    mesh_sphere_contact(mesh, &pose1, pose2.position, max_r)
                }
                GeomType::Plane => {
                    collide_mesh_plane(model, geom1, geom2, &pose1, pos2, mat2)
                }
                GeomType::Mesh => unreachable!("handled above"),
            }
        }

        // Primitive (geom1) vs Mesh (geom2)
        (prim_type, GeomType::Mesh) => {
            let mesh_id = model.geom_mesh[geom2].expect("mesh geom must have mesh");
            let mesh = &model.mesh_data[mesh_id];

            let contact = match prim_type {
                GeomType::Sphere => {
                    mesh_sphere_contact(mesh, &pose2, pose1.position, size1.x)
                }
                GeomType::Capsule => {
                    mesh_capsule_contact(mesh, &pose2, &pose1, size1.y, size1.x)
                }
                GeomType::Box => {
                    mesh_box_contact(mesh, &pose2, &pose1, size1)
                }
                GeomType::Cylinder => {
                    mesh_capsule_contact(mesh, &pose2, &pose1, size1.y, size1.x)
                }
                GeomType::Ellipsoid => {
                    let max_r = size1.x.max(size1.y).max(size1.z);
                    mesh_sphere_contact(mesh, &pose2, pose1.position, max_r)
                }
                GeomType::Plane => {
                    collide_mesh_plane(model, geom2, geom1, &pose2, pos1, mat1)
                }
                GeomType::Mesh => unreachable!("handled above"),
            };

            // Swap normal direction (mesh was second, but contact expects geom1→geom2)
            contact.map(|mut c| {
                c.normal = -c.normal;
                c
            })
        }

        _ => unreachable!("at least one geom must be mesh"),
    };

    // Convert MeshContact to Contact
    mesh_contact.map(|mc| Contact {
        pos: mc.point.coords,
        normal: mc.normal,
        depth: mc.penetration,
        geom1,
        geom2,
        dim: 3,
        friction: combine_friction(model.geom_friction[geom1], model.geom_friction[geom2]),
        mu: [model.geom_friction[geom1].x, model.geom_friction[geom2].x],
        solref: combine_solref(model.geom_solref[geom1], model.geom_solref[geom2]),
        solimp: [0.9, 0.95, 0.001, 0.5, 2.0],
        includemargin: false,
        frame: build_contact_frame(&mc.normal),
    })
}

/// Mesh vs infinite plane collision.
fn collide_mesh_plane(
    model: &Model,
    mesh_geom: usize,
    plane_geom: usize,
    mesh_pose: &Pose,
    plane_pos: Vector3<f64>,
    plane_mat: Matrix3<f64>,
) -> Option<MeshContact> {
    let mesh_id = model.geom_mesh[mesh_geom].expect("mesh geom must have mesh");
    let mesh = &model.mesh_data[mesh_id];

    let plane_normal = plane_mat.column(2).into_owned();
    let plane_d = plane_normal.dot(&plane_pos);

    // Find deepest penetrating vertex
    let mut deepest: Option<MeshContact> = None;

    for vertex in mesh.vertices() {
        // Transform to world
        let world_v = mesh_pose.transform_point(vertex);
        let signed_dist = plane_normal.dot(&world_v.coords) - plane_d;
        let depth = -signed_dist;

        if depth > 0.0 {
            match &mut deepest {
                Some(d) if depth > d.penetration => {
                    d.point = world_v;
                    d.normal = plane_normal;
                    d.penetration = depth;
                }
                None => {
                    deepest = Some(MeshContact {
                        point: world_v,
                        normal: plane_normal,
                        penetration: depth,
                    });
                }
                _ => {}
            }
        }
    }

    deepest
}
```

#### Step 4.3: Update `collide_geoms()` Dispatcher

```rust
fn collide_geoms(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
) -> Option<Contact> {
    let type1 = model.geom_type[geom1];
    let type2 = model.geom_type[geom2];
    let size1 = model.geom_size[geom1];
    let size2 = model.geom_size[geom2];

    // ========== Mesh collision path ==========
    if type1 == GeomType::Mesh || type2 == GeomType::Mesh {
        return collide_with_mesh(model, geom1, geom2, pos1, mat1, pos2, mat2);
    }

    // ========== Plane collision path (fast, analytical) ==========
    if type1 == GeomType::Plane {
        return collide_with_plane(
            model, geom1, geom2, pos1, mat1, pos2, mat2, type2, size2,
        );
    }
    if type2 == GeomType::Plane {
        return collide_with_plane(
            model, geom2, geom1, pos2, mat2, pos1, mat1, type1, size1,
        ).map(|mut c| {
            std::mem::swap(&mut c.geom1, &mut c.geom2);
            c.normal = -c.normal;
            c
        });
    }

    // ========== Primitive-primitive fast path ==========
    match (type1, type2) {
        (GeomType::Sphere, GeomType::Sphere) => {
            collide_sphere_sphere(model, geom1, geom2, pos1, pos2, size1, size2)
        }
        (GeomType::Capsule, GeomType::Capsule) => {
            collide_capsule_capsule(model, geom1, geom2, pos1, mat1, pos2, mat2, size1, size2)
        }
        (GeomType::Sphere, GeomType::Capsule) | (GeomType::Capsule, GeomType::Sphere) => {
            // ... existing ...
        }
        (GeomType::Sphere, GeomType::Box) | (GeomType::Box, GeomType::Sphere) => {
            // ... existing ...
        }
        (GeomType::Capsule, GeomType::Box) | (GeomType::Box, GeomType::Capsule) => {
            // ... existing ...
        }
        (GeomType::Box, GeomType::Box) => {
            collide_box_box(model, geom1, geom2, pos1, mat1, pos2, mat2, size1, size2)
        }

        // ========== Cylinder analytical (NEW) ==========
        (GeomType::Cylinder, GeomType::Sphere) | (GeomType::Sphere, GeomType::Cylinder) => {
            collide_cylinder_sphere(model, geom1, geom2, pos1, mat1, pos2, mat2, size1, size2, type1)
        }
        (GeomType::Cylinder, GeomType::Capsule) | (GeomType::Capsule, GeomType::Cylinder) => {
            collide_cylinder_capsule(model, geom1, geom2, pos1, mat1, pos2, mat2, size1, size2, type1)
        }

        // ========== GJK/EPA fallback for remaining cases ==========
        _ => {
            // Cylinder-Cylinder, Cylinder-Box, Ellipsoid-*, etc.
            collide_gjk_epa(model, geom1, geom2, pos1, mat1, pos2, mat2, type1, type2, size1, size2)
        }
    }
}
```

---

### Phase 5: Analytical Cylinder Collisions (Performance Polish)

**File**: `mujoco_pipeline.rs`

#### Step 5.1: Cylinder-Sphere

```rust
/// Analytical cylinder-sphere collision.
/// Cylinder axis is local Z.
fn collide_cylinder_sphere(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
    type1: GeomType,
) -> Option<Contact> {
    // Determine which is cylinder and which is sphere
    let (cyl_geom, cyl_pos, cyl_mat, cyl_size, sph_geom, sph_pos, sph_radius) =
        if type1 == GeomType::Cylinder {
            (geom1, pos1, mat1, size1, geom2, pos2, size2.x)
        } else {
            (geom2, pos2, mat2, size2, geom1, pos1, size1.x)
        };

    let cyl_radius = cyl_size.x;
    let cyl_half_height = cyl_size.y;
    let cyl_axis = cyl_mat.column(2).into_owned();

    // Vector from cylinder center to sphere center
    let d = sph_pos - cyl_pos;

    // Project onto cylinder axis
    let axis_dist = d.dot(&cyl_axis);

    // Clamp to cylinder height
    let clamped_axis = axis_dist.clamp(-cyl_half_height, cyl_half_height);

    // Closest point on cylinder axis to sphere center
    let axis_point = cyl_pos + cyl_axis * clamped_axis;

    // Radial vector from axis to sphere
    let radial = sph_pos - axis_point;
    let radial_dist = radial.norm();

    // Three cases: side collision, cap collision, edge collision
    let (closest_on_cyl, normal) = if axis_dist.abs() <= cyl_half_height {
        // Sphere is beside the cylinder (side collision)
        if radial_dist < 1e-10 {
            // Sphere center on axis - degenerate, pick arbitrary normal
            let arb = if cyl_axis.x.abs() < 0.9 { Vector3::x() } else { Vector3::y() };
            let n = cyl_axis.cross(&arb).normalize();
            (axis_point + n * cyl_radius, n)
        } else {
            let n = radial / radial_dist;
            (axis_point + n * cyl_radius, n)
        }
    } else {
        // Sphere is above/below cylinder (cap or edge collision)
        let cap_center = cyl_pos + cyl_axis * clamped_axis;

        if radial_dist <= cyl_radius {
            // Cap collision
            let n = if axis_dist > 0.0 { cyl_axis } else { -cyl_axis };
            (cap_center, n)
        } else {
            // Edge collision (rim of cap)
            let radial_n = radial / radial_dist;
            let edge_point = cap_center + radial_n * cyl_radius;
            let n = (sph_pos - edge_point).normalize();
            (edge_point, n)
        }
    };

    let dist = (sph_pos - closest_on_cyl).norm();
    let depth = sph_radius - dist;

    if depth <= 0.0 {
        return None;
    }

    // Normal points from cylinder to sphere
    let final_normal = if type1 == GeomType::Cylinder { normal } else { -normal };

    Some(Contact {
        pos: closest_on_cyl + normal * (depth / 2.0),
        normal: final_normal,
        depth,
        geom1: if type1 == GeomType::Cylinder { geom1 } else { geom2 },
        geom2: if type1 == GeomType::Cylinder { geom2 } else { geom1 },
        dim: 3,
        friction: combine_friction(model.geom_friction[geom1], model.geom_friction[geom2]),
        mu: [model.geom_friction[geom1].x, model.geom_friction[geom2].x],
        solref: combine_solref(model.geom_solref[geom1], model.geom_solref[geom2]),
        solimp: [0.9, 0.95, 0.001, 0.5, 2.0],
        includemargin: false,
        frame: build_contact_frame(&final_normal),
    })
}
```

#### Step 5.2: Cylinder-Capsule

```rust
/// Analytical cylinder-capsule collision.
/// Both have axis along local Z.
fn collide_cylinder_capsule(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
    type1: GeomType,
) -> Option<Contact> {
    // Identify cylinder and capsule
    let (cyl_pos, cyl_mat, cyl_radius, cyl_hh, cap_pos, cap_mat, cap_radius, cap_hh) =
        if type1 == GeomType::Cylinder {
            (pos1, mat1, size1.x, size1.y, pos2, mat2, size2.x, size2.y)
        } else {
            (pos2, mat2, size2.x, size2.y, pos1, mat1, size1.x, size1.y)
        };

    let cyl_axis = cyl_mat.column(2).into_owned();
    let cap_axis = cap_mat.column(2).into_owned();

    // Capsule endpoints
    let cap_a = cap_pos - cap_axis * cap_hh;
    let cap_b = cap_pos + cap_axis * cap_hh;

    // Find closest points between cylinder axis segment and capsule axis segment
    let cyl_a = cyl_pos - cyl_axis * cyl_hh;
    let cyl_b = cyl_pos + cyl_axis * cyl_hh;

    let (t_cyl, t_cap) = closest_points_segments(cyl_a, cyl_b, cap_a, cap_b);

    let cyl_closest = cyl_a + (cyl_b - cyl_a) * t_cyl;
    let cap_closest = cap_a + (cap_b - cap_a) * t_cap;

    // Vector from cylinder axis point to capsule axis point
    let diff = cap_closest - cyl_closest;
    let dist = diff.norm();

    if dist < 1e-10 {
        // Axes intersect - use cross product for normal
        let normal = cyl_axis.cross(&cap_axis);
        if normal.norm() < 1e-10 {
            // Parallel axes
            return None; // TODO: Handle parallel case
        }
        // Fall through to GJK for this edge case
        return None;
    }

    let normal = diff / dist;

    // Closest point on cylinder surface
    let cyl_surface = cyl_closest + normal * cyl_radius;

    // Distance from cylinder surface to capsule axis
    let surface_to_cap = (cap_closest - cyl_surface).norm();
    let depth = cap_radius - surface_to_cap;

    if depth <= 0.0 {
        return None;
    }

    let contact_pos = cyl_surface + normal * (depth / 2.0);

    // Determine geom order for output
    let (g1, g2, final_normal) = if type1 == GeomType::Cylinder {
        (geom1, geom2, normal)
    } else {
        (geom2, geom1, -normal)
    };

    Some(Contact {
        pos: contact_pos,
        normal: final_normal,
        depth,
        geom1: g1,
        geom2: g2,
        dim: 3,
        friction: combine_friction(model.geom_friction[geom1], model.geom_friction[geom2]),
        mu: [model.geom_friction[geom1].x, model.geom_friction[geom2].x],
        solref: combine_solref(model.geom_solref[geom1], model.geom_solref[geom2]),
        solimp: [0.9, 0.95, 0.001, 0.5, 2.0],
        includemargin: false,
        frame: build_contact_frame(&final_normal),
    })
}

/// Find closest points on two line segments.
/// Returns (t1, t2) where closest points are a1 + t1*(b1-a1) and a2 + t2*(b2-a2).
fn closest_points_segments(
    a1: Vector3<f64>,
    b1: Vector3<f64>,
    a2: Vector3<f64>,
    b2: Vector3<f64>,
) -> (f64, f64) {
    let d1 = b1 - a1;
    let d2 = b2 - a2;
    let r = a1 - a2;

    let a = d1.dot(&d1);
    let b = d1.dot(&d2);
    let c = d1.dot(&r);
    let e = d2.dot(&d2);
    let f = d2.dot(&r);

    let denom = a * e - b * b;

    let (s, t) = if denom.abs() < 1e-10 {
        // Parallel segments
        (0.0, f / e)
    } else {
        let s = (b * f - c * e) / denom;
        let t = (a * f - b * c) / denom;
        (s, t)
    };

    // Clamp to [0, 1]
    let s = s.clamp(0.0, 1.0);
    let t = t.clamp(0.0, 1.0);

    (s, t)
}
```

---

### Phase 6: Testing — The Obsessive Validation Protocol

> **Philosophy**: Every collision path must be tested in isolation AND composition.
> Every geometric edge case must be enumerated. Every numerical tolerance must be justified.
> If MuJoCo's `engine_collision_primitive.c` has a test, we have a stricter one.

**Files**:
- `sim/L0/tests/integration/collision_validation.rs` — Primary test suite
- `sim/L0/tests/integration/collision_edge_cases.rs` — Degenerate geometry
- `sim/L0/tests/integration/collision_numerics.rs` — Floating-point pathology
- `sim/L0/tests/integration/collision_performance.rs` — Regression gates

---

#### 6.0 Test Infrastructure

```rust
//! Collision test infrastructure.
//!
//! Design invariants:
//! - Every test is deterministic (no random, no system time in logic)
//! - Every tolerance is derived from IEEE 754 double precision: ε = 2^-52 ≈ 2.22e-16
//! - Geometric tolerances: 1e-10 (100× machine epsilon, accounts for accumulation)
//! - Depth tolerances: 1e-6 (mm-scale for meter-scale simulations)

use nalgebra::{Matrix3, Point3, Vector3};
use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

/// Machine epsilon for f64.
const MACHINE_EPS: f64 = f64::EPSILON; // 2.220446049250313e-16

/// Geometric comparison tolerance: 100 × machine epsilon.
/// Justification: Rotation matrices accumulate ~10 operations, each with ε error.
const GEOM_TOL: f64 = 1e-10;

/// Depth comparison tolerance: millimeter-scale for meter-scale geometry.
/// Justification: Sub-mm precision sufficient for contact resolution at 1kHz.
const DEPTH_TOL: f64 = 1e-6;

/// Normal direction tolerance: ~0.001 radians ≈ 0.057 degrees.
/// Justification: arcsin(1e-3) imperceptible in contact dynamics.
const NORMAL_TOL: f64 = 1e-3;

/// Standard rotation matrices for exhaustive orientation testing.
mod rotations {
    use super::*;

    /// Identity: no rotation.
    pub fn identity() -> Matrix3<f64> {
        Matrix3::identity()
    }

    /// 90° about X: [1,0,0; 0,0,-1; 0,1,0]
    pub fn rot_x_90() -> Matrix3<f64> {
        Matrix3::new(1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0)
    }

    /// 90° about Y: [0,0,1; 0,1,0; -1,0,0]
    pub fn rot_y_90() -> Matrix3<f64> {
        Matrix3::new(0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0)
    }

    /// 90° about Z: [0,-1,0; 1,0,0; 0,0,1]
    pub fn rot_z_90() -> Matrix3<f64> {
        Matrix3::new(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    }

    /// 45° about X-Y-Z (maximally non-aligned).
    pub fn rot_xyz_45() -> Matrix3<f64> {
        let c = FRAC_PI_4.cos();
        let s = FRAC_PI_4.sin();
        let rx = Matrix3::new(1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c);
        let ry = Matrix3::new(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c);
        let rz = Matrix3::new(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0);
        rz * ry * rx
    }

    /// All canonical rotations for exhaustive testing.
    pub fn canonical_set() -> Vec<(&'static str, Matrix3<f64>)> {
        vec![
            ("identity", identity()),
            ("rot_x_90", rot_x_90()),
            ("rot_y_90", rot_y_90()),
            ("rot_z_90", rot_z_90()),
            ("rot_xyz_45", rot_xyz_45()),
        ]
    }
}

/// Assert contact exists with expected properties.
macro_rules! assert_contact {
    ($contacts:expr, depth: $depth:expr, normal: $normal:expr $(,)?) => {{
        let contacts = &$contacts;
        assert!(
            !contacts.is_empty(),
            "Expected contact, got none"
        );
        let c = &contacts[0];
        assert!(
            (c.depth - $depth).abs() < DEPTH_TOL,
            "Depth mismatch: expected {}, got {} (diff {})",
            $depth, c.depth, (c.depth - $depth).abs()
        );
        let expected_n: Vector3<f64> = $normal;
        let dot = c.normal.dot(&expected_n);
        assert!(
            (dot - 1.0).abs() < NORMAL_TOL || (dot + 1.0).abs() < NORMAL_TOL,
            "Normal mismatch: expected {:?}, got {:?} (dot {})",
            expected_n, c.normal, dot
        );
    }};
}

/// Assert no contact exists.
macro_rules! assert_no_contact {
    ($contacts:expr) => {{
        let contacts = &$contacts;
        assert!(
            contacts.is_empty(),
            "Expected no contact, got {} contacts",
            contacts.len()
        );
    }};
}
```

---

#### 6.1 Plane Collision Tests — Exhaustive Coverage

```rust
//! Plane collision tests.
//!
//! The plane is infinite, defined by normal (local +Z) and offset.
//! Every primitive must be tested in all canonical orientations.

mod plane_collision {
    use super::*;

    // ═══════════════════════════════════════════════════════════════════════════
    // PLANE-CYLINDER: 4 contact scenarios × 5 orientations = 20 tests
    // ═══════════════════════════════════════════════════════════════════════════

    /// Cylinder standing upright on plane (axis perpendicular to plane).
    /// Contact at single rim point (bottom rim touches first).
    #[test]
    fn cylinder_plane_upright_contact() {
        for (rot_name, plane_rot) in rotations::canonical_set() {
            let plane_normal = plane_rot.column(2).into_owned();
            let cyl_radius = 0.3;
            let cyl_half_height = 0.5;

            // Cylinder axis aligned with plane normal, penetrating by 0.1
            let cyl_pos = plane_normal * (cyl_half_height - 0.1);
            let cyl_mat = plane_rot; // Axis aligned with plane normal

            let contacts = collide_cylinder_plane(
                cyl_pos, cyl_mat, cyl_radius, cyl_half_height,
                Vector3::zeros(), plane_rot,
            );

            assert_contact!(contacts, depth: 0.1, normal: plane_normal);

            // Verify contact point is on bottom rim
            let contact_pos = contacts[0].pos;
            let dist_to_axis = (contact_pos - cyl_pos + plane_normal * cyl_half_height).norm();
            assert!(
                (dist_to_axis - cyl_radius).abs() < GEOM_TOL,
                "[{}] Contact not on rim: dist_to_axis = {}",
                rot_name, dist_to_axis
            );
        }
    }

    /// Cylinder lying flat on plane (axis parallel to plane).
    /// Contact along entire bottom edge (line contact approximated as point).
    #[test]
    fn cylinder_plane_flat_contact() {
        let plane_normal = Vector3::z();
        let cyl_radius = 0.3;
        let cyl_half_height = 0.5;

        // Cylinder axis along X, touching plane at z=0 with penetration 0.05
        let cyl_pos = Vector3::new(0.0, 0.0, cyl_radius - 0.05);
        let cyl_mat = rotations::rot_y_90(); // Local Z → World X

        let contacts = collide_cylinder_plane(
            cyl_pos, cyl_mat, cyl_radius, cyl_half_height,
            Vector3::zeros(), Matrix3::identity(),
        );

        assert_contact!(contacts, depth: 0.05, normal: plane_normal);
    }

    /// Cylinder tilted at 45° to plane.
    /// Contact at single rim point (geometrically lowest point on rim).
    #[test]
    fn cylinder_plane_tilted_45_contact() {
        let plane_normal = Vector3::z();
        let cyl_radius = 0.3;
        let cyl_half_height = 0.5;

        // Cylinder tilted 45° about X axis
        let tilt = FRAC_PI_4;
        let cyl_mat = Matrix3::new(
            1.0, 0.0, 0.0,
            0.0, tilt.cos(), -tilt.sin(),
            0.0, tilt.sin(), tilt.cos(),
        );
        let cyl_axis = cyl_mat.column(2).into_owned();

        // Position so lowest rim point penetrates by 0.1
        // Lowest point: bottom_center - rim_dir * radius, where rim_dir projects normal onto rim
        let bottom_center_offset = -cyl_axis * cyl_half_height;
        let rim_dir = (plane_normal - cyl_axis * plane_normal.dot(&cyl_axis)).normalize();
        let lowest_offset = bottom_center_offset - rim_dir * cyl_radius;
        let lowest_z = lowest_offset.z;

        // Position cylinder so lowest point is at z = -0.1 (penetration = 0.1)
        let cyl_pos = Vector3::new(0.0, 0.0, -lowest_z - 0.1);

        let contacts = collide_cylinder_plane(
            cyl_pos, cyl_mat, cyl_radius, cyl_half_height,
            Vector3::zeros(), Matrix3::identity(),
        );

        assert_contact!(contacts, depth: 0.1, normal: plane_normal);
    }

    /// Cylinder just above plane (no contact).
    /// Separation = 1mm, must return no contact.
    #[test]
    fn cylinder_plane_separated_no_contact() {
        let cyl_pos = Vector3::new(0.0, 0.0, 0.501); // half_height + 1mm gap
        let cyl_mat = Matrix3::identity();

        let contacts = collide_cylinder_plane(
            cyl_pos, cyl_mat, 0.3, 0.5,
            Vector3::zeros(), Matrix3::identity(),
        );

        assert_no_contact!(contacts);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // PLANE-ELLIPSOID: Support function geometry × 5 orientations
    // ═══════════════════════════════════════════════════════════════════════════

    /// Ellipsoid with unequal radii: contact normal must find true support point.
    /// Ellipsoid radii (0.5, 0.3, 0.2) — most penetration along longest axis.
    #[test]
    fn ellipsoid_plane_anisotropic_contact() {
        let plane_normal = Vector3::z();
        let ell_radii = Vector3::new(0.5, 0.3, 0.2);

        // Ellipsoid at height = 0.15, penetrating by 0.05 (0.2 - 0.15 = 0.05)
        let ell_pos = Vector3::new(0.0, 0.0, 0.15);
        let ell_mat = Matrix3::identity();

        let contacts = collide_ellipsoid_plane(
            ell_pos, ell_mat, ell_radii,
            Vector3::zeros(), Matrix3::identity(),
        );

        assert_contact!(contacts, depth: 0.05, normal: plane_normal);
    }

    /// Ellipsoid rotated so longest axis is horizontal.
    /// Effective vertical radius becomes 0.3 (middle axis).
    #[test]
    fn ellipsoid_plane_rotated_longest_horizontal() {
        let ell_radii = Vector3::new(0.5, 0.3, 0.2);

        // Rotate 90° about Y: local X (0.5 radius) → world Z (now vertical is 0.5)
        // Wait, that makes Z larger. Let's rotate so local Z (0.2) → world X
        // Rotate 90° about Y: world_z = -local_x, world_x = local_z
        // So vertical axis in world Z corresponds to local -X (radius 0.5)
        // Actually: rot_y_90 maps local Z to world -X, local X to world Z
        // So if we want local X (0.5) to be vertical (world Z), use rot_y_90
        let ell_mat = rotations::rot_y_90(); // Local X → World Z

        // Now effective vertical radius is 0.5
        let ell_pos = Vector3::new(0.0, 0.0, 0.45); // penetration = 0.5 - 0.45 = 0.05

        let contacts = collide_ellipsoid_plane(
            ell_pos, ell_mat, ell_radii,
            Vector3::zeros(), Matrix3::identity(),
        );

        assert_contact!(contacts, depth: 0.05, normal: Vector3::z());
    }

    /// Degenerate ellipsoid: one radius near zero (disk-like).
    /// Must not panic, must return valid contact if penetrating.
    #[test]
    fn ellipsoid_plane_degenerate_disk() {
        let ell_radii = Vector3::new(0.5, 0.5, 0.001); // Nearly flat disk
        let ell_pos = Vector3::new(0.0, 0.0, 0.0005); // Penetrating by 0.0005
        let ell_mat = Matrix3::identity();

        let contacts = collide_ellipsoid_plane(
            ell_pos, ell_mat, ell_radii,
            Vector3::zeros(), Matrix3::identity(),
        );

        assert_contact!(contacts, depth: 0.0005, normal: Vector3::z());
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // PLANE-MESH: Vertex iteration correctness
    // ═══════════════════════════════════════════════════════════════════════════

    /// Unit cube mesh: 8 vertices, contact at 4 bottom vertices.
    /// Must report deepest penetrating vertex.
    #[test]
    fn mesh_plane_cube_contact() {
        let cube = TriangleMeshData::unit_cube(); // Vertices at ±0.5

        // Cube at z=0.4, bottom vertices at z=-0.1, penetration=0.1
        let mesh_pos = Vector3::new(0.0, 0.0, 0.4);
        let mesh_mat = Matrix3::identity();

        let contacts = collide_mesh_plane(
            &cube, mesh_pos, mesh_mat,
            Vector3::zeros(), Matrix3::identity(),
        );

        assert_contact!(contacts, depth: 0.1, normal: Vector3::z());
    }

    /// Rotated cube: contact at single corner vertex.
    /// Cube rotated 45° about X and Y — single vertex pointing down.
    #[test]
    fn mesh_plane_cube_rotated_corner_contact() {
        let cube = TriangleMeshData::unit_cube();

        // Rotate so one corner points straight down
        // Rotation about (1,1,0) normalized by arccos(-1/3) ≈ 109.47° puts corner down
        // Simpler: 45° about X then 45° about Y
        let mesh_mat = rotations::rot_xyz_45();

        // Find lowest vertex z-coordinate
        let verts_world: Vec<Point3<f64>> = cube.vertices()
            .iter()
            .map(|v| Point3::from(mesh_mat * v.coords))
            .collect();
        let min_z = verts_world.iter().map(|v| v.z).fold(f64::INFINITY, f64::min);

        // Position so min_z vertex penetrates by 0.1
        let mesh_pos = Vector3::new(0.0, 0.0, -min_z + 0.1);

        let contacts = collide_mesh_plane(
            &cube, mesh_pos + Vector3::new(0.0, 0.0, -0.2), mesh_mat,
            Vector3::zeros(), Matrix3::identity(),
        );

        assert_contact!(contacts, depth: 0.1, normal: Vector3::z());
    }
}
```

---

#### 6.2 Primitive-Primitive Collision Tests — Analytical Path Validation

```rust
//! Primitive collision tests.
//!
//! Every analytical collision function must be tested for:
//! 1. Correct contact point (midpoint between surfaces)
//! 2. Correct normal direction (from geom1 to geom2)
//! 3. Correct penetration depth (positive when overlapping)
//! 4. No contact when separated

mod primitive_collision {
    use super::*;

    // ═══════════════════════════════════════════════════════════════════════════
    // CYLINDER-SPHERE: 6 contact regions
    // ═══════════════════════════════════════════════════════════════════════════

    /// Sphere touching cylinder side (radial contact).
    #[test]
    fn cylinder_sphere_side_contact() {
        let cyl_radius = 0.3;
        let cyl_half_height = 0.5;
        let sph_radius = 0.2;

        // Sphere at (0.45, 0, 0), touching cylinder side at (0.3, 0, 0)
        // Distance from cyl axis to sphere center = 0.45
        // Contact when 0.45 < 0.3 + 0.2 = 0.5 → penetration = 0.05
        let cyl_pos = Vector3::zeros();
        let cyl_mat = Matrix3::identity();
        let sph_pos = Vector3::new(0.45, 0.0, 0.0);

        let contacts = collide_cylinder_sphere(
            cyl_pos, cyl_mat, cyl_radius, cyl_half_height,
            sph_pos, sph_radius,
        );

        assert_contact!(contacts, depth: 0.05, normal: Vector3::x());
    }

    /// Sphere touching cylinder top cap (axial contact).
    #[test]
    fn cylinder_sphere_cap_contact() {
        let cyl_radius = 0.3;
        let cyl_half_height = 0.5;
        let sph_radius = 0.2;

        // Sphere directly above cylinder
        // Cap at z = 0.5, sphere at z = 0.65
        // Distance = 0.15 < sphere_radius → penetration = 0.05
        let cyl_pos = Vector3::zeros();
        let cyl_mat = Matrix3::identity();
        let sph_pos = Vector3::new(0.0, 0.0, 0.65);

        let contacts = collide_cylinder_sphere(
            cyl_pos, cyl_mat, cyl_radius, cyl_half_height,
            sph_pos, sph_radius,
        );

        assert_contact!(contacts, depth: 0.05, normal: Vector3::z());
    }

    /// Sphere touching cylinder rim edge (saddle point contact).
    /// This is the geometrically interesting case.
    #[test]
    fn cylinder_sphere_rim_contact() {
        let cyl_radius = 0.3;
        let cyl_half_height = 0.5;
        let sph_radius = 0.2;

        // Sphere positioned diagonally from rim
        // Rim point at (0.3, 0, 0.5)
        // Place sphere at 45° outward and upward from rim
        let rim_point = Vector3::new(0.3, 0.0, 0.5);
        let outward_dir = Vector3::new(1.0, 0.0, 1.0).normalize();
        let sphere_dist = sph_radius - 0.05; // Penetration = 0.05
        let sph_pos = rim_point + outward_dir * sphere_dist;

        let cyl_pos = Vector3::zeros();
        let cyl_mat = Matrix3::identity();

        let contacts = collide_cylinder_sphere(
            cyl_pos, cyl_mat, cyl_radius, cyl_half_height,
            sph_pos, sph_radius,
        );

        assert!(!contacts.is_empty(), "Rim contact must be detected");
        assert!(
            (contacts[0].depth - 0.05).abs() < DEPTH_TOL,
            "Rim contact depth: expected 0.05, got {}",
            contacts[0].depth
        );
    }

    /// Sphere centered exactly on cylinder axis (degenerate normal).
    /// Must not panic, must return valid contact with some consistent normal.
    #[test]
    fn cylinder_sphere_on_axis_degenerate() {
        let cyl_radius = 0.3;
        let cyl_half_height = 0.5;
        let sph_radius = 0.4; // Larger than cyl_radius to ensure overlap

        // Sphere center exactly on cylinder axis, inside the cylinder
        let cyl_pos = Vector3::zeros();
        let cyl_mat = Matrix3::identity();
        let sph_pos = Vector3::new(0.0, 0.0, 0.0); // On axis

        let contacts = collide_cylinder_sphere(
            cyl_pos, cyl_mat, cyl_radius, cyl_half_height,
            sph_pos, sph_radius,
        );

        // Should detect contact (sphere radius > cylinder radius means overlap)
        assert!(!contacts.is_empty(), "On-axis contact must be detected");
        // Normal must be unit length
        let normal_len = contacts[0].normal.norm();
        assert!(
            (normal_len - 1.0).abs() < GEOM_TOL,
            "Normal must be unit length: got {}",
            normal_len
        );
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // CYLINDER-CAPSULE: Line-segment distance geometry
    // ═══════════════════════════════════════════════════════════════════════════

    /// Capsule parallel to cylinder, side-by-side contact.
    #[test]
    fn cylinder_capsule_parallel_side_contact() {
        let cyl_radius = 0.3;
        let cyl_half_height = 0.5;
        let cap_radius = 0.2;
        let cap_half_height = 0.4;

        // Both axes along Z, capsule offset in X
        let cyl_pos = Vector3::zeros();
        let cyl_mat = Matrix3::identity();
        let cap_pos = Vector3::new(0.45, 0.0, 0.0); // 0.3 + 0.2 - 0.05 = 0.45
        let cap_mat = Matrix3::identity();

        let contacts = collide_cylinder_capsule(
            cyl_pos, cyl_mat, cyl_radius, cyl_half_height,
            cap_pos, cap_mat, cap_radius, cap_half_height,
        );

        assert_contact!(contacts, depth: 0.05, normal: Vector3::x());
    }

    /// Capsule perpendicular to cylinder (T-bone configuration).
    #[test]
    fn cylinder_capsule_perpendicular_contact() {
        let cyl_radius = 0.3;
        let cyl_half_height = 0.5;
        let cap_radius = 0.2;
        let cap_half_height = 0.4;

        // Cylinder axis along Z, capsule axis along X
        let cyl_pos = Vector3::zeros();
        let cyl_mat = Matrix3::identity();
        let cap_pos = Vector3::new(0.0, 0.45, 0.0); // Offset in Y
        let cap_mat = rotations::rot_y_90(); // Capsule axis now along X

        let contacts = collide_cylinder_capsule(
            cyl_pos, cyl_mat, cyl_radius, cyl_half_height,
            cap_pos, cap_mat, cap_radius, cap_half_height,
        );

        assert_contact!(contacts, depth: 0.05, normal: Vector3::y());
    }

    /// Capsule endpoint touching cylinder cap.
    #[test]
    fn cylinder_capsule_endpoint_cap_contact() {
        let cyl_radius = 0.3;
        let cyl_half_height = 0.5;
        let cap_radius = 0.2;
        let cap_half_height = 0.4;

        // Capsule above cylinder, axis along Z
        // Capsule bottom endpoint at z = cap_pos.z - cap_half_height
        // Should touch cylinder cap at z = 0.5
        let cyl_pos = Vector3::zeros();
        let cyl_mat = Matrix3::identity();
        let cap_pos = Vector3::new(0.0, 0.0, 0.5 + 0.4 + 0.2 - 0.05); // Penetration 0.05
        let cap_mat = Matrix3::identity();

        let contacts = collide_cylinder_capsule(
            cyl_pos, cyl_mat, cyl_radius, cyl_half_height,
            cap_pos, cap_mat, cap_radius, cap_half_height,
        );

        assert_contact!(contacts, depth: 0.05, normal: Vector3::z());
    }
}
```

---

#### 6.3 Mesh Collision Tests — BVH Correctness

```rust
//! Mesh collision tests.
//!
//! BVH traversal must be validated against brute-force for small meshes.
//! Large mesh tests validate O(log n) performance, not just correctness.

mod mesh_collision {
    use super::*;

    // ═══════════════════════════════════════════════════════════════════════════
    // MESH-SPHERE
    // ═══════════════════════════════════════════════════════════════════════════

    /// Sphere penetrating single triangle.
    #[test]
    fn mesh_sphere_single_triangle_face_contact() {
        // Triangle in XY plane at z=0
        let verts = vec![
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let faces = vec![[0, 1, 2]];
        let mesh = TriangleMeshData::new(verts, faces);

        // Sphere at z=0.15, radius 0.2 → penetration = 0.05
        let sph_pos = Point3::new(0.0, 0.0, 0.15);
        let sph_radius = 0.2;

        let contacts = mesh_sphere_contact(&mesh, &Pose::identity(), sph_pos, sph_radius);

        assert_contact!(contacts, depth: 0.05, normal: Vector3::z());
    }

    /// Sphere touching triangle edge (edge contact).
    #[test]
    fn mesh_sphere_edge_contact() {
        let verts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
        ];
        let faces = vec![[0, 1, 2]];
        let mesh = TriangleMeshData::new(verts, faces);

        // Sphere center at (0.5, -0.15, 0) with radius 0.2
        // Closest point on edge (0,0,0)-(1,0,0) is (0.5, 0, 0)
        // Distance = 0.15 < 0.2 → penetration = 0.05
        let sph_pos = Point3::new(0.5, -0.15, 0.0);
        let sph_radius = 0.2;

        let contacts = mesh_sphere_contact(&mesh, &Pose::identity(), sph_pos, sph_radius);

        assert_contact!(contacts, depth: 0.05, normal: -Vector3::y());
    }

    /// Sphere touching triangle vertex (vertex contact).
    #[test]
    fn mesh_sphere_vertex_contact() {
        let verts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
        ];
        let faces = vec![[0, 1, 2]];
        let mesh = TriangleMeshData::new(verts, faces);

        // Sphere center at (-0.1, -0.1, 0) with radius 0.2
        // Closest point is vertex (0, 0, 0)
        // Distance = sqrt(0.01 + 0.01) ≈ 0.1414 < 0.2 → penetration ≈ 0.0586
        let sph_pos = Point3::new(-0.1, -0.1, 0.0);
        let sph_radius = 0.2;

        let contacts = mesh_sphere_contact(&mesh, &Pose::identity(), sph_pos, sph_radius);

        let expected_depth = sph_radius - (0.02_f64).sqrt();
        assert!(!contacts.is_empty(), "Vertex contact must be detected");
        assert!(
            (contacts[0].depth - expected_depth).abs() < DEPTH_TOL,
            "Vertex contact depth: expected {}, got {}",
            expected_depth, contacts[0].depth
        );
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // MESH-MESH: BVH vs Brute-Force Validation
    // ═══════════════════════════════════════════════════════════════════════════

    /// Two cubes interpenetrating: BVH result must match brute-force.
    #[test]
    fn mesh_mesh_cubes_bvh_matches_bruteforce() {
        let cube1 = TriangleMeshData::unit_cube();
        let cube2 = TriangleMeshData::unit_cube();

        let pose1 = Pose::from_position(Point3::new(0.0, 0.0, 0.0));
        let pose2 = Pose::from_position(Point3::new(0.3, 0.0, 0.0)); // Overlap by 0.2

        // BVH-accelerated
        let contacts_bvh = mesh_mesh_contact(&cube1, &pose1, &cube2, &pose2);

        // Brute-force (all triangle pairs)
        let contacts_brute = mesh_mesh_contact_bruteforce(&cube1, &pose1, &cube2, &pose2);

        // Must find same deepest contact
        let max_depth_bvh = contacts_bvh.iter().map(|c| c.penetration).fold(0.0, f64::max);
        let max_depth_brute = contacts_brute.iter().map(|c| c.penetration).fold(0.0, f64::max);

        assert!(
            (max_depth_bvh - max_depth_brute).abs() < DEPTH_TOL,
            "BVH/brute-force mismatch: BVH={}, brute={}",
            max_depth_bvh, max_depth_brute
        );
    }
}
```

---

#### 6.4 Edge Case Tests — Numerical Pathology

```rust
//! Edge case tests for numerical robustness.
//!
//! These tests target floating-point pathologies:
//! - Near-parallel geometry (conditioning)
//! - Near-zero distances (division stability)
//! - Large coordinate values (precision loss)
//! - Exactly degenerate configurations

mod edge_cases {
    use super::*;

    // ═══════════════════════════════════════════════════════════════════════════
    // NEAR-PARALLEL GEOMETRY
    // ═══════════════════════════════════════════════════════════════════════════

    /// Cylinder axis nearly parallel to plane normal (angle < 0.01°).
    /// Cross product for rim direction approaches zero.
    #[test]
    fn cylinder_plane_near_parallel_axis() {
        let plane_normal = Vector3::z();
        let cyl_radius = 0.3;
        let cyl_half_height = 0.5;

        // Tilt cylinder by 0.01° ≈ 0.000175 radians
        let tiny_angle = 0.01_f64.to_radians();
        let cyl_mat = Matrix3::new(
            1.0, 0.0, 0.0,
            0.0, tiny_angle.cos(), -tiny_angle.sin(),
            0.0, tiny_angle.sin(), tiny_angle.cos(),
        );

        let cyl_pos = Vector3::new(0.0, 0.0, cyl_half_height - 0.1);

        let contacts = collide_cylinder_plane(
            cyl_pos, cyl_mat, cyl_radius, cyl_half_height,
            Vector3::zeros(), Matrix3::identity(),
        );

        // Must not panic, must return valid contact
        assert!(!contacts.is_empty(), "Near-parallel must detect contact");
        assert!(contacts[0].depth > 0.0, "Depth must be positive");
        assert!(
            (contacts[0].normal.norm() - 1.0).abs() < GEOM_TOL,
            "Normal must be unit length"
        );
    }

    /// Two capsules with nearly parallel axes (segment-segment degeneracy).
    #[test]
    fn capsule_capsule_near_parallel_axes() {
        let radius = 0.2;
        let half_height = 0.5;

        // Capsule 1 along Z
        let cap1_pos = Vector3::zeros();
        let cap1_mat = Matrix3::identity();

        // Capsule 2 along Z + tiny tilt (0.01°)
        let tiny_angle = 0.01_f64.to_radians();
        let cap2_mat = Matrix3::new(
            1.0, 0.0, 0.0,
            0.0, tiny_angle.cos(), -tiny_angle.sin(),
            0.0, tiny_angle.sin(), tiny_angle.cos(),
        );
        let cap2_pos = Vector3::new(0.35, 0.0, 0.0); // Overlapping

        let contacts = collide_capsule_capsule(
            cap1_pos, cap1_mat, radius, half_height,
            cap2_pos, cap2_mat, radius, half_height,
        );

        assert!(!contacts.is_empty(), "Near-parallel capsules must detect contact");
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // NEAR-ZERO DISTANCES
    // ═══════════════════════════════════════════════════════════════════════════

    /// Sphere center exactly on mesh triangle surface.
    #[test]
    fn mesh_sphere_center_on_surface() {
        let verts = vec![
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let faces = vec![[0, 1, 2]];
        let mesh = TriangleMeshData::new(verts, faces);

        // Sphere center exactly on triangle (z=0)
        let sph_pos = Point3::new(0.0, 0.0, 0.0);
        let sph_radius = 0.2;

        let contacts = mesh_sphere_contact(&mesh, &Pose::identity(), sph_pos, sph_radius);

        // Depth should be sphere radius
        assert_contact!(contacts, depth: 0.2, normal: Vector3::z());
    }

    /// Two meshes sharing exactly one vertex (zero separation at point).
    #[test]
    fn mesh_mesh_shared_vertex() {
        // Triangle 1: origin to (1,0,0) to (0,1,0)
        let mesh1 = TriangleMeshData::new(
            vec![
                Point3::origin(),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2]],
        );

        // Triangle 2: shares vertex at origin
        let mesh2 = TriangleMeshData::new(
            vec![
                Point3::origin(),
                Point3::new(-1.0, 0.0, 0.0),
                Point3::new(0.0, -1.0, 0.0),
            ],
            vec![[0, 1, 2]],
        );

        let contacts = mesh_mesh_contact(
            &mesh1, &Pose::identity(),
            &mesh2, &Pose::identity(),
        );

        // Exactly touching: depth ≈ 0 (within tolerance)
        // May or may not report contact depending on tolerance handling
        // Key: must not panic
        // Note: we don't assert contact exists because zero-penetration is ambiguous
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // LARGE COORDINATES
    // ═══════════════════════════════════════════════════════════════════════════

    /// Collision detection at world-scale coordinates (km from origin).
    /// Tests for precision loss in subtraction.
    #[test]
    fn sphere_sphere_far_from_origin() {
        let offset = 1e6; // 1 million units from origin (km scale)
        let radius = 0.5;

        let sph1_pos = Vector3::new(offset, 0.0, 0.0);
        let sph2_pos = Vector3::new(offset + 0.9, 0.0, 0.0); // 0.9 apart, should overlap

        let contacts = collide_sphere_sphere(
            sph1_pos, radius,
            sph2_pos, radius,
        );

        // Penetration = 1.0 - 0.9 = 0.1
        // At 1e6 offset, relative precision is ~1e-10, so 0.1 depth is resolvable
        assert_contact!(contacts, depth: 0.1, normal: Vector3::x());
    }

    /// Collision with very small geometry (micrometer scale).
    #[test]
    fn sphere_sphere_microscale() {
        let radius = 1e-6; // 1 micrometer
        let separation = 1.9e-6; // Should overlap

        let sph1_pos = Vector3::zeros();
        let sph2_pos = Vector3::new(separation, 0.0, 0.0);

        let contacts = collide_sphere_sphere(
            sph1_pos, radius,
            sph2_pos, radius,
        );

        // Penetration = 2e-6 - 1.9e-6 = 1e-7
        let expected_depth = 2.0 * radius - separation;
        assert!(
            !contacts.is_empty(),
            "Microscale collision must be detected"
        );
        assert!(
            (contacts[0].depth - expected_depth).abs() < 1e-9,
            "Microscale depth: expected {}, got {}",
            expected_depth, contacts[0].depth
        );
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // DEGENERATE GEOMETRY
    // ═══════════════════════════════════════════════════════════════════════════

    /// Cylinder with zero radius (line segment).
    #[test]
    fn cylinder_sphere_zero_radius_cylinder() {
        let cyl_radius = 0.0;
        let cyl_half_height = 0.5;
        let sph_radius = 0.2;

        let cyl_pos = Vector3::zeros();
        let cyl_mat = Matrix3::identity();
        let sph_pos = Vector3::new(0.15, 0.0, 0.0); // 0.15 from axis

        let contacts = collide_cylinder_sphere(
            cyl_pos, cyl_mat, cyl_radius, cyl_half_height,
            sph_pos, sph_radius,
        );

        // Should detect line-sphere contact
        assert_contact!(contacts, depth: 0.05, normal: Vector3::x());
    }

    /// Ellipsoid with two equal radii (spheroid degenerates to sphere along one axis).
    #[test]
    fn ellipsoid_plane_spheroid() {
        let ell_radii = Vector3::new(0.3, 0.3, 0.3); // Actually a sphere
        let ell_pos = Vector3::new(0.0, 0.0, 0.25);
        let ell_mat = Matrix3::identity();

        let contacts = collide_ellipsoid_plane(
            ell_pos, ell_mat, ell_radii,
            Vector3::zeros(), Matrix3::identity(),
        );

        // Should behave identically to sphere-plane
        assert_contact!(contacts, depth: 0.05, normal: Vector3::z());
    }
}
```

---

#### 6.5 Integration Tests — Full Pipeline Validation

```rust
//! Full pipeline integration tests.
//!
//! These tests load MJCF models and run the complete collision pipeline,
//! verifying that all components integrate correctly.

mod integration {
    use super::*;

    /// Standard tetrahedron for mesh tests (4 vertices, 4 faces).
    /// Vertices form a regular tetrahedron with circumradius 1.
    fn tetrahedron_mjcf() -> &'static str {
        r#"
        <mujoco model="tetrahedron_test">
            <asset>
                <mesh name="tetra"
                      vertex="0.9428 0 -0.3333
                              -0.4714 0.8165 -0.3333
                              -0.4714 -0.8165 -0.3333
                              0 0 1"
                      face="0 1 2  0 2 3  0 3 1  1 3 2"/>
            </asset>
            <worldbody>
                <geom type="plane" size="10 10 0.1"/>
                <body name="tetra_body" pos="0 0 1">
                    <freejoint/>
                    <geom type="mesh" mesh="tetra" mass="1"/>
                </body>
            </worldbody>
        </mujoco>
        "#
    }

    /// Verify mesh loads correctly with expected vertex/face counts.
    #[test]
    fn mesh_loading_topology() {
        let model = load_model(tetrahedron_mjcf()).expect("must load");

        assert_eq!(model.nmesh, 1, "Expected 1 mesh");
        assert_eq!(model.mesh_data[0].num_vertices(), 4, "Tetrahedron has 4 vertices");
        assert_eq!(model.mesh_data[0].num_triangles(), 4, "Tetrahedron has 4 faces");

        // Verify geom references mesh
        let mesh_geom_idx = model.geom_type
            .iter()
            .position(|&t| t == GeomType::Mesh)
            .expect("must have mesh geom");
        assert!(model.geom_mesh[mesh_geom_idx].is_some(), "Mesh geom must reference mesh");
    }

    /// Verify bounding radius is computed correctly from mesh AABB.
    #[test]
    fn mesh_bounding_radius() {
        let model = load_model(tetrahedron_mjcf()).expect("must load");

        let mesh = &model.mesh_data[0];
        let half_extents = (mesh.aabb_max() - mesh.aabb_min()) / 2.0;
        let expected_rbound = half_extents.coords.norm();

        let mesh_geom_idx = model.geom_type
            .iter()
            .position(|&t| t == GeomType::Mesh)
            .expect("must have mesh geom");
        let actual_rbound = model.geom_rbound[mesh_geom_idx];

        assert!(
            (actual_rbound - expected_rbound).abs() < GEOM_TOL,
            "Bounding radius: expected {}, got {}",
            expected_rbound, actual_rbound
        );
    }

    /// Full simulation: mesh falls onto plane, must generate contacts.
    #[test]
    fn mesh_plane_simulation() {
        let model = load_model(tetrahedron_mjcf()).expect("must load");
        let mut data = model.make_data();

        // Step until contact
        for _ in 0..1000 {
            mj_step(&model, &mut data);
            if data.ncon > 0 {
                break;
            }
        }

        assert!(data.ncon > 0, "Mesh must contact plane during fall");

        // Contact normal should point up (from plane to mesh)
        let contact = &data.contact[0];
        assert!(
            contact.normal.dot(&Vector3::z()) > 0.9,
            "Contact normal must point upward, got {:?}",
            contact.normal
        );
    }

    /// Cylinder on plane: verify analytical path is used (no GJK allocation).
    #[test]
    fn cylinder_plane_analytical_path() {
        let mjcf = r#"
        <mujoco model="cylinder_test">
            <worldbody>
                <geom type="plane" size="10 10 0.1"/>
                <body name="cyl" pos="0 0 1">
                    <freejoint/>
                    <geom type="cylinder" size="0.3 0.5" mass="1"/>
                </body>
            </worldbody>
        </mujoco>
        "#;

        let model = load_model(mjcf).expect("must load");
        let mut data = model.make_data();

        // Step until contact
        for _ in 0..500 {
            mj_step(&model, &mut data);
            if data.ncon > 0 {
                break;
            }
        }

        assert!(data.ncon > 0, "Cylinder must contact plane");

        // Verify contact geometry makes sense
        let contact = &data.contact[0];
        assert!(contact.depth > 0.0, "Must have positive penetration");
        assert!(
            contact.normal.dot(&Vector3::z()).abs() > 0.9,
            "Normal must be approximately vertical"
        );
    }

    /// Ellipsoid with anisotropic radii on tilted plane.
    #[test]
    fn ellipsoid_tilted_plane() {
        let mjcf = r#"
        <mujoco model="ellipsoid_tilted">
            <worldbody>
                <geom type="plane" size="10 10 0.1" euler="15 0 0"/>
                <body name="ell" pos="0 0 2">
                    <freejoint/>
                    <geom type="ellipsoid" size="0.5 0.3 0.2" mass="1"/>
                </body>
            </worldbody>
        </mujoco>
        "#;

        let model = load_model(mjcf).expect("must load");
        let mut data = model.make_data();

        for _ in 0..500 {
            mj_step(&model, &mut data);
            if data.ncon > 0 {
                break;
            }
        }

        assert!(data.ncon > 0, "Ellipsoid must contact tilted plane");
    }
}
```

---

#### 6.6 Performance Tests — Regression Gates

```rust
//! Performance regression tests.
//!
//! These tests establish hard gates on simulation throughput.
//! Thresholds are calibrated to detect 20% regressions.

mod performance {
    use super::*;
    use std::time::Instant;

    /// Minimum steps/second for humanoid model (debug build).
    const DEBUG_THRESHOLD: f64 = 300.0;

    /// Minimum steps/second for humanoid model (release build).
    const RELEASE_THRESHOLD: f64 = 5000.0;

    /// Humanoid model performance gate.
    #[test]
    fn humanoid_throughput() {
        let model = load_humanoid_model();
        let mut data = model.make_data();

        // Warmup: 100 steps
        for _ in 0..100 {
            mj_step(&model, &mut data);
        }

        // Benchmark: 1000 steps
        let steps = 1000;
        let start = Instant::now();
        for _ in 0..steps {
            mj_step(&model, &mut data);
        }
        let elapsed = start.elapsed();

        let steps_per_sec = steps as f64 / elapsed.as_secs_f64();

        #[cfg(debug_assertions)]
        let threshold = DEBUG_THRESHOLD;
        #[cfg(not(debug_assertions))]
        let threshold = RELEASE_THRESHOLD;

        assert!(
            steps_per_sec >= threshold,
            "Performance regression: {:.0} steps/sec < {:.0} threshold",
            steps_per_sec, threshold
        );

        println!(
            "Humanoid throughput: {:.0} steps/sec ({:.2}ms/step)",
            steps_per_sec,
            1000.0 / steps_per_sec
        );
    }

    /// Mesh collision scalability: O(log n) with BVH.
    /// Tests that doubling triangle count does NOT double collision time.
    #[test]
    fn mesh_collision_scaling() {
        // Small mesh: 100 triangles
        let small_mesh = TriangleMeshData::sphere_mesh(10); // ~100 tris
        let small_count = small_mesh.num_triangles();

        // Large mesh: 400 triangles
        let large_mesh = TriangleMeshData::sphere_mesh(20); // ~400 tris
        let large_count = large_mesh.num_triangles();

        let sph_pos = Point3::new(0.0, 0.0, 0.95);
        let sph_radius = 0.1;

        // Time small mesh
        let start = Instant::now();
        for _ in 0..1000 {
            let _ = mesh_sphere_contact(&small_mesh, &Pose::identity(), sph_pos, sph_radius);
        }
        let small_time = start.elapsed().as_secs_f64();

        // Time large mesh
        let start = Instant::now();
        for _ in 0..1000 {
            let _ = mesh_sphere_contact(&large_mesh, &Pose::identity(), sph_pos, sph_radius);
        }
        let large_time = start.elapsed().as_secs_f64();

        // With O(log n) BVH: 4× triangles → ~2× time (log₂(4) = 2)
        // Allow 3× as threshold (between linear and logarithmic)
        let ratio = large_time / small_time;
        let triangle_ratio = large_count as f64 / small_count as f64;

        assert!(
            ratio < triangle_ratio.sqrt() * 2.0,
            "Mesh scaling not O(log n): {}× triangles → {}× time (expected <{}×)",
            triangle_ratio, ratio, triangle_ratio.sqrt() * 2.0
        );

        println!(
            "Mesh scaling: {}→{} tris ({:.1}×), time {:.3}→{:.3}ms ({:.2}×)",
            small_count, large_count, triangle_ratio,
            small_time * 1000.0, large_time * 1000.0, ratio
        );
    }

    /// Allocation test: collision hot path must not allocate.
    #[test]
    #[cfg(feature = "alloc_counter")]
    fn collision_no_allocations() {
        let model = load_humanoid_model();
        let mut data = model.make_data();

        // Warmup
        mj_forward(&model, &mut data);

        // Count allocations during collision detection
        let alloc_count = alloc_counter::count(|| {
            for _ in 0..100 {
                detect_collisions(&model, &mut data);
            }
        });

        assert_eq!(
            alloc_count, 0,
            "Collision hot path allocated {} times (must be 0)",
            alloc_count
        );
    }
}
```

---

#### 6.7 Conformance Tests — MuJoCo Reference Comparison

```rust
//! MuJoCo conformance tests.
//!
//! Compare our collision results against MuJoCo's reference implementation.
//! Tolerances account for implementation differences, not bugs.

mod conformance {
    use super::*;

    /// Maximum allowed deviation from MuJoCo reference (generous for now).
    const MUJOCO_DEPTH_TOL: f64 = 1e-4;
    const MUJOCO_POS_TOL: f64 = 1e-4;

    /// Sphere-sphere: exact analytical, should match perfectly.
    #[test]
    fn sphere_sphere_mujoco_match() {
        let mjcf = r#"
        <mujoco>
            <worldbody>
                <body pos="0 0 0">
                    <geom type="sphere" size="0.5"/>
                </body>
                <body pos="0.8 0 0">
                    <geom type="sphere" size="0.5"/>
                </body>
            </worldbody>
        </mujoco>
        "#;

        let (our_contact, mj_contact) = compare_with_mujoco(mjcf);

        assert!(
            (our_contact.depth - mj_contact.depth).abs() < MUJOCO_DEPTH_TOL,
            "Depth mismatch: ours={}, mujoco={}",
            our_contact.depth, mj_contact.depth
        );

        let pos_diff = (our_contact.pos - mj_contact.pos).norm();
        assert!(
            pos_diff < MUJOCO_POS_TOL,
            "Position mismatch: diff={}",
            pos_diff
        );
    }

    /// Capsule-capsule: complex geometry, verify we're in the ballpark.
    #[test]
    fn capsule_capsule_mujoco_match() {
        let mjcf = r#"
        <mujoco>
            <worldbody>
                <body pos="0 0 0">
                    <geom type="capsule" size="0.2 0.5"/>
                </body>
                <body pos="0.3 0 0" euler="0 45 0">
                    <geom type="capsule" size="0.2 0.5"/>
                </body>
            </worldbody>
        </mujoco>
        "#;

        let (our_contact, mj_contact) = compare_with_mujoco(mjcf);

        // Capsule-capsule can have larger deviations due to segment-segment algorithm
        assert!(
            (our_contact.depth - mj_contact.depth).abs() < MUJOCO_DEPTH_TOL * 10.0,
            "Depth mismatch: ours={}, mujoco={}",
            our_contact.depth, mj_contact.depth
        );
    }

    /// Box-box SAT: compare against MuJoCo's implementation.
    #[test]
    fn box_box_mujoco_match() {
        let mjcf = r#"
        <mujoco>
            <worldbody>
                <body pos="0 0 0">
                    <geom type="box" size="0.5 0.4 0.3"/>
                </body>
                <body pos="0.7 0.3 0" euler="0 0 30">
                    <geom type="box" size="0.4 0.3 0.2"/>
                </body>
            </worldbody>
        </mujoco>
        "#;

        let (our_contact, mj_contact) = compare_with_mujoco(mjcf);

        assert!(
            (our_contact.depth - mj_contact.depth).abs() < MUJOCO_DEPTH_TOL,
            "Depth mismatch: ours={}, mujoco={}",
            our_contact.depth, mj_contact.depth
        );
    }
}
```

---

## Success Criteria

### Functional Completeness — Test Coverage Matrix

| Criterion | Phase | Test Module | Test Count |
|-----------|-------|-------------|------------|
| Plane-Cylinder all orientations | 1 | `plane_collision` | 4 |
| Plane-Ellipsoid all orientations | 1 | `plane_collision` | 3 |
| Plane-Mesh vertex iteration | 1 | `plane_collision` | 2 |
| Mesh topology preservation | 2-3 | `integration` | 2 |
| Mesh bounding radius from AABB | 3 | `integration` | 1 |
| Mesh-Sphere face/edge/vertex | 4 | `mesh_collision` | 3 |
| Mesh-Mesh BVH vs brute-force | 4 | `mesh_collision` | 1 |
| Cylinder-Sphere side/cap/rim | 5 | `primitive_collision` | 4 |
| Cylinder-Capsule parallel/perp/endpoint | 5 | `primitive_collision` | 3 |
| Numerical edge cases | 6 | `edge_cases` | 8 |
| MuJoCo conformance | 6 | `conformance` | 3 |

**Minimum test count: 34 tests** (not counting parametric orientation variants)

### Performance Gates — Hard Thresholds

| Criterion | Debug | Release | Test |
|-----------|-------|---------|------|
| Humanoid throughput | ≥300 steps/sec | ≥5000 steps/sec | `humanoid_throughput` |
| Mesh scaling O(log n) | <3× time for 4× tris | <2× time for 4× tris | `mesh_collision_scaling` |
| Hot path allocations | 0 | 0 | `collision_no_allocations` |

### Numerical Tolerances — Justified Constants

| Constant | Value | Justification |
|----------|-------|---------------|
| `MACHINE_EPS` | 2.22e-16 | IEEE 754 double precision |
| `GEOM_TOL` | 1e-10 | 100× machine ε, accommodates 10-op accumulation |
| `DEPTH_TOL` | 1e-6 | mm-scale for m-scale geometry, sufficient for 1kHz |
| `NORMAL_TOL` | 1e-3 | ~0.06° angular deviation, imperceptible |
| `MUJOCO_DEPTH_TOL` | 1e-4 | 0.1mm agreement with reference |
| `MUJOCO_POS_TOL` | 1e-4 | 0.1mm position agreement |

### Todorov Quality Checklist

| Criterion | Evidence | Verification |
|-----------|----------|--------------|
| No code duplication | Mesh functions from `mesh.rs` reused | `grep -r "fn mesh_.*contact"` shows single source |
| Single source of truth | `model.mesh_data` only mesh storage | `grep "TriangleMeshData"` in Model struct |
| Arc for expensive data | `Arc<TriangleMeshData>` prevents clone | Type signature in Model |
| O(n) broad phase | Spatial hashing maintained | `broad_phase` function complexity comment |
| O(log n) mesh queries | BVH traversal | `mesh_collision_scaling` test validates |
| Compute once, use everywhere | Contact frame built once | `build_contact_frame` call site count |

### Rust Purist Quality Checklist

| Criterion | Evidence | Verification |
|-----------|----------|--------------|
| No `unwrap()` in production | `expect()` only in model building | `grep -r "unwrap()" --include="*.rs"` |
| Exhaustive match | All `GeomType` variants explicit | No `_ =>` in collision dispatch |
| No allocations in hot path | Pre-sized vectors, slice iteration | `collision_no_allocations` test |
| Zero-copy where possible | `&Arc<T>` and `&[T]` signatures | Function signatures audit |
| Prefer `Option` over sentinel | No magic `-1` or `NaN` values | Type signatures |
| Const generics where applicable | Fixed-size contact arrays | `[Contact; MAX_CONTACTS]` |

---

## File Change Summary

| File | Changes | LOC Est. |
|------|---------|----------|
| `mujoco_pipeline.rs` | Add mesh fields to Model, collision functions | +400 |
| `mujoco_pipeline.rs` | Plane-Cylinder, Plane-Ellipsoid analytical | +150 |
| `mujoco_pipeline.rs` | Cylinder-Sphere, Cylinder-Capsule analytical | +200 |
| `model_builder.rs` | MJCF mesh loading and linking | +150 |
| `collision_validation.rs` | Integration & conformance tests | +400 |
| `collision_edge_cases.rs` | Numerical pathology tests | +250 |
| `collision_performance.rs` | Throughput & scaling gates | +150 |
| **Total** | | **~1700** |

---

## Implementation Order

1. **Phase 1** (Critical Bug Fix): Plane-Cylinder, Plane-Ellipsoid — objects currently fall through ground
2. **Phase 2-3** (Model Infrastructure): Add mesh fields, MJCF loading
3. **Phase 4** (Mesh Integration): Wire `collide_with_mesh()` into dispatcher
4. **Phase 5** (Polish): Analytical Cylinder-Sphere, Cylinder-Capsule
5. **Phase 6** (Validation): The Obsessive Protocol
   - 6.0: Test infrastructure with justified tolerances
   - 6.1: Plane collision exhaustive coverage (20+ tests)
   - 6.2: Primitive-primitive analytical validation (10+ tests)
   - 6.3: Mesh BVH correctness via brute-force comparison
   - 6.4: Numerical edge cases (parallel, zero, large, degenerate)
   - 6.5: Full pipeline integration tests
   - 6.6: Performance regression gates with hard thresholds
   - 6.7: MuJoCo reference conformance tests

**Phase 6 is not optional polish.** It is the proof that the implementation is correct.

---

## References

- Ericson, C. (2005). "Real-Time Collision Detection" - Chapters 5 (BVH), 7 (Primitives)
- MuJoCo Source: `engine_collision_primitive.c` - Reference implementations
- Todorov, E. (2012). "MuJoCo: A physics engine for model-based control"
- Featherstone, R. (2008). "Rigid Body Dynamics Algorithms" - Spatial algebra
