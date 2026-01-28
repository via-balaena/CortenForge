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

### Phase 6: Testing

**File**: `sim/L0/tests/integration/validation.rs`

#### Test 6.1: Mesh Loading

```rust
#[test]
fn test_mesh_loading_embedded() {
    let mjcf = r#"
    <mujoco model="mesh_test">
        <asset>
            <mesh name="tetra"
                  vertex="0 0 0  1 0 0  0.5 0.866 0  0.5 0.289 0.816"
                  face="0 1 2  0 2 3  0 3 1  1 3 2"/>
        </asset>
        <worldbody>
            <body name="tetra_body" pos="0 0 1">
                <freejoint/>
                <geom type="mesh" mesh="tetra" mass="1"/>
            </body>
        </worldbody>
    </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load mesh model");

    assert_eq!(model.nmesh, 1, "should have 1 mesh");
    assert_eq!(model.mesh_name[0], "tetra");
    assert_eq!(model.mesh_data[0].num_triangles(), 4);
    assert!(model.geom_mesh[1].is_some(), "mesh geom should reference mesh");
}
```

#### Test 6.2: Mesh-Plane Collision

```rust
#[test]
fn test_mesh_plane_collision() {
    let mjcf = r#"
    <mujoco model="mesh_plane">
        <asset>
            <mesh name="cube"
                  vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5
                          -0.5 -0.5  0.5  0.5 -0.5  0.5  0.5 0.5  0.5  -0.5 0.5  0.5"
                  face="0 1 2  0 2 3  4 6 5  4 7 6  0 4 5  0 5 1  2 6 7  2 7 3  0 3 7  0 7 4  1 5 6  1 6 2"/>
        </asset>
        <worldbody>
            <geom type="plane" size="10 10 0.1"/>
            <body name="cube_body" pos="0 0 0.4">
                <freejoint/>
                <geom type="mesh" mesh="cube" mass="1"/>
            </body>
        </worldbody>
    </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    mj_forward(&model, &mut data);

    assert!(data.ncon > 0, "mesh should contact plane");
    assert!(data.contact[0].depth > 0.0, "should have penetration");
}
```

#### Test 6.3: Cylinder-Plane Collision

```rust
#[test]
fn test_cylinder_plane_collision() {
    let mjcf = r#"
    <mujoco model="cylinder_plane">
        <worldbody>
            <geom type="plane" size="10 10 0.1"/>
            <body name="cyl" pos="0 0 0.4">
                <freejoint/>
                <geom type="cylinder" size="0.3 0.5" mass="1"/>
            </body>
        </worldbody>
    </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    mj_forward(&model, &mut data);

    assert!(data.ncon > 0, "cylinder should contact plane");
}
```

#### Test 6.4: Cylinder-Sphere Collision

```rust
#[test]
fn test_cylinder_sphere_analytical() {
    let mjcf = r#"
    <mujoco model="cyl_sphere">
        <worldbody>
            <body name="cyl" pos="0 0 0">
                <geom type="cylinder" size="0.3 0.5"/>
            </body>
            <body name="sph" pos="0.5 0 0">
                <geom type="sphere" size="0.3"/>
            </body>
        </worldbody>
    </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    mj_forward(&model, &mut data);

    // Should detect collision (0.5 < 0.3 + 0.3 = 0.6)
    assert!(data.ncon > 0, "cylinder-sphere should collide");
}
```

#### Test 6.5: Performance Regression

```rust
#[test]
fn test_collision_performance_no_regression() {
    let model = load_humanoid_model();
    let mut data = model.make_data();

    let start = std::time::Instant::now();
    let steps = 1000;

    for _ in 0..steps {
        mj_step(&model, &mut data);
    }

    let elapsed = start.elapsed();
    let steps_per_sec = steps as f64 / elapsed.as_secs_f64();

    // Must maintain 1000 steps/sec in release, 300 in debug (CI)
    #[cfg(debug_assertions)]
    let threshold = if std::env::var("CI").is_ok() { 300.0 } else { 1000.0 };
    #[cfg(not(debug_assertions))]
    let threshold = 1000.0;

    assert!(
        steps_per_sec >= threshold,
        "Performance regression: {:.0} steps/sec < {:.0} threshold",
        steps_per_sec,
        threshold
    );
}
```

---

## Success Criteria

### Functional Completeness

| Criterion | Phase | Test |
|-----------|-------|------|
| Plane-Cylinder collision works | 1 | `test_cylinder_plane_collision` |
| Plane-Ellipsoid collision works | 1 | `test_ellipsoid_plane_collision` |
| Mesh loads from MJCF | 2-3 | `test_mesh_loading_embedded` |
| Mesh bounding radius correct | 3 | `test_mesh_bounding_radius` |
| Mesh-Plane collision works | 4 | `test_mesh_plane_collision` |
| Mesh-Sphere collision works | 4 | `test_mesh_sphere_collision` |
| Mesh-Mesh collision works | 4 | `test_mesh_mesh_collision` |
| Cylinder-Sphere analytical | 5 | `test_cylinder_sphere_analytical` |
| Cylinder-Capsule analytical | 5 | `test_cylinder_capsule_analytical` |

### Performance

| Criterion | Threshold | Test |
|-----------|-----------|------|
| Humanoid debug mode | ≥1000 steps/sec | `test_collision_performance_no_regression` |
| Humanoid release mode | ≥5000 steps/sec | `test_collision_performance_no_regression` |
| No allocation in collision hot path | 0 allocs | Manual inspection + profiling |

### Todorov Quality

| Criterion | Evidence |
|-----------|----------|
| No code duplication | Mesh functions from `mesh.rs` reused, not copied |
| Single source of truth | `model.mesh_data` is the only mesh storage |
| Arc for expensive data | `Arc<TriangleMeshData>` used throughout |
| O(n) broad phase | Spatial hashing maintained |
| O(log n) mesh queries | BVH used for all mesh collision |

### Rust Purist Quality

| Criterion | Evidence |
|-----------|----------|
| No unwrap in production | All `expect()` in model building only |
| Exhaustive match | All `GeomType` variants handled explicitly |
| No allocations in hot path | Pre-sized contact vectors, no per-frame allocs |
| Zero-copy where possible | `&Arc<TriangleMeshData>` passed by reference |

---

## File Change Summary

| File | Changes | LOC Est. |
|------|---------|----------|
| `mujoco_pipeline.rs` | Add mesh fields to Model, collision functions | +400 |
| `mujoco_pipeline.rs` | Plane-Cylinder, Plane-Ellipsoid analytical | +150 |
| `mujoco_pipeline.rs` | Cylinder-Sphere, Cylinder-Capsule analytical | +200 |
| `model_builder.rs` | MJCF mesh loading and linking | +150 |
| `validation.rs` | New collision tests | +200 |
| **Total** | | **~1100** |

---

## Implementation Order

1. **Phase 1** (Critical Bug Fix): Plane-Cylinder, Plane-Ellipsoid - objects currently fall through ground
2. **Phase 2-3** (Model Infrastructure): Add mesh fields, MJCF loading
3. **Phase 4** (Mesh Integration): Wire `collide_with_mesh()` into dispatcher
4. **Phase 5** (Polish): Analytical Cylinder-Sphere, Cylinder-Capsule
5. **Phase 6** (Validation): Full test suite

---

## References

- Ericson, C. (2005). "Real-Time Collision Detection" - Chapters 5 (BVH), 7 (Primitives)
- MuJoCo Source: `engine_collision_primitive.c` - Reference implementations
- Todorov, E. (2012). "MuJoCo: A physics engine for model-based control"
- Featherstone, R. (2008). "Rigid Body Dynamics Algorithms" - Spatial algebra
