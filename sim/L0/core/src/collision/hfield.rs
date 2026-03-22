//! Height field collision — prism-based convex-vs-heightfield collision.
//!
//! Implements MuJoCo's `mjc_ConvexHField()`: decomposes the heightfield into
//! triangular prisms in the sub-grid overlapping the convex geom, then tests
//! each prism against the geom using GJK/EPA penetration. Returns 0..50
//! contacts per geom pair.

use super::narrow::{geom_to_shape, make_contact_from_geoms};
use crate::gjk_epa::{gjk_epa_contact, support};
use crate::heightfield::HeightFieldData;
use crate::types::{Contact, GeomType, Model};
use cf_geometry::Shape;
use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
use sim_types::Pose;
use std::cell::Cell;

/// Maximum contacts per heightfield-geom pair (MuJoCo `mjMAXCONPAIR`).
pub const MAX_CONTACTS_PER_PAIR: usize = 50;

/// Prism-based heightfield collision for all convex geom types.
///
/// Implements MuJoCo's `mjc_ConvexHField`: decomposes the heightfield into
/// triangular prisms in the sub-grid overlapping the convex geom, then tests
/// each prism against the geom using GJK/EPA penetration.
///
/// Returns 0..50 contacts.
#[allow(
    clippy::too_many_arguments,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::similar_names
)]
pub fn collide_hfield_multi(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    margin: f64,
) -> Vec<Contact> {
    let mut contacts = Vec::new();

    // --- Phase 0: Identify hfield and convex geom ---
    let (hf_geom, conv_geom, hf_pos, hf_mat, conv_pos, conv_mat) =
        if model.geom_type[geom1] == GeomType::Hfield {
            (geom1, geom2, pos1, mat1, pos2, mat2)
        } else {
            (geom2, geom1, pos2, mat2, pos1, mat1)
        };

    let Some(hfield_id) = model.geom_hfield[hf_geom] else {
        return contacts;
    };
    let hfield = &model.hfield_data[hfield_id];
    let hf_size = &model.hfield_size[hfield_id];

    // Build convex collision shape
    let conv_type = model.geom_type[conv_geom];
    let conv_size = model.geom_size[conv_geom];
    let conv_shape = match conv_type {
        GeomType::Mesh => {
            let Some(mesh_id) = model.geom_mesh[conv_geom] else {
                return contacts;
            };
            let Some(hull) = model.mesh_data[mesh_id].convex_hull() else {
                return contacts;
            };
            Shape::convex_mesh(hull.clone())
        }
        _ => match geom_to_shape(conv_type, conv_size) {
            Some(s) => s,
            None => return contacts,
        },
    };

    // Hfield pose: world position WITHOUT centering offset — the offset is
    // baked into prism vertex positions via `dx * c - hf_x_half`.
    let hf_quat = UnitQuaternion::from_matrix(&hf_mat);
    let hf_pose = Pose::from_position_rotation(Point3::from(hf_pos), hf_quat);

    // Convex geom pose
    let conv_quat = UnitQuaternion::from_matrix(&conv_mat);
    let conv_pose = Pose::from_position_rotation(Point3::from(conv_pos), conv_quat);

    // --- Phase 1: Express convex geom in hfield-local frame ---
    let local_pos = hf_mat.transpose() * (conv_pos - hf_pos);

    // --- Phase 2: Bounding sphere early exit ---
    let rbound = model.geom_rbound[conv_geom] + margin;
    let hf_x_half = hf_size[0];
    let hf_y_half = hf_size[1];
    let hf_z_top = hfield.max_height();
    let hf_z_bot = -hf_size[3];

    if local_pos.x + rbound < -hf_x_half
        || local_pos.x - rbound > hf_x_half
        || local_pos.y + rbound < -hf_y_half
        || local_pos.y - rbound > hf_y_half
        || local_pos.z + rbound < hf_z_bot
        || local_pos.z - rbound > hf_z_top
    {
        return contacts;
    }

    // --- Phase 3: AABB via support queries ---
    let local_mat = hf_mat.transpose() * conv_mat;
    let local_quat = UnitQuaternion::from_matrix(&local_mat);
    let local_pose = Pose::from_position_rotation(Point3::from(local_pos), local_quat);

    let (aabb_min, aabb_max) = compute_local_aabb(&conv_shape, &local_pose);

    if aabb_max.x < -hf_x_half
        || aabb_min.x > hf_x_half
        || aabb_max.y < -hf_y_half
        || aabb_min.y > hf_y_half
        || aabb_max.z < hf_z_bot
        || aabb_min.z > hf_z_top
    {
        return contacts;
    }

    // --- Phase 4: Sub-grid clipping ---
    let ncol = hfield.width();
    let nrow = hfield.depth();
    if ncol < 2 || nrow < 2 {
        return contacts;
    }
    let dx = 2.0 * hf_x_half / (ncol - 1) as f64;
    let dy = 2.0 * hf_y_half / (nrow - 1) as f64;

    let cmin = ((aabb_min.x + hf_x_half) / dx).floor().max(0.0) as usize;
    let cmax = ((aabb_max.x + hf_x_half) / dx)
        .ceil()
        .min((ncol - 1) as f64) as usize;
    let rmin = ((aabb_min.y + hf_y_half) / dy).floor().max(0.0) as usize;
    let rmax = ((aabb_max.y + hf_y_half) / dy)
        .ceil()
        .min((nrow - 1) as f64) as usize;

    let zmin = aabb_min.z;

    // --- Phase 5 & 6: Prism iteration and collision ---
    for r in rmin..rmax {
        for c in cmin..cmax {
            for i in 0..2u8 {
                let prism_verts = build_prism(hfield, hf_size, r, c, i, dx, dy, margin);

                // Skip if all 3 top vertices are below zmin
                if prism_verts[3].z < zmin && prism_verts[4].z < zmin && prism_verts[5].z < zmin {
                    continue;
                }

                // Build prism as ConvexMesh (6 vertices)
                let prism_points: Vec<Point3<f64>> =
                    prism_verts.iter().map(|v| Point3::from(*v)).collect();
                let Some(hull) = cf_geometry::convex_hull(&prism_points, None) else {
                    continue;
                };
                let prism_shape = Shape::convex_mesh(hull);

                // Test prism vs convex geom using GJK/EPA.
                // Prism is in hfield-local coords → use hfield world pose.
                if let Some(result) = gjk_epa_contact(
                    &prism_shape,
                    &hf_pose,
                    &conv_shape,
                    &conv_pose,
                    model.ccd_iterations,
                    model.ccd_tolerance,
                ) {
                    // Contact normal: GJK/EPA returns normal from shape_a
                    // (prism/hfield) toward shape_b (convex geom).
                    // When hfield is geom1, normal points from geom1 to geom2
                    // (correct convention). When hfield is geom2, negate.
                    let normal = if hf_geom == geom1 {
                        result.normal
                    } else {
                        -result.normal
                    };

                    contacts.push(make_contact_from_geoms(
                        model,
                        result.point.coords,
                        normal,
                        result.penetration,
                        geom1,
                        geom2,
                        margin,
                    ));

                    if contacts.len() >= MAX_CONTACTS_PER_PAIR {
                        return contacts;
                    }
                }
            }
        }
    }

    contacts
}

/// Build a triangular prism for one triangle of a heightfield grid cell.
///
/// Returns 6 vertices: [bottom0, bottom1, bottom2, top0, top1, top2].
/// Bottom vertices are at Z = -size[3] (base elevation).
/// Top vertices are at Z = height + margin (surface elevation).
///
/// `i=0`: lower-left triangle (vertices at (c,r), (c+1,r), (c,r+1))
/// `i=1`: upper-right triangle (vertices at (c+1,r), (c+1,r+1), (c,r+1))
#[allow(
    clippy::cast_precision_loss,
    clippy::too_many_arguments,
    clippy::similar_names
)]
fn build_prism(
    hfield: &HeightFieldData,
    hf_size: &[f64; 4],
    r: usize,
    c: usize,
    i: u8,
    dx: f64,
    dy: f64,
    margin: f64,
) -> [Vector3<f64>; 6] {
    let base_z = -hf_size[3];
    let hf_x_half = hf_size[0];
    let hf_y_half = hf_size[1];

    // Grid vertex positions in MuJoCo center-origin coords
    let (v0, v1, v2) = if i == 0 {
        // Lower-left triangle: (c,r), (c+1,r), (c,r+1)
        ((c, r), (c + 1, r), (c, r + 1))
    } else {
        // Upper-right triangle: (c+1,r), (c+1,r+1), (c,r+1)
        ((c + 1, r), (c + 1, r + 1), (c, r + 1))
    };

    let x0 = dx * v0.0 as f64 - hf_x_half;
    let y0 = dy * v0.1 as f64 - hf_y_half;
    let x1 = dx * v1.0 as f64 - hf_x_half;
    let y1 = dy * v1.1 as f64 - hf_y_half;
    let x2 = dx * v2.0 as f64 - hf_x_half;
    let y2 = dy * v2.1 as f64 - hf_y_half;

    // Heights from HeightFieldData — already pre-scaled by size[2].
    // MuJoCo (c, r) maps to CortenForge (x=c, y=r).
    let h0 = hfield.get(v0.0, v0.1).unwrap_or(0.0) + margin;
    let h1 = hfield.get(v1.0, v1.1).unwrap_or(0.0) + margin;
    let h2 = hfield.get(v2.0, v2.1).unwrap_or(0.0) + margin;

    [
        // Bottom layer (at base elevation)
        Vector3::new(x0, y0, base_z),
        Vector3::new(x1, y1, base_z),
        Vector3::new(x2, y2, base_z),
        // Top layer (at surface + margin)
        Vector3::new(x0, y0, h0),
        Vector3::new(x1, y1, h1),
        Vector3::new(x2, y2, h2),
    ]
}

/// Compute tight AABB of a shape at a given pose.
///
/// Queries the GJK support function along ±X, ±Y, ±Z to get exact
/// extremes of the convex shape.
fn compute_local_aabb(shape: &Shape, pose: &Pose) -> (Vector3<f64>, Vector3<f64>) {
    let dirs = [
        Vector3::x(),
        -Vector3::x(),
        Vector3::y(),
        -Vector3::y(),
        Vector3::z(),
        -Vector3::z(),
    ];

    let warm_start = Cell::new(0);
    let mut min = Vector3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max = Vector3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);

    for dir in &dirs {
        let pt = support(shape, pose, dir, &warm_start);
        min.x = min.x.min(pt.x);
        min.y = min.y.min(pt.y);
        min.z = min.z.min(pt.z);
        max.x = max.x.max(pt.x);
        max.y = max.y.max(pt.y);
        max.z = max.z.max(pt.z);
    }

    (min, max)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::cast_precision_loss)]
mod tests {
    use super::*;
    use crate::heightfield::HeightFieldData;
    use crate::types::Model;
    use std::sync::Arc;

    /// Build a minimal Model with one hfield geom (index 0) and one convex geom (index 1).
    ///
    /// `hf_heights`: flat-row-major heights (already pre-scaled).
    /// `hf_width`/`hf_depth`: grid dimensions.
    /// `hf_size`: MuJoCo [x_half, y_half, z_top_scale, z_bottom].
    /// `conv_type`: GeomType of the convex geom.
    /// `conv_size`: geom_size of the convex geom.
    /// `conv_rbound`: bounding sphere radius for the convex geom.
    fn make_hfield_model(
        hf_heights: Vec<f64>,
        hf_width: usize,
        hf_depth: usize,
        hf_size: [f64; 4],
        conv_type: GeomType,
        conv_size: Vector3<f64>,
        conv_rbound: f64,
    ) -> Model {
        let cell_size = if hf_width > 1 {
            2.0 * hf_size[0] / (hf_width - 1) as f64
        } else {
            1.0
        };
        let hfield = HeightFieldData::new(hf_heights, hf_width, hf_depth, cell_size);

        let mut model = Model::empty();
        model.ngeom = 2;
        model.nbody = 2;
        model.body_parent.push(0); // body 1 parent = world
        model.body_rootid.push(1);
        model.body_jnt_adr.push(0);
        model.body_jnt_num.push(0);
        model.body_dof_adr.push(0);
        model.body_dof_num.push(0);
        model.body_geom_adr.push(1);
        model.body_geom_num.push(1);
        model.body_pos.push(Vector3::zeros());
        model.body_quat.push(UnitQuaternion::identity());
        model.body_ipos.push(Vector3::zeros());
        model.body_iquat.push(UnitQuaternion::identity());
        model.body_mass.push(1.0);
        model.body_inertia.push(Vector3::new(1.0, 1.0, 1.0));
        model.body_name.push(None);
        model.body_subtreemass.push(1.0);
        model.body_mocapid.push(None);
        model.body_gravcomp.push(0.0);
        model.body_invweight0.push([1.0; 2]);
        model.body_treeid.push(0);

        model.geom_type = vec![GeomType::Hfield, conv_type];
        model.geom_body = vec![0, 1];
        model.geom_pos = vec![Vector3::zeros(); 2];
        model.geom_quat = vec![UnitQuaternion::identity(); 2];
        model.geom_size = vec![Vector3::zeros(), conv_size];
        model.geom_name = vec![None; 2];
        model.geom_rbound = vec![f64::INFINITY, conv_rbound];
        model.geom_mesh = vec![None; 2];
        model.geom_hfield = vec![Some(0), None];
        model.geom_shape = vec![None; 2];
        model.geom_contype = vec![1; 2];
        model.geom_conaffinity = vec![1; 2];
        model.geom_friction = vec![Vector3::new(1.0, 0.005, 0.0001); 2];
        model.geom_condim = vec![3; 2];
        model.geom_solref = vec![[0.02, 1.0]; 2];
        model.geom_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; 2];
        model.geom_priority = vec![0; 2];
        model.geom_solmix = vec![1.0; 2];
        model.geom_margin = vec![0.0; 2];
        model.geom_gap = vec![0.0; 2];
        model.geom_group = vec![0; 2];
        model.geom_rgba = vec![[1.0; 4]; 2];
        model.geom_fluid = vec![[0.0; 12]; 2];

        model.hfield_data = vec![Arc::new(hfield)];
        model.hfield_size = vec![hf_size];
        model.mesh_data = vec![];
        model
    }

    /// Build a flat heightfield at the given height.
    /// Grid spans [-x_half..+x_half] x [-y_half..+y_half] with `n×n` vertices.
    fn flat_hfield_model(
        n: usize,
        x_half: f64,
        y_half: f64,
        height: f64,
        conv_type: GeomType,
        conv_size: Vector3<f64>,
        conv_rbound: f64,
    ) -> Model {
        let heights = vec![height; n * n];
        make_hfield_model(
            heights,
            n,
            n,
            [x_half, y_half, 1.0, 1.0], // z_top_scale=1.0, z_bottom=1.0
            conv_type,
            conv_size,
            conv_rbound,
        )
    }

    // ========================================================================
    // T1: Hfield-sphere prism contact → AC1
    // ========================================================================
    #[test]
    fn t1_hfield_sphere_prism_contact() {
        // 5×5 flat hfield at z=0.5, sphere radius=0.3 at (0,0,0.7)
        // Sphere bottom at z=0.4, below surface at z=0.5 → penetrating.
        let model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.5,
            GeomType::Sphere,
            Vector3::new(0.3, 0.0, 0.0),
            0.3,
        );
        let pos1 = Vector3::zeros(); // hfield at origin
        let mat1 = Matrix3::identity();
        let pos2 = Vector3::new(0.0, 0.0, 0.7);
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);

        assert!(!contacts.is_empty(), "Expected at least 1 contact");
        let c = &contacts[0];
        // Depth should be approximately 0.1 (sphere bottom at 0.4, surface at 0.5).
        // Tolerance ±0.05 per spec.
        assert!(
            c.depth > 0.05 && c.depth < 0.15,
            "Depth {} should be ≈0.1 (±0.05)",
            c.depth
        );
        // Normal should be approximately +Z (flat terrain)
        assert!(c.normal.z > 0.5, "Normal Z {} should be upward", c.normal.z);
    }

    // ========================================================================
    // T2: Hfield-mesh contact (new pair) → AC2
    // ========================================================================
    #[test]
    fn t2_hfield_mesh_with_hull() {
        use crate::mesh::TriangleMeshData;

        // Build a unit cube mesh centered at origin with a convex hull.
        let s = 0.5;
        let verts = vec![
            Point3::new(-s, -s, -s),
            Point3::new(s, -s, -s),
            Point3::new(s, s, -s),
            Point3::new(-s, s, -s),
            Point3::new(-s, -s, s),
            Point3::new(s, -s, s),
            Point3::new(s, s, s),
            Point3::new(-s, s, s),
        ];
        #[rustfmt::skip]
        let indices = vec![
            0,1,2, 0,2,3, // -Z face
            4,6,5, 4,7,6, // +Z face
            0,4,5, 0,5,1, // -Y face
            2,6,7, 2,7,3, // +Y face
            0,3,7, 0,7,4, // -X face
            1,5,6, 1,6,2, // +X face
        ];
        let mut mesh = TriangleMeshData::new(verts, indices);
        mesh.compute_convex_hull(None);
        assert!(mesh.convex_hull().is_some(), "Hull should be computed");

        // 5×5 flat hfield at z=0.0
        let mut model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.0,
            GeomType::Mesh,
            Vector3::new(0.5, 0.5, 0.5),
            0.87, // rbound ≈ sqrt(0.5² + 0.5² + 0.5²)
        );
        model.mesh_data = vec![Arc::new(mesh)];
        model.geom_mesh = vec![None, Some(0)]; // geom 1 = mesh index 0

        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        // Cube center at z=0.2 → bottom face at z=-0.3, deeply penetrating terrain at z=0.0
        let pos2 = Vector3::new(0.0, 0.0, 0.2);
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            !contacts.is_empty(),
            "Mesh with convex hull penetrating terrain should produce contacts"
        );
        // Verify at least one contact has positive depth. Some prism-mesh contacts
        // may be edge/corner touches with depth=0; the important thing is that
        // penetrating prisms produce positive-depth contacts.
        let max_depth = contacts.iter().map(|c| c.depth).fold(0.0_f64, f64::max);
        assert!(
            max_depth > 0.0,
            "At least one contact should have positive depth, max was {max_depth}",
        );
    }

    // ========================================================================
    // T3: Hfield-cylinder exact (no capsule approximation) → AC3
    // ========================================================================
    #[test]
    fn t3_hfield_cylinder_exact() {
        // Cylinder (radius=0.5, half_length=1.0) on its side, penetrating flat terrain at z=0.
        let model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.0,
            GeomType::Cylinder,
            Vector3::new(0.5, 1.0, 0.0), // MuJoCo: size[0]=radius, size[1]=half_length
            1.2,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        // Cylinder at z=0.3 with radius=0.5 → bottom at z=-0.2 (below terrain at z=0)
        let pos2 = Vector3::new(0.0, 0.0, 0.3);
        let mat2 = Matrix3::identity(); // Z-axis up = standing

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            !contacts.is_empty(),
            "Cylinder penetrating terrain should produce contacts"
        );
    }

    // ========================================================================
    // T4: Hfield-ellipsoid exact (no sphere approximation) → AC4
    // ========================================================================
    #[test]
    fn t4_hfield_ellipsoid_exact() {
        // Ellipsoid (radii 0.5, 0.3, 0.2) at z=0.15 → bottom at z=-0.05
        let model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.0,
            GeomType::Ellipsoid,
            Vector3::new(0.5, 0.3, 0.2),
            0.5,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        let pos2 = Vector3::new(0.0, 0.0, 0.15);
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            !contacts.is_empty(),
            "Ellipsoid penetrating terrain should produce contacts"
        );
    }

    // ========================================================================
    // T5: Multi-contact generation → AC5
    // ========================================================================
    #[test]
    fn t5_multi_contact_generation() {
        // 10×10 heightfield with sinusoidal terrain. Large box spanning multiple cells.
        let n = 10;
        let x_half = 5.0;
        let y_half = 5.0;
        let mut heights = Vec::with_capacity(n * n);
        for y in 0..n {
            for x in 0..n {
                let fx = x as f64 / (n - 1) as f64 * std::f64::consts::TAU;
                let fy = y as f64 / (n - 1) as f64 * std::f64::consts::TAU;
                heights.push(0.5 * (fx.sin() + fy.sin()) + 1.0);
            }
        }
        let model = make_hfield_model(
            heights,
            n,
            n,
            [x_half, y_half, 1.0, 2.0],
            GeomType::Box,
            Vector3::new(2.0, 2.0, 0.5), // Large box: 4×4×1
            3.0,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        // Box center at z=0.5 → bottom at z=0.0, terrain peaks ~2.0 → overlapping
        let pos2 = Vector3::new(0.0, 0.0, 0.5);
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            contacts.len() >= 4,
            "Large box on sinusoidal terrain should produce ≥4 contacts, got {}",
            contacts.len()
        );
    }

    // ========================================================================
    // T6: MAX_CONTACTS_PER_PAIR = 50 limit → AC6
    // ========================================================================
    #[test]
    fn t6_max_contacts_limit() {
        // 50×50 heightfield, very large box spanning entire grid, deeply penetrating.
        let n = 50;
        let x_half = 25.0;
        let y_half = 25.0;
        let heights = vec![1.0; n * n]; // Flat at z=1.0
        let model = make_hfield_model(
            heights,
            n,
            n,
            [x_half, y_half, 1.0, 2.0],
            GeomType::Box,
            Vector3::new(24.0, 24.0, 0.5), // Huge box spanning almost entire grid
            35.0,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        // Box at z=0.0 → bottom at z=-0.5, terrain at z=1.0 → deep penetration
        let pos2 = Vector3::new(0.0, 0.0, 0.0);
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            contacts.len() <= 50,
            "Should be capped at 50 contacts, got {}",
            contacts.len()
        );
        assert!(
            contacts.len() == 50,
            "With so many cells, should hit the 50-contact limit, got {}",
            contacts.len()
        );
    }

    // ========================================================================
    // T7: Geom outside hfield bounds → AC7
    // ========================================================================
    #[test]
    fn t7_outside_bounds_no_contact() {
        let model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.0,
            GeomType::Sphere,
            Vector3::new(0.3, 0.0, 0.0),
            0.3,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        let pos2 = Vector3::new(100.0, 100.0, 0.0); // Way outside
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            contacts.is_empty(),
            "Outside bounds should produce 0 contacts"
        );
    }

    // ========================================================================
    // T8: Mesh with no convex hull → AC8
    // ========================================================================
    #[test]
    fn t8_mesh_no_hull_no_contact() {
        // Model with mesh geom but no hull → 0 contacts, no panic.
        // geom_mesh[1] = None (no mesh data) → early return
        let model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.0,
            GeomType::Mesh,
            Vector3::new(0.5, 0.5, 0.5),
            1.0,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        let pos2 = Vector3::new(0.0, 0.0, 0.0);
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(contacts.is_empty());
    }

    // ========================================================================
    // T11: Hfield-plane returns no contacts → AC11
    // ========================================================================
    #[test]
    fn t11_hfield_plane_no_contact() {
        // Plane has no Shape → geom_to_shape returns None → 0 contacts
        let model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.0,
            GeomType::Plane,
            Vector3::new(1.0, 1.0, 0.0),
            f64::INFINITY,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        let pos2 = Vector3::new(0.0, 0.0, 0.0);
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            contacts.is_empty(),
            "Hfield-plane should produce 0 contacts"
        );
    }

    // ========================================================================
    // TS1: Single-cell hfield (2×2 grid)
    // ========================================================================
    #[test]
    fn ts1_single_cell_hfield() {
        // Minimum valid: 2×2 = 1 cell = 2 prisms
        let model = flat_hfield_model(
            2,
            1.0,
            1.0,
            0.5,
            GeomType::Sphere,
            Vector3::new(0.3, 0.0, 0.0),
            0.3,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        let pos2 = Vector3::new(0.0, 0.0, 0.7); // Sphere bottom at 0.4, terrain at 0.5
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            !contacts.is_empty(),
            "Single-cell hfield should still produce contacts"
        );
    }

    // ========================================================================
    // TS2: Edge-straddling geom
    // ========================================================================
    #[test]
    fn ts2_edge_straddling() {
        // Sphere at the edge of the hfield
        let model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.5,
            GeomType::Sphere,
            Vector3::new(0.5, 0.0, 0.0),
            0.5,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        // Sphere center at x=1.8 (near edge at x=2.0), z=0.9 → bottom at 0.4
        let pos2 = Vector3::new(1.8, 0.0, 0.9);
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            !contacts.is_empty(),
            "Edge-straddling sphere should produce contacts"
        );
    }

    // ========================================================================
    // TS3: Small geom in single cell — sub-grid converges to 1-2 prisms
    // ========================================================================
    #[test]
    fn ts3_small_geom_one_cell() {
        // 10×10 hfield with tiny sphere fitting within a single cell.
        // Cell size = 2*5/(10-1) ≈ 1.11. Sphere radius=0.1, well within one cell.
        let model = flat_hfield_model(
            10,
            5.0,
            5.0,
            0.5,
            GeomType::Sphere,
            Vector3::new(0.1, 0.0, 0.0),
            0.1,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        // Sphere at origin, z=0.55 → bottom at 0.45, terrain at 0.5 → penetrating
        let pos2 = Vector3::new(0.0, 0.0, 0.55);
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            !contacts.is_empty(),
            "Tiny sphere in single cell should produce contacts"
        );
        // Tiny sphere should only hit 1-2 prisms → few contacts
        assert!(
            contacts.len() <= 4,
            "Tiny sphere should hit at most a few prisms, got {}",
            contacts.len()
        );
    }

    // ========================================================================
    // TS4: Tilted hfield (non-identity pose)
    // ========================================================================
    #[test]
    fn ts4_tilted_hfield() {
        // Hfield rotated 45 degrees around Y axis
        let model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.5,
            GeomType::Sphere,
            Vector3::new(0.3, 0.0, 0.0),
            0.3,
        );
        let angle = std::f64::consts::FRAC_PI_4; // 45 degrees
        let hf_mat = Matrix3::new(
            angle.cos(),
            0.0,
            angle.sin(),
            0.0,
            1.0,
            0.0,
            -angle.sin(),
            0.0,
            angle.cos(),
        );
        let pos1 = Vector3::zeros();
        let pos2 = Vector3::new(0.0, 0.0, 0.7);
        let mat2 = Matrix3::identity();

        // Should not panic with non-identity hfield rotation
        let contacts = collide_hfield_multi(&model, 0, 1, pos1, hf_mat, pos2, mat2, 0.0);
        // Whether contacts are generated depends on geometry after rotation,
        // but no panic is the key assertion.
        let _ = contacts;
    }

    // ========================================================================
    // TS5: Flat prism (all heights same = degenerate slab)
    // ========================================================================
    #[test]
    fn ts5_flat_prism_degenerate() {
        // All heights identical — prism top is flat. GJK should still work.
        let model = flat_hfield_model(
            3,
            1.0,
            1.0,
            1.0, // flat at z=1.0
            GeomType::Sphere,
            Vector3::new(0.3, 0.0, 0.0),
            0.3,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        let pos2 = Vector3::new(0.0, 0.0, 1.2); // Sphere bottom at 0.9, terrain at 1.0
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            !contacts.is_empty(),
            "Flat prism should still produce contacts"
        );
    }

    // ========================================================================
    // TS6: Hfield-hfield (no crash, graceful)
    // ========================================================================
    #[test]
    fn ts6_hfield_hfield_no_crash() {
        // Two hfield geoms — should return 0 contacts (Hfield has no convex Shape)
        let heights = vec![0.5; 25];
        let cell_size = 1.0;
        let hfield = HeightFieldData::new(heights, 5, 5, cell_size);

        let mut model = Model::empty();
        model.ngeom = 2;
        model.nbody = 2;
        model.body_parent.push(0);
        model.body_rootid.push(1);
        model.body_jnt_adr.push(0);
        model.body_jnt_num.push(0);
        model.body_dof_adr.push(0);
        model.body_dof_num.push(0);
        model.body_geom_adr.push(1);
        model.body_geom_num.push(1);
        model.body_pos.push(Vector3::zeros());
        model.body_quat.push(UnitQuaternion::identity());
        model.body_ipos.push(Vector3::zeros());
        model.body_iquat.push(UnitQuaternion::identity());
        model.body_mass.push(0.0);
        model.body_inertia.push(Vector3::zeros());
        model.body_name.push(None);
        model.body_subtreemass.push(0.0);
        model.body_mocapid.push(None);
        model.body_gravcomp.push(0.0);
        model.body_invweight0.push([0.0; 2]);
        model.body_treeid.push(0);

        model.geom_type = vec![GeomType::Hfield; 2];
        model.geom_body = vec![0, 1];
        model.geom_pos = vec![Vector3::zeros(); 2];
        model.geom_quat = vec![UnitQuaternion::identity(); 2];
        model.geom_size = vec![Vector3::zeros(); 2];
        model.geom_name = vec![None; 2];
        model.geom_rbound = vec![f64::INFINITY; 2];
        model.geom_mesh = vec![None; 2];
        model.geom_hfield = vec![Some(0), Some(0)];
        model.geom_shape = vec![None; 2];
        model.geom_contype = vec![1; 2];
        model.geom_conaffinity = vec![1; 2];
        model.geom_friction = vec![Vector3::new(1.0, 0.005, 0.0001); 2];
        model.geom_condim = vec![3; 2];
        model.geom_solref = vec![[0.02, 1.0]; 2];
        model.geom_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; 2];
        model.geom_priority = vec![0; 2];
        model.geom_solmix = vec![1.0; 2];
        model.geom_margin = vec![0.0; 2];
        model.geom_gap = vec![0.0; 2];
        model.geom_group = vec![0; 2];
        model.geom_rgba = vec![[1.0; 4]; 2];
        model.geom_fluid = vec![[0.0; 12]; 2];

        model.hfield_data = vec![Arc::new(hfield)];
        model.hfield_size = vec![[2.0, 2.0, 1.0, 1.0]];

        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        let pos2 = Vector3::new(0.0, 0.0, 0.0);
        let mat2 = Matrix3::identity();

        // Hfield-hfield: the "convex" geom is also Hfield → geom_to_shape returns None → 0 contacts
        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            contacts.is_empty(),
            "Hfield-hfield should produce 0 contacts"
        );
    }

    // ========================================================================
    // Test: Swapped ordering (convex is geom1, hfield is geom2)
    // ========================================================================
    #[test]
    fn test_swapped_geom_order() {
        // Ensure contacts work when hfield is geom2 (not geom1)
        let model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.5,
            GeomType::Sphere,
            Vector3::new(0.3, 0.0, 0.0),
            0.3,
        );
        // Pass geom indices swapped: sphere=geom1(idx=1), hfield=geom2(idx=0)
        let pos_hf = Vector3::zeros();
        let mat_hf = Matrix3::identity();
        let pos_sphere = Vector3::new(0.0, 0.0, 0.7);
        let mat_sphere = Matrix3::identity();

        let contacts = collide_hfield_multi(
            &model, 1, 0, // sphere first, hfield second
            pos_sphere, mat_sphere, pos_hf, mat_hf, 0.0,
        );
        assert!(
            !contacts.is_empty(),
            "Swapped order should still produce contacts"
        );
        // geom1 and geom2 should be assigned correctly
        assert_eq!(contacts[0].geom1, 1);
        assert_eq!(contacts[0].geom2, 0);
    }

    // ========================================================================
    // Test: Hfield-box capsule contact (basic sanity for box type)
    // ========================================================================
    #[test]
    fn test_hfield_box_contact() {
        let model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.0,
            GeomType::Box,
            Vector3::new(0.5, 0.5, 0.5), // 1×1×1 box
            0.87,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        // Box center at z=0.3 → bottom at z=-0.2, terrain at z=0 → penetrating
        let pos2 = Vector3::new(0.0, 0.0, 0.3);
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            !contacts.is_empty(),
            "Box penetrating flat terrain should produce contacts"
        );
    }

    // ========================================================================
    // Test: Capsule on hfield
    // ========================================================================
    #[test]
    fn test_hfield_capsule_contact() {
        let model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.0,
            GeomType::Capsule,
            Vector3::new(0.3, 0.5, 0.0), // radius=0.3, half_length=0.5
            0.8,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        // Standing capsule at z=0.2 → bottom hemisphere center at z=-0.3,
        // hemisphere bottom at z=-0.6 → penetrating terrain at z=0
        let pos2 = Vector3::new(0.0, 0.0, 0.2);
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            !contacts.is_empty(),
            "Capsule penetrating terrain should produce contacts"
        );
    }
}
