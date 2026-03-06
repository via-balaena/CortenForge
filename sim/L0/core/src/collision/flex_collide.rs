//! Flex-rigid collision helpers — narrowphase for flex vertex spheres against rigid geoms.

use super::{assign_friction, assign_imp, assign_margin, assign_ref, contact_param_flex_rigid};
use crate::collision_shape::CollisionShape;
use crate::forward::closest_point_segment;
use crate::gjk_epa::gjk_epa_contact;
use crate::heightfield::heightfield_sphere_contact;
use crate::mesh::mesh_sphere_contact;
use crate::sdf::sdf_sphere_contact;
use crate::types::{Contact, DISABLE_MIDPHASE, GeomType, Model, compute_tangent_frame, disabled};
use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector2, Vector3};
use sim_types::Pose;

/// Narrowphase collision between a sphere (flex vertex) and a rigid geom.
///
/// Returns `(depth, normal, contact_pos)` if penetrating, `None` otherwise.
/// The normal points from the rigid geom surface toward the vertex.
pub fn narrowphase_sphere_geom(
    sphere_pos: Vector3<f64>,
    sphere_radius: f64,
    geom_idx: usize,
    model: &Model,
    geom_pos: Vector3<f64>,
    geom_mat: Matrix3<f64>,
) -> Option<(f64, Vector3<f64>, Vector3<f64>)> {
    let v = sphere_pos;

    let (d_surface, normal) = match model.geom_type[geom_idx] {
        GeomType::Plane => {
            let plane_normal = geom_mat.column(2).into_owned();
            let d = (v - geom_pos).dot(&plane_normal);
            (d, plane_normal)
        }
        GeomType::Sphere => {
            let radius = model.geom_size[geom_idx].x;
            let diff = v - geom_pos;
            let dist = diff.norm();
            if dist < 1e-10 {
                (-radius, Vector3::new(0.0, 0.0, 1.0))
            } else {
                (dist - radius, diff / dist)
            }
        }
        GeomType::Box => {
            let half = model.geom_size[geom_idx];
            let v_local = geom_mat.transpose() * (v - geom_pos);
            let clamped = Vector3::new(
                v_local.x.clamp(-half.x, half.x),
                v_local.y.clamp(-half.y, half.y),
                v_local.z.clamp(-half.z, half.z),
            );
            let diff_local = v_local - clamped;
            let dist_outside = diff_local.norm();
            if dist_outside > 1e-10 {
                let normal_world = geom_mat * (diff_local / dist_outside);
                (dist_outside, normal_world)
            } else {
                let dx = half.x - v_local.x.abs();
                let dy = half.y - v_local.y.abs();
                let dz = half.z - v_local.z.abs();
                let min_depth = dx.min(dy).min(dz);
                let normal_local = if (dx - min_depth).abs() < 1e-10 {
                    Vector3::new(v_local.x.signum(), 0.0, 0.0)
                } else if (dy - min_depth).abs() < 1e-10 {
                    Vector3::new(0.0, v_local.y.signum(), 0.0)
                } else {
                    Vector3::new(0.0, 0.0, v_local.z.signum())
                };
                (-min_depth, geom_mat * normal_local)
            }
        }
        GeomType::Capsule => {
            let radius = model.geom_size[geom_idx].x;
            let half_len = model.geom_size[geom_idx].y;
            let axis = geom_mat.column(2).into_owned();
            let a = geom_pos - axis * half_len;
            let b = geom_pos + axis * half_len;
            let closest = closest_point_segment(a, b, v);
            let diff = v - closest;
            let dist = diff.norm();
            if dist < 1e-10 {
                (-radius, Vector3::new(0.0, 0.0, 1.0))
            } else {
                (dist - radius, diff / dist)
            }
        }
        GeomType::Cylinder => {
            let radius = model.geom_size[geom_idx].x;
            let half_len = model.geom_size[geom_idx].y;
            let v_local = geom_mat.transpose() * (v - geom_pos);
            let radial_dist = Vector2::new(v_local.x, v_local.y).norm();
            let on_barrel = v_local.z.abs() <= half_len;
            let in_radial = radial_dist <= radius;

            if on_barrel && in_radial {
                let d_radial = radius - radial_dist;
                let d_axial = half_len - v_local.z.abs();
                if d_radial < d_axial {
                    let nl = if radial_dist > 1e-10 {
                        Vector3::new(v_local.x / radial_dist, v_local.y / radial_dist, 0.0)
                    } else {
                        Vector3::new(1.0, 0.0, 0.0)
                    };
                    (-d_radial, geom_mat * nl)
                } else {
                    let nl = Vector3::new(0.0, 0.0, v_local.z.signum());
                    (-d_axial, geom_mat * nl)
                }
            } else if on_barrel {
                let nl = if radial_dist > 1e-10 {
                    Vector3::new(v_local.x / radial_dist, v_local.y / radial_dist, 0.0)
                } else {
                    Vector3::new(1.0, 0.0, 0.0)
                };
                (radial_dist - radius, geom_mat * nl)
            } else if in_radial {
                let nl = Vector3::new(0.0, 0.0, v_local.z.signum());
                (v_local.z.abs() - half_len, geom_mat * nl)
            } else {
                let z_clamped = v_local.z.clamp(-half_len, half_len);
                let closest_local = Vector3::new(
                    v_local.x * radius / radial_dist,
                    v_local.y * radius / radial_dist,
                    z_clamped,
                );
                let diff_local = Vector3::new(v_local.x, v_local.y, v_local.z) - closest_local;
                let dist = diff_local.norm();
                if dist < 1e-10 {
                    return None;
                }
                (dist, geom_mat * (diff_local / dist))
            }
        }
        GeomType::Ellipsoid => {
            let radii = model.geom_size[geom_idx];
            let v_local = geom_mat.transpose() * (v - geom_pos);
            let v_scaled = Vector3::new(
                v_local.x / radii.x,
                v_local.y / radii.y,
                v_local.z / radii.z,
            );
            let dist_scaled = v_scaled.norm();
            if dist_scaled < 1e-10 {
                return None;
            }
            let d_approx = (dist_scaled - 1.0) * radii.x.min(radii.y).min(radii.z);
            let grad_local = Vector3::new(
                v_local.x / (radii.x * radii.x),
                v_local.y / (radii.y * radii.y),
                v_local.z / (radii.z * radii.z),
            );
            let grad_len = grad_local.norm();
            if grad_len < 1e-10 {
                return None;
            }
            (d_approx, geom_mat * (grad_local / grad_len))
        }
        GeomType::Mesh => {
            let mesh_id = model.geom_mesh[geom_idx]?;
            let mesh = &model.mesh_data[mesh_id];
            let geom_quat = UnitQuaternion::from_matrix(&geom_mat);
            let mesh_pose = Pose::from_position_rotation(Point3::from(geom_pos), geom_quat);

            // Primary path: convex hull GJK (MuJoCo-conformant)
            if let Some(hull) = mesh.convex_hull() {
                let hull_shape = CollisionShape::convex_mesh_from_hull(hull);
                let sphere_shape = CollisionShape::Sphere {
                    radius: sphere_radius,
                };
                let sphere_pose =
                    Pose::from_position_rotation(Point3::from(v), UnitQuaternion::identity());

                if let Some(gjk) = gjk_epa_contact(
                    &sphere_shape,
                    &sphere_pose,
                    &hull_shape,
                    &mesh_pose,
                    model.ccd_iterations,
                    model.ccd_tolerance,
                ) {
                    // GJK/EPA returns normal from shape_a toward shape_b
                    // (see hfield.rs:170 comment). Here A=sphere, B=hull, so
                    // the raw normal points from vertex toward mesh. Negate to
                    // get outward-from-mesh-toward-vertex convention.
                    return Some((gjk.penetration, -gjk.normal, gjk.point.coords));
                }
                return None;
            }

            // Fallback: per-triangle BVH (meshes without hull)
            let use_bvh = !disabled(model, DISABLE_MIDPHASE);
            if let Some(mc) =
                mesh_sphere_contact(mesh, &mesh_pose, Point3::from(v), sphere_radius, use_bvh)
            {
                return Some((mc.penetration, mc.normal, mc.point.coords));
            }
            return None;
        }
        GeomType::Hfield => {
            let hfield_id = model.geom_hfield[geom_idx]?;
            let hfield = &model.hfield_data[hfield_id];
            let hf_size = &model.hfield_size[hfield_id];

            // Centering offset: convert center-origin to corner-origin
            let hf_offset = geom_mat * Vector3::new(-hf_size[0], -hf_size[1], 0.0);
            let geom_quat = UnitQuaternion::from_matrix(&geom_mat);
            let hf_pose =
                Pose::from_position_rotation(Point3::from(geom_pos + hf_offset), geom_quat);

            if let Some(hc) =
                heightfield_sphere_contact(hfield, &hf_pose, Point3::from(v), sphere_radius)
            {
                return Some((hc.penetration, hc.normal, hc.point.coords));
            }
            return None;
        }
        GeomType::Sdf => {
            let sdf_id = model.geom_sdf[geom_idx]?;
            let sdf = &model.sdf_data[sdf_id];
            let geom_quat = UnitQuaternion::from_matrix(&geom_mat);
            let sdf_pose = Pose::from_position_rotation(Point3::from(geom_pos), geom_quat);

            if let Some(sc) = sdf_sphere_contact(sdf, &sdf_pose, Point3::from(v), sphere_radius) {
                return Some((sc.penetration, sc.normal, sc.point.coords));
            }
            return None;
        } // All GeomType variants handled — exhaustive match.
    };

    let depth = sphere_radius - d_surface;
    if depth <= 0.0 {
        return None;
    }

    let contact_pos = v - normal * d_surface;
    Some((depth, normal, contact_pos))
}

/// Create a Contact for a flex-rigid collision.
///
/// Uses the unified `contact_param_flex_rigid()` function for parameter
/// combination, mirroring `make_contact_from_geoms()` for rigid-rigid contacts.
pub fn make_contact_flex_rigid(
    model: &Model,
    vertex_idx: usize,
    geom_idx: usize,
    pos: Vector3<f64>,
    normal: Vector3<f64>,
    depth: f64,
) -> Contact {
    let flex_id = model.flexvert_flexid[vertex_idx];
    let (condim, gap, solref, solimp, mu) = contact_param_flex_rigid(model, flex_id, geom_idx);
    // S10: apply global override to solver params when ENABLE_OVERRIDE is active.
    let solref = assign_ref(model, &solref);
    let solimp = assign_imp(model, &solimp);
    let mu = assign_friction(model, &mu);
    // Effective margin = flex_margin + geom_margin, overridden by assign_margin.
    let margin = assign_margin(
        model,
        model.flex_margin[flex_id] + model.geom_margin[geom_idx],
    );
    let includemargin = margin - gap;

    let (t1, t2) = compute_tangent_frame(&normal);

    let dim: usize = match condim {
        1 => 1,
        4 => 4,
        _ => 3, // condim 3 and any other value default to 3D friction cone
    };

    Contact {
        pos,
        normal,
        depth,
        // For flex contacts, geom1 = geom2 = the rigid geom index.
        // The vertex index is stored in flex_vertex.
        // This ensures model.geom_body[contact.geom1] is always valid.
        geom1: geom_idx,
        geom2: geom_idx,
        friction: mu[0],
        dim,
        includemargin,
        mu,
        solref,
        solreffriction: assign_ref(model, &[0.0, 0.0]),
        solimp,
        frame: (t1, t2).into(),
        // Safety: vertex_idx comes from iterating 0..model.nflexvert in
        // mj_collision_flex, so it is always in-bounds. Rust's bounds-checked
        // indexing would panic loudly if a malformed model violated this.
        // No additional validation needed (Phase 8 DT-25 audit decision).
        flex_vertex: Some(vertex_idx),
    }
}

// =============================================================================
// Spec E tests — flex-vs-complex-geom narrowphase
// =============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::cast_precision_loss)]
mod spec_e_tests {
    use super::*;
    use crate::heightfield::HeightFieldData;
    use crate::mesh::TriangleMeshData;
    use crate::sdf::SdfCollisionData;
    use crate::types::Model;
    use nalgebra::{Matrix3, Point3, UnitQuaternion};
    use std::sync::Arc;

    /// Build a minimal Model with one geom at the given type and identity pose.
    fn make_single_geom_model(geom_type: GeomType) -> Model {
        let mut model = Model::empty();
        model.ngeom = 1;
        model.geom_type = vec![geom_type];
        model.geom_body = vec![0];
        model.geom_pos = vec![Vector3::zeros()];
        model.geom_quat = vec![UnitQuaternion::identity()];
        model.geom_size = vec![Vector3::new(0.5, 0.5, 0.5)];
        model.geom_mesh = vec![None];
        model.geom_hfield = vec![None];
        model.geom_sdf = vec![None];
        model.geom_contype = vec![1];
        model.geom_conaffinity = vec![1];
        model.geom_friction = vec![Vector3::new(1.0, 0.005, 0.0001)];
        model.geom_condim = vec![3];
        model.geom_solref = vec![[0.02, 1.0]];
        model.geom_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.geom_priority = vec![0];
        model.geom_solmix = vec![1.0];
        model.geom_margin = vec![0.0];
        model.geom_gap = vec![0.0];
        model.geom_group = vec![0];
        model.geom_rgba = vec![[1.0; 4]];
        model.geom_fluid = vec![[0.0; 12]];
        model.geom_name = vec![None];
        model.geom_rbound = vec![1.0];
        model
    }

    /// Unit cube mesh (half-size 0.5) with convex hull.
    fn cube_mesh_with_hull() -> Arc<TriangleMeshData> {
        let h = 0.5;
        let vertices = vec![
            Point3::new(-h, -h, -h),
            Point3::new(-h, -h, h),
            Point3::new(h, -h, h),
            Point3::new(h, -h, -h),
            Point3::new(-h, h, -h),
            Point3::new(-h, h, h),
            Point3::new(h, h, h),
            Point3::new(h, h, -h),
        ];
        let indices = vec![
            0, 1, 2, 0, 2, 3, // -Y face
            4, 6, 5, 4, 7, 6, // +Y face
            0, 4, 5, 0, 5, 1, // -X face
            3, 2, 6, 3, 6, 7, // +X face
            0, 3, 7, 0, 7, 4, // -Z face
            1, 5, 6, 1, 6, 2, // +Z face
        ];
        let mut mesh = TriangleMeshData::new(vertices, indices);
        mesh.compute_convex_hull(None);
        Arc::new(mesh)
    }

    /// Single-triangle mesh (no convex hull).
    fn single_triangle_mesh() -> Arc<TriangleMeshData> {
        let vertices = vec![
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let indices = vec![0, 1, 2];
        Arc::new(TriangleMeshData::new(vertices, indices))
    }

    // --- T1: Flex vertex sphere vs mesh with convex hull (AC1) ---

    #[test]
    fn t1_flex_vs_mesh_with_hull() {
        let mut model = make_single_geom_model(GeomType::Mesh);
        let mesh = cube_mesh_with_hull();
        assert!(mesh.convex_hull().is_some());
        model.mesh_data = vec![mesh];
        model.geom_mesh = vec![Some(0)];

        // Vertex at z=0.45, radius=0.1. Sphere bottom at 0.35, hull top at 0.5.
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.45),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );

        let (depth, normal, _contact_pos) = result.expect("Should generate contact");
        assert!(
            depth > 0.10 && depth < 0.20,
            "depth={depth}, expected ~0.15"
        );
        assert!(normal.z > 0.5, "normal.z={}, expected >0.5", normal.z);
    }

    // --- T2: Flex vertex sphere vs heightfield (AC2) ---

    #[test]
    fn t2_flex_vs_hfield() {
        let mut model = make_single_geom_model(GeomType::Hfield);

        // 5×5 flat heightfield at height=0.5, half-extent 2.0
        let n = 5;
        let hf_size: [f64; 4] = [2.0, 2.0, 0.5, 0.0];
        let cell_size = 2.0 * hf_size[0] / (n - 1) as f64;
        let heights = vec![0.5_f64; n * n];
        let hfield = HeightFieldData::new(heights, n, n, cell_size);
        model.hfield_data = vec![Arc::new(hfield)];
        model.hfield_size = vec![hf_size];
        model.geom_hfield = vec![Some(0)];

        // Vertex at z=0.45, radius=0.1. Sphere bottom at 0.35, terrain at 0.5.
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.45),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );

        let (depth, normal, _contact_pos) = result.expect("Should generate contact");
        assert!(
            depth > 0.05 && depth < 0.25,
            "depth={depth}, expected ~0.15"
        );
        assert!(normal.z > 0.9, "normal.z={}, expected ~1.0", normal.z);
    }

    // --- T3: Flex vertex sphere vs SDF (AC3) ---

    #[test]
    fn t3_flex_vs_sdf() {
        let mut model = make_single_geom_model(GeomType::Sdf);

        // Sphere SDF, radius 1.0, centered at origin, 32 resolution
        let sdf = SdfCollisionData::sphere(Point3::origin(), 1.0, 32, 1.0);
        model.sdf_data = vec![Arc::new(sdf)];
        model.geom_sdf = vec![Some(0)];

        // Vertex at z=0.95, radius=0.1. Distance to SDF surface = 0.05.
        // Penetration = 0.1 - 0.05 = 0.05.
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.95),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );

        let (depth, normal, _contact_pos) = result.expect("Should generate contact");
        assert!(
            depth > 0.01 && depth < 0.15,
            "depth={depth}, expected ~0.05"
        );
        assert!(normal.z > 0.5, "normal.z={}, expected >0.5", normal.z);
    }

    // --- T4: Missing geom data guards (AC4, AC5, AC6) ---

    #[test]
    fn t4_missing_data_no_panic() {
        // Mesh with no data
        let model_mesh = make_single_geom_model(GeomType::Mesh);
        assert!(model_mesh.geom_mesh[0].is_none());
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.0),
            0.1,
            0,
            &model_mesh,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        assert!(result.is_none(), "Mesh with no data should return None");

        // Hfield with no data
        let model_hfield = make_single_geom_model(GeomType::Hfield);
        assert!(model_hfield.geom_hfield[0].is_none());
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.0),
            0.1,
            0,
            &model_hfield,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        assert!(result.is_none(), "Hfield with no data should return None");

        // SDF with no data
        let model_sdf = make_single_geom_model(GeomType::Sdf);
        assert!(model_sdf.geom_sdf[0].is_none());
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.0),
            0.1,
            0,
            &model_sdf,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        assert!(result.is_none(), "SDF with no data should return None");
    }

    // --- T5: Vertex above surface — no contact (AC7) ---

    #[test]
    fn t5_above_surface_no_contact() {
        // Mesh: vertex well above
        let mut model = make_single_geom_model(GeomType::Mesh);
        model.mesh_data = vec![cube_mesh_with_hull()];
        model.geom_mesh = vec![Some(0)];
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 10.0),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        assert!(result.is_none(), "Vertex above mesh should return None");

        // Hfield: vertex well above
        let mut model = make_single_geom_model(GeomType::Hfield);
        let n = 5;
        let hf_size: [f64; 4] = [2.0, 2.0, 0.5, 0.0];
        let cell_size = 2.0 * hf_size[0] / (n - 1) as f64;
        let heights = vec![0.5_f64; n * n];
        let hfield = HeightFieldData::new(heights, n, n, cell_size);
        model.hfield_data = vec![Arc::new(hfield)];
        model.hfield_size = vec![hf_size];
        model.geom_hfield = vec![Some(0)];
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 10.0),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        assert!(result.is_none(), "Vertex above hfield should return None");

        // SDF: vertex well above
        let mut model = make_single_geom_model(GeomType::Sdf);
        let sdf = SdfCollisionData::sphere(Point3::origin(), 1.0, 32, 1.0);
        model.sdf_data = vec![Arc::new(sdf)];
        model.geom_sdf = vec![Some(0)];
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 10.0),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        assert!(result.is_none(), "Vertex above SDF should return None");
    }

    // --- T6: Per-triangle BVH fallback (AC10) ---

    #[test]
    fn t6_flex_vs_mesh_per_triangle_fallback() {
        let mut model = make_single_geom_model(GeomType::Mesh);
        let mesh = single_triangle_mesh();
        assert!(
            mesh.convex_hull().is_none(),
            "No hull on single-triangle mesh"
        );
        model.mesh_data = vec![mesh];
        model.geom_mesh = vec![Some(0)];

        // Triangle in Z=0 plane. Vertex at z=-0.05, radius=0.1.
        // Sphere top at z=0.05, triangle at z=0.0 → penetrating.
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, -0.05),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );

        let (depth, _normal, _contact_pos) = result.expect("BVH fallback should generate contact");
        assert!(depth > 0.0, "depth={depth}, expected >0");
    }

    // --- TS1: Zero-radius vertex vs mesh ---

    #[test]
    fn ts1_zero_radius_vs_mesh() {
        let mut model = make_single_geom_model(GeomType::Mesh);
        model.mesh_data = vec![cube_mesh_with_hull()];
        model.geom_mesh = vec![Some(0)];

        // Zero-radius sphere inside cube → should not panic
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.0),
            0.0,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        // Zero radius means no penetration extent — result may be None or Some
        // depending on GJK/EPA; the key requirement is no panic.
        let _ = result;
    }

    // --- TS2: Hfield vertex on surface boundary ---

    #[test]
    fn ts2_hfield_vertex_on_surface() {
        let mut model = make_single_geom_model(GeomType::Hfield);
        let n = 5;
        let hf_size: [f64; 4] = [2.0, 2.0, 0.5, 0.0];
        let cell_size = 2.0 * hf_size[0] / (n - 1) as f64;
        let heights = vec![0.5_f64; n * n];
        let hfield = HeightFieldData::new(heights, n, n, cell_size);
        model.hfield_data = vec![Arc::new(hfield)];
        model.hfield_size = vec![hf_size];
        model.geom_hfield = vec![Some(0)];

        // Vertex at z=0.55, radius=0.05 → sphere bottom at 0.50 = terrain height
        // Boundary: penetration ≈ 0. Should not panic regardless of result.
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.55),
            0.05,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        let _ = result; // No panic is the key requirement
    }

    // --- TS3: SDF vertex outside bounding box ---

    #[test]
    fn ts3_sdf_vertex_outside_bbox() {
        let mut model = make_single_geom_model(GeomType::Sdf);
        let sdf = SdfCollisionData::sphere(Point3::origin(), 1.0, 32, 1.0);
        model.sdf_data = vec![Arc::new(sdf)];
        model.geom_sdf = vec![Some(0)];

        // Vertex far outside SDF bounding box
        let result = narrowphase_sphere_geom(
            Vector3::new(100.0, 100.0, 100.0),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        assert!(
            result.is_none(),
            "Vertex outside SDF bbox should return None"
        );
    }

    // --- TS4: Single-cell (2×2) hfield ---

    #[test]
    fn ts4_single_cell_hfield() {
        let mut model = make_single_geom_model(GeomType::Hfield);
        let n = 2; // Minimum valid hfield: 2×2 grid = 1 cell
        let hf_size: [f64; 4] = [1.0, 1.0, 0.5, 0.0];
        let cell_size = 2.0 * hf_size[0] / (n - 1) as f64;
        let heights = vec![0.5_f64; n * n];
        let hfield = HeightFieldData::new(heights, n, n, cell_size);
        model.hfield_data = vec![Arc::new(hfield)];
        model.hfield_size = vec![hf_size];
        model.geom_hfield = vec![Some(0)];

        // Vertex penetrating
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.45),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );

        let (depth, normal, _contact_pos) = result.expect("Should generate contact on 2×2 hfield");
        assert!(depth > 0.0, "depth={depth}");
        assert!(normal.z > 0.5, "normal.z={}", normal.z);
    }
}
