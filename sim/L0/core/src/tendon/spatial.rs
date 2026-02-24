//! Spatial tendon kinematics — 3D pairwise routing with geometry wrapping.
//!
//! Implements the spatial tendon path computation from MuJoCo's
//! `engine_core_smooth.c`, including Jacobian accumulation and
//! tendon force application.

use super::wrap_math::{WrapResult, cylinder_wrap, sphere_wrap};
use crate::types::{Data, GeomType, MjJointType, Model, TendonType, WrapType};
use nalgebra::{DVector, UnitQuaternion, Vector3};

/// Spatial tendon kinematics for a single tendon.
///
/// Computes tendon length as the sum of straight-line segment distances (and
/// wrapping arcs when geom wrapping is active), with optional pulley divisor
/// scaling. Populates `data.ten_J[t]` with the tendon Jacobian via
/// `accumulate_point_jacobian`.
///
/// Algorithm follows MuJoCo's pairwise loop over the wrap array
/// (`engine_core_smooth.c`, function `mj_tendon`).
#[allow(clippy::cast_possible_wrap, clippy::cast_possible_truncation)] // geom_id as i32: safe for any practical model size (< i32::MAX geoms)
pub fn mj_fwd_tendon_spatial(model: &Model, data: &mut Data, t: usize, wrapcount: &mut usize) {
    let adr = model.tendon_adr[t];
    let num = model.tendon_num[t];
    data.ten_J[t].fill(0.0);
    let mut total_length = 0.0;
    let mut divisor: f64 = 1.0;

    // §40b: record wrap path start address
    data.ten_wrapadr[t] = *wrapcount;
    data.ten_wrapnum[t] = 0;

    if num < 2 {
        data.ten_length[t] = 0.0;
        return; // degenerate — validation should prevent this
    }

    let mut j = 0;
    while j < num - 1 {
        let type0 = model.wrap_type[adr + j];
        let type1 = model.wrap_type[adr + j + 1];

        // ---- Pulley handling ----
        // MuJoCo processes pulleys as a pair-skip: when a pulley appears as
        // type0 or type1, advance j by 1 without processing a segment.
        // Divisor is updated only when the pulley is type0.
        if type0 == WrapType::Pulley || type1 == WrapType::Pulley {
            if type0 == WrapType::Pulley {
                divisor = model.wrap_prm[adr + j];
                // §40b: store pulley marker
                debug_assert!(
                    *wrapcount < data.wrap_xpos.len(),
                    "wrap path overflow: wrapcount={} >= capacity={}",
                    *wrapcount,
                    data.wrap_xpos.len()
                );
                data.wrap_xpos[*wrapcount] = Vector3::zeros();
                data.wrap_obj[*wrapcount] = -2;
                data.ten_wrapnum[t] += 1;
                *wrapcount += 1;
            }
            j += 1;
            continue;
        }

        // ---- At this point, type0 must be a Site ----
        debug_assert!(
            type0 == WrapType::Site,
            "type0 must be Site after pulley check"
        );
        let id0 = model.wrap_objid[adr + j];
        let p0 = data.site_xpos[id0];
        let body0 = model.site_body[id0];

        if type1 == WrapType::Site {
            // ---- Site–Site: straight segment ----
            let id1 = model.wrap_objid[adr + j + 1];
            let p1 = data.site_xpos[id1];
            let body1 = model.site_body[id1];

            // §40b: store leading site
            debug_assert!(
                *wrapcount < data.wrap_xpos.len(),
                "wrap path overflow: wrapcount={} >= capacity={}",
                *wrapcount,
                data.wrap_xpos.len()
            );
            data.wrap_xpos[*wrapcount] = p0;
            data.wrap_obj[*wrapcount] = -1;
            data.ten_wrapnum[t] += 1;
            *wrapcount += 1;

            let diff = p1 - p0;
            let dist = diff.norm();
            // Length: accumulated unconditionally (matching MuJoCo).
            total_length += dist / divisor;
            // Jacobian: guarded by distance threshold to avoid bogus direction.
            if dist > 1e-10 && body0 != body1 {
                let dir = diff / dist;
                accumulate_point_jacobian(
                    model,
                    &data.xpos,
                    &data.xquat,
                    &mut data.ten_J[t],
                    body1,
                    &p1,
                    &dir,
                    1.0 / divisor,
                );
                accumulate_point_jacobian(
                    model,
                    &data.xpos,
                    &data.xquat,
                    &mut data.ten_J[t],
                    body0,
                    &p0,
                    &dir,
                    -1.0 / divisor,
                );
            }

            j += 1; // advance to site1 (it becomes type0 on next iteration)
        } else if type1 == WrapType::Geom {
            // ---- Site–Geom–Site: wrapping segment ----
            // Safety: j+2 < num is guaranteed by validation rules 1-2.
            debug_assert!(j + 2 < num, "Geom at j+1 must be followed by Site at j+2");
            let geom_id = model.wrap_objid[adr + j + 1];
            let geom_body = model.geom_body[geom_id];

            // The site AFTER the geom is at j+2
            let id1 = model.wrap_objid[adr + j + 2];
            let p1 = data.site_xpos[id1];
            let body1 = model.site_body[id1];

            // Transform site positions into geom-local frame
            let geom_pos = data.geom_xpos[geom_id];
            let geom_mat = data.geom_xmat[geom_id];
            let p0_local = geom_mat.transpose() * (p0 - geom_pos);
            let p1_local = geom_mat.transpose() * (p1 - geom_pos);

            // Resolve sidesite (if specified) in geom-local frame
            #[allow(clippy::if_not_else)] // Matches spec pseudocode ordering (§4.3).
            let sidesite_local = if model.wrap_sidesite[adr + j + 1] != usize::MAX {
                let ss_id = model.wrap_sidesite[adr + j + 1];
                Some(geom_mat.transpose() * (data.site_xpos[ss_id] - geom_pos))
            } else {
                None
            };

            // Dispatch to wrapping geometry
            let wrap_result = match model.geom_type[geom_id] {
                GeomType::Sphere => sphere_wrap(
                    p0_local,
                    p1_local,
                    model.geom_size[geom_id].x,
                    sidesite_local,
                ),
                GeomType::Cylinder => cylinder_wrap(
                    p0_local,
                    p1_local,
                    model.geom_size[geom_id].x,
                    sidesite_local,
                ),
                _ => unreachable!("wrapping geom type validated at model build"),
            };

            match wrap_result {
                WrapResult::Wrapped {
                    tangent_point_1,
                    tangent_point_2,
                    arc_length,
                } => {
                    // Transform tangent points back to world frame
                    let t1 = geom_pos + geom_mat * tangent_point_1;
                    let t2 = geom_pos + geom_mat * tangent_point_2;

                    // §40b: store 3 points: site0, tangent₁, tangent₂
                    debug_assert!(
                        *wrapcount + 2 < data.wrap_xpos.len(),
                        "wrap path overflow: wrapcount+2={} >= capacity={}",
                        *wrapcount + 2,
                        data.wrap_xpos.len()
                    );
                    data.wrap_xpos[*wrapcount] = p0;
                    data.wrap_obj[*wrapcount] = -1;
                    data.wrap_xpos[*wrapcount + 1] = t1;
                    data.wrap_obj[*wrapcount + 1] = geom_id as i32;
                    data.wrap_xpos[*wrapcount + 2] = t2;
                    data.wrap_obj[*wrapcount + 2] = geom_id as i32;
                    data.ten_wrapnum[t] += 3;
                    *wrapcount += 3;

                    // 3 sub-segments: [p0→t1, t1→t2 (arc), t2→p1]
                    let d1 = t1 - p0;
                    let dist1 = d1.norm();
                    let d3 = p1 - t2;
                    let dist3 = d3.norm();
                    total_length += (dist1 + arc_length + dist3) / divisor;

                    // Sub-segment 1: p0 (body0) → t1 (geom_body)
                    if dist1 > 1e-10 && body0 != geom_body {
                        let dir1 = d1 / dist1;
                        accumulate_point_jacobian(
                            model,
                            &data.xpos,
                            &data.xquat,
                            &mut data.ten_J[t],
                            geom_body,
                            &t1,
                            &dir1,
                            1.0 / divisor,
                        );
                        accumulate_point_jacobian(
                            model,
                            &data.xpos,
                            &data.xquat,
                            &mut data.ten_J[t],
                            body0,
                            &p0,
                            &dir1,
                            -1.0 / divisor,
                        );
                    }

                    // Sub-segment 2: t1 → t2 (arc on geom surface)
                    // Both endpoints on geom_body → Jacobian difference is zero.

                    // Sub-segment 3: t2 (geom_body) → p1 (body1)
                    if dist3 > 1e-10 && geom_body != body1 {
                        let dir3 = d3 / dist3;
                        accumulate_point_jacobian(
                            model,
                            &data.xpos,
                            &data.xquat,
                            &mut data.ten_J[t],
                            body1,
                            &p1,
                            &dir3,
                            1.0 / divisor,
                        );
                        accumulate_point_jacobian(
                            model,
                            &data.xpos,
                            &data.xquat,
                            &mut data.ten_J[t],
                            geom_body,
                            &t2,
                            &dir3,
                            -1.0 / divisor,
                        );
                    }
                }
                WrapResult::NoWrap => {
                    // §40b: store 1 point: site0 only (straight line, no tangent points)
                    debug_assert!(
                        *wrapcount < data.wrap_xpos.len(),
                        "wrap path overflow: wrapcount={} >= capacity={}",
                        *wrapcount,
                        data.wrap_xpos.len()
                    );
                    data.wrap_xpos[*wrapcount] = p0;
                    data.wrap_obj[*wrapcount] = -1;
                    data.ten_wrapnum[t] += 1;
                    *wrapcount += 1;

                    // No wrapping — straight segment p0 → p1
                    let diff = p1 - p0;
                    let dist = diff.norm();
                    total_length += dist / divisor;
                    if dist > 1e-10 && body0 != body1 {
                        let dir = diff / dist;
                        accumulate_point_jacobian(
                            model,
                            &data.xpos,
                            &data.xquat,
                            &mut data.ten_J[t],
                            body1,
                            &p1,
                            &dir,
                            1.0 / divisor,
                        );
                        accumulate_point_jacobian(
                            model,
                            &data.xpos,
                            &data.xquat,
                            &mut data.ten_J[t],
                            body0,
                            &p0,
                            &dir,
                            -1.0 / divisor,
                        );
                    }
                }
            }

            j += 2; // advance past the geom to site1 (becomes type0 next)
        } else {
            unreachable!(
                "type0 must be Site after pulley check; \
                 validation ensures path starts with Site and \
                 geoms are always followed by sites"
            );
        }

        // §40b: store final site (endpoint of current segment)
        // After j advance, j points at the endpoint site. Store it if this
        // is the last element or the next element is a pulley.
        let at_end = j == num - 1;
        let before_pulley = j < num - 1 && model.wrap_type[adr + j + 1] == WrapType::Pulley;
        if at_end || before_pulley {
            let endpoint_id = model.wrap_objid[adr + j];
            debug_assert!(
                *wrapcount < data.wrap_xpos.len(),
                "wrap path overflow: wrapcount={} >= capacity={}",
                *wrapcount,
                data.wrap_xpos.len()
            );
            data.wrap_xpos[*wrapcount] = data.site_xpos[endpoint_id];
            data.wrap_obj[*wrapcount] = -1;
            data.ten_wrapnum[t] += 1;
            *wrapcount += 1;
        }
    }

    data.ten_length[t] = total_length;
}

/// Walk the kinematic chain from `body_id` to root, accumulating each joint's
/// velocity contribution into the tendon Jacobian row `ten_j`.
///
/// For each joint on the chain, projects the joint's velocity contribution
/// through `direction` and scales by `scale`. This is the same kinematic chain
/// walk as `compute_contact_jacobian`'s `add_body_jacobian` closure, but
/// operating on a `DVector<f64>` (1×nv) instead of a `DMatrix` row.
///
/// Uses body-frame axes (`R*e_i`) for free joint angular DOFs, matching
/// MuJoCo's `cdof` convention.
#[allow(clippy::too_many_arguments)]
pub fn accumulate_point_jacobian(
    model: &Model,
    xpos: &[Vector3<f64>],
    xquat: &[UnitQuaternion<f64>],
    ten_j: &mut DVector<f64>,
    body_id: usize,
    point: &Vector3<f64>,
    direction: &Vector3<f64>,
    scale: f64,
) {
    if body_id == 0 {
        return; // world body has no DOFs
    }

    let mut current = body_id;
    while current != 0 {
        let jnt_start = model.body_jnt_adr[current];
        let jnt_end = jnt_start + model.body_jnt_num[current];
        for jnt_id in jnt_start..jnt_end {
            let dof = model.jnt_dof_adr[jnt_id];
            let jnt_body = model.jnt_body[jnt_id];
            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    let axis = xquat[jnt_body] * model.jnt_axis[jnt_id];
                    let jpos = xpos[jnt_body] + xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = point - jpos;
                    ten_j[dof] += scale * direction.dot(&axis.cross(&r));
                }
                MjJointType::Slide => {
                    let axis = xquat[jnt_body] * model.jnt_axis[jnt_id];
                    ten_j[dof] += scale * direction.dot(&axis);
                }
                MjJointType::Ball => {
                    let jpos = xpos[jnt_body] + xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = point - jpos;
                    let rot = xquat[jnt_body].to_rotation_matrix();
                    for i in 0..3 {
                        let omega = rot * Vector3::ith(i, 1.0);
                        ten_j[dof + i] += scale * direction.dot(&omega.cross(&r));
                    }
                }
                MjJointType::Free => {
                    // Translational DOFs: direction projects directly.
                    ten_j[dof] += scale * direction.x;
                    ten_j[dof + 1] += scale * direction.y;
                    ten_j[dof + 2] += scale * direction.z;
                    // Rotational DOFs: body-frame axes (R*e_i), matching MuJoCo's
                    // cdof convention.
                    let jpos = xpos[jnt_body];
                    let r = point - jpos;
                    let rot = xquat[jnt_body].to_rotation_matrix();
                    for i in 0..3 {
                        let omega = rot * Vector3::ith(i, 1.0);
                        ten_j[dof + 3 + i] += scale * direction.dot(&omega.cross(&r));
                    }
                }
            }
        }
        current = model.body_parent[current];
    }
}

/// Map a scalar tendon force to generalized forces via J^T.
///
/// For `TendonType::Fixed`, uses the sparse wrap-array pattern (DOF addresses
/// and coefficients stored in `wrap_objid`/`wrap_prm`). For
/// `TendonType::Spatial`, uses the dense Jacobian row `ten_j` computed by
/// `mj_fwd_tendon_spatial()`.
pub fn apply_tendon_force(
    model: &Model,
    ten_j: &DVector<f64>,
    tendon_type: TendonType,
    t: usize,
    force: f64,
    target: &mut DVector<f64>,
) {
    match tendon_type {
        TendonType::Fixed => {
            let adr = model.tendon_adr[t];
            let num = model.tendon_num[t];
            for w in adr..(adr + num) {
                let dof_adr = model.wrap_objid[w];
                let coef = model.wrap_prm[w];
                if dof_adr < model.nv {
                    target[dof_adr] += coef * force;
                }
            }
        }
        TendonType::Spatial => {
            for dof in 0..model.nv {
                let j = ten_j[dof];
                if j != 0.0 {
                    target[dof] += j * force;
                }
            }
        }
    }
}

/// Compute quaternion difference as axis-angle 3-vector.
///
/// Satisfies `qb * quat(res) = qa`, matching MuJoCo's `mju_subQuat`.
/// Returns the rotation from `qb` to `qa` expressed as an expmap vector.
pub fn subquat(qa: &UnitQuaternion<f64>, qb: &UnitQuaternion<f64>) -> Vector3<f64> {
    // dq = conjugate(qb) * qa  — relative quaternion in qb's frame
    let dq = qb.conjugate() * qa;
    let xyz = Vector3::new(dq.i, dq.j, dq.k);
    let sin_half = xyz.norm();
    if sin_half < 1e-14 {
        // Small-angle limit: atan2(0, ~1) → 0, so result is zero vector.
        return Vector3::zeros();
    }
    let axis = xyz / sin_half;
    let mut angle = 2.0 * sin_half.atan2(dq.w);
    // Shortest path
    if angle > std::f64::consts::PI {
        angle -= 2.0 * std::f64::consts::PI;
    }
    axis * angle
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod subquat_tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn identity_difference_is_zero() {
        let q = UnitQuaternion::identity();
        let res = subquat(&q, &q);
        assert_relative_eq!(res.norm(), 0.0, epsilon = 1e-14);
    }

    #[test]
    fn ninety_degrees_about_x() {
        let angle = std::f64::consts::FRAC_PI_2;
        let qa = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle);
        let qb = UnitQuaternion::identity();
        let res = subquat(&qa, &qb);
        // Should be (π/2, 0, 0)
        assert_relative_eq!(res.x, angle, epsilon = 1e-12);
        assert_relative_eq!(res.y, 0.0, epsilon = 1e-12);
        assert_relative_eq!(res.z, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn ninety_degrees_about_z() {
        let angle = std::f64::consts::FRAC_PI_2;
        let qa = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), angle);
        let qb = UnitQuaternion::identity();
        let res = subquat(&qa, &qb);
        assert_relative_eq!(res.x, 0.0, epsilon = 1e-12);
        assert_relative_eq!(res.y, 0.0, epsilon = 1e-12);
        assert_relative_eq!(res.z, angle, epsilon = 1e-12);
    }

    #[test]
    fn opposite_quaternions_give_pi() {
        // 180° about y
        let qa = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f64::consts::PI);
        let qb = UnitQuaternion::identity();
        let res = subquat(&qa, &qb);
        // Magnitude should be π
        assert_relative_eq!(res.norm(), std::f64::consts::PI, epsilon = 1e-10);
    }

    #[test]
    fn shortest_path_wraps() {
        // qa = 270° about x = -90° via shortest path
        let qa =
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 3.0 * std::f64::consts::FRAC_PI_2);
        let qb = UnitQuaternion::identity();
        let res = subquat(&qa, &qb);
        // Should report -π/2 about x (shortest path)
        assert_relative_eq!(res.x, -std::f64::consts::FRAC_PI_2, epsilon = 1e-10);
        assert_relative_eq!(res.norm(), std::f64::consts::FRAC_PI_2, epsilon = 1e-10);
    }

    #[test]
    fn relative_rotation() {
        // qa = 60° about z, qb = 20° about z → difference = 40° about z
        let qa = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 60.0_f64.to_radians());
        let qb = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 20.0_f64.to_radians());
        let res = subquat(&qa, &qb);
        assert_relative_eq!(res.z, 40.0_f64.to_radians(), epsilon = 1e-10);
        assert_relative_eq!(res.x, 0.0, epsilon = 1e-12);
        assert_relative_eq!(res.y, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn small_angle_near_zero() {
        let tiny = 1e-15;
        let qa = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), tiny);
        let qb = UnitQuaternion::identity();
        let res = subquat(&qa, &qb);
        // Should be zero vector (within small-angle guard)
        assert_relative_eq!(res.norm(), 0.0, epsilon = 1e-10);
    }
}
