//! Spatial tendon kinematics — 3D pairwise routing with geometry wrapping.
//!
//! Implements the spatial tendon path computation from MuJoCo's
//! `engine_core_smooth.c`, including Jacobian accumulation and
//! tendon force application.

use super::wrap_math::{WrapResult, cylinder_wrap, sphere_wrap};
use crate::jacobian::{mark_merged_chain, mj_jac};
use crate::quat::normalize3;
use crate::types::{Data, GeomType, Model, TendonType, WrapType};
use nalgebra::{DVector, UnitQuaternion, Vector3};

/// Spatial tendon kinematics for a single tendon.
///
/// Computes tendon length as the sum of straight-line segment distances (and
/// wrapping arcs when geom wrapping is active), with optional pulley divisor
/// scaling. Populates `data.ten_J[t]` with the tendon Jacobian via
/// `add_segment_jacobian`.
///
/// Algorithm follows MuJoCo's pairwise loop over the wrap array
/// (`engine_core_smooth.c`, function `mj_tendon`).
#[allow(clippy::cast_possible_wrap, clippy::cast_possible_truncation)] // geom_id as i32: safe for any practical model size (< i32::MAX geoms)
#[allow(clippy::unreachable)] // wrap-type invariants enforced by model build validation
pub fn mj_fwd_tendon_spatial(model: &Model, data: &mut Data, t: usize, wrapcount: &mut usize) {
    let adr = model.tendon_adr[t];
    let num = model.tendon_num[t];
    data.ten_J[t].fill(0.0);
    let mut chain = vec![false; model.nv];
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
            add_segment_jacobian(model, data, t, &mut chain, body0, &p0, body1, &p1, divisor);

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
                    add_segment_jacobian(
                        model, data, t, &mut chain, body0, &p0, geom_body, &t1, divisor,
                    );

                    // Sub-segment 2: t1 → t2 (arc on geom surface)
                    // Both endpoints on geom_body → Jacobian difference is zero.

                    // Sub-segment 3: t2 (geom_body) → p1 (body1)
                    add_segment_jacobian(
                        model, data, t, &mut chain, geom_body, &t2, body1, &p1, divisor,
                    );
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
                    add_segment_jacobian(
                        model, data, t, &mut chain, body0, &p0, body1, &p1, divisor,
                    );
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
    data.ten_J_rownnz[t] = chain.iter().filter(|&&c| c).count();
}

/// Add a straight segment's term to tendon `t`'s Jacobian, as MuJoCo 3.5.0's
/// `mj_tendon` does for consecutive points on different bodies
/// (`engine_core_smooth.c:1054-1098`): the segment's direction, near point
/// `p0` (on `b0`) to far point `p1` (on `b1`), normalized as
/// `mju_normalize3`; the points' Jacobians' difference, far minus near
/// ([`mj_jac`]), projected on it as `mju_mulMatTVec` projects (a zero
/// component skipped); added scaled by `1 / divisor`. The two bodies' merged
/// chain joins the tendon's `chain` (MuJoCo's sparse row, `:1063-1082`).
// The segment's ends and the tendon it belongs to.
#[allow(clippy::too_many_arguments)]
fn add_segment_jacobian(
    model: &Model,
    data: &mut Data,
    t: usize,
    chain: &mut [bool],
    b0: usize,
    p0: &Vector3<f64>,
    b1: usize,
    p1: &Vector3<f64>,
    divisor: f64,
) {
    if b0 == b1 {
        return;
    }
    mark_merged_chain(model, b0, b1, chain);
    let mut dif = [p1.x - p0.x, p1.y - p0.y, p1.z - p0.z];
    normalize3(&mut dif);
    let jacdif = mj_jac(model, data, b1, p1).0 - mj_jac(model, data, b0, p0).0;
    let scl = 1.0 / divisor;
    for i in 0..model.nv {
        let mut tmp = 0.0;
        for r in 0..3 {
            if dif[r] != 0.0 {
                tmp += jacdif[(r, i)] * dif[r];
            }
        }
        data.ten_J[t][i] += tmp * scl;
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
