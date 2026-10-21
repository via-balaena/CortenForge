//! Spatial algebra utilities for 6D motion and force vectors.
//!
//! Implements Featherstone's spatial vector algebra used throughout the
//! dynamics pipeline (CRBA, RNE, forward kinematics). Functions here are
//! pure math — no pipeline state dependencies.

use crate::types::{Data, Model};
use nalgebra::{Matrix3, Matrix6, Vector3, Vector6};

/// 6D spatial vector: [angular (3), linear (3)].
///
/// Following Featherstone's convention:
/// - Motion vectors: [ω, v] (angular velocity, linear velocity)
/// - Force vectors: [τ, f] (torque, force)
pub type SpatialVector = Vector6<f64>;

/// Spatial cross product for motion vectors: v × s.
#[allow(clippy::inline_always)] // Profiling shows inlining improves debug performance
#[inline(always)]
#[must_use]
pub fn spatial_cross_motion(v: SpatialVector, s: SpatialVector) -> SpatialVector {
    let w = Vector3::new(v[0], v[1], v[2]);
    let v_lin = Vector3::new(v[3], v[4], v[5]);
    let s_ang = Vector3::new(s[0], s[1], s[2]);
    let s_lin = Vector3::new(s[3], s[4], s[5]);

    let result_ang = w.cross(&s_ang);
    let result_lin = w.cross(&s_lin) + v_lin.cross(&s_ang);

    SpatialVector::new(
        result_ang.x,
        result_ang.y,
        result_ang.z,
        result_lin.x,
        result_lin.y,
        result_lin.z,
    )
}

/// Spatial cross product for force vectors: v ×* f.
#[allow(clippy::inline_always)] // Profiling shows inlining improves debug performance
#[inline(always)]
#[must_use]
pub fn spatial_cross_force(v: SpatialVector, f: SpatialVector) -> SpatialVector {
    let w = Vector3::new(v[0], v[1], v[2]);
    let v_lin = Vector3::new(v[3], v[4], v[5]);
    let f_ang = Vector3::new(f[0], f[1], f[2]);
    let f_lin = Vector3::new(f[3], f[4], f[5]);

    let result_ang = w.cross(&f_ang) + v_lin.cross(&f_lin);
    let result_lin = w.cross(&f_lin);

    SpatialVector::new(
        result_ang.x,
        result_ang.y,
        result_ang.z,
        result_lin.x,
        result_lin.y,
        result_lin.z,
    )
}

/// A dof's motion axis about a reference point, MuJoCo's `mju_dofCom`
/// (`engine_util_spatial.c:445-458`): a rotation about `axis` through a point
/// `offset` away from the reference point (`offset` = reference − point) is
/// `[axis; axis × offset]`, a translation along `axis` (`offset` `None`) is
/// `[0; axis]`.
#[must_use]
pub fn mju_dof_com(axis: &Vector3<f64>, offset: Option<&Vector3<f64>>) -> SpatialVector {
    match offset {
        Some(offset) => {
            let lin = axis.cross(offset);
            SpatialVector::new(axis.x, axis.y, axis.z, lin.x, lin.y, lin.z)
        }
        None => SpatialVector::new(0.0, 0.0, 0.0, axis.x, axis.y, axis.z),
    }
}

/// The motion of `dof.len()` dofs at the rates `vec`, `Σ dof[r]·vec[r]`, as
/// MuJoCo's `mju_mulDofVec` (`engine_util_spatial.c:461-470`): one dof is
/// scaled; several are summed from 0, a dof at rate 0 left out
/// (`mju_mulMatTVec`).
#[must_use]
pub fn mju_mul_dof_vec(dof: &[SpatialVector], vec: &[f64]) -> SpatialVector {
    match vec {
        [] => SpatialVector::zeros(),
        [rate] => dof[0] * *rate,
        _ => {
            let mut res = SpatialVector::zeros();
            for (axis, &rate) in dof.iter().zip(vec) {
                if rate != 0.0 {
                    res += axis * rate;
                }
            }
            res
        }
    }
}

/// A spatial vector moved from `oldpos` to `newpos`, then, given
/// `rotnew2old`, turned into that frame: MuJoCo's `mju_transformSpatial`
/// (`engine_util_spatial.c:473-504`). A force's torque loses
/// `(newpos − oldpos) × force`; a motion's linear part loses
/// `(newpos − oldpos) × angular`.
#[must_use]
pub fn mju_transform_spatial(
    vec: &SpatialVector,
    flg_force: bool,
    newpos: &Vector3<f64>,
    oldpos: &Vector3<f64>,
    rotnew2old: Option<&Matrix3<f64>>,
) -> SpatialVector {
    let mut tran = *vec;
    let dif = newpos - oldpos;
    let (moved, by) = if flg_force { (0, 3) } else { (3, 0) };
    let cros = dif.cross(&Vector3::new(vec[by], vec[by + 1], vec[by + 2]));
    for k in 0..3 {
        tran[moved + k] = vec[moved + k] - cros[k];
    }
    let Some(rot) = rotnew2old else {
        return tran;
    };
    // `mji_mulMatTVec3`: each part times the rotation's transpose.
    let turn = |v: usize| {
        let row = |k: usize| {
            rot[(0, k)] * tran[v] + rot[(1, k)] * tran[v + 1] + rot[(2, k)] * tran[v + 2]
        };
        [row(0), row(1), row(2)]
    };
    let (ang, lin) = (turn(0), turn(3));
    SpatialVector::new(ang[0], ang[1], ang[2], lin[0], lin[1], lin[2])
}

/// A body's inertia about a reference point, as MuJoCo's 10-vector
/// `[Ixx, Iyy, Izz, Ixy, Ixz, Iyz, m·dx, m·dy, m·dz, m]`: MuJoCo's
/// `mju_inertCom` (`engine_util_spatial.c:400-431`). `inert` is the
/// principal inertia, `mat` the inertial frame's rotation, `dif` the centre
/// of mass from the reference point.
#[must_use]
pub fn mju_inert_com(
    inert: &Vector3<f64>,
    mat: &Matrix3<f64>,
    dif: &Vector3<f64>,
    mass: f64,
) -> [f64; 10] {
    let m = |k: usize| mat[(k / 3, k % 3)];
    // tmp = diag(inert) · matᵀ, row-major
    let tmp = [
        m(0) * inert[0],
        m(3) * inert[0],
        m(6) * inert[0],
        m(1) * inert[1],
        m(4) * inert[1],
        m(7) * inert[1],
        m(2) * inert[2],
        m(5) * inert[2],
        m(8) * inert[2],
    ];
    let mut res = [
        m(0) * tmp[0] + m(1) * tmp[3] + m(2) * tmp[6],
        m(3) * tmp[1] + m(4) * tmp[4] + m(5) * tmp[7],
        m(6) * tmp[2] + m(7) * tmp[5] + m(8) * tmp[8],
        m(0) * tmp[1] + m(1) * tmp[4] + m(2) * tmp[7],
        m(0) * tmp[2] + m(1) * tmp[5] + m(2) * tmp[8],
        m(3) * tmp[2] + m(4) * tmp[5] + m(5) * tmp[8],
        mass * dif[0],
        mass * dif[1],
        mass * dif[2],
        mass,
    ];
    res[0] += mass * (dif[1] * dif[1] + dif[2] * dif[2]);
    res[1] += mass * (dif[0] * dif[0] + dif[2] * dif[2]);
    res[2] += mass * (dif[0] * dif[0] + dif[1] * dif[1]);
    res[3] -= mass * dif[0] * dif[1];
    res[4] -= mass * dif[0] * dif[2];
    res[5] -= mass * dif[1] * dif[2];
    res
}

/// A 10-vector inertia times a motion vector, the momentum `[I·ω + h×v;
/// m·v − h×ω]` (`h` = `m·dif`): MuJoCo's `mju_mulInertVec`
/// (`engine_util_spatial.c:434-442`).
#[must_use]
pub fn mju_mul_inert_vec(i: &[f64; 10], v: &SpatialVector) -> SpatialVector {
    SpatialVector::new(
        i[0] * v[0] + i[3] * v[1] + i[4] * v[2] - i[8] * v[4] + i[7] * v[5],
        i[3] * v[0] + i[1] * v[1] + i[5] * v[2] + i[8] * v[3] - i[6] * v[5],
        i[4] * v[0] + i[5] * v[1] + i[2] * v[2] - i[7] * v[3] + i[6] * v[4],
        i[8] * v[1] - i[7] * v[2] + i[9] * v[3],
        i[6] * v[2] - i[8] * v[0] + i[9] * v[4],
        i[7] * v[0] - i[6] * v[1] + i[9] * v[5],
    )
}

/// Body `body`'s 6×6 spatial inertia about its origin `xpos`, from the
/// model (`cinert` is MuJoCo's 10-vector about its tree's subtree centre of
/// mass).
#[must_use]
pub fn body_origin_inertia(model: &Model, data: &Data, body: usize) -> Matrix6<f64> {
    compute_body_spatial_inertia(
        model.body_mass[body],
        model.body_inertia[body],
        &data.ximat[body],
        data.xipos[body] - data.xpos[body],
    )
}

/// Body `body`'s spatial velocity `[angular; linear]` about its origin
/// `xpos` (`cvel` is about its tree's subtree centre of mass).
#[must_use]
pub fn body_origin_velocity(model: &Model, data: &Data, body: usize) -> SpatialVector {
    mju_transform_spatial(
        &data.cvel[body],
        false,
        &data.xpos[body],
        &data.subtree_com[model.body_rootid[body]],
        None,
    )
}

/// Compute body spatial inertia in world frame.
///
/// This builds the 6×6 spatial inertia matrix from:
/// - `mass`: body mass
/// - `inertia_diag`: diagonal inertia in body's principal frame
/// - `i_mat`: rotation matrix from inertial frame to world (3×3)
/// - `h`: COM offset from body origin in world frame
///
/// The spatial inertia has the form:
/// ```text
/// I = [I_rot + m*(h·h*I - h⊗h),  m*[h]×  ]
///     [m*[h]×ᵀ,                  m*I_3×3 ]
/// ```
///
/// This is the canonical implementation - computed once per body in FK,
/// then used by both CRBA (as starting point for composite) and RNE (directly).
#[allow(clippy::inline_always)] // Called once per body in FK - inlining avoids function call overhead
#[inline(always)]
pub fn compute_body_spatial_inertia(
    mass: f64,
    inertia_diag: Vector3<f64>,
    i_mat: &Matrix3<f64>,
    h: Vector3<f64>,
) -> Matrix6<f64> {
    // Rotational inertia in world frame: I_world = R * I_diag * R^T
    // Element-wise is faster than matrix ops in debug mode
    let mut i_rot: Matrix3<f64> = Matrix3::zeros();
    for row in 0..3 {
        for col in 0..3 {
            i_rot[(row, col)] = i_mat[(row, 0)] * inertia_diag[0] * i_mat[(col, 0)]
                + i_mat[(row, 1)] * inertia_diag[1] * i_mat[(col, 1)]
                + i_mat[(row, 2)] * inertia_diag[2] * i_mat[(col, 2)];
        }
    }

    let mut crb = Matrix6::zeros();

    // Upper-left 3x3: rotational inertia about body origin (parallel axis theorem)
    let h_dot_h = h.x * h.x + h.y * h.y + h.z * h.z;
    for row in 0..3 {
        for col in 0..3 {
            let h_outer = h[row] * h[col];
            let delta = if row == col { 1.0 } else { 0.0 };
            crb[(row, col)] = i_rot[(row, col)] + mass * (h_dot_h * delta - h_outer);
        }
    }

    // Lower-right 3x3: translational inertia (diagonal mass matrix)
    crb[(3, 3)] = mass;
    crb[(4, 4)] = mass;
    crb[(5, 5)] = mass;

    // Off-diagonal: coupling (skew-symmetric of m*h)
    let mh_x = mass * h.x;
    let mh_y = mass * h.y;
    let mh_z = mass * h.z;
    crb[(0, 4)] = -mh_z;
    crb[(0, 5)] = mh_y;
    crb[(1, 3)] = mh_z;
    crb[(1, 5)] = -mh_x;
    crb[(2, 3)] = -mh_y;
    crb[(2, 4)] = mh_x;
    // Transpose for lower-left
    crb[(4, 0)] = -mh_z;
    crb[(5, 0)] = mh_y;
    crb[(3, 1)] = mh_z;
    crb[(5, 1)] = -mh_x;
    crb[(3, 2)] = -mh_y;
    crb[(4, 2)] = mh_x;

    crb
}

/// Transport a 6D spatial motion vector by the world-frame offset `r` (from the
/// current reference point to the target).
///
/// Pure-translation motion transport: angular unchanged, linear += angular × r:
/// a parent body's velocity or acceleration about its origin brought to a
/// child's origin.
#[inline]
#[must_use]
pub fn transport_motion_spatial(m: SpatialVector, r: &Vector3<f64>) -> SpatialVector {
    let omega = Vector3::new(m[0], m[1], m[2]);
    let lin = Vector3::new(m[3], m[4], m[5]) + omega.cross(r);
    SpatialVector::new(m[0], m[1], m[2], lin.x, lin.y, lin.z)
}

/// Shift a spatial motion vector (velocity or acceleration) from `body_origin`
/// to `target_pos`.
///
/// Motion transport: angular unchanged, linear += angular × r.
/// Equivalent to `mju_transformSpatial` with `flg_force=0` and no rotation.
#[inline]
pub fn transport_motion(
    angular: &Vector3<f64>,
    linear: &Vector3<f64>,
    target_pos: &Vector3<f64>,
    body_origin: &Vector3<f64>,
) -> (Vector3<f64>, Vector3<f64>) {
    let r = target_pos - body_origin;
    (*angular, linear + angular.cross(&r))
}

/// Shift a spatial force vector (wrench) from `body_origin` to `target_pos`.
///
/// Force transport: force unchanged, torque -= r × force.
/// Equivalent to `mju_transformSpatial` with `flg_force=1` and no rotation.
#[inline]
pub fn transport_force(
    torque: &Vector3<f64>,
    force: &Vector3<f64>,
    target_pos: &Vector3<f64>,
    body_origin: &Vector3<f64>,
) -> (Vector3<f64>, Vector3<f64>) {
    let r = target_pos - body_origin;
    (torque - r.cross(force), *force)
}

/// The spatial velocity `(angular, linear)` of a point fixed to a body.
///
/// `target_pos` moves with body `body_id`; the velocity is in the world's
/// orientation or, given `local_rot`, in that frame: MuJoCo's
/// `mj_objectVelocity` (`engine_core_util.c:720-774`), `cvel` moved from the
/// tree's subtree centre of mass to the point; 0 for a body welded to the
/// world.
#[must_use]
pub fn object_velocity(
    model: &Model,
    data: &Data,
    body_id: usize,
    target_pos: &Vector3<f64>,
    local_rot: Option<&Matrix3<f64>>,
) -> (Vector3<f64>, Vector3<f64>) {
    if model.body_weldid[body_id] == 0 {
        return (Vector3::zeros(), Vector3::zeros());
    }
    let res = mju_transform_spatial(
        &data.cvel[body_id],
        false,
        target_pos,
        &data.subtree_com[model.body_rootid[body_id]],
        local_rot,
    );
    (
        Vector3::new(res[0], res[1], res[2]),
        Vector3::new(res[3], res[4], res[5]),
    )
}

/// Compute 6D acceleration at `target_pos` on body `body_id` with Coriolis
/// correction, optionally rotated into a local frame.
///
/// Equivalent to `mj_objectAcceleration(m, d, ..., flg_local)`:
/// 1. Motion transport of `cacc` from body origin to target
/// 2. The velocity at the target (world frame), [`object_velocity`]
/// 3. Coriolis: a_lin += omega × v_lin
/// 4. Optional rotation
///
/// Convention: reads `cacc[body_id]` at `xpos[body_id]`.
#[must_use]
pub fn object_acceleration(
    model: &Model,
    data: &Data,
    body_id: usize,
    target_pos: &Vector3<f64>,
    local_rot: Option<&Matrix3<f64>>,
) -> (Vector3<f64>, Vector3<f64>) {
    let cacc = data.cacc[body_id];
    let alpha = Vector3::new(cacc[0], cacc[1], cacc[2]);
    let a_lin = Vector3::new(cacc[3], cacc[4], cacc[5]);

    let (alpha_at_point, a_at_point) =
        transport_motion(&alpha, &a_lin, target_pos, &data.xpos[body_id]);

    // Velocity at target point (world frame) for Coriolis
    let (omega_at_point, v_at_point) = object_velocity(model, data, body_id, target_pos, None);

    // Coriolis correction: a += omega × v
    let a_corrected = a_at_point + omega_at_point.cross(&v_at_point);

    match local_rot {
        Some(rot) => (
            rot.transpose() * alpha_at_point,
            rot.transpose() * a_corrected,
        ),
        None => (alpha_at_point, a_corrected),
    }
}

/// Compute constraint wrench at `target_pos` on body `body_id`, optionally
/// rotated into a local frame.
///
/// Force transport: force unchanged, torque shifted by lever arm.
/// No Coriolis — wrenches are not velocity-dependent.
///
/// Convention: reads `cfrc_int[body_id]` at `xpos[body_id]`.
#[must_use]
pub fn object_force(
    data: &Data,
    body_id: usize,
    target_pos: &Vector3<f64>,
    local_rot: Option<&Matrix3<f64>>,
) -> (Vector3<f64>, Vector3<f64>) {
    let cfrc = data.cfrc_int[body_id];
    let torque = Vector3::new(cfrc[0], cfrc[1], cfrc[2]);
    let force = Vector3::new(cfrc[3], cfrc[4], cfrc[5]);

    let (torque_at_point, force_at_point) =
        transport_force(&torque, &force, target_pos, &data.xpos[body_id]);

    match local_rot {
        Some(rot) => (
            rot.transpose() * torque_at_point,
            rot.transpose() * force_at_point,
        ),
        None => (torque_at_point, force_at_point),
    }
}

/// Compute 6D velocity at an object center in its local frame.
///
/// Equivalent to MuJoCo's `mj_objectVelocity(m, d, objtype, id, res, flg_local=1)`.
/// Returns `[ω_local; v_local]`.
///
/// This is a backward-compatible wrapper around `object_velocity`. New code
/// should call `object_velocity` directly.
#[must_use]
pub fn object_velocity_local(
    model: &Model,
    data: &Data,
    body_id: usize,
    point: &Vector3<f64>,
    rot: &Matrix3<f64>,
) -> [f64; 6] {
    let (omega, v) = object_velocity(model, data, body_id, point, Some(rot));
    [omega.x, omega.y, omega.z, v.x, v.y, v.z]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// T1: transport_motion — zero offset. Output equals input.
    #[test]
    fn t01_transport_motion_zero_offset() {
        let angular = Vector3::new(0.0, 0.0, 10.0);
        let linear = Vector3::new(1.0, 2.0, 3.0);
        let pos = Vector3::new(5.0, 6.0, 7.0);
        let (ang_out, lin_out) = transport_motion(&angular, &linear, &pos, &pos);
        assert_eq!(ang_out, angular);
        assert_eq!(lin_out, linear);
    }

    /// T2: transport_motion — cross product direction.
    #[test]
    fn t02_transport_motion_cross_product() {
        let angular = Vector3::new(0.0, 0.0, 1.0);
        let linear = Vector3::new(0.0, 0.0, 0.0);
        let origin = Vector3::new(0.0, 0.0, 0.0);
        let target = Vector3::new(1.0, 0.0, 0.0);
        let (ang_out, lin_out) = transport_motion(&angular, &linear, &target, &origin);
        assert_eq!(ang_out, angular);
        assert!((lin_out.x - 0.0).abs() < 1e-15);
        assert!((lin_out.y - 1.0).abs() < 1e-15);
        assert!((lin_out.z - 0.0).abs() < 1e-15);
    }

    /// T2b: transport_motion_spatial — zero offset returns the input unchanged,
    /// and a known ω×r case matches the lever (angular unchanged, linear += ω×r).
    #[test]
    fn t02b_transport_motion_spatial() {
        // Zero offset: identity.
        let m = SpatialVector::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let same = transport_motion_spatial(m, &Vector3::zeros());
        assert_eq!(same, m);

        // ω = ẑ, r = x̂ ⇒ ω×r = ŷ. Angular unchanged, linear gains ŷ.
        let spin = SpatialVector::new(0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
        let out = transport_motion_spatial(spin, &Vector3::new(1.0, 0.0, 0.0));
        assert_eq!(
            Vector3::new(out[0], out[1], out[2]),
            Vector3::new(0.0, 0.0, 1.0)
        );
        assert!((out[3] - 0.0).abs() < 1e-15);
        assert!((out[4] - 1.0).abs() < 1e-15);
        assert!((out[5] - 0.0).abs() < 1e-15);
    }

    /// T3: transport_force — moment arm.
    #[test]
    fn t03_transport_force_moment_arm() {
        let torque = Vector3::new(0.0, 0.0, 0.0);
        let force = Vector3::new(0.0, 0.0, 1.0);
        let origin = Vector3::new(0.0, 0.0, 0.0);
        let target = Vector3::new(1.0, 0.0, 0.0);
        let (torque_out, force_out) = transport_force(&torque, &force, &target, &origin);
        assert_eq!(force_out, force);
        assert!((torque_out.x - 0.0).abs() < 1e-15);
        assert!((torque_out.y - (1.0)).abs() < 1e-15);
        assert!((torque_out.z - 0.0).abs() < 1e-15);
    }

    /// T4: transport_force — force invariance across multiple offsets.
    #[test]
    fn t04_transport_force_invariance() {
        let torque = Vector3::new(1.0, 2.0, 3.0);
        let force = Vector3::new(3.0, 7.0, 11.0);
        let origin = Vector3::new(0.0, 0.0, 0.0);

        let offsets = [
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.5, -0.3, 0.8),
        ];
        for offset in &offsets {
            let (_, force_out) = transport_force(&torque, &force, offset, &origin);
            assert_eq!(
                force_out, force,
                "Force should be invariant for offset {offset:?}"
            );
        }
    }
}
