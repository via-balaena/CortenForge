//! Spatial algebra utilities for 6D motion and force vectors.
//!
//! Implements Featherstone's spatial vector algebra used throughout the
//! dynamics pipeline (CRBA, RNE, forward kinematics). Functions here are
//! pure math — no pipeline state dependencies.

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

/// Shift a 6×6 spatial inertia from one reference point to another.
///
/// Given Φ (spatial inertia about point A in world frame) and
/// d = xpos_child - xpos_parent (vector from parent to child origin),
/// returns the equivalent spatial inertia about the parent's origin.
///
/// Uses the approach: extract (I_COM, m, h) from Φ_A, then recompute
/// Φ_B with h_new = h + d (since COM - parent = (COM - child) + (child - parent)).
///
/// Convention: rows 0-2 = angular, rows 3-5 = linear.
/// Off-diagonal coupling block (upper-right 3x3) = m * skew(h).
#[allow(clippy::similar_names)]
pub fn shift_spatial_inertia(phi: &Matrix6<f64>, d: &Vector3<f64>) -> Matrix6<f64> {
    // Extract mass from lower-right diagonal
    let m = phi[(3, 3)];
    if m == 0.0 {
        return *phi;
    }

    // Extract m*h from coupling block: H = m*skew(h)
    // skew(h) = [0, -hz, hy; hz, 0, -hx; -hy, hx, 0]
    // So: phi[(2,4)] = m*hx, phi[(0,5)] = m*hy, phi[(1,3)] = m*hz
    let mh_x = phi[(2, 4)];
    let mh_y = phi[(0, 5)];
    let mh_z = phi[(1, 3)];
    let h_x = mh_x / m;
    let h_y = mh_y / m;
    let h_z = mh_z / m;

    // Extract I_about_A (rotational block, upper-left 3x3)
    // Then reverse parallel axis to get I_COM:
    // I_COM = I_A - m*(|h|²I₃ - h*hᵀ)
    let hh = h_x * h_x + h_y * h_y + h_z * h_z;
    let mut i_com = Matrix3::zeros();
    for row in 0..3 {
        for col in 0..3 {
            let h_rc = [h_x, h_y, h_z];
            let delta = if row == col { 1.0 } else { 0.0 };
            i_com[(row, col)] = phi[(row, col)] - m * (hh * delta - h_rc[row] * h_rc[col]);
        }
    }

    // Compute new h: h_new = h + d (COM offset from parent origin)
    let h_new_x = h_x + d.x;
    let h_new_y = h_y + d.y;
    let h_new_z = h_z + d.z;

    // Build new 6x6 spatial inertia about parent origin
    let h_new = [h_new_x, h_new_y, h_new_z];
    let hh_new = h_new_x * h_new_x + h_new_y * h_new_y + h_new_z * h_new_z;

    let mut result = Matrix6::zeros();

    // Upper-left 3x3: I_parent = I_COM + m*(|h_new|²I₃ - h_new*h_newᵀ)
    for row in 0..3 {
        for col in 0..3 {
            let delta = if row == col { 1.0 } else { 0.0 };
            result[(row, col)] = i_com[(row, col)] + m * (hh_new * delta - h_new[row] * h_new[col]);
        }
    }

    // Lower-right 3x3: mass
    result[(3, 3)] = m;
    result[(4, 4)] = m;
    result[(5, 5)] = m;

    // Off-diagonal coupling: m * skew(h_new)
    let mhn_x = m * h_new_x;
    let mhn_y = m * h_new_y;
    let mhn_z = m * h_new_z;

    result[(0, 4)] = -mhn_z;
    result[(0, 5)] = mhn_y;
    result[(1, 3)] = mhn_z;
    result[(1, 5)] = -mhn_x;
    result[(2, 3)] = -mhn_y;
    result[(2, 4)] = mhn_x;
    // Transpose (lower-left)
    result[(4, 0)] = -mhn_z;
    result[(5, 0)] = mhn_y;
    result[(3, 1)] = mhn_z;
    result[(5, 1)] = -mhn_x;
    result[(3, 2)] = -mhn_y;
    result[(4, 2)] = mhn_x;

    result
}
