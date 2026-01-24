//! Batch operations for physics simulation hot paths.
//!
//! This module provides high-level batch operations that are commonly used
//! in physics simulation, optimized for SIMD execution.

use nalgebra::{Point3, Vector3};

use crate::{Vec3x4, Vec3x8};

// =============================================================================
// Batch Dot Product Operations
// =============================================================================

/// Compute 4 dot products of vectors with a single direction.
///
/// This is the core operation for GJK support vertex search.
///
/// # Example
///
/// ```
/// use sim_simd::batch_dot_product_4;
/// use nalgebra::Vector3;
///
/// let vertices = [
///     Vector3::new(1.0, 0.0, 0.0),
///     Vector3::new(0.0, 1.0, 0.0),
///     Vector3::new(0.0, 0.0, 1.0),
///     Vector3::new(1.0, 1.0, 1.0),
/// ];
/// let direction = Vector3::new(1.0, 0.0, 0.0);
///
/// let dots = batch_dot_product_4(&vertices, &direction);
/// assert_eq!(dots, [1.0, 0.0, 0.0, 1.0]);
/// ```
#[must_use]
#[inline]
pub fn batch_dot_product_4(vectors: &[Vector3<f64>; 4], direction: &Vector3<f64>) -> [f64; 4] {
    Vec3x4::from_vectors(*vectors).dot(direction)
}

/// Compute dot products of a slice of vectors with a direction (4-wide batches).
///
/// Returns a vector of dot products, one for each input vector.
#[must_use]
pub fn batch_dot_product_slice(vectors: &[Vector3<f64>], direction: &Vector3<f64>) -> Vec<f64> {
    let n = vectors.len();
    let mut result = Vec::with_capacity(n);

    // Process in chunks of 4
    let chunks = n / 4;
    for chunk_idx in 0..chunks {
        let base = chunk_idx * 4;
        let batch = Vec3x4::from_slice(&vectors[base..]);
        let dots = batch.dot(direction);
        result.extend_from_slice(&dots);
    }

    // Handle remainder
    let remainder_start = chunks * 4;
    for v in &vectors[remainder_start..] {
        result.push(v.dot(direction));
    }

    result
}

/// Find the index and value of the maximum dot product in a slice.
///
/// This is optimized for finding support vertices in GJK.
///
/// # Example
///
/// ```
/// use sim_simd::find_max_dot;
/// use nalgebra::Vector3;
///
/// let vertices = vec![
///     Vector3::new(1.0, 0.0, 0.0),
///     Vector3::new(0.0, 2.0, 0.0),
///     Vector3::new(0.0, 0.0, 3.0),
/// ];
/// let direction = Vector3::new(0.0, 1.0, 0.0);
///
/// let (idx, dot) = find_max_dot(&vertices, &direction);
/// assert_eq!(idx, 1);
/// assert_eq!(dot, 2.0);
/// ```
#[must_use]
pub fn find_max_dot(vectors: &[Vector3<f64>], direction: &Vector3<f64>) -> (usize, f64) {
    if vectors.is_empty() {
        return (0, f64::NEG_INFINITY);
    }

    let n = vectors.len();
    let mut global_max_idx = 0;
    let mut global_max_val = f64::NEG_INFINITY;

    // Process in chunks of 8 for better cache utilization
    let chunks = n / 8;
    for chunk_idx in 0..chunks {
        let base = chunk_idx * 8;
        let batch = Vec3x8::from_slice(&vectors[base..]);
        let (local_idx, local_max) = batch.argmax_dot(direction);
        if local_max > global_max_val {
            global_max_val = local_max;
            global_max_idx = base + local_idx;
        }
    }

    // Process remaining in chunks of 4
    let remainder_start_8 = chunks * 8;
    let remaining = n - remainder_start_8;

    if remaining >= 4 {
        let batch = Vec3x4::from_slice(&vectors[remainder_start_8..]);
        let (local_idx, local_max) = batch.argmax_dot(direction);
        if local_max > global_max_val {
            global_max_val = local_max;
            global_max_idx = remainder_start_8 + local_idx;
        }
    }

    // Handle final remainder
    let final_start = remainder_start_8 + (remaining / 4) * 4;
    for (i, v) in vectors[final_start..].iter().enumerate() {
        let dot = v.dot(direction);
        if dot > global_max_val {
            global_max_val = dot;
            global_max_idx = final_start + i;
        }
    }

    (global_max_idx, global_max_val)
}

/// Find the index and value of the minimum dot product in a slice.
#[must_use]
pub fn find_min_dot(vectors: &[Vector3<f64>], direction: &Vector3<f64>) -> (usize, f64) {
    if vectors.is_empty() {
        return (0, f64::INFINITY);
    }

    let n = vectors.len();
    let mut global_min_idx = 0;
    let mut global_min_val = f64::INFINITY;

    // Process in chunks of 8
    let chunks = n / 8;
    for chunk_idx in 0..chunks {
        let base = chunk_idx * 8;
        let batch = Vec3x8::from_slice(&vectors[base..]);
        let (local_idx, local_min) = batch.argmin_dot(direction);
        if local_min < global_min_val {
            global_min_val = local_min;
            global_min_idx = base + local_idx;
        }
    }

    // Handle remainder
    let remainder_start = chunks * 8;
    for (i, v) in vectors[remainder_start..].iter().enumerate() {
        let dot = v.dot(direction);
        if dot < global_min_val {
            global_min_val = dot;
            global_min_idx = remainder_start + i;
        }
    }

    (global_min_idx, global_min_val)
}

// =============================================================================
// AABB Operations
// =============================================================================

/// Batch AABB represented as min/max points.
#[derive(Debug, Clone, Copy)]
pub struct Aabb4 {
    /// Minimum corners (4 AABBs).
    pub mins: Vec3x4,
    /// Maximum corners (4 AABBs).
    pub maxs: Vec3x4,
}

impl Aabb4 {
    /// Create from 4 AABB pairs.
    #[must_use]
    pub fn from_pairs(mins: [Point3<f64>; 4], maxs: [Point3<f64>; 4]) -> Self {
        Self {
            mins: Vec3x4::from_vectors([
                mins[0].coords,
                mins[1].coords,
                mins[2].coords,
                mins[3].coords,
            ]),
            maxs: Vec3x4::from_vectors([
                maxs[0].coords,
                maxs[1].coords,
                maxs[2].coords,
                maxs[3].coords,
            ]),
        }
    }

    /// Test if 4 AABBs overlap with a single AABB.
    ///
    /// Returns a bitmask where bit i is set if AABB i overlaps with the query.
    #[must_use]
    pub fn overlaps_single(&self, query_min: &Point3<f64>, query_max: &Point3<f64>) -> [bool; 4] {
        // AABB overlap test: all axes must overlap
        // overlap = (a.min <= b.max) && (a.max >= b.min) for each axis

        let mut result = [true; 4];

        // X axis
        for i in 0..4 {
            result[i] =
                result[i] && self.mins.xs[i] <= query_max.x && self.maxs.xs[i] >= query_min.x;
        }

        // Y axis
        for i in 0..4 {
            result[i] =
                result[i] && self.mins.ys[i] <= query_max.y && self.maxs.ys[i] >= query_min.y;
        }

        // Z axis
        for i in 0..4 {
            result[i] =
                result[i] && self.mins.zs[i] <= query_max.z && self.maxs.zs[i] >= query_min.z;
        }

        result
    }

    /// Expand all AABBs by a margin.
    #[must_use]
    pub fn expanded(&self, margin: f64) -> Self {
        let margin_vec = Vec3x4::splat(Vector3::new(margin, margin, margin));
        Self {
            mins: self.mins.sub(&margin_vec),
            maxs: self.maxs.add(&margin_vec),
        }
    }
}

/// Test if 4 AABBs pairwise overlap with another 4 AABBs.
///
/// Returns `[a0 overlaps b0, a1 overlaps b1, a2 overlaps b2, a3 overlaps b3]`.
#[must_use]
pub fn batch_aabb_overlap_4(
    a_mins: &Vec3x4,
    a_maxs: &Vec3x4,
    b_mins: &Vec3x4,
    b_maxs: &Vec3x4,
) -> [bool; 4] {
    let mut result = [true; 4];

    // X axis
    for i in 0..4 {
        result[i] = result[i] && a_mins.xs[i] <= b_maxs.xs[i] && a_maxs.xs[i] >= b_mins.xs[i];
    }

    // Y axis
    for i in 0..4 {
        result[i] = result[i] && a_mins.ys[i] <= b_maxs.ys[i] && a_maxs.ys[i] >= b_mins.ys[i];
    }

    // Z axis
    for i in 0..4 {
        result[i] = result[i] && a_mins.zs[i] <= b_maxs.zs[i] && a_maxs.zs[i] >= b_mins.zs[i];
    }

    result
}

// =============================================================================
// Contact Force Operations
// =============================================================================

/// Batch contact force input data.
#[derive(Debug, Clone, Copy)]
pub struct ContactBatch4 {
    /// Penetration depths for 4 contacts.
    pub penetrations: [f64; 4],
    /// Approach velocities for 4 contacts.
    pub approach_velocities: [f64; 4],
    /// Tangent velocities for 4 contacts.
    pub tangent_velocities: Vec3x4,
}

/// Batch contact force output data.
#[derive(Debug, Clone, Copy)]
pub struct ContactForceBatch4 {
    /// Normal force magnitudes.
    pub normal_magnitudes: [f64; 4],
    /// Friction force vectors.
    pub friction_forces: Vec3x4,
}

/// Compute normal force magnitudes for 4 contacts using the spring-damper model.
///
/// Formula: `F_n = k * d^p + c * v_approach`
///
/// # Arguments
///
/// * `penetrations` - Penetration depths (d)
/// * `approach_velocities` - Approach velocities (positive = approaching)
/// * `stiffness` - Spring stiffness (k)
/// * `stiffness_power` - Nonlinear stiffness power (p)
/// * `damping` - Damping coefficient (c)
#[must_use]
#[inline]
pub fn batch_normal_force_4(
    penetrations: &[f64; 4],
    approach_velocities: &[f64; 4],
    stiffness: f64,
    stiffness_power: f64,
    damping: f64,
) -> [f64; 4] {
    let mut result = [0.0; 4];

    for i in 0..4 {
        if penetrations[i] > 0.0 {
            // Spring force: k * d^p
            let spring_force = stiffness * penetrations[i].powf(stiffness_power);

            // Damping force: c * v_approach
            let damping_force = damping * approach_velocities[i];

            // Total (clamped to non-negative)
            result[i] = (spring_force + damping_force).max(0.0);
        }
    }

    result
}

/// Compute regularized friction forces for 4 contacts.
///
/// Uses smooth (regularized) Coulomb friction model.
#[must_use]
#[inline]
pub fn batch_friction_force_4(
    tangent_velocities: &Vec3x4,
    normal_magnitudes: &[f64; 4],
    friction_coefficient: f64,
    regularization_velocity: f64,
) -> Vec3x4 {
    let speeds = tangent_velocities.norm();
    let mut result = Vec3x4::zeros();

    for i in 0..4 {
        if normal_magnitudes[i] > 0.0 && speeds[i] > 1e-10 {
            let max_friction = friction_coefficient * normal_magnitudes[i];

            // Regularized friction: smooth transition near zero velocity
            let friction_mag = if speeds[i] < regularization_velocity {
                max_friction * speeds[i] / regularization_velocity
            } else {
                max_friction
            };

            // Friction opposes motion
            let scale = -friction_mag / speeds[i];
            result.xs[i] = tangent_velocities.xs[i] * scale;
            result.ys[i] = tangent_velocities.ys[i] * scale;
            result.zs[i] = tangent_velocities.zs[i] * scale;
        }
    }

    result
}

// =============================================================================
// Linear Algebra Operations
// =============================================================================

/// Compute 4 AXPY operations: `y[i] = a * x[i] + y[i]`
///
/// This is a common operation in iterative solvers (CG, GMRES).
#[inline]
pub fn batch_axpy_4(a: f64, x: &Vec3x4, y: &mut Vec3x4) {
    for i in 0..4 {
        y.xs[i] = a.mul_add(x.xs[i], y.xs[i]);
    }
    for i in 0..4 {
        y.ys[i] = a.mul_add(x.ys[i], y.ys[i]);
    }
    for i in 0..4 {
        y.zs[i] = a.mul_add(x.zs[i], y.zs[i]);
    }
}

/// Compute sum of 4 dot products (for norm computations).
#[must_use]
#[inline]
pub fn batch_dot_sum_4(a: &Vec3x4, b: &Vec3x4) -> f64 {
    let dots = a.dot_pairwise(b);
    dots[0] + dots[1] + dots[2] + dots[3]
}

/// Batch matrix-vector product for small dense matrices.
///
/// Computes `result[i] = M[i] * v` for 4 3x3 matrices.
#[must_use]
pub fn batch_mat3_vec3_4(matrices: &[[f64; 9]; 4], vectors: &Vec3x4) -> Vec3x4 {
    let mut result = Vec3x4::zeros();

    for i in 0..4 {
        let m = &matrices[i];
        result.xs[i] = m[0] * vectors.xs[i] + m[1] * vectors.ys[i] + m[2] * vectors.zs[i];
        result.ys[i] = m[3] * vectors.xs[i] + m[4] * vectors.ys[i] + m[5] * vectors.zs[i];
        result.zs[i] = m[6] * vectors.xs[i] + m[7] * vectors.ys[i] + m[8] * vectors.zs[i];
    }

    result
}

// =============================================================================
// Integration Operations
// =============================================================================

/// Batch position integration: pos += vel * dt
#[inline]
pub fn batch_integrate_position_4(positions: &mut Vec3x4, velocities: &Vec3x4, dt: f64) {
    for i in 0..4 {
        positions.xs[i] += velocities.xs[i] * dt;
    }
    for i in 0..4 {
        positions.ys[i] += velocities.ys[i] * dt;
    }
    for i in 0..4 {
        positions.zs[i] += velocities.zs[i] * dt;
    }
}

/// Batch velocity integration: vel += accel * dt
#[inline]
pub fn batch_integrate_velocity_4(velocities: &mut Vec3x4, accelerations: &Vec3x4, dt: f64) {
    for i in 0..4 {
        velocities.xs[i] += accelerations.xs[i] * dt;
    }
    for i in 0..4 {
        velocities.ys[i] += accelerations.ys[i] * dt;
    }
    for i in 0..4 {
        velocities.zs[i] += accelerations.zs[i] * dt;
    }
}
