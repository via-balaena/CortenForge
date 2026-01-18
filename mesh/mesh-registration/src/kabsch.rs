//! Kabsch algorithm for computing optimal rigid transformations.
//!
//! The Kabsch algorithm finds the optimal rotation matrix that minimizes
//! the RMSD (root mean square deviation) between two paired sets of points.

use crate::{RegistrationError, RegistrationResult, RigidTransform};
use nalgebra::{Matrix3, Point3, Rotation3, UnitQuaternion, Vector3};

/// Computes the optimal rigid transform that aligns source points to target points.
///
/// Uses the Kabsch algorithm (SVD-based) to find the rotation that minimizes
/// the root mean square deviation between the point sets.
///
/// # Arguments
///
/// * `source_points` - Points to be transformed
/// * `target_points` - Target points to align to
/// * `compute_scale` - If true, also computes optimal uniform scale
///
/// # Returns
///
/// The optimal rigid transform that minimizes alignment error.
///
/// # Errors
///
/// Returns an error if:
/// - The point sets have different lengths
/// - Either point set is empty
/// - SVD computation fails (degenerate configuration)
///
/// # Example
///
/// ```
/// use mesh_registration::compute_rigid_transform;
/// use nalgebra::Point3;
///
/// let source = vec![
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 0.0, 0.0),
///     Point3::new(0.0, 1.0, 0.0),
/// ];
///
/// // Target is source translated by (1, 2, 3)
/// let target = vec![
///     Point3::new(1.0, 2.0, 3.0),
///     Point3::new(2.0, 2.0, 3.0),
///     Point3::new(1.0, 3.0, 3.0),
/// ];
///
/// let transform = compute_rigid_transform(&source, &target, false).unwrap();
///
/// // Apply transform to first source point
/// let aligned = transform.transform_point(&source[0]);
/// assert!((aligned.coords - target[0].coords).norm() < 1e-6);
/// ```
pub fn compute_rigid_transform(
    source_points: &[Point3<f64>],
    target_points: &[Point3<f64>],
    compute_scale: bool,
) -> RegistrationResult<RigidTransform> {
    if source_points.is_empty() {
        return Err(RegistrationError::EmptySourceMesh);
    }
    if target_points.is_empty() {
        return Err(RegistrationError::EmptyTargetMesh);
    }
    if source_points.len() != target_points.len() {
        return Err(RegistrationError::InvalidParameter(format!(
            "point sets must have equal length: {} vs {}",
            source_points.len(),
            target_points.len()
        )));
    }

    // Compute centroids
    let source_centroid = compute_centroid(source_points);
    let target_centroid = compute_centroid(target_points);

    // Center the point sets
    let source_centered: Vec<Vector3<f64>> = source_points
        .iter()
        .map(|p| p.coords - source_centroid)
        .collect();
    let target_centered: Vec<Vector3<f64>> = target_points
        .iter()
        .map(|p| p.coords - target_centroid)
        .collect();

    // Compute the covariance matrix H = sum(source_i * target_i^T)
    let mut h = Matrix3::zeros();
    for (s, t) in source_centered.iter().zip(target_centered.iter()) {
        h += s * t.transpose();
    }

    // SVD of H
    let svd = h.svd(true, true);
    let u = svd.u.ok_or(RegistrationError::SvdFailed)?;
    let v_t = svd.v_t.ok_or(RegistrationError::SvdFailed)?;

    // Compute rotation R = V * U^T
    let mut rotation_matrix = v_t.transpose() * u.transpose();

    // Handle reflection case (det(R) = -1)
    if rotation_matrix.determinant() < 0.0 {
        // Flip the sign of the last column of V
        let mut v = v_t.transpose();
        for i in 0..3 {
            v[(i, 2)] = -v[(i, 2)];
        }
        rotation_matrix = v * u.transpose();
    }

    let rotation = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rotation_matrix));

    // Compute scale if requested
    let scale = if compute_scale {
        compute_optimal_scale(&source_centered, &target_centered, &rotation)
    } else {
        1.0
    };

    // Compute translation: t = target_centroid - scale * R * source_centroid
    let translation = target_centroid - scale * (rotation * source_centroid);

    Ok(RigidTransform::with_scale(rotation, translation, scale))
}

/// Computes the optimal rigid transform using weighted point correspondences.
///
/// Each correspondence can have a different weight, allowing for robust
/// estimation when some correspondences are more reliable than others.
///
/// # Arguments
///
/// * `source_points` - Points to be transformed
/// * `target_points` - Target points to align to
/// * `weights` - Weight for each correspondence (must sum to > 0)
/// * `compute_scale` - If true, also computes optimal uniform scale
///
/// # Errors
///
/// Returns an error if:
/// - The point sets have different lengths
/// - Weights length doesn't match point sets
/// - Total weight is zero or negative
/// - SVD computation fails
pub fn compute_weighted_rigid_transform(
    source_points: &[Point3<f64>],
    target_points: &[Point3<f64>],
    weights: &[f64],
    compute_scale: bool,
) -> RegistrationResult<RigidTransform> {
    if source_points.is_empty() {
        return Err(RegistrationError::EmptySourceMesh);
    }
    if target_points.is_empty() {
        return Err(RegistrationError::EmptyTargetMesh);
    }
    if source_points.len() != target_points.len() || source_points.len() != weights.len() {
        return Err(RegistrationError::InvalidParameter(
            "point sets and weights must have equal length".to_string(),
        ));
    }

    let total_weight: f64 = weights.iter().sum();
    if total_weight <= 0.0 {
        return Err(RegistrationError::InvalidParameter(
            "total weight must be positive".to_string(),
        ));
    }

    // Compute weighted centroids
    let source_centroid = compute_weighted_centroid(source_points, weights);
    let target_centroid = compute_weighted_centroid(target_points, weights);

    // Center the point sets
    let source_centered: Vec<Vector3<f64>> = source_points
        .iter()
        .map(|p| p.coords - source_centroid)
        .collect();
    let target_centered: Vec<Vector3<f64>> = target_points
        .iter()
        .map(|p| p.coords - target_centroid)
        .collect();

    // Compute weighted covariance matrix
    let mut h = Matrix3::zeros();
    for ((s, t), &w) in source_centered
        .iter()
        .zip(target_centered.iter())
        .zip(weights.iter())
    {
        h += w * s * t.transpose();
    }

    // SVD of H
    let svd = h.svd(true, true);
    let u = svd.u.ok_or(RegistrationError::SvdFailed)?;
    let v_t = svd.v_t.ok_or(RegistrationError::SvdFailed)?;

    // Compute rotation R = V * U^T
    let mut rotation_matrix = v_t.transpose() * u.transpose();

    // Handle reflection case
    if rotation_matrix.determinant() < 0.0 {
        let mut v = v_t.transpose();
        for i in 0..3 {
            v[(i, 2)] = -v[(i, 2)];
        }
        rotation_matrix = v * u.transpose();
    }

    let rotation = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rotation_matrix));

    // Compute scale if requested
    let scale = if compute_scale {
        compute_weighted_optimal_scale(&source_centered, &target_centered, weights, &rotation)
    } else {
        1.0
    };

    // Compute translation
    let translation = target_centroid - scale * (rotation * source_centroid);

    Ok(RigidTransform::with_scale(rotation, translation, scale))
}

/// Computes the centroid of a set of points.
fn compute_centroid(points: &[Point3<f64>]) -> Vector3<f64> {
    #[allow(clippy::cast_precision_loss)]
    let n = points.len() as f64;
    let sum: Vector3<f64> = points.iter().map(|p| p.coords).sum();
    sum / n
}

/// Computes the weighted centroid of a set of points.
fn compute_weighted_centroid(points: &[Point3<f64>], weights: &[f64]) -> Vector3<f64> {
    let total_weight: f64 = weights.iter().sum();
    let weighted_sum: Vector3<f64> = points
        .iter()
        .zip(weights.iter())
        .map(|(p, &w)| p.coords * w)
        .sum();
    weighted_sum / total_weight
}

/// Computes the optimal uniform scale factor.
fn compute_optimal_scale(
    source_centered: &[Vector3<f64>],
    target_centered: &[Vector3<f64>],
    rotation: &UnitQuaternion<f64>,
) -> f64 {
    let mut source_variance = 0.0;
    let mut cross_variance = 0.0;

    for (s, t) in source_centered.iter().zip(target_centered.iter()) {
        source_variance += s.norm_squared();
        let rotated_s = rotation * s;
        cross_variance += rotated_s.dot(t);
    }

    if source_variance > 1e-10 {
        cross_variance / source_variance
    } else {
        1.0
    }
}

/// Computes the weighted optimal uniform scale factor.
fn compute_weighted_optimal_scale(
    source_centered: &[Vector3<f64>],
    target_centered: &[Vector3<f64>],
    weights: &[f64],
    rotation: &UnitQuaternion<f64>,
) -> f64 {
    let mut source_variance = 0.0;
    let mut cross_variance = 0.0;

    for ((s, t), &w) in source_centered
        .iter()
        .zip(target_centered.iter())
        .zip(weights.iter())
    {
        source_variance += w * s.norm_squared();
        let rotated_s = rotation * s;
        cross_variance += w * rotated_s.dot(t);
    }

    if source_variance > 1e-10 {
        cross_variance / source_variance
    } else {
        1.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    fn make_triangle() -> Vec<Point3<f64>> {
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
        ]
    }

    #[test]
    fn test_pure_translation() {
        let source = make_triangle();
        let translation = Vector3::new(5.0, 3.0, 2.0);
        let target: Vec<Point3<f64>> = source
            .iter()
            .map(|p| Point3::from(p.coords + translation))
            .collect();

        let transform = compute_rigid_transform(&source, &target, false).unwrap();

        assert!(transform.rotation.angle() < 1e-6);
        assert_relative_eq!(transform.translation, translation, epsilon = 1e-6);
        assert_relative_eq!(transform.scale, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_pure_rotation() {
        let source = make_triangle();
        let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 4.0);
        let target: Vec<Point3<f64>> = source.iter().map(|p| rotation * p).collect();

        let transform = compute_rigid_transform(&source, &target, false).unwrap();

        // Check rotation angle matches
        assert_relative_eq!(transform.rotation.angle(), PI / 4.0, epsilon = 1e-6);
    }

    #[test]
    fn test_rotation_and_translation() {
        let source = make_triangle();
        let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 2.0);
        let translation = Vector3::new(10.0, 5.0, 0.0);
        let target: Vec<Point3<f64>> = source
            .iter()
            .map(|p| Point3::from((rotation * p).coords + translation))
            .collect();

        let transform = compute_rigid_transform(&source, &target, false).unwrap();

        // Verify transform aligns source to target
        for (s, t) in source.iter().zip(target.iter()) {
            let aligned = transform.transform_point(s);
            assert_relative_eq!(aligned.coords, t.coords, epsilon = 1e-6);
        }
    }

    #[test]
    fn test_with_scale() {
        let source = make_triangle();
        let scale = 2.5;
        let target: Vec<Point3<f64>> = source.iter().map(|p| Point3::from(p.coords * scale)).collect();

        let transform = compute_rigid_transform(&source, &target, true).unwrap();

        assert_relative_eq!(transform.scale, scale, epsilon = 1e-6);

        for (s, t) in source.iter().zip(target.iter()) {
            let aligned = transform.transform_point(s);
            assert_relative_eq!(aligned.coords, t.coords, epsilon = 1e-6);
        }
    }

    #[test]
    fn test_rotation_scale_translation() {
        let source = make_triangle();
        let rotation = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI / 3.0);
        let scale = 1.5;
        let translation = Vector3::new(1.0, 2.0, 3.0);

        let target: Vec<Point3<f64>> = source
            .iter()
            .map(|p| {
                let scaled = p.coords * scale;
                let rotated = rotation * scaled;
                Point3::from(rotated + translation)
            })
            .collect();

        let transform = compute_rigid_transform(&source, &target, true).unwrap();

        for (s, t) in source.iter().zip(target.iter()) {
            let aligned = transform.transform_point(s);
            assert_relative_eq!(aligned.coords, t.coords, epsilon = 1e-5);
        }
    }

    #[test]
    fn test_weighted_transform() {
        let source = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ];
        let translation = Vector3::new(5.0, 5.0, 5.0);
        let target: Vec<Point3<f64>> = source
            .iter()
            .map(|p| Point3::from(p.coords + translation))
            .collect();

        // Equal weights should give same result as unweighted
        let weights = vec![1.0, 1.0, 1.0, 1.0];
        let transform = compute_weighted_rigid_transform(&source, &target, &weights, false).unwrap();

        assert_relative_eq!(transform.translation, translation, epsilon = 1e-6);
    }

    #[test]
    fn test_weighted_different_weights() {
        let source = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0), // Outlier-ish
        ];
        let target = vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(100.0, 0.0, 0.0), // Even more outlier
        ];

        // Heavy weight on first correspondence
        let weights = vec![100.0, 0.01];
        let transform =
            compute_weighted_rigid_transform(&source, &target, &weights, false).unwrap();

        // With heavy weight on first point, translation should be close to (1, 0, 0)
        // (moving source[0] to target[0])
        assert!(transform.translation.x > 0.9 && transform.translation.x < 1.1);
    }

    #[test]
    fn test_empty_source() {
        let source: Vec<Point3<f64>> = vec![];
        let target = vec![Point3::new(1.0, 0.0, 0.0)];
        let result = compute_rigid_transform(&source, &target, false);
        assert!(matches!(result, Err(RegistrationError::EmptySourceMesh)));
    }

    #[test]
    fn test_empty_target() {
        let source = vec![Point3::new(1.0, 0.0, 0.0)];
        let target: Vec<Point3<f64>> = vec![];
        let result = compute_rigid_transform(&source, &target, false);
        assert!(matches!(result, Err(RegistrationError::EmptyTargetMesh)));
    }

    #[test]
    fn test_mismatched_lengths() {
        let source = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)];
        let target = vec![Point3::new(1.0, 0.0, 0.0)];
        let result = compute_rigid_transform(&source, &target, false);
        assert!(matches!(result, Err(RegistrationError::InvalidParameter(_))));
    }

    #[test]
    fn test_reflection_handling() {
        // Create a case that would result in reflection without correction
        let source = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        // Mirror across YZ plane
        let target = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];

        let transform = compute_rigid_transform(&source, &target, false).unwrap();
        let mat = transform.to_matrix4();
        let rot_det = mat.fixed_view::<3, 3>(0, 0).determinant();
        // Should still produce a proper rotation (det > 0)
        assert!(rot_det > 0.0);
    }
}
