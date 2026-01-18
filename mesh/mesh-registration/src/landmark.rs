//! Landmark-based mesh registration.
//!
//! Landmarks are known point correspondences between source and target meshes.
//! This module provides functions to compute optimal alignment from landmarks.

use crate::kabsch::{compute_rigid_transform, compute_weighted_rigid_transform};
use crate::{RegistrationError, RegistrationResult, RigidTransform};
use mesh_types::IndexedMesh;
use nalgebra::Point3;

/// A landmark correspondence between source and target meshes.
///
/// Each landmark specifies a point on the source mesh that should align
/// with a corresponding point on the target mesh.
#[derive(Debug, Clone, Copy)]
pub struct Landmark {
    /// Index of the vertex in the source mesh, or `None` for a free point.
    pub source_index: Option<usize>,
    /// Index of the vertex in the target mesh, or `None` for a free point.
    pub target_index: Option<usize>,
    /// Explicit source point (used if `source_index` is `None`).
    pub source_point: Option<Point3<f64>>,
    /// Explicit target point (used if `target_index` is `None`).
    pub target_point: Option<Point3<f64>>,
    /// Weight for this landmark (default 1.0).
    pub weight: f64,
}

impl Landmark {
    /// Creates a landmark from vertex indices.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_registration::Landmark;
    ///
    /// // Vertex 0 in source should align with vertex 5 in target
    /// let landmark = Landmark::from_indices(0, 5);
    /// ```
    #[must_use]
    pub const fn from_indices(source_index: usize, target_index: usize) -> Self {
        Self {
            source_index: Some(source_index),
            target_index: Some(target_index),
            source_point: None,
            target_point: None,
            weight: 1.0,
        }
    }

    /// Creates a landmark from explicit 3D points.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_registration::Landmark;
    /// use nalgebra::Point3;
    ///
    /// let source_pt = Point3::new(0.0, 0.0, 0.0);
    /// let target_pt = Point3::new(1.0, 2.0, 3.0);
    /// let landmark = Landmark::from_points(source_pt, target_pt);
    /// ```
    #[must_use]
    pub const fn from_points(source_point: Point3<f64>, target_point: Point3<f64>) -> Self {
        Self {
            source_index: None,
            target_index: None,
            source_point: Some(source_point),
            target_point: Some(target_point),
            weight: 1.0,
        }
    }

    /// Creates a landmark with a specific weight.
    ///
    /// Higher weights give more influence to this landmark in the alignment.
    #[must_use]
    pub const fn with_weight(mut self, weight: f64) -> Self {
        self.weight = weight;
        self
    }

    /// Resolves the source point, either from index or explicit point.
    fn resolve_source(&self, mesh: Option<&IndexedMesh>) -> Option<Point3<f64>> {
        self.source_point.or_else(|| {
            self.source_index
                .zip(mesh)
                .and_then(|(idx, m)| m.vertices.get(idx).map(|v| v.position))
        })
    }

    /// Resolves the target point, either from index or explicit point.
    fn resolve_target(&self, mesh: Option<&IndexedMesh>) -> Option<Point3<f64>> {
        self.target_point.or_else(|| {
            self.target_index
                .zip(mesh)
                .and_then(|(idx, m)| m.vertices.get(idx).map(|v| v.position))
        })
    }
}

/// Parameters for landmark-based registration.
#[derive(Debug, Clone, Default)]
pub struct LandmarkParams {
    /// Whether to compute uniform scale (default: false).
    pub compute_scale: bool,
}

impl LandmarkParams {
    /// Creates new landmark parameters.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Enables or disables scale computation.
    #[must_use]
    pub const fn with_scale(mut self, compute_scale: bool) -> Self {
        self.compute_scale = compute_scale;
        self
    }
}

/// Computes the optimal rigid transform from landmarks.
///
/// Requires at least 3 non-collinear landmarks for a unique solution.
/// With fewer landmarks or collinear points, the result may be ambiguous.
///
/// # Arguments
///
/// * `source` - Source mesh (for resolving vertex indices)
/// * `target` - Target mesh (for resolving vertex indices)
/// * `landmarks` - Point correspondences
/// * `params` - Registration parameters
///
/// # Returns
///
/// The optimal rigid transform that aligns the source landmarks to target landmarks.
///
/// # Errors
///
/// Returns an error if:
/// - Fewer than 3 landmarks provided
/// - Landmark indices are out of bounds
/// - All landmarks have zero weight
///
/// # Example
///
/// ```
/// use mesh_registration::{align_by_landmarks, Landmark, LandmarkParams};
/// use mesh_types::{IndexedMesh, Vertex};
///
/// let mut source = IndexedMesh::new();
/// source.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// source.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// source.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
///
/// let mut target = IndexedMesh::new();
/// target.vertices.push(Vertex::from_coords(5.0, 5.0, 0.0));
/// target.vertices.push(Vertex::from_coords(6.0, 5.0, 0.0));
/// target.vertices.push(Vertex::from_coords(5.0, 6.0, 0.0));
///
/// let landmarks = vec![
///     Landmark::from_indices(0, 0),
///     Landmark::from_indices(1, 1),
///     Landmark::from_indices(2, 2),
/// ];
///
/// let transform = align_by_landmarks(&source, &target, &landmarks, &LandmarkParams::default()).unwrap();
/// ```
pub fn align_by_landmarks(
    source: &IndexedMesh,
    target: &IndexedMesh,
    landmarks: &[Landmark],
    params: &LandmarkParams,
) -> RegistrationResult<RigidTransform> {
    const MIN_LANDMARKS: usize = 3;

    if landmarks.len() < MIN_LANDMARKS {
        return Err(RegistrationError::InsufficientLandmarks {
            required: MIN_LANDMARKS,
            provided: landmarks.len(),
        });
    }

    // Resolve all landmark points
    let mut source_points = Vec::with_capacity(landmarks.len());
    let mut target_points = Vec::with_capacity(landmarks.len());
    let mut weights = Vec::with_capacity(landmarks.len());

    for (i, landmark) in landmarks.iter().enumerate() {
        let source_pt = landmark.resolve_source(Some(source)).ok_or_else(|| {
            landmark.source_index.map_or_else(
                || RegistrationError::InvalidParameter(format!("landmark {i} missing source point")),
                |idx| RegistrationError::LandmarkOutOfBounds {
                    index: idx,
                    vertex_count: source.vertices.len(),
                },
            )
        })?;

        let target_pt = landmark.resolve_target(Some(target)).ok_or_else(|| {
            landmark.target_index.map_or_else(
                || RegistrationError::InvalidParameter(format!("landmark {i} missing target point")),
                |idx| RegistrationError::LandmarkOutOfBounds {
                    index: idx,
                    vertex_count: target.vertices.len(),
                },
            )
        })?;

        source_points.push(source_pt);
        target_points.push(target_pt);
        weights.push(landmark.weight);
    }

    // Check if any weights are non-uniform
    let has_weights = weights.iter().any(|&w| (w - 1.0).abs() > 1e-10);

    if has_weights {
        compute_weighted_rigid_transform(&source_points, &target_points, &weights, params.compute_scale)
    } else {
        compute_rigid_transform(&source_points, &target_points, params.compute_scale)
    }
}

/// Computes the optimal rigid transform from explicit point correspondences.
///
/// This is a convenience function when you have direct point coordinates
/// rather than mesh vertex indices.
///
/// # Arguments
///
/// * `source_points` - Points on the source
/// * `target_points` - Corresponding points on the target
/// * `compute_scale` - Whether to compute uniform scale
///
/// # Errors
///
/// Returns an error if:
/// - Fewer than 3 points provided
/// - Point sets have different lengths
/// - SVD computation fails
///
/// # Example
///
/// ```
/// use mesh_registration::align_points_to_points;
/// use nalgebra::Point3;
///
/// let source = vec![
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 0.0, 0.0),
///     Point3::new(0.0, 1.0, 0.0),
/// ];
///
/// let target = vec![
///     Point3::new(10.0, 10.0, 10.0),
///     Point3::new(11.0, 10.0, 10.0),
///     Point3::new(10.0, 11.0, 10.0),
/// ];
///
/// let transform = align_points_to_points(&source, &target, false).unwrap();
/// ```
pub fn align_points_to_points(
    source_points: &[Point3<f64>],
    target_points: &[Point3<f64>],
    compute_scale: bool,
) -> RegistrationResult<RigidTransform> {
    const MIN_POINTS: usize = 3;

    if source_points.len() < MIN_POINTS {
        return Err(RegistrationError::InsufficientLandmarks {
            required: MIN_POINTS,
            provided: source_points.len(),
        });
    }

    compute_rigid_transform(source_points, target_points, compute_scale)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use mesh_types::Vertex;
    use nalgebra::Vector3;

    fn make_test_mesh(points: &[(f64, f64, f64)]) -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        for (x, y, z) in points {
            mesh.vertices.push(Vertex::from_coords(*x, *y, *z));
        }
        mesh
    }

    #[test]
    fn test_landmark_from_indices() {
        let source = make_test_mesh(&[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]);
        let target = make_test_mesh(&[(5.0, 5.0, 0.0), (6.0, 5.0, 0.0), (5.0, 6.0, 0.0)]);

        let landmarks = vec![
            Landmark::from_indices(0, 0),
            Landmark::from_indices(1, 1),
            Landmark::from_indices(2, 2),
        ];

        let transform = align_by_landmarks(&source, &target, &landmarks, &LandmarkParams::default())
            .unwrap();

        // Verify alignment
        for (s, t) in source.vertices.iter().zip(target.vertices.iter()) {
            let aligned = transform.transform_point(&s.position);
            assert_relative_eq!(aligned.coords, t.position.coords, epsilon = 1e-6);
        }
    }

    #[test]
    fn test_landmark_from_points() {
        let source = make_test_mesh(&[(0.0, 0.0, 0.0)]);
        let target = make_test_mesh(&[(0.0, 0.0, 0.0)]);

        let landmarks = vec![
            Landmark::from_points(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)),
            Landmark::from_points(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)),
            Landmark::from_points(Point3::new(0.0, 1.0, 0.0), Point3::new(1.0, 1.0, 0.0)),
        ];

        let transform = align_by_landmarks(&source, &target, &landmarks, &LandmarkParams::default())
            .unwrap();

        // Should be a pure translation of (1, 0, 0)
        assert_relative_eq!(transform.translation, Vector3::new(1.0, 0.0, 0.0), epsilon = 1e-6);
    }

    #[test]
    fn test_landmark_with_weights() {
        let source = make_test_mesh(&[
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (100.0, 0.0, 0.0), // Outlier
        ]);
        let target = make_test_mesh(&[
            (5.0, 5.0, 0.0),
            (6.0, 5.0, 0.0),
            (5.0, 6.0, 0.0),
            (500.0, 0.0, 0.0), // Outlier target
        ]);

        // Give very low weight to outlier
        let landmarks = vec![
            Landmark::from_indices(0, 0).with_weight(1.0),
            Landmark::from_indices(1, 1).with_weight(1.0),
            Landmark::from_indices(2, 2).with_weight(1.0),
            Landmark::from_indices(3, 3).with_weight(0.001),
        ];

        let transform = align_by_landmarks(&source, &target, &landmarks, &LandmarkParams::default())
            .unwrap();

        // First three points should align better than the outlier
        // Note: with weighted fitting, the first 3 points won't be perfect
        // but should be much closer than the outlier
        for i in 0..3 {
            let source_pt = source.vertices[i].position;
            let target_pt = target.vertices[i].position;
            let aligned = transform.transform_point(&source_pt);
            let dist = (aligned.coords - target_pt.coords).norm();
            // Should be within 0.5 mm (not perfect due to outlier influence)
            assert!(dist < 0.5, "Point {i} distance {dist} too large");
        }
    }

    #[test]
    fn test_landmark_with_scale() {
        let source = make_test_mesh(&[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]);
        // Target is 2x scaled
        let target = make_test_mesh(&[(0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (0.0, 2.0, 0.0)]);

        let landmarks = vec![
            Landmark::from_indices(0, 0),
            Landmark::from_indices(1, 1),
            Landmark::from_indices(2, 2),
        ];

        let params = LandmarkParams::new().with_scale(true);
        let transform = align_by_landmarks(&source, &target, &landmarks, &params).unwrap();

        assert_relative_eq!(transform.scale, 2.0, epsilon = 1e-6);
    }

    #[test]
    fn test_insufficient_landmarks() {
        let source = make_test_mesh(&[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]);
        let target = make_test_mesh(&[(5.0, 5.0, 0.0), (6.0, 5.0, 0.0)]);

        let landmarks = vec![
            Landmark::from_indices(0, 0),
            Landmark::from_indices(1, 1),
        ];

        let result = align_by_landmarks(&source, &target, &landmarks, &LandmarkParams::default());
        assert!(matches!(
            result,
            Err(RegistrationError::InsufficientLandmarks { .. })
        ));
    }

    #[test]
    fn test_landmark_out_of_bounds() {
        let source = make_test_mesh(&[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]);
        let target = make_test_mesh(&[(5.0, 5.0, 0.0), (6.0, 5.0, 0.0), (5.0, 6.0, 0.0)]);

        let landmarks = vec![
            Landmark::from_indices(0, 0),
            Landmark::from_indices(1, 1),
            Landmark::from_indices(999, 2), // Out of bounds
        ];

        let result = align_by_landmarks(&source, &target, &landmarks, &LandmarkParams::default());
        assert!(matches!(
            result,
            Err(RegistrationError::LandmarkOutOfBounds { .. })
        ));
    }

    #[test]
    fn test_align_points_to_points() {
        let source = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];

        let translation = Vector3::new(10.0, 20.0, 30.0);
        let target: Vec<Point3<f64>> = source
            .iter()
            .map(|p| Point3::from(p.coords + translation))
            .collect();

        let transform = align_points_to_points(&source, &target, false).unwrap();

        assert_relative_eq!(transform.translation, translation, epsilon = 1e-6);
    }

    #[test]
    fn test_align_points_insufficient() {
        let source = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)];
        let target = vec![Point3::new(5.0, 0.0, 0.0), Point3::new(6.0, 0.0, 0.0)];

        let result = align_points_to_points(&source, &target, false);
        assert!(matches!(
            result,
            Err(RegistrationError::InsufficientLandmarks { .. })
        ));
    }
}
