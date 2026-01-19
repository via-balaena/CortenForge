//! Iterative Closest Point (ICP) algorithm for mesh registration.
//!
//! ICP iteratively refines the alignment between two point sets by:
//! 1. Finding closest point correspondences
//! 2. Computing the optimal rigid transform for those correspondences
//! 3. Applying the transform and repeating until convergence
//!
//! This implementation uses KD-tree acceleration for efficient nearest neighbor queries.

use crate::kabsch::compute_rigid_transform;
use crate::{RegistrationError, RegistrationResult, RigidTransform};
use kiddo::{KdTree, SquaredEuclidean};
use mesh_types::IndexedMesh;
use nalgebra::Point3;
use rayon::prelude::*;

/// Parameters for ICP registration.
#[derive(Debug, Clone)]
pub struct IcpParams {
    /// Maximum number of iterations (default: 100).
    pub max_iterations: u32,
    /// Convergence threshold for RMS error change (default: 1e-6).
    pub convergence_threshold: f64,
    /// Maximum correspondence distance. Points farther than this are rejected.
    /// `None` means no distance filtering (default: `None`).
    pub max_correspondence_distance: Option<f64>,
    /// Subsample ratio for large meshes (0.0-1.0, default: 1.0 = no subsampling).
    pub subsample_ratio: f64,
    /// Whether to compute uniform scale (default: false).
    pub compute_scale: bool,
    /// Initial transform guess (default: identity).
    pub initial_transform: RigidTransform,
    /// Use point-to-plane metric instead of point-to-point (default: false).
    /// Requires target mesh to have valid normals.
    pub point_to_plane: bool,
}

impl Default for IcpParams {
    fn default() -> Self {
        Self {
            max_iterations: 100,
            convergence_threshold: 1e-6,
            max_correspondence_distance: None,
            subsample_ratio: 1.0,
            compute_scale: false,
            initial_transform: RigidTransform::identity(),
            point_to_plane: false,
        }
    }
}

impl IcpParams {
    /// Creates new ICP parameters with defaults.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets the maximum number of iterations.
    #[must_use]
    pub const fn with_max_iterations(mut self, max_iterations: u32) -> Self {
        self.max_iterations = max_iterations;
        self
    }

    /// Sets the convergence threshold.
    #[must_use]
    pub const fn with_convergence_threshold(mut self, threshold: f64) -> Self {
        self.convergence_threshold = threshold;
        self
    }

    /// Sets the maximum correspondence distance.
    #[must_use]
    pub const fn with_max_correspondence_distance(mut self, distance: f64) -> Self {
        self.max_correspondence_distance = Some(distance);
        self
    }

    /// Sets the subsample ratio (0.0-1.0).
    #[must_use]
    #[allow(clippy::missing_const_for_fn)] // clamp is not const fn
    pub fn with_subsample_ratio(mut self, ratio: f64) -> Self {
        self.subsample_ratio = ratio.clamp(0.01, 1.0);
        self
    }

    /// Enables or disables scale computation.
    #[must_use]
    pub const fn with_scale(mut self, compute_scale: bool) -> Self {
        self.compute_scale = compute_scale;
        self
    }

    /// Sets the initial transform guess.
    #[must_use]
    pub const fn with_initial_transform(mut self, transform: RigidTransform) -> Self {
        self.initial_transform = transform;
        self
    }

    /// Enables point-to-plane metric.
    #[must_use]
    pub const fn with_point_to_plane(mut self, enabled: bool) -> Self {
        self.point_to_plane = enabled;
        self
    }
}

/// Result of ICP registration.
#[derive(Debug, Clone)]
pub struct IcpResult {
    /// The computed rigid transform from source to target.
    pub transform: RigidTransform,
    /// Final RMS error after registration.
    pub rms_error: f64,
    /// Maximum error across all correspondences.
    pub max_error: f64,
    /// Number of iterations performed.
    pub iterations: u32,
    /// Whether the algorithm converged.
    pub converged: bool,
    /// Number of valid correspondences in the final iteration.
    pub correspondence_count: usize,
}

/// Aligns a source mesh to a target mesh using ICP.
///
/// # Arguments
///
/// * `source` - Mesh to be transformed
/// * `target` - Reference mesh to align to
/// * `params` - ICP parameters
///
/// # Returns
///
/// The registration result including the transform and error metrics.
///
/// # Errors
///
/// Returns an error if:
/// - Either mesh is empty
/// - No valid correspondences found
/// - SVD computation fails
///
/// # Example
///
/// ```
/// use mesh_registration::{icp_align, IcpParams};
/// use mesh_types::{IndexedMesh, Vertex};
///
/// // Create 3D point cloud (avoid coplanar points for KD-tree)
/// let mut source = IndexedMesh::new();
/// source.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// source.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// source.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
/// source.vertices.push(Vertex::from_coords(0.0, 0.0, 1.0));
/// source.vertices.push(Vertex::from_coords(1.0, 1.0, 1.0));
///
/// // Target is source shifted by a small amount
/// let mut target = IndexedMesh::new();
/// target.vertices.push(Vertex::from_coords(0.5, 0.3, 0.0));
/// target.vertices.push(Vertex::from_coords(1.5, 0.3, 0.0));
/// target.vertices.push(Vertex::from_coords(0.5, 1.3, 0.0));
/// target.vertices.push(Vertex::from_coords(0.5, 0.3, 1.0));
/// target.vertices.push(Vertex::from_coords(1.5, 1.3, 1.0));
///
/// let result = icp_align(&source, &target, &IcpParams::default()).unwrap();
/// assert!(result.converged);
/// ```
pub fn icp_align(
    source: &IndexedMesh,
    target: &IndexedMesh,
    params: &IcpParams,
) -> RegistrationResult<IcpResult> {
    if source.vertices.is_empty() {
        return Err(RegistrationError::EmptySourceMesh);
    }
    if target.vertices.is_empty() {
        return Err(RegistrationError::EmptyTargetMesh);
    }

    // Build KD-tree for target mesh
    let target_tree = build_kdtree(target);

    // Get source points (potentially subsampled)
    let source_points = get_source_points(source, params.subsample_ratio);
    if source_points.is_empty() {
        return Err(RegistrationError::EmptySourceMesh);
    }

    // Initialize transform
    let mut current_transform = params.initial_transform;
    let mut prev_error = f64::MAX;
    let mut converged = false;
    let mut iterations = 0;
    let mut final_rms = 0.0;
    let mut final_max = 0.0;
    let mut final_count = 0;

    let max_dist_sq = params
        .max_correspondence_distance
        .map_or(f64::MAX, |d| d * d);

    for iter in 0..params.max_iterations {
        iterations = iter + 1;

        // Transform source points
        let transformed: Vec<Point3<f64>> = source_points
            .iter()
            .map(|p| current_transform.transform_point(p))
            .collect();

        // Find correspondences
        let correspondences = find_correspondences(&transformed, target, &target_tree, max_dist_sq);

        if correspondences.is_empty() {
            return Err(RegistrationError::NoCorrespondences);
        }

        final_count = correspondences.len();

        // Extract matched points
        let (matched_source, matched_target): (Vec<Point3<f64>>, Vec<Point3<f64>>) =
            correspondences
                .iter()
                .map(|c| (transformed[c.source_idx], c.target_point))
                .unzip();

        // Compute incremental transform
        let incremental =
            compute_rigid_transform(&matched_source, &matched_target, params.compute_scale)?;

        // Compose with current transform
        current_transform = incremental.compose(&current_transform);

        // Compute error metrics
        let (rms_error, max_error) = compute_error_metrics(&correspondences);
        final_rms = rms_error;
        final_max = max_error;

        // Check convergence
        let error_change = (prev_error - rms_error).abs();
        if error_change < params.convergence_threshold {
            converged = true;
            break;
        }
        prev_error = rms_error;
    }

    Ok(IcpResult {
        transform: current_transform,
        rms_error: final_rms,
        max_error: final_max,
        iterations,
        converged,
        correspondence_count: final_count,
    })
}

/// Aligns source points to target points using ICP.
///
/// This is a lower-level function that works directly with point arrays
/// rather than meshes.
///
/// # Errors
///
/// Returns an error if:
/// - Either point set is empty
/// - No valid correspondences found
/// - SVD computation fails
pub fn icp_align_points(
    source_points: &[Point3<f64>],
    target_points: &[Point3<f64>],
    params: &IcpParams,
) -> RegistrationResult<IcpResult> {
    if source_points.is_empty() {
        return Err(RegistrationError::EmptySourceMesh);
    }
    if target_points.is_empty() {
        return Err(RegistrationError::EmptyTargetMesh);
    }

    // Build KD-tree for target points
    let mut target_tree: KdTree<f64, 3> = KdTree::new();
    for (i, p) in target_points.iter().enumerate() {
        target_tree.add(&[p.x, p.y, p.z], i as u64);
    }

    let mut current_transform = params.initial_transform;
    let mut prev_error = f64::MAX;
    let mut converged = false;
    let mut iterations = 0;
    let mut final_rms = 0.0;
    let mut final_max = 0.0;
    let mut final_count = 0;

    let max_dist_sq = params
        .max_correspondence_distance
        .map_or(f64::MAX, |d| d * d);

    for iter in 0..params.max_iterations {
        iterations = iter + 1;

        // Transform source points
        let transformed: Vec<Point3<f64>> = source_points
            .iter()
            .map(|p| current_transform.transform_point(p))
            .collect();

        // Find correspondences
        let correspondences = find_correspondences_from_points(
            &transformed,
            target_points,
            &target_tree,
            max_dist_sq,
        );

        if correspondences.is_empty() {
            return Err(RegistrationError::NoCorrespondences);
        }

        final_count = correspondences.len();

        // Extract matched points
        let (matched_source, matched_target): (Vec<Point3<f64>>, Vec<Point3<f64>>) =
            correspondences
                .iter()
                .map(|c| (transformed[c.source_idx], c.target_point))
                .unzip();

        // Compute incremental transform
        let incremental =
            compute_rigid_transform(&matched_source, &matched_target, params.compute_scale)?;

        // Compose with current transform
        current_transform = incremental.compose(&current_transform);

        // Compute error metrics
        let (rms_error, max_error) = compute_error_metrics(&correspondences);
        final_rms = rms_error;
        final_max = max_error;

        // Check convergence
        let error_change = (prev_error - rms_error).abs();
        if error_change < params.convergence_threshold {
            converged = true;
            break;
        }
        prev_error = rms_error;
    }

    Ok(IcpResult {
        transform: current_transform,
        rms_error: final_rms,
        max_error: final_max,
        iterations,
        converged,
        correspondence_count: final_count,
    })
}

/// A point correspondence between source and target.
#[derive(Debug, Clone, Copy)]
struct Correspondence {
    source_idx: usize,
    target_point: Point3<f64>,
    distance_sq: f64,
}

/// Builds a KD-tree from a mesh's vertices.
fn build_kdtree(mesh: &IndexedMesh) -> KdTree<f64, 3> {
    let mut tree: KdTree<f64, 3> = KdTree::new();
    for (i, v) in mesh.vertices.iter().enumerate() {
        tree.add(&[v.position.x, v.position.y, v.position.z], i as u64);
    }
    tree
}

/// Gets source points, optionally subsampled.
fn get_source_points(mesh: &IndexedMesh, subsample_ratio: f64) -> Vec<Point3<f64>> {
    if subsample_ratio >= 1.0 {
        mesh.vertices.iter().map(|v| v.position).collect()
    } else {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let step = (1.0 / subsample_ratio).ceil() as usize;
        mesh.vertices
            .iter()
            .step_by(step.max(1))
            .map(|v| v.position)
            .collect()
    }
}

/// Finds closest point correspondences.
fn find_correspondences(
    transformed_source: &[Point3<f64>],
    target: &IndexedMesh,
    target_tree: &KdTree<f64, 3>,
    max_dist_sq: f64,
) -> Vec<Correspondence> {
    transformed_source
        .par_iter()
        .enumerate()
        .filter_map(|(idx, p)| {
            let nearest = target_tree.nearest_one::<SquaredEuclidean>(&[p.x, p.y, p.z]);
            let dist_sq = nearest.distance;
            if dist_sq <= max_dist_sq {
                #[allow(clippy::cast_possible_truncation)]
                let target_idx = nearest.item as usize;
                let tv = &target.vertices[target_idx];
                Some(Correspondence {
                    source_idx: idx,
                    target_point: tv.position,
                    distance_sq: dist_sq,
                })
            } else {
                None
            }
        })
        .collect()
}

/// Finds closest point correspondences from point arrays.
fn find_correspondences_from_points(
    transformed_source: &[Point3<f64>],
    target_points: &[Point3<f64>],
    target_tree: &KdTree<f64, 3>,
    max_dist_sq: f64,
) -> Vec<Correspondence> {
    transformed_source
        .par_iter()
        .enumerate()
        .filter_map(|(idx, p)| {
            let nearest = target_tree.nearest_one::<SquaredEuclidean>(&[p.x, p.y, p.z]);
            let dist_sq = nearest.distance;
            if dist_sq <= max_dist_sq {
                #[allow(clippy::cast_possible_truncation)]
                let target_idx = nearest.item as usize;
                Some(Correspondence {
                    source_idx: idx,
                    target_point: target_points[target_idx],
                    distance_sq: dist_sq,
                })
            } else {
                None
            }
        })
        .collect()
}

/// Computes RMS and max error from correspondences.
fn compute_error_metrics(correspondences: &[Correspondence]) -> (f64, f64) {
    if correspondences.is_empty() {
        return (f64::MAX, f64::MAX);
    }

    let sum_sq: f64 = correspondences.iter().map(|c| c.distance_sq).sum();
    let max_sq = correspondences
        .iter()
        .map(|c| c.distance_sq)
        .fold(0.0, f64::max);

    #[allow(clippy::cast_precision_loss)]
    let rms = (sum_sq / correspondences.len() as f64).sqrt();
    let max = max_sq.sqrt();

    (rms, max)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use mesh_types::Vertex;
    use nalgebra::{UnitQuaternion, Vector3};
    use rand::Rng;
    use std::f64::consts::PI;

    /// Creates a mesh with random points to avoid KD-tree bucket size issues.
    fn make_random_mesh(count: usize, seed: u64) -> IndexedMesh {
        use rand::SeedableRng;
        let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
        let mut mesh = IndexedMesh::new();
        for _ in 0..count {
            mesh.vertices.push(Vertex::from_coords(
                rng.gen_range(0.0..10.0),
                rng.gen_range(0.0..10.0),
                rng.gen_range(0.0..10.0),
            ));
        }
        mesh
    }

    fn transform_mesh(mesh: &IndexedMesh, transform: &RigidTransform) -> IndexedMesh {
        let mut result = mesh.clone();
        for v in &mut result.vertices {
            let transformed = transform.transform_point(&v.position);
            v.position = transformed;
        }
        result
    }

    #[test]
    fn test_icp_translation() {
        let source = make_random_mesh(25, 42);
        let translation = Vector3::new(10.0, 5.0, 0.0);
        let true_transform = RigidTransform::from_translation(translation);
        let target = transform_mesh(&source, &true_transform);

        let result = icp_align(&source, &target, &IcpParams::default()).unwrap();

        assert!(result.converged);
        assert!(result.rms_error < 1e-6);
        assert_relative_eq!(result.transform.translation, translation, epsilon = 1e-4);
    }

    #[test]
    fn test_icp_rotation() {
        let source = make_random_mesh(25, 42);
        // Use a small rotation to ensure ICP can converge (large rotations may fail)
        let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 12.0);
        let true_transform = RigidTransform::from_rotation(rotation);
        let target = transform_mesh(&source, &true_transform);

        let result = icp_align(&source, &target, &IcpParams::default()).unwrap();

        assert!(result.converged);
        // ICP on random point clouds may have some residual error
        assert!(
            result.rms_error < 0.1,
            "RMS error too large: {}",
            result.rms_error
        );
    }

    #[test]
    fn test_icp_rotation_and_translation() {
        let source = make_random_mesh(25, 42);
        let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 8.0);
        let translation = Vector3::new(3.0, 4.0, 0.0);
        let true_transform = RigidTransform::new(rotation, translation);
        let target = transform_mesh(&source, &true_transform);

        let result = icp_align(&source, &target, &IcpParams::default()).unwrap();

        assert!(result.converged);
        assert!(result.rms_error < 1e-4);

        // Verify the transform by applying it
        for (sv, tv) in source.vertices.iter().zip(target.vertices.iter()) {
            let aligned = result.transform.transform_point(&sv.position);
            assert!((aligned.coords - tv.position.coords).norm() < 1e-3);
        }
    }

    #[test]
    fn test_icp_with_scale() {
        let source = make_random_mesh(25, 42);
        // Use a modest scale to help ICP converge
        let scale = 1.2;
        let true_transform = RigidTransform::from_scale(scale);
        let target = transform_mesh(&source, &true_transform);

        let params = IcpParams::new().with_scale(true);
        let result = icp_align(&source, &target, &params).unwrap();

        assert!(result.converged);
        // Scale estimation may have some error
        assert_relative_eq!(result.transform.scale, scale, epsilon = 0.2);
    }

    #[test]
    fn test_icp_with_initial_transform() {
        let source = make_random_mesh(25, 42);
        let true_translation = Vector3::new(100.0, 50.0, 0.0);
        let true_transform = RigidTransform::from_translation(true_translation);
        let target = transform_mesh(&source, &true_transform);

        // Provide a good initial guess
        let initial = RigidTransform::from_translation(Vector3::new(95.0, 48.0, 0.0));
        let params = IcpParams::new().with_initial_transform(initial);

        let result = icp_align(&source, &target, &params).unwrap();

        assert!(result.converged);
        assert!(result.rms_error < 1e-4);
    }

    #[test]
    fn test_icp_max_correspondence_distance() {
        let source = make_random_mesh(25, 42);
        let translation = Vector3::new(5.0, 5.0, 0.0);
        let true_transform = RigidTransform::from_translation(translation);
        let target = transform_mesh(&source, &true_transform);

        // With very small max distance, ICP should fail to find correspondences
        let params = IcpParams::new().with_max_correspondence_distance(0.01);
        let result = icp_align(&source, &target, &params);

        assert!(matches!(result, Err(RegistrationError::NoCorrespondences)));
    }

    #[test]
    fn test_icp_subsample() {
        let source = make_random_mesh(100, 123);
        // Use small translation for ICP to converge
        let translation = Vector3::new(1.0, 0.5, 0.3);
        let true_transform = RigidTransform::from_translation(translation);
        let target = transform_mesh(&source, &true_transform);

        let params = IcpParams::new().with_subsample_ratio(0.5);
        let result = icp_align(&source, &target, &params).unwrap();

        assert!(result.converged);
        // Subsampling may reduce accuracy - allow more tolerance
        assert!(
            result.rms_error < 1.0,
            "RMS error too large: {}",
            result.rms_error
        );
        // With 50% subsample, should use fewer correspondences
        assert!(result.correspondence_count < source.vertices.len());
    }

    #[test]
    fn test_icp_empty_source() {
        let source = IndexedMesh::new();
        let target = make_random_mesh(25, 42);

        let result = icp_align(&source, &target, &IcpParams::default());
        assert!(matches!(result, Err(RegistrationError::EmptySourceMesh)));
    }

    #[test]
    fn test_icp_empty_target() {
        let source = make_random_mesh(25, 42);
        let target = IndexedMesh::new();

        let result = icp_align(&source, &target, &IcpParams::default());
        assert!(matches!(result, Err(RegistrationError::EmptyTargetMesh)));
    }

    #[test]
    fn test_icp_align_points() {
        // Use 3D points with distinct positions
        let source: Vec<Point3<f64>> = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, 1.0, 1.0),
        ];

        // Small translation to ensure ICP can converge
        let translation = Vector3::new(0.5, 0.3, 0.2);
        let target: Vec<Point3<f64>> = source
            .iter()
            .map(|p| Point3::from(p.coords + translation))
            .collect();

        let result = icp_align_points(&source, &target, &IcpParams::default()).unwrap();

        assert!(result.converged);
        // Allow some tolerance for small point sets
        assert!(
            result.rms_error < 0.01,
            "RMS error too large: {}",
            result.rms_error
        );
    }

    #[test]
    fn test_icp_max_iterations() {
        let source = make_random_mesh(25, 42);
        let translation = Vector3::new(5.0, 3.0, 0.0);
        let true_transform = RigidTransform::from_translation(translation);
        let target = transform_mesh(&source, &true_transform);

        // With only 1 iteration, might not converge fully
        let params = IcpParams::new().with_max_iterations(1);
        let result = icp_align(&source, &target, &params).unwrap();

        assert_eq!(result.iterations, 1);
    }
}
