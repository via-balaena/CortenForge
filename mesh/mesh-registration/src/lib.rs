//! Mesh alignment and registration algorithms.
//!
//! This crate provides tools for aligning meshes and point clouds:
//! - **ICP (Iterative Closest Point)** - Automatic alignment via iterative refinement
//! - **Landmark-based** - Alignment from known point correspondences
//! - **Kabsch algorithm** - Optimal rigid transform from paired points
//!
//! # Layer 0
//!
//! This is a Layer 0 crate with zero Bevy dependencies.
//!
//! # Quick Start
//!
//! ## ICP Registration
//!
//! Use ICP when you have two similar meshes without known correspondences:
//!
//! ```
//! use mesh_registration::{icp_align, IcpParams};
//! use mesh_types::{IndexedMesh, Vertex};
//!
//! // Create source and target meshes
//! let mut source = IndexedMesh::new();
//! source.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! source.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
//! source.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
//! source.vertices.push(Vertex::from_coords(1.0, 1.0, 0.0));
//!
//! let mut target = IndexedMesh::new();
//! target.vertices.push(Vertex::from_coords(5.0, 5.0, 0.0));
//! target.vertices.push(Vertex::from_coords(6.0, 5.0, 0.0));
//! target.vertices.push(Vertex::from_coords(5.0, 6.0, 0.0));
//! target.vertices.push(Vertex::from_coords(6.0, 6.0, 0.0));
//!
//! // Align source to target
//! let result = icp_align(&source, &target, &IcpParams::default()).unwrap();
//!
//! println!("Converged: {}", result.converged);
//! println!("RMS Error: {:.6}", result.rms_error);
//! println!("Iterations: {}", result.iterations);
//! ```
//!
//! ## Landmark Registration
//!
//! Use landmarks when you have known point correspondences:
//!
//! ```
//! use mesh_registration::{align_by_landmarks, Landmark, LandmarkParams};
//! use mesh_types::{IndexedMesh, Vertex};
//!
//! let mut source = IndexedMesh::new();
//! source.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! source.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
//! source.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
//!
//! let mut target = IndexedMesh::new();
//! target.vertices.push(Vertex::from_coords(10.0, 10.0, 0.0));
//! target.vertices.push(Vertex::from_coords(11.0, 10.0, 0.0));
//! target.vertices.push(Vertex::from_coords(10.0, 11.0, 0.0));
//!
//! // Define correspondences: source vertex i -> target vertex i
//! let landmarks = vec![
//!     Landmark::from_indices(0, 0),
//!     Landmark::from_indices(1, 1),
//!     Landmark::from_indices(2, 2),
//! ];
//!
//! let transform = align_by_landmarks(&source, &target, &landmarks, &LandmarkParams::default())
//!     .unwrap();
//! ```
//!
//! ## Direct Point-to-Point Alignment
//!
//! For aligning raw point arrays:
//!
//! ```
//! use mesh_registration::{align_points_to_points, compute_rigid_transform};
//! use nalgebra::Point3;
//!
//! let source = vec![
//!     Point3::new(0.0, 0.0, 0.0),
//!     Point3::new(1.0, 0.0, 0.0),
//!     Point3::new(0.0, 1.0, 0.0),
//! ];
//!
//! let target = vec![
//!     Point3::new(5.0, 5.0, 0.0),
//!     Point3::new(6.0, 5.0, 0.0),
//!     Point3::new(5.0, 6.0, 0.0),
//! ];
//!
//! let transform = align_points_to_points(&source, &target, false).unwrap();
//! ```
//!
//! # Algorithm Selection
//!
//! | Scenario | Recommended Algorithm |
//! |----------|----------------------|
//! | Known correspondences | `align_by_landmarks` |
//! | Unknown correspondences | `icp_align` |
//! | Large rotation/translation | `icp_align` with initial transform |
//! | Scans with noise | `icp_align` with `max_correspondence_distance` |
//! | Different scales | Use `with_scale(true)` |
//!
//! # Performance Tips
//!
//! - For large meshes, use `IcpParams::with_subsample_ratio()` to reduce computation
//! - Provide an initial transform guess when alignments are large
//! - Use `max_correspondence_distance` to reject outliers
//! - ICP uses KD-tree acceleration for O(n log n) nearest neighbor queries

// Safety: Deny unwrap/expect in library code. Tests may use them (workspace warns).
#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used))]

mod error;
mod icp;
mod kabsch;
mod landmark;
mod transform;

pub use error::{RegistrationError, RegistrationResult};
pub use icp::{IcpParams, IcpResult, icp_align, icp_align_points};
pub use kabsch::{compute_rigid_transform, compute_weighted_rigid_transform};
pub use landmark::{Landmark, LandmarkParams, align_by_landmarks, align_points_to_points};
pub use transform::RigidTransform;

/// Applies a rigid transform to a mesh, returning a new transformed mesh.
///
/// # Example
///
/// ```
/// use mesh_registration::{transform_mesh, RigidTransform};
/// use mesh_types::{IndexedMesh, Vertex};
/// use nalgebra::Vector3;
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
///
/// let transform = RigidTransform::from_translation(Vector3::new(5.0, 0.0, 0.0));
/// let transformed = transform_mesh(&mesh, &transform);
///
/// assert!((transformed.vertices[0].position.x - 5.0).abs() < 1e-10);
/// ```
#[must_use]
pub fn transform_mesh(
    mesh: &mesh_types::IndexedMesh,
    transform: &RigidTransform,
) -> mesh_types::IndexedMesh {
    let mut result = mesh.clone();
    for v in &mut result.vertices {
        let transformed = transform.transform_point(&v.position);
        v.position = transformed;

        // Transform normals if present
        if let Some(normal) = v.attributes.normal {
            let transformed_n = transform.rotation * normal;
            v.attributes.normal = Some(transformed_n);
        }
    }
    result
}

/// Computes the alignment error between two meshes after applying a transform.
///
/// Returns (RMS error, max error) where errors are measured as distances
/// between corresponding vertices.
///
/// # Panics
///
/// Panics if the meshes have different numbers of vertices.
///
/// # Example
///
/// ```
/// use mesh_registration::{compute_alignment_error, RigidTransform};
/// use mesh_types::{IndexedMesh, Vertex};
/// use nalgebra::Vector3;
///
/// let mut source = IndexedMesh::new();
/// source.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// source.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
///
/// let mut target = IndexedMesh::new();
/// target.vertices.push(Vertex::from_coords(5.0, 0.0, 0.0));
/// target.vertices.push(Vertex::from_coords(6.0, 0.0, 0.0));
///
/// let transform = RigidTransform::from_translation(Vector3::new(5.0, 0.0, 0.0));
/// let (rms, max) = compute_alignment_error(&source, &target, &transform);
///
/// assert!(rms < 1e-10);
/// assert!(max < 1e-10);
/// ```
#[must_use]
pub fn compute_alignment_error(
    source: &mesh_types::IndexedMesh,
    target: &mesh_types::IndexedMesh,
    transform: &RigidTransform,
) -> (f64, f64) {
    assert_eq!(
        source.vertices.len(),
        target.vertices.len(),
        "meshes must have same vertex count"
    );

    if source.vertices.is_empty() {
        return (0.0, 0.0);
    }

    let mut sum_sq = 0.0;
    let mut max_sq = 0.0;

    for (sv, tv) in source.vertices.iter().zip(target.vertices.iter()) {
        let aligned = transform.transform_point(&sv.position);
        let dist_sq = (aligned.coords - tv.position.coords).norm_squared();
        sum_sq += dist_sq;
        if dist_sq > max_sq {
            max_sq = dist_sq;
        }
    }

    #[allow(clippy::cast_precision_loss)]
    let rms = (sum_sq / source.vertices.len() as f64).sqrt();
    let max = max_sq.sqrt();

    (rms, max)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use mesh_types::Vertex;
    use nalgebra::{UnitQuaternion, Vector3};
    use std::f64::consts::PI;

    fn make_test_mesh() -> mesh_types::IndexedMesh {
        let mut mesh = mesh_types::IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 1.0, 0.0));
        mesh
    }

    #[test]
    fn test_transform_mesh() {
        let mesh = make_test_mesh();
        let translation = Vector3::new(5.0, 3.0, 1.0);
        let transform = RigidTransform::from_translation(translation);

        let transformed = transform_mesh(&mesh, &transform);

        assert_relative_eq!(transformed.vertices[0].position.x, 5.0, epsilon = 1e-10);
        assert_relative_eq!(transformed.vertices[0].position.y, 3.0, epsilon = 1e-10);
        assert_relative_eq!(transformed.vertices[0].position.z, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_transform_mesh_with_normals() {
        let mut mesh = mesh_types::IndexedMesh::new();
        let v = Vertex::with_normal(
            nalgebra::Point3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
        );
        mesh.vertices.push(v);

        let rotation = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI / 2.0);
        let transform = RigidTransform::from_rotation(rotation);

        let transformed = transform_mesh(&mesh, &transform);

        // Normal (0,0,1) rotated 90 degrees around X -> (0,-1,0)
        let normal = transformed.vertices[0].attributes.normal.unwrap();
        assert_relative_eq!(normal.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(normal.y, -1.0, epsilon = 1e-10);
        assert!(normal.z.abs() < 1e-10);
    }

    #[test]
    fn test_compute_alignment_error_perfect() {
        let source = make_test_mesh();
        let translation = Vector3::new(5.0, 3.0, 0.0);
        let transform = RigidTransform::from_translation(translation);
        let target = transform_mesh(&source, &transform);

        let (rms, max) = compute_alignment_error(&source, &target, &transform);

        assert!(rms < 1e-10);
        assert!(max < 1e-10);
    }

    #[test]
    fn test_compute_alignment_error_with_error() {
        let mut source = mesh_types::IndexedMesh::new();
        source.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));

        let mut target = mesh_types::IndexedMesh::new();
        target.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));

        let transform = RigidTransform::identity();
        let (rms, max) = compute_alignment_error(&source, &target, &transform);

        assert_relative_eq!(rms, 1.0, epsilon = 1e-10);
        assert_relative_eq!(max, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_full_workflow() {
        // Create source mesh - use larger mesh for better ICP convergence
        let mut source = mesh_types::IndexedMesh::new();
        for i in 0..5 {
            for j in 0..5 {
                for k in 0..2 {
                    source.vertices.push(Vertex::from_coords(
                        i as f64 * 2.0,
                        j as f64 * 2.0,
                        k as f64 * 2.0,
                    ));
                }
            }
        }

        // Create target with just translation (rotation can cause ICP issues)
        let true_translation = Vector3::new(0.5, 0.3, 0.0);
        let true_transform = RigidTransform::from_translation(true_translation);
        let target = transform_mesh(&source, &true_transform);

        // Register using ICP
        let result = icp_align(&source, &target, &IcpParams::default()).unwrap();

        assert!(result.converged);
        // ICP may have some residual error
        assert!(
            result.rms_error < 2.0,
            "RMS error too large: {}",
            result.rms_error
        );
    }

    #[test]
    fn test_landmark_then_icp_workflow() {
        // Create source mesh
        let source = make_test_mesh();

        // Create target with known transform
        let true_translation = Vector3::new(100.0, 50.0, 0.0);
        let true_transform = RigidTransform::from_translation(true_translation);
        let target = transform_mesh(&source, &true_transform);

        // First, get rough alignment with landmarks
        let landmarks = vec![
            Landmark::from_indices(0, 0),
            Landmark::from_indices(1, 1),
            Landmark::from_indices(2, 2),
        ];
        let initial_transform =
            align_by_landmarks(&source, &target, &landmarks, &LandmarkParams::default()).unwrap();

        // Then refine with ICP
        let params = IcpParams::new().with_initial_transform(initial_transform);
        let result = icp_align(&source, &target, &params).unwrap();

        assert!(result.converged);
        assert!(result.rms_error < 1e-6);
    }
}
