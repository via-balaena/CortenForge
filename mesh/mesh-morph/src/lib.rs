//! Mesh deformation via RBF and FFD morphing.
//!
//! This crate provides tools for deforming meshes using two algorithms:
//!
//! - **RBF (Radial Basis Functions)**: Smooth, global deformation based on constraint points
//! - **FFD (Free-Form Deformation)**: Lattice-based deformation with Bernstein polynomials
//!
//! # Layer 0
//!
//! This is a Layer 0 crate with zero Bevy dependencies.
//!
//! # Quick Start
//!
//! ## RBF Morphing
//!
//! RBF morphing creates smooth deformations by interpolating between constraint points:
//!
//! ```
//! use mesh_morph::{morph_mesh, MorphParams, Constraint};
//! use mesh_types::{IndexedMesh, Vertex};
//! use nalgebra::Point3;
//!
//! // Create a simple mesh
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! // Define constraints
//! let params = MorphParams::rbf()
//!     .with_constraint(Constraint::point(
//!         Point3::new(0.0, 0.0, 0.0),
//!         Point3::new(0.0, 0.0, 0.5), // Move up
//!     ))
//!     .with_constraint(Constraint::point(
//!         Point3::new(1.0, 0.0, 0.0),
//!         Point3::new(1.0, 0.0, 0.0), // Keep in place
//!     ))
//!     .with_constraint(Constraint::point(
//!         Point3::new(0.0, 1.0, 0.0),
//!         Point3::new(0.0, 1.0, 0.0), // Keep in place
//!     ));
//!
//! let result = morph_mesh(&mesh, &params).unwrap();
//! println!("Vertices modified: {}", result.vertices_modified);
//! ```
//!
//! ## FFD Morphing
//!
//! FFD morphing deforms the mesh using a control lattice:
//!
//! ```
//! use mesh_morph::{morph_mesh, MorphParams, Constraint};
//! use mesh_types::{IndexedMesh, Vertex};
//! use nalgebra::{Point3, Vector3};
//!
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(0.5, 0.5, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! let params = MorphParams::ffd()
//!     .with_constraint(Constraint::displacement(
//!         Point3::new(0.5, 0.25, 0.0),
//!         Vector3::new(0.0, 0.0, 0.2),
//!     ));
//!
//! let result = morph_mesh(&mesh, &params).unwrap();
//! ```
//!
//! # RBF Kernels
//!
//! Different kernels produce different deformation characteristics:
//!
//! | Kernel | Characteristic | Use Case |
//! |--------|----------------|----------|
//! | Thin-Plate Spline | Smooth, global | Natural deformations |
//! | Gaussian | Local, controllable | Localized edits |
//! | Multiquadric | Balanced | General purpose |
//! | Inverse Multiquadric | Strong local | Fine details |
//!
//! # Selective Deformation
//!
//! Use vertex masks to deform only specific parts of the mesh:
//!
//! ```
//! use mesh_morph::{morph_mesh, MorphParams, Constraint};
//! use mesh_types::{IndexedMesh, Vertex};
//! use nalgebra::Point3;
//! use std::collections::HashSet;
//!
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! // Only deform vertex 0
//! let mask: HashSet<usize> = [0].into_iter().collect();
//!
//! let params = MorphParams::rbf()
//!     .with_vertex_mask(mask)
//!     .with_constraint(Constraint::point(
//!         Point3::new(0.0, 0.0, 0.0),
//!         Point3::new(0.0, 0.0, 0.5),
//!     ))
//!     .with_constraint(Constraint::point(
//!         Point3::new(1.0, 0.0, 0.0),
//!         Point3::new(1.0, 0.0, 0.0),
//!     ))
//!     .with_constraint(Constraint::point(
//!         Point3::new(0.0, 1.0, 0.0),
//!         Point3::new(0.0, 1.0, 0.0),
//!     ));
//!
//! let result = morph_mesh(&mesh, &params).unwrap();
//! ```

mod constraint;
mod error;
mod ffd;
mod morph;
mod params;
mod rbf;
mod result;

pub use constraint::Constraint;
pub use error::{MorphError, MorphResult};
pub use ffd::{FfdConfig, bernstein_basis, binomial};
pub use morph::morph_mesh;
pub use params::{MorphAlgorithm, MorphParams};
pub use rbf::RbfKernel;
pub use result::MorphOutput;

#[cfg(test)]
mod integration_tests {
    use super::*;
    use approx::assert_relative_eq;
    use mesh_types::{IndexedMesh, Vertex};
    use nalgebra::{Point3, Vector3};

    fn make_cube() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        // 8 vertices of a unit cube
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 1.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 1.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 1.0, 1.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 1.0));

        // 12 triangles (2 per face)
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 2, 3]);
        mesh.faces.push([4, 6, 5]);
        mesh.faces.push([4, 7, 6]);
        mesh.faces.push([0, 4, 5]);
        mesh.faces.push([0, 5, 1]);
        mesh.faces.push([2, 6, 7]);
        mesh.faces.push([2, 7, 3]);
        mesh.faces.push([0, 7, 4]);
        mesh.faces.push([0, 3, 7]);
        mesh.faces.push([1, 5, 6]);
        mesh.faces.push([1, 6, 2]);

        mesh
    }

    #[test]
    fn test_cube_translation() {
        let mesh = make_cube();
        let offset = Vector3::new(10.0, 20.0, 30.0);

        // Translate all corners
        let mut params = MorphParams::rbf();
        for v in &mesh.vertices {
            params = params.with_constraint(Constraint::point(v.position, v.position + offset));
        }

        let result = morph_mesh(&mesh, &params).unwrap();

        // All vertices should be translated
        for (orig, deformed) in mesh.vertices.iter().zip(result.mesh.vertices.iter()) {
            let expected = orig.position + offset;
            let dist = (expected - deformed.position).norm();
            assert!(dist < 0.1, "Translation error: {}", dist);
        }
    }

    #[test]
    fn test_cube_scaling_with_ffd() {
        let mesh = make_cube();

        // Scale by moving corner constraints outward
        let params = MorphParams::ffd_with_resolution(3, 3, 3)
            .unwrap()
            .with_constraint(Constraint::point(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(-0.1, -0.1, -0.1),
            ))
            .with_constraint(Constraint::point(
                Point3::new(1.0, 1.0, 1.0),
                Point3::new(1.1, 1.1, 1.1),
            ));

        let result = morph_mesh(&mesh, &params).unwrap();

        // Volume should increase
        assert!(result.volume_ratio > 1.0, "Volume should increase");
    }

    #[test]
    fn test_all_kernels_work() {
        let mesh = make_cube();

        // Create constraints
        let mut constraints = Vec::new();
        for v in &mesh.vertices {
            constraints.push(Constraint::point(v.position, v.position));
        }
        // Add one deformation
        constraints[0] = Constraint::point(
            mesh.vertices[0].position,
            mesh.vertices[0].position + Vector3::new(0.0, 0.0, 0.2),
        );

        // Test all kernels
        let kernels = [
            MorphParams::rbf(),
            MorphParams::rbf_gaussian(1.0),
            MorphParams::rbf_multiquadric(1.0),
            MorphParams::rbf_inverse_multiquadric(1.0),
        ];

        for base_params in kernels {
            let params = base_params.with_constraints(constraints.clone());
            let result = morph_mesh(&mesh, &params);
            assert!(result.is_ok(), "Kernel failed: {:?}", params.algorithm);
        }
    }

    #[test]
    fn test_quality_thresholds() {
        let mesh = make_cube();

        let mut constraints = Vec::new();
        for v in &mesh.vertices {
            constraints.push(Constraint::point(v.position, v.position));
        }

        let params = MorphParams::rbf().with_constraints(constraints);
        let result = morph_mesh(&mesh, &params).unwrap();

        // Identity morph should have no distortion
        assert!(!result.has_significant_distortion(0.01));
        assert!(!result.has_significant_volume_change(0.01));
    }

    #[test]
    fn test_bernstein_polynomial_properties() {
        // Partition of unity
        for n in 1..=5 {
            for t in [0.0, 0.1, 0.5, 0.9, 1.0] {
                let sum: f64 = (0..=n).map(|i| bernstein_basis(n, i, t)).sum();
                assert_relative_eq!(sum, 1.0, epsilon = 1e-10);
            }
        }

        // Endpoint values
        assert_relative_eq!(bernstein_basis(3, 0, 0.0), 1.0, epsilon = 1e-10);
        assert_relative_eq!(bernstein_basis(3, 3, 1.0), 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_binomial_coefficients() {
        assert_eq!(binomial(5, 0), 1);
        assert_eq!(binomial(5, 1), 5);
        assert_eq!(binomial(5, 2), 10);
        assert_eq!(binomial(5, 3), 10);
        assert_eq!(binomial(5, 4), 5);
        assert_eq!(binomial(5, 5), 1);
    }
}
