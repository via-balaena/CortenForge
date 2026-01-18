//! Template fitting for parametric mesh customization.
//!
//! This crate provides tools for fitting template meshes to scans and measurements,
//! enabling parametric customization of product designs such as:
//!
//! - **Shoe lasts** fitted to foot scans
//! - **Helmet liners** adapted to head measurements
//! - **Prosthetics** customized to body scans
//! - **Size variations** of parametric designs
//!
//! # Layer 0
//!
//! This is a Layer 0 crate with zero Bevy dependencies.
//!
//! # Quick Start
//!
//! ## Fitting to a Scan
//!
//! Use [`FitTemplate::fit_to_scan`] when you have a 3D scan to fit against:
//!
//! ```
//! use mesh_template::{FitTemplate, ControlRegion};
//! use mesh_types::{IndexedMesh, Vertex};
//! use nalgebra::Point3;
//!
//! // Create template mesh
//! let mut template_mesh = IndexedMesh::new();
//! template_mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! template_mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
//! template_mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
//! template_mesh.faces.push([0, 1, 2]);
//!
//! // Create scan mesh (slightly offset)
//! let mut scan_mesh = IndexedMesh::new();
//! scan_mesh.vertices.push(Vertex::from_coords(0.1, 0.1, 0.0));
//! scan_mesh.vertices.push(Vertex::from_coords(1.1, 0.1, 0.0));
//! scan_mesh.vertices.push(Vertex::from_coords(0.6, 1.1, 0.0));
//! scan_mesh.faces.push([0, 1, 2]);
//!
//! // Create template with control regions
//! let template = FitTemplate::new(template_mesh)
//!     .with_control_region(ControlRegion::point("tip", Point3::new(0.5, 1.0, 0.0)));
//!
//! // Fit to scan
//! let result = template.fit_to_scan(&scan_mesh).unwrap();
//! println!("Fit error: {:.4}", result.fit_error);
//! ```
//!
//! ## Fitting to Landmarks
//!
//! Use [`FitParams::with_landmark_target`] for precise control over specific points:
//!
//! ```
//! use mesh_template::{FitTemplate, FitParams, ControlRegion};
//! use mesh_types::{IndexedMesh, Vertex};
//! use nalgebra::Point3;
//!
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! let template = FitTemplate::new(mesh)
//!     .with_control_region(ControlRegion::point("apex", Point3::new(0.5, 1.0, 0.0)));
//!
//! let params = FitParams::new()
//!     .with_landmark_target("apex", Point3::new(0.5, 1.5, 0.0)); // Move apex up
//!
//! let result = template.fit(&params).unwrap();
//! ```
//!
//! ## Fitting to Measurements
//!
//! Use [`Measurement`] to define dimensional constraints:
//!
//! ```
//! use mesh_template::{FitTemplate, FitParams, ControlRegion, Measurement, MeasurementType};
//! use mesh_types::{IndexedMesh, Vertex};
//! use nalgebra::{Point3, Vector3};
//!
//! let mut mesh = IndexedMesh::new();
//! // Create a simple mesh...
//! for i in 0..8 {
//!     let angle = std::f64::consts::PI * 2.0 * (i as f64) / 8.0;
//!     mesh.vertices.push(Vertex::from_coords(angle.cos(), angle.sin(), 0.0));
//! }
//! // Add faces...
//! for i in 0..6 {
//!     mesh.faces.push([0, i + 1, i + 2]);
//! }
//!
//! let template = FitTemplate::new(mesh)
//!     .with_control_region(ControlRegion::measurement(
//!         "waist",
//!         MeasurementType::Circumference,
//!         Point3::origin(),
//!         Vector3::z(),
//!     ));
//!
//! let params = FitParams::new()
//!     .with_measurement("waist", Measurement::exact(100.0)); // Target 100mm circumference
//!
//! let result = template.fit(&params);
//! // Note: measurement fitting requires appropriate mesh geometry
//! ```
//!
//! # Three-Stage Fitting Pipeline
//!
//! The [`FitTemplate::fit`] method implements a sophisticated pipeline:
//!
//! 1. **Rigid Alignment** (optional) - ICP registration to align template to scan
//! 2. **Landmark Deformation** - RBF morphing to move control points to targets
//! 3. **Measurement Adjustment** - Scale regions to match dimensional constraints
//!
//! Each stage is tracked in [`FitResult::stages`] for diagnostic purposes.
//!
//! # Control Region Types
//!
//! | Type | Description | Use Case |
//! |------|-------------|----------|
//! | [`Point`](RegionDefinition::Point) | Single landmark | Anatomical landmarks |
//! | [`Vertices`](RegionDefinition::Vertices) | Set of vertex indices | Specific mesh vertices |
//! | [`Faces`](RegionDefinition::Faces) | Set of face indices | Mesh face groups |
//! | [`Bounds`](RegionDefinition::Bounds) | Axis-aligned box | Rectangular regions |
//! | [`Sphere`](RegionDefinition::Sphere) | Spherical region | Localized areas |
//! | [`Cylinder`](RegionDefinition::Cylinder) | Cylindrical region | Limbs, tubes |
//! | [`MeasurementPlane`](RegionDefinition::MeasurementPlane) | Plane for measurements | Cross-sections |
//!
//! # Architecture
//!
//! This crate builds on:
//! - `mesh-registration` for ICP alignment
//! - `mesh-morph` for RBF/FFD deformation
//! - `mesh-types` for core mesh data structures

mod control_region;
mod error;
mod fit;
mod measurement;
mod params;
mod result;
mod template;

pub use control_region::{ControlRegion, RegionDefinition};
pub use error::{TemplateError, TemplateResult};
pub use measurement::{Measurement, MeasurementType};
pub use params::FitParams;
pub use result::{FitResult, FitStage};
pub use template::FitTemplate;

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod integration_tests {
    use super::*;
    use approx::assert_relative_eq;
    use mesh_types::{IndexedMesh, Vertex};
    use nalgebra::{Point3, Vector3};

    fn make_triangle() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    fn make_square() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 2, 3]);
        mesh
    }

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
    fn test_create_template() {
        let mesh = make_triangle();
        let template = FitTemplate::new(mesh);
        assert_eq!(template.region_names().len(), 0);
    }

    #[test]
    fn test_add_control_regions() {
        let mesh = make_triangle();
        let template = FitTemplate::new(mesh)
            .with_control_region(ControlRegion::point("tip", Point3::new(0.5, 1.0, 0.0)))
            .with_control_region(ControlRegion::vertices("base", vec![0, 1]));

        assert_eq!(template.region_names().len(), 2);
        assert!(template.get_region("tip").is_some());
        assert!(template.get_region("base").is_some());
    }

    #[test]
    fn test_landmark_position() {
        let mesh = make_triangle();
        let template = FitTemplate::new(mesh)
            .with_control_region(ControlRegion::point("tip", Point3::new(0.5, 1.0, 0.0)));

        let pos = template.get_landmark_position("tip").unwrap();
        assert_relative_eq!(pos.x, 0.5, epsilon = 1e-10);
        assert_relative_eq!(pos.y, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_fit_with_landmarks() {
        let mesh = make_cube();

        // Create template with corner landmarks
        let template = FitTemplate::new(mesh)
            .with_control_region(ControlRegion::point("corner0", Point3::new(0.0, 0.0, 0.0)))
            .with_control_region(ControlRegion::point("corner1", Point3::new(1.0, 1.0, 1.0)));

        // Fit with landmark targets (identity - no movement)
        let params = FitParams::new()
            .with_landmark_target("corner0", Point3::new(0.0, 0.0, 0.0))
            .with_landmark_target("corner1", Point3::new(1.0, 1.0, 1.0));

        let result = template.fit(&params).unwrap();
        assert!(result.fit_error < 0.1, "Fit error: {}", result.fit_error);
    }

    #[test]
    fn test_fit_with_translation() {
        let mesh = make_square();
        let offset = Vector3::new(0.0, 0.0, 0.5);

        // Create template with all corners as landmarks
        let template = FitTemplate::new(mesh)
            .with_control_region(ControlRegion::point("c0", Point3::new(0.0, 0.0, 0.0)))
            .with_control_region(ControlRegion::point("c1", Point3::new(1.0, 0.0, 0.0)))
            .with_control_region(ControlRegion::point("c2", Point3::new(1.0, 1.0, 0.0)))
            .with_control_region(ControlRegion::point("c3", Point3::new(0.0, 1.0, 0.0)));

        // Target: translated corners
        let params = FitParams::new()
            .with_landmark_target("c0", Point3::new(0.0, 0.0, 0.0) + offset)
            .with_landmark_target("c1", Point3::new(1.0, 0.0, 0.0) + offset)
            .with_landmark_target("c2", Point3::new(1.0, 1.0, 0.0) + offset)
            .with_landmark_target("c3", Point3::new(0.0, 1.0, 0.0) + offset);

        let result = template.fit(&params).unwrap();

        // Vertices should be translated
        for v in &result.mesh.vertices {
            assert!(
                v.position.z > 0.4,
                "Vertex should be translated up: {:?}",
                v.position
            );
        }
    }

    #[test]
    fn test_region_weights() {
        let region = ControlRegion::point("test", Point3::origin()).with_weight(2.0);
        assert_relative_eq!(region.weight, 2.0, epsilon = 1e-10);
    }

    #[test]
    fn test_preserved_region() {
        let region = ControlRegion::point("test", Point3::origin()).preserved();
        assert!(region.preserve);
    }

    #[test]
    fn test_measurement_exact() {
        let m = Measurement::exact(100.0);
        assert_relative_eq!(m.value, 100.0, epsilon = 1e-10);
        assert_relative_eq!(m.tolerance, 1.0, epsilon = 1e-10);
        assert!(!m.is_minimum);
    }

    #[test]
    fn test_measurement_with_tolerance() {
        let m = Measurement::with_tolerance(100.0, 5.0);
        assert_relative_eq!(m.value, 100.0, epsilon = 1e-10);
        assert_relative_eq!(m.tolerance, 5.0, epsilon = 1e-10);
    }

    #[test]
    fn test_measurement_minimum() {
        let m = Measurement::minimum(50.0);
        assert!(m.is_minimum);
    }

    #[test]
    fn test_fit_result_acceptable() {
        // Create a mock result
        let mesh = make_triangle();
        let result = FitResult {
            mesh,
            fit_error: 0.5,
            stages: vec![],
            transform: mesh_registration::RigidTransform::identity(),
        };

        assert!(result.is_acceptable(1.0));
        assert!(!result.is_acceptable(0.1));
    }

    #[test]
    fn test_region_definition_bounds() {
        let mesh = make_cube();
        let region = ControlRegion::bounds(
            "front",
            Point3::new(-0.1, -0.1, -0.1),
            Point3::new(0.5, 1.1, 1.1),
        );

        let indices = region.get_vertex_indices(&mesh);

        // Should capture vertices with x <= 0.5
        assert!(indices.contains(&0)); // (0,0,0)
        assert!(indices.contains(&3)); // (0,1,0)
        assert!(indices.contains(&4)); // (0,0,1)
        assert!(indices.contains(&7)); // (0,1,1)
    }

    #[test]
    fn test_region_definition_sphere() {
        let mesh = make_cube();
        let region = ControlRegion::sphere("center", Point3::new(0.5, 0.5, 0.5), 0.9);

        let indices = region.get_vertex_indices(&mesh);

        // Sphere radius 0.9 from center (0.5,0.5,0.5) should include all corners
        // Distance from center to corner = sqrt(3)/2 ≈ 0.866
        assert_eq!(indices.len(), 8);
    }

    #[test]
    fn test_region_definition_cylinder() {
        let mesh = make_cube();
        // Cylinder along Z axis through center
        let region = ControlRegion::cylinder(
            "z_axis",
            Point3::new(0.5, 0.5, -1.0),
            Point3::new(0.5, 0.5, 2.0),
            0.8,
        );

        let indices = region.get_vertex_indices(&mesh);

        // Should capture all vertices (distance from z-axis through (0.5,0.5) is sqrt(0.5) ≈ 0.707)
        assert_eq!(indices.len(), 8);
    }

    #[test]
    fn test_empty_mesh_error() {
        let mesh = IndexedMesh::new();
        let template = FitTemplate::new(mesh);

        let result = template.fit(&FitParams::new());
        assert!(result.is_err());
    }

    #[test]
    fn test_fit_stages_tracking() {
        let mesh = make_cube();

        let template = FitTemplate::new(mesh.clone())
            .with_control_region(ControlRegion::point("corner", Point3::new(0.0, 0.0, 0.0)));

        let params = FitParams::new()
            .with_target_scan(mesh)
            .with_landmark_target("corner", Point3::new(0.0, 0.0, 0.1));

        let result = template.fit(&params).unwrap();

        // Should have at least rigid alignment stage
        assert!(
            !result.stages.is_empty(),
            "Should track fitting stages: {:?}",
            result.stages
        );
    }

    #[test]
    fn test_smoothness_parameter() {
        let params = FitParams::new().with_smoothness(0.5);
        assert_relative_eq!(params.smoothness, 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_registration_iterations() {
        let params = FitParams::new().with_registration_iterations(50);
        assert_eq!(params.registration_iterations, 50);
    }
}
