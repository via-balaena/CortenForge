//! API Regression Tests for Mesh Crate Ecosystem
//!
//! These tests serve as a regression suite to ensure the public API remains
//! stable and consistent across the mesh crate ecosystem. They are organized
//! in tiers of increasing complexity:
//!
//! - Tier 1: Foundation (mesh-types, basic primitives)
//! - Tier 2: Core Operations (mesh-repair, mesh-io)
//! - Tier 3: Advanced Processing (mesh-offset)
//! - Tier 4: Analysis & Validation (mesh-printability, mesh-sdf, mesh-repair)
//! - Tier 5: Specialized (mesh-lattice)
//!
//! If any of these tests fail after API changes, it indicates a breaking change
//! that needs documentation in CHANGELOG.md and a version bump.

// Allow test-specific patterns
#![allow(clippy::unwrap_used)]
#![allow(clippy::expect_used)]
#![allow(clippy::let_underscore_must_use)]
#![allow(clippy::uninlined_format_args)]
#![allow(clippy::cast_lossless)]

use mesh::{prelude::*, repair, types};

// =============================================================================
// TIER 1: Foundation - Basic Types and Primitives
// =============================================================================

mod tier1_foundation {
    use super::*;

    #[test]
    fn point3_creation_and_access() {
        // Direct construction
        let v: types::Point3<f64> = types::Point3::new(1.0, 2.0, 3.0);
        assert!((v.x - 1.0).abs() < f64::EPSILON);
        assert!((v.y - 2.0).abs() < f64::EPSILON);
        assert!((v.z - 3.0).abs() < f64::EPSILON);
    }

    #[test]
    fn indexed_mesh_construction() {
        // Empty mesh
        let mesh = types::IndexedMesh::new();
        assert!(mesh.vertices.is_empty());
        assert!(mesh.faces.is_empty());

        // From parts
        let vertices = vec![
            types::Point3::new(0.0, 0.0, 0.0),
            types::Point3::new(1.0, 0.0, 0.0),
            types::Point3::new(0.0, 1.0, 0.0),
        ];
        let faces = vec![[0, 1, 2]];
        let mesh = types::IndexedMesh::from_parts(vertices, faces);
        assert_eq!(mesh.vertex_count(), 3);
        assert_eq!(mesh.face_count(), 1);
    }

    #[test]
    fn primitive_unit_cube() {
        let cube = types::unit_cube();
        assert_eq!(cube.vertex_count(), 8);
        assert_eq!(cube.face_count(), 12); // 6 faces × 2 triangles
    }

    #[test]
    fn mesh_bounds_calculation() {
        let cube = types::unit_cube();
        let bounds = cube.aabb();

        // Unit cube spans 0,0,0 to 1,1,1
        assert!((bounds.min.x - 0.0).abs() < f64::EPSILON);
        assert!((bounds.max.x - 1.0).abs() < f64::EPSILON);
        assert!((bounds.min.y - 0.0).abs() < f64::EPSILON);
        assert!((bounds.max.y - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn mesh_inherent_methods() {
        let cube = types::unit_cube();
        // face_count and vertex_count are inherent methods on IndexedMesh
        assert_eq!(cube.face_count(), 12);
        assert_eq!(cube.vertex_count(), 8);
    }
}

// =============================================================================
// TIER 2: Core Operations - Repair, I/O
// =============================================================================

mod tier2_core_operations {
    use super::*;
    use mesh::repair::{RepairParams, RepairSummary};

    #[test]
    fn repair_params_builder_pattern() {
        // Default params
        let params = RepairParams::default();
        assert!(params.weld_epsilon > 0.0);

        // Builder pattern
        let params = RepairParams::default()
            .with_weld_epsilon(1e-5)
            .with_degenerate_area_threshold(1e-8)
            .with_degenerate_aspect_ratio(1e6)
            .with_degenerate_min_edge_length(1e-9)
            .with_remove_unreferenced(true);

        assert!((params.weld_epsilon - 1e-5).abs() < f64::EPSILON);
        assert!((params.degenerate_area_threshold - 1e-8).abs() < f64::EPSILON);
    }

    #[test]
    fn repair_summary_usage() {
        // RepairSummary API
        let summary = RepairSummary::default();
        assert!(!summary.had_changes());

        // Display trait
        let display = format!("{}", summary);
        assert!(display.contains("verts"));
    }

    #[test]
    fn repair_mesh_operations() {
        let mut mesh = types::unit_cube();
        let params = RepairParams::default();
        let result = repair::repair_mesh(&mut mesh, &params);
        assert!(result.final_vertices > 0);
    }

    #[test]
    fn mesh_validation() {
        let cube = types::unit_cube();
        let report = repair::validate_mesh(&cube);

        // Unit cube should be manifold and watertight
        assert!(report.is_manifold);
        assert!(report.is_watertight);
        assert_eq!(report.boundary_edge_count, 0);
    }

    #[test]
    fn io_format_detection() {
        use mesh::io::MeshFormat;

        // Format detection
        assert_eq!(MeshFormat::from_path("model.stl"), Some(MeshFormat::Stl));
        assert_eq!(MeshFormat::from_path("model.obj"), Some(MeshFormat::Obj));
        assert_eq!(MeshFormat::from_path("model.ply"), Some(MeshFormat::Ply));
        assert_eq!(
            MeshFormat::from_path("model.3mf"),
            Some(MeshFormat::ThreeMf)
        );

        // Extensions
        assert_eq!(MeshFormat::Stl.extension(), "stl");
        assert_eq!(MeshFormat::Obj.extension(), "obj");
    }
}

// =============================================================================
// TIER 3: Advanced Processing - Offset
// =============================================================================

mod tier3_advanced_processing {
    use super::*;

    #[test]
    fn offset_mesh_operation() {
        use mesh::offset::offset_mesh_default;

        let cube = types::unit_cube();
        let result = offset_mesh_default(&cube, 0.1);

        assert!(result.is_ok());
        let offset_result = result.unwrap();
        // Offset mesh should be larger
        let original_bounds = cube.aabb();
        let offset_bounds = offset_result.aabb();
        assert!(offset_bounds.max.x > original_bounds.max.x);
    }
}

// =============================================================================
// TIER 4: Analysis & Validation
// =============================================================================

mod tier4_analysis {
    use super::*;

    #[test]
    fn printability_validation() {
        use mesh::printability::{PrinterConfig, validate_for_printing};

        let cube = types::unit_cube();
        let config = PrinterConfig::fdm_default();
        let result = validate_for_printing(&cube, &config).unwrap();

        // Check validation result API
        let _ = result.is_printable();
        let _ = result.critical_count();
        let _ = result.warning_count();
        let _ = result.summary();
    }

    #[test]
    fn printer_config_presets() {
        use mesh::printability::PrinterConfig;

        let fdm = PrinterConfig::fdm_default();
        let _sla = PrinterConfig::sla_default();
        let sls = PrinterConfig::sls_default();
        let _mjf = PrinterConfig::mjf_default();

        // FDM requires supports, SLS doesn't
        assert!(fdm.technology.requires_supports());
        assert!(!sls.technology.requires_supports());

        // Builder pattern
        let config = PrinterConfig::fdm_default().with_build_volume(200.0, 200.0, 200.0);

        assert!((config.build_volume.0 - 200.0).abs() < f64::EPSILON);
    }

    #[test]
    fn sdf_distance_queries() {
        use mesh::sdf::SignedDistanceField;

        let cube = types::unit_cube();
        let sdf = SignedDistanceField::new(cube).unwrap();

        // Query distance from outside the cube
        let outside = types::Point3::new(2.0, 0.5, 0.5);
        let distance = sdf.distance(outside);
        assert!(distance >= 0.0);
    }

    #[test]
    fn connected_components() {
        use mesh::repair::components::{find_connected_components, keep_largest_component};

        let cube = types::unit_cube();
        let components = find_connected_components(&cube);

        // Single connected mesh
        assert_eq!(components.component_count, 1);

        // Keep largest
        let mut mesh = cube.clone();
        let _ = keep_largest_component(&mut mesh);
        assert_eq!(mesh.face_count(), cube.face_count());
    }

    #[test]
    fn hole_detection() {
        use mesh::repair::MeshAdjacency;
        use mesh::repair::holes::detect_holes;

        let cube = types::unit_cube();
        let adj = MeshAdjacency::build(&cube.faces);
        let holes = detect_holes(&cube, &adj);

        // Watertight cube has no holes
        assert!(holes.is_empty());
    }
}

// =============================================================================
// TIER 5: Specialized Operations
// =============================================================================

mod tier5_specialized {
    #[test]
    fn lattice_structure() {
        use mesh::lattice::LatticeParams;

        let params = LatticeParams::default().with_cell_size(5.0);

        assert!((params.cell_size - 5.0).abs() < f64::EPSILON);
    }
}

// =============================================================================
// Error Handling Patterns
// =============================================================================

mod error_handling {
    use super::*;

    #[test]
    fn printability_empty_mesh_error() {
        use mesh::printability::{PrinterConfig, validate_for_printing};

        let empty = types::IndexedMesh::new();
        let result = validate_for_printing(&empty, &PrinterConfig::fdm_default());
        assert!(result.is_err());
    }
}
