//! API Regression Tests for Mesh Crate Ecosystem
//!
//! These tests serve as a regression suite to ensure the public API remains
//! stable and consistent across the mesh crate ecosystem. They are organized
//! in 5 tiers of increasing complexity:
//!
//! - Tier 1: Foundation (mesh-types, basic primitives)
//! - Tier 2: Core Operations (mesh-repair, mesh-decimate, mesh-io)
//! - Tier 3: Advanced Processing (mesh-boolean, mesh-slice, mesh-offset)
//! - Tier 4: Analysis & Validation (mesh-printability, mesh-sdf, mesh-region)
//! - Tier 5: Specialized (mesh-scan, mesh-morph, mesh-lattice)
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
    fn vertex_creation_and_access() {
        // Primary constructor
        let v = types::Vertex::from_coords(1.0, 2.0, 3.0);
        assert!((v.position.x - 1.0).abs() < f64::EPSILON);
        assert!((v.position.y - 2.0).abs() < f64::EPSILON);
        assert!((v.position.z - 3.0).abs() < f64::EPSILON);

        // From point
        let point = types::Point3::new(4.0, 5.0, 6.0);
        let v2 = types::Vertex::new(point);
        assert!((v2.position.x - 4.0).abs() < f64::EPSILON);
    }

    #[test]
    fn indexed_mesh_construction() {
        // Empty mesh
        let mesh = types::IndexedMesh::new();
        assert!(mesh.vertices.is_empty());
        assert!(mesh.faces.is_empty());

        // From parts
        let vertices = vec![
            types::Vertex::from_coords(0.0, 0.0, 0.0),
            types::Vertex::from_coords(1.0, 0.0, 0.0),
            types::Vertex::from_coords(0.0, 1.0, 0.0),
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
        let bounds = cube.bounds();

        // Unit cube spans 0,0,0 to 1,1,1
        assert!((bounds.min.x - 0.0).abs() < f64::EPSILON);
        assert!((bounds.max.x - 1.0).abs() < f64::EPSILON);
        assert!((bounds.min.y - 0.0).abs() < f64::EPSILON);
        assert!((bounds.max.y - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn mesh_topology_traits() {
        let cube = types::unit_cube();
        // IndexedMesh implements MeshTopology trait
        let count = <types::IndexedMesh as types::MeshTopology>::face_count(&cube);
        assert_eq!(count, 12);
    }
}

// =============================================================================
// TIER 2: Core Operations - Repair, Decimation, I/O
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
    fn decimate_params_presets() {
        use mesh::decimate::DecimateParams;

        // Target ratio
        let params = DecimateParams::with_target_ratio(0.5);
        assert!((params.target_ratio - 0.5).abs() < f64::EPSILON);

        // Aggressive preset
        let aggressive = DecimateParams::aggressive();
        assert!(aggressive.target_ratio < 0.5);

        // Builder pattern
        let params = DecimateParams::default()
            .with_preserve_boundary(true)
            .with_preserve_sharp_features(true);
        assert!(params.preserve_boundary);
    }

    #[test]
    fn decimate_mesh_operation() {
        use mesh::decimate::{DecimateParams, decimate_mesh};

        let cube = types::unit_cube();
        let result = decimate_mesh(&cube, &DecimateParams::default());

        // Result provides stats
        assert!(result.final_triangles <= cube.face_count());
        let display = format!("{}", result);
        assert!(display.contains("→")); // Shows before → after
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
// TIER 3: Advanced Processing - Boolean, Slicing, Offset
// =============================================================================

mod tier3_advanced_processing {
    use super::*;

    #[test]
    fn boolean_config_presets() {
        use mesh::boolean::{BooleanConfig, CleanupLevel, CoplanarStrategy};

        // Presets
        let default = BooleanConfig::default();
        let scans = BooleanConfig::for_scans();
        let cad = BooleanConfig::for_cad();
        let strict = BooleanConfig::strict();

        // Tolerance ordering
        assert!(scans.vertex_weld_tolerance > default.vertex_weld_tolerance);
        assert!(cad.vertex_weld_tolerance < default.vertex_weld_tolerance);
        assert!(strict.vertex_weld_tolerance < cad.vertex_weld_tolerance);

        // Builder pattern
        let config = BooleanConfig::default()
            .with_cleanup(CleanupLevel::Full)
            .with_coplanar_strategy(CoplanarStrategy::Exclude)
            .with_parallel(false);

        assert_eq!(config.cleanup, CleanupLevel::Full);
    }

    #[test]
    fn boolean_operations_return_result() {
        use mesh::boolean::{difference, intersection, union};

        let cube_a = types::unit_cube();

        // Create offset cube
        let mut cube_b = types::IndexedMesh::new();
        for v in &[
            types::Point3::new(5.0, 0.0, 0.0),
            types::Point3::new(6.0, 0.0, 0.0),
            types::Point3::new(6.0, 1.0, 0.0),
            types::Point3::new(5.0, 1.0, 0.0),
            types::Point3::new(5.0, 0.0, 1.0),
            types::Point3::new(6.0, 0.0, 1.0),
            types::Point3::new(6.0, 1.0, 1.0),
            types::Point3::new(5.0, 1.0, 1.0),
        ] {
            cube_b.vertices.push(types::Vertex::new(*v));
        }
        cube_b.faces = vec![
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [2, 3, 7],
            [2, 7, 6],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
        ];

        // All operations now return BooleanOperationResult consistently
        let union_result = union(&cube_a, &cube_b).unwrap();
        assert_eq!(union_result.mesh.faces.len(), 24); // Two disjoint cubes

        let diff_result = difference(&cube_a, &cube_b).unwrap();
        assert_eq!(diff_result.mesh.faces.len(), 12); // Just cube A

        let inter_result = intersection(&cube_a, &cube_b).unwrap();
        assert_eq!(inter_result.mesh.faces.len(), 0); // No overlap

        // Stats available on all operations
        assert!(!union_result.stats.meshes_intersected);
    }

    #[test]
    fn multi_boolean_operations() {
        use mesh::boolean::{BooleanConfig, multi_union};

        // Create 3 non-overlapping cubes
        let cubes: Vec<types::IndexedMesh> = (0..3)
            .map(|i| {
                let offset = i as f64 * 3.0;
                let mut mesh = types::IndexedMesh::new();
                for v in &[
                    types::Point3::new(offset, 0.0, 0.0),
                    types::Point3::new(offset + 1.0, 0.0, 0.0),
                    types::Point3::new(offset + 1.0, 1.0, 0.0),
                    types::Point3::new(offset, 1.0, 0.0),
                    types::Point3::new(offset, 0.0, 1.0),
                    types::Point3::new(offset + 1.0, 0.0, 1.0),
                    types::Point3::new(offset + 1.0, 1.0, 1.0),
                    types::Point3::new(offset, 1.0, 1.0),
                ] {
                    mesh.vertices.push(types::Vertex::new(*v));
                }
                mesh.faces = vec![
                    [0, 2, 1],
                    [0, 3, 2],
                    [4, 5, 6],
                    [4, 6, 7],
                    [0, 1, 5],
                    [0, 5, 4],
                    [2, 3, 7],
                    [2, 7, 6],
                    [0, 4, 7],
                    [0, 7, 3],
                    [1, 2, 6],
                    [1, 6, 5],
                ];
                mesh
            })
            .collect();

        let result = multi_union(&cubes, &BooleanConfig::default()).unwrap();
        assert_eq!(result.mesh.faces.len(), 36); // 3 cubes × 12 faces
        assert_eq!(result.stats.input_count, 3);
    }

    #[test]
    fn slice_mesh_operation() {
        use mesh::slice::{SliceParams, slice_mesh};

        let cube = types::unit_cube();
        let result = slice_mesh(&cube, &SliceParams::default());

        assert!(result.layer_count > 0);
        assert!(result.estimated_print_time >= 0.0);
    }

    #[test]
    fn offset_mesh_operation() {
        use mesh::offset::offset_mesh_default;

        let cube = types::unit_cube();
        let result = offset_mesh_default(&cube, 0.1);

        assert!(result.is_ok());
        let offset_result = result.unwrap();
        // Offset mesh should be larger
        let original_bounds = cube.bounds();
        let offset_bounds = offset_result.bounds();
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
    use super::*;

    #[test]
    fn point_cloud_operations() {
        use mesh::scan::PointCloud;

        let points = vec![
            types::Point3::new(0.0, 0.0, 0.0),
            types::Point3::new(1.0, 0.0, 0.0),
            types::Point3::new(0.0, 1.0, 0.0),
        ];

        let cloud = PointCloud::from_positions(&points);
        assert_eq!(cloud.len(), 3);
    }

    #[test]
    fn morph_params() {
        use mesh::morph::MorphParams;

        // MorphParams uses factory methods
        let params = MorphParams::rbf();
        assert!(params.constraints.is_empty());

        let params_gaussian = MorphParams::rbf_gaussian(1.0);
        assert!(params_gaussian.vertex_mask.is_none());
    }

    #[test]
    fn lattice_structure() {
        use mesh::lattice::LatticeParams;

        let params = LatticeParams::default().with_cell_size(5.0);

        assert!((params.cell_size - 5.0).abs() < f64::EPSILON);
    }

    #[test]
    fn remesh_operation() {
        use mesh::remesh::{RemeshParams, remesh};

        let cube = types::unit_cube();
        let params = RemeshParams::with_edge_length(0.5).with_iterations(3);
        let result = remesh(&cube, &params);

        assert!(result.is_ok());
        // Remeshing should produce stats about the operation
        let remeshed = result.unwrap();
        assert!(remeshed.final_faces > 0);
        assert!(remeshed.original_faces == cube.face_count());
    }

    #[test]
    fn registration_icp() {
        use mesh::registration::{IcpParams, icp_align};

        let source = types::unit_cube();
        let target = types::unit_cube(); // Same mesh for test

        let params = IcpParams::default();
        let result = icp_align(&source, &target, &params);

        assert!(result.is_ok());
        let alignment = result.unwrap();
        // Should converge quickly for identical meshes
        assert!(alignment.iterations < 50);
    }
}

// =============================================================================
// Error Handling Patterns
// =============================================================================

mod error_handling {
    use super::*;

    #[test]
    fn boolean_empty_mesh_error() {
        use mesh::boolean::union;

        let cube = types::unit_cube();
        let empty = types::IndexedMesh::new();

        // Operations on empty meshes should return errors
        let result = union(&cube, &empty);
        assert!(result.is_err());
    }

    #[test]
    fn printability_empty_mesh_error() {
        use mesh::printability::{PrinterConfig, validate_for_printing};

        let empty = types::IndexedMesh::new();
        let result = validate_for_printing(&empty, &PrinterConfig::fdm_default());
        assert!(result.is_err());
    }
}
