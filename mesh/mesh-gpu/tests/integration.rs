//! Integration tests for mesh-gpu.
//!
//! Tests marked with `#[ignore]` require a GPU and should be run with:
//! ```bash
//! cargo test -p mesh-gpu -- --ignored
//! ```

#![allow(clippy::unwrap_used, clippy::expect_used)]

use mesh_gpu::{
    GpuContext, GpuSdfParams, GpuSdfResult, TileConfig, compute_sdf_gpu, try_compute_sdf_gpu,
};
use mesh_types::{IndexedMesh, Vertex};

/// Create a unit cube mesh centered at origin.
fn create_unit_cube() -> IndexedMesh {
    let mut mesh = IndexedMesh::new();

    // Cube from (-1,-1,-1) to (1,1,1)
    let coords = [
        [-1.0, -1.0, -1.0],
        [1.0, -1.0, -1.0],
        [1.0, 1.0, -1.0],
        [-1.0, 1.0, -1.0],
        [-1.0, -1.0, 1.0],
        [1.0, -1.0, 1.0],
        [1.0, 1.0, 1.0],
        [-1.0, 1.0, 1.0],
    ];

    for c in &coords {
        mesh.vertices.push(Vertex::from_coords(c[0], c[1], c[2]));
    }

    // 12 triangles, 2 per face
    let faces: [[u32; 3]; 12] = [
        // Front (-Z)
        [0, 2, 1],
        [0, 3, 2],
        // Back (+Z)
        [4, 5, 6],
        [4, 6, 7],
        // Bottom (-Y)
        [0, 1, 5],
        [0, 5, 4],
        // Top (+Y)
        [2, 3, 7],
        [2, 7, 6],
        // Left (-X)
        [0, 4, 7],
        [0, 7, 3],
        // Right (+X)
        [1, 2, 6],
        [1, 6, 5],
    ];

    for f in &faces {
        mesh.faces.push(*f);
    }

    mesh
}

/// Create a simple triangle mesh.
fn create_single_triangle() -> IndexedMesh {
    let mut mesh = IndexedMesh::new();
    mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
    mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
    mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
    mesh.faces.push([0, 1, 2]);
    mesh
}

// ============================================================================
// Non-GPU tests (always run)
// ============================================================================

#[test]
fn test_gpu_context_availability_check() {
    // This should not panic regardless of GPU presence
    let _available = GpuContext::is_available();
}

#[test]
fn test_try_compute_returns_option() {
    let mesh = create_single_triangle();
    let params = GpuSdfParams::new([4, 4, 4], [-1.0, -1.0, -1.0], 0.5);

    // This should not panic - returns None if no GPU
    let result = try_compute_sdf_gpu(&mesh, &params);

    // We can't assert success/failure without knowing if GPU is present
    // But we can verify the result type is correct
    if let Some(r) = result {
        assert_eq!(r.dims, [4, 4, 4]);
        assert_eq!(r.values.len(), 64);
    }
}

#[test]
fn test_tile_config_calculations() {
    let config = TileConfig::new([100, 100, 100], 0);

    // 250x250x250 grid should need 3x3x3 = 27 tiles
    assert_eq!(config.tile_count([250, 250, 250]), [3, 3, 3]);
    assert_eq!(config.total_tiles([250, 250, 250]), 27);

    // Exact fit
    assert_eq!(config.tile_count([100, 200, 300]), [1, 2, 3]);
    assert_eq!(config.total_tiles([100, 200, 300]), 6);

    // Single tile
    assert_eq!(config.tile_count([50, 50, 50]), [1, 1, 1]);
    assert_eq!(config.total_tiles([50, 50, 50]), 1);
}

#[test]
fn test_sdf_result_get_zyx_ordering() {
    // Verify ZYX index ordering
    let values = vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0];
    let result = GpuSdfResult {
        values,
        dims: [2, 2, 2],
        compute_time_ms: 0.0,
    };

    // ZYX: idx = z + y * dims_z + x * dims_y * dims_z
    // For 2x2x2: idx = z + y*2 + x*4
    assert_eq!(result.get(0, 0, 0), Some(0.0)); // 0 + 0 + 0 = 0
    assert_eq!(result.get(0, 0, 1), Some(1.0)); // 1 + 0 + 0 = 1
    assert_eq!(result.get(0, 1, 0), Some(2.0)); // 0 + 2 + 0 = 2
    assert_eq!(result.get(0, 1, 1), Some(3.0)); // 1 + 2 + 0 = 3
    assert_eq!(result.get(1, 0, 0), Some(4.0)); // 0 + 0 + 4 = 4
    assert_eq!(result.get(1, 0, 1), Some(5.0)); // 1 + 0 + 4 = 5
    assert_eq!(result.get(1, 1, 0), Some(6.0)); // 0 + 2 + 4 = 6
    assert_eq!(result.get(1, 1, 1), Some(7.0)); // 1 + 2 + 4 = 7
}

#[test]
fn test_params_total_voxels() {
    let params = GpuSdfParams::new([10, 20, 30], [0.0, 0.0, 0.0], 0.1);
    assert_eq!(params.total_voxels(), 6000);
}

// ============================================================================
// GPU-required tests (run with --ignored)
// ============================================================================

#[test]
#[ignore = "Requires GPU"]
fn test_gpu_sdf_cube() {
    let mesh = create_unit_cube();
    let params = GpuSdfParams::new([32, 32, 32], [-2.0, -2.0, -2.0], 0.125);

    let result = compute_sdf_gpu(&mesh, &params).expect("GPU computation should succeed");

    // Verify dimensions
    assert_eq!(result.dims, [32, 32, 32]);
    assert_eq!(result.values.len(), 32 * 32 * 32);

    // Verify center is inside (negative distance)
    // Center voxel at (16, 16, 16) corresponds to world (0, 0, 0)
    let center_dist = result.get(16, 16, 16).expect("center exists");
    assert!(
        center_dist < 0.0,
        "Center should be inside (negative), got {center_dist}"
    );

    // Verify corner is outside (positive distance)
    let corner_dist = result.get(0, 0, 0).expect("corner exists");
    assert!(
        corner_dist > 0.0,
        "Corner should be outside (positive), got {corner_dist}"
    );
}

#[test]
#[ignore = "Requires GPU"]
fn test_gpu_sdf_empty_mesh() {
    let mesh = IndexedMesh::new();
    let params = GpuSdfParams::new([8, 8, 8], [-1.0, -1.0, -1.0], 0.25);

    // Empty mesh should produce large positive distances (all outside)
    let result = compute_sdf_gpu(&mesh, &params).expect("GPU computation should succeed");

    // All values should be large positive (or special value)
    // The shader should handle 0 triangles gracefully
    assert_eq!(result.values.len(), 512);
}

#[test]
#[ignore = "Requires GPU"]
fn test_gpu_context_info() {
    let ctx = GpuContext::try_get().expect("GPU should be available");

    // Verify adapter info is populated
    assert!(!ctx.adapter_info.name.is_empty());
    assert!(!ctx.adapter_info.backend.is_empty());

    // Verify limits are reasonable
    assert!(ctx.max_buffer_size() > 0);
    assert!(ctx.max_storage_buffer_size() > 0);
    assert!(ctx.max_invocations_per_workgroup() >= 256);

    let workgroup = ctx.max_workgroup_size();
    assert!(workgroup[0] >= 256);
}

#[test]
#[ignore = "Requires GPU"]
fn test_gpu_sdf_timing() {
    let mesh = create_unit_cube();
    let params = GpuSdfParams::new([64, 64, 64], [-2.0, -2.0, -2.0], 0.0625);

    let result = compute_sdf_gpu(&mesh, &params).expect("GPU computation should succeed");

    // Verify timing is recorded
    assert!(
        result.compute_time_ms > 0.0,
        "Computation time should be positive"
    );
    assert!(
        result.compute_time_ms < 10000.0,
        "Computation shouldn't take >10s"
    );

    println!("64³ grid SDF computed in {:.2}ms", result.compute_time_ms);
}
