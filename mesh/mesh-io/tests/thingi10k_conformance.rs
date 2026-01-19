//! Conformance tests using real-world 3D printing models.
//!
//! These tests use sample meshes to verify mesh-io handles real-world
//! inputs correctly. The test fixtures simulate common characteristics found
//! in the `Thingi10K` dataset:
//! - Multiple disconnected components
//! - Various polygon counts (100s to 1000s of triangles)
//! - Both ASCII and binary STL formats
//!
//! To run: cargo test -p mesh-io thingi10k
//!
//! # Adding More Test Models
//!
//! To expand testing coverage, download additional models from:
//! - `Thingi10K` dataset: <https://github.com/Thingi10K/Thingi10K>
//! - Thingiverse: <https://www.thingiverse.com>
//!
//! Place STL files in: tests/fixtures/thingi10k/

#![allow(clippy::unwrap_used, clippy::expect_used)]

use mesh_io::{load_mesh, save_mesh};
use mesh_types::MeshTopology;
use std::path::PathBuf;
use tempfile::tempdir;

/// Get the path to test fixtures directory.
fn fixtures_dir() -> PathBuf {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    PathBuf::from(manifest_dir)
        .parent()
        .expect("parent dir")
        .parent()
        .expect("grandparent dir")
        .join("tests")
        .join("fixtures")
        .join("thingi10k")
}

/// Get all STL files in the fixtures directory.
fn get_stl_files() -> Vec<PathBuf> {
    let dir = fixtures_dir();
    if !dir.exists() {
        return Vec::new();
    }

    std::fs::read_dir(&dir)
        .into_iter()
        .flatten()
        .filter_map(|entry| {
            let path = entry.ok()?.path();
            if path
                .extension()
                .is_some_and(|e| e.eq_ignore_ascii_case("stl"))
            {
                Some(path)
            } else {
                None
            }
        })
        .collect()
}

/// Get all OBJ files in the fixtures directory.
fn get_obj_files() -> Vec<PathBuf> {
    let dir = fixtures_dir();
    if !dir.exists() {
        return Vec::new();
    }

    std::fs::read_dir(&dir)
        .into_iter()
        .flatten()
        .filter_map(|entry| {
            let path = entry.ok()?.path();
            if path
                .extension()
                .is_some_and(|e| e.eq_ignore_ascii_case("obj"))
            {
                Some(path)
            } else {
                None
            }
        })
        .collect()
}

// =============================================================================
// Loading Tests
// =============================================================================

#[test]
fn test_load_all_stl_fixtures() {
    let files = get_stl_files();
    if files.is_empty() {
        eprintln!(
            "Note: No STL test fixtures found in {:?}. Add STL files to enable conformance tests.",
            fixtures_dir()
        );
        return;
    }

    for file in &files {
        let result = load_mesh(file);
        assert!(
            result.is_ok(),
            "Failed to load {}: {:?}",
            file.display(),
            result.err()
        );

        let mesh = result.expect("mesh should load");
        assert!(
            mesh.vertex_count() > 0,
            "Mesh {} has no vertices",
            file.display()
        );
        assert!(
            mesh.face_count() > 0,
            "Mesh {} has no faces",
            file.display()
        );
    }
}

#[test]
fn test_load_all_obj_fixtures() {
    let files = get_obj_files();
    if files.is_empty() {
        eprintln!(
            "Note: No OBJ test fixtures found in {:?}. Add OBJ files to enable conformance tests.",
            fixtures_dir()
        );
        return;
    }

    for file in &files {
        let result = load_mesh(file);
        assert!(
            result.is_ok(),
            "Failed to load {}: {:?}",
            file.display(),
            result.err()
        );

        let mesh = result.expect("mesh should load");
        assert!(
            mesh.vertex_count() > 0,
            "Mesh {} has no vertices",
            file.display()
        );
        assert!(
            mesh.face_count() > 0,
            "Mesh {} has no faces",
            file.display()
        );
    }
}

// =============================================================================
// Round-Trip Tests
// =============================================================================

#[test]
fn test_stl_roundtrip() {
    let files = get_stl_files();
    if files.is_empty() {
        return;
    }

    let temp_dir = tempdir().expect("failed to create temp dir");

    for file in &files {
        let mesh = load_mesh(file).expect("failed to load original");
        let original_vertices = mesh.vertex_count();
        let original_faces = mesh.face_count();

        // Save to binary STL
        let out_path = temp_dir.path().join("roundtrip.stl");
        save_mesh(&mesh, &out_path).expect("failed to save");

        // Reload
        let reloaded = load_mesh(&out_path).expect("failed to reload");

        // Verify geometry is preserved
        // Note: STL doesn't preserve vertex sharing, so vertex count may differ
        // but face count should match
        assert_eq!(
            reloaded.face_count(),
            original_faces,
            "Face count changed for {}",
            file.display()
        );

        // Vertex count may increase (STL unshares vertices) but shouldn't decrease
        assert!(
            reloaded.vertex_count() >= original_vertices
                || reloaded.vertex_count() == original_faces * 3,
            "Unexpected vertex count for {}",
            file.display()
        );
    }
}

#[test]
fn test_obj_roundtrip() {
    let files = get_obj_files();
    if files.is_empty() {
        return;
    }

    let temp_dir = tempdir().expect("failed to create temp dir");

    for file in &files {
        let mesh = load_mesh(file).expect("failed to load original");
        let original_vertices = mesh.vertex_count();
        let original_faces = mesh.face_count();

        // Save to OBJ
        let out_path = temp_dir.path().join("roundtrip.obj");
        save_mesh(&mesh, &out_path).expect("failed to save");

        // Reload
        let reloaded = load_mesh(&out_path).expect("failed to reload");

        // OBJ should preserve both vertex and face count
        assert_eq!(
            reloaded.vertex_count(),
            original_vertices,
            "Vertex count changed for {}",
            file.display()
        );
        assert_eq!(
            reloaded.face_count(),
            original_faces,
            "Face count changed for {}",
            file.display()
        );
    }
}

// =============================================================================
// Cross-Format Tests
// =============================================================================

#[test]
fn test_stl_to_obj_conversion() {
    let files = get_stl_files();
    if files.is_empty() {
        return;
    }

    let temp_dir = tempdir().expect("failed to create temp dir");

    for file in &files {
        let mesh = load_mesh(file).expect("failed to load STL");
        let original_faces = mesh.face_count();

        // Convert STL to OBJ
        let obj_path = temp_dir.path().join("converted.obj");
        save_mesh(&mesh, &obj_path).expect("failed to save as OBJ");

        // Load as OBJ
        let converted = load_mesh(&obj_path).expect("failed to load OBJ");

        // Face count should be preserved
        assert_eq!(
            converted.face_count(),
            original_faces,
            "Face count changed during STL->OBJ conversion for {}",
            file.display()
        );
    }
}

#[test]
fn test_obj_to_stl_conversion() {
    let files = get_obj_files();
    if files.is_empty() {
        return;
    }

    let temp_dir = tempdir().expect("failed to create temp dir");

    for file in &files {
        let mesh = load_mesh(file).expect("failed to load OBJ");
        let original_faces = mesh.face_count();

        // Convert OBJ to STL
        let stl_path = temp_dir.path().join("converted.stl");
        save_mesh(&mesh, &stl_path).expect("failed to save as STL");

        // Load as STL
        let converted = load_mesh(&stl_path).expect("failed to load STL");

        // Face count should be preserved
        assert_eq!(
            converted.face_count(),
            original_faces,
            "Face count changed during OBJ->STL conversion for {}",
            file.display()
        );
    }
}
