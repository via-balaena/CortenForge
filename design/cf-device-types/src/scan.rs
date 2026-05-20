//! Scan-side resources — the loaded cleaned scan plus its companion
//! readouts.

use std::path::PathBuf;

use bevy::prelude::Resource;
use mesh_types::IndexedMesh;
use nalgebra::Point3;

/// Bevy resource carrying the loaded cleaned scan in physics-frame
/// meters. Mirror of cf-scan-prep's `ScanMesh` (same posture).
#[derive(Resource)]
pub struct ScanMesh(pub IndexedMesh);

/// The on-disk path the cleaned scan loaded from, kept for the
/// design-TOML's `scan_ref.cleaned_stl` provenance line. The design
/// panel + Save section read this to know what the design is anchored
/// against.
#[derive(Resource, Debug, Clone)]
pub struct ScanFilePath(pub PathBuf);

/// Whether the scan mesh entity is visible this frame. Toggled by
/// the "Show scan mesh" checkbox in the Scan Info panel. Useful when
/// inspecting the cavity mesh, which sits INSIDE the scan and is
/// occluded by the scan mesh when both are drawn.
#[derive(Resource, Debug, Clone, Copy, PartialEq)]
pub struct ScanMeshVisible(pub bool);

impl Default for ScanMeshVisible {
    fn default() -> Self {
        Self(true)
    }
}

/// Bevy resource carrying scan-info readouts surfaced in the Scan
/// Info panel. Computed once at startup; immutable post-construction.
#[derive(Resource, Debug, Clone)]
pub struct ScanInfo {
    /// Display label for the loaded scan — typically the cleaned-STL
    /// file name.
    pub file_label: String,
    /// Vertex count of the loaded scan mesh.
    pub vertex_count: usize,
    /// Face (triangle) count of the loaded scan mesh.
    pub face_count: usize,
    /// Raw AABB extents in millimeters (workshop convention).
    pub aabb_mm_extents: [f64; 3],
    /// Bbox diagonal in meters — shown in the Scan Info panel + the
    /// startup log as a quick size sanity check.
    pub bbox_diagonal_m: f64,
    /// Centerline polyline length in physics meters (sum of segment
    /// distances). Zero if the centerline is absent or empty. Shown
    /// in the Scan Info panel.
    pub centerline_arc_length_m: f64,
    /// Centerline point count (display-only). Zero if the centerline
    /// is absent.
    pub centerline_point_count: usize,
}

/// Bevy resource carrying the cf-scan-prep centerline polyline (in
/// post-bake physics-frame meters — matches the cleaned STL's
/// coordinate system). Empty when no `.prep.toml` is present or its
/// `[centerline]` block is absent.
#[derive(Resource, Default, Clone)]
pub struct Centerline {
    /// Polyline points in post-bake physics-frame meters.
    pub points_m: Vec<Point3<f64>>,
}
